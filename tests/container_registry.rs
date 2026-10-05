//! The `icer` container through the `oxideav-core` registries: the probe
//! (cube magic, else the structural walk), the demuxer, the muxer, the
//! one-packet / one-frame-per-band cube mapping, and the
//! Layer-1-vs-registry byte-exact matrix (`IMAGE_CRATE_API`, Layer 2
//! acceptance for a container).
#![cfg(feature = "registry")]

use std::io::{Cursor, Seek, SeekFrom, Write};
use std::path::Path;
use std::sync::{Arc, Mutex};

use oxideav_core::{
    CodecId, CodecParameters, Error, Frame as CoreFrame, Packet, PixelFormat, ProbeData,
    RuntimeContext, StreamInfo, TimeBase, VideoFrame,
};
use oxideav_icer::container::{
    probe, PROBE_SCORE_CONTAINER_WALKED, PROBE_SCORE_SEGMENTS_WALKED, PROBE_SCORE_WITH_EXTENSION,
};
use oxideav_icer::registry::to_core_pixel_format;
use oxideav_icer::{EncodeOptions, Frame, IcerImage, IcerPixelFormat, StreamKind, CODEC_ID_STR};

const CONTAINER: &str = "icer";

/// A `Send + 'static` in-memory sink the muxer can own while the test
/// keeps a handle to read the bytes back.
#[derive(Clone, Default)]
struct SharedSink(Arc<Mutex<Cursor<Vec<u8>>>>);

impl SharedSink {
    fn bytes(&self) -> Vec<u8> {
        self.0.lock().unwrap().get_ref().clone()
    }
}

impl Write for SharedSink {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        self.0.lock().unwrap().write(buf)
    }
    fn flush(&mut self) -> std::io::Result<()> {
        self.0.lock().unwrap().flush()
    }
}

impl Seek for SharedSink {
    fn seek(&mut self, pos: SeekFrom) -> std::io::Result<u64> {
        self.0.lock().unwrap().seek(pos)
    }
}

fn ctx() -> RuntimeContext {
    let mut ctx = RuntimeContext::new();
    oxideav_icer::register(&mut ctx);
    ctx
}

/// Textured 8-bit planes (one, or three for the colour layouts).
fn image8(w: u32, h: u32, format: IcerPixelFormat, salt: u8) -> IcerImage {
    let mut img = IcerImage::zeros(w, h, format);
    for (p, plane) in img.planes.iter_mut().enumerate() {
        for y in 0..h as usize {
            for x in 0..w as usize {
                let v = ((x * 5 + y * 3 + p * 40) % 97) as u8;
                let v = if (x / 3 + y / 5) % 4 == 0 {
                    v.wrapping_add(40)
                } else {
                    v
                };
                plane.data[y * plane.stride + x] = v.wrapping_add(salt);
            }
        }
    }
    img
}

/// Textured deep gray at `bits` significant bits (LSB-aligned `u16`).
fn deep(w: u32, h: u32, bits: u8, salt: u16) -> IcerImage {
    let mut img = IcerImage::zeros_deep(w, h, bits).unwrap();
    let max = (1u32 << bits) - 1;
    for y in 0..h as usize {
        for x in 0..w as usize {
            let t = ((x * 13 + y * 29) % 257) as u32 * max / 256;
            let v = (t + salt as u32).min(max) as u16;
            let off = y * img.planes[0].stride + x * 2;
            img.planes[0].data[off..off + 2].copy_from_slice(&v.to_le_bytes());
        }
    }
    img
}

/// Every 2-D layout the crate writes, named, with the core label the
/// registry decoder gives its frames.
fn layouts() -> Vec<(&'static str, IcerImage, PixelFormat)> {
    let (w, h) = (24u32, 17u32);
    vec![
        (
            "Gray8 bare segment stream",
            image8(w, h, IcerPixelFormat::Gray8, 0),
            PixelFormat::Gray8,
        ),
        (
            "Gray16Le 12-bit",
            deep(w, h, 12, 100),
            PixelFormat::Gray12Le,
        ),
        ("Gray16Le 10-bit", deep(w, h, 10, 7), PixelFormat::Gray10Le),
        ("Gray16Le 11-bit", deep(w, h, 11, 3), PixelFormat::Gray16Le),
        (
            "Gray16Le 16-bit",
            deep(w, h, 16, 1000),
            PixelFormat::Gray16Le,
        ),
        (
            "Yuv444P",
            image8(w, h, IcerPixelFormat::Yuv444P, 5),
            PixelFormat::Yuv444P,
        ),
        (
            "Gbrp8",
            image8(w, h, IcerPixelFormat::Gbrp8, 9),
            PixelFormat::Gbrp8,
        ),
    ]
}

fn encode(img: &IcerImage) -> Vec<u8> {
    oxideav_icer::encode(img, &EncodeOptions::default()).unwrap()
}

/// What one registry pass produced: the stream, the packets, every
/// frame the decoder yielded, and the demuxer's metadata.
type Demuxed = (
    StreamInfo,
    Vec<Packet>,
    Vec<VideoFrame>,
    Vec<(String, String)>,
);

/// Open `bytes` through the registry and pump every packet through the
/// registry decoder, draining every frame each packet yields.
fn demux_decode(ctx: &RuntimeContext, bytes: &[u8], ext: Option<&str>) -> Demuxed {
    let name = ctx
        .containers
        .probe_input(&mut Cursor::new(bytes), ext)
        .expect("probe");
    assert_eq!(name, CONTAINER);
    let mut demuxer = ctx
        .containers
        .open_demuxer(&name, Box::new(Cursor::new(bytes.to_vec())), &ctx.codecs)
        .expect("open_demuxer");
    assert_eq!(demuxer.format_name(), CONTAINER);
    assert_eq!(demuxer.streams().len(), 1);
    let stream = demuxer.streams()[0].clone();
    assert_eq!(stream.params.codec_id, CodecId::new(CODEC_ID_STR));
    assert_eq!(stream.time_base, TimeBase::new(1, 1));
    let metadata = demuxer.metadata().to_vec();
    let mut dec = ctx.codecs.first_decoder(&stream.params).expect("decoder");
    let mut packets = Vec::new();
    let mut frames = Vec::new();
    loop {
        match demuxer.next_packet() {
            Ok(pkt) => {
                assert!(pkt.flags.keyframe);
                assert_eq!(pkt.stream_index, 0);
                dec.send_packet(&pkt).expect("send_packet");
                loop {
                    match dec.receive_frame() {
                        Ok(CoreFrame::Video(v)) => frames.push(v),
                        Ok(_) => panic!("expected a video frame"),
                        Err(Error::NeedMore) => break,
                        Err(e) => panic!("receive_frame: {e}"),
                    }
                }
                packets.push(pkt);
            }
            Err(Error::Eof) => break,
            Err(e) => panic!("next_packet: {e}"),
        }
    }
    assert!(matches!(demuxer.next_packet(), Err(Error::Eof)));
    (stream, packets, frames, metadata)
}

/// Encode `images` through the registry encoder and the muxer.
fn registry_mux(ctx: &RuntimeContext, format: PixelFormat, images: &[IcerImage]) -> Vec<u8> {
    let mut params = CodecParameters::video(CodecId::new(CODEC_ID_STR));
    params.width = Some(images[0].width);
    params.height = Some(images[0].height);
    params.pixel_format = Some(format);
    let mut enc = ctx.codecs.first_encoder(&params).expect("encoder");
    let stream = StreamInfo {
        index: 0,
        time_base: TimeBase::new(1, 1),
        duration: None,
        start_time: Some(0),
        params: enc.output_params().clone(),
    };
    let sink = SharedSink::default();
    {
        let mut muxer = ctx
            .containers
            .open_muxer(CONTAINER, Box::new(sink.clone()), &[stream])
            .expect("open_muxer");
        assert_eq!(muxer.format_name(), CONTAINER);
        muxer.write_header().unwrap();
        for (i, img) in images.iter().enumerate() {
            let mut vf: VideoFrame = img.clone().into();
            vf.pts = Some(i as i64);
            enc.send_frame(&CoreFrame::Video(vf)).unwrap();
            let mut pkt = enc.receive_packet().unwrap();
            pkt.pts = Some(i as i64);
            muxer.write_packet(&pkt).unwrap();
        }
        muxer.write_trailer().unwrap();
    }
    sink.bytes()
}

fn planes_of(vf: &VideoFrame) -> Vec<(usize, &[u8])> {
    vf.image_planes()
        .iter()
        .map(|p| (p.stride, p.data.as_slice()))
        .collect()
}

fn planes_of_image(img: &IcerImage) -> Vec<(usize, &[u8])> {
    img.planes
        .iter()
        .map(|p| (p.stride, p.data.as_slice()))
        .collect()
}

// ---- probe ----------------------------------------------------------------

#[test]
fn probe_names_icer_from_structure_and_with_extension() {
    let ctx = ctx();
    for (label, img, _) in layouts() {
        let bytes = encode(&img);
        for ext in [None, Some("icer")] {
            assert_eq!(
                ctx.containers
                    .probe_input(&mut Cursor::new(&bytes), ext)
                    .unwrap(),
                CONTAINER,
                "{label} / ext {ext:?}"
            );
        }
        let expected = match oxideav_icer::info(&bytes).unwrap().kind {
            StreamKind::Segments => PROBE_SCORE_SEGMENTS_WALKED,
            StreamKind::PlaneContainer => PROBE_SCORE_CONTAINER_WALKED,
            _ => unreachable!(),
        };
        assert_eq!(
            probe(&ProbeData {
                buf: &bytes,
                ext: None
            }),
            expected,
            "{label}"
        );
        assert_eq!(
            probe(&ProbeData {
                buf: &bytes,
                ext: Some("icer")
            }),
            expected.max(PROBE_SCORE_WITH_EXTENSION),
            "{label}"
        );
    }
    // A cube: the only real magic.
    let cube = oxideav_icer::encode_all(
        &[
            Frame::new(image8(8, 8, IcerPixelFormat::Gray8, 0), 0),
            Frame::new(image8(8, 8, IcerPixelFormat::Gray8, 50), 1),
        ],
        &EncodeOptions::default(),
    )
    .unwrap();
    assert_eq!(
        probe(&ProbeData {
            buf: &cube,
            ext: None
        }),
        oxideav_core::MAX_PROBE_SCORE
    );
    assert_eq!(
        ctx.containers
            .probe_input(&mut Cursor::new(&cube), None)
            .unwrap(),
        CONTAINER
    );
}

/// Foreign headers synthesised in-test (always run).
fn foreign_samples() -> Vec<(&'static str, Vec<u8>)> {
    let mut v: Vec<(&str, Vec<u8>)> = Vec::new();
    let mut farbfeld = b"farbfeld".to_vec();
    farbfeld.extend_from_slice(&2u32.to_be_bytes());
    farbfeld.extend_from_slice(&2u32.to_be_bytes());
    farbfeld.extend_from_slice(&[0x11; 32]);
    v.push(("ff", farbfeld));
    let mut png = vec![0x89, b'P', b'N', b'G', 0x0d, 0x0a, 0x1a, 0x0a, 0, 0, 0, 13];
    png.extend_from_slice(b"IHDR");
    png.extend_from_slice(&[0, 0, 0, 1, 0, 0, 0, 1, 8, 2, 0, 0, 0]);
    v.push(("png", png));
    v.push((
        "exr",
        vec![
            0x76, 0x2f, 0x31, 0x01, 0x02, 0, 0, 0, b'c', b'h', b'a', b'n', b'n', b'e', b'l', b's',
            0,
        ],
    ));
    v.push((
        "bmp",
        b"BM\x3a\0\0\0\0\0\0\0\x36\0\0\0\x28\0\0\0\x01\0\0\0\x01\0\0\0\x01\0\x18\0".to_vec(),
    ));
    v.push((
        "gif",
        b"GIF89a\x01\0\x01\0\x80\0\0\0\0\0\xff\xff\xff\x2c\0\0\0\0\x01\0\x01\0\0\x02\x02\x44\x01\0\x3b"
            .to_vec(),
    ));
    v.push((
        "qoi",
        b"qoif\0\0\0\x01\0\0\0\x01\x04\0\xfe\0\0\0\0\0\0\0\0\0\0\0\x01".to_vec(),
    ));
    v.push(("ppm", b"P6\n1 1\n255\n\0\0\0".to_vec()));
    v.push(("tif", b"II*\0\x08\0\0\0\0\0\0\0\0\0\0\0".to_vec()));
    v.push((
        "jpg",
        vec![
            0xff, 0xd8, 0xff, 0xe0, 0, 0x10, b'J', b'F', b'I', b'F', 0, 1, 1, 0, 0, 1, 0, 1, 0, 0,
            0xff, 0xd9,
        ],
    ));
    v.push((
        "hdr",
        b"#?RADIANCE\nFORMAT=32-bit_rle_rgbe\n\n-Y 1 +X 1\n\0\0\0\0".to_vec(),
    ));
    v.push(("bin", vec![0u8; 1024]));
    v.push(("bin", (0..=255u8).cycle().take(2048).collect()));
    // Bytes that pass the 12-byte segment-header plausibility but chain
    // to nothing: the structural walk must refuse them.
    let mut plausible = vec![
        0x12, 0x34, 0x22, 0x00, 0x40, 0x00, 0x30, 0x20, 0x00, 0x00, 0x05, 0x02,
    ];
    plausible.extend_from_slice(&[0xA5; 64]);
    v.push(("bin", plausible));
    v
}

#[test]
fn probe_rejects_foreign_files() {
    let ctx = ctx();
    for (ext, bytes) in foreign_samples() {
        assert_eq!(
            probe(&ProbeData {
                buf: &bytes,
                ext: Some(ext)
            }),
            0,
            "{ext} sample must not score as ICER"
        );
        assert!(ctx
            .containers
            .probe_input(&mut Cursor::new(&bytes), Some(ext))
            .is_err());
        assert!(ctx
            .containers
            .probe_input(&mut Cursor::new(&bytes), None)
            .is_err());
    }
}

/// Sweep the sibling crates' fixtures (the umbrella checkout, or
/// `OXIDEAV_CRATES_DIR`) with the first 256 KiB of every image file and
/// its real extension: none may score as ICER. Skips with a note when no
/// sibling is present (CI checks out this crate alone).
#[test]
fn probe_rejects_sibling_crate_fixtures() {
    let root = std::env::var_os("OXIDEAV_CRATES_DIR")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|| Path::new(env!("CARGO_MANIFEST_DIR")).join(".."));
    let Ok(entries) = std::fs::read_dir(&root) else {
        eprintln!("no sibling crates next to this one — sweep skipped");
        return;
    };
    let mut files = Vec::new();
    for crate_dir in entries.flatten() {
        let name = crate_dir.file_name();
        let name = name.to_string_lossy();
        if !name.starts_with("oxideav-") || name == "oxideav-icer" {
            continue;
        }
        let tests = crate_dir.path().join("tests");
        if tests.is_dir() {
            collect_files(&tests, &mut files);
        }
    }
    if files.is_empty() {
        eprintln!("no sibling fixtures found — sweep skipped");
        return;
    }
    let image_exts = [
        "png", "bmp", "gif", "tif", "tiff", "exr", "ff", "qoi", "tga", "pcx", "dcx", "pbm", "pgm",
        "ppm", "pam", "hdr", "pic", "dds", "ico", "cur", "jp2", "j2k", "jpc", "jxl", "jxs", "webp",
        "avif", "heic", "heif", "svg", "iff", "lbm", "ilbm", "jpg", "jpeg", "wbmp", "pict", "pct",
    ];
    let mut checked = 0usize;
    for path in files {
        let ext = path
            .extension()
            .and_then(|e| e.to_str())
            .map(|e| e.to_ascii_lowercase());
        let Some(ext) = ext else { continue };
        if !image_exts.contains(&ext.as_str()) {
            continue;
        }
        let Ok(bytes) = std::fs::read(&path) else {
            continue;
        };
        let head = &bytes[..bytes.len().min(256 * 1024)];
        for hint in [None, Some(ext.as_str())] {
            let score = probe(&ProbeData {
                buf: head,
                ext: hint,
            });
            assert_eq!(score, 0, "{} scored {score} as ICER", path.display());
        }
        checked += 1;
    }
    eprintln!("sibling sweep: {checked} image fixtures, none probed as ICER");
    assert!(checked > 0);
}

fn collect_files(dir: &Path, out: &mut Vec<std::path::PathBuf>) {
    let Ok(rd) = std::fs::read_dir(dir) else {
        return;
    };
    for e in rd.flatten() {
        let p = e.path();
        if p.is_dir() {
            collect_files(&p, out);
        } else {
            out.push(p);
        }
    }
}

// ---- demux: Layer 1 vs registry -------------------------------------------

#[test]
fn demux_matches_layer1_across_every_2d_layout() {
    let ctx = ctx();
    let mut pinned = 0;
    for (label, img, core_fmt) in layouts() {
        let bytes = encode(&img);
        let info = oxideav_icer::info(&bytes).unwrap();
        let l1 = oxideav_icer::decode(&bytes).unwrap();
        assert_eq!(
            l1.planes, img.planes,
            "{label}: the default encode is lossless"
        );
        let (stream, packets, frames, metadata) = demux_decode(&ctx, &bytes, None);
        assert_eq!(stream.params.width, Some(info.width), "{label}");
        assert_eq!(stream.params.height, Some(info.height), "{label}");
        assert_eq!(stream.params.pixel_format, Some(core_fmt), "{label}");
        assert_eq!(
            stream.params.pixel_format,
            Some(to_core_pixel_format(info.format, info.bit_depth)),
            "{label}: the stream label is the decoder's frame label"
        );
        assert_eq!(
            stream.params.color_signal,
            CodecParameters::video(CodecId::new(CODEC_ID_STR)).color_signal,
            "{label}: ICER carries no colour information — nothing stamped"
        );
        assert!(metadata.contains(&("bit_depth".to_string(), info.bit_depth.to_string())));
        assert_eq!(packets.len(), 1, "{label}");
        assert_eq!(packets[0].pts, Some(0));
        assert_eq!(
            packets[0].data, bytes,
            "{label}: the whole file is the packet"
        );
        assert_eq!(frames.len(), 1, "{label}");
        assert_eq!(frames[0].pts, Some(0));
        assert_eq!(
            frames[0].image_planes().len(),
            img.format.plane_count(),
            "{label}"
        );
        assert_eq!(
            planes_of(&frames[0]),
            planes_of_image(&l1),
            "{label}: registry planes == Layer 1 planes"
        );
        assert_eq!(frames[0].color_signal(), None, "{label}");
        pinned += 1;
    }
    assert_eq!(pinned, 7);
}

#[test]
fn cube_is_one_packet_and_one_frame_per_band_matching_decode_all() {
    let ctx = ctx();
    let cubes: Vec<(&str, Vec<Frame>)> = vec![
        (
            "8-bit, 4 bands",
            (0..4)
                .map(|b| Frame::new(image8(16, 12, IcerPixelFormat::Gray8, b as u8 * 30), b))
                .collect(),
        ),
        (
            "12-bit, 3 bands",
            (0..3)
                .map(|b| Frame::new(deep(16, 12, 12, b as u16 * 500), b))
                .collect(),
        ),
    ];
    for (label, frames) in cubes {
        let bytes = oxideav_icer::encode_all(&frames, &EncodeOptions::default()).unwrap();
        let info = oxideav_icer::info(&bytes).unwrap();
        assert_eq!(info.kind, StreamKind::Cube, "{label}");
        assert_eq!(info.frames as usize, frames.len(), "{label}");
        let l1 = oxideav_icer::decode_all(&bytes).unwrap();
        assert_eq!(l1.len(), frames.len(), "{label}");
        for (f, orig) in l1.iter().zip(&frames) {
            assert_eq!(
                f.image.planes, orig.image.planes,
                "{label}: the default cube encode is lossless"
            );
        }
        let (stream, packets, out, metadata) = demux_decode(&ctx, &bytes, Some("icer"));
        assert_eq!(stream.params.width, Some(info.width), "{label}");
        assert_eq!(stream.params.height, Some(info.height), "{label}");
        assert_eq!(
            stream.params.pixel_format,
            Some(to_core_pixel_format(info.format, info.bit_depth)),
            "{label}"
        );
        assert!(metadata.contains(&("bands".to_string(), frames.len().to_string())));
        assert_eq!(
            packets.len(),
            1,
            "{label}: a cube is one joint transform — one packet"
        );
        assert_eq!(packets[0].data, bytes);
        assert_eq!(out.len(), frames.len(), "{label}: one frame per band");
        for (b, vf) in out.iter().enumerate() {
            assert_eq!(vf.pts, Some(b as i64), "{label}: pts = band index");
            assert_eq!(l1[b].index, b as u32);
            assert_eq!(
                planes_of(vf),
                planes_of_image(&l1[b].image),
                "{label}: band {b} planes == decode_all"
            );
        }
    }
}

// ---- mux ------------------------------------------------------------------

#[test]
fn mux_single_packet_round_trips_every_layout() {
    let ctx = ctx();
    for (label, img, core_fmt) in layouts() {
        let out = registry_mux(&ctx, core_fmt, std::slice::from_ref(&img));
        let back = oxideav_icer::decode(&out).unwrap();
        assert_eq!(back.planes, img.planes, "{label}: lossless through mux");
        assert_eq!(back.format, img.format, "{label}");
        assert_eq!(back.bit_depth, img.bit_depth, "{label}");
        // demux(mux(frame)) == frame through the registry too.
        let (_, _, frames, _) = demux_decode(&ctx, &out, None);
        assert_eq!(frames.len(), 1);
        assert_eq!(planes_of(&frames[0]), planes_of_image(&img), "{label}");
    }
}

#[test]
fn mux_several_packets_writes_a_cube_decode_all_reads_back() {
    let ctx = ctx();
    for (label, images, core_fmt) in [
        (
            "8-bit",
            (0..3u8)
                .map(|b| image8(12, 10, IcerPixelFormat::Gray8, b * 60))
                .collect::<Vec<_>>(),
            PixelFormat::Gray8,
        ),
        (
            "12-bit",
            (0..3u16).map(|b| deep(12, 10, 12, b * 700)).collect(),
            PixelFormat::Gray12Le,
        ),
    ] {
        let out = registry_mux(&ctx, core_fmt, &images);
        let info = oxideav_icer::info(&out).unwrap();
        assert_eq!(info.kind, StreamKind::Cube, "{label}");
        assert_eq!(info.frames, 3, "{label}");
        let l1 = oxideav_icer::decode_all(&out).unwrap();
        assert_eq!(l1.len(), 3);
        for (b, f) in l1.iter().enumerate() {
            assert_eq!(f.index, b as u32);
            assert_eq!(
                f.image.planes, images[b].planes,
                "{label}: band {b} lossless"
            );
            assert_eq!(f.image.bit_depth, images[b].bit_depth);
        }
        // The muxed cube is what `encode_all` writes for the same frames.
        let frames: Vec<Frame> = images
            .iter()
            .enumerate()
            .map(|(i, im)| Frame::new(im.clone(), i as u32))
            .collect();
        assert_eq!(
            out,
            oxideav_icer::encode_all(&frames, &EncodeOptions::default()).unwrap(),
            "{label}: muxer == encode_all"
        );
        // Registry round trip: demux(mux(frames)) == frames.
        let (_, packets, vfs, _) = demux_decode(&ctx, &out, None);
        assert_eq!(packets.len(), 1);
        assert_eq!(vfs.len(), 3);
        for (b, vf) in vfs.iter().enumerate() {
            assert_eq!(vf.pts, Some(b as i64));
            assert_eq!(
                planes_of(vf),
                planes_of_image(&images[b]),
                "{label}: band {b}"
            );
        }
    }
}

#[test]
fn mux_rejects_bad_streams_packets_and_mismatched_bands() {
    let ctx = ctx();
    let video = StreamInfo {
        index: 0,
        time_base: TimeBase::new(1, 1),
        duration: None,
        start_time: Some(0),
        params: CodecParameters::video(CodecId::new(CODEC_ID_STR)),
    };
    let sink = || Box::new(Cursor::new(Vec::<u8>::new()));
    assert!(ctx.containers.open_muxer(CONTAINER, sink(), &[]).is_err());
    assert!(ctx
        .containers
        .open_muxer(CONTAINER, sink(), &[video.clone(), video.clone()])
        .is_err());
    let audio = StreamInfo {
        params: CodecParameters::audio(CodecId::new("pcm")),
        ..video.clone()
    };
    assert!(ctx
        .containers
        .open_muxer(CONTAINER, sink(), &[audio])
        .is_err());

    let mut muxer = ctx
        .containers
        .open_muxer(CONTAINER, sink(), std::slice::from_ref(&video))
        .unwrap();
    muxer.write_header().unwrap();
    assert!(muxer
        .write_packet(&Packet::new(0, TimeBase::new(1, 1), Vec::new()))
        .is_err());
    assert!(muxer
        .write_packet(&Packet::new(
            0,
            TimeBase::new(1, 1),
            b"\0\0\0\0\0\0\0\0\0\0\0\0\0".to_vec()
        ))
        .is_err());
    assert!(muxer.write_trailer().is_err(), "no packet, no file");

    // Bands of different geometry cannot form a cube.
    let a = encode(&image8(8, 8, IcerPixelFormat::Gray8, 0));
    let b = encode(&image8(9, 8, IcerPixelFormat::Gray8, 0));
    let mut muxer = ctx
        .containers
        .open_muxer(CONTAINER, sink(), std::slice::from_ref(&video))
        .unwrap();
    muxer.write_header().unwrap();
    muxer
        .write_packet(&Packet::new(0, TimeBase::new(1, 1), a))
        .unwrap();
    muxer
        .write_packet(&Packet::new(0, TimeBase::new(1, 1), b))
        .unwrap();
    assert!(muxer.write_trailer().is_err());
}

// ---- registration ---------------------------------------------------------

#[test]
fn register_installs_codec_and_container() {
    let ctx = ctx();
    assert!(ctx
        .codecs
        .decoder_ids()
        .any(|c| *c == CodecId::new(CODEC_ID_STR)));
    assert!(ctx.containers.demuxer_names().any(|n| n == CONTAINER));
    assert!(ctx.containers.muxer_names().any(|n| n == CONTAINER));
    assert_eq!(
        ctx.containers.container_for_extension("icer"),
        Some(CONTAINER)
    );
    assert_eq!(
        ctx.containers.container_for_extension("ICER"),
        Some(CONTAINER)
    );
    let mut via_entry = RuntimeContext::new();
    oxideav_icer::__oxideav_entry(&mut via_entry);
    assert!(via_entry.containers.demuxer_names().any(|n| n == CONTAINER));
    assert!(via_entry.containers.muxer_names().any(|n| n == CONTAINER));
}

// ---- hostile input --------------------------------------------------------

#[test]
fn hostile_inputs_never_panic() {
    let ctx = ctx();
    let open = |bytes: &[u8]| {
        ctx.containers
            .open_demuxer(
                CONTAINER,
                Box::new(Cursor::new(bytes.to_vec())),
                &ctx.codecs,
            )
            .map(|mut d| while d.next_packet().is_ok() {})
    };
    assert!(open(&[]).is_err());
    assert!(open(&[0u8; 11]).is_err());
    let bare = encode(&image8(16, 12, IcerPixelFormat::Gray8, 0));
    let container = encode(&deep(16, 12, 12, 0));
    let cube = oxideav_icer::encode_all(
        &[
            Frame::new(image8(8, 8, IcerPixelFormat::Gray8, 0), 0),
            Frame::new(image8(8, 8, IcerPixelFormat::Gray8, 50), 1),
        ],
        &EncodeOptions::default(),
    )
    .unwrap();
    for src in [&bare, &container, &cube] {
        for cut in (0..src.len()).step_by(3) {
            let _ = open(&src[..cut]);
            let _ = probe(&ProbeData {
                buf: &src[..cut],
                ext: Some("icer"),
            });
        }
        let mut mutated = src.clone();
        for i in 0..src.len().min(400) {
            for v in [0x00, 0xff, 0x7f] {
                let keep = mutated[i];
                mutated[i] = v;
                let _ = open(&mutated);
                let _ = probe(&ProbeData {
                    buf: &mutated,
                    ext: None,
                });
                mutated[i] = keep;
            }
        }
    }
    // Absurd cube geometry claimed in the header: refused by the caps
    // before allocation.
    let mut huge = cube.clone();
    huge[4..6].copy_from_slice(&0xFFFFu16.to_be_bytes());
    huge[6..8].copy_from_slice(&0xFFFFu16.to_be_bytes());
    huge[8..10].copy_from_slice(&0xFFFFu16.to_be_bytes());
    assert!(open(&huge).is_err());
}
