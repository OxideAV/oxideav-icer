//! The contract entry points (`IMAGE_CRATE_API`): `probe`, `info`,
//! `decode*`, `encode*`. Thin, documented fronts over the decoder's
//! crate-internal `decode_image` / `walk_stream`, the cube pipeline in
//! [`crate::cube`] and the encoder's crate-internal `encode_image`.
//!
//! # The three wire forms
//!
//! An ICER file is one of:
//!
//! * a **bare 2-D segment stream** (IPN 42-155 §IV framing: 12-byte
//!   segment headers, each followed by its packets) — `Gray8`;
//! * the crate's **plane container** (`0x0000` sentinel + format tag)
//!   around one deep stream (`Gray16Le`) or three component streams
//!   (`Yuv444P`, `Gbrp8`);
//! * an **ICER-3D cube** (IPN 42-164; `00 00 C3 01` magic) — a stack
//!   of `bands` spatial planes, each a frame of [`decode_all`].
//!
//! Every entry point here accepts all three.

use std::io::{Read, Write};

use crate::cube::{
    decode_cube_frames, encode_icer3d, is_cube, parse_cube_header, CubeEncodeOptions, IcerCube,
};

use crate::decoder::{decode_image, walk_stream};
use crate::encoder::{encode_image, EncodeOptions};
use crate::error::{IcerError, Result};
use crate::header::SegmentHeader;
use crate::image::{
    ColorInfo, Frame, IcerImage, IcerPixelFormat, ImageInfo, RgbImage, RgbaImage, StreamKind,
};
use crate::options::DecodeOptions;
use crate::plane_container::is_container;

/// `true` when `bytes` plausibly begins an ICER stream. Allocation-free,
/// never panics, `false` on short input. A `true` result does not
/// guarantee the stream decodes; use [`info`] for that.
///
/// ICER has no file magic — IPN 42-155 §IV only mandates a non-zero
/// "self-synchronising" 16-bit prefix whose value each deployment
/// chooses — so the sniff is structural, in this order:
///
/// 1. the ICER-3D cube magic `00 00 C3 01` ([`crate::is_cube`]);
/// 2. the plane container: `00 00`, a known format tag (`0`..=`3`) and
///    a plane-count byte matching that tag (1 or 3);
/// 3. a bare 2-D stream: at least 12 bytes whose segment header
///    parses — non-zero sync prefix, a defined filter id (`0..=6`),
///    `1..=6` decomposition levels, non-zero width and height,
///    `1..=32` bit-planes, and for a §V.B transform-domain segment a
///    non-zero total with `index < total`.
pub fn probe(bytes: &[u8]) -> bool {
    if is_cube(bytes) {
        return true;
    }
    if is_container(bytes) {
        return matches!(
            bytes.get(2..4),
            Some([0, 1]) | Some([2, 1]) | Some([1, 3]) | Some([3, 3])
        );
    }
    plausible_segment_header(bytes)
}

/// The allocation-free field check behind [`probe`] — mirrors every
/// rejection in [`SegmentHeader::parse`] without building its error
/// strings.
fn plausible_segment_header(b: &[u8]) -> bool {
    if b.len() < SegmentHeader::ENCODED_BYTES {
        return false;
    }
    if b[0] == 0 && b[1] == 0 {
        return false;
    }
    let filter = (b[2] >> 4) & 0b0111;
    let levels = (b[2] >> 1) & 0b0111;
    if filter > 6 || !(1..=6).contains(&levels) {
        return false;
    }
    if (b[3] == 0 && b[4] == 0) || (b[5] == 0 && b[6] == 0) {
        return false;
    }
    let bit_planes = b[7] >> 2;
    if bit_planes == 0 || bit_planes > 32 {
        return false;
    }
    if b[7] & 1 == 1 {
        // §V.B transform-domain: (total_segments, segment_index).
        if b[10] == 0 || b[11] >= b[10] {
            return false;
        }
    }
    true
}

/// Header-only inspection: dimensions, the native [`crate::PixelFormat`]
/// [`decode`] would return, the number of images (`1`, or a cube's
/// band count), the documented colour convention, the significant
/// `bit_depth`, which wire form the stream uses and — for a 2-D stream
/// — every segment's framing record. Walks the framing under
/// [`DecodeOptions::default`]; decodes no pixels and allocates no
/// plane.
pub fn info(bytes: &[u8]) -> Result<ImageInfo> {
    info_with(bytes, &DecodeOptions::default())
}

/// [`info`] under explicit [`DecodeOptions`] (the same caps the decode
/// path would apply; a refused geometry is `Error::LimitExceeded`).
pub fn info_with(bytes: &[u8], opts: &DecodeOptions) -> Result<ImageInfo> {
    if is_cube(bytes) {
        let (hdr, _) = parse_cube_header(bytes, opts)?;
        let format = if hdr.bit_depth > 8 {
            IcerPixelFormat::Gray16Le
        } else {
            IcerPixelFormat::Gray8
        };
        return Ok(ImageInfo {
            width: hdr.width as u32,
            height: hdr.height as u32,
            format,
            frames: hdr.bands as u32,
            has_alpha: false,
            color: ColorInfo::default_for(format),
            has_icc: false,
            has_exif: false,
            has_xmp: false,
            bit_depth: hdr.bit_depth,
            kind: StreamKind::Cube,
            segments: Vec::new(),
            working_set_bytes: hdr.working_set,
        });
    }

    let layout = walk_stream(bytes, opts)?;
    Ok(ImageInfo {
        width: layout.width,
        height: layout.height,
        format: layout.format,
        frames: 1,
        has_alpha: false,
        color: ColorInfo::default_for(layout.format),
        has_icc: false,
        has_exif: false,
        has_xmp: false,
        bit_depth: layout.bit_depth,
        kind: layout.kind,
        segments: layout.segments,
        working_set_bytes: layout.working_set,
    })
}

/// Decode the image in `bytes` into its native layout with
/// `DecodeOptions::default()`. For an ICER-3D cube this is band 0 (see
/// [`decode_all`] for the rest).
pub fn decode(bytes: &[u8]) -> Result<IcerImage> {
    decode_with(bytes, &DecodeOptions::default())
}

/// [`decode`] under explicit [`DecodeOptions`]: every cap is checked
/// against the framing before any plane or coefficient buffer is
/// allocated (`Error::LimitExceeded`). `strict` has no effect (see
/// [`DecodeOptions`]).
pub fn decode_with(bytes: &[u8], opts: &DecodeOptions) -> Result<IcerImage> {
    if is_cube(bytes) {
        // The 3-D transform couples every band, so the whole cube is
        // decoded; band 0 is handed out, the rest dropped.
        let mut frames = decode_cube_frames(bytes, opts)?;
        return Ok(frames.swap_remove(0));
    }
    decode_image(bytes, opts)
}

/// One-call raw path: the image as tightly packed 8-bit RGB
/// ([`IcerImage::to_rgb8`]).
pub fn decode_rgb8(bytes: &[u8]) -> Result<RgbImage> {
    let img = decode(bytes)?;
    Ok(RgbImage::new(img.width, img.height, img.to_rgb8()))
}

/// One-call raw path: the image as tightly packed 8-bit RGBA, alpha
/// `255` ([`IcerImage::to_rgba8`]).
pub fn decode_rgba8(bytes: &[u8]) -> Result<RgbaImage> {
    let img = decode(bytes)?;
    Ok(RgbaImage::new(img.width, img.height, img.to_rgba8()))
}

/// Decode every image in `bytes` with `DecodeOptions::default()`: one
/// [`Frame`] per spectral band of an ICER-3D cube (`index` = band,
/// `Gray8` / `Gray16Le` with the cube's `bit_depth`), or a one-element
/// `Vec` for a 2-D stream. `delay` is always `None` (ICER has no
/// timing). Every frame has the same layout.
pub fn decode_all(bytes: &[u8]) -> Result<Vec<Frame>> {
    decode_all_with(bytes, &DecodeOptions::default())
}

/// [`decode_all`] under explicit [`DecodeOptions`].
pub fn decode_all_with(bytes: &[u8], opts: &DecodeOptions) -> Result<Vec<Frame>> {
    if is_cube(bytes) {
        // The cube decoder writes its band frames directly (no cube →
        // frame copy), which is what the working-set plan counts.
        let frames = decode_cube_frames(bytes, opts)?;
        return Ok(frames
            .into_iter()
            .enumerate()
            .map(|(b, img)| Frame::new(img, b as u32))
            .collect());
    }
    Ok(vec![Frame::new(decode_image(bytes, opts)?, 0)])
}

/// Read `r` to end and [`decode`] it. I/O failures surface as
/// `Error::Io`.
pub fn decode_from<R: Read>(mut r: R) -> Result<IcerImage> {
    let mut buf = Vec::new();
    r.read_to_end(&mut buf)?;
    decode(&buf)
}

/// Write `image` as a complete ICER stream under `opts`.
///
/// The layout picks the wire form: `Gray8` → a bare segment stream
/// (the historical form, byte-for-byte unchanged); `Gray16Le` → the
/// deep plane container carrying `bit_depth`; `Yuv444P` / `Gbrp8` →
/// three independent component streams behind the plane container.
/// Every layout is written as given — nothing is converted. `color`
/// and `metadata` cannot be carried and are ignored; a `Gray8` image
/// with `bit_depth < 8` is written as 8-bit (the bare form has no
/// depth field; it decodes back as `bit_depth` 8).
///
/// `Error::InvalidData` for an image whose planes do not match its
/// geometry; `Error::Unsupported` for option combinations the encoder
/// rejects (see [`EncodeOptions`]).
pub fn encode(image: &IcerImage, opts: &EncodeOptions) -> Result<Vec<u8>> {
    encode_image(image, opts)
}

/// One-call raw path: `3 × width × height` interleaved RGB bytes →
/// planar RGB (`Gbrp8`, three independent component streams behind the
/// plane container). Lossless under the default / compressed options:
/// `decode_rgb8(encode_rgb8(..))` returns the input exactly — no colour
/// matrix is involved. ICER's deployed colour scheme (IPN 42-155 §III)
/// codes each band independently, which is exactly what this does.
pub fn encode_rgb8(width: u32, height: u32, rgb: &[u8], opts: &EncodeOptions) -> Result<Vec<u8>> {
    let img = IcerImage::from_rgb8(width, height, rgb.to_vec())?;
    encode(&img, opts)
}

/// One-call raw path: `4 × width × height` interleaved RGBA bytes →
/// planar RGB (`Gbrp8`) exactly as [`encode_rgb8`]; **alpha is
/// dropped** — ICER has no alpha mechanism. (Callers who need the
/// alpha plane preserved can encode it as a separate `Gray8` image.)
pub fn encode_rgba8(width: u32, height: u32, rgba: &[u8], opts: &EncodeOptions) -> Result<Vec<u8>> {
    let img = IcerImage::from_rgba8(width, height, rgba.to_vec())?;
    encode(&img, opts)
}

/// Streaming variant of [`encode`]: encodes to memory, then writes.
pub fn encode_to<W: Write>(image: &IcerImage, opts: &EncodeOptions, mut w: W) -> Result<()> {
    let bytes = encode(image, opts)?;
    w.write_all(&bytes)?;
    Ok(())
}

/// Multi-image encode, the mirror of [`decode_all`]: a single frame is
/// [`encode`]d as a 2-D stream; two or more frames (gray layouts,
/// identical geometry and `bit_depth`) are stacked as the spectral
/// bands of an **ICER-3D cube** (IPN 42-164) and encoded with
/// [`crate::encode_icer3d`]. `opts` maps onto [`CubeEncodeOptions`]:
/// `filter`, `wavelet_levels`, `segment_count` (`1..=255`),
/// `byte_budget` → byte quota, `min_loss`, `interleaved_entropy`,
/// `transform_segments`; the cube pipeline has no uncompressed mode
/// and is lossless whenever no quota / `min_loss` truncates it.
/// `Error::InvalidData` on an empty slice or mismatched frames;
/// `Error::Unsupported` for colour frames or a segment count above
/// 255.
pub fn encode_all(frames: &[Frame], opts: &EncodeOptions) -> Result<Vec<u8>> {
    match frames {
        [] => Err(IcerError::invalid("encode_all: no frames")),
        [one] => encode(&one.image, opts),
        many => {
            let images: Vec<IcerImage> = many.iter().map(|f| f.image.clone()).collect();
            let cube = IcerCube::from_band_images(&images)?;
            encode_icer3d(&cube, &cube_options_from(opts)?)
        }
    }
}

/// Map the 2-D [`EncodeOptions`] onto the cube pipeline's knobs.
fn cube_options_from(opts: &EncodeOptions) -> Result<CubeEncodeOptions> {
    let segments = u8::try_from(opts.segment_count.max(1)).map_err(|_| {
        IcerError::unsupported(format!(
            "encode_all: cube segment count {} exceeds 255",
            opts.segment_count
        ))
    })?;
    let mut c = CubeEncodeOptions::default()
        .with_filter(opts.filter)
        .with_levels(opts.wavelet_levels)
        .with_segment_count(segments)
        .with_min_loss(opts.min_loss);
    if let Some(q) = opts.byte_budget {
        c = c.with_byte_quota(q);
    }
    if opts.interleaved_entropy {
        c = c.with_interleaved_entropy();
    }
    if opts.transform_segments {
        c = c.with_transform_domain_segments();
    }
    Ok(c)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::image::PixelFormat;

    fn ramp(w: u32, h: u32) -> IcerImage {
        let mut img = IcerImage::zeros(w, h, PixelFormat::Gray8);
        for y in 0..h {
            for x in 0..w {
                img.set_sample(0, x, y, ((x * 7 + y * 13) % 256) as u16);
            }
        }
        img
    }

    #[test]
    fn probe_rules() {
        assert!(!probe(&[]));
        assert!(!probe(&[0xAC; 11]));
        let bytes = encode(&ramp(8, 4), &EncodeOptions::default()).unwrap();
        assert!(probe(&bytes));
        assert!(probe(&bytes[..12]));
        // Zero sync prefix, reserved filter 7, zero width, 0 bit-planes.
        let mut bad = bytes[..12].to_vec();
        bad[0] = 0;
        bad[1] = 0;
        assert!(!probe(&bad));
        let mut bad = bytes[..12].to_vec();
        bad[2] |= 0x70;
        assert!(!probe(&bad));
        let mut bad = bytes[..12].to_vec();
        bad[3] = 0;
        bad[4] = 0;
        assert!(!probe(&bad));
        let mut bad = bytes[..12].to_vec();
        bad[7] = 0;
        assert!(!probe(&bad));
        // Containers: tag/plane-count agreement.
        assert!(probe(&[0, 0, 2, 1, 12]));
        assert!(probe(&[0, 0, 3, 3]));
        assert!(!probe(&[0, 0, 2, 3]));
        assert!(!probe(&[0, 0, 7, 1]));
        assert!(!probe(&[0, 0, 1]));
        assert!(probe(&[0, 0, 0xC3, 0x01]));
        assert!(!probe(&[0, 0, 0xC3]));
    }

    #[test]
    fn info_matches_decode_for_every_layout() {
        let gray = ramp(10, 6);
        let deep = IcerImage::zeros_deep(10, 6, 12).unwrap();
        let rgb = IcerImage::from_rgb8(10, 6, vec![9; 180]).unwrap();
        let yuv = IcerImage::zeros(10, 6, PixelFormat::Yuv444P);
        for (img, kind) in [
            (gray, StreamKind::Segments),
            (deep, StreamKind::PlaneContainer),
            (rgb, StreamKind::PlaneContainer),
            (yuv, StreamKind::PlaneContainer),
        ] {
            let bytes = encode(&img, &EncodeOptions::compressed().with_segment_count(2)).unwrap();
            assert!(probe(&bytes));
            let i = info(&bytes).unwrap();
            assert_eq!((i.width, i.height, i.frames), (10, 6, 1));
            assert_eq!(i.format, img.format);
            assert_eq!(i.bit_depth, img.bit_depth);
            assert_eq!(i.kind, kind);
            assert_eq!(i.segment_count(), 2 * img.format.plane_count());
            assert!(!i.has_alpha && !i.has_icc && !i.has_exif && !i.has_xmp);
            let back = decode(&bytes).unwrap();
            assert_eq!(back, img);
            let all = decode_all(&bytes).unwrap();
            assert_eq!(all.len(), 1);
            assert_eq!(all[0].index, 0);
            assert_eq!(all[0].image, img);
        }
    }

    #[test]
    fn rgb_raw_path_is_lossless_and_drops_alpha() {
        let rgb: Vec<u8> = (0..6 * 5 * 3).map(|i| (i * 11 % 256) as u8).collect();
        for opts in [EncodeOptions::default(), EncodeOptions::compressed()] {
            let bytes = encode_rgb8(6, 5, &rgb, &opts).unwrap();
            let back = decode_rgb8(&bytes).unwrap();
            assert_eq!((back.width, back.height), (6, 5));
            assert_eq!(back.as_bytes(), &rgb[..]);
            let rgba: Vec<u8> = rgb
                .chunks_exact(3)
                .flat_map(|p| [p[0], p[1], p[2], 3])
                .collect();
            let bytes = encode_rgba8(6, 5, &rgba, &opts).unwrap();
            let back = decode_rgba8(&bytes).unwrap();
            assert!(back.data.chunks_exact(4).all(|p| p[3] == 255));
            assert_eq!(
                back.into_raw()
                    .chunks_exact(4)
                    .flat_map(|p| [p[0], p[1], p[2]])
                    .collect::<Vec<_>>(),
                rgb
            );
        }
        assert!(encode_rgb8(6, 5, &rgb[..10], &EncodeOptions::default()).is_err());
    }

    #[test]
    fn cube_streams_are_frames() {
        let bands: Vec<IcerImage> = (0..4)
            .map(|b| {
                let mut img = IcerImage::zeros_deep(8, 6, 12).unwrap();
                for y in 0..6 {
                    for x in 0..8 {
                        img.set_sample(0, x, y, (300 + b * 50 + x * 17 + y * 5) as u16);
                    }
                }
                img
            })
            .collect();
        let frames: Vec<Frame> = bands
            .iter()
            .cloned()
            .enumerate()
            .map(|(i, img)| Frame::new(img, i as u32))
            .collect();
        let bytes = encode_all(&frames, &EncodeOptions::compressed()).unwrap();
        assert!(is_cube(&bytes));
        assert!(probe(&bytes));
        let i = info(&bytes).unwrap();
        assert_eq!((i.width, i.height, i.frames, i.bit_depth), (8, 6, 4, 12));
        assert_eq!(i.format, PixelFormat::Gray16Le);
        assert_eq!(i.kind, StreamKind::Cube);
        let first = decode(&bytes).unwrap();
        assert_eq!(first, bands[0]);
        let all = decode_all(&bytes).unwrap();
        assert_eq!(all.len(), 4);
        for (f, b) in all.iter().zip(&bands) {
            assert_eq!(&f.image, b);
            assert!(f.delay.is_none());
        }
        assert_eq!(all[3].index, 3);
        // Caps apply to the cube too.
        let tight = DecodeOptions::new().with_max_pixels(10u64);
        assert!(info_with(&bytes, &tight).unwrap_err().is_limit_exceeded());
        assert!(decode_with(&bytes, &tight).unwrap_err().is_limit_exceeded());
        // encode_all: one frame is a 2-D stream; mismatched frames fail.
        let single = encode_all(&frames[..1], &EncodeOptions::compressed()).unwrap();
        assert!(!is_cube(&single));
        assert_eq!(decode(&single).unwrap(), bands[0]);
        assert!(encode_all(&[], &EncodeOptions::default()).is_err());
        let mut odd = frames.clone();
        odd[1].image = IcerImage::zeros_deep(8, 6, 10).unwrap();
        assert!(encode_all(&odd, &EncodeOptions::default()).is_err());
    }

    #[test]
    fn decode_from_and_encode_to_round_trip() {
        let img = ramp(5, 5);
        let mut buf = Vec::new();
        encode_to(&img, &EncodeOptions::default(), &mut buf).unwrap();
        let back = decode_from(std::io::Cursor::new(&buf)).unwrap();
        assert_eq!(back, img);
        struct Broken;
        impl Read for Broken {
            fn read(&mut self, _: &mut [u8]) -> std::io::Result<usize> {
                Err(std::io::Error::other("nope"))
            }
        }
        assert!(decode_from(Broken).unwrap_err().is_io());
    }

    #[test]
    fn limits_fail_before_allocation_with_limit_exceeded() {
        let bytes = encode(&ramp(16, 16), &EncodeOptions::default()).unwrap();
        for opts in [
            DecodeOptions::new().with_max_width(8),
            DecodeOptions::new().with_max_height(8),
            DecodeOptions::new().with_max_pixels(100u64),
            DecodeOptions::new().with_max_pixels_per_segment(100u64),
            DecodeOptions::new().with_max_bytes(100u64),
        ] {
            assert!(decode_with(&bytes, &opts).unwrap_err().is_limit_exceeded());
            assert!(info_with(&bytes, &opts).unwrap_err().is_limit_exceeded());
        }
        assert!(decode_with(&bytes, &DecodeOptions::new().with_strict(true)).is_ok());
        assert!(decode_with(&bytes, &DecodeOptions::new().unlimited()).is_ok());
    }
}
