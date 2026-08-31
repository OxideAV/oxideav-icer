//! Wire-form + decode digest pins across the encode mode matrix.
//!
//! Every case encodes a fixed deterministic fixture under one mode,
//! digests the produced byte stream (FNV-1a 64), decodes it with the
//! crate's own decoder, and digests the reconstructed samples too. The
//! pinned pairs freeze both directions bit-for-bit so a performance
//! change anywhere in the pipeline is provably output-identical: any
//! optimisation that moves a single wire byte or reconstructed sample
//! fails this test.
//!
//! To regenerate after an *intentional* wire change: empty `EXPECTED`,
//! run the test, paste the printed table back.

use oxideav_icer::{
    encode_icer, encode_icer3d, parse_icer, parse_icer3d, ChannelReliability, CubeEncodeOptions,
    EncodeOptions, IcerCube, IcerImage, IcerPixelFormat, WaveletFilter,
};

fn fnv1a(bytes: &[u8]) -> u64 {
    let mut h = 0xcbf2_9ce4_8422_2325u64;
    for &b in bytes {
        h ^= b as u64;
        h = h.wrapping_mul(0x0000_0100_0000_01b3);
    }
    h
}

fn textured(w: u32, h: u32) -> IcerImage {
    let mut img = IcerImage::zeros(w, h, IcerPixelFormat::Gray8);
    let stride = img.planes[0].stride;
    for y in 0..h as usize {
        for x in 0..w as usize {
            img.planes[0].data[y * stride + x] = ((x * 97 + y * 57 + (x ^ y) * 31) & 0xFF) as u8;
        }
    }
    img
}

fn deep_textured(w: u32, h: u32, bits: u8) -> IcerImage {
    let mut img = IcerImage::zeros(w, h, IcerPixelFormat::GrayDeep { bits });
    let mask = (1u32 << bits) - 1;
    for y in 0..h {
        for x in 0..w {
            img.set_sample(0, x, y, ((x * 97 + y * 57 + (x ^ y) * 31) & mask) as u16);
        }
    }
    img
}

fn colour_textured(w: u32, h: u32) -> IcerImage {
    let mut img = IcerImage::zeros(w, h, IcerPixelFormat::Yuv444P);
    for (p, plane) in img.planes.iter_mut().enumerate() {
        let stride = plane.stride;
        for y in 0..h as usize {
            for x in 0..w as usize {
                plane.data[y * stride + x] = ((x * 31 + y * 17 + p * 101) & 0xFF) as u8;
            }
        }
    }
    img
}

fn cube_fixture(w: u32, h: u32, bands: u32) -> IcerCube {
    let mut cube = IcerCube::zeros(w, h, bands, 12);
    let (wu, hu) = (w as usize, h as usize);
    for b in 0..bands as usize {
        let dc = 800 + ((b * 137) % 1200) as i32;
        for y in 0..hu {
            for x in 0..wu {
                let t = ((x * 13 + y * 29 + b * 7) % 257) as i32 - 128;
                cube.samples[b * wu * hu + y * wu + x] = (dc + t).clamp(0, 4095) as u16;
            }
        }
    }
    cube
}

fn decoded_digest(bytes: &[u8]) -> u64 {
    let img = parse_icer(bytes).expect("decode");
    let mut acc: Vec<u8> = Vec::new();
    for plane in &img.planes {
        acc.extend_from_slice(&plane.data);
    }
    fnv1a(&acc)
}

fn cube_decoded_digest(bytes: &[u8]) -> u64 {
    let cube = parse_icer3d(bytes).expect("cube decode");
    let mut acc: Vec<u8> = Vec::with_capacity(cube.samples.len() * 2);
    for &s in &cube.samples {
        acc.extend_from_slice(&s.to_le_bytes());
    }
    fnv1a(&acc)
}

/// (name, encode-digest, decode-digest) for every pinned mode.
fn cases() -> Vec<(&'static str, u64, u64)> {
    let img = textured(96, 80);
    let mut out = Vec::new();
    let mut push2d = |name: &'static str, opts: &EncodeOptions| {
        let bytes = encode_icer(&img, opts).expect(name);
        out.push((name, fnv1a(&bytes), decoded_digest(&bytes)));
    };

    push2d("gray8_uncompressed", &EncodeOptions::default());
    push2d("gray8_filter_q", &EncodeOptions::compressed());
    push2d("gray8_filter_a_levels4", &{
        let mut o = EncodeOptions::compressed();
        o.filter = WaveletFilter::FilterA;
        o.wavelet_levels = 4;
        o
    });
    push2d("gray8_strip4", &{
        let mut o = EncodeOptions::compressed();
        o.segment_count = 4;
        o
    });
    push2d("gray8_transform4", &{
        let mut o = EncodeOptions::compressed().with_transform_domain_segments();
        o.segment_count = 4;
        o
    });
    push2d(
        "gray8_priority_interleaved",
        &EncodeOptions::compressed().with_priority_interleaving(),
    );
    push2d(
        "gray8_interleaved_entropy",
        &EncodeOptions::compressed().with_interleaved_entropy(),
    );
    push2d(
        "gray8_minloss3",
        &EncodeOptions::compressed().with_min_loss(3),
    );
    push2d("gray8_budget900_strip4", &{
        let mut o = EncodeOptions::compressed().with_byte_budget(900);
        o.segment_count = 4;
        o
    });
    push2d(
        "gray8_auto_segments_typical",
        &EncodeOptions::compressed().with_auto_segments(ChannelReliability::Typical),
    );

    let deep = deep_textured(96, 80, 12);
    let bytes = encode_icer(&deep, &EncodeOptions::compressed()).expect("deep12");
    out.push(("deep12_filter_q", fnv1a(&bytes), decoded_digest(&bytes)));

    let colour = colour_textured(64, 48);
    let bytes = encode_icer(&colour, &EncodeOptions::compressed()).expect("yuv444");
    out.push(("yuv444_filter_q", fnv1a(&bytes), decoded_digest(&bytes)));

    let cube = cube_fixture(32, 32, 8);
    let bytes = encode_icer3d(&cube, &CubeEncodeOptions::default()).expect("cube");
    out.push((
        "cube3d_row_strip",
        fnv1a(&bytes),
        cube_decoded_digest(&bytes),
    ));
    let bytes = encode_icer3d(
        &cube,
        &CubeEncodeOptions::default()
            .with_transform_domain_segments()
            .with_segment_count(4),
    )
    .expect("cube td");
    out.push((
        "cube3d_transform4",
        fnv1a(&bytes),
        cube_decoded_digest(&bytes),
    ));
    let bytes = encode_icer3d(&cube, &CubeEncodeOptions::default().with_byte_quota(2000))
        .expect("cube quota");
    out.push((
        "cube3d_quota2000",
        fnv1a(&bytes),
        cube_decoded_digest(&bytes),
    ));

    out
}

const EXPECTED: &[(&str, u64, u64)] = &[
    ("gray8_uncompressed", 0xdc03c224646e58b1, 0x86aeb6be62c8e5a5),
    ("gray8_filter_q", 0x817708138e953b12, 0x86aeb6be62c8e5a5),
    (
        "gray8_filter_a_levels4",
        0x95341f28c975d5c1,
        0x86aeb6be62c8e5a5,
    ),
    ("gray8_strip4", 0x8ba7401e306d1ae0, 0x86aeb6be62c8e5a5),
    ("gray8_transform4", 0x8e0d5e4be9c295ab, 0x86aeb6be62c8e5a5),
    (
        "gray8_priority_interleaved",
        0x9b288d408dab57bc,
        0x86aeb6be62c8e5a5,
    ),
    (
        "gray8_interleaved_entropy",
        0xeef8868b2809a577,
        0x86aeb6be62c8e5a5,
    ),
    ("gray8_minloss3", 0x938911640141308c, 0x53e43c7eee85685f),
    (
        "gray8_budget900_strip4",
        0xc22781b49ef4b66f,
        0xa1d86664c8a83e22,
    ),
    (
        "gray8_auto_segments_typical",
        0x8ba7401e306d1ae0,
        0x86aeb6be62c8e5a5,
    ),
    ("deep12_filter_q", 0x08c4726f0db65e5b, 0x84cd3d6de6c26c89),
    ("yuv444_filter_q", 0x9bc7482451dbab47, 0x7ce056d667a7c025),
    ("cube3d_row_strip", 0x98bb6eaed60a66ef, 0xd1125ff3de5a8adc),
    ("cube3d_transform4", 0x95a272f8e7e668ea, 0xd1125ff3de5a8adc),
    ("cube3d_quota2000", 0x7fc38c466845ded7, 0xe16a95c64a85fa01),
];

#[test]
fn wire_and_decode_digests_pinned() {
    let got = cases();
    if EXPECTED.is_empty() {
        for (name, e, d) in &got {
            println!("    (\"{name}\", 0x{e:016x}, 0x{d:016x}),");
        }
        panic!("EXPECTED is empty — paste the printed table into EXPECTED");
    }
    assert_eq!(got.len(), EXPECTED.len(), "case count drifted");
    for ((gn, ge, gd), (en, ee, ed)) in got.iter().zip(EXPECTED.iter()) {
        assert_eq!(gn, en, "case order drifted");
        assert_eq!(ge, ee, "{gn}: wire digest moved");
        assert_eq!(gd, ed, "{gn}: decode digest moved");
    }
}
