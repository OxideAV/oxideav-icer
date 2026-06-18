//! Colour (YUV 4:4:4) round-trip cover.
//!
//! IPN 42-155 §III describes ICER as a single-component coder whose
//! deployed colour scheme runs one independent ICER instance per colour
//! component. This crate encodes a [`IcerPixelFormat::Yuv444P`] image as
//! three independent single-plane ICER bitstreams behind the
//! [`oxideav_icer::plane_container`] header; these tests pin the
//! end-to-end behaviour and confirm the Gray8 wire form is unchanged.

use oxideav_icer::{
    encode_icer, is_container, parse_icer, parse_icer_lenient, parse_icer_metadata, EncodeOptions,
    IcerImage, IcerPixelFormat,
};

/// Three planes with distinct, non-trivial content so the test would
/// catch a plane mix-up (e.g. all planes decoding to plane 0's data).
fn colour_image(w: u32, h: u32) -> IcerImage {
    let mut img = IcerImage::zeros(w, h, IcerPixelFormat::Yuv444P);
    for y in 0..h as usize {
        for x in 0..w as usize {
            let stride = img.planes[0].stride;
            img.planes[0].data[y * stride + x] = ((x + y) & 0xFF) as u8; // luma ramp
            img.planes[1].data[y * stride + x] = ((x * 3) & 0xFF) as u8; // Cb: horizontal
            img.planes[2].data[y * stride + x] = ((y * 5) & 0xFF) as u8; // Cr: vertical
        }
    }
    img
}

#[test]
fn colour_filter_q_roundtrip_is_bit_exact() {
    // Filter Q (reversible integer 5/3) must round-trip every plane
    // bit-exactly through the colour container.
    let original = colour_image(16, 16);
    let opts = EncodeOptions::compressed(); // default filter Q
    let bytes = encode_icer(&original, &opts).unwrap();

    assert!(is_container(&bytes), "colour stream must use the container");

    let decoded = parse_icer(&bytes).unwrap();
    assert_eq!(decoded.pixel_format, IcerPixelFormat::Yuv444P);
    assert_eq!(decoded.width, 16);
    assert_eq!(decoded.height, 16);
    assert_eq!(decoded.planes.len(), 3);
    for i in 0..3 {
        assert_eq!(
            decoded.planes[i].data, original.planes[i].data,
            "plane {i} must round-trip bit-exactly under filter Q"
        );
    }
}

#[test]
fn colour_planes_are_independent_not_aliased() {
    // The three planes carry different content; a decode that wrongly
    // aliased them (e.g. copying plane 0 into all slots) would fail.
    let original = colour_image(16, 16);
    let bytes = encode_icer(&original, &EncodeOptions::compressed()).unwrap();
    let decoded = parse_icer(&bytes).unwrap();
    assert_ne!(decoded.planes[0].data, decoded.planes[1].data);
    assert_ne!(decoded.planes[0].data, decoded.planes[2].data);
    assert_ne!(decoded.planes[1].data, decoded.planes[2].data);
}

#[test]
fn colour_uncompressed_roundtrip_is_bit_exact() {
    let original = colour_image(12, 9);
    let opts = EncodeOptions::default(); // uncompressed §III.D path
    let bytes = encode_icer(&original, &opts).unwrap();
    let decoded = parse_icer(&bytes).unwrap();
    assert_eq!(decoded.pixel_format, IcerPixelFormat::Yuv444P);
    for i in 0..3 {
        assert_eq!(decoded.planes[i].data, original.planes[i].data);
    }
}

#[test]
fn gray8_stream_is_not_a_container() {
    // Backward-compatibility guard: a single-plane Gray8 stream must
    // NOT be framed as a container (no sentinel prefix), so every
    // previously-encoded Gray8 stream still decodes byte-for-byte.
    let mut gray = IcerImage::zeros(16, 16, IcerPixelFormat::Gray8);
    for (i, b) in gray.planes[0].data.iter_mut().enumerate() {
        *b = (i & 0xFF) as u8;
    }
    let bytes = encode_icer(&gray, &EncodeOptions::compressed()).unwrap();
    assert!(!is_container(&bytes), "Gray8 stream must stay un-framed");
    let decoded = parse_icer(&bytes).unwrap();
    assert_eq!(decoded.pixel_format, IcerPixelFormat::Gray8);
    assert_eq!(decoded.planes[0].data, gray.planes[0].data);
}

#[test]
fn colour_metadata_walks_every_plane() {
    // Multi-segment colour: each plane is split into 2 segments, so the
    // metadata walker should surface 3 planes * 2 segments = 6 segments.
    let original = colour_image(16, 32);
    let opts = EncodeOptions {
        segment_count: 2,
        ..EncodeOptions::compressed()
    };
    let bytes = encode_icer(&original, &opts).unwrap();
    let meta = parse_icer_metadata(&bytes).unwrap();
    assert_eq!(
        meta.segments.len(),
        6,
        "3 planes * 2 segments = 6 segment headers"
    );
    // Offsets must be strictly increasing and absolute into the buffer.
    for w in meta.segments.windows(2) {
        assert!(w[0].offset < w[1].offset);
    }
}

#[test]
fn colour_lenient_decode_reports_luma_presence() {
    // A fully-present colour stream decodes leniently with zero missing.
    let original = colour_image(16, 32);
    let opts = EncodeOptions {
        segment_count: 2,
        ..EncodeOptions::compressed()
    };
    let bytes = encode_icer(&original, &opts).unwrap();
    let lenient = parse_icer_lenient(&bytes).unwrap();
    assert_eq!(lenient.missing_count, 0);
    assert_eq!(lenient.image.pixel_format, IcerPixelFormat::Yuv444P);
    for i in 0..3 {
        assert_eq!(lenient.image.planes[i].data, original.planes[i].data);
    }
}
