//! End-to-end round-trip cover for the round-1 uncompressed
//! (IPN 42-155 §III.D) encode → decode pipeline.

use oxideav_icer::{decode, encode, info, EncodeOptions, IcerImage, IcerPixelFormat};

fn ramp_image(w: u32, h: u32) -> IcerImage {
    let mut img = IcerImage::zeros(w, h, IcerPixelFormat::Gray8);
    let plane = &mut img.planes[0];
    for y in 0..h as usize {
        for x in 0..w as usize {
            plane.data[y * plane.stride + x] = ((x + y) & 0xFF) as u8;
        }
    }
    img
}

#[test]
fn uncompressed_roundtrip_small() {
    let original = ramp_image(16, 12);
    let bytes = encode(&original, &EncodeOptions::default()).unwrap();
    let decoded = decode(&bytes).unwrap();
    assert_eq!(decoded.width, original.width);
    assert_eq!(decoded.height, original.height);
    assert_eq!(decoded.format, original.format);
    assert_eq!(decoded.planes[0].data, original.planes[0].data);
}

#[test]
fn uncompressed_roundtrip_larger() {
    let original = ramp_image(64, 48);
    let bytes = encode(&original, &EncodeOptions::default()).unwrap();
    let decoded = decode(&bytes).unwrap();
    assert_eq!(decoded.planes[0].data, original.planes[0].data);
}

#[test]
fn metadata_walk_reports_one_segment() {
    let original = ramp_image(32, 32);
    let bytes = encode(&original, &EncodeOptions::default()).unwrap();
    let meta = info(&bytes).unwrap();
    assert_eq!(meta.segments.len(), 1);
    let seg = &meta.segments[0];
    assert_eq!(seg.header.width, 32);
    assert_eq!(seg.header.height, 32);
    assert!(seg.header.uncompressed);
    assert_eq!(seg.packet_count, 1);
    assert_eq!(seg.offset, 0);
    assert_eq!(seg.byte_length, bytes.len());
}

#[test]
fn empty_input_errors() {
    assert!(decode(&[]).is_err());
    assert!(info(&[0u8; 4]).is_err());
}
