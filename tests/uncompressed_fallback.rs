//! Per-segment automatic uncompressed-fallback (IPN 42-155 §III.D
//! "Performance with Difficult Imagery").
//!
//! `EncodeOptions::with_uncompressed_fallback()` lets the encoder
//! emit the uncompressed candidate when the compressed candidate
//! comes out larger -- the spaceflight behaviour the paper describes
//! for difficult tiles (random-noise / high-frequency content where
//! the entropy stage expands the payload). The decoder reads the
//! per-segment `uncompressed` flag and reconstructs from whichever
//! path was emitted, so the choice is transparent on decode.

use oxideav_icer::{
    encode_icer, parse_icer, parse_icer_metadata, EncodeOptions, IcerImage, IcerPixelFormat,
    WaveletFilter,
};

fn fill<F>(w: u32, h: u32, mut f: F) -> IcerImage
where
    F: FnMut(usize, usize) -> u8,
{
    let mut img = IcerImage::zeros(w, h, IcerPixelFormat::Gray8);
    let plane = &mut img.planes[0];
    let stride = plane.stride;
    for y in 0..h as usize {
        for x in 0..w as usize {
            plane.data[y * stride + x] = f(x, y);
        }
    }
    img
}

/// Noisy LCG-driven content -- compressed encoder cannot exploit any
/// structure here, so the entropy stage expands relative to raw
/// pixels. Cheap and deterministic.
fn noise_image(w: u32, h: u32, seed: u32) -> IcerImage {
    let mut state = seed.wrapping_mul(2_654_435_761).wrapping_add(1);
    fill(w, h, |_, _| {
        state = state.wrapping_mul(1_103_515_245).wrapping_add(12_345);
        (state >> 16) as u8
    })
}

/// Smooth diagonal ramp -- the entropy stage has plenty of structure
/// to exploit, so compressed is comfortably smaller than raw.
fn ramp_image(w: u32, h: u32) -> IcerImage {
    fill(w, h, |x, y| ((x + y) & 0xFF) as u8)
}

#[test]
fn fallback_picks_uncompressed_when_compressed_is_larger() {
    // 16x16 noise tile: compressed-path output is bigger than the
    // 256-byte raw pixels + 16-byte framing overhead. Fallback should
    // win.
    let img = noise_image(16, 16, 0xA5A5);

    let opts_no_fallback = EncodeOptions::compressed().with_byte_budget(u64::MAX); // disable any truncation effect
    let bytes_compressed = encode_icer(&img, &opts_no_fallback).unwrap();

    let opts_with_fallback = EncodeOptions::compressed()
        .with_byte_budget(u64::MAX)
        .with_uncompressed_fallback();
    let bytes_fallback = encode_icer(&img, &opts_with_fallback).unwrap();

    // The fallback can only equal or beat the compressed-only output.
    assert!(
        bytes_fallback.len() <= bytes_compressed.len(),
        "fallback {} > compressed {}",
        bytes_fallback.len(),
        bytes_compressed.len()
    );
    // For pure-noise content the entropy stage should expand; the
    // fallback should strictly win.
    assert!(
        bytes_fallback.len() < bytes_compressed.len(),
        "expected fallback strict win on noise content; \
         compressed={} fallback={}",
        bytes_compressed.len(),
        bytes_fallback.len(),
    );

    // The emitted segment header carries the uncompressed flag set,
    // so the decoder reconstructs via the §III.D path.
    let meta = parse_icer_metadata(&bytes_fallback).unwrap();
    assert_eq!(meta.segments.len(), 1);
    assert!(
        meta.segments[0].header.uncompressed,
        "noise segment should have taken the uncompressed fallback"
    );
}

#[test]
fn fallback_keeps_compressed_when_compressed_is_smaller() {
    // Smooth ramp: compressed wins by a wide margin, fallback is a
    // no-op (other than the extra encode of the uncompressed candidate
    // that gets thrown away).
    let img = ramp_image(64, 64);

    let opts = EncodeOptions::compressed().with_uncompressed_fallback();
    let bytes = encode_icer(&img, &opts).unwrap();
    let meta = parse_icer_metadata(&bytes).unwrap();
    assert_eq!(meta.segments.len(), 1);
    assert!(
        !meta.segments[0].header.uncompressed,
        "ramp segment should have stayed on the compressed path"
    );

    // And the decoder still reconstructs cleanly (filter Q is
    // bit-exact lossless, so the reconstructed pixels equal the
    // input).
    let decoded = parse_icer(&bytes).unwrap();
    assert_eq!(decoded.planes[0].data, img.planes[0].data);
}

#[test]
fn fallback_decode_roundtrip_on_noise() {
    // Decoder must reconstruct the noise tile byte-exactly when the
    // fallback emits uncompressed (the §III.D path is a literal copy).
    let img = noise_image(32, 32, 0xCAFE);

    let opts = EncodeOptions::compressed().with_uncompressed_fallback();
    let bytes = encode_icer(&img, &opts).unwrap();
    let decoded = parse_icer(&bytes).unwrap();
    assert_eq!(decoded.width, img.width);
    assert_eq!(decoded.height, img.height);
    // Uncompressed is a literal byte copy of the input plane -- any
    // mismatch is a wire-format bug, not a quantisation effect.
    let meta = parse_icer_metadata(&bytes).unwrap();
    assert!(meta.segments[0].header.uncompressed);
    assert_eq!(decoded.planes[0].data, img.planes[0].data);
}

#[test]
fn fallback_is_per_segment_in_multi_segment_image() {
    // Two stacked strips: top half is noise (will trigger fallback),
    // bottom half is a ramp (compressed should win). Each segment
    // makes its own decision independently per IPN 42-155 §III.D
    // ("a per-segment decision").
    let w = 32u32;
    let h = 32u32; // segment_count = 2 -> two 16-row strips.

    let noise = noise_image(w, h / 2, 0xDEAD);
    let ramp = ramp_image(w, h / 2);

    // Build the composite image manually.
    let mut img = IcerImage::zeros(w, h, IcerPixelFormat::Gray8);
    let stride = img.planes[0].stride;
    for y in 0..(h / 2) as usize {
        let src = &noise.planes[0].data
            [y * noise.planes[0].stride..y * noise.planes[0].stride + w as usize];
        img.planes[0].data[y * stride..y * stride + w as usize].copy_from_slice(src);
    }
    for y in 0..(h / 2) as usize {
        let src =
            &ramp.planes[0].data[y * ramp.planes[0].stride..y * ramp.planes[0].stride + w as usize];
        let dst_y = y + (h / 2) as usize;
        img.planes[0].data[dst_y * stride..dst_y * stride + w as usize].copy_from_slice(src);
    }

    let mut opts = EncodeOptions::compressed();
    opts.segment_count = 2;
    let opts = opts.with_uncompressed_fallback();
    let bytes = encode_icer(&img, &opts).unwrap();
    let meta = parse_icer_metadata(&bytes).unwrap();
    assert_eq!(meta.segments.len(), 2);

    // Segment 0 = noise; segment 1 = ramp. Index order on encode is
    // segment_index ascending (no priorities supplied).
    assert!(
        meta.segments[0].header.uncompressed,
        "noise strip should have taken the fallback"
    );
    assert!(
        !meta.segments[1].header.uncompressed,
        "ramp strip should have stayed compressed"
    );
}

#[test]
fn fallback_no_op_when_forced_uncompressed() {
    // `uncompressed = true` (the default `EncodeOptions::default()`)
    // forces the uncompressed path unconditionally; setting the
    // fallback flag on top is a no-op.
    let img = ramp_image(16, 16);
    let opts_force = EncodeOptions::default(); // uncompressed = true.
    let bytes_force = encode_icer(&img, &opts_force).unwrap();

    let opts_force_with_flag = EncodeOptions::default().with_uncompressed_fallback();
    let bytes_force_with_flag = encode_icer(&img, &opts_force_with_flag).unwrap();
    assert_eq!(bytes_force, bytes_force_with_flag);
}

#[test]
fn fallback_composes_with_filter_choice() {
    // Setting the fallback on a non-default filter should still
    // produce a valid decode-roundtrip — the fallback decision is
    // orthogonal to the filter choice on the compressed candidate.
    let img = noise_image(16, 16, 0xF00D);
    let mut opts = EncodeOptions::compressed();
    opts.filter = WaveletFilter::NineSevenA;
    let opts = opts.with_uncompressed_fallback();
    let bytes = encode_icer(&img, &opts).unwrap();
    let meta = parse_icer_metadata(&bytes).unwrap();
    assert_eq!(meta.segments.len(), 1);
    // Noise tile under either filter -- the uncompressed candidate
    // beats both. The decoded plane equals the input plane because
    // §III.D is a literal byte copy on this path.
    assert!(meta.segments[0].header.uncompressed);
    let decoded = parse_icer(&bytes).unwrap();
    assert_eq!(decoded.planes[0].data, img.planes[0].data);
}
