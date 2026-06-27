//! Tests for the round-174 [`DecodeLimits`] geometry cap.
//!
//! The wire-format 12-byte segment header can declare a width / height
//! of up to `u16 * u16 ≈ 4.29 GPx`, which the cargo-fuzz harness
//! (round 131) flagged as a DoS surface: a 12-byte input could request
//! a ~4 GB plane plus ~16 GB of coefficients before the decoder did any
//! real work. Round 174 adds [`DecodeLimits`] with conservative
//! application-level defaults (64 MPx per segment, 256 MPx total) and
//! the [`parse_icer_with_limits`] / [`parse_icer_metadata_with_limits`]
//! escape hatches for trusted-input batch processing.

use oxideav_icer::{
    encode_icer, parse_icer, parse_icer_metadata, parse_icer_metadata_with_limits,
    parse_icer_with_limits, DecodeLimits, EncodeOptions, IcerError, IcerImage, IcerPixelFormat,
    SegmentHeader, WaveletFilter,
};

/// Build a 12-byte segment header that declares oversized geometry but
/// carries no packet body. Useful for exercising the geometry cap
/// without paying the cost of producing a real encoded stream at that
/// size.
fn synth_header(width: u16, height: u16, uncompressed: bool) -> Vec<u8> {
    let header = SegmentHeader {
        sync_prefix: 0xACED,
        filter: WaveletFilter::Reversible53,
        decomp_levels: 3,
        uncompressed,
        width,
        height,
        bit_plane_count: 8,
        interleaved_entropy: false,
        segment_length: 0,
        segment_index: 0,
    };
    header.encode().to_vec()
}

#[test]
fn default_limits_accept_rover_sized_input() {
    // A 64x48 Gray8 image encodes + decodes round-trip cleanly under
    // default limits. This is the regression guard: the cap must NOT
    // affect realistic Mars-rover / Pancam-sized inputs (which top out
    // around 1024x1024 = 1 MPx, well below the 64 MPx per-segment cap).
    let mut img = IcerImage::zeros(64, 48, IcerPixelFormat::Gray8);
    for y in 0..48u32 {
        for x in 0..64u32 {
            img.planes[0].data[(y * 64 + x) as usize] = ((x + y) & 0xFF) as u8;
        }
    }
    let opts = EncodeOptions::compressed();
    let bytes = encode_icer(&img, &opts).expect("encode");
    let decoded = parse_icer(&bytes).expect("decode under default limits");
    assert_eq!(decoded.width, 64);
    assert_eq!(decoded.height, 48);
    // Filter Q is lossless: bit-exact round-trip.
    assert_eq!(decoded.planes[0].data, img.planes[0].data);
}

#[test]
fn default_limits_reject_4gb_synthetic_header() {
    // A 12-byte synthetic header declaring 65535x65535 ≈ 4.29 GPx
    // would, pre-round-174, force `parse_icer` to allocate a 4 GB
    // plane + 16 GB of coefficient buffers before discovering the
    // body was empty. Under default limits the metadata walk MUST
    // reject this with `Unsupported` before any allocation.
    let bytes = synth_header(65535, 65535, false);
    let err = parse_icer(&bytes).expect_err("must reject");
    assert!(
        matches!(err, IcerError::Unsupported(_)),
        "expected Unsupported, got {err:?}"
    );
    let err = parse_icer_metadata(&bytes).expect_err("metadata must reject");
    assert!(
        matches!(err, IcerError::Unsupported(_)),
        "expected Unsupported, got {err:?}"
    );
}

#[test]
fn parse_icer_metadata_with_limits_unlimited_walks_giant_header() {
    // The unlimited policy is opt-in. With it, the metadata walker
    // returns header info even for a 4 GPx synthetic header — it
    // performs no plane allocation, so honouring giant geometry on a
    // header-only walk is safe under explicit caller consent.
    let bytes = synth_header(65535, 65535, false);
    let meta = parse_icer_metadata_with_limits(&bytes, &DecodeLimits::unlimited())
        .expect("unlimited metadata walk");
    assert_eq!(meta.segments.len(), 1);
    assert_eq!(meta.segments[0].header.width, 65535);
    assert_eq!(meta.segments[0].header.height, 65535);
}

#[test]
fn parse_icer_with_limits_honours_explicit_per_segment_cap() {
    // Encode a real 32x32 image, then attempt to decode under a cap
    // that's too small for it (1023 pixels). Even though the input is
    // well within the wire-format range, the explicit policy says
    // "don't agree to allocate this", so the call must fail with
    // `Unsupported` (a deliberate application-policy refusal, not a
    // wire-format violation).
    let img = IcerImage::zeros(32, 32, IcerPixelFormat::Gray8);
    let bytes = encode_icer(&img, &EncodeOptions::compressed()).expect("encode");

    let strict = DecodeLimits {
        max_pixels_per_segment: 1023,
        max_total_pixels: 1024 * 1024,
    };
    let err = parse_icer_with_limits(&bytes, &strict).expect_err("must reject");
    assert!(
        matches!(err, IcerError::Unsupported(ref m) if m.contains("per-segment")),
        "expected per-segment Unsupported, got {err:?}"
    );

    // Same input under a cap that just-fits decodes cleanly.
    let permissive = DecodeLimits {
        max_pixels_per_segment: 32 * 32,
        max_total_pixels: 32 * 32,
    };
    let decoded = parse_icer_with_limits(&bytes, &permissive).expect("decode just-fits");
    assert_eq!(decoded.width, 32);
    assert_eq!(decoded.height, 32);
}

#[test]
fn parse_icer_with_limits_honours_multi_segment_total_cap() {
    // Encode a 4x12 image split into 4 segments (each 4x3 = 12 px,
    // total 48 px). A total cap of 47 px must reject the second-segment
    // walk (the first segment passes per-segment but the running total
    // crosses the cap mid-walk).
    let mut img = IcerImage::zeros(4, 12, IcerPixelFormat::Gray8);
    for y in 0..12u32 {
        for x in 0..4u32 {
            img.planes[0].data[(y * 4 + x) as usize] = ((y * 7 + x * 11) & 0xFF) as u8;
        }
    }
    let mut opts = EncodeOptions::compressed();
    opts.segment_count = 4;
    let bytes = encode_icer(&img, &opts).expect("encode 4-segment");

    let strict_total = DecodeLimits {
        max_pixels_per_segment: 1 << 20,
        max_total_pixels: 47,
    };
    let err = parse_icer_with_limits(&bytes, &strict_total).expect_err("must reject");
    assert!(
        matches!(err, IcerError::Unsupported(ref m) if m.contains("total")),
        "expected total-cap Unsupported, got {err:?}"
    );

    // Total cap that just-fits the 48 px aggregate decodes cleanly.
    let fits_total = DecodeLimits {
        max_pixels_per_segment: 1 << 20,
        max_total_pixels: 48,
    };
    let decoded = parse_icer_with_limits(&bytes, &fits_total).expect("decode just-fits total");
    assert_eq!(decoded.width, 4);
    assert_eq!(decoded.height, 12);
    // Filter Q is lossless: bit-exact round-trip across the 4-segment split.
    assert_eq!(decoded.planes[0].data, img.planes[0].data);
}

#[test]
fn decode_limits_default_constants_match_documented_values() {
    // The README + module docs cite the per-segment cap as 64 MPx and
    // the total cap as 256 MPx. Guard the constants so a future tweak
    // forces an explicit README + CHANGELOG update.
    assert_eq!(
        DecodeLimits::DEFAULT_MAX_PIXELS_PER_SEGMENT,
        64 * 1024 * 1024
    );
    assert_eq!(DecodeLimits::DEFAULT_MAX_TOTAL_PIXELS, 256 * 1024 * 1024);
    let d = DecodeLimits::default();
    assert_eq!(
        d.max_pixels_per_segment,
        DecodeLimits::DEFAULT_MAX_PIXELS_PER_SEGMENT
    );
    assert_eq!(d.max_total_pixels, DecodeLimits::DEFAULT_MAX_TOTAL_PIXELS);
    let u = DecodeLimits::unlimited();
    assert_eq!(u.max_pixels_per_segment, u64::MAX);
    assert_eq!(u.max_total_pixels, u64::MAX);
}
