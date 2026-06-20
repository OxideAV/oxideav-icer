//! A tight `DecodeLimits` bounds decode *compute*, not just allocation.
//!
//! Regression for a Fuzz-discovered slow-unit (denial-of-service surface):
//! a 12-byte segment header can declare a multi-megapixel geometry that
//! sits under the 64 MPx default per-segment cap yet costs tens of seconds
//! of inverse-DWT + bit-plane work — even when only a handful of packet
//! body bytes follow (the progressive-truncation feature makes a tiny body
//! legitimate). The two crash inputs declared ~34 MPx (4160×8240) and took
//! 24–59 s through `parse_icer` / `parse_icer_lenient`.
//!
//! `parse_icer_with_limits` / `parse_icer_lenient_with_limits` let a
//! caller bound the geometry the decoder will materialise *before* any
//! pixel buffer is allocated or any DWT runs. A tight limit therefore
//! turns the 50-second decode into a sub-millisecond refusal, which is the
//! mechanism the `decode_segment` fuzz harness now uses so a single
//! crafted header cannot dominate a fuzzing run's wall-clock budget.

use oxideav_icer::{
    parse_icer_lenient_with_limits, parse_icer_with_limits, DecodeLimits, IcerError,
};
use std::time::Instant;

/// Build a 12-byte compressed-segment header declaring `width × height`,
/// `decomp_levels`, `bit_plane_count`, with no packet body (segment_length
/// = 0). The byte layout matches `SegmentHeader::encode`.
fn giant_header(width: u16, height: u16, levels: u8, bit_planes: u8) -> Vec<u8> {
    let mut h = vec![0u8; 12];
    h[0..2].copy_from_slice(&0xACEDu16.to_be_bytes()); // sync prefix (non-zero)
    h[2] = (levels & 0b0000_0111) << 1; // levels in bits 1..=3, uncompressed=0
    h[3..5].copy_from_slice(&width.to_be_bytes());
    h[5..7].copy_from_slice(&height.to_be_bytes());
    h[7] = bit_planes << 2; // bit_plane_count in the high 6 bits
                            // bytes 8..10 segment_length = 0, bytes 10..12 segment_index = 0
    h
}

/// The Fuzz slow-unit geometry: 4160×8240 ≈ 34 MPx, 6 levels, 9 bit
/// planes. Under a tight 1 MPx/segment limit it is refused before any
/// pixel buffer is allocated, in well under a millisecond.
#[test]
fn tight_limits_reject_giant_geometry_fast() {
    let header = giant_header(4160, 8240, 6, 9);
    let limits = DecodeLimits {
        max_pixels_per_segment: 1 << 20, // 1 MPx
        max_total_pixels: 1 << 22,
    };

    let t = Instant::now();
    let strict = parse_icer_with_limits(&header, &limits);
    let lenient = parse_icer_lenient_with_limits(&header, &limits);
    let dt = t.elapsed();

    assert!(
        matches!(strict, Err(IcerError::Unsupported(_))),
        "strict decode should refuse oversized geometry, got {strict:?}"
    );
    assert!(
        lenient.is_err(),
        "lenient decode should also refuse oversized geometry, got {lenient:?}"
    );
    assert!(
        dt.as_millis() < 100,
        "tight-limit refusal must be near-instant, took {dt:?}"
    );
}

/// A geometry just under the tight cap still decodes (the limit refuses,
/// it does not silently truncate sub-cap inputs).
#[test]
fn tight_limits_admit_sub_cap_geometry() {
    // 512×512 = 256 KPx, comfortably under the 1 MPx tight cap.
    let header = giant_header(512, 512, 3, 4);
    let limits = DecodeLimits {
        max_pixels_per_segment: 1 << 20,
        max_total_pixels: 1 << 22,
    };
    // Zero-body compressed segment reconstructs as flat-128; the point is
    // that it is admitted (Ok) rather than refused.
    let decoded = parse_icer_with_limits(&header, &limits).expect("sub-cap geometry must decode");
    assert_eq!(decoded.width, 512);
    assert_eq!(decoded.height, 512);
}
