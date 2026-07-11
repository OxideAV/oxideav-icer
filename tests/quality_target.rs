//! Round 233 -- quality-target rate-control tests.
//!
//! ICER's bandwidth-limited downlink pipeline has historically been
//! configured via a byte budget (`with_byte_budget(n)`: emit at most N
//! bytes, take whatever quality the truncation yields). The inverse
//! shape -- "emit at whatever byte count gets us to quality Q, return
//! the smallest such output" -- is the natural control for pipelines
//! that ship every image at the same target quality rather than the
//! same target size.
//!
//! `EncodeOptions::with_quality_target(target_db)` (round 233) runs a
//! binary search over byte budgets, encodes + decodes each trial,
//! computes PSNR, and returns the smallest output meeting the target.
//! This file exercises:
//!
//! * The trivial-pass case (lossless filter Q: every reachable PSNR is
//!   `+inf`, the floor wins).
//! * The monotonicity property: a higher `target_db` produces an
//!   encode whose byte count is no smaller than a lower `target_db`'s.
//! * The achieved-PSNR property: the search's output decodes to a
//!   PSNR at or above the target (subject to the upper-bound caveat
//!   below).
//! * The upper-bound caveat: targets above what the configured filter
//!   can produce return the unbudgeted encode (best-effort, no
//!   error).
//! * The mutual-exclusion guard: combining quality-target with
//!   byte_budget / target_bytes / rd_pruning returns
//!   `IcerError::Unsupported`.

use oxideav_icer::{
    analyze::psnr_db, encode_icer, parse_icer, EncodeOptions, IcerError, IcerImage,
    IcerPixelFormat, WaveletFilter,
};

/// Build a deterministic diagonal-ramp image, identical to the one used
/// throughout the round-trip tests so test deltas are comparable.
fn ramp_image(w: u32, h: u32) -> IcerImage {
    let mut img = IcerImage::zeros(w, h, IcerPixelFormat::Gray8);
    let stride = img.planes[0].stride;
    for y in 0..h as usize {
        for x in 0..w as usize {
            img.planes[0].data[y * stride + x] = ((x + y) & 0xFF) as u8;
        }
    }
    img
}

/// Build a flat image. The lossless filter Q reproduces this exactly
/// at any non-trivial byte budget; the quality-target search should
/// collapse to the floor bracket on the first trial.
fn flat_image(w: u32, h: u32, value: u8) -> IcerImage {
    let mut img = IcerImage::zeros(w, h, IcerPixelFormat::Gray8);
    for byte in img.planes[0].data.iter_mut() {
        *byte = value;
    }
    img
}

#[test]
fn quality_target_lossless_filter_q_collapses_to_floor() {
    // Filter Q is bit-exact, so every reachable PSNR is +inf. The
    // search should accept the floor-bracket encode on the first
    // trial.
    let img = flat_image(32, 32, 128);
    let opts = EncodeOptions::compressed().with_quality_target(40.0);
    let bytes = encode_icer(&img, &opts).expect("encode_icer failed");
    let decoded = parse_icer(&bytes).expect("parse_icer failed");
    let p = psnr_db(&img, &decoded);
    assert!(
        p.is_infinite() || p >= 40.0,
        "lossless filter Q expected PSNR >= 40 dB, got {p}"
    );
    // The floor-bracket emission for a 32x32 flat input is small (the
    // arithmetic coder collapses to a near-trivial body).
    eprintln!(
        "quality_target_lossless_filter_q_collapses_to_floor: {} bytes, PSNR={p}",
        bytes.len()
    );
}

#[test]
fn quality_target_returns_target_meeting_psnr() {
    // Lossy filter A on a 32x32 ramp; the encoder must find a budget
    // whose decoded PSNR meets the target.
    let img = ramp_image(32, 32);
    let target = 25.0f32;
    let opts = EncodeOptions {
        filter: WaveletFilter::NineSevenA,
        wavelet_levels: 2,
        bit_plane_count: 8,
        uncompressed: false,
        ..EncodeOptions::default()
    }
    .with_quality_target(target);
    let bytes = encode_icer(&img, &opts).expect("encode_icer failed");
    let decoded = parse_icer(&bytes).expect("parse_icer failed");
    let p = psnr_db(&img, &decoded);
    assert!(
        p >= target,
        "quality target {target} dB not met: achieved {p} dB ({} bytes)",
        bytes.len()
    );
}

#[test]
fn quality_target_monotone_in_target_db() {
    // Higher target -> at least as many bytes emitted. The truncated
    // budget trials supply the quality steps the search walks over.
    let img = ramp_image(32, 32);
    let lo_target = 20.0f32;
    let hi_target = 35.0f32;
    let lo_opts = EncodeOptions {
        filter: WaveletFilter::NineSevenA,
        wavelet_levels: 2,
        bit_plane_count: 8,
        uncompressed: false,
        ..EncodeOptions::default()
    }
    .with_quality_target(lo_target);
    let hi_opts = EncodeOptions {
        filter: WaveletFilter::NineSevenA,
        wavelet_levels: 2,
        bit_plane_count: 8,
        uncompressed: false,
        ..EncodeOptions::default()
    }
    .with_quality_target(hi_target);
    let lo_bytes = encode_icer(&img, &lo_opts).expect("lo encode failed");
    let hi_bytes = encode_icer(&img, &hi_opts).expect("hi encode failed");
    assert!(
        hi_bytes.len() >= lo_bytes.len(),
        "monotonicity violated: lo {} bytes > hi {} bytes (targets {lo_target} / {hi_target})",
        lo_bytes.len(),
        hi_bytes.len()
    );
    // Sanity-check the achieved PSNRs satisfy the targets.
    let lo_dec = parse_icer(&lo_bytes).expect("lo decode failed");
    let hi_dec = parse_icer(&hi_bytes).expect("hi decode failed");
    let lo_psnr = psnr_db(&img, &lo_dec);
    let hi_psnr = psnr_db(&img, &hi_dec);
    assert!(
        lo_psnr >= lo_target,
        "lo target {lo_target} dB not met: achieved {lo_psnr} dB"
    );
    assert!(
        hi_psnr >= hi_target,
        "hi target {hi_target} dB not met: achieved {hi_psnr} dB"
    );
}

#[test]
fn quality_target_above_filter_ceiling_returns_best_effort() {
    // 9999 dB is unreachable; the encoder must return the unbudgeted
    // encode as the best effort rather than erroring or looping.
    let img = ramp_image(16, 16);
    let opts = EncodeOptions {
        filter: WaveletFilter::NineSevenA,
        wavelet_levels: 2,
        bit_plane_count: 8,
        uncompressed: false,
        ..EncodeOptions::default()
    }
    .with_quality_target(9999.0);
    let bytes = encode_icer(&img, &opts).expect("encode_icer should not error");
    let decoded = parse_icer(&bytes).expect("parse_icer failed");
    let p = psnr_db(&img, &decoded);
    // The achieved PSNR is whatever the unbudgeted filter A encode
    // can produce; record it for documentation but don't gate on a
    // specific value (the filter-A ceiling on this input is content-
    // sensitive).
    eprintln!(
        "quality_target_above_filter_ceiling_returns_best_effort: {} bytes, PSNR={p}",
        bytes.len()
    );
    // Output must at least be non-empty and decodable.
    assert!(!bytes.is_empty());
}

#[test]
fn quality_target_conflicts_with_byte_budget() {
    let img = ramp_image(16, 16);
    let opts = EncodeOptions::compressed()
        .with_byte_budget(1024)
        .with_quality_target(30.0);
    let err = encode_icer(&img, &opts).expect_err("should have rejected the combination");
    match err {
        IcerError::Unsupported(msg) => {
            assert!(
                msg.contains("byte_budget"),
                "expected byte_budget conflict, got: {msg}"
            );
        }
        other => panic!("expected Unsupported, got: {other:?}"),
    }
}

#[test]
fn quality_target_conflicts_with_target_bytes() {
    let img = ramp_image(16, 16);
    let opts = EncodeOptions::compressed()
        .with_target_bytes(1024)
        .with_quality_target(30.0);
    let err = encode_icer(&img, &opts).expect_err("should have rejected the combination");
    match err {
        IcerError::Unsupported(msg) => {
            assert!(
                msg.contains("target_bytes"),
                "expected target_bytes conflict, got: {msg}"
            );
        }
        other => panic!("expected Unsupported, got: {other:?}"),
    }
}

#[test]
fn quality_target_conflicts_with_rd_pruning() {
    let img = ramp_image(16, 16);
    let opts = EncodeOptions::compressed()
        .with_rd_budget(1024)
        .with_quality_target(30.0);
    let err = encode_icer(&img, &opts).expect_err("should have rejected the combination");
    match err {
        IcerError::Unsupported(msg) => {
            // The byte_budget guard fires first (with_rd_budget also
            // sets byte_budget). Either guard is acceptable -- both
            // are mutually exclusive with quality_target_psnr.
            assert!(
                msg.contains("byte_budget") || msg.contains("rd_pruning"),
                "expected byte_budget or rd_pruning conflict, got: {msg}"
            );
        }
        other => panic!("expected Unsupported, got: {other:?}"),
    }
}

#[test]
fn quality_target_uncompressed_short_circuits() {
    // EncodeOptions::default() forces the uncompressed path; the
    // round-trip is bit-exact regardless of the quality target, so
    // the search short-circuits and the regular encode runs.
    let img = ramp_image(16, 16);
    let opts = EncodeOptions::default().with_quality_target(50.0);
    let bytes = encode_icer(&img, &opts).expect("encode_icer failed");
    let decoded = parse_icer(&bytes).expect("parse_icer failed");
    let p = psnr_db(&img, &decoded);
    assert!(
        p.is_infinite(),
        "uncompressed path should round-trip bit-exact (PSNR inf), got {p}"
    );
}
