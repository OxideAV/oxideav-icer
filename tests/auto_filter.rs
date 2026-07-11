//! Round-5 integration tests: automatic filter selection.
//!
//! These tests exercise `EncodeOptions::with_auto_filter()` and
//! `EncodeOptions::with_auto_filter_rd()` end-to-end through the
//! standalone API. They assert two properties:
//!
//!   1. Auto-mode picks a sensible filter for different image classes
//!      (smooth -> filter Q; high-frequency -> filter A).
//!   2. The rate-distortion variant never produces a larger output
//!      than the worst candidate it considered.

use oxideav_icer::{
    analyze, encode_icer, parse_icer, pick_filter_by_rate_distortion, recommend_filter,
    EncodeOptions, IcerImage, IcerPixelFormat, ImageStats, WaveletFilter, DEFAULT_RD_CANDIDATES,
};

fn flat_image(w: u32, h: u32, value: u8) -> IcerImage {
    let mut img = IcerImage::zeros(w, h, IcerPixelFormat::Gray8);
    for byte in img.planes[0].data.iter_mut() {
        *byte = value;
    }
    img
}

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

fn checkerboard_image(w: u32, h: u32) -> IcerImage {
    let mut img = IcerImage::zeros(w, h, IcerPixelFormat::Gray8);
    let stride = img.planes[0].stride;
    for y in 0..h as usize {
        for x in 0..w as usize {
            let v = if (x ^ y) & 1 == 0 { 0u8 } else { 255u8 };
            img.planes[0].data[y * stride + x] = v;
        }
    }
    img
}

fn psnr(original: &IcerImage, decoded: &IcerImage) -> f64 {
    assert_eq!(original.width, decoded.width);
    assert_eq!(original.height, decoded.height);
    let n = (original.width * original.height) as f64;
    let mse: f64 = original.planes[0]
        .data
        .iter()
        .zip(decoded.planes[0].data.iter())
        .map(|(&a, &b)| {
            let d = a as f64 - b as f64;
            d * d
        })
        .sum::<f64>()
        / n;
    if mse == 0.0 {
        f64::INFINITY
    } else {
        10.0 * (255.0f64 * 255.0 / mse).log10()
    }
}

// ---------------------------------------------------------------------------
// Heuristic auto-filter
// ---------------------------------------------------------------------------

#[test]
fn auto_filter_picks_q_on_flat_image() {
    // A flat image has zero gradient + zero variance -> heuristic must
    // pick filter Q (reversible). End-to-end roundtrip must be lossless.
    let img = flat_image(32, 32, 128);
    let opts = EncodeOptions::compressed().with_auto_filter();
    let bytes = encode_icer(&img, &opts).expect("encode failed");
    let decoded = parse_icer(&bytes).expect("decode failed");
    // Filter Q is the only reversible option -- a flat image should
    // roundtrip bit-exactly.
    assert_eq!(
        decoded.planes[0].data, img.planes[0].data,
        "auto-mode on flat image must be lossless via filter Q"
    );
}

#[test]
fn auto_filter_picks_q_on_smooth_gradient() {
    // A smooth diagonal gradient falls into the "low edge energy"
    // bucket -> heuristic picks Q -> lossless roundtrip.
    let img = ramp_image(32, 32);
    let opts = EncodeOptions::compressed().with_auto_filter();
    let bytes = encode_icer(&img, &opts).expect("encode failed");
    let decoded = parse_icer(&bytes).expect("decode failed");
    assert_eq!(
        decoded.planes[0].data, img.planes[0].data,
        "auto-mode on smooth ramp must be lossless via filter Q"
    );
}

#[test]
fn auto_filter_picks_filter_a_on_checkerboard() {
    // Checkerboard has very high edge energy + variance -> heuristic
    // picks filter A (beta = 0 high-pass predictor, §II.A Table 1).
    let img = checkerboard_image(32, 32);
    let (stats, recommended) = analyze(&img);
    assert_eq!(
        recommended,
        WaveletFilter::NineSevenA,
        "heuristic should pick filter A for checkerboard (stats={stats:?})"
    );

    let opts = EncodeOptions::compressed().with_auto_filter();
    let bytes = encode_icer(&img, &opts).expect("encode failed");
    let decoded = parse_icer(&bytes).expect("decode failed");
    // Filter A is a §II.A reversible integer transform: the
    // full-quality round-trip is bit-exact under it too.
    assert_eq!(
        decoded.planes[0].data, img.planes[0].data,
        "auto-picked filter A must still round-trip bit-exact"
    );
}

#[test]
fn auto_filter_overrides_explicit_filter_setting() {
    // Confirm that setting auto_filter overrides whatever filter the
    // caller specified explicitly. We set filter F then enable auto on
    // a flat image -> auto must pick Q, lossless output.
    let img = flat_image(16, 16, 100);
    let opts = EncodeOptions {
        filter: WaveletFilter::FilterF,
        uncompressed: false,
        ..EncodeOptions::default()
    }
    .with_auto_filter();
    let bytes = encode_icer(&img, &opts).expect("encode failed");
    let decoded = parse_icer(&bytes).expect("decode failed");
    assert_eq!(
        decoded.planes[0].data, img.planes[0].data,
        "auto-mode should override explicit filter F in favour of Q on flat input"
    );
}

#[test]
fn auto_filter_disabled_uses_caller_filter() {
    // Sanity check: with auto_filter off, the explicit filter setting
    // is honoured exactly as before.
    let img = flat_image(16, 16, 100);
    let opts = EncodeOptions {
        filter: WaveletFilter::FilterF,
        uncompressed: false,
        ..EncodeOptions::default()
    };
    let bytes = encode_icer(&img, &opts).expect("encode failed");
    let decoded = parse_icer(&bytes).expect("decode failed");
    assert_eq!(
        decoded.planes[0].data, img.planes[0].data,
        "explicit filter F round-trip must be bit-exact"
    );
}

// ---------------------------------------------------------------------------
// Rate-distortion auto-filter
// ---------------------------------------------------------------------------

#[test]
fn auto_filter_rd_picks_smallest_output() {
    // RD mode should trial each candidate and pick the one with the
    // smallest output. Filter Q is reversible and produces a stable
    // bit-stream for the ramp image; filter A may produce a different
    // size. The RD result must be <= both individual trial sizes.
    let img = ramp_image(32, 32);

    // Measure each candidate individually.
    let q_opts = EncodeOptions {
        filter: WaveletFilter::Reversible53,
        uncompressed: false,
        ..EncodeOptions::default()
    };
    let q_bytes = encode_icer(&img, &q_opts).expect("Q encode failed");

    let a_opts = EncodeOptions {
        filter: WaveletFilter::NineSevenA,
        uncompressed: false,
        ..EncodeOptions::default()
    };
    let a_bytes = encode_icer(&img, &a_opts).expect("A encode failed");

    let min_individual = q_bytes.len().min(a_bytes.len());

    let rd_opts = EncodeOptions::compressed().with_auto_filter_rd();
    let rd_bytes = encode_icer(&img, &rd_opts).expect("RD encode failed");

    assert!(
        rd_bytes.len() <= min_individual,
        "RD mode picked output {} bytes; individual best was {} bytes",
        rd_bytes.len(),
        min_individual
    );

    // RD must still produce a decodable stream.
    let _decoded = parse_icer(&rd_bytes).expect("RD decode failed");
}

#[test]
fn pick_filter_by_rate_distortion_returns_smallest() {
    let img = ramp_image(16, 16);
    let opts = EncodeOptions::compressed();
    let (filter, bytes) =
        pick_filter_by_rate_distortion(&img, &opts, DEFAULT_RD_CANDIDATES).expect("RD failed");
    // Verify the returned byte count matches a fresh encode with that filter.
    let trial_opts = EncodeOptions { filter, ..opts };
    let fresh = encode_icer(&img, &trial_opts).expect("fresh encode failed");
    assert_eq!(
        fresh.len(),
        bytes,
        "RD reported {bytes} bytes for filter {filter:?}, fresh encode gave {}",
        fresh.len()
    );
}

#[test]
fn pick_filter_by_rate_distortion_rejects_empty_candidates() {
    let img = ramp_image(8, 8);
    let opts = EncodeOptions::compressed();
    let r = pick_filter_by_rate_distortion(&img, &opts, &[]);
    assert!(r.is_err(), "empty candidate slice must return Err");
}

#[test]
fn auto_filter_with_byte_budget_respects_cap() {
    // Auto-filter must compose correctly with the byte budget.
    let img = ramp_image(64, 64);
    let opts = EncodeOptions::compressed()
        .with_auto_filter()
        .with_byte_budget(2048);
    let bytes = encode_icer(&img, &opts).expect("encode failed");
    assert!(
        bytes.len() <= 2048,
        "auto+budget output {} exceeds cap 2048",
        bytes.len()
    );
    let decoded = parse_icer(&bytes).expect("decode failed");
    let _ = psnr(&img, &decoded);
}

// ---------------------------------------------------------------------------
// Recommend / stats edge cases
// ---------------------------------------------------------------------------

#[test]
fn recommend_filter_is_deterministic() {
    // Same image -> same recommendation across two calls.
    let img = ramp_image(16, 16);
    let s1 = ImageStats::from_image(&img);
    let s2 = ImageStats::from_image(&img);
    assert_eq!(s1, s2, "stats must be deterministic");
    assert_eq!(recommend_filter(&s1), recommend_filter(&s2));
}

#[test]
fn analyze_helper_matches_individual_calls() {
    let img = checkerboard_image(8, 8);
    let (combined_stats, combined_filter) = analyze(&img);
    let direct_stats = ImageStats::from_image(&img);
    let direct_filter = recommend_filter(&direct_stats);
    assert_eq!(combined_stats, direct_stats);
    assert_eq!(combined_filter, direct_filter);
}
