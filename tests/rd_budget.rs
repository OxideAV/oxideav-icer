//! Tests for rate-distortion budget pruning (round 91 feature,
//! IPN 42-155 §IV.B rate-allocation principle).
//!
//! The R-D budget mode `with_rd_budget(n)` differs from
//! `with_byte_budget(n)` in *which* packets are kept when the budget
//! is tight:
//!
//!   * `with_byte_budget(n)`: emit MSB-down, significance-before-
//!     refinement, until the next packet would exceed `n`. The cut
//!     happens at whatever packet fills the budget — typically a
//!     refinement packet at a low bit-plane, costing high bytes for
//!     low distortion-reduction.
//!   * `with_rd_budget(n)`: globally rank every encoded packet by
//!     `delta_distortion / wire_size`, greedy-include in descending
//!     order subject to the MSB-down dependency graph. This tends to
//!     drop trailing refinement packets (low ΔD/byte) and instead
//!     include deeper significance passes in their place.
//!
//! The test below empirically demonstrates the gain on the canonical
//! 256×256 gradient fixture used by the round-4 quota tests.

use oxideav_icer::{
    encode_icer, parse_icer, subband_weight_map, EncodeOptions, IcerImage, IcerPixelFormat,
    WaveletFilter,
};

/// Build a 256×256 8-bit gray gradient image with both horizontal and
/// vertical structure -- richer than a pure horizontal ramp so the
/// bit-plane scanner has work to do across many planes.
fn gradient_256x256() -> IcerImage {
    let w = 256u32;
    let h = 256u32;
    let mut img = IcerImage::zeros(w, h, IcerPixelFormat::Gray8);
    let plane = &mut img.planes[0];
    for y in 0..h as usize {
        for x in 0..w as usize {
            let v = ((x + y) & 0xFF) as u8;
            plane.data[y * plane.stride + x] = v;
        }
    }
    img
}

/// A noisier fixture: x*y modulo-driven texture for the R-D pruner to
/// chew on.
fn texture_256x256() -> IcerImage {
    let w = 256u32;
    let h = 256u32;
    let mut img = IcerImage::zeros(w, h, IcerPixelFormat::Gray8);
    let plane = &mut img.planes[0];
    for y in 0..h as usize {
        for x in 0..w as usize {
            let v = ((x.wrapping_mul(13) ^ y.wrapping_mul(7)) & 0xFF) as u8;
            plane.data[y * plane.stride + x] = v;
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
            let diff = a as f64 - b as f64;
            diff * diff
        })
        .sum::<f64>()
        / n;
    if mse == 0.0 {
        f64::INFINITY
    } else {
        10.0 * (255.0f64 * 255.0 / mse).log10()
    }
}

/// `with_rd_budget(n)` respects the byte cap exactly like
/// `with_byte_budget(n)` (no over-run), and the output decodes
/// cleanly.
#[test]
fn rd_budget_respects_hard_cap_and_decodes() {
    let image = gradient_256x256();
    let budgets: &[u64] = &[1024, 4096, 16384, 65536];
    let mut prev_psnr = 0.0f64;
    for &budget in budgets {
        let opts = EncodeOptions::compressed().with_rd_budget(budget);
        let encoded = encode_icer(&image, &opts).expect("encode failed");
        assert!(
            encoded.len() as u64 <= budget,
            "rd_budget={budget}: output {} bytes exceeds cap",
            encoded.len()
        );
        let decoded = parse_icer(&encoded).expect("decode failed");
        assert_eq!(decoded.width, 256);
        assert_eq!(decoded.height, 256);
        let p = psnr(&image, &decoded);
        assert!(p >= 0.0, "rd_budget={budget}: PSNR {p:.2} dB negative");
        // R-D mode should not regress PSNR as budget grows. Use the
        // same -0.1 dB slack as the round-4 quota tests.
        assert!(
            p >= prev_psnr - 0.1,
            "rd_budget={budget}: PSNR {p:.2} dB regressed from prev {prev_psnr:.2} dB"
        );
        prev_psnr = p;
        eprintln!(
            "rd_budget={budget}: {} bytes -> PSNR {p:.2} dB",
            encoded.len()
        );
    }
}

/// Determinism: the R-D selector returns the same kept set for
/// repeated runs over identical input.
#[test]
fn rd_budget_is_deterministic() {
    let image = gradient_256x256();
    let opts = EncodeOptions::compressed().with_rd_budget(4096);
    let bytes_a = encode_icer(&image, &opts).expect("encode 1");
    let bytes_b = encode_icer(&image, &opts).expect("encode 2");
    assert_eq!(
        bytes_a, bytes_b,
        "R-D selection produced different bytes on two identical runs"
    );
}

/// `with_rd_budget(huge_budget)` must converge to the full-quality
/// output: with enough budget every packet fits, so R-D selection
/// reduces to "include everything" and the result is the lossless
/// round-trip for filter Q.
#[test]
fn rd_budget_with_huge_cap_is_lossless_for_filter_q() {
    let image = gradient_256x256();
    let huge = 1024u64 * 1024; // 1 MiB ceiling
    let opts = EncodeOptions::compressed().with_rd_budget(huge);
    let encoded = encode_icer(&image, &opts).expect("encode failed");
    let decoded = parse_icer(&encoded).expect("decode failed");
    assert_eq!(
        decoded.planes[0].data, image.planes[0].data,
        "filter Q with R-D + huge budget must reconstruct bit-exactly"
    );
}

/// A 64×64 checkerboard fixture. High-frequency content -- exactly
/// the case where the strict-MSB cut-off spends bytes on zero-ΔD
/// significance packets at the high bit-planes (no coefficient
/// straddles those planes after the DWT) while skipping low-bit-plane
/// refinement packets that DO carry distortion-reduction value.
fn checker_64x64() -> IcerImage {
    let w = 64u32;
    let h = 64u32;
    let mut img = IcerImage::zeros(w, h, IcerPixelFormat::Gray8);
    let stride = img.planes[0].stride;
    for y in 0..h as usize {
        for x in 0..w as usize {
            img.planes[0].data[y * stride + x] = if (x ^ y) & 1 == 0 { 0 } else { 255 };
        }
    }
    img
}

/// A 64×64 image with four sparse impulses on a black background --
/// the wavelet transform yields many zero-ΔD bit-planes near the LSB
/// and a few high-magnitude coefficients near the MSB. R-D should be
/// able to skip the zero-ΔD significance packets near the LSB end.
fn sparse_impulses_64x64() -> IcerImage {
    let w = 64u32;
    let h = 64u32;
    let mut img = IcerImage::zeros(w, h, IcerPixelFormat::Gray8);
    let stride = img.planes[0].stride;
    for &(x, y, v) in &[
        (5usize, 5usize, 200u8),
        (20, 20, 180),
        (40, 40, 160),
        (50, 10, 140),
    ] {
        img.planes[0].data[y * stride + x] = v;
    }
    img
}

/// Headline R-D claim: at the same tight byte budget, the R-D pruner
/// matches or beats the strict MSB-truncation cut-off on PSNR.
///
/// The classical R-D failure of strict-MSB truncation is to spend
/// many bytes on a low-bit-plane refinement packet that yields tiny
/// distortion reduction. The R-D mode reallocates those bytes to a
/// higher-priority significance or refinement that yields more.
///
/// We measure across a few fixtures and budgets and assert R-D is
/// at least as good (within a tiny epsilon) on every case and
/// strictly better in at least one.
#[test]
fn rd_budget_matches_or_beats_strict_msb() {
    let cases = [
        ("gradient256", gradient_256x256()),
        ("texture256", texture_256x256()),
        ("checker64", checker_64x64()),
        ("impulses64", sparse_impulses_64x64()),
    ];
    // Budgets chosen to land *inside* the bit-plane chain (not so
    // tight that no packets fit; not so loose that everything fits).
    let budgets: &[u64] = &[200, 400, 800, 1600, 3200];

    let mut strict_wins = 0;
    let mut rd_wins = 0;
    let mut ties = 0;

    for (name, image) in &cases {
        for &budget in budgets {
            let strict_opts = EncodeOptions::compressed().with_byte_budget(budget);
            let rd_opts = EncodeOptions::compressed().with_rd_budget(budget);

            let strict_bytes = encode_icer(image, &strict_opts).expect("strict encode");
            let rd_bytes = encode_icer(image, &rd_opts).expect("rd encode");
            assert!(strict_bytes.len() as u64 <= budget);
            assert!(rd_bytes.len() as u64 <= budget);

            let strict_dec = parse_icer(&strict_bytes).expect("strict decode");
            let rd_dec = parse_icer(&rd_bytes).expect("rd decode");

            let strict_psnr = psnr(image, &strict_dec);
            let rd_psnr = psnr(image, &rd_dec);

            eprintln!(
                "{name} budget={budget}: strict={} bytes / {strict_psnr:.2} dB; \
                 rd={} bytes / {rd_psnr:.2} dB; delta={:.2} dB",
                strict_bytes.len(),
                rd_bytes.len(),
                rd_psnr - strict_psnr
            );

            // R-D must never be measurably worse (allow tiny FP slack).
            assert!(
                rd_psnr >= strict_psnr - 0.05,
                "{name} budget={budget}: R-D regressed (strict {strict_psnr:.2}, rd {rd_psnr:.2})"
            );

            if rd_psnr > strict_psnr + 0.05 {
                rd_wins += 1;
            } else if strict_psnr > rd_psnr + 0.05 {
                strict_wins += 1;
            } else {
                ties += 1;
            }
        }
    }

    eprintln!("R-D vs strict: {rd_wins} R-D wins, {strict_wins} strict wins, {ties} ties");
    // Strict should NEVER measurably win (asserted in the loop above).
    // R-D must win at least one case across the sweep -- demonstrates
    // the §IV.B rate-allocation benefit on high-frequency content.
    assert_eq!(
        strict_wins, 0,
        "strict MSB truncation should never measurably beat R-D pruning"
    );
    assert!(
        rd_wins >= 1,
        "R-D pruning expected to win at least one budget x fixture combination; \
         got {rd_wins} wins / {ties} ties"
    );
}

/// §III.A image-domain weighting: the per-coefficient weight map must
/// expose the non-unitary structure of ICER's wavelet transform — a unit
/// error in a low-frequency (deeper-level / LL) coefficient injects more
/// reconstructed-image distortion than the same error in a level-1
/// high-frequency (HH) coefficient. We assert the deepest-level (LL)
/// weight strictly exceeds the level-1 HH weight, since otherwise the
/// weighting would be a no-op and could not steer the R-D selector.
#[test]
fn weight_map_reflects_non_unitary_subband_structure() {
    let w = 32usize;
    let h = 32usize;
    let levels = 3u8;
    let map = subband_weight_map(w, h, levels, WaveletFilter::Reversible53);
    assert_eq!(map.len(), w * h);

    // Level-1 HH lives at an odd/odd interior position, e.g. (5, 5).
    let hh1 = map[5 * w + 5];
    // LL3 lives where every coordinate stays even through 3 levels: a
    // multiple of 8 away from the boundary, e.g. (8, 8).
    let ll3 = map[8 * w + 8];
    // HL1 (odd x, even y), e.g. (5, 8).
    let hl1 = map[8 * w + 5];

    assert!(
        hh1 > 0.0 && ll3 > 0.0 && hl1 > 0.0,
        "weights must be positive"
    );
    assert!(
        ll3 > hh1,
        "LL weight {ll3:.4} must exceed level-1 HH weight {hh1:.4} \
         (§III.A non-unitary effect on reconstructed-image MSE)"
    );
    eprintln!("weights: LL3={ll3:.4} HL1={hl1:.4} HH1={hh1:.4}");
}

/// The §III.A image-domain weighting must never make the R-D selector
/// produce an output that the strict-MSB path strictly beats, and on the
/// high-frequency checkerboard fixture it should deliver a sizeable PSNR
/// win at a tight budget (the canonical case where transform-domain MSE
/// mis-ranks packets relative to reconstructed-image MSE).
#[test]
fn weighted_rd_beats_strict_on_checkerboard() {
    let image = checker_64x64();
    let budget = 400u64;
    let strict = encode_icer(
        &image,
        &EncodeOptions::compressed().with_byte_budget(budget),
    )
    .expect("strict encode");
    let rd = encode_icer(&image, &EncodeOptions::compressed().with_rd_budget(budget))
        .expect("rd encode");
    assert!(strict.len() as u64 <= budget && rd.len() as u64 <= budget);
    let strict_psnr = psnr(&image, &parse_icer(&strict).unwrap());
    let rd_psnr = psnr(&image, &parse_icer(&rd).unwrap());
    eprintln!("checker64 b={budget}: strict={strict_psnr:.2} dB rd={rd_psnr:.2} dB");
    assert!(
        rd_psnr >= strict_psnr + 3.0,
        "§III.A-weighted R-D should beat strict by >= 3 dB on checkerboard; \
         strict={strict_psnr:.2} rd={rd_psnr:.2}"
    );
}
