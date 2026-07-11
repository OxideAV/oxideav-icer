//! IPN 42-155 §VI.B cross-segment progressive byte quota.
//!
//! §VI.B: "because ICER compresses each error-containment segment of a
//! subband bit plane before moving on to another subband bit plane,
//! the output bitstream interleaves data from the segments. This
//! organization of the output bitstream represents progressive
//! compression across all of the segments" — so a byte quota (or a
//! truncation to meet a downlink constraint) cuts that *global*
//! interleaved stream, and the surviving blocks are rearranged so
//! "portions corresponding to a given segment are concatenated in
//! order" (Fig. 23(b)) before transmission.
//!
//! Consequence under test: a budget-truncated multi-segment encode
//! spreads the quota progressively across every segment (each gets its
//! most-significant packets) instead of deep-refining the first
//! segment while later ones starve — the sequential-greedy
//! misallocation these tests would fail against loses 9-16 dB at
//! mid-range budgets on the fixtures below.

use oxideav_icer::{
    encode_icer, parse_icer, parse_icer_metadata, EncodeOptions, IcerImage, IcerPixelFormat,
};

/// Deterministic textured Gray8 fixture (same generator as the §V.B
/// transform-segmentation suite).
fn textured(w: usize, h: usize, seed: u64) -> IcerImage {
    let mut img = IcerImage::zeros(w as u32, h as u32, IcerPixelFormat::Gray8);
    let mut s = seed.wrapping_mul(0x9E37_79B9_7F4A_7C15);
    let stride = img.planes[0].stride;
    for y in 0..h {
        for x in 0..w {
            s = s
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            let noise = ((s >> 56) & 0x1F) as i32;
            let base = ((x * 2 + y * 3) % 200) as i32;
            img.planes[0].data[y * stride + x] = (base + noise).clamp(0, 255) as u8;
        }
    }
    img
}

fn psnr(a: &IcerImage, b: &IcerImage) -> f64 {
    let n = (a.width * a.height) as f64;
    let mse: f64 = a.planes[0]
        .data
        .iter()
        .zip(b.planes[0].data.iter())
        .map(|(&x, &y)| {
            let d = x as f64 - y as f64;
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

fn strip_opts(segments: u16) -> EncodeOptions {
    let mut o = EncodeOptions::compressed();
    o.segment_count = segments;
    o
}

fn transform_opts(segments: u16) -> EncodeOptions {
    let mut o = EncodeOptions::compressed().with_transform_domain_segments();
    o.segment_count = segments;
    o
}

/// An ample budget admits every packet, so the §VI.B path must emit a
/// stream *byte-identical* to the unbudgeted encode — the quota only
/// selects packets, it never reshapes the wire (Fig. 23(b): segments
/// concatenated in order). Covers row strips and §V.B transform
/// domain, both entropy backends, and §III.A priority interleaving.
#[test]
fn ample_budget_is_byte_identical_to_unbudgeted() {
    let img = textured(64, 64, 0x51EB);
    for base in [strip_opts(4), transform_opts(4)] {
        for interleaved_entropy in [false, true] {
            for priority in [false, true] {
                let mut opts = base.clone();
                if interleaved_entropy {
                    opts = opts.with_interleaved_entropy();
                }
                if priority {
                    opts = opts.with_priority_interleaving();
                }
                let free = encode_icer(&img, &opts).unwrap();
                let capped = encode_icer(&img, &opts.clone().with_byte_budget(1 << 20)).unwrap();
                assert_eq!(
                    free, capped,
                    "ample budget must be byte-identical (transform={} ixec={} prio={})",
                    base.transform_segments, interleaved_entropy, priority
                );
            }
        }
    }
}

/// §VI.B balance: at a mid-range budget every segment receives its
/// most-significant packets — no segment is starved to zero while a
/// sibling is deep-refined.
#[test]
fn mid_budget_feeds_every_segment() {
    let img = textured(64, 64, 0x9090);
    for opts in [
        strip_opts(4).with_byte_budget(500),
        transform_opts(4).with_byte_budget(500),
    ] {
        let bytes = encode_icer(&img, &opts).unwrap();
        assert!(bytes.len() as u64 <= 500);
        let meta = parse_icer_metadata(&bytes).unwrap();
        assert_eq!(meta.segments.len(), 4);
        for s in &meta.segments {
            assert!(
                s.packet_count >= 1,
                "segment {} starved at budget 500 (transform={}): {} packets",
                s.header.segment_index,
                opts.transform_segments,
                s.packet_count
            );
        }
    }
}

/// Progressive §VI.B truncation: PSNR is non-decreasing in the budget
/// on both multi-segment paths, and clears absolute floors the
/// sequential-greedy allocator missed by ~9-16 dB (row strips at
/// 1000 B decoded at 14.2 dB under sequential greed; §VI.B reaches
/// 27.4 dB — floors sit ~1 dB below the measured §VI.B values).
#[test]
fn budget_quality_monotone_and_clears_floor() {
    let img = textured(64, 64, 0x9090);
    // (budget, row-strip floor dB, transform floor dB).
    let cases: &[(u64, f64, f64)] = &[
        (500, 21.0, 21.0),  // measured 22.2 / 22.1
        (1000, 26.0, 26.4), // measured 27.4 / 27.5
        (2000, 35.0, 32.9), // measured 36.0 / 33.9
        (3000, 43.5, 41.3), // measured 44.6 / 42.3
    ];
    for transform in [false, true] {
        let mut prev = 0.0f64;
        for &(budget, strip_floor, transform_floor) in cases {
            let base = if transform {
                transform_opts(4)
            } else {
                strip_opts(4)
            };
            let bytes = encode_icer(&img, &base.with_byte_budget(budget)).unwrap();
            assert!(bytes.len() as u64 <= budget);
            let dec = parse_icer(&bytes).unwrap();
            let p = psnr(&img, &dec);
            let floor = if transform {
                transform_floor
            } else {
                strip_floor
            };
            assert!(
                p >= floor,
                "budget {budget} (transform={transform}): {p:.2} dB below §VI.B floor {floor}"
            );
            assert!(
                p >= prev - 0.1,
                "budget {budget} (transform={transform}): {p:.2} dB regressed from {prev:.2}"
            );
            prev = p;
        }
    }
}

/// The soft target stops the global walk after finishing the §VI.B
/// step in progress: output lands at-or-just-above the target and
/// still decodes with the full geometry.
#[test]
fn soft_target_lands_near_target() {
    let img = textured(64, 64, 0x7A26);
    for transform in [false, true] {
        let base = if transform {
            transform_opts(4)
        } else {
            strip_opts(4)
        };
        let free_len = encode_icer(&img, &base).unwrap().len() as u64;
        for target in [400u64, 900, 1600] {
            let bytes = encode_icer(&img, &base.clone().with_target_bytes(target)).unwrap();
            let len = bytes.len() as u64;
            assert!(
                len >= target.min(free_len),
                "target {target} (transform={transform}): output {len} fell short"
            );
            // The finish-the-step overshoot is bounded by one §VI.B
            // step, which on this fixture is far below the lossless
            // total; a runaway overshoot means the stop key is broken.
            assert!(
                len < free_len || free_len <= target,
                "target {target} (transform={transform}): output {len} ran to lossless {free_len}"
            );
            let dec = parse_icer(&bytes).unwrap();
            assert_eq!((dec.width, dec.height), (64, 64));
        }
    }
}

/// The §VI.B quota composes with §III.A priority interleaving, the
/// §IV interleaved entropy backend, and §VI.A minimum loss: budgets
/// hold, geometry frames, and quality stays monotone.
#[test]
fn quota_composes_with_other_modes() {
    let img = textured(64, 64, 0xC0DE);
    let variants: Vec<EncodeOptions> = vec![
        strip_opts(4).with_priority_interleaving(),
        strip_opts(4).with_interleaved_entropy(),
        strip_opts(3).with_min_loss(2),
        transform_opts(4).with_priority_interleaving(),
        transform_opts(4).with_interleaved_entropy(),
        transform_opts(4).with_min_loss(2),
    ];
    for base in variants {
        let mut prev = 0.0f64;
        for budget in [600u64, 1200, 2400] {
            let bytes = encode_icer(&img, &base.clone().with_byte_budget(budget)).unwrap();
            assert!(
                bytes.len() as u64 <= budget,
                "budget {budget} exceeded: {} (transform={} prio={} ixec={} M={})",
                bytes.len(),
                base.transform_segments,
                base.priority_interleaving,
                base.interleaved_entropy,
                base.min_loss
            );
            let dec = parse_icer(&bytes).unwrap();
            assert_eq!((dec.width, dec.height), (64, 64));
            let p = psnr(&img, &dec);
            assert!(
                p >= prev - 0.1,
                "budget {budget}: {p:.2} dB regressed from {prev:.2} \
                 (transform={} prio={} ixec={} M={})",
                base.transform_segments,
                base.priority_interleaving,
                base.interleaved_entropy,
                base.min_loss
            );
            prev = p;
        }
    }
}

/// ROI segment priorities keep their sequential whole-segment
/// scheduling — the §VI.B interleave must NOT override the documented
/// centre-first starvation semantics.
#[test]
fn roi_priorities_keep_sequential_scheduling() {
    let img = textured(64, 64, 0x2015);
    let opts = strip_opts(4).with_center_roi().with_byte_budget(900);
    let bytes = encode_icer(&img, &opts).unwrap();
    assert!(bytes.len() as u64 <= 900);
    let meta = parse_icer_metadata(&bytes).unwrap();
    assert_eq!(meta.segments.len(), 4);
    // Centre strips (ranks 0/1) must carry packets; with a 900-byte
    // budget the whole-segment greedy scheduler cannot fit all four,
    // so at least one periphery strip is a zero-packet placeholder.
    // The ROI scheduler is whole-segment all-or-nothing in rank order
    // (a segment whose full encode misses the residual budget is
    // skipped entirely). Under the §VI.B interleave every segment
    // would instead receive a partial packet prefix — so the signature
    // of the preserved ROI semantics is a mix of fully-kept and
    // zero-packet segments, never four partial ones.
    let by_index: Vec<usize> = meta.segments.iter().map(|s| s.packet_count).collect();
    assert!(
        by_index.contains(&0),
        "expected a starved strip under the whole-segment ROI scheduler: {by_index:?}"
    );
    assert!(
        by_index.iter().any(|&c| c >= 8),
        "expected a fully-kept strip under the whole-segment ROI scheduler: {by_index:?}"
    );
}
