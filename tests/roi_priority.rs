//! Region-of-interest (ROI) segment-priority tests (round 6).
//!
//! IPN 42-155 §III.E notes that ICER's segment partitioning gives the
//! encoder freedom to schedule segments independently. Round 6 adds
//! [`EncodeOptions::with_segment_priorities`] and the
//! [`EncodeOptions::with_center_roi`] convenience, so callers can
//! reorder the on-the-wire emission of segments while leaving the
//! decoder's stitch-by-segment-index behaviour untouched.
//!
//! Test coverage:
//!
//!   * `with_center_roi` produces the expected centre-out permutations
//!     for several segment counts.
//!   * Permuted-emission encode + decode round-trips bit-exactly for
//!     filter Q (lossless) when no byte budget is set.
//!   * Under a tight byte budget, the centre-priority encode preserves
//!     the centre strip's fidelity while the trailing edges may be
//!     truncated.
//!   * Invalid priority vectors (wrong length, out-of-range entry,
//!     duplicate rank) are rejected at encode time.

use oxideav_icer::{
    encode_icer, parse_icer, parse_icer_metadata, EncodeOptions, IcerImage, IcerPixelFormat,
};

/// Build a 256x128 8-bit gray image whose top, middle, and bottom
/// strips have distinctive content. The middle strip carries a
/// high-frequency diagonal pattern; the top + bottom strips are flat.
/// This lets ROI tests verify that the "centre" priority preserves the
/// middle content under a tight byte budget.
fn striped_test_image(w: u32, h: u32) -> IcerImage {
    let mut img = IcerImage::zeros(w, h, IcerPixelFormat::Gray8);
    let stride = img.planes[0].stride;
    let mid_lo = (h / 3) as usize;
    let mid_hi = (2 * h / 3) as usize;
    for y in 0..h as usize {
        for x in 0..w as usize {
            let v = if y < mid_lo {
                32u8
            } else if y < mid_hi {
                // High-frequency diagonal in the middle band.
                (((x as u32 + y as u32) * 13) & 0xFF) as u8
            } else {
                200u8
            };
            img.planes[0].data[y * stride + x] = v;
        }
    }
    img
}

#[test]
fn with_center_roi_orders_segments_outward() {
    // segment_count = 5 -> middle is index 2 (mid = (5-1)/2 = 2)
    // priorities[seg_idx]: seg 2 -> 0, seg 1 -> 1, seg 3 -> 2,
    //                      seg 0 -> 3, seg 4 -> 4.
    let opts = EncodeOptions::compressed();
    let opts = EncodeOptions {
        segment_count: 5,
        ..opts
    }
    .with_center_roi();
    assert_eq!(
        opts.segment_priorities.as_deref(),
        Some([3u16, 1, 0, 2, 4].as_slice())
    );
}

#[test]
fn with_center_roi_orders_segments_outward_even() {
    // segment_count = 4 -> mid = (4-1)/2 = 1
    // priorities[seg_idx]: seg 1 -> 0, seg 0 -> 1, seg 2 -> 2, seg 3 -> 3.
    let opts = EncodeOptions::compressed();
    let opts = EncodeOptions {
        segment_count: 4,
        ..opts
    }
    .with_center_roi();
    assert_eq!(
        opts.segment_priorities.as_deref(),
        Some([1u16, 0, 2, 3].as_slice())
    );
}

#[test]
fn with_center_roi_single_segment_is_noop() {
    let opts = EncodeOptions::compressed().with_center_roi();
    // segment_count default is 1 -> priorities vector of length 1.
    assert_eq!(opts.segment_priorities.as_deref(), Some([0u16].as_slice()));
}

#[test]
fn permuted_emission_roundtrips_bit_exact_filter_q() {
    // Filter Q is lossless, so a multi-segment permuted emission
    // must still round-trip pixel-exact.
    let image = striped_test_image(64, 64);

    // Baseline: index-order emission.
    let opts_baseline = EncodeOptions {
        segment_count: 4,
        ..EncodeOptions::compressed()
    };
    let encoded_baseline = encode_icer(&image, &opts_baseline).expect("baseline encode");
    let decoded_baseline = parse_icer(&encoded_baseline).expect("baseline decode");
    assert_eq!(
        decoded_baseline.planes[0].data, image.planes[0].data,
        "baseline (index-order) must be bit-exact for filter Q"
    );

    // Centre-out priorities.
    let opts_centre = EncodeOptions {
        segment_count: 4,
        ..EncodeOptions::compressed()
    }
    .with_center_roi();
    let encoded_centre = encode_icer(&image, &opts_centre).expect("centre encode");
    let decoded_centre = parse_icer(&encoded_centre).expect("centre decode");
    assert_eq!(
        decoded_centre.planes[0].data, image.planes[0].data,
        "centre-priority emission must be bit-exact for filter Q"
    );

    // Reverse priorities (priority vector [3, 2, 1, 0] -> segment 3 first).
    let opts_reverse = EncodeOptions {
        segment_count: 4,
        ..EncodeOptions::compressed()
    }
    .with_segment_priorities(vec![3, 2, 1, 0]);
    let encoded_reverse = encode_icer(&image, &opts_reverse).expect("reverse encode");
    let decoded_reverse = parse_icer(&encoded_reverse).expect("reverse decode");
    assert_eq!(
        decoded_reverse.planes[0].data, image.planes[0].data,
        "reverse-priority emission must be bit-exact for filter Q"
    );
}

#[test]
fn permuted_emission_writes_segments_in_priority_order() {
    // Verify the on-the-wire ordering: with priorities [3, 2, 1, 0],
    // the first segment in the byte stream should carry segment_index = 3.
    let image = striped_test_image(64, 64);
    let opts = EncodeOptions {
        segment_count: 4,
        ..EncodeOptions::compressed()
    }
    .with_segment_priorities(vec![3, 2, 1, 0]);
    let encoded = encode_icer(&image, &opts).expect("encode");

    let meta = parse_icer_metadata(&encoded).expect("metadata parse");
    assert_eq!(meta.segments.len(), 4, "expected 4 segments");

    let on_wire_indices: Vec<u16> = meta
        .segments
        .iter()
        .map(|s| s.header.segment_index)
        .collect();
    // Priority vector [3, 2, 1, 0] means:
    //   seg 0 -> rank 3, seg 1 -> rank 2, seg 2 -> rank 1, seg 3 -> rank 0.
    // Rank order in stream: rank 0 first = seg 3, then 2, then 1, then 0.
    assert_eq!(
        on_wire_indices,
        vec![3, 2, 1, 0],
        "stream order should match the rank-ascending order of priorities"
    );
}

#[test]
fn center_roi_preserves_centre_under_tight_budget() {
    // 4 segments, centre-out priorities. Under a tight budget the
    // top + bottom strips (priorities 2 + 3) get truncated; the
    // centre strips (priorities 0 + 1) get full bit-planes.
    // Decode and check that the centre region differs less from the
    // original than the periphery does.
    let h = 128u32;
    let w = 64u32;
    let image = striped_test_image(w, h);
    // Budget chosen so the top-priority centre strip fits (internally
    // truncated) but nothing else does: the r405 §II.B pyramid fix
    // moved the per-packet boundaries, so the centre strip's truncated
    // encode is ~875 B on this fixture (measured); 900 keeps exactly
    // segment 1 and drops the rest to placeholders.
    let budget = 900u64;
    let opts = EncodeOptions {
        segment_count: 4,
        ..EncodeOptions::compressed()
    }
    .with_center_roi()
    .with_byte_budget(budget);
    let encoded = encode_icer(&image, &opts).expect("encode");
    assert!(
        encoded.len() as u64 <= budget,
        "encoded {} bytes exceeds budget {}",
        encoded.len(),
        budget
    );

    let decoded = parse_icer(&encoded).expect("decode");
    assert_eq!(decoded.width, w);
    assert_eq!(decoded.height, h);

    // Recover which segment_index values made it into the stream.
    let meta = parse_icer_metadata(&encoded).expect("metadata");
    let kept: std::collections::BTreeSet<u16> = meta
        .segments
        .iter()
        .map(|s| s.header.segment_index)
        .collect();
    // Centre segments (indices 1 and 2 -- mid = (4-1)/2 = 1, priorities
    // [1, 0, 2, 3]) should be present.
    assert!(
        kept.contains(&1) || kept.contains(&2),
        "at least one centre segment should be kept; got {:?}",
        kept
    );

    // The dropped-segment rows decode to 128 (level-shifted zero).
    // Verify that the centre row(s) closely match the original
    // (significantly closer than the dropped rows).
    let strip_h = h.div_ceil(4) as usize;
    // The centre band straddles rows in segments 1 and 2.
    let centre_row = strip_h + (strip_h / 2);
    let mut centre_err = 0u64;
    let mut periphery_err = 0u64;
    let mut centre_n = 0u64;
    let mut periphery_n = 0u64;
    let stride = image.planes[0].stride;
    for y in 0..h as usize {
        let in_centre_band = y >= strip_h && y < 3 * strip_h;
        let _ = centre_row;
        for x in 0..w as usize {
            let a = image.planes[0].data[y * stride + x] as i32;
            let b = decoded.planes[0].data[y * stride + x] as i32;
            let d = (a - b).unsigned_abs() as u64;
            if in_centre_band {
                centre_err += d;
                centre_n += 1;
            } else {
                periphery_err += d;
                periphery_n += 1;
            }
        }
    }
    // Mean abs error in the centre band should be smaller than in the
    // periphery, since the centre is what we prioritised.
    let centre_mae = centre_err as f64 / centre_n as f64;
    let periphery_mae = periphery_err as f64 / periphery_n as f64;
    eprintln!(
        "center_roi_under_budget: centre MAE={:.2} periphery MAE={:.2}",
        centre_mae, periphery_mae
    );
    assert!(
        centre_mae <= periphery_mae,
        "centre MAE ({centre_mae:.2}) should be <= periphery MAE ({periphery_mae:.2}) under ROI prioritisation"
    );
}

#[test]
fn invalid_priorities_wrong_length_rejected() {
    let image = striped_test_image(16, 16);
    let opts = EncodeOptions {
        segment_count: 4,
        ..EncodeOptions::compressed()
    }
    .with_segment_priorities(vec![0, 1, 2]); // length 3 != segment_count 4
    let err = encode_icer(&image, &opts).expect_err("should reject wrong-length priorities");
    let msg = format!("{err}");
    assert!(
        msg.contains("segment_priorities length"),
        "expected length-mismatch error, got: {msg}"
    );
}

#[test]
fn invalid_priorities_out_of_range_rejected() {
    let image = striped_test_image(16, 16);
    let opts = EncodeOptions {
        segment_count: 4,
        ..EncodeOptions::compressed()
    }
    .with_segment_priorities(vec![0, 1, 2, 7]); // 7 >= 4
    let err = encode_icer(&image, &opts).expect_err("should reject out-of-range priority");
    let msg = format!("{err}");
    assert!(
        msg.contains("out of range"),
        "expected out-of-range error, got: {msg}"
    );
}

#[test]
fn invalid_priorities_duplicate_rank_rejected() {
    let image = striped_test_image(16, 16);
    let opts = EncodeOptions {
        segment_count: 4,
        ..EncodeOptions::compressed()
    }
    .with_segment_priorities(vec![0, 0, 1, 2]); // rank 0 twice
    let err = encode_icer(&image, &opts).expect_err("should reject duplicate priority");
    let msg = format!("{err}");
    assert!(
        msg.contains("more than once"),
        "expected duplicate error, got: {msg}"
    );
}

#[test]
fn priorities_compose_with_uncompressed_path() {
    // Uncompressed multi-segment with priorities should still round-trip.
    let image = striped_test_image(32, 16);
    let opts = EncodeOptions {
        segment_count: 4,
        uncompressed: true,
        ..EncodeOptions::default()
    }
    .with_segment_priorities(vec![3, 2, 1, 0]);
    let encoded = encode_icer(&image, &opts).expect("encode");

    // Wire-order should be segment indices [3, 2, 1, 0].
    let meta = parse_icer_metadata(&encoded).expect("metadata");
    let on_wire: Vec<u16> = meta
        .segments
        .iter()
        .map(|s| s.header.segment_index)
        .collect();
    assert_eq!(on_wire, vec![3, 2, 1, 0]);

    // Decoder stitches by index, so output must match input.
    let decoded = parse_icer(&encoded).expect("decode");
    assert_eq!(decoded.planes[0].data, image.planes[0].data);
}
