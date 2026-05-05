//! Bit-exact round-trip cover for the integer 5/3 wavelet — both
//! 1-D and dyadic 2-D paths.

use oxideav_icer::wavelet::{
    deinterleave_subbands, forward_53_1d, forward_53_2d_one_level, forward_53_dyadic,
    inverse_53_1d, inverse_53_2d_one_level, inverse_53_dyadic,
};

fn deterministic_signal(n: usize, seed: u64) -> Vec<i32> {
    // Tiny LCG so the test is reproducible without bringing in `rand`.
    let mut s = seed.wrapping_mul(0x9E37_79B9_7F4A_7C15);
    (0..n)
        .map(|_| {
            s = s
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            (s >> 33) as i32 - 0x4000_0000
        })
        .map(|x| x % 256)
        .collect()
}

#[test]
fn forward_inverse_53_1d_is_identity() {
    for n in [2usize, 3, 4, 7, 8, 15, 16, 17, 32, 33, 64] {
        let original = deterministic_signal(n, n as u64);
        let mut buf = original.clone();
        forward_53_1d(&mut buf);
        inverse_53_1d(&mut buf);
        assert_eq!(buf, original, "n={n} round-trip failed");
    }
}

#[test]
fn forward_inverse_53_2d_one_level_is_identity() {
    for &(w, h) in &[(2usize, 2), (4, 4), (8, 8), (15, 9), (33, 17), (64, 64)] {
        let mut buf = deterministic_signal(w * h, (w * h) as u64);
        let original = buf.clone();
        forward_53_2d_one_level(&mut buf, w, h);
        inverse_53_2d_one_level(&mut buf, w, h);
        assert_eq!(buf, original, "geometry {w}x{h} round-trip failed");
    }
}

#[test]
fn forward_inverse_53_dyadic_is_identity() {
    for levels in 1u8..=5 {
        for &(w, h) in &[(8usize, 8), (16, 16), (32, 32), (33, 17), (64, 48)] {
            let mut buf = deterministic_signal(w * h, w as u64 * h as u64 + levels as u64);
            let original = buf.clone();
            forward_53_dyadic(&mut buf, w, h, levels);
            inverse_53_dyadic(&mut buf, w, h, levels);
            assert_eq!(buf, original, "{w}x{h} levels={levels} round-trip failed");
        }
    }
}

#[test]
fn deinterleave_separates_quadrants() {
    // One-level transform of an 8x8 ramp, then check that the LL block
    // (top-left 4x4 of the quadrant layout) sums to the average DC of
    // the input scaled by the lifting filter's DC gain (4).
    let w = 8;
    let h = 8;
    let mut buf: Vec<i32> = (0..(w * h) as i32).collect();
    forward_53_2d_one_level(&mut buf, w, h);
    let layout = deinterleave_subbands(&buf, w, h);
    // The LL quadrant (top-left 4x4) should be non-zero (averaged
    // values of the ramp); HH should be near zero (high-frequency
    // content of a smooth ramp is small).
    let mut ll_sum = 0i64;
    for y in 0..4 {
        for x in 0..4 {
            ll_sum += layout[y * w + x] as i64;
        }
    }
    let mut hh_sum_abs = 0i64;
    for y in 4..8 {
        for x in 4..8 {
            hh_sum_abs += (layout[y * w + x] as i64).abs();
        }
    }
    assert!(
        ll_sum > 0,
        "LL quadrant sum should be positive (got {ll_sum})"
    );
    // HH energy should be much smaller than LL for a smooth ramp.
    assert!(
        hh_sum_abs < ll_sum.unsigned_abs() as i64,
        "HH abs-sum ({hh_sum_abs}) should be < |LL sum| ({ll_sum})"
    );
}
