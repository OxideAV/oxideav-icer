//! Spec-exact reversible integer wavelet transform for all seven ICER
//! filters (A, B, C, D, E, F, Q) -- IPN 42-155 §II.A, equations (1)-(3)
//! and the per-filter parameter table.
//!
//! IPN 42-155 §II "Wavelet Transform" specifies that an ICER user can
//! select one of *seven* reversible integer wavelet transforms, all
//! computed the same way but differing in their filter parameters
//! `(alpha_{-1}, alpha_0, alpha_1, beta)`. Every one of the seven is a
//! reversible integer-to-integer transform: lossless compression is
//! achieved when all transform outputs are losslessly entropy-coded,
//! independent of which filter is chosen (§I, §II.A).
//!
//! This module transcribes §II.A directly. For a length-`N` (`N >= 3`)
//! input sequence `x[0..N]`, a single 1-D stage produces `ceil(N/2)`
//! low-pass outputs `l[n]` and `floor(N/2)` high-pass outputs `h[n]`.
//!
//! Low-pass (§II.A):
//!
//! ```text
//!     l[n] = floor( (x[2n] + x[2n+1]) / 2 ),   n = 0 .. floor(N/2) - 1
//!     l[(N-1)/2] = x[N-1],                      N odd  (the last l)
//! ```
//!
//! High-pass, equation (1) defines the raw difference `d`, equation (2)
//! the running low-pass difference `r`, and equation (3) the final
//! high-pass output `h`:
//!
//! ```text
//!     d[n] = x[2n] - x[2n+1],   n = 0 .. ceil(N/2) - 2
//!     d[(N-1)/2] = 0,           N odd                                (1)
//!
//!     r[n] = l[n-1] - l[n],     n = 1 .. ceil(N/2) - 1               (2)
//!
//!     h[n] = d[n] - (predictor), with the predictor selected by n:   (3)
//!         n = 0:                       floor( (1/4) r[1] )
//!         n = 1, alpha_{-1} != 0:      floor( (1/4)r[1] + (3/8)r[2]
//!                                              - (1/4)d[2] + 1/2 )
//!         N even, n = N/2 - 1:         floor( (1/4) r[N/2 - 1] )
//!         otherwise:                   floor( alpha_{-1} r[n-1]
//!                                              + alpha_0 r[n]
//!                                              + alpha_1 r[n+1]
//!                                              - beta d[n+1] + 1/2 )
//! ```
//!
//! The per-filter `(alpha_{-1}, alpha_0, alpha_1, beta)` values come
//! from IPN 42-155 §II.A Table 1.
//!
//! The inverse (§II.A): recompute `r[n]` from the stored `l[n]` using
//! equation (2), invert equation (3) **in order of decreasing `n`** to
//! recover `d[n]`, then recover the samples:
//!
//! ```text
//!     x[2n]   = l[n] + floor( (d[n] + 1) / 2 )
//!     x[2n+1] = x[2n] - d[n]
//! ```
//!
//! Every step uses only integer additions, subtractions, and
//! floor-divisions, so the transform maps integers to integers exactly
//! and is exactly invertible -- the basis for ICER's lossless mode.

use crate::header::WaveletFilter;

/// Exact rational filter parameters `(alpha_{-1}, alpha_0, alpha_1,
/// beta)` for one ICER wavelet filter, scaled so that every coefficient
/// is an integer numerator over the shared denominator [`DENOM`].
///
/// IPN 42-155 §II.A Table 1 lists the parameters as rationals with
/// denominators of 4, 8, or 16. We promote them all to a common
/// denominator of 32 so the equation-(3) predictor -- a sum of four
/// coefficient-weighted terms plus the `+1/2` rounding offset -- can be
/// evaluated with a single floor-division by 32, preserving the
/// floor-after-summation order the paper specifies.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IntFilterParams {
    /// `alpha_{-1}` numerator over [`DENOM`].
    pub a_m1: i64,
    /// `alpha_0` numerator over [`DENOM`].
    pub a_0: i64,
    /// `alpha_1` numerator over [`DENOM`].
    pub a_1: i64,
    /// `beta` numerator over [`DENOM`].
    pub b: i64,
}

/// Shared denominator for the scaled coefficients in [`IntFilterParams`].
/// Chosen as 32 because it is the least common multiple of every Table 1
/// denominator (4, 8, 16) together with the `2` from the `+1/2`
/// equation-(3) rounding offset.
pub const DENOM: i64 = 32;

/// IPN 42-155 §II.A Table 1, scaled to numerators over [`DENOM`] = 32.
///
/// | Filter | alpha_{-1} | alpha_0 | alpha_1 | beta |
/// |--------|-----------:|--------:|--------:|-----:|
/// | A      |          0 |     1/4 |     1/4 |    0 |
/// | B      |          0 |     2/8 |     3/8 |  2/8 |
/// | C      |      -1/16 |    4/16 |    8/16 | 6/16 |
/// | D      |          0 |    4/16 |    5/16 | 2/16 |
/// | E      |          0 |    3/16 |    8/16 | 6/16 |
/// | F      |          0 |    3/16 |    9/16 | 8/16 |
/// | Q      |          0 |     1/4 |     1/4 |  1/4 |
///
/// Indexed by [`WaveletFilter`] discriminant (`0 = Q`, `1 = A`, ...,
/// `6 = F`). Filter Q shares filter A's `alpha` triple and differs only
/// in `beta = 1/4` (filter A has `beta = 0`).
pub const INT_FILTER_PARAMS: [IntFilterParams; 7] = [
    // Q (id = 0): (0, 1/4, 1/4, 1/4) -> (0, 8, 8, 8) / 32.
    IntFilterParams {
        a_m1: 0,
        a_0: 8,
        a_1: 8,
        b: 8,
    },
    // A (id = 1): (0, 1/4, 1/4, 0) -> (0, 8, 8, 0) / 32.
    IntFilterParams {
        a_m1: 0,
        a_0: 8,
        a_1: 8,
        b: 0,
    },
    // B (id = 2): (0, 2/8, 3/8, 2/8) -> (0, 8, 12, 8) / 32.
    IntFilterParams {
        a_m1: 0,
        a_0: 8,
        a_1: 12,
        b: 8,
    },
    // C (id = 3): (-1/16, 4/16, 8/16, 6/16) -> (-2, 8, 16, 12) / 32.
    IntFilterParams {
        a_m1: -2,
        a_0: 8,
        a_1: 16,
        b: 12,
    },
    // D (id = 4): (0, 4/16, 5/16, 2/16) -> (0, 8, 10, 4) / 32.
    IntFilterParams {
        a_m1: 0,
        a_0: 8,
        a_1: 10,
        b: 4,
    },
    // E (id = 5): (0, 3/16, 8/16, 6/16) -> (0, 6, 16, 12) / 32.
    IntFilterParams {
        a_m1: 0,
        a_0: 6,
        a_1: 16,
        b: 12,
    },
    // F (id = 6): (0, 3/16, 9/16, 8/16) -> (0, 6, 18, 16) / 32.
    IntFilterParams {
        a_m1: 0,
        a_0: 6,
        a_1: 18,
        b: 16,
    },
];

impl WaveletFilter {
    /// Look up the spec-exact integer-filter parameters for this filter
    /// id (IPN 42-155 §II.A Table 1).
    pub fn int_params(self) -> IntFilterParams {
        INT_FILTER_PARAMS[self as usize]
    }
}

/// Floor division `a / b` for `b > 0`, valid for negative `a`
/// (`i64::div_euclid` gives the floor when the divisor is positive).
#[inline]
fn floor_div(a: i64, b: i64) -> i64 {
    a.div_euclid(b)
}

/// One spec-exact forward 1-D reversible integer stage on `x[0..N]`,
/// producing low-pass outputs in `l` (length `ceil(N/2)`) and high-pass
/// outputs in `h` (length `floor(N/2)`). IPN 42-155 §II.A eqs (1)-(3).
///
/// `N >= 3` is required by the paper (the predictor in eq (3) references
/// `r[n-1]`, `r[n]`, `r[n+1]`). Shorter inputs are handled by the
/// caller's boundary policy and never reach this function.
pub fn forward_1d(x: &[i32], p: &IntFilterParams, l: &mut Vec<i32>, h: &mut Vec<i32>) {
    let n = x.len();
    debug_assert!(n >= 3, "spec eq (3) requires N >= 3");
    let n_lo = n.div_ceil(2); // ceil(N/2) low-pass outputs
    let n_hi = n / 2; // floor(N/2) high-pass outputs

    l.clear();
    h.clear();
    l.resize(n_lo, 0);
    h.resize(n_hi, 0);

    // Low-pass: l[n] = floor((x[2n] + x[2n+1]) / 2), and for odd N the
    // final low-pass output is l[(N-1)/2] = x[N-1].
    for k in 0..n_hi {
        let s = x[2 * k] as i64 + x[2 * k + 1] as i64;
        l[k] = floor_div(s, 2) as i32;
    }
    if n % 2 == 1 {
        l[n_lo - 1] = x[n - 1];
    }

    // Equation (1): d[n] = x[2n] - x[2n+1] for the floor(N/2) full
    // pairs. For odd N the trailing d[(N-1)/2] = 0 (no high-pass output
    // is emitted for the lone sample, so it is implicit).
    let mut d = vec![0i64; n_hi];
    for k in 0..n_hi {
        d[k] = x[2 * k] as i64 - x[2 * k + 1] as i64;
    }

    // Equation (2): r[n] = l[n-1] - l[n], n = 1 .. ceil(N/2) - 1.
    // r[0] is unused by eq (3); we store it as 0 for index alignment.
    let mut r = vec![0i64; n_lo];
    for k in 1..n_lo {
        r[k] = l[k - 1] as i64 - l[k] as i64;
    }

    // Equation (3): h[n] = d[n] - predictor(n).
    for n_idx in 0..n_hi {
        let pred = predictor(n_idx, n_hi, n, p, &d, &r);
        h[n_idx] = (d[n_idx] - pred) as i32;
    }
}

/// One spec-exact inverse 1-D reversible integer stage. Given the stored
/// low-pass `l` (length `ceil(N/2)`) and high-pass `h` (length
/// `floor(N/2)`), reconstruct `x[0..N]` exactly. IPN 42-155 §II.A.
pub fn inverse_1d(l: &[i32], h: &[i32], n: usize, p: &IntFilterParams, x: &mut Vec<i32>) {
    debug_assert!(n >= 3, "spec eq (3) requires N >= 3");
    let n_lo = n.div_ceil(2);
    let n_hi = n / 2;
    debug_assert_eq!(l.len(), n_lo);
    debug_assert_eq!(h.len(), n_hi);

    x.clear();
    x.resize(n, 0);

    // Recompute r[n] from the stored low-pass outputs (eq (2)).
    let mut r = vec![0i64; n_lo];
    for k in 1..n_lo {
        r[k] = l[k - 1] as i64 - l[k] as i64;
    }

    // Invert eq (3) in order of decreasing n: d[n] = h[n] + predictor(n).
    // The predictor for index n references d[n+1] (in the "otherwise"
    // and "n = 1" branches), which is why the recovery proceeds from the
    // highest index downward -- d[n+1] is already known when computing
    // d[n].
    let mut d = vec![0i64; n_hi];
    for n_idx in (0..n_hi).rev() {
        let pred = predictor(n_idx, n_hi, n, p, &d, &r);
        d[n_idx] = h[n_idx] as i64 + pred;
    }

    // Recover the samples: x[2n] = l[n] + floor((d[n] + 1) / 2);
    // x[2n+1] = x[2n] - d[n].
    for k in 0..n_hi {
        let x2n = l[k] as i64 + floor_div(d[k] + 1, 2);
        x[2 * k] = x2n as i32;
        x[2 * k + 1] = (x2n - d[k]) as i32;
    }
    if n % 2 == 1 {
        // Odd N: the lone trailing sample equals its low-pass output.
        x[n - 1] = l[n_lo - 1];
    }
}

/// Equation (3) predictor for high-pass index `n_idx`, shared by the
/// forward and inverse paths (the inverse adds it back; the forward
/// subtracts it). `n_hi = floor(N/2)` is the high-pass output count and
/// `n` is the original sequence length (needed to distinguish the
/// even-`N` tail case).
///
/// The four branches mirror eq (3) verbatim:
/// * `n = 0`            -> `floor((1/4) r[1])`
/// * `n = 1`, `a_m1 != 0` -> `floor((1/4)r[1] + (3/8)r[2] - (1/4)d[2] + 1/2)`
/// * even `N`, `n = N/2 - 1` -> `floor((1/4) r[N/2 - 1])`
/// * otherwise          -> `floor(a_m1 r[n-1] + a_0 r[n] + a_1 r[n+1]
///                                   - b d[n+1] + 1/2)`
fn predictor(
    n_idx: usize,
    n_hi: usize,
    n: usize,
    p: &IntFilterParams,
    d: &[i64],
    r: &[i64],
) -> i64 {
    let n_even = n % 2 == 0;
    if n_idx == 0 {
        // floor((1/4) r[1]) = floor(r[1] / 4).
        floor_div(r[1], 4)
    } else if n_even && n_idx == n_hi - 1 {
        // floor((1/4) r[N/2 - 1]) = floor(r[n_hi - 1] / 4).
        //
        // The even-`N` boundary takes precedence over the `n = 1`
        // special case when they coincide (`N = 4`, where
        // `n_hi - 1 == 1`): the trailing high-pass output has no
        // `r[n+1]` / `d[n+1]` to reference, so the tail predictor
        // applies.
        floor_div(r[n_hi - 1], 4)
    } else if n_idx == 1 && p.a_m1 != 0 {
        // floor((1/4)r[1] + (3/8)r[2] - (1/4)d[2] + 1/2).
        // Common denominator 8: (2 r[1] + 3 r[2] - 2 d[2] + 4) / 8.
        // `d[2]` follows the same trailing-zero rule as the generic
        // branch (eq (1): the implicit `d[(N-1)/2] = 0` for odd `N`).
        let d2 = if 2 < d.len() { d[2] } else { 0 };
        let num = 2 * r[1] + 3 * r[2] - 2 * d2 + 4;
        floor_div(num, 8)
    } else {
        // floor(a_m1 r[n-1] + a_0 r[n] + a_1 r[n+1] - b d[n+1] + 1/2).
        // All coefficients are scaled to /DENOM (= 32); the +1/2 offset
        // is DENOM/2 = 16. A single floor-division by DENOM evaluates
        // the whole predictor with the spec's floor-after-summation
        // order.
        //
        // For odd `N` the trailing difference `d[(N-1)/2] = 0` (eq (1));
        // that index is `d[n_hi]`, which is not materialised in the
        // length-`n_hi` `d` buffer, so a `d[n+1]` reference at the upper
        // boundary reads as 0. The `r` buffer has length `ceil(N/2)`, so
        // for odd `N` the `r[n+1]` reference is always in range.
        let d_np1 = if n_idx + 1 < d.len() { d[n_idx + 1] } else { 0 };
        let num = p.a_m1 * r[n_idx - 1] + p.a_0 * r[n_idx] + p.a_1 * r[n_idx + 1] - p.b * d_np1
            + DENOM / 2;
        floor_div(num, DENOM)
    }
}

/// In-place spec-exact forward 1-D stage written back into `row` in the
/// interleaved layout the rest of the crate uses (even indices hold the
/// low-pass outputs `l[k]`, odd indices the high-pass outputs `h[k]`),
/// matching [`crate::wavelet::forward_53_1d`]. `n < 3` is a no-op: the
/// §II.A predictor needs at least three samples, and the crate's
/// boundary policy never decomposes a region below that.
pub fn forward_1d_interleaved(row: &mut [i32], p: &IntFilterParams) {
    let n = row.len();
    if n < 3 {
        return;
    }
    let (mut l, mut h) = (Vec::new(), Vec::new());
    forward_1d(row, p, &mut l, &mut h);
    for (k, &v) in l.iter().enumerate() {
        row[2 * k] = v;
    }
    for (k, &v) in h.iter().enumerate() {
        row[2 * k + 1] = v;
    }
}

/// Exact inverse of [`forward_1d_interleaved`]: de-interleave the
/// even/odd layout back into `l`/`h`, invert the §II.A stage, and write
/// the reconstructed samples back.
pub fn inverse_1d_interleaved(row: &mut [i32], p: &IntFilterParams) {
    let n = row.len();
    if n < 3 {
        return;
    }
    let n_lo = n.div_ceil(2);
    let n_hi = n / 2;
    let mut l = vec![0i32; n_lo];
    let mut h = vec![0i32; n_hi];
    for (k, slot) in l.iter_mut().enumerate() {
        *slot = row[2 * k];
    }
    for (k, slot) in h.iter_mut().enumerate() {
        *slot = row[2 * k + 1];
    }
    let mut back = Vec::new();
    inverse_1d(&l, &h, n, p, &mut back);
    row.copy_from_slice(&back);
}

/// One spec-exact forward 2-D stage (rows then columns) on the top-left
/// `sub_w x sub_h` sub-rectangle of a buffer with row stride `stride`,
/// in interleaved layout. IPN 42-155 §II.B: each 2-D stage applies the
/// 1-D transform to every row, then to every column of the result.
fn forward_2d_one_level_sub(
    buf: &mut [i32],
    stride: usize,
    sub_w: usize,
    sub_h: usize,
    p: &IntFilterParams,
) {
    for y in 0..sub_h {
        forward_1d_interleaved(&mut buf[y * stride..y * stride + sub_w], p);
    }
    let mut col = vec![0i32; sub_h];
    for x in 0..sub_w {
        for y in 0..sub_h {
            col[y] = buf[y * stride + x];
        }
        forward_1d_interleaved(&mut col, p);
        for y in 0..sub_h {
            buf[y * stride + x] = col[y];
        }
    }
}

/// Inverse of [`forward_2d_one_level_sub`]. IPN 42-155 §II.B: a 2-D
/// stage is inverted in reverse order -- columns first, then rows.
fn inverse_2d_one_level_sub(
    buf: &mut [i32],
    stride: usize,
    sub_w: usize,
    sub_h: usize,
    p: &IntFilterParams,
) {
    let mut col = vec![0i32; sub_h];
    for x in 0..sub_w {
        for y in 0..sub_h {
            col[y] = buf[y * stride + x];
        }
        inverse_1d_interleaved(&mut col, p);
        for y in 0..sub_h {
            buf[y * stride + x] = col[y];
        }
    }
    for y in 0..sub_h {
        inverse_1d_interleaved(&mut buf[y * stride..y * stride + sub_w], p);
    }
}

/// `D`-level dyadic forward 2-D reversible integer transform for any of
/// the seven ICER filters (IPN 42-155 §II.B pyramidal decomposition).
/// Each level transforms the current low-frequency (top-left) region and
/// then recurses on its `ceil(w/2) x ceil(h/2)` LL sub-band.
pub fn forward_2d_dyadic(
    buf: &mut [i32],
    width: usize,
    height: usize,
    levels: u8,
    filter: WaveletFilter,
) {
    debug_assert_eq!(buf.len(), width * height);
    let p = filter.int_params();
    let (mut w, mut h) = (width, height);
    for _ in 0..levels {
        if w < 3 || h < 3 {
            break;
        }
        forward_2d_one_level_sub(buf, width, w, h, &p);
        w = w.div_ceil(2);
        h = h.div_ceil(2);
    }
}

/// Exact inverse of [`forward_2d_dyadic`].
pub fn inverse_2d_dyadic(
    buf: &mut [i32],
    width: usize,
    height: usize,
    levels: u8,
    filter: WaveletFilter,
) {
    debug_assert_eq!(buf.len(), width * height);
    let p = filter.int_params();
    let mut sizes = Vec::with_capacity(levels as usize);
    let (mut w, mut h) = (width, height);
    for _ in 0..levels {
        if w < 3 || h < 3 {
            break;
        }
        sizes.push((w, h));
        w = w.div_ceil(2);
        h = h.div_ceil(2);
    }
    for (sw, sh) in sizes.into_iter().rev() {
        inverse_2d_one_level_sub(buf, width, sw, sh, &p);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const ALL: [WaveletFilter; 7] = [
        WaveletFilter::Reversible53,
        WaveletFilter::NineSevenA,
        WaveletFilter::FilterB,
        WaveletFilter::FilterC,
        WaveletFilter::FilterD,
        WaveletFilter::FilterE,
        WaveletFilter::FilterF,
    ];

    fn roundtrip(x: &[i32], f: WaveletFilter) {
        let p = f.int_params();
        let (mut l, mut h, mut back) = (Vec::new(), Vec::new(), Vec::new());
        forward_1d(x, &p, &mut l, &mut h);
        assert_eq!(l.len(), x.len().div_ceil(2));
        assert_eq!(h.len(), x.len() / 2);
        inverse_1d(&l, &h, x.len(), &p, &mut back);
        assert_eq!(back, x, "filter {f:?} 1-D round-trip mismatch");
    }

    #[test]
    fn reversible_roundtrip_all_filters_even_and_odd() {
        // Every one of the seven filters is a reversible integer
        // transform (IPN 42-155 §I, §II.A): forward then inverse must
        // recover the input bit-exactly, for both even and odd N.
        for f in ALL {
            for n in [4usize, 6, 8, 16, 32, 5, 7, 9, 17, 33] {
                let ramp: Vec<i32> = (0..n).map(|i| (i as i32 * 7) % 251 - 120).collect();
                roundtrip(&ramp, f);
            }
        }
    }

    #[test]
    fn reversible_roundtrip_extreme_and_constant() {
        for f in ALL {
            // Constant input -> high-pass should be all zero (perfect
            // prediction) and the transform must still invert exactly.
            let flat = vec![77i32; 16];
            roundtrip(&flat, f);

            // Large-magnitude alternating input stresses the dynamic
            // range of the difference and predictor terms.
            let alt: Vec<i32> = (0..16)
                .map(|i| if i % 2 == 0 { 30_000 } else { -30_000 })
                .collect();
            roundtrip(&alt, f);
        }
    }

    #[test]
    fn flat_input_has_zero_high_pass() {
        // A constant sequence has d[n] = 0 and r[n] = 0, so eq (3)
        // yields h[n] = 0 for every filter. This pins the predictor
        // arithmetic (any stray rounding bias would surface as a
        // non-zero high-pass coefficient on flat input).
        for f in ALL {
            let p = f.int_params();
            let (mut l, mut h) = (Vec::new(), Vec::new());
            forward_1d(&[42i32; 20], &p, &mut l, &mut h);
            assert!(
                h.iter().all(|&c| c == 0),
                "filter {f:?}: flat input must give zero high-pass, got {h:?}"
            );
            // Low-pass of a constant is that constant.
            assert!(l.iter().all(|&c| c == 42));
        }
    }

    #[test]
    fn interleaved_1d_roundtrip_all_filters() {
        // The even/odd interleaved wrapper must round-trip exactly too
        // (it is the layout the 2-D path and the rest of the crate use).
        for f in ALL {
            let p = f.int_params();
            for n in [4usize, 8, 16, 7, 33] {
                let original: Vec<i32> = (0..n).map(|i| (i as i32 * 13) % 200 - 90).collect();
                let mut buf = original.clone();
                forward_1d_interleaved(&mut buf, &p);
                inverse_1d_interleaved(&mut buf, &p);
                assert_eq!(buf, original, "filter {f:?} interleaved 1-D mismatch");
            }
        }
    }

    #[test]
    fn dyadic_2d_roundtrip_all_filters() {
        // Every filter is reversible, so the full multi-level 2-D
        // pyramidal decomposition (IPN 42-155 §II.B) must reconstruct
        // the image bit-exactly -- this is ICER's lossless mode for an
        // arbitrary filter choice, not just filter Q.
        for f in ALL {
            for &(w, h) in &[(8usize, 8usize), (16, 16), (12, 10), (17, 13)] {
                let original: Vec<i32> = (0..w * h).map(|i| (i as i32 * 31) % 255 - 128).collect();
                let mut buf = original.clone();
                forward_2d_dyadic(&mut buf, w, h, 3, f);
                inverse_2d_dyadic(&mut buf, w, h, 3, f);
                assert_eq!(buf, original, "filter {f:?} 2-D {w}x{h} mismatch");
            }
        }
    }

    #[test]
    fn dyadic_2d_levels_sweep_filter_a() {
        // Decomposition depth must not affect reversibility for 1..=5
        // levels.
        let (w, h) = (32usize, 32usize);
        let original: Vec<i32> = (0..w * h).map(|i| ((i * 7) % 256) as i32 - 128).collect();
        for levels in 1..=5u8 {
            let mut buf = original.clone();
            forward_2d_dyadic(&mut buf, w, h, levels, WaveletFilter::NineSevenA);
            inverse_2d_dyadic(&mut buf, w, h, levels, WaveletFilter::NineSevenA);
            assert_eq!(buf, original, "filter A {levels}-level 2-D mismatch");
        }
    }

    #[test]
    fn spec_q_is_reversible_independent_of_textbook_path() {
        // The crate's pre-existing `wavelet::forward_53_1d` implements a
        // textbook integer 5/3 lifting that is reversible but uses a
        // different rounding/boundary formulation than IPN 42-155 §II.A
        // eq (3). The two are NOT bit-identical in general; what matters
        // for ICER is that each is individually reversible. This test
        // pins that the §II.A filter-Q stage round-trips on the same
        // inputs the textbook path is tested with, so a future migration
        // of the pipeline onto the spec-exact path has a known-good
        // baseline.
        let p = WaveletFilter::Reversible53.int_params();
        for n in [4usize, 8, 16, 17] {
            let original: Vec<i32> = (0..n).map(|i| (i as i32) - (n as i32) / 2).collect();
            let mut buf = original.clone();
            forward_1d_interleaved(&mut buf, &p);
            inverse_1d_interleaved(&mut buf, &p);
            assert_eq!(buf, original);
        }
    }

    #[test]
    fn filter_q_and_a_share_alpha_differ_in_beta() {
        // Table 1 cross-check: Q and A have the same alpha triple; only
        // beta differs (Q = 1/4 -> 8/32, A = 0).
        let q = WaveletFilter::Reversible53.int_params();
        let a = WaveletFilter::NineSevenA.int_params();
        assert_eq!((q.a_m1, q.a_0, q.a_1), (a.a_m1, a.a_0, a.a_1));
        assert_eq!(q.b, 8);
        assert_eq!(a.b, 0);
    }
}
