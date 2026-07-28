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

/// One spec-exact forward 2-D stage (rows then columns) on a compact
/// `w x h` buffer in interleaved layout. IPN 42-155 §II.B: each 2-D
/// stage applies the 1-D transform to every row, then to every column
/// of the result.
fn forward_2d_one_level(buf: &mut [i32], w: usize, h: usize, p: &IntFilterParams) {
    for y in 0..h {
        forward_1d_interleaved(&mut buf[y * w..y * w + w], p);
    }
    let mut col = vec![0i32; h];
    for x in 0..w {
        for y in 0..h {
            col[y] = buf[y * w + x];
        }
        forward_1d_interleaved(&mut col, p);
        for y in 0..h {
            buf[y * w + x] = col[y];
        }
    }
}

/// Inverse of [`forward_2d_one_level`]. IPN 42-155 §II.B: a 2-D
/// stage is inverted in reverse order -- columns first, then rows.
fn inverse_2d_one_level(buf: &mut [i32], w: usize, h: usize, p: &IntFilterParams) {
    let mut col = vec![0i32; h];
    for x in 0..w {
        for y in 0..h {
            col[y] = buf[y * w + x];
        }
        inverse_1d_interleaved(&mut col, p);
        for y in 0..h {
            buf[y * w + x] = col[y];
        }
    }
    for y in 0..h {
        inverse_1d_interleaved(&mut buf[y * w..y * w + w], p);
    }
}

/// `D`-level dyadic forward 2-D reversible integer transform for any of
/// the seven ICER filters (IPN 42-155 §II.B pyramidal decomposition).
/// Each further stage decomposes the **LL subband** of the previous
/// stage — in the interleaved layout, the even/even lattice at stride
/// `2^j` (see [`crate::wavelet::forward_53_dyadic`] for the layout
/// contract and the CHANGELOG note on the earlier top-left-rectangle
/// recursion this replaces).
pub fn forward_2d_dyadic(
    buf: &mut [i32],
    width: usize,
    height: usize,
    levels: u8,
    filter: WaveletFilter,
) {
    debug_assert_eq!(buf.len(), width * height);
    let p = filter.int_params();
    let mut stride = 1usize;
    for _ in 0..levels {
        let sw = width.div_ceil(stride);
        let sh = height.div_ceil(stride);
        if sw < 3 || sh < 3 {
            break;
        }
        let mut sub = crate::wavelet::gather_lattice(buf, width, stride, sw, sh);
        forward_2d_one_level(&mut sub, sw, sh, &p);
        crate::wavelet::scatter_lattice(buf, width, stride, sw, sh, &sub);
        stride *= 2;
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
    let mut stages = Vec::with_capacity(levels as usize);
    let mut stride = 1usize;
    for _ in 0..levels {
        let sw = width.div_ceil(stride);
        let sh = height.div_ceil(stride);
        if sw < 3 || sh < 3 {
            break;
        }
        stages.push((stride, sw, sh));
        stride *= 2;
    }
    for (stride, sw, sh) in stages.into_iter().rev() {
        let mut sub = crate::wavelet::gather_lattice(buf, width, stride, sw, sh);
        inverse_2d_one_level(&mut sub, sw, sh, &p);
        crate::wavelet::scatter_lattice(buf, width, stride, sw, sh, &sub);
    }
}

// ---------------------------------------------------------------------------
// IPN 42-155 §II.C — dynamic range of wavelet-transformed data
// ---------------------------------------------------------------------------

/// IPN 42-155 §II.C Table 3 — `Σ|c_i|`, the sum of the absolute values
/// of the taps of the linear high-pass filter that approximates one
/// 1-D high-pass operation, as an exact rational `(numerator,
/// denominator)`.
///
/// §II.C: after one high-pass operation the approximate dynamic-range
/// expansion is `Σ|c_i|` (`log2(Σ|c_i|)` bits); after two operations
/// (an HH subband — one horizontal + one vertical high-pass) it is
/// `(Σ|c_i|)^2`. Low-pass filtering "amounts to pixel averaging" and
/// does not expand the range, and under the §II.B pyramid every deeper
/// stage decomposes the (low-pass) LL lattice — so **two** high-pass
/// operations is the worst case for any subband of any 2-D
/// decomposition depth.
///
/// The same per-filter rationals appear as the IPN 42-164 §III.B `γ`
/// factors (both papers define the constant as the sum of the absolute
/// linear-approximation taps); this is a thin 2-D-named re-export of
/// [`crate::wavelet3d::high_pass_gamma`] kept so §II.C call sites read
/// against the 42-155 table they cite.
pub fn abs_tap_sum(filter: WaveletFilter) -> (u32, u32) {
    crate::wavelet3d::high_pass_gamma(filter)
}

/// IPN 42-155 §II.C equation (8) — the *approximate* maximum input
/// dynamic range `x_max - x_min` such that `b`-bit words storing the
/// output of `high_pass_ops` high-pass filter operations cannot
/// overflow:
///
/// ```text
///     x_max - x_min  <=  2 (2^(b-1) - 1) / (Σ|c_i|)^ops
/// ```
///
/// (the `2^(b-1) - 1` bound on both `h_max` and `-h_min` is §II.C
/// footnote 6: transformed pixels are stored in sign-magnitude form,
/// §III.A). Returns the floor of the bound, or `None` for a word size
/// outside `{8, 16, 32}` or an op count outside `{1, 2}` (the §II.C
/// tabulated domain). The bound is a linear approximation — §II.C
/// notes it can be "just slightly optimistic" against the exact
/// nonlinear transform; [`max_input_range`] carries the exact values.
pub fn approx_max_input_range(
    filter: WaveletFilter,
    word_bits: u8,
    high_pass_ops: u8,
) -> Option<u64> {
    if !matches!(word_bits, 8 | 16 | 32) || !matches!(high_pass_ops, 1 | 2) {
        return None;
    }
    let (num, den) = abs_tap_sum(filter);
    let (num, den) = (num as u128, den as u128);
    let half_range = (1u128 << (word_bits - 1)) - 1; // 2^(b-1) - 1
    let (n_pow, d_pow) = if high_pass_ops == 2 {
        (num * num, den * den)
    } else {
        (num, den)
    };
    Some((2 * half_range * d_pow / n_pow) as u64)
}

/// IPN 42-155 §II.C **Table 4** — the maximum input dynamic range
/// (`x_max - x_min`) that guarantees a `word_bits`-bit output word will
/// not overflow following `high_pass_ops` (1 or 2) high-pass filter
/// operations. "Entries in this table are computed exactly using the
/// nonlinear wavelet transforms" — these are the paper's authoritative
/// values, not the eq (8) approximation.
///
/// Returns `None` for a word size outside `{8, 16, 32}` or an op count
/// outside `{1, 2}`.
///
/// §II.C worked examples (pinned by tests):
///
/// * 12-bit input (range 4095) fits 16-bit words after two high-pass
///   operations under **every** filter (the table's two-op 16-bit
///   column is at least 6449) — the MER operating point ("On MER, all
///   cameras produce 12-bit pixels and each is stored using a 16-bit
///   word").
/// * 14-bit input (range 16383) fits 16-bit words after **one** but
///   not two high-pass operations, for every filter.
pub fn max_input_range(filter: WaveletFilter, word_bits: u8, high_pass_ops: u8) -> Option<u64> {
    // Table 4 rows in filter order A..F, Q; columns (one op: 8/16/32-bit
    // word, two ops: 8/16/32-bit word).
    let row: [u64; 6] = match filter {
        WaveletFilter::FilterA => [101, 26213, 1717986917, 40, 10485, 687194766],
        WaveletFilter::FilterB => [92, 23830, 1561806289, 33, 8665, 567929559],
        WaveletFilter::FilterC => [81, 20971, 1374389534, 25, 6710, 439804651],
        WaveletFilter::FilterD => [99, 25574, 1676084798, 38, 9980, 654081872],
        WaveletFilter::FilterE => [86, 22309, 1462116525, 29, 7594, 497741795],
        WaveletFilter::FilterF => [79, 20559, 1347440719, 24, 6449, 422726500],
        WaveletFilter::FilterQ => [92, 23830, 1561806289, 33, 8665, 567929559],
    };
    let col = match (high_pass_ops, word_bits) {
        (1, 8) => 0,
        (1, 16) => 1,
        (1, 32) => 2,
        (2, 8) => 3,
        (2, 16) => 4,
        (2, 32) => 5,
        _ => return None,
    };
    Some(row[col])
}

/// The smallest word size in `{8, 16, 32}` bits whose §II.C **Table 4**
/// two-high-pass-operation entry accommodates `input_range =
/// x_max - x_min` — i.e. the smallest tabulated word that can store
/// every coefficient of a 2-D pyramidal decomposition of such input
/// (the HH subbands see two high-pass operations; every other subband
/// sees at most one, §II.C). Returns `None` when even 32-bit words are
/// insufficient per the table.
///
/// The §II.C MER example pins `word_bits_for_input_range(4095, f) ==
/// 16` for every filter (12-bit pixels stored in 16-bit words).
pub fn word_bits_for_input_range(input_range: u64, filter: WaveletFilter) -> Option<u8> {
    for word_bits in [8u8, 16, 32] {
        if input_range <= max_input_range(filter, word_bits, 2)? {
            return Some(word_bits);
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    const ALL: [WaveletFilter; 7] = [
        WaveletFilter::FilterQ,
        WaveletFilter::FilterA,
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

    /// IPN 42-155 §II.B: each further stage decomposes only the LL
    /// subband — the stride-`2^(k-1)` even/even lattice. Every
    /// off-lattice coefficient must be identical between `(k-1)`- and
    /// `k`-level decompositions, for all seven filters. (Regression pin
    /// for the pre-r405 top-left-rectangle recursion.)
    #[test]
    fn dyadic_deeper_stages_touch_only_the_ll_lattice() {
        for f in ALL {
            let (w, h) = (48usize, 40usize);
            let original: Vec<i32> = (0..w * h).map(|i| (i as i32 * 37) % 255 - 128).collect();
            for k in 2u8..=4 {
                let mut shallow = original.clone();
                forward_2d_dyadic(&mut shallow, w, h, k - 1, f);
                let mut deep = original.clone();
                forward_2d_dyadic(&mut deep, w, h, k, f);
                let stride = 1usize << (k - 1);
                for y in 0..h {
                    for x in 0..w {
                        if x % stride == 0 && y % stride == 0 {
                            continue;
                        }
                        assert_eq!(
                            shallow[y * w + x],
                            deep[y * w + x],
                            "filter {f:?}: stage {k} modified off-lattice ({x},{y})"
                        );
                    }
                }
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
            forward_2d_dyadic(&mut buf, w, h, levels, WaveletFilter::FilterA);
            inverse_2d_dyadic(&mut buf, w, h, levels, WaveletFilter::FilterA);
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
        let p = WaveletFilter::FilterQ.int_params();
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
        let q = WaveletFilter::FilterQ.int_params();
        let a = WaveletFilter::FilterA.int_params();
        assert_eq!((q.a_m1, q.a_0, q.a_1), (a.a_m1, a.a_0, a.a_1));
        assert_eq!(q.b, 8);
        assert_eq!(a.b, 0);
    }

    // -- IPN 42-155 §II.C dynamic-range analysis -------------------------

    #[test]
    fn table3_abs_tap_sums_pin() {
        // §II.C Table 3, Σ|c_i| column, as exact rationals.
        let pins = [
            (WaveletFilter::FilterA, (5, 2)),
            (WaveletFilter::FilterB, (11, 4)),
            (WaveletFilter::FilterC, (25, 8)),
            (WaveletFilter::FilterD, (41, 16)),
            (WaveletFilter::FilterE, (47, 16)),
            (WaveletFilter::FilterF, (51, 16)),
            (WaveletFilter::FilterQ, (11, 4)),
        ];
        for (f, expect) in pins {
            assert_eq!(abs_tap_sum(f), expect, "{f:?} Σ|c_i|");
        }
        // Worst-case two-op expansion "about 3.34 bits, arising from
        // filter F": (51/16)^2 = 2601/256, log2 ≈ 3.345.
        let (n, d) = abs_tap_sum(WaveletFilter::FilterF);
        let two_op = ((n as f64) / (d as f64)).powi(2);
        let bits = two_op.log2();
        assert!((bits - 3.34).abs() < 0.01, "filter F two-op bits {bits}");
    }

    #[test]
    fn table4_exact_pins_and_eq8_consistency() {
        // Every Table 4 cell must sit within ±2 of the eq (8) linear
        // bound — §II.C notes eq (8) "turns out to be just slightly
        // optimistic" (filter F, 16-bit, two ops: eq (8) gives ≈6450.1
        // where the exact value is 6449), and the approximation error
        // is bounded by eq (6)/(7). This cross-validates the exact
        // transcription against the independently-computed rationals.
        for f in ALL {
            for ops in [1u8, 2] {
                for wb in [8u8, 16, 32] {
                    let exact = max_input_range(f, wb, ops).unwrap();
                    let approx = approx_max_input_range(f, wb, ops).unwrap();
                    let diff = exact.abs_diff(approx);
                    assert!(
                        diff <= 2,
                        "{f:?} {ops}-op {wb}-bit: exact {exact} vs eq(8) {approx}"
                    );
                }
            }
        }
        // The §II.C worked example: filter F, 16-bit words, two ops.
        assert_eq!(max_input_range(WaveletFilter::FilterF, 16, 2), Some(6449));
        assert_eq!(
            approx_max_input_range(WaveletFilter::FilterF, 16, 2),
            Some(6450)
        );
        // Filters B and Q share Σ|c_i| = 11/4, so their exact columns
        // coincide throughout Table 4.
        for ops in [1u8, 2] {
            for wb in [8u8, 16, 32] {
                assert_eq!(
                    max_input_range(WaveletFilter::FilterB, wb, ops),
                    max_input_range(WaveletFilter::FilterQ, wb, ops)
                );
            }
        }
        // Out-of-domain queries refuse rather than alias.
        assert_eq!(max_input_range(WaveletFilter::FilterQ, 12, 1), None);
        assert_eq!(max_input_range(WaveletFilter::FilterQ, 16, 3), None);
        assert_eq!(approx_max_input_range(WaveletFilter::FilterQ, 24, 2), None);
    }

    #[test]
    fn mer_12bit_and_14bit_word_size_examples() {
        // §II.C: "On MER, all cameras produce 12-bit pixels and each is
        // stored using a 16-bit word ... for each filter, using 16-bit
        // words, we can accommodate an input pixel dynamic range of at
        // least 6449, which easily supports 12-bit input pixels."
        for f in ALL {
            assert!(max_input_range(f, 16, 2).unwrap() >= 6449, "{f:?}");
            assert!(4095 <= max_input_range(f, 16, 2).unwrap(), "{f:?}");
            assert_eq!(word_bits_for_input_range(4095, f), Some(16), "{f:?}");
            // "for 14-bit input pixels, 16-bit words are adequate to
            // store the wavelet transform output following one but not
            // two high-pass filter operations."
            assert!(16383 <= max_input_range(f, 16, 1).unwrap(), "{f:?}");
            assert!(16383 > max_input_range(f, 16, 2).unwrap(), "{f:?}");
            assert_eq!(word_bits_for_input_range(16383, f), Some(32), "{f:?}");
            // 8-bit input needs 16-bit words two-op (the two-op 8-bit
            // column tops out at 40 < 255)...
            assert_eq!(word_bits_for_input_range(255, f), Some(16), "{f:?}");
            // ...and 16-bit input (range 65535) sits far below every
            // two-op 32-bit-word entry (min 422726500, filter F), so
            // the crate's i32 coefficient buffers can never overflow
            // for any sample depth up to 16 bits under any filter.
            assert!(65535 <= max_input_range(f, 32, 2).unwrap(), "{f:?}");
            assert_eq!(word_bits_for_input_range(65535, f), Some(32), "{f:?}");
        }
    }

    #[test]
    fn live_transform_respects_published_word_sizes() {
        // Empirical §II.C witness: a full-range 12-bit checkerboard
        // (the range-maximising input shape — §II.C: the output range
        // "is fully utilized when the rows and columns of the image
        // conspire together to maximize the range of output values in
        // an HH subband") stays within 16-bit coefficient words under
        // every filter and depth 1..=3, per the Table 4 12-bit MER
        // example. A full-range 16-bit checkerboard likewise stays
        // within the §II.C word budget for two high-pass operations
        // (`coefficient_word_bits(16, f, 2)`-bit signed words).
        for f in ALL {
            for levels in 1u8..=3 {
                let (w, h) = (32usize, 32);
                let mut buf12: Vec<i32> = (0..w * h)
                    .map(|i| {
                        let (x, y) = (i % w, i / w);
                        if (x ^ y) & 1 == 0 {
                            -2048
                        } else {
                            2047
                        }
                    })
                    .collect();
                forward_2d_dyadic(&mut buf12, w, h, levels, f);
                let max12 = buf12.iter().map(|c| c.unsigned_abs()).max().unwrap();
                assert!(
                    max12 < (1 << 15),
                    "{f:?} depth {levels}: 12-bit input overflowed 16-bit words ({max12})"
                );

                let mut buf16: Vec<i32> = (0..w * h)
                    .map(|i| {
                        let (x, y) = (i % w, i / w);
                        if (x ^ y) & 1 == 0 {
                            -32768
                        } else {
                            32767
                        }
                    })
                    .collect();
                forward_2d_dyadic(&mut buf16, w, h, levels, f);
                let max16 = buf16.iter().map(|c| c.unsigned_abs()).max().unwrap();
                let word = crate::wavelet3d::coefficient_word_bits(16, f, 2);
                assert!(
                    max16 < (1u32 << (word - 1)),
                    "{f:?} depth {levels}: 16-bit input exceeded {word}-bit words ({max16})"
                );
            }
        }
    }
}
