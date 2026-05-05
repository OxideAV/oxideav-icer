//! Float wavelet filters A-F — IPN 42-155 §III.A enumerates seven
//! candidate kernels labelled `A` through `G`, of which Filter `Q` is
//! the integer 5/3 (handled by [`crate::wavelet`]) and the remainder
//! are float-precision lifting variants. Filters `A` through `F` here
//! follow the Cohen-Daubechies-Feauveau biorthogonal lifting families
//! the paper draws from; the exact float coefficients chosen below
//! are the symmetric biorthogonal 9/7 lifting parameters from
//! Sweldens 1996, "The Lifting Scheme: A Custom-Design Construction
//! of Biorthogonal Wavelets" — well-known in the wavelet literature
//! and not derived from any JPL flight code or reference
//! implementation.
//!
//! All filters here are **lossless via lifting** in `f32` arithmetic:
//! within IEEE-754 round-off the inverse path reverses the forward
//! path bit-for-bit, modulo accumulated rounding (exact-equality is
//! impossible in float, so the round-trip tests assert tolerance, not
//! identity).
//!
//! The seven filter parameter sets land in [`FILTER_PARAMS`]; per-axis
//! lifting uses the same dual-step `predict / update` shape as the
//! integer 5/3 in [`crate::wavelet`]. This module is complementary to
//! the integer transform — the `q` filter is handled by the integer
//! module, the float filters here cover the IPN 42-155 §III.A
//! `A`-`F` fast lossy reconstructions.
//!
//! The `q` filter id (Reversible 5/3) is also accepted by
//! [`forward_2d`] / [`inverse_2d`] but routes through the integer
//! lifting path so a single call site can switch on the segment header
//! filter id without branching on the integer/float distinction.

use crate::error::{IcerError, Result};
use crate::header::WaveletFilter;
use crate::wavelet;

/// One pair of predict/update lifting coefficients for a CDF-style
/// 9/7-shaped filter:
///
/// ```text
///     d[k] += alpha * (s[k] + s[k+1])
///     s[k] += beta  * (d[k-1] + d[k])
///     d[k] += gamma * (s[k] + s[k+1])
///     s[k] += delta * (d[k-1] + d[k])
///     d[k] *= zeta
///     s[k] /= zeta
/// ```
///
/// Filter `Q` (Reversible 5/3) is marked `is_integer = true` — its
/// coefficients are unused (the integer path is a different code
/// path), but the table entry exists so a single switch on the
/// segment header filter id selects the right transform.
#[derive(Debug, Clone, Copy)]
pub struct FilterParams {
    pub alpha: f32,
    pub beta: f32,
    pub gamma: f32,
    pub delta: f32,
    pub zeta: f32,
    pub is_integer: bool,
}

/// Lookup table indexed by filter id (`0` = `Q`, `1` = `A`, ...).
/// Kiely & Klimesh's IPN 42-155 §III.A does not publish per-filter
/// numerical coefficients — it lists the seven candidates by name
/// only and refers to a follow-up paper [13] for the exact lifting
/// numbers. The non-`Q` entries here use the well-known CDF 9/7
/// lifting coefficients (Sweldens 1996); each filter offers the same
/// kernel but with a different post-scale `zeta`. This is sufficient
/// for the round-2 milestone (prove float kernels work + round-trip
/// to tolerance); swapping in the per-filter coefficients for a
/// real-world Mars-rover interop is a follow-up once the IPN [13]
/// reference lands in `docs/image/icer/`.
pub const FILTER_PARAMS: [FilterParams; 7] = [
    // Q: Reversible 5/3 (integer path; coefficients unused).
    FilterParams {
        alpha: 0.0,
        beta: 0.0,
        gamma: 0.0,
        delta: 0.0,
        zeta: 1.0,
        is_integer: true,
    },
    // A: CDF 9/7, post-scale zeta = 1.149604398
    FilterParams {
        alpha: -1.586_134_3,
        beta: -0.052_980_12,
        gamma: 0.882_911_1,
        delta: 0.443_506_85,
        zeta: 1.149_604_4,
        is_integer: false,
    },
    // B: same kernel, zeta = 1.0 (no post-scale).
    FilterParams {
        alpha: -1.586_134_3,
        beta: -0.052_980_12,
        gamma: 0.882_911_1,
        delta: 0.443_506_85,
        zeta: 1.0,
        is_integer: false,
    },
    // C: float 5/3 variant (alpha=-1/2, beta=1/4, no second pair).
    FilterParams {
        alpha: -0.5,
        beta: 0.25,
        gamma: 0.0,
        delta: 0.0,
        zeta: 1.0,
        is_integer: false,
    },
    // D: CDF 9/7 with negated post-scale (sign convention swap).
    FilterParams {
        alpha: -1.586_134_3,
        beta: -0.052_980_12,
        gamma: 0.882_911_1,
        delta: 0.443_506_85,
        zeta: -1.149_604_4,
        is_integer: false,
    },
    // E: Daubechies 4-tap analogue via lifting.
    FilterParams {
        alpha: -1.732_050_8, // -sqrt(3)
        beta: 0.433_012_7,   // sqrt(3)/4
        gamma: -0.066_987_3, // (sqrt(3)-2)/4
        delta: 0.0,
        zeta: 0.965_925_8, // (sqrt(3)+1)/sqrt(2)/2
        is_integer: false,
    },
    // F: Haar (degenerate lifting — single predict + update pair).
    FilterParams {
        alpha: -1.0,
        beta: 0.5,
        gamma: 0.0,
        delta: 0.0,
        zeta: std::f32::consts::FRAC_1_SQRT_2,
        is_integer: false,
    },
];

impl WaveletFilter {
    /// Look up the lifting parameters for this filter id.
    pub fn params(self) -> FilterParams {
        FILTER_PARAMS[self as usize]
    }
}

/// In-place 1-D forward float lifting (predict / update / predict /
/// update / scale). Uses whole-sample symmetric extension at the
/// boundaries, matching the integer 5/3 in [`crate::wavelet`].
pub fn forward_1d_float(row: &mut [f32], p: &FilterParams) {
    let n = row.len();
    if n < 2 {
        return;
    }
    lift_predict(row, p.alpha);
    lift_update(row, p.beta);
    if p.gamma != 0.0 {
        lift_predict(row, p.gamma);
    }
    if p.delta != 0.0 {
        lift_update(row, p.delta);
    }
    if p.zeta != 1.0 && p.zeta != 0.0 {
        let mut k = 0;
        while 2 * k < n {
            row[2 * k] /= p.zeta;
            k += 1;
        }
        let mut k = 0;
        while 2 * k + 1 < n {
            row[2 * k + 1] *= p.zeta;
            k += 1;
        }
    }
}

/// In-place 1-D inverse float lifting — exact reverse of
/// [`forward_1d_float`] up to IEEE-754 round-off.
pub fn inverse_1d_float(row: &mut [f32], p: &FilterParams) {
    let n = row.len();
    if n < 2 {
        return;
    }
    if p.zeta != 1.0 && p.zeta != 0.0 {
        let mut k = 0;
        while 2 * k < n {
            row[2 * k] *= p.zeta;
            k += 1;
        }
        let mut k = 0;
        while 2 * k + 1 < n {
            row[2 * k + 1] /= p.zeta;
            k += 1;
        }
    }
    if p.delta != 0.0 {
        lift_update(row, -p.delta);
    }
    if p.gamma != 0.0 {
        lift_predict(row, -p.gamma);
    }
    lift_update(row, -p.beta);
    lift_predict(row, -p.alpha);
}

fn lift_predict(row: &mut [f32], coef: f32) {
    let n = row.len();
    let mut k = 0;
    while 2 * k + 1 < n {
        let s_k = row[2 * k];
        let s_k1_idx = 2 * k + 2;
        let s_k1 = if s_k1_idx < n {
            row[s_k1_idx]
        } else {
            row[2 * k]
        };
        row[2 * k + 1] += coef * (s_k + s_k1);
        k += 1;
    }
}

fn lift_update(row: &mut [f32], coef: f32) {
    let n = row.len();
    let mut k = 0;
    while 2 * k < n {
        let h_k_idx = 2 * k + 1;
        let h_k = if h_k_idx < n { row[h_k_idx] } else { 0.0 };
        let h_km1 = if k == 0 { h_k } else { row[2 * k - 1] };
        row[2 * k] += coef * (h_km1 + h_k);
        k += 1;
    }
}

/// Forward 2-D one-level float lifting on a `width` x `height` buffer.
/// Row-then-column order matches
/// [`crate::wavelet::forward_53_2d_one_level`].
pub fn forward_2d_one_level_float(buf: &mut [f32], width: usize, height: usize, p: &FilterParams) {
    debug_assert_eq!(buf.len(), width * height);
    for y in 0..height {
        let row = &mut buf[y * width..y * width + width];
        forward_1d_float(row, p);
    }
    let mut col = vec![0f32; height];
    for x in 0..width {
        for y in 0..height {
            col[y] = buf[y * width + x];
        }
        forward_1d_float(&mut col, p);
        for y in 0..height {
            buf[y * width + x] = col[y];
        }
    }
}

/// Inverse of [`forward_2d_one_level_float`].
pub fn inverse_2d_one_level_float(buf: &mut [f32], width: usize, height: usize, p: &FilterParams) {
    debug_assert_eq!(buf.len(), width * height);
    let mut col = vec![0f32; height];
    for x in 0..width {
        for y in 0..height {
            col[y] = buf[y * width + x];
        }
        inverse_1d_float(&mut col, p);
        for y in 0..height {
            buf[y * width + x] = col[y];
        }
    }
    for y in 0..height {
        let row = &mut buf[y * width..y * width + width];
        inverse_1d_float(row, p);
    }
}

/// Dispatch helper: forward 2-D dyadic transform on either the
/// integer path (filter `Q`) or the float path (filters `A`-`F`).
/// The integer variant takes signed integers; the float variant
/// converts to/from float internally so callers can stay
/// filter-agnostic at the i32 layer.
pub fn forward_2d(
    coeffs: &mut [i32],
    width: usize,
    height: usize,
    levels: u8,
    filter: WaveletFilter,
) -> Result<()> {
    let p = filter.params();
    if p.is_integer {
        wavelet::forward_53_dyadic(coeffs, width, height, levels);
        return Ok(());
    }
    if width < 2 || height < 2 {
        return Err(IcerError::invalid(
            "float DWT requires width >= 2 and height >= 2",
        ));
    }
    let mut buf: Vec<f32> = coeffs.iter().map(|&v| v as f32).collect();
    let mut w = width;
    let mut h = height;
    for _ in 0..levels {
        if w < 2 || h < 2 {
            break;
        }
        forward_2d_one_level_sub_float(&mut buf, width, w, h, &p);
        w = w.div_ceil(2);
        h = h.div_ceil(2);
    }
    for (dst, src) in coeffs.iter_mut().zip(buf.iter()) {
        *dst = src.round() as i32;
    }
    Ok(())
}

/// Inverse counterpart of [`forward_2d`]. Note: float-filter
/// round-trip is **lossy** (the rounding step loses fractional
/// precision); callers should use filter `Q` if exact integer
/// round-trip is required.
pub fn inverse_2d(
    coeffs: &mut [i32],
    width: usize,
    height: usize,
    levels: u8,
    filter: WaveletFilter,
) -> Result<()> {
    let p = filter.params();
    if p.is_integer {
        wavelet::inverse_53_dyadic(coeffs, width, height, levels);
        return Ok(());
    }
    if width < 2 || height < 2 {
        return Err(IcerError::invalid(
            "float DWT requires width >= 2 and height >= 2",
        ));
    }
    let mut buf: Vec<f32> = coeffs.iter().map(|&v| v as f32).collect();
    let mut sizes = Vec::with_capacity(levels as usize);
    let mut w = width;
    let mut h = height;
    for _ in 0..levels {
        if w < 2 || h < 2 {
            break;
        }
        sizes.push((w, h));
        w = w.div_ceil(2);
        h = h.div_ceil(2);
    }
    for (sw, sh) in sizes.into_iter().rev() {
        inverse_2d_one_level_sub_float(&mut buf, width, sw, sh, &p);
    }
    for (dst, src) in coeffs.iter_mut().zip(buf.iter()) {
        *dst = src.round() as i32;
    }
    Ok(())
}

fn forward_2d_one_level_sub_float(
    buf: &mut [f32],
    stride: usize,
    sub_w: usize,
    sub_h: usize,
    p: &FilterParams,
) {
    for y in 0..sub_h {
        let row = &mut buf[y * stride..y * stride + sub_w];
        forward_1d_float(row, p);
    }
    let mut col = vec![0f32; sub_h];
    for x in 0..sub_w {
        for y in 0..sub_h {
            col[y] = buf[y * stride + x];
        }
        forward_1d_float(&mut col, p);
        for y in 0..sub_h {
            buf[y * stride + x] = col[y];
        }
    }
}

fn inverse_2d_one_level_sub_float(
    buf: &mut [f32],
    stride: usize,
    sub_w: usize,
    sub_h: usize,
    p: &FilterParams,
) {
    let mut col = vec![0f32; sub_h];
    for x in 0..sub_w {
        for y in 0..sub_h {
            col[y] = buf[y * stride + x];
        }
        inverse_1d_float(&mut col, p);
        for y in 0..sub_h {
            buf[y * stride + x] = col[y];
        }
    }
    for y in 0..sub_h {
        let row = &mut buf[y * stride..y * stride + sub_w];
        inverse_1d_float(row, p);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq_slice(a: &[f32], b: &[f32], tol: f32) {
        assert_eq!(a.len(), b.len());
        for (i, (x, y)) in a.iter().zip(b.iter()).enumerate() {
            assert!((x - y).abs() <= tol, "index {i}: {x} vs {y} (tol {tol})");
        }
    }

    fn ramp_f32(n: usize) -> Vec<f32> {
        (0..n).map(|i| ((i % 64) as f32) - 32.0).collect()
    }

    #[test]
    fn forward_inverse_1d_float_filter_a() {
        let p = WaveletFilter::NineSevenA.params();
        for n in [4usize, 8, 16, 17, 33, 64] {
            let original = ramp_f32(n);
            let mut buf = original.clone();
            forward_1d_float(&mut buf, &p);
            inverse_1d_float(&mut buf, &p);
            approx_eq_slice(&buf, &original, 1e-3);
        }
    }

    #[test]
    fn forward_inverse_1d_float_all_filters() {
        for filter in [
            WaveletFilter::NineSevenA,
            WaveletFilter::FilterB,
            WaveletFilter::FilterC,
            WaveletFilter::FilterD,
            WaveletFilter::FilterE,
            WaveletFilter::FilterF,
        ] {
            let p = filter.params();
            let original = ramp_f32(32);
            let mut buf = original.clone();
            forward_1d_float(&mut buf, &p);
            inverse_1d_float(&mut buf, &p);
            // Tolerance is generous: float lifting accumulates round-off
            // proportional to the buffer length and the magnitude of the
            // lifting coefficients. 1e-2 covers all 6 filters at n=32.
            approx_eq_slice(&buf, &original, 1e-2);
        }
    }

    #[test]
    fn forward_inverse_2d_float_filter_a() {
        let p = WaveletFilter::NineSevenA.params();
        let w = 8;
        let h = 8;
        let original: Vec<f32> = ramp_f32(w * h);
        let mut buf = original.clone();
        forward_2d_one_level_float(&mut buf, w, h, &p);
        inverse_2d_one_level_float(&mut buf, w, h, &p);
        approx_eq_slice(&buf, &original, 1e-2);
    }

    #[test]
    fn forward_2d_dispatch_routes_q_to_integer() {
        // For filter Q, the dispatch helper must produce a bit-exact
        // round-trip (integer path).
        let w = 8;
        let h = 8;
        let original: Vec<i32> = (0..(w * h) as i32).collect();
        let mut buf = original.clone();
        forward_2d(&mut buf, w, h, 2, WaveletFilter::Reversible53).unwrap();
        inverse_2d(&mut buf, w, h, 2, WaveletFilter::Reversible53).unwrap();
        assert_eq!(buf, original, "filter Q must round-trip bit-exactly");
    }

    #[test]
    fn forward_2d_dispatch_routes_a_to_float() {
        // For filter A, the dispatch helper rounds at the boundaries
        // so we only check tolerance.
        let w = 8;
        let h = 8;
        let original: Vec<i32> = (0..(w * h) as i32).map(|x| x * 4).collect();
        let mut buf = original.clone();
        forward_2d(&mut buf, w, h, 2, WaveletFilter::NineSevenA).unwrap();
        inverse_2d(&mut buf, w, h, 2, WaveletFilter::NineSevenA).unwrap();
        for (a, b) in buf.iter().zip(original.iter()) {
            assert!(
                (a - b).abs() <= 4,
                "float DWT round-trip too lossy: {a} vs {b}"
            );
        }
    }
}
