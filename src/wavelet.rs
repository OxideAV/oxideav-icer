//! Integer reversible 5/3 wavelet — both directions (forward DWT for
//! the encoder, inverse DWT for the decoder).
//!
//! ICER's Filter `Q` (IPN 42-155 §III.A bullet 7) is the canonical
//! integer 5/3 lifting wavelet — well-known in the wavelet literature
//! since Calderbank, Daubechies, Sweldens & Yeo (1998), "Wavelet
//! Transforms That Map Integers to Integers". This module implements
//! it from the textbook lifting equations:
//!
//! Forward (one level along one axis), splitting `x[0..N]` into low
//! samples `L[k] = x[2k]` and high samples `H[k] = x[2k+1]` indexed
//! by `k`:
//!
//! ```text
//!     H[k] -= ((L[k] + L[k+1]) >> 1)             ; predict
//!     L[k] += ((H[k-1] + H[k]  + 2) >> 2)        ; update
//! ```
//!
//! Inverse (exact reverse — the whole point of integer-to-integer
//! lifting):
//!
//! ```text
//!     L[k] -= ((H[k-1] + H[k]  + 2) >> 2)        ; un-update
//!     H[k] += ((L[k] + L[k+1]) >> 1)             ; un-predict
//! ```
//!
//! Boundaries use whole-sample symmetric extension (out-of-range
//! `L[N]` mirrors `L[N-2]`, `H[-1]` mirrors `H[0]`), matching the
//! periodic boundary extension recommended by IPN 42-155 §III.A
//! "boundary extension by reflection".
//!
//! After the 1-D transform along rows, the same transform is applied
//! along columns of the result. Repeating the 2-D pass on the LL
//! sub-band gives the dyadic decomposition (IPN 42-155 §III.A — `D`
//! levels). The inverse path runs the steps in opposite order.
//!
//! Round 1 only ships Filter `Q`. The other six filters from IPN
//! 42-155 §III.A (Filters A-G aside from Q) are deferred to round 2 —
//! they share the same dyadic-decomposition outer loop, only the
//! per-level lifting steps differ.

/// In-place 1-D forward 5/3 lifting along a single row of length `n`.
/// Even-indexed samples become low-pass coefficients; odd-indexed
/// samples become high-pass coefficients.
///
/// `n` must be `>= 2`. Single-sample inputs (`n == 1`) are a no-op
/// because there is no high sample to predict. Odd-length inputs use
/// an extra mirror sample at the right edge.
pub fn forward_53_1d(row: &mut [i32]) {
    let n = row.len();
    if n < 2 {
        return;
    }
    // Predict step: H[k] -= (L[k] + L[k+1]) / 2 with whole-sample
    // symmetric extension at the right edge (L[k+1] mirrors back into
    // the buffer when 2k+2 >= n).
    let mut k = 0;
    while 2 * k + 1 < n {
        let l_k = row[2 * k];
        let l_k1_idx = 2 * k + 2;
        let l_k1 = if l_k1_idx < n {
            row[l_k1_idx]
        } else {
            // Mirror: x[n] = x[n-2] for whole-sample symmetric extension.
            row[2 * k]
        };
        // Floor-division by shift requires the operand to be non-negative
        // for a deterministic result; coefficients are signed so we use a
        // arithmetic shift via Rust's `>>` on i32 which is arithmetic.
        row[2 * k + 1] = row[2 * k + 1].wrapping_sub((l_k + l_k1) >> 1);
        k += 1;
    }

    // Update step: L[k] += (H[k-1] + H[k] + 2) >> 2 with whole-sample
    // symmetric extension at the left edge (H[-1] mirrors H[0]).
    let mut k = 0;
    while 2 * k < n {
        let h_k_idx = 2 * k + 1;
        let h_k = if h_k_idx < n { row[h_k_idx] } else { 0 };
        let h_km1 = if k == 0 {
            // Mirror the right neighbour: H[-1] := H[0].
            h_k
        } else {
            row[2 * k - 1]
        };
        row[2 * k] = row[2 * k].wrapping_add((h_km1 + h_k + 2) >> 2);
        k += 1;
    }
}

/// In-place 1-D inverse 5/3 lifting along a single row.  Bit-exact
/// inverse of [`forward_53_1d`] (this is a guaranteed property of
/// integer-to-integer lifting).
pub fn inverse_53_1d(row: &mut [i32]) {
    let n = row.len();
    if n < 2 {
        return;
    }
    // Un-update step: L[k] -= (H[k-1] + H[k] + 2) >> 2.
    let mut k = 0;
    while 2 * k < n {
        let h_k_idx = 2 * k + 1;
        let h_k = if h_k_idx < n { row[h_k_idx] } else { 0 };
        let h_km1 = if k == 0 { h_k } else { row[2 * k - 1] };
        row[2 * k] = row[2 * k].wrapping_sub((h_km1 + h_k + 2) >> 2);
        k += 1;
    }
    // Un-predict step: H[k] += (L[k] + L[k+1]) >> 1.
    let mut k = 0;
    while 2 * k + 1 < n {
        let l_k = row[2 * k];
        let l_k1_idx = 2 * k + 2;
        let l_k1 = if l_k1_idx < n {
            row[l_k1_idx]
        } else {
            row[2 * k]
        };
        row[2 * k + 1] = row[2 * k + 1].wrapping_add((l_k + l_k1) >> 1);
        k += 1;
    }
}

/// 2-D one-level forward 5/3: rows then columns. The transformed
/// buffer is laid out in interleaved fashion (even = LL-along-axis,
/// odd = HH-along-axis); deinterleaving into the conventional
/// quadrant layout (`LL | LH | HL | HH`) is performed by
/// [`deinterleave_subbands`].
pub fn forward_53_2d_one_level(buf: &mut [i32], width: usize, height: usize) {
    debug_assert_eq!(buf.len(), width * height);
    // Rows.
    for y in 0..height {
        let row = &mut buf[y * width..y * width + width];
        forward_53_1d(row);
    }
    // Columns. Copy the column out, transform, copy back — keeps
    // memory-access patterns simple in round 1.
    let mut col = vec![0i32; height];
    for x in 0..width {
        for y in 0..height {
            col[y] = buf[y * width + x];
        }
        forward_53_1d(&mut col);
        for y in 0..height {
            buf[y * width + x] = col[y];
        }
    }
}

/// Bit-exact inverse of [`forward_53_2d_one_level`].
pub fn inverse_53_2d_one_level(buf: &mut [i32], width: usize, height: usize) {
    debug_assert_eq!(buf.len(), width * height);
    // Columns first (inverse of the forward order).
    let mut col = vec![0i32; height];
    for x in 0..width {
        for y in 0..height {
            col[y] = buf[y * width + x];
        }
        inverse_53_1d(&mut col);
        for y in 0..height {
            buf[y * width + x] = col[y];
        }
    }
    // Rows.
    for y in 0..height {
        let row = &mut buf[y * width..y * width + width];
        inverse_53_1d(row);
    }
}

/// `D`-level dyadic forward 5/3 transform. Each level halves the
/// active region (rounding *up* — odd dimensions are handled by the
/// boundary extension in [`forward_53_1d`]). After `D` levels, the
/// top-left `ceil(width/2^D) x ceil(height/2^D)` block holds the LL
/// subband, the rest is interleaved LH / HL / HH detail.
pub fn forward_53_dyadic(buf: &mut [i32], width: usize, height: usize, levels: u8) {
    let mut w = width;
    let mut h = height;
    for _ in 0..levels {
        if w < 2 || h < 2 {
            break;
        }
        // Transform the active LL region (top-left wxh sub-rectangle).
        forward_53_2d_one_level_sub(buf, width, w, h);
        w = w.div_ceil(2);
        h = h.div_ceil(2);
    }
}

/// `D`-level dyadic inverse 5/3 — exact reverse of
/// [`forward_53_dyadic`].
pub fn inverse_53_dyadic(buf: &mut [i32], width: usize, height: usize, levels: u8) {
    // Pre-compute per-level sub-rectangle sizes so we can iterate in
    // reverse without recomputing.
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
        inverse_53_2d_one_level_sub(buf, width, sw, sh);
    }
}

/// Helper: 1-level forward 5/3 on the top-left `sub_w x sub_h`
/// sub-rectangle of a buffer whose row stride is `stride` (in
/// `i32` samples).
fn forward_53_2d_one_level_sub(buf: &mut [i32], stride: usize, sub_w: usize, sub_h: usize) {
    // Rows.
    for y in 0..sub_h {
        let row = &mut buf[y * stride..y * stride + sub_w];
        forward_53_1d(row);
    }
    // Columns.
    let mut col = vec![0i32; sub_h];
    for x in 0..sub_w {
        for y in 0..sub_h {
            col[y] = buf[y * stride + x];
        }
        forward_53_1d(&mut col);
        for y in 0..sub_h {
            buf[y * stride + x] = col[y];
        }
    }
}

/// Helper: 1-level inverse 5/3 on the top-left sub-rectangle.
fn inverse_53_2d_one_level_sub(buf: &mut [i32], stride: usize, sub_w: usize, sub_h: usize) {
    let mut col = vec![0i32; sub_h];
    for x in 0..sub_w {
        for y in 0..sub_h {
            col[y] = buf[y * stride + x];
        }
        inverse_53_1d(&mut col);
        for y in 0..sub_h {
            buf[y * stride + x] = col[y];
        }
    }
    for y in 0..sub_h {
        let row = &mut buf[y * stride..y * stride + sub_w];
        inverse_53_1d(row);
    }
}

/// De-interleave a single-level 5/3-transformed buffer into the
/// conventional quadrant layout (`LL` top-left, `HL` top-right,
/// `LH` bottom-left, `HH` bottom-right). The forward / inverse
/// functions above operate in *interleaved* form because lifting is
/// natural in interleaved form; this helper produces the
/// quadrant-laid-out form needed by the entropy stage in round 2.
pub fn deinterleave_subbands(interleaved: &[i32], width: usize, height: usize) -> Vec<i32> {
    let half_w = width.div_ceil(2);
    let half_h = height.div_ceil(2);
    let mut out = vec![0i32; width * height];
    for y in 0..height {
        for x in 0..width {
            let v = interleaved[y * width + x];
            let (qx, qy) = (x % 2, y % 2);
            let (bx, by) = (x / 2, y / 2);
            // Quadrant offset within the dst buffer.
            let (off_x, off_y) = match (qx, qy) {
                (0, 0) => (0, 0),           // LL
                (1, 0) => (half_w, 0),      // HL
                (0, 1) => (0, half_h),      // LH
                (1, 1) => (half_w, half_h), // HH
                _ => unreachable!(),
            };
            out[(off_y + by) * width + (off_x + bx)] = v;
        }
    }
    out
}
