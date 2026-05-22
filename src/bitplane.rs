//! Bit-plane scanner -- significance, refinement, and sign passes that
//! drive the binary arithmetic coder.
//!
//! ICER's compressed segment body is built MSB-down by walking each
//! bit-plane of the wavelet coefficient buffer through three passes
//! (IPN 42-155 §III.B):
//!
//!   1. **Significance** -- for every coefficient still insignificant,
//!      send a single bit indicating whether bit `bp` is the first
//!      magnitude bit set; if so, also send the sign bit.
//!   2. **Refinement** -- for every coefficient that became significant
//!      in an earlier bit-plane, send bit `bp` of its magnitude.
//!   3. **(Cleanup)** -- IPN 42-155 §III.B notes that ICER (unlike
//!      JPEG 2000 EBCOT) folds the cleanup pass into the significance
//!      pass for some configurations; this implementation uses the
//!      merged variant (significance covers cleanup), matching the
//!      paper's "two-pass per bit-plane" reading of §III.B.
//!
//! ## Stripe-ordered scan (IPN 42-155 §III.B)
//!
//! The bit-plane scanner uses **stripe-ordered** processing. The image
//! is partitioned into horizontal stripes of height `STRIPE_HEIGHT`
//! (default 4 rows per IPN 42-155 §III.B). Within each bit-plane, all
//! passes process one full stripe before moving to the next. Within a
//! stripe, the significance pass processes left-to-right, top-to-bottom
//! (raster within the stripe), and the refinement pass follows in the
//! same order. This order maximises context-pattern locality because
//! stripes processed together share neighbour state.
//!
//! ## Multi-packet encoding (IPN 42-155 §IV)
//!
//! Each bit-plane's significance and refinement data are emitted as
//! separate packets per IPN 42-155 §IV. For a segment with bit-plane
//! count `Q`, the encoder produces `2*Q` packets in priority order:
//! for bit-plane `bp` from MSB (bp=0) to LSB (bp=Q-1):
//!   - Packet `(2*bp)`:   significance + sign data for bit-plane `bp`
//!   - Packet `(2*bp+1)`: refinement data for bit-plane `bp`
//!
//! Each packet is independently arithmetic-coded (fresh `ArithEncoder`
//! per packet) so truncated streams reconstruct at lower quality -- the
//! fundamental loss-tolerance property of ICER.
//!
//! ## Self-contained codec
//!
//! [`encode_bitplanes`] returns a `Vec<EncodedPacket>` (one pair per
//! bit-plane). [`decode_bitplanes`] reconstructs from that same packed
//! representation. The legacy `encode_bitplanes_single` /
//! `decode_bitplanes_single` path (single concatenated body) is kept for
//! the segment tests that predate the multi-packet refactor.

use crate::arith::{ArithDecoder, ArithEncoder};
use crate::context::{
    refinement_context, sign_context, sign_prediction_flip, significance_context, ContextModel,
    CONTEXT_COUNT,
};
use crate::error::{IcerError, Result};

/// Height of each scan stripe in rows. IPN 42-155 §III.B uses 4 rows.
pub const STRIPE_HEIGHT: usize = 4;

/// Describes one wavelet coefficient sub-band's bit-plane scan input.
/// `coeffs` holds the signed wavelet coefficients in raster scan
/// order (`width * height` samples). `q` is the bit-plane count from
/// MSB to LSB inclusive -- bit-plane index 0 is the MSB of the largest
/// magnitude in the buffer.
#[derive(Debug)]
pub struct BitPlaneInput<'a> {
    pub coeffs: &'a [i32],
    pub width: usize,
    pub height: usize,
    pub q: u8,
}

impl BitPlaneInput<'_> {
    fn validate(&self) -> Result<()> {
        if self.coeffs.len() != self.width * self.height {
            return Err(IcerError::invalid(format!(
                "bit-plane input: {} coeffs but width*height = {}",
                self.coeffs.len(),
                self.width * self.height
            )));
        }
        if self.q == 0 || self.q > 31 {
            return Err(IcerError::invalid(format!(
                "bit-plane count {} outside (0,31]",
                self.q
            )));
        }
        Ok(())
    }
}

/// One encoded packet body: the arithmetic-coded bytes for a single
/// pass of a single bit-plane.
#[derive(Debug, Clone)]
pub struct EncodedPacket {
    /// Bit-plane index (0 = MSB, Q-1 = LSB).
    pub bit_plane: u8,
    /// `true` for the significance+sign pass; `false` for the
    /// refinement pass.
    pub is_significance: bool,
    /// Entropy-coded body bytes.
    pub body: Vec<u8>,
    /// Distortion-reduction estimate (round-91 rate-distortion budget
    /// pruning, IPN 42-155 §IV.B rate-allocation principle).
    ///
    /// Computed during encoding as the per-packet contribution to the
    /// mean-squared-error reduction the decoder gains when this packet
    /// is included. Units are square-of-coefficient-units (i.e. the
    /// reduction in `sum (coeff - reconstructed_coeff)^2` summed over
    /// every coefficient touched in this packet).
    ///
    /// Model (clean-room from first principles, no IPN reference table):
    ///
    /// * **Significance pass** at bit-plane `bp`: every coefficient that
    ///   becomes significant in this packet transitions its
    ///   reconstruction from `0` to the mid-point of `[2^bp,
    ///   2^(bp+1))`, i.e. `1.5 * 2^bp` in magnitude. The expected MSE
    ///   reduction per newly-significant coefficient is `(1.5 *
    ///   2^bp)^2 - (2^bp)^2 / 12` ≈ `2.17 * 4^bp` (the first term is
    ///   the squared distance the reconstruction moves; the second is
    ///   the residual quantisation variance over the
    ///   bin). Approximated here as `2 * 4^bp` per coefficient.
    /// * **Refinement pass** at bit-plane `bp`: each refined coefficient
    ///   halves its quantisation-bin width from `2^(bp+1)` to `2^bp`,
    ///   reducing the per-coefficient MSE by `3 * 4^bp / 12 = 4^bp /
    ///   4`. We approximate as `4^bp / 4` per refined bit.
    ///
    /// Together with `body.len()`, the
    /// [`crate::encoder::EncodeOptions::with_rd_budget`] path computes
    /// `delta_distortion / body.len()` as the cost-per-byte score and
    /// greedily includes packets in descending score order, subject to
    /// the MSB-down decoding dependency graph (significance at bit-plane
    /// `bp` requires significance at all higher bit-planes; refinement
    /// at `bp` requires significance at `bp`).
    pub delta_distortion: f64,
}

/// Encode a coefficient buffer into per-bit-plane, per-pass packets
/// using stripe-ordered scanning (IPN 42-155 §III.B + §IV).
///
/// Returns one pair of [`EncodedPacket`] per bit-plane (significance
/// first, then refinement). The `Q` value from `input.q` is the number
/// of bit-planes processed.
pub fn encode_bitplanes(input: &BitPlaneInput<'_>) -> Result<Vec<EncodedPacket>> {
    input.validate()?;
    let n = input.coeffs.len();
    let q = input.q as usize;

    let mut significant = vec![false; n];
    let mut sign = vec![false; n];

    let mut packets = Vec::with_capacity(2 * q);

    for bp_idx in 0..q {
        let bp = q - 1 - bp_idx; // Process MSB first: bp = q-1 down to 0.
        let bp_weight = 4f64.powi(bp as i32);

        // --- Significance + sign pass (fresh coder, stripe order) ---
        let mut sig_model = ContextModel::new();
        let mut sig_enc = ArithEncoder::new();
        let sig_before = significant.iter().filter(|&&s| s).count();

        encode_significance_pass(
            &mut sig_enc,
            &mut sig_model,
            input.coeffs,
            &mut significant,
            &mut sign,
            input.width,
            input.height,
            bp,
        );

        let sig_after = significant.iter().filter(|&&s| s).count();
        let newly_sig = sig_after.saturating_sub(sig_before);
        // Distortion-reduction model: each newly-significant coefficient
        // moves reconstruction from 0 to mid-bin `1.5 * 2^bp`; per-coef
        // MSE drop ≈ 2.17 * 4^bp (see EncodedPacket::delta_distortion
        // docstring). We approximate by `2.0 * 4^bp`. Round-91 R-D
        // pruning per IPN 42-155 §IV.B rate-allocation principle.
        let sig_dist = (newly_sig as f64) * 2.0 * bp_weight;

        packets.push(EncodedPacket {
            bit_plane: bp_idx as u8,
            is_significance: true,
            body: sig_enc.finish(),
            delta_distortion: sig_dist,
        });

        // --- Refinement pass (fresh coder, stripe order) ---
        let mut ref_model = ContextModel::new();
        let mut ref_enc = ArithEncoder::new();
        let refined_count = count_refinement_coefficients(
            input.coeffs,
            &significant,
            input.width,
            input.height,
            bp,
        );

        encode_refinement_pass(
            &mut ref_enc,
            &mut ref_model,
            input.coeffs,
            &significant,
            input.width,
            input.height,
            bp,
        );

        // Distortion-reduction model: each refined coefficient halves
        // its quantisation bin, dropping per-coef MSE by `4^bp / 4`.
        let ref_dist = (refined_count as f64) * 0.25 * bp_weight;

        packets.push(EncodedPacket {
            bit_plane: bp_idx as u8,
            is_significance: false,
            body: ref_enc.finish(),
            delta_distortion: ref_dist,
        });
    }

    Ok(packets)
}

/// Count the coefficients that will be visited by the refinement pass at
/// bit-plane `bp` (round 91 helper for the R-D distortion estimate).
/// Matches the iteration inside [`encode_refinement_pass`] exactly.
fn count_refinement_coefficients(
    coeffs: &[i32],
    significant: &[bool],
    width: usize,
    height: usize,
    bp: usize,
) -> usize {
    let mut count = 0usize;
    let mut stripe_start = 0;
    while stripe_start < height {
        let stripe_end = (stripe_start + STRIPE_HEIGHT).min(height);
        for y in stripe_start..stripe_end {
            for x in 0..width {
                let i = y * width + x;
                if !significant[i] {
                    continue;
                }
                let m = coeffs[i].unsigned_abs();
                if highest_set_bit(m) == Some(bp as u32) {
                    continue;
                }
                count += 1;
            }
        }
        stripe_start += STRIPE_HEIGHT;
    }
    count
}

/// Decode coefficient buffer from per-bit-plane packets produced by
/// [`encode_bitplanes`]. Reconstructs `width * height` signed integers.
pub fn decode_bitplanes_multi(
    packets: &[EncodedPacket],
    width: usize,
    height: usize,
    q: u8,
) -> Result<Vec<i32>> {
    let n = width * height;
    if q == 0 || q > 31 {
        return Err(IcerError::invalid(format!(
            "bit-plane count {q} outside (0,31]"
        )));
    }
    let q_usize = q as usize;

    let mut significant = vec![false; n];
    let mut sign = vec![false; n];
    let mut mag = vec![0u32; n];

    for bp_idx in 0..q_usize {
        let bp = q_usize - 1 - bp_idx;

        // Find the significance packet for this bit-plane index.
        let sig_body = packets
            .iter()
            .find(|p| p.bit_plane == bp_idx as u8 && p.is_significance)
            .map(|p| p.body.as_slice())
            .unwrap_or(&[]);

        let mut sig_model = ContextModel::new();
        let mut sig_dec = ArithDecoder::new(sig_body)?;

        decode_significance_pass(
            &mut sig_dec,
            &mut sig_model,
            &mut significant,
            &mut sign,
            &mut mag,
            width,
            height,
            bp,
        )?;

        // Find the refinement packet for this bit-plane index.
        let ref_body = packets
            .iter()
            .find(|p| p.bit_plane == bp_idx as u8 && !p.is_significance)
            .map(|p| p.body.as_slice())
            .unwrap_or(&[]);

        let mut ref_model = ContextModel::new();
        let mut ref_dec = ArithDecoder::new(ref_body)?;

        decode_refinement_pass(
            &mut ref_dec,
            &mut ref_model,
            &significant,
            &mut mag,
            width,
            height,
            bp,
        )?;
    }

    let out = mag
        .iter()
        .zip(sign.iter())
        .map(|(&m, &s)| {
            let v = m as i32;
            if s {
                -v
            } else {
                v
            }
        })
        .collect();
    Ok(out)
}

/// Significance + sign pass for one bit-plane, stripe-ordered.
/// Modifies `significant` and `sign` in place.
#[allow(clippy::too_many_arguments)]
fn encode_significance_pass(
    enc: &mut ArithEncoder,
    model: &mut ContextModel,
    coeffs: &[i32],
    significant: &mut [bool],
    sign: &mut [bool],
    width: usize,
    height: usize,
    bp: usize,
) {
    let mut stripe_start = 0;
    while stripe_start < height {
        let stripe_end = (stripe_start + STRIPE_HEIGHT).min(height);
        for y in stripe_start..stripe_end {
            for x in 0..width {
                let i = y * width + x;
                if significant[i] {
                    continue;
                }
                let pat = neighbour_significance_pattern(significant, width, height, x, y);
                let ctx = significance_context(pat);
                debug_assert!(ctx < CONTEXT_COUNT);

                let mag = coeffs[i].unsigned_abs();
                let bit = ((mag >> bp) & 1) as u8;

                let (num, den) = model.probability(ctx);
                enc.encode_bit(bit, num, den);
                model.observe(ctx, bit);

                if bit == 1 {
                    significant[i] = true;
                    sign[i] = coeffs[i] < 0;

                    // Sign bit with sign-flip convention (IPN 42-155 §III.B).
                    let (h_pat, v_pat) =
                        neighbour_sign_pattern(significant, sign, width, height, x, y);
                    let sctx = sign_context(h_pat, v_pat);
                    debug_assert!(sctx < CONTEXT_COUNT);
                    let flip = sign_prediction_flip(h_pat, v_pat);
                    // Encode the (possibly flipped) sign bit.
                    let raw_sign = u8::from(sign[i]);
                    let coded_sign = if flip { 1 - raw_sign } else { raw_sign };
                    let (sn, sd) = model.probability(sctx);
                    enc.encode_bit(coded_sign, sn, sd);
                    model.observe(sctx, coded_sign);
                }
            }
        }
        stripe_start += STRIPE_HEIGHT;
    }
}

/// Significance + sign decode pass for one bit-plane, stripe-ordered.
#[allow(clippy::too_many_arguments)]
fn decode_significance_pass(
    dec: &mut ArithDecoder<'_>,
    model: &mut ContextModel,
    significant: &mut [bool],
    sign: &mut [bool],
    mag: &mut [u32],
    width: usize,
    height: usize,
    bp: usize,
) -> Result<()> {
    let mut stripe_start = 0;
    while stripe_start < height {
        let stripe_end = (stripe_start + STRIPE_HEIGHT).min(height);
        for y in stripe_start..stripe_end {
            for x in 0..width {
                let i = y * width + x;
                if significant[i] {
                    continue;
                }
                let pat = neighbour_significance_pattern(significant, width, height, x, y);
                let ctx = significance_context(pat);
                let (num, den) = model.probability(ctx);
                let bit = dec.decode_bit(num, den)?;
                model.observe(ctx, bit);

                if bit == 1 {
                    significant[i] = true;
                    mag[i] |= 1u32 << bp;
                    let (h_pat, v_pat) =
                        neighbour_sign_pattern(significant, sign, width, height, x, y);
                    let sctx = sign_context(h_pat, v_pat);
                    let flip = sign_prediction_flip(h_pat, v_pat);
                    let (sn, sd) = model.probability(sctx);
                    let coded_sign = dec.decode_bit(sn, sd)?;
                    model.observe(sctx, coded_sign);
                    // Undo the sign flip.
                    let raw_sign = if flip { 1 - coded_sign } else { coded_sign };
                    sign[i] = raw_sign == 1;
                }
            }
        }
        stripe_start += STRIPE_HEIGHT;
    }
    Ok(())
}

/// Refinement pass for one bit-plane, stripe-ordered.
fn encode_refinement_pass(
    enc: &mut ArithEncoder,
    model: &mut ContextModel,
    coeffs: &[i32],
    significant: &[bool],
    width: usize,
    height: usize,
    bp: usize,
) {
    let mut stripe_start = 0;
    while stripe_start < height {
        let stripe_end = (stripe_start + STRIPE_HEIGHT).min(height);
        for y in stripe_start..stripe_end {
            for x in 0..width {
                let i = y * width + x;
                if !significant[i] {
                    continue;
                }
                let m = coeffs[i].unsigned_abs();
                // Skip coefficients that became significant in THIS bit-plane
                // (they contribute to the significance pass, not refinement).
                if highest_set_bit(m) == Some(bp as u32) {
                    continue;
                }
                let has_sig_neighbour =
                    neighbour_significance_pattern(significant, width, height, x, y) != 0;
                let rctx = refinement_context(false, has_sig_neighbour);
                debug_assert!(rctx < CONTEXT_COUNT);
                let bit = ((m >> bp) & 1) as u8;
                let (num, den) = model.probability(rctx);
                enc.encode_bit(bit, num, den);
                model.observe(rctx, bit);
            }
        }
        stripe_start += STRIPE_HEIGHT;
    }
}

/// Refinement decode pass for one bit-plane, stripe-ordered.
fn decode_refinement_pass(
    dec: &mut ArithDecoder<'_>,
    model: &mut ContextModel,
    significant: &[bool],
    mag: &mut [u32],
    width: usize,
    height: usize,
    bp: usize,
) -> Result<()> {
    let mut stripe_start = 0;
    while stripe_start < height {
        let stripe_end = (stripe_start + STRIPE_HEIGHT).min(height);
        for y in stripe_start..stripe_end {
            for x in 0..width {
                let i = y * width + x;
                if !significant[i] {
                    continue;
                }
                if highest_set_bit(mag[i]) == Some(bp as u32) {
                    continue;
                }
                let has_sig_neighbour =
                    neighbour_significance_pattern(significant, width, height, x, y) != 0;
                let rctx = refinement_context(false, has_sig_neighbour);
                let (num, den) = model.probability(rctx);
                let bit = dec.decode_bit(num, den)?;
                model.observe(rctx, bit);
                if bit == 1 {
                    mag[i] |= 1u32 << bp;
                }
            }
        }
        stripe_start += STRIPE_HEIGHT;
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// Legacy single-packet codec (used by the old segment encode/decode path).
// The multi-packet path (encode_bitplanes / decode_bitplanes_multi) is
// preferred for new code; this path is kept for the existing tests that
// use the single-concatenated-body approach.
// ---------------------------------------------------------------------------

/// Encode a coefficient buffer into a single arithmetic-coded payload
/// (stripe-ordered, but all passes concatenated into one body).
///
/// Returns the entropy-coded body. The caller wraps this in a
/// `PacketHeader`.
pub fn encode_bitplanes_single(input: &BitPlaneInput<'_>) -> Result<Vec<u8>> {
    input.validate()?;
    let n = input.coeffs.len();
    let q = input.q as usize;

    let mut significant = vec![false; n];
    let mut sign = vec![false; n];
    let mut model = ContextModel::new();
    let mut enc = ArithEncoder::new();

    for bp_idx in 0..q {
        let bp = q - 1 - bp_idx;

        // Significance + sign pass (stripe order).
        let mut stripe_start = 0;
        while stripe_start < input.height {
            let stripe_end = (stripe_start + STRIPE_HEIGHT).min(input.height);
            for y in stripe_start..stripe_end {
                for x in 0..input.width {
                    let i = y * input.width + x;
                    if significant[i] {
                        continue;
                    }
                    let pat = neighbour_significance_pattern(
                        &significant,
                        input.width,
                        input.height,
                        x,
                        y,
                    );
                    let ctx = significance_context(pat);
                    debug_assert!(ctx < CONTEXT_COUNT);
                    let mag = input.coeffs[i].unsigned_abs();
                    let bit = ((mag >> bp) & 1) as u8;
                    let (num, den) = model.probability(ctx);
                    enc.encode_bit(bit, num, den);
                    model.observe(ctx, bit);
                    if bit == 1 {
                        significant[i] = true;
                        sign[i] = input.coeffs[i] < 0;
                        let (h_pat, v_pat) = neighbour_sign_pattern(
                            &significant,
                            &sign,
                            input.width,
                            input.height,
                            x,
                            y,
                        );
                        let sctx = sign_context(h_pat, v_pat);
                        debug_assert!(sctx < CONTEXT_COUNT);
                        let flip = sign_prediction_flip(h_pat, v_pat);
                        let raw_sign = u8::from(sign[i]);
                        let coded_sign = if flip { 1 - raw_sign } else { raw_sign };
                        let (sn, sd) = model.probability(sctx);
                        enc.encode_bit(coded_sign, sn, sd);
                        model.observe(sctx, coded_sign);
                    }
                }
            }
            stripe_start += STRIPE_HEIGHT;
        }

        // Refinement pass (stripe order).
        let mut stripe_start = 0;
        while stripe_start < input.height {
            let stripe_end = (stripe_start + STRIPE_HEIGHT).min(input.height);
            for y in stripe_start..stripe_end {
                for x in 0..input.width {
                    let i = y * input.width + x;
                    if !significant[i] {
                        continue;
                    }
                    let mag = input.coeffs[i].unsigned_abs();
                    if highest_set_bit(mag) == Some(bp as u32) {
                        continue;
                    }
                    let has_sig_neighbour = neighbour_significance_pattern(
                        &significant,
                        input.width,
                        input.height,
                        x,
                        y,
                    ) != 0;
                    let rctx = refinement_context(false, has_sig_neighbour);
                    debug_assert!(rctx < CONTEXT_COUNT);
                    let bit = ((mag >> bp) & 1) as u8;
                    let (num, den) = model.probability(rctx);
                    enc.encode_bit(bit, num, den);
                    model.observe(rctx, bit);
                }
            }
            stripe_start += STRIPE_HEIGHT;
        }
    }

    Ok(enc.finish())
}

/// Decode a coefficient buffer from a single arithmetic-coded payload
/// (legacy single-body path matching `encode_bitplanes_single`).
pub fn decode_bitplanes(bytes: &[u8], width: usize, height: usize, q: u8) -> Result<Vec<i32>> {
    let n = width * height;
    if q == 0 || q > 31 {
        return Err(IcerError::invalid(format!(
            "bit-plane count {q} outside (0,31]"
        )));
    }
    let q_usize = q as usize;

    let mut significant = vec![false; n];
    let mut sign = vec![false; n];
    let mut mag = vec![0u32; n];

    let mut model = ContextModel::new();
    let mut dec = ArithDecoder::new(bytes)?;

    for bp_idx in 0..q_usize {
        let bp = q_usize - 1 - bp_idx;

        // Significance + sign pass (stripe order).
        let mut stripe_start = 0;
        while stripe_start < height {
            let stripe_end = (stripe_start + STRIPE_HEIGHT).min(height);
            for y in stripe_start..stripe_end {
                for x in 0..width {
                    let i = y * width + x;
                    if significant[i] {
                        continue;
                    }
                    let pat = neighbour_significance_pattern(&significant, width, height, x, y);
                    let ctx = significance_context(pat);
                    let (num, den) = model.probability(ctx);
                    let bit = dec.decode_bit(num, den)?;
                    model.observe(ctx, bit);
                    if bit == 1 {
                        significant[i] = true;
                        mag[i] |= 1u32 << bp;
                        let (h_pat, v_pat) =
                            neighbour_sign_pattern(&significant, &sign, width, height, x, y);
                        let sctx = sign_context(h_pat, v_pat);
                        let flip = sign_prediction_flip(h_pat, v_pat);
                        let (sn, sd) = model.probability(sctx);
                        let coded_sign = dec.decode_bit(sn, sd)?;
                        model.observe(sctx, coded_sign);
                        let raw_sign = if flip { 1 - coded_sign } else { coded_sign };
                        sign[i] = raw_sign == 1;
                    }
                }
            }
            stripe_start += STRIPE_HEIGHT;
        }

        // Refinement pass (stripe order).
        let mut stripe_start = 0;
        while stripe_start < height {
            let stripe_end = (stripe_start + STRIPE_HEIGHT).min(height);
            for y in stripe_start..stripe_end {
                for x in 0..width {
                    let i = y * width + x;
                    if !significant[i] {
                        continue;
                    }
                    if highest_set_bit(mag[i]) == Some(bp as u32) {
                        continue;
                    }
                    let has_sig_neighbour =
                        neighbour_significance_pattern(&significant, width, height, x, y) != 0;
                    let rctx = refinement_context(false, has_sig_neighbour);
                    let (num, den) = model.probability(rctx);
                    let bit = dec.decode_bit(num, den)?;
                    model.observe(rctx, bit);
                    if bit == 1 {
                        mag[i] |= 1u32 << bp;
                    }
                }
            }
            stripe_start += STRIPE_HEIGHT;
        }
    }

    let out = mag
        .iter()
        .zip(sign.iter())
        .map(|(&m, &s)| {
            let v = m as i32;
            if s {
                -v
            } else {
                v
            }
        })
        .collect();
    Ok(out)
}

// ---------------------------------------------------------------------------
// Neighbour helpers
// ---------------------------------------------------------------------------

/// Pack the 8-neighbour significance pattern for `(x, y)` into a `u8`
/// using the layout documented in [`crate::context::significance_context`]
/// (NW=bit0, N=bit1, NE=bit2, W=bit3, E=bit4, SW=bit5, S=bit6, SE=bit7).
fn neighbour_significance_pattern(
    significant: &[bool],
    width: usize,
    height: usize,
    x: usize,
    y: usize,
) -> u8 {
    let mut pat = 0u8;
    let neighbours = [
        // (dx, dy, bit)
        (-1isize, -1isize, 0u8),
        (0, -1, 1),
        (1, -1, 2),
        (-1, 0, 3),
        (1, 0, 4),
        (-1, 1, 5),
        (0, 1, 6),
        (1, 1, 7),
    ];
    for (dx, dy, bit) in neighbours {
        let nx = x as isize + dx;
        let ny = y as isize + dy;
        if nx >= 0 && ny >= 0 && (nx as usize) < width && (ny as usize) < height {
            let idx = (ny as usize) * width + (nx as usize);
            if significant[idx] {
                pat |= 1 << bit;
            }
        }
    }
    pat
}

/// Build the `(h_pattern, v_pattern)` packed-pair fed into
/// [`crate::context::sign_context`] / [`crate::context::sign_prediction_flip`].
///
/// Layout per context.rs sign_context doc:
///   bits 0,1 = (neighbour-A significant, neighbour-A negative)
///   bits 2,3 = (neighbour-B significant, neighbour-B negative)
///
/// Horizontal: A=W, B=E. Vertical: A=N, B=S.
fn neighbour_sign_pattern(
    significant: &[bool],
    sign: &[bool],
    width: usize,
    height: usize,
    x: usize,
    y: usize,
) -> (u8, u8) {
    let h = pair_pattern(
        significant,
        sign,
        width,
        height,
        x as isize - 1,
        y as isize,
        x as isize + 1,
        y as isize,
    );
    let v = pair_pattern(
        significant,
        sign,
        width,
        height,
        x as isize,
        y as isize - 1,
        x as isize,
        y as isize + 1,
    );
    (h, v)
}

#[allow(clippy::too_many_arguments)]
fn pair_pattern(
    significant: &[bool],
    sign: &[bool],
    width: usize,
    height: usize,
    ax: isize,
    ay: isize,
    bx: isize,
    by: isize,
) -> u8 {
    let mut p = 0u8;
    for (i, (cx, cy)) in [(ax, ay), (bx, by)].iter().enumerate() {
        if *cx >= 0 && *cy >= 0 && (*cx as usize) < width && (*cy as usize) < height {
            let idx = (*cy as usize) * width + (*cx as usize);
            if significant[idx] {
                p |= 1 << (i * 2); // significant bit
                if sign[idx] {
                    p |= 1 << (i * 2 + 1); // negative bit
                }
            }
        }
    }
    p
}

fn highest_set_bit(x: u32) -> Option<u32> {
    if x == 0 {
        None
    } else {
        Some(31 - x.leading_zeros())
    }
}

/// Pick the smallest `q` in `[1,31]` such that every coefficient's
/// magnitude fits in `q` bits. Used by the encoder to size the
/// bit-plane Q field of the segment header.
pub fn select_bit_plane_count(coeffs: &[i32]) -> u8 {
    let max_abs = coeffs.iter().map(|c| c.unsigned_abs()).max().unwrap_or(0);
    if max_abs == 0 {
        // All-zero buffer still gets a single bit-plane of context bits
        // so the bit-plane scanner has work to do.
        return 1;
    }
    let bits = 32 - max_abs.leading_zeros();
    bits.clamp(1, 31) as u8
}

#[cfg(test)]
mod tests {
    use super::*;

    fn lcg_signal(n: usize, range: i32, seed: u64) -> Vec<i32> {
        let mut s = seed.wrapping_mul(0x9E37_79B9_7F4A_7C15);
        (0..n)
            .map(|_| {
                s = s
                    .wrapping_mul(6364136223846793005)
                    .wrapping_add(1442695040888963407);
                let v = (s >> 33) as i32;
                v % range
            })
            .collect()
    }

    #[test]
    fn empty_neighbours_are_zero() {
        let sig = vec![false; 4];
        assert_eq!(
            neighbour_significance_pattern(&sig, 2, 2, 0, 0),
            0,
            "all-insignificant neighbourhood"
        );
    }

    #[test]
    fn highest_set_bit_basics() {
        assert_eq!(highest_set_bit(0), None);
        assert_eq!(highest_set_bit(1), Some(0));
        assert_eq!(highest_set_bit(2), Some(1));
        assert_eq!(highest_set_bit(0x8000_0000), Some(31));
    }

    #[test]
    fn select_bit_plane_count_basics() {
        assert_eq!(select_bit_plane_count(&[0, 0, 0]), 1);
        assert_eq!(select_bit_plane_count(&[1, 0, -1]), 1);
        assert_eq!(select_bit_plane_count(&[3, -2]), 2);
        assert_eq!(select_bit_plane_count(&[255, -255]), 8);
    }

    #[test]
    fn bitplane_roundtrip_tiny_zero_single() {
        let coeffs = vec![0i32; 16];
        let input = BitPlaneInput {
            coeffs: &coeffs,
            width: 4,
            height: 4,
            q: 1,
        };
        let bytes = encode_bitplanes_single(&input).unwrap();
        let out = decode_bitplanes(&bytes, 4, 4, 1).unwrap();
        assert_eq!(out, coeffs);
    }

    #[test]
    fn bitplane_roundtrip_signed_random_single() {
        let w = 8;
        let h = 8; // Must be a multiple of STRIPE_HEIGHT for clean stripe alignment
        let coeffs = lcg_signal(w * h, 32, 0xCAFE);
        let q = select_bit_plane_count(&coeffs);
        let input = BitPlaneInput {
            coeffs: &coeffs,
            width: w,
            height: h,
            q,
        };
        let bytes = encode_bitplanes_single(&input).unwrap();
        let out = decode_bitplanes(&bytes, w, h, q).unwrap();
        assert_eq!(out, coeffs);
    }

    #[test]
    fn bitplane_roundtrip_large_dynamic_range_single() {
        let w = 16;
        let h = 16;
        let coeffs = lcg_signal(w * h, 4096, 0xBEEF);
        let q = select_bit_plane_count(&coeffs);
        let input = BitPlaneInput {
            coeffs: &coeffs,
            width: w,
            height: h,
            q,
        };
        let bytes = encode_bitplanes_single(&input).unwrap();
        let out = decode_bitplanes(&bytes, w, h, q).unwrap();
        assert_eq!(out, coeffs);
    }

    #[test]
    fn bitplane_roundtrip_multi_packet() {
        let w = 8;
        let h = 8;
        let coeffs = lcg_signal(w * h, 128, 0xDEAD);
        let q = select_bit_plane_count(&coeffs);
        let input = BitPlaneInput {
            coeffs: &coeffs,
            width: w,
            height: h,
            q,
        };
        let packets = encode_bitplanes(&input).unwrap();
        assert_eq!(
            packets.len(),
            2 * q as usize,
            "should have 2 packets per bit-plane"
        );
        let out = decode_bitplanes_multi(&packets, w, h, q).unwrap();
        assert_eq!(out, coeffs);
    }

    #[test]
    fn bitplane_multi_packet_all_zero() {
        let w = 4;
        let h = 4;
        let coeffs = vec![0i32; w * h];
        let q = 1u8;
        let input = BitPlaneInput {
            coeffs: &coeffs,
            width: w,
            height: h,
            q,
        };
        let packets = encode_bitplanes(&input).unwrap();
        let out = decode_bitplanes_multi(&packets, w, h, q).unwrap();
        assert_eq!(out, coeffs);
    }

    #[test]
    fn stripe_scan_matches_for_multiples_of_stripe_height() {
        // When height is a multiple of STRIPE_HEIGHT, the single and
        // multi-packet paths must produce identical decoded output.
        let w = 8;
        let h = 8;
        let coeffs = lcg_signal(w * h, 64, 0xF00D);
        let q = select_bit_plane_count(&coeffs);
        let input = BitPlaneInput {
            coeffs: &coeffs,
            width: w,
            height: h,
            q,
        };
        let single_body = encode_bitplanes_single(&input).unwrap();
        let single_out = decode_bitplanes(&single_body, w, h, q).unwrap();
        let packets = encode_bitplanes(&input).unwrap();
        let multi_out = decode_bitplanes_multi(&packets, w, h, q).unwrap();
        assert_eq!(single_out, coeffs, "single path must round-trip");
        assert_eq!(multi_out, coeffs, "multi path must round-trip");
    }
}
