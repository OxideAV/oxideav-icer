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
    magnitude_context, neighbour_counts, sign_context, sign_context_subband, sign_prediction_flip,
    sign_prediction_flip_subband, significance_context, significance_context_subband, ContextModel,
    MagnitudeContext, CONTEXT_COUNT, UNCODED_P1,
};
use crate::error::{IcerError, Result};
use crate::priority::{classify_position, subband_stride, SubbandType};

/// Height of each scan stripe in rows. IPN 42-155 §III.B uses 4 rows.
pub const STRIPE_HEIGHT: usize = 4;

/// Bit mask (within the 8-neighbour significance pattern produced by
/// [`neighbour_significance_pattern`]) selecting the four horizontally /
/// vertically adjacent pixels: N=bit1, W=bit3, E=bit4, S=bit6. The
/// IPN 42-155 §III.B category-1 magnitude context (9 vs 10) keys on
/// whether *any* of these four (not the diagonals) is significant.
const HV_NEIGHBOUR_MASK: u8 = 0b0100_1010;

/// `true` iff at least one horizontally or vertically adjacent pixel is
/// significant, given the packed 8-neighbour pattern.
#[inline]
fn has_hv_significant(pattern: u8) -> bool {
    pattern & HV_NEIGHBOUR_MASK != 0
}

/// Significance context for coefficient `(x, y)` from its 8-neighbour
/// significance `pattern`, dispatched on the coefficient's subband per
/// IPN 42-155 §III.B.
///
/// When `levels == 0` the scanner is subband-agnostic and uses the
/// uniform [`significance_context`] classification (legacy / unit-test
/// path). When `levels >= 1` the coefficient's `(SubbandType, level)` is
/// resolved via [`classify_position`] and the spec-exact §III.B Table 6
/// (LL/LH/HL) or Table 7 (HH) is used, with the HL context-template
/// transpose. The neighbour-count granularity also sharpens here: the
/// uniform path collapses V and D to "0 / 1+", while the spec tables key
/// on the full `(h, v, d)` counts.
#[inline]
fn significance_ctx_for(pattern: u8, x: usize, y: usize, levels: u8) -> usize {
    if levels == 0 {
        return significance_context(pattern);
    }
    let (kind, _) = classify_position(x, y, levels);
    let (h, v, d) = neighbour_counts(pattern);
    significance_context_subband(h, v, d, kind == SubbandType::Hh, kind == SubbandType::Hl)
}

/// `(sign_context, predicted_sign_is_negative)` for coefficient `(x, y)`,
/// dispatched on the subband's HL transpose per IPN 42-155 §III.B Table 8.
/// `levels == 0` keeps the subband-agnostic Table 8 lookup.
#[inline]
fn sign_ctx_for(h_pat: u8, v_pat: u8, x: usize, y: usize, levels: u8) -> (usize, bool) {
    if levels == 0 {
        return (
            sign_context(h_pat, v_pat),
            sign_prediction_flip(h_pat, v_pat),
        );
    }
    let (kind, _) = classify_position(x, y, levels);
    let is_hl = kind == SubbandType::Hl;
    (
        sign_context_subband(h_pat, v_pat, is_hl),
        sign_prediction_flip_subband(h_pat, v_pat, is_hl),
    )
}

/// Describes one wavelet coefficient sub-band's bit-plane scan input.
/// `coeffs` holds the signed wavelet coefficients in raster scan
/// order (`width * height` samples). `q` is the bit-plane count from
/// MSB to LSB inclusive -- bit-plane index 0 is the MSB of the largest
/// magnitude in the buffer.
///
/// `levels` is the dyadic wavelet decomposition depth used to produce
/// `coeffs`. It drives the per-coefficient subband classification
/// ([`crate::priority::classify_position`]) that selects the spec-exact
/// IPN 42-155 §III.B significance context table (Table 6 for LL/LH/HL,
/// Table 7 for HH) and the HL context-template transpose. `levels = 0`
/// disables subband awareness and falls back to the uniform
/// [`crate::context::significance_context`] classification (used by the
/// legacy single-body path and the subband-agnostic unit tests).
#[derive(Debug)]
pub struct BitPlaneInput<'a> {
    pub coeffs: &'a [i32],
    pub width: usize,
    pub height: usize,
    pub q: u8,
    /// Dyadic decomposition depth (`0` = subband-agnostic). See struct
    /// docs.
    pub levels: u8,
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
///
/// This is the unweighted variant: every coefficient contributes the
/// same per-coefficient distortion estimate regardless of its subband.
/// The rate-distortion packet selector that consumes `delta_distortion`
/// can instead optimise reconstructed-image MSE by passing a §III.A
/// per-coefficient weight map to [`encode_bitplanes_weighted`].
pub fn encode_bitplanes(input: &BitPlaneInput<'_>) -> Result<Vec<EncodedPacket>> {
    encode_bitplanes_weighted(input, None)
}

/// Like [`encode_bitplanes`], but accepts an optional per-coefficient
/// image-domain distortion weight map (IPN 42-155 §III.A).
///
/// When `weights` is `Some(w)` (length `width * height`), each packet's
/// `delta_distortion` accumulates `w[i]` for every coefficient `i` it
/// touches instead of a flat per-coefficient count. `w[i]` is the
/// image-domain MSE induced by a unit transform-domain error at
/// coefficient `i` (see [`crate::priority::subband_weight_map`]), so the
/// resulting `delta_distortion` estimates **reconstructed-image** MSE
/// reduction rather than transform-domain MSE reduction. Because ICER's
/// wavelet transforms are not unitary the two differ markedly: a unit
/// error in a low-frequency coefficient injects far more image-domain
/// distortion than the same error in a high-frequency coefficient, so
/// weighting steers the rate-distortion selector toward the packets that
/// matter most to image quality.
///
/// `weights = None` reproduces [`encode_bitplanes`] exactly (flat unit
/// weight): the self-roundtrip wire form is byte-identical, since
/// `delta_distortion` never reaches the wire — only the packet *bodies*
/// do, and those are independent of the distortion estimate.
pub fn encode_bitplanes_weighted(
    input: &BitPlaneInput<'_>,
    weights: Option<&[f64]>,
) -> Result<Vec<EncodedPacket>> {
    input.validate()?;
    let n = input.coeffs.len();
    let q = input.q as usize;
    if let Some(w) = weights {
        if w.len() != n {
            return Err(IcerError::invalid(format!(
                "weight map length {} != coeff count {}",
                w.len(),
                n
            )));
        }
    }

    let mut significant = vec![false; n];
    let mut sign = vec![false; n];
    // IPN 42-155 §III.B per-pixel category (0=insignificant, 1/2/3 =
    // magnitude bits coded so far). Persists across bit-planes; drives
    // the category-aware refinement context + category-3 uncoded bits.
    let mut cat = vec![0u8; n];

    let mut packets = Vec::with_capacity(2 * q);

    for bp_idx in 0..q {
        let bp = q - 1 - bp_idx; // Process MSB first: bp = q-1 down to 0.
        let bp_weight = 4f64.powi(bp as i32);

        // --- Significance + sign pass (fresh coder, stripe order) ---
        let mut sig_model = ContextModel::new();
        let mut sig_enc = ArithEncoder::new();
        let sig_before: Vec<bool> = significant.clone();

        encode_significance_pass(
            &mut sig_enc,
            &mut sig_model,
            input.coeffs,
            &mut significant,
            &mut sign,
            &mut cat,
            input.width,
            input.height,
            bp,
            input.levels,
        );

        // Distortion-reduction model: each newly-significant coefficient
        // moves reconstruction from 0 to mid-bin `1.5 * 2^bp`; per-coef
        // MSE drop ≈ 2.17 * 4^bp (see EncodedPacket::delta_distortion
        // docstring). We approximate by `2.0 * 4^bp`. Round-91 R-D
        // pruning per IPN 42-155 §IV.B rate-allocation principle. When a
        // §III.A weight map is supplied, each coefficient's contribution
        // is scaled by its image-domain weight `w[i]` so the estimate is
        // reconstructed-image MSE, not transform-domain MSE.
        let sig_weight_sum: f64 = match weights {
            Some(w) => significant
                .iter()
                .zip(sig_before.iter())
                .enumerate()
                .filter(|(_, (now, before))| **now && !**before)
                .map(|(i, _)| w[i])
                .sum(),
            None => significant
                .iter()
                .zip(sig_before.iter())
                .filter(|(now, before)| **now && !**before)
                .count() as f64,
        };
        let sig_dist = sig_weight_sum * 2.0 * bp_weight;

        packets.push(EncodedPacket {
            bit_plane: bp_idx as u8,
            is_significance: true,
            body: sig_enc.finish(),
            delta_distortion: sig_dist,
        });

        // --- Refinement pass (fresh coder, stripe order) ---
        let mut ref_model = ContextModel::new();
        let mut ref_enc = ArithEncoder::new();
        let ref_weight_sum = refinement_weight_sum(
            input.coeffs,
            &significant,
            input.width,
            input.height,
            bp,
            weights,
        );

        encode_refinement_pass(
            &mut ref_enc,
            &mut ref_model,
            input.coeffs,
            &significant,
            &mut cat,
            input.width,
            input.height,
            bp,
            input.levels,
        );

        // Distortion-reduction model: each refined coefficient halves
        // its quantisation bin, dropping per-coef MSE by `4^bp / 4`.
        // Scaled by the §III.A image-domain weight when supplied.
        let ref_dist = ref_weight_sum * 0.25 * bp_weight;

        packets.push(EncodedPacket {
            bit_plane: bp_idx as u8,
            is_significance: false,
            body: ref_enc.finish(),
            delta_distortion: ref_dist,
        });
    }

    Ok(packets)
}

/// Sum the §III.A image-domain weights of the coefficients that the
/// refinement pass at bit-plane `bp` will visit (round 91 helper for the
/// R-D distortion estimate). Matches the iteration inside
/// [`encode_refinement_pass`] exactly. With `weights = None` this is a
/// plain count of refined coefficients (each weight 1.0).
fn refinement_weight_sum(
    coeffs: &[i32],
    significant: &[bool],
    width: usize,
    height: usize,
    bp: usize,
    weights: Option<&[f64]>,
) -> f64 {
    let mut sum = 0.0f64;
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
                sum += match weights {
                    Some(w) => w[i],
                    None => 1.0,
                };
            }
        }
        stripe_start += STRIPE_HEIGHT;
    }
    sum
}

/// Decode coefficient buffer from per-bit-plane packets produced by
/// [`encode_bitplanes`]. Reconstructs `width * height` signed integers.
///
/// `levels` is the dyadic decomposition depth the encoder used (so the
/// decoder selects the identical subband-aware §III.B significance / sign
/// contexts); pass `0` for the subband-agnostic path.
pub fn decode_bitplanes_multi(
    packets: &[EncodedPacket],
    width: usize,
    height: usize,
    q: u8,
    levels: u8,
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
    // IPN 42-155 §III.B per-pixel category, advanced in lockstep with the
    // encoder so the category-aware refinement contexts + category-3
    // uncoded bits decode identically.
    let mut cat = vec![0u8; n];
    // Per-coefficient deepest delivered magnitude bit-plane (the §III.A
    // per-coefficient deadzone exponent). `q` means "no bit delivered"
    // (sentinel for an insignificant coefficient). A coefficient that
    // became significant at plane `s` but whose plane-`bp < s` refinement
    // packet was dropped keeps `last_bit = s`, so its reconstruction bin
    // is wider than a coefficient whose refinement survived. See the
    // per-coefficient reconstruction loop below.
    let mut last_bit = vec![q; n];

    for bp_idx in 0..q_usize {
        let bp = q_usize - 1 - bp_idx;

        // Find the significance packet for this bit-plane index. A
        // truncated stream simply omits trailing packets; a missing
        // significance packet means this and every lower plane were never
        // delivered, so nothing more can be decoded and `last_bit` must
        // not be lowered.
        let sig_body = packets
            .iter()
            .find(|p| p.bit_plane == bp_idx as u8 && p.is_significance)
            .map(|p| p.body.as_slice());
        let Some(sig_body) = sig_body else {
            // No significance packet at this depth -> the stream is
            // truncated here. Decoding lower planes from absent bodies
            // would inject spurious magnitude bits, so stop.
            break;
        };

        let mut sig_model = ContextModel::new();
        let mut sig_dec = ArithDecoder::new(sig_body)?;

        decode_significance_pass(
            &mut sig_dec,
            &mut sig_model,
            &mut significant,
            &mut sign,
            &mut mag,
            &mut cat,
            &mut last_bit,
            width,
            height,
            bp,
            levels,
        )?;

        // Find the refinement packet for this bit-plane index. When it is
        // absent (the budget cut fell between this plane's significance
        // and refinement packets) the refinement pass is skipped entirely
        // -- the already-significant coefficients keep their coarser
        // `last_bit` and a wider reconstruction bin, which is exactly the
        // §III.A deadzone the dropped plane implies for them.
        if let Some(ref_body) = packets
            .iter()
            .find(|p| p.bit_plane == bp_idx as u8 && !p.is_significance)
            .map(|p| p.body.as_slice())
        {
            let mut ref_model = ContextModel::new();
            let mut ref_dec = ArithDecoder::new(ref_body)?;

            decode_refinement_pass(
                &mut ref_dec,
                &mut ref_model,
                &significant,
                &mut mag,
                &mut cat,
                &mut last_bit,
                width,
                height,
                bp,
                levels,
            )?;
        } else {
            // No refinement packet at this depth (the budget cut fell
            // between this plane's significance and refinement packets).
            // The encoder still ran a refinement pass for this plane, so
            // its category transitions happened there; to keep the
            // decoder's categories aligned for the *lower* planes that
            // may still arrive, advance every visited coefficient's
            // category exactly as the refinement pass would have. (A
            // truncated stream usually has no lower planes either, but
            // this keeps the model exact when only the refinement packet
            // of an intermediate plane is missing.)
            advance_refinement_categories(&significant, &mag, &mut cat, width, height, bp);
        }
    }

    // §III.A deadzone-quantizer reconstruction point, applied
    // *per coefficient*. Reconstructing a coefficient from only its `q -
    // b` most-significant bit planes is equivalent to a deadzone scalar
    // quantizer with bin width `∆ = 2^b`, where `b` is the number of low
    // magnitude bit planes that were never delivered *for that
    // coefficient*. A budget cut can land between a plane's significance
    // and refinement packets, so two coefficients in the same strip can
    // have different `b`: one made newly significant at the cut plane
    // knows its MSB (`b = bp`); one that was already significant but whose
    // plane-`bp` refinement was dropped only knows down to `bp + 1`
    // (`b = bp + 1`), i.e. a bin twice as wide. The earlier global-`b`
    // approximation under-reconstructed the latter class. `last_bit[i]`
    // carries each coefficient's true deepest delivered plane.
    //
    // IPN 42-155 §III.A fixes the reconstruction point per bin:
    //   * the central deadzone bin `[-(∆-1), ∆-1]` (insignificant
    //     pixels) reconstructs to the origin, `0`;
    //   * every other bin `±[i∆, (i+1)∆-1]` reconstructs to
    //     `±((i + 1/2)∆ - 1)` -- the mid-bin value biased one step
    //     toward the origin.
    //
    // A decoded significant magnitude `mag` carries bits only down to
    // plane `b`, so `mag = i·∆` is the bin's lower edge; the spec point
    // is `mag + ∆/2 - 1`. When `∆ = 1` (b == 0, the full stream is
    // present) the offset is zero and the magnitude is exact -- so the
    // lossless / untruncated path is bit-identical to before.
    let out = mag
        .iter()
        .zip(sign.iter())
        .zip(last_bit.iter())
        .map(|((&m, &s), &b)| {
            // Insignificant pixels (mag == 0) sit in the central deadzone
            // bin and reconstruct to the origin regardless of `∆`.
            let v = if m == 0 {
                0
            } else {
                let off: i32 = if b == 0 {
                    0
                } else {
                    // ∆/2 - 1 = 2^(b-1) - 1.
                    (1i32 << (b - 1)) - 1
                };
                m as i32 + off
            };
            if s {
                -v
            } else {
                v
            }
        })
        .collect();
    Ok(out)
}

/// Number of least-significant magnitude bit planes that were *not*
/// delivered to the decoder, i.e. the deadzone bin-width exponent `b`
/// from IPN 42-155 §III.A (`∆ = 2^b`).
///
/// Every transmitted bit plane appears as at least one [`EncodedPacket`]
/// (the significance pass always produces a flushed body, even when no
/// pixel turned significant). A truncated progressive stream simply
/// drops its trailing least-significant packets, so the lowest magnitude
/// bit plane for which *any* packet (significance or refinement)
/// survives marks the boundary: every plane below it is unavailable.
///
/// Packets carry `bit_plane` as an MSB-first index (`0` = MSB plane,
/// `q-1` = LSB plane); the magnitude bit position is `q - 1 - bit_plane`.
/// `b` is the smallest such magnitude position over all present packets;
/// an empty packet set (handled by the caller) yields `q` here.
///
/// Superseded for reconstruction by the per-coefficient `last_bit`
/// tracking in [`decode_bitplanes_multi`] (which handles the
/// significance-survives / refinement-dropped case the strip-global `b`
/// could not distinguish); retained as the clean-boundary characterisation
/// the unit test pins.
#[cfg(test)]
fn unavailable_bit_planes(packets: &[EncodedPacket], q: usize) -> u32 {
    let deepest_bit = packets
        .iter()
        .map(|p| q.saturating_sub(1 + p.bit_plane as usize))
        .min();
    match deepest_bit {
        Some(b) => b as u32,
        None => q as u32,
    }
}

/// Significance + sign pass for one bit-plane, stripe-ordered.
/// Modifies `significant`, `sign`, and `cat` in place. A coefficient
/// that becomes significant here transitions to category 1 (IPN 42-155
/// §III.B: "after the first '1' bit from the pixel is encoded, the
/// pixel's category becomes 1").
#[allow(clippy::too_many_arguments)]
fn encode_significance_pass(
    enc: &mut ArithEncoder,
    model: &mut ContextModel,
    coeffs: &[i32],
    significant: &mut [bool],
    sign: &mut [bool],
    cat: &mut [u8],
    width: usize,
    height: usize,
    bp: usize,
    levels: u8,
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
                let pat = neighbour_significance_pattern(significant, width, height, x, y, levels);
                let ctx = significance_ctx_for(pat, x, y, levels);
                debug_assert!(ctx < CONTEXT_COUNT);

                let mag = coeffs[i].unsigned_abs();
                let bit = ((mag >> bp) & 1) as u8;

                let (num, den) = model.probability(ctx);
                enc.encode_bit(bit, num, den);
                model.observe(ctx, bit);

                if bit == 1 {
                    significant[i] = true;
                    cat[i] = 1;
                    sign[i] = coeffs[i] < 0;

                    // Sign bit with sign-flip convention (IPN 42-155 §III.B),
                    // subband-aware (HL axis transpose for Table 8).
                    let (h_pat, v_pat) =
                        neighbour_sign_pattern(significant, sign, width, height, x, y, levels);
                    let (sctx, flip) = sign_ctx_for(h_pat, v_pat, x, y, levels);
                    debug_assert!(sctx < CONTEXT_COUNT);
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
///
/// `last_bit[i]` records, for every coefficient that received a magnitude
/// bit, the deepest (smallest) bit-plane `bp` at which a bit was decoded
/// for it. A coefficient that becomes significant in this pass has its
/// MSB plane `bp` recorded here; the refinement pass lowers it further as
/// refinement packets survive. This drives the per-coefficient §III.A
/// deadzone reconstruction point (see [`decode_bitplanes_multi`]).
#[allow(clippy::too_many_arguments)]
fn decode_significance_pass(
    dec: &mut ArithDecoder<'_>,
    model: &mut ContextModel,
    significant: &mut [bool],
    sign: &mut [bool],
    mag: &mut [u32],
    cat: &mut [u8],
    last_bit: &mut [u8],
    width: usize,
    height: usize,
    bp: usize,
    levels: u8,
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
                let pat = neighbour_significance_pattern(significant, width, height, x, y, levels);
                let ctx = significance_ctx_for(pat, x, y, levels);
                let (num, den) = model.probability(ctx);
                let bit = dec.decode_bit(num, den)?;
                model.observe(ctx, bit);

                if bit == 1 {
                    significant[i] = true;
                    cat[i] = 1;
                    mag[i] |= 1u32 << bp;
                    last_bit[i] = bp as u8;
                    let (h_pat, v_pat) =
                        neighbour_sign_pattern(significant, sign, width, height, x, y, levels);
                    let (sctx, flip) = sign_ctx_for(h_pat, v_pat, x, y, levels);
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

/// Refinement pass for one bit-plane, stripe-ordered, with the
/// IPN 42-155 §III.B four-category coding scheme:
///
///   * category 1 (first refinement bit after becoming significant) ->
///     context 9 (no H/V significant neighbour) or 10;
///   * category 2 (second refinement bit) -> context 11;
///   * category 3 (third and later refinement bits) -> **uncoded** (fed
///     to the arithmetic coder at a fixed P(0)=1/2 with no model update).
///
/// `cat[i]` is advanced by one (saturating at 3) for every coefficient
/// visited, so the next bit-plane sees the next category. The decoder
/// runs the identical category transitions, keeping the two in lockstep.
#[allow(clippy::too_many_arguments)]
fn encode_refinement_pass(
    enc: &mut ArithEncoder,
    model: &mut ContextModel,
    coeffs: &[i32],
    significant: &[bool],
    cat: &mut [u8],
    width: usize,
    height: usize,
    bp: usize,
    levels: u8,
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
                let has_hv = has_hv_significant(neighbour_significance_pattern(
                    significant,
                    width,
                    height,
                    x,
                    y,
                    levels,
                ));
                let bit = ((m >> bp) & 1) as u8;
                match magnitude_context(cat[i], has_hv) {
                    MagnitudeContext::Coded(rctx) => {
                        debug_assert!(rctx < CONTEXT_COUNT);
                        let (num, den) = model.probability(rctx);
                        enc.encode_bit(bit, num, den);
                        model.observe(rctx, bit);
                    }
                    MagnitudeContext::Uncoded => {
                        let (num, den) = UNCODED_P1;
                        enc.encode_bit(bit, num, den);
                    }
                }
                cat[i] = cat[i].saturating_add(1).min(3);
            }
        }
        stripe_start += STRIPE_HEIGHT;
    }
}

/// Advance per-pixel categories exactly as [`decode_refinement_pass`]
/// would, without decoding any bits. Used when the refinement packet for
/// a bit-plane is absent (budget cut between this plane's significance
/// and refinement packets) so the decoder's category state stays aligned
/// with the encoder for any lower planes that still arrive.
fn advance_refinement_categories(
    significant: &[bool],
    mag: &[u32],
    cat: &mut [u8],
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
                if highest_set_bit(mag[i]) == Some(bp as u32) {
                    continue;
                }
                cat[i] = cat[i].saturating_add(1).min(3);
            }
        }
        stripe_start += STRIPE_HEIGHT;
    }
}

/// Refinement decode pass for one bit-plane, stripe-ordered.
///
/// Every coefficient *visited* in this pass (already significant, not at
/// its MSB plane) has its magnitude confirmed down to bit-plane `bp`
/// regardless of whether the decoded bit was 0 or 1, so `last_bit[i]` is
/// lowered to `bp`. This is what makes the per-coefficient deadzone
/// reconstruction (see [`decode_bitplanes_multi`]) exact: a coefficient
/// whose refinement at `bp` was *not* delivered keeps a coarser
/// `last_bit` and therefore a wider reconstruction bin than one that was.
#[allow(clippy::too_many_arguments)]
fn decode_refinement_pass(
    dec: &mut ArithDecoder<'_>,
    model: &mut ContextModel,
    significant: &[bool],
    mag: &mut [u32],
    cat: &mut [u8],
    last_bit: &mut [u8],
    width: usize,
    height: usize,
    bp: usize,
    levels: u8,
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
                let has_hv = has_hv_significant(neighbour_significance_pattern(
                    significant,
                    width,
                    height,
                    x,
                    y,
                    levels,
                ));
                let bit = match magnitude_context(cat[i], has_hv) {
                    MagnitudeContext::Coded(rctx) => {
                        let (num, den) = model.probability(rctx);
                        let bit = dec.decode_bit(num, den)?;
                        model.observe(rctx, bit);
                        bit
                    }
                    MagnitudeContext::Uncoded => {
                        let (num, den) = UNCODED_P1;
                        dec.decode_bit(num, den)?
                    }
                };
                if bit == 1 {
                    mag[i] |= 1u32 << bp;
                }
                cat[i] = cat[i].saturating_add(1).min(3);
                // The refinement bit (0 or 1) confirms the magnitude down
                // to plane `bp` for this coefficient.
                last_bit[i] = bp as u8;
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
    let mut cat = vec![0u8; n];
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
                    // Legacy single-packet path is subband-agnostic: pass
                    // levels = 0 to keep the unit-stride spatial-raster walk.
                    let pat = neighbour_significance_pattern(
                        &significant,
                        input.width,
                        input.height,
                        x,
                        y,
                        0,
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
                        cat[i] = 1;
                        sign[i] = input.coeffs[i] < 0;
                        let (h_pat, v_pat) = neighbour_sign_pattern(
                            &significant,
                            &sign,
                            input.width,
                            input.height,
                            x,
                            y,
                            0,
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
                    let has_hv = has_hv_significant(neighbour_significance_pattern(
                        &significant,
                        input.width,
                        input.height,
                        x,
                        y,
                        0,
                    ));
                    let bit = ((mag >> bp) & 1) as u8;
                    match magnitude_context(cat[i], has_hv) {
                        MagnitudeContext::Coded(rctx) => {
                            debug_assert!(rctx < CONTEXT_COUNT);
                            let (num, den) = model.probability(rctx);
                            enc.encode_bit(bit, num, den);
                            model.observe(rctx, bit);
                        }
                        MagnitudeContext::Uncoded => {
                            let (num, den) = UNCODED_P1;
                            enc.encode_bit(bit, num, den);
                        }
                    }
                    cat[i] = cat[i].saturating_add(1).min(3);
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
    let mut cat = vec![0u8; n];

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
                    // Legacy single-packet path is subband-agnostic: levels = 0.
                    let pat = neighbour_significance_pattern(&significant, width, height, x, y, 0);
                    let ctx = significance_context(pat);
                    let (num, den) = model.probability(ctx);
                    let bit = dec.decode_bit(num, den)?;
                    model.observe(ctx, bit);
                    if bit == 1 {
                        significant[i] = true;
                        cat[i] = 1;
                        mag[i] |= 1u32 << bp;
                        let (h_pat, v_pat) =
                            neighbour_sign_pattern(&significant, &sign, width, height, x, y, 0);
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
                    let has_hv = has_hv_significant(neighbour_significance_pattern(
                        &significant,
                        width,
                        height,
                        x,
                        y,
                        0,
                    ));
                    let bit = match magnitude_context(cat[i], has_hv) {
                        MagnitudeContext::Coded(rctx) => {
                            let (num, den) = model.probability(rctx);
                            let bit = dec.decode_bit(num, den)?;
                            model.observe(rctx, bit);
                            bit
                        }
                        MagnitudeContext::Uncoded => {
                            let (num, den) = UNCODED_P1;
                            dec.decode_bit(num, den)?
                        }
                    };
                    if bit == 1 {
                        mag[i] |= 1u32 << bp;
                    }
                    cat[i] = cat[i].saturating_add(1).min(3);
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
///
/// The eight neighbours are the eight nearest pixels **of the same
/// subband** (IPN 42-155 §III.B: "its eight nearest neighbors from the
/// same segment of the subband"). In the Mallat-interleaved transform
/// buffer those sit `subband_stride(x, y, levels)` apart along each axis,
/// not one buffer cell apart, so the walk steps by the subband stride
/// rather than the spatial-raster unit step. A neighbour stepped off the
/// strip edge is "at the edge of its subband segment" and is treated as
/// not yet significant (§III.B). `levels == 0` keeps the legacy unit-stride
/// spatial walk used by the subband-agnostic unit tests.
fn neighbour_significance_pattern(
    significant: &[bool],
    width: usize,
    height: usize,
    x: usize,
    y: usize,
    levels: u8,
) -> u8 {
    let stride = subband_stride(x, y, levels) as isize;
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
        let nx = x as isize + dx * stride;
        let ny = y as isize + dy * stride;
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
///
/// The four sign neighbours are the same-subband nearest pixels (IPN
/// 42-155 §III.B), i.e. `subband_stride(x, y, levels)` apart in the
/// interleaved buffer rather than one cell apart. `levels == 0` keeps the
/// legacy unit-stride spatial walk.
fn neighbour_sign_pattern(
    significant: &[bool],
    sign: &[bool],
    width: usize,
    height: usize,
    x: usize,
    y: usize,
    levels: u8,
) -> (u8, u8) {
    let stride = subband_stride(x, y, levels) as isize;
    let h = pair_pattern(
        significant,
        sign,
        width,
        height,
        x as isize - stride,
        y as isize,
        x as isize + stride,
        y as isize,
    );
    let v = pair_pattern(
        significant,
        sign,
        width,
        height,
        x as isize,
        y as isize - stride,
        x as isize,
        y as isize + stride,
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
            neighbour_significance_pattern(&sig, 2, 2, 0, 0, 0),
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
            levels: 0,
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
            levels: 0,
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
            levels: 0,
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
            levels: 0,
        };
        let packets = encode_bitplanes(&input).unwrap();
        assert_eq!(
            packets.len(),
            2 * q as usize,
            "should have 2 packets per bit-plane"
        );
        let out = decode_bitplanes_multi(&packets, w, h, q, 0).unwrap();
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
            levels: 0,
        };
        let packets = encode_bitplanes(&input).unwrap();
        let out = decode_bitplanes_multi(&packets, w, h, q, 0).unwrap();
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
            levels: 0,
        };
        let single_body = encode_bitplanes_single(&input).unwrap();
        let single_out = decode_bitplanes(&single_body, w, h, q).unwrap();
        let packets = encode_bitplanes(&input).unwrap();
        let multi_out = decode_bitplanes_multi(&packets, w, h, q, 0).unwrap();
        assert_eq!(single_out, coeffs, "single path must round-trip");
        assert_eq!(multi_out, coeffs, "multi path must round-trip");
    }

    /// The subband-aware path (`levels >= 1`) round-trips bit-exactly and
    /// the encoder + decoder agree on the §III.B Table 6/7 context
    /// dispatch (a mismatch would desynchronise the arithmetic coder and
    /// corrupt the decode).
    #[test]
    fn subband_aware_roundtrip_bit_exact() {
        let w = 16;
        let h = 16;
        let coeffs = lcg_signal(w * h, 512, 0xABCD);
        let q = select_bit_plane_count(&coeffs);
        for levels in 1..=4u8 {
            let input = BitPlaneInput {
                coeffs: &coeffs,
                width: w,
                height: h,
                q,
                levels,
            };
            let packets = encode_bitplanes(&input).unwrap();
            let out = decode_bitplanes_multi(&packets, w, h, q, levels).unwrap();
            assert_eq!(out, coeffs, "subband-aware round-trip (levels={levels})");
        }
    }

    /// The subband-aware significance contexts actually change the encoded
    /// bytes versus the subband-agnostic path -- proving the §III.B Table
    /// 6/7 dispatch is wired through the scanner rather than a no-op. (Both
    /// paths still round-trip; only the entropy-coder byte allocation
    /// differs.)
    #[test]
    fn subband_aware_changes_encoded_bytes() {
        let w = 16;
        let h = 16;
        // A signal with structure across subbands so the HH (Table 7) and
        // LL/LH/HL (Table 6 + HL transpose) dispatch diverges from uniform.
        let coeffs: Vec<i32> = (0..w * h)
            .map(|i| {
                let x = (i % w) as i32;
                let y = (i / w) as i32;
                ((x * 7) ^ (y * 13)) % 200 - 100
            })
            .collect();
        let q = select_bit_plane_count(&coeffs);
        let agnostic = encode_bitplanes(&BitPlaneInput {
            coeffs: &coeffs,
            width: w,
            height: h,
            q,
            levels: 0,
        })
        .unwrap();
        let aware = encode_bitplanes(&BitPlaneInput {
            coeffs: &coeffs,
            width: w,
            height: h,
            q,
            levels: 3,
        })
        .unwrap();
        let agnostic_bytes: usize = agnostic.iter().map(|p| p.body.len()).sum();
        let aware_bytes: usize = aware.iter().map(|p| p.body.len()).sum();
        assert_ne!(
            agnostic_bytes, aware_bytes,
            "subband-aware contexts must change the entropy-coded byte total"
        );
        // Both still round-trip exactly.
        assert_eq!(decode_bitplanes_multi(&aware, w, h, q, 3).unwrap(), coeffs);
    }

    /// §III.B same-subband neighbour walk: when `levels >= 1` the
    /// significance pattern is gathered from same-subband neighbours
    /// `2^level` apart, not the spatially-adjacent cells. A coefficient
    /// whose only significant *spatial* neighbour belongs to a different
    /// subband must see an empty same-subband neighbourhood; a coefficient
    /// with a significant *same-subband* neighbour at stride distance must
    /// see it.
    #[test]
    fn neighbour_walk_is_same_subband() {
        // 8x8 buffer, 3 decomposition levels. The level-1 HL coefficient at
        // (3,2): same-subband E neighbour is (3+2, 2) = (5,2) (also HL
        // level 1); its spatial E neighbour (4,2) is a *different* subband.
        let w = 8;
        let h = 8;
        let levels = 3u8;
        // Mark only (5,2) significant -- the strided same-subband E neighbour.
        let mut sig = vec![false; w * h];
        sig[2 * w + 5] = true;
        let strided = neighbour_significance_pattern(&sig, w, h, 3, 2, levels);
        // Bit 4 is E in the NW..SE packing.
        assert_ne!(strided & (1 << 4), 0, "same-subband E neighbour is seen");

        // Now mark only the spatial E neighbour (4,2) -- a different subband.
        let mut sig2 = vec![false; w * h];
        sig2[2 * w + 4] = true;
        let strided2 = neighbour_significance_pattern(&sig2, w, h, 3, 2, levels);
        assert_eq!(
            strided2, 0,
            "a different-subband spatial neighbour must not pollute the context"
        );
        // The legacy unit-stride (levels = 0) walk *would* see the spatial
        // neighbour -- confirming the two behaviours genuinely differ.
        let unit = neighbour_significance_pattern(&sig2, w, h, 3, 2, 0);
        assert_ne!(unit & (1 << 4), 0, "legacy unit walk sees the spatial cell");
    }

    /// `unavailable_bit_planes` maps the MSB-first packet `bit_plane`
    /// indices back to the §III.A bin-width exponent `b`. With every
    /// plane present `b == 0` (∆ = 1, exact); dropping the `k` trailing
    /// (least-significant) packet pairs leaves `b == k`.
    #[test]
    fn unavailable_bit_planes_counts_dropped_lsb_packets() {
        let q = 8usize;
        // Synthesise the full 2*q packet list (bodies irrelevant here).
        let full: Vec<EncodedPacket> = (0..q)
            .flat_map(|bp_idx| {
                [true, false].into_iter().map(move |sig| EncodedPacket {
                    bit_plane: bp_idx as u8,
                    is_significance: sig,
                    body: vec![0u8],
                    delta_distortion: 0.0,
                })
            })
            .collect();
        assert_eq!(unavailable_bit_planes(&full, q), 0, "full stream: b = 0");

        for k in 0..q {
            // Keep the highest-priority (smallest bit_plane index) packets:
            // a truncation drops the LSB tail, i.e. the largest indices.
            let kept: Vec<EncodedPacket> = full
                .iter()
                .filter(|p| (p.bit_plane as usize) < q - k)
                .cloned()
                .collect();
            assert_eq!(
                unavailable_bit_planes(&kept, q),
                k as u32,
                "dropping {k} trailing planes leaves b = {k}"
            );
        }

        assert_eq!(
            unavailable_bit_planes(&[], q),
            q as u32,
            "no packets: b = q"
        );
    }

    /// §III.A reconstruction point. A coefficient known only to its
    /// `q - b` most-significant bit planes reconstructs at the mid-bin
    /// value biased toward the origin: `±((i + 1/2)∆ - 1)` for `∆ = 2^b`,
    /// `i >= 1`; the central deadzone bin (insignificant) reconstructs to
    /// `0`. Feeding a truncated packet set must move the reconstruction
    /// off the bin's lower edge by exactly `∆/2 - 1`.
    #[test]
    fn deadzone_reconstruction_biases_toward_mid_bin() {
        // A single non-zero coefficient with magnitude 0b1011_0 = 22,
        // sign positive, in an 8x4 buffer so stripe scanning has shape.
        let w = 8;
        let h = 4;
        let q = 6u8; // 22 < 32, MSB at bit 4 -> needs q >= 5; use 6.
        let mut coeffs = vec![0i32; w * h];
        coeffs[10] = 22;
        let input = BitPlaneInput {
            coeffs: &coeffs,
            width: w,
            height: h,
            q,
            levels: 0,
        };
        let packets = encode_bitplanes(&input).unwrap();

        // Full stream: exact (b = 0).
        let full = decode_bitplanes_multi(&packets, w, h, q, 0).unwrap();
        assert_eq!(full[10], 22, "untruncated decode is exact");

        // Drop the 2 trailing (LSB) bit-plane pairs -> b = 2, ∆ = 4.
        // The surviving magnitude bits are 22 & !0b11 = 20 (the bin lower
        // edge i∆ with i = 5). Spec point = 20 + ∆/2 - 1 = 20 + 1 = 21.
        let drop2: Vec<EncodedPacket> = packets
            .iter()
            .filter(|p| (p.bit_plane as usize) < (q as usize - 2))
            .cloned()
            .collect();
        let trunc = decode_bitplanes_multi(&drop2, w, h, q, 0).unwrap();
        assert_eq!(trunc[10], 21, "mid-bin biased reconstruction (∆ = 4)");
        // The bias strictly reduces the reconstruction error vs the bin
        // lower edge (|22 - 21| = 1 < |22 - 20| = 2).
        assert!((22 - trunc[10]).abs() < (22 - 20));

        // Insignificant pixels stay at the origin (deadzone centre).
        assert!(trunc.iter().enumerate().all(|(i, &v)| i == 10 || v == 0));
    }

    /// On a random signed buffer, truncating the stream and reconstructing
    /// with the §III.A mid-bin point yields strictly lower total squared
    /// error than reconstructing at the bin lower edge (zeroed low bits).
    #[test]
    fn deadzone_reconstruction_lowers_truncation_mse() {
        let w = 8;
        let h = 8;
        let coeffs = lcg_signal(w * h, 256, 0x5151);
        let q = select_bit_plane_count(&coeffs);
        let input = BitPlaneInput {
            coeffs: &coeffs,
            width: w,
            height: h,
            q,
            levels: 0,
        };
        let packets = encode_bitplanes(&input).unwrap();

        // Drop the 3 least-significant bit-plane pairs (b = 3, ∆ = 8).
        let b = 3usize.min(q as usize - 1);
        let kept: Vec<EncodedPacket> = packets
            .iter()
            .filter(|p| (p.bit_plane as usize) < (q as usize - b))
            .cloned()
            .collect();
        let recon = decode_bitplanes_multi(&kept, w, h, q, 0).unwrap();

        // Lower-edge reference: same decode but with the mid-bin offset
        // removed (mask off the unavailable low bits, keep the sign).
        let delta = 1i32 << b;
        let lower_edge: Vec<i32> = recon
            .iter()
            .map(|&v| {
                let s = v.signum();
                let mag = v.unsigned_abs() as i32;
                // Strip the mid-bin bias back to the bin lower edge i∆.
                s * (mag & !(delta - 1))
            })
            .collect();

        let mse = |a: &[i32], b: &[i32]| -> i64 {
            a.iter()
                .zip(b)
                .map(|(&x, &y)| {
                    let d = (x - y) as i64;
                    d * d
                })
                .sum()
        };
        let mid_mse = mse(&coeffs, &recon);
        let edge_mse = mse(&coeffs, &lower_edge);
        assert!(
            mid_mse < edge_mse,
            "mid-bin reconstruction MSE {mid_mse} must beat lower-edge MSE {edge_mse}"
        );
    }

    /// Per-coefficient §III.A deadzone: when a budget cut lands *between*
    /// a plane's significance and refinement packets (sig(bp) survives,
    /// ref(bp) dropped), coefficients carry two different deadzone widths.
    /// A coefficient made newly significant at plane `bp` knows its MSB
    /// (`b = bp`); one already significant at a higher plane but missing
    /// its plane-`bp` refinement bit only knows down to `bp + 1`
    /// (`b = bp + 1`, a bin twice as wide). The reconstruction must place
    /// each at its own mid-bin point, not a single strip-global one.
    #[test]
    fn per_coefficient_deadzone_when_refinement_dropped() {
        // Two coefficients, far apart so their stripe neighbourhoods don't
        // interact. `big` becomes significant at a high plane; `small`
        // becomes significant exactly at the cut plane.
        let w = 8;
        let h = 8;
        let q = 7u8; // magnitudes < 128
        let mut coeffs = vec![0i32; w * h];
        // big = 0b101_0110 = 86: MSB at bit 6, refinement bits below.
        coeffs[3] = 86;
        // small = 0b000_1010 = 10: MSB at bit 3.
        coeffs[44] = 10;
        let input = BitPlaneInput {
            coeffs: &coeffs,
            width: w,
            height: h,
            q,
            levels: 0,
        };
        let packets = encode_bitplanes(&input).unwrap();
        // Full stream reconstructs exactly.
        let full = decode_bitplanes_multi(&packets, w, h, q, 0).unwrap();
        assert_eq!(full[3], 86);
        assert_eq!(full[44], 10);

        // Cut at magnitude plane bp = 3: keep every sig/ref packet for
        // planes 4,5,6 (bit_plane idx 0..=2 from MSB) plus *only* the
        // significance packet of plane 3 (bit_plane idx 3). Drop ref(3)
        // and everything below.
        let cut: Vec<EncodedPacket> = packets
            .iter()
            .filter(|p| {
                let idx = p.bit_plane as usize;
                idx < 3 || (idx == 3 && p.is_significance)
            })
            .cloned()
            .collect();
        let recon = decode_bitplanes_multi(&cut, w, h, q, 0).unwrap();

        // `small` (10) became significant at plane 3 in the surviving sig
        // packet: its MSB is known, b = 3, ∆ = 8, lower edge 8, point
        // 8 + ∆/2 - 1 = 11.
        assert_eq!(recon[44], 11, "newly-significant coef: b = 3 (∆ = 8)");

        // `big` (86) was already significant before plane 3; its plane-3
        // refinement bit was dropped, so it is known only down to plane 4:
        // b = 4, ∆ = 16. Surviving magnitude bits 86 & !0b1111 = 80 (lower
        // edge), point 80 + ∆/2 - 1 = 80 + 7 = 87.
        assert_eq!(recon[3], 87, "already-significant coef: b = 4 (∆ = 16)");

        // The strip-global approximation would have used b = 3 for *both*
        // (the deepest surviving packet is sig(3)), reconstructing `big`
        // at (86 & !0b111) + 3 = 80 + 3 = 83 -- |86 - 83| = 3, worse than
        // the per-coefficient |86 - 87| = 1.
        assert!(
            (86 - recon[3]).abs() < (86i32 - 83).abs(),
            "per-coef reconstruction must beat strip-global for the \
             refinement-dropped coefficient"
        );
    }

    /// On a textured buffer truncated mid-bit-plane (sig of the cut plane
    /// kept, its refinement dropped), the per-coefficient deadzone yields
    /// strictly lower MSE than the strip-global deadzone that applies one
    /// shared offset to every coefficient.
    #[test]
    fn per_coefficient_deadzone_beats_strip_global_mse() {
        let w = 8;
        let h = 8;
        let coeffs = lcg_signal(w * h, 200, 0xBEEF);
        let q = select_bit_plane_count(&coeffs);
        let input = BitPlaneInput {
            coeffs: &coeffs,
            width: w,
            height: h,
            q,
            levels: 0,
        };
        let packets = encode_bitplanes(&input).unwrap();

        // Pick a cut plane mid-way and keep its significance but not its
        // refinement.
        let cut_idx = (q as usize) / 2;
        let cut: Vec<EncodedPacket> = packets
            .iter()
            .filter(|p| {
                let idx = p.bit_plane as usize;
                idx < cut_idx || (idx == cut_idx && p.is_significance)
            })
            .cloned()
            .collect();
        let recon = decode_bitplanes_multi(&cut, w, h, q, 0).unwrap();

        // Strip-global reference: the old behaviour applied a single b
        // (= deepest surviving packet's plane) to every significant coef.
        // The deepest surviving packet here is sig(cut_idx) at magnitude
        // position b_g = q - 1 - cut_idx.
        let b_g = (q as usize) - 1 - cut_idx;
        let off_g: i32 = if b_g == 0 { 0 } else { (1i32 << (b_g - 1)) - 1 };
        let global: Vec<i32> = recon
            .iter()
            .map(|&v| {
                if v == 0 {
                    return 0;
                }
                let s = v.signum();
                let mag = v.unsigned_abs() as i32;
                // Strip whatever per-coef offset was applied, re-quantise
                // to the global lower edge, then add the global offset.
                let lower = mag & !((1i32 << b_g) - 1);
                s * (lower + off_g)
            })
            .collect();

        let mse = |a: &[i32], b: &[i32]| -> i64 {
            a.iter()
                .zip(b)
                .map(|(&x, &y)| {
                    let d = (x - y) as i64;
                    d * d
                })
                .sum()
        };
        let per_coef_mse = mse(&coeffs, &recon);
        let global_mse = mse(&coeffs, &global);
        assert!(
            per_coef_mse <= global_mse,
            "per-coefficient deadzone MSE {per_coef_mse} must not exceed \
             strip-global MSE {global_mse}"
        );
        assert!(
            per_coef_mse < global_mse,
            "with a mid-plane cut the per-coefficient deadzone should \
             strictly beat strip-global (got {per_coef_mse} vs {global_mse})"
        );
    }
}
