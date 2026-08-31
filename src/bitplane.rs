//! Bit-plane scanner -- significance, refinement, and sign passes that
//! drive a binary entropy coder. The passes are written once against the
//! [`crate::entropy::BitSink`] / [`crate::entropy::BitSource`] trait
//! surface, so they run unchanged on either entropy backend (the binary
//! arithmetic coder or ICER's §IV interleaved coder) -- the entropy
//! backend is selected per call via [`crate::entropy::EntropyKind`].
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
use crate::entropy::{BitSink, BitSource};
use crate::error::{IcerError, Result};
use crate::priority::{
    classify_position, packet_schedule, subband_lattice, subband_stride, SubbandBitPlane,
    SubbandType,
};

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

/// Restricts which coefficients (and which of their magnitude bit
/// planes) the scan passes visit. Two orthogonal restrictions compose:
///
/// * **§V.B transform-domain segmentation** — `segment = Some((map,
///   seg))` visits only the coefficients whose
///   [`crate::partition::coefficient_segment_map`] entry equals `seg`.
///   Coefficients of other segments are never visited and never become
///   significant in this scan's state, so the §III.B "eight nearest
///   neighbors from the same segment of the subband" rule holds
///   automatically: an out-of-segment neighbour reads as
///   not-yet-significant, exactly the §III.B treatment of a pixel at
///   the edge of its subband segment.
/// * **§VI.A minimum loss** — `skip = Some(map)` never codes the bit of
///   coefficient `i` at a magnitude bit position below `map[i]` (the
///   per-coefficient LSB-plane exclusion from
///   [`crate::priority::min_loss_skip_map`]).
///
/// [`ScanFilter::ALL`] (both `None`) visits everything; the passes are
/// then bit-for-bit identical to the unfiltered scan, so every
/// pre-existing wire form is unchanged.
#[derive(Debug, Clone, Copy, Default)]
pub struct ScanFilter<'a> {
    /// §V.B coefficient segment map + the segment index being coded.
    pub segment: Option<(&'a [u16], u16)>,
    /// §VI.A per-coefficient skipped-LSB-plane counts.
    pub skip: Option<&'a [u8]>,
    /// Optional bounding window `(x0, x1, y0, y1)` outside which the
    /// scan never visits — a pure iteration bound, not a correctness
    /// filter (the §V.B segment block is contiguous, see
    /// [`crate::partition::SegmentRect::image_window`]). The stripe
    /// grid stays aligned to `y = 0`, so restricting the walk to the
    /// window visits the same coefficients in the same order as a
    /// full-image walk with the segment mask alone — the output is
    /// byte-identical, just without the wasted out-of-segment
    /// iterations.
    pub window: Option<(usize, usize, usize, usize)>,
}

impl ScanFilter<'_> {
    /// The visit-everything filter (legacy whole-strip scan).
    pub const ALL: ScanFilter<'static> = ScanFilter {
        segment: None,
        skip: None,
        window: None,
    };

    /// Iteration bounds `(x0, x1, y0, y1)` clamped to the buffer.
    #[inline]
    fn bounds(&self, width: usize, height: usize) -> (usize, usize, usize, usize) {
        match self.window {
            Some((x0, x1, y0, y1)) => {
                (x0.min(width), x1.min(width), y0.min(height), y1.min(height))
            }
            None => (0, width, 0, height),
        }
    }

    /// `true` iff coefficient `i`'s bit at magnitude bit position `bp`
    /// is part of this scan.
    #[inline]
    fn visits(&self, i: usize, bp: usize) -> bool {
        if let Some((map, seg)) = self.segment {
            if map[i] != seg {
                return false;
            }
        }
        if let Some(skip) = self.skip {
            if (bp as u32) < skip[i] as u32 {
                return false;
            }
        }
        true
    }

    fn validate(&self, n: usize) -> Result<()> {
        if let Some((map, _)) = self.segment {
            if map.len() != n {
                return Err(IcerError::invalid(format!(
                    "segment map length {} != coeff count {n}",
                    map.len()
                )));
            }
        }
        if let Some(skip) = self.skip {
            if skip.len() != n {
                return Err(IcerError::invalid(format!(
                    "skip map length {} != coeff count {n}",
                    skip.len()
                )));
            }
        }
        if let Some((x0, x1, y0, y1)) = self.window {
            if x0 > x1 || y0 > y1 {
                return Err(IcerError::invalid(format!(
                    "inverted scan window ({x0}, {x1}, {y0}, {y1})"
                )));
            }
        }
        Ok(())
    }
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

/// Like [`encode_bitplanes`] but with the entropy backend chosen by
/// `kind` (IPN 42-155 §IV interleaved coder vs the binary arithmetic
/// coder). The pass logic — stripe order, contexts, category scheme — is
/// identical across backends; only the per-packet entropy coding differs.
pub fn encode_bitplanes_with(
    input: &BitPlaneInput<'_>,
    kind: crate::entropy::EntropyKind,
) -> Result<Vec<EncodedPacket>> {
    encode_bitplanes_inner(input, None, kind, &ScanFilter::ALL)
}

/// Like [`encode_bitplanes_with`], restricted by a [`ScanFilter`]
/// (IPN 42-155 §V.B transform-domain segmentation and/or the §VI.A
/// minimum-loss plane exclusion). With [`ScanFilter::ALL`] the output
/// is byte-identical to [`encode_bitplanes_with`].
pub fn encode_bitplanes_filtered(
    input: &BitPlaneInput<'_>,
    kind: crate::entropy::EntropyKind,
    filter: &ScanFilter<'_>,
) -> Result<Vec<EncodedPacket>> {
    encode_bitplanes_inner(input, None, kind, filter)
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
    encode_bitplanes_inner(
        input,
        weights,
        crate::entropy::EntropyKind::Arithmetic,
        &ScanFilter::ALL,
    )
}

/// The shared bit-plane encode driver. `kind` selects the entropy
/// backend; `weights` the optional §III.A distortion weighting; `filter`
/// the §V.B / §VI.A visit restriction. All public entrypoints funnel
/// here so the pass logic exists once.
fn encode_bitplanes_inner(
    input: &BitPlaneInput<'_>,
    weights: Option<&[f64]>,
    kind: crate::entropy::EntropyKind,
    filter: &ScanFilter<'_>,
) -> Result<Vec<EncodedPacket>> {
    input.validate()?;
    let n = input.coeffs.len();
    let q = input.q as usize;
    filter.validate(n)?;
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
        let mut sig_enc = kind.make_sink();
        let sig_before: Vec<bool> = significant.clone();

        encode_significance_pass(
            sig_enc.as_mut(),
            &mut sig_model,
            input.coeffs,
            &mut significant,
            &mut sign,
            &mut cat,
            input.width,
            input.height,
            bp,
            input.levels,
            filter,
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
            body: sig_enc.finish_bits(),
            delta_distortion: sig_dist,
        });

        // --- Refinement pass (fresh coder, stripe order) ---
        let mut ref_model = ContextModel::new();
        let mut ref_enc = kind.make_sink();
        let ref_weight_sum = refinement_weight_sum(
            input.coeffs,
            &significant,
            input.width,
            input.height,
            bp,
            weights,
            filter,
        );

        encode_refinement_pass(
            ref_enc.as_mut(),
            &mut ref_model,
            input.coeffs,
            &significant,
            &mut cat,
            input.width,
            input.height,
            bp,
            input.levels,
            filter,
        );

        // Distortion-reduction model: each refined coefficient halves
        // its quantisation bin, dropping per-coef MSE by `4^bp / 4`.
        // Scaled by the §III.A image-domain weight when supplied.
        let ref_dist = ref_weight_sum * 0.25 * bp_weight;

        packets.push(EncodedPacket {
            bit_plane: bp_idx as u8,
            is_significance: false,
            body: ref_enc.finish_bits(),
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
#[allow(clippy::too_many_arguments)]
fn refinement_weight_sum(
    coeffs: &[i32],
    significant: &[bool],
    width: usize,
    height: usize,
    bp: usize,
    weights: Option<&[f64]>,
    filter: &ScanFilter<'_>,
) -> f64 {
    let mut sum = 0.0f64;
    let (wx0, wx1, wy0, wy1) = filter.bounds(width, height);
    let mut stripe_start = wy0 - (wy0 % STRIPE_HEIGHT);
    while stripe_start < wy1 {
        let stripe_end = (stripe_start + STRIPE_HEIGHT).min(height);
        for y in stripe_start.max(wy0)..stripe_end.min(wy1) {
            for x in wx0..wx1 {
                let i = y * width + x;
                if !filter.visits(i, bp) || !significant[i] {
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
    decode_bitplanes_multi_with(
        packets,
        width,
        height,
        q,
        levels,
        crate::entropy::EntropyKind::Arithmetic,
    )
}

/// Like [`decode_bitplanes_multi`] but with the entropy backend selected
/// by `kind` (must match the one the encoder used).
pub fn decode_bitplanes_multi_with(
    packets: &[EncodedPacket],
    width: usize,
    height: usize,
    q: u8,
    levels: u8,
    kind: crate::entropy::EntropyKind,
) -> Result<Vec<i32>> {
    decode_bitplanes_filtered(packets, width, height, q, levels, kind, &ScanFilter::ALL)
}

/// Like [`decode_bitplanes_multi_with`], restricted by a [`ScanFilter`]
/// (must equal the filter the encoder used, or the entropy decode
/// desynchronises). Coefficients outside the filter decode as `0`.
#[allow(clippy::too_many_arguments)]
pub fn decode_bitplanes_filtered(
    packets: &[EncodedPacket],
    width: usize,
    height: usize,
    q: u8,
    levels: u8,
    kind: crate::entropy::EntropyKind,
    filter: &ScanFilter<'_>,
) -> Result<Vec<i32>> {
    let n = width * height;
    if q == 0 || q > 31 {
        return Err(IcerError::invalid(format!(
            "bit-plane count {q} outside (0,31]"
        )));
    }
    filter.validate(n)?;
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
        let mut sig_dec = kind.make_source(sig_body)?;

        decode_significance_pass(
            sig_dec.as_mut(),
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
            filter,
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
            let mut ref_dec = kind.make_source(ref_body)?;

            decode_refinement_pass(
                ref_dec.as_mut(),
                &mut ref_model,
                &significant,
                &mut mag,
                &mut cat,
                &mut last_bit,
                width,
                height,
                bp,
                levels,
                filter,
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
            advance_refinement_categories(&significant, &mag, &mut cat, width, height, bp, filter);
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
    Ok(deadzone_reconstruct(&mag, &sign, &last_bit))
}

/// Apply the per-coefficient §III.A deadzone reconstruction point (see
/// the comment block above): insignificant coefficients reconstruct to
/// the origin; a significant magnitude known down to plane `b` gets its
/// own `∆/2 - 1 = 2^(b-1) - 1` mid-bin offset. Shared by the MSB-down
/// and the §III.A priority-interleaved decoders.
fn deadzone_reconstruct(mag: &[u32], sign: &[bool], last_bit: &[u8]) -> Vec<i32> {
    mag.iter()
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
        .collect()
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
    enc: &mut dyn BitSink,
    model: &mut ContextModel,
    coeffs: &[i32],
    significant: &mut [bool],
    sign: &mut [bool],
    cat: &mut [u8],
    width: usize,
    height: usize,
    bp: usize,
    levels: u8,
    filter: &ScanFilter<'_>,
) {
    let (wx0, wx1, wy0, wy1) = filter.bounds(width, height);
    let mut stripe_start = wy0 - (wy0 % STRIPE_HEIGHT);
    while stripe_start < wy1 {
        let stripe_end = (stripe_start + STRIPE_HEIGHT).min(height);
        for y in stripe_start.max(wy0)..stripe_end.min(wy1) {
            for x in wx0..wx1 {
                let i = y * width + x;
                if !filter.visits(i, bp) || significant[i] {
                    continue;
                }
                let (ctx, stride, is_hl) =
                    significance_visit(significant, width, height, x, y, levels);
                debug_assert!(ctx < CONTEXT_COUNT);

                let mag = coeffs[i].unsigned_abs();
                let bit = ((mag >> bp) & 1) as u8;

                let (num, den) = model.probability(ctx);
                enc.put_bit(bit, num, den);
                model.observe(ctx, bit);

                if bit == 1 {
                    significant[i] = true;
                    cat[i] = 1;
                    sign[i] = coeffs[i] < 0;

                    // Sign bit with sign-flip convention (IPN 42-155 §III.B),
                    // subband-aware (HL axis transpose for Table 8).
                    let (sctx, flip) = sign_visit(
                        significant,
                        sign,
                        width,
                        height,
                        x,
                        y,
                        stride,
                        is_hl,
                        levels,
                    );
                    debug_assert!(sctx < CONTEXT_COUNT);
                    // Encode the (possibly flipped) sign bit.
                    let raw_sign = u8::from(sign[i]);
                    let coded_sign = if flip { 1 - raw_sign } else { raw_sign };
                    let (sn, sd) = model.probability(sctx);
                    enc.put_bit(coded_sign, sn, sd);
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
    dec: &mut dyn BitSource,
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
    filter: &ScanFilter<'_>,
) -> Result<()> {
    let (wx0, wx1, wy0, wy1) = filter.bounds(width, height);
    let mut stripe_start = wy0 - (wy0 % STRIPE_HEIGHT);
    while stripe_start < wy1 {
        let stripe_end = (stripe_start + STRIPE_HEIGHT).min(height);
        for y in stripe_start.max(wy0)..stripe_end.min(wy1) {
            for x in wx0..wx1 {
                let i = y * width + x;
                if !filter.visits(i, bp) || significant[i] {
                    continue;
                }
                let (ctx, stride, is_hl) =
                    significance_visit(significant, width, height, x, y, levels);
                let (num, den) = model.probability(ctx);
                let bit = dec.get_bit(num, den)?;
                model.observe(ctx, bit);

                if bit == 1 {
                    significant[i] = true;
                    cat[i] = 1;
                    mag[i] |= 1u32 << bp;
                    last_bit[i] = bp as u8;
                    let (sctx, flip) = sign_visit(
                        significant,
                        sign,
                        width,
                        height,
                        x,
                        y,
                        stride,
                        is_hl,
                        levels,
                    );
                    let (sn, sd) = model.probability(sctx);
                    let coded_sign = dec.get_bit(sn, sd)?;
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
    enc: &mut dyn BitSink,
    model: &mut ContextModel,
    coeffs: &[i32],
    significant: &[bool],
    cat: &mut [u8],
    width: usize,
    height: usize,
    bp: usize,
    levels: u8,
    filter: &ScanFilter<'_>,
) {
    let (wx0, wx1, wy0, wy1) = filter.bounds(width, height);
    let mut stripe_start = wy0 - (wy0 % STRIPE_HEIGHT);
    while stripe_start < wy1 {
        let stripe_end = (stripe_start + STRIPE_HEIGHT).min(height);
        for y in stripe_start.max(wy0)..stripe_end.min(wy1) {
            for x in wx0..wx1 {
                let i = y * width + x;
                if !filter.visits(i, bp) || !significant[i] {
                    continue;
                }
                let m = coeffs[i].unsigned_abs();
                // Skip coefficients that became significant in THIS bit-plane
                // (they contribute to the significance pass, not refinement).
                if highest_set_bit(m) == Some(bp as u32) {
                    continue;
                }
                let has_hv = refinement_has_hv(significant, width, height, x, y, levels);
                let bit = ((m >> bp) & 1) as u8;
                match magnitude_context(cat[i], has_hv) {
                    MagnitudeContext::Coded(rctx) => {
                        debug_assert!(rctx < CONTEXT_COUNT);
                        let (num, den) = model.probability(rctx);
                        enc.put_bit(bit, num, den);
                        model.observe(rctx, bit);
                    }
                    MagnitudeContext::Uncoded => {
                        let (num, den) = UNCODED_P1;
                        enc.put_bit(bit, num, den);
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
    filter: &ScanFilter<'_>,
) {
    let (wx0, wx1, wy0, wy1) = filter.bounds(width, height);
    let mut stripe_start = wy0 - (wy0 % STRIPE_HEIGHT);
    while stripe_start < wy1 {
        let stripe_end = (stripe_start + STRIPE_HEIGHT).min(height);
        for y in stripe_start.max(wy0)..stripe_end.min(wy1) {
            for x in wx0..wx1 {
                let i = y * width + x;
                if !filter.visits(i, bp) || !significant[i] {
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
    dec: &mut dyn BitSource,
    model: &mut ContextModel,
    significant: &[bool],
    mag: &mut [u32],
    cat: &mut [u8],
    last_bit: &mut [u8],
    width: usize,
    height: usize,
    bp: usize,
    levels: u8,
    filter: &ScanFilter<'_>,
) -> Result<()> {
    let (wx0, wx1, wy0, wy1) = filter.bounds(width, height);
    let mut stripe_start = wy0 - (wy0 % STRIPE_HEIGHT);
    while stripe_start < wy1 {
        let stripe_end = (stripe_start + STRIPE_HEIGHT).min(height);
        for y in stripe_start.max(wy0)..stripe_end.min(wy1) {
            for x in wx0..wx1 {
                let i = y * width + x;
                if !filter.visits(i, bp) || !significant[i] {
                    continue;
                }
                if highest_set_bit(mag[i]) == Some(bp as u32) {
                    continue;
                }
                let has_hv = refinement_has_hv(significant, width, height, x, y, levels);
                let bit = match magnitude_context(cat[i], has_hv) {
                    MagnitudeContext::Coded(rctx) => {
                        let (num, den) = model.probability(rctx);
                        let bit = dec.get_bit(num, den)?;
                        model.observe(rctx, bit);
                        bit
                    }
                    MagnitudeContext::Uncoded => {
                        let (num, den) = UNCODED_P1;
                        dec.get_bit(num, den)?
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
// §III.A priority-interleaved codec.
//
// IPN 42-155 §III specifies the coding order precisely: "ICER losslessly
// compresses the bit planes of a subband, starting with the most-
// significant bit plane and working toward the least significant.
// Compression of a subband bit plane proceeds one error-containment
// segment at a time, and the bits within a segment are compressed in
// raster scan order. The sign bits are handled differently: the sign bit
// of a pixel is encoded immediately after its first nonzero magnitude
// bit. Bit planes from different subbands are interleaved during this
// coding process ... ICER selects the next subband bit plane according
// to a simple prioritization scheme, described in Section III.A."
//
// The functions below realise that ordering: subband bit planes are
// walked in the §III.A priority order (crate::priority::priority_groups),
// each subband bit plane is coded in ONE combined raster pass over the
// subband's lattice (a still-insignificant coefficient contributes a
// significance bit, followed immediately by its sign when it turns
// significant; an already-significant coefficient contributes a
// refinement bit), and one packet is cut per priority *group* so a
// truncation always lands on a §III.A priority boundary. The context
// model persists across the whole segment (§III: "this probability-of-
// zero estimate relies only on previously encoded information from the
// same segment"), while each packet's body is a fresh entropy-coder run
// so packet boundaries stay byte-aligned.
// ---------------------------------------------------------------------------

/// First lattice coordinate `>= lo` with `coord ≡ phase (mod step)`.
#[inline]
fn lattice_start(phase: usize, step: usize, lo: usize) -> usize {
    if lo <= phase {
        phase
    } else {
        phase + (lo - phase).div_ceil(step) * step
    }
}

/// Encode one subband bit plane (a §III.A schedule *unit*) in a single
/// combined raster pass over the subband's lattice (IPN 42-155 §III).
/// `abs_bit` is the plane's absolute magnitude bit position (`0` = LSB).
/// Accumulates the newly-significant / refined coefficient counts into
/// `stats = (newly_significant, refined)` for the packet's
/// distortion-reduction estimate.
#[allow(clippy::too_many_arguments)]
fn encode_priority_unit(
    enc: &mut dyn BitSink,
    model: &mut ContextModel,
    coeffs: &[i32],
    significant: &mut [bool],
    sign: &mut [bool],
    cat: &mut [u8],
    width: usize,
    height: usize,
    unit: &SubbandBitPlane,
    abs_bit: usize,
    levels: u8,
    filter: &ScanFilter<'_>,
    stats: &mut (u64, u64),
) {
    let lat = subband_lattice(unit.subband);
    let (wx0, wx1, wy0, wy1) = filter.bounds(width, height);
    let mut y = lattice_start(lat.y0, lat.step, wy0);
    while y < wy1 {
        let mut x = lattice_start(lat.x0, lat.step, wx0);
        while x < wx1 {
            let i = y * width + x;
            if !filter.visits(i, abs_bit) {
                x += lat.step;
                continue;
            }
            if !significant[i] {
                // Significance bit; sign immediately after the first
                // nonzero magnitude bit (§III).
                let (ctx, stride, is_hl) =
                    significance_visit(significant, width, height, x, y, levels);
                debug_assert!(ctx < CONTEXT_COUNT);
                let mag = coeffs[i].unsigned_abs();
                let bit = ((mag >> abs_bit) & 1) as u8;
                let (num, den) = model.probability(ctx);
                enc.put_bit(bit, num, den);
                model.observe(ctx, bit);
                if bit == 1 {
                    significant[i] = true;
                    cat[i] = 1;
                    sign[i] = coeffs[i] < 0;
                    let (sctx, flip) = sign_visit(
                        significant,
                        sign,
                        width,
                        height,
                        x,
                        y,
                        stride,
                        is_hl,
                        levels,
                    );
                    debug_assert!(sctx < CONTEXT_COUNT);
                    let raw_sign = u8::from(sign[i]);
                    let coded_sign = if flip { 1 - raw_sign } else { raw_sign };
                    let (sn, sd) = model.probability(sctx);
                    enc.put_bit(coded_sign, sn, sd);
                    model.observe(sctx, coded_sign);
                    stats.0 += 1;
                }
            } else {
                // Refinement bit: the coefficient became significant at a
                // strictly higher plane of this subband (planes are walked
                // MSB-down within a subband, so a coefficient whose MSB is
                // this very plane took the significance branch above).
                let m = coeffs[i].unsigned_abs();
                debug_assert!(highest_set_bit(m) > Some(abs_bit as u32));
                let has_hv = refinement_has_hv(significant, width, height, x, y, levels);
                let bit = ((m >> abs_bit) & 1) as u8;
                match magnitude_context(cat[i], has_hv) {
                    MagnitudeContext::Coded(rctx) => {
                        debug_assert!(rctx < CONTEXT_COUNT);
                        let (num, den) = model.probability(rctx);
                        enc.put_bit(bit, num, den);
                        model.observe(rctx, bit);
                    }
                    MagnitudeContext::Uncoded => {
                        let (num, den) = UNCODED_P1;
                        enc.put_bit(bit, num, den);
                    }
                }
                cat[i] = cat[i].saturating_add(1).min(3);
                stats.1 += 1;
            }
            x += lat.step;
        }
        y += lat.step;
    }
}

/// Decode counterpart of [`encode_priority_unit`] — the identical
/// combined raster pass, updating `mag` / `last_bit` as magnitude bits
/// arrive so the per-coefficient §III.A deadzone reconstruction stays
/// exact under truncation.
#[allow(clippy::too_many_arguments)]
fn decode_priority_unit(
    dec: &mut dyn BitSource,
    model: &mut ContextModel,
    significant: &mut [bool],
    sign: &mut [bool],
    mag: &mut [u32],
    cat: &mut [u8],
    last_bit: &mut [u8],
    width: usize,
    height: usize,
    unit: &SubbandBitPlane,
    abs_bit: usize,
    levels: u8,
    filter: &ScanFilter<'_>,
) -> Result<()> {
    let lat = subband_lattice(unit.subband);
    let (wx0, wx1, wy0, wy1) = filter.bounds(width, height);
    let mut y = lattice_start(lat.y0, lat.step, wy0);
    while y < wy1 {
        let mut x = lattice_start(lat.x0, lat.step, wx0);
        while x < wx1 {
            let i = y * width + x;
            if !filter.visits(i, abs_bit) {
                x += lat.step;
                continue;
            }
            if !significant[i] {
                let (ctx, stride, is_hl) =
                    significance_visit(significant, width, height, x, y, levels);
                let (num, den) = model.probability(ctx);
                let bit = dec.get_bit(num, den)?;
                model.observe(ctx, bit);
                if bit == 1 {
                    significant[i] = true;
                    cat[i] = 1;
                    mag[i] |= 1u32 << abs_bit;
                    last_bit[i] = abs_bit as u8;
                    let (sctx, flip) = sign_visit(
                        significant,
                        sign,
                        width,
                        height,
                        x,
                        y,
                        stride,
                        is_hl,
                        levels,
                    );
                    let (sn, sd) = model.probability(sctx);
                    let coded_sign = dec.get_bit(sn, sd)?;
                    model.observe(sctx, coded_sign);
                    let raw_sign = if flip { 1 - coded_sign } else { coded_sign };
                    sign[i] = raw_sign == 1;
                }
            } else {
                let has_hv = refinement_has_hv(significant, width, height, x, y, levels);
                let bit = match magnitude_context(cat[i], has_hv) {
                    MagnitudeContext::Coded(rctx) => {
                        let (num, den) = model.probability(rctx);
                        let bit = dec.get_bit(num, den)?;
                        model.observe(rctx, bit);
                        bit
                    }
                    MagnitudeContext::Uncoded => {
                        let (num, den) = UNCODED_P1;
                        dec.get_bit(num, den)?
                    }
                };
                if bit == 1 {
                    mag[i] |= 1u32 << abs_bit;
                }
                cat[i] = cat[i].saturating_add(1).min(3);
                // The refinement bit (0 or 1) confirms the magnitude down
                // to this plane for this coefficient.
                last_bit[i] = abs_bit as u8;
            }
            x += lat.step;
        }
        y += lat.step;
    }
    Ok(())
}

/// Shared schedule validation for the priority-interleaved codec.
fn validate_prioritized(levels: u8, q: u8) -> Result<()> {
    if q == 0 || q > 31 {
        return Err(IcerError::invalid(format!(
            "bit-plane count {q} outside (0,31]"
        )));
    }
    if !(1..=6).contains(&levels) {
        return Err(IcerError::invalid(format!(
            "§III.A priority interleaving requires decomposition levels in [1,6]; got {levels}"
        )));
    }
    Ok(())
}

/// Encode a coefficient buffer as IPN 42-155 §III.A **priority-
/// interleaved** packets, one per [`crate::priority::packet_schedule`]
/// entry: subband bit planes walked in §III.A order, each coded in a
/// single combined raster pass, cut into packets at priority-group
/// boundaries with the fat units
/// (> [`crate::priority::FINE_PACKET_COEFFS`] coefficients) emitted
/// alone so a byte-quota truncation stays close to its exact §III.A
/// schedule position. `min_loss` drops whole subband bit planes per
/// the §VI.A quality goal. The returned packets carry the **priority
/// group index** in `bit_plane` (`is_significance` is `true` for all
/// of them — the pass is combined).
///
/// The context model persists across the whole segment; each packet's
/// body is a fresh entropy-coder run (byte-aligned truncation points).
/// Requires `input.levels` in `[1, 6]` — the §III.A schedule is defined
/// over subbands.
pub fn encode_bitplanes_prioritized(
    input: &BitPlaneInput<'_>,
    kind: crate::entropy::EntropyKind,
    filter: &ScanFilter<'_>,
    min_loss: u8,
) -> Result<Vec<EncodedPacket>> {
    input.validate()?;
    validate_prioritized(input.levels, input.q)?;
    let n = input.coeffs.len();
    filter.validate(n)?;
    let q = input.q as u32;

    let mut significant = vec![false; n];
    let mut sign = vec![false; n];
    let mut cat = vec![0u8; n];
    let mut model = ContextModel::new();

    let schedule = packet_schedule(input.levels, q, min_loss, input.width, input.height);
    let mut packets = Vec::with_capacity(schedule.len());
    for sp in &schedule {
        let mut enc = kind.make_sink();
        let mut delta_distortion = 0.0f64;
        for unit in &sp.units {
            let abs_bit = (q - 1 - unit.bp_from_msb) as usize;
            let mut stats = (0u64, 0u64);
            encode_priority_unit(
                enc.as_mut(),
                &mut model,
                input.coeffs,
                &mut significant,
                &mut sign,
                &mut cat,
                input.width,
                input.height,
                unit,
                abs_bit,
                input.levels,
                filter,
                &mut stats,
            );
            // Same distortion-reduction model as the MSB-down packets
            // (see EncodedPacket::delta_distortion): ~2 * 4^b per newly
            // significant coefficient, 4^b / 4 per refined bit.
            let bp_weight = 4f64.powi(abs_bit as i32);
            delta_distortion += stats.0 as f64 * 2.0 * bp_weight;
            delta_distortion += stats.1 as f64 * 0.25 * bp_weight;
        }
        packets.push(EncodedPacket {
            bit_plane: sp.group_index as u8,
            is_significance: true,
            body: enc.finish_bits(),
            delta_distortion,
        });
    }
    Ok(packets)
}

/// Decode packets produced by [`encode_bitplanes_prioritized`]. Packets
/// must arrive in schedule order (they are emitted that way); a
/// truncated stream simply stops at its last delivered packet, and
/// every coefficient reconstructs at its own §III.A deadzone point from
/// the deepest magnitude bit actually delivered for it.
#[allow(clippy::too_many_arguments)]
pub fn decode_bitplanes_prioritized(
    packets: &[EncodedPacket],
    width: usize,
    height: usize,
    q: u8,
    levels: u8,
    kind: crate::entropy::EntropyKind,
    filter: &ScanFilter<'_>,
    min_loss: u8,
) -> Result<Vec<i32>> {
    validate_prioritized(levels, q)?;
    let n = width * height;
    filter.validate(n)?;
    let q32 = q as u32;

    let mut significant = vec![false; n];
    let mut sign = vec![false; n];
    let mut mag = vec![0u32; n];
    let mut cat = vec![0u8; n];
    let mut last_bit = vec![q; n];
    let mut model = ContextModel::new();

    let schedule = packet_schedule(levels, q32, min_loss, width, height);
    for (cursor, sp) in schedule.iter().enumerate() {
        let Some(pkt) = packets.get(cursor) else {
            // Truncated stream: every remaining packet is undelivered.
            break;
        };
        if pkt.bit_plane != sp.group_index as u8 {
            // The expected packet is missing (mid-stream loss or
            // corruption). Later packets cannot be decoded without it —
            // the context model and significance state would desync — so
            // stop here, exactly like a truncation at this point.
            break;
        }
        let mut dec = kind.make_source(&pkt.body)?;
        for unit in &sp.units {
            let abs_bit = (q32 - 1 - unit.bp_from_msb) as usize;
            decode_priority_unit(
                dec.as_mut(),
                &mut model,
                &mut significant,
                &mut sign,
                &mut mag,
                &mut cat,
                &mut last_bit,
                width,
                height,
                unit,
                abs_bit,
                levels,
                filter,
            )?;
        }
    }

    Ok(deadzone_reconstruct(&mag, &sign, &last_bit))
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
    let stride = subband_stride(x, y, levels);
    gather_pattern_strided(significant, width, height, x, y, stride)
}

/// Interior/edge-split gather of the 8 same-subband neighbour
/// significance flags at a precomputed `stride` (r454 perf). The
/// interior fast path (all eight neighbours in bounds) drops the
/// per-neighbour signed bounds arithmetic that previously ran eight
/// times per visited bit; the edge fallback walks the identical
/// neighbour list. Bit layout unchanged.
#[inline]
fn gather_pattern_strided(
    significant: &[bool],
    width: usize,
    height: usize,
    x: usize,
    y: usize,
    stride: usize,
) -> u8 {
    if x >= stride && y >= stride && x + stride < width && y + stride < height {
        let up = (y - stride) * width + x;
        let mid = y * width + x;
        let down = (y + stride) * width + x;
        u8::from(significant[up - stride])
            | (u8::from(significant[up]) << 1)
            | (u8::from(significant[up + stride]) << 2)
            | (u8::from(significant[mid - stride]) << 3)
            | (u8::from(significant[mid + stride]) << 4)
            | (u8::from(significant[down - stride]) << 5)
            | (u8::from(significant[down]) << 6)
            | (u8::from(significant[down + stride]) << 7)
    } else {
        let s = stride as isize;
        let mut pat = 0u8;
        for (dx, dy, bit) in [
            (-1isize, -1isize, 0u8),
            (0, -1, 1),
            (1, -1, 2),
            (-1, 0, 3),
            (1, 0, 4),
            (-1, 1, 5),
            (0, 1, 6),
            (1, 1, 7),
        ] {
            let nx = x as isize + dx * s;
            let ny = y as isize + dy * s;
            if nx >= 0
                && ny >= 0
                && (nx as usize) < width
                && (ny as usize) < height
                && significant[(ny as usize) * width + (nx as usize)]
            {
                pat |= 1 << bit;
            }
        }
        pat
    }
}

/// One-shot per-visit classification for the significance pass (r454
/// perf): classify the coefficient once, gather its neighbour pattern
/// at that stride, and return the §III.B context together with the
/// `(stride, is_hl)` pair the sign branch reuses — the previous shape
/// re-ran [`classify_position`] two to four times per visited bit.
/// `levels == 0` keeps the legacy subband-agnostic classification.
#[inline]
fn significance_visit(
    significant: &[bool],
    width: usize,
    height: usize,
    x: usize,
    y: usize,
    levels: u8,
) -> (usize, usize, bool) {
    if levels == 0 {
        let pat = gather_pattern_strided(significant, width, height, x, y, 1);
        return (significance_context(pat), 1, false);
    }
    let (kind, level) = classify_position(x, y, levels);
    let stride = 1usize << level;
    let pat = gather_pattern_strided(significant, width, height, x, y, stride);
    let (h, v, d) = neighbour_counts(pat);
    (
        significance_context_subband(h, v, d, kind == SubbandType::Hh, kind == SubbandType::Hl),
        stride,
        kind == SubbandType::Hl,
    )
}

/// Sign context + prediction flip for a coefficient that just turned
/// significant, reusing the `(stride, is_hl)` classification from
/// [`significance_visit`] (r454 perf). The H/V pair patterns are the
/// same-subband W/E and N/S neighbours at the subband stride (IPN
/// 42-155 §III.B); `levels == 0` keeps the subband-agnostic Table 8
/// lookup. Identical bit behaviour to the previous
/// `neighbour_sign_pattern` + `sign_ctx_for` pair.
#[inline]
#[allow(clippy::too_many_arguments)]
fn sign_visit(
    significant: &[bool],
    sign: &[bool],
    width: usize,
    height: usize,
    x: usize,
    y: usize,
    stride: usize,
    is_hl: bool,
    levels: u8,
) -> (usize, bool) {
    let s = stride as isize;
    let h = pair_pattern(
        significant,
        sign,
        width,
        height,
        x as isize - s,
        y as isize,
        x as isize + s,
        y as isize,
    );
    let v = pair_pattern(
        significant,
        sign,
        width,
        height,
        x as isize,
        y as isize - s,
        x as isize,
        y as isize + s,
    );
    if levels == 0 {
        (sign_context(h, v), sign_prediction_flip(h, v))
    } else {
        (
            sign_context_subband(h, v, is_hl),
            sign_prediction_flip_subband(h, v, is_hl),
        )
    }
}

/// Significance probe for the refinement pass's context 9/10 split
/// (r454 perf): probe exactly the neighbours [`HV_NEIGHBOUR_MASK`]
/// selects — N (bit 1), W (bit 3), S (bit 6) at the subband stride —
/// instead of gathering the full 8-neighbour pattern. Exactly
/// `has_hv_significant(neighbour_significance_pattern(..))`, mask
/// semantics included (the historical mask does not test East; the
/// wire-digest suite pins that equivalence).
#[inline]
fn refinement_has_hv(
    significant: &[bool],
    width: usize,
    height: usize,
    x: usize,
    y: usize,
    levels: u8,
) -> bool {
    let stride = subband_stride(x, y, levels);
    let i = y * width + x;
    (x >= stride && significant[i - stride])
        || (y >= stride && significant[i - stride * width])
        || (y + stride < height && significant[i + stride * width])
}

/// Legacy `(h_pattern, v_pattern)` sign-neighbour gather (bits 0,1 =
/// W/N significant+negative, bits 2,3 = E/S) used by the single-packet
/// subband-agnostic path; the multi-packet passes use [`sign_visit`],
/// which reuses the stride from [`significance_visit`] instead of
/// re-deriving it here.
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

    /// End-to-end §IV path: encode a coefficient buffer's bit planes with
    /// the interleaved entropy backend and decode them back losslessly.
    /// The interleaved coder drives the identical §III.B passes the
    /// arithmetic coder does, so a full-quality decode reconstructs every
    /// coefficient exactly.
    #[test]
    fn interleaved_backend_bitplane_roundtrip() {
        use crate::entropy::EntropyKind;
        let w = 16;
        let h = 16;
        let coeffs = lcg_signal(w * h, 512, 0x51C3);
        let q = select_bit_plane_count(&coeffs);
        for levels in 0..=4u8 {
            let input = BitPlaneInput {
                coeffs: &coeffs,
                width: w,
                height: h,
                q,
                levels,
            };
            let packets = encode_bitplanes_with(&input, EntropyKind::Interleaved).unwrap();
            let out =
                decode_bitplanes_multi_with(&packets, w, h, q, levels, EntropyKind::Interleaved)
                    .unwrap();
            assert_eq!(out, coeffs, "interleaved §IV round-trip (levels={levels})");
        }
    }

    /// The interleaved backend produces a *different* byte stream than the
    /// arithmetic backend on the same coefficients (the two entropy coders
    /// pack the identical context/probability stream differently), yet both
    /// round-trip losslessly. Proves the backend switch is real, not a
    /// no-op alias.
    #[test]
    fn interleaved_differs_from_arithmetic() {
        use crate::entropy::EntropyKind;
        let w = 16;
        let h = 16;
        let coeffs = lcg_signal(w * h, 300, 0x9E37);
        let q = select_bit_plane_count(&coeffs);
        let input = BitPlaneInput {
            coeffs: &coeffs,
            width: w,
            height: h,
            q,
            levels: 2,
        };
        let arith = encode_bitplanes_with(&input, EntropyKind::Arithmetic).unwrap();
        let inter = encode_bitplanes_with(&input, EntropyKind::Interleaved).unwrap();
        let arith_bytes: usize = arith.iter().map(|p| p.body.len()).sum();
        let inter_bytes: usize = inter.iter().map(|p| p.body.len()).sum();
        assert_ne!(
            arith_bytes, inter_bytes,
            "the two entropy backends should not produce identical byte totals"
        );
        // Both decode losslessly under their own backend.
        assert_eq!(
            decode_bitplanes_multi_with(&arith, w, h, q, 2, EntropyKind::Arithmetic).unwrap(),
            coeffs
        );
        assert_eq!(
            decode_bitplanes_multi_with(&inter, w, h, q, 2, EntropyKind::Interleaved).unwrap(),
            coeffs
        );
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

    /// [`ScanFilter::ALL`] must be byte-identical to the unfiltered
    /// entry points on both entropy backends.
    #[test]
    fn scan_filter_all_is_wire_identical() {
        let (w, h) = (16usize, 16usize);
        let coeffs = lcg_signal(w * h, 256, 0x51CA);
        let q = select_bit_plane_count(&coeffs);
        let input = BitPlaneInput {
            coeffs: &coeffs,
            width: w,
            height: h,
            q,
            levels: 3,
        };
        for kind in [
            crate::entropy::EntropyKind::Arithmetic,
            crate::entropy::EntropyKind::Interleaved,
        ] {
            let plain = encode_bitplanes_with(&input, kind).unwrap();
            let filtered = encode_bitplanes_filtered(&input, kind, &ScanFilter::ALL).unwrap();
            assert_eq!(plain.len(), filtered.len());
            for (a, b) in plain.iter().zip(filtered.iter()) {
                assert_eq!(a.body, b.body, "backend {kind:?} wire form must not change");
            }
        }
    }

    /// §V.B segment restriction: coding the two halves of a buffer as
    /// separate segment scans and merging the decodes reproduces the
    /// original coefficients exactly, and each segment's decode leaves
    /// out-of-segment coefficients at zero.
    #[test]
    fn scan_filter_segment_split_roundtrips() {
        let (w, h) = (16usize, 12usize);
        let coeffs = lcg_signal(w * h, 200, 0xB0A7);
        let q = select_bit_plane_count(&coeffs);
        // Left/right split map (a §V.B-shaped mask; the real map comes
        // from partition::coefficient_segment_map in the pipeline).
        let map: Vec<u16> = (0..w * h).map(|i| u16::from(i % w >= w / 2)).collect();
        let mut merged = vec![0i32; w * h];
        for seg in 0..2u16 {
            let filter = ScanFilter {
                segment: Some((&map, seg)),
                skip: None,
                window: None,
            };
            let input = BitPlaneInput {
                coeffs: &coeffs,
                width: w,
                height: h,
                q,
                levels: 2,
            };
            let packets =
                encode_bitplanes_filtered(&input, crate::entropy::EntropyKind::Arithmetic, &filter)
                    .unwrap();
            let out = decode_bitplanes_filtered(
                &packets,
                w,
                h,
                q,
                2,
                crate::entropy::EntropyKind::Arithmetic,
                &filter,
            )
            .unwrap();
            for i in 0..w * h {
                if map[i] == seg {
                    merged[i] = out[i];
                } else {
                    assert_eq!(out[i], 0, "out-of-segment coefficient must stay zero");
                }
            }
        }
        assert_eq!(merged, coeffs, "two-segment merge must be lossless");
    }

    /// §VI.A skip restriction: a skip map zeroes exactly the excluded
    /// LSB planes (with the §III.A deadzone bias on the kept planes) and
    /// decodes without desynchronising.
    #[test]
    fn scan_filter_skip_planes_roundtrips() {
        let (w, h) = (12usize, 12usize);
        let coeffs = lcg_signal(w * h, 512, 0x0FF5);
        let q = select_bit_plane_count(&coeffs);
        // Uniform two-plane exclusion.
        let skip = vec![2u8; w * h];
        let filter = ScanFilter {
            segment: None,
            skip: Some(&skip),
            window: None,
        };
        let input = BitPlaneInput {
            coeffs: &coeffs,
            width: w,
            height: h,
            q,
            levels: 2,
        };
        let packets =
            encode_bitplanes_filtered(&input, crate::entropy::EntropyKind::Arithmetic, &filter)
                .unwrap();
        let out = decode_bitplanes_filtered(
            &packets,
            w,
            h,
            q,
            2,
            crate::entropy::EntropyKind::Arithmetic,
            &filter,
        )
        .unwrap();
        for (i, (&orig, &dec)) in coeffs.iter().zip(out.iter()).enumerate() {
            let mag = orig.unsigned_abs();
            if mag < 4 {
                // Entirely inside the excluded planes: deadzone centre.
                assert_eq!(dec, 0, "coeff {i}: {orig} within deadzone");
            } else {
                // Kept planes exact, excluded planes at the §III.A
                // mid-bin point biased toward the origin: ∆ = 4,
                // reconstruction = lower_edge + ∆/2 - 1.
                let expect_mag = (mag & !3) + 1;
                let expect = if orig < 0 {
                    -(expect_mag as i32)
                } else {
                    expect_mag as i32
                };
                assert_eq!(dec, expect, "coeff {i}: {orig}");
            }
        }
    }
}
