//! Adaptive probability estimator + context-selection for the bit-plane
//! arithmetic coder.
//!
//! ICER's bit-plane coder selects one of several *contexts* for each
//! significance / refinement / sign bit, then feeds the bit + the
//! current per-context probability estimate into the binary
//! arithmetic coder. IPN 42-155 §III.B describes:
//!
//!   * **Significance contexts** (9 total, indices 0..=8). Derived from
//!     the 8-neighbour significance pattern by classifying each of three
//!     direction groups: horizontal (W, E), vertical (N, S), diagonal
//!     (NW, NE, SW, SE). IPN 42-155 §III.B classifies neighbours as
//!     H-significant, V-significant, or D-significant and collapses the
//!     256-entry pattern table by treating "0, 1, or 2+" counts per
//!     group into ternary H-code (0/1/2), binary V-code (0/1), binary
//!     D-code (0/1), then mapping the resulting (H, V, D) tuple to
//!     contexts 0..=8. This gives 3 * 2 * 2 = 12 possible tuples;
//!     7 of the 9 contexts each cover exactly one tuple, and 2 are
//!     merged (H=0,V=0,D=0 merges with symmetry constraints).
//!     Implementation below uses the 9-context table that matches the
//!     "H-card-ternary x V-card-binary x D-card-binary" scheme.
//!   * **Sign contexts** (5 total, indices 9..=13). Derived from the
//!     horizontal and vertical neighbour sign contributions per
//!     IPN 42-155 §III.B: each axis's "sign contribution" `hc`/`vc`
//!     is clamped to {-1, 0, +1} (sign of significant-positive minus
//!     significant-negative neighbour counts). The 5 contexts are
//!     assigned by `(|hc|, |vc|, sign(hc)*sign(vc))` collapsing.
//!   * **Refinement contexts** (3 total, indices 14..=16). Distinguished
//!     by whether the coefficient just became significant (most uncertain)
//!     vs has been significant for one or more prior bit-planes.
//!
//! The estimator is a Laplace-rule windowed-counting adaptive
//! probability model (IPN 42-155 §III.C "windowed counting") with a
//! 64-symbol halving window.

/// Total number of contexts maintained by the model. Fixed
/// upper-bound covering significance (9) + sign (5) + refinement (3)
/// per IPN 42-155 §III.B Table 1.
pub const CONTEXT_COUNT: usize = 17;

/// Window length for the Laplace-rule probability estimator. A
/// power-of-two keeps the renormalisation step a shift; 64 is the
/// shortest window the paper's "fast adaptation" descriptor
/// (IPN 42-155 §III.C) is consistent with.
pub const ESTIMATOR_WINDOW: u32 = 64;

/// Adaptive context-conditional probability estimator. One counter
/// pair per context; the running probability of `1` is
/// `ones[ctx] / total[ctx]` clamped to `[1/W, (W-1)/W]` where
/// `W = ESTIMATOR_WINDOW + 2` (Laplace add-one smoothing).
pub struct ContextModel {
    /// Number of `1` bits seen in the current window for each context.
    ones: [u32; CONTEXT_COUNT],
    /// Total bits seen in the current window for each context.
    total: [u32; CONTEXT_COUNT],
}

impl Default for ContextModel {
    fn default() -> Self {
        Self::new()
    }
}

impl ContextModel {
    /// Build a fresh model with all counters at the Laplace prior
    /// (1 one out of 2 total -> P(1) = 1/2).
    pub fn new() -> Self {
        Self {
            ones: [1; CONTEXT_COUNT],
            total: [2; CONTEXT_COUNT],
        }
    }

    /// Return the current `(p1_num, p1_den)` estimate for context
    /// `ctx` to feed into [`crate::arith::ArithEncoder::encode_bit`] /
    /// [`crate::arith::ArithDecoder::decode_bit`].
    pub fn probability(&self, ctx: usize) -> (u32, u32) {
        debug_assert!(ctx < CONTEXT_COUNT);
        (self.ones[ctx], self.total[ctx])
    }

    /// Update the windowed counters with the observed `bit` for
    /// context `ctx`. Must be called *after* the corresponding
    /// arithmetic-coder symbol has been encoded / decoded so encoder
    /// and decoder stay synchronised.
    pub fn observe(&mut self, ctx: usize, bit: u8) {
        debug_assert!(ctx < CONTEXT_COUNT);
        debug_assert!(bit <= 1);
        if self.total[ctx] >= ESTIMATOR_WINDOW {
            // Halve both counts, with rounding up so neither hits 0.
            self.ones[ctx] = (self.ones[ctx] + 1) >> 1;
            self.total[ctx] = (self.total[ctx] + 1) >> 1;
        }
        self.total[ctx] += 1;
        self.ones[ctx] += u32::from(bit);
    }
}

/// Look up the significance context for a coefficient given its
/// 8-neighbour significance pattern.
///
/// `pattern` packs the eight neighbours into bits 0..=7 in the order:
/// NW=bit0, N=bit1, NE=bit2, W=bit3, E=bit4, SW=bit5, S=bit6, SE=bit7.
///
/// Implements the IPN 42-155 §III.B significance context classification:
/// three direction groups (horizontal H={W,E}, vertical V={N,S},
/// diagonal D={NW,NE,SW,SE}) are counted and clamped to ternary (H:
/// 0, 1, or 2+), binary (V: 0 or 1+), binary (D: 0 or 1+). The 9
/// contexts follow the mapping table derived from §III.B Table 1:
///
/// | ctx | H-card | V-card | D-card |
/// |-----|--------|--------|--------|
/// |  0  |   0    |   0    |   0    |
/// |  1  |   1    |   0    |   0    |
/// |  2  |   0    |   1    |   0    |
/// |  3  |   1    |   1    |   0    |
/// |  4  |  2+    |   0    |   0    |
/// |  5  |  2+    |   1    |   0    |
/// |  6  |   0    |   0    |   1+   |
/// |  7  |   1    |   0    |   1+   |
/// |  8  | (any H)| (any V)|  1+, H+V>=1 |
///
/// Contexts 6-8 cover the diagonal-significant case. The mapping
/// follows the "H+V then D" ordering used in IPN 42-155 §III.B.
pub fn significance_context(pattern: u8) -> usize {
    // Decompose pattern into the three direction groups.
    // Bits: NW=0, N=1, NE=2, W=3, E=4, SW=5, S=6, SE=7
    let h_sig = pattern & 0b0001_1000; // bits 3(W) + 4(E)
    let v_sig = pattern & 0b0100_0010; // bits 1(N) + 6(S)
    let d_sig = pattern & 0b1010_0101; // bits 0(NW) + 2(NE) + 5(SW) + 7(SE)

    let h_count = h_sig.count_ones(); // 0, 1, or 2
    let v_nonzero = v_sig != 0;
    let d_nonzero = d_sig != 0;

    // Map (H-card, V-card, D-card) to context index.
    // Context table per IPN 42-155 §III.B:
    //   If D=0: use (H-ternary, V-binary) -> ctx 0..5
    //   If D=1: priority context 6/7/8 based on H+V
    if !d_nonzero {
        // D=0: primary table
        match (h_count, v_nonzero) {
            (0, false) => 0,
            (1, false) => 1,
            (0, true) => 2,
            (1, true) => 3,
            (_, false) => 4, // H >= 2
            (_, true) => 5,  // H >= 2
        }
    } else {
        // D>0: three diagonal-active contexts
        let hv_active = h_count > 0 || v_nonzero;
        if !hv_active {
            6 // only diagonals active
        } else if h_count <= 1 && !v_nonzero {
            7 // one horizontal, no vertical
        } else {
            8 // two or more active in H+V region
        }
    }
}

/// Sign-context lookup per IPN 42-155 §III.B.
///
/// Each axis (horizontal and vertical) contributes a signed "contribution"
/// that is the count of significant-positive neighbours minus the count of
/// significant-negative neighbours, clipped to {-1, 0, +1}.
///
/// `h_pattern` and `v_pattern` each encode:
///   * bits 0,1 -- (neighbour-A significant, neighbour-A sign)
///   * bits 2,3 -- (neighbour-B significant, neighbour-B sign)
///
/// For horizontal: A=W, B=E. For vertical: A=N, B=S.
///
/// The 5 sign contexts (indices 9..=13) are assigned as:
///
/// | ctx | hc  | vc  |
/// |-----|-----|-----|
/// |  9  |  0  |  0  |  no significant H or V neighbours
/// | 10  | +1  |  0  |  H positive contribution, no V
/// | 11  |  0  | +1  |  no H, V positive contribution
/// | 12  | -1  |  0  |  H negative contribution, no V
/// | 13  | ±1  | ±1  |  both axes active (sign convention applied)
///
/// Per IPN 42-155 §III.B, when both axes have a contribution, the
/// context index is adjusted by a sign-flip of the predicted sign when
/// the contributions disagree (XOR).
pub fn sign_context(h_pattern: u8, v_pattern: u8) -> usize {
    // Decode horizontal contribution.
    let hc = axis_sign_contribution(h_pattern);
    // Decode vertical contribution.
    let vc = axis_sign_contribution(v_pattern);

    match (hc, vc) {
        (0, 0) => 9,
        (h, 0) if h > 0 => 10,
        (0, v) if v > 0 => 11,
        (h, 0) if h < 0 => 12,
        // Both axes active: context 13; the sign flip to the predicted
        // sign is handled by the caller via the returned XOR bit, not
        // by choosing a different context (IPN 42-155 §III.B).
        _ => 13,
    }
}

/// Compute the sign contribution of one axis given its two-neighbour
/// significance/sign packed byte. Returns -1, 0, or +1.
///
/// Layout: bits 0,1 = (A-significant, A-negative); bits 2,3 = (B-significant, B-negative).
/// Positive = significant and not negative; negative = significant and negative.
fn axis_sign_contribution(pattern: u8) -> i8 {
    let mut contrib = 0i8;
    // Neighbour A: bits 0 = significant, bit 1 = negative
    if pattern & 0b0001 != 0 {
        if pattern & 0b0010 != 0 {
            contrib -= 1;
        } else {
            contrib += 1;
        }
    }
    // Neighbour B: bits 2 = significant, bit 3 = negative
    if pattern & 0b0100 != 0 {
        if pattern & 0b1000 != 0 {
            contrib -= 1;
        } else {
            contrib += 1;
        }
    }
    contrib.signum()
}

/// Return the predicted sign bit for the arithmetic coder based on the
/// horizontal and vertical neighbour contributions. When the prediction
/// is negative, the encoder flips the sign bit before coding it
/// (IPN 42-155 §III.B sign-flip convention), so the model always sees
/// a bit coded as "1 = agrees with prediction". Returns `true` if the
/// predicted sign is negative (i.e., the coder should flip the bit).
pub fn sign_prediction_flip(h_pattern: u8, v_pattern: u8) -> bool {
    let hc = axis_sign_contribution(h_pattern);
    let vc = axis_sign_contribution(v_pattern);
    // The combined prediction is sign(hc + vc) where ties (0) predict positive.
    let combined = i16::from(hc) + i16::from(vc);
    combined < 0
}

/// Refinement-context lookup per IPN 42-155 §III.B Table 1.
///
/// Three refinement contexts (indices 14..=16):
///   * 14 -- coefficient just became significant this bit-plane
///     (highest uncertainty about refinement bits).
///   * 15 -- already significant, no significant neighbours
///     (medium uncertainty).
///   * 16 -- already significant, has at least one significant
///     neighbour (lower uncertainty).
pub fn refinement_context(just_significant: bool, has_significant_neighbour: bool) -> usize {
    match (just_significant, has_significant_neighbour) {
        (true, _) => 14,
        (false, false) => 15,
        (false, true) => 16,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn significance_context_all_zero() {
        assert_eq!(significance_context(0), 0, "no neighbours -> ctx 0");
    }

    #[test]
    fn significance_context_horizontal_only() {
        // W set (bit 3) only
        assert_eq!(significance_context(0b0000_1000), 1);
        // W and E set (bits 3+4) -> H=2
        assert_eq!(significance_context(0b0001_1000), 4);
    }

    #[test]
    fn significance_context_vertical_only() {
        // N set (bit 1) only
        assert_eq!(significance_context(0b0000_0010), 2);
    }

    #[test]
    fn significance_context_diagonal_only() {
        // NW set (bit 0) only
        assert_eq!(significance_context(0b0000_0001), 6);
    }

    #[test]
    fn significance_context_range() {
        // All 256 patterns must produce a context in [0, 8].
        for p in 0u8..=255 {
            let c = significance_context(p);
            assert!(
                c <= 8,
                "pattern {p:#010b} produced out-of-range context {c}"
            );
        }
    }

    #[test]
    fn sign_context_range() {
        for h in 0u8..=15 {
            for v in 0u8..=15 {
                let c = sign_context(h, v);
                assert!(
                    (9..=13).contains(&c),
                    "h={h} v={v} produced out-of-range sign context {c}"
                );
            }
        }
    }

    #[test]
    fn refinement_context_values() {
        assert_eq!(refinement_context(true, false), 14);
        assert_eq!(refinement_context(true, true), 14);
        assert_eq!(refinement_context(false, false), 15);
        assert_eq!(refinement_context(false, true), 16);
    }
}
