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
//!   * **Category-aware refinement contexts** (3 total, indices 9, 10,
//!     11) per the IPN 42-155 §III.B four-category pixel scheme. ICER
//!     keeps a *category* for each pixel that counts the magnitude bits
//!     already encoded: category 0 = not yet significant; category 1 =
//!     just became significant (the first magnitude '1' bit was coded);
//!     category 2 = one more magnitude bit coded; category 3 = one more
//!     again, and stays 3 permanently. The magnitude-bit context is then:
//!       - category 1 -> context 9 if no horizontally / vertically
//!         adjacent pixel is significant, else context 10;
//!       - category 2 -> context 11 regardless of neighbours;
//!       - category 3 -> the bit is empirically incompressible and is
//!         **left uncoded** (sent at a fixed probability-of-zero of 1/2,
//!         with no model update) per §III.B.
//!   * **Sign contexts** (5 total, indices 12..=16). Derived from the
//!     horizontal and vertical neighbour sign contributions per
//!     IPN 42-155 §III.B Table 8: each axis's signed sum `h1 + h2` /
//!     `v1 + v2` (neighbours contribute +1 positive-significant, -1
//!     negative-significant, 0 insignificant) selects both a predicted
//!     sign and one of contexts 12..=16.
//!
//! The estimator is a Laplace-rule windowed-counting adaptive
//! probability model (IPN 42-155 §III.C "windowed counting") with a
//! 64-symbol halving window.

/// Total number of contexts maintained by the model. The IPN 42-155
/// §III.B layout uses exactly 17 contexts: 0..=8 significance (category
/// 0), 9 and 10 category-1 refinement, 11 category-2 refinement, and
/// 12..=16 sign. Category-3 magnitude bits are left uncoded and use no
/// context (see [`MagnitudeContext::Uncoded`]).
pub const CONTEXT_COUNT: usize = 17;

/// Index of the lone category-2 refinement context (IPN 42-155 §III.B:
/// "A bit of a pixel in category 2 is assigned context 11 regardless of
/// the categories of adjacent pixels").
pub const CATEGORY2_CONTEXT: usize = 11;

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

/// Sign-context lookup per IPN 42-155 §III.B **Table 8**.
///
/// ICER uses the two horizontally adjacent (W, E) and the two vertically
/// adjacent (N, S) pixels. Each contributes +1 if significant-positive,
/// -1 if significant-negative, 0 if insignificant. The axis sums
/// `h1 + h2` and `v1 + v2` (each in {-2,-1,0,+1,+2}) select both a
/// predicted sign and a context per Table 8:
///
/// |              | h1+h2 < 0 | h1+h2 = 0 | h1+h2 > 0 |
/// |--------------|-----------|-----------|-----------|
/// | v1+v2 < 0    |  -, 16    |  +, 13    |  +, 14    |
/// | v1+v2 = 0    |  -, 15    |  +, 12    |  +, 15    |
/// | v1+v2 > 0    |  -, 14    |  -, 13    |  +, 16    |
///
/// `h_pattern` / `v_pattern` each pack the two axis neighbours:
///   * bits 0,1 -- (neighbour-A significant, neighbour-A negative)
///   * bits 2,3 -- (neighbour-B significant, neighbour-B negative)
///
/// For horizontal: A=W, B=E. For vertical: A=N, B=S.
pub fn sign_context(h_pattern: u8, v_pattern: u8) -> usize {
    let h = axis_sign_sum(h_pattern);
    let v = axis_sign_sum(v_pattern);
    sign_table8(h, v).1
}

/// Return the predicted sign bit per IPN 42-155 §III.B Table 8. When the
/// prediction is negative the encoder flips the sign bit before coding
/// it (the "agreement bit" convention), so the model always sees a bit
/// whose `1` means "agrees with prediction". Returns `true` if the
/// predicted sign is negative (i.e. the coder should flip the bit).
pub fn sign_prediction_flip(h_pattern: u8, v_pattern: u8) -> bool {
    let h = axis_sign_sum(h_pattern);
    let v = axis_sign_sum(v_pattern);
    sign_table8(h, v).0
}

/// IPN 42-155 §III.B Table 8 — `(predicted_sign_is_negative, context)`
/// as a function of the horizontal and vertical axis sign sums.
fn sign_table8(h: i8, v: i8) -> (bool, usize) {
    use core::cmp::Ordering::{Equal, Greater, Less};
    match (h.cmp(&0), v.cmp(&0)) {
        // v1+v2 < 0
        (Less, Less) => (true, 16),
        (Equal, Less) => (false, 13),
        (Greater, Less) => (false, 14),
        // v1+v2 = 0
        (Less, Equal) => (true, 15),
        (Equal, Equal) => (false, 12),
        (Greater, Equal) => (false, 15),
        // v1+v2 > 0
        (Less, Greater) => (true, 14),
        (Equal, Greater) => (true, 13),
        (Greater, Greater) => (false, 16),
    }
}

/// Signed sum of one axis's two neighbours, in {-2,-1,0,+1,+2}.
///
/// Layout: bits 0,1 = (A-significant, A-negative); bits 2,3 =
/// (B-significant, B-negative). A significant-positive neighbour adds
/// +1, a significant-negative neighbour adds -1, an insignificant one 0.
fn axis_sign_sum(pattern: u8) -> i8 {
    let mut sum = 0i8;
    if pattern & 0b0001 != 0 {
        sum += if pattern & 0b0010 != 0 { -1 } else { 1 };
    }
    if pattern & 0b0100 != 0 {
        sum += if pattern & 0b1000 != 0 { -1 } else { 1 };
    }
    sum
}

/// How a magnitude (significance-pass-survivor) refinement bit is coded,
/// driven by the pixel's IPN 42-155 §III.B category.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MagnitudeContext {
    /// Code the bit adaptively against the given context index
    /// (category 1 -> 9 or 10, category 2 -> 11).
    Coded(usize),
    /// Category-3 bit: empirically incompressible, left uncoded — coded
    /// at a fixed probability-of-zero of 1/2 with no model update.
    Uncoded,
}

/// Select the magnitude-bit coding mode for a refinement bit from the
/// pixel's category and whether any horizontally / vertically adjacent
/// pixel is significant (IPN 42-155 §III.B):
///
///   * category 1 -> context 9 (no H/V significant neighbour) or 10;
///   * category 2 -> context 11;
///   * category 3 -> uncoded.
///
/// Categories 0 (insignificant — handled by the significance pass) and
/// any value `>= 3` map to [`MagnitudeContext::Uncoded`]; only categories
/// 1 and 2 are adaptively coded.
pub fn magnitude_context(category: u8, has_hv_significant_neighbour: bool) -> MagnitudeContext {
    match category {
        1 => MagnitudeContext::Coded(if has_hv_significant_neighbour { 10 } else { 9 }),
        2 => MagnitudeContext::Coded(CATEGORY2_CONTEXT),
        _ => MagnitudeContext::Uncoded,
    }
}

/// Probability-of-one numerator / denominator for an uncoded bit: a flat
/// 1/2 (`1` out of `2`). Fed straight to the arithmetic coder for
/// category-3 magnitude bits with no model adaptation.
pub const UNCODED_P1: (u32, u32) = (1, 2);

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
                    (12..=16).contains(&c),
                    "h={h} v={v} produced out-of-range sign context {c}"
                );
            }
        }
    }

    #[test]
    fn sign_table8_corners() {
        // The nine Table 8 cells, addressed via the packed neighbour
        // bytes. Helpers: pos = significant-positive (0b01), neg =
        // significant-negative (0b11) for neighbour A; same shifted for B.
        let pos = 0b0001u8; // A significant, positive
        let neg = 0b0011u8; // A significant, negative
        let both_pos = 0b0101u8; // A + B both significant-positive (sum +2)
        let both_neg = 0b1111u8; // A + B both significant-negative (sum -2)
        let zero = 0u8;

        // h>0, v>0 -> +,16 ; h<0,v<0 -> -,16
        assert_eq!(sign_context(both_pos, both_pos), 16);
        assert!(!sign_prediction_flip(both_pos, both_pos));
        assert_eq!(sign_context(both_neg, both_neg), 16);
        assert!(sign_prediction_flip(both_neg, both_neg));
        // h=0,v=0 -> +,12
        assert_eq!(sign_context(zero, zero), 12);
        assert!(!sign_prediction_flip(zero, zero));
        // h<0,v=0 -> -,15 ; h>0,v=0 -> +,15
        assert_eq!(sign_context(neg, zero), 15);
        assert!(sign_prediction_flip(neg, zero));
        assert_eq!(sign_context(pos, zero), 15);
        assert!(!sign_prediction_flip(pos, zero));
        // h=0,v<0 -> +,13 ; h=0,v>0 -> -,13
        assert_eq!(sign_context(zero, neg), 13);
        assert!(!sign_prediction_flip(zero, neg));
        assert_eq!(sign_context(zero, pos), 13);
        assert!(sign_prediction_flip(zero, pos));
        // h>0,v<0 -> +,14 ; h<0,v>0 -> -,14
        assert_eq!(sign_context(pos, neg), 14);
        assert!(!sign_prediction_flip(pos, neg));
        assert_eq!(sign_context(neg, pos), 14);
        assert!(sign_prediction_flip(neg, pos));
    }

    #[test]
    fn magnitude_context_categories() {
        // Category 1: 9 with no H/V neighbour, 10 with one.
        assert_eq!(magnitude_context(1, false), MagnitudeContext::Coded(9));
        assert_eq!(magnitude_context(1, true), MagnitudeContext::Coded(10));
        // Category 2: always 11.
        assert_eq!(
            magnitude_context(2, false),
            MagnitudeContext::Coded(CATEGORY2_CONTEXT)
        );
        assert_eq!(
            magnitude_context(2, true),
            MagnitudeContext::Coded(CATEGORY2_CONTEXT)
        );
        // Category 3 (and beyond): uncoded.
        assert_eq!(magnitude_context(3, true), MagnitudeContext::Uncoded);
        assert_eq!(magnitude_context(4, false), MagnitudeContext::Uncoded);
    }
}
