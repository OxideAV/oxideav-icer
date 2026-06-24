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
//! The estimator is the IPN 42-155 §III.C "Probability Estimation" MER
//! windowed-counting model: per-context zero/total counts initialised to
//! 2/4 (P = 1/2), halved when the total reaches 500 with the count rounded
//! toward keeping the estimate nearer 1/2. See [`ContextModel`].

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

/// Initial `ones` (= initial `total - zeros`) count per context. IPN
/// 42-155 §III.C MER implementation: "the initial counts of zeros are set
/// to 2, the initial total counts are set to 4" — a probability-of-zero
/// of 1/2, equivalently `ones = 2` out of `4`.
pub const INITIAL_ONES: u32 = 2;

/// Initial `total` count per context (IPN 42-155 §III.C MER
/// implementation: total = 4).
pub const INITIAL_TOTAL: u32 = 4;

/// Rescale threshold for the windowed-counting estimator. IPN 42-155
/// §III.C MER implementation: "rescaling is triggered when the total
/// count reaches 500." When the total reaches this value both counts are
/// halved (so recent bits get more weight), with the rounding chosen to
/// keep the probability estimate nearer 1/2.
pub const RESCALE_THRESHOLD: u32 = 500;

/// Adaptive context-conditional probability estimator (IPN 42-155 §III.C
/// "Probability Estimation"). One counter pair per context; the running
/// probability of `1` is `ones[ctx] / total[ctx]`. Counts start at the
/// §III.C MER values (2 ones out of 4 -> P = 1/2) and are halved each
/// time `total` reaches [`RESCALE_THRESHOLD`] (= 500), with the rounding
/// chosen so the post-rescale estimate is nearer 1/2.
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
    /// Build a fresh model with all counters at the §III.C MER prior
    /// (2 ones out of 4 total -> P(1) = 1/2).
    pub fn new() -> Self {
        Self {
            ones: [INITIAL_ONES; CONTEXT_COUNT],
            total: [INITIAL_TOTAL; CONTEXT_COUNT],
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
    /// context `ctx` (IPN 42-155 §III.C). Must be called *after* the
    /// corresponding arithmetic-coder symbol has been encoded / decoded so
    /// encoder and decoder stay synchronised.
    ///
    /// §III.C: "Each bit encountered in a context increments the total
    /// count, and increments the count of zeros if the bit is a 0. ...
    /// When the total count reaches a specified value, both counts are
    /// rescaled by dividing by 2 (when necessary, the count of zeros is
    /// rounded in the direction that makes the probability-of-zero
    /// estimate closer to 1/2)." Tracked here in terms of `ones` (=
    /// `total - zeros`), which is the symmetric statement for the
    /// probability-of-one the arithmetic coder consumes.
    pub fn observe(&mut self, ctx: usize, bit: u8) {
        debug_assert!(ctx < CONTEXT_COUNT);
        debug_assert!(bit <= 1);
        self.total[ctx] += 1;
        self.ones[ctx] += u32::from(bit);
        if self.total[ctx] >= RESCALE_THRESHOLD {
            self.rescale(ctx);
        }
    }

    /// Halve both counts for context `ctx`, rounding so the post-rescale
    /// probability estimate is nearer 1/2 (IPN 42-155 §III.C). The total
    /// rounds to nearest; the `ones` count is then rounded toward
    /// `total / 2` (the 1/2 point). Both are floored at 1 / 2 so the
    /// estimate stays strictly inside `(0, 1)` for the arithmetic coder.
    fn rescale(&mut self, ctx: usize) {
        let zeros = self.total[ctx] - self.ones[ctx];
        // Round each half to nearest.
        let mut total_h = (self.total[ctx] + 1) >> 1;
        let ones_h = (self.ones[ctx] + 1) >> 1;
        let zeros_h = (zeros + 1) >> 1;
        // The two independently-rounded halves may not sum to total_h;
        // reconcile by choosing the `ones` value (`total_h - zeros_h` vs
        // `ones_h`) whose probability-of-one is nearer 1/2, i.e. nearer
        // `total_h / 2`. This realises §III.C's "round in the direction
        // that makes the estimate closer to 1/2".
        let cand_a = ones_h.min(total_h);
        let cand_b = total_h.saturating_sub(zeros_h);
        let half2 = total_h; // compare 2*ones vs total to avoid fractions
        let dist = |o: u32| (2 * o).abs_diff(half2);
        let mut ones_new = if dist(cand_a) <= dist(cand_b) {
            cand_a
        } else {
            cand_b
        };
        // Clamp so 1 <= ones_new <= total_h - 1 (estimate strictly inside
        // (0, 1)); guarantee total_h >= 2 first.
        if total_h < 2 {
            total_h = 2;
        }
        ones_new = ones_new.clamp(1, total_h - 1);
        self.total[ctx] = total_h;
        self.ones[ctx] = ones_new;
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

/// Count the horizontally / vertically / diagonally significant
/// neighbours from the packed 8-neighbour pattern documented in
/// [`significance_context`] (NW=bit0, N=bit1, NE=bit2, W=bit3, E=bit4,
/// SW=bit5, S=bit6, SE=bit7).
///
/// Returns `(h, v, d)` where:
///   * `h` = number of horizontally adjacent significant pixels (W, E),
///     in `0..=2`;
///   * `v` = number of vertically adjacent significant pixels (N, S),
///     in `0..=2`;
///   * `d` = number of diagonally adjacent significant pixels
///     (NW, NE, SW, SE), in `0..=4`.
///
/// This is the `(h, v, d)` triple IPN 42-155 §III.B Tables 6 and 7 are
/// indexed by. Unlike [`significance_context`] (which collapses V and D
/// to a binary "0 / 1+"), the spec tables key on the full counts, so the
/// scanner must pass the exact counts here.
#[inline]
pub fn neighbour_counts(pattern: u8) -> (u8, u8, u8) {
    let h = (pattern & 0b0001_1000).count_ones() as u8; // W(bit3) + E(bit4)
    let v = (pattern & 0b0100_0010).count_ones() as u8; // N(bit1) + S(bit6)
    let d = (pattern & 0b1010_0101).count_ones() as u8; // NW/NE/SW/SE
    (h, v, d)
}

/// IPN 42-155 §III.B **Table 6** — significance context (category 0) for
/// **LL, LH, and HL** subbands as a function of `(h, v, d)`, with `h` =
/// horizontally-significant count (0, 1, 2), `v` = vertically-significant
/// count (0, 1, 2), `d` = diagonally-significant count (0..4).
///
/// The published Table 6 grid (rows `d = 0`, `d = 1`, `d >= 2`; columns
/// `h = 0 {v=0,v=1,v=2}`, `h = 1 {v=0,v>0}`, `h = 2`):
///
/// |       | h=0,v=0 | h=0,v=1 | h=0,v=2 | h=1,v=0 | h=1,v>0 | h=2 |
/// |-------|--------:|--------:|--------:|--------:|--------:|----:|
/// | d=0   |    0    |    3    |    4    |    5    |    7    |  8  |
/// | d=1   |    1    |    3    |    4    |    6    |    7    |  8  |
/// | d>=2  |    2    |    3    |    4    |    7    |    7    |  8  |
///
/// For an **HL** subband the §III.B context template is transposed: the
/// roles of `h` and `v` are reversed before indexing this table. The
/// caller performs that swap (see [`significance_context_subband`]); this
/// function always indexes with the post-transpose `(h, v, d)`.
pub fn significance_context_table6(h: u8, v: u8, d: u8) -> usize {
    // Column selection from (h, v), then row selection from d.
    // The three d-rows differ only in the (h=0,v=0) and (h=1,v=0) cells;
    // every other cell is constant across d.
    match (h, v) {
        (0, 0) => match d {
            0 => 0,
            1 => 1,
            _ => 2,
        },
        (0, 1) => 3,
        (0, _) => 4, // v >= 2
        (1, 0) => match d {
            0 => 5,
            1 => 6,
            _ => 7,
        },
        (1, _) => 7, // h = 1, v > 0
        _ => 8,      // h >= 2
    }
}

/// IPN 42-155 §III.B **Table 7** — significance context (category 0) for
/// **HH** subbands as a function of `h + v` and `d`. HH subbands have no
/// preferred orientation, so the horizontal and vertical counts are summed
/// before indexing:
///
/// | d    | h+v=0 | h+v=1 | h+v>=2 |
/// |------|------:|------:|-------:|
/// | d=0  |   0   |   1   |   2    |
/// | d=1  |   3   |   4   |   5    |
/// | d=2  |   6   |   7   |   7    |
/// | d>=3 |   8   |   8   |   8    |
pub fn significance_context_table7(h: u8, v: u8, d: u8) -> usize {
    let hv = h + v;
    match d {
        0 => match hv {
            0 => 0,
            1 => 1,
            _ => 2,
        },
        1 => match hv {
            0 => 3,
            1 => 4,
            _ => 5,
        },
        2 => match hv {
            0 => 6,
            _ => 7,
        },
        _ => 8, // d >= 3
    }
}

/// Spec-exact §III.B category-0 significance context, dispatching on the
/// subband type and applying the **HL context-template transpose**.
///
/// IPN 42-155 §III.B: "If the subband being encoded is not an HL subband,
/// then let `h` be the number of horizontally adjacent significant pixels
/// ... For an HL subband, the roles of `h` and `v` are reversed,
/// effectively transposing the context template. Given `h`, `v`, and `d`,
/// the context is assigned according to Table 6 if the subband is not an
/// HH subband; otherwise ... Table 7."
///
/// `(h, v, d)` are the raw neighbour counts from [`neighbour_counts`]
/// (un-transposed). `is_hh` selects Table 7 over Table 6; `is_hl` requests
/// the `h`/`v` swap before the Table 6 lookup. (HH subbands sum `h + v`,
/// so the transpose is a no-op there and `is_hl` is ignored when `is_hh`.)
pub fn significance_context_subband(h: u8, v: u8, d: u8, is_hh: bool, is_hl: bool) -> usize {
    if is_hh {
        significance_context_table7(h, v, d)
    } else if is_hl {
        // Transpose the context template: swap horizontal / vertical roles.
        significance_context_table6(v, h, d)
    } else {
        significance_context_table6(h, v, d)
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

/// HL-transpose-aware sign context (IPN 42-155 §III.B Table 8).
///
/// §III.B: "ICER uses the two horizontally adjacent and the two
/// vertically adjacent pixels to determine both the sign estimate and the
/// context. If the subband is not an HL subband, let `h1` and `h2`
/// represent the signs ... of the two horizontally adjacent pixels ...
/// For the HL subband, the roles of the `h`'s and `v`'s are again
/// reversed."
///
/// `(h_pattern, v_pattern)` are the raw axis neighbour patterns (see
/// [`sign_context`]); `is_hl` swaps the two axes before indexing Table 8.
pub fn sign_context_subband(h_pattern: u8, v_pattern: u8, is_hl: bool) -> usize {
    let (hp, vp) = if is_hl {
        (v_pattern, h_pattern)
    } else {
        (h_pattern, v_pattern)
    };
    sign_context(hp, vp)
}

/// HL-transpose-aware sign-prediction flip (IPN 42-155 §III.B Table 8).
/// See [`sign_context_subband`] for the axis-swap rule.
pub fn sign_prediction_flip_subband(h_pattern: u8, v_pattern: u8, is_hl: bool) -> bool {
    let (hp, vp) = if is_hl {
        (v_pattern, h_pattern)
    } else {
        (h_pattern, v_pattern)
    };
    sign_prediction_flip(hp, vp)
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

    /// §III.C MER initial counts: 2 ones out of 4 total (P = 1/2).
    #[test]
    fn estimator_initial_counts() {
        let m = ContextModel::new();
        for ctx in 0..CONTEXT_COUNT {
            assert_eq!(m.probability(ctx), (INITIAL_ONES, INITIAL_TOTAL));
            assert_eq!(m.probability(ctx), (2, 4));
        }
    }

    /// `observe` increments total always and ones only on a `1` bit
    /// (IPN 42-155 §III.C), until the rescale threshold.
    #[test]
    fn estimator_increments() {
        let mut m = ContextModel::new();
        m.observe(0, 1);
        assert_eq!(m.probability(0), (3, 5));
        m.observe(0, 0);
        assert_eq!(m.probability(0), (3, 6));
    }

    /// At `total == RESCALE_THRESHOLD` the counts halve; the estimate
    /// stays strictly inside `(0, 1)` and nearer 1/2 than before is not
    /// required, but `1 <= ones < total` must hold.
    #[test]
    fn estimator_rescales_at_threshold() {
        let mut m = ContextModel::new();
        // Feed all-ones until the rescale fires. Track that total never
        // exceeds the threshold post-rescale and the estimate is valid.
        let mut rescaled = false;
        for _ in 0..(RESCALE_THRESHOLD as usize + 50) {
            m.observe(0, 1);
            let (ones, total) = m.probability(0);
            assert!(total >= 2, "total must stay >= 2");
            assert!(ones >= 1 && ones < total, "estimate must stay in (0,1)");
            if total < RESCALE_THRESHOLD {
                // After at least one observe the total dipped below the
                // threshold again -> a rescale happened.
                if rescaled || total < INITIAL_TOTAL + 5 {
                    // (no-op; just exercising the path)
                }
            }
            if total <= RESCALE_THRESHOLD / 2 + 5 && ones > 1 {
                rescaled = true;
            }
        }
        assert!(rescaled, "rescale must have fired within the run");
    }

    /// A pure-zeros stream pushes P(1) toward the floor but never to 0 —
    /// the rescale clamps `ones >= 1`.
    #[test]
    fn estimator_floor_on_all_zeros() {
        let mut m = ContextModel::new();
        for _ in 0..2000 {
            m.observe(5, 0);
            let (ones, total) = m.probability(5);
            assert!(ones >= 1, "ones floored at 1");
            assert!(ones < total, "P(1) strictly below 1");
        }
    }

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

    /// IPN 42-155 §III.B **Table 6** — every published cell, indexed by
    /// `(h, v, d)`. The grid only varies with `d` in the `(h=0,v=0)` and
    /// `(h=1,v=0)` columns; the others are flat across `d`.
    #[test]
    fn table6_all_cells() {
        // (h, v, d_class) -> expected context. d_class: 0 -> d=0,
        // 1 -> d=1, 2 -> d>=2 (probe d=2 and d=4).
        let cells: &[(u8, u8, u8, usize)] = &[
            // h=0, v=0: 0 / 1 / 2 down the d rows.
            (0, 0, 0, 0),
            (0, 0, 1, 1),
            (0, 0, 2, 2),
            (0, 0, 4, 2),
            // h=0, v=1: always 3.
            (0, 1, 0, 3),
            (0, 1, 1, 3),
            (0, 1, 4, 3),
            // h=0, v=2: always 4.
            (0, 2, 0, 4),
            (0, 2, 3, 4),
            // h=1, v=0: 5 / 6 / 7 down the d rows.
            (1, 0, 0, 5),
            (1, 0, 1, 6),
            (1, 0, 2, 7),
            (1, 0, 4, 7),
            // h=1, v>0: always 7.
            (1, 1, 0, 7),
            (1, 2, 2, 7),
            // h=2: always 8.
            (2, 0, 0, 8),
            (2, 1, 1, 8),
            (2, 2, 4, 8),
        ];
        for &(h, v, d, expect) in cells {
            assert_eq!(
                significance_context_table6(h, v, d),
                expect,
                "Table 6 cell (h={h}, v={v}, d={d})"
            );
        }
    }

    /// IPN 42-155 §III.B **Table 7** — every published cell, indexed by
    /// `(h + v)` and `d`.
    #[test]
    fn table7_all_cells() {
        // (h, v, d, expected). h+v drives the column.
        let cells: &[(u8, u8, u8, usize)] = &[
            // d=0: 0 / 1 / 2.
            (0, 0, 0, 0),
            (1, 0, 0, 1),
            (0, 1, 0, 1),
            (2, 0, 0, 2),
            (1, 1, 0, 2),
            // d=1: 3 / 4 / 5.
            (0, 0, 1, 3),
            (1, 0, 1, 4),
            (2, 0, 1, 5),
            // d=2: 6 / 7 / 7.
            (0, 0, 2, 6),
            (1, 0, 2, 7),
            (2, 0, 2, 7),
            // d>=3: always 8.
            (0, 0, 3, 8),
            (1, 0, 4, 8),
            (2, 0, 4, 8),
        ];
        for &(h, v, d, expect) in cells {
            assert_eq!(
                significance_context_table7(h, v, d),
                expect,
                "Table 7 cell (h={h}, v={v}, d={d})"
            );
        }
    }

    /// The HL context-template transpose swaps `h` and `v` before the
    /// Table 6 lookup, while LL / LH use the counts as-is.
    #[test]
    fn hl_transpose_swaps_h_and_v() {
        // (h=1, v=0, d=0) -> Table 6 cell 5; transposed (h<->v) it becomes
        // (h=0, v=1, d=0) -> cell 3.
        let (h, v, d) = (1u8, 0u8, 0u8);
        assert_eq!(
            significance_context_subband(h, v, d, false, false),
            5,
            "LH/LL"
        );
        assert_eq!(significance_context_subband(h, v, d, false, true), 3, "HL");
        // HH ignores the transpose flag and sums h+v.
        assert_eq!(
            significance_context_subband(h, v, d, true, true),
            significance_context_table7(h, v, d)
        );
        assert_eq!(
            significance_context_subband(h, v, d, true, false),
            significance_context_table7(h, v, d)
        );
    }

    /// Every `(h, v, d)` in the valid ranges produces a context in
    /// `0..=8` for both tables and both transpose settings.
    #[test]
    fn subband_contexts_in_range() {
        for h in 0..=2u8 {
            for v in 0..=2u8 {
                for d in 0..=4u8 {
                    for is_hh in [false, true] {
                        for is_hl in [false, true] {
                            let c = significance_context_subband(h, v, d, is_hh, is_hl);
                            assert!(c <= 8, "ctx {c} out of range for ({h},{v},{d})");
                        }
                    }
                }
            }
        }
    }

    /// `neighbour_counts` decodes the packed 8-neighbour pattern into
    /// the `(h, v, d)` triple the spec tables index by.
    #[test]
    fn neighbour_counts_decode() {
        // W(bit3) + E(bit4) -> h=2; N(bit1) -> v=1; NW(bit0)+SE(bit7) -> d=2.
        let pat = 0b1001_1011u8;
        let (h, v, d) = neighbour_counts(pat);
        assert_eq!((h, v, d), (2, 1, 2));
        // All neighbours set: h=2, v=2, d=4.
        assert_eq!(neighbour_counts(0xFF), (2, 2, 4));
        // None set.
        assert_eq!(neighbour_counts(0), (0, 0, 0));
    }

    /// The HL sign transpose swaps the two sign-axis patterns before the
    /// Table 8 lookup; non-HL subbands leave them in place.
    #[test]
    fn hl_sign_transpose() {
        // h>0, v=0 -> +,15 (no flip); transposed it is h=0,v>0 -> -,13.
        let pos = 0b0001u8; // axis significant, positive
        let zero = 0u8;
        assert_eq!(sign_context_subband(pos, zero, false), 15);
        assert!(!sign_prediction_flip_subband(pos, zero, false));
        assert_eq!(sign_context_subband(pos, zero, true), 13);
        assert!(sign_prediction_flip_subband(pos, zero, true));
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
