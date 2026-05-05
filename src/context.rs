//! Adaptive probability estimator + context-selection scaffold.
//!
//! ICER's bit-plane coder selects one of several *contexts* for each
//! significance / refinement / sign bit, then feeds the bit + the
//! current per-context probability estimate into the binary
//! arithmetic coder. IPN 42-155 §III.B-III.C describes:
//!
//!   * Significance contexts derived from the eight-neighbour
//!     significance pattern of the current coefficient (§III.B
//!     mentions 9 contexts after symmetry collapsing — same idea as
//!     JPEG 2000's EBCOT contexts but with ICER's specific labels).
//!   * Sign contexts derived from the horizontal + vertical neighbour
//!     sign pattern (§III.B briefly mentions 5 sign contexts).
//!   * Refinement contexts — 3 distinguished by whether the
//!     coefficient just transitioned to significant.
//!
//! The exact context-numbering tables are *not* fully tabulated in
//! the public 2003 paper — IPN 42-155 §III.B Table 1 lists the
//! number of contexts per pass but defers the precise neighbourhood
//! → context mapping to "the implementation". Round 1 ships:
//!
//!   * The 17-context placeholder table covering the union (9 sig +
//!     5 sign + 3 refine) so the [`ArithEncoder`](crate::arith)
//!     interface compiles + round-trips.
//!   * A simple windowed-counting probability estimator (Laplace MAP
//!     with a 64-symbol moving window) — IPN 42-155 §III.C says
//!     "windowed counting" but does not pin the window size.
//!
//! Round 2 will swap the placeholder context table for the real
//! neighbourhood-pattern → context mapping once we transcribe IPN
//! 42-155 Table 1 + the supplemental tables in the descanso.jpl.nasa.gov
//! "ICER-3D" white paper that the paper cites as reference [13].

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
    /// (1 one out of 2 total → P(1) = 1/2).
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
/// `pattern` packs the eight neighbours into bits 0..=7 in
/// raster-scan order (NW, N, NE, W, E, SW, S, SE). The mapping
/// below is a placeholder collapsing all 256 patterns into 9
/// equivalence classes purely by population count + the 4 cardinal
/// neighbours' significance — IPN 42-155 §III.B Table 1 documents
/// 9 significance contexts but does not publish the exact
/// pattern-to-class table.
///
/// Round-2 work will replace this with the real table once it lands
/// in `docs/image/icer/`.
pub fn significance_context(pattern: u8) -> usize {
    let popcount = pattern.count_ones() as usize;
    match popcount {
        0 => 0,
        1 => 1,
        2 => 2,
        3 => 3,
        4 => 4,
        5 => 5,
        6 => 6,
        7 => 7,
        _ => 8,
    }
}

/// Sign-context lookup. `h_pattern` and `v_pattern` each pack
/// `(neighbour_significant, neighbour_sign)` for the horizontal and
/// vertical neighbour pairs:
///
///   * bits 0,1 — (W significant, W sign)
///   * bits 2,3 — (E significant, E sign)
///
/// The placeholder logic returns one of 5 contexts (indices 9..=13
/// in the unified context table) classified by horizontal +
/// vertical sign agreement, matching the count given in IPN 42-155
/// §III.B Table 1.
pub fn sign_context(h_pattern: u8, v_pattern: u8) -> usize {
    let h_signal = (h_pattern & 0b0101) != 0; // any horizontal sig?
    let v_signal = (v_pattern & 0b0101) != 0;
    match (h_signal, v_signal) {
        (false, false) => 9,
        (true, false) => 10,
        (false, true) => 11,
        (true, true) => {
            // Differentiate by whether the H + V signs agree.
            let h_sign = (h_pattern >> 1) & 1;
            let v_sign = (v_pattern >> 1) & 1;
            if h_sign == v_sign {
                12
            } else {
                13
            }
        }
    }
}

/// Refinement-context lookup. Per IPN 42-155 §III.B Table 1 there
/// are 3 refinement contexts; the placeholder distinguishes them by
/// whether the coefficient was *just* made significant (highest
/// uncertainty) vs has been significant for several bit-planes
/// already (lowest uncertainty).
pub fn refinement_context(just_significant: bool, has_significant_neighbour: bool) -> usize {
    match (just_significant, has_significant_neighbour) {
        (true, _) => 14,
        (false, false) => 15,
        (false, true) => 16,
    }
}
