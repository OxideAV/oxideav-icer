//! ICER-3D spectral context modeler — IPN 42-164 §IV.C, Tables 2–6.
//!
//! ICER-3D classifies each bit to be encoded into one of **19**
//! contexts (Table 2: 9 for category-0 bits, 3 for category 1, 2 for
//! category 2, none for category 3 — those are left uncoded — and 5 for
//! sign bits). Unlike 2-D ICER's eight-spatial-neighbour model, the
//! ICER-3D context of a bit is determined from the **two neighbouring
//! coefficients in the spectral dimension and no neighbouring
//! coefficients in the same spatial plane** (§IV.C): the coefficients
//! at the same spatial location in the previous and next spatial plane
//! of the subband, denoted `C⁻` / `C⁺` (their categories) and `S⁻` /
//! `S⁺` (their signs). A neighbour that does not exist because the
//! coefficient sits in the first or last spatial plane of its subband
//! "is treated as being in category 0" (`C = 0`, sign unknown).
//!
//! Categories are the same four-state scheme as 2-D ICER (§IV.C): a
//! coefficient starts in category 0; the first encoded `1` magnitude
//! bit moves it to category 1; each further encoded magnitude bit
//! advances it to 2, then 3, where it stays.
//!
//! Sign bits are not encoded directly (§IV.C): the modeler first
//! *predicts* the sign from `(S⁻, S⁺)` per Table 6 and then encodes an
//! "agreement" bit — the exclusive-or of the sign bit and its predicted
//! value — in one of the five sign contexts.
//!
//! The probability estimator is the same adaptive procedure as 2-D ICER
//! (§IV.C cites the 42-155 §III.C procedure); [`new_model`] builds a
//! [`crate::context::ContextModel`] sized for the 19-context layout.

use crate::context::ContextModel;

/// Total number of contexts in the ICER-3D layout (IPN 42-164 §IV.C
/// Table 2): 9 category-0 + 3 category-1 + 2 category-2 + 5 sign.
pub const CONTEXT_COUNT_3D: usize = 19;

/// First category-1 context (`1-a`). Layout: 0..=8 category 0
/// (`0-a`..`0-i`), 9..=11 category 1 (`1-a`..`1-c`), 12..=13 category 2
/// (`2-a`, `2-b`), 14..=18 sign (`S-a`..`S-e`).
pub const CTX_CAT1_BASE: usize = 9;
/// First category-2 context (`2-a`).
pub const CTX_CAT2_BASE: usize = 12;
/// First sign context (`S-a`).
pub const CTX_SIGN_BASE: usize = 14;

/// A fresh 19-context adaptive probability model (the §IV.C estimation
/// procedure is 42-155 §III.C's, shared with 2-D ICER).
pub fn new_model() -> ContextModel {
    ContextModel::with_contexts(CONTEXT_COUNT_3D)
}

/// Clamp a spectral-neighbour category to the `{0, 1, >=2}` classes the
/// §IV.C tables index by.
#[inline]
fn cls(c: u8) -> usize {
    (c as usize).min(2)
}

/// Context for a bit of a coefficient in **category 0** — IPN 42-164
/// §IV.C **Table 3**, indexed by the categories of the spectral
/// neighbours (`c_minus` = previous spatial plane, `c_plus` = next).
///
/// Table 3 assigns letters `0-a`..`0-i` column-major (`0-a`/`0-b`/`0-c`
/// down the `C⁺ = 0` column, `0-d`..`0-f` down `C⁺ = 1`, `0-g`..`0-i`
/// down `C⁺ >= 2`), so the context number is `3 * min(C⁺, 2) +
/// min(C⁻, 2)`.
pub fn category0_context(c_minus: u8, c_plus: u8) -> usize {
    3 * cls(c_plus) + cls(c_minus)
}

/// Context for a bit of a coefficient in **category 1** — §IV.C
/// **Table 4**: `1-a` whenever `C⁻ < 2`; for `C⁻ >= 2` the context is
/// `1-a` / `1-b` / `1-c` as `C⁺` is `0` / `1` / `>= 2`.
pub fn category1_context(c_minus: u8, c_plus: u8) -> usize {
    if c_minus < 2 {
        CTX_CAT1_BASE
    } else {
        CTX_CAT1_BASE + cls(c_plus)
    }
}

/// Context for a bit of a coefficient in **category 2** — §IV.C
/// **Table 5**: `2-b` when both spectral neighbours are in category
/// `>= 2`, otherwise `2-a`.
pub fn category2_context(c_minus: u8, c_plus: u8) -> usize {
    if c_minus >= 2 && c_plus >= 2 {
        CTX_CAT2_BASE + 1
    } else {
        CTX_CAT2_BASE
    }
}

/// Sign of a spectral neighbour as the §IV.C tables see it: positive,
/// negative, or not yet known (the neighbour has no encoded `1` bit, or
/// does not exist).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NeighbourSign {
    /// Sign not yet known (`0` in Table 6).
    Unknown,
    /// Known positive (`+`).
    Positive,
    /// Known negative (`-`).
    Negative,
}

/// Sign prediction + context for a sign bit — IPN 42-164 §IV.C
/// **Table 6**, indexed by the signs of the spectral neighbours.
/// Returns `(predicted_negative, context)`:
///
/// ```text
///                S⁺ = +      S⁺ = 0      S⁺ = -
///     S⁻ = +    (+, S-e)    (+, S-d)    (+, S-b)
///     S⁻ = 0    (+, S-c)    (+, S-a)    (-, S-c)
///     S⁻ = -    (-, S-b)    (-, S-d)    (-, S-e)
/// ```
///
/// The encoder codes the agreement bit `sign_bit XOR predicted` (with
/// `1` = negative), so the model always sees a bit whose statistics
/// reflect spectral sign correlation (§IV.C: statistics "automatically
/// adapt to exploit both positive and negative correlations between
/// adjacent spatial planes").
pub fn sign_prediction_and_context(s_minus: NeighbourSign, s_plus: NeighbourSign) -> (bool, usize) {
    use NeighbourSign::{Negative, Positive, Unknown};
    let (neg, letter) = match (s_minus, s_plus) {
        (Positive, Positive) => (false, 4), // S-e
        (Positive, Unknown) => (false, 3),  // S-d
        (Positive, Negative) => (false, 1), // S-b
        (Unknown, Positive) => (false, 2),  // S-c
        (Unknown, Unknown) => (false, 0),   // S-a
        (Unknown, Negative) => (true, 2),   // S-c
        (Negative, Positive) => (true, 1),  // S-b
        (Negative, Unknown) => (true, 3),   // S-d
        (Negative, Negative) => (true, 4),  // S-e
    };
    (neg, CTX_SIGN_BASE + letter)
}

#[cfg(test)]
mod tests {
    use super::*;
    use NeighbourSign::{Negative, Positive, Unknown};

    #[test]
    fn table2_context_budget() {
        // Table 2: 9 + 3 + 2 + 5 = 19 contexts; category 3 has none.
        assert_eq!(CONTEXT_COUNT_3D, 19);
        assert_eq!(CTX_CAT1_BASE, 9);
        assert_eq!(CTX_CAT2_BASE, 12);
        assert_eq!(CTX_SIGN_BASE, 14);
    }

    #[test]
    fn table3_all_nine_cells() {
        // Table 3 grid: rows C⁻ ∈ {0, 1, >=2}, columns C⁺ ∈ {0, 1, >=2}.
        //          C⁺=0   C⁺=1   C⁺>=2
        //  C⁻=0    0-a    0-d    0-g
        //  C⁻=1    0-b    0-e    0-h
        //  C⁻>=2   0-c    0-f    0-i
        let a = |letter: usize| letter; // 0-a = 0 ... 0-i = 8
        assert_eq!(category0_context(0, 0), a(0));
        assert_eq!(category0_context(1, 0), a(1));
        assert_eq!(category0_context(2, 0), a(2));
        assert_eq!(category0_context(0, 1), a(3));
        assert_eq!(category0_context(1, 1), a(4));
        assert_eq!(category0_context(2, 1), a(5));
        assert_eq!(category0_context(0, 2), a(6));
        assert_eq!(category0_context(1, 2), a(7));
        assert_eq!(category0_context(2, 2), a(8));
        // Category 3 neighbours classify as ">= 2".
        assert_eq!(category0_context(3, 3), a(8));
    }

    #[test]
    fn table4_all_cells() {
        // Table 4: C⁻ < 2 -> 1-a for every C⁺; C⁻ >= 2 -> 1-a / 1-b /
        // 1-c as C⁺ = 0 / 1 / >= 2.
        for cp in 0..=3u8 {
            assert_eq!(category1_context(0, cp), CTX_CAT1_BASE);
            assert_eq!(category1_context(1, cp), CTX_CAT1_BASE);
        }
        assert_eq!(category1_context(2, 0), CTX_CAT1_BASE);
        assert_eq!(category1_context(2, 1), CTX_CAT1_BASE + 1);
        assert_eq!(category1_context(2, 2), CTX_CAT1_BASE + 2);
        assert_eq!(category1_context(3, 3), CTX_CAT1_BASE + 2);
    }

    #[test]
    fn table5_all_cells() {
        // Table 5: 2-b only when both neighbours are >= 2.
        assert_eq!(category2_context(0, 0), CTX_CAT2_BASE);
        assert_eq!(category2_context(1, 2), CTX_CAT2_BASE);
        assert_eq!(category2_context(2, 1), CTX_CAT2_BASE);
        assert_eq!(category2_context(2, 2), CTX_CAT2_BASE + 1);
        assert_eq!(category2_context(3, 2), CTX_CAT2_BASE + 1);
    }

    #[test]
    fn table6_all_nine_cells() {
        // Table 6 predictions + contexts, verbatim.
        let s = |letter: usize| CTX_SIGN_BASE + letter;
        assert_eq!(
            sign_prediction_and_context(Positive, Positive),
            (false, s(4))
        );
        assert_eq!(
            sign_prediction_and_context(Positive, Unknown),
            (false, s(3))
        );
        assert_eq!(
            sign_prediction_and_context(Positive, Negative),
            (false, s(1))
        );
        assert_eq!(
            sign_prediction_and_context(Unknown, Positive),
            (false, s(2))
        );
        assert_eq!(sign_prediction_and_context(Unknown, Unknown), (false, s(0)));
        assert_eq!(sign_prediction_and_context(Unknown, Negative), (true, s(2)));
        assert_eq!(
            sign_prediction_and_context(Negative, Positive),
            (true, s(1))
        );
        assert_eq!(sign_prediction_and_context(Negative, Unknown), (true, s(3)));
        assert_eq!(
            sign_prediction_and_context(Negative, Negative),
            (true, s(4))
        );
    }

    #[test]
    fn table6_symmetry() {
        // Table 6 is antisymmetric under global sign flip: flipping
        // both neighbour signs flips the prediction and keeps the
        // context — the property that lets one set of agreement
        // statistics serve both polarities.
        let flip = |s: NeighbourSign| match s {
            Positive => Negative,
            Negative => Positive,
            Unknown => Unknown,
        };
        for sm in [Positive, Unknown, Negative] {
            for sp in [Positive, Unknown, Negative] {
                let (neg, ctx) = sign_prediction_and_context(sm, sp);
                let (neg_f, ctx_f) = sign_prediction_and_context(flip(sm), flip(sp));
                assert_eq!(ctx, ctx_f, "context must be polarity-invariant");
                if (sm, sp) != (Unknown, Unknown) {
                    assert_ne!(neg, neg_f, "prediction must flip with polarity");
                }
            }
        }
    }

    #[test]
    fn model_is_19_contexts_with_mer_prior() {
        let m = new_model();
        for ctx in 0..CONTEXT_COUNT_3D {
            assert_eq!(m.probability(ctx), (2, 4), "ctx {ctx} prior");
        }
    }

    #[test]
    fn context_ranges_are_disjoint() {
        // §IV.C: "the contexts for the four types of bits that are
        // compressed ... do not overlap".
        for cm in 0..=3u8 {
            for cp in 0..=3u8 {
                assert!(category0_context(cm, cp) < CTX_CAT1_BASE);
                let c1 = category1_context(cm, cp);
                assert!((CTX_CAT1_BASE..CTX_CAT2_BASE).contains(&c1));
                let c2 = category2_context(cm, cp);
                assert!((CTX_CAT2_BASE..CTX_SIGN_BASE).contains(&c2));
            }
        }
        for sm in [Positive, Unknown, Negative] {
            for sp in [Positive, Unknown, Negative] {
                let (_, ctx) = sign_prediction_and_context(sm, sp);
                assert!((CTX_SIGN_BASE..CONTEXT_COUNT_3D).contains(&ctx));
            }
        }
    }
}
