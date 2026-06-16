//! Subband quantization priority factors -- IPN 42-155 §III.A
//! "Subband Quantization and Priority Factors".
//!
//! ICER compresses subband bit planes most-significant first, but it
//! does *not* simply walk one subband to completion before starting the
//! next. Bit planes from different subbands are interleaved: after each
//! subband bit plane is finished, ICER chooses the next subband bit
//! plane to compress according to a fixed **priority** scheme (§III.A).
//! The intent is to spend the next compressed bit on whichever subband
//! bit plane yields the largest reduction in reconstructed-image
//! distortion per bit, so a truncated stream is as good as it can be at
//! every cut point.
//!
//! # Priority weights (§III.A, Fig. 7)
//!
//! Because the wavelet transforms ICER uses are not unitary, mean-square
//! error measured in the transform domain is not equal to MSE of the
//! reconstructed image. §III.A scales the transform to an approximately
//! unitary form (`l~[n] = sqrt(2) l[n]`, `h~[n] = (1/sqrt(2)) h[n]`) and
//! reads off, from the resulting per-pixel weights, the relative effect
//! a subband's pixels have on reconstructed-image RMS distortion. Those
//! per-subband weights are the ones plotted in Fig. 7.
//!
//! For a `D`-stage dyadic decomposition the subbands and their Fig. 7
//! weights follow a clean closed form. Writing the decomposition
//! *level* `j` of a subband as `1` for the first (coarsest-filtered,
//! largest, outermost) stage up to `D` for the last (innermost,
//! smallest) stage:
//!
//! ```text
//!     w(HH_j)        = 2^(j-2)
//!     w(HL_j) = w(LH_j) = 2^(j-1)
//!     w(LL_D)        = 2^D          (LL exists only at the deepest level)
//! ```
//!
//! For `D = 3` this reproduces Fig. 7 exactly:
//!
//! | level | LL | HL | LH | HH  |
//! |-------|---:|---:|---:|----:|
//! | 1     |  - |  1 |  1 | 1/2 |
//! | 2     |  - |  2 |  2 |  1  |
//! | 3     |  8 |  4 |  4 |  2  |
//!
//! The two worked examples §III.A gives both fall out of this form:
//!
//! * "a pixel in the LL subband has a factor of 16 higher priority
//!   weight than a pixel in the level-1 HH subband" (D = 3):
//!   `w(LL_3) / w(HH_1) = 8 / (1/2) = 16`.
//! * "the `i`th least-significant bit plane of the level-1 HH subband
//!   has priority equal to that of the `(i+4)`th least-significant bit
//!   plane of the LL subband": each additional bit plane reduces a
//!   subband's RMS distortion by roughly a factor of 2, so a bit plane's
//!   priority is its subband weight times `2^(-k)` where `k` counts bit
//!   planes down from the most significant. The four-bit-plane offset is
//!   exactly `log2(16)`.
//!
//! # Encode order (§III.A)
//!
//! ICER encodes subband bit planes in order of decreasing priority.
//! §III.A pins the tie-breaks precisely:
//!
//! 1. higher priority weight first;
//! 2. when weights tie, the subband with the **higher decomposition
//!    level** first;
//! 3. when the level also ties, by subband **type** in the order
//!    `LL, HL, LH, HH`.
//!
//! Because every additional bit plane halves the priority, the
//! priorities live on a `log2` scale; this module represents a bit
//! plane's priority by an integer `log2`-priority so the ordering is
//! exact (no floating-point comparison). The most-significant magnitude
//! bit plane of a subband is `bp_from_msb = 0`.
//!
//! This module is the clean-room §III.A model. It produces the cross-
//! subband interleaving order; the bit-plane coder (`crate::bitplane`)
//! and the packet emitter consume that order. It deliberately has no
//! dependency on the entropy coder or the wire framing -- it is pure
//! arithmetic over the subband geometry.

/// One of the four subband types produced by a 2-D wavelet stage. The
/// two letters give horizontal then vertical filtering: `L` = low-pass,
/// `H` = high-pass (IPN 42-155 §II.B).
///
/// `LL` only ever appears at the deepest decomposition level (the
/// pyramidal decomposition replaces the previous stage's LL with four
/// new subbands at each further stage -- §II.B).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum SubbandType {
    /// Horizontal low-pass, vertical low-pass. The coarse approximation;
    /// present only at the deepest level.
    Ll,
    /// Horizontal high-pass, vertical low-pass.
    Hl,
    /// Horizontal low-pass, vertical high-pass.
    Lh,
    /// Horizontal high-pass, vertical high-pass.
    Hh,
}

impl SubbandType {
    /// §III.A tie-break rank when both priority weight and decomposition
    /// level coincide: `LL` before `HL` before `LH` before `HH`. Lower
    /// rank encodes first.
    pub fn order_rank(self) -> u8 {
        match self {
            SubbandType::Ll => 0,
            SubbandType::Hl => 1,
            SubbandType::Lh => 2,
            SubbandType::Hh => 3,
        }
    }
}

/// One subband of a `D`-stage decomposition: its type plus the
/// decomposition level (`1` = first/outermost stage, `D` = deepest).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Subband {
    /// Subband type (`LL`/`HL`/`LH`/`HH`).
    pub kind: SubbandType,
    /// Decomposition level, `1..=D`. `LL` only occurs at level `D`.
    pub level: u8,
}

impl Subband {
    /// The Fig. 7 (§III.A) priority weight, expressed as a base-2
    /// logarithm so it stays an exact integer.
    ///
    /// From the closed form in the module docs, with level `j`:
    ///
    /// ```text
    ///     log2 w(HH_j)         = j - 2
    ///     log2 w(HL_j)/w(LH_j) = j - 1
    ///     log2 w(LL_D)         = D
    /// ```
    ///
    /// (Note `w(HH_1) = 1/2` gives a negative `log2` of `-1`, hence the
    /// signed return.)
    pub fn weight_log2(self, decomposition_levels: u8) -> i32 {
        let j = self.level as i32;
        match self.kind {
            SubbandType::Ll => decomposition_levels as i32,
            SubbandType::Hl | SubbandType::Lh => j - 1,
            SubbandType::Hh => j - 2,
        }
    }
}

/// Enumerate every subband of a `D`-stage dyadic decomposition.
///
/// §II.B: each stage replaces the previous stage's LL with four new
/// subbands, so after `D` stages there are `3*D + 1` subbands -- three
/// detail subbands (`HL`, `LH`, `HH`) at every level `1..=D`, plus the
/// single `LL` at level `D`.
///
/// `decomposition_levels` is clamped to `1..=6` to match the segment
/// header's level field (IPN 42-155 §III.A; the same `1..=6` range the
/// encoder uses).
pub fn subbands(decomposition_levels: u8) -> Vec<Subband> {
    let d = decomposition_levels.clamp(1, 6);
    let mut out = Vec::with_capacity(3 * d as usize + 1);
    // LL exists only at the deepest level.
    out.push(Subband {
        kind: SubbandType::Ll,
        level: d,
    });
    for level in 1..=d {
        for kind in [SubbandType::Hl, SubbandType::Lh, SubbandType::Hh] {
            out.push(Subband { kind, level });
        }
    }
    out
}

/// A single subband bit plane scheduled for compression, with the
/// `log2`-priority used to order it against bit planes from other
/// subbands.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SubbandBitPlane {
    /// Which subband this bit plane belongs to.
    pub subband: Subband,
    /// Bit-plane index counted down from the most-significant magnitude
    /// bit plane of this subband (`0` = MSB plane).
    pub bp_from_msb: u32,
    /// The bit plane's `log2`-priority: `weight_log2(subband) -
    /// bp_from_msb`. Higher is more urgent. Each step down a bit plane
    /// halves the priority (subtracts 1 in `log2` space) per §III.A.
    pub priority_log2: i32,
}

/// Build the cross-subband encode order for a `D`-stage decomposition
/// where each subband contributes `bit_planes` magnitude bit planes,
/// most-significant first.
///
/// The returned vector lists every `(subband, bit-plane)` pair in the
/// exact §III.A priority order:
///
/// 1. higher `priority_log2` first;
/// 2. ties broken by higher decomposition level;
/// 3. then by subband type order `LL, HL, LH, HH`.
///
/// This is the order in which ICER interleaves subband bit planes into
/// the progressive stream. Truncating the returned order at any prefix
/// yields the §III.A-optimal subset of bit planes for that cut.
///
/// `bit_planes` is the per-subband magnitude bit-plane count (the
/// segment header's `q`); `decomposition_levels` is clamped to `1..=6`.
pub fn encode_order(decomposition_levels: u8, bit_planes: u32) -> Vec<SubbandBitPlane> {
    let bands = subbands(decomposition_levels);
    let mut plan: Vec<SubbandBitPlane> = Vec::with_capacity(bands.len() * bit_planes as usize);
    for &subband in &bands {
        let w = subband.weight_log2(decomposition_levels);
        for bp in 0..bit_planes {
            plan.push(SubbandBitPlane {
                subband,
                bp_from_msb: bp,
                priority_log2: w - bp as i32,
            });
        }
    }
    // §III.A ordering: priority descending, then level descending, then
    // subband type order LL < HL < LH < HH. `sort_by` is stable, so an
    // explicit total comparator is used for every tie level.
    plan.sort_by(|a, b| {
        b.priority_log2
            .cmp(&a.priority_log2)
            .then(b.subband.level.cmp(&a.subband.level))
            .then(
                a.subband
                    .kind
                    .order_rank()
                    .cmp(&b.subband.kind.order_rank()),
            )
            .then(a.bp_from_msb.cmp(&b.bp_from_msb))
    });
    plan
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn subband_count_is_3d_plus_1() {
        for d in 1..=6u8 {
            assert_eq!(subbands(d).len(), 3 * d as usize + 1, "D={d} subband count");
        }
    }

    #[test]
    fn ll_only_at_deepest_level() {
        let bands = subbands(3);
        let ll: Vec<_> = bands.iter().filter(|s| s.kind == SubbandType::Ll).collect();
        assert_eq!(ll.len(), 1, "exactly one LL subband");
        assert_eq!(ll[0].level, 3, "LL at deepest level");
    }

    /// IPN 42-155 §III.A Fig. 7 weights for a 3-stage decomposition,
    /// expressed as `log2`: level-1 HH = 1/2 -> -1, level-1 HL/LH = 1 ->
    /// 0, level-2 HH = 1 -> 0, level-2 HL/LH = 2 -> 1, level-3 HH = 2 ->
    /// 1, level-3 HL/LH = 4 -> 2, level-3 LL = 8 -> 3.
    #[test]
    fn fig7_weights_d3() {
        let d = 3;
        let cases = [
            (SubbandType::Hh, 1u8, -1i32),
            (SubbandType::Hl, 1, 0),
            (SubbandType::Lh, 1, 0),
            (SubbandType::Hh, 2, 0),
            (SubbandType::Hl, 2, 1),
            (SubbandType::Lh, 2, 1),
            (SubbandType::Hh, 3, 1),
            (SubbandType::Hl, 3, 2),
            (SubbandType::Lh, 3, 2),
            (SubbandType::Ll, 3, 3),
        ];
        for (kind, level, expect) in cases {
            let s = Subband { kind, level };
            assert_eq!(
                s.weight_log2(d),
                expect,
                "Fig.7 weight log2 for {kind:?} level {level}"
            );
        }
    }

    /// §III.A worked example: a pixel in the LL subband has a factor of
    /// 16 higher priority weight than a pixel in the level-1 HH subband
    /// (3-stage decomposition). 16 = 2^4, so the log2 difference is 4.
    #[test]
    fn ll_is_16x_level1_hh() {
        let d = 3;
        let ll = Subband {
            kind: SubbandType::Ll,
            level: 3,
        };
        let hh1 = Subband {
            kind: SubbandType::Hh,
            level: 1,
        };
        assert_eq!(ll.weight_log2(d) - hh1.weight_log2(d), 4);
    }

    /// §III.A worked example: the `i`th LSB plane of the level-1 HH
    /// subband has priority equal to the `(i+4)`th LSB plane of the LL
    /// subband. Equivalently, an LL bit plane four positions *more*
    /// significant matches a level-1 HH bit plane in priority.
    #[test]
    fn ll_bitplane_offset_matches_hh1() {
        let order = encode_order(3, 12);
        // Find an LL plane and a level-1 HH plane with equal priority.
        let find = |kind: SubbandType, level: u8, bp: u32| {
            order
                .iter()
                .find(|p| p.subband.kind == kind && p.subband.level == level && p.bp_from_msb == bp)
                .copied()
                .unwrap()
        };
        // LL plane 4 steps below its MSB vs level-1 HH MSB plane: equal.
        let ll = find(SubbandType::Ll, 3, 4);
        let hh1 = find(SubbandType::Hh, 1, 0);
        assert_eq!(ll.priority_log2, hh1.priority_log2);
    }

    #[test]
    fn order_is_priority_descending() {
        let order = encode_order(3, 8);
        for w in order.windows(2) {
            assert!(
                w[0].priority_log2 >= w[1].priority_log2,
                "priority must be non-increasing along the encode order"
            );
        }
    }

    /// The very first thing encoded is the LL subband's most-significant
    /// bit plane -- it has the strictly highest priority for any D >= 1.
    #[test]
    fn ll_msb_is_first() {
        for d in 1..=6u8 {
            let order = encode_order(d, 6);
            assert_eq!(order[0].subband.kind, SubbandType::Ll);
            assert_eq!(order[0].subband.level, d);
            assert_eq!(order[0].bp_from_msb, 0);
        }
    }

    /// §III.A tie-break 2 + 3: among equal-priority bit planes, the
    /// higher decomposition level wins, then type order LL,HL,LH,HH.
    #[test]
    fn tie_breaks_level_then_type() {
        // D=3: level-2 HL/LH (log2 weight 1) and level-3 HH (log2
        // weight 1) all share priority 1 at bp 0. Order among them:
        // level 3 before level 2; within a level, HL before LH.
        let order = encode_order(3, 1); // one bit plane each -> bp always 0
        let prio1: Vec<_> = order
            .iter()
            .filter(|p| p.priority_log2 == 1)
            .map(|p| (p.subband.level, p.subband.kind))
            .collect();
        assert_eq!(
            prio1,
            vec![
                (3, SubbandType::Hh),
                (2, SubbandType::Hl),
                (2, SubbandType::Lh),
            ]
        );
    }

    #[test]
    fn total_plan_size() {
        let d = 4u8;
        let q = 7u32;
        let order = encode_order(d, q);
        assert_eq!(order.len(), (3 * d as usize + 1) * q as usize);
    }

    #[test]
    fn order_is_a_permutation_of_all_pairs() {
        let d = 3u8;
        let q = 5u32;
        let order = encode_order(d, q);
        let mut seen = std::collections::HashSet::new();
        for p in &order {
            assert!(
                seen.insert((p.subband.kind, p.subband.level, p.bp_from_msb)),
                "duplicate (subband, bit-plane) in encode order"
            );
        }
        assert_eq!(seen.len(), (3 * d as usize + 1) * q as usize);
    }

    #[test]
    fn levels_clamped_to_1_6() {
        assert_eq!(subbands(0).len(), subbands(1).len());
        assert_eq!(subbands(9).len(), subbands(6).len());
    }
}
