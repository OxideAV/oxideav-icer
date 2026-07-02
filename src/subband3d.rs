//! ICER-3D subband geometry, bit-plane priorities, and index assignment
//! — IPN 42-164 §IV.A and the Appendix ("Subband Index Assignment").
//!
//! The §III.A decomposition (see [`crate::wavelet3d`]) produces, for a
//! full `D`-stage run, `3 * (2 + 3 + ... + (D+1)) + (D+1)` subbands: a
//! level-`j` spatial detail subband (`HL`/`LH`/`HH`) carries `j` levels
//! of spectral decomposition (spectral parts `Hλ_1 .. Hλ_j` plus
//! `Lλ_j`), and the deepest spatial low-pass `LL_D` carries all `D`
//! spectral levels. For `D = 3` that is the 31-subband structure of the
//! 42-164 Fig. 3 illustration.
//!
//! # Priority (§IV.A)
//!
//! Each subband is characterised by the number of high-pass (`H`) and
//! low-pass (`L`) one-dimensional filtering operations used to form it
//! (at most one high-pass per dimension, so `H <= 3`). The §IV.A weight
//! of bit plane `b` (LSB = 0) is `w = 2^b * (sqrt 2)^(L - H)` (eq (1)),
//! and the integer priority actually used to order bit planes is eq (2):
//!
//! ```text
//!     p = 3 + log_sqrt2(w) = 2b + L - H + 3
//! ```
//!
//! Subband bit planes are compressed in order of decreasing priority;
//! ties (necessarily from different subbands) are broken in order of
//! *decreasing subband index* (§IV: "Bit planes having the same priority
//! value ... are compressed in order of decreasing subband index").
//!
//! # Index assignment (Appendix)
//!
//! Indices start at 0 and are assigned by sorting with the published
//! rules: (1) larger `L - H` -> higher index; (2) same `L - H`: fewer
//! coefficients (larger `L + H`) -> higher index; (3) then low-pass in
//! the vertical direction -> higher index; (4) then low-pass in the
//! horizontal direction -> higher index. The Appendix notes the scheme
//! is "somewhat ad hoc"; for decompositions deeper than 3 stages the
//! four rules can leave a tie between subbands that differ only in the
//! *split* between spectral and spatial filtering (e.g. a level-3 `HH`
//! with 3 spectral levels vs. a level-4 `HH` with 1), which the paper's
//! `D = 3` illustration never exercises. This implementation totalises
//! the order with a documented rule (5): the subband with the larger
//! spectral decomposition level gets the higher index, and a spectrally
//! low-pass subband outranks a spectrally high-pass one at the same
//! level. Rules (1)-(4) are applied exactly as published, so every
//! `D <= 3` ordering is fully spec-determined.

use crate::priority::SubbandType;
use crate::wavelet3d::{spatial_stage_count, spectral_stage_count};

/// One subband of the §III.A ICER-3D decomposition.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Subband3d {
    /// Spatial subband type. `Ll` is the deepest spatial low-pass.
    pub spatial: SubbandType,
    /// Spatial decomposition level `1..=ts` for detail subbands; equals
    /// the spatial stage count `ts` for `Ll` (0 when no spatial stage
    /// ran at all, i.e. the whole plane is spatially low-pass).
    pub spatial_level: u8,
    /// `true` for a spectrally high-pass part `Hλ_m`, `false` for the
    /// spectrally low-pass part `Lλ_n`.
    pub spectral_high: bool,
    /// `m` for `Hλ_m` (`1..=n`); `n` for `Lλ_n` (0 when no spectral
    /// stage applies at this subband's spatial positions).
    pub spectral_level: u8,
}

impl Subband3d {
    /// Number of one-dimensional high-pass (`H`) and low-pass (`L`)
    /// filtering operations used to form this subband (§IV.A). A
    /// level-`j` spatial subband is filtered `j` times along each
    /// spatial axis; the spectral part `Hλ_m` adds `m` spectral
    /// operations (one high), `Lλ_n` adds `n` (all low).
    pub fn filter_ops(&self) -> (u8, u8) {
        let j = self.spatial_level;
        let h_spatial = match self.spatial {
            SubbandType::Ll => 0,
            SubbandType::Hl | SubbandType::Lh => 1,
            SubbandType::Hh => 2,
        };
        let (spec_ops, h_spec) = if self.spectral_high {
            (self.spectral_level, 1)
        } else {
            (self.spectral_level, 0)
        };
        let h = h_spatial + h_spec;
        let l = (2 * j - h_spatial) + (spec_ops - h_spec);
        (h, l)
    }

    /// §IV.A eq (2) integer priority of bit plane `b` (indexed from the
    /// least-significant bit, `b = 0`): `p = 2b + L - H + 3`. Always
    /// `>= 0` since `H <= 3` and `L >= 0`.
    pub fn priority(&self, b: u8) -> u16 {
        let (h, l) = self.filter_ops();
        (2 * b as u16 + 3 + l as u16) - h as u16
    }

    /// Spatial lattice of this subband in the Mallat-interleaved layout:
    /// `(x_offset, y_offset, stride)` — members sit at
    /// `x ≡ x_offset (mod stride)`, `y ≡ y_offset (mod stride)`.
    pub fn spatial_lattice(&self) -> (usize, usize, usize) {
        let j = self.spatial_level as usize;
        if j == 0 {
            return (0, 0, 1);
        }
        let stride = 1usize << j;
        let half = stride >> 1;
        match self.spatial {
            SubbandType::Ll => (0, 0, stride),
            SubbandType::Hl => (half, 0, stride),
            SubbandType::Lh => (0, half, stride),
            SubbandType::Hh => (half, half, stride),
        }
    }

    /// Spectral lattice of this subband: `(λ_offset, stride)`.
    pub fn spectral_lattice(&self) -> (usize, usize) {
        let m = self.spectral_level as usize;
        if self.spectral_high {
            (1usize << (m - 1), 1usize << m)
        } else {
            (0, 1usize << m)
        }
    }
}

/// How many stages of the §III.A decomposition apply per dimension for
/// a `width x height x bands` cube at a requested depth of `levels`
/// (see [`crate::wavelet3d`] for the `N >= 3` lattice rule).
pub fn stage_counts(width: usize, height: usize, bands: usize, levels: u8) -> (u8, u8) {
    (
        spatial_stage_count(width, height, levels),
        spectral_stage_count(bands, levels),
    )
}

/// Number of spectral decomposition levels applied at the spatial
/// positions of a given spatial subband (§III.A): a level-`j` detail
/// subband receives `min(j, tl)` spectral levels; the deepest spatial
/// low-pass receives all `tl`.
fn spectral_depth_for(spatial: SubbandType, spatial_level: u8, tl: u8) -> u8 {
    match spatial {
        SubbandType::Ll => tl,
        _ => spatial_level.min(tl),
    }
}

/// Enumerate every subband of a decomposition with `ts` spatial and
/// `tl` spectral stages, sorted ascending by the Appendix subband
/// index — i.e. `result[i]` is the subband with index `i`.
pub fn enumerate_subbands(ts: u8, tl: u8) -> Vec<Subband3d> {
    let mut out = Vec::new();
    let mut spatials: Vec<(SubbandType, u8)> = vec![(SubbandType::Ll, ts)];
    for j in 1..=ts {
        for kind in [SubbandType::Hl, SubbandType::Lh, SubbandType::Hh] {
            spatials.push((kind, j));
        }
    }
    for (spatial, spatial_level) in spatials {
        let n = spectral_depth_for(spatial, spatial_level, tl);
        out.push(Subband3d {
            spatial,
            spatial_level,
            spectral_high: false,
            spectral_level: n,
        });
        for m in 1..=n {
            out.push(Subband3d {
                spatial,
                spatial_level,
                spectral_high: true,
                spectral_level: m,
            });
        }
    }
    out.sort_by_key(index_sort_key);
    out
}

/// Sort key realising the Appendix ordering rules (ascending = index
/// ascending): (1) `L - H`; (2) `L + H` (fewer coefficients higher);
/// (3) vertically low-pass higher; (4) horizontally low-pass higher;
/// (5) implementation tie-break, see the module docs.
fn index_sort_key(sb: &Subband3d) -> (i16, u8, bool, bool, u8, bool) {
    let (h, l) = sb.filter_ops();
    let vert_low = matches!(sb.spatial, SubbandType::Ll | SubbandType::Hl);
    let horiz_low = matches!(sb.spatial, SubbandType::Ll | SubbandType::Lh);
    (
        l as i16 - h as i16,
        l + h,
        vert_low,
        horiz_low,
        sb.spectral_level,
        !sb.spectral_high,
    )
}

/// Classify the cube position `(x, y, λ)` of a decomposition with `ts`
/// spatial and `tl` spectral stages into its subband.
///
/// The spatial classification walks the interleaved lattice exactly like
/// the 2-D [`crate::priority::classify_position`]; the spectral part is
/// determined by λ's dyadic residue, capped at the spectral depth that
/// actually applies at this spatial position (§III.A: a level-`j`
/// spatial subband receives `j` spectral levels).
pub fn classify_cube_position(x: usize, y: usize, lambda: usize, ts: u8, tl: u8) -> Subband3d {
    let (spatial, spatial_level) = if ts == 0 {
        (SubbandType::Ll, 0)
    } else {
        crate::priority::classify_position(x, y, ts)
    };
    let n = spectral_depth_for(spatial, spatial_level, tl);
    let (spectral_high, spectral_level) = if n == 0 || lambda % (1usize << n) == 0 {
        (false, n)
    } else {
        let m = lambda.trailing_zeros() as u8 + 1;
        debug_assert!(m <= n);
        (true, m)
    };
    Subband3d {
        spatial,
        spatial_level,
        spectral_high,
        spectral_level,
    }
}

/// One subband bit plane in the §IV.A compression schedule.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CubeBitPlane {
    /// Index of the subband in the [`enumerate_subbands`] order (= the
    /// Appendix subband index).
    pub subband_index: usize,
    /// Bit plane, indexed from the least-significant bit (`b = 0`).
    pub b: u8,
    /// §IV.A eq (2) priority `2b + L - H + 3`.
    pub priority: u16,
}

/// Build the full §IV.A compression schedule for `q` magnitude bit
/// planes over the given subband list: every `(subband, bit plane)`
/// pair, sorted by decreasing priority, ties broken by decreasing
/// subband index (§IV).
pub fn cube_schedule(subbands: &[Subband3d], q: u8) -> Vec<CubeBitPlane> {
    let mut plan = Vec::with_capacity(subbands.len() * q as usize);
    for (idx, sb) in subbands.iter().enumerate() {
        for b in 0..q {
            plan.push(CubeBitPlane {
                subband_index: idx,
                b,
                priority: sb.priority(b),
            });
        }
    }
    plan.sort_by(|a, b| {
        b.priority
            .cmp(&a.priority)
            .then(b.subband_index.cmp(&a.subband_index))
    });
    plan
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn d3_has_31_subbands() {
        // Fig. 3 of IPN 42-164 labels the three-stage decomposition with
        // subband indices 0..=30.
        let sbs = enumerate_subbands(3, 3);
        assert_eq!(sbs.len(), 31);
    }

    #[test]
    fn subband_counts_follow_level_rule() {
        // A level-j spatial detail subband carries j+1 spectral parts;
        // LL_D carries D+1 (§III.A alternate description).
        for d in 1..=4u8 {
            let sbs = enumerate_subbands(d, d);
            let expect: usize =
                (1..=d).map(|j| 3 * (j as usize + 1)).sum::<usize>() + d as usize + 1;
            assert_eq!(sbs.len(), expect, "D = {d}");
        }
    }

    #[test]
    fn appendix_index_pins() {
        let sbs = enumerate_subbands(3, 3);
        // Subband 0 (Fig. 3 front cube): the level-1 HH with spectral
        // high-pass — the only subband with L - H = -3.
        assert_eq!(
            sbs[0],
            Subband3d {
                spatial: SubbandType::Hh,
                spatial_level: 1,
                spectral_high: true,
                spectral_level: 1,
            }
        );
        // Subband 30: the fully low-pass subband (Fig. 3 "LOW-PASS
        // SUBBAND"), largest L - H = 9.
        assert_eq!(
            sbs[30],
            Subband3d {
                spatial: SubbandType::Ll,
                spatial_level: 3,
                spectral_high: false,
                spectral_level: 3,
            }
        );
        // §IV.A worked example: "consider subband 21 in Fig. 3. ...
        // three stages of low-pass filtering are applied in the vertical
        // dimension; two stages of low-pass filtering followed by one
        // stage of high-pass filtering are applied in the horizontal
        // dimension; and one stage each of low-pass filtering and
        // high-pass filtering are applied in the spectral dimension.
        // Thus, H = 2 and L = 6 for this subband."
        let sb21 = sbs[21];
        assert_eq!(
            sb21,
            Subband3d {
                spatial: SubbandType::Hl,
                spatial_level: 3,
                spectral_high: true,
                spectral_level: 2,
            }
        );
        assert_eq!(sb21.filter_ops(), (2, 6));
        // "Thus, all bit planes of subband 21 have odd priority value,
        // with a minimum value of 7" (p = 2b + 7).
        assert_eq!(sb21.priority(0), 7);
        assert_eq!(sb21.priority(3), 13);
        // "For subband 3, H = 2 and L = 1, so bit plane b ... has
        // priority p = 2b + 2." Three level-1 subbands share (H, L) =
        // (2, 1); the Appendix rules place them at indices 1..=3 with
        // rule (3) ranking the vertically-low-pass HL highest.
        let sb3 = sbs[3];
        assert_eq!(sb3.filter_ops(), (2, 1));
        assert_eq!(sb3.priority(0), 2);
        assert_eq!(sb3.priority(5), 12);
        assert_eq!(
            sb3,
            Subband3d {
                spatial: SubbandType::Hl,
                spatial_level: 1,
                spectral_high: true,
                spectral_level: 1,
            }
        );
        for i in 1..=3 {
            assert_eq!(sbs[i].filter_ops(), (2, 1), "index {i}");
        }
    }

    #[test]
    fn indices_are_unique_and_total() {
        // The ordering must totally order every subband set we can
        // produce (the Appendix rules plus the documented tie-break).
        for (ts, tl) in [(3u8, 3u8), (4, 4), (5, 5), (3, 1), (1, 3), (2, 0), (0, 2)] {
            let sbs = enumerate_subbands(ts, tl);
            for i in 1..sbs.len() {
                assert!(
                    index_sort_key(&sbs[i - 1]) < index_sort_key(&sbs[i]),
                    "duplicate index key at ts={ts} tl={tl} i={i}: {:?} vs {:?}",
                    sbs[i - 1],
                    sbs[i]
                );
            }
        }
    }

    #[test]
    fn only_subband_0_has_priority_0_and_none_have_1() {
        // §IV.A: "only subband 0 has a bit plane with priority 0, and no
        // subbands have bit planes with priority 1."
        let sbs = enumerate_subbands(3, 3);
        let p0: Vec<usize> = (0..sbs.len())
            .filter(|&i| sbs[i].priority(0) == 0)
            .collect();
        assert_eq!(p0, vec![0]);
        assert!(sbs.iter().all(|sb| (0..8).all(|b| sb.priority(b) != 1)));
    }

    #[test]
    fn parity_rule_holds() {
        // §IV.A: "all of the bit planes in a subband have even-valued
        // priority, or all have odd-valued priority" (L and H are fixed
        // per subband and successive planes step the priority by 2).
        for sb in enumerate_subbands(3, 3) {
            let parity = sb.priority(0) % 2;
            for b in 1..12 {
                assert_eq!(sb.priority(b) % 2, parity, "{sb:?}");
            }
        }
    }

    #[test]
    fn classification_agrees_with_enumeration() {
        // Every cube position must classify to a subband present in the
        // enumeration, the subband lattices must tile the cube exactly,
        // and per-subband membership must match the lattice descriptor.
        let (w, h, bands) = (16usize, 16usize, 8usize);
        let (ts, tl) = stage_counts(w, h, bands, 3);
        let sbs = enumerate_subbands(ts, tl);
        let mut seen = vec![0usize; sbs.len()];
        for lambda in 0..bands {
            for y in 0..h {
                for x in 0..w {
                    let sb = classify_cube_position(x, y, lambda, ts, tl);
                    let idx = sbs
                        .iter()
                        .position(|s| *s == sb)
                        .unwrap_or_else(|| panic!("({x},{y},{lambda}) -> {sb:?} not enumerated"));
                    seen[idx] += 1;
                    let (xo, yo, sstride) = sb.spatial_lattice();
                    assert_eq!(x % sstride, xo, "{sb:?} x lattice");
                    assert_eq!(y % sstride, yo, "{sb:?} y lattice");
                    let (lo, lstride) = sb.spectral_lattice();
                    assert_eq!(lambda % lstride, lo, "{sb:?} λ lattice");
                }
            }
        }
        // Exact tiling: counts sum to the cube volume, nothing empty.
        assert_eq!(seen.iter().sum::<usize>(), w * h * bands);
        assert!(seen.iter().all(|&c| c > 0), "empty subband: {seen:?}");
    }

    #[test]
    fn schedule_is_priority_ordered_with_index_tiebreak() {
        let sbs = enumerate_subbands(3, 3);
        let plan = cube_schedule(&sbs, 8);
        assert_eq!(plan.len(), 31 * 8);
        for pair in plan.windows(2) {
            assert!(
                pair[0].priority > pair[1].priority
                    || (pair[0].priority == pair[1].priority
                        && pair[0].subband_index > pair[1].subband_index),
                "schedule order violated: {:?} then {:?}",
                pair[0],
                pair[1]
            );
        }
        // The very first entry is the MSB of the highest-priority
        // subband — the fully low-pass subband 30.
        assert_eq!(plan[0].subband_index, 30);
        assert_eq!(plan[0].b, 7);
        // The very last entry is the LSB of subband 0 (priority 0).
        let last = plan.last().unwrap();
        assert_eq!(last.subband_index, 0);
        assert_eq!(last.b, 0);
        assert_eq!(last.priority, 0);
    }

    #[test]
    fn degenerate_geometry_single_subband() {
        // No stages at all -> the whole cube is one subband with H = 0,
        // L = 0, priority p = 2b + 3.
        let sbs = enumerate_subbands(0, 0);
        assert_eq!(sbs.len(), 1);
        assert_eq!(sbs[0].filter_ops(), (0, 0));
        assert_eq!(sbs[0].priority(0), 3);
        assert_eq!(sbs[0].spatial_lattice(), (0, 0, 1));
        assert_eq!(sbs[0].spectral_lattice(), (0, 1));
    }
}
