//! ICER-3D three-dimensional wavelet decomposition — IPN 42-164 §III.A.
//!
//! ICER-3D (IPN Progress Report 42-164, "ICER-3D: A Progressive
//! Wavelet-Based Compressor for Hyperspectral Images") extends ICER's
//! reversible 2-D transform to hyperspectral cubes: two spatial
//! dimensions (`x` horizontal, `y` vertical) plus one spectral dimension
//! (`λ`, the band axis). The decomposition is **not** a plain 3-D Mallat
//! pyramid. §III.A: "in stages of decomposition after the first, not
//! only is the low-pass subband further decomposed, but spatially
//! low-pass, spectrally high-pass subbands are also further decomposed
//! spatially."
//!
//! # Stage structure
//!
//! Working in the Mallat-interleaved layout the rest of the crate uses
//! (a stage at stride `s` leaves its low-pass outputs on the `2s` lattice
//! and its high-pass outputs offset by `s`), stage `k` (1-based) of the
//! §III.A decomposition is:
//!
//! 1. **Spatial part** — one 2-D stage (rows then columns) on the
//!    stride-`2^(k-1)` spatial lattice of **every** spectral plane. This
//!    single rule realises both §III.A clauses at once: the planes of the
//!    previous low-pass subband LLL and the planes of every earlier
//!    spatially-low-pass / spectrally-high-pass subband together cover
//!    *all* λ, and each is spatially decomposed one more level.
//! 2. **Spectral part** — one 1-D stage along λ on the stride-`2^(k-1)`
//!    spectral lattice, applied at every spatial position of the block
//!    that entered this stage spatially low-pass (the LLL block's full
//!    spatial extent, stride `2^(k-1)`). Because the block was just
//!    spatially split, its newly-created level-`k` spatial detail
//!    subbands receive this spectral stage too — which is exactly how a
//!    level-`k` spatial subband ends up with `k` levels of spectral
//!    decomposition, matching the §III.A alternate description ("a
//!    single level of spectral decomposition is applied across the
//!    first-level spatial subbands; a two-level one-dimensional Mallat
//!    decomposition is applied spectrally across the second-level
//!    spatial subbands; and so on").
//!
//! §III.A footnote: the alternate description "does not produce exactly
//! the same result, because the integer DWT is not quite linear due to
//! rounding operations", and "the decoder should perform the inverse
//! transform operation in exactly the reverse order in which the
//! corresponding forward transforms were performed". The staged form
//! above IS this module's normative operation order; [`inverse_3d`]
//! replays it exactly backwards (spectral before spatial within a stage,
//! stages from deepest to first).
//!
//! # Degenerate extents
//!
//! The 1-D stage (IPN 42-155 §II.A, reused via [`crate::wavelet_int`])
//! needs at least three samples. A dimension whose lattice would fall
//! below three samples stops decomposing: [`spatial_stage_count`] /
//! [`spectral_stage_count`] compute how many stages actually run for a
//! given extent, as pure functions of the geometry, so the encoder and
//! decoder always agree without signalling. Once the spatial recursion
//! has stopped at `t_s < k`, the spectral stage of any later stage `k`
//! applies at the frozen spatial-low-pass lattice `2^t_s`.

use crate::header::WaveletFilter;
use crate::wavelet_int::{forward_1d, inverse_1d, IntFilterParams};

/// IPN 42-164 §III.B dynamic-range expansion factor `γ` of one
/// one-dimensional high-pass filtering operation, as an exact rational
/// `(numerator, denominator)`.
///
/// §III.B: "low-pass filtering does not expand the dynamic range, but
/// high-pass filtering does. The dynamic range expansion following a
/// single one-dimensional high-pass filtering operation can be
/// described by the approximation `h_max − h_min ≈ (x_max − x_min) γ`
/// ... The constant γ is equal to the sum of the absolute values of the
/// filter taps for the linear filter that approximates the particular
/// high-pass filter." The per-filter values are the Table 1 "one
/// high-pass filter operation" column.
pub fn high_pass_gamma(filter: WaveletFilter) -> (u32, u32) {
    match filter {
        WaveletFilter::FilterA => (5, 2),
        WaveletFilter::FilterB => (11, 4),
        WaveletFilter::FilterC => (25, 8),
        WaveletFilter::FilterD => (41, 16),
        WaveletFilter::FilterE => (47, 16),
        WaveletFilter::FilterF => (51, 16),
        WaveletFilter::FilterQ => (11, 4),
    }
}

/// `γ^ops` — the §III.B dynamic-range expansion after `ops`
/// one-dimensional high-pass filtering operations, as an exact rational
/// (IPN 42-164 Table 1 tabulates one, two, and three operations).
///
/// "Under the decomposition structure used by ICER-3D, each subband is
/// produced using at most one high-pass filtering operation in each of
/// the three dimensions (x, y, or λ), so the worst-case dynamic range
/// expansion comes from three high-pass filtering operations" (§III.B),
/// i.e. `ops <= 3` covers every ICER-3D subband.
pub fn dynamic_range_expansion(filter: WaveletFilter, ops: u8) -> (u64, u64) {
    let (n, d) = high_pass_gamma(filter);
    let mut num = 1u64;
    let mut den = 1u64;
    for _ in 0..ops {
        num *= n as u64;
        den *= d as u64;
    }
    (num, den)
}

/// Smallest binary word size (in bits) sufficient to store any DWT
/// coefficient produced by applying up to `ops` one-dimensional
/// high-pass filtering operations to `bit_depth`-bit source samples —
/// the §III.B word-size rule ("The last column of Table 1 can be used
/// to determine necessary binary word sizes to accommodate dynamic
/// range expansion for a given source bit depth"): the smallest `w`
/// with `2^w >= 2^bit_depth * γ^ops`.
///
/// Worked §III.B pin: "when using filter A, 16-bit words are sufficient
/// to store the coefficients produced by applying a 3-D decomposition
/// to 12-bit data" — `coefficient_word_bits(12, FilterA, 3) == 16` —
/// "but the other filter choices may produce DWT coefficients that
/// cannot be stored in 16-bit words".
pub fn coefficient_word_bits(bit_depth: u8, filter: WaveletFilter, ops: u8) -> u8 {
    let (num, den) = dynamic_range_expansion(filter, ops);
    let mut extra = 0u8;
    while (1u64 << extra) * den < num {
        extra += 1;
    }
    bit_depth + extra
}

/// Number of lattice positions `{0, s, 2s, ...}` inside `[0, extent)`.
#[inline]
fn lattice_len(extent: usize, stride: usize) -> usize {
    extent.div_ceil(stride)
}

/// How many spatial stages of the §III.A decomposition actually run for
/// a `width x height` spatial extent and a requested depth of `levels`.
///
/// Stage `k` runs only if the stride-`2^(k-1)` lattice still has at
/// least three samples along **both** spatial axes (the IPN 42-155 §II.A
/// 1-D stage needs `N >= 3`); once a stage is skipped every deeper
/// spatial stage is skipped too, so the result is a stage *count*.
pub fn spatial_stage_count(width: usize, height: usize, levels: u8) -> u8 {
    let mut t = 0u8;
    while t < levels {
        let s = 1usize << t;
        if lattice_len(width, s) < 3 || lattice_len(height, s) < 3 {
            break;
        }
        t += 1;
    }
    t
}

/// How many spectral stages actually run for a `bands`-deep cube and a
/// requested depth of `levels` (same `N >= 3` lattice rule along λ).
pub fn spectral_stage_count(bands: usize, levels: u8) -> u8 {
    let mut t = 0u8;
    while t < levels {
        let s = 1usize << t;
        if lattice_len(bands, s) < 3 {
            break;
        }
        t += 1;
    }
    t
}

/// One 1-D lattice inside the cube buffer: samples sit at
/// `base + i * step * stride` for `i` in `0..count`.
#[derive(Clone, Copy)]
struct Lattice {
    base: usize,
    step: usize,
    stride: usize,
    count: usize,
}

impl Lattice {
    #[inline]
    fn pos(&self, i: usize) -> usize {
        self.base + i * self.step * self.stride
    }
}

/// Reusable transform scratch buffers (sample gather + l/h split).
#[derive(Default)]
struct Scratch {
    x: Vec<i32>,
    l: Vec<i32>,
    h: Vec<i32>,
}

/// One forward 1-D stage over a [`Lattice`]: samples are gathered,
/// transformed, and written back with low-pass outputs on the doubled
/// lattice and high-pass outputs offset by one lattice step — the
/// Mallat-interleaved layout.
fn forward_lattice(buf: &mut [i32], lat: Lattice, p: &IntFilterParams, s: &mut Scratch) {
    debug_assert!(lat.count >= 3);
    s.x.clear();
    for i in 0..lat.count {
        s.x.push(buf[lat.pos(i)]);
    }
    forward_1d(&s.x, p, &mut s.l, &mut s.h);
    for (i, &v) in s.l.iter().enumerate() {
        buf[lat.pos(2 * i)] = v;
    }
    for (i, &v) in s.h.iter().enumerate() {
        buf[lat.pos(2 * i + 1)] = v;
    }
}

/// Exact inverse of [`forward_lattice`].
fn inverse_lattice(buf: &mut [i32], lat: Lattice, p: &IntFilterParams, s: &mut Scratch) {
    debug_assert!(lat.count >= 3);
    let n_lo = lat.count.div_ceil(2);
    let n_hi = lat.count / 2;
    s.l.clear();
    s.h.clear();
    for i in 0..n_lo {
        s.l.push(buf[lat.pos(2 * i)]);
    }
    for i in 0..n_hi {
        s.h.push(buf[lat.pos(2 * i + 1)]);
    }
    inverse_1d(&s.l, &s.h, lat.count, p, &mut s.x);
    for (i, &v) in s.x.iter().enumerate() {
        buf[lat.pos(i)] = v;
    }
}

/// Shared per-stage geometry: strides and lattice lengths for stage `k`
/// (1-based) of a `width x height x bands` cube with `ts` spatial and
/// `tl` spectral stages.
struct StageGeom {
    /// Spatial lattice stride for the stage's spatial part, `2^(k-1)`.
    s: usize,
    /// Lattice sample counts along x / y at stride `s`.
    nx: usize,
    ny: usize,
    /// Spatial stride of the spatial-low-pass lattice the spectral part
    /// applies at: `2^min(k-1, ts)` **before** the stage's spatial part,
    /// i.e. `2^min(k, ts)` after it ran — see [`stage_geom`].
    sp: usize,
    /// Spectral lattice sample count at stride `s`.
    nl: usize,
}

fn stage_geom(width: usize, height: usize, bands: usize, k: u8, ts: u8) -> StageGeom {
    let s = 1usize << (k - 1);
    // The spectral part of stage k applies over the spatial extent of
    // the block that entered stage k spatially low-pass. When the
    // spatial recursion is still running (k <= ts) that block is the
    // stride-2^(k-1) lattice; if it stopped earlier at ts < k, the
    // spatial-low-pass lattice is frozen at 2^ts.
    let sp = 1usize << (k - 1).min(ts);
    StageGeom {
        s,
        nx: lattice_len(width, s),
        ny: lattice_len(height, s),
        sp,
        nl: lattice_len(bands, s),
    }
}

/// Forward §III.A ICER-3D decomposition of a band-major cube
/// (`buf[λ * width * height + y * width + x]`), in place, using the
/// spec-exact reversible integer filter `filter` (IPN 42-155 §II.A
/// Table 1) in all three dimensions.
///
/// `levels` is the requested stage count; the per-dimension counts that
/// actually run are [`spatial_stage_count`] / [`spectral_stage_count`].
pub fn forward_3d(
    buf: &mut [i32],
    width: usize,
    height: usize,
    bands: usize,
    levels: u8,
    filter: WaveletFilter,
) {
    debug_assert_eq!(buf.len(), width * height * bands);
    let p = filter.int_params();
    let ts = spatial_stage_count(width, height, levels);
    let tl = spectral_stage_count(bands, levels);
    let plane = width * height;
    let mut sc = Scratch::default();

    for k in 1..=levels {
        let g = stage_geom(width, height, bands, k, ts);
        if k <= ts {
            // Spatial part: one 2-D stage (rows then columns) on the
            // stride-s lattice of every spectral plane.
            for band in 0..bands {
                let b0 = band * plane;
                for yi in 0..g.ny {
                    let lat = Lattice {
                        base: b0 + (yi * g.s) * width,
                        step: 1,
                        stride: g.s,
                        count: g.nx,
                    };
                    forward_lattice(buf, lat, &p, &mut sc);
                }
                for xi in 0..g.nx {
                    let lat = Lattice {
                        base: b0 + xi * g.s,
                        step: width,
                        stride: g.s,
                        count: g.ny,
                    };
                    forward_lattice(buf, lat, &p, &mut sc);
                }
            }
        }
        if k <= tl {
            // Spectral part: one 1-D stage along λ over the full spatial
            // extent of the spatially-low-pass block (stride g.sp).
            let mut y = 0usize;
            while y < height {
                let mut x = 0usize;
                while x < width {
                    let lat = Lattice {
                        base: y * width + x,
                        step: plane,
                        stride: g.s,
                        count: g.nl,
                    };
                    forward_lattice(buf, lat, &p, &mut sc);
                    x += g.sp;
                }
                y += g.sp;
            }
        }
    }
}

/// Exact inverse of [`forward_3d`]: the same stages replayed backwards
/// (deepest stage first; within a stage the spectral part is undone
/// before the spatial part), per the §III.A requirement that the decoder
/// "perform the inverse transform operation in exactly the reverse order
/// in which the corresponding forward transforms were performed".
pub fn inverse_3d(
    buf: &mut [i32],
    width: usize,
    height: usize,
    bands: usize,
    levels: u8,
    filter: WaveletFilter,
) {
    debug_assert_eq!(buf.len(), width * height * bands);
    let p = filter.int_params();
    let ts = spatial_stage_count(width, height, levels);
    let tl = spectral_stage_count(bands, levels);
    let plane = width * height;
    let mut sc = Scratch::default();

    for k in (1..=levels).rev() {
        let g = stage_geom(width, height, bands, k, ts);
        if k <= tl {
            let mut y = 0usize;
            while y < height {
                let mut x = 0usize;
                while x < width {
                    let lat = Lattice {
                        base: y * width + x,
                        step: plane,
                        stride: g.s,
                        count: g.nl,
                    };
                    inverse_lattice(buf, lat, &p, &mut sc);
                    x += g.sp;
                }
                y += g.sp;
            }
        }
        if k <= ts {
            // Undo columns before rows (the forward did rows then
            // columns).
            for band in 0..bands {
                let b0 = band * plane;
                for xi in 0..g.nx {
                    let lat = Lattice {
                        base: b0 + xi * g.s,
                        step: width,
                        stride: g.s,
                        count: g.ny,
                    };
                    inverse_lattice(buf, lat, &p, &mut sc);
                }
                for yi in 0..g.ny {
                    let lat = Lattice {
                        base: b0 + (yi * g.s) * width,
                        step: 1,
                        stride: g.s,
                        count: g.nx,
                    };
                    inverse_lattice(buf, lat, &p, &mut sc);
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const ALL: [WaveletFilter; 7] = [
        WaveletFilter::FilterQ,
        WaveletFilter::FilterA,
        WaveletFilter::FilterB,
        WaveletFilter::FilterC,
        WaveletFilter::FilterD,
        WaveletFilter::FilterE,
        WaveletFilter::FilterF,
    ];

    /// A deterministic pseudo-textured cube: per-band DC offsets (the
    /// "systematic variations in signal level of different spectral
    /// bands" §III.A calls out) over a spatial gradient + modulation.
    fn test_cube(w: usize, h: usize, bands: usize) -> Vec<i32> {
        let mut v = Vec::with_capacity(w * h * bands);
        for b in 0..bands {
            let dc = (b as i32 * 37) % 300 - 150;
            for y in 0..h {
                for x in 0..w {
                    let t = (x as i32 * 3 + y as i32 * 5 + b as i32 * 7) % 61 - 30;
                    v.push(dc + t + ((x * y) % 13) as i32);
                }
            }
        }
        v
    }

    #[test]
    fn roundtrip_all_filters_typical_geometry() {
        // Every filter is a reversible integer transform in all three
        // dimensions, so forward + inverse must be bit-exact (IPN 42-164
        // §III.A uses the same reversible filters as ICER).
        for f in ALL {
            let (w, h, bands) = (16usize, 12usize, 8usize);
            let original = test_cube(w, h, bands);
            let mut buf = original.clone();
            forward_3d(&mut buf, w, h, bands, 3, f);
            assert_ne!(buf, original, "filter {f:?}: transform must change data");
            inverse_3d(&mut buf, w, h, bands, 3, f);
            assert_eq!(buf, original, "filter {f:?} 3-D round-trip mismatch");
        }
    }

    #[test]
    fn roundtrip_odd_and_thin_geometries() {
        // Odd extents, thin strips, band counts around the lattice
        // minimum, and depth-limited combinations must all stay exactly
        // reversible under the stage-count gating.
        let cases = [
            (17usize, 13usize, 5usize, 3u8),
            (7, 7, 3, 4),
            (5, 32, 4, 2),
            (32, 5, 7, 3),
            (9, 9, 1, 3), // single band: pure spatial decomposition
            (3, 3, 3, 2), // minimum lattice everywhere
            (4, 4, 224, 3),
            (2, 2, 16, 3), // spatially too small: spectral-only
            (1, 1, 9, 2),
        ];
        for f in [WaveletFilter::FilterQ, WaveletFilter::FilterC] {
            for &(w, h, bands, levels) in &cases {
                let original = test_cube(w, h, bands);
                let mut buf = original.clone();
                forward_3d(&mut buf, w, h, bands, levels, f);
                inverse_3d(&mut buf, w, h, bands, levels, f);
                assert_eq!(buf, original, "{f:?} {w}x{h}x{bands} L{levels} mismatch");
            }
        }
    }

    #[test]
    fn roundtrip_levels_sweep() {
        let (w, h, bands) = (20usize, 20usize, 12usize);
        let original = test_cube(w, h, bands);
        for levels in 1..=6u8 {
            let mut buf = original.clone();
            forward_3d(&mut buf, w, h, bands, levels, WaveletFilter::FilterQ);
            inverse_3d(&mut buf, w, h, bands, levels, WaveletFilter::FilterQ);
            assert_eq!(buf, original, "level {levels} mismatch");
        }
    }

    #[test]
    fn flat_cube_has_zero_high_pass_everywhere() {
        // A constant cube must transform to a constant on the deepest
        // low-pass lattice and exact zeros everywhere else — any rounding
        // bias in the stage plumbing would surface here.
        let (w, h, bands, levels) = (16usize, 16usize, 8usize, 3u8);
        let mut buf = vec![55i32; w * h * bands];
        forward_3d(&mut buf, w, h, bands, levels, WaveletFilter::FilterQ);
        let ts = spatial_stage_count(w, h, levels) as usize;
        let tl = spectral_stage_count(bands, levels) as usize;
        for b in 0..bands {
            for y in 0..h {
                for x in 0..w {
                    let v = buf[b * w * h + y * w + x];
                    let deep_low = x % (1 << ts) == 0 && y % (1 << ts) == 0 && b % (1 << tl) == 0;
                    if deep_low {
                        assert_eq!(v, 55, "low-pass sample changed at ({x},{y},{b})");
                    } else {
                        assert_eq!(v, 0, "non-zero high-pass at ({x},{y},{b})");
                    }
                }
            }
        }
    }

    #[test]
    fn stage_counts_follow_lattice_minimum() {
        // The N >= 3 rule from IPN 42-155 §II.A gates each stage.
        assert_eq!(spatial_stage_count(64, 64, 6), 5); // 64/32 = 2 < 3 stops stage 6
        assert_eq!(spatial_stage_count(64, 64, 3), 3);
        assert_eq!(spatial_stage_count(3, 3, 6), 1);
        assert_eq!(spatial_stage_count(2, 64, 6), 0);
        assert_eq!(spectral_stage_count(224, 3), 3);
        assert_eq!(spectral_stage_count(224, 8), 7); // ceil(224/128) = 2 < 3
        assert_eq!(spectral_stage_count(3, 4), 1);
        assert_eq!(spectral_stage_count(1, 4), 0);
    }

    #[test]
    fn spectral_depth_matches_spatial_level() {
        // §III.A: a level-k spatial subband receives exactly k levels of
        // spectral decomposition (the alternate description's "a
        // single level of spectral decomposition is applied across the
        // first-level spatial subbands; a two-level one-dimensional
        // Mallat decomposition is applied spectrally across the
        // second-level spatial subbands; and so on").
        //
        // Probe it with a λ-constant cube `v(x, y, λ) = f(x, y)`: every
        // spatial stage acts identically on the identical planes, and a
        // spectral stage on a constant pencil emits zero high-pass and
        // leaves the low-pass untouched. So at a spatial position that
        // received `n` spectral stages, the coefficient at λ is zero
        // when λ's spectral-high level is `<= n` and equals the
        // spatially-transformed value F(x, y) when `λ ≡ 0 (mod 2^n)`.
        let (w, h, bands, levels) = (16usize, 16usize, 16usize, 3u8);
        assert_eq!(spectral_stage_count(bands, levels), 3);
        let f = |x: usize, y: usize| ((x * 7 + y * 13) % 47) as i32 + ((x * y) % 11) as i32 - 20;
        let mut cube = Vec::with_capacity(w * h * bands);
        for _b in 0..bands {
            for y in 0..h {
                for x in 0..w {
                    cube.push(f(x, y));
                }
            }
        }
        forward_3d(&mut cube, w, h, bands, levels, WaveletFilter::FilterQ);
        let at = |x: usize, y: usize, b: usize| cube[b * w * h + y * w + x];

        // (1, 0) is level-1 spatial detail: exactly one spectral stage
        // ran there, so λ = 1 is spectral-high (zero) but λ = 2 was
        // never split off — it still holds the same value as λ = 0.
        assert_ne!(at(1, 0, 0), 0, "fixture must have level-1 detail energy");
        assert_eq!(at(1, 0, 1), 0);
        assert_eq!(
            at(1, 0, 2),
            at(1, 0, 0),
            "level-1 detail got >1 spectral stage"
        );
        assert_eq!(at(1, 0, 4), at(1, 0, 0));

        // (2, 0) is level-2 spatial detail: two spectral stages ran, so
        // λ = 2 is now spectral-high (zero) while λ = 4 is not.
        assert_ne!(at(2, 0, 0), 0, "fixture must have level-2 detail energy");
        assert_eq!(at(2, 0, 1), 0);
        assert_eq!(
            at(2, 0, 2),
            0,
            "level-2 detail missed its 2nd spectral stage"
        );
        assert_eq!(
            at(2, 0, 4),
            at(2, 0, 0),
            "level-2 detail got >2 spectral stages"
        );

        // (0, 0) is deep low-pass: all three spectral stages ran.
        assert_eq!(
            at(0, 0, 4),
            0,
            "deep low-pass missed its 3rd spectral stage"
        );
    }

    #[test]
    fn table1_dynamic_range_expansion_pins() {
        // IPN 42-164 §III.B Table 1, all 21 cells: γ, γ², γ³ as exact
        // rationals, and the published log2 columns to their printed
        // 2-decimal precision.
        type Row = (WaveletFilter, (u32, u32), (u64, u64), (u64, u64), [f64; 3]);
        let rows: [Row; 7] = [
            (
                WaveletFilter::FilterA,
                (5, 2),
                (25, 4),
                (125, 8),
                [1.32, 2.64, 3.97],
            ),
            (
                WaveletFilter::FilterB,
                (11, 4),
                (121, 16),
                (1331, 64),
                [1.46, 2.92, 4.38],
            ),
            (
                WaveletFilter::FilterC,
                (25, 8),
                (625, 64),
                (15625, 512),
                [1.64, 3.29, 4.93],
            ),
            (
                WaveletFilter::FilterD,
                (41, 16),
                (1681, 256),
                (68921, 4096),
                [1.36, 2.72, 4.07],
            ),
            (
                WaveletFilter::FilterE,
                (47, 16),
                (2209, 256),
                (103823, 4096),
                [1.55, 3.11, 4.66],
            ),
            (
                WaveletFilter::FilterF,
                (51, 16),
                (2601, 256),
                (132651, 4096),
                [1.67, 3.34, 5.02],
            ),
            (
                WaveletFilter::FilterQ,
                (11, 4),
                (121, 16),
                (1331, 64),
                [1.46, 2.92, 4.38],
            ),
        ];
        for (filter, g1, g2, g3, log_bits) in rows {
            assert_eq!(high_pass_gamma(filter), g1, "{filter:?} γ");
            assert_eq!(
                dynamic_range_expansion(filter, 1),
                (g1.0 as u64, g1.1 as u64),
                "{filter:?} γ^1"
            );
            assert_eq!(dynamic_range_expansion(filter, 2), g2, "{filter:?} γ^2");
            assert_eq!(dynamic_range_expansion(filter, 3), g3, "{filter:?} γ^3");
            for (ops, expect) in log_bits.iter().enumerate() {
                let (n, d) = dynamic_range_expansion(filter, ops as u8 + 1);
                let bits = (n as f64 / d as f64).log2();
                assert!(
                    (bits - expect).abs() < 0.005,
                    "{filter:?} log2 γ^{}: {bits} vs published {expect}",
                    ops + 1
                );
            }
        }
    }

    #[test]
    fn word_size_rule_matches_the_worked_example() {
        // §III.B: "when using filter A, 16-bit words are sufficient to
        // store the coefficients produced by applying a 3-D
        // decomposition to 12-bit data (such as uncalibrated AVIRIS
        // data). But the other filter choices may produce DWT
        // coefficients that cannot be stored in 16-bit words."
        assert_eq!(coefficient_word_bits(12, WaveletFilter::FilterA, 3), 16);
        for filter in [
            WaveletFilter::FilterB,
            WaveletFilter::FilterC,
            WaveletFilter::FilterD,
            WaveletFilter::FilterE,
            WaveletFilter::FilterF,
            WaveletFilter::FilterQ,
        ] {
            assert!(
                coefficient_word_bits(12, filter, 3) > 16,
                "{filter:?} must overflow 16-bit words on 12-bit data"
            );
        }
        // Zero high-pass operations never expand the range (§III.B:
        // low-pass filtering does not expand the dynamic range).
        assert_eq!(coefficient_word_bits(12, WaveletFilter::FilterF, 0), 12);
        // The crate's i32 coefficient storage covers the worst case the
        // wire admits: 16-bit samples through the widest filter (F).
        assert!(coefficient_word_bits(16, WaveletFilter::FilterF, 3) <= 31);
    }

    #[test]
    fn empirical_range_stays_within_the_word_size_rule() {
        // Drive the actual 3-D transform with extreme-value cubes and
        // check no coefficient exceeds the §III.B word size (the γ rule
        // is an approximation of the linearised filter, but it is the
        // published storage rule — the transform must live within it
        // for the adversarial full-range inputs the encoder accepts).
        let (w, h, bands) = (16usize, 16usize, 8usize);
        for filter in [
            WaveletFilter::FilterA,
            WaveletFilter::FilterF,
            WaveletFilter::FilterQ,
        ] {
            for depth in [8u8, 12, 16] {
                let hi = (1i32 << depth) - 1;
                let shift = 1i32 << (depth - 1);
                // Checkerboard of range extremes (worst case for a
                // high-pass response), level-shifted like the encoder.
                let mut buf: Vec<i32> = (0..w * h * bands)
                    .map(|i| {
                        let x = i % w;
                        let y = (i / w) % h;
                        let l = i / (w * h);
                        if (x + y + l) % 2 == 0 {
                            hi - shift
                        } else {
                            -shift
                        }
                    })
                    .collect();
                forward_3d(&mut buf, w, h, bands, 3, filter);
                let max_mag = buf.iter().map(|&c| c.unsigned_abs()).max().unwrap();
                let word = coefficient_word_bits(depth, filter, 3);
                // A w-bit word stores magnitudes < 2^(w-1) (sign bit).
                assert!(
                    max_mag < 1u32 << (word - 1),
                    "{filter:?} depth {depth}: |coeff| {max_mag} exceeds {word}-bit words"
                );
            }
        }
    }
}
