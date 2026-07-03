//! ICER's error-containment partitioning algorithm — IPN 42-155 §V.D
//! ("The Partitioning Algorithm"), with the §V.B subband mapping hook.
//!
//! ICER determines segment boundaries for the **LL subband** first and
//! then maps them to the other subbands (§V.B); the algorithm here
//! "partitions a rectangle with integer side lengths into smaller
//! rectangles that also have integer side lengths" with the §V.D
//! properties: (1) integer-only arithmetic, (2) nearly square segments,
//! (3) nearly equal areas, (4) a valid result (no zero-width /
//! zero-height segments) whenever the rectangle's area is at least the
//! desired segment count (eq (9), `s <= w * h`).
//!
//! The partition is described by the §V.D output parameters: the
//! segments are arranged in `r` rows; a *top region* of height `h_t`
//! holds `r_t` rows of segments in `c` columns, and the *bottom region*
//! (present when `r_t < r`) holds `r - r_t` rows in `c + 1` columns.
//! Within the top region the first `r_t0` rows of segments have height
//! `y_t` and the rest `y_t + 1`; the first `c_t0` columns have width
//! `x_t` and the rest `x_t + 1`; the bottom region follows the same
//! pattern with `y_b` / `r_b0` / `x_b` / `c_b0`. Segment indices are
//! assigned in raster-scan order (Fig. 17). The decompressor recomputes
//! the same boundaries from the image dimensions, the decomposition
//! depth, and the segment count, so individual boundaries are never
//! encoded (§V.D).
//!
//! The row count `r` is chosen by eq (10) — if `h > (s-1) * w` then
//! `r = s`, else the unique positive integer with
//! `(r-1) * r * w < h * s <= (r+1) * r * w` — which the paper motivates
//! by setting the average segment width equal to the average height
//! (nearly-square Property (2)). The remaining parameters are the §V.D
//! assignments (11)–(21), all integer expressions (the `max` argument
//! of eq (13) is computed as `(h*c*r_t + s/2) / s` exactly as the paper
//! notes).
//!
//! The 2-D pipeline adopts this partition end-to-end via
//! [`crate::encoder::EncodeOptions::with_transform_domain_segments`]
//! (§V.B: one whole-image transform, the LL partition mapped to every
//! subband by [`coefficient_segment_map`], each segment coded
//! independently). The 3-D cube pipeline still uses its documented
//! row-strip convention.

use crate::error::{IcerError, Result};

/// The §V.D output parameters describing a partition (Fig. 17 uses
/// exactly these names).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PartitionParams {
    /// Total rows of segments.
    pub r: usize,
    /// Columns in the top region.
    pub c: usize,
    /// Rows of segments in the top region (`r_t <= r`; `r_t == r` means
    /// no bottom region).
    pub r_t: usize,
    /// Height of the top region.
    pub h_t: usize,
    /// Base segment width in the top region (first `c_t0` columns have
    /// width `x_t`, the rest `x_t + 1`).
    pub x_t: usize,
    /// Number of width-`x_t` columns in the top region.
    pub c_t0: usize,
    /// Base segment height in the top region (first `r_t0` rows).
    pub y_t: usize,
    /// Number of height-`y_t` rows in the top region.
    pub r_t0: usize,
    /// Base segment width in the bottom region (0 when absent).
    pub x_b: usize,
    /// Number of width-`x_b` columns in the bottom region.
    pub c_b0: usize,
    /// Base segment height in the bottom region.
    pub y_b: usize,
    /// Number of height-`y_b` rows in the bottom region.
    pub r_b0: usize,
}

/// One segment rectangle of the partition, in LL-subband coordinates.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SegmentRect {
    /// Raster-scan segment index (Fig. 17).
    pub index: usize,
    /// Left edge.
    pub x: usize,
    /// Top edge.
    pub y: usize,
    /// Width (always >= 1 under eq (9)).
    pub width: usize,
    /// Height (always >= 1 under eq (9)).
    pub height: usize,
}

/// LL-subband dimensions for a `W x H` image after `d` decomposition
/// stages: `w = ceil(W / 2^d)`, `h = ceil(H / 2^d)` (§V.D, citing
/// §II.B).
pub fn ll_dimensions(image_w: usize, image_h: usize, d: u8) -> (usize, usize) {
    let s = 1usize << d;
    (image_w.div_ceil(s), image_h.div_ceil(s))
}

/// Compute the §V.D partition parameters for a `w x h` rectangle and
/// `s` segments. Errors with [`IcerError::Unsupported`] when eq (9)
/// (`s <= w * h`) fails or any input is zero.
pub fn partition_params(w: usize, h: usize, s: usize) -> Result<PartitionParams> {
    if w == 0 || h == 0 || s == 0 {
        return Err(IcerError::unsupported(
            "partition inputs must be positive (§V.D)",
        ));
    }
    if s > w * h {
        return Err(IcerError::unsupported(format!(
            "eq (9) violated: {s} segments > {w} x {h} rectangle area"
        )));
    }

    // Eq (10): r = s when h > (s-1) * w; otherwise the unique positive
    // integer with (r-1) r w < h s <= (r+1) r w, found with the exact
    // integer loop the paper publishes.
    let mut r = 1usize;
    while r < s && (r + 1) * r * w < h * s {
        r += 1;
    }

    let c = s / r; // eq (11)
    let r_t = (c + 1) * r - s; // eq (12)

    // Eq (13): h_t = max(r_t, floor(h c r_t / s + 1/2)), integer form
    // (h*c*r_t + s/2) / s per the §V.D note.
    let h_t = r_t.max((h * c * r_t + s / 2) / s);
    let x_t = w / c; // eq (14)
    let c_t0 = (x_t + 1) * c - w; // eq (15)

    // Eqs (16)/(17) partition the top region's h_t rows into r_t rows
    // of segments. §V.E proves 1 <= r_t <= r (assignments (11)/(12)
    // split s into r divisions, r_t of which have size c, "none of the
    // divisions has size zero"), so the division is always defined.
    debug_assert!((1..=r).contains(&r_t));
    let y_t = h_t / r_t; // eq (16)
    let r_t0 = (y_t + 1) * r_t - h_t; // eq (17)

    // Bottom region (present iff r_t < r): eqs (18)-(21).
    let (x_b, c_b0, y_b, r_b0) = if r_t < r {
        let x_b = w / (c + 1);
        let c_b0 = (x_b + 1) * (c + 1) - w;
        let y_b = (h - h_t) / (r - r_t);
        let r_b0 = (y_b + 1) * (r - r_t) - (h - h_t);
        (x_b, c_b0, y_b, r_b0)
    } else {
        (0, 0, 0, 0)
    };

    Ok(PartitionParams {
        r,
        c,
        r_t,
        h_t,
        x_t,
        c_t0,
        y_t,
        r_t0,
        x_b,
        c_b0,
        y_b,
        r_b0,
    })
}

/// Split `extent` into `count` divisions where the first `base_count`
/// have size `base` and the remainder `base + 1`, returning the running
/// edge offsets (the §V.D row/column division pattern).
fn division_edges(base: usize, base_count: usize, count: usize) -> Vec<(usize, usize)> {
    let mut out = Vec::with_capacity(count);
    let mut at = 0usize;
    for i in 0..count {
        let size = if i < base_count { base } else { base + 1 };
        out.push((at, size));
        at += size;
    }
    out
}

/// Compute the full §V.D partition of a `w x h` rectangle into `s`
/// raster-indexed segment rectangles (Fig. 17 layout: the top region's
/// rows first, then the bottom region's).
pub fn partition(w: usize, h: usize, s: usize) -> Result<Vec<SegmentRect>> {
    let p = partition_params(w, h, s)?;
    let mut out = Vec::with_capacity(s);
    let mut index = 0usize;

    let top_cols = division_edges(p.x_t, p.c_t0, p.c);
    let top_rows = division_edges(p.y_t, p.r_t0, p.r_t);
    for &(y, height) in &top_rows {
        for &(x, width) in &top_cols {
            out.push(SegmentRect {
                index,
                x,
                y,
                width,
                height,
            });
            index += 1;
        }
    }

    if p.r_t < p.r {
        let bot_cols = division_edges(p.x_b, p.c_b0, p.c + 1);
        let bot_rows = division_edges(p.y_b, p.r_b0, p.r - p.r_t);
        for &(y, height) in &bot_rows {
            for &(x, width) in &bot_cols {
                out.push(SegmentRect {
                    index,
                    x,
                    y: p.h_t + y,
                    width,
                    height,
                });
                index += 1;
            }
        }
    }

    debug_assert_eq!(out.len(), s);
    Ok(out)
}

/// LL-resolution segment-index map: entry `y * w + x` is the raster
/// segment index (Fig. 17) of LL pixel `(x, y)` under the §V.D
/// partition of a `w x h` LL subband into `s` segments.
pub fn ll_segment_map(w: usize, h: usize, s: usize) -> Result<Vec<u16>> {
    if s > u16::MAX as usize {
        return Err(IcerError::unsupported(format!(
            "segment count {s} exceeds the u16 map range"
        )));
    }
    let rects = partition(w, h, s)?;
    let mut map = vec![0u16; w * h];
    for r in &rects {
        for y in r.y..r.y + r.height {
            for x in r.x..r.x + r.width {
                map[y * w + x] = r.index as u16;
            }
        }
    }
    Ok(map)
}

/// Full-resolution §V.B coefficient segment map for a `W x H` image
/// transformed with `d` decomposition stages and partitioned into `s`
/// segments.
///
/// §V.B: "segmentation occurs after the wavelet decomposition. Each
/// pixel of the transformed image is assigned to a segment. ... first
/// the LL subband is partitioned into `s` rectangular segments, and
/// then this partition is mapped to the other subbands. Thus, pixels
/// in different subbands corresponding to the same spatial location
/// will belong to the same segment."
///
/// In the crate's Mallat-interleaved coefficient layout (see
/// [`crate::priority::classify_position`]) a subband coefficient at
/// buffer position `(x, y)` covers the spatial neighbourhood of
/// `(x, y)` itself, so the LL pixel co-located with it is simply
/// `(x >> d, y >> d)` — one integer shift maps *every* subband onto
/// the §V.D LL partition, realising the §V.B "same spatial location,
/// same segment" rule exactly (Fig. 14). Entry `y * W + x` of the
/// returned map is the segment index of coefficient `(x, y)`.
pub fn coefficient_segment_map(
    image_w: usize,
    image_h: usize,
    d: u8,
    s: usize,
) -> Result<Vec<u16>> {
    if image_w == 0 || image_h == 0 {
        return Err(IcerError::unsupported(
            "coefficient map requires a non-empty image (§V.B)",
        ));
    }
    let (w_ll, h_ll) = ll_dimensions(image_w, image_h, d);
    let ll_map = ll_segment_map(w_ll, h_ll, s)?;
    let mut map = vec![0u16; image_w * image_h];
    for y in 0..image_h {
        let ly = y >> d;
        for x in 0..image_w {
            let lx = x >> d;
            map[y * image_w + x] = ll_map[ly * w_ll + lx];
        }
    }
    Ok(map)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Assert `rects` exactly tiles a `w x h` rectangle with no
    /// zero-dimension segment and raster-consistent indices.
    fn assert_valid_partition(rects: &[SegmentRect], w: usize, h: usize, s: usize) {
        assert_eq!(rects.len(), s, "{w}x{h}/{s}: wrong segment count");
        let mut cover = vec![0u8; w * h];
        for r in rects {
            assert!(r.width >= 1 && r.height >= 1, "{w}x{h}/{s}: empty {r:?}");
            for y in r.y..r.y + r.height {
                for x in r.x..r.x + r.width {
                    assert!(x < w && y < h, "{w}x{h}/{s}: out of bounds {r:?}");
                    cover[y * w + x] += 1;
                }
            }
        }
        assert!(
            cover.iter().all(|&c| c == 1),
            "{w}x{h}/{s}: not an exact tiling"
        );
        for (i, r) in rects.iter().enumerate() {
            assert_eq!(r.index, i);
        }
    }

    #[test]
    fn figure_17_worked_example() {
        // §V.D Fig. 17: w = 10, h = 14, s = 17 pins every output
        // parameter of the algorithm.
        let p = partition_params(10, 14, 17).unwrap();
        assert_eq!(
            p,
            PartitionParams {
                r: 5,
                c: 3,
                r_t: 3,
                h_t: 7,
                x_t: 3,
                c_t0: 2,
                y_t: 2,
                r_t0: 2,
                x_b: 2,
                c_b0: 2,
                y_b: 3,
                r_b0: 1,
            }
        );

        let rects = partition(10, 14, 17).unwrap();
        assert_valid_partition(&rects, 10, 14, 17);
        // Fig. 17 geometry spot-pins: segment 0 is the 3x2 top-left
        // rectangle; segment 2 is the widened (x_t + 1 = 4) rightmost
        // top-region column; segment 8 ends the top region's third row
        // (height y_t + 1 = 3); segment 9 opens the bottom region at
        // y = h_t = 7 with width x_b = 2 and height y_b = 3; segment 16
        // is the bottom-right corner.
        assert_eq!(
            rects[0],
            SegmentRect {
                index: 0,
                x: 0,
                y: 0,
                width: 3,
                height: 2
            }
        );
        assert_eq!(
            rects[2],
            SegmentRect {
                index: 2,
                x: 6,
                y: 0,
                width: 4,
                height: 2
            }
        );
        assert_eq!(
            rects[8],
            SegmentRect {
                index: 8,
                x: 6,
                y: 4,
                width: 4,
                height: 3
            }
        );
        assert_eq!(
            rects[9],
            SegmentRect {
                index: 9,
                x: 0,
                y: 7,
                width: 2,
                height: 3
            }
        );
        assert_eq!(
            rects[16],
            SegmentRect {
                index: 16,
                x: 7,
                y: 10,
                width: 3,
                height: 4
            }
        );
    }

    #[test]
    fn property_4_validity_sweep() {
        // §V.D Property (4): a valid partition (no zero-width /
        // zero-height segments) whenever s <= w * h. Sweep a broad
        // range of geometries and every legal segment count up to a
        // cap.
        for &(w, h) in &[
            (1usize, 1usize),
            (1, 7),
            (7, 1),
            (2, 7),
            (10, 14),
            (16, 16),
            (13, 5),
            (5, 13),
            (32, 4),
            (3, 3),
            (64, 48),
        ] {
            let max_s = (w * h).min(40);
            for s in 1..=max_s {
                let rects = partition(w, h, s)
                    .unwrap_or_else(|e| panic!("{w}x{h}/{s} unexpectedly failed: {e}"));
                assert_valid_partition(&rects, w, h, s);
            }
        }
        // And the full-degenerate corner: s == w * h gives all-1x1.
        let rects = partition(4, 3, 12).unwrap();
        assert_valid_partition(&rects, 4, 3, 12);
        assert!(rects.iter().all(|r| r.width == 1 && r.height == 1));
    }

    #[test]
    fn eq_9_violation_is_refused() {
        assert!(matches!(
            partition(4, 3, 13),
            Err(IcerError::Unsupported(_))
        ));
        assert!(matches!(partition(0, 3, 1), Err(IcerError::Unsupported(_))));
        assert!(matches!(partition(3, 3, 0), Err(IcerError::Unsupported(_))));
    }

    #[test]
    fn single_segment_is_whole_rectangle() {
        let rects = partition(10, 14, 1).unwrap();
        assert_eq!(
            rects,
            vec![SegmentRect {
                index: 0,
                x: 0,
                y: 0,
                width: 10,
                height: 14
            }]
        );
    }

    #[test]
    fn tall_narrow_rectangle_takes_r_equals_s() {
        // Eq (10) first branch: h > (s-1) w -> r = s (one column of
        // stacked segments).
        let rects = partition(2, 100, 4).unwrap();
        assert_valid_partition(&rects, 2, 100, 4);
        assert!(rects.iter().all(|r| r.width == 2));
        let p = partition_params(2, 100, 4).unwrap();
        assert_eq!(p.r, 4);
        assert_eq!(p.c, 1);
    }

    #[test]
    fn nearly_equal_areas() {
        // §V.D Property (3): areas within the partition stay close.
        // The paper argues areas differ by a bounded factor; pin a
        // conservative 3x max/min ratio across a sweep of non-tiny
        // cases (s well below the area).
        for &(w, h, s) in &[
            (16usize, 16usize, 5usize),
            (32, 20, 7),
            (10, 14, 6),
            (40, 40, 9),
            (25, 33, 11),
        ] {
            let rects = partition(w, h, s).unwrap();
            let areas: Vec<usize> = rects.iter().map(|r| r.width * r.height).collect();
            let (min, max) = (*areas.iter().min().unwrap(), *areas.iter().max().unwrap());
            assert!(
                max <= 3 * min,
                "{w}x{h}/{s}: areas too uneven ({min}..{max})"
            );
        }
    }

    #[test]
    fn ll_dimensions_follow_ceil_rule() {
        assert_eq!(ll_dimensions(256, 248, 3), (32, 31));
        assert_eq!(ll_dimensions(1024, 1024, 6), (16, 16));
        assert_eq!(ll_dimensions(17, 5, 2), (5, 2));
        assert_eq!(ll_dimensions(17, 5, 0), (17, 5));
    }

    #[test]
    fn ll_segment_map_matches_rects() {
        let (w, h, s) = (10usize, 14usize, 17usize);
        let map = ll_segment_map(w, h, s).unwrap();
        let rects = partition(w, h, s).unwrap();
        for r in &rects {
            for y in r.y..r.y + r.height {
                for x in r.x..r.x + r.width {
                    assert_eq!(map[y * w + x] as usize, r.index, "({x},{y})");
                }
            }
        }
        // Every segment index appears at least once.
        for idx in 0..s {
            assert!(
                map.iter().any(|&m| m as usize == idx),
                "segment {idx} empty"
            );
        }
    }

    /// §V.B: pixels in different subbands corresponding to the same
    /// spatial location belong to the same segment. In the interleaved
    /// layout the level-`j` detail coefficient covering spatial location
    /// `(u * 2^j, v * 2^j)` sits at `x = (2u+1) * 2^(j-1)` (high-pass
    /// axis) / `x = u * 2^j` (low-pass axis); all of them must map to
    /// the segment of LL pixel `(u * 2^j >> d, ...)`.
    #[test]
    fn same_spatial_location_same_segment() {
        let (iw, ih, d, s) = (48usize, 40usize, 3u8, 5usize);
        let map = coefficient_segment_map(iw, ih, d, s).unwrap();
        let step = 1usize << d;
        for by in (0..ih).step_by(step) {
            for bx in (0..iw).step_by(step) {
                // The LL coefficient of this spatial block.
                let seg_ll = map[by * iw + bx];
                for j in 1..=d {
                    let half = 1usize << (j - 1);
                    // HL: high-pass in x, low-pass in y.
                    let (hx, hy) = (bx + half, by);
                    // LH: low-pass in x, high-pass in y.
                    let (lx, ly) = (bx, by + half);
                    // HH: high-pass in both.
                    let (dx, dy) = (bx + half, by + half);
                    for &(x, y) in &[(hx, hy), (lx, ly), (dx, dy)] {
                        if x < iw && y < ih {
                            assert_eq!(
                                map[y * iw + x],
                                seg_ll,
                                "level-{j} coefficient ({x},{y}) disagrees with LL block ({bx},{by})"
                            );
                        }
                    }
                }
            }
        }
    }

    /// Fig. 14 analogue: a two-segment partition of a transformed image
    /// splits *every* subband at the scaled position of the LL boundary
    /// (the shaded pixels of Fig. 14 all belong to the left segment).
    #[test]
    fn fig14_two_segment_split_shape() {
        let (iw, ih, d, s) = (64usize, 64usize, 3u8, 2usize);
        let (w_ll, h_ll) = ll_dimensions(iw, ih, d);
        assert_eq!((w_ll, h_ll), (8, 8));
        let rects = partition(w_ll, h_ll, s).unwrap();
        // For an 8x8 LL and 2 segments the §V.D split is one row of two
        // columns (r = 1, c = 2): left/right halves.
        assert_eq!(rects.len(), 2);
        let map = coefficient_segment_map(iw, ih, d, s).unwrap();
        // Segment membership is a pure function of the LL x coordinate:
        // every coefficient column x maps to LL column x >> 3.
        for y in 0..ih {
            for x in 0..iw {
                let expect = if (x >> d) < rects[0].width { 0 } else { 1 };
                assert_eq!(map[y * iw + x] as usize, expect, "({x},{y})");
            }
        }
    }

    #[test]
    fn coefficient_map_covers_odd_geometry() {
        // Odd dims: LL is ceil-sized, and the trailing partial spatial
        // block must still map in-range.
        let (iw, ih, d, s) = (37usize, 23usize, 2u8, 6usize);
        let map = coefficient_segment_map(iw, ih, d, s).unwrap();
        assert_eq!(map.len(), iw * ih);
        assert!(map.iter().all(|&m| (m as usize) < s));
        for idx in 0..s {
            assert!(
                map.iter().any(|&m| m as usize == idx),
                "segment {idx} empty"
            );
        }
    }

    #[test]
    fn coefficient_map_refuses_eq9_violation() {
        // LL of a 16x16 image at d=3 is 2x2 = 4 pixels < 5 segments.
        assert!(matches!(
            coefficient_segment_map(16, 16, 3, 5),
            Err(IcerError::Unsupported(_))
        ));
    }

    #[test]
    fn mer_style_geometry() {
        // A rover-sized frame: 1024 x 1024, six decomposition stages,
        // eight segments (the §VI.B Fig. 22 configuration). The LL
        // subband is 16 x 16 and the partition must be valid + nearly
        // square.
        let (w, h) = ll_dimensions(1024, 1024, 6);
        let rects = partition(w, h, 8).unwrap();
        assert_valid_partition(&rects, w, h, 8);
        for r in &rects {
            let ratio = (r.width.max(r.height) as f64) / (r.width.min(r.height) as f64);
            assert!(ratio <= 2.0, "not nearly square: {r:?}");
        }
    }
}
