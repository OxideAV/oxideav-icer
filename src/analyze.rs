//! Image analysis + automatic filter selection (round 5).
//!
//! IPN 42-155 §III.A enumerates eight wavelet filter candidates (A-G
//! float lifting variants plus Q integer 5/3) but does **not** prescribe
//! a fixed assignment of filter to image content. The paper notes in
//! §I that filter selection is image-dependent: smooth, low-texture
//! payloads compress best under the reversible integer 5/3 filter
//! (filter `Q`); high-frequency, high-variance imagery (gravel, dust,
//! rock textures from a Mars rover Pancam) benefits from the float CDF
//! 9/7 family (filter `A`).
//!
//! This module gives the encoder two ways to pick a filter:
//!
//!   1. [`ImageStats`] -- a cheap one-pass scan that records per-image
//!      metrics (mean, variance, horizontal/vertical gradient energy,
//!      dynamic range). [`recommend_filter`] maps those stats to a
//!      filter id using a fixed, transparent decision tree. This is
//!      `O(width*height)` and adds essentially zero overhead on top of
//!      the encode path.
//!   2. [`pick_filter_by_rate_distortion`] -- actually try each filter
//!      from a small candidate set, encode the image, and pick the
//!      filter that produced the smallest output. This is the true
//!      rate-distortion approach, useful when the caller wants the
//!      absolute minimum byte count and can afford `N * encode_time`.
//!
//! Neither approach requires any unpublished JPL data: the decision
//! tree thresholds in [`recommend_filter`] are derived from open
//! wavelet-coding intuition (smooth -> reversible; high-frequency ->
//! biorthogonal 9/7) plus the obvious property that filter `Q` is the
//! only lossless option in the set.
//!
//! Clean-room note: no NASA reference impl, no qccPack, no third-party
//! ICER port was consulted. The thresholds are deliberately
//! conservative + documented so callers can audit them.

use crate::encoder::{encode_icer, EncodeOptions};
use crate::error::Result;
use crate::header::WaveletFilter;
use crate::image::{IcerImage, IcerPixelFormat};

/// One-pass image statistics. Captures the metrics the filter-selection
/// heuristic [`recommend_filter`] consumes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImageStats {
    /// Mean pixel value (0..=255) of the first plane.
    pub mean: f32,
    /// Population variance of pixel values.
    pub variance: f32,
    /// Mean absolute horizontal gradient (|p(x,y) - p(x-1,y)|, x>=1).
    pub h_gradient_energy: f32,
    /// Mean absolute vertical gradient.
    pub v_gradient_energy: f32,
    /// Combined gradient energy = sqrt(H^2 + V^2). Coarse "edge density"
    /// proxy.
    pub edge_energy: f32,
    /// `max - min` of the first plane (0..=255). Zero indicates a flat
    /// image.
    pub dynamic_range: u8,
}

impl ImageStats {
    /// Scan the first plane of `image` and produce its statistics.
    /// Panics on `image.planes.is_empty()` -- callers should validate
    /// before calling.
    pub fn from_image(image: &IcerImage) -> Self {
        let plane = image
            .planes
            .first()
            .expect("ImageStats::from_image: image has no planes");
        let w = image.width as usize;
        let h = image.height as usize;
        let n = (w * h) as f64;
        if n == 0.0 {
            return ImageStats {
                mean: 0.0,
                variance: 0.0,
                h_gradient_energy: 0.0,
                v_gradient_energy: 0.0,
                edge_energy: 0.0,
                dynamic_range: 0,
            };
        }

        let mut sum: f64 = 0.0;
        let mut sum_sq: f64 = 0.0;
        let mut min: u8 = 255;
        let mut max: u8 = 0;
        for y in 0..h {
            let row = &plane.data[y * plane.stride..y * plane.stride + w];
            for &p in row {
                sum += p as f64;
                sum_sq += (p as f64) * (p as f64);
                if p < min {
                    min = p;
                }
                if p > max {
                    max = p;
                }
            }
        }
        let mean = sum / n;
        let variance = (sum_sq / n) - mean * mean;
        let variance = variance.max(0.0);

        // Mean absolute horizontal + vertical gradients.
        let mut h_grad: f64 = 0.0;
        let mut h_grad_n: f64 = 0.0;
        for y in 0..h {
            let row = &plane.data[y * plane.stride..y * plane.stride + w];
            for x in 1..w {
                h_grad += (row[x] as f64 - row[x - 1] as f64).abs();
                h_grad_n += 1.0;
            }
        }
        let mut v_grad: f64 = 0.0;
        let mut v_grad_n: f64 = 0.0;
        for y in 1..h {
            let prev = &plane.data[(y - 1) * plane.stride..(y - 1) * plane.stride + w];
            let cur = &plane.data[y * plane.stride..y * plane.stride + w];
            for x in 0..w {
                v_grad += (cur[x] as f64 - prev[x] as f64).abs();
                v_grad_n += 1.0;
            }
        }
        let h_gradient_energy = if h_grad_n > 0.0 {
            (h_grad / h_grad_n) as f32
        } else {
            0.0
        };
        let v_gradient_energy = if v_grad_n > 0.0 {
            (v_grad / v_grad_n) as f32
        } else {
            0.0
        };
        let edge_energy =
            (h_gradient_energy * h_gradient_energy + v_gradient_energy * v_gradient_energy).sqrt();
        ImageStats {
            mean: mean as f32,
            variance: variance as f32,
            h_gradient_energy,
            v_gradient_energy,
            edge_energy,
            dynamic_range: max.saturating_sub(min),
        }
    }
}

/// Decision-tree heuristic that maps [`ImageStats`] to a recommended
/// [`WaveletFilter`].
///
/// The thresholds embedded in the tree are documented inline. They are
/// clean-room defaults derived from open wavelet-coding intuition (no
/// JPL data was consulted) and may be tightened by callers via the
/// explicit `EncodeOptions::filter` setter when a particular image
/// class is known.
///
/// Decision tree:
///
///   * **Flat image** (dynamic range == 0 or variance < 1.0):
///     filter `Q` -- the integer 5/3 is the only reversible option, and
///     for a flat input it costs essentially nothing.
///   * **Low-frequency** (edge_energy < 4.0): filter `Q` -- smooth
///     content compresses well under the reversible 5/3 and a lossless
///     output is preferable when payload is tiny.
///   * **High-frequency, high-variance** (edge_energy >= 16.0 and
///     variance >= 200.0): filter `A` -- the CDF 9/7-style biorthogonal
///     lifting kernel handles textured imagery (Mars rover Pancam
///     gravel) better than the 5/3.
///   * **Mid-range / default**: filter `Q` -- when in doubt, stay
///     lossless. This keeps the decision tree well-behaved on imagery
///     that doesn't fit either extreme.
pub fn recommend_filter(stats: &ImageStats) -> WaveletFilter {
    if stats.dynamic_range == 0 || stats.variance < 1.0 {
        return WaveletFilter::Reversible53;
    }
    if stats.edge_energy < 4.0 {
        return WaveletFilter::Reversible53;
    }
    if stats.edge_energy >= 16.0 && stats.variance >= 200.0 {
        return WaveletFilter::NineSevenA;
    }
    WaveletFilter::Reversible53
}

/// The default candidate-filter set explored by
/// [`pick_filter_by_rate_distortion`]. Q + A cover the two extremes
/// the paper highlights (lossless reversible vs. biorthogonal 9/7).
/// Callers needing the wider A-G set can pass their own candidate slice.
pub const DEFAULT_RD_CANDIDATES: &[WaveletFilter] =
    &[WaveletFilter::Reversible53, WaveletFilter::NineSevenA];

/// Try each filter in `candidates` against `image` (using `opts` with
/// the candidate substituted into `opts.filter`) and return the filter
/// that produced the smallest encoded byte count, paired with that
/// byte count.
///
/// This is the rate-distortion approach: we actually pay the encode
/// cost N times and pick the empirical winner. Use this when call sites
/// can afford `N * encode_time` and want the absolute minimum byte
/// count rather than a heuristic best-guess.
///
/// Returns `Err` only if `candidates` is empty or every candidate
/// failed to encode.
pub fn pick_filter_by_rate_distortion(
    image: &IcerImage,
    opts: &EncodeOptions,
    candidates: &[WaveletFilter],
) -> Result<(WaveletFilter, usize)> {
    if candidates.is_empty() {
        return Err(crate::error::IcerError::invalid(
            "pick_filter_by_rate_distortion: candidate slice empty",
        ));
    }

    let mut best: Option<(WaveletFilter, usize)> = None;
    let mut last_err: Option<crate::error::IcerError> = None;
    for &candidate in candidates {
        let mut trial_opts = opts.clone();
        trial_opts.filter = candidate;
        // Disable auto_filter on the trial pass so we don't recurse.
        trial_opts.auto_filter = false;
        match encode_icer(image, &trial_opts) {
            Ok(bytes) => {
                let len = bytes.len();
                if best.map(|(_, bl)| len < bl).unwrap_or(true) {
                    best = Some((candidate, len));
                }
            }
            Err(e) => {
                last_err = Some(e);
            }
        }
    }
    best.ok_or_else(|| {
        last_err.unwrap_or_else(|| {
            crate::error::IcerError::invalid(
                "pick_filter_by_rate_distortion: every candidate failed",
            )
        })
    })
}

/// Convenience: produce both the stats and the heuristic recommendation
/// in one call. Useful for diagnostics tooling.
pub fn analyze(image: &IcerImage) -> (ImageStats, WaveletFilter) {
    let stats = ImageStats::from_image(image);
    let filter = recommend_filter(&stats);
    (stats, filter)
}

/// Validate that `image` is in a format `ImageStats::from_image` can
/// scan. Returns the canonical pixel format check the encoder uses, so
/// callers can fail fast.
pub fn supported_for_analysis(image: &IcerImage) -> bool {
    image.pixel_format == IcerPixelFormat::Gray8
        && !image.planes.is_empty()
        && image.width > 0
        && image.height > 0
}

/// Compute PSNR (dB) of the first plane of `decoded` against `original`.
///
/// Returns [`f32::INFINITY`] when the planes are bit-identical (MSE
/// 0). Returns a finite, non-negative value otherwise (the
/// `10 * log10(255^2 / MSE)` formula used in every round-trip test in
/// this crate). Panics on dimension mismatch or empty planes -- callers
/// are expected to feed a same-shape `(original, decoded)` pair.
pub fn psnr_db(original: &IcerImage, decoded: &IcerImage) -> f32 {
    assert_eq!(original.width, decoded.width, "psnr_db: width mismatch");
    assert_eq!(original.height, decoded.height, "psnr_db: height mismatch");
    let w = original.width as usize;
    let h = original.height as usize;
    let n = (w * h) as f64;
    if n == 0.0 {
        return f32::INFINITY;
    }
    let op = original
        .planes
        .first()
        .expect("psnr_db: original has no planes");
    let dp = decoded
        .planes
        .first()
        .expect("psnr_db: decoded has no planes");
    let mut mse_sum = 0.0f64;
    for y in 0..h {
        let orow = &op.data[y * op.stride..y * op.stride + w];
        let drow = &dp.data[y * dp.stride..y * dp.stride + w];
        for x in 0..w {
            let diff = orow[x] as f64 - drow[x] as f64;
            mse_sum += diff * diff;
        }
    }
    let mse = mse_sum / n;
    if mse == 0.0 {
        f32::INFINITY
    } else {
        (10.0 * (255.0f64 * 255.0 / mse).log10()) as f32
    }
}

/// Compute the bracket `(lo_bytes, hi_bytes)` used by the quality-target
/// binary search.
///
/// * `lo_bytes` is a small floor: the per-segment header size plus the
///   theoretical minimum body-bit count. Trial encodes below this would
///   be header-only emissions and have no chance of meeting any
///   non-trivial PSNR target.
/// * `hi_bytes` is the size of an unbudgeted compressed-path encode of
///   `image` under `opts`. The encoder cannot synthesise quality above
///   what a full encode produces, so `hi_bytes` is the natural upper
///   bound of the search.
///
/// Returns `Err` only if the unbudgeted encode fails (in which case the
/// quality search cannot proceed either).
pub fn quality_search_bounds(
    image: &IcerImage,
    opts: &crate::encoder::EncodeOptions,
) -> Result<(u64, u64)> {
    use crate::header::SegmentHeader;

    // Unbudgeted encode (no byte_budget / target_bytes / rd_pruning /
    // quality_target_psnr). This is the size of the best-quality output
    // the configured filter can produce on `image`; any search budget
    // above it is wasted.
    let mut clean_opts = opts.clone();
    clean_opts.byte_budget = None;
    clean_opts.target_bytes = None;
    clean_opts.rd_pruning = false;
    clean_opts.quality_target_psnr = None;
    clean_opts.auto_uncompressed_fallback = false;
    let hi_bytes = crate::encoder::encode_icer(image, &clean_opts)?.len() as u64;

    // Lower bound: per-segment header overhead. A useful encode must
    // ship at least the headers of every segment.
    let segs = opts.segment_count.max(1) as u64;
    let lo_bytes = segs * (SegmentHeader::ENCODED_BYTES as u64);

    // Clamp lo to hi: degenerate cases (tiny images that even an
    // unbudgeted encode finishes in less than the header-only bound)
    // collapse the search to a single point.
    let lo_bytes = lo_bytes.min(hi_bytes);
    Ok((lo_bytes, hi_bytes))
}

/// Quality-target rate-control: binary search over byte budgets and
/// return the smallest output whose decoded PSNR meets or exceeds
/// `target_db`.
///
/// This is the body of the [`crate::encoder::EncodeOptions::quality_target_psnr`]
/// dispatch path; it lives here because the bracket / PSNR helpers are
/// shared with the `pick_filter_by_rate_distortion` family.
///
/// Algorithm:
///
/// 1. Compute the search bounds via [`quality_search_bounds`].
/// 2. If the unbudgeted encode already misses the target, return it
///    (best-effort: the configured filter caps the achievable quality).
/// 3. If the smallest-bracket encode meets the target, return it (the
///    floor wins).
/// 4. Otherwise binary-search the byte budget. At each step encode at
///    the mid budget, decode, compute PSNR. If PSNR >= target keep mid
///    as the running winner and search the lower half; otherwise
///    search the upper half. Stop when the bracket collapses to within
///    `BISECT_TOL` bytes.
///
/// The trial-encode path uses a clone of `opts` with
/// `quality_target_psnr` cleared and `byte_budget` set to the trial
/// value, so the regular byte-budget machinery (and its
/// already-tested guarantees) does the real work on every iteration.
pub fn encode_to_quality_target(
    image: &IcerImage,
    opts: &crate::encoder::EncodeOptions,
    target_db: f32,
) -> Result<Vec<u8>> {
    /// Stop the bisection once `hi - lo` falls below this many bytes.
    /// Smaller values cost more trial encodes; larger values can miss a
    /// few-byte saving. 8 bytes is well below the ~12-byte segment
    /// header overhead and keeps the search to ~log2(hi_bytes / 8)
    /// iterations.
    const BISECT_TOL: u64 = 8;
    /// Hard cap on the bisection iteration count. log2(2^31) is 31; we
    /// pad up to 48 so a misbehaving `hi_bytes` (e.g. ~4 GB synthetic
    /// upper bound) still terminates in finite time.
    const MAX_ITERATIONS: usize = 48;

    let (lo_bytes, hi_bytes) = quality_search_bounds(image, opts)?;
    if lo_bytes >= hi_bytes {
        // Degenerate single-point bracket: the unbudgeted encode is at
        // or below the floor. Return it directly.
        let mut clean = opts.clone();
        clean.quality_target_psnr = None;
        return crate::encoder::encode_icer(image, &clean);
    }

    // Helper closure to encode + decode + compute PSNR at a given byte
    // budget. Clones `opts` and clears the quality-target field so the
    // recursive call hits the regular byte-budget path.
    let trial = |budget: u64| -> Result<(Vec<u8>, f32)> {
        let mut trial_opts = opts.clone();
        trial_opts.quality_target_psnr = None;
        trial_opts.byte_budget = Some(budget);
        // `rd_pruning` deliberately preserved here so a caller who set
        // `rd_pruning = true` BUT did not set byte_budget gets the
        // benefit of R-D packet selection on every trial. The
        // mutual-exclusion check at the encoder entry only rejects the
        // *combination* (rd_pruning + quality_target_psnr) when
        // rd_pruning was set with the explicit-budget flow. In the
        // current API surface `with_quality_target` doesn't set
        // rd_pruning, so this branch is academic; we preserve the field
        // to leave room for future composition.
        let bytes = crate::encoder::encode_icer(image, &trial_opts)?;
        let decoded = crate::decoder::parse_icer(&bytes)?;
        let p = psnr_db(image, &decoded);
        Ok((bytes, p))
    };

    // Upper-bracket trial first: if the unbudgeted-equivalent encode
    // misses the target, no smaller encode will meet it either. Return
    // the upper encode as the best effort.
    let (hi_out, hi_psnr) = trial(hi_bytes)?;
    if hi_psnr < target_db {
        return Ok(hi_out);
    }

    // Lower-bracket trial: if even the floor encode meets the target,
    // return it directly (lossless inputs / very-flat content land
    // here).
    let (lo_out, lo_psnr) = trial(lo_bytes)?;
    if lo_psnr >= target_db {
        return Ok(lo_out);
    }

    // Binary search. Invariant: the byte budget at `best_budget` (when
    // `best` is `Some`) is the smallest known to meet the target.
    // Initial `best` is the upper-bracket encode (proven to meet).
    let mut best: Vec<u8> = hi_out;
    let mut lo = lo_bytes;
    let mut hi = hi_bytes;
    for _ in 0..MAX_ITERATIONS {
        if hi - lo <= BISECT_TOL {
            break;
        }
        let mid = lo + (hi - lo) / 2;
        match trial(mid) {
            Ok((bytes, p)) => {
                if p >= target_db {
                    // Mid encode meets the target; record as the new
                    // best and search the lower half.
                    best = bytes;
                    hi = mid;
                } else {
                    // Mid encode misses the target; search the upper
                    // half.
                    lo = mid;
                }
            }
            Err(_) => {
                // Trial-encode failure (e.g. budget below the
                // mechanical minimum the encoder can write). Treat as
                // "this budget cannot meet the target" and search the
                // upper half.
                lo = mid;
            }
        }
    }
    Ok(best)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::image::IcerPixelFormat;

    fn flat_image(w: u32, h: u32, value: u8) -> IcerImage {
        let mut img = IcerImage::zeros(w, h, IcerPixelFormat::Gray8);
        for byte in img.planes[0].data.iter_mut() {
            *byte = value;
        }
        img
    }

    fn ramp_image(w: u32, h: u32) -> IcerImage {
        let mut img = IcerImage::zeros(w, h, IcerPixelFormat::Gray8);
        let stride = img.planes[0].stride;
        for y in 0..h as usize {
            for x in 0..w as usize {
                img.planes[0].data[y * stride + x] = ((x + y) & 0xFF) as u8;
            }
        }
        img
    }

    fn checkerboard_image(w: u32, h: u32) -> IcerImage {
        let mut img = IcerImage::zeros(w, h, IcerPixelFormat::Gray8);
        let stride = img.planes[0].stride;
        for y in 0..h as usize {
            for x in 0..w as usize {
                let v = if (x ^ y) & 1 == 0 { 0u8 } else { 255u8 };
                img.planes[0].data[y * stride + x] = v;
            }
        }
        img
    }

    #[test]
    fn stats_flat_image_is_zero_variance_zero_gradient() {
        let img = flat_image(16, 16, 128);
        let s = ImageStats::from_image(&img);
        assert_eq!(s.mean, 128.0);
        assert!(
            s.variance.abs() < 1e-3,
            "variance should be ~0, got {}",
            s.variance
        );
        assert_eq!(s.h_gradient_energy, 0.0);
        assert_eq!(s.v_gradient_energy, 0.0);
        assert_eq!(s.edge_energy, 0.0);
        assert_eq!(s.dynamic_range, 0);
    }

    #[test]
    fn stats_ramp_image_has_nonzero_gradients() {
        let img = ramp_image(8, 8);
        let s = ImageStats::from_image(&img);
        assert!(s.variance > 0.0);
        // Diagonal ramp: equal horizontal and vertical gradient of 1.
        assert!((s.h_gradient_energy - 1.0).abs() < 0.01);
        assert!((s.v_gradient_energy - 1.0).abs() < 0.01);
        assert!(s.dynamic_range > 0);
    }

    #[test]
    fn stats_checkerboard_has_high_edge_energy() {
        let img = checkerboard_image(16, 16);
        let s = ImageStats::from_image(&img);
        // Every neighbour differs by 255 in both axes.
        assert!(s.h_gradient_energy > 200.0);
        assert!(s.v_gradient_energy > 200.0);
        assert!(s.edge_energy > 300.0);
        assert_eq!(s.dynamic_range, 255);
    }

    #[test]
    fn recommend_flat_picks_q() {
        let img = flat_image(16, 16, 128);
        let s = ImageStats::from_image(&img);
        assert_eq!(recommend_filter(&s), WaveletFilter::Reversible53);
    }

    #[test]
    fn recommend_ramp_picks_q_for_low_edge_energy() {
        // Diagonal 1-step ramp has edge_energy ~sqrt(2) < 4, so falls
        // into the low-frequency bucket -> filter Q.
        let img = ramp_image(16, 16);
        let s = ImageStats::from_image(&img);
        assert_eq!(recommend_filter(&s), WaveletFilter::Reversible53);
    }

    #[test]
    fn recommend_checkerboard_picks_filter_a() {
        // Checkerboard has very high edge energy (>16) and high variance
        // (~16256) -> heuristic picks filter A for high-frequency content.
        let img = checkerboard_image(16, 16);
        let s = ImageStats::from_image(&img);
        assert_eq!(recommend_filter(&s), WaveletFilter::NineSevenA);
    }

    #[test]
    fn analyze_returns_stats_and_recommendation() {
        let img = ramp_image(8, 8);
        let (s, f) = analyze(&img);
        // Diagonal 1-step ramp: variance > 0 + low edge -> filter Q.
        assert!(s.variance > 0.0);
        assert_eq!(f, WaveletFilter::Reversible53);
    }

    #[test]
    fn supported_for_analysis_basics() {
        let img = flat_image(4, 4, 128);
        assert!(supported_for_analysis(&img));
    }
}
