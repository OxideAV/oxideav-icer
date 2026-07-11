//! Image analysis + automatic filter selection (round 5).
//!
//! IPN 42-155 §II.A specifies seven reversible integer wavelet filter
//! candidates (A-F plus Q) but does **not** prescribe a fixed
//! assignment of filter to image content. §II.D ("Quantitative Filter
//! Comparison") shows the choice is image-dependent -- the per-filter
//! rate-distortion and lossless-rate rankings differ across imagery,
//! and the lossless ranking differs from the lossy one. Every filter
//! is losslessly reversible, so the selection is purely a byte-count /
//! truncated-quality trade, never a fidelity one.
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
//!      rate-allocation approach, useful when the caller wants the
//!      absolute minimum byte count and can afford `N * encode_time`.
//!
//! Neither approach requires any unpublished JPL data: the decision
//! tree thresholds in [`recommend_filter`] are derived from open
//! wavelet-coding intuition (smooth -> shorter support; textured ->
//! stronger high-pass prediction) and are deliberately conservative +
//! documented so callers can audit them.

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
/// Decision tree (every branch is losslessly reversible -- IPN 42-155
/// §II.A -- so the choice only moves the byte count and the truncated
/// progressive quality, never the full-quality fidelity):
///
///   * **Flat image** (dynamic range == 0 or variance < 1.0):
///     filter `Q` -- for a flat input every filter is near-free; the
///     deployed default wins ties.
///   * **Low-frequency** (edge_energy < 4.0): filter `Q` -- smooth
///     content compresses well under the 5/3-support kernel.
///   * **High-frequency, high-variance** (edge_energy >= 16.0 and
///     variance >= 200.0): filter `A` -- with `beta = 0` its high-pass
///     predictor ignores the next raw difference, which empirically
///     tracks textured imagery better than filter Q's `beta = 1/4`
///     (the two share the same `alpha` triple, §II.A Table 1).
///   * **Mid-range / default**: filter `Q` -- the deployed default.
///     This keeps the decision tree well-behaved on imagery that
///     doesn't fit either extreme.
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
/// [`pick_filter_by_rate_distortion`]. Q + A are the pair the paper's
/// deployments highlight; they share the same `alpha` triple and
/// differ only in `beta` (IPN 42-155 §II.A Table 1). Callers needing
/// the wider A-F set can pass their own candidate slice.
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

/// A structured distortion report comparing a decoded image against its
/// original, computed in a single pass over the first plane.
///
/// Unlike [`psnr_db`] (which panics on a shape mismatch and returns only
/// the single PSNR number), [`DistortionReport::compare`] returns
/// [`Err`] on a geometry contradiction and bundles every common
/// distortion metric so a caller verifying a lossy encode (or the
/// fidelity of an ROI-prioritised segment, round 6) gets them all
/// without re-scanning the pixels once per metric.
///
/// All metrics are over the 8-bit pixel domain (`0..=255`):
///
/// * `mse` -- mean squared error.
/// * `rmse` -- `sqrt(mse)`, the error in pixel units.
/// * `mae` -- mean absolute error.
/// * `max_abs_error` -- the single largest `|original - decoded|` over
///   the frame (`0..=255`). A worst-case bound, useful when a mission
///   needs a per-pixel error ceiling rather than an averaged one.
/// * `psnr_db` -- peak signal-to-noise ratio in dB, identical to
///   [`psnr_db`] (`f32::INFINITY` when the images are bit-identical).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DistortionReport {
    /// Mean squared error over the first plane.
    pub mse: f64,
    /// Root mean squared error (`sqrt(mse)`), in pixel units.
    pub rmse: f64,
    /// Mean absolute error over the first plane, in pixel units.
    pub mae: f64,
    /// Largest single-pixel absolute error (`0..=255`).
    pub max_abs_error: u8,
    /// Peak signal-to-noise ratio in dB (`f32::INFINITY` if bit-exact).
    pub psnr_db: f32,
}

impl DistortionReport {
    /// Compare the first plane of `decoded` against `original` and
    /// produce every distortion metric in one pass.
    ///
    /// Returns [`crate::IcerError::Unsupported`] on a width/height mismatch or
    /// when either image has no planes -- the non-panicking complement
    /// to [`psnr_db`]. A zero-pixel image (`width == 0 || height == 0`)
    /// is a bit-exact match by construction: all averaged metrics are
    /// `0` and `psnr_db` is `f32::INFINITY`.
    pub fn compare(original: &IcerImage, decoded: &IcerImage) -> Result<Self> {
        if original.width != decoded.width || original.height != decoded.height {
            return Err(crate::error::IcerError::unsupported(format!(
                "distortion: geometry mismatch {}x{} vs {}x{}",
                original.width, original.height, decoded.width, decoded.height
            )));
        }
        let op = original.planes.first().ok_or_else(|| {
            crate::error::IcerError::unsupported("distortion: original has no planes")
        })?;
        let dp = decoded.planes.first().ok_or_else(|| {
            crate::error::IcerError::unsupported("distortion: decoded has no planes")
        })?;

        let w = original.width as usize;
        let h = original.height as usize;
        let n = (w * h) as f64;
        if n == 0.0 {
            return Ok(DistortionReport {
                mse: 0.0,
                rmse: 0.0,
                mae: 0.0,
                max_abs_error: 0,
                psnr_db: f32::INFINITY,
            });
        }

        let mut sq_sum = 0.0f64;
        let mut abs_sum = 0.0f64;
        let mut max_abs = 0u8;
        for y in 0..h {
            let orow = &op.data[y * op.stride..y * op.stride + w];
            let drow = &dp.data[y * dp.stride..y * dp.stride + w];
            for x in 0..w {
                let abs = (orow[x] as i32 - drow[x] as i32).unsigned_abs() as u8;
                if abs > max_abs {
                    max_abs = abs;
                }
                let d = abs as f64;
                sq_sum += d * d;
                abs_sum += d;
            }
        }
        let mse = sq_sum / n;
        let psnr = if mse == 0.0 {
            f32::INFINITY
        } else {
            (10.0 * (255.0f64 * 255.0 / mse).log10()) as f32
        };
        Ok(DistortionReport {
            mse,
            rmse: mse.sqrt(),
            mae: abs_sum / n,
            max_abs_error: max_abs,
            psnr_db: psnr,
        })
    }
}

/// Compute the mean absolute error over a rectangular sub-region of the
/// first plane, in pixel units (`0.0..=255.0`).
///
/// The region is `[x0, x0 + region_w) x [y0, y0 + region_h)`. This is
/// the programmatic form of the centre-band / periphery MAE comparison
/// the round-6 ROI-prioritisation feature documents: a caller can
/// measure that a centre strip kept its fidelity (low MAE) while the
/// periphery was truncated (high MAE) under a tight byte budget.
///
/// Returns [`crate::IcerError::Unsupported`] on a geometry mismatch, a missing
/// plane, or a region that does not fit entirely inside the image.
pub fn region_mae(
    original: &IcerImage,
    decoded: &IcerImage,
    x0: u32,
    y0: u32,
    region_w: u32,
    region_h: u32,
) -> Result<f64> {
    if original.width != decoded.width || original.height != decoded.height {
        return Err(crate::error::IcerError::unsupported(format!(
            "region_mae: geometry mismatch {}x{} vs {}x{}",
            original.width, original.height, decoded.width, decoded.height
        )));
    }
    // The region must lie fully inside the image. `checked_add` guards
    // the u32 overflow a malicious (x0, region_w) pair could induce.
    let x_end = x0
        .checked_add(region_w)
        .ok_or_else(|| crate::error::IcerError::unsupported("region_mae: x overflow"))?;
    let y_end = y0
        .checked_add(region_h)
        .ok_or_else(|| crate::error::IcerError::unsupported("region_mae: y overflow"))?;
    if x_end > original.width || y_end > original.height {
        return Err(crate::error::IcerError::unsupported(format!(
            "region_mae: region {x0},{y0} {region_w}x{region_h} exceeds {}x{}",
            original.width, original.height
        )));
    }
    if region_w == 0 || region_h == 0 {
        return Ok(0.0);
    }
    let op = original.planes.first().ok_or_else(|| {
        crate::error::IcerError::unsupported("region_mae: original has no planes")
    })?;
    let dp = decoded
        .planes
        .first()
        .ok_or_else(|| crate::error::IcerError::unsupported("region_mae: decoded has no planes"))?;

    let x0 = x0 as usize;
    let x_end = x_end as usize;
    let mut abs_sum = 0.0f64;
    for y in y0 as usize..y_end as usize {
        let orow = &op.data[y * op.stride + x0..y * op.stride + x_end];
        let drow = &dp.data[y * dp.stride + x0..y * dp.stride + x_end];
        for (o, d) in orow.iter().zip(drow.iter()) {
            abs_sum += (*o as i32 - *d as i32).unsigned_abs() as f64;
        }
    }
    let n = (region_w as f64) * (region_h as f64);
    Ok(abs_sum / n)
}

/// Edge length of the sliding window used by [`ssim`]. The structural
/// similarity index is defined over local windows; an 8x8 block is the
/// common choice for an 8-bit grey image and keeps the per-window
/// statistics cheap (64 samples). Windows slide one pixel at a time and
/// the per-window scores are averaged into the global (mean) SSIM.
const SSIM_WINDOW: usize = 8;

/// Compute the mean structural-similarity index (SSIM) of the first
/// plane of `decoded` against `original`, returning a value in
/// `-1.0..=1.0` where `1.0` is a perfect (bit-identical) match.
///
/// SSIM is a perceptual image-quality measure that compares local
/// luminance, contrast, and structure rather than raw per-pixel error.
/// Two images with the same [`psnr_db`] can differ markedly in SSIM
/// when the error is structured (a shifted edge) versus diffuse
/// (uniform noise), so it complements the MSE-family metrics in
/// [`DistortionReport`].
///
/// This is a pure post-decode measurement over the reconstructed pixels:
/// it is spec-neutral and involves no wavelet, entropy-coder, or
/// framing machinery, so no ICER specification section is consulted.
///
/// Definition (8-bit luminance, dynamic range `L = 255`):
///
/// * For each `SSIM_WINDOW`x`SSIM_WINDOW` window the per-window score is
///   `((2 mu_x mu_y + C1)(2 cov_xy + C2)) / ((mu_x^2 + mu_y^2 + C1)(var_x + var_y + C2))`
///   where `mu` is the window mean, `var` the (population) variance,
///   `cov` the covariance, `C1 = (0.01 L)^2`, `C2 = (0.03 L)^2`.
/// * Windows slide one pixel at a time over every fully-contained
///   position; the returned value is the mean of the per-window scores.
///
/// Returns `1.0` when the planes are bit-identical. For images smaller
/// than one window in either axis, a single window covering the whole
/// (zero-padded-free) overlap is used; a zero-pixel image is treated as
/// a perfect match (`1.0`).
///
/// Returns [`crate::IcerError::Unsupported`] on a width/height mismatch or when
/// either image has no planes -- the same geometry contract as
/// [`DistortionReport::compare`].
pub fn ssim(original: &IcerImage, decoded: &IcerImage) -> Result<f64> {
    if original.width != decoded.width || original.height != decoded.height {
        return Err(crate::error::IcerError::unsupported(format!(
            "ssim: geometry mismatch {}x{} vs {}x{}",
            original.width, original.height, decoded.width, decoded.height
        )));
    }
    let op = original
        .planes
        .first()
        .ok_or_else(|| crate::error::IcerError::unsupported("ssim: original has no planes"))?;
    let dp = decoded
        .planes
        .first()
        .ok_or_else(|| crate::error::IcerError::unsupported("ssim: decoded has no planes"))?;

    let w = original.width as usize;
    let h = original.height as usize;
    if w == 0 || h == 0 {
        return Ok(1.0);
    }

    // Stabilisation constants from the standard SSIM definition for an
    // 8-bit dynamic range (L = 255): C1 = (0.01 L)^2, C2 = (0.03 L)^2.
    const L: f64 = 255.0;
    let c1 = (0.01 * L) * (0.01 * L);
    let c2 = (0.03 * L) * (0.03 * L);

    // Window edge clamped to the image extent so sub-window images use a
    // single full-image window rather than producing no scores at all.
    let win_w = SSIM_WINDOW.min(w);
    let win_h = SSIM_WINDOW.min(h);
    let n_win = (win_w * win_h) as f64;

    let mut score_sum = 0.0f64;
    let mut score_count = 0u64;

    // Slide the window one pixel at a time over every fully-contained
    // position. `y_end` / `x_end` are the inclusive-start bounds.
    let y_last = h - win_h;
    let x_last = w - win_w;
    for wy in 0..=y_last {
        for wx in 0..=x_last {
            let mut sum_x = 0.0f64;
            let mut sum_y = 0.0f64;
            let mut sum_xx = 0.0f64;
            let mut sum_yy = 0.0f64;
            let mut sum_xy = 0.0f64;
            for dy in 0..win_h {
                let orow = &op.data[(wy + dy) * op.stride..];
                let drow = &dp.data[(wy + dy) * dp.stride..];
                for dx in 0..win_w {
                    let x = orow[wx + dx] as f64;
                    let y = drow[wx + dx] as f64;
                    sum_x += x;
                    sum_y += y;
                    sum_xx += x * x;
                    sum_yy += y * y;
                    sum_xy += x * y;
                }
            }
            let mu_x = sum_x / n_win;
            let mu_y = sum_y / n_win;
            // Population variance / covariance (divide by N, not N-1):
            // SSIM's reference definition uses the biased estimator.
            let var_x = (sum_xx / n_win) - mu_x * mu_x;
            let var_y = (sum_yy / n_win) - mu_y * mu_y;
            let cov_xy = (sum_xy / n_win) - mu_x * mu_y;

            let numerator = (2.0 * mu_x * mu_y + c1) * (2.0 * cov_xy + c2);
            let denominator = (mu_x * mu_x + mu_y * mu_y + c1) * (var_x + var_y + c2);
            score_sum += numerator / denominator;
            score_count += 1;
        }
    }

    if score_count == 0 {
        // Defensive: the window clamp guarantees at least one position,
        // so this is unreachable, but avoid a divide-by-zero regardless.
        return Ok(1.0);
    }
    Ok(score_sum / score_count as f64)
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

    #[test]
    fn ssim_identical_is_one() {
        let img = ramp_image(32, 32);
        let s = ssim(&img, &img).unwrap();
        assert!(
            (s - 1.0).abs() < 1e-9,
            "identical ssim should be 1.0, got {s}"
        );
    }

    #[test]
    fn ssim_flat_identical_is_one() {
        let img = flat_image(16, 16, 100);
        let s = ssim(&img, &img).unwrap();
        assert!((s - 1.0).abs() < 1e-9, "flat identical ssim, got {s}");
    }

    #[test]
    fn ssim_small_image_below_window_is_one() {
        // 4x4 image is smaller than the 8x8 window; the clamp collapses to
        // a single full-image window. Identical inputs still score 1.0.
        let img = ramp_image(4, 4);
        let s = ssim(&img, &img).unwrap();
        assert!((s - 1.0).abs() < 1e-9, "sub-window identical ssim, got {s}");
    }

    #[test]
    fn ssim_degraded_is_below_identical() {
        // A noisy copy of a ramp must score strictly below a perfect 1.0.
        let img = ramp_image(32, 32);
        let mut noisy = img.clone();
        let stride = noisy.planes[0].stride;
        // Deterministic LCG so the test is reproducible.
        let mut state: u32 = 0x1234_5678;
        for y in 0..32usize {
            for x in 0..32usize {
                state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                let noise = ((state >> 24) as i32 % 41) - 20; // -20..=20
                let v = img.planes[0].data[y * stride + x] as i32 + noise;
                noisy.planes[0].data[y * stride + x] = v.clamp(0, 255) as u8;
            }
        }
        let s = ssim(&img, &noisy).unwrap();
        assert!(s < 1.0, "noisy ssim should be < 1.0, got {s}");
        assert!(s > -1.0, "ssim should stay in range, got {s}");
    }

    #[test]
    fn ssim_more_noise_scores_lower() {
        // Monotonicity sanity: heavier corruption -> lower SSIM.
        let img = ramp_image(48, 48);
        let stride = img.planes[0].stride;
        let make_offset = |delta: i32| {
            let mut out = img.clone();
            for y in 0..48usize {
                for x in 0..48usize {
                    // Alternating +/- so the structure is disrupted, not a
                    // pure luminance shift (which SSIM partly tolerates).
                    let sign = if (x ^ y) & 1 == 0 { 1 } else { -1 };
                    let v = img.planes[0].data[y * stride + x] as i32 + sign * delta;
                    out.planes[0].data[y * stride + x] = v.clamp(0, 255) as u8;
                }
            }
            out
        };
        let light = ssim(&img, &make_offset(5)).unwrap();
        let heavy = ssim(&img, &make_offset(30)).unwrap();
        assert!(
            heavy < light,
            "heavier corruption should score lower: heavy {heavy} vs light {light}"
        );
    }

    #[test]
    fn ssim_geometry_mismatch_errs() {
        let a = ramp_image(16, 16);
        let b = ramp_image(16, 8);
        assert!(ssim(&a, &b).is_err());
    }

    #[test]
    fn ssim_zero_pixel_image_is_one() {
        let a = IcerImage::zeros(0, 0, IcerPixelFormat::Gray8);
        assert_eq!(ssim(&a, &a).unwrap(), 1.0);
    }
}
