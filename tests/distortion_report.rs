//! Structured distortion-report tests (round 272).
//!
//! `analyze::psnr_db` reports a single number and panics on a shape
//! mismatch. Round 272 adds:
//!
//!   * [`DistortionReport::compare`] -- every common distortion metric
//!     (MSE / RMSE / MAE / max-abs-error / PSNR) in one pass, returning
//!     `Err` (not a panic) on a geometry contradiction.
//!   * [`region_mae`] -- the mean absolute error over a rectangular
//!     sub-region, the programmatic form of the centre-band vs.
//!     periphery MAE comparison the round-6 ROI feature documents in
//!     prose only.
//!
//! Both are pure post-decode image-quality measurements: spec-neutral,
//! no wavelet / entropy machinery involved.

use oxideav_icer::{
    encode_icer, parse_icer, region_mae, ssim, DistortionReport, EncodeOptions, IcerImage,
    IcerPixelFormat, WaveletFilter,
};

/// Diagonal ramp identical to the round-trip tests' fixtures.
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

/// Apply a constant per-pixel offset to a clone of `img`, clamped to
/// `0..=255`. Lets a test fabricate a decoded image with a known,
/// exact distortion relative to the original.
fn shifted(img: &IcerImage, delta: i32) -> IcerImage {
    let mut out = img.clone();
    let stride = out.planes[0].stride;
    let w = out.width as usize;
    let h = out.height as usize;
    for y in 0..h {
        for x in 0..w {
            let v = img.planes[0].data[y * stride + x] as i32 + delta;
            out.planes[0].data[y * stride + x] = v.clamp(0, 255) as u8;
        }
    }
    out
}

#[test]
fn distortion_report_bit_identical_is_lossless() {
    let img = ramp_image(32, 32);
    let report = DistortionReport::compare(&img, &img).unwrap();
    assert_eq!(report.mse, 0.0);
    assert_eq!(report.rmse, 0.0);
    assert_eq!(report.mae, 0.0);
    assert_eq!(report.max_abs_error, 0);
    assert!(report.psnr_db.is_infinite());
}

#[test]
fn distortion_report_constant_offset_metrics_are_exact() {
    // A ramp where every pixel sits in 3..=255-ish so a +3 offset never
    // clamps: the whole interior of a 32x32 ramp is in range for delta 3.
    let original = ramp_image(64, 64);
    // Guard: pick a delta small enough that clamping is rare. We compute
    // the expected metrics directly from the (possibly-clamped) shifted
    // image so the assertion holds regardless of edge clamping.
    let delta = 3;
    let decoded = shifted(&original, delta);

    // Reference computation straight from the two planes.
    let stride_o = original.planes[0].stride;
    let stride_d = decoded.planes[0].stride;
    let w = original.width as usize;
    let h = original.height as usize;
    let mut sq = 0.0f64;
    let mut ab = 0.0f64;
    let mut mx = 0u8;
    for y in 0..h {
        for x in 0..w {
            let o = original.planes[0].data[y * stride_o + x] as i32;
            let d = decoded.planes[0].data[y * stride_d + x] as i32;
            let a = (o - d).unsigned_abs() as u8;
            if a > mx {
                mx = a;
            }
            sq += (a as f64) * (a as f64);
            ab += a as f64;
        }
    }
    let n = (w * h) as f64;
    let exp_mse = sq / n;

    let report = DistortionReport::compare(&original, &decoded).unwrap();
    assert!(
        (report.mse - exp_mse).abs() < 1e-9,
        "mse {} vs {exp_mse}",
        report.mse
    );
    assert!((report.rmse - exp_mse.sqrt()).abs() < 1e-9);
    assert!((report.mae - ab / n).abs() < 1e-9);
    assert_eq!(report.max_abs_error, mx);
    // PSNR matches the standalone helper.
    let psnr = oxideav_icer::psnr_db(&original, &decoded);
    assert!((report.psnr_db - psnr).abs() < 1e-3);
}

#[test]
fn distortion_report_geometry_mismatch_errs_not_panics() {
    let a = ramp_image(16, 16);
    let b = ramp_image(16, 8);
    let err = DistortionReport::compare(&a, &b).unwrap_err();
    // Non-panicking complement to psnr_db (which would assert).
    let msg = format!("{err}");
    assert!(msg.contains("geometry mismatch"), "got: {msg}");
}

#[test]
fn distortion_report_zero_pixel_image_is_lossless() {
    let a = IcerImage::zeros(0, 0, IcerPixelFormat::Gray8);
    let report = DistortionReport::compare(&a, &a).unwrap();
    assert_eq!(report.mse, 0.0);
    assert!(report.psnr_db.is_infinite());
}

#[test]
fn region_mae_subregion_isolates_local_error() {
    // Build an original and a decoded copy that is perfect everywhere
    // except a known 8x8 patch at (4, 4) where every pixel is off by 10.
    let original = ramp_image(32, 32);
    let mut decoded = original.clone();
    let stride = decoded.planes[0].stride;
    for y in 4..12 {
        for x in 4..12 {
            let v = original.planes[0].data[y * stride + x] as i32 + 10;
            decoded.planes[0].data[y * stride + x] = v.clamp(0, 255) as u8;
        }
    }

    // The patch region's MAE is ~10 (modulo any clamping); the whole
    // frame's MAE is far smaller (only 64 of 1024 pixels differ).
    let patch_mae = region_mae(&original, &decoded, 4, 4, 8, 8).unwrap();
    assert!(
        patch_mae > 9.0 && patch_mae <= 10.0,
        "patch mae {patch_mae}"
    );

    // A region entirely outside the patch is lossless.
    let clean_mae = region_mae(&original, &decoded, 16, 16, 8, 8).unwrap();
    assert_eq!(clean_mae, 0.0);

    // Full-frame region MAE equals the DistortionReport MAE.
    let full = region_mae(&original, &decoded, 0, 0, 32, 32).unwrap();
    let report = DistortionReport::compare(&original, &decoded).unwrap();
    assert!((full - report.mae).abs() < 1e-9);
}

#[test]
fn region_mae_out_of_bounds_errs() {
    let original = ramp_image(16, 16);
    let decoded = original.clone();
    // Region spills past the right edge.
    assert!(region_mae(&original, &decoded, 10, 0, 10, 4).is_err());
    // Region spills past the bottom edge.
    assert!(region_mae(&original, &decoded, 0, 10, 4, 10).is_err());
    // u32 overflow on the x extent is rejected, not wrapped.
    assert!(region_mae(&original, &decoded, u32::MAX, 0, 1, 1).is_err());
    // Zero-area region is a trivial 0.0.
    assert_eq!(region_mae(&original, &decoded, 0, 0, 0, 4).unwrap(), 0.0);
}

#[test]
fn ssim_lossless_roundtrip_is_perfect() {
    // Filter Q (reversible integer 5/3) round-trips bit-exactly, so the
    // decoded image is identical to the original and SSIM is 1.0.
    let img = ramp_image(48, 48);
    let opts = EncodeOptions {
        filter: WaveletFilter::FilterQ,
        ..EncodeOptions::compressed()
    };
    let bytes = encode_icer(&img, &opts).unwrap();
    let decoded = parse_icer(&bytes).unwrap();
    let s = ssim(&img, &decoded).unwrap();
    assert!(
        (s - 1.0).abs() < 1e-9,
        "lossless ssim should be 1.0, got {s}"
    );
}

#[test]
fn ssim_drops_under_tight_byte_budget() {
    // A tight byte budget truncates the progressive stream, so the
    // decoded image is degraded and SSIM falls below the perfect 1.0
    // while staying in the valid range.
    let w = 64u32;
    let h = 64u32;
    let mut img = IcerImage::zeros(w, h, IcerPixelFormat::Gray8);
    let stride = img.planes[0].stride;
    for y in 0..h as usize {
        for x in 0..w as usize {
            img.planes[0].data[y * stride + x] = (((x as u32 + y as u32) * 7) & 0xFF) as u8;
        }
    }
    let opts = EncodeOptions::compressed().with_byte_budget(120);
    let bytes = encode_icer(&img, &opts).unwrap();
    assert!(bytes.len() <= 120);
    let decoded = parse_icer(&bytes).unwrap();
    let s = ssim(&img, &decoded).unwrap();
    assert!(
        s < 1.0 && s > -1.0,
        "truncated-stream ssim should be a degraded in-range value, got {s}"
    );
}

#[test]
fn region_mae_centre_beats_periphery_under_roi_budget() {
    // End-to-end: a centre-ROI encode under a tight byte budget should
    // keep the centre strip's fidelity higher (lower MAE) than the
    // periphery. This is the programmatic version of the README's
    // round-6 "centre band MAE vs periphery MAE" table.
    let w = 64u32;
    let h = 128u32;
    let mut img = IcerImage::zeros(w, h, IcerPixelFormat::Gray8);
    let stride = img.planes[0].stride;
    for y in 0..h as usize {
        for x in 0..w as usize {
            // Non-trivial content everywhere so truncation has something
            // to lose in every strip.
            img.planes[0].data[y * stride + x] = (((x as u32 + y as u32) * 7) & 0xFF) as u8;
        }
    }

    let opts = EncodeOptions {
        segment_count: 4,
        ..EncodeOptions::compressed()
    }
    .with_byte_budget(260)
    .with_center_roi();
    let bytes = encode_icer(&img, &opts).unwrap();
    assert!(bytes.len() <= 260);
    let decoded = parse_icer(&bytes).unwrap();

    // Centre band: middle 64 rows (32..96). Periphery: top + bottom 32.
    let centre = region_mae(&img, &decoded, 0, 32, w, 64).unwrap();
    let top = region_mae(&img, &decoded, 0, 0, w, 32).unwrap();
    let bottom = region_mae(&img, &decoded, 0, 96, w, 32).unwrap();
    let periphery = (top + bottom) / 2.0;

    assert!(
        centre < periphery,
        "centre MAE {centre} should be lower than periphery MAE {periphery}"
    );
}
