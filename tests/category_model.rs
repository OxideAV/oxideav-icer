//! IPN 42-155 §III.B four-category pixel model coverage.
//!
//! ICER keeps a *category* for every pixel that counts the magnitude bits
//! already coded: category 0 = not yet significant; 1 = the first '1' bit
//! was coded (pixel just became significant); 2 = one more magnitude bit
//! coded; 3 = one more again, and stays 3 permanently. The magnitude-bit
//! context for a refinement bit is then category-1 -> context 9/10,
//! category-2 -> context 11, and category-3 bits are *left uncoded*
//! (sent at a fixed probability-of-zero of 1/2). These tests exercise the
//! scheme end-to-end through the public `encode_icer` / `parse_icer`
//! surface: the category transitions must run identically on both sides,
//! so a divergence would corrupt the decode and fail the round-trips
//! below.

use oxideav_icer::context::{magnitude_context, MagnitudeContext, CATEGORY2_CONTEXT};
use oxideav_icer::{
    encode_icer, parse_icer, EncodeOptions, IcerImage, IcerPixelFormat, WaveletFilter,
};

/// The category -> context mapping is exactly the §III.B rule:
/// cat 1 -> 9 (no H/V significant neighbour) or 10; cat 2 -> 11; cat 3+
/// -> uncoded.
#[test]
fn category_context_mapping_matches_spec() {
    assert_eq!(magnitude_context(1, false), MagnitudeContext::Coded(9));
    assert_eq!(magnitude_context(1, true), MagnitudeContext::Coded(10));
    assert_eq!(
        magnitude_context(2, false),
        MagnitudeContext::Coded(CATEGORY2_CONTEXT)
    );
    assert_eq!(
        magnitude_context(2, true),
        MagnitudeContext::Coded(CATEGORY2_CONTEXT)
    );
    assert_eq!(magnitude_context(3, false), MagnitudeContext::Uncoded);
    // Saturated category (an over-incremented counter) stays uncoded.
    assert_eq!(magnitude_context(5, true), MagnitudeContext::Uncoded);
    // Category 0 is never reached by the refinement pass (handled by the
    // significance pass), but the mapping treats it as uncoded too.
    assert_eq!(magnitude_context(0, true), MagnitudeContext::Uncoded);
}

fn texture_64x64() -> IcerImage {
    let (w, h) = (64u32, 64u32);
    let mut img = IcerImage::zeros(w, h, IcerPixelFormat::Gray8);
    let p = &mut img.planes[0];
    for y in 0..h as usize {
        for x in 0..w as usize {
            let v =
                ((x.wrapping_mul(29) ^ y.wrapping_mul(17)).wrapping_add(x * y / 7) & 0xFF) as u8;
            p.data[y * p.stride + x] = v;
        }
    }
    img
}

/// A high-dynamic-range fixture: large coefficients drive several pixels
/// all the way to category 3, so the uncoded-bit path is genuinely
/// exercised (category-3 bits both encoded and decoded). The full-quality
/// filter-Q round-trip must stay bit-exact through the uncoded path.
fn high_range_64x64() -> IcerImage {
    let (w, h) = (64u32, 64u32);
    let mut img = IcerImage::zeros(w, h, IcerPixelFormat::Gray8);
    let p = &mut img.planes[0];
    for y in 0..h as usize {
        for x in 0..w as usize {
            // Strong ramps + an offset checker -> coefficients that span
            // many magnitude bit-planes and refine repeatedly.
            let ramp = (x * 4 + y * 3) & 0xFF;
            let checker = if (x / 4 + y / 4) & 1 == 0 { 0 } else { 80 };
            p.data[y * p.stride + x] = (ramp ^ checker) as u8;
        }
    }
    img
}

/// Full-quality filter-Q (lossless integer 5/3) must reconstruct
/// bit-exactly even though most coefficients pass through the category-3
/// uncoded-bit path: the uncoded bits are still coded at a fixed P=1/2
/// and decode losslessly.
#[test]
fn full_quality_filter_q_bit_exact_through_category3() {
    for img in [texture_64x64(), high_range_64x64()] {
        let mut opts = EncodeOptions::compressed();
        opts.filter = WaveletFilter::FilterQ;
        opts.wavelet_levels = 3;
        let bytes = encode_icer(&img, &opts).expect("encode");
        let dec = parse_icer(&bytes).expect("decode");
        assert_eq!(
            dec.planes[0].data, img.planes[0].data,
            "filter-Q full-quality decode must be bit-exact through the category-3 uncoded path"
        );
    }
}

/// Progressive truncation still decodes cleanly and PSNR rises with the
/// budget under the new context model. (The four-category scheme changes
/// packet byte sizes; this guards the end-to-end progressive contract.)
#[test]
fn progressive_truncation_monotone_under_category_model() {
    let img = high_range_64x64();
    let mut prev = 0.0f64;
    for budget in [256u64, 512, 1024, 2048, 4096, 8192] {
        let mut opts = EncodeOptions::compressed().with_byte_budget(budget);
        opts.filter = WaveletFilter::FilterQ;
        opts.wavelet_levels = 3;
        let bytes = encode_icer(&img, &opts).expect("encode");
        assert!(bytes.len() as u64 <= budget);
        let dec = parse_icer(&bytes).expect("decode");
        let n = (img.width * img.height) as f64;
        let mse: f64 = img.planes[0]
            .data
            .iter()
            .zip(dec.planes[0].data.iter())
            .map(|(&a, &b)| {
                let d = a as f64 - b as f64;
                d * d
            })
            .sum::<f64>()
            / n;
        let psnr = if mse == 0.0 {
            f64::INFINITY
        } else {
            10.0 * (255.0f64 * 255.0 / mse).log10()
        };
        assert!(
            psnr >= prev - 0.1,
            "budget {budget}: PSNR {psnr:.2} dB regressed from {prev:.2} dB"
        );
        prev = psnr;
    }
}

/// Colour (YUV 4:4:4) round-trip still bit-exact for filter Q with the
/// category model active on every plane independently.
#[test]
fn colour_filter_q_bit_exact_under_category_model() {
    let (w, h) = (32u32, 32u32);
    let mut img = IcerImage::zeros(w, h, IcerPixelFormat::Yuv444P);
    for (pi, plane) in img.planes.iter_mut().enumerate() {
        for y in 0..h as usize {
            for x in 0..w as usize {
                let v = ((x * 7 + y * 5 + pi * 33) & 0xFF) as u8;
                plane.data[y * plane.stride + x] = v;
            }
        }
    }
    let mut opts = EncodeOptions::compressed();
    opts.filter = WaveletFilter::FilterQ;
    let bytes = encode_icer(&img, &opts).expect("encode");
    let dec = parse_icer(&bytes).expect("decode");
    assert_eq!(dec.pixel_format, IcerPixelFormat::Yuv444P);
    for p in 0..3 {
        assert_eq!(
            dec.planes[p].data, img.planes[p].data,
            "colour plane {p} must round-trip bit-exact under the category model"
        );
    }
}
