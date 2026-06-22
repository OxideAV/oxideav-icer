//! End-to-end fidelity of budget-truncated decodes (IPN 42-155 §III.A
//! per-coefficient deadzone reconstruction).
//!
//! The unit tests in `src/bitplane.rs` pin the per-coefficient deadzone
//! arithmetic at the packet level. These tests exercise the same
//! reconstruction through the *public* `encode_icer` / `parse_icer`
//! budget path -- the path the deployed Mars-rover pipeline drives -- and
//! lock the truncated-stream PSNR so a regression to the older
//! strip-global single-`b` reconstruction would fail CI.
//!
//! A byte-budget cut routinely lands between a bit plane's significance
//! and refinement packets (they are separate, MSB-down packets):
//! `sig(bp)` survives, `ref(bp)` is dropped. The per-coefficient deadzone
//! reconstructs the already-significant coefficients (whose plane-`bp`
//! refinement bit never arrived) at the wider `b = bp + 1` bin while the
//! newly-significant ones use `b = bp`; the strip-global approximation
//! used one shared `b` and under-reconstructed the former class. The
//! mid-plane cuts below clear PSNR floors set ~1 dB above the strip-global
//! result they previously produced.

use oxideav_icer::{
    encode_icer, parse_icer, EncodeOptions, IcerImage, IcerPixelFormat, WaveletFilter,
};

/// A deterministic textured 64×64 fixture with structure across many
/// bit planes so a budget cut falls mid-plane rather than on a clean
/// boundary.
fn texture_64x64() -> IcerImage {
    let (w, h) = (64u32, 64u32);
    let mut img = IcerImage::zeros(w, h, IcerPixelFormat::Gray8);
    let plane = &mut img.planes[0];
    for y in 0..h as usize {
        for x in 0..w as usize {
            let v =
                ((x.wrapping_mul(29) ^ y.wrapping_mul(17)).wrapping_add(x * y / 7) & 0xFF) as u8;
            plane.data[y * plane.stride + x] = v;
        }
    }
    img
}

fn psnr(a: &IcerImage, b: &IcerImage) -> f64 {
    let n = (a.width * a.height) as f64;
    let mse: f64 = a.planes[0]
        .data
        .iter()
        .zip(b.planes[0].data.iter())
        .map(|(&x, &y)| {
            let d = x as f64 - y as f64;
            d * d
        })
        .sum::<f64>()
        / n;
    if mse == 0.0 {
        f64::INFINITY
    } else {
        10.0 * (255.0f64 * 255.0 / mse).log10()
    }
}

fn encode_at(img: &IcerImage, budget: u64) -> Vec<u8> {
    let mut opts = EncodeOptions::compressed().with_byte_budget(budget);
    opts.filter = WaveletFilter::Reversible53;
    opts.wavelet_levels = 3;
    encode_icer(img, &opts).expect("encode failed")
}

/// Mid-plane budget cuts clear PSNR floors set above the strip-global
/// reconstruction. The floors are ~1 dB below the measured per-coefficient
/// PSNR and ~0.5..2 dB *above* the strip-global PSNR for the same cut, so
/// a regression to strip-global fails here while leaving slack for
/// arithmetic-coder tweaks.
#[test]
fn mid_plane_truncation_clears_per_coefficient_floor() {
    let img = texture_64x64();
    // (requested budget, PSNR floor in dB). Floors sit between the
    // strip-global and per-coefficient measurements at each cut.
    let cases: &[(u64, f64)] = &[
        (1800, 28.0), // strip-global 27.0, per-coef 28.4
        (2400, 33.0), // strip-global 31.6, per-coef 33.7
        (2800, 39.0), // strip-global 36.6, per-coef 39.3
        (3500, 44.0), // strip-global 41.2, per-coef 44.2
    ];
    for &(budget, floor) in cases {
        let bytes = encode_at(&img, budget);
        assert!(
            bytes.len() as u64 <= budget,
            "budget {budget}: output {} exceeds cap",
            bytes.len()
        );
        let dec = parse_icer(&bytes).expect("decode failed");
        let p = psnr(&img, &dec);
        eprintln!(
            "budget {budget}: {} bytes -> {p:.3} dB (floor {floor})",
            bytes.len()
        );
        assert!(
            p >= floor,
            "budget {budget}: PSNR {p:.3} dB below per-coefficient floor {floor} dB \
             (a regression to strip-global deadzone reconstruction)"
        );
    }
}

/// PSNR is non-decreasing as the byte budget grows, end to end.
#[test]
fn truncated_psnr_is_monotone_in_budget() {
    let img = texture_64x64();
    let mut prev = 0.0f64;
    for budget in [512u64, 768, 1024, 1536, 2048, 3072, 4096] {
        let bytes = encode_at(&img, budget);
        let dec = parse_icer(&bytes).expect("decode failed");
        let p = psnr(&img, &dec);
        assert!(
            p >= prev - 0.1,
            "budget {budget}: PSNR {p:.3} dB regressed from {prev:.3} dB"
        );
        prev = p;
    }
}

/// A 64×64 checkerboard: the canonical high-frequency fixture. The
/// IPN 42-155 §III.B four-category context model (category-3 magnitude
/// bits left uncoded; category-1/2 coded against contexts 9/10/11) makes
/// the *strict-MSB* budget-truncated decode markedly better than the
/// pre-r359 model on this content. These floors pin that improvement so a
/// regression in the context model fails CI.
#[test]
fn checkerboard_strict_truncation_clears_category_model_floor() {
    let (w, h) = (64u32, 64u32);
    let mut img = IcerImage::zeros(w, h, IcerPixelFormat::Gray8);
    let stride = img.planes[0].stride;
    for y in 0..h as usize {
        for x in 0..w as usize {
            img.planes[0].data[y * stride + x] = if (x ^ y) & 1 == 0 { 0 } else { 255 };
        }
    }
    // (budget, PSNR floor). Floors are set ~0.5 dB below the measured
    // r359 strict-MSB result, which is itself several dB above the
    // pre-r359 model on this fixture.
    let cases: &[(u64, f64)] = &[(400, 24.5), (600, 30.5), (800, 37.0)];
    for &(budget, floor) in cases {
        let bytes = encode_at(&img, budget);
        assert!(bytes.len() as u64 <= budget);
        let dec = parse_icer(&bytes).expect("decode failed");
        let p = psnr(&img, &dec);
        eprintln!(
            "checkerboard strict b={budget}: {} bytes -> {p:.3} dB (floor {floor})",
            bytes.len()
        );
        assert!(
            p >= floor,
            "checkerboard strict b={budget}: PSNR {p:.3} dB below the §III.B category-model \
             floor {floor} dB"
        );
    }
}

/// An untruncated (generous-budget) filter-Q encode of an integer image
/// still reconstructs bit-exactly: the per-coefficient deadzone offset is
/// zero for every coefficient when every plane is delivered (`b = 0`).
#[test]
fn untruncated_filter_q_is_bit_exact() {
    let img = texture_64x64();
    let mut opts = EncodeOptions::compressed();
    opts.filter = WaveletFilter::Reversible53;
    opts.wavelet_levels = 3;
    let bytes = encode_icer(&img, &opts).expect("encode failed");
    let dec = parse_icer(&bytes).expect("decode failed");
    assert_eq!(
        dec.planes[0].data, img.planes[0].data,
        "untruncated filter-Q decode must be bit-exact"
    );
}
