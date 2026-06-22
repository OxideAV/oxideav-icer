//! Decode-configuration coverage: the compressed-segment decoder
//! (multi-packet arithmetic coder + per-coefficient §III.A deadzone +
//! inverse DWT) must reconstruct correctly across the full matrix of
//! decode configurations the wire format admits, and progressive
//! truncation must improve quality monotonically in every cell.
//!
//! Three axes are swept:
//!   * **geometry** -- including odd / non-power-of-two strips and very
//!     thin (`5xN`, `Nx5`) strips that exercise the DWT's `div_ceil`
//!     subband boundaries;
//!   * **decomposition levels** -- 1..=5 (past depth ~3 the LL subband
//!     stops shrinking on small images, so the deeper levels are no-ops
//!     that the decoder must still reproduce exactly);
//!   * **filter** -- the reversible integer 5/3 (filter Q) path is
//!     lossless and must be bit-exact; the float 9/7 (filter A) path is
//!     lossy but must stay within a bounded error.
//!
//! This complements `compressed_roundtrip.rs` (which pins a couple of
//! fixed configs) with a systematic matrix, and `truncation_fidelity.rs`
//! (one fixture) with progressive monotonicity across the whole matrix.

use oxideav_icer::{
    encode_icer, parse_icer, EncodeOptions, IcerImage, IcerPixelFormat, WaveletFilter,
};

fn ramp(w: u32, h: u32) -> IcerImage {
    let mut img = IcerImage::zeros(w, h, IcerPixelFormat::Gray8);
    let s = img.planes[0].stride;
    for y in 0..h as usize {
        for x in 0..w as usize {
            img.planes[0].data[y * s + x] = ((x * 3 + y * 5) & 0xFF) as u8;
        }
    }
    img
}

fn mse(a: &IcerImage, b: &IcerImage) -> f64 {
    let n = (a.width * a.height) as f64;
    a.planes[0]
        .data
        .iter()
        .zip(b.planes[0].data.iter())
        .map(|(&x, &y)| {
            let d = x as f64 - y as f64;
            d * d
        })
        .sum::<f64>()
        / n
}

fn psnr(a: &IcerImage, b: &IcerImage) -> f64 {
    let m = mse(a, b);
    if m == 0.0 {
        f64::INFINITY
    } else {
        10.0 * (255.0f64 * 255.0 / m).log10()
    }
}

const GEOMETRIES: &[(u32, u32)] = &[(17, 13), (31, 31), (64, 64), (5, 200), (200, 5)];

/// Filter-Q (lossless integer 5/3) full-quality decode is bit-exact for
/// every geometry x decomposition-level cell — including odd and very
/// thin strips, and decomposition depths that exceed the natural pyramid
/// of the strip.
#[test]
fn filter_q_full_quality_bit_exact_matrix() {
    for &(w, h) in GEOMETRIES {
        for levels in 1u8..=5 {
            let img = ramp(w, h);
            let mut opts = EncodeOptions::compressed();
            opts.filter = WaveletFilter::Reversible53;
            opts.wavelet_levels = levels;
            let bytes = encode_icer(&img, &opts)
                .unwrap_or_else(|e| panic!("encode {w}x{h} L{levels}: {e:?}"));
            let dec =
                parse_icer(&bytes).unwrap_or_else(|e| panic!("decode {w}x{h} L{levels}: {e:?}"));
            assert_eq!(dec.width, w);
            assert_eq!(dec.height, h);
            assert_eq!(
                dec.planes[0].data, img.planes[0].data,
                "filter-Q full-quality {w}x{h} L{levels} must be bit-exact"
            );
        }
    }
}

/// Progressive truncation improves (or holds) PSNR monotonically as the
/// budget grows, for every geometry x decomposition-level cell on the
/// lossless filter-Q path. Guards the §III.A per-coefficient deadzone +
/// the category-aware context model end to end across configs.
#[test]
fn filter_q_progressive_monotone_matrix() {
    for &(w, h) in GEOMETRIES {
        for levels in [1u8, 3, 5] {
            let img = ramp(w, h);
            let mut prev = -1.0f64;
            for budget in [128u64, 256, 512, 1024, 2048, 4096] {
                let mut opts = EncodeOptions::compressed().with_byte_budget(budget);
                opts.filter = WaveletFilter::Reversible53;
                opts.wavelet_levels = levels;
                let bytes = encode_icer(&img, &opts).expect("encode");
                assert!(
                    bytes.len() as u64 <= budget,
                    "{w}x{h} L{levels} b={budget}: output {} exceeds cap",
                    bytes.len()
                );
                let dec = parse_icer(&bytes).expect("decode");
                let p = psnr(&img, &dec);
                assert!(
                    p >= prev - 0.1,
                    "{w}x{h} L{levels} b={budget}: PSNR {p:.2} regressed from {prev:.2}"
                );
                prev = p;
            }
        }
    }
}

/// The lossy float 9/7 (filter A) decode path reconstructs within a
/// bounded error across geometries and levels (it is lossy by the float
/// lifting + integer-rounding round-trip, but must not blow up). A wide
/// `>= 20 dB` floor on the structured ramp guards against a decode-config
/// regression that would corrupt the inverse float DWT for a particular
/// strip shape.
#[test]
fn filter_a_decode_bounded_error_matrix() {
    for &(w, h) in GEOMETRIES {
        // Float DWT needs width/height >= 2; the 5x200 / 200x5 strips are
        // fine (both dims >= 2). Skip nothing here.
        for levels in [1u8, 2, 3] {
            let img = ramp(w, h);
            let mut opts = EncodeOptions::compressed();
            opts.filter = WaveletFilter::NineSevenA;
            opts.wavelet_levels = levels;
            let bytes = encode_icer(&img, &opts)
                .unwrap_or_else(|e| panic!("encode A {w}x{h} L{levels}: {e:?}"));
            let dec =
                parse_icer(&bytes).unwrap_or_else(|e| panic!("decode A {w}x{h} L{levels}: {e:?}"));
            assert_eq!(dec.width, w);
            assert_eq!(dec.height, h);
            let p = psnr(&img, &dec);
            assert!(
                p >= 20.0,
                "filter-A full-quality {w}x{h} L{levels}: PSNR {p:.2} dB below 20 dB floor"
            );
        }
    }
}

/// A non-natural bit-plane count (the encoder may emit more bit-plane
/// pairs than a strip's coefficients strictly need) still decodes
/// bit-exactly: the surplus near-empty packets carry no magnitude bits
/// and must be reproduced as no-ops by the decoder.
#[test]
fn oversized_bit_plane_count_decodes_bit_exact() {
    let img = ramp(64, 64);
    for q in [8u8, 12, 16] {
        let mut opts = EncodeOptions::compressed();
        opts.filter = WaveletFilter::Reversible53;
        opts.wavelet_levels = 3;
        opts.bit_plane_count = q;
        let bytes = encode_icer(&img, &opts).expect("encode");
        let dec = parse_icer(&bytes).expect("decode");
        assert_eq!(
            dec.planes[0].data, img.planes[0].data,
            "filter-Q with bit_plane_count={q} must be bit-exact"
        );
    }
}
