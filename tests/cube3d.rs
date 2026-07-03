//! ICER-3D end-to-end integration coverage (IPN 42-164).
//!
//! Complements the `cube` / `bitplane3d` unit tests with the paper's
//! headline claims measured on this implementation: exploiting the
//! spectral dimension must beat running 2-D ICER on each band
//! independently (§I / §V), the §IV.B controls must be progressive, and
//! the decoder must survive arbitrary corruption of a valid stream.

use oxideav_icer::{
    encode_icer, encode_icer3d, parse_icer3d, CubeEncodeOptions, EncodeOptions, IcerCube,
    IcerImage, IcerPixelFormat,
};

/// A synthetic scene with strong inter-band correlation: every band is
/// the same spatial scene under a band-dependent gain + signal-level
/// offset (the §III.A "systematic variations in signal level of
/// different spectral bands"), plus band-local detail. 8-bit so each
/// band is also encodable by the 2-D Gray8 path for the comparison
/// test.
fn correlated_cube(w: u32, h: u32, bands: u32) -> IcerCube {
    let mut cube = IcerCube::zeros(w, h, bands, 8);
    let (wu, hu) = (w as usize, h as usize);
    for b in 0..bands as usize {
        let offset = ((b * 11) % 60) as i32;
        for y in 0..hu {
            for x in 0..wu {
                let scene =
                    ((x * 5 + y * 3) % 97) as i32 + if (x / 3 + y / 5) % 4 == 0 { 40 } else { 0 };
                let detail = ((x * y + b) % 7) as i32;
                cube.samples[b * wu * hu + y * wu + x] =
                    (60 + offset + scene + detail).clamp(0, 255) as u16;
            }
        }
    }
    cube
}

fn mse(a: &IcerCube, b: &IcerCube) -> f64 {
    a.samples
        .iter()
        .zip(&b.samples)
        .map(|(&x, &y)| ((x as f64) - (y as f64)).powi(2))
        .sum::<f64>()
        / a.samples.len() as f64
}

#[test]
fn cube_coding_beats_per_band_2d_lossless() {
    // IPN 42-164 §I: "Exploiting dependencies in all three dimensions
    // of hyperspectral data sets promises substantially more effective
    // compression than two-dimensional (2-D) approaches such as
    // applying conventional image compression to each spectral band
    // independently." §V.A Table 7 makes the same point for lossless
    // rates (ICER-3D 5.35 vs 2-D per-band ICER 7.19 bits/sample on the
    // AVIRIS average). Verify the direction of that inequality on this
    // implementation: the lossless cube encode must be smaller than the
    // sum of lossless per-band 2-D encodes of the same data.
    let cube = correlated_cube(32, 32, 16);
    let cube_bytes = encode_icer3d(&cube, &CubeEncodeOptions::default()).unwrap();

    let (wu, hu) = (cube.width as usize, cube.height as usize);
    let mut band_sum = 0usize;
    for b in 0..cube.bands as usize {
        let mut img = IcerImage::zeros(cube.width, cube.height, IcerPixelFormat::Gray8);
        let stride = img.planes[0].stride;
        for y in 0..hu {
            for x in 0..wu {
                img.planes[0].data[y * stride + x] = cube.samples[b * wu * hu + y * wu + x] as u8;
            }
        }
        // 2-D lossless: compressed path, filter Q (the reversible
        // default), same 3-level decomposition depth.
        let opts = EncodeOptions {
            wavelet_levels: 3,
            ..EncodeOptions::compressed()
        };
        band_sum += encode_icer(&img, &opts).unwrap().len();
    }

    assert!(
        cube_bytes.len() < band_sum,
        "3-D coding must beat per-band 2-D: cube {} bytes vs per-band sum {} bytes",
        cube_bytes.len(),
        band_sum
    );
    // And it must still be lossless.
    assert_eq!(parse_icer3d(&cube_bytes).unwrap(), cube);
}

#[test]
fn quota_and_min_loss_compose() {
    // §IV.B: compression stops "once the quality goal (as expressed by
    // the minimum loss value) or byte quota is met, whichever comes
    // first". With a generous quota the min-loss cut dominates; with a
    // tight quota the quota dominates.
    let cube = correlated_cube(24, 24, 8);
    let min_loss_only =
        encode_icer3d(&cube, &CubeEncodeOptions::default().with_min_loss(6)).unwrap();
    let both_generous = encode_icer3d(
        &cube,
        &CubeEncodeOptions::default()
            .with_min_loss(6)
            .with_byte_quota(1_000_000),
    )
    .unwrap();
    assert_eq!(
        min_loss_only, both_generous,
        "a non-binding quota must not change the min-loss emission"
    );

    let tight = 600u64;
    let both_tight = encode_icer3d(
        &cube,
        &CubeEncodeOptions::default()
            .with_min_loss(6)
            .with_byte_quota(tight),
    )
    .unwrap();
    assert!(both_tight.len() as u64 <= tight);
    assert!(both_tight.len() < min_loss_only.len());
}

#[test]
fn multi_segment_quota_reconstructs_full_geometry() {
    // A tight quota on a multi-segment cube still frames every strip:
    // geometry is preserved and per-strip quality degrades instead.
    let cube = correlated_cube(16, 24, 6);
    let opts = CubeEncodeOptions::default()
        .with_segment_count(3)
        .with_byte_quota(700);
    let bytes = encode_icer3d(&cube, &opts).unwrap();
    assert!(bytes.len() as u64 <= 700);
    let decoded = parse_icer3d(&bytes).unwrap();
    assert_eq!(decoded.width, cube.width);
    assert_eq!(decoded.height, cube.height);
    assert_eq!(decoded.bands, cube.bands);
    // Quality should improve monotonically as the quota is relaxed.
    let mut last = f64::INFINITY;
    for quota in [700u64, 1400, 2800, 20_000] {
        let opts = CubeEncodeOptions::default()
            .with_segment_count(3)
            .with_byte_quota(quota);
        let decoded = parse_icer3d(&encode_icer3d(&cube, &opts).unwrap()).unwrap();
        let e = mse(&cube, &decoded);
        assert!(e <= last + 1e-9, "quality fell at quota {quota}");
        last = e;
    }
    assert_eq!(last, 0.0);
}

#[test]
fn interleaved_backend_matches_arithmetic_losslessness() {
    // Both entropy backends must be lossless at min-loss 0 and both
    // must honour a byte quota; their byte counts may differ.
    let cube = correlated_cube(20, 20, 10);
    for interleaved in [false, true] {
        let mut opts = CubeEncodeOptions::default().with_segment_count(2);
        if interleaved {
            opts = opts.with_interleaved_entropy();
        }
        let bytes = encode_icer3d(&cube, &opts).unwrap();
        assert_eq!(
            parse_icer3d(&bytes).unwrap(),
            cube,
            "interleaved={interleaved}"
        );

        let quota = (bytes.len() / 2) as u64;
        let cut = encode_icer3d(&cube, &opts.clone().with_byte_quota(quota)).unwrap();
        assert!(cut.len() as u64 <= quota, "interleaved={interleaved}");
        let decoded = parse_icer3d(&cut).unwrap();
        assert_eq!(decoded.width, cube.width);
        assert_eq!(decoded.height, cube.height);
    }
}

#[test]
fn mean_subtraction_pays_on_band_level_drift() {
    // §III.A motivates the mean subtraction with the wide-ranging mean
    // values of spatially low-pass spectral planes. On a cube whose
    // bands differ mainly by large DC offsets, encoding with the means
    // carried out-of-band (4 bytes/band) must beat what the bit-plane
    // coder would spend coding those offsets as significant
    // coefficients: the deepest spatial low-pass subband is near-zero
    // after subtraction, so q (and the packet count) drops. Verify the
    // negligible-overhead claim indirectly: the whole lossless stream
    // of a 12-bit cube with ±1500 band offsets stays within 2x of the
    // same cube with zero offsets.
    let (w, h, bands) = (16u32, 16u32, 8u32);
    let mut flat = IcerCube::zeros(w, h, bands, 12);
    let mut drifted = IcerCube::zeros(w, h, bands, 12);
    let (wu, hu) = (w as usize, h as usize);
    for b in 0..bands as usize {
        let dc = 2048 + (((b * 977) % 3000) as i32 - 1500);
        for y in 0..hu {
            for x in 0..wu {
                let t = ((x * 7 + y * 9) % 45) as i32;
                flat.samples[b * wu * hu + y * wu + x] = (2048 + t) as u16;
                drifted.samples[b * wu * hu + y * wu + x] = (dc + t).clamp(0, 4095) as u16;
            }
        }
    }
    let flat_bytes = encode_icer3d(&flat, &CubeEncodeOptions::default()).unwrap();
    let drift_bytes = encode_icer3d(&drifted, &CubeEncodeOptions::default()).unwrap();
    assert_eq!(parse_icer3d(&drift_bytes).unwrap(), drifted);
    assert!(
        drift_bytes.len() < 2 * flat_bytes.len(),
        "band-level drift exploded the stream: {} vs {} bytes",
        drift_bytes.len(),
        flat_bytes.len()
    );
}

#[test]
fn corrupted_streams_never_panic() {
    // Every single-byte corruption of a valid stream must decode to
    // Ok(garbage) or Err(_), never panic — the deep-space channel
    // motivates exactly this robustness.
    let cube = correlated_cube(12, 12, 6);
    let bytes = encode_icer3d(&cube, &CubeEncodeOptions::default().with_segment_count(2)).unwrap();
    for i in 0..bytes.len() {
        for flip in [0x01u8, 0x80, 0xFF] {
            let mut corrupt = bytes.clone();
            corrupt[i] ^= flip;
            let _ = parse_icer3d(&corrupt);
        }
    }
    // Truncation sweep too (every prefix).
    for n in 0..bytes.len() {
        let _ = parse_icer3d(&bytes[..n]);
    }
}
