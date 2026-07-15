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

/// 12-bit fixture with per-band signal-level drift (the §III.A
/// "systematic variations in signal level of different spectral bands")
/// over spatial texture — the scenario the ICER-3D mean subtraction and
/// error containment exist for.
fn drifted_cube(w: u32, h: u32, bands: u32) -> IcerCube {
    let mut cube = IcerCube::zeros(w, h, bands, 12);
    let (wu, hu) = (w as usize, h as usize);
    for b in 0..bands as usize {
        let dc = 800 + ((b * 137) % 1200) as i32;
        for y in 0..hu {
            for x in 0..wu {
                let t = ((x * 13 + y * 29 + b * 7) % 257) as i32 - 128;
                let ridge = if (x / 4 + y / 4) % 3 == 0 { 200 } else { 0 };
                cube.samples[b * wu * hu + y * wu + x] = (dc + t + ridge).clamp(0, 4095) as u16;
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

/// Walk the cube wire and drop segment `kill`'s packets (packet count
/// forced to 0, packet bytes spliced out) — the "data pertaining to a
/// segment are lost" scenario of IPN 42-164 §I.
fn drop_segment_packets(bytes: &[u8], kill: usize) -> Vec<u8> {
    let bands = u16::from_be_bytes([bytes[8], bytes[9]]) as usize;
    let segs = bytes[13] as usize;
    let mut out = Vec::new();
    let mut pos = 17usize; // magic (4) + header body (13)
    out.extend_from_slice(&bytes[..pos]);
    for seg in 0..segs {
        let count_off = pos + 2 + 4 * bands; // idx + q + means
        let count = u16::from_be_bytes([bytes[count_off], bytes[count_off + 1]]) as usize;
        let mut body_end = count_off + 2;
        for _ in 0..count {
            let len = u32::from_be_bytes([
                bytes[body_end + 1],
                bytes[body_end + 2],
                bytes[body_end + 3],
                bytes[body_end + 4],
            ]) as usize;
            body_end += 5 + len;
        }
        if seg == kill {
            out.extend_from_slice(&bytes[pos..count_off]);
            out.extend_from_slice(&[0, 0]);
        } else {
            out.extend_from_slice(&bytes[pos..body_end]);
        }
        pos = body_end;
    }
    out
}

#[test]
fn transform_domain_segment_loss_is_contained() {
    // IPN 42-164 §I: "Each segment is compressed independently so that
    // if data pertaining to a segment are lost or corrupted, the other
    // segments are unaffected." With the §V.D transform-domain
    // partition the segments share one inverse transform, so "the other
    // segments are unaffected" holds up to the wavelet-support bleed of
    // the boundary — pin the measured profile.
    //
    // Geometry: 64x64x6 at 2 levels -> ts = 2, 16x16 LL lattice, 16
    // §V.D segments of 4x4 LL pixels = 16x16-pixel spatial windows.
    // Segment 0's window is [0,16) x [0,16), extended through all
    // bands.
    let (w, h, bands) = (64usize, 64usize, 6usize);
    let cube = drifted_cube(w as u32, h as u32, bands as u32);
    let opts = CubeEncodeOptions::default()
        .with_transform_domain_segments()
        .with_segment_count(16)
        .with_levels(2);
    let bytes = encode_icer3d(&cube, &opts).unwrap();
    let full = parse_icer3d(&bytes).unwrap();
    assert_eq!(full, cube, "baseline must be lossless");

    let lost = parse_icer3d(&drop_segment_packets(&bytes, 0)).unwrap();
    let ts_stride = 4usize; // 2^ts
    let (wx1, wy1) = (16usize, 16usize); // segment 0 window end
    let max_err_outside = |dil: usize| -> i32 {
        let mut max_err = 0i32;
        for b in 0..bands {
            for y in 0..h {
                for x in 0..w {
                    if x < wx1 + dil && y < wy1 + dil {
                        continue; // inside the dilated window
                    }
                    let i = b * w * h + y * w + x;
                    let e = (lost.samples[i] as i32 - full.samples[i] as i32).abs();
                    max_err = max_err.max(e);
                }
            }
        }
        max_err
    };
    // Measured on this fixture: <= 4 beyond one lattice step, bit-exact
    // beyond two. Pin with headroom: residual <= 8 beyond a 1*2^ts
    // dilation, bit-identical beyond 3*2^ts.
    assert!(
        max_err_outside(ts_stride) <= 8,
        "bleed beyond one lattice step: {}",
        max_err_outside(ts_stride)
    );
    assert_eq!(
        max_err_outside(3 * ts_stride),
        0,
        "loss must be bit-contained beyond a 3*2^ts dilation"
    );

    // Inside the lost window, the §III.A per-segment means (which ride
    // the segment's fixed wire part, not its packets) still anchor the
    // reconstruction: the region degrades to a smooth mean-level patch,
    // far better than a flat mid-range fill.
    let (mut mae_lost, mut mae_flat, mut n) = (0f64, 0f64, 0f64);
    let flat = 1i32 << (cube.bit_depth - 1);
    for b in 0..bands {
        for y in 0..wy1 {
            for x in 0..wx1 {
                let i = b * w * h + y * w + x;
                mae_lost += (lost.samples[i] as f64 - cube.samples[i] as f64).abs();
                mae_flat += (flat as f64 - cube.samples[i] as f64).abs();
                n += 1.0;
            }
        }
    }
    assert!(
        mae_lost < mae_flat / 2.0,
        "mean-anchored loss ({:.1}) must beat a flat fill ({:.1}) by 2x+",
        mae_lost / n,
        mae_flat / n
    );
}

#[test]
fn transform_domain_every_segment_loss_survives() {
    // Dropping any single segment must still decode, and each loss must
    // stay confined to its own dilated window: the §II.A inverse
    // recursion propagates a floor-truncated, geometrically-decaying
    // tail (the 2-D §V.B containment profile), so pin residual <= 4
    // grey levels beyond a 3*2^ts dilation and bit-identity beyond
    // 8*2^ts, per band.
    let (w, h, bands) = (48usize, 32usize, 5usize);
    let cube = drifted_cube(w as u32, h as u32, bands as u32);
    let opts = CubeEncodeOptions::default()
        .with_transform_domain_segments()
        .with_segment_count(6)
        .with_levels(2);
    let bytes = encode_icer3d(&cube, &opts).unwrap();
    let full = parse_icer3d(&bytes).unwrap();
    let segs = bytes[13] as usize;
    assert_eq!(segs, 6);
    // Recompute the partition the way the codec does: ts = 2 on this
    // geometry, LL lattice 12x8, 6 segments.
    let (llw, llh, ts) = (12usize, 8usize, 2u8);
    let rects = oxideav_icer::partition(llw, llh, segs).unwrap();
    for rect in &rects {
        let lost = parse_icer3d(&drop_segment_packets(&bytes, rect.index)).unwrap();
        let (x0, x1) = (rect.x << ts, (rect.x + rect.width) << ts);
        let (y0, y1) = (rect.y << ts, (rect.y + rect.height) << ts);
        let mut touched = false;
        for b in 0..bands {
            for y in 0..h {
                for x in 0..w {
                    let i = b * w * h + y * w + x;
                    let e = (lost.samples[i] as i32 - full.samples[i] as i32).unsigned_abs();
                    touched |= e > 0;
                    let outside = |dil: usize| {
                        !(x + dil >= x0 && x < x1 + dil && y + dil >= y0 && y < y1 + dil)
                    };
                    if outside(3 << ts) {
                        assert!(
                            e <= 4,
                            "segment {} residual {e} at ({x},{y},{b}) beyond 3*2^ts",
                            rect.index
                        );
                    }
                    if outside(8 << ts) {
                        assert_eq!(
                            e, 0,
                            "segment {} loss leaked to ({x},{y},{b}) beyond 8*2^ts",
                            rect.index
                        );
                    }
                }
            }
        }
        assert!(touched, "segment {} drop had no effect", rect.index);
    }
}

#[test]
fn corrupted_streams_never_panic() {
    // Every single-byte corruption of a valid stream must decode to
    // Ok(garbage) or Err(_), never panic — the deep-space channel
    // motivates exactly this robustness.
    let cube = correlated_cube(12, 12, 6);
    for opts in [
        CubeEncodeOptions::default().with_segment_count(2),
        CubeEncodeOptions::default()
            .with_transform_domain_segments()
            .with_segment_count(3),
    ] {
        let bytes = encode_icer3d(&cube, &opts).unwrap();
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
}
