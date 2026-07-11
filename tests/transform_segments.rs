//! IPN 42-155 §V.B transform-domain segmentation — end-to-end coverage.
//!
//! §V.B: "segmentation occurs after the wavelet decomposition";
//! the LL subband is partitioned by the §V.D algorithm and the
//! partition is mapped to the other subbands, each segment compressed
//! independently (own context modeler + entropy coder) so a lost
//! segment cannot corrupt the others. The decompressor recomputes the
//! partition from the header parameters (§V.D) — boundaries are never
//! encoded.

use oxideav_icer::{
    encode_icer, ll_dimensions, parse_icer, parse_icer_lenient, parse_icer_metadata, partition,
    EncodeOptions, IcerImage, IcerPixelFormat, WaveletFilter,
};

/// Deterministic textured Gray8 fixture.
fn textured(w: usize, h: usize, seed: u64) -> IcerImage {
    let mut img = IcerImage::zeros(w as u32, h as u32, IcerPixelFormat::Gray8);
    let mut s = seed.wrapping_mul(0x9E37_79B9_7F4A_7C15);
    let stride = img.planes[0].stride;
    for y in 0..h {
        for x in 0..w {
            s = s
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            let noise = ((s >> 56) & 0x1F) as i32;
            let base = ((x * 2 + y * 3) % 200) as i32;
            img.planes[0].data[y * stride + x] = (base + noise).clamp(0, 255) as u8;
        }
    }
    img
}

fn opts_transform(segments: u16) -> EncodeOptions {
    let mut o = EncodeOptions::compressed().with_transform_domain_segments();
    o.segment_count = segments;
    o
}

/// Filter-Q full-quality decode is bit-exact through the §V.B path
/// across geometries, segment counts, and both entropy backends.
#[test]
fn lossless_roundtrip_filter_q() {
    for &(w, h, levels, segs) in &[
        (64usize, 64usize, 3u8, 4u16),
        (48, 40, 3, 5),
        (32, 32, 2, 7),
        (64, 64, 3, 1),
        (17, 16, 2, 3),
        (40, 24, 1, 6),
    ] {
        let img = textured(w, h, 0xA11CE ^ (w as u64) << 8 ^ segs as u64);
        for interleaved in [false, true] {
            let mut opts = opts_transform(segs);
            opts.wavelet_levels = levels;
            if interleaved {
                opts = opts.with_interleaved_entropy();
            }
            let bytes = encode_icer(&img, &opts)
                .unwrap_or_else(|e| panic!("{w}x{h}/{segs} L{levels} encode: {e}"));
            let dec = parse_icer(&bytes)
                .unwrap_or_else(|e| panic!("{w}x{h}/{segs} L{levels} decode: {e}"));
            assert_eq!(dec.width as usize, w);
            assert_eq!(dec.height as usize, h);
            assert_eq!(
                dec.planes[0].data, img.planes[0].data,
                "{w}x{h}/{segs} L{levels} interleaved={interleaved} must be bit-exact"
            );
        }
    }
}

/// Colour 4:4:4 images run three independent §V.B pipelines behind the
/// plane container — bit-exact on filter Q.
#[test]
fn colour_roundtrip() {
    let (w, h) = (32usize, 24usize);
    let mut img = IcerImage::zeros(w as u32, h as u32, IcerPixelFormat::Yuv444P);
    for p in 0..3 {
        let plane = textured(w, h, 0xC0102 + p as u64);
        img.planes[p] = plane.planes[0].clone();
    }
    let opts = opts_transform(4);
    let bytes = encode_icer(&img, &opts).unwrap();
    let dec = parse_icer(&bytes).unwrap();
    assert_eq!(dec.pixel_format, IcerPixelFormat::Yuv444P);
    for p in 0..3 {
        assert_eq!(dec.planes[p].data, img.planes[p].data, "plane {p}");
    }
}

/// A byte budget is honoured; every dropped segment leaves a
/// placeholder header so the decoded image still frames the full
/// geometry.
#[test]
fn budget_respects_cap_and_geometry() {
    let img = textured(64, 64, 0xB4D9);
    for budget in [100u64, 300, 700, 1500] {
        let opts = opts_transform(4).with_byte_budget(budget);
        let bytes = encode_icer(&img, &opts).unwrap();
        assert!(
            bytes.len() as u64 <= budget,
            "budget {budget}: emitted {}",
            bytes.len()
        );
        let dec = parse_icer(&bytes).unwrap();
        assert_eq!((dec.width, dec.height), (64, 64), "budget {budget}");
    }
}

/// Progressive quality: a larger budget never yields a worse
/// reconstruction (mean-abs error is non-increasing in budget).
#[test]
fn budget_quality_is_monotone() {
    let img = textured(64, 64, 0x9090);
    let mae = |dec: &IcerImage| -> f64 {
        img.planes[0]
            .data
            .iter()
            .zip(dec.planes[0].data.iter())
            .map(|(&a, &b)| (a as f64 - b as f64).abs())
            .sum::<f64>()
            / (64.0 * 64.0)
    };
    let mut last = f64::INFINITY;
    for budget in [200u64, 500, 1000, 2000, 4000, 8000] {
        let opts = opts_transform(4).with_byte_budget(budget);
        let bytes = encode_icer(&img, &opts).unwrap();
        let dec = parse_icer(&bytes).unwrap();
        let e = mae(&dec);
        assert!(
            e <= last + 1e-9,
            "budget {budget}: MAE {e} regressed above {last}"
        );
        last = e;
    }
}

/// §V.B error containment: dropping one segment from the stream leaves
/// the loss contained to the segment plus a decaying wavelet-support
/// "bleed". Under the spec-exact §II.A transform the inverse eq (3)
/// recursion (`d[n] = h[n] + predictor(..., d[n+1])`) propagates a
/// floor-truncated, geometrically-decaying tail beyond the strictly
/// finite support a plain lifting kernel would have, so containment is
/// pinned as a decay profile (measured on this fixture set):
///
///   * beyond a `3·2^D` dilation the residual is at most ±4 grey
///     levels (measured max 3 on this fixture, ≤4 across a wider
///     five-seed sweep);
///   * beyond an `8·2^D` dilation the decode is bit-exact (measured
///     max bleed distance: 7 LL pixels on this fixture, 8 across the
///     sweep).
///
/// The strict decoder refuses the gap; the lenient decoder reports it.
#[test]
fn lenient_missing_segment_is_contained() {
    let (w, h, levels, segs) = (128usize, 128usize, 2u8, 4usize);
    let img = textured(w, h, 0x10CA);
    let mut opts = opts_transform(segs as u16);
    opts.wavelet_levels = levels;
    let bytes = encode_icer(&img, &opts).unwrap();
    let full = parse_icer(&bytes).unwrap();
    assert_eq!(full.planes[0].data, img.planes[0].data);

    let meta = parse_icer_metadata(&bytes).unwrap();
    assert_eq!(meta.segments.len(), segs);
    // The §V.D partition the decoder recomputes, in image coordinates.
    let (w_ll, h_ll) = ll_dimensions(w, h, levels);
    let rects = partition(w_ll, h_ll, segs).unwrap();
    let scale = 1usize << levels;
    // (dilation in LL pixels, max abs residual allowed outside it).
    let profile: &[(usize, i32)] = &[(3, 4), (8, 0)];

    for (drop_idx, r) in rects.iter().enumerate() {
        let seg = &meta.segments[drop_idx];
        assert_eq!(seg.header.segment_index as usize, drop_idx);
        let mut cut = Vec::new();
        cut.extend_from_slice(&bytes[..seg.offset]);
        cut.extend_from_slice(&bytes[seg.offset + seg.byte_length..]);

        // Strict decode refuses the incomplete §V.D set.
        assert!(
            parse_icer(&cut).is_err(),
            "strict must reject gap {drop_idx}"
        );

        let lenient = parse_icer_lenient(&cut).unwrap();
        assert_eq!(lenient.missing_count, 1);
        assert!(!lenient.received[drop_idx]);
        assert_eq!(lenient.image.width as usize, w);
        assert_eq!(lenient.image.height as usize, h);

        for &(dilation_ll, max_residual) in profile {
            // The dropped segment's image-domain rectangle (LL rect
            // scaled up), dilated by the profile distance.
            let margin = dilation_ll * scale;
            let x0 = (r.x * scale).saturating_sub(margin);
            let y0 = (r.y * scale).saturating_sub(margin);
            let x1 = ((r.x + r.width) * scale + margin).min(w);
            let y1 = ((r.y + r.height) * scale + margin).min(h);
            let mut far_pixels = 0usize;
            for y in 0..h {
                for x in 0..w {
                    let inside_dilated = x >= x0 && x < x1 && y >= y0 && y < y1;
                    if inside_dilated {
                        continue;
                    }
                    far_pixels += 1;
                    let got = lenient.image.planes[0].data[y * lenient.image.planes[0].stride + x];
                    let want = full.planes[0].data[y * full.planes[0].stride + x];
                    let residual = (got as i32 - want as i32).abs();
                    assert!(
                        residual <= max_residual,
                        "segment {drop_idx}: pixel ({x},{y}) at > {dilation_ll} LL px \
                         from the lost region drifted by {residual} (> {max_residual}) \
                         (§V.B containment decay profile)"
                    );
                }
            }
            assert!(
                far_pixels > 0,
                "segment {drop_idx}: containment check at {dilation_ll} LL px \
                 must cover some pixels"
            );
        }
    }
}

/// Mixing §V.B transform-domain segments and row-strip segments in one
/// stream is a contradiction and is refused.
#[test]
fn mixed_mode_stream_is_rejected() {
    let img = textured(32, 32, 0x3113);
    let transform = encode_icer(&img, &opts_transform(2)).unwrap();
    let strip = encode_icer(&img, &EncodeOptions::compressed()).unwrap();
    let mut mixed = transform.clone();
    mixed.extend_from_slice(&strip);
    assert!(parse_icer(&mixed).is_err());
    let mut mixed2 = strip;
    mixed2.extend_from_slice(&transform);
    assert!(parse_icer(&mixed2).is_err());
}

/// §V.D eq (9): the segment count must not exceed the LL pixel count.
#[test]
fn eq9_violation_is_refused_at_encode() {
    // 16x16 at 3 levels -> LL is 2x2 = 4 pixels < 5 segments.
    let img = textured(16, 16, 0xE9);
    let mut opts = opts_transform(5);
    opts.wavelet_levels = 3;
    assert!(encode_icer(&img, &opts).is_err());
}

/// ROI segment priorities compose with the §V.B path: the emission
/// order changes but the decode is unchanged, and under a budget the
/// prioritised segment survives.
#[test]
fn roi_priorities_compose() {
    let img = textured(64, 64, 0x2019);
    // 4 segments; prioritise segment 3 first.
    let mut opts = opts_transform(4).with_segment_priorities(vec![1, 2, 3, 0]);
    opts.wavelet_levels = 3;
    let bytes = encode_icer(&img, &opts).unwrap();
    let dec = parse_icer(&bytes).unwrap();
    assert_eq!(dec.planes[0].data, img.planes[0].data);

    // Tight budget: segment 3 (rank 0) must be a real segment while
    // rank-3 segment 0 degrades to a placeholder.
    let bytes_full_one = {
        let o = opts_transform(1);
        encode_icer(&img, &o).unwrap().len() as u64
    };
    let tight = bytes_full_one / 3;
    let mut opts = opts_transform(4)
        .with_segment_priorities(vec![1, 2, 3, 0])
        .with_byte_budget(tight);
    opts.wavelet_levels = 3;
    let bytes = encode_icer(&img, &opts).unwrap();
    assert!(bytes.len() as u64 <= tight);
    let meta = parse_icer_metadata(&bytes).unwrap();
    let len_of = |idx: u16| {
        meta.segments
            .iter()
            .find(|s| s.header.segment_index == idx)
            .map(|s| s.header.segment_length)
            .unwrap()
    };
    assert!(
        len_of(3) > 0,
        "rank-0 segment 3 must survive the tight budget"
    );
    // Rank 0 is encoded first and gets the budget; rank 3 is encoded
    // last and degrades to a (possibly zero-body) truncated tail.
    assert!(
        len_of(3) > len_of(0),
        "rank-0 segment 3 ({}) must out-byte rank-3 segment 0 ({})",
        len_of(3),
        len_of(0)
    );
    // Geometry framed regardless.
    let dec = parse_icer(&bytes).unwrap();
    assert_eq!((dec.width, dec.height), (64, 64));
}

/// The metadata walker surfaces the §V.D parameters carried per header.
#[test]
fn metadata_reports_transform_parameters() {
    let img = textured(48, 32, 0x77);
    let bytes = encode_icer(&img, &opts_transform(3)).unwrap();
    let meta = parse_icer_metadata(&bytes).unwrap();
    assert_eq!(meta.segments.len(), 3);
    for (i, s) in meta.segments.iter().enumerate() {
        assert!(s.header.transform_segmented);
        assert_eq!(s.header.total_segments, 3);
        assert_eq!(s.header.segment_index as usize, i);
        assert_eq!(s.header.width, 48);
        assert_eq!(s.header.height, 32);
    }
}

/// Filter A through the §V.B path is bit-exact -- it is one of the
/// seven §II.A reversible integer transforms (as on the row-strip
/// path).
#[test]
fn filter_a_bit_exact() {
    let img = textured(64, 64, 0xF17A);
    let mut opts = opts_transform(4);
    opts.filter = WaveletFilter::FilterA;
    let bytes = encode_icer(&img, &opts).unwrap();
    let dec = parse_icer(&bytes).unwrap();
    assert_eq!(
        dec.planes[0].data, img.planes[0].data,
        "filter-A transform-domain full-quality decode must be bit-exact (§II.A)"
    );
}
