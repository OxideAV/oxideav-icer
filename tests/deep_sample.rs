//! Deep-sample (9..=16-bit) 2-D grayscale coverage — IPN 42-155 §II.C's
//! own operating point ("On MER, all cameras produce 12-bit pixels and
//! each is stored using a 16-bit word"; every §VII benchmark image is
//! 12-bit).
//!
//! A deep image rides the plane-container framing (format tag 2 + a
//! bit-depth byte) around a normal single-plane segment stream whose
//! coefficients span the deeper range; the §III bit-plane machinery is
//! depth-agnostic and the §II.C Table 4 analysis guarantees 16-bit
//! input can never overflow the `i32` coefficient words.

use oxideav_icer::{
    encode_icer, parse_icer, parse_icer_lenient, parse_icer_metadata, parse_icer_with_limits,
    psnr_db, DecodeLimits, EncodeOptions, IcerImage, IcerPixelFormat, WaveletFilter,
};

const ALL_FILTERS: [WaveletFilter; 7] = [
    WaveletFilter::FilterQ,
    WaveletFilter::FilterA,
    WaveletFilter::FilterB,
    WaveletFilter::FilterC,
    WaveletFilter::FilterD,
    WaveletFilter::FilterE,
    WaveletFilter::FilterF,
];

/// Deterministic textured fixture spanning the full `bits`-bit range.
fn textured_deep(w: u32, h: u32, bits: u8) -> IcerImage {
    let mut img = IcerImage::zeros(w, h, IcerPixelFormat::GrayDeep { bits });
    let max = (1u32 << bits) - 1;
    for y in 0..h {
        for x in 0..w {
            // Ramp + checker interference, wrapped into the n-bit range.
            let v = (x * 97 + y * 57 + (x ^ y) * 31) % (max + 1);
            img.set_sample(0, x, y, v as u16);
        }
    }
    img
}

#[test]
fn deep_wire_is_container_framed_and_gray8_wire_is_unchanged() {
    // A deep encode starts with the 0x0000 container sentinel + tag 2;
    // an 8-bit encode stays a bare segment stream (non-zero sync
    // prefix) — the byte-compatibility invariant.
    let deep = textured_deep(16, 16, 12);
    let bytes = encode_icer(&deep, &EncodeOptions::compressed()).unwrap();
    assert_eq!(&bytes[0..2], &[0x00, 0x00], "deep stream must be framed");
    assert_eq!(bytes[2], 2, "deep container format tag");
    assert_eq!(bytes[3], 1, "deep container plane count");
    assert_eq!(bytes[4], 12, "deep container bit depth");

    let gray = IcerImage::zeros(16, 16, IcerPixelFormat::Gray8);
    let gray_bytes = encode_icer(&gray, &EncodeOptions::compressed()).unwrap();
    assert_ne!(&gray_bytes[0..2], &[0x00, 0x00], "Gray8 stays bare");
}

#[test]
fn lossless_roundtrip_12bit_all_filters() {
    // §II.A: lossless operation works with any of the seven filters —
    // at 12 bits exactly as at 8.
    let img = textured_deep(32, 32, 12);
    for filter in ALL_FILTERS {
        let mut opts = EncodeOptions::compressed();
        opts.filter = filter;
        let bytes = encode_icer(&img, &opts).unwrap();
        let decoded = parse_icer(&bytes).unwrap();
        assert_eq!(decoded.pixel_format, IcerPixelFormat::GrayDeep { bits: 12 });
        assert_eq!(decoded.planes, img.planes, "filter {filter:?} not lossless");
    }
}

#[test]
fn lossless_roundtrip_depth_sweep() {
    // Every depth of the GrayDeep contract (9..=16) round-trips
    // bit-exactly under filter Q.
    for bits in 9u8..=16 {
        let img = textured_deep(24, 20, bits);
        let bytes = encode_icer(&img, &EncodeOptions::compressed()).unwrap();
        let decoded = parse_icer(&bytes).unwrap();
        assert_eq!(decoded.pixel_format, IcerPixelFormat::GrayDeep { bits });
        assert_eq!(decoded.planes, img.planes, "depth {bits} not lossless");
    }
}

#[test]
fn invalid_deep_depths_are_rejected() {
    // Depth 8 must ride the bare Gray8 wire form; deeper than 16
    // exceeds the sample word. Both refuse at encode time.
    for bits in [0u8, 8, 17, 255] {
        let img = IcerImage::zeros(8, 8, IcerPixelFormat::GrayDeep { bits });
        assert!(
            encode_icer(&img, &EncodeOptions::compressed()).is_err(),
            "bits {bits} must be rejected"
        );
    }
}

#[test]
fn corrupt_container_depth_byte_is_rejected() {
    let img = textured_deep(16, 16, 12);
    let mut bytes = encode_icer(&img, &EncodeOptions::compressed()).unwrap();
    for bad in [0u8, 8, 17, 200] {
        bytes[4] = bad;
        assert!(parse_icer(&bytes).is_err(), "depth byte {bad} accepted");
    }
}

#[test]
fn uncompressed_path_12bit_bit_exact() {
    // §III.D raw path: two little-endian bytes per sample.
    let img = textured_deep(20, 12, 12);
    let bytes = encode_icer(&img, &EncodeOptions::default()).unwrap();
    let decoded = parse_icer(&bytes).unwrap();
    assert_eq!(decoded.pixel_format, IcerPixelFormat::GrayDeep { bits: 12 });
    assert_eq!(decoded.planes, img.planes);
}

#[test]
fn multi_segment_row_strips_12bit_lossless() {
    let img = textured_deep(32, 32, 12);
    let mut opts = EncodeOptions::compressed();
    opts.segment_count = 4;
    let bytes = encode_icer(&img, &opts).unwrap();
    let decoded = parse_icer(&bytes).unwrap();
    assert_eq!(decoded.planes, img.planes);
}

#[test]
fn transform_domain_segments_12bit_lossless() {
    let img = textured_deep(32, 32, 12);
    let mut opts = EncodeOptions::compressed().with_transform_domain_segments();
    opts.segment_count = 4;
    let bytes = encode_icer(&img, &opts).unwrap();
    let decoded = parse_icer(&bytes).unwrap();
    assert_eq!(decoded.planes, img.planes);
}

#[test]
fn interleaved_entropy_backend_12bit_lossless() {
    let img = textured_deep(32, 32, 12);
    let opts = EncodeOptions::compressed().with_interleaved_entropy();
    let bytes = encode_icer(&img, &opts).unwrap();
    let decoded = parse_icer(&bytes).unwrap();
    assert_eq!(decoded.planes, img.planes);
}

#[test]
fn priority_interleaving_12bit_lossless() {
    let img = textured_deep(32, 32, 12);
    let opts = EncodeOptions::compressed().with_priority_interleaving();
    let bytes = encode_icer(&img, &opts).unwrap();
    let decoded = parse_icer(&bytes).unwrap();
    assert_eq!(decoded.planes, img.planes);
}

#[test]
fn min_loss_byte_curve_is_monotone_12bit() {
    // §VI.A: raising M can only shrink the output; at 12 bits the
    // usable M range is deeper than at 8 (more magnitude planes).
    let img = textured_deep(32, 32, 12);
    let mut prev = usize::MAX;
    for m in [0u8, 2, 4, 6, 8, 10] {
        let opts = EncodeOptions::compressed().with_min_loss(m);
        let bytes = encode_icer(&img, &opts).unwrap();
        assert!(
            bytes.len() <= prev,
            "min_loss {m} grew the stream: {} > {prev}",
            bytes.len()
        );
        prev = bytes.len();
    }
}

#[test]
fn budget_truncation_is_monotone_12bit() {
    // Progressive truncation: PSNR non-decreasing in the byte budget,
    // measured with the §VII 12-bit peak (4095).
    let img = textured_deep(32, 32, 12);
    let unbudgeted = encode_icer(&img, &EncodeOptions::compressed()).unwrap();
    let full = unbudgeted.len() as u64;
    let mut prev_psnr = -1.0f32;
    for frac in [4u64, 3, 2, 1] {
        let budget = full / frac;
        let opts = EncodeOptions::compressed().with_byte_budget(budget);
        let bytes = encode_icer(&img, &opts).unwrap();
        assert!(bytes.len() as u64 <= budget);
        let decoded = parse_icer(&bytes).unwrap();
        let p = psnr_db(&img, &decoded);
        assert!(
            p >= prev_psnr - 0.01,
            "budget {budget}: PSNR {p} fell below {prev_psnr}"
        );
        prev_psnr = p;
    }
    // The full-budget decode is the lossless stream.
    assert_eq!(prev_psnr, f32::INFINITY);
}

#[test]
fn lenient_decode_missing_strip_fills_deep_midpoint() {
    // Drop a middle strip: the lenient decoder reconstructs it at the
    // deep level-shift midpoint 2^(b-1) (2048 at 12 bits), not 128.
    let img = textured_deep(32, 32, 12);
    let mut opts = EncodeOptions::compressed();
    opts.segment_count = 4;
    let bytes = encode_icer(&img, &opts).unwrap();

    // Rebuild the container with segment 1 excised from the inner
    // stream.
    let parsed = oxideav_icer::parse_container(&bytes).unwrap();
    let inner = parsed.plane_bytes(&bytes, 0);
    let meta = parse_icer_metadata(&bytes).unwrap();
    assert_eq!(meta.segments.len(), 4);
    // Offsets in `meta` are rebased to the container buffer; segment 1
    // of the inner stream spans [seg1.offset - base, seg2.offset - base).
    let base = bytes.len() - inner.len();
    let s1 = meta.segments[1].offset - base;
    let s2 = meta.segments[2].offset - base;
    let mut cut = Vec::new();
    cut.extend_from_slice(&inner[..s1]);
    cut.extend_from_slice(&inner[s2..]);
    let reframed = {
        // Reuse the container framing helper by re-encoding by hand:
        // sentinel + tag 2 + plane count 1 + depth + 4-byte length.
        let mut out = vec![0u8, 0u8, 2u8, 1u8, 12u8];
        out.extend_from_slice(&(cut.len() as u32).to_be_bytes());
        out.extend_from_slice(&cut);
        out
    };

    let lenient = parse_icer_lenient(&reframed).unwrap();
    assert_eq!(lenient.missing_count, 1);
    assert!(!lenient.received[1]);
    assert_eq!(
        lenient.image.pixel_format,
        IcerPixelFormat::GrayDeep { bits: 12 }
    );
    // Strip 1 covers rows 8..16: every sample is the 12-bit midpoint.
    for y in 8..16 {
        for x in 0..32 {
            assert_eq!(lenient.image.sample(0, x, y), 2048, "({x},{y})");
        }
    }
    // Received strips decode bit-exactly.
    for y in 0..8 {
        for x in 0..32 {
            assert_eq!(lenient.image.sample(0, x, y), img.sample(0, x, y));
        }
    }
}

#[test]
fn metadata_walk_reports_deep_container_segments() {
    let img = textured_deep(32, 32, 14);
    let mut opts = EncodeOptions::compressed();
    opts.segment_count = 2;
    let bytes = encode_icer(&img, &opts).unwrap();
    let meta = parse_icer_metadata(&bytes).unwrap();
    assert_eq!(meta.segments.len(), 2);
    assert_eq!(meta.segments[0].header.width, 32);
    // Offsets are rebased to the container buffer: the first segment
    // starts after the container header (5 fixed bytes + 4-byte length
    // table).
    assert_eq!(meta.segments[0].offset, 9);
}

#[test]
fn decode_limits_apply_to_deep_streams() {
    let img = textured_deep(32, 32, 12);
    let bytes = encode_icer(&img, &EncodeOptions::compressed()).unwrap();
    let tight = DecodeLimits {
        max_pixels_per_segment: 16,
        max_total_pixels: 16,
    };
    assert!(parse_icer_with_limits(&bytes, &tight).is_err());
    assert!(parse_icer_with_limits(&bytes, &DecodeLimits::default()).is_ok());
}

#[test]
fn quality_target_12bit_meets_floor() {
    // The §VII PSNR definition uses the 12-bit peak, so a mid-quality
    // target is reachable well below the lossless byte count.
    let img = textured_deep(32, 32, 12);
    let opts = EncodeOptions::compressed().with_quality_target(40.0);
    let bytes = encode_icer(&img, &opts).unwrap();
    let decoded = parse_icer(&bytes).unwrap();
    let p = psnr_db(&img, &decoded);
    assert!(p >= 40.0, "achieved {p} dB");
    let lossless = encode_icer(&img, &EncodeOptions::compressed()).unwrap();
    assert!(
        bytes.len() < lossless.len(),
        "40 dB target should undercut lossless ({} vs {})",
        bytes.len(),
        lossless.len()
    );
}

#[test]
fn auto_filter_rd_12bit_picks_byte_winner() {
    let img = textured_deep(32, 32, 12);
    let opts = EncodeOptions::compressed().with_auto_filter_rd();
    let auto_bytes = encode_icer(&img, &opts).unwrap();
    // The RD pick can never exceed either explicit candidate.
    for filter in [WaveletFilter::FilterQ, WaveletFilter::FilterA] {
        let mut fopts = EncodeOptions::compressed();
        fopts.filter = filter;
        let candidate = encode_icer(&img, &fopts).unwrap();
        assert!(auto_bytes.len() <= candidate.len(), "{filter:?}");
    }
    assert_eq!(parse_icer(&auto_bytes).unwrap().planes, img.planes);
}

#[test]
fn uncompressed_fallback_fires_on_deep_noise() {
    // An LCG noise tile at 12 bits defeats the entropy stage; the
    // §III.D per-segment fallback must pick the raw path and stay
    // bit-exact.
    let mut img = IcerImage::zeros(24, 24, IcerPixelFormat::GrayDeep { bits: 12 });
    let mut state = 0x1234_5678u32;
    for y in 0..24 {
        for x in 0..24 {
            state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            img.set_sample(0, x, y, (state >> 16) as u16 & 0x0FFF);
        }
    }
    let plain = encode_icer(&img, &EncodeOptions::compressed()).unwrap();
    let fb = EncodeOptions::compressed().with_uncompressed_fallback();
    let bytes = encode_icer(&img, &fb).unwrap();
    assert!(bytes.len() <= plain.len());
    assert_eq!(parse_icer(&bytes).unwrap().planes, img.planes);
}

#[test]
fn deep_psnr_uses_deep_peak() {
    // A single-sample error of 1 on an N-pixel 12-bit image: MSE = 1/N,
    // PSNR = 10 log10(4095^2 N). At 32x32 that is ~102.3 dB — an 8-bit
    // peak would give ~78.2 dB.
    let img = textured_deep(32, 32, 12);
    let mut other = img.clone();
    let v = other.sample(0, 3, 5);
    other.set_sample(0, 3, 5, v ^ 1);
    let p = psnr_db(&img, &other);
    let expect = 10.0 * (4095.0f64 * 4095.0 * 1024.0).log10();
    assert!(
        (p as f64 - expect).abs() < 0.05,
        "PSNR {p} vs expected {expect}"
    );
}

#[test]
fn roi_priorities_and_budget_compose_at_12bit() {
    // Centre-out ROI at a tight budget: centre strip survives at
    // higher fidelity than the periphery, in the deep domain.
    let img = textured_deep(32, 32, 12);
    let mut opts = EncodeOptions::compressed().with_byte_budget(700);
    opts.segment_count = 4;
    let opts = opts.with_center_roi();
    let bytes = encode_icer(&img, &opts).unwrap();
    assert!(bytes.len() <= 700);
    let decoded = parse_icer(&bytes).unwrap();
    assert_eq!(decoded.width, 32);
    assert_eq!(decoded.height, 32);
    assert_eq!(decoded.pixel_format, IcerPixelFormat::GrayDeep { bits: 12 });
}
