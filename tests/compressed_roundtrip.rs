//! Round-trip cover for the compressed (wavelet + bit-plane) encode →
//! decode pipeline plus the multi-segment demuxer.

use oxideav_icer::{
    encode_icer, parse_icer, parse_icer_metadata, EncodeOptions, IcerImage, IcerPixelFormat,
    WaveletFilter,
};

fn ramp_image(w: u32, h: u32) -> IcerImage {
    let mut img = IcerImage::zeros(w, h, IcerPixelFormat::Gray8);
    let plane = &mut img.planes[0];
    for y in 0..h as usize {
        for x in 0..w as usize {
            plane.data[y * plane.stride + x] = ((x + y) & 0xFF) as u8;
        }
    }
    img
}

fn smooth_image(w: u32, h: u32) -> IcerImage {
    // A constant-mid-grey image is the friendliest possible payload for
    // the bit-plane scanner: every coefficient outside the LL subband
    // is zero, so the entropy stage should produce a tiny output and
    // the round-trip should be bit-exact.
    let mut img = IcerImage::zeros(w, h, IcerPixelFormat::Gray8);
    let plane = &mut img.planes[0];
    for byte in plane.data.iter_mut() {
        *byte = 128;
    }
    img
}

#[test]
fn compressed_roundtrip_smooth_image_is_bit_exact() {
    let original = smooth_image(16, 16);
    let opts = EncodeOptions::compressed();
    let bytes = encode_icer(&original, &opts).unwrap();
    let decoded = parse_icer(&bytes).unwrap();
    assert_eq!(decoded.width, 16);
    assert_eq!(decoded.height, 16);
    assert_eq!(decoded.planes[0].data, original.planes[0].data);
}

#[test]
fn compressed_roundtrip_ramp_image_filter_q_is_bit_exact() {
    // Filter Q (integer 5/3) is reversible — the round-trip must be
    // bit-exact even for a non-trivial image.
    let original = ramp_image(8, 8);
    let opts = EncodeOptions {
        filter: WaveletFilter::Reversible53,
        wavelet_levels: 2,
        bit_plane_count: 8,
        uncompressed: false,
        ..EncodeOptions::default()
    };
    let bytes = encode_icer(&original, &opts).unwrap();
    let decoded = parse_icer(&bytes).unwrap();
    assert_eq!(
        decoded.planes[0].data, original.planes[0].data,
        "filter Q lossless round-trip must be bit-exact"
    );
}

#[test]
fn compressed_segment_metadata_marks_compressed_flag() {
    let original = smooth_image(16, 16);
    let opts = EncodeOptions::compressed();
    let bytes = encode_icer(&original, &opts).unwrap();
    let meta = parse_icer_metadata(&bytes).unwrap();
    assert_eq!(meta.segments.len(), 1);
    assert!(!meta.segments[0].header.uncompressed);
}

#[test]
fn compressed_roundtrip_filter_a_within_tolerance() {
    // Float filter A is lossy through the float DWT + integer round
    // step; assert pixel error stays bounded.
    let original = ramp_image(16, 16);
    let opts = EncodeOptions {
        filter: WaveletFilter::NineSevenA,
        wavelet_levels: 2,
        bit_plane_count: 12,
        uncompressed: false,
        ..EncodeOptions::default()
    };
    let bytes = encode_icer(&original, &opts).unwrap();
    let decoded = parse_icer(&bytes).unwrap();
    let err: i32 = original.planes[0]
        .data
        .iter()
        .zip(decoded.planes[0].data.iter())
        .map(|(a, b)| (*a as i32 - *b as i32).abs())
        .sum();
    let mean_err = err as f32 / (original.width * original.height) as f32;
    assert!(
        mean_err < 8.0,
        "filter A mean abs error {mean_err} too high"
    );
}

#[test]
fn multi_segment_roundtrip_uncompressed() {
    // Build a 32x32 image and encode as 4 horizontal-strip segments.
    let original = ramp_image(32, 32);
    let opts = EncodeOptions {
        segment_count: 4,
        ..EncodeOptions::default()
    };
    let bytes = encode_icer(&original, &opts).unwrap();
    let meta = parse_icer_metadata(&bytes).unwrap();
    assert_eq!(meta.segments.len(), 4, "should produce 4 segments");
    let decoded = parse_icer(&bytes).unwrap();
    assert_eq!(decoded.width, 32);
    assert_eq!(decoded.height, 32);
    assert_eq!(decoded.planes[0].data, original.planes[0].data);
}

#[test]
fn multi_segment_roundtrip_compressed() {
    let original = smooth_image(32, 32);
    let opts = EncodeOptions {
        segment_count: 2,
        uncompressed: false,
        ..EncodeOptions::default()
    };
    let bytes = encode_icer(&original, &opts).unwrap();
    let meta = parse_icer_metadata(&bytes).unwrap();
    assert_eq!(meta.segments.len(), 2);
    let decoded = parse_icer(&bytes).unwrap();
    assert_eq!(decoded.planes[0].data, original.planes[0].data);
}

#[test]
fn multi_segment_indices_are_sequential() {
    let original = ramp_image(16, 16);
    let opts = EncodeOptions {
        segment_count: 4,
        ..EncodeOptions::default()
    };
    let bytes = encode_icer(&original, &opts).unwrap();
    let meta = parse_icer_metadata(&bytes).unwrap();
    for (i, s) in meta.segments.iter().enumerate() {
        assert_eq!(
            s.header.segment_index as usize, i,
            "segment {i} has index {}",
            s.header.segment_index
        );
    }
}

#[test]
fn compressed_payload_smaller_than_uncompressed_for_smooth_image() {
    // Sanity check that the compressed path is actually compressing
    // a friendly image. For pure constant-grey, the entropy stage
    // should produce far fewer bytes than the raw 256-byte payload.
    let img = smooth_image(16, 16);
    let unc = encode_icer(&img, &EncodeOptions::default()).unwrap();
    let cmp = encode_icer(&img, &EncodeOptions::compressed()).unwrap();
    assert!(
        cmp.len() < unc.len(),
        "compressed {} bytes should beat uncompressed {} bytes on a constant image",
        cmp.len(),
        unc.len()
    );
}

#[test]
fn compressed_roundtrip_filter_g_within_tolerance() {
    // Filter G (Le Gall 5/3 float variant) -- end-to-end encode + decode
    // through the full pipeline. Expected to be lossy (float DWT + integer
    // rounding) but within a reasonable error bound.
    let original = ramp_image(16, 16);
    let opts = EncodeOptions {
        filter: WaveletFilter::FilterG,
        wavelet_levels: 2,
        bit_plane_count: 12,
        uncompressed: false,
        ..EncodeOptions::default()
    };
    let bytes = encode_icer(&original, &opts).unwrap();
    let decoded = parse_icer(&bytes).unwrap();
    let err: i32 = original.planes[0]
        .data
        .iter()
        .zip(decoded.planes[0].data.iter())
        .map(|(a, b)| (*a as i32 - *b as i32).abs())
        .sum();
    let mean_err = err as f32 / (original.width * original.height) as f32;
    assert!(
        mean_err < 12.0,
        "filter G mean abs error {mean_err} too high"
    );
}

#[test]
fn compressed_roundtrip_filter_q_multi_packet_metadata() {
    // Verify that the multi-packet encoder produces more than one packet
    // per segment when using filter Q (one pair per bit-plane).
    let original = ramp_image(16, 16);
    let opts = EncodeOptions {
        filter: WaveletFilter::Reversible53,
        wavelet_levels: 2,
        bit_plane_count: 4,
        uncompressed: false,
        ..EncodeOptions::default()
    };
    let bytes = encode_icer(&original, &opts).unwrap();
    let meta = parse_icer_metadata(&bytes).unwrap();
    assert_eq!(meta.segments.len(), 1);
    // Each bit-plane produces 2 packets (significance + refinement).
    // With q=4 bit-planes minimum, we expect at least 8 packets.
    assert!(
        meta.segments[0].packet_count >= 8,
        "expected >= 8 packets for q>=4, got {}",
        meta.segments[0].packet_count
    );
}

#[test]
fn compressed_roundtrip_all_filters() {
    // Verify that all seven float filters A-G round-trip through the
    // full encode/decode pipeline (bounded error, not bit-exact).
    let original = smooth_image(16, 16);
    for filter in [
        WaveletFilter::NineSevenA,
        WaveletFilter::FilterB,
        WaveletFilter::FilterC,
        WaveletFilter::FilterD,
        WaveletFilter::FilterE,
        WaveletFilter::FilterF,
        WaveletFilter::FilterG,
    ] {
        let opts = EncodeOptions {
            filter,
            wavelet_levels: 2,
            bit_plane_count: 10,
            uncompressed: false,
            ..EncodeOptions::default()
        };
        let bytes = encode_icer(&original, &opts).unwrap();
        let decoded = parse_icer(&bytes).unwrap();
        let err: i32 = original.planes[0]
            .data
            .iter()
            .zip(decoded.planes[0].data.iter())
            .map(|(a, b)| (*a as i32 - *b as i32).abs())
            .sum();
        let mean_err = err as f32 / (original.width * original.height) as f32;
        assert!(
            mean_err < 16.0,
            "filter {filter:?} mean abs error {mean_err} too high on smooth image"
        );
    }
}

#[test]
fn multi_segment_compressed_all_filters() {
    // Multi-segment encode/decode with filters Q and G.
    for filter in [WaveletFilter::Reversible53, WaveletFilter::FilterG] {
        let original = ramp_image(16, 16);
        let opts = EncodeOptions {
            filter,
            wavelet_levels: 2,
            bit_plane_count: 8,
            uncompressed: false,
            segment_count: 2,
            ..EncodeOptions::default()
        };
        let bytes = encode_icer(&original, &opts).unwrap();
        let decoded = parse_icer(&bytes).unwrap();
        let err: i32 = original.planes[0]
            .data
            .iter()
            .zip(decoded.planes[0].data.iter())
            .map(|(a, b)| (*a as i32 - *b as i32).abs())
            .sum();
        let mean_err = err as f32 / (original.width * original.height) as f32;
        let max_err = if filter == WaveletFilter::Reversible53 {
            0.0
        } else {
            16.0
        };
        assert!(
            mean_err <= max_err,
            "filter {filter:?} multi-segment mean error {mean_err} > {max_err}"
        );
    }
}
