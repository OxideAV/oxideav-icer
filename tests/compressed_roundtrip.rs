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
        filter: WaveletFilter::FilterQ,
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
fn compressed_roundtrip_filter_a_is_bit_exact() {
    // Filter A is one of the seven IPN 42-155 §II.A reversible integer
    // transforms: a full-quality round-trip must be bit-exact, exactly
    // like filter Q.
    let original = ramp_image(16, 16);
    let opts = EncodeOptions {
        filter: WaveletFilter::FilterA,
        wavelet_levels: 2,
        bit_plane_count: 12,
        uncompressed: false,
        ..EncodeOptions::default()
    };
    let bytes = encode_icer(&original, &opts).unwrap();
    let decoded = parse_icer(&bytes).unwrap();
    assert_eq!(
        decoded.planes[0].data, original.planes[0].data,
        "filter A lossless round-trip must be bit-exact (§II.A)"
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
fn compressed_roundtrip_filter_q_multi_packet_metadata() {
    // Verify that the multi-packet encoder produces more than one packet
    // per segment when using filter Q (one pair per bit-plane).
    let original = ramp_image(16, 16);
    let opts = EncodeOptions {
        filter: WaveletFilter::FilterQ,
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
fn compressed_roundtrip_all_filters_bit_exact() {
    // IPN 42-155 §II.A: all seven filters (A-F + Q) are reversible
    // integer transforms, so a full-quality round-trip must be
    // bit-exact under every one -- on both a friendly constant image
    // and a textured ramp.
    for original in [smooth_image(16, 16), ramp_image(16, 16)] {
        for filter in [
            WaveletFilter::FilterQ,
            WaveletFilter::FilterA,
            WaveletFilter::FilterB,
            WaveletFilter::FilterC,
            WaveletFilter::FilterD,
            WaveletFilter::FilterE,
            WaveletFilter::FilterF,
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
            assert_eq!(
                decoded.planes[0].data, original.planes[0].data,
                "filter {filter:?} lossless round-trip must be bit-exact (§II.A)"
            );
        }
    }
}

#[test]
fn multi_segment_compressed_all_filters() {
    // Multi-segment encode/decode is bit-exact under every §II.A
    // filter; Q and F bracket the Table 1 parameter range.
    for filter in [WaveletFilter::FilterQ, WaveletFilter::FilterF] {
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
        assert_eq!(
            decoded.planes[0].data, original.planes[0].data,
            "filter {filter:?} multi-segment round-trip must be bit-exact"
        );
    }
}
