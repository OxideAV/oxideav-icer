//! IPN 42-155 §V.C "Choosing the Number of Segments" — automatic
//! segment-count selection.
//!
//! §V.C pins the operating envelope (MER ≤32 cap with tighter limits
//! for small images / deep decompositions, the four-to-six sweet spot,
//! 1 segment for rare losses / small images, and the image-size /
//! byte-volume / channel-reliability scaling axes); these tests pin
//! `analyze::recommend_segment_count`'s documented decision tree and
//! the `EncodeOptions::with_auto_segments` encode-time resolution
//! end to end through the crate's own decoder, including the
//! loss-tolerant truncated-stream path.

use oxideav_icer::{
    encode_icer, parse_icer, parse_icer_lenient, recommend_segment_count, walk_segment,
    ChannelReliability, EncodeOptions, IcerError, IcerImage, IcerPixelFormat,
};

fn ramp_image(w: u32, h: u32) -> IcerImage {
    let mut img = IcerImage::zeros(w, h, IcerPixelFormat::Gray8);
    let stride = img.planes[0].stride;
    for y in 0..h as usize {
        for x in 0..w as usize {
            img.planes[0].data[y * stride + x] = ((x * 3 + y * 7) & 0xFF) as u8;
        }
    }
    img
}

fn deep_image(w: u32, h: u32, bits: u8) -> IcerImage {
    let mut img = IcerImage::zeros(w, h, IcerPixelFormat::GrayDeep { bits });
    let mask = (1u32 << bits) - 1;
    for y in 0..h {
        for x in 0..w {
            img.set_sample(0, x, y, ((x * 7 + y * 13) & mask) as u16);
        }
    }
    img
}

/// Byte ranges of the segments in a bare (single-plane) stream.
fn segment_byte_ranges(bytes: &[u8]) -> Vec<std::ops::Range<usize>> {
    let mut ranges = Vec::new();
    let mut cursor = 0usize;
    while cursor < bytes.len() {
        let walked = walk_segment(&bytes[cursor..]).expect("walk_segment");
        ranges.push(cursor..cursor + walked.consumed);
        cursor += walked.consumed;
    }
    ranges
}

// ---------------------------------------------------------------
// The §V.C decision tree, pinned value by value.
// ---------------------------------------------------------------

#[test]
fn small_image_recommends_single_segment() {
    // §V.C: "If packet losses are rare or when compressing a small
    // image, one might reasonably set the number of segments to 1."
    for ch in [ChannelReliability::Reliable, ChannelReliability::Typical] {
        assert_eq!(recommend_segment_count(32, 32, 3, None, ch), 1, "{ch:?}");
    }
}

#[test]
fn typical_channel_hits_the_four_to_six_sweet_spot() {
    // §V.C: "Many images are most effectively compressed using four to
    // six segments" — the Typical base steps 4 / 5 / 6 with area.
    assert_eq!(
        recommend_segment_count(128, 128, 3, None, ChannelReliability::Typical),
        4
    );
    assert_eq!(
        recommend_segment_count(256, 256, 3, None, ChannelReliability::Typical),
        5
    );
    assert_eq!(
        recommend_segment_count(1024, 1024, 3, None, ChannelReliability::Typical),
        6
    );
}

#[test]
fn reliable_channel_prefers_one_but_segments_large_images() {
    assert_eq!(
        recommend_segment_count(256, 256, 3, None, ChannelReliability::Reliable),
        1
    );
    // "some amount of segmentation may slightly improve compression
    // effectiveness, especially on large images"
    assert_eq!(
        recommend_segment_count(1024, 1024, 3, None, ChannelReliability::Reliable),
        4
    );
}

#[test]
fn lossy_channel_scales_up() {
    assert_eq!(
        recommend_segment_count(256, 256, 3, None, ChannelReliability::Lossy),
        10
    );
    assert_eq!(
        recommend_segment_count(2048, 2048, 3, None, ChannelReliability::Lossy),
        12
    );
}

#[test]
fn eq9_ll_area_cap_tightens_small_deep_decompositions() {
    // §V.C: "tighter limits when the image is small and the number of
    // stages of wavelet decomposition is large" — realised via the
    // §V.D eq (9) validity cap on the LL subband (16/2^6 -> 1x1).
    assert_eq!(
        recommend_segment_count(16, 16, 6, None, ChannelReliability::Lossy),
        1
    );
}

#[test]
fn row_strip_two_row_minimum_caps_short_images() {
    // A 6-row image can hold at most 3 two-row strips.
    assert_eq!(
        recommend_segment_count(512, 6, 1, None, ChannelReliability::Lossy),
        3
    );
}

#[test]
fn byte_volume_axis_bounds_the_pick() {
    // Tiny quota: ~1 KiB of compressed data per segment floor.
    assert_eq!(
        recommend_segment_count(1024, 1024, 3, Some(2048), ChannelReliability::Lossy),
        2
    );
    // Huge quota raises the pick to the MER cap ("larger numbers of
    // compressed bytes" -> more segments), never beyond 32.
    assert_eq!(
        recommend_segment_count(1024, 1024, 3, Some(1 << 20), ChannelReliability::Typical),
        32
    );
}

#[test]
fn never_exceeds_the_mer_cap() {
    for area_side in [64u32, 512, 4096] {
        for ch in [
            ChannelReliability::Reliable,
            ChannelReliability::Typical,
            ChannelReliability::Lossy,
        ] {
            for bytes in [None, Some(1u64 << 30)] {
                let s = recommend_segment_count(area_side, area_side, 3, bytes, ch);
                assert!((1..=32).contains(&s), "{area_side} {ch:?} {bytes:?} -> {s}");
            }
        }
    }
}

// ---------------------------------------------------------------
// Encode-time resolution, round-tripped through the crate's decoder.
// ---------------------------------------------------------------

#[test]
fn auto_segments_row_strip_roundtrip_bit_exact() {
    let img = ramp_image(96, 64);
    let opts = EncodeOptions::compressed().with_auto_segments(ChannelReliability::Typical);
    let bytes = encode_icer(&img, &opts).expect("encode");
    // 96*64 = 6144 px -> Typical base 4; walk the wire and confirm.
    assert_eq!(segment_byte_ranges(&bytes).len(), 4);
    let decoded = parse_icer(&bytes).expect("decode");
    assert_eq!(
        decoded.planes[0].data, img.planes[0].data,
        "filter Q is lossless"
    );
}

#[test]
fn auto_segments_transform_domain_roundtrip_bit_exact() {
    let img = ramp_image(128, 128);
    let opts = EncodeOptions::compressed()
        .with_transform_domain_segments()
        .with_auto_segments(ChannelReliability::Typical);
    let bytes = encode_icer(&img, &opts).expect("encode");
    assert_eq!(segment_byte_ranges(&bytes).len(), 4);
    let decoded = parse_icer(&bytes).expect("decode");
    assert_eq!(decoded.planes[0].data, img.planes[0].data);
}

#[test]
fn auto_segments_deep_sample_roundtrip_bit_exact() {
    let img = deep_image(64, 64, 12);
    let opts = EncodeOptions::compressed().with_auto_segments(ChannelReliability::Typical);
    let bytes = encode_icer(&img, &opts).expect("encode");
    let decoded = parse_icer(&bytes).expect("decode");
    assert_eq!(decoded.pixel_format, IcerPixelFormat::GrayDeep { bits: 12 });
    assert_eq!(decoded.planes[0].data, img.planes[0].data);
}

#[test]
fn auto_segments_respects_byte_budget() {
    let img = ramp_image(256, 256);
    let opts = EncodeOptions::compressed()
        .with_auto_segments(ChannelReliability::Lossy)
        .with_byte_budget(20_000);
    let bytes = encode_icer(&img, &opts).expect("encode");
    assert!(
        bytes.len() as u64 <= 20_000,
        "hard cap holds: {}",
        bytes.len()
    );
    // base 10, byte-volume bounds keep it at 10; placeholders keep the
    // full frame geometry on the wire.
    assert_eq!(segment_byte_ranges(&bytes).len(), 10);
    let decoded = parse_icer(&bytes).expect("decode");
    assert_eq!(decoded.width, 256);
    assert_eq!(decoded.height, 256);
}

#[test]
fn auto_segments_truncated_stream_decodes_leniently() {
    // The §V.C point of segmentation is loss tolerance: cut the stream
    // on a segment boundary and salvage the delivered prefix.
    let img = ramp_image(128, 128);
    let opts = EncodeOptions::compressed().with_auto_segments(ChannelReliability::Typical);
    let bytes = encode_icer(&img, &opts).expect("encode");
    let ranges = segment_byte_ranges(&bytes);
    assert_eq!(ranges.len(), 4);
    let cut = ranges[1].end; // keep segments 0 and 1
    let lenient = parse_icer_lenient(&bytes[..cut]).expect("lenient decode");
    assert_eq!(lenient.image.width, 128);
    // Trailing drops truncate the image at the highest received strip
    // boundary (strip height 32).
    assert_eq!(lenient.image.height, 64);
    let stride = lenient.image.planes[0].stride;
    let orig_stride = img.planes[0].stride;
    for y in 0..64usize {
        assert_eq!(
            &lenient.image.planes[0].data[y * stride..y * stride + 128],
            &img.planes[0].data[y * orig_stride..y * orig_stride + 128],
            "delivered strips stay bit-exact (row {y})"
        );
    }
}

#[test]
fn auto_segments_rejects_roi_priorities() {
    let img = ramp_image(64, 64);
    let mut opts = EncodeOptions::compressed().with_auto_segments(ChannelReliability::Typical);
    opts.segment_count = 4;
    let opts = opts.with_center_roi();
    match encode_icer(&img, &opts) {
        Err(IcerError::Unsupported(_)) => {}
        other => panic!("expected Unsupported, got {other:?}"),
    }
}

#[test]
fn auto_segments_overrides_explicit_count() {
    let img = ramp_image(96, 64);
    let mut opts = EncodeOptions::compressed().with_auto_segments(ChannelReliability::Typical);
    opts.segment_count = 9;
    let bytes = encode_icer(&img, &opts).expect("encode");
    assert_eq!(
        segment_byte_ranges(&bytes).len(),
        4,
        "auto resolution overrides the explicit count, mirroring auto_filter"
    );
}
