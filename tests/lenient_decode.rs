//! Lenient multi-segment decode — tolerate missing segments due to
//! transit packet loss (IPN 42-155 §III.E independent-segment
//! scheduling).
//!
//! The Mars-rover deep-space link is lossy: individual ICER segments
//! can be dropped between the orbiter relay and the DSN ground
//! station. ICER's §III.E partitioning makes each segment
//! self-contained, so a receiver can still recover most of the image
//! by treating dropped segments as flat-128 placeholders. This test
//! file covers the [`parse_icer_lenient`] entry point that surfaces
//! that behaviour.

use oxideav_icer::{
    encode_icer, parse_icer, parse_icer_lenient, parse_icer_lenient_with_limits, DecodeLimits,
    EncodeOptions, IcerError, IcerImage, IcerPixelFormat, SegmentHeader, WaveletFilter,
};

fn fill<F>(w: u32, h: u32, mut f: F) -> IcerImage
where
    F: FnMut(usize, usize) -> u8,
{
    let mut img = IcerImage::zeros(w, h, IcerPixelFormat::Gray8);
    let plane = &mut img.planes[0];
    let stride = plane.stride;
    for y in 0..h as usize {
        for x in 0..w as usize {
            plane.data[y * stride + x] = f(x, y);
        }
    }
    img
}

/// Smooth diagonal ramp -- compressible by the entropy stage so the
/// per-strip body is small.
fn ramp_image(w: u32, h: u32) -> IcerImage {
    fill(w, h, |x, y| ((x + y) & 0xFF) as u8)
}

/// Walk the encoded bytestream and split it into the per-segment slice
/// boundaries so we can drop or reorder individual segments.
fn segment_byte_ranges(bytes: &[u8]) -> Vec<std::ops::Range<usize>> {
    let mut ranges = Vec::new();
    let mut cursor = 0usize;
    while cursor < bytes.len() {
        let walked = oxideav_icer::walk_segment(&bytes[cursor..]).expect("walk_segment");
        ranges.push(cursor..cursor + walked.consumed);
        cursor += walked.consumed;
    }
    ranges
}

fn drop_segment(bytes: &[u8], seg_index_to_drop: u16) -> Vec<u8> {
    let mut out = Vec::with_capacity(bytes.len());
    let mut cursor = 0usize;
    while cursor < bytes.len() {
        let walked = oxideav_icer::walk_segment(&bytes[cursor..]).expect("walk_segment");
        if walked.header.segment_index != seg_index_to_drop {
            out.extend_from_slice(&bytes[cursor..cursor + walked.consumed]);
        }
        cursor += walked.consumed;
    }
    out
}

#[test]
fn lenient_no_loss_matches_strict_parse() {
    // With every segment received, parse_icer_lenient must produce a
    // bit-identical image to parse_icer and report zero missing.
    let img = ramp_image(16, 12);
    let mut opts = EncodeOptions::compressed();
    opts.segment_count = 4;
    let bytes = encode_icer(&img, &opts).expect("encode");

    let strict = parse_icer(&bytes).expect("strict parse");
    let lenient = parse_icer_lenient(&bytes).expect("lenient parse");

    assert_eq!(lenient.missing_count, 0, "no segments should be missing");
    assert_eq!(
        lenient.received,
        vec![true, true, true, true],
        "all four segments should be present"
    );
    assert_eq!(lenient.image, strict, "lenient image must match strict");
}

#[test]
fn lenient_drops_middle_segment_to_flat_128() {
    // Encode a 4-strip image, drop segment 2 from the bytestream, and
    // confirm the lenient decoder fills its strip with 128.
    let img = ramp_image(16, 16); // 4 strips of 4 rows each.
    let mut opts = EncodeOptions::compressed();
    opts.segment_count = 4;
    let bytes = encode_icer(&img, &opts).expect("encode");
    let lossy = drop_segment(&bytes, 2);

    // Strict parse must reject the gap.
    let strict_err = parse_icer(&lossy).expect_err("strict must reject gap");
    let strict_msg = format!("{strict_err}");
    assert!(
        strict_msg.contains("non-contiguous") || strict_msg.contains("contiguous"),
        "strict error should mention contiguity, got: {strict_msg}"
    );

    // Lenient parse must succeed.
    let lenient = parse_icer_lenient(&lossy).expect("lenient parse");
    assert_eq!(lenient.missing_count, 1);
    assert_eq!(lenient.received, vec![true, true, false, true]);

    // The missing strip is rows 8..12 (segment 2 starts at row 2 *
    // strip_h_of_seg_0). strip_h here is segment 0's height = 4.
    let plane = &lenient.image.planes[0];
    let stride = plane.stride;
    for y in 8..12 {
        for x in 0..16 {
            assert_eq!(
                plane.data[y * stride + x],
                128,
                "missing strip should be flat 128 at ({x},{y})"
            );
        }
    }

    // Received strips (segments 0, 1, 3) should round-trip close to
    // the original ramp -- filter Q on Reversible53 is lossless on
    // smooth integer-rounded ramp coefficients.
    for y in 0..8 {
        for x in 0..16 {
            assert_eq!(
                plane.data[y * stride + x],
                img.planes[0].data[y * img.planes[0].stride + x],
                "received strip mismatch at ({x},{y})"
            );
        }
    }
    // Segment 3 covers rows 12..16.
    for y in 12..16 {
        for x in 0..16 {
            assert_eq!(
                plane.data[y * stride + x],
                img.planes[0].data[y * img.planes[0].stride + x],
                "received trailing strip mismatch at ({x},{y})"
            );
        }
    }
}

#[test]
fn lenient_drops_trailing_segment_truncates_image() {
    // Drop the highest-index segment. The lenient decoder reports a
    // shorter image (truncated at the highest-received boundary) with
    // zero missing entries (because the lost segment is *beyond* the
    // observed max index — we don't know it was supposed to be there).
    let img = ramp_image(16, 16); // 4 strips of 4 rows each.
    let mut opts = EncodeOptions::compressed();
    opts.segment_count = 4;
    let bytes = encode_icer(&img, &opts).expect("encode");
    let lossy = drop_segment(&bytes, 3);

    let lenient = parse_icer_lenient(&lossy).expect("lenient parse");
    // We can't detect that segment 3 was supposed to exist; image is
    // truncated at end of segment 2.
    assert_eq!(lenient.missing_count, 0);
    assert_eq!(lenient.received, vec![true, true, true]);
    assert_eq!(lenient.image.height, 12, "trailing-drop truncates image");
}

#[test]
fn lenient_requires_segment_zero() {
    // Drop segment 0: lenient parse returns Truncated because it has
    // no way to pin the canonical strip height.
    let img = ramp_image(16, 16);
    let mut opts = EncodeOptions::compressed();
    opts.segment_count = 4;
    let bytes = encode_icer(&img, &opts).expect("encode");
    let lossy = drop_segment(&bytes, 0);

    let err = parse_icer_lenient(&lossy).expect_err("no segment 0 must fail");
    assert!(
        matches!(err, IcerError::Truncated),
        "expected Truncated, got {err:?}"
    );
}

#[test]
fn lenient_rejects_width_mismatch() {
    // Synthesise a two-segment bytestream where segment 1 advertises a
    // different width. The lenient API must still error on geometry
    // contradiction.
    let img = ramp_image(16, 8); // 2 strips of 4 rows.
    let mut opts = EncodeOptions::compressed();
    opts.segment_count = 2;
    let bytes = encode_icer(&img, &opts).expect("encode");

    let ranges = segment_byte_ranges(&bytes);
    assert_eq!(ranges.len(), 2);

    // Forge: rewrite segment 1's width field. The segment header is
    // SegmentHeader::ENCODED_BYTES = 12 bytes at the start of the
    // segment; width sits in bytes 5..7 (sync_prefix:2 + flags:1 +
    // bit_plane_count:1 + segment_index:1 == 5 + 2 bytes width).
    // Round 1's wire layout is documented in src/header.rs. Rather
    // than hand-reach into the bytes, decode + re-encode segment 1's
    // header with the wrong width.
    let walked_1 = oxideav_icer::walk_segment(&bytes[ranges[1].clone()]).unwrap();
    let mut hdr_1 = walked_1.header;
    hdr_1.width = (canonical_width(&bytes) + 1) as u16;
    let mut forged = Vec::new();
    forged.extend_from_slice(&bytes[ranges[0].clone()]);
    forged.extend_from_slice(&hdr_1.encode());
    // Copy the body of segment 1 (after its 12-byte header).
    forged.extend_from_slice(&bytes[ranges[1].start + SegmentHeader::ENCODED_BYTES..ranges[1].end]);

    let err = parse_icer_lenient(&forged).expect_err("width mismatch must error");
    let msg = format!("{err}");
    assert!(
        msg.contains("width mismatch"),
        "expected width-mismatch error, got: {msg}"
    );
}

fn canonical_width(bytes: &[u8]) -> usize {
    oxideav_icer::walk_segment(bytes).unwrap().header.width as usize
}

#[test]
fn lenient_respects_decode_limits() {
    // Explicit limits surface through the lenient API.
    let img = ramp_image(16, 16);
    let mut opts = EncodeOptions::compressed();
    opts.segment_count = 4;
    let bytes = encode_icer(&img, &opts).expect("encode");

    // 8 pixels per segment cap rejects any segment (each one is 16x4 = 64 px).
    let tight = DecodeLimits {
        max_pixels_per_segment: 8,
        max_total_pixels: u64::MAX,
    };
    let err = parse_icer_lenient_with_limits(&bytes, &tight)
        .expect_err("tight limits must reject the first segment");
    let msg = format!("{err}");
    assert!(
        msg.contains("exceeds per-segment cap"),
        "expected per-segment cap error, got: {msg}"
    );

    // Default limits accept it.
    let ok = parse_icer_lenient_with_limits(&bytes, &DecodeLimits::default()).expect("default ok");
    assert_eq!(ok.missing_count, 0);
}

#[test]
fn lenient_uncompressed_path_also_supported() {
    // Multi-segment uncompressed encode + drop a middle segment.
    let img = ramp_image(16, 16);
    let opts = EncodeOptions {
        uncompressed: true,
        segment_count: 4,
        ..EncodeOptions::default()
    };
    let bytes = encode_icer(&img, &opts).expect("encode");
    let lossy = drop_segment(&bytes, 1);

    let lenient = parse_icer_lenient(&lossy).expect("lenient parse uncompressed");
    assert_eq!(lenient.missing_count, 1);
    assert_eq!(lenient.received, vec![true, false, true, true]);

    // Strip 1 (rows 4..8) is the dropped one -- flat 128.
    let plane = &lenient.image.planes[0];
    let stride = plane.stride;
    for y in 4..8 {
        for x in 0..16 {
            assert_eq!(
                plane.data[y * stride + x],
                128,
                "dropped uncompressed strip must be 128"
            );
        }
    }
    // Surviving uncompressed strips are bit-exact.
    for y in 0..4 {
        for x in 0..16 {
            assert_eq!(
                plane.data[y * stride + x],
                img.planes[0].data[y * img.planes[0].stride + x]
            );
        }
    }
    for y in 8..16 {
        for x in 0..16 {
            assert_eq!(
                plane.data[y * stride + x],
                img.planes[0].data[y * img.planes[0].stride + x]
            );
        }
    }
}

#[test]
fn lenient_filter_a_round_trip() {
    // Float-filter path: drop a middle segment from a filter-A encoding.
    let img = ramp_image(16, 12);
    let opts = EncodeOptions {
        filter: WaveletFilter::NineSevenA,
        segment_count: 3,
        ..EncodeOptions::compressed()
    };
    let bytes = encode_icer(&img, &opts).expect("encode");
    let lossy = drop_segment(&bytes, 1);

    let lenient = parse_icer_lenient(&lossy).expect("lenient parse filter A");
    assert_eq!(lenient.missing_count, 1);
    // strip_h derived from segment 0: 12 / 3 = 4 rows.
    let plane = &lenient.image.planes[0];
    let stride = plane.stride;
    for y in 4..8 {
        for x in 0..16 {
            assert_eq!(
                plane.data[y * stride + x],
                128,
                "middle strip should be flat 128"
            );
        }
    }
}

#[test]
fn lenient_empty_bytes_returns_truncated() {
    let err = parse_icer_lenient(&[]).expect_err("empty must fail");
    assert!(matches!(err, IcerError::Truncated));
}

#[test]
fn lenient_rejects_duplicate_segment_indices() {
    // Scheduled-fuzz crash regression (also seeded at
    // fuzz/corpus/decode_segment/seed_lenient_duplicate_segment_index.bin):
    // two segments sharing an index are a geometry contradiction, not
    // packet loss. Pre-fix, concatenating two single-segment streams of
    // *different* heights made the lenient height inference take the
    // canonical strip height from the taller first segment (40 rows)
    // and the total height from the shorter duplicate (8 rows), then
    // write the 40-row strip past the 8-row plane. Both orderings and
    // the equal-height duplicate must now be refused, never panic.
    let tall = encode_icer(&ramp_image(32, 40), &EncodeOptions::compressed()).expect("encode 40");
    let short = encode_icer(&ramp_image(32, 8), &EncodeOptions::compressed()).expect("encode 8");

    for (a, b) in [(&tall, &short), (&short, &tall), (&tall, &tall)] {
        let mut concat = a.clone();
        concat.extend_from_slice(b);
        let err = parse_icer_lenient(&concat).expect_err("duplicate index must be refused");
        assert!(
            matches!(err, IcerError::Unsupported(_)),
            "unexpected error kind: {err:?}"
        );
        // The strict decoder already refuses via its contiguity check.
        assert!(parse_icer(&concat).is_err());
    }
}
