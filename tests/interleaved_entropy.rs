//! End-to-end coverage for the IPN 42-155 §IV interleaved entropy coder
//! wired through the full `encode_icer` / `parse_icer` pipeline (selected
//! via `EncodeOptions::with_interleaved_entropy`).

use oxideav_icer::{
    encode_icer, parse_icer, parse_icer_metadata, EncodeOptions, IcerImage, IcerPixelFormat,
    SegmentHeader,
};

fn ramp_image(w: u32, h: u32) -> IcerImage {
    let mut img = IcerImage::zeros(w, h, IcerPixelFormat::Gray8);
    let plane = &mut img.planes[0];
    for y in 0..h as usize {
        for x in 0..w as usize {
            plane.data[y * plane.stride + x] = ((x * 3 + y * 5) & 0xFF) as u8;
        }
    }
    img
}

fn lcg_image(w: u32, h: u32, seed: u64) -> IcerImage {
    let mut img = IcerImage::zeros(w, h, IcerPixelFormat::Gray8);
    let plane = &mut img.planes[0];
    let mut s = seed;
    for byte in plane.data.iter_mut() {
        s = s
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        *byte = (s >> 33) as u8;
    }
    img
}

/// Filter-Q (lossless integer 5/3) full-quality round-trip through the
/// §IV interleaved entropy coder must be bit-exact, exactly as the
/// arithmetic-coded path is.
#[test]
fn interleaved_filter_q_is_bit_exact() {
    for (w, h) in [(16u32, 16u32), (31, 17), (64, 48), (5, 40)] {
        let original = ramp_image(w, h);
        let opts = EncodeOptions::compressed().with_interleaved_entropy(); // filter Q is the compressed() default
        let bytes = encode_icer(&original, &opts).unwrap();
        let decoded = parse_icer(&bytes).unwrap();
        assert_eq!(decoded.width, w, "{w}x{h} width");
        assert_eq!(decoded.height, h, "{w}x{h} height");
        assert_eq!(
            decoded.planes[0].data, original.planes[0].data,
            "{w}x{h} interleaved filter-Q must be bit-exact"
        );
    }
}

/// Random (high-entropy) content also round-trips losslessly under
/// filter Q + the interleaved coder.
#[test]
fn interleaved_random_content_bit_exact() {
    let original = lcg_image(48, 48, 0xC0FFEE);
    let opts = EncodeOptions::compressed().with_interleaved_entropy();
    let bytes = encode_icer(&original, &opts).unwrap();
    let decoded = parse_icer(&bytes).unwrap();
    assert_eq!(decoded.planes[0].data, original.planes[0].data);
}

/// The encoder sets the segment-header entropy-backend flag when the
/// interleaved coder is selected, and clears it otherwise. The decoder
/// dispatches on that flag, so a stream encoded one way cannot be
/// silently misread the other.
#[test]
fn interleaved_flag_is_recorded_on_the_wire() {
    let original = ramp_image(32, 32);

    let inter = encode_icer(
        &original,
        &EncodeOptions::compressed().with_interleaved_entropy(),
    )
    .unwrap();
    let (hdr_inter, _) = SegmentHeader::parse(&inter).unwrap();
    assert!(
        hdr_inter.interleaved_entropy,
        "interleaved encode must set the wire flag"
    );

    let arith = encode_icer(&original, &EncodeOptions::compressed()).unwrap();
    let (hdr_arith, _) = SegmentHeader::parse(&arith).unwrap();
    assert!(
        !hdr_arith.interleaved_entropy,
        "arithmetic encode must clear the wire flag"
    );

    // The two wire forms differ (distinct entropy stages).
    assert_ne!(
        inter, arith,
        "the two entropy backends produce distinct byte streams"
    );
}

/// Multi-segment images coded with the interleaved backend round-trip
/// bit-exactly and the metadata walker still enumerates every segment.
#[test]
fn interleaved_multi_segment_bit_exact() {
    let original = ramp_image(40, 40);
    let mut opts = EncodeOptions::compressed().with_interleaved_entropy();
    opts.segment_count = 4;
    let bytes = encode_icer(&original, &opts).unwrap();

    let meta = parse_icer_metadata(&bytes).unwrap();
    assert_eq!(meta.segments.len(), 4, "four segments enumerated");
    for seg in &meta.segments {
        assert!(
            seg.header.interleaved_entropy,
            "every compressed segment carries the interleaved flag"
        );
    }

    let decoded = parse_icer(&bytes).unwrap();
    assert_eq!(decoded.planes[0].data, original.planes[0].data);
}

/// Colour (YUV 4:4:4) filter-Q round-trip through the interleaved coder
/// is bit-exact across all three planes.
#[test]
fn interleaved_colour_bit_exact() {
    let mut original = IcerImage::zeros(24, 24, IcerPixelFormat::Yuv444P);
    for (p, plane) in original.planes.iter_mut().enumerate() {
        for y in 0..24usize {
            for x in 0..24usize {
                plane.data[y * plane.stride + x] = ((x * 7 + y * 11 + p * 40) & 0xFF) as u8;
            }
        }
    }
    let opts = EncodeOptions::compressed().with_interleaved_entropy();
    let bytes = encode_icer(&original, &opts).unwrap();
    let decoded = parse_icer(&bytes).unwrap();
    assert_eq!(decoded.pixel_format, IcerPixelFormat::Yuv444P);
    assert_eq!(decoded.planes.len(), 3);
    for (p, (d, o)) in decoded
        .planes
        .iter()
        .zip(original.planes.iter())
        .enumerate()
    {
        assert_eq!(d.data, o.data, "interleaved colour plane {p} bit-exact");
    }
}

/// Feeding the interleaved decode path a stream whose packet bodies are
/// garbage (header declares the interleaved backend, bodies are random)
/// must never panic — it decodes *something* (a bounded reconstruction)
/// or returns an error, but stays memory-safe. Guards the fuzz surface
/// the new backend opens via `parse_icer`.
#[test]
fn interleaved_garbage_body_does_not_panic() {
    // Build a legitimate interleaved stream, then corrupt every body byte.
    let original = ramp_image(32, 32);
    let opts = EncodeOptions::compressed().with_interleaved_entropy();
    let mut bytes = encode_icer(&original, &opts).unwrap();
    // Corrupt the payload after the 12-byte segment header.
    let mut s = 0x1357u64;
    for b in bytes.iter_mut().skip(SegmentHeader::ENCODED_BYTES) {
        s = s.wrapping_mul(2862933555777941757).wrapping_add(3037000493);
        *b = (s >> 40) as u8;
    }
    // Must not panic; either decodes a (garbage) image or errors cleanly.
    let _ = parse_icer(&bytes);
}

/// On structured content the §IV interleaved coder compresses the
/// significance/refinement bit stream — the lossless filter-Q output for
/// a smooth ramp is well under the raw pixel count, just like the
/// arithmetic backend.
#[test]
fn interleaved_compresses_structured_content() {
    let original = ramp_image(64, 64);
    let raw_pixels = 64 * 64;
    let bytes = encode_icer(
        &original,
        &EncodeOptions::compressed().with_interleaved_entropy(),
    )
    .unwrap();
    assert!(
        bytes.len() < raw_pixels,
        "interleaved filter-Q lossless ({} bytes) should beat the {raw_pixels}-byte raw image",
        bytes.len()
    );
    // And it round-trips losslessly.
    let decoded = parse_icer(&bytes).unwrap();
    assert_eq!(decoded.planes[0].data, original.planes[0].data);
}

/// A budget-truncated interleaved-coded stream still decodes (lower
/// quality) and frames the full geometry, exactly like the arithmetic
/// path — progressive truncation is a property of the packet ordering,
/// not the entropy stage.
#[test]
fn interleaved_budget_truncation_frames_geometry() {
    let original = ramp_image(64, 64);
    let opts = EncodeOptions::compressed()
        .with_interleaved_entropy()
        .with_byte_budget(400);
    let bytes = encode_icer(&original, &opts).unwrap();
    assert!(
        bytes.len() <= 400,
        "hard cap honoured: {} bytes",
        bytes.len()
    );
    let decoded = parse_icer(&bytes).unwrap();
    assert_eq!(decoded.width, 64);
    assert_eq!(decoded.height, 64);
}
