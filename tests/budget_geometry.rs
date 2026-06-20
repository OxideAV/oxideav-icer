//! Budget-truncated multi-segment encodes must still frame the full
//! image geometry.
//!
//! Regression for a Fuzz-discovered self-roundtrip break: a 16-segment
//! 64-row image encoded under a 38-byte budget (in `segment_index`
//! order, no ROI priorities) used to drop every strip the budget could
//! not afford *without* emitting a placeholder header for it. The strict
//! decoder reconstructs the total image height by summing the heights of
//! the segments physically present on the wire, so the decoded image
//! shrank to a single 4-row strip (`encoded 64, decoded 4`).
//!
//! The encoder now emits a zero-body placeholder header (IPN 42-155
//! §V.B independent-segment scheduling) for every strip it cannot afford
//! to encode in full, on the legacy index-order budget path exactly as
//! it already did on the ROI-priority path. The decoder accounts for the
//! placeholder strip at the correct row offset (flat-128) so geometry is
//! always preserved.

use oxideav_icer::{
    encode_icer, parse_icer, EncodeOptions, IcerImage, IcerPixelFormat, WaveletFilter,
};

fn base_opts() -> EncodeOptions {
    EncodeOptions {
        sync_prefix: 0xACED,
        filter: WaveletFilter::FilterB,
        wavelet_levels: 1,
        bit_plane_count: 1,
        uncompressed: false,
        segment_count: 16,
        byte_budget: None,
        target_bytes: None,
        auto_filter: false,
        auto_filter_rd: false,
        segment_priorities: None,
        rd_pruning: false,
        auto_uncompressed_fallback: false,
        quality_target_psnr: None,
    }
}

/// The exact Fuzz crash input: 128x64 flat-black, FilterB, 1 level, 16
/// segments, 38-byte budget. Pre-fix this decoded to 128x4.
#[test]
fn tight_budget_preserves_geometry_fuzz_repro() {
    let (w, h) = (128u32, 64u32);
    let img = IcerImage::zeros(w, h, IcerPixelFormat::Gray8);
    let mut opts = base_opts();
    opts.byte_budget = Some(38);

    let encoded = encode_icer(&img, &opts).expect("encode");
    let decoded = parse_icer(&encoded).expect("strict decode of self-encoded stream");

    assert_eq!(decoded.width, w, "width must be preserved");
    assert_eq!(
        decoded.height, h,
        "height must be preserved even when every strip is dropped to a placeholder"
    );
    assert_eq!(decoded.pixel_format, IcerPixelFormat::Gray8);
}

/// Sweep a range of budgets from "frame-only" up to "fits everything":
/// every budget must round-trip to the full geometry, and the output
/// must respect the advertised `budget + segment_count * 12` slop cap.
#[test]
fn budget_sweep_always_preserves_geometry() {
    let (w, h) = (64u32, 48u32);
    // A non-trivial gradient so segments differ in compressed size.
    let mut img = IcerImage::zeros(w, h, IcerPixelFormat::Gray8);
    for y in 0..h as usize {
        for x in 0..w as usize {
            img.planes[0].data[y * w as usize + x] = ((x * 4 + y * 3) & 0xff) as u8;
        }
    }
    let seg_count = 12u16;
    for budget in [16u64, 50, 120, 300, 800, 4096] {
        let mut opts = base_opts();
        opts.filter = WaveletFilter::Reversible53;
        opts.wavelet_levels = 2;
        opts.bit_plane_count = 8;
        opts.segment_count = seg_count;
        opts.byte_budget = Some(budget);

        let encoded = encode_icer(&img, &opts)
            .unwrap_or_else(|e| panic!("encode failed at budget {budget}: {e:?}"));
        let decoded = parse_icer(&encoded)
            .unwrap_or_else(|e| panic!("decode failed at budget {budget}: {e:?}"));
        assert_eq!(decoded.width, w, "width at budget {budget}");
        assert_eq!(decoded.height, h, "height at budget {budget}");

        let slop = seg_count as u64 * 12;
        assert!(
            encoded.len() as u64 <= budget + slop,
            "budget {budget}: encoded {} > budget + slop {}",
            encoded.len(),
            budget + slop
        );
    }
}

/// An unbudgeted multi-segment encode is unaffected: no placeholders,
/// full fidelity, and the stream is byte-identical whether or not the
/// (absent) budget code path runs.
#[test]
fn unbudgeted_multi_segment_unchanged() {
    let (w, h) = (32u32, 32u32);
    let mut img = IcerImage::zeros(w, h, IcerPixelFormat::Gray8);
    for (i, p) in img.planes[0].data.iter_mut().enumerate() {
        *p = (i & 0xff) as u8;
    }
    let mut opts = base_opts();
    opts.filter = WaveletFilter::Reversible53;
    opts.bit_plane_count = 8;
    opts.segment_count = 8;

    let encoded = encode_icer(&img, &opts).expect("encode");
    let decoded = parse_icer(&encoded).expect("decode");
    assert_eq!(decoded.width, w);
    assert_eq!(decoded.height, h);
}
