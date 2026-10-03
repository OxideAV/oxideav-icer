#![no_main]

//! Decode-side fuzz harness for the ICER framing + entropy parsers.
//!
//! Every byte slice is fed through the contract entry points
//! (`probe` / `info` / `decode` / `decode_all`) and the depth decoders:
//!
//! 0. [`oxideav_icer::probe`] — the allocation-free structural sniff.
//! 1. [`oxideav_icer::walk_segment`] — single-segment framing parse;
//!    surfaces header + packet boundaries without running the entropy
//!    stage.
//! 2. [`oxideav_icer::info`] — multi-segment walk
//!    returning only header-level metadata for every segment in the
//!    stream.
//! 3. [`oxideav_icer::decode`] / [`oxideav_icer::decode_all`] — full
//!    decode (framing + arithmetic coder + inverse wavelet +
//!    multi-segment stitch; one frame per cube band), followed by the
//!    `to_rgb8` / `to_rgba8` conversions.
//!
//! The contract under test is that every entry point *returns* — a
//! malformed stream produces `Err(IcerError::…)`, a well-formed one
//! produces `Ok(…)`, and neither path may panic, integer-overflow (in
//! a debug build), index out of bounds, or try to allocate an
//! attacker-controlled buffer the size of the wire-claimed
//! `width * height * planes`. Return values are intentionally
//! discarded.
//!
//! **Geometry budget.** A single 12-byte segment header can legitimately
//! declare a geometry up to the [`DecodeOptions`] cap (64 MPx per segment
//! by default), and a *valid* compressed segment at that geometry runs
//! the inverse DWT + bit-plane scan over the full coefficient buffer
//! regardless of how few packet body bytes survive (this is the
//! progressive-truncation feature: a tiny body is normal). At the
//! default 64 MPx cap a single crafted header therefore costs tens of
//! seconds of *legitimate, bounded* decode work — which libFuzzer flags
//! as a `slow-unit` and counts toward the run's wall-clock budget,
//! drowning out the framing/entropy exploration the target is for.
//!
//! The harness uses a tight per-run [`DecodeOptions`] (1 MPx / segment,
//! 4 MPx total) for the full-decode layer so each iteration stays in the
//! millisecond range while still exercising the allocator, the inverse
//! DWT, the arithmetic coder and the multi-segment stitch. The framing
//! layers (`walk_segment`, `info`) are header-only and
//! cheap at any geometry, so they keep the default-limits public entry
//! points for coverage of the geometry-validation refusal path.

use libfuzzer_sys::fuzz_target;
use oxideav_icer::{
    decode_all_with, decode_with, info, parse_icer3d_with, parse_icer_lenient_with, probe,
    walk_segment, DecodeOptions,
};

/// Per-iteration geometry budget. Far below the public 64 MPx default so
/// a single crafted header cannot make one iteration dominate the run's
/// wall-clock budget, but well above any geometry a realistic seed/corpus
/// entry needs to drive the full decode path.
fn fuzz_limits() -> DecodeOptions {
    DecodeOptions::new().with_max_pixels_per_segment(1u64 << 20).with_max_pixels(1u64 << 22)
}

fuzz_target!(|data: &[u8]| {
    // Layer 0: the contract sniff. Total and allocation-free by
    // contract; must never panic.
    let plausible = probe(data);

    // Layer 1: pure framing on the first segment. Exercises
    // `SegmentHeader::parse` + `PacketHeader::parse` for every packet
    // in the first segment.
    let _ = walk_segment(data);

    // Layer 2: multi-segment walk under the DEFAULT limits. Header-only
    // (no pixel buffers materialised), so it is cheap at any geometry and
    // keeps coverage of the default-limits geometry-validation refusal
    // path the public API enforces. A stream `info` accepts must have
    // been accepted by `probe` (the sniff is a superset of the walk).
    if let Ok(i) = info(data) {
        assert!(plausible, "info accepted a stream probe rejected");
        assert!(i.frames >= 1);
    }

    // Layer 3: full decode under the tight per-run geometry budget.
    // Drives the arithmetic coder + inverse wavelet + plane
    // reconstruction + multi-segment stitch. The tight cap keeps each
    // iteration in the millisecond range while still catching
    // attacker-controlled allocation sizing bugs and entropy-stage
    // panics. The RGB conversions must be infallible on anything the
    // decoder produced, and `decode_all` must agree with `decode` on the
    // first image.
    if let Ok(img) = decode_with(data, &fuzz_limits()) {
        let rgb = img.to_rgb8();
        assert_eq!(rgb.len(), img.width as usize * img.height as usize * 3);
        assert_eq!(img.to_rgba8().len(), rgb.len() / 3 * 4);
        let all = decode_all_with(data, &fuzz_limits()).expect("decode_all follows decode");
        assert!(!all.is_empty());
        assert_eq!(all[0].image, img);
    }
    let _ = parse_icer_lenient_with(data, &fuzz_limits());

    // Layer 4: the ICER-3D cube decoder (IPN 42-164) under the same
    // tight budget — its 0x0000 + 0xC3 magic never collides with the
    // 2-D layers, so this costs nothing on non-cube inputs while giving
    // the cube framing parser + 3-D inverse DWT + spectral-context
    // bit-plane decoder full corpus coverage.
    let _ = parse_icer3d_with(data, &fuzz_limits());
    let _ = oxideav_icer::parse_icer3d_lenient_with(data, &fuzz_limits());
});
