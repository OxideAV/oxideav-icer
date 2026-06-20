#![no_main]

//! Decode-side fuzz harness for the ICER framing + entropy parsers.
//!
//! Every byte slice is fed through three layers of the decode stack:
//!
//! 1. [`oxideav_icer::walk_segment`] — single-segment framing parse;
//!    surfaces header + packet boundaries without running the entropy
//!    stage.
//! 2. [`oxideav_icer::parse_icer_metadata`] — multi-segment walk
//!    returning only header-level metadata for every segment in the
//!    stream.
//! 3. [`oxideav_icer::parse_icer`] — full decode (framing + arithmetic
//!    coder + inverse wavelet + multi-segment stitch).
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
//! declare a geometry up to the [`DecodeLimits`] cap (64 MPx per segment
//! by default), and a *valid* compressed segment at that geometry runs
//! the inverse DWT + bit-plane scan over the full coefficient buffer
//! regardless of how few packet body bytes survive (this is the
//! progressive-truncation feature: a tiny body is normal). At the
//! default 64 MPx cap a single crafted header therefore costs tens of
//! seconds of *legitimate, bounded* decode work — which libFuzzer flags
//! as a `slow-unit` and counts toward the run's wall-clock budget,
//! drowning out the framing/entropy exploration the target is for.
//!
//! The harness uses a tight per-run [`DecodeLimits`] (1 MPx / segment,
//! 4 MPx total) for the full-decode layer so each iteration stays in the
//! millisecond range while still exercising the allocator, the inverse
//! DWT, the arithmetic coder and the multi-segment stitch. The framing
//! layers (`walk_segment`, `parse_icer_metadata`) are header-only and
//! cheap at any geometry, so they keep the default-limits public entry
//! points for coverage of the geometry-validation refusal path.

use libfuzzer_sys::fuzz_target;
use oxideav_icer::{
    parse_icer_lenient_with_limits, parse_icer_metadata, parse_icer_with_limits, walk_segment,
    DecodeLimits,
};

/// Per-iteration geometry budget. Far below the public 64 MPx default so
/// a single crafted header cannot make one iteration dominate the run's
/// wall-clock budget, but well above any geometry a realistic seed/corpus
/// entry needs to drive the full decode path.
const FUZZ_LIMITS: DecodeLimits = DecodeLimits {
    max_pixels_per_segment: 1 << 20, // 1 MPx
    max_total_pixels: 1 << 22,       // 4 MPx across all segments
};

fuzz_target!(|data: &[u8]| {
    // Layer 1: pure framing on the first segment. Exercises
    // `SegmentHeader::parse` + `PacketHeader::parse` for every packet
    // in the first segment.
    let _ = walk_segment(data);

    // Layer 2: multi-segment walk under the DEFAULT limits. Header-only
    // (no pixel buffers materialised), so it is cheap at any geometry and
    // keeps coverage of the default-limits geometry-validation refusal
    // path the public API enforces.
    let _ = parse_icer_metadata(data);

    // Layer 3: full decode under the tight per-run geometry budget.
    // Drives the arithmetic coder + inverse wavelet + plane
    // reconstruction + multi-segment stitch. The tight cap keeps each
    // iteration in the millisecond range while still catching
    // attacker-controlled allocation sizing bugs and entropy-stage
    // panics.
    let _ = parse_icer_with_limits(data, &FUZZ_LIMITS);
    let _ = parse_icer_lenient_with_limits(data, &FUZZ_LIMITS);
});
