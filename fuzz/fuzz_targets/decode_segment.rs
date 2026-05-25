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

use libfuzzer_sys::fuzz_target;
use oxideav_icer::{parse_icer, parse_icer_metadata, walk_segment};

fuzz_target!(|data: &[u8]| {
    // Layer 1: pure framing on the first segment. Exercises
    // `SegmentHeader::parse` + `PacketHeader::parse` for every packet
    // in the first segment.
    let _ = walk_segment(data);

    // Layer 2: multi-segment walk. Iterates `walk_segment` across the
    // whole input until the bytes are exhausted or a parse error is
    // returned. Exercises the segment-stitching cursor arithmetic.
    let _ = parse_icer_metadata(data);

    // Layer 3: full decode. Drives the arithmetic coder + inverse
    // wavelet + plane reconstruction. The wire-claimed geometry feeds
    // an allocator, so this catches attacker-controlled allocation
    // sizing bugs in addition to entropy-stage panics.
    let _ = parse_icer(data);
});
