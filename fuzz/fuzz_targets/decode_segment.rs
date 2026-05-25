#![no_main]

//! Decode arbitrary fuzz-supplied bytes through every standalone ICER
//! decode entry point. None of these calls may panic / abort /
//! integer-overflow (debug) / index out of bounds / OOM regardless of
//! how malformed the input is — they must always *return* a `Result`.
//!
//! ICER's decode path layers three entry points reachable from raw
//! bytes. `walk_segment` is the lowest-level outer framing parser
//! (IPN 42-155 §IV) that reads the 12-byte segment header and the
//! variable-length list of packet headers that follow it.
//! `parse_icer_metadata` walks every segment in the input buffer
//! back-to-back and returns a per-segment metadata report without
//! touching the entropy coder, exercising the cursor arithmetic that
//! stitches segments together. `parse_icer` is the full pixel
//! decoder — it demuxes segment headers, runs each packet through
//! the binary arithmetic coder, processes bit-planes through the
//! stripe-ordered significance + refinement scanner, dispatches the
//! inverse 5/3 integer or float A–G wavelet transform, and stitches
//! multi-segment strips vertically. Both the IPN 42-155 §III.D
//! uncompressed path and the compressed wavelet + bit-plane path
//! are reachable from the same entry point.
//!
//! Every return value is intentionally discarded — the contract
//! under test is *liveness*: malformed input yields
//! `Err(IcerError::…)`, well-formed input yields `Ok`, and neither
//! path may crash.

use libfuzzer_sys::fuzz_target;
use oxideav_icer::{header::walk_segment, parse_icer, parse_icer_metadata};

fuzz_target!(|data: &[u8]| {
    // Outer framing parser on a single segment slice. Most fuzz
    // inputs will fail here; the survivors widen coverage of the
    // packet-header walk.
    let _ = walk_segment(data);

    // Multi-segment metadata enumerator. Loops `walk_segment` over
    // the entire buffer; exercises the cursor arithmetic that stitches
    // segments back-to-back.
    let _ = parse_icer_metadata(data);

    // Full pixel decode. Reaches the binary arithmetic coder + the
    // bit-plane scanner + the inverse wavelet transform + the
    // multi-segment vertical-stitch path. The deepest attack surface
    // in the crate.
    let _ = parse_icer(data);
});
