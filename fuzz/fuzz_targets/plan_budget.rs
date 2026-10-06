#![no_main]

//! The decoder's memory plan under fuzz.
//!
//! `DecodeOptions::max_bytes` bounds the decoder's *planned peak working
//! set* (`ImageInfo::working_set_bytes`: output planes + the largest
//! per-segment coefficient buffer and coder state), computed from the
//! framing before anything is allocated. The first input byte selects a
//! byte budget (64 KiB … 16 MiB, the 1 GiB default, unlimited); the rest
//! is the stream. Properties checked on every input:
//!
//! * `info_with` and `decode_all_with` agree on budget refusals — the
//!   header-only planner never accepts a stream the decoder then refuses
//!   on a limit, and the decoder never decodes a stream the planner
//!   refused (the policy cannot be bypassed by skipping `info`);
//! * whenever `info` accepts, the plan is within the budget and the
//!   decoded frames match the planned geometry and count;
//! * the plan is a property of the stream, not of the policy;
//! * the loss-tolerant depth decoders never succeed where the planner
//!   refused on a limit.
//!
//! Run with `-malloc_limit_mb=32`: with the pixel caps held at 1 MP /
//! segment and 4 MP total (the `decode_segment` budget, keeping one
//! iteration in the millisecond range) the plan stays under 32 MiB, so
//! a single allocation above that is a finding in its own right (the
//! planner missed a buffer). Keep libFuzzer's default `-rss_limit_mb`
//! (2048): the sanitizer build's shadow memory and quarantine put a
//! garbage-decode iteration at many times its live heap.

use libfuzzer_sys::fuzz_target;
use oxideav_icer::{
    decode_all_with, info_with, parse_icer3d_lenient_with, parse_icer_lenient_with, DecodeOptions,
    IcerError,
};

const BUDGETS: [Option<u64>; 6] = [
    Some(64 << 10),
    Some(1 << 20),
    Some(4 << 20),
    Some(16 << 20),
    Some(DecodeOptions::DEFAULT_MAX_BYTES),
    None,
];

fn is_limit<T>(r: &Result<T, IcerError>) -> bool {
    matches!(r, Err(IcerError::LimitExceeded(_)))
}

fuzz_target!(|data: &[u8]| {
    let Some((&sel, stream)) = data.split_first() else {
        return;
    };
    // Pixel caps keep one iteration's CPU bounded whatever the budget
    // (the default 1 GiB / unlimited budgets would otherwise admit a
    // 64 MP inverse DWT from a 12-byte header; a 2.7 M-sample lenient
    // cube decode of garbage already took 1.5 s — 33 s under the
    // sanitizer — at a 16 MP cap).
    let opts = DecodeOptions::new()
        .with_max_bytes(BUDGETS[sel as usize % BUDGETS.len()])
        .with_max_pixels_per_segment(1u64 << 20)
        .with_max_pixels(1u64 << 22);

    let planned = info_with(stream, &opts);
    let decoded = decode_all_with(stream, &opts);
    match (&planned, &decoded) {
        (Ok(info), Ok(frames)) => {
            assert_eq!(info.frames as usize, frames.len(), "frame count vs plan");
            for f in frames {
                assert_eq!((f.image.width, f.image.height), (info.width, info.height));
                assert_eq!(f.image.format, info.format);
            }
            if let Some(budget) = opts.max_bytes {
                assert!(
                    info.working_set_bytes <= budget,
                    "plan {} accepted over budget {budget}",
                    info.working_set_bytes
                );
            }
            // The plan does not depend on the policy it was checked under.
            let free = info_with(stream, &DecodeOptions::new().unlimited())
                .expect("unlimited policy accepts what a bounded one did");
            assert_eq!(free.working_set_bytes, info.working_set_bytes);
        }
        (Err(e), Ok(_)) if e.is_limit_exceeded() => {
            panic!("decode_all accepted a stream the planner refused: {e}")
        }
        (Ok(_), Err(e)) if e.is_limit_exceeded() => {
            panic!("decoder refused on a limit the planner accepted: {e}")
        }
        _ => {}
    }

    // A planner refusal on a limit binds the lenient depth decoders too:
    // their reconstruction geometry is never smaller than the strict
    // decode's, so they must not succeed.
    if is_limit(&planned) {
        assert!(
            parse_icer_lenient_with(stream, &opts).is_err(),
            "lenient 2-D decode bypassed a budget refusal"
        );
        assert!(
            parse_icer3d_lenient_with(stream, &opts).is_err(),
            "lenient cube decode bypassed a budget refusal"
        );
    } else {
        let _ = parse_icer_lenient_with(stream, &opts);
        let _ = parse_icer3d_lenient_with(stream, &opts);
    }
});
