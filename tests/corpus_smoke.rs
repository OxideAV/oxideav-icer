//! Per-push smoke over the checked-in fuzz corpus.
//!
//! The scheduled fuzz workflow runs daily; this test gives every
//! checked-in `decode_segment` corpus entry (including the crash
//! regressions) per-push coverage through the exact entry-point stack
//! the fuzz target drives, so a decoder regression on a known-bad
//! input fails CI immediately instead of waiting for the cron.

use oxideav_icer::{
    decode_with, info, parse_icer3d_with, parse_icer_lenient_with, walk_segment, DecodeOptions,
};

/// Same tight per-iteration geometry budget the fuzz target uses.
fn fuzz_limits() -> DecodeOptions {
    DecodeOptions::new()
        .with_max_pixels_per_segment(1u64 << 20)
        .with_max_pixels(1u64 << 22)
}

#[test]
fn decode_segment_corpus_is_panic_free() {
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("fuzz/corpus/decode_segment");
    let mut driven = 0usize;
    for entry in std::fs::read_dir(&dir).expect("corpus dir") {
        let path = entry.expect("dir entry").path();
        if !path.is_file() {
            continue;
        }
        let data = std::fs::read(&path).expect("read corpus entry");
        let _ = walk_segment(&data);
        let _ = info(&data);
        let _ = decode_with(&data, &fuzz_limits());
        let _ = parse_icer_lenient_with(&data, &fuzz_limits());
        let _ = parse_icer3d_with(&data, &fuzz_limits());
        let _ = oxideav_icer::parse_icer3d_lenient_with(&data, &fuzz_limits());
        driven += 1;
    }
    // The corpus ships with the crate; a checkout that lost it should
    // fail loudly rather than vacuously pass.
    assert!(
        driven >= 10,
        "only {driven} corpus entries found in {dir:?}"
    );
}
