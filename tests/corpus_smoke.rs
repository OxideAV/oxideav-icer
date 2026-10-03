//! Per-push smoke over the checked-in fuzz corpus.
//!
//! The scheduled fuzz workflow runs daily; this test gives every
//! checked-in `decode_segment` corpus entry (including the crash
//! regressions) per-push coverage through the exact entry-point stack
//! the fuzz target drives, so a decoder regression on a known-bad
//! input fails CI immediately instead of waiting for the cron.

use oxideav_icer::{
    decode_all_with, decode_with, info, parse_icer3d_with, parse_icer_lenient_with, probe,
    walk_segment, DecodeOptions,
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
        let plausible = probe(&data);
        let _ = walk_segment(&data);
        if let Ok(i) = info(&data) {
            assert!(
                plausible,
                "{}: info accepted what probe rejected",
                path.display()
            );
            assert!(i.frames >= 1);
        }
        if let Ok(img) = decode_with(&data, &fuzz_limits()) {
            // The contract conversions are infallible on decoder output
            // and `decode_all` agrees with `decode` on the first image.
            assert_eq!(
                img.to_rgb8().len(),
                img.width as usize * img.height as usize * 3
            );
            assert_eq!(
                img.to_rgba8().len(),
                img.width as usize * img.height as usize * 4
            );
            let all = decode_all_with(&data, &fuzz_limits()).expect("decode_all follows decode");
            assert_eq!(all[0].image, img, "{}", path.display());
        }
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
