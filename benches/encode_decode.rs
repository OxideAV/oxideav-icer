//! Round 181 -- criterion benchmark covering the encode + decode hot
//! paths so future rounds (entropy-coder tweaks, wavelet vectorisation,
//! filter-selection heuristics, ...) have a stable baseline to compare
//! against.
//!
//! Three input shapes are exercised, each on a small synthetic image
//! whose construction is identical to the round-trip tests so that
//! benchmark deltas are interpreted relative to behaviour the
//! integration tests already cover:
//!
//! * `ramp_16x16` -- diagonal-gradient input over the integer 5/3
//!   reversible filter (`WaveletFilter::Reversible53`), the bit-exact
//!   round-trip path. Stresses the bit-plane scanner + arithmetic
//!   coder on a non-trivial coefficient distribution.
//! * `smooth_16x16` -- flat mid-grey input on the same filter. Almost
//!   every detail-band coefficient is zero, so the entropy stage
//!   collapses to its smallest-output regime.
//! * `ramp_64x64` -- a larger ramp so we get a load-bearing 4 KiB
//!   plane and can read throughput in MiB/s rather than per-frame
//!   nanoseconds.
//!
//! Each input is benchmarked end-to-end on `encode_icer` and
//! `parse_icer`. The uncompressed path (`EncodeOptions::default()`)
//! is exercised separately on the larger ramp to give us a contrast
//! against the compressed path on the same image -- the encoder and
//! the decoder's framing walk should both be near-`memcpy` speed when
//! the wavelet + entropy stages are bypassed.

use criterion::{black_box, criterion_group, criterion_main, Criterion, Throughput};
use oxideav_icer::{
    encode_icer, parse_icer, EncodeOptions, IcerImage, IcerPixelFormat, WaveletFilter,
};

/// Diagonal ramp identical to the round-trip tests' `ramp_image`. Keeps
/// benchmark inputs comparable to the (already-asserted-correct)
/// integration tests.
fn ramp_image(w: u32, h: u32) -> IcerImage {
    let mut img = IcerImage::zeros(w, h, IcerPixelFormat::Gray8);
    let plane = &mut img.planes[0];
    for y in 0..h as usize {
        for x in 0..w as usize {
            plane.data[y * plane.stride + x] = ((x + y) & 0xFF) as u8;
        }
    }
    img
}

/// Constant mid-grey image -- best-case entropy coder input (every
/// detail-band coefficient collapses to zero).
fn smooth_image(w: u32, h: u32) -> IcerImage {
    let mut img = IcerImage::zeros(w, h, IcerPixelFormat::Gray8);
    for byte in img.planes[0].data.iter_mut() {
        *byte = 128;
    }
    img
}

fn compressed_filter_q_opts() -> EncodeOptions {
    EncodeOptions {
        filter: WaveletFilter::Reversible53,
        wavelet_levels: 2,
        bit_plane_count: 8,
        uncompressed: false,
        ..EncodeOptions::default()
    }
}

fn bench_encode_compressed(c: &mut Criterion) {
    let mut group = c.benchmark_group("encode_compressed_filter_q");
    let opts = compressed_filter_q_opts();

    // Diagonal ramp 16x16: non-trivial coefficient distribution.
    let ramp_small = ramp_image(16, 16);
    group.throughput(Throughput::Bytes(
        (ramp_small.width as u64) * (ramp_small.height as u64),
    ));
    group.bench_function("ramp_16x16", |b| {
        b.iter(|| {
            let bytes = encode_icer(black_box(&ramp_small), black_box(&opts)).unwrap();
            black_box(bytes);
        });
    });

    // Mid-grey 16x16: best-case entropy.
    let smooth = smooth_image(16, 16);
    group.throughput(Throughput::Bytes(
        (smooth.width as u64) * (smooth.height as u64),
    ));
    group.bench_function("smooth_16x16", |b| {
        b.iter(|| {
            let bytes = encode_icer(black_box(&smooth), black_box(&opts)).unwrap();
            black_box(bytes);
        });
    });

    // Larger ramp 64x64: 4 KiB plane, throughput-readable.
    let ramp_large = ramp_image(64, 64);
    group.throughput(Throughput::Bytes(
        (ramp_large.width as u64) * (ramp_large.height as u64),
    ));
    group.bench_function("ramp_64x64", |b| {
        b.iter(|| {
            let bytes = encode_icer(black_box(&ramp_large), black_box(&opts)).unwrap();
            black_box(bytes);
        });
    });

    group.finish();
}

fn bench_decode_compressed(c: &mut Criterion) {
    let mut group = c.benchmark_group("decode_compressed_filter_q");
    let opts = compressed_filter_q_opts();

    // Pre-encode each fixture once -- we are measuring the decode path,
    // not the encode path.
    let ramp_small = ramp_image(16, 16);
    let ramp_small_bytes = encode_icer(&ramp_small, &opts).unwrap();
    group.throughput(Throughput::Bytes(
        (ramp_small.width as u64) * (ramp_small.height as u64),
    ));
    group.bench_function("ramp_16x16", |b| {
        b.iter(|| {
            let img = parse_icer(black_box(&ramp_small_bytes)).unwrap();
            black_box(img);
        });
    });

    let smooth = smooth_image(16, 16);
    let smooth_bytes = encode_icer(&smooth, &opts).unwrap();
    group.throughput(Throughput::Bytes(
        (smooth.width as u64) * (smooth.height as u64),
    ));
    group.bench_function("smooth_16x16", |b| {
        b.iter(|| {
            let img = parse_icer(black_box(&smooth_bytes)).unwrap();
            black_box(img);
        });
    });

    let ramp_large = ramp_image(64, 64);
    let ramp_large_bytes = encode_icer(&ramp_large, &opts).unwrap();
    group.throughput(Throughput::Bytes(
        (ramp_large.width as u64) * (ramp_large.height as u64),
    ));
    group.bench_function("ramp_64x64", |b| {
        b.iter(|| {
            let img = parse_icer(black_box(&ramp_large_bytes)).unwrap();
            black_box(img);
        });
    });

    group.finish();
}

fn bench_uncompressed_path(c: &mut Criterion) {
    // Uncompressed segments (IPN 42-155 §III.D) -- the wavelet + entropy
    // pipeline is bypassed. Useful contrast against the compressed
    // numbers so we can tell whether a regression is in the framing or
    // the algorithmic stages.
    let mut group = c.benchmark_group("uncompressed_path_64x64");
    let opts = EncodeOptions::default();
    let ramp = ramp_image(64, 64);
    group.throughput(Throughput::Bytes(
        (ramp.width as u64) * (ramp.height as u64),
    ));
    group.bench_function("encode", |b| {
        b.iter(|| {
            let bytes = encode_icer(black_box(&ramp), black_box(&opts)).unwrap();
            black_box(bytes);
        });
    });

    let bytes = encode_icer(&ramp, &opts).unwrap();
    group.bench_function("decode", |b| {
        b.iter(|| {
            let img = parse_icer(black_box(&bytes)).unwrap();
            black_box(img);
        });
    });

    group.finish();
}

/// Float-filter (`A`) encode-options. Filter A is the lossy 9/7-style
/// CDF lifting filter named in IPN 42-155 §III.A; it is the Mars-rover
/// lossy default and is the natural perf contrast against the integer
/// Q (`Reversible53`) lossless path -- the wavelet stage now runs in
/// `f64` lifting + integer quantisation rather than pure `i32`, so the
/// numbers exposed here are the baseline for any future float-DWT
/// vectorisation work.
fn compressed_filter_a_opts() -> EncodeOptions {
    EncodeOptions {
        filter: WaveletFilter::NineSevenA,
        wavelet_levels: 2,
        bit_plane_count: 8,
        uncompressed: false,
        ..EncodeOptions::default()
    }
}

fn bench_filter_a_path(c: &mut Criterion) {
    // Mirrors the filter-Q encode + decode groups but on the lossy float
    // 9/7 CDF path. Same three input shapes so the criterion report
    // makes the Q-vs-A delta directly readable.
    let opts = compressed_filter_a_opts();

    let mut enc_group = c.benchmark_group("encode_compressed_filter_a");
    let ramp_small = ramp_image(16, 16);
    enc_group.throughput(Throughput::Bytes(
        (ramp_small.width as u64) * (ramp_small.height as u64),
    ));
    enc_group.bench_function("ramp_16x16", |b| {
        b.iter(|| {
            let bytes = encode_icer(black_box(&ramp_small), black_box(&opts)).unwrap();
            black_box(bytes);
        });
    });

    let smooth = smooth_image(16, 16);
    enc_group.throughput(Throughput::Bytes(
        (smooth.width as u64) * (smooth.height as u64),
    ));
    enc_group.bench_function("smooth_16x16", |b| {
        b.iter(|| {
            let bytes = encode_icer(black_box(&smooth), black_box(&opts)).unwrap();
            black_box(bytes);
        });
    });

    let ramp_large = ramp_image(64, 64);
    enc_group.throughput(Throughput::Bytes(
        (ramp_large.width as u64) * (ramp_large.height as u64),
    ));
    enc_group.bench_function("ramp_64x64", |b| {
        b.iter(|| {
            let bytes = encode_icer(black_box(&ramp_large), black_box(&opts)).unwrap();
            black_box(bytes);
        });
    });
    enc_group.finish();

    let mut dec_group = c.benchmark_group("decode_compressed_filter_a");
    let ramp_small_bytes = encode_icer(&ramp_small, &opts).unwrap();
    dec_group.throughput(Throughput::Bytes(
        (ramp_small.width as u64) * (ramp_small.height as u64),
    ));
    dec_group.bench_function("ramp_16x16", |b| {
        b.iter(|| {
            let img = parse_icer(black_box(&ramp_small_bytes)).unwrap();
            black_box(img);
        });
    });

    let smooth_bytes = encode_icer(&smooth, &opts).unwrap();
    dec_group.throughput(Throughput::Bytes(
        (smooth.width as u64) * (smooth.height as u64),
    ));
    dec_group.bench_function("smooth_16x16", |b| {
        b.iter(|| {
            let img = parse_icer(black_box(&smooth_bytes)).unwrap();
            black_box(img);
        });
    });

    let ramp_large_bytes = encode_icer(&ramp_large, &opts).unwrap();
    dec_group.throughput(Throughput::Bytes(
        (ramp_large.width as u64) * (ramp_large.height as u64),
    ));
    dec_group.bench_function("ramp_64x64", |b| {
        b.iter(|| {
            let img = parse_icer(black_box(&ramp_large_bytes)).unwrap();
            black_box(img);
        });
    });
    dec_group.finish();
}

/// Round 210 -- wavelet-decomposition-depth sweep on the integer 5/3
/// (`Reversible53`) path. The filter-Q encode/decode groups above pin
/// the default `wavelet_levels = 2` configuration; this group sweeps
/// `wavelet_levels` over `[1, 2, 3, 4]` on the 64x64 ramp so a
/// regression (or, eventually, a wavelet-vectorisation win) on the
/// dyadic 5/3 recursion is visible per-depth rather than averaged into
/// a single number. Encode + decode are reported as separate benches
/// against the same per-depth input so the cost split between forward
/// DWT + entropy and inverse DWT + entropy is directly readable.
///
/// `wavelet_levels` is clamped to `1..=6` by the encoder (see
/// `EncodeOptions::wavelet_levels` and the clamps in
/// `encoder.rs`); depth 4 is the deepest sensible value for a 64x64
/// input (subband LL at depth 4 is 4x4 = 16 coefficients, below which
/// further dyadic recursion no longer changes the bit-plane scanner's
/// stripe coverage).
fn compressed_filter_q_opts_levels(levels: u8) -> EncodeOptions {
    EncodeOptions {
        filter: WaveletFilter::Reversible53,
        wavelet_levels: levels,
        bit_plane_count: 8,
        uncompressed: false,
        ..EncodeOptions::default()
    }
}

fn bench_wavelet_levels_sweep(c: &mut Criterion) {
    const DEPTHS: [u8; 4] = [1, 2, 3, 4];
    let ramp = ramp_image(64, 64);
    let pixel_bytes = (ramp.width as u64) * (ramp.height as u64);

    let mut enc_group = c.benchmark_group("encode_compressed_filter_q_levels_64x64");
    enc_group.throughput(Throughput::Bytes(pixel_bytes));
    for &d in &DEPTHS {
        let opts = compressed_filter_q_opts_levels(d);
        enc_group.bench_function(format!("levels_{}", d), |b| {
            b.iter(|| {
                let bytes = encode_icer(black_box(&ramp), black_box(&opts)).unwrap();
                black_box(bytes);
            });
        });
    }
    enc_group.finish();

    let mut dec_group = c.benchmark_group("decode_compressed_filter_q_levels_64x64");
    dec_group.throughput(Throughput::Bytes(pixel_bytes));
    for &d in &DEPTHS {
        let opts = compressed_filter_q_opts_levels(d);
        let bytes = encode_icer(&ramp, &opts).unwrap();
        dec_group.bench_function(format!("levels_{}", d), |b| {
            b.iter(|| {
                let img = parse_icer(black_box(&bytes)).unwrap();
                black_box(img);
            });
        });
    }
    dec_group.finish();
}

criterion_group!(
    benches,
    bench_encode_compressed,
    bench_decode_compressed,
    bench_uncompressed_path,
    bench_filter_a_path,
    bench_wavelet_levels_sweep,
);
criterion_main!(benches);
