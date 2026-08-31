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
//!   reversible filter (`WaveletFilter::FilterQ`), the bit-exact
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
    encode_icer, encode_icer3d, parse_icer, parse_icer3d, CubeEncodeOptions, EncodeOptions,
    IcerCube, IcerImage, IcerPixelFormat, WaveletFilter,
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
        filter: WaveletFilter::FilterQ,
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

/// Filter-A encode-options. Filter A is one of the seven IPN 42-155
/// §II.A reversible integer transforms; it shares filter Q's `alpha`
/// triple and differs only in `beta` (Table 1), so the Q-vs-A delta
/// isolates the cost of the `beta * d[n+1]` predictor term in the
/// shared eq (3) recurrence.
fn compressed_filter_a_opts() -> EncodeOptions {
    EncodeOptions {
        filter: WaveletFilter::FilterA,
        wavelet_levels: 2,
        bit_plane_count: 8,
        uncompressed: false,
        ..EncodeOptions::default()
    }
}

fn bench_filter_a_path(c: &mut Criterion) {
    // Mirrors the filter-Q encode + decode groups but on filter A.
    // Same three input shapes so the criterion report makes the
    // Q-vs-A delta directly readable.
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
/// (`FilterQ`) path. The filter-Q encode/decode groups above pin
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
        filter: WaveletFilter::FilterQ,
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

/// Round 225 -- segment-count sweep on the integer 5/3
/// (`FilterQ`) path. The filter-Q encode/decode groups above pin
/// `segment_count = 1` (the `EncodeOptions::default` value); this group
/// sweeps `segment_count` over `[1, 2, 4, 8]` on the 64x64 ramp so the
/// per-strip overhead of the IPN 42-155 §III.E independent-segment
/// partitioning is visible per-count rather than hidden by the
/// single-segment default. Each segment carries its own 12-byte segment
/// header + an independent arithmetic-coder context model + its own
/// stripe-ordered bit-plane scan, so the per-segment fixed cost is
/// expected to dominate the throughput trend as `segment_count` rises
/// against a fixed pixel budget.
///
/// Encode + decode are reported as separate benches against the same
/// per-count input so the per-stage cost is directly readable.
///
/// `segment_count` is clamped to `1..=u16::MAX` by the encoder, but the
/// usable upper bound on a 64x64 input is constrained by the encoder's
/// "minimum 2 rows per strip" check (see `encoder.rs`). The chosen
/// sweep `[1, 2, 4, 8]` keeps every strip at >= 8 rows (64 / 8 = 8),
/// well above the floor.
fn compressed_filter_q_opts_segments(segments: u16) -> EncodeOptions {
    EncodeOptions {
        filter: WaveletFilter::FilterQ,
        wavelet_levels: 2,
        bit_plane_count: 8,
        uncompressed: false,
        segment_count: segments,
        ..EncodeOptions::default()
    }
}

fn bench_segment_count_sweep(c: &mut Criterion) {
    const SEGMENTS: [u16; 4] = [1, 2, 4, 8];
    let ramp = ramp_image(64, 64);
    let pixel_bytes = (ramp.width as u64) * (ramp.height as u64);

    let mut enc_group = c.benchmark_group("encode_compressed_filter_q_segments_64x64");
    enc_group.throughput(Throughput::Bytes(pixel_bytes));
    for &n in &SEGMENTS {
        let opts = compressed_filter_q_opts_segments(n);
        enc_group.bench_function(format!("segments_{}", n), |b| {
            b.iter(|| {
                let bytes = encode_icer(black_box(&ramp), black_box(&opts)).unwrap();
                black_box(bytes);
            });
        });
    }
    enc_group.finish();

    let mut dec_group = c.benchmark_group("decode_compressed_filter_q_segments_64x64");
    dec_group.throughput(Throughput::Bytes(pixel_bytes));
    for &n in &SEGMENTS {
        let opts = compressed_filter_q_opts_segments(n);
        let bytes = encode_icer(&ramp, &opts).unwrap();
        dec_group.bench_function(format!("segments_{}", n), |b| {
            b.iter(|| {
                let img = parse_icer(black_box(&bytes)).unwrap();
                black_box(img);
            });
        });
    }
    dec_group.finish();
}

/// Round 230 -- bit-plane-count sweep on the integer 5/3
/// (`FilterQ`) path. The filter-Q encode/decode groups above pin
/// `bit_plane_count = 8` (the round-181 baseline); this group sweeps
/// `bit_plane_count` over `[4, 8, 12, 16]` on the 64x64 ramp so the
/// per-packet overhead of the IPN 42-155 §IV multi-packet ordering is
/// visible per-count rather than hidden by the default-floor pin.
/// `bit_plane_count` acts as a floor on the per-segment packet-count `q`
/// (the encoder takes `q = max(needed_for_largest_coeff,
/// caller_floor).min(31)`), so raising it above the natural `needed`
/// forces the encoder to emit additional bit-plane pairs that mostly
/// carry zero significance bits + zero refinement bits -- the cleanest
/// way to expose the per-packet fixed overhead of the arithmetic-coder
/// init / flush / framing in isolation from coefficient-magnitude noise.
///
/// Encode + decode are reported as separate benches against the same
/// per-count input so the per-stage cost is directly readable.
///
/// `bit_plane_count` is clamped to `1..=31` by the encoder (see
/// `encoder.rs`); the chosen sweep `[4, 8, 12, 16]` brackets the
/// round-181 default (`8`) symmetrically on the low side (entropy stage
/// dominated by the largest-coefficient `needed` floor; the caller
/// floor is overridden) and walks up to `16` on the high side (well
/// above what any natural 8-bit Gray8 input ever reaches; every added
/// plane is pure per-packet overhead). The 64x64 ramp's `needed` is
/// `select_bit_plane_count` of the DWT coefficients (filter Q on this
/// shape lands around 8 bit-planes), so `4` and `8` collapse to the
/// same effective `q` while `12` and `16` walk above it; the floor /
/// no-floor split is exactly the interesting dynamic.
fn compressed_filter_q_opts_bit_planes(bit_planes: u8) -> EncodeOptions {
    EncodeOptions {
        filter: WaveletFilter::FilterQ,
        wavelet_levels: 2,
        bit_plane_count: bit_planes,
        uncompressed: false,
        ..EncodeOptions::default()
    }
}

fn bench_bit_plane_count_sweep(c: &mut Criterion) {
    const BIT_PLANES: [u8; 4] = [4, 8, 12, 16];
    let ramp = ramp_image(64, 64);
    let pixel_bytes = (ramp.width as u64) * (ramp.height as u64);

    let mut enc_group = c.benchmark_group("encode_compressed_filter_q_bit_planes_64x64");
    enc_group.throughput(Throughput::Bytes(pixel_bytes));
    for &q in &BIT_PLANES {
        let opts = compressed_filter_q_opts_bit_planes(q);
        enc_group.bench_function(format!("q_{}", q), |b| {
            b.iter(|| {
                let bytes = encode_icer(black_box(&ramp), black_box(&opts)).unwrap();
                black_box(bytes);
            });
        });
    }
    enc_group.finish();

    let mut dec_group = c.benchmark_group("decode_compressed_filter_q_bit_planes_64x64");
    dec_group.throughput(Throughput::Bytes(pixel_bytes));
    for &q in &BIT_PLANES {
        let opts = compressed_filter_q_opts_bit_planes(q);
        let bytes = encode_icer(&ramp, &opts).unwrap();
        dec_group.bench_function(format!("q_{}", q), |b| {
            b.iter(|| {
                let img = parse_icer(black_box(&bytes)).unwrap();
                black_box(img);
            });
        });
    }
    dec_group.finish();
}

/// Round 262 -- wavelet-decomposition-depth sweep on the filter-A
/// path. Round 210 added the same shape of sweep over `wavelet_levels`
/// for filter Q; this group is the filter-A counterpart so the
/// per-depth cost is directly readable rather than averaged into the
/// round-205 default-depth (`wavelet_levels = 2`) filter-A number.
///
/// Since r411 both filters run the same §II.A integer recurrence and
/// differ only in the Table 1 parameters (filter A's `beta = 0` even
/// skips the `d[n+1]` predictor term), so the per-depth slope
/// difference between this group and the round-210
/// `encode_compressed_filter_q_levels_64x64` group is expected to be
/// small; the sweep is retained to catch a divergence.
///
/// Encode + decode are reported as separate benches against the same
/// per-depth input so the forward + inverse DWT cost split is directly
/// readable. The same `[1, 2, 3, 4]` depth set as round 210 is used so
/// the report rows line up cell-by-cell with the integer 5/3 sweep
/// when read side-by-side.
///
/// `wavelet_levels` is clamped to `1..=6` by the encoder; depth 4 is
/// the deepest sensible value for a 64x64 input (subband LL at depth
/// 4 is 4x4 = 16 coefficients) -- matches the round-210 ceiling.
fn compressed_filter_a_opts_levels(levels: u8) -> EncodeOptions {
    EncodeOptions {
        filter: WaveletFilter::FilterA,
        wavelet_levels: levels,
        bit_plane_count: 8,
        uncompressed: false,
        ..EncodeOptions::default()
    }
}

fn bench_filter_a_wavelet_levels_sweep(c: &mut Criterion) {
    const DEPTHS: [u8; 4] = [1, 2, 3, 4];
    let ramp = ramp_image(64, 64);
    let pixel_bytes = (ramp.width as u64) * (ramp.height as u64);

    let mut enc_group = c.benchmark_group("encode_compressed_filter_a_levels_64x64");
    enc_group.throughput(Throughput::Bytes(pixel_bytes));
    for &d in &DEPTHS {
        let opts = compressed_filter_a_opts_levels(d);
        enc_group.bench_function(format!("levels_{}", d), |b| {
            b.iter(|| {
                let bytes = encode_icer(black_box(&ramp), black_box(&opts)).unwrap();
                black_box(bytes);
            });
        });
    }
    enc_group.finish();

    let mut dec_group = c.benchmark_group("decode_compressed_filter_a_levels_64x64");
    dec_group.throughput(Throughput::Bytes(pixel_bytes));
    for &d in &DEPTHS {
        let opts = compressed_filter_a_opts_levels(d);
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

/// ICER-3D cube baseline (IPN 42-164): lossless filter-Q encode +
/// decode of a 32x32x16 correlated-band 12-bit cube (the shape of the
/// integration suite's headline comparison). Gives the 3-D DWT +
/// spectral-context bit-plane coder + priority-packet framing a stable
/// perf reference before any vectorisation work.
fn bench_cube3d_path(c: &mut Criterion) {
    let (w, h, bands) = (32u32, 32u32, 16u32);
    let mut cube = IcerCube::zeros(w, h, bands, 12);
    let (wu, hu) = (w as usize, h as usize);
    for b in 0..bands as usize {
        let dc = 800 + ((b * 137) % 1200) as i32;
        for y in 0..hu {
            for x in 0..wu {
                let t = ((x * 13 + y * 29 + b * 7) % 257) as i32 - 128;
                cube.samples[b * wu * hu + y * wu + x] = (dc + t).clamp(0, 4095) as u16;
            }
        }
    }
    let sample_bytes = (cube.samples.len() * 2) as u64;
    let opts = CubeEncodeOptions::default();

    let mut group = c.benchmark_group("cube3d_filter_q_32x32x16");
    group.throughput(Throughput::Bytes(sample_bytes));
    group.bench_function("encode", |b| {
        b.iter(|| {
            let bytes = encode_icer3d(black_box(&cube), black_box(&opts)).unwrap();
            black_box(bytes);
        });
    });
    let bytes = encode_icer3d(&cube, &opts).unwrap();
    group.bench_function("decode", |b| {
        b.iter(|| {
            let out = parse_icer3d(black_box(&bytes)).unwrap();
            black_box(out);
        });
    });
    group.finish();

    // §V.D transform-domain segmentation at the IPN 42-164 §V.B
    // operating point (four error-containment segments) vs the same
    // cube single-segment — the r414 mode's perf reference. Wire pin:
    // the transform-domain stream must stay lossless.
    let td = CubeEncodeOptions::default()
        .with_transform_domain_segments()
        .with_segment_count(4);
    let td_bytes = encode_icer3d(&cube, &td).unwrap();
    assert_eq!(
        parse_icer3d(&td_bytes).unwrap(),
        cube,
        "transform-domain cube decode must stay lossless"
    );
    let mut group = c.benchmark_group("cube3d_td_filter_q_32x32x16_s4");
    group.throughput(Throughput::Bytes(sample_bytes));
    group.bench_function("encode", |b| {
        b.iter(|| black_box(encode_icer3d(black_box(&cube), &td).unwrap()));
    });
    group.bench_function("decode", |b| {
        b.iter(|| black_box(parse_icer3d(black_box(&td_bytes)).unwrap()));
    });
    group.finish();
}

/// §V.B transform-domain segmentation vs the row-strip convention at
/// the same segment count, plus the §VI.A minimum-loss sweep — the
/// round-389 additions. Byte-identity of the M = 0 / row-strip wire
/// form is asserted inside the benchmark setup so a perf tweak that
/// silently changes the output fails the bench run.
fn bench_transform_segments_and_min_loss(c: &mut Criterion) {
    let img = ramp_image(64, 64);
    let plane_bytes = (64 * 64) as u64;

    let mut strip = compressed_filter_q_opts();
    strip.segment_count = 4;
    let mut tf = compressed_filter_q_opts().with_transform_domain_segments();
    tf.segment_count = 4;

    // Wire-form pins: M = 0 is byte-identical to the plain compressed
    // encode; the transform-domain stream decodes bit-exactly.
    let strip_bytes = encode_icer(&img, &strip).unwrap();
    let strip_m0 = encode_icer(&img, &strip.clone().with_min_loss(0)).unwrap();
    assert_eq!(strip_bytes, strip_m0, "M = 0 must not change the wire form");
    let tf_bytes = encode_icer(&img, &tf).unwrap();
    assert_eq!(
        parse_icer(&tf_bytes).unwrap().planes[0].data,
        img.planes[0].data,
        "transform-domain filter-Q decode must stay bit-exact"
    );

    let mut group = c.benchmark_group("transform_segments_64x64_s4");
    group.throughput(Throughput::Bytes(plane_bytes));
    group.bench_function("encode_row_strip", |b| {
        b.iter(|| black_box(encode_icer(black_box(&img), &strip).unwrap()));
    });
    group.bench_function("encode_transform", |b| {
        b.iter(|| black_box(encode_icer(black_box(&img), &tf).unwrap()));
    });
    group.bench_function("decode_row_strip", |b| {
        b.iter(|| black_box(parse_icer(black_box(&strip_bytes)).unwrap()));
    });
    group.bench_function("decode_transform", |b| {
        b.iter(|| black_box(parse_icer(black_box(&tf_bytes)).unwrap()));
    });
    group.finish();

    let mut group = c.benchmark_group("min_loss_64x64");
    group.throughput(Throughput::Bytes(plane_bytes));
    for m in [0u8, 2, 4, 8] {
        let opts = compressed_filter_q_opts().with_min_loss(m);
        let encoded = encode_icer(&img, &opts).unwrap();
        group.bench_function(format!("encode_m{m}"), |b| {
            b.iter(|| black_box(encode_icer(black_box(&img), &opts).unwrap()));
        });
        group.bench_function(format!("decode_m{m}"), |b| {
            b.iter(|| black_box(parse_icer(black_box(&encoded)).unwrap()));
        });
    }
    group.finish();
}

/// Deep-sample (IPN 42-155 §II.C 12-bit MER operating point) encode +
/// decode baseline: a full-range 12-bit textured 64x64 through the
/// filter-Q compressed path (lossless decode asserted in setup).
/// Throughput counts the raw sample bytes (2 per pixel) so the deep
/// numbers are comparable to the 8-bit groups per transmitted byte.
fn bench_deep_sample_path(c: &mut Criterion) {
    let mut img = IcerImage::zeros(64, 64, oxideav_icer::IcerPixelFormat::GrayDeep { bits: 12 });
    for y in 0..64u32 {
        for x in 0..64u32 {
            img.set_sample(0, x, y, ((x * 97 + y * 57 + (x ^ y) * 31) % 4096) as u16);
        }
    }
    let opts = compressed_filter_q_opts();
    let bytes = encode_icer(&img, &opts).unwrap();
    assert_eq!(
        parse_icer(&bytes).unwrap().planes,
        img.planes,
        "deep bench fixture must decode lossless"
    );

    let mut group = c.benchmark_group("deep12_filter_q_64x64");
    group.throughput(Throughput::Bytes(
        (img.width as u64) * (img.height as u64) * 2,
    ));
    group.bench_function("encode", |b| {
        b.iter(|| {
            let out = encode_icer(black_box(&img), black_box(&opts)).unwrap();
            black_box(out);
        });
    });
    group.bench_function("decode", |b| {
        b.iter(|| {
            let out = parse_icer(black_box(&bytes)).unwrap();
            black_box(out);
        });
    });
    group.finish();
}

/// Representative-size throughput reference (r454): the 2-D
/// compressed path at 256x256 and the 3-D cube path at 64x64x32 —
/// the sizes the README throughput table reports. Textured fixtures
/// keep the context modeler + entropy coder realistically busy;
/// lossless decode is asserted in setup so a perf tweak that changes
/// the wire form fails the bench run.
fn bench_representative_sizes(c: &mut Criterion) {
    let mut img = IcerImage::zeros(256, 256, oxideav_icer::IcerPixelFormat::Gray8);
    {
        let stride = img.planes[0].stride;
        for y in 0..256usize {
            for x in 0..256usize {
                img.planes[0].data[y * stride + x] =
                    ((x * 97 + y * 57 + (x ^ y) * 31) & 0xFF) as u8;
            }
        }
    }
    // A textured 256x256 frame overflows the wire's u16 segment-body
    // limit as a single segment — exactly the multi-segment operating
    // point §V.C prescribes at this size (Typical picks 4-6).
    let mut opts = compressed_filter_q_opts();
    opts.segment_count = 4;
    let bytes = encode_icer(&img, &opts).unwrap();
    assert_eq!(
        parse_icer(&bytes).unwrap().planes[0].data,
        img.planes[0].data,
        "representative 2-D fixture must decode lossless"
    );

    let mut group = c.benchmark_group("representative_gray8_256x256_s4");
    group.sample_size(20);
    group.throughput(Throughput::Bytes(256 * 256));
    group.bench_function("encode", |b| {
        b.iter(|| black_box(encode_icer(black_box(&img), &opts).unwrap()));
    });
    group.bench_function("decode", |b| {
        b.iter(|| black_box(parse_icer(black_box(&bytes)).unwrap()));
    });
    group.finish();

    let (w, h, bands) = (64u32, 64u32, 32u32);
    let mut cube = IcerCube::zeros(w, h, bands, 12);
    let (wu, hu) = (w as usize, h as usize);
    for b in 0..bands as usize {
        let dc = 800 + ((b * 137) % 1200) as i32;
        for y in 0..hu {
            for x in 0..wu {
                let t = ((x * 13 + y * 29 + b * 7) % 257) as i32 - 128;
                cube.samples[b * wu * hu + y * wu + x] = (dc + t).clamp(0, 4095) as u16;
            }
        }
    }
    let cube_opts = CubeEncodeOptions::default();
    let cube_bytes = encode_icer3d(&cube, &cube_opts).unwrap();
    assert_eq!(
        parse_icer3d(&cube_bytes).unwrap(),
        cube,
        "representative cube fixture must decode lossless"
    );

    let mut group = c.benchmark_group("representative_cube3d_64x64x32");
    group.sample_size(10);
    group.throughput(Throughput::Bytes((cube.samples.len() * 2) as u64));
    group.bench_function("encode", |b| {
        b.iter(|| black_box(encode_icer3d(black_box(&cube), &cube_opts).unwrap()));
    });
    group.bench_function("decode", |b| {
        b.iter(|| black_box(parse_icer3d(black_box(&cube_bytes)).unwrap()));
    });
    group.finish();
}

criterion_group!(
    benches,
    bench_encode_compressed,
    bench_decode_compressed,
    bench_uncompressed_path,
    bench_filter_a_path,
    bench_wavelet_levels_sweep,
    bench_segment_count_sweep,
    bench_bit_plane_count_sweep,
    bench_filter_a_wavelet_levels_sweep,
    bench_cube3d_path,
    bench_transform_segments_and_min_loss,
    bench_deep_sample_path,
    bench_representative_sizes,
);
criterion_main!(benches);
