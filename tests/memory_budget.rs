//! The decoder's memory bound.
//!
//! `DecodeOptions::max_bytes` is checked against the decoder's *planned
//! peak working set* (output planes + the largest per-segment
//! coefficient buffer and coder state — `ImageInfo::working_set_bytes`),
//! derived from the framing before anything is allocated. This file
//! pins, with a counting global allocator:
//!
//! * the plan is an upper bound on the measured heap of `decode` /
//!   `decode_all` on every layout and coding mode;
//! * a hostile header (the format's maximum geometry, a 64 MP header
//!   with no body, a maximal cube header) is refused — or served from
//!   its output plane alone — without any large allocation and quickly;
//! * `max_bytes` refuses a stream whose *working set* exceeds it even
//!   when the decoded planes alone would fit;
//! * truncated segments and impossible band counts never panic.

use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicUsize, Ordering::SeqCst};
use std::sync::{Mutex, MutexGuard};
use std::time::Instant;

use oxideav_icer::{
    decode, decode_all, decode_all_with, decode_with, encode, encode_icer3d, info, info_with,
    parse_icer3d_lenient_with, parse_icer3d_with, parse_icer_lenient, parse_icer_lenient_with,
    CubeEncodeOptions, DecodeOptions, EncodeOptions, IcerCube, IcerError, IcerImage,
    IcerPixelFormat, SegmentHeader, WaveletFilter,
};

// ---- counting allocator ----------------------------------------------------

struct Counting;

static LIVE: AtomicUsize = AtomicUsize::new(0);
static PEAK: AtomicUsize = AtomicUsize::new(0);
static LARGEST: AtomicUsize = AtomicUsize::new(0);

fn track(size: usize) {
    let live = LIVE.fetch_add(size, SeqCst) + size;
    PEAK.fetch_max(live, SeqCst);
    LARGEST.fetch_max(size, SeqCst);
}

unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let p = System.alloc(layout);
        if !p.is_null() {
            track(layout.size());
        }
        p
    }
    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        let p = System.alloc_zeroed(layout);
        if !p.is_null() {
            track(layout.size());
        }
        p
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        System.dealloc(ptr, layout);
        LIVE.fetch_sub(layout.size(), SeqCst);
    }
    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        let p = System.realloc(ptr, layout, new_size);
        if !p.is_null() {
            LIVE.fetch_sub(layout.size(), SeqCst);
            track(new_size);
        }
        p
    }
}

#[global_allocator]
static GLOBAL: Counting = Counting;

/// The tests of this binary measure a process-wide counter, so they run
/// one at a time.
static SERIAL: Mutex<()> = Mutex::new(());

fn serial() -> MutexGuard<'static, ()> {
    SERIAL.lock().unwrap_or_else(|e| e.into_inner())
}

/// Run `f`; return its result, the peak heap growth above the level at
/// entry, and the largest single allocation made.
fn measure<T>(f: impl FnOnce() -> T) -> (T, usize, usize) {
    let base = LIVE.load(SeqCst);
    PEAK.store(base, SeqCst);
    LARGEST.store(0, SeqCst);
    let out = f();
    let peak = PEAK.load(SeqCst).saturating_sub(base);
    (out, peak, LARGEST.load(SeqCst))
}

// ---- fixtures ---------------------------------------------------------------

/// Smooth gradient plus low-amplitude deterministic texture:
/// compressible, but not trivially so.
fn textured(salt: u32, max: u32) -> impl Fn(u32, u32) -> u32 {
    move |x, y| {
        let g = (x * 3 + y * 5 + salt) % (max + 1);
        let t = ((x.wrapping_mul(2654435761) ^ y.wrapping_mul(40503) ^ salt) >> 13) & 15;
        (g / 2 + t + max / 4).min(max)
    }
}

fn gray(w: u32, h: u32, bits: u8, salt: u32) -> IcerImage {
    let max = (1u32 << bits) - 1;
    let f = textured(salt, max);
    if bits <= 8 {
        let mut img = IcerImage::zeros(w, h, IcerPixelFormat::Gray8);
        for y in 0..h {
            for x in 0..w {
                img.planes[0].data[(y * w + x) as usize] = f(x, y) as u8;
            }
        }
        img.with_bit_depth(bits).unwrap()
    } else {
        let mut img = IcerImage::zeros(w, h, IcerPixelFormat::Gray16Le);
        for y in 0..h {
            for x in 0..w {
                let i = ((y * w + x) * 2) as usize;
                img.planes[0].data[i..i + 2].copy_from_slice(&(f(x, y) as u16).to_le_bytes());
            }
        }
        img.with_bit_depth(bits).unwrap()
    }
}

fn colour(w: u32, h: u32, format: IcerPixelFormat) -> IcerImage {
    let mut img = IcerImage::zeros(w, h, format);
    for (p, plane) in img.planes.iter_mut().enumerate() {
        let f = textured(17 * p as u32 + 3, 255);
        for y in 0..h {
            for x in 0..w {
                plane.data[(y * w + x) as usize] = f(x, y) as u8;
            }
        }
    }
    img
}

fn cube(w: u32, h: u32, bands: u32, bits: u8) -> IcerCube {
    let mut c = IcerCube::zeros(w, h, bands, bits);
    let plane = (w * h) as usize;
    for b in 0..bands {
        let f = textured(b * 37 + b * b, (1u32 << bits) - 1);
        for y in 0..h {
            for x in 0..w {
                c.samples[b as usize * plane + (y * w + x) as usize] = f(x, y) as u16;
            }
        }
    }
    c
}

/// Every layout and coding mode the decoder has, as `(label, stream)`,
/// encoded once per process (three tests share them; the debug-build
/// encoder is slow).
fn fixtures() -> &'static [(&'static str, Vec<u8>)] {
    static FIXTURES: std::sync::OnceLock<Vec<(&'static str, Vec<u8>)>> = std::sync::OnceLock::new();
    FIXTURES.get_or_init(build_fixtures)
}

fn build_fixtures() -> Vec<(&'static str, Vec<u8>)> {
    let g = gray(64, 48, 8, 7);
    let c = EncodeOptions::compressed;
    let mut out = vec![
        (
            "gray8 uncompressed",
            encode(&g, &EncodeOptions::new()).unwrap(),
        ),
        ("gray8 1 segment", encode(&g, &c()).unwrap()),
        (
            "gray8 4 strips",
            encode(&g, &c().with_segment_count(4)).unwrap(),
        ),
        (
            "gray8 priority interleaving",
            encode(&g, &c().with_priority_interleaving()).unwrap(),
        ),
        (
            "gray8 interleaved entropy",
            encode(&g, &c().with_interleaved_entropy()).unwrap(),
        ),
        (
            "gray8 min_loss 3",
            encode(&g, &c().with_min_loss(3)).unwrap(),
        ),
        (
            "gray8 transform-domain 4",
            encode(
                &g,
                &c().with_segment_count(4).with_transform_domain_segments(),
            )
            .unwrap(),
        ),
        (
            "gray8 transform-domain + priority + min_loss",
            encode(
                &g,
                &c().with_segment_count(3)
                    .with_transform_domain_segments()
                    .with_priority_interleaving()
                    .with_min_loss(2),
            )
            .unwrap(),
        ),
        (
            "gray8 byte budget (truncated progressive)",
            encode(&g, &c().with_byte_budget(600)).unwrap(),
        ),
        ("gray12", encode(&gray(48, 40, 12, 9), &c()).unwrap()),
        ("gray16", encode(&gray(40, 36, 16, 11), &c()).unwrap()),
        (
            "yuv444p 2 strips",
            encode(
                &colour(48, 32, IcerPixelFormat::Yuv444P),
                &c().with_segment_count(2),
            )
            .unwrap(),
        ),
        (
            "gbrp8",
            encode(&colour(40, 32, IcerPixelFormat::Gbrp8), &c()).unwrap(),
        ),
    ];
    let cd = CubeEncodeOptions::default;
    out.push((
        "cube 8-bit 2 strips",
        encode_icer3d(&cube(24, 16, 6, 8), &cd().with_segment_count(2)).unwrap(),
    ));
    out.push((
        "cube 12-bit transform-domain 2",
        encode_icer3d(
            &cube(24, 16, 6, 12),
            &cd().with_segment_count(2).with_transform_domain_segments(),
        )
        .unwrap(),
    ));
    out.push((
        "cube 16-bit interleaved entropy min_loss",
        encode_icer3d(
            &cube(16, 12, 5, 16),
            &cd().with_interleaved_entropy().with_min_loss(2),
        )
        .unwrap(),
    ));
    out
}

/// A 12-byte compressed segment header with no body.
fn bare_header(width: u16, height: u16) -> Vec<u8> {
    SegmentHeader {
        sync_prefix: 0xACED,
        filter: WaveletFilter::FilterQ,
        decomp_levels: 3,
        uncompressed: false,
        width,
        height,
        bit_plane_count: 8,
        interleaved_entropy: false,
        transform_segmented: false,
        total_segments: 0,
        priority_interleaved: false,
        segment_length: 0,
        segment_index: 0,
    }
    .encode()
    .to_vec()
}

/// The 17-byte ICER-3D cube header (magic, w, h, bands, depth, filter,
/// levels, segments, strip height, flags) with no body.
fn cube_header(w: u16, h: u16, bands: u16, seg_count: u8, strip_h: u16, flags: u8) -> Vec<u8> {
    let mut v = vec![0x00, 0x00, 0xC3, 0x01];
    v.extend_from_slice(&w.to_be_bytes());
    v.extend_from_slice(&h.to_be_bytes());
    v.extend_from_slice(&bands.to_be_bytes());
    v.extend_from_slice(&[12, 0, 3, seg_count]);
    v.extend_from_slice(&strip_h.to_be_bytes());
    v.push(flags);
    v
}

fn is_limit(r: &Result<impl Sized, IcerError>) -> bool {
    matches!(r, Err(IcerError::LimitExceeded(_)))
}

// ---- tests ------------------------------------------------------------------

#[test]
fn planned_working_set_bounds_measured_heap_on_every_layout() {
    let _g = serial();
    for (label, bytes) in fixtures() {
        let plan = info(bytes).unwrap().working_set_bytes;
        let (all, peak_all, _) = measure(|| decode_all(bytes));
        let all = all.unwrap_or_else(|e| panic!("{label}: decode_all {e}"));
        assert!(
            peak_all as u64 <= plan,
            "{label}: decode_all peak heap {peak_all} exceeds the planned working set {plan}"
        );
        let (one, peak_one, _) = measure(|| decode(bytes));
        one.unwrap_or_else(|e| panic!("{label}: decode {e}"));
        assert!(
            peak_one as u64 <= plan,
            "{label}: decode peak heap {peak_one} exceeds the planned working set {plan}"
        );
        // The plan is also within an order of magnitude of what the
        // frames alone take once the fixed overheads are discounted —
        // it is a bound, not a blank cheque.
        let samples: u64 = all
            .iter()
            .map(|f| f.image.width as u64 * f.image.height as u64 * f.image.planes.len() as u64)
            .sum();
        assert!(
            plan <= 16 * samples + bytes.len() as u64 + (1 << 20),
            "{label}: plan {plan} is implausibly loose for {samples} samples"
        );
    }
}

#[test]
fn lenient_decodes_stay_within_the_plan() {
    let _g = serial();
    for (label, bytes) in fixtures() {
        let plan = info(bytes).unwrap().working_set_bytes;
        if oxideav_icer::is_cube(bytes) {
            let (r, peak, _) = measure(|| parse_icer3d_lenient_with(bytes, &DecodeOptions::new()));
            r.unwrap_or_else(|e| panic!("{label}: {e}"));
            // The public cube record stacks the band frames once more
            // (`IcerCube::samples`, two bytes per sample).
            let cube = parse_icer3d_with(bytes, &DecodeOptions::new()).unwrap();
            let stacked = cube.samples.len() as u64 * 2;
            assert!(
                peak as u64 <= plan + stacked,
                "{label}: lenient cube peak {peak} > plan {plan} + cube {stacked}"
            );
        } else {
            let (r, peak, _) = measure(|| parse_icer_lenient(bytes));
            r.unwrap_or_else(|e| panic!("{label}: {e}"));
            assert!(
                peak as u64 <= plan,
                "{label}: lenient peak {peak} > plan {plan}"
            );
        }
    }
}

#[test]
fn maximal_2d_header_is_refused_quickly_without_large_allocations() {
    let _g = serial();
    // The format's maximum: 65535 × 65535 ≈ 4.29 GP from a 12-byte
    // header. Every entry point refuses before allocating anything the
    // size of a plane; the whole check is header parsing.
    let bytes = bare_header(65535, 65535);
    let t = Instant::now();
    let (results, _, largest) = measure(|| {
        (
            decode(&bytes).map(|_| ()),
            info(&bytes).map(|_| ()),
            decode_all(&bytes).map(|_| ()),
            parse_icer_lenient(&bytes).map(|_| ()),
        )
    });
    assert!(is_limit(&results.0), "decode: {:?}", results.0);
    assert!(is_limit(&results.1), "info: {:?}", results.1);
    assert!(is_limit(&results.2), "decode_all: {:?}", results.2);
    assert!(is_limit(&results.3), "lenient: {:?}", results.3);
    assert!(largest < 64 * 1024, "largest allocation {largest} bytes");
    assert!(t.elapsed().as_millis() < 200, "took {:?}", t.elapsed());

    // With the pixel caps lifted, the working-set plan alone still
    // refuses it under a finite `max_bytes` — the output-only
    // accounting of earlier releases would have let a 4 GB plane
    // through a 16 GiB budget.
    let lifted = DecodeOptions::new()
        .with_max_pixels(None)
        .with_max_pixels_per_segment(None)
        .with_max_bytes(16u64 << 30);
    let (r, _, largest) = measure(|| decode_with(&bytes, &lifted));
    assert!(is_limit(&r), "{r:?}");
    assert!(largest < 64 * 1024);
    let planned = info_with(&bytes, &DecodeOptions::new().unlimited())
        .unwrap()
        .working_set_bytes;
    assert!(planned > 16u64 << 30, "plan {planned}");
}

#[test]
fn sixty_four_megapixel_header_costs_only_its_output_plane() {
    let _g = serial();
    // 8192 × 8192 is exactly the default per-segment pixel cap, so the
    // default policy admits it. With no packet body the segment is the
    // level-shift midpoint, served without a coefficient buffer or an
    // inverse DWT: the decode allocates the 64 MiB plane and nothing
    // of comparable size.
    let bytes = bare_header(8192, 8192);
    let plan = info(&bytes).unwrap().working_set_bytes;
    let t = Instant::now();
    let (img, peak, _) = measure(|| decode(&bytes));
    let img = img.unwrap();
    assert_eq!((img.width, img.height), (8192, 8192));
    assert!(img.planes[0].data.iter().all(|&b| b == 128));
    assert!(peak <= 64 * 1024 * 1024 + (1 << 20), "peak {peak}");
    assert!(peak as u64 <= plan);
    assert!(t.elapsed().as_secs() < 10, "took {:?}", t.elapsed());
    drop(img);

    // A tighter byte budget refuses it from the plan, before the plane.
    let (r, _, largest) =
        measure(|| decode_with(&bytes, &DecodeOptions::new().with_max_bytes(32u64 << 20)));
    assert!(is_limit(&r), "{r:?}");
    assert!(largest < 64 * 1024, "largest {largest}");
}

#[test]
fn maximal_cube_header_is_refused_quickly() {
    let _g = serial();
    for (label, bytes) in [
        (
            "65535³ strips",
            cube_header(65535, 65535, 65535, 255, 257, 0),
        ),
        (
            "65535³ transform-domain",
            cube_header(65535, 65535, 65535, 255, 0, 0b10),
        ),
        (
            "4 GP single band",
            cube_header(65535, 65535, 1, 1, 65535, 0),
        ),
    ] {
        let t = Instant::now();
        let (results, _, largest) = measure(|| {
            (
                decode_all(&bytes).map(|_| ()),
                info(&bytes).map(|_| ()),
                parse_icer3d_with(&bytes, &DecodeOptions::new()).map(|_| ()),
                parse_icer3d_lenient_with(&bytes, &DecodeOptions::new()).map(|_| ()),
            )
        });
        assert!(is_limit(&results.0), "{label} decode_all: {:?}", results.0);
        assert!(is_limit(&results.1), "{label} info: {:?}", results.1);
        assert!(is_limit(&results.2), "{label} cube: {:?}", results.2);
        assert!(is_limit(&results.3), "{label} lenient: {:?}", results.3);
        assert!(largest < 64 * 1024, "{label}: largest allocation {largest}");
        assert!(
            t.elapsed().as_millis() < 200,
            "{label}: took {:?}",
            t.elapsed()
        );
    }
}

#[test]
fn max_bytes_bounds_the_working_set_not_only_the_output() {
    let _g = serial();
    // A 256 × 256 transform-domain image: 64 KiB of output, but the
    // shared-transform decode holds the whole image's coefficients,
    // state and segment map at once — several times the output.
    let img = gray(256, 256, 8, 5);
    let bytes = encode(
        &img,
        &EncodeOptions::compressed()
            .with_segment_count(4)
            .with_transform_domain_segments(),
    )
    .unwrap();
    let plan = info(&bytes).unwrap().working_set_bytes;
    assert!(
        plan > 4 * 256 * 256,
        "plan {plan} does not count the working set"
    );
    let output_only = DecodeOptions::new().with_max_bytes(256u64 * 256 + (256 << 10));
    let r = decode_with(&bytes, &output_only);
    assert!(is_limit(&r), "output-sized budget must refuse: {r:?}");
    assert!(is_limit(&info_with(&bytes, &output_only)));
    assert!(is_limit(&parse_icer_lenient_with(&bytes, &output_only)));
    // Exactly the plan is enough; the decode is still lossless.
    let fits = DecodeOptions::new().with_max_bytes(plan);
    let back = decode_with(&bytes, &fits).unwrap();
    assert_eq!(back.planes, img.planes);
    let r = decode_with(&bytes, &DecodeOptions::new().with_max_bytes(plan - 1));
    assert!(is_limit(&r));

    // Same for a cube: the frames are small, the per-segment state is
    // not.
    let c = cube(64, 64, 16, 12);
    let bytes = encode_icer3d(
        &c,
        &CubeEncodeOptions::default()
            .with_segment_count(2)
            .with_transform_domain_segments(),
    )
    .unwrap();
    let frames_bytes = 64u64 * 64 * 16 * 2;
    let plan = info(&bytes).unwrap().working_set_bytes;
    assert!(plan > 2 * frames_bytes);
    let r = decode_all_with(
        &bytes,
        &DecodeOptions::new().with_max_bytes(frames_bytes + (256 << 10)),
    );
    assert!(is_limit(&r), "{:?}", r.map(|_| ()));
    let frames = decode_all_with(&bytes, &DecodeOptions::new().with_max_bytes(plan)).unwrap();
    assert_eq!(
        IcerCube::from_band_images(&frames.into_iter().map(|f| f.image).collect::<Vec<_>>())
            .unwrap(),
        c
    );
}

#[test]
fn truncated_segments_and_impossible_band_counts_never_panic() {
    let _g = serial();
    // A representative subset (every wire family); `tests/mutation_smoke.rs`
    // is the exhaustive corruption sweep.
    let keep = [
        "gray8 4 strips",
        "gray8 transform-domain + priority + min_loss",
        "yuv444p 2 strips",
        "cube 12-bit transform-domain 2",
    ];
    for (label, bytes) in fixtures().iter().filter(|(l, _)| keep.contains(l)) {
        let mut cut = 1;
        while cut < bytes.len() {
            let b = &bytes[..cut];
            let _ = decode(b);
            let _ = decode_all(b);
            let _ = info(b);
            let _ = parse_icer_lenient(b);
            let _ = parse_icer3d_lenient_with(b, &DecodeOptions::new());
            cut += 1 + cut / 2;
        }
        // Every byte of the header region flipped, one at a time.
        for i in 0..bytes.len().min(8) {
            let mut m = bytes.clone();
            m[i] ^= 0x5A;
            let _ = decode(&m);
            let _ = decode_all(&m);
            let _ = info(&m);
            let _ = parse_icer_lenient(&m);
            let _ = parse_icer3d_lenient_with(&m, &DecodeOptions::new());
        }
        let _ = label;
    }

    // Cube headers at the band-count extremes (the §V.D partition and
    // the spectral lattice must cope with 1 and 65535 bands, 0 is
    // invalid), with no body: clean errors, no panics.
    for (bands, seg, strip, flags) in [
        (0u16, 1u8, 1u16, 0u8),
        (1, 1, 1, 0),
        (65535, 1, 1, 0),
        (65535, 1, 0, 0b10),
        (65535, 255, 0, 0b10),
        (3, 0, 1, 0),
        (3, 2, 0, 0),
        (3, 1, 1, 0b100),
    ] {
        let bytes = cube_header(1, 1, bands, seg, strip, flags);
        for b in [&bytes[..], &bytes[..bytes.len() - 1], &bytes[..5]] {
            assert!(decode_all(b).is_err());
            assert!(parse_icer3d_lenient_with(b, &DecodeOptions::new()).is_err() || bands > 0);
            let _ = info(b);
        }
    }
}
