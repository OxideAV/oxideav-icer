#![no_main]

//! Encode-side fuzz harness for the ICER encoder + self-roundtrip.
//!
//! The decode-side harness [`decode_segment`] hammers `parse_icer` /
//! `parse_icer_metadata` / `walk_segment` with attacker-controlled
//! bytes off the wire. That covers everything a malicious downlink
//! peer might send, but it does **not** exercise the encoder.
//!
//! This harness fills the other half of the surface:
//!
//! 1. Synthesise a small `IcerImage` whose width, height and pixel
//!    bytes are derived from the fuzz input.
//! 2. Synthesise an `EncodeOptions` whose filter, level count, segment
//!    count, byte-budget, ROI permutation, R-D pruning flag, automatic
//!    uncompressed fallback flag and uncompressed-force flag are all
//!    derived from the fuzz input.
//! 3. Call [`oxideav_icer::encode_icer`] and assert it returns (never
//!    panics, never integer-overflows in debug, never tries to
//!    allocate based on caller-controlled width/height products that
//!    overflow `usize`).
//! 4. On `Ok`, feed the produced bytes through both
//!    [`oxideav_icer::parse_icer`] and
//!    [`oxideav_icer::parse_icer_lenient`] and confirm those also
//!    return rather than panicking. When the strict decoder succeeds,
//!    verify the geometry matches what was encoded.
//!
//! Geometry is intentionally capped (max 128 x 128, max 16 segments,
//! filter range 0..=7, wavelet levels 1..=4) to keep individual
//! iterations under a few milliseconds — the goal is to find encoder
//! input-validation gaps and self-roundtrip break, not to stress
//! large-image allocations (which the decode harness already covers
//! via the wire-claimed dimensions path).

use libfuzzer_sys::fuzz_target;
use oxideav_icer::{
    encode_icer, parse_icer, parse_icer_lenient, EncodeOptions, IcerImage, IcerPixelFormat,
    WaveletFilter,
};

/// Maximum image dimension the encode fuzzer will produce. Kept small so
/// each iteration is cheap; the wire-side decode allocation paths are
/// covered by the decode_segment harness.
const MAX_DIM: u32 = 128;

/// Maximum segment count the encode fuzzer will produce. Bounded so
/// `with_segment_priorities` permutations stay small.
const MAX_SEGMENTS: u16 = 16;

/// Maximum wavelet level count the fuzzer will request.
const MAX_LEVELS: u8 = 4;

/// Build a wavelet filter from a fuzz byte. The encoder accepts any of
/// the 8 enum values (`FilterQ` + `FilterA` + `FilterB..G`).
fn pick_filter(b: u8) -> WaveletFilter {
    match b & 0x07 {
        0 => WaveletFilter::FilterQ,
        1 => WaveletFilter::FilterA,
        2 => WaveletFilter::FilterB,
        3 => WaveletFilter::FilterC,
        4 => WaveletFilter::FilterD,
        5 => WaveletFilter::FilterE,
        _ => WaveletFilter::FilterF,
    }
}

/// Construct a permutation of `0..n` from `seed`. Always a valid
/// permutation (so the encoder accepts it as a priority vector) but
/// the order is fuzz-derived.
fn permutation(n: u16, seed: u64) -> Vec<u16> {
    let n = n as usize;
    let mut v: Vec<u16> = (0..n as u16).collect();
    // Fisher–Yates with a tiny xorshift PRNG seeded from `seed`.
    let mut state = seed.wrapping_mul(0x9E37_79B9_7F4A_7C15).wrapping_add(1);
    for i in (1..n).rev() {
        // xorshift64
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        let j = (state as usize) % (i + 1);
        v.swap(i, j);
    }
    v
}

fuzz_target!(|data: &[u8]| {
    // Need at least a header byte for geometry + a header byte for
    // options + 1 pixel of payload. Anything shorter is uninteresting.
    if data.len() < 8 {
        return;
    }

    // ---- Geometry ----------------------------------------------------
    // width = 1..=MAX_DIM, height = 1..=MAX_DIM.
    let width = (data[0] as u32 % MAX_DIM) + 1;
    let height = (data[1] as u32 % MAX_DIM) + 1;

    let pixel_count = (width as usize) * (height as usize);

    // ---- Pixels ------------------------------------------------------
    // Tile the remaining fuzz bytes across the pixel plane. With a
    // small input we get a strongly-correlated image; with a long
    // input we get noise.
    let pixel_src = &data[8..];
    let mut pixels = vec![0u8; pixel_count];
    if !pixel_src.is_empty() {
        for (i, p) in pixels.iter_mut().enumerate() {
            *p = pixel_src[i % pixel_src.len()];
        }
    }

    let mut img = IcerImage::zeros(width, height, IcerPixelFormat::Gray8);
    img.planes[0].data = pixels;

    // ---- Options -----------------------------------------------------
    let opt_a = data[2];
    let opt_b = data[3];
    let opt_c = data[4];
    let opt_d = data[5];
    let opt_e = data[6];
    let opt_f = data[7];

    let filter = pick_filter(opt_a);
    // §III.A subband-priority interleaving rides an otherwise-unused
    // opt_a bit (the filter uses only the low three).
    let priority_interleaving = (opt_a & 0x08) != 0;
    let wavelet_levels = (opt_b & 0x07).clamp(1, MAX_LEVELS);
    let bit_plane_count = ((opt_b >> 3) & 0x0F).max(1); // 1..=15
    let uncompressed = (opt_c & 0x01) != 0;
    let segment_count = ((opt_c >> 1) as u16 & 0x1F).clamp(1, MAX_SEGMENTS); // 1..=16
    let auto_filter = (opt_d & 0x01) != 0;
    let auto_filter_rd = (opt_d & 0x02) != 0;
    let rd_pruning = (opt_d & 0x04) != 0;
    let auto_uncompressed_fallback = (opt_d & 0x08) != 0;
    let use_byte_budget = (opt_d & 0x10) != 0;
    let use_target_bytes = (opt_d & 0x20) != 0;
    let use_priorities = (opt_d & 0x40) != 0;
    let interleaved_entropy = (opt_d & 0x80) != 0;
    // §V.B transform-domain segmentation + §VI.A minimum loss (0..=3).
    let transform_segments = (opt_b & 0x80) != 0;
    let min_loss = opt_c >> 6;

    // Budgets are derived from opt_e + opt_f so a fuzzer can drive the
    // encoder over the full "tiny-budget early-stop" through
    // "budget-larger-than-output" range without exploding allocation.
    // Cap at 64 KB which is generous for 128 x 128 Gray8.
    let budget_raw = ((opt_e as u64) << 8) | (opt_f as u64);
    let byte_budget = if use_byte_budget {
        Some((budget_raw % 65_537) + 1)
    } else {
        None
    };
    let target_bytes = if use_target_bytes {
        Some((budget_raw % 32_771) + 1)
    } else {
        None
    };

    let segment_priorities = if use_priorities {
        let seed = u64::from(opt_e) | (u64::from(opt_f) << 8) | (u64::from(opt_a) << 16);
        Some(permutation(segment_count, seed))
    } else {
        None
    };

    let opts = EncodeOptions {
        sync_prefix: 0xACED,
        filter,
        wavelet_levels,
        bit_plane_count,
        uncompressed,
        segment_count,
        byte_budget,
        target_bytes,
        auto_filter,
        auto_filter_rd,
        segment_priorities,
        rd_pruning,
        auto_uncompressed_fallback,
        quality_target_psnr: None,
        interleaved_entropy,
        transform_segments,
        min_loss,
        priority_interleaving,
    };

    // ---- Encode ------------------------------------------------------
    // The contract: encode returns either Ok(bytes) or Err(IcerError).
    // It must not panic, must not integer-overflow in debug, must not
    // allocate beyond what `width * height * planes` justifies.
    let encoded = match encode_icer(&img, &opts) {
        Ok(b) => b,
        Err(_) => return,
    };

    // Apply the same hard-cap promise the encoder advertises (modulo a
    // small slop for the segment header that is committed before the
    // first packet). If this fails it is a real bug in the budget
    // accounting.
    if let Some(budget) = byte_budget {
        // The advertised contract is "≤ budget bytes plus the segment
        // header already committed before the first packet". The
        // segment header is 12 bytes; allow up to segment_count * 12
        // as committed-header slop.
        let slop = (segment_count as u64) * 12;
        assert!(
            encoded.len() as u64 <= budget + slop,
            "encoded {} bytes > budget {} + slop {}",
            encoded.len(),
            budget,
            slop
        );
    }

    // ---- Decode (strict) --------------------------------------------
    // The encoder's output should always be parseable by the strict
    // decoder. Geometry must match.
    let decoded = parse_icer(&encoded).expect("strict decode of self-encoded stream failed");
    assert_eq!(
        decoded.width, width,
        "strict-decode width mismatch (encoded {width}, decoded {})",
        decoded.width
    );
    assert_eq!(
        decoded.height, height,
        "strict-decode height mismatch (encoded {height}, decoded {})",
        decoded.height
    );
    assert_eq!(
        decoded.pixel_format,
        IcerPixelFormat::Gray8,
        "strict-decode pixel format flipped"
    );

    // ---- Decode (lenient) -------------------------------------------
    // The lenient decoder must accept anything the strict one does. It
    // additionally tolerates missing segments, so a successful strict
    // decode is the strongest constraint.
    let _ = parse_icer_lenient(&encoded);
});
