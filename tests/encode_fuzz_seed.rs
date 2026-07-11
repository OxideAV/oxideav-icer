//! Per-push smoke test for the encode-side fuzz harness logic.
//!
//! The round-199 `encode_roundtrip` cargo-fuzz target builds an
//! `IcerImage` and an `EncodeOptions` from arbitrary fuzz bytes, calls
//! [`encode_icer`], and self-roundtrips through both
//! [`parse_icer`] and [`parse_icer_lenient`]. The full fuzz harness runs
//! daily off the cron in `.github/workflows/fuzz.yml`; this test runs
//! the *same* extraction + drive logic on a small bank of hand-picked
//! seed inputs every push so a regression in the encoder's
//! input-validation surface shows up in normal CI instead of waiting
//! ~24 h for the next fuzz cron.
//!
//! The seeds are chosen to hit the matrix of option flags the encoder
//! exposes (uncompressed force, segment count, byte budget, ROI
//! priorities, R-D pruning, automatic uncompressed fallback) plus the
//! eight wavelet filters and a couple of pathological geometries
//! (1x1, 1x128, 128x1).

use oxideav_icer::{
    encode_icer, parse_icer, parse_icer_lenient, EncodeOptions, IcerImage, IcerPixelFormat,
    WaveletFilter,
};

const MAX_DIM: u32 = 128;
const MAX_SEGMENTS: u16 = 16;
const MAX_LEVELS: u8 = 4;

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

fn permutation(n: u16, seed: u64) -> Vec<u16> {
    let n = n as usize;
    let mut v: Vec<u16> = (0..n as u16).collect();
    let mut state = seed.wrapping_mul(0x9E37_79B9_7F4A_7C15).wrapping_add(1);
    for i in (1..n).rev() {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        let j = (state as usize) % (i + 1);
        v.swap(i, j);
    }
    v
}

/// Mirror of the `encode_roundtrip` fuzz target. Returns `true` if the
/// input was driven through encode + (on Ok) the strict decoder + the
/// lenient decoder with all invariants intact; returns `false` if the
/// input was too short to drive.
fn drive(data: &[u8]) -> bool {
    if data.len() < 8 {
        return false;
    }

    let width = (data[0] as u32 % MAX_DIM) + 1;
    let height = (data[1] as u32 % MAX_DIM) + 1;
    let pixel_count = (width as usize) * (height as usize);

    let pixel_src = &data[8..];
    let mut pixels = vec![0u8; pixel_count];
    if !pixel_src.is_empty() {
        for (i, p) in pixels.iter_mut().enumerate() {
            *p = pixel_src[i % pixel_src.len()];
        }
    }
    let mut img = IcerImage::zeros(width, height, IcerPixelFormat::Gray8);
    img.planes[0].data = pixels;

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
    let bit_plane_count = ((opt_b >> 3) & 0x0F).max(1);
    let uncompressed = (opt_c & 0x01) != 0;
    let segment_count = ((opt_c >> 1) as u16 & 0x1F).clamp(1, MAX_SEGMENTS);
    let auto_filter = (opt_d & 0x01) != 0;
    let auto_filter_rd = (opt_d & 0x02) != 0;
    let rd_pruning = (opt_d & 0x04) != 0;
    let auto_uncompressed_fallback = (opt_d & 0x08) != 0;
    let interleaved_entropy = (opt_d & 0x80) != 0;
    let use_byte_budget = (opt_d & 0x10) != 0;
    let use_target_bytes = (opt_d & 0x20) != 0;
    let use_priorities = (opt_d & 0x40) != 0;

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
        transform_segments: false,
        min_loss: 0,
        priority_interleaving,
    };

    let encoded = match encode_icer(&img, &opts) {
        Ok(b) => b,
        Err(_) => return true,
    };

    if let Some(budget) = byte_budget {
        let slop = (segment_count as u64) * 12;
        assert!(
            encoded.len() as u64 <= budget + slop,
            "encoded {} bytes > budget {} + slop {}",
            encoded.len(),
            budget,
            slop
        );
    }

    let decoded = parse_icer(&encoded).expect("strict decode of self-encoded stream failed");
    assert_eq!(decoded.width, width, "strict-decode width mismatch");
    assert_eq!(decoded.height, height, "strict-decode height mismatch");
    assert_eq!(
        decoded.pixel_format,
        IcerPixelFormat::Gray8,
        "strict-decode pixel format flipped"
    );

    let _ = parse_icer_lenient(&encoded);
    true
}

/// Seed inputs hand-chosen to exercise the option matrix the encoder
/// exposes. Each one is at least 8 bytes (the harness's minimum) and
/// carries enough pixel bytes (a few hundred at most — the harness
/// tiles them across the image) to make the encoder do real work.
fn seeds() -> Vec<Vec<u8>> {
    let mut out = Vec::new();

    // 1. Tiny 16x16 uncompressed, default options.
    out.push({
        let mut v = vec![15, 15, 0, 0, 0, 0, 0, 0];
        v.extend(std::iter::repeat(0x55).take(256));
        v
    });

    // 2. 32x32 compressed, filter Q, 3 levels, 8 bit-planes.
    out.push({
        let mut v = vec![31, 31, 0, 0b0100_0011, 0, 0, 0, 0];
        v.extend((0..1024u32).map(|x| x as u8));
        v
    });

    // 3. 64x16, compressed, filter G (id 7), 4 levels, multi-segment.
    out.push({
        let mut v = vec![63, 15, 7, 0b0100_0100, 0b0000_1000, 0, 0, 0];
        v.extend((0..1024u32).map(|x| (x * 3) as u8));
        v
    });

    // 4. 64x64, byte_budget on, target_bytes on, R-D pruning on,
    //    8 segments, ROI permutation.
    out.push({
        let mut v = vec![63, 63, 0, 0b0100_0011, 0b0001_0000, 0b0111_0100, 0x02, 0x00];
        v.extend((0..4096u32).map(|x| (x ^ (x >> 3)) as u8));
        v
    });

    // 5. 128x128 with all the flags on simultaneously (auto_filter +
    //    auto_filter_rd + rd_pruning + auto_uncompressed_fallback +
    //    byte_budget + target_bytes + priorities).
    out.push({
        let mut v = vec![
            127,
            127,
            0,
            0b0100_0011,
            0b0010_0000,
            0b0111_1111,
            0x10,
            0x00,
        ];
        v.extend((0..16384u32).map(|x| ((x as f32).sin() * 127.0 + 128.0) as u8));
        v
    });

    // 6. Pathological narrow image 1x32 (height-only ramp).
    out.push({
        let mut v = vec![0, 31, 0, 0b0100_0011, 0, 0, 0, 0];
        v.extend((0..32u32).map(|x| x as u8));
        v
    });

    // 7. Pathological wide image 32x1.
    out.push({
        let mut v = vec![31, 0, 0, 0b0100_0011, 0, 0, 0, 0];
        v.extend((0..32u32).map(|x| x as u8));
        v
    });

    // 8. 1x1 single pixel — edge case for stride / wavelet bounds.
    out.push(vec![0, 0, 0, 0b0100_0011, 0, 0, 0, 0, 0xAA]);

    // 9. Filter sweep — one seed per filter id 0..=7.
    for f in 0u8..=7 {
        let mut v = vec![31, 31, f, 0b0100_0011, 0, 0, 0, 0];
        v.extend((0..1024u32).map(|x| (x.wrapping_mul(f as u32 + 1)) as u8));
        out.push(v);
    }

    // 10. Tiny byte_budget (16 bytes) — the encoder must early-stop
    //     well before producing any meaningful packet.
    out.push({
        let mut v = vec![63, 63, 0, 0b0100_0011, 0, 0b0001_0000, 0x00, 0x10];
        v.extend((0..4096u32).map(|x| x as u8));
        v
    });

    // 11. Scheduled-fuzz crash regression: forced-uncompressed
    //     SINGLE-segment encode with a byte budget (125x128 image,
    //     budget 1537). The single-segment path used to return the raw
    //     §III.D emission (16016 bytes) without any budget check; it
    //     must emit the zero-body placeholder instead. Byte-exact fuzz
    //     artifact, also seeded at fuzz/corpus/encode_roundtrip/
    //     seed_single_segment_uncompressed_budget.bin.
    out.push(vec![
        0xfc, 0xff, 0xff, 0xf1, 0x01, 0xff, 0x06, 0x00, 0x04, 0x0a,
    ]);

    out
}

#[test]
fn encode_fuzz_seed_inputs_panic_free_and_roundtrip() {
    let seeds = seeds();
    let mut driven = 0usize;
    for (i, s) in seeds.iter().enumerate() {
        assert!(
            drive(s),
            "seed {i} ({} bytes) was rejected as too short",
            s.len()
        );
        driven += 1;
    }
    // Sanity check: every seed was actually driven through the harness.
    assert_eq!(driven, seeds.len());
}
