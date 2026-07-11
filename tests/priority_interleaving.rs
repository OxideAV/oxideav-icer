//! IPN 42-155 §III.A subband-priority interleaving, end to end.
//!
//! `EncodeOptions::with_priority_interleaving()` switches a compressed
//! segment's packet schedule from the crate's historical whole-strip
//! MSB-down packet pairs to the spec's progressive order: subband bit
//! planes walked in decreasing §III.A priority (Fig. 7 weights halved
//! per plane, ties to the higher decomposition level then LL, HL, LH,
//! HH), each subband bit plane coded in a single combined raster pass
//! (§III: sign immediately after the first nonzero magnitude bit), one
//! packet per priority group. This file pins:
//!
//!   * bit-exact filter-Q lossless round-trips across geometries,
//!     levels, both entropy backends, colour, row-strip and §V.B
//!     transform-domain segmentation;
//!   * the wire flag (previously-reserved header byte 2 top bit) and
//!     that legacy streams stay byte-identical + flag-clear;
//!   * §VI.A min-loss composition (whole subband bit planes dropped
//!     from the schedule, monotone byte / MSE curves, M = 0 identity);
//!   * budget-truncation behaviour: monotone quality in budget, full
//!     geometry always framed, and the headline §III.A property — a
//!     truncated priority-interleaved stream reconstructs at higher
//!     PSNR than the whole-strip MSB-down order at equal byte budgets;
//!   * the rd_pruning mutual exclusion.

use oxideav_icer::{
    encode_icer, parse_icer, walk_segment, EncodeOptions, IcerImage, IcerPixelFormat, WaveletFilter,
};

fn textured(w: usize, h: usize, seed: u64) -> IcerImage {
    let mut img = IcerImage::zeros(w as u32, h as u32, IcerPixelFormat::Gray8);
    let mut s = seed.wrapping_mul(0x9E37_79B9_7F4A_7C15);
    let stride = img.planes[0].stride;
    for y in 0..h {
        for x in 0..w {
            s = s
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            let noise = ((s >> 56) & 0x3F) as i32;
            let base = ((x * 3 + y * 2) % 180) as i32;
            img.planes[0].data[y * stride + x] = (base + noise).clamp(0, 255) as u8;
        }
    }
    img
}

fn mse(a: &IcerImage, b: &IcerImage) -> f64 {
    let n = (a.width * a.height) as f64;
    a.planes[0]
        .data
        .iter()
        .zip(b.planes[0].data.iter())
        .map(|(&x, &y)| {
            let d = x as f64 - y as f64;
            d * d
        })
        .sum::<f64>()
        / n
}

fn psnr(a: &IcerImage, b: &IcerImage) -> f64 {
    let m = mse(a, b);
    if m == 0.0 {
        f64::INFINITY
    } else {
        10.0 * (255.0f64 * 255.0 / m).log10()
    }
}

fn assert_bit_exact(a: &IcerImage, b: &IcerImage, what: &str) {
    assert_eq!(a.width, b.width, "{what}: width");
    assert_eq!(a.height, b.height, "{what}: height");
    for (pa, pb) in a.planes.iter().zip(b.planes.iter()) {
        for y in 0..a.height as usize {
            let ra = &pa.data[y * pa.stride..y * pa.stride + a.width as usize];
            let rb = &pb.data[y * pb.stride..y * pb.stride + b.width as usize];
            assert_eq!(ra, rb, "{what}: row {y}");
        }
    }
}

/// Filter-Q full-quality decode is bit-exact across geometries,
/// decomposition levels, and both entropy backends.
#[test]
fn lossless_bit_exact_geometry_matrix() {
    for &(w, h) in &[(17usize, 13usize), (31, 31), (64, 64), (5, 200), (200, 5)] {
        for levels in 1..=5u8 {
            for interleaved in [false, true] {
                let img = textured(w, h, 7);
                let mut opts = EncodeOptions::compressed().with_priority_interleaving();
                opts.wavelet_levels = levels;
                if interleaved {
                    opts = opts.with_interleaved_entropy();
                }
                let bytes = encode_icer(&img, &opts).unwrap();
                let dec = parse_icer(&bytes).unwrap();
                assert_bit_exact(
                    &img,
                    &dec,
                    &format!("{w}x{h} D={levels} interleaved={interleaved}"),
                );
            }
        }
    }
}

/// The wire flag rides header byte 2's previously-reserved top bit:
/// set on priority-interleaved segments, clear on legacy ones, and the
/// legacy wire form is byte-identical to what it was before the flag
/// existed (flag bit zero, same packets).
#[test]
fn wire_flag_and_legacy_byte_identity() {
    let img = textured(48, 40, 3);
    let legacy = encode_icer(&img, &EncodeOptions::compressed()).unwrap();
    let walked = walk_segment(&legacy).unwrap();
    assert!(!walked.header.priority_interleaved);
    assert_eq!(legacy[2] & 0x80, 0, "legacy streams keep the bit clear");

    let prio = encode_icer(
        &img,
        &EncodeOptions::compressed().with_priority_interleaving(),
    )
    .unwrap();
    let walked = walk_segment(&prio).unwrap();
    assert!(walked.header.priority_interleaved);
    assert_eq!(prio[2] & 0x80, 0x80);
    // Same filter / levels / geometry parse out of the shared fields.
    assert_eq!(walked.header.filter, WaveletFilter::FilterQ);
    assert_eq!(walked.header.width, 48);

    // Both decode bit-exact.
    assert_bit_exact(&img, &parse_icer(&legacy).unwrap(), "legacy");
    assert_bit_exact(&img, &parse_icer(&prio).unwrap(), "priority");
}

/// The wire packets follow the deterministic
/// `priority::packet_schedule` exactly: same count, and each packet
/// header carries its schedule entry's priority-group index. The
/// schedule is recomputed from header fields alone, which is what lets
/// the decoder replay it without any signalling.
#[test]
fn wire_packets_match_schedule() {
    let img = textured(64, 64, 11);
    let opts = EncodeOptions::compressed().with_priority_interleaving();
    let bytes = encode_icer(&img, &opts).unwrap();
    let walked = walk_segment(&bytes).unwrap();
    let q = walked.header.bit_plane_count as u32;
    let d = walked.header.decomp_levels;
    let schedule = oxideav_icer::priority::packet_schedule(d, q, 0, 64, 64);
    assert!(!walked.packets.is_empty());
    assert_eq!(
        walked.packets.len(),
        schedule.len(),
        "one wire packet per schedule entry (D={d}, q={q})"
    );
    for (p, sp) in walked.packets.iter().zip(schedule.iter()) {
        assert_eq!(
            p.header.bit_plane as usize, sp.group_index,
            "packet header carries the priority-group index"
        );
    }
    // Group indices are non-decreasing (priority order on the wire) and
    // the schedule spans every group: D + q + 1 distinct values.
    let mut groups: Vec<usize> = walked
        .packets
        .iter()
        .map(|p| p.header.bit_plane as usize)
        .collect();
    assert!(groups.windows(2).all(|w| w[0] <= w[1]));
    groups.dedup();
    assert_eq!(groups.len(), d as usize + q as usize + 1);
}

/// Colour (YUV 4:4:4) composes: three independent priority-interleaved
/// plane streams, bit-exact on filter Q.
#[test]
fn colour_roundtrip_bit_exact() {
    let mut img = IcerImage::zeros(32, 24, IcerPixelFormat::Yuv444P);
    for (pi, plane) in img.planes.iter_mut().enumerate() {
        let stride = plane.stride;
        for y in 0..24usize {
            for x in 0..32usize {
                plane.data[y * stride + x] = ((x * (pi + 2) + y * 3) % 251) as u8;
            }
        }
    }
    let opts = EncodeOptions::compressed().with_priority_interleaving();
    let bytes = encode_icer(&img, &opts).unwrap();
    let dec = parse_icer(&bytes).unwrap();
    assert_eq!(dec.pixel_format, IcerPixelFormat::Yuv444P);
    assert_bit_exact(&img, &dec, "colour 4:4:4");
}

/// Row-strip multi-segment images compose (each strip is its own
/// §III.A schedule), bit-exact on filter Q.
#[test]
fn row_strip_multi_segment_bit_exact() {
    for segs in [2u16, 4] {
        let img = textured(48, 64, 21);
        let mut opts = EncodeOptions::compressed().with_priority_interleaving();
        opts.segment_count = segs;
        let bytes = encode_icer(&img, &opts).unwrap();
        assert_bit_exact(
            &img,
            &parse_icer(&bytes).unwrap(),
            &format!("{segs} strips"),
        );
    }
}

/// §V.B transform-domain segmentation composes: whole-image DWT, §V.D
/// partition, each segment coded with its own §III.A priority schedule.
/// Bit-exact on filter Q, both entropy backends.
#[test]
fn transform_domain_segments_bit_exact() {
    for segs in [2u16, 5, 8] {
        for interleaved in [false, true] {
            let img = textured(72, 56, 15);
            let mut opts = EncodeOptions::compressed()
                .with_priority_interleaving()
                .with_transform_domain_segments();
            opts.segment_count = segs;
            if interleaved {
                opts = opts.with_interleaved_entropy();
            }
            let bytes = encode_icer(&img, &opts).unwrap();
            let dec = parse_icer(&bytes).unwrap();
            assert_bit_exact(
                &img,
                &dec,
                &format!("transform-domain s={segs} interleaved={interleaved}"),
            );
        }
    }
}

/// §VI.A minimum loss composes at schedule granularity: `M = 0` is
/// byte-identical to the plain priority-interleaved encode, bytes are
/// monotone non-increasing and MSE monotone non-decreasing in `M`, and
/// every stream decodes to the full geometry.
#[test]
fn min_loss_composes_monotone() {
    let img = textured(64, 64, 5);
    let plain = encode_icer(
        &img,
        &EncodeOptions::compressed().with_priority_interleaving(),
    )
    .unwrap();
    let m0 = encode_icer(
        &img,
        &EncodeOptions::compressed()
            .with_priority_interleaving()
            .with_min_loss(0),
    )
    .unwrap();
    assert_eq!(plain, m0, "M = 0 must be byte-identical");

    let mut prev_bytes = usize::MAX;
    let mut prev_mse = -1.0f64;
    for m in [0u8, 1, 2, 3, 4, 6, 8] {
        let bytes = encode_icer(
            &img,
            &EncodeOptions::compressed()
                .with_priority_interleaving()
                .with_min_loss(m),
        )
        .unwrap();
        let dec = parse_icer(&bytes).unwrap();
        assert_eq!(dec.width, 64);
        assert_eq!(dec.height, 64);
        let e = mse(&img, &dec);
        assert!(
            bytes.len() <= prev_bytes,
            "bytes must not grow with M (M={m}: {} > {prev_bytes})",
            bytes.len()
        );
        assert!(
            e >= prev_mse,
            "MSE must not shrink with M (M={m}: {e} < {prev_mse})"
        );
        prev_bytes = bytes.len();
        prev_mse = e;
    }
    // M = 0 is lossless.
    let dec = parse_icer(&m0).unwrap();
    assert_bit_exact(&img, &dec, "M = 0 lossless");
}

/// rd_pruning is a competing packet scheduler and is rejected.
#[test]
fn rd_pruning_rejected() {
    let img = textured(32, 32, 9);
    let opts = EncodeOptions::compressed()
        .with_priority_interleaving()
        .with_rd_budget(400);
    let err = encode_icer(&img, &opts).unwrap_err();
    let msg = format!("{err}");
    assert!(
        msg.contains("priority_interleaving"),
        "unexpected error: {msg}"
    );
}

/// Budget truncation: quality is monotone non-decreasing in the byte
/// budget, the hard cap is honoured, and the full image geometry is
/// always framed (tiny budgets included).
#[test]
fn budget_truncation_monotone_and_framed() {
    let img = textured(64, 64, 13);
    let mut prev = -1.0f64;
    for budget in [16u64, 60, 120, 250, 500, 1000, 2000, 4000, 8000] {
        let opts = EncodeOptions::compressed()
            .with_priority_interleaving()
            .with_byte_budget(budget);
        let bytes = encode_icer(&img, &opts).unwrap();
        assert!(bytes.len() as u64 <= budget.max(12), "cap honoured");
        let dec = parse_icer(&bytes).unwrap();
        assert_eq!((dec.width, dec.height), (64, 64), "geometry framed");
        let p = psnr(&img, &dec);
        assert!(
            p >= prev - 1e-9,
            "PSNR must be monotone in budget ({budget} B: {p:.2} < {prev:.2})"
        );
        prev = p;
    }
}

/// The headline §III.A property: at equal byte budgets a truncated
/// priority-interleaved stream reconstructs at higher PSNR than the
/// whole-strip MSB-down packet order, because the spec order defers
/// low-weight level-1 HH bits in favour of high-weight LL planes.
/// Measured on the textured 64x64 fixture (filter Q, 3 levels): +4.3 /
/// +5.6 dB at deep truncation (250 / 500 B), +1.6 dB through the
/// mid-range, mean +1.9 dB over the sweep, with one -2.7 dB outlier at
/// 2500 B — the "scalloping" the paper itself documents for
/// priority-boundary quantisation (§VI.B). Pins the deep-truncation
/// wins, the mean, and a floor under the scalloping.
#[test]
fn truncated_quality_beats_msb_down_order() {
    let img = textured(64, 64, 13);
    let mut gains = Vec::new();
    for budget in [250u64, 500, 750, 1000, 1500, 2000, 2500, 3000] {
        let legacy =
            encode_icer(&img, &EncodeOptions::compressed().with_byte_budget(budget)).unwrap();
        let prio = encode_icer(
            &img,
            &EncodeOptions::compressed()
                .with_priority_interleaving()
                .with_byte_budget(budget),
        )
        .unwrap();
        let p_legacy = psnr(&img, &parse_icer(&legacy).unwrap());
        let p_prio = psnr(&img, &parse_icer(&prio).unwrap());
        let gain = p_prio - p_legacy;
        println!(
            "budget {budget:>5}: legacy {} B / {p_legacy:.2} dB, priority {} B / {p_prio:.2} dB, gain {gain:+.2} dB",
            legacy.len(),
            prio.len()
        );
        gains.push(gain);
    }
    let mean = gains.iter().sum::<f64>() / gains.len() as f64;
    println!("mean gain {mean:+.2} dB");
    assert!(
        gains.iter().all(|&g| g > -3.5),
        "priority order must stay above the §VI.B scalloping floor: {gains:?}"
    );
    assert!(
        gains[0] > 2.0 && gains[1] > 2.0,
        "deep truncation (the downlink case §III.A exists for) must win decisively: {gains:?}"
    );
    assert!(
        mean > 1.25,
        "priority order must win clearly on average (mean {mean:+.2} dB)"
    );
}

/// Lossless rate: the §III.A schedule costs a bounded framing overhead
/// against the legacy order — more packets (one per schedule entry:
/// the D + q + 1 priority groups plus one per fat unit, vs the legacy
/// 2q pairs) means more packet headers + entropy-coder flushes.
/// Measured ~+3% on 64x64 filter-Q lossless; the persistent context
/// model claws back most of the per-packet model-reset cost. Pinned
/// <= +5%: the mode exists for progressive-truncation optimality;
/// pure-lossless archival keeps the default order.
#[test]
fn lossless_rate_bounded_overhead() {
    for seed in [5u64, 13, 21] {
        let img = textured(64, 64, seed);
        let legacy = encode_icer(&img, &EncodeOptions::compressed()).unwrap();
        let prio = encode_icer(
            &img,
            &EncodeOptions::compressed().with_priority_interleaving(),
        )
        .unwrap();
        println!(
            "seed {seed}: legacy {} B, priority {} B ({:+.1}%)",
            legacy.len(),
            prio.len(),
            100.0 * (prio.len() as f64 - legacy.len() as f64) / legacy.len() as f64
        );
        assert!(
            prio.len() as f64 <= legacy.len() as f64 * 1.05,
            "priority-interleaved lossless overhead must stay bounded \
             (seed {seed}: {} vs {})",
            prio.len(),
            legacy.len()
        );
    }
}

/// The soft byte target composes: the encoder finishes the in-progress
/// packet once the target is met, and the hard cap still binds.
#[test]
fn soft_target_composes() {
    let img = textured(64, 64, 17);
    let opts = EncodeOptions::compressed()
        .with_priority_interleaving()
        .with_target_bytes(600)
        .with_byte_budget(1200);
    let bytes = encode_icer(&img, &opts).unwrap();
    assert!(
        bytes.len() >= 600
            || bytes.len() < 600 && {
                // A stream naturally smaller than the target is legal.
                let full = encode_icer(
                    &img,
                    &EncodeOptions::compressed().with_priority_interleaving(),
                )
                .unwrap();
                full.len() < 600
            }
    );
    assert!(bytes.len() as u64 <= 1200);
    let dec = parse_icer(&bytes).unwrap();
    assert_eq!((dec.width, dec.height), (64, 64));
}

/// quality_target composes with the §III.A schedule: the binary search
/// runs over priority-interleaved trial encodes and meets the floor.
#[test]
fn quality_target_composes() {
    let img = textured(64, 64, 29);
    let opts = EncodeOptions::compressed()
        .with_priority_interleaving()
        .with_quality_target(30.0);
    let bytes = encode_icer(&img, &opts).unwrap();
    let dec = parse_icer(&bytes).unwrap();
    assert!(psnr(&img, &dec) >= 30.0);
    // And it is smaller than the lossless stream (30 dB is well below
    // the lossless ceiling on this textured fixture).
    let lossless = encode_icer(
        &img,
        &EncodeOptions::compressed().with_priority_interleaving(),
    )
    .unwrap();
    assert!(bytes.len() < lossless.len());
}

/// §III.D per-segment uncompressed fallback composes: a noise strip
/// takes the raw path, and the priority flag stays a compressed-path
/// property (clear on raw segments).
#[test]
fn uncompressed_fallback_composes() {
    // Pure-noise tile: raw wins.
    let mut img = IcerImage::zeros(32, 32, IcerPixelFormat::Gray8);
    let mut s = 0xDEADBEEFu64;
    let stride = img.planes[0].stride;
    for y in 0..32usize {
        for x in 0..32usize {
            s = s
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            img.planes[0].data[y * stride + x] = (s >> 56) as u8;
        }
    }
    let opts = EncodeOptions::compressed()
        .with_priority_interleaving()
        .with_uncompressed_fallback();
    let bytes = encode_icer(&img, &opts).unwrap();
    let walked = walk_segment(&bytes).unwrap();
    if walked.header.uncompressed {
        assert!(
            !walked.header.priority_interleaved,
            "raw segments must not carry the priority flag"
        );
    }
    assert_bit_exact(&img, &parse_icer(&bytes).unwrap(), "fallback");
}
