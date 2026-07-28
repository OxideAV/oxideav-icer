//! Deterministic mutation smoke over the §V.B transform-domain and
//! §VI.A minimum-loss wire forms.
//!
//! The scheduled fuzz workflow gets these paths via the checked-in
//! corpus seeds; this test gives every push a bounded, reproducible
//! bit-flip / truncation sweep through the same entry-point stack so a
//! decoder panic on a corrupted new-mode stream fails CI immediately.
//! Every decode outcome (Ok or Err) is acceptable — only panics,
//! aborts, and runaway allocation (bounded by the tight limits) are
//! failures.

use oxideav_icer::{
    encode_icer, encode_icer3d, parse_icer3d_with_limits, parse_icer_lenient_with_limits,
    parse_icer_metadata, parse_icer_with_limits, walk_segment, CubeEncodeOptions, DecodeLimits,
    EncodeOptions, IcerCube, IcerImage, IcerPixelFormat,
};

// Tight geometry caps: the seeds are 32x32, so valid decodes always
// fit, while a mutated width/height byte cannot buy a quarter-MPx
// inverse DWT per iteration (the sweeps run tens of thousands of
// decode attempts).
const LIMITS: DecodeLimits = DecodeLimits {
    max_pixels_per_segment: 1 << 13,
    max_total_pixels: 1 << 15,
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
            let noise = ((s >> 56) & 0x1F) as i32;
            let base = ((x * 2 + y * 3) % 200) as i32;
            img.planes[0].data[y * stride + x] = (base + noise).clamp(0, 255) as u8;
        }
    }
    img
}

fn drive(bytes: &[u8]) {
    let _ = walk_segment(bytes);
    let _ = parse_icer_metadata(bytes);
    let _ = parse_icer_with_limits(bytes, &LIMITS);
    let _ = parse_icer_lenient_with_limits(bytes, &LIMITS);
    let _ = parse_icer3d_with_limits(bytes, &LIMITS);
    let _ = oxideav_icer::parse_icer3d_lenient_with_limits(bytes, &LIMITS);
}

fn seeds() -> Vec<Vec<u8>> {
    let img = textured(32, 32, 0x5EED);
    let mut tf = EncodeOptions::compressed().with_transform_domain_segments();
    tf.segment_count = 4;
    tf.wavelet_levels = 2;
    let mut out = vec![encode_icer(&img, &tf).unwrap()];
    out.push(encode_icer(&img, &tf.clone().with_byte_budget(200)).unwrap());
    out.push(encode_icer(&img, &EncodeOptions::compressed().with_min_loss(3)).unwrap());
    out.push(
        encode_icer(
            &img,
            &tf.clone().with_min_loss(2).with_interleaved_entropy(),
        )
        .unwrap(),
    );
    // §III.A subband-priority interleaving (r405): plain, min-loss +
    // interleaved-entropy, and composed with §V.B transform-domain
    // segments — the three wire shapes the priority flag admits.
    out.push(
        encode_icer(
            &img,
            &EncodeOptions::compressed().with_priority_interleaving(),
        )
        .unwrap(),
    );
    out.push(
        encode_icer(
            &img,
            &EncodeOptions::compressed()
                .with_priority_interleaving()
                .with_min_loss(2)
                .with_interleaved_entropy(),
        )
        .unwrap(),
    );
    out.push(encode_icer(&img, &tf.with_priority_interleaving()).unwrap());
    // Deep-sample (tag 2) container wire forms: compressed, §V.B
    // transform-domain, and §III.D raw — the depth-byte decode paths.
    let mut deep = IcerImage::zeros(24, 20, IcerPixelFormat::GrayDeep { bits: 12 });
    for y in 0..20u32 {
        for x in 0..24u32 {
            deep.set_sample(0, x, y, ((x * 97 + y * 57 + (x ^ y) * 31) % 4096) as u16);
        }
    }
    out.push(encode_icer(&deep, &EncodeOptions::compressed()).unwrap());
    let mut deep_tf = EncodeOptions::compressed().with_transform_domain_segments();
    deep_tf.segment_count = 4;
    out.push(encode_icer(&deep, &deep_tf).unwrap());
    out.push(encode_icer(&deep, &EncodeOptions::default()).unwrap());
    // ICER-3D cube wire forms: §V.D transform-domain segments (r414),
    // plain and quota-truncated — the flags-bit-1 decode paths.
    let mut cube = IcerCube::zeros(16, 16, 4, 10);
    let mut s = 0xC0BEu64;
    for v in cube.samples.iter_mut() {
        s = s
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        *v = ((s >> 54) & 0x3FF) as u16;
    }
    let td = CubeEncodeOptions::default()
        .with_transform_domain_segments()
        .with_segment_count(4)
        .with_levels(2);
    out.push(encode_icer3d(&cube, &td).unwrap());
    out.push(encode_icer3d(&cube, &td.clone().with_byte_quota(400)).unwrap());
    out
}

/// Single-byte mutations across every offset (headers, packet framing,
/// entropy bodies): must never panic.
#[test]
fn single_byte_mutations_are_panic_free() {
    let mut rng = 0x1234_5678_9ABC_DEF0u64;
    for seed in seeds() {
        let mut pos = 0usize;
        while pos < seed.len() {
            rng = rng
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            let mut m = seed.clone();
            m[pos] ^= ((rng >> 33) as u8).max(1);
            drive(&m);
            // Exhaustive over the framing region (headers live in the
            // first bytes of each segment); sampled through the
            // entropy bodies, whose bytes all exercise the same
            // decode surface.
            pos += if pos < 64 { 1 } else { 7 };
        }
    }
}

/// Header-focused byte sweeps: every value of the mode-bearing bytes
/// (byte 2 filter/levels/flag, byte 7 Q/backends/§V.B flag, bytes
/// 10/11 total/index, packet byte 3 min-loss) on a valid stream.
#[test]
fn header_field_sweeps_are_panic_free() {
    for seed in seeds() {
        for &pos in &[2usize, 7, 10, 11, 15] {
            if pos >= seed.len() {
                continue;
            }
            for v in 0..=255u8 {
                let mut m = seed.clone();
                m[pos] = v;
                drive(&m);
            }
        }
    }
}

/// Every truncation point of every seed: must never panic.
#[test]
fn truncations_are_panic_free() {
    for seed in seeds() {
        for cut in 0..seed.len() {
            drive(&seed[..cut]);
        }
    }
}

/// Cross-seed splices (transform stream + min-loss stream fragments).
#[test]
fn splices_are_panic_free() {
    let all = seeds();
    for a in &all {
        for b in &all {
            let mut m = a.clone();
            m.extend_from_slice(&b[..b.len() / 2]);
            drive(&m);
            let mut m2 = a[..a.len() / 2].to_vec();
            m2.extend_from_slice(b);
            drive(&m2);
        }
    }
}
