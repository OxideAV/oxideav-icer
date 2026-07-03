//! IPN 42-155 §VI.A minimum-loss quality goal — end-to-end coverage.
//!
//! "The minimum loss parameter is a nonnegative integer that determines
//! a minimum number of bit planes that will not be encoded in each
//! subband": a subband with Fig. 18 offset `o` keeps its `max(0, M-o)`
//! LSB planes out of the stream. `M = 0` is lossless (byte quota
//! allowing); if all but `k` planes of a subband are encoded its pixels
//! are in effect quantised with step `2^k` (§VI.A / §III.A).

use oxideav_icer::{
    encode_icer, parse_icer, parse_icer_metadata, walk_segment, EncodeOptions, IcerImage,
    IcerPixelFormat,
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

/// `with_min_loss(0)` is byte-identical to the plain compressed encode
/// (M = 0 is the historical wire form).
#[test]
fn m0_is_wire_identical() {
    let img = textured(64, 64, 0x111);
    let plain = encode_icer(&img, &EncodeOptions::compressed()).unwrap();
    let m0 = encode_icer(&img, &EncodeOptions::compressed().with_min_loss(0)).unwrap();
    assert_eq!(plain, m0);
}

/// §VI.A: growing M shrinks the output monotonically and degrades MSE
/// monotonically; M = 0 stays lossless on filter Q.
#[test]
fn min_loss_is_monotone_in_bytes_and_mse() {
    let img = textured(64, 64, 0x222);
    let mut last_bytes = usize::MAX;
    let mut last_mse = -1.0f64;
    for m in 0..=8u8 {
        let opts = EncodeOptions::compressed().with_min_loss(m);
        let bytes = encode_icer(&img, &opts).unwrap();
        let dec = parse_icer(&bytes).unwrap();
        let e = mse(&img, &dec);
        if m == 0 {
            assert_eq!(dec.planes[0].data, img.planes[0].data, "M = 0 is lossless");
        }
        assert!(
            bytes.len() <= last_bytes,
            "M = {m}: {} bytes regressed above {last_bytes}",
            bytes.len()
        );
        assert!(
            e >= last_mse - 1e-9,
            "M = {m}: MSE {e} must not improve on M-1's {last_mse}"
        );
        last_bytes = bytes.len();
        last_mse = e;
    }
    // The knob is real: M = 8 must be materially smaller than lossless.
    let lossless = encode_icer(&img, &EncodeOptions::compressed()).unwrap();
    let m8 = encode_icer(&img, &EncodeOptions::compressed().with_min_loss(8)).unwrap();
    assert!(
        m8.len() * 2 < lossless.len(),
        "M = 8 ({}) should be well under half of lossless ({})",
        m8.len(),
        lossless.len()
    );
}

/// The parameter rides every packet header (byte 3), so the decoder
/// learns it from any surviving packet.
#[test]
fn min_loss_is_on_every_packet_header() {
    let img = textured(32, 32, 0x333);
    let bytes = encode_icer(&img, &EncodeOptions::compressed().with_min_loss(3)).unwrap();
    let walked = walk_segment(&bytes).unwrap();
    assert!(!walked.packets.is_empty());
    for p in &walked.packets {
        assert_eq!(p.header.min_loss, 3);
    }
}

/// §VI.A: "When the original image has a dynamic range of B bits per
/// pixel, D stages of wavelet decomposition are used, and M >= B + D,
/// then little or no bit-plane information will be encoded." B = 8,
/// D = 3.
#[test]
fn saturating_m_encodes_almost_nothing() {
    let img = textured(64, 64, 0x444);
    let lossless = encode_icer(&img, &EncodeOptions::compressed()).unwrap();
    let m_big = encode_icer(&img, &EncodeOptions::compressed().with_min_loss(11)).unwrap();
    assert!(
        m_big.len() * 10 < lossless.len(),
        "M = B + D = 11 must encode almost nothing ({} vs {})",
        m_big.len(),
        lossless.len()
    );
    // And it still decodes cleanly to the full geometry.
    let dec = parse_icer(&m_big).unwrap();
    assert_eq!((dec.width, dec.height), (64, 64));
}

/// Multi-segment (row-strip) encodes carry the exclusion per segment
/// and decode without desynchronising.
#[test]
fn min_loss_composes_with_row_strips() {
    let img = textured(64, 64, 0x555);
    for m in [1u8, 3, 5] {
        let mut opts = EncodeOptions::compressed().with_min_loss(m);
        opts.segment_count = 4;
        let bytes = encode_icer(&img, &opts).unwrap();
        let dec = parse_icer(&bytes).unwrap();
        assert_eq!((dec.width, dec.height), (64, 64), "M = {m}");
        let meta = parse_icer_metadata(&bytes).unwrap();
        assert_eq!(meta.segments.len(), 4);
    }
}

/// min-loss composes with the §V.B transform-domain path: fewer bytes
/// than the M = 0 transform encode, clean decode, monotone MSE.
#[test]
fn min_loss_composes_with_transform_segments() {
    let img = textured(64, 64, 0x666);
    let mut base = EncodeOptions::compressed().with_transform_domain_segments();
    base.segment_count = 4;
    let m0 = encode_icer(&img, &base).unwrap();
    let dec0 = parse_icer(&m0).unwrap();
    assert_eq!(dec0.planes[0].data, img.planes[0].data);

    let mut last_bytes = usize::MAX;
    let mut last_mse = -1.0f64;
    for m in [0u8, 2, 4, 6] {
        let opts = base.clone().with_min_loss(m);
        let bytes = encode_icer(&img, &opts).unwrap();
        let dec = parse_icer(&bytes).unwrap();
        let e = mse(&img, &dec);
        assert!(bytes.len() <= last_bytes, "M = {m} transform bytes");
        assert!(e >= last_mse - 1e-9, "M = {m} transform MSE");
        last_bytes = bytes.len();
        last_mse = e;
    }
}

/// min-loss composes with both entropy backends.
#[test]
fn min_loss_composes_with_interleaved_backend() {
    let img = textured(48, 48, 0x777);
    let opts = EncodeOptions::compressed()
        .with_interleaved_entropy()
        .with_min_loss(3);
    let bytes = encode_icer(&img, &opts).unwrap();
    let dec = parse_icer(&bytes).unwrap();
    assert_eq!((dec.width, dec.height), (48, 48));
    let plain = encode_icer(
        &img,
        &EncodeOptions::compressed().with_interleaved_entropy(),
    )
    .unwrap();
    assert!(bytes.len() < plain.len());
}

/// §VI: "ICER stops producing compressed bytes once the quality goal or
/// byte quota is met, whichever comes first" — min-loss + byte budget
/// compose; the cap always binds.
#[test]
fn min_loss_composes_with_byte_budget() {
    let img = textured(64, 64, 0x888);
    for budget in [400u64, 1200] {
        let opts = EncodeOptions::compressed()
            .with_min_loss(2)
            .with_byte_budget(budget);
        let bytes = encode_icer(&img, &opts).unwrap();
        assert!(bytes.len() as u64 <= budget);
        let dec = parse_icer(&bytes).unwrap();
        assert_eq!((dec.width, dec.height), (64, 64));
    }
}

/// Invalid combinations are refused loudly.
#[test]
fn min_loss_rejects_uncompressed_and_rd() {
    let img = textured(32, 32, 0x999);
    // Forced-uncompressed path has no bit planes.
    let opts = EncodeOptions {
        min_loss: 1,
        ..EncodeOptions::default()
    };
    assert!(encode_icer(&img, &opts).is_err());
    // R-D packet selection is a separate rate-control mode.
    let opts = EncodeOptions::compressed()
        .with_min_loss(1)
        .with_rd_budget(500);
    assert!(encode_icer(&img, &opts).is_err());
}
