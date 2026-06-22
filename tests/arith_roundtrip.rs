//! Round-trip cover for the binary arithmetic coder + adaptive
//! context model.

use oxideav_icer::arith::{ArithDecoder, ArithEncoder};
use oxideav_icer::context::{
    magnitude_context, sign_context, significance_context, ContextModel, MagnitudeContext,
    CONTEXT_COUNT,
};

fn deterministic_bits(n: usize, seed: u64) -> Vec<u8> {
    let mut s = seed.wrapping_mul(0x9E37_79B9_7F4A_7C15);
    (0..n)
        .map(|_| {
            s = s
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            ((s >> 33) & 1) as u8
        })
        .collect()
}

#[test]
fn arith_uniform_50_50_roundtrip() {
    let bits = deterministic_bits(1024, 0xDEAD);
    let mut enc = ArithEncoder::new();
    for &b in &bits {
        enc.encode_bit(b, 1, 2);
    }
    let stream = enc.finish();
    let mut dec = ArithDecoder::new(&stream).unwrap();
    let decoded: Vec<u8> = (0..bits.len())
        .map(|_| dec.decode_bit(1, 2).unwrap())
        .collect();
    assert_eq!(decoded, bits);
}

#[test]
fn arith_skewed_75_25_roundtrip() {
    let bits = deterministic_bits(2048, 0xBEEF);
    let mut enc = ArithEncoder::new();
    for &b in &bits {
        // P(1) = 1/4
        enc.encode_bit(b, 1, 4);
    }
    let stream = enc.finish();
    let mut dec = ArithDecoder::new(&stream).unwrap();
    let decoded: Vec<u8> = (0..bits.len())
        .map(|_| dec.decode_bit(1, 4).unwrap())
        .collect();
    assert_eq!(decoded, bits);
}

#[test]
fn arith_with_adaptive_context_roundtrip() {
    let bits = deterministic_bits(4096, 0xCAFE);
    let ctxs: Vec<usize> = (0..bits.len()).map(|i| i % CONTEXT_COUNT).collect();

    // Encoder side.
    let mut enc_model = ContextModel::new();
    let mut enc = ArithEncoder::new();
    for (b, c) in bits.iter().zip(ctxs.iter()) {
        let (num, den) = enc_model.probability(*c);
        enc.encode_bit(*b, num, den);
        enc_model.observe(*c, *b);
    }
    let stream = enc.finish();

    // Decoder side — must run the same context updates in lock-step.
    let mut dec_model = ContextModel::new();
    let mut dec = ArithDecoder::new(&stream).unwrap();
    for (b, c) in bits.iter().zip(ctxs.iter()) {
        let (num, den) = dec_model.probability(*c);
        let got = dec.decode_bit(num, den).unwrap();
        assert_eq!(got, *b);
        dec_model.observe(*c, got);
    }
}

#[test]
fn context_helpers_return_in_range_indices() {
    for pat in 0..=255u8 {
        assert!(significance_context(pat) < CONTEXT_COUNT);
    }
    for h in 0..16u8 {
        for v in 0..16u8 {
            assert!(sign_context(h, v) < CONTEXT_COUNT);
        }
    }
    // Category-aware magnitude (refinement) contexts: categories 1 and 2
    // map to coded contexts in range; category 3+ is left uncoded.
    for cat in 0..=4u8 {
        for hv in [false, true] {
            match magnitude_context(cat, hv) {
                MagnitudeContext::Coded(c) => assert!(c < CONTEXT_COUNT),
                MagnitudeContext::Uncoded => {}
            }
        }
    }
    // Category 1/2 must be coded; category 3 must be uncoded.
    assert!(matches!(
        magnitude_context(1, false),
        MagnitudeContext::Coded(_)
    ));
    assert!(matches!(
        magnitude_context(2, true),
        MagnitudeContext::Coded(_)
    ));
    assert_eq!(magnitude_context(3, true), MagnitudeContext::Uncoded);
}
