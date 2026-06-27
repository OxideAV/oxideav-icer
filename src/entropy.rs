//! Entropy-stage abstraction over ICER's two binary coders.
//!
//! The bit-plane significance / refinement passes (`crate::bitplane`)
//! drive a per-bit binary coder through exactly two operations: encode a
//! bit given a probability-of-one estimate, and the inverse on decode.
//! Two coders implement that contract:
//!
//! * [`crate::arith::ArithEncoder`] / [`crate::arith::ArithDecoder`] — a
//!   Witten-Neal-Cleary binary arithmetic coder (the crate's original
//!   entropy stage).
//! * [`crate::ixec::IxecEncoder`] / [`crate::ixec::IxecDecoder`] — ICER's
//!   own §IV interleaved entropy coder.
//!
//! Abstracting the two behind [`BitSink`] / [`BitSource`] lets the
//! bit-plane passes be written once and run on either backend, so the
//! context-model logic is guaranteed identical across both — the only
//! difference is which entropy coder consumes the (bit, probability)
//! stream. This is what makes a future switch onto the spec-exact §IV
//! coder a backend swap rather than a re-implementation of the passes.

use crate::arith::{ArithDecoder, ArithEncoder};
use crate::error::Result;
use crate::ixec::{IxecDecoder, IxecEncoder};

/// The encode side of ICER's entropy stage: accept one binary `symbol`
/// (0 or 1) with the probability that it is a `1` given as `p1_num /
/// p1_den`, and finish into the channel byte stream.
pub trait BitSink {
    /// Encode one binary symbol (`0` or `1`).
    fn put_bit(&mut self, symbol: u8, p1_num: u32, p1_den: u32);
    /// Flush + return the produced channel byte stream.
    fn finish_bits(self: Box<Self>) -> Vec<u8>;
}

/// The decode side: recover one binary symbol given the same probability
/// estimate the encoder used.
pub trait BitSource {
    /// Decode one binary symbol (`0` or `1`).
    fn get_bit(&mut self, p1_num: u32, p1_den: u32) -> Result<u8>;
}

impl BitSink for ArithEncoder {
    fn put_bit(&mut self, symbol: u8, p1_num: u32, p1_den: u32) {
        self.encode_bit(symbol, p1_num, p1_den);
    }
    fn finish_bits(self: Box<Self>) -> Vec<u8> {
        (*self).finish()
    }
}

impl BitSource for ArithDecoder<'_> {
    fn get_bit(&mut self, p1_num: u32, p1_den: u32) -> Result<u8> {
        self.decode_bit(p1_num, p1_den)
    }
}

impl BitSink for IxecEncoder {
    fn put_bit(&mut self, symbol: u8, p1_num: u32, p1_den: u32) {
        self.encode_bit(symbol, p1_num, p1_den);
    }
    fn finish_bits(self: Box<Self>) -> Vec<u8> {
        (*self).finish()
    }
}

impl BitSource for IxecDecoder<'_> {
    fn get_bit(&mut self, p1_num: u32, p1_den: u32) -> Result<u8> {
        // The interleaved decoder is infallible over a well-formed
        // channel; map its bool to the 0/1 symbol the passes expect.
        Ok(self.decode_bit(p1_num, p1_den))
    }
}

/// Which entropy backend a compressed segment uses. The arithmetic coder
/// is the crate's established wire form; the interleaved coder is the
/// spec-exact §IV path being brought up.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EntropyKind {
    /// Witten-Neal-Cleary binary arithmetic coder.
    Arithmetic,
    /// ICER's §IV interleaved entropy coder.
    Interleaved,
}

impl EntropyKind {
    /// A boxed encoder for this backend.
    pub fn make_sink(self) -> Box<dyn BitSink> {
        match self {
            EntropyKind::Arithmetic => Box::new(ArithEncoder::new()),
            EntropyKind::Interleaved => Box::new(IxecEncoder::new()),
        }
    }

    /// A boxed decoder for this backend over `channel`.
    pub fn make_source<'a>(self, channel: &'a [u8]) -> Result<Box<dyn BitSource + 'a>> {
        Ok(match self {
            EntropyKind::Arithmetic => Box::new(ArithDecoder::new(channel)?),
            EntropyKind::Interleaved => Box::new(IxecDecoder::new(channel)),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Both backends round-trip a fixed (symbol, probability) stream
    /// through the trait-object surface the passes will use.
    fn backend_roundtrip(kind: EntropyKind) {
        let symbols: Vec<u8> = (0..300).map(|i| ((i * 29 + 7) % 8 == 0) as u8).collect();
        let (p1n, p1d) = (1u32, 8u32);
        let mut sink = kind.make_sink();
        for &s in &symbols {
            sink.put_bit(s, p1n, p1d);
        }
        let channel = sink.finish_bits();
        let mut src = kind.make_source(&channel).unwrap();
        for (i, &s) in symbols.iter().enumerate() {
            assert_eq!(src.get_bit(p1n, p1d).unwrap(), s, "{kind:?} symbol {i}");
        }
    }

    #[test]
    fn arithmetic_backend_roundtrip() {
        backend_roundtrip(EntropyKind::Arithmetic);
    }

    #[test]
    fn interleaved_backend_roundtrip() {
        backend_roundtrip(EntropyKind::Interleaved);
    }
}
