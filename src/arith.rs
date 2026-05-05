//! Binary arithmetic coder scaffold — round-1 partial.
//!
//! ICER's entropy stage (IPN 42-155 §III.C "Adaptive Binary
//! Arithmetic Coding") is a context-modelled, adaptive binary
//! arithmetic coder. The paper describes it as a simplified
//! Witten-Neal-Cleary style range coder with:
//!
//!   * 16-bit range registers (`Hi` / `Lo`),
//!   * carry-propagation handled by the `pending` byte counter
//!     (a.k.a. "follow" counts in WNC),
//!   * one probability table indexed by *context*, where contexts
//!     come from [`crate::context`].
//!
//! IPN 42-155 §III.C does **not** publish a complete probability
//! transition table — the paper says "the probability estimator
//! adapts using a simple windowed counting scheme" and leaves the
//! window size + initial state to the implementation. Round 1 ships
//! the *register arithmetic* and a placeholder probability estimator
//! (Laplace-rule MAP estimate with a 64-symbol window) so the
//! round-trip test in `tests/arith_roundtrip.rs` passes; replacing
//! the estimator in round 2 will not require re-deriving the
//! register-arithmetic side.
//!
//! No JPL / qccPack / ground-system code consulted — the WNC range
//! coder is universally known and the variant here was written from
//! the original 1987 Communications of the ACM paper by Witten,
//! Neal and Cleary ("Arithmetic Coding for Data Compression") with
//! modifications in §III.C of IPN 42-155 explicitly noted.

use crate::error::{IcerError, Result};

/// Width of the range register, in bits. 16 matches the paper's
/// description in IPN 42-155 §III.C.
pub const RANGE_BITS: u32 = 16;
/// Top bit of the range register — `0x8000` for a 16-bit register.
const TOP: u32 = 1u32 << (RANGE_BITS - 1);
/// Second-from-top bit — `0x4000`.
const SECOND: u32 = 1u32 << (RANGE_BITS - 2);
const MASK: u32 = (1u32 << RANGE_BITS) - 1;

/// Encoder side of the 16-bit binary arithmetic coder. Produces a
/// big-endian byte stream that the matching [`ArithDecoder`] consumes.
pub struct ArithEncoder {
    /// Lower bound of the current interval, in `[0, 2^16)`.
    low: u32,
    /// Width of the current interval, in `[1, 2^16]`.
    range: u32,
    /// "Follow" counter — pending bits from a carry-propagation event
    /// that we haven't yet emitted because they could be flipped by a
    /// future carry. See WNC §3.4.
    pending: u32,
    /// Output bytes (big-endian per IPN 42-155 §IV "byte stream").
    out: Vec<u8>,
    /// Bit-accumulation buffer for the current partially-filled byte.
    bit_buf: u8,
    /// Number of valid bits (MSB first) currently held in `bit_buf`.
    bit_count: u8,
}

impl Default for ArithEncoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ArithEncoder {
    pub fn new() -> Self {
        Self {
            low: 0,
            range: MASK + 1, // initial range = 2^16
            pending: 0,
            out: Vec::new(),
            bit_buf: 0,
            bit_count: 0,
        }
    }

    /// Encode one binary symbol whose probability of being `1` is
    /// `p1_num / p1_den`. The estimator that drives this is
    /// [`crate::context::ContextModel::probability`].
    ///
    /// Both `p1_num` and `p1_den` are clamped into `[1, 2^15)` to
    /// avoid degenerate intervals — IPN 42-155 §III.C also recommends
    /// the encoder never feed a 0 or 1 probability to the arithmetic
    /// stage.
    pub fn encode_bit(&mut self, symbol: u8, p1_num: u32, p1_den: u32) {
        debug_assert!(symbol <= 1);
        let max_p = (1u32 << (RANGE_BITS - 1)) - 1;
        let p1_den = p1_den.clamp(2, 1u32 << (RANGE_BITS - 1));
        let p1_num = p1_num.clamp(1, p1_den - 1).min(max_p);
        // Width of the 0-symbol sub-interval = range * P(0) = range * (den - num) / den.
        let split = (self.range * (p1_den - p1_num)) / p1_den;
        let split = split.max(1).min(self.range - 1);
        if symbol == 0 {
            self.range = split;
        } else {
            self.low = (self.low + split) & MASK;
            self.range -= split;
        }
        self.renormalize();
    }

    /// Renormalisation loop — emits as many MSBs as are determined.
    /// Adapted from the WNC paper §3.4. The range can be at most
    /// `2^16` (initial); after each renormalisation step it doubles
    /// from a value `<= 2^15`, so it never exceeds the register.
    fn renormalize(&mut self) {
        loop {
            let high = self.low + self.range;
            if high <= TOP {
                self.emit_bit(0);
            } else if self.low >= TOP {
                self.emit_bit(1);
                self.low -= TOP;
            } else if self.low >= SECOND && high <= TOP + SECOND {
                self.pending += 1;
                self.low -= SECOND;
            } else {
                break;
            }
            self.low <<= 1;
            self.low &= MASK;
            self.range <<= 1;
        }
    }

    fn emit_bit(&mut self, bit: u8) {
        self.push_bit(bit);
        for _ in 0..self.pending {
            self.push_bit(1 - bit);
        }
        self.pending = 0;
    }

    fn push_bit(&mut self, bit: u8) {
        self.bit_buf = (self.bit_buf << 1) | (bit & 1);
        self.bit_count += 1;
        if self.bit_count == 8 {
            self.out.push(self.bit_buf);
            self.bit_buf = 0;
            self.bit_count = 0;
        }
    }

    /// Flush the trailing bits + return the produced byte stream.
    /// One extra bit (chosen to disambiguate the final interval) is
    /// emitted, as per WNC §3.4 modified for the SECOND-bit window.
    pub fn finish(mut self) -> Vec<u8> {
        self.pending += 1;
        if self.low < SECOND {
            self.emit_bit(0);
        } else {
            self.emit_bit(1);
        }
        if self.bit_count != 0 {
            self.out.push(self.bit_buf << (8 - self.bit_count));
        }
        self.out
    }
}

/// Decoder side. Consumes the byte stream produced by
/// [`ArithEncoder::finish`] and exposes [`Self::decode_bit`] for one
/// symbol at a time.
pub struct ArithDecoder<'a> {
    low: u32,
    range: u32,
    /// Code value — the running window into the input stream.
    value: u32,
    bytes: &'a [u8],
    /// Byte cursor into `bytes`.
    byte_pos: usize,
    /// Bits remaining in `bit_buf`.
    bit_count: u8,
    bit_buf: u8,
}

impl<'a> ArithDecoder<'a> {
    pub fn new(bytes: &'a [u8]) -> Result<Self> {
        let mut d = Self {
            low: 0,
            range: MASK + 1,
            value: 0,
            bytes,
            byte_pos: 0,
            bit_count: 0,
            bit_buf: 0,
        };
        // Pre-load 16 bits into `value`.
        for _ in 0..RANGE_BITS {
            d.value = (d.value << 1) | u32::from(d.read_bit()?);
        }
        Ok(d)
    }

    fn read_bit(&mut self) -> Result<u8> {
        if self.bit_count == 0 {
            if self.byte_pos < self.bytes.len() {
                self.bit_buf = self.bytes[self.byte_pos];
                self.byte_pos += 1;
                self.bit_count = 8;
            } else {
                // Past end of stream — return 0 bits (matches WNC's
                // standard treatment of trailing reads after EOF).
                return Ok(0);
            }
        }
        let bit = (self.bit_buf >> 7) & 1;
        self.bit_buf <<= 1;
        self.bit_count -= 1;
        Ok(bit)
    }

    pub fn decode_bit(&mut self, p1_num: u32, p1_den: u32) -> Result<u8> {
        let max_p = (1u32 << (RANGE_BITS - 1)) - 1;
        let p1_den = p1_den.clamp(2, 1u32 << (RANGE_BITS - 1));
        let p1_num = p1_num.clamp(1, p1_den - 1).min(max_p);
        let split = (self.range * (p1_den - p1_num)) / p1_den;
        let split = split.max(1).min(self.range - 1);
        let offset = self.value.wrapping_sub(self.low);
        let symbol = if offset < split {
            self.range = split;
            0u8
        } else {
            self.low = (self.low + split) & MASK;
            self.range -= split;
            1u8
        };
        self.renormalize_decoder()?;
        Ok(symbol)
    }

    fn renormalize_decoder(&mut self) -> Result<()> {
        loop {
            let high = self.low + self.range;
            if high <= TOP {
                // No-op: top bit is 0, just shift.
            } else if self.low >= TOP {
                self.low -= TOP;
                self.value = self.value.wrapping_sub(TOP) & MASK;
            } else if self.low >= SECOND && high <= TOP + SECOND {
                self.low -= SECOND;
                self.value = self.value.wrapping_sub(SECOND) & MASK;
            } else {
                break;
            }
            self.low <<= 1;
            self.low &= MASK;
            self.range <<= 1;
            self.value = ((self.value << 1) | u32::from(self.read_bit()?)) & MASK;
        }
        Ok(())
    }
}

/// Helper constructor used by the registry-feature error mapping.
#[allow(dead_code)]
pub(crate) fn _err_invalid(msg: &'static str) -> IcerError {
    IcerError::invalid(msg)
}
