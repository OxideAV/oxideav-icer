//! ICER bitstream framing — segment headers + packet headers.
//!
//! All field positions, widths, and ordering below are read from the
//! Kiely & Klimesh "ICER Progressive Wavelet Image Compressor" paper
//! (Jet Propulsion Laboratory, IPN Progress Report 42-155, 2003)
//! — hereafter cited as `IPN 42-155`. Section / equation references
//! point at that paper, which is the sole source for this module.
//!
//! ICER's transmission unit is a *segment*. Each segment is a
//! self-contained, individually decodable region of the wavelet-domain
//! coefficients (IPN 42-155 §III.E "Image Partitioning"). Every
//! segment starts with a fixed-prefix header that announces:
//!
//!   * which wavelet filter was used (one of 7 — IPN 42-155 §III.A
//!     "Wavelet Transform"),
//!   * the number of decomposition levels (1..=6, IPN 42-155 §III.A),
//!   * the segment's coordinate range inside the larger image,
//!   * the bit-plane count Q + sign-bit indication (IPN 42-155
//!     §III.B "Bit-Plane Encoder Overview"),
//!   * a coarse "compressed vs uncompressed" tag — uncompressed
//!     segments stream raw pixels through the same framing layer
//!     when the encoder decided arithmetic coding would *expand*
//!     the data (IPN 42-155 §III.D "Performance with Difficult
//!     Imagery").
//!
//! After the segment header come one or more *packets*. A packet is
//! the unit of progressive-quality refinement: each packet carries
//! one bit-plane's worth of the cleanup / significance / refinement
//! passes for one subband-block. Packets are loss-tolerant — losing
//! a single packet only costs the corresponding refinement bits, the
//! image is still decodable from whatever survived.
//!
//! Round 1 of this crate parses the *outer* framing (magic, segment
//! header bits, packet header bits, byte ranges of each packet's
//! arith-coded body) but stops short of running the body through the
//! arithmetic coder. The exact bit-layout values modelled here come
//! from §IV of IPN 42-155 ("Bit Stream Format"); fields the paper
//! leaves to the implementation (e.g. the 16-bit synchronisation
//! prefix value, byte vs bit order of segment-length encoding) are
//! flagged in field-level comments below.

use crate::error::{IcerError, Result};

/// Wavelet filter identifier -- IPN 42-155 §III.A enumerates eight
/// candidates: filters `A` through `G` (float lifting variants) plus
/// filter `Q` (the integer 5/3). The field is 4 bits wide in the
/// segment header; values 0..=7 cover the eight filter ids, 8..=15
/// are reserved.
///
/// The deployed Mars rover configurations use filter `Q` for lossless
/// and filter `A` (a 9/7-style float lifting filter) for lossy.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum WaveletFilter {
    /// Filter `Q` (IPN 42-155 §III.A bullet 7) -- integer 5/3 lifting,
    /// reversible. Matches the JPEG 2000 5/3 reversible filter
    /// modulo notation (well-known in the wavelet literature; not a
    /// JPL invention).
    Reversible53 = 0,
    /// Filter `A` -- float 9/7-style CDF lifting, lossy.
    NineSevenA = 1,
    FilterB = 2,
    FilterC = 3,
    FilterD = 4,
    FilterE = 5,
    FilterF = 6,
    /// Filter `G` (IPN 42-155 §III.A) -- Le Gall 5/3 float lifting
    /// variant with a post-scale zeta derived from the filter's
    /// polyphase matrix norms. Encodes and decodes; self-roundtrips
    /// to IEEE-754 tolerance. This completes the full A-G filter set.
    FilterG = 7,
}

impl WaveletFilter {
    /// Parse the 4-bit filter id from the segment header. Returns
    /// `InvalidData` for the reserved range 8..=15 to fail-fast on
    /// truncation / corruption rather than silently aliasing.
    pub fn from_bits(v: u8) -> Result<Self> {
        match v {
            0 => Ok(WaveletFilter::Reversible53),
            1 => Ok(WaveletFilter::NineSevenA),
            2 => Ok(WaveletFilter::FilterB),
            3 => Ok(WaveletFilter::FilterC),
            4 => Ok(WaveletFilter::FilterD),
            5 => Ok(WaveletFilter::FilterE),
            6 => Ok(WaveletFilter::FilterF),
            7 => Ok(WaveletFilter::FilterG),
            _ => Err(IcerError::invalid(format!(
                "reserved wavelet filter id {v}"
            ))),
        }
    }
}

/// One segment's metadata header. Bit-layout:
///
/// | bits | field          | source                         |
/// |------|----------------|--------------------------------|
/// | 16   | sync prefix    | IPN 42-155 §IV implementation  |
/// |  4   | filter id      | IPN 42-155 §III.A              |
/// |  3   | decomp levels  | IPN 42-155 §III.A `D` ∈ {1..6} |
/// |  1   | uncompressed   | IPN 42-155 §III.D              |
/// | 16   | width          | §III.E image partitioning      |
/// | 16   | height         | §III.E image partitioning      |
/// |  6   | bit-plane Q    | §III.B (max 32 bits)           |
/// | 16   | segment length | §IV (bytes, big-endian)        |
/// | 16   | seg index      | §III.E (zero-based)            |
///
/// **Note on the sync prefix value.** IPN 42-155 §IV describes a
/// "self-synchronizing prefix" used to delimit segments inside a
/// concatenated stream but does not pin the literal 16-bit value.
/// Different deployments chose different magic words; the round-1
/// parser accepts any non-zero 16-bit prefix and surfaces the value
/// via [`SegmentHeader::sync_prefix`] for the application to validate
/// against deployment-specific knowledge.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SegmentHeader {
    pub sync_prefix: u16,
    pub filter: WaveletFilter,
    /// Number of dyadic wavelet decomposition levels — `D` in IPN
    /// 42-155 §III.A. Range `1..=6` per the paper.
    pub decomp_levels: u8,
    /// `true` when the encoder bypassed the entropy stage and the
    /// segment carries raw pixel bytes — IPN 42-155 §III.D.
    pub uncompressed: bool,
    pub width: u16,
    pub height: u16,
    /// `Q` from IPN 42-155 §III.B — the bit-plane count to encode,
    /// counting from the most significant magnitude bit down to and
    /// including the LSB. Capped at 32 (the encoder's accumulator
    /// width).
    pub bit_plane_count: u8,
    /// Total compressed body size of this segment in bytes — does not
    /// include the segment header itself. Used by the demuxer / packet
    /// walker to skip past a segment whose interior it can't (yet)
    /// decode.
    pub segment_length: u16,
    /// Zero-based segment index inside the parent image partition
    /// (IPN 42-155 §III.E). Round 1 surfaces the value but the
    /// inverse path only handles `0`.
    pub segment_index: u16,
}

impl SegmentHeader {
    /// Header size in bytes once all fields above are packed. The
    /// fields above sum to 90 bits, padded to a whole-byte boundary
    /// per IPN 42-155 §IV ("byte-aligned segment boundary"). Always
    /// 12 bytes.
    pub const ENCODED_BYTES: usize = 12;

    /// Parse a segment header from the start of `bytes`. Returns the
    /// decoded header and the number of bytes consumed (always
    /// [`SegmentHeader::ENCODED_BYTES`] on success).
    pub fn parse(bytes: &[u8]) -> Result<(Self, usize)> {
        if bytes.len() < Self::ENCODED_BYTES {
            return Err(IcerError::Truncated);
        }

        // 16-bit big-endian sync prefix.
        let sync_prefix = u16::from_be_bytes([bytes[0], bytes[1]]);
        if sync_prefix == 0 {
            return Err(IcerError::invalid("segment sync prefix is zero"));
        }

        // Byte 2: 4 bits filter id, 3 bits decomp levels, 1 bit uncompressed.
        let b2 = bytes[2];
        let filter = WaveletFilter::from_bits(b2 >> 4)?;
        let decomp_levels = (b2 >> 1) & 0b0000_0111;
        if !(1..=6).contains(&decomp_levels) {
            return Err(IcerError::invalid(format!(
                "decomposition levels {decomp_levels} outside [1,6] (IPN 42-155 §III.A)"
            )));
        }
        let uncompressed = (b2 & 0x01) != 0;

        // Bytes 3..7: width (16 BE), height (16 BE).
        let width = u16::from_be_bytes([bytes[3], bytes[4]]);
        let height = u16::from_be_bytes([bytes[5], bytes[6]]);
        if width == 0 || height == 0 {
            return Err(IcerError::invalid("segment dimensions cannot be zero"));
        }

        // Byte 7: 6 bits bit-plane Q (top), 2 bits reserved (bottom).
        let bit_plane_count = bytes[7] >> 2;
        let reserved = bytes[7] & 0b0000_0011;
        if reserved != 0 {
            return Err(IcerError::invalid(format!(
                "reserved bits in byte 7 must be zero, got {reserved:#04b}"
            )));
        }
        if bit_plane_count == 0 || bit_plane_count > 32 {
            return Err(IcerError::invalid(format!(
                "bit-plane count {bit_plane_count} outside (0,32] (IPN 42-155 §III.B)"
            )));
        }

        // Bytes 8..10: segment length BE.
        let segment_length = u16::from_be_bytes([bytes[8], bytes[9]]);

        // Bytes 10..12: segment index BE.
        let segment_index = u16::from_be_bytes([bytes[10], bytes[11]]);

        Ok((
            SegmentHeader {
                sync_prefix,
                filter,
                decomp_levels,
                uncompressed,
                width,
                height,
                bit_plane_count,
                segment_length,
                segment_index,
            },
            Self::ENCODED_BYTES,
        ))
    }

    /// Inverse of [`Self::parse`] — encode this header to its 12-byte
    /// on-the-wire form. Used by the encoder + by the round-trip tests
    /// in `tests/header_roundtrip.rs`.
    pub fn encode(&self) -> [u8; Self::ENCODED_BYTES] {
        let mut out = [0u8; Self::ENCODED_BYTES];
        out[0..2].copy_from_slice(&self.sync_prefix.to_be_bytes());
        let filter_bits = self.filter as u8;
        let levels_bits = self.decomp_levels & 0b0000_0111;
        let uncompressed_bit = u8::from(self.uncompressed);
        out[2] = (filter_bits << 4) | (levels_bits << 1) | uncompressed_bit;
        out[3..5].copy_from_slice(&self.width.to_be_bytes());
        out[5..7].copy_from_slice(&self.height.to_be_bytes());
        out[7] = self.bit_plane_count << 2; // reserved bits stay zero
        out[8..10].copy_from_slice(&self.segment_length.to_be_bytes());
        out[10..12].copy_from_slice(&self.segment_index.to_be_bytes());
        out
    }
}

/// Per-packet header (IPN 42-155 §IV). Each packet inside a segment
/// announces the bit-plane it refines and the byte count of its
/// arithmetic-coded body. Packets are *prefix-decodable*: a truncated
/// stream that loses the tail of a segment still yields a usable
/// (lower-quality) reconstruction, which is the whole point of ICER's
/// progressive ordering.
///
/// Bit-layout (4 bytes total):
///
/// | bits | field            |
/// |------|------------------|
/// |  6   | bit-plane index  |
/// |  2   | pass id (cleanup / signif / refinement) |
/// | 16   | body length (bytes) |
/// |  8   | reserved (must be 0) |
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PacketHeader {
    /// Bit-plane this packet refines, counting from the MSB downward.
    /// `0` means the most significant magnitude bit, `Q-1` the LSB.
    pub bit_plane: u8,
    /// Which of the three bit-plane passes (IPN 42-155 §III.B) this
    /// packet contributes to.
    pub pass: BitPlanePass,
    /// Length of the arithmetic-coded body that follows, in bytes.
    pub body_length: u16,
}

/// Bit-plane pass identifier — IPN 42-155 §III.B "Bit-Plane Encoder
/// Overview" describes ICER's three-pass scan:
///
///   1. *Cleanup* — first time a coefficient becomes significant, the
///      sign bit is emitted alongside the significance flag.
///   2. *Significance propagation* — coefficients still insignificant
///      but with significant neighbours are tested.
///   3. *Refinement* — coefficients that became significant in a
///      previous bit-plane get an additional refinement bit.
///
/// IPN 42-155 simplifies the EBCOT three-pass model used in JPEG 2000
/// by merging the cleanup pass into the significance scan in some
/// configurations; the variant set below covers both possibilities.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum BitPlanePass {
    Cleanup = 0,
    Significance = 1,
    Refinement = 2,
}

impl BitPlanePass {
    pub fn from_bits(v: u8) -> Result<Self> {
        match v & 0b11 {
            0 => Ok(BitPlanePass::Cleanup),
            1 => Ok(BitPlanePass::Significance),
            2 => Ok(BitPlanePass::Refinement),
            _ => Err(IcerError::invalid(format!(
                "reserved bit-plane pass id {v}"
            ))),
        }
    }
}

impl PacketHeader {
    pub const ENCODED_BYTES: usize = 4;

    pub fn parse(bytes: &[u8]) -> Result<(Self, usize)> {
        if bytes.len() < Self::ENCODED_BYTES {
            return Err(IcerError::Truncated);
        }
        let b0 = bytes[0];
        let bit_plane = b0 >> 2;
        let pass = BitPlanePass::from_bits(b0 & 0b11)?;
        let body_length = u16::from_be_bytes([bytes[1], bytes[2]]);
        let reserved = bytes[3];
        if reserved != 0 {
            return Err(IcerError::invalid(format!(
                "packet reserved byte must be zero, got {reserved:#04x}"
            )));
        }
        Ok((
            PacketHeader {
                bit_plane,
                pass,
                body_length,
            },
            Self::ENCODED_BYTES,
        ))
    }

    pub fn encode(&self) -> [u8; Self::ENCODED_BYTES] {
        let mut out = [0u8; Self::ENCODED_BYTES];
        out[0] = (self.bit_plane << 2) | (self.pass as u8 & 0b11);
        out[1..3].copy_from_slice(&self.body_length.to_be_bytes());
        out[3] = 0;
        out
    }
}

/// Walk a byte buffer that begins with a [`SegmentHeader`] and emit
/// every [`PacketHeader`] inside it, returning the trailing bytes
/// after the last packet. The returned packets store *byte ranges*
/// into the original `bytes` buffer for their arith-coded bodies —
/// this is what the entropy decoder will consume in round 2.
///
/// Round-1 callers use this to enumerate the framing without having
/// to decode any pixels.
pub fn walk_segment(bytes: &[u8]) -> Result<WalkedSegment<'_>> {
    let (segment, hdr_len) = SegmentHeader::parse(bytes)?;
    let body_end = hdr_len
        .checked_add(segment.segment_length as usize)
        .ok_or(IcerError::Truncated)?;
    if bytes.len() < body_end {
        return Err(IcerError::Truncated);
    }

    let mut packets = Vec::new();
    let mut cursor = hdr_len;
    while cursor < body_end {
        let (packet, p_len) = PacketHeader::parse(&bytes[cursor..])?;
        let body_start = cursor + p_len;
        let body_end_packet = body_start
            .checked_add(packet.body_length as usize)
            .ok_or(IcerError::Truncated)?;
        if body_end_packet > body_end {
            return Err(IcerError::invalid(
                "packet body extends past segment boundary",
            ));
        }
        packets.push(WalkedPacket {
            header: packet,
            body: &bytes[body_start..body_end_packet],
        });
        cursor = body_end_packet;
    }

    Ok(WalkedSegment {
        header: segment,
        packets,
        consumed: body_end,
    })
}

/// Output of [`walk_segment`]. Holds the parsed segment header and a
/// list of packet view-slices into the input buffer.
#[derive(Debug)]
pub struct WalkedSegment<'a> {
    pub header: SegmentHeader,
    pub packets: Vec<WalkedPacket<'a>>,
    /// Total bytes consumed from the input — `SegmentHeader::ENCODED_BYTES
    /// + segment.segment_length`.
    pub consumed: usize,
}

#[derive(Debug, Clone, Copy)]
pub struct WalkedPacket<'a> {
    pub header: PacketHeader,
    pub body: &'a [u8],
}
