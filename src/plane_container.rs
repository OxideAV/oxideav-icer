//! Multi-plane (colour) container framing.
//!
//! IPN 42-155 §III describes ICER as fundamentally a **single-component**
//! coder; the paper's §III closing remark on multi-band imagery states
//! that the deployed colour scheme runs ICER independently on each colour
//! component, sharing only the outer image metadata. This crate models
//! that exactly: a colour ([`IcerPixelFormat::Yuv444P`]) image is encoded
//! as N independent single-plane ICER bitstreams, concatenated behind a
//! small container header so the decoder can split them apart again.
//!
//! # Why a separate container header
//!
//! A single-plane (`Gray8`) ICER stream is a bare concatenation of
//! [`crate::header::SegmentHeader`]-framed segments — the on-the-wire form
//! this crate has always produced. To stay **byte-for-byte backward
//! compatible** with every previously-encoded Gray8 stream, the colour
//! container is only emitted for multi-plane images, and it is
//! distinguished by a leading 16-bit sentinel of `0x0000`.
//!
//! That sentinel can never collide with a single-plane stream: the very
//! first two bytes of any valid segment are its 16-bit synchronisation
//! prefix, which [`crate::header::SegmentHeader::parse`] rejects when zero
//! (a zero prefix is treated as corruption). So a decoder can dispatch on
//! the first two bytes with no ambiguity:
//!
//!   * first two bytes `== 0x0000` -> multi-plane container;
//!   * otherwise -> a single-plane (Gray8) stream, decoded exactly as
//!     before.
//!
//! # Wire layout
//!
//! ```text
//! | bytes | field            | notes                                  |
//! |-------|------------------|----------------------------------------|
//! |   2   | sentinel 0x0000  | container marker (BE)                  |
//! |   1   | format tag       | IcerPixelFormat discriminant           |
//! |   1   | plane count N    | redundant with format; cross-checked   |
//! |  4*N  | plane lengths    | byte length of each plane substream, BE|
//! |  ...  | plane 0 substream| a full single-plane ICER bitstream     |
//! |  ...  | plane 1 substream|                                        |
//! |  ...  | ...              |                                        |
//! ```
//!
//! Each plane substream is itself a complete, independently-decodable
//! single-plane ICER bitstream (one or more segments). The container adds
//! no per-plane semantics beyond "here are N independent luma streams";
//! the luma↔chroma interpretation lives entirely in the [`IcerPixelFormat`]
//! tag.

use crate::error::{IcerError, Result};
use crate::image::IcerPixelFormat;

/// Leading 16-bit sentinel that marks a multi-plane container. Chosen as
/// `0x0000` precisely because a valid single-plane stream can never begin
/// with it (segment sync prefixes are non-zero).
pub const CONTAINER_SENTINEL: u16 = 0x0000;

/// Fixed size of the container header *before* the per-plane length table:
/// 2-byte sentinel + 1-byte format tag + 1-byte plane count.
const FIXED_PREFIX_BYTES: usize = 4;

/// Format-tag discriminant written to the container header. Kept stable
/// and explicit (not `as u8` on the enum) so the wire encoding does not
/// silently shift if the enum's declaration order ever changes.
fn format_tag(fmt: IcerPixelFormat) -> u8 {
    match fmt {
        IcerPixelFormat::Gray8 => 0,
        IcerPixelFormat::Yuv444P => 1,
    }
}

/// Inverse of [`format_tag`].
fn format_from_tag(tag: u8) -> Result<IcerPixelFormat> {
    match tag {
        0 => Ok(IcerPixelFormat::Gray8),
        1 => Ok(IcerPixelFormat::Yuv444P),
        other => Err(IcerError::invalid(format!(
            "unknown plane-container format tag {other}"
        ))),
    }
}

/// True when `bytes` begins with the multi-plane container sentinel.
///
/// A single-plane Gray8 stream never starts with `0x0000` (its first two
/// bytes are a non-zero segment sync prefix), so this is an unambiguous
/// one-shot dispatch.
pub fn is_container(bytes: &[u8]) -> bool {
    bytes.len() >= 2 && bytes[0] == 0x00 && bytes[1] == 0x00
}

/// Frame `plane_streams` (one complete single-plane ICER bitstream per
/// plane) into a multi-plane container with the given pixel `format`.
///
/// The number of plane streams must match `format.plane_count()`.
pub fn encode_container(format: IcerPixelFormat, plane_streams: &[Vec<u8>]) -> Result<Vec<u8>> {
    let n = format.plane_count();
    if plane_streams.len() != n {
        return Err(IcerError::invalid(format!(
            "plane-container expects {n} plane streams for {format:?}, got {}",
            plane_streams.len()
        )));
    }
    if n > u8::MAX as usize {
        return Err(IcerError::Unsupported(format!(
            "plane count {n} exceeds container capacity"
        )));
    }
    for (i, s) in plane_streams.iter().enumerate() {
        if s.len() > u32::MAX as usize {
            return Err(IcerError::Unsupported(format!(
                "plane {i} substream length {} exceeds 32-bit container field",
                s.len()
            )));
        }
    }

    let total: usize =
        FIXED_PREFIX_BYTES + 4 * n + plane_streams.iter().map(|s| s.len()).sum::<usize>();
    let mut out = Vec::with_capacity(total);
    out.extend_from_slice(&CONTAINER_SENTINEL.to_be_bytes());
    out.push(format_tag(format));
    out.push(n as u8);
    for s in plane_streams {
        out.extend_from_slice(&(s.len() as u32).to_be_bytes());
    }
    for s in plane_streams {
        out.extend_from_slice(s);
    }
    Ok(out)
}

/// A parsed multi-plane container: the declared pixel format plus the byte
/// range of each plane substream.
#[derive(Debug, Clone)]
pub struct ParsedContainer {
    /// Declared pixel format of the colour image.
    pub format: IcerPixelFormat,
    /// One byte range `(start, end)` per plane substream, relative to the
    /// original container buffer.
    pub plane_ranges: Vec<(usize, usize)>,
}

impl ParsedContainer {
    /// Borrow plane `i`'s substream bytes out of the original buffer.
    pub fn plane_bytes<'a>(&self, bytes: &'a [u8], i: usize) -> &'a [u8] {
        let (s, e) = self.plane_ranges[i];
        &bytes[s..e]
    }
}

/// Parse a multi-plane container header from the start of `bytes`,
/// returning the declared format and the byte range of each plane
/// substream. The caller must have already confirmed [`is_container`].
pub fn parse_container(bytes: &[u8]) -> Result<ParsedContainer> {
    if bytes.len() < FIXED_PREFIX_BYTES {
        return Err(IcerError::Truncated);
    }
    let sentinel = u16::from_be_bytes([bytes[0], bytes[1]]);
    if sentinel != CONTAINER_SENTINEL {
        return Err(IcerError::invalid(
            "not a plane-container (sentinel mismatch)",
        ));
    }
    let format = format_from_tag(bytes[2])?;
    let declared_n = bytes[3] as usize;
    let expected_n = format.plane_count();
    if declared_n != expected_n {
        return Err(IcerError::invalid(format!(
            "container plane count {declared_n} disagrees with format {format:?} ({expected_n})"
        )));
    }

    let table_end = FIXED_PREFIX_BYTES
        .checked_add(4 * declared_n)
        .ok_or(IcerError::Truncated)?;
    if bytes.len() < table_end {
        return Err(IcerError::Truncated);
    }

    let mut lengths = Vec::with_capacity(declared_n);
    for i in 0..declared_n {
        let off = FIXED_PREFIX_BYTES + 4 * i;
        let len = u32::from_be_bytes([bytes[off], bytes[off + 1], bytes[off + 2], bytes[off + 3]])
            as usize;
        lengths.push(len);
    }

    let mut plane_ranges = Vec::with_capacity(declared_n);
    let mut cursor = table_end;
    for (i, &len) in lengths.iter().enumerate() {
        let end = cursor
            .checked_add(len)
            .ok_or_else(|| IcerError::invalid(format!("plane {i} length overflow")))?;
        if end > bytes.len() {
            return Err(IcerError::Truncated);
        }
        plane_ranges.push((cursor, end));
        cursor = end;
    }

    Ok(ParsedContainer {
        format,
        plane_ranges,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sentinel_never_collides_with_single_plane() {
        // A single-plane stream begins with a non-zero sync prefix; the
        // container sentinel is 0x0000, so the dispatch is unambiguous.
        let single = [0xAC, 0xED, 0x00, 0x00];
        assert!(!is_container(&single));
        let container = [0x00, 0x00, 0x01, 0x03];
        assert!(is_container(&container));
    }

    #[test]
    fn roundtrip_three_planes() {
        let planes = vec![vec![1u8, 2, 3], vec![4u8, 5], vec![6u8, 7, 8, 9]];
        let framed = encode_container(IcerPixelFormat::Yuv444P, &planes).unwrap();
        assert!(is_container(&framed));
        let parsed = parse_container(&framed).unwrap();
        assert_eq!(parsed.format, IcerPixelFormat::Yuv444P);
        assert_eq!(parsed.plane_ranges.len(), 3);
        for (i, original) in planes.iter().enumerate() {
            assert_eq!(parsed.plane_bytes(&framed, i), original.as_slice());
        }
    }

    #[test]
    fn wrong_plane_count_rejected() {
        let planes = vec![vec![1u8], vec![2u8]];
        let err = encode_container(IcerPixelFormat::Yuv444P, &planes).unwrap_err();
        assert!(matches!(err, IcerError::InvalidData(_)));
    }

    #[test]
    fn truncated_substream_rejected() {
        let planes = vec![vec![1u8, 2, 3], vec![4u8, 5], vec![6u8, 7, 8, 9]];
        let mut framed = encode_container(IcerPixelFormat::Yuv444P, &planes).unwrap();
        framed.truncate(framed.len() - 2);
        assert!(matches!(
            parse_container(&framed),
            Err(IcerError::Truncated)
        ));
    }

    #[test]
    fn bad_format_tag_rejected() {
        let mut framed = [0x00u8, 0x00, 0x07, 0x03, 0, 0, 0, 0].to_vec();
        framed.extend_from_slice(&[0; 3]); // bogus body
        assert!(matches!(
            parse_container(&framed),
            Err(IcerError::InvalidData(_))
        ));
    }
}
