//! High-level decoder entry points.
//!
//! Round 1 surface:
//!
//!   * [`parse_icer_metadata`] — parse the framing of every segment
//!     in the input, returning per-segment header info + packet byte
//!     ranges. Does not run pixels through the entropy coder.
//!   * [`decode_uncompressed_icer`] — full pixel decode for the
//!     "uncompressed" path (IPN 42-155 §III.D), where the encoder
//!     bypassed entropy coding and the segment body is raw 8-bit
//!     pixels in scan order. This is the path Mars rover ICER
//!     deployments use as a fallback when the arithmetic coder would
//!     have *expanded* the segment instead of compressing it.
//!
//! Full entropy-decode for compressed segments is round-2 work — see
//! the README for what's deferred.

use crate::error::{IcerError, Result};
use crate::header::{walk_segment, SegmentHeader, WalkedSegment};
use crate::image::{IcerImage, IcerPixelFormat, IcerPlane};

/// Per-segment metadata returned by [`parse_icer_metadata`].
#[derive(Debug, Clone)]
pub struct SegmentMetadata {
    pub header: SegmentHeader,
    /// Number of packets the segment contains.
    pub packet_count: usize,
    /// Byte offset of the segment's first byte (the sync prefix)
    /// relative to the original input buffer.
    pub offset: usize,
    /// Byte length of the segment including its 12-byte header.
    pub byte_length: usize,
}

/// Whole-stream metadata report.
#[derive(Debug, Clone)]
pub struct IcerMetadata {
    pub segments: Vec<SegmentMetadata>,
}

/// Walk every segment in `bytes` and return per-segment metadata.
/// Does not allocate or produce any decoded pixels — useful for a
/// quick `probe()` style content sniff and for callers that want to
/// route only the "uncompressed" segments through this crate while
/// shipping the entropy-coded ones to a different decoder.
pub fn parse_icer_metadata(bytes: &[u8]) -> Result<IcerMetadata> {
    let mut segments = Vec::new();
    let mut cursor = 0;
    while cursor < bytes.len() {
        let walked = walk_segment(&bytes[cursor..])?;
        let byte_length = walked.consumed;
        segments.push(SegmentMetadata {
            header: walked.header,
            packet_count: walked.packets.len(),
            offset: cursor,
            byte_length,
        });
        cursor += byte_length;
    }
    Ok(IcerMetadata { segments })
}

/// Public entry point matching the "round 1 minimum" from the
/// dispatch brief: parse + reconstruct any segment whose
/// `uncompressed` flag is set. Compressed segments still parse
/// correctly (their headers are walked) but their pixels reconstruct
/// to zero — the entropy decoder lands in round 2.
///
/// On a single-segment, uncompressed input this returns a fully
/// decoded image. On a mixed input it returns the image with
/// compressed regions left at zero.
pub fn parse_icer(bytes: &[u8]) -> Result<IcerImage> {
    if bytes.is_empty() {
        return Err(IcerError::Truncated);
    }
    let walked = walk_segment(bytes)?;
    if walked.header.segment_index != 0 {
        return Err(IcerError::Unsupported(format!(
            "round 1 only supports segment_index = 0; got {}",
            walked.header.segment_index
        )));
    }
    if walked.header.uncompressed {
        decode_uncompressed_icer(&walked)
    } else {
        // Compressed path is round-2; for now return an image-shaped
        // zeros buffer so callers can still inspect geometry.
        Ok(IcerImage::zeros(
            walked.header.width as u32,
            walked.header.height as u32,
            IcerPixelFormat::Gray8,
        ))
    }
}

/// Decode the IPN 42-155 §III.D "uncompressed" path. Each packet's
/// body is treated as raw 8-bit luma in scan order. Round 1 only
/// implements the single-plane Gray8 case.
pub fn decode_uncompressed_icer(walked: &WalkedSegment<'_>) -> Result<IcerImage> {
    if !walked.header.uncompressed {
        return Err(IcerError::invalid(
            "decode_uncompressed_icer called on compressed segment",
        ));
    }
    let w = walked.header.width as usize;
    let h = walked.header.height as usize;
    let mut img = IcerImage::zeros(w as u32, h as u32, IcerPixelFormat::Gray8);
    // Concatenate every packet body, then copy at most w*h bytes
    // into the image plane.
    let mut concat: Vec<u8> = Vec::with_capacity(w * h);
    for p in &walked.packets {
        concat.extend_from_slice(p.body);
        if concat.len() >= w * h {
            break;
        }
    }
    if concat.len() < w * h {
        return Err(IcerError::Truncated);
    }
    let plane: &mut IcerPlane = &mut img.planes[0];
    for y in 0..h {
        let row_dst = &mut plane.data[y * plane.stride..y * plane.stride + w];
        let row_src = &concat[y * w..y * w + w];
        row_dst.copy_from_slice(row_src);
    }
    Ok(img)
}
