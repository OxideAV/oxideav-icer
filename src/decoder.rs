//! High-level decoder entry points.
//!
//! Round-3 surface:
//!
//!   * [`parse_icer_metadata`] -- parse the framing of every segment
//!     in the input, returning per-segment header info + packet byte
//!     ranges. Does not run pixels through the entropy coder.
//!   * [`parse_icer`] -- full pixel decode. Handles single-segment,
//!     multi-segment, uncompressed (IPN 42-155 §III.D), and compressed
//!     (bit-plane scanner + binary arithmetic coder) cases.
//!   * [`decode_uncompressed_icer`] -- explicit entry point for the
//!     uncompressed-only fallback.
//!
//! Multi-packet support: the compressed-segment decoder now processes
//! each packet independently per bit-plane (significance + refinement
//! per IPN 42-155 §IV). Packets can arrive truncated or out of order;
//! missing packets simply skip the corresponding bit-planes.

use crate::bitplane::{decode_bitplanes_multi, EncodedPacket};
use crate::error::{IcerError, Result};
use crate::header::{walk_segment, BitPlanePass, SegmentHeader, WalkedSegment};
use crate::image::{IcerImage, IcerPixelFormat, IcerPlane};
use crate::wavelet_float;

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
/// Does not allocate or produce any decoded pixels.
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

/// Decode the full ICER bytestream into an image.
///
/// Multi-segment inputs are demuxed by stitching each segment's
/// reconstructed strip (`segment_index` ascending) vertically. The
/// per-segment width must agree with every other segment (no
/// arbitrary tiling).
pub fn parse_icer(bytes: &[u8]) -> Result<IcerImage> {
    if bytes.is_empty() {
        return Err(IcerError::Truncated);
    }

    // Walk every segment first so we know the total height + canonical
    // width up-front.
    let mut walked_all: Vec<WalkedSegment<'_>> = Vec::new();
    let mut cursor = 0usize;
    while cursor < bytes.len() {
        let walked = walk_segment(&bytes[cursor..])?;
        cursor += walked.consumed;
        walked_all.push(walked);
    }
    if walked_all.is_empty() {
        return Err(IcerError::Truncated);
    }

    // Sort by segment_index so out-of-order delivery still composes.
    walked_all.sort_by_key(|w| w.header.segment_index);

    // Verify width agreement + monotonic-by-1 segment indexing.
    let canonical_width = walked_all[0].header.width as usize;
    let mut total_height = 0usize;
    for (expect_idx, w) in walked_all.iter().enumerate() {
        if w.header.width as usize != canonical_width {
            return Err(IcerError::Unsupported(format!(
                "multi-segment width mismatch: segment {} is {}, expected {}",
                w.header.segment_index, w.header.width, canonical_width
            )));
        }
        if w.header.segment_index as usize != expect_idx {
            return Err(IcerError::invalid(format!(
                "non-contiguous segment indices: got {} at position {}",
                w.header.segment_index, expect_idx
            )));
        }
        total_height = total_height
            .checked_add(w.header.height as usize)
            .ok_or_else(|| IcerError::invalid("multi-segment height overflow"))?;
    }
    if total_height > u32::MAX as usize {
        return Err(IcerError::invalid("multi-segment height overflow"));
    }

    let mut img = IcerImage::zeros(
        canonical_width as u32,
        total_height as u32,
        IcerPixelFormat::Gray8,
    );
    let mut y_cursor = 0usize;
    for walked in &walked_all {
        let strip_h = walked.header.height as usize;
        decode_segment_into(walked, &mut img.planes[0], y_cursor, canonical_width)?;
        y_cursor += strip_h;
    }
    Ok(img)
}

fn decode_segment_into(
    walked: &WalkedSegment<'_>,
    plane: &mut IcerPlane,
    y_offset: usize,
    canonical_width: usize,
) -> Result<()> {
    let strip_h = walked.header.height as usize;
    if walked.header.uncompressed {
        // A zero-packet (or zero-body) uncompressed segment is a
        // ROI-priority placeholder (round 6): no pixel data shipped,
        // strip is reconstructed as all-128 (level-shifted zero).
        if walked.packets.is_empty() {
            for y in 0..strip_h {
                let dst = &mut plane.data[(y_offset + y) * plane.stride
                    ..(y_offset + y) * plane.stride + canonical_width];
                dst.fill(128);
            }
            return Ok(());
        }
        // Concatenate every packet body, then copy at most w*h bytes.
        let mut concat: Vec<u8> = Vec::with_capacity(canonical_width * strip_h);
        for p in &walked.packets {
            concat.extend_from_slice(p.body);
            if concat.len() >= canonical_width * strip_h {
                break;
            }
        }
        if concat.len() < canonical_width * strip_h {
            return Err(IcerError::Truncated);
        }
        for y in 0..strip_h {
            let dst = &mut plane.data
                [(y_offset + y) * plane.stride..(y_offset + y) * plane.stride + canonical_width];
            let src = &concat[y * canonical_width..y * canonical_width + canonical_width];
            dst.copy_from_slice(src);
        }
        Ok(())
    } else {
        decode_compressed_segment_into(walked, plane, y_offset, canonical_width, strip_h)
    }
}

fn decode_compressed_segment_into(
    walked: &WalkedSegment<'_>,
    plane: &mut IcerPlane,
    y_offset: usize,
    width: usize,
    height: usize,
) -> Result<()> {
    let q = walked.header.bit_plane_count;
    let levels = walked.header.decomp_levels;

    // A zero-packet compressed segment is valid: it means the encoder
    // stopped before emitting any bit-plane data (e.g. due to a very
    // tight byte budget). Reconstruct as all-zero coefficients — after
    // the inverse DWT and level-shift this yields all-128 pixels.
    let mut coeffs = if walked.packets.is_empty() {
        vec![0i32; width * height]
    } else {
        // Reconstruct the EncodedPacket list from the walked packet
        // headers. Each WalkedPacket's header has bit_plane + pass
        // fields that map directly to EncodedPacket's bit_plane +
        // is_significance.
        let encoded_packets: Vec<EncodedPacket> = walked
            .packets
            .iter()
            .map(|wp| EncodedPacket {
                bit_plane: wp.header.bit_plane,
                is_significance: matches!(wp.header.pass, BitPlanePass::Significance),
                body: wp.body.to_vec(),
            })
            .collect();
        decode_bitplanes_multi(&encoded_packets, width, height, q)?
    };
    wavelet_float::inverse_2d(&mut coeffs, width, height, levels, walked.header.filter)?;
    // Inverse level-shift + clamp to 0..=255.
    for y in 0..height {
        let dst =
            &mut plane.data[(y_offset + y) * plane.stride..(y_offset + y) * plane.stride + width];
        for x in 0..width {
            let v = coeffs[y * width + x] + 128;
            dst[x] = v.clamp(0, 255) as u8;
        }
    }
    Ok(())
}

/// Decode the IPN 42-155 §III.D "uncompressed" path explicitly. The
/// generic [`parse_icer`] entry point also handles this case, but the
/// dedicated function is kept for callers that want to assert the
/// uncompressed-only invariant.
pub fn decode_uncompressed_icer(walked: &WalkedSegment<'_>) -> Result<IcerImage> {
    if !walked.header.uncompressed {
        return Err(IcerError::invalid(
            "decode_uncompressed_icer called on compressed segment",
        ));
    }
    let w = walked.header.width as usize;
    let h = walked.header.height as usize;
    let mut img = IcerImage::zeros(w as u32, h as u32, IcerPixelFormat::Gray8);
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
