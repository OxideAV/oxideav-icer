//! Encoder entry point.
//!
//! Round 1 only emits the IPN 42-155 §III.D "uncompressed" path:
//! the encoder bypasses the entropy stage entirely and packages raw
//! 8-bit pixel data inside a standard segment + packet framing. This
//! mirrors what real ICER deployments do when the entropy stage
//! would expand the data — see IPN 42-155 §III.D.
//!
//! Lossless 5/3 compressed encoding is round-2 work: the wavelet
//! transform module already implements forward 5/3 lifting, but the
//! bit-plane coder's significance / refinement pass orchestration
//! plus the placeholder probability estimator need replacement
//! before the output is round-trip-safe.

use crate::error::{IcerError, Result};
use crate::header::{BitPlanePass, PacketHeader, SegmentHeader, WaveletFilter};
use crate::image::{IcerImage, IcerPixelFormat};

/// Encoder options. Round 1 surface is intentionally tiny — the
/// caller can pick a sync prefix value (deployment-specific per
/// IPN 42-155 §IV) and which 5/3 filter id to record. The encoder
/// will round-2 grow `wavelet_levels`, `bit_plane_count`,
/// `target_byte_budget` etc.
#[derive(Debug, Clone, Copy)]
pub struct EncodeOptions {
    pub sync_prefix: u16,
    pub filter: WaveletFilter,
    pub wavelet_levels: u8,
    pub bit_plane_count: u8,
}

impl Default for EncodeOptions {
    fn default() -> Self {
        Self {
            // IPN 42-155 §IV does not pin the sync prefix value; pick
            // a non-zero default the round-trip tests can verify.
            sync_prefix: 0xACED,
            filter: WaveletFilter::Reversible53,
            wavelet_levels: 3,
            bit_plane_count: 8,
        }
    }
}

/// Encode `image` as a single uncompressed ICER segment. Returns the
/// complete on-the-wire byte stream (segment header + one packet
/// containing the raw pixels).
///
/// Only `IcerPixelFormat::Gray8` is accepted in round 1.
pub fn encode_icer(image: &IcerImage, opts: &EncodeOptions) -> Result<Vec<u8>> {
    if image.pixel_format != IcerPixelFormat::Gray8 {
        return Err(IcerError::Unsupported(
            "round 1 encoder only supports Gray8".into(),
        ));
    }
    let w = image.width as usize;
    let h = image.height as usize;
    let plane = image
        .planes
        .first()
        .ok_or_else(|| IcerError::invalid("image has no planes"))?;

    // Pack raw pixel data as a single packet body.
    let body_len = w * h;
    if body_len > u16::MAX as usize {
        return Err(IcerError::Unsupported(format!(
            "round 1 encoder limits a single segment to {} pixels (got {})",
            u16::MAX,
            body_len
        )));
    }
    let mut body = Vec::with_capacity(body_len);
    for y in 0..h {
        let row = &plane.data[y * plane.stride..y * plane.stride + w];
        body.extend_from_slice(row);
    }

    let packet = PacketHeader {
        bit_plane: 0,
        pass: BitPlanePass::Cleanup,
        body_length: body_len as u16,
    };
    let packet_bytes = packet.encode();
    let segment_length = packet_bytes.len() + body_len;
    if segment_length > u16::MAX as usize {
        return Err(IcerError::Unsupported(format!(
            "segment length {segment_length} exceeds u16 limit"
        )));
    }
    let segment = SegmentHeader {
        sync_prefix: opts.sync_prefix,
        filter: opts.filter,
        decomp_levels: opts.wavelet_levels.clamp(1, 6),
        uncompressed: true,
        width: image.width as u16,
        height: image.height as u16,
        bit_plane_count: opts.bit_plane_count.clamp(1, 32),
        segment_length: segment_length as u16,
        segment_index: 0,
    };

    let mut out = Vec::with_capacity(SegmentHeader::ENCODED_BYTES + segment_length);
    out.extend_from_slice(&segment.encode());
    out.extend_from_slice(&packet_bytes);
    out.extend_from_slice(&body);
    Ok(out)
}
