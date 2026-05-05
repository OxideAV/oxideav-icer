//! Encoder entry points.
//!
//! Round-3 surface:
//!
//!   * **Uncompressed path** (IPN 42-155 §III.D) -- bypass the entropy
//!     stage and ship raw 8-bit pixels in a single packet. This is
//!     the fallback Mars-rover deployments use when the entropy
//!     coder would *expand* the payload.
//!   * **Compressed path** -- wavelet transform (filter `Q` integer
//!     5/3 by default; float filters A-G also accepted) followed by
//!     the stripe-ordered bit-plane scanner in [`crate::bitplane`]
//!     feeding the binary arithmetic coder. Self-roundtrips with the
//!     matching decoder.
//!   * **Multi-packet ordering** -- the compressed path now emits one
//!     packet pair per bit-plane (significance + refinement) per IPN
//!     42-155 §IV. Truncated streams reconstruct at lower quality.
//!   * **Multi-segment** -- large images split into `segment_count`
//!     row-strip segments, each carrying an independently-decodable
//!     coefficient buffer per IPN 42-155 §III.E.
//!   * **Filter G** -- Le Gall 5/3 float variant; wired through both
//!     encode and decode dispatch paths.

use crate::bitplane::{encode_bitplanes, select_bit_plane_count, BitPlaneInput};
use crate::error::{IcerError, Result};
use crate::header::{BitPlanePass, PacketHeader, SegmentHeader, WaveletFilter};
use crate::image::{IcerImage, IcerPixelFormat, IcerPlane};
use crate::wavelet_float;

/// Encoder options.
#[derive(Debug, Clone, Copy)]
pub struct EncodeOptions {
    pub sync_prefix: u16,
    pub filter: WaveletFilter,
    pub wavelet_levels: u8,
    pub bit_plane_count: u8,
    /// Force the uncompressed-segment path (IPN 42-155 §III.D). When
    /// `false` the encoder runs the wavelet + bit-plane pipeline.
    pub uncompressed: bool,
    /// Number of segments to split the image into. `1` means a
    /// single segment. Larger values split the image into
    /// `segment_count` horizontal strips per IPN 42-155 §III.E.
    pub segment_count: u16,
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
            // Default to uncompressed so the round-1 self-roundtrip
            // covers the baseline. Compressed mode is opt-in via
            // EncodeOptions::compressed().
            uncompressed: true,
            segment_count: 1,
        }
    }
}

impl EncodeOptions {
    /// Convenience constructor: compressed-mode encoder with default
    /// filter `Q` + 3 dyadic levels.
    pub fn compressed() -> Self {
        Self {
            uncompressed: false,
            ..Self::default()
        }
    }
}

/// Encode `image` into the on-the-wire ICER byte stream. Single or
/// multiple segments depending on `opts.segment_count`.
pub fn encode_icer(image: &IcerImage, opts: &EncodeOptions) -> Result<Vec<u8>> {
    if image.pixel_format != IcerPixelFormat::Gray8 {
        return Err(IcerError::Unsupported("encoder only supports Gray8".into()));
    }
    let plane = image
        .planes
        .first()
        .ok_or_else(|| IcerError::invalid("image has no planes"))?;
    let w = image.width as usize;
    let h = image.height as usize;
    if w == 0 || h == 0 {
        return Err(IcerError::invalid("image has zero dimension"));
    }

    let segment_count = opts.segment_count.max(1);
    let levels = opts.wavelet_levels.clamp(1, 6);

    if segment_count == 1 {
        return encode_one_segment(plane, w, 0, h, 0, opts, levels);
    }

    // Multi-segment: split into `segment_count` row strips. Each strip
    // is at least 2 rows so the wavelet step has room. Strips may be
    // unequal -- the trailing strip absorbs the remainder.
    let strip_h = h.div_ceil(segment_count as usize);
    if strip_h < 2 {
        return Err(IcerError::Unsupported(format!(
            "segment_count {segment_count} too high for image height {h} (minimum strip 2 rows)"
        )));
    }

    let mut out = Vec::new();
    let mut y_cursor = 0usize;
    let mut idx = 0u16;
    while y_cursor < h {
        let this_h = (h - y_cursor).min(strip_h);
        let bytes = encode_one_segment(plane, w, y_cursor, this_h, idx, opts, levels)?;
        out.extend_from_slice(&bytes);
        y_cursor += this_h;
        idx += 1;
    }
    Ok(out)
}

#[allow(clippy::too_many_arguments)]
fn encode_one_segment(
    plane: &IcerPlane,
    img_w: usize,
    y_start: usize,
    strip_h: usize,
    segment_index: u16,
    opts: &EncodeOptions,
    levels: u8,
) -> Result<Vec<u8>> {
    if opts.uncompressed {
        encode_one_segment_uncompressed(plane, img_w, y_start, strip_h, segment_index, opts)
    } else {
        encode_one_segment_compressed(plane, img_w, y_start, strip_h, segment_index, opts, levels)
    }
}

fn encode_one_segment_uncompressed(
    plane: &IcerPlane,
    img_w: usize,
    y_start: usize,
    strip_h: usize,
    segment_index: u16,
    opts: &EncodeOptions,
) -> Result<Vec<u8>> {
    let body_len = img_w * strip_h;
    if body_len > u16::MAX as usize {
        return Err(IcerError::Unsupported(format!(
            "uncompressed segment limited to {} pixels (got {})",
            u16::MAX,
            body_len
        )));
    }
    let mut body = Vec::with_capacity(body_len);
    for y in 0..strip_h {
        let src_y = y_start + y;
        let row = &plane.data[src_y * plane.stride..src_y * plane.stride + img_w];
        body.extend_from_slice(row);
    }
    let packet = PacketHeader {
        bit_plane: 0,
        pass: BitPlanePass::Cleanup,
        body_length: body_len as u16,
    };
    finish_segment(&packet, &body, segment_index, img_w, strip_h, opts, true)
}

#[allow(clippy::too_many_arguments)]
fn encode_one_segment_compressed(
    plane: &IcerPlane,
    img_w: usize,
    y_start: usize,
    strip_h: usize,
    segment_index: u16,
    opts: &EncodeOptions,
    levels: u8,
) -> Result<Vec<u8>> {
    if img_w < 2 || strip_h < 2 {
        return Err(IcerError::Unsupported(format!(
            "compressed segment requires width >= 2 and height >= 2; got {img_w}x{strip_h}"
        )));
    }
    // Build signed coefficient buffer (shift by 128 so the centre of
    // the unsigned 8-bit range maps to 0 -- IPN 42-155 §III.A
    // "level-shift").
    let mut coeffs: Vec<i32> = Vec::with_capacity(img_w * strip_h);
    for y in 0..strip_h {
        let src_y = y_start + y;
        let row = &plane.data[src_y * plane.stride..src_y * plane.stride + img_w];
        for &px in row {
            coeffs.push(px as i32 - 128);
        }
    }
    // Forward DWT (filter-aware dispatch).
    wavelet_float::forward_2d(&mut coeffs, img_w, strip_h, levels, opts.filter)?;
    // Pick bit-plane count to fit the largest |coeff|, but never less
    // than the caller-requested floor.
    let needed = select_bit_plane_count(&coeffs);
    let q = needed.max(opts.bit_plane_count.min(31)).min(31);

    // Encode as per-bit-plane packets (IPN 42-155 §IV multi-packet ordering).
    let packets = encode_bitplanes(&BitPlaneInput {
        coeffs: &coeffs,
        width: img_w,
        height: strip_h,
        q,
    })?;

    // Serialise the per-bit-plane packets and build the segment.
    // The segment body is the concatenation of all packet headers +
    // bodies in priority order (MSB-down, significance before
    // refinement per §IV).
    let mut body: Vec<u8> = Vec::new();
    for pkt in &packets {
        let pass = if pkt.is_significance {
            BitPlanePass::Significance
        } else {
            BitPlanePass::Refinement
        };
        let ph = PacketHeader {
            bit_plane: pkt.bit_plane,
            pass,
            body_length: pkt.body.len() as u16,
        };
        // Guard against individual packet body overflowing u16.
        if pkt.body.len() > u16::MAX as usize {
            return Err(IcerError::Unsupported(format!(
                "packet body {} exceeds u16 limit",
                pkt.body.len()
            )));
        }
        body.extend_from_slice(&ph.encode());
        body.extend_from_slice(&pkt.body);
    }

    if body.len() > u16::MAX as usize {
        return Err(IcerError::Unsupported(format!(
            "compressed segment body {} exceeds u16 limit",
            body.len()
        )));
    }

    // The segment header segment_length covers the multi-packet body.
    // We need a dummy PacketHeader for the finish_segment call -- but
    // finish_segment's API assumes a single packet. For multi-packet
    // segments we embed all the packet headers+bodies directly into
    // `body` above, so we emit the SegmentHeader manually here.
    let mut opts_copy = *opts;
    opts_copy.bit_plane_count = q;
    emit_segment_header_and_body(&body, segment_index, img_w, strip_h, &opts_copy, false)
}

/// Emit a segment header followed by a pre-assembled body (which may
/// contain multiple packet headers + bodies) for the compressed path.
fn emit_segment_header_and_body(
    body: &[u8],
    segment_index: u16,
    width: usize,
    height: usize,
    opts: &EncodeOptions,
    uncompressed: bool,
) -> Result<Vec<u8>> {
    let segment_length = body.len();
    if segment_length > u16::MAX as usize {
        return Err(IcerError::Unsupported(format!(
            "segment length {segment_length} exceeds u16 limit"
        )));
    }
    let segment = SegmentHeader {
        sync_prefix: opts.sync_prefix,
        filter: opts.filter,
        decomp_levels: opts.wavelet_levels.clamp(1, 6),
        uncompressed,
        width: width as u16,
        height: height as u16,
        bit_plane_count: opts.bit_plane_count.clamp(1, 32),
        segment_length: segment_length as u16,
        segment_index,
    };
    let mut out = Vec::with_capacity(SegmentHeader::ENCODED_BYTES + segment_length);
    out.extend_from_slice(&segment.encode());
    out.extend_from_slice(body);
    Ok(out)
}

#[allow(clippy::too_many_arguments)]
fn finish_segment(
    packet: &PacketHeader,
    body: &[u8],
    segment_index: u16,
    width: usize,
    height: usize,
    opts: &EncodeOptions,
    uncompressed: bool,
) -> Result<Vec<u8>> {
    let packet_bytes = packet.encode();
    let segment_length = packet_bytes.len() + body.len();
    if segment_length > u16::MAX as usize {
        return Err(IcerError::Unsupported(format!(
            "segment length {segment_length} exceeds u16 limit"
        )));
    }
    let segment = SegmentHeader {
        sync_prefix: opts.sync_prefix,
        filter: opts.filter,
        decomp_levels: opts.wavelet_levels.clamp(1, 6),
        uncompressed,
        width: width as u16,
        height: height as u16,
        bit_plane_count: opts.bit_plane_count.clamp(1, 32),
        segment_length: segment_length as u16,
        segment_index,
    };
    let mut out = Vec::with_capacity(SegmentHeader::ENCODED_BYTES + segment_length);
    out.extend_from_slice(&segment.encode());
    out.extend_from_slice(&packet_bytes);
    out.extend_from_slice(body);
    Ok(out)
}
