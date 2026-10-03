//! High-level 2-D decoder entry points — the one implementation behind
//! the contract fronts in [`crate::api`] (`info` / `decode` /
//! `decode_with`) and the depth APIs kept here:
//!
//!   * `walk_stream` (crate-internal) -- header-only walk of every
//!     segment, container-aware; what [`crate::info`] reports. Does not
//!     run pixels through the entropy coder and allocates no plane.
//!   * `decode_image` (crate-internal) -- full pixel decode. Handles
//!     single-segment, multi-segment, uncompressed (IPN 42-155 §III.D),
//!     and compressed (bit-plane scanner + binary arithmetic coder)
//!     cases, bare or behind the plane container.
//!   * [`parse_icer_lenient`] / [`parse_icer_lenient_with`] -- the
//!     loss-tolerant decode with a presence report ([`LenientDecode`]).
//!   * [`decode_uncompressed_icer`] -- explicit entry point for the
//!     uncompressed-only fallback.
//!   * the pre-contract names ([`parse_icer`], [`parse_icer_with_limits`],
//!     [`parse_icer_metadata`], [`parse_icer_metadata_with_limits`],
//!     [`parse_icer_lenient_with_limits`]) stay for one release as
//!     deprecated wrappers.
//!
//! Multi-packet support: the compressed-segment decoder processes each
//! packet independently per bit-plane (significance + refinement per
//! IPN 42-155 §IV). Packets can arrive truncated or out of order;
//! missing packets simply skip the corresponding bit-planes.
//!
//! ## Decode-side resource limits
//!
//! The wire format admits arbitrary `(width, height)` pairs in the
//! 12-byte segment header (`u16 * u16`), which means a single tiny
//! header can request up to ~4 GB of decoder allocation per plane —
//! a DoS surface flagged by the cargo-fuzz harness. [`DecodeOptions`]
//! caps the per-segment and per-image pixel counts and the decoded
//! bytes the decoder will agree to materialise; every cap is checked
//! on the framing before any plane or coefficient buffer exists and
//! before any inverse DWT runs, failing with
//! [`IcerError::LimitExceeded`].

use crate::bitplane::{EncodedPacket, ScanFilter};
use crate::error::{IcerError, Result};
use crate::header::{walk_segment, BitPlanePass, SegmentHeader, WalkedSegment};
use crate::image::StreamKind;
use crate::image::{IcerImage, IcerPixelFormat, Plane};
#[allow(deprecated)]
use crate::options::DecodeLimits;
use crate::options::DecodeOptions;
use crate::wavelet_int;

/// Pixel-count of a segment, computed in `u64` to side-step any
/// `usize * usize` overflow risk on 32-bit targets. Always finite for
/// in-range `SegmentHeader::{width, height}` (both `u16`).
#[inline]
fn segment_pixels(header: &SegmentHeader) -> u64 {
    header.width as u64 * header.height as u64
}

/// The single-plane in-memory format for a given sample depth: the
/// historical [`IcerPixelFormat::Gray8`] at depth 8, the deep-gray
/// format above (depth is carried by the plane-container framing — the
/// 12-byte segment header has no depth field).
fn gray_format(depth: u8) -> IcerPixelFormat {
    if depth > 8 {
        IcerPixelFormat::Gray16Le
    } else {
        IcerPixelFormat::Gray8
    }
}

/// A zero-filled single-plane gray image of the given depth
/// (`Gray8` at depth 8, `Gray16Le` with `bit_depth = depth` deeper).
fn zeros_gray(width: u32, height: u32, depth: u8) -> IcerImage {
    let mut img = IcerImage::zeros(width, height, gray_format(depth));
    img.bit_depth = depth;
    img
}

/// Bytes per stored sample for a given depth (1, or 2 little-endian).
fn sample_bytes(depth: u8) -> usize {
    if depth > 8 {
        2
    } else {
        1
    }
}

/// Store one already-clamped pixel value into a plane row.
#[inline]
fn store_px(row: &mut [u8], x: usize, sb: usize, v: i32) {
    if sb == 2 {
        row[x * 2..x * 2 + 2].copy_from_slice(&(v as u16).to_le_bytes());
    } else {
        row[x] = v as u8;
    }
}

/// Fill rows `y_offset..y_offset + rows` of `plane` with the §III.A
/// level-shift midpoint `2^(depth-1)` (the placeholder / missing-strip
/// reconstruction value; 128 on the historical 8-bit path).
fn fill_mid_rows(plane: &mut Plane, y_offset: usize, rows: usize, width: usize, depth: u8) {
    let sb = sample_bytes(depth);
    let mid = 1i32 << (depth - 1);
    for y in 0..rows {
        let row = &mut plane.data[(y_offset + y) * plane.stride..][..width * sb];
        if sb == 1 {
            row.fill(mid as u8);
        } else {
            for x in 0..width {
                store_px(row, x, sb, mid);
            }
        }
    }
}

/// Per-segment framing record — one per 2-D segment, as reported by
/// [`crate::ImageInfo::segments`].
#[derive(Debug, Clone)]
#[non_exhaustive]
pub struct SegmentMetadata {
    /// The parsed 12-byte segment header.
    pub header: SegmentHeader,
    /// Number of packets the segment contains.
    pub packet_count: usize,
    /// Byte offset of the segment's first byte (the sync prefix)
    /// relative to the original input buffer.
    pub offset: usize,
    /// Byte length of the segment including its 12-byte header.
    pub byte_length: usize,
}

impl SegmentMetadata {
    /// Assemble a record.
    pub fn new(
        header: SegmentHeader,
        packet_count: usize,
        offset: usize,
        byte_length: usize,
    ) -> Self {
        Self {
            header,
            packet_count,
            offset,
            byte_length,
        }
    }
}

/// Whole-stream metadata report of the deprecated
/// [`parse_icer_metadata`]; the same records are
/// [`crate::ImageInfo::segments`].
#[deprecated(note = "use oxideav_icer::info(..).segments (IMAGE_CRATE_API)")]
#[derive(Debug, Clone)]
pub struct IcerMetadata {
    /// Every segment in stream order (plane by plane for a container).
    pub segments: Vec<SegmentMetadata>,
}

/// Header-only description of a 2-D stream (bare segments or the
/// plane container): the native layout, the stitched geometry and the
/// per-segment records. Produced by [`walk_stream`] without decoding a
/// pixel or allocating a plane.
#[derive(Debug, Clone)]
pub(crate) struct StreamLayout {
    pub format: IcerPixelFormat,
    pub bit_depth: u8,
    pub width: u32,
    pub height: u32,
    pub kind: StreamKind,
    pub segments: Vec<SegmentMetadata>,
}

/// Header-only walk of one single-plane segment stream: every segment's
/// framing record, the canonical width, the stitched height (sum of
/// the row strips; the declared image for §V.B transform-domain
/// streams) and the image pixel count — each segment and the running
/// total checked against `opts` as they are met.
fn walk_single_plane(
    bytes: &[u8],
    opts: &DecodeOptions,
) -> Result<(u32, u32, u64, Vec<SegmentMetadata>)> {
    if bytes.is_empty() {
        return Err(IcerError::Truncated);
    }
    let mut segments = Vec::new();
    let mut total_pixels: u64 = 0;
    let mut cursor = 0;
    let mut transform_counted = false;
    let mut width: u32 = 0;
    let mut height: u64 = 0;
    while cursor < bytes.len() {
        let walked = walk_segment(&bytes[cursor..])?;
        opts.check_segment(
            walked.header.segment_index,
            walked.header.width as u32,
            walked.header.height as u32,
        )?;
        if segments.is_empty() {
            width = walked.header.width as u32;
        } else if walked.header.width as u32 != width {
            return Err(IcerError::Unsupported(format!(
                "multi-segment width mismatch: segment {} is {}, expected {}",
                walked.header.segment_index, walked.header.width, width
            )));
        }
        // §V.B transform-domain segments all declare the full image
        // dimensions; count the image once against the total cap.
        let count_pixels = if walked.header.transform_segmented {
            let first = !transform_counted;
            transform_counted = true;
            first
        } else {
            true
        };
        if count_pixels {
            total_pixels = total_pixels
                .checked_add(segment_pixels(&walked.header))
                .ok_or_else(|| IcerError::invalid("multi-segment pixel-count overflow"))?;
            height += walked.header.height as u64;
        }
        opts.check_total_pixels(total_pixels, "multi-segment image")?;
        let byte_length = walked.consumed;
        segments.push(SegmentMetadata::new(
            walked.header,
            walked.packets.len(),
            cursor,
            byte_length,
        ));
        cursor += byte_length;
    }
    if segments.is_empty() {
        return Err(IcerError::Truncated);
    }
    let height =
        u32::try_from(height).map_err(|_| IcerError::invalid("multi-segment height overflow"))?;
    opts.check_width_height(width, height)?;
    Ok((width, height, total_pixels, segments))
}

/// Walk the framing of a whole 2-D stream — bare segments or the plane
/// container — applying every [`DecodeOptions`] cap (per segment,
/// total pixels, decoded bytes, width / height) exactly as the decode
/// path does, but without allocating a plane. Cube streams are not
/// handled here (see [`crate::cube`]).
pub(crate) fn walk_stream(bytes: &[u8], opts: &DecodeOptions) -> Result<StreamLayout> {
    if bytes.is_empty() {
        return Err(IcerError::Truncated);
    }
    if crate::plane_container::is_container(bytes) {
        // Colour / deep container: walk each plane substream and
        // concatenate the per-plane segment records. Segment `offset`
        // values are rebased to the original container buffer so
        // callers see absolute positions.
        let parsed = crate::plane_container::parse_container(bytes)?;
        let mut segments = Vec::new();
        let mut total_pixels: u64 = 0;
        let mut geometry: Option<(u32, u32)> = None;
        for i in 0..parsed.format.plane_count() {
            let (base, _end) = parsed.plane_ranges[i];
            let sub = parsed.plane_bytes(bytes, i);
            let (w, h, px, sub_segments) = walk_single_plane(sub, opts)?;
            match geometry {
                None => geometry = Some((w, h)),
                Some((w0, h0)) if (w0, h0) != (w, h) => {
                    return Err(IcerError::Unsupported(format!(
                        "colour plane {i} geometry {w}x{h} disagrees with plane 0 {w0}x{h0}"
                    )))
                }
                _ => {}
            }
            total_pixels = total_pixels
                .checked_add(px)
                .ok_or_else(|| IcerError::invalid("colour pixel-count overflow"))?;
            opts.check_total_pixels(total_pixels, "colour image")?;
            for mut s in sub_segments {
                s.offset += base;
                segments.push(s);
            }
        }
        let (width, height) = geometry.ok_or(IcerError::Truncated)?;
        opts.check_bytes(total_pixels * parsed.format.sample_bytes() as u64)?;
        return Ok(StreamLayout {
            format: parsed.format,
            bit_depth: parsed.bit_depth,
            width,
            height,
            kind: StreamKind::PlaneContainer,
            segments,
        });
    }
    let (width, height, total_pixels, segments) = walk_single_plane(bytes, opts)?;
    opts.check_bytes(total_pixels)?;
    Ok(StreamLayout {
        format: IcerPixelFormat::Gray8,
        bit_depth: 8,
        width,
        height,
        kind: StreamKind::Segments,
        segments,
    })
}

/// Walk every segment in `bytes` and return per-segment metadata under
/// the default caps. Superseded by [`crate::info`], whose
/// `segments` field carries the same records.
#[deprecated(note = "use oxideav_icer::info (IMAGE_CRATE_API)")]
#[allow(deprecated)]
pub fn parse_icer_metadata(bytes: &[u8]) -> Result<IcerMetadata> {
    Ok(IcerMetadata {
        segments: walk_stream(bytes, &DecodeOptions::default())?.segments,
    })
}

/// [`parse_icer_metadata`] under an explicit cap policy. Superseded by
/// [`crate::info`] (whose limits are [`DecodeOptions::default`]) and
/// the `_with` depth decoders.
#[deprecated(note = "use oxideav_icer::info (IMAGE_CRATE_API)")]
#[allow(deprecated)]
pub fn parse_icer_metadata_with_limits(
    bytes: &[u8],
    limits: &DecodeLimits,
) -> Result<IcerMetadata> {
    Ok(IcerMetadata {
        segments: walk_stream(bytes, &DecodeOptions::from(limits))?.segments,
    })
}

/// Decode the full 2-D ICER bytestream into an image under the default
/// caps. Superseded by [`crate::decode`].
#[deprecated(note = "use oxideav_icer::decode (IMAGE_CRATE_API)")]
pub fn parse_icer(bytes: &[u8]) -> Result<IcerImage> {
    decode_image(bytes, &DecodeOptions::default())
}

/// [`parse_icer`] under an explicit cap policy. Superseded by
/// [`crate::decode_with`].
#[deprecated(note = "use oxideav_icer::decode_with (IMAGE_CRATE_API)")]
#[allow(deprecated)]
pub fn parse_icer_with_limits(bytes: &[u8], limits: &DecodeLimits) -> Result<IcerImage> {
    decode_image(bytes, &DecodeOptions::from(limits))
}

/// Decode a 2-D ICER bytestream (bare segments or the plane container)
/// into an image, rejecting any segment / total geometry that exceeds
/// `opts` before allocating the plane or wavelet coefficient buffers.
///
/// Multi-segment inputs are demuxed by stitching each segment's
/// reconstructed strip (`segment_index` ascending) vertically. The
/// per-segment width must agree with every other segment (no
/// arbitrary tiling). The one implementation behind [`crate::decode`]
/// / [`crate::decode_with`] and the framework `Decoder`.
pub(crate) fn decode_image(bytes: &[u8], opts: &DecodeOptions) -> Result<IcerImage> {
    if bytes.is_empty() {
        return Err(IcerError::Truncated);
    }

    // Colour / deep images are framed as a multi-plane container
    // (leading 0x0000 sentinel — see `crate::plane_container`). A
    // single-plane Gray8 stream never starts with 0x0000, so the
    // dispatch is unambiguous and every historical Gray8 stream falls
    // through to the single-plane path below byte-for-byte unchanged.
    if crate::plane_container::is_container(bytes) {
        return parse_icer_multi_plane(bytes, opts);
    }

    parse_icer_single_plane(bytes, opts, 8)
}

/// Decode a multi-plane (colour) container: each plane substream is a full
/// single-plane ICER bitstream, decoded independently and re-assembled
/// into the declared [`IcerPixelFormat`].
fn parse_icer_multi_plane(bytes: &[u8], opts: &DecodeOptions) -> Result<IcerImage> {
    // Header-only pre-walk: the whole-image caps (total pixels across
    // planes, decoded bytes) are enforced here, before the first
    // plane's coefficient buffer exists.
    walk_stream(bytes, opts)?;
    let parsed = crate::plane_container::parse_container(bytes)?;
    let n = parsed.format.plane_count();
    let depth = parsed.bit_depth;

    let mut plane_images: Vec<IcerImage> = Vec::with_capacity(n);
    for i in 0..n {
        let sub = parsed.plane_bytes(bytes, i);
        plane_images.push(parse_icer_single_plane(sub, opts, depth)?);
    }

    // Every plane must agree on geometry — the container's planes are
    // co-sited (4:4:4) views of one image.
    let w = plane_images[0].width;
    let h = plane_images[0].height;
    for (i, p) in plane_images.iter().enumerate() {
        if p.width != w || p.height != h {
            return Err(IcerError::Unsupported(format!(
                "colour plane {i} geometry {}x{} disagrees with plane 0 {}x{}",
                p.width, p.height, w, h
            )));
        }
    }

    let mut out = IcerImage::zeros(w, h, parsed.format);
    out.bit_depth = parsed.bit_depth;
    for (i, p) in plane_images.into_iter().enumerate() {
        // Each decoded plane image is Gray8 with a single plane; move it
        // into slot `i` of the colour image.
        out.planes[i] = p
            .planes
            .into_iter()
            .next()
            .ok_or_else(|| IcerError::invalid("decoded colour plane has no data"))?;
    }
    Ok(out)
}

/// Decode a single-plane ICER bitstream at the given sample depth (8
/// for a bare historical Gray8 stream; deeper when dispatched from the
/// deep-gray plane container, which owns the depth field). This is the
/// historical `parse_icer` body.
fn parse_icer_single_plane(bytes: &[u8], opts: &DecodeOptions, depth: u8) -> Result<IcerImage> {
    if bytes.is_empty() {
        return Err(IcerError::Truncated);
    }

    // Walk every segment first so we know the total height + canonical
    // width up-front.
    let mut walked_all: Vec<WalkedSegment<'_>> = Vec::new();
    let mut cursor = 0usize;
    let mut total_pixels: u64 = 0;
    let mut transform_counted = false;
    while cursor < bytes.len() {
        let walked = walk_segment(&bytes[cursor..])?;
        opts.check_segment(
            walked.header.segment_index,
            walked.header.width as u32,
            walked.header.height as u32,
        )?;
        // §V.B transform-domain segments each declare the FULL image
        // dimensions, so the image's pixel count enters the total cap
        // once, not once per segment.
        let count_pixels = if walked.header.transform_segmented {
            let first = !transform_counted;
            transform_counted = true;
            first
        } else {
            true
        };
        if count_pixels {
            total_pixels = total_pixels
                .checked_add(segment_pixels(&walked.header))
                .ok_or_else(|| IcerError::invalid("multi-segment pixel-count overflow"))?;
        }
        opts.check_total_pixels(total_pixels, "multi-segment image")?;
        cursor += walked.consumed;
        walked_all.push(walked);
    }
    if walked_all.is_empty() {
        return Err(IcerError::Truncated);
    }

    // Sort by segment_index so out-of-order delivery still composes.
    walked_all.sort_by_key(|w| w.header.segment_index);

    // §V.B transform-domain streams decode through the shared-transform
    // path (strict: every §V.D segment must be present).
    if walked_all.iter().any(|w| w.header.transform_segmented) {
        return decode_transform_domain(&walked_all, true, depth).map(|(img, _, _)| img);
    }

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

    opts.check_width_height(canonical_width as u32, total_height as u32)?;
    opts.check_bytes(total_pixels * sample_bytes(depth) as u64)?;
    let mut img = zeros_gray(canonical_width as u32, total_height as u32, depth);
    let mut y_cursor = 0usize;
    for walked in &walked_all {
        let strip_h = walked.header.height as usize;
        decode_segment_into(walked, &mut img.planes[0], y_cursor, canonical_width, depth)?;
        y_cursor += strip_h;
    }
    Ok(img)
}

/// Decode a §V.B transform-domain segmented stream: every segment codes
/// the coefficients the §V.D LL-partition maps to it; the union decodes
/// into one shared coefficient buffer and a single whole-image inverse
/// DWT reconstructs the pixels (IPN 42-155 §V.B).
///
/// `strict` requires all `total_segments` segments present with
/// contiguous indices (the [`crate::decode`] contract); lenient mode
/// tolerates missing segments — their coefficients stay zero, which
/// reconstructs as a smooth low-detail patch through the shared inverse
/// transform (§V.B error containment; the loss "bleeds" only slightly
/// into adjacent segments via the wavelet support). Returns the image,
/// the per-index presence map, and the missing count.
fn decode_transform_domain(
    walked_all: &[WalkedSegment<'_>],
    strict: bool,
    depth: u8,
) -> Result<(IcerImage, Vec<bool>, usize)> {
    let first = &walked_all[0].header;
    if !first.transform_segmented {
        return Err(IcerError::invalid(
            "mixed §V.B transform-domain and row-strip segments in one stream",
        ));
    }
    let (w, h) = (first.width as usize, first.height as usize);
    let levels = first.decomp_levels;
    let total = first.total_segments as usize;

    // Every segment must agree on the whole-image parameters the §V.D
    // partition is recomputed from (§V.D: image dimensions, stages of
    // decomposition, total number of segments — all in each header).
    for wseg in walked_all {
        let hd = &wseg.header;
        if !hd.transform_segmented {
            return Err(IcerError::invalid(
                "mixed §V.B transform-domain and row-strip segments in one stream",
            ));
        }
        if hd.uncompressed {
            return Err(IcerError::invalid(
                "transform-domain segments carry coefficients, not §III.D raw pixels",
            ));
        }
        if hd.width as usize != w
            || hd.height as usize != h
            || hd.decomp_levels != levels
            || hd.total_segments as usize != total
            || hd.filter != first.filter
        {
            return Err(IcerError::invalid(format!(
                "transform-domain segment {} disagrees on the shared image parameters",
                hd.segment_index
            )));
        }
    }
    // Duplicate indices are a contradiction in either mode.
    for pair in walked_all.windows(2) {
        if pair[0].header.segment_index == pair[1].header.segment_index {
            return Err(IcerError::invalid(format!(
                "duplicate transform-domain segment index {}",
                pair[0].header.segment_index
            )));
        }
    }
    if strict {
        if walked_all.len() != total {
            return Err(IcerError::invalid(format!(
                "transform-domain stream carries {} of {} segments (§V.D)",
                walked_all.len(),
                total
            )));
        }
        for (expect, wseg) in walked_all.iter().enumerate() {
            if wseg.header.segment_index as usize != expect {
                return Err(IcerError::invalid(format!(
                    "non-contiguous transform-domain segment indices: got {} at position {expect}",
                    wseg.header.segment_index
                )));
            }
        }
    }

    // Recompute the §V.D partition (never encoded — §V.D) and decode
    // each present segment's coefficients into the shared buffer.
    let seg_map = crate::partition::coefficient_segment_map(w, h, levels, total)?;
    let (w_ll, h_ll) = crate::partition::ll_dimensions(w, h, levels);
    let rects = crate::partition::partition(w_ll, h_ll, total)?;
    let mut coeffs = vec![0i32; w * h];
    let mut received = vec![false; total];
    for wseg in walked_all {
        let seg_idx = wseg.header.segment_index;
        received[seg_idx as usize] = true;
        if wseg.packets.is_empty() {
            // Budget placeholder: zero coefficients (§V.B containment).
            continue;
        }
        // §VI.A minimum loss, replicated in every packet header.
        let min_loss = wseg.packets[0].header.min_loss;
        let window = rects[seg_idx as usize].image_window(levels, w, h);
        let encoded_packets: Vec<EncodedPacket> = wseg
            .packets
            .iter()
            .map(|wp| EncodedPacket {
                bit_plane: wp.header.bit_plane,
                is_significance: matches!(wp.header.pass, BitPlanePass::Significance),
                body: wp.body.to_vec(),
                delta_distortion: 0.0,
            })
            .collect();
        let kind = if wseg.header.interleaved_entropy {
            crate::entropy::EntropyKind::Interleaved
        } else {
            crate::entropy::EntropyKind::Arithmetic
        };
        let part = if wseg.header.priority_interleaved {
            // §III.A subband-priority interleaving over this §V.B
            // segment (min_loss excludes whole subband bit planes from
            // the schedule; no per-coefficient skip map applies).
            let filter = ScanFilter {
                segment: Some((&seg_map, seg_idx)),
                skip: None,
                window: Some(window),
            };
            crate::bitplane::decode_bitplanes_prioritized(
                &encoded_packets,
                w,
                h,
                wseg.header.bit_plane_count,
                levels,
                kind,
                &filter,
                min_loss,
            )?
        } else {
            let skip_map: Option<Vec<u8>> =
                (min_loss > 0).then(|| crate::priority::min_loss_skip_map(w, h, levels, min_loss));
            let filter = ScanFilter {
                segment: Some((&seg_map, seg_idx)),
                skip: skip_map.as_deref(),
                window: Some(window),
            };
            crate::bitplane::decode_bitplanes_filtered(
                &encoded_packets,
                w,
                h,
                wseg.header.bit_plane_count,
                levels,
                kind,
                &filter,
            )?
        };
        let (wx0, wx1, wy0, wy1) = window;
        for y in wy0..wy1 {
            for x in wx0..wx1 {
                let i = y * w + x;
                coeffs[i] = part[i];
            }
        }
    }
    let missing_count = received.iter().filter(|&&r| !r).count();

    // One shared inverse transform (§V.B: "the inverse wavelet
    // transform combines data from adjacent segments").
    crate::wavelet_int::inverse_2d_dyadic(&mut coeffs, w, h, levels, first.filter);
    let mut img = zeros_gray(w as u32, h as u32, depth);
    let plane = &mut img.planes[0];
    let sb = sample_bytes(depth);
    let mid = 1i32 << (depth - 1);
    let max_v = (1i32 << depth) - 1;
    for y in 0..h {
        let dst = &mut plane.data[y * plane.stride..][..w * sb];
        for x in 0..w {
            let v = coeffs[y * w + x].saturating_add(mid).clamp(0, max_v);
            store_px(dst, x, sb, v);
        }
    }
    Ok((img, received, missing_count))
}

fn decode_segment_into(
    walked: &WalkedSegment<'_>,
    plane: &mut Plane,
    y_offset: usize,
    canonical_width: usize,
    depth: u8,
) -> Result<()> {
    let strip_h = walked.header.height as usize;
    let sb = sample_bytes(depth);
    if walked.header.uncompressed {
        // A zero-packet (or zero-body) uncompressed segment is a
        // ROI-priority placeholder (round 6): no pixel data shipped,
        // strip is reconstructed as the level-shift midpoint (128 at
        // depth 8 — level-shifted zero).
        if walked.packets.is_empty() {
            fill_mid_rows(plane, y_offset, strip_h, canonical_width, depth);
            return Ok(());
        }
        // Concatenate every packet body, then copy at most the strip's
        // raw sample bytes (width * height * sample_bytes).
        let row_bytes = canonical_width * sb;
        let strip_bytes = row_bytes * strip_h;
        let mut concat: Vec<u8> = Vec::with_capacity(strip_bytes);
        for p in &walked.packets {
            concat.extend_from_slice(p.body);
            if concat.len() >= strip_bytes {
                break;
            }
        }
        if concat.len() < strip_bytes {
            return Err(IcerError::Truncated);
        }
        for y in 0..strip_h {
            let dst = &mut plane.data[(y_offset + y) * plane.stride..][..row_bytes];
            let src = &concat[y * row_bytes..(y + 1) * row_bytes];
            dst.copy_from_slice(src);
        }
        Ok(())
    } else {
        decode_compressed_segment_into(walked, plane, y_offset, canonical_width, strip_h, depth)
    }
}

fn decode_compressed_segment_into(
    walked: &WalkedSegment<'_>,
    plane: &mut Plane,
    y_offset: usize,
    width: usize,
    height: usize,
    depth: u8,
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
                // Decoder side does not need the R-D estimate; default to 0.0.
                delta_distortion: 0.0,
            })
            .collect();
        let kind = if walked.header.interleaved_entropy {
            crate::entropy::EntropyKind::Interleaved
        } else {
            crate::entropy::EntropyKind::Arithmetic
        };
        // §VI.A minimum loss, replicated in every packet header: apply
        // the identical per-subband plane exclusion the encoder used
        // (0 on every pre-existing stream = no exclusion).
        let min_loss = walked.packets[0].header.min_loss;
        if walked.header.priority_interleaved {
            // §III.A subband-priority interleaving: replay the identical
            // priority-group schedule (min_loss drops whole subband bit
            // planes from it, so no per-coefficient skip map applies).
            crate::bitplane::decode_bitplanes_prioritized(
                &encoded_packets,
                width,
                height,
                q,
                levels,
                kind,
                &ScanFilter::ALL,
                min_loss,
            )?
        } else {
            let skip_map: Option<Vec<u8>> = (min_loss > 0)
                .then(|| crate::priority::min_loss_skip_map(width, height, levels, min_loss));
            let filter = ScanFilter {
                segment: None,
                skip: skip_map.as_deref(),
                window: None,
            };
            crate::bitplane::decode_bitplanes_filtered(
                &encoded_packets,
                width,
                height,
                q,
                levels,
                kind,
                &filter,
            )?
        }
    };
    wavelet_int::inverse_2d_dyadic(&mut coeffs, width, height, levels, walked.header.filter);
    // Inverse level-shift + clamp to the n-bit domain (0..=255 at depth
    // 8). Saturating: a corrupted stream can decode coefficients near
    // i32::MAX (mutation smoke).
    let sb = sample_bytes(depth);
    let mid = 1i32 << (depth - 1);
    let max_v = (1i32 << depth) - 1;
    for y in 0..height {
        let dst = &mut plane.data[(y_offset + y) * plane.stride..][..width * sb];
        for x in 0..width {
            let v = coeffs[y * width + x].saturating_add(mid).clamp(0, max_v);
            store_px(dst, x, sb, v);
        }
    }
    Ok(())
}

/// Lenient-decode report returned by [`parse_icer_lenient`].
///
/// The lenient API tolerates a bytestream that is missing entire
/// segments (e.g. DSN packet loss in transit between the rover and the
/// ground station). Missing strips are reconstructed as flat 128
/// (level-shifted zero); the receiver gets back the [`IcerImage`] it
/// would have got plus a per-index "was this segment present?" report
/// so it knows which strips are genuine and which are placeholders.
///
/// IPN 42-155 §III.E "Image Partitioning" is the spec justification:
/// segments are self-contained independently-decodable units, and the
/// paper notes (§I, §III.E) that this independence is what makes ICER
/// loss-tolerant on the deep-space link. The strict [`crate::decode`]
/// rejects gaps in the `segment_index` sequence with
/// `IcerError::invalid("non-contiguous segment indices: ...")`; the
/// lenient API accepts them and surfaces the gap on the report instead.
#[derive(Debug, Clone)]
pub struct LenientDecode {
    /// Decoded image. Missing strips are filled with 128 (level-shifted
    /// zero, matching the round-6 ROI-priority placeholder semantic
    /// already implemented in [`crate::decode`]).
    pub image: IcerImage,
    /// `received[i] == true` iff segment with `segment_index == i` was
    /// present in the bytestream. Length equals
    /// `expected_segment_count` (the inferred maximum index + 1).
    pub received: Vec<bool>,
    /// Number of `false` entries in [`Self::received`] -- i.e. how many
    /// strips are flat-128 placeholders.
    pub missing_count: usize,
}
/// Decode an ICER bytestream that may be missing entire segments due
/// to packet loss in transit (IPN 42-155 §III.E independent-segment
/// scheduling). Missing strips are reconstructed as flat 128; the
/// report carries the per-index presence map and the missing-count.
///
/// Requirements:
///   * Segment 0 **must** be present (it pins the canonical strip
///     height + the canonical width); a missing segment 0 returns
///     `IcerError::Truncated`.
///   * The strip height is inferred as the modal height across all
///     received non-trailing segments. The last received segment is
///     allowed to be shorter (the encoder's `div_ceil` row-strip split
///     produces a trailing remainder).
///   * Width is required to agree across all received segments
///     (canonical-width mismatch still returns
///     `IcerError::Unsupported`).
///   * The reconstructed image height is
///     `last_received_index * strip_h + last_received_height` if the
///     highest-index segment was received, else
///     `(max_received_index + 1) * strip_h` -- the latter case rounds
///     up; the missing trailing-strip-shorter-than-strip_h case isn't
///     recoverable without out-of-band geometry coordination.
///
/// Applies [`DecodeOptions::default`] for geometry validation. Use
/// [`parse_icer_lenient_with`] for explicit control.
///
/// On a bytestream with **no** missing segments, the returned image
/// is bit-identical to what [`crate::decode`] would return and
/// `missing_count == 0`.
pub fn parse_icer_lenient(bytes: &[u8]) -> Result<LenientDecode> {
    parse_icer_lenient_with(bytes, &DecodeOptions::default())
}

/// [`parse_icer_lenient`] with an explicit pre-contract
/// [`DecodeLimits`] policy. Superseded by [`parse_icer_lenient_with`].
#[deprecated(note = "use oxideav_icer::parse_icer_lenient_with(bytes, &DecodeOptions)")]
#[allow(deprecated)]
pub fn parse_icer_lenient_with_limits(
    bytes: &[u8],
    limits: &DecodeLimits,
) -> Result<LenientDecode> {
    parse_icer_lenient_with(bytes, &DecodeOptions::from(limits))
}

/// [`parse_icer_lenient`] with an explicit [`DecodeOptions`] policy.
///
/// For colour images, each plane substream is decoded leniently and
/// re-assembled; the returned `received` / `missing_count` reflect the
/// **luma** plane (plane 0), which is the canonical presence map for the
/// colour image (the chroma planes are encoded with identical segment
/// geometry, so their presence maps coincide on any well-formed stream).
pub fn parse_icer_lenient_with(bytes: &[u8], opts: &DecodeOptions) -> Result<LenientDecode> {
    if bytes.is_empty() {
        return Err(IcerError::Truncated);
    }

    // Colour container: decode each plane substream leniently, assemble
    // into the declared format. Geometry must agree across planes; the
    // luma plane's presence map is reported.
    if crate::plane_container::is_container(bytes) {
        let parsed = crate::plane_container::parse_container(bytes)?;
        let n = parsed.format.plane_count();
        let depth = parsed.bit_depth;
        let mut plane_decodes: Vec<LenientDecode> = Vec::with_capacity(n);
        for i in 0..n {
            let sub = parsed.plane_bytes(bytes, i);
            plane_decodes.push(parse_icer_lenient_single_plane(sub, opts, depth)?);
        }
        let w = plane_decodes[0].image.width;
        let h = plane_decodes[0].image.height;
        for (i, d) in plane_decodes.iter().enumerate() {
            if d.image.width != w || d.image.height != h {
                return Err(IcerError::Unsupported(format!(
                    "colour plane {i} geometry {}x{} disagrees with plane 0 {}x{}",
                    d.image.width, d.image.height, w, h
                )));
            }
        }
        let received = plane_decodes[0].received.clone();
        let missing_count = plane_decodes[0].missing_count;
        let mut out = IcerImage::zeros(w, h, parsed.format);
        out.bit_depth = parsed.bit_depth;
        for (i, d) in plane_decodes.into_iter().enumerate() {
            out.planes[i] = d
                .image
                .planes
                .into_iter()
                .next()
                .ok_or_else(|| IcerError::invalid("decoded colour plane has no data"))?;
        }
        return Ok(LenientDecode {
            image: out,
            received,
            missing_count,
        });
    }

    parse_icer_lenient_single_plane(bytes, opts, 8)
}

/// Single-plane lenient decode at the given sample depth — the
/// historical `parse_icer_lenient` body (depth 8 for bare
/// streams; the deep-gray container carries deeper depths).
fn parse_icer_lenient_single_plane(
    bytes: &[u8],
    opts: &DecodeOptions,
    depth: u8,
) -> Result<LenientDecode> {
    if bytes.is_empty() {
        return Err(IcerError::Truncated);
    }

    // Walk every segment as in `decode_image` -- gather them
    // first so the strip-height inference has the full set.
    let mut walked_all: Vec<WalkedSegment<'_>> = Vec::new();
    let mut cursor = 0usize;
    let mut total_pixels: u64 = 0;
    let mut transform_counted = false;
    while cursor < bytes.len() {
        let walked = walk_segment(&bytes[cursor..])?;
        opts.check_segment(
            walked.header.segment_index,
            walked.header.width as u32,
            walked.header.height as u32,
        )?;
        // §V.B transform-domain segments each declare the full image
        // dimensions; count them once against the total cap.
        let count_pixels = if walked.header.transform_segmented {
            let first = !transform_counted;
            transform_counted = true;
            first
        } else {
            true
        };
        if count_pixels {
            total_pixels = total_pixels
                .checked_add(segment_pixels(&walked.header))
                .ok_or_else(|| IcerError::invalid("multi-segment pixel-count overflow"))?;
        }
        opts.check_total_pixels(total_pixels, "multi-segment image")?;
        cursor += walked.consumed;
        walked_all.push(walked);
    }
    if walked_all.is_empty() {
        return Err(IcerError::Truncated);
    }
    // Sort by segment_index so out-of-order delivery still composes.
    walked_all.sort_by_key(|w| w.header.segment_index);

    // Duplicate segment indices are a geometry contradiction, not a
    // loss-tolerance scenario: the lenient height inference assumes one
    // strip per index, and two same-index segments with different
    // heights would make the placement loop write past the inferred
    // plane (found by the scheduled decode_segment fuzz run; the strict
    // decoder already rejects duplicates via its contiguity check).
    for pair in walked_all.windows(2) {
        if pair[0].header.segment_index == pair[1].header.segment_index {
            return Err(IcerError::Unsupported(format!(
                "duplicate segment index {} in lenient stream",
                pair[0].header.segment_index
            )));
        }
    }

    // §V.B transform-domain streams: any received segment pins the full
    // geometry (every header carries the image dimensions + total
    // segment count), so even segment 0 may be lost. Missing segments'
    // coefficients stay zero — a smooth low-detail patch through the
    // shared inverse transform rather than a flat-128 strip.
    if walked_all.iter().any(|w| w.header.transform_segmented) {
        let (image, received, missing_count) = decode_transform_domain(&walked_all, false, depth)?;
        return Ok(LenientDecode {
            image,
            received,
            missing_count,
        });
    }

    // Segment 0 must be present so we can pin the canonical width +
    // strip height.
    let first = &walked_all[0];
    if first.header.segment_index != 0 {
        return Err(IcerError::Truncated);
    }
    let canonical_width = first.header.width as usize;

    // Width agreement across received segments.
    for w in &walked_all {
        if w.header.width as usize != canonical_width {
            return Err(IcerError::Unsupported(format!(
                "multi-segment width mismatch: segment {} is {}, expected {}",
                w.header.segment_index, w.header.width, canonical_width
            )));
        }
    }

    // Determine the canonical strip height. Convention (matches
    // `encode`'s `div_ceil(h, segment_count)` split): every strip
    // except the last has identical height. We use the height of
    // segment 0 as the canonical strip_h; the trailing segment is
    // allowed to be shorter.
    let strip_h = first.header.height as usize;
    if strip_h == 0 {
        return Err(IcerError::invalid("segment 0 has zero height"));
    }

    let max_received_index = walked_all.last().unwrap().header.segment_index as usize;
    // expected_segment_count = max + 1. The caller could in principle
    // know there were more trailing segments lost; we don't, so we
    // truncate the image at the highest-received index.
    let expected_segment_count = max_received_index + 1;
    let last_received_h = walked_all.last().unwrap().header.height as usize;

    // Image height: (n-1) * strip_h + last_strip_h, where last_strip_h
    // is the trailing-segment height for the highest-received index.
    // (If higher-indexed segments were dropped we don't know about
    // them; the image is truncated at the highest-received boundary.)
    let total_height = max_received_index
        .checked_mul(strip_h)
        .and_then(|v| v.checked_add(last_received_h))
        .ok_or_else(|| IcerError::invalid("multi-segment height overflow"))?;
    if total_height > u32::MAX as usize {
        return Err(IcerError::invalid("multi-segment height overflow"));
    }

    // The reconstructed image spans the full 0..=max_received_index
    // strip range including the missing gap strips, so the *total
    // reconstruction geometry* must honour the pixel cap — the
    // received-segment pixel sum alone does not bound it (two tiny
    // received strips at a huge segment_index gap would otherwise buy
    // a multi-GB placeholder allocation; found by the bounded
    // decode_segment fuzz campaign, r433).
    let recon_pixels = (canonical_width as u64)
        .checked_mul(total_height as u64)
        .ok_or_else(|| IcerError::invalid("multi-segment pixel-count overflow"))?;
    opts.check_total_pixels(recon_pixels, "lenient reconstruction geometry")?;
    opts.check_width_height(canonical_width as u32, total_height as u32)?;
    opts.check_bytes(recon_pixels * sample_bytes(depth) as u64)?;

    let mut img = zeros_gray(canonical_width as u32, total_height as u32, depth);
    let mut received = vec![false; expected_segment_count];

    // Place each received segment at its inferred y_offset.
    for walked in &walked_all {
        let seg_idx = walked.header.segment_index as usize;
        let y_offset = seg_idx * strip_h;
        received[seg_idx] = true;
        // For non-trailing received segments, verify height equals
        // canonical strip_h. A different height in the middle of the
        // stream is a geometry-policy contradiction (the trailing
        // remainder rule allows ONE shorter strip at the END only).
        if seg_idx != max_received_index && (walked.header.height as usize) != strip_h {
            return Err(IcerError::Unsupported(format!(
                "non-trailing segment {} height {} != canonical strip height {}",
                seg_idx, walked.header.height, strip_h
            )));
        }
        decode_segment_into(walked, &mut img.planes[0], y_offset, canonical_width, depth)?;
    }

    // Fill missing-segment regions with flat 128 (level-shifted zero,
    // matching the round-6 ROI-priority placeholder semantic).
    let mut missing_count = 0usize;
    for (seg_idx, was_received) in received.iter().enumerate() {
        if *was_received {
            continue;
        }
        missing_count += 1;
        let y_offset = seg_idx * strip_h;
        // Missing-segment height: canonical strip_h (we don't have a
        // tighter source). Trailing-segment-missing case is handled by
        // the highest-received-index truncation above, so any missing
        // seg here is a *gap*, not a trailing drop.
        fill_mid_rows(
            &mut img.planes[0],
            y_offset,
            strip_h,
            canonical_width,
            depth,
        );
    }

    Ok(LenientDecode {
        image: img,
        received,
        missing_count,
    })
}

/// Decode the IPN 42-155 §III.D "uncompressed" path explicitly. The
/// generic [`crate::decode`] entry point also handles this case, but the
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
    let plane: &mut Plane = &mut img.planes[0];
    for y in 0..h {
        let row_dst = &mut plane.data[y * plane.stride..y * plane.stride + w];
        let row_src = &concat[y * w..y * w + w];
        row_dst.copy_from_slice(row_src);
    }
    Ok(img)
}
