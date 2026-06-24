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
//!
//! ## Decode-side resource limits (round 174)
//!
//! The wire format admits arbitrary `(width, height)` pairs in the
//! 12-byte segment header (`u16 * u16`), which means a single tiny
//! header can request up to ~4 GB of decoder allocation per plane —
//! a DoS surface previously flagged by the cargo-fuzz harness. The
//! [`DecodeLimits`] struct caps the per-segment and per-image pixel
//! counts the decoder will agree to materialise. [`parse_icer`] and
//! [`parse_icer_metadata`] apply [`DecodeLimits::default`] (64 MPx
//! per segment, 256 MPx per image — well above the 1024x1024 / 2048
//! x2048 Mars-rover Pancam / Hazcam fixtures and three orders of
//! magnitude below the 4 GB wire-format ceiling). Callers that need
//! to override the policy use [`parse_icer_with_limits`] /
//! [`parse_icer_metadata_with_limits`].

use crate::bitplane::{decode_bitplanes_multi, EncodedPacket};
use crate::error::{IcerError, Result};
use crate::header::{walk_segment, BitPlanePass, SegmentHeader, WalkedSegment};
use crate::image::{IcerImage, IcerPixelFormat, IcerPlane};
use crate::wavelet_float;

/// Per-decode resource caps.
///
/// The wire format's 16-bit width / 16-bit height fields admit values
/// up to `65535 * 65535 ≈ 4.29 GPx` per segment. Honouring that
/// literally is a DoS surface — a 12-byte segment header could request
/// a ~4 GB plane allocation plus the matching `i32` coefficient buffer
/// (~16 GB). [`DecodeLimits`] is the application-supplied policy that
/// bounds what the decoder will agree to materialise.
///
/// All caps are in **pixels**, not bytes. A Gray8 plane is one byte
/// per pixel; the inverse-DWT coefficient buffer is four bytes
/// (`i32`) per pixel; so a 64-MPx cap bounds peak allocation per
/// segment to roughly `64 MB plane + 256 MB coefficients`.
///
/// The defaults are deliberately conservative-but-realistic for
/// every published Mars-rover ICER deployment:
///
///   * Pancam delivery frames are 1024x1024 = 1 MPx.
///   * Hazcam frames are 1024x1024 = 1 MPx.
///   * Mastcam-Z (Mars 2020) frames are 1648x1200 ≈ 2 MPx.
///   * HiRISE strips (the largest deep-space-imager target) cap out
///     around 20k x 20k = 400 MPx, but those run through ICER-3D
///     hyperspectral, not the 2-D path this crate covers.
///
/// 64 MPx per segment is two orders of magnitude above the actual
/// rover Pancam / Hazcam frames; 256 MPx total bounds a worst-case
/// multi-segment image at ~1 GB peak allocation. Callers with
/// validated trusted inputs can lift the cap via
/// [`DecodeLimits::unlimited`] or by constructing an explicit
/// [`DecodeLimits`] with the desired values.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DecodeLimits {
    /// Maximum `width * height` (in pixels) any single segment is
    /// allowed to request. A segment whose header asks for more
    /// returns [`IcerError::Unsupported`] from the parse / decode
    /// entry points (we treat oversized geometry as an
    /// application-policy refusal, not a wire-format violation).
    pub max_pixels_per_segment: u64,
    /// Maximum total `width * height` (in pixels) the stitched
    /// multi-segment image is allowed to request. Sum-of-segment-
    /// pixel-counts checked before any plane allocation.
    pub max_total_pixels: u64,
}

impl DecodeLimits {
    /// Application-defined default: 64 MPx per segment, 256 MPx total.
    /// See the type-level rationale for why these values were chosen.
    pub const DEFAULT_MAX_PIXELS_PER_SEGMENT: u64 = 64 * 1024 * 1024;
    pub const DEFAULT_MAX_TOTAL_PIXELS: u64 = 256 * 1024 * 1024;

    /// No-cap policy. Use only when the input is trusted (e.g. produced
    /// by this crate's own encoder in a controlled batch run). Equivalent
    /// to round-131 behaviour — fuzz inputs can drive ~4 GB / plane.
    pub const fn unlimited() -> Self {
        Self {
            max_pixels_per_segment: u64::MAX,
            max_total_pixels: u64::MAX,
        }
    }
}

impl Default for DecodeLimits {
    fn default() -> Self {
        Self {
            max_pixels_per_segment: Self::DEFAULT_MAX_PIXELS_PER_SEGMENT,
            max_total_pixels: Self::DEFAULT_MAX_TOTAL_PIXELS,
        }
    }
}

/// Pixel-count of a segment, computed in `u64` to side-step any
/// `usize * usize` overflow risk on 32-bit targets. Always finite for
/// in-range `SegmentHeader::{width, height}` (both `u16`).
#[inline]
fn segment_pixels(header: &SegmentHeader) -> u64 {
    header.width as u64 * header.height as u64
}

fn check_segment_pixels(header: &SegmentHeader, limits: &DecodeLimits) -> Result<()> {
    let px = segment_pixels(header);
    if px > limits.max_pixels_per_segment {
        return Err(IcerError::Unsupported(format!(
            "segment {} geometry {}x{} = {} pixels exceeds per-segment cap of {} pixels \
             (see DecodeLimits::max_pixels_per_segment)",
            header.segment_index, header.width, header.height, px, limits.max_pixels_per_segment
        )));
    }
    Ok(())
}

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
///
/// Applies [`DecodeLimits::default`] for geometry validation. Use
/// [`parse_icer_metadata_with_limits`] for explicit control.
pub fn parse_icer_metadata(bytes: &[u8]) -> Result<IcerMetadata> {
    parse_icer_metadata_with_limits(bytes, &DecodeLimits::default())
}

/// Walk every segment in `bytes` and return per-segment metadata,
/// rejecting any segment whose geometry exceeds `limits`. Header-only
/// walk — no plane allocation.
pub fn parse_icer_metadata_with_limits(
    bytes: &[u8],
    limits: &DecodeLimits,
) -> Result<IcerMetadata> {
    // Colour container: walk each plane substream and concatenate the
    // per-plane segment metadata. Segment `offset` values are rebased to
    // the original container buffer so callers see absolute positions.
    if crate::plane_container::is_container(bytes) {
        let parsed = crate::plane_container::parse_container(bytes)?;
        let mut segments = Vec::new();
        let mut total_pixels: u64 = 0;
        for i in 0..parsed.format.plane_count() {
            let (base, _end) = parsed.plane_ranges[i];
            let sub = parsed.plane_bytes(bytes, i);
            let sub_meta = parse_icer_metadata_with_limits(sub, limits)?;
            for s in &sub_meta.segments {
                total_pixels = total_pixels
                    .checked_add(segment_pixels(&s.header))
                    .ok_or_else(|| IcerError::invalid("colour pixel-count overflow"))?;
                if total_pixels > limits.max_total_pixels {
                    return Err(IcerError::Unsupported(format!(
                        "colour image pixel-count {} exceeds total cap of {} pixels \
                         (see DecodeLimits::max_total_pixels)",
                        total_pixels, limits.max_total_pixels
                    )));
                }
            }
            for mut s in sub_meta.segments {
                s.offset += base;
                segments.push(s);
            }
        }
        return Ok(IcerMetadata { segments });
    }

    let mut segments = Vec::new();
    let mut total_pixels: u64 = 0;
    let mut cursor = 0;
    while cursor < bytes.len() {
        let walked = walk_segment(&bytes[cursor..])?;
        check_segment_pixels(&walked.header, limits)?;
        total_pixels = total_pixels
            .checked_add(segment_pixels(&walked.header))
            .ok_or_else(|| IcerError::invalid("multi-segment pixel-count overflow"))?;
        if total_pixels > limits.max_total_pixels {
            return Err(IcerError::Unsupported(format!(
                "multi-segment image pixel-count {} exceeds total cap of {} pixels \
                 (see DecodeLimits::max_total_pixels)",
                total_pixels, limits.max_total_pixels
            )));
        }
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
///
/// Applies [`DecodeLimits::default`] for geometry validation before
/// any plane / coefficient allocation. Use [`parse_icer_with_limits`]
/// for explicit control.
pub fn parse_icer(bytes: &[u8]) -> Result<IcerImage> {
    parse_icer_with_limits(bytes, &DecodeLimits::default())
}

/// Decode the full ICER bytestream into an image, rejecting any
/// segment / total geometry that exceeds `limits` before allocating
/// the plane or wavelet coefficient buffers.
///
/// This is the constructor [`parse_icer`] delegates into. Pass
/// [`DecodeLimits::unlimited`] to recover the pre-round-174 behaviour
/// (trusted-input batch processing).
pub fn parse_icer_with_limits(bytes: &[u8], limits: &DecodeLimits) -> Result<IcerImage> {
    if bytes.is_empty() {
        return Err(IcerError::Truncated);
    }

    // Colour images are framed as a multi-plane container (leading
    // 0x0000 sentinel — see `crate::plane_container`). A single-plane
    // Gray8 stream never starts with 0x0000, so the dispatch is
    // unambiguous and every historical Gray8 stream falls through to the
    // single-plane path below byte-for-byte unchanged.
    if crate::plane_container::is_container(bytes) {
        return parse_icer_multi_plane(bytes, limits);
    }

    parse_icer_single_plane(bytes, limits)
}

/// Decode a multi-plane (colour) container: each plane substream is a full
/// single-plane ICER bitstream, decoded independently and re-assembled
/// into the declared [`IcerPixelFormat`].
fn parse_icer_multi_plane(bytes: &[u8], limits: &DecodeLimits) -> Result<IcerImage> {
    let parsed = crate::plane_container::parse_container(bytes)?;
    let n = parsed.format.plane_count();

    let mut plane_images: Vec<IcerImage> = Vec::with_capacity(n);
    for i in 0..n {
        let sub = parsed.plane_bytes(bytes, i);
        plane_images.push(parse_icer_single_plane(sub, limits)?);
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

/// Decode a single-plane (Gray8) ICER bitstream. This is the historical
/// `parse_icer_with_limits` body.
fn parse_icer_single_plane(bytes: &[u8], limits: &DecodeLimits) -> Result<IcerImage> {
    if bytes.is_empty() {
        return Err(IcerError::Truncated);
    }

    // Walk every segment first so we know the total height + canonical
    // width up-front.
    let mut walked_all: Vec<WalkedSegment<'_>> = Vec::new();
    let mut cursor = 0usize;
    let mut total_pixels: u64 = 0;
    while cursor < bytes.len() {
        let walked = walk_segment(&bytes[cursor..])?;
        check_segment_pixels(&walked.header, limits)?;
        total_pixels = total_pixels
            .checked_add(segment_pixels(&walked.header))
            .ok_or_else(|| IcerError::invalid("multi-segment pixel-count overflow"))?;
        if total_pixels > limits.max_total_pixels {
            return Err(IcerError::Unsupported(format!(
                "multi-segment image pixel-count {} exceeds total cap of {} pixels \
                 (see DecodeLimits::max_total_pixels)",
                total_pixels, limits.max_total_pixels
            )));
        }
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
                // Decoder side does not need the R-D estimate; default to 0.0.
                delta_distortion: 0.0,
            })
            .collect();
        decode_bitplanes_multi(&encoded_packets, width, height, q, levels)?
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
/// loss-tolerant on the deep-space link. The strict [`parse_icer`]
/// rejects gaps in the `segment_index` sequence with
/// `IcerError::invalid("non-contiguous segment indices: ...")`; the
/// lenient API accepts them and surfaces the gap on the report instead.
#[derive(Debug, Clone)]
pub struct LenientDecode {
    /// Decoded image. Missing strips are filled with 128 (level-shifted
    /// zero, matching the round-6 ROI-priority placeholder semantic
    /// already implemented in [`parse_icer`]).
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
/// Applies [`DecodeLimits::default`] for geometry validation. Use
/// [`parse_icer_lenient_with_limits`] for explicit control.
///
/// On a bytestream with **no** missing segments, the returned image
/// is bit-identical to what [`parse_icer`] would return and
/// `missing_count == 0`.
pub fn parse_icer_lenient(bytes: &[u8]) -> Result<LenientDecode> {
    parse_icer_lenient_with_limits(bytes, &DecodeLimits::default())
}

/// [`parse_icer_lenient`] with an explicit [`DecodeLimits`] policy.
///
/// For colour images, each plane substream is decoded leniently and
/// re-assembled; the returned `received` / `missing_count` reflect the
/// **luma** plane (plane 0), which is the canonical presence map for the
/// colour image (the chroma planes are encoded with identical segment
/// geometry, so their presence maps coincide on any well-formed stream).
pub fn parse_icer_lenient_with_limits(
    bytes: &[u8],
    limits: &DecodeLimits,
) -> Result<LenientDecode> {
    if bytes.is_empty() {
        return Err(IcerError::Truncated);
    }

    // Colour container: decode each plane substream leniently, assemble
    // into the declared format. Geometry must agree across planes; the
    // luma plane's presence map is reported.
    if crate::plane_container::is_container(bytes) {
        let parsed = crate::plane_container::parse_container(bytes)?;
        let n = parsed.format.plane_count();
        let mut plane_decodes: Vec<LenientDecode> = Vec::with_capacity(n);
        for i in 0..n {
            let sub = parsed.plane_bytes(bytes, i);
            plane_decodes.push(parse_icer_lenient_single_plane(sub, limits)?);
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

    parse_icer_lenient_single_plane(bytes, limits)
}

/// Single-plane (Gray8) lenient decode — the historical
/// `parse_icer_lenient_with_limits` body.
fn parse_icer_lenient_single_plane(bytes: &[u8], limits: &DecodeLimits) -> Result<LenientDecode> {
    if bytes.is_empty() {
        return Err(IcerError::Truncated);
    }

    // Walk every segment as in `parse_icer_with_limits` -- gather them
    // first so the strip-height inference has the full set.
    let mut walked_all: Vec<WalkedSegment<'_>> = Vec::new();
    let mut cursor = 0usize;
    let mut total_pixels: u64 = 0;
    while cursor < bytes.len() {
        let walked = walk_segment(&bytes[cursor..])?;
        check_segment_pixels(&walked.header, limits)?;
        total_pixels = total_pixels
            .checked_add(segment_pixels(&walked.header))
            .ok_or_else(|| IcerError::invalid("multi-segment pixel-count overflow"))?;
        if total_pixels > limits.max_total_pixels {
            return Err(IcerError::Unsupported(format!(
                "multi-segment image pixel-count {} exceeds total cap of {} pixels \
                 (see DecodeLimits::max_total_pixels)",
                total_pixels, limits.max_total_pixels
            )));
        }
        cursor += walked.consumed;
        walked_all.push(walked);
    }
    if walked_all.is_empty() {
        return Err(IcerError::Truncated);
    }
    // Sort by segment_index so out-of-order delivery still composes.
    walked_all.sort_by_key(|w| w.header.segment_index);

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
    // `encode_icer`'s `div_ceil(h, segment_count)` split): every strip
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

    let mut img = IcerImage::zeros(
        canonical_width as u32,
        total_height as u32,
        IcerPixelFormat::Gray8,
    );
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
        decode_segment_into(walked, &mut img.planes[0], y_offset, canonical_width)?;
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
        let plane = &mut img.planes[0];
        for y in 0..strip_h {
            let dst = &mut plane.data
                [(y_offset + y) * plane.stride..(y_offset + y) * plane.stride + canonical_width];
            dst.fill(128);
        }
    }

    Ok(LenientDecode {
        image: img,
        received,
        missing_count,
    })
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
