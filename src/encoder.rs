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

use crate::bitplane::{encode_bitplanes, select_bit_plane_count, BitPlaneInput, EncodedPacket};
use crate::error::{IcerError, Result};
use crate::header::{BitPlanePass, PacketHeader, SegmentHeader, WaveletFilter};
use crate::image::{IcerImage, IcerPixelFormat, IcerPlane};
use crate::wavelet_float;

/// Encoder options.
///
/// Note: this type is `Clone` (not `Copy`) because the
/// [`Self::segment_priorities`] field carries an owned `Vec<u16>` when
/// region-of-interest prioritisation is in use (round 6).
#[derive(Debug, Clone)]
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
    /// Hard byte budget for the entire output stream. The encoder
    /// finalises each packet and, before starting the next packet,
    /// checks whether the running output size plus the segment-header
    /// overhead would exceed this limit. If so, it stops emitting
    /// packets and returns what has been written so far.
    ///
    /// Mid-packet truncation is not performed; the encoder always
    /// finishes the in-progress packet cleanly. The output will be
    /// ≤ `byte_budget` bytes (plus the segment header that was already
    /// committed before the first packet).
    ///
    /// `None` (the default) disables the hard cap.
    pub byte_budget: Option<u64>,
    /// Soft byte target. When set, the encoder finishes the current
    /// bit-plane's packet pair (significance + refinement) even if the
    /// running output has already exceeded `target_bytes`, then stops.
    ///
    /// When used together with `byte_budget`, the semantics are
    /// "keep going until `target_bytes` is reached, then finish the
    /// current bit-plane pair, but never exceed `byte_budget`". The
    /// hard cap is always honoured.
    ///
    /// `None` (the default) falls back to the hard-cap behaviour (stop
    /// immediately after any packet that exceeds `byte_budget`).
    pub target_bytes: Option<u64>,
    /// Automatic filter selection (round 5). When `true`, the encoder
    /// runs [`crate::analyze::ImageStats::from_image`] over the input,
    /// then either:
    ///   * picks a filter via [`crate::analyze::recommend_filter`] (fast
    ///     heuristic, single encode pass), or
    ///   * if [`Self::auto_filter_rd`] is also `true`, runs the
    ///     [`crate::analyze::pick_filter_by_rate_distortion`] trial
    ///     loop (N encode passes, picks the smallest output).
    ///
    /// The explicitly-set [`Self::filter`] is **overridden** when this
    /// flag is on. `false` (the default) keeps the existing behaviour:
    /// the caller's chosen `filter` is used as-is.
    pub auto_filter: bool,
    /// When [`Self::auto_filter`] is on, this flag selects the
    /// rate-distortion (true minimum-byte-count) variant over the cheap
    /// heuristic. `false` (the default) uses the heuristic.
    pub auto_filter_rd: bool,
    /// Region-of-interest (ROI) segment-priority vector (round 6).
    ///
    /// When `Some(v)`, multi-segment encoding (i.e. `segment_count >
    /// 1`) walks segments in order of ascending priority value: rank
    /// `0` is encoded first, rank `1` second, and so on. The vector
    /// length must equal `segment_count` and the vector must be a
    /// permutation of `0..segment_count` (each rank appears exactly
    /// once).
    ///
    /// The output byte stream is the per-segment encoded blobs
    /// concatenated **in priority order**, not in segment-index
    /// order. The decoder (`parse_icer`) already sorts segments by
    /// their on-the-wire `segment_index` field before stitching the
    /// strips, so out-of-priority emission is transparent on the
    /// decode side.
    ///
    /// When combined with [`Self::byte_budget`], the practical effect
    /// is that the highest-priority segments are written to the
    /// stream before the budget is exhausted, so the highest-priority
    /// regions of the image keep their fidelity while the
    /// lower-priority regions are truncated. This is the spaceflight
    /// equivalent of JPEG 2000's ROI coding: Mars Pancam imagery
    /// typically prioritises the centre of the frame (where the
    /// science target sits) over the periphery (sky / rover hardware
    /// at the edges).
    ///
    /// `None` (the default) keeps the existing behaviour: segments
    /// are emitted in `segment_index` order.
    ///
    /// IPN 42-155 §III.E notes that the segment partitioning gives
    /// the encoder the freedom to schedule segments independently;
    /// this field is the realisation of that property.
    pub segment_priorities: Option<Vec<u16>>,
    /// Rate-distortion (R-D) budget pruning (round 91, IPN 42-155
    /// §IV.B rate-allocation principle).
    ///
    /// When `true` AND [`Self::byte_budget`] is set AND the
    /// compressed-path is in use, the compressed-segment encoder
    /// no longer truncates packets in strict MSB-down emission
    /// order. Instead it:
    ///
    /// 1. Encodes every `(bit-plane, pass)` packet into a candidate
    ///    list with its byte size and per-packet distortion-reduction
    ///    estimate (see [`crate::bitplane::EncodedPacket::delta_distortion`]).
    /// 2. Greedily includes packets in descending `delta_distortion /
    ///    byte_size` order (cost-per-byte ranking), subject to the
    ///    decoding-dependency graph: significance at bit-plane `bp`
    ///    requires significance at every higher bit-plane (MSB-down
    ///    chain); refinement at `bp` requires significance at `bp`.
    /// 3. Re-serialises the chosen subset in MSB-down on-the-wire order
    ///    (the decoder expects that ordering — only the *set* of
    ///    included packets changes, not the ordering of those that are
    ///    kept).
    ///
    /// In practice this typically drops the lowest-bit-plane
    /// refinement packets (low ΔD-per-byte) in favour of including
    /// more significance packets in the budget, yielding higher PSNR
    /// for the same byte budget than the strict-MSB-truncation
    /// behaviour.
    ///
    /// `false` (the default) preserves the prior behaviour: packets
    /// are emitted MSB-down sig-before-ref, truncated at the first
    /// packet that would overflow `byte_budget`.
    pub rd_pruning: bool,
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
            byte_budget: None,
            target_bytes: None,
            auto_filter: false,
            auto_filter_rd: false,
            segment_priorities: None,
            rd_pruning: false,
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

    /// Set a hard byte budget. The encoder will not exceed this many
    /// bytes in the output (excluding any partially-committed segment
    /// header). See [`EncodeOptions::byte_budget`].
    #[must_use]
    pub fn with_byte_budget(mut self, n: u64) -> Self {
        self.byte_budget = Some(n);
        self
    }

    /// Set a hard byte budget AND enable rate-distortion-driven packet
    /// selection (round 91, IPN 42-155 §IV.B). See
    /// [`EncodeOptions::rd_pruning`] for the full contract. The encoder
    /// will:
    ///
    /// * Compute every candidate `(bit-plane, pass)` packet up-front.
    /// * Rank them by `delta_distortion / byte_size` (cost-per-byte).
    /// * Greedily include packets in descending score order, subject to
    ///   the MSB-down dependency graph.
    /// * Emit the kept subset in MSB-down on-the-wire order.
    ///
    /// Equivalent to `.with_byte_budget(n)` followed by manually
    /// setting `rd_pruning = true`. Combined with `with_target_bytes`
    /// the soft-target stop is **disabled** (the R-D selection scans
    /// the full packet list and the hard cap is honoured by the
    /// greedy stage).
    #[must_use]
    pub fn with_rd_budget(mut self, n: u64) -> Self {
        self.byte_budget = Some(n);
        self.rd_pruning = true;
        self
    }

    /// Set a soft byte target. The encoder finishes the current
    /// bit-plane's packet pair once the running total has reached
    /// `n` bytes, then stops. See [`EncodeOptions::target_bytes`].
    #[must_use]
    pub fn with_target_bytes(mut self, n: u64) -> Self {
        self.target_bytes = Some(n);
        self
    }

    /// Enable automatic filter selection using the cheap
    /// [`crate::analyze::recommend_filter`] heuristic. Overrides any
    /// previously-set `filter` value on the call to [`encode_icer`].
    /// See [`EncodeOptions::auto_filter`].
    #[must_use]
    pub fn with_auto_filter(mut self) -> Self {
        self.auto_filter = true;
        self.auto_filter_rd = false;
        self
    }

    /// Enable rate-distortion-driven automatic filter selection. The
    /// encoder will trial-encode the image with each filter in
    /// [`crate::analyze::DEFAULT_RD_CANDIDATES`] and use the one that
    /// produces the smallest output. See
    /// [`EncodeOptions::auto_filter_rd`]. Costs roughly N x a single
    /// encode pass.
    #[must_use]
    pub fn with_auto_filter_rd(mut self) -> Self {
        self.auto_filter = true;
        self.auto_filter_rd = true;
        self
    }

    /// Attach an ROI segment-priority vector (round 6). See
    /// [`EncodeOptions::segment_priorities`] for the contract.
    ///
    /// The vector length must equal [`Self::segment_count`] and the
    /// values must form a permutation of `0..segment_count`. Validation
    /// happens at [`encode_icer`] time (so the builder remains
    /// infallible); a malformed vector returns an
    /// [`crate::error::IcerError::Unsupported`] error at encode.
    #[must_use]
    pub fn with_segment_priorities(mut self, priorities: Vec<u16>) -> Self {
        self.segment_priorities = Some(priorities);
        self
    }

    /// Convenience: assign segment priorities so the centre of the
    /// image (in row order) is encoded first, and the top/bottom edges
    /// last (round 6 ROI prioritisation).
    ///
    /// The mapping is:
    ///
    ///   * Segment `mid` (closest to the centre row) -> priority `0`.
    ///   * Then alternating outward (mid-1, mid+1, mid-2, mid+2, ...)
    ///     get priorities 1, 2, 3, ...
    ///
    /// For a segment_count of 5 the priority vector is `[3, 1, 0, 2,
    /// 4]`; for 4 segments it is `[3, 1, 0, 2]` (the lower of the two
    /// centre segments is selected as the absolute centre).
    ///
    /// Combined with [`Self::with_byte_budget`], this yields a
    /// "save the centre first" emission order useful for Mars rover
    /// Pancam imagery where the science target sits at the frame's
    /// centre. `segment_count == 1` is a no-op; the priorities vector
    /// is set to `[0]`.
    #[must_use]
    pub fn with_center_roi(mut self) -> Self {
        let n = self.segment_count.max(1) as usize;
        if n == 1 {
            self.segment_priorities = Some(vec![0]);
            return self;
        }
        let mid = (n - 1) / 2;
        // priorities[seg_idx] = rank; rank starts at 0 for `mid`, then
        // alternates outward (mid-1, mid+1, mid-2, mid+2, ...).
        let mut priorities = vec![0u16; n];
        let mut rank: u16 = 0;
        priorities[mid] = rank;
        rank = rank.saturating_add(1);
        let mut step: isize = 1;
        loop {
            // Try mid - step.
            let lo = mid as isize - step;
            if lo >= 0 {
                priorities[lo as usize] = rank;
                rank = rank.saturating_add(1);
            }
            // Try mid + step.
            let hi = mid as isize + step;
            if (hi as usize) < n {
                priorities[hi as usize] = rank;
                rank = rank.saturating_add(1);
            }
            if (lo < 0) && ((hi as usize) >= n) {
                break;
            }
            step += 1;
        }
        self.segment_priorities = Some(priorities);
        self
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

    // Round 5: automatic filter selection. Resolve the effective filter
    // once at the top of encode, then proceed with a `resolved_opts`
    // that has `auto_filter` cleared so downstream paths see a
    // concrete filter id.
    let resolved_opts = if opts.auto_filter && !opts.uncompressed {
        let chosen = if opts.auto_filter_rd {
            let (filter, _bytes) = crate::analyze::pick_filter_by_rate_distortion(
                image,
                opts,
                crate::analyze::DEFAULT_RD_CANDIDATES,
            )?;
            filter
        } else {
            let stats = crate::analyze::ImageStats::from_image(image);
            crate::analyze::recommend_filter(&stats)
        };
        EncodeOptions {
            filter: chosen,
            auto_filter: false,
            auto_filter_rd: false,
            ..opts.clone()
        }
    } else {
        opts.clone()
    };
    let opts = &resolved_opts;

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

    // Compute the per-segment row offsets (in segment_index order).
    let n_segs = segment_count as usize;
    let mut starts_heights: Vec<(usize, usize)> = Vec::with_capacity(n_segs);
    {
        let mut y_cursor = 0usize;
        while y_cursor < h {
            let this_h = (h - y_cursor).min(strip_h);
            starts_heights.push((y_cursor, this_h));
            y_cursor += this_h;
        }
    }
    if starts_heights.len() != n_segs {
        return Err(IcerError::Unsupported(format!(
            "segment_count {segment_count} would produce {} strips for height {h}",
            starts_heights.len()
        )));
    }

    // Round 6: build the emission order. By default it is
    // 0..segment_count (segment_index order). When
    // `opts.segment_priorities` is supplied the emission order is the
    // permutation that lists segment indices in ascending rank.
    let emission_order: Vec<u16> = match &opts.segment_priorities {
        None => (0..segment_count).collect(),
        Some(prios) => {
            if prios.len() != n_segs {
                return Err(IcerError::Unsupported(format!(
                    "segment_priorities length {} != segment_count {}",
                    prios.len(),
                    segment_count
                )));
            }
            // Validate that prios is a permutation of 0..n_segs.
            let mut seen = vec![false; n_segs];
            for &r in prios {
                let r_us = r as usize;
                if r_us >= n_segs {
                    return Err(IcerError::Unsupported(format!(
                        "segment_priorities entry {r} out of range 0..{n_segs}"
                    )));
                }
                if seen[r_us] {
                    return Err(IcerError::Unsupported(format!(
                        "segment_priorities entry {r} appears more than once"
                    )));
                }
                seen[r_us] = true;
            }
            // Invert: position_of_rank[rank] = segment_index that holds it.
            let mut position_of_rank = vec![0u16; n_segs];
            for (seg_idx, &rank) in prios.iter().enumerate() {
                position_of_rank[rank as usize] = seg_idx as u16;
            }
            position_of_rank
        }
    };

    // Round 6: when ROI priorities are in use AND we have a byte
    // budget, the budgeting strategy is different:
    //   * we encode each segment's *full* output into a per-segment
    //     `Vec<u8>` first;
    //   * then walk the emission order picking entire segments while
    //     budget allows;
    //   * for segments that don't fit, emit a 12-byte "placeholder
    //     header" (segment_length = 0, no packets) so the decoder
    //     can still account for the missing strip at the correct
    //     y_offset.
    //
    // Without priorities (the legacy path) we keep the original
    // index-order streaming behaviour, which also requires a
    // contiguous segment_index sequence for the decoder.
    let mut out = Vec::new();
    let priorities_active = opts.segment_priorities.is_some();

    if !priorities_active {
        // Legacy path: encode + emit in index order, respect the budget
        // strip-by-strip via per-segment budget propagation.
        for &seg_idx in &emission_order {
            let (y_start, this_h) = starts_heights[seg_idx as usize];

            if let Some(budget) = opts.byte_budget {
                if out.len() as u64 >= budget {
                    break;
                }
            }
            let inner_opts = if let Some(budget) = opts.byte_budget {
                let remaining = budget.saturating_sub(out.len() as u64);
                let mut o = opts.clone();
                o.byte_budget = Some(remaining);
                o
            } else {
                opts.clone()
            };
            let bytes =
                encode_one_segment(plane, w, y_start, this_h, seg_idx, &inner_opts, levels)?;
            if let Some(budget) = opts.byte_budget {
                if out.len() as u64 + bytes.len() as u64 > budget {
                    break;
                }
            }
            out.extend_from_slice(&bytes);
        }
        return Ok(out);
    }

    // ROI-priority path. Encode all segments first, then schedule.
    let mut encoded_segments: Vec<Vec<u8>> = Vec::with_capacity(n_segs);
    for seg_idx in 0..n_segs {
        let (y_start, this_h) = starts_heights[seg_idx];
        let bytes = encode_one_segment(plane, w, y_start, this_h, seg_idx as u16, opts, levels)?;
        encoded_segments.push(bytes);
    }
    // First pass: walk emission order, greedily include whole encoded
    // segments while the budget allows.
    let mut kept = vec![false; n_segs];
    for &seg_idx in &emission_order {
        let candidate_len = encoded_segments[seg_idx as usize].len() as u64;
        if let Some(budget) = opts.byte_budget {
            // Reserve 12 bytes for every later not-yet-kept segment so
            // the placeholder headers fit. (Including the current
            // segment in the not-yet-decided list.)
            let not_decided = (0..n_segs)
                .filter(|&i| !kept[i] && i != seg_idx as usize)
                .count();
            let reserve = (not_decided as u64) * (SegmentHeader::ENCODED_BYTES as u64);
            if (out.len() as u64) + candidate_len + reserve > budget {
                // Skip this segment; a 12-byte placeholder will be
                // emitted at the end.
                continue;
            }
        }
        out.extend_from_slice(&encoded_segments[seg_idx as usize]);
        kept[seg_idx as usize] = true;
    }
    // Second pass: emit placeholder headers for every segment that was
    // skipped, in segment_index order. The header carries the strip's
    // width + height so the decoder knows the row offset; segment_length
    // is zero so no packets follow.
    for (seg_idx, &was_kept) in kept.iter().enumerate() {
        if was_kept {
            continue;
        }
        let (_y_start, this_h) = starts_heights[seg_idx];
        // A placeholder uses bit_plane_count=1 so the field validates,
        // and the compressed flag matches the rest of the segments.
        let placeholder = SegmentHeader {
            sync_prefix: opts.sync_prefix,
            filter: opts.filter,
            decomp_levels: levels.clamp(1, 6),
            uncompressed: opts.uncompressed,
            width: w as u16,
            height: this_h as u16,
            bit_plane_count: opts.bit_plane_count.clamp(1, 32),
            segment_length: 0,
            segment_index: seg_idx as u16,
        };
        out.extend_from_slice(&placeholder.encode());
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

    // Round 91: rate-distortion-driven packet selection (IPN 42-155
    // §IV.B rate-allocation principle). When enabled, the packet
    // *set* is chosen by greedy ΔD/byte ranking before serialisation,
    // rather than walked MSB-down with a strict truncation cut-off.
    // See `select_packets_by_rd` below for the algorithm.
    let kept_mask: Option<Vec<bool>> = match (opts.rd_pruning, opts.byte_budget) {
        (true, Some(budget)) => {
            let seg_hdr_bytes = SegmentHeader::ENCODED_BYTES as u64;
            let body_budget = budget.saturating_sub(seg_hdr_bytes);
            Some(select_packets_by_rd(&packets, q, body_budget))
        }
        _ => None,
    };

    // Quota-controlled serialisation: walk packets in priority order
    // (MSB-down, significance before refinement per §IV), stopping
    // when the byte budget / soft target is exceeded.
    //
    // The segment header is SegmentHeader::ENCODED_BYTES bytes long and
    // is committed unconditionally; the budget covers the body (packet
    // headers + bodies only).
    //
    // Hard cap: before appending a packet, check if the current body
    // size + PacketHeader::ENCODED_BYTES + packet body would push the
    // total output (segment header + body) over byte_budget. If so,
    // drop the packet and all subsequent ones.
    //
    // Soft target: once the body size has met or exceeded target_bytes
    // (accounting for the segment header), finish the current
    // bit-plane's pair (the remainder of the current significance +
    // its matching refinement packet), then stop.
    //
    // The two options are composable: soft target controls when the
    // encoder *decides* to wrap up the current bit-plane pair; hard
    // cap enforces the absolute ceiling even within that pair.
    let seg_hdr_bytes = SegmentHeader::ENCODED_BYTES as u64;
    let mut body: Vec<u8> = Vec::new();
    // Whether the encoder has decided to stop after the current
    // bit-plane pair (triggered by soft target).
    let mut soft_stop_after_bp: Option<u8> = None;

    // Walk packets in order: they arrive as (sig_0, ref_0, sig_1, ref_1, ...)
    // from encode_bitplanes. We process them two at a time (by bit_plane)
    // to implement the soft-target "finish this bit-plane pair" semantic.
    let q_usize = q as usize;
    'bp_loop: for bp_idx in 0..q_usize {
        // Check if soft-target stop was requested for a previous bit-plane.
        if let Some(stop_bp) = soft_stop_after_bp {
            if bp_idx as u8 > stop_bp {
                break 'bp_loop;
            }
        }

        for &is_sig in &[true, false] {
            let pkt_index = packets
                .iter()
                .position(|p| p.bit_plane == bp_idx as u8 && p.is_significance == is_sig);
            let pkt_index = match pkt_index {
                Some(i) => i,
                None => continue,
            };
            let pkt = &packets[pkt_index];

            // Round 91: R-D pruning -- skip packets the greedy
            // ΔD/byte selector did not pick. Skipping is safe because
            // the dependency graph (sig(bp) depends on sig(bp-1);
            // ref(bp) depends on sig(bp)) was enforced inside the
            // selector.
            if let Some(mask) = &kept_mask {
                if !mask[pkt_index] {
                    continue;
                }
            }

            // Guard against individual packet body overflowing u16.
            if pkt.body.len() > u16::MAX as usize {
                return Err(IcerError::Unsupported(format!(
                    "packet body {} exceeds u16 limit",
                    pkt.body.len()
                )));
            }

            let pkt_wire_len = PacketHeader::ENCODED_BYTES as u64 + pkt.body.len() as u64;
            let new_body_len = body.len() as u64 + pkt_wire_len;
            let new_total = seg_hdr_bytes + new_body_len;

            // Hard cap check: if adding this packet would exceed the
            // budget, stop immediately (do not emit the packet).
            // (When the R-D selector is in use it has already enforced
            // the budget; this check remains as a defensive belt &
            // braces.)
            if let Some(budget) = opts.byte_budget {
                if new_total > budget {
                    if kept_mask.is_some() {
                        // R-D mode: selector should have prevented this,
                        // but guard against floating-point or off-by-one
                        // corner cases by simply skipping this packet
                        // (do not break, later packets may still fit).
                        continue;
                    }
                    break 'bp_loop;
                }
            }

            // Emit the packet.
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
            body.extend_from_slice(&ph.encode());
            body.extend_from_slice(&pkt.body);

            // Soft target check: once the running total meets or
            // exceeds target_bytes, request a stop after the current
            // bit-plane pair finishes (i.e. after the refinement
            // packet for this bit-plane index).
            if let Some(target) = opts.target_bytes {
                if (seg_hdr_bytes + body.len() as u64) >= target && soft_stop_after_bp.is_none() {
                    soft_stop_after_bp = Some(bp_idx as u8);
                }
            }
        }
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
    let mut opts_copy = opts.clone();
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

/// Rate-distortion packet selector (round 91, IPN 42-155 §IV.B
/// rate-allocation principle).
///
/// Returns a mask `kept[i] = true` for every packet in `packets` that
/// should be emitted, given the body byte budget `body_budget` (the
/// budget the body has after the segment-header overhead is
/// subtracted) and the bit-plane count `q`.
///
/// ICER's per-packet dependency graph is a CHAIN: a significance
/// packet at bit-plane index `bp` requires the significance packets
/// at every higher-priority bit-plane (`0..bp`) to have been decoded
/// first, otherwise the decoder's per-stripe iteration order drifts
/// from the encoder's. Refinement packets at `bp` similarly require
/// the full sig chain up to and including `bp` so the
/// significant-coefficient set in the decoder matches the encoder's
/// at refinement-decode time.
///
/// Given that chain, the optimisation reduces to two coupled
/// decisions:
///
///   1. **Truncation depth K**: the highest bit-plane index whose
///      significance packet is included. Sig packets `0..=K` are
///      mandatory (skipping any would desync the decoder for every
///      packet at higher bp).
///   2. **Refinement subset**: among the refinement packets at
///      `bp_idx ∈ 0..=K` (those whose sig prerequisites are present),
///      which ones to include given the remaining byte budget.
///
/// Algorithm: for every candidate depth `K ∈ 0..q`, compute the cost
/// of the mandatory sig chain `0..=K` and the residual budget
/// `body_budget - mandatory_cost`. If the residual is non-negative,
/// greedily fill it with the highest ΔD-per-byte refinement packets
/// from `bp_idx ∈ 0..=K`, sorted by score descending (with a packet-
/// index tie-break for determinism). The total `delta_distortion`
/// collected at this depth is the sig chain's distortion plus the
/// chosen refinements'.
///
/// The depth K that yields the highest total ΔD becomes the chosen
/// plan; its mask is returned. Ties in total ΔD are broken in favour
/// of the SHALLOWER depth (less bytes), keeping the output minimum-
/// size at equal quality.
///
/// Complexity: O(q² + q log q) per call. For ICER segments q ≤ 31
/// this is essentially constant.
fn select_packets_by_rd(packets: &[EncodedPacket], q: u8, body_budget: u64) -> Vec<bool> {
    let n = packets.len();
    let q_usize = q as usize;
    if n == 0 || body_budget == 0 {
        return vec![false; n];
    }

    // Per-packet wire size (PacketHeader::ENCODED_BYTES + body bytes).
    let wire_sizes: Vec<u64> = packets
        .iter()
        .map(|p| PacketHeader::ENCODED_BYTES as u64 + p.body.len() as u64)
        .collect();

    // Index lookup: sig_idx[bp] = index in `packets` of the sig packet
    // for bit-plane index bp (or None if absent).
    let mut sig_idx: Vec<Option<usize>> = vec![None; q_usize];
    let mut ref_idx: Vec<Option<usize>> = vec![None; q_usize];
    for (i, p) in packets.iter().enumerate() {
        let bp = p.bit_plane as usize;
        if bp >= q_usize {
            continue;
        }
        if p.is_significance {
            sig_idx[bp] = Some(i);
        } else {
            ref_idx[bp] = Some(i);
        }
    }

    let mut best_mask = vec![false; n];
    let mut best_score = f64::NEG_INFINITY;
    let mut best_bytes = u64::MAX;

    // Depth K = -1 (empty plan, zero distortion). Always a candidate
    // (handles the body_budget-too-small-for-any-sig case).
    if 0.0 > best_score {
        best_score = 0.0;
        best_bytes = 0;
    }

    // Try every depth K from 0..q (inclusive of the deepest possible
    // sig). For each, compute the mandatory sig-chain cost.
    let mut mandatory_cost: u64 = 0;
    for k in 0..q_usize {
        let s_idx = match sig_idx[k] {
            Some(i) => i,
            None => break, // chain breaks here; can't go deeper
        };
        let s_cost = wire_sizes[s_idx];
        if mandatory_cost + s_cost > body_budget {
            // Sig chain `0..=K` doesn't fit. Deeper K won't fit either.
            break;
        }
        mandatory_cost += s_cost;
        let mut plan_mask = vec![false; n];
        let mut plan_score = 0.0f64;
        for j in 0..=k {
            if let Some(i) = sig_idx[j] {
                plan_mask[i] = true;
                plan_score += packets[i].delta_distortion;
            }
        }
        let mut plan_bytes = mandatory_cost;

        // Refinement candidates: ref(bp) for bp ∈ 0..=K, sorted by
        // ΔD/byte descending, packet-index tie-break.
        let mut ref_candidates: Vec<usize> = (0..=k).filter_map(|bp| ref_idx[bp]).collect();
        ref_candidates.sort_by(|&a, &b| {
            let sa = if wire_sizes[a] == 0 {
                0.0
            } else {
                packets[a].delta_distortion / wire_sizes[a] as f64
            };
            let sb = if wire_sizes[b] == 0 {
                0.0
            } else {
                packets[b].delta_distortion / wire_sizes[b] as f64
            };
            sb.partial_cmp(&sa)
                .unwrap_or(std::cmp::Ordering::Equal)
                .then(a.cmp(&b))
        });

        // Greedy fill.
        for ri in ref_candidates {
            let w = wire_sizes[ri];
            if plan_bytes + w > body_budget {
                continue;
            }
            // Skip if it contributes nothing (don't waste bytes on
            // zero-ΔD refinements when other options exist; harmless
            // when none do).
            if packets[ri].delta_distortion <= 0.0 {
                continue;
            }
            plan_mask[ri] = true;
            plan_bytes += w;
            plan_score += packets[ri].delta_distortion;
        }

        // Pick the plan with the highest score; tie-break on smaller
        // bytes (so we don't bloat output with no quality gain).
        let better =
            plan_score > best_score || (plan_score == best_score && plan_bytes < best_bytes);
        if better {
            best_score = plan_score;
            best_bytes = plan_bytes;
            best_mask = plan_mask;
        }
    }

    best_mask
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
