//! Encoder entry points.
//!
//! Round-3 surface:
//!
//!   * **Uncompressed path** (IPN 42-155 §III.D) -- bypass the entropy
//!     stage and ship raw 8-bit pixels in a single packet. This is
//!     the fallback Mars-rover deployments use when the entropy
//!     coder would *expand* the payload.
//!   * **Compressed path** -- the IPN 42-155 §II.A reversible integer
//!     wavelet transform (any of the seven Table 1 filters A-F + Q,
//!     filter `Q` by default; see [`crate::wavelet_int`]) followed by
//!     the stripe-ordered bit-plane scanner in [`crate::bitplane`]
//!     feeding the binary arithmetic coder. Self-roundtrips bit-exact
//!     with the matching decoder under every filter; lossy operation
//!     is progressive truncation, never the transform.
//!   * **Multi-packet ordering** -- the compressed path now emits one
//!     packet pair per bit-plane (significance + refinement) per IPN
//!     42-155 §IV. Truncated streams reconstruct at lower quality.
//!   * **Multi-segment** -- large images split into `segment_count`
//!     row-strip segments, each carrying an independently-decodable
//!     coefficient buffer per IPN 42-155 §III.E.

use crate::bitplane::{select_bit_plane_count, BitPlaneInput, EncodedPacket, ScanFilter};
use crate::error::{IcerError, Result};
use crate::header::{BitPlanePass, PacketHeader, SegmentHeader, WaveletFilter};
use crate::image::{IcerImage, IcerPixelFormat, IcerPlane};
use crate::wavelet_int;

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
    /// Per-segment automatic uncompressed fallback (IPN 42-155 §III.D
    /// "Performance with Difficult Imagery").
    ///
    /// When `true` AND the compressed path is in use (i.e.
    /// [`Self::uncompressed`] is `false`), the encoder produces *both*
    /// the compressed and the uncompressed encoding of each segment and
    /// emits whichever is smaller. The wire-format `uncompressed` flag
    /// on the segment header records which path was taken, so the
    /// decoder reconstructs from whatever was emitted without any
    /// caller-side awareness.
    ///
    /// IPN 42-155 §III.D motivates this as the path Mars-rover
    /// deployments take when the entropy stage would *expand* the
    /// payload: pure-noise tiles, single-pixel transitions on otherwise
    /// flat backgrounds, and other content where the per-packet header
    /// plus arithmetic-coder per-packet renormalisation overhead
    /// exceeds the data the entropy stage can squeeze out. The paper
    /// notes (§III.D) that a per-segment decision keeps the entropy
    /// stage for the easy parts of the image while shipping the
    /// difficult tiles uncompressed.
    ///
    /// Compose-rules:
    ///
    /// * The fallback only fires when [`Self::uncompressed`] is
    ///   `false`. Setting [`Self::uncompressed`] to `true` forces
    ///   the uncompressed path unconditionally and ignores this flag.
    /// * The uncompressed segment body must fit `body_len <=
    ///   u16::MAX = 65535 pixels` per IPN 42-155 §IV. If the strip
    ///   exceeds that cap the uncompressed candidate cannot be
    ///   produced and the encoder keeps the compressed result with no
    ///   error.
    /// * The fallback is per-segment; in a multi-segment image
    ///   different segments may independently take the compressed or
    ///   uncompressed path.
    /// * `auto_filter_rd` runs *outside* the fallback selection: each
    ///   filter candidate is offered the fallback choice
    ///   independently, then the byte-smallest of `(filter,
    ///   path)` combinations wins.
    /// * Quota interaction (`byte_budget` / `target_bytes` /
    ///   `rd_pruning`) is honoured by the compressed candidate as
    ///   today; the uncompressed candidate is byte-fixed (header +
    ///   raw pixels) and never truncated.
    ///
    /// `false` (the default) keeps the compressed-or-fail semantic
    /// callers had before this option existed.
    pub auto_uncompressed_fallback: bool,
    /// Quality-target rate-control (round 233).
    ///
    /// When `Some(target_db)`, the compressed-path encoder runs a binary
    /// search over `byte_budget` candidates, decoding each trial output
    /// and computing the PSNR (dB) of the reconstruction against the
    /// original `image`. The smallest output whose PSNR is greater than
    /// or equal to `target_db` is emitted.
    ///
    /// This is the inverse of [`Self::byte_budget`]: byte-budget says
    /// "encode at most N bytes, take whatever quality that yields";
    /// quality-target says "encode at whatever byte count is needed to
    /// reach quality Q, return the smallest such output". For
    /// bandwidth-limited downlink pipelines that ship every image at the
    /// same quality (rather than the same byte count) this is the
    /// natural control knob.
    ///
    /// The search is bracketed by the byte-count range exposed by
    /// [`crate::analyze::quality_search_bounds`] (a small lower bound
    /// covering a header-only emission and an upper bound covering an
    /// unbudgeted compressed-path encode). It costs `O(log(N))`
    /// encode-then-decode trials, where `N` is the byte-count range. A
    /// `target_db` already met by the smallest trial returns the smallest
    /// trial; a `target_db` above the unbudgeted encode's PSNR returns
    /// the unbudgeted encode (with the §II.A reversible integer
    /// transform the unbudgeted encode is lossless, so every finite
    /// target is reachable there).
    ///
    /// Compose-rules:
    ///
    /// * Only fires when [`Self::uncompressed`] is `false`. Forcing the
    ///   uncompressed path makes quality-target a no-op (uncompressed
    ///   is bit-exact already; PSNR is `+inf`).
    /// * Composes with [`Self::auto_filter`] / [`Self::auto_filter_rd`]:
    ///   the search runs *after* filter resolution, so the chosen filter
    ///   is used for every trial.
    /// * Mutually exclusive with [`Self::byte_budget`] /
    ///   [`Self::target_bytes`] / [`Self::rd_pruning`] (the search
    ///   manages the budget directly). Setting both returns
    ///   `IcerError::Unsupported` at encode time.
    /// * The PSNR used is identical to the one computed in the
    ///   round-trip tests: MSE over the first plane, then `10 *
    ///   log10(255^2 / MSE)`. Identical reconstructions return
    ///   `f32::INFINITY` and trivially satisfy any finite `target_db`.
    ///
    /// `None` (the default) preserves the pre-round-233 behaviour: no
    /// quality search, the caller's `byte_budget` (if any) governs.
    pub quality_target_psnr: Option<f32>,

    /// Code each compressed segment's packet bodies with ICER's §IV
    /// interleaved entropy coder instead of the binary arithmetic coder.
    /// `false` (default) keeps the established arithmetic-coded wire form;
    /// `true` emits the spec-exact §IV coder and sets the segment header's
    /// entropy-backend flag so the decoder dispatches the matching
    /// backend. No effect on the uncompressed §III.D path.
    pub interleaved_entropy: bool,

    /// §V.B **transform-domain** segmentation (IPN 42-155 §V.B + §V.D).
    ///
    /// When `true` (compressed path only), the wavelet transform runs
    /// over the **whole image** once, the LL subband is partitioned into
    /// `segment_count` nearly-square rectangles by the §V.D algorithm,
    /// and the partition is mapped to every subband (§V.B: pixels in
    /// different subbands corresponding to the same spatial location
    /// belong to the same segment — see
    /// [`crate::partition::coefficient_segment_map`]). Each segment is
    /// then bit-plane-coded independently with its own context modeler
    /// and entropy coder, exactly the §V.B error-containment contract.
    ///
    /// Versus the historical row-strip convention (`false`, the
    /// default) this eliminates the strip-boundary artifacts §V.B
    /// warns about ("by segmenting the image in the transform domain,
    /// we can virtually guarantee that such artifacts will not occur")
    /// and achieves better decorrelation because the transform sees the
    /// whole image; a lost segment degrades to a smooth low-detail
    /// patch (zero coefficients through the shared inverse transform)
    /// instead of a flat-128 strip.
    ///
    /// The §V.D eq (9) validity condition applies: `segment_count`
    /// must not exceed the LL subband's pixel count
    /// (`ceil(W/2^D) * ceil(H/2^D)`); the wire form caps the count at
    /// 255. Composes with `byte_budget` / `target_bytes` /
    /// `segment_priorities` / `min_loss` and both entropy backends;
    /// `rd_pruning` and `auto_uncompressed_fallback` remain row-strip
    /// features and return `Unsupported` when combined.
    pub transform_segments: bool,

    /// §VI.A *minimum loss* quality-goal parameter `M`.
    ///
    /// "The minimum loss parameter is a nonnegative integer that
    /// determines a minimum number of bit planes that will not be
    /// encoded in each subband" (§VI.A): a subband with Fig. 18
    /// relative-importance offset `o` never encodes its
    /// `max(0, M - o)` least-significant magnitude bit planes, no
    /// matter how large the byte quota is. `M = 0` (the default) is
    /// lossless when the byte quota allows; each increment excludes one
    /// more plane of the level-1 HH subband and correspondingly fewer
    /// planes of the more important subbands (offsets
    /// `HH_j = j - 1`, `HL_j = LH_j = j`, `LL = D + 1`; see
    /// [`crate::priority::min_loss_skip_map`]). Carried on the wire in
    /// every packet header so the decoder applies the identical
    /// exclusion. Composes with the byte quota: "ICER stops producing
    /// compressed bytes once the quality goal or byte quota is met,
    /// whichever comes first" (§VI). Applies to the compressed path
    /// only (the §III.D uncompressed path ships raw pixels).
    pub min_loss: u8,

    /// IPN 42-155 §III.A **subband-priority interleaving**.
    ///
    /// When `true` (compressed path only), each segment's packets follow
    /// the spec's progressive order instead of the crate's historical
    /// whole-strip MSB-down packet pairs: subband bit planes are walked
    /// in decreasing §III.A priority (Fig. 7 weight, halved per plane;
    /// ties to the higher decomposition level, then `LL, HL, LH, HH`),
    /// each subband bit plane is coded in a **single combined raster
    /// pass** over the subband (§III: significance and refinement bits
    /// pixel-interleaved, sign immediately after the first nonzero
    /// magnitude bit), and one packet is cut per priority group so a
    /// byte-quota truncation always lands on a §III.A priority boundary.
    /// The context model persists across the segment (§III: estimates
    /// rely on "previously encoded information from the same segment").
    ///
    /// This is what makes a truncated stream "the best image achievable
    /// for that many bytes" (§III.A): under the historical whole-strip
    /// plane order a truncation spends bytes on low-weight level-1 HH
    /// bits before high-weight LL planes; the §III.A order defers them.
    ///
    /// The mode is recorded per segment in a previously-reserved header
    /// bit ([`crate::header::SegmentHeader::priority_interleaved`]), so
    /// every pre-existing stream decodes unchanged and the decoder
    /// dispatches on the wire flag with no caller-side awareness.
    ///
    /// Compose-rules: composes with both entropy backends, row-strip
    /// and §V.B transform-domain segmentation, `min_loss` (whole
    /// subband bit planes are dropped from the schedule), the byte
    /// quota / soft target, ROI segment priorities, colour, the §III.D
    /// uncompressed fallback, and `quality_target_psnr`. Mutually
    /// exclusive with `rd_pruning` (two different packet schedulers;
    /// the §III.A order already places every packet at its
    /// distortion-priority position). Ignored when
    /// [`Self::uncompressed`] forces the raw-pixel path.
    pub priority_interleaving: bool,
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
            auto_uncompressed_fallback: false,
            quality_target_psnr: None,
            interleaved_entropy: false,
            transform_segments: false,
            min_loss: 0,
            priority_interleaving: false,
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

    /// Enable per-segment automatic uncompressed fallback (IPN 42-155
    /// §III.D "Performance with Difficult Imagery"). See
    /// [`EncodeOptions::auto_uncompressed_fallback`] for the full
    /// contract.
    ///
    /// Only meaningful with the compressed path — calling this on an
    /// `EncodeOptions::default()` (which forces uncompressed) is
    /// harmless but has no observable effect: the segment is already
    /// emitted via the uncompressed path. The intended usage is
    /// `EncodeOptions::compressed().with_uncompressed_fallback()`.
    #[must_use]
    pub fn with_uncompressed_fallback(mut self) -> Self {
        self.auto_uncompressed_fallback = true;
        self
    }

    /// Enable quality-target rate-control (round 233). See
    /// [`EncodeOptions::quality_target_psnr`] for the full contract.
    ///
    /// `target_db` is the minimum acceptable PSNR (dB) of the decoded
    /// reconstruction relative to the input. The encoder runs a binary
    /// search over byte-budget values and returns the smallest output
    /// whose decoded PSNR is greater than or equal to `target_db`.
    ///
    /// Mutually exclusive with [`Self::with_byte_budget`] /
    /// [`Self::with_target_bytes`] / [`Self::with_rd_budget`] (the
    /// search manages the byte budget directly).
    #[must_use]
    pub fn with_quality_target(mut self, target_db: f32) -> Self {
        self.quality_target_psnr = Some(target_db);
        self
    }

    /// Code the compressed segments with ICER's §IV interleaved entropy
    /// coder instead of the binary arithmetic coder. See
    /// [`EncodeOptions::interleaved_entropy`].
    #[must_use]
    pub fn with_interleaved_entropy(mut self) -> Self {
        self.interleaved_entropy = true;
        self
    }

    /// Segment the image in the wavelet **transform domain** per IPN
    /// 42-155 §V.B, using the §V.D LL-subband partitioning algorithm.
    /// See [`EncodeOptions::transform_segments`] for the full contract.
    #[must_use]
    pub fn with_transform_domain_segments(mut self) -> Self {
        self.transform_segments = true;
        self
    }

    /// Enable IPN 42-155 §III.A subband-priority interleaving: packets
    /// follow the spec's cross-subband progressive order (one packet
    /// per §III.A priority group, combined single-raster-pass subband
    /// bit planes) instead of the historical whole-strip MSB-down packet
    /// pairs. See [`EncodeOptions::priority_interleaving`] for the full
    /// contract. Recorded on the wire per segment; pre-existing streams
    /// decode unchanged.
    #[must_use]
    pub fn with_priority_interleaving(mut self) -> Self {
        self.priority_interleaving = true;
        self
    }

    /// Set the §VI.A *minimum loss* quality-goal parameter. See
    /// [`EncodeOptions::min_loss`] for the full contract; `0` keeps the
    /// lossless-capable default.
    #[must_use]
    pub fn with_min_loss(mut self, m: u8) -> Self {
        self.min_loss = m;
        self
    }

    /// The entropy backend this option set selects.
    pub(crate) fn entropy_kind(&self) -> crate::entropy::EntropyKind {
        if self.interleaved_entropy {
            crate::entropy::EntropyKind::Interleaved
        } else {
            crate::entropy::EntropyKind::Arithmetic
        }
    }
}

/// Encode `image` into the on-the-wire ICER byte stream. Single or
/// multiple segments depending on `opts.segment_count`.
///
/// # Colour images
///
/// A single-plane [`IcerPixelFormat::Gray8`] image produces a bare
/// single-plane ICER bitstream (the historical wire form — byte-for-byte
/// unchanged). A colour [`IcerPixelFormat::Yuv444P`] image is encoded as
/// three **independent** single-plane ICER bitstreams (IPN 42-155 §III
/// describes ICER as a single-component coder whose deployed colour scheme
/// runs one ICER instance per component), concatenated behind the small
/// [`crate::plane_container`] header. The decoder
/// ([`crate::decoder::parse_icer`]) dispatches on the leading sentinel.
pub fn encode_icer(image: &IcerImage, opts: &EncodeOptions) -> Result<Vec<u8>> {
    match image.pixel_format {
        IcerPixelFormat::Gray8 => encode_icer_single_plane(image, opts),
        IcerPixelFormat::Yuv444P => encode_icer_multi_plane(image, opts),
    }
}

/// Encode a multi-plane (colour) image as N independent single-plane ICER
/// bitstreams wrapped in a [`crate::plane_container`] header.
///
/// Each plane is run through the unchanged single-plane encoder with the
/// same [`EncodeOptions`]; the per-plane streams are then concatenated
/// behind the container header. This mirrors the deployed §III colour
/// scheme: independent ICER instances sharing outer image metadata.
fn encode_icer_multi_plane(image: &IcerImage, opts: &EncodeOptions) -> Result<Vec<u8>> {
    let n = image.pixel_format.plane_count();
    if image.planes.len() != n {
        return Err(IcerError::invalid(format!(
            "image declares {:?} ({n} planes) but carries {}",
            image.pixel_format,
            image.planes.len()
        )));
    }
    let mut plane_streams: Vec<Vec<u8>> = Vec::with_capacity(n);
    for plane in &image.planes {
        // Build a transient single-plane Gray8 view so the existing
        // single-plane encoder (and its auto-filter / quality-target
        // analysis, which take an `IcerImage`) operate per component.
        let plane_image = IcerImage {
            width: image.width,
            height: image.height,
            pixel_format: IcerPixelFormat::Gray8,
            planes: vec![plane.clone()],
            pts: image.pts,
        };
        plane_streams.push(encode_icer_single_plane(&plane_image, opts)?);
    }
    crate::plane_container::encode_container(image.pixel_format, &plane_streams)
}

/// Encode a single-plane (Gray8) image. This is the historical
/// `encode_icer` body; the wire form it produces is unchanged.
fn encode_icer_single_plane(image: &IcerImage, opts: &EncodeOptions) -> Result<Vec<u8>> {
    if image.pixel_format != IcerPixelFormat::Gray8 {
        return Err(IcerError::Unsupported(
            "single-plane encoder requires Gray8".into(),
        ));
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

    // Round 233: quality-target rate-control. Run a binary search over
    // byte-budget values, encode + decode each trial, compute PSNR,
    // and emit the smallest output meeting the target. The search is
    // implemented in `crate::analyze` so the encoder's main flow stays
    // focused on the single-shot path.
    if let Some(target_db) = opts.quality_target_psnr {
        if !opts.uncompressed {
            // Reject conflicting options up-front: byte_budget /
            // target_bytes / rd_pruning all conflict with the quality
            // search managing the budget itself.
            if opts.byte_budget.is_some() {
                return Err(IcerError::Unsupported(
                    "quality_target_psnr conflicts with byte_budget; pick one".into(),
                ));
            }
            if opts.target_bytes.is_some() {
                return Err(IcerError::Unsupported(
                    "quality_target_psnr conflicts with target_bytes; pick one".into(),
                ));
            }
            if opts.rd_pruning {
                return Err(IcerError::Unsupported(
                    "quality_target_psnr conflicts with rd_pruning; pick one".into(),
                ));
            }
            return crate::analyze::encode_to_quality_target(image, opts, target_db);
        }
        // Uncompressed-forced: the round-trip is bit-exact by
        // construction, every finite PSNR target is satisfied
        // trivially. Fall through to the regular encode path.
    }

    let segment_count = opts.segment_count.max(1);
    let levels = opts.wavelet_levels.clamp(1, 6);

    // §VI.A minimum loss composes with the quota paths but not with the
    // row-strip R-D packet selector (whose ΔD model assumes full-plane
    // packets); §III.D raw pixels have no bit planes to exclude.
    if opts.min_loss > 0 {
        if opts.uncompressed {
            return Err(IcerError::Unsupported(
                "min_loss applies to the compressed path only (§VI.A)".into(),
            ));
        }
        if opts.rd_pruning {
            return Err(IcerError::Unsupported(
                "min_loss + rd_pruning is unsupported; pick one rate-control mode".into(),
            ));
        }
    }

    // §III.A subband-priority interleaving is a packet *scheduler*; the
    // R-D selector is a competing one (its ΔD model and dependency
    // graph assume the whole-strip MSB-down packet pairs).
    if opts.priority_interleaving && opts.rd_pruning {
        return Err(IcerError::Unsupported(
            "priority_interleaving + rd_pruning is unsupported; the §III.A order \
             already schedules packets by distortion priority"
                .into(),
        ));
    }

    // §V.B transform-domain segmentation: whole-image DWT, §V.D LL
    // partition mapped to every subband, one independently-coded
    // segment per partition rectangle.
    if opts.transform_segments {
        if opts.uncompressed {
            return Err(IcerError::Unsupported(
                "transform-domain segmentation requires the compressed path (§V.B)".into(),
            ));
        }
        if opts.rd_pruning || opts.auto_uncompressed_fallback {
            return Err(IcerError::Unsupported(
                "rd_pruning / uncompressed fallback are row-strip features; \
                 not combinable with §V.B transform-domain segmentation"
                    .into(),
            ));
        }
        return encode_transform_segmented(plane, w, h, opts, levels);
    }

    if segment_count == 1 {
        let bytes = encode_one_segment(plane, w, 0, h, 0, opts, levels)?;
        if let Some(budget) = opts.byte_budget {
            if bytes.len() as u64 > budget {
                // The lone segment cannot fit the byte budget even after
                // its own internal truncation — the §III.D uncompressed
                // path is all-or-nothing (a partial raw body would not
                // decode). Mirror the multi-segment skip semantics: emit
                // the zero-body placeholder header so the stream still
                // frames the full image geometry (flat-128 strip on
                // decode) instead of blowing through the advertised hard
                // cap (found by the scheduled encode_roundtrip fuzz run:
                // a forced-uncompressed single-segment encode previously
                // ignored `byte_budget` entirely).
                let mut out = Vec::new();
                emit_skipped_placeholders(&mut out, &[false], &[(0, h)], w, levels, opts);
                return Ok(out);
            }
        }
        return Ok(bytes);
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
    let emission_order: Vec<u16> = resolve_emission_order(opts, n_segs)?;

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
        //
        // When a segment does not fit the remaining budget we MUST NOT
        // drop it silently: the strict decoder reconstructs the total
        // image height by summing the heights of the segments present
        // on the wire, so a missing trailing strip would shrink the
        // decoded image (e.g. a 64-row 16-segment image truncated to a
        // single 4-row strip). Instead we emit a zero-body placeholder
        // header for every skipped strip — identical to the ROI-priority
        // second pass below — so the decoder accounts for the strip at
        // the correct y-offset (reconstructed as the flat-128 §V.B
        // placeholder) and the contiguous segment_index sequence the
        // decoder requires is preserved.
        let mut kept = vec![false; n_segs];
        for &seg_idx in &emission_order {
            let (y_start, this_h) = starts_heights[seg_idx as usize];

            // Reserve a placeholder header for every later strip that has
            // not yet been written, so that committing this segment never
            // starves the budget needed to frame the rest of the image.
            let inner_opts = if let Some(budget) = opts.byte_budget {
                if out.len() as u64 >= budget {
                    break;
                }
                let reserve =
                    (n_segs - 1 - seg_idx as usize) as u64 * (SegmentHeader::ENCODED_BYTES as u64);
                let remaining = budget
                    .saturating_sub(out.len() as u64)
                    .saturating_sub(reserve);
                let mut o = opts.clone();
                o.byte_budget = Some(remaining);
                o
            } else {
                opts.clone()
            };
            let bytes =
                encode_one_segment(plane, w, y_start, this_h, seg_idx, &inner_opts, levels)?;
            if let Some(budget) = opts.byte_budget {
                let reserve =
                    (n_segs - 1 - seg_idx as usize) as u64 * (SegmentHeader::ENCODED_BYTES as u64);
                if out.len() as u64 + bytes.len() as u64 + reserve > budget {
                    // Skip this strip; a placeholder is emitted below.
                    continue;
                }
            }
            out.extend_from_slice(&bytes);
            kept[seg_idx as usize] = true;
        }
        emit_skipped_placeholders(&mut out, &kept, &starts_heights, w, levels, opts);
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
    // skipped, in segment_index order.
    emit_skipped_placeholders(&mut out, &kept, &starts_heights, w, levels, opts);
    Ok(out)
}

/// Resolve the ROI emission order (round 6): identity when no
/// [`EncodeOptions::segment_priorities`] vector is supplied, else the
/// validated inverse permutation (ascending rank). Shared by the
/// row-strip and §V.B transform-domain paths.
fn resolve_emission_order(opts: &EncodeOptions, n_segs: usize) -> Result<Vec<u16>> {
    match &opts.segment_priorities {
        None => Ok((0..n_segs as u16).collect()),
        Some(prios) => {
            if prios.len() != n_segs {
                return Err(IcerError::Unsupported(format!(
                    "segment_priorities length {} != segment_count {}",
                    prios.len(),
                    n_segs
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
            Ok(position_of_rank)
        }
    }
}

/// §V.B transform-domain segmented encode (IPN 42-155 §V.B + §V.D).
///
/// One whole-image forward DWT, then the §V.D partition of the LL
/// subband is mapped to every subband
/// ([`crate::partition::coefficient_segment_map`]) and each segment's
/// coefficients are bit-plane coded independently — separate context
/// modeler and entropy coder per segment per §V.B. Segments are
/// emitted whole in [`resolve_emission_order`] order under the byte
/// budget; a segment that does not fit is emitted as a zero-body
/// placeholder header so the stream always frames the full geometry
/// (the decoder reconstructs it as zero coefficients — a smooth
/// low-detail patch through the shared inverse transform, the §V.B
/// containment behaviour).
fn encode_transform_segmented(
    plane: &IcerPlane,
    w: usize,
    h: usize,
    opts: &EncodeOptions,
    levels: u8,
) -> Result<Vec<u8>> {
    if w < 2 || h < 2 {
        return Err(IcerError::Unsupported(format!(
            "compressed encode requires width >= 2 and height >= 2; got {w}x{h}"
        )));
    }
    let n_segs = opts.segment_count.max(1) as usize;
    if n_segs > u8::MAX as usize {
        return Err(IcerError::Unsupported(format!(
            "transform-domain segment count {n_segs} exceeds the wire cap of 255 (§V.D)"
        )));
    }
    // §V.D eq (9) validity (s <= LL pixel count) is enforced inside the
    // map constructor.
    let seg_map = crate::partition::coefficient_segment_map(w, h, levels, n_segs)?;
    // Per-segment bounding windows: the §V.B block is contiguous in the
    // interleaved buffer, so each segment's scan is confined to its own
    // rectangle instead of walking the whole image (byte-identical
    // output; the stripe grid stays y=0-aligned).
    let (w_ll, h_ll) = crate::partition::ll_dimensions(w, h, levels);
    let rects = crate::partition::partition(w_ll, h_ll, n_segs)?;

    // §III.A level shift + one whole-image forward DWT.
    let mut coeffs: Vec<i32> = Vec::with_capacity(w * h);
    for y in 0..h {
        let row = &plane.data[y * plane.stride..y * plane.stride + w];
        for &px in row {
            coeffs.push(px as i32 - 128);
        }
    }
    wavelet_int::forward_2d_dyadic(&mut coeffs, w, h, levels, opts.filter);

    // §VI.A minimum-loss plane exclusion (whole-image map; identical
    // for every segment).
    let skip_map: Option<Vec<u8>> = (opts.min_loss > 0)
        .then(|| crate::priority::min_loss_skip_map(w, h, levels, opts.min_loss));

    let emission_order = resolve_emission_order(opts, n_segs)?;
    let mut out = Vec::new();
    let mut kept = vec![false; n_segs];
    for &seg_idx in &emission_order {
        // Reserve a placeholder header for every other undecided
        // segment so committing this one never starves the budget
        // needed to frame the rest of the image.
        let inner_budget = match opts.byte_budget {
            Some(budget) => {
                let not_decided = (0..n_segs)
                    .filter(|&i| !kept[i] && i != seg_idx as usize)
                    .count() as u64;
                let reserve = not_decided * (SegmentHeader::ENCODED_BYTES as u64);
                Some(
                    budget
                        .saturating_sub(out.len() as u64)
                        .saturating_sub(reserve),
                )
            }
            None => None,
        };
        let bytes = encode_one_transform_segment(
            &coeffs,
            &seg_map,
            skip_map.as_deref(),
            rects[seg_idx as usize].image_window(levels, w, h),
            w,
            h,
            seg_idx,
            n_segs as u8,
            opts,
            levels,
            inner_budget,
        )?;
        if let Some(budget) = inner_budget {
            if bytes.len() as u64 > budget {
                // Even the truncated form does not fit; placeholder.
                continue;
            }
        }
        out.extend_from_slice(&bytes);
        kept[seg_idx as usize] = true;
    }
    // Placeholder headers for the skipped segments, in index order.
    for (seg_idx, &was_kept) in kept.iter().enumerate() {
        if was_kept {
            continue;
        }
        let placeholder = SegmentHeader {
            sync_prefix: opts.sync_prefix,
            filter: opts.filter,
            decomp_levels: levels,
            uncompressed: false,
            width: w as u16,
            height: h as u16,
            bit_plane_count: opts.bit_plane_count.clamp(1, 32),
            interleaved_entropy: opts.interleaved_entropy,
            transform_segmented: true,
            total_segments: n_segs as u8,
            priority_interleaved: opts.priority_interleaving,
            segment_length: 0,
            segment_index: seg_idx as u16,
        };
        out.extend_from_slice(&placeholder.encode());
    }
    Ok(out)
}

/// Encode one §V.B transform-domain segment: the coefficients the §V.D
/// partition maps to `seg_idx`, bit-plane coded with a fresh context
/// modeler + entropy coder ("ICER maintains separate context modeler
/// and entropy coder data for each segment", §V.B), serialised MSB-down
/// under the hard cap / soft target quota semantics.
#[allow(clippy::too_many_arguments)]
fn encode_one_transform_segment(
    coeffs: &[i32],
    seg_map: &[u16],
    skip_map: Option<&[u8]>,
    window: (usize, usize, usize, usize),
    w: usize,
    h: usize,
    seg_idx: u16,
    total_segments: u8,
    opts: &EncodeOptions,
    levels: u8,
    byte_budget: Option<u64>,
) -> Result<Vec<u8>> {
    let filter = ScanFilter {
        segment: Some((seg_map, seg_idx)),
        skip: skip_map,
        window: Some(window),
    };
    // Bit-plane count sized to this segment's own dynamic range (the
    // §V.B block is exactly the window rectangle).
    let (wx0, wx1, wy0, wy1) = window;
    let max_abs = (wy0..wy1)
        .flat_map(|y| (wx0..wx1).map(move |x| y * w + x))
        .map(|i| coeffs[i].unsigned_abs())
        .max()
        .unwrap_or(0);
    let needed = if max_abs == 0 {
        1
    } else {
        (32 - max_abs.leading_zeros()).clamp(1, 31) as u8
    };
    let q = needed.max(opts.bit_plane_count.min(31)).min(31);

    let bp_input = BitPlaneInput {
        coeffs,
        width: w,
        height: h,
        q,
        levels,
    };
    let body = if opts.priority_interleaving {
        // §III.A priority schedule over this §V.B segment's coefficients
        // (segment mask + window; min_loss drops whole subband bit
        // planes from the schedule, no per-coefficient skip map).
        let priority_filter = ScanFilter {
            segment: Some((seg_map, seg_idx)),
            skip: None,
            window: Some(window),
        };
        let packets = crate::bitplane::encode_bitplanes_prioritized(
            &bp_input,
            opts.entropy_kind(),
            &priority_filter,
            opts.min_loss,
        )?;
        serialize_priority_packets(&packets, opts, byte_budget)?
    } else {
        let packets =
            crate::bitplane::encode_bitplanes_filtered(&bp_input, opts.entropy_kind(), &filter)?;
        serialize_packets_msb_down(
            &packets,
            q,
            opts,
            byte_budget,
            min_visited_skip(&filter, w * h),
        )?
    };

    let segment = SegmentHeader {
        sync_prefix: opts.sync_prefix,
        filter: opts.filter,
        decomp_levels: levels,
        uncompressed: false,
        width: w as u16,
        height: h as u16,
        bit_plane_count: q,
        interleaved_entropy: opts.interleaved_entropy,
        transform_segmented: true,
        total_segments,
        priority_interleaved: opts.priority_interleaving,
        segment_length: body.len() as u16,
        segment_index: seg_idx,
    };
    let mut out = Vec::with_capacity(SegmentHeader::ENCODED_BYTES + body.len());
    out.extend_from_slice(&segment.encode());
    out.extend_from_slice(&body);
    Ok(out)
}

/// The smallest §VI.A skip value over the coefficients this filter
/// visits — every magnitude bit plane strictly below it codes nothing,
/// so the emitter drops those planes' packets entirely (the byte
/// saving the §VI.A quality goal exists to deliver). Returns 0 when no
/// skip map is active.
fn min_visited_skip(filter: &ScanFilter<'_>, n: usize) -> u8 {
    let Some(skip) = filter.skip else {
        return 0;
    };
    let mut min = u8::MAX;
    for i in 0..n {
        let in_seg = match filter.segment {
            Some((map, seg)) => map[i] == seg,
            None => true,
        };
        if in_seg {
            min = min.min(skip[i]);
            if min == 0 {
                return 0;
            }
        }
    }
    if min == u8::MAX {
        0
    } else {
        min
    }
}

/// Serialise per-bit-plane packets MSB-down (sig before ref, §IV order)
/// under the hard cap / soft target quota semantics, dropping the
/// trailing planes the §VI.A `floor_planes` exclusion leaves empty.
/// Returns the segment body (packet headers + bodies).
fn serialize_packets_msb_down(
    packets: &[EncodedPacket],
    q: u8,
    opts: &EncodeOptions,
    byte_budget: Option<u64>,
    floor_planes: u8,
) -> Result<Vec<u8>> {
    let seg_hdr_bytes = SegmentHeader::ENCODED_BYTES as u64;
    let mut body: Vec<u8> = Vec::new();
    let mut soft_stop_after_bp: Option<u8> = None;
    'bp_loop: for bp_idx in 0..q as usize {
        if let Some(stop_bp) = soft_stop_after_bp {
            if bp_idx as u8 > stop_bp {
                break 'bp_loop;
            }
        }
        // Absolute magnitude bit position of this plane; planes fully
        // below the §VI.A floor code nothing and are not emitted.
        let abs_bit = q as usize - 1 - bp_idx;
        if abs_bit < floor_planes as usize {
            break 'bp_loop;
        }
        for &is_sig in &[true, false] {
            let Some(pkt) = packets
                .iter()
                .find(|p| p.bit_plane == bp_idx as u8 && p.is_significance == is_sig)
            else {
                continue;
            };
            if pkt.body.len() > u16::MAX as usize {
                return Err(IcerError::Unsupported(format!(
                    "packet body {} exceeds u16 limit",
                    pkt.body.len()
                )));
            }
            let pkt_wire_len = PacketHeader::ENCODED_BYTES as u64 + pkt.body.len() as u64;
            if let Some(budget) = byte_budget {
                if seg_hdr_bytes + body.len() as u64 + pkt_wire_len > budget {
                    break 'bp_loop;
                }
            }
            let ph = PacketHeader {
                bit_plane: pkt.bit_plane,
                pass: if pkt.is_significance {
                    BitPlanePass::Significance
                } else {
                    BitPlanePass::Refinement
                },
                body_length: pkt.body.len() as u16,
                min_loss: opts.min_loss,
            };
            body.extend_from_slice(&ph.encode());
            body.extend_from_slice(&pkt.body);
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
    Ok(body)
}

/// Serialise §III.A priority-interleaved packets in schedule order
/// under the hard cap / soft target quota semantics. Packets are
/// already in decreasing-priority order (one per priority group), so
/// the quota cut is simply a prefix — exactly the §IV.B "compression
/// can be terminated once the byte quota is met" behaviour, landing on
/// a §III.A priority boundary. The soft target finishes the in-progress
/// packet, then stops. Returns the segment body (packet headers +
/// bodies); each packet header carries the group index in `bit_plane`,
/// the `Cleanup` pass id (the pass is combined), and the replicated
/// §VI.A `min_loss`.
fn serialize_priority_packets(
    packets: &[EncodedPacket],
    opts: &EncodeOptions,
    byte_budget: Option<u64>,
) -> Result<Vec<u8>> {
    let seg_hdr_bytes = SegmentHeader::ENCODED_BYTES as u64;
    let mut body: Vec<u8> = Vec::new();
    for pkt in packets {
        if pkt.body.len() > u16::MAX as usize {
            return Err(IcerError::Unsupported(format!(
                "packet body {} exceeds u16 limit",
                pkt.body.len()
            )));
        }
        let pkt_wire_len = PacketHeader::ENCODED_BYTES as u64 + pkt.body.len() as u64;
        if let Some(budget) = byte_budget {
            if seg_hdr_bytes + body.len() as u64 + pkt_wire_len > budget {
                break;
            }
        }
        let ph = PacketHeader {
            bit_plane: pkt.bit_plane,
            pass: BitPlanePass::Cleanup,
            body_length: pkt.body.len() as u16,
            min_loss: opts.min_loss,
        };
        body.extend_from_slice(&ph.encode());
        body.extend_from_slice(&pkt.body);
        if let Some(target) = opts.target_bytes {
            if seg_hdr_bytes + body.len() as u64 >= target {
                break;
            }
        }
    }
    if body.len() > u16::MAX as usize {
        return Err(IcerError::Unsupported(format!(
            "compressed segment body {} exceeds u16 limit",
            body.len()
        )));
    }
    Ok(body)
}

/// Append a zero-body placeholder segment header for every strip whose
/// `kept[seg_idx]` flag is `false`, in ascending `segment_index` order.
///
/// A placeholder carries the strip's width + height (so the decoder knows
/// the row offset) with `segment_length = 0` and no packets, so the
/// decoder reconstructs the strip as the flat-128 §V.B placeholder while
/// still accounting for its rows in the total image height. This is shared
/// by both the legacy index-order budget path and the ROI-priority path so
/// a budget-truncated stream always frames the full image geometry.
fn emit_skipped_placeholders(
    out: &mut Vec<u8>,
    kept: &[bool],
    starts_heights: &[(usize, usize)],
    w: usize,
    levels: u8,
    opts: &EncodeOptions,
) {
    for (seg_idx, &was_kept) in kept.iter().enumerate() {
        if was_kept {
            continue;
        }
        let (_y_start, this_h) = starts_heights[seg_idx];
        // A placeholder uses bit_plane_count clamped into range so the
        // field validates, and the compressed flag matches the rest of
        // the segments.
        let placeholder = SegmentHeader {
            sync_prefix: opts.sync_prefix,
            filter: opts.filter,
            decomp_levels: levels.clamp(1, 6),
            uncompressed: opts.uncompressed,
            width: w as u16,
            height: this_h as u16,
            bit_plane_count: opts.bit_plane_count.clamp(1, 32),
            interleaved_entropy: opts.interleaved_entropy && !opts.uncompressed,
            transform_segmented: false,
            total_segments: 0,
            priority_interleaved: opts.priority_interleaving && !opts.uncompressed,
            segment_length: 0,
            segment_index: seg_idx as u16,
        };
        out.extend_from_slice(&placeholder.encode());
    }
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
        return encode_one_segment_uncompressed(
            plane,
            img_w,
            y_start,
            strip_h,
            segment_index,
            opts,
        );
    }

    // Compressed path. Optionally also produce the uncompressed
    // candidate and pick the smaller (IPN 42-155 §III.D "Performance
    // with Difficult Imagery"). Per-segment decision: the wire-format
    // `uncompressed` flag on the segment header records which path
    // was emitted, so the decoder reconstructs each segment via its
    // own flag without any caller-side awareness.
    let compressed =
        encode_one_segment_compressed(plane, img_w, y_start, strip_h, segment_index, opts, levels)?;

    if !opts.auto_uncompressed_fallback {
        return Ok(compressed);
    }

    // Uncompressed candidate. The wire format caps each segment's body
    // length at u16::MAX bytes (§IV), so strips with more than 65535
    // pixels can't be shipped uncompressed and the compressed result
    // is kept unconditionally.
    if img_w.saturating_mul(strip_h) > u16::MAX as usize {
        return Ok(compressed);
    }
    // The uncompressed encoder needs `opts.uncompressed = true` to set
    // the wire-format flag correctly; clone + flip on a local copy so
    // the caller's options stay untouched.
    let mut uncompressed_opts = opts.clone();
    uncompressed_opts.uncompressed = true;
    match encode_one_segment_uncompressed(
        plane,
        img_w,
        y_start,
        strip_h,
        segment_index,
        &uncompressed_opts,
    ) {
        Ok(uncompressed) if uncompressed.len() < compressed.len() => Ok(uncompressed),
        // Equal-length tie goes to the compressed path: the entropy
        // coder's per-bit-plane progressive structure is strictly more
        // useful to a truncating decoder than the uncompressed dump.
        _ => Ok(compressed),
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
        min_loss: 0,
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
    // Forward DWT -- the §II.A reversible integer transform for the
    // selected Table 1 filter.
    wavelet_int::forward_2d_dyadic(&mut coeffs, img_w, strip_h, levels, opts.filter);
    // Pick bit-plane count to fit the largest |coeff|, but never less
    // than the caller-requested floor.
    let needed = select_bit_plane_count(&coeffs);
    let q = needed.max(opts.bit_plane_count.min(31)).min(31);

    // §III.A subband-priority interleaving: the packet schedule is the
    // spec's cross-subband priority order (one packet per priority
    // group), coded with the combined single-raster-pass subband scan.
    // min_loss drops whole subband bit planes from the schedule, so no
    // per-coefficient skip map is needed on this path.
    if opts.priority_interleaving {
        let bp_input = BitPlaneInput {
            coeffs: &coeffs,
            width: img_w,
            height: strip_h,
            q,
            levels,
        };
        let packets = crate::bitplane::encode_bitplanes_prioritized(
            &bp_input,
            opts.entropy_kind(),
            &ScanFilter::ALL,
            opts.min_loss,
        )?;
        let body = serialize_priority_packets(&packets, opts, opts.byte_budget)?;
        let mut opts_copy = opts.clone();
        opts_copy.bit_plane_count = q;
        return emit_segment_header_and_body(
            &body,
            segment_index,
            img_w,
            strip_h,
            &opts_copy,
            false,
        );
    }

    // Encode as per-bit-plane packets (IPN 42-155 §IV multi-packet
    // ordering). When rate-distortion pruning is active, weight each
    // packet's distortion estimate by the §III.A per-coefficient
    // image-domain weight map so the selector optimises
    // reconstructed-image MSE rather than transform-domain MSE (the two
    // differ because ICER's wavelet transforms are not unitary). The
    // weight map is only needed for the R-D path; the strict-MSB and
    // unbudgeted paths ignore `delta_distortion` entirely, so we skip the
    // (one inverse-DWT-per-subband-class) probe cost there.
    let bp_input = BitPlaneInput {
        coeffs: &coeffs,
        width: img_w,
        height: strip_h,
        q,
        levels,
    };
    let rd_weights: Option<Vec<f64>> = if opts.rd_pruning {
        Some(crate::priority::subband_weight_map(
            img_w,
            strip_h,
            levels,
            opts.filter,
        ))
    } else {
        None
    };
    // §VI.A minimum-loss plane exclusion for this strip (None when
    // M = 0 — the scanner is then bit-identical to the unfiltered one).
    let skip_map: Option<Vec<u8>> = (opts.min_loss > 0)
        .then(|| crate::priority::min_loss_skip_map(img_w, strip_h, levels, opts.min_loss));
    let scan_filter = ScanFilter {
        segment: None,
        skip: skip_map.as_deref(),
        window: None,
    };
    let packets = if let Some(weights) = &rd_weights {
        // R-D weighting is only computed for the arithmetic backend's
        // packet-selection path; the interleaved backend (a distinct wire
        // form) is not combined with R-D pruning here. (min_loss +
        // rd_pruning is rejected up-front, so no filter is needed.)
        crate::bitplane::encode_bitplanes_weighted(&bp_input, Some(weights))?
    } else {
        crate::bitplane::encode_bitplanes_filtered(&bp_input, opts.entropy_kind(), &scan_filter)?
    };

    // Round 91: rate-distortion-driven packet selection (IPN 42-155
    // §IV.B rate-allocation principle). When enabled, the packet
    // *set* is chosen by greedy ΔD/byte ranking before serialisation,
    // rather than walked MSB-down with a strict truncation cut-off.
    // See `select_packets_by_rd` below for the algorithm.
    //
    // The greedy ΔD/byte plan is then validated against the plain
    // strict-MSB prefix plan by *actual* decoded distortion (decode each
    // candidate, measure the §III.A image-domain weighted MSE against the
    // pre-truncation coefficients) and the lower-distortion plan wins.
    // The ΔD estimate is approximate, so without this guard the greedy
    // plan can decode worse than strict on some content; the guard makes
    // the R-D mode provably never worse than strict-MSB truncation.
    let kept_mask: Option<Vec<bool>> = match (opts.rd_pruning, opts.byte_budget) {
        (true, Some(budget)) => {
            let seg_hdr_bytes = SegmentHeader::ENCODED_BYTES as u64;
            let body_budget = budget.saturating_sub(seg_hdr_bytes);
            let greedy = select_packets_by_rd(&packets, q, body_budget);
            let strict = strict_msb_prefix_mask(&packets, q, body_budget);
            // Original strip pixels (level-shifted to match the decoder's
            // pre-inverse-shift domain) so the candidate comparison can
            // measure *exact* reconstructed-image MSE.
            let mut orig: Vec<i32> = Vec::with_capacity(img_w * strip_h);
            for y in 0..strip_h {
                let src_y = y_start + y;
                let row = &plane.data[src_y * plane.stride..src_y * plane.stride + img_w];
                for &px in row {
                    orig.push(px as i32 - 128);
                }
            }
            Some(pick_lower_distortion_mask(
                &packets,
                q,
                img_w,
                strip_h,
                levels,
                opts.filter,
                &orig,
                greedy,
                strict,
            ))
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
    // §VI.A: every magnitude bit plane strictly below the smallest
    // per-subband skip codes nothing; drop those packets entirely.
    let floor_planes = min_visited_skip(&scan_filter, img_w * strip_h) as usize;
    'bp_loop: for bp_idx in 0..q_usize {
        // Check if soft-target stop was requested for a previous bit-plane.
        if let Some(stop_bp) = soft_stop_after_bp {
            if bp_idx as u8 > stop_bp {
                break 'bp_loop;
            }
        }
        if q_usize - 1 - bp_idx < floor_planes {
            break 'bp_loop;
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
                min_loss: opts.min_loss,
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
        interleaved_entropy: opts.interleaved_entropy && !uncompressed,
        transform_segmented: false,
        total_segments: 0,
        priority_interleaved: opts.priority_interleaving && !uncompressed,
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

/// Build the packet mask the strict-MSB truncation path would emit for
/// `body_budget` bytes: walk packets in `(sig_0, ref_0, sig_1, ref_1,
/// ...)` order, including each while it fits, stopping at the first
/// packet that would overflow. This is the baseline the §IV.B R-D plan
/// must never decode worse than.
fn strict_msb_prefix_mask(packets: &[EncodedPacket], q: u8, body_budget: u64) -> Vec<bool> {
    let mut mask = vec![false; packets.len()];
    let mut used: u64 = 0;
    'outer: for bp_idx in 0..q as usize {
        for is_sig in [true, false] {
            if let Some(i) = packets
                .iter()
                .position(|p| p.bit_plane == bp_idx as u8 && p.is_significance == is_sig)
            {
                let cost = PacketHeader::ENCODED_BYTES as u64 + packets[i].body.len() as u64;
                if used + cost > body_budget {
                    break 'outer;
                }
                used += cost;
                mask[i] = true;
            }
        }
    }
    mask
}

/// Decode both candidate packet masks through the full inverse pipeline
/// (multi-packet decode + inverse DWT) and return the one whose
/// reconstruction has the lower **exact reconstructed-image** MSE against
/// the (level-shifted) original strip pixels `orig`.
///
/// The greedy ΔD/byte selector ([`select_packets_by_rd`]) optimises an
/// *estimate* of the distortion reduction, and even a §III.A weighted
/// coefficient-domain estimate is only an approximation because ICER's
/// transform is not unitary (its basis vectors are not orthogonal, so the
/// true image MSE carries cross terms the per-coefficient weight ignores).
/// Comparing the genuine inverse-DWT reconstruction MSE is exact, and
/// makes the R-D mode provably never worse than strict-MSB truncation
/// (IPN 42-155 §IV.B: minimum reconstructed-image distortion for the
/// rate). The inverse DWT is the same transform the decoder runs, so the
/// comparison sees exactly what the receiver will.
///
/// Ties (equal MSE) and any decode failure fall back to the greedy plan,
/// which is byte-minimal among equal-quality plans.
#[allow(clippy::too_many_arguments)]
fn pick_lower_distortion_mask(
    packets: &[EncodedPacket],
    q: u8,
    width: usize,
    height: usize,
    levels: u8,
    filter: WaveletFilter,
    orig: &[i32],
    greedy: Vec<bool>,
    strict: Vec<bool>,
) -> Vec<bool> {
    // If the two plans are identical there is nothing to choose.
    if greedy == strict {
        return greedy;
    }
    let recon_mse = |mask: &[bool]| -> Option<f64> {
        let kept: Vec<EncodedPacket> = packets
            .iter()
            .zip(mask.iter())
            .filter(|(_, &m)| m)
            .map(|(p, _)| p.clone())
            .collect();
        let mut coeffs =
            crate::bitplane::decode_bitplanes_multi(&kept, width, height, q, levels).ok()?;
        wavelet_int::inverse_2d_dyadic(&mut coeffs, width, height, levels, filter);
        let mut acc = 0.0f64;
        for (&o, &r) in orig.iter().zip(coeffs.iter()) {
            // Apply the decoder's inverse level-shift + [0,255] clamp so the
            // MSE measured here is *exactly* the reconstructed-image MSE the
            // receiver sees. `orig` is the level-shifted strip (`px - 128`),
            // always in `[-128, 127]`, so its clamped form equals itself; the
            // candidate `r` can exceed that range and is clamped just as
            // `decode.rs` does before comparison. Without the clamp the guard
            // ranks two plans by an *unclamped* MSE that can disagree with the
            // decoded PSNR on large-coefficient (e.g. sparse-impulse) content,
            // letting R-D pick a plan that actually decodes worse than strict.
            let r_pixel = (r + 128).clamp(0, 255);
            let o_pixel = (o + 128).clamp(0, 255);
            let d = (o_pixel - r_pixel) as f64;
            acc += d * d;
        }
        Some(acc)
    };
    match (recon_mse(&greedy), recon_mse(&strict)) {
        (Some(g), Some(s)) if s < g => strict,
        // Greedy wins ties (byte-minimal at equal quality) and is the
        // fallback when either decode fails.
        _ => greedy,
    }
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
        interleaved_entropy: opts.interleaved_entropy && !uncompressed,
        transform_segmented: false,
        total_segments: 0,
        priority_interleaved: opts.priority_interleaving && !uncompressed,
        segment_length: segment_length as u16,
        segment_index,
    };
    let mut out = Vec::with_capacity(SegmentHeader::ENCODED_BYTES + segment_length);
    out.extend_from_slice(&segment.encode());
    out.extend_from_slice(&packet_bytes);
    out.extend_from_slice(body);
    Ok(out)
}
