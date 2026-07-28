# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Fixed

- **Lenient-decode allocation hole** (found by the r433 bounded
  `decode_segment` fuzz campaign): the lenient reconstruction spans
  the full `0..=max_received_index` strip range including missing gap
  strips, so two tiny *received* segments carrying a huge
  `segment_index` gap bought a multi-GB placeholder allocation that
  the received-segment pixel sum never counted against
  `DecodeLimits`. `parse_icer_lenient` / `parse_icer_lenient_with_limits`
  now cap the **reconstruction geometry** itself against
  `max_total_pixels` (refusal pinned by `tests/lenient_decode.rs`;
  the finding is seeded at
  `fuzz/corpus/decode_segment/seed_lenient_index_gap_oom.bin`).

### Added

- **Deep-sample (9..=16-bit) 2-D grayscale support**
  (`IcerPixelFormat::GrayDeep { bits }`) — IPN 42-155 §II.C's own
  operating point ("On MER, all cameras produce 12-bit pixels and each
  is stored using a 16-bit word"; every §VII benchmark image is
  12-bit). A deep image is one single-plane segment stream whose
  coefficients span the deeper range — the §III bit-plane coder,
  context model, and packet machinery are depth-agnostic — wrapped in
  the plane-container framing with new format tag `2` plus one
  bit-depth byte (the 12-byte segment header has no free field left,
  and a bare deep stream would be indistinguishable from an 8-bit
  one). The §III.A level shift generalises to `2^(n-1)`, the decode
  clamp to `[0, 2^n - 1]`, placeholder / missing-strip fills to the
  n-bit midpoint, and the §III.D raw path ships little-endian sample
  pairs. Samples live LSB-aligned in little-endian `u16` words
  (`IcerImage::{sample, set_sample}` accessors added); the `registry`
  conversion maps 10-/12-/16-bit onto the matching `oxideav-core`
  plain-gray deep formats. **Every existing wire form is unchanged**:
  Gray8 streams stay bare byte-for-byte, colour containers keep tag 1.
  Composes with all seven §II.A filters (lossless end to end, pinned
  9..=16), row-strip and §V.B transform-domain segmentation, both
  entropy backends, §III.A priority interleaving, §VI.A min-loss,
  §VI.B byte quotas (the container's 9 framing bytes are charged
  against the budget so the hard cap bounds the total output), ROI
  priorities, the §III.D fallback, lenient decode (deep midpoint
  fill), `DecodeLimits`, and quality-target rate-control. Quality
  metrics follow IPN 42-155 §VII: PSNR / SSIM peaks are now
  `2^b - 1` per the original image's depth (`DistortionReport::
  max_abs_error` widened to `u16`); `ImageStats` scans deep samples
  down-shifted to the 8-bit domain so the filter-selection thresholds
  keep their calibration.

- **IPN 42-155 §II.C dynamic-range analysis for the 2-D path**
  (`wavelet_int::{abs_tap_sum, approx_max_input_range, max_input_range,
  word_bits_for_input_range}`): the Table 3 `Σ|c_i|` per-filter
  rationals (shared with the IPN 42-164 §III.B `γ` factors — both
  papers define the constant identically), the equation (8) approximate
  overflow bound, and the exact **Table 4** maximum-input-dynamic-range
  values for 8/16/32-bit coefficient words after one or two high-pass
  filter operations (two operations — an HH subband — is the §II.B
  pyramid's worst case at any depth). Every Table 4 cell is
  cross-validated to within ±2 of the independently-computed eq (8)
  bound; the §II.C worked examples are pinned (filter F 16-bit two-op
  6449 vs eq (8) ≈6450; 12-bit MER pixels fit 16-bit words under every
  filter; 14-bit pixels fit 16-bit words after one but not two
  operations; 16-bit input can never overflow the crate's `i32`
  coefficient buffers). Full-range 12-bit / 16-bit checkerboards are
  driven through the live §II.A transform and verified within the
  published word sizes.

### Changed

- **`WaveletFilter` variants renamed to the IPN 42-155 §II.A spec
  names**: `Reversible53` -> `FilterQ` and `NineSevenA` -> `FilterA`
  (the old names described the pre-spec stand-in transforms — a
  "textbook 5/3" and a "9/7 float" — that r411 retired; filter A is
  *not* a 9/7 filter in the spec, it is the Table 1
  `alpha = (0, 1/4, 1/4)`, `beta = 0` reversible integer transform).
  Wire ids are unchanged (`FilterQ = 0`, `FilterA = 1`); this is a
  source-level API rename only. No sibling crate references the old
  names.

- **The encode/decode pipeline now runs the spec-exact IPN 42-155
  §II.A reversible integer wavelet transform for every filter**
  (`wavelet_int`), retiring the pre-spec stand-ins: the "textbook" 5/3
  lifting that previously backed filter Q (reversible, but a different
  rounding/boundary formulation than the §II.A eq (3) recurrence) and
  the lossy float lifting that backed filters A-G. All seven Table 1
  filters (A-F + Q) are now **losslessly bit-exact end to end** —
  §II.A specifies lossless operation with *any* of the seven; lossy
  operation is progressive truncation, never the transform. Wire
  filter id 7 (the pre-spec float "filter G", which IPN 42-155 does
  not define) is reserved-invalid again and `WaveletFilter::FilterG`
  is removed; the `wavelet_float` module is deleted. **Wire-affecting
  for every compressed stream** (the crate's wire format is
  self-defined and pre-1.0; no external interop existed): filter-Q
  coefficients differ wherever the eq (3) rounding differs from the
  old lifting, and filters A-F change from lossy-float to reversible
  semantics. Consequences pinned by tests: the all-HH checkerboard
  truncation fidelity jumps ~15 dB at equal budgets (Table 7 contexts
  now see eq (3) HH statistics); §V.B lost-segment containment becomes
  a decaying profile rather than a hard support cutoff (the inverse
  eq (3) `d[n+1]` recursion propagates a floor-truncated tail:
  residual ≤ 4 grey levels beyond a `3·2^D` dilation, bit-exact beyond
  `8·2^D` on the pinned fixtures, vs the old strict `3·2^D`); the
  truncation-fidelity PSNR floors are re-measured. Filter selection
  (`recommend_filter`, `DEFAULT_RD_CANDIDATES`) is unchanged in shape
  but now a pure byte-count trade — every candidate is lossless.

### Added

- **ICER-3D §II.B transform-domain error-containment segmentation**
  (`CubeEncodeOptions::with_transform_domain_segments()`). The spec
  form of IPN 42-164 segmentation: segments are "defined spatially (in
  the wavelet transform domain)" using "the same rectangle partitioning
  algorithm" as 2-D ICER (IPN 42-155 §V.D), "except that in ICER-3D the
  segments extend through all spectral bands". One whole-cube 3-D
  transform; the §V.D partition of the deepest spatially-low-pass
  lattice maps every coefficient `(x, y, λ)` to the segment of low-pass
  pixel `(x >> ts, y >> ts)`; each segment carries its own context
  modeler, entropy coder, and §III.A per-spatial-plane means (computed
  per error-containment segment after all decomposition stages, as
  §III.A requires). The decoder recomputes the partition from the
  header fields alone — segment boundaries never ride the wire. Wire
  form: previously-reserved cube flags bit 1, strip-height field pinned
  to 0 (non-canonical values refused); every pre-existing row-strip
  cube stream parses and decodes unchanged. Lossless at min-loss 0
  across segment counts, filters, both entropy backends, and degenerate
  geometries; §IV.B quota / min-loss compose; §V.D eq (9) segment-count
  ceiling enforced on both sides. This replaces the row-strip
  *approximation* for spec-form use (the row-strip mode remains the
  wire default) — a strip boundary was a hard transform edge, while the
  §V.D mode shares one transform so a lost segment's damage decays
  smoothly instead of leaving a stitched seam.

- **IPN 42-164 §III.B dynamic-range expansion analysis**
  (`high_pass_gamma` / `dynamic_range_expansion` /
  `coefficient_word_bits` in `wavelet3d`, re-exported at the crate
  root). Table 1's per-filter expansion factor γ (the sum of absolute
  linearised high-pass filter taps) as exact rationals for one, two,
  and three high-pass operations — the ICER-3D worst case is three,
  "at most one high-pass filtering operation in each of the three
  dimensions" — plus the §III.B word-size rule. All 21 Table 1 cells
  pinned (rationals + the published log2 columns to printed precision),
  the worked example pinned (filter A: 12-bit data fits 16-bit words
  after a 3-D decomposition; every other filter overflows 16-bit
  words), and the actual 3-D transform verified to stay within the
  published word sizes on full-range checkerboard extremes — which also
  proves the crate's i32 coefficient storage covers the widest case the
  wire admits (16-bit samples through filter F: 22-bit words).

- **ICER-3D loss-tolerant decode** (`parse_icer3d_lenient` /
  `parse_icer3d_lenient_with_limits`, returning `LenientCubeDecode`).
  IPN 42-164 §I: "because compression is progressive within each
  segment, when data loss does occur, any received data for the
  affected segment that precedes the lost portion will allow a lower
  fidelity reconstruction of that segment" — but the strict parser
  refuses a truncated stream outright. The lenient entry point salvages
  every complete packet in wire order: a cut mid-packet keeps the
  segment's delivered packet prefix (deadzone reconstruction), a cut
  before the packets keeps the segment's §III.A means (mean-anchored
  patch), a fully missing segment reconstructs as zero coefficients,
  and a corrupt packet body degrades that one segment instead of
  failing the decode. The report carries per-segment packet counts,
  the intact-segment count, and a truncation flag. Pinned: every
  truncation point of a valid stream (both segmentation modes) decodes
  leniently where strict errors, quality is monotone at segment
  boundaries with the full stream exact, and mid-packet salvage beats
  the nothing-arrived decode. Both smoke harnesses and the fuzz target
  drive the new entry point.

- **ICER-3D cross-segment progressive byte quota.** The cube path's
  §IV.B byte quota now truncates the **global** progressive order —
  every segment's packets of one §IV.A priority value before any packet
  of the next lower priority, segments in index order within a priority
  (the IPN 42-155 §VI.B Fig. 23 cross-segment arrangement carried to
  the cube path) — cut at the first packet that does not fit. The
  previous allocation was sequential-greedy: segment 0 consumed the
  remaining budget before segment 1 was considered, so mid-range quotas
  deep-refined early strips while later strips starved. A single cut of
  the global order still leaves every segment holding a prefix of its
  own packet sequence, so any existing (prefix-tolerant) decode path is
  unaffected; an ample quota remains byte-identical to the unbudgeted
  encode. Wire-affecting for quota-truncated multi-segment cube streams
  (which packets survive changes); applies to both row-strip and
  transform-domain segmentation.

- **IPN 42-155 §VI.B cross-segment progressive byte quota.** A byte
  quota (or soft target) on a multi-segment encode now truncates the
  *global interleaved* packet stream — "ICER compresses each
  error-containment segment of a subband bit plane before moving on to
  another subband bit plane, [so] the output bitstream ... represents
  progressive compression across all of the segments" (§VI.B) — and
  the surviving packets are rearranged per segment for transmission
  (Fig. 23(b)). Previously the quota was allocated
  sequential-greedily: each segment consumed as much of the remaining
  budget as it could before the next was considered, so mid-range
  budgets deep-refined the first segment while later segments starved
  at zero packets (quality could even *decrease* as the budget grew).
  Measured on the textured 64x64 filter-Q fixture, 4 segments, equal
  budgets: row strips 500 B 13.4 -> 22.2 dB, 1000 B 14.2 -> 27.4 dB,
  1500 B 14.4 -> 30.7 dB, 2000 B 20.9 -> 36.0 dB; §V.B
  transform-domain 500 B MSE 2960 -> 402 (both paths now monotone in
  budget). Packets align across segments by absolute magnitude bit
  (default mode) or §III.A group priority (priority-interleaved mode),
  so segments with different bit-plane counts interleave correctly; an
  ample budget is proven byte-identical to the unbudgeted encode. ROI
  `segment_priorities` / `with_center_roi` keep the sequential
  whole-segment scheduling (starving the periphery is that mode's
  documented intent), as do `rd_pruning`, the §III.D fallback, and
  forced-uncompressed. Composes with both entropy backends, §III.A
  priority interleaving, §VI.A `min_loss`, row strips, and §V.B
  transform-domain segments; wire format unchanged (only packet
  selection moves). New `tests/quota_interleave.rs` suite.

- **IPN 42-155 §III.A subband-priority interleaving, end to end**
  (`EncodeOptions::with_priority_interleaving()`). The packet schedule
  becomes the spec's progressive order: subband bit planes walked in
  decreasing §III.A priority (Fig. 7 weights halved per plane, ties to
  the higher decomposition level then `LL, HL, LH, HH`), each subband
  bit plane coded in a **single combined raster pass** over its
  subband lattice (§III: sign immediately after the first nonzero
  magnitude bit), the context model persistent across the whole
  segment (§III.C), and one packet per schedule entry — priority
  groups, with fat subband bit planes (> 256 coefficients,
  `priority::FINE_PACKET_COEFFS`) cut into their own packets so a
  byte-quota truncation stays close to its exact §III.A position. New
  wire flag in previously-reserved header byte 2 bit 7 (the top of the
  old 4-bit filter field), packet headers carry the priority-group
  index; every pre-existing stream parses and decodes unchanged.
  §VI.A `min_loss` composes at schedule granularity (whole subband bit
  planes dropped, `priority::min_loss_excludes_unit`); composes with
  both entropy backends, row-strip + §V.B transform-domain segments,
  colour, budgets/targets, ROI priorities, `quality_target`, and the
  §III.D fallback; mutually exclusive with `rd_pruning`. Measured on
  the textured 64x64 filter-Q fixture at equal budgets vs the
  whole-strip MSB-down order: **+4.3 / +5.6 dB at deep truncation
  (250 / 500 B), mean +1.9 dB** over the sweep (one -2.7 dB outlier —
  the §VI.B "scalloping" of priority-boundary quantisation), at a
  bounded ~+3% lossless framing overhead (more, smaller packets).
  Filter-Q full-quality round-trips stay bit-exact in every composed
  mode. New schedule helpers in `priority`
  (`priority_groups` / `packet_schedule` / `subband_lattice` /
  `subband_coeff_count`), codec drivers in `bitplane`
  (`encode_bitplanes_prioritized` / `decode_bitplanes_prioritized`),
  a dedicated `tests/priority_interleaving.rs` suite, mutation-smoke
  configurations, three checked-in fuzz corpus seeds, and the
  encode-roundtrip fuzz harness drives the new flag.

### Fixed

- **IPN 42-155 §II.B pyramid recursion** (wire-affecting for
  decompositions deeper than 1 level). The dyadic transforms recursed
  on the **top-left `ceil(w/2) x ceil(h/2)` rectangle**, which in the
  interleaved layout contains the previous stage's low- *and*
  high-pass outputs — a mixture, not the LL subband §II.B says each
  further stage decomposes. Deeper stages therefore scribbled over
  detail subbands (a smooth linear ramp grew spurious "HH" energy at
  2+ levels), and every consumer of the dyadic-parity layout — the
  §III.B Table 6/7 subband contexts, the same-subband neighbour walk,
  the §III.A priorities and Fig. 18 min-loss offsets, the §V.B
  segment maps, the §III.A subband weights — was keyed to subbands the
  transform did not actually produce. All three 2-D dyadic paths
  (`wavelet`, `wavelet_float`, `wavelet_int`; the 3-D `wavelet3d` path
  was already lattice-correct) now gather/scatter the stride-`2^j`
  even/even lattice per stage. Round-trips remain bit-exact; lossless
  filter-Q output shrinks ~3% on the 64x64 textured fixture; new
  regression pins assert deeper stages touch only the LL lattice (all
  seven §II.A filters + the float path) and that the 5/3 annihilates a
  linear ramp off the LL lattice. **Streams encoded by earlier
  revisions with `wavelet_levels >= 2` decode to different pixels**
  (the crate's wire format is self-defined and pre-1.0; no external
  interop existed). Byte/PSNR-pinned tests recalibrated
  (`rd_budget` — the winning fixture moved to the sparse-impulse
  image, +7 dB at b = 250; `roi_priority`; `truncation_fidelity`).
- **§IV.C interleaved-entropy-coder mid-stream flush desync.** When
  the 2048-word buffer filled behind a stuck partial front word
  (§IV.C.1), the encoder completed that word with flush bits and
  opened a fresh word for the bin's later source bits — but the
  decoder kept serving the flush bits *as* source bits and
  desynchronised. The decoder now replays the encoder's word list from
  the decoded bit stream (creation order, completion status, drains)
  and, when the mirrored list hits the buffer-full flush, discards the
  flushed bin's pending suffix — exactly the flush bits, since every
  genuinely-arrived bit was already consumed in source order. Found by
  the §III.A priority packets (bigger bodies + persistent model reach
  deep Golomb bins with stuck words); reachable in principle from any
  sufficiently long interleaved-entropy packet. Two regression tests
  pin the path via a test-only mid-stream-flush counter.

- **§V.B transform-domain segmentation (IPN 42-155 §V.B + §V.D), end to
  end.** `EncodeOptions::with_transform_domain_segments()` runs one
  whole-image DWT, partitions the LL subband with the §V.D algorithm
  (`partition::ll_segment_map` / `coefficient_segment_map` — the §V.B
  same-spatial-location rule is a single integer shift in the
  interleaved layout), and codes each segment independently (own
  context modeler + entropy coder). New wire framing in previously-
  reserved space: header byte 7 bit 0 flags the mode; bytes 10..12
  carry `(total_segments, segment_index)` so the decoder recomputes the
  partition per §V.D — boundaries are never encoded. Filter-Q decode is
  bit-exact (both entropy backends, colour, odd geometries); a dropped
  segment is pixel-exact-contained outside a `3·2^D` bleed margin and
  the lenient decoder no longer needs segment 0. Lossless output is
  2–3% smaller than row strips at equal segment count (whole-image
  decorrelation), with no §V.B strip-boundary artifacts.
- **§VI.A minimum-loss quality goal for the 2-D path.**
  `EncodeOptions::with_min_loss(M)`: per-subband Fig. 18 LSB-plane
  exclusion (`priority::min_loss_offset` / `min_loss_skip_map`,
  offsets `HH_j = j-1`, `HL/LH_j = j`, `LL = D+1`, pinned against the
  published D = 3 figure), `M` carried in every packet header's
  previously-reserved byte (0 = historical form; `with_min_loss(0)` is
  byte-identical), fully-excluded trailing planes dropped from the
  wire. Composes with row strips, transform-domain segments, both
  entropy backends, and the byte quota; monotone in bytes + MSE
  (textured 128×128 curve: 12884 B at M=0 down to 442 B at M=8).
- **`bitplane::ScanFilter`** — composable §V.B segment-mask + §VI.A
  plane-exclusion restriction over the significance/refinement passes
  (`encode_bitplanes_filtered` / `decode_bitplanes_filtered`);
  `ScanFilter::ALL` proven byte-identical to the unfiltered scan.
- **Per-push mutation smoke + new fuzz seeds** for the new wire modes
  (`tests/mutation_smoke.rs`, four `seed_transform_*`/`seed_minloss*`
  corpus entries, `encode_roundtrip` fuzz target drives the new
  options); criterion group `transform_segments_64x64_s4` +
  `min_loss_64x64` with wire-form byte-identity pins in the setup.
- **Windowed §V.B segment scans.** `ScanFilter::window` +
  `SegmentRect::image_window` confine each transform-domain segment's
  passes to its contiguous coefficient block (stripe grid kept
  `y = 0`-aligned, so the wire form is byte-identical — pinned on five
  fnv-hashed configurations across backends and min-loss values);
  transform-domain encode drops from ~40% over row strips to parity
  (−29%) on the criterion 64×64 s = 4 pin.

### Fixed

- Two decode-side debug-build panics reachable from corrupted streams
  (bit-plane count near 31 decodes coefficients near `i32::MAX`),
  found by the new mutation smoke: the 5/3 lifting inner neighbour
  sums now wrap and the decoder's inverse level shift saturates before
  the `[0, 255]` clamp.

- **ICER-3D (IPN Progress Report 42-164) — the full hyperspectral cube
  pipeline.** Round 383 implements the staged 42-164 paper end-to-end
  across four new modules plus public API:
  * `wavelet3d` — the §III.A three-dimensional wavelet decomposition.
    Not a plain 3-D Mallat pyramid: after the first stage the
    spatially-low-pass / spectrally-high-pass subbands keep decomposing
    spatially, so a level-k spatial subband carries exactly k spectral
    levels. Implemented as, per stage, one 2-D spatial stage on every
    spectral plane's low-pass lattice then one spectral stage over the
    low-pass block, with the inverse replaying the exact reverse order
    (normative under integer rounding per the §III.A footnote). All
    seven IPN 42-155 §II.A reversible integer filters supported,
    bit-exact reversible across odd/thin/single-band/degenerate
    geometries via pure-function per-dimension stage gating.
  * `subband3d` — §IV.A bit-plane priorities `p = 2b + L - H + 3` and
    the Appendix subband index assignment (paper pins verified: 31
    subbands at D = 3; subband 21 H = 2 / L = 6 with p = 2b + 7; only
    subband 0 reaches priority 0 and none reach 1; priority parity),
    plus the decreasing-priority / decreasing-index schedule. An
    Appendix tie that can arise for D > 3 is broken by a documented
    implementation rule (5).
  * `context3d` — the §IV.C spectral context modeler: 19 contexts
    (Table 2) from the two spectral-neighbour coefficients' categories
    (Tables 3/4/5) and signs (Table 6 prediction + XOR agreement-bit
    sign coding); category-3 bits uncoded; every table cell pinned by a
    unit test. `ContextModel` gains `with_contexts(n)` (counters are now
    length-configurable; the 17-context 2-D default is unchanged) since
    §IV.C shares the 42-155 §III.C MER estimation procedure.
  * `bitplane3d` — the §IV coding engine: one spatial plane at a time in
    raster order, sign coded immediately after a coefficient's first '1'
    bit, four-category tracking in encoder/decoder lockstep, one packet
    per §IV.A priority value, per-subband deadzone mid-bin
    reconstruction of truncated planes, both entropy backends
    (arithmetic + the §IV interleaved coder) through the existing
    `BitSink`/`BitSource` surface.
  * `cube` — `IcerCube` (band-major, 1–16-bit samples),
    `CubeEncodeOptions`, `encode_icer3d` / `parse_icer3d` /
    `parse_icer3d_with_limits` / `is_cube`. Pipeline per row-strip
    error-containment segment (strips extend through all bands, §II.B):
    level shift → 3-D DWT → §III.A per-spatial-plane mean subtraction
    over the spatially-low-pass lattice (one mean per band per segment
    on the wire, added back at the matching decompression step) →
    priority-granular packets. §IV.B rate control verbatim: byte quota
    (framing floor enforced, per-segment packet-prefix truncation,
    geometry always framed) + integer minimum-loss parameter (stop at
    its priority boundary; 0 = lossless when the quota allows),
    composing "whichever comes first". Implementation-defined framing
    behind a `0x0000 0xC3` magic that can never collide with a 2-D
    stream or the colour container; strict validation + the same
    `DecodeLimits` caps as the 2-D path before any allocation;
    saturating decode arithmetic so hostile means/coefficients cannot
    overflow.
  Measured on a 32x32x16 correlated-band scene (lossless, filter Q,
  3 levels): 4.12 bits/sample vs 7.00 bits/sample for per-band 2-D ICER
  (−41% bytes) — the §V.A Table 7 direction. Coverage: ~50 new unit +
  integration tests (`tests/cube3d.rs` + module tests) spanning
  lossless round-trips for all filters/backends/depths/segmentations,
  quota + min-loss sweeps with monotone quality, mean-subtraction
  effectiveness, single-byte-corruption and prefix-truncation sweeps;
  the `decode_segment` fuzz target gains `parse_icer3d` as a fourth
  layer with three encoder-produced cube corpus seeds, and the new
  `tests/corpus_smoke.rs` drives the whole corpus per push.

- **IPN 42-155 §V.D partitioning algorithm.** New `partition` module
  (re-exported at the crate root: `partition`, `partition_params`,
  `ll_dimensions`, `PartitionParams`, `SegmentRect`) transcribing the
  §V.D LL-subband rectangle partition exactly: eq (10)'s row-count
  search (including the published integer loop), assignments (11)-(21)
  with the integer form of eq (13)'s rounding, raster-scan segment
  indices, and the eq (9) `s <= w*h` validity precondition. The Fig. 17
  worked example (w=10, h=14, s=17) pins every output parameter and
  five segment rectangles; property tests sweep validity (§V.D Property
  (4): exact tiling, no zero-dimension segments) across geometries and
  every legal segment count, nearly-equal areas (Property (3)), the
  tall-narrow `r = s` branch of eq (10), and the MER-style 1024x1024 /
  6-stage / 8-segment configuration. Standalone geometry for now: the
  2-D and 3-D pipelines keep their documented image-domain row-strip
  convention until an emitter rework adopts §V.B transform-domain
  segmentation.

- **ICER-3D criterion baseline.** `benches/encode_decode.rs` gains a
  `cube3d_filter_q_32x32x16` group (lossless filter-Q encode + decode of
  a correlated-band 12-bit cube). aarch64-darwin `--quick` smoke:
  encode ~2.43 ms / ~12.9 MiB/s, decode ~2.40 ms / ~13.0 MiB/s — the
  perf reference for future 3-D DWT / spectral-context vectorisation.

### Fixed

- **`parse_icer_lenient` out-of-bounds panic on duplicate segment
  indices** (scheduled decode_segment fuzz crash). Two segments sharing
  a `segment_index` but with different heights made the lenient height
  inference take the canonical strip height from the first duplicate
  and the total height from the last, then write the taller strip past
  the inferred plane. Duplicate indices are a geometry contradiction,
  not §III.E packet loss, and are now refused with
  `IcerError::Unsupported` (the strict decoder already refuses them via
  its contiguity check). Byte-exact corpus seed
  (`seed_lenient_duplicate_segment_index.bin`) + a three-ordering
  regression test in `tests/lenient_decode.rs`.
- **Single-segment encode bypassed `byte_budget`** (scheduled
  encode_roundtrip fuzz crash). The `segment_count == 1` fast path
  returned the raw §III.D uncompressed emission without any budget
  check — 16016 bytes against a 1537-byte cap on the fuzz artifact.
  The §III.D path is all-or-nothing, so a lone segment that cannot fit
  now falls back to the zero-body placeholder header (budget honoured,
  full geometry framed, flat-128 decode), mirroring the multi-segment
  skip semantics. Byte-exact corpus seed + seed-bank entry 11 in
  `tests/encode_fuzz_seed.rs` +
  `single_segment_uncompressed_respects_budget`.

### Added

- **IPN 42-155 §IV interleaved entropy coder — spec-exact, selectable
  encode/decode backend.** ICER's actual entropy stage is not arithmetic
  coding; §IV specifies a bit-wise adaptable *interleaved entropy coder*.
  The new `ixec` module implements it spec-exactly: the §IV.B Golomb codes
  `G_m` (verified bit-for-bit against the Table 9 G5 listing), the §IV.D
  shorthand-tree component codes for bins 2–8, the full Table 10 17-bin
  design (probability cutoffs over the 65536 denominator + Golomb
  assignments G5/G6/G7/G11/G17/G31/G70/G200/G512), and the §IV.C
  interleaving machinery (MER 2048-word circular buffer, FIFO
  front-of-list emission in word-creation order, the §IV.C flush of
  partial words on buffer-full and end-of-stream, and the per-bin suffix
  decode bookkeeping). A context-driven `IxecEncoder`/`IxecDecoder` wrap
  it with the same `(symbol, p1_num, p1_den)` signature as the arithmetic
  coder, applying the §IV.C `p0 >= 1/2` reduction (inverting the bit when
  needed) to select the bin. The bit-plane significance/refinement/sign
  passes are abstracted over a new `entropy::{BitSink, BitSource}` trait
  surface so the identical §III.B pass logic drives either backend, and
  `EncodeOptions::with_interleaved_entropy()` selects the §IV coder for a
  full `encode_icer` → `parse_icer` round-trip. The choice is recorded in
  a previously-reserved segment-header bit (byte 7 bit 1), so every
  pre-existing arithmetic-coded stream parses unchanged and decodes
  exactly as before. Filter-Q full-quality round-trips (Gray8 + colour +
  multi-segment) are bit-exact through the interleaved coder; budget
  truncation frames the full geometry identically. This resolves the
  headline interop gap the README flagged (the §IV coder was previously a
  Witten-Neal-Cleary stand-in). New unit + integration tests cover the
  component-code bijections, a 2000-iteration randomised interleaver
  round-trip, the bit-plane end-to-end path on both backends, and the
  full pipeline.

- **End-to-end fidelity regression coverage for the §III.B same-subband
  walk.** `truncation_fidelity` gains `same_subband_walk_shrinks_lossless_output`
  (filter-Q lossless byte ceilings on the diagonal-ramp and checkerboard
  fixtures, with a bit-exact decode assertion) so a regression to the
  cross-subband spatial-raster walk fails CI. The checkerboard strict-MSB
  PSNR floors are tightened to the r368 measurements (b=600 now clears
  31.0 dB, up from the r365 floor of 24.9 dB — a ~6 dB improvement from the
  same-subband HH neighbourhood).

### Changed

- **Spec-exact §III.B same-subband neighbour walk.** The bit-plane
  scanner's significance + sign context now gathers each coefficient's
  eight nearest neighbours **from the same subband** (IPN 42-155 §III.B:
  "its eight nearest neighbors from the same segment of the subband"),
  retiring the earlier spatial-raster approximation that sampled
  cross-subband neighbours in the Mallat-interleaved buffer. A subband at
  decomposition level `j` is interleaved at stride `2^j` per axis, so the
  same-subband neighbour of `(x, y)` in direction `(dx, dy)` is the buffer
  position `(x + dx·2^j, y + dy·2^j)` (new `priority::subband_stride`);
  off-strip neighbours are "at the edge of the subband segment" and treated
  as not-significant per §III.B. The stride is a pure function of
  `(x, y, levels)` shared by encoder and decoder, so filter-Q full-quality
  and colour round-trips stay bit-exact. Gathering genuinely correlated
  neighbours improves filter-Q lossless compression materially on
  structured 64×64 content: diagonal ramp 2076→1429 bytes (−31%),
  checkerboard 1839→1627 (−11.5%), textured 3570→3226 (−9.6%). The legacy
  single-packet path (`encode_bitplanes_single` / `decode_bitplanes`) stays
  subband-agnostic (`levels = 0` → unit stride). New `priority` +
  `bitplane` unit tests pin the stride formula, the same-subband-only
  pattern gathering, and the divergence from the legacy unit walk.
- **R-D candidate selector compares *clamped* reconstructed-image MSE.**
  `encoder::pick_lower_distortion_mask` now applies the decoder's inverse
  level-shift + `[0,255]` clamp before measuring the candidate MSE, so the
  guard ranks the greedy and strict-MSB plans by exactly the MSE the
  receiver decodes. The previous unclamped comparison could rank a plan as
  no-worse on raw coefficient MSE while it actually decoded with lower PSNR
  on large-coefficient (sparse-impulse) content, letting R-D regress below
  strict; the clamp makes "R-D is never worse than strict-MSB" provably
  hold. `rd_budget` checkerboard win band re-pinned to the budgets where
  the better context model now exposes R-D's residual-fill advantage
  (b=210/310, +3..+8 dB).
- **Spec-exact §III.C probability estimator (MER initial counts +
  rescale-at-500).** The adaptive context-conditional estimator
  (`context::ContextModel`) now matches the IPN 42-155 §III.C "Probability
  Estimation" MER implementation verbatim: initial counts are 2 ones out
  of 4 total (P = 1/2; new `INITIAL_ONES` / `INITIAL_TOTAL` constants),
  and both counts are halved when `total` reaches 500 (new
  `RESCALE_THRESHOLD`), with the count rounded "in the direction that
  makes the probability estimate closer to 1/2" per §III.C, floored so the
  estimate stays strictly inside `(0, 1)` for the arithmetic coder. This
  replaces the round-1 placeholder windowed-counting model (window 64,
  init 1/2, removed `ESTIMATOR_WINDOW`). Because encoder and decoder run
  the identical estimator the self-roundtrip stays exact (filter-Q
  full-quality + colour decodes bit-exact); the longer 500-symbol window
  improves the probability estimate's stationarity on real content. New
  `context` unit tests pin the MER initial counts, the increment rule, the
  rescale-at-threshold halving, and the all-zeros floor. The
  `truncation_fidelity` / `rd_budget` checkerboard floors are re-pinned to
  the r365 measurements (the denser entropy coding shifts where the
  byte-budget cut lands).

- **Subband-aware bit-plane scanner — wires the §III.B Table 6/7 contexts
  + HL transpose into encode + decode.** `BitPlaneInput` gains a `levels`
  field (the dyadic decomposition depth); when `levels >= 1` the
  significance pass classifies each coefficient's `(SubbandType, level)`
  via `priority::classify_position` and selects the spec-exact §III.B
  context — **Table 6** for LL/LH/HL (with the **HL context-template
  transpose**, swapping the `h`/`v` neighbour roles) and **Table 7** for
  HH (keyed on `h + v`). The sign pass applies the matching HL axis-swap
  to the Table 8 prediction. `levels = 0` preserves the prior
  subband-agnostic uniform classification (legacy single-body path + the
  subband-agnostic unit tests). The encoder threads its
  `wavelet_levels` through; the decoder reads `decomp_levels` from the
  segment header, so encoder and decoder dispatch the identical contexts
  and the arithmetic coder stays in lockstep. `decode_bitplanes_multi`
  gains a `levels` parameter. Filter-Q full-quality and colour decodes
  remain bit-exact (the change is a context-selection refinement, not a
  wire-format change); on the all-HH 64×64 checkerboard the spec-exact
  Table-7 HH model now reaches lossless in fewer bytes than the uniform
  model. New `bitplane` tests pin the subband-aware round-trip across
  levels 1..=4 and that the aware path changes the entropy-coded byte
  total vs. the agnostic path (proving the dispatch is wired);
  `tests/truncation_fidelity.rs` checkerboard floors re-pinned to the
  r365 measurements.

### Added

- **Spec-exact IPN 42-155 §III.B per-subband significance context tables
  (Table 6 + Table 7) with the HL context-template transpose.** New pure
  functions in `context` (re-exported at the crate root):
  `neighbour_counts(pattern) -> (h, v, d)` decodes the packed 8-neighbour
  significance pattern into the full horizontal / vertical / diagonal
  counts (`h, v in 0..=2`, `d in 0..=4`) the spec tables index by;
  `significance_context_table6(h, v, d)` is the §III.B **Table 6** grid
  (LL / LH / HL subbands); `significance_context_table7(h, v, d)` is the
  §III.B **Table 7** grid (HH subbands, indexed by `h + v` and `d`);
  `significance_context_subband(h, v, d, is_hh, is_hl)` dispatches between
  the two tables and applies the §III.B **HL transpose** (swap the roles
  of `h` and `v` before the Table 6 lookup). `sign_context_subband` /
  `sign_prediction_flip_subband` apply the matching HL axis-swap to the
  §III.B Table 8 sign prediction. Every published Table 6 / Table 7 cell
  is pinned by a unit test, plus the transpose, the range invariant, and
  the neighbour-count decode. These close the long-documented gap that the
  scanner shipped the collapsed H/V/D classification uniformly rather than
  the subband-specific Table 6/7 indices + the HL transpose; the next
  commit wires them into a subband-aware bit-plane scanner.

### Changed

- **Spec-exact IPN 42-155 §III.B four-category context model with
  category-3 uncoded magnitude bits.** The bit-plane coder now tracks the
  §III.B *category* of every pixel (0 = insignificant; 1 = the first '1'
  bit was coded; 2 = one more magnitude bit; 3 = one more again, stays 3
  permanently) and selects each refinement (magnitude) bit's coding mode
  from the category, replacing the prior `(just_significant,
  has_neighbour)` 3-context heuristic:
  - category 1 → context 9 (no horizontally / vertically adjacent
    significant pixel) or context 10;
  - category 2 → context 11;
  - category 3 → **left uncoded** (fed to the arithmetic coder at a fixed
    probability-of-zero of 1/2 with no model adaptation), matching §III.B
    "Bits of pixels in category 3 are empirically nearly incompressible …
    therefore … left uncoded in the compressor's output."
  The sign contexts are now the exact IPN 42-155 §III.B **Table 8** sign
  prediction + context grid (indices 12..=16) addressed by the horizontal
  and vertical neighbour sign sums, replacing the earlier collapsed 5-way
  `axis_sign_contribution` scheme. The 17-context layout is now the
  spec's: 0..=8 significance, 9/10 category-1 + 11 category-2 refinement,
  12..=16 sign. Encoder and decoder run the identical category
  transitions (including a category-advance fast path when a refinement
  packet is dropped mid-stream), so the change is fully roundtrip-safe:
  filter-Q full-quality and colour (YUV 4:4:4) decodes stay bit-exact,
  and progressive truncation stays monotone. The improved contexts make
  the *strict-MSB* truncated decode markedly better on high-frequency
  content (e.g. the 64×64 checkerboard at a 400-byte budget rose from
  ~19 dB to ~26 dB), which in turn collapsed the old strict-vs-R-D gap on
  that fixture — a baseline improvement, with the R-D selector still
  never regressing. New `tests/category_model.rs` (4 tests) pins the
  category→context mapping, the category-3 uncoded round-trip on a
  high-dynamic-range fixture, progressive monotonicity, and colour
  bit-exactness; `tests/arith_roundtrip.rs` and the `rd_budget`
  checkerboard test were updated to the new model. New public
  `context::{magnitude_context, MagnitudeContext, CATEGORY2_CONTEXT,
  UNCODED_P1}`; `context::refinement_context` removed.
- **Per-coefficient §III.A deadzone reconstruction on mid-plane
  truncation.** The truncated-stream reconstruction point (IPN 42-155
  §III.A) now derives the deadzone bin-width exponent `b` **per
  coefficient** instead of once per strip. A byte-budget cut routinely
  lands between a plane's significance and refinement packets (they are
  separate MSB-down packets): `sig(bp)` survives, `ref(bp)` is dropped.
  Coefficients made newly significant in the surviving `sig(bp)` know
  their MSB (`b = bp`), but coefficients already significant at a higher
  plane never received their plane-`bp` refinement bit and are known only
  down to `bp + 1` (`b = bp + 1`, a bin twice as wide). The decoder now
  tracks the deepest delivered magnitude bit plane for each coefficient
  and applies that coefficient's own `∆/2 - 1` mid-bin offset; a missing
  refinement packet is skipped entirely rather than decoded from an empty
  body, so it injects no spurious magnitude bits. Clean bit-plane-boundary
  truncations are unchanged (every coefficient shares one `b`); mid-plane
  cuts gain **+1.4..+3.0 dB** PSNR (textured 64×64, filter Q, 3-level
  DWT). Decode-side only — the wire format is unchanged and every
  previously-encoded stream decodes better with no re-encode. Two new
  `bitplane` regression tests pin the per-coefficient arithmetic and the
  MSE improvement vs. the strip-global reconstruction.

### Fixed

- **`decode_segment` fuzz harness bounded decode compute, not just
  allocation.** A 12-byte segment header can declare a multi-megapixel
  geometry that sits under the 64 MPx default per-segment
  [`DecodeLimits`] cap yet costs tens of seconds of inverse-DWT +
  bit-plane work — even with a near-empty packet body (a tiny body is a
  legitimate progressive-truncation case, so the geometry cannot be
  rejected by body size). Two scheduled-Fuzz slow-units declared ~34 MPx
  (4160×8240) and took 24–59 s through `parse_icer` /
  `parse_icer_lenient`, drowning out the framing/entropy exploration the
  target exists for. The harness now drives the full-decode layer through
  `parse_icer_with_limits` / `parse_icer_lenient_with_limits` with a
  tight per-run budget (1 MPx/segment, 4 MPx total), turning the
  50-second decode into a sub-millisecond geometry refusal while still
  exercising the allocator, inverse DWT, arithmetic coder and
  multi-segment stitch. The header-only `walk_segment` /
  `parse_icer_metadata` layers keep the default-limits public entry
  points for coverage of the geometry-validation refusal path. Both
  slow-units added to `fuzz/corpus/decode_segment/` as
  `seed_giant_geometry_34mpx_{a,b}.bin`; new `tests/geometry_limits.rs`
  (2 tests) pins the sub-millisecond refusal and the sub-cap-admitted
  behaviour. No public-API or default-limits change.

- **Budget-truncated multi-segment encode dropped image rows** (IPN
  42-155 §V.B independent-segment scheduling). On the legacy
  `segment_index`-order budget path (no ROI priorities), when the byte
  budget ran out the encoder stopped emitting segments without leaving a
  placeholder header for the strips it skipped. Because the strict
  decoder reconstructs the total image height by summing the heights of
  the segments physically present on the wire, the decoded image shrank:
  a 128×64 16-segment image under a 38-byte budget self-roundtripped to
  128×4 (a single 4-row strip). The encoder now emits a zero-body
  placeholder header for every skipped strip — sharing the new
  `emit_skipped_placeholders` helper with the ROI-priority path that
  already did this — and reserves header bytes for the not-yet-decided
  strips while spending the budget so framing the geometry never starves
  itself. The decoder reconstructs each placeholder strip as flat-128 at
  the correct row offset, so geometry is preserved at every budget.
  Found by the scheduled `encode_roundtrip` cargo-fuzz target (crash
  added to the corpus as
  `fuzz/corpus/encode_roundtrip/seed_budget_geometry_16seg_64rows.bin`);
  new `tests/budget_geometry.rs` (3 tests) pins the fuzz repro plus a
  budget sweep and the unchanged unbudgeted path. Unbudgeted encodes are
  byte-for-byte unchanged.

### Added

- **Colour (YUV 4:4:4) encode + decode** (IPN 42-155 §III). The paper
  describes ICER as a single-component coder whose deployed colour scheme runs
  one independent ICER instance per colour component, sharing only the outer
  image metadata. `encode_icer` / `parse_icer` now honour
  `IcerPixelFormat::Yuv444P`: each of the three planes is encoded as an
  independent single-plane ICER bitstream, concatenated behind a small
  multi-plane container header (new `plane_container` module, exported as
  `is_container` / `parse_container` / `ParsedContainer`). The container is
  marked by a leading `0x0000` sentinel, which a single-plane (Gray8) stream
  can never begin with (segment sync prefixes are non-zero), so **every
  previously-encoded Gray8 stream is byte-for-byte unchanged and decodes
  exactly as before**. Colour support threads through `parse_icer`,
  `parse_icer_with_limits`, `parse_icer_metadata`, `parse_icer_lenient`, and
  the registry `Encoder` (a 3-plane `Frame::Video` now selects the colour
  path; 1-plane stays Gray8). Filter-Q colour round-trips are bit-exact across
  all three planes; the uncompressed §III.D path is bit-exact too. New
  `tests/colour_roundtrip.rs` (6 tests) plus `plane_container` unit tests
  cover round-trip fidelity, plane independence, Gray8 non-framing, multi-plane
  metadata, and lenient decode.

- IPN 42-155 §III.A **deadzone-quantizer reconstruction point** for truncated
  streams. Reconstructing a subband from only its `q - b` most-significant bit
  planes is equivalent to a deadzone scalar quantizer with bin width `∆ = 2^b`
  (`b` = number of unavailable least-significant bit planes). The decoder now
  reconstructs each significant coefficient at the §III.A point
  `±((i + 1/2)∆ - 1)` (the mid-bin value biased one step toward the origin)
  instead of the bin lower edge `±i∆`, while insignificant coefficients stay at
  the deadzone-centre origin. The new `unavailable_bit_planes` helper derives
  `b` from which MSB-first packets survived a byte-budget truncation. The
  untruncated path (`b = 0`, `∆ = 1`) keeps a zero offset, so the lossless
  filter-Q round-trip remains bit-exact. Measured on a textured 64×64 image
  under `with_byte_budget`: **+0.5 to +3.8 dB PSNR** on truncated streams
  (e.g. 705 B: 20.93 → 24.75 dB; 969 B: 25.09 → 28.83 dB). Five new tests:
  `unavailable_bit_planes_counts_dropped_lsb_packets`,
  `deadzone_reconstruction_biases_toward_mid_bin`,
  `deadzone_reconstruction_lowers_truncation_mse` (unit) and
  `deadzone_reconstruction_end_to_end` (integration).

- `wavelet_int` -- spec-exact reversible integer wavelet transform for all
  seven ICER filters (A, B, C, D, E, F, Q), transcribed directly from the
  newly-staged IPN 42-155 §II.A equations (1)-(3) and Table 1. Every filter is
  a reversible integer-to-integer transform (ICER's lossless mode works with
  any of the seven, not just Q). Public surface: `forward_1d` / `inverse_1d`
  (one 1-D stage, `ceil(N/2)` low-pass + `floor(N/2)` high-pass outputs);
  `forward_1d_interleaved` / `inverse_1d_interleaved` (even/odd interleaved
  layout matching the rest of the crate); `forward_2d_dyadic` /
  `inverse_2d_dyadic` (the §II.B pyramidal D-level decomposition);
  `WaveletFilter::int_params()` returning the Table 1 parameters
  (`alpha_{-1}, alpha_0, alpha_1, beta`) scaled to a common denominator of 32.
  All seven filters are proven bit-exact reversible across even/odd `N`, the
  interleaved layout, and 1..=5-level 2-D decompositions. This closes the
  previously-documented gap that claimed the per-filter coefficients were
  deferred to an external reference -- they are published in §II.A Table 1.
  README errata note added: IPN 42-155 defines seven reversible integer
  filters (A-F + Q), not "eight float filters A-G plus Q"; the legacy float
  lifting path is retained for backward-compatible round-trip tests but is
  superseded by `wavelet_int` for spec conformance.

- `ssim(original, decoded)` in `analyze` (re-exported at the crate root):
  the mean structural-similarity index of the decoded first plane against
  the original, returned in `-1.0..=1.0` (`1.0` for a bit-identical
  match). SSIM compares local luminance, contrast, and structure over a
  sliding 8x8 window with the standard 8-bit stabilisation constants
  (`C1 = (0.01 * 255)^2`, `C2 = (0.03 * 255)^2`) and complements the
  MSE-family metrics in `DistortionReport`: a structured error (a shifted
  edge) and a diffuse one (uniform noise) can share a PSNR yet differ
  markedly in SSIM. Like `DistortionReport::compare`, it returns
  `IcerError::Unsupported` on a geometry mismatch or a missing plane
  rather than panicking. A pure post-decode measurement -- no wavelet,
  entropy-coder, or framing machinery, and no ICER specification section
  is involved.

## [0.0.4](https://github.com/OxideAV/oxideav-icer/compare/v0.0.3...v0.0.4) - 2026-06-10

### Other

- structured DistortionReport + region_mae quality metrics (round 272)
- Round 262: filter-A wavelet-depth benchmark sweep
- drop release-plz.toml — use release-plz defaults across the workspace
- Round 233: quality-target rate-control
- Round 230: bit-plane-count benchmark sweep
- Round 225: segment-count benchmark sweep
- Round 210: wavelet-decomposition-depth benchmark sweep
- Round 205: float-filter (A) benchmark coverage
- Round 199: encode-side cargo-fuzz harness + per-push seed test

### Added (round 272)

- `DistortionReport` and `region_mae` in `analyze` (re-exported at the
  crate root). `DistortionReport::compare(original, decoded)` walks the
  first plane once and returns every common distortion metric -- `mse`,
  `rmse`, `mae`, `max_abs_error` (the worst single-pixel error, useful
  when a mission needs a per-pixel error ceiling rather than an averaged
  one), and `psnr_db` -- in a single struct. Unlike `psnr_db`, which
  asserts on a shape mismatch and reports only the single PSNR number,
  `compare` returns `IcerError::Unsupported` on a geometry contradiction
  or a missing plane (the non-panicking complement) and computes all
  metrics in one pass instead of re-scanning the pixels per metric.
- `region_mae(original, decoded, x0, y0, w, h)` measures the mean
  absolute error over a rectangular sub-region -- the programmatic form
  of the centre-band vs. periphery MAE comparison the round-6 ROI
  prioritisation feature previously documented in prose only. A caller
  can now verify, in code, that a centre-ROI encode under a tight byte
  budget kept the centre strip's fidelity (low MAE) while the periphery
  was truncated (high MAE). The region must lie fully inside the image
  (`checked_add` guards the `u32` extent overflow a malicious
  `(x0, w)` pair could induce); an out-of-bounds region returns
  `IcerError::Unsupported`, a zero-area region returns `0.0`.
- Both helpers are pure post-decode image-quality measurements
  (spec-neutral; no wavelet / entropy machinery), depend only on std
  primitives, and so are available on both the default `registry` and
  the `default-features = false` standalone builds.
- `tests/distortion_report.rs` -- seven tests covering bit-identical
  (lossless) inputs, exact constant-offset metrics cross-checked against
  a direct per-pixel computation and against `psnr_db`, geometry-mismatch
  error (not panic), the zero-pixel degenerate case, sub-region error
  isolation, region bounds/overflow rejection, and an end-to-end
  centre-beats-periphery assertion on a centre-ROI encode under a
  260-byte budget.

### Added (round 262)

- Filter-A wavelet-depth sweep in the criterion suite under
  `benches/encode_decode.rs`. Two new groups,
  `encode_compressed_filter_a_levels_64x64` and
  `decode_compressed_filter_a_levels_64x64`, sweep `wavelet_levels` over
  `[1, 2, 3, 4]` on the 64x64 ramp on the lossy float 9/7 (`NineSevenA`)
  path -- the round-210 filter-Q `wavelet_levels` sweep's counterpart for
  the float CDF lifting recursion. The round-205 filter-A coverage pinned
  the round-181 default `wavelet_levels = 2`, so the per-depth cost of
  the float dyadic DWT recursion was hidden in the default-depth filter-A
  number; the sweep splits the per-depth cost out so future float-DWT
  work (lifting vectorisation, IEEE-754 quantisation amortisation) has a
  clean per-depth reference. The interesting Q-vs-A delta on a per-depth
  basis is the lifting arithmetic cost -- filter Q's integer 5/3 lifting
  is pure `i32` add/shift, filter A's float 9/7 CDF lifting is `f64`
  multiply-add. Both paths share the same bit-plane scanner +
  arithmetic coder, so the slope difference between this group and the
  round-210 `encode_compressed_filter_q_levels_64x64` group isolates the
  float-vs-integer lifting overhead per added dyadic level.
- Baseline numbers on aarch64-darwin (M-series, default `--bench`
  opt-level, criterion `--quick` smoke):
  * `encode_compressed_filter_a_levels_64x64/levels_1`: ~354 µs / 11.0 MiB/s
  * `encode_compressed_filter_a_levels_64x64/levels_2`: ~441 µs /  8.8 MiB/s
  * `encode_compressed_filter_a_levels_64x64/levels_3`: ~477 µs /  8.2 MiB/s
  * `encode_compressed_filter_a_levels_64x64/levels_4`: ~445 µs /  8.8 MiB/s
  * `decode_compressed_filter_a_levels_64x64/levels_1`: ~224 µs / 17.4 MiB/s
  * `decode_compressed_filter_a_levels_64x64/levels_2`: ~247 µs / 15.8 MiB/s
  * `decode_compressed_filter_a_levels_64x64/levels_3`: ~273 µs / 14.2 MiB/s
  * `decode_compressed_filter_a_levels_64x64/levels_4`: ~243 µs / 16.1 MiB/s
  Encode rises monotonically from depth 1 through depth 3, then flattens
  at depth 4 because the now-tiny LL subband (4x4 = 16 coefficients at
  depth 4) stops adding meaningful forward-lifting work on top of the
  entropy stage. Decode shows the same shape -- the bit-plane scanner's
  stripe coverage hits the same floor regardless of filter choice.
  Filter A is consistently ~40% slower on encode than filter Q at the
  same depth (round 210: ~254-313 µs), which quantifies the float
  lifting overhead vs the integer 5/3 lifting; decode shows ~5% delta
  because the inverse path is entropy-coder dominated.

### Added (round 233)

- Quality-target rate-control via
  `EncodeOptions::with_quality_target(target_db: f32)` (new field
  `quality_target_psnr: Option<f32>`). The compressed-path encoder runs
  a binary search over byte budgets, decodes each trial output,
  computes PSNR against the original `image`, and returns the smallest
  output whose PSNR is greater than or equal to `target_db`. This is
  the inverse shape of [`Self::byte_budget`]: byte-budget says "emit
  at most N bytes, take whatever quality the truncation yields";
  quality-target says "emit at whatever byte count is needed to reach
  quality Q, return the smallest such output". The control suits
  downlink pipelines that ship every image at the same quality (rather
  than the same byte count).
- Algorithm (in `analyze::encode_to_quality_target`): compute the
  bracket via `analyze::quality_search_bounds` (lo = segment-header
  floor, hi = unbudgeted compressed-path encode); if the upper-bracket
  encode misses the target, return it as the best effort; if the
  lower-bracket encode already meets the target, return it; otherwise
  bisect with `BISECT_TOL = 8` bytes for at most `MAX_ITERATIONS = 48`
  steps. Each trial encodes + decodes + computes PSNR via the new
  `analyze::psnr_db(original, decoded)` helper (identical formula to
  the `tests/quota_encode.rs::psnr` helper that has gated the
  round-trip tests since round 4).
- Mutually exclusive with `byte_budget` / `target_bytes` / `rd_pruning`
  -- combining them returns `IcerError::Unsupported` at encode time
  (the quality search manages the budget directly). No-op on the
  uncompressed path (`uncompressed: true`): the round-trip is bit-exact
  by construction so every finite PSNR target is satisfied trivially;
  the encoder short-circuits and uses the regular uncompressed path.
- Public surface additions (re-exported from the crate root):
  `analyze::psnr_db`, `analyze::quality_search_bounds`,
  `analyze::encode_to_quality_target`, plus the new field and builder
  on `EncodeOptions`.
- Tests under `tests/quality_target.rs` exercise the trivial-pass case
  (lossless filter Q collapses to the floor), the target-meeting
  property (filter A on a 32x32 ramp at 25 dB), monotonicity
  (`hi_target` -> at least as many bytes as `lo_target`), the
  best-effort behaviour above the filter ceiling (9999 dB returns the
  unbudgeted encode without erroring), and the three mutual-exclusion
  guards.

### Added (round 230)

- Bit-plane-count sweep in the criterion suite under
  `benches/encode_decode.rs`. Two new groups,
  `encode_compressed_filter_q_bit_planes_64x64` and
  `decode_compressed_filter_q_bit_planes_64x64`, sweep `bit_plane_count`
  over `[4, 8, 12, 16]` on the 64x64 ramp on the integer 5/3 (filter Q)
  path. The round-181 baseline pinned `bit_plane_count = 8` (the
  round-181 default) so the per-bit-plane overhead of the IPN 42-155 §IV
  multi-packet ordering was hidden in the default-floor number; the
  sweep splits the per-count cost out so future entropy-stage work
  (per-packet flush amortisation, bit-plane scanner vectorisation) has
  a clean per-floor reference. `bit_plane_count` acts as a floor on the
  per-segment packet-count `q` (`q = max(needed_for_largest_coeff,
  caller_floor).min(31)`), so raising it above the natural `needed`
  forces the encoder to emit additional bit-plane pairs that mostly
  carry zero significance + zero refinement -- the cleanest way to
  isolate the per-packet fixed cost (arith-coder init / flush / packet
  framing) from coefficient-magnitude noise. The chosen sweep `[4, 8,
  12, 16]` brackets the round-181 default symmetrically: `q_4` lands at
  the natural floor (overridden by `needed`), `q_8` is the round-181
  default, and `q_12` / `q_16` walk well above what any 8-bit Gray8
  input ever reaches so every added plane is pure per-packet overhead.
- Baseline numbers on aarch64-darwin (M-series, default `--bench`
  opt-level, criterion `--quick` smoke):
  * `encode_compressed_filter_q_bit_planes_64x64/q_4`: ~238 µs / 16.4 MiB/s
  * `encode_compressed_filter_q_bit_planes_64x64/q_8`: ~268 µs / 14.6 MiB/s
  * `encode_compressed_filter_q_bit_planes_64x64/q_12`: ~380 µs / 10.3 MiB/s
  * `encode_compressed_filter_q_bit_planes_64x64/q_16`: ~480 µs / 8.1 MiB/s
  * `decode_compressed_filter_q_bit_planes_64x64/q_4`: ~207 µs / 18.9 MiB/s
  * `decode_compressed_filter_q_bit_planes_64x64/q_8`: ~230 µs / 17.0 MiB/s
  * `decode_compressed_filter_q_bit_planes_64x64/q_12`: ~339 µs / 11.5 MiB/s
  * `decode_compressed_filter_q_bit_planes_64x64/q_16`: ~415 µs / 9.4 MiB/s
  Encode + decode both rise monotonically with the caller-supplied
  floor as the encoder emits an extra significance + refinement packet
  pair per added bit-plane and the decoder walks them in turn. The
  step from `q_8` to `q_12` is the largest (+42% encode / +47% decode)
  because `q_8` is already at-or-below the natural `needed` floor on
  this input (the largest DWT coefficient on the 64x64 ramp lands
  around 7-8 bit-planes), so `q_4` and `q_8` both effectively walk the
  same number of packets while `q_12` and `q_16` add 4 and 8 extra
  empty pairs respectively. The ~100% encode-side spread between `q_4`
  and `q_16` is the headroom envelope future per-packet
  arith-coder-init / flush amortisation work has on this input shape.

### Added (round 225)

- Segment-count sweep in the criterion suite under
  `benches/encode_decode.rs`. Two new groups,
  `encode_compressed_filter_q_segments_64x64` and
  `decode_compressed_filter_q_segments_64x64`, sweep `segment_count`
  over `[1, 2, 4, 8]` on the 64x64 ramp on the integer 5/3 (filter Q)
  path. The round-181 baseline pinned `segment_count = 1` (the
  `EncodeOptions::default` value) so the per-strip overhead of the
  IPN 42-155 §III.E independent-segment partitioning was hidden in the
  single-segment number; the sweep splits the per-count cost out so
  future multi-segment work (per-segment parallelism, shared-context
  reuse, lenient-decode tightening) has a clean per-count reference.
  Every value in `[1, 2, 4, 8]` keeps strips >= 8 rows on the 64x64
  ramp, well above the encoder's "minimum 2 rows per strip" floor.
- Baseline numbers on aarch64-darwin (M-series, default `--bench`
  opt-level, criterion `--quick` smoke):
  * `encode_compressed_filter_q_segments_64x64/segments_1`: ~292 µs / 13.4 MiB/s
  * `encode_compressed_filter_q_segments_64x64/segments_2`: ~288 µs / 13.6 MiB/s
  * `encode_compressed_filter_q_segments_64x64/segments_4`: ~296 µs / 13.2 MiB/s
  * `encode_compressed_filter_q_segments_64x64/segments_8`: ~317 µs / 12.3 MiB/s
  * `decode_compressed_filter_q_segments_64x64/segments_1`: ~258 µs / 15.2 MiB/s
  * `decode_compressed_filter_q_segments_64x64/segments_2`: ~255 µs / 15.3 MiB/s
  * `decode_compressed_filter_q_segments_64x64/segments_4`: ~262 µs / 14.9 MiB/s
  * `decode_compressed_filter_q_segments_64x64/segments_8`: ~279 µs / 14.0 MiB/s
  Encode stays within criterion noise across `segments_1..4` then takes
  a ~9% step up at `segments_8` as the per-strip fixed cost
  (arith-coder init + sub-band de-interleave + wavelet boundary
  handling) starts dominating the now-tiny 8-row strip payload. Decode
  rises monotonically (~258 → ~282 µs) on the per-segment framing-parse
  cost. The ~10% encode-side spread is the headroom envelope future
  multi-segment encoder work has on this input shape.

### Added (round 210)

- Wavelet-decomposition-depth sweep in the criterion suite under
  `benches/encode_decode.rs`. Two new groups,
  `encode_compressed_filter_q_levels_64x64` and
  `decode_compressed_filter_q_levels_64x64`, sweep `wavelet_levels`
  over `[1, 2, 3, 4]` on the 64x64 ramp on the integer 5/3 (filter Q)
  path. The pre-round-210 baseline pinned `wavelet_levels = 2` so any
  cost change from the dyadic recursion depth was hidden in the
  default-depth number; the sweep splits the per-depth cost out so
  future wavelet-cache-locality or vectorisation work has a clean
  per-depth reference. `wavelet_levels` is clamped to `1..=6` in
  `src/encoder.rs`; depth 4 is the deepest sensible value for a 64x64
  input (subband LL at depth 4 is 4x4 = 16 coefficients).
- Baseline numbers on aarch64-darwin (M-series, default `--bench`
  opt-level, criterion `--quick` smoke):
  * `encode_compressed_filter_q_levels_64x64/levels_1`: ~254 µs / 15.4 MiB/s
  * `encode_compressed_filter_q_levels_64x64/levels_2`: ~275 µs / 14.2 MiB/s
  * `encode_compressed_filter_q_levels_64x64/levels_3`: ~306 µs / 12.8 MiB/s
  * `encode_compressed_filter_q_levels_64x64/levels_4`: ~313 µs / 12.5 MiB/s
  * `decode_compressed_filter_q_levels_64x64/levels_1`: ~233 µs / 16.8 MiB/s
  * `decode_compressed_filter_q_levels_64x64/levels_2`: ~236 µs / 16.6 MiB/s
  * `decode_compressed_filter_q_levels_64x64/levels_3`: ~270 µs / 14.5 MiB/s
  * `decode_compressed_filter_q_levels_64x64/levels_4`: ~261 µs / 15.0 MiB/s
  Encode shows a clean ~5%/depth slowdown as the dyadic recursion adds
  forward-lifting passes; decode rises with depth on the inverse-DWT
  cost but flattens at depth 4 because the entropy-stage cost
  dominates over the now-tiny LL subband. The two-extreme split
  (levels_1 = single-level DWT vs levels_4 = deepest sensible) gives
  future wavelet-vectorisation work a ~20% encode-side headroom
  envelope to target on this input shape.

### Added (round 205)

- Float-filter (`A`) benchmark coverage. The criterion suite under
  `benches/encode_decode.rs` gains two new groups,
  `encode_compressed_filter_a` and `decode_compressed_filter_a`,
  mirroring the existing filter-Q groups on the same three input
  shapes (`ramp_16x16`, `smooth_16x16`, `ramp_64x64`) so the Q-vs-A
  delta is directly readable from the criterion report. Filter A is
  the lossy float 9/7-style CDF lifting filter named in IPN 42-155
  §III.A and is the Mars-rover lossy default; the existing baseline
  only exercised the integer 5/3 reversible path (filter Q), leaving
  the float-DWT pipeline uncovered as a perf-regression target.
- Baseline numbers on aarch64-darwin (M-series, default `--bench`
  opt-level, criterion `--quick` smoke):
  * `encode_compressed_filter_a/ramp_64x64`: ~286 µs / 13.7 MiB/s
    (vs ~271 µs / 14.4 MiB/s for filter Q -- ~5% encode overhead
    from float lifting + integer quantisation).
  * `decode_compressed_filter_a/ramp_64x64`: ~258 µs / 15.2 MiB/s
    (vs ~254 µs / 15.2 MiB/s for filter Q -- decode is dominated by
    the entropy coder, the inverse-DWT lifting cost is negligible).
  * `encode_compressed_filter_a/smooth_16x16`: ~15 µs / 15.8 MiB/s
    (constant input collapses the entropy stage to its smallest
    regime; same shape on filter Q).
- Documentation gap closed: the round-181 "Baseline numbers" table in
  `README.md` now includes filter-A rows so future float-DWT
  vectorisation has a stable reference point to compare against.

### Added (round 199)

- Encode-side cargo-fuzz harness (`fuzz/fuzz_targets/encode_roundtrip.rs`).
  Synthesises a bounded `IcerImage` (≤ 128 x 128 Gray8) and an
  `EncodeOptions` value (filter A-G + Q, wavelet levels 1-4, segment
  count 1-16, byte budget on/off, target bytes on/off, ROI priority
  permutation on/off, R-D pruning, automatic uncompressed fallback,
  uncompressed-force) from fuzzer-controlled bytes, calls
  `encode_icer`, and on success self-roundtrips through both
  `parse_icer` (strict) and `parse_icer_lenient`. Asserts the
  advertised byte-budget hard cap is honoured (modulo committed
  segment-header slop) and that strict-decode geometry matches the
  encoded geometry. Complements the round-131 `decode_segment` harness
  which only exercises the decoder.
- Per-push smoke test (`tests/encode_fuzz_seed.rs`) that runs the
  exact same extraction + drive logic on a bank of 17 hand-picked seed
  inputs every CI run. Exercises the matrix of encoder option flags
  (uncompressed force, segment count, byte budget, R-D pruning,
  automatic uncompressed fallback, ROI priorities, auto-filter, auto-
  filter-RD), the eight wavelet filters individually, and the
  pathological geometries 1 x 1 / 1 x 32 / 32 x 1 / 128 x 128. A
  regression in the encoder's input-validation surface surfaces in
  normal CI rather than waiting ~24 h for the daily fuzz cron.

## [0.0.3](https://github.com/OxideAV/oxideav-icer/compare/v0.0.2...v0.0.3) - 2026-05-30

### Other

- Round 192: lenient multi-segment decode (IPN 42-155 §III.E)
- Round 189: per-segment uncompressed fallback (IPN 42-155 §III.D)
- Round 181: criterion benchmark for encode + decode hot paths
- Round 174: DecodeLimits — close round-131 fuzz-flagged DoS surface
- Round 131: cargo-fuzz harness for the decoder framing + entropy path
- Round 91: rate-distortion budget pruning (IPN 42-155 §IV.B)
- Round 6: ROI segment prioritisation (IPN 42-155 §III.E)
- Round 5: automatic wavelet-filter selection

### Added (round 192)

- Lenient multi-segment decode (IPN 42-155 §III.E independent-segment
  scheduling). New `parse_icer_lenient` + `parse_icer_lenient_with_limits`
  entry points and a `LenientDecode { image, received, missing_count }`
  report. Tolerates a bytestream missing entire segments (the
  spaceflight scenario where DSN packet loss drops individual ICER
  segments in transit between the orbiter relay and the ground
  station). Missing strips are reconstructed as flat 128 (level-shifted
  zero, matching the round-6 ROI-priority placeholder semantic).
- Geometry rules: segment 0 must be present (it pins the canonical
  strip height + canonical width). The canonical strip height is taken
  from segment 0; non-trailing received segments must agree on that
  height (a wire-format guarantee from `encode_icer`'s `div_ceil`
  row-strip split). The trailing received segment is allowed to be
  shorter. Width mismatch among received segments still surfaces as
  `IcerError::Unsupported` -- this is a real geometry contradiction,
  not a loss-tolerance scenario.
- Image-height semantics: `(max_received_index * strip_h) +
  last_received_height`. A trailing-drop case (segment N missing when
  it was the last) truncates the image at end of segment N-1 rather
  than padding with placeholder rows -- the receiver has no way to
  tell that a higher-indexed segment was supposed to exist.
- Composes with `DecodeLimits` (the round-174 DoS-cap policy applies
  identically) and with all encoder paths (compressed filter Q,
  compressed filter A, uncompressed §III.D).
- New integration test file `tests/lenient_decode.rs` with 9 tests
  covering: no-loss round-trip equivalence with strict `parse_icer`;
  middle-segment drop reconstructing as flat 128; trailing-segment
  drop truncating the image; segment-0-missing rejected with
  `IcerError::Truncated`; width-mismatch among received segments still
  rejected; explicit `DecodeLimits` honoured; uncompressed-path drop
  reconstructing identically; filter-A drop round-tripping; empty-input
  rejected.

### Added (round 189)

- Per-segment automatic uncompressed fallback (IPN 42-155 §III.D
  "Performance with Difficult Imagery"). New
  `EncodeOptions::auto_uncompressed_fallback` field plus a
  `with_uncompressed_fallback()` builder. When enabled on a compressed
  encode, each segment is also encoded via the §III.D raw-pixel path
  and the byte-smaller of the two candidates is emitted. The on-the-
  wire segment-header `uncompressed` flag records the path taken, so
  the decoder reconstructs each segment via its own flag with no
  caller-side awareness.
- Per-segment decision: in a multi-segment image, noisy strips may
  take the uncompressed path while smooth strips stay on the
  entropy-coded path -- matching the spaceflight behaviour the paper
  describes for content where the entropy stage would expand the
  payload (random-noise / high-frequency tiles).
- Fallback honours the wire-format §IV per-segment body-length ceiling
  (`u16::MAX = 65535` pixels): strips exceeding that cap keep the
  compressed result with no error. Equal-length ties go to the
  compressed path (its per-bit-plane progressive structure is strictly
  more useful to a truncating decoder than the uncompressed dump).
- Composes naturally with `auto_filter_rd` (each filter candidate is
  offered the fallback decision independently), `byte_budget` /
  `target_bytes` / `rd_pruning` (compressed candidate honours the
  budget; uncompressed candidate is fixed-size), and
  `segment_priorities` (priority order is independent of path
  choice).
- `tests/uncompressed_fallback.rs` -- six cover-cases: strict noise
  win, ramp keeps compressed, decode round-trip on noise, per-segment
  decision in a two-strip image, no-op when forced uncompressed,
  composes with the float A filter.

### Added (round 181)

- `benches/encode_decode.rs` -- criterion benchmark suite covering the
  encode + decode hot paths. Three input shapes (`ramp_16x16`,
  `smooth_16x16`, `ramp_64x64`) exercised across:
  * `encode_compressed_filter_q` -- the reversible 5/3 wavelet path
    (`WaveletFilter::Reversible53`), `wavelet_levels = 2`,
    `bit_plane_count = 8`.
  * `decode_compressed_filter_q` -- the matching `parse_icer` round-trip
    on pre-encoded fixtures.
  * `uncompressed_path_64x64` -- the IPN 42-155 §III.D path on the
    larger ramp, as a near-`memcpy` contrast against the compressed
    pipeline.
  Each subgroup records `Throughput::Bytes(width * height)` so
  regressions surface as MiB/s or GiB/s on the criterion report.
- Baseline numbers on aarch64-darwin (M-series, debug-info release with
  default `--bench` opt-level, criterion `--quick` smoke):
  * `encode_compressed_filter_q/ramp_64x64`: ~271 µs / 14.4 MiB/s.
  * `decode_compressed_filter_q/ramp_64x64`: ~254 µs / 15.2 MiB/s.
  * `uncompressed_path_64x64/encode`: ~197 ns / 19.2 GiB/s.
  * `uncompressed_path_64x64/decode`: ~336 ns / 11.3 GiB/s.
  The two-order-of-magnitude gap between the compressed and
  uncompressed paths sets the dynamic range future entropy-coder /
  wavelet vectorisation work has to play with.
- `[dev-dependencies] criterion = "0.5"` and a `[[bench]]` table entry
  in `Cargo.toml`. CI's existing `cargo build --all-targets` step
  exercises the bench file as a compilation gate; the benchmark itself
  is opt-in via `cargo bench`.

### Added (round 174)

- `DecodeLimits` struct (`src/decoder.rs`) capping per-segment and
  per-image pixel counts the decoder will agree to materialise.
  Defaults: 64 MPx per segment, 256 MPx total — two orders of
  magnitude above every published Mars-rover Pancam / Hazcam /
  Mastcam-Z delivery frame and three orders of magnitude below the
  4 GB wire-format ceiling. `DecodeLimits::unlimited()` recovers the
  pre-round-174 behaviour for trusted-input batch processing.
- `parse_icer_with_limits` + `parse_icer_metadata_with_limits` —
  explicit-policy entry points. The bare-name `parse_icer` /
  `parse_icer_metadata` functions now apply
  `DecodeLimits::default()` automatically, so every existing caller
  picks up the conservative policy without an API change.
- Both the metadata walk and the full decode check the per-segment
  cap before any plane allocation, and check the multi-segment
  running pixel total against `max_total_pixels` mid-walk. A segment
  / total over the cap returns `IcerError::Unsupported`.
- Closes the round-131 fuzz-flagged DoS surface: a 12-byte segment
  header declaring 65535x65535 used to drive `parse_icer` into a
  ~4 GB plane + ~16 GB coefficient-buffer allocation; under the
  default limits the same input is rejected before any allocation.
- New integration test file `tests/decode_limits.rs` with 6 tests
  covering: rover-sized round-trip under default limits; default
  limits rejecting a 4 GPx synthetic header (both `parse_icer` and
  `parse_icer_metadata`); `unlimited()` letting the metadata walker
  return giant-geometry headers; explicit per-segment cap honoured;
  explicit multi-segment total cap honoured (4-segment 4x12 fixture);
  default constants pinned to the documented values.

### Added (round 131)

- `fuzz/` directory with a cargo-fuzz harness (`decode_segment`)
  exercising the three decode-side entry points against arbitrary
  bytes: `header::walk_segment` (single-segment framing),
  `parse_icer_metadata` (multi-segment walk), and `parse_icer`
  (full decode including arithmetic coder + inverse DWT +
  multi-segment stitch). Panic-free contract under test; no
  reference-codec oracle (clean-room).
- 5 seed-corpus files under `fuzz/corpus/decode_segment/`,
  generated from icer's own encoder: an uncompressed gray ramp
  (16x12), a compressed gray ramp with filter Q (32x32), a
  compressed flat-128 baseline (32x32), a two-segment compressed
  ramp (32x24, segment_count=2), and a truncated compressed ramp
  (32x32, byte_budget=96) covering the partial-packet path.
- `.github/workflows/fuzz.yml` mirroring the sibling crate
  convention (daily 30-minute run; `OxideAV/.github/.../crate-fuzz.yml`).
- Local 60-second fuzz session: 11612 runs, 0 crashes. One slow-unit
  (~800 ms in release) flagged: a 69-byte input with
  width=223, height=65312 causes `parse_icer` to allocate a
  ~14.5 MB plane + inverse-DWT it. Documented as a known DoS
  surface (u16 wire-format width/height permits up to 4 GB per
  plane); not a fuzz-found crash, no source fix in this commit.
- `.gitignore` anchored to `/Cargo.lock` so `fuzz/Cargo.lock` can
  be tracked while the library's lockfile stays untracked.

### Added (round 91)

- Round 91: rate-distortion (R-D) budget pruning per IPN 42-155
  §IV.B rate-allocation principle. New `EncodeOptions` field +
  builder:
  - `rd_pruning: bool` -- when `true` AND `byte_budget` is set, the
    compressed-segment encoder runs a per-segment R-D packet
    selector instead of strict-MSB truncation.
  - `with_rd_budget(n: u64)` -- convenience builder that sets
    `byte_budget = Some(n)` AND `rd_pruning = true`.
- `EncodedPacket` gains a `delta_distortion: f64` field carrying a
  clean-room MSE-reduction estimate (mid-bin variance argument:
  `~2 * 4^bp` per newly-significant coefficient, `~4^bp / 4` per
  refined bit). Populated by `encode_bitplanes`; defaulted to `0.0`
  by the decoder-side `EncodedPacket` construction path.
- R-D selection algorithm (`select_packets_by_rd` in
  `src/encoder.rs`): for every candidate sig-chain depth `K ∈ 0..q`
  compute the mandatory sig-chain cost, residual budget, and
  greedy-fill the residual with refinement packets at `bp_idx ∈
  0..=K` sorted by ΔD/byte descending. Pick the depth K maximising
  total ΔD within the byte cap; ties broken in favour of smaller
  output bytes. Complexity O(q²); fully deterministic.
- New integration test file `tests/rd_budget.rs` with 4 tests:
  - `rd_budget_respects_hard_cap_and_decodes`: 4 budgets (1024-65536)
    on the 256² gradient; output ≤ budget, decodes cleanly, PSNR
    monotonically non-decreasing.
  - `rd_budget_is_deterministic`: identical input -> identical output
    bytes.
  - `rd_budget_with_huge_cap_is_lossless_for_filter_q`: huge budget
    -> R-D selection reduces to "include everything" -> bit-exact
    round-trip for filter Q.
  - `rd_budget_matches_or_beats_strict_msb`: 4 fixtures × 5 budgets
    sweep; R-D never measurably regresses, strictly wins +6.09 dB
    on the 64² checkerboard at the 400-byte budget (strict 19.42 dB,
    R-D 25.51 dB).
- Round 91 README section + workspace-row capability tag.

### Added

- Round 6: ROI (region-of-interest) segment prioritisation per IPN
  42-155 §III.E independent-segment scheduling. New `EncodeOptions`
  field + builders:
  - `segment_priorities: Option<Vec<u16>>` -- per-segment priority
    vector. When `Some(v)`, segments are emitted on the wire in
    `v[seg_idx]`-ascending order (rank 0 first).
  - `with_segment_priorities(prios: Vec<u16>)` -- attach a priority
    vector. Permutation validation happens at `encode_icer` time.
  - `with_center_roi()` -- convenience that constructs a centre-out
    permutation matching the current `segment_count` (Mars Pancam
    use-case: science target at frame centre).
- Combined with `with_byte_budget`, the centre-priority encode keeps
  the highest-priority strips fully encoded while dropping
  lower-priority strips when the budget runs out. Dropped strips
  are emitted as zero-body placeholder segment headers (12 B each)
  so the decoder knows the image height + reconstructs the missing
  strip as flat 128 (level-shifted zero).
- Decoder uncompressed-segment path now tolerates zero-packet
  segments (round 4 already handled zero-packet compressed segments;
  round 6 extends the same tolerance to uncompressed).
- New integration test file `tests/roi_priority.rs` with 10 tests
  covering: `with_center_roi` permutation correctness for various
  segment counts; permuted emission round-tripping bit-exactly for
  filter Q; on-the-wire ordering verification via metadata;
  centre-priority under tight budget yielding lower centre-band MAE
  than periphery MAE (measured 63.98 vs. 84.00 on a 128-row test
  image at 220-byte budget); priority + uncompressed-path
  composition; and rejection of malformed priority vectors (wrong
  length / out-of-range entry / duplicate rank).
- `EncodeOptions` is now `Clone` (no longer `Copy`) because of the
  `Vec<u16>` priorities field. All call sites updated to use
  `.clone()` instead of deref-copy.

### Added (round 5)

- Round 5: automatic wavelet-filter selection. New `analyze` module
  exposes `ImageStats::from_image` (one-pass mean / variance /
  horizontal + vertical gradient energy / dynamic range scan),
  `recommend_filter` (transparent decision tree mapping stats to a
  `WaveletFilter`), `pick_filter_by_rate_distortion` (trial-encode
  each candidate, pick smallest), and the `DEFAULT_RD_CANDIDATES`
  constant (currently `[Q, A]`). Also adds the `analyze` convenience
  helper that returns both stats and recommendation.
- New `EncodeOptions` fields + builders:
  - `with_auto_filter()` -- enable heuristic selection.
  - `with_auto_filter_rd()` -- enable rate-distortion-driven trial
    selection.
  Both override any explicitly-set `filter` value and compose with the
  existing `with_byte_budget` / `with_target_bytes` quota options.
- New integration test file `tests/auto_filter.rs` with 11 tests
  covering: heuristic correctness on flat / smooth / high-frequency
  inputs, override semantics over explicit filter, RD-mode picking
  the byte-minimum candidate, RD interaction with byte budget,
  determinism, edge cases (empty candidate slice).
- 8 new unit tests in `src/analyze.rs` covering `ImageStats` extraction
  + decision-tree branches.

## [0.0.2](https://github.com/OxideAV/oxideav-icer/compare/v0.0.1...v0.0.2) - 2026-05-06

### Other

- rustfmt the `__oxideav_entry` re-export line
- drop dead `linkme` dep
- re-export __oxideav_entry from registry sub-module
- icer round 4: quota-controlled encoding (with_byte_budget + with_target_bytes)
- drop committed Cargo.lock + relax oxideav-core to "0.1"
- roadmap — quota-controlled encoding + ICER 3D
- bump oxideav-core 0.1.18 -> 0.1.21 for first_decoder
- Round 3: filter G + real context tables + stripe scan + multi-packet ordering
- registry calls: rename make_decoder/make_encoder → first_decoder/first_encoder
- bump oxideav-core 0.1.16 -> 0.1.18 for register! macro
- Round 2: bit-plane scanner + compressed segments + multi-segment + float filters A-F
- auto-register via oxideav_core::register! macro (linkme distributed slice)
- unify entry point on register(&mut RuntimeContext) ([#502](https://github.com/OxideAV/oxideav-icer/pull/502))
- add register_containers for .icer extension lookup
- release v0.0.1

### Added

- Round 4: quota-controlled encoding. `EncodeOptions` gains two new
  builder methods:
  - `with_byte_budget(n: u64)` — hard byte cap. Before starting each
    packet the encoder checks whether the total output (segment header
    + body so far + next packet) would exceed `n`; if so, encoding
    stops. Output is strictly ≤ `n` bytes.
  - `with_target_bytes(n: u64)` — soft byte target. Once the running
    total meets or exceeds `n`, the encoder finishes the current
    bit-plane's packet pair (significance + refinement), then stops.
    Output may be slightly above `n`. Composable with `with_byte_budget`
    as a "soft target with hard cap" semantic.
- Decoder now handles zero-packet compressed segments gracefully:
  a segment header with no following packets decodes to an all-128
  (level-shifted zero-coefficient) image rather than returning an
  error. This is the correct behaviour when a very tight byte budget
  allows only the 12-byte segment header.
- New integration test file `tests/quota_encode.rs` covering:
  - `budget_respected_and_decodable`: four budgets (1024, 4096, 16384,
    65536 B) on a 256×256 gray gradient; asserts output ≤ budget,
    decodes successfully, and PSNR is monotonically non-decreasing.
  - `budget_psnr_thresholds`: minimum PSNR bounds per budget level.
  - `soft_target_with_hard_cap`: combined soft target (8192 B) + hard
    cap (10 000 B) produces output in that range.
  - `soft_target_only`: soft target alone finishes within 2× the
    target.
  - `budget_header_only_decodes`: budget of 12 bytes (header only)
    decodes to all-128 pixels.
  - `no_budget_full_roundtrip`: no quota → full lossless round-trip
    (filter Q).

### Added (round 3)

- Round 3: Filter G (Le Gall 5/3 float variant, IPN 42-155 §III.A).
  Same predict/update coefficients as filter Q (alpha=-0.5, beta=0.25)
  applied in floating point with orthonormal post-scale zeta=sqrt(2)/2.
  Completes the full A-G filter set; encoder + decoder dispatch updated;
  header parser accepts filter id 7 (previously reserved-rejected).
- Round 3: real IPN 42-155 §III.B significance context classification.
  Replaces the round-2 popcount placeholder with the H/V/D neighbour-count
  scheme (9 significance contexts; H: 0/1/2+ horizontal, V: 0/1+ vertical,
  D: 0/1+ diagonal). Sign coding uses the §III.B sign-flip convention
  (coded bit = raw sign XOR prediction from neighbour contributions).
- Round 3: stripe-ordered bit-plane scan (IPN 42-155 §III.B). The scanner
  now processes coefficients in horizontal stripes of height 4 rows
  (`STRIPE_HEIGHT = 4`), matching the paper's context-locality optimisation.
  Both significance and refinement passes are stripe-ordered.
- Round 3: multi-packet ordering per IPN 42-155 §IV. Each compressed segment
  now emits one significance packet + one refinement packet per bit-plane
  (2*Q packets total), each independently arithmetic-coded. A decoder
  receiving a truncated stream reconstructs at lower quality. Decoder updated
  to reconstruct coefficient buffer from per-bit-plane `EncodedPacket` slices.
- New `EncodedPacket` struct exported from `bitplane` module; new
  `encode_bitplanes` (multi-packet) + `decode_bitplanes_multi` API.
  Legacy `encode_bitplanes_single` / `decode_bitplanes` kept for backward
  compat with the single-body test path.
- New tests: `compressed_roundtrip_filter_g_within_tolerance`,
  `compressed_roundtrip_filter_q_multi_packet_metadata`,
  `compressed_roundtrip_all_filters` (filters A-G end-to-end),
  `multi_segment_compressed_all_filters`, filter G header roundtrip.

### Changed

- Round 2: bit-plane scanner (`bitplane` module) -- significance + sign
  + refinement passes, MSB-down, raster scan inside each pass; drives
  the binary arithmetic coder for the compressed-segment path.
- Round 2: float wavelet filters A through F (`wavelet_float` module)
  -- CDF 9/7-style + Daubechies + Haar lifting kernels; 1-D + 2-D + dyadic
  paths round-trip to IEEE-754 tolerance. Filter-aware dispatch helper
  `wavelet_float::forward_2d` / `inverse_2d` selects integer (filter `Q`)
  vs float (filters A-F) automatically.
- Round 2: compressed-segment encode + decode wired through `encode_icer`
  / `parse_icer`. New `EncodeOptions::compressed()` constructor + the
  existing `uncompressed: bool` field flips between paths. Filter `Q`
  self-roundtrips bit-exactly; filters A-F self-roundtrip to bounded
  mean-abs error (lossy through float DWT + integer rounding).
- Round 2: multi-segment images via `EncodeOptions::segment_count`. The
  encoder splits the image into row-strips per IPN 42-155 §III.E; the
  decoder stitches segments back in `segment_index` order, validating
  width agreement + contiguous indexing.
- New tests in `tests/compressed_roundtrip.rs` cover all four new sub-
  features; new unit tests in `bitplane` + `wavelet_float` modules cover
  the building blocks.

## [0.0.1](https://github.com/OxideAV/oxideav-icer/releases/tag/v0.0.1) - 2026-05-05

### Other

- Round 1: scaffold + spec coverage map (clean-room from IPN 42-155)
- Initial commit: MIT LICENSE (Karpelès Lab Inc.)

### Added

- Round-1 scaffold for ICER (JPL's progressive wavelet image compressor,
  Mars rover heritage). Clean-room from Kiely & Klimesh, IPN Progress
  Report 42-155 (2003) as the sole specification source.
- Segment header + packet header parser/encoder pair (12-byte and
  4-byte fixed-width framings respectively).
- `walk_segment` enumerator that returns per-packet body byte ranges
  without decoding pixels.
- Integer reversible 5/3 wavelet (Filter `Q` in IPN 42-155 §III.A) —
  forward + inverse, 1-D + 2-D one-level + dyadic D-level, with
  whole-sample symmetric boundary extension.
- Subband de-interleave helper that rearranges the lifted output into
  the conventional 4-quadrant LL / HL / LH / HH layout.
- Binary arithmetic coder + decoder (16-bit registers, follow-bit
  carry handling) with adaptive Laplace-windowed context-conditional
  probability estimator (`ContextModel`).
- Decoder entry points: `parse_icer_metadata`, `parse_icer`,
  `decode_uncompressed_icer`. Compressed-segment paths parse headers
  but defer pixel decode to round 2.
- Encoder entry point: `encode_icer` with `EncodeOptions`. Round 1
  emits the IPN 42-155 §III.D uncompressed (entropy-bypassed) path.
- `oxideav-core` `Decoder` / `Encoder` trait impls plus
  `registry::register` entry point — gated behind the default-on
  `registry` feature.
