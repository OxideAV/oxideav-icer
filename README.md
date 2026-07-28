# oxideav-icer

[![CI](https://github.com/OxideAV/oxideav-icer/actions/workflows/ci.yml/badge.svg)](https://github.com/OxideAV/oxideav-icer/actions/workflows/ci.yml) [![crates.io](https://img.shields.io/crates/v/oxideav-icer.svg)](https://crates.io/crates/oxideav-icer) [![docs.rs](https://docs.rs/oxideav-icer/badge.svg)](https://docs.rs/oxideav-icer) [![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

Pure-Rust ICER -- JPL's progressive wavelet image compressor used by every
Mars surface mission since the 2003 Mars Exploration Rovers (Spirit and
Opportunity), continued on Mars Science Laboratory (Curiosity), Mars 2020
(Perseverance), and follow-on missions.

This is a **clean-room** implementation. The specification sources
consulted are the two open JPL papers: Kiely & Klimesh, "The ICER
Progressive Wavelet Image Compressor", *IPN Progress Report 42-155*
(2003) -- abbreviated `IPN 42-155` in source comments -- and, for the
hyperspectral ICER-3D pipeline, Kiely, Klimesh, Xie & Aranki, "ICER-3D:
A Progressive Wavelet-Based Compressor for Hyperspectral Images", *IPN
Progress Report 42-164* (2006) -- abbreviated `IPN 42-164`. Those two
papers are the sole sources; no other material was consulted,
paraphrased, or cross-checked.

## Status

| Subsystem                | Status            |
|--------------------------|-------------------|
| Segment header parser    | full (12-byte fixed-width framing, both directions) |
| Packet header parser     | full (4-byte fixed-width framing, both directions) |
| Segment walker           | full (enumerates every packet body in a buffer) |
| Integer 5/3 wavelet (legacy) | retained (pre-spec "textbook" lifting, kept as the interleaved-layout contract reference + lattice helpers; the pipeline no longer runs it -- see `wavelet_int`) |
| Spec-exact reversible integer filters A-F + Q | full (IPN 42-155 §II.A eqs (1)-(3) + Table 1; all seven filters bit-exact reversible, 1-D + interleaved + dyadic 2-D -- see `wavelet_int`). **The encode/decode pipeline runs on it for every filter (r411)**: all seven filters are lossless end to end; lossy operation is progressive truncation only |
| Subband de-interleave    | full (4-quadrant LL / HL / LH / HH layout) |
| Binary arithmetic coder  | full (16-bit registers, follow-bit carry, both directions) |
| §IV interleaved entropy coder | full (IPN 42-155 §IV.B Golomb codes G_m + §IV.D shorthand-tree component codes + **Table 10** 17-bin design + §IV.C 2048-word circular-buffer interleaving with flush bits; a selectable encode/decode backend via `EncodeOptions::with_interleaved_entropy()`, recorded in a previously-reserved segment-header bit so old streams parse unchanged -- see "Interleaved entropy coder" below) |
| Probability estimator    | full (IPN 42-155 §III.C MER implementation: initial counts 2/4, rescale when total reaches 500, round-toward-1/2 on the halving) |
| Context model            | full (IPN 42-155 §III.B; 9 significance + 5 sign + 3 refinement contexts) |
| Per-subband context tables | full (IPN 42-155 §III.B **Table 6** LL/LH/HL + **Table 7** HH, keyed on the full `(h, v, d)` neighbour counts, with the **HL context-template transpose** + matching Table 8 sign transpose; the bit-plane scanner is subband-aware via `priority::classify_position`, dispatching the correct table per coefficient, and gathers the **same-subband** neighbourhood at stride `2^level` (`priority::subband_stride`) per §III.B rather than the cross-subband spatial-raster cells -- see "Per-subband context tables" below) |
| Bit-plane scanner        | full (stripe-ordered significance + sign + refinement passes, MSB-down) |
| Multi-packet ordering    | full (one significance + one refinement packet per bit-plane per IPN 42-155 §IV) |
| Subband priority model   | full (IPN 42-155 §III.A Fig. 7 per-subband priority weights + cross-subband bit-plane encode order with the LL/HL/LH/HH tie-breaks -- see `priority`) |
| §III.A subband-priority interleaving | full (`EncodeOptions::with_priority_interleaving()` -- the spec's progressive packet schedule: subband bit planes in decreasing §III.A priority, each coded in one combined raster pass over its subband lattice with the sign immediately after the first nonzero magnitude bit (§III), persistent per-segment context model (§III.C), one packet per priority group with fat subband bit planes split out; wire flag in previously-reserved header space, pre-existing streams decode unchanged; **+10.4 dB at deep truncation, mean +5.4 dB, every budget point positive** vs the whole-strip MSB-down order at equal budgets (re-measured r411 on the §II.A transform) -- see "Subband-priority interleaving" below) |
| §II.B pyramid recursion (r405 fix) | full (each further dyadic stage decomposes the **LL lattice** (stride `2^j` even/even), not the top-left rectangle; all seven §II.A filters pinned by stage-invariance + linear-ramp-annihilation tests; wire-affecting fix for `levels >= 2` -- see CHANGELOG) |
| Compressed-segment encode | full (level-shift + DWT + stripe scan + multi-packet arith) |
| Compressed-segment decode | full (multi-packet arith + stripe scan + inverse DWT + clamp) |
| Deadzone reconstruction point | full (IPN 42-155 §III.A; truncated streams reconstruct significant coefficients at the mid-bin `±((i+1/2)∆-1)` point biased toward the origin, ∆=2^b; the deadzone exponent `b` is tracked **per coefficient** so a budget cut that lands between a plane's significance and refinement packets reconstructs newly-significant and already-significant coefficients at their own bin widths; insignificant coefficients at the deadzone centre. +0.5..+3.8 dB on bit-plane-boundary truncation, a further +1.4..+3.0 dB on mid-plane (refinement-dropped) cuts; untruncated filter-Q stays bit-exact -- see "Deadzone reconstruction" below) |
| Quota-controlled encoding | full (`with_byte_budget` hard cap + `with_target_bytes` soft target; every budget path frames the full image geometry, IPN 42-155 §V.B) |
| §VI.B cross-segment progressive quota | full (a quota on a multi-segment encode truncates the **global interleaved** packet stream -- each segment of a bit plane before the next bit plane -- then rearranges the survivors per segment, IPN 42-155 §VI.B Fig. 23; replaces the sequential-greedy allocation that deep-refined segment 0 while later segments starved: row strips at equal budgets gain +8.8..+16.3 dB, transform-domain 500 B MSE 2960 -> 402, both monotone in budget; ample budgets proven byte-identical to the unbudgeted encode; ROI priorities / rd_pruning / §III.D fallback keep their per-segment scheduling -- see `tests/quota_interleave.rs`) |
| Multi-segment images     | full (row-strip split on encode, stitch by `segment_index` on decode) |
| Uncompressed-segment decode | full (IPN 42-155 §III.D; placeholder-segment tolerant) |
| Uncompressed-segment encode | full (IPN 42-155 §III.D) |
| Image statistics + filter recommendation | full (`ImageStats::from_image` + `recommend_filter` decision tree) |
| Auto filter selection (heuristic) | full (`EncodeOptions::with_auto_filter()`, single-pass) |
| Auto filter selection (rate-distortion) | full (`EncodeOptions::with_auto_filter_rd()`, N-pass trial) |
| ROI segment prioritisation | full (`with_segment_priorities` + `with_center_roi`; IPN 42-155 §III.E independent-segment scheduling) |
| R-D budget pruning | full (`EncodeOptions::with_rd_budget(n)` -- per-segment cost-per-byte packet selection per IPN 42-155 §IV.B rate-allocation principle) |
| Decoder / Encoder traits | full (gated on default `registry` feature) |
| Decode-side resource limits | full (`DecodeLimits` + `parse_icer_with_limits`; default 64 MPx/segment, 256 MPx total; closes the 4 GB-per-plane DoS surface the wire format admits; `DecodeLimits` bounds decode *compute* as well as allocation -- the `decode_segment` fuzz harness uses a tight 1 MPx/segment budget so a single crafted header declaring multi-MPx geometry cannot spend tens of seconds in the inverse DWT + bit-plane scan) |
| Per-segment uncompressed fallback | full (`EncodeOptions::with_uncompressed_fallback()` -- per-segment §III.D choice between compressed and raw-pixel paths; byte-smaller wins) |
| Lenient multi-segment decode | full (`parse_icer_lenient` -- tolerates missing segments per IPN 42-155 §III.E independent-segment scheduling; missing strips reconstruct as flat 128; segment 0 must be present to pin canonical strip height) |
| Encode-side fuzz harness | full (`fuzz/fuzz_targets/encode_roundtrip.rs` synthesises bounded `IcerImage` + `EncodeOptions` from fuzzer bytes, drives `encode_icer`, self-roundtrips through `parse_icer` + `parse_icer_lenient`; complements the decode harness; `tests/encode_fuzz_seed.rs` runs the same logic on 17 hand-curated seeds every CI push) |
| Quality-target rate-control | full (`EncodeOptions::with_quality_target(target_db: f32)` -- binary search over byte budgets, decode each trial, compute PSNR via `analyze::psnr_db`, emit the smallest output whose PSNR is >= the target. Inverse shape of `with_byte_budget`. Mutually exclusive with `byte_budget` / `target_bytes` / `rd_pruning`; uncompressed-forced is a no-op; above-ceiling targets return the unbudgeted encode as best-effort) |
| Post-decode quality metrics | full (`DistortionReport` MSE/RMSE/MAE/max-abs/PSNR + `region_mae` + `ssim` -- mean structural-similarity index over a sliding 8x8 window; all spec-neutral) |
| Benchmark sweeps | full (criterion suite sweeps `wavelet_levels`, `segment_count`, and `bit_plane_count` on the §II.A filter-Q and filter-A paths -- see Benchmarks below) |
| Colour (YUV 4:4:4) encode + decode | full (IPN 42-155 §III independent-per-component scheme; each plane is its own single-plane ICER bitstream behind a `plane_container` header; filter-Q + uncompressed round-trips bit-exact across all three planes; Gray8 wire form byte-for-byte unchanged -- see "Colour images" below) |
| ICER-3D 3-D wavelet decomposition | full (IPN 42-164 §III.A staged decomposition -- spatially-low-pass / spectrally-high-pass subbands keep decomposing spatially; bit-exact reversible for all seven §II.A integer filters across degenerate geometries -- see `wavelet3d`) |
| ICER-3D subband priorities + indices | full (IPN 42-164 §IV.A `p = 2b + L - H + 3` + the Appendix index-assignment rules; paper pins verified (subband 21 H=2/L=6, `p = 2b+7`; only subband 0 reaches priority 0) -- see `subband3d`) |
| ICER-3D spectral context modeler | full (IPN 42-164 §IV.C Tables 2-6: 19 contexts from the two spectral-neighbour coefficients, Table 6 sign prediction + agreement bits, category-3 uncoded -- see `context3d`) |
| ICER-3D cube encode + decode | full (IPN 42-164 pipeline: §III.A mean subtraction (one mean per band per segment on the wire) + §IV bit-plane coder with one packet per priority value + §IV.B byte quota / minimum-loss rate control; lossless at min-loss 0, both entropy backends, row-strip **and** §V.D transform-domain segments, `DecodeLimits` caps -- see "ICER-3D" below) |
| ICER-3D §II.B transform-domain segmentation | full (`CubeEncodeOptions::with_transform_domain_segments()` -- the spec form: one whole-cube transform, error-containment segments as IPN 42-155 §V.D rectangles of the spatially-low-pass lattice "extend[ing] through all spectral bands", per-segment context modeler / entropy coder / §III.A means; decoder recomputes the partition from header fields; segment-loss containment pinned (residual ≤ 4 beyond a `3·2^ts` dilation, bit-exact beyond `8·2^ts`, lost windows reconstruct mean-anchored at 2x+ better MAE than a flat fill) -- see "ICER-3D" below) |
| ICER-3D cross-segment progressive quota | full (the §IV.B byte quota cuts the **global** priority-interleaved packet order -- all segments' packets of a priority before the next lower priority, the IPN 42-155 §VI.B Fig. 23 arrangement carried to cubes -- so no segment starves; budgeted wire pinned packet-for-packet against a simulated global-order prefix; ample quota byte-identical) |
| ICER-3D §III.B dynamic-range analysis | full (Table 1 γ factors as exact rationals + the word-size rule, `high_pass_gamma` / `dynamic_range_expansion` / `coefficient_word_bits`; all 21 table cells + the filter-A 12-bit → 16-bit-word worked example pinned; the live transform verified within the published word sizes on full-range extremes) |
| ICER-3D loss-tolerant decode | full (`parse_icer3d_lenient` -- salvages every complete packet of a truncated / partially corrupt cube stream per the §I progressive-within-segment promise: packet-prefix deadzone reconstruction, mean-anchored patches when only a segment's fixed part arrived, zero-coefficient fill for missing segments; `LenientCubeDecode` reports per-segment recovery; every truncation point of a valid stream pinned decodable with quality monotone at segment boundaries) |
| §V.D partitioning algorithm | full (IPN 42-155 §V.D eqs (9)-(21): LL-subband rectangle partition, integer-only, Fig. 17 worked example pinned parameter-for-parameter; `partition` + the §V.B maps `ll_segment_map` / `coefficient_segment_map`) |
| §V.B transform-domain segmentation | full (`EncodeOptions::with_transform_domain_segments()` -- one whole-image DWT, §V.D LL partition mapped to every subband, each segment coded with its own context modeler + entropy coder; decoder recomputes the partition from the header parameters per §V.D; lenient decode no longer needs segment 0; a lost segment's loss decays to bit-exact outside a bounded wavelet-support bleed -- see "Transform-domain segmentation" below) |
| §VI.A minimum-loss parameter (2-D) | full (`EncodeOptions::with_min_loss(M)` -- per-subband Fig. 18 LSB-plane exclusion, `M` on the wire in every packet header, composes with both segmentation modes + both entropy backends + the byte quota; M=0 byte-identical to the historical stream -- see "Minimum-loss quality goal" below) |
| §II.C dynamic-range analysis (2-D) | full (Table 3 `Σ\|c_i\|` rationals + eq (8) approximate bound + the **exact Table 4** max-input-dynamic-range values for 8/16/32-bit words after one or two high-pass operations, `wavelet_int::{abs_tap_sum, approx_max_input_range, max_input_range, word_bits_for_input_range}`; every cell cross-validated within ±2 of eq (8), the §II.C worked examples pinned, full-range 12-/16-bit checkerboards verified within the published word sizes on the live transform) |
| Deep-sample (9..=16-bit) grayscale | full (`IcerPixelFormat::GrayDeep { bits }` -- the §II.C MER operating point (12-bit pixels in 16-bit words); a plane-container tag-2 framing carries the depth around a normal segment stream; lossless under all seven filters at every depth, composes with every mode -- see "Deep-sample grayscale" below) |

End-to-end round-trips:

* Uncompressed Gray8 -- bit-identical.
* Compressed Gray8 with **any of the seven §II.A filters (A-F + Q)** --
  bit-identical via the spec-exact reversible integer transform +
  stripe-ordered bit-plane scanner + multi-packet arithmetic coder.
* Multi-segment row-strip split -- bit-identical for the uncompressed path,
  bit-identical for compressed filter `Q`.
* Multi-packet metadata: segment with q=4 bit-planes produces >= 8 packets
  (verified by `compressed_roundtrip_filter_q_multi_packet_metadata`).
* **Decode-configuration matrix** (`tests/decode_configurations.rs`):
  filter-Q full-quality decode is bit-exact across geometries
  17×13 / 31×31 / 64×64 / 5×200 / 200×5 and decomposition levels 1..=5
  (including odd / thin strips and over-deep pyramids); progressive
  truncation is monotone in budget in every cell; the filter-A path is
  bit-exact across the same geometries; and an oversized `bit_plane_count`
  (8 / 12 / 16) still decodes bit-exact.

## Spec-exact reversible integer filters (IPN 42-155 §II.A)

The staged IPN 42-155 §II.A "Wavelet Transform" section specifies that an ICER
user selects **one of seven reversible integer wavelet transforms**: filters
**A, B, C, D, E, F, Q**. Every one is a reversible integer-to-integer transform
-- ICER's lossless mode works with *any* of the seven, not just Q. §II.A
equations (1)-(3) give the exact integer high-pass recurrence, and **Table 1**
publishes the per-filter parameters `(alpha_{-1}, alpha_0, alpha_1, beta)`
directly (these are *not* deferred to a follow-up reference -- they are in the
spec). The `wavelet_int` module transcribes §II.A verbatim:

* `forward_1d` / `inverse_1d` -- one 1-D stage producing `ceil(N/2)` low-pass
  and `floor(N/2)` high-pass outputs, exactly invertible.
* `forward_1d_interleaved` / `inverse_1d_interleaved` -- the same stage in the
  even/odd interleaved layout used by the rest of the crate.
* `forward_2d_dyadic` / `inverse_2d_dyadic` -- the §II.B pyramidal D-level
  decomposition (rows-then-columns forward; columns-then-rows inverse).

All seven filters are proven **bit-exact reversible** across even/odd `N`, the
interleaved layout, and 1..=5-level 2-D decompositions (see the `wavelet_int`
tests). `WaveletFilter::int_params()` exposes the Table 1 parameters scaled to
a common denominator of 32 so the equation-(3) predictor evaluates with a
single floor-division.

> **Errata note (resolved r411).** The crate's early rounds carried a
> pre-spec "float wavelet filters A-G" framing (lossy float lifting for
> A-F plus an invented "filter G"). IPN 42-155 §II.A defines **seven**
> filters (A-F + Q), all integer-reversible, and no filter G. The
> pipeline has been migrated onto `wavelet_int` for every filter, the
> float module is deleted, and wire filter id 7 is reserved-invalid
> again. Streams encoded by earlier revisions decode to different
> pixels (self-defined pre-1.0 wire; no external interop existed).

## Wavelet filter coverage

All seven §II.A filters drive the full pipeline and are lossless end
to end. Measured lossless byte counts on the textured 64x64 fixture
(3-level DWT, arithmetic backend) show the per-filter spread the
paper's §II.D lossless comparison describes:

| filter | Q | A | D | B | E | C | F |
|--------|--:|--:|--:|--:|--:|--:|--:|
| bytes  | 3149 | 3148 | 3148 | 3160 | 3205 | 3213 | 3241 |

The ranking is image-dependent (§II.D publishes a different ranking on
12-bit Mars imagery than its lossy comparison); `with_auto_filter_rd()`
finds the empirical winner per image.

## Context model (IPN 42-155 §III.B)

ICER's bit-plane coder keeps a **category** for every pixel that
summarises how many magnitude bits have already been coded (§III.B):

* **category 0** -- not yet significant;
* **category 1** -- the first `1` magnitude bit was coded (the pixel just
  became significant);
* **category 2** -- one more magnitude bit coded;
* **category 3** -- one more again, and stays 3 permanently.

The 17 contexts split exactly as the paper specifies:

* **9 significance contexts** (indices 0..=8, category 0) -- determined
  by the count of significant horizontal neighbours (H: 0, 1, or 2+),
  vertical neighbours (V: 0 or 1+), and diagonal neighbours (D: 0 or 1+).
* **Category-aware refinement contexts** -- a category-1 magnitude bit
  uses context 9 (no horizontally / vertically adjacent significant
  pixel) or 10; a category-2 magnitude bit uses context 11; a
  **category-3 magnitude bit is left uncoded** -- empirically nearly
  incompressible, so it is fed to the arithmetic coder at a fixed
  probability-of-zero of 1/2 with no model adaptation (§III.B). This both
  matches the spec and frees bytes for the packets that carry real
  information, so a budget-truncated decode reconstructs at higher quality
  for the same byte count.
* **5 sign contexts** (indices 12..=16) -- the exact §III.B **Table 8**
  sign-prediction + context grid, addressed by the horizontal and
  vertical neighbour sign sums (`h1 + h2`, `v1 + v2`). Sign bits are
  coded as the "agreement" XOR of the raw sign and the Table 8
  prediction, so the model always sees a bit whose `1` means "agrees with
  prediction".

The category transitions run identically on the encoder and decoder
(including a category-advance fast path when a budget cut drops a
refinement packet mid-stream), so the model stays in lockstep and the
filter-Q full-quality + colour round-trips remain bit-exact through the
category-3 uncoded path.

## Per-subband context tables (IPN 42-155 §III.B Tables 6 + 7)

The §III.B category-0 significance context is **subband-dependent**, and
the bit-plane scanner now dispatches it per coefficient. Each
coefficient's `(SubbandType, level)` is resolved from its `(x, y)`
position via `priority::classify_position`, then the significance context
is selected from:

* **Table 6** — LL, LH, and HL subbands, indexed by the full horizontal
  `h` (0/1/2), vertical `v` (0/1/2), and diagonal `d` (0..4) significant-
  neighbour counts (`context::significance_context_table6`). Unlike the
  earlier uniform classification, which collapsed V and D to a binary
  "0 / 1+", the spec table keys on the full counts.
* **Table 7** — HH subbands, indexed by `h + v` and `d`
  (`context::significance_context_table7`). HH has no preferred
  orientation, so the horizontal and vertical counts are summed.
* the **HL context-template transpose** — for an HL subband §III.B
  reverses the roles of `h` and `v` before the Table 6 lookup
  (`significance_context_subband(.., is_hl = true)`).

The sign pass applies the matching HL axis-swap to the §III.B **Table 8**
prediction (`sign_context_subband` / `sign_prediction_flip_subband`).

The encoder threads its `wavelet_levels` into `BitPlaneInput::levels`; the
decoder reads `decomp_levels` from the segment header, so both sides
dispatch the identical contexts and the arithmetic coder stays in
lockstep. The change is a context-*selection* refinement, not a
wire-format change: filter-Q full-quality and colour decodes stay
bit-exact, and on the all-HH 64×64 checkerboard the spec-exact Table-7 HH
model now reaches lossless in fewer bytes than the uniform model.

The §III.B "neighbours from the *same segment of the subband*" rule is now
followed exactly: the bit-plane scanner walks each coefficient's eight
nearest **same-subband** neighbours (significance + sign), not the
spatially-adjacent cells. In the Mallat-interleaved coefficient buffer a
subband at decomposition level `j` is interleaved at stride `2^j` per axis,
so the same-subband neighbour of `(x, y)` in direction `(dx, dy)` is the
buffer position `(x + dx·2^j, y + dy·2^j)` (`priority::subband_stride`); a
neighbour stepped off the strip edge is "at the edge of its subband
segment" and treated as not-yet-significant (§III.B). The previous
spatial-raster walk sampled *cross-subband* neighbours, polluting the
context model; gathering genuinely correlated same-subband neighbours
shrinks the filter-Q lossless output materially (measured pre-r405,
before the §II.B pyramid-recursion fix moved every byte count):

| fixture (64×64) | spatial-raster | same-subband | reduction |
|-----------------|---------------:|-------------:|----------:|
| diagonal ramp   |          2076  |        1429  |  **−31%** |
| checkerboard    |          1839  |        1627  |    −11.5% |
| textured        |          3570  |        3226  |    −9.6%  |

The walk runs identically on encode and decode (the stride is a pure
function of `(x, y, levels)`), so the filter-Q full-quality + colour
round-trips stay bit-exact through the change.

## Stripe-ordered scan

The bit-plane scanner now uses **stripe-ordered** processing (IPN 42-155
§III.B): the image is partitioned into horizontal stripes of height 4 rows
(`STRIPE_HEIGHT = 4`). Within each bit-plane, the significance pass and
refinement pass each process one complete stripe before advancing to the next.
This maximises context-pattern locality.

## Multi-packet ordering

Each compressed segment now emits one packet pair per bit-plane per IPN
42-155 §IV:

* Packet 0 of bit-plane `bp`: significance + sign pass body (independently
  arithmetic-coded).
* Packet 1 of bit-plane `bp`: refinement pass body (independently coded).

For a segment with bit-plane count Q, the encoder emits `2*Q` packets in
MSB-first priority order. A decoder receiving a truncated stream can still
reconstruct a lower-quality image from the packets it received.

## Deadzone reconstruction (IPN 42-155 §III.A)

Reconstructing a subband from only its `q - b` most-significant bit planes is
*equivalent* to applying a deadzone scalar quantizer with bin width `∆ = 2^b`,
where `b` is the number of bit planes the decoder did **not** receive (a
byte-budget-truncated stream simply drops its trailing least-significant
packets). §III.A pins the reconstruction point per bin:

* the central deadzone bin `[-(∆-1), ∆-1]` (pixels that never became
  significant) reconstructs to the **origin**, `0`;
* every other bin `±[i∆, (i+1)∆-1]` reconstructs to `±((i + 1/2)∆ - 1)` -- the
  mid-bin value biased one step toward the origin (the wavelet detail
  coefficients are sharply peaked at zero, so the unbiased midpoint would
  over-shoot).

A decoded significant magnitude `mag` carries bits only down to plane `b`, so
`mag = i·∆` is the bin's *lower edge*; the decoder adds `∆/2 - 1 = 2^(b-1) - 1`
to reach the §III.A point. When the full stream is present (`b = 0`, `∆ = 1`)
the offset is zero and the magnitude is exact -- the lossless filter-Q
round-trip stays bit-identical.

### Per-coefficient deadzone width

`b` is **not** a single strip-global value. A byte-budget truncation cuts the
progressive packet stream at packet granularity, and a plane's significance and
refinement passes are *separate* packets emitted MSB-down (sig before ref). So
a cut frequently lands *between* a plane's significance and refinement packets:
`sig(bp)` is delivered but `ref(bp)` is dropped. The two coefficient classes
then have different deadzone widths:

* a coefficient made **newly significant** in the surviving `sig(bp)` knows its
  most-significant magnitude bit -- its deadzone is `b = bp`, `∆ = 2^bp`;
* a coefficient that was **already significant** at a higher plane but whose
  `ref(bp)` refinement bit was dropped is known only down to plane `bp + 1` --
  its deadzone is `b = bp + 1`, `∆ = 2^(bp+1)`, a bin twice as wide.

The decoder tracks, per coefficient, the deepest magnitude bit plane that was
actually delivered for it (a magnitude bit decode in either the significance or
the refinement pass), and applies each coefficient's own `∆/2 - 1` offset. A
missing refinement packet is *skipped entirely* (never decoded from an empty
body), so it cannot inject spurious magnitude bits, and the already-significant
coefficients keep their wider bin. On a clean bit-plane-boundary truncation
every coefficient shares the same `b`, so this reduces exactly to the earlier
strip-global behaviour; the gain is on the common mid-plane cut.

**Empirical PSNR gain on mid-plane cuts** (textured 64×64 image, filter Q,
3-level DWT, `with_byte_budget`; per-coefficient deadzone vs. the strip-global
single-`b` reconstruction; measured pre-r405 — both the r405 §II.B
pyramid fix and the r411 §II.A transform migration moved the packet
boundaries since, so treat the table as the historical demonstration of
the per-coefficient mechanism; `tests/truncation_fidelity.rs` carries
the current recalibrated floors. The rows below are the budgets whose
cut dropped a refinement packet at measurement time:

| output bytes | strip-global | per-coefficient |     Δ |
|-------------:|-------------:|----------------:|------:|
| 1325         |   22.63 dB   |     22.91 dB    | +0.28 |
| 1792         |   27.01 dB   |     28.42 dB    | +1.41 |
| 2258         |   31.62 dB   |     33.66 dB    | +2.05 |
| 2758         |   36.64 dB   |     39.30 dB    | +2.66 |
| 3275         |   41.16 dB   |     44.16 dB    | +3.00 |

The win grows with the number of dropped bit planes (larger `∆`) and is purely
a decode-side change -- the wire format is unchanged, so every previously-
encoded stream decodes better with no re-encode.

## Interleaved entropy coder (IPN 42-155 §IV)

ICER's entropy stage is **not** arithmetic coding. §IV specifies a
bit-wise adaptable *interleaved entropy coder*: a set of component
variable-to-variable-length binary source codes, one per probability
"bin", whose output codewords are interleaved into a single stream so the
decoder reconstructs them in the order the encoder produced them. The
`ixec` module implements it spec-exactly, built bottom-up:

* **Component codes** (`ComponentCode`, §IV.B) — a bijection between a
  prefix-free + exhaustive set of *input* codewords (source bits) and an
  equally prefix-free + exhaustive set of *output* codewords (channel
  bits). Encode parses one input codeword and emits its paired output;
  decode reverses the roles.
* **Golomb codes `G_m`** (§IV.B) — `m + 1` input codewords
  `1, 01, …, 0^(m-1) 1, 0^m`, with the published output mapping
  (`ℓ = ⌈log2 m⌉`, `i = 2^ℓ − m`; input `0^k 1` → the `ℓ`-bit binary of
  `k` when `k < i`, else the `(ℓ+1)`-bit binary of `k + i`; `0^m` → a
  single `1`). Verified bit-for-bit against the §IV.B **Table 9** G5
  listing.
* **Shorthand-tree codes** (§IV.D) — bins 2–8 are given as a decoding-tree
  shorthand (each leaf an input codeword, the root-to-leaf branch path its
  output codeword); the parser materialises the `(input, output)` pairs.
* **Table 10's 17-bin design** (`bins()`) — bin 1 uncoded, bins 2–8 the
  shorthand-tree codes, bins 9–17 `G5/G6/G7/G11/G17/G31/G70/G200/G512`,
  each with its probability cutoff `z_j` over the fixed denominator
  `2^16 = 65536`. `bin_for_probability` locates the bin for a
  probability-of-zero estimate `p ≥ 1/2` by integer cross-multiplication.
* **Interleaving machinery** (`InterleavedEncoder` / `InterleavedDecoder`,
  §IV.C) — the MER **2048-word** circular buffer, FIFO front-of-list
  emission that keeps the channel in word-creation order (the order the
  decoder reconstructs in), the §IV.C flush of partial words when the
  buffer fills or the input is exhausted, and the per-bin suffix
  bookkeeping the decoder uses to reverse it.

A context-driven `IxecEncoder` / `IxecDecoder` wraps the interleaver with
the same `encode_bit(symbol, p1_num, p1_den)` / `decode_bit(...)`
signature as the arithmetic coder, applying the §IV.C `p0 ≥ 1/2` reduction
(inverting the bit when `p0 < 1/2`) before bin selection. The bit-plane
significance / refinement / sign passes are written once against a
`entropy::{BitSink, BitSource}` trait surface, so the identical §III.B
pass logic — stripe order, contexts, the four-category scheme — drives
either backend; only the per-packet entropy coding differs.

```rust
let opts = EncodeOptions::compressed()
    .with_interleaved_entropy();         // code with the §IV coder
let bytes = encode_icer(&image, &opts)?;
let decoded = parse_icer(&bytes)?;       // decoder dispatches on the wire flag
```

The backend choice is recorded in a previously-reserved segment-header bit
(byte 7, bit 1), so **every existing arithmetic-coded stream parses
unchanged** (reserved = 0 ⇒ arithmetic) and decodes exactly as before; the
default remains the arithmetic coder. Filter-Q full-quality round-trips
(Gray8, colour 4:4:4, and multi-segment) are bit-exact through the
interleaved coder, and a budget-truncated interleaved stream frames the
full image geometry identically to the arithmetic path (progressive
truncation is a property of the packet ordering, not the entropy stage).

## Subband priority model (IPN 42-155 §III.A)

ICER does not finish one subband before starting the next: it
*interleaves* subband bit planes, always compressing next the subband
bit plane with the highest **priority** so that a stream truncated at any
point is the best image achievable for that many bytes (IPN 42-155
§III.A "Subband Quantization and Priority Factors").

Because ICER's wavelet transforms are not unitary, transform-domain MSE
is not reconstructed-image MSE. §III.A scales the transform to the
approximately-unitary form `l~ = sqrt(2)*l`, `h~ = (1/sqrt(2))*h` and
reads off per-subband priority weights (Fig. 7). For a `D`-stage
decomposition those weights are, with decomposition level `j` (`1` =
outermost/largest stage, `D` = deepest/smallest):

```text
    w(HH_j)           = 2^(j-2)
    w(HL_j) = w(LH_j) = 2^(j-1)
    w(LL_D)           = 2^D        (LL exists only at the deepest level)
```

For `D = 3` this reproduces Fig. 7 exactly (LL=8; level-3 HL/LH=4,
HH=2; level-2 HL/LH=2, HH=1; level-1 HL/LH=1, HH=1/2). The two worked
examples in §III.A both fall out of the formula and are pinned by tests:
LL has 16× (= 2^4) the weight of the level-1 HH subband, so the `i`th
LSB plane of level-1 HH ties in priority with the `(i+4)`th LSB plane of
LL.

The `priority` module turns this into the cross-subband encode order.
Each subband contributes `q` magnitude bit planes; each additional bit
plane halves the priority (subtracts 1 from the `log2` priority), so the
whole schedule lives on an exact integer `log2` scale (no float
comparison). `encode_order(decomposition_levels, bit_planes)` returns
every `(subband, bit-plane)` pair sorted by the §III.A rule set:

1. higher priority first;
2. ties broken by higher decomposition level;
3. then by subband type order `LL, HL, LH, HH`.

```rust
use oxideav_icer::{encode_order, SubbandType};
let plan = encode_order(3, 8);          // 3 stages, q = 8 bit planes
assert_eq!(plan[0].subband.kind, SubbandType::Ll); // LL MSB encoded first
assert_eq!(plan[0].bp_from_msb, 0);
// 25 subbands * 8 bit planes for D=3:
assert_eq!(plan.len(), (3 * 3 + 1) * 8);
```

The module is pure arithmetic over the subband geometry -- no entropy
coder or wire-framing coupling. As of r405 the packet emitter *can*
adopt this order end to end: `with_priority_interleaving()` (next
section). The default emitter keeps the historical per-segment
MSB-down packet pairs for wire compatibility.

## Subband-priority interleaving (IPN 42-155 §III.A)

`EncodeOptions::with_priority_interleaving()` switches a compressed
segment's packet schedule to the spec's progressive order, coded
exactly as §III describes:

* subband bit planes are walked in decreasing §III.A priority
  (`priority::priority_groups`), ties to the higher decomposition
  level then `LL, HL, LH, HH`;
* each subband bit plane is coded in **one combined raster pass** over
  its subband lattice ("the bits within a segment are compressed in
  raster scan order", §III) — a still-insignificant coefficient
  contributes a significance bit with its **sign immediately after the
  first nonzero magnitude bit**; an already-significant coefficient
  contributes a refinement bit;
* the context model persists across the whole segment (§III: the
  probability estimate "relies only on previously encoded information
  from the same segment"); each packet body is a fresh entropy-coder
  run so packet boundaries stay byte-aligned;
* packets follow `priority::packet_schedule`: one per priority group,
  with *fat* subband bit planes (> `FINE_PACKET_COEFFS` = 256
  coefficients) cut into their own packets so a byte-quota truncation
  lands close to its exact §III.A schedule position. Both sides
  recompute the schedule from header fields alone; the packet header
  carries the priority-group index in its `bit_plane` field.

```rust
let opts = EncodeOptions::compressed()
    .with_priority_interleaving()       // §III.A progressive order
    .with_byte_budget(2_000);           // quota cut on a priority boundary
let bytes = encode_icer(&image, &opts)?;
let decoded = parse_icer(&bytes)?;      // decoder dispatches on the wire flag
```

The mode rides a previously-reserved header bit (byte 2 bit 7 — the
top of the old 4-bit filter field, zero on every pre-existing stream),
so all historical streams parse and decode unchanged. Filter-Q
full-quality round-trips are bit-exact across geometries, levels, both
entropy backends, colour 4:4:4, row-strip and §V.B transform-domain
segmentation; §VI.A `min_loss` composes at schedule granularity (whole
subband bit planes drop out of the schedule on both sides);
`rd_pruning` is rejected (two competing schedulers — the §III.A order
already places every packet at its distortion-priority position).

**Measured** (textured 64×64, filter Q, 3 levels, equal `byte_budget`,
PSNR of the truncated decode vs the whole-strip MSB-down order;
re-measured r411 on the §II.A transform):

| budget (B) | MSB-down | §III.A interleaved | gain |
|-----------:|---------:|-------------------:|-----:|
| 250  | 16.07 dB | 26.50 dB | **+10.43** |
| 500  | 22.09 dB | 28.75 dB | **+6.66** |
| 750  | 27.54 dB | 30.63 dB | +3.09 |
| 1000 | 27.54 dB | 31.49 dB | +3.95 |
| 1500 | 33.24 dB | 36.97 dB | +3.73 |
| 2000 | 39.71 dB | 40.54 dB | +0.83 |
| 2500 | 41.31 dB | 46.74 dB | +5.43 |
| 3000 | 44.40 dB | 53.20 dB | +8.80 |

Mean +5.4 dB, every point positive (the pre-r411 sweep had one
"scalloping" outlier; under the §II.A transform the schedule wins
everywhere on this fixture). Deep truncation — the deep-space
downlink case §III.A exists for — wins decisively. Cost: ~+2.2% on
the *lossless* byte count (3218 vs 3149 B on the same fixture) from
the finer packetisation (more headers + entropy flushes);
pure-lossless archival should keep the default order.

## Transform-domain segmentation (IPN 42-155 §V.B + §V.D)

The spec's error-containment segmentation happens **after** the wavelet
decomposition: "first the LL subband is partitioned into s rectangular
segments, and then this partition is mapped to the other subbands.
Thus, pixels in different subbands corresponding to the same spatial
location will belong to the same segment" (§V.B, Fig. 14). In the
crate's Mallat-interleaved coefficient layout that mapping is a single
integer shift — coefficient `(x, y)` belongs to the §V.D rectangle
containing LL pixel `(x >> D, y >> D)` (`coefficient_segment_map`).

```rust
let mut opts = oxideav_icer::EncodeOptions::compressed()
    .with_transform_domain_segments();
opts.segment_count = 8;                  // §V.D partition of the LL subband
let bytes = oxideav_icer::encode_icer(&image, &opts)?;
let decoded = oxideav_icer::parse_icer(&bytes)?;   // recomputes the partition
```

Wire form: one segment per §V.D rectangle, each header carrying the
full image dimensions plus `(total_segments, segment_index)` in the
previously-reserved framing space, flagged by the last reserved header
bit so every pre-existing row-strip stream parses unchanged. Segment
boundaries are never encoded — the decoder recomputes them "from the
image dimensions, the number of stages of wavelet decomposition, and
the total number of segments" (§V.D). Each segment is coded with its
own context modeler and entropy coder (§V.B) over the coefficients the
partition maps to it; §III.B same-segment neighbour semantics hold
automatically because out-of-segment coefficients never enter the scan
state.

Properties (all pinned by `tests/transform_segments.rs`):

* Filter-Q full-quality decode is **bit-exact** across geometries,
  segment counts, both entropy backends, and colour 4:4:4.
* Whole-image decorrelation edges the row-strip convention at equal
  segment count: textured 128×128, filter Q lossless, 8 segments emits
  **13014 B vs 13016 B** (re-measured r411 on the §II.A transform;
  the spec-exact pyramid decorrelates row strips nearly as well, so
  the gap is small); and §V.B's strip-boundary artifact
  ("Fig. 15(a)") cannot occur because there is only one transform.
* **Error containment**: dropping any one segment leaves the loss
  confined to the segment plus a *decaying* wavelet-support bleed —
  the §II.A inverse eq (3) recursion (`d[n] = h[n] + predictor(...,
  d[n+1])`) propagates a floor-truncated, geometrically-decaying tail
  rather than a hard support cutoff. Pinned profile: residual at most
  ±4 grey levels beyond a `3·2^D` dilation of the segment's
  image-domain rectangle, **bit-identical** beyond `8·2^D`; the lost
  region itself degrades to a smooth low-detail patch (zero
  coefficients through the shared inverse transform), not a flat-128
  strip. The lenient decoder no longer needs segment 0 — *any*
  surviving segment pins the full geometry.
* Byte budgets compose: whole segments are scheduled in ROI-priority
  order, each internally truncated MSB-down, dropped segments leave
  placeholder headers, and the full image geometry is always framed.
* §V.D eq (9) (`s <= LL pixel count`) is enforced at encode and
  decode; the wire caps the count at 255 (the MER build allows at most
  32 segments, §V.C).

Cost: each segment's scan is confined to its own bounding window (the
§V.B block is contiguous in the interleaved buffer —
`SegmentRect::image_window`), with the stripe grid kept `y = 0`-aligned
so the visit order — and therefore the wire form — is byte-identical to
a full-image walk. Transform-domain encode runs at row-strip parity on
the criterion 64×64 s = 4 pin (~373 µs vs ~378 µs; the window trim
bought −29% over the initial full-buffer scan).

## Minimum-loss quality goal (IPN 42-155 §VI.A)

ICER's two rate-control knobs are the **byte quota** and the **quality
goal** (§VI): "ICER stops producing compressed bytes once the quality
goal or byte quota is met, whichever comes first." The quality goal is
the *minimum loss* parameter `M`: a nonnegative integer such that each
subband's `max(0, M - offset)` least-significant magnitude bit planes
are never encoded, with the per-subband relative-importance offsets of
Fig. 18 (derived from the Fig. 7 priority weights):

```text
    offset(HH_j)              = j - 1
    offset(HL_j) = offset(LH_j) = j
    offset(LL_D)              = D + 1
```

`priority::min_loss_offset` reproduces the published D = 3 figure
exactly (LL = 4; level-3 HL/LH = 3, HH = 2; level-2 HL/LH = 2, HH = 1;
level-1 HL/LH = 1, HH = 0). `M = 0` is lossless when the quota allows;
if all but `k` planes of a subband are encoded, its pixels are in
effect quantised with step `2^k` and reconstruct at the §III.A
deadzone points; `M >= B + D` encodes almost nothing (§VI.A).

```rust
let opts = oxideav_icer::EncodeOptions::compressed()
    .with_min_loss(3)                    // quality goal
    .with_byte_budget(4096);             // quota — whichever binds first
```

`M` rides the previously-reserved byte of **every packet header** so a
loss-tolerant decoder learns it from any surviving packet; 0 is the
historical value, so every pre-existing stream decodes unchanged, and
`with_min_loss(0)` is proven byte-identical to the plain encode.
Measured byte curve (textured 128×128, filter Q, 3 levels;
re-measured r411 on the §II.A transform):

| M | 0 | 1 | 2 | 3 | 4 | 6 | 8 |
|---|--:|--:|--:|--:|--:|--:|--:|
| bytes | 12246 | 11718 | 9935 | 7707 | 5205 | 994 | 274 |

Composes with row strips, §V.B transform-domain segments, both entropy
backends, and the byte quota; monotone in bytes and MSE by test.
(`min_loss + rd_pruning` is rejected — two different rate-control
modes.) The 3-D cube path has carried its own §IV.B (IPN 42-164)
min-loss since r383; this is the 2-D §VI.A realisation.

## What is *not* implemented

* **Real-world bitstream interop**. The context model follows IPN 42-155
  §III.B / §III.C exactly — the Table 6/7 per-subband significance contexts
  with the HL transpose, the Table 8 sign prediction, the four-category
  magnitude scheme, and the §III.C MER probability estimator (2/4 init,
  rescale at 500). The §IV **interleaved entropy coder** is now implemented
  spec-exactly (Table 10's 17-bin design, the §IV.B Golomb codes, the §IV.D
  shorthand-tree component codes, and the §IV.C 2048-word circular-buffer
  interleaving with flush bits) and is a selectable encode/decode backend
  (`EncodeOptions::with_interleaved_entropy()`) — see "Interleaved entropy
  coder" below. The crate still defaults to the Witten-Neal-Cleary binary
  arithmetic coder for backward-compatible wire output; the remaining
  interop unknown is the **bit ordering / packetisation conventions** of a
  specific Mars-rover ICER build (the §IV component codes are pinned, but
  the paper leaves the sync-prefix value and the exact packet framing to
  "the implementation"), so neither backend is yet guaranteed
  bit-equivalent to a particular JPL ICER file.
  (The earlier §III.B same-subband-neighbour approximation is **resolved** --
  see "Per-subband context tables" above; the scanner now walks the
  spec-exact same-subband neighbourhood.)
* **Chroma subsampling**. Colour support (`IcerPixelFormat::Yuv444P`) is
  implemented for co-sited 4:4:4 (see "Colour images" below). Subsampled
  layouts (4:2:2 / 4:2:0) and the RGB↔YCbCr colour-transform stage are
  deferred -- the deployed Mars-rover Bayer/colour pipeline applies the colour
  transform *before* ICER, which sees three already-decorrelated 4:4:4 planes.

## Documentation gaps

The 2003 IPN 42-155 paper deliberately leaves several implementation details
to "the implementation":

* The **literal sync-prefix value** (§IV mentions "self-synchronising prefix"
  but doesn't pin a 16-bit value). This crate accepts any non-zero 16-bit
  prefix and surfaces it via `SegmentHeader::sync_prefix`.
* The **probability estimator window size** for the adaptive arithmetic coder.
  *Resolved (r365).* §III.C's MER implementation in fact pins the values:
  "the initial counts of zeros are set to 2, the initial total counts are set
  to 4, and rescaling is triggered when the total count reaches 500." The
  estimator now uses exactly these (`INITIAL_ONES = 2`, `INITIAL_TOTAL = 4`,
  `RESCALE_THRESHOLD = 500`), with the rescale rounding chosen "in the
  direction that makes the probability estimate closer to 1/2" per §III.C.
  (The round-1 placeholder used a 64-symbol window.)
* The **per-subband significance context tables** (§III.B Table 6 for
  LL/LH/HL and Table 7 for HH, plus the HL context-template transpose).
  **Resolved (r365).** The bit-plane scanner is now subband-aware: each
  coefficient's `(SubbandType, level)` is resolved via
  `priority::classify_position`, and the significance pass selects the
  spec-exact §III.B **Table 6** (LL/LH/HL, keyed on the full `(h, v, d)`
  counts) or **Table 7** (HH, keyed on `h + v` and `d`), with the **HL
  context-template transpose** (swap `h`/`v` before the Table 6 lookup).
  The sign pass applies the matching HL axis-swap to the Table 8
  prediction. The sign contexts already followed §III.B **Table 8
  exactly**, and the four-category magnitude scheme (contexts 9/10/11 +
  category-3 uncoded) was implemented in r359. The §III.B "neighbours
  from the *same segment of the subband*" rule is now followed exactly
  (r368): the scanner walks each coefficient's eight nearest same-subband
  neighbours at the subband stride `2^level`, with off-strip neighbours
  treated as not-significant per §III.B; the spatial-raster approximation
  is retired.
* **Per-filter lifting coefficients** -- *resolved.* IPN 42-155 §II.A
  Table 1 publishes the parameters for all seven filters; `wavelet_int`
  transcribes them and (since r411) the whole pipeline runs on it.

## Automatic filter selection

ICER's seven wavelet filters (A-F plus Q) have different
rate-distortion and lossless-rate profiles depending on image content
(IPN 42-155 §II.D "Quantitative Filter Comparison" shows the two
rankings even differ from each other). Every filter is losslessly
reversible, so the choice only moves the byte count and the truncated
progressive quality. Two ways let the encoder pick the filter:

* **Heuristic** (`EncodeOptions::with_auto_filter()`): a one-pass scan
  computes image statistics (mean, variance, horizontal + vertical
  gradient energy, dynamic range) via `analyze::ImageStats::from_image`,
  then `analyze::recommend_filter` runs a transparent decision tree:
  flat / low-frequency content gets filter `Q`; high-frequency,
  high-variance content gets filter `A` (same `alpha` triple as Q,
  `beta = 0`, §II.A Table 1).
  Cost: `O(width*height)`, single encode pass.
* **Rate-distortion** (`EncodeOptions::with_auto_filter_rd()`):
  trial-encodes the image once per candidate filter and returns the
  byte-smallest result. The default candidate set is `[Q, A]`; callers
  may pass any subset to `pick_filter_by_rate_distortion`. Cost:
  `N x encode_time` where `N = candidates.len()`.

Both modes compose with `with_byte_budget` / `with_target_bytes` -- the
auto-selected filter is then passed through the existing quota path.

```rust
let opts = EncodeOptions::compressed()
    .with_auto_filter_rd()              // try Q and A, pick smaller
    .with_byte_budget(8192);            // hard cap on output bytes
let bytes = encode_icer(&image, &opts)?;
```

## ROI segment prioritisation

IPN 42-155 §III.E specifies that ICER's image partitioning (the
`segment_count` row-strip split) gives the encoder freedom to
*schedule* segments independently. The Mars rover deployments use this
to save the centre of a Pancam frame -- where the science target sits
-- ahead of the periphery (sky / rover hardware at the edges) when the
downlink budget is tight.

That freedom is surfaced as two `EncodeOptions` builders:

* **`with_segment_priorities(Vec<u16>)`** -- supply a per-segment
  priority vector. Entry `prios[seg_idx] = rank` means segment
  `seg_idx` is emitted in `rank`-ascending order on the wire (rank 0
  first). The vector must be a permutation of `0..segment_count`.

* **`with_center_roi()`** -- convenience builder that constructs a
  centre-out permutation for the current `segment_count`: the middle
  segment is rank 0, then alternating outward (mid-1, mid+1, mid-2,
  mid+2, ...). For 5 segments the priority vector is `[3, 1, 0, 2,
  4]`; for 4 segments it is `[1, 0, 2, 3]`.

The on-the-wire byte stream contains every segment in priority order
*plus* zero-body placeholder headers for any segment the byte budget
forced the encoder to drop. The decoder (`parse_icer`) already sorts
segments by `segment_index` before stitching, so the priority ordering
is transparent on decode; dropped segments reconstruct as flat 128
(level-shifted zero coefficients).

```rust
let opts = EncodeOptions::compressed()
    .with_byte_budget(220)              // very tight budget
    .with_center_roi();                 // centre-first emission
let bytes = encode_icer(&image, &opts)?;
// Decode: dropped strips materialise as flat 128, centre strips
// keep their fidelity.
let decoded = parse_icer(&bytes)?;
```

**Empirical measurement** (128-row image, 4 segments, 900-byte budget
— re-measured r411 on the §II.A transform; centre + periphery
mean-absolute error vs. original):

| metric          | value |
|-----------------|------:|
| centre band MAE | 34.13 |
| periphery MAE   | 84.00 |

The centre band's MAE is ~60% lower than the periphery's; under
index-order emission the same budget would distribute the loss
uniformly (or worse, drop the centre while saving the unimportant
edges).

The priority vector is validated at encode-time (length match,
permutation property); a malformed vector returns
`IcerError::Unsupported`.

**Empirical byte counts** (32x32 test inputs, default DWT depth, q=8;
re-measured r411 on the §II.A transform — every cell is lossless):

| image          | filter Q | filter A | heuristic pick | RD pick |
|----------------|---------:|---------:|----------------|--------:|
| flat           |     108  |     108  | 108 (=Q)       |    108  |
| diagonal ramp  |     149  |     145  | 149 (=Q)       |    145  |
| checkerboard   |     393  |     325  | 325 (=A)       |    325  |

On the checkerboard the heuristic's high-edge-energy pick (filter A)
is now also the byte winner — under the spec-exact integer transforms
filter A's `beta = 0` predictor genuinely codes the all-HH content in
fewer bytes, and there is no lossless-vs-lossy trade left to weigh.
The ramp shows the residual gap the heuristic accepts: it stays on Q
(149 B) where RD finds A 4 bytes smaller. Use the heuristic when you
want zero per-image overhead; use RD-mode when you want the true
minimum.

## Rate-distortion budget pruning

The quota-controlled encoder truncates packets in strict
MSB-down emission order: it emits sig(0), ref(0), sig(1), ref(1),
... and stops the moment the next packet would exceed the byte cap.
That semantic is byte-honest but R-D-suboptimal: the truncation cut
often falls on a refinement packet whose distortion-reduction-per-
byte is poor compared to *other* packets later in the chain that
would have fit individually but for the budget being already spent.

A true rate-distortion (R-D) packet selector is available
(`EncodeOptions::with_rd_budget(n)`) per IPN 42-155 §IV.B
rate-allocation principle. The selector:

1. Encodes every `(bit-plane, pass)` packet in one shot, recording
   each packet's `delta_distortion` -- a clean-room MSE-reduction
   estimate (mid-bin variance argument; `~2 * 4^bp` per
   newly-significant coefficient, `~4^bp / 4` per refined bit).
2. For every candidate sig-chain depth K (0..q), computes the
   mandatory cost of `sig(0..=K)` and the residual budget.
3. Greedily fills the residual with refinement packets at bit-plane
   indices `0..=K`, sorted by `delta_distortion / wire_size`
   descending.
4. Picks the depth K (and its refinement subset) that maximises
   total distortion-reduction within the byte cap; ties broken in
   favour of smaller output bytes.

The selector enforces ICER's MSB-down decode dependency (sig at
bp_idx N requires sig at every higher-priority bp_idx; ref at N
requires the full sig chain up to N) so the kept subset always
decodes correctly. The output is emitted in MSB-down on-the-wire
order -- only the *set* of included packets changes vs. the strict
mode, not the ordering of those kept.

**Empirical win** (single-segment compressed encode, filter Q,
64×64 sparse-impulse fixture, re-measured r411 on the §II.A
transform):

| fixture        | budget | strict-MSB | R-D              |
|----------------|-------:|------------|------------------|
| sparse impulse | 250 B  | 46.01 dB   | **50.49 dB**     |

The +4.5 dB win comes from the selector recognising that the strict
cut's tail bytes go to a wide low-value significance packet while the
high-value refinement packets that fit individually are skipped; R-D
reallocates the residual budget to them. The greedy plan is validated
against the strict plan by actual decoded distortion, so R-D never
regresses.

For monotonically-ordered images (smooth gradients, natural
textures) the R-D selector converges to strict-MSB plus trims the
tail of zero-ΔD packets (5-bytes-per-packet savings only). R-D
never regresses PSNR vs strict; the headline gains are on
high-frequency content where the natural MSB-down order is
score-non-monotonic.

```rust
let opts = EncodeOptions::compressed()
    .with_rd_budget(400);                // hard cap + R-D selection
let bytes = encode_icer(&image, &opts)?;
assert!(bytes.len() <= 400);
```

The R-D mode composes with `with_auto_filter` and
`with_segment_priorities`; the R-D selection runs *inside*
`encode_one_segment_compressed`, after the wavelet transform and
bit-plane encode, so per-segment dependencies are honoured
naturally.

## Lenient multi-segment decode

The Mars-rover deep-space link is lossy: ICER segments can be dropped
in transit between the orbiter relay and the DSN ground station.
IPN 42-155 §III.E "Image Partitioning" makes each segment a
self-contained, independently-decodable unit precisely so that the
receiver can still recover most of the image from whatever survived.

`parse_icer` enforces a contiguous `segment_index` sequence and
rejects a stream missing any segment with
`IcerError::invalid("non-contiguous segment indices: ...")`. The lenient
counterpart:

* **`parse_icer_lenient(bytes)`** -- accept a stream that may be
  missing entire segments. Missing strips are reconstructed as flat
  128 (level-shifted zero, identical to the ROI-priority placeholder
  semantic the encoder produces under tight byte budgets). The returned [`LenientDecode`] report carries the
  per-index presence map and the missing-segment count.

```rust
let lenient = oxideav_icer::parse_icer_lenient(&bytes)?;
assert_eq!(lenient.received[2], false); // segment 2 was lost in transit
assert_eq!(lenient.missing_count, 1);
// `lenient.image` is the reconstructed image with segment 2's strip
// filled flat 128.
```

Constraints:

* Segment 0 must be present (it pins the canonical strip height + the
  canonical width). A missing segment 0 returns `IcerError::Truncated`.
* The canonical strip height is read from segment 0; non-trailing
  received segments must agree on that height (matches `encode_icer`'s
  `div_ceil(h, segment_count)` row-strip split, where every strip
  except the last has identical height).
* Width mismatch among received segments still surfaces as
  `IcerError::Unsupported` -- that's a geometry contradiction, not a
  loss-tolerance scenario.
* A trailing-segment drop (e.g. segment N missing when it was the
  last) truncates the image at the end of segment N-1; the receiver
  has no way to detect that a higher-indexed segment was supposed to
  exist (the wire format carries no total-segment-count field).

Composes with `DecodeLimits` (the DoS-cap policy applies
identically via `parse_icer_lenient_with_limits`) and with every
encoder path (filter Q / filter A / uncompressed §III.D).

## Quality-target rate-control

The quota-controlled encoder lets the caller pin a byte budget and reports
back whatever quality the truncation yields. The inverse shape: pin a quality (PSNR floor) and let the encoder
report back the smallest byte count that meets it.

```rust
let opts = EncodeOptions::compressed()
    .with_quality_target(30.0);                 // PSNR floor in dB
let bytes = encode_icer(&image, &opts)?;
let decoded = parse_icer(&bytes)?;
let achieved = oxideav_icer::analyze::psnr_db(&image, &decoded);
assert!(achieved >= 30.0);
```

Algorithm:

1. Compute the search bracket via `analyze::quality_search_bounds`:
   * `lo_bytes` = `segment_count * SegmentHeader::ENCODED_BYTES`
     (header-only floor; finer-resolution encodes are nonsensical).
   * `hi_bytes` = byte count of an unbudgeted compressed-path encode
     under the resolved filter (the encoder cannot synthesise quality
     above this ceiling).
2. Upper-bracket trial: if the unbudgeted encode misses `target_db`,
   return it as the best effort (no smaller encode can meet the
   target). Since r411 the unbudgeted compressed encode is lossless
   under every filter (PSNR `+inf`), so this branch fires only in
   principle; every finite target is reachable.
3. Lower-bracket trial: if the floor encode already meets the target
   (lossless filter Q on a flat or near-flat input lands here),
   return it.
4. Binary search the byte budget. Each iteration encodes at the mid
   budget, decodes, computes PSNR. PSNR >= target -> record as the
   new best and search the lower half; PSNR < target -> search the
   upper half. Stops when the bracket falls below `BISECT_TOL = 8`
   bytes (~one packet header) or after `MAX_ITERATIONS = 48` steps.

Costs roughly `log2((hi_bytes - lo_bytes) / 8)` encode-then-decode
trials. Mutually exclusive with `with_byte_budget` / `with_target_bytes`
/ `with_rd_budget` (the search manages the byte budget directly);
combining returns `IcerError::Unsupported`. On the uncompressed path
(`EncodeOptions::default`) the round-trip is bit-exact by construction
so every finite target is satisfied trivially -- the quality-target
flag is then a no-op.

Composes with `with_auto_filter` / `with_auto_filter_rd`: the search
runs after filter resolution, so each trial uses the chosen filter.

## Roadmap

One piece is deliberately not in the current rounds — it is core
to ICER's value proposition for deep-space imaging:

### Quota-controlled encoding

`encode_icer(image, &opts)` now supports truncation to a caller-
specified byte budget. ICER's signature feature — **emitting
progressive packets MSB-down and stopping once the quota is
exhausted** — is fully implemented. Anything not yet emitted simply
isn't transmitted; the decoder reconstructs at whatever quality the
truncation point allows. This is exactly how the Mars rovers compress
every image.

On a multi-segment encode (row strips or §V.B transform-domain) the
quota is applied per IPN 42-155 §VI.B: the packet stream is the
*global interleave* across segments (each segment of a bit plane
before the next bit plane), the quota truncates that interleaved
stream, and the surviving packets are rearranged per segment for
transmission (Fig. 23(b)) — so every segment receives its
most-significant packets and quality is monotone in the budget.
Measured on the textured 64×64 4-strip fixture the §VI.B interleave
gains +8.8..+16.3 dB over the earlier sequential-greedy allocation at
equal mid-range budgets (see `tests/quota_interleave.rs`). ROI
`segment_priorities` / `with_center_roi` keep the sequential
whole-segment scheduling — starving the periphery is that mode's
point.

```rust
let opts = EncodeOptions::compressed()
    .with_byte_budget(8192);          // hard cap
let bytes = encode_icer(&image, &opts)?;
assert!(bytes.len() <= 8192);

// Soft target + hard cap:
let opts = EncodeOptions::compressed()
    .with_target_bytes(8192)          // soft target — finish current bit-plane pair
    .with_byte_budget(10_000);        // hard cap — never exceeded
let bytes = encode_icer(&image, &opts)?;
assert!(bytes.len() <= 10_000);
```

Semantics:

* **Hard cap** (`with_byte_budget(n)`): before starting each packet the
  encoder checks whether the total output (segment header + all packets
  so far + next packet) would exceed `n`. If so, it stops immediately.
  The output is strictly ≤ `n` bytes.
* **Soft target** (`with_target_bytes(n)`): once the running output
  meets or exceeds `n`, the encoder finishes the current bit-plane's
  packet pair (significance + refinement), then stops. Output may be
  slightly above `n`.
* **Combined**: soft target controls *when* to decide to stop; hard cap
  enforces the absolute ceiling even within the finishing pair.

Mid-packet truncation is deferred (needs the IPN 42-155 supplemental
[13] reference tables — see "Documentation gaps" below).

### ICER 3D

**Implemented** (round 383; §V.D transform-domain segmentation, the
cross-segment progressive quota, and the §III.B dynamic-range analysis
landed r414) from the staged IPN Progress Report 42-164 (2006) -- see
the "ICER-3D" section above. The remaining 3-D delta is the same
interop unknown as the 2-D path: the papers leave the byte-level
container to the implementation.

## Standalone vs registry build

The default `registry` feature wires up the `oxideav-core`
`Decoder` / `Encoder` trait surface plus the `register()` entry
point. Image-library consumers that want to decode ICER without
pulling in the framework should depend on the crate with
`default-features = false`:

```toml
[dependencies]
oxideav-icer = { version = "0.0", default-features = false }
```

The standalone API is `parse_icer(bytes) -> Result<IcerImage>`,
`parse_icer_metadata(bytes) -> Result<IcerMetadata>`, and
`encode_icer(image, &EncodeOptions) -> Result<Vec<u8>>`. Compressed
mode is opt-in via `EncodeOptions::compressed()`; multi-segment
encode is opt-in via `EncodeOptions::segment_count`.

## Fuzzing

A cargo-fuzz harness under `fuzz/` runs every byte slice through
three decode-side entry points and asserts none panic / abort /
OOM:

* `header::walk_segment` -- single-segment framing parse.
* `parse_icer_metadata` -- multi-segment walk (header-only).
* `parse_icer` -- full decode (arith coder + inverse DWT + stitch).

The seed corpus under `fuzz/corpus/decode_segment/` is generated
from icer's own encoder: an uncompressed ramp, compressed filter-Q
ramps + flat, filter-A and filter-F §II.A streams, a two-segment
split, byte-budget-truncated inputs covering the partial-packet path,
§V.B transform-domain + §VI.A minimum-loss + §III.A
priority-interleaved wire modes, §VI.B budget-truncated multi-segment
streams (row-strip and transform-domain), ICER-3D cubes (row-strip
plus the r414 §V.D transform-domain mode: plain, quota-truncated, and
interleaved-entropy + min-loss), and deep-sample (tag-2 container)
streams at 10/12/16 bits covering the compressed, §V.B, §III.D raw,
quota-truncated, and priority-interleaved deep wire forms (r433). The
`encode_roundtrip` target also synthesises `GrayDeep` images at
fuzz-chosen depths with deliberately unmasked (out-of-range) sample
words.

```bash
cd fuzz
cargo +nightly fuzz run decode_segment -- -max_total_time=60
```

Local bounded runs on the r433 pipeline (deep-sample wire + §II.C
analysis): `decode_segment` 22353 iterations / 180 s and
`encode_roundtrip` 6376 iterations / 240 s, 0 findings after fixes.
(The historical geometry-DoS surface — a 12-byte header requesting
~4 GB of decode allocation — is closed by the `DecodeLimits` caps
below.) The r433 campaign additionally surfaced — and the same round
fixed — a **lenient-decode allocation hole**: the gap-filled
reconstruction spans the full `0..=max_received_index` strip range, so
two tiny received strips at a huge `segment_index` gap bought a
multi-GB placeholder allocation the received-pixel sum never counted.
`parse_icer_lenient*` now caps the *reconstruction geometry* against
`DecodeLimits::max_total_pixels`
(`fuzz/corpus/decode_segment/seed_lenient_index_gap_oom.bin` +
`tests/lenient_decode.rs` pin the refusal).

A daily fuzz run lives at `.github/workflows/fuzz.yml` (30-minute
budget; OxideAV reusable workflow).

`tests/mutation_smoke.rs` additionally gives every push a bounded,
deterministic corruption sweep (single-byte flips, exhaustive
header-field value sweeps, every truncation point, cross-seed splices)
over the §V.B transform-domain, §VI.A minimum-loss, §III.A
priority-interleaved, ICER-3D §V.D, and deep-sample (tag-2 container)
wire forms. On
its first run it caught two decode-side panics reachable from corrupted
streams (bit-plane counts near 31 decode coefficients near `i32::MAX`):
the 5/3 lifting inner neighbour sums and the inverse level shift both
overflowed in debug — the lifting sums now wrap and the level shift
saturates. Corpus seeds for the new wire modes (`seed_transform_*`,
`seed_minloss*`) feed both the scheduled fuzz run and the per-push
corpus smoke.

## Decode-side resource limits

The cargo-fuzz decode harness surfaced a DoS vector inherent
to the wire format: the 12-byte segment header carries `width` and
`height` as `u16` each, which means a 12-byte input can declare up
to `65535 * 65535 ≈ 4.29 GPx` per segment. Without a cap,
`parse_icer` would dutifully allocate a ~4 GB plane plus
~16 GB of `i32` coefficient buffers before discovering the body was
empty.

An application-level geometry cap is available via
[`DecodeLimits`]:

```rust
let limits = oxideav_icer::DecodeLimits::default();
//         max_pixels_per_segment = 64 MPx
//         max_total_pixels       = 256 MPx
let img = oxideav_icer::parse_icer_with_limits(bytes, &limits)?;

// Trusted-input batch path — uncapped:
let img = oxideav_icer::parse_icer_with_limits(
    bytes,
    &oxideav_icer::DecodeLimits::unlimited(),
)?;
```

The bare-name `parse_icer` / `parse_icer_metadata` entry points
apply `DecodeLimits::default` automatically — every existing caller
gets the conservative policy without an API change. Callers who
need a different policy (oversized HiRISE-style strips, trusted
batch processing) use the `_with_limits` variants explicitly.

The defaults (64 MPx per segment, 256 MPx total) sit two orders of
magnitude above every published Mars-rover Pancam / Hazcam / Mastcam-Z
delivery frame and three orders of magnitude below the 4 GB
wire-format ceiling, so realistic inputs are unaffected and synthetic
worst-case inputs are rejected with `IcerError::Unsupported` before
any plane allocation.

A segment that exceeds the per-segment cap, or a multi-segment image
whose stitched pixel count exceeds the total cap, returns
`IcerError::Unsupported` (a deliberate application-policy refusal,
not a wire-format error). The metadata walker
(`parse_icer_metadata`) and the full decoder (`parse_icer`) apply
the same cap, so an attacker cannot bypass the policy by stopping at
the metadata stage.

## Per-segment uncompressed fallback

IPN 42-155 §III.D "Performance with Difficult Imagery" specifies
that the encoder may bypass the entropy stage on a per-segment basis
when arithmetic coding would *expand* the payload -- pure-noise tiles,
single-pixel transitions on otherwise flat backgrounds, and other
content where the per-packet header plus arithmetic-coder per-packet
renormalisation overhead exceeds what the entropy stage can squeeze
out. The §III.D path ships raw 8-bit pixels in a single packet under
the same segment-framing layer, with the wire-format
`SegmentHeader::uncompressed` flag (1 bit) telling the decoder which
path was taken.

This is surfaced as an `EncodeOptions` builder:

* **`with_uncompressed_fallback()`** -- enable per-segment fallback.
  When set, the compressed-path encoder produces *both* candidates
  (compressed + uncompressed) for each segment and emits whichever is
  smaller. The decision is recorded per-segment on the wire, so a
  multi-segment image may mix paths (noisy strip -> uncompressed,
  smooth strip -> compressed) transparently.

```rust
let opts = EncodeOptions::compressed()
    .with_uncompressed_fallback();
let bytes = encode_icer(&image, &opts)?;
// Decoder reads each segment's `uncompressed` flag and reconstructs
// accordingly -- no caller-side awareness needed.
let decoded = parse_icer(&bytes)?;
```

Compose-rules:

* The fallback only fires on the compressed path
  (`opts.uncompressed = false`). Forcing `opts.uncompressed = true`
  short-circuits the comparison and emits uncompressed
  unconditionally (the default).
* The wire-format §IV per-segment body-length ceiling is `u16::MAX
  = 65535` pixels; strips that exceed that cap can't be shipped raw
  and keep the compressed result with no error.
* Equal-length ties go to compressed: the entropy-coded packets are
  strictly more useful to a truncating decoder than the raw dump.
* Composes with `auto_filter_rd` (each filter candidate is offered
  the fallback choice independently), `byte_budget` / `target_bytes`
  / `rd_pruning` (compressed candidate honours the budget;
  uncompressed candidate is fixed-size), and `segment_priorities`
  (priority order is independent of path choice).

The per-segment behaviour is tested in
`tests/uncompressed_fallback.rs` against an LCG-driven noise tile
(strict fallback win), a smooth diagonal ramp (compressed stays),
and a stacked noise/ramp image (each strip independently decides).

## Colour images

IPN 42-155 §III describes ICER as fundamentally a **single-component**
coder; the deployed colour scheme runs one independent ICER instance per
colour component, sharing only the outer image metadata. This crate models
that exactly. An `IcerPixelFormat::Yuv444P` image is encoded as **three
independent single-plane ICER bitstreams** (luma + Cb + Cr, co-sited
4:4:4), each carrying its own segments / packets / arithmetic-coded bodies,
concatenated behind a small multi-plane container header (the
`plane_container` module).

```rust
use oxideav_icer::{encode_icer, parse_icer, EncodeOptions, IcerImage, IcerPixelFormat};

let img = IcerImage::zeros(64, 64, IcerPixelFormat::Yuv444P); // 3 planes
let bytes = encode_icer(&img, &EncodeOptions::compressed())?; // filter Q
let decoded = parse_icer(&bytes)?;
assert_eq!(decoded.pixel_format, IcerPixelFormat::Yuv444P);
assert_eq!(decoded.planes.len(), 3);
```

**Backward compatibility.** The container is marked by a leading `0x0000`
16-bit sentinel. A single-plane (Gray8) stream can never begin with
`0x0000` -- the first two bytes of any valid segment are its non-zero
synchronisation prefix (`SegmentHeader::parse` rejects a zero prefix as
corruption). So the decoder dispatches on the first two bytes with zero
ambiguity, and **every previously-encoded Gray8 stream is byte-for-byte
unchanged** and decodes exactly as before -- the colour container is only
emitted for multi-plane images.

The colour path threads through every decode entry point
(`parse_icer`, `parse_icer_with_limits`, `parse_icer_metadata`,
`parse_icer_lenient`) and the registry `Encoder` (a 3-plane `Frame::Video`
selects the colour path; a 1-plane frame stays Gray8). Filter-Q colour
round-trips are bit-exact across all three planes; the uncompressed §III.D
colour path is bit-exact too. The `DecodeLimits` DoS caps apply per plane
*and* to the colour-image total. Chroma subsampling (4:2:2 / 4:2:0) and the
RGB↔YCbCr colour-transform stage are not implemented (the deployed pipeline
applies the colour transform before ICER -- this crate sees three
already-decorrelated 4:4:4 planes).

## Deep-sample grayscale (IPN 42-155 §II.C)

The paper's own operating point is deep imagery: "On MER, all cameras
produce 12-bit pixels and each is stored using a 16-bit word" (§II.C),
and every §VII benchmark image is 12-bit. `IcerPixelFormat::GrayDeep {
bits }` (9..=16) carries that path end to end:

```rust
use oxideav_icer::{encode_icer, parse_icer, EncodeOptions, IcerImage, IcerPixelFormat};

let mut img = IcerImage::zeros(1024, 1024, IcerPixelFormat::GrayDeep { bits: 12 });
img.set_sample(0, 3, 7, 4095);                       // LSB-aligned LE u16 words
let bytes = encode_icer(&img, &EncodeOptions::compressed())?; // lossless, any filter
let decoded = parse_icer(&bytes)?;
assert_eq!(decoded.sample(0, 3, 7), 4095);
```

Wire form: the 12-byte segment header has no free field left for a
sample depth, and a bare deep stream would be indistinguishable from an
8-bit one — so a deep image rides the plane-container framing with
format tag `2` plus one bit-depth byte, around a normal single-plane
segment stream whose coefficients simply span the deeper range (the
§III bit-plane coder, context model, and packet machinery are
depth-agnostic; §II.C Table 4 guarantees 16-bit input can never
overflow the crate's `i32` coefficient words). **Every existing wire
form is unchanged**: Gray8 streams stay bare byte-for-byte, colour
containers keep tag 1.

The §III.A level shift generalises to `2^(n-1)`, the decode clamp to
`[0, 2^n - 1]`, placeholder / missing-strip fills to the n-bit
midpoint, and the §III.D raw path ships little-endian sample pairs
(halving the raw-strip pixel ceiling to 32767 samples). Deep composes
with all seven §II.A filters (lossless end to end, pinned at every
depth 9..=16), row-strip and §V.B transform-domain segmentation, both
entropy backends, §III.A priority interleaving, §VI.A min-loss, §VI.B
byte quotas (the container's 9 framing bytes are charged against the
budget so the hard cap bounds the total output), ROI priorities, the
§III.D fallback, lenient decode, `DecodeLimits`, and quality-target
rate-control. PSNR / SSIM peaks follow the §VII definition (`2^b - 1`
for `b`-bit input); `ImageStats` scans deep samples down-shifted to
the 8-bit domain so the filter-selection thresholds keep their
calibration. The `registry` conversion maps 10-/12-/16-bit onto the
matching `oxideav-core` plain-gray deep formats (other depths ride the
16-bit word LSB-aligned), and the registry encoder selects the deep
path when `CodecParameters::pixel_format` declares `Gray10Le` /
`Gray12Le` / `Gray16Le` — without the declaration a 16-bit plane would
be silently misread as twice-as-wide 8-bit samples.

## ICER-3D (IPN 42-164)

Hyperspectral cubes -- `width x height x bands` stacks of spectral-band
images, 1..=16-bit samples -- compress through the ICER-3D pipeline of
IPN Progress Report 42-164, implemented across four modules:

* **`wavelet3d`** -- the §III.A decomposition. Not a plain 3-D Mallat
  pyramid: after the first stage, spatially-low-pass / spectrally
  high-pass subbands keep decomposing *spatially*, so a level-`k`
  spatial subband carries exactly `k` spectral levels. Realised as, per
  stage, one 2-D spatial stage on every spectral plane's low-pass
  lattice followed by one spectral stage over the low-pass block; the
  inverse replays the exact reverse order (the §III.A footnote makes
  the operation order normative under integer rounding). All seven
  IPN 42-155 §II.A reversible integer filters are supported and proven
  bit-exact reversible, including degenerate geometries (per-dimension
  stage gating is a pure function of the geometry).
* **`subband3d`** -- §IV.A bit-plane priorities `p = 2b + L - H + 3`
  (from the per-subband high/low filtering-operation counts) and the
  Appendix subband index assignment; the schedule sorts by decreasing
  priority with the decreasing-index tie-break. Paper pins verified:
  31 subbands at three stages, subband 21 has `H = 2, L = 6` and
  `p = 2b + 7`, only subband 0 owns a priority-0 bit plane and no
  priority-1 plane exists, per-subband priority parity.
* **`context3d`** -- the §IV.C spectral context modeler: 19 contexts
  (Table 2) computed from the categories / signs of the two
  spectral-neighbour coefficients only (Tables 3/4/5), Table 6 sign
  prediction with XOR agreement-bit coding, category-3 bits uncoded.
  The probability estimator is the shared 42-155 §III.C MER procedure.
* **`bitplane3d` + `cube`** -- the §IV coding engine (one spatial plane
  at a time, raster order, sign coded immediately after a coefficient's
  first `1` bit) and the public pipeline: level shift, 3-D DWT, §III.A
  per-spatial-plane **mean subtraction** over the spatially-low-pass
  lattice (one mean per band per segment on the wire -- the negligible
  overhead the paper promises), then priority-granular packets.

```rust
use oxideav_icer::{encode_icer3d, parse_icer3d, CubeEncodeOptions, IcerCube};

let cube = IcerCube::zeros(64, 64, 224, 12);      // AVIRIS-shaped
let opts = CubeEncodeOptions::default()           // filter Q, 3 levels
    .with_transform_domain_segments()             // §II.B spec form (§V.D)
    .with_segment_count(4)                        // the paper's operating point
    .with_byte_quota(200_000)                     // §IV.B byte quota
    .with_min_loss(0);                            // 0 = lossless if quota allows
let bytes = encode_icer3d(&cube, &opts)?;
let decoded = parse_icer3d(&bytes)?;
```

Rate control is §IV.B verbatim: compression stops when the *minimum
loss* parameter's priority boundary or the *byte quota* is reached,
whichever comes first. Packets are cut per priority value, so the
min-loss stop lands exactly on its defining boundary. A byte quota
truncates the **global** priority-interleaved packet order -- every
segment's packets of one §IV.A priority value before any packet of the
next lower priority (the IPN 42-155 §VI.B Fig. 23 cross-segment
arrangement carried to the cube path) -- so mid-range quotas degrade
all segments together instead of deep-refining segment 0 while later
segments starve; a single cut of that order still leaves each segment
a prefix of its own packet sequence, geometry is always preserved, and
an ample quota is byte-identical to the unbudgeted encode
(packet-for-packet pinned against a simulated global-order prefix).
Truncated subbands reconstruct at the deadzone mid-bin point inherited
from the 2-D path.

### Transform-domain error-containment segments (§II.B + IPN 42-155 §V.D)

IPN 42-164 §II.B defines the spec form of ICER-3D segmentation: the
segments are "defined spatially (in the wavelet transform domain)",
partitioned "in much the same way as in ICER, except that in ICER-3D
the segments extend through all spectral bands", "using the same
rectangle partitioning algorithm ... described in [IPN 42-155,
Section V.D]". `with_transform_domain_segments()` implements it end to
end:

* **one** whole-cube 3-D transform (no per-strip transform seams);
* the §V.D partition of the deepest spatially-low-pass lattice, mapped
  to every coefficient by `(x >> ts, y >> ts)` and extended through
  all λ -- the 3-D analogue of the 2-D §V.B "same spatial location,
  same segment" rule. Both sides compute it from the header fields
  (width, height, levels, segment count); boundaries never ride the
  wire. §V.D eq (9) (`s <=` low-pass pixel count) is enforced on both
  sides;
* per-segment §III.A means: "mean values are computed for and
  subtracted from each spatial plane **of each error-containment
  segment** of each spatially low-pass subband", after all
  decomposition stages (§III.A pins the order -- one shared transform
  makes it automatic);
* each segment coded with its own context modeler + entropy coder; the
  §IV.C spectral context model composes cleanly because a segment's
  spectral neighbours are always in the same segment.

Wire form: previously-reserved cube-flags bit 1, strip-height field
pinned to 0 (non-canonical values refused); every pre-existing
row-strip cube stream parses and decodes unchanged, and the row-strip
mode remains the default. A single-segment encode differs from the
row-strip wire only in the flag bits (byte-count parity pinned).

**Error containment, measured** (64×64×6 12-bit band-drifted cube,
2 levels, 16 §V.D segments; drop one segment's packets): residual
≤ 4 grey levels one `2^ts` lattice step beyond the segment's window,
**bit-identical beyond a `3·2^ts` dilation**; across every segment of
a 6-segment cube the 2-D-style profile holds (≤ 4 beyond `3·2^ts`,
bit-exact beyond `8·2^ts`). The lost window itself reconstructs from
its §III.A means -- which ride the segment's fixed wire part, not its
packets -- as a smooth mean-anchored patch at **2x+ better MAE than a
flat mid-range fill** (measured 89 vs 829 on the drifted fixture), so
a lost deep-space packet burst costs detail, not the scene's spectral
signature. Corruption / truncation sweeps and the fuzz corpus cover
the new wire mode.

Cost: on the 32×32×16 headline cube the §V.D mode is byte-identical
at one segment and spends ~3.6% more lossless bytes than row strips at
the paper's 4-segment operating point (13299 vs 12833 B) -- the price
of the finer per-segment packetisation -- while removing the row-strip
transform seams and keeping loss soft-bounded per the profile above.

### Loss-tolerant decode (§I)

"Because compression is progressive within each segment, when data
loss does occur, any received data for the affected segment that
precedes the lost portion will allow a lower fidelity reconstruction
of that segment" (IPN 42-164 §I). `parse_icer3d_lenient` (and its
`_with_limits` variant) realises that promise for a truncated or
partially corrupt downlink where the strict `parse_icer3d` refuses the
stream: every complete packet is salvaged in wire order — a cut
mid-packet keeps the segment's delivered packet prefix (§III.A
deadzone reconstruction), a cut before the packet stream still keeps
the segment's §III.A means (a mean-anchored smooth patch preserving
the band's spectral signature), a fully missing segment reconstructs
as zero coefficients (mid-range), and a corrupt packet body degrades
only its own segment. The returned [`LenientCubeDecode`] reports
per-segment recovered packet counts, the intact-segment count, and a
truncation flag. Every truncation point of a valid stream is pinned
decodable (both segmentation modes), with quality monotone at segment
boundaries and the full stream exact.

**Measured** (32x32x16 correlated-band 8-bit scene, lossless filter Q,
3 levels): the cube stream is **4.12 bits/sample** vs **7.00
bits/sample** for lossless 2-D ICER applied to each band independently
-- a 41% byte reduction, the direction IPN 42-164 §V.A Table 7 reports
for AVIRIS (5.35 vs 7.19). Both entropy backends (arithmetic + the §IV
interleaved coder) drive the cube path; a stream records its backend in
the header flags.

The wire framing (0x0000 + 0xC3 magic, never ambiguous against a 2-D
single-plane stream or the colour container) is implementation-defined
-- both papers leave the byte-level container open. Decode applies the
same `DecodeLimits` caps as the 2-D path (per strip and per cube)
before any allocation, survives every single-byte corruption and prefix
truncation of a valid stream (test-pinned), and the `decode_segment`
fuzz target drives `parse_icer3d` on every input alongside the 2-D
layers.

Two 42-164 ambiguities are resolved by documented implementation
choices: the within-stage operation order (spatial rows/columns then
spectral; the paper only requires forward/inverse symmetry), and an
Appendix index tie that can arise for decompositions deeper than the
paper's 3-stage illustration (broken by a documented rule (5) --
spectral depth, then spectral lowness).

## Benchmarks

`benches/encode_decode.rs` is a criterion suite covering the encode +
decode hot paths so future entropy-coder, wavelet, or filter-selection
work has a stable baseline to compare against. Three input shapes
(diagonal-ramp 16x16, mid-grey 16x16, diagonal-ramp 64x64) are
exercised on the reversible 5/3 wavelet path
(`WaveletFilter::FilterQ`, `wavelet_levels = 2`,
`bit_plane_count = 8`) plus a fourth group covering the IPN 42-155
§III.D uncompressed path on the 64x64 ramp as a near-`memcpy`
contrast against the compressed pipeline. Each subgroup records
`Throughput::Bytes(width * height)` so regressions surface as MiB/s
or GiB/s on the criterion report.

Run with:

```sh
cargo bench --bench encode_decode -- --quick   # smoke (sub-second)
cargo bench --bench encode_decode              # full criterion run
```

Baseline numbers on aarch64-darwin (M-series, default `--bench`
opt-level, `--quick` smoke):

| Group                                  | Time   | Throughput |
|----------------------------------------|--------|------------|
| `encode_compressed_filter_q/ramp_64x64`| ~322 µs| ~12.1 MiB/s|
| `decode_compressed_filter_q/ramp_64x64`| ~364 µs| ~10.7 MiB/s|
| `encode_compressed_filter_a/ramp_64x64`| ~345 µs| ~11.3 MiB/s|
| `decode_compressed_filter_a/ramp_64x64`| ~372 µs| ~10.5 MiB/s|
| `uncompressed_path_64x64/encode`       | ~198 ns| ~19.3 GiB/s|
| `uncompressed_path_64x64/decode`       | ~327 ns| ~11.7 GiB/s|
| `cube3d_filter_q_32x32x16/encode`      | ~2.4 ms| ~12.8 MiB/s|
| `cube3d_filter_q_32x32x16/decode`      | ~2.3 ms| ~13.4 MiB/s|
| `cube3d_td_filter_q_32x32x16_s4/encode`| ~2.6 ms| ~12.0 MiB/s|
| `cube3d_td_filter_q_32x32x16_s4/decode`| ~2.5 ms| ~12.3 MiB/s|
| `deep12_filter_q_64x64/encode`         | ~1.0 ms| ~7.6 MiB/s |
| `deep12_filter_q_64x64/decode`         | ~1.0 ms| ~7.9 MiB/s |

(2-D numbers refreshed r411 after the pipeline migration onto the
§II.A integer transform.)

The filter-A groups run the same §II.A integer recurrence as filter Q
with the Table 1 filter-A parameters (`beta = 0` skips the `d[n+1]`
predictor term); the Q-vs-A delta on the 64×64 ramp is ~7% on encode
and ~2% on decode — dominated by the entropy stage coding the two
filters' different coefficient statistics, not the transform
arithmetic. The two-order-of-magnitude gap between the compressed
and uncompressed paths sets the dynamic range available to future
entropy-coder and wavelet vectorisation work. CI's
`cargo build --all-targets` step exercises the bench file as a
compilation gate; the benchmark itself is opt-in via `cargo bench`.

The suite additionally sweeps three encoder knobs on the 64×64 ramp so
each one's cost is readable in isolation rather than averaged into a
single default:

* **`wavelet_levels` over `[1, 2, 3, 4]`** (both filter Q and filter
  A): encode rises with depth (each level adds a forward pass over the
  shrinking LL lattice + bit-plane scan); decode flattens past depth 3
  as the entropy stage dominates and the LL subband shrinks to 16
  coefficients. Since r411 both filters run the same §II.A integer
  recurrence, so the per-depth Q-vs-A slopes track each other; the
  sweep is retained to catch a divergence.
* **`segment_count` over `[1, 2, 4, 8]`** (filter Q): each segment
  carries its own header + arithmetic-coder context + stripe scan, so
  fixed per-segment cost accumulates as strip payloads shrink. Encode
  is ~flat through 4 segments then steps up at 8; decode rises
  monotonically with the per-segment framing-parse cost.
* **ICER-3D cube baseline** (`cube3d_filter_q_32x32x16`): lossless
  filter-Q encode + decode of a 32x32x16 correlated-band 12-bit cube
  (the integration suite's headline-comparison shape), throughput over
  the 32 KiB of u16 samples — a stable reference for the 3-D DWT +
  spectral-context bit-plane coder before any vectorisation work. The
  companion `cube3d_td_filter_q_32x32x16_s4` group runs the same cube
  through the §V.D transform-domain mode at the paper's 4-segment
  operating point (lossless decode asserted in the bench setup); the
  ~7% encode delta vs the baseline is the per-segment model +
  packetisation cost.
* **`bit_plane_count` over `[4, 8, 12, 16]`** (filter Q): the field is
  a floor on the per-segment packet count, so raising it past the
  natural `needed` (~7-8 on this ramp) emits extra near-empty bit-plane
  pairs, isolating the per-packet fixed cost (arith-coder init / flush
  / framing). Both directions roughly double between `q_4` and `q_16`.
* **Deep-sample baseline** (`deep12_filter_q_64x64`): lossless
  filter-Q encode + decode of a full-range 12-bit textured 64×64
  (lossless decode asserted in setup), throughput per raw sample byte
  (2 per pixel). The 12-bit image carries ~4 more magnitude bit planes
  than the 8-bit ramp, so the per-pixel cost is ~3× the 8-bit group —
  the §III bit-plane coder's cost scales with the delivered planes,
  not the sample width.

## Licence

MIT. See [LICENSE](LICENSE). Copyright (c) 2026 Karpeles Lab Inc.
