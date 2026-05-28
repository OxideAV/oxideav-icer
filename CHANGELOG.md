# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

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
  Report 42-155 (2003); no JPL flight code or third-party
  re-implementation consulted.
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
