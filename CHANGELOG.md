# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Fixed

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
