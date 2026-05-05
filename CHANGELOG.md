# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

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
