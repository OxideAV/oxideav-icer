# oxideav-icer

Pure-Rust ICER — JPL's progressive wavelet image compressor used by every
Mars surface mission since the 2003 Mars Exploration Rovers (Spirit and
Opportunity), continued on Mars Science Laboratory (Curiosity), Mars 2020
(Perseverance), and follow-on missions.

This is a **clean-room** implementation. The only specification source
consulted was the open Kiely & Klimesh paper "The ICER Progressive
Wavelet Image Compressor", Jet Propulsion Laboratory, *IPN Progress
Report 42-155* (2003) — abbreviated `IPN 42-155` in source comments.
No JPL flight code, no DSN ground software, no `qccPack`, no third-party
ICER re-implementation was consulted, paraphrased, or cross-checked.

## Round-2 status

| Subsystem                | Status            |
|--------------------------|-------------------|
| Segment header parser    | full (12-byte fixed-width framing, both directions) |
| Packet header parser     | full (4-byte fixed-width framing, both directions) |
| Segment walker           | full (enumerates every packet body in a buffer) |
| Integer 5/3 wavelet      | full (forward + inverse, 1-D, 2-D one-level, dyadic D-level) |
| Float wavelet filters A-F | full (1-D + 2-D + dyadic; round-trip to IEEE-754 tolerance) |
| Subband de-interleave    | full (4-quadrant LL / HL / LH / HH layout) |
| Binary arithmetic coder  | full (16-bit registers, follow-bit carry, both directions) |
| Adaptive context model   | scaffold (17-context Laplace estimator; placeholder neighbourhood-pattern → context table) |
| Bit-plane scanner        | full (significance + sign + refinement passes, raster scan, MSB-down) |
| Compressed-segment encode | full (level-shift + DWT + bit-plane + arith) |
| Compressed-segment decode | full (arith + bit-plane + inverse DWT + clamp) |
| Multi-segment images     | full (row-strip split on encode, stitch by `segment_index` on decode) |
| Uncompressed-segment decode | full (IPN 42-155 §III.D) |
| Uncompressed-segment encode | full (IPN 42-155 §III.D) |
| Decoder / Encoder traits | full (gated on default `registry` feature) |

End-to-end round-trips:

* Uncompressed Gray8 → bit-identical (round 1).
* Compressed Gray8 with filter `Q` (lossless integer 5/3) → bit-identical
  via the bit-plane scanner self-roundtrip.
* Compressed Gray8 with float filters A-F → bounded mean-abs error
  (lossy, as expected from float lifting + integer rounding).
* Multi-segment row-strip split → bit-identical for the uncompressed
  path, bit-identical for compressed filter `Q`.

## Wavelet filter coverage

IPN 42-155 §III.A enumerates seven candidate wavelet filters labelled
`A` through `G`. Round 2 implements:

* Filter `Q` (the integer 5/3 reversible lifting filter, well-known in
  the wavelet literature since Calderbank-Daubechies-Sweldens-Yeo 1998)
  — bit-exact integer round-trip.
* Filters `A` through `F` (float CDF-style + Daubechies + Haar lifting
  variants) — float-precision round-trip to IEEE-754 tolerance, lossy
  through the integer coefficient quantisation step.

Filter `G` is reserved by IPN 42-155 §III.A but not yet wired through
the dispatch table; the parser still accepts the filter id since the
header layout reserves the slot.

## What is *not* in round 2

* **Stripe-ordered scan**. The bit-plane scanner uses a straight
  raster scan; IPN 42-155 §III.B describes a stripe ordering as an
  optimisation. The scan order does not affect *which* bits are
  coded (so the self-roundtrip is correct) but a future round will
  switch to stripe order to reduce context-pattern cache misses + to
  match the wire format real Mars-rover ICER files use.
* **Real-world bitstream interop**. The placeholder
  context-pattern → context-index tables in `src/context.rs` mean
  the compressed payload is not yet bit-equivalent to a real
  Mars-rover-produced ICER file. Replacing those tables is the
  blocker for cross-validation against external ICER decoders.
* **Multi-packet ordering inside a segment**. Each compressed segment
  currently carries one packet (the entropy-coded body in full). IPN
  42-155 packetisation supports per-bit-plane progressive packets so
  truncated streams reconstruct at lower quality; that work is
  scheduled for the follow-up that replaces the placeholder context
  tables.

## Documentation gaps

The 2003 IPN 42-155 paper is the public source of record but it
deliberately leaves several implementation details to "the
implementation":

* The **literal sync-prefix value** (§IV mentions "self-synchronising
  prefix" but doesn't pin a 16-bit value). Different deployments chose
  different magic words; this crate's parser accepts any non-zero
  16-bit prefix and surfaces it via `SegmentHeader::sync_prefix` for
  the application to validate.
* The **probability estimator window size** for the adaptive
  arithmetic coder (§III.C says "windowed counting"; window size is
  unspecified). This crate uses 64.
* The **exact neighbourhood-pattern → context-index tables** for the
  significance + sign + refinement passes (§III.B Table 1 lists the
  context counts but not the per-pattern lookup). Round 2 ships
  placeholder tables that produce in-range context indices and
  round-trip self-consistently; a follow-up will replace them once
  the supplemental tables in the `descanso.jpl.nasa.gov` ICER white
  papers (cited as reference [13] in IPN 42-155) land in
  `docs/image/icer/`.
* **Per-filter lifting coefficients** for `A` through `F`. IPN 42-155
  §III.A names the seven filters but defers the numerical
  coefficients to reference [13]. The crate currently uses
  CDF 9/7 + Daubechies + Haar lifting parameters drawn from open
  wavelet literature (Sweldens 1996); Mars-rover interop will need
  the JPL-specific numbers from reference [13] when those become
  available.

These gaps do not prevent self-roundtrip correctness (the encoder +
decoder use the same tables + coefficients) but will need
clarification for round-3 bit-stream interop with real Mars-rover-
produced ICER files.

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

## Licence

MIT. See [LICENSE](LICENSE). Copyright (c) 2026 Karpelès Lab Inc.
