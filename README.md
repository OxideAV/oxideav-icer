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

## Round-1 status

| Subsystem                | Status            |
|--------------------------|-------------------|
| Segment header parser    | full (12-byte fixed-width framing, both directions) |
| Packet header parser     | full (4-byte fixed-width framing, both directions) |
| Segment walker           | full (enumerates every packet body in a buffer) |
| Integer 5/3 wavelet      | full (forward + inverse, 1-D, 2-D one-level, dyadic D-level) |
| Subband de-interleave    | full (4-quadrant LL / HL / LH / HH layout) |
| Binary arithmetic coder  | full (16-bit registers, follow-bit carry, both directions) |
| Adaptive context model   | scaffold (17-context Laplace estimator; placeholder neighbourhood-pattern → context table) |
| Bit-plane scanner        | not yet (round 2) |
| Compressed-segment decode | not yet (round 2) |
| Compressed-segment encode | not yet (round 2) |
| Uncompressed-segment decode | full (IPN 42-155 §III.D) |
| Uncompressed-segment encode | full (IPN 42-155 §III.D) |
| Decoder / Encoder traits | full (gated on default `registry` feature) |

End-to-end uncompressed round-trip works: `encode_icer(image)` followed
by `parse_icer(bytes)` returns an image bit-identical to the input on
the Gray8 path.

## Wavelet filter coverage

IPN 42-155 §III.A enumerates seven candidate wavelet filters labelled
`A` through `G`. Round 1 implements only Filter `Q` (the integer 5/3
reversible lifting filter, well-known in the wavelet literature since
Calderbank-Daubechies-Sweldens-Yeo 1998). The deployed Mars rover
configurations use this filter for lossless delivery; the float
9/7-style filters used for lossy delivery (Filter `A` in particular)
parse but do not yet transform — the inverse / forward call refuses
with `IcerError::Unsupported` on those.

## What is *not* in round 1

* Bit-plane scan orchestration. The arithmetic coder + context model
  exist (and round-trip in isolation), but the per-bit-plane
  significance / refinement / cleanup pass scheduling described in
  IPN 42-155 §III.B has not been written. As a result, the decoder's
  compressed-segment path returns a zeros buffer of the correct
  geometry instead of failing.
* Multi-segment images. ICER's image-partitioning scheme (IPN 42-155
  §III.E) lets the encoder split an image into multiple segments, each
  separately decodable. Round 1 only handles `segment_index == 0`.
* Float / non-5/3 wavelet filters (Filters A-G except Q in §III.A).
  Headers carrying them parse, but the transform stage rejects them.
* Lossy operating points. Without the bit-plane coder, only the
  uncompressed (IPN 42-155 §III.D) byte-passthrough path produces
  output — that path is inherently lossless because it copies
  pixels, but it is also the *worst* compression ratio (1:1).

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
  context counts but not the per-pattern lookup). Round 1 ships
  placeholder tables that produce in-range context indices; round 2
  will replace them once the supplemental tables in the
  `descanso.jpl.nasa.gov` ICER white papers (cited as reference [13]
  in IPN 42-155) land in `docs/image/icer/`.

These gaps are not blockers for round-1 milestones (header parse,
wavelet round-trip, arith-coder round-trip, uncompressed-segment
round-trip) but will need clarification for round-2 bit-stream interop
with real Mars-rover-produced ICER files.

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
`encode_icer(image, &EncodeOptions) -> Result<Vec<u8>>`.

## Licence

MIT. See [LICENSE](LICENSE). Copyright (c) 2026 Karpelès Lab Inc.
