# oxideav-icer

Pure-Rust ICER -- JPL's progressive wavelet image compressor used by every
Mars surface mission since the 2003 Mars Exploration Rovers (Spirit and
Opportunity), continued on Mars Science Laboratory (Curiosity), Mars 2020
(Perseverance), and follow-on missions.

This is a **clean-room** implementation. The only specification source
consulted was the open Kiely & Klimesh paper "The ICER Progressive Wavelet
Image Compressor", Jet Propulsion Laboratory, *IPN Progress Report 42-155*
(2003) -- abbreviated `IPN 42-155` in source comments.
No JPL flight code, no DSN ground software, no `qccPack`, no third-party
ICER re-implementation was consulted, paraphrased, or cross-checked.

## Round-6 status

| Subsystem                | Status            |
|--------------------------|-------------------|
| Segment header parser    | full (12-byte fixed-width framing, both directions) |
| Packet header parser     | full (4-byte fixed-width framing, both directions) |
| Segment walker           | full (enumerates every packet body in a buffer) |
| Integer 5/3 wavelet      | full (forward + inverse, 1-D, 2-D one-level, dyadic D-level) |
| Float wavelet filters A-G | full (1-D + 2-D + dyadic; round-trip to IEEE-754 tolerance) |
| Subband de-interleave    | full (4-quadrant LL / HL / LH / HH layout) |
| Binary arithmetic coder  | full (16-bit registers, follow-bit carry, both directions) |
| Context model            | full (IPN 42-155 §III.B H/V/D classification; 9 significance + 5 sign + 3 refinement contexts) |
| Bit-plane scanner        | full (stripe-ordered significance + sign + refinement passes, MSB-down) |
| Multi-packet ordering    | full (one significance + one refinement packet per bit-plane per IPN 42-155 §IV) |
| Compressed-segment encode | full (level-shift + DWT + stripe scan + multi-packet arith) |
| Compressed-segment decode | full (multi-packet arith + stripe scan + inverse DWT + clamp) |
| Quota-controlled encoding | full (`with_byte_budget` hard cap + `with_target_bytes` soft target) |
| Multi-segment images     | full (row-strip split on encode, stitch by `segment_index` on decode) |
| Uncompressed-segment decode | full (IPN 42-155 §III.D; placeholder-segment tolerant) |
| Uncompressed-segment encode | full (IPN 42-155 §III.D) |
| Image statistics + filter recommendation | full (`ImageStats::from_image` + `recommend_filter` decision tree) |
| Auto filter selection (heuristic) | full (`EncodeOptions::with_auto_filter()`, single-pass) |
| Auto filter selection (rate-distortion) | full (`EncodeOptions::with_auto_filter_rd()`, N-pass trial) |
| ROI segment prioritisation | full (`with_segment_priorities` + `with_center_roi`; IPN 42-155 §III.E independent-segment scheduling) |
| R-D budget pruning | full (round 91; `EncodeOptions::with_rd_budget(n)` -- per-segment cost-per-byte packet selection per IPN 42-155 §IV.B rate-allocation principle) |
| Decoder / Encoder traits | full (gated on default `registry` feature) |
| Decode-side resource limits | full (round 174; `DecodeLimits` + `parse_icer_with_limits`; default 64 MPx/segment, 256 MPx total; closes round-131 4 GB-per-plane DoS surface) |

End-to-end round-trips:

* Uncompressed Gray8 -- bit-identical.
* Compressed Gray8 with filter `Q` (lossless integer 5/3) -- bit-identical
  via stripe-ordered bit-plane scanner + multi-packet arithmetic coder.
* Compressed Gray8 with float filters A-G -- bounded mean-abs error
  (lossy, as expected from float lifting + integer rounding).
* Multi-segment row-strip split -- bit-identical for the uncompressed path,
  bit-identical for compressed filter `Q`.
* Multi-packet metadata: segment with q=4 bit-planes produces >= 8 packets
  (verified by `compressed_roundtrip_filter_q_multi_packet_metadata`).

## Wavelet filter coverage

IPN 42-155 §III.A enumerates eight candidate wavelet filters: A through G
(float lifting variants) plus Q (integer 5/3). Round 3 implements all eight:

* Filter `Q` (the integer 5/3 reversible lifting filter) -- bit-exact integer
  round-trip.
* Filters `A` through `F` (float CDF-style + Daubechies + Haar lifting
  variants) -- float-precision round-trip to IEEE-754 tolerance, lossy through
  the integer coefficient quantisation step.
* Filter `G` (Le Gall 5/3 float variant, IPN 42-155 §III.A) -- same
  predict/update shape as filter Q (alpha=-0.5, beta=0.25) applied in
  floating point with an orthonormal post-scale (zeta = sqrt(2)/2). Lossy
  like filters A-F. Completes the full A-G filter set.

## Context model (round-3 upgrade)

The round-2 placeholder popcount-based significance context table has been
replaced with the IPN 42-155 §III.B H/V/D neighbour-count classification:

* **9 significance contexts** -- determined by the count of significant
  horizontal neighbours (H: 0, 1, or 2+), vertical neighbours (V: 0 or 1+),
  and diagonal neighbours (D: 0 or 1+).
* **5 sign contexts** -- determined by the horizontal and vertical neighbour
  sign contributions (clipped to {-1, 0, +1} per axis) per §III.B, with
  sign-flip coding convention applied (the coder always codes the sign
  relative to the prediction, not the raw sign).
* **3 refinement contexts** -- unchanged from round 2.

## Stripe-ordered scan (round-3 upgrade)

The bit-plane scanner now uses **stripe-ordered** processing (IPN 42-155
§III.B): the image is partitioned into horizontal stripes of height 4 rows
(`STRIPE_HEIGHT = 4`). Within each bit-plane, the significance pass and
refinement pass each process one complete stripe before advancing to the next.
This maximises context-pattern locality.

## Multi-packet ordering (round-3 upgrade)

Each compressed segment now emits one packet pair per bit-plane per IPN
42-155 §IV:

* Packet 0 of bit-plane `bp`: significance + sign pass body (independently
  arithmetic-coded).
* Packet 1 of bit-plane `bp`: refinement pass body (independently coded).

For a segment with bit-plane count Q, the encoder emits `2*Q` packets in
MSB-first priority order. A decoder receiving a truncated stream can still
reconstruct a lower-quality image from the packets it received.

## What is *not* in round 3

* **Real-world bitstream interop**. The context model now uses the IPN 42-155
  §III.B classification scheme, but the exact pattern-to-context index
  mapping and the exact probability estimator window size are not published
  in the paper. This crate implements a clean-room interpretation; the
  self-roundtrip is correct but may not be bit-equivalent to real Mars-rover
  ICER files.
* **Per-filter lifting coefficients** for `A` through `G`. IPN 42-155 §III.A
  names the filters but defers the numerical coefficients to reference [13].
  The crate currently uses CDF 9/7 + Daubechies + Haar + Le Gall parameters
  drawn from open wavelet literature (Sweldens 1996); Mars-rover interop will
  need the JPL-specific numbers from reference [13] when those become
  available.
* **Colour plane support**. The encoder + decoder are Gray8 only; YCbCr and
  multi-plane ICER are deferred.

## Documentation gaps

The 2003 IPN 42-155 paper deliberately leaves several implementation details
to "the implementation":

* The **literal sync-prefix value** (§IV mentions "self-synchronising prefix"
  but doesn't pin a 16-bit value). This crate accepts any non-zero 16-bit
  prefix and surfaces it via `SegmentHeader::sync_prefix`.
* The **probability estimator window size** for the adaptive arithmetic coder
  (§III.C says "windowed counting"; window size unspecified). This crate uses
  64.
* The **exact neighbourhood-pattern to context-index tables** for the
  significance + sign passes (§III.B Table 1 lists the context counts but not
  the per-pattern lookup). Round 3 ships the H/V/D classification scheme
  described in §III.B; the supplemental tables in the
  `descanso.jpl.nasa.gov` ICER white papers (reference [13]) are not yet
  in `docs/image/icer/`.
* **Per-filter lifting coefficients** for `A` through `G` (see above).

## Automatic filter selection (round-5 upgrade)

ICER's eight wavelet filters (A-G plus Q) have different rate-distortion
profiles depending on image content. IPN 42-155 §I notes that the choice
is image-dependent but does not prescribe a fixed mapping. Round 5 adds
two ways to let the encoder pick the filter:

* **Heuristic** (`EncodeOptions::with_auto_filter()`): a one-pass scan
  computes image statistics (mean, variance, horizontal + vertical
  gradient energy, dynamic range) via `analyze::ImageStats::from_image`,
  then `analyze::recommend_filter` runs a transparent decision tree:
  flat / low-frequency content gets filter `Q` (reversible 5/3);
  high-frequency, high-variance content gets filter `A` (CDF 9/7).
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

## ROI segment prioritisation (round-6 upgrade)

IPN 42-155 §III.E specifies that ICER's image partitioning (the
`segment_count` row-strip split) gives the encoder freedom to
*schedule* segments independently. The Mars rover deployments use this
to save the centre of a Pancam frame -- where the science target sits
-- ahead of the periphery (sky / rover hardware at the edges) when the
downlink budget is tight.

Round 6 surfaces that freedom as two new `EncodeOptions` builders:

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

**Empirical measurement** (128-row image, 4 segments, 220-byte budget,
centre + periphery mean-absolute error vs. original):

| metric          | value |
|-----------------|------:|
| centre band MAE | 63.98 |
| periphery MAE   | 84.00 |

The centre strip's MAE is ~24% lower than the periphery's; under
index-order emission the same budget would distribute the loss
uniformly (or worse, drop the centre while saving the unimportant
edges).

The priority vector is validated at encode-time (length match,
permutation property); a malformed vector returns
`IcerError::Unsupported`.

**Empirical byte counts** (32x32 test inputs, default 3-level DWT, q=8):

| image          | filter Q | filter A | heuristic pick    | RD pick |
|----------------|---------:|---------:|-------------------|--------:|
| flat           |     124  |     124  | 124 (=Q)          |    124  |
| diagonal ramp  |     516  |     538  | 516 (=Q)          |    516  |
| checkerboard   |     283  |     413  | 413 (=A, lossy)   |    283  |

The checkerboard row illustrates the trade-off: the heuristic follows
wavelet-theory intuition (high edge energy -> biorthogonal 9/7) and
picks filter `A` (which is also lossy on this input); the
rate-distortion mode empirically determines that filter `Q` (lossless
integer 5/3) actually produces fewer bytes on this implementation's
arithmetic coder. Use the heuristic when you want zero per-image
overhead; use RD-mode when you want the true minimum.

## Rate-distortion budget pruning (round-91 upgrade)

The round-4 quota-controlled encoder truncates packets in strict
MSB-down emission order: it emits sig(0), ref(0), sig(1), ref(1),
... and stops the moment the next packet would exceed the byte cap.
That semantic is byte-honest but R-D-suboptimal: the truncation cut
often falls on a refinement packet whose distortion-reduction-per-
byte is poor compared to *other* packets later in the chain that
would have fit individually but for the budget being already spent.

Round 91 adds a true rate-distortion (R-D) packet selector
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

**Empirical wins** (single-segment compressed encode, filter Q,
64×64 fixtures, 400-byte budget):

| fixture        | strict-MSB byte_budget | R-D budget | strict PSNR | R-D PSNR |
|----------------|-----------------------:|-----------:|------------:|---------:|
| checkerboard   | 355 B / 19.42 dB       | 396 B      |    19.42 dB | **25.51 dB** |
| sparse impulse | 398 B / 29.93 dB       | 393 B      |    29.93 dB |    29.93 dB |
| 256² gradient  | 300 B / 10.78 dB       | 295 B      |    10.78 dB |    10.78 dB |

The +6.09 dB checkerboard win comes from the selector recognising
that the bit-plane-1 refinement packet (25 body bytes, ΔD ~4.2 M)
is worth more per byte than the bit-plane-3 refinement packet
(42 body bytes, ΔD ~459 k) the strict-MSB cut-off was forced to
spend the trailing bytes on. R-D drops the heavy low-priority
refinement and reallocates the bytes to the higher-priority one.

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

## Roadmap

One piece is deliberately not in the current rounds — it is core
to ICER's value proposition for deep-space imaging:

### ✅ Quota-controlled encoding (landed in round 4)

`encode_icer(image, &opts)` now supports truncation to a caller-
specified byte budget. ICER's signature feature — **emitting
progressive packets MSB-down and stopping once the quota is
exhausted** — is fully implemented. Anything not yet emitted simply
isn't transmitted; the decoder reconstructs at whatever quality the
truncation point allows. This is exactly how the Mars rovers compress
every image.

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

The 2009 follow-on paper Kiely *et al.*, "ICER-3D: A Progressive
Wavelet-Based Compressor for Hyperspectral Images" (IPN Progress
Report 42-178) extends the same coder to a 3-D wavelet transform for
hyperspectral cubes (HiRISE-style data with N spectral bands stacked).
The 2-D 5/3 + float A-G filters become a 3-D separable transform; the
context model gains a band-axis dimension. The encoder + decoder
machinery already in this crate (segment framing, packet ordering,
arithmetic coder, context model) carry over largely unchanged — the
delta is the 3-D DWT + extended context indexing.

This is the natural follow-on once the 2-D path is at JPL-interop
parity (i.e. once the [13] reference tables are transcribed and the
quota-controlled encoder is in place).

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

## Fuzzing (round 131)

A cargo-fuzz harness under `fuzz/` runs every byte slice through
three decode-side entry points and asserts none panic / abort /
OOM:

* `header::walk_segment` -- single-segment framing parse.
* `parse_icer_metadata` -- multi-segment walk (header-only).
* `parse_icer` -- full decode (arith coder + inverse DWT + stitch).

The seed corpus under `fuzz/corpus/decode_segment/` is generated
from icer's own encoder (no third-party ICER reference): an
uncompressed ramp, compressed filter-Q ramps + flat, a two-segment
split, and a byte-budget-truncated input covering the partial-
packet path.

```bash
cd fuzz
cargo +nightly fuzz run decode_segment -- -max_total_time=60
```

Local 60-second run: 11612 iterations, 0 crashes. One slow-unit
documented: a 69-byte input with width=223, height=65312 allocates
~14.5 MB and runs `parse_icer` in ~800 ms (release). This is a
pre-existing DoS surface: the wire format admits any (width,
height) in `u16 * u16`, so a malicious 12-byte segment header
can request up to ~4 GB of decoder-side allocation per plane.
Not a fuzz crash; documented for a future header-validation pass
(application-supplied max-geometry cap).

A daily fuzz run lives at `.github/workflows/fuzz.yml` (30-minute
budget; OxideAV reusable workflow).

## Decode-side resource limits (round 174)

The cargo-fuzz harness from round 131 surfaced a DoS vector inherent
to the wire format: the 12-byte segment header carries `width` and
`height` as `u16` each, which means a 12-byte input can declare up
to `65535 * 65535 ≈ 4.29 GPx` per segment. Pre-round-174,
`parse_icer` would dutifully allocate a ~4 GB plane plus
~16 GB of `i32` coefficient buffers before discovering the body was
empty.

Round 174 adds an application-level geometry cap via
[`DecodeLimits`]:

```rust
let limits = oxideav_icer::DecodeLimits::default();
//         max_pixels_per_segment = 64 MPx
//         max_total_pixels       = 256 MPx
let img = oxideav_icer::parse_icer_with_limits(bytes, &limits)?;

// Trusted-input batch path — preserves pre-round-174 behaviour:
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

## Licence

MIT. See [LICENSE](LICENSE). Copyright (c) 2026 Karpeles Lab Inc.
