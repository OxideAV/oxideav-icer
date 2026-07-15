//! ICER-3D hyperspectral cube encode / decode — the public pipeline for
//! IPN 42-164.
//!
//! A hyperspectral image is a `width x height x bands` cube of unsigned
//! samples up to 16 bits deep ([`IcerCube`], band-major storage — a
//! stack of spatial planes, matching the "stack of individual images"
//! framing of IPN 42-164 §I). The pipeline per error-containment
//! segment is:
//!
//! 1. level shift (subtract `2^(depth-1)`),
//! 2. the §III.A three-dimensional wavelet decomposition
//!    ([`crate::wavelet3d`]),
//! 3. §III.A mean subtraction: "mean values are computed for and
//!    subtracted from each spatial plane of each error-containment
//!    segment of each spatially low-pass subband ... The mean values
//!    are encoded in the compressed bitstream and added back to the
//!    data at the appropriate decompression step." The spatially
//!    low-pass subbands jointly cover every λ at the spatial-low-pass
//!    lattice, so the wire carries one mean per band per segment
//!    ("only a few bits per spectral band per segment, which is
//!    negligible", §III.A);
//! 4. the §IV bit-plane coder over the 19-context spectral model
//!    ([`crate::bitplane3d`]), emitting one packet per §IV.A priority
//!    value.
//!
//! Rate / quality control follows §IV.B: "ICER-3D provides two
//! parameters ... a *byte quota*, indicating a rough maximum number of
//! compressed bytes to produce, and an integer *minimum loss* parameter
//! `q` that tells the compressor to stop when all subband bit planes
//! having priority value `q` have been encoded. ... ICER-3D stops
//! producing compressed bytes once the quality goal ... or byte quota
//! is met, whichever comes first. Setting the minimum loss parameter to
//! zero will result in compression limited only by the byte quota, and
//! thus compression is lossless when the byte quota is sufficiently
//! large."
//!
//! # Error-containment segmentation
//!
//! The spec form (IPN 42-164 §II.B) defines segments **in the wavelet
//! transform domain**: "The wavelet-transformed data are partitioned in
//! much the same way as in ICER, except that in ICER-3D the segments
//! extend through all spectral bands. Error-containment segments in
//! ICER and ICER-3D are defined using the same rectangle partitioning
//! algorithm; it is described in \[IPN 42-155\] Section V.D."
//! `CubeEncodeOptions::with_transform_domain_segments()` selects it:
//! one whole-cube 3-D transform, the §V.D partition of the deepest
//! spatially-low-pass lattice mapped to every subband (each coefficient
//! `(x, y, λ)` belongs to the segment of low-pass pixel
//! `(x >> ts, y >> ts)`, for every λ), each segment coded with its own
//! context modeler + entropy coder and its own §III.A per-plane means.
//! The decoder recomputes the partition from the header parameters, so
//! segment boundaries never ride the wire.
//!
//! The historical default keeps the crate's row-strip convention:
//! horizontal strips extending through all spectral bands, each
//! independently *transformed* and coded (data loss is still contained,
//! but a strip boundary is a hard transform edge).
//!
//! # Wire format (implementation-defined framing)
//!
//! Both IPN papers leave the byte-level container to the
//! implementation; this crate frames a cube as:
//!
//! ```text
//! | bytes     | field                                             |
//! |-----------|---------------------------------------------------|
//! | 2         | 0x0000 sentinel (never a valid 2-D sync prefix)   |
//! | 1         | 0xC3 cube tag (disambiguates from the colour      |
//! |           | container's IcerPixelFormat tags)                 |
//! | 1         | version, 0x01                                     |
//! | 2 + 2 + 2 | width, height, bands (BE, all >= 1)               |
//! | 1         | bit depth (1..=16)                                |
//! | 1         | wavelet filter id (integer filters A-F, Q)        |
//! | 1         | decomposition levels (1..=6)                      |
//! | 1         | segment count N (>= 1)                            |
//! | 2         | strip height (BE; 0 in the transform-domain mode, |
//! |           | whose partition is recomputed per §V.D)           |
//! | 1         | flags: bit 0 = interleaved entropy backend,       |
//! |           | bit 1 = §V.D transform-domain segmentation        |
//! | per seg   | u8 segment index; u8 q (bit-plane count);         |
//! |           | bands x i32 BE plane means; u16 packet count;     |
//! |           | packets of [u8 priority, u32 BE length, body]     |
//! ```

use crate::bitplane3d::{
    decode_cube_bitplanes, decode_cube_bitplanes_into, encode_cube_bitplanes, CubeGeometry,
    CubePacket,
};
use crate::decoder::DecodeLimits;
use crate::entropy::EntropyKind;
use crate::error::{IcerError, Result};
use crate::header::WaveletFilter;
use crate::partition::{ll_dimensions, partition, SegmentRect};
use crate::subband3d::stage_counts;
use crate::wavelet3d::{forward_3d, inverse_3d};

/// Leading bytes of a cube stream: the 0x0000 sentinel (which no 2-D
/// single-plane stream can start with — segment sync prefixes are
/// non-zero) plus a cube tag distinct from every
/// [`crate::image::IcerPixelFormat`] discriminant the colour container
/// uses, plus a version byte.
pub const CUBE_MAGIC: [u8; 4] = [0x00, 0x00, 0xC3, 0x01];

/// Fixed-size portion of the cube header following [`CUBE_MAGIC`]:
/// width + height + bands (2 each) + depth + filter + levels + segment
/// count (1 each) + strip height (2) + flags (1).
const HEADER_BODY_BYTES: usize = 13;

/// Per-packet framing overhead: 1-byte priority + 4-byte body length.
const PACKET_OVERHEAD: usize = 5;

/// A hyperspectral image cube: `bands` spatial planes of
/// `width x height` unsigned samples, band-major
/// (`samples[λ * width * height + y * width + x]`), each sample
/// `< 2^bit_depth`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct IcerCube {
    /// Spatial width in samples (1..=65535).
    pub width: u32,
    /// Spatial height in samples (1..=65535).
    pub height: u32,
    /// Number of spectral bands (1..=65535).
    pub bands: u32,
    /// Sample bit depth, 1..=16 (AVIRIS-style radiance data is 12-16
    /// bits; IPN 42-164 §III.B discusses word sizes for 12-bit data).
    pub bit_depth: u8,
    /// Band-major samples, length `width * height * bands`.
    pub samples: Vec<u16>,
}

impl IcerCube {
    /// An all-zero cube of the given geometry.
    pub fn zeros(width: u32, height: u32, bands: u32, bit_depth: u8) -> Self {
        Self {
            width,
            height,
            bands,
            bit_depth,
            samples: vec![0; width as usize * height as usize * bands as usize],
        }
    }

    fn validate(&self) -> Result<()> {
        for (name, v) in [
            ("width", self.width),
            ("height", self.height),
            ("bands", self.bands),
        ] {
            if v == 0 || v > u16::MAX as u32 {
                return Err(IcerError::unsupported(format!(
                    "cube {name} {v} outside 1..=65535"
                )));
            }
        }
        if self.bit_depth == 0 || self.bit_depth > 16 {
            return Err(IcerError::unsupported(format!(
                "cube bit depth {} outside 1..=16",
                self.bit_depth
            )));
        }
        let volume = self.width as usize * self.height as usize * self.bands as usize;
        if self.samples.len() != volume {
            return Err(IcerError::invalid(format!(
                "cube sample count {} != {volume}",
                self.samples.len()
            )));
        }
        let ceil = if self.bit_depth == 16 {
            u16::MAX
        } else {
            (1u16 << self.bit_depth) - 1
        };
        if self.samples.iter().any(|&s| s > ceil) {
            return Err(IcerError::invalid(format!(
                "cube sample exceeds {}-bit range",
                self.bit_depth
            )));
        }
        Ok(())
    }
}

/// Encoder options for the ICER-3D cube pipeline.
#[derive(Debug, Clone)]
pub struct CubeEncodeOptions {
    /// Reversible integer wavelet filter (IPN 42-155 §II.A Table 1;
    /// IPN 42-164 uses filter A for its examples and extends the
    /// dynamic-range analysis to all seven).
    pub filter: WaveletFilter,
    /// Requested decomposition stages, clamped to 1..=6 (the 42-164
    /// examples use three).
    pub wavelet_levels: u8,
    /// Number of error-containment segments (>= 1): §V.D rectangles in
    /// the transform-domain mode, row strips otherwise.
    pub segment_count: u8,
    /// §II.B spec-form segmentation: one whole-cube transform, segments
    /// as IPN 42-155 §V.D rectangles of the spatially-low-pass lattice
    /// extended through all spectral bands. Default `false` (row
    /// strips, the historical wire form).
    pub transform_domain: bool,
    /// §IV.B byte quota: rough hard cap on the output size. The framing
    /// floor (header + per-segment fixed parts) must fit.
    pub byte_quota: Option<u64>,
    /// §IV.B minimum loss parameter: stop once all subband bit planes
    /// with priority >= this value are encoded. 0 (default) = encode
    /// everything (lossless when the quota allows).
    pub min_loss: u8,
    /// Use the §IV interleaved entropy coder instead of the arithmetic
    /// backend (recorded in the header flags, as on the 2-D path).
    pub interleaved_entropy: bool,
}

impl Default for CubeEncodeOptions {
    fn default() -> Self {
        Self {
            filter: WaveletFilter::FilterQ,
            wavelet_levels: 3,
            segment_count: 1,
            transform_domain: false,
            byte_quota: None,
            min_loss: 0,
            interleaved_entropy: false,
        }
    }
}

impl CubeEncodeOptions {
    /// Select the wavelet filter.
    pub fn with_filter(mut self, filter: WaveletFilter) -> Self {
        self.filter = filter;
        self
    }

    /// Set the decomposition depth (clamped to 1..=6 at encode time).
    pub fn with_levels(mut self, levels: u8) -> Self {
        self.wavelet_levels = levels;
        self
    }

    /// Split the cube into `n` error-containment segments (row strips
    /// by default; §V.D rectangles under
    /// [`Self::with_transform_domain_segments`]).
    pub fn with_segment_count(mut self, n: u8) -> Self {
        self.segment_count = n;
        self
    }

    /// Use the IPN 42-164 §II.B spec-form segmentation: one whole-cube
    /// 3-D transform, segments as IPN 42-155 §V.D rectangles of the
    /// spatially-low-pass lattice, extended through all spectral bands.
    pub fn with_transform_domain_segments(mut self) -> Self {
        self.transform_domain = true;
        self
    }

    /// Apply a §IV.B byte quota.
    pub fn with_byte_quota(mut self, quota: u64) -> Self {
        self.byte_quota = Some(quota);
        self
    }

    /// Apply the §IV.B minimum loss parameter.
    pub fn with_min_loss(mut self, q: u8) -> Self {
        self.min_loss = q;
        self
    }

    /// Code with the §IV interleaved entropy coder.
    pub fn with_interleaved_entropy(mut self) -> Self {
        self.interleaved_entropy = true;
        self
    }

    fn entropy_kind(&self) -> EntropyKind {
        if self.interleaved_entropy {
            EntropyKind::Interleaved
        } else {
            EntropyKind::Arithmetic
        }
    }
}

/// Rounded-to-nearest integer mean (ties away from the floor), exact
/// over i64 sums.
fn rounded_mean(sum: i64, count: i64) -> i32 {
    debug_assert!(count > 0);
    (2 * sum + count).div_euclid(2 * count) as i32
}

/// The spatially-low-pass lattice positions (stride `2^ts`, origin
/// `(0, 0)`) that fall inside the half-open window `(x0, x1, y0, y1)`
/// of one band's spatial plane. §V.D windows are lattice-aligned
/// (`x0 = rect.x << ts`), but the walk realigns defensively.
fn low_pass_window_positions(
    geom: &CubeGeometry,
    window: (usize, usize, usize, usize),
    mut f: impl FnMut(usize),
) {
    let stride = 1usize << geom.ts;
    let (x0, x1, y0, y1) = window;
    let (x1, y1) = (x1.min(geom.width), y1.min(geom.height));
    let xs = x0.div_ceil(stride) * stride;
    let mut y = y0.div_ceil(stride) * stride;
    while y < y1 {
        let mut x = xs;
        while x < x1 {
            f(y * geom.width + x);
            x += stride;
        }
        y += stride;
    }
}

/// Compute and subtract the §III.A per-spatial-plane means over the
/// spatially low-pass lattice (stride `2^ts`) of every band, restricted
/// to one error-containment segment's spatial window ("mean values are
/// computed for and subtracted from each spatial plane **of each
/// error-containment segment** of each spatially low-pass subband",
/// §III.A); returns one mean per band.
fn subtract_plane_means(
    coeffs: &mut [i32],
    geom: &CubeGeometry,
    window: (usize, usize, usize, usize),
) -> Vec<i32> {
    let plane = geom.width * geom.height;
    let mut means = Vec::with_capacity(geom.bands);
    for lambda in 0..geom.bands {
        let base = lambda * plane;
        let (mut sum, mut count) = (0i64, 0i64);
        low_pass_window_positions(geom, window, |pos| {
            sum += coeffs[base + pos] as i64;
            count += 1;
        });
        let mean = if count == 0 {
            0
        } else {
            rounded_mean(sum, count)
        };
        low_pass_window_positions(geom, window, |pos| {
            coeffs[base + pos] -= mean;
        });
        means.push(mean);
    }
    means
}

/// Add the §III.A means back (the decompression counterpart).
/// Saturating: a corrupt stream can carry arbitrary means against
/// arbitrary decoded coefficients, and the decoder clamps to the sample
/// range afterwards anyway.
fn add_plane_means(
    coeffs: &mut [i32],
    geom: &CubeGeometry,
    means: &[i32],
    window: (usize, usize, usize, usize),
) {
    let plane = geom.width * geom.height;
    for (lambda, &mean) in means.iter().enumerate() {
        let base = lambda * plane;
        low_pass_window_positions(geom, window, |pos| {
            let c = &mut coeffs[base + pos];
            *c = c.saturating_add(mean);
        });
    }
}

/// Extract one row strip (all bands) into a level-shifted i32 buffer.
fn extract_strip(cube: &IcerCube, y0: usize, strip_h: usize) -> Vec<i32> {
    let (w, h) = (cube.width as usize, cube.height as usize);
    let bands = cube.bands as usize;
    let shift = 1i32 << (cube.bit_depth - 1);
    let mut out = Vec::with_capacity(w * strip_h * bands);
    for b in 0..bands {
        let plane = b * w * h;
        for y in y0..y0 + strip_h {
            for x in 0..w {
                out.push(cube.samples[plane + y * w + x] as i32 - shift);
            }
        }
    }
    out
}

/// One segment's encoded pieces, pre-assembly.
struct EncodedSegment {
    q: u8,
    means: Vec<i32>,
    packets: Vec<CubePacket>,
}

/// Fixed wire cost of one segment before any packet: index + q + means
/// + packet count.
fn segment_fixed_bytes(bands: usize) -> usize {
    1 + 1 + 4 * bands + 2
}

/// Compute the §V.D transform-domain partition of a cube's deepest
/// spatially-low-pass lattice: the effective spatial stage count plus
/// the segment rectangles (in LL-lattice coordinates).
fn cube_partition(
    w: usize,
    h: usize,
    bands: usize,
    levels: u8,
    seg_count: usize,
) -> Result<(u8, Vec<SegmentRect>)> {
    let (ts, _) = stage_counts(w, h, bands, levels);
    let (llw, llh) = ll_dimensions(w, h, ts);
    let rects = partition(llw, llh, seg_count)?;
    Ok((ts, rects))
}

/// Encode a hyperspectral cube into the ICER-3D wire form.
pub fn encode_icer3d(cube: &IcerCube, opts: &CubeEncodeOptions) -> Result<Vec<u8>> {
    cube.validate()?;
    if opts.segment_count == 0 {
        return Err(IcerError::unsupported("segment_count must be >= 1"));
    }
    let levels = opts.wavelet_levels.clamp(1, 6);
    let (w, h, bands) = (
        cube.width as usize,
        cube.height as usize,
        cube.bands as usize,
    );
    let kind = opts.entropy_kind();

    // Encode every segment's transform + means + packet stream first.
    let mut segments;
    let seg_count;
    let strip_h_field;
    if opts.transform_domain {
        // §II.B spec form: ONE whole-cube transform, then the IPN
        // 42-155 §V.D rectangle partition of the spatially-low-pass
        // lattice, each rectangle extended through all spectral bands.
        seg_count = opts.segment_count as usize;
        strip_h_field = 0usize;
        let (ts, rects) = cube_partition(w, h, bands, levels, seg_count)?;
        let mut coeffs = extract_strip(cube, 0, h);
        forward_3d(&mut coeffs, w, h, bands, levels, opts.filter);
        segments = Vec::with_capacity(seg_count);
        for rect in &rects {
            let window = rect.image_window(ts, w, h);
            let geom = CubeGeometry::with_window(w, h, bands, levels, window);
            // §III.A: means are subtracted per spatial plane per
            // error-containment segment, AFTER all decomposition
            // stages (a shared transform makes the order automatic).
            let means = subtract_plane_means(&mut coeffs, &geom, window);
            let q = geom.member_bit_planes_needed(&coeffs);
            let packets = encode_cube_bitplanes(&geom, &coeffs, q, opts.min_loss, kind);
            segments.push(EncodedSegment { q, means, packets });
        }
    } else {
        // Historical row-strip form: each strip independently
        // transformed and coded.
        let strip_h = h.div_ceil(opts.segment_count as usize);
        seg_count = h.div_ceil(strip_h);
        strip_h_field = strip_h;
        segments = Vec::with_capacity(seg_count);
        for seg in 0..seg_count {
            let y0 = seg * strip_h;
            let sh = strip_h.min(h - y0);
            let mut coeffs = extract_strip(cube, y0, sh);
            forward_3d(&mut coeffs, w, sh, bands, levels, opts.filter);
            let geom = CubeGeometry::new(w, sh, bands, levels);
            let means = subtract_plane_means(&mut coeffs, &geom, (0, w, 0, sh));
            let q = CubeGeometry::bit_planes_needed(&coeffs);
            let packets = encode_cube_bitplanes(&geom, &coeffs, q, opts.min_loss, kind);
            segments.push(EncodedSegment { q, means, packets });
        }
    }

    // §IV.B byte quota: the framing floor (header + every segment's
    // fixed part) must fit; packets are then kept along the **global
    // progressive order** — every segment's packets of one §IV.A
    // priority value before any packet of the next lower priority,
    // segments in index order within a priority (the IPN 42-155 §VI.B
    // Fig. 23 cross-segment arrangement, carried to the cube path) —
    // and the stream is cut at the first packet that does not fit.
    // Because each segment's own packets are strictly
    // priority-descending, a single cut of the global order leaves
    // every segment holding a prefix of its packet sequence (the
    // decoder's schedule walk stays valid), and no segment starves
    // while an earlier one deep-refines.
    let floor = CUBE_MAGIC.len() + HEADER_BODY_BYTES + seg_count * segment_fixed_bytes(bands);
    if let Some(quota) = opts.byte_quota {
        if (floor as u64) > quota {
            return Err(IcerError::unsupported(format!(
                "byte quota {quota} below the framing floor {floor}"
            )));
        }
        let mut order: Vec<(usize, usize)> = segments
            .iter()
            .enumerate()
            .flat_map(|(si, seg)| (0..seg.packets.len()).map(move |pi| (si, pi)))
            .collect();
        order.sort_by_key(|&(si, pi)| (core::cmp::Reverse(segments[si].packets[pi].priority), si));
        let mut used = floor as u64;
        let mut keep = vec![0usize; segments.len()];
        for &(si, pi) in &order {
            let cost = (PACKET_OVERHEAD + segments[si].packets[pi].body.len()) as u64;
            if used + cost > quota {
                break;
            }
            used += cost;
            debug_assert_eq!(keep[si], pi, "global cut must leave per-segment prefixes");
            keep[si] = pi + 1;
        }
        for (seg, &k) in segments.iter_mut().zip(&keep) {
            seg.packets.truncate(k);
        }
    }

    // Assemble the wire stream.
    let mut out = Vec::with_capacity(floor);
    out.extend_from_slice(&CUBE_MAGIC);
    out.extend_from_slice(&(cube.width as u16).to_be_bytes());
    out.extend_from_slice(&(cube.height as u16).to_be_bytes());
    out.extend_from_slice(&(cube.bands as u16).to_be_bytes());
    out.push(cube.bit_depth);
    out.push(opts.filter as u8);
    out.push(levels);
    out.push(seg_count as u8);
    out.extend_from_slice(&(strip_h_field as u16).to_be_bytes());
    out.push(u8::from(opts.interleaved_entropy) | (u8::from(opts.transform_domain) << 1));
    for (seg_idx, seg) in segments.iter().enumerate() {
        out.push(seg_idx as u8);
        out.push(seg.q);
        for &m in &seg.means {
            out.extend_from_slice(&m.to_be_bytes());
        }
        out.extend_from_slice(&(seg.packets.len() as u16).to_be_bytes());
        for pkt in &seg.packets {
            out.push(pkt.priority);
            out.extend_from_slice(&(pkt.body.len() as u32).to_be_bytes());
            out.extend_from_slice(&pkt.body);
        }
    }
    Ok(out)
}

/// `true` if `bytes` begins with the ICER-3D cube magic.
pub fn is_cube(bytes: &[u8]) -> bool {
    bytes.len() >= CUBE_MAGIC.len() && bytes[..CUBE_MAGIC.len()] == CUBE_MAGIC
}

struct Reader<'a> {
    buf: &'a [u8],
    pos: usize,
}

impl<'a> Reader<'a> {
    fn take(&mut self, n: usize) -> Result<&'a [u8]> {
        if self.pos + n > self.buf.len() {
            return Err(IcerError::Truncated);
        }
        let s = &self.buf[self.pos..self.pos + n];
        self.pos += n;
        Ok(s)
    }
    fn u8(&mut self) -> Result<u8> {
        Ok(self.take(1)?[0])
    }
    fn u16(&mut self) -> Result<u16> {
        let s = self.take(2)?;
        Ok(u16::from_be_bytes([s[0], s[1]]))
    }
    fn u32(&mut self) -> Result<u32> {
        let s = self.take(4)?;
        Ok(u32::from_be_bytes([s[0], s[1], s[2], s[3]]))
    }
    fn i32(&mut self) -> Result<i32> {
        let s = self.take(4)?;
        Ok(i32::from_be_bytes([s[0], s[1], s[2], s[3]]))
    }
}

/// Decode an ICER-3D cube stream with the default [`DecodeLimits`]
/// policy (the cube's `width * height * bands` sample count is measured
/// against the same per-segment / total caps the 2-D decoder applies to
/// pixels).
pub fn parse_icer3d(bytes: &[u8]) -> Result<IcerCube> {
    parse_icer3d_with_limits(bytes, &DecodeLimits::default())
}

/// Decode an ICER-3D cube stream under an explicit resource-cap policy.
pub fn parse_icer3d_with_limits(bytes: &[u8], limits: &DecodeLimits) -> Result<IcerCube> {
    Ok(parse_cube(bytes, limits, false)?.cube)
}

/// Report of a loss-tolerant ICER-3D decode (see
/// [`parse_icer3d_lenient`]).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LenientCubeDecode {
    /// The reconstructed cube. Regions whose data never arrived
    /// reconstruct from whatever did: a segment whose packets were cut
    /// short reconstructs from its delivered packet prefix at the
    /// §III.A deadzone points; a segment whose fixed part (index, q,
    /// means) arrived but whose packets did not still anchors to its
    /// §III.A means; a fully missing segment contributes zero
    /// coefficients (mid-range after the level shift).
    pub cube: IcerCube,
    /// Complete, successfully decoded packets per segment (a prefix of
    /// each segment's emission).
    pub packets_received: Vec<usize>,
    /// Number of leading segments whose fixed wire part (index +
    /// bit-plane count + means) arrived intact.
    pub segments_received: usize,
    /// `true` when the stream ended (or became unreadable) before the
    /// last segment's last packet.
    pub truncated: bool,
}

/// Loss-tolerant decode of an ICER-3D cube stream (default
/// [`DecodeLimits`]).
///
/// IPN 42-164 §I: "because compression is progressive within each
/// segment, when data loss does occur, any received data for the
/// affected segment that precedes the lost portion will allow a lower
/// fidelity reconstruction of that segment." The strict
/// [`parse_icer3d`] refuses a truncated stream outright; this entry
/// point salvages every complete packet that arrived, in wire order,
/// and reports what was recovered. Only the 17-byte fixed header (and
/// the [`DecodeLimits`] policy) can still fail the decode — the
/// geometry, filter, and segment layout are unrecoverable without it.
pub fn parse_icer3d_lenient(bytes: &[u8]) -> Result<LenientCubeDecode> {
    parse_cube(bytes, &DecodeLimits::default(), true)
}

/// [`parse_icer3d_lenient`] under an explicit resource-cap policy.
pub fn parse_icer3d_lenient_with_limits(
    bytes: &[u8],
    limits: &DecodeLimits,
) -> Result<LenientCubeDecode> {
    parse_cube(bytes, limits, true)
}

/// Shared strict / lenient decode core. In strict mode every wire
/// shortfall is an error; in lenient mode segment-level shortfalls
/// degrade to partial reconstruction and only header-level problems
/// error.
fn parse_cube(bytes: &[u8], limits: &DecodeLimits, lenient: bool) -> Result<LenientCubeDecode> {
    let mut r = Reader { buf: bytes, pos: 0 };
    if r.take(CUBE_MAGIC.len())? != CUBE_MAGIC {
        return Err(IcerError::invalid("not an ICER-3D cube stream"));
    }
    let width = r.u16()? as usize;
    let height = r.u16()? as usize;
    let bands = r.u16()? as usize;
    let bit_depth = r.u8()?;
    let filter = WaveletFilter::from_bits(r.u8()?)?;
    let levels = r.u8()?;
    let seg_count = r.u8()? as usize;
    let strip_h = r.u16()? as usize;
    let flags = r.u8()?;

    if width == 0 || height == 0 || bands == 0 {
        return Err(IcerError::invalid("cube dimension is zero"));
    }
    if bit_depth == 0 || bit_depth > 16 {
        return Err(IcerError::invalid(format!(
            "cube bit depth {bit_depth} outside 1..=16"
        )));
    }
    if !(1..=6).contains(&levels) {
        return Err(IcerError::invalid(format!(
            "cube decomposition levels {levels} outside 1..=6"
        )));
    }
    if flags & !0b11 != 0 {
        return Err(IcerError::invalid("reserved cube flag bits set"));
    }
    let transform_domain = flags & 0b10 != 0;
    if seg_count == 0 {
        return Err(IcerError::invalid("cube segment geometry is zero"));
    }
    if transform_domain {
        // The §V.D partition is a pure function of the header fields;
        // the strip-height field is unused and pinned to 0 so streams
        // stay canonical.
        if strip_h != 0 {
            return Err(IcerError::invalid(
                "transform-domain cube stream carries a nonzero strip height",
            ));
        }
    } else {
        if strip_h == 0 {
            return Err(IcerError::invalid("cube segment geometry is zero"));
        }
        // The strips must tile the height: N-1 full strips plus a final
        // strip of 1..=strip_h rows.
        let body = (seg_count - 1) * strip_h;
        if body >= height || height - body > strip_h {
            return Err(IcerError::invalid(format!(
                "strip layout {seg_count} x {strip_h} does not tile height {height}"
            )));
        }
    }
    let kind = if flags & 1 == 1 {
        EntropyKind::Interleaved
    } else {
        EntropyKind::Arithmetic
    };

    // Resource caps: samples per segment and for the whole cube. In the
    // transform-domain mode the cube is one shared transform (a segment
    // spans the full cube volume for allocation purposes — the same
    // policy shape as the 2-D §V.B path, whose segment headers carry
    // the full image dimensions).
    let total = (width * height * bands) as u64;
    let per_seg = if transform_domain {
        total
    } else {
        (width * strip_h * bands) as u64
    };
    if per_seg > limits.max_pixels_per_segment || total > limits.max_total_pixels {
        return Err(IcerError::unsupported(format!(
            "cube geometry {width}x{height}x{bands} exceeds the decode limits"
        )));
    }

    let shift = 1i32 << (bit_depth - 1);
    let ceil = if bit_depth == 16 {
        u16::MAX as i32
    } else {
        (1i32 << bit_depth) - 1
    };
    let mut cube = IcerCube::zeros(width as u32, height as u32, bands as u32, bit_depth);

    /// One segment's wire fields: q, per-band means, packets.
    type SegmentFields<'a> = (u8, Vec<i32>, Vec<(u8, &'a [u8])>);

    /// A lenient segment read: how much of the segment arrived.
    enum SegRead<'a> {
        /// Everything, including all framed packets.
        Full(SegmentFields<'a>),
        /// The fixed part (index + q + means) arrived; the packet
        /// stream was cut — only the complete packets are kept.
        Partial(SegmentFields<'a>),
        /// The fixed part itself did not arrive (or was inconsistent).
        Missing,
    }

    fn read_segment<'a>(
        r: &mut Reader<'a>,
        expect_idx: usize,
        bands: usize,
        lenient: bool,
    ) -> Result<SegRead<'a>> {
        let fixed = (|| {
            let seg_idx = r.u8()?;
            if seg_idx as usize != expect_idx {
                return Err(IcerError::invalid(format!(
                    "segment index {seg_idx} out of order (expected {expect_idx})"
                )));
            }
            let q = r.u8()?;
            if q > 30 {
                return Err(IcerError::invalid(format!("bit-plane count {q} > 30")));
            }
            let mut means = Vec::with_capacity(bands);
            for _ in 0..bands {
                means.push(r.i32()?);
            }
            Ok((q, means))
        })();
        let (q, means) = match fixed {
            Ok(v) => v,
            Err(_) if lenient => return Ok(SegRead::Missing),
            Err(e) => return Err(e),
        };
        let packet_count = match r.u16() {
            Ok(c) => c as usize,
            Err(_) if lenient => return Ok(SegRead::Partial((q, means, Vec::new()))),
            Err(e) => return Err(e),
        };
        let mut packets: Vec<(u8, &[u8])> = Vec::new();
        for _ in 0..packet_count {
            let one = (|| {
                let priority = r.u8()?;
                let len = r.u32()? as usize;
                Ok((priority, r.take(len)?))
            })();
            match one {
                Ok(p) => packets.push(p),
                Err(_) if lenient => return Ok(SegRead::Partial((q, means, packets))),
                Err(e) => return Err(e),
            }
        }
        Ok(SegRead::Full((q, means, packets)))
    }

    let mut packets_received = vec![0usize; seg_count];
    let mut segments_received = 0usize;
    let mut truncated = false;
    let mut stopped = false;
    // Per-segment salvage: `None` = nothing arrived (zero coefficients,
    // zero means). Once the wire falls short, every later segment is
    // missing — the sequential framing cannot resynchronise.
    let mut read_one = |seg: usize| -> Result<Option<SegmentFields<'_>>> {
        if stopped {
            return Ok(None);
        }
        match read_segment(&mut r, seg, bands, lenient)? {
            SegRead::Full(f) => Ok(Some(f)),
            SegRead::Partial(f) => {
                truncated = true;
                stopped = true;
                Ok(Some(f))
            }
            SegRead::Missing => {
                truncated = true;
                stopped = true;
                Ok(None)
            }
        }
    };

    if transform_domain {
        // Recompute the §V.D partition "from the image dimensions, the
        // number of stages of wavelet decomposition, and the total
        // number of segments" (IPN 42-155 §V.D, adopted by 42-164
        // §II.B); a count the partition cannot realise is corrupt data.
        let (ts, rects) = cube_partition(width, height, bands, levels, seg_count)
            .map_err(|e| IcerError::invalid(format!("cube partition: {e}")))?;
        let mut coeffs = vec![0i32; width * height * bands];
        for (seg, rect) in rects.iter().enumerate() {
            let Some((q, means, packets)) = read_one(seg)? else {
                continue;
            };
            segments_received += 1;
            let window = rect.image_window(ts, width, height);
            let geom = CubeGeometry::with_window(width, height, bands, levels, window);
            match decode_cube_bitplanes_into(&geom, &packets, q, kind, &mut coeffs) {
                Ok(()) => packets_received[seg] = packets.len(),
                // A corrupt body: the coefficient buffer is untouched
                // (reconstruction only runs after a clean decode), so
                // the segment degrades to its means alone.
                Err(_) if lenient => packets_received[seg] = 0,
                Err(e) => return Err(e),
            }
            add_plane_means(&mut coeffs, &geom, &means, window);
        }
        inverse_3d(&mut coeffs, width, height, bands, levels, filter);
        for (dst, &c) in cube.samples.iter_mut().zip(&coeffs) {
            *dst = c.saturating_add(shift).clamp(0, ceil) as u16;
        }
    } else {
        for seg in 0..seg_count {
            let y0 = seg * strip_h;
            let sh = strip_h.min(height - y0);
            let geom = CubeGeometry::new(width, sh, bands, levels);
            let mut coeffs = vec![0i32; width * sh * bands];
            if let Some((q, means, packets)) = read_one(seg)? {
                segments_received += 1;
                match decode_cube_bitplanes(&geom, &packets, q, kind) {
                    Ok(c) => {
                        coeffs = c;
                        packets_received[seg] = packets.len();
                    }
                    Err(_) if lenient => {}
                    Err(e) => return Err(e),
                }
                add_plane_means(&mut coeffs, &geom, &means, (0, width, 0, sh));
            }
            inverse_3d(&mut coeffs, width, sh, bands, levels, filter);

            // Undo the level shift, clamp to the sample range, and place
            // the strip rows into the output cube.
            for b in 0..bands {
                let src_plane = b * width * sh;
                let dst_plane = b * width * height;
                for y in 0..sh {
                    for x in 0..width {
                        let v = coeffs[src_plane + y * width + x]
                            .saturating_add(shift)
                            .clamp(0, ceil);
                        cube.samples[dst_plane + (y0 + y) * width + x] = v as u16;
                    }
                }
            }
        }
    }
    Ok(LenientCubeDecode {
        cube,
        packets_received,
        segments_received,
        truncated,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// AVIRIS-flavoured fixture: per-band signal level drift (the
    /// §III.A "systematic variations in signal level of different
    /// spectral bands") over spatial texture, 12-bit.
    fn hyperspectral_fixture(w: u32, h: u32, bands: u32) -> IcerCube {
        let mut cube = IcerCube::zeros(w, h, bands, 12);
        let (wu, hu) = (w as usize, h as usize);
        for b in 0..bands as usize {
            let dc = 800 + ((b * 137) % 1200) as i32;
            for y in 0..hu {
                for x in 0..wu {
                    let t = ((x * 13 + y * 29 + b * 7) % 257) as i32 - 128;
                    let ridge = if (x / 4 + y / 4) % 3 == 0 { 200 } else { 0 };
                    cube.samples[b * wu * hu + y * wu + x] = (dc + t + ridge).clamp(0, 4095) as u16;
                }
            }
        }
        cube
    }

    #[test]
    fn lossless_roundtrip_default_options() {
        // §IV.B: minimum loss 0 and no byte quota -> lossless.
        let cube = hyperspectral_fixture(16, 16, 8);
        let bytes = encode_icer3d(&cube, &CubeEncodeOptions::default()).unwrap();
        assert!(is_cube(&bytes));
        let decoded = parse_icer3d(&bytes).unwrap();
        assert_eq!(decoded, cube);
    }

    #[test]
    fn lossless_roundtrip_all_integer_filters() {
        let cube = hyperspectral_fixture(12, 10, 6);
        for filter in [
            WaveletFilter::FilterQ,
            WaveletFilter::FilterA,
            WaveletFilter::FilterB,
            WaveletFilter::FilterC,
            WaveletFilter::FilterD,
            WaveletFilter::FilterE,
            WaveletFilter::FilterF,
        ] {
            let opts = CubeEncodeOptions::default().with_filter(filter);
            let decoded = parse_icer3d(&encode_icer3d(&cube, &opts).unwrap()).unwrap();
            assert_eq!(decoded, cube, "{filter:?}");
        }
    }

    #[test]
    fn lossless_roundtrip_multi_segment_and_interleaved() {
        let cube = hyperspectral_fixture(16, 22, 5);
        for segs in [1u8, 2, 3, 5] {
            let opts = CubeEncodeOptions::default()
                .with_segment_count(segs)
                .with_interleaved_entropy();
            let decoded = parse_icer3d(&encode_icer3d(&cube, &opts).unwrap()).unwrap();
            assert_eq!(decoded, cube, "{segs} segments");
        }
    }

    #[test]
    fn mean_subtraction_is_wired() {
        // The per-band means must appear in the wire stream: a cube with
        // large per-band DC offsets encodes each band's low-pass around
        // zero, so the means block is non-trivial. Sanity-check by
        // corrupting one mean and observing a decode difference.
        let cube = hyperspectral_fixture(16, 16, 4);
        let bytes = encode_icer3d(&cube, &CubeEncodeOptions::default()).unwrap();
        let mut corrupted = bytes.clone();
        // First mean lives right after magic + header body + seg idx + q.
        let off = CUBE_MAGIC.len() + HEADER_BODY_BYTES + 2;
        corrupted[off] ^= 0x01; // flip a high bit of mean[0]
        let a = parse_icer3d(&bytes).unwrap();
        let b = parse_icer3d(&corrupted).unwrap();
        assert_ne!(a, b, "plane means must participate in reconstruction");
    }

    #[test]
    fn min_loss_reduces_bytes_and_bounds_error() {
        // §IV.B: raising the minimum loss parameter cuts more (lower
        // priority) bit planes: byte counts fall monotonically and
        // reconstruction error grows monotonically; q = 0 is lossless.
        let cube = hyperspectral_fixture(16, 16, 8);
        let mut last_len = usize::MAX;
        let mut last_mse = -1.0f64;
        for q in [0u8, 4, 8, 12, 16] {
            let opts = CubeEncodeOptions::default().with_min_loss(q);
            let bytes = encode_icer3d(&cube, &opts).unwrap();
            let decoded = parse_icer3d(&bytes).unwrap();
            let mse: f64 = decoded
                .samples
                .iter()
                .zip(&cube.samples)
                .map(|(&d, &o)| ((d as f64) - (o as f64)).powi(2))
                .sum::<f64>()
                / cube.samples.len() as f64;
            if q == 0 {
                assert_eq!(decoded, cube, "min loss 0 must be lossless");
            }
            assert!(bytes.len() <= last_len, "bytes rose at q = {q}");
            assert!(mse >= last_mse, "error fell at q = {q}");
            last_len = bytes.len();
            last_mse = mse;
        }
        assert!(last_mse > 0.0, "sweep never became lossy");
    }

    #[test]
    fn byte_quota_is_honoured_and_progressive() {
        let cube = hyperspectral_fixture(16, 16, 8);
        let unbounded = encode_icer3d(&cube, &CubeEncodeOptions::default()).unwrap();
        let mut last_mse = f64::INFINITY;
        for quota in [400u64, 800, 1600, 3200, unbounded.len() as u64 + 16] {
            let opts = CubeEncodeOptions::default().with_byte_quota(quota);
            let bytes = encode_icer3d(&cube, &opts).unwrap();
            assert!(bytes.len() as u64 <= quota, "quota {quota} exceeded");
            let decoded = parse_icer3d(&bytes).unwrap();
            let mse: f64 = decoded
                .samples
                .iter()
                .zip(&cube.samples)
                .map(|(&d, &o)| ((d as f64) - (o as f64)).powi(2))
                .sum::<f64>()
                / cube.samples.len() as f64;
            assert!(
                mse <= last_mse + 1e-9,
                "quality fell as quota rose ({quota}: {mse} > {last_mse})"
            );
            last_mse = mse;
        }
        assert_eq!(last_mse, 0.0, "big-enough quota must be lossless");
    }

    /// Walk a cube wire stream: per-segment `(priority, body_len)`
    /// packet lists (framing only — no entropy decode).
    fn walk_cube_packets(bytes: &[u8]) -> Vec<Vec<(u8, usize)>> {
        let bands = u16::from_be_bytes([bytes[8], bytes[9]]) as usize;
        let segs = bytes[13] as usize;
        let mut pos = CUBE_MAGIC.len() + HEADER_BODY_BYTES;
        let mut out = Vec::with_capacity(segs);
        for _ in 0..segs {
            pos += 2 + 4 * bands; // idx + q + means
            let count = u16::from_be_bytes([bytes[pos], bytes[pos + 1]]) as usize;
            pos += 2;
            let mut pkts = Vec::with_capacity(count);
            for _ in 0..count {
                let prio = bytes[pos];
                let len = u32::from_be_bytes([
                    bytes[pos + 1],
                    bytes[pos + 2],
                    bytes[pos + 3],
                    bytes[pos + 4],
                ]) as usize;
                pkts.push((prio, len));
                pos += PACKET_OVERHEAD + len;
            }
            out.push(pkts);
        }
        assert_eq!(pos, bytes.len(), "wire walk must consume the stream");
        out
    }

    #[test]
    fn quota_interleaves_across_segments() {
        // The §IV.B quota must truncate the GLOBAL progressive order
        // (all segments' packets of a priority before the next lower
        // priority — the §VI.B cross-segment arrangement), not feed
        // early segments first. Pin: (a) the budgeted stream's kept
        // packets are exactly the simulated global-order prefix of the
        // unbudgeted stream, (b) at a quota that one segment's full
        // packet chain alone would exhaust, every segment still
        // receives packets, (c) an ample quota is byte-identical to
        // the unbudgeted encode.
        let cube = hyperspectral_fixture(16, 24, 6);
        for transform_domain in [false, true] {
            let mut base = CubeEncodeOptions::default().with_segment_count(3);
            if transform_domain {
                base = base.with_transform_domain_segments();
            }
            let full = encode_icer3d(&cube, &base).unwrap();
            let full_pkts = walk_cube_packets(&full);
            let floor = CUBE_MAGIC.len()
                + HEADER_BODY_BYTES
                + full_pkts.len() * segment_fixed_bytes(cube.bands as usize);

            // A quota that segment 0's own packets would exhaust.
            let seg0_total: usize = full_pkts[0]
                .iter()
                .map(|&(_, len)| PACKET_OVERHEAD + len)
                .sum();
            let quota = (floor + seg0_total) as u64;
            let cut = encode_icer3d(&cube, &base.clone().with_byte_quota(quota)).unwrap();
            assert!(cut.len() as u64 <= quota);
            let cut_pkts = walk_cube_packets(&cut);

            // (a) simulate the global-order cut on the unbudgeted lists.
            let mut order: Vec<(usize, usize)> = full_pkts
                .iter()
                .enumerate()
                .flat_map(|(si, pkts)| (0..pkts.len()).map(move |pi| (si, pi)))
                .collect();
            order.sort_by_key(|&(si, pi)| (core::cmp::Reverse(full_pkts[si][pi].0), si));
            let mut used = floor as u64;
            let mut expect = vec![0usize; full_pkts.len()];
            for &(si, pi) in &order {
                let cost = (PACKET_OVERHEAD + full_pkts[si][pi].1) as u64;
                if used + cost > quota {
                    break;
                }
                used += cost;
                expect[si] = pi + 1;
            }
            for (si, pkts) in cut_pkts.iter().enumerate() {
                assert_eq!(
                    pkts.len(),
                    expect[si],
                    "td={transform_domain} segment {si} kept-packet count"
                );
                assert_eq!(
                    pkts.as_slice(),
                    &full_pkts[si][..pkts.len()],
                    "td={transform_domain} segment {si} must keep a prefix"
                );
            }

            // (b) no segment starves.
            assert!(
                cut_pkts.iter().all(|p| !p.is_empty()),
                "td={transform_domain}: a segment starved under the quota: {:?}",
                cut_pkts.iter().map(Vec::len).collect::<Vec<_>>()
            );

            // (c) ample quota is byte-identical.
            let ample =
                encode_icer3d(&cube, &base.clone().with_byte_quota(full.len() as u64)).unwrap();
            assert_eq!(ample, full, "td={transform_domain} ample quota");
        }
    }

    #[test]
    fn quota_below_floor_is_refused() {
        let cube = hyperspectral_fixture(8, 8, 4);
        let opts = CubeEncodeOptions::default().with_byte_quota(10);
        assert!(matches!(
            encode_icer3d(&cube, &opts),
            Err(IcerError::Unsupported(_))
        ));
    }

    #[test]
    fn decode_limits_apply_to_cubes() {
        let cube = hyperspectral_fixture(16, 16, 8);
        let bytes = encode_icer3d(&cube, &CubeEncodeOptions::default()).unwrap();
        let tight = DecodeLimits {
            max_pixels_per_segment: 64,
            max_total_pixels: 64,
        };
        assert!(matches!(
            parse_icer3d_with_limits(&bytes, &tight),
            Err(IcerError::Unsupported(_))
        ));
        // Unlimited recovers the decode.
        let ok = parse_icer3d_with_limits(&bytes, &DecodeLimits::unlimited()).unwrap();
        assert_eq!(ok, cube);
    }

    #[test]
    fn truncated_and_corrupt_streams_error_cleanly() {
        let cube = hyperspectral_fixture(12, 12, 4);
        let bytes = encode_icer3d(&cube, &CubeEncodeOptions::default()).unwrap();
        // Every prefix must either decode (it can't — strict framing) or
        // return an error, never panic.
        for n in 0..bytes.len() {
            let _ = parse_icer3d(&bytes[..n]);
        }
        // Non-cube input is refused up front.
        assert!(matches!(
            parse_icer3d(&[0x12, 0x34, 0x00, 0x00]),
            Err(IcerError::InvalidData(_))
        ));
    }

    #[test]
    fn transform_domain_lossless_roundtrip() {
        // §II.B spec-form segmentation: one shared transform + §V.D
        // rectangles. Lossless at min-loss 0 across segment counts and
        // both entropy backends, including the paper's operating point
        // of four error-containment segments (§V.B).
        let cube = hyperspectral_fixture(20, 18, 7);
        for segs in [1u8, 2, 3, 4, 7] {
            for interleaved in [false, true] {
                let mut opts = CubeEncodeOptions::default()
                    .with_transform_domain_segments()
                    .with_segment_count(segs);
                if interleaved {
                    opts = opts.with_interleaved_entropy();
                }
                let bytes = encode_icer3d(&cube, &opts).unwrap();
                let decoded = parse_icer3d(&bytes).unwrap();
                assert_eq!(decoded, cube, "{segs} segments interleaved={interleaved}");
            }
        }
    }

    #[test]
    fn transform_domain_all_filters_lossless() {
        let cube = hyperspectral_fixture(13, 11, 5);
        for filter in [
            WaveletFilter::FilterQ,
            WaveletFilter::FilterA,
            WaveletFilter::FilterB,
            WaveletFilter::FilterC,
            WaveletFilter::FilterD,
            WaveletFilter::FilterE,
            WaveletFilter::FilterF,
        ] {
            let opts = CubeEncodeOptions::default()
                .with_transform_domain_segments()
                .with_segment_count(4)
                .with_filter(filter);
            let decoded = parse_icer3d(&encode_icer3d(&cube, &opts).unwrap()).unwrap();
            assert_eq!(decoded, cube, "{filter:?}");
        }
    }

    #[test]
    fn transform_domain_degenerate_geometries_roundtrip() {
        // Thin strips, deep pyramids, single-band and tiny cubes: the
        // §V.D partition + windowed scan must stay exact everywhere the
        // encoder accepts the segment count.
        for (w, h, bands, levels, segs) in [
            (5u32, 40u32, 4u32, 3u8, 3u8),
            (40, 5, 4, 3, 3),
            (16, 16, 1, 5, 4),
            (7, 7, 7, 6, 2),
            (3, 3, 3, 1, 2),
        ] {
            let cube = hyperspectral_fixture(w, h, bands);
            let opts = CubeEncodeOptions::default()
                .with_transform_domain_segments()
                .with_segment_count(segs)
                .with_levels(levels);
            let decoded = parse_icer3d(&encode_icer3d(&cube, &opts).unwrap()).unwrap();
            assert_eq!(decoded, cube, "{w}x{h}x{bands} L{levels} s{segs}");
        }
    }

    #[test]
    fn transform_domain_wire_flag_and_strip_field() {
        // Bit 1 of the flags byte marks the mode; the strip-height field
        // is pinned to zero. The row-strip wire form is byte-for-byte
        // unaffected by the new mode's existence.
        let cube = hyperspectral_fixture(16, 16, 4);
        let td = encode_icer3d(
            &cube,
            &CubeEncodeOptions::default()
                .with_transform_domain_segments()
                .with_segment_count(4),
        )
        .unwrap();
        let flags_off = CUBE_MAGIC.len() + HEADER_BODY_BYTES - 1;
        let strip_off = flags_off - 2;
        assert_eq!(td[flags_off] & 0b10, 0b10);
        assert_eq!(&td[strip_off..strip_off + 2], &[0, 0]);

        let rows = encode_icer3d(&cube, &CubeEncodeOptions::default()).unwrap();
        assert_eq!(rows[flags_off] & 0b10, 0);

        // A transform-domain stream with a nonzero strip height is
        // refused as non-canonical.
        let mut bad = td.clone();
        bad[strip_off + 1] = 1;
        assert!(matches!(parse_icer3d(&bad), Err(IcerError::InvalidData(_))));
    }

    #[test]
    fn transform_domain_segment_count_beyond_eq9_is_refused() {
        // §V.D eq (9): s <= LL pixel count. A 16x16 cube at 3 levels has
        // a 2x2 spatially-low-pass lattice -> at most 4 segments.
        let cube = hyperspectral_fixture(16, 16, 4);
        let opts = CubeEncodeOptions::default()
            .with_transform_domain_segments()
            .with_segment_count(5);
        assert!(matches!(
            encode_icer3d(&cube, &opts),
            Err(IcerError::Unsupported(_))
        ));
        // 4 is fine.
        let opts = CubeEncodeOptions::default()
            .with_transform_domain_segments()
            .with_segment_count(4);
        assert_eq!(
            parse_icer3d(&encode_icer3d(&cube, &opts).unwrap()).unwrap(),
            cube
        );
    }

    #[test]
    fn transform_domain_quota_and_min_loss_compose() {
        let cube = hyperspectral_fixture(16, 16, 8);
        let base = CubeEncodeOptions::default()
            .with_transform_domain_segments()
            .with_segment_count(4);
        // Quota: honoured, quality monotone, ample quota lossless.
        let unbounded = encode_icer3d(&cube, &base).unwrap();
        let mut last_mse = f64::INFINITY;
        for quota in [500u64, 1000, 2000, 4000, unbounded.len() as u64 + 16] {
            let bytes = encode_icer3d(&cube, &base.clone().with_byte_quota(quota)).unwrap();
            assert!(bytes.len() as u64 <= quota);
            let decoded = parse_icer3d(&bytes).unwrap();
            let mse: f64 = decoded
                .samples
                .iter()
                .zip(&cube.samples)
                .map(|(&d, &o)| ((d as f64) - (o as f64)).powi(2))
                .sum::<f64>()
                / cube.samples.len() as f64;
            assert!(mse <= last_mse + 1e-9, "quality fell at quota {quota}");
            last_mse = mse;
        }
        assert_eq!(last_mse, 0.0);
        // Min loss: bytes fall / error rises monotonically; 0 lossless.
        let mut last_len = usize::MAX;
        let mut last_err = -1.0f64;
        for q in [0u8, 4, 8, 12] {
            let bytes = encode_icer3d(&cube, &base.clone().with_min_loss(q)).unwrap();
            let decoded = parse_icer3d(&bytes).unwrap();
            let mse: f64 = decoded
                .samples
                .iter()
                .zip(&cube.samples)
                .map(|(&d, &o)| ((d as f64) - (o as f64)).powi(2))
                .sum::<f64>()
                / cube.samples.len() as f64;
            if q == 0 {
                assert_eq!(decoded, cube);
            }
            assert!(bytes.len() <= last_len, "bytes rose at q = {q}");
            assert!(mse >= last_err, "error fell at q = {q}");
            last_len = bytes.len();
            last_err = mse;
        }
        assert!(last_err > 0.0);
    }

    #[test]
    fn transform_domain_truncation_and_corruption_never_panic() {
        let cube = hyperspectral_fixture(12, 12, 5);
        let bytes = encode_icer3d(
            &cube,
            &CubeEncodeOptions::default()
                .with_transform_domain_segments()
                .with_segment_count(3),
        )
        .unwrap();
        for n in 0..bytes.len() {
            let _ = parse_icer3d(&bytes[..n]);
        }
        for i in 0..bytes.len() {
            let mut corrupt = bytes.clone();
            corrupt[i] ^= 0x80;
            let _ = parse_icer3d(&corrupt);
        }
    }

    /// Byte offsets of each segment's end within a cube wire stream.
    fn segment_end_offsets(bytes: &[u8]) -> Vec<usize> {
        let bands = u16::from_be_bytes([bytes[8], bytes[9]]) as usize;
        let segs = bytes[13] as usize;
        let mut pos = CUBE_MAGIC.len() + HEADER_BODY_BYTES;
        let mut out = Vec::with_capacity(segs);
        for _ in 0..segs {
            let count_off = pos + 2 + 4 * bands;
            let count = u16::from_be_bytes([bytes[count_off], bytes[count_off + 1]]) as usize;
            pos = count_off + 2;
            for _ in 0..count {
                let len = u32::from_be_bytes([
                    bytes[pos + 1],
                    bytes[pos + 2],
                    bytes[pos + 3],
                    bytes[pos + 4],
                ]) as usize;
                pos += PACKET_OVERHEAD + len;
            }
            out.push(pos);
        }
        assert_eq!(pos, bytes.len());
        out
    }

    fn cube_mse(a: &IcerCube, b: &IcerCube) -> f64 {
        a.samples
            .iter()
            .zip(&b.samples)
            .map(|(&x, &y)| ((x as f64) - (y as f64)).powi(2))
            .sum::<f64>()
            / a.samples.len() as f64
    }

    #[test]
    fn lenient_matches_strict_on_intact_streams() {
        let cube = hyperspectral_fixture(16, 18, 6);
        for opts in [
            CubeEncodeOptions::default().with_segment_count(3),
            CubeEncodeOptions::default()
                .with_transform_domain_segments()
                .with_segment_count(4),
        ] {
            let bytes = encode_icer3d(&cube, &opts).unwrap();
            let strict = parse_icer3d(&bytes).unwrap();
            let lenient = parse_icer3d_lenient(&bytes).unwrap();
            assert_eq!(lenient.cube, strict);
            assert!(!lenient.truncated);
            assert_eq!(lenient.segments_received, lenient.packets_received.len());
            assert!(lenient.packets_received.iter().all(|&n| n > 0));
        }
    }

    #[test]
    fn lenient_salvages_every_truncation_point() {
        // §I: "any received data for the affected segment that precedes
        // the lost portion will allow a lower fidelity reconstruction
        // of that segment." Every prefix past the fixed header must
        // decode leniently (the strict parser refuses them all), and
        // quality must improve monotonically as whole segments arrive.
        let cube = hyperspectral_fixture(16, 16, 6);
        for opts in [
            CubeEncodeOptions::default().with_segment_count(4),
            CubeEncodeOptions::default()
                .with_transform_domain_segments()
                .with_segment_count(4),
        ] {
            let bytes = encode_icer3d(&cube, &opts).unwrap();
            let header_end = CUBE_MAGIC.len() + HEADER_BODY_BYTES;
            for n in header_end..bytes.len() {
                let l = parse_icer3d_lenient(&bytes[..n]).unwrap();
                assert!(l.truncated, "prefix {n} must report truncation");
                assert!(matches!(
                    parse_icer3d(&bytes[..n]),
                    Err(IcerError::Truncated)
                ));
            }
            // Monotone at segment boundaries; exact at the full length.
            let mut last = f64::INFINITY;
            for &end in &segment_end_offsets(&bytes) {
                let l = parse_icer3d_lenient(&bytes[..end]).unwrap();
                let e = cube_mse(&l.cube, &cube);
                assert!(e <= last + 1e-9, "quality fell at boundary {end}");
                last = e;
            }
            assert_eq!(last, 0.0, "all segments received must be exact");
        }
    }

    #[test]
    fn lenient_reports_what_arrived() {
        let cube = hyperspectral_fixture(16, 16, 6);
        let opts = CubeEncodeOptions::default()
            .with_transform_domain_segments()
            .with_segment_count(4);
        let bytes = encode_icer3d(&cube, &opts).unwrap();
        let full = parse_icer3d_lenient(&bytes).unwrap();
        let ends = segment_end_offsets(&bytes);

        // Cut just before segment 2's fixed part completes: segments 0
        // and 1 fully decoded, segment 2 missing, segment 3 missing.
        let cut = ends[1] + 3;
        let l = parse_icer3d_lenient(&bytes[..cut]).unwrap();
        assert!(l.truncated);
        assert_eq!(l.segments_received, 2);
        assert_eq!(l.packets_received[0], full.packets_received[0]);
        assert_eq!(l.packets_received[1], full.packets_received[1]);
        assert_eq!(&l.packets_received[2..], &[0, 0]);

        // Cut mid-way through segment 0's packets: the fixed part (and
        // its §III.A means) arrived, a strict prefix of the packets is
        // salvaged, later segments are missing.
        let cut = (CUBE_MAGIC.len() + HEADER_BODY_BYTES + ends[0]) / 2;
        let l = parse_icer3d_lenient(&bytes[..cut]).unwrap();
        assert!(l.truncated);
        assert_eq!(l.segments_received, 1);
        assert!(l.packets_received[0] > 0);
        assert!(l.packets_received[0] < full.packets_received[0]);
        // The salvaged prefix must beat the nothing-at-all decode.
        let nothing = parse_icer3d_lenient(&bytes[..CUBE_MAGIC.len() + HEADER_BODY_BYTES]).unwrap();
        assert!(cube_mse(&l.cube, &cube) < cube_mse(&nothing.cube, &cube));
    }

    #[test]
    fn lenient_survives_corrupt_bodies() {
        // A corrupted packet body must never fail the lenient decode —
        // the affected segment degrades (at worst to its means), the
        // rest decode normally.
        let cube = hyperspectral_fixture(12, 12, 4);
        let opts = CubeEncodeOptions::default()
            .with_transform_domain_segments()
            .with_segment_count(2);
        let bytes = encode_icer3d(&cube, &opts).unwrap();
        for i in 0..bytes.len() {
            let mut corrupt = bytes.clone();
            corrupt[i] ^= 0xFF;
            let _ = parse_icer3d_lenient(&corrupt);
        }
        for n in 0..bytes.len() {
            let _ = parse_icer3d_lenient(&bytes[..n]);
        }
    }

    #[test]
    fn sixteen_bit_and_one_bit_depths_roundtrip() {
        let mut deep = IcerCube::zeros(9, 7, 5, 16);
        for (i, s) in deep.samples.iter_mut().enumerate() {
            *s = ((i * 40503) % 65536) as u16;
        }
        let decoded =
            parse_icer3d(&encode_icer3d(&deep, &CubeEncodeOptions::default()).unwrap()).unwrap();
        assert_eq!(decoded, deep);

        let mut binary = IcerCube::zeros(8, 8, 4, 1);
        for (i, s) in binary.samples.iter_mut().enumerate() {
            *s = ((i / 3) % 2) as u16;
        }
        let decoded =
            parse_icer3d(&encode_icer3d(&binary, &CubeEncodeOptions::default()).unwrap()).unwrap();
        assert_eq!(decoded, binary);
    }
}
