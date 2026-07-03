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
//! Segments are horizontal row strips extending through all spectral
//! bands (IPN 42-164 §II.B: segments correspond to rectangular spatial
//! regions and "extend through all spectral bands"), each transformed
//! and coded independently so data loss is contained — the same strip
//! convention the crate's 2-D multi-segment path uses.
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
//! | 2         | strip height (BE)                                 |
//! | 1         | flags: bit 0 = interleaved entropy backend        |
//! | per seg   | u8 segment index; u8 q (bit-plane count);         |
//! |           | bands x i32 BE plane means; u16 packet count;     |
//! |           | packets of [u8 priority, u32 BE length, body]     |
//! ```

use crate::bitplane3d::{decode_cube_bitplanes, encode_cube_bitplanes, CubeGeometry, CubePacket};
use crate::decoder::DecodeLimits;
use crate::entropy::EntropyKind;
use crate::error::{IcerError, Result};
use crate::header::WaveletFilter;
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
    /// dynamic-range analysis to all seven). The float legacy filter G
    /// is not part of the integer set and is rejected.
    pub filter: WaveletFilter,
    /// Requested decomposition stages, clamped to 1..=6 (the 42-164
    /// examples use three).
    pub wavelet_levels: u8,
    /// Number of row-strip error-containment segments (>= 1).
    pub segment_count: u8,
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
            filter: WaveletFilter::Reversible53,
            wavelet_levels: 3,
            segment_count: 1,
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

    /// Split the cube into `n` row-strip error-containment segments.
    pub fn with_segment_count(mut self, n: u8) -> Self {
        self.segment_count = n;
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

/// Compute and subtract the §III.A per-spatial-plane means over the
/// spatially low-pass lattice (stride `2^ts`) of every band; returns
/// one mean per band.
fn subtract_plane_means(coeffs: &mut [i32], geom: &CubeGeometry) -> Vec<i32> {
    let stride = 1usize << geom.ts;
    let plane = geom.width * geom.height;
    let mut means = Vec::with_capacity(geom.bands);
    for lambda in 0..geom.bands {
        let base = lambda * plane;
        let (mut sum, mut count) = (0i64, 0i64);
        let mut y = 0usize;
        while y < geom.height {
            let mut x = 0usize;
            while x < geom.width {
                sum += coeffs[base + y * geom.width + x] as i64;
                count += 1;
                x += stride;
            }
            y += stride;
        }
        let mean = rounded_mean(sum, count);
        let mut y = 0usize;
        while y < geom.height {
            let mut x = 0usize;
            while x < geom.width {
                coeffs[base + y * geom.width + x] -= mean;
                x += stride;
            }
            y += stride;
        }
        means.push(mean);
    }
    means
}

/// Add the §III.A means back (the decompression counterpart).
/// Saturating: a corrupt stream can carry arbitrary means against
/// arbitrary decoded coefficients, and the decoder clamps to the sample
/// range afterwards anyway.
fn add_plane_means(coeffs: &mut [i32], geom: &CubeGeometry, means: &[i32]) {
    let stride = 1usize << geom.ts;
    let plane = geom.width * geom.height;
    for (lambda, &mean) in means.iter().enumerate() {
        let base = lambda * plane;
        let mut y = 0usize;
        while y < geom.height {
            let mut x = 0usize;
            while x < geom.width {
                let c = &mut coeffs[base + y * geom.width + x];
                *c = c.saturating_add(mean);
                x += stride;
            }
            y += stride;
        }
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

/// Encode a hyperspectral cube into the ICER-3D wire form.
pub fn encode_icer3d(cube: &IcerCube, opts: &CubeEncodeOptions) -> Result<Vec<u8>> {
    cube.validate()?;
    if opts.filter == WaveletFilter::FilterG {
        return Err(IcerError::unsupported(
            "cube pipeline requires one of the seven reversible integer filters (A-F, Q)",
        ));
    }
    if opts.segment_count == 0 {
        return Err(IcerError::unsupported("segment_count must be >= 1"));
    }
    let levels = opts.wavelet_levels.clamp(1, 6);
    let (w, h, bands) = (
        cube.width as usize,
        cube.height as usize,
        cube.bands as usize,
    );
    let strip_h = h.div_ceil(opts.segment_count as usize);
    let seg_count = h.div_ceil(strip_h);
    let kind = opts.entropy_kind();

    // Encode every segment's transform + means + packet stream first.
    let mut segments = Vec::with_capacity(seg_count);
    for seg in 0..seg_count {
        let y0 = seg * strip_h;
        let sh = strip_h.min(h - y0);
        let mut coeffs = extract_strip(cube, y0, sh);
        forward_3d(&mut coeffs, w, sh, bands, levels, opts.filter);
        let geom = CubeGeometry::new(w, sh, bands, levels);
        let means = subtract_plane_means(&mut coeffs, &geom);
        let q = CubeGeometry::bit_planes_needed(&coeffs);
        let packets = encode_cube_bitplanes(&geom, &coeffs, q, opts.min_loss, kind);
        segments.push(EncodedSegment { q, means, packets });
    }

    // §IV.B byte quota: the framing floor (header + every segment's
    // fixed part) must fit; packets are then kept in emission order
    // until the quota is exhausted.
    let floor = CUBE_MAGIC.len() + HEADER_BODY_BYTES + seg_count * segment_fixed_bytes(bands);
    if let Some(quota) = opts.byte_quota {
        if (floor as u64) > quota {
            return Err(IcerError::unsupported(format!(
                "byte quota {quota} below the framing floor {floor}"
            )));
        }
        let mut used = floor as u64;
        for seg in segments.iter_mut() {
            // Keep the longest packet prefix that fits — prefix
            // semantics per segment, so the decoder's schedule walk
            // stays valid; later (independent) segments may still fit
            // packets in whatever budget remains.
            let mut keep = 0usize;
            for pkt in &seg.packets {
                let cost = (PACKET_OVERHEAD + pkt.body.len()) as u64;
                if used + cost > quota {
                    break;
                }
                used += cost;
                keep += 1;
            }
            seg.packets.truncate(keep);
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
    out.extend_from_slice(&(strip_h as u16).to_be_bytes());
    out.push(u8::from(opts.interleaved_entropy));
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
    if filter == WaveletFilter::FilterG {
        return Err(IcerError::invalid("cube stream with non-integer filter id"));
    }
    if !(1..=6).contains(&levels) {
        return Err(IcerError::invalid(format!(
            "cube decomposition levels {levels} outside 1..=6"
        )));
    }
    if flags & !0b1 != 0 {
        return Err(IcerError::invalid("reserved cube flag bits set"));
    }
    if seg_count == 0 || strip_h == 0 {
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
    let kind = if flags & 1 == 1 {
        EntropyKind::Interleaved
    } else {
        EntropyKind::Arithmetic
    };

    // Resource caps: samples per segment strip and for the whole cube.
    let total = (width * height * bands) as u64;
    let per_seg = (width * strip_h * bands) as u64;
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
    for seg in 0..seg_count {
        let y0 = seg * strip_h;
        let sh = strip_h.min(height - y0);
        let seg_idx = r.u8()?;
        if seg_idx as usize != seg {
            return Err(IcerError::invalid(format!(
                "segment index {seg_idx} out of order (expected {seg})"
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
        let packet_count = r.u16()? as usize;
        let mut packets: Vec<(u8, &[u8])> = Vec::with_capacity(packet_count);
        for _ in 0..packet_count {
            let priority = r.u8()?;
            let len = r.u32()? as usize;
            packets.push((priority, r.take(len)?));
        }

        let geom = CubeGeometry::new(width, sh, bands, levels);
        let mut coeffs = decode_cube_bitplanes(&geom, &packets, q, kind)?;
        add_plane_means(&mut coeffs, &geom, &means);
        inverse_3d(&mut coeffs, width, sh, bands, levels, filter);

        // Undo the level shift, clamp to the sample range, and place the
        // strip rows into the output cube.
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
    Ok(cube)
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
            WaveletFilter::Reversible53,
            WaveletFilter::NineSevenA,
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
    fn filter_g_is_rejected() {
        let cube = hyperspectral_fixture(8, 8, 4);
        let opts = CubeEncodeOptions::default().with_filter(WaveletFilter::FilterG);
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
