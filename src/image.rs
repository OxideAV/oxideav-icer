//! The standalone image types: the shapes every `oxideav-<format>`
//! image crate shares (`IMAGE_CRATE_API`), specialised for ICER.
//!
//! * [`IcerImage`] — the native-layout image [`crate::decode`] returns
//!   and [`crate::encode`] consumes: dimensions, a [`PixelFormat`] tag,
//!   one [`Plane`] per component, [`ColorInfo`], [`Metadata`] and the
//!   ICER extra — the significant [`IcerImage::bit_depth`] of every
//!   sample. ICER has no palette mechanism, so there is no `palette`
//!   field.
//! * [`RgbImage`] / [`RgbaImage`] — the tightly packed 8-bit raw paths
//!   ([`crate::decode_rgb8`] / [`crate::decode_rgba8`],
//!   [`IcerImage::to_rgb8`] / [`IcerImage::to_rgba8`]).
//! * [`Frame`] — one entry of [`crate::decode_all`] (one spectral band
//!   of an ICER-3D cube, or the single image of a 2-D stream).
//! * [`ImageInfo`] — what [`crate::info`] reads from the framing.
//!
//! Defined here (rather than reusing `oxideav_core::VideoFrame`) so the
//! crate can be built with the default `registry` feature off — i.e.
//! without depending on `oxideav-core` at all. When the `registry`
//! feature is on, `crate::registry` provides `From<IcerImage> for
//! oxideav_core::VideoFrame` (and the matching [`IcerPixelFormat`] ↔
//! `oxideav_core::PixelFormat` mapping) so the trait-side `Decoder` /
//! `Encoder` impls are thin adapters over the same functions.
//!
//! # Sample layouts
//!
//! ICER (IPN 42-155 §III) is a single-component coder; the deployed
//! multi-band scheme runs one ICER instance per component with shared
//! outer metadata. This crate models that as a planar image with one
//! plane per component, every plane the same geometry:
//!
//! | [`IcerPixelFormat`] | planes | bytes / sample | `bit_depth` | wire form |
//! |---|---|---|---|---|
//! | `Gray8` | 1 | 1 | 8 (1..=8 from a cube band) | bare segment stream |
//! | `Gray16Le` | 1 | 2, little-endian, **exact LSB-aligned value** | 9..=16 | plane container tag 2 |
//! | `Yuv444P` | 3 (Y, Cb, Cr) | 1 | 8 | plane container tag 1 |
//! | `Gbrp8` | 3 (G, B, R) | 1 | 8 | plane container tag 3 |
//!
//! Deep samples are stored as the exact integer value in a 16-bit
//! little-endian word (a 12-bit sample `0xABC` is the bytes
//! `BC 0A`), never left-justified; `bit_depth` says how many low bits
//! are significant. This is the §II.C MER convention ("12-bit pixels
//! ... each stored using a 16-bit word").

use std::time::Duration;

use crate::decoder::SegmentMetadata;
use crate::error::{IcerError, Result};

/// Pixel layout of an [`IcerImage`].
///
/// Variant names mirror `oxideav_core::PixelFormat` exactly, so the
/// `registry` feature's conversion is a 1:1 name match. See the
/// [module docs](self) for the per-layout sample conventions.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum IcerPixelFormat {
    /// Single 8-bit luma plane (Mars rover Pancam / Hazcam delivery),
    /// one byte per sample.
    Gray8,
    /// Single deep luma plane: each sample an exact `9..=16`-bit value
    /// (the image's [`IcerImage::bit_depth`]) stored LSB-aligned in a
    /// little-endian `u16` (`stride >= width * 2`). The §II.C MER
    /// operating point is 12-bit pixels in 16-bit words.
    Gray16Le,
    /// 8-bit luma + 8-bit Cb + 8-bit Cr, full 4:4:4 sampling, three
    /// planes. The caller's colour-difference planes are coded as three
    /// independent ICER instances; see [`ColorInfo::yuv444p_default`]
    /// for the range / matrix convention [`IcerImage::to_rgb8`] applies.
    Yuv444P,
    /// Planar 8-bit RGB in G, B, R plane order (core's `Gbrp8`), three
    /// planes. The natural layout of [`crate::encode_rgb8`]: each
    /// component is coded losslessly by its own ICER instance, so an
    /// RGB round trip is exact with no colour matrix involved.
    Gbrp8,
}

/// The contract name for [`IcerPixelFormat`].
pub type PixelFormat = IcerPixelFormat;

impl IcerPixelFormat {
    /// Every layout, in declaration order.
    pub const ALL: [Self; 4] = [Self::Gray8, Self::Gray16Le, Self::Yuv444P, Self::Gbrp8];

    /// Number of planes carried by this format.
    pub fn plane_count(self) -> usize {
        match self {
            Self::Gray8 | Self::Gray16Le => 1,
            Self::Yuv444P | Self::Gbrp8 => 3,
        }
    }

    /// Bytes each sample occupies in [`Plane::data`] (1 for the byte
    /// formats, 2 little-endian for [`IcerPixelFormat::Gray16Le`]).
    pub fn sample_bytes(self) -> usize {
        match self {
            Self::Gray8 | Self::Yuv444P | Self::Gbrp8 => 1,
            Self::Gray16Le => 2,
        }
    }

    /// The significant bit depth an image of this layout carries when
    /// nothing more specific is declared: 8 for the byte formats, 16
    /// for `Gray16Le`.
    pub fn natural_bit_depth(self) -> u8 {
        match self {
            Self::Gray16Le => 16,
            _ => 8,
        }
    }

    /// The range of [`IcerImage::bit_depth`] values this layout admits.
    pub fn bit_depth_range(self) -> std::ops::RangeInclusive<u8> {
        match self {
            Self::Gray8 => 1..=8,
            Self::Gray16Le => 9..=16,
            Self::Yuv444P | Self::Gbrp8 => 8..=8,
        }
    }

    /// `true` for the three-plane layouts.
    pub fn is_color(self) -> bool {
        matches!(self, Self::Yuv444P | Self::Gbrp8)
    }

    /// `false` for every ICER layout (ICER carries no alpha).
    pub fn has_alpha(self) -> bool {
        false
    }

    /// Minimum bytes one row of `width` samples occupies in this layout
    /// (`width × sample_bytes`), or `None` on `usize` overflow.
    pub fn row_bytes(self, width: u32) -> Option<usize> {
        (width as usize).checked_mul(self.sample_bytes())
    }
}

/// One sample plane (single component / channel).
///
/// Mirrors `oxideav_core::VideoPlane` so the registry-side conversion
/// is a trivial field-by-field copy.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub struct Plane {
    /// Row stride in bytes (`>= width * sample_bytes`).
    pub stride: usize,
    /// Plane bytes — at least `stride * (height - 1) + row_bytes`.
    pub data: Vec<u8>,
}

impl Plane {
    /// Wrap a plane buffer with its row stride.
    pub fn new(stride: usize, data: Vec<u8>) -> Self {
        Self { stride, data }
    }
}

/// Former name of [`Plane`].
#[deprecated(note = "renamed to oxideav_icer::Plane (IMAGE_CRATE_API)")]
pub type IcerPlane = Plane;

/// Nominal sample range (H.273 `VideoFullRangeFlag`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Default)]
#[non_exhaustive]
pub enum ColorRange {
    /// No range was signalled.
    #[default]
    Unspecified,
    /// Limited (video / studio) range: `VideoFullRangeFlag == 0`.
    Limited,
    /// Full (PC) range: `VideoFullRangeFlag == 1`.
    Full,
}

/// Colour signalling of an image: the sample range plus the H.273
/// `ColourPrimaries` / `TransferCharacteristics` /
/// `MatrixCoefficients` code points (`2` = unspecified).
///
/// ICER streams carry **no** colour signalling — IPN 42-155 and
/// IPN 42-164 define a sample coder and say nothing about primaries,
/// transfer or matrices — so [`crate::decode`] fills this with the
/// crate's documented convention ([`ColorInfo::default_for`] the
/// layout) and the encoder cannot write any of it back. Because the
/// convention is assumed rather than defined by the format, the
/// `registry` adapter does **not** stamp it on framework frames.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub struct ColorInfo {
    /// Sample range.
    pub range: ColorRange,
    /// H.273 `ColourPrimaries` code point (`1` = BT.709 / sRGB, `2` =
    /// unspecified).
    pub primaries: u8,
    /// H.273 `TransferCharacteristics` code point (`2` = unspecified).
    pub transfer: u8,
    /// H.273 `MatrixCoefficients` code point (`0` = identity / RGB /
    /// Y-only, `1` = BT.709, `5` / `6` = BT.601, `2` = unspecified).
    pub matrix: u8,
}

impl ColorInfo {
    /// H.273 "unspecified" code point.
    pub const UNSPECIFIED: u8 = 2;
    /// H.273 `MatrixCoefficients` identity (RGB / GBR / Y-only) code
    /// point.
    pub const MATRIX_IDENTITY: u8 = 0;
    /// H.273 `MatrixCoefficients` BT.709 code point.
    pub const MATRIX_BT709: u8 = 1;
    /// H.273 `MatrixCoefficients` BT.470BG (BT.601 coefficients) code
    /// point.
    pub const MATRIX_BT470BG: u8 = 5;
    /// H.273 `MatrixCoefficients` SMPTE 170M (BT.601 coefficients) code
    /// point.
    pub const MATRIX_SMPTE170M: u8 = 6;

    /// Build a description from its four parts.
    pub const fn new(range: ColorRange, primaries: u8, transfer: u8, matrix: u8) -> Self {
        Self {
            range,
            primaries,
            transfer,
            matrix,
        }
    }

    /// Every field unspecified.
    pub const fn unspecified() -> Self {
        Self::new(
            ColorRange::Unspecified,
            Self::UNSPECIFIED,
            Self::UNSPECIFIED,
            Self::UNSPECIFIED,
        )
    }

    /// The convention for the single-component layouts (`Gray8`,
    /// `Gray16Le`) and planar RGB (`Gbrp8`): full range (the §III.A
    /// level shift by `2^(depth-1)` assumes samples span the whole
    /// `[0, 2^depth)` word), identity matrix, primaries and transfer
    /// unspecified — the ICER papers define no colour space, so none
    /// is invented.
    pub const fn icer_default() -> Self {
        Self::new(
            ColorRange::Full,
            Self::UNSPECIFIED,
            Self::UNSPECIFIED,
            Self::MATRIX_IDENTITY,
        )
    }

    /// The convention for the `Yuv444P` layout: limited (studio) range
    /// — the range core's `Yuv444P` name denotes — with primaries,
    /// transfer and matrix unspecified. [`IcerImage::to_rgb8`] decodes
    /// an unspecified matrix with the BT.601 coefficients (see there).
    pub const fn yuv444p_default() -> Self {
        Self::new(
            ColorRange::Limited,
            Self::UNSPECIFIED,
            Self::UNSPECIFIED,
            Self::UNSPECIFIED,
        )
    }

    /// The documented default for a layout: [`ColorInfo::yuv444p_default`]
    /// for `Yuv444P`, [`ColorInfo::icer_default`] otherwise.
    pub const fn default_for(format: PixelFormat) -> Self {
        match format {
            PixelFormat::Yuv444P => Self::yuv444p_default(),
            _ => Self::icer_default(),
        }
    }

    /// Set the range.
    pub fn with_range(mut self, range: ColorRange) -> Self {
        self.range = range;
        self
    }

    /// Set the primaries code point.
    pub fn with_primaries(mut self, primaries: u8) -> Self {
        self.primaries = primaries;
        self
    }

    /// Set the transfer code point.
    pub fn with_transfer(mut self, transfer: u8) -> Self {
        self.transfer = transfer;
        self
    }

    /// Set the matrix code point.
    pub fn with_matrix(mut self, matrix: u8) -> Self {
        self.matrix = matrix;
        self
    }

    /// `true` when both primaries and transfer are specified (`!= 2`).
    pub fn is_specified(&self) -> bool {
        self.primaries != Self::UNSPECIFIED && self.transfer != Self::UNSPECIFIED
    }
}

impl Default for ColorInfo {
    /// [`ColorInfo::icer_default`].
    fn default() -> Self {
        Self::icer_default()
    }
}

/// The metadata blobs every image crate surfaces: an ICC profile, an
/// Exif payload, an XMP packet and a file gamma.
///
/// ICER has no metadata mechanism — the 12-byte segment header
/// (IPN 42-155 §IV) and this crate's plane / cube containers carry
/// geometry and coding parameters only — so every field is `None` on a
/// decoded image and the encoder cannot carry whatever a caller sets.
/// The type exists so [`IcerImage`] has the same shape as every other
/// image crate's image.
#[derive(Clone, Debug, Default, PartialEq)]
#[non_exhaustive]
pub struct Metadata {
    /// ICC profile bytes. Always `None` from the decoder.
    pub icc: Option<Vec<u8>>,
    /// Exif payload. Always `None` from the decoder.
    pub exif: Option<Vec<u8>>,
    /// XMP packet. Always `None` from the decoder.
    pub xmp: Option<Vec<u8>>,
    /// File gamma as an encoding exponent. Always `None` from the
    /// decoder.
    pub gamma: Option<f32>,
}

impl Metadata {
    /// Empty metadata.
    pub fn new() -> Self {
        Self::default()
    }

    /// Set (or clear) the ICC profile.
    pub fn with_icc(mut self, icc: impl Into<Option<Vec<u8>>>) -> Self {
        self.icc = icc.into();
        self
    }

    /// Set (or clear) the Exif payload.
    pub fn with_exif(mut self, exif: impl Into<Option<Vec<u8>>>) -> Self {
        self.exif = exif.into();
        self
    }

    /// Set (or clear) the XMP packet.
    pub fn with_xmp(mut self, xmp: impl Into<Option<Vec<u8>>>) -> Self {
        self.xmp = xmp.into();
        self
    }

    /// Set (or clear) the file gamma.
    pub fn with_gamma(mut self, gamma: impl Into<Option<f32>>) -> Self {
        self.gamma = gamma.into();
        self
    }

    /// `true` when no field is set.
    pub fn is_empty(&self) -> bool {
        self.icc.is_none() && self.exif.is_none() && self.xmp.is_none() && self.gamma.is_none()
    }
}

/// Decoded ICER image — one plane per component plus pixel layout, as
/// returned by [`crate::decode`] and consumed by [`crate::encode`].
///
/// `planes.len() == format.plane_count()`, every plane the same
/// geometry; `color` is the crate's documented convention (the stream
/// carries none); `metadata` is always empty (ICER has none). There is
/// no palette field: ICER has no indexed form. `bit_depth` is the ICER
/// extra: the number of significant low bits in every sample (8 for
/// the byte layouts, 9..=16 for `Gray16Le`, 1..=8 for a `Gray8` band
/// decoded from a shallow ICER-3D cube).
///
/// Construct with [`IcerImage::new`] / [`IcerImage::zeros`] /
/// [`IcerImage::from_rgb8`] / [`IcerImage::from_rgba8`], which validate
/// the plane geometry so an inconsistent image cannot exist and
/// [`IcerImage::to_rgb8`] / [`IcerImage::to_rgba8`] are infallible.
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub struct IcerImage {
    /// Picture width in pixels (`1..=65535`).
    pub width: u32,
    /// Picture height in pixels (`1..=65535`).
    pub height: u32,
    /// Pixel layout the planes carry.
    pub format: PixelFormat,
    /// One [`Plane`] per component (`format.plane_count()` of them).
    pub planes: Vec<Plane>,
    /// Colour signalling (range + H.273 code points) — the documented
    /// convention, since the stream carries none.
    pub color: ColorInfo,
    /// ICC / Exif / XMP / gamma — always empty for ICER.
    pub metadata: Metadata,
    /// Significant bits per sample, LSB-aligned (see the
    /// [module docs](self)): `8` for `Gray8` / `Yuv444P` / `Gbrp8`,
    /// `9..=16` for `Gray16Le`. The 2-D encoder writes it to the deep
    /// plane container; the ICER-3D cube header carries it for every
    /// band.
    pub bit_depth: u8,
}

impl IcerImage {
    /// Assemble an image from its geometry, layout and planes
    /// (`format.plane_count()` of them). Colour is
    /// [`ColorInfo::default_for`] the layout, metadata empty and
    /// `bit_depth` the layout's [`IcerPixelFormat::natural_bit_depth`];
    /// the `with_*` builders fill those in.
    ///
    /// Validates the geometry and returns [`IcerError::InvalidData`]
    /// when `width` or `height` is `0` or above `65535` (the 16-bit
    /// segment-header fields), when the plane count disagrees with the
    /// layout, when a plane's `stride` is below the layout's row size
    /// ([`IcerPixelFormat::row_bytes`]), or when its `data` is shorter
    /// than `stride × (height − 1) + row_bytes`.
    pub fn new(width: u32, height: u32, format: PixelFormat, planes: Vec<Plane>) -> Result<Self> {
        let img = Self {
            width,
            height,
            format,
            planes,
            color: ColorInfo::default_for(format),
            metadata: Metadata::default(),
            bit_depth: format.natural_bit_depth(),
        };
        img.validate()?;
        Ok(img)
    }

    /// Build a fresh, fully-zero image of the requested geometry with
    /// tightly packed planes (`stride = width × sample_bytes`) and the
    /// layout's natural bit depth. Used by the inverse-transform path to
    /// allocate the reconstruction buffer before pixel writes.
    ///
    /// Panics if the geometry overflows `usize` (the decoder checks
    /// its limits first; callers building images by hand use
    /// [`IcerImage::new`] for a fallible path).
    pub fn zeros(width: u32, height: u32, format: PixelFormat) -> Self {
        let stride = width as usize * format.sample_bytes();
        let plane = Plane::new(stride, vec![0u8; stride * height as usize]);
        let mut planes = Vec::with_capacity(format.plane_count());
        for _ in 0..format.plane_count() {
            planes.push(plane.clone());
        }
        Self {
            width,
            height,
            format,
            planes,
            color: ColorInfo::default_for(format),
            metadata: Metadata::default(),
            bit_depth: format.natural_bit_depth(),
        }
    }

    /// [`IcerImage::zeros`] for a deep (`9..=16`-bit) grayscale image:
    /// a `Gray16Le` plane with `bit_depth = bits`. Returns
    /// [`IcerError::InvalidData`] when `bits` is outside `9..=16`.
    pub fn zeros_deep(width: u32, height: u32, bits: u8) -> Result<Self> {
        Self::zeros(width, height, PixelFormat::Gray16Le).with_bit_depth(bits)
    }

    /// Tightly packed planar RGB (`Gbrp8`) from `3 × width × height`
    /// interleaved R, G, B bytes (more is tolerated; fewer is
    /// [`IcerError::InvalidData`]). Encodes losslessly as three
    /// independent ICER component streams.
    pub fn from_rgb8(width: u32, height: u32, data: Vec<u8>) -> Result<Self> {
        Self::from_interleaved(width, height, &data, 3)
    }

    /// Tightly packed planar RGB (`Gbrp8`) from `4 × width × height`
    /// interleaved R, G, B, A bytes (more is tolerated; fewer is
    /// [`IcerError::InvalidData`]). **Alpha is dropped** — ICER has no
    /// alpha mechanism (documented, like [`crate::encode_rgba8`]).
    pub fn from_rgba8(width: u32, height: u32, data: Vec<u8>) -> Result<Self> {
        Self::from_interleaved(width, height, &data, 4)
    }

    fn from_interleaved(width: u32, height: u32, data: &[u8], bpp: usize) -> Result<Self> {
        check_dimensions(width, height)?;
        let w = width as usize;
        let h = height as usize;
        let need = w
            .checked_mul(h)
            .and_then(|n| n.checked_mul(bpp))
            .ok_or_else(|| IcerError::invalid("icer: image size overflows usize"))?;
        if data.len() < need {
            return Err(IcerError::invalid(format!(
                "icer: {} interleaved bytes, geometry needs {need}",
                data.len()
            )));
        }
        let mut g = Vec::with_capacity(w * h);
        let mut b = Vec::with_capacity(w * h);
        let mut r = Vec::with_capacity(w * h);
        for px in data[..need].chunks_exact(bpp) {
            r.push(px[0]);
            g.push(px[1]);
            b.push(px[2]);
        }
        Self::new(
            width,
            height,
            PixelFormat::Gbrp8,
            vec![Plane::new(w, g), Plane::new(w, b), Plane::new(w, r)],
        )
    }

    /// Set the colour signalling. Informational only — ICER cannot
    /// carry it, so the encoder ignores it; [`IcerImage::to_rgb8`]
    /// honours `range` / `matrix` on a `Yuv444P` image.
    pub fn with_color(mut self, color: ColorInfo) -> Self {
        self.color = color;
        self
    }

    /// Set the metadata. ICER cannot carry any of it; the encoder
    /// ignores it.
    pub fn with_metadata(mut self, metadata: Metadata) -> Self {
        self.metadata = metadata;
        self
    }

    /// Declare the significant bits per sample. Fallible so an invalid
    /// image cannot exist: [`IcerError::InvalidData`] when `bits` is
    /// outside the layout's [`IcerPixelFormat::bit_depth_range`]
    /// (`1..=8` for `Gray8`, `9..=16` for `Gray16Le`, exactly `8` for
    /// the colour layouts).
    pub fn with_bit_depth(mut self, bits: u8) -> Result<Self> {
        if !self.format.bit_depth_range().contains(&bits) {
            return Err(IcerError::invalid(format!(
                "icer: bit depth {bits} outside {:?} for {:?}",
                self.format.bit_depth_range(),
                self.format
            )));
        }
        self.bit_depth = bits;
        Ok(self)
    }

    /// Image width in pixels.
    pub fn width(&self) -> u32 {
        self.width
    }

    /// Image height in pixels.
    pub fn height(&self) -> u32 {
        self.height
    }

    /// Native pixel layout.
    pub fn format(&self) -> PixelFormat {
        self.format
    }

    /// Significant bits per sample (see [`Self::bit_depth`]).
    pub fn bit_depth(&self) -> u8 {
        self.bit_depth
    }

    /// `false`: ICER carries no alpha.
    pub fn has_alpha(&self) -> bool {
        false
    }

    /// Row stride in bytes of the first plane (`0` if there is none).
    pub fn stride(&self) -> usize {
        self.planes.first().map(|p| p.stride).unwrap_or(0)
    }

    /// Minimum bytes one row of a plane occupies (`width ×
    /// sample_bytes`; saturates on overflow). A plane's actual `stride`
    /// may be larger.
    pub fn min_row_bytes(&self) -> usize {
        self.format.row_bytes(self.width).unwrap_or(usize::MAX)
    }

    /// Validate that every plane carries enough bytes for the declared
    /// `width` × `height` × layout, given the plane's own `stride`, and
    /// that `bit_depth` is in the layout's range. [`Self::new`] runs
    /// this, so a decoder- or constructor-produced image always passes;
    /// a caller who mutated the public fields afterwards can re-check.
    pub fn validate(&self) -> Result<()> {
        check_dimensions(self.width, self.height)?;
        if !self.format.bit_depth_range().contains(&self.bit_depth) {
            return Err(IcerError::invalid(format!(
                "icer: bit depth {} outside {:?} for {:?}",
                self.bit_depth,
                self.format.bit_depth_range(),
                self.format
            )));
        }
        let n = self.format.plane_count();
        if self.planes.len() != n {
            return Err(IcerError::invalid(format!(
                "icer: {:?} needs {n} planes, got {}",
                self.format,
                self.planes.len()
            )));
        }
        let min_row = self
            .format
            .row_bytes(self.width)
            .ok_or_else(|| IcerError::invalid("icer: row size overflows usize"))?;
        for (i, plane) in self.planes.iter().enumerate() {
            if plane.stride < min_row {
                return Err(IcerError::invalid(format!(
                    "icer: plane {i} stride {} below row size {min_row}",
                    plane.stride
                )));
            }
            let need = plane
                .stride
                .checked_mul(self.height as usize - 1)
                .and_then(|n| n.checked_add(min_row))
                .ok_or_else(|| IcerError::invalid("icer: plane size overflows usize"))?;
            if plane.data.len() < need {
                return Err(IcerError::invalid(format!(
                    "icer: plane {i} holds {} bytes, geometry needs {need}",
                    plane.data.len()
                )));
            }
        }
        Ok(())
    }

    /// The pixel bytes of the single plane — `Some` for the packed
    /// single-component layouts (`Gray8`, `Gray16Le`), `None` for the
    /// planar colour layouts (use [`Self::into_raw`] or `planes`).
    pub fn as_bytes(&self) -> Option<&[u8]> {
        if self.format.plane_count() == 1 {
            self.planes.first().map(|p| p.data.as_slice())
        } else {
            None
        }
    }

    /// Consume the image and return its plane bytes — the single plane
    /// for the gray layouts, the three planes concatenated in order
    /// (strides as reported) for `Yuv444P` / `Gbrp8`.
    pub fn into_raw(self) -> Vec<u8> {
        let mut planes = self.planes.into_iter();
        let mut out = planes.next().map(|p| p.data).unwrap_or_default();
        for p in planes {
            out.extend_from_slice(&p.data);
        }
        out
    }

    /// Read the sample at `(x, y)` of plane `plane_idx`, widened to
    /// `u16` (a byte-layout sample occupies the low 8 bits). Panics on
    /// an out-of-range plane / coordinate, like a slice index.
    pub fn sample(&self, plane_idx: usize, x: u32, y: u32) -> u16 {
        assert!(x < self.width && y < self.height, "sample out of bounds");
        let plane = &self.planes[plane_idx];
        let sb = self.format.sample_bytes();
        let off = y as usize * plane.stride + x as usize * sb;
        if sb == 2 {
            u16::from_le_bytes([plane.data[off], plane.data[off + 1]])
        } else {
            plane.data[off] as u16
        }
    }

    /// Write the sample at `(x, y)` of plane `plane_idx`. For the byte
    /// layouts the value's low 8 bits are stored; for `Gray16Le` the
    /// full 16-bit word is stored little-endian (callers are expected
    /// to stay within `0..2^bit_depth`). Panics on an out-of-range
    /// plane / coordinate.
    pub fn set_sample(&mut self, plane_idx: usize, x: u32, y: u32, value: u16) {
        assert!(x < self.width && y < self.height, "sample out of bounds");
        let sb = self.format.sample_bytes();
        let plane = &mut self.planes[plane_idx];
        let off = y as usize * plane.stride + x as usize * sb;
        if sb == 2 {
            plane.data[off..off + 2].copy_from_slice(&value.to_le_bytes());
        } else {
            plane.data[off] = value as u8;
        }
    }

    /// Tightly packed 8-bit RGBA, `4 × width` bytes per row, alpha
    /// always `255` (ICER has no alpha). Exact integer kernels per
    /// layout:
    ///
    /// * `Gray8` / `Gray16Le`: the sample is clamped to
    ///   `2^bit_depth − 1` and reduced by round-half-up
    ///   `(v × 255 + max / 2) / max`, `max = 2^bit_depth − 1` (the
    ///   identity at depth 8), then replicated to R = G = B.
    /// * `Gbrp8`: the G, B, R planes are re-interleaved as R, G, B.
    /// * `Yuv444P`: Y, Cb, Cr → R, G, B with the BT.601 coefficients
    ///   (`Kr = 0.299`, `Kb = 0.114`) unless `color.matrix` is `1`
    ///   (BT.709, `Kr = 0.2126`, `Kb = 0.0722`); limited range
    ///   (`Y 16..235`, `C 16..240`) unless `color.range` is `Full`.
    ///   Fixed-point 16-fraction-bit arithmetic, rounded, clamped to
    ///   `0..=255`. No colour management is applied.
    ///
    /// Infallible on any image produced by the decoder or a constructor;
    /// a plane too short for the geometry (after mutating the public
    /// fields) leaves the missing pixels zero / opaque.
    pub fn to_rgba8(&self) -> Vec<u8> {
        let w = self.width as usize;
        let h = self.height as usize;
        let mut out = vec![0u8; w.saturating_mul(h).saturating_mul(4)];
        let rgb = self.to_rgb8();
        for (s, d) in rgb.chunks_exact(3).zip(out.chunks_exact_mut(4)) {
            d[..3].copy_from_slice(s);
            d[3] = 255;
        }
        // Rows the RGB kernel could not read stay zero; keep them opaque.
        for d in out.chunks_exact_mut(4).skip(rgb.len() / 3) {
            d[3] = 255;
        }
        out
    }

    /// Tightly packed 8-bit RGB, `3 × width` bytes per row — the same
    /// kernels as [`Self::to_rgba8`] without the alpha byte.
    pub fn to_rgb8(&self) -> Vec<u8> {
        let w = self.width as usize;
        let h = self.height as usize;
        let mut out = vec![0u8; w.saturating_mul(h).saturating_mul(3)];
        let row_bytes = self.min_row_bytes();
        fn row(plane: &Plane, y: usize, row_bytes: usize) -> Option<&[u8]> {
            plane_row(plane, y, row_bytes)
        }
        match self.format {
            PixelFormat::Gray8 | PixelFormat::Gray16Le => {
                let Some(plane) = self.planes.first() else {
                    return out;
                };
                let depth = self.bit_depth.clamp(1, 16);
                let max: u32 = (1u32 << depth) - 1;
                for y in 0..h {
                    let Some(src) = row(plane, y, row_bytes) else {
                        break;
                    };
                    let dst = &mut out[y * w * 3..(y + 1) * w * 3];
                    if self.format == PixelFormat::Gray8 {
                        for (s, d) in src.iter().zip(dst.chunks_exact_mut(3)) {
                            let v = scale_to_u8(*s as u32, max);
                            d.fill(v);
                        }
                    } else {
                        for (s, d) in src.chunks_exact(2).zip(dst.chunks_exact_mut(3)) {
                            let v = scale_to_u8(u16::from_le_bytes([s[0], s[1]]) as u32, max);
                            d.fill(v);
                        }
                    }
                }
            }
            PixelFormat::Gbrp8 => {
                if self.planes.len() < 3 {
                    return out;
                }
                for y in 0..h {
                    let (Some(g), Some(b), Some(r)) = (
                        row(&self.planes[0], y, row_bytes),
                        row(&self.planes[1], y, row_bytes),
                        row(&self.planes[2], y, row_bytes),
                    ) else {
                        break;
                    };
                    let dst = &mut out[y * w * 3..(y + 1) * w * 3];
                    for (x, d) in dst.chunks_exact_mut(3).enumerate() {
                        d[0] = r[x];
                        d[1] = g[x];
                        d[2] = b[x];
                    }
                }
            }
            PixelFormat::Yuv444P => {
                if self.planes.len() < 3 {
                    return out;
                }
                let kernel = YcbcrKernel::new(&self.color);
                for y in 0..h {
                    let (Some(yp), Some(cb), Some(cr)) = (
                        row(&self.planes[0], y, row_bytes),
                        row(&self.planes[1], y, row_bytes),
                        row(&self.planes[2], y, row_bytes),
                    ) else {
                        break;
                    };
                    let dst = &mut out[y * w * 3..(y + 1) * w * 3];
                    for (x, d) in dst.chunks_exact_mut(3).enumerate() {
                        let [r, g, b] = kernel.rgb(yp[x], cb[x], cr[x]);
                        d[0] = r;
                        d[1] = g;
                        d[2] = b;
                    }
                }
            }
        }
        out
    }
}

/// Row `y` of `plane` (`row_bytes` long), or `None` when the plane is
/// too short / the offset overflows.
fn plane_row(plane: &Plane, y: usize, row_bytes: usize) -> Option<&[u8]> {
    let start = y.checked_mul(plane.stride)?;
    let end = start.checked_add(row_bytes)?;
    plane.data.get(start..end)
}

/// `width` / `height` must fit the 16-bit segment-header fields and be
/// non-zero.
fn check_dimensions(width: u32, height: u32) -> Result<()> {
    if width == 0 || height == 0 {
        return Err(IcerError::invalid(
            "icer: width and height must be non-zero",
        ));
    }
    if width > u16::MAX as u32 || height > u16::MAX as u32 {
        return Err(IcerError::invalid(format!(
            "icer: geometry {width}x{height} exceeds the 16-bit segment-header fields"
        )));
    }
    Ok(())
}

/// Reduce a `depth`-bit sample (`max = 2^depth − 1`) to 8 bits by
/// round-half-up `(v × 255 + max / 2) / max`, clamping `v` to `max`
/// first. The identity for `max == 255`.
#[inline]
pub(crate) fn scale_to_u8(v: u32, max: u32) -> u8 {
    let v = v.min(max);
    if max == 255 {
        return v as u8;
    }
    ((v as u64 * 255 + (max as u64 / 2)) / max as u64) as u8
}

/// Fixed-point (16 fraction bits) YCbCr → RGB kernel for one
/// [`ColorInfo`]: BT.601 coefficients unless `matrix == 1` (BT.709);
/// limited range unless `range == Full`.
struct YcbcrKernel {
    y_off: i32,
    y_scale: i32,
    c_scale: i32,
    kr: f64,
    kb: f64,
}

impl YcbcrKernel {
    const FRAC: i32 = 16;

    fn new(color: &ColorInfo) -> Self {
        let (kr, kb) = if color.matrix == ColorInfo::MATRIX_BT709 {
            (0.2126, 0.0722)
        } else {
            (0.299, 0.114)
        };
        let full = color.range == ColorRange::Full;
        let (y_off, y_scale, c_scale) = if full {
            (0, 1.0, 1.0)
        } else {
            (16, 255.0 / 219.0, 255.0 / 224.0)
        };
        let fp = |v: f64| (v * (1u32 << Self::FRAC) as f64).round() as i32;
        Self {
            y_off,
            y_scale: fp(y_scale),
            c_scale: fp(c_scale),
            kr,
            kb,
        }
    }

    #[inline]
    fn rgb(&self, y: u8, cb: u8, cr: u8) -> [u8; 3] {
        let fp = |v: f64| (v * (1u32 << Self::FRAC) as f64).round() as i64;
        let kg = 1.0 - self.kr - self.kb;
        let yy = (y as i32 - self.y_off) as i64 * self.y_scale as i64;
        let cbb = (cb as i32 - 128) as i64 * self.c_scale as i64;
        let crr = (cr as i32 - 128) as i64 * self.c_scale as i64;
        let half = 1i64 << (2 * Self::FRAC - 1);
        let r =
            (yy * (1 << Self::FRAC) + crr * fp(2.0 * (1.0 - self.kr)) + half) >> (2 * Self::FRAC);
        let g = (yy * (1 << Self::FRAC)
            - cbb * fp(2.0 * (1.0 - self.kb) * self.kb / kg)
            - crr * fp(2.0 * (1.0 - self.kr) * self.kr / kg)
            + half)
            >> (2 * Self::FRAC);
        let b =
            (yy * (1 << Self::FRAC) + cbb * fp(2.0 * (1.0 - self.kb)) + half) >> (2 * Self::FRAC);
        [
            r.clamp(0, 255) as u8,
            g.clamp(0, 255) as u8,
            b.clamp(0, 255) as u8,
        ]
    }
}

/// Tightly packed 8-bit RGB image: `width × height × 3` bytes,
/// row-major, no padding. What [`crate::decode_rgb8`] returns.
#[derive(Clone, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub struct RgbImage {
    /// Width in pixels.
    pub width: u32,
    /// Height in pixels.
    pub height: u32,
    /// `width × height × 3` bytes, R, G, B per pixel.
    pub data: Vec<u8>,
}

impl RgbImage {
    /// Wrap a packed RGB buffer.
    pub fn new(width: u32, height: u32, data: Vec<u8>) -> Self {
        Self {
            width,
            height,
            data,
        }
    }

    /// The pixel bytes.
    pub fn as_bytes(&self) -> &[u8] {
        &self.data
    }

    /// Consume and return the pixel bytes.
    pub fn into_raw(self) -> Vec<u8> {
        self.data
    }
}

/// Tightly packed 8-bit RGBA image: `width × height × 4` bytes,
/// row-major, no padding. What [`crate::decode_rgba8`] returns.
#[derive(Clone, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub struct RgbaImage {
    /// Width in pixels.
    pub width: u32,
    /// Height in pixels.
    pub height: u32,
    /// `width × height × 4` bytes, R, G, B, A per pixel.
    pub data: Vec<u8>,
}

impl RgbaImage {
    /// Wrap a packed RGBA buffer.
    pub fn new(width: u32, height: u32, data: Vec<u8>) -> Self {
        Self {
            width,
            height,
            data,
        }
    }

    /// The pixel bytes.
    pub fn as_bytes(&self) -> &[u8] {
        &self.data
    }

    /// Consume and return the pixel bytes.
    pub fn into_raw(self) -> Vec<u8> {
        self.data
    }
}

/// Which of the crate's three wire forms a stream uses — reported by
/// [`crate::info`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum StreamKind {
    /// A bare 2-D segment stream (IPN 42-155 §IV framing): `Gray8`.
    Segments,
    /// The crate's multi-plane / deep-gray plane container around 2-D
    /// segment streams (`Gray16Le`, `Yuv444P`, `Gbrp8`).
    PlaneContainer,
    /// An ICER-3D (IPN 42-164) hyperspectral cube.
    Cube,
}

/// What [`crate::info`] reads from the framing without decoding pixels.
#[derive(Clone, Debug)]
#[non_exhaustive]
pub struct ImageInfo {
    /// Width in pixels.
    pub width: u32,
    /// Height in pixels (the stitched height of every row-strip
    /// segment for a 2-D stream; the spatial height for a cube).
    pub height: u32,
    /// Native layout [`crate::decode`] would return.
    pub format: PixelFormat,
    /// Number of images: `1` for a 2-D stream, the number of spectral
    /// bands for an ICER-3D cube (see [`crate::decode_all`]).
    pub frames: u32,
    /// Always `false` (ICER carries no alpha).
    pub has_alpha: bool,
    /// Colour signalling — the documented convention for the layout.
    pub color: ColorInfo,
    /// Always `false` (ICER carries no ICC profile).
    pub has_icc: bool,
    /// Always `false` (ICER carries no Exif).
    pub has_exif: bool,
    /// Always `false` (ICER carries no XMP).
    pub has_xmp: bool,
    /// Significant bits per sample (see [`IcerImage::bit_depth`]).
    pub bit_depth: u8,
    /// Which wire form the stream uses.
    pub kind: StreamKind,
    /// The 2-D segment walk — one record per segment (every plane's
    /// segments, in plane order, for a plane container; offsets are
    /// relative to the whole input). Empty for a cube.
    pub segments: Vec<SegmentMetadata>,
}

impl ImageInfo {
    /// Number of 2-D segments in the stream (`0` for a cube).
    pub fn segment_count(&self) -> usize {
        self.segments.len()
    }
}

/// One image of [`crate::decode_all`]: the single image of a 2-D
/// stream, or one spectral band of an ICER-3D cube. ICER has no timing,
/// so `delay` is always `None`.
#[derive(Clone, Debug)]
#[non_exhaustive]
pub struct Frame {
    /// The decoded image in its native layout.
    pub image: IcerImage,
    /// Always `None` (ICER carries no timing).
    pub delay: Option<Duration>,
    /// Zero-based index of this image: the spectral band (slice) of a
    /// cube, `0` for a 2-D stream.
    pub index: u32,
}

impl Frame {
    /// Wrap an image with its slice index.
    pub fn new(image: IcerImage, index: u32) -> Self {
        Self {
            image,
            delay: None,
            index,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_validates_geometry_and_planes() {
        assert!(IcerImage::new(0, 1, PixelFormat::Gray8, vec![Plane::new(0, vec![])]).is_err());
        assert!(IcerImage::new(
            70000,
            1,
            PixelFormat::Gray8,
            vec![Plane::new(70000, vec![])]
        )
        .is_err());
        // Short plane.
        assert!(IcerImage::new(4, 2, PixelFormat::Gray8, vec![Plane::new(4, vec![0; 7])]).is_err());
        // Stride below row size.
        assert!(IcerImage::new(
            4,
            2,
            PixelFormat::Gray16Le,
            vec![Plane::new(4, vec![0; 16])]
        )
        .is_err());
        // Wrong plane count.
        assert!(IcerImage::new(2, 2, PixelFormat::Gbrp8, vec![Plane::new(2, vec![0; 4])]).is_err());
        // Last row may omit padding.
        let img = IcerImage::new(4, 2, PixelFormat::Gray8, vec![Plane::new(6, vec![0; 10])]);
        assert!(img.is_ok());
        assert_eq!(img.unwrap().bit_depth, 8);
    }

    #[test]
    fn bit_depth_builder_is_range_checked() {
        let g16 = IcerImage::zeros(2, 2, PixelFormat::Gray16Le);
        assert_eq!(g16.bit_depth, 16);
        assert!(g16.clone().with_bit_depth(8).is_err());
        assert!(g16.clone().with_bit_depth(17).is_err());
        assert_eq!(g16.with_bit_depth(12).unwrap().bit_depth, 12);
        assert!(IcerImage::zeros(2, 2, PixelFormat::Gray8)
            .with_bit_depth(9)
            .is_err());
        assert!(IcerImage::zeros(2, 2, PixelFormat::Yuv444P)
            .with_bit_depth(7)
            .is_err());
        assert!(IcerImage::zeros_deep(2, 2, 8).is_err());
    }

    #[test]
    fn deep_gray_tone_scales_by_round_half_up() {
        let mut img = IcerImage::zeros_deep(4, 1, 12).unwrap();
        for (x, v) in [0u16, 1, 2048, 4095].into_iter().enumerate() {
            img.set_sample(0, x as u32, 0, v);
        }
        let rgb = img.to_rgb8();
        // (1*255 + 2047) / 4095 = 0 ; (2048*255 + 2047)/4095 = 128
        assert_eq!(rgb, [0, 0, 0, 0, 0, 0, 128, 128, 128, 255, 255, 255]);
        let rgba = img.to_rgba8();
        assert_eq!(rgba.len(), 16);
        assert!(rgba.chunks_exact(4).all(|p| p[3] == 255));
        // Out-of-range samples clamp instead of wrapping.
        img.set_sample(0, 0, 0, 0xFFFF);
        assert_eq!(&img.to_rgb8()[..3], &[255, 255, 255]);
    }

    #[test]
    fn gray8_is_identity_and_shallow_depths_scale() {
        let mut img = IcerImage::zeros(3, 1, PixelFormat::Gray8);
        img.planes[0].data = vec![0, 100, 255];
        assert_eq!(img.to_rgb8(), [0, 0, 0, 100, 100, 100, 255, 255, 255]);
        let four_bit = IcerImage::zeros(2, 1, PixelFormat::Gray8)
            .with_bit_depth(4)
            .unwrap();
        let mut four_bit = four_bit;
        four_bit.planes[0].data = vec![15, 7];
        // 15 -> 255 ; (7*255 + 7)/15 = 119
        assert_eq!(four_bit.to_rgb8(), [255, 255, 255, 119, 119, 119]);
    }

    #[test]
    fn gbrp_round_trips_interleaved_rgb_exactly() {
        let rgb: Vec<u8> = (0..2 * 3 * 3).map(|i| (i * 37 % 256) as u8).collect();
        let img = IcerImage::from_rgb8(3, 2, rgb.clone()).unwrap();
        assert_eq!(img.format, PixelFormat::Gbrp8);
        assert_eq!(img.planes.len(), 3);
        assert_eq!(img.planes[2].data[0], rgb[0]); // R plane is third
        assert_eq!(img.to_rgb8(), rgb);
        assert!(img.as_bytes().is_none());
        let rgba: Vec<u8> = rgb
            .chunks_exact(3)
            .flat_map(|p| [p[0], p[1], p[2], 7])
            .collect();
        let img2 = IcerImage::from_rgba8(3, 2, rgba).unwrap();
        assert_eq!(img2.to_rgb8(), rgb);
        assert!(img2.to_rgba8().chunks_exact(4).all(|p| p[3] == 255));
        assert!(IcerImage::from_rgb8(3, 2, vec![0; 17]).is_err());
        assert_eq!(img.into_raw().len(), 18);
    }

    #[test]
    fn yuv444p_kernel_hits_the_studio_swing_anchors() {
        let mut img = IcerImage::zeros(3, 1, PixelFormat::Yuv444P);
        // black, white, mid-grey in limited range
        img.planes[0].data = vec![16, 235, 126];
        img.planes[1].data = vec![128, 128, 128];
        img.planes[2].data = vec![128, 128, 128];
        let rgb = img.to_rgb8();
        assert_eq!(&rgb[..6], &[0, 0, 0, 255, 255, 255]);
        assert_eq!(rgb[6], rgb[7]);
        assert_eq!(rgb[7], rgb[8]);
        assert_eq!(rgb[6], 128); // (126-16)*255/219 = 128.08
                                 // Full range is the identity on the luma axis.
        let full = img
            .clone()
            .with_color(ColorInfo::yuv444p_default().with_range(ColorRange::Full));
        let rgb = full.to_rgb8();
        assert_eq!(&rgb[..3], &[16, 16, 16]);
        assert_eq!(&rgb[3..6], &[235, 235, 235]);
        // Pure red in BT.601 limited: Y=81, Cb=90, Cr=240.
        let mut red = IcerImage::zeros(1, 1, PixelFormat::Yuv444P);
        red.planes[0].data = vec![81];
        red.planes[1].data = vec![90];
        red.planes[2].data = vec![240];
        let rgb = red.to_rgb8();
        assert!(rgb[0] >= 254 && rgb[1] <= 1 && rgb[2] <= 1, "{rgb:?}");
        // BT.709 limited pure red: Y=63, Cb=102, Cr=240.
        let mut red709 = IcerImage::zeros(1, 1, PixelFormat::Yuv444P);
        red709.planes[0].data = vec![63];
        red709.planes[1].data = vec![102];
        red709.planes[2].data = vec![240];
        let red709 =
            red709.with_color(ColorInfo::yuv444p_default().with_matrix(ColorInfo::MATRIX_BT709));
        let rgb = red709.to_rgb8();
        assert!(rgb[0] >= 254 && rgb[1] <= 1 && rgb[2] <= 1, "{rgb:?}");
    }

    #[test]
    fn defaults_follow_the_documented_convention() {
        assert_eq!(ColorInfo::default(), ColorInfo::icer_default());
        assert_eq!(
            ColorInfo::default_for(PixelFormat::Gray8).range,
            ColorRange::Full
        );
        assert_eq!(
            ColorInfo::default_for(PixelFormat::Yuv444P).range,
            ColorRange::Limited
        );
        assert!(!ColorInfo::default().is_specified());
        assert!(Metadata::new().is_empty());
        assert!(!Metadata::new().with_gamma(0.45455).is_empty());
        for f in PixelFormat::ALL {
            assert!(!f.has_alpha());
            assert!(f.bit_depth_range().contains(&f.natural_bit_depth()));
            assert_eq!(IcerImage::zeros(1, 1, f).planes.len(), f.plane_count());
        }
    }
}
