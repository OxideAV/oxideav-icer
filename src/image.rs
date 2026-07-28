//! In-memory image type returned by the standalone API.
//!
//! Kept deliberately minimal so the standalone (no-`registry`) build
//! never references `oxideav-core` types. The `registry::From` impls
//! convert this to `oxideav_core::Frame` when the feature is on.

/// Pixel layout of an [`IcerImage`].
///
/// Models the formats Mars-rover ICER deployments use in practice:
/// monochrome luma — 8-bit or deep (IPN 42-155 §II.C: "On MER, all
/// cameras produce 12-bit pixels and each is stored using a 16-bit
/// word"; §VII benchmarks 12-bit Mars surface imagery throughout) —
/// optionally as a co-sited colour triplet. The IPN report (Kiely &
/// Klimesh 2003 §III) describes ICER as fundamentally a
/// single-component coder; the deployed multi-band scheme runs three
/// independent ICER instances with shared header metadata — modelled
/// here as a single planar image with one or three planes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IcerPixelFormat {
    /// Single 8-bit luma plane (Mars rover Pancam / Hazcam delivery).
    Gray8,
    /// 8-bit luma + 8-bit Cb + 8-bit Cr, full 4:4:4 sampling.
    Yuv444P,
    /// Single deep luma plane, `bits` significant bits per sample with
    /// `9 <= bits <= 16` (the §II.C MER operating point is 12-bit
    /// pixels in 16-bit words). Samples are stored LSB-aligned as
    /// **little-endian `u16`** pairs in [`IcerPlane::data`] (matching
    /// the plain-gray deep-word convention of the `oxideav-core`
    /// `Gray16Le` ladder), so `stride >= width * 2`.
    GrayDeep {
        /// Significant bits per sample, `9..=16`.
        bits: u8,
    },
}

impl IcerPixelFormat {
    /// Number of planes carried by this format.
    pub fn plane_count(self) -> usize {
        match self {
            IcerPixelFormat::Gray8 | IcerPixelFormat::GrayDeep { .. } => 1,
            IcerPixelFormat::Yuv444P => 3,
        }
    }

    /// Significant bits per sample (8 for the byte formats).
    pub fn bit_depth(self) -> u8 {
        match self {
            IcerPixelFormat::Gray8 | IcerPixelFormat::Yuv444P => 8,
            IcerPixelFormat::GrayDeep { bits } => bits,
        }
    }

    /// Bytes each sample occupies in [`IcerPlane::data`] (1 for the
    /// byte formats, 2 little-endian for [`IcerPixelFormat::GrayDeep`]).
    pub fn sample_bytes(self) -> usize {
        match self {
            IcerPixelFormat::Gray8 | IcerPixelFormat::Yuv444P => 1,
            IcerPixelFormat::GrayDeep { .. } => 2,
        }
    }
}

/// One sample plane (single component / channel).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct IcerPlane {
    /// Row stride in bytes (`>= width * sample_bytes`).
    pub stride: usize,
    /// Plane bytes — `height * stride`.
    pub data: Vec<u8>,
}

/// Decoded ICER image — one or more planes plus pixel layout.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct IcerImage {
    pub width: u32,
    pub height: u32,
    pub pixel_format: IcerPixelFormat,
    pub planes: Vec<IcerPlane>,
    /// Decoded presentation timestamp — meaningless for still ICER but
    /// preserved when wrapping in a video pipeline.
    pub pts: i64,
}

impl IcerImage {
    /// Build a fresh, fully-zero image of the requested geometry. Used
    /// by the inverse-transform path to allocate the reconstruction
    /// buffer before pixel writes.
    pub fn zeros(width: u32, height: u32, pixel_format: IcerPixelFormat) -> Self {
        let stride = width as usize * pixel_format.sample_bytes();
        let plane = IcerPlane {
            stride,
            data: vec![0u8; stride * height as usize],
        };
        let mut planes = Vec::with_capacity(pixel_format.plane_count());
        for _ in 0..pixel_format.plane_count() {
            planes.push(plane.clone());
        }
        Self {
            width,
            height,
            pixel_format,
            planes,
            pts: 0,
        }
    }

    /// Read the sample at `(x, y)` of plane `plane_idx`, widened to
    /// `u16` (a Gray8 / Yuv444P sample occupies the low 8 bits).
    /// Panics on an out-of-range plane / coordinate, like a slice
    /// index.
    pub fn sample(&self, plane_idx: usize, x: u32, y: u32) -> u16 {
        assert!(x < self.width && y < self.height, "sample out of bounds");
        let plane = &self.planes[plane_idx];
        let sb = self.pixel_format.sample_bytes();
        let off = y as usize * plane.stride + x as usize * sb;
        if sb == 2 {
            u16::from_le_bytes([plane.data[off], plane.data[off + 1]])
        } else {
            plane.data[off] as u16
        }
    }

    /// Write the sample at `(x, y)` of plane `plane_idx`. For the byte
    /// formats the value's low 8 bits are stored; for
    /// [`IcerPixelFormat::GrayDeep`] the full 16-bit word is stored
    /// little-endian (callers are expected to stay within
    /// `0..2^bits`). Panics on an out-of-range plane / coordinate.
    pub fn set_sample(&mut self, plane_idx: usize, x: u32, y: u32, value: u16) {
        assert!(x < self.width && y < self.height, "sample out of bounds");
        let sb = self.pixel_format.sample_bytes();
        let plane = &mut self.planes[plane_idx];
        let off = y as usize * plane.stride + x as usize * sb;
        if sb == 2 {
            plane.data[off..off + 2].copy_from_slice(&value.to_le_bytes());
        } else {
            plane.data[off] = value as u8;
        }
    }
}
