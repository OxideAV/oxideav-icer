//! In-memory image type returned by the standalone API.
//!
//! Kept deliberately minimal so the standalone (no-`registry`) build
//! never references `oxideav-core` types. The `registry::From` impls
//! convert this to `oxideav_core::Frame` when the feature is on.

/// Pixel layout of an [`IcerImage`].
///
/// Round 1 only models the formats Mars-rover ICER deployments use in
/// practice: monochrome 8-bit luma, optionally with a paired chroma
/// pair. The IPN report (Kiely & Klimesh 2003 §III) describes ICER as
/// fundamentally a single-component coder; the deployed multi-band
/// scheme runs three independent ICER instances with shared header
/// metadata — modelled here as a single planar image with one to four
/// planes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IcerPixelFormat {
    /// Single 8-bit luma plane (Mars rover Pancam / Hazcam delivery).
    Gray8,
    /// 8-bit luma + 8-bit Cb + 8-bit Cr, full 4:4:4 sampling.
    Yuv444P,
}

impl IcerPixelFormat {
    /// Number of planes carried by this format.
    pub fn plane_count(self) -> usize {
        match self {
            IcerPixelFormat::Gray8 => 1,
            IcerPixelFormat::Yuv444P => 3,
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
        let stride = width as usize;
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
}
