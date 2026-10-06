//! Decode-side limits ([`DecodeOptions`]) of the standalone API, plus
//! the deprecated pre-contract [`DecodeLimits`] record.
//!
//! The encode-side knobs ([`crate::EncodeOptions`]) live next to the
//! encoder in [`crate::encoder`].

use crate::error::{IcerError, Result};

/// Limits and strictness for [`crate::decode_with`] /
/// [`crate::decode_all_with`] (and the `_with` forms of the depth
/// decoders).
///
/// Every limit is checked against the parsed framing **before** any
/// plane or wavelet-coefficient buffer is allocated, so a hostile
/// header fails with [`IcerError::LimitExceeded`] instead of
/// committing memory or compute (the inverse DWT never runs on a
/// refused geometry). `None` means unlimited.
///
/// # Why pixel caps, not only bytes
///
/// The wire format's 16-bit width / 16-bit height fields admit values
/// up to `65535 × 65535 ≈ 4.29 GPx` per segment: a 12-byte segment
/// header could request a ~4 GB plane plus the matching `i32`
/// coefficient buffer (~16 GB). The defaults are
/// conservative-but-realistic for every published Mars-rover ICER
/// deployment (Pancam / Hazcam 1024×1024 = 1 MPx; Mastcam-Z 1648×1200
/// ≈ 2 MPx; HiRISE strips ≈ 400 MPx run through ICER-3D):
///
/// | field | default | meaning |
/// |---|---|---|
/// | `max_width` / `max_height` | `None` | the 16-bit header fields already bound them to 65535 |
/// | `max_pixels_per_segment` | 64 MPx | one segment's `width × height` (ICER extra) |
/// | `max_pixels` | 256 MPx | the stitched image's `width × height` (summed over row-strip segments; a cube's `width × height × bands`) |
/// | `max_bytes` | 1 GiB | the decoder's planned peak **working set** — decoded planes plus the largest per-segment coefficient buffer and coder state (README "Memory"; `ImageInfo::working_set_bytes`) |
/// | `strict` | `false` | no effect — see below |
///
/// # `strict`
///
/// A documented no-op. ICER's framing has no optional leniency: every
/// structural rule (sync prefix, filter id, level count, geometry,
/// bit-plane count, segment contiguity, container tags) is enforced in
/// both modes, and a stream cut short mid-packet is the format's
/// progressive design (IPN 42-155 §I), not an error. Loss-tolerant
/// decoding of a stream with *missing segments* is the separate
/// [`crate::parse_icer_lenient`] depth API.
#[derive(Clone, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub struct DecodeOptions {
    /// Reject images wider than this (pixels).
    pub max_width: Option<u32>,
    /// Reject images taller than this (pixels).
    pub max_height: Option<u32>,
    /// Reject images with more than this many pixels (`width ×
    /// height`, summed over row-strip segments; `× bands` for a cube).
    pub max_pixels: Option<u64>,
    /// Reject streams whose planned peak working set (decoded planes +
    /// the largest per-segment decode state, see the type docs) would
    /// exceed this many bytes.
    pub max_bytes: Option<u64>,
    /// No effect (see the type docs); exists for contract shape.
    pub strict: bool,
    /// ICER extra: reject any single segment whose `width × height`
    /// exceeds this (the segment is the unit the inverse DWT and
    /// coefficient buffer are sized by).
    pub max_pixels_per_segment: Option<u64>,
}

impl DecodeOptions {
    /// Default [`Self::max_pixels_per_segment`]: 64 MPx.
    pub const DEFAULT_MAX_PIXELS_PER_SEGMENT: u64 = 64 * 1024 * 1024;
    /// Default [`Self::max_pixels`]: 256 MPx.
    pub const DEFAULT_MAX_PIXELS: u64 = 256 * 1024 * 1024;
    /// Default [`Self::max_bytes`]: 1 GiB of planned working set.
    pub const DEFAULT_MAX_BYTES: u64 = 1 << 30;

    /// The defaults (see the type docs).
    pub fn new() -> Self {
        Self::default()
    }

    /// Set (or lift with `None`) the width limit.
    pub fn with_max_width(mut self, max_width: impl Into<Option<u32>>) -> Self {
        self.max_width = max_width.into();
        self
    }

    /// Set (or lift with `None`) the height limit.
    pub fn with_max_height(mut self, max_height: impl Into<Option<u32>>) -> Self {
        self.max_height = max_height.into();
        self
    }

    /// Set (or lift with `None`) the total pixel-count limit.
    pub fn with_max_pixels(mut self, max_pixels: impl Into<Option<u64>>) -> Self {
        self.max_pixels = max_pixels.into();
        self
    }

    /// Set (or lift with `None`) the working-set byte limit.
    pub fn with_max_bytes(mut self, max_bytes: impl Into<Option<u64>>) -> Self {
        self.max_bytes = max_bytes.into();
        self
    }

    /// Set strict mode (a documented no-op for ICER).
    pub fn with_strict(mut self, strict: bool) -> Self {
        self.strict = strict;
        self
    }

    /// Set (or lift with `None`) the per-segment pixel-count limit.
    pub fn with_max_pixels_per_segment(mut self, max: impl Into<Option<u64>>) -> Self {
        self.max_pixels_per_segment = max.into();
        self
    }

    /// Lift every limit (`max_*` all `None`). Use only when the input is
    /// trusted (e.g. produced by this crate's own encoder in a
    /// controlled batch run) — fuzz inputs can then drive ~4 GB / plane.
    pub fn unlimited(mut self) -> Self {
        self.max_width = None;
        self.max_height = None;
        self.max_pixels = None;
        self.max_bytes = None;
        self.max_pixels_per_segment = None;
        self
    }

    /// Check one segment's geometry against
    /// [`Self::max_pixels_per_segment`] (and the width / height caps).
    pub(crate) fn check_segment(&self, segment_index: u16, width: u32, height: u32) -> Result<()> {
        self.check_width_height(width, height)?;
        let px = width as u64 * height as u64;
        if let Some(m) = self.max_pixels_per_segment {
            if px > m {
                return Err(IcerError::limit(format!(
                    "segment {segment_index} geometry {width}x{height} = {px} pixels exceeds \
                     per-segment cap of {m} pixels (see DecodeOptions::max_pixels_per_segment)"
                )));
            }
        }
        Ok(())
    }

    /// Check a width / height pair against [`Self::max_width`] /
    /// [`Self::max_height`].
    pub(crate) fn check_width_height(&self, width: u32, height: u32) -> Result<()> {
        if let Some(m) = self.max_width {
            if width > m {
                return Err(IcerError::limit(format!(
                    "width {width} exceeds max_width {m}"
                )));
            }
        }
        if let Some(m) = self.max_height {
            if height > m {
                return Err(IcerError::limit(format!(
                    "height {height} exceeds max_height {m}"
                )));
            }
        }
        Ok(())
    }

    /// Check a running total pixel count against [`Self::max_pixels`].
    pub(crate) fn check_total_pixels(&self, total: u64, what: &str) -> Result<()> {
        if let Some(m) = self.max_pixels {
            if total > m {
                return Err(IcerError::limit(format!(
                    "{what} pixel-count {total} exceeds total cap of {m} pixels \
                     (see DecodeOptions::max_pixels)"
                )));
            }
        }
        Ok(())
    }

    /// Check a planned peak working set against [`Self::max_bytes`].
    pub(crate) fn check_bytes(&self, bytes: u64) -> Result<()> {
        if let Some(m) = self.max_bytes {
            if bytes > m {
                return Err(IcerError::limit(format!(
                    "planned decoder working set of {bytes} bytes exceeds max_bytes {m} \
                     (see DecodeOptions::max_bytes)"
                )));
            }
        }
        Ok(())
    }
}

impl Default for DecodeOptions {
    fn default() -> Self {
        Self {
            max_width: None,
            max_height: None,
            max_pixels: Some(Self::DEFAULT_MAX_PIXELS),
            max_bytes: Some(Self::DEFAULT_MAX_BYTES),
            strict: false,
            max_pixels_per_segment: Some(Self::DEFAULT_MAX_PIXELS_PER_SEGMENT),
        }
    }
}

/// Pre-contract decode resource caps — superseded by [`DecodeOptions`]
/// (`max_pixels_per_segment` → `max_pixels_per_segment`,
/// `max_total_pixels` → `max_pixels`). Converts with `From`.
#[deprecated(note = "use oxideav_icer::DecodeOptions (IMAGE_CRATE_API)")]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DecodeLimits {
    /// Maximum `width * height` (in pixels) any single segment may
    /// request.
    pub max_pixels_per_segment: u64,
    /// Maximum total `width * height` (in pixels) of the stitched image.
    pub max_total_pixels: u64,
}

#[allow(deprecated)]
impl DecodeLimits {
    /// 64 MPx per segment.
    pub const DEFAULT_MAX_PIXELS_PER_SEGMENT: u64 = DecodeOptions::DEFAULT_MAX_PIXELS_PER_SEGMENT;
    /// 256 MPx total.
    pub const DEFAULT_MAX_TOTAL_PIXELS: u64 = DecodeOptions::DEFAULT_MAX_PIXELS;

    /// No-cap policy (both fields `u64::MAX`).
    pub const fn unlimited() -> Self {
        Self {
            max_pixels_per_segment: u64::MAX,
            max_total_pixels: u64::MAX,
        }
    }
}

#[allow(deprecated)]
impl Default for DecodeLimits {
    fn default() -> Self {
        Self {
            max_pixels_per_segment: Self::DEFAULT_MAX_PIXELS_PER_SEGMENT,
            max_total_pixels: Self::DEFAULT_MAX_TOTAL_PIXELS,
        }
    }
}

#[allow(deprecated)]
impl From<DecodeLimits> for DecodeOptions {
    /// `u64::MAX` (the old "unlimited") becomes `None`; the byte cap is
    /// lifted too so the converted policy is never stricter than the
    /// original pixel-only one.
    fn from(l: DecodeLimits) -> Self {
        let lift = |v: u64| if v == u64::MAX { None } else { Some(v) };
        DecodeOptions::new()
            .with_max_pixels_per_segment(lift(l.max_pixels_per_segment))
            .with_max_pixels(lift(l.max_total_pixels))
            .with_max_bytes(None)
    }
}

#[allow(deprecated)]
impl From<&DecodeLimits> for DecodeOptions {
    fn from(l: &DecodeLimits) -> Self {
        (*l).into()
    }
}

#[cfg(test)]
#[allow(deprecated)]
mod tests {
    use super::*;

    #[test]
    fn defaults_are_finite() {
        let d = DecodeOptions::default();
        assert_eq!(d.max_width, None);
        assert_eq!(d.max_height, None);
        assert_eq!(d.max_pixels, Some(256 * 1024 * 1024));
        assert_eq!(d.max_pixels_per_segment, Some(64 * 1024 * 1024));
        assert_eq!(d.max_bytes, Some(1 << 30));
        assert!(!d.strict);
        let u = d.clone().unlimited();
        assert_eq!(u.max_bytes, None);
        assert_eq!(u.max_pixels, None);
        assert!(DecodeOptions::new()
            .with_max_width(4)
            .check_segment(0, 5, 1)
            .unwrap_err()
            .is_limit_exceeded());
        assert!(DecodeOptions::new()
            .with_max_height(4)
            .check_segment(0, 1, 5)
            .is_err());
        assert!(DecodeOptions::new()
            .with_max_pixels_per_segment(10u64)
            .check_segment(0, 4, 4)
            .is_err());
        assert!(DecodeOptions::new()
            .with_max_pixels(10u64)
            .check_total_pixels(16, "x")
            .is_err());
        assert!(DecodeOptions::new()
            .with_max_bytes(10u64)
            .check_bytes(16)
            .is_err());
        assert!(DecodeOptions::new().with_strict(true).strict);
    }

    #[test]
    fn legacy_limits_convert() {
        let o: DecodeOptions = DecodeLimits::default().into();
        assert_eq!(o.max_pixels_per_segment, Some(64 * 1024 * 1024));
        assert_eq!(o.max_pixels, Some(256 * 1024 * 1024));
        assert_eq!(o.max_bytes, None);
        let u: DecodeOptions = (&DecodeLimits::unlimited()).into();
        assert_eq!(u.max_pixels_per_segment, None);
        assert_eq!(u.max_pixels, None);
    }
}
