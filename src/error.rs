//! Crate-local error type — std-primitives only so the standalone
//! (no `registry`) build never depends on `oxideav-core`.
//!
//! See the `registry` module for the (optional) `From<IcerError> for
//! oxideav_core::Error` bridge.

use core::fmt;

/// Result alias used by every public entry point in the crate.
pub type Result<T, E = IcerError> = core::result::Result<T, E>;

/// The contract name for [`IcerError`] (`IMAGE_CRATE_API`).
pub type Error = IcerError;

/// Errors produced by ICER bitstream parsing / wavelet inverse / entropy
/// decode, and by the encoder.
///
/// The variants mirror the subset of `oxideav_core::Error` the codec
/// can hit (the `registry` feature maps them 1:1). The enum deliberately
/// does not derive `Clone` / `PartialEq` (it carries a
/// [`std::io::Error`]); tests match on the variant or on `Display`.
#[derive(Debug)]
#[non_exhaustive]
pub enum IcerError {
    /// The bitstream violates a syntactic rule (bad magic, reserved
    /// bit set, header field out of range, etc.), or a caller-assembled
    /// image has inconsistent geometry.
    InvalidData(String),

    /// The bitstream is syntactically valid but uses a feature this
    /// crate does not implement, or the encoder was asked for an
    /// option combination / layout ICER cannot represent.
    Unsupported(String),

    /// A [`crate::DecodeOptions`] limit (`max_width` / `max_height` /
    /// `max_pixels` / `max_pixels_per_segment` / `max_bytes`) would be
    /// exceeded; raised from the framing, before any plane or
    /// coefficient buffer is allocated.
    LimitExceeded(String),

    /// An I/O error from [`crate::decode_from`] / [`crate::encode_to`].
    Io(std::io::Error),

    /// The buffer ends before the next required field could be read.
    Truncated,
}

impl fmt::Display for IcerError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            IcerError::InvalidData(s) => write!(f, "icer: invalid data: {s}"),
            IcerError::Unsupported(s) => write!(f, "icer: unsupported: {s}"),
            IcerError::LimitExceeded(s) => write!(f, "icer: limit exceeded: {s}"),
            IcerError::Io(e) => write!(f, "icer: i/o error: {e}"),
            IcerError::Truncated => write!(f, "icer: truncated input"),
        }
    }
}

impl std::error::Error for IcerError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            IcerError::Io(e) => Some(e),
            _ => None,
        }
    }
}

impl From<std::io::Error> for IcerError {
    fn from(e: std::io::Error) -> Self {
        IcerError::Io(e)
    }
}

impl IcerError {
    /// Construct an [`IcerError::InvalidData`] from a stringy message.
    pub fn invalid<S: Into<String>>(s: S) -> Self {
        IcerError::InvalidData(s.into())
    }
    /// Construct an [`IcerError::Unsupported`] from a stringy message.
    pub fn unsupported<S: Into<String>>(s: S) -> Self {
        IcerError::Unsupported(s.into())
    }
    /// Construct an [`IcerError::LimitExceeded`] from a stringy message.
    pub fn limit<S: Into<String>>(s: S) -> Self {
        IcerError::LimitExceeded(s.into())
    }

    /// `true` for [`IcerError::InvalidData`].
    pub fn is_invalid_data(&self) -> bool {
        matches!(self, Self::InvalidData(_))
    }
    /// `true` for [`IcerError::Unsupported`].
    pub fn is_unsupported(&self) -> bool {
        matches!(self, Self::Unsupported(_))
    }
    /// `true` for [`IcerError::LimitExceeded`].
    pub fn is_limit_exceeded(&self) -> bool {
        matches!(self, Self::LimitExceeded(_))
    }
    /// `true` for [`IcerError::Io`].
    pub fn is_io(&self) -> bool {
        matches!(self, Self::Io(_))
    }
    /// `true` for [`IcerError::Truncated`].
    pub fn is_truncated(&self) -> bool {
        matches!(self, Self::Truncated)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn display_prefixes_each_variant() {
        assert_eq!(IcerError::invalid("x").to_string(), "icer: invalid data: x");
        assert_eq!(
            IcerError::unsupported("y").to_string(),
            "icer: unsupported: y"
        );
        assert_eq!(IcerError::limit("z").to_string(), "icer: limit exceeded: z");
        assert_eq!(IcerError::Truncated.to_string(), "icer: truncated input");
        let io: IcerError = std::io::Error::other("boom").into();
        assert!(io.is_io());
        assert!(io.to_string().starts_with("icer: i/o error: "));
        assert!(std::error::Error::source(&io).is_some());
        assert!(std::error::Error::source(&IcerError::Truncated).is_none());
    }
}
