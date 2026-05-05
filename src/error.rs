//! Crate-local error type — std-primitives only so the standalone
//! (no `registry`) build never depends on `oxideav-core`.
//!
//! See [`crate::registry`] for the (optional) `From<IcerError> for
//! oxideav_core::Error` bridge.

use core::fmt;

/// Result alias used by every public entry point in the crate.
pub type Result<T, E = IcerError> = core::result::Result<T, E>;

/// Errors produced by ICER bitstream parsing / wavelet inverse / entropy
/// decode.
///
/// Variants intentionally mirror `oxideav_core::Error::{InvalidData,
/// Unsupported, Truncated}` (registry-feature crate maps them 1:1).
/// Keeping a local copy lets the standalone build remain
/// dependency-free.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum IcerError {
    /// The bitstream violates a syntactic rule (bad magic, reserved
    /// bit set, header field out of range, etc.).
    InvalidData(String),

    /// The bitstream is syntactically valid but uses a feature this
    /// crate does not yet implement (alternative wavelet filter,
    /// segment count > supported, etc.). Round 1 returns this for
    /// most non-5/3 paths.
    Unsupported(String),

    /// The buffer ends before the next required field could be read.
    Truncated,
}

impl fmt::Display for IcerError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            IcerError::InvalidData(s) => write!(f, "icer: invalid data: {s}"),
            IcerError::Unsupported(s) => write!(f, "icer: unsupported: {s}"),
            IcerError::Truncated => write!(f, "icer: truncated input"),
        }
    }
}

impl std::error::Error for IcerError {}

impl IcerError {
    pub fn invalid<S: Into<String>>(s: S) -> Self {
        IcerError::InvalidData(s.into())
    }
    pub fn unsupported<S: Into<String>>(s: S) -> Self {
        IcerError::Unsupported(s.into())
    }
}
