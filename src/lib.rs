//! Pure-Rust ICER — JPL's progressive wavelet image compressor used
//! by every Mars surface mission since the 2003 Mars Exploration
//! Rovers (Spirit, Opportunity), continued on Mars Science Laboratory
//! (Curiosity), Mars 2020 (Perseverance), and follow-ons.
//!
//! Round-1 status:
//!
//! * **Framing parser** — segment + packet header walk over the
//!   on-the-wire byte stream; both directions (encode + decode).
//!   See [`header::SegmentHeader`] / [`header::PacketHeader`].
//! * **Integer 5/3 wavelet transform** — forward + inverse, 1-D + 2-D
//!   one-level + dyadic D-level. Round-trip-bit-exact on signed i32
//!   coefficients. Filters A, B, C, D, E, F (the float / non-5/3
//!   alternatives in IPN 42-155 §III.A) parse but do not yet
//!   transform.
//! * **Binary arithmetic coder + context model** — registers, range
//!   renormalisation, follow-bit handling, adaptive Laplace-windowed
//!   probability estimator. Round-trips through itself but not yet
//!   wired to the bit-plane scanner.
//! * **Decoder** — full pixel reconstruction for the IPN 42-155 §III.D
//!   "uncompressed" path (single segment); compressed segments parse
//!   their header but reconstruct as zeros (round 2).
//! * **Encoder** — emits the uncompressed path; lossless 5/3
//!   compressed encoding is round 2.
//!
//! ## Specification source
//!
//! Every section / equation reference in this crate cites Kiely &
//! Klimesh, "The ICER Progressive Wavelet Image Compressor" — Jet
//! Propulsion Laboratory, IPN Progress Report 42-155 (2003) — abbreviated
//! "IPN 42-155" throughout. Public ICER reference implementations
//! (JPL DSN ground software, MSL flight code, qccPack, third-party
//! GitHub re-implementations) were **not** consulted during the
//! clean-room write. Where the paper defers to "the implementation"
//! (e.g. literal sync prefix value, exact context-pattern tables) the
//! choice is documented + flagged as an interop risk for round 2.
//!
//! ## Standalone vs registry-integrated
//!
//! With the default `registry` Cargo feature on, the crate exposes
//! [`oxideav_core::Decoder`] / [`oxideav_core::Encoder`] trait impls
//! plus a [`registry::register`] entry point against `oxideav-core`.
//! With the feature off the crate ships only the standalone
//! [`parse_icer`] / [`parse_icer_metadata`] / [`encode_icer`] API
//! plus the local [`IcerImage`] / [`IcerError`] types, with no
//! `oxideav-core` dep in the tree. Image-library consumers should
//! depend on `oxideav-icer` with `default-features = false`.

#![cfg_attr(not(feature = "registry"), allow(dead_code))]
#![allow(clippy::needless_range_loop)]
#![allow(clippy::manual_div_ceil)]

pub mod arith;
pub mod context;
pub mod decoder;
pub mod encoder;
pub mod error;
pub mod header;
pub mod image;
#[cfg(feature = "registry")]
pub mod registry;
pub mod wavelet;

/// Codec identifier string used by the registry + by container demuxers
/// when emitting `CodecId::new(CODEC_ID_STR)`. Kept lowercase to match
/// the convention used by every other codec in the workspace.
pub const CODEC_ID_STR: &str = "icer";

// Standalone public surface — works whether or not `registry` is on.
pub use decoder::{
    decode_uncompressed_icer, parse_icer, parse_icer_metadata, IcerMetadata, SegmentMetadata,
};
pub use encoder::{encode_icer, EncodeOptions};
pub use error::{IcerError, Result};
pub use header::{
    walk_segment, BitPlanePass, PacketHeader, SegmentHeader, WalkedPacket, WalkedSegment,
    WaveletFilter,
};
pub use image::{IcerImage, IcerPixelFormat, IcerPlane};

// Registry-gated public surface.
#[cfg(feature = "registry")]
pub use registry::{register, register_codecs, register_containers, IcerDecoder, IcerEncoder};
