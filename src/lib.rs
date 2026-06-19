//! Pure-Rust ICER -- JPL's progressive wavelet image compressor used
//! by every Mars surface mission since the 2003 Mars Exploration
//! Rovers (Spirit, Opportunity), continued on Mars Science Laboratory
//! (Curiosity), Mars 2020 (Perseverance), and follow-ons.
//!
//! Round-6 status:
//!
//! * **Framing parser** -- segment + packet header walk over the
//!   on-the-wire byte stream; both directions (encode + decode).
//!   See [`header::SegmentHeader`] / [`header::PacketHeader`].
//! * **Integer 5/3 wavelet transform** -- forward + inverse, 1-D + 2-D
//!   one-level + dyadic D-level. Round-trip-bit-exact on signed i32
//!   coefficients. (See [`wavelet`].)
//! * **Float wavelet filters A-G** -- round-trip to IEEE-754 tolerance
//!   on the 1-D + 2-D + dyadic paths; integer/float dispatch helper
//!   in [`wavelet_float::forward_2d`] / [`wavelet_float::inverse_2d`].
//!   Filter G (Le Gall 5/3 float variant) completes the full A-G set.
//! * **Binary arithmetic coder + context model** -- registers, range
//!   renormalisation, follow-bit handling, adaptive Laplace-windowed
//!   probability estimator. (See [`arith`] + [`context`].)
//! * **Real JPL significance context table** -- the IPN 42-155 §III.B
//!   H/V/D neighbour-count classification (9 contexts) replaces the
//!   round-2 popcount placeholder. Sign-flip convention per §III.B.
//! * **Stripe-ordered scan** -- bit-plane scanner processes coefficients
//!   in horizontal stripes of height `STRIPE_HEIGHT` (4 rows, IPN
//!   42-155 §III.B) to maximise context-pattern locality.
//! * **Multi-packet ordering** -- one packet pair (significance + sign,
//!   then refinement) per bit-plane per IPN 42-155 §IV. Truncated
//!   streams reconstruct at lower quality.
//! * **Decoder** -- full pixel reconstruction for both the
//!   IPN 42-155 §III.D "uncompressed" path *and* the compressed
//!   wavelet + bit-plane path. Multi-segment images stitched in
//!   `segment_index` order.
//! * **Encoder** -- emits the uncompressed path *and* the compressed
//!   path (`EncodeOptions::compressed()`). Multi-segment encode via
//!   `EncodeOptions::segment_count`. Quota-controlled output via
//!   `with_byte_budget` / `with_target_bytes`. Self-roundtrips
//!   byte-exactly on filter Q.
//! * **Automatic filter selection** (round 5) -- `with_auto_filter()`
//!   runs a one-pass [`analyze::ImageStats`] scan and picks a filter
//!   via the [`analyze::recommend_filter`] decision tree;
//!   `with_auto_filter_rd()` trials every candidate filter and picks
//!   the byte-smallest output via [`analyze::pick_filter_by_rate_distortion`].
//!   Both compose with the quota options.
//! * **ROI segment prioritisation** (round 6) --
//!   `with_segment_priorities(Vec<u16>)` attaches a per-segment priority
//!   permutation; `with_center_roi()` builds a centre-out priority
//!   permutation for the current `segment_count`. Combined with
//!   `with_byte_budget`, high-priority strips keep full fidelity while
//!   low-priority strips can be dropped to zero-body placeholders. The
//!   decoder reconstructs dropped strips as flat 128 (IPN 42-155
//!   §III.E independent-segment scheduling).
//!
//! ## Specification source
//!
//! Every section / equation reference in this crate cites Kiely &
//! Klimesh, "The ICER Progressive Wavelet Image Compressor" — Jet
//! Propulsion Laboratory, IPN Progress Report 42-155 (2003) — abbreviated
//! "IPN 42-155" throughout; that paper is the sole specification
//! source for this crate's clean-room write. Where the paper defers to
//! "the implementation" (e.g. literal sync prefix value, exact
//! context-pattern tables) the choice is documented + flagged as an
//! interop risk for round 2.
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

pub mod analyze;
pub mod arith;
pub mod bitplane;
pub mod context;
pub mod decoder;
pub mod encoder;
pub mod error;
pub mod header;
pub mod image;
pub mod plane_container;
pub mod priority;
#[cfg(feature = "registry")]
pub mod registry;
pub mod wavelet;
pub mod wavelet_float;
pub mod wavelet_int;

/// Codec identifier string used by the registry + by container demuxers
/// when emitting `CodecId::new(CODEC_ID_STR)`. Kept lowercase to match
/// the convention used by every other codec in the workspace.
pub const CODEC_ID_STR: &str = "icer";

// Standalone public surface — works whether or not `registry` is on.
pub use analyze::{
    analyze, encode_to_quality_target, pick_filter_by_rate_distortion, psnr_db,
    quality_search_bounds, recommend_filter, region_mae, ssim, supported_for_analysis,
    DistortionReport, ImageStats, DEFAULT_RD_CANDIDATES,
};
pub use bitplane::EncodedPacket;
pub use decoder::{
    decode_uncompressed_icer, parse_icer, parse_icer_lenient, parse_icer_lenient_with_limits,
    parse_icer_metadata, parse_icer_metadata_with_limits, parse_icer_with_limits, DecodeLimits,
    IcerMetadata, LenientDecode, SegmentMetadata,
};
pub use encoder::{encode_icer, EncodeOptions};
pub use error::{IcerError, Result};
pub use header::{
    walk_segment, BitPlanePass, PacketHeader, SegmentHeader, WalkedPacket, WalkedSegment,
    WaveletFilter,
};
pub use image::{IcerImage, IcerPixelFormat, IcerPlane};
pub use plane_container::{is_container, parse_container, ParsedContainer};
pub use priority::{
    classify_position, encode_order, subband_weight_map, subbands, Subband, SubbandBitPlane,
    SubbandType,
};

// Registry-gated public surface.
#[cfg(feature = "registry")]
pub use registry::{
    __oxideav_entry, register, register_codecs, register_containers, IcerDecoder, IcerEncoder,
};
