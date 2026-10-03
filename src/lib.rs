//! Pure-Rust ICER -- JPL's progressive wavelet image compressor used
//! by every Mars surface mission since the 2003 Mars Exploration
//! Rovers (Spirit, Opportunity), continued on Mars Science Laboratory
//! (Curiosity), Mars 2020 (Perseverance), and follow-ons.
//!
//! Current status:
//!
//! * **Framing parser** -- segment + packet header walk over the
//!   on-the-wire byte stream; both directions (encode + decode).
//!   See [`header::SegmentHeader`] / [`header::PacketHeader`].
//! * **Spec-exact reversible integer wavelet transform** -- the IPN
//!   42-155 §II.A equations (1)-(3) recurrence with the Table 1
//!   parameters for all seven filters (A-F + Q); forward + inverse,
//!   1-D + interleaved + dyadic D-level, bit-exact reversible under
//!   every filter. The encode/decode pipeline runs on it (see
//!   [`wavelet_int`]); [`wavelet`] keeps the pre-spec textbook 5/3
//!   lifting as an internal layout-contract reference.
//! * **Binary arithmetic coder + context model** -- registers, range
//!   renormalisation, follow-bit handling, and the IPN 42-155 §III.C MER
//!   probability estimator (initial counts 2/4, rescale when the total
//!   reaches 500). (See [`arith`] + [`context`].)
//! * **Spec-exact §III.B significance contexts** -- the subband-aware
//!   IPN 42-155 §III.B Table 6 (LL/LH/HL) / Table 7 (HH) context
//!   assignment, with the HL context-template transpose; the bit-plane
//!   scanner resolves each coefficient's subband via
//!   [`priority::classify_position`]. Sign contexts follow §III.B Table 8
//!   (with the HL sign transpose).
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
//! * **ROI segment prioritisation** --
//!   `with_segment_priorities(Vec<u16>)` attaches a per-segment priority
//!   permutation; `with_center_roi()` builds a centre-out priority
//!   permutation for the current `segment_count`. Combined with
//!   `with_byte_budget`, high-priority strips keep full fidelity while
//!   low-priority strips can be dropped to zero-body placeholders. The
//!   decoder reconstructs dropped strips as flat 128 (IPN 42-155
//!   §III.E independent-segment scheduling).
//! * **Geometry-preserving budget truncation** -- on *every* budget
//!   path (with or without ROI priorities), a strip that does not fit
//!   the byte budget is emitted as a zero-body placeholder header rather
//!   than dropped, so a budget-truncated stream always frames the full
//!   image geometry (IPN 42-155 §V.B). `DecodeLimits` additionally
//!   bounds decode *compute*, not just allocation: a header declaring a
//!   geometry over the cap is refused before any inverse DWT runs.
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
//! ## Standalone use (`IMAGE_CRATE_API`)
//!
//! The crate exposes the OxideAV image-crate contract at its root —
//! [`probe`], [`info`], [`decode`] / [`decode_with`] / [`decode_rgb8`] /
//! [`decode_rgba8`] / [`decode_all`] / [`decode_from`], [`encode`] /
//! [`encode_rgb8`] / [`encode_rgba8`] / [`encode_to`] / [`encode_all`] —
//! around [`IcerImage`] (native layout, one [`Plane`] per component,
//! [`ColorInfo`], [`Metadata`], the significant `bit_depth`),
//! [`RgbImage`] / [`RgbaImage`], [`ImageInfo`], [`Frame`],
//! [`DecodeOptions`], [`EncodeOptions`], [`PixelFormat`] (=
//! [`IcerPixelFormat`], names mirroring `oxideav_core::PixelFormat`) and
//! [`Error`] (= [`IcerError`]). It builds with `default-features = false`
//! and no `oxideav-core`.
//!
//! ```
//! let rgb = vec![0u8; 4 * 3 * 3];
//! let bytes = oxideav_icer::encode_rgb8(4, 3, &rgb, &Default::default())?;
//! assert!(oxideav_icer::probe(&bytes));
//! let info = oxideav_icer::info(&bytes)?;                 // framing only
//! assert_eq!((info.width, info.height, info.frames), (4, 3, 1));
//! let img = oxideav_icer::decode(&bytes)?;                // IcerImage, native Gbrp8
//! let rgba: Vec<u8> = img.to_rgba8();                     // 4 × width bytes per row
//! let opts = oxideav_icer::EncodeOptions::compressed().with_segment_count(1);
//! let out = oxideav_icer::encode_rgba8(img.width(), img.height(), &rgba, &opts)?;
//! assert_eq!(oxideav_icer::decode_rgb8(&out)?.data, rgb);  // lossless
//! # Ok::<(), oxideav_icer::Error>(())
//! ```
//!
//! Native layouts: `Gray8` (bare segment stream), `Gray16Le` (exact
//! `9..=16`-bit samples in little-endian words behind the deep plane
//! container), `Yuv444P` and `Gbrp8` (three independent component
//! streams). An ICER-3D cube decodes band by band through
//! [`decode_all`]. The depth APIs — the segment walker, the wavelet /
//! context / entropy stages, [`parse_icer_lenient`], the cube pipeline
//! ([`encode_icer3d`] / [`parse_icer3d`]), the analysis helpers — keep
//! their names.
//!
//! ## Framework use
//!
//! The default `registry` Cargo feature pulls in `oxideav-core` and
//! exposes `register` (`RuntimeContext`), `register_codecs` /
//! `register_containers`, the `make_decoder` / `make_encoder`
//! factories, and the frame bridge (`From<IcerImage> for VideoFrame`,
//! `IcerImage::from_video_frame`). The trait-side `Decoder` / `Encoder`
//! are thin adapters over the standalone functions.

#![cfg_attr(not(feature = "registry"), allow(dead_code))]
#![allow(clippy::needless_range_loop)]
#![allow(clippy::manual_div_ceil)]

pub mod analyze;
pub mod api;
pub mod arith;
pub mod bitplane;
pub mod bitplane3d;
pub mod context;
pub mod context3d;
pub mod cube;
pub mod decoder;
pub mod encoder;
pub mod entropy;
pub mod error;
pub mod header;
pub mod image;
pub mod ixec;
pub mod options;
pub mod partition;
pub mod plane_container;
pub mod priority;
#[cfg(feature = "registry")]
pub mod registry;
pub mod subband3d;
pub mod wavelet;
pub mod wavelet3d;
pub mod wavelet_int;

/// Codec identifier string used by the registry + by container demuxers
/// when emitting `CodecId::new(CODEC_ID_STR)`. Kept lowercase to match
/// the convention used by every other codec in the workspace.
pub const CODEC_ID_STR: &str = "icer";

// ---- the contract surface (IMAGE_CRATE_API) ---------------------------------
pub use api::{
    decode, decode_all, decode_all_with, decode_from, decode_rgb8, decode_rgba8, decode_with,
    encode, encode_all, encode_rgb8, encode_rgba8, encode_to, info, info_with, probe,
};
pub use encoder::EncodeOptions;
pub use error::{Error, IcerError, Result};
pub use image::{
    ColorInfo, ColorRange, Frame, IcerImage, IcerPixelFormat, ImageInfo, Metadata, PixelFormat,
    Plane, RgbImage, RgbaImage, StreamKind,
};
pub use options::DecodeOptions;

// ---- depth APIs (keep their names) -------------------------------------------
pub use analyze::{
    analyze, encode_to_quality_target, pick_filter_by_rate_distortion, psnr_db,
    quality_search_bounds, recommend_filter, recommend_segment_count, region_mae, ssim,
    supported_for_analysis, ChannelReliability, DistortionReport, ImageStats,
    DEFAULT_RD_CANDIDATES,
};
pub use bitplane::{
    decode_bitplanes_filtered, encode_bitplanes_filtered, EncodedPacket, ScanFilter,
};
pub use context::{
    neighbour_counts, sign_context_subband, sign_prediction_flip_subband,
    significance_context_subband, significance_context_table6, significance_context_table7,
};
pub use cube::{
    encode_icer3d, is_cube, parse_icer3d, parse_icer3d_lenient, parse_icer3d_lenient_with,
    parse_icer3d_with, CubeEncodeOptions, IcerCube, LenientCubeDecode,
};
pub use decoder::{
    decode_uncompressed_icer, parse_icer_lenient, parse_icer_lenient_with, LenientDecode,
    SegmentMetadata,
};
pub use entropy::EntropyKind;
pub use header::{
    walk_segment, BitPlanePass, PacketHeader, SegmentHeader, WalkedPacket, WalkedSegment,
    WaveletFilter,
};
pub use ixec::{
    bin_for_probability, bins, Bin, ComponentCode, InterleavedDecoder, InterleavedEncoder,
    IxecDecoder, IxecEncoder, BUFFER_WORDS,
};
pub use partition::{
    coefficient_segment_map, ll_dimensions, ll_segment_map, partition, partition_params,
    PartitionParams, SegmentRect,
};
pub use plane_container::{is_container, parse_container, ParsedContainer};
pub use priority::{
    classify_position, encode_order, subband_weight_map, subbands, Subband, SubbandBitPlane,
    SubbandType,
};
pub use wavelet3d::{
    coefficient_word_bits, dynamic_range_expansion, forward_3d, high_pass_gamma, inverse_3d,
    spatial_stage_count, spectral_stage_count,
};
pub use wavelet_int::{
    abs_tap_sum, approx_max_input_range, max_input_range, word_bits_for_input_range,
};

// ---- deprecated pre-contract entry points (one release) ----------------------
#[allow(deprecated)]
pub use cube::{parse_icer3d_lenient_with_limits, parse_icer3d_with_limits};
#[allow(deprecated)]
pub use decoder::{
    parse_icer, parse_icer_lenient_with_limits, parse_icer_metadata,
    parse_icer_metadata_with_limits, parse_icer_with_limits, IcerMetadata,
};
#[allow(deprecated)]
pub use encoder::encode_icer;
#[allow(deprecated)]
pub use image::IcerPlane;
#[allow(deprecated)]
pub use options::DecodeLimits;

// ---- registry-gated public surface -------------------------------------------
#[cfg(feature = "registry")]
#[doc(hidden)]
pub use registry::__oxideav_entry;
#[cfg(feature = "registry")]
pub use registry::{
    from_core_pixel_format, make_decoder, make_encoder, register, register_codecs,
    register_containers, to_core_pixel_format, IcerDecoder, IcerEncoder,
};
