//! `oxideav-core` integration — `Decoder` / `Encoder` trait impls,
//! the frame bridge, the pixel-format mapping, the `Error` conversion
//! and the [`register`] entry point.
//!
//! Gated behind the default-on `registry` Cargo feature. Without
//! `registry` the rest of the crate still exposes the standalone
//! contract API ([`crate::decode`] / [`crate::encode`] / …) plus the
//! crate-local [`IcerImage`] / [`IcerError`] types — none of which
//! depend on `oxideav-core`. The framework `Decoder` / `Encoder` here
//! are thin adapters over those standalone functions (one
//! implementation).
//!
//! # What the framework frame carries
//!
//! * the native layout's planes, mapped 1:1 by name
//!   ([`to_core_pixel_format`]) — except that an exact 10- / 12-bit
//!   `Gray16Le` image is labelled with core's dedicated `Gray10Le` /
//!   `Gray12Le` rung (the samples are the same LSB-aligned words);
//! * the per-plane **significant-bits** side-channel whenever
//!   `bit_depth` is not implied by the core format (a 9-, 11-, 13-,
//!   14- or 15-bit `Gray16Le`, or a shallow cube band in `Gray8`);
//! * **no colour signal**: ICER defines no colour semantics and the
//!   stream carries none, so the crate's documented `ColorInfo`
//!   convention stays on the standalone image and is not stamped on
//!   the frame (wave-3 ruling).

use std::collections::VecDeque;

use oxideav_core::{
    frame::VideoPlane, CodecCapabilities, CodecId, CodecInfo, CodecParameters, CodecRegistry,
    ContainerRegistry, Decoder, Encoder, Error, Frame, Packet, PixelFormat, Result, RuntimeContext,
    TimeBase, VideoFrame,
};

use crate::api::decode_all_with;
use crate::encoder::{encode_image, EncodeOptions};
use crate::error::IcerError;
use crate::image::{IcerImage, IcerPixelFormat, Plane};
use crate::options::DecodeOptions;
use crate::CODEC_ID_STR;

impl From<IcerError> for Error {
    fn from(e: IcerError) -> Self {
        match e {
            IcerError::InvalidData(s) => Error::InvalidData(s),
            IcerError::Unsupported(s) => Error::Unsupported(s),
            IcerError::LimitExceeded(s) => Error::ResourceExhausted(s),
            IcerError::Io(e) => Error::Io(e),
            IcerError::Truncated => Error::InvalidData("icer: truncated input".into()),
        }
    }
}

// ---- pixel formats ----------------------------------------------------------

/// The framework format for a native layout + significant bit depth:
/// a 1:1 name match, except that `Gray16Le` at exactly 10 / 12 bits is
/// labelled `Gray10Le` / `Gray12Le` (core's dedicated rungs for the
/// same LSB-aligned 16-bit words). Every other depth rides the named
/// format and is described by the frame's significant-bits
/// side-channel (see [`image_into_video_frame`]).
pub fn to_core_pixel_format(f: IcerPixelFormat, bit_depth: u8) -> PixelFormat {
    match (f, bit_depth) {
        (IcerPixelFormat::Gray8, _) => PixelFormat::Gray8,
        (IcerPixelFormat::Gray16Le, 10) => PixelFormat::Gray10Le,
        (IcerPixelFormat::Gray16Le, 12) => PixelFormat::Gray12Le,
        (IcerPixelFormat::Gray16Le, _) => PixelFormat::Gray16Le,
        (IcerPixelFormat::Yuv444P, _) => PixelFormat::Yuv444P,
        (IcerPixelFormat::Gbrp8, _) => PixelFormat::Gbrp8,
    }
}

/// The inverse of [`to_core_pixel_format`]: the native layout and the
/// bit depth the core format implies (`Gray10Le` → `Gray16Le` at 10
/// bits, `Gray16Le` → 16, byte layouts → 8). `Err(Unsupported)` for a
/// layout ICER cannot carry.
pub fn from_core_pixel_format(f: PixelFormat) -> crate::Result<(IcerPixelFormat, u8)> {
    Ok(match f {
        PixelFormat::Gray8 => (IcerPixelFormat::Gray8, 8),
        PixelFormat::Gray10Le => (IcerPixelFormat::Gray16Le, 10),
        PixelFormat::Gray12Le => (IcerPixelFormat::Gray16Le, 12),
        PixelFormat::Gray16Le => (IcerPixelFormat::Gray16Le, 16),
        PixelFormat::Yuv444P => (IcerPixelFormat::Yuv444P, 8),
        PixelFormat::Gbrp8 => (IcerPixelFormat::Gbrp8, 8),
        other => {
            return Err(IcerError::unsupported(format!(
                "icer: pixel format {other:?} is not an ICER layout \
                 (Gray8 / Gray10Le / Gray12Le / Gray16Le / Yuv444P / Gbrp8)"
            )))
        }
    })
}

impl From<IcerPixelFormat> for PixelFormat {
    /// [`to_core_pixel_format`] at the layout's natural bit depth.
    fn from(p: IcerPixelFormat) -> Self {
        to_core_pixel_format(p, p.natural_bit_depth())
    }
}

impl TryFrom<PixelFormat> for IcerPixelFormat {
    type Error = IcerError;
    fn try_from(f: PixelFormat) -> crate::Result<Self> {
        from_core_pixel_format(f).map(|(p, _)| p)
    }
}

// ---- frame bridge -----------------------------------------------------------

/// Move an image's planes into a framework frame with the given `pts`,
/// attaching the significant-bits side-channel when `bit_depth` is not
/// what [`to_core_pixel_format`] already implies. No colour signal is
/// stamped (see the module docs).
pub(crate) fn image_into_video_frame(mut image: IcerImage, pts: Option<i64>) -> VideoFrame {
    let core_fmt = to_core_pixel_format(image.format, image.bit_depth);
    let implied = match core_fmt {
        PixelFormat::Gray10Le => 10,
        PixelFormat::Gray12Le => 12,
        PixelFormat::Gray16Le => 16,
        _ => 8,
    };
    let planes: Vec<VideoPlane> = std::mem::take(&mut image.planes)
        .into_iter()
        .map(|p| VideoPlane {
            stride: p.stride,
            data: p.data,
        })
        .collect();
    let n = planes.len();
    let mut frame = VideoFrame { pts, planes };
    if image.bit_depth != implied {
        frame.set_significant_bits(vec![image.bit_depth; n]);
    }
    frame
}

impl From<IcerImage> for VideoFrame {
    /// The planes (+ significant bits when needed), `pts` `None`.
    fn from(image: IcerImage) -> Self {
        image_into_video_frame(image, None)
    }
}

impl From<&IcerImage> for VideoFrame {
    fn from(image: &IcerImage) -> Self {
        image_into_video_frame(image.clone(), None)
    }
}

impl From<IcerImage> for Frame {
    /// `Frame::Video` of [`From<IcerImage> for VideoFrame`].
    fn from(img: IcerImage) -> Self {
        Frame::Video(VideoFrame::from(img))
    }
}

impl IcerImage {
    /// Rebuild an image from a framework frame and the stream parameters
    /// that describe it: `width` and `height` are required
    /// ([`IcerError::InvalidData`] when missing); `pixel_format` selects
    /// the layout and implied depth through [`from_core_pixel_format`]
    /// ([`IcerError::Unsupported`] for a layout ICER cannot carry) — when
    /// it is `None`, a one-plane frame is `Gray8` and a three-plane frame
    /// `Yuv444P` (the historical adapter rule). A significant-bits
    /// side-channel on the frame overrides the implied depth when it is
    /// in the layout's range. The frame's image planes become the pixel
    /// planes (geometry validated by [`IcerImage::new`]). Colour is the
    /// crate's documented convention — the stream cannot carry the
    /// frame's colour signal.
    pub fn from_video_frame(frame: &VideoFrame, params: &CodecParameters) -> crate::Result<Self> {
        let width = params
            .width
            .ok_or_else(|| IcerError::invalid("icer: width missing in CodecParameters"))?;
        let height = params
            .height
            .ok_or_else(|| IcerError::invalid("icer: height missing in CodecParameters"))?;
        let image_planes = frame.image_planes();
        let (format, implied_depth) = match params.pixel_format {
            Some(f) => from_core_pixel_format(f)?,
            None => match image_planes.len() {
                1 => (IcerPixelFormat::Gray8, 8),
                3 => (IcerPixelFormat::Yuv444P, 8),
                n => {
                    return Err(IcerError::unsupported(format!(
                        "icer: {n}-plane frame without a pixel_format (1 = Gray8, 3 = Yuv444P)"
                    )))
                }
            },
        };
        let planes = image_planes
            .iter()
            .map(|p| Plane::new(p.stride, p.data.clone()))
            .collect();
        let img = IcerImage::new(width, height, format, planes)?;
        let depth = match frame.plane_significant_bits(0) {
            Some(b) if format.bit_depth_range().contains(&b) => b,
            _ => implied_depth,
        };
        img.with_bit_depth(depth)
    }
}

impl TryFrom<(&VideoFrame, &CodecParameters)> for IcerImage {
    type Error = IcerError;
    fn try_from((frame, params): (&VideoFrame, &CodecParameters)) -> crate::Result<Self> {
        IcerImage::from_video_frame(frame, params)
    }
}

// ---- registration -----------------------------------------------------------

/// Register the ICER decoder + encoder factories.
pub fn register_codecs(reg: &mut CodecRegistry) {
    let caps = CodecCapabilities::video(CODEC_ID_STR)
        .with_lossy(false)
        .with_intra_only(true)
        .with_max_size(65535, 65535)
        .with_pixel_formats(vec![
            PixelFormat::Gray8,
            PixelFormat::Gray10Le,
            PixelFormat::Gray12Le,
            PixelFormat::Gray16Le,
            PixelFormat::Yuv444P,
            PixelFormat::Gbrp8,
        ]);
    reg.register(
        CodecInfo::new(CodecId::new(CODEC_ID_STR))
            .capabilities(caps)
            .decoder(make_decoder)
            .encoder(make_encoder),
    );
}

/// Unified registration entry point: install both the ICER codec
/// factories and the `.icer` extension hint into a [`RuntimeContext`].
///
/// This is the preferred entry point for new code — it matches the
/// convention every sibling crate follows. Direct callers that only
/// need one of the two sub-registries can keep using
/// [`register_codecs`] / [`register_containers`].
pub fn register(ctx: &mut RuntimeContext) {
    register_codecs(&mut ctx.codecs);
    register_containers(&mut ctx.containers);
}

oxideav_core::register!("icer", register);

/// Register the `.icer` file extension so the container registry can
/// resolve the codec identifier from a filename hint.
///
/// ICER has no separate container format — the on-the-wire byte stream
/// (see [`crate::header::SegmentHeader`]) is also the file format — so
/// only the extension hook is wired up here. No demuxer or muxer is
/// registered.
pub fn register_containers(reg: &mut ContainerRegistry) {
    reg.register_extension("icer", "icer");
}

/// Decoder factory registered with the codec registry. One packet per
/// whole ICER stream; one frame per packet for a 2-D stream, one frame
/// per spectral band for an ICER-3D cube (the same walk as
/// [`crate::decode_all`]).
pub fn make_decoder(params: &CodecParameters) -> Result<Box<dyn Decoder>> {
    Ok(Box::new(IcerDecoder::new(params.codec_id.clone())))
}

/// Encoder factory registered with the codec registry. `params` must
/// carry `width` and `height`; `pixel_format` selects the layout (see
/// [`IcerImage::from_video_frame`]). Encodes with
/// [`EncodeOptions::default`] until [`IcerEncoder::set_options`].
pub fn make_encoder(params: &CodecParameters) -> Result<Box<dyn Encoder>> {
    Ok(Box::new(IcerEncoder::new_from_params(params)))
}

/// Framework decoder — a thin adapter over [`crate::decode_all_with`].
pub struct IcerDecoder {
    codec_id: CodecId,
    options: DecodeOptions,
    pending: VecDeque<VideoFrame>,
    eof: bool,
}

impl IcerDecoder {
    /// A decoder for `codec_id` with [`DecodeOptions::default`].
    pub fn new(codec_id: CodecId) -> Self {
        Self {
            codec_id,
            options: DecodeOptions::default(),
            pending: VecDeque::new(),
            eof: false,
        }
    }

    /// Replace the decode caps applied to every packet.
    pub fn set_options(&mut self, options: DecodeOptions) {
        self.options = options;
    }
}

impl Decoder for IcerDecoder {
    fn codec_id(&self) -> &CodecId {
        &self.codec_id
    }

    fn send_packet(&mut self, packet: &Packet) -> oxideav_core::Result<()> {
        for frame in decode_all_with(&packet.data, &self.options)? {
            self.pending
                .push_back(image_into_video_frame(frame.image, packet.pts));
        }
        Ok(())
    }

    fn receive_frame(&mut self) -> oxideav_core::Result<Frame> {
        match self.pending.pop_front() {
            Some(f) => Ok(Frame::Video(f)),
            None if self.eof => Err(Error::Eof),
            None => Err(Error::NeedMore),
        }
    }

    fn flush(&mut self) -> oxideav_core::Result<()> {
        self.eof = true;
        Ok(())
    }
}

/// Framework encoder — a thin adapter over [`crate::encode`] via
/// [`IcerImage::from_video_frame`].
pub struct IcerEncoder {
    output_params: CodecParameters,
    opts: EncodeOptions,
    pending: Option<Packet>,
    eof: bool,
}

impl IcerEncoder {
    /// An encoder for `codec_id` whose parameters (width / height /
    /// pixel format) must be filled before the first frame.
    pub fn new(codec_id: CodecId) -> Self {
        Self::new_from_params(&CodecParameters::video(codec_id))
    }

    /// An encoder over a copy of `params`.
    pub fn new_from_params(params: &CodecParameters) -> Self {
        Self {
            output_params: params.clone(),
            opts: EncodeOptions::default(),
            pending: None,
            eof: false,
        }
    }

    /// Replace the [`EncodeOptions`] used for every following frame.
    pub fn set_options(&mut self, opts: EncodeOptions) {
        self.opts = opts;
    }
}

impl Encoder for IcerEncoder {
    fn codec_id(&self) -> &CodecId {
        &self.output_params.codec_id
    }

    fn output_params(&self) -> &CodecParameters {
        &self.output_params
    }

    fn send_frame(&mut self, frame: &Frame) -> oxideav_core::Result<()> {
        let video = match frame {
            Frame::Video(v) => v,
            _ => return Err(Error::invalid("icer encoder: expected Frame::Video")),
        };
        let img = IcerImage::from_video_frame(video, &self.output_params)?;
        let bytes = encode_image(&img, &self.opts)?;
        let mut pkt = Packet::new(0u32, TimeBase::new(1, 1), bytes);
        pkt.pts = video.pts;
        pkt.flags.keyframe = true;
        self.pending = Some(pkt);
        Ok(())
    }

    fn receive_packet(&mut self) -> oxideav_core::Result<Packet> {
        match self.pending.take() {
            Some(p) => Ok(p),
            None if self.eof => Err(Error::Eof),
            None => Err(Error::NeedMore),
        }
    }

    fn flush(&mut self) -> oxideav_core::Result<()> {
        self.eof = true;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::decode;

    #[test]
    fn register_containers_resolves_icer_extension_case_insensitive() {
        let mut reg = ContainerRegistry::new();
        register_containers(&mut reg);
        assert_eq!(reg.container_for_extension("icer"), Some("icer"));
        assert_eq!(reg.container_for_extension("ICER"), Some("icer"));
        assert_eq!(reg.container_for_extension("Icer"), Some("icer"));
        assert_eq!(reg.container_for_extension("png"), None);
    }

    #[test]
    fn declared_deep_pixel_format_selects_deep_encode() {
        // A 1-plane frame whose CodecParameters declare Gray12Le must
        // encode through the deep-sample path (LSB-aligned LE u16
        // words), not be misread as twice-as-wide 8-bit samples.
        let mut params = CodecParameters::video(CodecId::new(CODEC_ID_STR));
        params.width = Some(8);
        params.height = Some(6);
        params.pixel_format = Some(PixelFormat::Gray12Le);
        let mut enc = IcerEncoder::new_from_params(&params);
        enc.set_options(EncodeOptions::compressed());

        let mut img = IcerImage::zeros_deep(8, 6, 12).unwrap();
        for y in 0..6u32 {
            for x in 0..8u32 {
                img.set_sample(0, x, y, ((x * 511 + y * 173) % 4096) as u16);
            }
        }
        let frame = Frame::from(img.clone());
        enc.send_frame(&frame).expect("deep send_frame");
        let pkt = enc.receive_packet().expect("deep packet");
        assert!(pkt.flags.keyframe);
        let decoded = decode(&pkt.data).expect("deep registry stream");
        assert_eq!(decoded.format, IcerPixelFormat::Gray16Le);
        assert_eq!(decoded.bit_depth, 12);
        assert_eq!(decoded.planes, img.planes);
        assert!(matches!(enc.receive_packet(), Err(Error::NeedMore)));
    }

    #[test]
    fn frame_bridge_round_trips_every_layout_and_depth() {
        let mut params = CodecParameters::video(CodecId::new(CODEC_ID_STR));
        params.width = Some(4);
        params.height = Some(3);
        let cases: Vec<(IcerImage, PixelFormat, Option<u8>)> = vec![
            (
                IcerImage::zeros(4, 3, IcerPixelFormat::Gray8),
                PixelFormat::Gray8,
                None,
            ),
            (
                IcerImage::zeros_deep(4, 3, 10).unwrap(),
                PixelFormat::Gray10Le,
                None,
            ),
            (
                IcerImage::zeros_deep(4, 3, 12).unwrap(),
                PixelFormat::Gray12Le,
                None,
            ),
            (
                IcerImage::zeros_deep(4, 3, 16).unwrap(),
                PixelFormat::Gray16Le,
                None,
            ),
            (
                IcerImage::zeros_deep(4, 3, 11).unwrap(),
                PixelFormat::Gray16Le,
                Some(11),
            ),
            (
                IcerImage::zeros(4, 3, IcerPixelFormat::Gray8)
                    .with_bit_depth(5)
                    .unwrap(),
                PixelFormat::Gray8,
                Some(5),
            ),
            (
                IcerImage::zeros(4, 3, IcerPixelFormat::Yuv444P),
                PixelFormat::Yuv444P,
                None,
            ),
            (
                IcerImage::from_rgb8(4, 3, vec![1; 36]).unwrap(),
                PixelFormat::Gbrp8,
                None,
            ),
        ];
        for (img, core_fmt, sig) in cases {
            assert_eq!(to_core_pixel_format(img.format, img.bit_depth), core_fmt);
            let frame = VideoFrame::from(&img);
            assert_eq!(frame.image_planes().len(), img.format.plane_count());
            assert_eq!(frame.plane_significant_bits(0), sig);
            assert!(frame.color_signal().is_none(), "no colour stamped");
            params.pixel_format = Some(core_fmt);
            let back = IcerImage::from_video_frame(&frame, &params).unwrap();
            assert_eq!(back, img);
            let back2 = IcerImage::try_from((&frame, &params)).unwrap();
            assert_eq!(back2, img);
        }
        // Without a declared pixel format the plane count decides.
        params.pixel_format = None;
        let gray = IcerImage::zeros(4, 3, IcerPixelFormat::Gray8);
        assert_eq!(
            IcerImage::from_video_frame(&VideoFrame::from(&gray), &params)
                .unwrap()
                .format,
            IcerPixelFormat::Gray8
        );
        let yuv = IcerImage::zeros(4, 3, IcerPixelFormat::Yuv444P);
        assert_eq!(
            IcerImage::from_video_frame(&VideoFrame::from(&yuv), &params)
                .unwrap()
                .format,
            IcerPixelFormat::Yuv444P
        );
        // Unsupported core layouts and missing geometry are errors.
        params.pixel_format = Some(PixelFormat::Rgb24);
        assert!(
            IcerImage::from_video_frame(&VideoFrame::from(&gray), &params)
                .unwrap_err()
                .is_unsupported()
        );
        assert!(from_core_pixel_format(PixelFormat::Yuv420P).is_err());
        params.width = None;
        params.pixel_format = Some(PixelFormat::Gray8);
        assert!(
            IcerImage::from_video_frame(&VideoFrame::from(&gray), &params)
                .unwrap_err()
                .is_invalid_data()
        );
    }

    #[test]
    fn register_via_runtime_context_installs_codec_factory() {
        let mut ctx = RuntimeContext::new();
        register(&mut ctx);
        let params = CodecParameters::video(CodecId::new(CODEC_ID_STR));
        let dec = ctx
            .codecs
            .first_decoder(&params)
            .expect("icer decoder factory");
        assert_eq!(dec.codec_id().as_str(), CODEC_ID_STR);
        // The unified entry point also wires the .icer extension hint
        // through the same call.
        assert_eq!(ctx.containers.container_for_extension("icer"), Some("icer"),);
    }

    #[test]
    fn decoder_adapter_emits_one_frame_per_cube_band() {
        let bands: Vec<IcerImage> = (0..3u16)
            .map(|b| {
                let mut img = IcerImage::zeros_deep(6, 4, 9).unwrap();
                for y in 0..4 {
                    for x in 0..6 {
                        img.set_sample(0, x, y, (b * 100 + x as u16 * 3 + y as u16) % 512);
                    }
                }
                img
            })
            .collect();
        let frames: Vec<crate::Frame> = bands
            .iter()
            .enumerate()
            .map(|(i, b)| crate::Frame::new(b.clone(), i as u32))
            .collect();
        let bytes = crate::encode_all(&frames, &EncodeOptions::compressed()).unwrap();
        let params = CodecParameters::video(CodecId::new(CODEC_ID_STR));
        let mut dec = make_decoder(&params).unwrap();
        assert!(matches!(dec.receive_frame(), Err(Error::NeedMore)));
        dec.send_packet(&Packet::new(0u32, TimeBase::new(1, 1), bytes))
            .unwrap();
        for b in &bands {
            match dec.receive_frame().unwrap() {
                Frame::Video(v) => {
                    assert_eq!(v.image_planes()[0].data, b.planes[0].data);
                    assert_eq!(v.plane_significant_bits(0), Some(9));
                }
                _ => panic!("video frame expected"),
            }
        }
        dec.flush().unwrap();
        assert!(matches!(dec.receive_frame(), Err(Error::Eof)));
    }

    #[test]
    fn limit_errors_map_to_resource_exhausted() {
        let e: Error = IcerError::limit("x").into();
        assert!(matches!(e, Error::ResourceExhausted(_)));
        let e: Error = IcerError::Truncated.into();
        assert!(matches!(e, Error::InvalidData(_)));
        let e: Error = IcerError::from(std::io::Error::other("io")).into();
        assert!(matches!(e, Error::Io(_)));
    }
}
