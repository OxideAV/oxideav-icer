//! `oxideav-core` integration — `Decoder` / `Encoder` trait impls,
//! `Frame` / `Error` conversions, and the [`register`] entry point.
//!
//! Gated behind the default-on `registry` Cargo feature. Without
//! `registry` the rest of the crate still exposes the standalone
//! [`crate::parse_icer_metadata`] / [`crate::parse_icer`] /
//! [`crate::encode_icer`] API plus crate-local
//! [`crate::IcerImage`] / [`crate::IcerError`] types — none of
//! which depend on `oxideav-core`.

use oxideav_core::{
    frame::VideoPlane, CodecCapabilities, CodecId, CodecInfo, CodecParameters, CodecRegistry,
    ContainerRegistry, Decoder, Encoder, Error, Frame, Packet, PixelFormat, Result, RuntimeContext,
    TimeBase, VideoFrame,
};

use crate::encoder::EncodeOptions;
use crate::error::IcerError;
use crate::image::{IcerImage, IcerPixelFormat};
use crate::{encode_icer, parse_icer, CODEC_ID_STR};

impl From<IcerError> for Error {
    fn from(e: IcerError) -> Self {
        match e {
            IcerError::InvalidData(s) => Error::InvalidData(s),
            IcerError::Unsupported(s) => Error::Unsupported(s),
            IcerError::Truncated => Error::InvalidData("icer: truncated input".into()),
        }
    }
}

impl From<IcerPixelFormat> for PixelFormat {
    fn from(p: IcerPixelFormat) -> Self {
        match p {
            IcerPixelFormat::Gray8 => PixelFormat::Gray8,
            IcerPixelFormat::Yuv444P => PixelFormat::Yuv444P,
            // Deep gray samples are stored LSB-aligned in little-endian
            // 16-bit words (the plain-gray deep-word convention); the
            // exact 10-/12-bit rungs map onto their dedicated core
            // formats, every other depth rides the 16-bit word as-is.
            IcerPixelFormat::GrayDeep { bits: 10 } => PixelFormat::Gray10Le,
            IcerPixelFormat::GrayDeep { bits: 12 } => PixelFormat::Gray12Le,
            IcerPixelFormat::GrayDeep { .. } => PixelFormat::Gray16Le,
        }
    }
}

impl From<IcerImage> for Frame {
    fn from(img: IcerImage) -> Self {
        let planes = img
            .planes
            .into_iter()
            .map(|p| VideoPlane {
                stride: p.stride,
                data: p.data,
            })
            .collect();
        Frame::Video(VideoFrame {
            pts: Some(img.pts),
            planes,
        })
    }
}

/// Register the ICER decoder + encoder factories.
pub fn register_codecs(reg: &mut CodecRegistry) {
    let caps = CodecCapabilities::video(CODEC_ID_STR)
        .with_lossy(false)
        .with_intra_only(true);
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
/// convention every sibling crate now follows. Direct callers that
/// only need one of the two sub-registries can keep using
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

fn make_decoder(params: &CodecParameters) -> Result<Box<dyn Decoder>> {
    Ok(Box::new(IcerDecoder::new(params.codec_id.clone())))
}

fn make_encoder(params: &CodecParameters) -> Result<Box<dyn Encoder>> {
    Ok(Box::new(IcerEncoder::new_from_params(params)))
}

/// Round-1 ICER decoder — parses framing + reconstructs the
/// uncompressed (IPN 42-155 §III.D) path.
pub struct IcerDecoder {
    codec_id: CodecId,
    queued: Option<Frame>,
}

impl IcerDecoder {
    pub fn new(codec_id: CodecId) -> Self {
        Self {
            codec_id,
            queued: None,
        }
    }
}

impl Decoder for IcerDecoder {
    fn codec_id(&self) -> &CodecId {
        &self.codec_id
    }

    fn send_packet(&mut self, packet: &Packet) -> oxideav_core::Result<()> {
        let img = parse_icer(&packet.data)?;
        self.queued = Some(Frame::from(img));
        Ok(())
    }

    fn receive_frame(&mut self) -> oxideav_core::Result<Frame> {
        self.queued
            .take()
            .ok_or_else(|| Error::invalid("icer: no frame queued"))
    }

    fn flush(&mut self) -> oxideav_core::Result<()> {
        self.queued = None;
        Ok(())
    }
}

/// Round-1 ICER encoder — emits the uncompressed (IPN 42-155 §III.D)
/// path. Lossless 5/3 entropy-coded encoding lands in round 2.
pub struct IcerEncoder {
    output_params: CodecParameters,
    opts: EncodeOptions,
    pending: Option<Packet>,
    seq_counter: u64,
}

impl IcerEncoder {
    pub fn new(codec_id: CodecId) -> Self {
        Self {
            output_params: CodecParameters::video(codec_id),
            opts: EncodeOptions::default(),
            pending: None,
            seq_counter: 0,
        }
    }

    pub fn new_from_params(params: &CodecParameters) -> Self {
        Self {
            output_params: params.clone(),
            opts: EncodeOptions::default(),
            pending: None,
            seq_counter: 0,
        }
    }

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
        let width = self
            .output_params
            .width
            .ok_or_else(|| Error::invalid("icer encoder: missing width in params"))?;
        let height = self
            .output_params
            .height
            .ok_or_else(|| Error::invalid("icer encoder: missing height in params"))?;
        let video = match frame {
            Frame::Video(v) => v,
            _ => return Err(Error::invalid("icer encoder: expected Frame::Video")),
        };
        // Build an IcerImage from the incoming video frame. A
        // single-plane frame is Gray8 — or, when the encoder's
        // `CodecParameters::pixel_format` declares one of the plain-gray
        // deep-word formats, the matching deep-sample depth (the frame's
        // plane then carries LSB-aligned little-endian u16 words; without
        // the declaration a 16-bit plane would be silently misread as
        // twice-as-wide 8-bit samples). A three-plane frame is the
        // co-sited 4:4:4 colour path (encoded as three independent ICER
        // streams behind the plane container — see `encode_icer`).
        if video.planes.is_empty() {
            return Err(Error::invalid("icer encoder: video frame has no planes"));
        }
        let pixel_format = match (video.planes.len(), self.output_params.pixel_format) {
            (1, Some(PixelFormat::Gray10Le)) => IcerPixelFormat::GrayDeep { bits: 10 },
            (1, Some(PixelFormat::Gray12Le)) => IcerPixelFormat::GrayDeep { bits: 12 },
            (1, Some(PixelFormat::Gray16Le)) => IcerPixelFormat::GrayDeep { bits: 16 },
            (1, _) => IcerPixelFormat::Gray8,
            (3, _) => IcerPixelFormat::Yuv444P,
            (n, _) => {
                return Err(Error::unsupported(format!(
                    "icer encoder: {n}-plane frame (only 1=Gray8/deep or 3=Yuv444P supported)"
                )))
            }
        };
        let planes = video
            .planes
            .iter()
            .map(|p| crate::image::IcerPlane {
                stride: p.stride,
                data: p.data.clone(),
            })
            .collect();
        let img = IcerImage {
            width,
            height,
            pixel_format,
            planes,
            pts: video.pts.unwrap_or(0),
        };
        let bytes = encode_icer(&img, &self.opts)?;
        let pkt = Packet::new(0u32, TimeBase::new(1, 1), bytes);
        self.seq_counter = self.seq_counter.wrapping_add(1);
        self.pending = Some(pkt);
        Ok(())
    }

    fn receive_packet(&mut self) -> oxideav_core::Result<Packet> {
        self.pending
            .take()
            .ok_or_else(|| Error::invalid("icer encoder: no packet queued"))
    }

    fn flush(&mut self) -> oxideav_core::Result<()> {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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

        let mut img = IcerImage::zeros(8, 6, IcerPixelFormat::GrayDeep { bits: 12 });
        for y in 0..6u32 {
            for x in 0..8u32 {
                img.set_sample(0, x, y, ((x * 511 + y * 173) % 4096) as u16);
            }
        }
        let frame = Frame::from(img.clone());
        enc.send_frame(&frame).expect("deep send_frame");
        let pkt = enc.receive_packet().expect("deep packet");
        let decoded = parse_icer(&pkt.data).expect("deep registry stream");
        assert_eq!(decoded.pixel_format, IcerPixelFormat::GrayDeep { bits: 12 });
        assert_eq!(decoded.planes, img.planes);
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
}
