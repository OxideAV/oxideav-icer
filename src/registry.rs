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
    Decoder, Encoder, Error, Frame, Packet, PixelFormat, Result, TimeBase, VideoFrame,
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
pub fn register(reg: &mut CodecRegistry) {
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
        // Build an IcerImage from the incoming video frame. Round 1
        // only supports a single-plane Gray8 path.
        let plane = video
            .planes
            .first()
            .ok_or_else(|| Error::invalid("icer encoder: video frame has no planes"))?;
        let img = IcerImage {
            width,
            height,
            pixel_format: IcerPixelFormat::Gray8,
            planes: vec![crate::image::IcerPlane {
                stride: plane.stride,
                data: plane.data.clone(),
            }],
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
