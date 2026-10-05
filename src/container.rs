//! ICER container: demuxer + muxer for the `oxideav-core` container
//! registry (the `icer` container, `.icer`).
//!
//! An ICER file *is* the codec bitstream (IPN 42-155 §IV segment stream,
//! the crate's plane container, or an IPN 42-164 ICER-3D cube), so the
//! framework packet is the whole file and the registry
//! [`crate::make_decoder`] decodes it with [`crate::decode_all_with`]
//! (one implementation):
//!
//! * **Probe.** ICER has no file magic except the cube's `00 00 C3 01`
//!   (score 100). Everything else is structural, through the same walk
//!   [`crate::info`] performs: a plane container whose every segment
//!   header chains to the end of the buffer scores 90, a bare 2-D
//!   segment stream that does so 70; a container whose walk runs off the
//!   probe buffer (a stream larger than the buffer) 50; a bare stream
//!   that runs off the buffer is indistinguishable from noise (the
//!   12-byte header plausibility alone accepts about a quarter of random
//!   inputs) and scores 0 without an `.icer` hint. The hint lifts any
//!   structural match to at least 75 and scores 25 alone.
//! * **Demuxer.** One video stream — `width` / `height` and the native
//!   layout from [`crate::info`] (`Gray8` / `Gray16Le` / `Yuv444P` /
//!   `Gbrp8`, labelled with core's depth rung exactly as the registry
//!   decoder labels its frames: `Gray10Le` / `Gray12Le` for 10- / 12-bit
//!   deep gray), **no colour signal** (ICER carries none; the crate's
//!   documented conventions stay on the standalone `ColorInfo`) — and
//!   **one packet holding the whole file**, `pts` 0 in a `1/1` time
//!   base. An ICER-3D cube is one joint 3-D wavelet transform: no
//!   per-band bitstream exists, so the cube stays one packet and the
//!   registry decoder emits one frame per spectral band in
//!   [`crate::decode_all`] order with `pts` = band index (the §V.C
//!   segment election and the band walk stay in the codec). ICER has no
//!   timing: no `duration` is stamped.
//! * **Muxer.** One packet — the encoder's complete 2-D stream (or a
//!   cube) — is written verbatim. Several packets are the bands of one
//!   cube: each is decoded and the images go through
//!   [`crate::encode_all`] (`IcerCube::from_band_images` →
//!   `encode_icer3d` under [`crate::EncodeOptions::default`]), the mirror
//!   of the demuxer's band walk. Bands must share geometry and depth.
//!
//! Gated behind the `registry` feature: every type here comes from
//! `oxideav-core`.

use std::io::{Read, SeekFrom, Write};

use oxideav_core::{
    CodecId, CodecParameters, CodecResolver, ContainerRegistry, Demuxer, Error, MediaType, Muxer,
    Packet, ProbeData, ProbeScore, ReadSeek, Result, StreamInfo, TimeBase, WriteSeek,
    MAX_PROBE_SCORE, PROBE_SCORE_EXTENSION,
};

use crate::cube::is_cube;
use crate::error::IcerError;
use crate::image::{Frame, StreamKind};
use crate::plane_container::is_container;
use crate::registry::to_core_pixel_format;
use crate::{DecodeOptions, EncodeOptions};

/// Registered container name (the same string as the codec id).
pub const CONTAINER_NAME: &str = "icer";

/// Score of a plane container whose every segment header chains to the
/// end of the probe buffer.
pub const PROBE_SCORE_CONTAINER_WALKED: ProbeScore = 90;
/// Score of a bare 2-D segment stream whose every segment header chains
/// to the end of the probe buffer.
pub const PROBE_SCORE_SEGMENTS_WALKED: ProbeScore = 70;
/// Score of a plane container whose walk runs off the probe buffer (a
/// stream larger than the buffer).
pub const PROBE_SCORE_CONTAINER_CUT: ProbeScore = 50;
/// Floor for a structural match corroborated by the `.icer` extension.
pub const PROBE_SCORE_WITH_EXTENSION: ProbeScore = 75;

/// Register the ICER container: demuxer, muxer, `.icer` extension and
/// the probe.
pub fn register(reg: &mut ContainerRegistry) {
    reg.register_demuxer(CONTAINER_NAME, open_demuxer);
    reg.register_muxer(CONTAINER_NAME, open_muxer);
    reg.register_extension("icer", CONTAINER_NAME);
    reg.register_probe(CONTAINER_NAME, probe);
}

/// Structural probe (see the module docs).
pub fn probe(data: &ProbeData) -> ProbeScore {
    let ext_hint = matches!(data.ext, Some("icer"));
    if is_cube(data.buf) {
        return MAX_PROBE_SCORE;
    }
    let score = if !crate::probe(data.buf) {
        0
    } else {
        match crate::info_with(data.buf, &DecodeOptions::default()) {
            Ok(i) => match i.kind {
                StreamKind::PlaneContainer => PROBE_SCORE_CONTAINER_WALKED,
                _ => PROBE_SCORE_SEGMENTS_WALKED,
            },
            Err(IcerError::Truncated) if is_container(data.buf) => PROBE_SCORE_CONTAINER_CUT,
            Err(_) => 0,
        }
    };
    if score == 0 {
        return if ext_hint { PROBE_SCORE_EXTENSION } else { 0 };
    }
    if ext_hint {
        score.max(PROBE_SCORE_WITH_EXTENSION)
    } else {
        score
    }
}

// ---------------------------------------------------------------------------
// Demuxer
// ---------------------------------------------------------------------------

/// Open an ICER file as a demuxer (see the module docs).
pub fn open_demuxer(
    mut input: Box<dyn ReadSeek>,
    _codecs: &dyn CodecResolver,
) -> Result<Box<dyn Demuxer>> {
    input.seek(SeekFrom::Start(0))?;
    let mut buf = Vec::new();
    input.read_to_end(&mut buf)?;
    drop(input);
    // Framing only: geometry, layout, depth and band count, with the
    // default decode caps — the same accept / reject verdict as `info`.
    let info = crate::info(&buf)?;
    let mut params = CodecParameters::video(CodecId::new(crate::CODEC_ID_STR));
    params.width = Some(info.width);
    params.height = Some(info.height);
    params.pixel_format = Some(to_core_pixel_format(info.format, info.bit_depth));
    let stream = StreamInfo {
        index: 0,
        time_base: TimeBase::new(1, 1),
        duration: None,
        start_time: Some(0),
        params,
    };
    let mut metadata = Vec::new();
    if info.kind == StreamKind::Cube {
        metadata.push(("bands".to_string(), info.frames.to_string()));
    }
    metadata.push(("bit_depth".to_string(), info.bit_depth.to_string()));
    Ok(Box::new(IcerDemuxer {
        streams: vec![stream],
        data: Some(buf),
        metadata,
    }))
}

struct IcerDemuxer {
    streams: Vec<StreamInfo>,
    data: Option<Vec<u8>>,
    metadata: Vec<(String, String)>,
}

impl Demuxer for IcerDemuxer {
    fn format_name(&self) -> &str {
        CONTAINER_NAME
    }
    fn streams(&self) -> &[StreamInfo] {
        &self.streams
    }
    fn next_packet(&mut self) -> Result<Packet> {
        match self.data.take() {
            Some(bytes) => {
                let mut pkt = Packet::new(0, TimeBase::new(1, 1), bytes);
                pkt.pts = Some(0);
                pkt.dts = Some(0);
                pkt.flags.keyframe = true;
                Ok(pkt)
            }
            None => Err(Error::Eof),
        }
    }
    fn metadata(&self) -> &[(String, String)] {
        &self.metadata
    }
}

// ---------------------------------------------------------------------------
// Muxer
// ---------------------------------------------------------------------------

/// Open an ICER muxer over `output` for exactly one video stream.
pub fn open_muxer(output: Box<dyn WriteSeek>, streams: &[StreamInfo]) -> Result<Box<dyn Muxer>> {
    if streams.len() != 1 {
        return Err(Error::invalid(
            "ICER muxer: expected exactly one video stream",
        ));
    }
    if streams[0].params.media_type != MediaType::Video {
        return Err(Error::invalid("ICER muxer: stream must be video"));
    }
    Ok(Box::new(IcerMuxer {
        output,
        packets: Vec::new(),
    }))
}

struct IcerMuxer {
    output: Box<dyn WriteSeek>,
    packets: Vec<Vec<u8>>,
}

impl Muxer for IcerMuxer {
    fn format_name(&self) -> &str {
        CONTAINER_NAME
    }
    fn write_header(&mut self) -> Result<()> {
        Ok(())
    }
    fn write_packet(&mut self, packet: &Packet) -> Result<()> {
        if packet.data.is_empty() {
            return Err(Error::invalid("ICER muxer: empty packet"));
        }
        if !crate::probe(&packet.data) {
            return Err(Error::invalid(
                "ICER muxer: packet is not an ICER stream (no cube magic, plane container or \
                 segment header)",
            ));
        }
        self.packets.push(packet.data.clone());
        Ok(())
    }
    fn write_trailer(&mut self) -> Result<()> {
        let packets = std::mem::take(&mut self.packets);
        match packets.len() {
            0 => Err(Error::invalid(
                "ICER muxer: no packet written (an ICER file holds at least one image)",
            )),
            1 => {
                self.output.write_all(&packets[0])?;
                Ok(())
            }
            _ => {
                let cube = combine_bands(&packets)?;
                self.output.write_all(&cube)?;
                Ok(())
            }
        }
    }
}

/// Decode every packet as one spectral band and write the ICER-3D cube
/// [`crate::encode_all`] writes for those images (the inverse of the
/// registry decoder's band walk).
pub(crate) fn combine_bands(packets: &[Vec<u8>]) -> crate::Result<Vec<u8>> {
    // Framing first, pixels later: every band must share the first
    // packet's geometry, layout and depth, and be a single image. Checked
    // from the headers so a mismatched set fails before any plane is
    // allocated.
    let first = crate::info(&packets[0])?;
    for (i, p) in packets.iter().enumerate().skip(1) {
        let info = crate::info(p)?;
        if info.frames != 1 || first.frames != 1 {
            return Err(IcerError::invalid(format!(
                "ICER muxer: packet {i} holds {} image(s); cube bands are single 2-D streams",
                info.frames
            )));
        }
        if (info.width, info.height, info.format, info.bit_depth)
            != (first.width, first.height, first.format, first.bit_depth)
        {
            return Err(IcerError::invalid(format!(
                "ICER muxer: packet {i} is {}×{} {:?} {}-bit, band 0 is {}×{} {:?} {}-bit; cube \
                 bands must match",
                info.width,
                info.height,
                info.format,
                info.bit_depth,
                first.width,
                first.height,
                first.format,
                first.bit_depth
            )));
        }
    }
    let mut frames = Vec::with_capacity(packets.len());
    for (i, p) in packets.iter().enumerate() {
        let image = crate::decode(p).map_err(|e| match e {
            IcerError::InvalidData(msg) => IcerError::InvalidData(format!(
                "ICER muxer: packet {i} does not decode as a band: {msg}"
            )),
            other => other,
        })?;
        frames.push(Frame::new(image, i as u32));
    }
    crate::encode_all(&frames, &EncodeOptions::default())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{IcerImage, IcerPixelFormat};

    fn gray(w: u32, h: u32, salt: u8) -> IcerImage {
        let mut img = IcerImage::zeros(w, h, IcerPixelFormat::Gray8);
        for (i, b) in img.planes[0].data.iter_mut().enumerate() {
            *b = (i as u8).wrapping_mul(7).wrapping_add(salt);
        }
        img
    }

    #[test]
    fn probe_scores_by_structure() {
        let bare = crate::encode(&gray(16, 12, 1), &EncodeOptions::default()).unwrap();
        assert_eq!(
            probe(&ProbeData {
                buf: &bare,
                ext: None
            }),
            PROBE_SCORE_SEGMENTS_WALKED
        );
        assert_eq!(
            probe(&ProbeData {
                buf: &bare,
                ext: Some("icer")
            }),
            PROBE_SCORE_WITH_EXTENSION
        );
        // A bare stream cut short is indistinguishable from noise.
        assert_eq!(
            probe(&ProbeData {
                buf: &bare[..bare.len() - 3],
                ext: None
            }),
            0
        );
        assert_eq!(
            probe(&ProbeData {
                buf: &bare[..bare.len() - 3],
                ext: Some("icer")
            }),
            PROBE_SCORE_EXTENSION
        );
        let deep = crate::encode(
            &IcerImage::zeros_deep(16, 12, 12).unwrap(),
            &EncodeOptions::default(),
        )
        .unwrap();
        assert_eq!(
            probe(&ProbeData {
                buf: &deep,
                ext: None
            }),
            PROBE_SCORE_CONTAINER_WALKED
        );
        assert_eq!(
            probe(&ProbeData {
                buf: &deep[..deep.len() - 3],
                ext: None
            }),
            PROBE_SCORE_CONTAINER_CUT
        );
        let cube = crate::encode_all(
            &[Frame::new(gray(8, 8, 0), 0), Frame::new(gray(8, 8, 9), 1)],
            &EncodeOptions::default(),
        )
        .unwrap();
        assert_eq!(
            probe(&ProbeData {
                buf: &cube,
                ext: None
            }),
            MAX_PROBE_SCORE
        );
        assert_eq!(
            probe(&ProbeData {
                buf: b"farbfeld\0\0\0\x01\0\0\0\x01",
                ext: Some("icer")
            }),
            PROBE_SCORE_EXTENSION
        );
        assert_eq!(
            probe(&ProbeData {
                buf: b"farbfeld\0\0\0\x01\0\0\0\x01",
                ext: Some("ff")
            }),
            0
        );
        assert_eq!(
            probe(&ProbeData {
                buf: &[],
                ext: None
            }),
            0
        );
    }
}
