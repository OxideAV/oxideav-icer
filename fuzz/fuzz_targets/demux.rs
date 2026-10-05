#![no_main]
//! The `icer` container through the registry (feature `registry`): the
//! probe with and without the `.icer` hint, the demuxer, the registry
//! decoder draining every band frame of the packet, and the muxer on the
//! demuxed packet (twice, so the cube-combine path runs). Every input
//! must end in `Ok` or `Err` — never a panic, a debug overflow or an
//! attacker-sized allocation.

use std::io::Cursor;

use libfuzzer_sys::fuzz_target;
use oxideav_core::{CodecId, CodecParameters, ProbeData, RuntimeContext, StreamInfo, TimeBase};

fuzz_target!(|data: &[u8]| {
    if data.len() > 1 << 20 {
        return;
    }
    let mut ctx = RuntimeContext::new();
    oxideav_icer::register(&mut ctx);
    let _ = oxideav_icer::container::probe(&ProbeData {
        buf: data,
        ext: None,
    });
    let _ = oxideav_icer::container::probe(&ProbeData {
        buf: data,
        ext: Some("icer"),
    });
    let _ = ctx
        .containers
        .probe_input(&mut Cursor::new(data), Some("icer"));
    let Ok(mut demuxer) = ctx.containers.open_demuxer(
        "icer",
        Box::new(Cursor::new(data.to_vec())),
        &ctx.codecs,
    ) else {
        return;
    };
    let stream = demuxer.streams()[0].clone();
    let Ok(pkt) = demuxer.next_packet() else {
        return;
    };
    // The codec legs below allocate in proportion to the geometry the
    // header claims (the decoder's working set is a few hundred bytes per
    // sample; the ICER-3D encoder's ~40 per sample per band), and a
    // 12-byte header may lawfully claim tens of megapixels within the
    // decoder's 256 MP cap. The probe and demuxer legs above are
    // allocation-free and run on every input; bound the pixel budget of
    // the codec legs so the harness measures bugs, not footprint.
    let Ok(info) = oxideav_icer::info(&pkt.data) else {
        return;
    };
    let samples = u64::from(info.width) * u64::from(info.height) * u64::from(info.frames.max(1));
    if samples > 1 << 20 {
        return;
    }
    if let Ok(mut dec) = ctx.codecs.first_decoder(&stream.params) {
        if dec.send_packet(&pkt).is_ok() {
            let mut n = 0;
            while dec.receive_frame().is_ok() && n < 64 {
                n += 1;
            }
        }
    }
    let out_stream = StreamInfo {
        index: 0,
        time_base: TimeBase::new(1, 1),
        duration: None,
        start_time: Some(0),
        params: CodecParameters::video(CodecId::new(oxideav_icer::CODEC_ID_STR)),
    };
    if let Ok(mut muxer) = ctx.containers.open_muxer(
        "icer",
        Box::new(Cursor::new(Vec::<u8>::new())),
        &[out_stream],
    ) {
        let _ = muxer.write_header();
        let _ = muxer.write_packet(&pkt);
        let _ = muxer.write_packet(&pkt);
        let _ = muxer.write_trailer();
    }
});
