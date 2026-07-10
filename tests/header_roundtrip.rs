//! Round-trip cover for `SegmentHeader` + `PacketHeader` parse/encode.

use oxideav_icer::{walk_segment, BitPlanePass, PacketHeader, SegmentHeader, WaveletFilter};

#[test]
fn segment_header_roundtrip_all_filters() {
    for filter in [
        WaveletFilter::Reversible53,
        WaveletFilter::NineSevenA,
        WaveletFilter::FilterB,
        WaveletFilter::FilterC,
        WaveletFilter::FilterD,
        WaveletFilter::FilterE,
        WaveletFilter::FilterF,
        WaveletFilter::FilterG,
    ] {
        for levels in 1..=6 {
            for uncompressed in [false, true] {
                for interleaved_entropy in [false, true] {
                    for priority_interleaved in [false, true] {
                        let h = SegmentHeader {
                            sync_prefix: 0xACED,
                            filter,
                            decomp_levels: levels,
                            uncompressed,
                            width: 256,
                            height: 192,
                            bit_plane_count: 8,
                            interleaved_entropy,
                            transform_segmented: false,
                            total_segments: 0,
                            priority_interleaved,
                            segment_length: 1234,
                            segment_index: 0,
                        };
                        let enc = h.encode();
                        let (parsed, n) = SegmentHeader::parse(&enc).unwrap();
                        assert_eq!(n, SegmentHeader::ENCODED_BYTES);
                        assert_eq!(parsed, h);
                    }
                }
            }
        }
    }
}

/// §V.B transform-domain segment headers round-trip the
/// (total_segments, segment_index) pair carried in bytes 10..12, and
/// the parser enforces the §V.D validity conditions.
#[test]
fn segment_header_transform_segmented_roundtrip() {
    for (total, index) in [(1u8, 0u16), (2, 1), (17, 16), (255, 254)] {
        let h = SegmentHeader {
            sync_prefix: 0xACED,
            filter: WaveletFilter::Reversible53,
            decomp_levels: 3,
            uncompressed: false,
            width: 64,
            height: 64,
            bit_plane_count: 8,
            interleaved_entropy: false,
            transform_segmented: true,
            total_segments: total,
            priority_interleaved: false,
            segment_length: 0,
            segment_index: index,
        };
        let enc = h.encode();
        let (parsed, _) = SegmentHeader::parse(&enc).unwrap();
        assert_eq!(parsed, h);
    }
}

#[test]
fn segment_header_transform_segmented_rejects_bad_counts() {
    let h = SegmentHeader {
        sync_prefix: 0xACED,
        filter: WaveletFilter::Reversible53,
        decomp_levels: 3,
        uncompressed: false,
        width: 64,
        height: 64,
        bit_plane_count: 8,
        interleaved_entropy: false,
        transform_segmented: true,
        total_segments: 4,
        priority_interleaved: false,
        segment_length: 0,
        segment_index: 0,
    };
    // total_segments = 0 is invalid.
    let mut enc = h.encode();
    enc[10] = 0;
    assert!(SegmentHeader::parse(&enc).is_err());
    // segment_index >= total_segments is invalid.
    let mut enc = h.encode();
    enc[11] = 4;
    assert!(SegmentHeader::parse(&enc).is_err());
}

/// The §VI.A minimum-loss byte (packet header byte 3, previously
/// reserved-must-be-zero) round-trips; 0 remains the historical form.
#[test]
fn packet_header_min_loss_roundtrip() {
    for m in [0u8, 1, 7, 255] {
        let p = PacketHeader {
            bit_plane: 3,
            pass: BitPlanePass::Significance,
            body_length: 99,
            min_loss: m,
        };
        let enc = p.encode();
        assert_eq!(enc[3], m);
        let (parsed, _) = PacketHeader::parse(&enc).unwrap();
        assert_eq!(parsed, p);
    }
}

#[test]
fn segment_header_rejects_zero_sync() {
    let bytes = [0u8; SegmentHeader::ENCODED_BYTES];
    assert!(SegmentHeader::parse(&bytes).is_err());
}

#[test]
fn segment_header_rejects_truncated() {
    let bytes = [0x42u8; SegmentHeader::ENCODED_BYTES - 1];
    assert!(SegmentHeader::parse(&bytes).is_err());
}

#[test]
fn segment_header_rejects_zero_levels() {
    // Bit-pattern: filter=0, levels=0, uncompressed=0 => byte 2 = 0.
    let mut bytes = [0u8; SegmentHeader::ENCODED_BYTES];
    bytes[0] = 0xAC;
    bytes[1] = 0xED;
    bytes[2] = 0; // filter=0 levels=0
    bytes[3] = 0;
    bytes[4] = 16;
    bytes[5] = 0;
    bytes[6] = 16;
    bytes[7] = 8u8 << 2; // bit_plane_count=8
    assert!(SegmentHeader::parse(&bytes).is_err());
}

#[test]
fn packet_header_roundtrip() {
    for bp in [0u8, 1, 5, 31] {
        for pass in [
            BitPlanePass::Cleanup,
            BitPlanePass::Significance,
            BitPlanePass::Refinement,
        ] {
            let p = PacketHeader {
                bit_plane: bp,
                pass,
                body_length: 0xCAFE,
                min_loss: 0,
            };
            let enc = p.encode();
            let (parsed, n) = PacketHeader::parse(&enc).unwrap();
            assert_eq!(n, PacketHeader::ENCODED_BYTES);
            assert_eq!(parsed, p);
        }
    }
}

#[test]
fn walk_segment_with_two_packets() {
    // Build a segment containing two packets manually then walk.
    let body0 = vec![0xAA; 32];
    let body1 = vec![0x55; 16];
    let p0 = PacketHeader {
        bit_plane: 0,
        pass: BitPlanePass::Cleanup,
        body_length: body0.len() as u16,
        min_loss: 0,
    };
    let p1 = PacketHeader {
        bit_plane: 1,
        pass: BitPlanePass::Refinement,
        body_length: body1.len() as u16,
        min_loss: 0,
    };
    let mut packets_concat = Vec::new();
    packets_concat.extend_from_slice(&p0.encode());
    packets_concat.extend_from_slice(&body0);
    packets_concat.extend_from_slice(&p1.encode());
    packets_concat.extend_from_slice(&body1);

    let h = SegmentHeader {
        sync_prefix: 0xACED,
        filter: WaveletFilter::Reversible53,
        decomp_levels: 3,
        uncompressed: false,
        width: 32,
        height: 16,
        bit_plane_count: 8,
        interleaved_entropy: false,
        transform_segmented: false,
        total_segments: 0,
        priority_interleaved: false,
        segment_length: packets_concat.len() as u16,
        segment_index: 0,
    };
    let mut buf = Vec::new();
    buf.extend_from_slice(&h.encode());
    buf.extend_from_slice(&packets_concat);

    let walked = walk_segment(&buf).unwrap();
    assert_eq!(walked.header, h);
    assert_eq!(walked.packets.len(), 2);
    assert_eq!(walked.packets[0].header, p0);
    assert_eq!(walked.packets[0].body, body0.as_slice());
    assert_eq!(walked.packets[1].header, p1);
    assert_eq!(walked.packets[1].body, body1.as_slice());
    assert_eq!(walked.consumed, buf.len());
}
