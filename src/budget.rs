//! Decoder working-set planning — every byte the decoder will hold at
//! its peak, derived from the framing alone (IPN 42-155 §IV segment
//! headers, the plane container, the IPN 42-164 cube header) before
//! the first buffer is allocated.
//!
//! [`crate::DecodeOptions::max_bytes`] is checked against this plan,
//! not against the decoded planes alone: ICER's wire format lets a
//! 12-byte header claim any `u16 × u16` geometry, and the decoder's
//! coefficient buffer + coder state are several times the output size,
//! so an output-only budget under-counts what a hostile header can
//! make the process commit. The constants below are the per-sample
//! sizes of the buffers the decode paths actually allocate; the
//! `memory_budget` integration test pins that the measured peak heap of
//! every decode stays at or below the plan.
//!
//! # Formula
//!
//! For a 2-D stream (bare segments or the plane container; planes and
//! row-strip segments are decoded one after another, so the output
//! planes are the only buffers alive across segments):
//!
//! ```text
//! plan = Σ_planes (width × height × sample_bytes)        -- the output
//!      + max over segments of segment_working_set
//!      + FIXED_OVERHEAD
//!
//! segment_working_set (compressed row strip, w × h pixels)
//!      = w × h × (COEFF_BYTES + STATE_BYTES_2D [+ SKIP_MAP_BYTES when min_loss > 0])
//!      + DWT_SCRATCH_PER_LINE × max(w, h) + segment_bytes + SEGMENT_OVERHEAD
//! segment_working_set (§V.B transform-domain, W × H image)
//!      = W × H × (COEFF_BYTES + STATE_BYTES_2D + SEGMENT_MAP_BYTES [+ SKIP_MAP_BYTES])
//!      + DWT_SCRATCH_PER_LINE × max(W, H) + Σ segment_bytes + SEGMENT_OVERHEAD
//! segment_working_set (§III.D uncompressed) = 0   -- copied straight into the plane
//! ```
//!
//! For an ICER-3D cube (`W × H × bands` samples, `strip_h`-row strips
//! or one §V.D transform-domain partition), decoded band by band into
//! `bands` frames:
//!
//! ```text
//! plan = W × H × bands × sample_bytes                     -- the band frames
//!      + V_seg × (COEFF_BYTES + STATE_BYTES_3D) + geometry tables + FIXED_OVERHEAD
//! V_seg = W × strip_h × bands (strips) or W × H × bands (transform-domain)
//! ```

use crate::header::SegmentHeader;

/// Bytes of one wavelet coefficient: the `i32` buffer the bit-plane
/// decoder accumulates magnitudes into and the inverse DWT runs over.
/// IPN 42-155 §II.C Table 4: a 16-bit input range fits 32-bit words
/// after two high-pass operations under every filter (the smallest
/// two-op 32-bit entry, filter F, is 422 726 500 ≫ 65 535), so `i32`
/// is sufficient for every sample depth the crate decodes; the header's
/// 6-bit bit-plane count (`Q ≤ 32`) bounds hostile magnitudes to the
/// same word.
pub(crate) const COEFF_BYTES: u64 = 4;

/// Bytes of per-coefficient coder state in the 2-D bit-plane decoder:
/// one packed byte — the §III.B category (four categories, 2 bits),
/// the sign (1 bit) and the deepest delivered bit plane (`0..=31`,
/// 5 bits; the segment header's `Q` field is 6 bits and the coder
/// accepts `Q ≤ 31`).
pub(crate) const STATE_BYTES_2D: u64 = 1;

/// Bytes of per-coefficient coder state in the ICER-3D decoder: the
/// IPN 42-164 §IV.C category (2 bits) and the sign (1 bit), packed in
/// one byte.
pub(crate) const STATE_BYTES_3D: u64 = 1;

/// Bytes per coefficient of the §VI.A minimum-loss skip map, present
/// only when a segment's `min_loss > 0`.
pub(crate) const SKIP_MAP_BYTES: u64 = 1;

/// Bytes per coefficient of the §V.B coefficient → segment map
/// (`u16` segment index), present only for transform-domain streams.
pub(crate) const SEGMENT_MAP_BYTES: u64 = 2;

/// Inverse-DWT scratch per lattice line (the longest row or column):
/// gathered samples, low/high halves and the reconstructed line as
/// `i32`, the §II.A `d` / `r` sequences as `i64`.
pub(crate) const DWT_SCRATCH_PER_LINE: u64 = 32;

/// Per-segment bookkeeping that does not scale with the geometry:
/// the walked packet records, the `EncodedPacket` copies' headers,
/// the §III.A packet schedule (at most `(3D + 1) × Q` units), the
/// context model, the entropy decoder.
pub(crate) const SEGMENT_OVERHEAD: u64 = 128 * 1024;

/// Per-decode bookkeeping that does not scale with the geometry: the
/// framing walk, the frame records, the container ranges.
pub(crate) const FIXED_OVERHEAD: u64 = 128 * 1024;

/// Working set of decoding one 2-D segment, given its header, the
/// §VI.A `min_loss` its packets carry and its total wire size in
/// bytes (header + body; the decoder copies packet bodies into
/// `EncodedPacket`s).
pub(crate) fn segment_working_set(
    header: &SegmentHeader,
    min_loss: u8,
    segment_bytes: usize,
) -> u64 {
    if header.uncompressed {
        return 0;
    }
    let (w, h) = (header.width as u64, header.height as u64);
    let mut per_sample = COEFF_BYTES + STATE_BYTES_2D;
    if min_loss > 0 {
        per_sample += SKIP_MAP_BYTES;
    }
    if header.transform_segmented {
        per_sample += SEGMENT_MAP_BYTES;
    }
    w * h * per_sample + DWT_SCRATCH_PER_LINE * w.max(h) + segment_bytes as u64 + SEGMENT_OVERHEAD
}

/// Decoded plane bytes of a `pixels`-sample plane at `sample_bytes`
/// per sample.
pub(crate) fn output_bytes(pixels: u64, sample_bytes: u64) -> u64 {
    pixels * sample_bytes
}

/// Working set of decoding an ICER-3D cube into one frame per band:
/// the frames, the per-segment coefficient buffer and coder state
/// (`V_seg` samples — a row strip across all bands, or the whole cube
/// for a transform-domain partition), the subband geometry tables and
/// the fixed overhead.
pub(crate) fn cube_working_set(
    width: u64,
    height: u64,
    bands: u64,
    strip_h: u64,
    transform_domain: bool,
    sample_bytes: u64,
) -> u64 {
    let total = width * height * bands;
    let v_seg = if transform_domain {
        total
    } else {
        width * strip_h * bands
    };
    // Per-subband member tables: ≤ 31 subbands (IPN 42-164 Appendix,
    // D = 3 is the deepest 3-D case the tables cover; deeper levels add
    // 4 per stage up to 43 at D = 6), each with its λ list (≤ bands) and a
    // lattice descriptor; the per-segment means (one `i32` per band).
    let geometry = 64 * (bands * 8 + 64) + bands * 4;
    output_bytes(total, sample_bytes)
        + v_seg * (COEFF_BYTES + STATE_BYTES_3D)
        + DWT_SCRATCH_PER_LINE * width.max(height).max(bands)
        + geometry
        + FIXED_OVERHEAD
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::header::WaveletFilter;

    fn header(w: u16, h: u16, uncompressed: bool, transform: bool) -> SegmentHeader {
        SegmentHeader {
            sync_prefix: 0xACED,
            filter: WaveletFilter::FilterQ,
            decomp_levels: 3,
            uncompressed,
            width: w,
            height: h,
            bit_plane_count: 8,
            interleaved_entropy: false,
            transform_segmented: transform,
            total_segments: if transform { 1 } else { 0 },
            priority_interleaved: false,
            segment_length: 0,
            segment_index: 0,
        }
    }

    #[test]
    fn segment_terms_follow_the_documented_formula() {
        let n = 64u64 * 48;
        let plain = segment_working_set(&header(64, 48, false, false), 0, 100);
        assert_eq!(
            plain,
            n * (COEFF_BYTES + STATE_BYTES_2D) + DWT_SCRATCH_PER_LINE * 64 + 100 + SEGMENT_OVERHEAD
        );
        let skipped = segment_working_set(&header(64, 48, false, false), 3, 100);
        assert_eq!(skipped - plain, n * SKIP_MAP_BYTES);
        let td = segment_working_set(&header(64, 48, false, true), 0, 100);
        assert_eq!(td - plain, n * SEGMENT_MAP_BYTES);
        assert_eq!(segment_working_set(&header(64, 48, true, false), 0, 100), 0);
    }

    #[test]
    fn cube_terms_scale_with_the_segment_volume() {
        let strips = cube_working_set(256, 256, 16, 64, false, 1);
        let td = cube_working_set(256, 256, 16, 0, true, 1);
        let total = 256u64 * 256 * 16;
        assert_eq!(
            td - strips,
            (total - 256 * 64 * 16) * (COEFF_BYTES + STATE_BYTES_3D)
        );
        assert!(strips > total);
        // A hostile 65535 × 65535 × 65535 header plans without overflow.
        let huge = cube_working_set(65535, 65535, 65535, 0, true, 2);
        assert!(huge > 1 << 50);
    }
}
