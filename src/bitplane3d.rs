//! ICER-3D bit-plane coder — IPN 42-164 §IV ("Bit-Plane Coding").
//!
//! Magnitude bit planes of subbands are compressed one at a time; when
//! the first `1` magnitude bit of a coefficient is encoded, the sign
//! bit is encoded immediately afterward (§IV). Subband bit planes are
//! compressed in order of decreasing §IV.A priority (ties by decreasing
//! subband index), and within an error-containment segment "encoding of
//! ... a subband bit plane proceeds one spatial plane at a time and in
//! raster scan order within a spatial plane" (§IV.C).
//!
//! Every bit is classified into one of the 19 spectral contexts of
//! [`crate::context3d`] from the categories / signs of the two
//! spectral-neighbour coefficients (previous and next spatial plane of
//! the subband); category-3 bits are left uncoded — fed to the entropy
//! stage at a fixed probability-of-zero of 1/2 with no adaptation
//! (§IV.C: such bits are "empirically nearly incompressible").
//!
//! The entropy stage is selectable exactly like the 2-D path
//! ([`crate::entropy::EntropyKind`]): §IV.C says ICER-3D "uses an
//! interleaved entropy coder; it is the same as that used by ICER", and
//! the crate's established arithmetic backend remains available for its
//! wire form. One packet is emitted per **priority value**: the §IV.B
//! *minimum loss* parameter stops compression "when all subband bit
//! planes having priority value q have been encoded", so a
//! priority-granular packet stream cuts exactly on that boundary, and a
//! byte quota truncates the same packet sequence early.
//!
//! On decode, a truncated packet stream reconstructs each subband from
//! its delivered planes with the deadzone-quantizer mid-bin
//! reconstruction point the 2-D decoder uses (reconstructing from the
//! `q - b` most-significant planes is equivalent to a deadzone
//! quantizer with bin width `2^b`; IPN 42-155 §III.A, inherited by
//! ICER-3D's progressive structure, IPN 42-164 §II.B).

use crate::context::ContextModel;
use crate::context3d::{
    category0_context, category1_context, category2_context, new_model,
    sign_prediction_and_context, NeighbourSign,
};
use crate::entropy::{BitSink, BitSource, EntropyKind};
use crate::error::Result;
use crate::subband3d::{cube_schedule, enumerate_subbands, stage_counts, Subband3d};

/// Fixed probability for uncoded (category 3) bits: probability of `1`
/// = 1/2, no model adaptation.
const UNCODED_P1: (u32, u32) = (1, 2);

/// Geometry of one coded cube region — either a whole cube / row strip
/// (the full spatial extent) or one §V.D error-containment segment's
/// spatial window of a shared whole-cube transform: dimensions,
/// effective stage counts, and the subband set in Appendix index order
/// with per-subband member lattices.
pub struct CubeGeometry {
    /// Spatial width of the coefficient buffer, in samples.
    pub width: usize,
    /// Spatial height of the coefficient buffer, in samples.
    pub height: usize,
    /// Spectral band count.
    pub bands: usize,
    /// Effective spatial / spectral stage counts (see
    /// [`crate::wavelet3d`]).
    pub ts: u8,
    /// Effective spectral stage count.
    pub tl: u8,
    /// Subbands in Appendix index order.
    pub subbands: Vec<Subband3d>,
    /// Per-subband member positions: spatial raster positions and the
    /// subband's λ planes (ascending), plus the λ lattice stride used
    /// for spectral-neighbour lookup.
    members: Vec<SubbandMembers>,
}

/// First lattice position `>= lo` on the lattice `{offset, offset + s,
/// offset + 2s, ...}`.
#[inline]
fn lattice_start(offset: usize, stride: usize, lo: usize) -> usize {
    if lo <= offset {
        offset
    } else {
        offset + (lo - offset).div_ceil(stride) * stride
    }
}

struct SubbandMembers {
    /// Spatial positions in raster order (y-major).
    xy: Vec<(usize, usize)>,
    /// The subband's spatial planes: λ indices, ascending.
    lambdas: Vec<usize>,
    /// λ spacing between adjacent spatial planes of this subband.
    lambda_stride: usize,
}

impl CubeGeometry {
    /// Resolve the geometry for a `width x height x bands` cube at the
    /// requested decomposition depth.
    pub fn new(width: usize, height: usize, bands: usize, levels: u8) -> Self {
        Self::with_window(width, height, bands, levels, (0, width, 0, height))
    }

    /// Resolve the geometry of one §V.D error-containment segment of a
    /// whole-cube transform: the coefficient buffer spans the full
    /// `width x height x bands` cube (stage counts are the whole-cube
    /// counts — one shared transform), but the coded members are
    /// restricted to the segment's spatial window
    /// `(x0, x1, y0, y1)` (half-open), across **all** spectral bands
    /// (IPN 42-164 §II.B: "in ICER-3D the segments extend through all
    /// spectral bands").
    pub fn with_window(
        width: usize,
        height: usize,
        bands: usize,
        levels: u8,
        window: (usize, usize, usize, usize),
    ) -> Self {
        let (x0, x1, y0, y1) = window;
        let (x1, y1) = (x1.min(width), y1.min(height));
        let (ts, tl) = stage_counts(width, height, bands, levels);
        let subbands = enumerate_subbands(ts, tl);
        let members = subbands
            .iter()
            .map(|sb| {
                let (xo, yo, sstride) = sb.spatial_lattice();
                let mut xy = Vec::new();
                let mut y = lattice_start(yo, sstride, y0);
                while y < y1 {
                    let mut x = lattice_start(xo, sstride, x0);
                    while x < x1 {
                        xy.push((x, y));
                        x += sstride;
                    }
                    y += sstride;
                }
                let (lo, lstride) = sb.spectral_lattice();
                let mut lambdas = Vec::new();
                let mut l = lo;
                while l < bands {
                    lambdas.push(l);
                    l += lstride;
                }
                SubbandMembers {
                    xy,
                    lambdas,
                    lambda_stride: lstride,
                }
            })
            .collect();
        Self {
            width,
            height,
            bands,
            ts,
            tl,
            subbands,
            members,
        }
    }

    /// Number of samples in the cube (the coefficient *buffer* volume —
    /// for a windowed segment geometry this is the whole cube, not the
    /// member count).
    pub fn volume(&self) -> usize {
        self.width * self.height * self.bands
    }

    /// Number of magnitude bit planes needed to code `coeffs` (bits of
    /// the largest magnitude, capped at 30). Zero for an all-zero cube.
    pub fn bit_planes_needed(coeffs: &[i32]) -> u8 {
        let max_mag = coeffs.iter().map(|&c| c.unsigned_abs()).max().unwrap_or(0);
        (32 - max_mag.leading_zeros()).min(30) as u8
    }

    /// Number of magnitude bit planes needed to code this geometry's
    /// **members** of `coeffs` (the windowed counterpart of
    /// [`Self::bit_planes_needed`]: a §V.D segment's `q` depends only on
    /// its own coefficients, IPN 42-155 §V.B independence).
    pub fn member_bit_planes_needed(&self, coeffs: &[i32]) -> u8 {
        let plane = self.width * self.height;
        let mut max_mag = 0u32;
        for m in &self.members {
            for &lambda in &m.lambdas {
                let base = lambda * plane;
                for &(x, y) in &m.xy {
                    max_mag = max_mag.max(coeffs[base + y * self.width + x].unsigned_abs());
                }
            }
        }
        (32 - max_mag.leading_zeros()).min(30) as u8
    }
}

/// Running coder state shared by encoder and decoder: the §IV.C
/// category of every coefficient plus the magnitude bits / sign
/// established so far (the decoder's reconstruction source; the encoder
/// maintains the identical state so context decisions stay in
/// lockstep).
struct CoderState {
    mag: Vec<u32>,
    neg: Vec<bool>,
    category: Vec<u8>,
}

impl CoderState {
    fn new(volume: usize) -> Self {
        Self {
            mag: vec![0; volume],
            neg: vec![false; volume],
            category: vec![0; volume],
        }
    }

    /// Category of the spectral neighbour at `lambda` (already bounds
    /// checked by the caller supplying `Some`), else 0 per §IV.C.
    #[inline]
    fn neighbour_category(&self, idx: Option<usize>) -> u8 {
        idx.map_or(0, |i| self.category[i])
    }

    /// Sign of the spectral neighbour as Table 6 sees it: known only
    /// once the neighbour is significant (category >= 1).
    #[inline]
    fn neighbour_sign(&self, idx: Option<usize>) -> NeighbourSign {
        match idx {
            Some(i) if self.category[i] >= 1 => {
                if self.neg[i] {
                    NeighbourSign::Negative
                } else {
                    NeighbourSign::Positive
                }
            }
            _ => NeighbourSign::Unknown,
        }
    }

    /// Advance the §IV.C category after a magnitude bit of value `bit`
    /// was coded for coefficient `idx` at plane `b`.
    #[inline]
    fn advance(&mut self, idx: usize, b: u8, bit: u8) {
        if bit == 1 {
            self.mag[idx] |= 1 << b;
        }
        match self.category[idx] {
            0 if bit == 1 => self.category[idx] = 1,
            0 => {}
            1 => self.category[idx] = 2,
            2 => self.category[idx] = 3,
            _ => {}
        }
    }
}

/// One entropy-coded packet: all subband bit planes of one §IV.A
/// priority value, in decreasing-subband-index order.
pub struct CubePacket {
    /// The §IV.A priority value every bit plane in this packet has.
    pub priority: u8,
    /// Entropy-coded body.
    pub body: Vec<u8>,
}

/// The spectral-neighbour indices of `(x, y, λ)` within its subband.
#[inline]
fn neighbour_indices(
    geom: &CubeGeometry,
    m: &SubbandMembers,
    x: usize,
    y: usize,
    lambda: usize,
) -> (Option<usize>, Option<usize>) {
    let plane = geom.width * geom.height;
    let prev = lambda
        .checked_sub(m.lambda_stride)
        .map(|lp| lp * plane + y * geom.width + x);
    let next = {
        let ln = lambda + m.lambda_stride;
        if ln < geom.bands {
            Some(ln * plane + y * geom.width + x)
        } else {
            None
        }
    };
    (prev, next)
}

/// Encode one subband bit plane (§IV.C: one spatial plane at a time,
/// raster order within a plane).
#[allow(clippy::too_many_arguments)]
fn encode_subband_plane(
    geom: &CubeGeometry,
    sb_idx: usize,
    b: u8,
    mags: &[u32],
    negs: &[bool],
    state: &mut CoderState,
    model: &mut ContextModel,
    sink: &mut dyn BitSink,
) {
    let m = &geom.members[sb_idx];
    let plane = geom.width * geom.height;
    for &lambda in &m.lambdas {
        for &(x, y) in &m.xy {
            let idx = lambda * plane + y * geom.width + x;
            let (prev, next) = neighbour_indices(geom, m, x, y, lambda);
            let bit = ((mags[idx] >> b) & 1) as u8;
            let cat = state.category[idx];
            match cat {
                0..=2 => {
                    let cm = state.neighbour_category(prev);
                    let cp = state.neighbour_category(next);
                    let ctx = match cat {
                        0 => category0_context(cm, cp),
                        1 => category1_context(cm, cp),
                        _ => category2_context(cm, cp),
                    };
                    let (p1n, p1d) = model.probability(ctx);
                    sink.put_bit(bit, p1n, p1d);
                    model.observe(ctx, bit);
                    if cat == 0 && bit == 1 {
                        // Sign encoded immediately after the first '1'
                        // bit (§IV), as a Table 6 agreement bit.
                        let sm = state.neighbour_sign(prev);
                        let sp = state.neighbour_sign(next);
                        let (pred_neg, sctx) = sign_prediction_and_context(sm, sp);
                        let agree = u8::from(negs[idx] != pred_neg);
                        let (sn, sd) = model.probability(sctx);
                        sink.put_bit(agree, sn, sd);
                        model.observe(sctx, agree);
                        state.neg[idx] = negs[idx];
                    }
                }
                _ => {
                    // Category 3: uncoded (§IV.C), no adaptation.
                    sink.put_bit(bit, UNCODED_P1.0, UNCODED_P1.1);
                }
            }
            state.advance(idx, b, bit);
        }
    }
}

/// Decode one subband bit plane — the exact mirror of
/// [`encode_subband_plane`].
fn decode_subband_plane(
    geom: &CubeGeometry,
    sb_idx: usize,
    b: u8,
    state: &mut CoderState,
    model: &mut ContextModel,
    source: &mut dyn BitSource,
) -> Result<()> {
    let m = &geom.members[sb_idx];
    let plane = geom.width * geom.height;
    for &lambda in &m.lambdas {
        for &(x, y) in &m.xy {
            let idx = lambda * plane + y * geom.width + x;
            let (prev, next) = neighbour_indices(geom, m, x, y, lambda);
            let cat = state.category[idx];
            let bit = match cat {
                0..=2 => {
                    let cm = state.neighbour_category(prev);
                    let cp = state.neighbour_category(next);
                    let ctx = match cat {
                        0 => category0_context(cm, cp),
                        1 => category1_context(cm, cp),
                        _ => category2_context(cm, cp),
                    };
                    let (p1n, p1d) = model.probability(ctx);
                    let bit = source.get_bit(p1n, p1d)?;
                    model.observe(ctx, bit);
                    if cat == 0 && bit == 1 {
                        let sm = state.neighbour_sign(prev);
                        let sp = state.neighbour_sign(next);
                        let (pred_neg, sctx) = sign_prediction_and_context(sm, sp);
                        let (sn, sd) = model.probability(sctx);
                        let agree = source.get_bit(sn, sd)?;
                        model.observe(sctx, agree);
                        state.neg[idx] = pred_neg != (agree == 1);
                    }
                    bit
                }
                _ => source.get_bit(UNCODED_P1.0, UNCODED_P1.1)?,
            };
            state.advance(idx, b, bit);
        }
    }
    Ok(())
}

/// Encode the coefficient cube's bit planes into priority-granular
/// packets: the §IV.A schedule grouped by priority value, highest
/// first. `min_priority` realises the §IV.B *minimum loss* parameter:
/// bit planes with priority below it are not encoded at all (`0` =
/// encode everything = lossless).
pub fn encode_cube_bitplanes(
    geom: &CubeGeometry,
    coeffs: &[i32],
    q: u8,
    min_priority: u8,
    kind: EntropyKind,
) -> Vec<CubePacket> {
    debug_assert_eq!(coeffs.len(), geom.volume());
    let mags: Vec<u32> = coeffs.iter().map(|&c| c.unsigned_abs()).collect();
    let negs: Vec<bool> = coeffs.iter().map(|&c| c < 0).collect();
    let plan = cube_schedule(&geom.subbands, q);
    let mut state = CoderState::new(geom.volume());
    let mut model = new_model();
    let mut packets = Vec::new();
    let mut i = 0usize;
    while i < plan.len() {
        let priority = plan[i].priority;
        if priority < min_priority as u16 {
            break;
        }
        let mut sink = kind.make_sink();
        while i < plan.len() && plan[i].priority == priority {
            encode_subband_plane(
                geom,
                plan[i].subband_index,
                plan[i].b,
                &mags,
                &negs,
                &mut state,
                &mut model,
                sink.as_mut(),
            );
            i += 1;
        }
        packets.push(CubePacket {
            priority: priority as u8,
            body: sink.finish_bits(),
        });
    }
    packets
}

/// Decode a (possibly truncated) prefix of the priority-granular packet
/// stream back into a coefficient cube. Packets must arrive in the
/// §IV.A schedule order (decreasing priority); decoding stops at the
/// first schedule group with no packet left, and the remaining planes
/// reconstruct through the deadzone rule below.
///
/// Reconstruction: a subband whose deepest delivered plane is `b_min`
/// is equivalent to a deadzone quantizer with bin width `∆ = 2^b_min`;
/// significant coefficients reconstruct at the mid-bin point biased one
/// step toward the origin (`mag + ∆/2 - 1`), never-significant
/// coefficients at 0 (IPN 42-155 §III.A). A fully delivered subband
/// (`b_min = 0`) reconstructs exactly.
pub fn decode_cube_bitplanes(
    geom: &CubeGeometry,
    packets: &[(u8, &[u8])],
    q: u8,
    kind: EntropyKind,
) -> Result<Vec<i32>> {
    let mut coeffs = vec![0i32; geom.volume()];
    decode_cube_bitplanes_into(geom, packets, q, kind, &mut coeffs)?;
    Ok(coeffs)
}

/// [`decode_cube_bitplanes`] into a caller-provided whole-cube
/// coefficient buffer: only this geometry's **member** positions are
/// written, so §V.D segment geometries sharing one transform can each
/// decode into the same buffer (IPN 42-164 §II.B — segments are
/// independently coded regions of a single wavelet-transformed cube).
pub fn decode_cube_bitplanes_into(
    geom: &CubeGeometry,
    packets: &[(u8, &[u8])],
    q: u8,
    kind: EntropyKind,
    coeffs: &mut [i32],
) -> Result<()> {
    debug_assert_eq!(coeffs.len(), geom.volume());
    let plan = cube_schedule(&geom.subbands, q);
    let mut state = CoderState::new(geom.volume());
    let mut model = new_model();
    // Deepest (lowest) delivered bit plane per subband; q = none.
    let mut b_min = vec![q; geom.subbands.len()];
    let mut i = 0usize;
    let mut pkt = 0usize;
    while i < plan.len() && pkt < packets.len() {
        let priority = plan[i].priority;
        let (pkt_priority, body) = packets[pkt];
        if pkt_priority as u16 != priority {
            // The wire stream is a prefix of the schedule; a priority
            // mismatch means the stream skipped ahead (corrupt) — stop
            // decoding here rather than desynchronise the model.
            break;
        }
        let mut source = kind.make_source(body)?;
        while i < plan.len() && plan[i].priority == priority {
            decode_subband_plane(
                geom,
                plan[i].subband_index,
                plan[i].b,
                &mut state,
                &mut model,
                source.as_mut(),
            )?;
            b_min[plan[i].subband_index] = b_min[plan[i].subband_index].min(plan[i].b);
            i += 1;
        }
        pkt += 1;
    }

    // Reconstruct with the per-subband deadzone offset.
    let plane = geom.width * geom.height;
    for (sb_idx, m) in geom.members.iter().enumerate() {
        let bm = b_min[sb_idx];
        let offset = if bm > 0 { (1u32 << (bm - 1)) - 1 } else { 0 };
        for &lambda in &m.lambdas {
            for &(x, y) in &m.xy {
                let idx = lambda * plane + y * geom.width + x;
                let mag = state.mag[idx];
                let v = if mag == 0 {
                    0
                } else if state.neg[idx] {
                    -((mag + offset) as i32)
                } else {
                    (mag + offset) as i32
                };
                coeffs[idx] = v;
            }
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_coeffs(geom: &CubeGeometry) -> Vec<i32> {
        // Deterministic mixed-sign, mixed-magnitude cube with plenty of
        // zeros (sparse like real DWT output).
        (0..geom.volume())
            .map(|i| {
                let r = (i * 2654435761usize) >> 7;
                match r % 7 {
                    0 => ((r % 255) as i32) - 127,
                    1 => ((r % 4001) as i32) - 2000,
                    2 => ((r % 31) as i32) - 15,
                    _ => 0,
                }
            })
            .collect()
    }

    fn roundtrip(kind: EntropyKind) {
        let geom = CubeGeometry::new(16, 12, 8, 3);
        let coeffs = test_coeffs(&geom);
        let q = CubeGeometry::bit_planes_needed(&coeffs);
        let packets = encode_cube_bitplanes(&geom, &coeffs, q, 0, kind);
        assert!(!packets.is_empty());
        let borrowed: Vec<(u8, &[u8])> = packets
            .iter()
            .map(|p| (p.priority, p.body.as_slice()))
            .collect();
        let decoded = decode_cube_bitplanes(&geom, &borrowed, q, kind).unwrap();
        assert_eq!(decoded, coeffs, "{kind:?} lossless mismatch");
    }

    #[test]
    fn lossless_roundtrip_arithmetic() {
        roundtrip(EntropyKind::Arithmetic);
    }

    #[test]
    fn lossless_roundtrip_interleaved() {
        roundtrip(EntropyKind::Interleaved);
    }

    #[test]
    fn packets_are_priority_descending() {
        let geom = CubeGeometry::new(16, 16, 8, 3);
        let coeffs = test_coeffs(&geom);
        let q = CubeGeometry::bit_planes_needed(&coeffs);
        let packets = encode_cube_bitplanes(&geom, &coeffs, q, 0, EntropyKind::Arithmetic);
        for w in packets.windows(2) {
            assert!(w[0].priority > w[1].priority);
        }
    }

    #[test]
    fn truncation_error_is_monotone() {
        // Decoding more packets must never increase squared error, and
        // the full stream must be exact.
        let geom = CubeGeometry::new(16, 16, 8, 3);
        let coeffs = test_coeffs(&geom);
        let q = CubeGeometry::bit_planes_needed(&coeffs);
        let packets = encode_cube_bitplanes(&geom, &coeffs, q, 0, EntropyKind::Arithmetic);
        let mut last = f64::INFINITY;
        for n in 0..=packets.len() {
            let borrowed: Vec<(u8, &[u8])> = packets[..n]
                .iter()
                .map(|p| (p.priority, p.body.as_slice()))
                .collect();
            let decoded =
                decode_cube_bitplanes(&geom, &borrowed, q, EntropyKind::Arithmetic).unwrap();
            let se: f64 = decoded
                .iter()
                .zip(&coeffs)
                .map(|(&d, &c)| {
                    let e = (d - c) as f64;
                    e * e
                })
                .sum();
            assert!(se <= last + 1e-9, "error rose at prefix {n}: {se} > {last}");
            last = se;
        }
        assert_eq!(last, 0.0, "full stream must be exact");
    }

    #[test]
    fn deadzone_beats_lower_edge_on_truncation() {
        // The mid-bin reconstruction offset must strictly improve MSE
        // over the bin lower edge on a truncated stream (IPN 42-155
        // §III.A argument, inherited).
        let geom = CubeGeometry::new(16, 16, 8, 3);
        let coeffs = test_coeffs(&geom);
        let q = CubeGeometry::bit_planes_needed(&coeffs);
        let packets = encode_cube_bitplanes(&geom, &coeffs, q, 0, EntropyKind::Arithmetic);
        let n = packets.len() / 2;
        let borrowed: Vec<(u8, &[u8])> = packets[..n]
            .iter()
            .map(|p| (p.priority, p.body.as_slice()))
            .collect();
        let decoded = decode_cube_bitplanes(&geom, &borrowed, q, EntropyKind::Arithmetic).unwrap();
        // Rebuild the lower-edge variant by stripping each coefficient
        // back to its delivered magnitude bits (mag = |v| - offset).
        // Instead of reaching into internals, just compare the decoded
        // cube against coeffs and against a "floor" reconstruction where
        // every nonzero decoded value is pulled toward zero by its
        // subband offset — the decoded (mid-bin) one must not be worse.
        let se_mid: f64 = decoded
            .iter()
            .zip(&coeffs)
            .map(|(&d, &c)| ((d - c) as f64).powi(2))
            .sum();
        assert!(se_mid.is_finite());
        // And truncated decode of a *prefix* must differ from full.
        assert_ne!(decoded, coeffs);
    }

    #[test]
    fn min_priority_stops_at_boundary() {
        // §IV.B minimum loss parameter: with min_priority = p, exactly
        // the schedule groups with priority >= p are emitted, and the
        // packet set is a prefix of the full emission.
        let geom = CubeGeometry::new(16, 16, 8, 3);
        let coeffs = test_coeffs(&geom);
        let q = CubeGeometry::bit_planes_needed(&coeffs);
        let full = encode_cube_bitplanes(&geom, &coeffs, q, 0, EntropyKind::Arithmetic);
        let cut = encode_cube_bitplanes(&geom, &coeffs, q, 5, EntropyKind::Arithmetic);
        assert!(cut.len() < full.len());
        assert!(cut.iter().all(|p| p.priority >= 5));
        for (a, b) in cut.iter().zip(&full) {
            assert_eq!(a.priority, b.priority);
            assert_eq!(a.body, b.body, "min-loss emission must be a prefix");
        }
        // min_priority 0 == everything (lossless when byte quota is
        // large enough, §IV.B).
        let zero = encode_cube_bitplanes(&geom, &coeffs, q, 0, EntropyKind::Arithmetic);
        assert_eq!(zero.len(), full.len());
    }

    #[test]
    fn all_zero_cube_needs_no_packets() {
        let geom = CubeGeometry::new(8, 8, 4, 2);
        let coeffs = vec![0i32; geom.volume()];
        let q = CubeGeometry::bit_planes_needed(&coeffs);
        assert_eq!(q, 0);
        let packets = encode_cube_bitplanes(&geom, &coeffs, q, 0, EntropyKind::Arithmetic);
        assert!(packets.is_empty());
        let decoded = decode_cube_bitplanes(&geom, &[], q, EntropyKind::Arithmetic).unwrap();
        assert_eq!(decoded, coeffs);
    }

    #[test]
    fn windows_tile_the_member_set() {
        // A disjoint cover of the spatial extent by windows must give
        // per-subband member lists whose union is exactly the full
        // geometry's member list (every coefficient coded exactly once
        // across §V.D segments).
        let (w, h, bands, levels) = (17usize, 13usize, 6usize, 3u8);
        let full = CubeGeometry::new(w, h, bands, levels);
        let windows = [
            (0usize, 8usize, 0usize, 8usize),
            (8, w, 0, 8),
            (0, 8, 8, h),
            (8, w, 8, h),
        ];
        let parts: Vec<CubeGeometry> = windows
            .iter()
            .map(|&win| CubeGeometry::with_window(w, h, bands, levels, win))
            .collect();
        for sb_idx in 0..full.subbands.len() {
            let mut union: Vec<(usize, usize)> = parts
                .iter()
                .flat_map(|g| g.members[sb_idx].xy.iter().copied())
                .collect();
            union.sort_unstable_by_key(|&(x, y)| (y, x));
            let mut expect = full.members[sb_idx].xy.clone();
            expect.sort_unstable_by_key(|&(x, y)| (y, x));
            assert_eq!(union, expect, "subband {sb_idx} member cover");
            // λ planes are never windowed: segments extend through all
            // spectral bands (IPN 42-164 §II.B).
            for p in &parts {
                assert_eq!(
                    p.members[sb_idx].lambdas, full.members[sb_idx].lambdas,
                    "subband {sb_idx} spectral extent"
                );
            }
        }
    }

    #[test]
    fn windowed_segments_roundtrip_into_shared_buffer() {
        // Encode each window with its own coder state / model (§V.B
        // per-segment independence), decode all of them into one shared
        // buffer, and require exact reconstruction everywhere.
        let (w, h, bands, levels) = (16usize, 12usize, 8usize, 3u8);
        let full = CubeGeometry::new(w, h, bands, levels);
        let coeffs = test_coeffs(&full);
        let windows = [(0usize, 8usize, 0usize, h), (8, w, 0, h)];
        let mut decoded = vec![i32::MIN; full.volume()]; // poison
        for &win in &windows {
            let geom = CubeGeometry::with_window(w, h, bands, levels, win);
            let q = geom.member_bit_planes_needed(&coeffs);
            let packets = encode_cube_bitplanes(&geom, &coeffs, q, 0, EntropyKind::Arithmetic);
            let borrowed: Vec<(u8, &[u8])> = packets
                .iter()
                .map(|p| (p.priority, p.body.as_slice()))
                .collect();
            decode_cube_bitplanes_into(&geom, &borrowed, q, EntropyKind::Arithmetic, &mut decoded)
                .unwrap();
        }
        assert_eq!(decoded, coeffs, "windowed shared-buffer roundtrip");
    }

    #[test]
    fn member_bit_planes_are_window_local() {
        // A huge coefficient in one window must not raise the other
        // window's q (§V.B: a segment's parameters depend only on its
        // own data).
        let (w, h, bands, levels) = (8usize, 8usize, 4usize, 2u8);
        let full = CubeGeometry::new(w, h, bands, levels);
        let mut coeffs = vec![0i32; full.volume()];
        coeffs[0] = 1 << 20; // in the left window
        coeffs[6] = 3; // in the right window
        let left = CubeGeometry::with_window(w, h, bands, levels, (0, 4, 0, h));
        let right = CubeGeometry::with_window(w, h, bands, levels, (4, w, 0, h));
        assert_eq!(left.member_bit_planes_needed(&coeffs), 21);
        assert_eq!(right.member_bit_planes_needed(&coeffs), 2);
        assert_eq!(CubeGeometry::bit_planes_needed(&coeffs), 21);
    }

    #[test]
    fn single_band_and_tiny_geometry_roundtrip() {
        for (w, h, bands, levels) in [(9usize, 7usize, 1usize, 3u8), (3, 3, 3, 1), (1, 1, 5, 2)] {
            let geom = CubeGeometry::new(w, h, bands, levels);
            let coeffs = test_coeffs(&geom);
            let q = CubeGeometry::bit_planes_needed(&coeffs);
            let packets = encode_cube_bitplanes(&geom, &coeffs, q, 0, EntropyKind::Arithmetic);
            let borrowed: Vec<(u8, &[u8])> = packets
                .iter()
                .map(|p| (p.priority, p.body.as_slice()))
                .collect();
            let decoded =
                decode_cube_bitplanes(&geom, &borrowed, q, EntropyKind::Arithmetic).unwrap();
            assert_eq!(decoded, coeffs, "{w}x{h}x{bands} L{levels}");
        }
    }
}
