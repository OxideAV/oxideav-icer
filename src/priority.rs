//! Subband quantization priority factors -- IPN 42-155 §III.A
//! "Subband Quantization and Priority Factors".
//!
//! ICER compresses subband bit planes most-significant first, but it
//! does *not* simply walk one subband to completion before starting the
//! next. Bit planes from different subbands are interleaved: after each
//! subband bit plane is finished, ICER chooses the next subband bit
//! plane to compress according to a fixed **priority** scheme (§III.A).
//! The intent is to spend the next compressed bit on whichever subband
//! bit plane yields the largest reduction in reconstructed-image
//! distortion per bit, so a truncated stream is as good as it can be at
//! every cut point.
//!
//! # Priority weights (§III.A, Fig. 7)
//!
//! Because the wavelet transforms ICER uses are not unitary, mean-square
//! error measured in the transform domain is not equal to MSE of the
//! reconstructed image. §III.A scales the transform to an approximately
//! unitary form (`l~[n] = sqrt(2) l[n]`, `h~[n] = (1/sqrt(2)) h[n]`) and
//! reads off, from the resulting per-pixel weights, the relative effect
//! a subband's pixels have on reconstructed-image RMS distortion. Those
//! per-subband weights are the ones plotted in Fig. 7.
//!
//! For a `D`-stage dyadic decomposition the subbands and their Fig. 7
//! weights follow a clean closed form. Writing the decomposition
//! *level* `j` of a subband as `1` for the first (coarsest-filtered,
//! largest, outermost) stage up to `D` for the last (innermost,
//! smallest) stage:
//!
//! ```text
//!     w(HH_j)        = 2^(j-2)
//!     w(HL_j) = w(LH_j) = 2^(j-1)
//!     w(LL_D)        = 2^D          (LL exists only at the deepest level)
//! ```
//!
//! For `D = 3` this reproduces Fig. 7 exactly:
//!
//! | level | LL | HL | LH | HH  |
//! |-------|---:|---:|---:|----:|
//! | 1     |  - |  1 |  1 | 1/2 |
//! | 2     |  - |  2 |  2 |  1  |
//! | 3     |  8 |  4 |  4 |  2  |
//!
//! The two worked examples §III.A gives both fall out of this form:
//!
//! * "a pixel in the LL subband has a factor of 16 higher priority
//!   weight than a pixel in the level-1 HH subband" (D = 3):
//!   `w(LL_3) / w(HH_1) = 8 / (1/2) = 16`.
//! * "the `i`th least-significant bit plane of the level-1 HH subband
//!   has priority equal to that of the `(i+4)`th least-significant bit
//!   plane of the LL subband": each additional bit plane reduces a
//!   subband's RMS distortion by roughly a factor of 2, so a bit plane's
//!   priority is its subband weight times `2^(-k)` where `k` counts bit
//!   planes down from the most significant. The four-bit-plane offset is
//!   exactly `log2(16)`.
//!
//! # Encode order (§III.A)
//!
//! ICER encodes subband bit planes in order of decreasing priority.
//! §III.A pins the tie-breaks precisely:
//!
//! 1. higher priority weight first;
//! 2. when weights tie, the subband with the **higher decomposition
//!    level** first;
//! 3. when the level also ties, by subband **type** in the order
//!    `LL, HL, LH, HH`.
//!
//! Because every additional bit plane halves the priority, the
//! priorities live on a `log2` scale; this module represents a bit
//! plane's priority by an integer `log2`-priority so the ordering is
//! exact (no floating-point comparison). The most-significant magnitude
//! bit plane of a subband is `bp_from_msb = 0`.
//!
//! This module is the clean-room §III.A model. It produces the cross-
//! subband interleaving order; the bit-plane coder (`crate::bitplane`)
//! and the packet emitter consume that order. It deliberately has no
//! dependency on the entropy coder or the wire framing -- it is pure
//! arithmetic over the subband geometry.

/// One of the four subband types produced by a 2-D wavelet stage. The
/// two letters give horizontal then vertical filtering: `L` = low-pass,
/// `H` = high-pass (IPN 42-155 §II.B).
///
/// `LL` only ever appears at the deepest decomposition level (the
/// pyramidal decomposition replaces the previous stage's LL with four
/// new subbands at each further stage -- §II.B).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum SubbandType {
    /// Horizontal low-pass, vertical low-pass. The coarse approximation;
    /// present only at the deepest level.
    Ll,
    /// Horizontal high-pass, vertical low-pass.
    Hl,
    /// Horizontal low-pass, vertical high-pass.
    Lh,
    /// Horizontal high-pass, vertical high-pass.
    Hh,
}

impl SubbandType {
    /// §III.A tie-break rank when both priority weight and decomposition
    /// level coincide: `LL` before `HL` before `LH` before `HH`. Lower
    /// rank encodes first.
    pub fn order_rank(self) -> u8 {
        match self {
            SubbandType::Ll => 0,
            SubbandType::Hl => 1,
            SubbandType::Lh => 2,
            SubbandType::Hh => 3,
        }
    }
}

/// One subband of a `D`-stage decomposition: its type plus the
/// decomposition level (`1` = first/outermost stage, `D` = deepest).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Subband {
    /// Subband type (`LL`/`HL`/`LH`/`HH`).
    pub kind: SubbandType,
    /// Decomposition level, `1..=D`. `LL` only occurs at level `D`.
    pub level: u8,
}

impl Subband {
    /// The Fig. 7 (§III.A) priority weight, expressed as a base-2
    /// logarithm so it stays an exact integer.
    ///
    /// From the closed form in the module docs, with level `j`:
    ///
    /// ```text
    ///     log2 w(HH_j)         = j - 2
    ///     log2 w(HL_j)/w(LH_j) = j - 1
    ///     log2 w(LL_D)         = D
    /// ```
    ///
    /// (Note `w(HH_1) = 1/2` gives a negative `log2` of `-1`, hence the
    /// signed return.)
    pub fn weight_log2(self, decomposition_levels: u8) -> i32 {
        let j = self.level as i32;
        match self.kind {
            SubbandType::Ll => decomposition_levels as i32,
            SubbandType::Hl | SubbandType::Lh => j - 1,
            SubbandType::Hh => j - 2,
        }
    }
}

/// Enumerate every subband of a `D`-stage dyadic decomposition.
///
/// §II.B: each stage replaces the previous stage's LL with four new
/// subbands, so after `D` stages there are `3*D + 1` subbands -- three
/// detail subbands (`HL`, `LH`, `HH`) at every level `1..=D`, plus the
/// single `LL` at level `D`.
///
/// `decomposition_levels` is clamped to `1..=6` to match the segment
/// header's level field (IPN 42-155 §III.A; the same `1..=6` range the
/// encoder uses).
pub fn subbands(decomposition_levels: u8) -> Vec<Subband> {
    let d = decomposition_levels.clamp(1, 6);
    let mut out = Vec::with_capacity(3 * d as usize + 1);
    // LL exists only at the deepest level.
    out.push(Subband {
        kind: SubbandType::Ll,
        level: d,
    });
    for level in 1..=d {
        for kind in [SubbandType::Hl, SubbandType::Lh, SubbandType::Hh] {
            out.push(Subband { kind, level });
        }
    }
    out
}

/// A single subband bit plane scheduled for compression, with the
/// `log2`-priority used to order it against bit planes from other
/// subbands.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SubbandBitPlane {
    /// Which subband this bit plane belongs to.
    pub subband: Subband,
    /// Bit-plane index counted down from the most-significant magnitude
    /// bit plane of this subband (`0` = MSB plane).
    pub bp_from_msb: u32,
    /// The bit plane's `log2`-priority: `weight_log2(subband) -
    /// bp_from_msb`. Higher is more urgent. Each step down a bit plane
    /// halves the priority (subtracts 1 in `log2` space) per §III.A.
    pub priority_log2: i32,
}

/// Build the cross-subband encode order for a `D`-stage decomposition
/// where each subband contributes `bit_planes` magnitude bit planes,
/// most-significant first.
///
/// The returned vector lists every `(subband, bit-plane)` pair in the
/// exact §III.A priority order:
///
/// 1. higher `priority_log2` first;
/// 2. ties broken by higher decomposition level;
/// 3. then by subband type order `LL, HL, LH, HH`.
///
/// This is the order in which ICER interleaves subband bit planes into
/// the progressive stream. Truncating the returned order at any prefix
/// yields the §III.A-optimal subset of bit planes for that cut.
///
/// `bit_planes` is the per-subband magnitude bit-plane count (the
/// segment header's `q`); `decomposition_levels` is clamped to `1..=6`.
pub fn encode_order(decomposition_levels: u8, bit_planes: u32) -> Vec<SubbandBitPlane> {
    let bands = subbands(decomposition_levels);
    let mut plan: Vec<SubbandBitPlane> = Vec::with_capacity(bands.len() * bit_planes as usize);
    for &subband in &bands {
        let w = subband.weight_log2(decomposition_levels);
        for bp in 0..bit_planes {
            plan.push(SubbandBitPlane {
                subband,
                bp_from_msb: bp,
                priority_log2: w - bp as i32,
            });
        }
    }
    // §III.A ordering: priority descending, then level descending, then
    // subband type order LL < HL < LH < HH. `sort_by` is stable, so an
    // explicit total comparator is used for every tie level.
    plan.sort_by(|a, b| {
        b.priority_log2
            .cmp(&a.priority_log2)
            .then(b.subband.level.cmp(&a.subband.level))
            .then(
                a.subband
                    .kind
                    .order_rank()
                    .cmp(&b.subband.kind.order_rank()),
            )
            .then(a.bp_from_msb.cmp(&b.bp_from_msb))
    });
    plan
}

/// One §III.A **priority group**: every subband bit plane sharing a
/// single `log2`-priority value, listed in the §III.A tie-break order
/// (higher decomposition level first, then `LL, HL, LH, HH`, then the
/// per-subband MSB-down order that equal priorities can never violate).
///
/// The priority-interleaved packet emitter cuts the progressive stream
/// at group granularity: one packet per group, so a byte-quota
/// truncation always lands on a §III.A priority boundary ("all subband
/// bit planes having some fixed priority value" -- the same boundary
/// §VI's minimum-loss parameter is defined on).
#[derive(Debug, Clone)]
pub struct PriorityGroup {
    /// The shared `log2`-priority of every unit in this group.
    pub priority_log2: i32,
    /// The subband bit planes of this group, §III.A tie-break ordered.
    pub units: Vec<SubbandBitPlane>,
}

/// Group the §III.A encode order into [`PriorityGroup`]s of equal
/// `log2`-priority, highest priority first.
///
/// For a `D`-stage decomposition with `bit_planes = q` planes per
/// subband the group priorities are exactly the contiguous range
/// `D, D-1, .., -(q)` -- `D + q + 1` groups (LL's MSB plane alone owns
/// priority `D`; level-1 HH's LSB plane alone owns `-q`). The group
/// *index* (position in the returned vector, `0` = highest priority) is
/// what a priority-interleaved packet header carries in its `bit_plane`
/// field, so both codec sides recompute the identical schedule from
/// `(D, q)` alone.
pub fn priority_groups(decomposition_levels: u8, bit_planes: u32) -> Vec<PriorityGroup> {
    let order = encode_order(decomposition_levels, bit_planes);
    let mut groups: Vec<PriorityGroup> = Vec::new();
    for unit in order {
        match groups.last_mut() {
            Some(g) if g.priority_log2 == unit.priority_log2 => g.units.push(unit),
            _ => groups.push(PriorityGroup {
                priority_log2: unit.priority_log2,
                units: vec![unit],
            }),
        }
    }
    groups
}

/// `true` when the §VI.A minimum-loss parameter `M` excludes this
/// subband bit plane from encoding entirely: the plane's absolute
/// magnitude bit position (`q - 1 - bp_from_msb`, `0` = the LSB) lies
/// below the subband's `max(0, M - offset)` excluded-LSB-plane count
/// (Fig. 18 offsets, [`min_loss_offset`]).
///
/// This is the per-*plane* form of the per-*coefficient*
/// [`min_loss_skip_map`]: within one subband every coefficient shares
/// the same exclusion, so a priority-interleaved schedule drops whole
/// units rather than filtering pixel visits. Both codec sides apply it
/// to the [`priority_groups`] schedule, keyed off the `M` replicated in
/// every packet header.
pub fn min_loss_excludes_unit(
    unit: &SubbandBitPlane,
    decomposition_levels: u8,
    bit_planes: u32,
    min_loss: u8,
) -> bool {
    if min_loss == 0 {
        return false;
    }
    let abs_bit = (bit_planes - 1 - unit.bp_from_msb) as i64;
    let excluded =
        (min_loss as i64 - min_loss_offset(unit.subband, decomposition_levels) as i64).max(0);
    abs_bit < excluded
}

/// Fine-packetisation threshold for the priority-interleaved wire
/// schedule (see [`packet_schedule`]): a subband bit plane whose
/// subband holds **more than** this many coefficients in the strip is
/// emitted as its own packet instead of sharing its priority group's
/// packet. Both papers leave packetisation to the implementation
/// (IPN 42-155 §V.A: "multiple packets may be used for a single
/// compressed segment"); this value trades per-packet header overhead
/// (4 bytes) against byte-quota truncation granularity — the fat
/// low-level subband bit planes carry most of the stream's bytes, so
/// giving them their own packets keeps a quota cut close to its exact
/// §III.A schedule position.
pub const FINE_PACKET_COEFFS: usize = 256;

/// One wire packet of the §III.A priority-interleaved schedule: the
/// index of the [`priority_groups`] group it belongs to (what the
/// packet header's `bit_plane` field carries) plus the subband bit
/// planes its body codes, in §III.A order.
#[derive(Debug, Clone)]
pub struct SchedulePacket {
    /// Index into [`priority_groups`] (`0` = highest priority).
    pub group_index: usize,
    /// The units this packet codes, in §III.A tie-break order.
    pub units: Vec<SubbandBitPlane>,
}

/// Number of coefficients of `subband` inside a `width x height`
/// Mallat-interleaved buffer (the count of its [`subband_lattice`]
/// points).
pub fn subband_coeff_count(subband: Subband, width: usize, height: usize) -> usize {
    let lat = subband_lattice(subband);
    let nx = if lat.x0 < width {
        (width - lat.x0).div_ceil(lat.step)
    } else {
        0
    };
    let ny = if lat.y0 < height {
        (height - lat.y0).div_ceil(lat.step)
    } else {
        0
    };
    nx * ny
}

/// The deterministic wire-packet schedule of a priority-interleaved
/// segment: [`priority_groups`] filtered by the §VI.A `min_loss`
/// exclusion ([`min_loss_excludes_unit`]), with every *fat* unit (its
/// subband holds more than [`FINE_PACKET_COEFFS`] coefficients in the
/// `width x height` strip) cut into its own packet. Unit order — the
/// concatenation of every packet's `units` — is exactly the §III.A
/// encode order; only the packet boundaries move. Empty groups produce
/// no packet.
///
/// Both codec sides recompute this schedule from values carried in the
/// segment + packet headers alone (`decomp_levels`, `bit_plane_count`,
/// `width`, `height`, `min_loss`), so packet contents are never
/// signalled — the header's group index is a consistency check, and a
/// truncated stream simply stops at its last delivered packet.
pub fn packet_schedule(
    decomposition_levels: u8,
    bit_planes: u32,
    min_loss: u8,
    width: usize,
    height: usize,
) -> Vec<SchedulePacket> {
    let groups = priority_groups(decomposition_levels, bit_planes);
    let mut out = Vec::new();
    for (group_index, group) in groups.iter().enumerate() {
        let mut open: Option<SchedulePacket> = None;
        for unit in &group.units {
            if min_loss_excludes_unit(unit, decomposition_levels, bit_planes, min_loss) {
                continue;
            }
            if subband_coeff_count(unit.subband, width, height) > FINE_PACKET_COEFFS {
                // Fat unit: close the open run, emit the unit alone.
                if let Some(p) = open.take() {
                    out.push(p);
                }
                out.push(SchedulePacket {
                    group_index,
                    units: vec![*unit],
                });
            } else {
                open.get_or_insert_with(|| SchedulePacket {
                    group_index,
                    units: Vec::new(),
                })
                .units
                .push(*unit);
            }
        }
        if let Some(p) = open.take() {
            out.push(p);
        }
    }
    out
}

/// The lattice a subband occupies inside the Mallat-interleaved
/// coefficient buffer: `x ≡ x0 (mod step)`, `y ≡ y0 (mod step)`.
///
/// Inverse of [`classify_position`] under the crate's
/// interleaved-sub-rectangle dyadic layout: one decomposition stage
/// interleaves low/high outputs at stride 2 per axis, so a subband at
/// decomposition level `j` lives on the stride-`2^j` sub-lattice whose
/// per-axis phase is `0` for the low-pass (`L`) axis and `2^(j-1)` for
/// the high-pass (`H`) axis (the odd index of the level-`j`
/// interleave). The deepest `LL` sits at phase `(0, 0)` with stride
/// `2^D`.
///
/// The §III raster scan of a subband bit plane ("the bits within a
/// segment are compressed in raster scan order") walks this lattice
/// row-major: `y = y0, y0+step, ..` outer, `x = x0, x0+step, ..` inner.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SubbandLattice {
    /// Horizontal phase of the lattice (`x % step == x0`).
    pub x0: usize,
    /// Vertical phase of the lattice (`y % step == y0`).
    pub y0: usize,
    /// Per-axis stride between adjacent lattice points.
    pub step: usize,
}

/// The [`SubbandLattice`] of `subband` (see the struct docs). `level`
/// must be `1..=6` per the segment-header range.
pub fn subband_lattice(subband: Subband) -> SubbandLattice {
    let j = subband.level as u32;
    let step = 1usize << j;
    let half = 1usize << (j - 1);
    let (x0, y0) = match subband.kind {
        SubbandType::Ll => (0, 0),
        SubbandType::Hl => (half, 0),
        SubbandType::Lh => (0, half),
        SubbandType::Hh => (half, half),
    };
    SubbandLattice { x0, y0, step }
}

/// Classify the transform-coefficient position `(x, y)` into its
/// `(SubbandType, level)` under the crate's interleaved-sub-rectangle
/// dyadic layout (the one produced by
/// [`crate::wavelet_int::forward_2d_dyadic`]).
///
/// The crate's dyadic transform leaves each level in Mallat-interleaved
/// form (low-pass at even indices, high-pass at odd indices per axis)
/// and then applies the next level to the **even/even low-pass
/// lattice** (stride doubling per stage — IPN 42-155 §II.B: each
/// further stage decomposes the LL subband). So a coefficient's subband
/// is found by repeatedly
/// halving: at the current level, if both coordinates are even the
/// coefficient belongs to the low/low band that the next level
/// re-transforms (descend); otherwise the parity pair `(x%2, y%2)` names
/// the detail subband (`(1,0) = HL`, `(0,1) = LH`, `(1,1) = HH`). A
/// position that stays even/even through every level is the final `LL`.
///
/// Returns `(subband_type, level)` where `level` is `1..=levels`.
pub fn classify_position(mut x: usize, mut y: usize, levels: u8) -> (SubbandType, u8) {
    let mut level = 1u8;
    loop {
        if level > levels {
            return (SubbandType::Ll, levels);
        }
        match (x % 2, y % 2) {
            (0, 0) => {
                x /= 2;
                y /= 2;
                level += 1;
            }
            (1, 0) => return (SubbandType::Hl, level),
            (0, 1) => return (SubbandType::Lh, level),
            _ => return (SubbandType::Hh, level),
        }
    }
}

/// Spacing, in the Mallat-interleaved coefficient buffer, between two
/// coefficients of the **same subband** that are nearest neighbours
/// within that subband.
///
/// # §III.B same-subband neighbourhood
///
/// IPN 42-155 §III.B specifies that a bit's coding context is "determined
/// by the bits already encoded in the pixel and in its eight nearest
/// neighbors **from the same segment of the subband**". The paper is
/// emphatic that the context is built only from pixels in the immediate
/// nine-pixel neighbourhood *within the given subband* -- not from the
/// spatially-adjacent pixels, which in the interleaved transform buffer
/// belong to *different* subbands.
///
/// In this crate's interleaved sub-rectangle dyadic layout (see
/// [`classify_position`]) one decomposition stage interleaves low/high
/// outputs at stride 2 per axis, two stages at stride 4, and so on, so two
/// coefficients of a subband at decomposition level `j` that are adjacent
/// *within that subband* sit `2^j` apart in the buffer along each axis.
/// The deepest LL (level `D`) shares the same `2^D` spacing.
///
/// So the same-subband neighbour of `(x, y)` in subband direction
/// `(dx, dy)` is the buffer position `(x + dx*stride, y + dy*stride)` with
/// `stride = subband_stride(x, y, levels)`. That neighbour classifies to
/// the *same* subband by construction; a neighbour that falls outside the
/// strip is "at the edge of its subband segment" and is treated as not yet
/// significant (§III.B).
///
/// `levels` is clamped to `1..=6` to match the rest of the crate; a
/// `levels == 0` request returns a unit stride (the legacy
/// subband-agnostic spatial-raster walk).
#[inline]
pub fn subband_stride(x: usize, y: usize, levels: u8) -> usize {
    if levels == 0 {
        return 1;
    }
    let (_, level) = classify_position(x, y, levels);
    1usize << level
}

/// §VI.A Fig. 18 relative-importance offset of a subband: "if the
/// minimum loss value minus the number shown is positive, then this
/// difference equals the number of bit planes of the subband that will
/// not be encoded no matter how large the byte quota is."
///
/// Fig. 18 is derived from the Fig. 7 priority weights (§VI.A), with
/// the level-1 HH subband (the least important) pinned at 0, so the
/// offset is the subband's `log2` weight relative to level-1 HH's
/// (`weight_log2 = -1`):
///
/// ```text
///     offset(HH_j)              = j - 1
///     offset(HL_j) = offset(LH_j) = j
///     offset(LL_D)              = D + 1
/// ```
///
/// For `D = 3` this reproduces Fig. 18 exactly (LL = 4; level-3 HL/LH =
/// 3, HH = 2; level-2 HL/LH = 2, HH = 1; level-1 HL/LH = 1, HH = 0).
pub fn min_loss_offset(subband: Subband, decomposition_levels: u8) -> u32 {
    (subband.weight_log2(decomposition_levels) + 1) as u32
}

/// Per-coefficient count of least-significant magnitude bit planes that
/// the §VI.A minimum-loss parameter `M` excludes from encoding:
/// `max(0, M - offset)` with the Fig. 18 per-subband offset.
///
/// "A minimum loss value of M means that compression will stop before
/// the last M bit planes of the level-1 HH subband are encoded.
/// Similarly, bit planes with sufficiently low importance in other
/// subbands are not encoded, taking into account the relative
/// importance of the different subbands" (§VI.A). The returned map is
/// indexed like the interleaved coefficient buffer; entry `i` is the
/// number of LSB planes never coded for coefficient `i`, so a plane at
/// magnitude bit position `bp` is coded iff `bp >= map[i]`.
///
/// `min_loss == 0` yields the all-zero map (lossless when the byte
/// quota allows — §VI.A).
pub fn min_loss_skip_map(width: usize, height: usize, levels: u8, min_loss: u8) -> Vec<u8> {
    let levels = levels.clamp(1, 6);
    let m = min_loss as i64;
    // One skip value per (SubbandType, level) class; small table.
    let mut out = vec![0u8; width * height];
    if min_loss == 0 {
        return out;
    }
    for y in 0..height {
        for x in 0..width {
            let (kind, level) = classify_position(x, y, levels);
            let off = min_loss_offset(Subband { kind, level }, levels) as i64;
            out[y * width + x] = (m - off).clamp(0, u8::MAX as i64) as u8;
        }
    }
    out
}

/// Per-coefficient image-domain distortion weight for a `levels`-stage
/// decomposition of a `width * height` strip under `filter`.
///
/// # §III.A image-domain weighting
///
/// IPN 42-155 §III.A observes that, because ICER's wavelet transforms
/// are *not* unitary, transform-domain mean-square error is **not** equal
/// to reconstructed-image MSE: "the weights shown in Fig. 7 ... indicate
/// the approximate relative effect (per pixel of the subband) on the
/// reconstructed image of root-mean-squared distortion values in the
/// subbands". Fig. 7 publishes those weights for the *idealised*
/// approximately-unitary scaled transform. The §II.A integer
/// transforms are concrete non-unitary realisations whose actual
/// per-subband effect differs from the idealised figure, so rather
/// than assume the Fig. 7 numbers literally this routine **measures**
/// the §III.A effect directly from the transform in use.
///
/// The image-domain energy injected by a unit error in transform
/// coefficient `i` is `||T^{-1} e_i||^2` — the squared norm of the
/// inverse transform of a unit basis vector at position `i`. That is
/// exactly the "effect on the reconstructed image of distortion in the
/// subband" §III.A weights quantify, computed for the implemented
/// (non-unitary) `T^{-1}` with no dependency on resolving the scrambled
/// subband identity. The energy is constant within a subband class away
/// from the strip boundary, so we evaluate one representative basis
/// vector per `(SubbandType, level)` class and broadcast it to every
/// coefficient of that class.
///
/// The returned weights multiply the per-coefficient transform-domain
/// squared-error terms in the rate-distortion packet selector
/// ([`crate::bitplane`]) so the selector optimises **reconstructed-image**
/// MSE rather than transform-domain MSE.
pub fn subband_weight_map(
    width: usize,
    height: usize,
    levels: u8,
    filter: crate::header::WaveletFilter,
) -> Vec<f64> {
    let levels = levels.clamp(1, 6);
    // One representative weight per (SubbandType, level) class. Probe a
    // near-centre coefficient of each class so the §III.A basis energy is
    // measured away from the boundary-extension transient.
    let mut class_weight: std::collections::HashMap<(SubbandType, u8), f64> =
        std::collections::HashMap::new();

    // Pick an interior probe position for each class by scanning the
    // central region once; the first interior hit per class is used.
    let margin = 4usize;
    let (x0, x1) = if width > 2 * margin {
        (margin, width - margin)
    } else {
        (0, width)
    };
    let (y0, y1) = if height > 2 * margin {
        (margin, height - margin)
    } else {
        (0, height)
    };
    'scan: for y in y0..y1 {
        for x in x0..x1 {
            let class = classify_position(x, y, levels);
            if class_weight.contains_key(&class) {
                continue;
            }
            class_weight.insert(class, basis_energy(width, height, levels, filter, x, y));
            // 3*levels detail classes + 1 LL = full set; stop once seen.
            if class_weight.len() == 3 * levels as usize + 1 {
                break 'scan;
            }
        }
    }

    // Build the per-coefficient map. Any class not hit by the interior
    // scan (tiny strips) falls back to a fresh probe at its position.
    let mut out = vec![1.0f64; width * height];
    for y in 0..height {
        for x in 0..width {
            let class = classify_position(x, y, levels);
            let w = *class_weight
                .entry(class)
                .or_insert_with(|| basis_energy(width, height, levels, filter, x, y));
            out[y * width + x] = w;
        }
    }
    out
}

/// Inverse-DWT basis energy `||T^{-1} e_i||^2` for a unit error at
/// `(px, py)`, normalised so the weight is the image-domain MSE per unit
/// transform-domain MSE (IPN 42-155 §III.A; see [`subband_weight_map`]).
fn basis_energy(
    width: usize,
    height: usize,
    levels: u8,
    filter: crate::header::WaveletFilter,
    px: usize,
    py: usize,
) -> f64 {
    // A scaled delta survives the integer-rounding steps inside the
    // §II.A reversible integer transform; the energy is normalised back
    // by the delta squared so the result is per-unit-coefficient.
    const DELTA: i32 = 64;
    let mut buf = vec![0i32; width * height];
    buf[py * width + px] = DELTA;
    crate::wavelet_int::inverse_2d_dyadic(&mut buf, width, height, levels, filter);
    let energy: f64 = buf.iter().map(|&v| (v as f64).powi(2)).sum();
    energy / ((DELTA as f64) * (DELTA as f64))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn subband_count_is_3d_plus_1() {
        for d in 1..=6u8 {
            assert_eq!(subbands(d).len(), 3 * d as usize + 1, "D={d} subband count");
        }
    }

    #[test]
    fn ll_only_at_deepest_level() {
        let bands = subbands(3);
        let ll: Vec<_> = bands.iter().filter(|s| s.kind == SubbandType::Ll).collect();
        assert_eq!(ll.len(), 1, "exactly one LL subband");
        assert_eq!(ll[0].level, 3, "LL at deepest level");
    }

    /// IPN 42-155 §III.A Fig. 7 weights for a 3-stage decomposition,
    /// expressed as `log2`: level-1 HH = 1/2 -> -1, level-1 HL/LH = 1 ->
    /// 0, level-2 HH = 1 -> 0, level-2 HL/LH = 2 -> 1, level-3 HH = 2 ->
    /// 1, level-3 HL/LH = 4 -> 2, level-3 LL = 8 -> 3.
    #[test]
    fn fig7_weights_d3() {
        let d = 3;
        let cases = [
            (SubbandType::Hh, 1u8, -1i32),
            (SubbandType::Hl, 1, 0),
            (SubbandType::Lh, 1, 0),
            (SubbandType::Hh, 2, 0),
            (SubbandType::Hl, 2, 1),
            (SubbandType::Lh, 2, 1),
            (SubbandType::Hh, 3, 1),
            (SubbandType::Hl, 3, 2),
            (SubbandType::Lh, 3, 2),
            (SubbandType::Ll, 3, 3),
        ];
        for (kind, level, expect) in cases {
            let s = Subband { kind, level };
            assert_eq!(
                s.weight_log2(d),
                expect,
                "Fig.7 weight log2 for {kind:?} level {level}"
            );
        }
    }

    /// §III.A worked example: a pixel in the LL subband has a factor of
    /// 16 higher priority weight than a pixel in the level-1 HH subband
    /// (3-stage decomposition). 16 = 2^4, so the log2 difference is 4.
    #[test]
    fn ll_is_16x_level1_hh() {
        let d = 3;
        let ll = Subband {
            kind: SubbandType::Ll,
            level: 3,
        };
        let hh1 = Subband {
            kind: SubbandType::Hh,
            level: 1,
        };
        assert_eq!(ll.weight_log2(d) - hh1.weight_log2(d), 4);
    }

    /// §III.A worked example: the `i`th LSB plane of the level-1 HH
    /// subband has priority equal to the `(i+4)`th LSB plane of the LL
    /// subband. Equivalently, an LL bit plane four positions *more*
    /// significant matches a level-1 HH bit plane in priority.
    #[test]
    fn ll_bitplane_offset_matches_hh1() {
        let order = encode_order(3, 12);
        // Find an LL plane and a level-1 HH plane with equal priority.
        let find = |kind: SubbandType, level: u8, bp: u32| {
            order
                .iter()
                .find(|p| p.subband.kind == kind && p.subband.level == level && p.bp_from_msb == bp)
                .copied()
                .unwrap()
        };
        // LL plane 4 steps below its MSB vs level-1 HH MSB plane: equal.
        let ll = find(SubbandType::Ll, 3, 4);
        let hh1 = find(SubbandType::Hh, 1, 0);
        assert_eq!(ll.priority_log2, hh1.priority_log2);
    }

    #[test]
    fn order_is_priority_descending() {
        let order = encode_order(3, 8);
        for w in order.windows(2) {
            assert!(
                w[0].priority_log2 >= w[1].priority_log2,
                "priority must be non-increasing along the encode order"
            );
        }
    }

    /// The very first thing encoded is the LL subband's most-significant
    /// bit plane -- it has the strictly highest priority for any D >= 1.
    #[test]
    fn ll_msb_is_first() {
        for d in 1..=6u8 {
            let order = encode_order(d, 6);
            assert_eq!(order[0].subband.kind, SubbandType::Ll);
            assert_eq!(order[0].subband.level, d);
            assert_eq!(order[0].bp_from_msb, 0);
        }
    }

    /// §III.A tie-break 2 + 3: among equal-priority bit planes, the
    /// higher decomposition level wins, then type order LL,HL,LH,HH.
    #[test]
    fn tie_breaks_level_then_type() {
        // D=3: level-2 HL/LH (log2 weight 1) and level-3 HH (log2
        // weight 1) all share priority 1 at bp 0. Order among them:
        // level 3 before level 2; within a level, HL before LH.
        let order = encode_order(3, 1); // one bit plane each -> bp always 0
        let prio1: Vec<_> = order
            .iter()
            .filter(|p| p.priority_log2 == 1)
            .map(|p| (p.subband.level, p.subband.kind))
            .collect();
        assert_eq!(
            prio1,
            vec![
                (3, SubbandType::Hh),
                (2, SubbandType::Hl),
                (2, SubbandType::Lh),
            ]
        );
    }

    #[test]
    fn total_plan_size() {
        let d = 4u8;
        let q = 7u32;
        let order = encode_order(d, q);
        assert_eq!(order.len(), (3 * d as usize + 1) * q as usize);
    }

    #[test]
    fn order_is_a_permutation_of_all_pairs() {
        let d = 3u8;
        let q = 5u32;
        let order = encode_order(d, q);
        let mut seen = std::collections::HashSet::new();
        for p in &order {
            assert!(
                seen.insert((p.subband.kind, p.subband.level, p.bp_from_msb)),
                "duplicate (subband, bit-plane) in encode order"
            );
        }
        assert_eq!(seen.len(), (3 * d as usize + 1) * q as usize);
    }

    #[test]
    fn levels_clamped_to_1_6() {
        assert_eq!(subbands(0).len(), subbands(1).len());
        assert_eq!(subbands(9).len(), subbands(6).len());
    }

    /// The §III.A priority groups form the contiguous descending
    /// priority range `D ..= -q` (`D + q + 1` groups), partition the
    /// full `(subband, plane)` set, and preserve the encode-order
    /// tie-breaks within each group.
    #[test]
    fn priority_groups_are_contiguous_and_partition() {
        for d in 1..=6u8 {
            for q in [1u32, 3, 8, 12] {
                let groups = priority_groups(d, q);
                assert_eq!(
                    groups.len(),
                    d as usize + q as usize + 1,
                    "D={d} q={q} group count"
                );
                assert_eq!(groups[0].priority_log2, d as i32, "top group is LL MSB");
                assert_eq!(
                    groups.last().unwrap().priority_log2,
                    -(q as i32),
                    "bottom group is level-1 HH LSB"
                );
                for w in groups.windows(2) {
                    assert_eq!(
                        w[0].priority_log2 - 1,
                        w[1].priority_log2,
                        "groups descend by exactly 1 in log2 priority"
                    );
                }
                let total: usize = groups.iter().map(|g| g.units.len()).sum();
                assert_eq!(total, (3 * d as usize + 1) * q as usize);
                // Flattening the groups reproduces the encode order.
                let flat: Vec<_> = groups.iter().flat_map(|g| g.units.iter()).collect();
                let order = encode_order(d, q);
                assert_eq!(flat.len(), order.len());
                for (a, b) in flat.iter().zip(order.iter()) {
                    assert_eq!(**a, *b, "grouping must preserve the §III.A order");
                }
            }
        }
    }

    /// §III.A worked example at group granularity: with D = 3 the LL
    /// plane 4 steps below its MSB shares a group with level-1 HH's MSB
    /// plane (the 16x / four-plane offset).
    #[test]
    fn priority_groups_pin_ll_hh1_offset() {
        let groups = priority_groups(3, 12);
        let g = groups
            .iter()
            .find(|g| {
                g.units.iter().any(|u| {
                    u.subband.kind == SubbandType::Hh && u.subband.level == 1 && u.bp_from_msb == 0
                })
            })
            .unwrap();
        assert!(
            g.units
                .iter()
                .any(|u| u.subband.kind == SubbandType::Ll && u.bp_from_msb == 4),
            "LL bp 4 must tie level-1 HH bp 0 (D=3, 16x weight)"
        );
    }

    /// `min_loss_excludes_unit` agrees with the per-coefficient
    /// `min_loss_skip_map` on every subband bit plane.
    #[test]
    fn min_loss_unit_exclusion_matches_skip_map() {
        let (w, h) = (32usize, 32usize);
        for d in 1..=4u8 {
            for m in [0u8, 1, 2, 4, 7] {
                let q = 8u32;
                let skip = min_loss_skip_map(w, h, d, m);
                for unit in encode_order(d, q) {
                    let lat = subband_lattice(unit.subband);
                    // Representative coefficient of this subband.
                    let i = lat.y0 * w + lat.x0;
                    let abs_bit = q - 1 - unit.bp_from_msb;
                    let expect = abs_bit < skip[i] as u32;
                    assert_eq!(
                        min_loss_excludes_unit(&unit, d, q, m),
                        expect,
                        "D={d} M={m} {unit:?}"
                    );
                }
            }
        }
    }

    /// `subband_lattice` is the exact inverse of `classify_position`:
    /// every buffer position lies on precisely its own subband's
    /// lattice, and walking all lattices covers the buffer once.
    #[test]
    fn subband_lattice_inverts_classify_position() {
        let (w, h) = (48usize, 40usize); // non-power-of-two on purpose
        for d in 1..=5u8 {
            let mut visited = vec![0u32; w * h];
            for subband in subbands(d) {
                let lat = subband_lattice(subband);
                let mut y = lat.y0;
                while y < h {
                    let mut x = lat.x0;
                    while x < w {
                        assert_eq!(
                            classify_position(x, y, d),
                            (subband.kind, subband.level),
                            "D={d} ({x},{y}) lattice/classify disagree"
                        );
                        visited[y * w + x] += 1;
                        x += lat.step;
                    }
                    y += lat.step;
                }
            }
            assert!(
                visited.iter().all(|&v| v == 1),
                "D={d}: lattices must tile the buffer exactly once"
            );
        }
    }

    /// `classify_position` follows the interleaved sub-rectangle dyadic
    /// layout: odd parity on either axis names a detail subband at the
    /// current level; even/even descends to the next level; all-even
    /// through every level is the final LL.
    #[test]
    fn classify_position_basic_parities() {
        let d = 3u8;
        // Level-1 detail bands: parity at level 1.
        assert_eq!(classify_position(1, 0, d), (SubbandType::Hl, 1));
        assert_eq!(classify_position(0, 1, d), (SubbandType::Lh, 1));
        assert_eq!(classify_position(1, 1, d), (SubbandType::Hh, 1));
        // (2,2) is even/even at level 1 -> descends; (1,1) in level-2
        // coords -> HH at level 2. (2,2)/2 = (1,1).
        assert_eq!(classify_position(2, 2, d), (SubbandType::Hh, 2));
        // (4,0): /2=(2,0) even/even -> /2=(1,0) -> HL level 3.
        assert_eq!(classify_position(4, 0, d), (SubbandType::Hl, 3));
        // (0,0) stays even through all levels -> LL at deepest level.
        assert_eq!(classify_position(0, 0, d), (SubbandType::Ll, d));
        // (8,8) all-even through 3 levels -> LL.
        assert_eq!(classify_position(8, 8, d), (SubbandType::Ll, d));
    }

    /// §III.B same-subband neighbour spacing: a coefficient at
    /// decomposition level `j` has its nearest same-subband neighbours
    /// `2^j` apart in the interleaved buffer, and that strided neighbour
    /// classifies into the *same* subband.
    #[test]
    fn subband_stride_matches_level_and_keeps_class() {
        let levels = 3u8;
        // A few representative coefficients across subbands/levels.
        for &(x, y) in &[
            (1usize, 0usize),
            (0, 1),
            (1, 1),
            (2, 2),
            (4, 0),
            (0, 0),
            (4, 4),
        ] {
            let (kind, level) = classify_position(x, y, levels);
            let stride = subband_stride(x, y, levels);
            assert_eq!(stride, 1usize << level, "stride is 2^level at ({x},{y})");
            // Stepping by the stride along either axis stays in-class.
            let (kx, _) = classify_position(x + stride, y, levels);
            let (ky, _) = classify_position(x, y + stride, levels);
            assert_eq!(kx, kind, "+x stride keeps subband at ({x},{y})");
            assert_eq!(ky, kind, "+y stride keeps subband at ({x},{y})");
        }
    }

    /// `levels == 0` requests the legacy unit-stride spatial walk.
    #[test]
    fn subband_stride_levels0_is_unit() {
        assert_eq!(subband_stride(3, 7, 0), 1);
        assert_eq!(subband_stride(0, 0, 0), 1);
    }

    /// §VI.A Fig. 18: relative importance of subbands for a 3-stage
    /// decomposition — LL = 4, level-3 HL/LH = 3, level-3 HH = 2,
    /// level-2 HL/LH = 2, level-2 HH = 1, level-1 HL/LH = 1, HH = 0.
    #[test]
    fn fig18_offsets_d3() {
        let d = 3u8;
        let cases = [
            (SubbandType::Ll, 3u8, 4u32),
            (SubbandType::Hl, 3, 3),
            (SubbandType::Lh, 3, 3),
            (SubbandType::Hh, 3, 2),
            (SubbandType::Hl, 2, 2),
            (SubbandType::Lh, 2, 2),
            (SubbandType::Hh, 2, 1),
            (SubbandType::Hl, 1, 1),
            (SubbandType::Lh, 1, 1),
            (SubbandType::Hh, 1, 0),
        ];
        for (kind, level, expect) in cases {
            assert_eq!(
                min_loss_offset(Subband { kind, level }, d),
                expect,
                "Fig. 18 offset for {kind:?} level {level}"
            );
        }
    }

    /// §VI.A worked examples: M = 1 excludes exactly the level-1 HH LSB
    /// plane; M = 2 excludes 2 planes of level-1 HH and 1 plane of each
    /// of level-1 LH / level-1 HL / level-2 HH.
    #[test]
    fn min_loss_skip_map_follows_fig18() {
        let (w, h, d) = (16usize, 16usize, 3u8);
        let m0 = min_loss_skip_map(w, h, d, 0);
        assert!(m0.iter().all(|&v| v == 0), "M = 0 is lossless");

        let m1 = min_loss_skip_map(w, h, d, 1);
        let m2 = min_loss_skip_map(w, h, d, 2);
        for y in 0..h {
            for x in 0..w {
                let (kind, level) = classify_position(x, y, d);
                let i = y * w + x;
                let expect1 = u8::from(kind == SubbandType::Hh && level == 1);
                assert_eq!(m1[i], expect1, "M=1 at ({x},{y}) {kind:?} L{level}");
                let expect2 = match (kind, level) {
                    (SubbandType::Hh, 1) => 2,
                    (SubbandType::Hl, 1) | (SubbandType::Lh, 1) | (SubbandType::Hh, 2) => 1,
                    _ => 0,
                };
                assert_eq!(m2[i], expect2, "M=2 at ({x},{y}) {kind:?} L{level}");
            }
        }
    }

    /// §VI.A: "When the original image has a dynamic range of B bits per
    /// pixel, D stages of wavelet decomposition are used, and M >= B + D,
    /// then little or no bit-plane information will be encoded" — the LL
    /// offset D + 1 is the largest, so M above it skips planes everywhere.
    #[test]
    fn min_loss_saturates_every_subband() {
        let (w, h, d) = (16usize, 16usize, 3u8);
        let map = min_loss_skip_map(w, h, d, 12);
        // Every coefficient skips at least 12 - (D + 1) = 8 planes.
        assert!(map.iter().all(|&v| v >= 8));
    }

    /// The §III.A weight map is well-formed (one positive weight per
    /// coefficient) and the unweighted-vs-weighted distinction is real:
    /// the deepest-level LL weight differs from the level-1 HH weight
    /// (the transform is not unitary, so the per-subband image-domain
    /// effect is not flat).
    #[test]
    fn weight_map_is_positive_and_non_flat() {
        let w = 32usize;
        let h = 32usize;
        let map = subband_weight_map(w, h, 3, crate::header::WaveletFilter::Reversible53);
        assert_eq!(map.len(), w * h);
        assert!(map.iter().all(|&v| v > 0.0), "all weights positive");
        let ll = map[8 * w + 8];
        let hh1 = map[5 * w + 5];
        assert!(
            (ll - hh1).abs() > 1e-3,
            "LL ({ll:.4}) and HH1 ({hh1:.4}) weights must differ (non-unitary)"
        );
    }
}
