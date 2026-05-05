//! Bit-plane scanner — significance, refinement, and sign passes that
//! drive the binary arithmetic coder.
//!
//! ICER's compressed segment body is built MSB-down by walking each
//! bit-plane of the wavelet coefficient buffer through three passes
//! (IPN 42-155 §III.B):
//!
//!   1. **Significance** — for every coefficient still insignificant,
//!      send a single bit indicating whether bit `bp` is the first
//!      magnitude bit set; if so, also send the sign bit.
//!   2. **Refinement** — for every coefficient that became significant
//!      in an earlier bit-plane, send bit `bp` of its magnitude.
//!   3. **(Cleanup)** — IPN 42-155 §III.B notes that ICER (unlike
//!      JPEG 2000 EBCOT) folds the cleanup pass into the significance
//!      pass for some configurations; this implementation uses the
//!      merged variant (significance covers cleanup), matching the
//!      paper's "two-pass per bit-plane" reading of §III.B.
//!
//! The scan order inside each pass is straight raster (top-down,
//! left-to-right). The paper's stripe-based ordering is an
//! optimisation that reduces context-pattern cache misses but does
//! not change which bits get coded — so a self-roundtrip test
//! validates the scheme end-to-end. Real Mars-rover interop will
//! require switching to the stripe order in a follow-up; the
//! `BitPlaneInput` interface below is structured so that change is
//! local.
//!
//! ## Self-contained codec
//!
//! [`encode_bitplanes`] takes a coefficient buffer + a target
//! bit-plane count `q` and produces a single arithmetic-coded byte
//! stream that [`decode_bitplanes`] reconstructs bit-for-bit.
//!
//! No JPL flight code or third-party ICER reference was consulted;
//! the scan order + context-selection logic is the canonical EBCOT
//! shape adapted to the 17-context union table this crate defines
//! in [`crate::context`].

use crate::arith::{ArithDecoder, ArithEncoder};
use crate::context::{
    refinement_context, sign_context, significance_context, ContextModel, CONTEXT_COUNT,
};
use crate::error::{IcerError, Result};

/// Describes one wavelet coefficient sub-band's bit-plane scan input.
/// `coeffs` holds the signed wavelet coefficients in raster scan
/// order (`width * height` samples). `q` is the bit-plane count from
/// MSB to LSB inclusive — bit-plane index 0 is the MSB of the largest
/// magnitude in the buffer.
#[derive(Debug)]
pub struct BitPlaneInput<'a> {
    pub coeffs: &'a [i32],
    pub width: usize,
    pub height: usize,
    pub q: u8,
}

impl BitPlaneInput<'_> {
    fn validate(&self) -> Result<()> {
        if self.coeffs.len() != self.width * self.height {
            return Err(IcerError::invalid(format!(
                "bit-plane input: {} coeffs but width*height = {}",
                self.coeffs.len(),
                self.width * self.height
            )));
        }
        if self.q == 0 || self.q > 31 {
            return Err(IcerError::invalid(format!(
                "bit-plane count {} outside (0,31]",
                self.q
            )));
        }
        Ok(())
    }
}

/// Encode a coefficient buffer into a single arithmetic-coded payload.
///
/// Returns the entropy-coded body. The caller wraps this in a
/// `PacketHeader` (one packet per call is the simplest packetisation
/// choice; multi-packet ordering is a follow-up).
pub fn encode_bitplanes(input: &BitPlaneInput<'_>) -> Result<Vec<u8>> {
    input.validate()?;
    let n = input.coeffs.len();
    let q = input.q as usize;

    // Significance state (true once magnitude has been emitted).
    let mut significant = vec![false; n];
    // Sign state (true = negative). Only meaningful once `significant`.
    let mut sign = vec![false; n];

    let mut model = ContextModel::new();
    let mut enc = ArithEncoder::new();

    for bp in (0..q).rev() {
        // --- Significance + sign pass (also covers cleanup) ---
        for y in 0..input.height {
            for x in 0..input.width {
                let i = y * input.width + x;
                if significant[i] {
                    continue;
                }
                let pat =
                    neighbour_significance_pattern(&significant, input.width, input.height, x, y);
                let ctx = significance_context(pat);
                debug_assert!(ctx < CONTEXT_COUNT);

                let mag = input.coeffs[i].unsigned_abs();
                let bit = ((mag >> bp) & 1) as u8;

                let (num, den) = model.probability(ctx);
                enc.encode_bit(bit, num, den);
                model.observe(ctx, bit);

                if bit == 1 {
                    significant[i] = true;
                    sign[i] = input.coeffs[i] < 0;

                    // Emit sign bit using a sign-context.
                    let (h_pat, v_pat) = neighbour_sign_pattern(
                        &significant,
                        &sign,
                        input.width,
                        input.height,
                        x,
                        y,
                    );
                    let sctx = sign_context(h_pat, v_pat);
                    debug_assert!(sctx < CONTEXT_COUNT);
                    let sign_bit = u8::from(sign[i]);
                    let (sn, sd) = model.probability(sctx);
                    enc.encode_bit(sign_bit, sn, sd);
                    model.observe(sctx, sign_bit);
                }
            }
        }

        // --- Refinement pass ---
        for y in 0..input.height {
            for x in 0..input.width {
                let i = y * input.width + x;
                // A coefficient is refined only if it was significant
                // *before* this bit-plane started. We detect that by
                // skipping any coefficient whose top significant bit is
                // exactly `bp` (just-became-significant in the
                // significance pass we ran moments ago).
                if !significant[i] {
                    continue;
                }
                let mag = input.coeffs[i].unsigned_abs();
                if highest_set_bit(mag) == Some(bp as u32) {
                    continue;
                }
                let just_sig = false;
                let has_sig_neighbour =
                    neighbour_significance_pattern(&significant, input.width, input.height, x, y)
                        != 0;
                let rctx = refinement_context(just_sig, has_sig_neighbour);
                debug_assert!(rctx < CONTEXT_COUNT);
                let bit = ((mag >> bp) & 1) as u8;
                let (num, den) = model.probability(rctx);
                enc.encode_bit(bit, num, den);
                model.observe(rctx, bit);
            }
        }
    }

    Ok(enc.finish())
}

/// Decode a coefficient buffer from a single arithmetic-coded payload.
/// Reconstructs `width * height` signed integers with magnitudes up to
/// `(1 << q) - 1`.
pub fn decode_bitplanes(bytes: &[u8], width: usize, height: usize, q: u8) -> Result<Vec<i32>> {
    let n = width * height;
    if q == 0 || q > 31 {
        return Err(IcerError::invalid(format!(
            "bit-plane count {q} outside (0,31]"
        )));
    }
    let q = q as usize;

    let mut significant = vec![false; n];
    let mut sign = vec![false; n];
    let mut mag = vec![0u32; n];

    let mut model = ContextModel::new();
    let mut dec = ArithDecoder::new(bytes)?;

    for bp in (0..q).rev() {
        // --- Significance + sign pass ---
        for y in 0..height {
            for x in 0..width {
                let i = y * width + x;
                if significant[i] {
                    continue;
                }
                let pat = neighbour_significance_pattern(&significant, width, height, x, y);
                let ctx = significance_context(pat);
                let (num, den) = model.probability(ctx);
                let bit = dec.decode_bit(num, den)?;
                model.observe(ctx, bit);

                if bit == 1 {
                    significant[i] = true;
                    mag[i] |= 1u32 << bp;
                    let (h_pat, v_pat) =
                        neighbour_sign_pattern(&significant, &sign, width, height, x, y);
                    let sctx = sign_context(h_pat, v_pat);
                    let (sn, sd) = model.probability(sctx);
                    let sign_bit = dec.decode_bit(sn, sd)?;
                    model.observe(sctx, sign_bit);
                    sign[i] = sign_bit == 1;
                }
            }
        }

        // --- Refinement pass ---
        for y in 0..height {
            for x in 0..width {
                let i = y * width + x;
                if !significant[i] {
                    continue;
                }
                if highest_set_bit(mag[i]) == Some(bp as u32) {
                    continue;
                }
                let has_sig_neighbour =
                    neighbour_significance_pattern(&significant, width, height, x, y) != 0;
                let rctx = refinement_context(false, has_sig_neighbour);
                let (num, den) = model.probability(rctx);
                let bit = dec.decode_bit(num, den)?;
                model.observe(rctx, bit);
                if bit == 1 {
                    mag[i] |= 1u32 << bp;
                }
            }
        }
    }

    let out = mag
        .iter()
        .zip(sign.iter())
        .map(|(&m, &s)| {
            let v = m as i32;
            if s {
                -v
            } else {
                v
            }
        })
        .collect();
    Ok(out)
}

/// Pack the 8-neighbour significance pattern for `(x, y)` into a `u8`
/// using the layout documented in [`crate::context::significance_context`]
/// (NW=bit0, N=bit1, NE=bit2, W=bit3, E=bit4, SW=bit5, S=bit6, SE=bit7).
fn neighbour_significance_pattern(
    significant: &[bool],
    width: usize,
    height: usize,
    x: usize,
    y: usize,
) -> u8 {
    let mut pat = 0u8;
    let neighbours = [
        // (dx, dy, bit)
        (-1isize, -1isize, 0u8),
        (0, -1, 1),
        (1, -1, 2),
        (-1, 0, 3),
        (1, 0, 4),
        (-1, 1, 5),
        (0, 1, 6),
        (1, 1, 7),
    ];
    for (dx, dy, bit) in neighbours {
        let nx = x as isize + dx;
        let ny = y as isize + dy;
        if nx >= 0 && ny >= 0 && (nx as usize) < width && (ny as usize) < height {
            let idx = (ny as usize) * width + (nx as usize);
            if significant[idx] {
                pat |= 1 << bit;
            }
        }
    }
    pat
}

/// Build the `(h_pattern, v_pattern)` packed-pair fed into
/// [`crate::context::sign_context`].
///
/// Each pattern has bits laid out
/// `[(W/N significant, W/N sign), (E/S significant, E/S sign)]`
/// to match the docstring of `sign_context`. Out-of-range neighbours
/// contribute zero significance.
fn neighbour_sign_pattern(
    significant: &[bool],
    sign: &[bool],
    width: usize,
    height: usize,
    x: usize,
    y: usize,
) -> (u8, u8) {
    let h = pair_pattern(
        significant,
        sign,
        width,
        height,
        x as isize - 1,
        y as isize,
        x as isize + 1,
        y as isize,
    );
    let v = pair_pattern(
        significant,
        sign,
        width,
        height,
        x as isize,
        y as isize - 1,
        x as isize,
        y as isize + 1,
    );
    (h, v)
}

#[allow(clippy::too_many_arguments)]
fn pair_pattern(
    significant: &[bool],
    sign: &[bool],
    width: usize,
    height: usize,
    ax: isize,
    ay: isize,
    bx: isize,
    by: isize,
) -> u8 {
    let mut p = 0u8;
    for (i, (cx, cy)) in [(ax, ay), (bx, by)].iter().enumerate() {
        if *cx >= 0 && *cy >= 0 && (*cx as usize) < width && (*cy as usize) < height {
            let idx = (*cy as usize) * width + (*cx as usize);
            if significant[idx] {
                p |= 1 << (i * 2);
                if sign[idx] {
                    p |= 1 << (i * 2 + 1);
                }
            }
        }
    }
    p
}

fn highest_set_bit(x: u32) -> Option<u32> {
    if x == 0 {
        None
    } else {
        Some(31 - x.leading_zeros())
    }
}

/// Pick the smallest `q` in `[1,31]` such that every coefficient's
/// magnitude fits in `q` bits. Used by the encoder to size the
/// bit-plane Q field of the segment header.
pub fn select_bit_plane_count(coeffs: &[i32]) -> u8 {
    let max_abs = coeffs.iter().map(|c| c.unsigned_abs()).max().unwrap_or(0);
    if max_abs == 0 {
        // All-zero buffer still gets a single bit-plane of context bits
        // so the bit-plane scanner has work to do.
        return 1;
    }
    let bits = 32 - max_abs.leading_zeros();
    bits.clamp(1, 31) as u8
}

#[cfg(test)]
mod tests {
    use super::*;

    fn lcg_signal(n: usize, range: i32, seed: u64) -> Vec<i32> {
        let mut s = seed.wrapping_mul(0x9E37_79B9_7F4A_7C15);
        (0..n)
            .map(|_| {
                s = s
                    .wrapping_mul(6364136223846793005)
                    .wrapping_add(1442695040888963407);
                let v = (s >> 33) as i32;
                v % range
            })
            .collect()
    }

    #[test]
    fn empty_neighbours_are_zero() {
        let sig = vec![false; 4];
        assert_eq!(
            neighbour_significance_pattern(&sig, 2, 2, 0, 0),
            0,
            "all-insignificant neighbourhood"
        );
    }

    #[test]
    fn highest_set_bit_basics() {
        assert_eq!(highest_set_bit(0), None);
        assert_eq!(highest_set_bit(1), Some(0));
        assert_eq!(highest_set_bit(2), Some(1));
        assert_eq!(highest_set_bit(0x8000_0000), Some(31));
    }

    #[test]
    fn select_bit_plane_count_basics() {
        assert_eq!(select_bit_plane_count(&[0, 0, 0]), 1);
        assert_eq!(select_bit_plane_count(&[1, 0, -1]), 1);
        assert_eq!(select_bit_plane_count(&[3, -2]), 2);
        assert_eq!(select_bit_plane_count(&[255, -255]), 8);
    }

    #[test]
    fn bitplane_roundtrip_tiny_zero() {
        let coeffs = vec![0i32; 16];
        let input = BitPlaneInput {
            coeffs: &coeffs,
            width: 4,
            height: 4,
            q: 1,
        };
        let bytes = encode_bitplanes(&input).unwrap();
        let out = decode_bitplanes(&bytes, 4, 4, 1).unwrap();
        assert_eq!(out, coeffs);
    }

    #[test]
    fn bitplane_roundtrip_signed_random() {
        let w = 8;
        let h = 6;
        let coeffs = lcg_signal(w * h, 32, 0xCAFE);
        let q = select_bit_plane_count(&coeffs);
        let input = BitPlaneInput {
            coeffs: &coeffs,
            width: w,
            height: h,
            q,
        };
        let bytes = encode_bitplanes(&input).unwrap();
        let out = decode_bitplanes(&bytes, w, h, q).unwrap();
        assert_eq!(out, coeffs);
    }

    #[test]
    fn bitplane_roundtrip_large_dynamic_range() {
        let w = 16;
        let h = 16;
        let coeffs = lcg_signal(w * h, 4096, 0xBEEF);
        let q = select_bit_plane_count(&coeffs);
        let input = BitPlaneInput {
            coeffs: &coeffs,
            width: w,
            height: h,
            q,
        };
        let bytes = encode_bitplanes(&input).unwrap();
        let out = decode_bitplanes(&bytes, w, h, q).unwrap();
        assert_eq!(out, coeffs);
    }
}
