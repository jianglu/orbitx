//! N-body gravitational force calculation and J2/J3/J4 zonal harmonics.
//!
//! Mirrors Orbiter's `Psys.cpp` functions:
//! - `SingleGacc` (Psys.cpp:668): single-body point-mass acceleration
//! - `Gacc_intermediate` (Psys.cpp:697): N-body summation
//! - `SingleGacc_perturbation` J-coeff branch (Psys.cpp:619-664): zonal harmonics

use orbitx_math::consts::GGRAV;
use orbitx_math::{cross, Vec3};

/// A gravitational body with position, mass, and optional nonspherical model.
#[derive(Clone, Debug)]
pub struct GravBody {
    /// Position in global frame [m].
    pub pos: Vec3,
    /// Mass [kg].
    pub mass: f64,
    /// Mean radius [m] (used for J-coeff reference radius).
    pub size: f64,
    /// Optional J2, J3, J4, ... zonal coefficients (jcoeff[0] = J2).
    pub jcoeff: Vec<f64>,
}

impl GravBody {
    /// Convenience: G*mass.
    pub fn gm(&self) -> f64 {
        GGRAV * self.mass
    }
}

/// Single-body point-mass gravitational acceleration.
///
/// `rpos` = position of the gravitating body relative to the test point
/// (i.e. `body.pos - test.pos`). Returns acceleration toward the body.
///
/// Mirrors `SingleGacc` (Psys.cpp:668) point-mass part.
pub fn single_gacc(rpos: Vec3, gm: f64) -> Vec3 {
    let d = rpos.length();
    rpos * (gm / (d * d * d))
}

/// N-body gravitational acceleration at position `gpos` from a list of bodies.
///
/// `exclude` optionally skips a body by index (e.g. the body being integrated).
///
/// Mirrors `Gacc_intermediate` (Psys.cpp:697).
pub fn gacc_nbody(gpos: Vec3, bodies: &[GravBody], exclude: Option<usize>) -> Vec3 {
    let mut acc = Vec3::ZERO;
    for (i, body) in bodies.iter().enumerate() {
        if Some(i) == exclude {
            continue;
        }
        let rpos = body.pos - gpos;
        // Point mass + optional J-coeff perturbation.
        acc += single_gacc(rpos, body.gm());
        if !body.jcoeff.is_empty() {
            acc += jcoeff_perturbation(rpos, body.size, body.gm(), &body.jcoeff);
        }
    }
    acc
}

/// J2/J3/J4 zonal harmonic perturbation acceleration.
///
/// Mirrors `SingleGacc_perturbation` J-coeff branch (Psys.cpp:619-664).
///
/// `rpos` = position of body relative to test point (global frame).
/// `body_size` = reference radius [m]. `gm` = G*mass. `jcoeff` = [J2, J3, J4, ...].
pub fn jcoeff_perturbation(rpos: Vec3, body_size: f64, gm: f64, jcoeff: &[f64]) -> Vec3 {
    if jcoeff.is_empty() {
        return Vec3::ZERO;
    }

    const EPS: f64 = 1e-10;

    let d = rpos.length();
    let rr = body_size / d;
    let rrn = rr * rr; // (R/r)^2

    let j2_rrn = jcoeff[0] * rrn;
    if j2_rrn.abs() <= EPS {
        return Vec3::ZERO;
    }

    let er = rpos.unit();
    // Latitude in Orbiter's frame: lat = asin(-loc.y), where loc is body-frame position.
    // For the perturbation, Orbiter uses the global-frame y component directly
    // (assuming the body's rotation axis is approximately the y-axis).
    let slat = -rpos.y / d;
    let clat = (1.0 - slat * slat).sqrt();

    let mut gacc_r = 1.5 * j2_rrn * (1.0 - 3.0 * slat * slat);
    let mut gacc_p = 3.0 * j2_rrn * clat * slat;

    if jcoeff.len() > 1 {
        let j3_rr3 = jcoeff[1] * rr * rrn;
        if j3_rr3.abs() > EPS {
            gacc_r += 2.0 * j3_rr3 * slat * (3.0 - 5.0 * slat * slat);
            gacc_p += 1.5 * j3_rr3 * clat * (-1.0 + 5.0 * slat * slat);
        }
    }

    if jcoeff.len() > 2 {
        let j4_rr4 = jcoeff[2] * rrn * rrn;
        if j4_rr4.abs() > EPS {
            gacc_r += -0.625 * j4_rr4 * (3.0 + slat * slat * (-30.0 + 35.0 * slat * slat));
            gacc_p += 2.5 * j4_rr4 * clat * slat * (-3.0 + 7.0 * slat * slat);
        }
    }

    let t0 = gm / (d * d);

    // Polar unit vector: perpendicular to er, in the plane containing er and y-axis.
    // ep = crossp(er, RotAxis) normalized, where RotAxis ≈ y-axis.
    // For simplicity (matching Orbiter's approximation), ep points in the
    // direction of increasing latitude.
    let ey = Vec3::new(0.0, 1.0, 0.0);
    let ep_cross = cross(er, ey);
    let ep = if ep_cross.length() > EPS {
        // The polar direction is perpendicular to er, toward the y-axis projection.
        let proj = er * dot(er, ey);
        (ey - proj).unit()
    } else {
        ey
    };

    er * (t0 * gacc_r) + ep * (t0 * gacc_p)
}

// Re-export dot for use in jcoeff_perturbation.
use orbitx_math::dot;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn single_body_acceleration() {
        // Body at origin, test point at (r, 0, 0).
        let r = 1.0e7;
        let gm = 3.986e14;
        let rpos = Vec3::new(r, 0.0, 0.0);
        let acc = single_gacc(rpos, gm);
        // Expected: rpos * GM/r^3 = (GM/r^2, 0, 0)
        let expected = gm / (r * r);
        assert!((acc.x - expected).abs() / expected < 1e-12);
        assert!(acc.y.abs() < 1e-6);
        assert!(acc.z.abs() < 1e-6);
    }

    #[test]
    fn nbody_two_bodies() {
        let bodies = vec![
            GravBody {
                pos: Vec3::ZERO,
                mass: 5.97e24,
                size: 6.371e6,
                jcoeff: vec![],
            },
            GravBody {
                pos: Vec3::new(3.84e8, 0.0, 0.0),
                mass: 7.35e22,
                size: 1.74e6,
                jcoeff: vec![],
            },
        ];
        let gpos = Vec3::new(1.0e7, 0.0, 0.0);
        let acc = gacc_nbody(gpos, &bodies, None);
        // Should be dominated by Earth's gravity.
        assert!(acc.x < 0.0); // toward origin
    }

    #[test]
    fn j2_perturbation_direction() {
        // J2 perturbation should be nonzero off-equator, zero on equator.
        let gm = 3.986e14;
        let size = 6.371e6;
        let jcoeff = vec![1.0826e-3]; // Earth J2

        // At equator (y=0): J2 perturbation in y should be zero.
        let rpos_eq = Vec3::new(7.0e6, 0.0, 0.0);
        let acc_eq = jcoeff_perturbation(rpos_eq, size, gm, &jcoeff);
        assert!(acc_eq.y.abs() < 1e-6, "J2 at equator y = {}", acc_eq.y);

        // At pole (x=0, y=r): J2 perturbation should be nonzero.
        let rpos_pole = Vec3::new(0.0, 7.0e6, 0.0);
        let acc_pole = jcoeff_perturbation(rpos_pole, size, gm, &jcoeff);
        assert!(acc_pole.y.abs() > 0.01, "J2 at pole y = {}", acc_pole.y);
    }
}
