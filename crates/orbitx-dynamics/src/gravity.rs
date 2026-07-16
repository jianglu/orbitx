//! N-body gravitational force calculation and J2/J3/J4 zonal harmonics.
//!
//! Mirrors Orbiter's `Psys.cpp` functions:
//! - `SingleGacc` (Psys.cpp:668): single-body point-mass acceleration
//! - `Gacc_intermediate` (Psys.cpp:697): N-body summation
//! - `SingleGacc_perturbation` J-coeff branch (Psys.cpp:619-664): zonal harmonics
//!
//! Also supports Pines spherical-harmonic perturbation (Psys.cpp:586-617)
//! via the `pines` field on `GravBody`.

use std::sync::Arc;

use orbitx_math::consts::GGRAV;
use orbitx_math::mat3::{self, Matrix3};
use orbitx_math::{cross, dot, Vec3};

use crate::pines::{PinesModel, Vec3Pines};

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
    /// Optional rotation matrix for J-coeff latitude computation.
    /// When `None`, assumes rotation axis = y-axis (backward compatible).
    pub rotation: Option<Matrix3>,
    /// Optional Pines spherical-harmonic gravity model and cutoff.
    pub pines: Option<(Arc<PinesModel>, usize)>,
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
/// Supports J-coeff perturbation (with rotation matrix if provided) and
/// Pines spherical-harmonic perturbation.
pub fn gacc_nbody(gpos: Vec3, bodies: &[GravBody], exclude: Option<usize>) -> Vec3 {
    let mut acc = Vec3::ZERO;
    for (i, body) in bodies.iter().enumerate() {
        if Some(i) == exclude {
            continue;
        }
        let rpos = body.pos - gpos;
        // Point mass.
        acc += single_gacc(rpos, body.gm());

        // J-coeff perturbation (with rotation if available).
        if !body.jcoeff.is_empty() {
            let rot = body.rotation.unwrap_or(Matrix3::IDENTITY);
            acc += jcoeff_perturbation_with_rot(rpos, body.size, body.gm(), &body.jcoeff, &rot);
        }

        // Pines perturbation.
        if let Some((ref pines, cutoff)) = body.pines {
            let rot = body.rotation.unwrap_or(Matrix3::IDENTITY);
            acc += pines_perturbation(rpos, pines, cutoff, &rot);
        }
    }
    acc
}

/// J2/J3/J4 zonal harmonic perturbation acceleration (backward compatible).
///
/// Assumes rotation axis = y-axis. For proper body-frame rotation, use
/// `jcoeff_perturbation_with_rot` instead.
pub fn jcoeff_perturbation(rpos: Vec3, body_size: f64, gm: f64, jcoeff: &[f64]) -> Vec3 {
    jcoeff_perturbation_with_rot(rpos, body_size, gm, jcoeff, &Matrix3::IDENTITY)
}

/// J2/J3/J4 zonal harmonic perturbation with body rotation matrix.
///
/// Mirrors `SingleGacc_perturbation` J-coeff branch (Psys.cpp:619-664).
///
/// When `rot` is the identity matrix, this reduces to the y-axis assumption.
/// When `rot` is the body's actual rotation matrix, the position is rotated
/// into the body frame before computing latitude, matching the C++ which
/// uses `loc = tmul(body->GRot(), er)` and `lat = asin(-loc.y)`.
pub fn jcoeff_perturbation_with_rot(
    rpos: Vec3,
    body_size: f64,
    gm: f64,
    jcoeff: &[f64],
    rot: &Matrix3,
) -> Vec3 {
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

    // Rotate into body frame to compute latitude.
    // C++: loc = tmul(body->GRot(), er); lat = asin(-loc.y)
    let loc = mat3::tmul(*rot, er);
    let slat = -loc.y;
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

    // Rotation axis in global frame.
    // C++: RotAxis() = tmul(GRot(), Vector(0,1,0))
    let rot_axis = mat3::tmul(*rot, Vec3::new(0.0, 1.0, 0.0));

    // Polar unit vector: perpendicular to er, in the plane containing er and rot_axis.
    // C++: ea = crossp(er, RotAxis()); ep = crossp(er, ea/|ea|)
    let ea = cross(er, rot_axis);
    let lea = ea.length();
    let ep = if lea > EPS {
        cross(er, ea * (1.0 / lea))
    } else {
        Vec3::ZERO
    };

    er * (t0 * gacc_r) + ep * (t0 * gacc_p)
}

/// Pines spherical-harmonic perturbation acceleration.
///
/// Mirrors `SingleGacc_perturbation` Pines branch (Psys.cpp:586-617).
///
/// Algorithm:
/// 1. Rotate position into body frame: `lpos = -tmul(rot, rpos) / 1000` (m→km)
/// 2. Convert left-handed → right-handed (swap y↔z)
/// 3. Compute perturbation via `pines.accel()`
/// 4. Convert right-handed → left-handed (swap y↔z)
/// 5. Rotate back to global frame: `dg = mul(rot, dg) * 1000` (km→m)
pub fn pines_perturbation(
    rpos: Vec3,
    pines: &PinesModel,
    cutoff: usize,
    rot: &Matrix3,
) -> Vec3 {
    // Rotate position vector into the planet's local frame.
    // C++: lpos = -tmul(rot, rpos) / 1000.0
    let lpos_global = mat3::tmul(*rot, rpos) * (-1.0 / 1000.0); // m → km

    // Convert from Orbiter's left-handed to right-handed (swap y↔z).
    let lpos = Vec3Pines::new(lpos_global.x, lpos_global.z, lpos_global.y);

    // Compute perturbation acceleration [km/s²].
    let dg = pines.accel(lpos, cutoff, cutoff);

    // Convert back to Orbiter's left-handedness (swap y↔z).
    let dg_global = Vec3::new(dg.x, dg.z, dg.y);

    // Rotate back into global frame and convert km/s² → m/s².
    mat3::mul(*rot, dg_global) * 1000.0
}

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
                rotation: None,
                pines: None,
            },
            GravBody {
                pos: Vec3::new(3.84e8, 0.0, 0.0),
                mass: 7.35e22,
                size: 1.74e6,
                jcoeff: vec![],
                rotation: None,
                pines: None,
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

    #[test]
    fn jcoeff_with_rot_matches_identity_for_y_axis() {
        // With identity rotation, the new function should match the old one
        // (which assumes y-axis = rotation axis).
        let gm = 3.986e14;
        let size = 6.371e6;
        let jcoeff = vec![1.0826e-3];
        let rpos = Vec3::new(7.0e6, 3.0e6, 0.0);

        let with_rot =
            jcoeff_perturbation_with_rot(rpos, size, gm, &jcoeff, &Matrix3::IDENTITY);
        let without_rot = jcoeff_perturbation(rpos, size, gm, &jcoeff);

        let diff = (with_rot - without_rot).length();
        assert!(diff < 1e-6, "diff = {diff}");
    }

    #[test]
    fn pines_perturbation_at_pole() {
        // Create a minimal Pines model with J2.
        let data = "6378.1363, 398600.4415, 0, 2, 2, 1, 0, 0\n\
                    2, 0, -0.00108263, 0.0, 0.0, 0.0\n";
        let pines = PinesModel::from_reader(data.as_bytes(), 2).unwrap();

        // Position at 7000 km on y-axis (pole in Orbiter's left-handed frame).
        let rpos = Vec3::new(0.0, 7.0e6, 0.0);
        let dg = pines_perturbation(rpos, &pines, 2, &Matrix3::IDENTITY);
        let mag = dg.length();
        assert!(mag > 1e-6, "Pines perturbation at pole = {mag}");
    }

    #[test]
    fn pines_perturbation_decreases_with_distance() {
        let data = "6378.1363, 398600.4415, 0, 2, 2, 1, 0, 0\n\
                    2, 0, -0.00108263, 0.0, 0.0, 0.0\n";
        let pines = PinesModel::from_reader(data.as_bytes(), 2).unwrap();

        let dg_near = pines_perturbation(
            Vec3::new(0.0, 7.0e6, 0.0),
            &pines,
            2,
            &Matrix3::IDENTITY,
        );
        let dg_far = pines_perturbation(
            Vec3::new(0.0, 20.0e6, 0.0),
            &pines,
            2,
            &Matrix3::IDENTITY,
        );
        assert!(
            dg_near.length() > dg_far.length(),
            "near = {}, far = {}",
            dg_near.length(),
            dg_far.length()
        );
    }

    #[test]
    fn gacc_nbody_with_jcoeff_and_rotation() {
        // Test that gacc_nbody uses rotation matrix for J-coeff.
        let jcoeff = vec![1.0826e-3];
        let bodies = vec![
            GravBody {
                pos: Vec3::ZERO,
                mass: 5.97e24,
                size: 6.371e6,
                jcoeff: jcoeff.clone(),
                rotation: Some(Matrix3::IDENTITY),
                pines: None,
            },
        ];
        let gpos = Vec3::new(7.0e6, 3.0e6, 0.0);
        let acc = gacc_nbody(gpos, &bodies, None);
        // Should have nonzero y-component due to J2 off-equator.
        assert!(acc.y.abs() > 0.001, "acc.y = {}", acc.y);
    }
}
