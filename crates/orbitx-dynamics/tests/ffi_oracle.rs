//! Property tests comparing orbitx-dynamics Rust implementation against the
//! C++ oracle (`orbitx-dynamics-ffi`).

#![allow(clippy::approx_constant, clippy::excessive_precision)]

use orbitx_dynamics::kepler::Elements;
use orbitx_dynamics::pines::{nm, PinesModel, Vec3Pines};
use orbitx_dynamics::{gacc_nbody, jcoeff_perturbation, single_gacc, GravBody};
use orbitx_dynamics_ffi as ffi;
use orbitx_math::Vec3;
use proptest::prelude::*;

const TOL: f64 = 1e-10;
const ATOL: f64 = 1e-12;

fn assert_close(a: f64, b: f64, msg: &str) {
    let diff = (a - b).abs();
    let maxmag = a.abs().max(b.abs());
    let allowed = TOL * maxmag + ATOL;
    assert!(
        diff <= allowed || (a.is_nan() && b.is_nan()),
        "{msg}: {a} vs {b} (diff={diff}, allowed={allowed})"
    );
}

fn assert_close3(a: &[f64; 3], b: &[f64; 3], ctx: &str) {
    for i in 0..3 {
        assert_close(a[i], b[i], &format!("{ctx}[{i}]"));
    }
}

// ===========================================================
// Point-mass gravity property tests
// ===========================================================

proptest! {
    #[test]
    fn prop_single_gacc(
        rx in -1e9_f64..1e9,
        ry in -1e9_f64..1e9,
        rz in -1e9_f64..1e9,
        gm in 1e10_f64..1e20,
    ) {
        // Skip near-zero positions
        prop_assume!(rx*rx + ry*ry + rz*rz > 1e6);

        let rpos = Vec3::new(rx, ry, rz);
        let rust = single_gacc(rpos, gm);
        let cpp = ffi::single_gacc([rx, ry, rz], gm);

        assert_close3(&[rust.x, rust.y, rust.z], &cpp, "single_gacc");
    }
}

// ===========================================================
// J2/J3/J4 property tests
// ===========================================================

proptest! {
    #[test]
    fn prop_jcoeff_pert(
        rx in 6.5e6_f64..1e8,
        rz in -1e8_f64..1e8,
    ) {
        let ry = 0.0_f64;
        let body_size = 6.37101e6_f64;
        let gm = 3.986e14_f64;
        let jcoeff = vec![1.0826e-3, -2.51e-6, -1.60e-6];

        let rpos = Vec3::new(rx, ry, rz);
        let rust = jcoeff_perturbation(rpos, body_size, gm, &jcoeff);
        let cpp = ffi::jcoeff_pert([rx, ry, rz], body_size, gm, &jcoeff);

        assert_close3(&[rust.x, rust.y, rust.z], &cpp, "jcoeff_pert");
    }
}

// ===========================================================
// Pines spherical harmonic gravity property tests
// ===========================================================

fn make_simple_pines_model() -> (PinesModel, Vec<f64>, Vec<f64>) {
    // A model with C(2,0) = J2 and C(3,0) = J3.
    let data = "6378.1363, 398600.4415, 0, 3, 3, 1, 0, 0\n\
                2, 0, -0.00108263, 0.0, 0.0, 0.0\n\
                3, 0, 2.54e-6, 0.0, 0.0, 0.0\n";
    let model = PinesModel::from_reader(data.as_bytes(), 3).unwrap();

    // Build flat C/S arrays matching the oracle's NM indexing.
    let max_idx = nm(5, 5);
    let mut c = vec![0.0_f64; max_idx + 1];
    let s = vec![0.0_f64; max_idx + 1];
    c[nm(2, 0)] = -0.00108263;
    c[nm(3, 0)] = 2.54e-6;

    (model, c, s)
}

proptest! {
    #[test]
    fn prop_pines_accel(
        x in -20000.0_f64..20000.0,
        y in -20000.0_f64..20000.0,
        z in 1000.0_f64..20000.0,
    ) {
        let (model, c, s) = make_simple_pines_model();
        let rpos = Vec3Pines::new(x, y, z);
        let rust = model.accel(rpos, model.degree, model.order);
        let cpp = ffi::pines_accel(
            [x, y, z],
            model.ref_rad,
            model.gm,
            &c,
            &s,
            model.degree,
            model.order,
        );

        assert_close3(&[rust.x, rust.y, rust.z], &cpp, "pines_accel");
    }
}

// ===========================================================
// Kepler EccAnomaly property tests
// ===========================================================

proptest! {
    #[test]
    fn prop_ecc_anomaly_closed(
        ma in 0.0_f64..6.28,
        e in 0.0_f64..0.95,
    ) {
        // Create elements with the given eccentricity by starting from
        // periapsis of an elliptical orbit.
        let mu: f64 = 3.986e14;
        let a = 8.0e6_f64;
        let r_pe = a * (1.0 - e);
        let v_pe = (mu * (2.0 / r_pe - 1.0 / a)).sqrt();
        let el = Elements::calculate(
            Vec3::new(r_pe, 0.0, 0.0),
            Vec3::new(0.0, 0.0, v_pe),
            mu,
            0.0,
        );

        let rust_ea = el.ecc_anomaly(ma);
        let cpp_ea = ffi::ecc_anomaly(ma, e, el.ecc_anm(), el.mean_anm());

        assert_close(rust_ea, cpp_ea, "ecc_anomaly");
    }
}

// ===========================================================
// N-body gravity property tests
// ===========================================================

proptest! {
    #[test]
    fn prop_nbody_gacc(
        gx in -1e9_f64..1e9,
        gy in -1e9_f64..1e9,
        gz in -1e9_f64..1e9,
    ) {
        prop_assume!(gx*gx + gy*gy + gz*gz > 1e10);

        let bodies = vec![
            GravBody {
                pos: Vec3::new(0.0, 0.0, 0.0),
                mass: 5.97e24,
                size: 6.371e6,
                jcoeff: vec![],
            },
            GravBody {
                pos: Vec3::new(1.5e11, 0.0, 0.0),
                mass: 1.99e30,
                size: 6.96e8,
                jcoeff: vec![],
            },
        ];

        let gpos = Vec3::new(gx, gy, gz);
        let rust = gacc_nbody(gpos, &bodies, None);

        // Compare against manual summation via the C++ single_gacc oracle.
        let mut cpp = [0.0_f64; 3];
        for body in &bodies {
            let rpos = [body.pos.x - gx, body.pos.y - gy, body.pos.z - gz];
            let acc = ffi::single_gacc(rpos, body.gm());
            for i in 0..3 {
                cpp[i] += acc[i];
            }
        }

        assert_close3(&[rust.x, rust.y, rust.z], &cpp, "nbody_gacc");
    }
}
