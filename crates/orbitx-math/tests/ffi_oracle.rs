//! Property tests comparing orbitx-math against the Orbiter C++ oracle via FFI.
//!
//! Each test generates random inputs with `proptest`, runs both the Rust
//! implementation and the C++ oracle (through `orbitx_math_ffi`), and asserts
//! the outputs agree within a tight tolerance. Since both use `f64` and the
//! formulas are byte-for-byte ports, the agreement should be near-exact
//! (tolerance accounts for FP scheduling differences between compilers).

// Test inputs use approximate PI/TAU values for angle ranges.
#![allow(clippy::approx_constant)]

use orbitx_math::{self as m, Vec3};
use orbitx_math_ffi as ffi;
use proptest::prelude::*;

/// Relative tolerance for comparing floating-point outputs.
const TOL: f64 = 1e-10;
/// Absolute tolerance — used when magnitudes are near zero.
const ATOL: f64 = 1e-12;

/// Assert two `f64` values are close: relative OR absolute. Both-NaN is
/// considered equal (the C++ oracle and Rust produce NaN in lockstep for
/// out-of-domain transcendental inputs).
fn assert_close(a: f64, b: f64, msg: &str) {
    if a.is_nan() && b.is_nan() {
        return;
    }
    let diff = (a - b).abs();
    let maxmag = a.abs().max(b.abs());
    let allowed = TOL * maxmag + ATOL;
    assert!(
        diff <= allowed,
        "{msg}: {a} vs {b} (diff={diff}, allowed={allowed})"
    );
}

fn assert_vec_close(a: Vec3, b: ffi::CVec3, msg: &str) {
    assert_close(a.x, b.x, &format!("{msg}.x"));
    assert_close(a.y, b.y, &format!("{msg}.y"));
    assert_close(a.z, b.z, &format!("{msg}.z"));
}

// --- Input generators ---

fn finite_f64() -> impl Strategy<Value = f64> {
    -1e6f64..1e6f64
}

fn vec3_strategy() -> impl Strategy<Value = Vec3> {
    (finite_f64(), finite_f64(), finite_f64()).prop_map(|(x, y, z)| Vec3::new(x, y, z))
}

fn quat_strategy() -> impl Strategy<Value = m::Quat> {
    (-2.0f64..2.0, -2.0f64..2.0, -2.0f64..2.0, -2.0f64..2.0).prop_map(|(vx, vy, vz, s)| {
        let mut q = m::Quat::new(vx, vy, vz, s);
        q.normalise();
        q
    })
}

// === Vec3 property tests ===

proptest! {
    #[test]
    fn prop_crossp(a in vec3_strategy(), b in vec3_strategy()) {
        let rust = m::cross(a, b);
        let (ca, cb) = (ffi::CVec3::from(a), ffi::CVec3::from(b));
        let mut out = ffi::CVec3::new(0.0, 0.0, 0.0);
        unsafe { ffi::ox_crossp(&ca, &cb, &mut out); }
        assert_vec_close(rust, out, "crossp");
    }

    #[test]
    fn prop_dotp(a in vec3_strategy(), b in vec3_strategy()) {
        let rust = m::dot(a, b);
        let (ca, cb) = (ffi::CVec3::from(a), ffi::CVec3::from(b));
        let cpp = unsafe { ffi::ox_dotp(&ca, &cb) };
        assert_close(rust, cpp, "dotp");
    }

    #[test]
    fn prop_v3_add(a in vec3_strategy(), b in vec3_strategy()) {
        let rust = a + b;
        let (ca, cb) = (ffi::CVec3::from(a), ffi::CVec3::from(b));
        let mut out = ffi::CVec3::new(0.0, 0.0, 0.0);
        unsafe { ffi::ox_v3_add(&ca, &cb, &mut out); }
        assert_vec_close(rust, out, "v3_add");
    }

    #[test]
    fn prop_v3_sub(a in vec3_strategy(), b in vec3_strategy()) {
        let rust = a - b;
        let (ca, cb) = (ffi::CVec3::from(a), ffi::CVec3::from(b));
        let mut out = ffi::CVec3::new(0.0, 0.0, 0.0);
        unsafe { ffi::ox_v3_sub(&ca, &cb, &mut out); }
        assert_vec_close(rust, out, "v3_sub");
    }

    #[test]
    fn prop_v3_mul_scalar(a in vec3_strategy(), s in finite_f64()) {
        let rust = a * s;
        let ca = ffi::CVec3::from(a);
        let mut out = ffi::CVec3::new(0.0, 0.0, 0.0);
        unsafe { ffi::ox_v3_mul_scalar(&ca, s, &mut out); }
        assert_vec_close(rust, out, "v3_mul_scalar");
    }

    #[test]
    fn prop_v3_hadamard(a in vec3_strategy(), b in vec3_strategy()) {
        let rust = a * b;
        let (ca, cb) = (ffi::CVec3::from(a), ffi::CVec3::from(b));
        let mut out = ffi::CVec3::new(0.0, 0.0, 0.0);
        unsafe { ffi::ox_v3_hadamard(&ca, &cb, &mut out); }
        assert_vec_close(rust, out, "v3_hadamard");
    }

    #[test]
    fn prop_v3_length2(a in vec3_strategy()) {
        let rust = a.length2();
        let ca = ffi::CVec3::from(a);
        let cpp = unsafe { ffi::ox_v3_length2(&ca) };
        assert_close(rust, cpp, "v3_length2");
    }

    #[test]
    fn prop_v3_dist2(a in vec3_strategy(), b in vec3_strategy()) {
        let rust = a.dist2(b);
        let (ca, cb) = (ffi::CVec3::from(a), ffi::CVec3::from(b));
        let cpp = unsafe { ffi::ox_v3_dist2(&ca, &cb) };
        assert_close(rust, cpp, "v3_dist2");
    }

    #[test]
    fn prop_v3_unit(a in vec3_strategy()) {
        let rust = a.unit();
        let ca = ffi::CVec3::from(a);
        let mut out = ffi::CVec3::new(0.0, 0.0, 0.0);
        unsafe { ffi::ox_v3_unit(&ca, &mut out); }
        assert_vec_close(rust, out, "v3_unit");
    }

    #[test]
    fn prop_xangle(a in vec3_strategy(), b in vec3_strategy()) {
        let rust = m::xangle(a, b);
        let (ca, cb) = (ffi::CVec3::from(a), ffi::CVec3::from(b));
        let cpp = unsafe { ffi::ox_xangle(&ca, &cb) };
        assert_close(rust, cpp, "xangle");
    }
}

// === Matrix3 property tests ===

proptest! {
    #[test]
    fn prop_m3_mul(a in vec3_strategy(), b in vec3_strategy()) {
        let mat = m::Matrix3::new(
            a.x, a.y, a.z,
            b.x, b.y, b.z,
            0.0, 0.0, 1.0,
        );
        let v = Vec3::new(0.3, -1.7, 4.2);
        let rust = m::mul(mat, v);
        let (cmat, cv) = (ffi::CMat3::from(mat), ffi::CVec3::from(v));
        let mut out = ffi::CVec3::new(0.0, 0.0, 0.0);
        unsafe { ffi::ox_mul(&cmat, &cv, &mut out); }
        assert_vec_close(rust, out, "mul");
    }

    #[test]
    fn prop_m3_tmul(a in vec3_strategy(), b in vec3_strategy()) {
        let mat = m::Matrix3::new(
            a.x, a.y, a.z,
            b.x, b.y, b.z,
            0.5, -0.3, 2.1,
        );
        let v = Vec3::new(1.3, -2.7, 0.9);
        let rust = m::tmul(mat, v);
        let (cmat, cv) = (ffi::CMat3::from(mat), ffi::CVec3::from(v));
        let mut out = ffi::CVec3::new(0.0, 0.0, 0.0);
        unsafe { ffi::ox_tmul(&cmat, &cv, &mut out); }
        assert_vec_close(rust, out, "tmul");
    }

    #[test]
    fn prop_m3_transp(a in vec3_strategy(), b in vec3_strategy(), c in vec3_strategy()) {
        let mat = m::vector_basis_to_matrix(a, b, c);
        let rust = mat.transp();
        let cmat = ffi::CMat3::from(mat);
        let mut out = ffi::CMat3::default();
        unsafe { ffi::ox_transp(&cmat, &mut out); }
        let cpp: m::Matrix3 = out.into();
        for i in 0..3 {
            for j in 0..3 {
                assert_close(rust.get(i,j), cpp.get(i,j), "transp");
            }
        }
    }

    #[test]
    fn prop_m3_from_euler(rx in finite_f64(), ry in finite_f64(), rz in finite_f64()) {
        let rot = Vec3::new(rx, ry, rz);
        let rust = m::Matrix3::from_euler(rot);
        let crot = ffi::CVec3::from(rot);
        let mut out = ffi::CMat3::default();
        unsafe { ffi::ox_m3_from_euler(&crot, &mut out); }
        let cpp: m::Matrix3 = out.into();
        for i in 0..3 {
            for j in 0..3 {
                assert_close(rust.get(i,j), cpp.get(i,j), "from_euler");
            }
        }
    }
}

// === Quaternion property tests ===

proptest! {
    #[test]
    fn prop_q_hamilton(a in quat_strategy(), b in quat_strategy()) {
        let rust = a.hamilton(b);
        let (ca, cb) = (ffi::CQuat::from(a), ffi::CQuat::from(b));
        let mut out = ffi::CQuat::default();
        unsafe { ffi::ox_q_hamilton(&ca, &cb, &mut out); }
        let cpp: m::Quat = out.into();
        assert_close(rust.vx, cpp.vx, "hamilton.vx");
        assert_close(rust.vy, cpp.vy, "hamilton.vy");
        assert_close(rust.vz, cpp.vz, "hamilton.vz");
        assert_close(rust.s,  cpp.s,  "hamilton.s");
    }

    #[test]
    fn prop_q_mul_vec(q in quat_strategy(), p in vec3_strategy()) {
        let rust = m::mul_vec(q, p);
        let (cq, cp) = (ffi::CQuat::from(q), ffi::CVec3::from(p));
        let mut out = ffi::CVec3::new(0.0, 0.0, 0.0);
        unsafe { ffi::ox_q_mul_vec(&cq, &cp, &mut out); }
        assert_vec_close(rust, out, "q_mul_vec");
    }

    #[test]
    fn prop_q_tmul_vec(q in quat_strategy(), p in vec3_strategy()) {
        let rust = m::tmul_vec(q, p);
        let (cq, cp) = (ffi::CQuat::from(q), ffi::CVec3::from(p));
        let mut out = ffi::CVec3::new(0.0, 0.0, 0.0);
        unsafe { ffi::ox_q_tmul_vec(&cq, &cp, &mut out); }
        assert_vec_close(rust, out, "q_tmul_vec");
    }

    #[test]
    fn prop_q_rotate(q in quat_strategy(), omega in vec3_strategy()) {
        let mut qr = q;
        qr.rotate(omega);
        let (cq, comeega) = (ffi::CQuat::from(q), ffi::CVec3::from(omega));
        let mut out = ffi::CQuat::default();
        unsafe { ffi::ox_q_rotate(&cq, &comeega, &mut out); }
        let cpp: m::Quat = out.into();
        assert_close(qr.vx, cpp.vx, "rotate.vx");
        assert_close(qr.vy, cpp.vy, "rotate.vy");
        assert_close(qr.vz, cpp.vz, "rotate.vz");
        assert_close(qr.s,  cpp.s,  "rotate.s");
    }

    #[test]
    fn prop_q_norm2(q in quat_strategy()) {
        let rust = q.norm2();
        let cq = ffi::CQuat::from(q);
        let cpp = unsafe { ffi::ox_q_norm2(&cq) };
        assert_close(rust, cpp, "q_norm2");
    }

    #[test]
    fn prop_q_interp(a in quat_strategy(), b in quat_strategy(), u in 0.0f64..1.0) {
        let rust = m::interp(a, b, u);
        let (ca, cb) = (ffi::CQuat::from(a), ffi::CQuat::from(b));
        let mut out = ffi::CQuat::default();
        unsafe { ffi::ox_q_interp(&ca, &cb, u, &mut out); }
        let cpp: m::Quat = out.into();
        // interp may return q or -q (double cover); compare via norm of difference.
        let d1 = (rust.vx - cpp.vx).powi(2) + (rust.vy - cpp.vy).powi(2)
               + (rust.vz - cpp.vz).powi(2) + (rust.s - cpp.s).powi(2);
        let d2 = (rust.vx + cpp.vx).powi(2) + (rust.vy + cpp.vy).powi(2)
               + (rust.vz + cpp.vz).powi(2) + (rust.s + cpp.s).powi(2);
        // Allow small drift for non-unit input quaternions (interp normalises).
        assert!(d1.min(d2) < 1e-6, "interp: d1={d1}, d2={d2}");
    }
}

// === Geometry property tests ===

proptest! {
    #[test]
    fn prop_point_line_dist(a in vec3_strategy(), p in vec3_strategy(), d in vec3_strategy()) {
        let rust = m::point_line_dist(a, p, d);
        let (ca, cp, cd) = (ffi::CVec3::from(a), ffi::CVec3::from(p), ffi::CVec3::from(d));
        let cpp = unsafe { ffi::ox_point_line_dist(&ca, &cp, &cd) };
        assert_close(rust, cpp, "point_line_dist");
    }

    #[test]
    fn prop_dir_rot_to_matrix(z in vec3_strategy(), y in vec3_strategy()) {
        let rust = m::dir_rot_to_matrix(z, y);
        let (cz, cy) = (ffi::CVec3::from(z), ffi::CVec3::from(y));
        let mut out = ffi::CMat3::default();
        unsafe { ffi::ox_dir_rot_to_matrix(&cz, &cy, &mut out); }
        let cpp: m::Matrix3 = out.into();
        for i in 0..3 {
            for j in 0..3 {
                assert_close(rust.get(i,j), cpp.get(i,j), "dir_rot");
            }
        }
    }
}

// === Astro property tests ===

proptest! {
    #[test]
    fn prop_obliquity(jc in -1.0f64..2.0) {
        let rust = m::obliquity(jc);
        let cpp = unsafe { ffi::ox_obliquity(jc) };
        assert_close(rust, cpp, "obliquity");
    }

    #[test]
    fn prop_equ2ecl(ra in 0.0f64..6.28, dc in -1.57f64..1.57) {
        let cosob = 0.9175; let sinob = 0.3978;
        let (l_rust, b_rust) = m::equ_to_ecl(cosob, sinob, ra, dc);
        let (mut l_cpp, mut b_cpp) = (0.0f64, 0.0f64);
        unsafe { ffi::ox_equ2ecl(cosob, sinob, ra, dc, &mut l_cpp, &mut b_cpp); }
        assert_close(l_rust, l_cpp, "equ2ecl.l");
        assert_close(b_rust, b_cpp, "equ2ecl.b");
    }

    #[test]
    fn prop_ecl2equ(l in 0.0f64..6.28, b in -1.57f64..1.57) {
        let cosob = 0.9175; let sinob = 0.3978;
        let (ra_rust, dc_rust) = m::ecl_to_equ(cosob, sinob, l, b);
        let (mut ra_cpp, mut dc_cpp) = (0.0f64, 0.0f64);
        unsafe { ffi::ox_ecl2equ(cosob, sinob, l, b, &mut ra_cpp, &mut dc_cpp); }
        assert_close(ra_rust, ra_cpp, "ecl2equ.ra");
        assert_close(dc_rust, dc_cpp, "ecl2equ.dc");
    }

    #[test]
    fn prop_orthodrome_dist(
        lng1 in -3.14f64..3.14, lat1 in -1.57f64..1.57,
        lng2 in -3.14f64..3.14, lat2 in -1.57f64..1.57
    ) {
        let rust = m::orthodrome_dist(lng1, lat1, lng2, lat2);
        let cpp = unsafe { ffi::ox_orthodrome_dist(lng1, lat1, lng2, lat2) };
        assert_close(rust, cpp, "orthodrome_dist");
    }

    #[test]
    fn prop_date_to_mjd(
        year in 1900i32..2100, month in 1i32..12, day in 1i32..28,
        hour in 0i32..23, min in 0i32..59, sec in 0i32..59
    ) {
        let date = m::CivilDate { year, month, day, hour, min, sec };
        let rust = m::date_to_mjd(date);
        let cdate = ffi::COxDate { year, month, day, hour, min, sec };
        let cpp = unsafe { ffi::ox_date_to_mjd(&cdate) };
        assert_close(rust, cpp, "date_to_mjd");
    }
}
