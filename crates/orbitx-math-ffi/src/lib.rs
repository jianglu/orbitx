//! Rust FFI bindings to the C++ Orbiter math oracle.
//!
//! All struct arguments and returns go through raw pointers (see the C++
//! `shim.cpp`) to avoid ABI drift between clang's union/struct-by-value calling
//! convention and Rust's `#[repr(C)]` struct convention. The Rust side
//! constructs values in `#[repr(C)]` shadow structs with identical byte
//! layouts and passes pointers.
//!
//! These bindings exist only for property tests in `orbitx-math`; they are not
//! part of the public API.

use core::ffi::c_int;

/// C++ `Vector` (Vecmat.h:164): `union { double data[3]; struct { x,y,z }; }`.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct CVec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl CVec3 {
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }
}

/// C++ `Matrix` (Vecmat.h:233): 9 contiguous doubles, row-major.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct CMat3 {
    pub m11: f64,
    pub m12: f64,
    pub m13: f64,
    pub m21: f64,
    pub m22: f64,
    pub m23: f64,
    pub m31: f64,
    pub m32: f64,
    pub m33: f64,
}

/// C++ `Quaternion` (Vecmat.h:400-402): `{ qvx, qvy, qvz, qs }`.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct CQuat {
    pub vx: f64,
    pub vy: f64,
    pub vz: f64,
    pub s: f64,
}

/// C++ `struct tm` subset used by `date2mjd`.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct COxDate {
    pub year: c_int,
    pub month: c_int,
    pub day: c_int,
    pub hour: c_int,
    pub min: c_int,
    pub sec: c_int,
}

extern "C" {
    // Vec3 — all out-params via pointers
    pub fn ox_crossp(a: *const CVec3, b: *const CVec3, out: *mut CVec3) -> f64;
    pub fn ox_dotp(a: *const CVec3, b: *const CVec3) -> f64;
    pub fn ox_v3_length2(v: *const CVec3) -> f64;
    pub fn ox_v3_unit(v: *const CVec3, out: *mut CVec3) -> f64;
    pub fn ox_v3_dist2(a: *const CVec3, b: *const CVec3) -> f64;
    pub fn ox_xangle(a: *const CVec3, b: *const CVec3) -> f64;
    pub fn ox_v3_add(a: *const CVec3, b: *const CVec3, out: *mut CVec3);
    pub fn ox_v3_sub(a: *const CVec3, b: *const CVec3, out: *mut CVec3);
    pub fn ox_v3_mul_scalar(a: *const CVec3, s: f64, out: *mut CVec3);
    pub fn ox_v3_hadamard(a: *const CVec3, b: *const CVec3, out: *mut CVec3);

    // Matrix3
    pub fn ox_m3_mul_m(a: *const CMat3, b: *const CMat3, out: *mut CMat3);
    pub fn ox_mul(a: *const CMat3, b: *const CVec3, out: *mut CVec3);
    pub fn ox_tmul(a: *const CMat3, b: *const CVec3, out: *mut CVec3);
    pub fn ox_inv(a: *const CMat3, out: *mut CMat3);
    pub fn ox_transp(a: *const CMat3, out: *mut CMat3);
    pub fn ox_imatrix(out: *mut CMat3);
    pub fn ox_m3_from_quat(q: *const CQuat, out: *mut CMat3);
    pub fn ox_m3_from_euler(rot: *const CVec3, out: *mut CMat3);
    pub fn ox_orthogonalise(m: *mut CMat3, axis: c_int);
    pub fn ox_qrdcmp3(a: *mut CMat3, c: *mut CVec3, d: *mut CVec3, sing: *mut c_int);
    pub fn ox_qrsolv3(a: *const CMat3, c: *const CVec3, d: *const CVec3, b: *mut CVec3);

    // Quaternion
    pub fn ox_q_identity(out: *mut CQuat);
    pub fn ox_q_from_matrix(r: *const CMat3, out: *mut CQuat);
    pub fn ox_q_hamilton(a: *const CQuat, b: *const CQuat, out: *mut CQuat);
    pub fn ox_q_mul_vec(q: *const CQuat, p: *const CVec3, out: *mut CVec3);
    pub fn ox_q_tmul_vec(q: *const CQuat, p: *const CVec3, out: *mut CVec3);
    pub fn ox_q_rotate(q: *const CQuat, omega: *const CVec3, out: *mut CQuat);
    pub fn ox_q_interp(a: *const CQuat, b: *const CQuat, u: f64, out: *mut CQuat);
    pub fn ox_q_norm2(q: *const CQuat) -> f64;

    // Geometry
    pub fn ox_plane_coeffs(
        p1: *const CVec3,
        p2: *const CVec3,
        p3: *const CVec3,
        a: *mut f64,
        b: *mut f64,
        c: *mut f64,
        d: *mut f64,
    );
    pub fn ox_point_line_dist(a: *const CVec3, p: *const CVec3, d: *const CVec3) -> f64;
    pub fn ox_point_plane_dist(p: *const CVec3, a: f64, b: f64, c: f64, d: f64) -> f64;
    pub fn ox_vector_basis_to_matrix(
        x: *const CVec3,
        y: *const CVec3,
        z: *const CVec3,
        r: *mut CMat3,
    );
    pub fn ox_dir_rot_to_matrix(z: *const CVec3, y: *const CVec3, r: *mut CMat3);

    // Astro
    pub fn ox_obliquity(jc: f64) -> f64;
    pub fn ox_equ2ecl(cosob: f64, sinob: f64, ra: f64, dc: f64, l: *mut f64, b: *mut f64);
    pub fn ox_ecl2equ(cosob: f64, sinob: f64, l: f64, b: f64, ra: *mut f64, dc: *mut f64);
    pub fn ox_orthodrome_dist(lng1: f64, lat1: f64, lng2: f64, lat2: f64) -> f64;
    pub fn ox_orthodrome(lng1: f64, lat1: f64, lng2: f64, lat2: f64, dist: *mut f64, dir: *mut f64);
    pub fn ox_date_to_mjd(d: *const COxDate) -> f64;
}

// --- Conversion helpers between orbitx_math types and C types ---

impl From<orbitx_math::Vec3> for CVec3 {
    fn from(v: orbitx_math::Vec3) -> Self {
        Self::new(v.x, v.y, v.z)
    }
}

impl From<CVec3> for orbitx_math::Vec3 {
    fn from(c: CVec3) -> Self {
        Self::new(c.x, c.y, c.z)
    }
}

impl From<orbitx_math::Matrix3> for CMat3 {
    fn from(m: orbitx_math::Matrix3) -> Self {
        let a = m.as_array();
        Self {
            m11: a[0],
            m12: a[1],
            m13: a[2],
            m21: a[3],
            m22: a[4],
            m23: a[5],
            m31: a[6],
            m32: a[7],
            m33: a[8],
        }
    }
}

impl From<CMat3> for orbitx_math::Matrix3 {
    fn from(c: CMat3) -> Self {
        Self::new(
            c.m11, c.m12, c.m13, c.m21, c.m22, c.m23, c.m31, c.m32, c.m33,
        )
    }
}

impl From<orbitx_math::Quat> for CQuat {
    fn from(q: orbitx_math::Quat) -> Self {
        Self {
            vx: q.vx,
            vy: q.vy,
            vz: q.vz,
            s: q.s,
        }
    }
}

impl From<CQuat> for orbitx_math::Quat {
    fn from(c: CQuat) -> Self {
        Self {
            vx: c.vx,
            vy: c.vy,
            vz: c.vz,
            s: c.s,
        }
    }
}
