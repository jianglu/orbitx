//! # orbitx-math
//!
//! Left-handed math library for orbitx, mirroring Orbiter's `Vecmat.h` and
//! `Astro.h` exactly. All types use `#[repr(C)]` with storage layouts that
//! match the C++ originals byte-for-byte, so that FFI property tests can
//! compare outputs directly.
//!
//! ## Coordinate system
//!
//! The library is hard-coded for Orbiter's **left-handed** ecliptic J2000 frame
//! (`ẑ = ŷ × x̂`). The handedness is encoded in the quaternion Hamilton product
//! signs, the `mul`/`tmul` swap, the cross-product call ordering in
//! [`geom::dir_rot_to_matrix`], and the plane-coefficient signs. There is no
//! runtime or compile-time switch; all callers assume left-handedness.
//!
//! ## Module overview
//!
//! | Module     | C++ source     | Contents |
//! |------------|----------------|----------|
//! | [`consts`] | `Vecmat.h`, `Astro.h` | π, G, AU, MJD2000, angle helpers |
//! | [`vec3`]   | `class Vector` | 3D vector, arithmetic, dot/cross |
//! | [`vec4`]   | `class Vector4` | 4-element vector (for Matrix4 QR) |
//! | [`mat3`]   | `class Matrix` | 3×3 matrix, mul/tmul/inv/QR |
//! | [`mat4`]   | `class Matrix4` | 4×4 matrix, QR family |
//! | [`quat`]   | `class Quaternion` | Left-handed quaternion |
//! | [`state`]  | `class StateVectors` | Rigid-body state bundle |
//! | [`geom`]   | free functions | Plane/line geometry, basis→matrix |
//! | [`astro`]  | `Astro.h/cpp` | MJD, obliquity, ecl↔equ, orthodrome |

// Constants replicate Orbiter's C++ literals (not std::f64::consts) for
// bit-for-bit parity with the oracle.
#![allow(clippy::approx_constant, clippy::excessive_precision)]

pub mod astro;
pub mod consts;
pub mod geom;
pub mod mat3;
pub mod mat4;
pub mod quat;
pub mod state;
pub mod vec3;
pub mod vec4;

// Re-export the most commonly used items at the crate root for ergonomics.
pub use astro::{
    date_to_mjd, ecl_to_equ, equ_to_ecl, mjd_to_date, obliquity, orthodrome, orthodrome_dist,
    CivilDate,
};
pub use consts::{deg, diff_angle, rad, GGRAV, MJD2000, PI, PI05, PI2};
pub use geom::{
    dir_rot_to_matrix, plane_coeffs, point_line_dist, point_plane_dist, vector_basis_to_matrix,
};
pub use mat3::{identity, inv, mul, qrdcmp, qrsolv, tmul, transp, Matrix3};
pub use mat4::{qr_factorize, qr_solve, qrdcmp as qrdcmp4, qrsolv as qrsolv4, r_solve, Matrix4};
pub use quat::{angle, dotp as qdotp, interp, mul_vec, tmul_vec, Quat};
pub use state::StateVectors;
pub use vec3::{cross, dot, xangle, Vec3};
pub use vec4::Vec4;
