//! Quaternion — mirrors `class Quaternion` (Vecmat.h:329-404).
//!
//! **Left-handed convention.** Component order is `{vx, vy, vz, s}` (vector
//! part first, scalar last). The Hamilton product sign pattern, `mul`/`tmul`
//! swap, and `Rotate` formula all encode the left-handed system — these are
//! reproduced byte-for-byte from Vecmat.cpp.

use crate::mat3::Matrix3;
use crate::vec3::{dot, Vec3};

/// Quaternion with `f64` components (`class Quaternion`, Vecmat.h:329).
///
/// Storage: `{vx, vy, vz, s}` where `s` is the scalar (real) part and
/// `(vx,vy,vz)` the vector (imaginary) part. Identity is `(0,0,0,1)`.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct Quat {
    pub vx: f64,
    pub vy: f64,
    pub vz: f64,
    pub s: f64,
}

impl Default for Quat {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Quat {
    /// Identity quaternion `(0,0,0,1)`.
    pub const IDENTITY: Self = Self {
        vx: 0.0,
        vy: 0.0,
        vz: 0.0,
        s: 1.0,
    };

    /// Construct from 4 scalars `(vx, vy, vz, s)` (`Quaternion(double x4)`,
    /// Vecmat.h:337).
    #[inline]
    pub const fn new(vx: f64, vy: f64, vz: f64, s: f64) -> Self {
        Self { vx, vy, vz, s }
    }

    /// Construct from a vector part and scalar (`Quaternion(Vector, double)`,
    /// Vecmat.h:340).
    #[inline]
    pub const fn from_axis(v: Vec3, s: f64) -> Self {
        Self {
            vx: v.x,
            vy: v.y,
            vz: v.z,
            s,
        }
    }

    /// Squared norm (`norm2`, Vecmat.cpp:489): `s² + vx² + vy² + vz²`.
    #[inline]
    pub fn norm2(self) -> f64 {
        self.s * self.s + self.vx * self.vx + self.vy * self.vy + self.vz * self.vz
    }

    /// Norm (`norm`, Vecmat.h:365).
    #[inline]
    pub fn norm(self) -> f64 {
        self.norm2().sqrt()
    }

    /// Normalise in place (`normalise`, Vecmat.h:368).
    #[inline]
    pub fn normalise(&mut self) {
        let n = self.norm();
        if n > 0.0 {
            self.vx /= n;
            self.vy /= n;
            self.vz /= n;
            self.s /= n;
        }
    }

    /// Conjugate (`conj`, Vecmat.h:390): negates the vector part. Note the C++
    /// member takes an argument and ignores `*this`; here we conjugate `self`.
    #[inline]
    pub fn conj(self) -> Self {
        Self {
            vx: -self.vx,
            vy: -self.vy,
            vz: -self.vz,
            s: self.s,
        }
    }

    /// Integrate orientation by angular rate `omega` in place (`Rotate`,
    /// Vecmat.cpp:499). Implements `q̇ = ½ q ω` for the left-handed convention,
    /// then renormalises. Falls back to identity if the norm collapses.
    pub fn rotate(&mut self, omega: Vec3) {
        let dvx = 0.5 * (self.s * omega.x - self.vy * omega.z + self.vz * omega.y);
        let dvy = 0.5 * (self.s * omega.y - self.vz * omega.x + self.vx * omega.z);
        let dvz = 0.5 * (self.s * omega.z - self.vx * omega.y + self.vy * omega.x);
        let ds = 0.5 * (-self.vx * omega.x - self.vy * omega.y - self.vz * omega.z);
        self.vx += dvx;
        self.vy += dvy;
        self.vz += dvz;
        self.s += ds;

        let arg = self.norm2();
        if arg > 0.0 {
            let f = 1.0 / arg.sqrt();
            self.vx *= f;
            self.vy *= f;
            self.vz *= f;
            self.s *= f;
        } else {
            // Desperation: reset to identity (matches Vecmat.cpp:518-521).
            *self = Self::IDENTITY;
        }
    }

    /// Return a rotated copy without mutating `self` (`Rot`, Vecmat.cpp:524).
    /// Note: unlike [`rotate`](Self::rotate), this does **not** renormalise.
    #[inline]
    pub fn rotated(self, omega: Vec3) -> Self {
        let dvx = 0.5 * (self.s * omega.x - self.vy * omega.z + self.vz * omega.y);
        let dvy = 0.5 * (self.s * omega.y - self.vz * omega.x + self.vx * omega.z);
        let dvz = 0.5 * (self.s * omega.z - self.vx * omega.y + self.vy * omega.x);
        let ds = 0.5 * (-self.vx * omega.x - self.vy * omega.y - self.vz * omega.z);
        Self::new(self.vx + dvx, self.vy + dvy, self.vz + dvz, self.s + ds)
    }

    /// `*self = *this * Q` then renormalise (`operator+=`, Vecmat.cpp:532).
    pub fn add_assign(&mut self, q: Quat) {
        self.vx += q.vx;
        self.vy += q.vy;
        self.vz += q.vz;
        self.s += q.s;
        let inorm = 1.0 / self.norm();
        self.vx *= inorm;
        self.vy *= inorm;
        self.vz *= inorm;
        self.s *= inorm;
    }

    /// `*self = Q * *self` (`premul`, Vecmat.cpp:549). Note: the C++
    /// renormalisation block is wrapped in `#ifdef UNDEF` (disabled), so this
    /// does **not** renormalise.
    #[inline]
    pub fn premul(&mut self, q: Quat) {
        *self = q.hamilton(*self);
    }

    /// `*self = *self * Q` (`postmul`, Vecmat.cpp:568). Renormalisation disabled.
    #[inline]
    pub fn postmul(&mut self, q: Quat) {
        *self = self.hamilton(q);
    }

    /// `*self = *self * Q⁻¹` (`tpostmul`, Vecmat.cpp:587). Renormalisation disabled.
    #[inline]
    pub fn tpostmul(&mut self, q: Quat) {
        *self = self.hamilton(q.conj());
    }

    /// Left-handed Hamilton product `self * q` (`operator*`, Vecmat.cpp:606).
    ///
    /// The cross-term signs are negated relative to the standard right-handed
    /// product — this is the load-bearing left-handed signature.
    #[inline]
    pub fn hamilton(self, q: Quat) -> Quat {
        Quat::new(
            self.s * q.vx + self.vx * q.s - self.vy * q.vz + self.vz * q.vy,
            self.s * q.vy + self.vx * q.vz + self.vy * q.s - self.vz * q.vx,
            self.s * q.vz - self.vx * q.vy + self.vy * q.vx + self.vz * q.s,
            self.s * q.s - self.vx * q.vx - self.vy * q.vy - self.vz * q.vz,
        )
    }

    /// Rotate a vector by this quaternion (`mul(Quat, Vector)`, Vecmat.cpp:616).
    /// Convenience wrapper around [`crate::quat::mul_vec`].
    #[inline]
    pub fn mul_vec(self, p: Vec3) -> Vec3 {
        mul_vec(self, p)
    }

    /// Rotate a vector by the inverse quaternion (`tmul(Quat, Vector)`,
    /// Vecmat.cpp:630).
    #[inline]
    pub fn tmul_vec(self, p: Vec3) -> Vec3 {
        tmul_vec(self, p)
    }
}

/// Rotate vector `p` by quaternion `q` (`mul(Quat, Vector)`, Vecmat.cpp:616).
///
/// Faithful reproduction of Vecmat.cpp:619-627. The intermediate `vz`, `qvx2`,
/// etc. follow the left-handed convention documented at Vecmat.cpp:618.
#[inline]
pub fn mul_vec(q: Quat, p: Vec3) -> Vec3 {
    let vx = q.s * p.x - q.vy * p.z + q.vz * p.y;
    let vy = q.s * p.y - q.vz * p.x + q.vx * p.z;
    let vz = q.s * p.z - q.vx * p.y + q.vy * p.x;
    let qvx2 = -2.0 * q.vx;
    let qvy2 = -2.0 * q.vy;
    let qvz2 = -2.0 * q.vz;
    Vec3::new(
        p.x + qvy2 * vz - qvz2 * vy,
        p.y + qvz2 * vx - qvx2 * vz,
        p.z + qvx2 * vy - qvy2 * vx,
    )
}

/// Rotate vector `p` by the inverse of quaternion `q` (`tmul(Quat, Vector)`,
/// Vecmat.cpp:630). The sign pattern is swapped relative to [`mul_vec`].
#[inline]
pub fn tmul_vec(q: Quat, p: Vec3) -> Vec3 {
    let vx = q.s * p.x + q.vy * p.z - q.vz * p.y;
    let vy = q.s * p.y + q.vz * p.x - q.vx * p.z;
    let vz = q.s * p.z + q.vx * p.y - q.vy * p.x;
    let qvx2 = 2.0 * q.vx;
    let qvy2 = 2.0 * q.vy;
    let qvz2 = 2.0 * q.vz;
    Vec3::new(
        p.x + qvy2 * vz - qvz2 * vy,
        p.y + qvz2 * vx - qvx2 * vz,
        p.z + qvx2 * vy - qvy2 * vx,
    )
}

/// Dot product of two quaternions (`dotp`, Vecmat.cpp:494).
#[inline]
pub fn dotp(a: Quat, b: Quat) -> f64 {
    a.s * b.s + a.vx * b.vx + a.vy * b.vy + a.vz * b.vz
}

/// SLERP-like interpolation (`interp`, Vecmat.cpp:644).
///
/// Sets `*self` to the interpolation of `a` (at u=0) and `b` (at u=1). Resolves
/// the double-cover ambiguity by flipping `b` if the dot product is negative,
/// falls back to linear interpolation when nearly parallel, and forces
/// `s ≥ 0` on the result.
pub fn interp(a: Quat, b: Quat, u: f64) -> Quat {
    let mut dotab = dotp(a, b);
    let mut sign = 1.0;
    if dotab < 0.0 {
        dotab = -dotab;
        sign = -1.0;
    }
    let omega = dotab.clamp(-1.0, 1.0).acos();
    let sino = omega.sin();
    let (fa, fb) = if sino.abs() < 1e-8 {
        (1.0 - u, u)
    } else {
        (
            ((1.0 - u) * omega).sin() / sino,
            (u * omega).sin() / sino * sign,
        )
    };
    let mut r = Quat::new(
        fa * a.vx + fb * b.vx,
        fa * a.vy + fb * b.vy,
        fa * a.vz + fb * b.vz,
        fa * a.s + fb * b.s,
    );
    r.normalise();
    let inorm = if r.s < 0.0 { -1.0 } else { 1.0 };
    r.vx *= inorm;
    r.vy *= inorm;
    r.vz *= inorm;
    r.s *= inorm;
    r
}

/// Angle between the **vector parts** of two quaternions (`angle`,
/// Vecmat.cpp:681). Returns 0 if either vector part is zero.
pub fn angle(a: Quat, b: Quat) -> f64 {
    let av = Vec3::new(a.vx, a.vy, a.vz);
    let bv = Vec3::new(b.vx, b.vy, b.vz);
    let denom = av.length() * bv.length();
    if denom == 0.0 {
        0.0
    } else {
        (dot(av, bv) / denom).clamp(-1.0, 1.0).acos()
    }
}

// --- Quaternion ↔ Matrix conversion ---

impl Quat {
    /// Build from a rotation matrix (`Set(Matrix)`, Vecmat.cpp:457). Uses
    /// Shepperd's method with four branches and `eps = 1e-12`.
    pub fn from_matrix(r: Matrix3) -> Self {
        let eps = 1e-12;
        let t = 1.0 + r.m11 + r.m22 + r.m33;
        if t > eps {
            let s = 2.0 * t.sqrt();
            Quat::new(
                (r.m23 - r.m32) / s,
                (r.m31 - r.m13) / s,
                (r.m12 - r.m21) / s,
                0.25 * s,
            )
        } else if r.m11 > r.m22 && r.m11 > r.m33 {
            let s = 2.0 * (1.0 + r.m11 - r.m22 - r.m33).sqrt();
            Quat::new(
                0.25 * s,
                (r.m21 + r.m12) / s,
                (r.m31 + r.m13) / s,
                (r.m23 - r.m32) / s,
            )
        } else if r.m22 > r.m33 {
            let s = 2.0 * (1.0 + r.m22 - r.m11 - r.m33).sqrt();
            Quat::new(
                (r.m21 + r.m12) / s,
                0.25 * s,
                (r.m32 + r.m23) / s,
                (r.m31 - r.m13) / s,
            )
        } else {
            let s = 2.0 * (1.0 + r.m33 - r.m11 - r.m22).sqrt();
            Quat::new(
                (r.m31 + r.m13) / s,
                (r.m32 + r.m23) / s,
                0.25 * s,
                (r.m12 - r.m21) / s,
            )
        }
    }
}

impl Matrix3 {
    /// Build from a quaternion (`Set(Quaternion)`, Vecmat.cpp:70). Standard
    /// quaternion-to-matrix form; combined with the left-handed `mul`/`tmul`
    /// it produces left-handed behaviour.
    pub fn from_quat(q: Quat) -> Self {
        let qx2 = 2.0 * q.vx;
        let qy2 = 2.0 * q.vy;
        let qz2 = 2.0 * q.vz;
        let qxx2 = qx2 * q.vx;
        let qyy2 = qy2 * q.vy;
        let qzz2 = qz2 * q.vz;
        let qxy2 = qx2 * q.vy;
        let qxz2 = qx2 * q.vz;
        let qxw2 = qx2 * q.s;
        let qyz2 = qy2 * q.vz;
        let qyw2 = qy2 * q.s;
        let qzw2 = qz2 * q.s;
        Matrix3::new(
            1.0 - qyy2 - qzz2,
            qxy2 + qzw2,
            qxz2 - qyw2,
            qxy2 - qzw2,
            1.0 - qxx2 - qzz2,
            qyz2 + qxw2,
            qxz2 + qyw2,
            qyz2 - qxw2,
            1.0 - qxx2 - qyy2,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identity_is_unit() {
        let q = Quat::IDENTITY;
        assert!((q.norm() - 1.0).abs() < 1e-12);
    }

    #[test]
    fn hamilton_identity() {
        let q = Quat::new(1.0, 2.0, 3.0, 4.0);
        let p = q.hamilton(Quat::IDENTITY);
        assert_eq!(p, q);
    }

    #[test]
    fn matrix_quat_roundtrip() {
        // Build a known rotation matrix (90° about y, left-handed) and convert.
        let r = Matrix3::new(0.0, 0.0, -1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0);
        let q2 = Quat::from_matrix(r);
        let r2 = Matrix3::from_quat(q2);
        for i in 0..3 {
            for j in 0..3 {
                assert!(
                    (r.get(i, j) - r2.get(i, j)).abs() < 1e-9,
                    "mismatch at ({i},{j}): {} vs {}",
                    r.get(i, j),
                    r2.get(i, j)
                );
            }
        }
    }

    #[test]
    fn interp_endpoints() {
        let a = Quat::new(0.0, 0.0, 0.0, 1.0);
        let b = Quat::new(0.0, 0.0, 1.0, 0.0); // 180° about z
        let r0 = interp(a, b, 0.0);
        let r1 = interp(a, b, 1.0);
        assert!((r0.s - 1.0).abs() < 1e-9);
        assert!((r1.vz.abs() - 1.0).abs() < 1e-9 || (r1.s.abs() - 1.0).abs() < 1e-9);
    }
}
