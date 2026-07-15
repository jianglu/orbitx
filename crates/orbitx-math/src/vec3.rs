//! 3D vector — mirrors `class Vector` (Vecmat.h:74-168).
//!
//! Storage is `{x, y, z}` as `#[repr(C)]` to match the C++ `union { double data[3];
//! struct { double x, y, z; }; }`. The cross product uses the **standard component
//! formula**; the left-handedness of the Orbiter library arises from how callers
//! order arguments (see `DirRotToMatrix` in [`crate::geom`]).

use core::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Sub, SubAssign};

use crate::consts::PI2;

/// 3D vector with `f64` components (`class Vector`, Vecmat.h:74).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    /// All-zero vector.
    pub const ZERO: Self = Self::new(0.0, 0.0, 0.0);

    /// `const` constructor mirroring `Vector(double, double, double)` (Vecmat.h:79).
    #[inline]
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    /// Set all three components (`Vector::Set(3 dbl)`, Vecmat.h:85).
    #[inline]
    pub fn set(&mut self, x: f64, y: f64, z: f64) {
        self.x = x;
        self.y = y;
        self.z = z;
    }

    /// Indexing, matching `operator()(int i)` → `data[i]` (Vecmat.h:91,94).
    #[inline]
    pub fn get(&self, i: usize) -> f64 {
        match i {
            0 => self.x,
            1 => self.y,
            2 => self.z,
            _ => panic!("Vec3 index out of range: {i}"),
        }
    }

    /// Mutable indexing.
    #[inline]
    pub fn get_mut(&mut self, i: usize) -> &mut f64 {
        match i {
            0 => &mut self.x,
            1 => &mut self.y,
            2 => &mut self.z,
            _ => panic!("Vec3 index out of range: {i}"),
        }
    }

    /// Squared magnitude (`length2`, Vecmat.h:142).
    #[inline]
    pub fn length2(self) -> f64 {
        self.x * self.x + self.y * self.y + self.z * self.z
    }

    /// Magnitude (`length`, Vecmat.h:145).
    #[inline]
    pub fn length(self) -> f64 {
        self.length2().sqrt()
    }

    /// Squared distance to `other` (`dist2`, Vecmat.cpp:18).
    #[inline]
    pub fn dist2(self, other: Self) -> f64 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        let dz = self.z - other.z;
        dx * dx + dy * dy + dz * dz
    }

    /// Distance to `other` (`dist`, Vecmat.h:150).
    #[inline]
    pub fn dist(self, other: Self) -> f64 {
        self.dist2(other).sqrt()
    }

    /// Normalised copy (`unit`, Vecmat.cpp:26). Returns the zero vector if the
    /// input is zero (matches C++ behaviour of dividing by zero length).
    #[inline]
    pub fn unit(self) -> Self {
        let len = self.length();
        if len > 0.0 {
            self / len
        } else {
            Self::ZERO
        }
    }

    /// Normalise in place (`unify`, Vecmat.cpp:32).
    #[inline]
    pub fn unify(&mut self) {
        let len = self.length();
        if len > 0.0 {
            *self /= len;
        }
    }
}

// --- Operators (Vecmat.h:100-133) ---

impl Add for Vec3 {
    type Output = Self;
    #[inline]
    fn add(self, rhs: Self) -> Self {
        Self::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
}

impl Sub for Vec3 {
    type Output = Self;
    #[inline]
    fn sub(self, rhs: Self) -> Self {
        Self::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
}

impl Neg for Vec3 {
    type Output = Self;
    #[inline]
    fn neg(self) -> Self {
        Self::new(-self.x, -self.y, -self.z)
    }
}

impl Mul<f64> for Vec3 {
    type Output = Self;
    #[inline]
    fn mul(self, s: f64) -> Self {
        Self::new(self.x * s, self.y * s, self.z * s)
    }
}

/// Component-wise (Hadamard) product — `Vector::operator*(Vector)` (Vecmat.h:112).
/// Note: this is **not** the cross product.
impl Mul<Self> for Vec3 {
    type Output = Self;
    #[inline]
    fn mul(self, rhs: Self) -> Self {
        Self::new(self.x * rhs.x, self.y * rhs.y, self.z * rhs.z)
    }
}

impl Div<f64> for Vec3 {
    type Output = Self;
    #[inline]
    fn div(self, s: f64) -> Self {
        Self::new(self.x / s, self.y / s, self.z / s)
    }
}

/// Component-wise divide — `Vector::operator/(Vector)` (Vecmat.h:118).
impl Div<Self> for Vec3 {
    type Output = Self;
    #[inline]
    fn div(self, rhs: Self) -> Self {
        Self::new(self.x / rhs.x, self.y / rhs.y, self.z / rhs.z)
    }
}

impl AddAssign for Vec3 {
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
        self.x += rhs.x;
        self.y += rhs.y;
        self.z += rhs.z;
    }
}

impl SubAssign for Vec3 {
    #[inline]
    fn sub_assign(&mut self, rhs: Self) {
        self.x -= rhs.x;
        self.y -= rhs.y;
        self.z -= rhs.z;
    }
}

impl MulAssign<f64> for Vec3 {
    #[inline]
    fn mul_assign(&mut self, s: f64) {
        self.x *= s;
        self.y *= s;
        self.z *= s;
    }
}

impl DivAssign<f64> for Vec3 {
    #[inline]
    fn div_assign(&mut self, s: f64) {
        self.x /= s;
        self.y /= s;
        self.z /= s;
    }
}

// --- Free functions ---

/// Dot product (`dotp`, Vecmat.h:139). In C++ this is also accessible via
/// `operator&`.
#[inline]
pub fn dot(a: Vec3, b: Vec3) -> f64 {
    a.x * b.x + a.y * b.y + a.z * b.z
}

/// Cross product (`crossp`, Vecmat.h:136). Uses the standard component formula;
/// left-handedness is a property of *callers*, not this function.
#[inline]
pub fn cross(a: Vec3, b: Vec3) -> Vec3 {
    Vec3::new(
        a.y * b.z - b.y * a.z,
        a.z * b.x - b.z * a.x,
        a.x * b.y - b.x * a.y,
    )
}

/// Angle between two direction vectors (`xangle`, Vecmat.cpp:38).
///
/// Returns the line angle in `[0, π]`. When `cos α ≥ 0` returns `acos(cos α)`;
/// otherwise folds back via `2π - acos(...)` so the result never exceeds π.
#[inline]
pub fn xangle(a: Vec3, b: Vec3) -> f64 {
    let la = a.length();
    let lb = b.length();
    if la == 0.0 || lb == 0.0 {
        return 0.0;
    }
    let cosa = dot(a, b) / (la * lb);
    if cosa >= 1.0 {
        0.0
    } else if cosa >= 0.0 {
        cosa.acos()
    } else {
        PI2 - cosa.acos()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::consts::PI05;

    #[test]
    fn dot_and_cross() {
        let x = Vec3::new(1.0, 0.0, 0.0);
        let y = Vec3::new(0.0, 1.0, 0.0);
        assert_eq!(dot(x, y), 0.0);
        // Standard formula: x × y = (0,0,1).
        let z = cross(x, y);
        assert!((z - Vec3::new(0.0, 0.0, 1.0)).length() < 1e-12);
    }

    #[test]
    fn length_unit() {
        let v = Vec3::new(3.0, 4.0, 0.0);
        assert_eq!(v.length(), 5.0);
        let u = v.unit();
        assert!((u.length() - 1.0).abs() < 1e-12);
    }

    #[test]
    fn hadamard_product() {
        let a = Vec3::new(2.0, 3.0, 4.0);
        let b = Vec3::new(1.0, 2.0, 3.0);
        assert_eq!(a * b, Vec3::new(2.0, 6.0, 12.0));
    }

    #[test]
    fn xangle_orthogonal() {
        let a = Vec3::new(1.0, 0.0, 0.0);
        let b = Vec3::new(0.0, 1.0, 0.0);
        assert!((xangle(a, b) - PI05).abs() < 1e-12);
    }
}
