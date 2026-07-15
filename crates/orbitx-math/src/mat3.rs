//! 3×3 matrix — mirrors `class Matrix` (Vecmat.h:173-237).
//!
//! Storage is 9 contiguous `f64` in **row-major** order (`{m11,m12,m13,m21,...}`),
//! matching `data[i*3+j]` (Vecmat.h:204). `mul(A, b)` returns `A·b`,
//! `tmul(A, b)` returns `Aᵀ·b`. Rows are local basis vectors expressed in the
//! global frame.

use crate::vec3::{cross, Vec3};

/// 3×3 row-major matrix (`class Matrix`, Vecmat.h:173).
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct Matrix3 {
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

impl Default for Matrix3 {
    fn default() -> Self {
        Self::ZERO
    }
}

impl Matrix3 {
    /// All-zero matrix (`Matrix()` default ctor, Vecmat.cpp:51).
    pub const ZERO: Self = Self {
        m11: 0.0,
        m12: 0.0,
        m13: 0.0,
        m21: 0.0,
        m22: 0.0,
        m23: 0.0,
        m31: 0.0,
        m32: 0.0,
        m33: 0.0,
    };

    /// Identity (`IMatrix`, Vecmat.cpp:200).
    pub const IDENTITY: Self = Self {
        m11: 1.0,
        m12: 0.0,
        m13: 0.0,
        m21: 0.0,
        m22: 1.0,
        m23: 0.0,
        m31: 0.0,
        m32: 0.0,
        m33: 1.0,
    };

    /// Row-major construction from 9 scalars (`Matrix(double x9)`, Vecmat.cpp:61).
    #[inline]
    #[allow(clippy::too_many_arguments)]
    pub const fn new(
        m11: f64,
        m12: f64,
        m13: f64,
        m21: f64,
        m22: f64,
        m23: f64,
        m31: f64,
        m32: f64,
        m33: f64,
    ) -> Self {
        Self {
            m11,
            m12,
            m13,
            m21,
            m22,
            m23,
            m31,
            m32,
            m33,
        }
    }

    /// Index `(i, j)` → `data[i*3+j]` (`operator()(int,int)`, Vecmat.h:204).
    #[inline]
    pub fn get(&self, i: usize, j: usize) -> f64 {
        self.as_array()[i * 3 + j]
    }

    /// Mutable index.
    #[inline]
    pub fn get_mut(&mut self, i: usize, j: usize) -> &mut f64 {
        let arr = self.as_array_mut();
        &mut arr[i * 3 + j]
    }

    /// View as a flat `[f64; 9]` row-major array.
    #[inline]
    pub fn as_array(&self) -> &[f64; 9] {
        // Safe because Matrix3 is #[repr(C)] with 9 contiguous f64 fields.
        unsafe { &*(self as *const Self as *const [f64; 9]) }
    }

    /// Mutable view as a flat `[f64; 9]` row-major array.
    #[inline]
    pub fn as_array_mut(&mut self) -> &mut [f64; 9] {
        unsafe { &mut *(self as *mut Self as *mut [f64; 9]) }
    }

    /// Copy a vector into row `r` (`SetRow`, Vecmat.h:194).
    #[inline]
    pub fn set_row(&mut self, r: usize, v: Vec3) {
        let arr = self.as_array_mut();
        arr[r * 3] = v.x;
        arr[r * 3 + 1] = v.y;
        arr[r * 3 + 2] = v.z;
    }

    /// Copy a vector into column `c` (`SetCol`, Vecmat.h:199).
    #[inline]
    pub fn set_col(&mut self, c: usize, v: Vec3) {
        let arr = self.as_array_mut();
        arr[c] = v.x;
        arr[3 + c] = v.y;
        arr[6 + c] = v.z;
    }

    /// Get row `r` as a vector.
    #[inline]
    pub fn row(&self, r: usize) -> Vec3 {
        let arr = self.as_array();
        Vec3::new(arr[r * 3], arr[r * 3 + 1], arr[r * 3 + 2])
    }

    /// Get column `c` as a vector.
    #[inline]
    pub fn col(&self, c: usize) -> Vec3 {
        let arr = self.as_array();
        Vec3::new(arr[c], arr[3 + c], arr[6 + c])
    }

    /// Matrix × matrix product (`operator*(Matrix)`, Vecmat.cpp:126).
    #[inline]
    pub fn matmul(self, other: Self) -> Self {
        let a = self.as_array();
        let b = other.as_array();
        let mut r = [0.0f64; 9];
        for i in 0..3 {
            for j in 0..3 {
                r[i * 3 + j] = a[i * 3] * b[j] + a[i * 3 + 1] * b[3 + j] + a[i * 3 + 2] * b[6 + j];
            }
        }
        Self::from_array(&r)
    }

    /// Scale all entries (`operator*(double)`, Vecmat.cpp:135).
    #[inline]
    pub fn scaled(self, s: f64) -> Self {
        let a = self.as_array();
        let r = [
            a[0] * s,
            a[1] * s,
            a[2] * s,
            a[3] * s,
            a[4] * s,
            a[5] * s,
            a[6] * s,
            a[7] * s,
            a[8] * s,
        ];
        Self::from_array(&r)
    }

    /// `*self = A * *self` (`premul`, Vecmat.cpp:144).
    #[inline]
    pub fn premul(&mut self, a: Self) {
        *self = a.matmul(*self);
    }

    /// `*self = *self * A` (`postmul`, Vecmat.cpp:158).
    #[inline]
    pub fn postmul(&mut self, a: Self) {
        *self = self.matmul(a);
    }

    /// `*self = Aᵀ * *self` (`tpremul`, Vecmat.cpp:172).
    #[inline]
    pub fn tpremul(&mut self, a: Self) {
        *self = a.transp().matmul(*self);
    }

    /// `*self = *self * Aᵀ` (`tpostmul`, Vecmat.cpp:186).
    #[inline]
    pub fn tpostmul(&mut self, a: Self) {
        *self = self.matmul(a.transp());
    }

    /// Re-orthonormalise rows, trusting the pair not containing `axis`
    /// (`orthogonalise`, Vecmat.cpp:245). `axis` ∈ {0,1,2}.
    pub fn orthogonalise(&mut self, axis: usize) {
        let mut r0 = self.row(0);
        let mut r1 = self.row(1);
        let mut r2 = self.row(2);
        match axis {
            0 => {
                // Trust rows 1 & 2; rebuild row 0 = crossp(r1, r2).
                r1.unify();
                r2.unify();
                r0 = cross(r1, r2);
            }
            1 => {
                // Trust rows 0 & 2; rebuild row 1 = crossp(r2, r0).
                r0.unify();
                r2.unify();
                r1 = cross(r2, r0);
            }
            2 => {
                // Trust rows 0 & 1; rebuild row 2 = crossp(r0, r1).
                r0.unify();
                r1.unify();
                r2 = cross(r0, r1);
            }
            _ => panic!("orthogonalise axis must be 0, 1, or 2"),
        }
        self.set_row(0, r0);
        self.set_row(1, r1);
        self.set_row(2, r2);
    }

    /// Build from a flat `[f64; 9]` row-major array.
    #[inline]
    pub const fn from_array(a: &[f64; 9]) -> Self {
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

    /// Construct from a rotation vector (Euler angles) (`Set(Vector rot)`,
    /// Vecmat.cpp:96). `rot = (rx, ry, rz)` are rotation angles about x, y, z.
    pub fn from_euler(rot: Vec3) -> Self {
        let sinx = rot.x.sin();
        let cosx = rot.x.cos();
        let siny = rot.y.sin();
        let cosy = rot.y.cos();
        let sinz = rot.z.sin();
        let cosz = rot.z.cos();
        Self::new(
            cosy * cosz,
            cosy * sinz,
            -siny,
            sinx * siny * cosz - cosx * sinz,
            sinx * siny * sinz + cosx * cosz,
            sinx * cosy,
            cosx * siny * cosz + sinx * sinz,
            cosx * siny * sinz - sinx * cosz,
            cosx * cosy,
        )
    }
}

// --- Free functions (Vecmat.h:239-244, Vecmat.cpp:200-268) ---

/// Identity matrix (`IMatrix`).
#[inline]
pub const fn identity() -> Matrix3 {
    Matrix3::IDENTITY
}

/// `A * b` — matrix-vector product (`mul`, Vecmat.cpp:205).
#[inline]
pub fn mul(a: Matrix3, b: Vec3) -> Vec3 {
    Vec3::new(
        a.m11 * b.x + a.m12 * b.y + a.m13 * b.z,
        a.m21 * b.x + a.m22 * b.y + a.m23 * b.z,
        a.m31 * b.x + a.m32 * b.y + a.m33 * b.z,
    )
}

/// `Aᵀ * b` — transposed matrix-vector product (`tmul`, Vecmat.cpp:213).
#[inline]
pub fn tmul(a: Matrix3, b: Vec3) -> Vec3 {
    Vec3::new(
        a.m11 * b.x + a.m21 * b.y + a.m31 * b.z,
        a.m12 * b.x + a.m22 * b.y + a.m32 * b.z,
        a.m13 * b.x + a.m23 * b.y + a.m33 * b.z,
    )
}

/// Transpose (`transp`, Vecmat.cpp:238).
#[inline]
pub fn transp(a: Matrix3) -> Matrix3 {
    Matrix3::new(
        a.m11, a.m21, a.m31, a.m12, a.m22, a.m32, a.m13, a.m23, a.m33,
    )
}

impl Matrix3 {
    /// Transpose (method form for convenience).
    #[inline]
    pub fn transp(self) -> Matrix3 {
        transp(self)
    }
}

/// Classical adjugate/det inverse for 3×3 (`inv`, Vecmat.cpp:221). **No
/// singularity check** — returns inf/nan if the determinant is zero, matching
/// the C++ behaviour.
pub fn inv(a: Matrix3) -> Matrix3 {
    let det = a.m11 * (a.m22 * a.m33 - a.m32 * a.m23) - a.m12 * (a.m21 * a.m33 - a.m31 * a.m23)
        + a.m13 * (a.m21 * a.m32 - a.m31 * a.m22);
    let inv_det = 1.0 / det;
    Matrix3::new(
        (a.m22 * a.m33 - a.m32 * a.m23) * inv_det,
        (-a.m12 * a.m33 + a.m32 * a.m13) * inv_det,
        (a.m12 * a.m23 - a.m22 * a.m13) * inv_det,
        (-a.m21 * a.m33 + a.m31 * a.m23) * inv_det,
        (a.m11 * a.m33 - a.m31 * a.m13) * inv_det,
        (-a.m11 * a.m23 + a.m21 * a.m13) * inv_det,
        (a.m21 * a.m32 - a.m31 * a.m22) * inv_det,
        (-a.m11 * a.m32 + a.m31 * a.m12) * inv_det,
        (a.m11 * a.m22 - a.m21 * a.m12) * inv_det,
    )
}

// --- QR decomposition (3×3) (Vecmat.cpp:395-448) ---
//
// Faithful port of the Numerical-Recipes-style Householder QR. The loop runs
// k in [0,2) (two Householder steps for a 3×3), then copies the last diagonal.

/// Householder QR decomposition of a 3×3 matrix (`qrdcmp`, Vecmat.cpp:395).
///
/// On return `a` holds the accumulated Householder reflectors and R, `c` holds
/// the off-diagonal scaling factors, `d` the diagonal of R. Returns `false` if
/// the matrix is singular (matches the C++ `sing` flag).
pub fn qrdcmp(a: &mut Matrix3, c: &mut Vec3, d: &mut Vec3) -> bool {
    let mut sing = false;

    for k in 0..2 {
        let mut scale: f64 = 0.0;
        for i in k..3 {
            scale = scale.max(a.get(i, k).abs());
        }
        if scale == 0.0 {
            sing = true;
            c.as_arr_mut()[k] = 0.0;
            d.as_arr_mut()[k] = 0.0;
        } else {
            for i in k..3 {
                *a.get_mut(i, k) /= scale;
            }
            let mut sum = 0.0;
            for i in k..3 {
                sum += a.get(i, k) * a.get(i, k);
            }
            let sigma = if a.get(k, k) < 0.0 {
                -sum.sqrt()
            } else {
                sum.sqrt()
            };
            *a.get_mut(k, k) += sigma;
            c.as_arr_mut()[k] = sigma * a.get(k, k);
            d.as_arr_mut()[k] = -scale * sigma;
            for j in (k + 1)..3 {
                sum = 0.0;
                for i in k..3 {
                    sum += a.get(i, k) * a.get(i, j);
                }
                let tau = sum / c.as_arr()[k];
                for i in k..3 {
                    *a.get_mut(i, j) -= tau * a.get(i, k);
                }
            }
        }
    }
    d.as_arr_mut()[2] = a.get(2, 2);
    if d.as_arr()[2] == 0.0 {
        sing = true;
    }
    !sing
}

/// Solve `A·x = b` from QR factors produced by [`qrdcmp`] (`qrsolv`,
/// Vecmat.cpp:430). `a`, `c`, `d` are the outputs of `qrdcmp`; `b` is replaced
/// in place by the solution `x`.
pub fn qrsolv(a: Matrix3, c: Vec3, d: Vec3, b: &mut Vec3) {
    // Apply the two Householder reflectors (Qᵀ·b).
    for j in 0..2 {
        let mut sum = 0.0;
        for i in j..3 {
            sum += a.get(i, j) * b.as_arr()[i];
        }
        let tau = sum / c.as_arr()[j];
        for i in j..3 {
            b.as_arr_mut()[i] -= tau * a.get(i, j);
        }
    }
    // Back-substitution on R.
    b.as_arr_mut()[2] /= d.as_arr()[2];
    for i in (0..2).rev() {
        let mut sum = 0.0;
        for j in (i + 1)..3 {
            sum += a.get(i, j) * b.as_arr()[j];
        }
        b.as_arr_mut()[i] = (b.as_arr()[i] - sum) / d.as_arr()[i];
    }
}

// Helper trait for indexed array access on Vec3 in QR routines.
trait Vec3ArrExt {
    fn as_arr(&self) -> &[f64; 3];
    fn as_arr_mut(&mut self) -> &mut [f64; 3];
}
impl Vec3ArrExt for Vec3 {
    #[inline]
    fn as_arr(&self) -> &[f64; 3] {
        unsafe { &*(self as *const Vec3 as *const [f64; 3]) }
    }
    #[inline]
    fn as_arr_mut(&mut self) -> &mut [f64; 3] {
        unsafe { &mut *(self as *mut Vec3 as *mut [f64; 3]) }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn matmul_identity() {
        let m = Matrix3::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        let p = m.matmul(Matrix3::IDENTITY);
        assert_eq!(p, m);
    }

    #[test]
    fn mul_tmul_transpose_relation() {
        let m = Matrix3::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        let v = Vec3::new(1.0, 0.0, 0.0);
        // mul(m, v) picks row 0; tmul(mᵀ, v) should equal mul(m, v).
        assert_eq!(mul(m, v), tmul(m.transp(), v));
    }

    #[test]
    fn inv_roundtrip() {
        let m = Matrix3::new(4.0, 7.0, 0.0, 2.0, 6.0, 0.0, 0.0, 0.0, 1.0);
        let mi = inv(m);
        let p = m.matmul(mi);
        assert!((p.m11 - 1.0).abs() < 1e-9);
        assert!((p.m22 - 1.0).abs() < 1e-9);
        assert!((p.m33 - 1.0).abs() < 1e-9);
        assert!(p.m12.abs() < 1e-9);
    }

    #[test]
    fn qr_solve_matches_direct() {
        // Solve A·x = b via QR and compare to the known solution.
        let mut a = Matrix3::new(4.0, 7.0, 1.0, 2.0, 6.0, 2.0, 1.0, 1.0, 3.0);
        let mut c = Vec3::ZERO;
        let mut d = Vec3::ZERO;
        let nonsing = qrdcmp(&mut a, &mut c, &mut d);
        assert!(nonsing);
        let mut b = Vec3::new(12.0, 10.0, 5.0);
        qrsolv(a, c, d, &mut b);

        // Verify: original_A · b ≈ (12, 10, 5).
        let orig = Matrix3::new(4.0, 7.0, 1.0, 2.0, 6.0, 2.0, 1.0, 1.0, 3.0);
        let r = mul(orig, b);
        assert!(
            (r - Vec3::new(12.0, 10.0, 5.0)).length() < 1e-9,
            "got {:?}",
            r
        );
    }

    #[test]
    fn orthogonalise_axis0() {
        // Slightly perturbed identity.
        let mut m = Matrix3::new(1.0, 0.01, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
        m.orthogonalise(0);
        // Rows 1 and 2 should be unchanged units.
        assert!((m.row(1) - Vec3::new(0.0, 1.0, 0.0)).length() < 1e-9);
        assert!((m.row(2) - Vec3::new(0.0, 0.0, 1.0)).length() < 1e-9);
        // Row 0 = cross(row1, row2) = (1,0,0).
        assert!((m.row(0) - Vec3::new(1.0, 0.0, 0.0)).length() < 1e-9);
    }
}
