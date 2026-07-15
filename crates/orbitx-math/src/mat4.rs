//! 4×4 matrix — mirrors `class Matrix4` (Vecmat.h:281-324).
//!
//! Row-major, 16 contiguous `f64`. Used for graphics projection and the QR
//! family of linear solvers. Faithful port of Vecmat.cpp:273-393.

use crate::vec4::Vec4;

/// 4×4 row-major matrix (`class Matrix4`, Vecmat.h:281).
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct Matrix4 {
    pub m11: f64,
    pub m12: f64,
    pub m13: f64,
    pub m14: f64,
    pub m21: f64,
    pub m22: f64,
    pub m23: f64,
    pub m24: f64,
    pub m31: f64,
    pub m32: f64,
    pub m33: f64,
    pub m34: f64,
    pub m41: f64,
    pub m42: f64,
    pub m43: f64,
    pub m44: f64,
}

impl Default for Matrix4 {
    fn default() -> Self {
        Self::ZERO
    }
}

impl Matrix4 {
    pub const ZERO: Self = Self {
        m11: 0.0,
        m12: 0.0,
        m13: 0.0,
        m14: 0.0,
        m21: 0.0,
        m22: 0.0,
        m23: 0.0,
        m24: 0.0,
        m31: 0.0,
        m32: 0.0,
        m33: 0.0,
        m34: 0.0,
        m41: 0.0,
        m42: 0.0,
        m43: 0.0,
        m44: 0.0,
    };

    /// Row-major construction from 16 scalars (`Matrix4(double x16)`).
    #[inline]
    #[allow(clippy::too_many_arguments)]
    pub const fn new(
        m11: f64,
        m12: f64,
        m13: f64,
        m14: f64,
        m21: f64,
        m22: f64,
        m23: f64,
        m24: f64,
        m31: f64,
        m32: f64,
        m33: f64,
        m34: f64,
        m41: f64,
        m42: f64,
        m43: f64,
        m44: f64,
    ) -> Self {
        Self {
            m11,
            m12,
            m13,
            m14,
            m21,
            m22,
            m23,
            m24,
            m31,
            m32,
            m33,
            m34,
            m41,
            m42,
            m43,
            m44,
        }
    }

    /// Index `(i, j)` → `data[i*4+j]` (`operator()(int,int)`, Vecmat.h:307).
    #[inline]
    pub fn get(&self, i: usize, j: usize) -> f64 {
        self.as_array()[i * 4 + j]
    }

    #[inline]
    pub fn get_mut(&mut self, i: usize, j: usize) -> &mut f64 {
        &mut self.as_array_mut()[i * 4 + j]
    }

    #[inline]
    pub fn as_array(&self) -> &[f64; 16] {
        unsafe { &*(self as *const Self as *const [f64; 16]) }
    }

    #[inline]
    pub fn as_array_mut(&mut self) -> &mut [f64; 16] {
        unsafe { &mut *(self as *mut Self as *mut [f64; 16]) }
    }

    #[inline]
    pub const fn from_array(a: &[f64; 16]) -> Self {
        Self {
            m11: a[0],
            m12: a[1],
            m13: a[2],
            m14: a[3],
            m21: a[4],
            m22: a[5],
            m23: a[6],
            m24: a[7],
            m31: a[8],
            m32: a[9],
            m33: a[10],
            m34: a[11],
            m41: a[12],
            m42: a[13],
            m43: a[14],
            m44: a[15],
        }
    }
}

// --- QR family (Vecmat.cpp:284-393) ---

/// Householder QR decomposition of a 4×4 matrix (`qrdcmp`, Vecmat.cpp:284).
/// Returns `false` if singular.
pub fn qrdcmp(a: &mut Matrix4, c: &mut Vec4, d: &mut Vec4) -> bool {
    let mut sing = false;

    for k in 0..3 {
        let mut scale: f64 = 0.0;
        for i in k..4 {
            scale = scale.max(a.get(i, k).abs());
        }
        if scale == 0.0 {
            sing = true;
            c.as_array_mut()[k] = 0.0;
            d.as_array_mut()[k] = 0.0;
        } else {
            for i in k..4 {
                *a.get_mut(i, k) /= scale;
            }
            let mut sum = 0.0;
            for i in k..4 {
                sum += a.get(i, k) * a.get(i, k);
            }
            let sigma = if a.get(k, k) < 0.0 {
                -sum.sqrt()
            } else {
                sum.sqrt()
            };
            *a.get_mut(k, k) += sigma;
            c.as_array_mut()[k] = sigma * a.get(k, k);
            d.as_array_mut()[k] = -scale * sigma;
            for j in (k + 1)..4 {
                sum = 0.0;
                for i in k..4 {
                    sum += a.get(i, k) * a.get(i, j);
                }
                let tau = sum / c.as_array()[k];
                for i in k..4 {
                    *a.get_mut(i, j) -= tau * a.get(i, k);
                }
            }
        }
    }
    d.as_array_mut()[3] = a.get(3, 3);
    if d.as_array()[3] == 0.0 {
        sing = true;
    }
    !sing
}

/// Solve `A·x = b` from QR factors of a 4×4 (`qrsolv`, Vecmat.cpp:319).
pub fn qrsolv(a: Matrix4, c: Vec4, d: Vec4, b: &mut Vec4) {
    for j in 0..3 {
        let mut sum = 0.0;
        for i in j..4 {
            sum += a.get(i, j) * b.as_array()[i];
        }
        let tau = sum / c.as_array()[j];
        for i in j..4 {
            b.as_array_mut()[i] -= tau * a.get(i, j);
        }
    }
    b.as_array_mut()[3] /= d.as_array()[3];
    for i in (0..3).rev() {
        let mut sum = 0.0;
        for j in (i + 1)..4 {
            sum += a.get(i, j) * b.as_array()[j];
        }
        b.as_array_mut()[i] = (b.as_array()[i] - sum) / d.as_array()[i];
    }
}

/// Alternate normalised-Householder QR factorisation (`QRFactorize`,
/// Vecmat.cpp:340). Used together with [`r_solve`] / [`qr_solve`].
pub fn qr_factorize(a: &mut Matrix4, _c: &mut Vec4, d: &mut Vec4) {
    for k in 0..4 {
        let mut sum = 0.0;
        for i in k..4 {
            sum += a.get(i, k) * a.get(i, k);
        }
        d.as_array_mut()[k] = if a.get(k, k) < 0.0 {
            -sum.sqrt()
        } else {
            sum.sqrt()
        };
        let b = (2.0 * d.as_array()[k] * (a.get(k, k) + d.as_array()[k])).sqrt();
        *a.get_mut(k, k) = (a.get(k, k) + d.as_array()[k]) / b;
        for i in (k + 1)..4 {
            *a.get_mut(i, k) /= b;
        }
        for j in (k + 1)..4 {
            sum = 0.0;
            for i in k..4 {
                sum += a.get(i, k) * a.get(i, j);
            }
            let f = 2.0 * sum;
            for i in k..4 {
                *a.get_mut(i, j) -= f * a.get(i, k);
            }
        }
    }
}

/// Back-solve `R·x = b` using the `d` from [`qr_factorize`] (`RSolve`,
/// Vecmat.cpp:363). **Note the `-d(i)` sign** (Vecmat.cpp:367,371).
pub fn r_solve(a: Matrix4, d: Vec4, b: &mut Vec4) {
    b.as_array_mut()[3] /= -d.as_array()[3];
    for i in (0..3).rev() {
        let mut sum = 0.0;
        for j in (i + 1)..4 {
            sum += a.get(i, j) * b.as_array()[j];
        }
        b.as_array_mut()[i] = (b.as_array()[i] - sum) / -d.as_array()[i];
    }
}

/// Full least-squares solve `A·x = b` from [`qr_factorize`] factors
/// (`QRSolve`, Vecmat.cpp:375).
pub fn qr_solve(a: Matrix4, _c: Vec4, d: Vec4, b: Vec4, x: &mut Vec4) {
    *x = b;
    for k in 0..4 {
        let mut sum = 0.0;
        for i in k..4 {
            sum += a.get(i, k) * x.as_array()[i];
        }
        sum *= 2.0;
        for i in k..4 {
            x.as_array_mut()[i] -= sum * a.get(i, k);
        }
    }
    r_solve(a, d, x);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn qr4_solve() {
        // Random-ish well-conditioned 4×4.
        let mut a = Matrix4::new(
            4.0, 1.0, 0.0, 0.0, 1.0, 3.0, 1.0, 0.0, 0.0, 1.0, 2.0, 1.0, 0.0, 0.0, 1.0, 5.0,
        );
        let mut c = Vec4::ZERO;
        let mut d = Vec4::ZERO;
        let nonsing = qrdcmp(&mut a, &mut c, &mut d);
        assert!(nonsing);
        let mut b = Vec4::new(5.0, 5.0, 4.0, 6.0);
        qrsolv(a, c, d, &mut b);
        // Verify: original · b ≈ (5,5,4,6).
        let orig = Matrix4::new(
            4.0, 1.0, 0.0, 0.0, 1.0, 3.0, 1.0, 0.0, 0.0, 1.0, 2.0, 1.0, 0.0, 0.0, 1.0, 5.0,
        );
        let r = Vec4::new(
            orig.m11 * b.x + orig.m12 * b.y + orig.m13 * b.z + orig.m14 * b.w,
            orig.m21 * b.x + orig.m22 * b.y + orig.m23 * b.z + orig.m24 * b.w,
            orig.m31 * b.x + orig.m32 * b.y + orig.m33 * b.z + orig.m34 * b.w,
            orig.m41 * b.x + orig.m42 * b.y + orig.m43 * b.z + orig.m44 * b.w,
        );
        assert!((r.x - 5.0).abs() < 1e-9);
        assert!((r.y - 5.0).abs() < 1e-9);
        assert!((r.z - 4.0).abs() < 1e-9);
        assert!((r.w - 6.0).abs() < 1e-9);
    }
}
