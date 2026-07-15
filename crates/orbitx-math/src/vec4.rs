//! 4-element vector — mirrors `class Vector4` (Vecmat.h:249-276).
//!
//! Plain 4-element container with no arithmetic operators (unlike `Vec3`); used
//! only with the Matrix4 QR routines.

/// 4-element vector (`class Vector4`, Vecmat.h:249).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct Vec4 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

impl Vec4 {
    pub const ZERO: Self = Self::new(0.0, 0.0, 0.0, 0.0);

    /// 4-scalar constructor (`Vector4(double x4)`, Vecmat.h:254).
    #[inline]
    pub const fn new(x: f64, y: f64, z: f64, w: f64) -> Self {
        Self { x, y, z, w }
    }

    /// Indexing matching `operator()(int i)` → `data[i]` (Vecmat.h:266,269).
    #[inline]
    pub fn get(&self, i: usize) -> f64 {
        self.as_array()[i]
    }

    #[inline]
    pub fn get_mut(&mut self, i: usize) -> &mut f64 {
        &mut self.as_array_mut()[i]
    }

    #[inline]
    pub fn as_array(&self) -> &[f64; 4] {
        unsafe { &*(self as *const Self as *const [f64; 4]) }
    }

    #[inline]
    pub fn as_array_mut(&mut self) -> &mut [f64; 4] {
        unsafe { &mut *(self as *mut Self as *mut [f64; 4]) }
    }
}
