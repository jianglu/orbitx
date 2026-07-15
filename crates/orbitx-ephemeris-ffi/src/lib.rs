//! Rust FFI bindings to the C++ ephemeris oracle.
//!
//! The oracle (`cpp/shim.cpp`) re-implements the VSOP87 and ELP82 evaluation
//! algorithms as free functions, copied verbatim from Orbiter's
//! `Src/Celbody/Vsop87/Vsop87.cpp` and `Src/Celbody/Moon/ELP82.cpp`. This avoids
//! the `CELBODY`/`CELBODY2`/`ATMOSPHERE` class hierarchy and engine dependencies
//! while testing exactly the same numerical algorithms.
//!
//! These bindings exist only for property tests in `orbitx-ephemeris`.

use std::ffi::CString;
use std::os::raw::{c_char, c_int};

/// C++ `Sample` (CelBodyAPI.h:42-46): `{ t, rad, param[6] }`.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct CSample {
    pub t: f64,
    pub rad: f64,
    pub param: [f64; 6],
}

impl From<orbitx_ephemeris::Sample> for CSample {
    fn from(s: orbitx_ephemeris::Sample) -> Self {
        Self {
            t: s.t,
            rad: s.rad,
            param: s.param,
        }
    }
}

impl From<CSample> for orbitx_ephemeris::Sample {
    fn from(c: CSample) -> Self {
        Self {
            t: c.t,
            rad: c.rad,
            param: c.param,
        }
    }
}

/// Opaque handle to a C++ `VsopData` struct.
#[repr(C)]
pub struct CVsopData {
    _private: [u8; 0],
}

extern "C" {
    // VSOP87
    pub fn ox_vsop_create(sid: c_char, a0: f64, prec: f64, interval: f64) -> *mut CVsopData;
    pub fn ox_vsop_destroy(vd: *mut CVsopData);
    pub fn ox_vsop_read(vd: *mut CVsopData, path: *const c_char) -> c_int;
    pub fn ox_vsop_eval(vd: *const CVsopData, mjd: f64, ret: *mut f64);
    pub fn ox_vsop_fast_eval(vd: *mut CVsopData, simt: f64, ret: *mut f64);
    pub fn ox_vsop_set_mjd_ref(ref_mjd: f64);
    pub fn ox_vsop_get_mjd_ref() -> f64;

    // ELP82
    pub fn ox_elp_read(path: *const c_char, prec: f64) -> c_int;
    pub fn ox_elp_eval(mjd: f64, ret: *mut f64);

    // Interpolate
    pub fn ox_interpolate(t: f64, data: *mut f64, s0: *const CSample, s1: *const CSample);
}

// --- High-level Rust wrappers ---

/// RAII wrapper around a C++ VSOP87 oracle handle.
pub struct VsopOracle {
    handle: *mut CVsopData,
}

impl VsopOracle {
    /// Create a new VSOP87 oracle for the given series.
    ///
    /// `sid` is `'B'` (spherical) or `'E'` (rectangular barycentric).
    pub fn new(sid: char, a0: f64, prec: f64, interval: f64) -> Self {
        let handle = unsafe { ox_vsop_create(sid as c_char, a0, prec, interval) };
        assert!(!handle.is_null(), "ox_vsop_create returned null");
        Self { handle }
    }

    /// Load data from a `.dat` file. Returns `true` on success.
    pub fn read_data(&self, path: &str) -> bool {
        let cpath = CString::new(path).unwrap();
        let r = unsafe { ox_vsop_read(self.handle, cpath.as_ptr()) };
        r != 0
    }

    /// Evaluate at MJD, returning `[pos; 3] + [vel; 3]`.
    pub fn eval(&self, mjd: f64) -> [f64; 6] {
        let mut ret = [0.0; 6];
        unsafe { ox_vsop_eval(self.handle, mjd, ret.as_mut_ptr()) };
        ret
    }

    /// Fast-ephemeris evaluation at simulation time `simt` (seconds since J2000).
    pub fn fast_eval(&mut self, simt: f64) -> [f64; 6] {
        let mut ret = [0.0; 6];
        unsafe { ox_vsop_fast_eval(self.handle, simt, ret.as_mut_ptr()) };
        ret
    }

    /// Set the global MJD reference (for `oapiTime2MJD(simt) = mjd_ref + simt/86400`).
    pub fn set_mjd_ref(mjd_ref: f64) {
        unsafe { ox_vsop_set_mjd_ref(mjd_ref) };
    }
}

impl Drop for VsopOracle {
    fn drop(&mut self) {
        unsafe { ox_vsop_destroy(self.handle) };
    }
}

unsafe impl Send for VsopOracle {}

/// Load ELP82 data from a `.dat` file with the given precision.
pub fn elp_read_data(path: &str, prec: f64) -> bool {
    let cpath = CString::new(path).unwrap();
    let r = unsafe { ox_elp_read(cpath.as_ptr(), prec) };
    r == 0
}

/// Evaluate ELP82 at MJD, returning `[pos; 3] + [vel; 3]`.
pub fn elp_eval(mjd: f64) -> [f64; 6] {
    let mut ret = [0.0; 6];
    unsafe { ox_elp_eval(mjd, ret.as_mut_ptr()) };
    ret
}

/// Hermite interpolation via the C++ oracle.
pub fn interpolate(t: f64, s0: &CSample, s1: &CSample) -> [f64; 6] {
    let mut data = [0.0; 6];
    unsafe {
        ox_interpolate(
            t,
            data.as_mut_ptr(),
            s0 as *const CSample,
            s1 as *const CSample,
        )
    };
    data
}

// --- TASS17 ---

/// Opaque handle to a C++ `TasModel` struct.
#[repr(C)]
pub struct CTasModel {
    _private: [u8; 0],
}

extern "C" {
    pub fn ox_tass17_create() -> *mut CTasModel;
    pub fn ox_tass17_destroy(m: *mut CTasModel);
    pub fn ox_tass17_read(m: *mut CTasModel, path: *const c_char) -> c_int;
    pub fn ox_tass17_eval(m: *const CTasModel, jd: f64, isat: c_int, ret: *mut f64);
}

/// RAII wrapper around a C++ TASS17 oracle handle.
pub struct TasOracle {
    handle: *mut CTasModel,
}

impl TasOracle {
    /// Create a new TASS17 oracle.
    pub fn new() -> Self {
        let handle = unsafe { ox_tass17_create() };
        assert!(!handle.is_null(), "ox_tass17_create returned null");
        Self { handle }
    }

    /// Load data from a `.dat` file. Returns `true` on success.
    pub fn read_data(&self, path: &str) -> bool {
        let cpath = CString::new(path).unwrap();
        let r = unsafe { ox_tass17_read(self.handle, cpath.as_ptr()) };
        r != 0
    }

    /// Evaluate at Julian Date `jd` for satellite `isat` (0-7).
    pub fn eval(&self, jd: f64, isat: usize) -> [f64; 6] {
        let mut ret = [0.0; 6];
        unsafe { ox_tass17_eval(self.handle, jd, isat as c_int, ret.as_mut_ptr()) };
        ret
    }
}

impl Default for TasOracle {
    fn default() -> Self {
        Self::new()
    }
}

impl Drop for TasOracle {
    fn drop(&mut self) {
        unsafe { ox_tass17_destroy(self.handle) };
    }
}

unsafe impl Send for TasOracle {}
