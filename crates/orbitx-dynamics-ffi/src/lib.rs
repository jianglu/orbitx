//! Rust FFI bindings to the C++ dynamics oracle.
//!
//! The oracle (`cpp/shim.cpp`) re-implements key dynamics algorithms as free
//! functions, copied from Orbiter's `Psys.cpp`, `PinesGrav.cpp`, and
//! `Element.cpp`. These bindings exist only for property tests in
//! `orbitx-dynamics`.

use std::os::raw::{c_double, c_int};

extern "C" {
    pub fn ox_single_gacc(
        rx: c_double,
        ry: c_double,
        rz: c_double,
        gm: c_double,
        ax: *mut c_double,
        ay: *mut c_double,
        az: *mut c_double,
    );
    pub fn ox_jcoeff_pert(
        rx: c_double,
        ry: c_double,
        rz: c_double,
        body_size: c_double,
        gm: c_double,
        jcoeff: *const c_double,
        nj: c_int,
        ax: *mut c_double,
        ay: *mut c_double,
        az: *mut c_double,
    );
    pub fn ox_pines_accel(
        rx: c_double,
        ry: c_double,
        rz: c_double,
        ref_rad: c_double,
        gm: c_double,
        c: *const c_double,
        s: *const c_double,
        cs_len: c_int,
        max_degree: c_int,
        max_order: c_int,
        ax: *mut c_double,
        ay: *mut c_double,
        az: *mut c_double,
    );
    pub fn ox_ecc_anomaly(
        ma: c_double,
        e: c_double,
        ea0_in: c_double,
        ma0_in: c_double,
    ) -> c_double;
}

// --- High-level wrappers ---

/// Point-mass gravity acceleration via C++ oracle.
pub fn single_gacc(rpos: [f64; 3], gm: f64) -> [f64; 3] {
    let (mut ax, mut ay, mut az) = (0.0, 0.0, 0.0);
    unsafe {
        ox_single_gacc(rpos[0], rpos[1], rpos[2], gm, &mut ax, &mut ay, &mut az);
    }
    [ax, ay, az]
}

/// J2/J3/J4 zonal perturbation via C++ oracle.
pub fn jcoeff_pert(rpos: [f64; 3], body_size: f64, gm: f64, jcoeff: &[f64]) -> [f64; 3] {
    let (mut ax, mut ay, mut az) = (0.0, 0.0, 0.0);
    unsafe {
        ox_jcoeff_pert(
            rpos[0],
            rpos[1],
            rpos[2],
            body_size,
            gm,
            jcoeff.as_ptr(),
            jcoeff.len() as c_int,
            &mut ax,
            &mut ay,
            &mut az,
        );
    }
    [ax, ay, az]
}

/// Pines spherical harmonic acceleration via C++ oracle.
pub fn pines_accel(
    rpos: [f64; 3],
    ref_rad: f64,
    gm: f64,
    c: &[f64],
    s: &[f64],
    max_degree: usize,
    max_order: usize,
) -> [f64; 3] {
    let (mut ax, mut ay, mut az) = (0.0, 0.0, 0.0);
    unsafe {
        ox_pines_accel(
            rpos[0],
            rpos[1],
            rpos[2],
            ref_rad,
            gm,
            c.as_ptr(),
            s.as_ptr(),
            c.len() as c_int,
            max_degree as c_int,
            max_order as c_int,
            &mut ax,
            &mut ay,
            &mut az,
        );
    }
    [ax, ay, az]
}

/// Kepler eccentric anomaly solver via C++ oracle.
pub fn ecc_anomaly(ma: f64, e: f64, ea0: f64, ma0: f64) -> f64 {
    unsafe { ox_ecc_anomaly(ma, e, ea0, ma0) }
}
