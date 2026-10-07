//! Rust FFI bindings to the C++ dynamics oracle.
//!
//! - `cpp/shim.cpp`: Orbiter-derived algorithms (gravity, Pines, Kepler, RK/SY).
//! - `cpp/rocket_aero.cpp`: wave/induced from OrbiterAPI; body+fin synthesis
//!   second-implements `AERO.md` / `rocket.rs`.
//!
//! Bindings exist only for property tests in `orbitx-dynamics`.

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
    pub fn ox_euler_inv_full(
        taux: c_double,
        tauy: c_double,
        tauz: c_double,
        wx: c_double,
        wy: c_double,
        wz: c_double,
        px: c_double,
        py: c_double,
        pz: c_double,
        ax: *mut c_double,
        ay: *mut c_double,
        az: *mut c_double,
    );
    pub fn ox_euler_inv_simple(
        taux: c_double,
        tauy: c_double,
        tauz: c_double,
        px: c_double,
        py: c_double,
        pz: c_double,
        ax: *mut c_double,
        ay: *mut c_double,
        az: *mut c_double,
    );
    pub fn ox_euler_full(
        odx: c_double,
        ody: c_double,
        odz: c_double,
        wx: c_double,
        wy: c_double,
        wz: c_double,
        px: c_double,
        py: c_double,
        pz: c_double,
        tx: *mut c_double,
        ty: *mut c_double,
        tz: *mut c_double,
    );
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

/// `EulerInv_full` (Rigidbody.cpp:468-481) via C++ oracle.
pub fn euler_inv_full(tau: [f64; 3], omega: [f64; 3], pmi: [f64; 3]) -> [f64; 3] {
    let (mut ax, mut ay, mut az) = (0.0, 0.0, 0.0);
    unsafe {
        ox_euler_inv_full(
            tau[0], tau[1], tau[2], omega[0], omega[1], omega[2], pmi[0], pmi[1], pmi[2],
            &mut ax, &mut ay, &mut az,
        );
    }
    [ax, ay, az]
}

/// `EulerInv_simple` (Rigidbody.cpp:485-497) via C++ oracle.
pub fn euler_inv_simple(tau: [f64; 3], pmi: [f64; 3]) -> [f64; 3] {
    let (mut ax, mut ay, mut az) = (0.0, 0.0, 0.0);
    unsafe {
        ox_euler_inv_simple(
            tau[0], tau[1], tau[2], pmi[0], pmi[1], pmi[2],
            &mut ax, &mut ay, &mut az,
        );
    }
    [ax, ay, az]
}

/// `Euler_full` (Rigidbody.cpp:458-464) via C++ oracle.
pub fn euler_full(omegadot: [f64; 3], omega: [f64; 3], pmi: [f64; 3]) -> [f64; 3] {
    let (mut tx, mut ty, mut tz) = (0.0, 0.0, 0.0);
    unsafe {
        ox_euler_full(
            omegadot[0], omegadot[1], omegadot[2], omega[0], omega[1], omega[2], pmi[0], pmi[1],
            pmi[2], &mut tx, &mut ty, &mut tz,
        );
    }
    [tx, ty, tz]
}

// ===========================================================
// RK4 integrator (linear only, BodyIntegrator.cpp RKdrv_LinAng)
// ===========================================================

/// Force callback type: `(pos[3], vel[3], tfrac) → acc[3]`.
pub type ForceCallback = extern "C" fn(
    x: c_double,
    y: c_double,
    z: c_double,
    vx: c_double,
    vy: c_double,
    vz: c_double,
    tfrac: c_double,
    ax: *mut c_double,
    ay: *mut c_double,
    az: *mut c_double,
);

extern "C" {
    pub fn ox_set_force_callback(cb: ForceCallback);
    pub fn ox_rk2_step(
        x: c_double,
        y: c_double,
        z: c_double,
        vx: c_double,
        vy: c_double,
        vz: c_double,
        h: c_double,
        ox: *mut c_double,
        oy: *mut c_double,
        oz: *mut c_double,
        ovx: *mut c_double,
        ovy: *mut c_double,
        ovz: *mut c_double,
    );
    pub fn ox_rk4_step(
        x: c_double,
        y: c_double,
        z: c_double,
        vx: c_double,
        vy: c_double,
        vz: c_double,
        h: c_double,
        ox: *mut c_double,
        oy: *mut c_double,
        oz: *mut c_double,
        ovx: *mut c_double,
        ovy: *mut c_double,
        ovz: *mut c_double,
    );
    pub fn ox_rk_drv_step(
        x: c_double,
        y: c_double,
        z: c_double,
        vx: c_double,
        vy: c_double,
        vz: c_double,
        h: c_double,
        n: c_int,
        alpha: *const c_double,
        beta: *const c_double,
        gamma: *const c_double,
        ox: *mut c_double,
        oy: *mut c_double,
        oz: *mut c_double,
        ovx: *mut c_double,
        ovy: *mut c_double,
        ovz: *mut c_double,
    );
    pub fn ox_sy_step(
        x: c_double,
        y: c_double,
        z: c_double,
        vx: c_double,
        vy: c_double,
        vz: c_double,
        h: c_double,
        ndrift: c_int,
        nkick: c_int,
        c: *const c_double,
        d: *const c_double,
        ox: *mut c_double,
        oy: *mut c_double,
        oz: *mut c_double,
        ovx: *mut c_double,
        ovy: *mut c_double,
        ovz: *mut c_double,
    );
}

/// Register a force callback for subsequent integrator calls.
pub fn set_force_callback(cb: ForceCallback) {
    unsafe { ox_set_force_callback(cb) };
}

/// RK2 step for linear dynamics (pos/vel only) via C++ oracle.
///
/// Requires a force callback previously registered via [`set_force_callback`].
/// Returns the new `(pos, vel)` as `([x,y,z], [vx,vy,vz])`.
pub fn rk2_step_linear(pos: [f64; 3], vel: [f64; 3], h: f64) -> ([f64; 3], [f64; 3]) {
    let (mut ox, mut oy, mut oz) = (0.0, 0.0, 0.0);
    let (mut ovx, mut ovy, mut ovz) = (0.0, 0.0, 0.0);
    unsafe {
        ox_rk2_step(
            pos[0], pos[1], pos[2], vel[0], vel[1], vel[2], h,
            &mut ox, &mut oy, &mut oz, &mut ovx, &mut ovy, &mut ovz,
        );
    }
    ([ox, oy, oz], [ovx, ovy, ovz])
}

/// RK4 step for linear dynamics (pos/vel only) via C++ oracle.
///
/// Requires a force callback previously registered via [`set_force_callback`].
/// Returns the new `(pos, vel)` as `([x,y,z], [vx,vy,vz])`.
pub fn rk4_step_linear(pos: [f64; 3], vel: [f64; 3], h: f64) -> ([f64; 3], [f64; 3]) {
    let (mut ox, mut oy, mut oz) = (0.0, 0.0, 0.0);
    let (mut ovx, mut ovy, mut ovz) = (0.0, 0.0, 0.0);
    unsafe {
        ox_rk4_step(
            pos[0], pos[1], pos[2], vel[0], vel[1], vel[2], h,
            &mut ox, &mut oy, &mut oz, &mut ovx, &mut ovy, &mut ovz,
        );
    }
    ([ox, oy, oz], [ovx, ovy, ovz])
}

/// Generic RK driver step for linear dynamics (pos/vel only) via C++ oracle.
///
/// This is used for RK5 through RK8. The `alpha`, `beta`, `gamma` arrays
/// define the Butcher tableau. `n` is the number of stages.
///
/// Requires a force callback previously registered via [`set_force_callback`].
/// Returns the new `(pos, vel)` as `([x,y,z], [vx,vy,vz])`.
pub fn rk_drv_step_linear(
    pos: [f64; 3],
    vel: [f64; 3],
    h: f64,
    n: usize,
    alpha: &[f64],
    beta: &[f64],
    gamma: &[f64],
) -> ([f64; 3], [f64; 3]) {
    let (mut ox, mut oy, mut oz) = (0.0, 0.0, 0.0);
    let (mut ovx, mut ovy, mut ovz) = (0.0, 0.0, 0.0);
    unsafe {
        ox_rk_drv_step(
            pos[0], pos[1], pos[2], vel[0], vel[1], vel[2], h,
            n as c_int,
            alpha.as_ptr(),
            beta.as_ptr(),
            gamma.as_ptr(),
            &mut ox, &mut oy, &mut oz, &mut ovx, &mut ovy, &mut ovz,
        );
    }
    ([ox, oy, oz], [ovx, ovy, ovz])
}

/// Symplectic (Yoshida) step for linear dynamics (pos/vel only) via C++ oracle.
///
/// The `c` array contains drift fractions (length `ndrift = nkick + 1`),
/// the `d` array contains kick fractions (length `nkick`).
///
/// Requires a force callback previously registered via [`set_force_callback`].
/// Returns the new `(pos, vel)` as `([x,y,z], [vx,vy,vz])`.
pub fn sy_step_linear(
    pos: [f64; 3],
    vel: [f64; 3],
    h: f64,
    c: &[f64],
    d: &[f64],
) -> ([f64; 3], [f64; 3]) {
    let (mut ox, mut oy, mut oz) = (0.0, 0.0, 0.0);
    let (mut ovx, mut ovy, mut ovz) = (0.0, 0.0, 0.0);
    unsafe {
        ox_sy_step(
            pos[0], pos[1], pos[2], vel[0], vel[1], vel[2], h,
            c.len() as c_int,
            d.len() as c_int,
            c.as_ptr(),
            d.as_ptr(),
            &mut ox, &mut oy, &mut oz, &mut ovx, &mut ovy, &mut ovz,
        );
    }
    ([ox, oy, oz], [ovx, ovy, ovz])
}

// ===========================================================
// Product rocket aero (AERO.md / rocket.rs)
// ===========================================================

/// POD mirror of C `OxLiftingSurface` (`oracle.h`).
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct OxLiftingSurface {
    pub ref_pos_x: c_double,
    pub ref_pos_y: c_double,
    pub ref_pos_z: c_double,
    pub normal_x: c_double,
    pub normal_y: c_double,
    pub normal_z: c_double,
    pub chord_x: c_double,
    pub chord_y: c_double,
    pub chord_z: c_double,
    pub area: c_double,
    pub aspect_ratio: c_double,
    pub cl_alpha: c_double,
    pub cd0: c_double,
    pub alpha_stall0: c_double,
    pub kind: c_int,
    pub deploy: c_double,
    pub leeward_sheltered: c_int,
}

/// POD mirror of C `OxAeroForces` (`oracle.h`).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct OxAeroForces {
    pub force_x: c_double,
    pub force_y: c_double,
    pub force_z: c_double,
    pub torque_x: c_double,
    pub torque_y: c_double,
    pub torque_z: c_double,
    pub mach: c_double,
    pub dynamic_pressure: c_double,
    pub drag_force: c_double,
    pub cd_eff: c_double,
    pub lift_force: c_double,
}

extern "C" {
    pub fn ox_wave_drag(
        mach: c_double,
        m1: c_double,
        m2: c_double,
        m3: c_double,
        cmax: c_double,
    ) -> c_double;
    pub fn ox_induced_drag(cl: c_double, aspect_ratio: c_double, oswald: c_double) -> c_double;
    pub fn ox_alpha_stall_mach(alpha_stall0: c_double, mach: c_double) -> c_double;
    pub fn ox_grid_eta(mach: c_double) -> c_double;
    pub fn ox_slew_deploy(
        deploy: c_double,
        deploy_target: c_double,
        deploy_rate: c_double,
        dt: c_double,
    ) -> c_double;
    pub fn ox_weighted_area(
        area_x: c_double,
        area_y: c_double,
        area_z: c_double,
        vx: c_double,
        vy: c_double,
        vz: c_double,
    ) -> c_double;
    pub fn ox_side_area(
        area_x: c_double,
        area_y: c_double,
        area_z: c_double,
        avx: c_double,
        avy: c_double,
        avz: c_double,
    ) -> c_double;
    pub fn ox_fin_local_alpha(
        avx: c_double,
        avy: c_double,
        avz: c_double,
        nx: c_double,
        ny: c_double,
        nz: c_double,
        cx: c_double,
        cy: c_double,
        cz: c_double,
    ) -> c_double;
    pub fn ox_moment_about_cg(
        fx: c_double,
        fy: c_double,
        fz: c_double,
        px: c_double,
        py: c_double,
        pz: c_double,
        cgx: c_double,
        cgy: c_double,
        cgz: c_double,
        tx: *mut c_double,
        ty: *mut c_double,
        tz: *mut c_double,
    );
    pub fn ox_compute_body_aero(
        avx: c_double,
        avy: c_double,
        avz: c_double,
        wx: c_double,
        wy: c_double,
        wz: c_double,
        rho: c_double,
        sound_speed: c_double,
        area_x: c_double,
        area_y: c_double,
        area_z: c_double,
        copx: c_double,
        copy: c_double,
        copz: c_double,
        cgx: c_double,
        cgy: c_double,
        cgz: c_double,
        cd0: c_double,
        cn_alpha: c_double,
        pitch_damp: c_double,
        yaw_damp: c_double,
        roll_damp: c_double,
        cd_mach_m: *const c_double,
        cd_mach_cd: *const c_double,
        n_cd: c_int,
        out: *mut OxAeroForces,
    );
    pub fn ox_compute_rocket_aero(
        avx: c_double,
        avy: c_double,
        avz: c_double,
        wx: c_double,
        wy: c_double,
        wz: c_double,
        rho: c_double,
        sound_speed: c_double,
        area_x: c_double,
        area_y: c_double,
        area_z: c_double,
        copx: c_double,
        copy: c_double,
        copz: c_double,
        cgx: c_double,
        cgy: c_double,
        cgz: c_double,
        cd0: c_double,
        cn_alpha: c_double,
        pitch_damp: c_double,
        yaw_damp: c_double,
        roll_damp: c_double,
        cd_mach_m: *const c_double,
        cd_mach_cd: *const c_double,
        n_cd: c_int,
        surfaces: *const OxLiftingSurface,
        n_surf: c_int,
        out: *mut OxAeroForces,
    );
}

pub fn wave_drag(mach: f64, m1: f64, m2: f64, m3: f64, cmax: f64) -> f64 {
    unsafe { ox_wave_drag(mach, m1, m2, m3, cmax) }
}

pub fn induced_drag(cl: f64, aspect_ratio: f64, oswald: f64) -> f64 {
    unsafe { ox_induced_drag(cl, aspect_ratio, oswald) }
}

pub fn alpha_stall_mach(alpha_stall0: f64, mach: f64) -> f64 {
    unsafe { ox_alpha_stall_mach(alpha_stall0, mach) }
}

pub fn grid_eta(mach: f64) -> f64 {
    unsafe { ox_grid_eta(mach) }
}

pub fn slew_deploy(deploy: f64, deploy_target: f64, deploy_rate: f64, dt: f64) -> f64 {
    unsafe { ox_slew_deploy(deploy, deploy_target, deploy_rate, dt) }
}

pub fn weighted_area(areas: [f64; 3], vhat: [f64; 3]) -> f64 {
    unsafe { ox_weighted_area(areas[0], areas[1], areas[2], vhat[0], vhat[1], vhat[2]) }
}

pub fn side_area(areas: [f64; 3], airvel: [f64; 3]) -> f64 {
    unsafe { ox_side_area(areas[0], areas[1], areas[2], airvel[0], airvel[1], airvel[2]) }
}

pub fn fin_local_alpha(airvel: [f64; 3], normal: [f64; 3], chord: [f64; 3]) -> f64 {
    unsafe {
        ox_fin_local_alpha(
            airvel[0], airvel[1], airvel[2], normal[0], normal[1], normal[2], chord[0],
            chord[1], chord[2],
        )
    }
}

pub fn moment_about_cg(force: [f64; 3], point: [f64; 3], cg: [f64; 3]) -> [f64; 3] {
    let (mut tx, mut ty, mut tz) = (0.0, 0.0, 0.0);
    unsafe {
        ox_moment_about_cg(
            force[0], force[1], force[2], point[0], point[1], point[2], cg[0], cg[1], cg[2],
            &mut tx, &mut ty, &mut tz,
        );
    }
    [tx, ty, tz]
}

fn split_cd_mach(cd_mach: &[(f64, f64)]) -> (Vec<f64>, Vec<f64>) {
    let m: Vec<f64> = cd_mach.iter().map(|(a, _)| *a).collect();
    let cd: Vec<f64> = cd_mach.iter().map(|(_, b)| *b).collect();
    (m, cd)
}

pub fn compute_body_aero(
    airvel: [f64; 3],
    omega: [f64; 3],
    rho: f64,
    sound_speed: f64,
    areas: [f64; 3],
    body_cop: [f64; 3],
    cg: [f64; 3],
    cd0: f64,
    cn_alpha: f64,
    pitch_damp: f64,
    yaw_damp: f64,
    roll_damp: f64,
    cd_mach: &[(f64, f64)],
) -> OxAeroForces {
    let (m, cd) = split_cd_mach(cd_mach);
    let mut out = OxAeroForces::default();
    unsafe {
        ox_compute_body_aero(
            airvel[0],
            airvel[1],
            airvel[2],
            omega[0],
            omega[1],
            omega[2],
            rho,
            sound_speed,
            areas[0],
            areas[1],
            areas[2],
            body_cop[0],
            body_cop[1],
            body_cop[2],
            cg[0],
            cg[1],
            cg[2],
            cd0,
            cn_alpha,
            pitch_damp,
            yaw_damp,
            roll_damp,
            m.as_ptr(),
            cd.as_ptr(),
            m.len() as c_int,
            &mut out,
        );
    }
    out
}

pub fn compute_rocket_aero(
    airvel: [f64; 3],
    omega: [f64; 3],
    rho: f64,
    sound_speed: f64,
    areas: [f64; 3],
    body_cop: [f64; 3],
    cg: [f64; 3],
    cd0: f64,
    cn_alpha: f64,
    pitch_damp: f64,
    yaw_damp: f64,
    roll_damp: f64,
    cd_mach: &[(f64, f64)],
    surfaces: &[OxLiftingSurface],
) -> OxAeroForces {
    let (m, cd) = split_cd_mach(cd_mach);
    let mut out = OxAeroForces::default();
    unsafe {
        ox_compute_rocket_aero(
            airvel[0],
            airvel[1],
            airvel[2],
            omega[0],
            omega[1],
            omega[2],
            rho,
            sound_speed,
            areas[0],
            areas[1],
            areas[2],
            body_cop[0],
            body_cop[1],
            body_cop[2],
            cg[0],
            cg[1],
            cg[2],
            cd0,
            cn_alpha,
            pitch_damp,
            yaw_damp,
            roll_damp,
            m.as_ptr(),
            cd.as_ptr(),
            m.len() as c_int,
            surfaces.as_ptr(),
            surfaces.len() as c_int,
            &mut out,
        );
    }
    out
}
