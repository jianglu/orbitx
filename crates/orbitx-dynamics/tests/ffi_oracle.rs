//! Property tests comparing orbitx-dynamics Rust implementation against the
//! C++ oracle (`orbitx-dynamics-ffi`).
//!
//! **注意**：积分器测试使用 C++ 全局回调（`g_force_cb`）和 Rust 全局静态
//! （`G_GM`），多线程并行会竞态。所有积分器测试通过 `INTEGRATOR_LOCK` 互斥锁
//! 序列化，确保正确性。

#![allow(clippy::approx_constant, clippy::excessive_precision)]

use std::sync::Mutex;

use orbitx_dynamics::kepler::Elements;
use orbitx_dynamics::pines::{nm, PinesModel, Vec3Pines};
use orbitx_dynamics::{
    alpha_stall_mach, compute_body_aero, compute_rocket_aero, euler_full, euler_inv_full,
    euler_inv_simple, fin_local_alpha, gacc_nbody, grid_eta, induced_drag, jcoeff_perturbation,
    moment_about_cg, side_area, single_gacc, slew_deploy, wave_drag, FinKind, GravBody,
    LiftingSurface, RocketAeroInput, RocketBodyAero, TriaxialAreas,
};
use orbitx_dynamics_ffi as ffi;
use orbitx_math::Vec3;
use proptest::prelude::*;

/// 全局互斥锁：序列化所有积分器测试（因 C++ oracle 使用全局 `g_force_cb`，
/// Rust 端使用 `static mut G_GM`，多线程不安全）。
static INTEGRATOR_LOCK: Mutex<()> = Mutex::new(());

const TOL: f64 = 1e-10;
const ATOL: f64 = 1e-12;

fn assert_close(a: f64, b: f64, msg: &str) {
    let diff = (a - b).abs();
    let maxmag = a.abs().max(b.abs());
    let allowed = TOL * maxmag + ATOL;
    assert!(
        diff <= allowed || (a.is_nan() && b.is_nan()),
        "{msg}: {a} vs {b} (diff={diff}, allowed={allowed})"
    );
}

fn assert_close3(a: &[f64; 3], b: &[f64; 3], ctx: &str) {
    for i in 0..3 {
        assert_close(a[i], b[i], &format!("{ctx}[{i}]"));
    }
}

// ===========================================================
// Point-mass gravity property tests
// ===========================================================

proptest! {
    #[test]
    fn prop_single_gacc(
        rx in -1e9_f64..1e9,
        ry in -1e9_f64..1e9,
        rz in -1e9_f64..1e9,
        gm in 1e10_f64..1e20,
    ) {
        // Skip near-zero positions
        prop_assume!(rx*rx + ry*ry + rz*rz > 1e6);

        let rpos = Vec3::new(rx, ry, rz);
        let rust = single_gacc(rpos, gm);
        let cpp = ffi::single_gacc([rx, ry, rz], gm);

        assert_close3(&[rust.x, rust.y, rust.z], &cpp, "single_gacc");
    }
}

// ===========================================================
// J2/J3/J4 property tests
// ===========================================================

proptest! {
    #[test]
    fn prop_jcoeff_pert(
        rx in 6.5e6_f64..1e8,
        rz in -1e8_f64..1e8,
    ) {
        let ry = 0.0_f64;
        let body_size = 6.37101e6_f64;
        let gm = 3.986e14_f64;
        let jcoeff = vec![1.0826e-3, -2.51e-6, -1.60e-6];

        let rpos = Vec3::new(rx, ry, rz);
        let rust = jcoeff_perturbation(rpos, body_size, gm, &jcoeff);
        let cpp = ffi::jcoeff_pert([rx, ry, rz], body_size, gm, &jcoeff);

        assert_close3(&[rust.x, rust.y, rust.z], &cpp, "jcoeff_pert");
    }
}

// ===========================================================
// Pines spherical harmonic gravity property tests
// ===========================================================

fn make_simple_pines_model() -> (PinesModel, Vec<f64>, Vec<f64>) {
    // A model with C(2,0) = J2 and C(3,0) = J3.
    let data = "6378.1363, 398600.4415, 0, 3, 3, 1, 0, 0\n\
                2, 0, -0.00108263, 0.0, 0.0, 0.0\n\
                3, 0, 2.54e-6, 0.0, 0.0, 0.0\n";
    let model = PinesModel::from_reader(data.as_bytes(), 3).unwrap();

    // Build flat C/S arrays matching the oracle's NM indexing.
    let max_idx = nm(5, 5);
    let mut c = vec![0.0_f64; max_idx + 1];
    let s = vec![0.0_f64; max_idx + 1];
    c[nm(2, 0)] = -0.00108263;
    c[nm(3, 0)] = 2.54e-6;

    (model, c, s)
}

proptest! {
    #[test]
    fn prop_pines_accel(
        x in -20000.0_f64..20000.0,
        y in -20000.0_f64..20000.0,
        z in 1000.0_f64..20000.0,
    ) {
        let (model, c, s) = make_simple_pines_model();
        let rpos = Vec3Pines::new(x, y, z);
        let rust = model.accel(rpos, model.degree, model.order);
        let cpp = ffi::pines_accel(
            [x, y, z],
            model.ref_rad,
            model.gm,
            &c,
            &s,
            model.degree,
            model.order,
        );

        assert_close3(&[rust.x, rust.y, rust.z], &cpp, "pines_accel");
    }
}

// ===========================================================
// Kepler EccAnomaly property tests
// ===========================================================

proptest! {
    #[test]
    fn prop_ecc_anomaly_closed(
        ma in 0.0_f64..6.28,
        e in 0.0_f64..0.95,
    ) {
        // Create elements with the given eccentricity by starting from
        // periapsis of an elliptical orbit.
        let mu: f64 = 3.986e14;
        let a = 8.0e6_f64;
        let r_pe = a * (1.0 - e);
        let v_pe = (mu * (2.0 / r_pe - 1.0 / a)).sqrt();
        let el = Elements::calculate(
            Vec3::new(r_pe, 0.0, 0.0),
            Vec3::new(0.0, 0.0, v_pe),
            mu,
            0.0,
        );

        let rust_ea = el.ecc_anomaly(ma);
        let cpp_ea = ffi::ecc_anomaly(ma, e, el.ecc_anm(), el.mean_anm());

        assert_close(rust_ea, cpp_ea, "ecc_anomaly");
    }
}

// ===========================================================
// N-body gravity property tests
// ===========================================================

proptest! {
    #[test]
    fn prop_nbody_gacc(
        gx in -1e9_f64..1e9,
        gy in -1e9_f64..1e9,
        gz in -1e9_f64..1e9,
    ) {
        prop_assume!(gx*gx + gy*gy + gz*gz > 1e10);

        let bodies = vec![
            GravBody {
                pos: Vec3::new(0.0, 0.0, 0.0),
                mass: 5.97e24,
                size: 6.371e6,
                jcoeff: vec![],
                rotation: None,
                pines: None,
            },
            GravBody {
                pos: Vec3::new(1.5e11, 0.0, 0.0),
                mass: 1.99e30,
                size: 6.96e8,
                jcoeff: vec![],
                rotation: None,
                pines: None,
            },
        ];

        let gpos = Vec3::new(gx, gy, gz);
        let rust = gacc_nbody(gpos, &bodies, None);

        // Compare against manual summation via the C++ single_gacc oracle.
        let mut cpp = [0.0_f64; 3];
        for body in &bodies {
            let rpos = [body.pos.x - gx, body.pos.y - gy, body.pos.z - gz];
            let acc = ffi::single_gacc(rpos, body.gm());
            for i in 0..3 {
                cpp[i] += acc[i];
            }
        }

        assert_close3(&[rust.x, rust.y, rust.z], &cpp, "nbody_gacc");
    }
}

// ===========================================================
// Rigid-body angular dynamics property tests (Rigidbody.cpp:458-511)
// ===========================================================

proptest! {
    #[test]
    fn prop_euler_inv_full(
        taux in -1e4_f64..1e4,
        tauy in -1e4_f64..1e4,
        tauz in -1e4_f64..1e4,
        wx in   -1.0_f64..1.0,
        wy in   -1.0_f64..1.0,
        wz in   -1.0_f64..1.0,
        px in    1e2_f64..1e6,
        py in    1e2_f64..1e6,
        pz in    1e2_f64..1e6,
    ) {
        let tau = Vec3::new(taux, tauy, tauz);
        let omega = Vec3::new(wx, wy, wz);
        let pmi = Vec3::new(px, py, pz);

        let rust = euler_inv_full(tau, omega, pmi);
        let cpp = ffi::euler_inv_full([taux, tauy, tauz], [wx, wy, wz], [px, py, pz]);

        assert_close3(&[rust.x, rust.y, rust.z], &cpp, "euler_inv_full");
    }

    #[test]
    fn prop_euler_inv_simple(
        taux in -1e4_f64..1e4,
        tauy in -1e4_f64..1e4,
        tauz in -1e4_f64..1e4,
        px in    1e2_f64..1e6,
        py in    1e2_f64..1e6,
        pz in    1e2_f64..1e6,
    ) {
        let tau = Vec3::new(taux, tauy, tauz);
        let pmi = Vec3::new(px, py, pz);

        let rust = euler_inv_simple(tau, pmi);
        let cpp = ffi::euler_inv_simple([taux, tauy, tauz], [px, py, pz]);

        assert_close3(&[rust.x, rust.y, rust.z], &cpp, "euler_inv_simple");
    }

    #[test]
    fn prop_euler_full(
        odx in -1e3_f64..1e3,
        ody in -1e3_f64..1e3,
        odz in -1e3_f64..1e3,
        wx in  -1.0_f64..1.0,
        wy in  -1.0_f64..1.0,
        wz in  -1.0_f64..1.0,
        px in   1e2_f64..1e6,
        py in   1e2_f64..1e6,
        pz in   1e2_f64..1e6,
    ) {
        let omegadot = Vec3::new(odx, ody, odz);
        let omega = Vec3::new(wx, wy, wz);
        let pmi = Vec3::new(px, py, pz);

        let rust = euler_full(omegadot, omega, pmi);
        let cpp = ffi::euler_full([odx, ody, odz], [wx, wy, wz], [px, py, pz]);

        assert_close3(&[rust.x, rust.y, rust.z], &cpp, "euler_full");
    }
}

// ===========================================================
// RK4 integrator property tests (BodyIntegrator.cpp RK4_LinAng)
// ===========================================================

// 全局 GM（供 extern "C" 回调使用，因 C ABI 不能捕获闭包）。
static mut G_GM: f64 = 1.0;

/// 点质量引力加速度回调（extern "C"，供 C++ oracle 调用）。
extern "C" fn point_mass_acc(
    x: f64,
    y: f64,
    z: f64,
    _vx: f64,
    _vy: f64,
    _vz: f64,
    _tfrac: f64,
    ax: *mut f64,
    ay: *mut f64,
    az: *mut f64,
) {
    unsafe {
        let gm = G_GM;
        let r2 = x * x + y * y + z * z;
        let r = r2.sqrt();
        let f = -gm / (r2 * r);
        *ax = x * f;
        *ay = y * f;
        *az = z * f;
    }
}

/// 椭圆轨道加速度回调（含 J2 扰动，用于区分积分器阶数）。
extern "C" fn j2_acc(
    x: f64,
    y: f64,
    z: f64,
    _vx: f64,
    _vy: f64,
    _vz: f64,
    _tfrac: f64,
    ax: *mut f64,
    ay: *mut f64,
    az: *mut f64,
) {
    unsafe {
        let gm = G_GM;
        let r2 = x * x + y * y + z * z;
        let r = r2.sqrt();
        // 点质量引力
        let f = -gm / (r2 * r);
        let mut fx = x * f;
        let mut fy = y * f;
        let mut fz = z * f;
        // J2 扰动（地球扁率）
        let re = 6.37101e6;
        let j2 = 1.0826e-3;
        let zr2 = (z / r) * (z / r);
        let rr5 = re * re / (r2 * r2 * r);
        let fj = -1.5 * j2 * gm * rr5;
        fx += fj * x * (1.0 - 5.0 * zr2);
        fy += fj * y * (1.0 - 5.0 * zr2);
        fz += fj * z * (3.0 - 5.0 * zr2);
        *ax = fx;
        *ay = fy;
        *az = fz;
    }
}

/// 辅助：构建圆轨道初值。
fn circular_orbit_ic(r0: f64, theta: f64, gm: f64) -> ([f64; 3], [f64; 3]) {
    let px = r0 * theta.cos();
    let pz = r0 * theta.sin();
    let vc = (gm / r0).sqrt();
    let vx = -vc * theta.sin();
    let vz = vc * theta.cos();
    ([px, 0.0, pz], [vx, 0.0, vz])
}

/// 辅助：构建椭圆轨道初值（偏心率 e）。
fn elliptic_orbit_ic(a: f64, e: f64, gm: f64) -> ([f64; 3], [f64; 3]) {
    // 近地点出发
    let r_pe = a * (1.0 - e);
    let v_pe = (gm * (2.0 / r_pe - 1.0 / a)).sqrt();
    ([r_pe, 0.0, 0.0], [0.0, 0.0, v_pe])
}

/// 辅助：Rust 端点质量力函数。
fn make_point_mass_force(gm: f64) -> impl FnMut(&orbitx_math::StateVectors, f64) -> (Vec3, Vec3) {
    move |s: &orbitx_math::StateVectors, _t: f64| {
        let r2 = s.pos.x * s.pos.x + s.pos.y * s.pos.y + s.pos.z * s.pos.z;
        let r = r2.sqrt();
        let f = -gm / (r2 * r);
        (s.pos * f, Vec3::ZERO)
    }
}

/// 辅助：Rust 端 J2 力函数。
fn make_j2_force(gm: f64) -> impl FnMut(&orbitx_math::StateVectors, f64) -> (Vec3, Vec3) {
    move |s: &orbitx_math::StateVectors, _t: f64| {
        let r2 = s.pos.x * s.pos.x + s.pos.y * s.pos.y + s.pos.z * s.pos.z;
        let r = r2.sqrt();
        let f = -gm / (r2 * r);
        let mut acc = s.pos * f;
        // J2
        let re = 6.37101e6;
        let j2 = 1.0826e-3;
        let zr2 = (s.pos.z / r) * (s.pos.z / r);
        let rr5 = re * re / (r2 * r2 * r);
        let fj = -1.5 * j2 * gm * rr5;
        acc.x += fj * s.pos.x * (1.0 - 5.0 * zr2);
        acc.y += fj * s.pos.y * (1.0 - 5.0 * zr2);
        acc.z += fj * s.pos.z * (3.0 - 5.0 * zr2);
        (acc, Vec3::ZERO)
    }
}

/// 辅助：验证多步轨迹偏差。
fn assert_trajectory_close(
    rust_pos: &[f64; 3],
    cpp_pos: &[f64; 3],
    rust_vel: &[f64; 3],
    cpp_vel: &[f64; 3],
    label: &str,
    rel_tol: f64,
    abs_tol: f64,
) {
    let pos_err = (rust_pos[0] - cpp_pos[0])
        .abs()
        .max((rust_pos[1] - cpp_pos[1]).abs())
        .max((rust_pos[2] - cpp_pos[2]).abs());
    let pos_mag = cpp_pos
        .iter()
        .map(|v| v.abs())
        .fold(0.0_f64, f64::max)
        .max(1.0);
    assert!(
        pos_err <= rel_tol * pos_mag + abs_tol,
        "{label} pos 累积偏差过大: {pos_err} (allowed={})",
        rel_tol * pos_mag + abs_tol
    );
    let vel_err = (rust_vel[0] - cpp_vel[0])
        .abs()
        .max((rust_vel[1] - cpp_vel[1]).abs())
        .max((rust_vel[2] - cpp_vel[2]).abs());
    let vel_mag = cpp_vel
        .iter()
        .map(|v| v.abs())
        .fold(0.0_f64, f64::max)
        .max(1.0);
    assert!(
        vel_err <= rel_tol * vel_mag + abs_tol,
        "{label} vel 累积偏差过大: {vel_err} (allowed={})",
        rel_tol * vel_mag + abs_tol
    );
}

proptest! {
    /// RK4 单步：Rust rk4_step 的线性部分 vs C++ ox_rk4_step（圆轨道初值）。
    #[test]
    fn prop_rk4_single_step_circular(
        r0 in 1e6_f64..1e8,
        theta in 0.0_f64..6.28,
    ) {
        let _lock = INTEGRATOR_LOCK.lock().unwrap();
        let gm = 3.986e14; // 地球
        unsafe { G_GM = gm; }
        ffi::set_force_callback(point_mass_acc);

        // 圆轨道初值：位置在 xz 平面，速度切向。
        let (pos, vel) = circular_orbit_ic(r0, theta, gm);
        let h = 10.0; // 10 秒

        // Rust rk4_step（含 omega/q，但引力无力矩，角通道保持零）。
        use orbitx_math::{Matrix3, Quat, StateVectors};
        let mut force = make_point_mass_force(gm);
        let s0 = StateVectors {
            pos: Vec3::new(pos[0], pos[1], pos[2]),
            vel: Vec3::new(vel[0], vel[1], vel[2]),
            omega: Vec3::ZERO,
            r: Matrix3::IDENTITY,
            q: Quat::IDENTITY,
        };
        let s1 = orbitx_dynamics::rk4_step(s0, h, &mut force);

        // C++ ox_rk4_step（仅线性）。
        let (cpp_pos, cpp_vel) = ffi::rk4_step_linear(pos, vel, h);

        assert_close3(&[s1.pos.x, s1.pos.y, s1.pos.z], &cpp_pos, "rk4 pos");
        assert_close3(&[s1.vel.x, s1.vel.y, s1.vel.z], &cpp_vel, "rk4 vel");
    }

    /// RK4 多步：100 步积分后轨迹对照（排除步间累积偏差）。
    #[test]
    fn prop_rk4_multistep_trajectory(
        r0 in 5e6_f64..5e7,
        theta in 0.0_f64..6.28,
        nsteps in 10_usize..200,
    ) {
        let _lock = INTEGRATOR_LOCK.lock().unwrap();
        let gm = 3.986e14;
        unsafe { G_GM = gm; }
        ffi::set_force_callback(point_mass_acc);

        let (pos0, vel0) = circular_orbit_ic(r0, theta, gm);
        let h = 30.0; // 较大步长放大可检测性

        // Rust 多步。
        use orbitx_math::{Matrix3, Quat, StateVectors};
        let mut sr = StateVectors {
            pos: Vec3::new(pos0[0], pos0[1], pos0[2]),
            vel: Vec3::new(vel0[0], vel0[1], vel0[2]),
            omega: Vec3::ZERO,
            r: Matrix3::IDENTITY,
            q: Quat::IDENTITY,
        };
        let mut cpp_pos = pos0;
        let mut cpp_vel = vel0;
        for _ in 0..nsteps {
            let mut force = make_point_mass_force(gm);
            sr = orbitx_dynamics::rk4_step(sr, h, &mut force);
            let (p, v) = ffi::rk4_step_linear(cpp_pos, cpp_vel, h);
            cpp_pos = p;
            cpp_vel = v;
        }

        // 多步累积，容差略放宽（相对 1e-9）。
        let pos_err = (sr.pos.x - cpp_pos[0]).abs().max((sr.pos.y - cpp_pos[1]).abs()).max((sr.pos.z - cpp_pos[2]).abs());
        let pos_mag = cpp_pos.iter().map(|v| v.abs()).fold(0.0_f64, f64::max);
        assert!(pos_err <= 1e-9 * pos_mag + 1e-6, "rk4 多步 pos 累积偏差过大: {pos_err}");
        let vel_err = (sr.vel.x - cpp_vel[0]).abs().max((sr.vel.y - cpp_vel[1]).abs()).max((sr.vel.z - cpp_vel[2]).abs());
        let vel_mag = cpp_vel.iter().map(|v| v.abs()).fold(0.0_f64, f64::max);
        assert!(vel_err <= 1e-9 * vel_mag + 1e-9, "rk4 多步 vel 累积偏差过大: {vel_err}");
    }
}

// ===========================================================
// RK2 integrator property tests (BodyIntegrator.cpp RK2_LinAng)
// ===========================================================

proptest! {
    /// RK2 多步轨迹对照（圆轨道）。
    #[test]
    fn prop_rk2_multistep_circular(
        r0 in 5e6_f64..5e7,
        theta in 0.0_f64..6.28,
        nsteps in 10_usize..100,
    ) {
        let _lock = INTEGRATOR_LOCK.lock().unwrap();
        let gm = 3.986e14;
        unsafe { G_GM = gm; }
        ffi::set_force_callback(point_mass_acc);

        let (pos0, vel0) = circular_orbit_ic(r0, theta, gm);
        let h = 10.0; // RK2 是低阶方法，用较小步长

        use orbitx_math::{Matrix3, Quat, StateVectors};
        let mut sr = StateVectors {
            pos: Vec3::new(pos0[0], pos0[1], pos0[2]),
            vel: Vec3::new(vel0[0], vel0[1], vel0[2]),
            omega: Vec3::ZERO,
            r: Matrix3::IDENTITY,
            q: Quat::IDENTITY,
        };
        let mut cpp_pos = pos0;
        let mut cpp_vel = vel0;
        for _ in 0..nsteps {
            let mut force = make_point_mass_force(gm);
            sr = orbitx_dynamics::rk2_step(sr, h, &mut force);
            let (p, v) = ffi::rk2_step_linear(cpp_pos, cpp_vel, h);
            cpp_pos = p;
            cpp_vel = v;
        }

        // RK2 低阶，多步累积偏差较大，容差放宽
        assert_trajectory_close(
            &[sr.pos.x, sr.pos.y, sr.pos.z], &cpp_pos,
            &[sr.vel.x, sr.vel.y, sr.vel.z], &cpp_vel,
            "rk2_circular", 1e-9, 1e-4,
        );
    }

    /// RK2 多步轨迹对照（椭圆轨道）。
    #[test]
    fn prop_rk2_multistep_elliptic(
        a in 7e6_f64..4e7,
        e in 0.01_f64..0.6,
        nsteps in 10_usize..100,
    ) {
        let _lock = INTEGRATOR_LOCK.lock().unwrap();
        let gm = 3.986e14;
        unsafe { G_GM = gm; }
        ffi::set_force_callback(point_mass_acc);

        let (pos0, vel0) = elliptic_orbit_ic(a, e, gm);
        let h = 10.0;

        use orbitx_math::{Matrix3, Quat, StateVectors};
        let mut sr = StateVectors {
            pos: Vec3::new(pos0[0], pos0[1], pos0[2]),
            vel: Vec3::new(vel0[0], vel0[1], vel0[2]),
            omega: Vec3::ZERO,
            r: Matrix3::IDENTITY,
            q: Quat::IDENTITY,
        };
        let mut cpp_pos = pos0;
        let mut cpp_vel = vel0;
        for _ in 0..nsteps {
            let mut force = make_point_mass_force(gm);
            sr = orbitx_dynamics::rk2_step(sr, h, &mut force);
            let (p, v) = ffi::rk2_step_linear(cpp_pos, cpp_vel, h);
            cpp_pos = p;
            cpp_vel = v;
        }

        assert_trajectory_close(
            &[sr.pos.x, sr.pos.y, sr.pos.z], &cpp_pos,
            &[sr.vel.x, sr.vel.y, sr.vel.z], &cpp_vel,
            "rk2_elliptic", 1e-9, 1e-3,
        );
    }
}

// ===========================================================
// RK5/RK8 integrator property tests (BodyIntegrator.cpp RKdrv_LinAng)
// ===========================================================

proptest! {
    /// RK5 多步轨迹对照（圆轨道）。
    #[test]
    fn prop_rk5_multistep_circular(
        r0 in 5e6_f64..5e7,
        theta in 0.0_f64..6.28,
        nsteps in 10_usize..200,
    ) {
        let _lock = INTEGRATOR_LOCK.lock().unwrap();
        let gm = 3.986e14;
        unsafe { G_GM = gm; }
        ffi::set_force_callback(point_mass_acc);

        let (pos0, vel0) = circular_orbit_ic(r0, theta, gm);
        let h = 30.0;

        use orbitx_math::{Matrix3, Quat, StateVectors};
        use orbitx_dynamics::integrator::{RK5, rk_drv};
        let mut sr = StateVectors {
            pos: Vec3::new(pos0[0], pos0[1], pos0[2]),
            vel: Vec3::new(vel0[0], vel0[1], vel0[2]),
            omega: Vec3::ZERO,
            r: Matrix3::IDENTITY,
            q: Quat::IDENTITY,
        };
        let mut cpp_pos = pos0;
        let mut cpp_vel = vel0;
        for _ in 0..nsteps {
            let mut force = make_point_mass_force(gm);
            sr = rk_drv(sr, h, &RK5, &mut force);
            let (p, v) = ffi::rk_drv_step_linear(
                cpp_pos, cpp_vel, h,
                RK5.n, RK5.alpha, RK5.beta, RK5.gamma,
            );
            cpp_pos = p;
            cpp_vel = v;
        }

        assert_trajectory_close(
            &[sr.pos.x, sr.pos.y, sr.pos.z], &cpp_pos,
            &[sr.vel.x, sr.vel.y, sr.vel.z], &cpp_vel,
            "rk5_circular", 1e-9, 1e-6,
        );
    }

    /// RK8 多步轨迹对照（圆轨道）。
    #[test]
    fn prop_rk8_multistep_circular(
        r0 in 5e6_f64..5e7,
        theta in 0.0_f64..6.28,
        nsteps in 10_usize..200,
    ) {
        let _lock = INTEGRATOR_LOCK.lock().unwrap();
        let gm = 3.986e14;
        unsafe { G_GM = gm; }
        ffi::set_force_callback(point_mass_acc);

        let (pos0, vel0) = circular_orbit_ic(r0, theta, gm);
        let h = 60.0; // RK8 高阶，可用更大步长

        use orbitx_math::{Matrix3, Quat, StateVectors};
        use orbitx_dynamics::integrator::{RK8, rk_drv};
        let mut sr = StateVectors {
            pos: Vec3::new(pos0[0], pos0[1], pos0[2]),
            vel: Vec3::new(vel0[0], vel0[1], vel0[2]),
            omega: Vec3::ZERO,
            r: Matrix3::IDENTITY,
            q: Quat::IDENTITY,
        };
        let mut cpp_pos = pos0;
        let mut cpp_vel = vel0;
        for _ in 0..nsteps {
            let mut force = make_point_mass_force(gm);
            sr = rk_drv(sr, h, &RK8, &mut force);
            let (p, v) = ffi::rk_drv_step_linear(
                cpp_pos, cpp_vel, h,
                RK8.n, RK8.alpha, RK8.beta, RK8.gamma,
            );
            cpp_pos = p;
            cpp_vel = v;
        }

        assert_trajectory_close(
            &[sr.pos.x, sr.pos.y, sr.pos.z], &cpp_pos,
            &[sr.vel.x, sr.vel.y, sr.vel.z], &cpp_vel,
            "rk8_circular", 1e-9, 1e-6,
        );
    }

    /// RK8 多步轨迹对照（椭圆轨道 + J2 扰动）。
    #[test]
    fn prop_rk8_multistep_elliptic_j2(
        a in 7e6_f64..4e7,
        e in 0.01_f64..0.5,
        nsteps in 10_usize..200,
    ) {
        let _lock = INTEGRATOR_LOCK.lock().unwrap();
        let gm = 3.986e14;
        unsafe { G_GM = gm; }
        ffi::set_force_callback(j2_acc);

        let (pos0, vel0) = elliptic_orbit_ic(a, e, gm);
        let h = 30.0;

        use orbitx_math::{Matrix3, Quat, StateVectors};
        use orbitx_dynamics::integrator::{RK8, rk_drv};
        let mut sr = StateVectors {
            pos: Vec3::new(pos0[0], pos0[1], pos0[2]),
            vel: Vec3::new(vel0[0], vel0[1], vel0[2]),
            omega: Vec3::ZERO,
            r: Matrix3::IDENTITY,
            q: Quat::IDENTITY,
        };
        let mut cpp_pos = pos0;
        let mut cpp_vel = vel0;
        for _ in 0..nsteps {
            let mut force = make_j2_force(gm);
            sr = rk_drv(sr, h, &RK8, &mut force);
            let (p, v) = ffi::rk_drv_step_linear(
                cpp_pos, cpp_vel, h,
                RK8.n, RK8.alpha, RK8.beta, RK8.gamma,
            );
            cpp_pos = p;
            cpp_vel = v;
        }

        assert_trajectory_close(
            &[sr.pos.x, sr.pos.y, sr.pos.z], &cpp_pos,
            &[sr.vel.x, sr.vel.y, sr.vel.z], &cpp_vel,
            "rk8_elliptic_j2", 1e-9, 1e-5,
        );
    }
}

// ===========================================================
// Symplectic integrator property tests (BodyIntegrator.cpp SY*_LinAng)
// ===========================================================

proptest! {
    /// SY2 多步轨迹对照（圆轨道）。
    #[test]
    fn prop_sy2_multistep_circular(
        r0 in 5e6_f64..5e7,
        theta in 0.0_f64..6.28,
        nsteps in 10_usize..200,
    ) {
        let _lock = INTEGRATOR_LOCK.lock().unwrap();
        let gm = 3.986e14;
        unsafe { G_GM = gm; }
        ffi::set_force_callback(point_mass_acc);

        let (pos0, vel0) = circular_orbit_ic(r0, theta, gm);
        let h = 10.0;

        use orbitx_math::{Matrix3, Quat, StateVectors};
        use orbitx_dynamics::integrator::{SY2, sy_step};
        let mut sr = StateVectors {
            pos: Vec3::new(pos0[0], pos0[1], pos0[2]),
            vel: Vec3::new(vel0[0], vel0[1], vel0[2]),
            omega: Vec3::ZERO,
            r: Matrix3::IDENTITY,
            q: Quat::IDENTITY,
        };
        let mut cpp_pos = pos0;
        let mut cpp_vel = vel0;
        for _ in 0..nsteps {
            let mut force = make_point_mass_force(gm);
            sr = sy_step(sr, h, &SY2, &mut force);
            let (p, v) = ffi::sy_step_linear(cpp_pos, cpp_vel, h, SY2.c, SY2.d);
            cpp_pos = p;
            cpp_vel = v;
        }

        assert_trajectory_close(
            &[sr.pos.x, sr.pos.y, sr.pos.z], &cpp_pos,
            &[sr.vel.x, sr.vel.y, sr.vel.z], &cpp_vel,
            "sy2_circular", 1e-9, 1e-5,
        );
    }

    /// SY4 多步轨迹对照（圆轨道）。
    #[test]
    fn prop_sy4_multistep_circular(
        r0 in 5e6_f64..5e7,
        theta in 0.0_f64..6.28,
        nsteps in 10_usize..200,
    ) {
        let _lock = INTEGRATOR_LOCK.lock().unwrap();
        let gm = 3.986e14;
        unsafe { G_GM = gm; }
        ffi::set_force_callback(point_mass_acc);

        let (pos0, vel0) = circular_orbit_ic(r0, theta, gm);
        let h = 30.0;

        use orbitx_math::{Matrix3, Quat, StateVectors};
        use orbitx_dynamics::integrator::{SY4, sy_step};
        let mut sr = StateVectors {
            pos: Vec3::new(pos0[0], pos0[1], pos0[2]),
            vel: Vec3::new(vel0[0], vel0[1], vel0[2]),
            omega: Vec3::ZERO,
            r: Matrix3::IDENTITY,
            q: Quat::IDENTITY,
        };
        let mut cpp_pos = pos0;
        let mut cpp_vel = vel0;
        for _ in 0..nsteps {
            let mut force = make_point_mass_force(gm);
            sr = sy_step(sr, h, &SY4, &mut force);
            let (p, v) = ffi::sy_step_linear(cpp_pos, cpp_vel, h, SY4.c, SY4.d);
            cpp_pos = p;
            cpp_vel = v;
        }

        assert_trajectory_close(
            &[sr.pos.x, sr.pos.y, sr.pos.z], &cpp_pos,
            &[sr.vel.x, sr.vel.y, sr.vel.z], &cpp_vel,
            "sy4_circular", 1e-9, 1e-5,
        );
    }

    /// SY6 多步轨迹对照（圆轨道）。
    #[test]
    fn prop_sy6_multistep_circular(
        r0 in 5e6_f64..5e7,
        theta in 0.0_f64..6.28,
        nsteps in 10_usize..200,
    ) {
        let _lock = INTEGRATOR_LOCK.lock().unwrap();
        let gm = 3.986e14;
        unsafe { G_GM = gm; }
        ffi::set_force_callback(point_mass_acc);

        let (pos0, vel0) = circular_orbit_ic(r0, theta, gm);
        let h = 30.0;

        use orbitx_math::{Matrix3, Quat, StateVectors};
        use orbitx_dynamics::integrator::{SY6, sy_step};
        let mut sr = StateVectors {
            pos: Vec3::new(pos0[0], pos0[1], pos0[2]),
            vel: Vec3::new(vel0[0], vel0[1], vel0[2]),
            omega: Vec3::ZERO,
            r: Matrix3::IDENTITY,
            q: Quat::IDENTITY,
        };
        let mut cpp_pos = pos0;
        let mut cpp_vel = vel0;
        for _ in 0..nsteps {
            let mut force = make_point_mass_force(gm);
            sr = sy_step(sr, h, &SY6, &mut force);
            let (p, v) = ffi::sy_step_linear(cpp_pos, cpp_vel, h, SY6.c, SY6.d);
            cpp_pos = p;
            cpp_vel = v;
        }

        assert_trajectory_close(
            &[sr.pos.x, sr.pos.y, sr.pos.z], &cpp_pos,
            &[sr.vel.x, sr.vel.y, sr.vel.z], &cpp_vel,
            "sy6_circular", 1e-9, 1e-5,
        );
    }

    /// SY8 多步轨迹对照（圆轨道）。
    #[test]
    fn prop_sy8_multistep_circular(
        r0 in 5e6_f64..5e7,
        theta in 0.0_f64..6.28,
        nsteps in 10_usize..200,
    ) {
        let _lock = INTEGRATOR_LOCK.lock().unwrap();
        let gm = 3.986e14;
        unsafe { G_GM = gm; }
        ffi::set_force_callback(point_mass_acc);

        let (pos0, vel0) = circular_orbit_ic(r0, theta, gm);
        let h = 60.0; // SY8 高阶，可用更大步长

        use orbitx_math::{Matrix3, Quat, StateVectors};
        use orbitx_dynamics::integrator::{SY8, sy_step};
        let mut sr = StateVectors {
            pos: Vec3::new(pos0[0], pos0[1], pos0[2]),
            vel: Vec3::new(vel0[0], vel0[1], vel0[2]),
            omega: Vec3::ZERO,
            r: Matrix3::IDENTITY,
            q: Quat::IDENTITY,
        };
        let mut cpp_pos = pos0;
        let mut cpp_vel = vel0;
        for _ in 0..nsteps {
            let mut force = make_point_mass_force(gm);
            sr = sy_step(sr, h, &SY8, &mut force);
            let (p, v) = ffi::sy_step_linear(cpp_pos, cpp_vel, h, SY8.c, SY8.d);
            cpp_pos = p;
            cpp_vel = v;
        }

        assert_trajectory_close(
            &[sr.pos.x, sr.pos.y, sr.pos.z], &cpp_pos,
            &[sr.vel.x, sr.vel.y, sr.vel.z], &cpp_vel,
            "sy8_circular", 1e-9, 1e-5,
        );
    }

    /// SY8 多步轨迹对照（椭圆轨道 + J2 扰动）。
    #[test]
    fn prop_sy8_multistep_elliptic_j2(
        a in 7e6_f64..4e7,
        e in 0.01_f64..0.5,
        nsteps in 10_usize..200,
    ) {
        let _lock = INTEGRATOR_LOCK.lock().unwrap();
        let gm = 3.986e14;
        unsafe { G_GM = gm; }
        ffi::set_force_callback(j2_acc);

        let (pos0, vel0) = elliptic_orbit_ic(a, e, gm);
        let h = 30.0;

        use orbitx_math::{Matrix3, Quat, StateVectors};
        use orbitx_dynamics::integrator::{SY8, sy_step};
        let mut sr = StateVectors {
            pos: Vec3::new(pos0[0], pos0[1], pos0[2]),
            vel: Vec3::new(vel0[0], vel0[1], vel0[2]),
            omega: Vec3::ZERO,
            r: Matrix3::IDENTITY,
            q: Quat::IDENTITY,
        };
        let mut cpp_pos = pos0;
        let mut cpp_vel = vel0;
        for _ in 0..nsteps {
            let mut force = make_j2_force(gm);
            sr = sy_step(sr, h, &SY8, &mut force);
            let (p, v) = ffi::sy_step_linear(cpp_pos, cpp_vel, h, SY8.c, SY8.d);
            cpp_pos = p;
            cpp_vel = v;
        }

        assert_trajectory_close(
            &[sr.pos.x, sr.pos.y, sr.pos.z], &cpp_pos,
            &[sr.vel.x, sr.vel.y, sr.vel.z], &cpp_vel,
            "sy8_elliptic_j2", 1e-9, 1e-4,
        );
    }
}

// ===========================================================
// Product rocket aero (AERO.md / rocket.rs vs C++ oracle)
// ===========================================================

fn assert_aero_close(rust: &orbitx_dynamics::AeroForces, cpp: &ffi::OxAeroForces, ctx: &str) {
    assert_close3(
        &[rust.force.x, rust.force.y, rust.force.z],
        &[cpp.force_x, cpp.force_y, cpp.force_z],
        &format!("{ctx}.force"),
    );
    assert_close3(
        &[rust.torque.x, rust.torque.y, rust.torque.z],
        &[cpp.torque_x, cpp.torque_y, cpp.torque_z],
        &format!("{ctx}.torque"),
    );
    assert_close(rust.mach, cpp.mach, &format!("{ctx}.mach"));
    assert_close(
        rust.dynamic_pressure,
        cpp.dynamic_pressure,
        &format!("{ctx}.q"),
    );
    assert_close(rust.drag_force, cpp.drag_force, &format!("{ctx}.drag"));
    assert_close(rust.lift_force, cpp.lift_force, &format!("{ctx}.lift"));
    assert_close(rust.cd_eff, cpp.cd_eff, &format!("{ctx}.cd_eff"));
}

proptest! {
    #[test]
    fn prop_wave_drag(
        mach in 0.0_f64..5.0,
        m1 in 0.5_f64..0.9,
        m2 in 0.9_f64..1.2,
        m3 in 1.2_f64..2.0,
        cmax in 0.0_f64..0.2,
    ) {
        prop_assume!(m1 < m2 && m2 < m3);
        let rust = wave_drag(mach, m1, m2, m3, cmax);
        let cpp = ffi::wave_drag(mach, m1, m2, m3, cmax);
        assert_close(rust, cpp, "wave_drag");
    }
}

proptest! {
    #[test]
    fn prop_induced_drag(
        cl in -3.0_f64..3.0,
        aspect in 0.0_f64..10.0,
        oswald in 0.0_f64..1.0,
    ) {
        let rust = induced_drag(cl, aspect, oswald);
        let cpp = ffi::induced_drag(cl, aspect, oswald);
        assert_close(rust, cpp, "induced_drag");
    }
}

proptest! {
    #[test]
    fn prop_alpha_stall_mach(
        a0 in 0.05_f64..0.6,
        mach in 0.0_f64..3.0,
    ) {
        let rust = alpha_stall_mach(a0, mach);
        let cpp = ffi::alpha_stall_mach(a0, mach);
        assert_close(rust, cpp, "alpha_stall_mach");
    }
}

proptest! {
    #[test]
    fn prop_grid_eta(mach in 0.0_f64..3.0) {
        let rust = grid_eta(mach);
        let cpp = ffi::grid_eta(mach);
        assert_close(rust, cpp, "grid_eta");
    }
}

proptest! {
    #[test]
    fn prop_slew_deploy(
        deploy in -0.2_f64..1.2,
        target in -0.2_f64..1.2,
        rate in 0.0_f64..5.0,
        dt in 0.0_f64..1.0,
    ) {
        let rust = slew_deploy(deploy, target, rate, dt);
        let cpp = ffi::slew_deploy(deploy, target, rate, dt);
        assert_close(rust, cpp, "slew_deploy");
    }
}

proptest! {
    #[test]
    fn prop_side_area(
        ax in 0.1_f64..100.0,
        ay in 0.1_f64..20.0,
        az in 0.1_f64..100.0,
        vx in -200.0_f64..200.0,
        vy in -200.0_f64..200.0,
        vz in -200.0_f64..200.0,
    ) {
        let areas = TriaxialAreas { x: ax, y: ay, z: az };
        let airvel = Vec3::new(vx, vy, vz);
        let rust = side_area(areas, airvel);
        let cpp = ffi::side_area([ax, ay, az], [vx, vy, vz]);
        assert_close(rust, cpp, "side_area");
    }
}

proptest! {
    #[test]
    fn prop_fin_local_alpha(
        avx in -200.0_f64..200.0,
        avy in -200.0_f64..200.0,
        avz in -200.0_f64..200.0,
        nx in -1.0_f64..1.0,
        ny in -1.0_f64..1.0,
        nz in -1.0_f64..1.0,
        cx in -1.0_f64..1.0,
        cy in -1.0_f64..1.0,
        cz in -1.0_f64..1.0,
    ) {
        prop_assume!(nx * nx + ny * ny + nz * nz > 1e-6);
        prop_assume!(cx * cx + cy * cy + cz * cz > 1e-6);
        let airvel = Vec3::new(avx, avy, avz);
        let normal = Vec3::new(nx, ny, nz);
        let chord = Vec3::new(cx, cy, cz);
        let rust = fin_local_alpha(airvel, normal, chord);
        let cpp = ffi::fin_local_alpha([avx, avy, avz], [nx, ny, nz], [cx, cy, cz]);
        assert_close(rust, cpp, "fin_local_alpha");
    }
}

proptest! {
    #[test]
    fn prop_moment_about_cg(
        fx in -1e5_f64..1e5,
        fy in -1e5_f64..1e5,
        fz in -1e5_f64..1e5,
        px in -50.0_f64..50.0,
        py in -50.0_f64..50.0,
        pz in -50.0_f64..50.0,
        cgx in -50.0_f64..50.0,
        cgy in -50.0_f64..50.0,
        cgz in -50.0_f64..50.0,
    ) {
        let rust = moment_about_cg(
            Vec3::new(fx, fy, fz),
            Vec3::new(px, py, pz),
            Vec3::new(cgx, cgy, cgz),
        );
        let cpp = ffi::moment_about_cg([fx, fy, fz], [px, py, pz], [cgx, cgy, cgz]);
        assert_close3(&[rust.x, rust.y, rust.z], &cpp, "moment_about_cg");
    }
}

proptest! {
    #[test]
    fn prop_compute_body_aero(
        avx in -200.0_f64..200.0,
        avy in -200.0_f64..200.0,
        avz in -200.0_f64..200.0,
        rho in 0.0_f64..1.5,
        sound in 200.0_f64..400.0,
        ax in 1.0_f64..80.0,
        ay in 0.5_f64..20.0,
        az in 1.0_f64..80.0,
        copy in -20.0_f64..20.0,
        cgy in -20.0_f64..20.0,
        cd0 in 0.1_f64..0.8,
        cn_alpha in 0.5_f64..4.0,
        pitch_damp in 0.0_f64..2.0,
        yaw_damp in 0.0_f64..2.0,
        roll_damp in 0.0_f64..1.0,
        wx in -2.0_f64..2.0,
        wy in -2.0_f64..2.0,
        wz in -2.0_f64..2.0,
        use_table in proptest::bool::ANY,
    ) {
        prop_assume!(avx * avx + avy * avy + avz * avz > 1.0);
        let areas = TriaxialAreas { x: ax, y: ay, z: az };
        let airvel = Vec3::new(avx, avy, avz);
        let omega = Vec3::new(wx, wy, wz);
        let cop = Vec3::new(0.0, copy, 0.0);
        let cg = Vec3::new(0.0, cgy, 0.0);
        let cd_mach = if use_table {
            vec![(0.0, cd0), (1.0, cd0 * 1.4), (5.0, cd0 * 1.1)]
        } else {
            Vec::new()
        };
        let body = RocketBodyAero {
            cd_mach: cd_mach.clone(),
            cd0,
            cn_alpha,
            pitch_damp,
            yaw_damp,
            roll_damp,
        };
        let rust = compute_body_aero(airvel, omega, rho, sound, areas, cop, cg, &body);
        let cpp = ffi::compute_body_aero(
            [avx, avy, avz],
            [wx, wy, wz],
            rho,
            sound,
            [ax, ay, az],
            [0.0, copy, 0.0],
            [0.0, cgy, 0.0],
            cd0,
            cn_alpha,
            pitch_damp,
            yaw_damp,
            roll_damp,
            &cd_mach,
        );
        assert_aero_close(&rust, &cpp, "compute_body_aero");
    }
}

proptest! {
    #[test]
    fn prop_compute_rocket_aero(
        avx in -200.0_f64..200.0,
        avy in -200.0_f64..200.0,
        avz in -200.0_f64..200.0,
        rho in 0.1_f64..1.5,
        sound in 280.0_f64..360.0,
        ax in 10.0_f64..60.0,
        ay in 1.0_f64..10.0,
        az in 10.0_f64..60.0,
        cd0 in 0.2_f64..0.5,
        cn_alpha in 1.0_f64..3.0,
        area in 0.5_f64..5.0,
        cl_a in 2.0_f64..5.0,
        deploy in 0.0_f64..1.0,
        is_grid in proptest::bool::ANY,
        leeward in proptest::bool::ANY,
        n_surf in 0usize..=2,
        wx in -2.0_f64..2.0,
        wy in -2.0_f64..2.0,
        wz in -2.0_f64..2.0,
        pitch_damp in 0.0_f64..2.0,
        yaw_damp in 0.0_f64..2.0,
        roll_damp in 0.0_f64..1.0,
    ) {
        prop_assume!(avx * avx + avy * avy + avz * avz > 4.0);
        let areas = TriaxialAreas { x: ax, y: ay, z: az };
        let airvel = Vec3::new(avx, avy, avz);
        let omega = Vec3::new(wx, wy, wz);
        let body = RocketBodyAero {
            cd_mach: vec![(0.0, cd0), (1.0, cd0 * 1.3), (5.0, cd0)],
            cd0,
            cn_alpha,
            pitch_damp,
            yaw_damp,
            roll_damp,
        };
        let kind = if is_grid { FinKind::Grid } else { FinKind::Fixed };
        let mut surfaces = Vec::new();
        let mut cpp_surfs = Vec::new();
        for i in 0..n_surf {
            let sign = if i == 0 { 1.0 } else { -1.0 };
            let surf = LiftingSurface {
                ref_pos: Vec3::new(sign * 1.2, -5.0, 0.0),
                normal: Vec3::new(0.0, 0.0, sign),
                chord_dir: Vec3::new(0.0, 1.0, 0.0),
                area,
                aspect_ratio: 2.0,
                cl_alpha: cl_a,
                cd0: 0.02,
                alpha_stall0: 18.0_f64.to_radians(),
                kind,
                deploy,
                leeward_sheltered: leeward,
            };
            cpp_surfs.push(ffi::OxLiftingSurface {
                ref_pos_x: surf.ref_pos.x,
                ref_pos_y: surf.ref_pos.y,
                ref_pos_z: surf.ref_pos.z,
                normal_x: surf.normal.x,
                normal_y: surf.normal.y,
                normal_z: surf.normal.z,
                chord_x: surf.chord_dir.x,
                chord_y: surf.chord_dir.y,
                chord_z: surf.chord_dir.z,
                area: surf.area,
                aspect_ratio: surf.aspect_ratio,
                cl_alpha: surf.cl_alpha,
                cd0: surf.cd0,
                alpha_stall0: surf.alpha_stall0,
                kind: if is_grid { 1 } else { 0 },
                deploy: surf.deploy,
                leeward_sheltered: if leeward { 1 } else { 0 },
            });
            surfaces.push(surf);
        }
        let rust = compute_rocket_aero(&RocketAeroInput {
            airvel_body: airvel,
            omega_body: omega,
            rho,
            sound_speed: sound,
            areas,
            body_cop: Vec3::new(0.0, 5.0, 0.0),
            cg: Vec3::new(0.0, 2.0, 0.0),
            body: &body,
            surfaces: &surfaces,
        });
        let cpp = ffi::compute_rocket_aero(
            [avx, avy, avz],
            [wx, wy, wz],
            rho,
            sound,
            [ax, ay, az],
            [0.0, 5.0, 0.0],
            [0.0, 2.0, 0.0],
            cd0,
            cn_alpha,
            pitch_damp,
            yaw_damp,
            roll_damp,
            &body.cd_mach,
            &cpp_surfs,
        );
        assert_aero_close(&rust, &cpp, "compute_rocket_aero");
    }
}
