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
use orbitx_dynamics::{euler_full, euler_inv_full, euler_inv_simple, gacc_nbody, jcoeff_perturbation, single_gacc, GravBody};
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
    x: f64, y: f64, z: f64,
    _vx: f64, _vy: f64, _vz: f64,
    _tfrac: f64,
    ax: *mut f64, ay: *mut f64, az: *mut f64,
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
    x: f64, y: f64, z: f64,
    _vx: f64, _vy: f64, _vz: f64,
    _tfrac: f64,
    ax: *mut f64, ay: *mut f64, az: *mut f64,
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
    let pos_err = (rust_pos[0] - cpp_pos[0]).abs()
        .max((rust_pos[1] - cpp_pos[1]).abs())
        .max((rust_pos[2] - cpp_pos[2]).abs());
    let pos_mag = cpp_pos.iter().map(|v| v.abs()).fold(0.0_f64, f64::max).max(1.0);
    assert!(
        pos_err <= rel_tol * pos_mag + abs_tol,
        "{label} pos 累积偏差过大: {pos_err} (allowed={})", rel_tol * pos_mag + abs_tol
    );
    let vel_err = (rust_vel[0] - cpp_vel[0]).abs()
        .max((rust_vel[1] - cpp_vel[1]).abs())
        .max((rust_vel[2] - cpp_vel[2]).abs());
    let vel_mag = cpp_vel.iter().map(|v| v.abs()).fold(0.0_f64, f64::max).max(1.0);
    assert!(
        vel_err <= rel_tol * vel_mag + abs_tol,
        "{label} vel 累积偏差过大: {vel_err} (allowed={})", rel_tol * vel_mag + abs_tol
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
