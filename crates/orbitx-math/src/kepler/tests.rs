//! Kepler 单元测试：要素往返、Kepler 方程、纬度幅角（C3b 圈判定口径）。

use super::*;
use crate::consts::{pos_angle, PI2};
use crate::vec3::{cross, dot, Vec3};

#[test]
fn circular_orbit_roundtrip() {
    // Circular orbit: r=7000km, mu=3.986e14
    let mu: f64 = 3.986e14;
    let r0: f64 = 7.0e6;
    let v0 = (mu / r0).sqrt();
    let r = Vec3::new(r0, 0.0, 0.0);
    let v = Vec3::new(0.0, 0.0, v0);

    let el = Elements::calculate(r, v, mu, 0.0);

    // Circular orbit: e ≈ 0, a = r
    assert!(el.e < 1e-6, "e = {}", el.e);
    assert!((el.a - r0).abs() / r0 < 1e-6, "a = {}", el.a);

    // Roundtrip: pos_vel at t=0 should return original r,v
    let (r1, v1) = el.pos_vel(0.0);
    assert!(
        (r1 - r).length() / r0 < 1e-6,
        "pos roundtrip err = {}",
        (r1 - r).length()
    );
    assert!(
        (v1 - v).length() / v0 < 1e-6,
        "vel roundtrip err = {}",
        (v1 - v).length()
    );
}

#[test]
fn elliptical_orbit_elements() {
    // Elliptical orbit: a=8000km, e=0.1, in the xz plane
    let mu: f64 = 3.986e14;
    let a: f64 = 8.0e6;
    let e: f64 = 0.1;
    let r_pe = a * (1.0 - e); // periapsis
    let v_pe = (mu * (2.0 / r_pe - 1.0 / a)).sqrt();
    let r = Vec3::new(r_pe, 0.0, 0.0);
    let v = Vec3::new(0.0, 0.0, v_pe);

    let el = Elements::calculate(r, v, mu, 0.0);

    assert!((el.a - a).abs() / a < 1e-10, "a = {}", el.a);
    assert!((el.e - e).abs() / e < 1e-10, "e = {}", el.e);
    assert!(
        (el.pe_dist() - r_pe).abs() / r_pe < 1e-6,
        "pe_dist = {}",
        el.pe_dist()
    );
}

#[test]
fn ecc_anomaly_kepler_eq() {
    // For a given E, M = E - e*sin(E). Verify ecc_anomaly(M) = E.
    let mu: f64 = 3.986e14;
    let r = Vec3::new(7.0e6, 0.0, 0.0);
    let v = Vec3::new(0.0, 0.0, (mu / 7.0e6).sqrt());
    let el = Elements::calculate(r, v, mu, 0.0);

    let e_test: f64 = 0.0; // circular
    let ea_test: f64 = 1.0;
    let ma = ea_test - e_test * ea_test.sin();
    let ea_solved = el.ecc_anomaly(ma);
    assert!((ea_solved - ea_test).abs() < 1e-10, "ea = {}", ea_solved);
}

#[test]
fn arg_latitude_equals_omega_plus_true_anomaly() {
    // 椭圆轨道：u = ω + ν，归一化 [0, 2π)。
    let mu: f64 = 3.986e14;
    // r=7e6 m 处速度 7800 m/s（超圆速 7546）→ e ≈ 0.068 椭圆。
    let r = Vec3::new(7.0e6, 0.0, 0.0);
    let v = Vec3::new(0.0, 0.0, 7_800.0);
    let el = Elements::calculate(r, v, mu, 0.0);
    assert!(el.e > 1e-3 && el.e < 1.0, "椭圆前提 e = {}", el.e);
    let u = el.arg_latitude();
    assert!((0.0..PI2).contains(&u), "u ∈ [0,2π)：{u}");
    let u_expect = pos_angle(el.arg_per() + el.true_anm());
    assert!((u - u_expect).abs() < 1e-12, "u = ω+ν：{u} vs {u_expect}");
}

#[test]
fn arg_latitude_circular_equals_true_anomaly() {
    // 圆轨（e ≤ E_CIRCLE_LIMIT）：ω 恒 0（引擎口径，Element.cpp 同源）——
    // u = ν 自升交点起算，与椭圆口径连续（任务二圈判定的数值前提）。
    let mu: f64 = 3.986e14;
    let r0: f64 = 7.0e6;
    let r = Vec3::new(r0, 0.0, 0.0);
    let v = Vec3::new(0.0, 0.0, (mu / r0).sqrt());
    let el = Elements::calculate(r, v, mu, 0.0);
    assert!(el.e < 1e-6, "圆轨前提 e = {}", el.e);
    assert_eq!(el.arg_per(), 0.0, "圆轨 ω=0 口径");
    assert!((el.arg_latitude() - el.true_anm()).abs() < 1e-12);
    // 初始态在 +x 轴、无倾角（XZ 面内）→ 升交点方向 = +x（n_vec 退化分支
    // 置 (1,0,0)），ν=0 → u=0。
    assert!(
        el.arg_latitude() < 1e-9,
        "自升交点起算 u=0：{}",
        el.arg_latitude()
    );
}

#[test]
fn arg_latitude_zero_at_periapsis_half_pi_at_apoapsis() {
    // 物理锚定：初始态 = 近地点（e_vec ∝ r、rv=0）→ u=0；半周期后 = 远地点
    // → u=π。验证 u 沿运动方向度量（近地点为 0、顺行增加）。
    let mu: f64 = 3.986e14;
    let r_pe = 7.0e6;
    let v_pe = (mu * (2.0 / r_pe - 1.0 / (r_pe / 0.9))).sqrt(); // a = r_pe/0.9
    let el0 = Elements::calculate(
        Vec3::new(r_pe, 0.0, 0.0),
        Vec3::new(0.0, 0.0, v_pe),
        mu,
        0.0,
    );
    assert!(el0.e > 1e-3 && el0.e < 1.0, "椭圆前提 e = {}", el0.e);
    assert!(
        el0.arg_latitude() < 1e-9 || (PI2 - el0.arg_latitude()) < 1e-9,
        "近地点 u≈0：{}",
        el0.arg_latitude()
    );
    let (r_ap, v_ap) = el0.pos_vel(el0.orbit_t() * 0.5);
    let el_ap = Elements::calculate(r_ap, v_ap, mu, 0.0);
    let u_ap = el_ap.arg_latitude();
    assert!(
        (u_ap - std::f64::consts::PI).abs() < 1e-6,
        "远地点 u≈π：{u_ap}"
    );
}

#[test]
fn arg_latitude_progresses_full_revolution() {
    // 任务二圈判定消费口径（切片 arg_latitude_deg 逐帧最小角差累积）：
    // 一周期内 u 逐帧前进、总累积恰 2π（回绕按最小角差处理）。
    let mu: f64 = 3.986e14;
    let el0 = Elements::calculate(
        Vec3::new(7.0e6, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 7_800.0),
        mu,
        0.0,
    );
    let period = el0.orbit_t();
    let n = 720;
    let mut total = 0.0;
    let mut prev = el0.arg_latitude();
    for k in 1..=n {
        let (r, v) = el0.pos_vel(period * k as f64 / n as f64);
        let el = Elements::calculate(r, v, mu, 0.0);
        let u = el.arg_latitude();
        let mut d = u - prev;
        if d < 0.0 {
            d += PI2; // 回绕：最小角差
        }
        total += d;
        prev = u;
    }
    assert!(
        (total - PI2).abs() < 1e-3,
        "一周期 u 累积 = 2π，实际 {total}"
    );
}
