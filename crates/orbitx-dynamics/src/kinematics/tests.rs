use super::*;
use orbitx_math::{Matrix3, Quat, StateVectors, Vec3, PI05};

fn state_at(pos: Vec3, rot: Matrix3) -> StateVectors {
    StateVectors {
        pos,
        vel: Vec3::ZERO,
        omega: Vec3::ZERO,
        r: rot,
        q: Quat::from_matrix(rot),
    }
}

#[test]
fn pitch_yaw_zero_when_nose_up_at_equator() {
    // 体 +Y 指向径向（竖直）→ pitch/yaw/tip 全 0。
    let pos = Vec3::new(0.0, 0.0, 6_371_000.0);
    let up = pos * (1.0 / pos.length());
    // 构造旋转使 body +Y = up：取 bx = X, bz = up×X, by = up
    let bx = Vec3::new(1.0, 0.0, 0.0);
    let bz = cross(up, bx).unit();
    let by = up;
    let rot = Matrix3::new(bx.x, by.x, bz.x, bx.y, by.y, bz.y, bx.z, by.z, bz.z);
    let st = state_at(pos, rot);
    let (p, y) = pitch_yaw_angles(&st);
    assert!(p.abs() < 1e-9, "pitch = {}", p);
    assert!(y.abs() < 1e-9, "yaw = {}", y);
    assert!(tip_angle(&st).abs() < 1e-9);
}

#[test]
fn tip_angle_90_when_horizontal() {
    // 体 +Y 与径向垂直 → tip = 90°。
    let pos = Vec3::new(0.0, 0.0, 6_371_000.0);
    let rot = Matrix3::IDENTITY; // body +Y = world +Y，径向 = world +Z → 90°
    let st = state_at(pos, rot);
    let tip = tip_angle(&st);
    assert!((tip - PI05).abs() < 1e-9, "tip = {}", tip);
}

#[test]
fn attitude_errors_signs() {
    // 径向 = +Z（world），体旋转使 body x 略偏 → attitude_errors 返回 (z_body, -x_body)
    let pos = Vec3::new(0.0, 0.0, 6_371_000.0);
    let st = state_at(pos, Matrix3::IDENTITY);
    let (ep, ey) = attitude_errors(&st);
    // 径向 world +Z → body frame（identity）= +Z → (z=1, -x=0) ⇒ ep≈1, ey≈0
    assert!((ep - 1.0).abs() < 1e-9, "ep = {}", ep);
    assert!(ey.abs() < 1e-9, "ey = {}", ey);
}

// —— nose_dir_from_az_el：罗盘 (az, el) → d̂ ——

/// 与实现同源的读回公式（Godot update_attitude 同构：el=asin(d·up)、
/// az=atan2(d·east, d·north)，0=北顺时针）。构造/读回同架 ⇒ 往返即恒等。
fn nose_readback(radial: Vec3, d: Vec3) -> (f64, f64) {
    let up = radial.unit();
    let east = Vec3::new(-up.z, 0.0, up.x).unit();
    let north = cross(east, up);
    let el = dot(d, up).clamp(-1.0, 1.0).asin();
    let az = dot(d, east).atan2(dot(d, north));
    (az, el)
}

#[test]
fn nose_dir_round_trip_global() {
    // 全球面采样（避开 ±90° 极点）：(az, el) → d̂ → 读回 → 同值。
    let radial = Vec3::new(0.0, 0.0, 6_371_000.0);
    let azs = [0.0, 0.5, 1.0, 2.0, 3.0, -0.7, -1.8, -2.9];
    let els = [-1.4, -0.5, 0.0, 0.3, 1.2];
    for az in azs {
        for el in els {
            let d = nose_dir_from_az_el(radial, az, el);
            assert!(
                (d.length() - 1.0).abs() < 1e-12,
                "d̂ 未单位化 az={az} el={el}: {}",
                d.length()
            );
            let (az2, el2) = nose_readback(radial, d);
            assert!(
                (az2 - az).abs() < 1e-9 && (el2 - el).abs() < 1e-9,
                "往返失配 az {az}->{az2} el {el}->{el2}"
            );
        }
    }
}

#[test]
fn nose_dir_poles_collapse_to_vertical() {
    // 精确 ±90°：方位不适定 → 塌缩 ±up；读回 (az=0, el=±90°)。
    let radial = Vec3::new(0.0, 0.0, 6_371_000.0);
    let up = radial.unit();
    let d_up = nose_dir_from_az_el(radial, 1.234, PI05);
    assert!(d_up.dist(up) < 1e-12, "el=+90 应天顶: {:?}", d_up);
    let d_down = nose_dir_from_az_el(radial, -2.345, -PI05);
    assert!(d_down.dist(-up) < 1e-12, "el=−90 应天底: {:?}", d_down);
    let (az, el) = nose_readback(radial, d_up);
    assert!(az.abs() < 1e-12 && (el - PI05).abs() < 1e-12, "读回 ({az},{el})");
}

#[test]
fn nose_dir_compass_semantics() {
    // 罗盘语义锚点：0=北、+90°=东、180°=南（el=0 水平面）。
    let radial = Vec3::new(0.0, 0.0, 6_371_000.0);
    let up = radial.unit();
    let east = Vec3::new(-up.z, 0.0, up.x).unit();
    let north = cross(east, up);
    assert!(nose_dir_from_az_el(radial, 0.0, 0.0).dist(north) < 1e-12);
    assert!(nose_dir_from_az_el(radial, PI05, 0.0).dist(east) < 1e-12);
    assert!(
        nose_dir_from_az_el(radial, std::f64::consts::PI, 0.0)
            .dist(-north)
            < 1e-12
    );
}

// —— nose_target_angles：d̂ → (pitch, yaw) TVC 目标 ——

#[test]
fn nose_target_angles_vertical_is_zero() {
    // 目标 = 天顶（radial）→ (0, 0)，与 VerticalHold 语义一致。
    let radial = Vec3::new(0.0, 0.0, 1.0);
    let (p, y) = nose_target_angles(radial, radial);
    assert!(p.abs() < 1e-12 && y.abs() < 1e-12, "({p},{y})");
}

#[test]
fn nose_target_angles_along_body_x() {
    // d̂ ∥ bx（旧投影式在此退化并把机头送到 −d̂）：固定罗盘架参考下直接
    // 良定。radial=+Z 构型 east=(−1,0,0)：d̂=+X（西）→ y = atan2(−1,0) = −π/2。
    let radial = Vec3::new(0.0, 0.0, 1.0);
    let (p, y) = nose_target_angles(radial, Vec3::new(1.0, 0.0, 0.0));
    assert!((p - 0.0).abs() < 1e-12, "p={p}");
    assert!((y - (-PI05)).abs() < 1e-12, "y={y}");
}

#[test]
fn nose_target_angles_known_configurations() {
    // pos=(0,0,R)（radial=+Z）固定构型（east=(−1,0,0)、north=(0,1,0)）的
    // 手算锚点：正北水平（el=0）→ (−π/2, 0)（+pitch 朝 −north=北 ✓）；
    // 正北上仰 45° → (−π/4, 0)；极点 radial 自动塌缩 (0,0)。
    let radial = Vec3::new(0.0, 0.0, 1.0);
    let north = nose_dir_from_az_el(radial, 0.0, 0.0);
    let (p, y) = nose_target_angles(radial, north);
    assert!((p - (-PI05)).abs() < 1e-12, "北水平 p={p}");
    assert!(y.abs() < 1e-12, "北水平 y={y}");

    let north_up = nose_dir_from_az_el(radial, 0.0, std::f64::consts::FRAC_PI_4);
    let (p, y) = nose_target_angles(radial, north_up);
    assert!(
        (p - (-std::f64::consts::FRAC_PI_4)).abs() < 1e-12,
        "北上仰45° p={p}"
    );
    assert!(y.abs() < 1e-12, "北上仰45° y={y}");

    // 极点 radial：east/north 零向量 → 点积全 0 → (0,0) 塌缩（无 NaN）。
    let pole = Vec3::new(0.0, 1.0, 0.0);
    let d_pole = nose_dir_from_az_el(pole, 2.345, std::f64::consts::FRAC_PI_4);
    let (p, y) = nose_target_angles(pole, d_pole);
    assert!(p.is_finite() && y.is_finite(), "极点 (p,y)=({p},{y}) 应有限");
    assert!(p.abs() < 1e-12 && y.abs() < 1e-12, "极点塌缩 ({p},{y})");
}

// —— nose_target_angles_body：体架相位对齐目标（NoseTo 控制路径） ——

/// 构造机头 = `nose`、滚转相位 ψ 的体架：bx0 为 ψ=0 参考轴（须 ⊥ nose），
/// 绕 nose 转 ψ；bz = bx×by 与 place.rs 同一手性。
fn body_axes_at(nose: Vec3, bx0: Vec3, psi: f64) -> (Vec3, Vec3, Vec3) {
    let w = cross(nose, bx0);
    let bx = bx0 * psi.cos() + w * psi.sin();
    let bz = cross(bx, nose);
    (bx, nose, bz)
}

#[test]
fn nose_target_angles_body_roll_invariant() {
    // 修复核心恒等式：机头已对准 d̂ 时，任意滚转相位下「当前读数 ==
    // 目标读数」（PD 误差恒 0）。旧固定罗盘架目标与体架读数相减在该
    // 构型非零（滚转耦合根因），对偶断言见下。
    let radial = Vec3::new(0.0, 0.0, 6_371_000.0);
    let d = nose_dir_from_az_el(radial, 0.6, 0.9);
    let up = radial.unit();
    let east = Vec3::new(-up.z, 0.0, up.x).unit();
    // ψ=0 参考轴：east 的垂直 d̂ 分量（保证 bx0 ⊥ d̂）。
    let bx0 = (east - d * dot(east, d)).unit();
    let psis = [
        0.0,
        0.5,
        1.0,
        std::f64::consts::FRAC_PI_2,
        std::f64::consts::PI,
        -2.3,
    ];
    for &psi in &psis {
        let (bx, _by, bz) = body_axes_at(d, bx0, psi);
        // 当前读数：与 pitch_yaw_angles 同一定义（径向在体架的分解）。
        let cur_p = dot(bz, up).clamp(-1.0, 1.0).asin();
        let cur_y = (-dot(bx, up)).clamp(-1.0, 1.0).asin();
        let (tp, ty) = nose_target_angles_body(radial, d, bx, bz);
        assert!(
            (cur_p - tp).abs() < 1e-9 && (cur_y - ty).abs() < 1e-9,
            "ψ={psi}: cur=({cur_p},{cur_y}) tgt=({tp},{ty})"
        );
        // 对偶断言：ψ=π/2 时固定罗盘架目标与当前读数显著偏离（>11°）——
        // 证明本测试能区分新旧两版实现（旧版在该相位必有稳态偏差）。
        if (psi - std::f64::consts::FRAC_PI_2).abs() < 1e-12 {
            let (cp, _cy) = nose_target_angles(radial, d);
            assert!(
                (cur_p - cp).abs() > 0.2,
                "ψ=π/2 时罗盘架目标应与当前读数显著偏离，差={}",
                cur_p - cp
            );
        }
    }
}

#[test]
fn nose_target_angles_body_degenerate_phase_axis() {
    // d̂ 沿体 +Z（bz ∥ d̂）：bz 投影退化 → 回落 bx 参考仍良定（手算锚点）；
    // 非正交输入双重退化（bx = bz = d̂）防御返回 (0,0) 不 NaN。
    let radial = Vec3::new(0.0, 0.0, 6_371_000.0);
    let up = radial.unit();
    let bx = Vec3::new(1.0, 0.0, 0.0);
    let by = up;
    let bz = cross(bx, by);
    // d̂ = bz：目标体架 by* = bz、bz* = bx、bx* = bz×bx。
    let (p, y) = nose_target_angles_body(radial, bz, bx, bz);
    let exp_p = dot(bx, up).clamp(-1.0, 1.0).asin();
    let exp_y = (-dot(cross(bz, bx), up)).clamp(-1.0, 1.0).asin();
    assert!(
        (p - exp_p).abs() < 1e-9 && (y - exp_y).abs() < 1e-9,
        "({p},{y}) vs ({exp_p},{exp_y})"
    );

    // 双重退化防御。
    let (p2, y2) = nose_target_angles_body(radial, bx, bx, bx);
    assert_eq!((p2, y2), (0.0, 0.0));
}
