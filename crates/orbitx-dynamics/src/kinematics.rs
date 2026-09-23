//! 姿态运动学：由 `StateVectors` 导出姿态角（俯仰/偏航/滚转/tip）。
//!
//! 移自 vessel::attitude；属状态级运动学，归 dynamics。输入为纯数学类型
//! `StateVectors`，不依赖任何 vessel 类型。

use orbitx_math::{cross, dot, mul, tmul, StateVectors, Vec3};

/// 有符号俯仰/偏航角 [rad]（体轴相对当地径向）。
pub fn pitch_yaw_angles(state: &StateVectors) -> (f64, f64) {
    let (sp, sy) = attitude_errors(state);
    (
        sp.clamp(-1.0, 1.0).asin(),
        sy.clamp(-1.0, 1.0).asin(),
    )
}

/// 俯仰/偏航误差（径向在体坐标的分量，未 asin）。
pub fn attitude_errors(state: &StateVectors) -> (f64, f64) {
    let r_mag = state.pos.length();
    if r_mag < 1e-3 {
        return (0.0, 0.0);
    }
    let radial = state.pos * (1.0 / r_mag);
    let radial_body = tmul(state.r, radial);
    (radial_body.z, -radial_body.x)
}

/// 体 +Y 与径向夹角 [rad]。
pub fn tip_angle(state: &StateVectors) -> f64 {
    let r_mag = state.pos.length();
    if r_mag < 1e-3 {
        return 0.0;
    }
    let radial = state.pos * (1.0 / r_mag);
    let body_y = mul(state.r, Vec3::new(0.0, 1.0, 0.0));
    dot(body_y, radial).clamp(-1.0, 1.0).acos()
}

/// 绕体 +Y 滚转角 [rad]（当地东向参考）。
pub fn roll_angle(state: &StateVectors) -> f64 {
    let r_mag = state.pos.length();
    if r_mag < 1e-3 {
        return 0.0;
    }
    let pos = state.pos;
    let east = Vec3::new(-pos.z, 0.0, pos.x);
    if east.length() < 1e-9 {
        let radial = pos * (1.0 / r_mag);
        let east = cross(Vec3::new(0.0, 1.0, 0.0), radial);
        if east.length() < 1e-9 {
            return 0.0;
        }
        return roll_about_body_y(state.r, east.unit());
    }
    roll_about_body_y(state.r, east.unit())
}

fn roll_about_body_y(r: orbitx_math::Matrix3, east: Vec3) -> f64 {
    let body_x = mul(r, Vec3::new(1.0, 0.0, 0.0));
    let body_y = mul(r, Vec3::new(0.0, 1.0, 0.0));
    let body_z = mul(r, Vec3::new(0.0, 0.0, 1.0));
    let mut refr = east - body_y * dot(east, body_y);
    if refr.length() < 1e-9 {
        return 0.0;
    }
    refr = refr.unit();
    dot(body_z, refr).atan2(dot(body_x, refr))
}

#[cfg(test)]
mod tests {
    use super::*;
    use orbitx_math::{Matrix3, PI05, Quat, StateVectors, Vec3};

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
}
