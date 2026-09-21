//! 由步进写回后的 `StateVectors` 导出姿态角（唯一几何实现，供 diagnostics 写入）。

use orbitx_math::{cross, dot, mul, tmul, Matrix3, StateVectors, Vec3};

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

fn roll_about_body_y(r: Matrix3, east: Vec3) -> f64 {
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
