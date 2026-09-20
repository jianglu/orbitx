//! 发射台 / 表面惯性初速（地球自转 ω×r）。

use orbitx_math::{cross, Vec3};

/// 表面点在惯性系中的共转速度：`v = ω × r`。
///
/// `sid_rot_period`：恒星自转周期 [s]；`pos`：相对天体质心位置（惯性系）[m]。
/// 自转轴取惯性系 **+Y**（与 CLI 纬度 `asin(y/r)` 约定一致；不含黄赤交角）。
pub fn surface_inertial_velocity(pos: Vec3, sid_rot_period: f64) -> Vec3 {
    if sid_rot_period.abs() < 1e-9 {
        return Vec3::ZERO;
    }
    let omega = Vec3::new(0.0, std::f64::consts::TAU / sid_rot_period, 0.0);
    cross(omega, pos)
}

#[cfg(test)]
mod tests;
