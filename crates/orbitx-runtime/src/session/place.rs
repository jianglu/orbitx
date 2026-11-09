//! 发射台 / scenario 初始状态（地心系，地球在原点）。
//!
//! 地理经纬 / ENU 在**行星体固系**计算，再经 [`PlanetOrientation`] 转到惯性系。

use orbitx_dynamics::{to_inertial, PlanetOrientation};
use orbitx_math::{cross, Matrix3, Quat, StateVectors, Vec3};
use orbitx_vessel::{geocentric_up, surface_inertial_velocity, StageSpec};

/// 体 +Y（头部）对齐径向 up。
pub fn launch_attitude(up: Vec3) -> (Matrix3, Quat) {
    attitude_for_axis(up, 0.0)
}

/// 体 +Y 对齐给定轴向，并绕轴旋转 heading（弧度，正向按右手则）。
pub fn attitude_for_axis(axis: Vec3, heading_rad: f64) -> (Matrix3, Quat) {
    let up = axis.unit();
    let ref_axis = if up.y.abs() < 0.9 {
        Vec3::new(0.0, 1.0, 0.0)
    } else {
        Vec3::new(1.0, 0.0, 0.0)
    };
    let mut bx = cross(up, ref_axis).unit();
    let mut bz = cross(bx, up).unit();
    if heading_rad.abs() > 0.0 {
        let (c, s) = (heading_rad.cos(), heading_rad.sin());
        let rotate = |v: Vec3| v * c + cross(up, v) * s;
        bx = rotate(bx);
        bz = rotate(bz);
    }
    let by = up;
    let r = Matrix3::new(bx.x, by.x, bz.x, bx.y, by.y, bz.y, bx.z, by.z, bz.z);
    let q = Quat::from_matrix(r);
    (r, q)
}

/// 发射场当地 ENU 基（**体固系**）：(east, north, up)。与地表一致性规范一致。
pub fn enu_basis_geocentric(lat_deg: f64, lon_deg: f64) -> (Vec3, Vec3, Vec3) {
    let up = geocentric_up(lat_deg, lon_deg);
    let lat = lat_deg.to_radians();
    let lon = lon_deg.to_radians();
    let east = Vec3::new(-lon.sin(), 0.0, lon.cos());
    let north = Vec3::new(-lat.sin() * lon.cos(), lat.cos(), -lat.sin() * lon.sin());
    (east, north, up)
}

/// 台位支持面（火箭底部落点）地心半径：`R_elev + base + 台架高度`。
///
/// 发射场坐标系换算，不走地形采样：地形低于场坪时台架等效架空、
/// 高于场坪时采样侧截平（见 `surface_session`），两侧与本式自洽。
pub fn pad_support_radius_m(elev_radius_m: f64, base_elevation_m: f64, mount_height_m: f64) -> f64 {
    elev_radius_m + base_elevation_m + mount_height_m
}

/// 体固姿态矩阵 → 惯性：`R_inertial = R_planet · R_body`。
fn attitude_to_inertial(planet_rot: Matrix3, r_body: Matrix3) -> (Matrix3, Quat) {
    let r = planet_rot.matmul(r_body);
    let q = Quat::from_matrix(r);
    (r, q)
}

/// 按发射场台位摆放：轴基点当地偏移 → 台位方向，支持面按发射场坐标系换算
/// （`R_elev + base_elevation_m + axis_offset_m[2]`，上分量只加一次），
/// 质心沿轴 +half_h 近似（装配后由会话按底部精确归零），姿态绕轴 heading，初速共转。
///
/// 经纬/ENU 在体固系；`orient` 转到惯性系。
#[allow(clippy::too_many_arguments)]
pub fn initial_state_on_pad(
    stages: &[StageSpec],
    lat_deg: f64,
    lon_deg: f64,
    axis_offset_m: [f64; 3],
    axis_dir_enu: [f64; 3],
    heading_deg: f64,
    elev_radius_m: f64,
    base_elevation_m: f64,
    orient: PlanetOrientation,
) -> StateVectors {
    let half_h: f64 = stages.iter().map(|s| s.length).sum::<f64>() / 2.0;
    let (east, north, up) = enu_basis_geocentric(lat_deg, lon_deg);

    // 场坪平面上的台位点（东/北偏移）→ 方向；支持面半径沿该方向按坐标系换算。
    let pad_point = up * (elev_radius_m + base_elevation_m)
        + east * axis_offset_m[0]
        + north * axis_offset_m[1];
    let pad_dir = pad_point.unit();
    let support_point =
        pad_dir * pad_support_radius_m(elev_radius_m, base_elevation_m, axis_offset_m[2]);

    let axis_body =
        (east * axis_dir_enu[0] + north * axis_dir_enu[1] + up * axis_dir_enu[2]).unit();
    let pos_body = support_point + axis_body * half_h;
    let (r_body, _) = attitude_for_axis(axis_body, heading_deg.to_radians());

    let pos = to_inertial(orient.rot, pos_body);
    let (init_r, init_q) = attitude_to_inertial(orient.rot, r_body);
    let vel = surface_inertial_velocity(pos, orient.omega);
    StateVectors {
        pos,
        vel,
        r: init_r,
        q: init_q,
        ..Default::default()
    }
}
