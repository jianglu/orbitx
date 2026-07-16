//! 着陆/碰撞检测：触地点弹簧-阻尼-摩擦模型。
//!
//! 移植自 Orbiter `AddSurfaceForces`（`Vessel.cpp:4289-4590`）。
//! 每个触地点（`TouchdownVertex`）在穿透地面时产生弹簧法向力、
//! 阻尼力和摩擦力，模拟着陆架或碰撞响应。

use orbitx_math::{cross, dot, mul, tmul, Quat, StateVectors, Vec3};

/// 着陆触点（对应 Orbiter `TOUCHDOWN_VTX`，`Vesselbase.h:19`）。
#[derive(Clone, Debug)]
pub struct TouchdownVertex {
    /// 体坐标系位置 [m]。
    pub pos: Vec3,
    /// 弹簧常数 [N/m]。
    pub stiffness: f64,
    /// 阻尼系数 [N*s/m]。
    pub damping: f64,
    /// 各向同性/横向摩擦系数。
    pub mu: f64,
    /// 纵向摩擦系数（前 3 个触点有效，用于车轮制动）。
    pub mu_lng: f64,
}

impl TouchdownVertex {
    /// 创建新的触点。
    pub fn new(pos: Vec3, stiffness: f64, damping: f64, mu: f64) -> Self {
        Self { pos, stiffness, damping, mu, mu_lng: mu }
    }

    /// 创建带纵向摩擦的触点。
    pub fn with_mu_lng(pos: Vec3, stiffness: f64, damping: f64, mu: f64, mu_lng: f64) -> Self {
        Self { pos, stiffness, damping, mu, mu_lng }
    }
}

/// 地面接触力计算结果。
#[derive(Clone, Debug)]
pub struct SurfaceContact {
    /// 世界坐标系合力 [N]。
    pub force: Vec3,
    /// 体坐标系合力矩 [N*m]。
    pub torque: Vec3,
    /// 是否有触点接触地面。
    pub in_contact: bool,
    /// 最大穿透深度 [m]（负值表示穿透）。
    pub max_penetration: f64,
}

impl Default for SurfaceContact {
    fn default() -> Self {
        Self {
            force: Vec3::ZERO,
            torque: Vec3::ZERO,
            in_contact: false,
            max_penetration: 0.0,
        }
    }
}

/// 计算地面接触力（简化版 `Vessel.cpp:4289-4590`）。
///
/// # 算法
/// 1. 对每个触点，将体坐标位置转到世界坐标，计算穿透深度。
/// 2. 穿透时：弹簧法向力 + 阻尼力 + 摩擦力。
/// 3. 力限幅：防止速度反转。
/// 4. 汇总力和力矩。
///
/// # 参数
/// - `td_points`: 触地点列表
/// - `state`: 当前运动状态
/// - `planet_radius`: 行星半径 [m]
/// - `dt`: 时间步长 [s]
/// - `mass`: 总质量 [kg]
pub fn compute_surface_forces(
    td_points: &[TouchdownVertex],
    state: &StateVectors,
    planet_radius: f64,
    dt: f64,
    mass: f64,
) -> SurfaceContact {
    if td_points.is_empty() || mass <= 0.0 || dt <= 0.0 {
        return SurfaceContact::default();
    }

    let mut result = SurfaceContact::default();

    // 径向方向（从地心指向航天器）。
    let r_mag = state.pos.length();
    if r_mag < 1e-3 {
        return result;
    }
    let radial = state.pos * (1.0 / r_mag);

    // 体→世界旋转矩阵。
    let rot = state.r;

    // 对每个触点计算穿透和接触力。
    let n = td_points.len();
    let mut fn_total: f64 = 0.0; // 法向力总和
    let mut flng_total: f64 = 0.0; // 纵向摩擦力总和
    let mut flat_total: f64 = 0.0; // 横向摩擦力总和

    // 纵向和横向方向（在体坐标系中）。
    // 用第一个触点定义纵向方向。
    let d1_body = if n >= 3 {
        (td_points[0].pos - (td_points[1].pos + td_points[2].pos) * 0.5).unit()
    } else {
        Vec3::new(0.0, 1.0, 0.0)
    };

    // 转世界坐标。
    let d1_world = mul(rot, d1_body);
    // 投影到水平面（垂直于径向）。
    let d1h = (d1_world - radial * dot(d1_world, radial)).unit();
    let d2h = cross(radial, d1h); // 横向方向

    for (i, td) in td_points.iter().enumerate() {
        // 触点世界坐标位置。
        let pos_world = state.pos + mul(rot, td.pos);
        let pr = pos_world.length();
        let altitude = pr - planet_radius;

        // 穿透深度（负值 = 穿透）。
        let penetration = altitude;

        if penetration >= 0.0 {
            continue; // 未穿透
        }

        result.in_contact = true;
        result.max_penetration = result.max_penetration.min(penetration);

        // 地面速度 = 世界速度 + omega × r_body（转世界坐标）。
        let v_body = cross(state.omega, td.pos);
        let ground_vel = state.vel + mul(rot, v_body);

        // 分解地面速度。
        let gv_n = dot(ground_vel, radial); // 法向（下沉为负）
        let gv_lng = dot(ground_vel, d1h); // 纵向
        let gv_lat = dot(ground_vel, d2h); // 横向

        // 弹簧法向力（向上，对抗穿透）。
        let mut f_normal = -penetration * td.stiffness;

        // 阻尼力（对抗下沉速度）。
        f_normal -= gv_n * td.damping;

        // 限幅：防止反弹（法向力不能超过使速度反转的量）。
        let fn_max = -gv_n * mass / dt;
        if f_normal > fn_max && fn_max > 0.0 {
            f_normal = fn_max;
        }
        if f_normal < 0.0 {
            f_normal = 0.0; // 不产生吸引力
        }

        // 摩擦力。
        let max_press = (-penetration).min(0.1) * td.stiffness;
        let mu = if i < 3 { td.mu_lng } else { td.mu };
        let mut flng = mu * max_press;
        let flat = mu * max_press;

        // 摩擦力方向：反向于滑动速度。
        // 低速时缩放摩擦力（防止数值抖动）。
        if gv_lng.abs() < 10.0 {
            flng *= (0.1 * gv_lng.abs()).sqrt().min(1.0);
        }
        let flng_signed = if gv_lng.abs() > 1e-6 { -flng * gv_lng.signum() } else { 0.0 };
        let flat_signed = if gv_lat.abs() > 1e-6 { -flat * gv_lat.signum() } else { 0.0 };

        // 累加到总力。
        fn_total += f_normal;
        flng_total += flng_signed;
        flat_total += flat_signed;

        // 各触点的力和力矩（用于力矩计算）。
        let f_point = radial * f_normal + d1h * flng_signed + d2h * flat_signed;
        let tau_point = cross(mul(rot, td.pos), f_point);
        // 转体坐标系力矩。
        result.torque += tmul(rot, tau_point);
    }

    if result.in_contact {
        result.force = radial * fn_total + d1h * flng_total + d2h * flat_total;

        // 力限幅：防止纵向/横向速度反转。
        let gv_lng_total = dot(state.vel, d1h);
        let gv_lat_total = dot(state.vel, d2h);
        let fmax_lng = -gv_lng_total * mass / dt;
        let fmax_lat = -gv_lat_total * mass / dt;

        let flng_component = dot(result.force, d1h);
        if flng_component.abs() > fmax_lng.abs() && fmax_lng.abs() > 0.0 {
            let scale = fmax_lng.abs() / flng_component.abs();
            result.force = radial * fn_total + d1h * flng_total * scale + d2h * flat_total;
        }
        let flat_component = dot(result.force, d2h);
        if flat_component.abs() > fmax_lat.abs() && fmax_lat.abs() > 0.0 {
            let scale = fmax_lat.abs() / flat_component.abs();
            result.force = radial * fn_total + d1h * flng_total + d2h * flat_total * scale;
        }
    }

    result
}

/// 创建三点着陆架（三角布局）。
///
/// 典型配置：3 个触点在底部等间隔分布，模拟着陆三角架。
///
/// # 参数
/// - `radius`: 着陆架半径 [m]（触点到纵轴距离）
/// - `y_offset`: 着陆架在体坐标系 Y 的位置 [m]（负值 = 底部）
/// - `stiffness`: 弹簧常数 [N/m]
/// - `damping`: 阻尼系数 [N*s/m]
/// - `mu`: 摩擦系数
pub fn make_landing_gear(radius: f64, y_offset: f64, stiffness: f64, damping: f64, mu: f64) -> Vec<TouchdownVertex> {
    let mut pts = Vec::with_capacity(3);
    for i in 0..3 {
        let angle = i as f64 * std::f64::consts::TAU / 3.0;
        let x = radius * angle.cos();
        let z = radius * angle.sin();
        pts.push(TouchdownVertex::with_mu_lng(
            Vec3::new(x, y_offset, z),
            stiffness, damping, mu, mu * 0.8,
        ));
    }
    pts
}

#[cfg(test)]
mod tests {
    use super::*;
    use orbitx_math::Matrix3;

    fn make_state_at_alt(alt: f64, vel: Vec3) -> StateVectors {
        StateVectors {
            pos: Vec3::new(0.0, 0.0, 6_371_000.0 + alt),
            vel,
            omega: Vec3::ZERO,
            r: Matrix3::IDENTITY,
            q: Quat::IDENTITY,
        }
    }

    #[test]
    fn no_contact_when_above_ground() {
        let td = vec![TouchdownVertex::new(Vec3::ZERO, 1e6, 1e4, 0.5)];
        let state = make_state_at_alt(100.0, Vec3::ZERO);
        let result = compute_surface_forces(&td, &state, 6_371_000.0, 0.05, 1000.0);
        assert!(!result.in_contact);
        assert_eq!(result.force, Vec3::ZERO);
    }

    #[test]
    fn spring_force_opposes_penetration() {
        let td = vec![TouchdownVertex::new(Vec3::ZERO, 1e6, 0.0, 0.0)];
        let state = make_state_at_alt(-0.1, Vec3::ZERO); // 0.1 m penetration
        let result = compute_surface_forces(&td, &state, 6_371_000.0, 0.05, 1000.0);
        assert!(result.in_contact);
        // Force should be along +Z (upward, opposing penetration).
        assert!(result.force.z > 0.0, "spring force should be upward: {:?}", result.force);
    }

    #[test]
    fn damping_opposes_velocity() {
        let td = vec![TouchdownVertex::new(Vec3::ZERO, 1e6, 1e5, 0.0)];
        // Sinking at 10 m/s.
        let state = make_state_at_alt(-0.01, Vec3::new(0.0, 0.0, -10.0));
        let result = compute_surface_forces(&td, &state, 6_371_000.0, 0.05, 1000.0);
        assert!(result.in_contact);
        // Damping should add to the upward force.
        assert!(result.force.z > 0.0, "damping should add upward force: {:?}", result.force);
    }

    #[test]
    fn friction_opposes_sliding() {
        let td = vec![TouchdownVertex::new(Vec3::ZERO, 1e6, 0.0, 0.5)];
        // Sliding in +X at 20 m/s while in contact.
        let state = make_state_at_alt(-0.05, Vec3::new(20.0, 0.0, 0.0));
        let result = compute_surface_forces(&td, &state, 6_371_000.0, 0.05, 1000.0);
        assert!(result.in_contact);
        // Friction should oppose sliding: force should have -X component.
        assert!(result.force.x < 0.0, "friction should oppose sliding: {:?}", result.force);
    }

    #[test]
    fn three_point_landing_gear() {
        let gear = make_landing_gear(2.0, -5.0, 1e6, 1e4, 0.5);
        assert_eq!(gear.len(), 3);
        // All points should be at y = -5.
        for pt in &gear {
            assert!((pt.pos.y - (-5.0)).abs() < 1e-10);
        }
        // Points should be at radius 2 from Y axis.
        for pt in &gear {
            let r = (pt.pos.x * pt.pos.x + pt.pos.z * pt.pos.z).sqrt();
            assert!((r - 2.0).abs() < 1e-10);
        }
    }

    #[test]
    fn hard_landing_produces_large_force() {
        let td = vec![TouchdownVertex::new(Vec3::ZERO, 1e7, 1e5, 0.5)];
        // Fast sinking at 100 m/s, 0.5 m penetration.
        let state = make_state_at_alt(-0.5, Vec3::new(0.0, 0.0, -100.0));
        let result = compute_surface_forces(&td, &state, 6_371_000.0, 0.05, 1000.0);
        assert!(result.in_contact);
        assert!(result.force.z > 1e6, "hard landing should produce very large force: {:?}", result.force);
    }

    #[test]
    fn no_touchdown_points_no_force() {
        let state = make_state_at_alt(-1.0, Vec3::ZERO);
        let result = compute_surface_forces(&[], &state, 6_371_000.0, 0.05, 1000.0);
        assert!(!result.in_contact);
    }
}
