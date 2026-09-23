//! 地面接触力：触地点弹簧-阻尼-摩擦模型（移植自 Orbiter
//! `AddSurfaceForces`，`Vessel.cpp:4289-4590`）。
//!
//! 每个触地点（`TouchdownVertex`）在穿透地面时产生弹簧法向力、阻尼力和
//! 摩擦力，模拟着陆架或碰撞响应。`make_landing_gear` 布局构造器留在
//! `orbitx_vessel`（布局/装配关注点），本模块只含物理数据与算法。

use orbitx_math::{cross, dot, mul, tmul, StateVectors, Vec3};

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

    let n = td_points.len();
    let mut fn_total: f64 = 0.0;
    let mut flng_total: f64 = 0.0;
    let mut flat_total: f64 = 0.0;

    // 纵向和横向方向（在体坐标系中）。用第一个触点定义纵向方向。
    let d1_body = if n >= 3 {
        (td_points[0].pos - (td_points[1].pos + td_points[2].pos) * 0.5).unit()
    } else {
        Vec3::new(0.0, 1.0, 0.0)
    };

    let d1_world = mul(rot, d1_body);
    let d1h = (d1_world - radial * dot(d1_world, radial)).unit();
    let d2h = cross(radial, d1h); // 横向方向

    for (i, td) in td_points.iter().enumerate() {
        let pos_world = state.pos + mul(rot, td.pos);
        let pr = pos_world.length();
        let altitude = pr - planet_radius;

        let penetration = altitude;

        if penetration >= 0.0 {
            continue;
        }

        result.in_contact = true;
        result.max_penetration = result.max_penetration.min(penetration);

        let v_body = cross(state.omega, td.pos);
        let ground_vel = state.vel + mul(rot, v_body);

        let gv_n = dot(ground_vel, radial);
        let gv_lng = dot(ground_vel, d1h);
        let gv_lat = dot(ground_vel, d2h);

        let mut f_normal = -penetration * td.stiffness;
        f_normal -= gv_n * td.damping;

        let fn_max = -gv_n * mass / dt;
        if f_normal > fn_max && fn_max > 0.0 {
            f_normal = fn_max;
        }
        if f_normal < 0.0 {
            f_normal = 0.0;
        }

        let max_press = (-penetration).min(0.1) * td.stiffness;
        let mu = if i < 3 { td.mu_lng } else { td.mu };
        let mut flng = mu * max_press;
        let flat = mu * max_press;

        if gv_lng.abs() < 10.0 {
            flng *= (0.1 * gv_lng.abs()).sqrt().min(1.0);
        }
        let flng_signed = if gv_lng.abs() > 1e-6 { -flng * gv_lng.signum() } else { 0.0 };
        let flat_signed = if gv_lat.abs() > 1e-6 { -flat * gv_lat.signum() } else { 0.0 };

        fn_total += f_normal;
        flng_total += flng_signed;
        flat_total += flat_signed;

        let f_point = radial * f_normal + d1h * flng_signed + d2h * flat_signed;
        let tau_point = cross(mul(rot, td.pos), f_point);
        result.torque += tmul(rot, tau_point);
    }

    if result.in_contact {
        result.force = radial * fn_total + d1h * flng_total + d2h * flat_total;

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

#[cfg(test)]
mod tests {
    use super::*;
    use orbitx_math::Matrix3;
    use orbitx_math::Quat;

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
        let state = make_state_at_alt(-0.1, Vec3::ZERO);
        let result = compute_surface_forces(&td, &state, 6_371_000.0, 0.05, 1000.0);
        assert!(result.in_contact);
        assert!(result.force.z > 0.0, "spring force should be upward: {:?}", result.force);
    }

    #[test]
    fn damping_opposes_velocity() {
        let td = vec![TouchdownVertex::new(Vec3::ZERO, 1e6, 1e5, 0.0)];
        let state = make_state_at_alt(-0.01, Vec3::new(0.0, 0.0, -10.0));
        let result = compute_surface_forces(&td, &state, 6_371_000.0, 0.05, 1000.0);
        assert!(result.in_contact);
        assert!(result.force.z > 0.0, "damping should add upward force: {:?}", result.force);
    }

    #[test]
    fn friction_opposes_sliding() {
        let td = vec![TouchdownVertex::new(Vec3::ZERO, 1e6, 0.0, 0.5)];
        let state = make_state_at_alt(-0.05, Vec3::new(20.0, 0.0, 0.0));
        let result = compute_surface_forces(&td, &state, 6_371_000.0, 0.05, 1000.0);
        assert!(result.in_contact);
        assert!(result.force.x < 0.0, "friction should oppose sliding: {:?}", result.force);
    }

    #[test]
    fn hard_landing_produces_large_force() {
        let td = vec![TouchdownVertex::new(Vec3::ZERO, 1e7, 1e5, 0.5)];
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
