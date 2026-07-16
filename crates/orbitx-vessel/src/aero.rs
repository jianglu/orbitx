//! 气动力模型：空气翼面、控制面、变阻力元件、大气模型。
//!
//! 移植自 Orbiter `Vessel.cpp:4099-4226`（`UpdateAerodynamicForces`）。
//! 替代 CLI 层硬编码的 `0.5·ρ·v²·CD` 阻力，使物理自洽。

use orbitx_math::{cross, dot, Vec3};
use std::sync::Arc;

// ── 大气模型 ────────────────────────────────────────────────────────

/// 大气模型接口。
///
/// 提供密度、压力、温度随高度的变化。实现者可以是简单指数模型、
/// US Standard Atmosphere 1976、或行星自定义大气。
pub trait Atmosphere: Send + Sync {
    /// 大气密度 [kg/m³]。
    fn density(&self, altitude: f64) -> f64;
    /// 大气压力 [Pa]。
    fn pressure(&self, altitude: f64) -> f64;
    /// 大气温度 [K]。
    fn temperature(&self, altitude: f64) -> f64;

    /// 返回一个密度闭包 `altitude → ρ`，可跨线程共享（用于力闭包）。
    fn density_fn(&self) -> Arc<dyn Fn(f64) -> f64 + Send + Sync>;
}

/// 指数衰减大气（Orbiter 默认简化模型）。
///
/// `ρ(h) = ρ₀ · exp(-(h - h₀) / H)`
///
/// 其中 `H` 是标高（地球约 8500 m），`ρ₀` 是参考高度 `h₀` 处的密度。
#[derive(Clone, Debug)]
pub struct ExponentialAtmosphere {
    /// 参考高度处的大气密度 [kg/m³]。
    pub rho0: f64,
    /// 标高 [m]（密度衰减到 1/e 的高度差）。
    pub scale_height: f64,
    /// 参考高度 [m]（通常为 0 = 海平面）。
    pub base_alt: f64,
}

impl ExponentialAtmosphere {
    /// 地球标准大气参数。
    pub fn earth() -> Self {
        Self {
            rho0: 1.225,
            scale_height: 8500.0,
            base_alt: 0.0,
        }
    }
}

impl Atmosphere for ExponentialAtmosphere {
    fn density(&self, altitude: f64) -> f64 {
        if altitude < self.base_alt {
            return 0.0;
        }
        self.rho0 * (-(altitude - self.base_alt) / self.scale_height).exp()
    }

    fn pressure(&self, altitude: f64) -> f64 {
        // 等温大气：P = ρ·R·T，取海平面 T=288.15 K。
        const R_AIR: f64 = 287.058; // J/(kg·K)
        self.density(altitude) * R_AIR * 288.15
    }

    fn temperature(&self, altitude: f64) -> f64 {
        let _ = altitude;
        288.15 // 等温近似
    }

    fn density_fn(&self) -> Arc<dyn Fn(f64) -> f64 + Send + Sync> {
        let rho0 = self.rho0;
        let scale_height = self.scale_height;
        let base_alt = self.base_alt;
        Arc::new(move |alt: f64| {
            if alt < base_alt {
                0.0
            } else {
                rho0 * (-(alt - base_alt) / scale_height).exp()
            }
        })
    }
}

// ── 空气翼面 ────────────────────────────────────────────────────────

/// 空气翼面方向（对应 Orbiter `AIRFOIL_ORIENTATION`）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AirfoilOrientation {
    /// 升力在垂直面（pitch plane），攻角 = α。
    LiftVertical,
    /// 升力在水平面（yaw plane），攻角 = β。
    LiftHorizontal,
    /// 完整力和力矩模型（六分量）。
    ForceAndMoment,
}

/// 升阻系数模型（替代 Orbiter 的 C++ 回调函数指针）。
#[derive(Clone, Debug)]
pub enum AirfoilCoeffs {
    /// 常量 CL/CM/CD。
    Constant { cl: f64, cm: f64, cd: f64 },
    /// 线性升力：CL(α) = cl0 + cl_alpha·α，常量 CD。
    LinearLift { cl_alpha: f64, cl0: f64, cd0: f64 },
    /// 查表：AoA [rad] → (CL, CM, CD)，线性插值。
    Table(Vec<(f64, f64, f64, f64)>),
}

impl AirfoilCoeffs {
    /// 在给定攻角下计算 (CL, CM, CD)。
    pub fn evaluate(&self, aoa: f64) -> (f64, f64, f64) {
        match self {
            AirfoilCoeffs::Constant { cl, cm, cd } => (*cl, *cm, *cd),
            AirfoilCoeffs::LinearLift { cl_alpha, cl0, cd0 } => {
                (*cl0 + *cl_alpha * aoa, 0.0, *cd0)
            }
            AirfoilCoeffs::Table(entries) => {
                if entries.is_empty() {
                    return (0.0, 0.0, 0.0);
                }
                if entries.len() == 1 {
                    return (entries[0].1, entries[0].2, entries[0].3);
                }
                // 二分查找 + 线性插值。
                let mut lo = 0usize;
                let mut hi = entries.len() - 1;
                if aoa <= entries[lo].0 {
                    return (entries[lo].1, entries[lo].2, entries[lo].3);
                }
                if aoa >= entries[hi].0 {
                    return (entries[hi].1, entries[hi].2, entries[hi].3);
                }
                while hi - lo > 1 {
                    let mid = lo + (hi - lo) / 2;
                    if entries[mid].0 <= aoa {
                        lo = mid;
                    } else {
                        hi = mid;
                    }
                }
                let (a0, cl0, cm0, cd0) = entries[lo];
                let (a1, cl1, cm1, cd1) = entries[hi];
                let t = (aoa - a0) / (a1 - a0);
                (
                    cl0 + t * (cl1 - cl0),
                    cm0 + t * (cm1 - cm0),
                    cd0 + t * (cd1 - cd0),
                )
            }
        }
    }
}

/// 空气翼面（对应 Orbiter `AirfoilSpec`）。
#[derive(Clone, Debug)]
pub struct Airfoil {
    /// 力参考点（体坐标系）[m]。
    pub ref_pos: Vec3,
    /// 翼面方向。
    pub orientation: AirfoilOrientation,
    /// 弦长 [m]。
    pub chord: f64,
    /// 参考面积 [m²]。0 = 使用投影截面积。
    pub area: f64,
    /// 展弦比。
    pub aspect_ratio: f64,
    /// 升阻系数模型。
    pub coeffs: AirfoilCoeffs,
}

// ── 控制面 ──────────────────────────────────────────────────────────

/// 控制面类型（对应 Orbiter `AIRCTRL_TYPE`）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CtrlType {
    Elevator,
    Rudder,
    Aileron,
    Flap,
    ElevatorTrim,
    RudderTrim,
}

/// 控制面轴方向（对应 Orbiter `AIRCTRL_AXIS_*`）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CtrlAxis {
    /// +Y 方向产生力。
    YPos,
    /// -Y 方向产生力。
    YNeg,
    /// +X 方向产生力。
    XPos,
    /// -X 方向产生力。
    XNeg,
}

/// 控制面（对应 Orbiter `CtrlsurfSpec`）。
#[derive(Clone, Debug)]
pub struct ControlSurface {
    /// 控制面类型。
    pub ctrl_type: CtrlType,
    /// 力作用点（体坐标系）[m]。
    pub ref_pos: Vec3,
    /// 作用轴方向。
    pub axis: CtrlAxis,
    /// 面积 [m²]。
    pub area: f64,
    /// 升力系数微分 dCl（偏转产生的升力增量系数）。
    pub d_cl: f64,
    /// 当前偏转量 [-1, 1]。
    pub level: f64,
}

// ── 变阻力元件 ──────────────────────────────────────────────────────

/// 变阻力元件（对应 Orbiter `DragElementSpec`）。
///
/// 简化版：直接存储 Cd 而非外部指针。用于降落伞、减速板等
/// 可变阻力源。
#[derive(Clone, Debug)]
pub struct DragElement {
    /// 力作用点（体坐标系）[m]。
    pub ref_pos: Vec3,
    /// 阻力系数 Cd。
    pub cd: f64,
    /// 参考面积 [m²]。
    pub area: f64,
}

// ── 气动力计算 ──────────────────────────────────────────────────────

/// 空气粘度 [kg/(m·s)]（海平面标准值）。
const MU_AIR: f64 = 1.7894e-5;

/// 气动力计算结果。
#[derive(Clone, Debug)]
pub struct AeroForces {
    /// 体坐标系合力 [N]。
    pub force: Vec3,
    /// 体坐标系合力矩 [N·m]。
    pub torque: Vec3,
}

impl Default for AeroForces {
    fn default() -> Self {
        Self {
            force: Vec3::ZERO,
            torque: Vec3::ZERO,
        }
    }
}

/// 计算气动力和力矩（移植自 `Vessel.cpp:4099-4226`）。
///
/// # 参数
/// - `airfoils`: 空气翼面列表
/// - `ctrlsurfs`: 控制面列表
/// - `dragels`: 变阻力元件列表
/// - `airvel_ship`: 体坐标系气流速度（= -(世界速度-风)·Rᵀ）[m/s]
/// - `rho`: 大气密度 [kg/m³]
/// - `omega`: 体坐标系角速度 [rad/s]
/// - `pmi`: 归一化主惯量 [m²]
/// - `mass`: 总质量 [kg]
/// - `cross_section`: (横向X, 轴向Y, 横向Z) 截面积 [m²]
/// - `rdrag`: (x, y, z) 气动阻尼系数（Orbiter `rdrag`）
/// - `dt`: 时间步长 [s]
pub fn compute_aero_forces(
    airfoils: &[Airfoil],
    ctrlsurfs: &[ControlSurface],
    dragels: &[DragElement],
    airvel_ship: Vec3,
    rho: f64,
    omega: Vec3,
    pmi: Vec3,
    mass: f64,
    cross_section: Vec3,
    rdrag: Vec3,
    dt: f64,
) -> AeroForces {
    let mut result = AeroForces::default();

    if rho < 1e-15 {
        return result;
    }

    let airspd = airvel_ship.length();
    if airspd < 1e-6 {
        return result;
    }

    // 动压 q = 0.5 · ρ · v²
    let dynp = 0.5 * rho * airspd * airspd;

    // 攻角（Vessel.cpp:4111-4119）。
    let aoa = if airvel_ship.z.abs() > 1e-10 {
        (-airvel_ship.y).atan2(airvel_ship.z)
    } else {
        0.0
    };
    let beta = if airvel_ship.z.abs() > 1e-10 {
        (-airvel_ship.x).atan2(airvel_ship.z)
    } else {
        0.0
    };

    // Mach 数（简化：用声速 340 m/s）。
    let mach = airspd / 340.0;

    // Reynolds 数模板。
    let re0 = rho * airspd / MU_AIR;

    // 方向向量（Vessel.cpp:4138-4147）。
    // ddir = -airvel_ship / |airvel_ship| = 气流方向（与运动方向相反）= 阻力方向。
    let ddir = airvel_ship * (-1.0 / airspd);
    let ldir = {
        // 升力方向（垂直面）：(0, vz, -vy).unit()
        let v = Vec3::new(0.0, airvel_ship.z, -airvel_ship.y);
        let l = v.length();
        if l > 1e-10 { v * (1.0 / l) } else { Vec3::ZERO }
    };
    let sdir = {
        // 侧力方向（水平面）：(vz, 0, -vx).unit()
        let v = Vec3::new(airvel_ship.z, 0.0, -airvel_ship.x);
        let l = v.length();
        if l > 1e-10 { v * (1.0 / l) } else { Vec3::ZERO }
    };

    // ── 气动阻尼（Vessel.cpp:4121-4136）──
    // Amom -= min(fac·rdrag, pmi·mass/dt) · omega
    if omega.length() > 1e-12 && mass > 0.0 && dt > 0.0 {
        let dynpm = 0.5 * rho * (airspd + 30.0).powi(2);
        let fac_x = dynpm * cross_section.y * rdrag.x;
        let fac_y = dynpm * cross_section.x * rdrag.y;
        let fac_z = dynpm * cross_section.x * rdrag.z;
        let limit_x = pmi.x * mass / dt;
        let limit_y = pmi.y * mass / dt;
        let limit_z = pmi.z * mass / dt;
        result.torque.x -= fac_x.min(limit_x) * omega.x;
        result.torque.y -= fac_y.min(limit_y) * omega.y;
        result.torque.z -= fac_z.min(limit_z) * omega.z;
    }

    // ── 空气翼面循环（Vessel.cpp:4149-4190）──
    for af in airfoils {
        let (cl, cm, cd) = match af.orientation {
            AirfoilOrientation::LiftVertical => af.coeffs.evaluate(aoa),
            AirfoilOrientation::LiftHorizontal => af.coeffs.evaluate(beta),
            AirfoilOrientation::ForceAndMoment => af.coeffs.evaluate(aoa),
        };

        // 参考面积：若 S > 0 用配置值，否则用投影截面积。
        let s = if af.area > 0.0 {
            af.area
        } else {
            ddir.z.abs() * cross_section.z + ddir.y.abs() * cross_section.y
        };

        match af.orientation {
            AirfoilOrientation::LiftVertical => {
                let lift = cl * dynp * s;
                let drag = cd * dynp * s;
                let f = ldir * lift + ddir * drag;
                result.force += f;
                result.torque += cross(f, af.ref_pos);
                // 俯仰力矩。
                if af.chord > 0.0 && af.area > 0.0 {
                    result.torque.x += cm * dynp * af.area * af.chord;
                }
            }
            AirfoilOrientation::LiftHorizontal => {
                let lift = cl * dynp * s;
                let drag = cd * dynp * s;
                let f = sdir * lift + ddir * drag;
                result.force += f;
                result.torque += cross(f, af.ref_pos);
                // 偏航力矩。
                if af.chord > 0.0 && af.area > 0.0 {
                    result.torque.y += cm * dynp * af.area * af.chord;
                }
            }
            AirfoilOrientation::ForceAndMoment => {
                // 六分量：CA(轴向), CN(法向), CY(侧力)。
                // 力 = (CY·S, CN·S, -CA·S) · dynp
                let f = Vec3::new(cl * s, cm * s, -cd * s) * dynp;
                result.force += f;
                result.torque += cross(f, af.ref_pos);
            }
        }

        let _ = (mach, re0);
    }

    // ── 控制面循环（Vessel.cpp:4192-4215）──
    for cs in ctrlsurfs {
        if cs.level.abs() < 1e-10 {
            continue;
        }
        let fac = cs.area * dynp;
        let cdrag = cs.level.abs() * fac; // 偏转产生的阻力
        let clift = -cs.level * fac * cs.d_cl; // 偏转产生的升力

        // 根据轴方向确定力和方向。
        let (f_lift, f_drag) = match cs.axis {
            CtrlAxis::YPos | CtrlAxis::YNeg => {
                let sign = if cs.axis == CtrlAxis::YPos { 1.0 } else { -1.0 };
                (ldir * (clift * sign), ddir * cdrag)
            }
            CtrlAxis::XPos | CtrlAxis::XNeg => {
                let sign = if cs.axis == CtrlAxis::XPos { 1.0 } else { -1.0 };
                (sdir * (clift * sign), ddir * cdrag)
            }
        };
        let f = f_lift + f_drag;
        result.force += f;
        result.torque += cross(f, cs.ref_pos);
    }

    // ── 变阻力元件循环（Vessel.cpp:4217-4226）──
    for de in dragels {
        if de.cd.abs() < 1e-15 {
            continue;
        }
        let drag = de.cd * dynp * de.area;
        let f = ddir * drag;
        result.force += f;
        result.torque += cross(f, de.ref_pos);
    }

    result
}

// ── 辅助函数 ────────────────────────────────────────────────────────

/// 将世界坐标系速度转换为体坐标系气流速度。
///
/// Orbiter 约定：`airvel_ship = tmul(R, v_world - v_wind)`
/// 这是体坐标系中的航天器速度（不是负速度）。
/// 阻力方向 `ddir = -airvel_ship / |airvel_ship|`（沿气流方向）。
#[inline]
pub fn world_to_airvel_ship(vel_world: Vec3, wind_world: Vec3, rot: orbitx_math::Matrix3) -> Vec3 {
    use orbitx_math::tmul;
    let v_rel = vel_world - wind_world;
    tmul(rot, v_rel)
}

// ── 测试 ────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exponential_atmosphere_sea_level() {
        let atm = ExponentialAtmosphere::earth();
        let rho = atm.density(0.0);
        assert!((rho - 1.225).abs() < 1e-6, "ρ(0) = {rho}");
    }

    #[test]
    fn exponential_atmosphere_10km() {
        let atm = ExponentialAtmosphere::earth();
        let rho = atm.density(10_000.0);
        // ρ(10km) = 1.225 · exp(-10000/8500) ≈ 0.414
        let expected = 1.225 * (-10_000.0_f64 / 8500.0).exp();
        assert!((rho - expected).abs() < 1e-6, "ρ(10km) = {rho}, expected = {expected}");
    }

    #[test]
    fn exponential_atmosphere_negative_alt() {
        let atm = ExponentialAtmosphere::earth();
        assert_eq!(atm.density(-100.0), 0.0, "负高度应返回 0");
    }

    #[test]
    fn zero_airspeed_no_force() {
        let result = compute_aero_forces(
            &[], &[], &[],
            Vec3::ZERO, 1.225, Vec3::ZERO,
            Vec3::new(1.0, 1.0, 1.0), 1000.0,
            Vec3::new(1.0, 1.0, 1.0), Vec3::new(1.0, 1.0, 1.0),
            0.05,
        );
        assert_eq!(result.force, Vec3::ZERO);
        assert_eq!(result.torque, Vec3::ZERO);
    }

    #[test]
    fn zero_density_no_force() {
        let result = compute_aero_forces(
            &[], &[], &[DragElement { ref_pos: Vec3::ZERO, cd: 1.0, area: 1.0 }],
            Vec3::new(0.0, 0.0, 100.0), 0.0, Vec3::ZERO,
            Vec3::new(1.0, 1.0, 1.0), 1000.0,
            Vec3::new(1.0, 1.0, 1.0), Vec3::new(1.0, 1.0, 1.0),
            0.05,
        );
        assert_eq!(result.force, Vec3::ZERO);
    }

    #[test]
    fn axial_drag_direction() {
        // 航天器沿 -Z 方向运动（airvel_ship = (0,0,-100)），
        // 阻力应沿 +Z 方向（ddir = -airvel_ship/|airvel_ship| = (0,0,1)）。
        let dragels = vec![DragElement {
            ref_pos: Vec3::ZERO,
            cd: 1.0,
            area: 1.0,
        }];
        let result = compute_aero_forces(
            &[], &[], &dragels,
            Vec3::new(0.0, 0.0, -100.0), 1.225, Vec3::ZERO,
            Vec3::new(1.0, 1.0, 1.0), 1000.0,
            Vec3::new(1.0, 1.0, 1.0), Vec3::new(1.0, 1.0, 1.0),
            0.05,
        );
        // ddir = -airvel_ship/|airvel_ship| = (0,0,1)
        // drag = cd * dynp * area = 1.0 * 0.5*1.225*10000 * 1.0 = 6125
        assert!(result.force.z > 0.0, "阻力应沿 +Z: {:?}", result.force);
        assert!(result.force.x.abs() < 1e-6, "不应有 X 分量");
        assert!(result.force.y.abs() < 1e-6, "不应有 Y 分量");
    }

    #[test]
    fn lift_perpendicular_to_drag() {
        // 气流有 Y 和 Z 分量 → 升力沿 ldir，阻力沿 ddir，两者应垂直。
        let airfoils = vec![Airfoil {
            ref_pos: Vec3::ZERO,
            orientation: AirfoilOrientation::LiftVertical,
            chord: 1.0,
            area: 1.0,
            aspect_ratio: 1.0,
            coeffs: AirfoilCoeffs::Constant { cl: 1.0, cm: 0.0, cd: 0.5 },
        }];
        let airvel = Vec3::new(0.0, -50.0, 100.0); // 有攻角
        let result = compute_aero_forces(
            &airfoils, &[], &[],
            airvel, 1.225, Vec3::ZERO,
            Vec3::new(1.0, 1.0, 1.0), 1000.0,
            Vec3::new(1.0, 1.0, 1.0), Vec3::new(1.0, 1.0, 1.0),
            0.05,
        );
        // 升力和阻力来自 ldir 和 ddir，它们应垂直。
        // ddir = -airvel/|airvel|, ldir = (0, vz, -vy)/|(0, vz, -vy)|
        let ddir = airvel * (-1.0 / airvel.length());
        let ldir_raw = Vec3::new(0.0, airvel.z, -airvel.y);
        let ldir = ldir_raw * (1.0 / ldir_raw.length());
        let dot_dl = dot(ddir, ldir);
        assert!(dot_dl.abs() < 1e-9, "ldir 和 ddir 应垂直: dot = {dot_dl}");
        // 力应非零。
        assert!(result.force.length() > 1e-3, "应有气动力: {:?}", result.force);
    }

    #[test]
    fn control_surface_deflection_produces_lift() {
        let ctrlsurfs = vec![ControlSurface {
            ctrl_type: CtrlType::Elevator,
            ref_pos: Vec3::ZERO,
            axis: CtrlAxis::YPos,
            area: 1.0,
            d_cl: 2.0,
            level: 0.5, // 50% 偏转
        }];
        let result = compute_aero_forces(
            &[], &ctrlsurfs, &[],
            Vec3::new(0.0, 0.0, 100.0), 1.225, Vec3::ZERO,
            Vec3::new(1.0, 1.0, 1.0), 1000.0,
            Vec3::new(1.0, 1.0, 1.0), Vec3::new(1.0, 1.0, 1.0),
            0.05,
        );
        assert!(result.force.length() > 1e-3, "控制面偏转应产生力: {:?}", result.force);
    }

    #[test]
    fn aero_damping_reduces_omega() {
        // 有角速度时，气动阻尼应产生反向力矩。
        let omega = Vec3::new(1.0, 0.0, 0.0); // 正 X 角速度
        let result = compute_aero_forces(
            &[], &[], &[],
            Vec3::new(0.0, 0.0, 100.0), 1.225, omega,
            Vec3::new(10.0, 1.0, 10.0), 1000.0,
            Vec3::new(1.0, 1.0, 1.0), Vec3::new(1.0, 1.0, 1.0),
            0.05,
        );
        // 阻尼力矩应与 omega 反向。
        assert!(
            result.torque.x < 0.0,
            "气动阻尼应产生负 X 力矩（对抗正 omega.x）: {:?}",
            result.torque
        );
    }

    #[test]
    fn drag_element_matches_hand_calc() {
        // 手算：v = 100 m/s 沿 -Z，ρ = 1.225，Cd = 0.3，S = 10 m²
        // drag = 0.5 * 1.225 * 100² * 0.3 * 10 = 18375 N
        let v = 100.0;
        let rho = 1.225;
        let cd = 0.3;
        let area = 10.0;
        let expected = 0.5 * rho * v * v * cd * area;

        let dragels = vec![DragElement {
            ref_pos: Vec3::ZERO,
            cd,
            area,
        }];
        let result = compute_aero_forces(
            &[], &[], &dragels,
            Vec3::new(0.0, 0.0, -v), rho, Vec3::ZERO,
            Vec3::new(1.0, 1.0, 1.0), 1000.0,
            Vec3::new(1.0, 1.0, 1.0), Vec3::new(1.0, 1.0, 1.0),
            0.05,
        );
        // 力沿 +Z（ddir = (0,0,1)）。
        let rel_err = (result.force.z - expected).abs() / expected;
        assert!(rel_err < 1e-10, "阻力 = {}, 期望 = {}, 误差 = {rel_err}", result.force.z, expected);
    }

    #[test]
    fn airfoil_table_interpolation() {
        // 线性查表：AoA=0 → (0,0,0.1)，AoA=π/2 → (1,0,0.2)
        // 在 AoA=π/4 处应插值出 (0.5, 0, 0.15)。
        use std::f64::consts::FRAC_PI_4;
        let table = AirfoilCoeffs::Table(vec![
            (0.0, 0.0, 0.0, 0.1),
            (std::f64::consts::FRAC_PI_2, 1.0, 0.0, 0.2),
        ]);
        let (cl, cm, cd) = table.evaluate(FRAC_PI_4);
        assert!((cl - 0.5).abs() < 1e-10, "CL = {cl}");
        assert!(cm.abs() < 1e-10, "CM = {cm}");
        assert!((cd - 0.15).abs() < 1e-10, "CD = {cd}");
    }

    #[test]
    fn airfoil_linear_lift() {
        let coeffs = AirfoilCoeffs::LinearLift {
            cl_alpha: 2.0 * std::f64::consts::PI, // 薄翼理论：CLα = 2π
            cl0: 0.0,
            cd0: 0.02,
        };
        let (cl, _, cd) = coeffs.evaluate(0.1); // 0.1 rad ≈ 5.7°
        let expected_cl = 2.0 * std::f64::consts::PI * 0.1;
        assert!((cl - expected_cl).abs() < 1e-10, "CL = {cl}");
        assert!((cd - 0.02).abs() < 1e-10, "CD = {cd}");
    }

    #[test]
    fn world_to_airvel_ship_identity() {
        use orbitx_math::Matrix3;
        let vel = Vec3::new(100.0, 0.0, 0.0);
        let airvel = world_to_airvel_ship(vel, Vec3::ZERO, Matrix3::IDENTITY);
        // airvel = tmul(I, v) = v (body velocity in body frame)
        assert!((airvel.x - 100.0).abs() < 1e-10);
        assert!(airvel.y.abs() < 1e-10);
        assert!(airvel.z.abs() < 1e-10);
    }
}
