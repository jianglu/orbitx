//! 气动力模型：空气翼面、控制面、变阻力元件（移植自 Orbiter
//! `Vessel.cpp:4099-4226` `UpdateAerodynamicForces`）。
//!
//! 本模块含气动力计算所需的数据类型与纯算法；大气模型见
//! [`crate::atmosphere`]，Cd(M) 查表插值见 `orbitx_math::piecewise_linear`。

use orbitx_math::{cross, dot, piecewise_linear, tmul, Matrix3, Vec3};

// ── 空气翼面 ────────────────────────────────────────────────────────

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AirfoilOrientation {
    LiftVertical,
    LiftHorizontal,
    ForceAndMoment,
}

#[derive(Clone, Debug)]
pub enum AirfoilCoeffs {
    Constant { cl: f64, cm: f64, cd: f64 },
    LinearLift { cl_alpha: f64, cl0: f64, cd0: f64 },
    /// `(aoa, [cl, cm, cd])` 查表，按 aoa 升序；插值复用
    /// `orbitx_math::piecewise_linear` 的多分量核心。
    Table(Vec<(f64, [f64; 3])>),
}

impl AirfoilCoeffs {
    pub fn evaluate(&self, aoa: f64) -> (f64, f64, f64) {
        match self {
            AirfoilCoeffs::Constant { cl, cm, cd } => (*cl, *cm, *cd),
            AirfoilCoeffs::LinearLift { cl_alpha, cl0, cd0 } => {
                (*cl0 + *cl_alpha * aoa, 0.0, *cd0)
            }
            AirfoilCoeffs::Table(entries) => {
                let [cl, cm, cd] = piecewise_linear(entries, aoa, [0.0; 3]);
                (cl, cm, cd)
            }
        }
    }
}

#[derive(Clone, Debug)]
pub struct Airfoil {
    pub ref_pos: Vec3,
    pub orientation: AirfoilOrientation,
    pub chord: f64,
    pub area: f64,
    pub aspect_ratio: f64,
    pub coeffs: AirfoilCoeffs,
}

// ── 控制面 ──────────────────────────────────────────────────────────

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CtrlType {
    Elevator,
    Rudder,
    Aileron,
    Flap,
    ElevatorTrim,
    RudderTrim,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CtrlAxis {
    YPos,
    YNeg,
    XPos,
    XNeg,
}

#[derive(Clone, Debug)]
pub struct ControlSurface {
    pub ctrl_type: CtrlType,
    pub ref_pos: Vec3,
    pub axis: CtrlAxis,
    pub area: f64,
    pub d_cl: f64,
    pub level: f64,
}

// ── 变阻力元件 ──────────────────────────────────────────────────────

/// 变阻力元件；可选 Cd(M) 表覆盖常量 `cd`。
#[derive(Clone, Debug)]
pub struct DragElement {
    pub ref_pos: Vec3,
    /// 常量阻力系数（无 `cd_mach` 时使用）。
    pub cd: f64,
    pub area: f64,
    /// 可选 Cd(M) 表。
    pub cd_mach: Option<Vec<(f64, f64)>>,
}

impl DragElement {
    pub fn constant(ref_pos: Vec3, cd: f64, area: f64) -> Self {
        Self {
            ref_pos,
            cd,
            area,
            cd_mach: None,
        }
    }

    pub fn with_cd_mach(mut self, table: Vec<(f64, f64)>) -> Self {
        self.cd_mach = Some(table);
        self
    }

    pub fn effective_cd(&self, mach: f64) -> f64 {
        match &self.cd_mach {
            Some(table) => piecewise_linear(table, mach, self.cd),
            None => self.cd,
        }
    }
}

// ── 气动力计算 ──────────────────────────────────────────────────────

/// 世界系相对风速 → 体坐标系空速（Orbiter `tmul(GRot(), vel - wind)`）。
pub fn world_to_airvel_ship(vel: Vec3, wind: Vec3, rot: Matrix3) -> Vec3 {
    tmul(rot, vel - wind)
}

const MU_AIR: f64 = 1.7894e-5;

/// 气动力计算结果。
#[derive(Clone, Debug, Default)]
pub struct AeroForces {
    pub force: Vec3,
    pub torque: Vec3,
    /// 马赫数（无大气/零速时为 0）。
    pub mach: f64,
    /// 动压 [Pa]。
    pub dynamic_pressure: f64,
    /// 阻力元件贡献的阻力合力模 [N]。
    pub drag_force: f64,
    /// 阻力加权平均有效 Cd。
    pub cd_eff: f64,
}

/// 计算气动力和力矩。
///
/// `sound_speed`：当地声速 [m/s]，用于 Ma = v/a；须由大气温度导出，禁止写死。
#[allow(clippy::too_many_arguments)]
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
    sound_speed: f64,
) -> AeroForces {
    let mut result = AeroForces::default();

    if rho < 1e-15 {
        return result;
    }

    let airspd = airvel_ship.length();
    if airspd < 1e-6 {
        return result;
    }

    let dynp = 0.5 * rho * airspd * airspd;
    result.dynamic_pressure = dynp;

    let a = sound_speed.max(1.0);
    let mach = airspd / a;
    result.mach = mach;

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

    let re0 = rho * airspd / MU_AIR;
    let _ = re0; // Re 本期不驱动系数

    let ddir = airvel_ship * (-1.0 / airspd);
    let ldir = {
        let v = Vec3::new(0.0, airvel_ship.z, -airvel_ship.y);
        let l = v.length();
        if l > 1e-10 {
            v * (1.0 / l)
        } else {
            Vec3::ZERO
        }
    };
    let sdir = {
        let v = Vec3::new(airvel_ship.z, 0.0, -airvel_ship.x);
        let l = v.length();
        if l > 1e-10 {
            v * (1.0 / l)
        } else {
            Vec3::ZERO
        }
    };

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

    for af in airfoils {
        let (cl, cm, cd) = match af.orientation {
            AirfoilOrientation::LiftVertical => af.coeffs.evaluate(aoa),
            AirfoilOrientation::LiftHorizontal => af.coeffs.evaluate(beta),
            AirfoilOrientation::ForceAndMoment => af.coeffs.evaluate(aoa),
        };

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
                if af.chord > 0.0 && af.area > 0.0 {
                    result.torque.y += cm * dynp * af.area * af.chord;
                }
            }
            AirfoilOrientation::ForceAndMoment => {
                let f = Vec3::new(cl * s, cm * s, -cd * s) * dynp;
                result.force += f;
                result.torque += cross(f, af.ref_pos);
            }
        }
    }

    for cs in ctrlsurfs {
        if cs.level.abs() < 1e-10 {
            continue;
        }
        let fac = cs.area * dynp;
        let cdrag = cs.level.abs() * fac;
        let clift = -cs.level * fac * cs.d_cl;

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

    let mut drag_sum = 0.0;
    let mut area_cd_sum = 0.0;
    let mut area_sum = 0.0;
    for de in dragels {
        let cd = de.effective_cd(mach);
        if cd.abs() < 1e-15 {
            continue;
        }
        let drag = cd * dynp * de.area;
        drag_sum += drag;
        area_cd_sum += cd * de.area;
        area_sum += de.area;
        let f = ddir * drag;
        result.force += f;
        result.torque += cross(f, de.ref_pos);
    }
    result.drag_force = drag_sum;
    result.cd_eff = if area_sum > 1e-15 {
        area_cd_sum / area_sum
    } else {
        0.0
    };

    let _ = dot;
    result
}

#[cfg(test)]
mod tests {
    use super::*;
    use orbitx_math::{Matrix3, PI, PI025, PI05};

    #[test]
    fn world_to_airvel_ship_identity() {
        let vel = Vec3::new(100.0, 0.0, 0.0);
        let airvel = world_to_airvel_ship(vel, Vec3::ZERO, Matrix3::IDENTITY);
        assert!((airvel.x - 100.0).abs() < 1e-10);
    }

    #[test]
    fn zero_airspeed_no_force() {
        let result = compute_aero_forces(
            &[], &[], &[], Vec3::ZERO, 1.225, Vec3::ZERO,
            Vec3::new(1.0, 1.0, 1.0), 1000.0, Vec3::new(1.0, 1.0, 1.0),
            Vec3::new(1.0, 1.0, 1.0), 0.05, 340.0,
        );
        assert_eq!(result.force, Vec3::ZERO);
    }

    #[test]
    fn zero_density_no_force() {
        let result = compute_aero_forces(
            &[], &[], &[DragElement::constant(Vec3::ZERO, 1.0, 1.0)],
            Vec3::new(0.0, 0.0, 100.0), 0.0, Vec3::ZERO,
            Vec3::new(1.0, 1.0, 1.0), 1000.0, Vec3::new(1.0, 1.0, 1.0),
            Vec3::new(1.0, 1.0, 1.0), 0.05, 340.0,
        );
        assert_eq!(result.force, Vec3::ZERO);
    }

    #[test]
    fn constant_cd_matches_legacy() {
        let v = 100.0;
        let rho = 1.225;
        let cd = 0.3;
        let area = 10.0;
        let expected = 0.5 * rho * v * v * cd * area;
        let dragels = vec![DragElement::constant(Vec3::ZERO, cd, area)];
        let result = compute_aero_forces(
            &[], &[], &dragels, Vec3::new(0.0, 0.0, -v), rho, Vec3::ZERO,
            Vec3::new(1.0, 1.0, 1.0), 1000.0, Vec3::new(1.0, 1.0, 1.0),
            Vec3::new(1.0, 1.0, 1.0), 0.05, 340.0,
        );
        let rel_err = (result.force.z - expected).abs() / expected;
        assert!(rel_err < 1e-10);
    }

    #[test]
    fn axial_drag_direction() {
        let dragels = vec![DragElement::constant(Vec3::ZERO, 1.0, 1.0)];
        let result = compute_aero_forces(
            &[], &[], &dragels, Vec3::new(0.0, 0.0, -100.0), 1.225, Vec3::ZERO,
            Vec3::new(1.0, 1.0, 1.0), 1000.0, Vec3::new(1.0, 1.0, 1.0),
            Vec3::new(1.0, 1.0, 1.0), 0.05, 340.0,
        );
        assert!(result.force.z > 0.0);
        assert!(result.force.x.abs() < 1e-6);
        assert!(result.force.y.abs() < 1e-6);
    }

    #[test]
    fn cd_mach_changes_drag_magnitude() {
        let table = vec![(0.0, 0.3), (1.0, 0.3), (1.2, 1.0), (2.0, 0.5)];
        let de = DragElement::constant(Vec3::ZERO, 0.3, 10.0).with_cd_mach(table);
        let rho = 1.225;
        let v = 200.0;
        let airvel = Vec3::new(0.0, 0.0, -v);
        let r_lo = compute_aero_forces(
            &[], &[], &[de.clone()], airvel, rho, Vec3::ZERO,
            Vec3::new(1.0, 1.0, 1.0), 1000.0, Vec3::new(1.0, 1.0, 1.0),
            Vec3::new(1.0, 1.0, 1.0), 0.05, 400.0,
        );
        let r_hi = compute_aero_forces(
            &[], &[], &[de], airvel, rho, Vec3::ZERO,
            Vec3::new(1.0, 1.0, 1.0), 1000.0, Vec3::new(1.0, 1.0, 1.0),
            Vec3::new(1.0, 1.0, 1.0), 0.05, 200.0 / 1.2,
        );
        assert!((r_lo.mach - 0.5).abs() < 1e-9, "Ma low = {}", r_lo.mach);
        assert!((r_hi.mach - 1.2).abs() < 1e-9, "Ma high = {}", r_hi.mach);
        assert!(
            (r_hi.drag_force - r_lo.drag_force).abs() > 100.0,
            "same q, different Cd(M): {} vs {}",
            r_hi.drag_force, r_lo.drag_force
        );
    }

    #[test]
    fn lift_perpendicular_to_drag() {
        let airfoils = vec![Airfoil {
            ref_pos: Vec3::ZERO,
            orientation: AirfoilOrientation::LiftVertical,
            chord: 1.0,
            area: 1.0,
            aspect_ratio: 1.0,
            coeffs: AirfoilCoeffs::Constant { cl: 1.0, cm: 0.0, cd: 0.5 },
        }];
        let airvel = Vec3::new(0.0, -50.0, 100.0);
        let result = compute_aero_forces(
            &airfoils, &[], &[], airvel, 1.225, Vec3::ZERO,
            Vec3::new(1.0, 1.0, 1.0), 1000.0, Vec3::new(1.0, 1.0, 1.0),
            Vec3::new(1.0, 1.0, 1.0), 0.05, 340.0,
        );
        let ddir = airvel * (-1.0 / airvel.length());
        let ldir_raw = Vec3::new(0.0, airvel.z, -airvel.y);
        let ldir = ldir_raw * (1.0 / ldir_raw.length());
        assert!(dot(ddir, ldir).abs() < 1e-9);
        assert!(result.force.length() > 1e-3);
    }

    #[test]
    fn control_surface_deflection_produces_lift() {
        let ctrlsurfs = vec![ControlSurface {
            ctrl_type: CtrlType::Elevator,
            ref_pos: Vec3::ZERO,
            axis: CtrlAxis::YPos,
            area: 1.0,
            d_cl: 2.0,
            level: 0.5,
        }];
        let result = compute_aero_forces(
            &[], &ctrlsurfs, &[], Vec3::new(0.0, 0.0, 100.0), 1.225, Vec3::ZERO,
            Vec3::new(1.0, 1.0, 1.0), 1000.0, Vec3::new(1.0, 1.0, 1.0),
            Vec3::new(1.0, 1.0, 1.0), 0.05, 340.0,
        );
        assert!(result.force.length() > 1e-3);
    }

    #[test]
    fn aero_damping_reduces_omega() {
        let omega = Vec3::new(1.0, 0.0, 0.0);
        let result = compute_aero_forces(
            &[], &[], &[], Vec3::new(0.0, 0.0, 100.0), 1.225, omega,
            Vec3::new(10.0, 1.0, 10.0), 1000.0, Vec3::new(1.0, 1.0, 1.0),
            Vec3::new(1.0, 1.0, 1.0), 0.05, 340.0,
        );
        assert!(result.torque.x < 0.0);
    }

    #[test]
    fn airfoil_table_interpolation() {
        let table = AirfoilCoeffs::Table(vec![
            (0.0, [0.0, 0.0, 0.1]),
            (PI05, [1.0, 0.0, 0.2]),
        ]);
        let (cl, cm, cd) = table.evaluate(PI025);
        assert!((cl - 0.5).abs() < 1e-10);
        assert!(cm.abs() < 1e-10);
        assert!((cd - 0.15).abs() < 1e-10);
    }

    #[test]
    fn airfoil_linear_lift() {
        let coeffs = AirfoilCoeffs::LinearLift {
            cl_alpha: 2.0 * PI,
            cl0: 0.0,
            cd0: 0.02,
        };
        let (cl, _, cd) = coeffs.evaluate(0.1);
        assert!((cl - 2.0 * PI * 0.1).abs() < 1e-10);
        assert!((cd - 0.02).abs() < 1e-10);
    }
}
