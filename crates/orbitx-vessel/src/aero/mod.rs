//! 气动力模型：空气翼面、控制面、变阻力元件、大气模型。
//!
//! 移植自 Orbiter `Vessel.cpp:4099-4226`（`UpdateAerodynamicForces`）。

mod us76;

#[cfg(test)]
mod tests;

pub use us76::UsStd1976Atmosphere;

use orbitx_math::{cross, dot, tmul, Matrix3, Vec3};
use std::sync::Arc;

// ── 大气模型 ────────────────────────────────────────────────────────

/// 大气模型接口。
pub trait Atmosphere: Send + Sync {
    /// 大气密度 [kg/m³]。
    fn density(&self, altitude: f64) -> f64;
    /// 大气压力 [Pa]。
    fn pressure(&self, altitude: f64) -> f64;
    /// 大气温度 [K]。
    fn temperature(&self, altitude: f64) -> f64;
    /// 比气体常数 [J/(kg·K)]。
    fn gas_constant(&self) -> f64 {
        287.058
    }
    /// 比热比 γ。
    fn gamma(&self) -> f64 {
        1.4
    }
    /// 当地声速 [m/s]：`a = √(γ R T)`。
    fn sound_speed(&self, altitude: f64) -> f64 {
        let t = self.temperature(altitude).max(1.0);
        (self.gamma() * self.gas_constant() * t).sqrt()
    }
    /// 返回一个密度闭包 `altitude → ρ`，可跨线程共享。
    fn density_fn(&self) -> Arc<dyn Fn(f64) -> f64 + Send + Sync>;
}

/// 指数衰减大气（Orbiter 默认简化模型）。
#[derive(Clone, Debug)]
pub struct ExponentialAtmosphere {
    pub rho0: f64,
    pub scale_height: f64,
    pub base_alt: f64,
    pub gas_constant: f64,
    pub gamma: f64,
    pub temperature0: f64,
}

impl ExponentialAtmosphere {
    pub fn earth() -> Self {
        Self {
            rho0: 1.225,
            scale_height: 8500.0,
            base_alt: 0.0,
            gas_constant: 287.058,
            gamma: 1.4,
            temperature0: 288.15,
        }
    }

    /// 从 TOML 大气配置构造。
    pub fn from_config(cfg: &orbitx_config::AtmosphereConfig) -> Self {
        Self {
            rho0: cfg.density0,
            scale_height: cfg.scale_height,
            base_alt: 0.0,
            gas_constant: cfg.gas_constant,
            gamma: cfg.gamma,
            temperature0: if cfg.density0 > 1e-12 {
                cfg.pressure0 / (cfg.density0 * cfg.gas_constant)
            } else {
                288.15
            },
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
        self.density(altitude) * self.gas_constant * self.temperature0
    }

    fn temperature(&self, altitude: f64) -> f64 {
        let _ = altitude;
        self.temperature0
    }

    fn gas_constant(&self) -> f64 {
        self.gas_constant
    }

    fn gamma(&self) -> f64 {
        self.gamma
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

/// 由 `BodyConfig` / `AtmosphereConfig` 构造大气；`None` 或 model=none 返回 `None`。
pub fn atmosphere_from_config(
    cfg: Option<&orbitx_config::AtmosphereConfig>,
) -> Option<Box<dyn Atmosphere>> {
    let cfg = cfg?;
    match cfg.model {
        orbitx_config::AtmosphereModel::None => None,
        orbitx_config::AtmosphereModel::Us76 => {
            Some(Box::new(UsStd1976Atmosphere::with_limits(
                cfg.alt_limit,
                cfg.gas_constant,
                cfg.gamma,
            )))
        }
        orbitx_config::AtmosphereModel::Exponential => {
            Some(Box::new(ExponentialAtmosphere::from_config(cfg)))
        }
    }
}

// ── Cd(M) 插值 ──────────────────────────────────────────────────────

/// 线性插值 Cd(M) 表；表空则返回 `fallback`。
pub fn interpolate_cd_mach(table: &[(f64, f64)], mach: f64, fallback: f64) -> f64 {
    if table.is_empty() {
        return fallback;
    }
    if table.len() == 1 {
        return table[0].1;
    }
    let mut lo = 0usize;
    let mut hi = table.len() - 1;
    if mach <= table[lo].0 {
        return table[lo].1;
    }
    if mach >= table[hi].0 {
        return table[hi].1;
    }
    while hi - lo > 1 {
        let mid = lo + (hi - lo) / 2;
        if table[mid].0 <= mach {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let (m0, c0) = table[lo];
    let (m1, c1) = table[hi];
    let t = (mach - m0) / (m1 - m0);
    c0 + t * (c1 - c0)
}

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
    Table(Vec<(f64, f64, f64, f64)>),
}

impl AirfoilCoeffs {
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
            Some(table) => interpolate_cd_mach(table, mach, self.cd),
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
