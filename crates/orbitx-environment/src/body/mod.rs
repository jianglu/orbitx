//! 单颗天体：位置、自转开关、历表、重力模型。

use std::io::BufReader;
use std::path::Path;
use std::sync::Arc;

use orbitx_config::{AtmosphereConfig, EphemerisConfig, GravityConfig, RotationConfig};
use orbitx_dynamics::PinesModel;
use orbitx_dynamics::RotationState;
use orbitx_math::consts::GGRAV;
use orbitx_math::mat3::Matrix3;
use orbitx_math::vec3::Vec3;

use crate::ephem_path::{
    find_elp_path, find_galsat_path, find_gravity_model_path, find_tass_path, find_vsop_path,
};

/// 已加载的历表。位置相对父体；无父体时为日心/绝对。
pub enum EphemerisModel {
    Vsop87(orbitx_ephemeris::VsopModel),
    Elp82(orbitx_ephemeris::ElpModel),
    Galsat {
        model: orbitx_ephemeris::GalModel,
        index: usize,
    },
    Tass17 {
        model: orbitx_ephemeris::TasModel,
        index: usize,
    },
}

impl EphemerisModel {
    /// `[x, y, z, vx, vy, vz]`，米与米/秒，相对父体。
    pub fn eval(&mut self, mjd: f64) -> [f64; 6] {
        match self {
            EphemerisModel::Vsop87(model) => {
                let ret = model.eval(mjd);
                if model.series.is_polar() {
                    polar_to_cartesian(ret[0], ret[1], ret[2])
                } else {
                    ret
                }
            }
            EphemerisModel::Elp82(model) => model.eval(mjd),
            EphemerisModel::Galsat { model, index } => {
                let jd = mjd + 2_400_000.5;
                model.eval(jd, *index as i32)
            }
            EphemerisModel::Tass17 { model, index } => {
                let jd = mjd + 2_400_000.5;
                model.eval(jd, *index)
            }
        }
    }
}

fn polar_to_cartesian(l: f64, b: f64, r_au: f64) -> [f64; 6] {
    let [x, y, z] = orbitx_math::polar_to_cartesian(l, b, r_au);
    [x, y, z, 0.0, 0.0, 0.0]
}

/// 已加载的重力模型。
pub enum GravityModel {
    PointMass,
    Jcoeff { values: Vec<f64> },
    Pines { model: Arc<PinesModel>, cutoff: usize },
}

/// 场景里的一颗天体。
pub struct CelestialBody {
    pub name: String,
    pub mass: f64,
    pub size: f64,
    pub pos: Vec3,
    pub parent_idx: Option<usize>,
    /// 进入 `GravBody`。假太阳为 false。
    pub dynamics: bool,
    pub ephemeris: Option<EphemerisModel>,
    /// 无历表时的位置。有父体则相对父体。
    pub fixed_pos: Option<Vec3>,
    pub rotation: Option<RotationState>,
    /// `false`：保留初值转角，不推进。
    pub rotation_enabled: bool,
    pub gravity: GravityModel,
    /// 仅 `dynamics` 天体携带，供主天体刷新大气。
    pub atmosphere: Option<AtmosphereConfig>,
    pub color: [f32; 4],
    pub radius_m: f64,
    pub min_render_radius: f32,
}

impl CelestialBody {
    pub fn gm(&self) -> f64 {
        GGRAV * self.mass
    }

    pub fn rot_matrix(&self) -> Matrix3 {
        match &self.rotation {
            Some(r) => *r.rot_matrix(),
            None => Matrix3::IDENTITY,
        }
    }

    /// 关闭自转时返回 0，地面风速为 0。
    pub fn sid_rot_period(&self) -> f64 {
        if !self.rotation_enabled {
            return 0.0;
        }
        self.rotation
            .as_ref()
            .map(|r| r.sid_rot_period())
            .unwrap_or(0.0)
    }
}

pub(crate) fn load_body(
    cfg_name: &str,
    mass: f64,
    size: f64,
    dynamics: bool,
    fixed_pos: Option<[f64; 3]>,
    ephemeris: &Option<EphemerisConfig>,
    rotation: &Option<RotationConfig>,
    gravity: &Option<GravityConfig>,
    atmosphere: &Option<AtmosphereConfig>,
    color: [f32; 4],
    min_render_radius: f32,
    parent_idx: Option<usize>,
    data_root: &Path,
) -> Result<CelestialBody, String> {
    let ephemeris = load_ephemeris(ephemeris, data_root)?;
    let rotation_enabled = rotation.as_ref().is_some_and(|r| r.enabled);
    let rotation = rotation.as_ref().map(RotationState::from_config);
    let gravity = load_gravity(gravity, data_root)?;
    let pos = match fixed_pos {
        Some([x, y, z]) => Vec3::new(x, y, z),
        None => Vec3::ZERO,
    };
    Ok(CelestialBody {
        name: cfg_name.to_string(),
        mass,
        size,
        pos,
        parent_idx,
        dynamics,
        ephemeris,
        fixed_pos: fixed_pos.map(|p| Vec3::new(p[0], p[1], p[2])),
        rotation,
        rotation_enabled,
        gravity,
        atmosphere: if dynamics {
            atmosphere.clone()
        } else {
            None
        },
        color,
        radius_m: size,
        min_render_radius,
    })
}

fn load_ephemeris(
    cfg: &Option<EphemerisConfig>,
    orbiter_src: &Path,
) -> Result<Option<EphemerisModel>, String> {
    match cfg {
        None => Ok(None),
        Some(EphemerisConfig::Vsop87 {
            dat_file,
            series,
            a0,
            prec,
            interval,
        }) => {
            let path = find_vsop_path(orbiter_src, dat_file);
            let file = std::fs::File::open(&path)
                .map_err(|e| format!("无法读取 {}: {e}", path.display()))?;
            let s = if series == "E" {
                orbitx_ephemeris::Series::E
            } else {
                orbitx_ephemeris::Series::B
            };
            let model = orbitx_ephemeris::VsopModel::from_reader(
                BufReader::new(file),
                s,
                *a0,
                *prec,
                *interval,
            )
            .map_err(|e| format!("解析 {dat_file} 失败: {e}"))?;
            Ok(Some(EphemerisModel::Vsop87(model)))
        }
        Some(EphemerisConfig::Elp82 { dat_file, prec }) => {
            let path = find_elp_path(orbiter_src, dat_file);
            let file = std::fs::File::open(&path)
                .map_err(|e| format!("无法读取 {}: {e}", path.display()))?;
            let model = orbitx_ephemeris::ElpModel::from_reader(BufReader::new(file), *prec)
                .map_err(|e| format!("解析 {dat_file} 失败: {e}"))?;
            Ok(Some(EphemerisModel::Elp82(model)))
        }
        Some(EphemerisConfig::Galsat { dat_file, index }) => {
            let path = find_galsat_path(orbiter_src, dat_file);
            let file = std::fs::File::open(&path)
                .map_err(|e| format!("无法读取 {}: {e}", path.display()))?;
            let model = orbitx_ephemeris::GalModel::from_reader(BufReader::new(file))
                .map_err(|e| format!("解析 {dat_file} 失败: {e}"))?;
            Ok(Some(EphemerisModel::Galsat {
                model,
                index: *index,
            }))
        }
        Some(EphemerisConfig::Tass17 { dat_file, index }) => {
            let path = find_tass_path(orbiter_src, dat_file);
            let file = std::fs::File::open(&path)
                .map_err(|e| format!("无法读取 {}: {e}", path.display()))?;
            let model = orbitx_ephemeris::TasModel::from_reader(BufReader::new(file))
                .map_err(|e| format!("解析 {dat_file} 失败: {e}"))?;
            Ok(Some(EphemerisModel::Tass17 {
                model,
                index: *index,
            }))
        }
    }
}

fn load_gravity(cfg: &Option<GravityConfig>, orbiter_src: &Path) -> Result<GravityModel, String> {
    match cfg {
        None => Ok(GravityModel::PointMass),
        Some(GravityConfig::Jcoeff { values }) => Ok(GravityModel::Jcoeff {
            values: values.clone(),
        }),
        Some(GravityConfig::Pines {
            model_path,
            cutoff,
        }) => {
            let path = find_gravity_model_path(orbiter_src, model_path);
            match std::fs::File::open(&path) {
                Ok(file) => match PinesModel::from_reader(BufReader::new(file), *cutoff) {
                    Ok(model) => Ok(GravityModel::Pines {
                        model: Arc::new(model),
                        cutoff: *cutoff,
                    }),
                    Err(e) => Err(format!("解析重力模型 {model_path} 失败: {e}")),
                },
                Err(_) => {
                    eprintln!(
                        "Note: gravity model {} not found, falling back to point mass",
                        path.display()
                    );
                    Ok(GravityModel::PointMass)
                }
            }
        }
    }
}

#[cfg(test)]
mod tests;
