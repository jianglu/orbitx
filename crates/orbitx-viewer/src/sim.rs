//! 模拟状态：加载 VSOP87 历表，驱动时间推进，提供天体位置。
//!
//! 从 Orbiter 源码目录读取 `.dat` 文件（路径通过环境变量或默认路径配置）。

use orbitx_ephemeris::{ElpModel, Series, VsopModel};
use std::io::BufReader;
use std::path::PathBuf;

use crate::bridge::AU_METERS;

/// J2000 历元的 MJD。
pub const MJD2000: f64 = 51_544.5;

/// 行星定义：名称、VSOP87 数据文件名、渲染颜色、渲染半径。
struct BodyDef {
    name: &'static str,
    dat_file: &'static str,
    color: [f32; 4],
    /// 物理半径（米），用于显示。
    radius_m: f64,
    /// 渲染时的最小显示半径（kiss3d 单位），避免太远看不见。
    min_render_radius: f32,
}

/// 太阳系天体定义表。
const BODIES: &[BodyDef] = &[
    BodyDef {
        name: "Sun",
        dat_file: "Vsop87E_sun.dat",
        color: [1.0, 0.9, 0.3, 1.0],
        radius_m: 6.96e8,
        min_render_radius: 1.0,
    },
    BodyDef {
        name: "Mercury",
        dat_file: "Vsop87B_mer.dat",
        color: [0.6, 0.5, 0.4, 1.0],
        radius_m: 2.44e6,
        min_render_radius: 0.15,
    },
    BodyDef {
        name: "Venus",
        dat_file: "Vsop87B_ven.dat",
        color: [0.9, 0.8, 0.5, 1.0],
        radius_m: 6.05e6,
        min_render_radius: 0.2,
    },
    BodyDef {
        name: "Earth",
        dat_file: "Vsop87B_ear.dat",
        color: [0.2, 0.4, 0.9, 1.0],
        radius_m: 6.37e6,
        min_render_radius: 0.22,
    },
    BodyDef {
        name: "Mars",
        dat_file: "Vsop87B_mar.dat",
        color: [0.8, 0.2, 0.1, 1.0],
        radius_m: 3.39e6,
        min_render_radius: 0.18,
    },
    BodyDef {
        name: "Jupiter",
        dat_file: "Vsop87B_jup.dat",
        color: [0.9, 0.7, 0.4, 1.0],
        radius_m: 6.99e7,
        min_render_radius: 0.5,
    },
    BodyDef {
        name: "Saturn",
        dat_file: "Vsop87B_sat.dat",
        color: [0.9, 0.85, 0.6, 1.0],
        radius_m: 5.82e7,
        min_render_radius: 0.45,
    },
    BodyDef {
        name: "Uranus",
        dat_file: "Vsop87B_ura.dat",
        color: [0.4, 0.7, 0.9, 1.0],
        radius_m: 2.54e7,
        min_render_radius: 0.3,
    },
    BodyDef {
        name: "Neptune",
        dat_file: "Vsop87B_nep.dat",
        color: [0.2, 0.3, 0.9, 1.0],
        radius_m: 2.46e7,
        min_render_radius: 0.3,
    },
];

/// 天体在某一时刻的状态。
pub struct BodyState {
    pub name: &'static str,
    /// 位置（米，左手系），相对于太阳系质心。
    pub pos: [f64; 3],
    pub color: [f32; 4],
    pub radius_m: f64,
    pub min_render_radius: f32,
}

/// 模拟器：加载历表数据，推进时间，查询天体状态。
pub struct Simulation {
    /// VSOP87 模型列表（Sun 用 series E，其余用 series B）。
    vsop_models: Vec<Option<VsopModel>>,
    /// 月球 ELP82 模型。
    elp_moon: Option<ElpModel>,
    /// 当前模拟时间（MJD）。
    pub mjd: f64,
    /// 时间加速倍率（1.0 = 实时）。
    pub time_scale: f64,
}

impl Simulation {
    /// 加载所有历表数据。
    ///
    /// 从 Orbiter 源码目录 `Src/Celbody/Vsop87/Data/` 读取 `.dat` 文件。
    pub fn new() -> Self {
        let vsop_dir = find_vsop_dir();
        let mut vsop_models: Vec<Option<VsopModel>> = Vec::with_capacity(BODIES.len());

        for def in BODIES {
            let path = vsop_dir.join(def.dat_file);
            match std::fs::File::open(&path) {
                Ok(file) => {
                    let series = if def.name == "Sun" {
                        Series::E
                    } else {
                        Series::B
                    };
                    let a0 = 1.0;
                    let reader = BufReader::new(file);
                    match VsopModel::from_reader(reader, series, a0, 1e-6, 10.0) {
                        Ok(model) => {
                            eprintln!("已加载: {} ({} 项)", def.name, def.dat_file);
                            vsop_models.push(Some(model));
                        }
                        Err(e) => {
                            eprintln!("警告: 解析 {} 失败: {e}", def.dat_file);
                            vsop_models.push(None);
                        }
                    }
                }
                Err(e) => {
                    eprintln!("警告: 无法读取 {}: {e}", path.display());
                    vsop_models.push(None);
                }
            }
        }

        // 加载月球 ELP82。
        let elp_moon = {
            let elp_path = find_elp_path();
            match std::fs::File::open(&elp_path) {
                Ok(file) => match ElpModel::from_reader(BufReader::new(file), 1e-6) {
                    Ok(model) => {
                        eprintln!("已加载: Moon (ELP82)");
                        Some(model)
                    }
                    Err(e) => {
                        eprintln!("警告: 解析 ELP82 失败: {e}");
                        None
                    }
                },
                Err(e) => {
                    eprintln!("警告: 无法读取 {}: {e}", elp_path.display());
                    None
                }
            }
        };

        // 验证至少加载了地球。
        let earth_idx = BODIES.iter().position(|b| b.name == "Earth").unwrap();
        if vsop_models[earth_idx].is_none() {
            eprintln!("错误: 无法加载地球历表数据。请确认 Orbiter 源码路径。");
        }

        Simulation {
            vsop_models,
            elp_moon,
            mjd: MJD2000,
            time_scale: 86400.0 * 10.0, // 默认 10 天/秒
        }
    }

    /// 推进模拟时间。
    ///
    /// `dt` 是真实时间步长（秒）。
    pub fn step(&mut self, dt: f64) {
        self.mjd += dt * self.time_scale / 86400.0;
    }

    /// 获取所有天体的当前状态。
    ///
    /// VSOP87 series B 返回极坐标（经度、纬度、半径 AU）。
    /// series E 返回直角坐标（米）。
    /// 这里统一转为直角坐标（米，左手系）。
    pub fn body_states(&self) -> Vec<BodyState> {
        let mut states = Vec::with_capacity(BODIES.len() + 1); // +1 for Moon

        for (i, def) in BODIES.iter().enumerate() {
            if let Some(model) = &self.vsop_models[i] {
                let ret = model.eval(self.mjd);
                let pos = if model.series.is_polar() {
                    // series B: ret[0]=经度(rad), ret[1]=纬度(rad), ret[2]=半径(AU)
                    polar_to_cartesian(ret[0], ret[1], ret[2])
                } else {
                    // series E: ret[0..2] = x,y,z (米)
                    [ret[0], ret[1], ret[2]]
                };
                states.push(BodyState {
                    name: def.name,
                    pos,
                    color: def.color,
                    radius_m: def.radius_m,
                    min_render_radius: def.min_render_radius,
                });
            }
        }

        // 月球：相对地球的位置。
        if let Some(elp) = &self.elp_moon {
            let moon_pos = elp.eval(self.mjd);
            // 找地球位置，加上月球偏移。
            let earth_idx = states.iter().position(|s| s.name == "Earth");
            if let Some(ei) = earth_idx {
                let earth_pos = states[ei].pos;
                states.push(BodyState {
                    name: "Moon",
                    pos: [
                        earth_pos[0] + moon_pos[0],
                        earth_pos[1] + moon_pos[1],
                        earth_pos[2] + moon_pos[2],
                    ],
                    color: [0.7, 0.7, 0.7, 1.0],
                    radius_m: 1.74e6,
                    min_render_radius: 0.08,
                });
            }
        }

        states
    }

    /// 采样某个天体的轨道（用于绘制轨道线）。
    ///
    /// 返回 N 个位置点（米，左手系）。
    pub fn sample_orbit(&self, body_idx: usize, n_samples: usize) -> Vec<[f64; 3]> {
        let model = match &self.vsop_models[body_idx] {
            Some(m) => m,
            None => return Vec::new(),
        };

        let def = &BODIES[body_idx];
        // 采样一个完整轨道周期。用近似的公转周期。
        let period_days = approximate_period(def.name);
        let mut points = Vec::with_capacity(n_samples);

        for i in 0..n_samples {
            let frac = i as f64 / n_samples as f64;
            let sample_mjd = self.mjd + frac * period_days;
            let ret = model.eval(sample_mjd);
            let pos = if model.series.is_polar() {
                polar_to_cartesian(ret[0], ret[1], ret[2])
            } else {
                [ret[0], ret[1], ret[2]]
            };
            points.push(pos);
        }

        points
    }

    /// 天体定义数量（不含月球）。
    pub fn num_bodies(&self) -> usize {
        BODIES.len()
    }

    /// 获取天体定义。
    pub fn body_name(&self, idx: usize) -> &'static str {
        BODIES[idx].name
    }
}

impl Default for Simulation {
    fn default() -> Self {
        Self::new()
    }
}

/// 极坐标（经度、纬度、半径 AU）→ 直角坐标（米，左手系）。
///
/// VSOP87 series B 约定：
/// - 经度 l = 在黄道面内的角度（从 x 轴起）
/// - 纬度 b = 黄纬（从黄道面起）
/// - 半径 r = 到太阳的距离（AU）
///
/// 直角坐标（左手系）：
/// - x = r*cos(b)*cos(l)
/// - y = r*sin(b)（黄道北极方向）
/// - z = r*cos(b)*sin(l)
fn polar_to_cartesian(l: f64, b: f64, r_au: f64) -> [f64; 3] {
    let r = r_au * AU_METERS;
    let cosb = b.cos();
    let cosl = l.cos();
    let sinl = l.sin();
    let sinb = b.sin();
    [r * cosb * cosl, r * sinb, r * cosb * sinl]
}

/// 近似公转周期（天）。
fn approximate_period(name: &str) -> f64 {
    match name {
        "Mercury" => 88.0,
        "Venus" => 225.0,
        "Earth" => 365.25,
        "Mars" => 687.0,
        "Jupiter" => 4333.0,
        "Saturn" => 10_759.0,
        "Uranus" => 30_687.0,
        "Neptune" => 60_190.0,
        _ => 365.25,
    }
}

/// 查找 Orbiter 源码中的 VSOP87 数据目录。
fn find_vsop_dir() -> PathBuf {
    // 尝试从环境变量获取。
    if let Ok(path) = std::env::var("ORBITER_SRC") {
        let p = PathBuf::from(path)
            .join("Src")
            .join("Celbody")
            .join("Vsop87")
            .join("Data");
        if p.exists() {
            return p;
        }
    }

    // 默认：orbitx 的兄弟目录 orbiter。
    let manifest = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    manifest
        .ancestors()
        .nth(2)
        .unwrap()
        .parent()
        .unwrap()
        .join("orbiter")
        .join("Src")
        .join("Celbody")
        .join("Vsop87")
        .join("Data")
}

/// 查找 ELP82 数据文件路径。
fn find_elp_path() -> PathBuf {
    if let Ok(path) = std::env::var("ORBITER_SRC") {
        let p = PathBuf::from(path)
            .join("Src")
            .join("Celbody")
            .join("Moon")
            .join("Config")
            .join("Moon")
            .join("Data")
            .join("ELP82.dat");
        if p.exists() {
            return p;
        }
    }

    let manifest = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    manifest
        .ancestors()
        .nth(2)
        .unwrap()
        .parent()
        .unwrap()
        .join("orbiter")
        .join("Src")
        .join("Celbody")
        .join("Moon")
        .join("Config")
        .join("Moon")
        .join("Data")
        .join("ELP82.dat")
}
