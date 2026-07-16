//! 天体配置（body.toml）。
//!
//! 对应 Orbiter 的 Planet.cfg + 大气参数。
//! 定义天体的物理参数：质量、半径、历表、自转/姿态、重力模型、大气。

use serde::{Deserialize, Serialize};

/// 天体配置。
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct BodyConfig {
    /// 天体名称。
    pub name: String,
    /// 质量 [kg]。
    pub mass: f64,
    /// 平均半径 [m]。
    pub size: f64,

    /// 历表配置。
    #[serde(default)]
    pub ephemeris: Option<EphemerisConfig>,

    /// 自转/姿态配置。
    #[serde(default)]
    pub rotation: Option<RotationConfig>,

    /// 重力模型配置。
    #[serde(default)]
    pub gravity: Option<GravityConfig>,

    /// 大气配置。
    #[serde(default)]
    pub atmosphere: Option<AtmosphereConfig>,

    /// 渲染颜色 [r, g, b, a]。
    #[serde(default)]
    pub color: [f32; 4],

    /// 渲染时的最小显示半径。
    #[serde(default = "default_min_render_radius")]
    pub min_render_radius: f32,
}

fn default_min_render_radius() -> f32 {
    0.2
}

/// 历表配置。
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum EphemerisConfig {
    /// VSOP87 行星历表（Mercury–Neptune, Sun）。
    Vsop87 {
        /// 数据文件名（如 "Vsop87B_ear.dat"）。
        dat_file: String,
        /// 级数类型："B"（极坐标）或 "E"（直角坐标）。
        series: String,
        /// 半长轴 [AU]。
        #[serde(default = "default_a0")]
        a0: f64,
        /// 精度截断。
        #[serde(default = "default_prec")]
        prec: f64,
        /// 采样间隔 [s]（快速历表插值用）。
        #[serde(default = "default_interval")]
        interval: f64,
    },
    /// ELP2000-82 月球历表。
    Elp82 {
        /// 数据文件名（如 "ELP82.dat"）。
        dat_file: String,
        /// 精度截断。
        #[serde(default = "default_prec")]
        prec: f64,
    },
    /// GALSAT 木星伽利略卫星历表。
    Galsat {
        /// 数据文件名（如 "ephem_e15.dat"）。
        dat_file: String,
        /// 卫星索引：1=Io, 2=Europa, 3=Ganymede, 4=Callisto。
        index: usize,
    },
    /// TASS17 土星卫星历表。
    Tass17 {
        /// 数据文件名（如 "tass17.dat"）。
        dat_file: String,
        /// 卫星索引：0–7。
        index: usize,
    },
}

fn default_a0() -> f64 {
    1.0
}
fn default_prec() -> f64 {
    1e-6
}
fn default_interval() -> f64 {
    10.0
}

/// 自转/姿态配置。
///
/// 对应 Orbiter Planet.cfg 中的 Rotation and precession parameters。
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RotationConfig {
    /// 恒星自转周期 [s]。
    pub sid_rot_period: f64,
    /// 自转偏移 [rad]（t=0 时的初始旋转角）。
    pub sid_rot_offset: f64,
    /// 赤道倾斜角（黄赤交角）[rad]。
    pub obliquity: f64,
    /// 升交点经度 [rad]。
    pub lan: f64,
    /// 升交点经度参考 MJD。
    pub lan_mjd: f64,
    /// 岁差周期 [days]（0 = 无岁差）。
    #[serde(default)]
    pub precession_period: f64,
    /// 岁差倾斜角 [rad]。
    #[serde(default)]
    pub precession_obliquity: f64,
    /// 岁差升交点经度 [rad]。
    #[serde(default)]
    pub precession_lan: f64,
}

/// 重力模型配置。
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum GravityConfig {
    /// J 系数（J2, J3, J4, ...）带状谐函数。
    Jcoeff {
        /// J 系数值（jcoeff[0] = J2）。
        values: Vec<f64>,
    },
    /// Pines 球谐重力模型。
    Pines {
        /// 重力模型文件路径（如 "egm96_to360.tab"）。
        model_path: String,
        /// 最大阶/次截断。
        cutoff: usize,
    },
}

/// 大气配置。
///
/// 对应 Orbiter Planet.cfg 中的 Atmospheric Parameters。
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AtmosphereConfig {
    /// 海平面密度 [kg/m³]。
    pub density0: f64,
    /// 标高 [m]。
    pub scale_height: f64,
    /// 海平面气压 [Pa]。
    pub pressure0: f64,
    /// 比气体常数 [J/(K kg)]。
    pub gas_constant: f64,
    /// 比热比 c_p/c_v。
    pub gamma: f64,
    /// 截止高度 [m]。
    #[serde(default = "default_alt_limit")]
    pub alt_limit: f64,
}

fn default_alt_limit() -> f64 {
    200e3
}

impl BodyConfig {
    /// 从 TOML 字符串解析。
    pub fn from_toml_str(s: &str) -> Result<Self, toml::de::Error> {
        toml::from_str(s)
    }

    /// 序列化为 TOML 字符串。
    pub fn to_toml_string(&self) -> Result<String, toml::ser::Error> {
        toml::to_string_pretty(self)
    }

    // ─── 内置默认配置（与 Orbiter Planet.cfg 数值一致）───

    /// 太阳。
    pub fn sun() -> Self {
        Self {
            name: "Sun".to_string(),
            mass: 1.9885e30,
            size: 6.96e8,
            ephemeris: Some(EphemerisConfig::Vsop87 {
                dat_file: "Vsop87E_sun.dat".to_string(),
                series: "E".to_string(),
                a0: 1.0,
                prec: 1e-6,
                interval: 10.0,
            }),
            rotation: None,
            gravity: None,
            atmosphere: None,
            color: [1.0, 0.95, 0.4, 1.0],
            min_render_radius: 1.2,
        }
    }

    /// 水星。
    pub fn mercury() -> Self {
        Self {
            name: "Mercury".to_string(),
            mass: 3.3011e23,
            size: 2.44e6,
            ephemeris: Some(EphemerisConfig::Vsop87 {
                dat_file: "Vsop87B_mer.dat".to_string(),
                series: "B".to_string(),
                a0: 1.0,
                prec: 1e-6,
                interval: 10.0,
            }),
            rotation: None,
            gravity: None,
            atmosphere: None,
            color: [0.9, 0.8, 0.7, 1.0],
            min_render_radius: 0.2,
        }
    }

    /// 金星。
    pub fn venus() -> Self {
        Self {
            name: "Venus".to_string(),
            mass: 4.8675e24,
            size: 6.052e6,
            ephemeris: Some(EphemerisConfig::Vsop87 {
                dat_file: "Vsop87B_ven.dat".to_string(),
                series: "B".to_string(),
                a0: 1.0,
                prec: 1e-6,
                interval: 10.0,
            }),
            rotation: None,
            gravity: None,
            atmosphere: None,
            color: [1.0, 0.9, 0.6, 1.0],
            min_render_radius: 0.25,
        }
    }

    /// 地球。
    ///
    /// 数值来自 Orbiter `Earth.cfg`：Mass=5.973698968e+24, Size=6.37101e6。
    pub fn earth() -> Self {
        Self {
            name: "Earth".to_string(),
            mass: 5.973698968e24,
            size: 6.37101e6,
            ephemeris: Some(EphemerisConfig::Vsop87 {
                dat_file: "Vsop87B_ear.dat".to_string(),
                series: "B".to_string(),
                a0: 1.0,
                prec: 1e-8,
                interval: 79.0,
            }),
            rotation: Some(RotationConfig {
                sid_rot_period: 86164.10132,
                sid_rot_offset: 4.88948754,
                obliquity: 0.4090928023,
                lan: 0.0,
                lan_mjd: 51544.5,
                precession_period: -9413040.4,
                precession_obliquity: 0.0,
                precession_lan: 0.0,
            }),
            gravity: Some(GravityConfig::Pines {
                model_path: "egm96_to360.tab".to_string(),
                cutoff: 10,
            }),
            atmosphere: Some(AtmosphereConfig {
                density0: 1.293,
                scale_height: 8500.0,
                pressure0: 101.4e3,
                gas_constant: 286.91,
                gamma: 1.4,
                alt_limit: 200e3,
            }),
            color: [0.3, 0.6, 1.0, 1.0],
            min_render_radius: 0.27,
        }
    }

    /// 火星。
    pub fn mars() -> Self {
        Self {
            name: "Mars".to_string(),
            mass: 6.418542e23,
            size: 3.39e6,
            ephemeris: Some(EphemerisConfig::Vsop87 {
                dat_file: "Vsop87B_mar.dat".to_string(),
                series: "B".to_string(),
                a0: 1.0,
                prec: 1e-6,
                interval: 10.0,
            }),
            rotation: Some(RotationConfig {
                sid_rot_period: 88642.66435,
                sid_rot_offset: 5.469523488,
                obliquity: 0.4397415938,
                lan: 0.6210531483,
                lan_mjd: 51544.5,
                precession_period: -63346652.48,
                precession_obliquity: 0.03224369545,
                precession_lan: 4.005081124,
            }),
            gravity: Some(GravityConfig::Pines {
                model_path: "jgmro_120f_sha.tab".to_string(),
                cutoff: 10,
            }),
            atmosphere: None,
            color: [1.0, 0.4, 0.2, 1.0],
            min_render_radius: 0.22,
        }
    }

    /// 木星。
    pub fn jupiter() -> Self {
        Self {
            name: "Jupiter".to_string(),
            mass: 1.8986111e27,
            size: 6.9911e7,
            ephemeris: Some(EphemerisConfig::Vsop87 {
                dat_file: "Vsop87B_jup.dat".to_string(),
                series: "B".to_string(),
                a0: 1.0,
                prec: 1e-6,
                interval: 671.0,
            }),
            rotation: Some(RotationConfig {
                sid_rot_period: 13500.3,
                sid_rot_offset: 2.547801285,
                obliquity: 0.05443758224,
                lan: 3.782814532,
                lan_mjd: 51544.5,
                precession_period: -307703725.6,
                precession_obliquity: 0.02276340837,
                precession_lan: 4.89539507,
            }),
            gravity: Some(GravityConfig::Jcoeff {
                values: vec![0.01475], // J2 only
            }),
            atmosphere: None,
            color: [1.0, 0.85, 0.6, 1.0],
            min_render_radius: 0.6,
        }
    }

    /// 土星。
    pub fn saturn() -> Self {
        Self {
            name: "Saturn".to_string(),
            mass: 5.6832e26,
            size: 5.8232e7,
            ephemeris: Some(EphemerisConfig::Vsop87 {
                dat_file: "Vsop87B_sat.dat".to_string(),
                series: "B".to_string(),
                a0: 1.0,
                prec: 1e-6,
                interval: 10.0,
            }),
            rotation: None,
            gravity: None,
            atmosphere: None,
            color: [1.0, 0.95, 0.7, 1.0],
            min_render_radius: 0.55,
        }
    }

    /// 天王星。
    pub fn uranus() -> Self {
        Self {
            name: "Uranus".to_string(),
            mass: 8.681e25,
            size: 2.5362e7,
            ephemeris: Some(EphemerisConfig::Vsop87 {
                dat_file: "Vsop87B_ura.dat".to_string(),
                series: "B".to_string(),
                a0: 1.0,
                prec: 1e-6,
                interval: 10.0,
            }),
            rotation: None,
            gravity: None,
            atmosphere: None,
            color: [0.5, 0.9, 1.0, 1.0],
            min_render_radius: 0.4,
        }
    }

    /// 海王星。
    pub fn neptune() -> Self {
        Self {
            name: "Neptune".to_string(),
            mass: 1.024e26,
            size: 2.4526e7,
            ephemeris: Some(EphemerisConfig::Vsop87 {
                dat_file: "Vsop87B_nep.dat".to_string(),
                series: "B".to_string(),
                a0: 1.0,
                prec: 1e-6,
                interval: 10.0,
            }),
            rotation: None,
            gravity: None,
            atmosphere: None,
            color: [0.3, 0.5, 1.0, 1.0],
            min_render_radius: 0.4,
        }
    }

    /// 月球。
    ///
    /// 数值来自 Orbiter `Moon.cfg`：Mass=7.347673176382784e+22, Size=1.738e6。
    pub fn moon() -> Self {
        Self {
            name: "Moon".to_string(),
            mass: 7.347673176382784e22,
            size: 1.738e6,
            ephemeris: Some(EphemerisConfig::Elp82 {
                dat_file: "ELP82.dat".to_string(),
                prec: 1e-5,
            }),
            rotation: Some(RotationConfig {
                sid_rot_period: 2360588.15,
                sid_rot_offset: 4.769465382,
                obliquity: 0.02692416821,
                lan: 1.71817749,
                lan_mjd: 51544.5,
                precession_period: -6793.219721,
                precession_obliquity: 7.259562816e-5,
                precession_lan: 0.4643456618,
            }),
            gravity: Some(GravityConfig::Pines {
                model_path: "jgl165p1.sha".to_string(),
                cutoff: 10,
            }),
            atmosphere: None,
            color: [0.7, 0.7, 0.7, 1.0],
            min_render_radius: 0.08,
        }
    }

    /// Io（木卫一）。
    pub fn io() -> Self {
        Self {
            name: "Io".to_string(),
            mass: 8.9319e22,
            size: 1.8216e6,
            ephemeris: Some(EphemerisConfig::Galsat {
                dat_file: "ephem_e15.dat".to_string(),
                index: 1,
            }),
            rotation: None,
            gravity: None,
            atmosphere: None,
            color: [0.9, 0.8, 0.3, 1.0],
            min_render_radius: 0.06,
        }
    }

    /// Europa（木卫二）。
    pub fn europa() -> Self {
        Self {
            name: "Europa".to_string(),
            mass: 4.7998e22,
            size: 1.5608e6,
            ephemeris: Some(EphemerisConfig::Galsat {
                dat_file: "ephem_e15.dat".to_string(),
                index: 2,
            }),
            rotation: None,
            gravity: None,
            atmosphere: None,
            color: [0.8, 0.7, 0.5, 1.0],
            min_render_radius: 0.05,
        }
    }

    /// Ganymede（木卫三）。
    pub fn ganymede() -> Self {
        Self {
            name: "Ganymede".to_string(),
            mass: 1.4819e23,
            size: 2.6341e6,
            ephemeris: Some(EphemerisConfig::Galsat {
                dat_file: "ephem_e15.dat".to_string(),
                index: 3,
            }),
            rotation: None,
            gravity: None,
            atmosphere: None,
            color: [0.6, 0.6, 0.6, 1.0],
            min_render_radius: 0.07,
        }
    }

    /// Callisto（木卫四）。
    pub fn callisto() -> Self {
        Self {
            name: "Callisto".to_string(),
            mass: 1.0759e23,
            size: 2.4103e6,
            ephemeris: Some(EphemerisConfig::Galsat {
                dat_file: "ephem_e15.dat".to_string(),
                index: 4,
            }),
            rotation: None,
            gravity: None,
            atmosphere: None,
            color: [0.4, 0.4, 0.4, 1.0],
            min_render_radius: 0.06,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn earth_default_mass() {
        let earth = BodyConfig::earth();
        assert!(
            (earth.mass - 5.973698968e24).abs() / 5.973698968e24 < 1e-12,
            "Earth mass = {}",
            earth.mass
        );
    }

    #[test]
    fn earth_default_gravity() {
        let earth = BodyConfig::earth();
        match &earth.gravity {
            Some(GravityConfig::Pines {
                model_path,
                cutoff,
            }) => {
                assert_eq!(model_path, "egm96_to360.tab");
                assert_eq!(*cutoff, 10);
            }
            _ => panic!("Earth gravity should be Pines model"),
        }
    }

    #[test]
    fn earth_default_rotation() {
        let earth = BodyConfig::earth();
        let rot = earth.rotation.as_ref().expect("Earth should have rotation");
        assert!(
            (rot.sid_rot_period - 86164.10132).abs() < 1e-4,
            "sid_rot_period = {}",
            rot.sid_rot_period
        );
        assert!(
            (rot.obliquity - 0.4090928023).abs() < 1e-10,
            "obliquity = {}",
            rot.obliquity
        );
    }

    #[test]
    fn moon_default_obliquity() {
        let moon = BodyConfig::moon();
        let rot = moon.rotation.as_ref().expect("Moon should have rotation");
        assert!(
            (rot.obliquity - 0.02692416821).abs() < 1e-10,
            "Moon obliquity = {}",
            rot.obliquity
        );
    }

    #[test]
    fn jupiter_j2() {
        let jup = BodyConfig::jupiter();
        match &jup.gravity {
            Some(GravityConfig::Jcoeff { values }) => {
                assert_eq!(values.len(), 1);
                assert!((values[0] - 0.01475).abs() < 1e-10);
            }
            _ => panic!("Jupiter gravity should be Jcoeff"),
        }
    }

    #[test]
    fn toml_roundtrip() {
        let earth = BodyConfig::earth();
        let toml_str = earth.to_toml_string().unwrap();
        let parsed = BodyConfig::from_toml_str(&toml_str).unwrap();
        assert_eq!(parsed.name, "Earth");
        assert!((parsed.mass - earth.mass).abs() / earth.mass < 1e-10);
        assert!((parsed.size - earth.size).abs() / earth.size < 1e-10);
    }

    #[test]
    fn parse_toml_string() {
        let toml_str = r#"
name = "TestBody"
mass = 1.0e24
size = 6.4e6

[rotation]
sid_rot_period = 86400.0
sid_rot_offset = 0.0
obliquity = 0.4
lan = 0.0
lan_mjd = 51544.5

[gravity]
type = "Jcoeff"
values = [0.001]

[atmosphere]
density0 = 1.225
scale_height = 8500.0
pressure0 = 101325.0
gas_constant = 286.91
gamma = 1.4
"#;
        let config = BodyConfig::from_toml_str(toml_str).unwrap();
        assert_eq!(config.name, "TestBody");
        assert!((config.mass - 1.0e24).abs() < 1e10);
        assert!(config.rotation.is_some());
        assert!(config.atmosphere.is_some());
    }
}
