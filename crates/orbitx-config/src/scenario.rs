//! 场景配置（scenario.toml）。
//!
//! 对应 Orbiter 的 .scn 文件。描述模拟环境的初始状态：
//! 时间、焦点天体、相机、HUD、飞船列表。

use serde::{Deserialize, Serialize};
use std::path::Path;

/// 场景配置。
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ScenarioConfig {
    /// 模拟环境。
    pub environment: Environment,
    /// 相机焦点。
    pub focus: Focus,
    /// 相机配置（可选）。
    #[serde(default)]
    pub camera: Option<CameraConfig>,
    /// HUD 配置（可选）。
    #[serde(default)]
    pub hud: Option<HudConfig>,
    /// 飞船列表。
    pub ships: Vec<ShipConfig>,
}

/// 模拟环境。
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Environment {
    /// 行星系名称。
    pub system: String,
    /// 模拟开始时间（MJD）。
    pub mjd: f64,
}

/// 相机焦点。
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Focus {
    /// 相机跟随的飞船名称。
    pub ship: String,
}

/// 相机配置。
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CameraConfig {
    /// 相机目标（天体或飞船名）。
    pub target: String,
    /// 相机模式。
    #[serde(default = "default_camera_mode")]
    pub mode: String,
    /// 距离 [m]。
    #[serde(default = "default_distance")]
    pub distance: f64,
    /// 方位角 [rad]。
    #[serde(default)]
    pub azimuth: f64,
    /// 仰角 [rad]。
    #[serde(default)]
    pub elevation: f64,
    /// 视场角 [deg]。
    #[serde(default = "default_fov")]
    pub fov: f64,
}

fn default_camera_mode() -> String {
    "external".to_string()
}
fn default_distance() -> f64 {
    300.0
}
fn default_fov() -> f64 {
    45.0
}

/// HUD 配置。
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct HudConfig {
    /// HUD 模式："surface" | "orbit" | "docking"。
    pub mode: String,
}

/// 飞船配置。
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ShipConfig {
    /// 飞船名称。
    pub name: String,
    /// 类名（对应火箭配置的 class）。
    pub class: String,
    /// 状态："landed" | "orbiting"。
    pub status: String,
    /// 参考天体。
    pub body: String,
    /// 着陆：经度 [deg]。
    #[serde(default)]
    pub longitude: Option<f64>,
    /// 着陆：纬度 [deg]。
    #[serde(default)]
    pub latitude: Option<f64>,
    /// 着陆：朝向 [deg]。
    #[serde(default)]
    pub heading: Option<f64>,
    /// 着陆：地面高度 [m]。
    #[serde(default)]
    pub altitude: Option<f64>,
    /// 轨道：位置（相对参考天体）[m]。
    #[serde(default)]
    pub rpos: Option<[f64; 3]>,
    /// 轨道：速度 [m/s]。
    #[serde(default)]
    pub rvel: Option<[f64; 3]>,
    /// 姿态欧拉角 [deg]。
    #[serde(default)]
    pub arot: Option<[f64; 3]>,
    /// 各燃料罐液位 (0..1)。
    #[serde(default)]
    pub fuel_level: Option<Vec<f64>>,
    /// 对接信息：[[本端口, 对方端口, 对方名称], ...]。
    #[serde(default)]
    pub dock_info: Option<Vec<DockInfo>>,
}

/// 对接信息。
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct DockInfo {
    /// 本方端口索引。
    pub port: u32,
    /// 对方端口索引。
    pub remote_port: u32,
    /// 对方飞船名称。
    pub vessel: String,
}

impl ScenarioConfig {
    /// 从 TOML 字符串解析。
    pub fn from_toml_str(s: &str) -> Result<Self, toml::de::Error> {
        toml::from_str(s)
    }

    /// 序列化为 TOML 字符串。
    pub fn to_toml_string(&self) -> Result<String, toml::ser::Error> {
        toml::to_string_pretty(self)
    }

    /// 从文件读取。
    pub fn from_file(path: &Path) -> Result<Self, Box<dyn std::error::Error>> {
        let s = std::fs::read_to_string(path)?;
        Self::from_toml_str(&s).map_err(Into::into)
    }

    /// 写入文件。
    pub fn to_file(&self, path: &Path) -> Result<(), Box<dyn std::error::Error>> {
        let s = self.to_toml_string()?;
        std::fs::write(path, s)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip_scenario() {
        let config = ScenarioConfig {
            environment: Environment {
                system: "Sol".to_string(),
                mjd: 52345.5,
            },
            focus: Focus {
                ship: "Falcon-9".to_string(),
            },
            camera: Some(CameraConfig {
                target: "Earth".to_string(),
                mode: "external".to_string(),
                distance: 300.0,
                azimuth: 0.0,
                elevation: 0.3,
                fov: 45.0,
            }),
            hud: Some(HudConfig {
                mode: "surface".to_string(),
            }),
            ships: vec![ShipConfig {
                name: "Falcon-9".to_string(),
                class: "Falcon9".to_string(),
                status: "landed".to_string(),
                body: "Earth".to_string(),
                longitude: Some(-118.08),
                latitude: Some(34.64),
                heading: Some(58.0),
                altitude: Some(2.5),
                rpos: None,
                rvel: None,
                arot: None,
                fuel_level: Some(vec![1.0, 1.0]),
                dock_info: None,
            }],
        };

        let toml_str = config.to_toml_string().unwrap();
        let parsed = ScenarioConfig::from_toml_str(&toml_str).unwrap();

        assert_eq!(parsed.focus.ship, "Falcon-9");
        assert_eq!(parsed.ships.len(), 1);
        assert_eq!(parsed.ships[0].status, "landed");
        assert!((parsed.ships[0].longitude.unwrap() - (-118.08)).abs() < 1e-6);
    }

    #[test]
    fn parse_orbiting_ship() {
        let toml_str = r#"
[environment]
system = "Sol"
mjd = 52345.5

[focus]
ship = "ISS"

[[ships]]
name = "ISS"
class = "Station"
status = "orbiting"
body = "Earth"
rpos = [4770488.0, 4245945.0, -2122703.0]
rvel = [5242.0, -5610.0, 567.0]
arot = [107.6, -62.6, -58.7]
fuel_level = [1.0]
"#;
        let config = ScenarioConfig::from_toml_str(toml_str).unwrap();
        assert_eq!(config.ships[0].status, "orbiting");
        assert!(config.ships[0].rpos.is_some());
        assert!(config.ships[0].longitude.is_none());
    }

    #[test]
    fn parse_docked_ships() {
        let toml_str = r#"
[environment]
system = "Sol"
mjd = 51544.5

[focus]
ship = "GL-02"

[[ships]]
name = "ISS"
class = "Station"
status = "orbiting"
body = "Earth"
rpos = [1000000.0, 0.0, 0.0]
rvel = [0.0, 7000.0, 0.0]

[[ships]]
name = "GL-02"
class = "DeltaGlider"
status = "orbiting"
body = "Earth"
rpos = [1000100.0, 0.0, 0.0]
rvel = [0.0, 7000.0, 0.0]
fuel_level = [1.0]

[[ships]]
dock_info = [
  { port = 0, remote_port = 1, vessel = "ISS" },
]
"#;
        // This test verifies dock_info parsing format.
        // Note: the last ship block has no name/class, so it would fail.
        let _toml_str = r#"
[environment]
system = "Sol"
mjd = 51544.5

[focus]
ship = "GL-02"

[[ships]]
name = "GL-02"
class = "DeltaGlider"
status = "orbiting"
body = "Earth"
rpos = [1000100.0, 0.0, 0.0]
rvel = [0.0, 7000.0, 0.0]

[[ships]]
name = "ISS"
class = "Station"
status = "orbiting"
body = "Earth"
dock_info = [
  { port = 0, remote_port = 1, vessel = "GL-02" },
]
rpos = [1000000.0, 0.0, 0.0]
rvel = [0.0, 7000.0, 0.0]
"#;
        let config = ScenarioConfig::from_toml_str(_toml_str).unwrap();
        assert!(config.ships[1].dock_info.is_some());
        let di = &config.ships[1].dock_info.as_ref().unwrap()[0];
        assert_eq!(di.port, 0);
        assert_eq!(di.vessel, "GL-02");
    }
}
