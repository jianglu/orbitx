//! 火箭配置（rocket.toml）。
//!
//! 对应 Orbiter 的 vessel .cfg + clbkSetClassCaps。
//! 定义火箭的级结构、质量、推力等静态参数。

use serde::{Deserialize, Serialize};
use std::path::Path;

/// 火箭配置。
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RocketConfig {
    /// 火箭名称。
    pub name: String,
    /// 类名（对应 Orbiter 的 Module 名）。
    pub class: String,
    /// 级列表（从底到顶）。
    pub stages: Vec<StageConfig>,
}

/// 单级配置。
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct StageConfig {
    /// 级名称。
    pub name: String,
    /// 空重（不含燃料）[kg]。
    pub dry_mass: f64,
    /// 燃料质量 [kg]。
    pub fuel_mass: f64,
    /// 发动机总推力 [N]。
    pub thrust: f64,
    /// 比冲 [s]。
    pub isp: f64,
    /// 级长度 [m]。
    pub length: f64,
    /// 级半径 [m]。
    pub radius: f64,
    /// 分离时施加的脉冲速度 [m/s]。
    pub separation_impulse: f64,
    /// 推力方向（体坐标系）[x, y, z]。
    pub engine_dir: [f64; 3],
    /// 发动机位置（体坐标系）[x, y, z]。
    pub engine_pos: [f64; 3],
}

impl RocketConfig {
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
    fn roundtrip_falcon9() {
        let config = RocketConfig {
            name: "Falcon 9".to_string(),
            class: "Falcon9".to_string(),
            stages: vec![
                StageConfig {
                    name: "F9-S1".to_string(),
                    dry_mass: 25600.0,
                    fuel_mass: 411000.0,
                    thrust: 7607000.0,
                    isp: 282.0,
                    length: 47.0,
                    radius: 1.85,
                    separation_impulse: 3.0,
                    engine_dir: [0.0, -1.0, 0.0],
                    engine_pos: [0.0, -23.5, 0.0],
                },
                StageConfig {
                    name: "F9-S2".to_string(),
                    dry_mass: 4000.0,
                    fuel_mass: 107500.0,
                    thrust: 934000.0,
                    isp: 348.0,
                    length: 14.0,
                    radius: 1.85,
                    separation_impulse: 2.0,
                    engine_dir: [0.0, -1.0, 0.0],
                    engine_pos: [0.0, -7.0, 0.0],
                },
            ],
        };

        let toml_str = config.to_toml_string().unwrap();
        let parsed = RocketConfig::from_toml_str(&toml_str).unwrap();

        assert_eq!(parsed.name, "Falcon 9");
        assert_eq!(parsed.stages.len(), 2);
        assert!((parsed.stages[0].dry_mass - 25600.0).abs() < 0.1);
        assert!((parsed.stages[0].engine_dir[1] - (-1.0)).abs() < 1e-10);
    }

    #[test]
    fn parse_toml_string() {
        let toml_str = r#"
name = "Test Rocket"
class = "TestRocket"

[[stages]]
name = "S1"
dry_mass = 1000.0
fuel_mass = 5000.0
thrust = 100000.0
isp = 300.0
length = 10.0
radius = 1.0
separation_impulse = 2.0
engine_dir = [0.0, -1.0, 0.0]
engine_pos = [0.0, -5.0, 0.0]
"#;
        let config = RocketConfig::from_toml_str(toml_str).unwrap();
        assert_eq!(config.name, "Test Rocket");
        assert_eq!(config.stages.len(), 1);
        assert!((config.stages[0].fuel_mass - 5000.0).abs() < 0.1);
    }
}
