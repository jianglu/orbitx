//! 环境文件 `scenario_xxx.toml`。
//!
//! 只描述行星系统。不加载火箭、发射台或任务。

use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

use crate::body::BodyConfig;

/// 一份可运行的行星场景。
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PlanetaryScenario {
    /// 场景名。
    pub name: String,
    /// 画面上当作恒星的天体名（可无力学）。
    #[serde(default)]
    pub star: String,
    /// 起始修正儒略日。
    pub mjd: f64,
    /// 积分原点与大气所属天体。
    pub primary: String,
    /// 天体表。渲染与力学读同一张表。
    pub bodies: Vec<BodyConfig>,
    /// 父子关系：`(child_name, parent_name)`。
    #[serde(default)]
    pub parents: Vec<(String, String)>,
}

impl PlanetaryScenario {
    pub fn from_toml_str(s: &str) -> Result<Self, toml::de::Error> {
        toml::from_str(s)
    }

    pub fn from_file(path: &Path) -> Result<Self, String> {
        let text = std::fs::read_to_string(path)
            .map_err(|e| format!("读取环境 `{}` 失败：{e}", path.display()))?;
        Self::from_toml_str(&text)
            .map_err(|e| format!("解析环境 `{}` 失败：{e}", path.display()))
    }
}

/// 把 CLI/runtime 的 `--scenario` 参数解析成文件路径。
///
/// 无路径分隔符且无扩展名时当作别名。目前只有 `earth`。
pub fn resolve_scenario_spec(spec: &str) -> Result<PathBuf, String> {
    let path = Path::new(spec);
    let is_alias = path.extension().is_none() && path.components().count() <= 1;
    if is_alias {
        return scenario_alias_path(spec);
    }
    Ok(path.to_path_buf())
}

fn scenario_alias_path(alias: &str) -> Result<PathBuf, String> {
    match alias {
        "earth" => Ok(Path::new(env!("CARGO_MANIFEST_DIR")).join("presets/scenario_earth.toml")),
        other => Err(format!(
            "未知环境别名 `{other}`（可用：earth，或直接传 scenario_xxx.toml 路径）"
        )),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn earth_alias_points_at_preset() {
        let path = resolve_scenario_spec("earth").unwrap();
        assert!(path.ends_with("scenario_earth.toml"), "{}", path.display());
        let scn = PlanetaryScenario::from_file(&path).unwrap();
        assert_eq!(scn.primary, "Earth");
        assert_eq!(scn.bodies.len(), 2);
        assert!(scn.bodies.iter().any(|b| b.name == "Sun" && !b.dynamics));
        assert!(scn.bodies.iter().any(|b| b.name == "Earth" && b.dynamics));
    }
}
