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
/// `earth` 先找进程旁 `scenarios/scenario_earth.toml`（含 macOS `Contents/Resources`），
/// 旁路都不存在时才用编译期 crate 预设。
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
        "earth" => resolve_earth_preset(&earth_sidecar_roots(), &earth_dev_preset()),
        other => Err(format!(
            "未知环境别名 `{other}`（可用：earth，或直接传 scenario_xxx.toml 路径）"
        )),
    }
}

/// 导出包数据根：exe 同目录，以及 macOS `.app` 的 `Contents/Resources`。
fn earth_sidecar_roots() -> Vec<PathBuf> {
    let mut roots = Vec::new();
    if let Ok(exe) = std::env::current_exe() {
        if let Some(dir) = exe.parent() {
            roots.push(dir.to_path_buf());
            roots.push(dir.join("..").join("Resources"));
        }
    }
    roots
}

fn earth_dev_preset() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("presets/scenario_earth.toml")
}

/// 按顺序查找 `scenarios/scenario_earth.toml`。旁路都不存在、且 `dev_preset` 是文件时才用它。
fn resolve_earth_preset(roots: &[PathBuf], dev_preset: &Path) -> Result<PathBuf, String> {
    let mut tried = Vec::with_capacity(roots.len() + 1);
    for root in roots {
        let side = root.join("scenarios").join("scenario_earth.toml");
        tried.push(side.display().to_string());
        if side.is_file() {
            return Ok(side);
        }
    }
    tried.push(dev_preset.display().to_string());
    if dev_preset.is_file() {
        return Ok(dev_preset.to_path_buf());
    }
    Err(format!(
        "环境别名 `earth` 未找到 scenario_earth.toml（已查找：{}）",
        tried.join("；")
    ))
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

    /// 多个候选根时命中第一个存在的旁路文件，不落到后面的编译期预设。
    #[test]
    fn earth_sidecar_beats_dev_preset() {
        let tmp = std::env::temp_dir().join(format!("orbitx-earth-side-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&tmp);
        let missing = tmp.join("missing");
        let first = tmp.join("first");
        let later = tmp.join("later");
        std::fs::create_dir_all(first.join("scenarios")).unwrap();
        std::fs::create_dir_all(later.join("scenarios")).unwrap();
        let first_file = first.join("scenarios").join("scenario_earth.toml");
        std::fs::write(&first_file, "first").unwrap();
        std::fs::write(later.join("scenarios").join("scenario_earth.toml"), "later").unwrap();
        let dev = tmp.join("presets").join("scenario_earth.toml");
        std::fs::create_dir_all(dev.parent().unwrap()).unwrap();
        std::fs::write(&dev, "dev").unwrap();

        let got = resolve_earth_preset(&[missing, first, later], &dev).unwrap();
        assert_eq!(got, first_file);
        let _ = std::fs::remove_dir_all(&tmp);
    }

    /// 旁路都不存在时才用编译期预设。
    #[test]
    fn earth_falls_back_to_dev_preset_when_sidecars_missing() {
        let missing = std::env::temp_dir().join(format!(
            "orbitx-earth-miss-{}",
            std::process::id()
        ));
        let dev = earth_dev_preset();
        let got = resolve_earth_preset(&[missing], &dev).unwrap();
        assert_eq!(got, dev);
    }
}
