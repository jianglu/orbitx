//! 发射场 `lp_xxx.toml`（orbitx 与 Godot 共用；见 `docs/CONFIG_TOML.md`）。
//!
//! 只解析/校验，不采样、不算高程。覆盖网格数据本体是伴生二进制文件，
//! 由 [`LaunchpadConfig::load_grid_data`] 按 lp 中参数读取校验。

use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

/// 本轮支持的 lp schema 版本。
pub const LAUNCHPAD_SCHEMA: i64 = 2;

/// 单个台位（借鉴 Orbiter `Base` 的多台位模型；坐标为发射场当地 ENU 系）。
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PadConfig {
    /// 台位名（场内唯一）。
    pub name: String,
    /// 发射轴基点相对场原点的 ENU 偏移 [east, north, up]（m）。
    /// `up` = 台架高度：台架顶面高出发射场底部平面（`base_elevation_m`）的高度。
    #[serde(default)]
    pub axis_offset_m: [f64; 3],
    /// 发射轴方向（当地 ENU 单位向量；竖直为 [0,0,1]）。
    #[serde(default = "default_axis_dir")]
    pub axis_dir: [f64; 3],
    /// 绕发射轴的方位角（度）。
    #[serde(default)]
    pub heading_deg: f64,
}

fn default_axis_dir() -> [f64; 3] {
    [0.0, 0.0, 1.0]
}

/// 场级填平域（schema 2 增量段，可选）：发射场坪圆域强制填平 +
/// 过渡带羽化，与 Godot 侧 shader **同一公式**（双端一字不差）：
///
/// ```text
/// t = clamp((dist − radius_m) / blend_m, 0, 1)
/// h = lerp(base_elevation_m, h_global, t)      // 域内强制 h = base（填平，非只截高）
/// ```
///
/// `dist` = 查询点相对圆心的 ENU 东/北分量范数（切平面投影）。
/// 域内法向 = ENU up、地表分类恒陆地；过渡带法向/分类取全球采样。
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PadAreaConfig {
    /// 圆心相对场原点的 ENU 偏移 [east, north]（m）；缺省 = 场原点。
    #[serde(default)]
    pub center_enu_m: [f64; 2],
    /// 填平半径（m，自圆心）。
    pub radius_m: f64,
    /// 过渡带宽度（m，自 radius 外沿向外羽化）；缺省 0 = 硬边界。
    #[serde(default)]
    pub blend_m: f64,
}

/// 发射场配置（`lp_xxx.toml`）。
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LaunchpadConfig {
    /// 文件版本（必须存在；规则见 `docs/CONFIG_TOML.md`）。
    pub schema: i64,
    /// 发射场名。
    pub name: String,
    /// 主天体名（与 scenario `primary` 对照）。
    pub body: String,
    /// 场原点纬度（度）。
    pub latitude_deg: f64,
    /// 场原点经度（度）。
    pub longitude_deg: f64,
    /// 发射场底部高程（m）：场坪的绝对地面高，与全球地表包同口径
    /// （相对天体高程球 `R_elev`）。发射场当地坐标系原点立在该平面上；
    /// 局部高程高于此值在采样时截平，水面不处理。
    pub base_elevation_m: f64,
    /// 台位列表（至少一个）。
    #[serde(default)]
    pub pads: Vec<PadConfig>,
    /// 场坪填平域（可选；语义见 `PadAreaConfig`）。
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub pad_area: Option<PadAreaConfig>,
    /// 覆盖网格数据文件（相对 lp 文件目录；f32 小端，行主序，row0 = 北缘）。
    #[serde(default)]
    pub elevation_override: Option<String>,
    /// 覆盖格距（m）；与 `elevation_override` 同时出现。
    #[serde(default)]
    pub grid_cell_m: Option<f64>,
    /// 覆盖网格列数（东向节点数）。
    #[serde(default)]
    pub grid_width: Option<usize>,
    /// 覆盖网格行数（北向节点数）。
    #[serde(default)]
    pub grid_height: Option<usize>,
    /// lp 文件所在目录（网格相对路径基准）；不入 TOML。
    #[serde(skip)]
    pub source_dir: PathBuf,
}

#[derive(Deserialize)]
struct RawLaunchpad {
    schema: Option<i64>,
    #[serde(flatten)]
    rest: toml::Value,
}

impl LaunchpadConfig {
    pub fn from_toml_str(s: &str) -> Result<Self, String> {
        let raw: RawLaunchpad =
            toml::from_str(s).map_err(|e| format!("解析发射场失败：{e}"))?;
        let schema = raw.schema.ok_or("发射场文件缺少 `schema` 字段，拒绝加载")?;
        if schema > LAUNCHPAD_SCHEMA {
            return Err(format!(
                "发射场文件版本更高（schema={schema}，当前支持 {LAUNCHPAD_SCHEMA}），请升级程序后再读取"
            ));
        }
        if schema < LAUNCHPAD_SCHEMA {
            return Err(format!(
                "发射场文件版本过旧（schema={schema}，当前 {LAUNCHPAD_SCHEMA}），暂无迁移规则"
            ));
        }
        let mut rest = raw.rest;
        if let toml::Value::Table(ref mut t) = rest {
            t.insert("schema".into(), toml::Value::Integer(schema));
        }
        let mut cfg: LaunchpadConfig = rest
            .try_into()
            .map_err(|e| format!("解析发射场失败：{e}"))?;
        cfg.validate()?;
        Ok(cfg)
    }

    pub fn from_file(path: &Path) -> Result<Self, String> {
        let text = std::fs::read_to_string(path)
            .map_err(|e| format!("读取发射场 `{}` 失败：{e}", path.display()))?;
        let mut cfg = Self::from_toml_str(&text)?;
        cfg.source_dir = path
            .parent()
            .map(|p| p.to_path_buf())
            .unwrap_or_else(|| PathBuf::from("."));
        Ok(cfg)
    }

    /// 按名查台位；`None` = 第一个台位。
    pub fn pad(&self, name: Option<&str>) -> Result<&PadConfig, String> {
        match name {
            None => self.pads.first().ok_or_else(|| "发射场无台位".to_string()),
            Some(n) => self
                .pads
                .iter()
                .find(|p| p.name == n)
                .ok_or_else(|| format!("发射场 `{}` 无台位 `{n}`", self.name)),
        }
    }

    /// 覆盖网格数据文件绝对路径（无覆盖时 None）。
    pub fn grid_data_path(&self) -> Option<PathBuf> {
        self.elevation_override
            .as_ref()
            .map(|rel| self.source_dir.join(rel))
    }

    /// 读取并校验覆盖网格数据（f32 小端，长度须 = width×height）。
    pub fn load_grid_data(&self) -> Result<Option<Vec<f32>>, String> {
        let Some(path) = self.grid_data_path() else {
            return Ok(None);
        };
        let (w, h) = (self.grid_width.unwrap_or(0), self.grid_height.unwrap_or(0));
        let bytes = std::fs::read(&path)
            .map_err(|e| format!("读取高程覆盖 `{}` 失败：{e}", path.display()))?;
        let expect = w * h * 4;
        if bytes.len() != expect {
            return Err(format!(
                "高程覆盖 `{}` 长度不符：期望 {expect} 字节（{w}×{h} f32），实际 {}",
                path.display(),
                bytes.len()
            ));
        }
        let mut data = Vec::with_capacity(w * h);
        for chunk in bytes.chunks_exact(4) {
            let v = f32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
            if !v.is_finite() {
                return Err(format!("高程覆盖 `{}` 含非有限值", path.display()));
            }
            data.push(v);
        }
        Ok(Some(data))
    }

    fn validate(&mut self) -> Result<(), String> {
        if self.name.trim().is_empty() {
            return Err("发射场 `name` 不能为空".into());
        }
        if self.body.trim().is_empty() {
            return Err("发射场 `body` 不能为空".into());
        }
        if !(-90.0..=90.0).contains(&self.latitude_deg) {
            return Err(format!("latitude_deg 越界：{}", self.latitude_deg));
        }
        if !(-180.0..=180.0).contains(&self.longitude_deg) {
            return Err(format!("longitude_deg 越界：{}", self.longitude_deg));
        }
        if !self.base_elevation_m.is_finite() || !(-12_000.0..=12_000.0).contains(&self.base_elevation_m) {
            return Err(format!("base_elevation_m 非法：{}", self.base_elevation_m));
        }
        if self.pads.is_empty() {
            return Err("发射场至少需要一个 [[pads]] 台位".into());
        }
        let mut names = std::collections::HashSet::new();
        for pad in &mut self.pads {
            if pad.name.trim().is_empty() {
                return Err("台位 `name` 不能为空".into());
            }
            if !names.insert(pad.name.clone()) {
                return Err(format!("台位名重复：{}", pad.name));
            }
            let len = (pad.axis_dir[0].powi(2)
                + pad.axis_dir[1].powi(2)
                + pad.axis_dir[2].powi(2))
            .sqrt();
            if !(len > 1e-9) || !len.is_finite() {
                return Err(format!("台位 `{}` 的 axis_dir 非法", pad.name));
            }
            for c in &mut pad.axis_dir {
                *c /= len;
            }
            if !pad.heading_deg.is_finite() || pad.axis_offset_m.iter().any(|v| !v.is_finite()) {
                return Err(format!("台位 `{}` 含非有限数值", pad.name));
            }
        }
        if let Some(pa) = &self.pad_area {
            if !(pa.radius_m > 0.0) || !pa.radius_m.is_finite() {
                return Err("pad_area.radius_m 须为正有限值".into());
            }
            if !(pa.blend_m >= 0.0) || !pa.blend_m.is_finite() {
                return Err("pad_area.blend_m 须为非负有限值".into());
            }
            if pa.center_enu_m.iter().any(|v| !v.is_finite()) {
                return Err("pad_area.center_enu_m 含非有限数值".into());
            }
        }
        match (
            &self.elevation_override,
            self.grid_cell_m,
            self.grid_width,
            self.grid_height,
        ) {            (None, None, None, None) => {}
            (Some(_), Some(cell), Some(w), Some(h)) => {
                if !(cell > 0.0) || w < 2 || h < 2 {
                    return Err("覆盖网格参数非法（grid_cell_m>0 且宽高≥2）".into());
                }
                // 被选台位（全部台位）轴基点须在网格足迹内。
                let ex = (w - 1) as f64 * cell / 2.0;
                let ey = (h - 1) as f64 * cell / 2.0;
                for pad in &self.pads {
                    if pad.axis_offset_m[0].abs() > ex || pad.axis_offset_m[1].abs() > ey {
                        return Err(format!(
                            "台位 `{}` 轴基点 ({}, {}) 在覆盖网格足迹 (±{ex}, ±{ey}) 外",
                            pad.name, pad.axis_offset_m[0], pad.axis_offset_m[1]
                        ));
                    }
                }
            }
            _ => {
                return Err(
                    "elevation_override 与 grid_cell_m/grid_width/grid_height 须同时出现".into(),
                )
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const LP: &str = r#"
schema = 2
name = "Cape Test"
body = "Earth"
latitude_deg = 28.5
longitude_deg = -80.5
base_elevation_m = 12.0

[[pads]]
name = "LC-39A"
axis_offset_m = [100.0, -50.0, 0.0]
heading_deg = 90.0

[[pads]]
name = "LC-39B"
"#;

    #[test]
    fn parses_pads_and_defaults() {
        let cfg = LaunchpadConfig::from_toml_str(LP).unwrap();
        assert_eq!(cfg.pads.len(), 2);
        assert_eq!(cfg.base_elevation_m, 12.0);
        let a = cfg.pad(None).unwrap();
        assert_eq!(a.name, "LC-39A");
        assert_eq!(a.axis_dir, [0.0, 0.0, 1.0]);
        let b = cfg.pad(Some("LC-39B")).unwrap();
        assert_eq!(b.axis_offset_m, [0.0, 0.0, 0.0]);
        assert!(cfg.pad(Some("nope")).is_err());
        assert!(cfg.load_grid_data().unwrap().is_none());
    }

    #[test]
    fn schema_gate() {
        assert!(LaunchpadConfig::from_toml_str("name='x'").is_err());
        let newer = LP.replace("schema = 2", "schema = 3");
        let err = LaunchpadConfig::from_toml_str(&newer).unwrap_err();
        assert!(err.contains("版本更高"), "{err}");
        let older = LP.replace("schema = 2", "schema = 1");
        let err = LaunchpadConfig::from_toml_str(&older).unwrap_err();
        assert!(err.contains("版本过旧"), "{err}");
        // 缺 base_elevation_m → 拒绝
        let no_base = LP.replace("base_elevation_m = 12.0\n", "");
        assert!(LaunchpadConfig::from_toml_str(&no_base).is_err());
    }

    #[test]
    fn rejects_bad_lat_and_dup_pad() {
        let bad = LP.replace("28.5", "95.0");
        assert!(LaunchpadConfig::from_toml_str(&bad).is_err());
        let dup = LP.replace("LC-39B", "LC-39A");
        assert!(LaunchpadConfig::from_toml_str(&dup).is_err());
        let no_pad = "schema=2\nname='x'\nbody='Earth'\nlatitude_deg=0.0\nlongitude_deg=0.0\nbase_elevation_m=0.0\n";
        assert!(LaunchpadConfig::from_toml_str(no_pad).is_err());
    }

    #[test]
    fn grid_footprint_gate_and_data_len() {
        let tmp = tempfile::tempdir().unwrap();
        let lp_path = tmp.path().join("lp_t.toml");
        let with_grid = LP.replacen(
            "[[pads]]",
            "elevation_override = \"grid.f32\"\ngrid_cell_m = 10.0\ngrid_width = 11\ngrid_height = 11\n\n[[pads]]",
            1,
        );
        // 足迹 ±50 m；LC-39A 偏移 (100,-50) 越界 → 拒绝
        std::fs::write(&lp_path, &with_grid).unwrap();
        assert!(LaunchpadConfig::from_file(&lp_path).is_err());

        let ok = with_grid.replace("[100.0, -50.0, 0.0]", "[40.0, -50.0, 0.0]");
        std::fs::write(&lp_path, &ok).unwrap();
        let cfg = LaunchpadConfig::from_file(&lp_path).unwrap();
        // 数据文件缺失 → 读取报错
        assert!(cfg.load_grid_data().is_err());
        // 长度不符 → 报错；补齐后通过
        std::fs::write(tmp.path().join("grid.f32"), vec![0u8; 100]).unwrap();
        assert!(cfg.load_grid_data().is_err());
        let mut bytes = Vec::new();
        for _ in 0..121 {
            bytes.extend_from_slice(&7.0f32.to_le_bytes());
        }
        std::fs::write(tmp.path().join("grid.f32"), bytes).unwrap();
        let data = cfg.load_grid_data().unwrap().unwrap();
        assert_eq!(data.len(), 121);
        assert_eq!(data[0], 7.0);
    }

    #[test]
    fn pad_area_parsed_and_validated() {
        let s = r#"
schema = 2
name = "P"
body = "Earth"
latitude_deg = 0.0
longitude_deg = 0.0
base_elevation_m = 1.4

[pad_area]
center_enu_m = [10.0, -5.0]
radius_m = 60.0
blend_m = 20.0

[[pads]]
name = "LC-1"
"#;
        let c = LaunchpadConfig::from_toml_str(s).unwrap();
        let pa = c.pad_area.as_ref().unwrap();
        assert_eq!(pa.center_enu_m, [10.0, -5.0]);
        assert!((pa.radius_m - 60.0).abs() < 1e-12);
        assert!((pa.blend_m - 20.0).abs() < 1e-12);

        // 缺段兼容：pad_area = None。
        let s2 = s.replace("[pad_area]\ncenter_enu_m = [10.0, -5.0]\nradius_m = 60.0\nblend_m = 20.0\n\n", "");
        let c2 = LaunchpadConfig::from_toml_str(&s2).unwrap();
        assert!(c2.pad_area.is_none());

        // radius 非法拒绝。
        let s3 = s.replace("radius_m = 60.0", "radius_m = 0.0");
        assert!(LaunchpadConfig::from_toml_str(&s3).is_err());
    }
}
