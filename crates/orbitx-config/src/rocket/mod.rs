//! 火箭配置（rocket.toml）。
//!
//! 对应 Orbiter 的 vessel .cfg + clbkSetClassCaps。
//! 定义火箭的级结构、质量、推力等静态参数。
//! 推进一律 `[[stages.thrusters]]`，无级级单机糖字段。

use serde::{Deserialize, Serialize};
use std::path::Path;

/// 火箭配置。
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RocketConfig {
    /// 火箭名称。
    pub name: String,
    /// 类名（对应 Orbiter 的 Module 名）。
    pub class: String,
    /// 级列表（从底到顶；侧挂助推可插在列表中由 `dock_links` 连接）。
    pub stages: Vec<StageConfig>,
    /// 显式对接边。缺省时运行时对相邻级做顶/底自动对接。
    #[serde(default)]
    pub dock_links: Option<Vec<DockLinkConfig>>,
}

/// 两级之间的硬对接边（`stages` 下标 + 端口下标）。
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct DockLinkConfig {
    /// 本侧级在 `stages` 中的下标。
    pub stage: usize,
    /// 本侧端口下标。
    pub port: usize,
    /// 对方级下标。
    pub remote_stage: usize,
    /// 对方端口下标。
    pub remote_port: usize,
}

/// 单级上的对接口配置（可选；缺省由运行时按 length 生成顶/底）。
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct DockConfig {
    /// 体坐标系位置 [m]。
    pub pos: [f64; 3],
    /// 接近方向（单位向量）。
    pub dir: [f64; 3],
    /// 滚转对齐参考（单位向量）。
    pub rot: [f64; 3],
}

/// 单台推进器配置（真空额定 + 可选海平面双点）。
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ThrusterConfig {
    /// 体坐标系位置 [m]。
    pub pos: [f64; 3],
    /// 推力方向（体坐标系，单位向量）。
    pub dir: [f64; 3],
    /// 真空最大推力 [N]。
    pub thrust: f64,
    /// 真空比冲 [s]。
    pub isp: f64,
    /// 海平面推力 [N]（可选，与 isp_sl 用于推导 pfac）。
    #[serde(default)]
    pub thrust_sl: Option<f64>,
    /// 海平面比冲 [s]。
    #[serde(default)]
    pub isp_sl: Option<f64>,
    /// TVC 最大偏转角 [rad]。默认 0。
    #[serde(default)]
    pub max_gimbal: f64,
    /// TVC 最大偏转角速率 [rad/s]。默认 0。
    #[serde(default)]
    pub max_gimbal_rate: f64,
    /// TVC 偏转轴（体坐标系）。默认 [1,0,0]。
    #[serde(default = "default_gimbal_axis")]
    pub gimbal_axis: [f64; 3],
    /// 节流斜坡最大速率 [1/s]（开度分数每秒）。默认 0 = 瞬时。
    #[serde(default)]
    pub throttle_rate: f64,
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
    /// 推进器列表（有动力级非空；载荷为空）。
    #[serde(default)]
    pub thrusters: Vec<ThrusterConfig>,
    /// 级长度 [m]。
    pub length: f64,
    /// 级半径 [m]。
    pub radius: f64,
    /// 分离时施加的脉冲速度 [m/s]。
    pub separation_impulse: f64,
    /// 主惯量张量（体坐标系对角线）[kg·m²]，**真实惯量**（非归一化）。
    #[serde(default)]
    pub inertia: Option<[f64; 3]>,
    /// 重力梯度阻尼（Orbiter tidaldamp）。默认 0。
    #[serde(default)]
    pub tidaldamp: f64,
    /// 轴向阻力 Cd(M) 表 `[[mach, cd], …]`；有动力级建议填写。
    #[serde(default)]
    pub cd_mach: Vec<[f64; 2]>,
    /// 自定义对接口列表。缺省则运行时按 `length` 生成顶/底口。
    #[serde(default)]
    pub docks: Option<Vec<DockConfig>>,
}

/// serde 默认：gimbal 轴 = X。
fn default_gimbal_axis() -> [f64; 3] {
    [1.0, 0.0, 0.0]
}

impl StageConfig {
    /// 真空总推力 [N]。
    pub fn vacuum_thrust_sum(&self) -> f64 {
        self.thrusters.iter().map(|t| t.thrust).sum()
    }
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
mod tests;
