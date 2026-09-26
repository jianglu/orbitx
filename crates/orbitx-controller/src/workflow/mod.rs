//! `WorkFlow`——Godot 编辑的工作流 toml 解析 + 编排（模式 c/d）。
//!
//! WorkFlow 模式下 **WorkFlow 自管子控制器**（Runtime 不直接持多 Controller）。`WorkFlow`
//! trait tick 取 `&mut Assembly` 以便为多 body 构造 `BaseController`。
//!
//! - [`TargetWorkFlow`]（模式 c）：own 1 个 `TargetController`（主 body）+ 主 body caps，
//!   tick 按阶段条件设目标配置，构造主 body `BaseController` 调 `target.tick`。
//! - [`SuperWorkFlow`]（模式 d）：own 子控制器舰队 + 各 body caps，tick 按 toml 编排为每 body
//!   构造 `BaseController`、经 base 读遥测/姿态、精确命令各部件；分离时按编排重建 caps +
//!   派生子控制器（入轨自动驾驶）。
//!
//! caps 归属：WorkFlow 模式 → WorkFlow own；手动模式 → Runtime own。控制器不 own caps。
//!
//! ## TOML schema
//!
//! ### TargetWorkFlow（`kind = "target"`）
//! ```toml
//! kind = "target"
//! name = "falcon9-ascent"
//!
//! [[phases]]
//! mode = "vertical_hold"
//! throttle = 1.0
//! transition = { altitude_gt = 10000.0 }
//!
//! [[phases]]
//! mode = "gravity_turn"
//! throttle = 1.0
//! kick_angle = 0.087
//! kick_rate = 0.05
//! transition = { altitude_gt = 80000.0 }
//!
//! [[phases]]
//! mode = "prograde_hold"
//! throttle = 1.0
//! # 末段可不写 transition（workflow 永驻该阶段，is_done 由上层判）
//! ```
//!
//! ### SuperWorkFlow（`kind = "super"`）
//! ```toml
//! kind = "super"
//! name = "falcon9-full"
//!
//! [[steps]]
//! action = "throttle"
//! group = "Core"
//! level = 1.0
//!
//! [[steps]]
//! action = "tvc"
//! group = "Core-tvc"
//! pitch = 0.0
//! yaw = 0.0
//!
//! [[steps]]
//! action = "wait"
//! duration = 5.0
//!
//! [[steps]]
//! action = "separate"
//! point = "Booster-sep-0"
//! ```

pub mod super_wf;
pub mod target_wf;

use serde::Deserialize;

use crate::target::TargetMode;

/// 顶层工作流 tick 入口（编排，需 `&mut Assembly` 为多 body 构造 `BaseController`）。
pub trait WorkFlow: Send {
    fn tick(&mut self, asm: &mut orbitx_vessel::Assembly, dt: f64);
    fn is_done(&self) -> bool;
}

/// 工作流种类。
#[derive(Debug, Clone, Copy, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum WorkFlowKind {
    Target,
    Super,
}

/// 工作流 TOML 描述（顶层）。
#[derive(Debug, Clone, Deserialize)]
pub struct WorkFlowDesc {
    pub kind: WorkFlowKind,
    pub name: String,
    #[serde(default)]
    pub phases: Option<Vec<PhaseDesc>>,
    #[serde(default)]
    pub steps: Option<Vec<StepDesc>>,
}

/// 目标导向模式（TOML 友好；`#[serde(tag = "mode")]` 内联表示）。
#[derive(Debug, Clone, Deserialize, PartialEq)]
#[serde(tag = "mode")]
pub enum TargetModeDesc {
    #[serde(rename = "vertical_hold")]
    VerticalHold { throttle: f64 },
    #[serde(rename = "pitch_to")]
    PitchTo { pitch: f64, yaw: f64, throttle: f64 },
    #[serde(rename = "prograde_hold")]
    ProgradeHold { throttle: f64 },
    #[serde(rename = "retrograde_hold")]
    RetrogradeHold { throttle: f64 },
    #[serde(rename = "gravity_turn")]
    GravityTurn {
        throttle: f64,
        #[serde(default = "default_kick_angle")]
        kick_angle: f64,
        #[serde(default = "default_kick_rate")]
        kick_rate: f64,
    },
}

fn default_kick_angle() -> f64 {
    crate::target::DEFAULT_KICK_ANGLE
}
fn default_kick_rate() -> f64 {
    crate::target::DEFAULT_KICK_RATE
}

impl TargetModeDesc {
    /// 转为运行时 `TargetMode`。
    pub fn to_mode(&self) -> TargetMode {
        self.clone().into()
    }
}

impl From<TargetModeDesc> for TargetMode {
    fn from(d: TargetModeDesc) -> Self {
        match d {
            TargetModeDesc::VerticalHold { throttle } => TargetMode::VerticalHold { throttle },
            TargetModeDesc::PitchTo { pitch, yaw, throttle } => TargetMode::PitchTo { pitch, yaw, throttle },
            TargetModeDesc::ProgradeHold { throttle } => TargetMode::ProgradeHold { throttle },
            TargetModeDesc::RetrogradeHold { throttle } => TargetMode::RetrogradeHold { throttle },
            TargetModeDesc::GravityTurn {
                throttle,
                kick_angle,
                kick_rate,
            } => TargetMode::GravityTurn {
                throttle,
                kick_angle,
                kick_rate,
            },
        }
    }
}

/// 阶段过渡条件。**恰好一个**条件字段被设（解析时校验）；满足即进入下一阶段。
#[derive(Debug, Clone, Deserialize, PartialEq, Default)]
pub struct TransitionDesc {
    pub altitude_gt: Option<f64>,
    pub speed_gt: Option<f64>,
    pub apoapsis_gt: Option<f64>,
    pub periapsis_gt: Option<f64>,
    pub fuel_pct_lt: Option<f64>,
    pub time_gt: Option<f64>,
}

impl TransitionDesc {
    /// 已设条件数（合法值恰为 1；末段可省略 transition 整体）。
    pub fn condition_count(&self) -> usize {
        [
            self.altitude_gt.is_some(),
            self.speed_gt.is_some(),
            self.apoapsis_gt.is_some(),
            self.periapsis_gt.is_some(),
            self.fuel_pct_lt.is_some(),
            self.time_gt.is_some(),
        ]
        .iter()
        .filter(|b| **b)
        .count()
    }
}

/// TargetWorkFlow 阶段描述。
#[derive(Debug, Clone, Deserialize, PartialEq)]
pub struct PhaseDesc {
    #[serde(flatten)]
    pub mode: TargetModeDesc,
    /// 末段可省略（永驻）；非末段必须恰好一个条件。
    #[serde(default)]
    pub transition: Option<TransitionDesc>,
}

/// SuperWorkFlow 步骤（`#[serde(tag = "action")]` 内联表示）。
#[derive(Debug, Clone, Deserialize, PartialEq)]
#[serde(tag = "action")]
pub enum StepDesc {
    #[serde(rename = "throttle")]
    Throttle { group: String, level: f64 },
    #[serde(rename = "tvc")]
    Tvc { group: String, pitch: f64, yaw: f64 },
    #[serde(rename = "rcs")]
    Rcs { group: String, axis: String, level: f64 },
    #[serde(rename = "separate")]
    Separate { point: String },
    #[serde(rename = "wait")]
    Wait { duration: f64 },
}

use std::fmt;

/// 解析错误：toml 语法 + schema 校验。
#[derive(Debug)]
pub enum SchemaError {
    Parse(toml::de::Error),
    Validate(String),
}

impl fmt::Display for SchemaError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SchemaError::Parse(e) => write!(f, "toml 解析失败: {e}"),
            SchemaError::Validate(msg) => write!(f, "schema 校验失败: {msg}"),
        }
    }
}

impl std::error::Error for SchemaError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            SchemaError::Parse(e) => Some(e),
            SchemaError::Validate(_) => None,
        }
    }
}

impl From<toml::de::Error> for SchemaError {
    fn from(e: toml::de::Error) -> Self {
        SchemaError::Parse(e)
    }
}

/// 解析工作流 TOML（含 kind 与 phases/steps 匹配校验）。
pub fn from_toml_str(s: &str) -> Result<WorkFlowDesc, SchemaError> {
    let desc: WorkFlowDesc = toml::from_str(s)?;
    validate(&desc)?;
    Ok(desc)
}

fn validate(desc: &WorkFlowDesc) -> Result<(), SchemaError> {
    if desc.name.trim().is_empty() {
        return Err(SchemaError::Validate("name 不能为空".into()));
    }
    match desc.kind {
        WorkFlowKind::Target => {
            let phases = desc
                .phases
                .as_ref()
                .ok_or_else(|| SchemaError::Validate("target 工作流需要 [[phases]]".into()))?;
            if desc.steps.is_some() {
                return Err(SchemaError::Validate("target 工作流不应含 [[steps]]".into()));
            }
            if phases.is_empty() {
                return Err(SchemaError::Validate("phases 不能为空".into()));
            }
            for (i, p) in phases.iter().enumerate() {
                let is_last = i == phases.len() - 1;
                match &p.transition {
                    None => {
                        if !is_last {
                            return Err(SchemaError::Validate(format!(
                                "phase[{i}] 非末段必须 transition"
                            )));
                        }
                    }
                    Some(t) => {
                        let n = t.condition_count();
                        if n != 1 {
                            return Err(SchemaError::Validate(format!(
                                "phase[{i}] transition 须恰好一个条件，实际 {n}"
                            )));
                        }
                    }
                }
            }
        }
        WorkFlowKind::Super => {
            let steps = desc
                .steps
                .as_ref()
                .ok_or_else(|| SchemaError::Validate("super 工作流需要 [[steps]]".into()))?;
            if desc.phases.is_some() {
                return Err(SchemaError::Validate("super 工作流不应含 [[phases]]".into()));
            }
            if steps.is_empty() {
                return Err(SchemaError::Validate("steps 不能为空".into()));
            }
            for (i, s) in steps.iter().enumerate() {
                if let StepDesc::Wait { duration } = s {
                    if *duration < 0.0 {
                        return Err(SchemaError::Validate(format!(
                            "step[{i}] wait duration 不能为负"
                        )));
                    }
                }
                if let StepDesc::Throttle { level, .. } = s {
                    if !(0.0..=1.0).contains(level) {
                        return Err(SchemaError::Validate(format!(
                            "step[{i}] throttle level 须在 [0,1]，实际 {level}"
                        )));
                    }
                }
            }
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests;
