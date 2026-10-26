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
//! transition = { kind = "altitude", op = "gt", value = 10000.0 }
//!
//! [[phases]]
//! mode = "gravity_turn"
//! throttle = 1.0
//! kick_pitch = 0.0
//! kick_yaw = 0.087
//! kick_rate = 0.05
//! transition = { kind = "altitude", op = "gt", value = 80000.0 }
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

pub mod graph;
pub mod pred;
pub mod super_wf;
pub mod target_wf;

use serde::{Deserialize, Serialize};

use crate::target::TargetMode;

pub use pred::{PredOp, PredTerm};

/// 顶层工作流 tick 入口（编排，需 `&mut Assembly` 为多 body 构造 `BaseController`）。
/// `slice` 是上一步步进切片；第一步未就绪时遥测条件为假。
pub trait WorkFlow: Send {
    fn tick(
        &mut self,
        asm: &mut orbitx_vessel::Assembly,
        slice: &orbitx_protocol::mem::Slice,
        dt: f64,
    );
    fn is_done(&self) -> bool;
    /// 图编排点火（发射命令）。线性 phases/steps 工作流忽略。
    fn ignite(&mut self) {}
    /// HUD：当前目标油门（无 Target 驱动时为 0）。
    fn throttle_cmd(&self) -> f64 {
        0.0
    }
    /// HUD：俯仰 / 偏航 / 滚转目标 [rad]（无则全 0）。
    fn attitude_targets(&self) -> (f64, f64, f64) {
        (0.0, 0.0, 0.0)
    }
}

/// 从 `TargetMode` + 控制器瞬时指令生成 HUD 姿态目标。
pub fn attitude_from_target(ctrl: &crate::target::TargetController) -> (f64, f64, f64) {
    match ctrl.mode() {
        TargetMode::VerticalHold { .. } => (0.0, 0.0, 0.0),
        TargetMode::PitchTo {
            pitch, yaw, roll, ..
        } => (pitch, yaw, roll),
        TargetMode::GravityTurn { .. } => (ctrl.pitch_cmd(), ctrl.yaw_cmd(), 0.0),
        TargetMode::ProgradeHold { .. } | TargetMode::RetrogradeHold { .. } => (0.0, 0.0, 0.0),
    }
}

/// 工作流种类。
#[derive(Debug, Clone, Copy, Deserialize, Serialize, PartialEq, Eq)]
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
#[derive(Debug, Clone, Deserialize, Serialize, PartialEq)]
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
        #[serde(default = "default_kick_pitch")]
        kick_pitch: f64,
        #[serde(default = "default_kick_yaw")]
        kick_yaw: f64,
        #[serde(default = "default_kick_rate")]
        kick_rate: f64,
        #[serde(default = "default_min_alt")]
        min_alt: f64,
        #[serde(default = "default_min_speed")]
        min_speed: f64,
    },
}

fn default_kick_pitch() -> f64 {
    crate::target::DEFAULT_KICK_PITCH
}
fn default_kick_yaw() -> f64 {
    crate::target::DEFAULT_KICK_YAW
}
fn default_kick_rate() -> f64 {
    crate::target::DEFAULT_KICK_RATE
}
fn default_min_alt() -> f64 {
    crate::target::DEFAULT_MIN_ALT
}
fn default_min_speed() -> f64 {
    crate::target::DEFAULT_MIN_SPEED
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
            TargetModeDesc::PitchTo {
                pitch,
                yaw,
                throttle,
            } => TargetMode::PitchTo {
                pitch,
                yaw,
                roll: 0.0,
                throttle,
            },
            TargetModeDesc::ProgradeHold { throttle } => TargetMode::ProgradeHold { throttle },
            TargetModeDesc::RetrogradeHold { throttle } => TargetMode::RetrogradeHold { throttle },
            TargetModeDesc::GravityTurn {
                throttle,
                kick_pitch,
                kick_yaw,
                kick_rate,
                min_alt,
                min_speed,
            } => TargetMode::GravityTurn {
                throttle,
                kick_pitch,
                kick_yaw,
                kick_rate,
                min_alt,
                min_speed,
            },
        }
    }
}

/// 阶段过渡条件：与图 `terms[]` 元素同形的单条谓词（无 combine）。
pub type TransitionDesc = PredTerm;

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
    Rcs {
        group: String,
        axis: String,
        level: f64,
    },
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
                return Err(SchemaError::Validate(
                    "target 工作流不应含 [[steps]]".into(),
                ));
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
                        t.validate(&format!("phase[{i}] transition"))
                            .map_err(SchemaError::Validate)?;
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
                return Err(SchemaError::Validate(
                    "super 工作流不应含 [[phases]]".into(),
                ));
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
