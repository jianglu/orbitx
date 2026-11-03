//! 黑匣子帧类型（schema 1）。
//!
//! 字段全是 plain 类型 / f64 数组，与 runtime 的 `Assembly` / `Slice` 类型解耦：
//! 帧里记录的是**权威直录**（vessel state / diagnostics 原样），不是对外遥测投影。
//! 序列化为 CBOR map；`kind` 由 serde tag 写入。新增字段保持向后兼容
//! （serde 忽略未知字段），不兼容变更须升 [`SCHEMA_VERSION`]。

use serde::{Deserialize, Serialize};

/// 当前磁盘格式版本（`FLIGHT_RECORDER.md`：不兼容变更必须递增）。
pub const SCHEMA_VERSION: u32 = 1;

/// 单帧：会话头帧或步帧。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind")]
pub enum RecorderFrame {
    #[serde(rename = "session_start")]
    SessionStart(SessionStartFrame),
    #[serde(rename = "step")]
    Step(StepFrame),
}

impl RecorderFrame {
    pub fn sim_t(&self) -> u64 {
        match self {
            Self::SessionStart(f) => f.sim_t,
            Self::Step(f) => f.sim_t,
        }
    }
}

/// `kind = "session_start"`：会话初（启动 / Reset 成功后）一帧。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SessionStartFrame {
    pub schema_version: u32,
    /// 会话起始仿真时刻 [ms]（Reset 后归零）。
    pub sim_t: u64,
    /// 固定物理步长 [ms]。
    #[serde(rename = "sim_dt")]
    pub sim_dt_ms: u64,
    pub rocket: RocketInfo,
    pub control: ControlInfo,
    /// 环境完整初始快照（`star` 仅此帧带）。
    pub environment: EnvironmentSnapshot,
    /// 全部航天器初始状态（含日后可分离体）。
    pub vessels: Vec<VesselInitial>,
}

/// `kind = "step"`：每完整物理步一帧。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StepFrame {
    /// 本步结束后仿真时刻 [ms]。
    pub sim_t: u64,
    pub step_index: u64,
    pub inputs: InputsSnapshot,
    /// 本步环境状态（与船同步，可视化精度）。
    pub environment: EnvironmentSnapshot,
    /// 本步被分离拆走的 vessel 下标（`Assembly.vessels` 下标）；仅分离步非空。
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub separated: Vec<u32>,
    /// 终局消息（坠毁 / 着陆 / 发射失败）；非终局步为空结构。
    #[serde(default)]
    pub terminal: TerminalInfo,
    /// 主组合体 + 全部分离体状态。
    pub bodies: Vec<VesselBody>,
}

/// 火箭标识。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RocketInfo {
    pub name: String,
    pub class: String,
    pub stage_count: u32,
}

/// 控制方式 / 配置摘要。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ControlInfo {
    /// `"base"` | `"target"` | `"workflow"`。
    pub mode: String,
    /// WorkFlow 图 IR 的 kind（target / super）；非工作流为 None。
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub workflow_kind: Option<String>,
    pub gravity_turn: bool,
    pub rcs_enabled: bool,
    pub throttle: f64,
}

/// 环境快照：星历时刻 + 全天体状态。`star` 仅 `session_start` 帧带（`Some`）。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EnvironmentSnapshot {
    pub mjd: f64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub star: Option<String>,
    pub bodies: Vec<BodyState>,
}

/// 单天体状态（可视化精度）。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BodyState {
    pub name: String,
    /// 惯性系位置 [m]（地心系场景下非主天体为相对位置，与 `PlanetarySystem` 同口径）。
    pub pos: [f64; 3],
    /// 自转角 [rad]（无自转为 0）。
    pub rotation_rad: f64,
    pub rotation_enabled: bool,
    pub dynamics: bool,
}

/// 刚体状态（与 `orbitx_math::StateVectors` 同构投影）。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RigidBodyState {
    pub pos: [f64; 3],
    pub vel: [f64; 3],
    /// 体轴→惯性，行主序 9。
    pub rot: [f64; 9],
    pub omega: [f64; 3],
    /// orbitx `Quat {vx, vy, vz, s}` 分量序。
    pub q: [f64; 4],
}

/// 航天器初始状态（`session_start` 帧）。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct VesselInitial {
    /// `Assembly.vessels` 下标。
    pub index: u32,
    pub name: String,
    pub detached: bool,
    pub state: RigidBodyState,
    pub fuel_kg: f64,
    pub initial_fuel_kg: f64,
}

/// 航天器步态（`step` 帧）：状态 + 全量诊断（回放零重算）。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct VesselBody {
    pub index: u32,
    pub name: String,
    pub detached: bool,
    pub crashed: bool,
    pub state: RigidBodyState,
    pub fuel_kg: f64,
    /// 本步实际推力占该级主推真空总推力比 ∈ [0,1]（与视觉/点火口径一致）。
    pub thrust_level: f64,
    pub diagnostics: DiagnosticsSnapshot,
}

/// 步级诊断快照（与 `FlightDiagnostics` 的浮点投影同构）。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DiagnosticsSnapshot {
    pub a_grav: f64,
    pub g_multiple: f64,
    pub mach: f64,
    pub density: f64,
    pub dynamic_pressure: f64,
    pub pressure: f64,
    pub thrust_atm_scale: f64,
    pub isp_eff: f64,
    pub drag_force: f64,
    pub cd_eff: f64,
    pub temperature: f64,
    pub sound_speed: f64,
    pub load_factor: f64,
}

/// 本步生效输入 / 会话命令摘要 + 节奏状态。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct InputsSnapshot {
    /// 本步飞行输入（Runtime 帧初 drain 生效者）。
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub flight: Vec<FlightInput>,
    /// 本步生效会话命令（不含 Shutdown / Reset：会话边界由 `session_start` 帧表达）。
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub session: Vec<SessionEvent>,
    pub warp: f64,
    pub paused: bool,
}

/// 终局消息；非终局步全部为 `None`。
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct TerminalInfo {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub crash_msg: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub landed_msg: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub launch_failed_msg: Option<String>,
}

impl TerminalInfo {
    pub fn is_terminal(&self) -> bool {
        self.crash_msg.is_some() || self.landed_msg.is_some() || self.launch_failed_msg.is_some()
    }
}

/// 飞行输入摘要。与 runtime `InputCmd` 同构但**格式自持**（黑匣子要读旧会话，
/// 不绑内存枚举版本）。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "op", rename_all = "snake_case")]
pub enum FlightInput {
    SetThrottle { level: f64 },
    SetAttitudeAxes { pitch: f64, yaw: f64, roll: f64 },
    Separate,
    SetGravityTurn { enabled: bool },
    WorkflowIgnite,
    SetRcsEnabled { enabled: bool },
}

/// 会话命令摘要。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "op", rename_all = "snake_case")]
pub enum SessionEvent {
    Pause,
    Resume,
    SetWarp { scale: f64 },
    Step { n: u32 },
    Launch,
    SetSurfaceWorkingMaxLevel { level: u32 },
}
