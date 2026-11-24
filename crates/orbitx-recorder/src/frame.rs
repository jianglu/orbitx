//! 黑匣子帧类型（schema 2）。
//!
//! 字段全是 plain 类型 / f64 数组，与 runtime 的 `Assembly` / `Slice` 类型解耦：
//! 帧里记录的是**权威直录**（vessel state / diagnostics 原样），不是对外遥测投影。
//! 序列化为 CBOR map；`kind` 由 serde tag 写入。新增字段保持向后兼容
//! （serde 忽略未知字段），不兼容变更须升 [`SCHEMA_VERSION`]。
//!
//! 兼容铁律（C4）：schema 2 = schema 1 **纯超集**（仅增 `kind="scoring"` 评分
//! 快照帧）——读方对未知 `kind` 走 [`RecorderFrame::Unknown`] 跳过不报错；
//! 格式权威见 `docs/FLIGHT_RECORDER.md`。

use serde::{Deserialize, Serialize};

/// 当前磁盘格式版本（`FLIGHT_RECORDER.md`：不兼容变更必须递增）。
///
/// schema 2（C4）：新增 `kind="scoring"` 评分快照帧；读侧接受 ≤ 本值。
pub const SCHEMA_VERSION: u32 = 2;

/// 单帧：会话头帧、步帧或评分快照帧。
///
/// `Unknown` 捕获未知 `kind`（`#[serde(other)]`）：读方**跳过不报错**——
/// 旧 reader 读新会话只丢不认识的帧，其余功能完好（前向兼容铁律）。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind")]
pub enum RecorderFrame {
    #[serde(rename = "session_start")]
    SessionStart(SessionStartFrame),
    #[serde(rename = "step")]
    Step(StepFrame),
    /// 评分快照帧（C4）：停机收尾至多一帧（`HaltFlush` drain 后、`finish` 前）。
    #[serde(rename = "scoring")]
    Scoring(ScoringFrame),
    /// 未知 `kind` 兜底（仅读侧反序列化产生；写侧恒不产出）。
    #[serde(other)]
    Unknown,
}

impl RecorderFrame {
    pub fn sim_t(&self) -> u64 {
        match self {
            Self::SessionStart(f) => f.sim_t,
            Self::Step(f) => f.sim_t,
            Self::Scoring(f) => f.sim_t,
            // 未知帧 sim_t 未定义；恒 0 使范围查询（from > 0）天然排除。
            Self::Unknown => 0,
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
    /// 速览字段（本步新产生事件 vessels 并集）保留供旧读者；权威记录见
    /// `separations`（含产生步号/时间戳）。
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub separated: Vec<u32>,
    /// 分离事件权威记录（产生步一次性登记；schema 1 扩展字段，旧读者忽略）。
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub separations: Vec<SeparationRecord>,
    /// 终局消息（坠毁 / 着陆 / 发射失败）；非终局步为空结构。
    #[serde(default)]
    pub terminal: TerminalInfo,
    /// 主组合体 + 全部分离体状态。
    pub bodies: Vec<VesselBody>,
}

/// 单次分离事件（`separations` 元素，产生步一次性登记）：born_* 为事件产生
/// 物理步的步号/时间戳（权威直录，回放端按 born_step 对账倍速一致性）。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SeparationRecord {
    /// 事件产生物理步步号。
    pub born_step: u64,
    /// 事件产生物理步时间戳 [ms]。
    pub born_t_ms: u64,
    /// 被拆走的 vessel 下标（`Assembly.vessels` 口径）。
    pub vessels: Vec<u32>,
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
    SetThrottle {
        level: f64,
    },
    SetAttitudeAxes {
        pitch: f64,
        yaw: f64,
        roll: f64,
    },
    Separate,
    SetGravityTurn {
        enabled: bool,
    },
    WorkflowIgnite,
    SetRcsEnabled {
        enabled: bool,
    },
    /// 罗盘机头指向（方位 0=北顺时针 [rad]、仰角 [rad]）。旧录像无此 op，
    /// 追加变体向后兼容（serde tag="op" 逐 op 解码）。
    SetAttitudeNose {
        azimuth: f64,
        elevation: f64,
        roll: f64,
    },
}

/// 停机原因（C3b）：与 runtime / 线协议 `HaltReason` 口径一致（黑匣子格式
/// 自持副本——本 crate 无 orbitx 兄弟依赖），serde snake_case 落盘。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HaltReason {
    Unspecified,
    Completion,
    SimTimeExceeded,
    WallTimeExceeded,
    WorkflowHalt,
}

/// 会话命令摘要。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "op", rename_all = "snake_case")]
pub enum SessionEvent {
    Pause,
    Resume,
    SetWarp {
        scale: f64,
    },
    Step {
        n: u32,
    },
    Launch,
    SetSurfaceWorkingMaxLevel {
        level: u32,
    },
    /// 按参数请求停机收尾（C3b）：判定器判成 / 超时监测触发。
    HaltRequest {
        reason: HaltReason,
    },
}

/// `kind = "scoring"` 评分快照帧（C4 方案 A）：停机收尾至多一帧，评分与
/// 轨道复核的**权威直录**（消费方零换算）。数值口径与消费契约见 sim-rocket
/// `COMPETITION_RULES.md` §6；格式权威见 `docs/FLIGHT_RECORDER.md`。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ScoringFrame {
    /// 停机收尾时刻 [ms]。
    pub sim_t: u64,
    /// 会话初始装载燃料总和 [kg]（kf 分子；与发射前检查同出 sim.toml 单源）。
    pub loaded_fuel_kg: f64,
    /// 关机事件序列（armed 后「有推力→无推力」下降沿全量；边沿只看活动体
    /// （未 detached）**主推进器**，detached 级与 RCS 等追加组不参与检测）。
    pub cutoffs: Vec<CutoffRecord>,
    /// 首次点火（armed 后首个推力上升沿）时刻 [ms]；未点火 = `None`。
    /// 兼容超集字段（2026-11-23 任务一评分锚历元修正）：评分器据此把关机
    /// 体固经纬反旋回点火历元（Δt = 关机 − 点火），与惯性冻结的入轨点锚
    /// 天球系对齐；旧档案缺字段 = 未点火/旧版，消费方按 `None` 走旧行为。
    #[serde(default)]
    pub first_burn_sim_t_ms: Option<u64>,
    /// 停机时刻实测形状要素（**仅作形状要素锚**：pe/ap/inc/raan/argp 为
    /// 二体常量；不含真近点角/幅角类时变量）。
    pub halt_state: OrbitElements,
}

/// 单条关机事件：五要素 + 关机点轨道要素，关机瞬间权威直录。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CutoffRecord {
    /// 关机时刻 [ms]。
    pub sim_t: u64,
    /// 体固系 geocentric 纬度 [deg]（同 HUD）。
    pub lat_deg: f64,
    /// 体固系 geocentric 经度 [deg]（同 HUD）。
    pub lon_deg: f64,
    /// 海拔 [m]（\|r\| − R_elev）。
    pub alt_m: f64,
    /// 惯性地心速 \|v\| [m/s]。
    pub speed_ms: f64,
    /// 飞行路径角 asin(v·r̂/\|v\|) [deg]。
    pub gamma_deg: f64,
    /// 关机点轨道近地点高度 [m]（`orbitx-math::Elements` 口径；e ≥ 1 时 0）。
    pub pe_alt_m: f64,
    /// 关机点轨道远地点高度 [m]（e ≥ 1 = 逃逸时 `INFINITY`，wire 投影为 null）。
    pub ap_alt_m: f64,
    /// 倾角 [deg]。
    pub inc_deg: f64,
    /// 升交点赤经 [deg]。
    pub raan_deg: f64,
    /// 近地点幅角 [deg]。
    pub argp_deg: f64,
    /// 纬度幅角 [deg]（与切片 `arg_latitude_deg` 同源，圈判定对拍锚）。
    pub arg_lat_deg: f64,
    /// 停机补记标记（2026-11-23 评分点口径切换）：`true` = 停机收尾补记的
    /// 定格时刻快照（「停机即关机」，= 玩家所见冻结态；燃烧中停机与滑行
    /// 停机都补）——任务一评分点消费此条目；任务二 D 变轨耗时排除之（防
    /// 把滑行/绕圈时长算进变轨）。旧档案缺字段回落 `false`（前向兼容）。
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub at_halt: bool,
}

/// 形状要素集（`halt_state`；与 [`CutoffRecord`] 的要素六元组同构）。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct OrbitElements {
    /// 近地点高度 [m]。
    pub pe_alt_m: f64,
    /// 远地点高度 [m]。
    pub ap_alt_m: f64,
    /// 倾角 [deg]。
    pub inc_deg: f64,
    /// 升交点赤经 [deg]。
    pub raan_deg: f64,
    /// 近地点幅角 [deg]。
    pub argp_deg: f64,
    /// 纬度幅角 [deg]。
    pub arg_lat_deg: f64,
}
