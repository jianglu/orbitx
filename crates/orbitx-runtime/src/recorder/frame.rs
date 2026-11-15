//! 完整黑匣子帧构建：`Assembly` / `PlanetarySystem` → `orbitx_recorder::RecorderFrame`。
//!
//! 本模块是 runtime→帧的**唯一**转换落点：环境快照单一函数（session_start 与
//! step 帧共用）；诊断为 `FlightDiagnostics` **权威直录**（与 Slice 遥测投影是
//! 两条口径链，禁止合并）；`thrust_level` 复用 tick 的视觉/点火口径。

use orbitx_controller::workflow::WorkFlowKind;
use orbitx_environment::PlanetarySystem;
use orbitx_math::{dot, Elements, StateVectors, Vec3, GGRAV};
use orbitx_recorder::frame::{
    BodyState, ControlInfo, CutoffRecord, DiagnosticsSnapshot, EnvironmentSnapshot, FlightInput,
    HaltReason as SessionHaltReason, InputsSnapshot, OrbitElements, RecorderFrame, RigidBodyState,
    RocketInfo, ScoringFrame, SeparationRecord, SessionEvent, SessionStartFrame, StepFrame,
    TerminalInfo, VesselBody, VesselInitial, SCHEMA_VERSION,
};
use orbitx_vessel::{geocentric_lat_lon_deg_orient, Assembly, FlightDiagnostics};

use crate::cli::{ControlKindArg, LoadedSession, SessionControl};
use crate::input::{InputCmd, SessionCmd};
use crate::runtime::clock::Clock;
use crate::runtime::separation_queue::{born_t_ms_of, SeparationEvent};
use crate::runtime::tick::visual_throttle;
use crate::session::SimBundle;
use crate::world::World;

/// 环境快照（session_start 与 step 共用；`with_star` 仅会话头帧为 true）。
pub fn environment_snapshot(psys: &PlanetarySystem, with_star: bool) -> EnvironmentSnapshot {
    EnvironmentSnapshot {
        mjd: psys.mjd,
        star: with_star.then(|| psys.star().to_string()),
        bodies: psys
            .bodies()
            .iter()
            .map(|b| BodyState {
                name: b.name.clone(),
                pos: [b.pos.x, b.pos.y, b.pos.z],
                rotation_rad: b.rotation.as_ref().map_or(0.0, |r| r.rotation()),
                rotation_enabled: b.rotation_enabled,
                dynamics: b.dynamics,
            })
            .collect(),
    }
}

/// `session_start` 帧（启动 / Reset 成功后各一帧，各开新会话目录）。
pub fn session_start_frame(
    sim_t: u64,
    sim_dt_ms: u64,
    session: &LoadedSession,
    control: &crate::session::ActiveControl,
    world: &World,
    asm: &Assembly,
    initial_fuel: &[f64],
) -> RecorderFrame {
    RecorderFrame::SessionStart(SessionStartFrame {
        schema_version: SCHEMA_VERSION,
        sim_t,
        sim_dt_ms,
        rocket: RocketInfo {
            name: session.rocket.name.clone(),
            class: session.rocket.class.clone(),
            stage_count: asm.stage_count() as u32,
        },
        control: control_info(session, control),
        environment: environment_snapshot(&world.psys, true),
        vessels: asm
            .vessels
            .iter()
            .enumerate()
            .map(|(i, v)| VesselInitial {
                index: i as u32,
                name: v.name.clone(),
                detached: v.detached,
                state: rigid_state(&v.state),
                fuel_kg: v.fuel_mass(),
                initial_fuel_kg: initial_fuel
                    .get(i)
                    .copied()
                    .unwrap_or_else(|| v.fuel_mass()),
            })
            .collect(),
    })
}

/// `step` 帧（tick 完成后调用：状态为 T1，clock 已推进）。`new_separations`
/// = 本步新产生的分离事件（**产生步一次性**记录：born_step / born_t_ms 权威
/// 直录，不随切片 TTL 重发扩散；旧字段 `separated` 填本步产生 vessels 并集
/// 供旧读者）。
pub fn step_frame(
    clock: &Clock,
    sim: &SimBundle,
    flight: &[InputCmd],
    session_cmds: &[SessionCmd],
    new_separations: &[SeparationEvent],
) -> RecorderFrame {
    let separations: Vec<SeparationRecord> = new_separations
        .iter()
        .map(|e| SeparationRecord {
            born_step: e.born_step,
            born_t_ms: born_t_ms_of(e.born_step, clock),
            vessels: e.vessels.iter().map(|&i| i as u32).collect(),
        })
        .collect();
    let mut separated: Vec<u32> = Vec::new();
    for rec in &separations {
        for &vi in &rec.vessels {
            if !separated.contains(&vi) {
                separated.push(vi);
            }
        }
    }
    separated.sort_unstable();
    RecorderFrame::Step(StepFrame {
        sim_t: clock.sim_t_ms(),
        step_index: clock.step_index(),
        inputs: InputsSnapshot {
            flight: flight.iter().map(flight_input).collect(),
            session: session_cmds.iter().filter_map(session_event).collect(),
            warp: clock.warp(),
            paused: clock.paused(),
        },
        environment: environment_snapshot(&sim.world.psys, false),
        separated,
        separations,
        terminal: TerminalInfo {
            crash_msg: (!sim.crash_msg.is_empty()).then(|| sim.crash_msg.clone()),
            landed_msg: (!sim.landed_msg.is_empty()).then(|| sim.landed_msg.clone()),
            launch_failed_msg: (!sim.launch_failed_msg.is_empty())
                .then(|| sim.launch_failed_msg.clone()),
        },
        bodies: sim
            .asm
            .vessels
            .iter()
            .enumerate()
            .map(|(i, v)| VesselBody {
                index: i as u32,
                name: v.name.clone(),
                detached: v.detached,
                crashed: v.crashed,
                state: rigid_state(&v.state),
                fuel_kg: v.fuel_mass(),
                thrust_level: visual_throttle(v).0,
                diagnostics: diagnostics_snapshot(&v.diagnostics),
            })
            .collect(),
    })
}

// —— 评分快照数据链（C4 方案 A）——
//
// 运行期 cutoff 跟踪（每步一次阈值比较，不进步进热路径开销）+ 停机收尾
// 组装 `kind="scoring"` 帧。数值口径与消费契约见 sim-rocket
// `COMPETITION_RULES.md` §6；写入时序见 `docs/RUNTIME.md`「停机收尾」。

/// 关机事件跟踪器（每会话内存态；Reset 清空）。
///
/// 语义与 Godot `CompletionJudge` 同源：armed = 经历过点火上升沿；下降沿 =
/// armed 后「有推力→无推力」。推力判定（0.02 阈、只看未 detached 级）由
/// 调用方经 [`scoring_thrusting`] 判定，本结构只管边沿与 armed 门槛。
#[derive(Debug, Default)]
pub(crate) struct CutoffTracker {
    burning: bool,
    armed: bool,
    cutoffs: Vec<CutoffRecord>,
}

impl CutoffTracker {
    pub(crate) fn new() -> Self {
        Self::default()
    }

    /// 每步喂活动体推力态。返回 true = 本步为 armed 后下降沿（调用方构建
    /// [`CutoffRecord`] 并 [`Self::push`] 补记）。
    pub(crate) fn observe(&mut self, thrusting: bool) -> bool {
        let falling = !thrusting && self.burning && self.armed;
        if thrusting && !self.burning {
            self.armed = true;
        }
        self.burning = thrusting;
        falling
    }

    pub(crate) fn push(&mut self, rec: CutoffRecord) {
        self.cutoffs.push(rec);
    }

    /// 当前已记录关机事件（诊断/测试查看；消耗取走用 [`Self::into_cutoffs`]）。
    pub(crate) fn cutoffs(&self) -> &[CutoffRecord] {
        &self.cutoffs
    }

    pub(crate) fn into_cutoffs(self) -> Vec<CutoffRecord> {
        self.cutoffs
    }

    pub(crate) fn reset(&mut self) {
        *self = Self::new();
    }
}

/// 活动体推力态（cutoff 边沿判定唯一口径）：任一**未 detached** 级处于
/// 点火推力（`visual_throttle` firing = ratio > 0.02，与焰效/点火显示及
/// Godot `CompletionJudge` THRUST_EPS 同源）；detached 级不参与。
pub(crate) fn scoring_thrusting(asm: &Assembly) -> bool {
    asm.vessels
        .iter()
        .filter(|v| !v.detached)
        .any(|v| visual_throttle(v).1)
}

/// 关机点快照（关机瞬间活动体状态权威直录）：focus 口径取活动体船原点
/// pos/vel（与 `build_focus_telem` 一致，禁质心混入）。
pub(crate) fn cutoff_record(sim_t: u64, sim: &SimBundle) -> CutoffRecord {
    let asm = &sim.asm;
    let vi = asm.active.min(asm.vessels.len().saturating_sub(1));
    let v = &asm.vessels[vi];
    let pos = v.state.pos;
    let vel = v.state.vel;
    let r_mag = pos.length().max(1e-3);
    let radial = pos * (1.0 / r_mag);
    let speed = vel.length();
    let gamma_deg = if speed > 1e-9 {
        (dot(vel, radial) / speed).clamp(-1.0, 1.0).asin().to_degrees()
    } else {
        0.0
    };
    let (lat_deg, lon_deg) = geocentric_lat_lon_deg_orient(pos, asm.planet_orientation.rot);
    let mu = GGRAV * sim.world.psys.primary_surface().mass;
    let el = orbit_elements_of(pos, vel, mu, sim.earth_radius);
    CutoffRecord {
        sim_t,
        lat_deg,
        lon_deg,
        alt_m: r_mag - sim.earth_radius,
        speed_ms: speed,
        gamma_deg,
        pe_alt_m: el.pe_alt_m,
        ap_alt_m: el.ap_alt_m,
        inc_deg: el.inc_deg,
        raan_deg: el.raan_deg,
        argp_deg: el.argp_deg,
        arg_lat_deg: el.arg_lat_deg,
    }
}

/// 轨道形状要素直算（`orbitx-math::Elements` 权威口径，写侧禁近似）。
/// pe/ap 与切片 `build_slice` 同式：e < 1 用 a(1∓e)；e ≥ 1（逃逸）pe 取
/// 保护值 0（必坠语义，gate 判定恒关）、ap = `INFINITY`（wire 投影 null，
/// 消费方按偏差最大处理）。
pub(crate) fn orbit_elements_of(pos: Vec3, vel: Vec3, mu: f64, earth_radius: f64) -> OrbitElements {
    let el = Elements::calculate(pos, vel, mu, 0.0);
    let (pe_r, ap_r) = if el.e < 1.0 {
        (el.a * (1.0 - el.e), el.a * (1.0 + el.e))
    } else {
        (0.0, f64::INFINITY)
    };
    OrbitElements {
        pe_alt_m: pe_r - earth_radius,
        ap_alt_m: ap_r - earth_radius,
        inc_deg: el.i.to_degrees(),
        raan_deg: el.theta.to_degrees(),
        argp_deg: el.arg_per().to_degrees(),
        arg_lat_deg: el.arg_latitude().to_degrees(),
    }
}

/// `kind = "scoring"` 评分快照帧（停机收尾至多一帧）：`sim_t` 由调用方传
/// （最后切片时刻；未步进过 = 0），装载 = 会话初始装载总和，`halt_state` =
/// 收尾时刻活动体要素直算。
pub(crate) fn scoring_frame(
    sim_t: u64,
    sim: &SimBundle,
    cutoffs: Vec<CutoffRecord>,
) -> RecorderFrame {
    let asm = &sim.asm;
    let vi = asm.active.min(asm.vessels.len().saturating_sub(1));
    let focus = &asm.vessels[vi];
    let mu = GGRAV * sim.world.psys.primary_surface().mass;
    RecorderFrame::Scoring(ScoringFrame {
        sim_t,
        loaded_fuel_kg: sim.initial_fuel.iter().sum(),
        cutoffs,
        halt_state: orbit_elements_of(focus.state.pos, focus.state.vel, mu, sim.earth_radius),
    })
}

fn rigid_state(s: &StateVectors) -> RigidBodyState {
    RigidBodyState {
        pos: [s.pos.x, s.pos.y, s.pos.z],
        vel: [s.vel.x, s.vel.y, s.vel.z],
        rot: [
            s.r.m11, s.r.m12, s.r.m13, s.r.m21, s.r.m22, s.r.m23, s.r.m31, s.r.m32, s.r.m33,
        ],
        omega: [s.omega.x, s.omega.y, s.omega.z],
        q: [s.q.vx, s.q.vy, s.q.vz, s.q.s],
    }
}

/// `FlightDiagnostics` → 帧诊断（权威直录，env 类 13 项；姿态在 state 中）。
fn diagnostics_snapshot(d: &FlightDiagnostics) -> DiagnosticsSnapshot {
    DiagnosticsSnapshot {
        a_grav: d.a_grav,
        g_multiple: d.g_multiple,
        mach: d.mach,
        density: d.density,
        dynamic_pressure: d.dynamic_pressure,
        pressure: d.pressure,
        thrust_atm_scale: d.thrust_atm_scale,
        isp_eff: d.isp_eff,
        drag_force: d.drag_force,
        cd_eff: d.cd_eff,
        temperature: d.temperature,
        sound_speed: d.sound_speed,
        load_factor: d.load_factor,
    }
}

/// 控制摘要（mode 用闭集词，不用展示标签）。
fn control_info(session: &LoadedSession, control: &crate::session::ActiveControl) -> ControlInfo {
    let (mode, workflow_kind) = match &session.control {
        SessionControl::Control(ControlKindArg::Base) => ("base", None),
        SessionControl::Control(ControlKindArg::Target) => ("target", None),
        SessionControl::WorkFlow { desc, .. } => ("workflow", Some(workflow_kind_str(desc.kind))),
        SessionControl::Graph { ir, .. } => ("workflow", Some(workflow_kind_str(ir.kind))),
    };
    let (rcs_enabled, _) = control.rcs_state();
    ControlInfo {
        mode: mode.into(),
        workflow_kind: workflow_kind.map(str::to_owned),
        gravity_turn: control.gravity_turn_enabled(),
        rcs_enabled,
        throttle: control.throttle_cmd(),
    }
}

fn workflow_kind_str(kind: WorkFlowKind) -> &'static str {
    match kind {
        WorkFlowKind::Target => "target",
        WorkFlowKind::Super => "super",
    }
}

/// 飞行输入 → 帧摘要枚举（格式自持，不绑内存枚举）。
pub(crate) fn flight_input(cmd: &InputCmd) -> FlightInput {
    match cmd {
        InputCmd::SetThrottle { level } => FlightInput::SetThrottle { level: *level },
        InputCmd::SetAttitudeAxes { pitch, yaw, roll } => FlightInput::SetAttitudeAxes {
            pitch: *pitch,
            yaw: *yaw,
            roll: *roll,
        },
        InputCmd::Separate => FlightInput::Separate,
        InputCmd::SetGravityTurn { enabled } => FlightInput::SetGravityTurn { enabled: *enabled },
        InputCmd::WorkflowIgnite => FlightInput::WorkflowIgnite,
        InputCmd::SetRcsEnabled { enabled } => FlightInput::SetRcsEnabled { enabled: *enabled },
        InputCmd::SetAttitudeNose {
            azimuth,
            elevation,
            roll,
        } => FlightInput::SetAttitudeNose {
            azimuth: *azimuth,
            elevation: *elevation,
            roll: *roll,
        },
    }
}

/// 会话命令 → 帧摘要；`Shutdown` / `Reset` 不入帧（会话边界由 session_start 表达）。
pub(crate) fn session_event(cmd: &SessionCmd) -> Option<SessionEvent> {
    match cmd {
        SessionCmd::Pause => Some(SessionEvent::Pause),
        SessionCmd::Resume => Some(SessionEvent::Resume),
        SessionCmd::SetWarp { scale } => Some(SessionEvent::SetWarp { scale: *scale }),
        SessionCmd::Step { n } => Some(SessionEvent::Step { n: *n }),
        SessionCmd::Launch => Some(SessionEvent::Launch),
        SessionCmd::SetSurfaceWorkingMaxLevel { level } => {
            Some(SessionEvent::SetSurfaceWorkingMaxLevel { level: *level })
        }
        SessionCmd::HaltRequest { reason } => Some(SessionEvent::HaltRequest {
            reason: match reason {
                crate::input::HaltReason::Unspecified => SessionHaltReason::Unspecified,
                crate::input::HaltReason::Completion => SessionHaltReason::Completion,
                crate::input::HaltReason::SimTimeExceeded => SessionHaltReason::SimTimeExceeded,
                crate::input::HaltReason::WallTimeExceeded => SessionHaltReason::WallTimeExceeded,
                crate::input::HaltReason::WorkflowHalt => SessionHaltReason::WorkflowHalt,
            },
        }),
        SessionCmd::Shutdown | SessionCmd::Reset => None,
        // Configure 仅空载期有效（host 消费），不会随步进帧入黑匣子；防御性
        // 到达也不入帧（会话配置由 session_start 帧表达）。
        SessionCmd::Configure { .. } => None,
    }
}
