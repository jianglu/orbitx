//! 完整黑匣子帧构建：`Assembly` / `PlanetarySystem` → `orbitx_recorder::RecorderFrame`。
//!
//! 本模块是 runtime→帧的**唯一**转换落点：环境快照单一函数（session_start 与
//! step 帧共用）；诊断为 `FlightDiagnostics` **权威直录**（与 Slice 遥测投影是
//! 两条口径链，禁止合并）；`thrust_level` 复用 tick 的视觉/点火口径。

use orbitx_controller::workflow::WorkFlowKind;
use orbitx_environment::PlanetarySystem;
use orbitx_math::StateVectors;
use orbitx_recorder::frame::{
    BodyState, ControlInfo, DiagnosticsSnapshot, EnvironmentSnapshot, FlightInput, InputsSnapshot,
    RecorderFrame, RigidBodyState, RocketInfo, SessionEvent, SessionStartFrame, StepFrame,
    TerminalInfo, VesselBody, VesselInitial, SCHEMA_VERSION,
};
use orbitx_vessel::{Assembly, FlightDiagnostics};

use crate::cli::{ControlKindArg, LoadedSession, SessionControl};
use crate::input::{InputCmd, SessionCmd};
use crate::runtime::clock::Clock;
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

/// `step` 帧（tick 完成后调用：状态为 T1，clock 已推进）。
pub fn step_frame(
    clock: &Clock,
    sim: &SimBundle,
    flight: &[InputCmd],
    session_cmds: &[SessionCmd],
    separated: &[u32],
) -> RecorderFrame {
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
        separated: separated.to_vec(),
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
        SessionCmd::Shutdown | SessionCmd::Reset => None,
    }
}
