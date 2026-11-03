//! RuntimeService：固定步步进权威（`std::thread`）。

pub mod clock;
pub mod tick;

use std::sync::Arc;
use std::thread::{self, JoinHandle};
use std::time::Duration;

use flume::Receiver;
use tracing::{error, info, warn};

use crate::channel::{send_slice_keep_latest, RuntimeInbound, SliceBus};
use crate::cli::{DriveModeArg, LoadedSession};
use crate::input::{InputCmd, SessionCmd};
use crate::recorder::{frame, RecorderEnqueue};
use crate::session::{build_sim_bundle, SimBundle};
use crate::shutdown::ShutdownFlag;

use self::clock::Clock;
use self::tick::TickOutcome;

pub struct RuntimeServiceConfig {
    pub sim_dt_ms: u64,
    pub drive: DriveModeArg,
}

pub struct RuntimeService {
    shutdown: ShutdownFlag,
    cmd_rx: Receiver<RuntimeInbound>,
    slice_bus: SliceBus,
    recorder: RecorderEnqueue,
    config: RuntimeServiceConfig,
    sim: SimBundle,
    /// 用于 Reset 重建。
    session: LoadedSession,
    ephemeris_data: std::path::PathBuf,
    /// 发射门禁：true = 已解禁正常步进。client-step 预发射 hold。
    gate_open: bool,
    /// 首步是否已放行（hold 期仍放行第 1 步出初始切片）。
    gate_stepped_once: bool,
    /// ClientStep 待步数：Recorder 水位未降时保留（推迟不丢步，C1）。
    pending_steps: u64,
    /// Recorder IO 失败后已上报（一次性 fatal，不刷屏）。
    recorder_halted: bool,
}

impl RuntimeService {
    pub fn spawn(self) -> JoinHandle<()> {
        thread::Builder::new()
            .name("orbitx-runtime".into())
            .spawn(move || self.run_loop())
            .expect("spawn RuntimeService")
    }

    fn run_loop(mut self) {
        let mut clock = Clock::new(self.config.sim_dt_ms);
        let rocket = self.sim.world.rocket_name.clone();
        let class = self.sim.world.rocket_class.clone();
        info!(
            drive = ?self.config.drive,
            sim_dt_ms = self.config.sim_dt_ms,
            rocket = %rocket,
            class = %class,
            control = self.sim.control.label(),
            "RuntimeService started"
        );

        // B3：session_start 是会话第一帧，必须入队（队满则等待重试；disabled 恒放行）。
        while !self.enqueue_session_start(&clock) {
            if self.shutdown.is_requested() {
                return;
            }
            thread::sleep(Duration::from_millis(1));
        }

        while !self.shutdown.is_requested() {
            // 本 pass 收集的命令快照：随本 pass 首帧入黑匣子。
            let mut pass_flight: Vec<InputCmd> = Vec::new();
            let mut pass_session: Vec<SessionCmd> = Vec::new();

            while let Ok(msg) = self.cmd_rx.try_recv() {
                match msg {
                    RuntimeInbound::Session(SessionCmd::Shutdown) => {
                        self.shutdown.request();
                    }
                    RuntimeInbound::Session(SessionCmd::Pause) => {
                        pass_session.push(SessionCmd::Pause);
                        clock.set_paused(true);
                    }
                    RuntimeInbound::Session(SessionCmd::Resume) => {
                        pass_session.push(SessionCmd::Resume);
                        clock.set_paused(false);
                    }
                    RuntimeInbound::Session(SessionCmd::SetWarp { scale }) => {
                        pass_session.push(SessionCmd::SetWarp { scale });
                        clock.set_warp(scale);
                    }
                    RuntimeInbound::Session(SessionCmd::Step { n }) => {
                        pass_session.push(SessionCmd::Step { n });
                        if matches!(self.config.drive, DriveModeArg::ClientStep)
                            && gate_allows(self.gate_open, &mut self.gate_stepped_once)
                        {
                            self.pending_steps =
                                self.pending_steps.saturating_add(u64::from(n.max(1)));
                        }
                    }
                    RuntimeInbound::Session(SessionCmd::Launch) => {
                        pass_session.push(SessionCmd::Launch);
                        self.gate_open = true;
                        info!("launch gate opened: stepping begins");
                    }
                    RuntimeInbound::Session(SessionCmd::Reset) => {
                        // B4：重建失败不开新会话目录；成功 = 新会话（新 session_start）。
                        match build_sim_bundle(&self.session, &self.ephemeris_data) {
                            Ok(sim) => {
                                self.sim = sim;
                                clock = Clock::new(self.config.sim_dt_ms);
                                // Reset 回预发射态：重关门禁 + 重置首步标记
                                //（Reset 后下一 Step 放行出新初始帧）。
                                self.gate_open = false;
                                self.gate_stepped_once = false;
                                self.pending_steps = 0;
                                while !self.enqueue_session_start(&clock) {
                                    if self.shutdown.is_requested() {
                                        break;
                                    }
                                    thread::sleep(Duration::from_millis(1));
                                }
                                info!("session reset: new recorder session opened");
                            }
                            Err(e) => warn!(error = %e, "reset failed"),
                        }
                    }
                    RuntimeInbound::Session(SessionCmd::SetSurfaceWorkingMaxLevel { level }) => {
                        pass_session.push(SessionCmd::SetSurfaceWorkingMaxLevel { level });
                        if let Some(ref mut surf) = self.sim.surface {
                            surf.set_working_max_level(level);
                            info!(
                                level = surf.working_max_level,
                                "surface working_max_level updated"
                            );
                        } else {
                            warn!("SetSurfaceWorkingMaxLevel ignored (no surface session)");
                        }
                    }
                    RuntimeInbound::Input(cmd) => {
                        pass_flight.push(cmd.clone());
                        self.sim.control.apply_input(&mut self.sim.asm, cmd);
                    }
                }
            }

            if self.shutdown.is_requested() {
                break;
            }

            // 命令归组只挂本 pass 首步；后续步不带（`take` 语义）。
            let mut first_inputs = Some((pass_flight, pass_session));

            match self.config.drive {
                DriveModeArg::ClientStep => {
                    while self.pending_steps > 0 {
                        if self.shutdown.is_requested() {
                            break;
                        }
                        if !self.recorder.healthy() {
                            self.report_recorder_failure();
                            break;
                        }
                        if !self.recorder.accepts() {
                            break; // C1：水位未降，保留 pending_steps 推迟
                        }
                        let (flight, session) = first_inputs
                            .take()
                            .unwrap_or_else(|| (Vec::new(), Vec::new()));
                        if self.do_one_step(&mut clock, &flight, &session) {
                            self.pending_steps -= 1;
                        } else {
                            break; // paused 等：pending 保留，Resume 后继续
                        }
                    }
                    thread::sleep(Duration::from_millis(1));
                }
                DriveModeArg::SelfPaced => {
                    if clock.paused() {
                        thread::sleep(Duration::from_millis(1));
                        continue;
                    }
                    if !self.recorder.healthy() {
                        self.report_recorder_failure();
                        thread::sleep(Duration::from_millis(10));
                        continue;
                    }
                    if !self.recorder.accepts() {
                        // C1：跳拍等待，不丢帧。
                        thread::sleep(Duration::from_millis(1));
                        continue;
                    }
                    let (flight, session) = first_inputs
                        .take()
                        .unwrap_or_else(|| (Vec::new(), Vec::new()));
                    self.do_one_step(&mut clock, &flight, &session);
                    let sleep_ms = ((self.config.sim_dt_ms as f64) / clock.warp().max(1e-6))
                        .round()
                        .max(1.0) as u64;
                    thread::sleep(Duration::from_millis(sleep_ms));
                }
            }
        }

        info!(
            sim_t_ms = clock.sim_t_ms(),
            "RuntimeService stopping after full steps"
        );
    }

    /// 构建 session_start 帧并入队；false = 队满需重试（B3）。disabled 恒 true。
    fn enqueue_session_start(&self, clock: &Clock) -> bool {
        if !self.recorder.is_enabled() {
            return true;
        }
        let f = frame::session_start_frame(
            clock.sim_t_ms(),
            self.config.sim_dt_ms,
            &self.session,
            &self.sim.control,
            &self.sim.world,
            &self.sim.asm,
            &self.sim.initial_fuel,
        );
        self.recorder.try_enqueue(f)
    }

    /// N1：IO 写盘失败 → 暂停积步（一次性显式失败）。
    fn report_recorder_failure(&mut self) {
        if !self.recorder_halted {
            self.recorder_halted = true;
            error!("FlightRecorder IO failed: stepping halted (frames cannot be persisted)");
        }
    }

    /// 单步。返回 false = 推迟（paused / IO 失败），调用方保留待步计数；
    /// 返回 true = tick 已完成（含 gate hold 的 Skipped，消耗待步数）。
    fn do_one_step(
        &mut self,
        clock: &mut Clock,
        flight: &[InputCmd],
        session_cmds: &[SessionCmd],
    ) -> bool {
        if clock.paused() {
            return false;
        }
        if !self.recorder.healthy() {
            self.report_recorder_failure();
            return false;
        }
        let outcome = tick::tick(clock, &mut self.sim, self.gate_open);
        match outcome {
            TickOutcome::Stepped { slice } => {
                let frame = frame::step_frame(
                    clock,
                    &self.sim,
                    flight,
                    session_cmds,
                    &slice.separated_vessels,
                );
                if !self.recorder.try_enqueue(frame) {
                    // C1 前置检查后的竞态兜底：本帧丢失（不阻塞步进）。
                    warn!("recorder L1 full at enqueue (race); frame dropped");
                }
                send_slice_keep_latest(&self.slice_bus, Arc::new(slice));
            }
            TickOutcome::Skipped => warn!("tick skipped"),
        }
        true
    }
}

pub fn build_runtime_service(
    shutdown: ShutdownFlag,
    cmd_rx: Receiver<RuntimeInbound>,
    slice_bus: SliceBus,
    recorder: RecorderEnqueue,
    config: RuntimeServiceConfig,
    sim: SimBundle,
    session: LoadedSession,
    ephemeris_data: std::path::PathBuf,
) -> RuntimeService {
    RuntimeService {
        shutdown,
        cmd_rx,
        slice_bus,
        recorder,
        config,
        sim,
        session,
        ephemeris_data,
        gate_open: false,
        gate_stepped_once: false,
        pending_steps: 0,
        recorder_halted: false,
    }
}

/// 发射门禁决策（预发射 hold）：Launch 前只放行首步（出初始切片，之后客户端
/// 依切片 `stepping=false` 停发 Step）；Launch 后全放行。放行时置首步标记。
pub(crate) fn gate_allows(gate_open: bool, stepped_once: &mut bool) -> bool {
    let allow = gate_open || !*stepped_once;
    if allow {
        *stepped_once = true;
    }
    allow
}

#[cfg(test)]
mod tests;
