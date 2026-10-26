//! RuntimeService：固定步步进权威（`std::thread`）。

pub mod clock;
pub mod tick;

use std::sync::Arc;
use std::thread::{self, JoinHandle};
use std::time::Duration;

use flume::Receiver;
use tracing::{info, warn};

use crate::channel::{send_slice_keep_latest, RuntimeInbound, SliceBus};
use crate::cli::{DriveModeArg, LoadedSession};
use crate::input::SessionCmd;
use crate::recorder::RecorderEnqueue;
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

        let _ = self
            .recorder
            .try_enqueue_session_start(clock.sim_t_ms(), &self.sim.world);

        while !self.shutdown.is_requested() {
            let mut steps_this_pass = 0u32;

            while let Ok(msg) = self.cmd_rx.try_recv() {
                match msg {
                    RuntimeInbound::Session(SessionCmd::Shutdown) => {
                        self.shutdown.request();
                    }
                    RuntimeInbound::Session(SessionCmd::Pause) => clock.set_paused(true),
                    RuntimeInbound::Session(SessionCmd::Resume) => clock.set_paused(false),
                    RuntimeInbound::Session(SessionCmd::SetWarp { scale }) => {
                        clock.set_warp(scale);
                    }
                    RuntimeInbound::Session(SessionCmd::Step { n }) => {
                        if matches!(self.config.drive, DriveModeArg::ClientStep)
                            && gate_allows(self.gate_open, &mut self.gate_stepped_once)
                        {
                            steps_this_pass = steps_this_pass.saturating_add(n.max(1));
                        }
                    }
                    RuntimeInbound::Session(SessionCmd::Launch) => {
                        self.gate_open = true;
                        info!("launch gate opened: stepping begins");
                    }
                    RuntimeInbound::Session(SessionCmd::Reset) => {
                        match build_sim_bundle(&self.session, &self.ephemeris_data) {
                            Ok(sim) => {
                                self.sim = sim;
                                clock = Clock::new(self.config.sim_dt_ms);
                                // Reset 回预发射态：重关门禁 + 重置首步标记
                                //（Reset 后下一 Step 放行出新初始帧）。
                                self.gate_open = false;
                                self.gate_stepped_once = false;
                                info!("session reset");
                            }
                            Err(e) => warn!(error = %e, "reset failed"),
                        }
                    }
                    RuntimeInbound::Session(SessionCmd::SetSurfaceWorkingMaxLevel { level }) => {
                        if let Some(ref mut surf) = self.sim.surface {
                            surf.set_working_max_level(level);
                            info!(level = surf.working_max_level, "surface working_max_level updated");
                        } else {
                            warn!("SetSurfaceWorkingMaxLevel ignored (no surface session)");
                        }
                    }
                    RuntimeInbound::Input(cmd) => {
                        self.sim.control.apply_input(&mut self.sim.asm, cmd);
                    }
                }
            }

            if self.shutdown.is_requested() {
                break;
            }

            match self.config.drive {
                DriveModeArg::ClientStep => {
                    for _ in 0..steps_this_pass {
                        if self.shutdown.is_requested() {
                            break;
                        }
                        self.do_one_step(&mut clock);
                    }
                    thread::sleep(Duration::from_millis(1));
                }
                DriveModeArg::SelfPaced => {
                    if clock.paused() {
                        thread::sleep(Duration::from_millis(1));
                        continue;
                    }
                    self.do_one_step(&mut clock);
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

    fn do_one_step(&mut self, clock: &mut Clock) {
        if clock.paused() {
            return;
        }
        let outcome = tick::tick(clock, &mut self.sim, self.gate_open);
        match outcome {
            TickOutcome::Stepped { slice } => {
                let slice = Arc::new(slice);
                let _ = self.recorder.try_enqueue_step(slice.clone());
                send_slice_keep_latest(&self.slice_bus, slice);
            }
            TickOutcome::Skipped => warn!("tick skipped"),
        }
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
