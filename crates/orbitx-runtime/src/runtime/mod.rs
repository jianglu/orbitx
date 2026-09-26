//! RuntimeService：固定步步进权威（`std::thread`）。

pub mod clock;
pub mod tick;

use std::sync::Arc;
use std::thread::{self, JoinHandle};
use std::time::Duration;

use flume::{Receiver, Sender};
use tracing::{debug, info, warn};

use crate::channel::{send_slice_keep_latest, RuntimeInbound};
use crate::cli::DriveModeArg;
use crate::input::SessionCmd;
use crate::recorder::RecorderEnqueue;
use crate::session::SimBundle;
use crate::shutdown::ShutdownFlag;
use crate::slice::Slice;
use crate::world::World;

use self::clock::Clock;
use self::tick::TickOutcome;

pub struct RuntimeServiceConfig {
    pub sim_dt_ms: u64,
    pub drive: DriveModeArg,
}

pub struct RuntimeService {
    shutdown: ShutdownFlag,
    cmd_rx: Receiver<RuntimeInbound>,
    slice_tx: Sender<Arc<Slice>>,
    recorder: RecorderEnqueue,
    config: RuntimeServiceConfig,
    sim: SimBundle,
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
        let world = World::with_rocket(
            self.sim.rocket_name.clone(),
            self.sim.rocket_class.clone(),
        );
        info!(
            drive = ?self.config.drive,
            sim_dt_ms = self.config.sim_dt_ms,
            rocket = %world.rocket_name,
            class = %world.rocket_class,
            control = self.sim.control.label(),
            "RuntimeService started"
        );

        let _ = self.recorder.try_enqueue_session_start(clock.sim_t_ms(), &world);

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
                        if matches!(self.config.drive, DriveModeArg::ClientStep) {
                            steps_this_pass = steps_this_pass.saturating_add(n.max(1));
                        }
                    }
                    RuntimeInbound::Input(_cmd) => {
                        debug!("InputCmd received (stub; Base mode / future Zenoh)");
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
                    thread::sleep(Duration::from_millis(self.config.sim_dt_ms.max(1)));
                }
            }
        }

        info!(sim_t_ms = clock.sim_t_ms(), "RuntimeService stopping after full steps");
    }

    fn do_one_step(&mut self, clock: &mut Clock) {
        if clock.paused() {
            return;
        }
        let outcome = tick::tick(clock, &mut self.sim);
        match outcome {
            TickOutcome::Stepped { slice } => {
                let slice = Arc::new(slice);
                let _ = self.recorder.try_enqueue_step(slice.clone());
                send_slice_keep_latest(&self.slice_tx, slice);
            }
            TickOutcome::Skipped => warn!("tick skipped"),
        }
    }
}

pub fn build_runtime_service(
    shutdown: ShutdownFlag,
    cmd_rx: Receiver<RuntimeInbound>,
    slice_tx: Sender<Arc<Slice>>,
    recorder: RecorderEnqueue,
    config: RuntimeServiceConfig,
    sim: SimBundle,
) -> RuntimeService {
    RuntimeService {
        shutdown,
        cmd_rx,
        slice_tx,
        recorder,
        config,
        sim,
    }
}
