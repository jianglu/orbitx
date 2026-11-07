//! RuntimeService：固定步步进权威（`std::thread`）。

pub mod clock;
pub mod separation_queue;
pub mod tick;

use std::sync::Arc;
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use flume::Receiver;
use tracing::{error, info, warn};

use crate::channel::{send_slice_keep_latest, RuntimeInbound, SliceBus};
use crate::cli::{DriveModeArg, LoadedSession};
use crate::diag;
use crate::input::{InputCmd, SessionCmd};
use crate::recorder::{frame, RecorderEnqueue};
use crate::session::{build_sim_bundle, SimBundle};
use crate::shutdown::ShutdownFlag;

use self::clock::Clock;
use self::separation_queue::SeparationQueue;
use self::tick::TickOutcome;

pub struct RuntimeServiceConfig {
    pub sim_dt_ms: u64,
    pub drive: DriveModeArg,
}

/// 非步进期状态切片心跳周期（P3.13）：hold / paused / 客户端停发期间，
/// 客户端唯一切片来源。命令（Pause/Resume/SetWarp）回显不等心跳，即时发。
const STATUS_ECHO_PERIOD: Duration = Duration::from_millis(100);

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
    /// 改 E：上 pass 首步推迟时回收的 Input 命令（黑匣子输入链不丢步）。
    /// Input 在收集时已 apply，回收仅并入下一 pass 首步帧记录。
    pending_flight: Option<Vec<InputCmd>>,
    /// 分离事件 TTL 队列（P45）：步进入队、窗口出队随切片下发（TTL 重发防
    /// 倍速 keep-latest 覆盖丢片）；Reset 清空。
    separations: SeparationQueue,
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

        // 节奏诊断：启动实测 sleep(1ms) 真实粒度。Windows 定时器精度按进程
        // 隔离（Win10 2004+ / Win11 / Server 2022+），本进程未调 timeBeginPeriod
        // 时 1ms 可能实睡 ~15.6ms——ClientStep 每步预算只有 sim_dt(20ms)，此值
        // 直接决定仿真倍率是否达标。
        let probe = diag::probe_sleep(thread::sleep, Duration::from_millis(1), 32);
        info!(
            avg_us = probe.avg.as_micros() as u64,
            max_us = probe.max.as_micros() as u64,
            "sleep granularity probe: target 1ms (over ~2000us means OS timer granularity not raised)"
        );

        // 节奏诊断窗口：步进耗时（阈值 = 单步预算 sim_dt）、主循环空转 sleep
        // 实测（阈值 2ms = 名义 1ms 的 2 倍即视为粒度受损）、recorder 推迟计数。
        let step_budget = Duration::from_millis(self.config.sim_dt_ms);
        let mut step_cost = diag::DurationWindow::new(1024, step_budget);
        let mut idle_sleep = diag::DurationWindow::new(8192, Duration::from_millis(2));
        let mut recorder_defers = 0u64;
        let mut last_report = Instant::now();
        // P3.13：最近一次切片发布时刻（步进 + 状态统一计时）。心跳以其闲时
        // 判定——活跃步进期步进切片本身 ≤sim_dt 一发，心跳自然静默。
        let mut last_pub = Instant::now();

        while !self.shutdown.is_requested() {
            // 本 pass 收集的命令快照：随本 pass 首帧入黑匣子。
            let mut pass_flight: Vec<InputCmd> = Vec::new();
            let mut pass_session: Vec<SessionCmd> = Vec::new();

            // 改 E：上 pass 首步推迟时回收的 Input 命令并入本 pass 首步帧
            //（apply 已在上 pass 收集时执行过，这里只是黑匣子记录不丢）。
            if let Some(f) = self.pending_flight.take() {
                pass_flight.splice(0..0, f);
            }

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
                                // 分离事件随会话作废（新装配无历史拆级）。
                                self.separations.clear();
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

            // P3.13 非步进期状态切片回显：Pause/Resume/SetWarp 应用后即时回显
            // （state_echo，派生自本 pass 归集的会话命令，match 臂零侵入）；
            // 其余空闲期（hold / paused / ClientStep 停发）按心跳周期兜底发状态
            // 快照（完整富切片，纯读 build_slice，不入 recorder）。`last_pub`
            // 步进+状态统一计时——活跃步进期步切片 ≤sim_dt 一发（< 心跳周期），
            // 心跳自然静默。
            // P45：回显切片同样携带分离队列快照（TTL 窗口内每片重发——切片
            // 丢片不再等于事件丢失，keep-latest 覆盖无害）；出队窗口按当前
            // 步推进（暂停期步不走表，窗口不推进）。
            let state_echo = pass_session.iter().any(|c| {
                matches!(
                    c,
                    SessionCmd::Pause | SessionCmd::Resume | SessionCmd::SetWarp { .. }
                )
            });
            if state_echo || last_pub.elapsed() >= STATUS_ECHO_PERIOD {
                self.separations.prune(clock.step_index(), clock.warp());
                let slice =
                    tick::build_slice(&clock, &mut self.sim, self.gate_open, &self.separations);
                send_slice_keep_latest(&self.slice_bus, Arc::new(slice));
                last_pub = Instant::now();
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
                            recorder_defers += 1; // C1：水位未降，保留 pending_steps 推迟
                            break;
                        }
                        let (flight, session) = first_inputs
                            .take()
                            .unwrap_or_else(|| (Vec::new(), Vec::new()));
                        let t0 = Instant::now();
                        if self.do_one_step(&mut clock, &flight, &session, &mut last_pub) {
                            self.pending_steps -= 1;
                            step_cost.push(t0.elapsed());
                        } else {
                            // 改 E：首步就推迟（paused / recorder 竞态）→ Input
                            // 命令暂存，下一 pass 首步帧补记（不丢输入链）。
                            self.pending_flight = Some(flight);
                            break; // paused 等：pending 保留，Resume 后继续
                        }
                    }
                    let t0 = Instant::now();
                    thread::sleep(Duration::from_millis(1));
                    idle_sleep.push(t0.elapsed());
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
                    if !self.do_one_step(&mut clock, &flight, &session, &mut last_pub) {
                        // 改 E：推迟（recorder 竞态等）→ Input 命令暂存重试。
                        self.pending_flight = Some(flight);
                    }
                    let sleep_ms = ((self.config.sim_dt_ms as f64) / clock.warp().max(1e-6))
                        .round()
                        .max(1.0) as u64;
                    thread::sleep(Duration::from_millis(sleep_ms));
                }
            }

            // 节奏诊断：5s 窗口报告（暂停/hold 期窗口为空则跳过）。
            if last_report.elapsed() >= Duration::from_secs(5) {
                last_report = Instant::now();
                if let Some(st) = step_cost.snapshot() {
                    diag::log_window(
                        "step cost window (5s; over = steps over sim_dt budget)",
                        &st,
                        step_budget,
                    );
                }
                if let Some(st) = idle_sleep.snapshot() {
                    diag::log_window(
                        "idle sleep(1ms) window (5s; avg >> 1000us = timer granularity issue)",
                        &st,
                        Duration::from_millis(2),
                    );
                }
                if recorder_defers > 0 {
                    info!(
                        defers = recorder_defers,
                        "recorder backpressure defers (5s)"
                    );
                }
                step_cost.reset();
                idle_sleep.reset();
                recorder_defers = 0;
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
    /// `last_pub`：切片发布时刻（步进+状态统一计时，P3.13），步进发切片后更新。
    fn do_one_step(
        &mut self,
        clock: &mut Clock,
        flight: &[InputCmd],
        session_cmds: &[SessionCmd],
        last_pub: &mut Instant,
    ) -> bool {
        if clock.paused() {
            return false;
        }
        if !self.recorder.healthy() {
            self.report_recorder_failure();
            return false;
        }
        let outcome = tick::tick(clock, &mut self.sim, self.gate_open, &mut self.separations);
        match outcome {
            TickOutcome::Stepped {
                slice,
                new_separations,
            } => {
                // 黑匣子产生步一次性记录（原生 born_step/born_t_ms/vessels，
                // 不随切片 TTL 重发扩散）；旧字段 separated = 本步产生并集。
                let frame =
                    frame::step_frame(clock, &self.sim, flight, session_cmds, &new_separations);
                if !self.recorder.try_enqueue(frame) {
                    // C1 前置检查后的竞态兜底：本帧丢失（不阻塞步进）。
                    warn!("recorder L1 full at enqueue (race); frame dropped");
                }
                send_slice_keep_latest(&self.slice_bus, Arc::new(slice));
                *last_pub = Instant::now();
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
        pending_flight: None,
        separations: SeparationQueue::new(),
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
