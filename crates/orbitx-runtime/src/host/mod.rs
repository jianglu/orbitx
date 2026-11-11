//! 装配 Runtime 线程 + Comms / IO（测试与 bin 共用）。
//!
//! 会话来源双模式（`cli::ArgvMode`）：
//! - **Eager**：argv 全量（`orbitx-cli` 路径）——启动即解析构建；
//! - **Deferred（空载）**：argv 无会话级参数——先起 Comms（zenoh 就绪），
//!   再进入 pre-configure 循环等待客户端 `SessionCmd::Configure`；校验失败
//!   经 ack 通道回执并保持空载，成功后按配置构建会话继续常规装配。
//!
//! Configure 的消费放在 host 层（而非 RuntimeService）：recorder 会话目录、
//! `build_sim_bundle`、`build_runtime_service` 全依赖 `LoadedSession`，在
//! host 顺序装配一次即可，RuntimeService 结构零改动。

use std::thread::JoinHandle;
use std::time::Duration;

use flume::{Receiver, Sender};
use tracing::{info, warn};

use crate::channel::{RuntimeChannels, RuntimeInbound};
use crate::cli::{load_session, ArgvMode, DriveModeArg, RuntimeArgs};
use crate::comms::{run_comms, CommsHandles};
use crate::input::SessionCmd;
use crate::recorder::RecorderIo;
use crate::runtime::{build_runtime_service, RuntimeServiceConfig};
use crate::session::build_sim_bundle;
use crate::shutdown::ShutdownFlag;
use crate::slice::Slice;

pub struct HostHandles {
    pub shutdown: ShutdownFlag,
    pub cmd_tx: Sender<RuntimeInbound>,
    pub slice_rx: Receiver<std::sync::Arc<Slice>>,
    join: Option<JoinHandle<()>>,
}

impl HostHandles {
    pub fn request_shutdown(&self) {
        self.shutdown.request();
    }

    pub fn join(mut self) {
        if let Some(h) = self.join.take() {
            let _ = h.join();
        }
    }
}

/// 拉起 Runtime 线程 + 独立线程上的 Comms/IO tokio；调用方 `request_shutdown` 后 `join`。
pub fn spawn_host(args: RuntimeArgs) -> HostHandles {
    let shutdown = ShutdownFlag::new();
    let channels = RuntimeChannels::bounded();
    let cmd_tx = channels.cmd_tx.clone();
    let slice_rx = channels.slice_rx.clone();
    let flag = shutdown.clone();

    let join = std::thread::Builder::new()
        .name("orbitx-comms-io".into())
        .spawn(move || {
            let rt = tokio::runtime::Builder::new_multi_thread()
                .enable_all()
                .thread_name("orbitx-io-comms")
                .build()
                .expect("tokio runtime");
            rt.block_on(run_with_channels(args, flag, channels));
        })
        .expect("spawn io/comms thread");

    HostHandles {
        shutdown,
        cmd_tx,
        slice_rx,
        join: Some(join),
    }
}

async fn run_with_channels(args: RuntimeArgs, shutdown: ShutdownFlag, channels: RuntimeChannels) {
    // Comms 先行：空载模式下客户端在会话构建前就要连上 zenoh（下发
    // Configure、收 ack），监听必须先就绪。cmd_rx 以 clone 供 pre-configure
    // 循环消费，原 receiver 之后 move 给 RuntimeService（循环结束时 clone
    // 已 drop，不再竞争）。
    let comms = CommsHandles::from_channels(&channels);
    let comms_task = tokio::spawn(run_comms(
        shutdown.clone(),
        comms,
        args.zenoh_endpoint.clone(),
    ));

    // 会话来源：argv Eager 直通 / Deferred（空载）等 Configure。
    let session = match args.argv_mode().expect("validated before run") {
        ArgvMode::Eager(spec) => load_session(&spec, &args.ephemeris_data)
            .expect("session must validate before run"),
        ArgvMode::Deferred => match configure_wait(&args, &channels, &shutdown).await {
            Some(session) => session,
            // 空载期收到 Shutdown：直接收尾（runtime service 不装配）。
            None => {
                info!("shutdown during configure wait; runtime service not started");
                let _ = comms_task.await;
                return;
            }
        },
    };
    let sim = build_sim_bundle(&session, &args.ephemeris_data).expect("sim bundle");
    info!(
        rocket = %session.rocket.name,
        class = %session.rocket.class,
        scenario = %session.scenario.name,
        control = session.control.kind_label(),
        "session config loaded"
    );

    let (recorder_enq, recorder_io) = if args.recorder_enabled {
        let dir = args
            .recorder_dir
            .clone()
            .expect("validate: recorder_dir when enabled");
        let drive = match args.drive {
            DriveModeArg::ClientStep => "client-step",
            DriveModeArg::SelfPaced => "self-paced",
        };
        let meta = orbitx_recorder::SessionMeta {
            drive: drive.into(),
            sim_dt_ms: args.sim_dt,
            created_unix_ms: None,
        };
        let now_ms = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_millis() as u64)
            .unwrap_or(0);
        let session_base = crate::recorder::new_session_id(&session.rocket.name, now_ms);
        let (enq, io) = RecorderIo::pair(dir, session_base, meta);
        (enq, Some(io))
    } else {
        info!("FlightRecorder disabled (--recorder-enabled=false)");
        (crate::recorder::RecorderEnqueue::disabled(), None)
    };

    let runtime = build_runtime_service(
        shutdown.clone(),
        channels.cmd_rx,
        channels.slice_bus.clone(),
        recorder_enq,
        RuntimeServiceConfig {
            sim_dt_ms: args.sim_dt,
            drive: args.drive,
        },
        sim,
        session,
        args.ephemeris_data.clone(),
    );
    let runtime_join = runtime.spawn();

    let recorder_task = recorder_io.map(|io| tokio::spawn(io.run(shutdown.clone())));

    while !shutdown.is_requested() {
        tokio::time::sleep(Duration::from_millis(10)).await;
    }

    info!("shutdown ordered: comms → runtime join → recorder drain");
    let _ = comms_task.await;
    let _ = runtime_join.join();
    if let Some(t) = recorder_task {
        let _ = t.await;
    }
}

/// 空载 pre-configure 循环：等 `SessionCmd::Configure`，校验通过回 ack(ok)
/// 并返回会话；失败回 ack(err) 并保持空载继续等。空载期其余命令（Step /
/// Input 等）无会话可作用，丢弃。返回 None = 收到 Shutdown。
async fn configure_wait(
    args: &RuntimeArgs,
    channels: &RuntimeChannels,
    shutdown: &ShutdownFlag,
) -> Option<crate::cli::LoadedSession> {
    let ack_tx = channels.ack_tx.clone();
    let cmd_rx = channels.cmd_rx.clone();
    loop {
        if shutdown.is_requested() {
            return None;
        }
        while let Ok(msg) = cmd_rx.try_recv() {
            match msg {
                RuntimeInbound::Session(SessionCmd::Configure { spec }) => {
                    match load_session(&spec, &args.ephemeris_data) {
                        Ok(session) => {
                            info!(
                                rocket = %session.rocket.name,
                                scenario = %session.scenario.name,
                                "session configured via zenoh Configure"
                            );
                            let _ = ack_tx.send(orbitx_protocol::ConfigureAck {
                                ok: true,
                                error: String::new(),
                            });
                            return Some(session);
                        }
                        Err(e) => {
                            warn!(error = %e, "configure rejected; staying unconfigured");
                            let _ = ack_tx.send(orbitx_protocol::ConfigureAck {
                                ok: false,
                                error: e,
                            });
                        }
                    }
                }
                RuntimeInbound::Session(SessionCmd::Shutdown) => {
                    shutdown.request();
                    return None;
                }
                _ => {
                    // 空载期无会话：Input / Step / Launch 等不作用。
                }
            }
        }
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
}

/// bin：在调用方 tokio runtime 上跑 Comms + Recorder；Runtime 为 `std::thread`。
pub async fn run_with_shutdown(args: RuntimeArgs, shutdown: ShutdownFlag) {
    let channels = RuntimeChannels::bounded();
    run_with_channels(args, shutdown, channels).await;
}

#[cfg(test)]
mod tests;
