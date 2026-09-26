//! 装配 Runtime 线程 + Comms / IO（测试与 bin 共用）。

use std::thread::JoinHandle;
use std::time::Duration;

use flume::{Receiver, Sender};
use tracing::info;

use crate::channel::{RuntimeChannels, RuntimeInbound};
use crate::cli::RuntimeArgs;
use crate::comms::{run_comms, CommsHandles};
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
    let session = args
        .load_session()
        .expect("session must validate before run");
    let sim = build_sim_bundle(&session, args.ephemeris_data.as_deref())
        .expect("sim bundle");
    info!(
        rocket = %session.rocket.name,
        class = %session.rocket.class,
        has_scenario = session.scenario.is_some(),
        control = session.control.kind_label(),
        "session config loaded"
    );

    let (recorder_enq, recorder_io) = RecorderIo::pair(args.recorder_dir.clone());

    let runtime = build_runtime_service(
        shutdown.clone(),
        channels.cmd_rx,
        channels.slice_tx,
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

    let comms = CommsHandles {
        cmd_tx: channels.cmd_tx,
        slice_rx: channels.slice_rx,
    };

    let comms_task = tokio::spawn(run_comms(
        shutdown.clone(),
        comms,
        args.zenoh_endpoint.clone(),
    ));
    let recorder_task = tokio::spawn(recorder_io.run(shutdown.clone()));

    while !shutdown.is_requested() {
        tokio::time::sleep(Duration::from_millis(10)).await;
    }

    info!("shutdown ordered: comms → runtime join → recorder drain");
    let _ = comms_task.await;
    let _ = runtime_join.join();
    let _ = recorder_task.await;
}

/// bin：在调用方 tokio runtime 上跑 Comms + Recorder；Runtime 为 `std::thread`。
pub async fn run_with_shutdown(args: RuntimeArgs, shutdown: ShutdownFlag) {
    let channels = RuntimeChannels::bounded();
    run_with_channels(args, shutdown, channels).await;
}

#[cfg(test)]
mod tests;
