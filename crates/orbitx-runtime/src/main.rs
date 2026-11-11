//! orbitx-runtime 二进制入口。

// Windows 下为 GUI 子系统：被 Godot（GUI 进程）拉起时不分配控制台终端窗口；
// 非 Windows 目标忽略此属性。stderr 句柄由拉起方（orbitx-cli 继承 / bridge 管道）重定向，诊断不受影响。
#![windows_subsystem = "windows"]

use clap::Parser;
use tracing::info;

use orbitx_runtime::cli::{load_session, ArgvMode, RuntimeArgs};
use orbitx_runtime::host::run_with_shutdown;
use orbitx_runtime::log_setup;
use orbitx_runtime::ShutdownFlag;

fn main() {
    let args = RuntimeArgs::parse();
    // argv 模式判定（部分会话参数 = 配置错误）；进程级 + Eager 模式全量校验。
    let mode = match args.argv_mode() {
        Ok(m) => m,
        Err(e) => {
            eprintln!("error: {e}");
            std::process::exit(2);
        }
    };
    if let Err(e) = args.validate() {
        eprintln!("error: {e}");
        std::process::exit(2);
    }

    let _guard = match log_setup::init(&args.log_dir, false) {
        Ok(g) => g,
        Err(e) => {
            eprintln!("failed to init logging: {e}");
            std::process::exit(2);
        }
    };

    match &mode {
        ArgvMode::Eager(spec) => {
            // Eager：argv 已通过 validate，会话解析失败即 bug（panic 可见）。
            let session = load_session(spec, &args.ephemeris_data).expect("validated");
            info!(
                rocket = %session.rocket.name,
                class = %session.rocket.class,
                stages = session.rocket.stages.len(),
                scenario = %session.scenario.name,
                control = session.control.kind_label(),
                drive = ?args.drive,
                sim_dt_ms = args.sim_dt,
                zenoh_endpoint = %args.zenoh_endpoint,
                "orbitx-runtime starting (local-SHM only)"
            );
        }
        ArgvMode::Deferred => {
            info!(
                drive = ?args.drive,
                sim_dt_ms = args.sim_dt,
                zenoh_endpoint = %args.zenoh_endpoint,
                "orbitx-runtime starting UNCONFIGURED (awaiting zenoh Configure; local-SHM only)"
            );
        }
    }

    let shutdown = ShutdownFlag::new();
    let flag = shutdown.clone();

    let rt = tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .thread_name("orbitx-comms-io")
        .build()
        .expect("tokio");

    rt.block_on(async move {
        let shutdown_task = {
            let flag = flag.clone();
            tokio::spawn(async move {
                match tokio::signal::ctrl_c().await {
                    Ok(()) => flag.request(),
                    Err(e) => {
                        tracing::error!(error = %e, "ctrl_c handler failed");
                        flag.request();
                    }
                }
            })
        };

        run_with_shutdown(args, shutdown).await;
        shutdown_task.abort();
    });

    info!("orbitx-runtime exited cleanly");
}
