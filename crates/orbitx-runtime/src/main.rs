//! orbitx-runtime 二进制入口。

use clap::Parser;
use tracing::info;

use orbitx_runtime::cli::RuntimeArgs;
use orbitx_runtime::host::run_with_shutdown;
use orbitx_runtime::log_setup;
use orbitx_runtime::ShutdownFlag;

fn main() {
    let args = RuntimeArgs::parse();
    if let Err(e) = args.validate() {
        eprintln!("error: {e}");
        std::process::exit(2);
    }

    let session = args.load_session().expect("validated");

    let _guard = match log_setup::init(&args.log_dir, false) {
        Ok(g) => g,
        Err(e) => {
            eprintln!("failed to init logging: {e}");
            std::process::exit(2);
        }
    };

    info!(
        rocket = %session.rocket.name,
        class = %session.rocket.class,
        stages = session.rocket.stages.len(),
        has_scenario = session.scenario.is_some(),
        control = session.control.kind_label(),
        drive = ?args.drive,
        sim_dt_ms = args.sim_dt,
        zenoh_endpoint = %args.zenoh_endpoint,
        "orbitx-runtime starting (local-SHM only)"
    );

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
