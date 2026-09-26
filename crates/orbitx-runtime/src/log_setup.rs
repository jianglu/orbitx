//! tracing + subscriber + non_blocking appender + tracing-log。

use std::path::Path;

use tracing_appender::non_blocking::WorkerGuard;
use tracing_subscriber::{fmt, layer::SubscriberExt, util::SubscriberInitExt, EnvFilter};

/// 初始化全局 subscriber。返回 guard，须持有至进程结束。
pub fn init(log_dir: &Path, console: bool) -> std::io::Result<WorkerGuard> {
    std::fs::create_dir_all(log_dir)?;
    let file_appender = tracing_appender::rolling::daily(log_dir, "orbitx-runtime.log");
    let (non_blocking, guard) = tracing_appender::non_blocking(file_appender);

    let filter = EnvFilter::try_from_default_env()
        .unwrap_or_else(|_| EnvFilter::new(if cfg!(debug_assertions) { "debug" } else { "info" }));

    let registry = tracing_subscriber::registry().with(filter);

    if console && cfg!(debug_assertions) {
        registry
            .with(fmt::layer().with_writer(std::io::stderr))
            .with(fmt::layer().with_writer(non_blocking).with_ansi(false))
            .init();
    } else {
        registry
            .with(fmt::layer().with_writer(non_blocking).with_ansi(false))
            .init();
    }

    let _ = tracing_log::LogTracer::init();
    Ok(guard)
}
