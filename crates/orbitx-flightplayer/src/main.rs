//! `orbitx-flightplayer`：FlightRecorder 查询服务（zenoh queryable 独立进程）。
//!
//! 协议：key `orbitx/recorder/query`；请求/响应 JSON（见 [`service::handle`]）。
//! 存储 root 缺省 = 平台用户数据目录（与 Godot 传入的 `--recorder-dir` 一致）：
//! macOS `~/Library/Application Support/WLCY/SimRocket/orbitx_records`、
//! Windows `%APPDATA%\WLCY\SimRocket\orbitx_records`、
//! Linux `$XDG_DATA_HOME|$HOME/.local/share` + `/WLCY/SimRocket/orbitx_records`。

mod service;

use std::path::PathBuf;

use clap::Parser;
use tracing::{error, info};
use zenoh::Config;

use crate::service::handle;

#[derive(Debug, Parser)]
#[command(name = "orbitx-flightplayer", about = "FlightRecorder query service")]
struct Args {
    /// 空或 `local` → listen tcp/127.0.0.1:17448；`tcp/...` 原样使用。
    #[arg(long, default_value = "local")]
    zenoh_endpoint: String,
    /// 录制根目录；缺省 = 平台用户数据目录 WLCY/SimRocket/orbitx_records。
    #[arg(long)]
    root: Option<PathBuf>,
}

fn default_root() -> PathBuf {
    #[cfg(target_os = "macos")]
    let base =
        std::env::var_os("HOME").map(|h| PathBuf::from(h).join("Library/Application Support"));
    #[cfg(windows)]
    let base = std::env::var_os("APPDATA").map(PathBuf::from);
    #[cfg(all(unix, not(target_os = "macos")))]
    let base = std::env::var_os("XDG_DATA_HOME")
        .map(PathBuf::from)
        .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".local/share")));
    base.unwrap_or_default()
        .join("WLCY")
        .join("SimRocket")
        .join("orbitx_records")
}

/// 独立 listen 端口（runtime host local 端口 17447 之后顺延）。
const LOCAL_LOCATOR: &str = "tcp/127.0.0.1:17448";

fn build_config(endpoint: &str) -> Result<Config, String> {
    let ep = endpoint.trim();
    let locator = if ep.is_empty() || ep.eq_ignore_ascii_case("local") {
        LOCAL_LOCATOR.to_string()
    } else if ep.starts_with("tcp/") || ep.starts_with("udp/") {
        ep.to_string()
    } else {
        return Err(format!(
            "unsupported --zenoh-endpoint {ep:?} (local | tcp/…)"
        ));
    };
    let mut config = Config::default();
    config
        .insert_json5("mode", r#""peer""#)
        .map_err(|e| e.to_string())?;
    config
        .insert_json5("listen/endpoints", &format!(r#"["{locator}"]"#))
        .map_err(|e| e.to_string())?;
    config
        .insert_json5("scouting/multicast/enabled", "false")
        .map_err(|e| e.to_string())?;
    Ok(config)
}

const QUERY_KEY: &str = "orbitx/recorder/query";

#[tokio::main]
async fn main() {
    tracing_subscriber::fmt::init();
    let args = Args::parse();
    let root = args.root.clone().unwrap_or_else(default_root);
    let key = QUERY_KEY.to_string();

    let config = match build_config(&args.zenoh_endpoint) {
        Ok(c) => c,
        Err(e) => {
            error!(error = %e, "zenoh config failed");
            return;
        }
    };
    let session = match zenoh::open(config).await {
        Ok(s) => s,
        Err(e) => {
            error!(error = %e, "zenoh open failed");
            return;
        }
    };
    let queryable = match session.declare_queryable(key.clone()).await {
        Ok(q) => q,
        Err(e) => {
            error!(error = %e, "declare queryable failed");
            return;
        }
    };
    info!(
        key = %key,
        root = %root.display(),
        endpoint = %args.zenoh_endpoint,
        "orbitx-flightplayer serving"
    );

    loop {
        let query = tokio::select! {
            _ = tokio::signal::ctrl_c() => {
                info!("ctrl-c: flightplayer stopping");
                break;
            }
            q = queryable.recv_async() => match q {
                Ok(q) => q,
                Err(e) => {
                    error!(error = %e, "queryable closed");
                    break;
                }
            },
        };
        let request = query
            .payload()
            .map(|p| String::from_utf8_lossy(&p.to_bytes()).into_owned())
            .unwrap_or_default();
        let body = match handle(&request, &root) {
            Ok(v) => v.to_string(),
            Err(e) => {
                tracing::warn!(request = %request, error = %e, "query failed");
                serde_json::json!({ "ok": false, "error": e }).to_string()
            }
        };
        if let Err(e) = query.reply(key.clone(), body).await {
            error!(error = %e, "reply failed");
        }
    }
}

#[cfg(test)]
mod tests;
