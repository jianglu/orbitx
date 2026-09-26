//! 本机 Zenoh + SHM Comms（官方 peer API；禁跨设备）。

use std::time::Duration;

use tracing::{debug, error, info, warn};
use zenoh::Config;
use zenoh::bytes::ZBytes;

use crate::channel::RuntimeInbound;
use crate::comms::CommsHandles;
use crate::input::{InputCmd, SessionCmd};
use crate::shutdown::ShutdownFlag;
use orbitx_protocol::{
    self, Inbound, encode_message,
    inbound::Payload as InboundPayload,
    input_cmd::Kind as InputKind,
    keyexpr,
    session_cmd::Kind as SessionKind,
};

/// 由 `--zenoh-endpoint` 得到本机 listen/connect locator。
pub fn local_locator(endpoint: &str) -> String {
    let ep = endpoint.trim();
    if ep.is_empty() || ep.eq_ignore_ascii_case("local") {
        return "tcp/127.0.0.1:17447".into();
    }
    let lower = ep.to_ascii_lowercase();
    if lower.starts_with("tcp/") || lower.starts_with("udp/") {
        return ep.to_string();
    }
    // 会话 id → 固定本机端口（哈希），避免跨设备。
    let mut h: u32 = 2166136261;
    for b in ep.bytes() {
        h ^= b as u32;
        h = h.wrapping_mul(16777619);
    }
    let port = 17000 + (h % 2000);
    format!("tcp/127.0.0.1:{port}")
}

pub fn build_host_config(endpoint: &str) -> Result<Config, String> {
    let locator = local_locator(endpoint);
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
    config
        .insert_json5("transport/shared_memory/enabled", "true")
        .map_err(|e| e.to_string())?;
    Ok(config)
}

pub fn build_client_config(endpoint: &str) -> Result<Config, String> {
    let locator = local_locator(endpoint);
    let mut config = Config::default();
    config
        .insert_json5("mode", r#""peer""#)
        .map_err(|e| e.to_string())?;
    config
        .insert_json5("connect/endpoints", &format!(r#"["{locator}"]"#))
        .map_err(|e| e.to_string())?;
    config
        .insert_json5("scouting/multicast/enabled", "false")
        .map_err(|e| e.to_string())?;
    config
        .insert_json5("transport/shared_memory/enabled", "true")
        .map_err(|e| e.to_string())?;
    Ok(config)
}

fn inbound_from_proto(msg: Inbound) -> Option<RuntimeInbound> {
    match msg.payload? {
        InboundPayload::Input(input) => {
            let cmd = match input.kind? {
                InputKind::SetThrottle(t) => InputCmd::SetThrottle { level: t.level },
                InputKind::SetAttitudeAxes(a) => InputCmd::SetAttitudeAxes {
                    pitch: a.pitch,
                    yaw: a.yaw,
                    roll: a.roll,
                },
                InputKind::Separate(_) => InputCmd::Separate,
                InputKind::SetGravityTurn(g) => InputCmd::SetGravityTurn {
                    enabled: g.enabled,
                },
            };
            Some(RuntimeInbound::Input(cmd))
        }
        InboundPayload::Session(session) => {
            let cmd = match session.kind? {
                SessionKind::Pause(_) => SessionCmd::Pause,
                SessionKind::Resume(_) => SessionCmd::Resume,
                SessionKind::SetWarp(w) => SessionCmd::SetWarp { scale: w.scale },
                SessionKind::Step(s) => SessionCmd::Step { n: s.n },
                SessionKind::Shutdown(_) => SessionCmd::Shutdown,
                SessionKind::Reset(_) => SessionCmd::Reset,
            };
            Some(RuntimeInbound::Session(cmd))
        }
    }
}

pub async fn run(shutdown: ShutdownFlag, handles: CommsHandles, zenoh_endpoint: String) {
    let config = match build_host_config(&zenoh_endpoint) {
        Ok(c) => c,
        Err(e) => {
            error!(error = %e, "zenoh host config failed");
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

    let cmd_key = keyexpr::cmd_key(&zenoh_endpoint);
    let slice_key = keyexpr::slice_key(&zenoh_endpoint);
    info!(
        endpoint = %zenoh_endpoint,
        listen = %local_locator(&zenoh_endpoint),
        cmd = %cmd_key,
        slice = %slice_key,
        "CommsService Zenoh started (local SHM)"
    );

    let subscriber = match session.declare_subscriber(cmd_key).await {
        Ok(s) => s,
        Err(e) => {
            error!(error = %e, "declare cmd subscriber failed");
            return;
        }
    };
    let publisher = match session.declare_publisher(slice_key).await {
        Ok(p) => p,
        Err(e) => {
            error!(error = %e, "declare slice publisher failed");
            return;
        }
    };

    while !shutdown.is_requested() {
        while let Ok(Some(sample)) = subscriber.try_recv() {
            let bytes = sample.payload().to_bytes();
            match orbitx_protocol::decode_inbound(&bytes) {
                Ok(msg) => {
                    if let Some(inbound) = inbound_from_proto(msg) {
                        if let Err(e) = handles.cmd_tx.send(inbound) {
                            warn!(error = %e, "cmd channel closed");
                        }
                    }
                }
                Err(e) => warn!(error = %e, "bad inbound protobuf"),
            }
        }

        // keep-latest：排空队列，只发最新一帧。
        let mut latest = None;
        while let Ok(slice) = handles.slice_rx.try_recv() {
            latest = Some(slice);
        }
        if let Some(slice) = latest {
            let proto = slice.to_proto();
            let bytes = encode_message(&proto);
            if let Err(e) = publisher.put(ZBytes::from(bytes)).await {
                debug!(error = %e, "slice put failed");
            }
        }

        tokio::time::sleep(Duration::from_millis(2)).await;
    }

    while handles.slice_rx.try_recv().is_ok() {}
    info!("CommsService Zenoh stopped");
}
