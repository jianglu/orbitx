//! Zenoh 客户端：编解码 Inbound / 收 Slice。

use orbitx_protocol::{
    self, Inbound, InputCmd, SessionCmd, SetAttitudeAxes, SetGravityTurn, SetThrottle, SetWarp,
    encode_message, inbound, input_cmd, keyexpr, session_cmd,
};
use zenoh::bytes::ZBytes;
use zenoh::Config;
use zenoh::Session;

/// 与 runtime Comms 对齐的本机 locator。
pub fn local_locator(endpoint: &str) -> String {
    let ep = endpoint.trim();
    if ep.is_empty() || ep.eq_ignore_ascii_case("local") {
        return "tcp/127.0.0.1:17447".into();
    }
    let lower = ep.to_ascii_lowercase();
    if lower.starts_with("tcp/") || lower.starts_with("udp/") {
        return ep.to_string();
    }
    let mut h: u32 = 2166136261;
    for b in ep.bytes() {
        h ^= b as u32;
        h = h.wrapping_mul(16777619);
    }
    let port = 17000 + (h % 2000);
    format!("tcp/127.0.0.1:{port}")
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

pub struct ZenohClient {
    session: Session,
    cmd_key: String,
    pub endpoint: String,
}

impl ZenohClient {
    pub async fn connect(endpoint: &str) -> Result<Self, String> {
        let config = build_client_config(endpoint)?;
        let session = zenoh::open(config)
            .await
            .map_err(|e| format!("zenoh open: {e}"))?;
        Ok(Self {
            session,
            cmd_key: keyexpr::cmd_key(endpoint),
            endpoint: endpoint.to_string(),
        })
    }

    pub fn slice_key(&self) -> String {
        keyexpr::slice_key(&self.endpoint)
    }

    pub async fn declare_slice_sub(
        &self,
    ) -> Result<zenoh::pubsub::Subscriber<zenoh::handlers::FifoChannelHandler<zenoh::sample::Sample>>, String>
    {
        self.session
            .declare_subscriber(self.slice_key())
            .await
            .map_err(|e| format!("declare slice sub: {e}"))
    }

    pub async fn put_inbound(&self, msg: &Inbound) -> Result<(), String> {
        let bytes = encode_message(msg);
        self.session
            .put(&self.cmd_key, ZBytes::from(bytes))
            .await
            .map_err(|e| format!("put cmd: {e}"))
    }

    pub async fn send_input(&self, kind: input_cmd::Kind) -> Result<(), String> {
        self.put_inbound(&Inbound {
            payload: Some(inbound::Payload::Input(InputCmd { kind: Some(kind) })),
        })
        .await
    }

    pub async fn send_session(&self, kind: session_cmd::Kind) -> Result<(), String> {
        self.put_inbound(&Inbound {
            payload: Some(inbound::Payload::Session(SessionCmd { kind: Some(kind) })),
        })
        .await
    }

    pub async fn set_throttle(&self, level: f64) -> Result<(), String> {
        self.send_input(input_cmd::Kind::SetThrottle(SetThrottle { level }))
            .await
    }

    pub async fn set_attitude(&self, pitch: f64, yaw: f64) -> Result<(), String> {
        self.send_input(input_cmd::Kind::SetAttitudeAxes(SetAttitudeAxes {
            pitch,
            yaw,
            roll: 0.0,
        }))
        .await
    }

    pub async fn set_gravity_turn(&self, enabled: bool) -> Result<(), String> {
        self.send_input(input_cmd::Kind::SetGravityTurn(SetGravityTurn { enabled }))
            .await
    }

    pub async fn separate(&self) -> Result<(), String> {
        self.send_input(input_cmd::Kind::Separate(
            orbitx_protocol::Separate {},
        ))
        .await
    }

    pub async fn pause(&self) -> Result<(), String> {
        self.send_session(session_cmd::Kind::Pause(orbitx_protocol::Pause {}))
            .await
    }

    pub async fn resume(&self) -> Result<(), String> {
        self.send_session(session_cmd::Kind::Resume(orbitx_protocol::Resume {}))
            .await
    }

    pub async fn set_warp(&self, scale: f64) -> Result<(), String> {
        self.send_session(session_cmd::Kind::SetWarp(SetWarp { scale }))
            .await
    }

    pub async fn reset(&self) -> Result<(), String> {
        self.send_session(session_cmd::Kind::Reset(orbitx_protocol::Reset {}))
            .await
    }

    pub async fn shutdown(&self) -> Result<(), String> {
        self.send_session(session_cmd::Kind::Shutdown(orbitx_protocol::Shutdown {}))
            .await
    }
}
