//! 查询服务纯逻辑：请求 JSON → `orbitx-recorder` Reader → 响应 JSON。
//!
//! 与 zenoh 无关（main 只做收发）；会话定位经 `list_sessions` 白名单匹配，
//! 请求中的 session 字符串不参与路径拼接（防穿越）。

use std::path::Path;

use serde_json::{json, Value};

use orbitx_recorder::{list_sessions, SessionReader};

/// 单条查询处理。请求：`{"op":"sessions"}` 或
/// `{"op":"frames","session":"<目录名>","from":ms?,"to":ms?}`。
/// 响应：`{"ok":true,...}`；错误经 `Err(描述)` 由调用方包成 `{"ok":false,"error":...}`。
pub fn handle(request: &str, root: &Path) -> Result<Value, String> {
    let req: Value = serde_json::from_str(request).map_err(|e| format!("bad request json: {e}"))?;
    match req.get("op").and_then(Value::as_str) {
        Some("sessions") => sessions_response(root),
        Some("frames") => {
            let session = req
                .get("session")
                .and_then(Value::as_str)
                .ok_or("frames: missing session")?;
            let from = req.get("from").and_then(Value::as_u64).unwrap_or(0);
            let to = req.get("to").and_then(Value::as_u64).unwrap_or(u64::MAX);
            frames_response(root, session, from, to)
        }
        other => Err(format!("unknown op {other:?} (want sessions|frames)")),
    }
}

fn sessions_response(root: &Path) -> Result<Value, String> {
    let dirs = list_sessions(root).map_err(|e| e.to_string())?;
    let mut sessions = Vec::new();
    for dir in dirs {
        let r = match SessionReader::open(&dir) {
            Ok(r) => r,
            Err(e) => {
                tracing::warn!(dir = %dir.display(), error = %e, "skip unreadable session");
                continue;
            }
        };
        let m = r.manifest();
        sessions.push(json!({
            "dir": dir.file_name().and_then(|s| s.to_str()).unwrap_or_default(),
            "drive": m.drive,
            "sim_dt_ms": m.sim_dt_ms,
            "created_unix_ms": m.created_unix_ms,
            "segment_count": m.segments.len(),
            "frame_count": m.segments.iter().map(|s| u64::from(s.frame_count)).sum::<u64>(),
            "first_sim_t": m.segments.first().map(|s| s.first_sim_t).unwrap_or(0),
            "last_sim_t": m.segments.last().map(|s| s.last_sim_t).unwrap_or(0),
        }));
    }
    Ok(json!({ "ok": true, "sessions": sessions }))
}

fn frames_response(root: &Path, session: &str, from: u64, to: u64) -> Result<Value, String> {
    let dir = list_sessions(root)
        .map_err(|e| e.to_string())?
        .into_iter()
        .find(|d| d.file_name().and_then(|s| s.to_str()) == Some(session))
        .ok_or_else(|| format!("session not found: {session}"))?;
    let r = SessionReader::open(&dir).map_err(|e| e.to_string())?;
    let frames = r.frames_between(from, to).map_err(|e| e.to_string())?;
    let count = frames.len();
    let frames = serde_json::to_value(frames).map_err(|e| e.to_string())?;
    Ok(json!({ "ok": true, "session": session, "count": count, "frames": frames }))
}
