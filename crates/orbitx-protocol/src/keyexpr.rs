//! Zenoh keyexpr：`orbitx/{session_id}/cmd` 与 `orbitx/{session_id}/slice`。

/// 大于等于该字节数的 put 自动进入共享内存。命令远小于此值，留在 TCP 上。
pub const SHM_MESSAGE_SIZE_THRESHOLD: u64 = 512;

/// 规范化会话 id（`--zenoh-endpoint`）。
pub fn session_id(endpoint: &str) -> String {
    let ep = endpoint.trim();
    if ep.is_empty() || ep.eq_ignore_ascii_case("local") {
        "local".into()
    } else {
        // 用安全字符替换，避免 keyexpr 非法片段。
        ep.chars()
            .map(|c| {
                if c.is_ascii_alphanumeric() || c == '-' || c == '_' {
                    c
                } else {
                    '_'
                }
            })
            .collect()
    }
}

pub fn cmd_key(endpoint: &str) -> String {
    format!("orbitx/{}/cmd", session_id(endpoint))
}

pub fn slice_key(endpoint: &str) -> String {
    format!("orbitx/{}/slice", session_id(endpoint))
}

/// 回执通道（ConfigureAck 等控制面消息）：与切片分 key——ack 低频、切片
/// 高频且被消费端判新逻辑覆盖，混流会互相干扰。
pub fn ack_key(endpoint: &str) -> String {
    format!("orbitx/{}/ack", session_id(endpoint))
}

/// flightplayer 黑匣子查询 key(全 endpoint 共用;P6.x Agent 对接)。
/// 真相源:本常量——flightplayer 服务端与 bridge 客户端共同引用,勿双写。
pub fn recorder_query_key() -> String {
    "orbitx/recorder/query".into()
}

/// flightplayer 本机 listen locator(runtime `local` 17447 之后顺延)。
/// 单实例假设:flightplayer 为无状态只读查询服务,多客户端并发 query 安全。
pub const FLIGHTPLAYER_LOCAL_LOCATOR: &str = "tcp/127.0.0.1:17448";

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn local_keys() {
        assert_eq!(cmd_key("local"), "orbitx/local/cmd");
        assert_eq!(slice_key("local"), "orbitx/local/slice");
        assert_eq!(ack_key("local"), "orbitx/local/ack");
    }

    #[test]
    fn shm_threshold_covers_slice_not_commands() {
        assert_eq!(SHM_MESSAGE_SIZE_THRESHOLD, 512);
    }

    #[test]
    fn recorder_query_key_and_locator() {
        assert_eq!(recorder_query_key(), "orbitx/recorder/query");
        assert_eq!(FLIGHTPLAYER_LOCAL_LOCATOR, "tcp/127.0.0.1:17448");
    }
}
