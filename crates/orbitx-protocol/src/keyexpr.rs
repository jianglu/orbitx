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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn local_keys() {
        assert_eq!(cmd_key("local"), "orbitx/local/cmd");
        assert_eq!(slice_key("local"), "orbitx/local/slice");
    }

    #[test]
    fn shm_threshold_covers_slice_not_commands() {
        assert_eq!(SHM_MESSAGE_SIZE_THRESHOLD, 512);
    }
}
