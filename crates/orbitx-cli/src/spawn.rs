//! 查找并拉起同 profile 的 `orbitx-runtime` 子进程。

use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};

/// 在当前可执行文件同目录查找 `orbitx-runtime`（debug/release 同 profile）。
pub fn find_runtime_exe() -> Result<PathBuf, String> {
    let self_exe = std::env::current_exe().map_err(|e| format!("current_exe: {e}"))?;
    let dir = self_exe
        .parent()
        .ok_or_else(|| "current_exe has no parent".to_string())?;
    let name = if cfg!(windows) {
        "orbitx-runtime.exe"
    } else {
        "orbitx-runtime"
    };
    let candidate = dir.join(name);
    if candidate.is_file() {
        return Ok(candidate);
    }
    // 开发时常从 workspace 跑：再试 target/<profile>/
    for profile in ["debug", "release"] {
        let alt = dir
            .join("..")
            .join(profile)
            .join(name);
        if alt.is_file() {
            return Ok(alt.canonicalize().unwrap_or(alt));
        }
    }
    Err(format!(
        "找不到 {name}（已查 {} 与 target/debug|release）；请先 `cargo build -p orbitx-runtime`",
        dir.display()
    ))
}

pub struct RuntimeChild {
    child: Child,
}

impl RuntimeChild {
    pub fn spawn(exe: &Path, args: &[String]) -> Result<Self, String> {
        let mut cmd = Command::new(exe);
        cmd.args(args)
            .stdin(Stdio::null())
            .stdout(Stdio::inherit())
            .stderr(Stdio::inherit());
        let child = cmd
            .spawn()
            .map_err(|e| format!("spawn {}: {e}", exe.display()))?;
        Ok(Self { child })
    }

    pub fn kill_and_wait(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}

impl Drop for RuntimeChild {
    fn drop(&mut self) {
        self.kill_and_wait();
    }
}
