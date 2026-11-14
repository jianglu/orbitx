//! 会话读取端（Reader）：manifest 加载 + 段序帧遍历 + sim_t 范围过滤。
//!
//! 只读不写；不校验帧内容（schema 校验在解码层）。段按 `manifest.segments[]`
//! 顺序遍历；`frames_between` 先按段级 `first/last_sim_t` 剪枝再帧内过滤。

use std::path::{Path, PathBuf};

use crate::frame::RecorderFrame;
use crate::segment::decode_segment;
use crate::{Error, Manifest, SCHEMA_VERSION};

/// `root` 下的录制会话目录（含 `manifest.json`），按路径升序（时间序）。
///
/// C3 起**递归子目录**（竞赛子树 `competition/…`，见 FLIGHT_RECORDER.md
/// 「竞赛会话子树」）：会话目录最深 `MAX_DEPTH` 层（root 为第 0 层）防失控；
/// 不含 `root` 本身。
pub fn list_sessions(root: &Path) -> Result<Vec<PathBuf>, Error> {
    const MAX_DEPTH: usize = 8;
    let mut sessions = Vec::new();
    let mut stack = vec![(root.to_path_buf(), 0usize)];
    while let Some((dir, depth)) = stack.pop() {
        let entries = match std::fs::read_dir(&dir) {
            Ok(e) => e,
            // 子树目录缺读权限等：跳过该层（会话枚举尽力而为，不整体失败）。
            Err(_) => continue,
        };
        for entry in entries.flatten() {
            let path = entry.path();
            if !path.is_dir() || depth >= MAX_DEPTH {
                continue;
            }
            if path.join(super::writer::MANIFEST_NAME).is_file() {
                sessions.push(path);
            } else {
                stack.push((path, depth + 1));
            }
        }
    }
    sessions.sort();
    Ok(sessions)
}

/// 已打开的录制会话（manifest 已加载并校验）。
#[derive(Debug, Clone)]
pub struct SessionReader {
    dir: PathBuf,
    manifest: Manifest,
}

impl SessionReader {
    /// 打开会话目录：读 `manifest.json` 并校验 schema 版本。
    pub fn open(dir: &Path) -> Result<Self, Error> {
        let manifest_path = dir.join(super::writer::MANIFEST_NAME);
        let raw = std::fs::read_to_string(&manifest_path)?;
        let manifest: Manifest = serde_json::from_str(&raw).map_err(|e| Error::Manifest {
            path: manifest_path.clone(),
            why: e.to_string(),
        })?;
        if manifest.schema_version != SCHEMA_VERSION {
            return Err(Error::Manifest {
                path: manifest_path,
                why: format!(
                    "schema_version {} != {SCHEMA_VERSION}",
                    manifest.schema_version
                ),
            });
        }
        Ok(Self {
            dir: dir.to_path_buf(),
            manifest,
        })
    }

    pub fn dir(&self) -> &Path {
        &self.dir
    }

    pub fn manifest(&self) -> &Manifest {
        &self.manifest
    }

    /// 段文件绝对路径。
    pub fn segment_path(&self, name: &str) -> PathBuf {
        self.dir.join(name)
    }

    /// 按段序遍历全部帧（逐段解码；任一段损坏即返回 Err）。
    pub fn frames(&self) -> Result<Vec<RecorderFrame>, Error> {
        let mut frames = Vec::new();
        for seg in &self.manifest.segments {
            frames.extend(decode_segment(&self.segment_path(&seg.name))?);
        }
        Ok(frames)
    }

    /// `sim_t ∈ [first, last]` 的帧（段级 `last_sim_t` 剪枝后逐帧过滤）。
    pub fn frames_between(
        &self,
        first_sim_t: u64,
        last_sim_t: u64,
    ) -> Result<Vec<RecorderFrame>, Error> {
        let mut frames = Vec::new();
        for seg in &self.manifest.segments {
            if seg.last_sim_t < first_sim_t {
                continue;
            }
            if seg.first_sim_t > last_sim_t {
                break; // 段序即 sim_t 序（单会话单调时间）
            }
            for frame in decode_segment(&self.segment_path(&seg.name))? {
                let t = frame.sim_t();
                if t < first_sim_t {
                    continue;
                }
                if t > last_sim_t {
                    break;
                }
                frames.push(frame);
            }
        }
        Ok(frames)
    }
}
