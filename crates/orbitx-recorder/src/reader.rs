//! 会话读取端（Reader）：manifest 加载 + 段序帧遍历 + sim_t 范围过滤。
//!
//! 只读不写；不校验帧内容（schema 校验在解码层）。段按 `manifest.segments[]`
//! 顺序遍历；`frames_between` 先按段级 `first/last_sim_t` 剪枝再帧内过滤。

use std::path::{Path, PathBuf};

use crate::frame::RecorderFrame;
use crate::segment::decode_segment;
use crate::{Error, Manifest, SCHEMA_VERSION};

/// `root` 下的录制会话目录（含 `manifest.json`），按目录名升序（时间序）。
pub fn list_sessions(root: &Path) -> Result<Vec<PathBuf>, Error> {
    let mut sessions = Vec::new();
    for entry in std::fs::read_dir(root)? {
        let entry = entry?;
        let path = entry.path();
        if path.is_dir() && path.join(super::writer::MANIFEST_NAME).is_file() {
            sessions.push(path);
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
