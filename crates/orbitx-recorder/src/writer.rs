//! 会话写状态机：懒建目录 → 段内存缓冲 → 滚动封段（zstd）→ manifest 原子重写。
//!
//! 格式权威：`FLIGHT_RECORDER.md`「目录布局 / manifest.json / *.orec 段文件」。
//! 本模块不含 runtime 知识（Assembly / Slice）；帧构建在 `orbitx-runtime`。

use std::fs;
use std::io::Write as _;
use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

use serde::{Deserialize, Serialize};

use crate::frame::{RecorderFrame, SCHEMA_VERSION};
use crate::segment::{encode_frame, encode_segment_header, SegmentHeader};
use crate::{Error, CODEC_ZSTD, OREC_MAGIC};

/// 默认段滚动阈值：未压缩帧流 16 MiB。
pub const DEFAULT_MAX_UNCOMPRESSED_BYTES: usize = 16 * 1024 * 1024;
/// 默认段滚动阈值：65536 帧。
pub const DEFAULT_MAX_FRAMES_PER_SEGMENT: u32 = 65_536;
/// 默认稀疏索引间隔：每 1000 帧一条。
pub const DEFAULT_INDEX_EVERY: u32 = 1_000;
/// 默认 zstd 压缩级（`FLIGHT_RECORDER.md` 产品默认 3）。
pub const DEFAULT_ZSTD_LEVEL: i32 = 3;

/// 段滚动与压缩参数。测试注入小阈值。
#[derive(Debug, Clone)]
pub struct SegmentPolicy {
    pub max_uncompressed_bytes: usize,
    pub max_frames: u32,
    pub index_every: u32,
    pub zstd_level: i32,
}

impl Default for SegmentPolicy {
    fn default() -> Self {
        Self {
            max_uncompressed_bytes: DEFAULT_MAX_UNCOMPRESSED_BYTES,
            max_frames: DEFAULT_MAX_FRAMES_PER_SEGMENT,
            index_every: DEFAULT_INDEX_EVERY,
            zstd_level: DEFAULT_ZSTD_LEVEL,
        }
    }
}

/// manifest 头元数据（帧事件不携带，构造时传入）。
#[derive(Debug, Clone)]
pub struct SessionMeta {
    /// 步进节奏：`"client-step"` | `"self-paced"`。
    pub drive: String,
    /// 固定物理步长 [ms]。
    pub sim_dt_ms: u64,
    /// 会话创建墙钟 [ms]；None = 取当前。
    pub created_unix_ms: Option<u64>,
}

/// manifest `segments[]` 元素。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SegmentEntry {
    pub name: String,
    pub first_sim_t: u64,
    pub last_sim_t: u64,
    pub frame_count: u32,
    pub uncompressed_len: u64,
    pub compressed_len: u64,
}

/// manifest `index[]` 稀疏索引条目（全局帧序号）。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct IndexEntry {
    pub sim_t: u64,
    pub segment: String,
    pub frame_ordinal: u64,
}

/// `manifest.json`（schema 1）。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Manifest {
    pub schema_version: u32,
    #[serde(rename = "sim_dt")]
    pub sim_dt_ms: u64,
    pub drive: String,
    pub created_unix_ms: u64,
    pub codec: String,
    pub zstd_level: i32,
    pub segments: Vec<SegmentEntry>,
    pub index: Vec<IndexEntry>,
}

pub const MANIFEST_NAME: &str = "manifest.json";

/// 一次会话的写状态机。一个实例对应一个会话目录；
/// Reset（同进程重开会话）= 丢弃本实例、另建新实例（新目录）。
pub struct SessionWriter {
    session_id: String,
    dir: PathBuf,
    manifest: Manifest,
    manifest_dirty: bool,
    finished: bool,
    policy: SegmentPolicy,
    buf: Vec<u8>,
    seg_frames: u32,
    seg_first_sim_t: Option<u64>,
    seg_last_sim_t: u64,
    /// 已写入帧的全局序号（下一帧 = 该值）。
    next_ordinal: u64,
    next_segment: usize,
}

impl SessionWriter {
    /// 创建会话目录（`<root>/<session_id>/`）并写 manifest 初版。
    /// 目录名冲突时按 `<base>-2` … 依次探测；存在但为空则复用。
    pub fn create(
        root: impl AsRef<Path>,
        base_session_id: &str,
        meta: SessionMeta,
        policy: SegmentPolicy,
    ) -> Result<Self, Error> {
        let root = root.as_ref();
        let (session_id, dir) = resolve_session_dir(root, base_session_id)?;
        fs::create_dir_all(&dir)?;
        let manifest = Manifest {
            schema_version: SCHEMA_VERSION,
            sim_dt_ms: meta.sim_dt_ms,
            drive: meta.drive,
            created_unix_ms: meta.created_unix_ms.unwrap_or_else(now_unix_ms),
            codec: "zstd".into(),
            zstd_level: policy.zstd_level,
            segments: Vec::new(),
            index: Vec::new(),
        };
        let mut w = Self {
            session_id,
            dir,
            manifest,
            manifest_dirty: true,
            finished: false,
            policy,
            buf: Vec::new(),
            seg_frames: 0,
            seg_first_sim_t: None,
            seg_last_sim_t: 0,
            next_ordinal: 0,
            next_segment: 0,
        };
        w.write_manifest()?;
        Ok(w)
    }

    pub fn session_id(&self) -> &str {
        &self.session_id
    }

    pub fn dir(&self) -> &Path {
        &self.dir
    }

    /// 已写入帧总数（含未封段缓冲）。
    pub fn frames_written(&self) -> u64 {
        self.next_ordinal
    }

    pub fn is_finished(&self) -> bool {
        self.finished
    }

    /// 追加一帧。达到滚动阈值即封段；索引按 `index_every` 间隔记全局序号。
    pub fn push_frame(&mut self, frame: &RecorderFrame) -> Result<(), Error> {
        if self.finished {
            return Err(Error::WriterClosed);
        }
        let sim_t = frame.sim_t();
        let ordinal = self.next_ordinal;
        encode_frame(frame, &mut self.buf)?;
        if self.seg_first_sim_t.is_none() {
            self.seg_first_sim_t = Some(sim_t);
        }
        self.seg_last_sim_t = sim_t;
        self.seg_frames += 1;
        self.next_ordinal += 1;
        if self.policy.index_every > 0 && (ordinal + 1) % u64::from(self.policy.index_every) == 0 {
            self.manifest.index.push(IndexEntry {
                sim_t,
                segment: segment_name(self.next_segment),
                frame_ordinal: ordinal,
            });
            self.manifest_dirty = true;
        }
        if self.buf.len() >= self.policy.max_uncompressed_bytes
            || self.seg_frames >= self.policy.max_frames
        {
            self.flush_segment()?;
        }
        Ok(())
    }

    /// 结束会话：封最后一段 + 落终版 manifest。幂等；之后 `push_frame` 报错。
    pub fn finish(&mut self) -> Result<(), Error> {
        if self.finished {
            return Ok(());
        }
        if self.seg_frames > 0 {
            self.flush_segment()?;
        } else if self.manifest_dirty {
            self.write_manifest()?;
        }
        self.finished = true;
        Ok(())
    }

    fn flush_segment(&mut self) -> Result<(), Error> {
        if self.seg_frames == 0 {
            return Ok(());
        }
        let name = segment_name(self.next_segment);
        let uncompressed_len = self.buf.len();
        let compressed = zstd::bulk::compress(&self.buf, self.policy.zstd_level)?;
        let zstd_level = i16::try_from(self.policy.zstd_level).unwrap_or(0);
        let header = encode_segment_header(SegmentHeader {
            codec: CODEC_ZSTD,
            zstd_level,
            uncompressed_len: uncompressed_len as u64,
            compressed_len: compressed.len() as u64,
            frame_count: self.seg_frames,
        });
        {
            let mut f = fs::File::create(self.dir.join(&name))?;
            f.write_all(&OREC_MAGIC)?;
            f.write_all(&header)?;
            f.write_all(&compressed)?;
            f.flush()?;
        }
        self.manifest.segments.push(SegmentEntry {
            name,
            first_sim_t: self.seg_first_sim_t.unwrap_or(0),
            last_sim_t: self.seg_last_sim_t,
            frame_count: self.seg_frames,
            uncompressed_len: uncompressed_len as u64,
            compressed_len: compressed.len() as u64,
        });
        self.manifest_dirty = true;
        self.buf.clear();
        self.seg_frames = 0;
        self.seg_first_sim_t = None;
        self.next_segment += 1;
        self.write_manifest()
    }

    /// 原子重写 manifest：写 `manifest.json.tmp` → rename。
    fn write_manifest(&mut self) -> Result<(), Error> {
        let tmp = self.dir.join(format!("{MANIFEST_NAME}.tmp"));
        {
            let mut f = fs::File::create(&tmp)?;
            f.write_all(&serde_json::to_vec_pretty(&self.manifest)?)?;
            f.flush()?;
        }
        fs::rename(&tmp, self.dir.join(MANIFEST_NAME))?;
        self.manifest_dirty = false;
        Ok(())
    }
}

/// 段文件名：`seg_%06u.orec`。
pub fn segment_name(idx: usize) -> String {
    format!("seg_{idx:06}.orec")
}

fn now_unix_ms() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0)
}

/// 目录名探测：不存在、或存在但为空 → 可用；否则追加 `-2`、`-3` … 后缀。
fn resolve_session_dir(root: &Path, base: &str) -> Result<(String, PathBuf), Error> {
    let mut candidate = base.to_string();
    let mut attempt = 2u32;
    loop {
        let dir = root.join(&candidate);
        let usable = match fs::metadata(&dir) {
            Err(_) => true,
            Ok(m) => m.is_dir() && is_empty_dir(&dir),
        };
        if usable {
            return Ok((candidate, dir));
        }
        if attempt > 100 {
            return Err(Error::SessionDirConflict(base.to_string()));
        }
        candidate = format!("{base}-{attempt}");
        attempt += 1;
    }
}

fn is_empty_dir(dir: &Path) -> bool {
    fs::read_dir(dir)
        .map(|mut d| d.next().is_none())
        .unwrap_or(false)
}
