//! orbitx 黑匣子磁盘格式读写（schema 1）。
//!
//! 格式权威：[`docs/FLIGHT_RECORDER.md`](../../docs/FLIGHT_RECORDER.md)。
//! 一次会话一个目录：`manifest.json` + `seg_NNNNNN.orec`；
//! 段 = 魔数 `OXREC001` + 36B 定长头 + 整段 zstd payload；
//! 帧流 = length-delimited CBOR（`[u32 LE length][CBOR]`）。
//!
//! 分层边界：本 crate 只负责格式（帧类型 / 编码 / 会话写状态机 / 段解码），
//! 不依赖任何 orbitx 兄弟 crate；从 `Assembly` / `PlanetarySystem` 构建帧
//! 属于 `orbitx-runtime`，zenoh 查询服务属于 `orbitx-flightplayer`。

pub mod frame;
pub mod reader;
pub mod segment;
pub mod writer;

#[cfg(test)]
mod tests;

use std::fmt;
use std::path::PathBuf;

pub use frame::{
    BodyState, ControlInfo, DiagnosticsSnapshot, EnvironmentSnapshot, FlightInput, InputsSnapshot,
    RecorderFrame, RigidBodyState, RocketInfo, SessionEvent, SessionStartFrame, StepFrame,
    TerminalInfo, VesselBody, VesselInitial, SCHEMA_VERSION,
};
pub use reader::{list_sessions, SessionReader};
pub use segment::{
    decode_frame_stream, decode_segment, decode_segment_header, encode_frame,
    encode_segment_header, SegmentHeader, HEADER_SIZE,
};
pub use writer::{
    IndexEntry, Manifest, SegmentEntry, SegmentPolicy, SessionMeta, SessionWriter,
    DEFAULT_INDEX_EVERY, DEFAULT_MAX_FRAMES_PER_SEGMENT, DEFAULT_MAX_UNCOMPRESSED_BYTES,
    DEFAULT_ZSTD_LEVEL, MANIFEST_NAME,
};

/// 段文件魔数（ASCII `OXREC001`，8 字节）。
pub const OREC_MAGIC: [u8; 8] = *b"OXREC001";
/// 段头 `header_version`。
pub const HEADER_VERSION: u32 = 1;
/// 段头 `codec`：不压缩（仅开发）。
pub const CODEC_NONE: u16 = 0;
/// 段头 `codec`：zstd（产品默认）。
pub const CODEC_ZSTD: u16 = 1;

/// 格式层错误。
#[derive(Debug)]
pub enum Error {
    Io(std::io::Error),
    Json(serde_json::Error),
    Cbor(String),
    /// 单帧 CBOR 超过 u32 长度前缀上限。
    FrameTooLarge,
    /// 帧流被截断（长度前缀或 CBOR 体不完整）。
    TruncatedFrame,
    /// 段文件格式错误（魔数 / 头 / 尺寸不一致）。
    BadSegment {
        path: PathBuf,
        why: String,
    },
    /// 会话已 `finish` 后再写入。
    WriterClosed,
    /// 会话目录命名冲突且后缀探测耗尽。
    SessionDirConflict(String),
    /// `manifest.json` 缺失 / 非法 / schema 不符。
    Manifest {
        path: PathBuf,
        why: String,
    },
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io(e) => write!(f, "recorder io: {e}"),
            Self::Json(e) => write!(f, "recorder manifest json: {e}"),
            Self::Cbor(e) => write!(f, "recorder cbor: {e}"),
            Self::FrameTooLarge => write!(f, "recorder frame exceeds u32 length prefix"),
            Self::TruncatedFrame => write!(f, "recorder frame stream truncated"),
            Self::BadSegment { path, why } => {
                write!(f, "recorder bad segment {}: {why}", path.display())
            }
            Self::WriterClosed => write!(f, "recorder session already finished"),
            Self::SessionDirConflict(id) => write!(f, "recorder session dir conflict: {id}"),
            Self::Manifest { path, why } => {
                write!(f, "recorder manifest {}: {why}", path.display())
            }
        }
    }
}

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Io(e) => Some(e),
            Self::Json(e) => Some(e),
            _ => None,
        }
    }
}

impl From<std::io::Error> for Error {
    fn from(e: std::io::Error) -> Self {
        Self::Io(e)
    }
}

impl From<serde_json::Error> for Error {
    fn from(e: serde_json::Error) -> Self {
        Self::Json(e)
    }
}
