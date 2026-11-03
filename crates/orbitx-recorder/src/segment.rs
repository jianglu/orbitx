//! 段文件编码（`*.orec`）：魔数 + 36B 定长头 + payload；帧流 = length-delimited CBOR。
//!
//! 字节布局与字段序冻结于 `FLIGHT_RECORDER.md`「*.orec 段文件」；此处是唯一实现。

use std::path::{Path, PathBuf};

use crate::frame::RecorderFrame;
use crate::{Error, CODEC_NONE, CODEC_ZSTD, HEADER_VERSION, OREC_MAGIC, SCHEMA_VERSION};

/// 段头定长字节数：u32 + u32 + u16 + i16 + u64 + u64 + u32 + u32(reserved)。
pub const HEADER_SIZE: usize = 36;

/// 段头字段（schema 1）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SegmentHeader {
    /// `1` = zstd，`0` = none（仅开发）。
    pub codec: u16,
    /// codec=zstd 时的压缩级，否则 0。
    pub zstd_level: i16,
    pub uncompressed_len: u64,
    pub compressed_len: u64,
    pub frame_count: u32,
}

/// 编码 36B 定长段头（小端；reserved 恒 0）。
pub fn encode_segment_header(h: SegmentHeader) -> [u8; HEADER_SIZE] {
    let mut b = [0u8; HEADER_SIZE];
    b[0..4].copy_from_slice(&HEADER_VERSION.to_le_bytes());
    b[4..8].copy_from_slice(&SCHEMA_VERSION.to_le_bytes());
    b[8..10].copy_from_slice(&h.codec.to_le_bytes());
    b[10..12].copy_from_slice(&h.zstd_level.to_le_bytes());
    b[12..20].copy_from_slice(&h.uncompressed_len.to_le_bytes());
    b[20..28].copy_from_slice(&h.compressed_len.to_le_bytes());
    b[28..32].copy_from_slice(&h.frame_count.to_le_bytes());
    b
}

/// 解码 36B 定长段头，校验 header/schema 版本。
pub fn decode_segment_header(buf: &[u8; HEADER_SIZE]) -> Result<SegmentHeader, Error> {
    let header_version = u32::from_le_bytes(buf[0..4].try_into().expect("4 bytes"));
    if header_version != HEADER_VERSION {
        return Err(Error::BadSegment {
            path: PathBuf::new(),
            why: format!("header_version {header_version} != {HEADER_VERSION}"),
        });
    }
    let schema_version = u32::from_le_bytes(buf[4..8].try_into().expect("4 bytes"));
    if schema_version != SCHEMA_VERSION {
        return Err(Error::BadSegment {
            path: PathBuf::new(),
            why: format!("schema_version {schema_version} != {SCHEMA_VERSION}"),
        });
    }
    Ok(SegmentHeader {
        codec: u16::from_le_bytes(buf[8..10].try_into().expect("2 bytes")),
        zstd_level: i16::from_le_bytes(buf[10..12].try_into().expect("2 bytes")),
        uncompressed_len: u64::from_le_bytes(buf[12..20].try_into().expect("8 bytes")),
        compressed_len: u64::from_le_bytes(buf[20..28].try_into().expect("8 bytes")),
        frame_count: u32::from_le_bytes(buf[28..32].try_into().expect("4 bytes")),
    })
}

/// 追加一帧到帧流：`[u32 LE length][CBOR]`。
pub fn encode_frame(frame: &RecorderFrame, out: &mut Vec<u8>) -> Result<(), Error> {
    let start = out.len();
    out.extend_from_slice(&0u32.to_le_bytes());
    ciborium::ser::into_writer(frame, &mut *out).map_err(|e| Error::Cbor(e.to_string()))?;
    let len = out.len() - start - 4;
    let len = u32::try_from(len).map_err(|_| Error::FrameTooLarge)?;
    out[start..start + 4].copy_from_slice(&len.to_le_bytes());
    Ok(())
}

/// 逐帧解码 length-delimited CBOR 帧流。
pub fn decode_frame_stream(buf: &[u8]) -> Result<Vec<RecorderFrame>, Error> {
    let mut frames = Vec::new();
    let mut off = 0usize;
    while off < buf.len() {
        if buf.len() - off < 4 {
            return Err(Error::TruncatedFrame);
        }
        let len = u32::from_le_bytes(buf[off..off + 4].try_into().expect("4 bytes")) as usize;
        off += 4;
        if buf.len() - off < len {
            return Err(Error::TruncatedFrame);
        }
        let frame: RecorderFrame = ciborium::de::from_reader(&buf[off..off + len])
            .map_err(|e| Error::Cbor(e.to_string()))?;
        frames.push(frame);
        off += len;
    }
    Ok(frames)
}

/// 解码整个段文件（当前全量解码，供 Writer 测试与小型段读取；
/// 流式帧迭代随 Reader 提供）。
pub fn decode_segment(path: &Path) -> Result<Vec<RecorderFrame>, Error> {
    let raw = std::fs::read(path)?;
    let bad = |why: String| Error::BadSegment {
        path: path.to_path_buf(),
        why,
    };
    if raw.len() < 8 + HEADER_SIZE {
        return Err(bad(format!(
            "file shorter than magic+header ({} bytes)",
            raw.len()
        )));
    }
    if raw[0..8] != OREC_MAGIC {
        return Err(bad("bad magic".into()));
    }
    let header: [u8; HEADER_SIZE] = raw[8..8 + HEADER_SIZE].try_into().expect("36 bytes");
    let h = decode_segment_header(&header).map_err(|e| bad(e.to_string()))?;
    let payload = &raw[8 + HEADER_SIZE..];
    if payload.len() as u64 != h.compressed_len {
        return Err(bad(format!(
            "payload {} bytes != compressed_len {}",
            payload.len(),
            h.compressed_len
        )));
    }
    let uncompressed = match h.codec {
        CODEC_ZSTD => zstd::decode_all(payload).map_err(|e| bad(format!("zstd: {e}")))?,
        CODEC_NONE => payload.to_vec(),
        other => return Err(bad(format!("unknown codec {other}"))),
    };
    if uncompressed.len() as u64 != h.uncompressed_len {
        return Err(bad(format!(
            "uncompressed {} bytes != uncompressed_len {}",
            uncompressed.len(),
            h.uncompressed_len
        )));
    }
    let frames = decode_frame_stream(&uncompressed)?;
    if frames.len() as u32 != h.frame_count {
        return Err(bad(format!(
            "decoded {} frames != frame_count {}",
            frames.len(),
            h.frame_count
        )));
    }
    Ok(frames)
}
