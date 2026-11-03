//! orbitx-recorder 单元测试：帧 roundtrip、段头、帧流、写状态机（滚动/索引/manifest/冲突）。

use std::fs;

use tempfile::TempDir;

use crate::frame::{
    BodyState, ControlInfo, DiagnosticsSnapshot, EnvironmentSnapshot, FlightInput, InputsSnapshot,
    RecorderFrame, RigidBodyState, RocketInfo, SessionEvent, SessionStartFrame, StepFrame,
    TerminalInfo, VesselBody, VesselInitial,
};
use crate::segment::{
    decode_frame_stream, decode_segment, decode_segment_header, encode_frame,
    encode_segment_header, SegmentHeader, HEADER_SIZE,
};
use crate::writer::{Manifest, SegmentPolicy, SessionMeta, SessionWriter, MANIFEST_NAME};
use crate::Error;

fn rigid_state(seed: f64) -> RigidBodyState {
    RigidBodyState {
        pos: [seed, seed + 1.0, seed + 2.0],
        vel: [0.1, 0.2, 0.3],
        rot: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        omega: [0.0, 0.0, 0.0],
        q: [0.0, 0.0, 0.0, 1.0],
    }
}

fn env_snapshot(star: bool) -> EnvironmentSnapshot {
    EnvironmentSnapshot {
        mjd: 60_000.5,
        star: star.then(|| "Sun".to_string()),
        bodies: vec![BodyState {
            name: "Earth".into(),
            pos: [0.0, 0.0, 0.0],
            rotation_rad: 1.25,
            rotation_enabled: true,
            dynamics: true,
        }],
    }
}

fn diag(seed: f64) -> DiagnosticsSnapshot {
    DiagnosticsSnapshot {
        a_grav: seed,
        g_multiple: seed + 1.0,
        mach: seed + 2.0,
        density: seed + 3.0,
        dynamic_pressure: seed + 4.0,
        pressure: seed + 5.0,
        thrust_atm_scale: seed + 6.0,
        isp_eff: seed + 7.0,
        drag_force: seed + 8.0,
        cd_eff: seed + 9.0,
        temperature: seed + 10.0,
        sound_speed: seed + 11.0,
        load_factor: seed + 12.0,
    }
}

fn session_frame(sim_t: u64) -> RecorderFrame {
    RecorderFrame::SessionStart(SessionStartFrame {
        schema_version: crate::SCHEMA_VERSION,
        sim_t,
        sim_dt_ms: 20,
        rocket: RocketInfo {
            name: "Falcon 9".into(),
            class: "Falcon9".into(),
            stage_count: 2,
        },
        control: ControlInfo {
            mode: "target".into(),
            workflow_kind: None,
            gravity_turn: true,
            rcs_enabled: false,
            throttle: 0.8,
        },
        environment: env_snapshot(true),
        vessels: vec![VesselInitial {
            index: 0,
            name: "Stage1".into(),
            detached: false,
            state: rigid_state(3.0),
            fuel_kg: 400_000.0,
            initial_fuel_kg: 400_000.0,
        }],
    })
}

fn step_frame(sim_t: u64, step_index: u64, separated: bool) -> RecorderFrame {
    RecorderFrame::Step(StepFrame {
        sim_t,
        step_index,
        inputs: InputsSnapshot {
            flight: if separated {
                vec![FlightInput::Separate]
            } else {
                vec![FlightInput::SetThrottle { level: 1.0 }]
            },
            session: vec![SessionEvent::SetWarp { scale: 2.0 }],
            warp: 2.0,
            paused: false,
        },
        environment: env_snapshot(false),
        separated: if separated { vec![1] } else { vec![] },
        terminal: TerminalInfo::default(),
        bodies: vec![
            VesselBody {
                index: 0,
                name: "Stage1".into(),
                detached: false,
                crashed: false,
                state: rigid_state(sim_t as f64 * 0.001),
                fuel_kg: 100.0,
                thrust_level: 0.9,
                diagnostics: diag(1.0),
            },
            VesselBody {
                index: 1,
                name: "Stage2".into(),
                detached: separated,
                crashed: false,
                state: rigid_state(9.0),
                fuel_kg: 50.0,
                thrust_level: 0.0,
                diagnostics: diag(2.0),
            },
        ],
    })
}

fn small_policy() -> SegmentPolicy {
    SegmentPolicy {
        max_uncompressed_bytes: usize::MAX,
        max_frames: 3,
        index_every: 2,
        zstd_level: 1,
    }
}

fn meta() -> SessionMeta {
    SessionMeta {
        drive: "client-step".into(),
        sim_dt_ms: 20,
        created_unix_ms: Some(1_700_000_000_000),
    }
}

#[test]
fn segment_header_roundtrip() {
    let h = SegmentHeader {
        codec: crate::CODEC_ZSTD,
        zstd_level: 3,
        uncompressed_len: 12_345,
        compressed_len: 678,
        frame_count: 42,
    };
    let bytes = encode_segment_header(h);
    assert_eq!(bytes.len(), HEADER_SIZE);
    assert_eq!(decode_segment_header(&bytes).unwrap(), h);
    // reserved 恒 0
    assert!(bytes[32..36].iter().all(|&b| b == 0));
    // 版本校验
    let mut bad = bytes;
    bad[4..8].copy_from_slice(&99u32.to_le_bytes());
    assert!(matches!(
        decode_segment_header(&bad),
        Err(Error::BadSegment { .. })
    ));
}

#[test]
fn frame_cbor_roundtrip_session_start() {
    let f = session_frame(0);
    let mut buf = Vec::new();
    encode_frame(&f, &mut buf).unwrap();
    let back: RecorderFrame = ciborium::de::from_reader(&buf[4..]).expect("decode single frame");
    assert_eq!(back, f);
    assert_eq!(back.sim_t(), 0);
}

#[test]
fn frame_cbor_roundtrip_step_and_inputs() {
    let f = step_frame(20_000, 1000, true);
    assert_eq!(f.sim_t(), 20_000);
    // 验证 kind tag 与分离 / 输入字段进入 CBOR
    let mut buf = Vec::new();
    encode_frame(&f, &mut buf).unwrap();
    let back: RecorderFrame = ciborium::de::from_reader(&buf[4..]).expect("decode step frame");
    assert_eq!(back, f);
    let crate::frame::RecorderFrame::Step(ref s) = back else {
        panic!("expected step frame")
    };
    assert_eq!(s.separated, vec![1]);
    assert_eq!(s.inputs.flight, vec![FlightInput::Separate]);
    assert_eq!(s.inputs.session, vec![SessionEvent::SetWarp { scale: 2.0 }]);
    assert!(!s.terminal.is_terminal());
    // 终局帧字段
    let mut term = step_frame(20_040, 1001, false);
    if let RecorderFrame::Step(ref mut s) = term {
        s.terminal = TerminalInfo {
            crash_msg: Some("坠入水面".into()),
            landed_msg: None,
            launch_failed_msg: None,
        };
    }
    let mut buf2 = Vec::new();
    encode_frame(&term, &mut buf2).unwrap();
    let back2: RecorderFrame =
        ciborium::de::from_reader(&buf2[4..]).expect("decode terminal frame");
    assert_eq!(back2, term);
    assert!(back2.sim_t() == 20_040);
}

#[test]
fn frame_stream_roundtrip() {
    let frames = vec![
        session_frame(0),
        step_frame(20, 1, false),
        step_frame(40, 2, false),
    ];
    let mut buf = Vec::new();
    for f in &frames {
        encode_frame(f, &mut buf).unwrap();
    }
    let back = decode_frame_stream(&buf).unwrap();
    assert_eq!(back, frames);
}

#[test]
fn frame_stream_truncated() {
    let mut buf = Vec::new();
    encode_frame(&step_frame(0, 0, false), &mut buf).unwrap();
    buf.truncate(buf.len() - 1);
    assert!(matches!(
        decode_frame_stream(&buf),
        Err(Error::TruncatedFrame)
    ));
}

#[test]
fn writer_creates_manifest() {
    let tmp = TempDir::new().unwrap();
    let mut w =
        SessionWriter::create(tmp.path(), "20261103_120000_ab12", meta(), small_policy()).unwrap();
    assert_eq!(w.session_id(), "20261103_120000_ab12");
    assert!(w.dir().join(MANIFEST_NAME).is_file());
    let m: Manifest =
        serde_json::from_str(&fs::read_to_string(w.dir().join(MANIFEST_NAME)).unwrap()).unwrap();
    assert_eq!(m.schema_version, crate::SCHEMA_VERSION);
    assert_eq!(m.sim_dt_ms, 20);
    assert_eq!(m.drive, "client-step");
    assert_eq!(m.created_unix_ms, 1_700_000_000_000);
    assert_eq!(m.codec, "zstd");
    assert_eq!(m.zstd_level, 1);
    assert!(m.segments.is_empty());
    assert!(m.index.is_empty());
    w.finish().unwrap();
}

#[test]
fn writer_roundtrip_without_roll() {
    let tmp = TempDir::new().unwrap();
    let mut w = SessionWriter::create(tmp.path(), "sess", meta(), small_policy()).unwrap();
    let frames = vec![
        session_frame(0),
        step_frame(20, 1, false),
        step_frame(40, 2, false),
    ];
    for f in &frames {
        w.push_frame(f).unwrap();
    }
    assert_eq!(w.frames_written(), 3);
    w.finish().unwrap();
    let segs = fs::read_dir(w.dir()).unwrap().count();
    // manifest + 1 段
    assert_eq!(segs, 2);
    let back = decode_segment(&w.dir().join(crate::writer::segment_name(0))).unwrap();
    assert_eq!(back, frames);
    let m: Manifest =
        serde_json::from_str(&fs::read_to_string(w.dir().join(MANIFEST_NAME)).unwrap()).unwrap();
    assert_eq!(m.segments.len(), 1);
    assert_eq!(m.segments[0].frame_count, 3);
    assert_eq!(m.segments[0].first_sim_t, 0);
    assert_eq!(m.segments[0].last_sim_t, 40);
}

#[test]
fn writer_rolls_segments_and_indexes() {
    let tmp = TempDir::new().unwrap();
    let mut w = SessionWriter::create(tmp.path(), "sess", meta(), small_policy()).unwrap();
    for i in 0..10u64 {
        w.push_frame(&step_frame((i + 1) * 20, i, false)).unwrap();
    }
    w.finish().unwrap();
    let m: Manifest =
        serde_json::from_str(&fs::read_to_string(w.dir().join(MANIFEST_NAME)).unwrap()).unwrap();
    // max_frames=3 → 4 段：3+3+3+1
    assert_eq!(m.segments.len(), 4);
    let total: u32 = m.segments.iter().map(|s| s.frame_count).sum();
    assert_eq!(total, 10);
    // 段范围连续
    assert_eq!(m.segments[0].first_sim_t, 20);
    assert_eq!(m.segments[0].last_sim_t, 60);
    assert_eq!(m.segments[3].frame_count, 1);
    assert_eq!(m.segments[3].last_sim_t, 200);
    // 稀疏索引：ordinal+1 % 2 == 0 → ordinals 1,3,5,7,9
    assert_eq!(m.index.len(), 5);
    assert_eq!(m.index[0].frame_ordinal, 1);
    assert_eq!(m.index[0].sim_t, 40);
    assert_eq!(m.index[0].segment, "seg_000000.orec");
    assert_eq!(m.index[1].segment, "seg_000001.orec");
    assert_eq!(m.index[4].frame_ordinal, 9);
    assert_eq!(m.index[4].segment, "seg_000003.orec");
    // 每段可独立解码且帧序保持
    let all: Vec<RecorderFrame> = (0..4)
        .flat_map(|i| decode_segment(&w.dir().join(crate::writer::segment_name(i))).unwrap())
        .collect();
    assert_eq!(all.len(), 10);
    assert_eq!(all[9].sim_t(), 200);
}

#[test]
fn writer_finish_idempotent_and_closed() {
    let tmp = TempDir::new().unwrap();
    let mut w = SessionWriter::create(tmp.path(), "sess", meta(), small_policy()).unwrap();
    w.push_frame(&step_frame(20, 0, false)).unwrap();
    w.finish().unwrap();
    w.finish().unwrap();
    assert!(w.is_finished());
    assert!(matches!(
        w.push_frame(&step_frame(40, 1, false)),
        Err(Error::WriterClosed)
    ));
    let segs = fs::read_dir(w.dir()).unwrap().count();
    assert_eq!(segs, 2); // manifest + 1 段，无重复
}

#[test]
fn writer_empty_session_manifest_only() {
    let tmp = TempDir::new().unwrap();
    let mut w = SessionWriter::create(tmp.path(), "sess", meta(), small_policy()).unwrap();
    w.finish().unwrap();
    let segs: Vec<_> = fs::read_dir(w.dir())
        .unwrap()
        .map(|e| e.unwrap().file_name().into_string().unwrap())
        .collect();
    assert_eq!(segs, vec![MANIFEST_NAME.to_string()]);
}

#[test]
fn writer_session_dir_conflict_suffix() {
    let tmp = TempDir::new().unwrap();
    let mut w1 = SessionWriter::create(tmp.path(), "dup", meta(), small_policy()).unwrap();
    w1.push_frame(&step_frame(20, 0, false)).unwrap();
    // 非目录文件占用 dup-2 名字 → 探测到 dup-3
    fs::write(tmp.path().join("dup-2"), b"x").unwrap();
    let w3 = SessionWriter::create(tmp.path(), "dup", meta(), small_policy()).unwrap();
    assert_eq!(w3.session_id(), "dup-3");
}

#[test]
fn writer_session_dir_empty_reused() {
    let tmp = TempDir::new().unwrap();
    fs::create_dir(tmp.path().join("reuse")).unwrap();
    let w = SessionWriter::create(tmp.path(), "reuse", meta(), small_policy()).unwrap();
    assert_eq!(w.session_id(), "reuse");
}

#[test]
fn decode_segment_bad_magic_and_short() {
    let tmp = TempDir::new().unwrap();
    let p = tmp.path().join("bad.orec");
    fs::write(&p, b"NOTMAGIC").unwrap();
    assert!(matches!(decode_segment(&p), Err(Error::BadSegment { .. })));
    let p2 = tmp.path().join("short.orec");
    fs::write(&p2, b"OX").unwrap();
    assert!(matches!(decode_segment(&p2), Err(Error::BadSegment { .. })));
}

#[test]
fn decode_segment_detects_frame_count_mismatch() {
    let tmp = TempDir::new().unwrap();
    let mut w = SessionWriter::create(tmp.path(), "sess", meta(), small_policy()).unwrap();
    w.push_frame(&step_frame(20, 0, false)).unwrap();
    w.finish().unwrap();
    let mut raw = fs::read(w.dir().join(crate::writer::segment_name(0))).unwrap();
    // 篡改 frame_count（头部偏移 8+28）
    let off = 8 + 28;
    let fc = u32::from_le_bytes(raw[off..off + 4].try_into().unwrap());
    raw[off..off + 4].copy_from_slice(&(fc + 1).to_le_bytes());
    let p = tmp.path().join("tampered.orec");
    fs::write(&p, &raw).unwrap();
    assert!(
        matches!(decode_segment(&p), Err(Error::BadSegment { why, .. }) if why.contains("frames"))
    );
}

// ---------- Reader ----------

/// 写两段（小滚动阈值）→ list_sessions / open / frames 全量与顺序。
#[test]
fn reader_lists_and_reads_full_session() {
    let tmp = TempDir::new().unwrap();
    let policy = SegmentPolicy {
        max_uncompressed_bytes: usize::MAX,
        max_frames: 4,
        index_every: 1000,
        zstd_level: 3,
    };
    let mut w = SessionWriter::create(
        tmp.path(),
        "20260101_000000_abcd",
        SessionMeta {
            drive: "client-step".into(),
            sim_dt_ms: 20,
            created_unix_ms: None,
        },
        policy,
    )
    .unwrap();
    w.push_frame(&session_frame(0)).unwrap();
    for i in 0..10u64 {
        w.push_frame(&step_frame(20 * (i + 1), i + 1, false))
            .unwrap();
    }
    w.finish().unwrap();

    let sessions = crate::reader::list_sessions(tmp.path()).unwrap();
    assert_eq!(sessions.len(), 1);

    let r = crate::reader::SessionReader::open(&sessions[0]).unwrap();
    assert_eq!(r.manifest().schema_version, 1);
    assert_eq!(r.manifest().segments.len(), 3, "4+4+3 frames");
    let frames = r.frames().unwrap();
    assert_eq!(frames.len(), 11);
    assert!(matches!(frames[0], RecorderFrame::SessionStart(_)));
    assert_eq!(frames[10].sim_t(), 200);
}

/// frames_between：段级剪枝 + 帧内过滤（跨段边界区间）。
#[test]
fn reader_frames_between_filters_by_sim_t() {
    let tmp = TempDir::new().unwrap();
    let policy = SegmentPolicy {
        max_uncompressed_bytes: usize::MAX,
        max_frames: 3,
        index_every: 1000,
        zstd_level: 3,
    };
    let mut w = SessionWriter::create(
        tmp.path(),
        "20260101_000000_abcd",
        SessionMeta {
            drive: "self-paced".into(),
            sim_dt_ms: 20,
            created_unix_ms: None,
        },
        policy,
    )
    .unwrap();
    w.push_frame(&session_frame(0)).unwrap();
    for i in 0..9u64 {
        w.push_frame(&step_frame(20 * (i + 1), i + 1, false))
            .unwrap();
    }
    w.finish().unwrap();

    let dir = crate::reader::list_sessions(tmp.path()).unwrap().remove(0);
    let r = crate::reader::SessionReader::open(&dir).unwrap();
    // [30, 90]：步帧在 20 的倍数上，区间内为 40/60/80。
    let got = r.frames_between(30, 90).unwrap();
    assert_eq!(
        got.iter().map(|f| f.sim_t()).collect::<Vec<_>>(),
        vec![40, 60, 80]
    );
    // 全范围含 session_start。
    let all = r.frames_between(0, u64::MAX).unwrap();
    assert_eq!(all.len(), 10);
    assert!(matches!(all[0], RecorderFrame::SessionStart(_)));
}

/// open 校验：manifest 缺失 / schema 不符 → Err。
#[test]
fn reader_rejects_bad_manifest() {
    let tmp = TempDir::new().unwrap();
    let dir = tmp.path().join("20260101_000000_abcd");
    fs::create_dir_all(&dir).unwrap();
    assert!(matches!(
        crate::reader::SessionReader::open(&dir),
        Err(Error::Io(_))
    ));
    fs::write(dir.join(MANIFEST_NAME), r#"{"schema_version": 99}"#).unwrap();
    assert!(matches!(
        crate::reader::SessionReader::open(&dir),
        Err(Error::Manifest { .. })
    ));
}
