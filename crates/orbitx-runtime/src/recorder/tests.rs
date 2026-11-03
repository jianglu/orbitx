use super::{civil_from_days, new_session_id, RecorderEnqueue, RecorderIo};
use crate::input::{InputCmd, SessionCmd};
use crate::shutdown::ShutdownFlag;
use orbitx_recorder::frame::{FlightInput, RecorderFrame, SessionEvent, SessionStartFrame};
use orbitx_recorder::SessionMeta;
use std::time::{Duration, SystemTime};

/// civil_from_days 对已知 UTC 日期的锚点（1970-01-01 / 2000-03-01 / 2024-02-29）。
#[test]
fn civil_from_days_known_dates() {
    assert_eq!(civil_from_days(0), (1970, 1, 1));
    // Hinnant 论文锚点：2000-03-01 = day 11017。
    assert_eq!(civil_from_days(11_017), (2000, 3, 1));
    // 2024-02-29 = 19782 天（闰日）。
    assert_eq!(civil_from_days(19_782), (2024, 2, 29));
}

/// session_id 形如 `<火箭ID>_<hex4>_<YYYY-MM-DD_HHMMSS>`；火箭名净化为安全字符。
#[test]
fn new_session_id_format() {
    let now = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_millis() as u64;
    let id = new_session_id("Test_1", now);
    let parts: Vec<&str> = id.split('_').collect();
    assert!(id.starts_with("Test_1_"), "id: {id}");
    assert_eq!(parts.len(), 5, "rocket(2段)_hex4_date_time: {id}");
    assert_eq!(parts[2].len(), 4, "hex4");
    assert!(parts[2].chars().all(|c| c.is_ascii_hexdigit()));
    let (date, time) = (parts[3], parts[4]);
    let date: Vec<&str> = date.split('-').collect();
    assert_eq!(date.len(), 3, "YYYY-MM-DD: {id}");
    assert_eq!(date[0].len(), 4);
    assert_eq!(date[1].len(), 2);
    assert_eq!(date[2].len(), 2);
    assert!(date.iter().all(|s| s.chars().all(|c| c.is_ascii_digit())));
    assert_eq!(time.len(), 6);
    assert!(time.chars().all(|c| c.is_ascii_digit()));

    // 非法字符净化（中文/空格/斜杠 → `_`）+ 空名兜底。
    let id = new_session_id("星舰 9/Pro", now);
    assert!(id.starts_with("___9_Pro_"), "sanitize: {id}");
    assert!(new_session_id("", now).starts_with("rocket_"));
}

/// 映射：飞行输入 → 帧摘要（值透传）。
#[test]
fn flight_input_mapping() {
    let f = super::frame::flight_input(&InputCmd::SetThrottle { level: 0.75 });
    assert_eq!(f, FlightInput::SetThrottle { level: 0.75 });
    let f = super::frame::flight_input(&InputCmd::SetAttitudeAxes {
        pitch: 1.0,
        yaw: 2.0,
        roll: 3.0,
    });
    assert_eq!(
        f,
        FlightInput::SetAttitudeAxes {
            pitch: 1.0,
            yaw: 2.0,
            roll: 3.0
        }
    );
    assert_eq!(
        super::frame::flight_input(&InputCmd::Separate),
        FlightInput::Separate
    );
    assert_eq!(
        super::frame::flight_input(&InputCmd::SetGravityTurn { enabled: true }),
        FlightInput::SetGravityTurn { enabled: true }
    );
    assert_eq!(
        super::frame::flight_input(&InputCmd::WorkflowIgnite),
        FlightInput::WorkflowIgnite
    );
    assert_eq!(
        super::frame::flight_input(&InputCmd::SetRcsEnabled { enabled: false }),
        FlightInput::SetRcsEnabled { enabled: false }
    );
}

/// 映射：会话命令 → 帧摘要；Shutdown / Reset 不入帧（会话边界由 session_start 表达）。
#[test]
fn session_event_filters_boundary_cmds() {
    assert_eq!(
        super::frame::session_event(&SessionCmd::Pause),
        Some(SessionEvent::Pause)
    );
    assert_eq!(
        super::frame::session_event(&SessionCmd::Resume),
        Some(SessionEvent::Resume)
    );
    assert_eq!(
        super::frame::session_event(&SessionCmd::SetWarp { scale: 10.0 }),
        Some(SessionEvent::SetWarp { scale: 10.0 })
    );
    assert_eq!(
        super::frame::session_event(&SessionCmd::Step { n: 5 }),
        Some(SessionEvent::Step { n: 5 })
    );
    assert_eq!(
        super::frame::session_event(&SessionCmd::Launch),
        Some(SessionEvent::Launch)
    );
    assert_eq!(
        super::frame::session_event(&SessionCmd::SetSurfaceWorkingMaxLevel { level: 3 }),
        Some(SessionEvent::SetSurfaceWorkingMaxLevel { level: 3 })
    );
    assert_eq!(super::frame::session_event(&SessionCmd::Shutdown), None);
    assert_eq!(super::frame::session_event(&SessionCmd::Reset), None);
}

/// disabled 入队：恒接受、恒健康、水位恒放行。
#[test]
fn disabled_enqueue_is_noop() {
    let q = RecorderEnqueue::disabled();
    assert!(!q.is_enabled());
    assert!(q.accepts());
    assert!(q.healthy());
    assert!(q.try_enqueue(dummy_session_start()));
}

/// pair + handle：session_start 开目录 → step 落帧 → finalize 终版 manifest。
#[test]
fn recorder_io_writes_session_artifacts() {
    let tmp = tempfile::tempdir().unwrap();
    let (enq, mut io) = RecorderIo::pair(
        tmp.path().to_path_buf(),
        new_session_id("Test_1", 0),
        SessionMeta {
            drive: "client-step".into(),
            sim_dt_ms: 20,
            created_unix_ms: None,
        },
    );
    assert!(enq.is_enabled());

    assert!(io.handle(dummy_session_start()));
    for i in 0..3 {
        assert!(io.handle(dummy_step(i)));
    }
    assert_eq!(io.written_count(), 4);
    io.finalize();

    let dirs: Vec<_> = std::fs::read_dir(tmp.path())
        .unwrap()
        .map(|e| e.unwrap().path())
        .collect();
    assert_eq!(dirs.len(), 1, "one session dir");
    let manifest = std::fs::read_to_string(dirs[0].join("manifest.json")).unwrap();
    assert!(
        manifest.contains("\"schema_version\""),
        "manifest: {manifest}"
    );
    assert!(manifest.contains("\"drive\"") && manifest.contains("client-step"));
    assert!(manifest.contains("seg_000000.orec"), "manifest: {manifest}");
    let seg = std::fs::read(dirs[0].join("seg_000000.orec")).unwrap();
    assert_eq!(&seg[..8], b"OXREC001");
    assert_eq!(enq.dropped_count(), 0);
}

/// 多帧单段冒烟（滚动语义由 orbitx-recorder 的 writer 测试覆盖）。
#[test]
fn recorder_io_multi_frame_single_segment() {
    let tmp = tempfile::tempdir().unwrap();
    let (_enq, mut io) = RecorderIo::pair(
        tmp.path().to_path_buf(),
        new_session_id("Test_1", 0),
        SessionMeta {
            drive: "self-paced".into(),
            sim_dt_ms: 20,
            created_unix_ms: None,
        },
    );
    assert!(io.handle(dummy_session_start()));
    for i in 0..16 {
        assert!(io.handle(dummy_step(i)));
    }
    io.finalize();
    let dirs: Vec<_> = std::fs::read_dir(tmp.path())
        .unwrap()
        .map(|e| e.unwrap().path())
        .collect();
    assert_eq!(dirs.len(), 1);
    assert!(dirs[0].join("manifest.json").exists());
}

/// step 帧在无会话头时被丢弃（B3 保证 session_start 必先入队；此为防御断言）。
#[test]
fn recorder_io_drops_step_without_session() {
    let tmp = tempfile::tempdir().unwrap();
    let (_enq, mut io) = RecorderIo::pair(
        tmp.path().to_path_buf(),
        new_session_id("Test_1", 0),
        SessionMeta {
            drive: "self-paced".into(),
            sim_dt_ms: 20,
            created_unix_ms: None,
        },
    );
    assert!(!io.handle(dummy_step(0)));
    io.finalize();
    assert!(
        std::fs::read_dir(tmp.path()).unwrap().next().is_none(),
        "no session dir should be created"
    );
}

/// IO 任务端到端：run() 消费 L1 → shutdown 后 drain + finalize（终版 manifest）。
#[tokio::test]
async fn recorder_io_run_drains_and_finalizes() {
    let tmp = tempfile::tempdir().unwrap();
    let (enq, io) = RecorderIo::pair(
        tmp.path().to_path_buf(),
        new_session_id("Test_1", 0),
        SessionMeta {
            drive: "self-paced".into(),
            sim_dt_ms: 20,
            created_unix_ms: None,
        },
    );
    let shutdown = ShutdownFlag::new();
    let task = tokio::spawn(io.run(shutdown.clone()));

    assert!(enq.try_enqueue(dummy_session_start()));
    for i in 0..5 {
        assert!(enq.try_enqueue(dummy_step(i)));
    }
    // 轮询等待 IO 消费 L1（帧缓冲在段滚动/finish 才写盘，不能等 seg 文件）。
    for _ in 0..400 {
        if enq.queued_len() == 0 {
            break;
        }
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    assert_eq!(enq.queued_len(), 0, "L1 should be drained");
    shutdown.request();
    task.await.unwrap();

    let dirs: Vec<_> = std::fs::read_dir(tmp.path())
        .unwrap()
        .map(|e| e.unwrap().path())
        .collect();
    assert_eq!(dirs.len(), 1);
    let manifest = std::fs::read_to_string(dirs[0].join("manifest.json")).unwrap();
    assert!(manifest.contains("seg_000000.orec"), "manifest: {manifest}");
    assert!(dirs[0].join("seg_000000.orec").exists());
}

fn dummy_session_start() -> RecorderFrame {
    RecorderFrame::SessionStart(SessionStartFrame {
        schema_version: orbitx_recorder::frame::SCHEMA_VERSION,
        sim_t: 0,
        sim_dt_ms: 20,
        rocket: orbitx_recorder::frame::RocketInfo {
            name: "test".into(),
            class: "Test".into(),
            stage_count: 1,
        },
        control: orbitx_recorder::frame::ControlInfo {
            mode: "base".into(),
            workflow_kind: None,
            gravity_turn: false,
            rcs_enabled: false,
            throttle: 0.0,
        },
        environment: orbitx_recorder::frame::EnvironmentSnapshot {
            mjd: 0.0,
            star: None,
            bodies: Vec::new(),
        },
        vessels: Vec::new(),
    })
}

fn dummy_step(i: u64) -> RecorderFrame {
    RecorderFrame::Step(orbitx_recorder::frame::StepFrame {
        sim_t: 20 * (i + 1),
        step_index: i + 1,
        inputs: orbitx_recorder::frame::InputsSnapshot {
            flight: Vec::new(),
            session: Vec::new(),
            warp: 1.0,
            paused: false,
        },
        environment: orbitx_recorder::frame::EnvironmentSnapshot {
            mjd: 0.0,
            star: None,
            bodies: Vec::new(),
        },
        separated: Vec::new(),
        terminal: orbitx_recorder::frame::TerminalInfo {
            crash_msg: None,
            landed_msg: None,
            launch_failed_msg: None,
        },
        bodies: Vec::new(),
    })
}
