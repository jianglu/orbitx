use super::{civil_from_days, new_session_id, RecorderCmd, RecorderEnqueue, RecorderIo};
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
    assert_eq!(
        super::frame::session_event(&SessionCmd::HaltRequest {
            reason: crate::input::HaltReason::Completion
        }),
        Some(SessionEvent::HaltRequest {
            reason: orbitx_recorder::frame::HaltReason::Completion
        })
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

    assert!(io.handle_frame(dummy_session_start()));
    for i in 0..3 {
        assert!(io.handle_frame(dummy_step(i)));
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
    assert!(io.handle_frame(dummy_session_start()));
    for i in 0..16 {
        assert!(io.handle_frame(dummy_step(i)));
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
    assert!(!io.handle_frame(dummy_step(0)));
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

/// HaltFlush 收尾四步：drain 残余帧（保序）→ finish → 置 None → reply 回实际目录；
/// 后续步帧走「无会话头丢弃」warn 路径（优雅降级，不再写盘）。
#[test]
fn halt_flush_drains_finishes_and_replies_dir() {
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

    // 直接经 handle_cmd 模拟 IO tokio 消费顺序（session_start → 步帧 → 收尾）。
    io.handle_cmd(RecorderCmd::Frame(dummy_session_start()));
    for i in 0..3 {
        io.handle_cmd(RecorderCmd::Frame(dummy_step(i)));
    }
    let (reply_tx, reply_rx) = flume::bounded(1);
    io.handle_cmd(RecorderCmd::HaltFlush { reply: reply_tx });
    let dir = reply_rx.recv_timeout(Duration::from_secs(1)).unwrap();
    assert!(!dir.is_empty(), "reply 回实际会话目录");
    let dir_path = std::path::Path::new(&dir);
    let manifest = std::fs::read_to_string(dir_path.join("manifest.json")).unwrap();
    assert!(
        manifest.contains("\"schema_version\""),
        "manifest: {manifest}"
    );
    assert!(
        manifest.contains("seg_000000.orec"),
        "残余步帧已落盘: {manifest}"
    );
    let before = std::fs::read_dir(dir_path).unwrap().count();

    // 收尾后残余步帧：优雅丢弃（不 panic、不再写盘）。
    assert!(!io.handle_frame(dummy_step(100)));
    let after = std::fs::read_dir(dir_path).unwrap().count();
    assert_eq!(before, after, "收尾后不再落盘");
    io.finalize();
}

/// 未启用录制：request_halt_flush 直接回空目录（不建 IO）。
#[test]
fn halt_flush_disabled_replies_empty_dir() {
    let q = RecorderEnqueue::disabled();
    let (reply_tx, reply_rx) = flume::bounded(1);
    assert!(q.request_halt_flush(reply_tx));
    let dir = reply_rx.recv_timeout(Duration::from_secs(1)).unwrap();
    assert!(dir.is_empty(), "未启用录制 → dir 空");
}

/// 水位满时 HaltFlush 仍可达：request_halt_flush 不查水位（控制消息绕过反压）。
#[test]
fn halt_flush_reachable_when_watermark_full() {
    let tmp = tempfile::tempdir().unwrap();
    let (enq, _io) = RecorderIo::pair(
        tmp.path().to_path_buf(),
        new_session_id("Test_1", 0),
        SessionMeta {
            drive: "client-step".into(),
            sim_dt_ms: 20,
            created_unix_ms: None,
        },
    );
    // 灌到高水位以上（accepts=false）。
    assert!(enq.try_enqueue(dummy_session_start()));
    while enq.accepts() {
        assert!(enq.try_enqueue(dummy_step(0)));
    }
    assert!(!enq.accepts(), "预置：水位已满");
    // 控制消息仍可入队（未丢弃、未反压）：队列在满水位之上再 +1。
    let (reply_tx, reply_rx) = flume::bounded(1);
    let len_before = enq.queued_len();
    assert!(
        enq.request_halt_flush(reply_tx),
        "水位满时 HaltFlush 仍可达"
    );
    assert_eq!(
        enq.queued_len(),
        len_before + 1,
        "控制消息入队（绕过水位检查）"
    );
    drop(reply_rx); // 无消费者：reply 失败由 `let _ =` 吞掉，不 panic
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
        separations: Vec::new(),
        terminal: orbitx_recorder::frame::TerminalInfo {
            crash_msg: None,
            landed_msg: None,
            launch_failed_msg: None,
        },
        bodies: Vec::new(),
    })
}
