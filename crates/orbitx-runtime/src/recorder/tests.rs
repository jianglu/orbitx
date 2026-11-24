use super::{
    civil_from_days, new_session_id, RecorderCmd, RecorderEnqueue, RecorderIo,
    frame::{halt_cutoff_needed, CutoffTracker},
};
use crate::input::{InputCmd, SessionCmd};
use crate::shutdown::ShutdownFlag;
use orbitx_math::StateVectors;
use orbitx_vessel::{assembly::Assembly, presets};
use orbitx_recorder::frame::{
    CutoffRecord, FlightInput, OrbitElements, RecorderFrame, ScoringFrame, SessionEvent,
    SessionStartFrame,
};
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
    io.handle_cmd(RecorderCmd::HaltFlush {
        reply: reply_tx,
        scoring: None,
    });
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

/// CutoffTracker 边沿语义（与 Godot CompletionJudge 同源）：
/// armed = 经历过点火上升沿；下降沿仅 armed 后记录；每次上升沿重置燃烧态；
/// `armed_at` = 首次点火时刻，此后再点火不改写（评分锚历元）。
#[test]
fn cutoff_tracker_edge_semantics() {
    let mut t = CutoffTracker::new();
    assert_eq!(t.armed_at(), None, "未点火无首燃时刻");
    // 未点火：无事件。
    assert!(!t.observe(false, 10), "初始静止无下降沿");
    // 上升沿：armed 置位，不记；首燃时刻记录。
    assert!(!t.observe(true, 20), "上升沿非下降沿");
    assert_eq!(t.armed_at(), Some(20), "首次点火时刻");
    // 持续推力：无事件。
    assert!(!t.observe(true, 40));
    // 首次关机（armed 后下降沿）：记录（调用方在下降沿补记快照）。
    assert!(t.observe(false, 60), "armed 后下降沿 = 关机事件");
    t.push(dummy_cutoff(60));
    assert!(!t.observe(false, 80), "持续关机不重复记");
    // 再次点火（第二次 armed 已置位）→ 再关机：第二条；首燃时刻不变。
    assert!(!t.observe(true, 100));
    assert_eq!(t.armed_at(), Some(20), "再点火不改写首燃时刻");
    assert!(t.observe(false, 120), "第二次关机再记一条");
    t.push(dummy_cutoff(120));
    assert_eq!(t.cutoffs().len(), 2, "两条关机事件");
    assert_eq!(t.into_cutoffs()[1].sim_t, 120);

    // 未 armed 的边沿不记（未点火直接判下降沿不存在；前缘即推力首次消失
    // 之前必须有上升沿）。
    let mut t2 = CutoffTracker::new();
    t2.observe(true, 5);
    assert!(t2.observe(false, 10), "单次燃尽即 armed");
    t2.reset();
    assert!(t2.cutoffs().is_empty(), "reset 清空");
    assert_eq!(t2.armed_at(), None, "reset 清首燃时刻");
    assert!(!t2.observe(false, 15), "reset 后未 armed 下降沿不记");
}

/// 停机补记判据（`halt_cutoff_needed`，2026-11-23 恒补记口径）：已 armed
/// 且无同刻关机 → 补——参数是 **armed**（点过火与否），燃烧中停机与滑行
/// 停机对判据不可区分（都 armed = true），二者都补（评分点 = 定格时刻）；
/// 同刻已记（关机与停机同 tick）/ 未点火（armed = false）→ 不补。
#[test]
fn halt_cutoff_needed_pure_logic() {
    let rec = dummy_cutoff(300);
    // 燃烧中停机（armed）：补。
    assert!(halt_cutoff_needed(&[rec.clone()], true, 612_480));
    // 空表但已 armed = 首燃尚未关机即停机（首次 MECO 前打断）→ 补。
    assert!(halt_cutoff_needed(&[], true, 612_480));
    // 关机与停机同 tick：已记，不补。
    assert!(!halt_cutoff_needed(&[rec.clone()], true, 300));
    // 滑行中停机（armed 恒 true——点过火即 armed）：补（定格态 ≠ 末级
    // 关机态，评分点 = 停机定格时刻；与燃烧中停机对判据同支）。
    assert!(halt_cutoff_needed(&[rec], true, 612_480));
    // 未点火（未 armed）：不补（cutoffs 空 = A 项 0 分口径不变）。
    assert!(!halt_cutoff_needed(&[], false, 612_480));
}

/// HaltFlush 携带评分快照帧：drain 残余步帧后、finish 前落盘——会话读回
/// 含 step 帧 + scoring 帧（顺序：步帧在前）。
#[test]
fn halt_flush_writes_scoring_snapshot_frame() {
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
    io.handle_cmd(RecorderCmd::Frame(dummy_session_start()));
    io.handle_cmd(RecorderCmd::Frame(dummy_step(0)));
    let (reply_tx, reply_rx) = flume::bounded(1);
    io.handle_cmd(RecorderCmd::HaltFlush {
        reply: reply_tx,
        scoring: Some(RecorderFrame::Scoring(ScoringFrame {
            sim_t: 540_250,
            loaded_fuel_kg: 400_000.0,
            first_burn_sim_t_ms: Some(1_200),
            cutoffs: vec![dummy_cutoff(172_300)],
            halt_state: OrbitElements {
                pe_alt_m: 199_900.0,
                ap_alt_m: 210_500.0,
                inc_deg: 28.5,
                raan_deg: 0.0,
                argp_deg: 0.0,
                arg_lat_deg: 90.0,
            },
        })),
    });
    let dir = reply_rx.recv_timeout(Duration::from_secs(1)).unwrap();
    assert!(!dir.is_empty());
    let r = orbitx_recorder::reader::SessionReader::open(std::path::Path::new(&dir))
        .expect("会话可读");
    assert_eq!(r.manifest().schema_version, 2, "schema 2");
    let frames = r.frames().unwrap();
    let kinds: Vec<&str> = frames
        .iter()
        .map(|f| match f {
            RecorderFrame::SessionStart(_) => "session_start",
            RecorderFrame::Step(_) => "step",
            RecorderFrame::Scoring(_) => "scoring",
            RecorderFrame::Unknown => "unknown",
        })
        .collect();
    assert_eq!(
        kinds,
        vec!["session_start", "step", "scoring"],
        "快照帧在步帧之后: {kinds:?}"
    );
    let orbitx_recorder::frame::RecorderFrame::Scoring(sf) = &frames[2] else {
        panic!("expected scoring frame")
    };
    assert_eq!(sf.sim_t, 540_250);
    assert_eq!(sf.first_burn_sim_t_ms, Some(1_200), "首燃时刻读回");
    assert_eq!(sf.cutoffs.len(), 1);
    assert_eq!(sf.cutoffs[0].sim_t, 172_300);
    io.finalize();
}

/// scoring_thrusting 主推口径：RCS 等非主推槽位点火不构成燃烧态（MECO 后
/// 姿态脉冲不制造虚假关机沿）；主推点火 = 燃烧态。
#[test]
fn scoring_thrusting_ignores_non_main_thrusters() {
    use orbitx_math::Vec3;
    use orbitx_vessel::thruster::Thruster;

    let stages = presets::falcon9();
    let mut asm = Assembly::new(&stages, StateVectors::default());
    {
        let v = &mut asm.vessels[0];
        // 全关：非燃烧。
        for t in v.thrusters.iter_mut() {
            t.level = 0.0;
        }
    }
    assert!(!super::frame::scoring_thrusting(&asm), "全关非燃烧");

    let n_main = asm.vessels[0].n_main_thrusters;
    {
        let v = &mut asm.vessels[0];
        // 追加一台「RCS」槽位（尾插，n_main 之外）并点火；主推保持全关。
        v.thrusters.push(Thruster::new(
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(0.0, 1.0, 0.0),
            1.0e3,
            220.0,
        ));
        let last = v.thrusters.len() - 1;
        v.thrusters[last].level = 1.0;
        assert!(last >= n_main, "追加槽位在主推区间之外");
    }
    assert!(
        !super::frame::scoring_thrusting(&asm),
        "仅 RCS 点火不构成燃烧态"
    );

    {
        let v = &mut asm.vessels[0];
        for t in v.thrusters[..n_main].iter_mut() {
            t.level = 1.0;
        }
    }
    assert!(super::frame::scoring_thrusting(&asm), "主推点火 = 燃烧态");
}

/// 逃逸轨 pe 保护值（与切片 build_slice 同式）：pe_alt_m = 0（海拔口径，
/// 必坠语义），ap = +∞；椭圆轨 pe/ap 照常实算。
#[test]
fn orbit_elements_escape_pe_alt_is_zero() {
    use orbitx_math::Vec3;
    let r = 1.0;
    // 双曲：ε = v²/2 − μ/r = 2 − 1 > 0。
    let esc = super::frame::orbit_elements_of(
        Vec3::new(r, 0.0, 0.0),
        Vec3::new(0.0, 2.0, 0.0),
        1.0,
        0.5,
    );
    assert!((esc.pe_alt_m - 0.0).abs() < 1e-12, "逃逸 pe 保护值 = 0，got {}", esc.pe_alt_m);
    assert!(esc.ap_alt_m.is_infinite(), "逃逸 ap = ∞");
    // 椭圆：ε = 0.32 − 1 < 0。
    let ell = super::frame::orbit_elements_of(
        Vec3::new(r, 0.0, 0.0),
        Vec3::new(0.0, 0.8, 0.0),
        1.0,
        0.5,
    );
    assert!(ell.pe_alt_m.is_finite(), "椭圆 pe 实算");
    assert!(ell.ap_alt_m.is_finite(), "椭圆 ap 实算");
}

fn dummy_cutoff(sim_t: u64) -> CutoffRecord {
    CutoffRecord {
        sim_t,
        lat_deg: 0.0,
        lon_deg: 0.0,
        alt_m: 200_000.0,
        speed_ms: 7790.0,
        gamma_deg: 0.0,
        pe_alt_m: 199_900.0,
        ap_alt_m: 210_500.0,
        inc_deg: 28.5,
        raan_deg: 0.0,
        argp_deg: 0.0,
        arg_lat_deg: 90.0,
        at_halt: false,
    }
}

/// 未启用录制：request_halt_flush 直接回空目录（不建 IO）。
#[test]
fn halt_flush_disabled_replies_empty_dir() {
    let q = RecorderEnqueue::disabled();
    let (reply_tx, reply_rx) = flume::bounded(1);
    assert!(q.request_halt_flush(reply_tx, None));
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
        enq.request_halt_flush(reply_tx, None),
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
