use std::time::Duration;

use clap::Parser;

use super::spawn_host;
use crate::channel::RuntimeInbound;
use crate::cli::{DriveModeArg, RuntimeArgs};
use crate::input::{InputCmd, SessionCmd};

#[test]
fn shutdown_stops_runtime_and_io() {
    let args = crate::cli::test_runtime_args("falcon9");
    let host = spawn_host(args);
    std::thread::sleep(Duration::from_millis(50));
    host.request_shutdown();
    host.join();
}

#[test]
fn parse_help_smoke() {
    let err = RuntimeArgs::try_parse_from(["orbitx-runtime", "--help"]).unwrap_err();
    let _ = err;
}

/// FlightRecorder 端到端：enabled → self-paced 步进 → shutdown 后断言
/// 会话目录 / manifest / orec 段文件落盘。
#[test]
fn recorder_enabled_writes_session_artifacts() {
    let tmp = tempfile::tempdir().unwrap();
    let mut args = crate::cli::test_runtime_args("falcon9");
    args.recorder_enabled = true;
    args.recorder_dir = Some(tmp.path().to_path_buf());
    args.drive = DriveModeArg::SelfPaced;
    let host = spawn_host(args);
    std::thread::sleep(Duration::from_millis(400));
    host.request_shutdown();
    host.join();

    let dirs: Vec<_> = std::fs::read_dir(tmp.path())
        .unwrap()
        .map(|e| e.unwrap().path())
        .collect();
    assert_eq!(dirs.len(), 1, "one session dir: {dirs:?}");
    let manifest = std::fs::read_to_string(dirs[0].join("manifest.json")).unwrap();
    assert!(
        manifest.contains("\"schema_version\""),
        "manifest: {manifest}"
    );
    assert!(manifest.contains("seg_000000.orec"), "manifest: {manifest}");
    let seg = std::fs::read(dirs[0].join("seg_000000.orec")).unwrap();
    assert_eq!(&seg[..8], b"OXREC001");
}

/// C3 halt 端到端（参数化缺省全真）：workflow 图（drive + halt）→
/// ignite + launch → halt 锁存 → `Outbound::Halted` 经 ack 通道发出
/// （黑匣子 flush 收尾：目录落盘 + manifest 终版）；停机终态下 sim_t
/// 不再前进（步进停止；Shutdown/Reset 仍响应）。
#[test]
fn halt_workflow_emits_halted_and_stops_stepping() {
    let tmp = tempfile::tempdir().unwrap();
    let rec_root = tmp.path().join("records");
    let wf_path = tmp.path().join("wf_halt.toml");
    std::fs::write(
        &wf_path,
        r#"
schema = 1
kind = "target"
name = "halt_demo"

[[nodes]]
id = "ign0"
type = "ignite"

[[nodes]]
id = "m"
type = "main"
stage = "F9-S1"

[[nodes]]
id = "d"
type = "drive"
mode = "vertical_hold"
throttle = 1.0

[[nodes]]
id = "h"
type = "halt"

[[edges]]
from = "ign0"
to = "m"

[[edges]]
from = "m"
from_port = "flight"
to = "d"

[[edges]]
from = "m"
from_port = "flight"
to = "h"
"#,
    )
    .unwrap();

    let mut args = crate::cli::test_runtime_args("falcon9");
    args.control = None;
    args.workflow = Some(wf_path);
    args.drive = DriveModeArg::SelfPaced;
    args.recorder_enabled = true;
    args.recorder_dir = Some(rec_root.clone());
    let host = spawn_host(args);

    // 点火 + 开闸：main Active → drive 写油门，同 tick halt 锁存 → 停步进 + flush。
    host.cmd_tx
        .send(RuntimeInbound::Input(InputCmd::WorkflowIgnite))
        .unwrap();
    host.cmd_tx
        .send(RuntimeInbound::Session(SessionCmd::Launch))
        .unwrap();

    // 等 Halted（跳过空载期可能缓存的 ack？空载期无 Configure 不会发 ack，
    // 首个 Outbound 即 Halted；有界 15s 兜底 CI 慢机）。
    let halted = loop {
        match host.ack_rx.recv_timeout(Duration::from_secs(15)) {
            Ok(outbound) => match outbound.payload {
                Some(orbitx_protocol::outbound::Payload::Halted(h)) => break h,
                Some(orbitx_protocol::outbound::Payload::Ack(_)) => {}
                None => {}
            },
            Err(e) => panic!("Halted not received in time: {e}"),
        }
    };
    assert!(!halted.session_id.is_empty(), "session_id = session_base");
    assert_eq!(
        halted.recorder_schema_version,
        orbitx_recorder::SCHEMA_VERSION
    );
    // argv Eager 无 session_context → 无子树：目录直接位于 root 下。
    let dir = std::path::Path::new(&halted.recorder_session_dir);
    assert!(
        dir.starts_with(&rec_root) && dir.join("manifest.json").is_file(),
        "flush 后会话目录 + manifest 落盘: {}",
        halted.recorder_session_dir
    );

    // 停机终态：黑匣子步帧数不再增长（磁盘权威观测。切片通道与 Comms
    // 竞争消费、keep-latest，不作断言依据）。flush 是终态收尾——此后无新步帧。
    let count_step_frames = || -> u64 {
        orbitx_recorder::SessionReader::open(dir)
            .unwrap()
            .frames()
            .unwrap()
            .iter()
            .filter(|f| matches!(f, orbitx_recorder::RecorderFrame::Step(_)))
            .count() as u64
    };
    let f1 = count_step_frames();
    assert!(f1 > 0, "halt 前应有步帧");
    std::thread::sleep(Duration::from_millis(500));
    assert_eq!(
        count_step_frames(),
        f1,
        "halted 后黑匣子无新步帧（步进停止）"
    );

    host.request_shutdown();
    host.join();
}

/// C3 补全轮三全关静默端到端：halt{三开关全 false = 零指令，wf 另携 `title`
/// 呈现键} 激活 → dispatch 两个 if 均不命中 → **无 Halted 上行**（save_recorder
/// =false）、步进继续（stop_stepping=false）、控制不清零（shutdown_controls=
/// false，油门保持——controller 层断言见 graph/tests.rs）。任务二发射「无误
/// 收口」的机制前提（COMPETITION §8 矩阵）；执行流走出边锚点（前缀边界切换）
/// 属 C5 修订，不在本断言范围。
#[test]
fn halt_all_off_is_silent() {
    let tmp = tempfile::tempdir().unwrap();
    let rec_root = tmp.path().join("records");
    let wf_path = tmp.path().join("wf_halt_zero.toml");
    std::fs::write(
        &wf_path,
        r#"
schema = 1
kind = "target"
name = "halt_zero_demo"

[[nodes]]
id = "ign0"
type = "ignite"

[[nodes]]
id = "m"
type = "main"
stage = "F9-S1"

[[nodes]]
id = "d"
type = "drive"
mode = "vertical_hold"
throttle = 1.0

[[nodes]]
id = "h"
type = "halt"
shutdown_controls = false
stop_stepping = false
save_recorder = false
title = "发射入轨·目标完成"

[[edges]]
from = "ign0"
to = "m"

[[edges]]
from = "m"
from_port = "flight"
to = "d"

[[edges]]
from = "m"
from_port = "flight"
to = "h"
"#,
    )
    .unwrap();

    let mut args = crate::cli::test_runtime_args("falcon9");
    args.control = None;
    args.workflow = Some(wf_path);
    args.drive = DriveModeArg::SelfPaced;
    args.recorder_enabled = true;
    args.recorder_dir = Some(rec_root.clone());
    let host = spawn_host(args);

    host.cmd_tx
        .send(RuntimeInbound::Input(InputCmd::WorkflowIgnite))
        .unwrap();
    host.cmd_tx
        .send(RuntimeInbound::Session(SessionCmd::Launch))
        .unwrap();

    // 无 Halted 窗口（save_recorder=false 不发停机事件）：过滤 Ack，任何
    // Halted 到达即失败；2s 超时 = 静默成立。前段 2s sleep = SelfPaced 步进
    // 实跑 4s（误停形态下 halt 锁存拍即冻结，步进继续则持续累积帧）。
    std::thread::sleep(Duration::from_secs(2));
    loop {
        match host.ack_rx.recv_timeout(Duration::from_secs(2)) {
            Ok(outbound) => match outbound.payload {
                Some(orbitx_protocol::outbound::Payload::Halted(h)) => {
                    panic!("三全关不应发 Halted: session={}", h.session_id);
                }
                _ => {}
            },
            Err(_) => break,
        }
    }

    // 步进继续（磁盘权威，shutdown 触发 finish flush 后读）：若 stop_stepping
    // 误生效，帧数停留在锁存拍（≈1-2 帧）；步进继续 → 4s 全速步进帧数远超阈值。
    // （active 段在会话中不落盘——writer 内存攒帧、rotate/finish 才写，故无法
    // 中途读磁盘，须 finish 后验证。）
    host.request_shutdown();
    host.join();
    let mut dirs: Vec<_> = std::fs::read_dir(&rec_root)
        .expect("rec_root 落盘")
        .filter_map(|e| e.ok().map(|e| e.path()))
        .collect();
    dirs.sort();
    assert_eq!(dirs.len(), 1, "one session dir: {dirs:?}");
    let frames = orbitx_recorder::SessionReader::open(&dirs[0])
        .unwrap()
        .frames()
        .unwrap()
        .iter()
        .filter(|f| matches!(f, orbitx_recorder::RecorderFrame::Step(_)))
        .count();
    assert!(
        frames > 30,
        "步进应继续：Step 帧 {frames} 应远超误停形态（≈1-2 帧）"
    );
}

/// C3 client-step + halt：`Step(100)` 残量在 halt 锁存拍被丢弃
/// （`pending_steps = 0` + `do_one_step` halted 守卫），黑匣子只落
/// halt 前的少量步帧且不再增长。
#[test]
fn halt_client_step_discards_pending_steps() {
    let tmp = tempfile::tempdir().unwrap();
    let rec_root = tmp.path().join("records");
    let wf_path = tmp.path().join("wf_halt.toml");
    std::fs::write(
        &wf_path,
        r#"
schema = 1
kind = "target"
name = "halt_step"

[[nodes]]
id = "ign0"
type = "ignite"

[[nodes]]
id = "m"
type = "main"
stage = "F9-S1"

[[nodes]]
id = "h"
type = "halt"

[[edges]]
from = "ign0"
to = "m"

[[edges]]
from = "m"
from_port = "flight"
to = "h"
"#,
    )
    .unwrap();

    let mut args = crate::cli::test_runtime_args("falcon9");
    args.control = None;
    args.workflow = Some(wf_path);
    args.drive = DriveModeArg::ClientStep;
    args.recorder_enabled = true;
    args.recorder_dir = Some(rec_root.clone());
    let host = spawn_host(args);

    host.cmd_tx
        .send(RuntimeInbound::Input(InputCmd::WorkflowIgnite))
        .unwrap();
    host.cmd_tx
        .send(RuntimeInbound::Session(SessionCmd::Launch))
        .unwrap();
    host.cmd_tx
        .send(RuntimeInbound::Session(SessionCmd::Step { n: 100 }))
        .unwrap();

    // 等 Halted（步进至 halt 锁存拍即停）。
    loop {
        match host.ack_rx.recv_timeout(Duration::from_secs(15)) {
            Ok(outbound) => {
                if matches!(
                    outbound.payload,
                    Some(orbitx_protocol::outbound::Payload::Halted(_))
                ) {
                    break;
                }
            }
            Err(e) => panic!("Halted not received in time: {e}"),
        }
    }

    // flush reply 前队列已 drain：Halted 时磁盘帧数定数；此后无新步帧。
    std::thread::sleep(Duration::from_millis(200));
    let dirs: Vec<_> = std::fs::read_dir(&rec_root)
        .unwrap()
        .map(|e| e.unwrap().path())
        .collect();
    assert_eq!(dirs.len(), 1);
    let count_step_frames = || -> u64 {
        orbitx_recorder::SessionReader::open(&dirs[0])
            .unwrap()
            .frames()
            .unwrap()
            .iter()
            .filter(|f| matches!(f, orbitx_recorder::RecorderFrame::Step(_)))
            .count() as u64
    };
    let f1 = count_step_frames();
    assert!(f1 < 100, "残量被丢弃：步帧 {f1} ≪ 100");
    std::thread::sleep(Duration::from_millis(400));
    assert_eq!(count_step_frames(), f1, "halted 后无新步帧");

    host.request_shutdown();
    host.join();
}

/// C3 halt → Reset → 新会话：Reset 解除终态 + 复位锁存 + 黑匣子开新目录；
/// 再 Launch 后步进恢复（main Idle 不再触发 halt）。
#[test]
fn halt_then_reset_reopens_session_and_resumes_stepping() {
    let tmp = tempfile::tempdir().unwrap();
    let rec_root = tmp.path().join("records");
    let wf_path = tmp.path().join("wf_halt.toml");
    std::fs::write(
        &wf_path,
        r#"
schema = 1
kind = "target"
name = "halt_reset"

[[nodes]]
id = "ign0"
type = "ignite"

[[nodes]]
id = "m"
type = "main"
stage = "F9-S1"

[[nodes]]
id = "h"
type = "halt"

[[edges]]
from = "ign0"
to = "m"

[[edges]]
from = "m"
from_port = "flight"
to = "h"
"#,
    )
    .unwrap();

    let mut args = crate::cli::test_runtime_args("falcon9");
    args.control = None;
    args.workflow = Some(wf_path);
    args.drive = DriveModeArg::SelfPaced;
    args.recorder_enabled = true;
    args.recorder_dir = Some(rec_root.clone());
    let host = spawn_host(args);

    host.cmd_tx
        .send(RuntimeInbound::Input(InputCmd::WorkflowIgnite))
        .unwrap();
    host.cmd_tx
        .send(RuntimeInbound::Session(SessionCmd::Launch))
        .unwrap();
    // 等 Halted（首会话收尾）。
    loop {
        match host.ack_rx.recv_timeout(Duration::from_secs(15)) {
            Ok(outbound) => {
                if matches!(
                    outbound.payload,
                    Some(orbitx_protocol::outbound::Payload::Halted(_))
                ) {
                    break;
                }
            }
            Err(e) => panic!("Halted not received in time: {e}"),
        }
    }

    // Reset：终态解除 + 锁存复位 + 新黑匣子会话目录。
    // 先 drain 通道积压（心跳占满 8 槽会让 Reset 后步进切片落 latest 槽）。
    while host
        .slice_rx
        .recv_timeout(Duration::from_millis(50))
        .is_ok()
    {}
    host.cmd_tx
        .send(RuntimeInbound::Session(SessionCmd::Reset))
        .unwrap();
    std::thread::sleep(Duration::from_millis(300));
    let dirs: Vec<_> = std::fs::read_dir(&rec_root)
        .unwrap()
        .map(|e| e.unwrap().path())
        .collect();
    assert_eq!(dirs.len(), 2, "Reset 后开新会话目录: {dirs:?}");

    // Launch（gate 已重关）：步进恢复——sim_t 前进（持续消费保通道不积压）。
    host.cmd_tx
        .send(RuntimeInbound::Session(SessionCmd::Launch))
        .unwrap();
    let mut t0 = None;
    let deadline = std::time::Instant::now() + Duration::from_secs(10);
    loop {
        assert!(
            std::time::Instant::now() < deadline,
            "步进未恢复（sim_t 未前进）"
        );
        match host.slice_rx.recv_timeout(Duration::from_millis(200)) {
            Ok(slice) => match t0 {
                None => t0 = Some(slice.sim_t),
                Some(prev) if slice.sim_t > prev => break,
                _ => {}
            },
            Err(_) => continue,
        }
    }

    host.request_shutdown();
    host.join();
}
