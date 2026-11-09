use std::fs;
use std::path::PathBuf;

use tempfile::TempDir;

use orbitx_recorder::frame::{RecorderFrame, SessionStartFrame, SCHEMA_VERSION};
use orbitx_recorder::{SessionMeta, SessionWriter};

use crate::service::handle;

fn minimal_session_start() -> RecorderFrame {
    RecorderFrame::SessionStart(SessionStartFrame {
        schema_version: SCHEMA_VERSION,
        sim_t: 0,
        sim_dt_ms: 20,
        rocket: orbitx_recorder::frame::RocketInfo {
            name: "T".into(),
            class: "T".into(),
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

fn write_fixture(root: &PathBuf, name: &str, steps: u64) {
    fs::create_dir_all(root).unwrap();
    let mut w = SessionWriter::create(
        root,
        name,
        SessionMeta {
            drive: "self-paced".into(),
            sim_dt_ms: 20,
            created_unix_ms: Some(1_770_000_000_000),
        },
        Default::default(),
    )
    .unwrap();
    w.push_frame(&minimal_session_start()).unwrap();
    for i in 0..steps {
        w.push_frame(&RecorderFrame::Step(orbitx_recorder::frame::StepFrame {
            sim_t: 20 * (i + 1),
            step_index: i + 1,
            inputs: orbitx_recorder::frame::InputsSnapshot {
                flight: vec![],
                session: vec![],
                warp: 1.0,
                paused: false,
            },
            environment: orbitx_recorder::frame::EnvironmentSnapshot {
                mjd: 0.0,
                star: None,
                bodies: Vec::new(),
            },
            separated: vec![],
            separations: vec![],
            terminal: orbitx_recorder::frame::TerminalInfo {
                crash_msg: None,
                landed_msg: None,
                launch_failed_msg: None,
            },
            bodies: vec![],
        }))
        .unwrap();
    }
    w.finish().unwrap();
}

/// sessions：目录名白名单 + manifest 摘要。
#[test]
fn sessions_lists_manifest_summary() {
    let tmp = TempDir::new().unwrap();
    let root = tmp.path().join("orbitx_records");
    write_fixture(&root, "20260101_000000_aaaa", 3);
    write_fixture(&root, "20260102_000000_bbbb", 1);

    let resp = handle(r#"{"op":"sessions"}"#, &root).unwrap();
    assert_eq!(resp["ok"], true);
    let sessions = resp["sessions"].as_array().unwrap();
    assert_eq!(sessions.len(), 2);
    assert_eq!(sessions[0]["dir"], "20260101_000000_aaaa");
    assert_eq!(sessions[0]["frame_count"], 4);
    assert_eq!(sessions[1]["frame_count"], 2);
    assert_eq!(sessions[1]["last_sim_t"], 20);
}

/// frames：范围过滤 + count；未知会话与坏请求报错。
#[test]
fn frames_range_and_errors() {
    let tmp = TempDir::new().unwrap();
    let root = tmp.path().join("orbitx_records");
    write_fixture(&root, "20260101_000000_aaaa", 5);

    let resp = handle(
        r#"{"op":"frames","session":"20260101_000000_aaaa","from":40,"to":80}"#,
        &root,
    )
    .unwrap();
    assert_eq!(resp["ok"], true);
    assert_eq!(resp["count"], 3);
    assert_eq!(resp["frames"][0]["sim_t"], 40);
    assert_eq!(resp["frames"][1]["sim_t"], 60);
    assert_eq!(resp["frames"][2]["sim_t"], 80);

    let err = handle(r#"{"op":"frames","session":"nope"}"#, &root).unwrap_err();
    assert!(err.contains("not found"));
    let err = handle(r#"{"op":"bogus"}"#, &root).unwrap_err();
    assert!(err.contains("unknown op"));
    let err = handle("not json", &root).unwrap_err();
    assert!(err.contains("bad request"));
}

/// 路径穿越防护：session 字段含 `../` 时只能匹配真实目录名（匹配不到即报错）。
#[test]
fn frames_rejects_path_traversal() {
    let tmp = TempDir::new().unwrap();
    let root = tmp.path().join("orbitx_records");
    write_fixture(&root, "20260101_000000_aaaa", 1);
    let err = handle(r#"{"op":"frames","session":"../secret"}"#, &root).unwrap_err();
    assert!(err.contains("not found"));
}
