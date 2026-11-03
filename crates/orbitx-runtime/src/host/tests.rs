use std::time::Duration;

use clap::Parser;

use super::spawn_host;
use crate::cli::{DriveModeArg, RuntimeArgs};

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
