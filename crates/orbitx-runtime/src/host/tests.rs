use std::time::Duration;

use clap::Parser;

use super::spawn_host;
use crate::cli::{DriveModeArg, RuntimeArgs};

#[test]
fn shutdown_stops_runtime_and_io() {
    let dir = tempfile::tempdir().unwrap();
    let args = RuntimeArgs {
        rocket: "falcon9".into(),
        scenario: None,
        control: None,
        workflow: None,
        drive: DriveModeArg::SelfPaced,
        sim_dt: 20,
        log_dir: dir.path().join("logs"),
        recorder_dir: dir.path().join("rec"),
        zenoh_endpoint: "local".into(),
        ephemeris_data: None,
    };
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
