use std::time::Duration;

use clap::Parser;

use super::spawn_host;
use crate::cli::RuntimeArgs;

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
