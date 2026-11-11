use std::io::Write;
use std::path::PathBuf;

use clap::Parser;
use orbitx_controller::workflow::WorkFlowKind;

use super::test_runtime_args;
use super::{validate_zenoh_endpoint, ControlKindArg, RuntimeArgs, SessionControl};

#[test]
fn rejects_missing_flags() {
    assert!(RuntimeArgs::try_parse_from(["orbitx-runtime"]).is_err());
}

#[test]
fn explicit_args_validate() {
    let args = test_runtime_args("falcon9");
    assert_eq!(args.scenario.as_deref(), Some("earth"));
    assert_eq!(args.sim_dt, 20);
    args.validate().unwrap();
    match args.resolve_control().unwrap() {
        SessionControl::Control(ControlKindArg::Target) => {}
        other => panic!("expected Control(Target), got {other:?}"),
    }
}

#[test]
fn control_without_value_is_rejected() {
    let err = RuntimeArgs::try_parse_from(["orbitx-runtime", "--control"]).unwrap_err();
    let _ = err;
}

#[test]
fn control_base() {
    let mut args = test_runtime_args("falcon9");
    args.control = Some(ControlKindArg::Base);
    match args.resolve_control().unwrap() {
        SessionControl::Control(ControlKindArg::Base) => {}
        other => panic!("expected Control(Base), got {other:?}"),
    }
}

#[test]
fn workflow_loads_target_toml() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("ascent.toml");
    let mut f = std::fs::File::create(&path).unwrap();
    write!(
        f,
        r#"
kind = "target"
name = "ascent"

[[phases]]
mode = "vertical_hold"
throttle = 1.0
"#
    )
    .unwrap();

    let mut args = test_runtime_args("falcon9");
    args.control = None;
    args.workflow = Some(path);
    let ctrl = args.resolve_control().unwrap();
    match ctrl {
        SessionControl::WorkFlow { desc, .. } => {
            assert_eq!(desc.kind, WorkFlowKind::Target);
            assert_eq!(desc.name, "ascent");
        }
        other => panic!("expected WorkFlow, got {other:?}"),
    }
    args.validate().unwrap();
}

#[test]
fn workflow_missing_file_errors() {
    let mut args = test_runtime_args("falcon9");
    args.control = None;
    args.workflow = Some("no-such-wf.toml".into());
    let err = args.resolve_control().unwrap_err();
    assert!(err.contains("不存在") || err.contains("workflow"));
}

#[test]
fn workflow_loads_graph_ir() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("wf_t_demo.toml");
    std::fs::write(
        &path,
        r#"
schema = 1
kind = "target"
name = "demo"

[[nodes]]
id = "ign0"
type = "ignite"

[[nodes]]
id = "m"
type = "main"
stage = "Core"

[[edges]]
from = "ign0"
to = "m"
"#,
    )
    .unwrap();
    let mut args = test_runtime_args("falcon9");
    args.control = None;
    args.workflow = Some(path);
    match args.resolve_control().unwrap() {
        SessionControl::Graph { ir, .. } => {
            assert_eq!(ir.kind, WorkFlowKind::Target);
            assert_eq!(ir.name, "demo");
        }
        other => panic!("expected Graph, got {other:?}"),
    }
}

#[test]
fn workflow_invalid_toml_errors() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("bad.toml");
    std::fs::write(&path, "kind = \"nope\"\nname = \"x\"\n").unwrap();
    let mut args = test_runtime_args("falcon9");
    args.control = None;
    args.workflow = Some(path);
    assert!(args.resolve_control().is_err());
}

#[test]
fn control_and_workflow_conflict() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("ascent.toml");
    std::fs::write(
        &path,
        r#"
kind = "target"
name = "ascent"
[[phases]]
mode = "vertical_hold"
throttle = 1.0
"#,
    )
    .unwrap();
    let mut args = test_runtime_args("falcon9");
    args.workflow = Some(path);
    let err = args.resolve_control().unwrap_err();
    assert!(err.contains("互斥"));
    assert!(args.validate().is_err());
}

#[test]
fn rejects_non_positive_sim_dt() {
    let mut args = test_runtime_args("falcon9");
    args.sim_dt = 0;
    assert!(args.validate().is_err());
}

#[test]
fn neither_control_nor_workflow_fails() {
    let mut args = test_runtime_args("falcon9");
    args.control = None;
    let err = args.resolve_control().unwrap_err();
    assert!(err.contains("--control"));
}

#[test]
fn accepts_local_and_loopback() {
    validate_zenoh_endpoint("local").unwrap();
    validate_zenoh_endpoint("tcp/127.0.0.1:7447").unwrap();
    validate_zenoh_endpoint("tcp/localhost:9").unwrap();
    validate_zenoh_endpoint("unixpipe:///tmp/orbitx.sock").unwrap();
}

#[test]
fn rejects_cross_device_tcp() {
    let err = validate_zenoh_endpoint("tcp/192.168.1.10:7447").unwrap_err();
    assert!(err.contains("cross-device") || err.contains("not supported"));
}

#[test]
fn loads_builtin_rocket_on_validate() {
    let args = test_runtime_args("saturnv");
    let session = args.load_session().unwrap();
    assert_eq!(session.rocket.class, "SaturnV");
    assert!(matches!(
        session.control,
        SessionControl::Control(ControlKindArg::Target)
    ));
}

#[test]
fn rejects_unknown_rocket() {
    let args = test_runtime_args("nope");
    assert!(args.validate().is_err());
}

const SCN_TWO_BODIES: &str = r#"
name = "TestSys"
mjd = 51544.5
primary = "Earth"

[[bodies]]
name = "Earth"
mass = 5.973698968e24
size = 6371010.0

[[bodies]]
name = "Moon"
mass = 7.34767309e22
size = 1737100.0
dynamics = false
fixed_pos = [3.844e8, 0.0, 0.0]
"#;

fn write_scenario(dir: &std::path::Path) -> std::path::PathBuf {
    let p = dir.join("scenario_test.toml");
    std::fs::write(&p, SCN_TWO_BODIES).unwrap();
    p
}

fn write_lp_body(dir: &std::path::Path, body: &str) -> std::path::PathBuf {
    let p = dir.join("lp_b.toml");
    std::fs::write(
        &p,
        format!(
            "schema=2\nname='B'\nbody='{body}'\nlatitude_deg=0.0\nlongitude_deg=0.0\nbase_elevation_m=0.0\n[[pads]]\nname='P'\n"
        ),
    )
    .unwrap();
    p
}

fn write_fake_package(dir: &std::path::Path, r_elev: f64) -> std::path::PathBuf {
    let root = dir.join("pkg");
    std::fs::create_dir_all(&root).unwrap();
    std::fs::write(
        root.join("manifest.toml"),
        format!(
            "format_version = 2\ndataset_id = \"fake\"\nR_elev_m = {r_elev}\nocean_surface_m = 0.0\n"
        ),
    )
    .unwrap();
    root
}

#[test]
fn missing_launchpad_errors() {
    // argv 层：会话级参数部分给出 → 防呆报「不完整」（防静默走错模式）。
    let mut args = test_runtime_args("falcon9");
    args.launchpad = None;
    let err = args.load_session().unwrap_err();
    assert!(err.contains("会话级参数不完整"), "{err}");

    // spec 层：其余齐全、缺 launchpad → validate_required 报「缺少 launchpad」。
    let spec = super::SessionSpec {
        rocket: Some("falcon9".into()),
        scenario: Some("earth".into()),
        control: Some("target".into()),
        ..Default::default()
    };
    let err = super::load_session(&spec, &args.ephemeris_data).unwrap_err();
    assert!(err.contains("缺少 launchpad"), "{err}");
}

#[test]
fn argv_mode_classifies_eager_deferred_partial() {
    // 全给 → Eager。
    let args = test_runtime_args("falcon9");
    assert!(matches!(args.argv_mode().unwrap(), super::ArgvMode::Eager(_)));
    // 全缺 → Deferred（空载启动）。
    let mut empty = test_runtime_args("falcon9");
    empty.rocket = None;
    empty.scenario = None;
    empty.launchpad = None;
    empty.control = None;
    assert!(matches!(
        empty.argv_mode().unwrap(),
        super::ArgvMode::Deferred
    ));
    // 部分给出 → 配置错误。
    let mut partial = test_runtime_args("falcon9");
    partial.launchpad = None;
    assert!(partial.argv_mode().unwrap_err().contains("会话级参数不完整"));
}

#[test]
fn session_spec_from_proto_maps_has_flags_and_empties() {
    use orbitx_protocol::Configure;
    // 空 proto → 全 None（空字符串 = 未指定）。
    let spec = super::SessionSpec::from_proto(Configure::default());
    assert!(spec.rocket.is_none());
    assert!(spec.scenario.is_none());
    assert!(spec.launchpad.is_none());
    assert!(spec.control.is_none());
    assert!(spec.validate_required().is_err());
    // has_* = false 时即便数值非零也不生效。
    let spec = super::SessionSpec::from_proto(Configure {
        rocket: "falcon9".into(),
        scenario_alias_or_path: "earth".into(),
        launchpad_path: "/abs/lp.toml".into(),
        control: "target".into(),
        mjd: 51544.5,
        has_mjd: false,
        heading_deg: 90.0,
        has_heading_deg: false,
        surface_working_max_level: 3,
        has_surface_working_max_level: false,
        ..Default::default()
    });
    assert_eq!(spec.mjd, None);
    assert_eq!(spec.heading_deg, None);
    assert_eq!(spec.surface_working_max_level, None);
    assert_eq!(spec.launchpad, Some(std::path::PathBuf::from("/abs/lp.toml")));
    assert!(spec.validate_required().is_ok());
}

#[test]
fn load_session_accepts_inline_toml_over_paths() {
    // 内嵌 scenario/lp TOML 优先（PCK 内资源直传路径）：两者全内嵌、
    // 路径字段留空 → 加载成功且内容来自内嵌文本。
    let dir = tempfile::tempdir().unwrap();
    let args = test_runtime_args("falcon9");
    let lp_text = std::fs::read_to_string(
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../../sim-rocket/launchpads/lp_default.toml"),
    )
    .unwrap();
    let _ = dir; // lp 用单源文本内嵌；scenario 用内嵌文本。
    let spec = super::SessionSpec {
        rocket: Some("falcon9".into()),
        scenario_toml: Some(SCN_TWO_BODIES.into()),
        launchpad_toml: Some(lp_text),
        control: Some("target".into()),
        ..Default::default()
    };
    let session = super::load_session(&spec, &args.ephemeris_data).unwrap();
    assert_eq!(session.scenario.name, "TestSys");
    assert!(session.launchpad.is_some());
}

#[test]
fn launchpad_on_secondary_body_binds_by_name() {
    let dir = tempfile::tempdir().unwrap();
    let mut args = test_runtime_args("falcon9");
    args.scenario = Some((write_scenario(dir.path())).display().to_string());
    args.launchpad = Some(write_lp_body(dir.path(), "Moon"));
    let session = args.load_session().unwrap();
    let lp = session.launchpad.unwrap();
    assert_eq!(lp.body_name, "Moon");
    assert!(!lp.is_primary);
    assert!((lp.body_size_m - 1_737_100.0).abs() < 1e-9);

    // 未知天体 → 报错并列出可用天体
    let mut args2 = test_runtime_args("falcon9");
    args2.scenario = Some((write_scenario(dir.path())).display().to_string());
    args2.launchpad = Some(write_lp_body(dir.path(), "Pluto"));
    let err = args2.load_session().unwrap_err();
    assert!(err.contains("不在环境天体表中"), "{err}");
    assert!(err.contains("Earth"), "{err}");
}

#[test]
fn surface_package_checked_against_bound_body() {
    // 包是地球半径、发射场在月球 → 加载期报错（对月球 size 校验）
    let dir = tempfile::tempdir().unwrap();
    let mut args = test_runtime_args("falcon9");
    args.scenario = Some((write_scenario(dir.path())).display().to_string());
    args.launchpad = Some(write_lp_body(dir.path(), "Moon"));
    args.surface_package = Some(write_fake_package(dir.path(), 6_371_010.0));
    let err = args.load_session().unwrap_err();
    assert!(err.contains("绑定天体 `Moon`"), "{err}");

    // 包半径与月球相符 → 加载通过且会话绑定 Moon
    let dir2 = tempfile::tempdir().unwrap();
    let mut args2 = test_runtime_args("falcon9");
    args2.scenario = Some((write_scenario(dir2.path())).display().to_string());
    args2.launchpad = Some(write_lp_body(dir2.path(), "Moon"));
    args2.surface_package = Some(write_fake_package(dir2.path(), 1_737_100.0));
    let session = args2.load_session().unwrap();
    assert_eq!(session.surface.unwrap().body_name, "Moon");
}

#[test]
fn launchpad_body_mismatch_and_override_needs_package() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("lp_x.toml");
    std::fs::write(
        &path,
        "schema=2\nname='X'\nbody='Mars'\nlatitude_deg=0.0\nlongitude_deg=0.0\nbase_elevation_m=0.0\n[[pads]]\nname='P'\n",
    )
    .unwrap();
    let mut args = test_runtime_args("falcon9");
    args.launchpad = Some(path);
    let err = args.load_session().unwrap_err();
    assert!(err.contains("body"), "{err}");

    // 带覆盖网格但无地表包 → 报错
    let dir2 = tempfile::tempdir().unwrap();
    let path2 = dir2.path().join("lp_y.toml");
    std::fs::write(
        &path2,
        "schema=2\nname='Y'\nbody='Earth'\nlatitude_deg=0.0\nlongitude_deg=90.0\nbase_elevation_m=0.0\nelevation_override='g.f32'\ngrid_cell_m=10.0\ngrid_width=3\ngrid_height=3\n[[pads]]\nname='P'\n",
    )
    .unwrap();
    let mut bytes = Vec::new();
    for _ in 0..9 {
        bytes.extend_from_slice(&1.0f32.to_le_bytes());
    }
    std::fs::write(dir2.path().join("g.f32"), bytes).unwrap();
    let mut args2 = test_runtime_args("falcon9");
    args2.launchpad = Some(path2);
    let err2 = args2.load_session().unwrap_err();
    assert!(err2.contains("surface-package"), "{err2}");

    // --pad 无 --launchpad → 报错
    let mut args3 = test_runtime_args("falcon9");
    args3.pad = Some("P".into());
    assert!(args3.load_session().is_err());
}
