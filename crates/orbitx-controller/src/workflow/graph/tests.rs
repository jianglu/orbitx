use super::*;
use crate::workflow::graph::runtime::GraphWorkFlow;
use crate::workflow::WorkFlow;
use orbitx_math::{cross, Matrix3, Quat, StateVectors, Vec3};
use orbitx_vessel::{Assembly, StageSpec, ThrusterSpec};

fn hold_spec(name: &'static str) -> StageSpec {
    StageSpec {
        name: name.into(),
        dry_mass: 10_000.0,
        thrusters: vec![ThrusterSpec {
            pos: Vec3::new(0.0, -15.0, 0.0),
            dir: Vec3::new(0.0, 1.0, 0.0),
            thrust: 800_000.0,
            isp: 300.0,
            max_gimbal: 0.15,
            max_gimbal_rate: 1.0,
            gimbal_axis: Vec3::new(1.0, 0.0, 0.0),
            ..Default::default()
        }],
        length: 30.0,
        radius: 1.5,
        ..Default::default()
    }
    .with_fuel(40_000.0)
}

fn launch_asm(name: &'static str) -> Assembly {
    let earth_r = 6_371_000.0;
    let pos = Vec3::new(0.0, 0.0, earth_r + 20.0);
    let up = pos * (1.0 / pos.length());
    let ref_axis = Vec3::new(0.0, 1.0, 0.0);
    let bx = cross(up, ref_axis).unit();
    let bz = cross(bx, up).unit();
    let by = up;
    let rot = Matrix3::new(bx.x, by.x, bz.x, bx.y, by.y, bz.y, bx.z, by.z, bz.z);
    let q = Quat::from_matrix(rot);
    let mut asm = Assembly::new(
        &[hold_spec(name)],
        StateVectors {
            pos,
            vel: Vec3::ZERO,
            omega: Vec3::ZERO,
            r: rot,
            q,
        },
    );
    asm.planet_radius = earth_r;
    asm
}

fn two_stage() -> Assembly {
    let earth_r = 6_371_000.0;
    let pos = Vec3::new(0.0, 0.0, earth_r + 20.0);
    let up = pos * (1.0 / pos.length());
    let ref_axis = Vec3::new(0.0, 1.0, 0.0);
    let bx = cross(up, ref_axis).unit();
    let bz = cross(bx, up).unit();
    let by = up;
    let rot = Matrix3::new(bx.x, by.x, bz.x, bx.y, by.y, bz.y, bx.z, by.z, bz.z);
    let mut asm = Assembly::new(
        &[hold_spec("Core"), hold_spec("Upper")],
        StateVectors {
            pos,
            vel: Vec3::ZERO,
            omega: Vec3::ZERO,
            r: rot,
            q: Quat::from_matrix(rot),
        },
    );
    asm.planet_radius = earth_r;
    asm
}

const MIN_TARGET: &str = r#"
schema = 1
kind = "target"
name = "demo"

[[nodes]]
id = "ign0"
type = "ignite"

[[nodes]]
id = "m1"
type = "main"
stage = "hold"

[[nodes]]
id = "d1"
type = "drive"
mode = "vertical_hold"
throttle = 1.0

[[edges]]
from = "ign0"
to = "m1"

[[edges]]
from = "m1"
from_port = "flight"
to = "d1"
"#;

#[test]
fn parse_round_trip() {
    let ir = from_toml_str(MIN_TARGET, None).expect("parse");
    let text = ir.to_toml().expect("ser");
    let ir2 = from_toml_str(&text, None).expect("reparse");
    assert_eq!(ir, ir2);
}

#[test]
fn reject_ignite_fanout() {
    let s = r#"
schema = 1
kind = "target"
name = "x"
[[nodes]]
id = "ign0"
type = "ignite"
[[nodes]]
id = "m1"
type = "main"
stage = "hold"
[[nodes]]
id = "m2"
type = "main"
stage = "hold"
[[edges]]
from = "ign0"
to = "m1"
[[edges]]
from = "ign0"
to = "m2"
"#;
    let err = from_toml_str(s, None).unwrap_err();
    let msg = err.to_string();
    assert!(msg.contains("恰好"), "{msg}");
}

#[test]
fn reject_flight_separate_and_activate_without_next() {
    let sep = r#"
schema = 1
kind = "target"
name = "x"
[[nodes]]
id = "ign0"
type = "ignite"
[[nodes]]
id = "m1"
type = "main"
stage = "hold"
[[nodes]]
id = "d1"
type = "drive"
action = "separate"
point = "stage-sep"
[[edges]]
from = "ign0"
to = "m1"
[[edges]]
from = "m1"
from_port = "flight"
to = "d1"
"#;
    assert!(from_toml_str(sep, None).is_err());

    let act = r#"
schema = 1
kind = "super"
name = "x"
[[nodes]]
id = "ign0"
type = "ignite"
[[nodes]]
id = "m1"
type = "main"
stage = "Core"
[[nodes]]
id = "a1"
type = "activate"
invalidate_current = true
[[edges]]
from = "ign0"
to = "m1"
[[edges]]
from = "m1"
from_port = "sep"
to = "a1"
"#;
    let err = from_toml_str(act, None).unwrap_err();
    assert!(err.to_string().contains("下一主"), "{}", err);
}

#[test]
fn ignite_once_and_idle_before() {
    let asm = launch_asm("hold");
    let ir = from_toml_str(MIN_TARGET, None).unwrap();
    let mut wf = GraphWorkFlow::try_new(ir, &asm, 0.0).unwrap();
    assert_eq!(wf.main_state("m1"), Some(MainState::Idle));
    let mut asm = asm;
    wf.tick(&mut asm, &orbitx_protocol::mem::Slice::default(), 0.1);
    assert_eq!(wf.fires("d1"), 0, "IDLE 不发令");
    wf.ignite();
    assert!(wf.ignite_spent());
    assert_eq!(wf.main_state("m1"), Some(MainState::Active));
    wf.ignite();
    assert_eq!(wf.main_state("m1"), Some(MainState::Active));
}

#[test]
fn invalid_main_does_not_drive() {
    let asm = launch_asm("hold");
    let ir = from_toml_str(MIN_TARGET, None).unwrap();
    let mut wf = GraphWorkFlow::try_new(ir, &asm, 0.0).unwrap();
    wf.ignite();
    wf.force_state("m1", MainState::Invalid);
    let mut asm = asm;
    wf.tick(&mut asm, &orbitx_protocol::mem::Slice::default(), 0.1);
    assert_eq!(wf.fires("d1"), 0);
}

#[test]
fn condition_true_false_ports() {
    let s = r#"
schema = 1
kind = "target"
name = "x"
[[nodes]]
id = "ign0"
type = "ignite"
[[nodes]]
id = "m1"
type = "main"
stage = "hold"
[[nodes]]
id = "c1"
type = "condition"
combine = "and"
terms = [{ kind = "time", op = "gt", value = 0.0 }]
[[nodes]]
id = "dt"
type = "drive"
mode = "vertical_hold"
throttle = 1.0
[[nodes]]
id = "df"
type = "drive"
mode = "vertical_hold"
throttle = 0.0
[[edges]]
from = "ign0"
to = "m1"
[[edges]]
from = "m1"
from_port = "flight"
to = "c1"
[[edges]]
from = "c1"
from_port = "true"
to = "dt"
[[edges]]
from = "c1"
from_port = "false"
to = "df"
"#;
    let asm = launch_asm("hold");
    let ir = from_toml_str(s, None).unwrap();
    let mut wf = GraphWorkFlow::try_new(ir, &asm, 0.0).unwrap();
    wf.ignite();
    let mut asm = asm;
    wf.tick(&mut asm, &orbitx_protocol::mem::Slice::default(), 0.1);
    assert_eq!(wf.fires("df"), 1, "首拍 time=0 走 false");
    assert_eq!(wf.fires("dt"), 0);
    wf.tick(&mut asm, &orbitx_protocol::mem::Slice::default(), 0.1);
    assert_eq!(wf.fires("dt"), 1, "次拍累计时间后走 true");
}

#[test]
fn delay_finite_and_infinite() {
    let finite = r#"
schema = 1
kind = "target"
name = "x"
[[nodes]]
id = "ign0"
type = "ignite"
[[nodes]]
id = "m1"
type = "main"
stage = "hold"
[[nodes]]
id = "dly"
type = "delay"
count = 2
interval_s = 1.0
[[nodes]]
id = "d1"
type = "drive"
mode = "vertical_hold"
throttle = 1.0
[[edges]]
from = "ign0"
to = "m1"
[[edges]]
from = "m1"
from_port = "flight"
to = "dly"
[[edges]]
from = "dly"
to = "d1"
"#;
    let asm = launch_asm("hold");
    let ir = from_toml_str(finite, None).unwrap();
    let mut wf = GraphWorkFlow::try_new(ir, &asm, 0.0).unwrap();
    wf.ignite();
    let mut asm = asm;
    for _ in 0..5 {
        wf.tick(&mut asm, &orbitx_protocol::mem::Slice::default(), 1.0);
    }
    assert_eq!(wf.fires("dly"), 2);
    assert_eq!(wf.fires("d1"), 2);

    let infinite = finite.replace("count = 2", "count = 0");
    let ir = from_toml_str(&infinite, None).unwrap();
    let mut wf = GraphWorkFlow::try_new(ir, &asm, 0.0).unwrap();
    wf.ignite();
    for _ in 0..5 {
        wf.tick(&mut asm, &orbitx_protocol::mem::Slice::default(), 1.0);
    }
    assert!(wf.fires("dly") >= 5, "无限延时应每拍脉冲");
}

#[test]
fn target_mode_hold_sets_throttle_cmd() {
    let asm = launch_asm("hold");
    let ir = from_toml_str(MIN_TARGET, None).unwrap();
    let mut wf = GraphWorkFlow::try_new(ir, &asm, 0.0).unwrap();
    wf.ignite();
    let mut asm = asm;
    wf.tick(&mut asm, &orbitx_protocol::mem::Slice::default(), 0.05);
    let cmd = asm.vessels[0].thrusters[0].level_cmd;
    assert!(
        cmd > 0.9,
        "vertical_hold throttle=1 应写入油门指令，实际 {cmd}"
    );
}

#[test]
fn multi_active_both_drive() {
    let s = r#"
schema = 1
kind = "target"
name = "x"
[[nodes]]
id = "ign0"
type = "ignite"
[[nodes]]
id = "m1"
type = "main"
stage = "Core"
[[nodes]]
id = "m2"
type = "main"
stage = "Upper"
[[nodes]]
id = "d1"
type = "drive"
mode = "vertical_hold"
throttle = 1.0
[[nodes]]
id = "d2"
type = "drive"
mode = "vertical_hold"
throttle = 0.5
[[edges]]
from = "ign0"
to = "m1"
[[edges]]
from = "m1"
from_port = "flight"
to = "d1"
[[edges]]
from = "m2"
from_port = "flight"
to = "d2"
"#;
    let asm = two_stage();
    let ir = from_toml_str(s, None).unwrap();
    let mut wf = GraphWorkFlow::try_new(ir, &asm, 0.0).unwrap();
    wf.ignite();
    wf.force_state("m2", MainState::Active);
    let mut asm = asm;
    wf.tick(&mut asm, &orbitx_protocol::mem::Slice::default(), 0.05);
    assert_eq!(wf.fires("d1"), 1);
    assert_eq!(wf.fires("d2"), 1);
}

#[test]
fn super_throttle_tvc_rcs() {
    let s = r#"
schema = 1
kind = "super"
name = "x"
[[nodes]]
id = "ign0"
type = "ignite"
[[nodes]]
id = "m1"
type = "main"
stage = "hold"
[[nodes]]
id = "th"
type = "drive"
action = "throttle"
group = "hold"
level = 0.4
[[nodes]]
id = "tv"
type = "drive"
action = "tvc"
group = "hold-tvc"
pitch = 0.0
yaw = 0.0
[[nodes]]
id = "rc"
type = "drive"
action = "rcs"
group = "hold-pitch_up"
axis = "pitch"
level = 1.0
[[edges]]
from = "ign0"
to = "m1"
[[edges]]
from = "m1"
from_port = "flight"
to = "th"
[[edges]]
from = "m1"
from_port = "flight"
to = "tv"
[[edges]]
from = "m1"
from_port = "flight"
to = "rc"
"#;
    let mut asm = launch_asm("hold");
    orbitx_vessel::rcs::add_default_rcs(&mut asm.vessels[0], 5.0, 10_000.0);
    let ir = from_toml_str(s, None).unwrap();
    let mut wf = GraphWorkFlow::try_new(ir, &asm, 0.0).unwrap();
    wf.ignite();
    wf.tick(&mut asm, &orbitx_protocol::mem::Slice::default(), 0.05);
    assert!((asm.vessels[0].thrusters[0].level_cmd - 0.4).abs() < 1e-6);
    assert_eq!(wf.fires("tv"), 1);
    assert_eq!(wf.fires("rc"), 1);
}

#[test]
fn reject_target_file_with_throttle_action() {
    let s = r#"
schema = 1
kind = "target"
name = "x"
[[nodes]]
id = "ign0"
type = "ignite"
[[nodes]]
id = "m1"
type = "main"
stage = "hold"
[[nodes]]
id = "d1"
type = "drive"
action = "throttle"
group = "hold"
level = 1.0
[[edges]]
from = "ign0"
to = "m1"
[[edges]]
from = "m1"
from_port = "flight"
to = "d1"
"#;
    assert!(from_toml_str(s, None).is_err());
}

#[test]
fn separate_and_activate_once() {
    let s = r#"
schema = 1
kind = "super"
name = "x"
[[nodes]]
id = "ign0"
type = "ignite"
[[nodes]]
id = "m1"
type = "main"
stage = "Core"
[[nodes]]
id = "m2"
type = "main"
stage = "Upper"
[[nodes]]
id = "sep"
type = "drive"
action = "separate"
point = "stage-sep"
[[nodes]]
id = "act"
type = "activate"
invalidate_current = true
[[edges]]
from = "ign0"
to = "m1"
[[edges]]
from = "m1"
from_port = "sep"
to = "sep"
[[edges]]
from = "m1"
from_port = "sep"
to = "act"
[[edges]]
from = "act"
from_port = "next"
to = "m2"
"#;
    let asm = two_stage();
    let n0 = asm.vessels.len();
    let ir = from_toml_str(s, None).unwrap();
    let mut wf = GraphWorkFlow::try_new(ir, &asm, 0.0).unwrap();
    wf.ignite();
    let mut asm = asm;
    for _ in 0..4 {
        wf.tick(&mut asm, &orbitx_protocol::mem::Slice::default(), 0.05);
    }
    assert_eq!(wf.fires("sep"), 1, "分离器一次有效");
    assert_eq!(wf.fires("act"), 1, "激活一次有效");
    assert!(asm.vessels.len() >= n0);
    assert_eq!(wf.main_state("m1"), Some(MainState::Invalid));
    assert_eq!(wf.main_state("m2"), Some(MainState::Active));
}

#[test]
fn activate_can_keep_current() {
    let s = r#"
schema = 1
kind = "super"
name = "x"
[[nodes]]
id = "ign0"
type = "ignite"
[[nodes]]
id = "m1"
type = "main"
stage = "Core"
[[nodes]]
id = "m2"
type = "main"
stage = "Upper"
[[nodes]]
id = "act"
type = "activate"
invalidate_current = false
[[edges]]
from = "ign0"
to = "m1"
[[edges]]
from = "m1"
from_port = "sep"
to = "act"
[[edges]]
from = "act"
from_port = "next"
to = "m2"
"#;
    let asm = two_stage();
    let ir = from_toml_str(s, None).unwrap();
    let mut wf = GraphWorkFlow::try_new(ir, &asm, 0.0).unwrap();
    wf.ignite();
    let mut asm = asm;
    wf.tick(&mut asm, &orbitx_protocol::mem::Slice::default(), 0.05);
    assert_eq!(wf.main_state("m1"), Some(MainState::Active));
    assert_eq!(wf.main_state("m2"), Some(MainState::Active));
}

#[test]
fn reject_activate_without_sep_capability() {
    let s = r#"
schema = 1
kind = "target"
name = "x"
[[nodes]]
id = "ign0"
type = "ignite"
[[nodes]]
id = "m1"
type = "main"
stage = "hold"
[[nodes]]
id = "m2"
type = "main"
stage = "hold"
[[nodes]]
id = "act"
type = "activate"
invalidate_current = true
[[edges]]
from = "ign0"
to = "m1"
[[edges]]
from = "m1"
from_port = "sep"
to = "act"
[[edges]]
from = "act"
from_port = "next"
to = "m2"
"#;
    let asm = launch_asm("hold");
    let ir = from_toml_str(s, None).unwrap();
    let err = match GraphWorkFlow::try_new(ir, &asm, 0.0) {
        Err(e) => e,
        Ok(_) => panic!("应拒绝无分离口的激活"),
    };
    assert!(
        err.to_string().contains("未露出") || err.to_string().contains("分离"),
        "{err}"
    );
}
