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

// ---- Halt 停机指令（WORKFLOW.md §5.5 参数化原语）----

const HALT_TARGET: &str = r#"
schema = 1
kind = "target"
name = "halt"
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
[[nodes]]
id = "h1"
type = "halt"
[[edges]]
from = "ign0"
to = "m1"
[[edges]]
from = "m1"
from_port = "flight"
to = "d1"
[[edges]]
from = "m1"
from_port = "flight"
to = "h1"
"#;

/// Super 图双 halt 模板（C3b 多 halt 分时锁存用例）：前缀 hp（三关静默，
/// 出边 1 → 增量段入口 m2，合法拓扑见 super_halt_topology_end_and_prefix）
/// + 末段 he（缺省全真，出边 0）。m2 经 hp 出边可达， ignite 恰连 m1。
const HALT_SUPER_TWO: &str = r#"
schema = 1
kind = "super"
name = "halt_two"

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

[[nodes]]
id = "hp"
type = "halt"
shutdown_controls = false
stop_stepping = false
save_recorder = false

[[nodes]]
id = "m2"
type = "main"
stage = "hold"

[[nodes]]
id = "d2"
type = "drive"
action = "throttle"
group = "hold"
level = 1.0

[[nodes]]
id = "he"
type = "halt"

[[edges]]
from = "ign0"
to = "m1"

[[edges]]
from = "m1"
from_port = "flight"
to = "d1"

[[edges]]
from = "m1"
from_port = "flight"
to = "hp"

[[edges]]
from = "hp"
to = "m2"

[[edges]]
from = "m2"
from_port = "flight"
to = "d2"

[[edges]]
from = "m2"
from_port = "flight"
to = "he"
"#;

#[test]
fn halt_parse_defaults_are_all_true() {
    // 旧文件不含三字段 → 解析为缺省全真 = 真停机（向后兼容，schema 维持 1）。
    let ir = from_toml_str(HALT_TARGET, None).unwrap();
    let h = ir
        .nodes
        .iter()
        .find(|n| matches!(n.body, NodeBody::Halt { .. }))
        .expect("halt 节点");
    match &h.body {
        NodeBody::Halt {
            shutdown_controls,
            stop_stepping,
            save_recorder,
            ..
        } => {
            assert!(*shutdown_controls && *stop_stepping && *save_recorder);
        }
        other => panic!("unexpected {other:?}"),
    }
}

#[test]
fn halt_parse_explicit_params_round_trip() {
    let s = HALT_TARGET.replace(
        "type = \"halt\"",
        "type = \"halt\"\nshutdown_controls = true\nstop_stepping = false\nsave_recorder = false",
    );
    let ir = from_toml_str(&s, None).unwrap();
    let h = ir
        .nodes
        .iter()
        .find(|n| matches!(n.body, NodeBody::Halt { .. }))
        .unwrap();
    match &h.body {
        NodeBody::Halt {
            shutdown_controls,
            stop_stepping,
            save_recorder,
            ..
        } => {
            assert!(*shutdown_controls && !*stop_stepping && !*save_recorder);
        }
        other => panic!("unexpected {other:?}"),
    }
    let text = ir.to_toml().unwrap();
    let ir2 = from_toml_str(&text, None).unwrap();
    assert_eq!(ir, ir2);
}

#[test]
fn halt_parse_title_round_trip() {
    // title = 呈现标识能力字段（WORKFLOW.md §5.5）：解析保留 + round-trip 不丢；
    // 旧文件形态（无 title）→ None，序列化跳过该键（skip_serializing_if）。
    let s = HALT_TARGET.replace(
        "type = \"halt\"",
        "type = \"halt\"\ntitle = \"发射入轨·目标完成\"",
    );
    let ir = from_toml_str(&s, None).unwrap();
    let h = ir
        .nodes
        .iter()
        .find(|n| matches!(n.body, NodeBody::Halt { .. }))
        .unwrap();
    match &h.body {
        NodeBody::Halt { title, .. } => {
            assert_eq!(title.as_deref(), Some("发射入轨·目标完成"));
        }
        other => panic!("unexpected {other:?}"),
    }
    let text = ir.to_toml().unwrap();
    assert!(text.contains("title = \"发射入轨·目标完成\""), "{text}");
    let ir2 = from_toml_str(&text, None).unwrap();
    assert_eq!(ir, ir2);

    let ir_plain = from_toml_str(HALT_TARGET, None).unwrap();
    let h_plain = ir_plain
        .nodes
        .iter()
        .find(|n| matches!(n.body, NodeBody::Halt { .. }))
        .unwrap();
    match &h_plain.body {
        NodeBody::Halt { title, .. } => assert!(title.is_none(), "旧文件无 title = None"),
        other => panic!("unexpected {other:?}"),
    }
    assert!(
        !ir_plain.to_toml().unwrap().contains("title"),
        "None 不应输出 title 键"
    );
}

#[test]
fn halt_all_off_latches_silent_action() {
    // 三全关（零指令）：同样锁存 HaltAction{false,false,false}——runtime 分派
    // 两个 if 均不命中 → 零收尾动作、无 Halted 上行（端到端固化见 orbitx-runtime
    // host/tests.rs halt_all_off_is_silent）；执行流走出边锚点（前缀边界切换）
    // 属 C5 修订，不在本断言范围。shutdown_controls=false → 控制输出不清零。
    let s = HALT_TARGET.replace(
        "type = \"halt\"",
        "type = \"halt\"\nshutdown_controls = false\nstop_stepping = false\nsave_recorder = false",
    );
    let asm = launch_asm("hold");
    let ir = from_toml_str(&s, None).unwrap();
    let mut wf = GraphWorkFlow::try_new(ir, &asm, 0.0).unwrap();
    wf.ignite();
    let mut asm = asm;
    wf.tick(&mut asm, &orbitx_protocol::mem::Slice::default(), 0.05);
    let act = wf
        .halt_action()
        .expect("三全关同样锁存（动作内容 = 零指令）");
    assert!(!act.shutdown_controls && !act.stop_stepping && !act.save_recorder);
    let cmd = asm.vessels[0].thrusters[0].level_cmd;
    assert!(cmd > 0.9, "零指令不清零：drive 输出保持，实际 {cmd}");
}

#[test]
fn reject_target_halt_with_outgoing_edge() {
    let s = HALT_TARGET.replace(
        "[[edges]]\nfrom = \"m1\"\nfrom_port = \"flight\"\nto = \"h1\"",
        "[[edges]]\nfrom = \"m1\"\nfrom_port = \"flight\"\nto = \"h1\"\n[[edges]]\nfrom = \"h1\"\nto = \"d1\"",
    );
    let err = from_toml_str(&s, None).unwrap_err();
    assert!(err.to_string().contains("出边数须为 0"), "{err}");
}

#[test]
fn reject_drive_source_to_halt() {
    // 入边闭集：驱动 → halt 拒连（驱动本不允许出边；halt 只接受
    // 主节点口 / 条件 / 延时来源）。
    let s = HALT_TARGET.replace(
        "[[edges]]\nfrom = \"m1\"\nfrom_port = \"flight\"\nto = \"h1\"",
        "[[nodes]]\nid = \"d2\"\ntype = \"drive\"\naction = \"throttle\"\ngroup = \"hold\"\nlevel = 0.5\n[[edges]]\nfrom = \"m1\"\nfrom_port = \"flight\"\nto = \"h1\"\n[[edges]]\nfrom = \"d2\"\nto = \"h1\"",
    );
    assert!(from_toml_str(&s, None).is_err());
}

#[test]
fn condition_to_halt_is_accepted() {
    // 条件 true 口 → halt：闭集合法来源。
    let s = HALT_TARGET.replace(
        "[[edges]]\nfrom = \"m1\"\nfrom_port = \"flight\"\nto = \"h1\"",
        "[[nodes]]\nid = \"c1\"\ntype = \"condition\"\ncombine = \"and\"\nterms = [{ kind = \"time\", op = \"gt\", value = 0.0 }]\n[[edges]]\nfrom = \"m1\"\nfrom_port = \"flight\"\nto = \"c1\"\n[[edges]]\nfrom = \"c1\"\nfrom_port = \"true\"\nto = \"h1\"",
    );
    from_toml_str(&s, None).unwrap_or_else(|e| panic!("条件→halt 应合法：{e}"));
}

#[test]
fn reject_delay_source_to_halt_via_bad_port() {
    // 延时来源合法但端口错（非 out）→ 端口检查拒。
    let s = HALT_TARGET.replace(
        "[[edges]]\nfrom = \"m1\"\nfrom_port = \"flight\"\nto = \"h1\"",
        "[[nodes]]\nid = \"dl\"\ntype = \"delay\"\ncount = 1\ninterval_s = 1.0\n[[edges]]\nfrom = \"m1\"\nfrom_port = \"flight\"\nto = \"dl\"\n[[edges]]\nfrom = \"dl\"\nfrom_port = \"true\"\nto = \"h1\"",
    );
    let err = from_toml_str(&s, None).unwrap_err();
    assert!(err.to_string().contains("不允许端口"), "{err}");
}

#[test]
fn delay_to_halt_is_accepted() {
    // 延时 out → halt：合法来源。
    let s = HALT_TARGET.replace(
        "[[edges]]\nfrom = \"m1\"\nfrom_port = \"flight\"\nto = \"h1\"",
        "[[nodes]]\nid = \"dl\"\ntype = \"delay\"\ncount = 1\ninterval_s = 0.0\n[[edges]]\nfrom = \"m1\"\nfrom_port = \"flight\"\nto = \"dl\"\n[[edges]]\nfrom = \"dl\"\nto = \"h1\"",
    );
    from_toml_str(&s, None).unwrap_or_else(|e| panic!("延时→halt 应合法：{e}"));
}

#[test]
fn super_halt_topology_end_and_prefix() {
    // Super：恰好一个末段（出边 0）+ 前缀（出边 1 → 主节点）。
    let base = r#"
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
id = "hp"
type = "halt"
[[nodes]]
id = "he"
type = "halt"
[[edges]]
from = "ign0"
to = "m1"
[[edges]]
from = "m1"
from_port = "flight"
to = "hp"
[[edges]]
from = "hp"
to = "m1"
[[edges]]
from = "m1"
from_port = "sep"
to = "he"
"#;
    // hold 级无分离口 → sep 接线被拒（域检查先于 halt 拓扑）；换 flight 口挂末段。
    let ok = base.replace(
        "[[edges]]\nfrom = \"m1\"\nfrom_port = \"sep\"\nto = \"he\"",
        "[[edges]]\nfrom = \"m1\"\nfrom_port = \"flight\"\nto = \"he\"",
    );
    from_toml_str(&ok, None).unwrap_or_else(|e| panic!("super 前缀+末段 halt 应合法：{e}"));

    // 两个末段 → 拒。
    let bad = base.replace("[[edges]]\nfrom = \"hp\"\nto = \"m1\"", "");
    let err = from_toml_str(&bad, None).unwrap_err();
    assert!(err.to_string().contains("末段停机"), "{err}");

    // 前缀出边 2 条 → 拒。
    let bad2 = base.replace(
        "[[edges]]\nfrom = \"hp\"\nto = \"m1\"",
        "[[edges]]\nfrom = \"hp\"\nto = \"m1\"\n[[edges]]\nfrom = \"hp\"\nto = \"m1\"",
    );
    let err2 = from_toml_str(&bad2, None).unwrap_err();
    assert!(err2.to_string().contains("出边须恰好 1"), "{err2}");

    // 前缀出边指向非主节点 → 拒（d2 本身合法挂在 flight 口下）。
    let bad3 = base.replace(
        "[[edges]]\nfrom = \"hp\"\nto = \"m1\"",
        "[[nodes]]\nid = \"d2\"\ntype = \"drive\"\naction = \"throttle\"\ngroup = \"hold\"\nlevel = 0.5\n[[edges]]\nfrom = \"m1\"\nfrom_port = \"flight\"\nto = \"d2\"\n[[edges]]\nfrom = \"hp\"\nto = \"d2\"",
    );
    let err3 = from_toml_str(&bad3, None).unwrap_err();
    assert!(err3.to_string().contains("指向主节点"), "{err3}");
}

#[test]
fn halt_activation_latches_params_and_zeroes_output() {
    // 激活当拍：drive 先写油门，锁存后 tick 尾清零 → level_cmd = 0；
    // 后续拍 drive 再次写入，锁存断言持续清零；参数经 halt_action() 透传。
    let s = HALT_TARGET.replace(
        "type = \"halt\"",
        "type = \"halt\"\nshutdown_controls = true\nstop_stepping = false\nsave_recorder = true",
    );
    let asm = launch_asm("hold");
    let ir = from_toml_str(&s, None).unwrap();
    let mut wf = GraphWorkFlow::try_new(ir, &asm, 0.0).unwrap();
    assert!(wf.halt_action().is_none(), "未激活 = None");
    wf.ignite();
    let mut asm = asm;
    wf.tick(&mut asm, &orbitx_protocol::mem::Slice::default(), 0.05);
    assert_eq!(wf.fires("h1"), 1, "首次激活计数 1 次");
    let act = wf.halt_action().expect("激活后锁存");
    assert!(act.shutdown_controls);
    assert!(!act.stop_stepping, "参数透传 stop_stepping=false");
    assert!(act.save_recorder, "参数透传 save_recorder=true");
    let cmd = asm.vessels[0].thrusters[0].level_cmd;
    assert!(cmd < 1e-9, "激活当拍清零，实际 {cmd}");
    // 锁存后续拍仍为零（drive 每拍重写 → tick 尾断言覆盖）。
    wf.tick(&mut asm, &orbitx_protocol::mem::Slice::default(), 0.05);
    let cmd2 = asm.vessels[0].thrusters[0].level_cmd;
    assert!(cmd2 < 1e-9, "锁存后每拍持续清零，实际 {cmd2}");
    assert_eq!(wf.fires("h1"), 1, "锁存幂等：不重复计数");
}

#[test]
fn halt_shutdown_controls_false_keeps_drive_output() {
    // shutdown_controls=false：不清零（任务二形态：禁停控制）。
    let s = HALT_TARGET.replace(
        "type = \"halt\"",
        "type = \"halt\"\nshutdown_controls = false\nstop_stepping = true\nsave_recorder = false",
    );
    let asm = launch_asm("hold");
    let ir = from_toml_str(&s, None).unwrap();
    let mut wf = GraphWorkFlow::try_new(ir, &asm, 0.0).unwrap();
    wf.ignite();
    let mut asm = asm;
    wf.tick(&mut asm, &orbitx_protocol::mem::Slice::default(), 0.05);
    let act = wf.halt_action().expect("锁存");
    assert!(!act.shutdown_controls && act.stop_stepping && !act.save_recorder);
    let cmd = asm.vessels[0].thrusters[0].level_cmd;
    assert!(cmd > 0.9, "未清零：drive 输出保持，实际 {cmd}");
}

#[test]
fn halt_reset_state_reenables_stepping() {
    // Reset 接线：复位锁存与图状态（fires 清零、重新可点火）；重跑同一张图
    // halt 仍会再次锁存并清零——行为可重复，旧锁存不再泄漏进新会话。
    let asm = launch_asm("hold");
    let ir = from_toml_str(HALT_TARGET, None).unwrap();
    let mut wf = GraphWorkFlow::try_new(ir, &asm, 0.0).unwrap();
    wf.ignite();
    let mut asm = asm;
    wf.tick(&mut asm, &orbitx_protocol::mem::Slice::default(), 0.05);
    assert!(wf.halt_action().is_some());
    let cmd = asm.vessels[0].thrusters[0].level_cmd;
    assert!(cmd < 1e-9);
    wf.reset_state();
    assert!(wf.halt_action().is_none(), "复位后锁存清空");
    assert_eq!(wf.fires("h1"), 0, "复位后触发计数清零");
    // 重新点火：图从头执行，halt 仍在图中 → 再次锁存 + 清零（无旧锁存泄漏）。
    wf.ignite();
    wf.tick(&mut asm, &orbitx_protocol::mem::Slice::default(), 0.05);
    assert_eq!(wf.fires("h1"), 1, "重新点火后 halt 再次锁存");
    let act = wf.halt_action().expect("重新锁存");
    assert!(act.shutdown_controls && act.stop_stepping && act.save_recorder);
    let cmd2 = asm.vessels[0].thrusters[0].level_cmd;
    assert!(cmd2 < 1e-9, "再次停机清零，实际 {cmd2}");
}

#[test]
fn halt_multi_activation_refreshes_latch() {
    // C3b 分时锁存经 C5 修订（WORKFLOW.md §5.5）：前缀 hp 出边 1 = 边界锚点，
    // **不再锁存**（零停机动作，三关静默参数不消费）；末段 he（出边 0）激活
    // 锁存全真——「前缀锁存 + 末段覆盖刷新」旧形态作废，本用例改写为真实
    // 图执行流边界切换（C5 前 force_state 等价模拟）。
    // 单文件 super 整图 = 合成图（前缀 hp 图内出边 → m2 合法拓扑）。
    let asm = launch_asm("hold");
    let ir = from_toml_str(HALT_SUPER_TWO, None).unwrap();
    let mut wf = GraphWorkFlow::try_new(ir, &asm, 0.0).unwrap();
    wf.ignite();
    let mut asm = asm;
    // 任务一段：hp 边界锚点当拍激活——不锁存（三关静默参数被忽略）、不清零；
    // 同拍 m2 已激活（快照口径 m1 子树 d1 仍发令一拍）。
    wf.tick(&mut asm, &orbitx_protocol::mem::Slice::default(), 0.05);
    assert_eq!(wf.fires("hp"), 1, "边界锚点首次激活计数");
    assert!(wf.halt_action().is_none(), "边界锚点不锁存（零停机动作）");
    assert_eq!(wf.fires("he"), 0, "末段未激活未计数");
    assert_eq!(wf.main_state("m1"), Some(MainState::Invalid), "前缀失效");
    assert_eq!(wf.main_state("m2"), Some(MainState::Active), "增量段激活");
    // 任务二段：d2 直驱 + he 末段激活锁存全真（真停机）。
    wf.tick(&mut asm, &orbitx_protocol::mem::Slice::default(), 0.05);
    let act2 = wf.halt_action().expect("末段激活锁存");
    assert!(
        act2.shutdown_controls && act2.stop_stepping && act2.save_recorder,
        "末段取值（缺省全真）：{act2:?}"
    );
    let cmd2 = asm.vessels[0].thrusters[0].level_cmd;
    assert!(cmd2 < 1e-9, "末段 shutdown_controls 清零生效，实际 {cmd2}");
    assert_eq!(wf.fires("hp"), 1, "前缀计数不变（幂等）");
    assert_eq!(wf.fires("he"), 1, "末段首次激活计数 1 次");
}

#[test]
fn no_halt_workflow_has_none_action() {
    // 无 halt 的自由程序照常运行（零回归）：halt_action 恒 None。
    let asm = launch_asm("hold");
    let ir = from_toml_str(MIN_TARGET, None).unwrap();
    let mut wf = GraphWorkFlow::try_new(ir, &asm, 0.0).unwrap();
    wf.ignite();
    let mut asm = asm;
    wf.tick(&mut asm, &orbitx_protocol::mem::Slice::default(), 0.05);
    assert!(wf.halt_action().is_none());
    let cmd = asm.vessels[0].thrusters[0].level_cmd;
    assert!(cmd > 0.9);
}

// ---- v13 口域统一（Target 飞控口放开直驱；Super 飞控口放开 TargetMode）----

#[test]
fn target_direct_drives_execute() {
    // Target 图飞控口下 throttle/tvc/rcs 直驱合法（v13 口域统一）且执行生效：
    // 执行器 exec_drive 双语义本就统一，校验层不再是闸门。
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
fn reject_separate_on_flight_both_kinds() {
    // 口域统一后唯一保留的飞控口拒绝臂：分离器仍限 sep 口（两 kind 同一臂）。
    for kind in ["target", "super"] {
        let s = format!(
            r#"
schema = 1
kind = "{kind}"
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
"#
        );
        let err = from_toml_str(&s, None).unwrap_err();
        assert!(
            err.to_string().contains("飞控口不能挂分离器"),
            "kind={kind}：{err}"
        );
    }
}

#[test]
fn super_prefix_mode_drive_end_to_end() {
    // seed 链守护（任务二）：wf_super 前缀 = wf_target 快照原样复制，含 mode
    // 驱动——v13 前校验拒「Super 飞控口不能挂 TargetMode」，发射时 parse 失败。
    // 端到端：parse + try_new + 前缀段 tick（mode 驱动执行）+ 边界切换（C5 起
    // 走真实图执行流——hp 边界锚点切换，见 halt_multi_activation_refreshes_latch）。
    let s = HALT_SUPER_TWO.replace(
        "id = \"d1\"\ntype = \"drive\"\naction = \"throttle\"\ngroup = \"hold\"\nlevel = 1.0",
        "id = \"d1\"\ntype = \"drive\"\nmode = \"prograde_hold\"\nthrottle = 1.0",
    );
    let asm = launch_asm("hold");
    let ir = from_toml_str(&s, None).unwrap_or_else(|e| panic!("seed 前缀 mode 驱动应合法：{e}"));
    let mut wf = GraphWorkFlow::try_new(ir, &asm, 0.0).unwrap();
    wf.ignite();
    let mut asm = asm;
    // 任务一段：d1 mode 驱动执行（prograde_hold throttle 1.0 → 油门满）；hp
    // 边界锚点当拍切换（不锁存、不清零），m2 激活。
    wf.tick(&mut asm, &orbitx_protocol::mem::Slice::default(), 0.05);
    assert_eq!(wf.fires("hp"), 1, "前缀 halt 当拍边界切换");
    assert!(wf.halt_action().is_none(), "边界锚点不锁存");
    let cmd1 = asm.vessels[0].thrusters[0].level_cmd;
    assert!(cmd1 > 0.9, "前缀 mode 驱动执行（当拍快照），实际 {cmd1}");
    assert_eq!(wf.main_state("m2"), Some(MainState::Active));
    // 任务二段：d2 直驱 + he（缺省全真）末段锁存 → 清零。
    wf.tick(&mut asm, &orbitx_protocol::mem::Slice::default(), 0.05);
    let act2 = wf.halt_action().expect("末段激活锁存");
    assert!(
        act2.shutdown_controls && act2.stop_stepping && act2.save_recorder,
        "末段取值：{act2:?}"
    );
    assert_eq!(wf.fires("he"), 1, "末段首次激活计数 1 次");
}

#[test]
fn guidance_conditions_unlimited_nested_chain_ok() {
    // 2026-11-15 去规则化：撤销 v13「飞控口引导条件 ≤1」静态校验——条件每拍
    // 只走一支（visit 按当拍真值选口），嵌套 else-if 分段引导同拍并发上界 1，
    // 运行时无互抢；分段发射程序（实编 wf_target 形态）曾被误拒，现应放行。
    let chain = r#"
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
terms = [{ kind = "time", op = "lt", value = 1.0 }]
[[nodes]]
id = "d1"
type = "drive"
mode = "nose_to"
throttle = 1.0
azimuth = 0.0
elevation = 1.0
[[nodes]]
id = "c2"
type = "condition"
combine = "and"
terms = [{ kind = "time", op = "gte", value = 0.0 }]
[[nodes]]
id = "d2"
type = "drive"
mode = "vertical_hold"
throttle = 0.3
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
to = "d1"
[[edges]]
from = "c1"
from_port = "false"
to = "c2"
[[edges]]
from = "c2"
from_port = "true"
to = "d2"
"#;
    let asm = launch_asm("hold");
    let ir = from_toml_str(chain, None).unwrap_or_else(|e| panic!("嵌套分段引导链应放行：{e}"));
    let mut wf = GraphWorkFlow::try_new(ir, &asm, 0.0).unwrap();
    wf.ignite();
    let mut asm = asm;
    // 首拍：c1 真（time 0 < 1）→ d1 nose_to；c2 未被 visit（挂 c1 假分支）。
    wf.tick(&mut asm, &orbitx_protocol::mem::Slice::default(), 0.05);
    assert_eq!(wf.fires("d1"), 1, "前段引导发令");
    assert_eq!(wf.fires("d2"), 0, "后段未接管");
    // 30 拍（1.5 s > c1 阈值 1.0）：c1 转假 → c2 真 → d2 vertical_hold 接管，
    // d1 本拍不再 visit——互斥链每拍恰好一支引导发令。
    for _ in 0..29 {
        wf.tick(&mut asm, &orbitx_protocol::mem::Slice::default(), 0.05);
    }
    assert!(wf.fires("d2") >= 1, "后段引导接管");
    assert!(
        (wf.throttle_cmd() - 0.3).abs() < 1e-9,
        "后段模式生效，throttle_cmd={}",
        wf.throttle_cmd()
    );
    assert_eq!(
        wf.target_mode_sets_last_tick(),
        1,
        "互斥链每拍至多一个引导发令"
    );
}

#[test]
fn parallel_guidance_leaves_last_wins_with_warn() {
    // 并挂双引导条件（同拍都真）不拒：运行时 set_mode 互覆，同拍仅**遍历序
    // 最后者生效**（先 visit 叶当拍被覆盖，为死代码）；本拍第 2 次引导发令
    // 打 WARN（target_mode_sets==2，每拍至多一条）——去规则化后的排障
    // 可观察性（WORKFLOW.md 同拍混用写序注记）。
    let two_mode = r#"
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
id = "d1"
type = "drive"
mode = "nose_to"
throttle = 1.0
azimuth = 0.0
elevation = 1.0
[[nodes]]
id = "c2"
type = "condition"
combine = "and"
terms = [{ kind = "time", op = "gt", value = 0.0 }]
[[nodes]]
id = "d2"
type = "drive"
mode = "vertical_hold"
throttle = 0.3
[[edges]]
from = "ign0"
to = "m1"
[[edges]]
from = "m1"
from_port = "flight"
to = "c1"
[[edges]]
from = "m1"
from_port = "flight"
to = "c2"
[[edges]]
from = "c1"
from_port = "true"
to = "d1"
[[edges]]
from = "c2"
from_port = "true"
to = "d2"
"#;
    let asm = launch_asm("hold");
    let ir = from_toml_str(two_mode, None).unwrap_or_else(|e| panic!("并挂双引导条件应放行：{e}"));
    let mut wf = GraphWorkFlow::try_new(ir, &asm, 0.0).unwrap();
    wf.ignite();
    let mut asm = asm;
    // 首拍 time=0 走 false（先例 condition_true_false_ports），次拍两条件同真：
    // children 边序 c1 先 c2 后 → d1 先 visit、d2 后 visit。
    wf.tick(&mut asm, &orbitx_protocol::mem::Slice::default(), 0.05);
    wf.tick(&mut asm, &orbitx_protocol::mem::Slice::default(), 0.05);
    assert_eq!(
        wf.target_mode_sets_last_tick(),
        2,
        "同拍双引导发令计数（WARN 触发）"
    );
    assert_eq!(wf.fires("d1"), 1, "先 visit 叶发令（当拍被覆盖）");
    assert_eq!(wf.fires("d2"), 1, "后 visit 叶发令");
    assert!(
        (wf.throttle_cmd() - 0.3).abs() < 1e-9,
        "遍历序最后者（d2 vertical_hold 0.3）胜出，throttle_cmd={}",
        wf.throttle_cmd()
    );
}

#[test]
fn mode_condition_coexists_with_direct_drive() {
    // 直驱条件与引导条件共存合法（无额度概念）：mode 驱动执行、恒假直驱
    // 条件不发令（原 v13 测试 b 语义保留）。
    let mixed = r#"
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
id = "d1"
type = "drive"
mode = "vertical_hold"
throttle = 1.0
[[nodes]]
id = "c2"
type = "condition"
combine = "and"
terms = [{ kind = "time", op = "gt", value = 1000000000.0 }]
[[nodes]]
id = "d2"
type = "drive"
action = "throttle"
group = "hold"
level = 0.4
[[edges]]
from = "ign0"
to = "m1"
[[edges]]
from = "m1"
from_port = "flight"
to = "c1"
[[edges]]
from = "m1"
from_port = "flight"
to = "c2"
[[edges]]
from = "c1"
from_port = "true"
to = "d1"
[[edges]]
from = "c2"
from_port = "true"
to = "d2"
"#;
    let asm = launch_asm("hold");
    let ir = from_toml_str(&mixed, None).unwrap_or_else(|e| panic!("mode+直驱条件应共存：{e}"));
    let mut wf = GraphWorkFlow::try_new(ir, &asm, 0.0).unwrap();
    wf.ignite();
    let mut asm = asm;
    // 首拍 time=0 走 false（先例 condition_true_false_ports），次拍累计后走 true。
    wf.tick(&mut asm, &orbitx_protocol::mem::Slice::default(), 0.05);
    wf.tick(&mut asm, &orbitx_protocol::mem::Slice::default(), 0.05);
    assert_eq!(wf.fires("d1"), 1, "mode 条件真分支发令");
    assert_eq!(wf.fires("d2"), 0, "直驱条件恒假不发令");
    assert_eq!(wf.target_mode_sets_last_tick(), 1, "仅引导发令计数");
    let cmd = asm.vessels[0].thrusters[0].level_cmd;
    assert!(cmd > 0.9, "mode 驱动执行不被恒假直驱分支覆盖，实际 {cmd}");
}

#[test]
fn target_throttle_level_range_still_enforced() {
    // 口域统一 ≠ 放弃数值校验：Target 图 throttle level 越界仍拒（节点循环
    // 内 kind 无关校验，守护「放开不等于不校验」）。
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
level = 1.5
[[edges]]
from = "ign0"
to = "m1"
[[edges]]
from = "m1"
from_port = "flight"
to = "d1"
"#;
    let err = from_toml_str(s, None).unwrap_err();
    assert!(err.to_string().contains("level"), "{err}");
}

#[test]
fn mode_drive_on_actuatorless_stage() {
    // 级口投影统一（v13）：flight 口两 kind 恒露出——seed 前缀 mode 驱动可能
    // 挂在无执行器级（纯分离/滑行级），投影不得因 kind 收窄，否则 seeded
    // wf_super 发射时误报「级 Coast 未露出 flight 口」。
    let coast = StageSpec {
        name: "Coast".into(),
        dry_mass: 5_000.0,
        thrusters: vec![],
        length: 10.0,
        radius: 1.5,
        ..Default::default()
    }
    .with_fuel(1_000.0);
    let earth_r = 6_371_000.0;
    let pos = Vec3::new(0.0, 0.0, earth_r + 20.0);
    let up = pos * (1.0 / pos.length());
    let ref_axis = Vec3::new(0.0, 1.0, 0.0);
    let bx = cross(up, ref_axis).unit();
    let bz = cross(bx, up).unit();
    let rot = Matrix3::new(bx.x, up.x, bz.x, bx.y, up.y, bz.y, bx.z, up.z, bz.z);
    let mut asm = Assembly::new(
        &[hold_spec("hold"), coast],
        StateVectors {
            pos,
            vel: Vec3::ZERO,
            omega: Vec3::ZERO,
            r: rot,
            q: Quat::from_matrix(rot),
        },
    );
    asm.planet_radius = earth_r;
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
stage = "Coast"
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
    // try_new 内部走 stage_ports_from_assembly 投影校验（ports=Some）。
    let ir = from_toml_str(s, None).unwrap();
    let mut wf = GraphWorkFlow::try_new(ir, &asm, 0.0)
        .unwrap_or_else(|e| panic!("无执行器级 mode 驱动应合法（flight 口恒露出）：{e}"));
    wf.ignite();
    wf.tick(&mut asm, &orbitx_protocol::mem::Slice::default(), 0.05);
    assert_eq!(wf.fires("d1"), 1, "无执行器级 mode 驱动照常发令");
}

// ---- C5 DLC 叠加合成（WORKFLOW.md §8.1 / §9.1；P1）----

/// 基础图（target 口径，halt 出边 0 = 唯一末段；`entry_link.halt` 所指）。
const BASE_TOML: &str = r#"
schema = 1
kind = "target"
name = "base"

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

[[nodes]]
id = "hp"
type = "halt"

[[edges]]
from = "ign0"
to = "m1"

[[edges]]
from = "m1"
from_port = "flight"
to = "d1"

[[edges]]
from = "m1"
from_port = "flight"
to = "hp"
"#;

/// 叠加文件（super 口径，引用头 + 增量段；不含基础图任何节点）。
const ADDON_TOML: &str = r#"
schema = 1
kind = "super"
name = "addon"
base = "base.toml"

[entry_link]
halt = "hp"
to = "m2"

[[nodes]]
id = "m2"
type = "main"
stage = "hold"

[[nodes]]
id = "d2"
type = "drive"
action = "throttle"
group = "hold"
level = 1.0

[[nodes]]
id = "he"
type = "halt"

[[edges]]
from = "m2"
from_port = "flight"
to = "d2"

[[edges]]
from = "m2"
from_port = "flight"
to = "he"
"#;

fn base_ir() -> GraphIr {
    from_toml_str(BASE_TOML, None).unwrap()
}

fn addon_ir() -> GraphIr {
    // 叠加文件含 base/entry_link 键，from_toml_str 直解析即拒（防护）——
    // compose 测试用裸解析构造（合法消费路径 = load_composed 内部裸解析）。
    toml::from_str::<GraphIr>(ADDON_TOML).unwrap()
}

/// 裸解析叠加 TOML（compose 测试专用，同 addon_ir 注记）。
fn raw_addon(toml_text: &str) -> GraphIr {
    toml::from_str::<GraphIr>(toml_text).unwrap()
}

#[test]
fn compose_merges_and_expands_entry_link() {
    let composed = compose(base_ir(), addon_ir(), &EntryLink {
        halt: "hp".into(),
        to: "m2".into(),
    })
    .expect("合成");
    // 产出 kind=super（沿用叠加文件），base/entry_link 剥离。
    assert_eq!(composed.kind, WorkFlowKind::Super);
    assert!(composed.base.is_none());
    assert!(composed.entry_link.is_none());
    assert_eq!(composed.nodes.len(), 7, "基础 4 + 增量 3");
    // entry_link 展开为真实边界边（from_port = "out"）。
    assert!(composed.edges.iter().any(|e| {
        e.from == "hp" && e.to == "m2" && e.from_port == "out"
    }));
    // 基础与增量边均在（基础边不丢、增量边不丢）。
    assert!(composed.edges.iter().any(|e| e.from == "m1" && e.to == "d1"));
    assert!(composed.edges.iter().any(|e| e.from == "m2" && e.to == "he"));
    // 合成图通过整体校验（super 口径：end=he 唯一、prefix=hp 出边 1）。
    validate(&composed, None).expect("合成图整体校验");
}

#[test]
fn compose_rejects_base_super_and_addon_target() {
    let err = compose(addon_ir(), addon_ir(), &EntryLink {
        halt: "hp".into(),
        to: "m2".into(),
    })
    .unwrap_err();
    assert!(err.to_string().contains("kind 须为 target"), "{err}");
    let err2 = compose(base_ir(), base_ir(), &EntryLink {
        halt: "hp".into(),
        to: "m1".into(),
    })
    .unwrap_err();
    assert!(err2.to_string().contains("kind 须为 super"), "{err2}");
}

#[test]
fn compose_rejects_multi_halt_base() {
    // 基础图多 halt → 拒（多余 Halt = 真停机残留提前终局 + from 引用歧义）。
    let mut base = base_ir();
    base.nodes.push(GraphNode {
        id: "hx".into(),
        body: NodeBody::Halt {
            shutdown_controls: true,
            stop_stepping: true,
            save_recorder: true,
            title: None,
        },
        x: 0.0,
        y: 0.0,
    });
    base.edges.push(GraphEdge {
        from: "m1".into(),
        to: "hx".into(),
        from_port: "flight".into(),
    });
    let err = compose(base, addon_ir(), &EntryLink {
        halt: "hp".into(),
        to: "m2".into(),
    })
    .unwrap_err();
    assert!(err.to_string().contains("恰好 1 个停机节点"), "{err}");
}

#[test]
fn compose_rejects_entry_link_type_mismatch() {
    // from 非 halt（是 main）→ 拒。
    let err = compose(base_ir(), addon_ir(), &EntryLink {
        halt: "m1".into(),
        to: "m2".into(),
    })
    .unwrap_err();
    assert!(err.to_string().contains("须为停机节点"), "{err}");
    // to 非 main（是 drive）→ 拒。
    let err2 = compose(base_ir(), addon_ir(), &EntryLink {
        halt: "hp".into(),
        to: "d2".into(),
    })
    .unwrap_err();
    assert!(err2.to_string().contains("须为主节点"), "{err2}");
    // from 不存在 → 拒。
    let err3 = compose(base_ir(), addon_ir(), &EntryLink {
        halt: "nope".into(),
        to: "m2".into(),
    })
    .unwrap_err();
    assert!(err3.to_string().contains("不在基础图中"), "{err3}");
    // to 不存在 → 拒。
    let err4 = compose(base_ir(), addon_ir(), &EntryLink {
        halt: "hp".into(),
        to: "nope".into(),
    })
    .unwrap_err();
    assert!(err4.to_string().contains("不在叠加文件中"), "{err4}");
}

#[test]
fn compose_rejects_id_collision_and_cross_file_edge() {
    // 节点 id 跨文件冲突 → 拒。
    let addon_dup = ADDON_TOML.replace("id = \"m2\"\ntype = \"main\"\nstage = \"hold\"", "id = \"m1\"\ntype = \"main\"\nstage = \"hold\"");
    let addon_dup = addon_dup.replace("from = \"m2\"", "from = \"m1\"").replace("to = \"m2\"", "to = \"m1\"");
    let err = compose(base_ir(), raw_addon(&addon_dup), &EntryLink {
        halt: "hp".into(),
        to: "m1".into(),
    })
    .unwrap_err();
    assert!(err.to_string().contains("与基础图冲突"), "{err}");

    // 叠加段边引用基础图节点（d2 → m1）→ 拒（跨文件边仅 entry_link 合成产生）。
    let cross = ADDON_TOML.replace(
        "[[edges]]\nfrom = \"m2\"\nfrom_port = \"flight\"\nto = \"he\"",
        "[[edges]]\nfrom = \"m2\"\nfrom_port = \"flight\"\nto = \"he\"\n[[edges]]\nfrom = \"d2\"\nto = \"m1\"",
    );
    let err2 = compose(base_ir(), raw_addon(&cross), &EntryLink {
        halt: "hp".into(),
        to: "m2".into(),
    })
    .unwrap_err();
    assert!(err2.to_string().contains("引用了基础图节点"), "{err2}");
}

#[test]
fn from_toml_str_rejects_addon_keys() {
    // 直解析读到 base/entry_link → 拒（叠加文件须经 load_composed）。
    let err = from_toml_str(ADDON_TOML, None).unwrap_err();
    assert!(err.to_string().contains("load_composed"), "{err}");
}

fn temp_dir_unique(tag: &str) -> std::path::PathBuf {
    let dir = std::env::temp_dir().join(format!(
        "orbitx_c5_test_{}_{}_{}",
        tag,
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    ));
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

#[test]
fn load_composed_single_file_and_addon() {
    let dir = temp_dir_unique("compose_ok");
    std::fs::write(dir.join("base.toml"), BASE_TOML).unwrap();
    std::fs::write(dir.join("addon.toml"), ADDON_TOML).unwrap();

    // 单文件无 base → 直载（等价 from_toml_str 语义）。
    let direct = load_composed(&dir.join("base.toml"), None).expect("单文件直载");
    assert_eq!(direct.kind, WorkFlowKind::Target);

    // 叠加文件 → 同目录拼接合成。
    let composed = load_composed(&dir.join("addon.toml"), None).expect("合成加载");
    assert_eq!(composed.kind, WorkFlowKind::Super);
    assert!(composed.base.is_none());
    assert_eq!(composed.nodes.len(), 7);
    assert!(composed.edges.iter().any(|e| e.from == "hp" && e.to == "m2"));

    std::fs::remove_dir_all(&dir).ok();
}

#[test]
fn load_composed_error_paths_carry_file_names() {
    let dir = temp_dir_unique("compose_err");
    // 基础文件缺失 → 错误带两文件名。
    std::fs::write(dir.join("orphan.toml"), ADDON_TOML).unwrap();
    let err = load_composed(&dir.join("orphan.toml"), None).unwrap_err();
    assert!(
        err.to_string().contains("orphan.toml") && err.to_string().contains("base.toml"),
        "{err}"
    );

    // 有 base 缺 entry_link → 拒。
    let no_link = ADDON_TOML.replace("[entry_link]\nhalt = \"hp\"\nto = \"m2\"\n\n", "");
    std::fs::write(dir.join("base.toml"), BASE_TOML).unwrap();
    std::fs::write(dir.join("nolink.toml"), &no_link).unwrap();
    let err2 = load_composed(&dir.join("nolink.toml"), None).unwrap_err();
    assert!(err2.to_string().contains("缺 entry_link"), "{err2}");

    // 基础文件自身非法（schema ≠ 1）→ 错误定位到基础文件。
    let bad_base = BASE_TOML.replace("schema = 1", "schema = 2");
    std::fs::write(dir.join("badbase.toml"), &bad_base).unwrap();
    std::fs::write(dir.join("withbad.toml"), ADDON_TOML.replace("base = \"base.toml\"", "base = \"badbase.toml\"")).unwrap();
    let err3 = load_composed(&dir.join("withbad.toml"), None).unwrap_err();
    assert!(err3.to_string().contains("badbase.toml"), "{err3}");

    std::fs::remove_dir_all(&dir).ok();
}

#[test]
fn composed_boundary_switch_end_to_end() {
    // C5 端到端：引导段执行 → 边界锚点切换（零停机动作）→ 增量段直驱接管
    // → 末段仅停控制。三开关忽略断言：hp 三参数缺省全真（前缀参数行禁编辑），
    // 切换不锁存（halt_action None）= 未消费 → 未停步进 / 未存匣 / 不发 Halted。
    let composed = {
        let dir = temp_dir_unique("e2e");
        std::fs::write(dir.join("base.toml"), BASE_TOML).unwrap();
        std::fs::write(dir.join("addon.toml"), ADDON_TOML).unwrap();
        load_composed(&dir.join("addon.toml"), None).expect("合成")
    };
    let asm = launch_asm("hold");
    let mut wf = GraphWorkFlow::try_new(composed, &asm, 0.0).expect("try_new");
    assert!(wf.halt_action().is_none(), "未点火无锁存");
    wf.ignite();
    assert_eq!(wf.main_state("m1"), Some(MainState::Active));

    let mut asm = asm;
    // 首拍：m1 子树执行——d1 mode 驱动先 visit（快照口径，当拍仍发令），
    // hp 边界锚点后 visit：fires=1、不锁存、m1 失效、m2 激活。
    wf.tick(&mut asm, &orbitx_protocol::mem::Slice::default(), 0.05);
    assert_eq!(wf.fires("hp"), 1, "边界锚点激活计数");
    assert!(wf.halt_action().is_none(), "边界锚点零停机动作（三开关不消费）");
    assert_eq!(wf.fires("d1"), 1, "一拍延迟口径：当拍前缀叶执行完");
    assert_eq!(wf.fires("d2"), 0);
    assert_eq!(wf.main_state("m1"), Some(MainState::Invalid), "前缀失效");
    assert_eq!(wf.main_state("m2"), Some(MainState::Active), "增量段入口激活");
    let cmd1 = asm.vessels[0].thrusters[0].level_cmd;
    assert!(cmd1 > 0.9, "切换当拍不清零（滑行保持最后值），实际 {cmd1}");

    // 次拍：active 快照仅 m2——d2 直驱发令 + he 末段锁存全真 → tick 尾清零。
    wf.tick(&mut asm, &orbitx_protocol::mem::Slice::default(), 0.05);
    assert_eq!(wf.fires("d1"), 1, "前缀已失效不再发令");
    assert_eq!(wf.fires("d2"), 1, "增量段直驱接管");
    assert_eq!(wf.fires("hp"), 1, "边界锚点幂等不重复计数");
    assert_eq!(wf.fires("he"), 1, "末段停机首次激活");
    let act = wf.halt_action().expect("末段真停机锁存");
    assert!(act.shutdown_controls && act.stop_stepping && act.save_recorder);
    let cmd2 = asm.vessels[0].thrusters[0].level_cmd;
    assert!(cmd2 < 1e-9, "末段 shutdown_controls 清零生效，实际 {cmd2}");
}

#[test]
fn composed_reset_reruns_full_flow() {
    // 合成图 reset 接线：复位后重新点火全流程可重跑（边界锚点 / 末段锁存
    // 无旧状态泄漏）。
    let dir = temp_dir_unique("reset");
    std::fs::write(dir.join("base.toml"), BASE_TOML).unwrap();
    std::fs::write(dir.join("addon.toml"), ADDON_TOML).unwrap();
    let composed = load_composed(&dir.join("addon.toml"), None).unwrap();
    std::fs::remove_dir_all(&dir).ok();
    let asm = launch_asm("hold");
    let mut wf = GraphWorkFlow::try_new(composed, &asm, 0.0).unwrap();
    wf.ignite();
    let mut asm = asm;
    wf.tick(&mut asm, &orbitx_protocol::mem::Slice::default(), 0.05);
    wf.tick(&mut asm, &orbitx_protocol::mem::Slice::default(), 0.05);
    assert!(wf.halt_action().is_some());
    wf.reset_state();
    assert!(wf.halt_action().is_none());
    assert_eq!(wf.fires("hp"), 0);
    assert_eq!(wf.fires("he"), 0);
    assert_eq!(wf.main_state("m1"), Some(MainState::Idle));
    wf.ignite();
    assert_eq!(wf.main_state("m1"), Some(MainState::Active));
    wf.tick(&mut asm, &orbitx_protocol::mem::Slice::default(), 0.05);
    assert_eq!(wf.fires("hp"), 1, "重跑再次边界切换");
}
