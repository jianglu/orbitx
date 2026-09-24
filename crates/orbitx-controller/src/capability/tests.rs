use super::*;
use orbitx_math::StateVectors;
use orbitx_vessel::{Assembly, DockPort, StageSpec};

fn coaxial_two_stage() -> Vec<StageSpec> {
    vec![
        StageSpec::with_single_thruster("Core", 1000.0, 1000.0, 1000.0, 300.0,
            Vec3::new(0.0, -5.0, 0.0), Vec3::new(0.0, 1.0, 0.0), 10.0, 1.0, 1.0),
        StageSpec::with_single_thruster("Upper", 200.0, 500.0, 400.0, 300.0,
            Vec3::new(0.0, -2.0, 0.0), Vec3::new(0.0, 1.0, 0.0), 4.0, 1.0, 1.0),
    ]
}

fn core_upper_and_booster() -> (Vec<StageSpec>, Vec<(usize, usize, usize, usize)>) {
    let mut core = StageSpec::with_single_thruster("Core", 1000.0, 1000.0, 1000.0, 300.0,
        Vec3::new(0.0, -5.0, 0.0), Vec3::new(0.0, 1.0, 0.0), 10.0, 1.0, 1.0);
    core.docks = Some(vec![
        DockPort::with_rot(Vec3::new(0.0, -5.0, 0.0), Vec3::new(0.0, -1.0, 0.0), Vec3::new(0.0, 0.0, 1.0)),
        DockPort::with_rot(Vec3::new(0.0, 5.0, 0.0), Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, 0.0, 1.0)),
        DockPort::with_rot(Vec3::new(2.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0)),
    ]);
    let upper = StageSpec::with_single_thruster("Upper", 200.0, 500.0, 400.0, 300.0,
        Vec3::new(0.0, -2.0, 0.0), Vec3::new(0.0, 1.0, 0.0), 4.0, 1.0, 1.0);
    let mut booster = StageSpec::with_single_thruster("Booster", 500.0, 500.0, 2000.0, 300.0,
        Vec3::new(0.0, -4.0, 0.0), Vec3::new(0.0, 1.0, 0.0), 8.0, 0.5, 2.0);
    booster.docks = Some(vec![DockPort::with_rot(
        Vec3::new(-0.5, 0.0, 0.0), Vec3::new(-1.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0))]);
    (vec![core, upper, booster], vec![(0, 1, 1, 0), (0, 2, 2, 0)])
}

#[test]
fn for_primary_falcon9_throttle_groups() {
    let asm = Assembly::new(&orbitx_vessel::presets::falcon9(), StateVectors::default());
    let caps = ControlCapability::for_primary(&asm);
    assert_eq!(caps.body, BodyRef::Primary);
    let ids: Vec<&str> = caps.throttle_groups.iter().map(|g| g.id.as_str()).collect();
    assert_eq!(ids, vec!["F9-S1", "F9-S2"]);
    // F9-S1: 9 Merlin；F9-S2: 1 MVac。
    assert_eq!(caps.throttle_groups[0].thruster_indices.len(), 9);
    assert_eq!(caps.throttle_groups[1].thruster_indices.len(), 1);
    // slew_rate 来自 throttle_rate = 0.8。
    assert!((caps.throttle_groups[0].slew_rate - 0.8).abs() < 1e-9);
}

#[test]
fn for_primary_falcon9_tvc_groups() {
    let asm = Assembly::new(&orbitx_vessel::presets::falcon9(), StateVectors::default());
    let caps = ControlCapability::for_primary(&asm);
    let ids: Vec<&str> = caps.tvc_groups.iter().map(|g| g.id.as_str()).collect();
    assert_eq!(ids, vec!["F9-S1-tvc", "F9-S2-tvc"]);
    assert_eq!(caps.tvc_groups[0].thrusters.len(), 9);
    assert!((caps.tvc_groups[0].max_angle - 0.122).abs() < 1e-9);
}

#[test]
fn for_primary_falcon9_separation_stage_only() {
    let asm = Assembly::new(&orbitx_vessel::presets::falcon9(), StateVectors::default());
    let caps = ControlCapability::for_primary(&asm);
    // 同轴三级，无侧挂叶 → 仅 stage-sep。
    assert_eq!(caps.separation_points.len(), 1);
    assert_eq!(caps.separation_points[0].id, "stage-sep");
    assert_eq!(caps.separation_points[0].kind, SeparationKind::CoaxialStage);
    // 无 RCS 组（预设未加 add_default_rcs）。
    assert!(caps.rcs_groups.is_empty());
}

#[test]
fn for_primary_saturn_v_three_throttle_groups() {
    let asm = Assembly::new(&orbitx_vessel::presets::saturn_v(), StateVectors::default());
    let caps = ControlCapability::for_primary(&asm);
    let ids: Vec<&str> = caps.throttle_groups.iter().map(|g| g.id.as_str()).collect();
    assert_eq!(ids, vec!["S-IC", "S-II", "S-IVB"]);
    // CSM-LM 无主推 → 不在 throttle_groups。
    assert_eq!(caps.throttle_groups.len(), 3);
}

#[test]
fn for_primary_core_upper_booster_has_strap_on_separation() {
    let (stages, links) = core_upper_and_booster();
    let asm = Assembly::with_dock_links(&stages, StateVectors::default(), &links);
    let caps = ControlCapability::for_primary(&asm);
    // 油门组：Core/Upper/Booster。
    let ids: Vec<&str> = caps.throttle_groups.iter().map(|g| g.id.as_str()).collect();
    assert_eq!(ids, vec!["Core", "Upper", "Booster"]);
    // 分离点：Booster 侧挂叶 + stage-sep。
    assert!(caps.separation_points.iter().any(|s| s.id == "Booster-sep-0"
        && s.kind == SeparationKind::StrapOnLeaf { vessel: 2, port: 0 }));
    assert!(caps.separation_points.iter().any(|s| s.id == "stage-sep"));
    assert_eq!(caps.separation_points.len(), 2);
}

#[test]
fn for_primary_single_stage_no_separation() {
    let spec = StageSpec::with_single_thruster("solo", 100.0, 100.0, 1000.0, 300.0,
        Vec3::new(0.0, -1.0, 0.0), Vec3::new(0.0, 1.0, 0.0), 2.0, 1.0, 0.0);
    let asm = Assembly::new(&[spec], StateVectors::default());
    let caps = ControlCapability::for_primary(&asm);
    assert!(caps.separation_points.is_empty());
    assert_eq!(caps.throttle_groups.len(), 1);
}

#[test]
fn for_detached_projects_booster_after_undock() {
    let (stages, links) = core_upper_and_booster();
    let mut asm = Assembly::with_dock_links(&stages, StateVectors::default(), &links);
    let id = asm.vessels[2].id;
    let sep = asm.vessels[2].separation_impulse;
    asm.undock(id, 0, sep);
    assert!(asm.vessels[2].detached);

    let caps = ControlCapability::for_detached(&asm, 2);
    assert_eq!(caps.body, BodyRef::Detached(2));
    assert_eq!(caps.throttle_groups.len(), 1);
    assert_eq!(caps.throttle_groups[0].id, "Booster");
    assert_eq!(caps.throttle_groups[0].vessel_index, 2);
    // detached 单 vessel 无分离点。
    assert!(caps.separation_points.is_empty());
}

#[test]
fn for_detached_invalid_index_returns_empty() {
    let asm = Assembly::new(&coaxial_two_stage(), StateVectors::default());
    let caps = ControlCapability::for_detached(&asm, 99);
    assert_eq!(caps.body, BodyRef::Detached(99));
    assert!(caps.throttle_groups.is_empty());
    assert!(caps.tvc_groups.is_empty());
    assert!(caps.dock_ports.is_empty());
}

#[test]
fn for_detached_payload_no_thrusters() {
    let asm = Assembly::new(&orbitx_vessel::presets::falcon9(), StateVectors::default());
    // Payload(2) 无主推。
    let caps = ControlCapability::for_detached(&asm, 2);
    assert!(caps.throttle_groups.is_empty());
    assert!(caps.tvc_groups.is_empty());
}

#[test]
fn for_primary_dock_port_ids_derived() {
    let asm = Assembly::new(&coaxial_two_stage(), StateVectors::default());
    let caps = ControlCapability::for_primary(&asm);
    // 至少存在 Core-dock-* 与 Upper-dock-* 形式 id。
    assert!(caps.dock_ports.iter().any(|d| d.id.starts_with("Core-dock-")));
    assert!(caps.dock_ports.iter().any(|d| d.id.starts_with("Upper-dock-")));
}
