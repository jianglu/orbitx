use super::*;
use crate::capability::ControlCapability;
use orbitx_math::{StateVectors, Vec3};
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

fn level(asm: &Assembly, idx: usize) -> f64 {
    asm.vessels[idx].thrusters.first().map(|t| t.level).unwrap_or(0.0)
}

#[test]
fn sync_primary_coaxial_lights_active_only() {
    let stages = coaxial_two_stage();
    let mut asm = Assembly::new(&stages, StateVectors::default());
    let caps = ControlCapability::for_primary(&asm);
    apply_throttle(&mut asm, &caps, ThrottlePolicy::SyncPrimary, 1.0);
    assert!((level(&asm, 0) - 1.0).abs() < 1e-9);
    assert!(level(&asm, 1).abs() < 1e-9);
}

#[test]
fn sync_primary_core_and_booster_not_upper() {
    let (stages, links) = core_upper_and_booster();
    let mut asm = Assembly::with_dock_links(&stages, StateVectors::default(), &links);
    let caps = ControlCapability::for_primary(&asm);
    apply_throttle(&mut asm, &caps, ThrottlePolicy::SyncPrimary, 1.0);
    assert!((level(&asm, 0) - 1.0).abs() < 1e-9);
    assert!(level(&asm, 1).abs() < 1e-9);
    assert!((level(&asm, 2) - 1.0).abs() < 1e-9);
}

#[test]
fn active_only_leaves_booster_idle() {
    let (stages, links) = core_upper_and_booster();
    let mut asm = Assembly::with_dock_links(&stages, StateVectors::default(), &links);
    let caps = ControlCapability::for_primary(&asm);
    apply_throttle(&mut asm, &caps, ThrottlePolicy::ActiveOnly, 1.0);
    assert!((level(&asm, 0) - 1.0).abs() < 1e-9);
    assert!(level(&asm, 2).abs() < 1e-9);
}

#[test]
fn sync_primary_after_booster_undock_upper_still_idle() {
    let (stages, links) = core_upper_and_booster();
    let mut asm = Assembly::with_dock_links(&stages, StateVectors::default(), &links);
    let id = asm.vessels[2].id;
    let sep = asm.vessels[2].separation_impulse;
    asm.undock(id, 0, sep);
    let caps = ControlCapability::for_primary(&asm);
    apply_throttle(&mut asm, &caps, ThrottlePolicy::SyncPrimary, 1.0);
    assert!((level(&asm, 0) - 1.0).abs() < 1e-9);
    assert!(level(&asm, 1).abs() < 1e-9);
}

#[test]
fn sync_primary_all_unlit_keeps_zero() {
    // 未点火（level=0）：SyncPrimary 应保持全部 0。
    let (stages, links) = core_upper_and_booster();
    let mut asm = Assembly::with_dock_links(&stages, StateVectors::default(), &links);
    let caps = ControlCapability::for_primary(&asm);
    apply_throttle(&mut asm, &caps, ThrottlePolicy::SyncPrimary, 0.0);
    assert!(level(&asm, 0).abs() < 1e-9);
    assert!(level(&asm, 2).abs() < 1e-9);
}

#[test]
fn detached_body_active_only_sets_detached_vessel() {
    let (stages, links) = core_upper_and_booster();
    let mut asm = Assembly::with_dock_links(&stages, StateVectors::default(), &links);
    let id = asm.vessels[2].id;
    let sep = asm.vessels[2].separation_impulse;
    asm.undock(id, 0, sep);
    let caps = ControlCapability::for_detached(&asm, 2);
    apply_throttle(&mut asm, &caps, ThrottlePolicy::ActiveOnly, 1.0);
    assert!((level(&asm, 2) - 1.0).abs() < 1e-9);
}
