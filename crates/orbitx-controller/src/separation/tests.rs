use super::*;
use crate::capability::ControlCapability;
use orbitx_math::{StateVectors, Vec3};
use orbitx_vessel::{Assembly, DockPort, StageSpec};

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
fn perform_separate_strap_on_leaf_undocks_booster() {
    let (stages, links) = core_upper_and_booster();
    let mut asm = Assembly::with_dock_links(&stages, StateVectors::default(), &links);
    let caps = ControlCapability::for_primary(&asm);
    let left = perform_separate(&mut asm, &caps, "Booster-sep-0");
    assert_eq!(left, vec![2]);
    assert!(asm.vessels[2].detached);
    assert!(!asm.vessels[0].detached);
}

#[test]
fn perform_separate_coaxial_stage() {
    let (stages, links) = core_upper_and_booster();
    let mut asm = Assembly::with_dock_links(&stages, StateVectors::default(), &links);
    // 先拆 booster，再同轴拆芯。
    let caps = ControlCapability::for_primary(&asm);
    perform_separate(&mut asm, &caps, "Booster-sep-0");
    let caps = ControlCapability::for_primary(&asm);
    let left = perform_separate(&mut asm, &caps, "stage-sep");
    // separate_stage 返回 [active]（新的活动级）。
    assert_eq!(left, vec![asm.active]);
    assert!(asm.vessels[0].detached);
}

#[test]
fn perform_separate_unknown_id_returns_empty() {
    let (stages, links) = core_upper_and_booster();
    let mut asm = Assembly::with_dock_links(&stages, StateVectors::default(), &links);
    let caps = ControlCapability::for_primary(&asm);
    let left = perform_separate(&mut asm, &caps, "no-such-point");
    assert!(left.is_empty());
}

#[test]
fn should_auto_separate_when_booster_empty() {
    let (stages, links) = core_upper_and_booster();
    let mut asm = Assembly::with_dock_links(&stages, StateVectors::default(), &links);
    asm.vessels[2].fuel_mass = 0.0;
    assert!(should_auto_separate(&asm));
}

#[test]
fn should_not_auto_separate_when_fueled() {
    let (stages, links) = core_upper_and_booster();
    let asm = Assembly::with_dock_links(&stages, StateVectors::default(), &links);
    assert!(!should_auto_separate(&asm));
}

#[test]
fn should_not_auto_separate_single_stage() {
    let spec = StageSpec::with_single_thruster("solo", 100.0, 100.0, 1000.0, 300.0,
        Vec3::new(0.0, -1.0, 0.0), Vec3::new(0.0, 1.0, 0.0), 2.0, 1.0, 0.0);
    let asm = Assembly::new(&[spec], StateVectors::default());
    assert!(!should_auto_separate(&asm));
}
