use super::*;
use crate::capability::ControlCapability;
use orbitx_math::{StateVectors, Vec3};
use orbitx_vessel::{add_default_rcs, Assembly, RotAxis, StageSpec};

fn rcs_vessel() -> Vec<StageSpec> {
    vec![StageSpec {
        name: "rcs-test",
        dry_mass: 5000.0,
        fuel_mass: 5000.0,
        thrusters: vec![],
        length: 10.0,
        radius: 1.0,
        separation_impulse: 0.0,
        pmi: Vec3::new(-1.0, -1.0, -1.0),
        ..Default::default()
    }]
}

#[test]
fn set_rcs_delegates_to_attitude_rot() {
    let spec = rcs_vessel();
    let mut asm = Assembly::new(&spec, StateVectors {
        pos: Vec3::new(0.0, 0.0, 6_371_000.0),
        vel: Vec3::ZERO, omega: Vec3::ZERO,
        r: orbitx_math::Matrix3::IDENTITY, q: orbitx_math::Quat::IDENTITY,
    });
    add_default_rcs(&mut asm.vessels[0], 5.0, 10_000.0);
    let caps = ControlCapability::for_primary(&asm);
    // 找到 pitch_up 组 id。
    let pitch_id = caps
        .rcs_groups
        .iter()
        .find(|g| g.group_type == orbitx_vessel::rcs::ThrusterGroupType::AttPitchUp)
        .map(|g| g.id.clone())
        .unwrap();
    set_rcs(&mut asm, &caps, &pitch_id, RotAxis::Pitch, 1.0);
    // 正 level → AttPitchUp 组推进器 level=1。
    let g = asm.vessels[0]
        .thruster_groups
        .iter()
        .find(|g| g.group_type == orbitx_vessel::rcs::ThrusterGroupType::AttPitchUp)
        .unwrap();
    let lvl = asm.vessels[0].thrusters[g.thruster_indices[0]].level;
    assert!((lvl - 1.0).abs() < 1e-9);
}

#[test]
fn set_rcs_unknown_group_id_no_op() {
    let spec = rcs_vessel();
    let mut asm = Assembly::new(&spec, StateVectors::default());
    add_default_rcs(&mut asm.vessels[0], 5.0, 10_000.0);
    let caps = ControlCapability::for_primary(&asm);
    // 未命中 group id：不应 panic，无副作用。
    set_rcs(&mut asm, &caps, "no-such-rcs", RotAxis::Yaw, 1.0);
    // 所有推进器 level 仍为 0。
    for t in &asm.vessels[0].thrusters {
        assert!(t.level.abs() < 1e-9);
    }
}
