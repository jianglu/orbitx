use super::*;
use crate::stage::StageSpec;
use orbitx_math::{cross, Matrix3, Quat, StateVectors};

fn make_vessel_with_rcs() -> Vessel {
    let mut v = Vessel::from_spec(
        0,
        &StageSpec {
            name: "test",
            dry_mass: 5000.0,
            fuel_mass: 5000.0,
            thrust: 0.0,
            isp: 0.0,
            engine_dir: Vec3::ZERO,
            engine_pos: Vec3::ZERO,
            length: 10.0,
            radius: 1.0,
            separation_impulse: 0.0,
            pmi: Vec3::new(-1.0, -1.0, -1.0),
            max_gimbal: 0.0,
            max_gimbal_rate: 0.0,
            gimbal_axis: Vec3::new(1.0, 0.0, 0.0),
            ..Default::default()
        },
        StateVectors {
            pos: Vec3::new(0.0, 0.0, 6_371_000.0),
            vel: Vec3::ZERO,
            omega: Vec3::ZERO,
            r: Matrix3::IDENTITY,
            q: Quat::IDENTITY,
        },
    );
    add_default_rcs(&mut v, 5.0, 10_000.0);
    v
}

#[test]
fn default_rcs_layout_12_thrusters() {
    let v = make_vessel_with_rcs();
    assert_eq!(v.thrusters.len(), 12, "should have 12 RCS thrusters");
    assert_eq!(v.thruster_groups.len(), 12, "should have 12 thruster groups");
}

#[test]
fn rcs_pitch_up_produces_torque() {
    let v = make_vessel_with_rcs();
    let group = v.thruster_groups.iter()
        .find(|g| g.group_type == ThrusterGroupType::AttPitchUp).unwrap();
    let idx = group.thruster_indices[0];
    let t = &v.thrusters[idx];
    let f = t.base_dir * t.max_thrust;
    let tau = cross(f, t.pos);
    assert!(tau.x > 0.0, "pitch up should produce +X torque: {:?}", tau);
}

#[test]
fn rcs_yaw_produces_torque() {
    let v = make_vessel_with_rcs();
    let group = v.thruster_groups.iter()
        .find(|g| g.group_type == ThrusterGroupType::AttYawLeft).unwrap();
    let idx = group.thruster_indices[0];
    let t = &v.thrusters[idx];
    let f = t.base_dir * t.max_thrust;
    let tau = cross(f, t.pos);
    assert!(tau.length() > 1e-3, "yaw thruster should produce torque: {:?}", tau);
}

#[test]
fn rcs_translation_produces_no_torque() {
    let v = make_vessel_with_rcs();
    for gt in [
        ThrusterGroupType::AttRight,
        ThrusterGroupType::AttUp,
        ThrusterGroupType::AttForward,
    ] {
        let group = v.thruster_groups.iter().find(|g| g.group_type == gt).unwrap();
        let idx = group.thruster_indices[0];
        let t = &v.thrusters[idx];
        let f = t.base_dir * t.max_thrust;
        let tau = cross(f, t.pos);
        assert!(tau.length() < 1e-9, "translation group {:?} should not produce torque: {:?}", gt, tau);
    }
}

#[test]
fn group_level_clamps_0_to_1() {
    let mut v = make_vessel_with_rcs();
    set_group_level(&mut v, ThrusterGroupType::AttPitchUp, 2.0);
    let level = get_group_level(&v, ThrusterGroupType::AttPitchUp);
    assert!(level <= 1.0, "level should clamp to 1.0: {level}");
    set_group_level(&mut v, ThrusterGroupType::AttPitchUp, -1.0);
    let level = get_group_level(&v, ThrusterGroupType::AttPitchUp);
    assert!(level >= 0.0, "level should clamp to 0.0: {level}");
}

#[test]
fn set_attitude_rot_pitch() {
    let mut v = make_vessel_with_rcs();
    set_attitude_rot(&mut v, RotAxis::Pitch, 0.5);
    assert!((get_group_level(&v, ThrusterGroupType::AttPitchUp) - 0.5).abs() < 1e-10);
    assert!(get_group_level(&v, ThrusterGroupType::AttPitchDown).abs() < 1e-10);
    set_attitude_rot(&mut v, RotAxis::Pitch, -0.3);
    assert!(get_group_level(&v, ThrusterGroupType::AttPitchUp).abs() < 1e-10);
    assert!((get_group_level(&v, ThrusterGroupType::AttPitchDown) - 0.3).abs() < 1e-10);
}

#[test]
fn set_attitude_lin_y() {
    let mut v = make_vessel_with_rcs();
    set_attitude_lin(&mut v, LinAxis::Y, 0.8);
    assert!((get_group_level(&v, ThrusterGroupType::AttUp) - 0.8).abs() < 1e-10);
    assert!(get_group_level(&v, ThrusterGroupType::AttDown).abs() < 1e-10);
}
