use super::*;
use crate::capability::ControlCapability;
use crate::throttle::ThrottlePolicy;
use orbitx_math::{StateVectors, Vec3};
use orbitx_vessel::{Assembly, StageSpec};

fn coaxial() -> Vec<StageSpec> {
    vec![
        StageSpec::with_single_thruster("Core", 1000.0, 1000.0, 1000.0, 300.0,
            Vec3::new(0.0, -5.0, 0.0), Vec3::new(0.0, 1.0, 0.0), 10.0, 1.0, 1.0),
        StageSpec::with_single_thruster("Upper", 200.0, 500.0, 400.0, 300.0,
            Vec3::new(0.0, -2.0, 0.0), Vec3::new(0.0, 1.0, 0.0), 4.0, 1.0, 1.0),
    ]
}

#[test]
fn reads_active_vessel_attitude_and_omega() {
    let mut asm = Assembly::new(&coaxial(), StateVectors::default());
    let w = Vec3::new(0.1, 0.0, -0.05);
    asm.vessels[asm.active].state.omega = w;
    let caps = ControlCapability::for_primary(&asm);
    let base = BaseController::new(&mut asm, &caps);
    assert_eq!(base.omega(), w);
    // 垂直默认态：tip≈0。
    assert!(base.tip_angle() < 1e-6);
    assert!(base.pitch_yaw_angles().0.abs() < 1e-6);
}

#[test]
fn reads_primary_composite_kinematics_and_mass() {
    let asm = Assembly::new(&orbitx_vessel::presets::falcon9(), StateVectors::default());
    let mut asm2 = Assembly::new(&orbitx_vessel::presets::falcon9(), StateVectors::default());
    let caps2 = ControlCapability::for_primary(&asm2);
    let base = BaseController::new(&mut asm2, &caps2);
    // 主组合体质量 = total_mass。
    assert!((base.total_mass() - asm.total_mass()).abs() < 1e-6);
    assert!((base.fuel_mass() - asm.total_fuel()).abs() < 1e-6);
    assert_eq!(base.velocity(), asm.state.vel);
    assert_eq!(base.position(), asm.state.pos);
}

#[test]
fn reads_detached_body_single_vessel() {
    let mut asm = Assembly::new(&coaxial(), StateVectors::default());
    // 构造 detached：标记 Upper(1) detached。
    asm.vessels[1].detached = true;
    let mut asm2 = Assembly::new(&coaxial(), StateVectors::default());
    asm2.vessels[1].detached = true;
    let caps2 = ControlCapability::for_detached(&asm2, 1);
    let base = BaseController::new(&mut asm2, &caps2);
    // detached 单船质量 = dry + fuel。
    let v = &asm.vessels[1];
    assert!((base.total_mass() - v.mass()).abs() < 1e-9);
    assert!((base.fuel_mass() - v.fuel_mass).abs() < 1e-9);
    assert_eq!(base.omega(), v.state.omega);
}

#[test]
fn set_throttle_delegates_to_executor() {
    let mut asm = Assembly::new(&coaxial(), StateVectors::default());
    let caps = ControlCapability::for_primary(&asm);
    {
        let mut base = BaseController::new(&mut asm, &caps);
        base.set_throttle(ThrottlePolicy::SyncPrimary, 1.0);
    }
    let lvl = asm.vessels[0].thrusters.first().map(|t| t.level).unwrap_or(0.0);
    assert!((lvl - 1.0).abs() < 1e-9);
}

#[test]
fn separate_delegates_and_returns_detached() {
    use orbitx_vessel::DockPort;
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
    let mut asm = Assembly::with_dock_links(&[core, upper, booster], StateVectors::default(),
        &[(0, 1, 1, 0), (0, 2, 2, 0)]);
    let caps = ControlCapability::for_primary(&asm);
    let left = {
        let mut base = BaseController::new(&mut asm, &caps);
        base.separate("Booster-sep-0")
    };
    assert_eq!(left, vec![2]);
    assert!(asm.vessels[2].detached);
}

#[test]
fn caps_accessor_returns_reference() {
    let mut asm2 = Assembly::new(&coaxial(), StateVectors::default());
    let caps2 = ControlCapability::for_primary(&asm2);
    let base = BaseController::new(&mut asm2, &caps2);
    assert_eq!(base.caps().body, crate::capability::BodyRef::Primary);
}
