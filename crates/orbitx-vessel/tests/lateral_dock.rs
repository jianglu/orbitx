//! 侧挂 Dock / SuperVessel 子集集成测试（CZ-2F 类）。

use orbitx_math::{StateVectors, Vec3};
use orbitx_vessel::{Assembly, DockPort, StageSpec};


fn core_and_booster() -> (Vec<StageSpec>, Vec<(usize, usize, usize, usize)>) {
    let core = StageSpec {
        name: "Core",
        dry_mass: 1000.0,
        fuel_mass: 1000.0,
        thrust: 0.0,
        isp: 300.0,
        engine_dir: Vec3::new(0.0, 1.0, 0.0),
        engine_pos: Vec3::new(0.0, -5.0, 0.0),
        length: 10.0,
        radius: 1.0,
        separation_impulse: 1.0,
        docks: Some(vec![
            DockPort::with_rot(
                Vec3::new(0.0, -5.0, 0.0),
                Vec3::new(0.0, -1.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
            ),
            DockPort::with_rot(
                Vec3::new(0.0, 5.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
            ),
            DockPort::with_rot(
                Vec3::new(2.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
            ),
        ]),
        ..Default::default()
    };
    let booster = StageSpec {
        name: "Booster",
        dry_mass: 500.0,
        fuel_mass: 500.0,
        thrust: 2000.0,
        isp: 300.0,
        engine_dir: Vec3::new(0.0, 1.0, 0.0),
        engine_pos: Vec3::new(0.0, -4.0, 0.0),
        length: 8.0,
        radius: 0.5,
        separation_impulse: 2.0,
        docks: Some(vec![DockPort::with_rot(
            Vec3::new(-0.5, 0.0, 0.0),
            Vec3::new(-1.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        )]),
        ..Default::default()
    };
    (vec![core, booster], vec![(0, 2, 1, 0)])
}

#[test]
fn dock_merges_mass_and_layout() {
    let (stages, links) = core_and_booster();
    let asm = Assembly::with_dock_links(&stages, StateVectors::default(), &links);
    assert_eq!(asm.components.len(), 2);
    assert!((asm.total_mass() - 3000.0).abs() < 0.1);
    let booster = asm
        .components
        .iter()
        .find(|c| c.vessel_index == 1)
        .unwrap();
    assert!(
        booster.rpos.x > 2.0,
        "助推应在 +X: {:?}",
        booster.rpos
    );
}

#[test]
fn lateral_thrust_makes_torque() {
    let (stages, links) = core_and_booster();
    let mut asm = Assembly::with_dock_links(&stages, StateVectors::default(), &links);
    // 仅助推点火。
    asm.vessels[1].set_throttle(1.0);
    asm.vessels[0].set_throttle(0.0);
    let omega0 = asm.state.omega;
    for _ in 0..20 {
        asm.step(0.05, &[]);
    }
    assert!(
        (asm.state.omega - omega0).length() > 1e-6,
        "偏心推力应产生角速度: ω={:?}",
        asm.state.omega
    );
}

#[test]
fn undock_booster_preserves_stage_count() {
    let (stages, links) = core_and_booster();
    let mut asm = Assembly::with_dock_links(&stages, StateVectors::default(), &links);
    assert_eq!(asm.stage_count(), 2);
    let core_id = asm.vessels[0].id;
    let leave = asm.undock(core_id, 2, 2.0);
    assert_eq!(leave, vec![1]);
    assert!(asm.vessels[1].detached);
    assert_eq!(asm.stage_count(), 1);
    assert_eq!(asm.components.len(), 1);
}

#[test]
fn four_boosters_like_cz2f() {
    // 芯级 + 四侧挂，不经 config 解析（配置测在 orbitx-config）。
    let mut core = StageSpec {
        name: "Core",
        dry_mass: 1000.0,
        fuel_mass: 1000.0,
        length: 10.0,
        radius: 1.0,
        docks: Some(vec![
            DockPort::with_rot(
                Vec3::new(0.0, -5.0, 0.0),
                Vec3::new(0.0, -1.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
            ),
            DockPort::with_rot(
                Vec3::new(0.0, 5.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
            ),
            DockPort::with_rot(
                Vec3::new(2.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
            ),
            DockPort::with_rot(
                Vec3::new(-2.0, 0.0, 0.0),
                Vec3::new(-1.0, 0.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
            ),
            DockPort::with_rot(
                Vec3::new(0.0, 0.0, 2.0),
                Vec3::new(0.0, 0.0, 1.0),
                Vec3::new(0.0, 1.0, 0.0),
            ),
            DockPort::with_rot(
                Vec3::new(0.0, 0.0, -2.0),
                Vec3::new(0.0, 0.0, -1.0),
                Vec3::new(0.0, 1.0, 0.0),
            ),
        ]),
        ..Default::default()
    };
    let _ = &mut core;
    let mk_b = |name: &'static str, pos: Vec3, dir: Vec3, rot: Vec3| StageSpec {
        name,
        dry_mass: 100.0,
        fuel_mass: 100.0,
        length: 5.0,
        radius: 0.5,
        docks: Some(vec![DockPort::with_rot(pos, dir, rot)]),
        ..Default::default()
    };
    let stages = vec![
        core,
        mk_b(
            "B0",
            Vec3::new(-0.5, 0.0, 0.0),
            Vec3::new(-1.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        ),
        mk_b(
            "B1",
            Vec3::new(0.5, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        ),
        mk_b(
            "B2",
            Vec3::new(0.0, 0.0, -0.5),
            Vec3::new(0.0, 0.0, -1.0),
            Vec3::new(0.0, 1.0, 0.0),
        ),
        mk_b(
            "B3",
            Vec3::new(0.0, 0.0, 0.5),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(0.0, 1.0, 0.0),
        ),
    ];
    let links = vec![
        (0, 2, 1, 0),
        (0, 3, 2, 0),
        (0, 4, 3, 0),
        (0, 5, 4, 0),
    ];
    let mut asm = Assembly::with_dock_links(&stages, StateVectors::default(), &links);
    assert_eq!(asm.components.len(), 5);
    assert!((asm.total_mass() - 2800.0).abs() < 0.1);
    let leave = asm.undock(asm.vessels[0].id, 2, 1.0);
    assert_eq!(leave, vec![1]);
    assert_eq!(asm.stage_count(), 4);
    assert_eq!(asm.components.len(), 4);
}
