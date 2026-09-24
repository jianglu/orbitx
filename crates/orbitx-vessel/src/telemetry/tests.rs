#[cfg(test)]
mod tests {
    use crate::telemetry::{
        AttitudeReadout, BodyReadout, KinematicsReadout, MassReadout, StageReadout, ThrustReadout,
    };
    use crate::{Assembly, DockPort, StageSpec};
    use orbitx_math::{StateVectors, Vec3};

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
    fn primary_thrusting_indices_coaxial() {
        let asm = Assembly::new(&coaxial_two_stage(), StateVectors::default());
        assert_eq!(asm.primary_thrusting_indices().collect::<Vec<_>>(), vec![0, 1]);
    }

    #[test]
    fn lit_thrusting_indices_coaxial_only_active() {
        let mut asm = Assembly::new(&coaxial_two_stage(), StateVectors::default());
        asm.set_throttle(1.0);
        assert_eq!(asm.lit_thrusting_indices().collect::<Vec<_>>(), vec![0]);
    }

    #[test]
    fn lit_thrusting_indices_core_and_booster() {
        let (stages, links) = core_upper_and_booster();
        let mut asm = Assembly::with_dock_links(&stages, StateVectors::default(), &links);
        asm.set_throttle(1.0);
        let mut got = asm.lit_thrusting_indices().collect::<Vec<_>>();
        got.sort();
        assert_eq!(got, vec![0, 2]);
    }

    #[test]
    fn strap_on_leaf_finds_booster() {
        let (stages, links) = core_upper_and_booster();
        let asm = Assembly::with_dock_links(&stages, StateVectors::default(), &links);
        assert_eq!(asm.strap_on_leaf_indices().collect::<Vec<_>>(), vec![(2, 0)]);
    }

    #[test]
    fn strap_on_leaf_empty_for_coaxial() {
        let asm = Assembly::new(&coaxial_two_stage(), StateVectors::default());
        assert!(asm.strap_on_leaf_indices().count() == 0);
    }

    #[test]
    fn pick_strap_on_leaf_prefers_booster() {
        let (stages, links) = core_upper_and_booster();
        let mut asm = Assembly::with_dock_links(&stages, StateVectors::default(), &links);
        assert_eq!(asm.pick_strap_on_leaf(), Some((2, 0)));
        asm.vessels[2].fuel_mass = 0.0;
        assert_eq!(asm.pick_strap_on_leaf(), Some((2, 0)));
    }

    #[test]
    fn pick_strap_on_leaf_none_when_no_strap() {
        let asm = Assembly::new(&coaxial_two_stage(), StateVectors::default());
        assert!(asm.pick_strap_on_leaf().is_none());
    }

    #[test]
    fn primary_thrust_sum_coaxial_active_only() {
        let mut asm = Assembly::new(&coaxial_two_stage(), StateVectors::default());
        asm.set_throttle(1.0);
        assert!((asm.primary_thrust_sum() - 1000.0).abs() < 1e-6);
    }

    #[test]
    fn primary_thrust_sum_core_and_booster() {
        let (stages, links) = core_upper_and_booster();
        let mut asm = Assembly::with_dock_links(&stages, StateVectors::default(), &links);
        // SyncPrimary 语义：active ∪ 侧挂叶均点火。set_throttle 仅设 active，故 Booster 须另设。
        asm.set_throttle(1.0);
        asm.vessels[2].set_throttle(1.0);
        assert!((asm.primary_thrust_sum() - 3000.0).abs() < 1e-6);
    }

    #[test]
    fn primary_thrust_sum_zero_when_unlit() {
        let asm = Assembly::new(&coaxial_two_stage(), StateVectors::default());
        assert!(asm.primary_thrust_sum().abs() < 1e-9);
    }

    #[test]
    fn lit_after_booster_undock() {
        let (stages, links) = core_upper_and_booster();
        let mut asm = Assembly::with_dock_links(&stages, StateVectors::default(), &links);
        asm.set_throttle(1.0);
        let id = asm.vessels[2].id;
        let sep = asm.vessels[2].separation_impulse;
        asm.undock(id, 0, sep);
        asm.set_throttle(1.0);
        assert_eq!(asm.lit_thrusting_indices().collect::<Vec<_>>(), vec![0]);
        assert!(asm.vessels[2].detached);
    }

    #[test]
    fn lit_after_core_separate_lights_upper() {
        let (stages, links) = core_upper_and_booster();
        let mut asm = Assembly::with_dock_links(&stages, StateVectors::default(), &links);
        let id = asm.vessels[2].id;
        let sep = asm.vessels[2].separation_impulse;
        asm.undock(id, 0, sep);
        asm.separate_stage();
        assert_eq!(asm.active, 1);
        asm.set_throttle(1.0);
        assert_eq!(asm.lit_thrusting_indices().collect::<Vec<_>>(), vec![1]);
        assert!((asm.primary_thrust_sum() - 400.0).abs() < 1e-6);
    }

    #[test]
    fn index_set_iterators_are_lazy() {
        let asm = Assembly::new(&coaxial_two_stage(), StateVectors::default());
        assert_eq!(asm.primary_thrusting_indices().next(), Some(0));
        // lit 集 = active ∪ 侧挂叶（与油门开度无关）；同轴无侧挂 → 仅 active。
        assert_eq!(asm.lit_thrusting_indices().count(), 1);
        assert_eq!(asm.strap_on_leaf_indices().count(), 0);
    }

    #[test]
    fn attitude_vertical_launch_zero_tip() {
        use orbitx_math::{cross, Matrix3, Quat};
        let pos = Vec3::new(0.0, 0.0, 6_371_000.0);
        let up = pos * (1.0 / pos.length());
        let bx = cross(up, Vec3::new(0.0, 1.0, 0.0)).unit();
        let bz = cross(bx, up).unit();
        let by = up;
        let rot = Matrix3::new(bx.x, by.x, bz.x, bx.y, by.y, bz.y, bx.z, by.z, bz.z);
        let q = Quat::from_matrix(rot);
        let spec = StageSpec::with_single_thruster("hold", 1000.0, 1000.0, 100_000.0, 300.0,
            Vec3::new(0.0, -5.0, 0.0), Vec3::new(0.0, 1.0, 0.0), 10.0, 1.0, 0.0);
        let asm = Assembly::new(&[spec], StateVectors { pos, vel: Vec3::ZERO, omega: Vec3::ZERO, r: rot, q });
        assert!(asm.tip_angle() < 1e-6, "tip 应≈0");
        let (p, y) = asm.pitch_yaw_angles();
        assert!(p.abs() < 1e-6 && y.abs() < 1e-6);
        let (ep, ey) = asm.attitude_errors();
        assert!(ep.abs() < 1e-6 && ey.abs() < 1e-6);
        assert!(asm.roll_angle().abs() < 1e-6);
        assert!(asm.omega().length() < 1e-9);
    }

    #[test]
    fn attitude_reads_active_vessel_omega() {
        let mut asm = Assembly::new(&coaxial_two_stage(), StateVectors::default());
        let w = Vec3::new(0.1, 0.0, -0.05);
        asm.vessels[asm.active].state.omega = w;
        assert_eq!(asm.omega(), w);
    }

    #[test]
    fn mass_readout_falcon9() {
        let asm = Assembly::new(&crate::presets::falcon9(), StateVectors::default());
        let expected = 436_600.0 + 111_500.0 + 22_800.0;
        assert!((asm.total_mass() - expected).abs() < 0.1);
        assert!(asm.fuel_mass() > 0.0);
        assert!(asm.fuel_percent() <= 100.0);
    }

    #[test]
    fn kinematics_reads_state() {
        // 单级：组合体 CG = 该船 pos，state.pos 不被 rebuild 偏移。
        let pos = Vec3::new(1.0, 2.0, 3.0);
        let vel = Vec3::new(4.0, 0.0, 0.0);
        let spec = StageSpec::with_single_thruster("k", 100.0, 100.0, 0.0, 300.0,
            Vec3::new(0.0, -1.0, 0.0), Vec3::new(0.0, 1.0, 0.0), 2.0, 1.0, 0.0);
        let asm = Assembly::new(&[spec], StateVectors {
            pos, vel, omega: Vec3::ZERO,
            r: orbitx_math::Matrix3::IDENTITY, q: orbitx_math::Quat::IDENTITY,
        });
        assert_eq!(asm.position(), pos);
        assert_eq!(asm.velocity(), vel);
        assert!((asm.speed() - 4.0).abs() < 1e-9);
    }

    #[test]
    fn stage_readout_falcon9() {
        let asm = Assembly::new(&crate::presets::falcon9(), StateVectors::default());
        assert_eq!(asm.stage_count(), 3);
        assert_eq!(asm.active_vessel(), 0);
        assert_eq!(asm.active_name(), "F9-S1");
    }

    #[test]
    fn stage_readout_after_separation() {
        let mut asm = Assembly::new(&crate::presets::falcon9(), StateVectors::default());
        asm.separate_stage();
        assert_eq!(asm.stage_count(), 2);
        assert_eq!(asm.active_name(), "F9-S2");
    }

    #[test]
    fn body_readout_initial() {
        let asm = Assembly::new(&coaxial_two_stage(), StateVectors::default());
        assert!(asm.primary_present());
        assert_eq!(asm.detached_vessels().count(), 0);
    }

    #[test]
    fn body_readout_detects_detached() {
        let (stages, links) = core_upper_and_booster();
        let mut asm = Assembly::with_dock_links(&stages, StateVectors::default(), &links);
        let id = asm.vessels[2].id;
        let sep = asm.vessels[2].separation_impulse;
        asm.undock(id, 0, sep);
        assert_eq!(asm.detached_vessels().collect::<Vec<_>>(), vec![2]);
        assert!(asm.primary_present());
    }

    #[test]
    fn body_readout_primary_absent_when_all_detached() {
        let mut asm = Assembly::new(&coaxial_two_stage(), StateVectors::default());
        for v in &mut asm.vessels {
            v.detached = true;
        }
        asm.components.clear();
        assert!(!asm.primary_present());
    }

    #[test]
    fn ambient_pressure_zero_without_atmosphere() {
        let asm = Assembly::new(&coaxial_two_stage(), StateVectors::default());
        assert!(asm.ambient_pressure().abs() < 1e-9);
    }
}
