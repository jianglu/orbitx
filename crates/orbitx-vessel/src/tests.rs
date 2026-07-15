#[cfg(test)]
mod tests {
    use crate::*;
    use orbitx_math::StateVectors;

    fn falcon9() -> Vec<StageSpec> {
        presets::falcon9()
    }

    #[test]
    fn total_mass() {
        let asm = Assembly::new(&falcon9(), StateVectors::default());
        // S1: 25600 + 411000 = 436600
        // S2: 4000 + 107500 = 111500
        // Payload: 22800
        let expected = 436_600.0 + 111_500.0 + 22_800.0;
        assert!(
            (asm.total_mass() - expected).abs() < 0.1,
            "总质量: {} vs 期望 {}",
            asm.total_mass(),
            expected
        );
    }

    #[test]
    fn stage_count() {
        let asm = Assembly::new(&falcon9(), StateVectors::default());
        assert_eq!(asm.stage_count(), 3);
    }

    #[test]
    fn separation_reduces_stages() {
        let mut asm = Assembly::new(&falcon9(), StateVectors::default());
        assert_eq!(asm.stage_count(), 3);
        asm.separate_stage();
        assert_eq!(asm.stage_count(), 2);
        assert_eq!(asm.active_name(), "F9-S2");
    }

    #[test]
    fn double_separation() {
        let mut asm = Assembly::new(&falcon9(), StateVectors::default());
        asm.separate_stage();
        asm.separate_stage();
        assert_eq!(asm.stage_count(), 1);
        assert_eq!(asm.active_name(), "Payload");
    }

    #[test]
    fn no_separation_when_one_stage() {
        let mut asm = Assembly::new(&falcon9(), StateVectors::default());
        asm.separate_stage();
        asm.separate_stage();
        let active = asm.active;
        asm.separate_stage(); // 不应崩溃
        assert_eq!(asm.active, active);
        assert_eq!(asm.stage_count(), 1);
    }

    #[test]
    fn fuel_consumption() {
        let mut asm = Assembly::new(&falcon9(), StateVectors::default());
        asm.set_throttle(1.0);
        let fuel_before = asm.total_fuel();
        // 1 秒，无引力。
        asm.step(1.0, &[]);
        let fuel_after = asm.total_fuel();
        assert!(
            fuel_after < fuel_before,
            "燃料应减少: {} -> {}",
            fuel_before,
            fuel_after
        );
        // 消耗率 = thrust / (isp * g0) = 7607000 / (282 * 9.80665) ≈ 2753.8 kg/s
        let expected_rate = 7_607_000.0 / (282.0 * thruster::G0);
        let actual_rate = fuel_before - fuel_after;
        let rel_err = (actual_rate - expected_rate).abs() / expected_rate;
        // RK4 积分 + 燃料在步末一次性扣除，与连续消耗有轻微差异。
        assert!(
            rel_err < 0.15,
            "消耗率误差 {rel_err}: {actual_rate} vs {expected_rate}"
        );
    }

    #[test]
    fn dock_connections() {
        let asm = Assembly::new(&falcon9(), StateVectors::default());
        // S1 顶端口应连接到 S2 底端口。
        let s1 = &asm.vessels[0];
        let s2 = &asm.vessels[1];
        assert!(s1.docks[1].connected_to.is_some(), "S1 顶端口应已连接");
        assert!(s2.docks[0].connected_to.is_some(), "S2 底端口应已连接");
        // 验证连接目标是正确的 ID。
        let (s1_conn_id, _) = s1.docks[1].connected_to.unwrap();
        assert_eq!(s1_conn_id, s2.id);
    }

    #[test]
    fn separation_breaks_docks() {
        let mut asm = Assembly::new(&falcon9(), StateVectors::default());
        asm.separate_stage();
        // S1 应被标记为 detached。
        assert!(asm.vessels[0].detached);
        // S2 底端口应断开。
        let s2 = &asm.vessels[1];
        assert!(s2.docks[0].connected_to.is_none(), "S2 底端口应已断开");
    }

    #[test]
    fn throttle_clamps() {
        let mut asm = Assembly::new(&falcon9(), StateVectors::default());
        asm.set_throttle(1.5); // 超范围
        for v in &asm.vessels {
            if !v.detached {
                for t in &v.thrusters {
                    assert!(t.level <= 1.0);
                }
            }
        }
    }

    #[test]
    fn saturn_v_mass() {
        let specs = presets::saturn_v();
        let asm = Assembly::new(&specs, StateVectors::default());
        // S-IC: 130000 + 2150000 = 2280000
        // S-II: 36000 + 440000 = 476000
        // S-IVB: 10000 + 110000 = 120000
        // CSM-LM: 45000
        let expected = 2_280_000.0 + 476_000.0 + 120_000.0 + 45_000.0;
        assert!(
            (asm.total_mass() - expected).abs() < 0.1,
            "Saturn V 总质量: {} vs 期望 {}",
            asm.total_mass(),
            expected
        );
    }
}
