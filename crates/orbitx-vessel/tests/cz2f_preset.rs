//! CZ-2F TOML → Assembly 端到端：独立助推建树 + undock。

use orbitx_config::RocketConfig;
use orbitx_math::StateVectors;
use orbitx_vessel::{Assembly, StageSpec, stage_spec_from_config};

fn stages_and_links_from_toml(toml: &str) -> (Vec<StageSpec>, Vec<(usize, usize, usize, usize)>) {
    let config = RocketConfig::from_toml_str(toml).expect("parse long_march_2f");
    let stages: Vec<StageSpec> = config.stages.iter().map(stage_spec_from_config).collect();
    let links = config
        .dock_links
        .expect("dock_links")
        .iter()
        .map(|l| (l.stage, l.port, l.remote_stage, l.remote_port))
        .collect();
    (stages, links)
}

#[test]
fn long_march_2f_preset_builds_and_undocks_booster() {
    let toml = include_str!("../../orbitx-config/presets/long_march_2f.toml");
    let (stages, links) = stages_and_links_from_toml(toml);
    assert_eq!(stages.len(), 7);
    assert_eq!(links.len(), 6);

    let mut asm = Assembly::with_dock_links(&stages, StateVectors::default(), &links);
    assert_eq!(asm.components.len(), 7);
    // Y 型公开起飞质量约 479.8 t（芯+二级+四助推+载荷差额）
    assert!(
        (asm.total_mass() - 479_800.0).abs() < 1.0,
        "total_mass={}",
        asm.total_mass()
    );

    // 起飞推力：芯 + 四助推（二级此时未点火）
    let liftoff_thrust: f64 = [0usize, 3, 4, 5, 6]
        .iter()
        .map(|&i| {
            asm.vessels[i]
                .thrusters
                .iter()
                .map(|t| t.max_thrust)
                .sum::<f64>()
        })
        .sum();
    assert!(
        (liftoff_thrust - 5_923_200.0).abs() < 1.0,
        "liftoff_thrust={liftoff_thrust}"
    );

    // 显式 undock 侧口（物理原语）；控制顺序由上层决定
    let leave = asm.undock(asm.vessels[0].id, 2, 4.0);
    assert_eq!(leave.len(), 1);
    assert!(asm.vessels[leave[0]].detached);
    assert_eq!(asm.stage_count(), 6);
    assert_eq!(asm.components.len(), 6);
}

#[test]
fn long_march_2f_undock_all_boosters_then_core() {
    let toml = include_str!("../../orbitx-config/presets/long_march_2f.toml");
    let (stages, links) = stages_and_links_from_toml(toml);
    let mut asm = Assembly::with_dock_links(&stages, StateVectors::default(), &links);

    // 芯级侧口 2..5 → 四助推
    for port in 2..=5 {
        let leave = asm.undock(asm.vessels[0].id, port, 4.0);
        assert_eq!(leave.len(), 1, "port {port}");
    }
    assert_eq!(asm.components.len(), 3); // 芯+二级+神舟
    assert!(!asm.vessels[0].detached);

    // 同轴拆芯：separate_stage
    let _ = asm.separate_stage();
    assert!(asm.vessels[0].detached);
    assert_eq!(asm.components.len(), 2);
}
