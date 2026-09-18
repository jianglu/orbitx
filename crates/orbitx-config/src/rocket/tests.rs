use super::*;

#[test]
fn roundtrip_falcon9() {
    let config = RocketConfig {
        name: "Falcon 9".to_string(),
        class: "Falcon9".to_string(),
        dock_links: None,
        stages: vec![
            StageConfig {
                name: "F9-S1".to_string(),
                dry_mass: 25600.0,
                fuel_mass: 411000.0,
                thrust: 7607000.0,
                isp: 282.0,
                length: 47.0,
                radius: 1.85,
                separation_impulse: 3.0,
                engine_dir: [0.0, 1.0, 0.0],
                engine_pos: [0.0, -23.5, 0.0],
                inertia: None,
                max_gimbal: 0.0,
                max_gimbal_rate: 0.0,
                gimbal_axis: [1.0, 0.0, 0.0],
                docks: None,
            },
            StageConfig {
                name: "F9-S2".to_string(),
                dry_mass: 4000.0,
                fuel_mass: 107500.0,
                thrust: 934000.0,
                isp: 348.0,
                length: 14.0,
                radius: 1.85,
                separation_impulse: 2.0,
                engine_dir: [0.0, 1.0, 0.0],
                engine_pos: [0.0, -7.0, 0.0],
                inertia: None,
                max_gimbal: 0.0,
                max_gimbal_rate: 0.0,
                gimbal_axis: [1.0, 0.0, 0.0],
                docks: None,
            },
        ],
    };

    let toml_str = config.to_toml_string().unwrap();
    let parsed = RocketConfig::from_toml_str(&toml_str).unwrap();

    assert_eq!(parsed.name, "Falcon 9");
    assert_eq!(parsed.stages.len(), 2);
    assert!((parsed.stages[0].dry_mass - 25600.0).abs() < 0.1);
    assert!((parsed.stages[0].engine_dir[1] - 1.0).abs() < 1e-10);
}

#[test]
fn parse_toml_string() {
    let toml_str = r#"
name = "Test Rocket"
class = "TestRocket"

[[stages]]
name = "S1"
dry_mass = 1000.0
fuel_mass = 5000.0
thrust = 100000.0
isp = 300.0
length = 10.0
radius = 1.0
separation_impulse = 2.0
engine_dir = [0.0, 1.0, 0.0]
engine_pos = [0.0, -5.0, 0.0]
"#;
    let config = RocketConfig::from_toml_str(toml_str).unwrap();
    assert_eq!(config.name, "Test Rocket");
    assert_eq!(config.stages.len(), 1);
    assert!((config.stages[0].fuel_mass - 5000.0).abs() < 0.1);
}

#[test]
fn parse_docks_and_dock_links() {
    let toml_str = r#"
name = "Side Mount"
class = "SideMount"
dock_links = [
  { stage = 0, port = 1, remote_stage = 1, remote_port = 0 },
  { stage = 0, port = 2, remote_stage = 2, remote_port = 0 },
]

[[stages]]
name = "Core"
dry_mass = 1000.0
fuel_mass = 1000.0
thrust = 100.0
isp = 300.0
length = 10.0
radius = 1.0
separation_impulse = 1.0
engine_dir = [0.0, 1.0, 0.0]
engine_pos = [0.0, -5.0, 0.0]
docks = [
  { pos = [0.0, -5.0, 0.0], dir = [0.0, -1.0, 0.0], rot = [0.0, 0.0, 1.0] },
  { pos = [0.0, 5.0, 0.0], dir = [0.0, 1.0, 0.0], rot = [0.0, 0.0, 1.0] },
  { pos = [2.0, 0.0, 0.0], dir = [1.0, 0.0, 0.0], rot = [0.0, 0.0, 1.0] },
]

[[stages]]
name = "Upper"
dry_mass = 100.0
fuel_mass = 0.0
thrust = 0.0
isp = 0.0
length = 2.0
radius = 1.0
separation_impulse = 1.0
engine_dir = [0.0, 0.0, 0.0]
engine_pos = [0.0, 0.0, 0.0]

[[stages]]
name = "Booster"
dry_mass = 200.0
fuel_mass = 200.0
thrust = 50.0
isp = 300.0
length = 8.0
radius = 0.5
separation_impulse = 2.0
engine_dir = [0.0, 1.0, 0.0]
engine_pos = [0.0, -4.0, 0.0]
docks = [
  { pos = [-0.5, 0.0, 0.0], dir = [-1.0, 0.0, 0.0], rot = [0.0, 0.0, 1.0] },
]
"#;
    let config = RocketConfig::from_toml_str(toml_str).unwrap();
    assert_eq!(config.stages[0].docks.as_ref().unwrap().len(), 3);
    let links = config.dock_links.unwrap();
    assert_eq!(links.len(), 2);
    assert_eq!(links[1].remote_stage, 2);
}

#[test]
fn parse_long_march_2f_preset() {
    let toml = include_str!("../../presets/long_march_2f.toml");
    let config = RocketConfig::from_toml_str(toml).unwrap();
    assert_eq!(config.class, "LongMarch2F");
    assert_eq!(config.stages.len(), 7);
    assert_eq!(config.dock_links.as_ref().unwrap().len(), 6);
    assert_eq!(config.stages[0].docks.as_ref().unwrap().len(), 6);
    assert_eq!(config.stages[3].docks.as_ref().unwrap().len(), 1);
    // 公开资料矫正：芯一级 4×YF-20 合计推力、助推单台推力
    assert!((config.stages[0].thrust - 2_961_600.0).abs() < 1.0);
    assert!((config.stages[3].thrust - 740_400.0).abs() < 1.0);
    let total: f64 = config.stages.iter().map(|s| s.dry_mass + s.fuel_mass).sum();
    assert!((total - 479_800.0).abs() < 1.0);
}
