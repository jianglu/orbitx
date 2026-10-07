use super::*;

fn merlin(pos: [f64; 3]) -> ThrusterConfig {
    ThrusterConfig {
        pos,
        dir: [0.0, 1.0, 0.0],
        thrust: 914_000.0,
        isp: 311.0,
        thrust_sl: Some(845_000.0),
        isp_sl: Some(282.0),
        max_gimbal: 0.122,
        max_gimbal_rate: 0.35,
        gimbal_axis: [1.0, 0.0, 0.0],
        throttle_rate: 0.8,
    }
}

fn tank(max_mass: f64) -> TankConfig {
    TankConfig {
        id: 0,
        max_mass,
        mass: -1.0,
        pos: [0.0, 0.0, 0.0],
        inertia: [1.0e6, 1.0e5, 1.0e6],
        efficiency: 1.0,
    }
}

#[test]
fn roundtrip_falcon9() {
    let mut s1_thrusters = vec![merlin([0.0, -23.5, 0.0])];
    let r = 1.2;
    for i in 0..8 {
        let a = (i as f64) * std::f64::consts::TAU / 8.0;
        s1_thrusters.push(merlin([r * a.cos(), -23.5, r * a.sin()]));
    }

    let config = RocketConfig {
        name: "Falcon 9".to_string(),
        class: "Falcon9".to_string(),
        dock_links: None,
        stages: vec![
            StageConfig {
                name: "F9-S1".to_string(),
                dry_mass: 25600.0,
                dry_center: [0.0, -1.88, 0.0],
                dry_inertia: [4.7e6, 4.4e4, 4.7e6],
                tanks: vec![tank(411000.0)],
                thrusters: s1_thrusters,
                length: 47.0,
                radius: 1.85,
                separation_impulse: 3.0,
                tidaldamp: 0.0,
                cd_mach: vec![
                    [0.0, 0.30],
                    [0.6, 0.32],
                    [0.9, 0.55],
                    [1.1, 0.95],
                    [1.5, 0.70],
                    [2.5, 0.45],
                    [5.0, 0.35],
                ],
                cn_alpha: None,
                lifting_surfaces: vec![],
                docks: None,
            },
            StageConfig {
                name: "F9-S2".to_string(),
                dry_mass: 4000.0,
                dry_center: [0.0, -1.93, 0.0],
                dry_inertia: [6.9e4, 6.8e3, 6.9e4],
                tanks: vec![tank(107500.0)],
                thrusters: vec![ThrusterConfig {
                    pos: [0.0, -7.0, 0.0],
                    dir: [0.0, 1.0, 0.0],
                    thrust: 934_000.0,
                    isp: 348.0,
                    thrust_sl: Some(900_000.0),
                    isp_sl: Some(330.0),
                    max_gimbal: 0.087,
                    max_gimbal_rate: 0.17,
                    gimbal_axis: [1.0, 0.0, 0.0],
                    throttle_rate: 0.8,
                }],
                length: 14.0,
                radius: 1.85,
                separation_impulse: 2.0,
                tidaldamp: 0.0,
                cd_mach: vec![[0.0, 0.30], [5.0, 0.35]],
                cn_alpha: None,
                lifting_surfaces: vec![],
                docks: None,
            },
        ],
    };

    let toml_str = config.to_toml_string().unwrap();
    let parsed = RocketConfig::from_toml_str(&toml_str).unwrap();

    assert_eq!(parsed.name, "Falcon 9");
    assert_eq!(parsed.stages.len(), 2);
    assert!((parsed.stages[0].dry_mass - 25600.0).abs() < 0.1);
    assert_eq!(parsed.stages[0].thrusters.len(), 9);
    assert!((parsed.stages[0].thrusters[0].dir[1] - 1.0).abs() < 1e-10);
    assert!((parsed.stages[0].vacuum_thrust_sum() - 9.0 * 914_000.0).abs() < 1.0);
    assert!((parsed.stages[0].fuel_mass() - 411000.0).abs() < 0.1);
}

#[test]
fn parse_toml_string() {
    let toml_str = r#"
name = "Test Rocket"
class = "TestRocket"

[[stages]]
name = "S1"
dry_mass = 1000.0
dry_center = [0.0, -1.67, 0.0]
dry_inertia = [1.0e4, 500.0, 1.0e4]
length = 10.0
radius = 1.0
separation_impulse = 2.0

[[stages.tanks]]
id = 0
max_mass = 5000.0
mass = 5000.0
pos = [0.0, 0.33, 0.0]
inertia = [2.0e4, 2000.0, 2.0e4]
efficiency = 1.0

[[stages.thrusters]]
pos = [0.0, -5.0, 0.0]
dir = [0.0, 1.0, 0.0]
thrust = 100000.0
isp = 300.0
"#;
    let config = RocketConfig::from_toml_str(toml_str).unwrap();
    assert_eq!(config.name, "Test Rocket");
    assert_eq!(config.stages.len(), 1);
    assert!((config.stages[0].fuel_mass() - 5000.0).abs() < 0.1);
    assert_eq!(config.stages[0].thrusters.len(), 1);
    assert!((config.stages[0].thrusters[0].thrust - 100_000.0).abs() < 0.1);
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
length = 10.0
radius = 1.0
separation_impulse = 1.0
docks = [
  { pos = [0.0, -5.0, 0.0], dir = [0.0, -1.0, 0.0], rot = [0.0, 0.0, 1.0] },
  { pos = [0.0, 5.0, 0.0], dir = [0.0, 1.0, 0.0], rot = [0.0, 0.0, 1.0] },
  { pos = [2.0, 0.0, 0.0], dir = [1.0, 0.0, 0.0], rot = [0.0, 0.0, 1.0] },
]

[[stages.tanks]]
id = 0
max_mass = 1000.0
pos = [0.0, 0.0, 0.0]
inertia = [1.0e3, 1.0e2, 1.0e3]

[[stages.thrusters]]
pos = [0.0, -5.0, 0.0]
dir = [0.0, 1.0, 0.0]
thrust = 100.0
isp = 300.0

[[stages]]
name = "Upper"
dry_mass = 100.0
length = 2.0
radius = 1.0
separation_impulse = 1.0
thrusters = []

[[stages]]
name = "Booster"
dry_mass = 200.0
length = 8.0
radius = 0.5
separation_impulse = 2.0
docks = [
  { pos = [-0.5, 0.0, 0.0], dir = [-1.0, 0.0, 0.0], rot = [0.0, 0.0, 1.0] },
]

[[stages.tanks]]
id = 0
max_mass = 200.0
pos = [0.0, 0.0, 0.0]
inertia = [100.0, 10.0, 100.0]

[[stages.thrusters]]
pos = [0.0, -4.0, 0.0]
dir = [0.0, 1.0, 0.0]
thrust = 50.0
isp = 300.0
"#;
    let config = RocketConfig::from_toml_str(toml_str).unwrap();
    assert_eq!(config.stages[0].docks.as_ref().unwrap().len(), 3);
    assert_eq!(config.stages[0].thrusters.len(), 1);
    assert!(config.stages[1].thrusters.is_empty());
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
    assert!((config.stages[0].vacuum_thrust_sum() - 2_961_600.0).abs() < 1.0);
    assert!((config.stages[3].thrusters[0].thrust - 740_400.0).abs() < 1.0);
    for (i, stage) in config.stages.iter().enumerate() {
        let powered = i != 2; // Shenzhou 载荷
        assert_eq!(
            !stage.thrusters.is_empty(),
            powered,
            "stage {i} ({}) thrusters",
            stage.name
        );
    }
    let total: f64 = config
        .stages
        .iter()
        .map(|s| s.dry_mass + s.fuel_mass())
        .sum();
    assert!((total - 479_800.0).abs() < 1.0);
}

#[test]
fn parse_lifting_surfaces() {
    let toml_str = r#"
name = "Fin Rocket"
class = "FinRocket"

[[stages]]
name = "S1"
dry_mass = 1000.0
length = 10.0
radius = 1.0
separation_impulse = 1.0
cd_mach = [[0.0, 0.3], [5.0, 0.35]]
cn_alpha = 2.2

[[stages.lifting_surfaces]]
ref = [0.5, -4.0, 0.0]
normal = [1.0, 0.0, 0.0]
chord_dir = [0.0, -1.0, 0.0]
area = 0.4
aspect_ratio = 2.0
cl_alpha = 3.5
cd0 = 0.02
alpha_stall0 = 0.314
kind = "grid"
deploy = 0.0
deploy_target = 1.0
deploy_rate = 0.5

[[stages.lifting_surfaces]]
ref = [-0.5, -4.0, 0.0]
normal = [-1.0, 0.0, 0.0]
area = 0.4
aspect_ratio = 2.0
cl_alpha = 3.5
kind = "fixed"
"#;
    let config = RocketConfig::from_toml_str(toml_str).unwrap();
    let s = &config.stages[0];
    assert!((s.cn_alpha.unwrap() - 2.2).abs() < 1e-12);
    assert_eq!(s.lifting_surfaces.len(), 2);
    let g = &s.lifting_surfaces[0];
    assert!((g.ref_pos[0] - 0.5).abs() < 1e-12);
    assert_eq!(g.kind, FinKindConfig::Grid);
    assert!((g.deploy_rate - 0.5).abs() < 1e-12);
    let f = &s.lifting_surfaces[1];
    assert_eq!(f.kind, FinKindConfig::Fixed);
    assert!(f.chord_dir.is_none());
    assert!((f.deploy - 1.0).abs() < 1e-12);
    assert!(f.alpha_stall0.is_none());
}

#[test]
fn falcon9_preset_has_grid_fins() {
    let toml = include_str!("../../presets/falcon9.toml");
    let config = RocketConfig::from_toml_str(toml).unwrap();
    assert_eq!(config.stages[0].lifting_surfaces.len(), 4);
    assert_eq!(config.stages[0].lifting_surfaces[0].kind, FinKindConfig::Grid);
    assert!(config.stages[1].lifting_surfaces.is_empty());
}

#[test]
fn saturn_v_preset_has_fixed_fins() {
    let toml = include_str!("../../presets/saturn_v.toml");
    let config = RocketConfig::from_toml_str(toml).unwrap();
    assert_eq!(config.stages[0].lifting_surfaces.len(), 4);
    assert_eq!(config.stages[0].lifting_surfaces[0].kind, FinKindConfig::Fixed);
}
