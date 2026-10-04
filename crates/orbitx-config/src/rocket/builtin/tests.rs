use super::{builtin_aliases, builtin_rocket_toml, expand_rocket_spec, load_rocket_source};

#[test]
fn falcon9_alias_loads() {
    let cfg = load_rocket_source("falcon9").expect("falcon9");
    assert_eq!(cfg.class, "Falcon9");
    assert!(!cfg.stages.is_empty());
}

#[test]
fn builtin_toml_matches_aliases_table() {
    for (alias, _) in builtin_aliases() {
        assert!(
            builtin_rocket_toml(alias).is_some(),
            "missing toml for {alias}"
        );
    }
}

#[test]
fn unknown_alias_errors() {
    let err = load_rocket_source("not-a-rocket").unwrap_err();
    assert!(err.contains("未知火箭"));
}

#[test]
fn load_from_preset_file_path() {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/presets/saturn_v.toml"
    );
    let cfg = load_rocket_source(path).expect("path");
    assert_eq!(cfg.class, "SaturnV");
}

#[test]
fn expand_tilde_joins_home() {
    let p = expand_rocket_spec("~/Library/Application Support/WLCY/foo.toml");
    let s = p.to_string_lossy();
    assert!(!s.starts_with('~'), "tilde must expand, got {s}");
    assert!(s.contains("Library/Application Support/WLCY/foo.toml"));
}

#[test]
fn expand_strips_wrapping_quotes() {
    let p = expand_rocket_spec("\"~/a/b.toml\"");
    let s = p.to_string_lossy();
    assert!(!s.contains('"'));
    assert!(!s.starts_with('~'));
}

#[test]
fn sim_toml_with_schema_header_loads() {
    let toml = r#"
schema = 1
name = "Demo"
class = "sc_0b49113b"

[[stages]]
name = "S1"
dry_mass = 3100.0
dry_center = [0.0, -1.85, 0.0]
dry_inertia = [2.6e4, 4.3e3, 2.6e4]
length = 10.0
radius = 1.675
separation_impulse = 0.0
docks = []

[[stages.tanks]]
id = 0
max_mass = 37680.0
pos = [0.0, 0.15, 0.0]
inertia = [1.8e5, 5.3e4, 1.8e5]
efficiency = 1.0

[[stages.thrusters]]
pos = [0.0, 0.0, 0.0]
dir = [0.0, 1.0, 0.0]
thrust = 740400.0
isp = 260.7
thrust_sl = 680000.0
isp_sl = 240.0
throttle_rate = 0.8
"#;
    let cfg = crate::RocketConfig::from_toml_str(toml).expect("schema extra key ignored");
    assert_eq!(cfg.name, "Demo");
    assert_eq!(cfg.stages.len(), 1);
    assert_eq!(cfg.stages[0].thrusters.len(), 1);
}
