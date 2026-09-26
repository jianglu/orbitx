use super::{builtin_aliases, builtin_rocket_toml, load_rocket_source};

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
