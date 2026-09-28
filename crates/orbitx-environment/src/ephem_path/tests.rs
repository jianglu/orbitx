use super::*;

#[test]
fn resolve_hits_bundled_assets() {
    let p = resolve_ephemeris_data(None);
    assert!(
        looks_like_ephemeris_root(&p) || p.ends_with("orbitx-data"),
        "expected bundled assets path, got {}",
        p.display()
    );
    assert!(
        !p.to_string_lossy().replace('\\', "/").ends_with("/orbiter"),
        "must not fall back to sibling orbiter tree: {}",
        p.display()
    );
}
