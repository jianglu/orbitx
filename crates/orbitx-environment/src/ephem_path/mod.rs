//! 历表数据根。产品只认 `assets/orbitx-data` 这一布局。

use std::path::{Path, PathBuf};

pub fn looks_like_ephemeris_root(p: &Path) -> bool {
    p.join("Src/Celbody/Vsop87/Data/Vsop87E_sun.dat").exists()
}

/// 解析历表数据根。
///
/// 顺序：显式 path → `ORBITX_EPHEMERIS_DATA` → 编译期 `assets/orbitx-data` → cwd。
/// `orbitx-runtime` 不得用本函数填补缺失的 `--ephemeris-data`；只给 CLI 与测试展开默认路径。
pub fn resolve_ephemeris_data(explicit: Option<&Path>) -> PathBuf {
    if let Some(p) = explicit {
        return p.to_path_buf();
    }
    if let Ok(p) = std::env::var("ORBITX_EPHEMERIS_DATA") {
        return PathBuf::from(p);
    }

    let bundled = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("..")
        .join("..")
        .join("assets")
        .join("orbitx-data");
    if looks_like_ephemeris_root(&bundled) {
        return bundled;
    }

    let cwd = PathBuf::from("assets/orbitx-data");
    if looks_like_ephemeris_root(&cwd) {
        return cwd;
    }
    bundled
}

pub fn find_vsop_path(orbiter_src: &Path, dat_file: &str) -> PathBuf {
    orbiter_src
        .join("Src")
        .join("Celbody")
        .join("Vsop87")
        .join("Data")
        .join(dat_file)
}

pub fn find_elp_path(orbiter_src: &Path, dat_file: &str) -> PathBuf {
    let primary = orbiter_src
        .join("Src")
        .join("Celbody")
        .join("Moon")
        .join(dat_file);
    if primary.exists() {
        return primary;
    }
    orbiter_src
        .join("Src")
        .join("Celbody")
        .join("Moon")
        .join("Config")
        .join("Moon")
        .join("Data")
        .join(dat_file)
}

pub fn find_galsat_path(orbiter_src: &Path, dat_file: &str) -> PathBuf {
    let primary = orbiter_src
        .join("Src")
        .join("Celbody")
        .join("Galsat")
        .join(dat_file);
    if primary.exists() {
        return primary;
    }
    orbiter_src
        .join("Src")
        .join("Celbody")
        .join("Galsat")
        .join("Data")
        .join(dat_file)
}

pub fn find_tass_path(orbiter_src: &Path, dat_file: &str) -> PathBuf {
    let primary = orbiter_src
        .join("Src")
        .join("Celbody")
        .join("Satsat")
        .join(dat_file);
    if primary.exists() {
        return primary;
    }
    orbiter_src
        .join("Src")
        .join("Celbody")
        .join("Satsat")
        .join("Data")
        .join(dat_file)
}

pub fn find_gravity_model_path(orbiter_src: &Path, model_path: &str) -> PathBuf {
    orbiter_src.join("GravityModels").join(model_path)
}

#[cfg(test)]
mod tests;
