//! PlanetarySystem 加载与历表数据路径（仅 orbitx `assets/orbiter-data`，不依赖 Orbiter 工程）。

use std::path::{Path, PathBuf};

use orbitx_config::SystemConfig;
use orbitx_dynamics::{GravBody, PlanetarySystem};
use orbitx_math::Vec3;
use tracing::warn;

const MJD_J2000: f64 = 51544.5;

fn looks_like_ephemeris_root(p: &Path) -> bool {
    p.join("Src/Celbody/Vsop87/Data/Vsop87E_sun.dat").exists()
}

/// 解析历表数据根目录（须含 `Src/Celbody/...`）。
///
/// 顺序：显式 path → `ORBITX_EPHEMERIS_DATA` → 工作区 `assets/orbiter-data`（编译期）→ cwd `assets/orbiter-data`。
/// **不**回落到兄弟目录 `../orbiter`。
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
        .join("orbiter-data");
    if looks_like_ephemeris_root(&bundled) {
        return bundled;
    }

    let cwd = PathBuf::from("assets/orbiter-data");
    if looks_like_ephemeris_root(&cwd) {
        return cwd;
    }

    // 缺数据时仍返回 bundled 期望路径，由 create_planetary_system fallback。
    bundled
}

/// 加载 `SystemConfig::sol()`；星历失败则 strip 后重试（点质量 fallback）。
pub fn create_planetary_system(ephemeris_data: &Path) -> PlanetarySystem {
    let config = SystemConfig::sol();
    match PlanetarySystem::from_config(&config, ephemeris_data) {
        Ok(mut psys) => {
            psys.mjd = MJD_J2000;
            psys.update_positions();
            psys
        }
        Err(e) => {
            warn!(
                error = %e,
                path = %ephemeris_data.display(),
                "ephemeris load failed; using no-ephemeris fallback"
            );
            let nc = strip_ephemeris(&config);
            let mut psys = PlanetarySystem::from_config(&nc, Path::new("/nonexistent"))
                .expect("no-ephemeris config should always work");
            psys.mjd = MJD_J2000;
            psys.update_positions();
            psys
        }
    }
}

fn strip_ephemeris(config: &SystemConfig) -> SystemConfig {
    let mut s = config.clone();
    for b in &mut s.bodies {
        b.ephemeris = None;
        b.gravity = None;
        b.rotation = None;
    }
    s
}

/// 将星历位置平移为**地心系** `GravBody`（地球在原点），供 `Assembly::step` / 大气使用。
pub fn grav_bodies_earth_centered(psys: &PlanetarySystem) -> Vec<GravBody> {
    let earth_pos = earth_heliocentric_pos(psys);
    let mut bodies = psys.to_grav_bodies();
    for b in &mut bodies {
        b.pos = b.pos - earth_pos;
    }
    bodies
}

/// `to_grav_bodies` 会跳过 `mass==0`；把 `psys` 体名映射到过滤后切片下标。
pub fn grav_body_index(psys: &PlanetarySystem, name: &str) -> Option<usize> {
    let idx = psys.body_index(name)?;
    if psys.bodies[idx].mass <= 0.0 {
        return None;
    }
    Some(
        psys.bodies[..idx]
            .iter()
            .filter(|b| b.mass > 0.0)
            .count(),
    )
}

/// 地心系引力表 + Earth 为 `StepEnv::primary`（环境权威；vessel 不猜测）。
pub fn earth_centered_grav_env(psys: &PlanetarySystem) -> (Vec<GravBody>, usize) {
    let bodies = grav_bodies_earth_centered(psys);
    let primary = grav_body_index(psys, "Earth").unwrap_or(0);
    debug_assert!(
        primary < bodies.len() || bodies.is_empty(),
        "Earth primary {primary} out of range for {} grav bodies",
        bodies.len()
    );
    (bodies, primary)
}

pub fn earth_heliocentric_pos(psys: &PlanetarySystem) -> Vec3 {
    psys.body_index("Earth")
        .map(|i| psys.bodies[i].pos)
        .unwrap_or(Vec3::ZERO)
}

pub fn earth_radius_m(psys: &PlanetarySystem) -> f64 {
    psys.body_index("Earth")
        .map(|i| psys.bodies[i].size)
        .unwrap_or_else(|| orbitx_config::BodyConfig::earth().size)
}

pub fn earth_mass_kg(psys: &PlanetarySystem) -> f64 {
    psys.body_index("Earth")
        .map(|i| psys.bodies[i].mass)
        .unwrap_or_else(|| orbitx_config::BodyConfig::earth().mass)
}

/// 恒星自转周期：与大气同源，取 `BodyConfig::earth()`（P4.2；CelestialBody 未承载 sid 周期）。
pub fn earth_sid_rot_period(psys: &PlanetarySystem) -> f64 {
    let _ = psys;
    orbitx_config::BodyConfig::earth()
        .rotation
        .as_ref()
        .map(|r| r.sid_rot_period)
        .unwrap_or(86_164.1)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn resolve_hits_bundled_assets_without_orbiter_tree() {
        let p = resolve_ephemeris_data(None);
        assert!(
            looks_like_ephemeris_root(&p) || p.ends_with("orbiter-data"),
            "expected bundled assets path, got {}",
            p.display()
        );
        // 工作区应自带历表；不依赖 ../orbiter
        assert!(
            !p.to_string_lossy().replace('\\', "/").ends_with("/orbiter"),
            "must not fall back to sibling orbiter tree: {}",
            p.display()
        );
    }

    #[test]
    fn earth_primary_not_sun_in_sol_grav_list() {
        let src = resolve_ephemeris_data(None);
        let mut psys = create_planetary_system(&src);
        psys.update_positions();
        let (grav, primary) = earth_centered_grav_env(&psys);
        assert!(!grav.is_empty());
        assert!(primary < grav.len());
        // Earth at origin after centering; Sun is far.
        assert!(
            grav[primary].pos.length() < 1.0,
            "primary body should be Earth at origin, got |pos|={}",
            grav[primary].pos.length()
        );
        assert!(
            primary > 0,
            "Sun is first in sol(); Earth primary must be > 0"
        );
        let em = earth_mass_kg(&psys);
        assert!((grav[primary].mass - em).abs() / em < 1e-9);
    }
}
