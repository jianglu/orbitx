//! Ephemeris bridge: synchronizes PlanetarySystem positions to SceneManager.

use std::path::{Path, PathBuf};

use orbitx_config::SystemConfig;
use orbitx_dynamics::PlanetarySystem;
use orbitx_render::{NodeType, PlanetRenderState, SceneNode, SceneManager};

const MJD_J2000: f64 = 51544.5;

pub fn sim_time_to_mjd(sim_time: f64) -> f64 {
    MJD_J2000 + sim_time / 86400.0
}

pub fn create_planetary_system(orbiter_src: &Path) -> PlanetarySystem {
    let config = SystemConfig::sol();
    match PlanetarySystem::from_config(&config, orbiter_src) {
        Ok(psys) => psys,
        Err(e) => {
            eprintln!("Warning: failed to load ephemeris: {e}");
            let nc = strip_ephemeris(&config);
            PlanetarySystem::from_config(&nc, Path::new("/nonexistent"))
                .expect("no-ephemeris config should always work")
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

/// Resolve the directory that contains ephemeris data (`Src/Celbody/...`).
///
/// Search order:
/// 1. `ORBITER_SRC` env var (full Orbiter install, if the user sets it)
/// 2. In-project bundled data `assets/orbiter-data` (compile-time workspace
///    path, so it works regardless of the current working directory)
/// 3. `assets/orbiter-data` relative to the current working directory
/// 4. `../orbiter` legacy fallback
///
/// The bundled data covers the ephemeris `.dat` files needed for positions;
/// gravity models are optional and degrade to point mass if absent.
pub fn resolve_orbiter_src() -> PathBuf {
    if let Ok(p) = std::env::var("ORBITER_SRC") {
        return PathBuf::from(p);
    }

    // Compile-time workspace location: <crate>/../../assets/orbiter-data
    let bundled = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("..")
        .join("..")
        .join("assets")
        .join("orbiter-data");
    if bundled.join("Src/Celbody/Vsop87/Data/Vsop87E_sun.dat").exists() {
        return bundled;
    }

    for cand in ["assets/orbiter-data", "../orbiter"] {
        let p = PathBuf::from(cand);
        if p.join("Src/Celbody/Vsop87/Data/Vsop87E_sun.dat").exists() {
            return p;
        }
    }

    // Last resort: legacy default (may not exist; ephemeris then falls back).
    PathBuf::from("../orbiter")
}

pub fn create_scene_from_psys(psys: &PlanetarySystem) -> SceneManager {
    let mut scene = SceneManager::new();
    for (i, body) in psys.bodies.iter().enumerate() {
        let nt = if body.parent_idx.is_none() {
            NodeType::Star
        } else {
            NodeType::Planet(PlanetRenderState {
                radius: body.radius_m,
                min_render_radius: body.min_render_radius,
                color: body.color,
                has_atmosphere: false,
                has_rings: false,
            })
        };
        let mut node = SceneNode::new(i as u64, nt);
        node.transform.position = body.pos;
        node.transform.scale = body.radius_m;
        node.visible = true;
        scene.add_node(node);
    }
    scene
}

pub fn sync_positions(psys: &PlanetarySystem, scene: &mut SceneManager) {
    let nodes = scene.nodes_mut();
    for (i, body) in psys.bodies.iter().enumerate() {
        if i >= nodes.len() { break; }
        nodes[i].transform.position = body.pos;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn sim_time_j2000() { assert!((sim_time_to_mjd(0.0) - MJD_J2000).abs() < 1e-10); }
    #[test]
    fn sim_time_one_day() { assert!((sim_time_to_mjd(86400.0) - (MJD_J2000+1.0)).abs() < 1e-10); }
    #[test]
    fn scene_no_ephem() {
        let cfg = strip_ephemeris(&SystemConfig::sol());
        let psys = PlanetarySystem::from_config(&cfg, Path::new("/nonexistent")).unwrap();
        let scene = create_scene_from_psys(&psys);
        assert_eq!(scene.len(), 14);
        let ns = scene.nodes();
        assert!(matches!(ns[0].node_type, NodeType::Star));
        assert!(matches!(ns[1].node_type, NodeType::Planet(_)));
    }
    #[test]
    fn sync_pos() {
        let cfg = strip_ephemeris(&SystemConfig::sol());
        let mut psys = PlanetarySystem::from_config(&cfg, Path::new("/nonexistent")).unwrap();
        let mut scene = create_scene_from_psys(&psys);
        psys.bodies[0].pos = orbitx_math::vec3::Vec3::new(1e11, 2e10, -3e10);
        sync_positions(&psys, &mut scene);
        assert!((scene.nodes()[0].transform.position.x - 1e11).abs() < 1.0);
    }
}
