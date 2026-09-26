//! Ephemeris bridge: synchronizes PlanetarySystem positions to SceneManager.
//!
//! 历表数据根为 orbitx `assets/orbiter-data`（不依赖兄弟 Orbiter 工程）。

use std::path::{Path, PathBuf};

use orbitx_config::SystemConfig;
use orbitx_dynamics::PlanetarySystem;
use orbitx_math::vec3::Vec3;
use orbitx_render::{NodeType, PlanetRenderState, SceneManager, SceneNode};

use crate::vessel::UserVessel;

const MJD_J2000: f64 = 51544.5;

pub fn sim_time_to_mjd(sim_time: f64) -> f64 {
    MJD_J2000 + sim_time / 86400.0
}

pub fn create_planetary_system(ephemeris_data: &Path) -> PlanetarySystem {
    let config = SystemConfig::sol();
    match PlanetarySystem::from_config(&config, ephemeris_data) {
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

fn looks_like_ephemeris_root(p: &Path) -> bool {
    p.join("Src/Celbody/Vsop87/Data/Vsop87E_sun.dat").exists()
}

/// 解析历表数据根（须含 `Src/Celbody/...`）。
///
/// 顺序：`ORBITX_EPHEMERIS_DATA` → 编译期 `assets/orbiter-data` → cwd `assets/orbiter-data`。
/// **不**回落到 `../orbiter`。
pub fn resolve_ephemeris_data() -> PathBuf {
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

    bundled
}

/// 兼容旧名。
#[deprecated(note = "use resolve_ephemeris_data")]
pub fn resolve_orbiter_src() -> PathBuf {
    resolve_ephemeris_data()
}

/// Body names that have a bundled equirectangular surface map under
/// `assets/textures/planets/<name>.{jpg,png}`.
const TEXTURED_BODIES: &[&str] = &[
    "Mercury", "Venus", "Earth", "Mars", "Moon", "Jupiter", "Saturn", "Uranus", "Neptune",
    "Titan", "Triton", "Io", "Europa", "Ganymede", "Callisto", "Phobos", "Deimos", "Iapetus",
];

/// Bodies with a visible atmosphere (used for the atmosphere shell in P3B-2).
const ATMOSPHERE_BODIES: &[&str] = &[
    "Earth", "Venus", "Mars", "Jupiter", "Saturn", "Uranus", "Neptune", "Titan",
];

fn texture_key_for(name: &str) -> Option<String> {
    TEXTURED_BODIES
        .iter()
        .find(|&&b| b == name)
        .map(|&b| b.to_string())
}

/// Atmosphere glow tint per body (RGB); None if the body has no atmosphere.
fn atmosphere_color_for(name: &str) -> Option<[f32; 3]> {
    match name {
        "Earth" => Some([0.30, 0.55, 1.0]),
        "Venus" => Some([0.95, 0.85, 0.55]),
        "Mars" => Some([0.85, 0.55, 0.4]),
        "Titan" => Some([0.85, 0.6, 0.3]),
        "Jupiter" => Some([0.85, 0.75, 0.6]),
        "Saturn" => Some([0.9, 0.82, 0.65]),
        "Uranus" => Some([0.6, 0.85, 0.9]),
        "Neptune" => Some([0.4, 0.55, 0.95]),
        _ => None,
    }
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
                has_atmosphere: ATMOSPHERE_BODIES.contains(&body.name.as_str()),
                has_rings: body.name == "Saturn",
                texture: texture_key_for(&body.name),
                atmosphere_color: atmosphere_color_for(&body.name),
                clouds: body.name == "Earth",
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
        if i >= nodes.len() {
            break;
        }
        nodes[i].transform.position = body.pos;
    }
}

/// 向场景添加航天器节点，返回其索引。
///
/// 位置由下一帧 [`sync_vessel_position`] 从 UserVessel 相对位置同步。
pub fn add_vessel_node(
    scene: &mut SceneManager,
    mesh_name: &str,
    color: [f32; 4],
    scale_m: f64,
) -> usize {
    let id = scene.len() as u64;
    let node = SceneNode::new_vessel(id, scale_m, mesh_name, color);
    scene.add_node(node);
    scene.len() - 1
}

/// 每帧同步 UserVessel：`scene[idx].pos = parent.pos + vessel.rel_pos`。
pub fn sync_vessel_position(
    scene: &mut SceneManager,
    vessel: &UserVessel,
    psys: &PlanetarySystem,
    vessel_node_idx: usize,
) {
    if vessel.parent_idx >= psys.bodies.len() {
        return;
    }
    let parent = &psys.bodies[vessel.parent_idx];
    let nodes = scene.nodes_mut();
    if vessel_node_idx >= nodes.len() {
        return;
    }
    let abs_pos: Vec3 = parent.pos + vessel.rel_pos;
    nodes[vessel_node_idx].transform.position = abs_pos;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sim_time_j2000() {
        assert!((sim_time_to_mjd(0.0) - MJD_J2000).abs() < 1e-10);
    }

    #[test]
    fn sim_time_one_day() {
        assert!((sim_time_to_mjd(86400.0) - (MJD_J2000 + 1.0)).abs() < 1e-10);
    }

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

    #[test]
    fn resolve_does_not_use_sibling_orbiter() {
        let p = resolve_ephemeris_data();
        let s = p.to_string_lossy().replace('\\', "/");
        assert!(
            s.contains("orbiter-data") || !looks_like_ephemeris_root(&p),
            "unexpected path {}",
            p.display()
        );
        assert!(
            !s.ends_with("/orbiter"),
            "must not fall back to sibling orbiter: {}",
            p.display()
        );
    }
}
