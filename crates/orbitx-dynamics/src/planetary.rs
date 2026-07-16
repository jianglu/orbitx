//! Multi-body planetary system container.
//!
//! Mirrors Orbiter's `PlanetarySystem` (Psys.cpp): a tree of celestial bodies
//! (star → planets → moons) with ephemeris-driven positions, rotation models,
//! and gravity field aggregation.

use std::io::BufReader;
use std::path::Path;
use std::sync::Arc;

use orbitx_config::{EphemerisConfig, GravityConfig, SystemConfig};
use orbitx_math::consts::GGRAV;
use orbitx_math::mat3::Matrix3;
use orbitx_math::vec3::Vec3;

use crate::gravity::{jcoeff_perturbation_with_rot, pines_perturbation, GravBody};
use crate::pines::PinesModel;
use crate::rotation::RotationState;

// ─── Ephemeris model wrapper ───

/// Ephemeris model for a celestial body.
pub enum EphemerisModel {
    /// VSOP87 planetary ephemeris.
    Vsop87(orbitx_ephemeris::VsopModel),
    /// ELP2000-82 lunar ephemeris.
    Elp82(orbitx_ephemeris::ElpModel),
    /// GALSAT Jupiter Galilean moon ephemeris.
    Galsat {
        model: orbitx_ephemeris::GalModel,
        index: usize,
    },
    /// TASS17 Saturn moon ephemeris.
    Tass17 {
        model: orbitx_ephemeris::TasModel,
        index: usize,
    },
}

impl EphemerisModel {
    /// Evaluate position + velocity at MJD.
    /// Returns [x, y, z, vx, vy, vz] in meters/m/s relative to parent body.
    pub fn eval(&mut self, mjd: f64) -> [f64; 6] {
        match self {
            EphemerisModel::Vsop87(model) => {
                let ret = model.eval(mjd);
                if model.series.is_polar() {
                    // Series B: polar → cartesian.
                    polar_to_cartesian(ret[0], ret[1], ret[2])
                } else {
                    // Series E: already cartesian (meters).
                    ret
                }
            }
            EphemerisModel::Elp82(model) => model.eval(mjd),
            EphemerisModel::Galsat { model, index } => {
                // GALSAT uses JD, not MJD. ksat is i32.
                let jd = mjd + 2_400_000.5;
                model.eval(jd, *index as i32)
            }
            EphemerisModel::Tass17 { model, index } => {
                let jd = mjd + 2_400_000.5;
                model.eval(jd, *index)
            }
        }
    }
}

/// VSOP87 series B polar → cartesian conversion.
fn polar_to_cartesian(l: f64, b: f64, r_au: f64) -> [f64; 6] {
    let au_meters = 1.495978707e11;
    let r = r_au * au_meters;
    let cosb = b.cos();
    let cosl = l.cos();
    let sinl = l.sin();
    let sinb = b.sin();
    [r * cosb * cosl, r * sinb, r * cosb * sinl, 0.0, 0.0, 0.0]
}

// ─── Gravity model ───

/// Gravity model for a celestial body.
pub enum GravityModel {
    /// Point-mass only.
    PointMass,
    /// J-coefficient zonal harmonics (J2, J3, J4, ...).
    Jcoeff { values: Vec<f64> },
    /// Pines spherical-harmonic gravity model.
    Pines { model: Arc<PinesModel>, cutoff: usize },
}

// ─── Celestial body ───

/// A celestial body with full physical parameters.
pub struct CelestialBody {
    /// Name.
    pub name: String,
    /// Mass [kg].
    pub mass: f64,
    /// Mean radius [m].
    pub size: f64,
    /// Current position in global frame [m].
    pub pos: Vec3,
    /// Parent body index (None for star/root).
    pub parent_idx: Option<usize>,

    /// Ephemeris model (None for fixed-position bodies).
    pub ephemeris: Option<EphemerisModel>,
    /// Rotation state (None for non-rotating bodies).
    pub rotation: Option<RotationState>,
    /// Gravity model.
    pub gravity: GravityModel,

    /// Render color [r, g, b, a].
    pub color: [f32; 4],
    /// Physical radius for display [m].
    pub radius_m: f64,
    /// Minimum render radius.
    pub min_render_radius: f32,
}

impl CelestialBody {
    /// Convenience: G * mass.
    pub fn gm(&self) -> f64 {
        GGRAV * self.mass
    }

    /// Get rotation matrix (identity if no rotation model).
    pub fn rot_matrix(&self) -> Matrix3 {
        match &self.rotation {
            Some(r) => *r.rot_matrix(),
            None => Matrix3::IDENTITY,
        }
    }
}

// ─── Planetary system ───

/// Multi-body planetary system: tree of celestial bodies with gravity aggregation.
pub struct PlanetarySystem {
    /// All celestial bodies (star + planets + moons).
    pub bodies: Vec<CelestialBody>,
    /// Indices of massive bodies sorted by mass (descending), for gravity computation.
    pub celestials: Vec<usize>,
    /// Current MJD.
    pub mjd: f64,
}

impl PlanetarySystem {
    /// Build from a `SystemConfig`, loading ephemeris data and gravity models.
    ///
    /// `orbiter_src` is the Orbiter source directory root (for finding .dat files).
    pub fn from_config(config: &SystemConfig, orbiter_src: &Path) -> Result<Self, String> {
        let mut bodies = Vec::with_capacity(config.bodies.len());

        for body_cfg in &config.bodies {
            let ephemeris = load_ephemeris(&body_cfg.ephemeris, orbiter_src)?;
            let rotation = body_cfg.rotation.as_ref().map(RotationState::from_config);
            let gravity = load_gravity(&body_cfg.gravity, orbiter_src)?;

            // Find parent index.
            let parent_idx = config
                .parents
                .iter()
                .find(|(c, _)| c == &body_cfg.name)
                .and_then(|(_, p)| config.bodies.iter().position(|b| b.name == *p));

            bodies.push(CelestialBody {
                name: body_cfg.name.clone(),
                mass: body_cfg.mass,
                size: body_cfg.size,
                pos: Vec3::ZERO, // Will be updated by update_positions.
                parent_idx,
                ephemeris,
                rotation,
                gravity,
                color: body_cfg.color,
                radius_m: body_cfg.size,
                min_render_radius: body_cfg.min_render_radius,
            });
        }

        // Sort massive bodies by mass (descending).
        let mut celestials: Vec<usize> = (0..bodies.len()).collect();
        celestials.sort_by(|&a, &b| bodies[b].mass.partial_cmp(&bodies[a].mass).unwrap());

        Ok(PlanetarySystem {
            bodies,
            celestials,
            mjd: 51544.5, // J2000
        })
    }

    /// Update all body positions from ephemeris at current MJD.
    pub fn update_positions(&mut self) {
        // First pass: evaluate ephemeris for all bodies.
        // Store results temporarily to avoid borrow issues.
        let mut positions: Vec<Option<[f64; 6]>> = vec![None; self.bodies.len()];

        for (i, body) in self.bodies.iter_mut().enumerate() {
            if let Some(ref mut eph) = body.ephemeris {
                positions[i] = Some(eph.eval(self.mjd));
            }
        }

        // Second pass: compute global positions.
        // For VSOP87 bodies (planets), position is already heliocentric.
        // For moon ephemerides (ELP82, GALSAT, TASS17), position is relative to parent.
        // We need to handle parent positions carefully to avoid borrow issues.
        // Process in order: bodies without parents first, then those with parents
        // (since parents come before children in the config).
        let mut new_positions: Vec<Vec3> = self.bodies.iter().map(|b| b.pos).collect();

        for (i, body) in self.bodies.iter().enumerate() {
            if let Some(pos_vel) = positions[i] {
                let local_pos = Vec3::new(pos_vel[0], pos_vel[1], pos_vel[2]);

                if let Some(parent_idx) = body.parent_idx {
                    // Moon: add parent position (already computed since parents come first).
                    new_positions[i] = new_positions[parent_idx] + local_pos;
                } else {
                    // Planet/star: position is heliocentric.
                    new_positions[i] = local_pos;
                }
            }
        }

        // Apply new positions.
        for (i, body) in self.bodies.iter_mut().enumerate() {
            body.pos = new_positions[i];
        }

        // Update rotation states.
        for body in &mut self.bodies {
            if let Some(ref mut rot) = body.rotation {
                // Convert MJD to simulation time (seconds from J2000).
                let sim_t = (self.mjd - 51544.5) * 86400.0;
                rot.update_precession(self.mjd);
                rot.update_rotation(sim_t);
            }
        }
    }

    /// Compute N-body gravitational acceleration at position `gpos`.
    ///
    /// Includes point-mass, J-coeff, and Pines perturbations.
    /// `exclude` optionally skips a body by index.
    pub fn gacc(&self, gpos: Vec3, exclude: Option<usize>) -> Vec3 {
        let mut acc = Vec3::ZERO;
        for &bi in &self.celestials {
            if Some(bi) == exclude {
                continue;
            }
            let body = &self.bodies[bi];
            let rpos = body.pos - gpos;
            let d = rpos.length();
            if d < 1.0 {
                continue;
            }

            // Point-mass.
            acc += rpos * (body.gm() / (d * d * d));

            // Perturbation.
            let rot = body.rot_matrix();
            match &body.gravity {
                GravityModel::PointMass => {}
                GravityModel::Jcoeff { values } => {
                    acc += jcoeff_perturbation_with_rot(rpos, body.size, body.gm(), values, &rot);
                }
                GravityModel::Pines { model, cutoff } => {
                    acc += pines_perturbation(rpos, model, *cutoff, &rot);
                }
            }
        }
        acc
    }

    /// Convert to `Vec<GravBody>` for backward compatibility.
    pub fn to_grav_bodies(&self) -> Vec<GravBody> {
        self.bodies
            .iter()
            .filter(|b| b.mass > 0.0)
            .map(|b| GravBody {
                pos: b.pos,
                mass: b.mass,
                size: b.size,
                jcoeff: match &b.gravity {
                    GravityModel::Jcoeff { values } => values.clone(),
                    _ => vec![],
                },
                rotation: Some(b.rot_matrix()),
                pines: match &b.gravity {
                    GravityModel::Pines { model, cutoff } => {
                        Some((model.clone(), *cutoff))
                    }
                    _ => None,
                },
            })
            .collect()
    }

    /// Find body index by name.
    pub fn body_index(&self, name: &str) -> Option<usize> {
        self.bodies.iter().position(|b| b.name == name)
    }

    /// Advance MJD by `dt_days` days.
    pub fn advance(&mut self, dt_days: f64) {
        self.mjd += dt_days;
    }
}

// ─── Loading helpers ───

fn load_ephemeris(
    cfg: &Option<EphemerisConfig>,
    orbiter_src: &Path,
) -> Result<Option<EphemerisModel>, String> {
    match cfg {
        None => Ok(None),
        Some(EphemerisConfig::Vsop87 {
            dat_file,
            series,
            a0,
            prec,
            interval,
        }) => {
            let path = find_vsop_path(orbiter_src, dat_file);
            match std::fs::File::open(&path) {
                Ok(file) => {
                    let s = if series == "E" {
                        orbitx_ephemeris::Series::E
                    } else {
                        orbitx_ephemeris::Series::B
                    };
                    match orbitx_ephemeris::VsopModel::from_reader(
                        BufReader::new(file),
                        s,
                        *a0,
                        *prec,
                        *interval,
                    ) {
                        Ok(model) => Ok(Some(EphemerisModel::Vsop87(model))),
                        Err(e) => Err(format!("解析 {} 失败: {e}", dat_file)),
                    }
                }
                Err(e) => Err(format!("无法读取 {}: {e}", path.display())),
            }
        }
        Some(EphemerisConfig::Elp82 { dat_file, prec }) => {
            let path = find_elp_path(orbiter_src, dat_file);
            match std::fs::File::open(&path) {
                Ok(file) => {
                    match orbitx_ephemeris::ElpModel::from_reader(BufReader::new(file), *prec) {
                        Ok(model) => Ok(Some(EphemerisModel::Elp82(model))),
                        Err(e) => Err(format!("解析 {} 失败: {e}", dat_file)),
                    }
                }
                Err(e) => Err(format!("无法读取 {}: {e}", path.display())),
            }
        }
        Some(EphemerisConfig::Galsat { dat_file, index }) => {
            let path = find_galsat_path(orbiter_src, dat_file);
            match std::fs::File::open(&path) {
                Ok(file) => {
                    match orbitx_ephemeris::GalModel::from_reader(BufReader::new(file)) {
                        Ok(model) => Ok(Some(EphemerisModel::Galsat {
                            model,
                            index: *index,
                        })),
                        Err(e) => Err(format!("解析 {} 失败: {e}", dat_file)),
                    }
                }
                Err(e) => Err(format!("无法读取 {}: {e}", path.display())),
            }
        }
        Some(EphemerisConfig::Tass17 { dat_file, index }) => {
            let path = find_tass_path(orbiter_src, dat_file);
            match std::fs::File::open(&path) {
                Ok(file) => {
                    match orbitx_ephemeris::TasModel::from_reader(BufReader::new(file)) {
                        Ok(model) => Ok(Some(EphemerisModel::Tass17 {
                            model,
                            index: *index,
                        })),
                        Err(e) => Err(format!("解析 {} 失败: {e}", dat_file)),
                    }
                }
                Err(e) => Err(format!("无法读取 {}: {e}", path.display())),
            }
        }
    }
}

fn load_gravity(cfg: &Option<GravityConfig>, orbiter_src: &Path) -> Result<GravityModel, String> {
    match cfg {
        None => Ok(GravityModel::PointMass),
        Some(GravityConfig::Jcoeff { values }) => {
            Ok(GravityModel::Jcoeff {
                values: values.clone(),
            })
        }
        Some(GravityConfig::Pines {
            model_path,
            cutoff,
        }) => {
            let path = find_gravity_model_path(orbiter_src, model_path);
            match std::fs::File::open(&path) {
                Ok(file) => match PinesModel::from_reader(BufReader::new(file), *cutoff) {
                    Ok(model) => Ok(GravityModel::Pines {
                        model: Arc::new(model),
                        cutoff: *cutoff,
                    }),
                    Err(e) => Err(format!("解析重力模型 {} 失败: {e}", model_path)),
                },
                // 重力模型文件仅用于 N 体传播，缺失时不应阻断历表位置加载；
                // 回退到点质量模型（可视化只需位置，与此无关）。
                Err(_) => {
                    eprintln!(
                        "Note: gravity model {} not found, falling back to point mass",
                        path.display()
                    );
                    Ok(GravityModel::PointMass)
                }
            }
        }
    }
}

// ─── Path helpers ───

fn find_vsop_path(orbiter_src: &Path, dat_file: &str) -> std::path::PathBuf {
    orbiter_src
        .join("Src")
        .join("Celbody")
        .join("Vsop87")
        .join("Data")
        .join(dat_file)
}

fn find_elp_path(orbiter_src: &Path, dat_file: &str) -> std::path::PathBuf {
    // Try direct location first (Moon/ELP82.dat)
    let primary = orbiter_src
        .join("Src")
        .join("Celbody")
        .join("Moon")
        .join(dat_file);
    if primary.exists() {
        return primary;
    }
    // Fallback: nested Config/Moon/Data/ structure
    orbiter_src
        .join("Src")
        .join("Celbody")
        .join("Moon")
        .join("Config")
        .join("Moon")
        .join("Data")
        .join(dat_file)
}

fn find_galsat_path(orbiter_src: &Path, dat_file: &str) -> std::path::PathBuf {
    // ephem_e15.dat is directly in Galsat/, not Galsat/Data/
    let primary = orbiter_src
        .join("Src")
        .join("Celbody")
        .join("Galsat")
        .join(dat_file);
    if primary.exists() {
        return primary;
    }
    // Fallback: some builds use Galsat/Data/
    orbiter_src
        .join("Src")
        .join("Celbody")
        .join("Galsat")
        .join("Data")
        .join(dat_file)
}

fn find_tass_path(orbiter_src: &Path, dat_file: &str) -> std::path::PathBuf {
    // tass17.dat is directly in Satsat/, not Satsat/Data/
    let primary = orbiter_src
        .join("Src")
        .join("Celbody")
        .join("Satsat")
        .join(dat_file);
    if primary.exists() {
        return primary;
    }
    // Fallback: some builds use Satsat/Data/
    orbiter_src
        .join("Src")
        .join("Celbody")
        .join("Satsat")
        .join("Data")
        .join(dat_file)
}

fn find_gravity_model_path(orbiter_src: &Path, model_path: &str) -> std::path::PathBuf {
    orbiter_src.join("GravityModels").join(model_path)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn celestials_sorted_by_mass() {
        // Build a minimal system config without ephemeris.
        let config = orbitx_config::SystemConfig {
            name: "Test".to_string(),
            star: "Sun".to_string(),
            bodies: vec![
                orbitx_config::BodyConfig {
                    name: "Sun".to_string(),
                    mass: 1.989e30,
                    size: 6.96e8,
                    ephemeris: None,
                    rotation: None,
                    gravity: None,
                    atmosphere: None,
                    color: [1.0, 1.0, 1.0, 1.0],
                    min_render_radius: 1.0,
                },
                orbitx_config::BodyConfig {
                    name: "Earth".to_string(),
                    mass: 5.97e24,
                    size: 6.371e6,
                    ephemeris: None,
                    rotation: None,
                    gravity: None,
                    atmosphere: None,
                    color: [0.3, 0.6, 1.0, 1.0],
                    min_render_radius: 0.27,
                },
                orbitx_config::BodyConfig {
                    name: "Moon".to_string(),
                    mass: 7.35e22,
                    size: 1.74e6,
                    ephemeris: None,
                    rotation: None,
                    gravity: None,
                    atmosphere: None,
                    color: [0.7, 0.7, 0.7, 1.0],
                    min_render_radius: 0.08,
                },
            ],
            parents: vec![("Moon".to_string(), "Earth".to_string())],
        };

        let psys = PlanetarySystem::from_config(&config, Path::new("/nonexistent")).unwrap();
        // celestials should be sorted by mass descending: Sun, Earth, Moon.
        assert_eq!(psys.celestials.len(), 3);
        assert!(psys.bodies[psys.celestials[0]].mass > psys.bodies[psys.celestials[1]].mass);
        assert!(psys.bodies[psys.celestials[1]].mass > psys.bodies[psys.celestials[2]].mass);
    }

    #[test]
    fn gacc_point_mass_only() {
        let config = orbitx_config::SystemConfig {
            name: "Test".to_string(),
            star: "Sun".to_string(),
            bodies: vec![
                orbitx_config::BodyConfig {
                    name: "Earth".to_string(),
                    mass: 5.97e24,
                    size: 6.371e6,
                    ephemeris: None,
                    rotation: None,
                    gravity: None,
                    atmosphere: None,
                    color: [0.3, 0.6, 1.0, 1.0],
                    min_render_radius: 0.27,
                },
            ],
            parents: vec![],
        };

        let mut psys = PlanetarySystem::from_config(&config, Path::new("/nonexistent")).unwrap();
        psys.bodies[0].pos = Vec3::ZERO;

        let gpos = Vec3::new(7.0e6, 0.0, 0.0);
        let acc = psys.gacc(gpos, None);
        // Should point toward origin (negative x).
        assert!(acc.x < 0.0, "acc.x = {} should be negative", acc.x);
    }

    #[test]
    fn gacc_with_jcoeff() {
        let config = orbitx_config::SystemConfig {
            name: "Test".to_string(),
            star: "Earth".to_string(),
            bodies: vec![orbitx_config::BodyConfig {
                name: "Earth".to_string(),
                mass: 5.97e24,
                size: 6.371e6,
                ephemeris: None,
                rotation: None,
                gravity: Some(orbitx_config::GravityConfig::Jcoeff {
                    values: vec![1.0826e-3],
                }),
                atmosphere: None,
                color: [0.3, 0.6, 1.0, 1.0],
                min_render_radius: 0.27,
            }],
            parents: vec![],
        };

        let mut psys = PlanetarySystem::from_config(&config, Path::new("/nonexistent")).unwrap();
        psys.bodies[0].pos = Vec3::ZERO;

        // Off-equator: J2 perturbation should add a y-component.
        let gpos = Vec3::new(7.0e6, 3.0e6, 0.0);
        let acc = psys.gacc(gpos, None);
        assert!(acc.y.abs() > 0.001, "J2 perturbation acc.y = {}", acc.y);
    }

    #[test]
    fn to_grav_bodies_backward_compat() {
        let config = orbitx_config::SystemConfig {
            name: "Test".to_string(),
            star: "Earth".to_string(),
            bodies: vec![orbitx_config::BodyConfig {
                name: "Earth".to_string(),
                mass: 5.97e24,
                size: 6.371e6,
                ephemeris: None,
                rotation: None,
                gravity: None,
                atmosphere: None,
                color: [0.3, 0.6, 1.0, 1.0],
                min_render_radius: 0.27,
            }],
            parents: vec![],
        };

        let mut psys = PlanetarySystem::from_config(&config, Path::new("/nonexistent")).unwrap();
        psys.bodies[0].pos = Vec3::new(1.0e11, 0.0, 0.0);

        let grav_bodies = psys.to_grav_bodies();
        assert_eq!(grav_bodies.len(), 1);
        assert!((grav_bodies[0].pos.x - 1.0e11).abs() < 1.0);
        assert!((grav_bodies[0].mass - 5.97e24).abs() < 1e10);
    }
}
