//! 把引力体平移到主天体原点，并给出主天体表面参数。

use orbitx_config::AtmosphereConfig;
use orbitx_dynamics::GravBody;

use crate::body::{CelestialBody, GravityModel};
use orbitx_math::vec3::Vec3;

/// 主天体表面：每 tick 写入 Assembly。
#[derive(Clone, Debug)]
pub struct PrimarySurface {
    pub radius: f64,
    pub mass: f64,
    /// 自转关闭时为 0。
    pub sid_rot_period: f64,
    pub atmosphere: Option<AtmosphereConfig>,
}

pub(crate) fn grav_bodies(bodies: &[CelestialBody], primary_idx: usize) -> Vec<GravBody> {
    let origin = bodies.get(primary_idx).map(|b| b.pos).unwrap_or(Vec3::ZERO);
    bodies
        .iter()
        .filter(|b| b.dynamics && b.mass > 0.0)
        .map(|b| GravBody {
            pos: b.pos - origin,
            mass: b.mass,
            size: b.size,
            jcoeff: match &b.gravity {
                GravityModel::Jcoeff { values } => values.clone(),
                _ => vec![],
            },
            rotation: Some(b.rot_matrix()),
            pines: match &b.gravity {
                GravityModel::Pines { model, cutoff } => Some((model.clone(), *cutoff)),
                _ => None,
            },
        })
        .collect()
}

pub(crate) fn primary_grav_index(bodies: &[CelestialBody], primary_idx: usize) -> usize {
    bodies
        .iter()
        .take(primary_idx)
        .filter(|b| b.dynamics && b.mass > 0.0)
        .count()
}

pub(crate) fn primary_surface(bodies: &[CelestialBody], primary_idx: usize) -> PrimarySurface {
    let body = &bodies[primary_idx];
    PrimarySurface {
        radius: body.size,
        mass: body.mass,
        sid_rot_period: body.sid_rot_period(),
        atmosphere: body.atmosphere.clone(),
    }
}

#[cfg(test)]
mod tests;
