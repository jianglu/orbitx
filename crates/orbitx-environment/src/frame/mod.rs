//! 把引力体平移到主天体原点，并给出主天体表面参数。

use std::sync::Arc;

use orbitx_config::AtmosphereConfig;
use orbitx_dynamics::{GravBody, PlanetOrientation};
use orbitx_surface::SurfacePackage;

use crate::body::{CelestialBody, GravityModel};
use crate::surface::SurfaceBinding;
use orbitx_math::vec3::Vec3;

/// 主天体表面：每 tick 写入 Assembly。
///
/// G4：可选地表包绑定后提供 `contact_height` / `ground_radius`（CPU 转发
/// `orbitx-surface`）。无绑定时 `contact_height ≡ 0`，`elev_radius_m = radius`
///（光滑球，与 P4.4 行为一致）。**不**改变 Runtime 坠毁判据（P5.1）。
#[derive(Clone)]
pub struct PrimarySurface {
    pub radius: f64,
    pub mass: f64,
    /// 自转关闭时为 0（遥测/诊断；速度与采样以 [`Self::orientation`] 为准）。
    pub sid_rot_period: f64,
    /// 体固→惯性姿态 + 惯性系角速度（与引力 `GravBody.rotation` 同源）。
    pub orientation: PlanetOrientation,
    pub atmosphere: Option<AtmosphereConfig>,
    /// Elevation reference sphere (manifest `R_elev_m`, or `radius` if unbound).
    pub elev_radius_m: f64,
    /// Read-only ocean MSL constant from package (0 if unbound).
    pub ocean_surface_m: f64,
    package: Option<Arc<SurfacePackage>>,
}

impl std::fmt::Debug for PrimarySurface {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PrimarySurface")
            .field("radius", &self.radius)
            .field("mass", &self.mass)
            .field("sid_rot_period", &self.sid_rot_period)
            .field("orientation", &self.orientation)
            .field("elev_radius_m", &self.elev_radius_m)
            .field("ocean_surface_m", &self.ocean_surface_m)
            .field("has_package", &self.package.is_some())
            .finish()
    }
}

impl PrimarySurface {
    /// §3.1 contact height (m, MSL). Unbound → 0.
    pub fn contact_height(&self, lat_deg: f64, lon_deg: f64) -> f64 {
        match &self.package {
            Some(p) => p.sample_contact_height(lat_deg, lon_deg),
            None => 0.0,
        }
    }

    /// Geocentric ground radius = `elev_radius_m + contact_height`.
    pub fn ground_radius(&self, lat_deg: f64, lon_deg: f64) -> f64 {
        self.elev_radius_m + self.contact_height(lat_deg, lon_deg)
    }

    pub fn package(&self) -> Option<&Arc<SurfacePackage>> {
        self.package.as_ref()
    }
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

pub(crate) fn primary_surface(
    bodies: &[CelestialBody],
    primary_idx: usize,
    binding: Option<&SurfaceBinding>,
) -> PrimarySurface {
    let body = &bodies[primary_idx];
    let (elev_radius_m, ocean_surface_m, package) = match binding {
        Some(b) => (
            b.package.r_elev_m,
            b.package.ocean_surface_m,
            Some(Arc::clone(&b.package)),
        ),
        None => (body.size, 0.0, None),
    };
    PrimarySurface {
        radius: body.size,
        mass: body.mass,
        sid_rot_period: body.sid_rot_period(),
        orientation: body.orientation(),
        atmosphere: body.atmosphere.clone(),
        elev_radius_m,
        ocean_surface_m,
        package,
    }
}

#[cfg(test)]
mod tests;
