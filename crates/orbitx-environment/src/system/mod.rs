//! 运行中的行星系统。

use std::path::Path;

use orbitx_config::PlanetaryScenario;
use orbitx_dynamics::gravity::{jcoeff_perturbation_with_rot, pines_perturbation};
use orbitx_dynamics::GravBody;
use orbitx_math::vec3::Vec3;

use crate::body::{load_body, CelestialBody, GravityModel};
use crate::frame::{self, PrimarySurface};

const MJD_J2000: f64 = 51544.5;

/// 场景天体表。渲染读 `bodies`，力学读 `grav_bodies`。
pub struct PlanetarySystem {
    bodies: Vec<CelestialBody>,
    /// `dynamics` 且质量大于 0 的下标，按质量降序。供绝对坐标系 `gacc`。
    celestials: Vec<usize>,
    pub mjd: f64,
    primary_idx: usize,
    star: String,
}

impl PlanetarySystem {
    pub fn load(scenario: &PlanetaryScenario, data_root: &Path) -> Result<Self, String> {
        let primary_idx = scenario
            .bodies
            .iter()
            .position(|b| b.name == scenario.primary)
            .ok_or_else(|| format!("环境主天体 `{}` 不在天体表里", scenario.primary))?;
        if !scenario.bodies[primary_idx].dynamics {
            return Err(format!(
                "主天体 `{}` 的 dynamics 必须为 true",
                scenario.primary
            ));
        }

        let mut bodies = Vec::with_capacity(scenario.bodies.len());
        for body_cfg in &scenario.bodies {
            let parent_idx = scenario
                .parents
                .iter()
                .find(|(c, _)| c == &body_cfg.name)
                .and_then(|(_, p)| scenario.bodies.iter().position(|b| b.name == *p));
            bodies.push(load_body(
                &body_cfg.name,
                body_cfg.mass,
                body_cfg.size,
                body_cfg.dynamics,
                body_cfg.fixed_pos,
                &body_cfg.ephemeris,
                &body_cfg.rotation,
                &body_cfg.gravity,
                &body_cfg.atmosphere,
                body_cfg.color,
                body_cfg.min_render_radius,
                parent_idx,
                data_root,
            )?);
        }

        let mut celestials: Vec<usize> = (0..bodies.len())
            .filter(|&i| bodies[i].dynamics && bodies[i].mass > 0.0)
            .collect();
        celestials.sort_by(|&a, &b| bodies[b].mass.partial_cmp(&bodies[a].mass).unwrap());

        let mut psys = PlanetarySystem {
            bodies,
            celestials,
            mjd: scenario.mjd,
            primary_idx,
            star: scenario.star.clone(),
        };
        psys.update();
        Ok(psys)
    }

    pub fn bodies(&self) -> &[CelestialBody] {
        &self.bodies
    }

    pub fn bodies_mut(&mut self) -> &mut [CelestialBody] {
        &mut self.bodies
    }

    pub fn star(&self) -> &str {
        &self.star
    }

    pub fn body_index(&self, name: &str) -> Option<usize> {
        self.bodies.iter().position(|b| b.name == name)
    }

    /// 自转 + 有历表的天体推位。`fixed_pos` 不动（相对父体时跟随父体）。
    pub fn update(&mut self) {
        let mut eph_pos: Vec<Option<[f64; 6]>> = vec![None; self.bodies.len()];
        for (i, body) in self.bodies.iter_mut().enumerate() {
            if let Some(ref mut eph) = body.ephemeris {
                eph_pos[i] = Some(eph.eval(self.mjd));
            }
        }

        let mut new_positions: Vec<Vec3> = self.bodies.iter().map(|b| b.pos).collect();
        for (i, body) in self.bodies.iter().enumerate() {
            if let Some(pos_vel) = eph_pos[i] {
                let local = Vec3::new(pos_vel[0], pos_vel[1], pos_vel[2]);
                new_positions[i] = match body.parent_idx {
                    Some(parent_idx) => new_positions[parent_idx] + local,
                    None => local,
                };
            } else if let Some(fixed) = body.fixed_pos {
                new_positions[i] = match body.parent_idx {
                    Some(parent_idx) => new_positions[parent_idx] + fixed,
                    None => fixed,
                };
            }
        }
        for (i, body) in self.bodies.iter_mut().enumerate() {
            body.pos = new_positions[i];
        }

        for body in &mut self.bodies {
            if !body.rotation_enabled {
                continue;
            }
            if let Some(ref mut rot) = body.rotation {
                let sim_t = (self.mjd - MJD_J2000) * 86400.0;
                rot.update_precession(self.mjd);
                rot.update_rotation(sim_t);
            }
        }
    }

    pub fn advance(&mut self, dt_days: f64) {
        self.mjd += dt_days;
    }

    /// 仅 `dynamics = true`，平移到主天体原点。
    pub fn grav_bodies(&self) -> Vec<GravBody> {
        frame::grav_bodies(&self.bodies, self.primary_idx)
    }

    pub fn primary_grav_index(&self) -> usize {
        frame::primary_grav_index(&self.bodies, self.primary_idx)
    }

    pub fn primary_surface(&self) -> PrimarySurface {
        frame::primary_surface(&self.bodies, self.primary_idx)
    }

    /// 绝对坐标系下、只累加力学天体的加速度。单测与对照用。
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
            acc += rpos * (body.gm() / (d * d * d));
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
}

#[cfg(test)]
mod tests;
