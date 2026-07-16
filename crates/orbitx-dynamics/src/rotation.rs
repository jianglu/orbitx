//! Planetary rotation and precession model.
//!
//! Mirrors `CelestialBody::UpdatePrecession()` and `UpdateRotation()` /
//! `GetRotation()` from `Celbody.cpp:493-548`.
//!
//! # Algorithm
//!
//! 1. **Precession** (`update_precession`): Computes the tilt of the rotation
//!    axis over time due to precession. Produces the precession matrix `R_ecl`
//!    and the rotation offset `rotation_off`.
//!
//! 2. **Rotation** (`update_rotation`): Computes the body's rotation angle
//!    around its local axis, then builds the full rotation matrix
//!    `rot_matrix = R_ecl * R_rot`.
//!
//! # Coordinate convention
//!
//! Matches Orbiter's left-handed ecliptic frame:
//! - y-axis = ecliptic north pole
//! - Rotation axis is tilted from y by the obliquity

use orbitx_math::mat3::{self, Matrix3};
use orbitx_math::vec3::Vec3;

/// Two-argument atan2 that returns a value in [0, 2π).
/// Mirrors Orbiter's `posangle` macro.
fn posangle(y: f64, x: f64) -> f64 {
    let a = y.atan2(x);
    if a < 0.0 { a + 2.0 * std::f64::consts::PI } else { a }
}

/// Planetary rotation state: precession + sidereal rotation.
///
/// Ported from `CelestialBody` (Celbody.cpp:493-548).
pub struct RotationState {
    // ─── Configuration (immutable after construction) ───
    /// Sidereal rotation period [s].
    rot_T: f64,
    /// Sidereal angular velocity = 2π / rot_T [rad/s].
    rot_omega: f64,
    /// Rotation offset at t=0 [rad].
    dphi: f64,
    /// Obliquity relative to reference axis [rad].
    eps_rel: f64,
    /// Ascending node longitude at mjd_rel [rad].
    lrel0: f64,
    /// Reference MJD for LAN.
    mjd_rel: f64,
    /// Precession period [days] (0 = no precession).
    prec_T: f64,
    /// Precession angular velocity = 2π / prec_T [rad/day].
    prec_omega: f64,
    /// Precession obliquity [rad].
    eps_ref: f64,
    /// Precession LAN [rad].
    lan_ref: f64,

    // ─── Precession reference matrix (computed once from eps_ref, lan_ref) ───
    /// R_ref: ecliptic normal → precession reference axis.
    /// Only non-identity if eps_ref != 0.
    r_ref: Matrix3,

    // ─── Runtime state (updated by update_precession / update_rotation) ───
    /// Current LAN relative to reference axis [rad].
    lrel: f64,
    /// sin(eps_rel).
    sin_eps: f64,
    /// cos(eps_rel).
    cos_eps: f64,
    /// Rotation axis direction in global coords.
    r_axis: Vec3,
    /// Obliquity against ecliptic [rad].
    eps_ecl: f64,
    /// LAN against ecliptic [rad].
    lan_ecl: f64,
    /// Precession matrix (axis tilt).
    r_ecl: Matrix3,
    /// Rotation offset from precession [rad].
    rotation_off: f64,
    /// Current rotation angle [rad].
    rotation: f64,
    /// Full rotation matrix = R_ecl * R_rot.
    rot_matrix: Matrix3,
}

impl RotationState {
    /// Construct from a `RotationConfig`.
    ///
    /// Initialises precession at J2000 (MJD 51544.5) and rotation at t=0.
    pub fn from_config(cfg: &orbitx_config::RotationConfig) -> Self {
        let rot_omega = 2.0 * std::f64::consts::PI / cfg.sid_rot_period;
        let prec_omega = if cfg.precession_period != 0.0 {
            2.0 * std::f64::consts::PI / cfg.precession_period
        } else {
            0.0
        };

        // Build R_ref from precession reference parameters.
        let r_ref = if cfg.precession_obliquity != 0.0 {
            // R_ref rotates from ecliptic frame to precession reference frame.
            // In Orbiter, this is computed from eps_ref and lan_ref.
            // For simplicity, we use a rotation about the y-axis by eps_ref,
            // then about z by lan_ref.
            let ce = cfg.precession_obliquity.cos();
            let se = cfg.precession_obliquity.sin();
            let cl = cfg.precession_lan.cos();
            let sl = cfg.precession_lan.sin();
            Matrix3::new(
                cl, -sl * ce, -sl * se,
                0.0,      ce,     -se,
                sl,  cl * ce,  cl * se,
            )
        } else {
            Matrix3::IDENTITY
        };

        let sin_eps = cfg.obliquity.sin();
        let cos_eps = cfg.obliquity.cos();

        let mut state = Self {
            rot_T: cfg.sid_rot_period,
            rot_omega,
            dphi: cfg.sid_rot_offset,
            eps_rel: cfg.obliquity,
            lrel0: cfg.lan,
            mjd_rel: cfg.lan_mjd,
            prec_T: cfg.precession_period,
            prec_omega,
            eps_ref: cfg.precession_obliquity,
            lan_ref: cfg.precession_lan,
            r_ref,
            lrel: cfg.lan,
            sin_eps,
            cos_eps,
            r_axis: Vec3::new(0.0, 1.0, 0.0),
            eps_ecl: cfg.obliquity,
            lan_ecl: cfg.lan,
            r_ecl: Matrix3::IDENTITY,
            rotation_off: 0.0,
            rotation: 0.0,
            rot_matrix: Matrix3::IDENTITY,
        };

        // Initialise precession at J2000.
        state.update_precession(cfg.lan_mjd);
        // Initialise rotation at t=0.
        state.update_rotation(0.0);

        state
    }

    /// Update precession state for a given MJD.
    ///
    /// Mirrors `CelestialBody::UpdatePrecession()` (Celbody.cpp:493-518).
    pub fn update_precession(&mut self, mjd: f64) {
        // Lrel = Lrel0 + prec_omega * (MJD - mjd_rel)
        self.lrel = self.lrel0 + self.prec_omega * (mjd - self.mjd_rel);
        let sinl = self.lrel.sin();
        let cosl = self.lrel.cos();

        // R_ref_rel = R_rel(Lrel, eps_rel)
        // This is the rotation from ecliptic to body frame considering
        // the obliquity and LAN.
        let mut r_ref_rel = Matrix3::new(
            cosl, -sinl * self.sin_eps, -sinl * self.cos_eps,
               0.0,        self.cos_eps,       -self.sin_eps,
            sinl,  cosl * self.sin_eps,  cosl * self.cos_eps,
        );

        // Apply precession reference tilt if non-trivial.
        if self.eps_ref != 0.0 {
            r_ref_rel.premul(self.r_ref);
        }

        // Rotation axis direction in global coords.
        self.r_axis = mat3::mul(r_ref_rel, Vec3::new(0.0, 1.0, 0.0));

        // Axis obliquity and LAN against ecliptic.
        self.eps_ecl = self.r_axis.y.acos();
        self.lan_ecl = (-self.r_axis.x).atan2(self.r_axis.z);

        let sinL = self.lan_ecl.sin();
        let cosL = self.lan_ecl.cos();
        let sine = self.eps_ecl.sin();
        let cose = self.eps_ecl.cos();

        // Precession matrix R_ecl.
        self.r_ecl = Matrix3::new(
            cosL, -sinL * sine, -sinL * cose,
              0.0,        cose,       -sine,
            sinL,  cosL * sine,  cosL * cose,
        );

        // Rotation offset from precession.
        let cos_poff = cosL * r_ref_rel.m11 + sinL * r_ref_rel.m31;
        let sin_poff = -(cosL * r_ref_rel.m13 + sinL * r_ref_rel.m33);
        self.rotation_off = sin_poff.atan2(cos_poff);
    }

    /// Update rotation state for a given simulation time [s].
    ///
    /// Mirrors `CelestialBody::UpdateRotation()` (Celbody.cpp:521-534).
    pub fn update_rotation(&mut self, sim_t: f64) {
        // Rotation angle around local y-axis.
        self.rotation = posangle(
            (self.dphi + sim_t * self.rot_omega - self.lrel * self.cos_eps + self.rotation_off).sin(),
            (self.dphi + sim_t * self.rot_omega - self.lrel * self.cos_eps + self.rotation_off).cos(),
        );

        let cosr = self.rotation.cos();
        let sinr = self.rotation.sin();

        // Rotation about local y-axis.
        let r_rot = Matrix3::new(
            cosr, 0.0, -sinr,
             0.0, 1.0,   0.0,
            sinr, 0.0,  cosr,
        );

        // Full rotation matrix = R_ecl * R_rot (premul = R_ecl * r_rot).
        self.rot_matrix = self.r_ecl.matmul(r_rot);
    }

    /// Get rotation matrix at an arbitrary simulation time [s].
    ///
    /// Mirrors `CelestialBody::GetRotation(t)` (Celbody.cpp:537-548).
    /// Note: assumes current precession state is valid (mjd sufficiently close).
    pub fn get_rotation(&self, t: f64) -> Matrix3 {
        let angle = self.dphi + t * self.rot_omega - self.lrel * self.cos_eps + self.rotation_off;
        let cosr = angle.cos();
        let sinr = angle.sin();

        let r_rot = Matrix3::new(
            cosr, 0.0, -sinr,
             0.0, 1.0,   0.0,
            sinr, 0.0,  cosr,
        );

        self.r_ecl.matmul(r_rot)
    }

    /// Return the current full rotation matrix.
    pub fn rot_matrix(&self) -> &Matrix3 {
        &self.rot_matrix
    }

    /// Return the rotation axis direction in global coords.
    pub fn r_axis(&self) -> Vec3 {
        self.r_axis
    }

    /// Return the current obliquity against ecliptic [rad].
    pub fn eps_ecl(&self) -> f64 {
        self.eps_ecl
    }

    /// Return the current rotation angle [rad].
    pub fn rotation(&self) -> f64 {
        self.rotation
    }

    /// Return the sidereal rotation period [s].
    pub fn sid_rot_period(&self) -> f64 {
        self.rot_T
    }

    /// Return the sidereal angular velocity [rad/s].
    pub fn rot_omega(&self) -> f64 {
        self.rot_omega
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn earth_rotation_config() -> orbitx_config::RotationConfig {
        orbitx_config::RotationConfig {
            sid_rot_period: 86164.10132,
            sid_rot_offset: 4.88948754,
            obliquity: 0.4090928023,
            lan: 0.0,
            lan_mjd: 51544.5,
            precession_period: -9413040.4,
            precession_obliquity: 0.0,
            precession_lan: 0.0,
        }
    }

    fn moon_rotation_config() -> orbitx_config::RotationConfig {
        orbitx_config::RotationConfig {
            sid_rot_period: 2360588.15,
            sid_rot_offset: 4.769465382,
            obliquity: 0.02692416821,
            lan: 1.71817749,
            lan_mjd: 51544.5,
            precession_period: -6793.219721,
            precession_obliquity: 7.259562816e-5,
            precession_lan: 0.4643456618,
        }
    }

    #[test]
    fn earth_rotation_period() {
        let cfg = earth_rotation_config();
        let state = RotationState::from_config(&cfg);
        assert!(
            (state.sid_rot_period() - 86164.10132).abs() < 1e-4,
            "rot_T = {}",
            state.sid_rot_period()
        );
    }

    #[test]
    fn earth_rotation_angle_advances() {
        let cfg = earth_rotation_config();
        let mut state = RotationState::from_config(&cfg);
        let rot0 = state.rotation();
        // Advance by 1/4 rotation period.
        state.update_rotation(state.sid_rot_period() / 4.0);
        let rot1 = state.rotation();
        // Rotation angle should have advanced by approximately π/2.
        let delta = rot1 - rot0;
        // Handle wrap-around.
        let delta = if delta < 0.0 { delta + 2.0 * std::f64::consts::PI } else { delta };
        assert!(
            (delta - std::f64::consts::FRAC_PI_2).abs() < 0.01,
            "delta = {} rad, expected π/2",
            delta
        );
    }

    #[test]
    fn earth_obliquity_correct() {
        let cfg = earth_rotation_config();
        let state = RotationState::from_config(&cfg);
        // R_axis should be tilted from y-axis by ~23.44°.
        let expected_deg = 23.439291; // from Earth.cfg comments
        let actual_deg = state.eps_ecl().to_degrees();
        assert!(
            (actual_deg - expected_deg).abs() < 0.01,
            "obliquity = {}°, expected ~{}°",
            actual_deg,
            expected_deg
        );
    }

    #[test]
    fn moon_obliquity_correct() {
        let cfg = moon_rotation_config();
        let state = RotationState::from_config(&cfg);
        // Moon obliquity is small (~1.54°).
        let actual_deg = state.eps_ecl().to_degrees();
        assert!(
            actual_deg < 5.0,
            "Moon obliquity = {}°, should be small",
            actual_deg
        );
    }

    #[test]
    fn no_precession_simplifies() {
        // With prec_T = 0, R_ecl should only contain the obliquity tilt.
        let cfg = orbitx_config::RotationConfig {
            sid_rot_period: 86400.0,
            sid_rot_offset: 0.0,
            obliquity: 0.4,
            lan: 0.0,
            lan_mjd: 51544.5,
            precession_period: 0.0,
            precession_obliquity: 0.0,
            precession_lan: 0.0,
        };
        let state = RotationState::from_config(&cfg);
        // With lan=0 and no precession, R_ecl should be a simple tilt about x-axis.
        let _r = state.rot_matrix();
        // The rotation axis should be tilted from y by obliquity.
        let axis = state.r_axis();
        let angle_from_y = axis.y.acos();
        assert!(
            (angle_from_y - 0.4).abs() < 1e-10,
            "angle from y = {}, expected 0.4",
            angle_from_y
        );
    }

    #[test]
    fn get_rotation_matches_update() {
        let cfg = earth_rotation_config();
        let mut state = RotationState::from_config(&cfg);
        let t = 3600.0; // 1 hour
        state.update_rotation(t);
        let m_update = *state.rot_matrix();
        let m_get = state.get_rotation(t);
        // Should match within floating-point tolerance.
        for i in 0..3 {
            for j in 0..3 {
                let a = m_update.get(i, j);
                let b = m_get.get(i, j);
                assert!(
                    (a - b).abs() < 1e-12,
                    "R[{},{}] mismatch: update={}, get={}",
                    i, j, a, b
                );
            }
        }
    }

    #[test]
    fn rotation_matrix_orthonormal() {
        let cfg = earth_rotation_config();
        let state = RotationState::from_config(&cfg);
        let r = state.rot_matrix();
        // R * R^T should be identity.
        let rr_t = r.matmul(r.transp());
        let eps = 1e-10;
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                let actual = rr_t.get(i, j);
                assert!(
                    (actual - expected).abs() < eps,
                    "R*R^T[{},{}] = {}, expected {}",
                    i, j, actual, expected
                );
            }
        }
    }

    #[test]
    fn jupiter_rotation() {
        let cfg = orbitx_config::RotationConfig {
            sid_rot_period: 13500.3,
            sid_rot_offset: 2.547801285,
            obliquity: 0.05443758224,
            lan: 3.782814532,
            lan_mjd: 51544.5,
            precession_period: -307703725.6,
            precession_obliquity: 0.02276340837,
            precession_lan: 4.89539507,
        };
        let state = RotationState::from_config(&cfg);
        // Jupiter's obliquity is ~3.12°.
        let actual_deg = state.eps_ecl().to_degrees();
        assert!(
            actual_deg < 10.0,
            "Jupiter obliquity = {}°, should be small",
            actual_deg
        );
    }
}
