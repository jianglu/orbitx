//! Rigid-body state bundle — mirrors `class StateVectors` (Vecmat.h:411-431).

use crate::mat3::Matrix3;
use crate::quat::Quat;
use crate::vec3::Vec3;

/// Dual-buffered rigid-body state (`class StateVectors`, Vecmat.h:411).
///
/// Bundles the linear state (position, velocity), the rotational state
/// (quaternion `Q`, rotation matrix `R`, angular velocity `omega`) into a
/// single struct. `R` is kept in sync with `Q` as a derived/cached value.
#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct StateVectors {
    pub pos: Vec3,
    pub vel: Vec3,
    pub omega: Vec3,
    pub r: Matrix3,
    pub q: Quat,
}

impl StateVectors {
    /// Copy all fields from another (`Set(StateVectors)`, Vecmat.cpp:696).
    #[inline]
    pub fn set(&mut self, s: StateVectors) {
        self.pos = s.pos;
        self.vel = s.vel;
        self.omega = s.omega;
        self.r = s.r;
        self.q = s.q;
    }

    /// Set from explicit linear/angular state (`Set(v,p,av,ap)`, Vecmat.cpp:705).
    /// `R` is derived from the quaternion argument `ap`.
    #[inline]
    pub fn set_from(&mut self, vel: Vec3, pos: Vec3, omega: Vec3, q: Quat) {
        self.vel = vel;
        self.pos = pos;
        self.omega = omega;
        self.q = q;
        self.r = Matrix3::from_quat(q);
    }

    /// Set rotation from a matrix (`SetRot(Matrix)`, Vecmat.cpp:714).
    #[inline]
    pub fn set_rot_matrix(&mut self, r: Matrix3) {
        self.r = r;
        self.q = Quat::from_matrix(r);
    }

    /// Set rotation from a quaternion (`SetRot(Quaternion)`, Vecmat.cpp:720).
    #[inline]
    pub fn set_rot_quat(&mut self, q: Quat) {
        self.q = q;
        self.r = Matrix3::from_quat(q);
    }

    /// Advance the state by one integration substep (`Advance`, Vecmat.cpp:726).
    ///
    /// Note: `pos += v*dt` uses the **passed-in** velocity `v`, not `self.vel`;
    /// `Q.Rotate(av*dt)` uses the passed-in angular velocity `av`. This mirrors
    /// the C++ exactly and matters for multi-stage integrators (RK4 etc.).
    pub fn advance(&mut self, dt: f64, a: Vec3, v: Vec3, aa: Vec3, av: Vec3) {
        self.vel += a * dt;
        self.pos += v * dt;
        self.omega += aa * dt;
        self.q.rotate(av * dt);
        self.r = Matrix3::from_quat(self.q);
    }
}

impl Default for StateVectors {
    fn default() -> Self {
        Self {
            pos: Vec3::ZERO,
            vel: Vec3::ZERO,
            omega: Vec3::ZERO,
            r: Matrix3::IDENTITY,
            q: Quat::IDENTITY,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn advance_drift() {
        let mut s = StateVectors {
            pos: Vec3::ZERO,
            vel: Vec3::new(1.0, 0.0, 0.0),
            ..StateVectors::default()
        };
        // dt=1, a=0, v=self.vel, aa=0, av=0 → pos += vel*1
        s.advance(1.0, Vec3::ZERO, s.vel, Vec3::ZERO, Vec3::ZERO);
        assert!((s.pos - Vec3::new(1.0, 0.0, 0.0)).length() < 1e-12);
    }
}
