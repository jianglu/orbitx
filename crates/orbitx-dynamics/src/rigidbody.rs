//! Rigid-body angular dynamics: Euler's equation and gravity-gradient torque.
//!
//! Mirrors Orbiter's `Rigidbody.cpp`. The integrators in [`crate::integrator`]
//! operate on a `StateVectors` bundle that already carries the angular state
//! (`omega`, `q`, `r`). They expect the force closure to return angular
//! *acceleration*, so converting a torque into `dω/dt` — i.e. solving Euler's
//! equation — is the caller's responsibility (see the note in
//! `integrator.rs:14-17`). The free functions here perform that conversion,
//! exactly as Orbiter's `RigidBody::EulerInv_*` member methods do.
//!
//! ## Principal moments of inertia (PMI)
//!
//! Orbiter stores the inertia tensor as a diagonal `Vector pmi` in the vessel
//! body frame (`Rigidbody.h:224`). For a rocket the longitudinal axis is **Y**
//! (`+Y` toward the nose, `-Y` toward the engines), so `pmi.y` is the *axial*
//! moment (small) and `pmi.x = pmi.z` are the *transverse* moments (large).
//!
//! ## Mass-normalised torque convention
//!
//! Orbiter's `Vessel::GetIntermediateMoments` (`Vessel.cpp:910-922`) divides
//! the accumulated torque by mass before returning it:
//!
//! ```text
//! tau += M / mass;
//! ```
//!
//! i.e. the `tau` handed to `EulerInv_*` is a *specific torque* [N·m / kg]. The
//! expressions below are used verbatim; they do **not** divide by mass again.
//!
//! ## Left-handed system
//!
//! The cross-axis coupling signs follow Orbiter's left-handed ecliptic J2000
//! frame (see `orbitx-math/src/lib.rs:8-14`). The formulae are copied
//! symbol-for-symbol from `Rigidbody.cpp:458-511`; do not "correct" the signs.

use orbitx_math::{cross, Vec3};
use orbitx_math::consts::GGRAV;

/// Solve Euler's equation for angular acceleration — full coupled form
/// (`EulerInv_full`, Rigidbody.cpp:468-481).
///
/// Solves the left-handed Euler equation
///
/// ```text
/// I·dω/dt + (Iω) × ω = τ
/// ```
///
/// for `dω/dt`, given specific torque `tau`, angular velocity `omega`, and the
/// diagonal inertia tensor `pmi`.
pub fn euler_inv_full(tau: Vec3, omega: Vec3, pmi: Vec3) -> Vec3 {
    Vec3::new(
        (tau.x - (pmi.y - pmi.z) * omega.y * omega.z) / pmi.x,
        (tau.y - (pmi.z - pmi.x) * omega.z * omega.x) / pmi.y,
        (tau.z - (pmi.x - pmi.y) * omega.x * omega.y) / pmi.z,
    )
}

/// Solve Euler's equation — simplified decoupled form
/// (`EulerInv_simple`, Rigidbody.cpp:485-497).
///
/// Drops the `(Iω) × ω` cross-axis coupling and simply returns `τ / I`. Orbiter
/// uses this at high time-acceleration to avoid the coupling-driven
/// instabilities of the full equation.
pub fn euler_inv_simple(tau: Vec3, pmi: Vec3) -> Vec3 {
    Vec3::new(tau.x / pmi.x, tau.y / pmi.y, tau.z / pmi.z)
}

/// Trivial angular acceleration — returns zero
/// (`EulerInv_zero`, Rigidbody.cpp:501-511).
///
/// Suppresses both the coupling terms and the torque, solving `I·dω/dt = 0`.
/// Used by Orbiter to disable attitude dynamics entirely.
pub fn euler_inv_zero() -> Vec3 {
    Vec3::ZERO
}

/// Forward Euler equation — returns specific torque from angular acceleration
/// (`Euler_full`, Rigidbody.cpp:458-464).
///
/// The inverse of [`euler_inv_full`]: given `omegadot` and `omega`, returns the
/// specific torque `τ` that would produce that angular acceleration. Mainly
/// useful for tests.
pub fn euler_full(omegadot: Vec3, omega: Vec3, pmi: Vec3) -> Vec3 {
    Vec3::new(
        omegadot.x * pmi.x + (pmi.y - pmi.z) * omega.y * omega.z,
        omegadot.y * pmi.y + (pmi.z - pmi.x) * omega.z * omega.x,
        omegadot.z * pmi.z + (pmi.x - pmi.y) * omega.x * omega.y,
    )
}

/// Gravity-gradient torque (mass-normalised) with optional tidal damping
/// (`RigidBody::GetIntermediateMoments` angular part, Rigidbody.cpp:345-363;
/// also `RigidBody::GetTorque`, Rigidbody.cpp:424-447).
///
/// Computes the specific torque exerted on a rigid body by the gravity gradient
/// of a central body of mass `cbody_mass` whose position relative to the vessel
/// is `rel_pos` (in the **global** frame). `pmi` is the body-frame diagonal
/// inertia tensor; `rot` maps body→world (`mul(rot, v)`); `omega` is the
/// current angular velocity (body frame); `tidaldamp` is the dimensionless
/// damping factor (Orbiter's `GravityGradientDamping` config value); `dt` is
/// the current step size used to cap the damping.
///
/// Returns the specific torque [N·m / kg] in the **body** frame, ready to be
/// passed to [`euler_inv_full`]. Returns zero when the gravity-gradient effect
/// is to be suppressed (`b_ignore` set) or `rel_pos` is degenerate.
/// Gravity-gradient torque (mass-normalised) with optional tidal damping
/// (`RigidBody::GetIntermediateMoments` angular part, Rigidbody.cpp:345-363;
/// also `RigidBody::GetTorque`, Rigidbody.cpp:424-447).
///
/// Computes the specific torque exerted on a rigid body by the gravity gradient
/// of a central body of mass `cbody_mass` whose position relative to the vessel
/// is `rel_pos` (in the **global** frame). `pmi` is the body-frame diagonal
/// inertia tensor; `rot` maps body→world (`mul(rot, v)`); `omega` is the
/// current angular velocity (body frame); `tidaldamp` is the dimensionless
/// damping factor (Orbiter's `GravityGradientDamping` config value); `dt` is
/// the current step size used to cap the damping.
///
/// Returns the specific torque [N·m / kg] in the **body** frame, ready to be
/// passed to [`euler_inv_full`]. Returns zero when the gravity-gradient effect
/// is to be suppressed (`b_ignore` set) or `rel_pos` is degenerate.
#[allow(clippy::too_many_arguments)] // 忠实移植 Orbiter 力矩计算的完整输入集
pub fn gravity_gradient_torque(
    rel_pos: Vec3,
    cbody_mass: f64,
    pmi: Vec3,
    rot: orbitx_math::Matrix3,
    omega: Vec3,
    tidaldamp: f64,
    dt: f64,
    b_ignore: bool,
) -> Vec3 {
    if b_ignore {
        return Vec3::ZERO;
    }
    let r0 = rel_pos.length();
    if r0 < 1e-3 {
        return Vec3::ZERO;
    }
    // Map the central body direction into the vessel frame.
    // Rigidbody.cpp:349: R0 = tmul(state.Q, cbody_pos - state.pos)
    let r0_body = orbitx_math::tmul(rot, rel_pos);
    let re = r0_body * (1.0 / r0);
    let mag = 3.0 * GGRAV * cbody_mass / (r0 * r0 * r0);
    let mut tau = cross(pmi * re, re) * mag;

    // Damping of angular velocity (Rigidbody.cpp:356-362).
    if tidaldamp != 0.0 {
        let damp = tidaldamp * mag;
        let scale = damp.min(dt * 0.1);
        if omega.x != 0.0 {
            tau.x -= scale * pmi.x * omega.x;
        }
        if omega.y != 0.0 {
            tau.y -= scale * pmi.y * omega.y;
        }
        if omega.z != 0.0 {
            tau.z -= scale * pmi.z * omega.z;
        }
    }
    tau
}

#[cfg(test)]
mod tests {
    use super::*;
    use orbitx_math::{Matrix3, Vec3};

    /// `EulerInv_full` must invert `Euler_full` exactly.
    #[test]
    fn euler_inv_inverts_full() {
        let pmi = Vec3::new(1e5, 3e4, 1e5);
        let omega = Vec3::new(0.02, -0.01, 0.05);
        let omegadot = Vec3::new(1e-3, 2e-3, -1e-3);
        let tau = euler_full(omegadot, omega, pmi);
        let recovered = euler_inv_full(tau, omega, pmi);
        for (a, b) in [
            (recovered.x, omegadot.x),
            (recovered.y, omegadot.y),
            (recovered.z, omegadot.z),
        ] {
            assert!((a - b).abs() < 1e-9, "{} vs {}", a, b);
        }
    }

    /// No coupling when two PMI components are equal (axisymmetric body): a pure
    /// spin about the symmetry axis must yield zero torque for zero external τ.
    #[test]
    fn axisymmetric_spin_no_coupling() {
        let pmi = Vec3::new(1e5, 3e4, 1e5); // x == z → axisymmetric about Y
        let omega = Vec3::new(0.0, 0.1, 0.0); // pure axial spin
        let tau = Vec3::ZERO;
        let arot = euler_inv_full(tau, omega, pmi);
        assert!(arot.length() < 1e-9, "non-zero α for free axial spin");
    }

    /// `EulerInv_simple` must equal `EulerInv_full` when ω = 0 (no coupling).
    #[test]
    fn simple_equals_full_at_rest() {
        let pmi = Vec3::new(1e5, 3e4, 1e5);
        let tau = Vec3::new(10.0, -5.0, 8.0);
        let full = euler_inv_full(tau, Vec3::ZERO, pmi);
        let simple = euler_inv_simple(tau, pmi);
        assert!((full - simple).length() < 1e-9);
    }

    /// `EulerInv_zero` always returns the zero vector.
    #[test]
    fn zero_is_zero() {
        assert_eq!(euler_inv_zero(), Vec3::ZERO);
    }

    /// Gravity-gradient torque vanishes for a spherically symmetric body
    /// (pmi.x == pmi.y == pmi.z): `cross(pmi*Re, Re) = cross(c*Re, Re) = 0`.
    #[test]
    fn grav_gradient_zero_for_isotropic() {
        let pmi = Vec3::new(1e5, 1e5, 1e5);
        let rel_pos = Vec3::new(6.4e6, 0.0, 0.0);
        let tau = gravity_gradient_torque(
            rel_pos,
            5.972e24,
            pmi,
            Matrix3::IDENTITY,
            Vec3::ZERO,
            0.0,
            1.0,
            false,
        );
        assert!(tau.length() < 1e-6, "isotropic body should have no ggd torque");
    }

    /// A non-isotropic body whose long axis is *not* aligned with the radial
    /// direction experiences a gradient torque that tends to restore alignment
    /// (gravity-gradient stabilisation). With the body tipped 45° between X and
    /// Y, `cross(pmi*Re, Re)` has a non-zero Z component because `pmi.x != pmi.y`.
    #[test]
    fn grav_gradient_nonzero_for_slender_body() {
        let pmi = Vec3::new(1e5, 1e3, 1e5); // slender about Y
        // Central body lies in the XY plane at 45°; with identity rotation the
        // body frame coincides with the global frame, so Re is not along a
        // principal axis and the PMI-weighted vector is no longer parallel to Re.
        let rel_pos = Vec3::new(4.5e6, 4.5e6, 0.0);
        let tau = gravity_gradient_torque(
            rel_pos,
            5.972e24,
            pmi,
            Matrix3::IDENTITY,
            Vec3::ZERO,
            0.0,
            1.0,
            false,
        );
        // Torque should be small but distinctly non-zero, along the body Z axis.
        assert!(tau.length() > 1e-9, "expected non-zero gradient torque, got {:?}", tau);
        // The restoring torque points along ±Z (out of the XY plane).
        assert!(tau.x.abs() < 1e-12 && tau.y.abs() < 1e-12,
            "torque should be along Z, got {:?}", tau);
    }
}
