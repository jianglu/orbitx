//! Dynamics system for orbitx: numerical integrators, N-body gravity, Pines
//! spherical-harmonic gravity, and Kepler orbit solver.
//!
//! Mirrors Orbiter's rigid-body dynamics:
//! - Integrators (`BodyIntegrator.cpp`): RK2-RK8 (Runge-Kutta), SY2-SY8 (Yoshida
//!   symplectic)
//! - Gravity (`Psys.cpp`): N-body point-mass summation, J2/J3/J4 zonal harmonics
//! - Pines (`PinesGrav.cpp`): spherical-harmonic acceleration via normalized
//!   associated Legendre functions
//! - Kepler (`Element.cpp`): classical orbital elements, Kepler equation solver,
//!   2-body analytic propagation
//!
//! All algorithms are symbol-for-symbol replicas of the C++ implementation.

#![allow(clippy::approx_constant, clippy::excessive_precision)]

pub mod gravity;
pub mod integrator;
pub mod kepler;
pub mod pines;

pub use gravity::{gacc_nbody, jcoeff_perturbation, single_gacc, GravBody};
pub use integrator::{rk2_step, rk4_step, rk_drv, rk_step, sy_step, ForceFn, RkCoeffs, SyCoeffs};
pub use kepler::Elements;
pub use pines::PinesModel;
