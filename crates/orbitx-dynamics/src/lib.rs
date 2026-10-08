//! Dynamics system for orbitx: numerical integrators, N-body gravity, Pines
//! spherical-harmonic gravity, and Kepler orbit solver.
//!
//! 纯算法库。行星系统状态与步进在 `orbitx-environment`。
//!
//! Mirrors Orbiter's rigid-body dynamics:
//! - Integrators (`BodyIntegrator.cpp`): RK2-RK8 (Runge-Kutta), SY2-SY8 (Yoshida
//!   symplectic)
//! - Gravity (`Psys.cpp`): N-body point-mass summation, J2/J3/J4 zonal harmonics
//! - Pines (`PinesGrav.cpp`): spherical-harmonic acceleration via normalized
//!   associated Legendre functions
//! - Kepler (`Element.cpp`): classical orbital elements, Kepler equation solver,
//!   2-body analytic propagation
//! - Rigid-body angular dynamics (`Rigidbody.cpp`): Euler's equation solver and
//!   gravity-gradient torque
//!
//! All algorithms are symbol-for-symbol replicas of the C++ implementation.

#![allow(clippy::approx_constant, clippy::excessive_precision)]

pub mod aero;
pub mod atmosphere;
pub mod contact;
pub mod gravity;
pub mod integrator;
pub mod kepler;
pub mod kinematics;
pub mod pines;
pub mod propulsion;
pub mod rigidbody;
pub mod rotation;

pub use gravity::{gacc_nbody, jcoeff_perturbation, single_gacc, GravBody};
pub use integrator::{
    advance_state, rk2_step, rk4_step, rk_drv, rk_step, sy_step, ForceFn, RkCoeffs, SyCoeffs,
};
pub use kepler::Elements;
pub use kinematics::{attitude_errors, pitch_yaw_angles, roll_angle, tip_angle};
pub use pines::PinesModel;
pub use propulsion::{
    atm_scale, current_dir, effective_isp, mass_flow_rate, pfac_from_isp_sl, pfac_from_sl_points,
    pfac_from_thrust_sl, slew_gimbal, slew_throttle, thrust, yaw_axis, G0, P_REF_SL,
};
pub use rigidbody::{
    add_component_force_and_moment, center_of_mass, component_state_vectors, composite_pmi,
    rel_docking_pos, supervessel_state_from_root, DockGeometry, SubVesselData,
};
pub use rigidbody::{
    euler_full, euler_inv_full, euler_inv_simple, euler_inv_zero, gravity_gradient_torque,
};
pub use rotation::surface_inertial_velocity;
pub use rotation::RotationState;

pub use aero::{
    alpha_stall_mach, compute_aero_forces, compute_body_aero, compute_rocket_aero,
    fin_local_alpha, grid_eta, induced_drag, moment_about_cg, side_area, slew_deploy,
    update_leeward_sheltered, wave_drag, world_to_airvel_ship, AeroForces, Airfoil, AirfoilCoeffs,
    AirfoilOrientation, ControlSurface, CtrlAxis, CtrlType, DragElement, FinKind, LiftingSurface,
    RocketAeroInput, RocketBodyAero,
    TriaxialAreas, DEFAULT_ALPHA_STALL_FIN, DEFAULT_ALPHA_STALL_GRID, LEEWARD_Q_FACTOR,
};
pub use atmosphere::{
    atmosphere_from_config, Atmosphere, ExponentialAtmosphere, UsStd1976Atmosphere,
};
pub use contact::{compute_surface_forces, SurfaceContact, TouchdownVertex};
