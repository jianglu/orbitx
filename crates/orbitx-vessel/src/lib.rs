//! 航天器物理库导出。

pub mod aero;
pub mod assembly;
pub mod attitude;
pub mod diagnostics;
pub mod dock;
pub mod fuel;
pub mod pad;
pub mod rcs;
pub mod stage;
pub mod supervessel;
pub mod telemetry;
pub mod thruster;
pub mod touchdown;
pub mod vessel;

pub mod presets;

#[cfg(test)]
mod tests;

pub use aero::{
    atmosphere_from_config, compute_aero_forces, interpolate_cd_mach, world_to_airvel_ship,
    AeroForces, Airfoil, AirfoilCoeffs, AirfoilOrientation, Atmosphere, ControlSurface, CtrlAxis,
    CtrlType, DragElement, ExponentialAtmosphere, UsStd1976Atmosphere,
};
pub use assembly::{Assembly, StepEnv};
pub use attitude::{attitude_errors, pitch_yaw_angles, roll_angle, tip_angle};
pub use diagnostics::FlightDiagnostics;
pub use dock::DockPort;
pub use fuel::PropellantTank;
pub use pad::surface_inertial_velocity;
pub use rcs::{
    add_default_rcs, get_group_level, set_attitude_lin, set_attitude_rot, set_group_level, LinAxis,
    RotAxis, ThrusterGroup, ThrusterGroupType,
};
pub use stage::{default_rocket_cd_mach, StageSpec, ThrusterSpec, PMI_UNDEF};
pub use supervessel::{rel_docking_pos, SubVesselData};
pub use thruster::{
    pfac_from_isp_sl, pfac_from_sl_points, pfac_from_thrust_sl, Thruster, G0, P_REF_SL,
};
pub use touchdown::{compute_surface_forces, make_landing_gear, SurfaceContact, TouchdownVertex};
pub use vessel::{stage_spec_from_config, Vessel};
pub use telemetry::{
    AttitudeReadout, BodyReadout, KinematicsReadout, MassReadout, StageReadout, ThrustReadout,
};
