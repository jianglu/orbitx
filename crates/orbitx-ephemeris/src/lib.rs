//! Ephemeris system for orbitx: VSOP87 planetary positions, ELP2000-82 lunar
//! positions, and cubic Hermite spline interpolation.
//!
//! Mirrors Orbiter's `Src/Celbody/` modules:
//! - VSOP87 (`Vsop87.cpp`): planets Mercury–Neptune and the Sun
//! - ELP2000-82 (`ELP82.cpp`): Earth's Moon
//! - Hermite interpolation (`Interpolate()` in Vsop87.cpp/Moon.cpp)
//!
//! All evaluation algorithms are symbol-for-symbol replicas of the C++
//! implementation to ensure numerical parity. Correctness is verified via
//! property tests against the C++ oracle in `orbitx-ephemeris-ffi`.

#![allow(
    clippy::approx_constant,
    clippy::excessive_precision,
    clippy::needless_range_loop
)]

pub mod elp82;
pub mod galsat;
pub mod sample;
pub mod tass17;
pub mod vsop87;

// Re-exports for ergonomic access.
pub use elp82::{ElpConstants, ElpModel, ElpTerm};
pub use galsat::GalModel;
pub use sample::{interpolate, Sample};
pub use tass17::{HyperionBlock, TasIks, TasModel, TasSat, TasTerm};
pub use vsop87::{Series, VsopModel, VsopTerm, VSOP_MAXALPHA};
