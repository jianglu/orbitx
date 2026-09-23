//! 发射台 / 表面惯性初速（地球自转 ω×r）。
//!
//! 算法已下沉到 `orbitx_dynamics::rotation::surface_inertial_velocity`；
//! 本模块仅 re-export 以保持 vessel 对外接口稳定。

pub use orbitx_dynamics::surface_inertial_velocity;
