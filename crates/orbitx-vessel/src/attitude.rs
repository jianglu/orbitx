//! 姿态角导出（俯仰/偏航/滚转/tip）。
//!
//! 算法已下沉到 `orbitx_dynamics::kinematics`；本模块仅 re-export 以保持
//! vessel 对外接口稳定。`Assembly::refresh_vessel_diagnostics` 等仍经此导入。

pub use orbitx_dynamics::kinematics::{
    attitude_errors, pitch_yaw_angles, roll_angle, tip_angle,
};
