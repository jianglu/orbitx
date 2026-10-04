//! 姿态角导出（俯仰/偏航/滚转/tip）。
//!
//! 算法已下沉到 `orbitx_dynamics::kinematics`；本模块仅 re-export 以保持
//! vessel 对外接口稳定。步进诊断姿态经此导入写入 `FlightDiagnostics`。

pub use orbitx_dynamics::kinematics::{attitude_errors, pitch_yaw_angles, roll_angle, tip_angle};
