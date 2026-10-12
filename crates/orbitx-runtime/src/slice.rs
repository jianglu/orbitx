//! 对外切片。进程内结构在 `orbitx_protocol::mem`；Comms 调用 [`Slice::to_proto`]。

pub use orbitx_protocol::mem::{
    AttitudeTelem, EnvTelem, FocusTelem, LaunchpadTelem, OrbitTelem, Slice, StageTelem,
};
