//! 产品仿真主进程库：双服务拓扑、channel、生命周期。
//!
//! 权威设计见 `docs/RUNTIME.md`；黑匣子格式见 `docs/FLIGHT_RECORDER.md`。

pub mod channel;
pub mod cli;
pub mod comms;
pub mod crash;
pub mod host;
pub mod input;
pub mod log_setup;
pub mod pad;
pub mod recorder;
pub mod runtime;
pub mod session;
pub mod shutdown;
pub mod slice;
pub mod world;

pub use cli::{DriveModeArg, RuntimeArgs};
pub use host::{run_with_shutdown, spawn_host, HostHandles};
pub use shutdown::ShutdownFlag;
