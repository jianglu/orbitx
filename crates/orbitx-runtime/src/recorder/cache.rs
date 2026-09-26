//! L1 队列容量与事件类型。

use std::sync::Arc;

use crate::slice::Slice;

/// L1 有界容量（骨架偏大，降低误丢；**P6** 配 L2 spill）。
pub const RECORDER_L1_CAPACITY: usize = 4096;

#[derive(Debug)]
pub enum RecorderEvent {
    SessionStart { sim_t: u64, world_label: String },
    Step { slice: Arc<Slice> },
}
