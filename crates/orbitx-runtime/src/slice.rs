//! 对外切片（可瘦身）；完整轨迹见 FlightRecorder 格式专文。

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Slice {
    /// 本步结束后仿真时刻 [ms]。
    pub sim_t: u64,
    pub step_index: u64,
    pub paused: bool,
    pub warp: f64,
    /// 占位：阶段 B 填入主船/分离体摘要。
    pub summary: String,
}
