//! 固定 `sim_dt` 时钟（权威时刻为整数毫秒）。

#[derive(Debug)]
pub struct Clock {
    sim_dt_ms: u64,
    sim_t_ms: u64,
    step_index: u64,
    paused: bool,
    warp: f64,
}

impl Clock {
    pub fn new(sim_dt_ms: u64) -> Self {
        Self {
            sim_dt_ms,
            sim_t_ms: 0,
            step_index: 0,
            paused: false,
            warp: 1.0,
        }
    }

    pub fn sim_dt_ms(&self) -> u64 {
        self.sim_dt_ms
    }

    pub fn sim_t_ms(&self) -> u64 {
        self.sim_t_ms
    }

    /// 物理边界用：`Assembly::step` 等仍吃秒。
    pub fn sim_dt_secs(&self) -> f64 {
        self.sim_dt_ms as f64 / 1000.0
    }

    pub fn sim_t_secs(&self) -> f64 {
        self.sim_t_ms as f64 / 1000.0
    }

    pub fn step_index(&self) -> u64 {
        self.step_index
    }

    pub fn paused(&self) -> bool {
        self.paused
    }

    pub fn set_paused(&mut self, paused: bool) {
        self.paused = paused;
    }

    pub fn warp(&self) -> f64 {
        self.warp
    }

    pub fn set_warp(&mut self, scale: f64) {
        self.warp = if scale.is_finite() && scale > 0.0 {
            scale
        } else {
            1.0
        };
    }

    /// 完成一步固定 `sim_dt_ms`（不随 warp 改变）。
    pub fn advance_fixed_step(&mut self) {
        self.sim_t_ms = self.sim_t_ms.saturating_add(self.sim_dt_ms);
        self.step_index = self.step_index.saturating_add(1);
    }
}

#[cfg(test)]
mod tests;
