//! 推进器：产生推力并消耗燃料。

use orbitx_math::Vec3;

/// 标准重力加速度 [m/s²]。
pub const G0: f64 = 9.80665;

/// 推进器：产生推力并消耗燃料。
#[derive(Clone, Debug)]
pub struct Thruster {
    /// 体坐标系下的位置。
    pub pos: Vec3,
    /// 体坐标系下的推力方向（单位向量）。
    pub dir: Vec3,
    /// 最大推力 [N]。
    pub max_thrust: f64,
    /// 比冲 [s]。
    pub isp: f64,
    /// 当前油门（0..1）。
    pub level: f64,
}

impl Thruster {
    /// 创建新推进器。
    pub fn new(pos: Vec3, dir: Vec3, max_thrust: f64, isp: f64) -> Self {
        Self {
            pos,
            dir,
            max_thrust,
            isp,
            level: 0.0,
        }
    }

    /// 当前推力 [N]。
    pub fn current_thrust(&self) -> f64 {
        self.max_thrust * self.level
    }

    /// 燃料消耗率 [kg/s] = thrust / (isp * g0)。
    pub fn mass_flow_rate(&self) -> f64 {
        if self.isp > 0.0 {
            self.current_thrust() / (self.isp * G0)
        } else {
            0.0
        }
    }
}
