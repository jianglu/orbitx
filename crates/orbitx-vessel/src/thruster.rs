//! 推进器：产生推力并消耗燃料。
//!
//! TVC（推力矢量控制）：推进器的 [`base_dir`] 是名义推力方向（沿火箭
//! 体轴），而 [`gimbal`] 是当前万向节偏转角。实际推力方向
//! [`current_dir`] = `base_dir` 绕 [`gimbal_axis`] 旋转 `gimbal` 弧度。
//! 这个侧向偏移产生恢复力矩，由 [`super::assembly`] 在欧拉方程中积分。

use orbitx_math::{cross, dot, Vec3};

/// 标准重力加速度 [m/s²]。
pub const G0: f64 = 9.80665;

/// 推进器：产生推力并消耗燃料。
#[derive(Clone, Debug)]
pub struct Thruster {
    /// 体坐标系下的位置。
    pub pos: Vec3,
    /// 名义推力方向（体坐标系，单位向量），不随 TVC 改变。
    pub base_dir: Vec3,
    /// 当前万向节偏转角 [rad]，绕 [`gimbal_axis`]，带符号。
    pub gimbal: f64,
    /// 万向节偏转轴（体坐标系，单位向量）。默认 X 轴。
    pub gimbal_axis: Vec3,
    /// 最大偏转角 [rad]（绝对值）。0 = 无 TVC。
    pub max_gimbal: f64,
    /// 最大偏转角速率 [rad/s]（作动器速率限制）。0 = 无限制。
    pub max_gimbal_rate: f64,
    /// 最大推力 [N]。
    pub max_thrust: f64,
    /// 比冲 [s]。
    pub isp: f64,
    /// 当前油门（0..1）。
    pub level: f64,
    /// 关联的推进剂储箱 ID。`None` = 使用 Vessel 的旧式 `fuel_mass`。
    pub tank_id: Option<u32>,
}

impl Thruster {
    /// 创建新推进器（默认无 TVC：max_gimbal=0，gimbal_axis=X）。
    pub fn new(pos: Vec3, dir: Vec3, max_thrust: f64, isp: f64) -> Self {
        Self {
            pos,
            base_dir: dir,
            gimbal: 0.0,
            gimbal_axis: Vec3::new(1.0, 0.0, 0.0),
            max_gimbal: 0.0,
            max_gimbal_rate: 0.0,
            max_thrust,
            isp,
            level: 0.0,
            tank_id: None,
        }
    }

    /// 设置 TVC 参数。
    pub fn with_tvc(mut self, max_gimbal: f64, max_gimbal_rate: f64, gimbal_axis: Vec3) -> Self {
        self.max_gimbal = max_gimbal;
        self.max_gimbal_rate = max_gimbal_rate;
        self.gimbal_axis = gimbal_axis;
        self
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

    /// 设置目标万向节偏转角 [rad]，限幅到 ±max_gimbal。
    /// 不做速率限制——调用方按 dt 分步调用以模拟作动器动力学。
    pub fn set_gimbal(&mut self, angle: f64) {
        if self.max_gimbal > 0.0 {
            self.gimbal = angle.clamp(-self.max_gimbal, self.max_gimbal);
        } else {
            self.gimbal = 0.0;
        }
    }

    /// 将万向节角以最大速率 `max_gimbal_rate` 趋向目标 `target` [rad]。
    /// 模拟作动器一阶速率限制。
    pub fn slew_gimbal(&mut self, target: f64, dt: f64) {
        if self.max_gimbal <= 0.0 {
            self.gimbal = 0.0;
            return;
        }
        let target = target.clamp(-self.max_gimbal, self.max_gimbal);
        let err = target - self.gimbal;
        if self.max_gimbal_rate > 0.0 {
            let max_step = self.max_gimbal_rate * dt;
            let step = err.clamp(-max_step, max_step);
            self.gimbal += step;
        } else {
            self.gimbal = target;
        }
    }

    /// 实际推力方向（体坐标系，单位向量）。
    ///
    /// `base_dir` 绕 `gimbal_axis` 旋转 `gimbal` 弧度（Rodrigues 公式）。
    /// `gimbal=0` 时等于 `base_dir`。
    pub fn current_dir(&self) -> Vec3 {
        if self.gimbal.abs() < 1e-12 {
            return self.base_dir;
        }
        let axis = if self.gimbal_axis.length() > 1e-9 {
            self.gimbal_axis.unit()
        } else {
            Vec3::new(1.0, 0.0, 0.0)
        };
        let c = self.gimbal.cos();
        let s = self.gimbal.sin();
        let v = self.base_dir;
        // Rodrigues: v' = v cosθ + (k×v) sinθ + k (k·v)(1−cosθ)
        let kxv = cross(axis, v);
        let kdv = dot(axis, v);
        v * c + kxv * s + axis * (kdv * (1.0 - c))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_gimbal_returns_base_dir() {
        let mut t = Thruster::new(Vec3::ZERO, Vec3::new(0.0, -1.0, 0.0), 1000.0, 300.0);
        t.max_gimbal = 0.1; // enable TVC but gimbal=0
        let d = t.current_dir();
        assert!((d - Vec3::new(0.0, -1.0, 0.0)).length() < 1e-12);
    }

    #[test]
    fn gimbal_rotates_about_axis() {
        // base_dir = -Y, gimbal about X axis.
        let mut t = Thruster::new(Vec3::ZERO, Vec3::new(0.0, -1.0, 0.0), 1000.0, 300.0)
            .with_tvc(std::f64::consts::FRAC_PI_6, 0.0, Vec3::new(1.0, 0.0, 0.0));
        t.gimbal = std::f64::consts::FRAC_PI_2; // 90° about X
        let d = t.current_dir();
        // Rotating -Y by 90° about X (left-handed frame via cross) → ±Z.
        // Just check the result is unit length and has no X component.
        assert!((d.length() - 1.0).abs() < 1e-9, "not unit: {:?}", d);
        assert!(d.x.abs() < 1e-9, "should stay in YZ plane: {:?}", d);
    }

    #[test]
    fn set_gimbal_clamps() {
        let mut t = Thruster::new(Vec3::ZERO, Vec3::new(0.0, -1.0, 0.0), 1000.0, 300.0)
            .with_tvc(0.1, 0.0, Vec3::new(1.0, 0.0, 0.0));
        t.set_gimbal(5.0);
        assert!((t.gimbal - 0.1).abs() < 1e-12);
        t.set_gimbal(-5.0);
        assert!((t.gimbal + 0.1).abs() < 1e-12);
    }

    #[test]
    fn slew_rate_limited() {
        let mut t = Thruster::new(Vec3::ZERO, Vec3::new(0.0, -1.0, 0.0), 1000.0, 300.0)
            .with_tvc(1.0, 1.0, Vec3::new(1.0, 0.0, 0.0)); // 1 rad/s
        t.slew_gimbal(1.0, 0.1); // step 0.1 → 0.1 rad
        assert!((t.gimbal - 0.1).abs() < 1e-9, "got {}", t.gimbal);
    }
}
