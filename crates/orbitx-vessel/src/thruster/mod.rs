//! 推进器数据结构（薄包装层）。
//!
//! 物理算法（推力 / 比冲 / 流量 / TVC 几何 / 节流斜坡）已下沉到
//! `orbitx_dynamics::propulsion`；本模块保留 `Thruster` 数据字段与构造器，
//! 计算方法退化为对 dynamics 的薄包装。

use orbitx_dynamics::propulsion as prop;
use orbitx_math::Vec3;

pub use orbitx_dynamics::propulsion::{
    pfac_from_isp_sl, pfac_from_sl_points, pfac_from_thrust_sl, G0, P_REF_SL,
};

/// 推进器：产生推力并消耗燃料。
#[derive(Clone, Debug)]
pub struct Thruster {
    /// 体坐标系下的位置。
    pub pos: Vec3,
    /// 名义推力方向（体坐标系，单位向量），不随 TVC 改变。
    pub base_dir: Vec3,
    /// 俯仰万向节角 [rad]（绕 [`gimbal_axis`]，带符号）。
    pub gimbal_pitch: f64,
    /// 偏航万向节角 [rad]（绕 `gimbal_axis × base_dir` 的正交轴）。
    pub gimbal_yaw: f64,
    /// 俯仰偏转轴（体坐标系）。默认 X 轴。
    pub gimbal_axis: Vec3,
    /// 最大偏转角 [rad]（俯仰/偏航各自限幅）。0 = 无 TVC。
    pub max_gimbal: f64,
    /// 最大偏转角速率 [rad/s]。0 = 无限制。
    pub max_gimbal_rate: f64,
    /// 真空最大推力 [N]。
    pub max_thrust: f64,
    /// 真空比冲 [s]。
    pub isp: f64,
    /// 气压缩放因子（Orbiter `pfac`）。0 = 不随气压变化。
    pub pfac: f64,
    /// 当前实际油门（0..1）；推力按此计算。
    pub level: f64,
    /// 指令油门（0..1）；由 `set_throttle` 写入，经 [`slew_throttle`] 逼近 `level`。
    pub level_cmd: f64,
    /// 节流斜坡最大速率 [1/s]。0 = 瞬时跟随指令。
    pub throttle_rate: f64,
    /// 关联的推进剂储箱 ID。`None` = 使用 Vessel 的旧式 `fuel_mass`。
    pub tank_id: Option<u32>,
}

impl Thruster {
    /// 创建新推进器（默认无 TVC、无气压修正、瞬时节流）。
    pub fn new(pos: Vec3, dir: Vec3, max_thrust: f64, isp: f64) -> Self {
        Self {
            pos,
            base_dir: dir,
            gimbal_pitch: 0.0,
            gimbal_yaw: 0.0,
            gimbal_axis: Vec3::new(1.0, 0.0, 0.0),
            max_gimbal: 0.0,
            max_gimbal_rate: 0.0,
            max_thrust,
            isp,
            pfac: 0.0,
            level: 0.0,
            level_cmd: 0.0,
            throttle_rate: 0.0,
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

    /// 设置节流斜坡速率 [1/s]。0 = 瞬时。
    pub fn with_throttle_rate(mut self, throttle_rate: f64) -> Self {
        self.throttle_rate = throttle_rate.max(0.0);
        self
    }

    /// 设置气压缩放 `pfac`。
    pub fn with_pfac(mut self, pfac: f64) -> Self {
        self.pfac = pfac.max(0.0);
        self
    }

    /// 偏航轴：`gimbal_axis × base_dir`（对 +Y 推力与 +X 俯仰轴 → +Z）。
    pub fn yaw_axis(&self) -> Vec3 {
        prop::yaw_axis(self.gimbal_axis, self.base_dir)
    }

    /// 环境气压缩放因子 `s(p)=max(0, 1−p·pfac)`。
    pub fn atm_scale(&self, pressure_pa: f64) -> f64 {
        prop::atm_scale(self.pfac, pressure_pa)
    }

    /// 当前推力 [N]（含气压缩放）。
    pub fn current_thrust(&self, pressure_pa: f64) -> f64 {
        prop::thrust(self.max_thrust, self.level, self.pfac, pressure_pa)
    }

    /// 当前有效比冲 [s]。
    pub fn effective_isp(&self, pressure_pa: f64) -> f64 {
        prop::effective_isp(self.isp, self.pfac, pressure_pa)
    }

    /// 燃料消耗率 [kg/s] = thrust / (isp_eff · g0)。
    pub fn mass_flow_rate(&self, pressure_pa: f64) -> f64 {
        let thr = self.current_thrust(pressure_pa);
        let isp_e = self.effective_isp(pressure_pa);
        prop::mass_flow_rate(thr, isp_e)
    }

    /// 设置俯仰万向节角 [rad]（兼容旧单轴 API）。
    pub fn set_gimbal(&mut self, pitch: f64) {
        self.set_gimbal_2(pitch, self.gimbal_yaw);
    }

    /// 设置俯仰/偏航万向节角 [rad]。
    pub fn set_gimbal_2(&mut self, pitch: f64, yaw: f64) {
        if self.max_gimbal > 0.0 {
            self.gimbal_pitch = pitch.clamp(-self.max_gimbal, self.max_gimbal);
            self.gimbal_yaw = yaw.clamp(-self.max_gimbal, self.max_gimbal);
        } else {
            self.gimbal_pitch = 0.0;
            self.gimbal_yaw = 0.0;
        }
    }

    /// 将双轴万向节以最大速率趋向目标。
    pub fn slew_gimbal(&mut self, pitch_target: f64, yaw_target: f64, dt: f64) {
        let (p, y) = prop::slew_gimbal(
            self.gimbal_pitch,
            self.gimbal_yaw,
            pitch_target,
            yaw_target,
            self.max_gimbal,
            self.max_gimbal_rate,
            dt,
        );
        self.gimbal_pitch = p;
        self.gimbal_yaw = y;
    }

    /// 将实际油门以 `throttle_rate` 趋向指令；速率为 0 时瞬时跟随。
    pub fn slew_throttle(&mut self, dt: f64) {
        self.level = prop::slew_throttle(self.level, self.level_cmd, self.throttle_rate, dt);
    }

    /// 实际推力方向（体坐标系，单位向量）：先俯仰后偏航。
    pub fn current_dir(&self) -> Vec3 {
        prop::current_dir(
            self.base_dir,
            self.gimbal_axis,
            self.gimbal_pitch,
            self.gimbal_yaw,
        )
    }
}

#[cfg(test)]
mod tests;
