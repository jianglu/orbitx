//! 推进器：产生推力并消耗燃料；真空额定 + 气压缩放（Orbiter `pfac`）。
//! TVC：俯仰/偏航双轴万向节（体坐标绕 pitch 轴与 yaw 轴）。

use orbitx_math::{cross, dot, Vec3};

/// 标准重力加速度 [m/s²]。
pub const G0: f64 = 9.80665;

/// 海平面参考气压 [Pa]（推导 `pfac` 用）。
pub const P_REF_SL: f64 = 101_325.0;

/// 由真空与海平面比冲推导 `pfac`：`Isp(p)=Isp0·(1−p·pfac)`。
pub fn pfac_from_isp_sl(isp_vac: f64, isp_sl: f64) -> f64 {
    if isp_vac <= 1e-9 || isp_sl <= 0.0 || isp_sl >= isp_vac {
        return 0.0;
    }
    (1.0 - isp_sl / isp_vac) / P_REF_SL
}

/// 由真空与海平面推力推导 `pfac`（与比冲公式同形）。
pub fn pfac_from_thrust_sl(thrust_vac: f64, thrust_sl: f64) -> f64 {
    if thrust_vac <= 1e-9 || thrust_sl <= 0.0 || thrust_sl >= thrust_vac {
        return 0.0;
    }
    (1.0 - thrust_sl / thrust_vac) / P_REF_SL
}

/// 优先用比冲双点，否则推力双点。
pub fn pfac_from_sl_points(
    isp_vac: f64,
    isp_sl: Option<f64>,
    thrust_vac: f64,
    thrust_sl: Option<f64>,
) -> f64 {
    if let Some(isl) = isp_sl {
        let p = pfac_from_isp_sl(isp_vac, isl);
        if p > 0.0 {
            return p;
        }
    }
    if let Some(tsl) = thrust_sl {
        return pfac_from_thrust_sl(thrust_vac, tsl);
    }
    0.0
}

#[inline]
fn rodrigues(v: Vec3, axis: Vec3, angle: f64) -> Vec3 {
    if angle.abs() < 1e-12 {
        return v;
    }
    let axis = if axis.length() > 1e-9 {
        axis.unit()
    } else {
        return v;
    };
    let c = angle.cos();
    let s = angle.sin();
    let kxv = cross(axis, v);
    let kdv = dot(axis, v);
    v * c + kxv * s + axis * (kdv * (1.0 - c))
}

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
    /// 当前油门（0..1）。
    pub level: f64,
    /// 关联的推进剂储箱 ID。`None` = 使用 Vessel 的旧式 `fuel_mass`。
    pub tank_id: Option<u32>,
}

impl Thruster {
    /// 创建新推进器（默认无 TVC、无气压修正）。
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

    /// 设置气压缩放 `pfac`。
    pub fn with_pfac(mut self, pfac: f64) -> Self {
        self.pfac = pfac.max(0.0);
        self
    }

    /// 偏航轴：`gimbal_axis × base_dir`（对 +Y 推力与 +X 俯仰轴 → +Z）。
    pub fn yaw_axis(&self) -> Vec3 {
        let pitch_ax = if self.gimbal_axis.length() > 1e-9 {
            self.gimbal_axis.unit()
        } else {
            Vec3::new(1.0, 0.0, 0.0)
        };
        let y = cross(pitch_ax, self.base_dir);
        if y.length() > 1e-9 {
            y.unit()
        } else {
            Vec3::new(0.0, 0.0, 1.0)
        }
    }

    /// 环境气压缩放因子 `s(p)=max(0, 1−p·pfac)`。
    pub fn atm_scale(&self, pressure_pa: f64) -> f64 {
        if self.pfac <= 0.0 {
            return 1.0;
        }
        (1.0 - pressure_pa.max(0.0) * self.pfac).max(0.0)
    }

    /// 当前推力 [N]（含气压缩放）。
    pub fn current_thrust(&self, pressure_pa: f64) -> f64 {
        self.max_thrust * self.level * self.atm_scale(pressure_pa)
    }

    /// 当前有效比冲 [s]。
    pub fn effective_isp(&self, pressure_pa: f64) -> f64 {
        self.isp * self.atm_scale(pressure_pa)
    }

    /// 燃料消耗率 [kg/s] = thrust / (isp_eff · g0)。
    pub fn mass_flow_rate(&self, pressure_pa: f64) -> f64 {
        let thr = self.current_thrust(pressure_pa);
        let isp_e = self.effective_isp(pressure_pa);
        if isp_e > 0.0 && thr > 0.0 {
            thr / (isp_e * G0)
        } else {
            0.0
        }
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
        if self.max_gimbal <= 0.0 {
            self.gimbal_pitch = 0.0;
            self.gimbal_yaw = 0.0;
            return;
        }
        let tp = pitch_target.clamp(-self.max_gimbal, self.max_gimbal);
        let ty = yaw_target.clamp(-self.max_gimbal, self.max_gimbal);
        if self.max_gimbal_rate > 0.0 {
            let max_step = self.max_gimbal_rate * dt;
            let ep = tp - self.gimbal_pitch;
            let ey = ty - self.gimbal_yaw;
            self.gimbal_pitch += ep.clamp(-max_step, max_step);
            self.gimbal_yaw += ey.clamp(-max_step, max_step);
        } else {
            self.gimbal_pitch = tp;
            self.gimbal_yaw = ty;
        }
    }

    /// 实际推力方向（体坐标系，单位向量）：先俯仰后偏航。
    pub fn current_dir(&self) -> Vec3 {
        let pitch_ax = if self.gimbal_axis.length() > 1e-9 {
            self.gimbal_axis.unit()
        } else {
            Vec3::new(1.0, 0.0, 0.0)
        };
        let yaw_ax = self.yaw_axis();
        let after_pitch = rodrigues(self.base_dir, pitch_ax, self.gimbal_pitch);
        let d = rodrigues(after_pitch, yaw_ax, self.gimbal_yaw);
        let len = d.length();
        if len > 1e-12 {
            d * (1.0 / len)
        } else {
            self.base_dir
        }
    }
}

#[cfg(test)]
mod tests;
