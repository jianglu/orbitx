//! 每船飞行诊断缓存（步进写入；CLI 遥测只读，禁止重算）。

/// 每步更新的飞行环境/受力/姿态诊断。
#[derive(Clone, Debug, Default)]
pub struct FlightDiagnostics {
    /// 引力加速度模 [m/s²]。
    pub a_grav: f64,
    /// `|a_grav| / g0`。
    pub g_multiple: f64,
    /// 马赫数。
    pub mach: f64,
    /// 大气密度 [kg/m³]。
    pub density: f64,
    /// 动压 [Pa]。
    pub dynamic_pressure: f64,
    /// 大气压 [Pa]。
    pub pressure: f64,
    /// 主推气压缩放因子 s(p)（多机按推力加权平均）。
    pub thrust_atm_scale: f64,
    /// 有效比冲 [s]（多机加权）。
    pub isp_eff: f64,
    /// 本步实际推力模 [N]（无燃料则为 0）。
    pub thrust: f64,
    /// 气动阻力模 [N]。
    pub drag_force: f64,
    /// 有效阻力系数。
    pub cd_eff: f64,
    /// 气温 [K]。
    pub temperature: f64,
    /// 声速 [m/s]。
    pub sound_speed: f64,
    /// 过载：非引力加速度模 / g0。
    pub load_factor: f64,
    /// 有符号俯仰角 [rad]。
    pub pitch: f64,
    /// 有符号偏航角 [rad]。
    pub yaw: f64,
    /// 滚转角 [rad]。
    pub roll: f64,
    /// 体 +Y 与径向夹角 [rad]。
    pub tip: f64,
}
