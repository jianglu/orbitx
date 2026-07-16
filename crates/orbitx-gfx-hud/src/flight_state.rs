//! 飞行状态数据 — 供 HUD/MFD 每帧读取的实时飞行参数。

use orbitx_math::vec3::Vec3;

/// 飞行状态（每帧由仿真层填充，HUD/MFD 只读）。
#[derive(Clone, Debug)]
pub struct FlightState {
    // --- 位置/速度 ---
    /// 航天器位置（米，黄道 J2000）。
    pub position: Vec3,
    /// 航天器速度（m/s）。
    pub velocity: Vec3,
    /// 速度大小（m/s）。
    pub speed: f64,
    /// 垂直速度（m/s，径向分量）。
    pub vertical_speed: f64,
    /// 水平速度（m/s）。
    pub horizontal_speed: f64,

    // --- 轨道要素 ---
    /// 半长轴（m）。
    pub semi_major_axis: f64,
    /// 离心率。
    pub eccentricity: f64,
    /// 轨道倾角（rad）。
    pub inclination: f64,
    /// 近地点高度（m）。
    pub periapsis_alt: f64,
    /// 远地点高度（m）。
    pub apoapsis_alt: f64,
    /// 轨道周期（s）。
    pub period: f64,
    /// 比轨道能量（J/kg）。
    pub specific_energy: f64,

    // --- 姿态 ---
    /// 俯仰角（rad）。
    pub pitch: f64,
    /// 偏航角（rad）。
    pub yaw: f64,
    /// 滚转角（rad）。
    pub bank: f64,

    // --- 推进 ---
    /// 推力（N）。
    pub thrust: f64,
    /// 推重比。
    pub tw_ratio: f64,
    /// 油门（0-1）。
    pub throttle: f64,
    /// 燃料质量（kg）。
    pub fuel_mass: f64,
    /// 总质量（kg）。
    pub total_mass: f64,

    // --- 环境 ---
    /// 距焦点天体距离（m）。
    pub focus_dist: f64,
    /// 距焦点天体表面高度（m）。
    pub altitude: f64,
    /// 焦点天体名称。
    pub focus_name: String,
    /// 大气密度（kg/m³）。
    pub air_density: f64,
    /// 动压（Pa）。
    pub dynamic_pressure: f64,
    /// 马赫数。
    pub mach: f64,

    // --- 时间 ---
    /// 仿真时间（s）。
    pub sim_time: f64,
    /// 修正儒略日。
    pub mjd: f64,
    /// 时间加速倍率。
    pub time_warp: f64,

    // --- 标志 ---
    /// 是否在推力中。
    pub is_thrusting: bool,
    /// 是否已坠毁。
    pub is_crashed: bool,
    /// 是否在地面。
    pub on_ground: bool,
}

impl Default for FlightState {
    fn default() -> Self {
        Self {
            position: Vec3::ZERO,
            velocity: Vec3::ZERO,
            speed: 0.0,
            vertical_speed: 0.0,
            horizontal_speed: 0.0,
            semi_major_axis: 0.0,
            eccentricity: 0.0,
            inclination: 0.0,
            periapsis_alt: 0.0,
            apoapsis_alt: 0.0,
            period: 0.0,
            specific_energy: 0.0,
            pitch: 0.0,
            yaw: 0.0,
            bank: 0.0,
            thrust: 0.0,
            tw_ratio: 0.0,
            throttle: 0.0,
            fuel_mass: 0.0,
            total_mass: 0.0,
            focus_dist: 0.0,
            altitude: 0.0,
            focus_name: String::from("Earth"),
            air_density: 0.0,
            dynamic_pressure: 0.0,
            mach: 0.0,
            sim_time: 0.0,
            mjd: 0.0,
            time_warp: 1.0,
            is_thrusting: false,
            is_crashed: false,
            on_ground: false,
        }
    }
}

impl FlightState {
    /// 轨道分类描述。
    pub fn orbit_class(&self) -> &'static str {
        if self.eccentricity < 1e-6 {
            "Circular"
        } else if self.eccentricity < 1.0 {
            "Elliptical"
        } else if (self.eccentricity - 1.0).abs() < 1e-6 {
            "Parabolic"
        } else {
            "Hyperbolic"
        }
    }

    /// 格式化高度（自动选择 km/Mm）。
    pub fn fmt_altitude(&self) -> String {
        if self.altitude < 1.0e6 {
            format!("{:.1} km", self.altitude / 1e3)
        } else {
            format!("{:.3} Mm", self.altitude / 1e6)
        }
    }

    /// 格式化速度。
    pub fn fmt_speed(&self) -> String {
        if self.speed < 1e3 {
            format!("{:.1} m/s", self.speed)
        } else {
            format!("{:.2} km/s", self.speed / 1e3)
        }
    }
}
