//! 推进物理：推力 / 比冲 / 燃料流量 / TVC 万向节几何。
//!
//! 本模块只含纯算法（输入数据 → 输出数据），不持状态。推进器数据结构
//! （`Thruster`）仍由 `orbitx_vessel` 持有，其方法退化为对这里的薄包装。
//!
//! 模型对齐 Orbiter `pfac` 气压缩放：`Isp(p)=Isp0·(1−p·pfac)`，
//! 推力同步缩放。TVC 为俯仰/偏航双轴万向节（体坐标绕 pitch 轴与 yaw 轴）。

use orbitx_math::{cross, rodrigues, Vec3};

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

/// 环境气压缩放因子 `s(p)=max(0, 1−p·pfac)`；`pfac<=0` 时恒为 1。
pub fn atm_scale(pfac: f64, pressure_pa: f64) -> f64 {
    if pfac <= 0.0 {
        return 1.0;
    }
    (1.0 - pressure_pa.max(0.0) * pfac).max(0.0)
}

/// 当前推力 [N]（含气压缩放）= `max_thrust · level · s(p)`。
pub fn thrust(max_thrust: f64, level: f64, pfac: f64, pressure_pa: f64) -> f64 {
    max_thrust * level * atm_scale(pfac, pressure_pa)
}

/// 当前有效比冲 [s] = `isp · s(p)`。
pub fn effective_isp(isp: f64, pfac: f64, pressure_pa: f64) -> f64 {
    isp * atm_scale(pfac, pressure_pa)
}

/// 燃料消耗率 [kg/s] = `thrust / (isp_eff · g0)`；任一非正则返回 0。
pub fn mass_flow_rate(thrust_n: f64, isp_eff: f64) -> f64 {
    if isp_eff > 0.0 && thrust_n > 0.0 {
        thrust_n / (isp_eff * G0)
    } else {
        0.0
    }
}

/// 偏航轴：`gimbal_axis × base_dir`（对 +Y 推力与 +X 俯仰轴 → +Z）。
/// `gimbal_axis` 退化时回退到 +X；叉积退化时回退到 +Z。
pub fn yaw_axis(gimbal_axis: Vec3, base_dir: Vec3) -> Vec3 {
    let pitch_ax = if gimbal_axis.length() > 1e-9 {
        gimbal_axis.unit()
    } else {
        Vec3::new(1.0, 0.0, 0.0)
    };
    let y = cross(pitch_ax, base_dir);
    if y.length() > 1e-9 {
        y.unit()
    } else {
        Vec3::new(0.0, 0.0, 1.0)
    }
}

/// 实际推力方向（体坐标系，单位向量）：先俯仰后偏航。
pub fn current_dir(
    base_dir: Vec3,
    gimbal_axis: Vec3,
    gimbal_pitch: f64,
    gimbal_yaw: f64,
) -> Vec3 {
    let pitch_ax = if gimbal_axis.length() > 1e-9 {
        gimbal_axis.unit()
    } else {
        Vec3::new(1.0, 0.0, 0.0)
    };
    let yaw_ax = yaw_axis(gimbal_axis, base_dir);
    let after_pitch = rodrigues(base_dir, pitch_ax, gimbal_pitch);
    let d = rodrigues(after_pitch, yaw_ax, gimbal_yaw);
    let len = d.length();
    if len > 1e-12 {
        d * (1.0 / len)
    } else {
        base_dir
    }
}

/// 将双轴万向节以最大速率趋向目标，返回新的 `(pitch, yaw)`。
///
/// `max_gimbal<=0` 表示无 TVC，恒返回 `(0,0)`；`max_gimbal_rate<=0` 表示瞬时跟随。
pub fn slew_gimbal(
    pitch: f64,
    yaw: f64,
    pitch_target: f64,
    yaw_target: f64,
    max_gimbal: f64,
    max_gimbal_rate: f64,
    dt: f64,
) -> (f64, f64) {
    if max_gimbal <= 0.0 {
        return (0.0, 0.0);
    }
    let tp = pitch_target.clamp(-max_gimbal, max_gimbal);
    let ty = yaw_target.clamp(-max_gimbal, max_gimbal);
    if max_gimbal_rate > 0.0 {
        let max_step = max_gimbal_rate * dt;
        let ep = tp - pitch;
        let ey = ty - yaw;
        (pitch + ep.clamp(-max_step, max_step), yaw + ey.clamp(-max_step, max_step))
    } else {
        (tp, ty)
    }
}

/// 将实际油门以 `throttle_rate` 趋向指令；速率为 0 时瞬时跟随。返回新油门。
pub fn slew_throttle(level: f64, level_cmd: f64, throttle_rate: f64, dt: f64) -> f64 {
    let cmd = level_cmd.clamp(0.0, 1.0);
    if throttle_rate <= 0.0 {
        return cmd;
    }
    let max_step = throttle_rate * dt;
    let mut new_level = level + (cmd - level).clamp(-max_step, max_step);
    new_level = new_level.clamp(0.0, 1.0);
    new_level
}

#[cfg(test)]
mod tests {
    use super::*;
    use orbitx_math::PI05;

    #[test]
    fn pfac_from_isp_sl_basic() {
        let pfac = pfac_from_isp_sl(300.0, 270.0);
        assert!(pfac > 0.0);
        let isp_e = effective_isp(300.0, pfac, P_REF_SL);
        assert!((isp_e - 270.0).abs() < 0.05, "got {isp_e}");
        assert!((effective_isp(300.0, pfac, 0.0) - 300.0).abs() < 1e-9);
    }

    #[test]
    fn pfac_from_isp_sl_degenerate() {
        assert_eq!(pfac_from_isp_sl(300.0, 300.0), 0.0);
        assert_eq!(pfac_from_isp_sl(300.0, 350.0), 0.0);
        assert_eq!(pfac_from_isp_sl(0.0, 270.0), 0.0);
        assert_eq!(pfac_from_isp_sl(300.0, 0.0), 0.0);
    }

    #[test]
    fn pfac_from_thrust_sl_matches_isp_form() {
        let p1 = pfac_from_isp_sl(311.0, 282.0);
        let p2 = pfac_from_thrust_sl(914_000.0, 914_000.0 * 282.0 / 311.0);
        assert!((p1 - p2).abs() < 1e-9, "{p1} vs {p2}");
    }

    #[test]
    fn pfac_from_sl_points_prefers_isp() {
        let p_isp = pfac_from_sl_points(311.0, Some(282.0), 914_000.0, Some(0.0));
        assert!((p_isp - pfac_from_isp_sl(311.0, 282.0)).abs() < 1e-12);
        let p_thr = pfac_from_sl_points(311.0, None, 914_000.0, Some(828_000.0));
        assert!((p_thr - pfac_from_thrust_sl(914_000.0, 828_000.0)).abs() < 1e-12);
        assert_eq!(pfac_from_sl_points(311.0, None, 914_000.0, None), 0.0);
    }

    #[test]
    fn atm_scale_clamps_at_zero() {
        assert_eq!(atm_scale(0.0, P_REF_SL), 1.0);
        let pfac = pfac_from_isp_sl(300.0, 270.0);
        // 远超使缩放饱和归零的压力，避免浮点边界抖动。
        assert_eq!(atm_scale(pfac, 100.0 * P_REF_SL), 0.0);
    }

    #[test]
    fn thrust_and_mass_flow() {
        let pfac = pfac_from_isp_sl(300.0, 270.0);
        let thr = thrust(1000.0, 1.0, pfac, 0.0);
        assert!((thr - 1000.0).abs() < 1e-9);
        let isp_e = effective_isp(300.0, pfac, 0.0);
        let mdot = mass_flow_rate(thr, isp_e);
        assert!((mdot - 1000.0 / (300.0 * G0)).abs() < 1e-12, "got {mdot}");
        assert_eq!(mass_flow_rate(0.0, 300.0), 0.0);
        assert_eq!(mass_flow_rate(1000.0, 0.0), 0.0);
    }

    #[test]
    fn current_dir_no_gimbal_returns_base() {
        let d = current_dir(Vec3::new(0.0, -1.0, 0.0), Vec3::new(1.0, 0.0, 0.0), 0.0, 0.0);
        assert!((d - Vec3::new(0.0, -1.0, 0.0)).length() < 1e-12);
    }

    #[test]
    fn current_dir_pitch_stays_in_yz_plane() {
        let d = current_dir(
            Vec3::new(0.0, -1.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            PI05,
            0.0,
        );
        assert!((d.length() - 1.0).abs() < 1e-9, "not unit: {d:?}");
        assert!(d.x.abs() < 1e-9, "should stay in YZ plane: {d:?}");
    }

    #[test]
    fn current_dir_yaw_about_z() {
        let d = current_dir(
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            0.0,
            PI05,
        );
        assert!((d.length() - 1.0).abs() < 1e-9);
        assert!((d.x + 1.0).abs() < 1e-9 && d.y.abs() < 1e-9, "got {d:?}");
    }

    #[test]
    fn slew_gimbal_rate_limited() {
        let (p, y) = slew_gimbal(0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 0.1);
        assert!((p - 0.1).abs() < 1e-9, "got {p}");
        assert!((y - 0.1).abs() < 1e-9, "got {y}");
        let (p, y) = slew_gimbal(0.3, 0.3, 1.0, 1.0, 0.0, 1.0, 0.1);
        assert_eq!((p, y), (0.0, 0.0));
        let (p, y) = slew_gimbal(0.0, 0.0, 5.0, -5.0, 0.1, 0.0, 0.1);
        assert!((p - 0.1).abs() < 1e-12 && (y + 0.1).abs() < 1e-12);
    }

    #[test]
    fn slew_throttle_rate_limited() {
        let lvl = slew_throttle(0.0, 1.0, 0.8, 0.1);
        assert!((lvl - 0.08).abs() < 1e-12, "got {lvl}");
        let lvl = slew_throttle(0.0, 0.75, 0.0, 0.01);
        assert!((lvl - 0.75).abs() < 1e-12);
        let lvl = slew_throttle(0.99, 1.0, 0.8, 100.0);
        assert_eq!(lvl, 1.0);
    }

    #[test]
    fn yaw_axis_degenerate_fallbacks() {
        let y = yaw_axis(Vec3::ZERO, Vec3::new(0.0, 1.0, 0.0));
        assert!((y - Vec3::new(0.0, 0.0, 1.0)).length() < 1e-12, "got {y:?}");
        let y = yaw_axis(Vec3::new(1.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0));
        assert!((y - Vec3::new(0.0, 0.0, 1.0)).length() < 1e-12, "got {y:?}");
    }
}
