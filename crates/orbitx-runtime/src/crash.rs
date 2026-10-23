//! 触地终局判定（对齐 Orbiter `CheckLanded` / `InitLanded` / 接触力坠毁）。
//!
//! - 坠毁：只看接触力/穿透（及水面）；**不用**撞击速度门限。
//! - 着陆：无动力 + 位姿速率稳定连续 5 s → `InitLanded` 校正 + Landed。
//! - 从未离地慢速回落台面 → [`ContactOutcome::ReturnedToPad`]（非终局）。

use std::collections::HashSet;

use orbitx_math::{dot, Quat, Vec3};
use orbitx_vessel::{
    surface_inertial_velocity, Assembly, GroundContact, SurfaceContact, SurfaceKind, G0,
};

use crate::session::launch_attitude;

/// 着陆稳定窗口 [s]（与 Orbiter `CheckLanded` 一致）。
pub const LAND_STABLE_S: f64 = 5.0;

/// 有效推力低于此值视为无动力 [N]。
pub const LAND_THRUST_EPS_N: f64 = 1.0;

/// 法向接触力超过 `k × 重量` 即坠毁。
pub const CRASH_FORCE_WEIGHT_MULT: f64 = 5.0;

/// 穿透深度超过此值（|penetration|）即坠毁 [m]。
pub const CRASH_MAX_PENETRATION_M: f64 = 0.5;

/// Orbiter 式位移速率阈值：`dst²/dt > 1e-4` → 仍在移动 [m²/s]。
const LAND_MOVE_RATE2: f64 = 1e-4;

/// Orbiter 式姿态变化率阈值 [rad/s]。
const LAND_ANGLE_RATE: f64 = 1e-4;

/// 主栈触地终局（独立体坠毁在本模块内直接标记）。
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ContactOutcome {
    None,
    /// 坠毁。`water` 时文案为坠入水面；`force_n` 为触发时法向接触力。
    Crash { water: bool, force_n: f64 },
    /// 着陆成功（已校正贴地）。
    Landed { force_n: f64 },
    /// 已释放但从未离地、慢速回落台面：台架重新接管，非终局。
    ReturnedToPad,
}

/// 对齐 Orbiter `LandingTest`：无动力贴地稳定计时。
#[derive(Debug, Clone, Default)]
pub struct LandDwellState {
    pub testing: bool,
    pub since: Option<f64>,
    /// 上帧地表相对位置（地心系，相对光滑球近似即可）。
    surf_pos: Option<Vec3>,
    /// 上帧姿态四元数。
    surf_q: Option<Quat>,
}

impl LandDwellState {
    pub fn clear(&mut self) {
        self.testing = false;
        self.since = None;
        self.surf_pos = None;
        self.surf_q = None;
    }
}

/// 坠毁力门限：`max(k×重量, 绝对下限)`。
pub fn crash_force_limit_n(mass_kg: f64) -> f64 {
    let weight = mass_kg.max(1.0) * G0;
    (CRASH_FORCE_WEIGHT_MULT * weight).max(1e5)
}

/// 接触力/穿透是否构成坠毁。
pub fn contact_force_is_crash(contact: &SurfaceContact, mass_kg: f64) -> bool {
    if !contact.in_contact {
        return false;
    }
    let penet = (-contact.max_penetration).max(0.0);
    if penet > CRASH_MAX_PENETRATION_M {
        return true;
    }
    contact.max_normal_force > crash_force_limit_n(mass_kg)
}

/// 判定主栈触地终局，并顺带标记触地坠毁的独立体。
///
/// - [`GroundContact::Off`]：不算地形终局。
/// - 顺序：水面 → Crash；接触力过大 → Crash；未离地回落 → ReturnedToPad；
///   已离地无动力稳定满 5 s → InitLanded + Landed。
pub fn apply_contact_checks(
    asm: &mut Assembly,
    launched: bool,
    was_airborne: bool,
    thrust_attempted: bool,
    effective_thrust_n: f64,
    land_dwell: &mut LandDwellState,
    sim_t_s: f64,
) -> ContactOutcome {
    if matches!(asm.ground, GroundContact::Off) {
        land_dwell.clear();
        return ContactOutcome::None;
    }

    let primary: HashSet<usize> = asm.components.iter().map(|c| c.vessel_index).collect();

    for i in 0..asm.vessels.len() {
        if asm.vessels[i].crashed || primary.contains(&i) {
            continue;
        }
        if detached_is_crash(asm, i) {
            asm.mark_crashed(i);
        }
    }

    let primary_crashed = asm
        .components
        .iter()
        .any(|c| asm.vessels[c.vessel_index].crashed);
    if !launched || primary_crashed {
        land_dwell.clear();
        return ContactOutcome::None;
    }

    let contact = asm.last_surface_contact.clone();
    let in_contact = contact.in_contact
        || asm
            .bottom_clearance()
            .map(|c| c <= 0.0)
            .unwrap_or(false);

    if !in_contact {
        land_dwell.clear();
        return ContactOutcome::None;
    }

    let pos = asm.state.pos;
    let water = matches!(
        asm.ground.surface_kind_at_world(pos),
        Some(SurfaceKind::Water)
    );
    let mass = asm.total_mass();
    let force_n = contact.max_normal_force;

    if water {
        land_dwell.clear();
        let active = asm.active.min(asm.vessels.len().saturating_sub(1));
        asm.mark_crashed(active);
        return ContactOutcome::Crash {
            water: true,
            force_n,
        };
    }

    let descending = impact_speed_mps(asm, pos, asm.state.vel) >= 0.0;
    if !was_airborne {
        // 从未离地：台架刮地/穿丘的接触力不判坠毁（否则起飞横漂即误毁）。
        land_dwell.clear();
        if thrust_attempted && descending {
            return ContactOutcome::ReturnedToPad;
        }
        return ContactOutcome::None;
    }

    // 曾离地后：过大接触力/穿透 → 坠毁（不用 v_imp）。
    if contact_force_is_crash(&contact, mass) {
        land_dwell.clear();
        let active = asm.active.min(asm.vessels.len().saturating_sub(1));
        asm.mark_crashed(active);
        return ContactOutcome::Crash {
            water: false,
            force_n,
        };
    }

    // —— CheckLanded：无动力 + 位姿稳定 5 s ——
    let thrust_off = effective_thrust_n <= LAND_THRUST_EPS_N;
    if !thrust_off {
        land_dwell.clear();
        return ContactOutcome::None;
    }

    let q = asm.state.q;
    if land_dwell.testing {
        let dt = match land_dwell.since {
            Some(t0) => (sim_t_s - t0).max(1e-9),
            None => {
                land_dwell.since = Some(sim_t_s);
                land_dwell.surf_pos = Some(pos);
                land_dwell.surf_q = Some(q);
                return ContactOutcome::None;
            }
        };
        // Orbiter：dt < 0.1 不采样
        if dt < 0.1 {
            return ContactOutcome::None;
        }
        let prev_pos = land_dwell.surf_pos.unwrap_or(pos);
        let prev_q = land_dwell.surf_q.unwrap_or(q);
        let dst2 = (pos - prev_pos).length2();
        let dangle = quat_angle(prev_q, q);
        land_dwell.surf_pos = Some(pos);
        land_dwell.surf_q = Some(q);
        if dst2 / dt > LAND_MOVE_RATE2 || dangle / dt > LAND_ANGLE_RATE {
            land_dwell.since = Some(sim_t_s);
            return ContactOutcome::None;
        }
        let t0 = land_dwell.since.unwrap_or(sim_t_s);
        if sim_t_s - t0 < LAND_STABLE_S {
            return ContactOutcome::None;
        }
        init_landed(asm);
        land_dwell.clear();
        return ContactOutcome::Landed { force_n };
    }

    // 开始计时
    land_dwell.testing = true;
    land_dwell.since = Some(sim_t_s);
    land_dwell.surf_pos = Some(pos);
    land_dwell.surf_q = Some(q);
    ContactOutcome::None
}

/// 独立体：水面或接触力过大 → 坠毁（无速度兜底）。
fn detached_is_crash(asm: &Assembly, vessel_index: usize) -> bool {
    let Some(clearance) = asm.vessel_bottom_clearance(vessel_index) else {
        return false;
    };
    if clearance > 0.0 {
        return false;
    }
    let v = &asm.vessels[vessel_index];
    let water = matches!(
        asm.ground.surface_kind_at_world(v.state.pos),
        Some(SurfaceKind::Water)
    );
    if water {
        return true;
    }
    // 用主栈最近接触摘要近似；独立体若无步进接触力，以穿透深度代理。
    // 独立体步进也会写自己的力，但目前只缓存主栈；穿透过深仍判毁。
    let contact = &asm.last_surface_contact;
    if contact.in_contact && contact_force_is_crash(contact, v.mass()) {
        return true;
    }
    // 间隙深度超过坠毁穿透门限（无主栈接触缓存时）
    (-clearance) > CRASH_MAX_PENETRATION_M
}

/// InitLanded：姿态径向对齐，再把底部钉到当地地面，速度共转、`omega = 0`。
pub fn init_landed(asm: &mut Assembly) {
    let pos = asm.state.pos;
    let r_mag = pos.length().max(1e-9);
    let radial = pos * (1.0 / r_mag);
    let (lock_r, lock_q) = launch_attitude(radial);

    // 先统一姿态（与台架钉死同口径），再按底部归零。
    for v in &mut asm.vessels {
        if !v.detached {
            v.state.r = lock_r;
            v.state.q = lock_q;
            v.state.omega = Vec3::ZERO;
        }
    }
    asm.state.r = lock_r;
    asm.state.q = lock_q;
    asm.state.omega = Vec3::ZERO;

    let ground_r = asm
        .ground
        .radius_at_world(pos)
        .unwrap_or(asm.planet_radius);
    if let Some(bottom) = asm.primary_bottom_point() {
        let br = bottom.length().max(1e-9);
        let shift = bottom * ((ground_r - br) / br);
        if shift.length() > 1e-12 {
            for v in &mut asm.vessels {
                if !v.detached {
                    v.state.pos = v.state.pos + shift;
                }
            }
            asm.state.pos = asm.state.pos + shift;
        }
    }

    let new_pos = asm.state.pos;
    let pad_vel = if asm.sid_rot_period > 1e-9 {
        surface_inertial_velocity(new_pos, asm.sid_rot_period)
    } else {
        Vec3::ZERO
    };
    for v in &mut asm.vessels {
        if !v.detached {
            v.state.vel = pad_vel;
        }
    }
    asm.state.vel = pad_vel;
    let root = asm.root.min(asm.vessels.len().saturating_sub(1));
    asm.state = asm.vessels[root].state;
}

fn quat_angle(a: Quat, b: Quat) -> f64 {
    // |2 acos(|dot|)| 相对旋转角
    let d = (a.s * b.s + a.vx * b.vx + a.vy * b.vy + a.vz * b.vz).abs().clamp(0.0, 1.0);
    2.0 * d.acos()
}

/// Ground-relative radial impact speed [m/s], positive when moving into the surface.
fn impact_speed_mps(asm: &Assembly, pos: Vec3, vel: Vec3) -> f64 {
    let r_mag = pos.length().max(1e-9);
    let radial = pos * (1.0 / r_mag);
    let v_ground = if asm.sid_rot_period > 1e-9 {
        vel - surface_inertial_velocity(pos, asm.sid_rot_period)
    } else {
        vel
    };
    -dot(v_ground, radial)
}

#[cfg(test)]
mod tests {
    use super::*;
    use orbitx_math::StateVectors;
    use orbitx_vessel::{presets, GroundSampler, GroundContact, SurfaceKind};
    use std::sync::Arc;

    struct WaterSampler;
    impl GroundSampler for WaterSampler {
        fn ground_radius_m(&self, _lat: f64, _lon: f64) -> f64 {
            6_371_000.0
        }
        fn surface_kind_at(&self, _lat: f64, _lon: f64) -> SurfaceKind {
            SurfaceKind::Water
        }
    }

    fn pad_asm(pos: Vec3, vel: Vec3) -> Assembly {
        let stages = presets::falcon9();
        let r = 6_371_000.0;
        let mut asm = Assembly::new(
            &stages,
            StateVectors {
                pos,
                vel,
                ..Default::default()
            },
        );
        asm.planet_radius = r;
        asm.sid_rot_period = 86_164.0;
        asm.ground = GroundContact::Sphere { radius_m: r };
        asm
    }

    fn vel_down(pos: Vec3, down_mps: f64) -> Vec3 {
        surface_inertial_velocity(pos, 86_164.0) + Vec3::new(-down_mps, 0.0, 0.0)
    }

    fn soft_contact(force_n: f64) -> SurfaceContact {
        SurfaceContact {
            force: Vec3::ZERO,
            torque: Vec3::ZERO,
            in_contact: true,
            max_penetration: -0.01,
            max_normal_force: force_n,
        }
    }

    fn hard_contact(force_n: f64, penet: f64) -> SurfaceContact {
        SurfaceContact {
            force: Vec3::ZERO,
            torque: Vec3::ZERO,
            in_contact: true,
            max_penetration: -penet,
            max_normal_force: force_n,
        }
    }

    #[test]
    fn under_five_seconds_does_not_land() {
        let r = 6_371_000.0;
        let pos = Vec3::new(r - 1.0, 0.0, 0.0);
        let vel = surface_inertial_velocity(pos, 86_164.0);
        let mut asm = pad_asm(pos, vel);
        asm.last_surface_contact = soft_contact(1e3);
        let mut dwell = LandDwellState::default();
        for i in 0..20 {
            let t = i as f64 * 0.2; // 0..3.8 s < 5 s
            let out = apply_contact_checks(
                &mut asm, true, true, true, 0.0, &mut dwell, t,
            );
            assert_eq!(out, ContactOutcome::None, "t={t}");
        }
        assert!(dwell.testing);
        assert!((3.8 - dwell.since.unwrap()) < LAND_STABLE_S);
    }

    #[test]
    fn soft_force_high_v_imp_does_not_crash() {
        // 大撞击速度但接触力/穿透未超门限 → 不 Crash（无 v_imp 兜底）。
        let r = 6_371_000.0;
        let pos = Vec3::new(r - 1.0, 0.0, 0.0);
        let vel = vel_down(pos, 50.0);
        let mut asm = pad_asm(pos, vel);
        asm.last_surface_contact = soft_contact(1e3);
        let mut dwell = LandDwellState::default();
        let out = apply_contact_checks(
            &mut asm, true, true, true, 0.0, &mut dwell, 1.0,
        );
        assert!(
            !matches!(out, ContactOutcome::Crash { .. }),
            "got {out:?}"
        );
    }

    #[test]
    fn thrust_on_never_lands() {
        let r = 6_371_000.0;
        let pos = Vec3::new(r - 1.0, 0.0, 0.0);
        let vel = surface_inertial_velocity(pos, 86_164.0);
        let mut asm = pad_asm(pos, vel);
        asm.last_surface_contact = soft_contact(1e3);
        let mut dwell = LandDwellState::default();
        for i in 0..300 {
            let t = i as f64 * 0.02;
            let out = apply_contact_checks(
                &mut asm, true, true, true, 1e5, &mut dwell, t,
            );
            assert_eq!(out, ContactOutcome::None, "t={t}");
        }
        assert!(!dwell.testing);
    }

    #[test]
    fn stable_five_seconds_lands_and_snaps() {
        let r = 6_371_000.0;
        let pos = Vec3::new(r + 10.0, 0.0, 0.0);
        let vel = surface_inertial_velocity(pos, 86_164.0);
        let mut asm = pad_asm(pos, vel);
        // 把底部放到地面附近：质心高度约 half stack；用接触标志驱动判定。
        asm.last_surface_contact = soft_contact(1e3);
        let mut dwell = LandDwellState::default();
        let mut landed = false;
        // 每帧保持同一位置（稳定）；步进 0.2 s 以满足 Orbiter dt≥0.1 采样
        for i in 0..40 {
            let t = i as f64 * 0.2;
            // 冻结位姿：不移动
            asm.state.pos = pos;
            asm.state.vel = vel;
            for v in &mut asm.vessels {
                if !v.detached {
                    v.state.pos = pos;
                    v.state.vel = vel;
                }
            }
            let out = apply_contact_checks(
                &mut asm, true, true, true, 0.0, &mut dwell, t,
            );
            if matches!(out, ContactOutcome::Landed { .. }) {
                landed = true;
                break;
            }
        }
        assert!(landed, "expected Landed after ~5s stable");
        let clearance = asm.bottom_clearance().unwrap();
        assert!(
            clearance.abs() < 1.0,
            "after InitLanded clearance {clearance}"
        );
    }

    #[test]
    fn motion_resets_dwell() {
        let r = 6_371_000.0;
        let mut pos = Vec3::new(r - 1.0, 0.0, 0.0);
        let mut asm = pad_asm(pos, surface_inertial_velocity(pos, 86_164.0));
        asm.last_surface_contact = soft_contact(1e3);
        let mut dwell = LandDwellState::default();
        // 前 2 s 稳定
        for i in 0..10 {
            let t = i as f64 * 0.2;
            let _ = apply_contact_checks(
                &mut asm, true, true, true, 0.0, &mut dwell, t,
            );
        }
        assert!(dwell.testing);
        let t0 = dwell.since.unwrap();
        // 突然平移 → 重置 since
        pos.x += 10.0;
        asm.state.pos = pos;
        for v in &mut asm.vessels {
            if !v.detached {
                v.state.pos = pos;
            }
        }
        let _ = apply_contact_checks(
            &mut asm, true, true, true, 0.0, &mut dwell, 2.2,
        );
        assert!(dwell.since.unwrap() >= t0 + 0.1);
    }

    #[test]
    fn never_airborne_large_force_does_not_crash() {
        let r = 6_371_000.0;
        let pos = Vec3::new(r - 1.0, 0.0, 0.0);
        let vel = vel_down(pos, 1.0);
        let mut asm = pad_asm(pos, vel);
        let mass = asm.total_mass();
        asm.last_surface_contact = hard_contact(crash_force_limit_n(mass) * 2.0, 0.8);
        let mut dwell = LandDwellState::default();
        let out = apply_contact_checks(
            &mut asm, true, false, true, 0.0, &mut dwell, 1.0,
        );
        assert_eq!(out, ContactOutcome::ReturnedToPad);
        assert!(!asm.vessels[asm.active].crashed);
    }

    #[test]
    fn large_force_crashes_without_speed() {
        let r = 6_371_000.0;
        let pos = Vec3::new(r - 1.0, 0.0, 0.0);
        // 共转 ≈ 零撞击速度
        let vel = surface_inertial_velocity(pos, 86_164.0);
        let mut asm = pad_asm(pos, vel);
        let mass = asm.total_mass();
        let huge = crash_force_limit_n(mass) * 2.0;
        asm.last_surface_contact = hard_contact(huge, 0.05);
        let mut dwell = LandDwellState::default();
        let out = apply_contact_checks(
            &mut asm, true, true, true, 0.0, &mut dwell, 1.0,
        );
        assert!(
            matches!(out, ContactOutcome::Crash { water: false, .. }),
            "got {out:?}"
        );
        assert!(asm.vessels[asm.active].crashed);
    }

    #[test]
    fn deep_penetration_crashes() {
        let r = 6_371_000.0;
        let pos = Vec3::new(r - 1.0, 0.0, 0.0);
        let vel = surface_inertial_velocity(pos, 86_164.0);
        let mut asm = pad_asm(pos, vel);
        asm.last_surface_contact = hard_contact(1.0, CRASH_MAX_PENETRATION_M + 0.1);
        let mut dwell = LandDwellState::default();
        let out = apply_contact_checks(
            &mut asm, true, true, true, 0.0, &mut dwell, 1.0,
        );
        assert!(matches!(out, ContactOutcome::Crash { .. }), "got {out:?}");
    }

    #[test]
    fn never_airborne_slow_sink_returns_to_pad() {
        let r = 6_371_000.0;
        let pos = Vec3::new(r - 1.0, 0.0, 0.0);
        let vel = vel_down(pos, 1.0);
        let mut asm = pad_asm(pos, vel);
        asm.last_surface_contact = soft_contact(1e3);
        let mut dwell = LandDwellState::default();
        let out = apply_contact_checks(
            &mut asm, true, false, true, 0.0, &mut dwell, 1.0,
        );
        assert_eq!(out, ContactOutcome::ReturnedToPad);
    }

    #[test]
    fn never_airborne_ascending_is_not_returned() {
        let r = 6_371_000.0;
        let pos = Vec3::new(r - 1.0, 0.0, 0.0);
        let vel = vel_down(pos, -4.0);
        let mut asm = pad_asm(pos, vel);
        asm.last_surface_contact = soft_contact(1e3);
        let mut dwell = LandDwellState::default();
        assert_eq!(
            apply_contact_checks(
                &mut asm, true, false, true, 0.0, &mut dwell, 1.0,
            ),
            ContactOutcome::None
        );
    }

    #[test]
    fn water_touch_is_crash() {
        let r = 6_371_000.0;
        let pos = Vec3::new(r - 1.0, 0.0, 0.0);
        let vel = vel_down(pos, 0.5);
        let mut asm = pad_asm(pos, vel);
        asm.ground = GroundContact::query(Arc::new(WaterSampler), 0.0);
        asm.last_surface_contact = soft_contact(1e3);
        let mut dwell = LandDwellState::default();
        let out = apply_contact_checks(
            &mut asm, true, true, true, 0.0, &mut dwell, 1.0,
        );
        assert!(
            matches!(out, ContactOutcome::Crash { water: true, .. }),
            "got {out:?}"
        );
    }

    #[test]
    fn above_ground_never_crashes() {
        let r = 6_371_000.0;
        let pos = Vec3::new(r + 100.0, 0.0, 0.0);
        let vel = vel_down(pos, 200.0);
        let mut asm = pad_asm(pos, vel);
        asm.last_surface_contact = SurfaceContact::default();
        let mut dwell = LandDwellState::default();
        assert_eq!(
            apply_contact_checks(
                &mut asm, true, true, true, 0.0, &mut dwell, 1.0,
            ),
            ContactOutcome::None
        );
    }

    #[test]
    fn off_ground_skips() {
        let r = 6_371_000.0;
        let pos = Vec3::new(r - 1.0, 0.0, 0.0);
        let mut asm = pad_asm(pos, Vec3::new(-100.0, 0.0, 0.0));
        asm.ground = GroundContact::Off;
        asm.last_surface_contact = soft_contact(1e6);
        let mut dwell = LandDwellState::default();
        assert_eq!(
            apply_contact_checks(
                &mut asm, true, true, true, 0.0, &mut dwell, 1.0,
            ),
            ContactOutcome::None
        );
    }
}
