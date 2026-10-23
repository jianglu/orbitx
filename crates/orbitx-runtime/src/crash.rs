//! 触地终局判定（对齐 Orbiter `CheckLanded` / `InitLanded` / 接触力坠毁）。
//!
//! - 坠毁：只看接触力/穿透（及水面）；**不用**撞击速度门限。
//! - 着陆：无动力 + **体固相对**位姿速率稳定连续 5 s → `InitLanded`（保留朝向、体固钉姿）+ Landed。
//! - 从未离地慢速回落台面 → [`ContactOutcome::ReturnedToPad`]（非终局）。

use std::collections::HashSet;

use orbitx_dynamics::{to_body, to_inertial};
use orbitx_math::{dot, Matrix3, Quat, Vec3};
use orbitx_vessel::{
    surface_inertial_velocity, Assembly, GroundContact, SurfaceContact, SurfaceKind, G0,
};

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
///
/// 位移/姿态均在**行星体固系**比较（相对地面），共转贴地不计惯性东漂；
/// 姿态用 `Rᵀ·r`（补 Orbiter 注释「应扣行星旋转」）。
#[derive(Debug, Clone, Default)]
pub struct LandDwellState {
    pub testing: bool,
    pub since: Option<f64>,
    /// 上帧体固位置 `Rᵀ·pos`（对齐 Orbiter `prel`）。
    surf_pos: Option<Vec3>,
    /// 上帧体固姿态四元数（由 `Rᵀ·r`）。
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

/// 软着陆落定状态（对齐 Orbiter `land_rot` / `FLIGHTSTATUS_LANDED`）。
///
/// 位置与姿态钉在**行星体固系**（`InitLanded` 一次写入，保留落地朝向）；
/// 每拍 [`apply_landed_nail`] 用当前 `PlanetOrientation.rot` 转到惯性系并共转速度。
#[derive(Debug, Clone)]
pub struct LandedState {
    pub active: bool,
    /// 落定点（行星体固系）[m]。
    pub pos_body: Vec3,
    /// 落定姿态（体固系：vessel → 行星体固），对齐 Orbiter `land_rot`。
    pub r_body: Matrix3,
}

impl Default for LandedState {
    fn default() -> Self {
        Self {
            active: false,
            pos_body: Vec3::ZERO,
            r_body: Matrix3::IDENTITY,
        }
    }
}

/// 落定后每拍钉死：`R·r_body` + `ω×r`（与台架钉死同源变换）。
pub fn apply_landed_nail(asm: &mut Assembly, landed: &LandedState) {
    if !landed.active {
        return;
    }
    let rot = asm.planet_orientation.rot;
    let pos = to_inertial(rot, landed.pos_body);
    let vel = surface_inertial_velocity(pos, asm.planet_orientation.omega);
    let lock_r = rot.matmul(landed.r_body);
    let lock_q = Quat::from_matrix(lock_r);
    for v in &mut asm.vessels {
        if !v.detached {
            v.state.pos = pos;
            v.state.vel = vel;
            v.state.omega = Vec3::ZERO;
            v.state.q = lock_q;
            v.state.r = lock_r;
        }
    }
    let root = asm.root.min(asm.vessels.len().saturating_sub(1));
    asm.state = asm.vessels[root].state;
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
    landed: &mut LandedState,
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

    // —— CheckLanded：无动力 + 体固相对位姿稳定 5 s ——
    let thrust_off = effective_thrust_n <= LAND_THRUST_EPS_N;
    if !thrust_off {
        land_dwell.clear();
        return ContactOutcome::None;
    }

    let (pos_body, q_body) = dwell_body_sample(asm);
    if land_dwell.testing {
        let dt = match land_dwell.since {
            Some(t0) => (sim_t_s - t0).max(1e-9),
            None => {
                land_dwell.since = Some(sim_t_s);
                land_dwell.surf_pos = Some(pos_body);
                land_dwell.surf_q = Some(q_body);
                return ContactOutcome::None;
            }
        };
        // Orbiter：dt < 0.1 不采样
        if dt < 0.1 {
            return ContactOutcome::None;
        }
        let prev_pos = land_dwell.surf_pos.unwrap_or(pos_body);
        let prev_q = land_dwell.surf_q.unwrap_or(q_body);
        let dst2 = (pos_body - prev_pos).length2();
        let dangle = quat_angle(prev_q, q_body);
        land_dwell.surf_pos = Some(pos_body);
        land_dwell.surf_q = Some(q_body);
        if dst2 / dt > LAND_MOVE_RATE2 || dangle / dt > LAND_ANGLE_RATE {
            land_dwell.since = Some(sim_t_s);
            return ContactOutcome::None;
        }
        let t0 = land_dwell.since.unwrap_or(sim_t_s);
        if sim_t_s - t0 < LAND_STABLE_S {
            return ContactOutcome::None;
        }
        init_landed(asm, landed);
        land_dwell.clear();
        return ContactOutcome::Landed { force_n };
    }

    // 开始计时
    land_dwell.testing = true;
    land_dwell.since = Some(sim_t_s);
    land_dwell.surf_pos = Some(pos_body);
    land_dwell.surf_q = Some(q_body);
    ContactOutcome::None
}

/// 体固相对采样：位置 `Rᵀ·pos`、姿态 `Quat(Rᵀ·r)`。
fn dwell_body_sample(asm: &Assembly) -> (Vec3, Quat) {
    let rot = asm.planet_orientation.rot;
    let pos_body = to_body(rot, asm.state.pos);
    let r_body = rot.transp().matmul(asm.state.r);
    (pos_body, Quat::from_matrix(r_body))
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

/// InitLanded（对齐 Orbiter 动态触地）：保留落地朝向，底部贴地，速度共转，
/// 体固钉姿写入 [`LandedState`]（之后每拍 [`apply_landed_nail`]）。
pub fn init_landed(asm: &mut Assembly, landed: &mut LandedState) {
    for v in &mut asm.vessels {
        if !v.detached {
            v.state.omega = Vec3::ZERO;
        }
    }
    asm.state.omega = Vec3::ZERO;

    let pos = asm.state.pos;
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
    let pad_vel = surface_inertial_velocity(new_pos, asm.planet_orientation.omega);
    for v in &mut asm.vessels {
        if !v.detached {
            v.state.vel = pad_vel;
        }
    }
    asm.state.vel = pad_vel;
    let root = asm.root.min(asm.vessels.len().saturating_sub(1));
    asm.state = asm.vessels[root].state;

    let rot = asm.planet_orientation.rot;
    // Orbiter: land_rot = R_planetᵀ · R_inertial（保留落地朝向，不硬掰竖直）。
    landed.pos_body = to_body(rot, asm.state.pos);
    landed.r_body = rot.transp().matmul(asm.state.r);
    landed.active = true;
    apply_landed_nail(asm, landed);
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
    let v_ground = vel - surface_inertial_velocity(pos, asm.planet_orientation.omega);
    -dot(v_ground, radial)
}

#[cfg(test)]
mod tests {
    use super::*;
    use orbitx_dynamics::PlanetOrientation;
    use orbitx_math::{Matrix3, StateVectors, PI2};
    use orbitx_vessel::{presets, GroundSampler, GroundContact, SurfaceKind};
    use std::sync::Arc;

    const SID: f64 = 86_164.0;
    fn omega_y() -> Vec3 {
        Vec3::new(0.0, PI2 / SID, 0.0)
    }
    fn orient_y() -> PlanetOrientation {
        PlanetOrientation {
            rot: Matrix3::IDENTITY,
            omega: omega_y(),
        }
    }

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
        asm.sid_rot_period = SID;
        asm.planet_orientation = orient_y();
        asm.ground = GroundContact::Sphere { radius_m: r };
        asm
    }

    fn vel_down(pos: Vec3, down_mps: f64) -> Vec3 {
        surface_inertial_velocity(pos, omega_y()) + Vec3::new(-down_mps, 0.0, 0.0)
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
        let vel = surface_inertial_velocity(pos, omega_y());
        let mut asm = pad_asm(pos, vel);
        asm.last_surface_contact = soft_contact(1e3);
        let mut dwell = LandDwellState::default();
        let mut landed = LandedState::default();
        for i in 0..20 {
            let t = i as f64 * 0.2; // 0..3.8 s < 5 s
            let out = apply_contact_checks(
                &mut asm, true, true, true, 0.0, &mut dwell, &mut landed, t,
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
        let mut landed = LandedState::default();
        let out = apply_contact_checks(
            &mut asm, true, true, true, 0.0, &mut dwell, &mut landed, 1.0,
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
        let vel = surface_inertial_velocity(pos, omega_y());
        let mut asm = pad_asm(pos, vel);
        asm.last_surface_contact = soft_contact(1e3);
        let mut dwell = LandDwellState::default();
        let mut landed = LandedState::default();
        for i in 0..300 {
            let t = i as f64 * 0.02;
            let out = apply_contact_checks(
                &mut asm, true, true, true, 1e5, &mut dwell, &mut landed, t,
            );
            assert_eq!(out, ContactOutcome::None, "t={t}");
        }
        assert!(!dwell.testing);
    }

    /// 绕 +Y 推进 `rot`，惯性 pos/姿态随共转（体固钉死量不变）。
    fn set_corotating_pose(asm: &mut Assembly, pos_body: Vec3, r_body: Matrix3, ang_rad: f64) {
        let (c, s) = (ang_rad.cos(), ang_rad.sin());
        let rot = Matrix3::new(c, 0.0, s, 0.0, 1.0, 0.0, -s, 0.0, c);
        let omega = omega_y();
        asm.planet_orientation = PlanetOrientation { rot, omega };
        let pos = to_inertial(rot, pos_body);
        let vel = surface_inertial_velocity(pos, omega);
        let lock_r = rot.matmul(r_body);
        let lock_q = Quat::from_matrix(lock_r);
        asm.state.pos = pos;
        asm.state.vel = vel;
        asm.state.r = lock_r;
        asm.state.q = lock_q;
        asm.state.omega = Vec3::ZERO;
        for v in &mut asm.vessels {
            if !v.detached {
                v.state.pos = pos;
                v.state.vel = vel;
                v.state.r = lock_r;
                v.state.q = lock_q;
                v.state.omega = Vec3::ZERO;
            }
        }
    }

    #[test]
    fn stable_five_seconds_lands_while_corotating() {
        // 共转 + 推进 rot、不冻惯性 pos：体固相对静止 → 满 ~5 s 应 Landed。
        let r = 6_371_000.0;
        let pos_body = Vec3::new(r + 10.0, 0.0, 0.0);
        let r_body = Matrix3::IDENTITY;
        let mut asm = pad_asm(pos_body, Vec3::ZERO);
        asm.last_surface_contact = soft_contact(1e3);
        let mut dwell = LandDwellState::default();
        let mut landed_st = LandedState::default();
        let mut got_landed = false;
        let omega = PI2 / SID;
        for i in 0..40 {
            let t = i as f64 * 0.2;
            set_corotating_pose(&mut asm, pos_body, r_body, omega * t);
            let out = apply_contact_checks(
                &mut asm, true, true, true, 0.0, &mut dwell, &mut landed_st, t,
            );
            if matches!(out, ContactOutcome::Landed { .. }) {
                got_landed = true;
                break;
            }
        }
        assert!(got_landed, "expected Landed after ~5s body-relative stable");
        assert!(landed_st.active, "LandedState should be active");
        let clearance = asm.bottom_clearance().unwrap();
        assert!(
            clearance.abs() < 1.0,
            "after InitLanded clearance {clearance}"
        );
    }

    #[test]
    fn body_slide_resets_dwell() {
        // 体固平移超门限 → since 重置（相对滑动，不是惯性共转）。
        let r = 6_371_000.0;
        let mut pos_body = Vec3::new(r - 1.0, 0.0, 0.0);
        let r_body = Matrix3::IDENTITY;
        let mut asm = pad_asm(pos_body, Vec3::ZERO);
        asm.last_surface_contact = soft_contact(1e3);
        let mut dwell = LandDwellState::default();
        let mut landed = LandedState::default();
        let omega = PI2 / SID;
        for i in 0..10 {
            let t = i as f64 * 0.2;
            set_corotating_pose(&mut asm, pos_body, r_body, omega * t);
            let _ = apply_contact_checks(
                &mut asm, true, true, true, 0.0, &mut dwell, &mut landed, t,
            );
        }
        assert!(dwell.testing);
        let t0 = dwell.since.unwrap();
        pos_body.x += 10.0;
        let t_slide = 2.2;
        set_corotating_pose(&mut asm, pos_body, r_body, omega * t_slide);
        let _ = apply_contact_checks(
            &mut asm, true, true, true, 0.0, &mut dwell, &mut landed, t_slide,
        );
        assert!(dwell.since.unwrap() >= t0 + 0.1);
        assert!(!landed.active);
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
        let mut landed = LandedState::default();
        let out = apply_contact_checks(
            &mut asm, true, false, true, 0.0, &mut dwell, &mut landed, 1.0,
        );
        assert_eq!(out, ContactOutcome::ReturnedToPad);
        assert!(!asm.vessels[asm.active].crashed);
    }

    #[test]
    fn large_force_crashes_without_speed() {
        let r = 6_371_000.0;
        let pos = Vec3::new(r - 1.0, 0.0, 0.0);
        // 共转 ≈ 零撞击速度
        let vel = surface_inertial_velocity(pos, omega_y());
        let mut asm = pad_asm(pos, vel);
        let mass = asm.total_mass();
        let huge = crash_force_limit_n(mass) * 2.0;
        asm.last_surface_contact = hard_contact(huge, 0.05);
        let mut dwell = LandDwellState::default();
        let mut landed = LandedState::default();
        let out = apply_contact_checks(
            &mut asm, true, true, true, 0.0, &mut dwell, &mut landed, 1.0,
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
        let vel = surface_inertial_velocity(pos, omega_y());
        let mut asm = pad_asm(pos, vel);
        asm.last_surface_contact = hard_contact(1.0, CRASH_MAX_PENETRATION_M + 0.1);
        let mut dwell = LandDwellState::default();
        let mut landed = LandedState::default();
        let out = apply_contact_checks(
            &mut asm, true, true, true, 0.0, &mut dwell, &mut landed, 1.0,
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
        let mut landed = LandedState::default();
        let out = apply_contact_checks(
            &mut asm, true, false, true, 0.0, &mut dwell, &mut landed, 1.0,
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
        let mut landed = LandedState::default();
        assert_eq!(
            apply_contact_checks(
                &mut asm, true, false, true, 0.0, &mut dwell, &mut landed, 1.0,
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
        asm.ground = GroundContact::query(Arc::new(WaterSampler), Matrix3::IDENTITY);
        asm.last_surface_contact = soft_contact(1e3);
        let mut dwell = LandDwellState::default();
        let mut landed = LandedState::default();
        let out = apply_contact_checks(
            &mut asm, true, true, true, 0.0, &mut dwell, &mut landed, 1.0,
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
        let mut landed = LandedState::default();
        assert_eq!(
            apply_contact_checks(
                &mut asm, true, true, true, 0.0, &mut dwell, &mut landed, 1.0,
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
        let mut landed = LandedState::default();
        assert_eq!(
            apply_contact_checks(
                &mut asm, true, true, true, 0.0, &mut dwell, &mut landed, 1.0,
            ),
            ContactOutcome::None
        );
    }

    fn mat3_close(a: Matrix3, b: Matrix3, eps: f64) -> bool {
        (a.m11 - b.m11).abs() < eps
            && (a.m12 - b.m12).abs() < eps
            && (a.m13 - b.m13).abs() < eps
            && (a.m21 - b.m21).abs() < eps
            && (a.m22 - b.m22).abs() < eps
            && (a.m23 - b.m23).abs() < eps
            && (a.m31 - b.m31).abs() < eps
            && (a.m32 - b.m32).abs() < eps
            && (a.m33 - b.m33).abs() < eps
    }

    #[test]
    fn init_landed_preserves_attitude_and_corotates() {
        // 非竖直落地朝向 → InitLanded 不得硬掰；推进 rot 后体固相对姿态不变。
        let r = 6_371_000.0;
        let pos = Vec3::new(r + 20.0, 0.0, 0.0);
        let vel = surface_inertial_velocity(pos, omega_y());
        let mut asm = pad_asm(pos, vel);
        // 绕 +X（当地径向）转 30°。
        let a = 30.0_f64.to_radians();
        let (ca, sa) = (a.cos(), a.sin());
        let r0 = Matrix3::new(1.0, 0.0, 0.0, 0.0, ca, -sa, 0.0, sa, ca);
        let q0 = Quat::from_matrix(r0);
        asm.state.r = r0;
        asm.state.q = q0;
        for v in &mut asm.vessels {
            if !v.detached {
                v.state.r = r0;
                v.state.q = q0;
                v.state.pos = pos;
                v.state.vel = vel;
            }
        }

        let mut landed = LandedState::default();
        init_landed(&mut asm, &mut landed);
        assert!(landed.active);
        assert!(
            mat3_close(asm.state.r, r0, 1e-9),
            "attitude wiped at InitLanded"
        );
        let r_body0 = landed.r_body;

        let ang = 15.0_f64.to_radians();
        let (c, s) = (ang.cos(), ang.sin());
        let rot = Matrix3::new(c, 0.0, s, 0.0, 1.0, 0.0, -s, 0.0, c);
        asm.planet_orientation = PlanetOrientation {
            rot,
            omega: omega_y(),
        };
        apply_landed_nail(&mut asm, &landed);
        let expect = rot.matmul(r_body0);
        assert!(
            mat3_close(asm.state.r, expect, 1e-9),
            "corotation attitude mismatch"
        );
        assert!(mat3_close(landed.r_body, r_body0, 1e-15));
        let pos_i = asm.state.pos;
        let v_expect = surface_inertial_velocity(pos_i, omega_y());
        assert!((asm.state.vel - v_expect).length() < 1e-6);
        assert_eq!(asm.state.omega, Vec3::ZERO);
    }
}
