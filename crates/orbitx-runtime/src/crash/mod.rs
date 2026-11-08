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
    Crash {
        water: bool,
        force_n: f64,
    },
    /// 着陆成功（已校正贴地）。
    Landed {
        force_n: f64,
    },
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
    let primary: HashSet<usize> = asm.components.iter().map(|c| c.vessel_index).collect();

    if matches!(asm.ground, GroundContact::Off) {
        land_dwell.clear();
        // 地面注入暂停（高空 Paused）也要给独立体终局：坠入行星内部的
        // 分离残骸曾无限期滞留（crash=0），直到地面注入恢复的瞬间被
        // `detached_is_crash` 集体标毁（2026-11-08 S3 于 10,698 km
        // 「坠入水面」根因③）。口径与触地一致用底部点；球半径是 Off
        // 期间可用的最保守地面（无地形采样）。静默终局：残骸销毁不是
        // 任务事件，不上报文案。
        mark_underground_detached_crashed(asm, &primary);
        return ContactOutcome::None;
    }

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
    let in_contact =
        contact.in_contact || asm.bottom_clearance().map(|c| c <= 0.0).unwrap_or(false);

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

/// 地面注入暂停（`GroundContact::Off`）期间的独立体终局：底部点低于行星
/// 球半径 = 已坠入行星内部，直接标毁（速度清零、步进跳过）。
fn mark_underground_detached_crashed(asm: &mut Assembly, primary: &HashSet<usize>) {
    for i in 0..asm.vessels.len() {
        if asm.vessels[i].crashed || primary.contains(&i) {
            continue;
        }
        let bottom = Assembly::vessel_bottom_point(&asm.vessels[i]);
        if bottom.length() < asm.planet_radius {
            asm.mark_crashed(i);
        }
    }
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
    let ground_r = asm.ground.radius_at_world(pos).unwrap_or(asm.planet_radius);
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
    let d = (a.s * b.s + a.vx * b.vx + a.vy * b.vy + a.vz * b.vz)
        .abs()
        .clamp(0.0, 1.0);
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
mod tests;
