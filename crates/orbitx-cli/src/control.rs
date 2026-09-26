//! CLI 过渡控制策略（日后迁入 `orbitx-controller`）。
//!
//! 物理层只提供单船 `set_throttle` / `undock`；本模块决定组合方式。
//! 点火集只认对接图 + 单值 `active`，不按航天器名 / class 特判。

use orbitx_math::{dot, Vec3};
use orbitx_vessel::{
    attitude_errors as vessel_attitude_errors, pitch_yaw_angles as vessel_pitch_yaw,
    roll_angle as vessel_roll, tip_angle as vessel_tip, Assembly,
};

/// TVC PD 增益（与 CLI 竖直保持 / 重力转向共用）。
pub const TVC_KP: f64 = 1.0;
pub const TVC_KD: f64 = 2.0;

/// 油门组合策略。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ThrottlePolicy {
    /// 仅 `active` vessel（历史行为）。
    ActiveOnly,
    /// 同步 lit 集：`active` ∪ 侧挂叶（有推）；同轴非 lit 有推保持 0。
    SyncPrimary,
}

fn in_primary(asm: &Assembly, idx: usize) -> bool {
    asm.components.iter().any(|c| c.vessel_index == idx)
}

fn has_main_thrust(asm: &Assembly, idx: usize) -> bool {
    asm.vessels[idx]
        .thrusters
        .iter()
        .any(|t| t.max_thrust > 0.0)
}

/// 船主推轴（有推 thruster 的 `base_dir`）；无则 `None`。
fn thrust_axis(asm: &Assembly, idx: usize) -> Option<Vec3> {
    let t = asm.vessels[idx]
        .thrusters
        .iter()
        .find(|t| t.max_thrust > 0.0)?;
    let len = t.base_dir.length();
    if len < 1e-9 {
        None
    } else {
        Some(t.base_dir * (1.0 / len))
    }
}

/// 叶挂在 mate 口上是否为侧向（对接 dir 与 mate 主推轴近似正交）。
/// 同轴堆叠口与主推轴对齐，不得当作侧挂。
fn is_lateral_on_mate(asm: &Assembly, leaf_idx: usize, mate_idx: usize, mate_port: usize) -> bool {
    let Some(port) = asm.vessels[mate_idx].docks.get(mate_port) else {
        return false;
    };
    let dlen = port.dir.length();
    if dlen < 1e-9 {
        return false;
    }
    let dock_dir = port.dir * (1.0 / dlen);
    let axis = thrust_axis(asm, mate_idx)
        .or_else(|| thrust_axis(asm, leaf_idx))
        .unwrap_or(Vec3::new(0.0, 1.0, 0.0));
    // |cosθ| < 0.5 ⇒ 夹角 > 60°，视为侧挂而非同轴堆叠。
    dot(dock_dir, axis).abs() < 0.5
}

/// 主组合体中有主推（`max_thrust > 0`）的 vessel 下标。
pub fn primary_thrusting_indices(asm: &Assembly) -> Vec<usize> {
    asm.components
        .iter()
        .map(|c| c.vessel_index)
        .filter(|&i| !asm.vessels[i].detached && has_main_thrust(asm, i))
        .collect()
}

/// 侧挂叶：度 1、mate 度 ≥ 2、非 `active`、在主组合体、且挂在 mate 的侧向口上。
/// 返回 `(vessel_index, port_on_leaf)`，按 vessel 下标排序。
pub fn strap_on_leaf_indices(asm: &Assembly) -> Vec<(usize, usize)> {
    let mut out = Vec::new();
    for (i, v) in asm.vessels.iter().enumerate() {
        if v.detached || i == asm.active || !in_primary(asm, i) {
            continue;
        }
        if dock_degree(asm, i) != 1 {
            continue;
        }
        let Some((port, mate_id, mate_port)) = v.docks.iter().enumerate().find_map(|(p, d)| {
            d.connected_to
                .map(|(id, mp)| (p, id, mp))
        }) else {
            continue;
        };
        let Some(mate_idx) = asm.vessels.iter().position(|x| x.id == mate_id) else {
            continue;
        };
        if asm.vessels[mate_idx].detached || dock_degree(asm, mate_idx) < 2 {
            continue;
        }
        if !is_lateral_on_mate(asm, i, mate_idx, mate_port) {
            continue;
        }
        out.push((i, port));
    }
    out.sort_by_key(|&(i, _)| i);
    out
}

/// 本帧应响应油门的有推船：`active` ∪ 侧挂叶（均须有主推、在主组合体）。
pub fn lit_thrusting_indices(asm: &Assembly) -> Vec<usize> {
    let strap: Vec<usize> = strap_on_leaf_indices(asm)
        .into_iter()
        .map(|(i, _)| i)
        .filter(|&i| has_main_thrust(asm, i))
        .collect();
    primary_thrusting_indices(asm)
        .into_iter()
        .filter(|&i| i == asm.active || strap.iter().any(|&s| s == i))
        .collect()
}

/// 有符号 tip 分量（体轴，≈ sin θ）：相对径向。活动级。
pub fn attitude_errors(asm: &Assembly) -> (f64, f64) {
    attitude_errors_at(asm, asm.active)
}

/// 有符号 tip 分量：指定 vessel（读本船 `state`，与步进几何同一实现）。
pub fn attitude_errors_at(asm: &Assembly, vessel_index: usize) -> (f64, f64) {
    let Some(v) = asm.vessels.get(vessel_index) else {
        return (0.0, 0.0);
    };
    vessel_attitude_errors(&v.state)
}

/// 有符号俯仰/偏航角 [rad]：活动级。
pub fn pitch_yaw_angles(asm: &Assembly) -> (f64, f64) {
    pitch_yaw_angles_at(asm, asm.active)
}

/// 有符号俯仰/偏航角 [rad]：指定 vessel。
pub fn pitch_yaw_angles_at(asm: &Assembly, vessel_index: usize) -> (f64, f64) {
    let Some(v) = asm.vessels.get(vessel_index) else {
        return (0.0, 0.0);
    };
    vessel_pitch_yaw(&v.state)
}

/// 体 +Y 与径向无符号夹角 [rad]（总 tip）：活动级。
pub fn tip_angle(asm: &Assembly) -> f64 {
    tip_angle_at(asm, asm.active)
}

/// 体 +Y 与径向无符号夹角 [rad]：指定 vessel。
pub fn tip_angle_at(asm: &Assembly, vessel_index: usize) -> f64 {
    let Some(v) = asm.vessels.get(vessel_index) else {
        return 0.0;
    };
    vessel_tip(&v.state)
}

/// 绕体 +Y 滚转角 [rad]：活动级。
pub fn roll_angle(asm: &Assembly) -> f64 {
    roll_angle_at(asm, asm.active)
}

/// 绕体 +Y（纵轴）的滚转角 [rad]（HUD）：指定 vessel。
pub fn roll_angle_at(asm: &Assembly, vessel_index: usize) -> f64 {
    let Some(v) = asm.vessels.get(vessel_index) else {
        return 0.0;
    };
    vessel_roll(&v.state)
}

/// lit 集有推船推力之和 [N]（`Vessel::current_thrust`，已门控燃料；供松台架/控制）。
/// 遥测展示请读各船 `diagnostics.thrust`（步进写入）。
pub fn primary_thrust_sum(asm: &Assembly) -> f64 {
    let p = asm.ambient_pressure();
    lit_thrusting_indices(asm)
        .into_iter()
        .map(|i| asm.vessels[i].current_thrust(p))
        .sum()
}

/// 双轴 TVC PD：仅 lit 集主推；`pitch_target` / `yaw_target` 为期望有符号 tip 角 [rad]（竖直=0）。
///
/// `gimbal = −(Kp·err + Kd·ω)`：P/D 同号反对 tip 与 tip-rate（植物：+gimbal → +err）。
/// 滚转无执行器，不在此闭环。
pub fn apply_tvc(asm: &mut Assembly, pitch_target: f64, yaw_target: f64, dt: f64) {
    let (p, y) = pitch_yaw_angles(asm);
    let err_p = p - pitch_target;
    let err_y = y - yaw_target;
    let w = asm.vessels[asm.active].state.omega;
    let cmd_p = TVC_KP * err_p + TVC_KD * w.x;
    let cmd_y = TVC_KP * err_y + TVC_KD * w.z;

    let lit = lit_thrusting_indices(asm);
    for &vi in &lit {
        let n_main = asm.vessels[vi]
            .n_main_thrusters
            .min(asm.vessels[vi].thrusters.len());
        for t in &mut asm.vessels[vi].thrusters[..n_main] {
            if t.max_gimbal > 0.0 {
                t.slew_gimbal(-cmd_p, -cmd_y, dt);
            }
        }
    }
}

/// 按策略设置油门（逐船调用物理原语，不改 `Assembly::set_throttle` 语义）。
pub fn apply_throttle(asm: &mut Assembly, policy: ThrottlePolicy, level: f64) {
    match policy {
        ThrottlePolicy::ActiveOnly => {
            asm.set_throttle(level);
        }
        ThrottlePolicy::SyncPrimary => {
            let lit = lit_thrusting_indices(asm);
            for i in primary_thrusting_indices(asm) {
                let thr = if lit.iter().any(|&j| j == i) {
                    level
                } else {
                    0.0
                };
                asm.vessels[i].set_throttle(thr);
            }
        }
    }
}

/// 未分离船在对接图上的度数（已占用口数量）。
fn dock_degree(asm: &Assembly, idx: usize) -> usize {
    asm.vessels[idx]
        .docks
        .iter()
        .filter(|d| {
            d.connected_to
                .map(|(id, _)| {
                    asm.vessels
                        .iter()
                        .find(|v| v.id == id)
                        .map(|v| !v.detached)
                        .unwrap_or(false)
                })
                .unwrap_or(false)
        })
        .count()
}

/// 侧挂叶候选：优先空燃料有推叶。
/// 排除 active 以免同轴底级被误当作侧挂；同轴仍走 `separate_stage`。
/// 返回 `(vessel_index, port_on_leaf)`。
pub fn pick_strap_on_leaf(asm: &Assembly) -> Option<(usize, usize)> {
    let mut candidates: Vec<(usize, usize, bool, bool)> = Vec::new();
    // (leaf_idx, port, empty_fuel, has_thrust)
    for (i, port) in strap_on_leaf_indices(asm) {
        let v = &asm.vessels[i];
        let has_thrust = has_main_thrust(asm, i);
        let empty = v.fuel_mass < 1.0;
        candidates.push((i, port, empty && has_thrust, has_thrust));
    }
    // 优先：空燃料有推 → 有推 → 最低下标
    candidates.sort_by_key(|&(i, _, empty_thrust, has_thrust)| {
        let prio = if empty_thrust {
            0
        } else if has_thrust {
            1
        } else {
            2
        };
        (prio, i)
    });
    candidates.first().map(|&(i, p, _, _)| (i, p))
}

/// 是否应自动分离（上层判据）。
pub fn should_auto_separate(asm: &Assembly) -> bool {
    if asm.stage_count() <= 1 {
        return false;
    }
    if let Some((leaf_idx, _)) = pick_strap_on_leaf(asm) {
        let v = &asm.vessels[leaf_idx];
        if v.fuel_mass < 1.0 && has_main_thrust(asm, leaf_idx) {
            return true;
        }
    }
    let active = &asm.vessels[asm.active];
    active.fuel_mass < 1.0 && has_main_thrust(asm, asm.active)
}

/// 执行一次分离：有侧挂叶则 `undock`，否则同轴 `separate_stage`。
pub fn perform_separate(asm: &mut Assembly) {
    if let Some((leaf_idx, port)) = pick_strap_on_leaf(asm) {
        let id = asm.vessels[leaf_idx].id;
        let sep = asm.vessels[leaf_idx].separation_impulse;
        let _ = asm.undock(id, port, sep);
        return;
    }
    if asm.stage_count() > 1 {
        asm.separate_stage();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use orbitx_math::{StateVectors, Vec3};
    use orbitx_vessel::{DockPort, StageSpec, StepEnv};

    fn thruster_level(asm: &Assembly, idx: usize) -> f64 {
        asm.vessels[idx]
            .thrusters
            .first()
            .map(|t| t.level)
            .unwrap_or(0.0)
    }

    /// 同轴两级均有推、无侧挂。
    fn coaxial_two_stage() -> Vec<StageSpec> {
        vec![
            StageSpec::with_single_thruster(
                "Core",
                1000.0,
                1000.0,
                1000.0,
                300.0,
                Vec3::new(0.0, -5.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
                10.0,
                1.0,
                1.0,
            ),
            StageSpec::with_single_thruster(
                "Upper",
                200.0,
                500.0,
                400.0,
                300.0,
                Vec3::new(0.0, -2.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
                4.0,
                1.0,
                1.0,
            ),
        ]
    }

    /// 芯 + 同轴上级（有推）+ 侧挂叶。
    fn core_upper_and_booster() -> (Vec<StageSpec>, Vec<(usize, usize, usize, usize)>) {
        let mut core = StageSpec::with_single_thruster(
            "Core",
            1000.0,
            1000.0,
            1000.0,
            300.0,
            Vec3::new(0.0, -5.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            10.0,
            1.0,
            1.0,
        );
        core.docks = Some(vec![
            DockPort::with_rot(
                Vec3::new(0.0, -5.0, 0.0),
                Vec3::new(0.0, -1.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
            ),
            DockPort::with_rot(
                Vec3::new(0.0, 5.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
            ),
            DockPort::with_rot(
                Vec3::new(2.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
            ),
        ]);
        let upper = StageSpec::with_single_thruster(
            "Upper",
            200.0,
            500.0,
            400.0,
            300.0,
            Vec3::new(0.0, -2.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            4.0,
            1.0,
            1.0,
        );
        let mut booster = StageSpec::with_single_thruster(
            "Booster",
            500.0,
            500.0,
            2000.0,
            300.0,
            Vec3::new(0.0, -4.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            8.0,
            0.5,
            2.0,
        );
        booster.docks = Some(vec![DockPort::with_rot(
            Vec3::new(-0.5, 0.0, 0.0),
            Vec3::new(-1.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        )]);
        // 芯顶↔上级底；芯侧↔助推
        (
            vec![core, upper, booster],
            vec![(0, 1, 1, 0), (0, 2, 2, 0)],
        )
    }

    #[test]
    fn sync_primary_coaxial_lights_active_only() {
        let stages = coaxial_two_stage();
        let mut asm = Assembly::new(&stages, StateVectors::default());
        assert_eq!(asm.active, 0);
        apply_throttle(&mut asm, ThrottlePolicy::SyncPrimary, 1.0);
        assert!((thruster_level(&asm, 0) - 1.0).abs() < 1e-9);
        assert!(thruster_level(&asm, 1).abs() < 1e-9);
        assert!((primary_thrust_sum(&asm) - 1000.0).abs() < 1e-6);
        let lit = lit_thrusting_indices(&asm);
        assert_eq!(lit, vec![0]);
    }

    #[test]
    fn sync_primary_coaxial_after_separate_lights_upper() {
        let stages = coaxial_two_stage();
        let mut asm = Assembly::new(&stages, StateVectors::default());
        apply_throttle(&mut asm, ThrottlePolicy::SyncPrimary, 1.0);
        perform_separate(&mut asm);
        assert!(asm.vessels[0].detached);
        assert_eq!(asm.active, 1);
        apply_throttle(&mut asm, ThrottlePolicy::SyncPrimary, 1.0);
        assert!(thruster_level(&asm, 0).abs() < 1e-9 || asm.vessels[0].detached);
        assert!((thruster_level(&asm, 1) - 1.0).abs() < 1e-9);
        assert!((primary_thrust_sum(&asm) - 400.0).abs() < 1e-6);
    }

    #[test]
    fn sync_primary_lights_core_and_booster_not_upper() {
        let (stages, links) = core_upper_and_booster();
        let mut asm = Assembly::with_dock_links(&stages, StateVectors::default(), &links);
        apply_throttle(&mut asm, ThrottlePolicy::SyncPrimary, 1.0);
        assert!((thruster_level(&asm, 0) - 1.0).abs() < 1e-9);
        assert!(thruster_level(&asm, 1).abs() < 1e-9);
        assert!((thruster_level(&asm, 2) - 1.0).abs() < 1e-9);
        assert!((primary_thrust_sum(&asm) - 3000.0).abs() < 1e-6);
        let mut lit = lit_thrusting_indices(&asm);
        lit.sort();
        assert_eq!(lit, vec![0, 2]);
    }

    #[test]
    fn sync_primary_after_booster_undock_upper_still_idle() {
        let (stages, links) = core_upper_and_booster();
        let mut asm = Assembly::with_dock_links(&stages, StateVectors::default(), &links);
        apply_throttle(&mut asm, ThrottlePolicy::SyncPrimary, 1.0);
        perform_separate(&mut asm);
        assert!(asm.vessels[2].detached);
        apply_throttle(&mut asm, ThrottlePolicy::SyncPrimary, 1.0);
        assert!((thruster_level(&asm, 0) - 1.0).abs() < 1e-9);
        assert!(thruster_level(&asm, 1).abs() < 1e-9);
        assert_eq!(asm.active, 0);
    }

    #[test]
    fn sync_primary_after_core_separate_lights_upper() {
        let (stages, links) = core_upper_and_booster();
        let mut asm = Assembly::with_dock_links(&stages, StateVectors::default(), &links);
        perform_separate(&mut asm); // booster
        perform_separate(&mut asm); // core via separate_stage
        assert!(asm.vessels[0].detached);
        assert_eq!(asm.active, 1);
        apply_throttle(&mut asm, ThrottlePolicy::SyncPrimary, 1.0);
        assert!((thruster_level(&asm, 1) - 1.0).abs() < 1e-9);
        assert!((primary_thrust_sum(&asm) - 400.0).abs() < 1e-6);
    }

    #[test]
    fn active_only_leaves_booster_idle() {
        let (stages, links) = core_upper_and_booster();
        let mut asm = Assembly::with_dock_links(&stages, StateVectors::default(), &links);
        apply_throttle(&mut asm, ThrottlePolicy::ActiveOnly, 1.0);
        assert!((thruster_level(&asm, 0) - 1.0).abs() < 1e-9);
        assert!(thruster_level(&asm, 2).abs() < 1e-9);
    }

    #[test]
    fn perform_separate_undocks_booster_first() {
        let (stages, links) = core_upper_and_booster();
        let mut asm = Assembly::with_dock_links(&stages, StateVectors::default(), &links);
        assert!(pick_strap_on_leaf(&asm).is_some());
        perform_separate(&mut asm);
        assert!(asm.vessels[2].detached);
        assert!(!asm.vessels[0].detached);
        assert_eq!(asm.components.len(), 2);
    }

    #[test]
    fn should_auto_separate_when_booster_empty() {
        let (stages, links) = core_upper_and_booster();
        let mut asm = Assembly::with_dock_links(&stages, StateVectors::default(), &links);
        asm.vessels[2].fuel_mass = 0.0;
        assert!(should_auto_separate(&asm));
    }

    /// 无操作竖直上升：有符号双轴 TVC 保持 tip 有界（旧无符号 pitch 会摇摆坠毁）。
    #[test]
    fn vertical_hold_tip_stays_bounded() {
        use orbitx_dynamics::GravBody;
        use orbitx_math::{cross, Matrix3, Quat};
        use orbitx_vessel::ThrusterSpec;

        let earth_r = 6_371_000.0;
        let earth = GravBody {
            pos: Vec3::ZERO,
            mass: 5.972e24,
            size: earth_r,
            jcoeff: vec![],
            rotation: None,
            pines: None,
        };

        let spec = StageSpec {
            name: "hold",
            dry_mass: 10_000.0,
            fuel_mass: 40_000.0,
            thrusters: vec![ThrusterSpec {
                pos: Vec3::new(0.0, -15.0, 0.0),
                dir: Vec3::new(0.0, 1.0, 0.0),
                thrust: 800_000.0,
                isp: 300.0,
                max_gimbal: 0.15,
                max_gimbal_rate: 1.0,
                gimbal_axis: Vec3::new(1.0, 0.0, 0.0),
                ..Default::default()
            }],
            length: 30.0,
            radius: 1.5,
            ..Default::default()
        };

        let pos = Vec3::new(0.0, 0.0, earth_r + 20.0);
        let up = pos * (1.0 / pos.length());
        let ref_axis = Vec3::new(0.0, 1.0, 0.0);
        let bx = cross(up, ref_axis).unit();
        let bz = cross(bx, up).unit();
        let by = up;
        let rot = Matrix3::new(bx.x, by.x, bz.x, bx.y, by.y, bz.y, bx.z, by.z, bz.z);
        let q = Quat::from_matrix(rot);

        let mut asm = Assembly::new(
            &[spec],
            StateVectors {
                pos,
                vel: Vec3::ZERO,
                // 小初始扰动，迫使闭环介入。
                omega: Vec3::new(0.03, 0.0, -0.02),
                r: rot,
                q,
            },
        );
        asm.planet_radius = earth_r;

        let dt = 0.05;
        let mut max_tip = 0.0_f64;
        for _ in 0..(40.0 / dt) as usize {
            apply_throttle(&mut asm, ThrottlePolicy::SyncPrimary, 1.0);
            apply_tvc(&mut asm, 0.0, 0.0, dt);
            asm.step(dt, StepEnv::primary0(&[earth.clone()]));
            max_tip = max_tip.max(tip_angle(&asm));
        }

        let tip_deg = max_tip.to_degrees();
        assert!(
            tip_deg < 8.0,
            "竖直保持 tip 应 < 8°，实际峰值 {tip_deg:.2}°"
        );
        let h = asm.vessels[asm.active].state.pos.length() - earth_r;
        assert!(h > 100.0, "应明显离地，高度={h:.1} m");
    }

    /// 非零俯仰目标：稳态 pitch 角应逼近目标，而非 asin(目标弧度)≈更大角。
    #[test]
    fn pitch_target_tracks_angle_not_sin() {
        use orbitx_dynamics::GravBody;
        use orbitx_math::{cross, Matrix3, Quat};
        use orbitx_vessel::ThrusterSpec;

        let earth_r = 6_371_000.0;
        let earth = GravBody {
            pos: Vec3::ZERO,
            mass: 5.972e24,
            size: earth_r,
            jcoeff: vec![],
            rotation: None,
            pines: None,
        };

        let spec = StageSpec {
            name: "hold",
            dry_mass: 10_000.0,
            fuel_mass: 40_000.0,
            thrusters: vec![ThrusterSpec {
                pos: Vec3::new(0.0, -15.0, 0.0),
                dir: Vec3::new(0.0, 1.0, 0.0),
                thrust: 800_000.0,
                isp: 300.0,
                max_gimbal: 0.15,
                max_gimbal_rate: 1.0,
                gimbal_axis: Vec3::new(1.0, 0.0, 0.0),
                ..Default::default()
            }],
            length: 30.0,
            radius: 1.5,
            ..Default::default()
        };

        let pos = Vec3::new(0.0, 0.0, earth_r + 20.0);
        let up = pos * (1.0 / pos.length());
        let ref_axis = Vec3::new(0.0, 1.0, 0.0);
        let bx = cross(up, ref_axis).unit();
        let bz = cross(bx, up).unit();
        let by = up;
        let rot = Matrix3::new(bx.x, by.x, bz.x, bx.y, by.y, bz.y, bx.z, by.z, bz.z);
        let q = Quat::from_matrix(rot);

        let mut asm = Assembly::new(
            &[spec],
            StateVectors {
                pos,
                vel: Vec3::ZERO,
                omega: Vec3::ZERO,
                r: rot,
                q,
            },
        );
        asm.planet_radius = earth_r;

        let pitch_tgt = 30.0_f64.to_radians();
        let dt = 0.05;
        for _ in 0..(25.0 / dt) as usize {
            apply_throttle(&mut asm, ThrottlePolicy::SyncPrimary, 1.0);
            apply_tvc(&mut asm, pitch_tgt, 0.0, dt);
            asm.step(dt, StepEnv::primary0(&[earth.clone()]));
        }

        let (p, y) = pitch_yaw_angles(&asm);
        let p_deg = p.to_degrees();
        let wrong_eq = pitch_tgt.asin().to_degrees(); // 旧 bug 稳态 ≈ 31.6°
        assert!(
            (p_deg - 30.0).abs() < 3.0,
            "稳态俯仰应≈30°，实际 {p_deg:.2}°（旧 sin 稳态≈{wrong_eq:.2}°）"
        );
        assert!(
            y.abs().to_degrees() < 5.0,
            "偏航应保持近 0，实际 {:.2}°",
            y.to_degrees()
        );
        // 明确不是旧的 asin(target) 平衡点
        assert!(
            (p_deg - wrong_eq).abs() > 0.5,
            "不应停在旧 asin 平衡点 {wrong_eq:.2}°"
        );
    }
}
