//! CLI 过渡控制策略（日后迁入 `orbitx-controller`）。
//!
//! 物理层只提供单船 `set_throttle` / `undock`；本模块决定组合方式。
//! 点火集只认对接图 + 单值 `active`，不按航天器名 / class 特判。

use orbitx_math::{dot, Vec3};
use orbitx_vessel::Assembly;

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

/// lit 集有推船推力之和（HUD）。
pub fn primary_thrust_sum(asm: &Assembly) -> f64 {
    lit_thrusting_indices(asm)
        .into_iter()
        .map(|i| asm.vessels[i].current_thrust())
        .sum()
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
    use orbitx_vessel::{DockPort, StageSpec};

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
            StageSpec {
                name: "Core",
                dry_mass: 1000.0,
                fuel_mass: 1000.0,
                thrust: 1000.0,
                isp: 300.0,
                engine_dir: Vec3::new(0.0, 1.0, 0.0),
                engine_pos: Vec3::new(0.0, -5.0, 0.0),
                length: 10.0,
                radius: 1.0,
                separation_impulse: 1.0,
                docks: None,
                ..Default::default()
            },
            StageSpec {
                name: "Upper",
                dry_mass: 200.0,
                fuel_mass: 500.0,
                thrust: 400.0,
                isp: 300.0,
                engine_dir: Vec3::new(0.0, 1.0, 0.0),
                engine_pos: Vec3::new(0.0, -2.0, 0.0),
                length: 4.0,
                radius: 1.0,
                separation_impulse: 1.0,
                docks: None,
                ..Default::default()
            },
        ]
    }

    /// 芯 + 同轴上级（有推）+ 侧挂叶。
    fn core_upper_and_booster() -> (Vec<StageSpec>, Vec<(usize, usize, usize, usize)>) {
        let core = StageSpec {
            name: "Core",
            dry_mass: 1000.0,
            fuel_mass: 1000.0,
            thrust: 1000.0,
            isp: 300.0,
            engine_dir: Vec3::new(0.0, 1.0, 0.0),
            engine_pos: Vec3::new(0.0, -5.0, 0.0),
            length: 10.0,
            radius: 1.0,
            separation_impulse: 1.0,
            docks: Some(vec![
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
            ]),
            ..Default::default()
        };
        let upper = StageSpec {
            name: "Upper",
            dry_mass: 200.0,
            fuel_mass: 500.0,
            thrust: 400.0,
            isp: 300.0,
            engine_dir: Vec3::new(0.0, 1.0, 0.0),
            engine_pos: Vec3::new(0.0, -2.0, 0.0),
            length: 4.0,
            radius: 1.0,
            separation_impulse: 1.0,
            docks: None,
            ..Default::default()
        };
        let booster = StageSpec {
            name: "Booster",
            dry_mass: 500.0,
            fuel_mass: 500.0,
            thrust: 2000.0,
            isp: 300.0,
            engine_dir: Vec3::new(0.0, 1.0, 0.0),
            engine_pos: Vec3::new(0.0, -4.0, 0.0),
            length: 8.0,
            radius: 0.5,
            separation_impulse: 2.0,
            docks: Some(vec![DockPort::with_rot(
                Vec3::new(-0.5, 0.0, 0.0),
                Vec3::new(-1.0, 0.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
            )]),
            ..Default::default()
        };
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
}
