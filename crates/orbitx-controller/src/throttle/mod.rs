//! 油门组合策略 `ThrottlePolicy` + `apply_throttle`。
//!
//! 物理层 `Assembly::set_throttle` 只设 active 级；`Vessel::set_throttle` 逐船设。本模块
//! 决定组合方式：`ActiveOnly`（仅 active，历史行为）与 `SyncPrimary`（同步 lit 集：
//! active ∪ 侧挂叶，同轴非 lit 有推保持 0）。点火集从对接图动态算，不按航天器名/class 特判。
//!
//! 命令经 `caps.throttle_groups` 的 vessel_index 落到物理层，不硬编码索引。

use crate::capability::{BodyRef, ControlCapability};
use orbitx_vessel::{Assembly, ThrustReadout};

/// 油门组合策略。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ThrottlePolicy {
    /// 仅 `active` vessel（历史行为）。
    ActiveOnly,
    /// 同步 lit 集：`active` ∪ 侧挂叶（有推）；同轴非 lit 有推保持 0。
    SyncPrimary,
}

/// 按策略设置油门（逐船调用 `Vessel::set_throttle`，不改 `Assembly::set_throttle` 语义）。
///
/// `caps.body` 决定 lit 集来源：主组合体用对接图动态 lit 集；detached 单体即自身。
pub fn apply_throttle(
    asm: &mut Assembly,
    caps: &ControlCapability,
    policy: ThrottlePolicy,
    level: f64,
) {    match policy {
        ThrottlePolicy::ActiveOnly => {
            let vi = body_vessel_index(asm, caps);
            if vi < asm.vessels.len() {
                asm.vessels[vi].set_throttle(level);
            }
        }
        ThrottlePolicy::SyncPrimary => {
            // 先收集 lit 集（消费迭代器借用），再逐船写入，避免借用冲突。
            let lit: Vec<usize> = match caps.body {
                BodyRef::Primary => asm.lit_thrusting_indices().collect(),
                BodyRef::Detached(idx) => vec![idx],
            };
            for g in &caps.throttle_groups {
                let vi = g.vessel_index;
                if vi >= asm.vessels.len() {
                    continue;
                }
                let thr = if lit.contains(&vi) { level } else { 0.0 };
                asm.vessels[vi].set_throttle(thr);
            }
        }
    }
}

/// 仅设指定 throttle group 对应 vessel 的油门（按 group id 命令单船，不改变其他船）。
/// group id 未命中则无操作。
pub fn apply_group_throttle(
    asm: &mut Assembly,
    caps: &ControlCapability,
    group_id: &str,
    level: f64,
) {
    let Some(g) = caps.throttle_groups.iter().find(|g| g.id == group_id) else {
        return;
    };
    let vi = g.vessel_index;
    if let Some(v) = asm.vessels.get_mut(vi) {
        v.set_throttle(level);
    }
}

/// 本体当前参考 vessel 下标：主组合体 → active；detached → 该 vessel。
pub(crate) fn body_vessel_index(asm: &Assembly, caps: &ControlCapability) -> usize {
    match caps.body {
        BodyRef::Primary => asm.active,
        BodyRef::Detached(idx) => idx,
    }
}

#[cfg(test)]
mod tests;
