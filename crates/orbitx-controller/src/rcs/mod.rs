//! RCS 姿态/平移控制。
//!
//! 按 rcs group id 委托 `orbitx_vessel::set_attitude_rot`。RCS 不走 throttle_rate 斜坡，
//! 直接设 level。group id 命中 `caps.rcs_groups` 的 vessel_index 决定作用船。

use crate::capability::ControlCapability;
use orbitx_vessel::{set_attitude_rot, Assembly, RotAxis};

/// 按 rcs group id 设置姿态旋转率。
///
/// `group_id` 命中 `caps.rcs_groups` → 取 vessel_index → `set_attitude_rot(vessel, axis, level)`。
/// 未命中则无操作。
pub fn set_rcs(
    asm: &mut Assembly,
    caps: &ControlCapability,
    group_id: &str,
    axis: RotAxis,
    level: f64,
) {
    let Some(group) = caps.rcs_groups.iter().find(|g| g.id == group_id) else {
        return;
    };
    let vi = group.vessel_index;
    let Some(v) = asm.vessels.get_mut(vi) else {
        return;
    };
    set_attitude_rot(v, axis, level);
}

#[cfg(test)]
mod tests;
