//! 分离判据与执行。
//!
//! `should_auto_separate`：侧挂叶空燃料有推则触发。`perform_separate`：按 separation point id
//! 命令，侧挂叶优先 `Assembly::undock`，否则同轴 `separate_stage`；透传 `undock` 返回的
//! 分离出 vessel 下标（Runtime/WorkFlow 据此重建 caps + 派生子控制器）。

use crate::capability::{ControlCapability, SeparationKind};
use orbitx_vessel::{Assembly, ThrustReadout};

/// 是否应自动分离（上层判据）：侧挂叶空燃料有推，或 active 空燃料有推且多级。
pub fn should_auto_separate(asm: &Assembly) -> bool {
    if asm.stage_count() <= 1 {
        return false;
    }
    // 侧挂叶空燃料有推。
    for (vi, _port) in asm.strap_on_leaf_indices() {
        let v = &asm.vessels[vi];
        let has_thrust = v.thrusters.iter().any(|t| t.max_thrust > 0.0);
        if v.fuel_mass < 1.0 && has_thrust {
            return true;
        }
    }
    // active 空燃料有推。
    let active = &asm.vessels[asm.active];
    let has_thrust = active.thrusters.iter().any(|t| t.max_thrust > 0.0);
    active.fuel_mass < 1.0 && has_thrust
}

/// 按 separation point id 执行分离，返回分离出 vessel 下标。
///
/// `point_id` 命中 `caps.separation_points`：`StrapOnLeaf` → `undock`；`CoaxialStage` →
/// `separate_stage`（返回 `[active]`）。未命中返回空。
pub fn perform_separate(
    asm: &mut Assembly,
    caps: &ControlCapability,
    point_id: &str,
) -> Vec<usize> {
    let Some(point) = caps.separation_points.iter().find(|s| s.id == point_id) else {
        return Vec::new();
    };
    match point.kind {
        SeparationKind::StrapOnLeaf { vessel, port } => {
            let id = asm.vessels[vessel].id;
            let sep = asm.vessels[vessel].separation_impulse;
            asm.undock(id, port, sep)
        }
        SeparationKind::CoaxialStage => {
            let active = asm.separate_stage();
            vec![active]
        }
    }
}

#[cfg(test)]
mod tests;
