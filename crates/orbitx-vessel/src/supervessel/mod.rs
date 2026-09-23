//! SuperVessel 几何与刚体合成辅助（薄 re-export 层）。
//!
//! 物理算法已下沉到 `orbitx_dynamics`（`rigidbody` 模块）；本模块仅保留：
//! - `SubVesselData` / `DockGeometry` 等数据类型的 re-export
//! - 接受 `DockPort` 的 `rel_docking_pos` 薄包装（把对接端口几何转成
//!   `DockGeometry` 后调 dynamics）
//!
//! 组合体坐标系约定（对齐 `SuperVessel.h` 注释）：
//! - 原点与姿态取 root 成员体坐标（root 的 `rpos = 0`、`rrot = I`）
//! - 子船点：`ps = rrot_i * pv + rpos_i`
//! - 世界点：`pg = R_sv * (ps - cg) + gpos`（`gpos` 为组合体 CG）

use crate::dock::DockPort;
use orbitx_dynamics::DockGeometry;
use orbitx_math::{Matrix3, Vec3};

pub use orbitx_dynamics::{
    add_component_force_and_moment, center_of_mass, composite_pmi, component_state_vectors,
    supervessel_state_from_root, SubVesselData,
};

/// 计算 `target` 相对 `mine` 的位姿，使双方指定端口对齐对接。
///
/// 移植 `Vessel::RelDockingPos`（`Vessel.cpp:2884-2928`）：
/// - 本口 `dir` ↔ 对方 `-dir`
/// - 本口 `rot` ↔ 对方 `rot`
/// - 返回的 `rpos` / `rrot` 把 **target 体坐标** 映到 **mine 体坐标**
///
/// 物理实现见 `orbitx_dynamics::rigidbody::rel_docking_pos`。
pub fn rel_docking_pos(mine: &DockPort, target: &DockPort) -> (Vec3, Matrix3) {
    let g_mine = DockGeometry {
        pos: mine.pos,
        dir: mine.dir,
        rot: mine.rot,
    };
    let g_target = DockGeometry {
        pos: target.pos,
        dir: target.dir,
        rot: target.rot,
    };
    orbitx_dynamics::rel_docking_pos(&g_mine, &g_target)
}
