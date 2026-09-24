//! 组合体级遥测只读 trait（P4.1）。
//!
//! 物理仿真权威是 `Assembly`/`Vessel`，数据由步进写入。本模块不存副本、不缓存、不维护
//! 状态——只定义一组只读访问 trait，外部经此读权威数据。零副本：trait 方法全部委托
//! `Assembly` 内部状态即时计算。
//!
//! 两个消费者（都直读 `Assembly`，不经彼此）：
//! - `BaseController`（控制环读，经 base 方法暴露给控制器）
//! - Runtime（显示切片读）
//!
//! 热路径零 alloc：index-set 方法返回 `impl Iterator + '_`（借 `Assembly` 现有数据惰性
//! 迭代，无 mutation、无 copy、无 alloc）；姿态/运动学/质量方法返回 `f64` / `(f64,f64)`
//! / `Vec3`（Copy）。

use orbitx_math::{dot, Vec3};

use crate::assembly::Assembly;
use crate::attitude as att;

/// 姿态读数（TVC PD、TargetController 用）。读活动级 `state`。
pub trait AttitudeReadout {
    /// 有符号 tip 分量（体轴，≈ sin θ）：相对径向。
    fn attitude_errors(&self) -> (f64, f64);
    /// 有符号俯仰/偏航角 [rad]。
    fn pitch_yaw_angles(&self) -> (f64, f64);
    /// 体 +Y 与径向无符号夹角 [rad]（总 tip）。
    fn tip_angle(&self) -> f64;
    /// 绕体 +Y（纵轴）的滚转角 [rad]。
    fn roll_angle(&self) -> f64;
    /// 角速度 [rad/s]（体坐标系）。
    fn omega(&self) -> Vec3;
}

/// 推力/点火集读数（BaseController、transition 用）。
///
/// index-set 方法返回借 `Assembly` 的惰性迭代器：无 mutation、无 copy、无 alloc。
pub trait ThrustReadout {
    /// 主组合体中有主推（`max_thrust > 0`）的 vessel 下标。
    fn primary_thrusting_indices(&self) -> impl Iterator<Item = usize> + '_;
    /// 侧挂叶 `(vessel_index, port_on_leaf)`，按 vessel 下标有序。
    fn strap_on_leaf_indices(&self) -> impl Iterator<Item = (usize, usize)> + '_;
    /// 本帧应响应油门的有推船：`active` ∪ 侧挂叶（均须有主推、在主组合体）。
    fn lit_thrusting_indices(&self) -> impl Iterator<Item = usize> + '_;
    /// 优先空燃料有推叶的侧挂叶候选（标量，无 alloc）。
    fn pick_strap_on_leaf(&self) -> Option<(usize, usize)>;
    /// lit 集有推船推力之和 [N]（已门控燃料）。
    fn primary_thrust_sum(&self) -> f64;
    /// 当前高度处大气压 [Pa]（无大气则为 0）。
    fn ambient_pressure(&self) -> f64;
}

/// 质量/燃料读数（WorkFlow transition fuel_empty 用）。
pub trait MassReadout {
    fn total_mass(&self) -> f64;
    fn fuel_mass(&self) -> f64;
    fn fuel_percent(&self) -> f64;
}

/// 运动学读数（prograde/retrograde hold、gravity turn 用）。读主组合体 `state`。
pub trait KinematicsReadout {
    fn velocity(&self) -> Vec3;
    fn position(&self) -> Vec3;
    fn speed(&self) -> f64;
}

/// 级结构读数（stage_count、active 用）。
pub trait StageReadout {
    fn active_vessel(&self) -> usize;
    fn stage_count(&self) -> usize;
    fn active_name(&self) -> &str;
}

/// 可控体读数（runtime 检测分离/派生控制器用）。
pub trait BodyReadout {
    /// 主组合体是否存在（components 非空）。
    fn primary_present(&self) -> bool;
    /// 已分离独立体下标。
    fn detached_vessels(&self) -> impl Iterator<Item = usize> + '_;
}

// ── 内部查询助手（过渡副本，P4.3 cli 切 Zenoh 后 cli/control.rs 退役时统一此处）──

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

// ── impl Assembly ──────────────────────────────────────────────────────

impl AttitudeReadout for Assembly {
    fn attitude_errors(&self) -> (f64, f64) {
        let v = &self.vessels[self.active];
        att::attitude_errors(&v.state)
    }
    fn pitch_yaw_angles(&self) -> (f64, f64) {
        let v = &self.vessels[self.active];
        att::pitch_yaw_angles(&v.state)
    }
    fn tip_angle(&self) -> f64 {
        let v = &self.vessels[self.active];
        att::tip_angle(&v.state)
    }
    fn roll_angle(&self) -> f64 {
        let v = &self.vessels[self.active];
        att::roll_angle(&v.state)
    }
    fn omega(&self) -> Vec3 {
        self.vessels[self.active].state.omega
    }
}

impl ThrustReadout for Assembly {
    fn primary_thrusting_indices(&self) -> impl Iterator<Item = usize> + '_ {
        self.components
            .iter()
            .map(|c| c.vessel_index)
            .filter(move |&i| !self.vessels[i].detached && has_main_thrust(self, i))
    }

    fn strap_on_leaf_indices(&self) -> impl Iterator<Item = (usize, usize)> + '_ {
        self.vessels
            .iter()
            .enumerate()
            .filter_map(move |(i, v)| {
                if v.detached || i == self.active || !in_primary(self, i) {
                    return None;
                }
                if dock_degree(self, i) != 1 {
                    return None;
                }
                let (port, mate_id, mate_port) = v.docks.iter().enumerate().find_map(|(p, d)| {
                    d.connected_to.map(|(id, mp)| (p, id, mp))
                })?;
                let mate_idx = self.vessels.iter().position(|x| x.id == mate_id)?;
                if self.vessels[mate_idx].detached || dock_degree(self, mate_idx) < 2 {
                    return None;
                }
                if !is_lateral_on_mate(self, i, mate_idx, mate_port) {
                    return None;
                }
                Some((i, port))
            })
    }

    fn lit_thrusting_indices(&self) -> impl Iterator<Item = usize> + '_ {
        let active = self.active;
        self.primary_thrusting_indices().filter(move |&i| {
            i == active || self.strap_on_leaf_indices().any(|(s, _)| s == i)
        })
    }

    fn pick_strap_on_leaf(&self) -> Option<(usize, usize)> {
        // 标量最小值追踪，无 alloc。优先：空燃料有推(0) → 有推(1) → 无推(2)，再按 vessel 下标。
        self.strap_on_leaf_indices()
            .map(|(i, port)| {
                let v = &self.vessels[i];
                let has_thrust = has_main_thrust(self, i);
                let empty = v.fuel_mass < 1.0;
                let prio = if empty && has_thrust {
                    0
                } else if has_thrust {
                    1
                } else {
                    2
                };
                (prio, i, port)
            })
            .min_by_key(|&(prio, i, _)| (prio, i))
            .map(|(_, i, port)| (i, port))
    }

    fn primary_thrust_sum(&self) -> f64 {
        let p = self.ambient_pressure();
        self.lit_thrusting_indices()
            .map(|i| self.vessels[i].current_thrust(p))
            .sum()
    }

    fn ambient_pressure(&self) -> f64 {
        Assembly::ambient_pressure(self)
    }
}

impl MassReadout for Assembly {
    fn total_mass(&self) -> f64 {
        Assembly::total_mass(self)
    }
    fn fuel_mass(&self) -> f64 {
        Assembly::total_fuel(self)
    }
    fn fuel_percent(&self) -> f64 {
        Assembly::fuel_percent(self)
    }
}

impl KinematicsReadout for Assembly {
    fn velocity(&self) -> Vec3 {
        self.state.vel
    }
    fn position(&self) -> Vec3 {
        self.state.pos
    }
    fn speed(&self) -> f64 {
        self.state.vel.length()
    }
}

impl StageReadout for Assembly {
    fn active_vessel(&self) -> usize {
        self.active
    }
    fn stage_count(&self) -> usize {
        Assembly::stage_count(self)
    }
    fn active_name(&self) -> &str {
        Assembly::active_name(self)
    }
}

impl BodyReadout for Assembly {
    fn primary_present(&self) -> bool {
        !self.components.is_empty()
    }
    fn detached_vessels(&self) -> impl Iterator<Item = usize> + '_ {
        self.vessels
            .iter()
            .enumerate()
            .filter_map(|(i, v)| v.detached.then_some(i))
    }
}

#[cfg(test)]
mod tests;
