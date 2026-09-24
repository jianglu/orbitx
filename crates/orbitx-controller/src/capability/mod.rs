//! 能力契约 `ControlCapability` + 可控体标识 `BodyRef`。
//!
//! controller 能控制什么，由航天器抽象决定，不靠 introspect vessel 字段。契约类型在本
//! crate（消费者拥有契约），数据由 `for_primary` / `for_detached` 从 `Assembly` 投影
//! （vessel 是物理真相源），按需构建。controller 按稳定 group id 命令执行器。
//!
//! 生命周期：按需（事件驱动）重建——会话开始建主 caps，分离/对接事件点重建。稳态 tick
//! 不重建、不指纹轮询。caps 由 Runtime（手动模式）或 WorkFlow（WorkFlow 模式）拥有，
//! 控制器不 own caps。

use orbitx_math::Vec3;
use orbitx_vessel::rcs::ThrusterGroupType;
use orbitx_vessel::{Assembly, ThrustReadout};

/// 可控体标识：一个控制器绑定一个 `BodyRef`。
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
pub enum BodyRef {
    /// 主组合体（connected components）。
    #[default]
    Primary,
    /// 某分离出的 vessel 下标。
    Detached(usize),
}

/// 能力契约：描述一个可控体的执行器（油门组 / TVC 组 / 对接口 / RCS 组 / 分离点）。
///
/// owned 值（Vec 拥有），不 borrow `Assembly`，故 `BaseController` 读 caps 与方法取
/// `&mut Assembly` 无借用冲突。
#[derive(Clone, Debug, Default)]
pub struct ControlCapability {
    pub body: BodyRef,
    pub throttle_groups: Vec<ThrottleGroup>,
    pub tvc_groups: Vec<TvcGroup>,
    pub dock_ports: Vec<DockPortRef>,
    pub rcs_groups: Vec<RcsGroupRef>,
    pub separation_points: Vec<SeparationPoint>,
}

/// 油门组：一台 vessel 的主推进器集合（共享节流斜坡）。
#[derive(Clone, Debug)]
pub struct ThrottleGroup {
    /// 稳定 id（P4.1 由 vessel 名派生）。
    pub id: String,
    pub vessel_index: usize,
    /// 组内推进器在 `Vessel.thrusters` 中的下标（主推范围 `..n_main_thrusters`）。
    pub thruster_indices: Vec<usize>,
    /// 节流斜坡最大速率 [1/s]（取组内首台主推的 `throttle_rate`）。
    pub slew_rate: f64,
}

/// TVC 组：一台 vessel 的可万向偏转推进器集合（共享偏转参数）。
#[derive(Clone, Debug)]
pub struct TvcGroup {
    pub id: String,
    /// `(vessel_index, thruster_index)` 列表。
    pub thrusters: Vec<(usize, usize)>,
    /// 最大偏转角 [rad]。
    pub max_angle: f64,
    /// 最大偏转角速率 [rad/s]。
    pub max_rate: f64,
    /// 俯仰偏转轴（体坐标系）。
    pub gimbal_axis: Vec3,
}

/// 对接口引用。
#[derive(Clone, Debug)]
pub struct DockPortRef {
    pub id: String,
    pub vessel_index: usize,
    pub port_index: usize,
}

/// RCS 推进器组引用。
#[derive(Clone, Debug)]
pub struct RcsGroupRef {
    pub id: String,
    pub vessel_index: usize,
    pub group_type: ThrusterGroupType,
}

/// 分离点。
#[derive(Clone, Debug)]
pub struct SeparationPoint {
    pub id: String,
    pub kind: SeparationKind,
}

/// 分离点类型。
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SeparationKind {
    /// 侧挂叶：`(vessel, port)` 经 `undock` 拆出。
    StrapOnLeaf { vessel: usize, port: usize },
    /// 同轴底级：经 `separate_stage` 拆出。
    CoaxialStage,
}

impl ControlCapability {
    /// 主组合体执行器（connected components 内）。
    pub fn for_primary(asm: &Assembly) -> Self {
        let mut caps = ControlCapability {
            body: BodyRef::Primary,
            ..Default::default()
        };

        for c in &asm.components {
            let vi = c.vessel_index;
            let v = &asm.vessels[vi];
            let n_main = v.n_main_thrusters.min(v.thrusters.len());

            // 油门组：主推范围且有推力。
            let main_idxs: Vec<usize> = (0..n_main)
                .filter(|&i| v.thrusters[i].max_thrust > 0.0)
                .collect();
            if !main_idxs.is_empty() {
                let slew = v.thrusters[main_idxs[0]].throttle_rate;
                caps.throttle_groups.push(ThrottleGroup {
                    id: v.name.clone(),
                    vessel_index: vi,
                    thruster_indices: main_idxs,
                    slew_rate: slew,
                });
            }

            // TVC 组：主推范围内可万向偏转的推进器。
            let tvc: Vec<(usize, usize)> = (0..n_main)
                .filter(|&i| v.thrusters[i].max_gimbal > 0.0)
                .map(|i| (vi, i))
                .collect();
            if !tvc.is_empty() {
                let t0 = &v.thrusters[tvc[0].1];
                caps.tvc_groups.push(TvcGroup {
                    id: format!("{}-tvc", v.name),
                    thrusters: tvc,
                    max_angle: t0.max_gimbal,
                    max_rate: t0.max_gimbal_rate,
                    gimbal_axis: t0.gimbal_axis,
                });
            }

            // 对接口。
            for (pi, _d) in v.docks.iter().enumerate() {
                caps.dock_ports.push(DockPortRef {
                    id: format!("{}-dock-{pi}", v.name),
                    vessel_index: vi,
                    port_index: pi,
                });
            }

            // RCS 组。
            for g in &v.thruster_groups {
                caps.rcs_groups.push(RcsGroupRef {
                    id: format!("{}-{}", v.name, group_type_name(g.group_type)),
                    vessel_index: vi,
                    group_type: g.group_type,
                });
            }
        }

        // 分离点：侧挂叶（经 ThrustReadout 投影）+ 同轴底级。
        for (vi, port) in asm.strap_on_leaf_indices() {
            let name = asm.vessels[vi].name.clone();
            caps.separation_points.push(SeparationPoint {
                id: format!("{name}-sep-{port}"),
                kind: SeparationKind::StrapOnLeaf { vessel: vi, port },
            });
        }
        // 同轴底级：多于一级未分离时存在。
        if asm.stage_count() > 1 {
            caps.separation_points.push(SeparationPoint {
                id: "stage-sep".to_string(),
                kind: SeparationKind::CoaxialStage,
            });
        }

        caps
    }

    /// 某分离船执行器（detached vessel）。仅投影该 vessel 的 thruster/rcs/dock；
    /// 分离点为空（P4.1：单 detached vessel 不再细分）。
    pub fn for_detached(asm: &Assembly, vessel_idx: usize) -> Self {
        let mut caps = ControlCapability {
            body: BodyRef::Detached(vessel_idx),
            ..Default::default()
        };
        let Some(v) = asm.vessels.get(vessel_idx) else {
            return caps;
        };
        let n_main = v.n_main_thrusters.min(v.thrusters.len());

        let main_idxs: Vec<usize> = (0..n_main)
            .filter(|&i| v.thrusters[i].max_thrust > 0.0)
            .collect();
        if !main_idxs.is_empty() {
            let slew = v.thrusters[main_idxs[0]].throttle_rate;
            caps.throttle_groups.push(ThrottleGroup {
                id: v.name.clone(),
                vessel_index: vessel_idx,
                thruster_indices: main_idxs,
                slew_rate: slew,
            });
        }

        let tvc: Vec<(usize, usize)> = (0..n_main)
            .filter(|&i| v.thrusters[i].max_gimbal > 0.0)
            .map(|i| (vessel_idx, i))
            .collect();
        if !tvc.is_empty() {
            let t0 = &v.thrusters[tvc[0].1];
            caps.tvc_groups.push(TvcGroup {
                id: format!("{}-tvc", v.name),
                thrusters: tvc,
                max_angle: t0.max_gimbal,
                max_rate: t0.max_gimbal_rate,
                gimbal_axis: t0.gimbal_axis,
            });
        }

        for (pi, _d) in v.docks.iter().enumerate() {
            caps.dock_ports.push(DockPortRef {
                id: format!("{}-dock-{pi}", v.name),
                vessel_index: vessel_idx,
                port_index: pi,
            });
        }
        for g in &v.thruster_groups {
            caps.rcs_groups.push(RcsGroupRef {
                id: format!("{}-{}", v.name, group_type_name(g.group_type)),
                vessel_index: vessel_idx,
                group_type: g.group_type,
            });
        }

        caps
    }
}

fn group_type_name(g: ThrusterGroupType) -> &'static str {
    use ThrusterGroupType::*;
    match g {
        Main => "main",
        Retro => "retro",
        Hover => "hover",
        AttPitchUp => "pitch_up",
        AttPitchDown => "pitch_down",
        AttYawLeft => "yaw_left",
        AttYawRight => "yaw_right",
        AttBankLeft => "bank_left",
        AttBankRight => "bank_right",
        AttRight => "right",
        AttLeft => "left",
        AttUp => "up",
        AttDown => "down",
        AttForward => "forward",
        AttBack => "back",
    }
}

#[cfg(test)]
mod tests;
