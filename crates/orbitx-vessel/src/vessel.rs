//! Vessel：单个航天器实体，对应 Orbiter 的 Vessel。

use crate::aero::{Airfoil, ControlSurface, DragElement};
use crate::dock::DockPort;
use crate::fuel::PropellantTank;
use crate::rcs::ThrusterGroup;
use crate::thruster::Thruster;
use crate::touchdown::TouchdownVertex;
use orbitx_math::{cross, StateVectors, Vec3};

/// 单个航天器实体。
pub struct Vessel {
    /// 唯一标识。
    pub id: u64,
    /// 名称。
    pub name: String,
    /// 运动状态（位置、速度、姿态、角速度）。
    pub state: StateVectors,
    /// 干质量（不含燃料）[kg]。
    pub dry_mass: f64,
    /// 当前燃料质量 [kg]。
    pub fuel_mass: f64,
    /// 级长度 [m]。
    pub length: f64,
    /// 级半径 [m]。
    pub radius: f64,
    /// 分离脉冲 [m/s]。
    pub separation_impulse: f64,
    /// 主惯量张量（体坐标系对角线）[kg·m²]。
    pub pmi: Vec3,
    /// 潮汐（重力梯度）阻尼系数，对应 Orbiter `tidaldamp`。
    pub tidaldamp: f64,
    /// 推进器列表。
    pub thrusters: Vec<Thruster>,
    /// 推进器组列表（RCS 等）。
    pub thruster_groups: Vec<ThrusterGroup>,
    /// 对接端口列表。
    pub docks: Vec<DockPort>,
    /// 是否已分离。
    pub detached: bool,
    /// 累积的体坐标系线性力 [N]（Orbiter `Flin_add`，`Vessel.h:1673`）。
    pub flin_add: Vec3,
    /// 累积的体坐标系力矩 [N·m]（Orbiter `Amom_add`，`Vessel.h:1674`）。
    pub amom_add: Vec3,
    // ── 气动力子系统 ──
    /// 空气翼面列表。
    pub airfoils: Vec<Airfoil>,
    /// 控制面列表。
    pub ctrlsurfs: Vec<ControlSurface>,
    /// 变阻力元件列表。
    pub dragels: Vec<DragElement>,
    /// 截面积 (横向X, 轴向Y, 横向Z) [m²]。
    pub cross_section: Vec3,
    /// 气动阻尼系数 (x, y, z)（Orbiter `rdrag`）。
    pub rdrag: Vec3,
    // ── 燃料子系统 ──
    /// 推进剂储箱列表（Orbiter `TankSpec` 数组）。
    pub tanks: Vec<PropellantTank>,
    /// 着陆触点列表（Orbiter `TOUCHDOWN_VTX` 数组）。
    pub touchdown_points: Vec<TouchdownVertex>,
}

impl Vessel {
    /// 从级定义创建。
    pub fn from_spec(id: u64, spec: &crate::stage::StageSpec, state: StateVectors) -> Self {
        Self {
            id,
            name: spec.name.to_string(),
            state,
            dry_mass: spec.dry_mass,
            fuel_mass: spec.fuel_mass,
            length: spec.length,
            radius: spec.radius,
            separation_impulse: spec.separation_impulse,
            pmi: spec.effective_pmi(),
            tidaldamp: 0.0,
            thrusters: spec.make_thrusters(),
            thruster_groups: Vec::new(),
            docks: spec.make_docks(),
            detached: false,
            flin_add: Vec3::ZERO,
            amom_add: Vec3::ZERO,
            airfoils: Vec::new(),
            ctrlsurfs: Vec::new(),
            dragels: Vec::new(),
            cross_section: Vec3::ZERO,
            rdrag: Vec3::ZERO,
            tanks: Vec::new(),
            touchdown_points: Vec::new(),
        }
    }

    /// 总质量（干质量+燃料）。
    pub fn mass(&self) -> f64 {
        self.dry_mass + self.fuel_mass
    }

    /// 当前总推力 [N]。
    pub fn current_thrust(&self) -> f64 {
        self.thrusters.iter().map(|t| t.current_thrust()).sum()
    }

    /// 燃料消耗率 [kg/s]。
    pub fn mass_flow_rate(&self) -> f64 {
        self.thrusters.iter().map(|t| t.mass_flow_rate()).sum()
    }

    /// 设置所有推进器油门。
    pub fn set_throttle(&mut self, level: f64) {
        let level = level.clamp(0.0, 1.0);
        for t in &mut self.thrusters {
            t.level = level;
        }
    }

    /// 消耗燃料 [kg]，返回实际消耗量。
    pub fn consume_fuel(&mut self, mass: f64) -> f64 {
        let consumed = mass.min(self.fuel_mass);
        self.fuel_mass -= consumed;
        if self.fuel_mass < 0.0 {
            self.fuel_mass = 0.0;
        }
        consumed
    }

    /// 累加一个作用于体坐标点 `r` 的力 `F`（`Vessel.h:1316-1320` AddForce）。
    ///
    /// 同时累积线性力和力矩：`Flin_add += F`，`Amom_add += F × r`。
    #[inline]
    pub fn add_force(&mut self, f: Vec3, r: Vec3) {
        self.flin_add += f;
        self.amom_add += cross(f, r);
    }

    /// 累加一个纯力矩（无力）`M`（体坐标系）。
    #[inline]
    pub fn add_torque(&mut self, m: Vec3) {
        self.amom_add += m;
    }

    /// 清空累积的力和力矩（每步开始调用）。
    #[inline]
    pub fn clear_forces(&mut self) {
        self.flin_add = Vec3::ZERO;
        self.amom_add = Vec3::ZERO;
    }

    /// 从指定储箱消耗燃料 [kg]，返回实际消耗量。
    ///
    /// 若 `tank_id` 对应的储箱不存在或已空，返回 0。
    pub fn consume_fuel_from_tank(&mut self, tank_id: u32, mass: f64) -> f64 {
        if let Some(tank) = self.tanks.iter_mut().find(|t| t.id == tank_id) {
            tank.consume(mass)
        } else {
            0.0
        }
    }

    /// 查询指定储箱的燃料质量 [kg]。
    pub fn tank_mass(&self, tank_id: u32) -> f64 {
        self.tanks
            .iter()
            .find(|t| t.id == tank_id)
            .map(|t| t.mass)
            .unwrap_or(0.0)
    }

    /// 所有储箱的总燃料质量 [kg]。
    pub fn tanks_total_mass(&self) -> f64 {
        self.tanks.iter().map(|t| t.mass).sum()
    }

    /// 快照所有储箱质量（每步开始调用，用于流率计算）。
    pub fn snapshot_tanks(&mut self) {
        for tank in &mut self.tanks {
            tank.snapshot();
        }
    }
}
