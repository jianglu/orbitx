//! Vessel：单个航天器实体，对应 Orbiter 的 Vessel。

use crate::aero::{Airfoil, ControlSurface, DragElement};
use crate::dock::DockPort;
use crate::fuel::PropellantTank;
use crate::rcs::ThrusterGroup;
use crate::stage::{StageSpec, ThrusterSpec};
use crate::thruster::Thruster;
use crate::touchdown::TouchdownVertex;
use orbitx_math::{cross, StateVectors, Vec3};

/// 单个航天器实体。
pub struct Vessel {
    pub id: u64,
    pub name: String,
    pub state: StateVectors,
    pub dry_mass: f64,
    pub fuel_mass: f64,
    pub length: f64,
    pub radius: f64,
    pub separation_impulse: f64,
    pub pmi: Vec3,
    /// 潮汐（重力梯度）阻尼系数，对应 Orbiter `tidaldamp`。
    pub tidaldamp: f64,
    pub thrusters: Vec<Thruster>,
    /// 主推台数（`from_spec` 创建时的 thrusters 长度；RCS 追加在其后）。
    pub n_main_thrusters: usize,
    pub thruster_groups: Vec<ThrusterGroup>,
    pub docks: Vec<DockPort>,
    pub detached: bool,
    pub flin_add: Vec3,
    pub amom_add: Vec3,
    pub airfoils: Vec<Airfoil>,
    pub ctrlsurfs: Vec<ControlSurface>,
    pub dragels: Vec<DragElement>,
    pub cross_section: Vec3,
    pub rdrag: Vec3,
    pub tanks: Vec<PropellantTank>,
    pub touchdown_points: Vec<TouchdownVertex>,
}

impl Vessel {
    /// 从级定义创建（含轴向 Cd(M) 阻力元件）。
    pub fn from_spec(id: u64, spec: &StageSpec, state: StateVectors) -> Self {
        let thrusters = spec.make_thrusters();
        let n_main = thrusters.len();
        let area = std::f64::consts::PI * spec.radius * spec.radius;
        let cd_table = spec.cd_mach_table();
        let dragels = if area > 0.0 {
            vec![DragElement::constant(Vec3::ZERO, cd_table[0].1, area)
                .with_cd_mach(cd_table)]
        } else {
            Vec::new()
        };
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
            tidaldamp: spec.tidaldamp,
            thrusters,
            n_main_thrusters: n_main,
            thruster_groups: Vec::new(),
            docks: spec.make_docks(),
            detached: false,
            flin_add: Vec3::ZERO,
            amom_add: Vec3::ZERO,
            airfoils: Vec::new(),
            ctrlsurfs: Vec::new(),
            dragels,
            cross_section: Vec3::new(area, area * 2.0, area),
            rdrag: Vec3::new(1.0, 0.1, 1.0),
            tanks: Vec::new(),
            touchdown_points: Vec::new(),
        }
    }

    pub fn mass(&self) -> f64 {
        self.dry_mass + self.fuel_mass
    }

    /// 当前总推力 [N]（含气压缩放）。
    pub fn current_thrust(&self, pressure_pa: f64) -> f64 {
        self.thrusters
            .iter()
            .map(|t| t.current_thrust(pressure_pa))
            .sum()
    }

    /// 燃料消耗率 [kg/s]。
    pub fn mass_flow_rate(&self, pressure_pa: f64) -> f64 {
        self.thrusters
            .iter()
            .map(|t| t.mass_flow_rate(pressure_pa))
            .sum()
    }

    /// 设置主推油门（不覆盖 RCS 组内推进器）。
    pub fn set_throttle(&mut self, level: f64) {
        let level = level.clamp(0.0, 1.0);
        let n = self.n_main_thrusters.min(self.thrusters.len());
        for t in &mut self.thrusters[..n] {
            t.level = level;
        }
    }

    pub fn consume_fuel(&mut self, mass: f64) -> f64 {
        let consumed = mass.min(self.fuel_mass);
        self.fuel_mass -= consumed;
        if self.fuel_mass < 0.0 {
            self.fuel_mass = 0.0;
        }
        consumed
    }

    #[inline]
    pub fn add_force(&mut self, f: Vec3, r: Vec3) {
        self.flin_add += f;
        self.amom_add += cross(f, r);
    }

    #[inline]
    pub fn add_torque(&mut self, m: Vec3) {
        self.amom_add += m;
    }

    #[inline]
    pub fn clear_forces(&mut self) {
        self.flin_add = Vec3::ZERO;
        self.amom_add = Vec3::ZERO;
    }

    pub fn consume_fuel_from_tank(&mut self, tank_id: u32, mass: f64) -> f64 {
        if let Some(tank) = self.tanks.iter_mut().find(|t| t.id == tank_id) {
            tank.consume(mass)
        } else {
            0.0
        }
    }

    pub fn tank_mass(&self, tank_id: u32) -> f64 {
        self.tanks
            .iter()
            .find(|t| t.id == tank_id)
            .map(|t| t.mass)
            .unwrap_or(0.0)
    }

    pub fn tanks_total_mass(&self) -> f64 {
        self.tanks.iter().map(|t| t.mass).sum()
    }

    pub fn snapshot_tanks(&mut self) {
        for tank in &mut self.tanks {
            tank.snapshot();
        }
    }
}

/// 由 config `StageConfig` 构造运行时 `StageSpec`（`'static` 名用泄漏字符串）。
pub fn stage_spec_from_config(cfg: &orbitx_config::StageConfig) -> StageSpec {
    let name: &'static str = Box::leak(cfg.name.clone().into_boxed_str());
    let thrusters = cfg
        .thrusters
        .iter()
        .map(|t| ThrusterSpec {
            pos: Vec3::new(t.pos[0], t.pos[1], t.pos[2]),
            dir: Vec3::new(t.dir[0], t.dir[1], t.dir[2]),
            thrust: t.thrust,
            isp: t.isp,
            thrust_sl: t.thrust_sl,
            isp_sl: t.isp_sl,
            max_gimbal: t.max_gimbal,
            max_gimbal_rate: t.max_gimbal_rate,
            gimbal_axis: Vec3::new(t.gimbal_axis[0], t.gimbal_axis[1], t.gimbal_axis[2]),
        })
        .collect();
    let docks = cfg.docks.as_ref().map(|ds| {
        ds.iter()
            .map(|d| DockPort::with_rot(
                Vec3::new(d.pos[0], d.pos[1], d.pos[2]),
                Vec3::new(d.dir[0], d.dir[1], d.dir[2]),
                Vec3::new(d.rot[0], d.rot[1], d.rot[2]),
            ))
            .collect()
    });
    let pmi = cfg
        .inertia
        .map(|i| Vec3::new(i[0], i[1], i[2]))
        .unwrap_or(crate::stage::PMI_UNDEF);
    StageSpec {
        name,
        dry_mass: cfg.dry_mass,
        fuel_mass: cfg.fuel_mass,
        thrusters,
        length: cfg.length,
        radius: cfg.radius,
        separation_impulse: cfg.separation_impulse,
        pmi,
        tidaldamp: cfg.tidaldamp,
        cd_mach: cfg.cd_mach.iter().map(|p| (p[0], p[1])).collect(),
        docks,
    }
}
