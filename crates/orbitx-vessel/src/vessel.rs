//! Vessel：单个航天器实体，对应 Orbiter 的 Vessel。

use crate::aero::{Airfoil, ControlSurface, DragElement};
use crate::diagnostics::FlightDiagnostics;
use crate::dock::DockPort;
use crate::fuel::{parallel_axis_diag, PropellantTank};
use crate::rcs::ThrusterGroup;
use crate::stage::{
    default_chord_dir, default_pmi, LiftingSurfaceSpec, StageSpec, TankSpec, ThrusterSpec,
};
use crate::thruster::Thruster;
use crate::touchdown::TouchdownVertex;
use orbitx_dynamics::propulsion as prop;
use orbitx_dynamics::{FinKind, LiftingSurface, RocketBodyAero};
use orbitx_math::{cross, StateVectors, Vec3};

/// 运行时升力面（级体坐标；`deploy` 可限速更新）。
#[derive(Clone, Debug)]
pub struct VesselLiftingSurface {
    pub surf: LiftingSurface,
    pub deploy_target: f64,
    pub deploy_rate: f64,
}

/// 单个航天器实体。
pub struct Vessel {
    pub id: u64,
    pub name: String,
    pub state: StateVectors,
    pub dry_mass: f64,
    /// 干质心 [m]（体坐标，原点 = 满载质心）。
    pub dry_center: Vec3,
    /// 干惯量对角线 [kg·m²]，绕 `dry_center`。
    pub dry_inertia: Vec3,
    pub length: f64,
    pub radius: f64,
    pub separation_impulse: f64,
    /// 归一化 PMI [m²]（绕当前质心，随烧耗刷新）。
    pub pmi: Vec3,
    /// 潮汐（重力梯度）阻尼系数，对应 Orbiter `tidaldamp`。
    pub tidaldamp: f64,
    pub thrusters: Vec<Thruster>,
    /// 主推台数（`from_spec` 创建时的 thrusters 长度；RCS 追加在其后）。
    pub n_main_thrusters: usize,
    pub thruster_groups: Vec<ThrusterGroup>,
    pub docks: Vec<DockPort>,
    pub detached: bool,
    /// 上层判定坠毁后置位；步进将跳过该船（独立体）或含该船的主组合体。
    pub crashed: bool,
    pub flin_add: Vec3,
    pub amom_add: Vec3,
    pub airfoils: Vec<Airfoil>,
    pub ctrlsurfs: Vec<ControlSurface>,
    pub dragels: Vec<DragElement>,
    /// 火箭筒体气动；`Some` 时步进优先 `compute_rocket_aero`（无 airfoil 时）。
    pub rocket_body: Option<RocketBodyAero>,
    /// 火箭升力面（级体坐标）。
    pub lifting_surfaces: Vec<VesselLiftingSurface>,
    pub cross_section: Vec3,
    pub rdrag: Vec3,
    pub tanks: Vec<PropellantTank>,
    pub touchdown_points: Vec<TouchdownVertex>,
    /// 本船最近一步飞行诊断（对齐 Orbiter `SurfParam` / Lift·Drag 缓存）。
    pub diagnostics: FlightDiagnostics,
}

impl Vessel {
    /// 从级定义创建。
    ///
    /// 火箭气动路径：写入 `rocket_body` / `lifting_surfaces`，**不**钉原点 `DragElement`。
    /// 否则：默认 Cd(M) 原点阻力元件（P1.1）。
    pub fn from_spec(id: u64, spec: &StageSpec, state: StateVectors) -> Self {
        let thrusters = spec.make_thrusters();
        let n_main = thrusters.len();
        let area = std::f64::consts::PI * spec.radius * spec.radius;
        let cd_table = spec.cd_mach_table();
        let (rocket_body, lifting_surfaces, dragels) = if spec.uses_rocket_aero() {
            let body = RocketBodyAero {
                cd_mach: cd_table.clone(),
                cd0: cd_table.first().map(|(_, c)| *c).unwrap_or(0.3),
                cn_alpha: spec.cn_alpha.unwrap_or(2.0),
                // 与下方 `rdrag = (1, 0.1, 1)` 相同；火箭步进只读这里。
                ..RocketBodyAero::default()
            };
            let surfaces = spec
                .lifting_surfaces
                .iter()
                .map(vessel_surface_from_spec)
                .collect();
            (Some(body), surfaces, Vec::new())
        } else {
            let dragels = if area > 0.0 {
                vec![DragElement::constant(Vec3::ZERO, cd_table[0].1, area).with_cd_mach(cd_table)]
            } else {
                Vec::new()
            };
            (None, Vec::new(), dragels)
        };
        let mut v = Self {
            id,
            name: spec.name.to_string(),
            state,
            dry_mass: spec.dry_mass,
            dry_center: spec.dry_center,
            dry_inertia: spec.dry_inertia,
            length: spec.length,
            radius: spec.radius,
            separation_impulse: spec.separation_impulse,
            pmi: Vec3::new(1.0, 1.0, 1.0),
            tidaldamp: spec.tidaldamp,
            thrusters,
            n_main_thrusters: n_main,
            thruster_groups: Vec::new(),
            docks: spec.make_docks(),
            detached: false,
            crashed: false,
            flin_add: Vec3::ZERO,
            amom_add: Vec3::ZERO,
            airfoils: Vec::new(),
            ctrlsurfs: Vec::new(),
            dragels,
            rocket_body,
            lifting_surfaces,
            cross_section: Vec3::new(area, area * 2.0, area),
            rdrag: Vec3::new(1.0, 0.1, 1.0),
            tanks: spec.make_tanks(),
            touchdown_points: Vec::new(),
            diagnostics: FlightDiagnostics::default(),
        };
        v.refresh_pmi();
        v
    }

    pub fn mass(&self) -> f64 {
        self.dry_mass + self.tanks_total_mass()
    }

    /// 当前推进剂总质量 [kg]。
    pub fn fuel_mass(&self) -> f64 {
        self.tanks_total_mass()
    }

    /// 满载推进剂质量 [kg]。
    pub fn fuel_max(&self) -> f64 {
        self.tanks.iter().map(|t| t.max_mass).sum()
    }

    /// 体坐标质心（原点 = 满载质心导出原点）。
    pub fn com_body(&self) -> Vec3 {
        let mut m_sum = self.dry_mass;
        let mut acc = self.dry_center * self.dry_mass;
        for t in &self.tanks {
            m_sum += t.mass;
            acc += t.pos * t.mass;
        }
        if m_sum > 1e-12 {
            acc * (1.0 / m_sum)
        } else {
            Vec3::ZERO
        }
    }

    /// 绕当前质心的绝对惯量对角线 [kg·m²]。
    pub fn inertia_about_com(&self) -> Vec3 {
        let com = self.com_body();
        let mut i = parallel_axis_diag(self.dry_inertia, self.dry_mass, self.dry_center, com);
        for t in &self.tanks {
            i += parallel_axis_diag(t.inertia_now(), t.mass, t.pos, com);
        }
        i
    }

    /// 按当前质量刷新归一化 PMI。
    pub fn refresh_pmi(&mut self) {
        let m = self.mass();
        if m > 1e-12 {
            let i = self.inertia_about_com();
            self.pmi = Vec3::new(i.x / m, i.y / m, i.z / m);
        } else {
            self.pmi = default_pmi(self.radius, self.length, 1.0);
        }
    }

    /// 罐池质量加权效率（无燃料时 1.0）。
    pub fn eta_pool(&self) -> f64 {
        let mut m = 0.0;
        let mut w = 0.0;
        for t in &self.tanks {
            if t.mass > 0.0 {
                m += t.mass;
                w += t.mass * t.efficiency.max(1e-9);
            }
        }
        if m > 1e-12 {
            w / m
        } else {
            1.0
        }
    }

    /// 名义质量流 [kg/s]（含罐池 η；未含燃料不足折扣）。
    pub fn mass_flow_rate(&self, pressure_pa: f64) -> f64 {
        let eta = self.eta_pool();
        self.thrusters
            .iter()
            .filter(|t| t.level > 0.0)
            .map(|t| {
                let thr = t.current_thrust(pressure_pa);
                let isp_e = t.effective_isp(pressure_pa);
                prop::mass_flow_rate_eff(thr, isp_e, eta)
            })
            .sum()
    }

    /// 燃料不足时的统一推力折扣 s∈[0,1]（整步冻结用）。
    pub fn fuel_thrust_scale(&self, pressure_pa: f64, dt: f64) -> f64 {
        let fuel = self.tanks_total_mass();
        if fuel <= 1e-12 {
            return 0.0;
        }
        let mdot = self.mass_flow_rate(pressure_pa);
        if mdot <= 1e-12 || dt <= 0.0 {
            return 1.0;
        }
        let need = mdot * dt;
        if need <= fuel {
            1.0
        } else {
            (fuel / need).clamp(0.0, 1.0)
        }
    }

    /// 当前总推力 [N]（含气压缩放；无燃料时为 0）。
    pub fn current_thrust(&self, pressure_pa: f64) -> f64 {
        if self.tanks_total_mass() <= 1e-12 {
            return 0.0;
        }
        self.thrusters
            .iter()
            .map(|t| t.current_thrust(pressure_pa))
            .sum()
    }

    /// 设置主推油门指令（不覆盖 RCS 组内推进器）。
    pub fn set_throttle(&mut self, level: f64) {
        let level = level.clamp(0.0, 1.0);
        let n = self.n_main_thrusters.min(self.thrusters.len());
        for t in &mut self.thrusters[..n] {
            t.level_cmd = level;
            if t.throttle_rate <= 0.0 {
                t.level = level;
            }
        }
    }

    /// 排空全部储箱（测试 / 分离判定）。
    pub fn drain_fuel(&mut self) {
        for t in &mut self.tanks {
            t.mass = 0.0;
        }
        self.refresh_pmi();
    }

    /// 从级内罐池按当前质量比例消耗 [kg]，返回实际消耗量。
    pub fn consume_fuel_pool(&mut self, mass: f64) -> f64 {
        if mass <= 0.0 {
            return 0.0;
        }
        let total = self.tanks_total_mass();
        if total <= 1e-12 {
            return 0.0;
        }
        let want = mass.min(total);
        let mut left = want;
        let n = self.tanks.len();
        for i in 0..n {
            if left <= 1e-15 {
                break;
            }
            let share = if i + 1 == n {
                left
            } else {
                want * (self.tanks[i].mass / total)
            };
            let got = self.tanks[i].consume(share);
            left -= got;
        }
        if left > 1e-9 {
            for t in &mut self.tanks {
                let got = t.consume(left);
                left -= got;
                if left <= 1e-15 {
                    break;
                }
            }
        }
        want - left.max(0.0)
    }

    #[inline]
    pub fn add_force(&mut self, f: Vec3, r: Vec3) {
        self.flin_add += f;
        self.amom_add += cross(r, f);
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
            throttle_rate: t.throttle_rate,
        })
        .collect();
    let tanks = cfg
        .tanks
        .iter()
        .map(|t| TankSpec {
            id: t.id,
            max_mass: t.max_mass,
            mass: t.resolved_mass(),
            pos: Vec3::new(t.pos[0], t.pos[1], t.pos[2]),
            inertia: Vec3::new(t.inertia[0], t.inertia[1], t.inertia[2]),
            efficiency: t.efficiency,
        })
        .collect();
    let docks = cfg.docks.as_ref().map(|ds| {
        ds.iter()
            .map(|d| {
                DockPort::with_rot(
                    Vec3::new(d.pos[0], d.pos[1], d.pos[2]),
                    Vec3::new(d.dir[0], d.dir[1], d.dir[2]),
                    Vec3::new(d.rot[0], d.rot[1], d.rot[2]),
                )
            })
            .collect()
    });
    let lifting_surfaces = cfg
        .lifting_surfaces
        .iter()
        .map(|s| {
            let kind = match s.kind {
                orbitx_config::FinKindConfig::Fixed => FinKind::Fixed,
                orbitx_config::FinKindConfig::Grid => FinKind::Grid,
            };
            let normal = Vec3::new(s.normal[0], s.normal[1], s.normal[2]);
            let chord_dir = s
                .chord_dir
                .map(|c| Vec3::new(c[0], c[1], c[2]))
                .unwrap_or_else(|| default_chord_dir(normal));
            let alpha_stall0 = s.alpha_stall0.unwrap_or(match kind {
                FinKind::Fixed => orbitx_dynamics::DEFAULT_ALPHA_STALL_FIN,
                FinKind::Grid => orbitx_dynamics::DEFAULT_ALPHA_STALL_GRID,
            });
            LiftingSurfaceSpec {
                ref_pos: Vec3::new(s.ref_pos[0], s.ref_pos[1], s.ref_pos[2]),
                normal,
                chord_dir,
                area: s.area,
                aspect_ratio: s.aspect_ratio,
                cl_alpha: s.cl_alpha,
                cd0: s.cd0,
                edge_area: s.edge_area,
                alpha_stall0,
                kind,
                deploy: s.deploy,
                deploy_target: s.deploy_target,
                deploy_rate: s.deploy_rate,
            }
        })
        .collect();
    StageSpec {
        name,
        dry_mass: cfg.dry_mass,
        dry_center: Vec3::new(cfg.dry_center[0], cfg.dry_center[1], cfg.dry_center[2]),
        dry_inertia: Vec3::new(cfg.dry_inertia[0], cfg.dry_inertia[1], cfg.dry_inertia[2]),
        tanks,
        thrusters,
        length: cfg.length,
        radius: cfg.radius,
        separation_impulse: cfg.separation_impulse,
        tidaldamp: cfg.tidaldamp,
        cd_mach: cfg.cd_mach.iter().map(|p| (p[0], p[1])).collect(),
        cn_alpha: cfg.cn_alpha,
        lifting_surfaces,
        docks,
    }
}

fn vessel_surface_from_spec(s: &LiftingSurfaceSpec) -> VesselLiftingSurface {
    VesselLiftingSurface {
        surf: LiftingSurface {
            ref_pos: s.ref_pos,
            normal: s.normal,
            chord_dir: s.chord_dir,
            area: s.area,
            aspect_ratio: s.aspect_ratio,
            cl_alpha: s.cl_alpha,
            cd0: s.cd0,
            edge_area: s.edge_area,
            alpha_stall0: s.alpha_stall0,
            kind: s.kind,
            deploy: s.deploy.clamp(0.0, 1.0),
            // 步初由 update_leeward_sheltered 写入。
            leeward_sheltered: false,
        },
        deploy_target: s.deploy_target,
        deploy_rate: s.deploy_rate,
    }
}
