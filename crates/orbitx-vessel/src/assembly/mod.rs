//! Assembly：多 Vessel 组合体管理（Orbiter SuperVessel 语义子集）。
//!
//! 刚体姿态动力学（移植自 Orbiter）：
//! - 推力力矩：每个推进器在体坐标系产生 `τ = F × r`（`Vessel.cpp:4024`）。
//! - 重力梯度力矩：`gravity_gradient_torque`（`Rigidbody.cpp:345-363`）。
//! - 组合体 PMI：[`crate::supervessel::composite_pmi`]（`SuperVessel::CalcPMI`）。
//! - 力合成：[`crate::supervessel::add_component_force_and_moment`]。
//! - Euler 方程：`euler_inv_full`；输入为质量归一化力矩。
//!
//! 分离语义（本轮）：一次 `undock` 拆口对面连通分量；不实现两边皆复合体时
//! 拆成两个 SuperVessel（见 `docs/ORBITER_QUIRKS.md`）。

mod aero_geom;

use std::cell::RefCell;
use std::collections::{HashMap, HashSet, VecDeque};
use std::sync::Arc;

use crate::aero::{
    compute_aero_forces, compute_rocket_aero, slew_deploy, update_leeward_sheltered,
    world_to_airvel_ship, AeroForces, Atmosphere, LiftingSurface, RocketAeroInput, RocketBodyAero,
};
use crate::attitude::{pitch_yaw_angles, roll_angle, tip_angle};
use crate::pad::surface_inertial_velocity;
use crate::stage::StageSpec;
use crate::supervessel::{
    add_component_force_and_moment, center_of_mass, component_state_vectors, composite_pmi,
    rel_docking_pos, supervessel_state_from_root, SubVesselData,
};
use crate::vessel::Vessel;
use orbitx_dynamics::euler_inv_full;
use orbitx_dynamics::gacc_nbody;
use orbitx_dynamics::gravity_gradient_torque;
use orbitx_dynamics::GravBody;
use orbitx_math::{cross, mul, Matrix3, Quat, StateVectors, Vec3};

use crate::thruster::G0;

use self::aero_geom::{compute_cluster_aero_geom, ClusterAeroGeom};

pub use crate::diagnostics::FlightDiagnostics;

#[derive(Clone, Copy, Debug, Default)]
struct StepThrustTelem {
    thrust: f64,
    thrust_atm_scale: f64,
    isp_eff: f64,
}

#[derive(Clone, Debug, Default)]
struct StepAeroTelem {
    last_aero: Option<AeroForces>,
    last_rho: f64,
    last_nongrav_acc: f64,
    last_g_acc: f64,
}

struct StepDiagInput {
    thrust: StepThrustTelem,
    p_amb: f64,
    temperature: f64,
    sound_speed: f64,
    density: f64,
    aero: AeroForces,
    a_grav: f64,
    load_factor: f64,
}

/// 一步物理积分的环境帧（由宿主 / PlanetarySystem 采样；vessel 不猜测中心天体）。
///
/// - `grav_bodies`：与飞船同一坐标系下的引力体（Runtime：地心系）。
/// - `primary`：重力梯度与地表半径参考体，索引进 `grav_bodies`。
///
/// `atmosphere` / `sid_rot_period` / `planet_radius` 由宿主每 tick 从环境主天体写入。
#[derive(Clone, Copy)]
pub struct StepEnv<'a> {
    pub grav_bodies: &'a [GravBody],
    pub primary: usize,
}

impl<'a> StepEnv<'a> {
    pub fn new(grav_bodies: &'a [GravBody], primary: usize) -> Self {
        Self {
            grav_bodies,
            primary,
        }
    }

    /// 单参考体或测试：`primary = 0`（空列表时梯度力矩跳过）。
    pub fn primary0(grav_bodies: &'a [GravBody]) -> Self {
        Self {
            grav_bodies,
            primary: 0,
        }
    }

    pub(crate) fn primary_body(&self) -> Option<&'a GravBody> {
        self.grav_bodies.get(self.primary)
    }
}

/// 管理多个 Vessel 的组合体。
///
/// 级从底到顶排列：vessels[0] = 第一级（底），最后 = 有效载荷（顶）。
/// `active` 指向当前主控级（分离后自动切换到下一级）。
/// `state` 为当前主组合体（含 `active` 的连通分量）的 CG 状态。
pub struct Assembly {
    /// 所有 Vessel。
    pub vessels: Vec<Vessel>,
    /// 当前活动级的索引。
    pub active: usize,
    /// 主组合体 root（组合体坐标系 = 该船的体坐标）。
    pub root: usize,
    /// 主组合体子船布局（相对 root）。
    pub components: Vec<SubVesselData>,
    /// 主组合体状态（`pos` = CG）。
    pub state: StateVectors,
    /// 大气模型（宿主注入；`None` 则不计算气动力）。
    pub atmosphere: Option<Box<dyn Atmosphere>>,
    /// 参考天体半径 [m]（宿主注入，与 `StepEnv::primary` 同源；用于步进外读数）。
    pub planet_radius: f64,
    /// 参考天体恒星自转周期 [s]（宿主注入）；`>0` 时大气随 `ω×r` 共转。
    pub sid_rot_period: f64,
    /// 刚体气动静态缓存（包络 + 烘焙筒体系数）；对接/分离时清空；与 `active` 无关。
    aero_static_cache: HashMap<Vec<usize>, RigidAeroStatic>,
}

/// 拓扑不变时复用的刚体气动静态量（几何包络 + 筒体系数）。
#[derive(Clone, Debug)]
struct RigidAeroStatic {
    geom: ClusterAeroGeom,
    body: RocketBodyAero,
}

impl Assembly {
    /// 从级定义列表创建多级火箭，并对相邻顶/底口硬对接。
    ///
    /// stages[0] = 底层级（第一级），最后一位 = 有效载荷。
    pub fn new(stages: &[StageSpec], initial_state: StateVectors) -> Self {
        let n = stages.len();
        let links: Vec<(usize, usize, usize, usize)> =
            (0..n.saturating_sub(1)).map(|i| (i, 1, i + 1, 0)).collect();
        Self::with_dock_links(stages, initial_state, &links)
    }

    /// 从级定义与显式对接边构建组合体。
    ///
    /// `links` 元素为 `(stage_a, port_a, stage_b, port_b)`。
    pub fn with_dock_links(
        stages: &[StageSpec],
        initial_state: StateVectors,
        links: &[(usize, usize, usize, usize)],
    ) -> Self {
        let mut vessels = Vec::with_capacity(stages.len());
        for (id, spec) in stages.iter().enumerate() {
            vessels.push(Vessel::from_spec(id as u64, spec, initial_state));
        }

        let mut asm = Assembly {
            vessels,
            active: 0,
            root: 0,
            components: Vec::new(),
            state: initial_state,
            atmosphere: None,
            planet_radius: 0.0,
            sid_rot_period: 0.0,
            aero_static_cache: HashMap::new(),
        };

        for &(a, pa, b, pb) in links {
            let _ = asm.dock(a, pa, b, pb, false);
        }
        asm.rebuild_primary_from_active();
        asm.writeback_primary_states();
        asm
    }

    /// 从一组已构造的 Vessel 创建空布局组合体（调用方随后 `dock`）。
    pub fn from_vessels(vessels: Vec<Vessel>, active: usize) -> Self {
        let state = vessels.get(active).map(|v| v.state).unwrap_or_default();
        let mut asm = Assembly {
            vessels,
            active,
            root: active,
            components: Vec::new(),
            state,
            atmosphere: None,
            planet_radius: 0.0,
            sid_rot_period: 0.0,
            aero_static_cache: HashMap::new(),
        };
        asm.rebuild_primary_from_active();
        asm.writeback_primary_states();
        asm
    }

    /// 在两艘船的指定端口之间硬对接（双方须尚未占用该口）。
    ///
    /// `mix_moments`：若 true，合并线动量到组合体；否则保留当前主组合体速度。
    pub fn dock(
        &mut self,
        idx_a: usize,
        port_a: usize,
        idx_b: usize,
        port_b: usize,
        mix_moments: bool,
    ) -> bool {
        if idx_a >= self.vessels.len()
            || idx_b >= self.vessels.len()
            || idx_a == idx_b
            || self.vessels[idx_a].detached
            || self.vessels[idx_b].detached
        {
            return false;
        }
        if port_a >= self.vessels[idx_a].docks.len() || port_b >= self.vessels[idx_b].docks.len() {
            return false;
        }
        if self.vessels[idx_a].docks[port_a].connected_to.is_some()
            || self.vessels[idx_b].docks[port_b].connected_to.is_some()
        {
            return false;
        }

        let id_a = self.vessels[idx_a].id;
        let id_b = self.vessels[idx_b].id;
        self.vessels[idx_a].docks[port_a].connected_to = Some((id_b, port_b));
        self.vessels[idx_b].docks[port_b].connected_to = Some((id_a, port_a));

        // 以含 active 的连通分量为准重建；若两边都不在 active 分量，以 a 为 root。
        self.rebuild_primary_from_active();
        if !self.components.iter().any(|c| c.vessel_index == idx_a)
            && !self.components.iter().any(|c| c.vessel_index == idx_b)
        {
            self.root = idx_a;
            self.rebuild_components_from_root();
        }

        if mix_moments {
            let masses: Vec<f64> = self
                .components
                .iter()
                .map(|c| self.vessels[c.vessel_index].mass())
                .collect();
            let m_tot: f64 = masses.iter().sum();
            if m_tot > 1e-3 {
                let mut vel = Vec3::ZERO;
                for (c, m) in self.components.iter().zip(masses.iter()) {
                    vel += self.vessels[c.vessel_index].state.vel * *m;
                }
                self.state.vel = vel * (1.0 / m_tot);
            }
        }

        self.invalidate_aero_geom();
        self.writeback_primary_states();
        true
    }

    /// 按口分离：切断 `(vessel_id, port)`，将不含 `active` 的那一侧连通分量拆出。
    ///
    /// 返回被拆出（`detached`）的 vessel 下标。分离速度沿本口 `dir`，按质量比分配。
    pub fn undock(&mut self, vessel_id: u64, port: usize, vsep: f64) -> Vec<usize> {
        let Some(idx) = self.vessels.iter().position(|v| v.id == vessel_id) else {
            return Vec::new();
        };
        if port >= self.vessels[idx].docks.len() {
            return Vec::new();
        }
        let Some((mate_id, mate_port)) = self.vessels[idx].docks[port].connected_to else {
            return Vec::new();
        };
        let Some(mate_idx) = self.vessels.iter().position(|v| v.id == mate_id) else {
            return Vec::new();
        };

        // 分离方向（组合体坐标 → 世界）：本口 dir。
        let sep_dir_body = self.vessels[idx].docks[port].dir;
        let (rp_idx, rrot_idx) = self
            .component_pose(idx)
            .unwrap_or((Vec3::ZERO, Matrix3::IDENTITY));
        let sep_dir_sv = mul(rrot_idx, sep_dir_body);
        let sep_dir_world = mul(self.state.r, sep_dir_sv);
        let cg = self.primary_cg();
        let base_vel = self.state.vel;
        let omega = self.state.omega;
        let r_sv = self.state.r;

        // 先写回各子船位姿，再断连。
        self.writeback_primary_states();

        self.vessels[idx].docks[port].connected_to = None;
        if mate_port < self.vessels[mate_idx].docks.len() {
            self.vessels[mate_idx].docks[mate_port].connected_to = None;
        }

        let side_a = self.connected_component_excluding(idx, mate_idx);
        let side_b = self.connected_component_excluding(mate_idx, idx);

        let (keep, leave) = if side_a.contains(&self.active) {
            (side_a, side_b)
        } else if side_b.contains(&self.active) {
            (side_b, side_a)
        } else {
            // active 不在任一侧（异常）；保留较大侧
            if side_a.len() >= side_b.len() {
                (side_a, side_b)
            } else {
                (side_b, side_a)
            }
        };

        let mass_leave: f64 = leave.iter().map(|&i| self.vessels[i].mass()).sum();
        let mass_keep: f64 = keep.iter().map(|&i| self.vessels[i].mass()).sum();
        let mass_tot = (mass_leave + mass_keep).max(1e-3);
        let v_struct = vsep * mass_leave / mass_tot;
        let v_leave = vsep - v_struct;

        for &i in &leave {
            let (rp, _) = self
                .component_pose(i)
                .unwrap_or((rp_idx, Matrix3::IDENTITY));
            let rotvel = mul(r_sv, cross(rp - cg, omega));
            // 口对面离开：沿 +sep_dir（相对 keep）
            self.vessels[i].state.vel = base_vel + sep_dir_world * v_leave + rotvel;
            self.vessels[i].detached = true;
        }
        self.state.vel = base_vel - sep_dir_world * v_struct;

        if !keep.contains(&self.active) {
            self.active = *keep.first().unwrap_or(&self.active);
        }
        self.rebuild_primary_from_active();
        self.invalidate_aero_geom();
        self.writeback_primary_states();
        leave
    }

    /// 分离最底层未分离级（兼容旧 API）：对其与上级的连接口调用 `undock`。
    pub fn separate_stage(&mut self) -> usize {
        let bottom = self
            .vessels
            .iter()
            .position(|v| !v.detached)
            .unwrap_or(self.active);
        let attached = self.vessels.iter().filter(|v| !v.detached).count();
        if attached <= 1 {
            return self.active;
        }

        let bottom_id = self.vessels[bottom].id;
        let sep = self.vessels[bottom].separation_impulse;
        let port = match self.vessels[bottom]
            .docks
            .iter()
            .position(|d| d.connected_to.is_some())
        {
            Some(p) => p,
            None => return self.active,
        };

        // 主控切到对接对方，使 undock 保留上级栈、拆走底级。
        if let Some((mate_id, _)) = self.vessels[bottom].docks[port].connected_to {
            if let Some(mi) = self.vessel_index_by_id(mate_id) {
                self.active = mi;
            }
        }

        let _ = self.undock(bottom_id, port, sep);
        self.active
    }

    /// 当前主组合体总质量（未 detached 且在 components 中）。
    pub fn total_mass(&self) -> f64 {
        self.components
            .iter()
            .map(|c| self.vessels[c.vessel_index].mass())
            .sum()
    }

    /// 当前燃料总量 [kg]（主组合体）。
    pub fn total_fuel(&self) -> f64 {
        self.components
            .iter()
            .map(|c| self.vessels[c.vessel_index].fuel_mass())
            .sum()
    }

    /// 燃料百分比（0..100），相对罐池 `max_mass` 之和。
    pub fn fuel_percent(&self) -> f64 {
        let current: f64 = self.vessels.iter().map(|v| v.fuel_mass()).sum();
        let max: f64 = self.vessels.iter().map(|v| v.fuel_max()).sum();
        if max > 0.0 {
            (current / max * 100.0).min(100.0)
        } else {
            0.0
        }
    }

    /// 油门设置（仅设置活动级的推进器）。
    pub fn set_throttle(&mut self, level: f64) {
        self.vessels[self.active].set_throttle(level);
    }

    /// 当前推力 [N]（仅活动级；按当前高度气压缩放）。
    pub fn current_thrust(&self) -> f64 {
        let p = self.ambient_pressure();
        self.vessels[self.active].current_thrust(p)
    }

    /// 当前高度处大气压 [Pa]（无大气则为 0）。
    pub fn ambient_pressure(&self) -> f64 {
        let alt = self.state.pos.length() - self.planet_radius;
        self.atmosphere
            .as_ref()
            .map(|a| a.pressure(alt))
            .unwrap_or(0.0)
    }

    /// 主组合体 tidaldamp（取活动船；无则 0）。
    fn primary_tidaldamp(&self) -> f64 {
        self.vessels
            .get(self.active)
            .map(|v| v.tidaldamp)
            .unwrap_or(0.0)
    }

    /// 合成主组合体归一化 PMI。
    pub fn composite_pmi(&self) -> Vec3 {
        let masses: Vec<f64> = self
            .components
            .iter()
            .map(|c| self.vessels[c.vessel_index].mass())
            .collect();
        let pmis: Vec<Vec3> = self
            .components
            .iter()
            .map(|c| self.vessels[c.vessel_index].pmi)
            .collect();
        let cg = center_of_mass(&self.components, &masses);
        composite_pmi(&self.components, &masses, &pmis, cg)
    }

    /// 活动级诊断（兼容旧 `asm.diagnostics` 读法）。
    pub fn diagnostics(&self) -> &FlightDiagnostics {
        &self.vessels[self.active.min(self.vessels.len().saturating_sub(1))].diagnostics
    }

    /// 上层判定坠毁后置位：清零该船速度/角速度；若在主栈则同步冻结 `state`。
    pub fn mark_crashed(&mut self, vi: usize) {
        if vi >= self.vessels.len() {
            return;
        }
        self.vessels[vi].crashed = true;
        self.vessels[vi].state.vel = Vec3::ZERO;
        self.vessels[vi].state.omega = Vec3::ZERO;
        if self.components.iter().any(|c| c.vessel_index == vi) {
            self.state.vel = Vec3::ZERO;
            self.state.omega = Vec3::ZERO;
        }
    }

    /// 一步物理积分（主组合体 + 已分离独立体）。
    ///
    /// 环境由 [`StepEnv`] 显式给出；重力梯度与步进内高度用 `env.primary`，不猜测列表首元。
    pub fn step(&mut self, dt: f64, env: StepEnv<'_>) {
        self.step_primary(dt, env);
        self.step_detached(dt, env);
    }

    fn step_primary(&mut self, dt: f64, env: StepEnv<'_>) {
        if self.total_mass() < 1e-3 || self.components.is_empty() {
            return;
        }
        if self
            .components
            .iter()
            .any(|c| self.vessels[c.vessel_index].crashed)
        {
            return;
        }
        let comps = self.components.clone();
        let aero_vi = self.active;
        let tidaldamp = self.primary_tidaldamp();
        let mut state = self.state;
        self.step_rigid_cluster(&comps, &mut state, aero_vi, tidaldamp, dt, env);
        self.state = state;
    }

    fn step_detached(&mut self, dt: f64, env: StepEnv<'_>) {
        let primary: HashSet<usize> = self.components.iter().map(|c| c.vessel_index).collect();
        let detached_idx: Vec<usize> = self
            .vessels
            .iter()
            .enumerate()
            .filter(|(i, v)| v.detached || !primary.contains(i))
            .map(|(i, _)| i)
            .collect();

        for vi in detached_idx {
            if primary.contains(&vi) {
                continue;
            }
            if self.vessels[vi].crashed {
                continue;
            }
            if self.vessels[vi].mass() < 1e-3 {
                continue;
            }
            let comps = [SubVesselData {
                vessel_index: vi,
                rpos: Vec3::ZERO,
                rrot: Matrix3::IDENTITY,
                rq: Quat::IDENTITY,
            }];
            let tidaldamp = self.vessels[vi].tidaldamp;
            let mut state = self.vessels[vi].state;
            self.step_rigid_cluster(&comps, &mut state, vi, tidaldamp, dt, env);
        }
    }

    /// 对任意刚体簇（主组合体或单船分离体）做完整物理积分；结束后刷新簇内每船 `diagnostics`。
    fn step_rigid_cluster(
        &mut self,
        components: &[SubVesselData],
        state: &mut StateVectors,
        aero_vessel_index: usize,
        tidaldamp: f64,
        dt: f64,
        env: StepEnv<'_>,
    ) {
        let grav_bodies = env.grav_bodies;
        let primary_pos = env.primary_body().map(|b| b.pos).unwrap_or(Vec3::ZERO);
        let primary_i = env.primary;

        for c in components {
            for t in &mut self.vessels[c.vessel_index].thrusters {
                t.slew_throttle(dt);
            }
            self.vessels[c.vessel_index].refresh_pmi();
        }

        let masses: Vec<f64> = components
            .iter()
            .map(|c| self.vessels[c.vessel_index].mass())
            .collect();
        let total_mass: f64 = masses.iter().sum();
        if total_mass < 1e-3 || components.is_empty() {
            return;
        }

        // 质量点 = 船原点 + 体坐标瞬时 COM（满载原点约定下）。
        let mass_comps: Vec<SubVesselData> = components
            .iter()
            .map(|c| {
                let com = self.vessels[c.vessel_index].com_body();
                let mut mc = c.clone();
                mc.rpos = c.rpos + mul(c.rrot, com);
                mc
            })
            .collect();
        let cg = center_of_mass(&mass_comps, &masses);
        let pmis: Vec<Vec3> = components
            .iter()
            .map(|c| self.vessels[c.vessel_index].pmi)
            .collect();
        let cluster_pmi = composite_pmi(&mass_comps, &masses, &pmis, cg);

        let planet_radius = env
            .primary_body()
            .map(|b| b.size)
            .filter(|r| *r > 0.0)
            .unwrap_or(self.planet_radius);
        let alt0 = state.pos.length() - planet_radius;
        let (p_amb, temperature0, a_snd_step) = if let Some(atm) = self.atmosphere.as_ref() {
            (
                atm.pressure(alt0),
                atm.temperature(alt0),
                atm.sound_speed(alt0).max(1.0),
            )
        } else {
            (0.0, 0.0, 1.0)
        };

        // 步初燃料折扣（整级罐池、多引擎统一 s）并冻结推力；同步填推力遥测 scratch。
        let mut thrust_by_comp: Vec<(usize, Vec3, Vec3)> = Vec::new();
        let mut consume_by_vessel: Vec<(usize, f64)> = Vec::new();
        let mut thrust_telem: HashMap<usize, StepThrustTelem> = HashMap::new();
        for (ci, c) in components.iter().enumerate() {
            let vi = c.vessel_index;
            let s_fuel = self.vessels[vi].fuel_thrust_scale(p_amb, dt);
            let com = self.vessels[vi].com_body();
            let mut f = Vec3::ZERO;
            let mut m = Vec3::ZERO;
            let mut mdot = 0.0;
            let mut thrust_scale_w = 0.0;
            let mut thrust_scale_sum = 0.0;
            let mut isp_w = 0.0;
            let mut isp_sum = 0.0;
            {
                let v = &self.vessels[vi];
                let eta = v.eta_pool();
                for t in &v.thrusters {
                    if t.level > 0.0 && s_fuel > 0.0 {
                        let thrust = t.current_thrust(p_amb) * s_fuel;
                        let dir = t.current_dir();
                        let fb = dir * thrust;
                        f += fb;
                        m += cross(t.pos - com, fb);
                        let isp_e = t.effective_isp(p_amb);
                        mdot += orbitx_dynamics::propulsion::mass_flow_rate_eff(thrust, isp_e, eta);
                        let w = thrust.max(0.0);
                        if w > 0.0 {
                            thrust_scale_w += w;
                            thrust_scale_sum += w * t.atm_scale(p_amb);
                            isp_w += w;
                            isp_sum += w * isp_e;
                        }
                    }
                }
            }
            thrust_telem.insert(
                vi,
                StepThrustTelem {
                    thrust: f.length(),
                    thrust_atm_scale: if thrust_scale_w > 0.0 {
                        thrust_scale_sum / thrust_scale_w
                    } else {
                        1.0
                    },
                    isp_eff: if isp_w > 0.0 { isp_sum / isp_w } else { 0.0 },
                },
            );
            if f.length() > 0.0 || m.length() > 0.0 {
                thrust_by_comp.push((ci, f, m));
            }
            if mdot > 0.0 {
                consume_by_vessel.push((vi, mdot * dt));
            }
        }

        let aero_vi = aero_vessel_index.min(self.vessels.len().saturating_sub(1));
        let use_rocket_aero = cluster_uses_rocket_aero(&self.vessels, components);

        // 步初：整步 dt 限速展收；RK 子步冻结。
        if use_rocket_aero {
            for c in components {
                for s in &mut self.vessels[c.vessel_index].lifting_surfaces {
                    s.surf.deploy =
                        slew_deploy(s.surf.deploy, s.deploy_target, s.deploy_rate, dt);
                }
            }
        }

        // 刚体静态量（拓扑缓存）+ 翼副本；背风标志按步初空速判定后冻结。
        let rocket_frozen: Option<(ClusterAeroGeom, RocketBodyAero, Vec<LiftingSurface>)> =
            if use_rocket_aero {
                let (geom, body) = self.rigid_aero_static(components);
                let mut surfaces = collect_cluster_surfaces(&self.vessels, components);
                let wind0 = if self.sid_rot_period > 1e-9 {
                    surface_inertial_velocity(state.pos, self.sid_rot_period)
                } else {
                    Vec3::ZERO
                };
                let airvel0 = world_to_airvel_ship(state.vel, wind0, state.r);
                update_leeward_sheltered(&mut surfaces, airvel0, |y| geom.body_radius_at_y(y));
                Some((geom, body, surfaces))
            } else {
                None
            };

        let aero_airfoils = self.vessels[aero_vi].airfoils.clone();
        let aero_ctrlsurfs = self.vessels[aero_vi].ctrlsurfs.clone();
        let aero_dragels = self.vessels[aero_vi].dragels.clone();
        let aero_cs = self.vessels[aero_vi].cross_section;
        let aero_rdrag = self.vessels[aero_vi].rdrag;

        let sid_rot_period = self.sid_rot_period;
        let rho_fn: Option<Arc<dyn Fn(f64) -> f64 + Send + Sync>> =
            self.atmosphere.as_ref().map(|atm| atm.density_fn());

        let (cbody_mass, cbody_pos) = match env.primary_body() {
            Some(b) => (b.mass, b.pos),
            None => (0.0, Vec3::ZERO),
        };

        let n_sub = 4;
        let sub_dt = dt / n_sub as f64;
        let mut current_state = *state;
        let aero_telem = RefCell::new(StepAeroTelem::default());

        for _ in 0..n_sub {
            // snap_rot：Copy；其余不变数据借自 n_sub 外（整步冻结）。
            let snap_rot = current_state.r;
            let mut force = |s: &StateVectors, _t: f64| {
                let g_acc = gacc_nbody(s.pos, grav_bodies, None)
                    - gacc_nbody(primary_pos, grav_bodies, Some(primary_i));

                let mut f_sv = Vec3::ZERO;
                let mut m_sv = Vec3::ZERO;
                for &(ci, f_comp, m_comp) in &thrust_by_comp {
                    add_component_force_and_moment(
                        &mut f_sv,
                        &mut m_sv,
                        f_comp,
                        m_comp,
                        &mass_comps[ci],
                        cg,
                    );
                }

                let mut nongrav_acc = mul(snap_rot, f_sv) / total_mass;
                let torque = m_sv;
                let mut aero_torque_body = Vec3::ZERO;

                if let Some(rho_fn) = &rho_fn {
                    let alt = s.pos.length() - planet_radius;
                    let rho = rho_fn(alt);
                    if rho > 1e-15 {
                        let wind = if sid_rot_period > 1e-9 {
                            surface_inertial_velocity(s.pos, sid_rot_period)
                        } else {
                            Vec3::ZERO
                        };
                        let airvel_ship = world_to_airvel_ship(s.vel, wind, snap_rot);
                        let aero = if let Some((geom, body, surfaces)) = rocket_frozen.as_ref()
                        {
                            compute_rocket_aero(&RocketAeroInput {
                                airvel_body: airvel_ship,
                                omega_body: s.omega,
                                rho,
                                sound_speed: a_snd_step,
                                areas: geom.areas,
                                body_cop: geom.body_cop,
                                cg,
                                body,
                                surfaces,
                            })
                        } else {
                            compute_aero_forces(
                                &aero_airfoils,
                                &aero_ctrlsurfs,
                                &aero_dragels,
                                airvel_ship,
                                rho,
                                s.omega,
                                cluster_pmi,
                                total_mass,
                                aero_cs,
                                aero_rdrag,
                                sub_dt,
                                a_snd_step,
                            )
                        };
                        nongrav_acc += mul(snap_rot, aero.force) / total_mass;
                        aero_torque_body = aero.torque;
                        let mut telem = aero_telem.borrow_mut();
                        telem.last_aero = Some(aero);
                        telem.last_rho = rho;
                    }
                }

                {
                    let mut telem = aero_telem.borrow_mut();
                    telem.last_nongrav_acc = nongrav_acc.length();
                    telem.last_g_acc = g_acc.length();
                }

                let gg_torque = if cbody_mass > 0.0 {
                    gravity_gradient_torque(
                        cbody_pos - s.pos,
                        cbody_mass,
                        cluster_pmi,
                        snap_rot,
                        s.omega,
                        tidaldamp,
                        sub_dt,
                        false,
                    )
                } else {
                    Vec3::ZERO
                };

                let tau = (torque + aero_torque_body) / total_mass + gg_torque;
                let arot = euler_inv_full(tau, s.omega, cluster_pmi);
                (g_acc + nongrav_acc, arot)
            };

            current_state = orbitx_dynamics::rk4_step(current_state, sub_dt, &mut force);
        }

        *state = current_state;
        // 状态写回仍用船原点（满载导出原点）相对簇质心。
        for c in components {
            self.vessels[c.vessel_index].state =
                component_state_vectors(&current_state, c, cg);
        }

        let aero_scratch = aero_telem.into_inner();
        let vessel_indices: Vec<usize> = components.iter().map(|c| c.vessel_index).collect();
        for vi in vessel_indices {
            let thrust = thrust_telem.get(&vi).copied().unwrap_or_default();
            // 组合体气动记在 aero_vi；其余船气动遥测置 0（推力仍用本船冻结值）。
            let (aero, density, load_factor, a_grav) = if vi == aero_vi {
                (
                    aero_scratch
                        .last_aero
                        .clone()
                        .unwrap_or_default(),
                    aero_scratch.last_rho,
                    aero_scratch.last_nongrav_acc / G0,
                    aero_scratch.last_g_acc,
                )
            } else {
                (
                    AeroForces::default(),
                    0.0,
                    0.0,
                    aero_scratch.last_g_acc,
                )
            };
            self.apply_step_diagnostics(
                vi,
                StepDiagInput {
                    thrust,
                    p_amb,
                    temperature: temperature0,
                    sound_speed: a_snd_step,
                    density,
                    aero,
                    a_grav,
                    load_factor,
                },
            );
        }

        for (vi, amount) in consume_by_vessel {
            self.vessels[vi].consume_fuel_pool(amount);
            self.vessels[vi].refresh_pmi();
        }
    }

    /// 将本步冻结/采样结果写入 `diagnostics`（不做第二套物理）。
    fn apply_step_diagnostics(&mut self, vi: usize, input: StepDiagInput) {
        let st = self.vessels[vi].state;
        let (pitch, yaw) = pitch_yaw_angles(&st);
        let roll = roll_angle(&st);
        let tip = tip_angle(&st);
        self.vessels[vi].diagnostics = FlightDiagnostics {
            a_grav: input.a_grav,
            g_multiple: input.a_grav / G0,
            mach: input.aero.mach,
            density: input.density,
            dynamic_pressure: input.aero.dynamic_pressure,
            pressure: input.p_amb,
            thrust_atm_scale: input.thrust.thrust_atm_scale,
            isp_eff: input.thrust.isp_eff,
            thrust: input.thrust.thrust,
            drag_force: input.aero.drag_force,
            cd_eff: input.aero.cd_eff,
            temperature: input.temperature,
            sound_speed: input.sound_speed,
            load_factor: input.load_factor,
            pitch,
            yaw,
            roll,
            tip,
        };
    }

    /// 主控级渲染信息。
    pub fn render_state(&self) -> (Vec3, Quat) {
        let v = &self.vessels[self.active];
        (v.state.pos, v.state.q)
    }

    /// 未分离级数量。
    pub fn stage_count(&self) -> usize {
        self.vessels.iter().filter(|v| !v.detached).count()
    }

    /// 当前活动级名称。
    pub fn active_name(&self) -> &str {
        &self.vessels[self.active].name
    }

    // ── 内部：布局 ──────────────────────────────────────────

    fn primary_cg(&self) -> Vec3 {
        let masses: Vec<f64> = self
            .components
            .iter()
            .map(|c| self.vessels[c.vessel_index].mass())
            .collect();
        center_of_mass(&self.components, &masses)
    }

    fn component_pose(&self, vessel_index: usize) -> Option<(Vec3, Matrix3)> {
        self.components
            .iter()
            .find(|c| c.vessel_index == vessel_index)
            .map(|c| (c.rpos, c.rrot))
    }

    fn vessel_index_by_id(&self, id: u64) -> Option<usize> {
        self.vessels.iter().position(|v| v.id == id)
    }

    /// 从 `start` BFS，不经过 `excluded`。
    fn connected_component_excluding(&self, start: usize, excluded: usize) -> Vec<usize> {
        let mut seen = HashSet::new();
        let mut q = VecDeque::new();
        q.push_back(start);
        seen.insert(start);
        while let Some(i) = q.pop_front() {
            for d in &self.vessels[i].docks {
                if let Some((tid, _)) = d.connected_to {
                    if let Some(j) = self.vessel_index_by_id(tid) {
                        if j == excluded || self.vessels[j].detached {
                            continue;
                        }
                        if seen.insert(j) {
                            q.push_back(j);
                        }
                    }
                }
            }
        }
        let mut v: Vec<_> = seen.into_iter().collect();
        v.sort_unstable();
        v
    }

    fn invalidate_aero_geom(&mut self) {
        self.aero_static_cache.clear();
    }

    /// 刚体气动静态量：拓扑不变则缓存；与 `active` 无关。
    fn rigid_aero_static(
        &mut self,
        components: &[SubVesselData],
    ) -> (ClusterAeroGeom, RocketBodyAero) {
        let mut key: Vec<usize> = components.iter().map(|c| c.vessel_index).collect();
        key.sort_unstable();
        if let Some(cached) = self.aero_static_cache.get(&key) {
            return (cached.geom.clone(), cached.body.clone());
        }
        let geom = compute_cluster_aero_geom(&self.vessels, components);
        let body = bake_cluster_rocket_body(&self.vessels, components);
        self.aero_static_cache.insert(
            key,
            RigidAeroStatic {
                geom: geom.clone(),
                body: body.clone(),
            },
        );
        (geom, body)
    }

    fn rebuild_primary_from_active(&mut self) {
        if self.vessels.is_empty() {
            self.components.clear();
            return;
        }
        if self.vessels[self.active].detached {
            self.active = self.vessels.iter().position(|v| !v.detached).unwrap_or(0);
        }
        self.root = self.active;
        // 选连通分量中 id 最小者作稳定 root，便于同轴回归
        let comp = self.connected_component_excluding(self.active, usize::MAX);
        if let Some(&r) = comp.iter().min() {
            self.root = r;
        }
        self.rebuild_components_from_root();

        // 用 root 船状态推组合体 CG 状态
        let cg = self.primary_cg();
        let root_state = self.vessels[self.root].state;
        self.state = supervessel_state_from_root(&root_state, cg);
    }

    fn rebuild_components_from_root(&mut self) {
        self.components.clear();
        if self.vessels.is_empty() || self.vessels[self.root].detached {
            return;
        }

        let mut poses: HashMap<usize, (Vec3, Matrix3)> = HashMap::new();
        poses.insert(self.root, (Vec3::ZERO, Matrix3::IDENTITY));
        let mut q = VecDeque::new();
        q.push_back(self.root);

        while let Some(i) = q.pop_front() {
            let (parent_pos, parent_rot) = poses[&i];
            let docks = self.vessels[i].docks.clone();
            for (pi, d) in docks.iter().enumerate() {
                let Some((tid, tport)) = d.connected_to else {
                    continue;
                };
                let Some(j) = self.vessel_index_by_id(tid) else {
                    continue;
                };
                if self.vessels[j].detached || poses.contains_key(&j) {
                    continue;
                }
                if tport >= self.vessels[j].docks.len() {
                    continue;
                }
                let (rel_pos, rel_rot) =
                    rel_docking_pos(&self.vessels[i].docks[pi], &self.vessels[j].docks[tport]);
                // child in root frame
                let child_rot = parent_rot.matmul(rel_rot);
                let child_pos = parent_pos + mul(parent_rot, rel_pos);
                poses.insert(j, (child_pos, child_rot));
                q.push_back(j);
            }
        }

        let mut indices: Vec<usize> = poses.keys().copied().collect();
        indices.sort_unstable();
        for i in indices {
            let (rpos, rrot) = poses[&i];
            self.components.push(SubVesselData {
                vessel_index: i,
                rpos,
                rrot,
                rq: Quat::from_matrix(rrot),
            });
        }
    }

    fn writeback_primary_states(&mut self) {
        let cg = self.primary_cg();
        let state = self.state;
        let comps = self.components.clone();
        for c in &comps {
            self.vessels[c.vessel_index].state = component_state_vectors(&state, c, cg);
        }
    }
}

/// 火箭路径：簇内有 `rocket_body` / 升力面，且无 P1.1 `airfoils`。
fn cluster_uses_rocket_aero(vessels: &[Vessel], components: &[SubVesselData]) -> bool {
    let mut has_rocket = false;
    let mut has_airfoil = false;
    for c in components {
        let v = &vessels[c.vessel_index];
        if v.rocket_body.is_some() || !v.lifting_surfaces.is_empty() {
            has_rocket = true;
        }
        if !v.airfoils.is_empty() {
            has_airfoil = true;
        }
    }
    has_rocket && !has_airfoil
}

/// 烘焙刚体筒体系数：取簇内带 `rocket_body` 且干重最大者；**禁止读 `active`**。
fn bake_cluster_rocket_body(
    vessels: &[Vessel],
    components: &[SubVesselData],
) -> RocketBodyAero {
    let mut best: Option<(f64, RocketBodyAero)> = None;
    for c in components {
        let v = &vessels[c.vessel_index];
        let Some(b) = v.rocket_body.as_ref() else {
            continue;
        };
        let m = v.dry_mass;
        if best.as_ref().map(|(bm, _)| m > *bm).unwrap_or(true) {
            best = Some((m, b.clone()));
        }
    }
    best.map(|(_, b)| b).unwrap_or_default()
}

/// 将各级升力面变到簇体坐标（冻结副本）。
fn collect_cluster_surfaces(
    vessels: &[Vessel],
    components: &[SubVesselData],
) -> Vec<LiftingSurface> {
    let mut out = Vec::new();
    for c in components {
        let v = &vessels[c.vessel_index];
        for s in &v.lifting_surfaces {
            let mut surf = s.surf.clone();
            surf.ref_pos = c.rpos + mul(c.rrot, surf.ref_pos);
            surf.normal = mul(c.rrot, surf.normal);
            surf.chord_dir = mul(c.rrot, surf.chord_dir);
            out.push(surf);
        }
    }
    out
}

#[cfg(test)]
mod tests;
