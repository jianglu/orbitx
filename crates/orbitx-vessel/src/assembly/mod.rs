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

use std::collections::{HashMap, HashSet, VecDeque};
use std::sync::Arc;

use crate::aero::{compute_aero_forces, world_to_airvel_ship, Atmosphere};
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
    /// 大气模型（`None` 则不计算气动力）。
    pub atmosphere: Option<Box<dyn Atmosphere>>,
    /// 中心天体半径 [m]（用于计算高度 → 大气密度）。
    pub planet_radius: f64,
}

impl Assembly {
    /// 从级定义列表创建多级火箭，并对相邻顶/底口硬对接。
    ///
    /// stages[0] = 底层级（第一级），最后一位 = 有效载荷。
    pub fn new(stages: &[StageSpec], initial_state: StateVectors) -> Self {
        let n = stages.len();
        let links: Vec<(usize, usize, usize, usize)> = (0..n.saturating_sub(1))
            .map(|i| (i, 1, i + 1, 0))
            .collect();
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
        let state = vessels
            .get(active)
            .map(|v| v.state)
            .unwrap_or_default();
        let mut asm = Assembly {
            vessels,
            active,
            root: active,
            components: Vec::new(),
            state,
            atmosphere: None,
            planet_radius: 0.0,
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
        if port_a >= self.vessels[idx_a].docks.len()
            || port_b >= self.vessels[idx_b].docks.len()
        {
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
            let (rp, _) = self.component_pose(i).unwrap_or((rp_idx, Matrix3::IDENTITY));
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
            .map(|c| self.vessels[c.vessel_index].fuel_mass)
            .sum()
    }

    /// 燃料百分比（0..100），基于所有级的燃料总量。
    pub fn fuel_percent(&self) -> f64 {
        let current: f64 = self.vessels.iter().map(|v| v.fuel_mass).sum();
        let max: f64 = self.vessels.iter().map(|v| v.fuel_mass).sum();
        // 用初始无法获知；沿用旧语义：当前相对「现存 vessel 当前燃料之和」无意义。
        // 保持与旧实现一致：current/max where max = sum of current fuel_mass fields
        // 旧代码 max = sum fuel_mass（当前值），故始终 ~100%。仍保留行为。
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

    /// 当前推力 [N]（仅活动级）。
    pub fn current_thrust(&self) -> f64 {
        self.vessels[self.active].current_thrust()
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

    /// 一步物理积分（主组合体 + 已分离独立体）。
    pub fn step(&mut self, dt: f64, grav_bodies: &[GravBody]) {
        self.step_primary(dt, grav_bodies);
        self.step_detached(dt, grav_bodies);
    }

    fn step_primary(&mut self, dt: f64, grav_bodies: &[GravBody]) {
        let total_mass = self.total_mass();
        if total_mass < 1e-3 || self.components.is_empty() {
            return;
        }

        let cg = self.primary_cg();
        let composite_pmi = self.composite_pmi();

        // 每子船：体坐标推力与力矩
        let mut thrust_by_comp: Vec<(usize, Vec3, Vec3)> = Vec::new(); // (comp_idx, F, M)
        let mut flow_rates: Vec<(usize, f64)> = Vec::new();
        for (ci, c) in self.components.iter().enumerate() {
            let v = &self.vessels[c.vessel_index];
            let has_fuel = v.fuel_mass > 0.0 || v.tanks_total_mass() > 0.0;
            let mut f = Vec3::ZERO;
            let mut m = Vec3::ZERO;
            for t in &v.thrusters {
                if t.level > 0.0 && has_fuel {
                    let thrust = t.current_thrust();
                    let dir = t.current_dir();
                    let fb = dir * thrust;
                    f += fb;
                    m += cross(fb, t.pos);
                    flow_rates.push((c.vessel_index, t.mass_flow_rate()));
                }
            }
            if f.length() > 0.0 || m.length() > 0.0 {
                thrust_by_comp.push((ci, f, m));
            }
        }

        let active_vi = self.active;
        let aero_airfoils = self.vessels[active_vi].airfoils.clone();
        let aero_ctrlsurfs = self.vessels[active_vi].ctrlsurfs.clone();
        let aero_dragels = self.vessels[active_vi].dragels.clone();
        let aero_cs = self.vessels[active_vi].cross_section;
        let aero_rdrag = self.vessels[active_vi].rdrag;

        let planet_radius = self.planet_radius;
        let rho_fn: Option<Arc<dyn Fn(f64) -> f64 + Send + Sync>> =
            self.atmosphere.as_ref().map(|atm| atm.density_fn());

        let cbody = grav_bodies.first();
        let cbody_mass = cbody.map(|b| b.mass).unwrap_or(0.0);
        let cbody_pos = cbody.map(|b| b.pos).unwrap_or(Vec3::ZERO);

        let comps = self.components.clone();
        let n_sub = 4;
        let sub_dt = dt / n_sub as f64;
        let mut current_state = self.state;

        for _ in 0..n_sub {
            let snap_rot = current_state.r;
            let ti = thrust_by_comp.clone();
            let gb = grav_bodies.to_vec();
            let pmi = composite_pmi;
            let comps_c = comps.clone();
            let cg_c = cg;
            let af = aero_airfoils.clone();
            let cs = aero_ctrlsurfs.clone();
            let de = aero_dragels.clone();
            let rho_fn_clone = rho_fn.clone();

            let mut force = move |s: &StateVectors, _t: f64| {
                let g_acc = gacc_nbody(s.pos, &gb, None);

                let mut f_sv = Vec3::ZERO;
                let mut m_sv = Vec3::ZERO;
                for &(ci, f_comp, m_comp) in &ti {
                    add_component_force_and_moment(
                        &mut f_sv,
                        &mut m_sv,
                        f_comp,
                        m_comp,
                        &comps_c[ci],
                        cg_c,
                    );
                }

                let mut thrust_acc = mul(snap_rot, f_sv) / total_mass;
                let torque = m_sv;

                let mut aero_torque_body = Vec3::ZERO;
                if let Some(rho_fn) = &rho_fn_clone {
                    let alt = s.pos.length() - planet_radius;
                    let rho = rho_fn(alt);
                    if rho > 1e-15 {
                        let airvel_ship = world_to_airvel_ship(s.vel, Vec3::ZERO, snap_rot);
                        let aero = compute_aero_forces(
                            &af,
                            &cs,
                            &de,
                            airvel_ship,
                            rho,
                            s.omega,
                            pmi,
                            total_mass,
                            aero_cs,
                            aero_rdrag,
                            sub_dt,
                        );
                        thrust_acc += mul(snap_rot, aero.force) / total_mass;
                        aero_torque_body = aero.torque;
                    }
                }

                let gg_torque = if cbody_mass > 0.0 {
                    gravity_gradient_torque(
                        cbody_pos - s.pos,
                        cbody_mass,
                        pmi,
                        snap_rot,
                        s.omega,
                        0.0,
                        sub_dt,
                        false,
                    )
                } else {
                    Vec3::ZERO
                };

                let tau = (torque + aero_torque_body) / total_mass + gg_torque;
                let arot = euler_inv_full(tau, s.omega, pmi);
                (g_acc + thrust_acc, arot)
            };

            current_state = orbitx_dynamics::rk4_step(current_state, sub_dt, &mut force);
        }

        self.state = current_state;
        self.writeback_primary_states();

        // 燃料
        for (vi, _) in &flow_rates {
            let v = &self.vessels[*vi];
            let has_tanks = !v.tanks.is_empty();
            let consumes: Vec<(Option<u32>, f64)> = v
                .thrusters
                .iter()
                .filter(|t| t.level > 0.0)
                .map(|t| (t.tank_id, t.mass_flow_rate() * dt))
                .collect();
            let v = &mut self.vessels[*vi];
            if has_tanks {
                for (tank_id, t_consume) in consumes {
                    if let Some(tank_id) = tank_id {
                        v.consume_fuel_from_tank(tank_id, t_consume);
                    } else {
                        v.consume_fuel(t_consume);
                    }
                }
            } else {
                let total: f64 = consumes.iter().map(|(_, c)| *c).sum();
                v.consume_fuel(total);
            }
        }

        // 质量变化后刷新 CG 对应的 state.pos（保持 root 姿态）
        self.rebuild_primary_from_active();
        let _ = self;
    }

    fn step_detached(&mut self, dt: f64, grav_bodies: &[GravBody]) {
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
            let mass = self.vessels[vi].mass();
            if mass < 1e-3 {
                continue;
            }
            let pmi = self.vessels[vi].pmi;
            let mut thrust_f = Vec3::ZERO;
            let mut thrust_m = Vec3::ZERO;
            let has_fuel =
                self.vessels[vi].fuel_mass > 0.0 || self.vessels[vi].tanks_total_mass() > 0.0;
            for t in &self.vessels[vi].thrusters {
                if t.level > 0.0 && has_fuel {
                    let thr = t.current_thrust();
                    let dir = t.current_dir();
                    let fb = dir * thr;
                    thrust_f += fb;
                    thrust_m += cross(fb, t.pos);
                }
            }

            let n_sub = 4;
            let sub_dt = dt / n_sub as f64;
            let mut st = self.vessels[vi].state;
            for _ in 0..n_sub {
                let snap_rot = st.r;
                let gb = grav_bodies.to_vec();
                let mut force = move |s: &StateVectors, _t: f64| {
                    let g_acc = gacc_nbody(s.pos, &gb, None);
                    let thrust_acc = mul(snap_rot, thrust_f) / mass;
                    let tau = thrust_m / mass;
                    let arot = euler_inv_full(tau, s.omega, pmi);
                    (g_acc + thrust_acc, arot)
                };
                st = orbitx_dynamics::rk4_step(st, sub_dt, &mut force);
            }
            self.vessels[vi].state = st;
        }
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

    fn rebuild_primary_from_active(&mut self) {
        if self.vessels.is_empty() {
            self.components.clear();
            return;
        }
        if self.vessels[self.active].detached {
            self.active = self
                .vessels
                .iter()
                .position(|v| !v.detached)
                .unwrap_or(0);
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

#[cfg(test)]
mod tests;
