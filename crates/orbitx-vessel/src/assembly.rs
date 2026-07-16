//! Assembly：多 Vessel 组合体管理，处理连接、分离和统一积分。
//!
//! 刚体姿态动力学（移植自 Orbiter）：
//! - 推力力矩：每个推进器在体坐标系产生 `τ = F × r`（`Vessel.cpp:4024`，
//!   `Vessel.h:1316-1320` AddForce）。
//! - 重力梯度力矩：`gravity_gradient_torque`（`Rigidbody.cpp:345-363`）。
//! - 组合体 PMI 合成：[`composite_pmi`]（`SuperVessel::CalcPMI`，
//!   `SuperVessel.cpp:1058-1104`）。
//! - Euler 方程：`euler_inv_full`（`Rigidbody.cpp:468-481`），输入是质量
//!   归一化的力矩（`Vessel.cpp:921` `tau += M/mass`）。

use crate::stage::StageSpec;
use crate::vessel::Vessel;
use orbitx_dynamics::gacc_nbody;
use orbitx_dynamics::gravity_gradient_torque;
use orbitx_dynamics::euler_inv_full;
use orbitx_dynamics::GravBody;
use orbitx_math::{cross, mul, tmul, Quat, StateVectors, Vec3};

/// 管理多个 Vessel 的组合体。
///
/// 级从底到顶排列：vessels[0] = 第一级（底），最后 = 有效载荷（顶）。
/// `active` 指向当前主控级（分离后自动切换到下一级）。
pub struct Assembly {
    /// 所有 Vessel。
    pub vessels: Vec<Vessel>,
    /// 当前活动级的索引。
    pub active: usize,
}

impl Assembly {
    /// 从级定义列表创建多级火箭。
    ///
    /// stages[0] = 底层级（第一级），最后一位 = 有效载荷。
    /// 所有级共享同一个初始运动状态，按级长度堆叠。
    pub fn new(stages: &[StageSpec], initial_state: StateVectors) -> Self {
        let mut vessels = Vec::with_capacity(stages.len());
        // 计算各级在体坐标系中的 Y 偏移（从底到顶堆叠）。
        let mut y_offset = 0.0_f64;
        for (id_counter, spec) in stages.iter().rev().enumerate() {
            let mut state = initial_state;
            let radial = initial_state.pos * (1.0 / initial_state.pos.length().max(1e-3));
            state.pos = initial_state.pos + radial * y_offset;
            y_offset += spec.length;

            let vessel = Vessel::from_spec(id_counter as u64, spec, state);

            // 连接对接端口：每级的顶端口连接上一级的底端口。
            // 连接关系在所有级创建后统一设置。
            vessels.push(vessel);
        }

        // 反转使 stages[0] = vessels[0]（底层级）。
        vessels.reverse();

        let mut asm = Assembly { vessels, active: 0 };
        asm.connect_docks();
        asm
    }

    /// 连接所有相邻级的对接端口。
    fn connect_docks(&mut self) {
        let n = self.vessels.len();
        for i in 0..n.saturating_sub(1) {
            let id_bottom = self.vessels[i].id;
            let id_top = self.vessels[i + 1].id;
            // 底层级的顶端口(1) → 上层级底端口(0)。
            if self.vessels[i].docks.len() > 1 && !self.vessels[i + 1].docks.is_empty() {
                self.vessels[i].docks[1].connected_to = Some((id_top, 0));
                self.vessels[i + 1].docks[0].connected_to = Some((id_bottom, 1));
            }
        }
        let _ = n; // suppress unused warning
    }

    /// 当前总质量（所有未分离级的干质量+燃料）。
    pub fn total_mass(&self) -> f64 {
        self.vessels
            .iter()
            .filter(|v| !v.detached)
            .map(|v| v.mass())
            .sum()
    }

    /// 当前燃料总量 [kg]。
    pub fn total_fuel(&self) -> f64 {
        self.vessels
            .iter()
            .filter(|v| !v.detached)
            .map(|v| v.fuel_mass)
            .sum()
    }

    /// 燃料百分比（0..100）。
    /// 燃料百分比（0..100），基于所有级的燃料总量。
    pub fn fuel_percent(&self) -> f64 {
        let current = self.total_fuel();
        let max: f64 = self.vessels.iter().map(|v| v.fuel_mass).sum();
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

    /// 每级相对组合体质心的体坐标系 Y 偏移与质量。
    ///
    /// 约定：组合体坐标系原点在底层级（vessels[0]）的几何中心，+Y 朝顶。
    /// 各级中心 y 坐标 = 下方所有级长度之和 + 自身长度/2。
    fn stage_layout(&self) -> Vec<(f64, f64)> {
        // vessels[0] 是底层级。按从底到顶累加长度。
        let mut y = 0.0;
        self.vessels
            .iter()
            .filter(|v| !v.detached)
            .map(|v| {
                let center = y + v.length * 0.5;
                y += v.length;
                (center, v.mass())
            })
            .collect()
    }

    /// 组合体质心（体坐标系 Y 坐标，原点在底层级中心）。
    fn center_of_mass(&self) -> f64 {
        let layout = self.stage_layout();
        let total_mass: f64 = layout.iter().map(|(_, m)| m).sum();
        if total_mass < 1e-3 {
            return 0.0;
        }
        layout
            .iter()
            .map(|(y, m)| y * m)
            .sum::<f64>()
            / total_mass
    }

    /// 合成组合体主惯量张量（对角线），移植自 `SuperVessel::CalcPMI`
    /// （`SuperVessel.cpp:1058-1104`）。
    ///
    /// 算法：把每级的 PMI 分解为 6 个虚拟质点（±各主轴方向，距质心
    /// `sqrt(1.5·|...|)`），用平行轴定理平移到组合体质心后求和。
    /// orbitx 的各级同轴对齐（`rrot = identity`），故只做平移。
    pub fn composite_pmi(&self) -> Vec3 {
        let layout = self.stage_layout();
        let total_mass: f64 = layout.iter().map(|(_, m)| m).sum();
        if total_mass < 1e-3 {
            return Vec3::new(1.0, 1.0, 1.0);
        }
        let cg = self.center_of_mass();

        // 收集未分离级（与 layout 顺序一致）。
        let attached: Vec<&Vessel> = self.vessels.iter().filter(|v| !v.detached).collect();

        let mut pmi = Vec3::ZERO;
        for (k, v) in attached.iter().enumerate() {
            let &(y_center, _) = layout.get(k).unwrap_or(&(0.0, 0.0));
            let vpmi = v.pmi;
            let vmass = v.mass() / 6.0;
            let dy = y_center - cg; // 该级质心相对组合体质心的 Y 偏移

            // 从各级 PMI 反解 6 个虚拟质点偏移（SuperVessel.cpp:1075-1077）。
            let r0x = (1.5 * (-vpmi.x + vpmi.y + vpmi.z).abs()).sqrt();
            let r0y = (1.5 * (vpmi.x - vpmi.y + vpmi.z).abs()).sqrt();
            let r0z = (1.5 * (vpmi.x + vpmi.y - vpmi.z).abs()).sqrt();
            // 6 个点：±x, ±y, ±z。y 分量加上 dy（平移到组合体质心）。
            let pts = [
                Vec3::new(r0x, dy, 0.0),
                Vec3::new(-r0x, dy, 0.0),
                Vec3::new(0.0, r0y + dy, 0.0),
                Vec3::new(0.0, -r0y + dy, 0.0),
                Vec3::new(0.0, dy, r0z),
                Vec3::new(0.0, dy, -r0z),
            ];
            let mut vpmix = 0.0;
            let mut vpmiy = 0.0;
            let mut vpmiz = 0.0;
            for rt in &pts {
                vpmix += rt.y * rt.y + rt.z * rt.z;
                vpmiy += rt.x * rt.x + rt.z * rt.z;
                vpmiz += rt.x * rt.x + rt.y * rt.y;
            }
            pmi.x += vmass * vpmix;
            pmi.y += vmass * vpmiy;
            pmi.z += vmass * vpmiz;
        }
        // 归一化（SuperVessel.cpp:1092-1094）。
        Vec3::new(pmi.x / total_mass, pmi.y / total_mass, pmi.z / total_mass)
    }

    /// 一步物理积分（含刚体姿态动力学）。
    ///
    /// 所有未分离级共享同一运动状态。推力来自活动级的推进器，其力矩
    /// `τ = F × engine_pos` 驱动 Euler 方程。重力梯度力矩可选启用。
    pub fn step(&mut self, dt: f64, grav_bodies: &[GravBody]) {
        let total_mass = self.total_mass();
        if total_mass < 1e-3 {
            return;
        }

        let state = self.vessels[self.active].state;
        let rot = state.r; // 旋转矩阵快照（body→world）
        let composite_pmi = self.composite_pmi();

        // 收集活动级推进器的推力信息（体坐标系方向 + 推力大小 + 作用点）。
        // 仅活动级消耗燃料。
        let active = &self.vessels[self.active];
        let thrust_infos: Vec<(Vec3, f64, Vec3)> = active
            .thrusters
            .iter()
            .filter(|t| t.level > 0.0 && active.fuel_mass > 0.0)
            .map(|t| (t.current_dir(), t.current_thrust(), t.pos))
            .collect();

        let flow_rates: Vec<(usize, f64)> = active
            .thrusters
            .iter()
            .filter(|t| t.level > 0.0 && active.fuel_mass > 0.0)
            .map(|t| (self.active, t.mass_flow_rate()))
            .collect();

        // 中心天体（用于重力梯度力矩）：取第一个引力源。
        let cbody = grav_bodies.first();
        let cbody_mass = cbody.map(|b| b.mass).unwrap_or(0.0);
        let cbody_pos = cbody.map(|b| b.pos).unwrap_or(Vec3::ZERO);

        let n_sub = 4;
        let sub_dt = dt / n_sub as f64;

        let mut current_state = state;
        for _ in 0..n_sub {
            let snap_rot = current_state.r;
            let ti = thrust_infos.clone();
            let gb = grav_bodies.to_vec();
            let pmi = composite_pmi;
            let cb_mass = cbody_mass;
            let cb_pos = cbody_pos;

            let mut force = move |s: &StateVectors, _t: f64| {
                // 重力加速度（线性）。
                let g_acc = gacc_nbody(s.pos, &gb, None);

                // 推力加速度（线性）+ 推力力矩累积。
                // 推力方向在体坐标系，经 snap_rot 转世界系。
                let mut thrust_acc = Vec3::ZERO;
                let mut torque = Vec3::ZERO; // 体坐标系力矩 [N·m]
                for (dir_body, thrust, pos_body) in &ti {
                    let world_dir = mul(snap_rot, *dir_body);
                    thrust_acc += world_dir * (*thrust / total_mass);
                    // τ = F × r，力和位置都在体坐标系（Vessel.cpp:4024）。
                    let f_body = *dir_body * (*thrust);
                    torque += cross(f_body, *pos_body);
                }

                // 重力梯度力矩（质量归一化，体坐标系）。
                // bIgnore 当轨道步长过大时为真——这里保守地始终启用。
                let gg_torque = if cb_mass > 0.0 {
                    gravity_gradient_torque(
                        cb_pos - s.pos,
                        cb_mass,
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

                // 质量归一化力矩（Vessel.cpp:921: tau += M/mass）。
                let tau = torque / total_mass + gg_torque;

                // 解 Euler 方程得角加速度（Rigidbody.cpp:260）。
                let arot = euler_inv_full(tau, s.omega, pmi);

                (g_acc + thrust_acc, arot)
            };

            current_state = orbitx_dynamics::rk4_step(current_state, sub_dt, &mut force);
        }

        // 同步所有未分离级的状态。
        for v in &mut self.vessels {
            if !v.detached {
                v.state = current_state;
            }
        }

        // 消耗燃料。
        let flow_per_substep = dt; // 总时间内的消耗
        for (vi, rate) in &flow_rates {
            let consume = rate * flow_per_substep;
            self.vessels[*vi].consume_fuel(consume);
        }

        let _ = rot;
        let _ = tmul; // (保留导入以便后续体↔世界变换扩展)
    }

    /// 分离最底层的未分离级。
    ///
    /// 1. 施加分离脉冲（被分离级减速）
    /// 2. 标记为 detached
    /// 3. 解除对接连接
    /// 4. 切换 active 到下一个未分离级
    ///
    /// 返回新的 active 索引。
    pub fn separate_stage(&mut self) -> usize {
        // 找到最底层的未分离级。
        let bottom = self
            .vessels
            .iter()
            .position(|v| !v.detached)
            .unwrap_or(self.active);

        let attached_count = self.vessels.iter().filter(|v| !v.detached).count();
        if attached_count <= 1 {
            return self.active;
        }

        let bottom_id = self.vessels[bottom].id;
        let sep_impulse = self.vessels[bottom].separation_impulse;
        let active_state = self.vessels[self.active].state;

        // 施加分离脉冲：被分离级沿其轴向减速。
        // 轴向 = 体坐标 -Y 方向（火箭尾部方向），在 world 坐标中。
        let axis_world = mul(active_state.r, Vec3::new(0.0, -1.0, 0.0));
        self.vessels[bottom].state.vel -= axis_world * sep_impulse;
        self.vessels[bottom].detached = true;

        // 解除对接连接。
        for v in &mut self.vessels {
            for d in &mut v.docks {
                if let Some((tid, _)) = d.connected_to {
                    if tid == bottom_id {
                        d.connected_to = None;
                    }
                }
            }
        }

        // 切换 active 到下一个未分离级。
        self.active = self
            .vessels
            .iter()
            .position(|v| !v.detached)
            .unwrap_or(self.active);

        self.active
    }

    /// 获取主控级的渲染信息（位置、姿态四元数）。
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
}
