//! Assembly：多 Vessel 组合体管理，处理连接、分离和统一积分。

use crate::stage::StageSpec;
use crate::vessel::Vessel;
use orbitx_dynamics::gacc_nbody;
use orbitx_dynamics::GravBody;
use orbitx_math::{mul, Quat, StateVectors, Vec3};

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
    /// stages[0] = 底层级（第一级），最后一级 = 有效载荷。
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

    /// 油门设置（设置所有未分离级的推进器）。
    pub fn set_throttle(&mut self, level: f64) {
        for v in &mut self.vessels {
            if !v.detached {
                v.set_throttle(level);
            }
        }
    }

    /// 当前总推力 [N]（所有未分离级）。
    pub fn current_thrust(&self) -> f64 {
        self.vessels
            .iter()
            .filter(|v| !v.detached)
            .map(|v| v.current_thrust())
            .sum()
    }

    /// 一步物理积分。
    ///
    /// 所有未分离级共享同一运动状态。推力来自所有未分离级的推进器。
    pub fn step(&mut self, dt: f64, grav_bodies: &[GravBody]) {
        let total_mass = self.total_mass();
        if total_mass < 1e-3 {
            return;
        }

        let state = self.vessels[self.active].state;
        let rot = state.r; // 旋转矩阵快照

        // 收集所有未分离级的推力信息。
        let thrust_infos: Vec<(Vec3, f64)> = self
            .vessels
            .iter()
            .filter(|v| !v.detached)
            .flat_map(|v| {
                v.thrusters
                    .iter()
                    .filter(|t| t.level > 0.0 && v.fuel_mass > 0.0)
                    .map(move |t| (t.dir, t.current_thrust()))
            })
            .collect();

        // 燃料消耗信息。
        let flow_rates: Vec<(usize, f64)> = self
            .vessels
            .iter()
            .enumerate()
            .filter(|(_, v)| !v.detached)
            .flat_map(|(vi, v)| {
                v.thrusters
                    .iter()
                    .filter(|t| t.level > 0.0 && v.fuel_mass > 0.0)
                    .map(move |t| (vi, t.mass_flow_rate()))
            })
            .collect();

        let n_sub = 4;
        let sub_dt = dt / n_sub as f64;

        let mut current_state = state;
        for _ in 0..n_sub {
            let snap_rot = current_state.r;
            let ti = thrust_infos.clone();
            let gb = grav_bodies.to_vec();

            let mut force = move |s: &StateVectors, _t: f64| {
                // 重力。
                let g_acc = gacc_nbody(s.pos, &gb, None);

                // 推力（体坐标→世界坐标）。
                let mut thrust_acc = Vec3::ZERO;
                for (dir, thrust) in &ti {
                    let world_dir = mul(snap_rot, *dir);
                    thrust_acc += world_dir * (*thrust / total_mass);
                }

                (g_acc + thrust_acc, Vec3::ZERO)
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
