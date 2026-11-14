//! 图调度：点火、ACTIVE 子树、条件、延时、驱动。

use std::collections::{HashMap, HashSet};

use orbitx_protocol::mem::Slice;
use orbitx_vessel::{Assembly, RotAxis};

use crate::base::{BaseController, Controller};
use crate::capability::ControlCapability;
use crate::target::TargetController;
use crate::workflow::graph::{
    stage_ports_from_assembly, ActionDrive, Combine, DriveSpec, GraphIr, MainState, NodeBody,
    PortDomain,
};
use crate::workflow::pred::{eval_term, PredTerm};
use crate::workflow::{HaltAction, SchemaError, WorkFlow};

pub struct GraphWorkFlow {
    ir: GraphIr,
    caps: ControlCapability,
    /// 工厂仍传入。条件改读切片。
    #[allow(dead_code)]
    mu: f64,
    main_state: HashMap<String, MainState>,
    ignite_spent: bool,
    ignite_id: String,
    delay_acc: HashMap<String, f64>,
    delay_fired: HashMap<String, u32>,
    once_spent: HashSet<String>,
    cond_time: HashMap<String, f64>,
    fires: HashMap<String, u32>,
    target: TargetController,
    outs: HashMap<(String, String), Vec<String>>,
    #[allow(dead_code)]
    domain: HashMap<String, PortDomain>,
    index: HashMap<String, usize>,
    /// Halt 首次激活锁存的停机动作（`None` = 未锁存）。
    halt_latch: Option<HaltAction>,
}

impl GraphWorkFlow {
    pub fn try_new(ir: GraphIr, asm: &Assembly, mu: f64) -> Result<Self, SchemaError> {
        let ports = stage_ports_from_assembly(asm, ir.kind);
        crate::workflow::graph::validate(&ir, Some(&ports))?;
        Ok(Self::from_validated(ir, asm, mu))
    }

    fn from_validated(ir: GraphIr, asm: &Assembly, mu: f64) -> Self {
        let caps = ControlCapability::for_primary(asm);
        let mut main_state = HashMap::new();
        let mut ignite_id = String::new();
        let mut index = HashMap::new();
        for (i, n) in ir.nodes.iter().enumerate() {
            index.insert(n.id.clone(), i);
            match &n.body {
                NodeBody::Main { .. } => {
                    main_state.insert(n.id.clone(), MainState::Idle);
                }
                NodeBody::Ignite => ignite_id = n.id.clone(),
                _ => {}
            }
        }
        let mut outs: HashMap<(String, String), Vec<String>> = HashMap::new();
        let mut domain = HashMap::new();
        for e in &ir.edges {
            let port = if e.from_port.is_empty() {
                "out".to_string()
            } else {
                e.from_port.clone()
            };
            outs.entry((e.from.clone(), port.clone()))
                .or_default()
                .push(e.to.clone());
        }
        // 域：从主节点口 DFS（与校验一致，此处已通过校验）。
        let main_ids: Vec<String> = main_state.keys().cloned().collect();
        for mid in main_ids {
            for (pname, dom) in [
                ("flight", PortDomain::Flight),
                ("sep", PortDomain::Sep),
                ("function", PortDomain::Function),
            ] {
                if let Some(children) = outs.get(&(mid.clone(), pname.to_string())) {
                    for c in children.clone() {
                        stamp_domain(&ir, &outs, &mut domain, &c, dom);
                    }
                }
            }
        }
        Self {
            ir,
            caps,
            mu,
            main_state,
            ignite_spent: false,
            ignite_id,
            delay_acc: HashMap::new(),
            delay_fired: HashMap::new(),
            once_spent: HashSet::new(),
            cond_time: HashMap::new(),
            fires: HashMap::new(),
            target: TargetController::new(crate::target::TargetMode::VerticalHold {
                throttle: 0.0,
            }),
            outs,
            domain,
            index,
            halt_latch: None,
        }
    }

    pub fn main_state(&self, id: &str) -> Option<MainState> {
        self.main_state.get(id).copied()
    }

    pub fn fires(&self, id: &str) -> u32 {
        self.fires.get(id).copied().unwrap_or(0)
    }

    pub fn ignite_spent(&self) -> bool {
        self.ignite_spent
    }

    /// 测试用：直接改主节点状态。进入 ACTIVE 时重置子树。
    pub fn force_state(&mut self, id: &str, state: MainState) {
        if state == MainState::Active {
            self.set_active(id);
        } else {
            self.main_state.insert(id.to_string(), state);
        }
    }

    fn node(&self, id: &str) -> &crate::workflow::graph::GraphNode {
        &self.ir.nodes[self.index[id]]
    }

    /// 条件所属主节点的级名。对不上主节点时为空，级燃料条件为假。
    fn owner_stage(&self, owner: &str) -> String {
        let Some(&idx) = self.index.get(owner) else {
            return String::new();
        };
        match &self.ir.nodes[idx].body {
            NodeBody::Main { stage } => stage.clone(),
            _ => String::new(),
        }
    }

    fn set_active(&mut self, id: &str) {
        let prev = self.main_state.get(id).copied();
        if prev == Some(MainState::Active) {
            return;
        }
        self.main_state.insert(id.to_string(), MainState::Active);
        self.reset_subtree(id);
    }

    fn reset_subtree(&mut self, main_id: &str) {
        let mut stack = Vec::new();
        for port in ["flight", "sep", "function"] {
            if let Some(ch) = self.outs.get(&(main_id.to_string(), port.to_string())) {
                stack.extend(ch.clone());
            }
        }
        let mut seen = HashSet::new();
        while let Some(id) = stack.pop() {
            if !seen.insert(id.clone()) {
                continue;
            }
            self.delay_acc.remove(&id);
            self.delay_fired.remove(&id);
            self.once_spent.remove(&id);
            self.cond_time.remove(&id);
            let body = self.node(&id).body.clone();
            match body {
                NodeBody::Condition { .. } => {
                    for p in ["true", "false"] {
                        if let Some(ch) = self.outs.get(&(id.clone(), p.to_string())) {
                            stack.extend(ch.clone());
                        }
                    }
                }
                NodeBody::Delay { .. } => {
                    if let Some(ch) = self.outs.get(&(id.clone(), "out".to_string())) {
                        stack.extend(ch.clone());
                    }
                }
                _ => {}
            }
        }
    }

    fn children(&self, id: &str, port: &str) -> Vec<String> {
        self.outs
            .get(&(id.to_string(), port.to_string()))
            .cloned()
            .unwrap_or_default()
    }

    /// Halt `shutdown_controls` 清零断言（幂等）：循环节流 / RCS / TVC 各组置零 +
    /// 内部 TargetController 重置。`BaseController` 命令写入 vessel 后持续生效、
    /// 无自动衰减，故锁存后每拍重复断言（WORKFLOW.md §5.5）。
    fn apply_shutdown_controls(&mut self, asm: &mut Assembly, dt: f64) {
        let throttle_ids: Vec<String> = self
            .caps
            .throttle_groups
            .iter()
            .map(|g| g.id.clone())
            .collect();
        let rcs_ids: Vec<String> = self.caps.rcs_groups.iter().map(|g| g.id.clone()).collect();
        let tvc_ids: Vec<String> = self.caps.tvc_groups.iter().map(|g| g.id.clone()).collect();
        for id in &throttle_ids {
            let mut base = BaseController::new(asm, &self.caps);
            base.set_group_throttle(id, 0.0);
        }
        for id in &rcs_ids {
            for axis in [RotAxis::Pitch, RotAxis::Yaw, RotAxis::Bank] {
                let mut base = BaseController::new(asm, &self.caps);
                base.set_rcs(id, axis, 0.0);
            }
        }
        for id in &tvc_ids {
            let mut base = BaseController::new(asm, &self.caps);
            base.apply_tvc(id, 0.0, 0.0, dt);
        }
        // 内部 TargetController 重置：不再产生新指令（HUD 面归零）。
        self.target =
            TargetController::new(crate::target::TargetMode::VerticalHold { throttle: 0.0 });
    }

    /// Reset 接线：复位 halt 锁存与全部图调度状态（回到未点火初始态，
    /// 新会话可正常步进；见 RUNTIME.md 停机收尾）。
    pub fn reset_state(&mut self) {
        for st in self.main_state.values_mut() {
            *st = MainState::Idle;
        }
        self.ignite_spent = false;
        self.delay_acc.clear();
        self.delay_fired.clear();
        self.once_spent.clear();
        self.cond_time.clear();
        self.fires.clear();
        self.halt_latch = None;
        self.target =
            TargetController::new(crate::target::TargetMode::VerticalHold { throttle: 0.0 });
    }
}

fn stamp_domain(
    ir: &GraphIr,
    outs: &HashMap<(String, String), Vec<String>>,
    domain: &mut HashMap<String, PortDomain>,
    id: &str,
    dom: PortDomain,
) {
    domain.insert(id.to_string(), dom);
    let body = ir.nodes.iter().find(|n| n.id == id).map(|n| n.body.clone());
    match body {
        Some(NodeBody::Condition { .. }) => {
            for p in ["true", "false"] {
                if let Some(ch) = outs.get(&(id.to_string(), p.to_string())) {
                    for c in ch {
                        stamp_domain(ir, outs, domain, c, dom);
                    }
                }
            }
        }
        Some(NodeBody::Delay { .. }) => {
            if let Some(ch) = outs.get(&(id.to_string(), "out".to_string())) {
                for c in ch {
                    stamp_domain(ir, outs, domain, c, dom);
                }
            }
        }
        _ => {}
    }
}

impl WorkFlow for GraphWorkFlow {
    fn ignite(&mut self) {
        if self.ignite_spent {
            return;
        }
        let Some(to) = self.children(&self.ignite_id, "out").into_iter().next() else {
            return;
        };
        self.ignite_spent = true;
        self.set_active(&to);
    }

    fn tick(&mut self, asm: &mut Assembly, slice: &Slice, dt: f64) {
        let active: Vec<String> = self
            .main_state
            .iter()
            .filter(|(_, s)| **s == MainState::Active)
            .map(|(id, _)| id.clone())
            .collect();
        for mid in active {
            for port in ["flight", "sep", "function"] {
                let children = self.children(&mid, port);
                for c in children {
                    self.visit(asm, slice, &c, true, dt, &mid);
                }
            }
        }
        // halt 锁存后每拍持续断言清零（幂等）——统一置于 visit 之后，覆盖
        // 同拍先行的 drive 写入；不依赖 halt 节点是否仍被 visit 到
        // （如挂在一次性 delay 之后）。
        if self.halt_latch.is_some_and(|h| h.shutdown_controls) {
            self.apply_shutdown_controls(asm, dt);
        }
    }

    fn is_done(&self) -> bool {
        false
    }

    fn halt_action(&self) -> Option<HaltAction> {
        self.halt_latch
    }

    fn reset_halt_state(&mut self) {
        self.reset_state();
    }

    fn throttle_cmd(&self) -> f64 {
        self.target.mode().throttle()
    }

    fn attitude_targets(&self) -> (f64, f64, f64) {
        crate::workflow::attitude_from_target(&self.target)
    }

    fn nose_target(&self) -> Option<[f64; 3]> {
        self.target.nose_cmd()
    }
}

impl GraphWorkFlow {
    fn visit(
        &mut self,
        asm: &mut Assembly,
        slice: &Slice,
        id: &str,
        enabled: bool,
        dt: f64,
        owner: &str,
    ) {
        if !enabled {
            return;
        }
        let body = self.node(id).body.clone();
        match body {
            NodeBody::Condition { combine, terms } => {
                let t = self.cond_time.entry(id.to_string()).or_insert(0.0);
                let time_before = *t;
                let stage = self.owner_stage(owner);
                let met = eval_terms(slice, &combine, &terms, time_before, &stage);
                *self.cond_time.get_mut(id).unwrap() += dt;
                let port = if met { "true" } else { "false" };
                let children = self.children(id, port);
                for c in children {
                    self.visit(asm, slice, &c, true, dt, owner);
                }
            }
            NodeBody::Delay { count, interval_s } => {
                let acc = self.delay_acc.entry(id.to_string()).or_insert(0.0);
                *acc += dt;
                let fired = self.delay_fired.get(id).copied().unwrap_or(0);
                let unlimited = count == 0;
                let quota = unlimited || fired < count;
                let due = if interval_s <= 0.0 {
                    true
                } else {
                    *self.delay_acc.get(id).unwrap() >= interval_s
                };
                if quota && due {
                    if !unlimited {
                        *self.delay_fired.entry(id.to_string()).or_insert(0) += 1;
                    }
                    if interval_s > 0.0 {
                        *self.delay_acc.get_mut(id).unwrap() = 0.0;
                    }
                    *self.fires.entry(id.to_string()).or_insert(0) += 1;
                    let children = self.children(id, "out");
                    for c in children {
                        self.visit(asm, slice, &c, true, dt, owner);
                    }
                }
            }
            NodeBody::Drive { spec } => {
                let sep = matches!(spec, DriveSpec::Action(ActionDrive::Separate { .. }));
                if sep && self.once_spent.contains(id) {
                    return;
                }
                self.exec_drive(asm, id, &spec, dt);
                *self.fires.entry(id.to_string()).or_insert(0) += 1;
                if sep {
                    self.once_spent.insert(id.to_string());
                    self.caps = ControlCapability::for_primary(asm);
                }
            }
            NodeBody::Activate { invalidate_current } => {
                if self.once_spent.contains(id) {
                    return;
                }
                let next = self.children(id, "next").into_iter().next();
                self.once_spent.insert(id.to_string());
                *self.fires.entry(id.to_string()).or_insert(0) += 1;
                if let Some(next) = next {
                    self.set_active(&next);
                }
                if invalidate_current {
                    self.main_state
                        .insert(owner.to_string(), MainState::Invalid);
                }
            }
            NodeBody::Halt {
                shutdown_controls,
                stop_stepping,
                save_recorder,
                ..
            } => {
                // 首次激活锁存三参数为本次停机动作（runtime 经 halt_action()
                // 分派收尾，见 RUNTIME.md）；控制清零由 tick 尾部每拍断言。
                if self.halt_latch.is_none() {
                    self.halt_latch = Some(HaltAction {
                        shutdown_controls,
                        stop_stepping,
                        save_recorder,
                    });
                    *self.fires.entry(id.to_string()).or_insert(0) += 1;
                }
            }
            NodeBody::Ignite | NodeBody::Main { .. } => {}
        }
    }

    fn exec_drive(&mut self, asm: &mut Assembly, id: &str, spec: &DriveSpec, dt: f64) {
        let _ = id;
        match spec {
            DriveSpec::Target(mode) => {
                self.target.set_mode(mode.to_mode());
                let caps = &self.caps;
                let target = &mut self.target;
                let mut base = BaseController::new(asm, caps);
                target.tick(&mut base, dt);
            }
            DriveSpec::Action(ActionDrive::Throttle { group, level }) => {
                let mut base = BaseController::new(asm, &self.caps);
                base.set_group_throttle(group, *level);
            }
            DriveSpec::Action(ActionDrive::Tvc { group, pitch, yaw }) => {
                let mut base = BaseController::new(asm, &self.caps);
                base.apply_tvc(group, *pitch, *yaw, dt);
            }
            DriveSpec::Action(ActionDrive::Rcs { group, axis, level }) => {
                let ax = match axis.as_str() {
                    "pitch" => RotAxis::Pitch,
                    "yaw" => RotAxis::Yaw,
                    "bank" => RotAxis::Bank,
                    _ => return,
                };
                let mut base = BaseController::new(asm, &self.caps);
                base.set_rcs(group, ax, *level);
            }
            DriveSpec::Action(ActionDrive::Separate { point }) => {
                let mut base = BaseController::new(asm, &self.caps);
                let _ = base.separate(point);
            }
        }
    }
}

fn eval_terms(
    slice: &Slice,
    combine: &Combine,
    terms: &[PredTerm],
    time: f64,
    stage: &str,
) -> bool {
    if terms.is_empty() {
        return false;
    }
    let eval = |t: &PredTerm| eval_term(t, slice, time, stage);
    match combine {
        Combine::And => terms.iter().all(eval),
        Combine::Or => terms.iter().any(eval),
    }
}
