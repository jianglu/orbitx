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
use crate::workflow::{SchemaError, WorkFlow};

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
    }

    fn is_done(&self) -> bool {
        false
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
