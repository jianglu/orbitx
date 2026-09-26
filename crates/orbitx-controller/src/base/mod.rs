//! `BaseController`——vessel 唯一全功能门面（模式 a）。
//!
//! 唯一直接操作 `Assembly` 的单元：**既读遥测也写执行器**。`TargetController` / `WorkFlow`
//! 都不直接碰 `Assembly`，必经 `BaseController`。持有 `&mut Assembly + &caps`，读方法取
//! `&self`、写方法取 `&mut self`，分离的方法调用无借用冲突。
//!
//! `Controller` trait 为叶控制器（`TargetController` 等）统一 tick 入口，tick 取
//! `&mut BaseController`。

use crate::capability::{BodyRef, ControlCapability};
use crate::rcs as rcs_mod;
use crate::separation as sep_mod;
use crate::throttle::{self, ThrottlePolicy};
use crate::tvc as tvc_mod;
use orbitx_math::Vec3;
use orbitx_vessel::{attitude as att, Assembly, RotAxis};

/// 叶控制器统一 tick 入口（`TargetController` 等）。
///
/// `BaseController` 不 impl `Controller`（模式 a 由外部直接调方法）。
pub trait Controller: Send {
    /// 读 pre-step 状态、算本步命令、经 `base` 写执行器。
    fn tick(&mut self, base: &mut BaseController, dt: f64);
}

/// vessel 唯一全功能门面：读遥测 + 写执行器。
pub struct BaseController<'a> {
    asm: &'a mut Assembly,
    caps: &'a ControlCapability,
}

impl<'a> BaseController<'a> {
    pub fn new(asm: &'a mut Assembly, caps: &'a ControlCapability) -> Self {
        Self { asm, caps }
    }

    pub fn caps(&self) -> &ControlCapability {
        self.caps
    }

    /// 本体参考 vessel 下标：主组合体 → active；detached → 该 vessel。
    fn vi(&self) -> usize {
        match self.caps.body {
            BodyRef::Primary => self.asm.active,
            BodyRef::Detached(idx) => idx,
        }
    }

    // ---- 读遥测（Base 是控制器侧唯一出口）----

    pub fn attitude_errors(&self) -> (f64, f64) {
        let v = &self.asm.vessels[self.vi()];
        att::attitude_errors(&v.state)
    }
    pub fn pitch_yaw_angles(&self) -> (f64, f64) {
        let v = &self.asm.vessels[self.vi()];
        att::pitch_yaw_angles(&v.state)
    }
    pub fn tip_angle(&self) -> f64 {
        let v = &self.asm.vessels[self.vi()];
        att::tip_angle(&v.state)
    }
    pub fn roll_angle(&self) -> f64 {
        let v = &self.asm.vessels[self.vi()];
        att::roll_angle(&v.state)
    }
    pub fn omega(&self) -> Vec3 {
        self.asm.vessels[self.vi()].state.omega
    }
    /// 本体参考 vessel 的体轴（world 系）：`(x, y, z)` = `state.r` 列。
    pub fn body_axes(&self) -> (Vec3, Vec3, Vec3) {
        let r = self.asm.vessels[self.vi()].state.r;
        (r.col(0), r.col(1), r.col(2))
    }
    pub fn velocity(&self) -> Vec3 {
        match self.caps.body {
            BodyRef::Primary => self.asm.state.vel,
            BodyRef::Detached(_) => self.asm.vessels[self.vi()].state.vel,
        }
    }
    pub fn position(&self) -> Vec3 {
        match self.caps.body {
            BodyRef::Primary => self.asm.state.pos,
            BodyRef::Detached(_) => self.asm.vessels[self.vi()].state.pos,
        }
    }
    pub fn speed(&self) -> f64 {
        self.velocity().length()
    }
    pub fn total_mass(&self) -> f64 {
        match self.caps.body {
            BodyRef::Primary => self.asm.total_mass(),
            BodyRef::Detached(_) => self.asm.vessels[self.vi()].mass(),
        }
    }
    pub fn fuel_mass(&self) -> f64 {
        match self.caps.body {
            BodyRef::Primary => self.asm.total_fuel(),
            BodyRef::Detached(_) => self.asm.vessels[self.vi()].fuel_mass,
        }
    }
    pub fn fuel_percent(&self) -> f64 {
        match self.caps.body {
            BodyRef::Primary => self.asm.fuel_percent(),
            BodyRef::Detached(_) => {
                let v = &self.asm.vessels[self.vi()];
                let tot = v.mass();
                if tot > 0.0 {
                    (v.fuel_mass / tot * 100.0).min(100.0)
                } else {
                    0.0
                }
            }
        }
    }

    // ---- 写执行器（委托 throttle/tvc/rcs/separation 模块）----

    pub fn set_throttle(&mut self, policy: ThrottlePolicy, level: f64) {
        throttle::apply_throttle(self.asm, self.caps, policy, level);
    }
    /// 仅设指定 throttle group（单船）油门；group id 未命中无操作。
    pub fn set_group_throttle(&mut self, group_id: &str, level: f64) {
        throttle::apply_group_throttle(self.asm, self.caps, group_id, level);
    }
    pub fn apply_tvc(&mut self, group_id: &str, pitch_target: f64, yaw_target: f64, dt: f64) {
        tvc_mod::apply_tvc(self.asm, self.caps, group_id, pitch_target, yaw_target, dt);
    }
    pub fn set_rcs(&mut self, group_id: &str, axis: RotAxis, level: f64) {
        rcs_mod::set_rcs(self.asm, self.caps, group_id, axis, level);
    }
    /// 执行分离，透传 `Assembly::undock` / `separate_stage` 返回的分离出 vessel 下标。
    pub fn separate(&mut self, point_id: &str) -> Vec<usize> {
        sep_mod::perform_separate(self.asm, self.caps, point_id)
    }
    pub fn should_auto_separate(&self) -> bool {
        sep_mod::should_auto_separate(self.asm)
    }
}

#[cfg(test)]
mod tests;
