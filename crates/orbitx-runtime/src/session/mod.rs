//! 从 LoadedSession 装配 Assembly / Control / PlanetarySystem。

mod place;

use orbitx_config::RocketConfig;
use orbitx_controller::capability::ControlCapability;
use orbitx_controller::factory::{build_control, build_graph_workflow, Control};
use orbitx_controller::target::{TargetController, TargetMode};
use orbitx_controller::workflow::WorkFlow;
use orbitx_environment::PlanetarySystem;
use orbitx_math::{StateVectors, Vec3, GGRAV};
use orbitx_vessel::{
    atmosphere_from_config, stage_spec_from_config, surface_inertial_velocity, Assembly, StageSpec,
};
use tracing::info;

use crate::cli::{ControlKindArg, LoadedSession, SessionControl};
use crate::crash::{LandDwellState, LandedState};
use crate::input::InputCmd;
use crate::pad::PadState;
use crate::slice::Slice;
use crate::world::World;

pub use place::launch_attitude;

/// 运行态：步进权威持有的船 / 控制 / 环境。
pub struct SimBundle {
    pub asm: Assembly,
    pub control: ActiveControl,
    pub world: World,
    pub earth_radius: f64,
    pub pad: PadState,
    /// 台位支持面（底部落点）地心半径 [m]：`R_elev + base + 台架高`；离地判定基准。
    pub pad_support_radius_m: f64,
    pub crash_msg: String,
    /// 着陆成功终局消息（非空即终局；CLI 负责暂停）。
    pub landed_msg: String,
    /// 发射失败终局消息（非空即终局；CLI 负责暂停）。
    pub launch_failed_msg: String,
    /// 本次飞行曾相对地面离地 >1 m（着陆成功解锁条件）。
    pub was_airborne: bool,
    /// 本次飞行曾有推力尝试（发射失败判定条件）。
    pub thrust_attempted: bool,
    /// Orbiter 式着陆稳定计时（无动力贴地连续 5 s）。
    pub land_dwell: LandDwellState,
    /// 软着陆落定（体固钉姿 / 共转，对齐 Orbiter `land_rot`）。
    pub landed: LandedState,
    pub initial_fuel: Vec<f64>,
    /// 级表显示序（vessel 下标）：上→下→侧挂；加载时算一次，不改 `asm.vessels`。
    pub stage_display_order: Vec<usize>,
    /// 上一步 `build_slice` 的产物。第一步为未就绪的空切片。
    pub last_slice: Slice,
    /// P5.1：可选地表会话（由 `--surface-package` 打开）。
    pub surface: Option<crate::surface_session::SurfaceSession>,
}

impl SimBundle {
    /// True when a surface package session is attached (inject owns `asm.ground`).
    pub fn has_surface_session(&self) -> bool {
        self.surface.is_some()
    }
}

/// 顶层控制（含 Base 空档）。
pub enum ActiveControl {
    Base,
    Manual {
        controller: TargetController,
        caps: ControlCapability,
    },
    WorkFlow(Box<dyn WorkFlow>),
}

impl ActiveControl {
    pub fn label(&self) -> &'static str {
        match self {
            Self::Base => "control:base",
            Self::Manual { .. } => "control:target",
            Self::WorkFlow(_) => "workflow",
        }
    }

    pub fn tick(&mut self, asm: &mut Assembly, slice: &Slice, dt: f64) {
        match self {
            Self::Base => {}
            Self::Manual { controller, caps } => {
                let mut base = orbitx_controller::base::BaseController::new(asm, caps);
                orbitx_controller::base::Controller::tick(controller, &mut base, dt);
            }
            Self::WorkFlow(wf) => wf.tick(asm, slice, dt),
        }
    }

    pub fn apply_input(&mut self, asm: &mut Assembly, cmd: InputCmd) {
        match self {
            Self::Manual { controller, caps } => match cmd {
                InputCmd::SetThrottle { level } => {
                    let mode = controller.mode().with_throttle(level.clamp(0.0, 1.0));
                    controller.set_mode(mode);
                }
                InputCmd::SetAttitudeAxes { pitch, yaw, roll } => {
                    let thr = controller.mode().throttle();
                    controller.set_mode(TargetMode::PitchTo {
                        pitch,
                        yaw,
                        roll,
                        throttle: thr,
                    });
                    controller.reset_turn();
                }
                InputCmd::SetRcsEnabled { enabled } => {
                    controller.set_rcs_enabled(enabled);
                }
                InputCmd::Separate => {
                    let id = pick_separate_point(caps, asm);
                    if let Some(id) = id {
                        let mut base = orbitx_controller::base::BaseController::new(asm, caps);
                        let _ = base.separate(&id);
                        *caps = ControlCapability::for_primary(asm);
                    }
                }
                InputCmd::SetGravityTurn { enabled } => {
                    let thr = controller.mode().throttle();
                    if enabled {
                        controller.set_mode(TargetMode::gravity_turn(thr));
                        controller.reset_turn();
                    } else {
                        let (p, y, r) = match controller.mode() {
                            TargetMode::PitchTo {
                                pitch, yaw, roll, ..
                            } => (pitch, yaw, roll),
                            TargetMode::GravityTurn { .. } => {
                                (controller.pitch_cmd(), controller.yaw_cmd(), 0.0)
                            }
                            _ => (0.0, 0.0, 0.0),
                        };
                        controller.set_mode(TargetMode::PitchTo {
                            pitch: p,
                            yaw: y,
                            roll: r,
                            throttle: thr,
                        });
                        controller.reset_turn();
                    }
                }
                InputCmd::WorkflowIgnite => {}
            },
            Self::WorkFlow(wf) => {
                if matches!(cmd, InputCmd::WorkflowIgnite) {
                    wf.ignite();
                }
            }
            Self::Base => {}
        }
    }

    pub fn throttle_cmd(&self) -> f64 {
        match self {
            Self::Manual { controller, .. } => controller.mode().throttle(),
            Self::WorkFlow(wf) => wf.throttle_cmd(),
            Self::Base => 0.0,
        }
    }

    /// RCS 姿态参与开关状态与船是否有 RCS 组（HUD 回显）。
    pub fn rcs_state(&self) -> (bool, bool) {
        match self {
            Self::Manual { controller, caps } => {
                (controller.rcs_enabled(), !caps.rcs_groups.is_empty())
            }
            Self::WorkFlow(_) | Self::Base => (false, false),
        }
    }

    pub fn attitude_targets(&self) -> (f64, f64, f64) {
        match self {
            Self::Manual { controller, .. } => {
                orbitx_controller::workflow::attitude_from_target(controller)
            }
            Self::WorkFlow(wf) => wf.attitude_targets(),
            Self::Base => (0.0, 0.0, 0.0),
        }
    }

    pub fn gravity_turn_enabled(&self) -> bool {
        matches!(
            self,
            Self::Manual {
                controller,
                ..
            } if matches!(controller.mode(), TargetMode::GravityTurn { .. })
        )
    }
}

/// 由已解析会话 + 历表数据路径构建运行态。
pub fn build_sim_bundle(
    session: &LoadedSession,
    ephemeris_data: &std::path::Path,
) -> Result<SimBundle, String> {
    let mut psys = PlanetarySystem::load(&session.scenario, ephemeris_data)?;
    psys.update();
    let surface = psys.primary_surface();

    // runtime 不内置默认发射场：无 lp 直接拒绝（load_session 已拦，此为后备）。
    let Some(ref lp) = session.launchpad else {
        return Err(
            "必须指定 --launchpad <lp_xxx.toml>（runtime 不内置默认发射场）".into(),
        );
    };
    // 本轮仅主天体可摆放/飞行；非主天体发射场明确拒绝，不按主天体坐标解释。
    if !lp.is_primary {
        return Err(format!(
            "发射场位于天体 `{}`，非主天体摆放/触地尚未支持（当前主天体 `{}`）",
            lp.body_name, session.scenario.primary
        ));
    }

    let stages = rocket_to_stages(&session.rocket);
    let links = dock_links_from_config(&session.rocket);

    // 先开地表会话（含发射场覆盖），摆放才能读合成地面。
    // 包按绑定天体（cfg.body_name，缺省主天体）的 size 校验 R_elev。
    let surface_sess = if let Some(ref cfg) = session.surface {
        let bound_size = if cfg.body_name.is_empty() {
            surface.radius
        } else {
            psys.bodies()
                .iter()
                .find(|b| b.name == cfg.body_name)
                .map(|b| b.size)
                .ok_or_else(|| format!("地表包绑定天体 `{}` 不在天体表中", cfg.body_name))?
        };
        let mut sess = crate::surface_session::SurfaceSession::open(cfg.clone(), bound_size)?;
        sess.set_rot(surface.orientation.rot);
        Some(sess)
    } else {
        None
    };

    let elev_radius_m = surface_sess
        .as_ref()
        .map(|s| s.elev_radius_m)
        .unwrap_or(surface.radius);
    let init = {
        let pad = lp.pad();
        place::initial_state_on_pad(
            &stages,
            lp.config.latitude_deg,
            lp.config.longitude_deg,
            pad.axis_offset_m,
            pad.axis_dir,
            lp.heading_deg,
            elev_radius_m,
            lp.config.base_elevation_m,
            surface.orientation,
        )
    };
    let mut asm = make_assembly(&stages, init, &links);
    for v in &mut asm.vessels {
        if v.rdrag.length() < 1e-12 {
            v.rdrag = Vec3::new(1.0, 0.1, 1.0);
        }
    }

    // 底部归零：place 只按质心 + 半箭高近似，装配原点偏移使底部位置不可靠；
    // 沿底部径向平移全栈，使主组合体底部精确落在台位支持面。
    let support_radius_m = lp.support_radius_m(elev_radius_m);
    if let Some(bottom) = asm.primary_bottom_point() {
        let r = bottom.length().max(1e-9);
        let shift = bottom * ((support_radius_m - r) / r);
        if shift.length() > 1e-12 {
            for v in &mut asm.vessels {
                if !v.detached {
                    v.state.pos = v.state.pos + shift;
                }
            }
            asm.state.pos = asm.state.pos + shift;
        }
    }
    // 台架钉死量 = 归零后 root 船惯性位姿（含 lp heading），再存体固系。
    let pad_pos = asm.vessels[asm.root].state.pos;
    let pad_r = asm.vessels[asm.root].state.r;
    asm.atmosphere = atmosphere_from_config(surface.atmosphere.as_ref());
    asm.planet_radius = surface.radius;
    asm.sid_rot_period = surface.sid_rot_period;
    asm.planet_orientation = surface.orientation;
    // 底部归零后重算共转速度（位置变了）。
    let corot = surface_inertial_velocity(asm.state.pos, surface.orientation.omega);
    for v in &mut asm.vessels {
        if !v.detached {
            v.state.vel = corot;
        }
    }
    asm.state.vel = corot;

    let initial_fuel: Vec<f64> = asm.vessels.iter().map(|v| v.fuel_mass()).collect();
    let stage_display_order = compute_stage_display_order(&asm);

    let mu = GGRAV * surface.mass;
    let control = build_active_control(&session.control, &asm, mu)?;

    info!(
        rocket = %session.rocket.name,
        class = %session.rocket.class,
        control = control.label(),
        scenario = %session.scenario.name,
        ephemeris_data = %ephemeris_data.display(),
        earth_r = surface.radius,
        surface = surface_sess
            .as_ref()
            .map(|s| s.package_root().display().to_string())
            .unwrap_or_else(|| "(none)".into()),
        launchpad = session
            .launchpad
            .as_ref()
            .map(|lp| format!("{}:{}", lp.config.name, lp.pad_name))
            .unwrap_or_else(|| "(default)".into()),
        "sim bundle ready"
    );

    Ok(SimBundle {
        asm,
        control,
        world: World::new(
            psys,
            session.rocket.name.clone(),
            session.rocket.class.clone(),
        ),
        earth_radius: surface.radius,
        pad: PadState::new(pad_pos, pad_r, surface.orientation.rot),
        pad_support_radius_m: support_radius_m,
        crash_msg: String::new(),
        landed_msg: String::new(),
        launch_failed_msg: String::new(),
        was_airborne: false,
        thrust_attempted: false,
        land_dwell: LandDwellState::default(),
        landed: LandedState::default(),
        initial_fuel,
        stage_display_order,
        last_slice: Slice::default(),
        surface: surface_sess,
    })
}

/// 级表显示序：芯级栈按 root 系 Y 降序（上→下），再侧挂按方位角。
///
/// 只读 `components` / strap-on 叶；不改 `vessels` 存储序。
pub fn compute_stage_display_order(asm: &Assembly) -> Vec<usize> {
    use orbitx_vessel::ThrustReadout;
    use std::collections::HashSet;

    let n = asm.vessels.len();
    if n == 0 {
        return Vec::new();
    }

    let strap: HashSet<usize> = asm.strap_on_leaf_indices().map(|(i, _)| i).collect();

    let mut y_of = vec![0.0_f64; n];
    let mut xz_of = vec![(0.0_f64, 0.0_f64); n];
    for c in &asm.components {
        if c.vessel_index < n {
            y_of[c.vessel_index] = c.rpos.y;
            xz_of[c.vessel_index] = (c.rpos.x, c.rpos.z);
        }
    }

    let mut stack: Vec<usize> = (0..n).filter(|i| !strap.contains(i)).collect();
    stack.sort_by(|&a, &b| {
        y_of[b]
            .partial_cmp(&y_of[a])
            .unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| a.cmp(&b))
    });

    let mut sides: Vec<usize> = strap.into_iter().collect();
    sides.sort_by(|&a, &b| {
        let (xa, za): (f64, f64) = xz_of[a];
        let (xb, zb): (f64, f64) = xz_of[b];
        let aa = za.atan2(xa);
        let ab = zb.atan2(xb);
        aa.partial_cmp(&ab)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| a.cmp(&b))
    });

    stack.extend(sides);
    stack
}

fn build_active_control(
    sc: &SessionControl,
    asm: &Assembly,
    mu: f64,
) -> Result<ActiveControl, String> {
    match sc {
        SessionControl::Control(ControlKindArg::Base) => Ok(ActiveControl::Base),
        SessionControl::Control(ControlKindArg::Target) => {
            // 冷启动油门 0。
            let controller = TargetController::new(TargetMode::VerticalHold { throttle: 0.0 });
            let caps = ControlCapability::for_primary(asm);
            Ok(ActiveControl::Manual { controller, caps })
        }
        SessionControl::WorkFlow { desc, .. } => match build_control(desc, asm, mu) {
            Control::WorkFlow(wf) => Ok(ActiveControl::WorkFlow(wf)),
            Control::Controller(_) => Err("build_control returned Controller".into()),
        },
        SessionControl::Graph { ir, .. } => {
            build_graph_workflow(ir, asm, mu).map(ActiveControl::WorkFlow)
        }
    }
}

fn pick_separate_point(caps: &ControlCapability, asm: &Assembly) -> Option<String> {
    use orbitx_controller::capability::SeparationKind;
    // 优先空燃料侧挂。
    for pt in &caps.separation_points {
        if let SeparationKind::StrapOnLeaf { vessel, .. } = pt.kind {
            if vessel < asm.vessels.len() && asm.vessels[vessel].fuel_mass() < 1.0 {
                return Some(pt.id.clone());
            }
        }
    }
    for pt in &caps.separation_points {
        if matches!(pt.kind, SeparationKind::StrapOnLeaf { .. }) {
            return Some(pt.id.clone());
        }
    }
    caps.separation_points
        .iter()
        .find(|p| matches!(p.kind, SeparationKind::CoaxialStage))
        .map(|p| p.id.clone())
}

fn rocket_to_stages(config: &RocketConfig) -> Vec<StageSpec> {
    config.stages.iter().map(stage_spec_from_config).collect()
}

fn dock_links_from_config(config: &RocketConfig) -> Option<Vec<(usize, usize, usize, usize)>> {
    config.dock_links.as_ref().map(|links| {
        links
            .iter()
            .map(|l| (l.stage, l.port, l.remote_stage, l.remote_port))
            .collect()
    })
}

fn make_assembly(
    stages: &[StageSpec],
    init_state: StateVectors,
    links: &Option<Vec<(usize, usize, usize, usize)>>,
) -> Assembly {
    match links {
        Some(l) => Assembly::with_dock_links(stages, init_state, l),
        None => Assembly::new(stages, init_state),
    }
}

#[cfg(test)]
mod tests;
