//! 从 LoadedSession 装配 Assembly / Control / PlanetarySystem。

mod place;

use orbitx_config::{BodyConfig, RocketConfig};
use orbitx_controller::capability::ControlCapability;
use orbitx_controller::factory::{build_control, Control};
use orbitx_controller::target::{TargetController, TargetMode};
use orbitx_controller::workflow::WorkFlow;
use orbitx_dynamics::PlanetarySystem;
use orbitx_math::{GGRAV, StateVectors, Vec3};
use orbitx_vessel::{
    atmosphere_from_config, stage_spec_from_config, Assembly, StageSpec,
};
use tracing::info;

use crate::cli::{ControlKindArg, LoadedSession, SessionControl};
use crate::ephem::{
    create_planetary_system, earth_mass_kg, earth_radius_m, earth_sid_rot_period, resolve_ephemeris_data,
};
use crate::input::InputCmd;
use crate::pad::PadState;

pub use place::launch_attitude;

/// 运行态：步进权威持有的船 / 控制 / 环境。
pub struct SimBundle {
    pub asm: Assembly,
    pub control: ActiveControl,
    pub psys: PlanetarySystem,
    pub earth_radius: f64,
    pub rocket_name: String,
    pub rocket_class: String,
    pub pad: PadState,
    pub crash_msg: String,
    pub initial_fuel: Vec<f64>,
    /// 级表显示序（vessel 下标）：上→下→侧挂；加载时算一次，不改 `asm.vessels`。
    pub stage_display_order: Vec<usize>,
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

    pub fn tick(&mut self, asm: &mut Assembly, dt: f64) {
        match self {
            Self::Base => {}
            Self::Manual { controller, caps } => {
                let mut base = orbitx_controller::base::BaseController::new(asm, caps);
                orbitx_controller::base::Controller::tick(controller, &mut base, dt);
            }
            Self::WorkFlow(wf) => wf.tick(asm, dt),
        }
    }

    pub fn apply_input(&mut self, asm: &mut Assembly, cmd: InputCmd) {
        match self {
            Self::Manual { controller, caps } => match cmd {
                InputCmd::SetThrottle { level } => {
                    let mode = controller.mode().with_throttle(level.clamp(0.0, 1.0));
                    controller.set_mode(mode);
                }
                InputCmd::SetAttitudeAxes { pitch, yaw, roll: _ } => {
                    let thr = controller.mode().throttle();
                    controller.set_mode(TargetMode::PitchTo {
                        pitch,
                        yaw,
                        throttle: thr,
                    });
                    controller.reset_turn();
                }
                InputCmd::Separate => {
                    let id = pick_separate_point(caps, asm);
                    if let Some(id) = id {
                        let mut base =
                            orbitx_controller::base::BaseController::new(asm, caps);
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
                        let (p, y) = match controller.mode() {
                            TargetMode::PitchTo { pitch, yaw, .. } => (pitch, yaw),
                            TargetMode::GravityTurn { .. } => (controller.turn_pitch(), 0.0),
                            _ => (0.0, 0.0),
                        };
                        controller.set_mode(TargetMode::PitchTo {
                            pitch: p,
                            yaw: y,
                            throttle: thr,
                        });
                        controller.reset_turn();
                    }
                }
            },
            Self::Base | Self::WorkFlow(_) => {
                // 忽略飞行 Input
            }
        }
    }

    pub fn throttle_cmd(&self) -> f64 {
        match self {
            Self::Manual { controller, .. } => controller.mode().throttle(),
            _ => 0.0,
        }
    }

    pub fn attitude_targets(&self) -> (f64, f64, f64) {
        match self {
            Self::Manual { controller, .. } => match controller.mode() {
                TargetMode::VerticalHold { .. } => (0.0, 0.0, 0.0),
                TargetMode::PitchTo { pitch, yaw, .. } => (pitch, yaw, 0.0),
                TargetMode::GravityTurn { .. } => (controller.turn_pitch(), 0.0, 0.0),
                TargetMode::ProgradeHold { .. } | TargetMode::RetrogradeHold { .. } => (0.0, 0.0, 0.0),
            },
            _ => (0.0, 0.0, 0.0),
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
    ephemeris_data: Option<&std::path::Path>,
) -> Result<SimBundle, String> {
    let src = resolve_ephemeris_data(ephemeris_data);
    let mut psys = create_planetary_system(&src);
    psys.update_positions();

    let earth_radius = earth_radius_m(&psys);
    let sid_period = earth_sid_rot_period(&psys);
    let earth_cfg = BodyConfig::earth();

    let stages = rocket_to_stages(&session.rocket);
    let links = dock_links_from_config(&session.rocket);
    let init = place::initial_state(
        &stages,
        session.scenario.as_ref(),
        earth_radius,
        sid_period,
    )?;
    let pad_pos = init.pos;
    let mut asm = make_assembly(&stages, init, &links);
    for v in &mut asm.vessels {
        if v.rdrag.length() < 1e-12 {
            v.rdrag = Vec3::new(1.0, 0.1, 1.0);
        }
    }
    asm.atmosphere = atmosphere_from_config(earth_cfg.atmosphere.as_ref());
    asm.planet_radius = earth_radius;
    asm.sid_rot_period = sid_period;

    if let Some(scn) = session.scenario.as_ref() {
        place::apply_fuel_levels(&mut asm, &stages, scn);
    }

    let initial_fuel: Vec<f64> = asm.vessels.iter().map(|v| v.fuel_mass).collect();
    let stage_display_order = compute_stage_display_order(&asm);

    let mu = GGRAV * earth_mass_kg(&psys);
    let control = build_active_control(&session.control, &asm, mu)?;

    info!(
        rocket = %session.rocket.name,
        class = %session.rocket.class,
        control = control.label(),
        ephemeris_data = %src.display(),
        earth_r = earth_radius,
        "sim bundle ready"
    );

    Ok(SimBundle {
        asm,
        control,
        psys,
        earth_radius,
        rocket_name: session.rocket.name.clone(),
        rocket_class: session.rocket.class.clone(),
        pad: PadState::new(pad_pos),
        crash_msg: String::new(),
        initial_fuel,
        stage_display_order,
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
            let controller =
                TargetController::new(TargetMode::VerticalHold { throttle: 0.0 });
            let caps = ControlCapability::for_primary(asm);
            Ok(ActiveControl::Manual { controller, caps })
        }
        SessionControl::WorkFlow { desc, .. } => match build_control(desc, asm, mu) {
            Control::WorkFlow(wf) => Ok(ActiveControl::WorkFlow(wf)),
            Control::Controller(_) => Err("build_control returned Controller".into()),
        },
    }
}

fn pick_separate_point(caps: &ControlCapability, asm: &Assembly) -> Option<String> {
    use orbitx_controller::capability::SeparationKind;
    // 优先空燃料侧挂。
    for pt in &caps.separation_points {
        if let SeparationKind::StrapOnLeaf { vessel, .. } = pt.kind {
            if vessel < asm.vessels.len() && asm.vessels[vessel].fuel_mass < 1.0 {
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
