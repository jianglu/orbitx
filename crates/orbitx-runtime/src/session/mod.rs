//! 从 LoadedSession 装配 Assembly / Control / PlanetarySystem。

mod place;

use orbitx_config::{BodyConfig, RocketConfig};
use orbitx_controller::capability::ControlCapability;
use orbitx_controller::factory::{
    build_control, build_manual_control, Control, ControllerEntry,
};
use orbitx_controller::target::TargetMode;
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

pub use place::launch_attitude;

/// 运行态：步进权威持有的船 / 控制 / 环境。
pub struct SimBundle {
    pub asm: Assembly,
    pub control: ActiveControl,
    pub psys: PlanetarySystem,
    pub earth_radius: f64,
    pub rocket_name: String,
    pub rocket_class: String,
}

/// 顶层控制（含 Base 空档）。
pub enum ActiveControl {
    /// 模式 a：本期无 Input，不写执行器。
    Base,
    /// 模式 b：手动 TargetController + Runtime 持 caps。
    Manual {
        entries: Vec<ControllerEntry>,
        caps: ControlCapability,
    },
    /// 模式 c/d。
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
            Self::Manual { entries, caps } => {
                for entry in entries.iter_mut() {
                    let mut base = orbitx_controller::base::BaseController::new(asm, caps);
                    orbitx_controller::base::Controller::tick(
                        entry.controller.as_mut(),
                        &mut base,
                        dt,
                    );
                }
            }
            Self::WorkFlow(wf) => wf.tick(asm, dt),
        }
    }
}

/// 由已解析会话 + 历表数据路径构建运行态。
pub fn build_sim_bundle(
    session: &LoadedSession,
    ephemeris_data: Option<&std::path::Path>,
) -> Result<SimBundle, String> {
    let src = resolve_ephemeris_data(ephemeris_data);
    let mut psys = create_planetary_system(&src);
    // 地心装船前确保位置已更新（fallback 时多为原点）。
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
    let mut asm = make_assembly(&stages, init, &links);
    for v in &mut asm.vessels {
        if v.rdrag.length() < 1e-12 {
            v.rdrag = Vec3::new(1.0, 0.1, 1.0);
        }
    }
    // 环境缓存：大气 / 半径 / 自转与参考体 Earth 同源（宿主注入；vessel 不自造默认）。
    asm.atmosphere = atmosphere_from_config(earth_cfg.atmosphere.as_ref());
    asm.planet_radius = earth_radius;
    asm.sid_rot_period = sid_period;

    if let Some(scn) = session.scenario.as_ref() {
        place::apply_fuel_levels(&mut asm, &stages, scn);
    }

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
    })
}

fn build_active_control(
    sc: &SessionControl,
    asm: &Assembly,
    mu: f64,
) -> Result<ActiveControl, String> {
    match sc {
        SessionControl::Control(ControlKindArg::Base) => Ok(ActiveControl::Base),
        SessionControl::Control(ControlKindArg::Target) => {
            let Control::Controller(entries) =
                build_manual_control(TargetMode::VerticalHold { throttle: 1.0 })
            else {
                return Err("build_manual_control returned WorkFlow".into());
            };
            let caps = ControlCapability::for_primary(asm);
            Ok(ActiveControl::Manual { entries, caps })
        }
        SessionControl::WorkFlow { desc, .. } => match build_control(desc, asm, mu) {
            Control::WorkFlow(wf) => Ok(ActiveControl::WorkFlow(wf)),
            Control::Controller(_) => Err("build_control returned Controller".into()),
        },
    }
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
