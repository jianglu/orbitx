//! 步进编排：Control → StepEnv → Assembly::step → 环境 T1 → Slice。

use crate::ephem::earth_centered_grav_env;
use crate::runtime::clock::Clock;
use crate::session::SimBundle;
use crate::slice::Slice;
use orbitx_vessel::StepEnv;

pub enum TickOutcome {
    Stepped { slice: Slice },
    Skipped,
}

/// 完成一步固定 `sim_dt`（物理秒）。
pub fn tick(clock: &mut Clock, sim: &mut SimBundle) -> TickOutcome {
    if clock.paused() {
        return TickOutcome::Skipped;
    }

    let dt = clock.sim_dt_secs();

    // 1 Control @ T0
    sim.control.tick(&mut sim.asm, dt);

    // 2 环境 @ T0（星历 → 地心系 GravBody + Earth primary）
    sim.psys.update_positions();
    let (grav, primary) = earth_centered_grav_env(&sim.psys);
    if let Some(b) = grav.get(primary) {
        sim.earth_radius = b.size;
        sim.asm.planet_radius = b.size;
    }

    // 3 航空器
    sim.asm.step(dt, StepEnv::new(&grav, primary));

    // 4 环境 → T1
    sim.psys.advance(dt / 86_400.0);
    clock.advance_fixed_step();

    let pos = sim.asm.state.pos;
    let vel = sim.asm.state.vel;
    let alt = pos.length() - sim.earth_radius;
    let speed = vel.length();
    let fuel: f64 = sim.asm.vessels.iter().map(|v| v.fuel_mass).sum();
    let mass = sim.asm.total_mass();

    TickOutcome::Stepped {
        slice: Slice {
            sim_t: clock.sim_t_ms(),
            step_index: clock.step_index(),
            paused: clock.paused(),
            warp: clock.warp(),
            summary: format!(
                "alt={alt:.1}m speed={speed:.2}m/s mass={mass:.0}kg fuel={fuel:.0}kg"
            ),
        },
    }
}
