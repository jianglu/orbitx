//! CLI 坠毁判定：扫描主栈与全部独立体高度，调用物理层 `mark_crashed`。

use std::collections::HashSet;

use orbitx_vessel::Assembly;

/// 对主栈与每个未坠毁独立体做高度检查；`alt = |pos| - planet_radius <= 0` 则置坠毁。
///
/// 返回 `Some((name, impact_speed))` 当且仅当本帧新判定主栈坠毁（供 UI 对话框）。
/// 独立体坠毁不返回、不要求 pause。
pub fn apply_crash_checks(asm: &mut Assembly, launched: bool) -> Option<(String, f64)> {
    let primary: HashSet<usize> = asm.components.iter().map(|c| c.vessel_index).collect();

    for i in 0..asm.vessels.len() {
        if asm.vessels[i].crashed || primary.contains(&i) {
            continue;
        }
        let alt = asm.vessels[i].state.pos.length() - asm.planet_radius;
        if alt <= 0.0 {
            asm.mark_crashed(i);
        }
    }

    let primary_crashed = asm
        .components
        .iter()
        .any(|c| asm.vessels[c.vessel_index].crashed);
    if !launched || primary_crashed {
        return None;
    }

    let alt = asm.state.pos.length() - asm.planet_radius;
    if alt > 0.0 {
        return None;
    }

    let active = asm.active.min(asm.vessels.len().saturating_sub(1));
    let impact_speed = asm.state.vel.length();
    let name = asm.active_name().to_string();
    asm.mark_crashed(active);
    Some((name, impact_speed))
}

#[cfg(test)]
mod tests {
    use super::*;
    use orbitx_dynamics::GravBody;
    use orbitx_math::{StateVectors, Vec3};
    use orbitx_vessel::presets;

    fn earth() -> GravBody {
        GravBody {
            pos: Vec3::ZERO,
            mass: 5.972e24,
            size: 6_371_000.0,
            jcoeff: vec![],
            rotation: None,
            pines: None,
        }
    }

    #[test]
    fn detached_at_or_below_surface_is_marked_without_pausing_primary() {
        let stages = presets::falcon9();
        let r = 6_371_000.0 + 50_000.0;
        let state = StateVectors {
            pos: Vec3::new(0.0, 0.0, r),
            vel: Vec3::new(100.0, 0.0, 0.0),
            ..Default::default()
        };
        let mut asm = Assembly::new(&stages, state);
        asm.planet_radius = 6_371_000.0;
        let _ = asm.separate_stage();
        assert!(asm.vessels[0].detached);

        // 压到地表以下。
        asm.vessels[0].state.pos = Vec3::new(0.0, 0.0, asm.planet_radius - 10.0);
        asm.vessels[0].state.vel = Vec3::new(0.0, 0.0, -200.0);

        let primary_evt = apply_crash_checks(&mut asm, true);
        assert!(primary_evt.is_none(), "primary still above ground");
        assert!(asm.vessels[0].crashed);

        let pos0 = asm.vessels[0].state.pos;
        let primary_pos = asm.state.pos;
        let bodies = [earth()];
        for _ in 0..10 {
            asm.step(0.05, &bodies);
        }
        assert!((asm.vessels[0].state.pos - pos0).length() < 1e-9);
        assert!(
            (asm.state.pos - primary_pos).length() > 0.1,
            "primary should keep integrating"
        );
    }

    #[test]
    fn primary_at_surface_returns_event_and_marks_active() {
        let stages = presets::falcon9();
        let state = StateVectors {
            pos: Vec3::new(0.0, 0.0, 6_371_000.0 + 100.0),
            vel: Vec3::new(0.0, 0.0, -50.0),
            ..Default::default()
        };
        let mut asm = Assembly::new(&stages, state);
        asm.planet_radius = 6_371_000.0;
        asm.state.pos = Vec3::new(0.0, 0.0, asm.planet_radius - 1.0);
        asm.state.vel = Vec3::new(10.0, 0.0, -80.0);

        assert!(apply_crash_checks(&mut asm, false).is_none());
        let evt = apply_crash_checks(&mut asm, true).expect("primary crash");
        assert!(evt.1 > 0.0);
        assert!(asm.vessels[asm.active].crashed);
        assert!(apply_crash_checks(&mut asm, true).is_none());
    }
}
