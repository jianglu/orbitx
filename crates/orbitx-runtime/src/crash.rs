//! 坠毁判定（P4.3 过渡启发式；终态 P5 高程/接触）。

use std::collections::HashSet;

use orbitx_vessel::Assembly;

/// 返回 `Some((name, impact_speed))` 当且仅当本帧新判定主栈坠毁。
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
