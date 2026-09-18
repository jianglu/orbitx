//! Assembly 对接 / 分离局部不变量（同轴栈）。

use super::*;
use crate::presets;
use orbitx_math::StateVectors;

#[test]
fn coaxial_new_docks_adjacent_top_bottom() {
    let asm = Assembly::new(&presets::falcon9(), StateVectors::default());
    assert!(asm.vessels[0].docks[1].connected_to.is_some());
    assert!(asm.vessels[1].docks[0].connected_to.is_some());
    assert_eq!(asm.components.len(), 3);
}

#[test]
fn separate_stage_detaches_bottom_keeps_upper() {
    let mut asm = Assembly::new(&presets::falcon9(), StateVectors::default());
    let next = asm.separate_stage();
    assert!(asm.vessels[0].detached);
    assert!(!asm.vessels[1].detached);
    assert_eq!(asm.active, next);
    assert_eq!(asm.active_name(), "F9-S2");
    assert_eq!(asm.components.len(), 2);
}

#[test]
fn with_dock_links_empty_is_single_components() {
    let stages = presets::falcon9();
    let asm = Assembly::with_dock_links(&stages[..1], StateVectors::default(), &[]);
    assert_eq!(asm.components.len(), 1);
    assert_eq!(asm.total_mass(), stages[0].total_mass());
}
