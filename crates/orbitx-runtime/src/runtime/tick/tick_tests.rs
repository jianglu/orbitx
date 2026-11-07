//! `build_focus_telem` 口径契约（P4.5 修订）：pos/vel/r 恒取焦点船原点状态，
//! 不随 `asm.state` 的双语义（pad 钉死 = 主级原点 / 飞行 = 簇质心）漂移。

use super::{build_focus_telem, build_slice, flatten_mat3, visual_throttle};
use crate::cli::test_runtime_args;
use crate::runtime::clock::Clock;
use crate::runtime::separation_queue::SeparationQueue;
use crate::session::build_sim_bundle;
use orbitx_math::{StateVectors, Vec3};
use orbitx_vessel::assembly::Assembly;
use orbitx_vessel::presets;

/// 钉死态：`asm.state` 被 pad.rs 覆盖为主级原点状态 → focus 与主级原点一致。
#[test]
fn focus_matches_root_vessel_state_when_pinned() {
    let stages = presets::falcon9();
    let mut asm = Assembly::new(&stages, StateVectors::default());
    let vi = asm.active.min(asm.vessels.len() - 1);
    // 模拟 pad.rs after_step 的钉死写回。
    asm.state = asm.vessels[vi].state;

    let f = build_focus_telem(&asm, vi, true, &[], 6.371e6);
    assert_eq!(
        f.pos,
        [
            asm.vessels[vi].state.pos.x,
            asm.vessels[vi].state.pos.y,
            asm.vessels[vi].state.pos.z
        ]
    );
    let rr = flatten_mat3(asm.vessels[vi].state.r);
    assert_eq!(f.rot, rr);
}

/// 飞行态：`asm.state` 是簇质心（step_rigid_cluster 写回），≠ 主级原点；
/// focus 仍必须取主级原点，禁止把质心位泄给渲染端（双扣 CG 的根因）。
#[test]
fn focus_ignores_cluster_cg_state_in_flight() {
    let stages = presets::falcon9();
    let mut asm = Assembly::new(&stages, StateVectors::default());
    let vi = asm.active.min(asm.vessels.len() - 1);
    // 模拟飞行写回后的质心状态：相对主级原点抬高 5 m（栈质心在原点上方）。
    asm.state.pos += orbitx_math::Vec3::new(0.0, 5.0, 0.0);

    let f = build_focus_telem(&asm, vi, true, &[], 6.371e6);
    assert_ne!(asm.state.pos, asm.vessels[vi].state.pos);
    assert_eq!(
        f.pos,
        [
            asm.vessels[vi].state.pos.x,
            asm.vessels[vi].state.pos.y,
            asm.vessels[vi].state.pos.z
        ]
    );
}

/// 视觉口径：未步进（diagnostics.thrust=0）→ 焰灭；与油门指令 level 无关。
#[test]
fn visual_throttle_zero_when_no_actual_thrust() {
    let stages = presets::falcon9();
    let asm = Assembly::new(&stages, StateVectors::default());
    let v = &asm.vessels[0];
    // level 可能 >0（指令），但 diagnostics.thrust 未由步进写入（燃空/关机/未点火）。
    let (thr, firing) = visual_throttle(v);
    assert_eq!(thr, 0.0);
    assert!(!firing);
}

/// 视觉口径随实际推力等比：50% 推力 → throttle=0.5；<2% → firing=false。
#[test]
fn visual_throttle_tracks_actual_thrust() {
    let stages = presets::falcon9();
    let mut asm = Assembly::new(&stages, StateVectors::default());
    let v = &mut asm.vessels[0];
    let max_total: f64 = v
        .thrusters
        .iter()
        .filter(|t| t.max_thrust > 0.0)
        .map(|t| t.max_thrust)
        .sum();
    assert!(max_total > 0.0);

    v.diagnostics.thrust = max_total * 0.5;
    let (thr, firing) = visual_throttle(v);
    assert!((thr - 0.5).abs() < 1e-9);
    assert!(firing);

    v.diagnostics.thrust = max_total * 0.01;
    let (_, firing) = visual_throttle(v);
    assert!(!firing);
}

/// 状态切片回显契约（P3.13）：`build_slice` 被 run_loop 复用为非步进期状态
/// 发布（hold / paused / Pause·Resume·SetWarp 命令回显）的单一组装来源——
/// clock 当前态必须如实回显，且组装纯读（不推进 sim_t / step_index）。
#[test]
fn status_slice_echoes_clock_state_without_side_effects() {
    let args = test_runtime_args("falcon9");
    let session = args.load_session().unwrap();
    let mut sim = build_sim_bundle(&session, &args.ephemeris_data).expect("bundle");
    let mut clock = Clock::new(50);
    let separations = SeparationQueue::new();

    // hold 态（gate 未开）：stepping=false 如实回显（客户端据此停发 Step）。
    let s = build_slice(&clock, &mut sim, false, &separations);
    assert!(!s.stepping);
    assert!(!s.paused);
    assert_eq!(s.warp, 1.0);

    // Pause + SetWarp 应用到 clock 后：状态切片即时回显（命令回显语义）。
    clock.set_paused(true);
    clock.set_warp(2.0);
    let s = build_slice(&clock, &mut sim, false, &separations);
    assert!(s.paused);
    assert_eq!(s.warp, 2.0);

    // 纯读约束：组装无步进副作用——暂停期状态切片 sim_t 不走表。
    let (t0, idx0) = (clock.sim_t_ms(), clock.step_index());
    let _ = build_slice(&clock, &mut sim, true, &separations);
    assert_eq!(clock.sim_t_ms(), t0);
    assert_eq!(clock.step_index(), idx0);
}
