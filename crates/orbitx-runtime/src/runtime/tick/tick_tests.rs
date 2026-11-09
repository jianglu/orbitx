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

// ---- gimbal / throttle 遥测口径（WP1 同款加权；tick.rs build_focus_telem）----

/// gimbal 遥测 = 主推 ∧ 点火 ∧ 矢量按 current_thrust 加权：首台固定引擎
/// 点火不再掩盖其余引擎的打舵（旧「首台 max_thrust>0」口径回归护栏）。
#[test]
fn focus_gimbal_telem_ignores_firing_fixed_thruster() {
    let stages = presets::falcon9();
    let mut asm = Assembly::new(&stages, StateVectors::default());
    let vi = asm.active.min(asm.vessels.len() - 1);
    {
        let v = &mut asm.vessels[vi];
        assert!(v.thrusters.len() >= 2, "用例需多台主推");
        for t in v.thrusters.iter_mut() {
            t.max_thrust = 1.0e6; // 等推力 → 加权即算术平均
            t.level = 1.0;
            t.max_gimbal = 0.15;
            t.gimbal_pitch = 0.04;
            t.gimbal_yaw = -0.02;
        }
        // 首台固定（max_gimbal=0，同 set_gimbal 固定语义：舵角恒 0）。
        v.thrusters[0].max_gimbal = 0.0;
        v.thrusters[0].gimbal_pitch = 0.0;
        v.thrusters[0].gimbal_yaw = 0.0;
    }
    let f = build_focus_telem(&asm, vi, true, &[], 6.371e6);
    assert!((f.gimbal_pitch - 0.04).abs() < 1e-12, "got {}", f.gimbal_pitch);
    assert!((f.gimbal_yaw + 0.02).abs() < 1e-12, "got {}", f.gimbal_yaw);
}

/// 多台矢量点火且舵角不同 → 推力加权平均（等推力下即算术平均）。
#[test]
fn focus_gimbal_telem_averages_firing_vector_thrusters() {
    let stages = presets::falcon9();
    let mut asm = Assembly::new(&stages, StateVectors::default());
    let vi = asm.active.min(asm.vessels.len() - 1);
    {
        let v = &mut asm.vessels[vi];
        assert!(v.thrusters.len() >= 2, "用例需多台主推");
        for (i, t) in v.thrusters.iter_mut().enumerate() {
            t.max_thrust = 1.0e6;
            t.max_gimbal = 0.15;
            t.level = if i < 2 { 1.0 } else { 0.0 };
            t.gimbal_pitch = match i {
                0 => 0.08,
                1 => -0.04,
                _ => 0.02, // 熄火：不进加权
            };
            t.gimbal_yaw = 0.0;
        }
    }
    let f = build_focus_telem(&asm, vi, true, &[], 6.371e6);
    assert!((f.gimbal_pitch - 0.02).abs() < 1e-12, "got {}", f.gimbal_pitch);
    assert!(f.gimbal_yaw.abs() < 1e-12, "got {}", f.gimbal_yaw);
}

/// 无点火矢量引擎（全固定 / 全熄火）→ gimbal 遥测回退 0（未打舵）。
#[test]
fn focus_gimbal_telem_zero_without_firing_vector_thrusters() {
    let stages = presets::falcon9();
    // 全固定且点火。
    let mut asm = Assembly::new(&stages, StateVectors::default());
    let vi = asm.active.min(asm.vessels.len() - 1);
    {
        let v = &mut asm.vessels[vi];
        for t in v.thrusters.iter_mut() {
            t.level = 1.0;
            t.max_gimbal = 0.0;
            t.gimbal_pitch = 0.1; // 固定件不该出现非零舵角；即便误置也不进聚合
        }
    }
    let f = build_focus_telem(&asm, vi, true, &[], 6.371e6);
    assert!(f.gimbal_pitch.abs() < 1e-12, "got {}", f.gimbal_pitch);
    assert!(f.gimbal_yaw.abs() < 1e-12, "got {}", f.gimbal_yaw);

    // 全矢量但全熄火。
    let mut asm = Assembly::new(&stages, StateVectors::default());
    let vi = asm.active.min(asm.vessels.len() - 1);
    {
        let v = &mut asm.vessels[vi];
        for t in v.thrusters.iter_mut() {
            t.level = 0.0;
            t.max_gimbal = 0.15;
            t.gimbal_pitch = 0.1; // 熄火不进聚合
        }
    }
    let f = build_focus_telem(&asm, vi, true, &[], 6.371e6);
    assert!(f.gimbal_pitch.abs() < 1e-12, "got {}", f.gimbal_pitch);
    assert!(f.gimbal_yaw.abs() < 1e-12, "got {}", f.gimbal_yaw);
}

/// TVC 量程（FocusTelem.gimbal_max）= 焦点级主推矢量引擎最大舵量：
/// 异构量程取 max，且为能力口径——未点火矢量引擎仍计入。
#[test]
fn focus_gimbal_max_takes_max_over_vector_thrusters() {
    let stages = presets::falcon9();
    let mut asm = Assembly::new(&stages, StateVectors::default());
    let vi = asm.active.min(asm.vessels.len() - 1);
    {
        let v = &mut asm.vessels[vi];
        assert!(v.n_main_thrusters >= 2, "用例需多台主推");
        for t in v.thrusters.iter_mut() {
            t.level = 0.0;
            t.max_gimbal = 0.0;
        }
        v.thrusters[0].level = 1.0; // 仅首台点火
        v.thrusters[0].max_gimbal = 0.10;
        // 未点火矢量引擎仍计入（能力口径，非状态）。
        v.thrusters[1].max_gimbal = 0.1745;
    }
    let f = build_focus_telem(&asm, vi, true, &[], 6.371e6);
    assert!((f.gimbal_max - 0.1745).abs() < 1e-12, "got {}", f.gimbal_max);
}

/// 焦点级无矢量引擎（全固定）→ gimbal_max = 0（消费端保留默认量程）。
#[test]
fn focus_gimbal_max_zero_without_vector_thrusters() {
    let stages = presets::falcon9();
    let mut asm = Assembly::new(&stages, StateVectors::default());
    let vi = asm.active.min(asm.vessels.len() - 1);
    {
        let v = &mut asm.vessels[vi];
        for t in v.thrusters.iter_mut() {
            t.level = 1.0;
            t.max_gimbal = 0.0;
        }
    }
    let f = build_focus_telem(&asm, vi, true, &[], 6.371e6);
    assert!(f.gimbal_max.abs() < 1e-12, "got {}", f.gimbal_max);
}

/// 非 main 槽位（ti ≥ n_main_thrusters）不进 gimbal 遥测——与
/// apply_tvc_stack 的 slew 范围（0..n_main）同界。
#[test]
fn focus_gimbal_telem_ignores_non_main_thrusters() {
    let stages = presets::falcon9();
    let mut asm = Assembly::new(&stages, StateVectors::default());
    let vi = asm.active.min(asm.vessels.len() - 1);
    {
        let v = &mut asm.vessels[vi];
        assert!(v.thrusters.len() >= 2, "用例需多台主推");
        v.n_main_thrusters = 1; // 仅首台为 main，其余视为非主推槽位
        for (i, t) in v.thrusters.iter_mut().enumerate() {
            t.max_thrust = 1.0e6;
            t.level = 1.0;
            t.max_gimbal = 0.15;
            t.gimbal_pitch = if i == 0 { 0.05 } else { 0.5 };
            t.gimbal_yaw = 0.0;
        }
    }
    let f = build_focus_telem(&asm, vi, true, &[], 6.371e6);
    assert!((f.gimbal_pitch - 0.05).abs() < 1e-12, "got {}", f.gimbal_pitch);
}

/// focus.throttle = visual_throttle 视觉口径（与 stages[]/分离体同源）：
/// 首台熄火不再把实际点火显示为 0（旧「首台 level」口径回归护栏）。
#[test]
fn focus_throttle_uses_visual_aggregate() {
    let stages = presets::falcon9();
    let mut asm = Assembly::new(&stages, StateVectors::default());
    let vi = asm.active.min(asm.vessels.len() - 1);
    {
        let v = &mut asm.vessels[vi];
        assert!(v.thrusters.len() >= 2, "用例需多台主推");
        for (i, t) in v.thrusters.iter_mut().enumerate() {
            t.level = if i == 0 { 0.0 } else { 1.0 }; // 首台熄火
        }
        let max_total: f64 = v.thrusters.iter().map(|t| t.max_thrust).sum();
        v.diagnostics.thrust = max_total * 0.5; // 实际推力 50%
    }
    let f = build_focus_telem(&asm, vi, true, &[], 6.371e6);
    let (vis, _) = visual_throttle(&asm.vessels[vi]);
    assert!((f.throttle - vis).abs() < 1e-12, "got {} vs {}", f.throttle, vis);
    assert!((f.throttle - 0.5).abs() < 1e-9, "got {}", f.throttle);
}
