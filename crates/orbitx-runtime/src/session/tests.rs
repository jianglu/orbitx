use clap::Parser;

use crate::cli::{ControlKindArg, RuntimeArgs, SessionControl};
use crate::runtime::clock::Clock;
use crate::runtime::tick;
use crate::session::build_sim_bundle;

#[test]
fn builds_falcon9_bundle() {
    let args = RuntimeArgs::try_parse_from(["orbitx-runtime", "--rocket", "falcon9"]).unwrap();
    let session = args.load_session().unwrap();
    let sim = build_sim_bundle(&session, None).expect("bundle");
    assert_eq!(sim.rocket_class, "Falcon9");
    assert!(sim.asm.vessels.len() >= 2);
    assert!(matches!(
        session.control,
        SessionControl::Control(ControlKindArg::Target)
    ));
}

#[test]
fn target_climb_increases_altitude() {
    let args = RuntimeArgs::try_parse_from([
        "orbitx-runtime",
        "--rocket",
        "falcon9",
        "--control",
        "target",
    ])
    .unwrap();
    let session = args.load_session().unwrap();
    let mut sim = build_sim_bundle(&session, None).expect("bundle");
    sim.control
        .apply_input(&mut sim.asm, crate::input::InputCmd::SetThrottle { level: 1.0 });
    let mut clock = Clock::new(20);
    let alt0 = sim.asm.state.pos.length() - sim.earth_radius;

    for _ in 0..500 {
        let _ = tick::tick(&mut clock, &mut sim);
    }

    let alt1 = sim.asm.state.pos.length() - sim.earth_radius;
    assert!(
        alt1 > alt0 + 10.0,
        "expected climb: alt0={alt0} alt1={alt1} summary steps={}",
        clock.step_index()
    );
}

#[test]
fn slice_detached_empty_when_intact_then_filled_after_separate() {
    use crate::runtime::tick::TickOutcome;

    let args = RuntimeArgs::try_parse_from([
        "orbitx-runtime",
        "--rocket",
        "falcon9",
        "--control",
        "target",
    ])
    .unwrap();
    let session = args.load_session().unwrap();
    let mut sim = build_sim_bundle(&session, None).expect("bundle");
    let mut clock = Clock::new(20);

    let intact = match tick::tick(&mut clock, &mut sim) {
        TickOutcome::Stepped { slice } => slice,
        TickOutcome::Skipped => panic!("expected stepped"),
    };
    assert!(
        intact.detached.is_empty(),
        "intact stack must not list detached FocusTelem"
    );
    assert!(intact.focus.is_primary);
    assert!(intact.focus.omega[0].is_finite());

    sim.control
        .apply_input(&mut sim.asm, crate::input::InputCmd::Separate);

    let after = match tick::tick(&mut clock, &mut sim) {
        TickOutcome::Stepped { slice } => slice,
        TickOutcome::Skipped => panic!("expected stepped"),
    };
    assert!(
        !after.detached.is_empty(),
        "after separate Slice.detached must be non-empty"
    );
    assert!(after.focus.is_primary);
    for d in &after.detached {
        assert!(!d.is_primary);
        assert!(!d.display_name.is_empty());
    }
}

#[test]
fn falcon9_stage_display_order_top_to_bottom() {
    use crate::runtime::tick::TickOutcome;

    let args = RuntimeArgs::try_parse_from(["orbitx-runtime", "--rocket", "falcon9"]).unwrap();
    let session = args.load_session().unwrap();
    let mut sim = build_sim_bundle(&session, None).expect("bundle");

    let names: Vec<&str> = sim
        .stage_display_order
        .iter()
        .map(|&i| sim.asm.vessels[i].name.as_str())
        .collect();
    assert_eq!(names, vec!["Payload", "F9-S2", "F9-S1"]);

    let mut clock = Clock::new(20);
    let slice = match tick::tick(&mut clock, &mut sim) {
        TickOutcome::Stepped { slice } => slice,
        TickOutcome::Skipped => panic!("expected stepped"),
    };
    let telem_names: Vec<&str> = slice.stages.iter().map(|s| s.name.as_str()).collect();
    assert_eq!(telem_names, vec!["Payload", "F9-S2", "F9-S1"]);
    assert_eq!(slice.stages[0].vessel_index, 2);
    assert_eq!(slice.stages[2].vessel_index, 0);
}

#[test]
fn lm2f_stage_display_order_stack_then_boosters() {
    let args = RuntimeArgs::try_parse_from(["orbitx-runtime", "--rocket", "lm2f"]).unwrap();
    let session = args.load_session().unwrap();
    let sim = build_sim_bundle(&session, None).expect("bundle");

    let names: Vec<&str> = sim
        .stage_display_order
        .iter()
        .map(|&i| sim.asm.vessels[i].name.as_str())
        .collect();
    assert!(
        names.len() >= 7,
        "expected 7 stages, got {names:?}"
    );
    assert_eq!(&names[..3], &["Shenzhou", "CZ2F-S2", "CZ2F-S1"]);
    let boosters = &names[3..];
    assert_eq!(boosters.len(), 4);
    for b in [
        "CZ2F-Booster-PX",
        "CZ2F-Booster-MX",
        "CZ2F-Booster-PZ",
        "CZ2F-Booster-MZ",
    ] {
        assert!(
            boosters.contains(&b),
            "missing {b} in boosters {boosters:?}"
        );
    }
}

#[test]
#[ignore = "手动性能采样：cargo test -p orbitx-runtime --release profile_tick -- --ignored --nocapture"]
fn profile_tick_wall_time_after_launch() {
    use crate::runtime::tick::TickOutcome;
    use std::time::Instant;

    for rocket in ["falcon9", "lm2f"] {
        let args = RuntimeArgs::try_parse_from([
            "orbitx-runtime",
            "--rocket",
            rocket,
            "--control",
            "target",
        ])
        .unwrap();
        let session = args.load_session().unwrap();
        let mut sim = build_sim_bundle(&session, None).expect("bundle");
        sim.control
            .apply_input(&mut sim.asm, crate::input::InputCmd::SetThrottle { level: 1.0 });
        let mut clock = Clock::new(20);

        // 暖机直到起飞。
        let mut launched = false;
        for _ in 0..5_000 {
            let _ = tick::tick(&mut clock, &mut sim);
            if sim.pad.launched {
                launched = true;
                break;
            }
        }
        assert!(launched, "{rocket}: failed to launch");

        const N: usize = 2_000;
        let mut samples_us = Vec::with_capacity(N);
        let mut phase = PhaseAccum::default();

        for _ in 0..N {
            let t0 = Instant::now();
            // 分段计时（与 tick 同序，避免双倍步进）。
            let dt = clock.sim_dt_secs();
            let thr_cmd = sim.control.throttle_cmd();

            let a = Instant::now();
            sim.control.tick(&mut sim.asm, dt);
            phase.control_ns += a.elapsed().as_nanos() as u64;

            let a = Instant::now();
            sim.psys.update_positions();
            let (grav, primary) = crate::ephem::earth_centered_grav_env(&sim.psys);
            if let Some(b) = grav.get(primary) {
                sim.earth_radius = b.size;
                sim.asm.planet_radius = b.size;
            }
            phase.env_ns += a.elapsed().as_nanos() as u64;

            let a = Instant::now();
            sim.asm
                .step(dt, orbitx_vessel::StepEnv::new(&grav, primary));
            phase.asm_ns += a.elapsed().as_nanos() as u64;

            let a = Instant::now();
            crate::pad::after_step(&mut sim.asm, &mut sim.pad, thr_cmd);
            let _ = crate::crash::apply_crash_checks(&mut sim.asm, sim.pad.launched);
            sim.psys.advance(dt / 86_400.0);
            clock.advance_fixed_step();
            phase.pad_crash_ns += a.elapsed().as_nanos() as u64;

            let a = Instant::now();
            // 走完整 tick 的 slice 路径：再调一次 tick 会双倍步进，故只测完整 tick 的另一批样本。
            let _ = a;
            samples_us.push(t0.elapsed().as_nanos() as u64);
        }

        // 完整 tick（含 build_slice）另采一批。
        let mut full_us = Vec::with_capacity(N);
        for _ in 0..N {
            let t0 = Instant::now();
            match tick::tick(&mut clock, &mut sim) {
                TickOutcome::Stepped { .. } => {}
                TickOutcome::Skipped => panic!("unexpected skip"),
            }
            full_us.push(t0.elapsed().as_nanos() as u64);
        }

        let report = |label: &str, v: &mut [u64]| {
            v.sort_unstable();
            let n = v.len() as f64;
            let sum: u64 = v.iter().sum();
            let mean = sum as f64 / n;
            let p50 = v[v.len() / 2] as f64;
            let p95 = v[((v.len() as f64) * 0.95) as usize] as f64;
            let max = *v.last().unwrap() as f64;
            eprintln!(
                "{rocket} {label}: mean={:.1}µs p50={:.1}µs p95={:.1}µs max={:.1}µs (n={})",
                mean / 1e3,
                p50 / 1e3,
                p95 / 1e3,
                max / 1e3,
                v.len()
            );
        };

        eprintln!(
            "{rocket} phases (mean over {N}): control={:.1}µs env={:.1}µs asm.step={:.1}µs pad+crash+advance={:.1}µs",
            phase.control_ns as f64 / N as f64 / 1e3,
            phase.env_ns as f64 / N as f64 / 1e3,
            phase.asm_ns as f64 / N as f64 / 1e3,
            phase.pad_crash_ns as f64 / N as f64 / 1e3,
        );
        report("phased(no slice)", &mut samples_us);
        report("full tick(+slice)", &mut full_us);
        eprintln!(
            "{rocket} budget: sim_dt=20ms → headroom ≈ {:.1}% if wall≈p95 full",
            {
                full_us.sort_unstable();
                let p95 = full_us[((full_us.len() as f64) * 0.95) as usize] as f64 / 1e6; // ms
                (1.0 - p95 / 20.0) * 100.0
            }
        );
    }
}

#[derive(Default)]
struct PhaseAccum {
    control_ns: u64,
    env_ns: u64,
    asm_ns: u64,
    pad_crash_ns: u64,
}
