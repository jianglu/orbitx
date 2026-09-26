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
