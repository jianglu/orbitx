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
