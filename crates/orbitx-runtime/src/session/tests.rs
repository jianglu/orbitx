use crate::cli::{test_runtime_args, ControlKindArg, SessionControl};
use crate::runtime::clock::Clock;
use crate::runtime::tick::{self, TickOutcome};
use crate::session::build_sim_bundle;

const LP_TWO_PADS: &str = r#"
schema = 2
name = "Test Cape"
body = "Earth"
latitude_deg = 28.5
longitude_deg = -80.5
base_elevation_m = 1.4

[[pads]]
name = "A"

[[pads]]
name = "B"
axis_offset_m = [500.0, 0.0, 0.0]
heading_deg = 45.0
"#;

fn write_lp(dir: &std::path::Path, body: &str) -> std::path::PathBuf {
    let path = dir.join("lp_test.toml");
    std::fs::write(&path, body).unwrap();
    path
}

#[test]
fn wenchang_idle_then_ramp_liftoff_no_false_landed() {
    use crate::runtime::tick::TickOutcome;
    // 复现用户交互：文昌怠速 5 s → ↑ 渐进拉满油门 → 观察是否误判着陆。
    let rocket = std::path::PathBuf::from(std::env::var("HOME").unwrap())
        .join("Library/Application Support/WLCY/SimRocket/spacecraft/sc_b827cb4b/sim.toml");
    if !rocket.is_file() {
        eprintln!("skip repro: user rocket missing");
        return;
    }
    let pkg = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../../simrocket_earth/packages/medium");
    let mut args = test_runtime_args("falcon9");
    args.rocket = rocket.display().to_string();
    args.surface_package = Some(pkg);
    let session = args.load_session().unwrap();
    let mut sim = build_sim_bundle(&session, &args.ephemeris_data).expect("bundle");
    let mut clock = Clock::new(20);

    // 怠速 5 s（250 tick @20 ms）
    for _ in 0..250 {
        let _ = tick::tick(&mut clock, &mut sim, false);
    }
    eprintln!(
        "after idle: clearance={:?} launched={} alt_slice={:.2}",
        sim.asm.bottom_clearance(),
        sim.pad.launched,
        sim.last_slice.focus.altitude
    );

    // 油门 3 s 线性拉满（150 tick），其后保持，总时长到 T+20 s
    for i in 0..750u64 {
        if i < 150 {
            let level = (i as f64 + 1.0) / 150.0;
            sim.control.apply_input(
                &mut sim.asm,
                crate::input::InputCmd::SetThrottle { level },
            );
        }
        let _ = tick::tick(&mut clock, &mut sim, false);
        if i % 25 == 0 || !sim.landed_msg.is_empty() || !sim.crash_msg.is_empty() {
            let bp = sim.asm.primary_bottom_point().unwrap();
            let (lat, lon) = orbitx_vessel::geocentric_lat_lon_deg_orient(
                sim.asm.state.pos,
                sim.asm.planet_orientation.rot,
            );
            eprintln!(
                "t={:.2} thr_i={i} clearance={:?} bottom_r={:.1} lat={lat:.5} lon={lon:.5} launched={} airborne={} landed={:?} crash={:?} failed={:?}",
                clock.sim_t_ms() as f64 / 1000.0,
                sim.asm.bottom_clearance(),
                bp.length() - sim.earth_radius,
                sim.pad.launched,
                sim.was_airborne,
                sim.landed_msg,
                sim.crash_msg,
                sim.launch_failed_msg,
            );
        }
        if !sim.landed_msg.is_empty() || !sim.crash_msg.is_empty() {
            break;
        }
        let _ = TickOutcome::Stepped { slice: sim.last_slice.clone() };
    }
    assert!(sim.landed_msg.is_empty(), "false landed: {}", sim.landed_msg);
    assert!(sim.crash_msg.is_empty(), "false crash: {}", sim.crash_msg);
    assert!(sim.launch_failed_msg.is_empty(), "false launch_failed: {}", sim.launch_failed_msg);
    assert!(sim.pad.launched);
    // 爬升脱离后应解锁离地（底部相对地形间隙曾 >1 m）
    assert!(sim.was_airborne, "should have become airborne after climb-out");
}

#[test]
fn launchpad_places_at_pad_a_by_default() {
    let dir = tempfile::tempdir().unwrap();
    let mut args = test_runtime_args("falcon9");
    args.launchpad = Some(write_lp(dir.path(), LP_TWO_PADS));
    let session = args.load_session().unwrap();
    let sim = build_sim_bundle(&session, &args.ephemeris_data).expect("bundle");
    let pos = sim.asm.state.pos;
    let (lat, lon) = orbitx_vessel::geocentric_lat_lon_deg_orient(pos, sim.asm.planet_orientation.rot);
    assert!((lat - 28.5).abs() < 1e-6, "lat {lat}");
    assert!((lon - -80.5).abs() < 1e-6, "lon {lon}");
    // 质心相对地面高度应与默认摆放一致（同一装配原点偏移）
    let alt = pos.length() - sim.earth_radius;
    let args_d = test_runtime_args("falcon9");
    let session_d = args_d.load_session().unwrap();
    let sim_d = build_sim_bundle(&session_d, &args_d.ephemeris_data).expect("bundle");
    let alt_d = sim_d.asm.state.pos.length() - sim_d.earth_radius;
    assert!(
        (alt - alt_d).abs() < 1e-3,
        "alt {alt} vs default-placement alt {alt_d}"
    );
}

#[test]
fn launchpad_pad_b_offset_and_heading_override() {
    let dir = tempfile::tempdir().unwrap();
    let mut args = test_runtime_args("falcon9");
    args.launchpad = Some(write_lp(dir.path(), LP_TWO_PADS));
    args.pad = Some("B".into());
    let session = args.load_session().unwrap();
    assert_eq!(session.launchpad.as_ref().unwrap().heading_deg, 45.0);
    let sim = build_sim_bundle(&session, &args.ephemeris_data).expect("bundle");
    let (lat, lon) = orbitx_vessel::geocentric_lat_lon_deg_orient(
        sim.asm.state.pos,
        sim.asm.planet_orientation.rot,
    );
    // 东移 500 m：经度增大、纬度近不变
    assert!(lon > -80.5 && lon < -80.49, "lon {lon}");
    assert!((lat - 28.5).abs() < 1e-3, "lat {lat}");

    // --heading-deg 覆盖台位朝向
    let mut args2 = test_runtime_args("falcon9");
    args2.launchpad = Some(write_lp(dir.path(), LP_TWO_PADS));
    args2.pad = Some("B".into());
    args2.heading_deg = Some(10.0);
    let session2 = args2.load_session().unwrap();
    assert_eq!(session2.launchpad.unwrap().heading_deg, 10.0);
}

#[test]
fn default_launchpad_is_wenchang_and_land() {
    // test_runtime_args 默认指向 presets/lp_wenchang.toml（海南文昌）。
    let args = test_runtime_args("falcon9");
    let session = args.load_session().unwrap();
    let lp = session.launchpad.as_ref().unwrap();
    assert_eq!(lp.config.name, "海南文昌发射中心");
    let sim = build_sim_bundle(&session, &args.ephemeris_data).expect("bundle");
    let (lat, lon) = orbitx_vessel::geocentric_lat_lon_deg_orient(
        sim.asm.state.pos,
        sim.asm.planet_orientation.rot,
    );
    assert!((lat - 19.614).abs() < 1e-3, "lat {lat}");
    assert!((lon - 110.930).abs() < 1e-3, "lon {lon}");

    // 带地表包（默认档 medium，缺失回退 low）时文昌点须为陆地。
    let root = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../../simrocket_earth/packages");
    let pkg = ["medium", "low"]
        .iter()
        .map(|t| root.join(t))
        .find(|p| p.join("manifest.toml").is_file());
    let Some(pkg) = pkg else {
        eprintln!("skip wenchang land check: no surface package");
        return;
    };
    let mut args2 = test_runtime_args("falcon9");
    args2.surface_package = Some(pkg);
    let session2 = args2.load_session().unwrap();
    let sim2 = build_sim_bundle(&session2, &args2.ephemeris_data).expect("bundle");
    let contact = sim2.surface.as_ref().unwrap().placement_contact();
    assert_eq!(
        contact.surface_kind_at_world(sim2.asm.state.pos),
        Some(orbitx_vessel::SurfaceKind::Land),
        "wenchang must be land in surface package"
    );
}

#[test]
fn first_slice_altitude_is_bottom_elevation() {
    use crate::runtime::tick::TickOutcome;
    // 文昌默认（Falcon 9 多级组合）：底部归零后底部落在支持面。
    let args = test_runtime_args("falcon9");
    let session = args.load_session().unwrap();
    let sim = build_sim_bundle(&session, &args.ephemeris_data).expect("bundle");
    let bottom = sim.asm.primary_bottom_point().unwrap();
    assert!(
        (bottom.length() - sim.pad_support_radius_m).abs() < 0.1,
        "bottom on support: |bottom| {} vs {}",
        bottom.length(),
        sim.pad_support_radius_m
    );

    // 带地表包（生产口径）首帧 Slice 高度 = 底部海拔 ≈ 场坪海拔 1.4 + 台架高 0。
    let root = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../../simrocket_earth/packages");
    let pkg = ["medium", "low"]
        .iter()
        .map(|t| root.join(t))
        .find(|p| p.join("manifest.toml").is_file());
    let Some(pkg) = pkg else {
        eprintln!("skip first-slice altitude: no surface package");
        return;
    };
    let mut args2 = test_runtime_args("falcon9");
    args2.surface_package = Some(pkg);
    let session2 = args2.load_session().unwrap();
    let mut sim2 = build_sim_bundle(&session2, &args2.ephemeris_data).expect("bundle");
    let mut clock = Clock::new(20);
    let slice = match tick::tick(&mut clock, &mut sim2, false) {
        TickOutcome::Stepped { slice } => slice,
        TickOutcome::Skipped => panic!("expected stepped"),
    };
    assert!(
        (slice.focus.altitude - 1.4).abs() < 0.5,
        "pad focus altitude (elevation) {}",
        slice.focus.altitude
    );
    assert!(
        (slice.launchpad.alt_m - 1.4).abs() < 0.5,
        "pad launchpad alt (elevation) {}",
        slice.launchpad.alt_m
    );
}

#[test]
fn pad_mount_height_offsets_support() {
    // 台位 axis_offset_m[2] = 台架高度 25 m：底部落在 R + base + 25 支持面。
    let dir = tempfile::tempdir().unwrap();
    let lp = LP_TWO_PADS.replace(
        "axis_offset_m = [500.0, 0.0, 0.0]",
        "axis_offset_m = [500.0, 0.0, 25.0]",
    );
    let mut args = test_runtime_args("falcon9");
    args.launchpad = Some(write_lp(dir.path(), &lp));
    args.pad = Some("B".into());
    let session = args.load_session().unwrap();
    let sim = build_sim_bundle(&session, &args.ephemeris_data).expect("bundle");
    let bottom = sim.asm.primary_bottom_point().unwrap();
    let expect = sim.earth_radius + 1.4 + 25.0;
    assert!(
        (bottom.length() - expect).abs() < 0.1,
        "mounted bottom |p| {} vs support {expect}",
        bottom.length()
    );
    assert!((sim.pad_support_radius_m - expect).abs() < 1e-9);
}

#[test]
fn non_primary_launchpad_bundle_rejected() {
    let dir = tempfile::tempdir().unwrap();
    let scn = dir.path().join("scenario_test.toml");
    std::fs::write(
        &scn,
        r#"
name = "TestSys"
mjd = 51544.5
primary = "Earth"

[[bodies]]
name = "Earth"
mass = 5.973698968e24
size = 6371010.0

[[bodies]]
name = "Moon"
mass = 7.34767309e22
size = 1737100.0
dynamics = false
fixed_pos = [3.844e8, 0.0, 0.0]
"#,
    )
    .unwrap();
    let mut args = test_runtime_args("falcon9");
    args.scenario = scn.display().to_string();
    args.launchpad = Some(write_lp(
        dir.path(),
        "schema=2\nname='M'\nbody='Moon'\nlatitude_deg=0.0\nlongitude_deg=0.0\nbase_elevation_m=0.0\n[[pads]]\nname='P'\n",
    ));
    let session = args.load_session().unwrap();
    let err = match build_sim_bundle(&session, &args.ephemeris_data) {
        Ok(_) => panic!("expected non-primary launchpad rejection"),
        Err(e) => e,
    };
    assert!(err.contains("非主天体"), "{err}");
    assert!(err.contains("Moon"), "{err}");
}

#[test]
fn launchpad_with_override_uses_composite_ground() {
    let pkg = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../../simrocket_earth/packages/low");
    if !pkg.join("manifest.toml").is_file() {
        eprintln!("skip launchpad_with_override: packages/low missing");
        return;
    }
    let dir = tempfile::tempdir().unwrap();
    // 网格 11×11 @100 m 全 120 m；台位在原点（印度洋全球接触高 ≈ 0）
    let mut bytes = Vec::new();
    for _ in 0..121 {
        bytes.extend_from_slice(&120.0f32.to_le_bytes());
    }
    std::fs::write(dir.path().join("grid.f32"), bytes).unwrap();
    let lp = r#"
schema = 2
name = "Ocean Pad"
body = "Earth"
latitude_deg = 0.0
longitude_deg = 90.0
base_elevation_m = 120.0
elevation_override = "grid.f32"
grid_cell_m = 100.0
grid_width = 11
grid_height = 11

[[pads]]
name = "P1"
"#;
    let mut args = test_runtime_args("falcon9");
    args.launchpad = Some(write_lp(dir.path(), lp));
    args.surface_package = Some(pkg);
    let session = args.load_session().unwrap();
    let sim = build_sim_bundle(&session, &args.ephemeris_data).expect("bundle");
    let sess = sim.surface.as_ref().expect("surface session");
    assert!(sess.has_override());
    let contact = sess.placement_contact();
    let pos = sim.asm.state.pos;
    let r_ground = contact.radius_at_world(pos).unwrap();
    // 合成地面半径应 = R_elev + 120（覆盖台面，base=120 截平 no-op）
    assert!(
        (r_ground - (sess.elev_radius_m + 120.0)).abs() < 1e-3,
        "composite ground radius {r_ground}"
    );
    // 底部归零：主组合体底部应落在支持面（= 合成地面，台架高 0），间隙 ≈ 0
    let bottom = sim.asm.primary_bottom_point().unwrap();
    let clearance = bottom.length() - contact.radius_at_world(bottom).unwrap();
    assert!(clearance.abs() < 0.1, "bottom clearance {clearance}");
    // 对照默认文昌（无包）：底部落在 R + base(1.4) 支持面，间隙同样 ≈ 0
    let args_d = test_runtime_args("falcon9");
    let session_d = args_d.load_session().unwrap();
    let sim_d = build_sim_bundle(&session_d, &args_d.ephemeris_data).expect("bundle");
    let bottom_d = sim_d.asm.primary_bottom_point().unwrap();
    let support_d = sim_d.earth_radius + 1.4;
    assert!(
        (bottom_d.length() - support_d).abs() < 0.1,
        "default bottom |p| {} vs support {support_d}",
        bottom_d.length()
    );
    // 台面在足迹内 → 陆地（全球包此处为海洋）
    assert_eq!(
        contact.surface_kind_at_world(pos),
        Some(orbitx_vessel::SurfaceKind::Land)
    );
}

#[test]
fn builds_falcon9_bundle() {
    let args = test_runtime_args("falcon9");
    let session = args.load_session().unwrap();
    let sim = build_sim_bundle(&session, &args.ephemeris_data).expect("bundle");
    assert_eq!(sim.world.rocket_class, "Falcon9");
    assert!(sim.asm.vessels.len() >= 2);
    assert!(matches!(
        session.control,
        SessionControl::Control(ControlKindArg::Target)
    ));
}

#[test]
fn target_climb_increases_altitude() {
    let args = test_runtime_args("falcon9");
    let session = args.load_session().unwrap();
    let mut sim = build_sim_bundle(&session, &args.ephemeris_data).expect("bundle");
    sim.control.apply_input(
        &mut sim.asm,
        crate::input::InputCmd::SetThrottle { level: 1.0 },
    );
    let mut clock = Clock::new(20);
    let alt0 = sim.asm.state.pos.length() - sim.earth_radius;

    for _ in 0..500 {
        let _ = tick::tick(&mut clock, &mut sim, false);
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

    let args = test_runtime_args("falcon9");
    let session = args.load_session().unwrap();
    let mut sim = build_sim_bundle(&session, &args.ephemeris_data).expect("bundle");
    let mut clock = Clock::new(20);

    let intact = match tick::tick(&mut clock, &mut sim, false) {
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

    let after = match tick::tick(&mut clock, &mut sim, false) {
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

    let args = test_runtime_args("falcon9");
    let session = args.load_session().unwrap();
    let mut sim = build_sim_bundle(&session, &args.ephemeris_data).expect("bundle");

    let names: Vec<&str> = sim
        .stage_display_order
        .iter()
        .map(|&i| sim.asm.vessels[i].name.as_str())
        .collect();
    assert_eq!(names, vec!["Payload", "F9-S2", "F9-S1"]);

    let mut clock = Clock::new(20);
    let slice = match tick::tick(&mut clock, &mut sim, false) {
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
    let args = test_runtime_args("lm2f");
    let session = args.load_session().unwrap();
    let sim = build_sim_bundle(&session, &args.ephemeris_data).expect("bundle");

    let names: Vec<&str> = sim
        .stage_display_order
        .iter()
        .map(|&i| sim.asm.vessels[i].name.as_str())
        .collect();
    assert!(names.len() >= 7, "expected 7 stages, got {names:?}");
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
        let args = test_runtime_args(rocket);
        let session = args.load_session().unwrap();
        let mut sim = build_sim_bundle(&session, &args.ephemeris_data).expect("bundle");
        sim.control.apply_input(
            &mut sim.asm,
            crate::input::InputCmd::SetThrottle { level: 1.0 },
        );
        let mut clock = Clock::new(20);

        // 暖机直到起飞。
        let mut launched = false;
        for _ in 0..5_000 {
            let _ = tick::tick(&mut clock, &mut sim, false);
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

            let a = Instant::now();
            sim.control.tick(&mut sim.asm, &sim.last_slice, dt);
            phase.control_ns += a.elapsed().as_nanos() as u64;

            let a = Instant::now();
            sim.world.psys.update();
            let surface = sim.world.psys.primary_surface();
            sim.earth_radius = surface.radius;
            sim.asm.planet_radius = surface.radius;
            sim.asm.sid_rot_period = surface.sid_rot_period;
            sim.asm.planet_orientation = surface.orientation;
            let grav = sim.world.psys.grav_bodies();
            let primary = sim.world.psys.primary_grav_index();
            phase.env_ns += a.elapsed().as_nanos() as u64;

            let a = Instant::now();
            sim.asm
                .step(dt, orbitx_vessel::StepEnv::new(&grav, primary));
            phase.asm_ns += a.elapsed().as_nanos() as u64;

            let a = Instant::now();
            let thr_level = crate::pad::max_thrusting_level(&sim.asm);
            let eff_thrust = crate::pad::effective_thrust(&sim.asm);
            crate::pad::after_step(&mut sim.asm, &mut sim.pad, thr_level, eff_thrust);
            let _ = crate::crash::apply_contact_checks(
                &mut sim.asm,
                sim.pad.launched,
                false,
                false,
                eff_thrust,
                &mut sim.land_dwell,
                &mut sim.landed,
                clock.sim_t_ms() as f64 / 1000.0,
            );
            if sim.landed.active {
                crate::crash::apply_landed_nail(&mut sim.asm, &sim.landed);
            }
            sim.world.psys.advance(dt / 86_400.0);
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
            match tick::tick(&mut clock, &mut sim, false) {
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

#[test]
fn build_slice_fills_solar_orientation_and_rot() {
    let args = test_runtime_args("lm7");
    let session = args.load_session().unwrap();
    let mut sim = build_sim_bundle(&session, &args.ephemeris_data).expect("bundle");
    let mut clock = Clock::new(20);
    let slice = match tick::tick(&mut clock, &mut sim, false) {
        TickOutcome::Stepped { slice } => slice,
        TickOutcome::Skipped => panic!("expected stepped"),
    };
    let sun_n = (slice.sun_direction[0].powi(2)
        + slice.sun_direction[1].powi(2)
        + slice.sun_direction[2].powi(2))
    .sqrt();
    assert!(
        (sun_n - 1.0).abs() < 1e-9,
        "sun_direction not unit: {sun_n}"
    );
    let focus = slice.focus.pos;
    let rel = slice.earth_position_rel_focus;
    assert!(
        (rel[0] + focus[0]).abs() < 1e-6
            && (rel[1] + focus[1]).abs() < 1e-6
            && (rel[2] + focus[2]).abs() < 1e-6,
        "earth_position_rel_focus should be -focus.pos"
    );
    let pr = slice.planet_rot;
    assert_eq!(pr.len(), 9);
    // 正交：列范数 ≈ 1
    for c in 0..3 {
        let n = (pr[c].powi(2) + pr[3 + c].powi(2) + pr[6 + c].powi(2)).sqrt();
        assert!((n - 1.0).abs() < 1e-6, "planet_rot col {c} norm {n}");
    }
    let flat = [
        sim.asm.state.r.m11,
        sim.asm.state.r.m12,
        sim.asm.state.r.m13,
        sim.asm.state.r.m21,
        sim.asm.state.r.m22,
        sim.asm.state.r.m23,
        sim.asm.state.r.m31,
        sim.asm.state.r.m32,
        sim.asm.state.r.m33,
    ];
    for i in 0..9 {
        assert!(
            (slice.focus.rot[i] - flat[i]).abs() < 1e-12,
            "focus.rot[{i}] mismatch"
        );
    }
    assert!((slice.earth_obliquity_deg - 23.44).abs() < 0.1);
}

#[derive(Default)]
struct PhaseAccum {
    control_ns: u64,
    env_ns: u64,
    asm_ns: u64,
    pad_crash_ns: u64,
}

/// 发射门禁 e2e：hold 期 gate_allows 只放行首步（状态冻结）；Launch 解禁后
/// 满 油门应点火 → 台架释放 → 爬升。用户船 sc_b827cb4b（缺文件则 skip）。
#[test]
fn launch_gate_holds_then_releases_on_launch() {
    use crate::input::InputCmd;
    use crate::runtime::gate_allows;

    let rocket = std::path::PathBuf::from(std::env::var("HOME").unwrap())
        .join("Library/Application Support/WLCY/SimRocket/spacecraft/sc_b827cb4b/sim.toml");
    if !rocket.is_file() {
        eprintln!("skip repro: user rocket missing");
        return;
    }
    let pkg = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../../simrocket_earth/packages/medium");
    let mut args = test_runtime_args("falcon9");
    args.rocket = rocket.display().to_string();
    args.surface_package = Some(pkg);
    let session = args.load_session().unwrap();
    let mut sim = build_sim_bundle(&session, &args.ephemeris_data).expect("bundle");
    let mut clock = Clock::new(20);

    // hold 期：client-step 持续发 Step，gate 只放行首步。
    let mut gate_open = false;
    let mut stepped_once = false;
    let mut steps = 0;
    for _ in 0..20 {
        if gate_allows(gate_open, &mut stepped_once) {
            steps += 1;
        }
    }
    assert_eq!(steps, 1, "hold 期只放行首步");
    for _ in 0..steps {
        tick::tick(&mut clock, &mut sim, gate_open);
    }
    let hold_step = sim.last_slice.step_index;
    assert!(!sim.pad.launched, "hold 期不应离台");
    assert!(!sim.last_slice.stepping, "hold 期切片 stepping=false");

    // Launch：gate 解禁 → 满油门 3 s（150 tick @20 ms）。
    gate_open = true;
    for _ in 0..150 {
        sim.control
            .apply_input(&mut sim.asm, InputCmd::SetThrottle { level: 1.0 });
        tick::tick(&mut clock, &mut sim, gate_open);
    }
    let alt = sim.last_slice.focus.altitude;
    eprintln!(
        "after launch: hold_step={hold_step} step={} alt={alt:.2} launched={} twr={:.2}",
        sim.last_slice.step_index, sim.pad.launched, sim.last_slice.focus.twr
    );
    assert!(sim.last_slice.focus.twr > 1.0, "满油门 twr 应 >1（船自身 TWR 足）");
    assert!(sim.pad.launched, "有效推力超门槛应释放台架");
    assert!(alt > 10.0, "应已爬升（alt={alt:.2}）");
    assert!(sim.last_slice.stepping, "Launch 后切片 stepping=true");
}

/// 体固经纬（切片/HUD 同口径）：船惯性位 × 当前体固→惯性 rot。
fn pad_test_body_lat_lon(sim: &crate::session::SimBundle) -> (f64, f64) {
    use orbitx_vessel::geocentric_lat_lon_deg_orient;
    geocentric_lat_lon_deg_orient(sim.asm.state.pos, sim.asm.planet_orientation.rot)
}

/// 共转判责（2026-10-26「起飞后相对地面横向移动」）：
/// ① 钉台期体固经纬恒定（钉点随地球转）；
/// ② 离台初速 = 当地共转速度（ω·r·cosλ，文昌 ≈438 m/s）；
/// ③ 释放后无推力抛体 60 s，体固经纬漂移仅为离心-引力纬向不平衡
///    （文昌理论 ~1e-4 °/min），与「地球转、船不转」的 bug 级横漂
///    （≈4°/min，438 m/s）相差 4 个量级——超阈即 orbitx 步进丢共转。
#[test]
fn pad_release_co_rotates_no_lateral_drift() {
    let mut args = test_runtime_args("falcon9");
    args.surface_package = None; // 球形地面：CI 无地表包可跑
    let session = args.load_session().unwrap();
    let mut sim = build_sim_bundle(&session, &args.ephemeris_data).expect("bundle");
    let mut clock = Clock::new(20);

    let (lat0, lon0) = pad_test_body_lat_lon(&sim);

    // ① 钉台 5 s：体固经纬必须恒定。
    for _ in 0..250 {
        let _ = tick::tick(&mut clock, &mut sim, false);
    }
    let (lat_h, lon_h) = pad_test_body_lat_lon(&sim);
    assert!((lat_h - lat0).abs() < 1e-6, "钉台期纬度漂移 {}", (lat_h - lat0).abs());
    assert!((lon_h - lon0).abs() < 1e-6, "钉台期经度漂移 {}", (lon_h - lon0).abs());

    // ② 共转初速。
    let v0 = sim.asm.state.vel.length();
    let v_exp = 7.292_115e-5_f64 * sim.asm.state.pos.length() * lat0.to_radians().cos();
    assert!(
        (v0 - v_exp).abs() / v_exp < 0.02,
        "共转初速偏差: v0={v0:.2} expect={v_exp:.2}"
    );

    // ③ 释放 + 300 m/s 上抛（滞空 ~60 s 全程空中验证），无推力抛体。
    sim.pad.launched = true;
    let up = sim.asm.state.pos.unit();
    sim.asm.state.vel = sim.asm.state.vel + up * 300.0;
    let mut lon_max = 0.0_f64;
    let mut lat_max = 0.0_f64;
    let mut lo_end = 0.0_f64;
    let mut la_end = 0.0_f64;
    for _ in 0..3000 {
        let _ = tick::tick(&mut clock, &mut sim, true);
        if clock.paused() {
            break; // 坠毁终局：位置冻结，漂移上限仍有效
        }
        let (la, lo) = pad_test_body_lat_lon(&sim);
        lon_max = lon_max.max((lo - lon0).abs());
        lat_max = lat_max.max((la - lat0).abs());
        lo_end = lo;
        la_end = la;
    }
    assert!(
        lon_max < 0.02,
        "释放后经度漂移 {:+.5}°（≈{:.0} m）→ orbitx 步进丢共转",
        (lo_end - lon0),
        (lo_end - lon0).abs() * 111_320.0 * lat0.to_radians().cos()
    );
    assert!(lat_max < 0.02, "释放后纬度漂移 {:.5}°", lat_max);
    eprintln!(
        "co-rot diag: lon_drift={:+.6}° lat_drift={:+.6}° omega={:?} v0={v0:.2}",
        lo_end - lon0,
        la_end - lat0,
        sim.asm.planet_orientation.omega,
    );
}

/// 并联助推 TOML（新 exporter dock dir 语义）：三枚助推挂芯级**下半部**
/// 侧向（B 口在自身质心高度、S1 口在 y=−3 侧面 → 质心连线斜向），dock
/// 口方向为贴合面法线（⊥ 纵轴、两口反向共线，Orbiter approach direction）。
const PARALLEL_ROCKET_TOML: &str = r#"
name = "parallel-test"
class = "parallel_test"

[[stages]]
name = "S1"
dry_mass = 60000.0
dry_center = [0.0, 0.0, 0.0]
dry_inertia = [2000000.0, 6000000.0, 2000000.0]
length = 20.0
radius = 2.0
separation_impulse = 4.0

[[stages.tanks]]
id = 0
max_mass = 120000.0
mass = 120000.0
pos = [0.0, 0.0, 0.0]
inertia = [1000000.0, 3000000.0, 1000000.0]
efficiency = 1.0

[[stages.thrusters]]
pos = [0.0, -10.0, 0.0]
dir = [0.0, 1.0, 0.0]
thrust = 900000.0
isp = 300.0
thrust_sl = 900000.0
isp_sl = 280.0

[[stages.docks]]
pos = [2.0, -3.0, 0.0]
dir = [1.0, 0.0, 0.0]
rot = [0.0, 0.0, 1.0]

[[stages.docks]]
pos = [-2.0, -3.0, 0.0]
dir = [-1.0, 0.0, 0.0]
rot = [0.0, 0.0, 1.0]

[[stages.docks]]
pos = [0.0, -3.0, 2.0]
dir = [0.0, 0.0, 1.0]
rot = [1.0, 0.0, 0.0]

[[stages]]
name = "B1"
dry_mass = 12000.0
dry_center = [0.0, 0.0, 0.0]
dry_inertia = [300000.0, 900000.0, 300000.0]
length = 12.0
radius = 1.5
separation_impulse = 4.0

[[stages.tanks]]
id = 0
max_mass = 60000.0
mass = 60000.0
pos = [0.0, 0.0, 0.0]
inertia = [200000.0, 600000.0, 200000.0]
efficiency = 1.0

[[stages.thrusters]]
pos = [0.0, -6.0, 0.0]
dir = [0.0, 1.0, 0.0]
thrust = 1500000.0
isp = 290.0
thrust_sl = 1500000.0
isp_sl = 270.0

[[stages.docks]]
pos = [-1.5, 0.0, 0.0]
dir = [-1.0, 0.0, 0.0]
rot = [0.0, 0.0, 1.0]

[[stages]]
name = "B2"
dry_mass = 12000.0
dry_center = [0.0, 0.0, 0.0]
dry_inertia = [300000.0, 900000.0, 300000.0]
length = 12.0
radius = 1.5
separation_impulse = 4.0

[[stages.tanks]]
id = 0
max_mass = 60000.0
mass = 60000.0
pos = [0.0, 0.0, 0.0]
inertia = [200000.0, 600000.0, 200000.0]
efficiency = 1.0

[[stages.thrusters]]
pos = [0.0, -6.0, 0.0]
dir = [0.0, 1.0, 0.0]
thrust = 1500000.0
isp = 290.0
thrust_sl = 1500000.0
isp_sl = 270.0

[[stages.docks]]
pos = [1.5, 0.0, 0.0]
dir = [1.0, 0.0, 0.0]
rot = [0.0, 0.0, 1.0]

[[stages]]
name = "B3"
dry_mass = 12000.0
dry_center = [0.0, 0.0, 0.0]
dry_inertia = [300000.0, 900000.0, 300000.0]
length = 12.0
radius = 1.5
separation_impulse = 4.0

[[stages.tanks]]
id = 0
max_mass = 60000.0
mass = 60000.0
pos = [0.0, 0.0, 0.0]
inertia = [200000.0, 600000.0, 200000.0]
efficiency = 1.0

[[stages.thrusters]]
pos = [0.0, -6.0, 0.0]
dir = [0.0, 1.0, 0.0]
thrust = 1500000.0
isp = 290.0
thrust_sl = 1500000.0
isp_sl = 270.0

[[stages.docks]]
pos = [0.0, 0.0, -1.5]
dir = [0.0, 0.0, -1.0]
rot = [1.0, 0.0, 0.0]

[[dock_links]]
stage = 0
port = 0
remote_stage = 1
remote_port = 0

[[dock_links]]
stage = 0
port = 1
remote_stage = 2
remote_port = 0

[[dock_links]]
stage = 0
port = 2
remote_stage = 3
remote_port = 0
"#;

/// 并联助推点火全链回归（2026-10-26「侧挂助推没有推力」）：
/// exporter 的径向 dock 口方向已改为贴合面法线（纯径向进近方向，对齐
/// Orbiter approach direction）——本测试以新语义 TOML 验证：
/// ① 下部挂接助推被识别为侧挂叶并进 lit 集；
/// ② SyncPrimary（target 档油门策略）点亮芯级 + 全部侧挂叶；
/// ③ 台架在点火 10 s 窗口内释放并爬升（修复前 TWR 0.25 < 1.05，
///    触发「点火 10 s 推力未超过重量的 1.05 倍」发射失败）。
#[test]
fn parallel_boosters_ignite_and_release_pad() {
    use crate::input::InputCmd;
    use orbitx_vessel::ThrustReadout;

    let dir = tempfile::tempdir().unwrap();
    let rocket_path = dir.path().join("rocket_parallel.toml");
    std::fs::write(&rocket_path, PARALLEL_ROCKET_TOML).unwrap();

    let mut args = test_runtime_args(rocket_path.to_str().unwrap());
    args.surface_package = None; // 球形地面：CI 无地表包可跑
    let session = args.load_session().unwrap();
    let mut sim = build_sim_bundle(&session, &args.ephemeris_data).expect("bundle");
    let mut clock = Clock::new(20);

    // ① 侧挂叶识别（下部挂接几何 + 纯径向 dock dir）。
    let strap: Vec<usize> = sim.asm.strap_on_leaf_indices().map(|(i, _)| i).collect();
    assert_eq!(strap.len(), 3, "strap-on leaves: {strap:?}");
    let lit: Vec<usize> = sim.asm.lit_thrusting_indices().collect();
    for b in &strap {
        assert!(lit.contains(b), "booster {b} not in lit set: {lit:?}");
    }

    // ② SyncPrimary 点火（target 档油门策略，真实会话路径）：油门在
    // controller tick 时落盘，先步进两拍再断言。
    sim.control
        .apply_input(&mut sim.asm, InputCmd::SetThrottle { level: 1.0 });
    for _ in 0..2 {
        let _ = tick::tick(&mut clock, &mut sim, true);
    }
    for (vi, _) in sim.asm.strap_on_leaf_indices() {
        let lv = sim.asm.vessels[vi].thrusters[0].level;
        assert!(lv > 0.99, "booster {vi} throttle {lv}");
    }
    let core_lv = sim.asm.vessels[sim.asm.active].thrusters[0].level;
    assert!(core_lv > 0.99, "core throttle {core_lv}");

    // ③ 台架释放：点火 10 s 窗口内（20 ms 步 → 500 步内）。
    let r0 = sim.asm.state.pos.length();
    let mut launched_at = None;
    for step in 0..500 {
        let _ = tick::tick(&mut clock, &mut sim, true);
        if !sim.launch_failed_msg.is_empty() {
            panic!("发射失败: {}", sim.launch_failed_msg);
        }
        if sim.pad.launched {
            launched_at = Some(step);
            break;
        }
    }
    let step = launched_at.unwrap_or_else(|| panic!("点火 10 s 内未释放台架"));
    for _ in 0..200 {
        let _ = tick::tick(&mut clock, &mut sim, true);
    }
    let alt_gain = sim.asm.state.pos.length() - r0;
    // 4 s 爬升：TWR≈1.39 → 净加速 ~3.8 m/s²，量级 ~30 m；下限只要求
    // 「确实离台爬升、未坠回」。
    assert!(alt_gain > 10.0, "释放后未爬升: {alt_gain:.1} m");
    eprintln!("parallel diag: launched@step={step} alt_gain={alt_gain:.1} m");
}
