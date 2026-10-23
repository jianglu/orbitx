use crate::cli::{test_runtime_args, ControlKindArg, SessionControl};
use crate::runtime::clock::Clock;
use crate::runtime::tick;
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
        let _ = tick::tick(&mut clock, &mut sim);
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
        let _ = tick::tick(&mut clock, &mut sim);
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
    let slice = match tick::tick(&mut clock, &mut sim2) {
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

    let args = test_runtime_args("falcon9");
    let session = args.load_session().unwrap();
    let mut sim = build_sim_bundle(&session, &args.ephemeris_data).expect("bundle");
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
