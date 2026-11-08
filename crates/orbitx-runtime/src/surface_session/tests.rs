use super::*;
use orbitx_math::dot;
use orbitx_vessel::GroundContact;

fn pkg_low() -> Option<PathBuf> {
    let p = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../../simrocket_earth/packages/physical");
    if p.join("manifest.toml").is_file() {
        Some(p)
    } else {
        None
    }
}

#[test]
fn three_states_and_contact_height() {
    let Some(root) = pkg_low() else {
        eprintln!("skip three_states_and_contact_height: packages/physical missing");
        return;
    };
    let elev = 6_371_010.0;
    let cfg = SurfaceSessionConfig {
        package_root: root,
        working_max_level: 0,
        prefetch_every: 1,
        inject_every: 1,
        ..Default::default()
    };
    // Match package R_elev
    let mut sess = SurfaceSession::open(cfg, elev).unwrap();
    assert!(!sess.workset_ready);

    // Near surface, outward radial vel small → stay Active; first tick prefetches.
    let pos = Vec3::new(elev + 100.0, 0.0, 0.0);
    let vel = Vec3::new(-10.0, 0.0, 0.0); // inward
    sess.on_tick(pos, vel, Matrix3::IDENTITY);
    assert_eq!(sess.schedule, ScheduleState::Active);
    assert!(sess.workset_ready);
    let g = sess.inject();
    assert!(
        matches!(g, GroundContact::Query { .. }),
        "expected Query after prefetch, got {g:?}"
    );
    let (lat, lon) = geocentric_lat_lon_deg_orient(pos, Matrix3::IDENTITY);
    let r_query = g.radius_at_world(pos).unwrap();
    let expect = elev + sess.package.sample_contact_height_at(lat, lon, 0);
    assert!(
        (r_query - expect).abs() < 1e-3,
        "Query radius {r_query} vs package {expect}"
    );

    // High altitude + outward → Paused → Off
    let pos_hi = Vec3::new(elev + 40_000.0, 0.0, 0.0);
    let vel_out = Vec3::new(100.0, 0.0, 0.0);
    sess.on_tick(pos_hi, vel_out, Matrix3::IDENTITY);
    assert_eq!(sess.schedule, ScheduleState::Paused);
    assert!(matches!(sess.inject(), GroundContact::Off));

    // Descend below resume → Active; Sphere path when not ready.
    let pos_mid = Vec3::new(elev + 15_000.0, 0.0, 0.0);
    let vel_in = Vec3::new(-50.0, 0.0, 0.0);
    sess.workset_ready = false;
    sess.schedule = ScheduleState::Paused;
    sess.on_tick(pos_mid, vel_in, Matrix3::IDENTITY);
    assert_eq!(sess.schedule, ScheduleState::Active);
    sess.workset_ready = false;
    assert!(matches!(sess.inject(), GroundContact::Sphere { .. }));
}

#[test]
fn corotating_body_fixed_sample_radius_stable() {
    // 共转位移后：体固 R 采样高度应与起点一致（惯性 lon 已变）。
    let Some(root) = pkg_low() else {
        eprintln!("skip corotating_body_fixed: packages/physical missing");
        return;
    };
    let elev = {
        let p = orbitx_surface::SurfacePackage::open(&root).unwrap();
        p.r_elev_m
    };
    let cfg = SurfaceSessionConfig {
        package_root: root,
        working_max_level: 0,
        prefetch_every: 1,
        inject_every: 1,
        ..Default::default()
    };
    let mut sess = SurfaceSession::open(cfg, elev).unwrap();
    let angle = orbitx_math::PI2 * 60.0 / 86_164.0;
    let c = angle.cos();
    let s = angle.sin();
    // R_y(+angle)：体固→惯性
    let rot = Matrix3::new(c, 0.0, s, 0.0, 1.0, 0.0, -s, 0.0, c);
    let pos0 = Vec3::new(elev + 50.0, 0.0, 0.0);
    let pos_t = orbitx_dynamics::to_inertial(rot, pos0);

    sess.on_tick(pos0, Vec3::new(-1.0, 0.0, 0.0), Matrix3::IDENTITY);
    let g0 = sess.inject();
    let r0 = g0.radius_at_world(pos0).unwrap();

    sess.on_tick(pos_t, Vec3::new(-1.0, 0.0, 0.0), rot);
    let g1 = sess.inject();
    let r1 = g1.radius_at_world(pos_t).unwrap();
    assert!(
        (r0 - r1).abs() < 1e-3,
        "body-fixed radius should match: r0={r0} r1={r1}"
    );
    // 若误用 R=I 采惯性 lon，半径通常会偏（全球包经度变化）。
    let g_wrong = GroundContact::query(
        match &g1 {
            GroundContact::Query { sampler, .. } => Arc::clone(sampler),
            _ => panic!("expected Query"),
        },
        Matrix3::IDENTITY,
    );
    let r_inertial_lon = g_wrong.radius_at_world(pos_t).unwrap();
    let _ = r_inertial_lon;
}

#[test]
fn override_hard_switch_inside_outside() {
    let Some(root) = pkg_low() else {
        eprintln!("skip override_hard_switch: packages/physical missing");
        return;
    };
    let elev = 6_371_010.0;
    // 原点 (0°, 90°E) 印度洋：±500 m 足迹内恒 120 m 台面，足迹外全球包。
    let spec = ElevationOverrideSpec {
        origin_lat_deg: 0.0,
        origin_lon_deg: 90.0,
        cell_m: 100.0,
        width: 11,
        height: 11,
        data: vec![120.0; 121],
    };
    let cfg = SurfaceSessionConfig {
        package_root: root,
        working_max_level: 0,
        elevation_override: Some(spec),
        ..Default::default()
    };
    let sess = SurfaceSession::open(cfg, elev).unwrap();
    assert!(sess.has_override());
    let contact = sess.placement_contact();

    // 足迹内：覆盖值精确替换 + 陆地 + 无全球层级
    let inside = Vec3::new(0.0, 0.0, elev + 120.0);
    let s = contact.sample_at_world(inside).unwrap();
    assert!(
        (s.radius_m - (elev + 120.0)).abs() < 1e-6,
        "radius {}",
        s.radius_m
    );
    assert_eq!(s.kind, SurfaceKind::Land);
    assert_eq!(s.level_used, None);
    assert!((s.normal.length() - 1.0).abs() < 1e-9);

    // 足迹外（东 ~11 km）：全球包值 + 层级回传
    let (lat_o, lon_o) = (0.0f64, 90.1f64);
    let dir = orbitx_vessel::geocentric_up(lat_o, lon_o);
    let outside = dir * elev;
    let s2 = contact.sample_at_world(outside).unwrap();
    let expect = elev + sess.package.sample_contact_height_at(lat_o, lon_o, 0);
    assert!(
        (s2.radius_m - expect).abs() < 1e-3,
        "outside radius {} vs global {expect}",
        s2.radius_m
    );
    assert!(s2.level_used.is_some());
}

#[test]
fn base_elevation_clamps_inside_footprint_only() {
    let Some(root) = pkg_low() else {
        eprintln!("skip base_elevation_clamps: packages/physical missing");
        return;
    };
    let elev = 6_371_010.0;
    let spec = ElevationOverrideSpec {
        origin_lat_deg: 0.0,
        origin_lon_deg: 90.0,
        cell_m: 100.0,
        width: 11,
        height: 11,
        data: vec![120.0; 121],
    };
    let inside = Vec3::new(0.0, 0.0, elev + 120.0);

    // base=50 < 网格 120：足迹内截平到 50
    let cfg = SurfaceSessionConfig {
        package_root: root.clone(),
        working_max_level: 0,
        elevation_override: Some(spec.clone()),
        launchpad_base_elevation_m: Some(50.0),
        ..Default::default()
    };
    let sess = SurfaceSession::open(cfg, elev).unwrap();
    let s = sess.placement_contact().sample_at_world(inside).unwrap();
    assert!(
        (s.radius_m - (elev + 50.0)).abs() < 1e-6,
        "clamped radius {}",
        s.radius_m
    );

    // base=200 > 网格 120：只截高不填低，足迹内仍 120
    let cfg = SurfaceSessionConfig {
        package_root: root.clone(),
        working_max_level: 0,
        elevation_override: Some(spec),
        launchpad_base_elevation_m: Some(200.0),
        ..Default::default()
    };
    let sess = SurfaceSession::open(cfg, elev).unwrap();
    let s = sess.placement_contact().sample_at_world(inside).unwrap();
    assert!(
        (s.radius_m - (elev + 120.0)).abs() < 1e-6,
        "unclamped radius {}",
        s.radius_m
    );

    // 无覆盖网格：不做面状对齐，采样与全球包一致（base 只作台位点告警）
    let cfg = SurfaceSessionConfig {
        package_root: root,
        working_max_level: 0,
        launchpad_base_elevation_m: Some(1.0),
        ..Default::default()
    };
    let sess = SurfaceSession::open(cfg, elev).unwrap();
    let contact = sess.placement_contact();
    let s = contact.sample_at_world(inside).unwrap();
    let expect = elev + sess.package.sample_contact_height_at(0.0, 90.0, 0);
    assert!(
        (s.radius_m - expect).abs() < 1e-3,
        "no-grid radius {} vs global {expect}",
        s.radius_m
    );
}

#[test]
fn rejects_radius_mismatch() {
    let Some(root) = pkg_low() else {
        eprintln!("skip rejects_radius_mismatch: packages/physical missing");
        return;
    };
    let cfg = SurfaceSessionConfig {
        package_root: root,
        ..Default::default()
    };
    match SurfaceSession::open(cfg, 6_000_000.0) {
        Ok(_) => panic!("expected radius mismatch error"),
        Err(err) => {
            assert!(err.contains("R_elev") || err.contains("body.size"), "{err}");
        }
    }
}

/// 填平域统一公式：域内强制 base（填平，含水面）、过渡带 lerp、域外原样、
/// 边界连续；法向域内 = ENU up、分类恒陆地。
#[test]
fn pad_area_flatten_unified_formula() {
    let Some(root) = pkg_low() else {
        eprintln!("skip pad_area_flatten: packages/physical missing");
        return;
    };
    let elev = 6_371_010.0;
    // 场原点 (0°, 90°)；圆心=场原点，radius=60，blend=20。
    let pa = PadAreaSpec {
        origin_lat_deg: 0.0,
        origin_lon_deg: 90.0,
        center_enu_m: [0.0, 0.0],
        radius_m: 60.0,
        blend_m: 20.0,
    };
    // ENU 偏移 → 经纬度（与 LocalElevationGrid 同投影）。
    // 经纬度采样经由世界向量（GroundContact::sample_at_world）。
    let sample_ll = |contact: &orbitx_vessel::ground::GroundContact,
                     lat: f64,
                     lon: f64|
     -> orbitx_vessel::ground::GroundSample {
        let dir = orbitx_vessel::geocentric_up(lat, lon);
        contact.sample_at_world(dir * elev).unwrap()
    };
    let latlon_at = |e_m: f64, n_m: f64| -> (f64, f64) {
        let (east, north, up) = enu_basis(0.0, 90.0);
        let p = [
            up[0] * elev + east[0] * e_m + north[0] * n_m,
            up[1] * elev + east[1] * e_m + north[1] * n_m,
            up[2] * elev + east[2] * e_m + north[2] * n_m,
        ];
        let r = (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt();
        (
            (p[1] / r).asin().to_degrees(),
            p[2].atan2(p[0]).to_degrees(),
        )
    };
    let cfg = SurfaceSessionConfig {
        package_root: root,
        working_max_level: 0,
        launchpad_base_elevation_m: Some(50.0),
        pad_area: Some(pa),
        ..Default::default()
    };
    let sess = SurfaceSession::open(cfg, elev).unwrap();
    let contact = sess.placement_contact();
    let r_elev = sess.package.r_elev_m;

    // 域内（dist≈0）：h = base，与全球采样无关（填平，非截平）。
    let (lat, lon) = latlon_at(0.0, 0.0);
    let s = sample_ll(&contact, lat, lon);
    assert!(
        (s.radius_m - (r_elev + 50.0)).abs() < 1e-6,
        "in-domain h must equal base, got {}",
        s.radius_m - r_elev
    );
    assert_eq!(s.kind, SurfaceKind::Land);
    assert!(s.level_used.is_none());
    // 域内法向 = ENU up。
    let up_dir = orbitx_vessel::geocentric_up(lat, lon);
    assert!(
        (dot(s.normal, up_dir) - 1.0).abs() < 1e-9,
        "normal must be up"
    );

    // 过渡带中点（dist = radius + blend/2 = 70，沿东向）：h = lerp(50, h_g, 0.5)。
    let (lat, lon) = latlon_at(70.0, 0.0);
    let h_g = sess.package.sample_contact_height_at(lat, lon, 0);
    let s = sample_ll(&contact, lat, lon);
    let expect = 50.0 + (h_g - 50.0) * 0.5;
    assert!(
        (s.radius_m - (r_elev + expect)).abs() < 1e-6,
        "blend midpoint {} vs {expect}",
        s.radius_m - r_elev
    );

    // 域外（dist = radius + blend + 10 = 90）：原样。
    let (lat, lon) = latlon_at(90.0, 0.0);
    let h_g = sess.package.sample_contact_height_at(lat, lon, 0);
    let s = sample_ll(&contact, lat, lon);
    assert!(
        (s.radius_m - (r_elev + h_g)).abs() < 1e-6,
        "outside must be untouched"
    );

    // 边界连续：dist = radius 外沿（t=0，h=base）与 dist→radius⁺（t→0）连续；
    // blend 外沿（t=1）与全球一致。
    let (lat, lon) = latlon_at(60.0, 0.0);
    let s = sample_ll(&contact, lat, lon);
    assert!((s.radius_m - (r_elev + 50.0)).abs() < 1e-6, "t=0 boundary");
    let (lat, lon) = latlon_at(80.0, 0.0);
    let h_g = sess.package.sample_contact_height_at(lat, lon, 0);
    let s = sample_ll(&contact, lat, lon);
    assert!(
        (s.radius_m - (r_elev + h_g)).abs() < 1e-6,
        "t=1 boundary continuity"
    );
}

/// 无 base 的 pad_area 拒绝加载（语义不完整）。
#[test]
fn pad_area_requires_base_elevation() {
    let Some(root) = pkg_low() else {
        eprintln!("skip pad_area_requires_base: packages/physical missing");
        return;
    };
    let cfg = SurfaceSessionConfig {
        package_root: root,
        working_max_level: 0,
        pad_area: Some(PadAreaSpec {
            origin_lat_deg: 0.0,
            origin_lon_deg: 90.0,
            center_enu_m: [0.0, 0.0],
            radius_m: 60.0,
            blend_m: 20.0,
        }),
        ..Default::default()
    };
    let err = match SurfaceSession::open(cfg, 6_371_010.0) {
        Err(e) => e,
        Ok(_) => panic!("pad_area without base must be rejected"),
    };
    assert!(err.contains("base_elevation_m"), "{err}");
}

#[test]
fn high_apogee_inward_velocity_stays_paused() {
    // 2026-11-08 误坠毁根因①回归：远地点 v_radial 过零转负不得把 Paused
    // 翻回 Active——恢复只看高度（alt < resume_alt_m）。S3 曾在 10,698 km
    // 远地点因 `v_radial <= 0` 恢复地面注入并连锁误判「坠入水面」。
    let Some(root) = pkg_low() else {
        eprintln!("skip high_apogee_inward_velocity_stays_paused: packages/physical missing");
        return;
    };
    let elev = {
        let p = orbitx_surface::SurfacePackage::open(&root).unwrap();
        p.r_elev_m
    };
    let cfg = SurfaceSessionConfig {
        package_root: root,
        ..Default::default()
    };
    let mut sess = SurfaceSession::open(cfg, elev).unwrap();
    sess.schedule = ScheduleState::Paused;
    sess.workset_ready = false;

    // 高空（alt >> resume_alt_m）+ 径向速度已转负（刚过远地点）。
    let pos = Vec3::new(elev + 10_698_000.0, 0.0, 0.0);
    let vel = Vec3::new(-3_000.0, 0.0, 0.0);
    sess.on_tick(pos, vel, Matrix3::IDENTITY);
    assert_eq!(sess.schedule, ScheduleState::Paused, "远地点过零不得恢复");
    assert!(
        matches!(sess.inject(), GroundContact::Off),
        "Paused 期间注入必须为 Off"
    );

    // 仍应回落到 resume 门限以下才恢复（既有语义不变）。
    let pos_low = Vec3::new(elev + 15_000.0, 0.0, 0.0);
    sess.on_tick(pos_low, vel, Matrix3::IDENTITY);
    assert_eq!(sess.schedule, ScheduleState::Active, "低于 resume 门槛应恢复");
}
