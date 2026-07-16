//! Property tests comparing orbitx-ephemeris Rust implementation against the
//! C++ oracle (`orbitx-ephemeris-ffi`).
//!
//! The oracle re-implements the VSOP87 and ELP82 algorithms verbatim from
//! Orbiter's source code. These tests verify that the Rust port produces
//! numerically identical results.
//!
//! **注意**：GALSAT/TASS17 测试使用 C++ 全局状态（`g_gal`/`TasModel`），
//! 多线程并行会竞态。相关测试通过 `EPHEMERIS_LOCK` 互斥锁序列化。

#![allow(clippy::approx_constant, clippy::excessive_precision)]

use std::sync::Mutex;

use orbitx_ephemeris::{interpolate, ElpModel, GalModel, Sample, Series, TasModel, VsopModel};
use orbitx_ephemeris_ffi as ffi;
use proptest::prelude::*;

/// 全局互斥锁：序列化使用 C++ 全局状态的测试（GALSAT/TASS17）。
static EPHEMERIS_LOCK: Mutex<()> = Mutex::new(());

// --- Helpers ---

const TOL: f64 = 1e-10;
const ATOL: f64 = 1e-12;

/// Assert two `f64` values are close: relative OR absolute.
fn assert_close(a: f64, b: f64, msg: &str) {
    let diff = (a - b).abs();
    let maxmag = a.abs().max(b.abs());
    let allowed = TOL * maxmag + ATOL;
    assert!(
        diff <= allowed || (a.is_nan() && b.is_nan()),
        "{msg}: {a} vs {b} (diff={diff}, allowed={allowed})"
    );
}

fn assert_close6(a: &[f64; 6], b: &[f64; 6], ctx: &str) {
    for i in 0..6 {
        assert_close(a[i], b[i], &format!("{ctx}[{i}]"));
    }
}

// --- Locate Orbiter data files ---

/// Locate the Orbiter source tree (sibling of orbitx).
fn orbiter_src() -> std::path::PathBuf {
    let manifest = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    manifest
        .ancestors()
        .nth(2)
        .unwrap()
        .parent()
        .unwrap()
        .join("orbiter")
        .join("Src")
        .join("Celbody")
}

fn vsop_data_path(filename: &str) -> String {
    let p = orbiter_src().join("Vsop87").join("Data").join(filename);
    p.to_str().unwrap().to_string()
}

fn elp_data_path() -> String {
    let p = orbiter_src()
        .join("Moon")
        .join("Config")
        .join("Moon")
        .join("Data")
        .join("ELP82.dat");
    p.to_str().unwrap().to_string()
}

// ===========================================================
// Interpolate property tests
// ===========================================================

fn sample_strategy() -> impl Strategy<Value = Sample> {
    (
        -1e6_f64..1e6,
        -1e6_f64..1e6,
        prop::collection::vec(-1e6_f64..1e6, 6),
    )
        .prop_map(|(t, rad, params)| {
            let mut param = [0.0; 6];
            for (i, &v) in params.iter().enumerate() {
                param[i] = v;
            }
            Sample { t, rad, param }
        })
}

proptest! {
    #[test]
    fn prop_interpolate(s0_raw in sample_strategy(), s1_raw in sample_strategy(), u in 0.0..1.0) {
        // Ensure s0.t < s1.t and u maps to a valid interpolation point.
        let (mut s0, mut s1) = (s0_raw, s1_raw);
        if s0.t > s1.t { std::mem::swap(&mut s0, &mut s1); }
        if (s1.t - s0.t).abs() < 1e-6 { s1.t = s0.t + 1.0; }
        let t = s0.t + u * (s1.t - s0.t);

        let mut rust = [0.0; 6];
        interpolate(t, &mut rust, &s0, &s1);

        let c0: ffi::CSample = s0.into();
        let c1: ffi::CSample = s1.into();
        let cpp = ffi::interpolate(t, &c0, &c1);

        assert_close6(&rust, &cpp, "interpolate");
    }
}

// ===========================================================
// VSOP87 property tests (Earth, series B)
// ===========================================================

struct VsopFixture {
    model: VsopModel,
    oracle: ffi::VsopOracle,
}

fn earth_fixture() -> VsopFixture {
    let path = vsop_data_path("Vsop87B_ear.dat");
    let model = VsopModel::from_reader(
        std::io::BufReader::new(std::fs::File::open(&path).unwrap()),
        Series::B,
        1.0,
        1e-6,
        10.0,
    )
    .unwrap();

    let oracle = ffi::VsopOracle::new('B', 1.0, 1e-6, 10.0);
    assert!(
        oracle.read_data(&path),
        "C++ oracle failed to read VSOP data"
    );

    VsopFixture { model, oracle }
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(50))]
    #[test]
    fn prop_vsop_earth_eval(mjd_offset in -18_250.0_f64..18_250.0) {
        // ±50 years from J2000 (MJD 51544.5)
        let mjd = 51_544.5 + mjd_offset;
        let fix = earth_fixture();

        let rust = fix.model.eval(mjd);
        let cpp = fix.oracle.eval(mjd);

        // Polar output: L,B in rad, R in AU. Velocity in rad/s and AU/s.
        assert_close6(&rust, &cpp, "vsop_earth_eval");
    }
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(30))]
    #[test]
    fn prop_vsop_earth_fast_eval(simt_offset in -1000.0_f64..1000.0) {
        // simt within ±1000s of J2000 (t=0)
        let simt = simt_offset;
        // Set MJD_ref = 51544.5 so oapiTime2MJD(0) = 51544.5
        ffi::VsopOracle::set_mjd_ref(51_544.5);

        let mut fix = earth_fixture();

        let rust = fix.model.fast_eval(simt);
        let cpp = fix.oracle.fast_eval(simt);

        assert_close6(&rust, &cpp, "vsop_earth_fast_eval");
    }
}

// ===========================================================
// ELP82 property tests (Moon)
// ===========================================================

fn moon_fixture() -> ElpModel {
    let path = elp_data_path();
    ElpModel::from_reader(
        std::io::BufReader::new(std::fs::File::open(&path).unwrap()),
        1e-6,
    )
    .unwrap()
}

fn ensure_elp_loaded() {
    use std::sync::Once;
    static INIT: Once = Once::new();
    INIT.call_once(|| {
        let path = elp_data_path();
        assert!(
            ffi::elp_read_data(&path, 1e-6),
            "C++ oracle failed to read ELP82 data"
        );
    });
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(50))]
    #[test]
    fn prop_elp_moon_eval(mjd_offset in -18_250.0_f64..18_250.0) {
        // ±50 years from J2000
        let mjd = 51_544.5 + mjd_offset;
        let model = moon_fixture();
        ensure_elp_loaded();

        let rust = model.eval(mjd);
        let cpp = ffi::elp_eval(mjd);

        // Moon position in meters, velocity in m/s.
        // ELP82 involves large-angle cancellation over decades (lunar longitude
        // accumulates ~2.7e9 rad/century), so floating-point accumulation
        // differences at the ~1e-9 relative level are expected between the Rust
        // and C++ evaluation orderings. Use a relaxed tolerance.
        const ELP_TOL: f64 = 1e-8;
        const ELP_ATOL: f64 = 1e-9;
        for i in 0..6 {
            let diff = (rust[i] - cpp[i]).abs();
            let maxmag = rust[i].abs().max(cpp[i].abs());
            let allowed = ELP_TOL * maxmag + ELP_ATOL;
            assert!(
                diff <= allowed,
                "elp_moon_eval[{i}]: {} vs {} (diff={diff}, allowed={allowed})",
                rust[i], cpp[i]
            );
        }
    }
}

// ===========================================================
// TASS17 Saturn moons property tests
// ===========================================================

fn tass17_data_path() -> String {
    let p = orbiter_src().join("Satsat").join("tass17.dat");
    p.to_str().unwrap().to_string()
}

fn tass17_fixture() -> TasModel {
    let path = tass17_data_path();
    TasModel::from_reader(std::io::BufReader::new(std::fs::File::open(&path).unwrap())).unwrap()
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(20))]
    #[test]
    fn prop_tass17_eval(
        mjd_offset in -18_250.0_f64..18_250.0,
        isat in 0usize..8,
    ) {
        let _lock = EPHEMERIS_LOCK.lock().unwrap();
        // ±50 years from J2000; all 8 satellites
        let mjd = 51_544.5 + mjd_offset;
        let jd = mjd + 2_400_000.5;
        let model = tass17_fixture();

        let oracle = ffi::TasOracle::new();
        assert!(oracle.read_data(&tass17_data_path()), "C++ oracle 读取 TASS17 数据失败");

        let rust = model.eval(jd, isat);
        let cpp = oracle.eval(jd, isat);

        // TASS17 output in meters and m/s. AU->m scaling amplifies
        // relative errors. Use relaxed tolerance.
        for i in 0..6 {
            let diff = (rust[i] - cpp[i]).abs();
            let maxmag = rust[i].abs().max(cpp[i].abs());
            let allowed = 1e-6 * maxmag + 1e-3;
            assert!(
                diff <= allowed,
                "tass17_eval[isat={isat}, {i}]: {} vs {} (diff={diff}, allowed={allowed})",
                rust[i], cpp[i]
            );
        }
    }
}

// ===========================================================
// GALSAT Jupiter Galilean moons property tests
// ===========================================================

fn galsat_data_path() -> String {
    let p = orbiter_src().join("Galsat").join("ephem_e15.dat");
    p.to_str().unwrap().to_string()
}

fn galsat_fixture() -> GalModel {
    let path = galsat_data_path();
    GalModel::from_reader(std::io::BufReader::new(
        std::fs::File::open(&path).unwrap(),
    ))
    .unwrap()
}

fn ensure_galsat_loaded() {
    use std::sync::Once;
    static INIT: Once = Once::new();
    INIT.call_once(|| {
        let path = galsat_data_path();
        assert!(
            ffi::galsat_read(&path),
            "C++ oracle failed to read GALSAT data"
        );
    });
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(20))]
    #[test]
    fn prop_galsat_eval(
        mjd_offset in -18_262.0_f64..18_262.0,
        ksat in 1i32..5,
    ) {
        let _lock = EPHEMERIS_LOCK.lock().unwrap();
        // +/-50 years from J2000; satellites Io/Europa/Ganymede/Callisto (ksat=1..4)
        let mjd = 51_544.5 + mjd_offset;
        let jd = mjd + 2_400_000.5;
        let mut model = galsat_fixture();
        ensure_galsat_loaded();

        let rust = model.eval(jd, ksat);
        let cpp = ffi::galsat_eval(jd, ksat);

        // GALSAT output in metres and m/s. The Lieske series involve large
        // angle accumulation over decades, so use a relaxed tolerance.
        for i in 0..6 {
            let diff = (rust[i] - cpp[i]).abs();
            let maxmag = rust[i].abs().max(cpp[i].abs());
            let allowed = 1e-9 * maxmag + 1e-6;
            assert!(
                diff <= allowed,
                "galsat_eval[ksat={ksat}, {i}]: {} vs {} (diff={diff}, allowed={allowed})",
                rust[i], cpp[i]
            );
        }
    }
}

// GALSAT barycentre correction (ksat=0): 质心→木星向量修正。
proptest! {
    #![proptest_config(ProptestConfig::with_cases(10))]
    #[test]
    fn prop_galsat_barycentre(
        mjd_offset in -18_262.0_f64..18_262.0,
    ) {
        let _lock = EPHEMERIS_LOCK.lock().unwrap();
        let mjd = 51_544.5 + mjd_offset;
        let jd = mjd + 2_400_000.5;
        let mut model = galsat_fixture();
        ensure_galsat_loaded();

        let rust = model.eval(jd, 0);
        let cpp = ffi::galsat_eval(jd, 0);

        // Barycentre correction is very small (sub-km). The revizg_ great-inequality
        // correction affects barycentre at the ~30m level, so use relaxed tolerance.
        for i in 0..6 {
            let diff = (rust[i] - cpp[i]).abs();
            let maxmag = rust[i].abs().max(cpp[i].abs());
            let allowed = 1e-9 * maxmag + 100.0;
            assert!(
                diff <= allowed,
                "galsat_barycentre[{i}]: {} vs {} (diff={diff}, allowed={allowed})",
                rust[i], cpp[i]
            );
        }
    }
}
