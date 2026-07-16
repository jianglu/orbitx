//! Demo 5: Headless verification of the ephemeris subsystem.
//!
//! No window, no wgpu. Prints results to stdout and reports PASS/FAIL for a
//! series of checks. Verifies that PlanetarySystem loads real J2000 positions
//! and that MJD advancement produces orbital motion.
//!
//! Run with:
//!   ORBITER_SRC=/path/to/orbiter cargo run -p orbitx-app --example demo_ephemeris

use orbitx_app::ephem_bridge;
use orbitx_math::vec3::Vec3;

/// One astronomical unit in meters.
const AU: f64 = 1.49597870700e11;

/// Dot product of two Vec3 (computed manually to avoid API assumptions).
fn dot(a: Vec3, b: Vec3) -> f64 {
    a.x * b.x + a.y * b.y + a.z * b.z
}

fn main() {
    let mut passed = 0usize;
    let mut failed = 0usize;

    // Small helper to record and print a check result.
    let mut check = |label: &str, ok: bool, detail: &str| {
        if ok {
            passed += 1;
            println!("[PASS] {label}: {detail}");
        } else {
            failed += 1;
            println!("[FAIL] {label}: {detail}");
        }
    };

    // 1. Resolve the orbiter source path (same logic as the app).
    let orbiter_src = ephem_bridge::resolve_orbiter_src();

    // 2. Create the planetary system.
    let mut psys = ephem_bridge::create_planetary_system(&orbiter_src);
    let has_ephemeris = psys.bodies.iter().any(|b| b.ephemeris.is_some());

    // 3. Header.
    println!("========================================");
    println!("  EPHEMERIS DEMO (headless verification)");
    println!("========================================");
    println!("orbiter_src   : {}", orbiter_src.display());
    println!("has_ephemeris : {has_ephemeris}");
    println!("body count    : {}", psys.bodies.len());
    println!();

    // 4. Set MJD to J2000 and update positions.
    psys.mjd = ephem_bridge::sim_time_to_mjd(0.0); // J2000
    psys.update_positions();

    // 5. Print per-body details.
    println!("Bodies at J2000 (MJD = {:.4}):", psys.mjd);
    println!(
        "{:>3}  {:<12} {:>7}  {:>42}  {:>14}  {:>10}  {:>12}",
        "idx", "name", "parent", "position (x, y, z) [m]", "dist [m]", "dist [AU]", "radius [m]"
    );
    for (i, body) in psys.bodies.iter().enumerate() {
        let dist = body.pos.length();
        let parent = match body.parent_idx {
            Some(p) => p.to_string(),
            None => "-".to_string(),
        };
        println!(
            "{:>3}  {:<12} {:>7}  ({:>12.3e}, {:>12.3e}, {:>12.3e})  {:>14.4e}  {:>10.4}  {:>12.4e}",
            i,
            body.name,
            parent,
            body.pos.x,
            body.pos.y,
            body.pos.z,
            dist,
            dist / AU,
            body.radius_m
        );
    }
    println!();

    // 6. Verification checks.
    println!("Verification checks:");

    // Check A: if ephemeris loaded, not all bodies at origin.
    if has_ephemeris {
        let max_dist = psys
            .bodies
            .iter()
            .map(|b| b.pos.length())
            .fold(0.0f64, f64::max);
        check(
            "A (bodies not all at origin)",
            max_dist > 1e9,
            &format!("max body distance = {max_dist:.4e} m"),
        );
    } else {
        println!("[SKIP] A: no ephemeris loaded");
    }

    // Check B: Sun (body 0, parent None) near origin.
    if has_ephemeris {
        if let Some(sun) = psys.bodies.first() {
            if sun.parent_idx.is_none() {
                let d = sun.pos.length();
                check(
                    "B (Sun near barycenter)",
                    d < 2e9,
                    &format!("Sun '{}' distance = {d:.4e} m (frame origin is the solar-system barycenter, so a ~1e9 m Sun offset is expected)", sun.name),
                );
            } else {
                println!("[SKIP] B: body 0 has a parent, not a root star");
            }
        }
    } else {
        println!("[SKIP] B: no ephemeris loaded");
    }

    // Check C: Earth roughly 1 AU from Sun.
    let earth_idx = psys
        .bodies
        .iter()
        .position(|b| b.name.to_lowercase().contains("earth"));
    if has_ephemeris {
        match earth_idx {
            Some(idx) => {
                let earth = &psys.bodies[idx];
                if earth.ephemeris.is_some() {
                    let dist_au = earth.pos.length() / AU;
                    check(
                        "C (Earth ~1 AU from Sun)",
                        (0.9..=1.1).contains(&dist_au),
                        &format!("Earth distance = {dist_au:.4} AU"),
                    );
                } else {
                    println!("[SKIP] C: Earth has no ephemeris");
                }
            }
            None => println!("[SKIP] C: no body with name containing 'Earth'"),
        }
    } else {
        println!("[SKIP] C: no ephemeris loaded");
    }
    println!();

    // 7. MJD advancement / orbital motion check.
    println!("MJD advancement / orbital motion:");
    if let (true, Some(idx)) = (has_ephemeris, earth_idx) {
        // Record Earth's position at J2000.
        let pos_t0 = psys.bodies[idx].pos;

        // Advance MJD by 90 days.
        psys.mjd = ephem_bridge::sim_time_to_mjd(90.0 * 86400.0);
        psys.update_positions();
        let pos_t1 = psys.bodies[idx].pos;

        // Distance moved.
        let moved = (pos_t1 - pos_t0).length();

        // Swept angle from the Sun (origin), via dot product.
        let l0 = pos_t0.length();
        let l1 = pos_t1.length();
        let swept_deg = if l0 > 0.0 && l1 > 0.0 {
            let cos_theta = (dot(pos_t0, pos_t1) / (l0 * l1)).clamp(-1.0, 1.0);
            cos_theta.acos().to_degrees()
        } else {
            0.0
        };

        println!(
            "Earth pos @ J2000    : ({:.4e}, {:.4e}, {:.4e})",
            pos_t0.x, pos_t0.y, pos_t0.z
        );
        println!(
            "Earth pos @ +90 days : ({:.4e}, {:.4e}, {:.4e})",
            pos_t1.x, pos_t1.y, pos_t1.z
        );
        println!("distance moved       : {moved:.4e} m");
        println!(
            "swept angle          : {swept_deg:.2} deg (expected ~88.7 deg)"
        );

        check(
            "D (swept angle 60-120 deg)",
            (60.0..=120.0).contains(&swept_deg),
            &format!("swept {swept_deg:.2} deg in 90 days"),
        );
        check(
            "E (Earth position changed)",
            moved > 1e9,
            &format!("moved {moved:.4e} m"),
        );
    } else {
        println!("[SKIP] D/E: no ephemeris or Earth not found");
    }
    println!();

    // 8. Summary.
    println!("========================================");
    println!("EPHEMERIS DEMO: {passed} checks passed, {failed} failed");
    println!("========================================");
}
