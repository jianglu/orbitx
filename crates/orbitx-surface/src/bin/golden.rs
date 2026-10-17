//! Compare SurfacePackage samples against golden points (same table as Python fixture).

use std::fs;
use std::path::PathBuf;
use std::process::ExitCode;

use clap::Parser;
use orbitx_surface::{SurfacePackage, WaterClass};
use serde::Deserialize;

#[derive(Parser, Debug)]
#[command(about = "G0b golden-point fixture (CPU SurfaceDataset)")]
struct Args {
    /// packages/<tier> root
    #[arg(long)]
    package: PathBuf,
    /// surface_golden_points.toml
    #[arg(long)]
    golden: PathBuf,
}

#[derive(Debug, Deserialize)]
struct GoldenFile {
    points: Vec<Point>,
}

#[derive(Debug, Deserialize)]
struct Point {
    id: String,
    lat_deg: f64,
    lon_deg: f64,
    class: u8,
    #[serde(default)]
    contact_approx_m: Option<f64>,
    #[serde(default)]
    min_terrain_m: Option<f64>,
}

fn main() -> ExitCode {
    let args = Args::parse();
    let pkg = match SurfacePackage::open(&args.package) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("open package: {e}");
            return ExitCode::FAILURE;
        }
    };
    let text = match fs::read_to_string(&args.golden) {
        Ok(t) => t,
        Err(e) => {
            eprintln!("read golden: {e}");
            return ExitCode::FAILURE;
        }
    };
    let file: GoldenFile = match toml::from_str(&text) {
        Ok(f) => f,
        Err(e) => {
            eprintln!("parse golden: {e}");
            return ExitCode::FAILURE;
        }
    };

    let mut failed = 0usize;
    for p in &file.points {
        let got = pkg.sample_water_class(p.lat_deg, p.lon_deg);
        let got_u = got as u8;
        let contact = pkg.sample_contact_height(p.lat_deg, p.lon_deg);
        let terrain = pkg.sample_terrain(p.lat_deg, p.lon_deg);
        let ok_class = got_u == p.class;
        let mut ok_contact = true;
        let mut ok_terrain = true;
        if p.class == WaterClass::Ocean as u8 {
            ok_contact = (contact - pkg.ocean_surface_m).abs() < 1.0;
        }
        if let Some(approx) = p.contact_approx_m {
            if p.class == WaterClass::InlandWater as u8 {
                ok_contact = (contact - approx).abs() < 5.0;
            }
        }
        if let Some(min_t) = p.min_terrain_m {
            if p.class == WaterClass::Land as u8 {
                ok_terrain = terrain.is_finite() && terrain >= min_t;
            }
        }
        let status = if ok_class && ok_contact && ok_terrain {
            "OK"
        } else {
            failed += 1;
            "FAIL"
        };
        println!(
            "{status} {}: class {got_u} (exp {}) contact={contact:.3} terrain={terrain:.3}",
            p.id, p.class
        );
    }
    println!(
        "summary: {}/{} passed",
        file.points.len() - failed,
        file.points.len()
    );
    if failed > 0 {
        ExitCode::FAILURE
    } else {
        ExitCode::SUCCESS
    }
}
