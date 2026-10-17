//! CPU-only SurfaceDataset sampling (G0b / SURFACE_DATASET §4).

use std::fs;
use std::path::{Path, PathBuf};

use image::GrayImage;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum WaterClass {
    Land = 0,
    Ocean = 1,
    InlandWater = 2,
    River = 3,
}

impl WaterClass {
    pub fn from_u8(v: u8) -> Option<Self> {
        match v {
            0 => Some(Self::Land),
            1 => Some(Self::Ocean),
            2 => Some(Self::InlandWater),
            3 => Some(Self::River),
            _ => None,
        }
    }
}

pub struct SurfacePackage {
    pub root: PathBuf,
    pub ocean_surface_m: f64,
    water_class: GrayImage,
    water_surface: FloatGrid,
    terrain: FloatGrid,
}

struct FloatGrid {
    width: u32,
    height: u32,
    data: Vec<f32>,
}

impl FloatGrid {
    fn load(path: &Path) -> Result<Self, String> {
        let meta_path = path.with_extension("json");
        let meta: serde_json::Value = serde_json::from_str(
            &fs::read_to_string(&meta_path).map_err(|e| format!("{meta_path:?}: {e}"))?,
        )
        .map_err(|e| e.to_string())?;
        let width = meta["width"].as_u64().ok_or("width")? as u32;
        let height = meta["height"].as_u64().ok_or("height")? as u32;
        let bytes = fs::read(path).map_err(|e| format!("{path:?}: {e}"))?;
        let need = (width as usize) * (height as usize) * 4;
        if bytes.len() != need {
            return Err(format!(
                "{path:?}: expected {need} bytes, got {}",
                bytes.len()
            ));
        }
        let mut data = Vec::with_capacity((width * height) as usize);
        for chunk in bytes.chunks_exact(4) {
            data.push(f32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]));
        }
        Ok(Self {
            width,
            height,
            data,
        })
    }

    fn nearest(&self, lon_deg: f64, lat_deg: f64) -> f32 {
        let (x, y) = lonlat_to_xy(lon_deg, lat_deg, self.width, self.height);
        let xi = (x.floor() as i64).rem_euclid(self.width as i64) as u32;
        let yi = y.floor().clamp(0.0, (self.height - 1) as f64) as u32;
        self.data[(yi * self.width + xi) as usize]
    }

    fn bilinear(&self, lon_deg: f64, lat_deg: f64) -> f32 {
        let (x, y) = lonlat_to_xy(lon_deg, lat_deg, self.width, self.height);
        let xf = x - 0.5;
        let yf = y - 0.5;
        let x0 = xf.floor() as i64;
        let y0 = yf.floor() as i64;
        let x1 = x0 + 1;
        let y1 = (y0 + 1).min(self.height as i64 - 1);
        let y0c = y0.clamp(0, self.height as i64 - 1);
        let fx = (xf - x0 as f64) as f32;
        let fy = (yf - y0 as f64) as f32;
        let get = |yy: i64, xx: i64| -> f32 {
            let xi = xx.rem_euclid(self.width as i64) as u32;
            let yi = yy.clamp(0, self.height as i64 - 1) as u32;
            self.data[(yi * self.width + xi) as usize]
        };
        let v00 = get(y0c, x0);
        let v10 = get(y0c, x1);
        let v01 = get(y1, x0);
        let v11 = get(y1, x1);
        if v00.is_nan() || v10.is_nan() || v01.is_nan() || v11.is_nan() {
            return self.nearest(lon_deg, lat_deg);
        }
        v00 * (1.0 - fx) * (1.0 - fy)
            + v10 * fx * (1.0 - fy)
            + v01 * (1.0 - fx) * fy
            + v11 * fx * fy
    }
}

fn lonlat_to_xy(lon_deg: f64, lat_deg: f64, w: u32, h: u32) -> (f64, f64) {
    let mut u = (lon_deg + 180.0) / 360.0;
    u = u.rem_euclid(1.0);
    let v = ((90.0 - lat_deg) / 180.0).clamp(0.0, 1.0 - 1e-9);
    (u * w as f64, v * h as f64)
}

const SUPPORTED_FORMAT_VERSION: i64 = 2;

fn parse_format_version(manifest: &str) -> Result<i64, String> {
    for line in manifest.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        if let Some(rest) = line.strip_prefix("format_version") {
            let v = rest.trim().trim_start_matches('=').trim();
            return v
                .parse::<i64>()
                .map_err(|e| format!("format_version: {e}"));
        }
    }
    Err("manifest missing format_version; rebake the surface package".into())
}

fn require_format_version(manifest: &str) -> Result<(), String> {
    let version = parse_format_version(manifest)?;
    if version != SUPPORTED_FORMAT_VERSION {
        return Err(format!(
            "unsupported format_version {version} (expected {SUPPORTED_FORMAT_VERSION}); rebake the surface package"
        ));
    }
    Ok(())
}

fn parse_ocean_surface(manifest: &str) -> f64 {
    for line in manifest.lines() {
        if let Some(rest) = line.strip_prefix("ocean_surface_m") {
            let v = rest.trim().trim_start_matches('=').trim();
            if let Ok(x) = v.parse::<f64>() {
                return x;
            }
        }
    }
    0.0
}

impl SurfacePackage {
    pub fn open(root: impl AsRef<Path>) -> Result<Self, String> {
        let root = root.as_ref().to_path_buf();
        let manifest = fs::read_to_string(root.join("manifest.toml"))
            .map_err(|e| format!("manifest: {e}"))?;
        require_format_version(&manifest)?;
        let ocean_surface_m = parse_ocean_surface(&manifest);
        let wc_path = root.join("water_class/z0_x0_y0.png");
        let water_class = image::open(&wc_path)
            .map_err(|e| format!("{wc_path:?}: {e}"))?
            .to_luma8();
        let water_surface = FloatGrid::load(&root.join("water_surface/z0_x0_y0.f32"))?;
        let terrain = FloatGrid::load(&root.join("terrain/z0_x0_y0.f32"))?;
        Ok(Self {
            root,
            ocean_surface_m,
            water_class,
            water_surface,
            terrain,
        })
    }

    pub fn sample_water_class(&self, lat_deg: f64, lon_deg: f64) -> WaterClass {
        let (x, y) = lonlat_to_xy(
            lon_deg,
            lat_deg,
            self.water_class.width(),
            self.water_class.height(),
        );
        let xi = (x.floor() as i64).rem_euclid(self.water_class.width() as i64) as u32;
        let yi = y
            .floor()
            .clamp(0.0, (self.water_class.height() - 1) as f64) as u32;
        WaterClass::from_u8(self.water_class.get_pixel(xi, yi)[0]).unwrap_or(WaterClass::Land)
    }

    pub fn sample_terrain(&self, lat_deg: f64, lon_deg: f64) -> f64 {
        self.terrain.bilinear(lon_deg, lat_deg) as f64
    }

    pub fn sample_water_surface(&self, lat_deg: f64, lon_deg: f64) -> Option<f64> {
        let v = self.water_surface.bilinear(lon_deg, lat_deg);
        if v.is_nan() {
            None
        } else {
            Some(v as f64)
        }
    }

    pub fn sample_contact_height(&self, lat_deg: f64, lon_deg: f64) -> f64 {
        match self.sample_water_class(lat_deg, lon_deg) {
            WaterClass::Land => self.sample_terrain(lat_deg, lon_deg),
            WaterClass::Ocean | WaterClass::InlandWater | WaterClass::River => self
                .sample_water_surface(lat_deg, lon_deg)
                .unwrap_or_else(|| self.sample_terrain(lat_deg, lon_deg)),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{parse_format_version, require_format_version};

    #[test]
    fn accepts_format_version_2() {
        let manifest = "# comment\nformat_version = 2\nocean_surface_m = 0\n";
        assert_eq!(parse_format_version(manifest).unwrap(), 2);
        assert!(require_format_version(manifest).is_ok());
    }

    #[test]
    fn rejects_other_major() {
        let err = require_format_version("format_version = 1\n").unwrap_err();
        assert!(err.contains("unsupported format_version 1"));
        assert!(err.contains("rebake"));
    }

    #[test]
    fn rejects_missing_version() {
        let err = require_format_version("ocean_surface_m = 0\n").unwrap_err();
        assert!(err.contains("missing format_version"));
    }
}
