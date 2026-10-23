//! Elevation / water_surface tile decode (z0 f32, z≥1 int16 unsigned / flat).

use std::fs;
use std::path::Path;

use serde_json::Value;

use crate::quad::{self, TileKey};

#[derive(Debug, Clone)]
pub enum ElevTile {
    /// Constant height (dtype=0); no payload file.
    Flat { height_m: f32 },
    /// Dense samples in meters (NaN = nodata for water_surface).
    Grid { width: u32, height: u32, data: Vec<f32> },
}

impl ElevTile {
    pub fn sample_bilinear(&self, lon_deg: f64, lat_deg: f64, key: TileKey) -> f32 {
        match self {
            ElevTile::Flat { height_m } => *height_m,
            ElevTile::Grid { width, height, data } => {
                let (x, y) = if key.level == 0 {
                    lonlat_to_xy_global(lon_deg, lat_deg, *width, *height)
                } else {
                    quad::tile_xy(key, lon_deg, lat_deg, *width, *height)
                };
                bilinear(data, *width, *height, x, y, lon_deg, lat_deg, key)
            }
        }
    }

}

fn lonlat_to_xy_global(lon_deg: f64, lat_deg: f64, w: u32, h: u32) -> (f64, f64) {
    let (u, v) = quad::lonlat_to_uv(lon_deg, lat_deg);
    (u * w as f64, v * h as f64)
}

fn nearest(data: &[f32], w: u32, h: u32, x: f64, y: f64) -> f32 {
    let xi = (x.floor() as i64).rem_euclid(w as i64) as u32;
    let yi = y.floor().clamp(0.0, (h - 1) as f64) as u32;
    data[(yi * w + xi) as usize]
}

fn bilinear(
    data: &[f32],
    w: u32,
    h: u32,
    x: f64,
    y: f64,
    lon_deg: f64,
    lat_deg: f64,
    key: TileKey,
) -> f32 {
    let xf = x - 0.5;
    let yf = y - 0.5;
    let x0 = xf.floor() as i64;
    let y0 = yf.floor() as i64;
    let x1 = x0 + 1;
    let y1 = (y0 + 1).min(h as i64 - 1);
    let y0c = y0.clamp(0, h as i64 - 1);
    let fx = (xf - x0 as f64) as f32;
    let fy = (yf - y0 as f64) as f32;
    // Lon wrap only for global z0; intra-tile clamps edges.
    let wrap_x = key.level == 0;
    let get = |yy: i64, xx: i64| -> f32 {
        let xi = if wrap_x {
            xx.rem_euclid(w as i64) as u32
        } else {
            xx.clamp(0, w as i64 - 1) as u32
        };
        let yi = yy.clamp(0, h as i64 - 1) as u32;
        data[(yi * w + xi) as usize]
    };
    let v00 = get(y0c, x0);
    let v10 = get(y0c, x1);
    let v01 = get(y1, x0);
    let v11 = get(y1, x1);
    if v00.is_nan() || v10.is_nan() || v01.is_nan() || v11.is_nan() {
        let _ = (lon_deg, lat_deg, key);
        return nearest(data, w, h, x, y);
    }
    v00 * (1.0 - fx) * (1.0 - fy)
        + v10 * fx * (1.0 - fy)
        + v01 * (1.0 - fx) * fy
        + v11 * fx * fy
}

/// Decode elev tile at `dir/zL_xX_yY` (.f32 z0 or .i16/.json).
/// When `nodata_u16` is set (water_surface: 65535), that raw becomes NaN.
pub fn load_elev_tile(
    dir: &Path,
    key: TileKey,
    nodata_u16: Option<u16>,
) -> Result<ElevTile, String> {
    let stem = format!("z{}_x{}_y{}", key.level, key.x, key.y);
    let json_path = dir.join(format!("{stem}.json"));
    if !json_path.is_file() {
        return Err(format!("missing {json_path:?}"));
    }
    let meta: Value = serde_json::from_str(
        &fs::read_to_string(&json_path).map_err(|e| format!("{json_path:?}: {e}"))?,
    )
    .map_err(|e| e.to_string())?;

    let width = meta["width"].as_u64().ok_or("width")? as u32;
    let height = meta["height"].as_u64().ok_or("height")? as u32;
    let dtype = &meta["dtype"];

    // z0 float32
    if key.level == 0
        || dtype.as_str() == Some("float32_le")
        || dir.join(format!("{stem}.f32")).is_file()
    {
        let f32_path = dir.join(format!("{stem}.f32"));
        if f32_path.is_file() {
            return load_f32(&f32_path, width, height);
        }
    }

    // Flat dtype=0
    let dtype_i = dtype.as_i64().or_else(|| dtype.as_u64().map(|u| u as i64));
    if dtype_i == Some(0) {
        let offset = meta["offset"].as_f64().unwrap_or(0.0) as f32;
        return Ok(ElevTile::Flat { height_m: offset });
    }

    // int16 quantized: h = offset + raw×scale, raw unsigned 0..65535
    let scale = meta["scale"].as_f64().unwrap_or(1.0) as f32;
    let offset = meta["offset"].as_f64().unwrap_or(0.0) as f32;
    let i16_path = dir.join(format!("{stem}.i16"));
    let bytes = fs::read(&i16_path).map_err(|e| format!("{i16_path:?}: {e}"))?;
    let need = (width as usize) * (height as usize) * 2;
    if bytes.len() != need {
        return Err(format!(
            "{i16_path:?}: expected {need} bytes, got {}",
            bytes.len()
        ));
    }
    let mut data = Vec::with_capacity((width * height) as usize);
    for chunk in bytes.chunks_exact(2) {
        let raw = u16::from_le_bytes([chunk[0], chunk[1]]);
        if nodata_u16 == Some(raw) {
            data.push(f32::NAN);
        } else {
            data.push(offset + (raw as f32) * scale);
        }
    }
    Ok(ElevTile::Grid {
        width,
        height,
        data,
    })
}

fn load_f32(path: &Path, width: u32, height: u32) -> Result<ElevTile, String> {
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
    Ok(ElevTile::Grid {
        width,
        height,
        data,
    })
}

/// Walk from `prefer` down to z0 until a tile loads.
pub fn load_elev_with_fallback(
    dir: &Path,
    lon_deg: f64,
    lat_deg: f64,
    prefer_level: u32,
    nodata_u16: Option<u16>,
) -> Result<(TileKey, ElevTile), String> {
    let mut level = prefer_level;
    loop {
        let key = quad::tile_covering(level, lon_deg, lat_deg);
        match load_elev_tile(dir, key, nodata_u16) {
            Ok(tile) => return Ok((key, tile)),
            Err(_) if level == 0 => {
                return Err(format!(
                    "no elev tile under {dir:?} for lon={lon_deg} lat={lat_deg}"
                ));
            }
            Err(_) => {
                level = parent_level(level);
            }
        }
    }
}

fn parent_level(level: u32) -> u32 {
    level.saturating_sub(1)
}

/// Encode helpers for unit tests (unsigned raw semantics).
#[cfg(test)]
pub mod testutil {
    use super::*;
    use std::io::Write;

    pub fn write_i16_tile(
        dir: &Path,
        key: TileKey,
        width: u32,
        height: u32,
        scale: f32,
        offset: f32,
        raws: &[u16],
    ) {
        assert_eq!(raws.len(), (width * height) as usize);
        fs::create_dir_all(dir).unwrap();
        let stem = format!("z{}_x{}_y{}", key.level, key.x, key.y);
        let meta = serde_json::json!({
            "width": width,
            "height": height,
            "dtype": -16,
            "scale": scale,
            "offset": offset,
            "emin": offset,
            "emax": offset + scale * 65535.0,
        });
        fs::write(dir.join(format!("{stem}.json")), meta.to_string()).unwrap();
        let mut f = fs::File::create(dir.join(format!("{stem}.i16"))).unwrap();
        for r in raws {
            f.write_all(&r.to_le_bytes()).unwrap();
        }
    }

    pub fn write_flat_tile(dir: &Path, key: TileKey, width: u32, height: u32, offset: f32) {
        fs::create_dir_all(dir).unwrap();
        let stem = format!("z{}_x{}_y{}", key.level, key.x, key.y);
        let meta = serde_json::json!({
            "width": width,
            "height": height,
            "dtype": 0,
            "scale": 1.0,
            "offset": offset,
            "emin": offset,
            "emax": offset,
            "emean": offset,
        });
        fs::write(dir.join(format!("{stem}.json")), meta.to_string()).unwrap();
    }

    pub fn write_f32_z0(dir: &Path, width: u32, height: u32, data: &[f32]) {
        fs::create_dir_all(dir).unwrap();
        let stem = "z0_x0_y0";
        let meta = serde_json::json!({
            "width": width,
            "height": height,
            "dtype": "float32_le",
        });
        fs::write(dir.join(format!("{stem}.json")), meta.to_string()).unwrap();
        let mut f = fs::File::create(dir.join(format!("{stem}.f32"))).unwrap();
        for v in data {
            f.write_all(&v.to_le_bytes()).unwrap();
        }
    }
}
