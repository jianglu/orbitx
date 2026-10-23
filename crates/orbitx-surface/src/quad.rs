//! equirect_quadtree tile index (G0b `tile_scheme`).
//! z0 = 1×1; zL (L≥1): cols=2^L, rows=2^(L-1).

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct TileKey {
    pub level: u32,
    pub x: u32,
    pub y: u32,
}

pub fn cols(level: u32) -> u32 {
    1u32 << level
}

pub fn rows(level: u32) -> u32 {
    if level == 0 {
        1
    } else {
        1u32 << (level - 1)
    }
}

/// Global equirect uv from lon/lat degrees (u east from −180, v south from +90).
pub fn lonlat_to_uv(lon_deg: f64, lat_deg: f64) -> (f64, f64) {
    let mut u = (lon_deg + 180.0) / 360.0;
    u = u.rem_euclid(1.0);
    let v = ((90.0 - lat_deg) / 180.0).clamp(0.0, 1.0 - 1e-9);
    (u, v)
}

pub fn tile_covering(level: u32, lon_deg: f64, lat_deg: f64) -> TileKey {
    let (u, v) = lonlat_to_uv(lon_deg, lat_deg);
    let c = cols(level);
    let r = rows(level);
    let x = ((u * c as f64).floor() as i64).rem_euclid(c as i64) as u32;
    let y = (v * r as f64).floor().clamp(0.0, (r - 1) as f64) as u32;
    TileKey { level, x, y }
}

/// Pixel coords in a tile of size (w,h) covering the given lon/lat.
pub fn tile_xy(key: TileKey, lon_deg: f64, lat_deg: f64, w: u32, h: u32) -> (f64, f64) {
    let (u, v) = lonlat_to_uv(lon_deg, lat_deg);
    let c = cols(key.level) as f64;
    let r = rows(key.level) as f64;
    let u0 = key.x as f64 / c;
    let v0 = key.y as f64 / r;
    let local_u = ((u - u0) * c).clamp(0.0, 1.0 - 1e-12);
    let local_v = ((v - v0) * r).clamp(0.0, 1.0 - 1e-12);
    (local_u * w as f64, local_v * h as f64)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn z0_covers_globe() {
        let k = tile_covering(0, 10.0, 20.0);
        assert_eq!(k, TileKey { level: 0, x: 0, y: 0 });
    }

    #[test]
    fn z1_splits_longitude_only() {
        let west = tile_covering(1, -90.0, 0.0);
        let east = tile_covering(1, 90.0, 0.0);
        assert_eq!(west, TileKey { level: 1, x: 0, y: 0 });
        assert_eq!(east, TileKey { level: 1, x: 1, y: 0 });
        assert_eq!(rows(1), 1);
        assert_eq!(cols(1), 2);
    }
}
