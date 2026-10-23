//! CPU-only SurfaceDataset sampling (G0b / SURFACE_DATASET §4).
//!
//! Coordinates: **degrees** (`lat_deg` / `lon_deg`), matching G0b golden fixtures.

mod elev;
pub mod local_grid;
mod lru;
mod manifest;
mod quad;

use std::fs;
use std::path::{Path, PathBuf};
use std::sync::Mutex;

use lru::LruMap;

use image::GrayImage;

pub use local_grid::{enu_basis, latlon_up, GridSample, LocalElevationGrid};
pub use manifest::{Manifest, SUPPORTED_FORMAT_VERSION};
pub use quad::TileKey;

use elev::{load_elev_with_fallback, ElevTile};
use manifest::Manifest as PkgManifest;
use quad::TileKey as QKey;

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

struct ClassTile {
    width: u32,
    height: u32,
    data: GrayImage,
    key: QKey,
}

impl ClassTile {
    fn sample_nearest(&self, lon_deg: f64, lat_deg: f64) -> u8 {
        let (x, y) = if self.key.level == 0 {
            let (u, v) = quad::lonlat_to_uv(lon_deg, lat_deg);
            (u * self.width as f64, v * self.height as f64)
        } else {
            quad::tile_xy(self.key, lon_deg, lat_deg, self.width, self.height)
        };
        let xi = (x.floor() as i64).rem_euclid(self.width as i64) as u32;
        let yi = y.floor().clamp(0.0, (self.height - 1) as f64) as u32;
        self.data.get_pixel(xi, yi)[0]
    }
}

fn load_class_tile(dir: &Path, key: QKey) -> Result<ClassTile, String> {
    let path = dir.join(format!("z{}_x{}_y{}.png", key.level, key.x, key.y));
    let img = image::open(&path)
        .map_err(|e| format!("{path:?}: {e}"))?
        .to_luma8();
    Ok(ClassTile {
        width: img.width(),
        height: img.height(),
        data: img,
        key,
    })
}

fn load_class_with_fallback(
    dir: &Path,
    lon_deg: f64,
    lat_deg: f64,
    prefer_level: u32,
) -> Result<ClassTile, String> {
    let mut level = prefer_level;
    loop {
        let key = quad::tile_covering(level, lon_deg, lat_deg);
        match load_class_tile(dir, key) {
            Ok(t) => return Ok(t),
            Err(_) if level == 0 => {
                return Err(format!("no water_class tile under {dir:?}"));
            }
            Err(_) => level = level.saturating_sub(1),
        }
    }
}

/// 接触采样细节（高度之外的法向/实际层级/类别）。
#[derive(Debug, Clone, Copy)]
pub struct ContactDetail {
    /// 相对高程球的接触高（m）。
    pub height_m: f64,
    /// 地面法向的 ENU 分量（查询点当地系，单位向量）。
    pub normal_enu: [f64; 3],
    /// 实际命中的四叉树层级（回退后）。
    pub level_used: u32,
    /// 水陆类别。
    pub kind: WaterClass,
}

/// Default tile-cache capacity (elev + class share separate maps).
pub const DEFAULT_TILE_CACHE_CAP: usize = 64;

/// Cache occupancy snapshot.
#[derive(Debug, Clone, Copy, Default)]
pub struct CacheStats {
    pub elev_tiles: usize,
    pub class_tiles: usize,
    pub elev_cap: usize,
    pub class_cap: usize,
}

/// Opened SurfaceDataset package (CPU sampling).
pub struct SurfacePackage {
    pub root: PathBuf,
    pub dataset_id: String,
    pub r_elev_m: f64,
    pub ocean_surface_m: f64,
    manifest: PkgManifest,
    /// Bounded tile memo (Mutex → Send+Sync for runtime threads).
    elev_cache: Mutex<LruMap<(u8, QKey), ElevTile>>,
    class_cache: Mutex<LruMap<QKey, ClassTile>>,
    cache_cap: usize,
}

impl SurfacePackage {
    pub fn open(root: impl AsRef<Path>) -> Result<Self, String> {
        Self::open_with_cache_cap(root, DEFAULT_TILE_CACHE_CAP)
    }

    pub fn open_with_cache_cap(root: impl AsRef<Path>, cache_cap: usize) -> Result<Self, String> {
        let root = root.as_ref().to_path_buf();
        let text = fs::read_to_string(root.join("manifest.toml"))
            .map_err(|e| format!("manifest: {e}"))?;
        let manifest = PkgManifest::parse(&text)?;
        let cap = cache_cap.max(1);
        Ok(Self {
            root,
            dataset_id: manifest.dataset_id.clone(),
            r_elev_m: manifest.r_elev_m,
            ocean_surface_m: manifest.ocean_surface_m,
            manifest,
            elev_cache: Mutex::new(LruMap::new(cap)),
            class_cache: Mutex::new(LruMap::new(cap)),
            cache_cap: cap,
        })
    }

    pub fn max_level(&self, layer: &str) -> u32 {
        self.manifest.layer_max_level(layer)
    }

    pub fn cache_stats(&self) -> CacheStats {
        let elev = self.elev_cache.lock().unwrap();
        let class = self.class_cache.lock().unwrap();
        CacheStats {
            elev_tiles: elev.len(),
            class_tiles: class.len(),
            elev_cap: self.cache_cap,
            class_cap: self.cache_cap,
        }
    }

    /// Prefetch covering elev/class tiles around `(lat, lon)` at `level`,
    /// plus a Chebyshev ring of `extra_ring` neighbour tiles.
    pub fn prefetch_around(
        &self,
        lat_deg: f64,
        lon_deg: f64,
        level: u32,
        extra_ring: u32,
    ) -> Result<(), String> {
        let t_lv = level.min(self.max_level("terrain"));
        let w_lv = level.min(self.max_level("water_surface"));
        let c_lv = level.min(self.max_level("water_class"));
        let lv = t_lv.max(w_lv).max(c_lv);
        let center = quad::tile_covering(lv, lon_deg, lat_deg);
        let cols = quad::cols(lv);
        let rows = quad::rows(lv);
        let ring = extra_ring as i32;
        for dy in -ring..=ring {
            for dx in -ring..=ring {
                let x = (center.x as i64 + dx as i64).rem_euclid(cols as i64) as u32;
                let y = (center.y as i32 + dy).clamp(0, rows.saturating_sub(1) as i32) as u32;
                let key = QKey { level: lv, x, y };
                let (lon, lat) = tile_center_lonlat(key);
                let _ = self.sample_elev("terrain", lat, lon, t_lv, None);
                let _ = self.sample_elev("water_surface", lat, lon, w_lv, Some(u16::MAX));
                let _ = self.sample_water_class_at(lat, lon, c_lv);
            }
        }
        Ok(())
    }

    pub fn sample_water_class(&self, lat_deg: f64, lon_deg: f64) -> WaterClass {
        self.sample_water_class_at(lat_deg, lon_deg, self.max_level("water_class"))
    }

    pub fn sample_water_class_at(&self, lat_deg: f64, lon_deg: f64, prefer_level: u32) -> WaterClass {
        let dir = self.root.join(self.manifest.layer_path("water_class"));
        let prefer = prefer_level.min(self.max_level("water_class"));
        // Cache hit on covering tile or any ancestor.
        {
            let mut cache = self.class_cache.lock().unwrap();
            let mut level = prefer;
            loop {
                let key = quad::tile_covering(level, lon_deg, lat_deg);
                if let Some(t) = cache.get(&key) {
                    return WaterClass::from_u8(t.sample_nearest(lon_deg, lat_deg))
                        .unwrap_or(WaterClass::Land);
                }
                if level == 0 {
                    break;
                }
                level -= 1;
            }
        }
        let tile = load_class_with_fallback(&dir, lon_deg, lat_deg, prefer).unwrap_or_else(|_| {
            ClassTile {
                width: 1,
                height: 1,
                data: GrayImage::from_pixel(1, 1, image::Luma([0])),
                key: QKey {
                    level: 0,
                    x: 0,
                    y: 0,
                },
            }
        });
        let v = tile.sample_nearest(lon_deg, lat_deg);
        self.class_cache.lock().unwrap().insert(tile.key, tile);
        WaterClass::from_u8(v).unwrap_or(WaterClass::Land)
    }

    pub fn sample_terrain(&self, lat_deg: f64, lon_deg: f64) -> f64 {
        self.sample_elev("terrain", lat_deg, lon_deg, self.max_level("terrain"), None) as f64
    }

    pub fn sample_terrain_at(&self, lat_deg: f64, lon_deg: f64, prefer_level: u32) -> f64 {
        self.sample_elev("terrain", lat_deg, lon_deg, prefer_level, None) as f64
    }

    pub fn sample_water_surface(&self, lat_deg: f64, lon_deg: f64) -> Option<f64> {
        self.sample_water_surface_at(lat_deg, lon_deg, self.max_level("water_surface"))
    }

    pub fn sample_water_surface_at(
        &self,
        lat_deg: f64,
        lon_deg: f64,
        prefer_level: u32,
    ) -> Option<f64> {
        let v = self.sample_elev(
            "water_surface",
            lat_deg,
            lon_deg,
            prefer_level,
            Some(u16::MAX),
        );
        if v.is_nan() {
            None
        } else {
            Some(v as f64)
        }
    }

    pub fn sample_contact_height(&self, lat_deg: f64, lon_deg: f64) -> f64 {
        self.sample_contact_height_at(
            lat_deg,
            lon_deg,
            self.max_level("water_class")
                .max(self.max_level("terrain"))
                .max(self.max_level("water_surface")),
        )
    }

    pub fn sample_contact_height_at(&self, lat_deg: f64, lon_deg: f64, prefer_level: u32) -> f64 {
        match self.sample_water_class_at(lat_deg, lon_deg, prefer_level) {
            WaterClass::Land => self.sample_terrain_at(lat_deg, lon_deg, prefer_level),
            WaterClass::Ocean | WaterClass::InlandWater | WaterClass::River => self
                .sample_water_surface_at(lat_deg, lon_deg, prefer_level)
                .unwrap_or_else(|| self.sample_terrain_at(lat_deg, lon_deg, prefer_level)),
        }
    }

    /// 接触采样细节：高度 + 地面法向（ENU）+ 实际命中层级 + 水陆类别。
    pub fn sample_contact_detail_at(
        &self,
        lat_deg: f64,
        lon_deg: f64,
        prefer_level: u32,
    ) -> ContactDetail {
        let kind = self.sample_water_class_at(lat_deg, lon_deg, prefer_level);
        let (height_m, level_used) = match kind {
            WaterClass::Land => self.sample_elev_detailed("terrain", lat_deg, lon_deg, prefer_level, None),
            WaterClass::Ocean | WaterClass::InlandWater | WaterClass::River => {
                let (v, lv) = self.sample_elev_detailed(
                    "water_surface",
                    lat_deg,
                    lon_deg,
                    prefer_level,
                    Some(u16::MAX),
                );
                if v.is_nan() {
                    self.sample_elev_detailed("terrain", lat_deg, lon_deg, prefer_level, None)
                } else {
                    (v, lv)
                }
            }
        };
        // 法向：以实际层 texel 尺度做中心差分（ENU 分量）。
        let texel_m = (2.0 * std::f64::consts::PI * self.r_elev_m)
            / ((1u64 << level_used.min(20)) as f64 * 1024.0);
        let step_m = texel_m.max(1.0);
        let dlat = (step_m / self.r_elev_m).to_degrees();
        let coslat = lat_deg.to_radians().cos().abs().max(1e-3);
        let dlon = (step_m / (self.r_elev_m * coslat)).to_degrees();
        let h_e = self.sample_contact_height_at(lat_deg, lon_deg + dlon, prefer_level);
        let h_w = self.sample_contact_height_at(lat_deg, lon_deg - dlon, prefer_level);
        let h_n = self.sample_contact_height_at(lat_deg + dlat, lon_deg, prefer_level);
        let h_s = self.sample_contact_height_at(lat_deg - dlat, lon_deg, prefer_level);
        let dzde = (h_e - h_w) / (2.0 * step_m);
        let dzdn = (h_n - h_s) / (2.0 * step_m);
        let inv = (dzde * dzde + dzdn * dzdn + 1.0).sqrt();
        ContactDetail {
            height_m: height_m as f64,
            normal_enu: [-dzde / inv, -dzdn / inv, 1.0 / inv],
            level_used,
            kind,
        }
    }

    fn sample_elev(
        &self,
        layer: &str,
        lat_deg: f64,
        lon_deg: f64,
        prefer_level: u32,
        nodata: Option<u16>,
    ) -> f32 {
        self.sample_elev_detailed(layer, lat_deg, lon_deg, prefer_level, nodata)
            .0
    }

    /// 如 [`Self::sample_elev`]，另回传实际命中的层级（回退后）。
    fn sample_elev_detailed(
        &self,
        layer: &str,
        lat_deg: f64,
        lon_deg: f64,
        prefer_level: u32,
        nodata: Option<u16>,
    ) -> (f32, u32) {
        let layer_id: u8 = match layer {
            "terrain" => 0,
            "water_surface" => 1,
            _ => 2,
        };
        let prefer = prefer_level.min(self.max_level(layer));
        let dir = self.root.join(self.manifest.layer_path(layer));
        {
            let mut cache = self.elev_cache.lock().unwrap();
            let mut level = prefer;
            loop {
                let key = quad::tile_covering(level, lon_deg, lat_deg);
                if let Some(tile) = cache.get(&(layer_id, key)) {
                    return (tile.sample_bilinear(lon_deg, lat_deg, key), key.level);
                }
                if level == 0 {
                    break;
                }
                level -= 1;
            }
        }
        let (key, tile) = load_elev_with_fallback(&dir, lon_deg, lat_deg, prefer, nodata)
            .unwrap_or((
                QKey {
                    level: 0,
                    x: 0,
                    y: 0,
                },
                ElevTile::Flat { height_m: 0.0 },
            ));
        let v = tile.sample_bilinear(lon_deg, lat_deg, key);
        self.elev_cache.lock().unwrap().insert((layer_id, key), tile);
        (v, key.level)
    }
}

fn tile_center_lonlat(key: QKey) -> (f64, f64) {
    let c = quad::cols(key.level) as f64;
    let r = quad::rows(key.level) as f64;
    let u = (key.x as f64 + 0.5) / c;
    let v = (key.y as f64 + 0.5) / r;
    let lon = u * 360.0 - 180.0;
    let lat = 90.0 - v * 180.0;
    (lon, lat)
}

#[cfg(test)]
mod tests {
    use super::*;
    use elev::testutil::{write_f32_z0, write_flat_tile, write_i16_tile};
    use image::{GrayImage, Luma};

    fn write_manifest(dir: &Path, body: &str) {
        fs::write(dir.join("manifest.toml"), body).unwrap();
    }

    fn write_wc_png(dir: &Path, key: QKey, w: u32, h: u32, fill: u8) {
        fs::create_dir_all(dir).unwrap();
        let img = GrayImage::from_fn(w, h, |_, _| Luma([fill]));
        img.save(dir.join(format!("z{}_x{}_y{}.png", key.level, key.x, key.y)))
            .unwrap();
    }

    fn write_wc_half(dir: &Path, key: QKey, w: u32, h: u32) {
        // left half land(0), right half ocean(1) — nearest must not lerp to 0.5
        fs::create_dir_all(dir).unwrap();
        let img = GrayImage::from_fn(w, h, |x, _| {
            if x < w / 2 {
                Luma([0])
            } else {
                Luma([1])
            }
        });
        img.save(dir.join(format!("z{}_x{}_y{}.png", key.level, key.x, key.y)))
            .unwrap();
    }

    #[test]
    fn terrain_i16_unsigned_high_half() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path();
        write_manifest(
            root,
            r#"
format_version = 2
dataset_id = "t_i16"
R_elev_m = 6371010.0
ocean_surface_m = 0.0
[layers.terrain]
path = "terrain"
format = "int16_le_i16"
max_level = 1
[layers.water_class]
path = "water_class"
format = "png_u8"
max_level = 0
[layers.water_surface]
path = "water_surface"
format = "int16_le_i16"
max_level = 0
"#,
        );
        // z0 required for fallback chain
        write_f32_z0(&root.join("terrain"), 2, 2, &[0.0; 4]);
        write_wc_png(
            &root.join("water_class"),
            QKey {
                level: 0,
                x: 0,
                y: 0,
            },
            2,
            2,
            0,
        );
        write_f32_z0(&root.join("water_surface"), 2, 2, &[f32::NAN; 4]);

        // raw=40000 (>32768): signed would be negative; unsigned → positive height
        let scale = 0.1f32;
        let offset = 100.0f32;
        let raw: u16 = 40000;
        let expect = offset + (raw as f32) * scale;
        write_i16_tile(
            &root.join("terrain"),
            QKey {
                level: 1,
                x: 0,
                y: 0,
            },
            2,
            2,
            scale,
            offset,
            &[raw; 4],
        );
        write_i16_tile(
            &root.join("terrain"),
            QKey {
                level: 1,
                x: 1,
                y: 0,
            },
            2,
            2,
            scale,
            offset,
            &[raw; 4],
        );

        let pkg = SurfacePackage::open(root).unwrap();
        let h = pkg.sample_terrain_at(0.0, -90.0, 1);
        assert!(
            (h - expect as f64).abs() < 1e-3,
            "got {h} expect {expect} (unsigned high-half)"
        );
        // Signed mis-decode would be offset + (i16::from_le_bytes(raw) as f32)*scale ≈ negative
        let signed_wrong = offset as f64 + (raw as i16 as f64) * scale as f64;
        assert!(
            (h - signed_wrong).abs() > 100.0,
            "must not match signed decode {signed_wrong}"
        );
    }

    #[test]
    fn flat_tile_dtype0() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path();
        write_manifest(
            root,
            r#"
format_version = 2
dataset_id = "flat"
R_elev_m = 6371010.0
ocean_surface_m = 0.0
[layers.terrain]
path = "terrain"
format = "int16_le_i16"
max_level = 1
[layers.water_class]
path = "water_class"
format = "png_u8"
max_level = 0
[layers.water_surface]
path = "water_surface"
format = "int16_le_i16"
max_level = 0
"#,
        );
        write_f32_z0(&root.join("terrain"), 2, 2, &[999.0; 4]);
        write_flat_tile(
            &root.join("terrain"),
            QKey {
                level: 1,
                x: 0,
                y: 0,
            },
            2,
            2,
            0.0,
        );
        write_flat_tile(
            &root.join("terrain"),
            QKey {
                level: 1,
                x: 1,
                y: 0,
            },
            2,
            2,
            0.0,
        );
        write_wc_png(
            &root.join("water_class"),
            QKey {
                level: 0,
                x: 0,
                y: 0,
            },
            2,
            2,
            0,
        );
        write_f32_z0(&root.join("water_surface"), 2, 2, &[f32::NAN; 4]);

        let pkg = SurfacePackage::open(root).unwrap();
        let h = pkg.sample_terrain_at(0.0, -90.0, 1);
        assert_eq!(h, 0.0);
    }

    #[test]
    fn contact_detail_reports_level_and_up_normal() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path();
        write_manifest(
            root,
            r#"
format_version = 2
dataset_id = "detail"
R_elev_m = 6371010.0
ocean_surface_m = 0.0
[layers.terrain]
path = "terrain"
format = "int16_le_i16"
max_level = 1
[layers.water_class]
path = "water_class"
format = "png_u8"
max_level = 0
[layers.water_surface]
path = "water_surface"
format = "int16_le_i16"
max_level = 0
"#,
        );
        write_f32_z0(&root.join("terrain"), 2, 2, &[30.0; 4]);
        write_flat_tile(
            &root.join("terrain"),
            QKey { level: 1, x: 0, y: 0 },
            2,
            2,
            30.0,
        );
        write_flat_tile(
            &root.join("terrain"),
            QKey { level: 1, x: 1, y: 0 },
            2,
            2,
            30.0,
        );
        write_wc_png(
            &root.join("water_class"),
            QKey { level: 0, x: 0, y: 0 },
            2,
            2,
            0,
        );
        write_f32_z0(&root.join("water_surface"), 2, 2, &[f32::NAN; 4]);

        let pkg = SurfacePackage::open(root).unwrap();
        let d = pkg.sample_contact_detail_at(0.0, -90.0, 1);
        assert!((d.height_m - 30.0).abs() < 1e-3);
        assert_eq!(d.level_used, 1);
        assert_eq!(d.kind, WaterClass::Land);
        // 平地法向 = 正上
        assert!(d.normal_enu[2] > 0.999, "normal {:?}", d.normal_enu);
        // 请求层高于实际（缺瓦回退）→ level 反映回退结果由 lod 测试覆盖
    }

    #[test]
    fn water_surface_nodata_65535() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path();
        write_manifest(
            root,
            r#"
format_version = 2
dataset_id = "ws_nd"
R_elev_m = 6371010.0
ocean_surface_m = 0.0
[layers.terrain]
path = "terrain"
format = "int16_le_i16"
max_level = 0
[layers.water_class]
path = "water_class"
format = "png_u8"
max_level = 0
[layers.water_surface]
path = "water_surface"
format = "int16_le_i16"
max_level = 1
"#,
        );
        write_f32_z0(&root.join("terrain"), 2, 2, &[12.0; 4]);
        write_f32_z0(&root.join("water_surface"), 2, 2, &[0.0; 4]);
        write_i16_tile(
            &root.join("water_surface"),
            QKey {
                level: 1,
                x: 0,
                y: 0,
            },
            2,
            2,
            1.0,
            0.0,
            &[u16::MAX; 4],
        );
        write_i16_tile(
            &root.join("water_surface"),
            QKey {
                level: 1,
                x: 1,
                y: 0,
            },
            2,
            2,
            1.0,
            0.0,
            &[u16::MAX; 4],
        );
        write_wc_png(
            &root.join("water_class"),
            QKey {
                level: 0,
                x: 0,
                y: 0,
            },
            2,
            2,
            1, // ocean → contact uses wsurf
        );

        let pkg = SurfacePackage::open(root).unwrap();
        assert!(pkg.sample_water_surface(0.0, -90.0).is_none());
        // contact falls back to terrain
        assert!((pkg.sample_contact_height(0.0, -90.0) - 12.0).abs() < 1e-3);
    }

    #[test]
    fn water_class_nearest_not_lerp() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path();
        write_manifest(
            root,
            r#"
format_version = 2
dataset_id = "wc"
R_elev_m = 6371010.0
ocean_surface_m = 0.0
[layers.terrain]
path = "terrain"
format = "float32_le"
max_level = 0
[layers.water_class]
path = "water_class"
format = "png_u8"
max_level = 0
[layers.water_surface]
path = "water_surface"
format = "float32_le"
max_level = 0
"#,
        );
        write_f32_z0(&root.join("terrain"), 4, 2, &[0.0; 8]);
        write_f32_z0(&root.join("water_surface"), 4, 2, &[0.0; 8]);
        write_wc_half(
            &root.join("water_class"),
            QKey {
                level: 0,
                x: 0,
                y: 0,
            },
            4,
            2,
        );
        let pkg = SurfacePackage::open(root).unwrap();
        // Just left of mid-meridian → land; just right → ocean
        assert_eq!(pkg.sample_water_class(0.0, -1.0), WaterClass::Land);
        assert_eq!(pkg.sample_water_class(0.0, 1.0), WaterClass::Ocean);
    }

    #[test]
    fn contact_height_land_vs_water() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path();
        write_manifest(
            root,
            r#"
format_version = 2
dataset_id = "contact"
R_elev_m = 6371010.0
ocean_surface_m = 0.0
[layers.terrain]
path = "terrain"
format = "float32_le"
max_level = 0
[layers.water_class]
path = "water_class"
format = "png_u8"
max_level = 0
[layers.water_surface]
path = "water_surface"
format = "float32_le"
max_level = 0
"#,
        );
        // 2x1: left land terrain=50, right ocean wsurf=0
        write_f32_z0(&root.join("terrain"), 2, 1, &[50.0, -100.0]);
        write_f32_z0(&root.join("water_surface"), 2, 1, &[f32::NAN, 0.0]);
        let img = GrayImage::from_fn(2, 1, |x, _| Luma([if x == 0 { 0 } else { 1 }]));
        fs::create_dir_all(root.join("water_class")).unwrap();
        img.save(root.join("water_class/z0_x0_y0.png")).unwrap();

        let pkg = SurfacePackage::open(root).unwrap();
        let land = pkg.sample_contact_height(0.0, -90.0);
        let sea = pkg.sample_contact_height(0.0, 90.0);
        assert!((land - 50.0).abs() < 1.0, "land contact {land}");
        assert!(sea.abs() < 1.0, "sea contact {sea}");
    }

    #[test]
    fn lod_ancestor_fallback() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path();
        write_manifest(
            root,
            r#"
format_version = 2
dataset_id = "lod"
R_elev_m = 6371010.0
ocean_surface_m = 0.0
[layers.terrain]
path = "terrain"
format = "int16_le_i16"
max_level = 2
[layers.water_class]
path = "water_class"
format = "png_u8"
max_level = 0
[layers.water_surface]
path = "water_surface"
format = "float32_le"
max_level = 0
"#,
        );
        // Only z0 present; prefer z2 must fall back
        write_f32_z0(&root.join("terrain"), 2, 2, &[42.0; 4]);
        write_f32_z0(&root.join("water_surface"), 2, 2, &[0.0; 4]);
        write_wc_png(
            &root.join("water_class"),
            QKey {
                level: 0,
                x: 0,
                y: 0,
            },
            2,
            2,
            0,
        );
        let pkg = SurfacePackage::open(root).unwrap();
        let h = pkg.sample_terrain_at(10.0, 20.0, 2);
        assert!((h - 42.0).abs() < 1e-3, "fallback got {h}");
    }

    #[test]
    fn open_rejects_bad_version() {
        let tmp = tempfile::tempdir().unwrap();
        write_manifest(tmp.path(), "format_version = 1\n");
        assert!(SurfacePackage::open(tmp.path()).is_err());
    }

    #[test]
    fn lru_evicts_when_over_cap() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path();
        write_manifest(
            root,
            r#"
format_version = 2
dataset_id = "lru"
R_elev_m = 6371010.0
ocean_surface_m = 0.0
[layers.terrain]
path = "terrain"
format = "float32_le"
max_level = 2
[layers.water_class]
path = "water_class"
format = "png_u8"
max_level = 0
[layers.water_surface]
path = "water_surface"
format = "float32_le"
max_level = 0
"#,
        );
        write_f32_z0(&root.join("terrain"), 2, 2, &[1.0; 4]);
        write_f32_z0(&root.join("water_surface"), 2, 2, &[0.0; 4]);
        write_wc_png(
            &root.join("water_class"),
            QKey {
                level: 0,
                x: 0,
                y: 0,
            },
            2,
            2,
            0,
        );
        // Distinct z2 tiles across the globe (flat).
        for (x, y) in [(0u32, 0u32), (1, 0), (2, 0), (3, 0)] {
            write_flat_tile(
                &root.join("terrain"),
                QKey { level: 2, x, y },
                2,
                2,
                10.0 + x as f32,
            );
        }

        let pkg = SurfacePackage::open_with_cache_cap(root, 2).unwrap();
        // Sample four distant longitudes at prefer z2 → four elev inserts; cap=2.
        let _ = pkg.sample_terrain_at(0.0, -135.0, 2);
        let _ = pkg.sample_terrain_at(0.0, -45.0, 2);
        let _ = pkg.sample_terrain_at(0.0, 45.0, 2);
        let _ = pkg.sample_terrain_at(0.0, 135.0, 2);
        let st = pkg.cache_stats();
        assert!(st.elev_tiles <= 2, "elev cache must be bounded, got {}", st.elev_tiles);
        assert_eq!(st.elev_cap, 2);
    }

    #[test]
    fn prefetch_around_fills_cache() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path();
        write_manifest(
            root,
            r#"
format_version = 2
dataset_id = "pf"
R_elev_m = 6371010.0
ocean_surface_m = 0.0
[layers.terrain]
path = "terrain"
format = "float32_le"
max_level = 1
[layers.water_class]
path = "water_class"
format = "png_u8"
max_level = 0
[layers.water_surface]
path = "water_surface"
format = "float32_le"
max_level = 0
"#,
        );
        write_f32_z0(&root.join("terrain"), 2, 2, &[7.0; 4]);
        write_f32_z0(&root.join("water_surface"), 2, 2, &[0.0; 4]);
        write_wc_png(
            &root.join("water_class"),
            QKey {
                level: 0,
                x: 0,
                y: 0,
            },
            2,
            2,
            0,
        );
        write_flat_tile(
            &root.join("terrain"),
            QKey {
                level: 1,
                x: 0,
                y: 0,
            },
            2,
            2,
            8.0,
        );
        write_flat_tile(
            &root.join("terrain"),
            QKey {
                level: 1,
                x: 1,
                y: 0,
            },
            2,
            2,
            9.0,
        );

        let pkg = SurfacePackage::open(root).unwrap();
        assert_eq!(pkg.cache_stats().elev_tiles, 0);
        pkg.prefetch_around(0.0, -90.0, 1, 1).unwrap();
        let st = pkg.cache_stats();
        assert!(st.elev_tiles >= 1, "prefetch should load tiles");
        assert!(st.class_tiles >= 1);
        // Sampling after prefetch must stay green
        let h = pkg.sample_contact_height_at(0.0, -90.0, 1);
        assert!((h - 8.0).abs() < 1e-3 || (h - 7.0).abs() < 1e-3, "h={h}");
    }
}
