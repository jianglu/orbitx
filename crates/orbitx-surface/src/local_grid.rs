//! 发射场局部高程覆盖网格（`lp_xxx.toml` 伴生 f32 数据）。
//!
//! 当地 ENU 切平面规则格、原点居中；行主序存储，row 0 = 北缘、col 0 = 西缘。
//! 采样为无状态纯函数：足迹内返回覆盖值（权威替换），足迹外 `None`（硬切换）。
//! 公式与 `docs/SURFACE_RUNTIME.md` 一致性规范一致，Godot 侧按同一规范实现。

/// 覆盖网格采样结果。
#[derive(Debug, Clone, Copy)]
pub struct GridSample {
    /// 相对高程球的绝对地面高（m），与全球包接触高同口径。
    pub height_m: f64,
    /// 地面法向的 ENU 分量（在查询点当地系下，单位向量）。
    pub normal_enu: [f64; 3],
}

/// 当地 ENU 基（地心系分量）：纬度 = Y 北、经度从 +X 向 +Z。
pub fn enu_basis(lat_deg: f64, lon_deg: f64) -> ([f64; 3], [f64; 3], [f64; 3]) {
    let lat = lat_deg.to_radians();
    let lon = lon_deg.to_radians();
    let (slat, clat) = (lat.sin(), lat.cos());
    let (slon, clon) = (lon.sin(), lon.cos());
    let up = [clat * clon, slat, clat * slon];
    let east = [-slon, 0.0, clon];
    let north = [-slat * clon, clat, -slat * slon];
    (east, north, up)
}

/// 经纬度 → 地心单位向量（与 `enu_basis` 的 up 一致）。
pub fn latlon_up(lat_deg: f64, lon_deg: f64) -> [f64; 3] {
    enu_basis(lat_deg, lon_deg).2
}

fn dot3(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// 发射场局部高程覆盖网格（全量常驻内存；采样零 IO）。
pub struct LocalElevationGrid {
    origin_lat_deg: f64,
    origin_lon_deg: f64,
    r_elev_m: f64,
    cell_m: f64,
    width: usize,
    height: usize,
    data: Vec<f32>,
    east: [f64; 3],
    north: [f64; 3],
    origin_ecef: [f64; 3],
}

impl std::fmt::Debug for LocalElevationGrid {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("LocalElevationGrid")
            .field("origin_lat_deg", &self.origin_lat_deg)
            .field("origin_lon_deg", &self.origin_lon_deg)
            .field("cell_m", &self.cell_m)
            .field("width", &self.width)
            .field("height", &self.height)
            .finish()
    }
}

impl LocalElevationGrid {
    pub fn new(
        origin_lat_deg: f64,
        origin_lon_deg: f64,
        r_elev_m: f64,
        cell_m: f64,
        width: usize,
        height: usize,
        data: Vec<f32>,
    ) -> Result<Self, String> {
        if !(cell_m > 0.0) || width < 2 || height < 2 {
            return Err("覆盖网格参数非法（cell>0 且宽高≥2）".into());
        }
        if data.len() != width * height {
            return Err(format!(
                "覆盖网格数据长度 {} 与 {}×{} 不符",
                data.len(),
                width,
                height
            ));
        }
        if data.iter().any(|v| !v.is_finite()) {
            return Err("覆盖网格含非有限值".into());
        }
        if !(r_elev_m > 0.0) {
            return Err("覆盖网格 R_elev 非法".into());
        }
        let (east, north, up) = enu_basis(origin_lat_deg, origin_lon_deg);
        let origin_ecef = [up[0] * r_elev_m, up[1] * r_elev_m, up[2] * r_elev_m];
        Ok(Self {
            origin_lat_deg,
            origin_lon_deg,
            r_elev_m,
            cell_m,
            width,
            height,
            data,
            east,
            north,
            origin_ecef,
        })
    }

    pub fn origin_lat_deg(&self) -> f64 {
        self.origin_lat_deg
    }

    pub fn origin_lon_deg(&self) -> f64 {
        self.origin_lon_deg
    }

    /// 足迹半边长（m）：外圈节点围成的矩形。
    pub fn half_extent_m(&self) -> (f64, f64) {
        (
            (self.width - 1) as f64 * self.cell_m / 2.0,
            (self.height - 1) as f64 * self.cell_m / 2.0,
        )
    }

    /// 经纬度 → 相对原点的当地 ENU 偏移（相对原点地心点做基投影）。
    pub fn latlon_to_east_north(&self, lat_deg: f64, lon_deg: f64) -> (f64, f64) {
        let up = latlon_up(lat_deg, lon_deg);
        let p = [
            up[0] * self.r_elev_m,
            up[1] * self.r_elev_m,
            up[2] * self.r_elev_m,
        ];
        let d = [
            p[0] - self.origin_ecef[0],
            p[1] - self.origin_ecef[1],
            p[2] - self.origin_ecef[2],
        ];
        (dot3(d, self.east), dot3(d, self.north))
    }

    /// 当地 ENU 偏移 → 经纬度（切平面点方向）。
    pub fn east_north_to_latlon(&self, east_m: f64, north_m: f64) -> (f64, f64) {
        let p = [
            self.origin_ecef[0] + self.east[0] * east_m + self.north[0] * north_m,
            self.origin_ecef[1] + self.east[1] * east_m + self.north[1] * north_m,
            self.origin_ecef[2] + self.east[2] * east_m + self.north[2] * north_m,
        ];
        let r = (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt().max(1e-9);
        let lat = (p[1] / r).clamp(-1.0, 1.0).asin().to_degrees();
        let lon = p[2].atan2(p[0]).to_degrees();
        (lat, lon)
    }

    /// 点是否在足迹内（边界算内；带 0.1 mm 投影往返容差）。
    pub fn contains_east_north(&self, east_m: f64, north_m: f64) -> bool {
        let (ex, ey) = self.half_extent_m();
        east_m.abs() <= ex + 1e-4 && north_m.abs() <= ey + 1e-4
    }

    fn node_height(&self, col: usize, row: usize) -> f64 {
        self.data[row * self.width + col] as f64
    }

    /// 足迹内双线性高度（足迹外 None）。
    pub fn height_at_east_north(&self, east_m: f64, north_m: f64) -> Option<f64> {
        if !self.contains_east_north(east_m, north_m) {
            return None;
        }
        let fx = (east_m / self.cell_m + (self.width - 1) as f64 / 2.0)
            .clamp(0.0, (self.width - 1) as f64);
        let fy = ((self.height - 1) as f64 / 2.0 - north_m / self.cell_m)
            .clamp(0.0, (self.height - 1) as f64);
        let x0 = (fx.floor() as usize).min(self.width - 2);
        let y0 = (fy.floor() as usize).min(self.height - 2);
        let tx = fx - x0 as f64;
        let ty = fy - y0 as f64;
        let v00 = self.node_height(x0, y0);
        let v10 = self.node_height(x0 + 1, y0);
        let v01 = self.node_height(x0, y0 + 1);
        let v11 = self.node_height(x0 + 1, y0 + 1);
        Some(
            v00 * (1.0 - tx) * (1.0 - ty)
                + v10 * tx * (1.0 - ty)
                + v01 * (1.0 - tx) * ty
                + v11 * tx * ty,
        )
    }

    /// 按经纬度采样：足迹内 Some（高程 + 法向），足迹外 None。
    pub fn sample(&self, lat_deg: f64, lon_deg: f64) -> Option<GridSample> {
        let (e, n) = self.latlon_to_east_north(lat_deg, lon_deg);
        let height_m = self.height_at_east_north(e, n)?;
        let (ex, ey) = self.half_extent_m();
        let de = self.cell_m;
        let e_lo = (e - de).max(-ex);
        let e_hi = (e + de).min(ex);
        let n_lo = (n - de).max(-ey);
        let n_hi = (n + de).min(ey);
        let dzde = (self.height_at_east_north(e_hi, n)? - self.height_at_east_north(e_lo, n)?)
            / (e_hi - e_lo).max(1e-9);
        let dzdn = (self.height_at_east_north(e, n_hi)? - self.height_at_east_north(e, n_lo)?)
            / (n_hi - n_lo).max(1e-9);
        let inv = (dzde * dzde + dzdn * dzdn + 1.0).sqrt();
        Some(GridSample {
            height_m,
            normal_enu: [-dzde / inv, -dzdn / inv, 1.0 / inv],
        })
    }

    /// 最外圈节点采样（供 runtime 与全球值比对告警）：返回 (lat, lon, height)。
    pub fn border_nodes(&self, max_count: usize) -> Vec<(f64, f64, f64)> {
        let mut pts = Vec::new();
        let step = (self.width.max(self.height) / max_count.max(1)).max(1);
        for col in (0..self.width).step_by(step) {
            for row in [0, self.height - 1] {
                pts.push(self.node_latlon(col, row));
            }
        }
        for row in (0..self.height).step_by(step) {
            for col in [0, self.width - 1] {
                pts.push(self.node_latlon(col, row));
            }
        }
        pts
    }

    fn node_latlon(&self, col: usize, row: usize) -> (f64, f64, f64) {
        let e = (col as f64 - (self.width - 1) as f64 / 2.0) * self.cell_m;
        let n = ((self.height - 1) as f64 / 2.0 - row as f64) * self.cell_m;
        let (lat, lon) = self.east_north_to_latlon(e, n);
        (lat, lon, self.node_height(col, row))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde::Deserialize;

    const R: f64 = 6_371_010.0;

    /// 平面 h(e,n) = 50 + 0.1e − 0.05n，5×5 @ 10 m。
    fn plane_grid() -> LocalElevationGrid {
        let (w, h, cell) = (5usize, 5usize, 10.0f64);
        let mut data = vec![0.0f32; w * h];
        for row in 0..h {
            for col in 0..w {
                let e = (col as f64 - 2.0) * cell;
                let n = (2.0 - row as f64) * cell;
                data[row * w + col] = (50.0 + 0.1 * e - 0.05 * n) as f32;
            }
        }
        LocalElevationGrid::new(28.5, -80.5, R, cell, w, h, data).unwrap()
    }

    #[test]
    fn footprint_hard_switch() {
        let g = plane_grid();
        // 原点
        let s = g.sample(28.5, -80.5).unwrap();
        assert!((s.height_m - 50.0).abs() < 1e-6);
        // 足迹外（东 100 m）→ None
        let (lat, lon) = g.east_north_to_latlon(100.0, 0.0);
        assert!(g.sample(lat, lon).is_none());
        // 角点边界算内
        let (lat, lon) = g.east_north_to_latlon(20.0, 20.0);
        let s = g.sample(lat, lon).unwrap();
        assert!((s.height_m - 51.0).abs() < 1e-3, "got {}", s.height_m);
    }

    #[test]
    fn bilinear_and_normal_on_plane() {
        let g = plane_grid();
        let (lat, lon) = g.east_north_to_latlon(10.0, -10.0);
        let s = g.sample(lat, lon).unwrap();
        assert!((s.height_m - 51.5).abs() < 1e-3);
        // 平面法向恒定
        let inv = (0.1f64.powi(2) + 0.05f64.powi(2) + 1.0).sqrt();
        assert!((s.normal_enu[0] - (-0.1 / inv)).abs() < 1e-3);
        assert!((s.normal_enu[1] - (0.05 / inv)).abs() < 1e-3);
        assert!((s.normal_enu[2] - (1.0 / inv)).abs() < 1e-3);
    }

    #[derive(Deserialize)]
    struct Golden {
        origin_lat_deg: f64,
        origin_lon_deg: f64,
        r_elev_m: f64,
        grid_cell_m: f64,
        grid_width: usize,
        grid_height: usize,
        origin_up: [f64; 3],
        samples: Vec<GoldenSample>,
    }

    #[derive(Deserialize)]
    struct GoldenSample {
        name: String,
        east_m: f64,
        north_m: f64,
        lat_deg: f64,
        lon_deg: f64,
        height_m: f64,
        normal_enu: [f64; 3],
    }

    #[test]
    fn golden_fixture_launchpad() {
        let text = include_str!("../fixtures/launchpad_golden.toml");
        let golden: Golden = toml::from_str(text).unwrap();
        // 原点地心单位向量
        let up = latlon_up(golden.origin_lat_deg, golden.origin_lon_deg);
        for i in 0..3 {
            assert!(
                (up[i] - golden.origin_up[i]).abs() < 1e-9,
                "origin_up[{i}]"
            );
        }
        let (w, h, cell) = (
            golden.grid_width,
            golden.grid_height,
            golden.grid_cell_m,
        );
        let mut data = vec![0.0f32; w * h];
        for row in 0..h {
            for col in 0..w {
                let e = (col as f64 - (w - 1) as f64 / 2.0) * cell;
                let n = ((h - 1) as f64 / 2.0 - row as f64) * cell;
                data[row * w + col] = (50.0 + 0.1 * e - 0.05 * n) as f32;
            }
        }
        let grid = LocalElevationGrid::new(
            golden.origin_lat_deg,
            golden.origin_lon_deg,
            golden.r_elev_m,
            cell,
            w,
            h,
            data,
        )
        .unwrap();
        for s in &golden.samples {
            // 经纬度 ↔ ENU 往返
            let (e, n) = grid.latlon_to_east_north(s.lat_deg, s.lon_deg);
            assert!(
                (e - s.east_m).abs() < 0.01 && (n - s.north_m).abs() < 0.01,
                "{}: enu ({e}, {n}) vs ({}, {})",
                s.name,
                s.east_m,
                s.north_m
            );
            let got = grid.sample(s.lat_deg, s.lon_deg).unwrap();
            assert!(
                (got.height_m - s.height_m).abs() < 0.01,
                "{}: height {} vs {}",
                s.name,
                got.height_m,
                s.height_m
            );
            for i in 0..3 {
                assert!(
                    (got.normal_enu[i] - s.normal_enu[i]).abs() < 1e-3,
                    "{}: normal[{i}]",
                    s.name
                );
            }
        }
    }
}
