//! 簇包络气动几何：AABB 侧视 + 迎风圆盘并集 + 筒体压心。
//!
//! 公式不在此；仅几何。算法见 `docs/AERO.md`。Godot / 级体：**+Y 纵轴**。

use orbitx_dynamics::TriaxialAreas;
use orbitx_math::{mul, Matrix3, Quat, Vec3};

use crate::supervessel::SubVesselData;
use crate::vessel::Vessel;

/// 刚体成员圆柱（簇体坐标），供迎风并集与背风 `R_body(y)`。
#[derive(Clone, Copy, Debug)]
pub struct ClusterCylinder {
    /// 轴线中点。
    pub center: Vec3,
    /// 单位轴向（体 +Y 经 `rrot`）。
    pub axis: Vec3,
    pub half_length: f64,
    pub radius: f64,
    /// 迎风圆盘圆心（XZ）。
    pub disk_cx: f64,
    pub disk_cz: f64,
}

impl ClusterCylinder {
    /// 轴向参数 `t ∈ [-half_length, half_length]` 对应点的 y；用于覆盖判定用端点 y 范围。
    pub fn y_extent(&self) -> (f64, f64) {
        let d = self.axis * self.half_length;
        let y0 = self.center.y - d.y;
        let y1 = self.center.y + d.y;
        (y0.min(y1), y0.max(y1))
    }

    /// 翼站位 `y` 是否落在该柱轴向投影区间内（教学级：按端点 y 包络）。
    pub fn covers_y(&self, y: f64) -> bool {
        let (lo, hi) = self.y_extent();
        y >= lo - 1e-9 && y <= hi + 1e-9
    }
}

/// 簇气动包络（拓扑变化时失效）。
#[derive(Clone, Debug)]
pub struct ClusterAeroGeom {
    pub areas: TriaxialAreas,
    /// 包络几何中部（簇体坐标）[m]。
    pub body_cop: Vec3,
    /// 成员圆柱（背风 `R_body(y)` / 调试）。
    pub cylinders: Vec<ClusterCylinder>,
}

impl Default for ClusterAeroGeom {
    fn default() -> Self {
        Self {
            areas: TriaxialAreas::default(),
            body_cop: Vec3::ZERO,
            cylinders: Vec::new(),
        }
    }
}

impl ClusterAeroGeom {
    /// 站位 `y` 处刚体截面外半径：覆盖该 y 的柱半径最大值；无覆盖则 0。
    pub fn body_radius_at_y(&self, y: f64) -> f64 {
        let mut r = 0.0_f64;
        for c in &self.cylinders {
            if c.covers_y(y) {
                r = r.max(c.radius);
            }
        }
        r
    }
}

#[derive(Clone, Copy, Debug)]
struct Aabb {
    min: Vec3,
    max: Vec3,
}

impl Aabb {
    fn empty() -> Self {
        Self {
            min: Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY),
            max: Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY),
        }
    }

    fn include_point(&mut self, p: Vec3) {
        self.min.x = self.min.x.min(p.x);
        self.min.y = self.min.y.min(p.y);
        self.min.z = self.min.z.min(p.z);
        self.max.x = self.max.x.max(p.x);
        self.max.y = self.max.y.max(p.y);
        self.max.z = self.max.z.max(p.z);
    }

    fn size(&self) -> Vec3 {
        self.max - self.min
    }

    fn center(&self) -> Vec3 {
        (self.min + self.max) * 0.5
    }

    fn is_valid(&self) -> bool {
        self.min.x.is_finite()
            && self.max.x.is_finite()
            && self.min.x <= self.max.x
            && self.min.y <= self.max.y
            && self.min.z <= self.max.z
    }
}

/// 圆柱体外接盒 8 角（体坐标：+Y 纵轴，半径 R，半长 L/2）。
fn cylinder_box_corners(length: f64, radius: f64) -> [Vec3; 8] {
    let hy = length * 0.5;
    let r = radius.max(0.0);
    let xs = [-r, r];
    let ys = [-hy, hy];
    let zs = [-r, r];
    let mut out = [Vec3::ZERO; 8];
    let mut i = 0;
    for &x in &xs {
        for &y in &ys {
            for &z in &zs {
                out[i] = Vec3::new(x, y, z);
                i += 1;
            }
        }
    }
    out
}

/// 两圆并面积（XZ 平面）。
fn two_disk_union(c0: (f64, f64, f64), c1: (f64, f64, f64)) -> f64 {
    let (x0, z0, r0) = c0;
    let (x1, z1, r1) = c1;
    let a0 = std::f64::consts::PI * r0 * r0;
    let a1 = std::f64::consts::PI * r1 * r1;
    if r0 <= 1e-15 {
        return a1;
    }
    if r1 <= 1e-15 {
        return a0;
    }
    let dx = x1 - x0;
    let dz = z1 - z0;
    let d = (dx * dx + dz * dz).sqrt();
    if d >= r0 + r1 {
        return a0 + a1;
    }
    if d <= (r0 - r1).abs() {
        return a0.max(a1);
    }
    let r0_2 = r0 * r0;
    let r1_2 = r1 * r1;
    let alpha = ((d * d + r0_2 - r1_2) / (2.0 * d * r0)).clamp(-1.0, 1.0).acos();
    let beta = ((d * d + r1_2 - r0_2) / (2.0 * d * r1)).clamp(-1.0, 1.0).acos();
    let inter = r0_2 * alpha + r1_2 * beta
        - 0.5
            * (-d + r0 + r1)
            * (d + r0 - r1)
            * (d - r0 + r1)
            * (d + r0 + r1)
            .max(0.0)
            .sqrt();
    a0 + a1 - inter
}

/// 多圆面积并集：n≤2 解析；否则在包围盒上栅格采样（拓扑时一次，教学级）。
pub fn disk_union_area(disks: &[(f64, f64, f64)]) -> f64 {
    let disks: Vec<_> = disks
        .iter()
        .copied()
        .filter(|(_, _, r)| *r > 1e-15)
        .collect();
    match disks.len() {
        0 => 0.0,
        1 => {
            let r = disks[0].2;
            std::f64::consts::PI * r * r
        }
        2 => two_disk_union(disks[0], disks[1]),
        _ => disk_union_area_grid(&disks, 192),
    }
}

fn disk_union_area_grid(disks: &[(f64, f64, f64)], n: usize) -> f64 {
    let mut min_x = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut min_z = f64::INFINITY;
    let mut max_z = f64::NEG_INFINITY;
    for &(cx, cz, r) in disks {
        min_x = min_x.min(cx - r);
        max_x = max_x.max(cx + r);
        min_z = min_z.min(cz - r);
        max_z = max_z.max(cz + r);
    }
    if !min_x.is_finite() || max_x <= min_x || max_z <= min_z {
        return 0.0;
    }
    let nx = n.max(8);
    let nz = n.max(8);
    let dx = (max_x - min_x) / nx as f64;
    let dz = (max_z - min_z) / nz as f64;
    let mut count = 0_u64;
    for ix in 0..nx {
        let x = min_x + (ix as f64 + 0.5) * dx;
        for iz in 0..nz {
            let z = min_z + (iz as f64 + 0.5) * dz;
            let mut inside = false;
            for &(cx, cz, r) in disks {
                let ex = x - cx;
                let ez = z - cz;
                if ex * ex + ez * ez <= r * r {
                    inside = true;
                    break;
                }
            }
            if inside {
                count += 1;
            }
        }
    }
    count as f64 * dx * dz
}

fn member_cylinder(v: &Vessel, c: &SubVesselData) -> ClusterCylinder {
    let axis = mul(c.rrot, Vec3::new(0.0, 1.0, 0.0));
    let alen = axis.length();
    let axis = if alen > 1e-12 {
        axis * (1.0 / alen)
    } else {
        Vec3::new(0.0, 1.0, 0.0)
    };
    let center = c.rpos;
    ClusterCylinder {
        center,
        axis,
        half_length: v.length.max(0.0) * 0.5,
        radius: v.radius.max(0.0),
        disk_cx: center.x,
        disk_cz: center.z,
    }
}

/// 由簇成员（船原点 `rpos`/`rrot` + length/radius）算包络气动几何。
pub fn compute_cluster_aero_geom(
    vessels: &[Vessel],
    components: &[SubVesselData],
) -> ClusterAeroGeom {
    let mut aabb = Aabb::empty();
    let mut cylinders = Vec::with_capacity(components.len());
    let mut disks = Vec::with_capacity(components.len());
    for c in components {
        let v = &vessels[c.vessel_index];
        for corner in cylinder_box_corners(v.length, v.radius) {
            let p = c.rpos + mul(c.rrot, corner);
            aabb.include_point(p);
        }
        let cyl = member_cylinder(v, c);
        disks.push((cyl.disk_cx, cyl.disk_cz, cyl.radius));
        cylinders.push(cyl);
    }
    if !aabb.is_valid() || components.is_empty() {
        return ClusterAeroGeom::default();
    }
    let size = aabb.size();
    let frontal = disk_union_area(&disks);
    let areas = TriaxialAreas {
        x: size.y.max(0.0) * size.z.max(0.0),
        y: frontal,
        z: size.y.max(0.0) * size.x.max(0.0),
    };
    ClusterAeroGeom {
        areas,
        body_cop: aabb.center(),
        cylinders,
    }
}

/// 测试辅助：单船相对位姿构造 `SubVesselData`。
#[cfg(test)]
pub(crate) fn test_comp(vessel_index: usize, rpos: Vec3, rrot: Matrix3) -> SubVesselData {
    SubVesselData {
        vessel_index,
        rpos,
        rrot,
        rq: Quat::from_matrix(rrot),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::stage::StageSpec;
    use crate::vessel::Vessel;
    use orbitx_math::{Matrix3, StateVectors, Vec3};

    fn cyl_vessel(length: f64, radius: f64) -> Vessel {
        Vessel::from_spec(
            0,
            &StageSpec {
                name: "c",
                dry_mass: 1000.0,
                length,
                radius,
                separation_impulse: 0.0,
                ..Default::default()
            },
            StateVectors::default(),
        )
    }

    #[test]
    fn tandem_same_radius_does_not_double_frontal() {
        let mut a = cyl_vessel(10.0, 1.0);
        a.id = 0;
        let mut b = cyl_vessel(10.0, 1.0);
        b.id = 1;
        let vessels = vec![a, b];
        let comps = vec![
            test_comp(0, Vec3::new(0.0, 0.0, 0.0), Matrix3::IDENTITY),
            test_comp(1, Vec3::new(0.0, 10.0, 0.0), Matrix3::IDENTITY),
        ];
        let g = compute_cluster_aero_geom(&vessels, &comps);
        let single = std::f64::consts::PI;
        assert!(
            (g.areas.y - single).abs() < 1e-9,
            "frontal should be one disk, got {}",
            g.areas.y
        );
        assert!((g.areas.x - 20.0 * 2.0).abs() < 0.5, "side x={}", g.areas.x);
        assert!((g.body_cop.y - 5.0).abs() < 0.5, "cop y={}", g.body_cop.y);
    }

    #[test]
    fn strap_on_increases_frontal_union() {
        let mut core = cyl_vessel(20.0, 1.0);
        core.id = 0;
        let mut boost = cyl_vessel(16.0, 0.5);
        boost.id = 1;
        let vessels = vec![core, boost];
        let comps = vec![
            test_comp(0, Vec3::ZERO, Matrix3::IDENTITY),
            test_comp(1, Vec3::new(1.6, 0.0, 0.0), Matrix3::IDENTITY),
        ];
        let g = compute_cluster_aero_geom(&vessels, &comps);
        let core_only = std::f64::consts::PI;
        assert!(
            g.areas.y > core_only + 0.3,
            "strap-on union Sy={} should exceed core {}",
            g.areas.y,
            core_only
        );
        // 解析两圆并：不完全分离也不完全包含
        let expect = two_disk_union((0.0, 0.0, 1.0), (1.6, 0.0, 0.5));
        assert!(
            (g.areas.y - expect).abs() / expect < 1e-9,
            "Sy {} vs analytic {}",
            g.areas.y,
            expect
        );
    }

    #[test]
    fn odd_strap_on_roll_changes_transverse_share() {
        let mut core = cyl_vessel(20.0, 1.0);
        core.id = 0;
        let mut boost = cyl_vessel(16.0, 0.5);
        boost.id = 1;
        let vessels = vec![core, boost];
        let comps0 = vec![
            test_comp(0, Vec3::ZERO, Matrix3::IDENTITY),
            test_comp(1, Vec3::new(1.6, 0.0, 0.0), Matrix3::IDENTITY),
        ];
        let g0 = compute_cluster_aero_geom(&vessels, &comps0);
        let r90 = Matrix3::new(0.0, 0.0, 1.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0);
        let comps1 = vec![
            test_comp(0, Vec3::ZERO, Matrix3::IDENTITY),
            test_comp(1, mul(r90, Vec3::new(1.6, 0.0, 0.0)), Matrix3::IDENTITY),
        ];
        let g1 = compute_cluster_aero_geom(&vessels, &comps1);
        assert!(
            (g0.areas.x - g1.areas.x).abs() > 0.5 || (g0.areas.z - g1.areas.z).abs() > 0.5,
            "roll should swap transverse shares: before {:?}/{:?} after {:?}/{:?}",
            g0.areas.x,
            g0.areas.z,
            g1.areas.x,
            g1.areas.z
        );
        // 迎风并集面积对滚转不变（圆盘绕 Y）
        assert!((g0.areas.y - g1.areas.y).abs() < 1e-9);
        assert!(g0.areas.y > std::f64::consts::PI);
    }

    #[test]
    fn body_radius_at_y_uses_covering_cylinder() {
        let mut core = cyl_vessel(20.0, 1.0);
        core.id = 0;
        let vessels = vec![core];
        let comps = vec![test_comp(0, Vec3::ZERO, Matrix3::IDENTITY)];
        let g = compute_cluster_aero_geom(&vessels, &comps);
        assert!((g.body_radius_at_y(0.0) - 1.0).abs() < 1e-12);
        assert!(g.body_radius_at_y(100.0) < 1e-15);
    }
}
