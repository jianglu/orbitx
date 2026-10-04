//! 多储箱燃料系统（级内罐池）。
//!
//! 本级所有推进器共享本级全部储箱；消耗按各罐当前质量比例分摊。

use orbitx_math::Vec3;

/// 推进剂储箱。
#[derive(Clone, Debug)]
pub struct PropellantTank {
    /// 储箱唯一标识。
    pub id: u32,
    /// 最大燃料质量 [kg]。
    pub max_mass: f64,
    /// 当前燃料质量 [kg]。
    pub mass: f64,
    /// 上步燃料质量 [kg]（用于流率计算）。
    pub prev_mass: f64,
    /// 燃料效率因子。1.0 = 无损耗。
    pub efficiency: f64,
    /// 满燃料质心 [m]（体坐标，原点 = 满载质心）。
    pub pos: Vec3,
    /// 满罐主惯量对角线 [kg·m²]，绕 `pos`。
    pub inertia_full: Vec3,
}

impl PropellantTank {
    /// 创建满储箱。
    pub fn new(id: u32, max_mass: f64, pos: Vec3, inertia_full: Vec3, efficiency: f64) -> Self {
        Self {
            id,
            max_mass,
            mass: max_mass,
            prev_mass: max_mass,
            efficiency,
            pos,
            inertia_full,
        }
    }

    /// 创建指定质量的储箱。
    pub fn with_mass(
        id: u32,
        max_mass: f64,
        mass: f64,
        pos: Vec3,
        inertia_full: Vec3,
        efficiency: f64,
    ) -> Self {
        let m = mass.min(max_mass).max(0.0);
        Self {
            id,
            max_mass,
            mass: m,
            prev_mass: m,
            efficiency,
            pos,
            inertia_full,
        }
    }

    /// 燃料百分比（0..100）。
    pub fn percent(&self) -> f64 {
        if self.max_mass > 0.0 {
            (self.mass / self.max_mass * 100.0).min(100.0)
        } else {
            0.0
        }
    }

    /// 当前燃料惯量（绕 `pos`）：按 `mass/max_mass` 线性缩放满罐惯量。
    pub fn inertia_now(&self) -> Vec3 {
        if self.max_mass > 1e-12 {
            self.inertia_full * (self.mass / self.max_mass)
        } else {
            Vec3::ZERO
        }
    }

    /// 燃料流率 [kg/s]（基于 prev_mass 和 mass 的差值）。
    pub fn flow_rate(&self, dt: f64) -> f64 {
        if dt > 0.0 {
            (self.prev_mass - self.mass) / dt
        } else {
            0.0
        }
    }

    /// 消耗燃料 [kg]，返回实际消耗量。
    pub fn consume(&mut self, mass: f64) -> f64 {
        let consumed = mass.min(self.mass).max(0.0);
        self.mass -= consumed;
        if self.mass < 0.0 {
            self.mass = 0.0;
        }
        consumed
    }

    /// 记录当前质量为 prev_mass（每步开始调用）。
    pub fn snapshot(&mut self) {
        self.prev_mass = self.mass;
    }

    /// 是否已耗尽。
    pub fn is_empty(&self) -> bool {
        self.mass <= 0.0
    }
}

impl Default for PropellantTank {
    fn default() -> Self {
        Self {
            id: 0,
            max_mass: 0.0,
            mass: 0.0,
            prev_mass: 0.0,
            efficiency: 1.0,
            pos: Vec3::ZERO,
            inertia_full: Vec3::ZERO,
        }
    }
}

/// 将对角线惯量从自身质心平行轴平移到参考点 `to`（`from` 为自身质心）。
pub fn parallel_axis_diag(inertia: Vec3, mass: f64, from: Vec3, to: Vec3) -> Vec3 {
    let d = from - to;
    let dx2 = d.x * d.x;
    let dy2 = d.y * d.y;
    let dz2 = d.z * d.z;
    Vec3::new(
        inertia.x + mass * (dy2 + dz2),
        inertia.y + mass * (dx2 + dz2),
        inertia.z + mass * (dx2 + dy2),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_tank_is_full() {
        let tank = PropellantTank::new(1, 1000.0, Vec3::ZERO, Vec3::new(1.0, 1.0, 1.0), 1.0);
        assert_eq!(tank.mass, 1000.0);
        assert_eq!(tank.percent(), 100.0);
        assert!(!tank.is_empty());
    }

    #[test]
    fn consume_reduces_mass() {
        let mut tank = PropellantTank::new(1, 1000.0, Vec3::ZERO, Vec3::new(1.0, 1.0, 1.0), 1.0);
        let consumed = tank.consume(200.0);
        assert_eq!(consumed, 200.0);
        assert_eq!(tank.mass, 800.0);
    }

    #[test]
    fn inertia_scales_with_fill() {
        let full = Vec3::new(100.0, 50.0, 100.0);
        let mut tank = PropellantTank::new(1, 1000.0, Vec3::ZERO, full, 1.0);
        tank.mass = 500.0;
        let i = tank.inertia_now();
        assert!((i.x - 50.0).abs() < 1e-9);
        assert!((i.y - 25.0).abs() < 1e-9);
    }

    #[test]
    fn parallel_axis_increases() {
        let i0 = Vec3::new(1.0, 1.0, 1.0);
        let i1 = parallel_axis_diag(i0, 10.0, Vec3::new(0.0, 2.0, 0.0), Vec3::ZERO);
        assert!(i1.x > i0.x);
        assert!((i1.y - i0.y).abs() < 1e-9);
    }
}
