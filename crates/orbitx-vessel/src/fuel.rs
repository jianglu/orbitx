//! 多储箱燃料系统（对应 Orbiter `TankSpec` / `CreatePropellantResource`）。
//!
//! 每个推进器可关联到特定储箱，从该储箱消耗燃料。
//! 若推进器无关联储箱（`tank_id = None`），则使用 Vessel 的
//! 旧式 `fuel_mass` 标量（向后兼容）。

/// 推进剂储箱（对应 Orbiter `TankSpec`，`Vessel.h:72`）。
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
    /// 燃料效率因子（Orbiter `efficiency`）。1.0 = 无损耗。
    pub efficiency: f64,
}

impl PropellantTank {
    /// 创建满储箱。
    pub fn new(id: u32, max_mass: f64, efficiency: f64) -> Self {
        Self {
            id,
            max_mass,
            mass: max_mass,
            prev_mass: max_mass,
            efficiency,
        }
    }

    /// 创建指定质量的储箱。
    pub fn with_mass(id: u32, max_mass: f64, mass: f64, efficiency: f64) -> Self {
        Self {
            id,
            max_mass,
            mass: mass.min(max_mass),
            prev_mass: mass.min(max_mass),
            efficiency,
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
        let consumed = mass.min(self.mass);
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
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_tank_is_full() {
        let tank = PropellantTank::new(1, 1000.0, 1.0);
        assert_eq!(tank.mass, 1000.0);
        assert_eq!(tank.percent(), 100.0);
        assert!(!tank.is_empty());
    }

    #[test]
    fn consume_reduces_mass() {
        let mut tank = PropellantTank::new(1, 1000.0, 1.0);
        let consumed = tank.consume(200.0);
        assert_eq!(consumed, 200.0);
        assert_eq!(tank.mass, 800.0);
    }

    #[test]
    fn consume_clamps_at_zero() {
        let mut tank = PropellantTank::new(1, 100.0, 1.0);
        let consumed = tank.consume(200.0);
        assert_eq!(consumed, 100.0);
        assert_eq!(tank.mass, 0.0);
        assert!(tank.is_empty());
    }

    #[test]
    fn flow_rate_computation() {
        let mut tank = PropellantTank::new(1, 1000.0, 1.0);
        tank.snapshot();
        tank.consume(50.0);
        let rate = tank.flow_rate(1.0);
        assert!((rate - 50.0).abs() < 1e-10, "流率 = {rate}");
    }

    #[test]
    fn efficiency_affects_consumption() {
        // efficiency < 1 → 消耗更多燃料达到相同推力。
        // 在 Thruster 层面：dm/dt = F / (eff * Isp * g0)。
        // 这里只验证 efficiency 字段存在且可读取。
        let tank = PropellantTank::new(1, 1000.0, 0.8);
        assert!((tank.efficiency - 0.8).abs() < 1e-10);
    }

    #[test]
    fn snapshot_tracks_prev_mass() {
        let mut tank = PropellantTank::new(1, 1000.0, 1.0);
        tank.consume(100.0);
        assert_eq!(tank.prev_mass, 1000.0); // 还没 snapshot
        tank.snapshot();
        assert_eq!(tank.prev_mass, 900.0);
        tank.consume(50.0);
        assert_eq!(tank.prev_mass, 900.0); // snapshot 不自动更新
    }

    #[test]
    fn with_mass_clamps() {
        let tank = PropellantTank::with_mass(1, 100.0, 200.0, 1.0);
        assert_eq!(tank.mass, 100.0); // clamped to max
    }
}
