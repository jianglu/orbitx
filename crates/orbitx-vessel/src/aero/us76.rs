//! 美国标准大气 1976（几何高度分层，教学用简化实现）。

use super::Atmosphere;
use std::sync::Arc;

/// US Standard Atmosphere 1976（0–86 km 分层；以上外推至截止高度）。
#[derive(Clone, Debug)]
pub struct UsStd1976Atmosphere {
    pub alt_limit: f64,
    pub gas_constant: f64,
    pub gamma: f64,
}

impl Default for UsStd1976Atmosphere {
    fn default() -> Self {
        Self {
            alt_limit: 200e3,
            gas_constant: 287.05287,
            gamma: 1.4,
        }
    }
}

impl UsStd1976Atmosphere {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_limits(alt_limit: f64, gas_constant: f64, gamma: f64) -> Self {
        Self {
            alt_limit,
            gas_constant,
            gamma,
        }
    }

    /// 几何高度 → (T [K], P [Pa])。
    fn tp(&self, h: f64) -> (f64, f64) {
        // 分层：底高度 h_b [m], T_b [K], L [K/m], P_b [Pa]
        // 参考 USSA1976 公开表（海平面 ISA）。
        const G0: f64 = 9.80665;
        let layers: [(f64, f64, f64, f64); 8] = [
            (0.0, 288.15, -0.0065, 101_325.0),
            (11_000.0, 216.65, 0.0, 22_632.1),
            (20_000.0, 216.65, 0.001, 5474.89),
            (32_000.0, 228.65, 0.0028, 868.019),
            (47_000.0, 270.65, 0.0, 110.906),
            (51_000.0, 270.65, -0.0028, 66.9389),
            (71_000.0, 214.65, -0.002, 3.95642),
            (86_000.0, 186.946, 0.0, 0.3734),
        ];

        if h < 0.0 {
            return (layers[0].1, layers[0].3);
        }
        if h > self.alt_limit {
            return (layers[7].1, 0.0);
        }

        let mut idx = 0usize;
        for i in 0..layers.len() {
            if h >= layers[i].0 {
                idx = i;
            }
        }
        let (hb, tb, lapse, pb) = layers[idx];
        let dh = h - hb;
        let t = if lapse.abs() < 1e-12 {
            tb
        } else {
            tb + lapse * dh
        };
        let r = self.gas_constant;
        let p = if lapse.abs() < 1e-12 {
            pb * (-G0 * dh / (r * tb)).exp()
        } else {
            pb * (t / tb).powf(-G0 / (lapse * r))
        };
        (t.max(1.0), p.max(0.0))
    }
}

impl Atmosphere for UsStd1976Atmosphere {
    fn density(&self, altitude: f64) -> f64 {
        let (t, p) = self.tp(altitude);
        if p <= 0.0 {
            return 0.0;
        }
        p / (self.gas_constant * t)
    }

    fn pressure(&self, altitude: f64) -> f64 {
        self.tp(altitude).1
    }

    fn temperature(&self, altitude: f64) -> f64 {
        self.tp(altitude).0
    }

    fn gas_constant(&self) -> f64 {
        self.gas_constant
    }

    fn gamma(&self) -> f64 {
        self.gamma
    }

    fn density_fn(&self) -> Arc<dyn Fn(f64) -> f64 + Send + Sync> {
        let this = self.clone();
        Arc::new(move |alt: f64| this.density(alt))
    }
}
