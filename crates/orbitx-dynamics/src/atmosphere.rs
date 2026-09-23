//! 大气模型接口与实现（移植自 vessel `aero`）。
//!
//! - `Atmosphere` trait：密度 / 压力 / 温度 / 声速。
//! - `ExponentialAtmosphere`：Orbiter 默认简化指数衰减模型。
//! - `UsStd1976Atmosphere`：美国标准大气 1976 分层模型。
//! - `atmosphere_from_config`：由 `orbitx_config::AtmosphereConfig` 构造。

use orbitx_config::{AtmosphereConfig, AtmosphereModel};
use std::sync::Arc;

/// 大气模型接口。
pub trait Atmosphere: Send + Sync {
    /// 大气密度 [kg/m³]。
    fn density(&self, altitude: f64) -> f64;
    /// 大气压力 [Pa]。
    fn pressure(&self, altitude: f64) -> f64;
    /// 大气温度 [K]。
    fn temperature(&self, altitude: f64) -> f64;
    /// 比气体常数 [J/(kg·K)]。
    fn gas_constant(&self) -> f64 {
        287.058
    }
    /// 比热比 γ。
    fn gamma(&self) -> f64 {
        1.4
    }
    /// 当地声速 [m/s]：`a = √(γ R T)`。
    fn sound_speed(&self, altitude: f64) -> f64 {
        let t = self.temperature(altitude).max(1.0);
        (self.gamma() * self.gas_constant() * t).sqrt()
    }
    /// 返回一个密度闭包 `altitude → ρ`，可跨线程共享。
    fn density_fn(&self) -> Arc<dyn Fn(f64) -> f64 + Send + Sync>;
}

/// 指数衰减大气（Orbiter 默认简化模型）。
#[derive(Clone, Debug)]
pub struct ExponentialAtmosphere {
    pub rho0: f64,
    pub scale_height: f64,
    pub base_alt: f64,
    pub gas_constant: f64,
    pub gamma: f64,
    pub temperature0: f64,
}

impl ExponentialAtmosphere {
    pub fn earth() -> Self {
        Self {
            rho0: 1.225,
            scale_height: 8500.0,
            base_alt: 0.0,
            gas_constant: 287.058,
            gamma: 1.4,
            temperature0: 288.15,
        }
    }

    /// 从 TOML 大气配置构造。
    pub fn from_config(cfg: &AtmosphereConfig) -> Self {
        Self {
            rho0: cfg.density0,
            scale_height: cfg.scale_height,
            base_alt: 0.0,
            gas_constant: cfg.gas_constant,
            gamma: cfg.gamma,
            temperature0: if cfg.density0 > 1e-12 {
                cfg.pressure0 / (cfg.density0 * cfg.gas_constant)
            } else {
                288.15
            },
        }
    }
}

impl Atmosphere for ExponentialAtmosphere {
    fn density(&self, altitude: f64) -> f64 {
        if altitude < self.base_alt {
            return 0.0;
        }
        self.rho0 * (-(altitude - self.base_alt) / self.scale_height).exp()
    }

    fn pressure(&self, altitude: f64) -> f64 {
        self.density(altitude) * self.gas_constant * self.temperature0
    }

    fn temperature(&self, altitude: f64) -> f64 {
        let _ = altitude;
        self.temperature0
    }

    fn gas_constant(&self) -> f64 {
        self.gas_constant
    }

    fn gamma(&self) -> f64 {
        self.gamma
    }

    fn density_fn(&self) -> Arc<dyn Fn(f64) -> f64 + Send + Sync> {
        let rho0 = self.rho0;
        let scale_height = self.scale_height;
        let base_alt = self.base_alt;
        Arc::new(move |alt: f64| {
            if alt < base_alt {
                0.0
            } else {
                rho0 * (-(alt - base_alt) / scale_height).exp()
            }
        })
    }
}

/// 美国标准大气 1976（几何高度分层，教学用简化实现）。
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

/// 由 `AtmosphereConfig` 构造大气；`None` 或 model=none 返回 `None`。
pub fn atmosphere_from_config(cfg: Option<&AtmosphereConfig>) -> Option<Box<dyn Atmosphere>> {
    let cfg = cfg?;
    match cfg.model {
        AtmosphereModel::None => None,
        AtmosphereModel::Us76 => Some(Box::new(UsStd1976Atmosphere::with_limits(
            cfg.alt_limit,
            cfg.gas_constant,
            cfg.gamma,
        ))),
        AtmosphereModel::Exponential => Some(Box::new(ExponentialAtmosphere::from_config(cfg))),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exponential_sea_level() {
        let atm = ExponentialAtmosphere::earth();
        assert!((atm.density(0.0) - 1.225).abs() < 1e-6);
    }

    #[test]
    fn exponential_10km() {
        let atm = ExponentialAtmosphere::earth();
        let rho = atm.density(10_000.0);
        let expected = 1.225 * (-10_000.0_f64 / 8500.0).exp();
        assert!((rho - expected).abs() < 1e-6);
    }

    #[test]
    fn exponential_negative_alt_zero() {
        let atm = ExponentialAtmosphere::earth();
        assert_eq!(atm.density(-100.0), 0.0);
    }

    #[test]
    fn us76_density_magnitudes() {
        let atm = UsStd1976Atmosphere::new();
        let rho0 = atm.density(0.0);
        assert!((rho0 - 1.225).abs() < 0.02, "sea-level ρ = {rho0}");
        let rho11 = atm.density(11_000.0);
        assert!(rho11 > 0.3 && rho11 < 0.45, "11 km ρ = {rho11}");
        let rho25 = atm.density(25_000.0);
        assert!(rho25 > 0.03 && rho25 < 0.05, "25 km ρ = {rho25}");
        let rho50 = atm.density(50_000.0);
        assert!(rho50 > 5e-4 && rho50 < 2e-3, "50 km ρ = {rho50}");
    }

    #[test]
    fn us76_sound_speed_decreases_through_troposphere() {
        let atm = UsStd1976Atmosphere::new();
        let a0 = atm.sound_speed(0.0);
        let a11 = atm.sound_speed(11_000.0);
        assert!(a0 > 330.0 && a0 < 350.0, "a(0) = {a0}");
        assert!(a11 < a0 - 20.0, "tropopause colder → slower: {a11} vs {a0}");
    }

    #[test]
    fn atmosphere_from_config_branches() {
        let base = AtmosphereConfig {
            model: AtmosphereModel::Exponential,
            density0: 1.225,
            scale_height: 8500.0,
            pressure0: 101325.0,
            gas_constant: 287.0,
            gamma: 1.4,
            alt_limit: 200e3,
        };
        let exp = atmosphere_from_config(Some(&base)).unwrap();
        assert!((exp.density(0.0) - 1.225).abs() < 1e-6);

        let mut us = base.clone();
        us.model = AtmosphereModel::Us76;
        let u = atmosphere_from_config(Some(&us)).unwrap();
        assert!((u.density(0.0) - 1.225).abs() < 0.02);

        let mut none = base;
        none.model = AtmosphereModel::None;
        assert!(atmosphere_from_config(Some(&none)).is_none());
        assert!(atmosphere_from_config(None).is_none());
    }
}
