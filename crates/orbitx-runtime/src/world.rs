//! 世界：运行中的行星系统与火箭标识。

use orbitx_environment::PlanetarySystem;

pub struct World {
    pub psys: PlanetarySystem,
    pub rocket_name: String,
    pub rocket_class: String,
}

impl World {
    pub fn new(psys: PlanetarySystem, name: impl Into<String>, class: impl Into<String>) -> Self {
        Self {
            psys,
            rocket_name: name.into(),
            rocket_class: class.into(),
        }
    }
}
