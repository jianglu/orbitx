//! 世界壳：P4.2 过渡宿主 `PlanetarySystem`；P4.4 换 `orbitx-environment`。

/// P4.4 将替换为本类型对 `orbitx-environment` 的持有。
#[derive(Debug, Default)]
pub struct World {
    pub label: String,
    pub rocket_name: String,
    pub rocket_class: String,
}

impl World {
    pub fn placeholder() -> Self {
        Self {
            label: "PlanetarySystem-transition".into(),
            rocket_name: String::new(),
            rocket_class: String::new(),
        }
    }

    pub fn with_rocket(name: impl Into<String>, class: impl Into<String>) -> Self {
        Self {
            label: "PlanetarySystem-transition".into(),
            rocket_name: name.into(),
            rocket_class: class.into(),
        }
    }
}
