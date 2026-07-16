//! 太阳系配置（system.toml）。
//!
//! 对应 Orbiter 的 `Sol.cfg`：定义恒星→行星→卫星的树形结构。

use serde::{Deserialize, Serialize};

use super::body::BodyConfig;

/// 太阳系配置。
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SystemConfig {
    /// 系统名称。
    pub name: String,
    /// 恒星名称。
    pub star: String,
    /// 所有天体配置。
    pub bodies: Vec<BodyConfig>,
    /// 父子关系：`(child_name, parent_name)`。
    pub parents: Vec<(String, String)>,
}

impl SystemConfig {
    /// 从 TOML 字符串解析。
    pub fn from_toml_str(s: &str) -> Result<Self, toml::de::Error> {
        toml::from_str(s)
    }

    /// 序列化为 TOML 字符串。
    pub fn to_toml_string(&self) -> Result<String, toml::ser::Error> {
        toml::to_string_pretty(self)
    }

    /// 默认太阳系配置（对应 Orbiter Sol.cfg）。
    ///
    /// 包含：Sun + 8 行星 + Moon + 4 Galilean moons。
    pub fn sol() -> Self {
        use super::body::BodyConfig;
        let bodies = vec![
            BodyConfig::sun(),
            BodyConfig::mercury(),
            BodyConfig::venus(),
            BodyConfig::earth(),
            BodyConfig::mars(),
            BodyConfig::jupiter(),
            BodyConfig::saturn(),
            BodyConfig::uranus(),
            BodyConfig::neptune(),
            BodyConfig::moon(),
            BodyConfig::io(),
            BodyConfig::europa(),
            BodyConfig::ganymede(),
            BodyConfig::callisto(),
        ];
        let parents = vec![
            ("Mercury".to_string(), "Sun".to_string()),
            ("Venus".to_string(), "Sun".to_string()),
            ("Earth".to_string(), "Sun".to_string()),
            ("Mars".to_string(), "Sun".to_string()),
            ("Jupiter".to_string(), "Sun".to_string()),
            ("Saturn".to_string(), "Sun".to_string()),
            ("Uranus".to_string(), "Sun".to_string()),
            ("Neptune".to_string(), "Sun".to_string()),
            ("Moon".to_string(), "Earth".to_string()),
            ("Io".to_string(), "Jupiter".to_string()),
            ("Europa".to_string(), "Jupiter".to_string()),
            ("Ganymede".to_string(), "Jupiter".to_string()),
            ("Callisto".to_string(), "Jupiter".to_string()),
        ];
        Self {
            name: "Sol".to_string(),
            star: "Sun".to_string(),
            bodies,
            parents,
        }
    }

    /// 查找天体索引。
    pub fn body_index(&self, name: &str) -> Option<usize> {
        self.bodies.iter().position(|b| b.name == name)
    }

    /// 获取天体的父天体名称。
    pub fn parent_name(&self, child_name: &str) -> Option<&str> {
        self.parents
            .iter()
            .find(|(c, _)| c == child_name)
            .map(|(_, p)| p.as_str())
    }

    /// 获取天体的所有子天体名称。
    pub fn children_names(&self, parent_name: &str) -> Vec<&str> {
        self.parents
            .iter()
            .filter(|(_, p)| p == parent_name)
            .map(|(c, _)| c.as_str())
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sol_default_has_earth() {
        let sol = SystemConfig::sol();
        assert!(sol.body_index("Earth").is_some());
    }

    #[test]
    fn sol_default_parent_moon_is_earth() {
        let sol = SystemConfig::sol();
        assert_eq!(sol.parent_name("Moon"), Some("Earth"));
    }

    #[test]
    fn sol_default_children_of_jupiter() {
        let sol = SystemConfig::sol();
        let children = sol.children_names("Jupiter");
        assert!(children.contains(&"Io"));
        assert!(children.contains(&"Europa"));
        assert!(children.contains(&"Ganymede"));
        assert!(children.contains(&"Callisto"));
    }

    #[test]
    fn sol_body_count() {
        let sol = SystemConfig::sol();
        // Sun + 8 planets + Moon + 4 Galilean = 14
        assert_eq!(sol.bodies.len(), 14);
    }

    #[test]
    fn system_config_toml_roundtrip() {
        let sol = SystemConfig::sol();
        let toml_str = sol.to_toml_string().unwrap();
        let parsed = SystemConfig::from_toml_str(&toml_str).unwrap();
        assert_eq!(parsed.name, "Sol");
        assert_eq!(parsed.star, "Sun");
        assert_eq!(parsed.bodies.len(), sol.bodies.len());
        assert_eq!(parsed.parents.len(), sol.parents.len());
    }
}
