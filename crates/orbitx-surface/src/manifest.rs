//! Parse SurfaceDataset `manifest.toml` (format_version = 2).

use std::collections::HashMap;

pub const SUPPORTED_FORMAT_VERSION: i64 = 2;

#[derive(Debug, Clone)]
pub struct LayerMeta {
    pub path: String,
    pub format: String,
    pub max_level: u32,
}

#[derive(Debug, Clone)]
pub struct Manifest {
    pub format_version: i64,
    pub dataset_id: String,
    pub r_elev_m: f64,
    pub ocean_surface_m: f64,
    pub layers: HashMap<String, LayerMeta>,
}

impl Manifest {
    pub fn parse(text: &str) -> Result<Self, String> {
        require_format_version(text)?;
        let format_version = parse_format_version(text)?;
        let dataset_id = parse_string_key(text, "dataset_id").unwrap_or_default();
        let r_elev_m = parse_f64_key(text, "R_elev_m").unwrap_or(0.0);
        let ocean_surface_m = parse_f64_key(text, "ocean_surface_m").unwrap_or(0.0);

        let mut layers = HashMap::new();
        let mut current: Option<String> = None;
        let mut path = String::new();
        let mut format = String::new();
        let mut max_level: u32 = 0;
        let mut have_path = false;

        let flush = |layers: &mut HashMap<String, LayerMeta>,
                     current: &mut Option<String>,
                     path: &mut String,
                     format: &mut String,
                     max_level: &mut u32,
                     have_path: &mut bool| {
            if let Some(name) = current.take() {
                if *have_path {
                    layers.insert(
                        name,
                        LayerMeta {
                            path: std::mem::take(path),
                            format: std::mem::take(format),
                            max_level: *max_level,
                        },
                    );
                }
            }
            *have_path = false;
            *max_level = 0;
        };

        for raw in text.lines() {
            let line = raw.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            if let Some(rest) = line.strip_prefix("[layers.") {
                flush(
                    &mut layers,
                    &mut current,
                    &mut path,
                    &mut format,
                    &mut max_level,
                    &mut have_path,
                );
                let name = rest.trim_end_matches(']').trim();
                current = Some(name.to_string());
                continue;
            }
            if line.starts_with('[') {
                flush(
                    &mut layers,
                    &mut current,
                    &mut path,
                    &mut format,
                    &mut max_level,
                    &mut have_path,
                );
                current = None;
                continue;
            }
            if current.is_none() {
                continue;
            }
            if let Some((k, v)) = line.split_once('=') {
                let k = k.trim();
                let v = v.trim().trim_matches('"');
                match k {
                    "path" => {
                        path = v.to_string();
                        have_path = true;
                    }
                    "format" => format = v.to_string(),
                    "max_level" => {
                        if let Ok(n) = v.parse::<u32>() {
                            max_level = n;
                        }
                    }
                    _ => {}
                }
            }
        }
        flush(
            &mut layers,
            &mut current,
            &mut path,
            &mut format,
            &mut max_level,
            &mut have_path,
        );

        Ok(Self {
            format_version,
            dataset_id,
            r_elev_m,
            ocean_surface_m,
            layers,
        })
    }

    /// Effective elev/terrain depth: only when format contains "i16".
    pub fn terrain_max_level(&self) -> u32 {
        self.layers
            .get("terrain")
            .map(|l| {
                if l.format.contains("i16") {
                    l.max_level
                } else {
                    0
                }
            })
            .unwrap_or(0)
    }

    pub fn layer_max_level(&self, name: &str) -> u32 {
        if name == "terrain" {
            return self.terrain_max_level();
        }
        self.layers.get(name).map(|l| l.max_level).unwrap_or(0)
    }

    pub fn layer_path(&self, name: &str) -> String {
        self.layers
            .get(name)
            .map(|l| l.path.clone())
            .unwrap_or_else(|| name.to_string())
    }
}

pub fn parse_format_version(manifest: &str) -> Result<i64, String> {
    for line in manifest.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        if let Some(rest) = line.strip_prefix("format_version") {
            let v = rest.trim().trim_start_matches('=').trim();
            return v
                .parse::<i64>()
                .map_err(|e| format!("format_version: {e}"));
        }
    }
    Err("manifest missing format_version; rebake the surface package".into())
}

pub fn require_format_version(manifest: &str) -> Result<(), String> {
    let version = parse_format_version(manifest)?;
    if version != SUPPORTED_FORMAT_VERSION {
        return Err(format!(
            "unsupported format_version {version} (expected {SUPPORTED_FORMAT_VERSION}); rebake the surface package"
        ));
    }
    Ok(())
}

fn parse_f64_key(manifest: &str, key: &str) -> Option<f64> {
    for line in manifest.lines() {
        let line = line.trim();
        if let Some(rest) = line.strip_prefix(key) {
            let v = rest.trim().trim_start_matches('=').trim();
            if let Ok(x) = v.parse::<f64>() {
                return Some(x);
            }
        }
    }
    None
}

fn parse_string_key(manifest: &str, key: &str) -> Option<String> {
    for line in manifest.lines() {
        let line = line.trim();
        if let Some(rest) = line.strip_prefix(key) {
            let v = rest
                .trim()
                .trim_start_matches('=')
                .trim()
                .trim_matches('"');
            if !v.is_empty() {
                return Some(v.to_string());
            }
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn accepts_format_version_2() {
        let manifest = "# comment\nformat_version = 2\nocean_surface_m = 0\n";
        assert_eq!(parse_format_version(manifest).unwrap(), 2);
        assert!(require_format_version(manifest).is_ok());
    }

    #[test]
    fn rejects_other_major() {
        let err = require_format_version("format_version = 1\n").unwrap_err();
        assert!(err.contains("unsupported format_version 1"));
        assert!(err.contains("rebake"));
    }

    #[test]
    fn rejects_missing_version() {
        let err = require_format_version("ocean_surface_m = 0\n").unwrap_err();
        assert!(err.contains("missing format_version"));
    }

    #[test]
    fn parses_manifest_meta() {
        let text = r#"
format_version = 2
dataset_id = "earth_g0b_low_v2"
R_elev_m = 6371010.0
ocean_surface_m = 0.0
[layers.terrain]
path = "terrain"
format = "int16_le_i16"
max_level = 7
[layers.water_class]
path = "water_class"
format = "png_u8"
max_level = 6
[layers.water_surface]
path = "water_surface"
format = "int16_le_i16"
max_level = 6
"#;
        let m = Manifest::parse(text).unwrap();
        assert_eq!(m.dataset_id, "earth_g0b_low_v2");
        assert_eq!(m.r_elev_m, 6371010.0);
        assert_eq!(m.ocean_surface_m, 0.0);
        assert_eq!(m.terrain_max_level(), 7);
        assert_eq!(m.layer_max_level("water_class"), 6);
        assert_eq!(m.layer_max_level("water_surface"), 6);
    }

    #[test]
    fn max_level_i16_gate() {
        let text = r#"
format_version = 2
dataset_id = "old"
R_elev_m = 6371010.0
ocean_surface_m = 0.0
[layers.terrain]
path = "terrain"
format = "float32_le"
max_level = 7
"#;
        let m = Manifest::parse(text).unwrap();
        assert_eq!(m.terrain_max_level(), 0);
    }
}
