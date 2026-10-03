//! 内置火箭别名（与 orbitx-cli 历史别名一致）。

use super::RocketConfig;

/// `(alias, display_name, toml)`.
const BUILTINS: &[(&str, &str, &str)] = &[
    (
        "falcon9",
        "Falcon 9 (SpaceX)",
        include_str!("../../../presets/falcon9.toml"),
    ),
    (
        "saturnv",
        "Saturn V (NASA)",
        include_str!("../../../presets/saturn_v.toml"),
    ),
    (
        "lm5",
        "长征五号 Long March 5",
        include_str!("../../../presets/long_march_5.toml"),
    ),
    (
        "lm2f",
        "长征二号F Long March 2F",
        include_str!("../../../presets/long_march_2f.toml"),
    ),
    (
        "lm7",
        "长征七号 Long March 7",
        include_str!("../../../presets/long_march_7.toml"),
    ),
    (
        "lm9",
        "长征九号 Long March 9",
        include_str!("../../../presets/long_march_9.toml"),
    ),
];

/// 别名 → 嵌入的 rocket.toml 文本。
pub fn builtin_rocket_toml(alias: &str) -> Option<&'static str> {
    BUILTINS
        .iter()
        .find(|(a, _, _)| *a == alias)
        .map(|(_, _, toml)| *toml)
}

/// `(alias, display_name)` 列表，供帮助 / 错误信息。
pub fn builtin_aliases() -> &'static [(&'static str, &'static str)] {
    static ALIASES: &[(&str, &str)] = &[
        ("falcon9", "Falcon 9 (SpaceX)"),
        ("saturnv", "Saturn V (NASA)"),
        ("lm5", "长征五号 Long March 5"),
        ("lm2f", "长征二号F Long March 2F"),
        ("lm7", "长征七号 Long March 7"),
        ("lm9", "长征九号 Long March 9"),
    ];
    ALIASES
}

/// Expand `~/…` (and a wrapping quote pair) to an absolute path.
///
/// Shells do **not** expand `~` inside `"~/Library/…"`. Passing that string
/// through clap used to miss the file and fall through as an unknown alias,
/// while orbitx-cli still drew an empty TUI.
pub fn expand_rocket_spec(spec: &str) -> std::path::PathBuf {
    let spec = spec.trim().trim_matches(|c| c == '"' || c == '\'');
    if spec == "~" {
        if let Some(home) = home_dir() {
            return home;
        }
    }
    if let Some(rest) = spec.strip_prefix("~/") {
        if let Some(home) = home_dir() {
            return home.join(rest);
        }
    }
    std::path::PathBuf::from(spec)
}

fn home_dir() -> Option<std::path::PathBuf> {
    std::env::var_os("HOME")
        .or_else(|| std::env::var_os("USERPROFILE"))
        .map(std::path::PathBuf::from)
}

/// 从别名或文件路径加载 `RocketConfig`。
///
/// - 路径存在 → 读文件（`~/` 会展开）；
/// - 否则按内置别名解析；
/// - 皆否 → 错误。
pub fn load_rocket_source(spec: &str) -> Result<RocketConfig, String> {
    let path = expand_rocket_spec(spec);
    if path.exists() {
        return RocketConfig::from_file(&path)
            .map_err(|e| format!("读火箭配置 `{}` 失败：{e}", path.display()));
    }
    let alias = spec.trim().trim_matches(|c| c == '"' || c == '\'');
    if let Some(toml) = builtin_rocket_toml(alias) {
        return RocketConfig::from_toml_str(toml)
            .map_err(|e| format!("解析内置火箭 `{alias}` 失败：{e}"));
    }
    let mut msg = format!(
        "未知火箭：`{spec}`（不是文件路径，也不是内置别名）\n已解析路径：{}\n可用别名：\n",
        path.display()
    );
    for (alias, name) in builtin_aliases() {
        msg.push_str(&format!("  {alias:<10} {name}\n"));
    }
    Err(msg)
}

#[cfg(test)]
mod tests;
