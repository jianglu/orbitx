//! Input mapping — key/mouse → `Action`，可选从 TOML 文件加载重映射。
//!
//! 内置默认 = Orbiter 风格。运行时按以下顺序解析用户配置：
//! 1. `$ORBITX_KEYBINDINGS`（若设置且指向可读文件）
//! 2. `$HOME/.config/orbitx/keybindings.toml`
//! 3. 内置默认（`assets/keybindings.toml` 编译时嵌入）
//!
//! 配置文件为极简 TOML，行内表 `key = "Action"` 或 `key = "CamModeSet(3)"`。
//! 未识别的键/动作会被静默丢弃并保留默认。

use std::collections::HashMap;
use std::path::{Path, PathBuf};

use winit::event::MouseButton;
use winit::keyboard::KeyCode;

/// User actions.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Action {
    // Camera
    CamOrbitLeft,
    CamOrbitRight,
    CamOrbitUp,
    CamOrbitDown,
    CamZoomIn,
    CamZoomOut,
    CamModeNext,
    CamModePrev,
    CamGroundObserver,
    /// 直接选择第 N 个外部模式（0..6）。
    CamModeSet(u8),
    /// 切换内部/外部驾驶舱视图。
    CamToggleInternal,
    /// 循环设置 TargetToObject/TargetFromObject 的参考天体。
    CamCycleDirref,
    // Flight control
    ThrottleUp,
    ThrottleDown,
    ThrottleFull,
    ThrottleCut,
    Prograde,
    Retrograde,
    RadialIn,
    RadialOut,
    RcsPitchUp,
    RcsPitchDown,
    RcsYawLeft,
    RcsYawRight,
    // MFD
    MfdLeftNext,
    MfdRightNext,
    // Time
    TimeWarpUp,
    TimeWarpDown,
    TimePause,
    // View
    FocusNextBody,
    FocusPrevBody,
    HudModeNext,
    HudColorNext,
    // General
    Quit,
}

impl Action {
    /// 解析 `"CamModeNext"` / `"CamModeSet(3)"` 等字符串为 Action。
    pub fn from_name(s: &str) -> Option<Action> {
        let s = s.trim();
        // 带参数：`CamModeSet(N)`
        if let Some(rest) = s.strip_prefix("CamModeSet(").and_then(|r| r.strip_suffix(')')) {
            let n: u8 = rest.trim().parse().ok()?;
            return Some(Action::CamModeSet(n));
        }
        Some(match s {
            "CamOrbitLeft" => Action::CamOrbitLeft,
            "CamOrbitRight" => Action::CamOrbitRight,
            "CamOrbitUp" => Action::CamOrbitUp,
            "CamOrbitDown" => Action::CamOrbitDown,
            "CamZoomIn" => Action::CamZoomIn,
            "CamZoomOut" => Action::CamZoomOut,
            "CamModeNext" => Action::CamModeNext,
            "CamModePrev" => Action::CamModePrev,
            "CamGroundObserver" => Action::CamGroundObserver,
            "CamToggleInternal" => Action::CamToggleInternal,
            "CamCycleDirref" => Action::CamCycleDirref,
            "ThrottleUp" => Action::ThrottleUp,
            "ThrottleDown" => Action::ThrottleDown,
            "ThrottleFull" => Action::ThrottleFull,
            "ThrottleCut" => Action::ThrottleCut,
            "Prograde" => Action::Prograde,
            "Retrograde" => Action::Retrograde,
            "RadialIn" => Action::RadialIn,
            "RadialOut" => Action::RadialOut,
            "RcsPitchUp" => Action::RcsPitchUp,
            "RcsPitchDown" => Action::RcsPitchDown,
            "RcsYawLeft" => Action::RcsYawLeft,
            "RcsYawRight" => Action::RcsYawRight,
            "MfdLeftNext" => Action::MfdLeftNext,
            "MfdRightNext" => Action::MfdRightNext,
            "TimeWarpUp" => Action::TimeWarpUp,
            "TimeWarpDown" => Action::TimeWarpDown,
            "TimePause" => Action::TimePause,
            "FocusNextBody" => Action::FocusNextBody,
            "FocusPrevBody" => Action::FocusPrevBody,
            "HudModeNext" => Action::HudModeNext,
            "HudColorNext" => Action::HudColorNext,
            "Quit" => Action::Quit,
            _ => return None,
        })
    }
}

/// 解析 `"KeyW"` / `"Digit1"` / `"Space"` / `"ArrowUp"` 等字符串为 winit KeyCode。
///
/// 支持所有 KeyCode 变体的名称匹配（区分大小写）。
pub fn parse_key(s: &str) -> Option<KeyCode> {
    use KeyCode::*;
    let s = s.trim();
    Some(match s {
        "KeyA" => KeyA, "KeyB" => KeyB, "KeyC" => KeyC, "KeyD" => KeyD,
        "KeyE" => KeyE, "KeyF" => KeyF, "KeyG" => KeyG, "KeyH" => KeyH,
        "KeyI" => KeyI, "KeyJ" => KeyJ, "KeyK" => KeyK, "KeyL" => KeyL,
        "KeyM" => KeyM, "KeyN" => KeyN, "KeyO" => KeyO, "KeyP" => KeyP,
        "KeyQ" => KeyQ, "KeyR" => KeyR, "KeyS" => KeyS, "KeyT" => KeyT,
        "KeyU" => KeyU, "KeyV" => KeyV, "KeyW" => KeyW, "KeyX" => KeyX,
        "KeyY" => KeyY, "KeyZ" => KeyZ,
        "Digit0" => Digit0, "Digit1" => Digit1, "Digit2" => Digit2, "Digit3" => Digit3,
        "Digit4" => Digit4, "Digit5" => Digit5, "Digit6" => Digit6, "Digit7" => Digit7,
        "Digit8" => Digit8, "Digit9" => Digit9,
        "Space" => Space, "Tab" => Tab, "Enter" => Enter, "Escape" => Escape,
        "Backspace" => Backspace, "Backquote" => Backquote,
        "Period" => Period, "Comma" => Comma,
        "BracketLeft" => BracketLeft, "BracketRight" => BracketRight,
        "Semicolon" => Semicolon, "Quote" => Quote, "Backslash" => Backslash,
        "Slash" => Slash, "Minus" => Minus, "Equal" => Equal,
        "ArrowUp" => ArrowUp, "ArrowDown" => ArrowDown,
        "ArrowLeft" => ArrowLeft, "ArrowRight" => ArrowRight,
        "PageUp" => PageUp, "PageDown" => PageDown,
        "Home" => Home, "End" => End, "Insert" => Insert, "Delete" => Delete,
        "F1" => F1, "F2" => F2, "F3" => F3, "F4" => F4, "F5" => F5, "F6" => F6,
        "F7" => F7, "F8" => F8, "F9" => F9, "F10" => F10, "F11" => F11, "F12" => F12,
        "ShiftLeft" => ShiftLeft, "ShiftRight" => ShiftRight,
        "ControlLeft" => ControlLeft, "ControlRight" => ControlRight,
        "AltLeft" => AltLeft, "AltRight" => AltRight,
        _ => return None,
    })
}

/// 键位映射表：KeyCode → Action。
///
/// `KeyMap::default()` 返回内置的 Orbiter 风格默认；
/// `KeyMap::load_or_default(path)` 尝试从 TOML 加载并 fallback 到默认。
#[derive(Clone, Debug)]
pub struct KeyMap {
    map: HashMap<KeyCode, Action>,
}

impl KeyMap {
    /// 查找按键对应的 Action。
    pub fn get(&self, key: KeyCode) -> Option<Action> {
        self.map.get(&key).copied()
    }

    /// 内置默认（编译时嵌入的 `assets/keybindings.toml`）。
    ///
    /// 若嵌入文件解析失败（不应发生），回退到硬编码。
    pub fn baked_default() -> Self {
        const EMBEDDED: &str = include_str!("../../../assets/keybindings.toml");
        Self::from_toml(EMBEDDED).unwrap_or_else(Self::hardcoded_default)
    }

    /// 硬编码 fallback（如果嵌入文件损坏也能工作）。
    pub fn hardcoded_default() -> Self {
        use Action::*;
        use KeyCode::*;
        let mut map = HashMap::new();
        for (k, a) in [
            (KeyW, CamOrbitUp), (KeyS, CamOrbitDown),
            (KeyA, CamOrbitLeft), (KeyD, CamOrbitRight),
            (KeyQ, CamZoomIn), (KeyE, CamZoomOut),
            (Tab, CamModeNext), (KeyG, CamGroundObserver),
            (KeyV, CamToggleInternal), (KeyR, CamCycleDirref),
            (Digit1, CamModeSet(0)), (Digit2, CamModeSet(1)),
            (Digit3, CamModeSet(2)), (Digit4, CamModeSet(3)),
            (Digit5, CamModeSet(4)), (Digit6, CamModeSet(5)),
            (ArrowUp, ThrottleUp), (ArrowDown, ThrottleDown),
            (Backquote, ThrottleCut), (Digit0, ThrottleFull),
            (Space, TimePause), (Period, TimeWarpUp), (Comma, TimeWarpDown),
            (BracketRight, FocusNextBody), (BracketLeft, FocusPrevBody),
            (KeyH, HudModeNext), (KeyC, HudColorNext),
            (KeyO, MfdLeftNext), (KeyM, MfdRightNext),
            (Escape, Quit),
        ] {
            map.insert(k, a);
        }
        Self { map }
    }

    /// 从 TOML 字符串解析。语法极简：`KeyCode = "ActionName"` 顶层键值对。
    ///
    /// 不识别的行会被跳过（不中断加载）。
    /// 支持整行注释（`# ...`）与行尾注释（值后的 ` # ...` 段被忽略）。
    pub fn from_toml(s: &str) -> Option<Self> {
        let mut map = HashMap::new();
        for line in s.lines() {
            let line = line.trim();
            // 跳过空行 / 整行注释
            if line.is_empty() || line.starts_with('#') { continue; }
            let Some(eq) = line.find('=') else { continue; };
            let key_raw = line[..eq].trim();
            // 值段：先切掉行尾注释（`#` 之后到行尾），再 trim 空白 + 引号。
            let val_seg = &line[eq + 1..];
            let val_no_comment = match val_seg.find('#') {
                Some(idx) => &val_seg[..idx],
                None => val_seg,
            };
            let val_raw = val_no_comment.trim().trim_matches('"');
            let Some(key) = parse_key(key_raw) else { continue; };
            let Some(action) = Action::from_name(val_raw) else { continue; };
            map.insert(key, action);
        }
        if map.is_empty() { None } else { Some(Self { map }) }
    }

    /// 从路径尝试加载 TOML；文件不存在或解析失败时使用嵌入默认。
    pub fn load_or_default(path: &Path) -> Self {
        std::fs::read_to_string(path)
            .ok()
            .and_then(|s| Self::from_toml(&s))
            .unwrap_or_else(Self::baked_default)
    }

    /// 按 [`ORBITX_KEYBINDINGS` env → `$HOME/.config/orbitx/keybindings.toml` → 内置默认] 顺序解析。
    pub fn resolve() -> Self {
        if let Ok(p) = std::env::var("ORBITX_KEYBINDINGS") {
            let path = PathBuf::from(p);
            if path.is_file() {
                return Self::load_or_default(&path);
            }
        }
        if let Some(home) = std::env::var_os("HOME") {
            let path = PathBuf::from(home).join(".config/orbitx/keybindings.toml");
            if path.is_file() {
                return Self::load_or_default(&path);
            }
        }
        Self::baked_default()
    }
}

impl Default for KeyMap {
    fn default() -> Self { Self::baked_default() }
}

/// 向后兼容：使用内置默认 KeyMap 的自由函数。
///
/// 新代码应通过 `KeyMap::resolve()` 或 `App::key_map` 进行查找。
pub fn key_to_action(key: KeyCode) -> Option<Action> {
    // 静态默认：只在首次调用时构建
    use std::sync::OnceLock;
    static DEFAULT: OnceLock<KeyMap> = OnceLock::new();
    DEFAULT.get_or_init(KeyMap::hardcoded_default).get(key)
}

/// Map winit mouse button to Action.
pub fn mouse_to_action(_button: MouseButton) -> Option<Action> {
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn action_from_name_roundtrip() {
        assert_eq!(Action::from_name("CamModeNext"), Some(Action::CamModeNext));
        assert_eq!(Action::from_name("ThrottleFull"), Some(Action::ThrottleFull));
        assert_eq!(Action::from_name("Quit"), Some(Action::Quit));
        assert_eq!(Action::from_name("Unknown"), None);
    }

    #[test]
    fn action_from_name_with_param() {
        assert_eq!(Action::from_name("CamModeSet(3)"), Some(Action::CamModeSet(3)));
        assert_eq!(Action::from_name("CamModeSet(0)"), Some(Action::CamModeSet(0)));
        assert_eq!(Action::from_name("CamModeSet( 5 )"), Some(Action::CamModeSet(5)));
        assert_eq!(Action::from_name("CamModeSet(abc)"), None);
    }

    #[test]
    fn parse_key_common() {
        assert_eq!(parse_key("KeyW"), Some(KeyCode::KeyW));
        assert_eq!(parse_key("Digit1"), Some(KeyCode::Digit1));
        assert_eq!(parse_key("Space"), Some(KeyCode::Space));
        assert_eq!(parse_key("ArrowUp"), Some(KeyCode::ArrowUp));
        assert_eq!(parse_key("Backquote"), Some(KeyCode::Backquote));
        assert_eq!(parse_key("Bogus"), None);
    }

    #[test]
    fn keymap_baked_default_covers_common() {
        let km = KeyMap::baked_default();
        assert_eq!(km.get(KeyCode::KeyW), Some(Action::CamOrbitUp));
        assert_eq!(km.get(KeyCode::Space), Some(Action::TimePause));
        assert_eq!(km.get(KeyCode::Escape), Some(Action::Quit));
        assert_eq!(km.get(KeyCode::Digit3), Some(Action::CamModeSet(2)));
    }

    #[test]
    fn keymap_from_toml_ignores_unknown() {
        let src = r#"
            # 注释行
            KeyW = "CamOrbitUp"
            Bogus = "CamOrbitDown"
            Space = "NotAnAction"
            Digit0 = "ThrottleFull"
        "#;
        let km = KeyMap::from_toml(src).expect("some parsed");
        assert_eq!(km.get(KeyCode::KeyW), Some(Action::CamOrbitUp));
        assert_eq!(km.get(KeyCode::Digit0), Some(Action::ThrottleFull));
        assert_eq!(km.get(KeyCode::Space), None); // 未加载：动作名无效
    }

    #[test]
    fn hardcoded_default_matches_baked_key_count() {
        // 两者不需完全相等，但要求 hardcoded 至少覆盖所有 baked 键
        // （baked 是权威来源，hardcoded 是 fallback）。
        let baked = KeyMap::baked_default();
        let hard = KeyMap::hardcoded_default();
        for (k, _) in &baked.map {
            assert!(hard.map.contains_key(k),
                "hardcoded fallback missing key: {:?}", k);
        }
    }
}
