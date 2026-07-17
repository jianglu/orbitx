//! Input mapping - Orbiter-compatible key bindings.

use winit::keyboard::KeyCode;
use winit::event::MouseButton;

/// User actions.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
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

/// Map winit keyboard event to Action.
pub fn key_to_action(key: KeyCode) -> Option<Action> {
    use KeyCode::*;
    match key {
        KeyW => Some(Action::CamOrbitUp),
        KeyS => Some(Action::CamOrbitDown),
        KeyA => Some(Action::CamOrbitLeft),
        KeyD => Some(Action::CamOrbitRight),
        KeyQ => Some(Action::CamZoomIn),
        KeyE => Some(Action::CamZoomOut),
        Tab => Some(Action::CamModeNext),
        KeyG => Some(Action::CamGroundObserver),
        KeyV => Some(Action::CamToggleInternal),
        KeyR => Some(Action::CamCycleDirref),
        Digit1 => Some(Action::CamModeSet(0)),
        Digit2 => Some(Action::CamModeSet(1)),
        Digit3 => Some(Action::CamModeSet(2)),
        Digit4 => Some(Action::CamModeSet(3)),
        Digit5 => Some(Action::CamModeSet(4)),
        Digit6 => Some(Action::CamModeSet(5)),
        ArrowUp => Some(Action::ThrottleUp),
        ArrowDown => Some(Action::ThrottleDown),
        Backquote => Some(Action::ThrottleCut),
        Digit0 => Some(Action::ThrottleFull),
        Space => Some(Action::TimePause),
        Period => Some(Action::TimeWarpUp),
        Comma => Some(Action::TimeWarpDown),
        BracketRight => Some(Action::FocusNextBody),
        BracketLeft => Some(Action::FocusPrevBody),
        KeyH => Some(Action::HudModeNext),
        KeyC => Some(Action::HudColorNext),
        KeyO => Some(Action::MfdLeftNext),
        KeyM => Some(Action::MfdRightNext),
        Escape => Some(Action::Quit),
        _ => None,
    }
}

/// Map winit mouse button to Action.
pub fn mouse_to_action(_button: MouseButton) -> Option<Action> {
    None
}
