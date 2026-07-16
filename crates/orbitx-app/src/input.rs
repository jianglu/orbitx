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
        ArrowUp => Some(Action::ThrottleUp),
        ArrowDown => Some(Action::ThrottleDown),
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
