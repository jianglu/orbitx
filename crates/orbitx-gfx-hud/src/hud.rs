//! HUD system - ported from Orbiter `hud.cpp` (2,147 lines).
//!
//! Three modes: Orbit (flight path ladder), Surface (horizon/pitch/heading), Docking (approach gates).
//! Four colors: green/red/yellow/blue.
//!
//! Drawn with egui::Painter, classic CRT green MFD aesthetic.

use crate::flight_state::FlightState;

/// HUD mode (corresponds to Orbiter hud.cpp three modes).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HudMode {
    Orbit,
    Surface,
    Docking,
}

impl HudMode {
    pub fn all() -> &'static [HudMode] {
        &[HudMode::Orbit, HudMode::Surface, HudMode::Docking]
    }

    pub fn next(self) -> Self {
        match self {
            HudMode::Orbit => HudMode::Surface,
            HudMode::Surface => HudMode::Docking,
            HudMode::Docking => HudMode::Orbit,
        }
    }

    pub fn name(&self) -> &'static str {
        match self {
            HudMode::Orbit => "Orbit",
            HudMode::Surface => "Surface",
            HudMode::Docking => "Docking",
        }
    }
}

/// HUD color (corresponds to Orbiter hud.dds four color variants).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HudColor {
    Green,
    Red,
    Yellow,
    Blue,
}

impl HudColor {
    pub fn all() -> &'static [HudColor] {
        &[HudColor::Green, HudColor::Red, HudColor::Yellow, HudColor::Blue]
    }

    pub fn next(self) -> Self {
        match self {
            HudColor::Green => HudColor::Red,
            HudColor::Red => HudColor::Yellow,
            HudColor::Yellow => HudColor::Blue,
            HudColor::Blue => HudColor::Green,
        }
    }

    pub fn egui_color(&self) -> egui::Color32 {
        match self {
            HudColor::Green => egui::Color32::from_rgb(0, 255, 64),
            HudColor::Red => egui::Color32::from_rgb(255, 64, 64),
            HudColor::Yellow => egui::Color32::from_rgb(255, 255, 64),
            HudColor::Blue => egui::Color32::from_rgb(64, 128, 255),
        }
    }

    pub fn egui_color_dim(&self) -> egui::Color32 {
        match self {
            HudColor::Green => egui::Color32::from_rgb(0, 160, 40),
            HudColor::Red => egui::Color32::from_rgb(160, 40, 40),
            HudColor::Yellow => egui::Color32::from_rgb(160, 160, 40),
            HudColor::Blue => egui::Color32::from_rgb(40, 80, 160),
        }
    }
}

/// HUD element toggles.
#[derive(Clone, Debug)]
pub struct HudElements {
    pub center_marker: bool,
    pub flight_path_ladder: bool,
    pub prograde_marker: bool,
    pub retrograde_marker: bool,
    pub horizon_line: bool,
    pub pitch_ladder: bool,
    pub bank_indicator: bool,
    pub heading_tape: bool,
    pub airspeed: bool,
    pub altitude: bool,
    pub vertical_speed: bool,
    pub approach_gates: bool,
    pub velocity_marker: bool,
    pub range_readout: bool,
}

impl Default for HudElements {
    fn default() -> Self {
        Self {
            center_marker: true,
            flight_path_ladder: true,
            prograde_marker: true,
            retrograde_marker: true,
            horizon_line: true,
            pitch_ladder: true,
            bank_indicator: true,
            heading_tape: true,
            airspeed: true,
            altitude: true,
            vertical_speed: true,
            approach_gates: true,
            velocity_marker: true,
            range_readout: true,
        }
    }
}

/// HUD state.
pub struct HudState {
    pub mode: HudMode,
    pub color: HudColor,
    pub brightness: f32,
    pub elements: HudElements,
    pub alpha: f32,
}

impl HudState {
    pub fn new() -> Self {
        Self {
            mode: HudMode::Orbit,
            color: HudColor::Green,
            brightness: 1.0,
            elements: HudElements::default(),
            alpha: 0.85,
        }
    }

    pub fn next_mode(&mut self) {
        self.mode = self.mode.next();
    }

    pub fn next_color(&mut self) {
        self.color = self.color.next();
    }

    /// Draw HUD overlay using egui.
    ///
    /// Caller provides `&mut Ui` via `CentralPanel::show`.
    pub fn draw(&self, ui: &mut egui::Ui, state: &FlightState) {
        let color = self.color.egui_color();
        let color_dim = self.color.egui_color_dim();
        let rect = ui.available_rect_before_wrap();
        let painter = ui.painter_at(rect);
        let center = rect.center();

        if self.elements.center_marker {
            let size = 8.0;
            painter.line_segment(
                [center + egui::vec2(-size, 0.0), center + egui::vec2(size, 0.0)],
                egui::Stroke::new(1.5, color),
            );
            painter.line_segment(
                [center + egui::vec2(0.0, -size), center + egui::vec2(0.0, size)],
                egui::Stroke::new(1.5, color),
            );
        }

        match self.mode {
            HudMode::Orbit => self.draw_orbit(&painter, rect, state, color, color_dim),
            HudMode::Surface => self.draw_surface(&painter, rect, state, color, color_dim),
            HudMode::Docking => self.draw_docking(&painter, rect, state, color, color_dim),
        }
    }

    fn draw_orbit(
        &self, painter: &egui::Painter, rect: egui::Rect, state: &FlightState,
        color: egui::Color32, color_dim: egui::Color32,
    ) {
        let center = rect.center();

        if self.elements.flight_path_ladder {
            let ladder_width = 100.0_f32;
            let pitch_step = 20.0_f32;
            let pitch_deg = state.pitch.to_degrees() as f32;
            for i in -3..=3i32 {
                let y = center.y + (i as f32 * pitch_step)
                    - (pitch_deg.fract() * pitch_step / 5.0);
                let pitch_label = (pitch_deg.round() as i32 + i * 5).to_string();
                painter.line_segment(
                    [egui::pos2(center.x - ladder_width, y),
                     egui::pos2(center.x + ladder_width, y)],
                    egui::Stroke::new(1.0, if i == 0 { color } else { color_dim }),
                );
                painter.text(
                    egui::pos2(center.x + ladder_width + 4.0, y),
                    egui::Align2::LEFT_CENTER, &pitch_label,
                    egui::FontId::proportional(10.0), color_dim,
                );
            }
        }

        if self.elements.prograde_marker {
            let pos = center + egui::vec2(0.0, -30.0);
            painter.circle_stroke(pos, 6.0_f32, egui::Stroke::new(1.5, color));
            painter.text(pos, egui::Align2::CENTER_CENTER, "PRO",
                egui::FontId::proportional(8.0), color);
        }
        if self.elements.retrograde_marker {
            let pos = center + egui::vec2(0.0, 30.0);
            painter.circle_stroke(pos, 6.0_f32, egui::Stroke::new(1.5, color_dim));
            painter.text(pos, egui::Align2::CENTER_CENTER, "RET",
                egui::FontId::proportional(8.0), color_dim);
        }
    }

    fn draw_surface(
        &self, painter: &egui::Painter, rect: egui::Rect, state: &FlightState,
        color: egui::Color32, color_dim: egui::Color32,
    ) {
        let center = rect.center();

        if self.elements.horizon_line {
            let horizon_y = center.y + (state.pitch.to_degrees() as f32 * 4.0);
            painter.line_segment(
                [egui::pos2(rect.left() + 50.0, horizon_y),
                 egui::pos2(rect.right() - 50.0, horizon_y)],
                egui::Stroke::new(2.0, color),
            );
        }

        if self.elements.pitch_ladder {
            let pitch_step = 20.0_f32;
            for i in -4..=4i32 {
                if i == 0 { continue; }
                let y = center.y + (i as f32 * pitch_step);
                let width = if i % 2 == 0 { 60.0 } else { 30.0 };
                painter.line_segment(
                    [egui::pos2(center.x - width, y), egui::pos2(center.x + width, y)],
                    egui::Stroke::new(1.0, color_dim),
                );
            }
        }

        if self.elements.heading_tape {
            let tape_y = rect.bottom() - 60.0;
            let tape_width = 200.0;
            painter.line_segment(
                [egui::pos2(center.x - tape_width, tape_y),
                 egui::pos2(center.x + tape_width, tape_y)],
                egui::Stroke::new(1.0, color_dim),
            );
            let heading = (state.yaw.to_degrees() as f32).rem_euclid(360.0);
            painter.text(
                egui::pos2(center.x, tape_y - 10.0),
                egui::Align2::CENTER_BOTTOM, &format!("{:03.0}\u{00b0}", heading),
                egui::FontId::monospace(14.0), color,
            );
        }

        if self.elements.airspeed {
            painter.text(egui::pos2(rect.left() + 20.0, center.y - 10.0),
                egui::Align2::LEFT_CENTER, &state.fmt_speed(),
                egui::FontId::monospace(12.0), color);
        }
        if self.elements.altitude {
            painter.text(egui::pos2(rect.right() - 20.0, center.y - 10.0),
                egui::Align2::RIGHT_CENTER, &state.fmt_altitude(),
                egui::FontId::monospace(12.0), color);
        }
        if self.elements.vertical_speed {
            painter.text(egui::pos2(rect.right() - 20.0, center.y + 10.0),
                egui::Align2::RIGHT_CENTER, &format!("{:+.0} m/s", state.vertical_speed),
                egui::FontId::monospace(10.0), color_dim);
        }
    }

    fn draw_docking(
        &self, painter: &egui::Painter, rect: egui::Rect, state: &FlightState,
        color: egui::Color32, color_dim: egui::Color32,
    ) {
        let center = rect.center();

        if self.elements.approach_gates {
            let gate_size = 40.0_f32;
            painter.rect_stroke(
                egui::Rect::from_center_size(center, egui::vec2(gate_size * 2.0, gate_size * 2.0)),
                0.0,
                egui::Stroke::new(1.5, color_dim),
                egui::StrokeKind::Outside,
            );
            painter.line_segment(
                [center + egui::vec2(-gate_size, 0.0), center + egui::vec2(gate_size, 0.0)],
                egui::Stroke::new(1.0, color_dim),
            );
            painter.line_segment(
                [center + egui::vec2(0.0, -gate_size), center + egui::vec2(0.0, gate_size)],
                egui::Stroke::new(1.0, color_dim),
            );
        }

        if self.elements.range_readout {
            painter.text(egui::pos2(rect.left() + 20.0, rect.top() + 40.0),
                egui::Align2::LEFT_TOP, &format!("R: {:.0} m", state.focus_dist),
                egui::FontId::monospace(12.0), color);
            painter.text(egui::pos2(rect.left() + 20.0, rect.top() + 58.0),
                egui::Align2::LEFT_TOP, &format!("R': {:.1} m/s", state.vertical_speed),
                egui::FontId::monospace(12.0), color_dim);
        }
    }
}

impl Default for HudState {
    fn default() -> Self { Self::new() }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hud_mode_cycle() {
        assert_eq!(HudMode::Orbit.next(), HudMode::Surface);
        assert_eq!(HudMode::Surface.next(), HudMode::Docking);
        assert_eq!(HudMode::Docking.next(), HudMode::Orbit);
    }

    #[test]
    fn hud_color_cycle() {
        assert_eq!(HudColor::Green.next(), HudColor::Red);
        assert_eq!(HudColor::Red.next(), HudColor::Yellow);
        assert_eq!(HudColor::Yellow.next(), HudColor::Blue);
        assert_eq!(HudColor::Blue.next(), HudColor::Green);
    }

    #[test]
    fn hud_state_default() {
        let state = HudState::new();
        assert_eq!(state.mode, HudMode::Orbit);
        assert_eq!(state.color, HudColor::Green);
    }

    #[test]
    fn flight_state_orbit_class() {
        let mut fs = FlightState::default();
        fs.eccentricity = 0.0;
        assert_eq!(fs.orbit_class(), "Circular");
        fs.eccentricity = 0.5;
        assert_eq!(fs.orbit_class(), "Elliptical");
        fs.eccentricity = 1.5;
        assert_eq!(fs.orbit_class(), "Hyperbolic");
    }
}
