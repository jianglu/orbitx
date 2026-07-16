//! MFD（多功能显示器）— 移植自 Orbiter `Mfd.cpp`（1,290 行）+ 10 种内置 MFD。
//!
//! 使用 egui 窗口绘制，经典 CRT 绿色美学。

use crate::flight_state::FlightState;

/// MFD 类型（对应 Orbiter 10 种内置 MFD）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MfdType {
    Orbit,
    Surface,
    Map,
    Hsi,
    Landing,
    Docking,
    Align,
    Sync,
    Transfer,
    Comms,
}

impl MfdType {
    pub fn all() -> &'static [MfdType] {
        &[
            MfdType::Orbit, MfdType::Surface, MfdType::Map, MfdType::Hsi,
            MfdType::Landing, MfdType::Docking, MfdType::Align, MfdType::Sync,
            MfdType::Transfer, MfdType::Comms,
        ]
    }

    pub fn key(&self) -> char {
        match self {
            MfdType::Orbit => 'O', MfdType::Surface => 'S', MfdType::Map => 'M',
            MfdType::Hsi => 'H', MfdType::Landing => 'L', MfdType::Docking => 'D',
            MfdType::Align => 'A', MfdType::Sync => 'Y', MfdType::Transfer => 'X',
            MfdType::Comms => 'C',
        }
    }

    pub fn name(&self) -> &'static str {
        match self {
            MfdType::Orbit => "Orbit", MfdType::Surface => "Surface", MfdType::Map => "Map",
            MfdType::Hsi => "HSI", MfdType::Landing => "Landing", MfdType::Docking => "Docking",
            MfdType::Align => "Align", MfdType::Sync => "Sync", MfdType::Transfer => "Transfer",
            MfdType::Comms => "Comms",
        }
    }

    pub fn next(self) -> Self {
        let all = Self::all();
        let idx = all.iter().position(|&t| t == self).unwrap();
        all[(idx + 1) % all.len()]
    }
}

/// MFD 面板位置（左/右）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MfdSize {
    Left,
    Right,
}

impl MfdSize {
    pub fn label(&self) -> &'static str {
        match self {
            MfdSize::Left => "MFD Left",
            MfdSize::Right => "MFD Right",
        }
    }
}

/// MFD 面板。
pub struct MfdPanel {
    pub mfd_type: MfdType,
    pub size: MfdSize,
    pub buttons: [String; 6],
    pub active: bool,
    fg_color: egui::Color32,
    dim_color: egui::Color32,
    bg_color: egui::Color32,
}

impl MfdPanel {
    pub fn new(mfd_type: MfdType, size: MfdSize) -> Self {
        Self {
            mfd_type,
            size,
            buttons: Self::default_buttons(mfd_type),
            active: true,
            fg_color: egui::Color32::from_rgb(0, 255, 64),
            dim_color: egui::Color32::from_rgb(0, 160, 40),
            bg_color: egui::Color32::from_rgba_unmultiplied(0, 10, 0, 160),
        }
    }

    pub fn next_type(&mut self) {
        self.mfd_type = self.mfd_type.next();
        self.buttons = Self::default_buttons(self.mfd_type);
    }

    fn default_buttons(mfd_type: MfdType) -> [String; 6] {
        match mfd_type {
            MfdType::Orbit => ["REF".into(), "FRM".into(), "PRO".into(), "APT".into(), "TGT".into(), "MOD".into()],
            MfdType::Map => ["ZM+".into(), "ZM-".into(), "CTR".into(), "TRK".into(), "LBL".into(), "MOD".into()],
            MfdType::Docking => ["TGT".into(), "NRD".into(), "APP".into(), "VCL".into(), "HOR".into(), "MOD".into()],
            MfdType::Landing => ["NAV".into(), "OS".into(), "HSI".into(), "ADM".into(), "ARB".into(), "MOD".into()],
            MfdType::Surface => ["SPD".into(), "ALT".into(), "V/S".into(), "HDG".into(), "TGT".into(), "MOD".into()],
            _ => ["  ".into(), "  ".into(), "  ".into(), "  ".into(), "  ".into(), "  ".into()],
        }
    }

    /// 使用 egui 绘制 MFD 面板。
    ///
    /// 调用方通过 `Window::show` 提供 `&mut Ui`。
    pub fn draw(&self, ui: &mut egui::Ui, state: &FlightState) {
        if !self.active {
            return;
        }

        let rect = ui.available_rect_before_wrap();
        let painter = ui.painter_at(rect);

        // CRT 绿色背景
        painter.rect_filled(rect, 0.0, self.bg_color);

        // MFD 标题
        painter.text(
            egui::pos2(rect.center().x, rect.top() + 12.0),
            egui::Align2::CENTER_TOP,
            &format!("{} [{}]", self.mfd_type.name(), self.mfd_type.key()),
            egui::FontId::monospace(14.0),
            self.fg_color,
        );

        // MFD 内容
        let content_rect = egui::Rect::from_min_max(
            egui::pos2(rect.left() + 8.0, rect.top() + 28.0),
            egui::pos2(rect.right() - 8.0, rect.bottom() - 30.0),
        );
        self.draw_content(&painter, content_rect, state);

        // 功能键按钮（底部）
        let btn_y = rect.bottom() - 20.0;
        let btn_width = content_rect.width() / 6.0;
        for (i, label) in self.buttons.iter().enumerate() {
            let x = content_rect.left() + (i as f32 + 0.5) * btn_width;
            painter.text(
                egui::pos2(x, btn_y),
                egui::Align2::CENTER_CENTER,
                label,
                egui::FontId::monospace(10.0),
                self.dim_color,
            );
        }
    }

    fn draw_content(&self, painter: &egui::Painter, rect: egui::Rect, state: &FlightState) {
        let left = rect.left();
        let top = rect.top();
        let line_h = 14.0_f32;

        match self.mfd_type {
            MfdType::Orbit => {
                let lines = [
                    format!("Ref: {}", state.focus_name),
                    format!("a: {:.0} km", state.semi_major_axis / 1e3),
                    format!("e: {:.4}", state.eccentricity),
                    format!("i: {:.2}°", state.inclination.to_degrees()),
                    format!("Pe: {:.0} km", state.periapsis_alt / 1e3),
                    format!("Ap: {:.0} km", state.apoapsis_alt / 1e3),
                    format!("T: {:.0} s", state.period),
                    format!("ε: {:.0} J/kg", state.specific_energy),
                    format!("Class: {}", state.orbit_class()),
                ];
                for (i, line) in lines.iter().enumerate() {
                    painter.text(
                        egui::pos2(left, top + (i as f32) * line_h),
                        egui::Align2::LEFT_TOP,
                        line,
                        egui::FontId::monospace(11.0),
                        self.fg_color,
                    );
                }

                // 简化轨道图：画圆形轮廓 + 航天器标记
                let center = rect.center() + egui::vec2(40.0, -10.0);
                let rx = 30.0_f32;
                painter.circle_stroke(center, rx, egui::Stroke::new(1.0, self.dim_color));
                let angle = (state.sim_time * 0.001) as f32;
                let sc_pos = center + egui::vec2(rx * angle.cos(), rx * angle.sin());
                painter.circle_filled(sc_pos, 3.0, self.fg_color);
            }
            MfdType::Surface => {
                let lines = [
                    format!("Alt: {}", state.fmt_altitude()),
                    format!("Spd: {}", state.fmt_speed()),
                    format!("V/S: {:+.0} m/s", state.vertical_speed),
                    format!("Hdg: {:03.0}°", (state.yaw.to_degrees() as f32).rem_euclid(360.0)),
                    format!("Pit: {:.1}°", state.pitch.to_degrees()),
                    format!("Ban: {:.1}°", state.bank.to_degrees()),
                    format!("ρ: {:.3e} kg/m³", state.air_density),
                    format!("q: {:.0} Pa", state.dynamic_pressure),
                    format!("M: {:.2}", state.mach),
                ];
                for (i, line) in lines.iter().enumerate() {
                    painter.text(
                        egui::pos2(left, top + (i as f32) * line_h),
                        egui::Align2::LEFT_TOP,
                        line,
                        egui::FontId::monospace(11.0),
                        self.fg_color,
                    );
                }
            }
            MfdType::Docking => {
                let lines = [
                    format!("Range: {:.0} m", state.focus_dist),
                    format!("Vrel: {:.1} m/s", state.vertical_speed),
                    format!("Status: {}", if state.focus_dist < 10.0 { "DOCKED" } else { "APPROACH" }),
                ];
                for (i, line) in lines.iter().enumerate() {
                    painter.text(
                        egui::pos2(left, top + (i as f32) * line_h),
                        egui::Align2::LEFT_TOP,
                        line,
                        egui::FontId::monospace(11.0),
                        self.fg_color,
                    );
                }
            }
            _ => {
                painter.text(
                    rect.center(),
                    egui::Align2::CENTER_CENTER,
                    &format!("{} MFD\n(not yet implemented)", self.mfd_type.name()),
                    egui::FontId::monospace(12.0),
                    self.dim_color,
                );
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mfd_type_cycle() {
        let t = MfdType::Orbit;
        let t2 = t.next();
        assert_eq!(t2, MfdType::Surface);
    }

    #[test]
    fn mfd_type_keys() {
        assert_eq!(MfdType::Orbit.key(), 'O');
        assert_eq!(MfdType::Docking.key(), 'D');
        assert_eq!(MfdType::Map.key(), 'M');
    }

    #[test]
    fn mfd_panel_default_buttons() {
        let panel = MfdPanel::new(MfdType::Orbit, MfdSize::Left);
        assert_eq!(panel.buttons[0], "REF");
        assert_eq!(panel.buttons[5], "MOD");
    }

    #[test]
    fn mfd_panel_next_type() {
        let mut panel = MfdPanel::new(MfdType::Orbit, MfdSize::Left);
        panel.next_type();
        assert_eq!(panel.mfd_type, MfdType::Surface);
        assert_eq!(panel.buttons[0], "SPD");
    }
}
