//! HUD system - ported from Orbiter `hud.cpp` (2,147 lines).
//!
//! 三种模式：Orbit（飞行路径梯 + 速度/高度带）、Surface（滚动地平线 + 航向带）、
//! Docking（目标框 + 接近率）。
//!
//! 用 `egui::Painter` 绘制，经典 CRT 绿色美学。

use crate::flight_state::FlightState;

/// HUD 模式（对应 Orbiter `hud.cpp` 三种模式）。
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

/// HUD 颜色（对应 Orbiter `hud.dds` 四色变体）。
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

/// HUD 元素开关。
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

/// HUD 状态。
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

    /// 绘制 HUD 叠加层。
    pub fn draw(&self, ui: &mut egui::Ui, state: &FlightState) {
        let color = self.color.egui_color();
        let color_dim = self.color.egui_color_dim();
        let rect = ui.available_rect_before_wrap();
        let painter = ui.painter_at(rect);
        let center = rect.center();

        // 中心十字（所有模式共用）
        if self.elements.center_marker {
            let s = 8.0;
            painter.line_segment(
                [center + egui::vec2(-s, 0.0), center + egui::vec2(s, 0.0)],
                egui::Stroke::new(1.5, color),
            );
            painter.line_segment(
                [center + egui::vec2(0.0, -s), center + egui::vec2(0.0, s)],
                egui::Stroke::new(1.5, color),
            );
        }

        // 顶栏：模式 + 时间加速
        painter.text(
            egui::pos2(rect.left() + 12.0, rect.top() + 8.0),
            egui::Align2::LEFT_TOP,
            &format!("MODE {}", self.mode.name().to_uppercase()),
            egui::FontId::monospace(12.0),
            color,
        );
        painter.text(
            egui::pos2(rect.right() - 12.0, rect.top() + 8.0),
            egui::Align2::RIGHT_TOP,
            &format!("Tx{:.0}", state.time_warp),
            egui::FontId::monospace(12.0),
            color_dim,
        );

        match self.mode {
            HudMode::Orbit => self.draw_orbit(&painter, rect, state, color, color_dim),
            HudMode::Surface => self.draw_surface(&painter, rect, state, color, color_dim),
            HudMode::Docking => self.draw_docking(&painter, rect, state, color, color_dim),
        }
    }

    // ------------------------------------------------------------------
    // Orbit 模式：飞行路径梯 + 前进/后退向量标 + 速度/高度侧带
    // ------------------------------------------------------------------
    fn draw_orbit(
        &self, painter: &egui::Painter, rect: egui::Rect, state: &FlightState,
        color: egui::Color32, color_dim: egui::Color32,
    ) {
        let center = rect.center();
        let px_per_deg = 5.0_f32;
        let pitch_deg = state.pitch.to_degrees() as f32;
        let bank = state.bank as f32;

        if self.elements.flight_path_ladder {
            draw_pitch_ladder(painter, center, pitch_deg, bank, px_per_deg, 120.0, color, color_dim);
        }

        // 前进（Prograde）— 沿速度方向；简化：中心上方随飞行路径角
        if self.elements.prograde_marker {
            let pro = rotate_pt(
                egui::pos2(center.x, center.y - state.pitch.to_degrees() as f32 * 0.3),
                center, bank,
            );
            draw_prograde(painter, pro, color);
        }
        // 后退（Retrograde）— 对侧
        if self.elements.retrograde_marker {
            let ret = rotate_pt(
                egui::pos2(center.x, center.y + state.pitch.to_degrees() as f32 * 0.3),
                center, bank,
            );
            draw_retrograde(painter, ret, color_dim);
        }

        // 左侧速度带
        if self.elements.airspeed {
            draw_side_tape(painter, rect.left() + 40.0, center.y, 160.0,
                state.speed, 100.0, "SPD", "m/s", color, color_dim, false);
        }
        // 右侧高度带
        if self.elements.altitude {
            draw_side_tape(painter, rect.right() - 40.0, center.y, 160.0,
                state.altitude, 5000.0, "ALT", "m", color, color_dim, true);
        }

        // 左上角：轨道快照
        let x0 = rect.left() + 12.0;
        let y0 = rect.top() + 28.0;
        let lh = 14.0;
        let block = [
            format!("REF   {}", state.focus_name),
            format!("Pe    {:>8.1} km", state.periapsis_alt / 1e3),
            format!("Ap    {:>8.1} km", state.apoapsis_alt / 1e3),
            format!("Ecc   {:>8.4}", state.eccentricity),
            format!("Inc   {:>7.2}\u{00b0}", state.inclination.to_degrees()),
            format!("T     {:>8.0} s", state.period),
        ];
        for (i, l) in block.iter().enumerate() {
            painter.text(
                egui::pos2(x0, y0 + (i as f32) * lh),
                egui::Align2::LEFT_TOP, l,
                egui::FontId::monospace(11.0), color,
            );
        }

        // 右上角：能量/质量
        let x1 = rect.right() - 12.0;
        let right_block = [
            format!("E    {:>9.2e}", state.specific_energy),
            format!("m    {:>7.0} kg", state.total_mass),
            format!("fuel {:>7.0} kg", state.fuel_mass),
            format!("thr  {:>6.0} %", state.throttle * 100.0),
            format!("T/W  {:>7.3}", state.tw_ratio),
        ];
        for (i, l) in right_block.iter().enumerate() {
            painter.text(
                egui::pos2(x1, y0 + (i as f32) * lh),
                egui::Align2::RIGHT_TOP, l,
                egui::FontId::monospace(11.0), color_dim,
            );
        }

        // 底部：轨道分类
        painter.text(
            egui::pos2(center.x, rect.bottom() - 34.0),
            egui::Align2::CENTER_BOTTOM,
            &format!("{}    a = {:.1} km", state.orbit_class(), state.semi_major_axis / 1e3),
            egui::FontId::monospace(12.0), color,
        );
    }

    // ------------------------------------------------------------------
    // Surface 模式：滚动地平线 + 俯仰梯 + 航向带 + 速度/高度/VS
    // ------------------------------------------------------------------
    fn draw_surface(
        &self, painter: &egui::Painter, rect: egui::Rect, state: &FlightState,
        color: egui::Color32, color_dim: egui::Color32,
    ) {
        let center = rect.center();
        let px_per_deg = 5.0_f32;
        let pitch_deg = state.pitch.to_degrees() as f32;
        let bank = state.bank as f32;

        if self.elements.horizon_line {
            draw_horizon(painter, center, pitch_deg, bank, px_per_deg, 260.0, color, color_dim);
        }
        if self.elements.pitch_ladder {
            draw_pitch_ladder(painter, center, pitch_deg, bank, px_per_deg, 90.0, color, color_dim);
        }
        if self.elements.heading_tape {
            let heading_deg = (state.yaw.to_degrees() as f32).rem_euclid(360.0);
            draw_heading_tape(painter, center.x, rect.top() + 38.0, 260.0,
                heading_deg, color, color_dim);
        }
        if self.elements.bank_indicator {
            draw_bank_indicator(painter, center, 100.0, bank, color, color_dim);
        }

        // 左侧速度带
        if self.elements.airspeed {
            draw_side_tape(painter, rect.left() + 40.0, center.y, 160.0,
                state.speed, 50.0, "SPD", "m/s", color, color_dim, false);
        }
        // 右侧高度带
        if self.elements.altitude {
            draw_side_tape(painter, rect.right() - 40.0, center.y, 160.0,
                state.altitude, 500.0, "ALT", "m", color, color_dim, true);
        }
        // 右下：垂直速度
        if self.elements.vertical_speed {
            let x = rect.right() - 12.0;
            let y = rect.bottom() - 60.0;
            painter.text(
                egui::pos2(x, y),
                egui::Align2::RIGHT_BOTTOM,
                &format!("V/S {:+7.1} m/s", state.vertical_speed),
                egui::FontId::monospace(12.0), color,
            );
        }

        // 左下：大气数据
        let x0 = rect.left() + 12.0;
        let y0 = rect.bottom() - 78.0;
        let lh = 14.0;
        let atmo = [
            format!("Mach  {:>5.2}", state.mach),
            format!("q     {:>5.0} Pa", state.dynamic_pressure),
            format!("\u{03c1}     {:>5.2e}", state.air_density),
        ];
        for (i, l) in atmo.iter().enumerate() {
            painter.text(
                egui::pos2(x0, y0 + (i as f32) * lh),
                egui::Align2::LEFT_TOP, l,
                egui::FontId::monospace(11.0), color_dim,
            );
        }
    }

    // ------------------------------------------------------------------
    // Docking 模式：目标框 + 相对位置十字 + 距离/接近率 + 相对速度矢量
    // ------------------------------------------------------------------
    fn draw_docking(
        &self, painter: &egui::Painter, rect: egui::Rect, state: &FlightState,
        color: egui::Color32, color_dim: egui::Color32,
    ) {
        let center = rect.center();

        if self.elements.approach_gates {
            // 大目标框（600m 视场标度）
            let gate = 80.0_f32;
            painter.rect_stroke(
                egui::Rect::from_center_size(center, egui::vec2(gate * 2.0, gate * 2.0)),
                0.0,
                egui::Stroke::new(1.5, color_dim),
                egui::StrokeKind::Outside,
            );
            // 内圈参考
            for r in [gate * 0.3, gate * 0.6] {
                painter.rect_stroke(
                    egui::Rect::from_center_size(center, egui::vec2(r * 2.0, r * 2.0)),
                    0.0,
                    egui::Stroke::new(1.0, color_dim),
                    egui::StrokeKind::Outside,
                );
            }
            // 四角标记
            let corner = 12.0;
            for (sx, sy) in [(-1.0, -1.0), (1.0, -1.0), (-1.0, 1.0), (1.0, 1.0)] {
                let p = center + egui::vec2(sx * gate, sy * gate);
                painter.line_segment(
                    [p, p + egui::vec2(-sx * corner, 0.0)],
                    egui::Stroke::new(2.0, color),
                );
                painter.line_segment(
                    [p, p + egui::vec2(0.0, -sy * corner)],
                    egui::Stroke::new(2.0, color),
                );
            }
        }

        // 相对速度矢量：把水平/垂直速度映射为屏幕偏移
        if self.elements.velocity_marker {
            let vx = state.horizontal_speed as f32 * 0.5;
            let vy = -state.vertical_speed as f32 * 0.5;
            let tip = center + egui::vec2(vx.clamp(-100.0, 100.0), vy.clamp(-100.0, 100.0));
            painter.line_segment([center, tip], egui::Stroke::new(1.5, color));
            painter.circle_filled(tip, 3.5, color);
        }

        // 距离 / 接近率 / 状态
        if self.elements.range_readout {
            let x = rect.left() + 20.0;
            let y = rect.top() + 40.0;
            let lh = 16.0;
            let closing = -state.vertical_speed;
            let block = [
                format!("RNG  {:>10.1} m", state.focus_dist),
                format!("V\u{2225}   {:>+8.2} m/s", closing),
                format!("V\u{22a5}   {:>+8.2} m/s", state.horizontal_speed),
                format!("TGO  {:>8.0} s",
                    if closing > 0.1 { state.focus_dist / closing } else { 0.0 }),
                format!("STAT   {}", if state.focus_dist < 5.0 { "DOCKED" }
                    else if closing > 0.0 { "APPROACH" } else { "DRIFT" }),
            ];
            for (i, l) in block.iter().enumerate() {
                painter.text(
                    egui::pos2(x, y + (i as f32) * lh),
                    egui::Align2::LEFT_TOP, l,
                    egui::FontId::monospace(12.0), color,
                );
            }
        }

        // 右侧：目标姿态占位
        let x1 = rect.right() - 20.0;
        let y1 = rect.top() + 40.0;
        let lh = 16.0;
        let att = [
            format!("PIT {:>+6.1}\u{00b0}", state.pitch.to_degrees()),
            format!("YAW {:>+6.1}\u{00b0}", state.yaw.to_degrees()),
            format!("BNK {:>+6.1}\u{00b0}", state.bank.to_degrees()),
        ];
        for (i, l) in att.iter().enumerate() {
            painter.text(
                egui::pos2(x1, y1 + (i as f32) * lh),
                egui::Align2::RIGHT_TOP, l,
                egui::FontId::monospace(12.0), color_dim,
            );
        }
    }
}

impl Default for HudState {
    fn default() -> Self { Self::new() }
}

// ======================================================================
//                              绘制辅助
// ======================================================================

fn rotate_pt(p: egui::Pos2, c: egui::Pos2, angle: f32) -> egui::Pos2 {
    let s = angle.sin();
    let co = angle.cos();
    let d = p - c;
    egui::pos2(c.x + d.x * co - d.y * s, c.y + d.x * s + d.y * co)
}

/// 俯仰梯：每 10° 一条横杠，正数上仰 / 负数俯冲。整体绕 center 按 bank 旋转。
fn draw_pitch_ladder(
    painter: &egui::Painter, center: egui::Pos2, pitch_deg: f32, bank: f32,
    px_per_deg: f32, half_width: f32, color: egui::Color32, dim: egui::Color32,
) {
    // pitch=+30 → 天线在中心之下 30*px；反之亦然（HUD 惯例：世界随姿态反向移动）
    for step in (-9..=9).map(|i| i * 10) {
        let raw_deg = step as f32;
        let y_offset = (pitch_deg - raw_deg) * px_per_deg;
        // 剔除超出屏幕的
        if y_offset.abs() > 180.0 { continue; }
        let is_positive = raw_deg > 0.0;
        let color_line = if raw_deg == 0.0 { color } else { dim };
        let width = if raw_deg == 0.0 { half_width * 1.6 } else { half_width };
        // 主线（负俯仰画虚线：切成 6 段）
        let left_end = egui::pos2(center.x - width, center.y + y_offset);
        let right_end = egui::pos2(center.x + width, center.y + y_offset);
        let left_r = rotate_pt(left_end, center, bank);
        let right_r = rotate_pt(right_end, center, bank);

        if is_positive || raw_deg == 0.0 {
            painter.line_segment([left_r, right_r], egui::Stroke::new(1.0, color_line));
        } else {
            // 虚线
            let segs = 6;
            for k in 0..segs {
                if k % 2 == 0 {
                    let t0 = k as f32 / segs as f32;
                    let t1 = (k as f32 + 1.0) / segs as f32;
                    let a = left_r + (right_r - left_r) * t0;
                    let b = left_r + (right_r - left_r) * t1;
                    painter.line_segment([a, b], egui::Stroke::new(1.0, color_line));
                }
            }
        }
        // 端部小钩
        let hook = 6.0_f32 * if raw_deg < 0.0 { 1.0 } else { -1.0 };
        let lh = rotate_pt(egui::pos2(left_end.x, left_end.y + hook), center, bank);
        let rh = rotate_pt(egui::pos2(right_end.x, right_end.y + hook), center, bank);
        painter.line_segment([left_r, lh], egui::Stroke::new(1.0, color_line));
        painter.line_segment([right_r, rh], egui::Stroke::new(1.0, color_line));
        // 数字（左右两端各一）
        if raw_deg != 0.0 {
            let label = format!("{:+}", raw_deg as i32);
            let label_pos = rotate_pt(
                egui::pos2(left_end.x - 4.0, left_end.y), center, bank);
            painter.text(label_pos, egui::Align2::RIGHT_CENTER, &label,
                egui::FontId::monospace(9.0), color_line);
        }
    }
}

/// 滚动地平线：单条粗线，随 pitch 上下平移，随 bank 旋转。
fn draw_horizon(
    painter: &egui::Painter, center: egui::Pos2, pitch_deg: f32, bank: f32,
    px_per_deg: f32, half_width: f32, color: egui::Color32, dim: egui::Color32,
) {
    let y_off = pitch_deg * px_per_deg;
    let a = rotate_pt(egui::pos2(center.x - half_width, center.y + y_off), center, bank);
    let b = rotate_pt(egui::pos2(center.x + half_width, center.y + y_off), center, bank);
    painter.line_segment([a, b], egui::Stroke::new(2.0, color));
    // 天/地记号
    for sign in [-1.0_f32, 1.0] {
        let tick_pos = egui::pos2(center.x + sign * (half_width * 0.6), center.y + y_off);
        let up = rotate_pt(egui::pos2(tick_pos.x, tick_pos.y - 6.0), center, bank);
        let dn = rotate_pt(egui::pos2(tick_pos.x, tick_pos.y + 6.0), center, bank);
        painter.line_segment([up, dn], egui::Stroke::new(1.0, dim));
    }
}

/// 航向带：顶部水平条，中心指示当前航向；每 10° 一小刻度，30° 一大刻度带 N/E/S/W 字符。
fn draw_heading_tape(
    painter: &egui::Painter, center_x: f32, y: f32, half_width: f32,
    heading_deg: f32, color: egui::Color32, dim: egui::Color32,
) {
    let px_per_deg = half_width / 45.0; // 显示 ±45°
    painter.line_segment(
        [egui::pos2(center_x - half_width, y), egui::pos2(center_x + half_width, y)],
        egui::Stroke::new(1.0, dim),
    );
    for d in (-45..=45_i32).step_by(5) {
        let world_hdg = (heading_deg + d as f32).rem_euclid(360.0);
        let x = center_x + (d as f32) * px_per_deg;
        let big = d.rem_euclid(30) == 0;
        let h = if big { 10.0 } else { 5.0 };
        painter.line_segment([egui::pos2(x, y), egui::pos2(x, y + h)],
            egui::Stroke::new(1.0, dim));
        if big {
            let label = match ((world_hdg / 45.0).round() as i32).rem_euclid(8) {
                0 => "N".into(), 1 => "NE".into(), 2 => "E".into(), 3 => "SE".into(),
                4 => "S".into(), 5 => "SW".into(), 6 => "W".into(), 7 => "NW".into(),
                _ => format!("{:03.0}", world_hdg),
            };
            painter.text(egui::pos2(x, y + h + 2.0),
                egui::Align2::CENTER_TOP, &label,
                egui::FontId::monospace(9.0), dim);
        }
    }
    // 中心指针
    painter.line_segment(
        [egui::pos2(center_x, y - 10.0), egui::pos2(center_x, y + 12.0)],
        egui::Stroke::new(2.0, color),
    );
    // 当前航向数字（大字，指针上方）
    painter.text(
        egui::pos2(center_x, y - 14.0),
        egui::Align2::CENTER_BOTTOM,
        &format!("{:03.0}\u{00b0}", heading_deg),
        egui::FontId::monospace(13.0), color,
    );
}

/// 侧边速度/高度带：竖直刻度，中心指针框内显示当前值。
fn draw_side_tape(
    painter: &egui::Painter, x: f32, center_y: f32, height: f32,
    value: f64, span_per_side: f64, label: &str, unit: &str,
    color: egui::Color32, dim: egui::Color32, right_side: bool,
) {
    // 主竖线
    let top = center_y - height * 0.5;
    let bot = center_y + height * 0.5;
    painter.line_segment(
        [egui::pos2(x, top), egui::pos2(x, bot)],
        egui::Stroke::new(1.0, dim),
    );
    // 刻度：每 span_per_side/5 一大刻，span_per_side/25 一小刻
    let px_per_unit = (height * 0.5) as f64 / span_per_side;
    let big_step = span_per_side / 5.0;
    let base = (value / big_step).floor() * big_step;
    for k in -5..=5 {
        let v = base + (k as f64) * big_step;
        let y = center_y - ((v - value) * px_per_unit) as f32;
        if y < top || y > bot { continue; }
        let tick_w = 10.0;
        let tx = if right_side { x - tick_w } else { x + tick_w };
        painter.line_segment(
            [egui::pos2(x, y), egui::pos2(tx, y)],
            egui::Stroke::new(1.0, dim),
        );
        let label_x = if right_side { x + 4.0 } else { x - 4.0 };
        let align = if right_side { egui::Align2::LEFT_CENTER } else { egui::Align2::RIGHT_CENTER };
        painter.text(egui::pos2(label_x, y), align,
            &format_short(v), egui::FontId::monospace(9.0), dim);
    }
    // 中心指针框
    let box_w = 48.0;
    let box_h = 16.0;
    let bx = if right_side { x - box_w - 2.0 } else { x + 2.0 };
    let rect = egui::Rect::from_min_size(
        egui::pos2(bx, center_y - box_h * 0.5),
        egui::vec2(box_w, box_h),
    );
    painter.rect_stroke(rect, 2.0, egui::Stroke::new(1.5, color), egui::StrokeKind::Outside);
    painter.text(rect.center(), egui::Align2::CENTER_CENTER,
        &format_short(value), egui::FontId::monospace(11.0), color);
    // 顶部标签
    painter.text(egui::pos2(x, top - 4.0), egui::Align2::CENTER_BOTTOM,
        label, egui::FontId::monospace(10.0), color);
    painter.text(egui::pos2(x, bot + 4.0), egui::Align2::CENTER_TOP,
        unit, egui::FontId::monospace(9.0), dim);
}

/// 短数值格式：自动 k / M 后缀。
fn format_short(v: f64) -> String {
    let a = v.abs();
    if a >= 1.0e6 { format!("{:.1}M", v / 1.0e6) }
    else if a >= 1.0e3 { format!("{:.1}k", v / 1.0e3) }
    else { format!("{:.0}", v) }
}

/// 前进（Prograde）向量标：圆 + 十字 + 三角。
fn draw_prograde(painter: &egui::Painter, p: egui::Pos2, color: egui::Color32) {
    let r = 7.0_f32;
    painter.circle_stroke(p, r, egui::Stroke::new(1.5, color));
    painter.line_segment([p + egui::vec2(-r - 4.0, 0.0), p + egui::vec2(-r, 0.0)],
        egui::Stroke::new(1.5, color));
    painter.line_segment([p + egui::vec2(r, 0.0), p + egui::vec2(r + 4.0, 0.0)],
        egui::Stroke::new(1.5, color));
    painter.line_segment([p + egui::vec2(0.0, -r - 4.0), p + egui::vec2(0.0, -r)],
        egui::Stroke::new(1.5, color));
}

/// 后退（Retrograde）向量标：圆 + X。
fn draw_retrograde(painter: &egui::Painter, p: egui::Pos2, color: egui::Color32) {
    let r = 7.0_f32;
    painter.circle_stroke(p, r, egui::Stroke::new(1.2, color));
    let d = r * 0.7;
    painter.line_segment([p + egui::vec2(-d, -d), p + egui::vec2(d, d)],
        egui::Stroke::new(1.2, color));
    painter.line_segment([p + egui::vec2(-d, d), p + egui::vec2(d, -d)],
        egui::Stroke::new(1.2, color));
}

/// 滚转指示器：中心上方弧线 + 顶部三角指针。
fn draw_bank_indicator(
    painter: &egui::Painter, center: egui::Pos2, radius: f32, bank: f32,
    color: egui::Color32, dim: egui::Color32,
) {
    // 顶部半圆刻度：-60,-30,0,+30,+60
    let arc_c = egui::pos2(center.x, center.y - radius * 0.4);
    for a_deg in [-60, -30, -10, 0, 10, 30, 60] {
        let a = (a_deg as f32).to_radians();
        let r0 = radius * 0.9;
        let r1 = radius * (if a_deg == 0 { 1.05 } else { 0.98 });
        let p0 = egui::pos2(arc_c.x + a.sin() * r0, arc_c.y - a.cos() * r0);
        let p1 = egui::pos2(arc_c.x + a.sin() * r1, arc_c.y - a.cos() * r1);
        painter.line_segment([p0, p1], egui::Stroke::new(1.0, dim));
    }
    // 三角指针（跟随 bank 反向）
    let a = -bank;
    let r_ptr = radius * 0.85;
    let tip = egui::pos2(arc_c.x + a.sin() * r_ptr, arc_c.y - a.cos() * r_ptr);
    let base = radius * 0.05;
    let left = rotate_pt(egui::pos2(tip.x - base, tip.y + base * 3.0), tip, a);
    let right = rotate_pt(egui::pos2(tip.x + base, tip.y + base * 3.0), tip, a);
    painter.line_segment([tip, left], egui::Stroke::new(1.5, color));
    painter.line_segment([tip, right], egui::Stroke::new(1.5, color));
    painter.line_segment([left, right], egui::Stroke::new(1.5, color));
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

    #[test]
    fn rotate_pt_identity() {
        let p = egui::pos2(1.0, 0.0);
        let c = egui::pos2(0.0, 0.0);
        let r = rotate_pt(p, c, 0.0);
        assert!((r.x - 1.0).abs() < 1e-6);
        assert!(r.y.abs() < 1e-6);
    }

    #[test]
    fn rotate_pt_quarter_turn() {
        let p = egui::pos2(1.0, 0.0);
        let c = egui::pos2(0.0, 0.0);
        let r = rotate_pt(p, c, std::f32::consts::FRAC_PI_2);
        assert!(r.x.abs() < 1e-6);
        assert!((r.y - 1.0).abs() < 1e-6);
    }

    #[test]
    fn format_short_ranges() {
        assert_eq!(format_short(42.0), "42");
        assert_eq!(format_short(1500.0), "1.5k");
        assert_eq!(format_short(2_500_000.0), "2.5M");
    }
}
