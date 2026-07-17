//! MFD（多功能显示器）— 移植自 Orbiter `Mfd.cpp`（1,290 行）+ 10 种内置 MFD。
//!
//! 使用 egui 窗口绘制，经典 CRT 绿色美学。P3D-3 阶段细化 4 种核心 MFD：
//! Orbit（椭圆轨道图）、Map（地面轨迹）、Docking（雷达显示）、Landing（ILS）。

use std::f32::consts::PI;

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
    warn_color: egui::Color32,
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
            warn_color: egui::Color32::from_rgb(255, 220, 60),
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
        match self.mfd_type {
            MfdType::Orbit => self.draw_orbit(painter, rect, state),
            MfdType::Map => self.draw_map(painter, rect, state),
            MfdType::Docking => self.draw_docking(painter, rect, state),
            MfdType::Landing => self.draw_landing(painter, rect, state),
            MfdType::Surface => self.draw_surface(painter, rect, state),
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

    // ------------------------------------------------------------------
    // Orbit MFD：真实椭圆 + 近远点 / 升降交点标注
    // ------------------------------------------------------------------
    fn draw_orbit(&self, painter: &egui::Painter, rect: egui::Rect, state: &FlightState) {
        let a = state.semi_major_axis as f32;
        let e = state.eccentricity as f32;
        let parent_r = (state.focus_dist - state.altitude) as f32;

        // 左侧数值列
        let x0 = rect.left() + 4.0;
        let y0 = rect.top() + 4.0;
        let lh = 13.0_f32;
        let block = [
            format!("REF   {}", state.focus_name),
            format!("a     {:>9.1} km", a * 1e-3),
            format!("e     {:>9.5}", e),
            format!("i     {:>8.2}\u{00b0}", state.inclination.to_degrees()),
            format!("Pe    {:>9.1} km", state.periapsis_alt / 1e3),
            format!("Ap    {:>9.1} km", state.apoapsis_alt / 1e3),
            format!("T     {:>9.0} s", state.period),
            format!("\u{03b5}     {:>9.2e}", state.specific_energy),
            format!("class {:>9}", state.orbit_class()),
        ];
        for (i, l) in block.iter().enumerate() {
            painter.text(
                egui::pos2(x0, y0 + (i as f32) * lh),
                egui::Align2::LEFT_TOP, l,
                egui::FontId::monospace(10.0),
                self.fg_color,
            );
        }

        // 右侧：真实椭圆
        let plot_center = egui::pos2(
            rect.left() + rect.width() * 0.65,
            rect.top() + rect.height() * 0.55,
        );
        let plot_radius = rect.width().min(rect.height()) * 0.28;

        if a > 0.0 && e >= 0.0 && e < 1.0 {
            // 尺度：让远地点（a*(1+e)）刚好占 plot_radius
            let scale = plot_radius / (a * (1.0 + e).max(1.0));
            let b = a * (1.0 - e * e).sqrt();
            // 椭圆焦点在原点 → 椭圆中心平移 (+ae)（近地点在 +x）
            let offset = a * e * scale;
            let ellipse_c = egui::pos2(plot_center.x - offset, plot_center.y);

            let n = 96;
            let mut prev: Option<egui::Pos2> = None;
            for k in 0..=n {
                let t = (k as f32 / n as f32) * PI * 2.0;
                let px = ellipse_c.x + (a * scale) * t.cos();
                let py = ellipse_c.y + (b * scale) * t.sin();
                let p = egui::pos2(px, py);
                if let Some(pv) = prev {
                    painter.line_segment([pv, p], egui::Stroke::new(1.2, self.fg_color));
                }
                prev = Some(p);
            }

            // 父天体（原点，即焦点）
            let body_r = (parent_r * scale).max(2.0).min(plot_radius * 0.35);
            painter.circle_stroke(plot_center, body_r,
                egui::Stroke::new(1.0, self.dim_color));
            painter.circle_filled(plot_center, 1.5, self.fg_color);

            // 近地点 (+x) / 远地点 (-x)
            let pe = egui::pos2(plot_center.x + a * (1.0 - e) * scale, plot_center.y);
            let ap = egui::pos2(plot_center.x - a * (1.0 + e) * scale, plot_center.y);
            painter.circle_filled(pe, 3.0, self.fg_color);
            painter.text(pe + egui::vec2(4.0, -2.0), egui::Align2::LEFT_BOTTOM,
                "Pe", egui::FontId::monospace(9.0), self.fg_color);
            painter.circle_filled(ap, 3.0, self.dim_color);
            painter.text(ap + egui::vec2(-4.0, -2.0), egui::Align2::RIGHT_BOTTOM,
                "Ap", egui::FontId::monospace(9.0), self.dim_color);

            // 当前航天器位置：由 r=focus_dist 反解真近点角 ν
            let r_now = state.focus_dist as f32;
            let semi_latus = a * (1.0 - e * e);
            let cos_nu = if e > 1e-6 {
                ((semi_latus / r_now - 1.0) / e).clamp(-1.0, 1.0)
            } else { 1.0 };
            // 用径向速度符号决定 ν 象限：v_r > 0 → 从近点向远点飞 → sin(ν) > 0
            let sign = if state.vertical_speed >= 0.0 { 1.0 } else { -1.0 };
            let nu = cos_nu.acos() * sign;
            let sc_pos = egui::pos2(
                plot_center.x + r_now * scale * nu.cos(),
                plot_center.y - r_now * scale * nu.sin(),
            );
            painter.circle_filled(sc_pos, 3.5, self.warn_color);
            painter.line_segment([plot_center, sc_pos],
                egui::Stroke::new(1.0, self.dim_color));

            // AN/DN（升/降交点）：由于二维投影 = 轨道平面本身，只在标签里提示
            painter.text(
                egui::pos2(plot_center.x, plot_center.y + plot_radius + 4.0),
                egui::Align2::CENTER_TOP,
                &format!("plane view  \u{03bd} = {:+.0}\u{00b0}", nu.to_degrees()),
                egui::FontId::monospace(9.0), self.dim_color,
            );
        } else {
            painter.text(plot_center, egui::Align2::CENTER_CENTER,
                "no valid orbit",
                egui::FontId::monospace(11.0), self.dim_color);
        }
    }

    // ------------------------------------------------------------------
    // Map MFD：等距圆柱投影 + 经纬网格 + 当前位置 + 前 N 步轨迹
    // ------------------------------------------------------------------
    fn draw_map(&self, painter: &egui::Painter, rect: egui::Rect, state: &FlightState) {
        // 保持 2:1 长宽比
        let map_h = rect.height().min(rect.width() * 0.5);
        let map_w = map_h * 2.0;
        let cx = rect.center().x;
        let cy = rect.top() + map_h * 0.5 + 4.0;
        let map_rect = egui::Rect::from_center_size(
            egui::pos2(cx, cy), egui::vec2(map_w, map_h),
        );
        painter.rect_stroke(map_rect, 0.0,
            egui::Stroke::new(1.0, self.dim_color),
            egui::StrokeKind::Outside);

        // 经度网（每 30°）与纬度网（每 30°）
        for lon in (-180..=180_i32).step_by(30) {
            let x = map_rect.left() + ((lon + 180) as f32 / 360.0) * map_w;
            painter.line_segment(
                [egui::pos2(x, map_rect.top()), egui::pos2(x, map_rect.bottom())],
                egui::Stroke::new(0.6, self.dim_color),
            );
            if lon.rem_euclid(60) == 0 {
                painter.text(egui::pos2(x, map_rect.bottom() + 2.0),
                    egui::Align2::CENTER_TOP,
                    &format!("{:+}", lon),
                    egui::FontId::monospace(8.0), self.dim_color);
            }
        }
        for lat in (-90..=90).step_by(30) {
            let y = map_rect.bottom() - ((lat + 90) as f32 / 180.0) * map_h;
            painter.line_segment(
                [egui::pos2(map_rect.left(), y), egui::pos2(map_rect.right(), y)],
                egui::Stroke::new(0.6, self.dim_color),
            );
            if lat != 90 && lat != -90 {
                painter.text(egui::pos2(map_rect.left() - 2.0, y),
                    egui::Align2::RIGHT_CENTER,
                    &format!("{:+}", lat),
                    egui::FontId::monospace(8.0), self.dim_color);
            }
        }
        // 赤道加粗
        let eq_y = map_rect.center().y;
        painter.line_segment(
            [egui::pos2(map_rect.left(), eq_y), egui::pos2(map_rect.right(), eq_y)],
            egui::Stroke::new(1.0, self.fg_color),
        );

        // 位置向量 → 经纬（λ 沿轨道演化，φ 由倾角 + 相位近似）
        let pos = state.position;
        let r = (pos.x * pos.x + pos.y * pos.y + pos.z * pos.z).sqrt().max(1.0);
        let lat_now = (pos.z / r).asin().to_degrees() as f32;
        let lon_now = (pos.y.atan2(pos.x).to_degrees() as f32).rem_euclid(360.0) - 180.0;
        let mean_motion = if state.period > 0.0 { 360.0 / state.period as f32 } else { 0.0 };
        let inc_deg = state.inclination.to_degrees() as f32;

        // 前向 60 分钟内轨迹（简化：绕轨迹面匀速前进 + Earth 24h 旋转补偿）
        let earth_rot = 360.0 / 86164.0_f32; // 度/秒（恒星日）
        let dt_step = 60.0_f32;
        let steps = 60;
        let mut prev: Option<egui::Pos2> = None;
        for k in 0..=steps {
            let dt = (k as f32) * dt_step;
            let lon = ((lon_now + mean_motion * dt) - earth_rot * dt + 540.0).rem_euclid(360.0) - 180.0;
            let phase = (state.sim_time as f32 * mean_motion + mean_motion * dt) * PI / 180.0;
            let lat2 = inc_deg * phase.sin();
            let p = latlon_to_screen(map_rect, lon, lat2);
            if let Some(pv) = prev {
                painter.line_segment([pv, p], egui::Stroke::new(1.0, self.dim_color));
            }
            prev = Some(p);
        }

        // 当前位置：亮点
        let cur = latlon_to_screen(map_rect,
            lon_now,
            inc_deg * ((state.sim_time as f32) * mean_motion * PI / 180.0).sin());
        painter.circle_filled(cur, 3.5, self.warn_color);
        painter.circle_stroke(cur, 5.0, egui::Stroke::new(1.0, self.warn_color));

        // 底部数值
        let y0 = map_rect.bottom() + 18.0;
        let lh = 12.0;
        let lines = [
            format!("REF {}   ALT {:.1} km", state.focus_name, state.altitude / 1e3),
            format!("LON {:+7.2}\u{00b0}   LAT {:+6.2}\u{00b0}", lon_now, lat_now),
            format!("HDG {:03.0}\u{00b0}   SPD {:.2} km/s",
                (state.yaw.to_degrees() as f32).rem_euclid(360.0),
                state.speed / 1e3),
        ];
        for (i, l) in lines.iter().enumerate() {
            painter.text(egui::pos2(rect.left() + 4.0, y0 + (i as f32) * lh),
                egui::Align2::LEFT_TOP, l,
                egui::FontId::monospace(10.0), self.fg_color);
        }
    }

    // ------------------------------------------------------------------
    // Docking MFD：雷达式显示，距离环 + 目标点 + 接近走廊
    // ------------------------------------------------------------------
    fn draw_docking(&self, painter: &egui::Painter, rect: egui::Rect, state: &FlightState) {
        let plot_c = egui::pos2(rect.center().x, rect.top() + rect.height() * 0.45);
        let plot_r = rect.width().min(rect.height() * 0.9) * 0.35;
        // 三层距离环
        for (f, label) in [(1.0_f32, "R"), (0.66, ""), (0.33, "")] {
            painter.circle_stroke(plot_c, plot_r * f,
                egui::Stroke::new(1.0, self.dim_color));
            if !label.is_empty() {
                painter.text(
                    egui::pos2(plot_c.x + plot_r * f + 4.0, plot_c.y),
                    egui::Align2::LEFT_CENTER,
                    &format!("{:.0}m", state.focus_dist),
                    egui::FontId::monospace(9.0), self.dim_color,
                );
            }
        }
        // 十字
        painter.line_segment(
            [egui::pos2(plot_c.x - plot_r, plot_c.y), egui::pos2(plot_c.x + plot_r, plot_c.y)],
            egui::Stroke::new(0.6, self.dim_color));
        painter.line_segment(
            [egui::pos2(plot_c.x, plot_c.y - plot_r), egui::pos2(plot_c.x, plot_c.y + plot_r)],
            egui::Stroke::new(0.6, self.dim_color));

        // 接近走廊：以 +y 方向为参考轴的锥形线（±5°）
        for sign in [-1.0_f32, 1.0] {
            let a = 5.0_f32.to_radians() * sign;
            let tip = egui::pos2(plot_c.x + a.sin() * plot_r, plot_c.y - a.cos() * plot_r);
            painter.line_segment([plot_c, tip],
                egui::Stroke::new(0.8, self.dim_color));
        }

        // 目标点：用 (horizontal_speed, vertical_speed) 投影表示
        let scale = plot_r / 200.0; // 200 m/s 满量程
        let tx = (state.horizontal_speed as f32) * scale;
        let ty = -(state.vertical_speed as f32) * scale;
        let target = egui::pos2(plot_c.x + tx.clamp(-plot_r, plot_r),
                                plot_c.y + ty.clamp(-plot_r, plot_r));
        painter.circle_filled(target, 4.0, self.warn_color);
        painter.line_segment([plot_c, target],
            egui::Stroke::new(1.0, self.warn_color));

        // 数值块
        let y0 = plot_c.y + plot_r + 12.0;
        let lh = 12.0;
        let closing = -state.vertical_speed;
        let tgo = if closing > 0.1 { state.focus_dist / closing } else { 0.0 };
        let block = [
            format!("TGT     {}", state.focus_name),
            format!("RNG   {:>9.1} m", state.focus_dist),
            format!("V\u{2225}   {:>+8.2} m/s", closing),
            format!("V\u{22a5}   {:>+8.2} m/s", state.horizontal_speed),
            format!("TGO   {:>8.0} s", tgo),
            format!("STAT  {}", if state.focus_dist < 5.0 { "DOCKED" }
                else if closing > 0.0 { "APPROACH" } else { "DRIFT" }),
        ];
        for (i, l) in block.iter().enumerate() {
            painter.text(egui::pos2(rect.left() + 4.0, y0 + (i as f32) * lh),
                egui::Align2::LEFT_TOP, l,
                egui::FontId::monospace(10.0), self.fg_color);
        }
    }

    // ------------------------------------------------------------------
    // Landing MFD：ILS（水平/垂直偏差针）+ 高度剖面 + 距离读数
    // ------------------------------------------------------------------
    fn draw_landing(&self, painter: &egui::Painter, rect: egui::Rect, state: &FlightState) {
        // 上半部分：ILS 双针指示器
        let ils_c = egui::pos2(rect.center().x, rect.top() + rect.height() * 0.30);
        let ils_r = rect.width().min(rect.height() * 0.6) * 0.30;
        painter.circle_stroke(ils_c, ils_r,
            egui::Stroke::new(1.0, self.dim_color));
        painter.circle_stroke(ils_c, ils_r * 0.5,
            egui::Stroke::new(0.6, self.dim_color));
        // 十字刻度
        for k in [-1.0_f32, -0.5, 0.5, 1.0] {
            let x = ils_c.x + k * ils_r;
            painter.line_segment(
                [egui::pos2(x, ils_c.y - 4.0), egui::pos2(x, ils_c.y + 4.0)],
                egui::Stroke::new(0.6, self.dim_color));
            let y = ils_c.y + k * ils_r;
            painter.line_segment(
                [egui::pos2(ils_c.x - 4.0, y), egui::pos2(ils_c.x + 4.0, y)],
                egui::Stroke::new(0.6, self.dim_color));
        }

        // 假想目标：LON=0, LAT=0 的地面点，以当前 yaw 距标为航向偏差
        // 航向偏差：yaw 与"正北"的角度（假设跑道朝北）
        let hdg_dev = (state.yaw.to_degrees() as f32).rem_euclid(360.0);
        let hdg_err = if hdg_dev > 180.0 { hdg_dev - 360.0 } else { hdg_dev };
        let loc_offset = (hdg_err / 10.0).clamp(-1.0, 1.0) * ils_r; // ±10° 满量程

        // 下滑道：期望 3° 下滑；当前航迹角 = atan2(vs, hs)
        let fpa_deg = if state.horizontal_speed.abs() > 1.0 {
            (-state.vertical_speed).atan2(state.horizontal_speed).to_degrees() as f32
        } else { 0.0 };
        let gs_err = fpa_deg - 3.0;
        let gs_offset = (gs_err / 5.0).clamp(-1.0, 1.0) * ils_r; // ±5° 满量程

        // 垂直针（localizer / 航向偏差）
        painter.line_segment(
            [egui::pos2(ils_c.x + loc_offset, ils_c.y - ils_r),
             egui::pos2(ils_c.x + loc_offset, ils_c.y + ils_r)],
            egui::Stroke::new(2.0, self.warn_color));
        // 水平针（glideslope）
        painter.line_segment(
            [egui::pos2(ils_c.x - ils_r, ils_c.y + gs_offset),
             egui::pos2(ils_c.x + ils_r, ils_c.y + gs_offset)],
            egui::Stroke::new(2.0, self.warn_color));
        // 飞机符号（中心）
        painter.circle_stroke(ils_c, 5.0,
            egui::Stroke::new(1.5, self.fg_color));
        painter.line_segment(
            [ils_c + egui::vec2(-8.0, 0.0), ils_c + egui::vec2(8.0, 0.0)],
            egui::Stroke::new(1.5, self.fg_color));
        painter.line_segment(
            [ils_c + egui::vec2(0.0, 0.0), ils_c + egui::vec2(0.0, 8.0)],
            egui::Stroke::new(1.5, self.fg_color));

        // 侧标
        painter.text(egui::pos2(ils_c.x - ils_r - 6.0, ils_c.y - ils_r - 4.0),
            egui::Align2::LEFT_BOTTOM, "GS",
            egui::FontId::monospace(9.0), self.dim_color);
        painter.text(egui::pos2(ils_c.x - ils_r - 6.0, ils_c.y + ils_r + 4.0),
            egui::Align2::LEFT_TOP, "LOC",
            egui::FontId::monospace(9.0), self.dim_color);

        // 下半部分：高度剖面（altitude vs distance-to-landing）
        let prof_top = rect.top() + rect.height() * 0.62;
        let prof = egui::Rect::from_min_max(
            egui::pos2(rect.left() + 24.0, prof_top),
            egui::pos2(rect.right() - 4.0, prof_top + rect.height() * 0.20),
        );
        painter.rect_stroke(prof, 0.0,
            egui::Stroke::new(1.0, self.dim_color),
            egui::StrokeKind::Outside);
        // 3° 下滑参考线（从右下到左上）
        painter.line_segment(
            [egui::pos2(prof.right(), prof.bottom()),
             egui::pos2(prof.left(), prof.top())],
            egui::Stroke::new(1.0, self.dim_color));
        // 当前航天器（右上角相对位置）
        let alt_frac = (state.altitude / 100_000.0).clamp(0.0, 1.0) as f32;
        let dist_frac = 0.85_f32; // 简化：假设距 landing site 尚远
        let sc = egui::pos2(
            prof.right() - dist_frac * prof.width(),
            prof.bottom() - alt_frac * prof.height(),
        );
        painter.circle_filled(sc, 3.5, self.warn_color);
        painter.text(egui::pos2(prof.left() + 2.0, prof.top() - 2.0),
            egui::Align2::LEFT_BOTTOM, "ALT",
            egui::FontId::monospace(9.0), self.dim_color);
        painter.text(egui::pos2(prof.right() - 2.0, prof.bottom() + 2.0),
            egui::Align2::RIGHT_TOP, "DIST",
            egui::FontId::monospace(9.0), self.dim_color);

        // 数值块（底部）
        let y0 = prof.bottom() + 12.0;
        let lh = 12.0;
        let block = [
            format!("NAV     KSC RWY 33 (proxy)"),
            format!("ALT   {:>9.1} km  \u{0394}HDG {:+6.1}\u{00b0}", state.altitude / 1e3, hdg_err),
            format!("V/S   {:+8.1} m/s  FPA {:+6.2}\u{00b0}", state.vertical_speed, fpa_deg),
            format!("SPD   {:>8.1} m/s  GS  {:+6.2}\u{00b0}", state.speed, gs_err),
        ];
        for (i, l) in block.iter().enumerate() {
            painter.text(egui::pos2(rect.left() + 4.0, y0 + (i as f32) * lh),
                egui::Align2::LEFT_TOP, l,
                egui::FontId::monospace(10.0), self.fg_color);
        }
    }

    // ------------------------------------------------------------------
    // Surface MFD：数据表（保留原样式）
    // ------------------------------------------------------------------
    fn draw_surface(&self, painter: &egui::Painter, rect: egui::Rect, state: &FlightState) {
        let left = rect.left();
        let top = rect.top();
        let line_h = 14.0_f32;
        let lines = [
            format!("Alt  {}", state.fmt_altitude()),
            format!("Spd  {}", state.fmt_speed()),
            format!("V/S  {:+.1} m/s", state.vertical_speed),
            format!("Hdg  {:03.0}\u{00b0}", (state.yaw.to_degrees() as f32).rem_euclid(360.0)),
            format!("Pit  {:+.2}\u{00b0}", state.pitch.to_degrees()),
            format!("Bnk  {:+.2}\u{00b0}", state.bank.to_degrees()),
            format!("\u{03c1}    {:.3e} kg/m\u{00b3}", state.air_density),
            format!("q    {:.1} Pa", state.dynamic_pressure),
            format!("Mach {:.2}", state.mach),
            format!("T/W  {:.3}", state.tw_ratio),
        ];
        for (i, line) in lines.iter().enumerate() {
            painter.text(
                egui::pos2(left + 4.0, top + 4.0 + (i as f32) * line_h),
                egui::Align2::LEFT_TOP, line,
                egui::FontId::monospace(11.0), self.fg_color);
        }
    }
}

/// 经纬度 → 屏幕坐标（等距圆柱投影）。
fn latlon_to_screen(rect: egui::Rect, lon_deg: f32, lat_deg: f32) -> egui::Pos2 {
    let x = rect.left() + ((lon_deg + 180.0).rem_euclid(360.0) / 360.0) * rect.width();
    let y = rect.bottom() - ((lat_deg.clamp(-90.0, 90.0) + 90.0) / 180.0) * rect.height();
    egui::pos2(x, y)
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

    #[test]
    fn latlon_to_screen_bounds() {
        let r = egui::Rect::from_min_size(egui::pos2(0.0, 0.0), egui::vec2(360.0, 180.0));
        let p = latlon_to_screen(r, 0.0, 0.0);
        assert!((p.x - 180.0).abs() < 1e-4);
        assert!((p.y - 90.0).abs() < 1e-4);
        let p = latlon_to_screen(r, -180.0, 90.0);
        assert!(p.x.abs() < 1e-4);
        assert!(p.y.abs() < 1e-4);
    }
}
