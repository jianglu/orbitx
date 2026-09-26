//! 基于 Slice 的 TUI 绘制（不持有 Assembly）。

use crate::focus::SliceFocus;
use orbitx_protocol::{FocusTelem, Slice};
use ratatui::layout::{Constraint, Layout};
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Span};
use ratatui::widgets::{Block, Borders, Gauge, Paragraph, Row, Table};
use ratatui::Frame;

fn fmt_time(ms: u64) -> String {
    let t = ms / 1000;
    format!("T+{:02}:{:02}:{:02}", t / 3600, (t % 3600) / 60, t % 60)
}

fn fmt_dist(m: f64) -> String {
    if m.abs() >= 1000.0 {
        format!("{:.1} km", m / 1e3)
    } else {
        format!("{:.0} m", m)
    }
}

fn fmt_mass(kg: f64) -> String {
    if kg >= 1000.0 {
        format!("{:.1} t", kg / 1000.0)
    } else {
        format!("{:.0} kg", kg)
    }
}

fn fmt_dms(deg: f64, pos_hemi: &str, neg_hemi: &str) -> String {
    let hemi = if deg < 0.0 { neg_hemi } else { pos_hemi };
    let mut d = deg.abs();
    let deg_part = d.trunc();
    d = (d - deg_part) * 60.0;
    let min_part = d.trunc();
    let sec_part = (d - min_part) * 60.0;
    format!(
        "{}°{}′{:05.2}″{}",
        deg_part as i32, min_part as i32, sec_part, hemi
    )
}

fn fmt_alt(m: f64) -> String {
    if m.abs() >= 1000.0 {
        format!("{:.2} km", m / 1000.0)
    } else {
        format!("{:.0} m", m)
    }
}

/// 显示用：若四舍五入到 `decimals` 位后为 0，则返回 +0.0。
fn scrub_display_zero(x: f64, decimals: u32) -> f64 {
    let scale = 10f64.powi(decimals as i32);
    if (x * scale).round().abs() < f64::EPSILON {
        0.0
    } else {
        x
    }
}

fn fuel_bar(fuel_kg: f64, fuel_pct: f64, empty_fuel: bool, no_thrust_hint: bool) -> String {
    if no_thrust_hint && empty_fuel {
        return "—".into();
    }
    let pct = (fuel_pct / 10.0).clamp(0.0, 10.0) as usize;
    format!(
        "[{}{}] {}",
        "#".repeat(pct),
        ".".repeat(10 - pct.min(10)),
        fmt_mass(fuel_kg)
    )
}

fn lat_lng_from_pos(f: &FocusTelem) -> (f64, f64) {
    let r = (f.pos_x * f.pos_x + f.pos_y * f.pos_y + f.pos_z * f.pos_z)
        .sqrt()
        .max(1e-3);
    let lat = (f.pos_y / r).asin().to_degrees();
    let lng = f.pos_z.atan2(f.pos_x).to_degrees();
    (lat, lng)
}

/// `focus`: Primary → `slice.focus`；Detached → `slice.detached[i]`。
pub fn draw(frame: &mut Frame, slice: &Slice, focus: SliceFocus, local_throttle: f64) {
    let telem = focus.telem(slice);
    let is_primary = focus.is_primary();
    let display_name = focus.display_name(slice);
    let view_vessel = focus.view_vessel_index(slice);

    let [title_area, main_area, fuel_area, help_area] = Layout::vertical([
        Constraint::Length(3),
        Constraint::Min(0),
        Constraint::Length(3),
        Constraint::Length(3),
    ])
    .areas(frame.area());

    let [left_area, right_area] =
        Layout::horizontal([Constraint::Percentage(33), Constraint::Percentage(67)])
            .areas(main_area);
    let [telem_area, pad_area] =
        Layout::vertical([Constraint::Min(0), Constraint::Length(9)]).areas(left_area);

    // Title
    let stage_left = slice.stages.iter().filter(|s| !s.detached && !s.crashed).count();
    let focus_tag = if is_primary {
        String::new()
    } else {
        format!(" 观察: {display_name} ")
    };
    let title = format!(
        " orbitx 发射模拟器 — {}  {}  Stage: {}  (剩余 {} 级){} ",
        slice.rocket_name,
        fmt_time(slice.sim_t),
        slice.active_name,
        stage_left,
        focus_tag,
    );
    let thrusting = if is_primary {
        local_throttle > 1e-6 || slice.throttle_cmd > 1e-6
    } else {
        telem
            .map(|f| f.throttle > 1e-6 || f.thrust > 1.0)
            .unwrap_or(false)
    };
    let status_tags = if slice.paused {
        " [暂停]"
    } else if thrusting {
        " [推力]"
    } else {
        ""
    };
    let lock_tag = if is_primary { "" } else { " [控制锁定]" };
    let gravity_tag = if slice.gravity_turn {
        " [重力转向]"
    } else {
        ""
    };
    let warp_tag = if slice.warp > 1.5 {
        format!(" [{:.0}x]", slice.warp)
    } else {
        String::new()
    };
    let crash_tag = if !slice.crash_msg.is_empty() {
        format!(" !!! {} !!!", slice.crash_msg)
    } else {
        String::new()
    };
    let right_spans = vec![
        Span::styled(
            "[Zenoh]",
            Style::default().fg(Color::Green).bold().bg(Color::Black),
        ),
        Span::styled(status_tags, Style::default().fg(Color::White).bg(Color::Black)),
        Span::styled(lock_tag, Style::default().fg(Color::Yellow).bg(Color::Black)),
        Span::styled(
            gravity_tag,
            Style::default().fg(Color::Yellow).bg(Color::Black),
        ),
        Span::styled(warp_tag, Style::default().fg(Color::Cyan).bg(Color::Black)),
        Span::styled(
            crash_tag,
            Style::default().fg(Color::Red).bold().bg(Color::Black),
        ),
    ];
    let right_width: usize = right_spans.iter().map(|s| s.width()).sum();
    let avail = title_area.width.saturating_sub(2) as usize;
    let pad = avail.saturating_sub(ratatui::text::Text::from(title.as_str()).width() + right_width);
    let mut title_spans = vec![
        Span::styled(
            title.clone(),
            Style::default()
                .fg(Color::White)
                .add_modifier(Modifier::BOLD)
                .bg(Color::Black),
        ),
        Span::styled(" ".repeat(pad), Style::default().bg(Color::Black)),
    ];
    title_spans.extend(right_spans);
    frame.render_widget(
        Block::default()
            .borders(Borders::ALL)
            .title(Line::from(title_spans))
            .style(Style::default().fg(Color::White).bg(Color::Black)),
        title_area,
    );

    let label = Style::default().fg(Color::Cyan).bold();
    let value = Style::default().fg(Color::White);
    let danger = Style::default().fg(Color::Red).bold();
    let warning = Style::default().fg(Color::Yellow).bold();
    let normal = Style::default().fg(Color::White);

    let (alt, speed, v_vert, v_horiz, mass, fuel, fuel_pct, thrust, twr, env) = match telem {
        Some(f) => (
            f.altitude,
            f.speed,
            f.v_vert,
            f.v_horiz,
            f.mass,
            f.fuel,
            f.fuel_pct,
            f.thrust,
            f.twr,
            f.env.as_ref(),
        ),
        None => (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, None),
    };

    let alt_style = if alt < 0.0 { danger } else { normal };
    let tw_style = if twr < 1.0 && thrusting && is_primary {
        warning
    } else {
        normal
    };
    let fuel_style = if fuel < 1.0 {
        danger
    } else if fuel_pct < 20.0 {
        warning
    } else {
        normal
    };

    let s_alt = fmt_dist(alt.max(0.0));
    let s_vel = format!("{:.0} m/s", speed);
    let s_vvert = format!("{:.0} m/s", v_vert);
    let s_vhoriz = format!("{:.0} m/s", v_horiz);
    let s_mass = fmt_mass(mass);
    let s_fuel = format!("{:.0} kg", fuel);
    let s_thrust = format!("{:.0} kN", (thrust / 1000.0).abs());
    let s_tw = format!("{:.2}", twr.abs());

    let mut rows = vec![
        Row::new(vec![
            ratatui::widgets::Cell::from("高度 Alt").style(label),
            ratatui::widgets::Cell::from(s_alt.as_str()).style(alt_style),
        ]),
        Row::new(vec![
            ratatui::widgets::Cell::from("速度 Vel").style(label),
            ratatui::widgets::Cell::from(s_vel.as_str()).style(value),
        ]),
        Row::new(vec![
            ratatui::widgets::Cell::from("垂直速度 Vvert").style(label),
            ratatui::widgets::Cell::from(s_vvert.as_str()).style(value),
        ]),
        Row::new(vec![
            ratatui::widgets::Cell::from("水平速度 Vhoriz").style(label),
            ratatui::widgets::Cell::from(s_vhoriz.as_str()).style(value),
        ]),
        Row::new(vec![
            ratatui::widgets::Cell::from("质量 Mass").style(label),
            ratatui::widgets::Cell::from(s_mass.as_str()).style(value),
        ]),
        Row::new(vec![
            ratatui::widgets::Cell::from("燃料 Fuel").style(label),
            ratatui::widgets::Cell::from(s_fuel.as_str()).style(fuel_style),
        ]),
        Row::new(vec![
            ratatui::widgets::Cell::from("推力 Thrust").style(label),
            ratatui::widgets::Cell::from(s_thrust.as_str()).style(value),
        ]),
        Row::new(vec![
            ratatui::widgets::Cell::from("推重比 T/W").style(label),
            ratatui::widgets::Cell::from(s_tw.as_str()).style(tw_style),
        ]),
    ];
    if let Some(d) = env {
        let extras = [
            ("引力加速度 Grav", format!("{:.3} m/s²", d.a_grav)),
            ("重力倍数 Gmul", format!("{:.3}", d.g_multiple)),
            ("过载 Load", format!("{:.2}", d.load_factor)),
            ("马赫数 Mach", format!("{:.2}", d.mach)),
            ("大气密度 Rho", format!("{:.4} kg/m³", d.density)),
            (
                "动压 Q",
                if d.dynamic_pressure >= 1000.0 {
                    format!("{:.1} kPa", d.dynamic_pressure / 1000.0)
                } else {
                    format!("{:.0} Pa", d.dynamic_pressure)
                },
            ),
            (
                "大气压 P",
                if d.pressure >= 1000.0 {
                    format!("{:.1} kPa", d.pressure / 1000.0)
                } else {
                    format!("{:.0} Pa", d.pressure)
                },
            ),
            ("推力气压修正 Pfac", format!("{:.3}", d.thrust_atm_scale)),
            ("有效比冲 Isp", format!("{:.0} s", d.isp_eff)),
            ("气动阻力 Drag", format!("{:.0} N", d.drag_force)),
            ("有效阻力系数 Cd", format!("{:.3}", d.cd_eff)),
            ("气温 Temp", format!("{:.1} °C", d.temperature - 273.15)),
            ("声速 Asnd", format!("{:.0} m/s", d.sound_speed)),
        ];
        for (k, v) in extras {
            rows.push(Row::new(vec![
                ratatui::widgets::Cell::from(k).style(label),
                ratatui::widgets::Cell::from(v).style(value),
            ]));
        }
    }
    frame.render_widget(
        Table::new(rows, [Constraint::Percentage(45), Constraint::Percentage(55)])
            .style(Style::default().fg(Color::White).bg(Color::Black))
            .column_spacing(1)
            .block(
                Block::default()
                    .borders(Borders::ALL)
                    .title(if is_primary {
                        " 遥测 Telemetry ".into()
                    } else {
                        format!(" 遥测 Telemetry · {display_name} ")
                    })
                    .style(Style::default().fg(Color::White).bg(Color::Black)),
            ),
        telem_area,
    );

    // Launchpad：经纬高跟焦点 FocusTelem
    let (lat, lng, pad_alt) = match telem {
        Some(f) => {
            let (la, ln) = lat_lng_from_pos(f);
            (la, ln, f.altitude)
        }
        None => {
            let pad = slice.launchpad.as_ref();
            (
                pad.map(|p| p.lat_deg).unwrap_or(0.0),
                pad.map(|p| p.lng_deg).unwrap_or(0.0),
                pad.map(|p| p.alt_m).unwrap_or(0.0),
            )
        }
    };
    let launched = slice
        .launchpad
        .as_ref()
        .map(|p| p.launched)
        .unwrap_or(slice.launched);
    let launch_status = if launched { "已起飞" } else { "待命" };
    let launch_style = if launched {
        Style::default().fg(Color::Green).bg(Color::Black)
    } else {
        Style::default().fg(Color::Yellow).bg(Color::Black)
    };
    let pad_label = Style::default().fg(Color::Cyan).bold().bg(Color::Black);
    let pad_value = Style::default().fg(Color::White).bg(Color::Black);
    let s_lat = fmt_dms(lat, "N", "S");
    let s_lng = fmt_dms(lng, "E", "W");
    let s_pad_alt = fmt_alt(pad_alt);
    let pad_rows = vec![
        Row::new(vec![
            ratatui::widgets::Cell::from("纬度 Lat").style(pad_label),
            ratatui::widgets::Cell::from(s_lat.as_str()).style(pad_value),
        ]),
        Row::new(vec![
            ratatui::widgets::Cell::from("经度 Lng").style(pad_label),
            ratatui::widgets::Cell::from(s_lng.as_str()).style(pad_value),
        ]),
        Row::new(vec![
            ratatui::widgets::Cell::from("高度 Alt").style(pad_label),
            ratatui::widgets::Cell::from(s_pad_alt.as_str()).style(pad_value),
        ]),
        Row::new(vec![
            ratatui::widgets::Cell::from("状态 Status").style(pad_label),
            ratatui::widgets::Cell::from(launch_status).style(launch_style),
        ]),
    ];
    frame.render_widget(
        Table::new(
            pad_rows,
            [Constraint::Percentage(45), Constraint::Percentage(55)],
        )
        .style(Style::default().fg(Color::White).bg(Color::Black))
        .column_spacing(1)
        .block(
            Block::default()
                .borders(Borders::ALL)
                .title(" 发射场 Launchpad ")
                .style(Style::default().fg(Color::White).bg(Color::Black)),
        ),
        pad_area,
    );

    let [orbit_area, stage_area, attitude_area] = Layout::vertical([
        Constraint::Length(8),
        Constraint::Min(0),
        Constraint::Length(9),
    ])
    .areas(right_area);

    // Orbit（与旧 cli 三分支一致；判据在 Runtime `hud_mode`）
    let mut orbit_lines: Vec<Line> = Vec::new();
    if let Some(o) = slice.orbit.as_ref() {
        match o.hud_mode {
            1 => {
                let ap_km = o.ap_alt / 1e3;
                let pe_km = o.pe_alt / 1e3;
                orbit_lines.push(Line::from(format!(" ApD     {:>8.0} km", ap_km)));
                if pe_km > -1000.0 {
                    orbit_lines.push(Line::from(format!(" PeD     {:>8.0} km", pe_km)));
                } else {
                    orbit_lines.push(Line::from(vec![
                        Span::raw(" PeD     "),
                        Span::styled(
                            format!("{:>8.0} km (亚轨道)", pe_km),
                            Style::default().fg(Color::Yellow),
                        ),
                    ]));
                }
                let t_min = o.period_s / 60.0;
                if t_min > 0.0 && t_min < 1e8 {
                    orbit_lines.push(Line::from(format!(" Period  {:>8.0} min", t_min)));
                }
            }
            2 => {
                orbit_lines.push(Line::from(vec![Span::styled(
                    " (逃逸轨道 escape)",
                    Style::default().fg(Color::Magenta),
                )]));
            }
            _ => {
                orbit_lines.push(Line::from(Span::styled(
                    " (亚轨道 suborbital)",
                    Style::default().fg(Color::Cyan),
                )));
            }
        }
        orbit_lines.push(Line::from(format!(" Energy  {:>8.1} MJ/kg", o.energy_mj_kg)));
    }
    frame.render_widget(
        Paragraph::new(orbit_lines)
            .style(Style::default().fg(Color::White).bg(Color::Black))
            .block(
                Block::default()
                    .borders(Borders::ALL)
                    .title(" 轨道 Orbit ")
                    .style(Style::default().fg(Color::White).bg(Color::Black)),
            ),
        orbit_area,
    );

    // Stages：Primary → active/View；Detached → stages[vessel_index]/View
    let stage_header = Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD);
    let stage_rows: Vec<Row> = slice
        .stages
        .iter()
        .map(|s| {
            let bar = fuel_bar(s.fuel, s.fuel_pct, s.empty_fuel, !s.firing && s.empty_fuel);
            let base = if s.crashed {
                "CRASHED"
            } else if s.detached {
                "DETACHED"
            } else if s.active {
                "ACTIVE"
            } else if s.firing {
                "FIRING"
            } else {
                "attached"
            };
            let is_view = match view_vessel {
                Some(vi) => s.vessel_index as usize == vi,
                None => s.active,
            };
            let status = if is_view {
                format!("{base}/View")
            } else {
                base.to_string()
            };
            let style = if s.crashed {
                Style::default()
                    .fg(Color::Red)
                    .add_modifier(Modifier::BOLD)
            } else if is_view {
                Style::default()
                    .fg(Color::Magenta)
                    .add_modifier(Modifier::BOLD)
            } else if s.active && !s.detached {
                Style::default()
                    .fg(Color::Green)
                    .add_modifier(Modifier::BOLD)
            } else if base == "FIRING" {
                Style::default()
                    .fg(Color::Yellow)
                    .add_modifier(Modifier::BOLD)
            } else if s.detached {
                Style::default().fg(Color::DarkGray).bg(Color::Black)
            } else {
                Style::default().fg(Color::White).bg(Color::Black)
            };
            Row::new(vec![s.name.clone(), bar, status]).style(style)
        })
        .collect();
    frame.render_widget(
        Table::new(
            stage_rows,
            [
                Constraint::Percentage(25),
                Constraint::Percentage(50),
                Constraint::Percentage(25),
            ],
        )
        .header(Row::new(vec!["Stage", "Fuel", "Status"]).style(stage_header))
        .style(Style::default().fg(Color::White).bg(Color::Black))
        .block(
            Block::default()
                .borders(Borders::ALL)
                .title(" 级 Stages ")
                .style(Style::default().fg(Color::White).bg(Color::Black)),
        ),
        stage_area,
    );

    // Attitude：「当前」跟焦点 FocusTelem；「目标」仅 Primary
    let dash = "—";
    let s_pitch = format!(
        "{:.1}°",
        scrub_display_zero(telem.map(|f| f.pitch).unwrap_or(0.0).to_degrees(), 1)
    );
    let s_yaw = format!(
        "{:.1}°",
        scrub_display_zero(telem.map(|f| f.yaw).unwrap_or(0.0).to_degrees(), 1)
    );
    let s_roll = format!(
        "{:.1}°",
        scrub_display_zero(telem.map(|f| f.roll).unwrap_or(0.0).to_degrees(), 1)
    );
    let s_tgt_p = if is_primary {
        format!(
            "{:.1}°",
            scrub_display_zero(slice.pitch_target.to_degrees(), 1)
        )
    } else {
        dash.into()
    };
    let s_tgt_y = if is_primary {
        format!(
            "{:.1}°",
            scrub_display_zero(slice.yaw_target.to_degrees(), 1)
        )
    } else {
        dash.into()
    };
    let s_tgt_r = if is_primary {
        format!(
            "{:.1}°",
            scrub_display_zero(slice.roll_target.to_degrees(), 1)
        )
    } else {
        dash.into()
    };
    let thr_cur = telem.map(|f| f.throttle).unwrap_or(0.0);
    let s_thr_cur = format!("{:.0}%", thr_cur * 100.0);
    let s_thr_tgt = if is_primary {
        format!("{:.0}%", slice.throttle_cmd * 100.0)
    } else {
        dash.into()
    };
    let (ox, oy, oz) = telem
        .map(|f| (f.omega_x, f.omega_y, f.omega_z))
        .unwrap_or((0.0, 0.0, 0.0));
    let s_omega = format!(
        "{:.2} / {:.2} / {:.2} °/s",
        scrub_display_zero(ox.to_degrees(), 2),
        scrub_display_zero(oy.to_degrees(), 2),
        scrub_display_zero(oz.to_degrees(), 2)
    );
    let (gp, gy) = telem
        .map(|f| (f.gimbal_pitch, f.gimbal_yaw))
        .unwrap_or((0.0, 0.0));
    let s_gimbal = format!(
        "{:.2} / {:.2}°",
        scrub_display_zero(gp.to_degrees(), 2),
        scrub_display_zero(gy.to_degrees(), 2)
    );

    let att_label = Style::default().fg(Color::Cyan).bold().bg(Color::Black);
    let att_value = Style::default().fg(Color::White).bg(Color::Black);
    let att_header = Style::default()
        .fg(Color::Cyan)
        .add_modifier(Modifier::BOLD)
        .bg(Color::Black);
    let att_rows = vec![
        Row::new(vec![
            ratatui::widgets::Cell::from("节流阀 Thr").style(att_label),
            ratatui::widgets::Cell::from(s_thr_cur.as_str()).style(att_value),
            ratatui::widgets::Cell::from(s_thr_tgt.as_str()).style(att_value),
        ]),
        Row::new(vec![
            ratatui::widgets::Cell::from("俯仰 Pitch").style(att_label),
            ratatui::widgets::Cell::from(s_pitch.as_str()).style(att_value),
            ratatui::widgets::Cell::from(s_tgt_p.as_str()).style(att_value),
        ]),
        Row::new(vec![
            ratatui::widgets::Cell::from("偏航 Yaw").style(att_label),
            ratatui::widgets::Cell::from(s_yaw.as_str()).style(att_value),
            ratatui::widgets::Cell::from(s_tgt_y.as_str()).style(att_value),
        ]),
        Row::new(vec![
            ratatui::widgets::Cell::from("滚转 Roll").style(att_label),
            ratatui::widgets::Cell::from(s_roll.as_str()).style(att_value),
            ratatui::widgets::Cell::from(s_tgt_r.as_str()).style(att_value),
        ]),
        Row::new(vec![
            ratatui::widgets::Cell::from("角速率 Rate").style(att_label),
            ratatui::widgets::Cell::from(s_omega.as_str()).style(att_value),
            ratatui::widgets::Cell::from("").style(att_value),
        ]),
        Row::new(vec![
            ratatui::widgets::Cell::from("推力矢量 TVC").style(att_label),
            ratatui::widgets::Cell::from(s_gimbal.as_str()).style(att_value),
            ratatui::widgets::Cell::from("").style(att_value),
        ]),
    ];
    frame.render_widget(
        Table::new(
            att_rows,
            [
                Constraint::Length(16),
                Constraint::Percentage(42),
                Constraint::Percentage(42),
            ],
        )
        .header(
            Row::new(vec!["", "当前 Current", "目标 Target"]).style(att_header),
        )
        .style(Style::default().fg(Color::White).bg(Color::Black))
        .column_spacing(1)
        .block(
            Block::default()
                .borders(Borders::ALL)
                .title(if is_primary {
                    " 姿态 Attitude ".into()
                } else {
                    format!(" 姿态 Attitude · {display_name} (只读) ")
                })
                .style(Style::default().fg(Color::White).bg(Color::Black)),
        ),
        attitude_area,
    );

    // Fuel gauge
    let fuel_ratio = (fuel_pct / 100.0).clamp(0.0, 1.0);
    let fuel_color = if fuel_ratio < 0.2 {
        Color::Red
    } else if fuel_ratio < 0.5 {
        Color::Yellow
    } else {
        Color::Green
    };
    frame.render_widget(
        Gauge::default()
            .block(
                Block::default()
                    .borders(Borders::ALL)
                    .title(" 燃料 Fuel ")
                    .style(Style::default().fg(Color::White).bg(Color::Black)),
            )
            .ratio(fuel_ratio)
            .gauge_style(Style::default().fg(fuel_color).bg(Color::Black))
            .style(Style::default().fg(Color::White).bg(Color::Black)),
        fuel_area,
    );

    let key_style = Style::default().fg(Color::Cyan).bold().bg(Color::Black);
    let desc_style = Style::default().fg(Color::White).bg(Color::Black);
    let muted = Style::default().fg(Color::DarkGray).bg(Color::Black);
    let help_text = if is_primary {
        Line::from(vec![
            Span::styled(" W", key_style),
            Span::styled(" 推力  ", desc_style),
            Span::styled("S", key_style),
            Span::styled(" 分离  ", desc_style),
            Span::styled("↑↓", key_style),
            Span::styled(" 节流阀  ", desc_style),
            Span::styled("←→", key_style),
            Span::styled(" 俯仰  ", desc_style),
            Span::styled("G", key_style),
            Span::styled(" 重力转向  ", desc_style),
            Span::styled("C", key_style),
            Span::styled(" 观察  ", desc_style),
            Span::styled("Space", key_style),
            Span::styled(" 暂停  ", desc_style),
            Span::styled("+/-", key_style),
            Span::styled(" 加速  ", desc_style),
            Span::styled("R", key_style),
            Span::styled(" 重置  ", desc_style),
            Span::styled("Q", Style::default().fg(Color::Red).bold().bg(Color::Black)),
            Span::styled(" 退出", desc_style),
        ])
    } else {
        Line::from(vec![
            Span::styled(" 控制已锁定  ", muted),
            Span::styled("C", key_style),
            Span::styled(" 切换/回主组合体  ", desc_style),
            Span::styled("Space", key_style),
            Span::styled(" 暂停  ", desc_style),
            Span::styled("+/-", key_style),
            Span::styled(" 加速  ", desc_style),
            Span::styled("R", key_style),
            Span::styled(" 重置  ", desc_style),
            Span::styled("Q", Style::default().fg(Color::Red).bold().bg(Color::Black)),
            Span::styled(" 退出", desc_style),
        ])
    };
    frame.render_widget(
        Paragraph::new(help_text).block(
            Block::default()
                .borders(Borders::ALL)
                .title(" 控制 Controls ")
                .style(Style::default().fg(Color::White).bg(Color::Black)),
        ),
        help_area,
    );

    if !slice.crash_msg.is_empty() {
        let invert = Style::default().bg(Color::Red).fg(Color::White);
        let invert_bold = Style::default().bg(Color::Red).fg(Color::White).bold();
        let dialog = Paragraph::new(vec![
            Line::raw(""),
            Line::from(Span::styled("!!! 坠毁 CRASH !!!", invert_bold)),
            Line::raw(""),
            Line::from(Span::styled(slice.crash_msg.as_str(), invert)),
            Line::raw(""),
            Line::from(Span::styled("按 R 重置  /  Press R to reset", invert)),
        ])
        .alignment(ratatui::layout::Alignment::Center)
        .block(
            Block::default()
                .borders(Borders::ALL)
                .title(Span::styled(" !!! ", invert_bold))
                .style(invert),
        );
        let area = frame.area();
        let dialog_area =
            ratatui::layout::Rect::new(area.width / 2 - 25, area.height / 2 - 5, 50, 10);
        frame.render_widget(dialog, dialog_area);
    }
}
