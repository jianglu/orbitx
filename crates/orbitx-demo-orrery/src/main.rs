//! orbitx-demo-orrery：多体太阳系演示。
//!
//! 终端 UI 显示太阳系天体配置信息：
//! - 各天体名称、质量、半径、自转周期
//! - 重力模型类型（J-coeff / Pines / PointMass）
//! - 大气参数
//! - 父子关系树
//!
//! 用法：cargo run -p orbitx-demo-orrery

use std::io;
use std::time::{Duration, Instant};

use crossterm::event::{self, Event, KeyCode};
use orbitx_config::{BodyConfig, SystemConfig};
use ratatui::layout::{Constraint, Layout, Rect};
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Span};
use ratatui::widgets::{Block, Borders, Paragraph, Row, Table};
use ratatui::DefaultTerminal;

fn main() -> io::Result<()> {
    let terminal = ratatui::init();
    let result = run(terminal);
    ratatui::restore();
    result
}

fn run(mut terminal: DefaultTerminal) -> io::Result<()> {
    let sol = SystemConfig::sol();
    let mut scroll_offset = 0u16;
    let mut last_tick = Instant::now();
    let tick_rate = Duration::from_millis(100);

    loop {
        terminal.draw(|frame| {
            let chunks = Layout::vertical([
                Constraint::Length(3),  // Title
                Constraint::Min(5),    // Body table
                Constraint::Length(3), // Footer
            ]).split(frame.area());

            // Title bar.
            let title = Line::from(vec![
                Span::styled(" ☀ ", Style::default().fg(Color::Yellow).add_modifier(Modifier::BOLD)),
                Span::styled("orbitx-demo-orrery", Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD)),
                Span::raw("  —  太阳系天体配置  "),
                Span::styled(format!("{} 个天体", sol.bodies.len()), Style::default().fg(Color::Green)),
            ]);
            frame.render_widget(
                Paragraph::new(title).style(Style::default().bg(Color::DarkGray)),
                chunks[0],
            );

            // Body table.
            render_body_table(frame, chunks[1], &sol, scroll_offset);

            // Footer.
            let footer = Line::from(vec![
                Span::styled(" ↑/↓ ", Style::default().fg(Color::Yellow)),
                Span::raw("滚动  "),
                Span::styled(" Q ", Style::default().fg(Color::Yellow)),
                Span::raw("退出"),
            ]);
            frame.render_widget(
                Paragraph::new(footer).style(Style::default().bg(Color::DarkGray)),
                chunks[2],
            );
        })?;

        // Event handling.
        let timeout = tick_rate.checked_sub(last_tick.elapsed()).unwrap_or(Duration::ZERO);
        if event::poll(timeout)? {
            if let Event::Key(key) = event::read()? {
                match key.code {
                    KeyCode::Char('q') | KeyCode::Esc => return Ok(()),
                    KeyCode::Down => {
                        scroll_offset = scroll_offset.saturating_add(1);
                    }
                    KeyCode::Up => {
                        scroll_offset = scroll_offset.saturating_sub(1);
                    }
                    KeyCode::Char(' ') => {
                        scroll_offset = scroll_offset.saturating_add(10);
                    }
                    _ => {}
                }
            }
        }

        if last_tick.elapsed() >= tick_rate {
            last_tick = Instant::now();
        }
    }
}

fn render_body_table(
    frame: &mut ratatui::Frame,
    area: Rect,
    sol: &SystemConfig,
    _scroll_offset: u16,
) {
    let header = Row::new(vec![
        Cell::from("名称").style(Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD)),
        Cell::from("质量 [kg]").style(Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD)),
        Cell::from("半径 [km]").style(Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD)),
        Cell::from("重力").style(Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD)),
        Cell::from("自转 [h]").style(Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD)),
        Cell::from("大气").style(Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD)),
        Cell::from("父天体").style(Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD)),
    ]);

    let rows: Vec<Row> = sol
        .bodies
        .iter()
        .map(|b| {
            let gravity_str = match &b.gravity {
                Some(orbitx_config::GravityConfig::Jcoeff { values }) => {
                    format!("J({})", values.len())
                }
                Some(orbitx_config::GravityConfig::Pines {
                    model_path,
                    cutoff,
                }) => format!("Pines({},{})", model_path.split('.').next().unwrap_or("?"), cutoff),
                None => "点质量".to_string(),
            };

            let rot_str = match &b.rotation {
                Some(r) => format!("{:.1}", r.sid_rot_period / 3600.0),
                None => "—".to_string(),
            };

            let atm_str = match &b.atmosphere {
                Some(a) => format!("ρ₀={:.3}", a.density0),
                None => "—".to_string(),
            };

            let parent_str = sol
                .parent_name(&b.name)
                .map(|p| p.to_string())
                .unwrap_or_else(|| "—".to_string());

            let mass_str = format_mass(b.mass);
            let radius_str = format!("{:.0}", b.size / 1000.0);

            Row::new(vec![
                Cell::from(b.name.as_str()).style(name_style(&b.name)),
                Cell::from(mass_str),
                Cell::from(radius_str),
                Cell::from(gravity_str),
                Cell::from(rot_str),
                Cell::from(atm_str),
                Cell::from(parent_str),
            ])
        })
        .collect();

    let table = Table::new(rows, [
        Constraint::Length(10),  // 名称
        Constraint::Length(14),  // 质量
        Constraint::Length(10),  // 半径
        Constraint::Length(16),  // 重力
        Constraint::Length(10),  // 自转
        Constraint::Length(12),  // 大气
        Constraint::Length(10),  // 父天体
    ])
    .header(header)
    .block(Block::default().borders(Borders::ALL).title(" 天体列表 "));

    frame.render_widget(table, area);
}

use ratatui::widgets::Cell;

fn format_mass(mass: f64) -> String {
    if mass >= 1e27 {
        format!("{:.3}e27", mass / 1e27)
    } else if mass >= 1e24 {
        format!("{:.3}e24", mass / 1e24)
    } else if mass >= 1e23 {
        format!("{:.3}e23", mass / 1e23)
    } else if mass >= 1e22 {
        format!("{:.3}e22", mass / 1e22)
    } else {
        format!("{:.2e}", mass)
    }
}

fn name_style(name: &str) -> Style {
    match name {
        "Sun" => Style::default().fg(Color::Yellow).add_modifier(Modifier::BOLD),
        "Earth" => Style::default().fg(Color::Blue).add_modifier(Modifier::BOLD),
        "Moon" => Style::default().fg(Color::Gray),
        "Jupiter" => Style::default().fg(Color::Rgb(255, 200, 100)),
        "Saturn" => Style::default().fg(Color::Rgb(255, 230, 150)),
        "Mars" => Style::default().fg(Color::Red),
        _ => Style::default(),
    }
}
