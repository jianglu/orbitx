//! orbitx-demo-aero：气动再入演示（ratatui TUI）。
//!
//! 左侧：有气动阻力的再入轨迹；右侧：无大气的参考轨迹。
//! 操作：Space=暂停  R=重置  Q/Esc=退出

use std::time::{Duration, Instant};
use crossterm::event::{self, Event, KeyCode, KeyEventKind};
use orbitx_dynamics::GravBody;
use orbitx_math::{StateVectors, Vec3};
use orbitx_vessel::{Assembly, DragElement, ExponentialAtmosphere, StageSpec};
use ratatui::layout::{Constraint, Layout};
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Span};
use ratatui::widgets::{Block, Borders, Paragraph, Row, Table};
use ratatui::DefaultTerminal;

const EARTH_R: f64 = 6_371_000.0;
const DT: f64 = 0.05;

fn leak_str(s: &str) -> &'static str { Box::leak(s.to_string().into_boxed_str()) }

fn capsule_spec() -> StageSpec {
    StageSpec {
        name: leak_str("Capsule"), dry_mass: 5000.0, fuel_mass: 1000.0,
        thrust: 0.0, isp: 0.0, engine_dir: Vec3::new(0.0, 1.0, 0.0),
        engine_pos: Vec3::ZERO, length: 3.0, radius: 1.5,
        separation_impulse: 0.0, pmi: orbitx_vessel::stage::PMI_UNDEF,
        max_gimbal: 0.0, max_gimbal_rate: 0.0, gimbal_axis: Vec3::new(1.0, 0.0, 0.0),
    }
}

/// 初始状态：80 km 高度，7500 m/s 水平速度，-50 m/s 垂直速度。
fn initial_state() -> StateVectors {
    StateVectors {
        pos: Vec3::new(0.0, 0.0, EARTH_R + 80_000.0),
        vel: Vec3::new(7500.0, 0.0, -50.0),
        ..Default::default()
    }
}

fn make_aero_assembly() -> Assembly {
    let spec = capsule_spec();
    let mut asm = Assembly::new(&[spec], initial_state());
    let v = &mut asm.vessels[0];
    v.dragels.push(DragElement { ref_pos: Vec3::ZERO, cd: 1.5, area: 10.0 });
    v.cross_section = Vec3::new(1.0, 10.0, 1.0);
    v.rdrag = Vec3::new(1.0, 0.1, 1.0);
    asm.atmosphere = Some(Box::new(ExponentialAtmosphere::earth()));
    asm.planet_radius = EARTH_R;
    asm
}

fn make_vacuum_assembly() -> Assembly {
    Assembly::new(&[capsule_spec()], initial_state())
}

struct Telemetry { altitude: f64, speed: f64, dyn_pressure: f64, air_density: f64, drag_force: f64, mach: f64 }

impl Telemetry {
    fn from_asm(asm: &Assembly) -> Self {
        let state = asm.vessels[asm.active].state;
        let alt = state.pos.length() - EARTH_R;
        let speed = state.vel.length();
        let rho = asm.atmosphere.as_ref().map(|atm| atm.density(alt)).unwrap_or(0.0);
        let dynp = 0.5 * rho * speed * speed;
        Telemetry {
            altitude: alt, speed, dyn_pressure: dynp, air_density: rho,
            drag_force: if rho > 1e-15 && speed > 1e-3 { 1.5 * dynp * 10.0 } else { 0.0 },
            mach: speed / 340.0,
        }
    }
}

struct App {
    aero_asm: Assembly, vacuum_asm: Assembly, met: f64,
    paused: bool, exit: bool, aero_done: bool, vacuum_done: bool, last_tick: Instant,
}

impl App {
    fn new() -> Self {
        App {
            aero_asm: make_aero_assembly(), vacuum_asm: make_vacuum_assembly(),
            met: 0.0, paused: false, exit: false, aero_done: false, vacuum_done: false,
            last_tick: Instant::now(),
        }
    }

    fn tick(&mut self) {
        if self.paused { return; }
        let earth = GravBody { pos: Vec3::ZERO, mass: 5.972e24, size: EARTH_R, jcoeff: vec![], rotation: None, pines: None };
        let grav = vec![earth];
        if !self.aero_done {
            self.aero_asm.step(DT, &grav);
            let alt = self.aero_asm.vessels[self.aero_asm.active].state.pos.length() - EARTH_R;
            if alt < 0.0 || alt > 200_000.0 { self.aero_done = true; }
        }
        if !self.vacuum_done {
            self.vacuum_asm.step(DT, &grav);
            let alt = self.vacuum_asm.vessels[self.vacuum_asm.active].state.pos.length() - EARTH_R;
            if alt < 0.0 || alt > 200_000.0 { self.vacuum_done = true; }
        }
        self.met += DT;
        if self.aero_done && self.vacuum_done && !self.paused { self.paused = true; }
    }

    fn reset(&mut self) {
        self.aero_asm = make_aero_assembly();
        self.vacuum_asm = make_vacuum_assembly();
        self.met = 0.0; self.paused = false; self.aero_done = false; self.vacuum_done = false;
    }

    fn handle_key(&mut self, key: KeyCode) {
        match key {
            KeyCode::Char('q') | KeyCode::Esc => self.exit = true,
            KeyCode::Char(' ') => self.paused = !self.paused,
            KeyCode::Char('r') => self.reset(),
            _ => {}
        }
    }

    fn run(&mut self, terminal: &mut DefaultTerminal) -> std::io::Result<()> {
        self.last_tick = Instant::now();
        while !self.exit {
            terminal.draw(|frame| self.draw(frame))?;
            if event::poll(Duration::from_millis(50))? {
                if let Event::Key(key) = event::read()? {
                    if key.kind == KeyEventKind::Press { self.handle_key(key.code); }
                }
            }
            self.tick();
        }
        Ok(())
    }

    fn draw_telem_column(
        &self, frame: &mut ratatui::Frame, area: ratatui::layout::Rect,
        title: &str, telem: &Telemetry, elapsed: f64, done: bool, is_aero: bool,
    ) {
        let lbl = Style::default().fg(Color::Cyan).bold();
        let val = Style::default().fg(Color::White);
        let dng = Style::default().fg(Color::Red).bold();
        let wrn = Style::default().fg(Color::Yellow).bold();
        let alt_s = if telem.altitude < 0.0 { dng } else if telem.altitude < 10_000.0 { wrn } else { val };
        let mach_s = if telem.mach > 25.0 { dng } else if telem.mach > 10.0 { wrn } else { val };
        let q_s = if telem.dyn_pressure > 10_000.0 { dng } else if telem.dyn_pressure > 1_000.0 { wrn } else { val };
        use ratatui::widgets::Cell;
        let rows = vec![
            Row::new(vec![Cell::from("高度 Alt").style(lbl), Cell::from(format!("{:.2} km", telem.altitude / 1000.0)).style(alt_s)]),
            Row::new(vec![Cell::from("速度 Speed").style(lbl), Cell::from(format!("{:.0} m/s", telem.speed)).style(val)]),
            Row::new(vec![Cell::from("动压 Q").style(lbl), Cell::from(format!("{:.2} kPa", telem.dyn_pressure / 1000.0)).style(q_s)]),
            Row::new(vec![Cell::from("密度 Rho").style(lbl), Cell::from(format!("{:.6e}", telem.air_density)).style(val)]),
            Row::new(vec![Cell::from("阻力 Drag").style(lbl), Cell::from(format!("{:.0} N", telem.drag_force)).style(val)]),
            Row::new(vec![Cell::from("马赫 Mach").style(lbl), Cell::from(format!("{:.1}", telem.mach)).style(mach_s)]),
            Row::new(vec![Cell::from("时间 T+").style(lbl), Cell::from(fmt_time(elapsed)).style(val)]),
        ];
        let bc = if done { Color::DarkGray } else if is_aero { Color::Green } else { Color::Yellow };
        let table = Table::new(rows, [Constraint::Percentage(40), Constraint::Percentage(60)])
            .style(Style::default().fg(Color::White).bg(Color::Black)).column_spacing(1)
            .block(Block::default().borders(Borders::ALL)
                .title(Span::styled(format!(" {} ", title), Style::default().fg(bc).add_modifier(Modifier::BOLD)))
                .style(Style::default().fg(bc).bg(Color::Black)));
        frame.render_widget(table, area);
    }

    fn draw(&self, frame: &mut ratatui::Frame) {
        let at = Telemetry::from_asm(&self.aero_asm);
        let vt = Telemetry::from_asm(&self.vacuum_asm);
        let [title_area, main_area, help_area] = Layout::vertical([
            Constraint::Length(3), Constraint::Min(0), Constraint::Length(3),
        ]).areas(frame.area());

        // 标题栏
        let status = if self.paused { " [暂停]" } else { "" };
        let title_text = format!(" orbitx 气动再入演示 — 80 km / 7500 m/s  T+{}{} ", fmt_time(self.met), status);
        frame.render_widget(
            Block::default().borders(Borders::ALL)
                .title(Span::styled(title_text, Style::default().fg(Color::White).add_modifier(Modifier::BOLD).bg(Color::Black)))
                .style(Style::default().fg(Color::White).bg(Color::Black)),
            title_area,
        );

        // 左右两列
        let [left, right] = Layout::horizontal([Constraint::Percentage(50), Constraint::Percentage(50)]).areas(main_area);
        self.draw_telem_column(frame, left, "气动再入 Aero Reentry", &at, self.met, self.aero_done, true);
        self.draw_telem_column(frame, right, "真空参考 Vacuum Ref", &vt, self.met, self.vacuum_done, false);

        // 底部：控制 + 结论
        let ks = Style::default().fg(Color::Cyan).bold().bg(Color::Black);
        let ds = Style::default().fg(Color::White).bg(Color::Black);
        let dv = vt.speed - at.speed;
        let conclusion = if dv > 100.0 {
            format!("气动减速：真空比有大气快 {:.0} m/s — 阻力显著减速!", dv)
        } else if self.met > 10.0 {
            "气动阻力正在累积减速效果...".to_string()
        } else {
            "再入初期，气动效应逐渐增强...".to_string()
        };
        let help = Paragraph::new(Line::from(vec![
            Span::styled("Space", ks), Span::styled(" 暂停  ", ds),
            Span::styled("R", ks), Span::styled(" 重置  ", ds),
            Span::styled("Q", Style::default().fg(Color::Red).bold().bg(Color::Black)), Span::styled(" 退出  ", ds),
            Span::styled("  |  ", ds),
            Span::styled(conclusion, Style::default().fg(Color::Green).bold().bg(Color::Black)),
        ])).block(Block::default().borders(Borders::ALL).title(" 控制 Controls ").style(Style::default().fg(Color::White).bg(Color::Black)));
        frame.render_widget(help, help_area);
    }
}

fn fmt_time(secs: f64) -> String {
    let t = secs.max(0.0) as u64;
    format!("{:02}:{:02}:{:02}", t / 3600, (t % 3600) / 60, t % 60)
}

fn main() -> std::io::Result<()> {
    let mut app = App::new();
    ratatui::run(|terminal| app.run(terminal))
}
