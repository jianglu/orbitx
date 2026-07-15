//! orbitx-cli：终端多级火箭发射模拟器（ratatui TUI）。
//!
//! 用法：
//!   cargo run -p orbitx-cli              # Falcon 9
//!   cargo run -p orbitx-cli -- saturnv   # Saturn V
//!
//! 操作：
//!   W（按住）   推力开关
//!   S          分离当前级
//!   ↑/↓        油门增/减
//!   ←/→        俯仰角增/减
//!   G          切换自动重力转向
//!   Space      暂停/继续
//!   +/-        时间加速/减速
//!   R          重置
//!   Q/Esc      退出

use std::time::{Duration, Instant};

use crossterm::event::{self, Event, KeyCode, KeyEventKind, KeyModifiers};
use orbitx_config::RocketConfig;
use orbitx_dynamics::{Elements, GravBody};
use orbitx_math::{cross, dot, mul, StateVectors, Vec3};
use orbitx_vessel::{Assembly, StageSpec};
use ratatui::layout::{Constraint, Layout};
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Span};
use ratatui::widgets::{Block, Borders, Gauge, Paragraph, Row, Table};
use ratatui::DefaultTerminal;

const EARTH_R: f64 = 6_371_000.0;
const EARTH_GM: f64 = orbitx_math::consts::GGRAV * 5.972e24;
const G0: f64 = 9.80665;
const RHO0: f64 = 1.225;
const SCALE_H: f64 = 8500.0;
const ATM_TOP: f64 = 100_000.0;
const DRAG_COEFF: f64 = 0.005;
const LAUNCH_POS: Vec3 = Vec3::new(0.0, 0.0, EARTH_R);

fn air_density(h: f64) -> f64 {
    if !(0.0..=ATM_TOP).contains(&h) {
        0.0
    } else {
        RHO0 * (-h / SCALE_H).exp()
    }
}

fn fmt_time(secs: f64) -> String {
    let t = secs.max(0.0) as u64;
    format!("T+{:02}:{:02}:{:02}", t / 3600, (t % 3600) / 60, t % 60)
}

/// 将 RocketConfig 转换为 StageSpec 列表。
fn rocket_to_stages(config: &RocketConfig) -> Vec<StageSpec> {
    config
        .stages
        .iter()
        .map(|s| StageSpec {
            name: leak_str(s.name.as_str()),
            dry_mass: s.dry_mass,
            fuel_mass: s.fuel_mass,
            thrust: s.thrust,
            isp: s.isp,
            engine_dir: Vec3::new(s.engine_dir[0], s.engine_dir[1], s.engine_dir[2]),
            engine_pos: Vec3::new(s.engine_pos[0], s.engine_pos[1], s.engine_pos[2]),
            length: s.length,
            radius: s.radius,
            separation_impulse: s.separation_impulse,
        })
        .collect()
}

/// 将 String 泄漏为 &'static str（StageSpec 需要 &'static str）。
fn leak_str(s: &str) -> &'static str {
    Box::leak(s.to_string().into_boxed_str())
}

struct App {
    asm: Assembly,
    rocket_name: String,
    met: f64,
    pitch: f64,
    throttle: f64,
    thrusting: bool,
    auto_gravity_turn: bool,
    paused: bool,
    time_scale: f64,
    exit: bool,
    last_tick: Instant,
    initial_stages: Vec<StageSpec>,
    initial_pos: Vec3,
    crash_msg: String,
}

impl App {
    fn new(stages: &[StageSpec], name: &str) -> Self {
        let half_h: f64 = stages.iter().map(|s| s.length).sum::<f64>() / 2.0;
        let radial = LAUNCH_POS * (1.0 / LAUNCH_POS.length());
        let init_pos = LAUNCH_POS + radial * half_h;
        let init_state = StateVectors {
            pos: init_pos,
            ..Default::default()
        };
        let asm = Assembly::new(stages, init_state);
        App {
            asm,
            rocket_name: name.to_string(),
            met: 0.0,
            pitch: 0.0,
            throttle: 0.0,
            thrusting: false,
            auto_gravity_turn: false,
            paused: false,
            time_scale: 1.0,
            exit: false,
            last_tick: Instant::now(),
            initial_stages: stages.to_vec(),
            initial_pos: init_pos,
            crash_msg: String::new(),
        }
    }

    fn altitude(&self) -> f64 {
        let (pos, _) = self.asm.render_state();
        pos.length() - EARTH_R
    }

    fn velocity(&self) -> Vec3 {
        self.asm.vessels[self.asm.active].state.vel
    }

    fn thrust_dir(&self) -> Vec3 {
        let r = self.asm.vessels[self.asm.active].state.pos;
        let r_mag = r.length();
        if r_mag < 1e-3 {
            return Vec3::new(0.0, 0.0, 1.0);
        }
        let radial = r * (1.0 / r_mag);
        let vel = self.velocity();
        let tangent_base = if vel.length() > 1e-3 {
            let v_radial = radial * dot(vel, radial);
            let v_tan = vel - v_radial;
            if v_tan.length() > 1e-3 {
                v_tan.unit()
            } else {
                cross(radial, Vec3::new(0.0, 1.0, 0.0)).unit()
            }
        } else {
            cross(radial, Vec3::new(0.0, 1.0, 0.0)).unit()
        };
        (radial * self.pitch.cos() + tangent_base * self.pitch.sin()).unit()
    }

    fn tick(&mut self) {
        if self.paused {
            return;
        }
        let now = Instant::now();
        let dt_real = now.duration_since(self.last_tick).as_secs_f64().min(0.1);
        self.last_tick = now;

        let dt = dt_real * self.time_scale;

        // 发射台锁定：无推力 + 低速 + 低空时，固定在发射台上，不应用重力。
        let on_pad = !self.thrusting && self.velocity().length() < 1.0 && self.altitude() < 200.0;
        if on_pad {
            // 将位置固定回初始位置，速度归零。
            self.asm.vessels[self.asm.active].state.pos = self.initial_pos;
            self.asm.vessels[self.asm.active].state.vel = Vec3::ZERO;
            self.last_tick = Instant::now();
            return;
        }

        // 重力转向。
        if self.auto_gravity_turn {
            let h = self.altitude();
            if h > 10_000.0 {
                let target = ((h - 10_000.0) / 70_000.0).min(1.0) * std::f64::consts::FRAC_PI_2;
                if self.pitch < target {
                    self.pitch = (self.pitch + 0.5).min(target);
                }
            }
        }

        // 设置油门。
        let thr = if self.thrusting { self.throttle } else { 0.0 };
        self.asm.set_throttle(thr);

        // 设置推进器方向。
        let td = self.thrust_dir();
        let rot = self.asm.vessels[self.asm.active].state.r;
        let td_body = mul(rot, td);
        for v in &mut self.asm.vessels {
            if !v.detached {
                for t in &mut v.thrusters {
                    t.dir = td_body;
                }
            }
        }

        // 积分。
        let earth = GravBody {
            pos: Vec3::ZERO,
            mass: 5.972e24,
            size: EARTH_R,
            jcoeff: vec![],
        };
        let grav = vec![earth];
        let pos_before = self.asm.vessels[self.asm.active].state.pos;
        let vel_before = self.asm.vessels[self.asm.active].state.vel;
        self.asm.step(dt, &grav);

        // 阻力。
        let h = pos_before.length() - EARTH_R;
        let rho = air_density(h);
        if rho > 1e-10 {
            let v_mag = vel_before.length();
            if v_mag > 1e-3 {
                let total_mass = self.asm.total_mass();
                let drag_mag = 0.5 * rho * v_mag * v_mag * DRAG_COEFF;
                let drag_acc = vel_before * (-drag_mag / (v_mag * total_mass));
                let dv = drag_acc * dt;
                for v in &mut self.asm.vessels {
                    if !v.detached {
                        v.state.vel += dv;
                    }
                }
            }
        }

        self.met += dt;

        // 自动分离。
        if self.asm.stage_count() > 1 {
            let active = &self.asm.vessels[self.asm.active];
            if active.fuel_mass < 1.0 && active.thrusters.iter().any(|t| t.max_thrust > 0.0) {
                self.asm.separate_stage();
            }
        }

        // 碰撞：仅在速度足够大时触发坠毁（避免起飞前微小抖动误判）。
        if self.altitude() < 0.0
            && self.velocity().length() > 50.0
            && self.crash_msg.is_empty()
            && self.met > 1.0
        {
            self.crash_msg = format!(
                "{} 撞击地面，速度 {:.0} m/s",
                self.asm.active_name(),
                self.velocity().length()
            );
            self.paused = true;
        }
    }

    fn reset(&mut self) {
        let init_state = StateVectors {
            pos: self.initial_pos,
            ..Default::default()
        };
        self.asm = Assembly::new(&self.initial_stages, init_state);
        self.met = 0.0;
        self.pitch = 0.0;
        self.throttle = 0.0;
        self.thrusting = false;
        self.crash_msg.clear();
        self.paused = false;
    }

    fn handle_key(&mut self, key: KeyCode, modifiers: KeyModifiers) {
        // 坠毁状态：只接受 R（重置）和 Q（退出）。
        if !self.crash_msg.is_empty() {
            match key {
                KeyCode::Char('r') => self.reset(),
                KeyCode::Char('q') | KeyCode::Esc => self.exit = true,
                KeyCode::Char('c') if modifiers.contains(KeyModifiers::CONTROL) => self.exit = true,
                _ => {}
            }
            return;
        }
        match key {
            KeyCode::Char('q') | KeyCode::Esc => self.exit = true,
            KeyCode::Char('w') => self.thrusting = !self.thrusting,
            KeyCode::Char('s') => {
                if self.asm.stage_count() > 1 {
                    self.asm.separate_stage();
                }
            }
            KeyCode::Up => self.throttle = (self.throttle + 0.1).min(1.0),
            KeyCode::Down => self.throttle = (self.throttle - 0.1).max(0.0),
            KeyCode::Left => self.pitch = (self.pitch - 0.1).max(0.0),
            KeyCode::Right => self.pitch = (self.pitch + 0.1).min(std::f64::consts::FRAC_PI_2),
            KeyCode::Char('g') => self.auto_gravity_turn = !self.auto_gravity_turn,
            KeyCode::Char(' ') => self.paused = !self.paused,
            KeyCode::Char('+') | KeyCode::Char('=') => self.time_scale *= 2.0,
            KeyCode::Char('-') => self.time_scale /= 2.0,
            KeyCode::Char('r') => self.reset(),
            KeyCode::Char('c') if modifiers.contains(KeyModifiers::CONTROL) => self.exit = true,
            _ => {}
        }
    }

    fn run(&mut self, terminal: &mut DefaultTerminal) -> std::io::Result<()> {
        self.last_tick = Instant::now();
        while !self.exit {
            terminal.draw(|frame| self.draw(frame))?;

            // 非阻塞事件轮询。
            let timeout = Duration::from_millis(50);
            if event::poll(timeout)? {
                if let Event::Key(key) = event::read()? {
                    if key.kind == KeyEventKind::Press {
                        self.handle_key(key.code, key.modifiers);
                    }
                }
            }

            self.tick();
        }
        Ok(())
    }

    fn draw(&self, frame: &mut ratatui::Frame) {
        let h = self.altitude().max(0.0);
        let vel = self.velocity();
        let speed = vel.length();
        let r = self.asm.vessels[self.asm.active].state.pos;
        let r_unit = r * (1.0 / r.length().max(1e-3));
        let v_vert = dot(vel, r_unit);
        let v_horiz = (vel - r_unit * v_vert).length();
        let mass = self.asm.total_mass();
        let fuel = self.asm.total_fuel();
        let initial_fuel: f64 = self.initial_stages.iter().map(|s| s.fuel_mass).sum();
        let fuel_pct = if initial_fuel > 0.0 {
            (fuel / initial_fuel * 100.0).clamp(0.0, 100.0)
        } else {
            0.0
        };
        let thrust = self.asm.current_thrust();
        let tw = if mass > 0.0 {
            thrust / (mass * G0)
        } else {
            0.0
        };

        // 标题 + 底部 = 3行各
        let [title_area, main_area, fuel_area, help_area] = Layout::vertical([
            Constraint::Length(3),
            Constraint::Min(0),
            Constraint::Length(3),
            Constraint::Length(3),
        ])
        .areas(frame.area());

        // 左右分栏：遥测 1/3，其余 2/3。
        let [left_area, right_area] =
            Layout::horizontal([Constraint::Percentage(33), Constraint::Percentage(67)])
                .areas(main_area);

        // === 标题栏 ===
        let title = format!(
            " orbitx 发射模拟器 — {}  {}  Stage: {}  (剩余 {} 级) ",
            self.rocket_name,
            fmt_time(self.met),
            self.asm.active_name(),
            self.asm.stage_count()
        );
        let status_tags = if self.paused {
            " [暂停]"
        } else if self.thrusting {
            " [推力]"
        } else {
            ""
        };
        let gravity_tag = if self.auto_gravity_turn {
            " [重力转向]"
        } else {
            ""
        };
        let warp_tag = if self.time_scale > 1.5 {
            format!(" [{:.0}x]", self.time_scale)
        } else {
            String::new()
        };
        let title_line = Line::from(vec![
            Span::styled(title.clone(), Style::default().add_modifier(Modifier::BOLD)),
            Span::raw(status_tags),
            Span::styled(gravity_tag, Style::default().fg(Color::Yellow)),
            Span::styled(warp_tag, Style::default().fg(Color::Cyan)),
            Span::styled(
                if !self.crash_msg.is_empty() {
                    format!(" !!! {} !!!", self.crash_msg)
                } else {
                    String::new()
                },
                Style::default().fg(Color::Red).bold(),
            ),
        ]);
        let title_block = Block::default().borders(Borders::ALL).title(title_line);
        frame.render_widget(title_block, title_area);

        // === 左侧：遥测表格 ===
        let s_alt = fmt_dist(h);
        let s_vel = format!("{:.0} m/s", speed);
        let s_vvert = format!("{:.0} m/s", v_vert);
        let s_vhoriz = format!("{:.0} m/s", v_horiz);
        let s_mass = fmt_mass(mass);
        let s_fuel = format!("{:.0} kg", fuel);
        let s_thrust = format!("{:.0} kN", thrust / 1000.0);
        let s_tw = format!("{:.2}", tw);
        let s_thr = format!(
            "{:.0}%",
            if self.thrusting {
                self.throttle * 100.0
            } else {
                0.0
            }
        );
        let s_pitch = format!("{:.1}°", self.pitch.to_degrees());

        // 危险状态高亮颜色。
        let danger = Style::default().fg(Color::Red).bold();
        let warning = Style::default().fg(Color::Yellow);
        let normal = Style::default();

        // 高度负值 = 地下（危险）。
        let alt_style = if self.altitude() < 0.0 {
            danger
        } else {
            normal
        };
        // T/W < 1 = 推力不足（警告）。
        let tw_style = if tw < 1.0 && self.thrusting {
            warning
        } else {
            normal
        };
        // 燃料 < 20%（警告）或 0%（危险）。
        let fuel_style = if fuel < 1.0 {
            danger
        } else if fuel_pct < 20.0 {
            warning
        } else {
            normal
        };
        let fuel_cell = ratatui::widgets::Cell::from(s_fuel.as_str()).style(fuel_style);

        let rows = vec![
            Row::new(["高度 Alt", s_alt.as_str()]).style(alt_style),
            Row::new(["速度 Vel", s_vel.as_str()]),
            Row::new(["垂直 Vvert", s_vvert.as_str()]),
            Row::new(["水平 Vhoriz", s_vhoriz.as_str()]),
            Row::new(["质量 Mass", s_mass.as_str()]),
            Row::new(vec!["燃料 Fuel".into(), fuel_cell]),
            Row::new(["推力 Thrust", s_thrust.as_str()]),
            Row::new(vec![
                "推重比 T/W".into(),
                ratatui::widgets::Cell::from(s_tw.as_str()).style(tw_style),
            ]),
            Row::new(["油门 Thr", s_thr.as_str()]),
            Row::new(["俯仰 Pitch", s_pitch.as_str()]),
        ];
        let telemetry = Table::new(
            rows,
            [Constraint::Percentage(45), Constraint::Percentage(55)],
        )
        .block(
            Block::default()
                .borders(Borders::ALL)
                .title(" 遥测 Telemetry "),
        )
        .style(Style::default().fg(Color::White));
        frame.render_widget(telemetry, left_area);

        // === 右侧：轨道参数 + 级状态 ===
        let [orbit_area, stage_area] =
            Layout::vertical([Constraint::Length(8), Constraint::Min(0)]).areas(right_area);

        // 轨道参数。
        let r_mag = r.length();
        let v_circular = (EARTH_GM / r_mag).sqrt();
        let energy = speed * speed / 2.0 - EARTH_GM / r_mag;
        let energy_margin = EARTH_GM / r_mag * 0.01;

        let mut orbit_lines: Vec<Line> = Vec::new();
        if v_horiz > v_circular * 0.5 && energy < -energy_margin {
            let el = Elements::calculate(r, vel, EARTH_GM, 0.0);
            let ap = (el.ap_dist() - EARTH_R) / 1e3;
            let pe = (el.pe_dist() - EARTH_R) / 1e3;
            orbit_lines.push(Line::from(format!(" ApD     {:>8.0} km", ap)));
            if pe > -1000.0 {
                orbit_lines.push(Line::from(format!(" PeD     {:>8.0} km", pe)));
            } else {
                orbit_lines.push(Line::from(vec![
                    Span::raw(" PeD     "),
                    Span::styled(
                        format!("{:>8.0} km (亚轨道)", pe),
                        Style::default().fg(Color::Yellow),
                    ),
                ]));
            }
            let t_min = el.orbit_t() / 60.0;
            if t_min > 0.0 && t_min < 1e8 {
                orbit_lines.push(Line::from(format!(" Period  {:>8.0} min", t_min)));
            }
        } else if energy > energy_margin && speed > 100.0 {
            orbit_lines.push(Line::from(vec![Span::styled(
                " (逃逸轨道 escape)",
                Style::default().fg(Color::Magenta),
            )]));
        } else {
            orbit_lines.push(Line::from(Span::styled(
                " (亚轨道 suborbital)",
                Style::default().fg(Color::DarkGray),
            )));
        }
        orbit_lines.push(Line::from(format!(" Energy  {:>8.1} MJ/kg", energy / 1e6)));
        let orbit_text = Paragraph::new(orbit_lines)
            .block(Block::default().borders(Borders::ALL).title(" 轨道 Orbit "));
        frame.render_widget(orbit_text, orbit_area);

        // 级状态。
        let stage_rows: Vec<Row> = self
            .asm
            .vessels
            .iter()
            .enumerate()
            .map(|(i, v)| {
                let fuel_bar = if v.thrusters.is_empty() || v.fuel_mass == 0.0 {
                    "—".to_string()
                } else {
                    let init_fuel = self
                        .initial_stages
                        .get(i)
                        .map(|s| s.fuel_mass)
                        .unwrap_or(1.0);
                    let pct = if init_fuel > 0.0 {
                        (v.fuel_mass / init_fuel * 10.0) as usize
                    } else {
                        0
                    };
                    format!(
                        "[{}{}] {:.0} kg",
                        "#".repeat(pct),
                        ".".repeat(10 - pct.min(10)),
                        v.fuel_mass
                    )
                };
                let status = if v.detached {
                    "DETACHED"
                } else if i == self.asm.active {
                    "ACTIVE"
                } else {
                    "attached"
                };
                let style = if i == self.asm.active && !v.detached {
                    Style::default()
                        .fg(Color::Green)
                        .add_modifier(Modifier::BOLD)
                } else if v.detached {
                    Style::default().fg(Color::DarkGray)
                } else {
                    Style::default()
                };
                Row::new(vec![v.name.clone(), fuel_bar, status.to_string()]).style(style)
            })
            .collect();
        let stage_table = Table::new(
            stage_rows,
            [
                Constraint::Percentage(25),
                Constraint::Percentage(50),
                Constraint::Percentage(25),
            ],
        )
        .header(
            Row::new(vec!["Stage", "Fuel", "Status"])
                .style(Style::default().add_modifier(Modifier::BOLD)),
        )
        .column_spacing(1)
        .block(
            Block::default()
                .borders(Borders::ALL)
                .title(" 级状态 Stages "),
        );
        frame.render_widget(stage_table, stage_area);

        // === 底部：燃料条 + 快捷键 ===
        let fuel_ratio = (fuel_pct / 100.0).clamp(0.0, 1.0);
        let fuel_gauge = Gauge::default()
            .block(Block::default().borders(Borders::ALL).title(" 燃料 Fuel "))
            .ratio(fuel_ratio)
            .style(Style::default().fg(if fuel_ratio < 0.2 {
                Color::Red
            } else if fuel_ratio < 0.5 {
                Color::Yellow
            } else {
                Color::Green
            }));
        frame.render_widget(fuel_gauge, fuel_area);

        let help_text = Line::from(vec![
            Span::styled(" W", Style::default().fg(Color::Cyan).bold()),
            Span::raw(" 推力  "),
            Span::styled("S", Style::default().fg(Color::Cyan).bold()),
            Span::raw(" 分离  "),
            Span::styled("↑↓", Style::default().fg(Color::Cyan).bold()),
            Span::raw(" 油门  "),
            Span::styled("←→", Style::default().fg(Color::Cyan).bold()),
            Span::raw(" 俯仰  "),
            Span::styled("G", Style::default().fg(Color::Cyan).bold()),
            Span::raw(" 重力转向  "),
            Span::styled("Space", Style::default().fg(Color::Cyan).bold()),
            Span::raw(" 暂停  "),
            Span::styled("+/-", Style::default().fg(Color::Cyan).bold()),
            Span::raw(" 加速  "),
            Span::styled("R", Style::default().fg(Color::Cyan).bold()),
            Span::raw(" 重置  "),
            Span::styled("Q", Style::default().fg(Color::Red).bold()),
            Span::raw(" 退出"),
        ]);
        let help = Paragraph::new(help_text).block(
            Block::default()
                .borders(Borders::ALL)
                .title(" 控制 Controls "),
        );
        frame.render_widget(help, help_area);

        // === 坠毁对话框（覆盖层） ===
        if !self.crash_msg.is_empty() {
            let dialog = Paragraph::new(vec![
                Line::raw(""),
                Line::from(Span::styled(
                    "!!! 坠毁 CRASH !!!",
                    Style::default().fg(Color::Red).bold(),
                )),
                Line::raw(""),
                Line::from(self.crash_msg.as_str()),
                Line::raw(""),
                Line::from(Span::styled(
                    "按 R 重置  /  Press R to reset",
                    Style::default().fg(Color::Yellow),
                )),
            ])
            .alignment(ratatui::layout::Alignment::Center)
            .block(
                Block::default()
                    .borders(Borders::ALL)
                    .title(" !!! ")
                    .style(Style::default().fg(Color::Red)),
            );

            let area = frame.area();
            let dialog_area =
                ratatui::layout::Rect::new(area.width / 2 - 25, area.height / 2 - 5, 50, 10);
            frame.render_widget(dialog, dialog_area);
        }
    }
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

fn main() -> std::io::Result<()> {
    let args: Vec<String> = std::env::args().collect();
    let toml_str = if args.len() > 1 && args[1] == "saturnv" {
        include_str!("../../orbitx-config/presets/saturn_v.toml")
    } else {
        include_str!("../../orbitx-config/presets/falcon9.toml")
    };

    let config = RocketConfig::from_toml_str(toml_str).expect("解析火箭配置失败");
    let stages = rocket_to_stages(&config);

    let mut app = App::new(&stages, &config.name);

    ratatui::run(|terminal| app.run(terminal))
}
