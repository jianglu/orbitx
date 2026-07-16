//! orbitx-demo-landing：着陆接触力演示（ratatui TUI）。
//!
//! 单级着陆器从 500 m 高度以 -2 m/s 垂直速度缓降，
//! 三点着陆架弹簧-阻尼-摩擦模型在触地时产生接触力，使着陆器减速并停稳。
//!
//! 操作：
//!   Space    暂停/继续
//!   Q/Esc    退出
//!   R        重置（软着陆）
//!   H        硬着陆（-20 m/s 下沉）

use std::time::{Duration, Instant};

use crossterm::event::{self, Event, KeyCode, KeyEventKind};
use orbitx_dynamics::GravBody;
use orbitx_math::{dot, mul, Matrix3, Quat, StateVectors, Vec3};
use orbitx_vessel::{
    compute_surface_forces, make_landing_gear, Assembly, StageSpec, SurfaceContact,
};
use ratatui::layout::{Constraint, Layout};
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Span};
use ratatui::widgets::{Block, Borders, Paragraph, Row, Table};
use ratatui::DefaultTerminal;

const EARTH_R: f64 = 6_371_000.0;
const DT: f64 = 0.01; // 小步长保证接触稳定性

/// 初始高度 [m]。
const INIT_ALT: f64 = 500.0;
/// 软着陆初始下沉速度 [m/s]。
const SOFT_VERT: f64 = -2.0;
/// 硬着陆初始下沉速度 [m/s]。
const HARD_VERT: f64 = -20.0;

/// 着陆器参数。
const DRY_MASS: f64 = 2000.0;
const FUEL_MASS: f64 = 500.0;
const STAGE_LENGTH: f64 = 10.0;
const STAGE_RADIUS: f64 = 2.0;

/// 着陆架参数：radius=2, y_offset=-5, stiffness=5e5, damping=1e4, mu=0.5。
const GEAR_RADIUS: f64 = 2.0;
const GEAR_Y: f64 = -5.0;
const GEAR_STIFFNESS: f64 = 5e5;
const GEAR_DAMPING: f64 = 1e4;
const GEAR_MU: f64 = 0.5;

struct App {
    asm: Assembly,
    met: f64,
    paused: bool,
    exit: bool,
    last_tick: Instant,
    /// 最近一次接触力计算结果。
    contact: SurfaceContact,
    /// 初始下沉速度（用于重置）。
    init_vert: f64,
}

impl App {
    fn new(init_vert: f64) -> Self {
        let (asm, _) = build_landing(init_vert);
        App {
            asm,
            met: 0.0,
            paused: false,
            exit: false,
            last_tick: Instant::now(),
            contact: SurfaceContact::default(),
            init_vert,
        }
    }

    fn altitude(&self) -> f64 {
        let pos = self.asm.vessels[self.asm.active].state.pos;
        pos.length() - EARTH_R
    }

    fn vertical_speed(&self) -> f64 {
        let state = self.asm.vessels[self.asm.active].state;
        let r_mag = state.pos.length();
        if r_mag < 1e-3 {
            return 0.0;
        }
        let radial = state.pos * (1.0 / r_mag);
        dot(state.vel, radial)
    }

    fn tick(&mut self) {
        if self.paused {
            return;
        }

        let earth = GravBody {
            pos: Vec3::ZERO,
            mass: 5.972e24,
            size: EARTH_R,
            jcoeff: vec![],
        };
        let grav = vec![earth];

        // 物理积分。
        self.asm.step(DT, &grav);

        // 计算地面接触力并施加到状态。
        let state = self.asm.vessels[self.asm.active].state;
        let td_points = &self.asm.vessels[self.asm.active].touchdown_points;
        let total_mass = self.asm.total_mass();
        let contact = compute_surface_forces(td_points, &state, EARTH_R, DT, total_mass);

        if contact.in_contact {
            // 施加接触力冲量到速度。
            let dv = contact.force * (DT / total_mass);
            // 施加接触力矩冲量到角速度。
            // torque 是体坐标系，需转世界系后除以惯量。
            let pmi = self.asm.composite_pmi();
            // 体坐标系角加速度 = tau / (mass * pmi)。
            let d_omega_body = Vec3::new(
                contact.torque.x / (total_mass * pmi.x),
                contact.torque.y / (total_mass * pmi.y),
                contact.torque.z / (total_mass * pmi.z),
            ) * DT;
            // 转世界系。
            let d_omega_world = mul(state.r, d_omega_body);

            for v in &mut self.asm.vessels {
                if !v.detached {
                    v.state.vel += dv;
                    v.state.omega += d_omega_world;
                }
            }
        }

        self.contact = contact;
        self.met += DT;
    }

    fn reset(&mut self, init_vert: f64) {
        let (asm, _) = build_landing(init_vert);
        self.asm = asm;
        self.met = 0.0;
        self.paused = false;
        self.contact = SurfaceContact::default();
        self.init_vert = init_vert;
    }

    fn handle_key(&mut self, key: KeyCode) {
        match key {
            KeyCode::Char('q') | KeyCode::Esc => self.exit = true,
            KeyCode::Char(' ') => self.paused = !self.paused,
            KeyCode::Char('r') => self.reset(SOFT_VERT),
            KeyCode::Char('h') => self.reset(HARD_VERT),
            _ => {}
        }
    }

    fn run(&mut self, terminal: &mut DefaultTerminal) -> std::io::Result<()> {
        self.last_tick = Instant::now();
        while !self.exit {
            terminal.draw(|frame| self.draw(frame))?;

            let timeout = Duration::from_millis(16);
            if event::poll(timeout)? {
                if let Event::Key(key) = event::read()? {
                    if key.kind == KeyEventKind::Press {
                        self.handle_key(key.code);
                    }
                }
            }

            // 每帧多步以加速仿真（0.05s 墙钟 ≈ 5 步）。
            let now = Instant::now();
            let elapsed = now.duration_since(self.last_tick).as_secs_f64().min(0.1);
            self.last_tick = now;
            let steps = (elapsed / DT).max(1.0) as usize;
            for _ in 0..steps {
                self.tick();
            }
        }
        Ok(())
    }

    fn draw(&self, frame: &mut ratatui::Frame) {
        let alt = self.altitude();
        let v_vert = self.vertical_speed();
        let contact_force = self.contact.force.length();
        let max_pen = self.contact.max_penetration;
        let in_contact = self.contact.in_contact;
        let omega = self.asm.vessels[self.asm.active].state.omega;
        let omega_mag = omega.length();

        let [title_area, table_area, help_area] = Layout::vertical([
            Constraint::Length(3),
            Constraint::Min(0),
            Constraint::Length(3),
        ])
        .areas(frame.area());

        // 标题栏。
        let mode = if self.init_vert < -10.0 {
            "硬着陆"
        } else {
            "软着陆"
        };
        let pause_tag = if self.paused { " [暂停]" } else { "" };
        let contact_tag = if in_contact { " [接触]" } else { "" };
        let title = format!(
            " orbitx 着陆接触力演示 — {}  T+{:.1}s{}{} ",
            mode, self.met, pause_tag, contact_tag,
        );
        let title_style = Style::default()
            .fg(Color::White)
            .add_modifier(Modifier::BOLD)
            .bg(Color::Black);
        let title_block = Block::default()
            .borders(Borders::ALL)
            .title(Line::from(Span::styled(title, title_style)))
            .style(Style::default().fg(Color::White).bg(Color::Black));
        frame.render_widget(title_block, title_area);

        // 遥测表格。
        let label_style = Style::default().fg(Color::Cyan).bold();
        let normal = Style::default().fg(Color::White);
        let danger = Style::default().fg(Color::Red).bold();
        let success = Style::default().fg(Color::Green).bold();
        let warning = Style::default().fg(Color::Yellow).bold();

        let alt_style = if alt < 0.0 { danger } else { normal };
        let contact_style = if in_contact { success } else { normal };
        let pen_style = if max_pen < -0.5 {
            danger
        } else if max_pen < 0.0 {
            warning
        } else {
            normal
        };
        let vert_style = if v_vert < -10.0 {
            danger
        } else if v_vert < -1.0 {
            warning
        } else {
            normal
        };

        let s_alt = format!("{:.2} m", alt);
        let s_vert = format!("{:.3} m/s", v_vert);
        let s_force = if contact_force > 1e6 {
            format!("{:.1} MN", contact_force / 1e6)
        } else if contact_force > 1e3 {
            format!("{:.1} kN", contact_force / 1e3)
        } else {
            format!("{:.1} N", contact_force)
        };
        let s_pen = format!("{:.4} m", max_pen);
        let s_contact = if in_contact { "YES" } else { "no" };
        let s_omega = format!("{:.4} rad/s", omega_mag);
        let s_time = format!("{:.2} s", self.met);
        let s_mass = format!("{:.0} kg", self.asm.total_mass());

        // 接触力分量。
        let f = self.contact.force;
        let s_fx = format!("{:.1} N", f.x);
        let s_fy = format!("{:.1} N", f.y);
        let s_fz = format!("{:.1} N", f.z);

        // 角速度分量。
        let s_wx = format!("{:.4}", omega.x);
        let s_wy = format!("{:.4}", omega.y);
        let s_wz = format!("{:.4}", omega.z);

        let rows = vec![
            Row::new(vec![
                cell("高度 Alt", label_style),
                cell(&s_alt, alt_style),
            ]),
            Row::new(vec![
                cell("垂直速度 Vvert", label_style),
                cell(&s_vert, vert_style),
            ]),
            Row::new(vec![
                cell("接触力 |F|", label_style),
                cell(&s_force, contact_style),
            ]),
            Row::new(vec![
                cell("  Fx", label_style),
                cell(&s_fx, normal),
            ]),
            Row::new(vec![
                cell("  Fy", label_style),
                cell(&s_fy, normal),
            ]),
            Row::new(vec![
                cell("  Fz", label_style),
                cell(&s_fz, normal),
            ]),
            Row::new(vec![
                cell("最大穿透 MaxPen", label_style),
                cell(&s_pen, pen_style),
            ]),
            Row::new(vec![
                cell("接触 Contact", label_style),
                cell(s_contact, contact_style),
            ]),
            Row::new(vec![
                cell("角速率 |ω|", label_style),
                cell(&s_omega, normal),
            ]),
            Row::new(vec![
                cell("  ωx", label_style),
                cell(&s_wx, normal),
            ]),
            Row::new(vec![
                cell("  ωy", label_style),
                cell(&s_wy, normal),
            ]),
            Row::new(vec![
                cell("  ωz", label_style),
                cell(&s_wz, normal),
            ]),
            Row::new(vec![
                cell("时间 Time", label_style),
                cell(&s_time, normal),
            ]),
            Row::new(vec![
                cell("质量 Mass", label_style),
                cell(&s_mass, normal),
            ]),
        ];

        let telemetry = Table::new(
            rows,
            [Constraint::Percentage(40), Constraint::Percentage(60)],
        )
        .style(Style::default().fg(Color::White).bg(Color::Black))
        .column_spacing(1)
        .block(
            Block::default()
                .borders(Borders::ALL)
                .title(" 遥测 Telemetry ")
                .style(Style::default().fg(Color::White).bg(Color::Black)),
        );
        frame.render_widget(telemetry, table_area);

        // 底部快捷键。
        let key_style = Style::default().fg(Color::Cyan).bold().bg(Color::Black);
        let desc_style = Style::default().fg(Color::White).bg(Color::Black);
        let help_text = Line::from(vec![
            Span::styled(" Space", key_style),
            Span::styled(" 暂停  ", desc_style),
            Span::styled("R", key_style),
            Span::styled(" 软着陆  ", desc_style),
            Span::styled("H", key_style),
            Span::styled(" 硬着陆  ", desc_style),
            Span::styled("Q", Style::default().fg(Color::Red).bold().bg(Color::Black)),
            Span::styled(" 退出", desc_style),
        ]);
        let help = Paragraph::new(help_text).block(
            Block::default()
                .borders(Borders::ALL)
                .title(" 控制 Controls ")
                .style(Style::default().fg(Color::White).bg(Color::Black)),
        );
        frame.render_widget(help, help_area);
    }
}

fn cell(text: &str, style: Style) -> ratatui::widgets::Cell<'_> {
    ratatui::widgets::Cell::from(text).style(style)
}

/// 构造着陆器 Assembly 和初始状态。
fn build_landing(init_vert: f64) -> (Assembly, f64) {
    let stage = StageSpec {
        name: "Lander",
        dry_mass: DRY_MASS,
        fuel_mass: FUEL_MASS,
        thrust: 0.0,
        isp: 0.0,
        engine_dir: Vec3::new(0.0, 1.0, 0.0),
        engine_pos: Vec3::ZERO,
        length: STAGE_LENGTH,
        radius: STAGE_RADIUS,
        separation_impulse: 0.0,
        pmi: orbitx_vessel::stage::PMI_UNDEF,
        max_gimbal: 0.0,
        max_gimbal_rate: 0.0,
        gimbal_axis: Vec3::new(1.0, 0.0, 0.0),
    };

    // 初始位置：地表 + 初始高度 + 半个级长度（级中心在半高处）。
    // 触地点在 body Y=-5，级中心在 Y=0，所以触地点比级中心低 5 m。
    // 级中心高度 = INIT_ALT + 5（使触地点恰好在 INIT_ALT）。
    let center_alt = INIT_ALT + GEAR_Y.abs();
    let init_pos = Vec3::new(0.0, 0.0, EARTH_R + center_alt);
    // 初始速度：沿径向（+Z 方向）向下。
    let init_vel = Vec3::new(0.0, 0.0, init_vert);
    // 姿态：体 +Y 朝上（径向），与 CLI 的 launch_attitude 一致。
    // 这里 pos 沿 +Z，径向 = +Z，体 +Y 应映射到 +Z。
    // 旋转矩阵：body Y → world Z。
    //   body X → world X, body Y → world Z, body Z → world -Y。
    let init_r = Matrix3::new(
        1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0,
    );
    let init_q = Quat::from_matrix(init_r);

    let init_state = StateVectors {
        pos: init_pos,
        vel: init_vel,
        omega: Vec3::ZERO,
        r: init_r,
        q: init_q,
    };

    let mut asm = Assembly::new(&[stage], init_state);
    asm.planet_radius = EARTH_R;

    // 配置着陆架触地点。
    let gear = make_landing_gear(GEAR_RADIUS, GEAR_Y, GEAR_STIFFNESS, GEAR_DAMPING, GEAR_MU);
    for v in &mut asm.vessels {
        v.touchdown_points = gear.clone();
    }

    (asm, init_vert)
}

fn main() -> std::io::Result<()> {
    let mut app = App::new(SOFT_VERT);
    ratatui::run(|terminal| app.run(terminal))
}
