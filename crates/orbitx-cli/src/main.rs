//! orbitx-cli：终端多级火箭发射模拟器（ratatui TUI）。
//!
//! 用法：
//!   cargo run -p orbitx-cli                # 默认 Falcon 9
//!   cargo run -p orbitx-cli -- falcon9     # 内置 Falcon 9
//!   cargo run -p orbitx-cli -- saturnv     # 内置 Saturn V
//!   cargo run -p orbitx-cli -- lm5         # 内置 长征五号
//!   cargo run -p orbitx-cli -- lm2f        # 内置 长征二号F
//!   cargo run -p orbitx-cli -- lm7         # 内置 长征七号
//!   cargo run -p orbitx-cli -- lm9         # 内置 长征九号
//!   cargo run -p orbitx-cli -- /path/to/rocket.toml  # 自定义文件
//!   cargo run -p orbitx-cli -- falcon9 --realtime     # 墙钟驱动（默认为固定步长可复现）
//!
//! 可复现性：默认使用固定步长（0.05s × time_scale），相同参数下每次运行
//! 物理轨迹完全一致。加 --realtime 切换为墙钟实时驱动（每帧 dt 来自实际
//! 时间差，适合直观体验但不保证可复现）。
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
use orbitx_config::{RocketConfig, ScenarioConfig};
use orbitx_dynamics::{Elements, GravBody};
use orbitx_math::{cross, dot, mul, Matrix3, Quat, StateVectors, Vec3};
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

/// 固定步长（可复现模式）每帧推进的仿真秒数。
/// 默认模式下 tick() 用此值乘以 time_scale，与墙钟解耦，保证轨迹可复现。
const FIXED_DT: f64 = 0.05;

fn air_density(h: f64) -> f64 {
    if !(0.0..=ATM_TOP).contains(&h) {
        0.0
    } else {
        RHO0 * (-h / SCALE_H).exp()
    }
}

/// 把十进制度格式化为度分秒 + 半球标识，如 `25°30′15″N`。
/// `pos_hemi`/`neg_hemi` 是正/负值的半球字母（纬度 N/S，经度 E/W）。
fn fmt_dms(deg: f64, pos_hemi: &str, neg_hemi: &str) -> String {
    let hemi = if deg < 0.0 { neg_hemi } else { pos_hemi };
    let mut d = deg.abs();
    let deg_part = d.trunc();
    d = (d - deg_part) * 60.0;
    let min_part = d.trunc();
    let sec_part = (d - min_part) * 60.0;
    format!("{}°{}′{:05.2}″{}", deg_part as i32, min_part as i32, sec_part, hemi)
}

/// 人类可读的高度：1 km 以上用 km，否则用 m。
fn fmt_alt(m: f64) -> String {
    if m.abs() >= 1000.0 {
        format!("{:.2} km", m / 1000.0)
    } else {
        format!("{:.0} m", m)
    }
}

/// 构造使火箭体 +Y 轴（头部）对齐到世界方向 `up`（径向"上"）的姿态。
///
/// 返回 `(Matrix3, Quat)`，使得 `mul(R, (0,1,0)) = up`（火箭垂直竖立）。
/// 由于 `engine_dir = +Y`（推力朝头部），推力方向也映射到 `up`（径向"上"），
/// 火箭得以垂直升起。gimbal 轴 X 保持在水平面内。
///
/// 这是发射台上的正确初始姿态：用 IDENTITY 会让体 +Y（推力）映射到世界
/// +Y（水平），火箭被横向加速而非垂直升起。
fn launch_attitude(up: Vec3) -> (Matrix3, Quat) {
    // body Y → up。选一个不平行于 up 的参考轴构造正交基。
    let ref_axis = if up.y.abs() < 0.9 {
        Vec3::new(0.0, 1.0, 0.0)
    } else {
        Vec3::new(1.0, 0.0, 0.0)
    };
    // body X = up × ref（水平面内）。
    let bx = cross(up, ref_axis).unit();
    // body Z = X × up 补全正交基。
    let bz = cross(bx, up).unit();
    let by = up;
    // 构造 R 使其列为 [bx, by, bz]：mul(R, e_i) = 第 i 列。
    // Matrix3::new 接收行优先 9 元素，故按列转置填入。
    let r = Matrix3::new(
        bx.x, by.x, bz.x, bx.y, by.y, bz.y, bx.z, by.z, bz.z,
    );
    let q = Quat::from_matrix(r);
    (r, q)
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
            pmi: s.inertia.map(|i| Vec3::new(i[0], i[1], i[2])).unwrap_or(orbitx_vessel::stage::PMI_UNDEF),
            max_gimbal: s.max_gimbal,
            max_gimbal_rate: s.max_gimbal_rate,
            gimbal_axis: Vec3::new(s.gimbal_axis[0], s.gimbal_axis[1], s.gimbal_axis[2]),
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
    pitch_target: f64, // 期望俯仰角 [rad]（制导律输出，由重力转向或手动 ←/→ 设定）
    throttle: f64,
    thrusting: bool,
    launched: bool,
    auto_gravity_turn: bool,
    paused: bool,
    time_scale: f64,
    /// true = 墙钟驱动（每帧 dt 来自实时时间差，轨迹不可复现）；
    /// false = 固定步长（可复现，默认）。
    realtime: bool,
    exit: bool,
    last_tick: Instant,
    initial_stages: Vec<StageSpec>,
    initial_pos: Vec3,
    crash_msg: String,
}

/// TVC 控制器增益（PD 控制：gimbal = Kp·err - Kd·ω）。
/// Kd > Kp 以提供强阻尼，避免 gimbal 饱和后姿态振荡发散。
/// 经增益扫描验证：Kp=1.0, Kd=2.0 对 Falcon9 级别火箭在 ~2s 内无超调收敛。
const TVC_KP: f64 = 1.0;
const TVC_KD: f64 = 2.0;

impl App {
    fn new(stages: &[StageSpec], name: &str) -> Self {
        let half_h: f64 = stages.iter().map(|s| s.length).sum::<f64>() / 2.0;
        let radial = LAUNCH_POS * (1.0 / LAUNCH_POS.length());
        let init_pos = LAUNCH_POS + radial * half_h;
        // 初始姿态：体 +Y（头部）对齐径向（垂直竖立）。
        // 不用 IDENTITY——否则体 -Y（推力）映射到世界 -Y（水平）而非朝下。
        let (init_r, init_q) = launch_attitude(radial);
        let init_state = StateVectors {
            pos: init_pos,
            r: init_r,
            q: init_q,
            ..Default::default()
        };
        let asm = Assembly::new(stages, init_state);
        App {
            asm,
            rocket_name: name.to_string(),
            met: 0.0,
            pitch_target: 0.0,
            throttle: 0.0,
            thrusting: false,
            launched: false,
            auto_gravity_turn: false,
            paused: false,
            time_scale: 1.0,
            realtime: false,
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

    /// 火箭当前真实俯仰角 [rad]：体 +Y 轴（轴向）在世界系中偏离径向的角度。
    ///
    /// 0 = 垂直（轴向沿径向），π/2 = 水平。由姿态四元数 `state.q` 反算，
    /// 反映刚体动力学的真实姿态。
    fn pitch(&self) -> f64 {
        let state = self.asm.vessels[self.asm.active].state;
        let r = state.pos;
        let r_mag = r.length();
        if r_mag < 1e-3 {
            return 0.0;
        }
        let radial = r * (1.0 / r_mag);
        // 体 +Y 轴在世界系的方向 = mul(state.r, +Y_body)。
        let body_axis_world = mul(state.r, Vec3::new(0.0, 1.0, 0.0));
        // 俯仰角 = arccos(body_axis · radial)。火箭垂直时两向量平行 → 0。
        let c = dot(body_axis_world, radial).clamp(-1.0, 1.0);
        c.acos()
    }

    /// 体坐标系角速度的俯仰分量 [rad/s]（绕 gimbal 轴 X 的转速）。
    fn pitch_rate(&self) -> f64 {
        self.asm.vessels[self.asm.active].state.omega.x
    }

    /// 当前活动级推进器的平均 gimbal 角 [rad]（用于遥测显示）。
    fn gimbal_angle(&self) -> f64 {
        let active = &self.asm.vessels[self.asm.active];
        let gimbals: Vec<f64> = active.thrusters.iter().map(|t| t.gimbal).collect();
        if gimbals.is_empty() {
            0.0
        } else {
            gimbals.iter().sum::<f64>() / gimbals.len() as f64
        }
    }

    /// TVC 闭环控制：根据俯仰误差生成 gimbal 指令并施加到活动级推进器。
    ///
    /// PD 控制：`gimbal_cmd = Kp·(pitch_target − pitch) − Kd·ω_pitch`。
    /// 正误差（需增大俯仰）→ 正 gimbal → 推力偏转产生正力矩 → 姿态前倾。
    /// Kd 项（角速度反馈）提供阻尼，防止 gimbal 饱和后姿态振荡发散。
    /// gimbal 角经推进器作动器速率限制（`slew_gimbal`）平滑过渡。
    fn update_tvc(&mut self, dt: f64) {
        let pitch = self.pitch();
        let omega_pitch = self.pitch_rate();
        let err = self.pitch_target - pitch;
        let cmd = TVC_KP * err - TVC_KD * omega_pitch;
        for v in &mut self.asm.vessels {
            if !v.detached {
                for t in &mut v.thrusters {
                    t.slew_gimbal(cmd, dt);
                }
            }
        }
    }

    fn tick(&mut self) {
        if self.paused {
            return;
        }
        // 步长来源：固定模式用 FIXED_DT（可复现），实时模式用墙钟差。
        let now = Instant::now();
        let dt = if self.realtime {
            let dt_real = now.duration_since(self.last_tick).as_secs_f64().min(0.1);
            dt_real * self.time_scale
        } else {
            FIXED_DT * self.time_scale
        };
        self.last_tick = now;

        // 发射台支撑力：仅在火箭未起飞时（launched=false）生效。
        // 一旦起飞（有推力且高度>1m），支撑力永久消失，后续触地即坠毁。
        let on_pad = !self.launched;

        // 重力转向更新目标俯仰角。
        if self.auto_gravity_turn {
            let h = self.altitude();
            if h > 10_000.0 {
                let target = ((h - 10_000.0) / 70_000.0).min(1.0) * std::f64::consts::FRAC_PI_2;
                self.pitch_target = target;
            }
        }

        // TVC 闭环控制：根据俯仰误差生成 gimbal 指令，驱动推进器偏转。
        // 力矩由 Assembly::step 内的 Euler 方程积分，姿态由 state.q 真实演化。
        self.update_tvc(dt);

        // 起飞检测：有推力且径向速度为正（远离地面）时标记为已起飞。
        if self.thrusting {
            let pos = self.asm.vessels[self.asm.active].state.pos;
            let vel = self.asm.vessels[self.asm.active].state.vel;
            let r_mag = pos.length();
            if r_mag > 1e-3 {
                let v_radial = dot(vel, pos * (1.0 / r_mag));
                if v_radial > 0.5 {
                    self.launched = true;
                }
            }
        }

        // 设置油门。
        let thr = if self.thrusting { self.throttle } else { 0.0 };
        self.asm.set_throttle(thr);

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

        // 发射台支撑力：火箭未起飞时生效。
        // 1. 径向法向约束：抵消朝下的径向速度分量。
        // 2. 姿态约束：抑制角速度并把姿态锁定在垂直（避免 TVC 力矩在地面
        //    使火箭倾倒——发射台塔架的物理约束）。
        if on_pad {
            let pos = self.asm.vessels[self.asm.active].state.pos;
            let vel = self.asm.vessels[self.asm.active].state.vel;
            let r_mag = pos.length();
            if r_mag > 1e-3 {
                let radial_unit = pos * (1.0 / r_mag);
                let v_radial = dot(vel, radial_unit);
                // 如果朝地面运动（v_radial < 0），移除径向速度分量。
                if v_radial < 0.0 {
                    let correction = radial_unit * (-v_radial);
                    for v in &mut self.asm.vessels {
                        if !v.detached {
                            v.state.vel += correction;
                        }
                    }
                }
                // 姿态约束：清零角速度，锁定姿态为垂直（体 +Y 对齐径向）。
                // 模拟发射塔对火箭的夹持。不能用 IDENTITY——否则推力方向错误。
                let (lock_r, lock_q) = launch_attitude(radial_unit);
                for v in &mut self.asm.vessels {
                    if !v.detached {
                        v.state.omega = Vec3::ZERO;
                        v.state.q = lock_q;
                        v.state.r = lock_r;
                    }
                }
                // 确保不低于初始高度。
                let floor = self.initial_pos.length();
                if r_mag < floor {
                    let fix = radial_unit * ((floor - r_mag) / r_mag);
                    for v in &mut self.asm.vessels {
                        if !v.detached {
                            v.state.pos += v.state.pos * fix;
                        }
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

        // 碰撞：起飞后触地即坠毁。
        if self.launched && self.altitude() < 0.0 && self.crash_msg.is_empty() {
            self.crash_msg = format!(
                "{} 撞击地面，速度 {:.0} m/s",
                self.asm.active_name(),
                self.velocity().length()
            );
            self.paused = true;
        }
    }

    fn reset(&mut self) {
        let radial = self.initial_pos * (1.0 / self.initial_pos.length().max(1e-3));
        let (init_r, init_q) = launch_attitude(radial);
        let init_state = StateVectors {
            pos: self.initial_pos,
            r: init_r,
            q: init_q,
            ..Default::default()
        };
        self.asm = Assembly::new(&self.initial_stages, init_state);
        self.met = 0.0;
        self.pitch_target = 0.0;
        self.throttle = 0.0;
        self.thrusting = false;
        self.launched = false;
        self.paused = false;
        self.crash_msg.clear();
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
            KeyCode::Left => self.pitch_target = (self.pitch_target - 0.1).max(0.0),
            KeyCode::Right => {
                self.pitch_target = (self.pitch_target + 0.1).min(std::f64::consts::FRAC_PI_2)
            }
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
        // 避免接近零时在 -0/0 间闪烁。
        let v_vert = if v_vert.abs() < 0.5 { 0.0 } else { v_vert };
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

        // 左侧：遥测 + 发射场信息；右侧：轨道 + 级状态。
        let [left_area, right_area] =
            Layout::horizontal([Constraint::Percentage(33), Constraint::Percentage(67)])
                .areas(main_area);

        // 左侧再上下分割：遥测 + 发射场。
        let [telem_area, pad_area] =
            Layout::vertical([Constraint::Min(0), Constraint::Length(8)]).areas(left_area);

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
        let mode_tag = if self.realtime { "[实时]" } else { "[可复现]" };
        let mode_style = if self.realtime {
            Style::default().fg(Color::Yellow).bold().bg(Color::Black)
        } else {
            Style::default().fg(Color::Green).bold().bg(Color::Black)
        };
        // 右侧标签：先拼接所有右侧 span（模式 + 状态 + 坠毁），再算填充。
        let crash_tag = if !self.crash_msg.is_empty() {
            format!(" !!! {} !!!", self.crash_msg)
        } else {
            String::new()
        };
        let right_spans = vec![
            Span::styled(mode_tag, mode_style),
            Span::styled(status_tags, Style::default().fg(Color::White).bg(Color::Black)),
            Span::styled(gravity_tag, Style::default().fg(Color::Yellow).bg(Color::Black)),
            Span::styled(warp_tag, Style::default().fg(Color::Cyan).bg(Color::Black)),
            Span::styled(crash_tag, Style::default().fg(Color::Red).bold().bg(Color::Black)),
        ];
        let right_width: usize = right_spans.iter().map(|s| s.width()).sum();
        // 有效宽度 = title_area 宽度 - 2（左右边框）。
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
        let title_line = Line::from(title_spans);
        let title_block = Block::default()
            .borders(Borders::ALL)
            .title(title_line)
            .style(Style::default().fg(Color::White).bg(Color::Black));
        frame.render_widget(title_block, title_area);

        // === 左侧：遥测表格 ===
        let s_alt = fmt_dist(h);
        let s_vel = format!("{:.0} m/s", speed);
        let s_vvert = format!("{:.0} m/s", v_vert);
        let s_vhoriz = format!("{:.0} m/s", v_horiz);
        let s_mass = fmt_mass(mass);
        let s_fuel = format!("{:.0} kg", fuel);
        let s_thrust = format!("{:.0} kN", (thrust / 1000.0).abs());
        let s_tw = format!("{:.2}", tw.abs());
        let s_thr = format!(
            "{:.0}%",
            if self.thrusting {
                self.throttle * 100.0
            } else {
                0.0
            }
        );
        let s_pitch = format!("{:.1}°", self.pitch().to_degrees());
        let s_omega = format!("{:.2} °/s", self.pitch_rate().to_degrees());
        let s_gimbal = format!("{:.2}°", self.gimbal_angle().to_degrees());

        // 危险状态高亮颜色。
        let danger = Style::default().fg(Color::Red).bold();
        let warning = Style::default().fg(Color::Yellow).bold();
        // 正常态：显式亮白（不能用 Style::default()，否则 fg 是终端默认色，
        // 在深色背景下几乎看不见）。
        let normal = Style::default().fg(Color::White);

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

        // 标签列样式：青色加粗，便于与数值列区分、提升对比度。
        let label_style = Style::default().fg(Color::Cyan).bold();
        // 数值列默认样式：亮白，确保在深色终端背景上清晰可读。
        let value_style = Style::default().fg(Color::White);

        let rows = vec![
            Row::new(vec![
                ratatui::widgets::Cell::from("高度 Alt").style(label_style),
                ratatui::widgets::Cell::from(s_alt.as_str()).style(alt_style),
            ]),
            Row::new(vec![
                ratatui::widgets::Cell::from("速度 Vel").style(label_style),
                ratatui::widgets::Cell::from(s_vel.as_str()).style(value_style),
            ]),
            Row::new(vec![
                ratatui::widgets::Cell::from("垂直 Vvert").style(label_style),
                ratatui::widgets::Cell::from(s_vvert.as_str()).style(value_style),
            ]),
            Row::new(vec![
                ratatui::widgets::Cell::from("水平 Vhoriz").style(label_style),
                ratatui::widgets::Cell::from(s_vhoriz.as_str()).style(value_style),
            ]),
            Row::new(vec![
                ratatui::widgets::Cell::from("质量 Mass").style(label_style),
                ratatui::widgets::Cell::from(s_mass.as_str()).style(value_style),
            ]),
            Row::new(vec![
                ratatui::widgets::Cell::from("燃料 Fuel").style(label_style),
                fuel_cell,
            ]),
            Row::new(vec![
                ratatui::widgets::Cell::from("推力 Thrust").style(label_style),
                ratatui::widgets::Cell::from(s_thrust.as_str()).style(value_style),
            ]),
            Row::new(vec![
                ratatui::widgets::Cell::from("推重比 T/W").style(label_style),
                ratatui::widgets::Cell::from(s_tw.as_str()).style(tw_style),
            ]),
            Row::new(vec![
                ratatui::widgets::Cell::from("油门 Thr").style(label_style),
                ratatui::widgets::Cell::from(s_thr.as_str()).style(value_style),
            ]),
            Row::new(vec![
                ratatui::widgets::Cell::from("俯仰 Pitch").style(label_style),
                ratatui::widgets::Cell::from(s_pitch.as_str()).style(value_style),
            ]),
            Row::new(vec![
                ratatui::widgets::Cell::from("角速率 Rate").style(label_style),
                ratatui::widgets::Cell::from(s_omega.as_str()).style(value_style),
            ]),
            Row::new(vec![
                ratatui::widgets::Cell::from("矢量 TVC").style(label_style),
                ratatui::widgets::Cell::from(s_gimbal.as_str()).style(value_style),
            ]),
        ];
        let telemetry = Table::new(
            rows,
            [Constraint::Percentage(45), Constraint::Percentage(55)],
        )
        .style(Style::default().fg(Color::White).bg(Color::Black))
        .column_spacing(1)
        .block(
            Block::default()
                .borders(Borders::ALL)
                .title(" 遥测 Telemetry ")
                .style(Style::default().fg(Color::White).bg(Color::Black)),
        );
        frame.render_widget(telemetry, telem_area);

        // === 左侧底部：发射场信息 ===
        let pos = self.asm.vessels[self.asm.active].state.pos;
        let r_mag = pos.length();
        // 计算经纬度（从 orbitx 左手系坐标）。
        let lat = (pos.y / r_mag).asin().to_degrees();
        let lng = pos.z.atan2(pos.x).to_degrees();
        let alt = r_mag - EARTH_R;
        let s_lat = fmt_dms(lat, "N", "S");
        let s_lng = fmt_dms(lng, "E", "W");
        let s_alt = fmt_alt(alt);
        let launch_status = if self.launched { "已起飞" } else { "待命" };
        let launch_style = if self.launched {
            Style::default().fg(Color::Green).bg(Color::Black)
        } else {
            Style::default().fg(Color::Yellow).bg(Color::Black)
        };
        let pad_label = Style::default().fg(Color::Cyan).bold().bg(Color::Black);
        let pad_value = Style::default().fg(Color::White).bg(Color::Black);
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
                ratatui::widgets::Cell::from(s_alt.as_str()).style(pad_value),
            ]),
            Row::new(vec![
                ratatui::widgets::Cell::from("状态 Status").style(pad_label),
                ratatui::widgets::Cell::from(launch_status).style(launch_style),
            ]),
        ];
        let pad_table = Table::new(
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
        );
        frame.render_widget(pad_table, pad_area);

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
                Style::default().fg(Color::Cyan),
            )));
        }
        orbit_lines.push(Line::from(format!(" Energy  {:>8.1} MJ/kg", energy / 1e6)));
        let orbit_text = Paragraph::new(orbit_lines)
            .style(Style::default().fg(Color::White).bg(Color::Black))
            .block(
                Block::default()
                    .borders(Borders::ALL)
                    .title(" 轨道 Orbit ")
                    .style(Style::default().fg(Color::White).bg(Color::Black)),
            );
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
                    Style::default().fg(Color::DarkGray).bg(Color::Black)
                } else {
                    Style::default().fg(Color::White).bg(Color::Black)
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
                .style(Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD)),
        )
        .style(Style::default().fg(Color::White).bg(Color::Black))
        .column_spacing(1)
        .block(
            Block::default()
                .borders(Borders::ALL)
                .title(" 级状态 Stages ")
                .style(Style::default().fg(Color::White).bg(Color::Black)),
        );
        frame.render_widget(stage_table, stage_area);

        // === 底部：燃料条 + 快捷键 ===
        let fuel_ratio = (fuel_pct / 100.0).clamp(0.0, 1.0);
        let fuel_color = if fuel_ratio < 0.2 {
            Color::Red
        } else if fuel_ratio < 0.5 {
            Color::Yellow
        } else {
            Color::Green
        };
        let fuel_gauge = Gauge::default()
            .block(
                Block::default()
                    .borders(Borders::ALL)
                    .title(" 燃料 Fuel ")
                    .style(Style::default().fg(Color::White).bg(Color::Black)),
            )
            .ratio(fuel_ratio)
            .gauge_style(Style::default().fg(fuel_color).bg(Color::Black))
            .style(Style::default().fg(Color::White).bg(Color::Black));
        frame.render_widget(fuel_gauge, fuel_area);

        let key_style = Style::default().fg(Color::Cyan).bold().bg(Color::Black);
        let desc_style = Style::default().fg(Color::White).bg(Color::Black);
        let help_text = Line::from(vec![
            Span::styled(" W", key_style),
            Span::styled(" 推力  ", desc_style),
            Span::styled("S", key_style),
            Span::styled(" 分离  ", desc_style),
            Span::styled("↑↓", key_style),
            Span::styled(" 油门  ", desc_style),
            Span::styled("←→", key_style),
            Span::styled(" 俯仰  ", desc_style),
            Span::styled("G", key_style),
            Span::styled(" 重力转向  ", desc_style),
            Span::styled("Space", key_style),
            Span::styled(" 暂停  ", desc_style),
            Span::styled("+/-", key_style),
            Span::styled(" 加速  ", desc_style),
            Span::styled("R", key_style),
            Span::styled(" 重置  ", desc_style),
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

        // === 坠毁对话框（反色覆盖层） ===
        if !self.crash_msg.is_empty() {
            let invert = Style::default().bg(Color::Red).fg(Color::White);
            let invert_bold = Style::default().bg(Color::Red).fg(Color::White).bold();
            let dialog = Paragraph::new(vec![
                Line::raw(""),
                Line::from(Span::styled("!!! 坠毁 CRASH !!!", invert_bold)),
                Line::raw(""),
                Line::from(Span::styled(self.crash_msg.as_str(), invert)),
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

/// 内置火箭别名 → TOML 内容。
fn builtin_rocket(arg: &str) -> Option<&'static str> {
    let map: &[(&str, &str, &str)] = &[
        (
            "falcon9",
            "Falcon 9 (SpaceX)",
            include_str!("../../orbitx-config/presets/falcon9.toml"),
        ),
        (
            "saturnv",
            "Saturn V (NASA)",
            include_str!("../../orbitx-config/presets/saturn_v.toml"),
        ),
        (
            "lm5",
            "长征五号 Long March 5",
            include_str!("../../orbitx-config/presets/long_march_5.toml"),
        ),
        (
            "lm2f",
            "长征二号F Long March 2F",
            include_str!("../../orbitx-config/presets/long_march_2f.toml"),
        ),
        (
            "lm7",
            "长征七号 Long March 7",
            include_str!("../../orbitx-config/presets/long_march_7.toml"),
        ),
        (
            "lm9",
            "长征九号 Long March 9",
            include_str!("../../orbitx-config/presets/long_march_9.toml"),
        ),
    ];
    for (alias, _name, toml) in map {
        if *alias == arg {
            return Some(toml);
        }
    }
    None
}

fn print_available() {
    eprintln!("可用火箭：");
    eprintln!("  falcon9     Falcon 9 (SpaceX)");
    eprintln!("  saturnv     Saturn V (NASA)");
    eprintln!("  lm5         长征五号 Long March 5");
    eprintln!("  lm2f        长征二号F Long March 2F");
    eprintln!("  lm7         长征七号 Long March 7");
    eprintln!("  lm9         长征九号 Long March 9");
    eprintln!();
    eprintln!("用法：cargo run -p orbitx-cli -- <名称|文件路径> [--realtime]");
}

fn main() -> std::io::Result<()> {
    let args: Vec<String> = std::env::args().skip(1).collect();

    // --realtime 标志：启用墙钟驱动（默认关闭 = 固定步长可复现）。
    let realtime = args.iter().any(|a| a == "--realtime");
    let args: Vec<String> = args.into_iter().filter(|a| a != "--realtime").collect();

    let toml_str: String = if args.is_empty() {
        // 默认 Falcon 9。
        include_str!("../../orbitx-config/presets/falcon9.toml").to_string()
    } else {
        let arg = &args[0];
        // 先检查是否为文件路径。
        let path = std::path::Path::new(arg);
        if path.exists() {
            std::fs::read_to_string(path).unwrap_or_else(|e| {
                eprintln!("读取文件失败：{e}");
                std::process::exit(1);
            })
        } else if let Some(toml) = builtin_rocket(arg) {
            toml.to_string()
        } else {
            eprintln!("未知火箭：{arg}");
            eprintln!();
            print_available();
            std::process::exit(1);
        }
    };

    let config = RocketConfig::from_toml_str(&toml_str).unwrap_or_else(|e| {
        eprintln!("解析火箭配置失败：{e}");
        std::process::exit(1);
    });
    let stages = rocket_to_stages(&config);

    // 检查是否有第二个参数作为场景文件。
    let scenario: Option<ScenarioConfig> = if args.len() >= 2 {
        let scen_path = std::path::Path::new(&args[1]);
        if scen_path.exists() {
            match ScenarioConfig::from_file(scen_path) {
                Ok(s) => Some(s),
                Err(e) => {
                    eprintln!("解析场景配置失败：{e}");
                    None
                }
            }
        } else {
            None
        }
    } else {
        None
    };

    let mut app = App::new(&stages, &config.name);
    app.realtime = realtime;

    // 应用场景配置。
    if let Some(ref scn) = scenario {
        if let Some(ship) = scn.ships.first() {
            if ship.status == "orbiting" {
                if let (Some(rpos), Some(rvel)) = (ship.rpos, ship.rvel) {
                    let init_state = StateVectors {
                        pos: Vec3::new(rpos[0], rpos[1], rpos[2]),
                        vel: Vec3::new(rvel[0], rvel[1], rvel[2]),
                        ..Default::default()
                    };
                    let half_h: f64 = stages.iter().map(|s| s.length).sum::<f64>() / 2.0;
                    // 轨道起始不需要发射台。
                    let radial = init_state.pos * (1.0 / init_state.pos.length().max(1e-3));
                    let pos = init_state.pos + radial * half_h;
                    app.asm = Assembly::new(
                        &stages,
                        StateVectors {
                            pos,
                            vel: init_state.vel,
                            ..Default::default()
                        },
                    );
                    app.initial_pos = pos;
                    app.launched = true;
                }
            } else if ship.status == "landed" {
                // 使用场景中的经纬度定位。
                if let (Some(lng), Some(lat)) = (ship.longitude, ship.latitude) {
                    let lng_r = lng.to_radians();
                    let lat_r = lat.to_radians();
                    let pos = Vec3::new(
                        EARTH_R * lat_r.cos() * lng_r.cos(),
                        EARTH_R * lat_r.sin(),
                        EARTH_R * lat_r.cos() * lng_r.sin(),
                    );
                    let half_h: f64 = stages.iter().map(|s| s.length).sum::<f64>() / 2.0;
                    let radial = pos * (1.0 / pos.length());
                    let pos_with_offset = pos + radial * half_h;
                    app.asm = Assembly::new(
                        &stages,
                        StateVectors {
                            pos: pos_with_offset,
                            ..Default::default()
                        },
                    );
                    app.initial_pos = pos_with_offset;
                }
                // 应用燃料液位。
                if let Some(ref fuel_levels) = ship.fuel_level {
                    for (i, &level) in fuel_levels.iter().enumerate() {
                        if i < app.asm.vessels.len() {
                            let max_fuel = app.initial_stages[i].fuel_mass;
                            app.asm.vessels[i].fuel_mass = max_fuel * level;
                        }
                    }
                }
            }
        }
    }

    ratatui::run(|terminal| app.run(terminal))
}
