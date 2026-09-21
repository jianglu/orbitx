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
//!   W          推力开关（开启时若节流阀为 0 则自动拉满）
//!   S          分离当前级
//!   ↑/↓        节流阀增/减
//!   ←/→        俯仰角增/减
//!   G          切换自动重力转向
//!   Space      暂停/继续
//!   +/-        时间加速/减速
//!   C          切换观察焦点（主组合体 / 分离体；非主时控制锁定）
//!   R          重置
//!   Q/Esc      退出

use std::time::{Duration, Instant};

use crossterm::event::{self, Event, KeyCode, KeyEventKind, KeyModifiers};
use orbitx_config::{BodyConfig, RocketConfig, ScenarioConfig};
use orbitx_dynamics::{Elements, GravBody};
use orbitx_math::{cross, dot, Matrix3, Quat, StateVectors, Vec3};
use orbitx_cli::control::{
    apply_throttle, apply_tvc, lit_thrusting_indices, perform_separate, primary_thrust_sum,
    should_auto_separate, ThrottlePolicy,
};
use orbitx_cli::crash::apply_crash_checks;
use orbitx_cli::focus::ViewFocus;
use orbitx_cli::telem;
use orbitx_vessel::{
    atmosphere_from_config, surface_inertial_velocity, Assembly, StageSpec,
};
use ratatui::layout::{Constraint, Layout};
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Span};
use ratatui::widgets::{Block, Borders, Gauge, Paragraph, Row, Table};
use ratatui::DefaultTerminal;

/// 地球物理参数（来自 BodyConfig::earth()，与 Orbiter Earth.cfg 一致）。
fn earth_body_config() -> BodyConfig {
    BodyConfig::earth()
}

const EARTH_R: f64 = 6.37101e6;  // BodyConfig::earth().size
const EARTH_GM: f64 = orbitx_math::consts::GGRAV * 5.973698968e24;  // Orbiter mass
const G0: f64 = 9.80665;
const LAUNCH_POS: Vec3 = Vec3::new(0.0, 0.0, EARTH_R);

/// 固定步长（可复现模式）每帧推进的仿真秒数。
/// 默认模式下 tick() 用此值乘以 time_scale，与墙钟解耦，保证轨迹可复现。
const FIXED_DT: f64 = 0.05;

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

/// 显示用：若四舍五入到 `decimals` 位后为 0，则返回 +0.0。
fn scrub_display_zero(x: f64, decimals: u32) -> f64 {
    let scale = 10f64.powi(decimals as i32);
    if (x * scale).round().abs() < f64::EPSILON {
        0.0
    } else {
        x
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
        .map(orbitx_vessel::stage_spec_from_config)
        .collect()
}

fn dock_links_from_config(config: &RocketConfig) -> Option<Vec<(usize, usize, usize, usize)>> {
    config.dock_links.as_ref().map(|links| {
        links
            .iter()
            .map(|l| (l.stage, l.port, l.remote_stage, l.remote_port))
            .collect()
    })
}

fn make_assembly(
    stages: &[StageSpec],
    init_state: StateVectors,
    links: &Option<Vec<(usize, usize, usize, usize)>>,
) -> Assembly {
    match links {
        Some(l) => Assembly::with_dock_links(stages, init_state, l),
        None => Assembly::new(stages, init_state),
    }
}

struct App {
    asm: Assembly,
    rocket_name: String,
    met: f64,
    pitch_target: f64, // 期望俯仰角 [rad]（制导律输出，由重力转向或手动 ←/→ 设定）
    yaw_target: f64,   // 期望偏航 tip [rad]（HUD / TVC；暂无键位）
    roll_target: f64,  // 期望滚转 [rad]（仅 HUD；无执行器）
    throttle_target: f64,
    thrusting: bool,
    /// 节流阀组合策略（过渡：日后迁 `orbitx-controller`）。
    throttle_policy: ThrottlePolicy,
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
    dock_links: Option<Vec<(usize, usize, usize, usize)>>,
    initial_pos: Vec3,
    crash_msg: String,
    /// UI 观察焦点；非 Primary 时飞行控制键锁定。
    view_focus: ViewFocus,
}

impl App {
    fn new(
        stages: &[StageSpec],
        name: &str,
        dock_links: Option<Vec<(usize, usize, usize, usize)>>,
    ) -> Self {
        let half_h: f64 = stages.iter().map(|s| s.length).sum::<f64>() / 2.0;
        let radial = LAUNCH_POS * (1.0 / LAUNCH_POS.length());
        let init_pos = LAUNCH_POS + radial * half_h;
        // 初始姿态：体 +Y（头部）对齐径向（垂直竖立）。
        // 不用 IDENTITY——否则体 -Y（推力）映射到世界 -Y（水平）而非朝下。
        let (init_r, init_q) = launch_attitude(radial);
        let earth_cfg = earth_body_config();
        let sid_period = earth_cfg
            .rotation
            .as_ref()
            .map(|r| r.sid_rot_period)
            .unwrap_or(86_164.1);
        // 台位速度 = ω×r（与共转大气一致，空速≈0）；位置由 tick 锁死，避免惯性经度漂移。
        let init_vel = surface_inertial_velocity(init_pos, sid_period);
        let init_state = StateVectors {
            pos: init_pos,
            vel: init_vel,
            r: init_r,
            q: init_q,
            ..Default::default()
        };
        let mut asm = make_assembly(stages, init_state, &dock_links);
        // from_spec 已写入 Cd(M) 阻力；仅补 rdrag 量级（勿重复 push dragels）。
        for v in &mut asm.vessels {
            if v.rdrag.length() < 1e-12 {
                v.rdrag = Vec3::new(1.0, 0.1, 1.0);
            }
        }
        asm.atmosphere = atmosphere_from_config(earth_cfg.atmosphere.as_ref());
        asm.planet_radius = earth_cfg.size;
        asm.sid_rot_period = sid_period;
        // 默认同步主组合体有推船（CZ-2F 侧挂冒烟；同轴火箭与只开底级等价）。
        App {
            asm,
            rocket_name: name.to_string(),
            met: 0.0,
            pitch_target: 0.0,
            yaw_target: 0.0,
            roll_target: 0.0,
            throttle_target: 0.0,
            thrusting: false,
            throttle_policy: ThrottlePolicy::SyncPrimary,
            launched: false,
            auto_gravity_turn: false,
            paused: false,
            time_scale: 1.0,
            realtime: false,
            exit: false,
            last_tick: Instant::now(),
            initial_stages: stages.to_vec(),
            dock_links,
            initial_pos: init_pos,
            crash_msg: String::new(),
            view_focus: ViewFocus::primary(),
        }
    }

    fn altitude(&self) -> f64 {
        let (pos, _) = self.asm.render_state();
        pos.length() - EARTH_R
    }

    /// 当前活动级可万向节主推的平均俯仰/偏航角 [rad]（HUD）。
    fn gimbal_angles(&self) -> (f64, f64) {
        let active = &self.asm.vessels[self.asm.active];
        let mut n = 0usize;
        let mut sum_p = 0.0;
        let mut sum_y = 0.0;
        for t in active.thrusters.iter().filter(|t| t.max_gimbal > 0.0) {
            sum_p += t.gimbal_pitch;
            sum_y += t.gimbal_yaw;
            n += 1;
        }
        if n == 0 {
            (0.0, 0.0)
        } else {
            let inv = 1.0 / n as f64;
            (sum_p * inv, sum_y * inv)
        }
    }

    /// 活动级主推实际开度均值（0..1）。
    fn actual_throttle(&self) -> f64 {
        let v = &self.asm.vessels[self.asm.active];
        let n = v.n_main_thrusters.min(v.thrusters.len());
        if n == 0 {
            return 0.0;
        }
        v.thrusters[..n].iter().map(|t| t.level).sum::<f64>() / n as f64
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

        // 发射台支撑：仅在未起飞时生效。
        // 松绑条件（hold-down）：已开推力且主组合体推重比 > 1.05，
        // 或径向速度已明显向上。避免「只改 vessel、不改 asm.state」导致
        // 速度判定永远过不了、燃料空烧的死锁。
        let on_pad = !self.launched;

        // 重力转向更新目标俯仰角。
        if self.auto_gravity_turn {
            let h = self.altitude();
            if h > 10_000.0 {
                let target = ((h - 10_000.0) / 70_000.0).min(1.0) * std::f64::consts::FRAC_PI_2;
                self.pitch_target = target;
            }
        }

        // TVC 闭环：有符号双轴 PD，仅 lit 主推；竖直保持时 pitch/yaw_target=0。
        apply_tvc(&mut self.asm, self.pitch_target, self.yaw_target, dt);

        // 先下节流阀指令；实际开度在 step 内斜坡逼近后再判松台架。
        let thr = if self.thrusting {
            self.throttle_target
        } else {
            0.0
        };
        apply_throttle(&mut self.asm, self.throttle_policy, thr);

        // 积分。
        // 使用 BodyConfig::earth() 的质量（Orbiter 值 5.973698968e24）。
        // 启用 J2 摄动（1.0826e-3），使轨道力学更真实。
        let earth_cfg = earth_body_config();
        let earth = GravBody {
            pos: Vec3::ZERO,
            mass: earth_cfg.mass,
            size: earth_cfg.size,
            jcoeff: vec![1.0826e-3],  // Earth J2
            rotation: None,  // TODO: use RotationState when integrated
            pines: None,
        };
        let grav = vec![earth];
        self.asm.step(dt, &grav);

        if self.thrusting && thr > 1e-6 {
            let thrust = primary_thrust_sum(&self.asm);
            let weight = self.asm.total_mass() * G0;
            if thrust > weight * 1.05 {
                self.launched = true;
            } else {
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
        }

        // 发射台：位置钉在 initial_pos（经纬度不变）；速度保持 ω×r（对地静止、空速≈0）。
        if on_pad && !self.launched {
            let pad_pos = self.initial_pos;
            let r_mag = pad_pos.length().max(1e-3);
            let radial_unit = pad_pos * (1.0 / r_mag);
            let pad_vel = if self.asm.sid_rot_period > 1e-9 {
                surface_inertial_velocity(pad_pos, self.asm.sid_rot_period)
            } else {
                Vec3::ZERO
            };
            let (lock_r, lock_q) = launch_attitude(radial_unit);
            for v in &mut self.asm.vessels {
                if !v.detached {
                    v.state.pos = pad_pos;
                    v.state.vel = pad_vel;
                    v.state.omega = Vec3::ZERO;
                    v.state.q = lock_q;
                    v.state.r = lock_r;
                }
            }
            let root = self.asm.root.min(self.asm.vessels.len().saturating_sub(1));
            self.asm.state = self.asm.vessels[root].state;
        }

        self.met += dt;

        // 可选诊断：ORBITX_CLI_DIAG=/path/log 时每 ~0.5s MET 追加一行。
        if let Ok(path) = std::env::var("ORBITX_CLI_DIAG") {
            if !path.is_empty() {
                let prev = (self.met - dt).div_euclid(0.5);
                let now = self.met.div_euclid(0.5);
                if now > prev || self.met < dt * 1.5 {
                    let thr_n = primary_thrust_sum(&self.asm);
                    let mass = self.asm.total_mass();
                    let fuel = self.asm.vessels.iter().map(|v| v.fuel_mass).sum::<f64>();
                    let h = self.altitude();
                    let vel = self.asm.vessels[self.asm.active].state.vel.length();
                    let line = format!(
                        "met={:.2} thr={:.0} T={:.0} W={:.0} T/W={:.3} fuel={:.0} alt={:.1} vel={:.2} pad={} launched={}\n",
                        self.met,
                        if self.thrusting {
                            self.throttle_target
                        } else {
                            0.0
                        },
                        thr_n,
                        mass * G0,
                        if mass > 1e-9 { thr_n / (mass * G0) } else { 0.0 },
                        fuel,
                        h,
                        vel,
                        on_pad && !self.launched,
                        self.launched,
                    );
                    let _ = std::fs::OpenOptions::new()
                        .create(true)
                        .append(true)
                        .open(&path)
                        .and_then(|mut f| {
                            use std::io::Write;
                            f.write_all(line.as_bytes())
                        });
                }
            }
        }

        // 自动分离（侧挂叶优先 undock，否则同轴 separate_stage）。
        if should_auto_separate(&self.asm) {
            perform_separate(&mut self.asm);
            self.view_focus.clamp(&self.asm);
        }

        // 碰撞：每 tick 扫描主栈 + 全部独立体（与焦点无关）；物理层只接收 mark_crashed。
        if let Some((name, impact_speed)) = apply_crash_checks(&mut self.asm, self.launched) {
            if self.crash_msg.is_empty() {
                self.crash_msg = format!("{} 撞击地面，速度 {:.0} m/s", name, impact_speed);
                self.paused = true;
            }
        }
    }

    fn reset(&mut self) {
        let radial = self.initial_pos * (1.0 / self.initial_pos.length().max(1e-3));
        let (init_r, init_q) = launch_attitude(radial);
        let earth_cfg = earth_body_config();
        let sid_period = earth_cfg
            .rotation
            .as_ref()
            .map(|r| r.sid_rot_period)
            .unwrap_or(86_164.1);
        let init_vel = surface_inertial_velocity(self.initial_pos, sid_period);
        let init_state = StateVectors {
            pos: self.initial_pos,
            vel: init_vel,
            r: init_r,
            q: init_q,
            ..Default::default()
        };
        self.asm = make_assembly(&self.initial_stages, init_state, &self.dock_links);
        self.asm.atmosphere = atmosphere_from_config(earth_cfg.atmosphere.as_ref());
        self.asm.planet_radius = earth_cfg.size;
        self.asm.sid_rot_period = sid_period;
        self.met = 0.0;
        self.pitch_target = 0.0;
        self.yaw_target = 0.0;
        self.roll_target = 0.0;
        self.throttle_target = 0.0;
        self.thrusting = false;
        self.launched = false;
        self.paused = false;
        self.crash_msg.clear();
        self.view_focus = ViewFocus::primary();
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

        // 始终有效：退出 / 观察切换 / 暂停 / 重置 / 倍速。
        match key {
            KeyCode::Char('q') | KeyCode::Esc => {
                self.exit = true;
                return;
            }
            KeyCode::Char('c') if modifiers.contains(KeyModifiers::CONTROL) => {
                self.exit = true;
                return;
            }
            KeyCode::Char('c') | KeyCode::Char('C') => {
                self.view_focus.cycle(&self.asm);
                return;
            }
            KeyCode::Char(' ') => {
                self.paused = !self.paused;
                return;
            }
            KeyCode::Char('r') => {
                self.reset();
                return;
            }
            KeyCode::Char('+') | KeyCode::Char('=') => {
                self.time_scale *= 2.0;
                return;
            }
            KeyCode::Char('-') => {
                self.time_scale /= 2.0;
                return;
            }
            _ => {}
        }

        // 飞行控制：仅主组合体焦点。
        if !self.view_focus.controls_enabled() {
            return;
        }

        match key {
            KeyCode::Char('w') => {
                self.thrusting = !self.thrusting;
                if self.thrusting && self.throttle_target < 1e-6 {
                    self.throttle_target = 1.0;
                }
            }
            KeyCode::Char('s') => {
                if self.asm.stage_count() > 1 {
                    perform_separate(&mut self.asm);
                    self.view_focus.clamp(&self.asm);
                }
            }
            KeyCode::Up => self.throttle_target = (self.throttle_target + 0.1).min(1.0),
            KeyCode::Down => self.throttle_target = (self.throttle_target - 0.1).max(0.0),
            KeyCode::Left => {
                self.pitch_target = (self.pitch_target - 1.0_f64.to_radians()).max(0.0)
            }
            KeyCode::Right => {
                self.pitch_target =
                    (self.pitch_target + 1.0_f64.to_radians()).min(std::f64::consts::FRAC_PI_2)
            }
            KeyCode::Char('g') => self.auto_gravity_turn = !self.auto_gravity_turn,
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
        let earth = GravBody {
            pos: Vec3::ZERO,
            mass: 5.973698968e24,
            size: EARTH_R,
            jcoeff: vec![],
            rotation: None,
            pines: None,
        };
        let snap = telem::snapshot(
            &self.asm,
            self.view_focus,
            &self.initial_stages,
            &[earth],
        );
        let h = snap.altitude.max(0.0);
        let vel_inertial = snap.vel_inertial;
        let r = snap.pos;
        let speed = snap.speed;
        let r_unit = r * (1.0 / r.length().max(1e-3));
        let v_vert = snap.v_vert;
        let v_horiz = snap.v_horiz;
        let mass = snap.mass;
        let fuel = snap.fuel;
        let fuel_pct = snap.fuel_pct;
        let thrust = snap.thrust;
        let tw = snap.twr;

        // 标题 + 底部 = 3行各
        let [title_area, main_area, fuel_area, help_area] = Layout::vertical([
            Constraint::Length(3),
            Constraint::Min(0),
            Constraint::Length(3),
            Constraint::Length(3),
        ])
        .areas(frame.area());

        // 左侧：遥测 + 发射场信息；右侧：轨道 + 级状态 + 姿态。
        let [left_area, right_area] =
            Layout::horizontal([Constraint::Percentage(33), Constraint::Percentage(67)])
                .areas(main_area);

        // 左侧再上下分割：遥测 + 发射场。
        let [telem_area, pad_area] =
            Layout::vertical([Constraint::Min(0), Constraint::Length(9)]).areas(left_area);

        // === 标题栏 ===
        let focus_tag = if snap.is_primary {
            String::new()
        } else {
            format!(" 观察: {} ", snap.display_name)
        };
        let title = format!(
            " orbitx 发射模拟器 — {}  {}  Stage: {}  (剩余 {} 级){} ",
            self.rocket_name,
            fmt_time(self.met),
            self.asm.active_name(),
            self.asm.stage_count(),
            focus_tag,
        );
        let status_tags = if self.paused {
            " [暂停]"
        } else if self.thrusting {
            " [推力]"
        } else {
            ""
        };
        let lock_tag = if snap.is_primary {
            ""
        } else {
            " [控制锁定]"
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
            Span::styled(lock_tag, Style::default().fg(Color::Yellow).bg(Color::Black)),
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

        let d = &snap.env;
        let s_agrav = format!("{:.3} m/s²", d.a_grav);
        let s_gmult = format!("{:.3}", d.g_multiple);
        let s_mach = format!("{:.2}", d.mach);
        let s_rho = format!("{:.4} kg/m³", d.density);
        let s_q = if d.dynamic_pressure >= 1000.0 {
            format!("{:.1} kPa", d.dynamic_pressure / 1000.0)
        } else {
            format!("{:.0} Pa", d.dynamic_pressure)
        };
        let s_p = if d.pressure >= 1000.0 {
            format!("{:.1} kPa", d.pressure / 1000.0)
        } else {
            format!("{:.0} Pa", d.pressure)
        };
        let s_pfac = format!("{:.3}", d.thrust_atm_scale);
        let s_isp = format!("{:.0} s", d.isp_eff);
        let s_drag = format!("{:.0} N", d.drag_force);
        let s_cd = format!("{:.3}", d.cd_eff);
        let s_tatm = format!("{:.1} °C", d.temperature - 273.15);
        let s_asnd = format!("{:.0} m/s", d.sound_speed);
        let s_n = format!("{:.2}", d.load_factor);

        // 危险状态高亮颜色。
        let danger = Style::default().fg(Color::Red).bold();
        let warning = Style::default().fg(Color::Yellow).bold();
        // 正常态：显式亮白（不能用 Style::default()，否则 fg 是终端默认色，
        // 在深色背景下几乎看不见）。
        let normal = Style::default().fg(Color::White);

        // 高度负值 = 地下（危险）。
        let alt_style = if snap.altitude < 0.0 {
            danger
        } else {
            normal
        };
        // T/W < 1 = 推力不足（警告）。
        let tw_style = if tw < 1.0 && self.thrusting && snap.is_primary {
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
                ratatui::widgets::Cell::from("垂直速度 Vvert").style(label_style),
                ratatui::widgets::Cell::from(s_vvert.as_str()).style(value_style),
            ]),
            Row::new(vec![
                ratatui::widgets::Cell::from("水平速度 Vhoriz").style(label_style),
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
                ratatui::widgets::Cell::from("引力加速度 Grav").style(label_style),
                ratatui::widgets::Cell::from(s_agrav.as_str()).style(value_style),
            ]),
            Row::new(vec![
                ratatui::widgets::Cell::from("重力倍数 Gmul").style(label_style),
                ratatui::widgets::Cell::from(s_gmult.as_str()).style(value_style),
            ]),
            Row::new(vec![
                ratatui::widgets::Cell::from("过载 Load").style(label_style),
                ratatui::widgets::Cell::from(s_n.as_str()).style(value_style),
            ]),
            Row::new(vec![
                ratatui::widgets::Cell::from("马赫数 Mach").style(label_style),
                ratatui::widgets::Cell::from(s_mach.as_str()).style(value_style),
            ]),
            Row::new(vec![
                ratatui::widgets::Cell::from("大气密度 Rho").style(label_style),
                ratatui::widgets::Cell::from(s_rho.as_str()).style(value_style),
            ]),
            Row::new(vec![
                ratatui::widgets::Cell::from("动压 Q").style(label_style),
                ratatui::widgets::Cell::from(s_q.as_str()).style(value_style),
            ]),
            Row::new(vec![
                ratatui::widgets::Cell::from("大气压 P").style(label_style),
                ratatui::widgets::Cell::from(s_p.as_str()).style(value_style),
            ]),
            Row::new(vec![
                ratatui::widgets::Cell::from("推力气压修正 Pfac").style(label_style),
                ratatui::widgets::Cell::from(s_pfac.as_str()).style(value_style),
            ]),
            Row::new(vec![
                ratatui::widgets::Cell::from("有效比冲 Isp").style(label_style),
                ratatui::widgets::Cell::from(s_isp.as_str()).style(value_style),
            ]),
            Row::new(vec![
                ratatui::widgets::Cell::from("气动阻力 Drag").style(label_style),
                ratatui::widgets::Cell::from(s_drag.as_str()).style(value_style),
            ]),
            Row::new(vec![
                ratatui::widgets::Cell::from("有效阻力系数 Cd").style(label_style),
                ratatui::widgets::Cell::from(s_cd.as_str()).style(value_style),
            ]),
            Row::new(vec![
                ratatui::widgets::Cell::from("气温 Temp").style(label_style),
                ratatui::widgets::Cell::from(s_tatm.as_str()).style(value_style),
            ]),
            Row::new(vec![
                ratatui::widgets::Cell::from("声速 Asnd").style(label_style),
                ratatui::widgets::Cell::from(s_asnd.as_str()).style(value_style),
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
                .title(if snap.is_primary {
                    " 遥测 Telemetry ".to_string()
                } else {
                    format!(" 遥测 Telemetry · {} ", snap.display_name)
                })
                .style(Style::default().fg(Color::White).bg(Color::Black)),
        );
        frame.render_widget(telemetry, telem_area);

        // === 左侧底部：发射场信息 ===
        let pos = snap.pos;
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

        // === 右侧：轨道参数 + 级状态 + 姿态 ===
        let [orbit_area, stage_area, attitude_area] = Layout::vertical([
            Constraint::Length(8),
            Constraint::Min(0),
            Constraint::Length(9), // 与左侧发射场等高
        ])
        .areas(right_area);

        // 轨道参数（用惯性速度；遥测速度是对地速度）。
        let r_mag = r.length();
        let speed_inertial = vel_inertial.length();
        let v_circular = (EARTH_GM / r_mag).sqrt();
        let energy = speed_inertial * speed_inertial / 2.0 - EARTH_GM / r_mag;
        let energy_margin = EARTH_GM / r_mag * 0.01;

        let mut orbit_lines: Vec<Line> = Vec::new();
        let v_horiz_inertial = (vel_inertial - r_unit * dot(vel_inertial, r_unit)).length();
        if v_horiz_inertial > v_circular * 0.5 && energy < -energy_margin {
            let el = Elements::calculate(r, vel_inertial, EARTH_GM, 0.0);
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
        } else if energy > energy_margin && speed_inertial > 100.0 {
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

        // 级状态：ACTIVE=主控；FIRING=lit 且非 active（侧挂同步节流阀中）。
        let lit = lit_thrusting_indices(&self.asm);
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
                let firing = lit.iter().any(|&j| j == i)
                    && self.throttle_target > 1e-6
                    && self.thrusting
                    && v.diagnostics.thrust > 1e-3;
                let base = if v.crashed {
                    "CRASHED"
                } else if v.detached {
                    "DETACHED"
                } else if i == self.asm.active {
                    "ACTIVE"
                } else if firing {
                    "FIRING"
                } else {
                    "attached"
                };
                let is_view = if snap.is_primary {
                    i == self.asm.active
                } else {
                    i == snap.vessel_index
                };
                let status = if is_view {
                    format!("{base}/View")
                } else {
                    base.to_string()
                };
                let style = if v.crashed {
                    Style::default()
                        .fg(Color::Red)
                        .add_modifier(Modifier::BOLD)
                } else if is_view {
                    Style::default()
                        .fg(Color::Magenta)
                        .add_modifier(Modifier::BOLD)
                } else if i == self.asm.active && !v.detached {
                    Style::default()
                        .fg(Color::Green)
                        .add_modifier(Modifier::BOLD)
                } else if base == "FIRING" {
                    Style::default()
                        .fg(Color::Yellow)
                        .add_modifier(Modifier::BOLD)
                } else if v.detached {
                    Style::default().fg(Color::DarkGray).bg(Color::Black)
                } else {
                    Style::default().fg(Color::White).bg(Color::Black)
                };
                Row::new(vec![v.name.clone(), fuel_bar, status]).style(style)
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

        // === 右侧底部：姿态（轴 | 当前 | 目标）===
        let s_pitch = format!(
            "{:.1}°",
            scrub_display_zero(snap.pitch.to_degrees(), 1)
        );
        let s_yaw = format!(
            "{:.1}°",
            scrub_display_zero(snap.yaw.to_degrees(), 1)
        );
        let s_roll = format!(
            "{:.1}°",
            scrub_display_zero(snap.roll.to_degrees(), 1)
        );
        let s_tgt_p = if snap.is_primary {
            format!(
                "{:.1}°",
                scrub_display_zero(self.pitch_target.to_degrees(), 1)
            )
        } else {
            "—".to_string()
        };
        let s_tgt_y = if snap.is_primary {
            format!(
                "{:.1}°",
                scrub_display_zero(self.yaw_target.to_degrees(), 1)
            )
        } else {
            "—".to_string()
        };
        let s_tgt_r = if snap.is_primary {
            format!(
                "{:.1}°",
                scrub_display_zero(self.roll_target.to_degrees(), 1)
            )
        } else {
            "—".to_string()
        };
        let focus_vi = snap.vessel_index;
        let w = self.asm.vessels[focus_vi].state.omega;
        let s_omega = format!(
            "{:.2} / {:.2} / {:.2} °/s",
            scrub_display_zero(w.x.to_degrees(), 2),
            scrub_display_zero(w.y.to_degrees(), 2),
            scrub_display_zero(w.z.to_degrees(), 2)
        );
        let (gimbal_p, gimbal_y) = if snap.is_primary {
            self.gimbal_angles()
        } else {
            (0.0, 0.0)
        };
        let s_gimbal = if snap.is_primary {
            format!(
                "{:.2} / {:.2}°",
                scrub_display_zero(gimbal_p.to_degrees(), 2),
                scrub_display_zero(gimbal_y.to_degrees(), 2)
            )
        } else {
            "—".to_string()
        };
        let thr_cur = if snap.is_primary {
            self.actual_throttle()
        } else {
            let v = &self.asm.vessels[focus_vi];
            let n = v.n_main_thrusters.min(v.thrusters.len());
            if n == 0 {
                0.0
            } else {
                v.thrusters[..n].iter().map(|t| t.level).sum::<f64>() / n as f64
            }
        };
        let s_thr_cur = format!("{:.0}%", thr_cur * 100.0);
        let s_thr_tgt = if snap.is_primary {
            format!("{:.0}%", self.throttle_target * 100.0)
        } else {
            "—".to_string()
        };
        let att_label = Style::default().fg(Color::Cyan).bold().bg(Color::Black);
        let att_value = Style::default().fg(Color::White).bg(Color::Black);
        let att_header = Style::default().fg(Color::Cyan).bold().bg(Color::Black);

        // 单表三列：轴 | 当前 | 目标；Rate/TVC 仅有当前值。
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
                ratatui::widgets::Cell::from("推力矢量角 TVC").style(att_label),
                ratatui::widgets::Cell::from(s_gimbal.as_str()).style(att_value),
                ratatui::widgets::Cell::from("").style(att_value),
            ]),
        ];
        let attitude_table = Table::new(
            att_rows,
            [
                Constraint::Length(16),
                Constraint::Percentage(42),
                Constraint::Percentage(42),
            ],
        )
        .header(
            Row::new(vec!["", "当前 Current", "目标 Target"])
                .style(att_header),
        )
        .style(Style::default().fg(Color::White).bg(Color::Black))
        .column_spacing(1)
        .block(
            Block::default()
                .borders(Borders::ALL)
                .title(if snap.is_primary {
                    " 姿态 Attitude ".to_string()
                } else {
                    format!(" 姿态 Attitude · {} (只读) ", snap.display_name)
                })
                .style(Style::default().fg(Color::White).bg(Color::Black)),
        );
        frame.render_widget(attitude_table, attitude_area);

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
                    .title(if snap.is_primary {
                        " 燃料 Fuel ".to_string()
                    } else {
                        format!(" 燃料 Fuel · {} ", snap.display_name)
                    })
                    .style(Style::default().fg(Color::White).bg(Color::Black)),
            )
            .ratio(fuel_ratio)
            .gauge_style(Style::default().fg(fuel_color).bg(Color::Black))
            .style(Style::default().fg(Color::White).bg(Color::Black));
        frame.render_widget(fuel_gauge, fuel_area);

        let key_style = Style::default().fg(Color::Cyan).bold().bg(Color::Black);
        let desc_style = Style::default().fg(Color::White).bg(Color::Black);
        let muted = Style::default().fg(Color::DarkGray).bg(Color::Black);
        let help_text = if snap.is_primary {
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
    let raw_args: Vec<String> = std::env::args().skip(1).collect();

    // --realtime 标志：启用墙钟驱动（默认关闭 = 固定步长可复现）。
    let realtime = raw_args.iter().any(|a| a == "--realtime");
    // --smoke <secs>：无头点火跑指定仿真秒并打印遥测（自动化验证用）。
    let smoke_secs: Option<f64> = raw_args
        .iter()
        .position(|a| a == "--smoke")
        .and_then(|i| raw_args.get(i + 1))
        .and_then(|s| s.parse().ok());
    let mut args = Vec::new();
    let mut skip_next = false;
    for a in &raw_args {
        if skip_next {
            skip_next = false;
            continue;
        }
        if a == "--realtime" {
            continue;
        }
        if a == "--smoke" {
            skip_next = true;
            continue;
        }
        args.push(a.clone());
    }

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
    let dock_links = dock_links_from_config(&config);

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

    let mut app = App::new(&stages, &config.name, dock_links);
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
                    app.asm = make_assembly(
                        &stages,
                        StateVectors {
                            pos,
                            vel: init_state.vel,
                            ..Default::default()
                        },
                        &app.dock_links,
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
                    app.asm = make_assembly(
                        &stages,
                        StateVectors {
                            pos: pos_with_offset,
                            ..Default::default()
                        },
                        &app.dock_links,
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

    if let Some(secs) = smoke_secs {
        // 模拟按 W：点火 + 节流阀拉满。
        app.thrusting = true;
        app.throttle_target = 1.0;
        let mut next_log = 0.0;
        println!(
            "smoke: rocket={} secs={secs}",
            app.rocket_name,
        );
        while app.met < secs && app.crash_msg.is_empty() {
            app.tick();
            if app.met + 1e-9 >= next_log {
                let thr_n = primary_thrust_sum(&app.asm);
                let mass = app.asm.total_mass();
                let fuel: f64 = app.asm.vessels.iter().map(|v| v.fuel_mass).sum();
                let vel = app.asm.vessels[app.asm.active].state.vel.length();
                println!(
                    "met={:.2} thr={:.0} T={:.0} W={:.0} T/W={:.3} fuel={:.0} alt={:.1} vel={:.2} launched={} crash={}",
                    app.met,
                    if app.thrusting {
                        app.throttle_target
                    } else {
                        0.0
                    },
                    thr_n,
                    mass * G0,
                    if mass > 1e-9 { thr_n / (mass * G0) } else { 0.0 },
                    fuel,
                    app.altitude(),
                    vel,
                    app.launched,
                    !app.crash_msg.is_empty(),
                );
                next_log += 0.5;
            }
        }
        if !app.crash_msg.is_empty() {
            println!("CRASH: {}", app.crash_msg);
        }
        return Ok(());
    }

    ratatui::run(|terminal| app.run(terminal))
}
