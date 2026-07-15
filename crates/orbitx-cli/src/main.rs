//! orbitx-cli：终端多级火箭发射模拟器。
//!
//! 用法：
//!   cargo run -p orbitx-cli              # Falcon 9
//!   cargo run -p orbitx-cli -- saturnv   # Saturn V
//!
//! 命令：
//!   throttle 1.0    设置油门 (0..1)
//!   sep             分离当前级
//!   pitch 45        设置俯仰角 (度)
//!   gravity on/off  开关自动重力转向
//!   run 60          快进 N 秒
//!   status          显示遥测
//!   help            帮助
//!   quit            退出

use std::io::{self, BufRead, Write};

use orbitx_dynamics::{Elements, GravBody};
use orbitx_math::{cross, dot, mul, StateVectors, Vec3};
use orbitx_vessel::{presets, Assembly, StageSpec};

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
    let h = t / 3600;
    let m = (t % 3600) / 60;
    let s = t % 60;
    format!("T+{h:02}:{m:02}:{s:02}")
}

fn fmt_mass(kg: f64) -> String {
    if kg >= 1000.0 {
        format!("{:.1} t", kg / 1000.0)
    } else {
        format!("{:.0} kg", kg)
    }
}

fn fmt_dist(m: f64) -> String {
    if m.abs() >= 1000.0 {
        format!("{:.1} km", m / 1e3)
    } else {
        format!("{:.0} m", m)
    }
}

struct Sim {
    asm: Assembly,
    met: f64,
    pitch: f64,
    throttle: f64,
    auto_gravity_turn: bool,
    auto_sep: bool,
    rocket_name: String,
}

impl Sim {
    fn new(stages: &[StageSpec], name: &str) -> Self {
        let half_h: f64 = stages.iter().map(|s| s.length).sum::<f64>() / 2.0;
        let radial = LAUNCH_POS * (1.0 / LAUNCH_POS.length());
        let init_pos = LAUNCH_POS + radial * half_h;
        let init_state = StateVectors {
            pos: init_pos,
            ..Default::default()
        };
        let asm = Assembly::new(stages, init_state);
        Sim {
            asm,
            met: 0.0,
            pitch: 0.0,
            throttle: 0.0,
            auto_gravity_turn: false,
            auto_sep: true,
            rocket_name: name.to_string(),
        }
    }

    fn altitude(&self) -> f64 {
        let (pos, _) = self.asm.render_state();
        pos.length() - EARTH_R
    }

    fn velocity(&self) -> Vec3 {
        let v = &self.asm.vessels[self.asm.active];
        v.state.vel
    }

    fn speed(&self) -> f64 {
        self.velocity().length()
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
        let cos_p = self.pitch.cos();
        let sin_p = self.pitch.sin();
        (radial * cos_p + tangent_base * sin_p).unit()
    }

    fn run(&mut self, duration: f64) {
        let earth = GravBody {
            pos: Vec3::ZERO,
            mass: 5.972e24,
            size: EARTH_R,
            jcoeff: vec![],
        };
        let grav = vec![earth];

        let dt_step = 0.1; // 0.1s 步长
        let n_steps = (duration / dt_step).round() as usize;
        for _ in 0..n_steps {
            // 更新俯仰角（重力转向）。
            if self.auto_gravity_turn {
                let h = self.altitude();
                if h > 10_000.0 {
                    let target = ((h - 10_000.0) / 70_000.0).min(1.0) * std::f64::consts::FRAC_PI_2;
                    if self.pitch < target {
                        self.pitch = (self.pitch + 30.0_f64.to_radians() * dt_step).min(target);
                    }
                }
            }

            self.asm.set_throttle(self.throttle);

            // 设置推进器方向为当前推力方向。
            let td = self.thrust_dir();
            let rot = self.asm.vessels[self.asm.active].state.r;
            let td_body = mul(rot, td); // 粗略的世界→体坐标转换
            for v in &mut self.asm.vessels {
                if !v.detached {
                    for t in &mut v.thrusters {
                        t.dir = td_body;
                    }
                }
            }

            // 大气阻力作为额外力加到积分中。
            // Assembly::step 不直接支持额外力，所以我们在 step 后手动修正。
            // 简化：用 Assembly::step 的引力 + 推力，然后手动加阻力。
            let pos_before = self.asm.vessels[self.asm.active].state.pos;
            let vel_before = self.asm.vessels[self.asm.active].state.vel;

            self.asm.step(dt_step, &grav);

            // 阻力修正。
            let h = pos_before.length() - EARTH_R;
            let rho = air_density(h);
            if rho > 1e-10 {
                let v_mag = vel_before.length();
                if v_mag > 1e-3 {
                    let total_mass = self.asm.total_mass();
                    let drag_mag = 0.5 * rho * v_mag * v_mag * DRAG_COEFF;
                    let drag_acc = vel_before * (-drag_mag / (v_mag * total_mass));
                    let dv = drag_acc * dt_step;
                    for v in &mut self.asm.vessels {
                        if !v.detached {
                            v.state.vel += dv;
                        }
                    }
                }
            }

            self.met += dt_step;

            // 自动分离：当前级燃料耗尽。
            if self.auto_sep && self.asm.stage_count() > 1 {
                let active = &self.asm.vessels[self.asm.active];
                if active.fuel_mass < 1.0 && active.thrusters.iter().any(|t| t.max_thrust > 0.0) {
                    self.asm.separate_stage();
                    println!(
                        "  >>> [T+{:.1}s] {} 燃料耗尽，分离！活动级: {}",
                        self.met,
                        self.asm.vessels[self.asm.active - 1].name,
                        self.asm.active_name()
                    );
                }
            }

            // 碰撞。
            let h_now = self.altitude();
            if h_now < 0.0 && self.speed() > 50.0 {
                println!(
                    "  >>> [T+{:.1}s] 坠毁！撞击速度 {:.0} m/s",
                    self.met,
                    self.speed()
                );
                return;
            }
        }
    }

    fn print_status(&self) {
        let h = self.altitude();
        let vel = self.velocity();
        let speed = vel.length();
        let r = self.asm.vessels[self.asm.active].state.pos;
        let r_unit = r * (1.0 / r.length().max(1e-3));
        let v_vert = dot(vel, r_unit);
        let v_horiz = (vel - r_unit * v_vert).length();

        let mass = self.asm.total_mass();
        let fuel = self.asm.total_fuel();
        let fuel_pct = self.asm.fuel_percent();
        let thrust = self.asm.current_thrust();
        let tw = if mass > 0.0 {
            thrust / (mass * G0)
        } else {
            0.0
        };
        let stage_name = self.asm.active_name();
        let stage_count = self.asm.stage_count();

        let bar = "═".repeat(45);
        println!("{bar}");
        println!(" orbitx 发射模拟器 — {rocket}", rocket = self.rocket_name);
        println!(
            " {} | Stage: {stage} ({count} 级剩余)",
            fmt_time(self.met),
            stage = stage_name,
            count = stage_count
        );
        println!("{bar}");

        println!(" 高度 Alt     |  {alt:>10}", alt = fmt_dist(h.max(0.0)));
        println!(" 速度 Vel     |  {v:>7.0} m/s", v = speed);
        println!(" 垂直 Vvert   |  {v:>7.0} m/s", v = v_vert);
        println!(" 水平 Vhoriz  |  {v:>7.0} m/s", v = v_horiz);
        println!(" 质量 Mass    |  {m:>10}", m = fmt_mass(mass));
        println!(
            " 燃料 Fuel    |  {pct:>6.1}%  ({fuel:>10})",
            pct = fuel_pct,
            fuel = fmt_mass(fuel)
        );
        println!(
            " 推力 Thrust  |  {t:>6.0} kN  (T/W: {tw:.1})",
            t = thrust / 1000.0,
            tw = tw
        );
        println!(" 油门 Throttle|  {thr:>7.0}%", thr = self.throttle * 100.0);
        println!(" 俯仰 Pitch   |  {p:>7.1}°", p = self.pitch.to_degrees());

        // 轨道参数。
        let r_mag = r.length();
        let v_circular = (EARTH_GM / r_mag).sqrt();
        let energy = speed * speed / 2.0 - EARTH_GM / r_mag;
        let energy_margin = EARTH_GM / r_mag * 0.01;
        if v_horiz > v_circular * 0.5 && energy < -energy_margin {
            let el = Elements::calculate(r, vel, EARTH_GM, 0.0);
            let ap_alt = (el.ap_dist() - EARTH_R) / 1e3;
            let pe_alt = (el.pe_dist() - EARTH_R) / 1e3;
            if pe_alt > -1000.0 {
                println!(" ApD          |  {ap:>7.0} km", ap = ap_alt);
                println!(" PeD          |  {pe:>7.0} km", pe = pe_alt);
            } else {
                println!(
                    " ApD          |  {ap:>7.0} km (亚轨道 suborbital)",
                    ap = ap_alt
                );
            }
        } else if energy > energy_margin && speed > 100.0 {
            println!(" ApD          |  (逃逸轨道 escape)");
        }

        if self.auto_gravity_turn {
            println!(" [自动重力转向 ON]");
        }
        if self.auto_sep {
            println!(" [自动分离 ON]");
        }

        println!("{bar}");
    }
}

fn print_help() {
    println!("命令：");
    println!("  throttle <0..1>   设置油门");
    println!("  sep               分离当前级");
    println!("  pitch <度>        设置俯仰角");
    println!("  gravity on|off    开关自动重力转向");
    println!("  autosep on|off    开关自动分离");
    println!("  run <秒>          快进 N 秒");
    println!("  status            显示遥测");
    println!("  help              帮助");
    println!("  quit              退出");
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let (stages, name) = if args.len() > 1 && args[1] == "saturnv" {
        (presets::saturn_v(), "Saturn V")
    } else {
        (presets::falcon9(), "Falcon 9")
    };

    let mut sim = Sim::new(&stages, name);

    println!("═══════════════════════════════════════════");
    println!(" orbitx 终端火箭发射模拟器");
    println!(" 火箭: {} ({} 级)", sim.rocket_name, sim.asm.stage_count());
    println!(" 输入 help 查看命令，throttle 1.0 + run 180 开始发射");
    println!("═══════════════════════════════════════════");
    sim.print_status();

    let stdin = io::stdin();
    let mut stdout = io::stdout();

    loop {
        print!("\n> ");
        stdout.flush().unwrap();

        let mut line = String::new();
        if stdin.lock().read_line(&mut line).unwrap() == 0 {
            break;
        }
        let line = line.trim();
        if line.is_empty() {
            continue;
        }

        let parts: Vec<&str> = line.split_whitespace().collect();
        match parts[0] {
            "quit" | "exit" | "q" => break,
            "help" | "h" | "?" => print_help(),
            "status" | "s" => sim.print_status(),
            "throttle" | "th" => {
                if parts.len() < 2 {
                    println!("用法: throttle <0..1>");
                } else {
                    match parts[1].parse::<f64>() {
                        Ok(v) => {
                            sim.throttle = v.clamp(0.0, 1.0);
                            println!("油门设置为 {:.0}%", sim.throttle * 100.0);
                        }
                        Err(_) => println!("无效数值"),
                    }
                }
            }
            "sep" | "separate" => {
                if sim.asm.stage_count() > 1 {
                    let old = sim.asm.active_name().to_string();
                    sim.asm.separate_stage();
                    println!("分离 {}！活动级: {}", old, sim.asm.active_name());
                } else {
                    println!("仅剩 1 级，无法分离");
                }
            }
            "pitch" => {
                if parts.len() < 2 {
                    println!("用法: pitch <度>");
                } else {
                    match parts[1].parse::<f64>() {
                        Ok(deg) => {
                            sim.pitch = deg.to_radians().clamp(0.0, std::f64::consts::FRAC_PI_2);
                            println!("俯仰角设置为 {:.0}°", sim.pitch.to_degrees());
                        }
                        Err(_) => println!("无效数值"),
                    }
                }
            }
            "gravity" => {
                if parts.len() >= 2 {
                    sim.auto_gravity_turn = parts[1] == "on" || parts[1] == "true";
                    println!(
                        "自动重力转向: {}",
                        if sim.auto_gravity_turn { "ON" } else { "OFF" }
                    );
                } else {
                    sim.auto_gravity_turn = !sim.auto_gravity_turn;
                    println!(
                        "自动重力转向: {}",
                        if sim.auto_gravity_turn { "ON" } else { "OFF" }
                    );
                }
            }
            "autosep" => {
                if parts.len() >= 2 {
                    sim.auto_sep = parts[1] == "on" || parts[1] == "true";
                } else {
                    sim.auto_sep = !sim.auto_sep;
                }
                println!("自动分离: {}", if sim.auto_sep { "ON" } else { "OFF" });
            }
            "run" | "r" => {
                let secs: f64 = if parts.len() >= 2 {
                    parts[1].parse().unwrap_or(10.0)
                } else {
                    10.0
                };
                println!("快进 {secs} 秒...");
                sim.run(secs);
                sim.print_status();
            }
            "launch" => {
                sim.throttle = 1.0;
                sim.auto_gravity_turn = true;
                println!("点火！油门 100%，自动重力转向 ON");
                sim.run(180.0);
                sim.print_status();
            }
            _ => println!("未知命令: {}。输入 help 查看。", parts[0]),
        }
    }

    println!("退出。");
}
