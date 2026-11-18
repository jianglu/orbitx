//! `wf_probe`：无渲染、无 zenoh 的快速飞行探针（开发期参数标定工具）。
//!
//! 把「一次飞行」压到秒级：直接构建 SimBundle 并以 20 ms 固定步进循环 tick，
//! 不走 Runtime 线程 / zenoh / 切片通道 / 地表包渲染，也不写黑匣子。
//!
//! 用法：
//! ```text
//! cargo run -p orbitx-runtime --example wf_probe -- \
//!     --rocket <航天器目录>/sim.toml \
//!     --workflow <wf toml 绝对路径> \
//!     [--sim-secs 900] [--every 2] [--launchpad <lp toml>]
//! ```
//!
//! 输出：每秒一行关键遥测；终局打印关机点五要素 + 轨道要素；若给了
//! `--revolutions N` 则额外镜像 DLC CompletionJudge 的幅角累积判成后停机。
//!
//! 与产品链路的差异（仅限本工具，勿照搬）：
//! - 不跑 `pad` 之外的 RuntimeService，故没有 Step 门禁 / 切片限流；
//! - 记录器关闭（`recorder_enabled=false`），无黑匣子写入；
//! - 地表走光滑球（不加载 surface package）。

use std::path::PathBuf;
use std::time::Instant;

use clap::Parser;
use orbitx_controller::capability::ControlCapability;
use orbitx_math::{dot, Elements, Vec3, GGRAV};
use orbitx_protocol::mem::Slice;
use orbitx_runtime::cli::{load_session, SessionSpec};
use orbitx_runtime::input::InputCmd;
use orbitx_runtime::runtime::clock::Clock;
use orbitx_runtime::runtime::separation_queue::SeparationQueue;
use orbitx_runtime::runtime::tick::{tick, TickOutcome};
use orbitx_runtime::session::{build_sim_bundle, SimBundle};
use orbitx_vessel::{geocentric_lat_lon_deg_orient, surface_inertial_velocity};

#[derive(Debug, Parser)]
#[command(name = "wf_probe", about = "Headless workflow flight probe (no render / no zenoh)")]
struct Args {
    /// 航天器 `sim.toml` 路径（或内置别名）。
    #[arg(long, default_value = "falcon9")]
    rocket: String,
    /// 工作流 TOML 绝对路径（图 IR；DLC 叠加文件会自动拼接基础层）。
    #[arg(long)]
    workflow: PathBuf,
    /// 仿真上限 [s]。
    #[arg(long, default_value_t = 900.0)]
    sim_secs: f64,
    /// 打印间隔 [s]（仿真钟）。
    #[arg(long, default_value_t = 2.0)]
    every: f64,
    /// 发射场 lp toml（缺省工程默认 lp_default）。
    #[arg(long)]
    launchpad: Option<PathBuf>,
    /// 完成判定圈数（>0 = 叠加程序镜像 DLC CompletionJudge：关机后无动力累积
    /// 幅角达 N×360° 即判成并停机，用于任务二验证）。
    #[arg(long, default_value_t = 0.0)]
    revolutions: f64,
    /// 环境别名（缺省 earth）。
    #[arg(long, default_value = "earth")]
    scenario: String,
}

const SIM_DT_MS: u64 = 20;
/// CompletionJudge 推力阈（与 DLC 同源）。
const THRUST_EPS: f64 = 0.02;
/// CompletionJudge 单帧幅角缺口阈 [deg]。
const GAP_DEG: f64 = 90.0;

struct Judge {
    revolutions: f64,
    burning: bool,
    armed: bool,
    anchored: bool,
    last_u: f64,
    accum: f64,
    done: bool,
}

impl Judge {
    fn disabled() -> Self {
        Self {
            revolutions: 0.0,
            burning: false,
            armed: false,
            anchored: false,
            last_u: 0.0,
            accum: 0.0,
            done: true,
        }
    }

    fn new(revolutions: f64) -> Self {
        Self {
            revolutions,
            burning: false,
            armed: false,
            anchored: false,
            last_u: 0.0,
            accum: 0.0,
            done: revolutions <= 0.0,
        }
    }

    /// 返回 true = 本帧判成。
    fn feed(&mut self, firing: bool, throttle: f64, arg_lat_deg: f64) -> bool {
        if self.done {
            return false;
        }
        let thrusting = firing || throttle > THRUST_EPS;
        if thrusting && !self.burning {
            self.armed = true;
            self.anchored = false;
            self.accum = 0.0;
        } else if !thrusting && self.burning && self.armed {
            self.anchored = true;
            self.last_u = arg_lat_deg;
            self.accum = 0.0;
        }
        self.burning = thrusting;
        if !self.anchored {
            return false;
        }
        let mut d = arg_lat_deg - self.last_u;
        while d > 180.0 {
            d -= 360.0;
        }
        while d < -180.0 {
            d += 360.0;
        }
        if d.abs() > GAP_DEG {
            return false;
        }
        self.last_u = arg_lat_deg;
        self.accum += d;
        if self.accum.abs() >= self.revolutions * 360.0 {
            self.done = true;
            return true;
        }
        false
    }
}

fn main() {
    let args = Args::parse();
    let start = Instant::now();

    let spec = SessionSpec {
        rocket: Some(args.rocket.clone()),
        scenario: Some(args.scenario.clone()),
        scenario_toml: None,
        launchpad: Some(args.launchpad.clone().unwrap_or_else(default_launchpad)),
        launchpad_toml: None,
        pad: None,
        heading_deg: None,
        mjd: None,
        surface_package: None,
        surface_working_max_level: None,
        control: None,
        workflow: Some(args.workflow.clone()),
        session_context: Default::default(),
        recorder_schema_expected: None,
    };
    let ephemeris = orbitx_environment::resolve_ephemeris_data(None);
    let session = match load_session(&spec, &ephemeris) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("[wf_probe] 会话加载失败：{e}");
            std::process::exit(2);
        }
    };
    let mut sim = match build_sim_bundle(&session, &ephemeris) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("[wf_probe] 会话构建失败：{e}");
            std::process::exit(2);
        }
    };

    // 点火（图编排入口；线性工作流为 no-op）。
    sim.control.apply_input(&mut sim.asm, InputCmd::WorkflowIgnite);

    let mut clock = Clock::new(SIM_DT_MS);
    let mut queue = SeparationQueue::new();
    let mut judge = Judge::new(args.revolutions);
    if args.revolutions <= 0.0 {
        judge = Judge::disabled();
    }

    println!(
        "[wf_probe] rocket={} dt={}ms sim_limit={}s judge_rev={}",
        args.rocket, SIM_DT_MS, args.sim_secs, args.revolutions
    );
    println!(
        "{:>8} {:>10} {:>10} {:>9} {:>9} {:>10} {:>10} {:>9} {:>8} {:>9} {:>9} {:>8} {:>8}",
        "sim_t", "alt_km", "drange_km", "speed", "v_vert", "ap_km", "pe_km", "gamma_deg",
        "tip_deg", "thr", "mass_t", "T_kN", "roll_deg"
    );

    let start_pos = sim.asm.state.pos;
    let pad_dir = start_pos.unit();
    let mut next_report = 0.0_f64;
    let mut halted_printed = false;
    let mut terminated = false;

    loop {
        let sim_t = clock.sim_t_secs();
        if sim_t > args.sim_secs {
            println!("[wf_probe] 达仿真上限 {}s，停止", args.sim_secs);
            break;
        }
        let outcome = tick(&mut clock, &mut sim, true, &mut queue);
        let slice = match outcome {
            TickOutcome::Stepped { slice, .. } => slice,
            // Skipped（暂停 / 停机终态）：仍要用上一步切片喂完成判定与打印，
            // 否则「仅停控制」的 halt 之后循环空转、判定永不推进。
            TickOutcome::Skipped => {
                if sim.control.halt_action().is_none() {
                    continue;
                }
                let s = sim.last_slice.clone();
                if !s.ready {
                    continue;
                }
                s
            }
        };

        if slice.sim_t as f64 / 1000.0 >= next_report {
            next_report += args.every;
            report_line(&sim, &slice, &pad_dir);
        }

        // 图内 halt（WorkFlow 图停机指令）首次锁存。
        if !halted_printed {
            if let Some(action) = sim.control.halt_action() {
                halted_printed = true;
                println!(
                    "[wf_probe] HALT @ {:.1}s  shutdown_controls={} stop_stepping={} save_recorder={}",
                    sim_t, action.shutdown_controls, action.stop_stepping, action.save_recorder
                );
                if action.stop_stepping {
                    dump_cutoff(&sim);
                    terminated = true;
                    break;
                }
            }
        }

        // 任务二镜像判定（DLC CompletionJudge）。
        if !judge.done {
            let (firing, throttle) = stages_thrust(&slice);
            let u = slice.orbit.arg_latitude_deg;
            if judge.feed(firing, throttle, u) {
                println!(
                    "[wf_probe] COMPLETION 判成 @ {:.1}s（累积 {:.1}° / 目标 {:.0}°）",
                    sim_t,
                    judge.accum,
                    args.revolutions * 360.0
                );
                dump_cutoff(&sim);
                terminated = true;
                break;
            }
        }

        if !sim.crash_msg.is_empty() {
            println!("[wf_probe] CRASH：{}", sim.crash_msg);
            dump_cutoff(&sim);
            terminated = true;
            break;
        }
        if !sim.launch_failed_msg.is_empty() {
            println!("[wf_probe] LAUNCH FAILED：{}", sim.launch_failed_msg);
            terminated = true;
            break;
        }
    }

    if !terminated {
        println!("[wf_probe] 未达终局（检查程序是否收敛）");
        dump_cutoff(&sim);
    }
    println!(
        "[wf_probe] 用时 {:.1}s wall（仿真 {:.1}s）",
        start.elapsed().as_secs_f64(),
        clock.sim_t_secs()
    );
}

fn default_launchpad() -> PathBuf {
    let candidates = [
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../../sim-rocket/launchpads/lp_default.toml"),
        PathBuf::from("../sim-rocket/launchpads/lp_default.toml"),
    ];
    for p in candidates {
        if p.is_file() {
            return p;
        }
    }
    PathBuf::from("launchpads/lp_default.toml")
}

fn stages_thrust(slice: &Slice) -> (bool, f64) {
    let mut firing = false;
    let mut throttle = 0.0_f64;
    for st in &slice.stages {
        firing |= st.firing;
        throttle = throttle.max(st.throttle);
    }
    (firing, throttle)
}

fn report_line(sim: &SimBundle, slice: &Slice, pad_dir: &Vec3) {
    let pos = slice.focus.pos;
    let vel = slice.focus.vel;
    let p = Vec3::new(pos[0], pos[1], pos[2]);
    let v = Vec3::new(vel[0], vel[1], vel[2]);
    let r = p.length().max(1e-9);
    let up = p * (1.0 / r);
    // 航程（沿发射台当地水平的角距 × R）。
    let cos_d = dot(*pad_dir, up).clamp(-1.0, 1.0);
    let downrange = cos_d.acos() * sim.earth_radius;
    let vrad = dot(v, up);
    let gamma = if v.length() > 1e-9 {
        (vrad / v.length()).clamp(-1.0, 1.0).asin().to_degrees()
    } else {
        0.0
    };
    println!(
        "{:>8.1} {:>10.1} {:>10.1} {:>9.1} {:>9.1} {:>10.1} {:>10.1} {:>8.2} {:>7.2} {:>9.2} {:>9.2} {:>8.0} {:>8.1}",
        slice.sim_t as f64 / 1000.0,
        slice.focus.altitude / 1000.0,
        downrange / 1000.0,
        slice.focus.speed,
        slice.focus.v_vert,
        slice.orbit.ap_alt / 1000.0,
        slice.orbit.pe_alt / 1000.0,
        gamma,
        slice.focus.tip.to_degrees(),
        slice.focus.throttle,
        slice.focus.mass / 1000.0,
        slice.focus.thrust / 1000.0,
        slice.focus.roll.to_degrees(),
    );
    // 诊断：体 +Y 与径向夹角（与 focus.tip 对照；rot 行主序，列 1 = 体 +Y）
    let r = slice.focus.rot;
    if r.len() == 9 && (slice.sim_t as f64 / 1000.0) < 3.0 {
        let by = Vec3::new(r[1], r[4], r[7]);      // 第 1 列 = 体 +Y
        let ang = dot(by, up).clamp(-1.0, 1.0).acos();
        println!(
            "    [diag] t={:.1}s rot=[{:.4},{:.4},{:.4} | {:.4},{:.4},{:.4} | {:.4},{:.4},{:.4}] \
             by=({:.3},{:.3},{:.3}) up=({:.3},{:.3},{:.3}) by·up={:.4} ang={:.2}° tip_raw={:.6}rad",
            slice.sim_t as f64 / 1000.0,
            r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8],
            by.x, by.y, by.z, up.x, up.y, up.z,
            dot(by, up), ang.to_degrees(), slice.focus.tip
        );
    }
}

fn dump_cutoff(sim: &SimBundle) {
    let asm = &sim.asm;
    let vi = asm.active.min(asm.vessels.len().saturating_sub(1));
    let pos = asm.vessels[vi].state.pos;
    let vel = asm.vessels[vi].state.vel;
    let r = pos.length().max(1e-3);
    let up = pos * (1.0 / r);
    let speed = vel.length();
    let gamma = if speed > 1e-9 {
        (dot(vel, up) / speed).clamp(-1.0, 1.0).asin().to_degrees()
    } else {
        0.0
    };
    let (lat, lon) = geocentric_lat_lon_deg_orient(pos, asm.planet_orientation.rot);
    let mu = GGRAV * sim.world.psys.primary_surface().mass;
    let el = Elements::calculate(pos, vel, mu, 0.0);
    let (pe_r, ap_r) = if el.e < 1.0 {
        (el.a * (1.0 - el.e), el.a * (1.0 + el.e))
    } else {
        (0.0, f64::INFINITY)
    };
    let corot = surface_inertial_velocity(pos, asm.planet_orientation.omega);
    println!("--- 关机点 / 终局要素 ---");
    println!(
        "  lat={:.4}° lon={:.4}° alt={:.2} km",
        lat,
        lon,
        (r - sim.earth_radius) / 1000.0
    );
    println!(
        "  speed={:.1} m/s（对地 {:.1}） gamma={:.3}°",
        speed,
        (vel - corot).length(),
        gamma
    );
    println!(
        "  pe={:.1} km ap={:.1} km e={:.5} inc={:.3}° raan={:.2}° argp={:.2}° u={:.2}°",
        (pe_r - sim.earth_radius) / 1000.0,
        if ap_r.is_finite() {
            (ap_r - sim.earth_radius) / 1000.0
        } else {
            f64::INFINITY
        },
        el.e,
        el.i.to_degrees(),
        el.theta.to_degrees(),
        el.omegab.to_degrees(),
        el.arg_latitude().to_degrees(),
    );
    let caps = ControlCapability::for_primary(asm);
    let _ = caps; // 保活：确保投影链无 panic（探针只做诊断）
    println!("  级燃料：");
    for (i, v) in asm.vessels.iter().enumerate() {
        println!(
            "    [{}] {} fuel={:.0} kg detached={}",
            i,
            v.name,
            v.fuel_mass(),
            v.detached
        );
    }
}
