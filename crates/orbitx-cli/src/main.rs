//! orbitx-cli：spawn `orbitx-runtime` + 本机 Zenoh TUI（不持有/步进 Assembly）。
//!
//! 用法：
//!   cargo run -p orbitx-cli -- --rocket falcon9
//!   cargo run -p orbitx-cli -- --rocket falcon9 --smoke 5

use std::path::PathBuf;
use std::time::{Duration, Instant};

use clap::{Parser, ValueEnum};
use crossterm::event::{self, Event, KeyCode, KeyEventKind, KeyModifiers};
use orbitx_cli::auto_sep::should_auto_separate;
use orbitx_cli::focus::SliceFocus;
use orbitx_cli::spawn::{find_runtime_exe, RuntimeChild};
use orbitx_cli::ui;
use orbitx_cli::zenoh_client::ZenohClient;
use orbitx_protocol::{decode_slice, Slice};
use ratatui::DefaultTerminal;

#[derive(Debug, Clone, Parser)]
#[command(name = "orbitx-cli", about = "Orbitx Zenoh TUI client (spawns orbitx-runtime)")]
struct CliArgs {
    /// 火箭类：内置别名或 rocket.toml 路径。
    #[arg(long, default_value = "falcon9")]
    rocket: String,

    /// 可选 scenario.toml。
    #[arg(long)]
    scenario: Option<PathBuf>,

    /// 控制模式（本阶段仅 Manual/target；不启 WorkFlow）。
    #[arg(long, value_enum, default_value_t = ControlArg::Target)]
    control: ControlArg,

    /// 本机 Zenoh 会话 id（默认 local）。
    #[arg(long = "zenoh-endpoint", default_value = "local")]
    zenoh_endpoint: String,

    /// 固定物理步长 [ms]（传给 runtime）。
    #[arg(long, default_value_t = 20)]
    sim_dt: u64,

    /// 历表数据根目录。
    #[arg(long)]
    ephemeris_data: Option<PathBuf>,

    /// 日志目录（传给 runtime）。
    #[arg(long, default_value = "./logs")]
    log_dir: PathBuf,

    /// 黑匣子目录（传给 runtime）。
    #[arg(long, default_value = "./flight_records")]
    recorder_dir: PathBuf,

    /// 无头跑 N 仿真秒后退出（自动化）。
    #[arg(long, value_name = "SECS")]
    smoke: Option<f64>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
enum ControlArg {
    Base,
    Target,
}

fn validate_zenoh_endpoint(endpoint: &str) -> Result<(), String> {
    let ep = endpoint.trim();
    if ep.is_empty() {
        return Err("--zenoh-endpoint must not be empty".into());
    }
    if ep.eq_ignore_ascii_case("local") {
        return Ok(());
    }
    let lower = ep.to_ascii_lowercase();
    if let Some(rest) = lower.strip_prefix("tcp/") {
        return validate_loopback(rest, ep);
    }
    if let Some(rest) = lower.strip_prefix("udp/") {
        return validate_loopback(rest, ep);
    }
    // 会话 id（哈希到本机端口）允许。
    if !lower.contains('/') {
        return Ok(());
    }
    Err(format!(
        "unsupported --zenoh-endpoint `{ep}`: local-SHM only"
    ))
}

fn validate_loopback(host_port: &str, original: &str) -> Result<(), String> {
    let host = host_port
        .rsplit_once(':')
        .map(|(h, _)| h)
        .unwrap_or(host_port)
        .trim_matches(|c| c == '[' || c == ']');
    if host == "127.0.0.1" || host == "::1" || host.eq_ignore_ascii_case("localhost") {
        Ok(())
    } else {
        Err(format!(
            "cross-device Zenoh refused: `{original}` (host `{host}`)"
        ))
    }
}

fn runtime_argv(args: &CliArgs) -> Vec<String> {
    let mut v = vec![
        "--rocket".into(),
        args.rocket.clone(),
        "--control".into(),
        match args.control {
            ControlArg::Base => "base".into(),
            ControlArg::Target => "target".into(),
        },
        "--zenoh-endpoint".into(),
        args.zenoh_endpoint.clone(),
        "--sim-dt".into(),
        args.sim_dt.to_string(),
        "--drive".into(),
        "self-paced".into(),
        "--log-dir".into(),
        args.log_dir.display().to_string(),
        "--recorder-dir".into(),
        args.recorder_dir.display().to_string(),
    ];
    if let Some(sc) = &args.scenario {
        v.push("--scenario".into());
        v.push(sc.display().to_string());
    }
    if let Some(ep) = &args.ephemeris_data {
        v.push("--ephemeris-data".into());
        v.push(ep.display().to_string());
    }
    v
}

struct AppState {
    slice: Slice,
    /// 本地油门指令镜像（冷启动 0；W 拉满）。
    throttle: f64,
    pitch_target: f64,
    yaw_target: f64,
    gravity_turn: bool,
    /// 纯本地 UI：Primary | Detached{list_idx → slice.detached}。
    focus: SliceFocus,
    exit: bool,
    last_auto_sep_step: u64,
}

impl AppState {
    fn new() -> Self {
        Self {
            slice: Slice::default(),
            throttle: 0.0,
            pitch_target: 0.0,
            yaw_target: 0.0,
            gravity_turn: false,
            focus: SliceFocus::Primary,
            exit: false,
            last_auto_sep_step: 0,
        }
    }

    fn controls_enabled(&self) -> bool {
        self.focus.controls_enabled() && self.slice.crash_msg.is_empty()
    }

    fn cycle_focus(&mut self) {
        self.focus.cycle(self.slice.detached.len());
    }
}

async fn drain_slices(
    sub: &zenoh::pubsub::Subscriber<zenoh::handlers::FifoChannelHandler<zenoh::sample::Sample>>,
    state: &mut AppState,
) {
    while let Ok(Some(sample)) = sub.try_recv() {
        let bytes = sample.payload().to_bytes();
        if let Ok(s) = decode_slice(&bytes) {
            state.slice = s;
            state.focus.clamp(state.slice.detached.len());
            state.gravity_turn = state.slice.gravity_turn;
            if state.slice.pitch_target.is_finite() {
                state.pitch_target = state.slice.pitch_target;
            }
            if state.slice.yaw_target.is_finite() {
                state.yaw_target = state.slice.yaw_target;
            }
        }
    }
}

async fn maybe_auto_separate(client: &ZenohClient, state: &mut AppState) {
    if !state.controls_enabled() {
        return;
    }
    if state.slice.step_index == state.last_auto_sep_step {
        return;
    }
    if should_auto_separate(&state.slice) {
        let _ = client.separate().await;
        state.last_auto_sep_step = state.slice.step_index;
    }
}

async fn handle_key(
    client: &ZenohClient,
    state: &mut AppState,
    key: KeyCode,
    modifiers: KeyModifiers,
) {
    if !state.slice.crash_msg.is_empty() {
        match key {
            KeyCode::Char('r') => {
                let _ = client.reset().await;
                state.throttle = 0.0;
                state.focus = SliceFocus::Primary;
            }
            KeyCode::Char('q') | KeyCode::Esc => state.exit = true,
            KeyCode::Char('c') if modifiers.contains(KeyModifiers::CONTROL) => state.exit = true,
            _ => {}
        }
        return;
    }

    match key {
        KeyCode::Char('q') | KeyCode::Esc => {
            state.exit = true;
            return;
        }
        KeyCode::Char('c') if modifiers.contains(KeyModifiers::CONTROL) => {
            state.exit = true;
            return;
        }
        KeyCode::Char('c') | KeyCode::Char('C') => {
            state.cycle_focus();
            return;
        }
        KeyCode::Char(' ') => {
            if state.slice.paused {
                let _ = client.resume().await;
            } else {
                let _ = client.pause().await;
            }
            return;
        }
        KeyCode::Char('r') => {
            let _ = client.reset().await;
            state.throttle = 0.0;
            state.focus = SliceFocus::Primary;
            return;
        }
        KeyCode::Char('+') | KeyCode::Char('=') => {
            let w = (state.slice.warp * 2.0).max(1.0);
            let _ = client.set_warp(w).await;
            return;
        }
        KeyCode::Char('-') => {
            let w = (state.slice.warp / 2.0).max(0.125);
            let _ = client.set_warp(w).await;
            return;
        }
        _ => {}
    }

    if !state.controls_enabled() {
        return;
    }

    match key {
        KeyCode::Char('w') => {
            if state.throttle < 1e-6 {
                state.throttle = 1.0;
            } else {
                state.throttle = 0.0;
            }
            let _ = client.set_throttle(state.throttle).await;
        }
        KeyCode::Char('s') => {
            let _ = client.separate().await;
        }
        KeyCode::Up => {
            state.throttle = (state.throttle + 0.1).min(1.0);
            let _ = client.set_throttle(state.throttle).await;
        }
        KeyCode::Down => {
            state.throttle = (state.throttle - 0.1).max(0.0);
            let _ = client.set_throttle(state.throttle).await;
        }
        KeyCode::Left => {
            state.pitch_target = (state.pitch_target - 1.0_f64.to_radians()).max(0.0);
            state.gravity_turn = false;
            let _ = client
                .set_attitude(state.pitch_target, state.yaw_target)
                .await;
        }
        KeyCode::Right => {
            state.pitch_target = (state.pitch_target + 1.0_f64.to_radians())
                .min(std::f64::consts::FRAC_PI_2);
            state.gravity_turn = false;
            let _ = client
                .set_attitude(state.pitch_target, state.yaw_target)
                .await;
        }
        KeyCode::Char('g') => {
            state.gravity_turn = !state.gravity_turn;
            let _ = client.set_gravity_turn(state.gravity_turn).await;
        }
        _ => {}
    }
}

async fn run_tui(client: &ZenohClient, state: &mut AppState) -> std::io::Result<()> {
    let mut terminal = ratatui::init();
    let result = run_loop(client, state, &mut terminal).await;
    ratatui::restore();
    result
}

async fn run_loop(
    client: &ZenohClient,
    state: &mut AppState,
    terminal: &mut DefaultTerminal,
) -> std::io::Result<()> {
    let sub = client
        .declare_slice_sub()
        .await
        .map_err(|e| std::io::Error::other(e))?;

    while !state.exit {
        drain_slices(&sub, state).await;
        maybe_auto_separate(client, state).await;

        terminal.draw(|frame| ui::draw(frame, &state.slice, state.focus, state.throttle))?;

        if event::poll(Duration::from_millis(16))? {
            if let Event::Key(key) = event::read()? {
                if key.kind == KeyEventKind::Press {
                    handle_key(client, state, key.code, key.modifiers).await;
                }
            }
        }
    }
    Ok(())
}

async fn run_smoke(client: &ZenohClient, state: &mut AppState, secs: f64) -> Result<(), String> {
    let sub = client.declare_slice_sub().await?;
    let deadline = Instant::now() + Duration::from_secs_f64(secs.max(0.1) + 2.0);
    // 自动点火以便 smoke 有运动。
    state.throttle = 1.0;
    client.set_throttle(1.0).await?;

    loop {
        drain_slices(&sub, state).await;
        maybe_auto_separate(client, state).await;
        let sim_s = state.slice.sim_t as f64 / 1000.0;
        if sim_s >= secs {
            eprintln!(
                "smoke ok: sim_t={sim_s:.2}s alt={:.1}m launched={} stages={}",
                state
                    .slice
                    .focus
                    .as_ref()
                    .map(|f| f.altitude)
                    .unwrap_or(0.0),
                state.slice.launched,
                state.slice.stages.len()
            );
            return Ok(());
        }
        if Instant::now() > deadline {
            return Err(format!(
                "smoke timeout waiting for {secs}s sim (got {sim_s:.2}s)"
            ));
        }
        tokio::time::sleep(Duration::from_millis(20)).await;
    }
}

#[tokio::main]
async fn main() {
    let args = CliArgs::parse();
    if let Err(e) = validate_zenoh_endpoint(&args.zenoh_endpoint) {
        eprintln!("{e}");
        std::process::exit(2);
    }

    let exe = match find_runtime_exe() {
        Ok(p) => p,
        Err(e) => {
            eprintln!("{e}");
            std::process::exit(1);
        }
    };
    let argv = runtime_argv(&args);
    eprintln!("spawn {} {}", exe.display(), argv.join(" "));
    let mut child = match RuntimeChild::spawn(&exe, &argv) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("{e}");
            std::process::exit(1);
        }
    };

    // 等 runtime 监听。
    tokio::time::sleep(Duration::from_millis(400)).await;

    let client = match ZenohClient::connect(&args.zenoh_endpoint).await {
        Ok(c) => c,
        Err(e) => {
            eprintln!("zenoh connect failed: {e}");
            child.kill_and_wait();
            std::process::exit(1);
        }
    };

    let mut state = AppState::new();
    let run_result = if let Some(secs) = args.smoke {
        run_smoke(&client, &mut state, secs)
            .await
            .map_err(|e| std::io::Error::other(e))
    } else {
        run_tui(&client, &mut state).await
    };

    let _ = client.shutdown().await;
    tokio::time::sleep(Duration::from_millis(100)).await;
    child.kill_and_wait();

    if let Err(e) = run_result {
        eprintln!("cli error: {e}");
        std::process::exit(1);
    }
}
