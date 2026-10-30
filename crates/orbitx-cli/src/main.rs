//! orbitx-cli：spawn `orbitx-runtime` + 本机 Zenoh TUI（不持有/步进 Assembly）。
//!
//! 用法：
//!   cargo run -p orbitx-cli -- --rocket falcon9
//!   cargo run -p orbitx-cli -- --rocket falcon9 --smoke 5
//!   cargo run -p orbitx-cli -- --rocket falcon9 --workflow path/to/wf_target.toml
//!   cargo run -p orbitx-cli -- --rocket "$HOME/Library/Application Support/WLCY/SimRocket/spacecraft/sc_<class>/sim.toml"

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
    /// 火箭类：内置别名或 sim.toml / rocket.toml 路径（`~/` 会展开）。
    #[arg(long, default_value = "falcon9")]
    rocket: String,

    /// 环境：别名 `earth` 或 scenario_xxx.toml 路径。
    #[arg(long, default_value = "earth")]
    scenario: String,

    /// 控制模式 a/b（与 `--workflow` 互斥；二者皆省略时默认 target）。
    #[arg(long, value_enum)]
    control: Option<ControlArg>,

    /// 工作流 TOML 路径（线性 phases/steps 或图 IR；与 `--control` 互斥，透传 runtime）。
    #[arg(long, value_name = "PATH")]
    workflow: Option<PathBuf>,

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

    /// 是否启用黑匣子（透传 runtime `--recorder-enabled`；兼容 flag 与显式值）。
    #[arg(
        long = "recorder-enabled",
        action = clap::ArgAction::Set,
        num_args = 0..=1,
        default_value_t = false,
        default_missing_value = "true",
        value_name = "BOOL"
    )]
    recorder_enabled: bool,

    /// 黑匣子目录（传给 runtime；仅 `--recorder-enabled` 时附加）。
    #[arg(long, default_value = "./flight_records")]
    recorder_dir: PathBuf,

    /// 共用地表包根目录（透传 runtime；省略且 scenario=earth 时尝试默认 low 包）。
    #[arg(long = "surface-package", value_name = "DIR")]
    surface_package: Option<PathBuf>,

    /// 地表工作集最大四叉树 level（透传 runtime；有包时才附加）。
    #[arg(long = "surface-working-max-level")]
    surface_working_max_level: Option<u32>,

    /// 发射场 `lp_xxx.toml`（透传 runtime；缺省默认单源 sim-rocket/launchpads/lp_default.toml）。
    #[arg(long = "launchpad", value_name = "PATH")]
    launchpad: Option<PathBuf>,

    /// 发射场台位名（透传 runtime；缺省 = lp 第一个台位）。
    #[arg(long = "pad", value_name = "NAME")]
    pad: Option<String>,

    /// 覆盖台位朝向（度；透传 runtime）。
    #[arg(long = "heading-deg", value_name = "DEG")]
    heading_deg: Option<f64>,

    /// 起始历元覆盖（MJD；透传 runtime `--mjd`）。
    #[arg(long = "mjd", value_name = "MJD")]
    mjd: Option<f64>,

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

/// `--control` / `--workflow` 互斥与路径预检（对齐 runtime 文案）。
fn validate_control_args(args: &CliArgs) -> Result<(), String> {
    match (&args.control, &args.workflow) {
        (Some(_), Some(_)) => Err("--control 与 --workflow 互斥，请只指定其一".into()),
        (None, Some(path)) => {
            if !path.exists() {
                return Err(format!("--workflow 文件不存在：{}", path.display()));
            }
            Ok(())
        }
        (Some(_), None) | (None, None) => Ok(()),
    }
}

/// 发射场：显式 `--launchpad`，否则默认 sim-rocket 单源 lp_default.toml（默认发射场，场区为海南文昌）。
fn resolve_launchpad(args: &CliArgs) -> Result<PathBuf, String> {
    if let Some(ref p) = args.launchpad {
        return Ok(p.clone());
    }
    let candidates = [
        // workspace 布局：orbitx/crates/orbitx-cli → SimRocket/sim-rocket（单源）
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../../sim-rocket/launchpads/lp_default.toml"),
        // cwd = SimRocket / orbitx / 导出旁路目录
        PathBuf::from("sim-rocket/launchpads/lp_default.toml"),
        PathBuf::from("../sim-rocket/launchpads/lp_default.toml"),
        PathBuf::from("launchpads/lp_default.toml"),
    ];
    for p in candidates {
        if p.is_file() {
            return Ok(p.canonicalize().unwrap_or(p));
        }
    }
    Err(
        "找不到默认发射场 lp_default.toml（sim-rocket/launchpads/ 单源）；请显式传 --launchpad <lp_xxx.toml>"
            .into(),
    )
}

fn runtime_argv(args: &CliArgs) -> Result<Vec<String>, String> {
    let ephemeris = args
        .ephemeris_data
        .clone()
        .unwrap_or_else(|| orbitx_environment::resolve_ephemeris_data(None));
    let mut argv = vec![
        "--rocket".into(),
        orbitx_config::expand_rocket_spec(&args.rocket)
            .display()
            .to_string(),
        "--scenario".into(),
        args.scenario.clone(),
    ];
    if let Some(path) = &args.workflow {
        argv.push("--workflow".into());
        argv.push(path.display().to_string());
    } else {
        let control = args.control.unwrap_or(ControlArg::Target);
        argv.push("--control".into());
        argv.push(match control {
            ControlArg::Base => "base".into(),
            ControlArg::Target => "target".into(),
        });
    }
    argv.extend([
        "--zenoh-endpoint".into(),
        args.zenoh_endpoint.clone(),
        "--sim-dt".into(),
        args.sim_dt.to_string(),
        "--drive".into(),
        "self-paced".into(),
        "--log-dir".into(),
        args.log_dir.display().to_string(),
        "--ephemeris-data".into(),
        ephemeris.display().to_string(),
    ]);
    argv.push("--recorder-enabled".into());
    argv.push(args.recorder_enabled.to_string());
    if args.recorder_enabled {
        argv.push("--recorder-dir".into());
        argv.push(args.recorder_dir.display().to_string());
    }
    if let Some(pkg) = resolve_surface_package(args) {
        argv.push("--surface-package".into());
        argv.push(pkg.display().to_string());
        if let Some(lv) = args.surface_working_max_level {
            argv.push("--surface-working-max-level".into());
            argv.push(lv.to_string());
        }
    }
    let lp = resolve_launchpad(args)?;
    argv.push("--launchpad".into());
    argv.push(lp.display().to_string());
    if let Some(pad) = &args.pad {
        argv.push("--pad".into());
        argv.push(pad.clone());
    }
    if let Some(h) = args.heading_deg {
        argv.push("--heading-deg".into());
        argv.push(h.to_string());
    }
    if let Some(m) = args.mjd {
        argv.push("--mjd".into());
        argv.push(m.to_string());
    }
    Ok(argv)
}

/// Explicit `--surface-package`, or default `simrocket_earth/packages/medium` when scenario is `earth`.
fn resolve_surface_package(args: &CliArgs) -> Option<PathBuf> {
    if let Some(ref p) = args.surface_package {
        return Some(p.clone());
    }
    if !args.scenario.eq_ignore_ascii_case("earth") {
        return None;
    }
    default_earth_surface_package()
}

fn default_earth_surface_package() -> Option<PathBuf> {
    // 默认 medium 档；缺失时回退 low。
    let candidates = [
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../../simrocket_earth/packages/medium"),
        PathBuf::from("simrocket_earth/packages/medium"),
        PathBuf::from("../simrocket_earth/packages/medium"),
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../../simrocket_earth/packages/low"),
        PathBuf::from("simrocket_earth/packages/low"),
        PathBuf::from("../simrocket_earth/packages/low"),
    ];
    for p in candidates {
        if p.join("manifest.toml").is_file() {
            return Some(p.canonicalize().unwrap_or(p));
        }
    }
    None
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
    /// 着陆成功/发射失败终局已由 CLI 发送过 Pause（一次性守卫）。
    outcome_pause_sent: bool,
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
            outcome_pause_sent: false,
        }
    }

    fn controls_enabled(&self) -> bool {
        self.focus.controls_enabled() && !self.terminal()
    }

    /// 任一终局（坠毁/着陆成功/发射失败）已上报。
    fn terminal(&self) -> bool {
        !self.slice.crash_msg.is_empty()
            || !self.slice.landed_msg.is_empty()
            || !self.slice.launch_failed_msg.is_empty()
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
    if state.terminal() {
        match key {
            KeyCode::Char('r') => {
                let _ = client.reset().await;
                state.throttle = 0.0;
                state.focus = SliceFocus::Primary;
                state.outcome_pause_sent = false;
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
            state.outcome_pause_sent = false;
            return;
        }
        KeyCode::Char('+') | KeyCode::Char('=') => {
            let w = (state.slice.warp * 2.0).clamp(0.125, 32.0);
            let _ = client.set_warp(w).await;
            return;
        }
        KeyCode::Char('-') => {
            let w = (state.slice.warp / 2.0).clamp(0.125, 32.0);
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
        // 图工作流点火。`w` 仍是油门切换，不占用。
        KeyCode::Char('i') => {
            let _ = client.workflow_ignite().await;
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
        // 着陆成功/发射失败：runtime 只上报，CLI 负责暂停步进（一次性）。
        if (!state.slice.landed_msg.is_empty() || !state.slice.launch_failed_msg.is_empty())
            && !state.outcome_pause_sent
        {
            let _ = client.pause().await;
            state.outcome_pause_sent = true;
        }
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
            for st in &state.slice.stages {
                eprintln!(
                    "smoke stage: {} throttle={:.3} firing={}",
                    st.name, st.throttle, st.firing
                );
            }
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
    if let Err(e) = validate_control_args(&args) {
        eprintln!("{e}");
        std::process::exit(2);
    }
    if let Err(e) = orbitx_config::load_rocket_source(&args.rocket) {
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
    let argv = match runtime_argv(&args) {
        Ok(v) => v,
        Err(e) => {
            eprintln!("{e}");
            std::process::exit(2);
        }
    };
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
    match child.try_wait() {
        Ok(Some(st)) => {
            eprintln!("orbitx-runtime 已退出 ({st})，火箭未加载");
            std::process::exit(1);
        }
        Err(e) => {
            eprintln!("检查 orbitx-runtime 状态失败：{e}");
            child.kill_and_wait();
            std::process::exit(1);
        }
        Ok(None) => {}
    }

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
