//! orbitx 航天器飞行模拟器：实时 N-body 引力 + RK4 积分 + 推力控制。
//!
//! 用法：cargo run -p orbitx-flight
//!
//! 操作：
//! - W/S：沿速度方向前进/后退推力
//! - A/D：径向（朝向/远离太阳）推力
//! - 鼠标拖拽：旋转视角
//! - 滚轮：缩放
//! - +/-：加速/减速时间
//! - Space：暂停/继续
//! - R：重置到初始状态
//! - Escape：退出

use kiss3d::event::{Action, Key, WindowEvent};
use kiss3d::light::Light;
use kiss3d::prelude::*;

use orbitx_dynamics::{gacc_nbody, rk4_step, GravBody};
use orbitx_math::{StateVectors, Vec3 as Vec3d};
use orbitx_scene::{BodyState, CameraFrame, Simulation};

/// 1 AU 在渲染中对应多少个单位。
const AU_RENDER_UNITS: f64 = 8.0;

/// 轨迹尾迹最大点数。
const TRAIL_MAX: usize = 3000;
/// 每隔多少帧采样一次轨迹点。
const TRAIL_INTERVAL: usize = 3;

/// 航天器初始状态：从地球轨道出发，带一定速度增量。
fn initial_spacecraft_state(sim: &Simulation) -> (Vec3d, Vec3d) {
    // 获取地球位置。
    let states = sim.body_states();
    let earth = states.iter().find(|s| s.name == "Earth").unwrap();

    // 地球位置（米，左手系）。
    let earth_pos = Vec3d::new(earth.pos[0], earth.pos[1], earth.pos[2]);

    // 地球轨道速度：用解析公式计算。
    // 地球轨道速度 ≈ sqrt(GM_sun / r)。
    let r_mag = earth_pos.length();
    let v_circular = (orbitx_math::consts::GGRAV * 1.989e30 / r_mag).sqrt(); // sqrt(GM_sun/r)

    // 速度方向：垂直于径向，在黄道面内。
    // 左手系中，位置 (x,y,z) 的切向速度方向 ≈ (z, 0, -x) 归一化。
    let tangent = Vec3d::new(earth_pos.z, 0.0, -earth_pos.x);
    let tangent_unit = if tangent.length() > 1e-3 {
        tangent.unit()
    } else {
        Vec3d::new(0.0, 0.0, -1.0)
    };

    let earth_vel = tangent_unit * v_circular;

    // 航天器：从地球位置出发，带 3 km/s 切向增量（进入更大椭圆轨道）。
    let pos = earth_pos + Vec3d::new(7.0e6, 0.0, 0.0); // 偏移 7000km
    let vel = earth_vel + tangent_unit * 3000.0;

    (pos, vel)
}

/// 收集当前所有天体为 GravBody 列表（用于 N-body 引力计算）。
fn collect_grav_bodies(states: &[BodyState]) -> Vec<GravBody> {
    states
        .iter()
        .map(|s| {
            let mass = match s.name {
                "Sun" => 1.989e30,
                "Mercury" => 3.301e23,
                "Venus" => 4.867e24,
                "Earth" => 5.972e24,
                "Mars" => 6.417e23,
                "Jupiter" => 1.898e27,
                "Saturn" => 5.683e26,
                "Uranus" => 8.681e25,
                "Neptune" => 1.024e26,
                "Moon" => 7.342e22,
                _ => 0.0,
            };
            GravBody {
                pos: Vec3d::new(s.pos[0], s.pos[1], s.pos[2]),
                mass,
                size: s.radius_m,
                jcoeff: vec![],
            }
        })
        .collect()
}

#[kiss3d::main]
async fn main() {
    eprintln!("orbitx 航天器飞行模拟器启动中...");
    eprintln!("加载历表数据...");

    let mut sim = Simulation::new();
    sim.time_scale = 600.0; // 默认 600 秒/帧 ≈ 10 分钟/秒

    let mut frame = CameraFrame::new(AU_RENDER_UNITS);

    // 初始化航天器状态。
    let (sc_pos, sc_vel) = initial_spacecraft_state(&sim);
    let mut sc_state = StateVectors {
        pos: sc_pos,
        vel: sc_vel,
        ..StateVectors::default()
    };
    let initial_state = sc_state;

    // 设置浮点原点为航天器位置。
    frame.set_origin([sc_state.pos.x, sc_state.pos.y, sc_state.pos.z]);

    let mut window = Window::new("orbitx 航天器飞行").await;
    window.set_background_color(Color::new(0.0, 0.0, 0.0, 1.0));
    window.set_ambient(0.8);
    window.rebind_close_key(Some(Key::Escape));

    let mut scene = SceneNode3d::empty();
    scene.add_light(Light::directional(Vec3::new(-0.3, -0.5, -0.8)).with_intensity(3.0));

    // 创建行星节点。
    let initial_states = sim.body_states();
    let mut body_nodes: Vec<SceneNode3d> = Vec::with_capacity(initial_states.len());
    for state in &initial_states {
        let color = Color::new(
            state.color[0],
            state.color[1],
            state.color[2],
            state.color[3],
        );
        let r = state.min_render_radius;
        let node = scene.add_sphere(r).set_color(color);
        body_nodes.push(node);
    }

    // 航天器节点：亮绿色小球。
    let mut sc_node = scene
        .add_sphere(0.3)
        .set_color(Color::new(0.2, 1.0, 0.3, 1.0));

    // 轨迹尾迹。
    let mut trail: Vec<[f64; 3]> = Vec::with_capacity(TRAIL_MAX);
    let mut trail_poly = Polyline3d::new(vec![Vec3::ZERO])
        .with_color(Color::new(0.3, 1.0, 0.4, 0.9))
        .with_width(2.0);
    trail_poly.perspective = false;

    // 相机：跟随航天器。
    let mut camera = OrbitCamera3d::new(Vec3::new(0.0, 3.0, 8.0), Vec3::ZERO);

    let mut paused = false;
    let mut last_instant = std::time::Instant::now();
    let mut frame_count = 0usize;
    let mut thrust_forward: f64;
    let mut thrust_radial: f64;

    eprintln!("渲染循环开始。");
    eprintln!("操作：W/S 前进后退，A/D 径向推力，+/- 调速，Space 暂停，R 重置，Esc 退出。");

    while window.render_3d(&mut scene, &mut camera).await {
        let now = std::time::Instant::now();
        let dt = now.duration_since(last_instant).as_secs_f64();
        last_instant = now;

        // 重置推力。
        thrust_forward = 0.0;
        thrust_radial = 0.0;

        for mut event in window.events().iter() {
            if let WindowEvent::Key(key, Action::Press, _) = event.value {
                match key {
                    Key::Space => {
                        paused = !paused;
                        eprintln!("暂停: {paused}");
                        event.inhibited = true;
                    }
                    Key::Add | Key::Equals => {
                        sim.time_scale *= 2.0;
                        eprintln!("时间加速: {} 秒/帧", sim.time_scale as u64);
                        event.inhibited = true;
                    }
                    Key::Minus => {
                        sim.time_scale /= 2.0;
                        eprintln!("时间减速: {} 秒/帧", sim.time_scale as u64);
                        event.inhibited = true;
                    }
                    Key::W => {
                        thrust_forward = 500.0; // +500 m/s 速度增量
                        event.inhibited = true;
                    }
                    Key::S => {
                        thrust_forward = -500.0;
                        event.inhibited = true;
                    }
                    Key::A => {
                        thrust_radial = -300.0; // 朝向太阳
                        event.inhibited = true;
                    }
                    Key::D => {
                        thrust_radial = 300.0; // 远离太阳
                        event.inhibited = true;
                    }
                    Key::R => {
                        sc_state = initial_state;
                        trail.clear();
                        eprintln!("重置到初始状态。");
                        event.inhibited = true;
                    }
                    _ => {}
                }
            }
        }

        if !paused {
            // 应用推力（瞬时速度增量，Burn 模式）。
            if thrust_forward != 0.0 || thrust_radial != 0.0 {
                let v = sc_state.vel;
                let v_mag = v.length();
                if v_mag > 1e-6 {
                    // 前进方向：沿速度方向。
                    let v_unit = v * (1.0 / v_mag);
                    sc_state.vel += v_unit * thrust_forward;
                }
                // 径向方向：朝向/远离太阳。
                let r = sc_state.pos;
                let r_mag = r.length();
                if r_mag > 1e-6 {
                    let r_unit = r * (1.0 / r_mag);
                    sc_state.vel += r_unit * thrust_radial;
                }
            }

            // 推进历表时间（行星位置更新）。
            let sim_dt = dt.min(0.1) * sim.time_scale;
            sim.mjd += sim_dt / 86400.0;

            // 获取当前天体位置作为引力源。
            let states = sim.body_states();
            let grav_bodies = collect_grav_bodies(&states);

            // 用 RK4 积分航天器状态。
            // 分成多个子步骤以提高精度。
            let n_substeps = 10;
            let sub_dt = sim_dt / n_substeps as f64;
            for _ in 0..n_substeps {
                let gb = grav_bodies.clone();
                let mut force = move |s: &StateVectors, _tfrac: f64| {
                    let acc = gacc_nbody(
                        s.pos, &gb, None, // 不排除任何天体
                    );
                    (acc, Vec3d::ZERO)
                };
                sc_state = rk4_step(sc_state, sub_dt, &mut force);
            }

            // 采样轨迹。
            if frame_count % TRAIL_INTERVAL == 0 {
                trail.push([sc_state.pos.x, sc_state.pos.y, sc_state.pos.z]);
                if trail.len() > TRAIL_MAX {
                    trail.remove(0);
                }
            }
        }

        frame_count += 1;

        // 更新浮点原点为航天器位置。
        frame.set_origin([sc_state.pos.x, sc_state.pos.y, sc_state.pos.z]);

        // 更新航天器节点位置。
        let sc_render_pos = frame.to_render([sc_state.pos.x, sc_state.pos.y, sc_state.pos.z]);
        sc_node.set_position(sc_render_pos);

        // 更新行星位置。
        let states = sim.body_states();
        for (i, state) in states.iter().enumerate() {
            if i < body_nodes.len() {
                let pos = frame.to_render(state.pos);
                body_nodes[i].set_position(pos);
            }
        }

        // 绘制轨迹尾迹。
        if trail.len() >= 2 {
            trail_poly.vertices.clear();
            for p in &trail {
                trail_poly.vertices.push(frame.to_render(*p));
            }
            window.draw_polyline(&trail_poly);
        }
    }

    eprintln!("退出。");
}
