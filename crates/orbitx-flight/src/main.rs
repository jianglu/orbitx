//! orbitx 航天器飞行模拟器：实时 N-body 引力 + RK4 积分 + 推力控制。
//!
//! 用法：cargo run -p orbitx-flight
//!
//! 操作：
//! - W/S（按住）：沿速度方向前进/后退推力
//! - A/D（按住）：径向（朝向/远离太阳）推力
//! - 1-9：切换焦点天体（1=太阳 3=地球 ...）
//! - C：切换 Orbit/Chase 相机模式
//! - 鼠标拖拽：旋转视角（Orbit 模式）
//! - 滚轮：缩放
//! - +/-：加速/减速时间
//! - Space：暂停/继续
//! - R：重置到初始状态
//! - Escape：退出

use kiss3d::event::{Action, Key, WindowEvent};
use kiss3d::light::Light;
use kiss3d::prelude::*;
use kiss3d::text::Font;

use orbitx_dynamics::{gacc_nbody, rk4_step, Elements, GravBody};
use orbitx_math::{StateVectors, Vec3 as Vec3d};
use orbitx_scene::{BodyState, CameraFrame, Simulation};

const AU_RENDER_UNITS: f64 = 8.0;
const TRAIL_MAX: usize = 3000;
const TRAIL_INTERVAL: usize = 3;

/// 推力加速度 [m/s²]。
const THRUST_ACCEL: f64 = 15.0;
/// 燃料消耗速率 [%/s]（推力时）。
const FUEL_RATE: f64 = 2.0;
/// 焦点天体列表。
const FOCUS_NAMES: [&str; 10] = [
    "Sun", "Mercury", "Venus", "Earth", "Mars", "Jupiter", "Saturn", "Uranus", "Neptune", "Moon",
];

/// 天体质量表。
fn body_mass(name: &str) -> f64 {
    match name {
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
    }
}

fn initial_spacecraft_state(sim: &Simulation) -> (Vec3d, Vec3d) {
    let states = sim.body_states();
    let earth = states.iter().find(|s| s.name == "Earth").unwrap();
    let earth_pos = Vec3d::new(earth.pos[0], earth.pos[1], earth.pos[2]);

    let r_mag = earth_pos.length();
    let v_circular = (orbitx_math::consts::GGRAV * 1.989e30 / r_mag).sqrt();

    let tangent = Vec3d::new(earth_pos.z, 0.0, -earth_pos.x);
    let tangent_unit = if tangent.length() > 1e-3 {
        tangent.unit()
    } else {
        Vec3d::new(0.0, 0.0, -1.0)
    };

    let earth_vel = tangent_unit * v_circular;
    let pos = earth_pos + Vec3d::new(7.0e6, 0.0, 0.0);
    let vel = earth_vel + tangent_unit * 3000.0;

    (pos, vel)
}

fn collect_grav_bodies(states: &[BodyState]) -> Vec<GravBody> {
    states
        .iter()
        .map(|s| GravBody {
            pos: Vec3d::new(s.pos[0], s.pos[1], s.pos[2]),
            mass: body_mass(s.name),
            size: s.radius_m,
            jcoeff: vec![],
        })
        .collect()
}

fn quat_from_to(from: Vec3, to: Vec3) -> Option<Quat> {
    let from = from.normalize_or_zero();
    let to = to.normalize_or_zero();
    let dot = from.dot(to);
    if dot > 0.999_999 {
        return None;
    }
    if dot < -0.999_999 {
        let axis = if from.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
        let perp = (from.cross(axis)).normalize_or_zero();
        return Some(Quat::from_axis_angle(perp, std::f32::consts::PI));
    }
    let axis = from.cross(to).normalize_or_zero();
    let angle = dot.acos();
    Some(Quat::from_axis_angle(axis, angle))
}

/// 查找焦点天体的位置和速度。
fn find_body(states: &[BodyState], name: &str) -> Option<(Vec3d, f64)> {
    states
        .iter()
        .find(|s| s.name == name)
        .map(|s| (Vec3d::new(s.pos[0], s.pos[1], s.pos[2]), body_mass(s.name)))
}

#[kiss3d::main]
async fn main() {
    eprintln!("orbitx 航天器飞行模拟器启动中...");
    eprintln!("加载历表数据...");

    let mut sim = Simulation::new();
    sim.time_scale = 600.0;

    let mut frame = CameraFrame::new(AU_RENDER_UNITS);

    let (sc_pos, sc_vel) = initial_spacecraft_state(&sim);
    let mut sc_state = StateVectors {
        pos: sc_pos,
        vel: sc_vel,
        ..StateVectors::default()
    };
    let initial_state = sc_state;
    let mut fuel = 100.0_f64;

    frame.set_origin([sc_state.pos.x, sc_state.pos.y, sc_state.pos.z]);

    let mut window = Window::new("orbitx 航天器飞行").await;
    window.set_background_color(Color::new(0.0, 0.0, 0.0, 1.0));
    window.set_ambient(0.8);
    window.rebind_close_key(Some(Key::Escape));

    let font = Font::default();

    let mut scene = SceneNode3d::empty();
    scene.add_light(Light::directional(Vec3::new(-0.3, -0.5, -0.8)).with_intensity(3.0));

    // 行星节点。
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

    // 航天器造型。
    let mut sc_node = scene.add_group();
    let mut nose = sc_node
        .add_cone(0.12, 0.25)
        .set_color(Color::new(0.9, 0.9, 0.95, 1.0));
    nose.set_position(Vec3::new(0.0, 0.0, 0.18));
    nose.set_rotation(Quat::from_axis_angle(Vec3::X, -std::f32::consts::FRAC_PI_2));
    let mut body_part = sc_node
        .add_cylinder(0.1, 0.3)
        .set_color(Color::new(0.6, 0.7, 0.8, 1.0));
    body_part.set_rotation(Quat::from_axis_angle(Vec3::X, -std::f32::consts::FRAC_PI_2));
    let mut wing_l = sc_node
        .add_cube(0.35, 0.02, 0.15)
        .set_color(Color::new(0.4, 0.5, 0.6, 1.0));
    wing_l.set_position(Vec3::new(-0.18, 0.0, -0.05));
    let mut wing_r = sc_node
        .add_cube(0.35, 0.02, 0.15)
        .set_color(Color::new(0.4, 0.5, 0.6, 1.0));
    wing_r.set_position(Vec3::new(0.18, 0.0, -0.05));

    const VEL_AXIS_LEN: f32 = 2.0;
    const THRUST_LEN: f32 = 3.0;

    let mut trail: Vec<[f64; 3]> = Vec::with_capacity(TRAIL_MAX);
    let mut trail_poly = Polyline3d::new(vec![Vec3::ZERO])
        .with_color(Color::new(0.3, 1.0, 0.4, 0.9))
        .with_width(2.0);
    trail_poly.perspective = false;

    // 状态变量。
    let mut camera = OrbitCamera3d::new(Vec3::new(0.0, 3.0, 8.0), Vec3::ZERO);
    let mut paused = false;
    let mut last_instant = std::time::Instant::now();
    let mut frame_count = 0usize;
    let mut focus_idx: usize = 3; // 默认 Earth
    let mut chase_cam = false;
    let mut keys_w = false;
    let mut keys_s = false;
    let mut keys_a = false;
    let mut keys_d = false;
    let mut collision_msg = String::new();
    let mut collision_msg_timer = 0.0_f64;

    eprintln!("渲染循环开始。");
    eprintln!("W/S 前进后退（按住），A/D 径向推力，1-9 焦点切换，C 相机切换，+/- 调速，Space 暂停，R 重置，Esc 退出。");

    while window.render_3d(&mut scene, &mut camera).await {
        let now = std::time::Instant::now();
        let dt = now.duration_since(last_instant).as_secs_f64();
        last_instant = now;

        // 处理事件。
        for mut event in window.events().iter() {
            match event.value {
                WindowEvent::Key(key, Action::Press, _) => match key {
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
                    Key::R => {
                        sc_state = initial_state;
                        fuel = 100.0;
                        trail.clear();
                        collision_msg.clear();
                        eprintln!("重置到初始状态。");
                        event.inhibited = true;
                    }
                    Key::C => {
                        chase_cam = !chase_cam;
                        eprintln!("相机模式: {}", if chase_cam { "Chase" } else { "Orbit" });
                        event.inhibited = true;
                    }
                    Key::Key1 => {
                        focus_idx = 0;
                        eprintln!("焦点: Sun");
                        event.inhibited = true;
                    }
                    Key::Key2 => {
                        focus_idx = 1;
                        eprintln!("焦点: Mercury");
                        event.inhibited = true;
                    }
                    Key::Key3 => {
                        focus_idx = 3;
                        eprintln!("焦点: Earth");
                        event.inhibited = true;
                    }
                    Key::Key4 => {
                        focus_idx = 4;
                        eprintln!("焦点: Mars");
                        event.inhibited = true;
                    }
                    Key::Key5 => {
                        focus_idx = 5;
                        eprintln!("焦点: Jupiter");
                        event.inhibited = true;
                    }
                    Key::Key6 => {
                        focus_idx = 6;
                        eprintln!("焦点: Saturn");
                        event.inhibited = true;
                    }
                    Key::Key7 => {
                        focus_idx = 7;
                        eprintln!("焦点: Uranus");
                        event.inhibited = true;
                    }
                    Key::Key8 => {
                        focus_idx = 8;
                        eprintln!("焦点: Neptune");
                        event.inhibited = true;
                    }
                    _ => {}
                },
                WindowEvent::Key(key, Action::Release, _) => match key {
                    Key::W => {
                        keys_w = false;
                        event.inhibited = true;
                    }
                    Key::S => {
                        keys_s = false;
                        event.inhibited = true;
                    }
                    Key::A => {
                        keys_a = false;
                        event.inhibited = true;
                    }
                    Key::D => {
                        keys_d = false;
                        event.inhibited = true;
                    }
                    _ => {}
                },
                _ => {}
            }
            // 按下推力键。
            if let WindowEvent::Key(key, Action::Press, _) = event.value {
                match key {
                    Key::W => {
                        keys_w = true;
                        event.inhibited = true;
                    }
                    Key::S => {
                        keys_s = true;
                        event.inhibited = true;
                    }
                    Key::A => {
                        keys_a = true;
                        event.inhibited = true;
                    }
                    Key::D => {
                        keys_d = true;
                        event.inhibited = true;
                    }
                    _ => {}
                }
            }
        }

        // 计算推力方向和强度。
        let mut thrust_dir = Vec3d::ZERO;
        let mut thrust_active = false;
        if fuel > 0.0 {
            let v = sc_state.vel;
            let v_mag = v.length();
            let r = sc_state.pos;
            let r_mag = r.length();

            if keys_w && v_mag > 1e-6 {
                thrust_dir += v * (1.0 / v_mag);
                thrust_active = true;
            }
            if keys_s && v_mag > 1e-6 {
                thrust_dir -= v * (1.0 / v_mag);
                thrust_active = true;
            }
            if keys_a && r_mag > 1e-6 {
                thrust_dir -= r * (1.0 / r_mag);
                thrust_active = true;
            }
            if keys_d && r_mag > 1e-6 {
                thrust_dir += r * (1.0 / r_mag);
                thrust_active = true;
            }
        }

        if !paused {
            // 推进历表时间。
            let sim_dt = dt.min(0.1) * sim.time_scale;
            sim.mjd += sim_dt / 86400.0;

            let states = sim.body_states();
            let grav_bodies = collect_grav_bodies(&states);

            // 燃料消耗。
            if thrust_active {
                fuel -= FUEL_RATE * sim_dt;
                if fuel < 0.0 {
                    fuel = 0.0;
                }
            }

            // 推力加速度向量。
            let thrust_acc = if thrust_active {
                let dir_mag = thrust_dir.length();
                if dir_mag > 1e-6 {
                    thrust_dir * (THRUST_ACCEL / dir_mag)
                } else {
                    Vec3d::ZERO
                }
            } else {
                Vec3d::ZERO
            };

            // RK4 积分（含推力）。
            let n_substeps = 10;
            let sub_dt = sim_dt / n_substeps as f64;
            for _ in 0..n_substeps {
                let gb = grav_bodies.clone();
                let ta = thrust_acc;
                let mut force = move |s: &StateVectors, _tfrac: f64| {
                    let mut acc = gacc_nbody(s.pos, &gb, None);
                    acc += ta;
                    (acc, Vec3d::ZERO)
                };
                sc_state = rk4_step(sc_state, sub_dt, &mut force);
            }

            // 碰撞检测。
            let states_now = sim.body_states();
            for bs in &states_now {
                let bp = Vec3d::new(bs.pos[0], bs.pos[1], bs.pos[2]);
                let dist = (sc_state.pos - bp).length();
                if dist < bs.radius_m {
                    collision_msg = format!("碰撞 {}！重置。", bs.name);
                    collision_msg_timer = 3.0;
                    eprintln!("{}", collision_msg);
                    sc_state = initial_state;
                    fuel = 100.0;
                    trail.clear();
                    break;
                }
            }

            // 采样轨迹。
            if frame_count % TRAIL_INTERVAL == 0 {
                trail.push([sc_state.pos.x, sc_state.pos.y, sc_state.pos.z]);
                if trail.len() > TRAIL_MAX {
                    trail.remove(0);
                }
            }

            collision_msg_timer -= dt;
            if collision_msg_timer <= 0.0 {
                collision_msg.clear();
            }
        }

        frame_count += 1;

        // 更新浮点原点。
        frame.set_origin([sc_state.pos.x, sc_state.pos.y, sc_state.pos.z]);

        // 更新航天器位置和朝向。
        let sc_render_pos = frame.to_render([sc_state.pos.x, sc_state.pos.y, sc_state.pos.z]);
        sc_node.set_position(sc_render_pos);
        {
            let v = sc_state.vel;
            let v_mag = v.length();
            if v_mag > 1e-3 {
                let v_target = frame.to_render([
                    sc_state.pos.x + v.x,
                    sc_state.pos.y + v.y,
                    sc_state.pos.z + v.z,
                ]);
                let v_dir = (v_target - sc_render_pos).normalize_or_zero();
                let forward = Vec3::new(0.0, 0.0, 1.0);
                if let Some(rot) = quat_from_to(forward, v_dir) {
                    sc_node.set_rotation(rot);
                }
            }
        }

        // 更新行星位置。
        let states = sim.body_states();
        for (i, state) in states.iter().enumerate() {
            if i < body_nodes.len() {
                let pos = frame.to_render(state.pos);
                body_nodes[i].set_position(pos);
            }
        }

        // Chase 相机。
        if chase_cam {
            let v = sc_state.vel;
            let v_mag = v.length();
            if v_mag > 1e-3 {
                let v_target = frame.to_render([
                    sc_state.pos.x + v.x,
                    sc_state.pos.y + v.y,
                    sc_state.pos.z + v.z,
                ]);
                let v_dir = (v_target - sc_render_pos).normalize_or_zero();
                let eye = sc_render_pos - v_dir * 5.0 + Vec3::new(0.0, 2.0, 0.0);
                camera = OrbitCamera3d::new(eye, sc_render_pos);
            }
        }

        // 绘制轨迹。
        if trail.len() >= 2 {
            trail_poly.vertices.clear();
            for p in &trail {
                trail_poly.vertices.push(frame.to_render(*p));
            }
            window.draw_polyline(&trail_poly);
        }

        // 绘制速度轴线（青色，尾部方向）。
        {
            let v = sc_state.vel;
            let v_mag = v.length();
            if v_mag > 1e-6 {
                let v_dir_f64 = [-v.x / v_mag, -v.y / v_mag, -v.z / v_mag];
                let v_render = frame.to_render([
                    sc_state.pos.x + v_dir_f64[0],
                    sc_state.pos.y + v_dir_f64[1],
                    sc_state.pos.z + v_dir_f64[2],
                ]);
                let vel_dir = (v_render - sc_render_pos).normalize_or_zero();
                let vel_end = sc_render_pos + vel_dir * VEL_AXIS_LEN;
                window.draw_line(
                    sc_render_pos,
                    vel_end,
                    Color::new(0.2, 1.0, 1.0, 1.0),
                    3.0,
                    false,
                );
            }
        }

        // 绘制推力矢量。
        if thrust_active {
            let dir_mag = thrust_dir.length();
            if dir_mag > 1e-6 {
                // 喷气方向 = 推力反方向。
                let exh = thrust_dir * (-1.0 / dir_mag);
                let exh_render = frame.to_render([
                    sc_state.pos.x + exh.x,
                    sc_state.pos.y + exh.y,
                    sc_state.pos.z + exh.z,
                ]);
                let dir = (exh_render - sc_render_pos).normalize_or_zero();
                let end = sc_render_pos + dir * THRUST_LEN;
                window.draw_line(
                    sc_render_pos,
                    end,
                    Color::new(1.0, 0.8, 0.2, 1.0),
                    4.0,
                    false,
                );
            }
        }

        // HUD：轨道根数。
        let focus_name = FOCUS_NAMES.get(focus_idx).copied().unwrap_or("Sun");
        if let Some((focus_pos, focus_mass)) = find_body(&states, focus_name) {
            let rel_pos = sc_state.pos - focus_pos;
            let rel_vel = sc_state.vel;
            let gm = orbitx_math::consts::GGRAV * focus_mass;

            // 检查是否为约束轨道（能量 < 0）。
            let v2 = rel_vel.length2();
            let r = rel_pos.length();
            let energy = v2 / 2.0 - gm / r;
            let is_bound = energy < 0.0;

            let mut hud = format!("Focus: {}\n", focus_name);
            hud += &format!("v = {:.1} m/s   r = {:.0} km\n", rel_vel.length(), r / 1e3);
            hud += &format!("Fuel = {:.0}%\n", fuel);

            if is_bound && gm > 0.0 {
                let el = Elements::calculate(rel_pos, rel_vel, gm, 0.0);
                hud += &format!("a = {:.0} km   e = {:.3}\n", el.a / 1e3, el.e);
                hud += &format!(
                    "PeD = {:.0} km   ApD = {:.0} km\n",
                    el.pe_dist() / 1e3,
                    el.ap_dist() / 1e3
                );
                let period_min = el.orbit_t() / 60.0;
                if period_min > 0.0 && period_min < 1e8 {
                    hud += &format!("T = {:.1} min\n", period_min);
                }
                hud += &format!("i = {:.1}°\n", el.i.to_degrees());
            } else {
                hud += "(双曲线路径)\n";
            }

            if !collision_msg.is_empty() {
                hud += &format!("\n!!! {} !!!", collision_msg);
            }

            window.draw_text(
                &hud,
                Vec2::new(10.0, 10.0),
                1.0,
                &font,
                Color::new(0.9, 1.0, 0.9, 1.0),
            );
        }
    }

    eprintln!("退出。");
}
