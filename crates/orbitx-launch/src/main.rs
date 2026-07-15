#![allow(dead_code)]

//! orbitx 地球发射测试器：真实比例，火箭从地表发射场起飞。
//!
//! 1:1 真实比例渲染（1 渲染单位 = 1 米）。浮点原点跟随火箭位置。
//! 起飞时地面是平的（正确），飞高后相机拉远看到地球弧度。
//!
//! 用法：cargo run -p orbitx-launch
//!
//! 操作：
//! - W（按住）：主发动机推力
//! - A/D：调整俯仰角（10km 后解锁）
//! - G：切换自动重力转向（默认开）
//! - C：切换相机（Chase/Orbit）
//! - R：重置到发射台
//! - Space：暂停
//! - +/-：时间加速
//! - Escape：退出

use kiss3d::event::{Action, Key, WindowEvent};
use kiss3d::light::Light;
use kiss3d::prelude::*;

use orbitx_dynamics::{rk4_step, Elements};
use orbitx_math::{cross, dot, Vec3 as Vec3d};
use orbitx_scene::{load_cjk_font, CameraFrame};

// === 地球参数 ===
const EARTH_R: f64 = 6_371_000.0;
const EARTH_MASS: f64 = 5.972e24;
const EARTH_GM: f64 = orbitx_math::consts::GGRAV * EARTH_MASS;

// === 大气参数 ===
const RHO0: f64 = 1.225;
const SCALE_H: f64 = 8500.0;
const ATM_TOP: f64 = 100_000.0;
const DRAG_COEFF: f64 = 0.005;

// === 火箭参数（Falcon 9 真实尺寸）===
const ROCKET_H: f64 = 70.0; // 高度 [m]
const ROCKET_R: f64 = 1.85; // 半径 [m]
const THRUST_ACCEL: f64 = 25.0;
const FUEL_RATE: f64 = 5.0;
const PITCH_RATE: f64 = 0.523_598_775_598_298_8;
const LOCK_ALT: f64 = 10_000.0;

/// 发射场位置。
const LAUNCH_POS: Vec3d = Vec3d::new(0.0, 0.0, EARTH_R);

fn air_density(h: f64) -> f64 {
    if !(0.0..=ATM_TOP).contains(&h) {
        0.0
    } else {
        RHO0 * (-h / SCALE_H).exp()
    }
}

fn quat_from_to(from: Vec3, to: Vec3) -> Option<Quat> {
    let from = from.normalize_or_zero();
    let to = to.normalize_or_zero();
    let d = from.dot(to);
    if d > 0.999_999 {
        return None;
    }
    if d < -0.999_999 {
        let axis = if from.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
        let perp = from.cross(axis).normalize_or_zero();
        return Some(Quat::from_axis_angle(perp, std::f32::consts::PI));
    }
    let axis = from.cross(to).normalize_or_zero();
    Some(Quat::from_axis_angle(axis, d.acos()))
}

struct Rocket {
    pos: Vec3d,
    vel: Vec3d,
    pitch: f64,
    fuel: f64,
}

impl Rocket {
    fn on_pad() -> Self {
        // 抬高半个火箭高度，使底部贴地。
        let radial = LAUNCH_POS * (1.0 / LAUNCH_POS.length());
        Rocket {
            pos: LAUNCH_POS + radial * (ROCKET_H / 2.0),
            vel: Vec3d::ZERO,
            pitch: 0.0,
            fuel: 100.0,
        }
    }

    fn altitude(&self) -> f64 {
        self.pos.length() - EARTH_R
    }

    fn speed(&self) -> f64 {
        self.vel.length()
    }

    fn thrust_dir(&self) -> Vec3d {
        let r = self.pos;
        let r_mag = r.length();
        if r_mag < 1e-3 {
            return Vec3d::new(0.0, 1.0, 0.0);
        }
        let radial = r * (1.0 / r_mag);
        let tangent_base = if self.vel.length() > 1e-3 {
            let v_radial = radial * dot(self.vel, radial);
            let v_tan = self.vel - v_radial;
            if v_tan.length() > 1e-3 {
                v_tan.unit()
            } else {
                cross(radial, Vec3d::new(0.0, 1.0, 0.0)).unit()
            }
        } else {
            cross(radial, Vec3d::new(0.0, 1.0, 0.0)).unit()
        };
        let cos_p = self.pitch.cos();
        let sin_p = self.pitch.sin();
        (radial * cos_p + tangent_base * sin_p).unit()
    }
}

const TRAIL_MAX: usize = 5000;
const TRAIL_INTERVAL: usize = 2;

/// 构建火箭模型（真实尺寸 70m）。
fn build_rocket(scene: &mut SceneNode3d) -> (SceneNode3d, SceneNode3d) {
    let total_h = ROCKET_H as f32;
    let radius = ROCKET_R as f32;
    let nose_h = total_h * 0.12;
    let body_h = total_h * 0.78;
    let nozzle_h = total_h * 0.10;
    let nose_tip = total_h / 2.0;
    let nose_base = nose_tip - nose_h;
    let body_bottom = nose_base - body_h;
    let nozzle_tip = body_bottom - nozzle_h;

    let mut rocket = scene.add_group();

    // 头部锥。
    let mut nose = rocket
        .add_cone(radius, nose_h)
        .set_color(Color::new(0.85, 0.15, 0.1, 1.0));
    nose.set_position(Vec3::new(0.0, (nose_tip + nose_base) / 2.0, 0.0));

    // 主体。
    let mut body = rocket
        .add_cylinder(radius, body_h)
        .set_color(Color::new(0.92, 0.92, 0.92, 1.0));
    body.set_position(Vec3::new(0.0, (nose_base + body_bottom) / 2.0, 0.0));

    // 条纹。
    let stripe_h = total_h * 0.01;
    let mut stripe = rocket
        .add_cylinder(radius * 1.02, stripe_h)
        .set_color(Color::new(0.85, 0.15, 0.1, 1.0));
    stripe.set_position(Vec3::new(0.0, body_bottom + body_h * 0.2, 0.0));

    // 喷管。
    let nozzle_r = radius * 0.7;
    let mut nozzle = rocket
        .add_cone(nozzle_r, nozzle_h)
        .set_color(Color::new(0.2, 0.2, 0.23, 1.0));
    nozzle.set_position(Vec3::new(0.0, (body_bottom + nozzle_tip) / 2.0, 0.0));

    // 尾翼。
    let fin_color = Color::new(0.35, 0.35, 0.4, 1.0);
    let fin_inner = radius;
    let fin_outer = radius * 2.5;
    let fin_y_top = body_bottom + body_h * 0.3;
    let fin_y_bot = body_bottom + body_h * 0.05;
    for i in 0..4u32 {
        let angle = std::f32::consts::FRAC_PI_2 * i as f32;
        let (ca, sa) = (angle.cos(), angle.sin());
        let rot_v = |x: f32, z: f32| Vec3::new(x * ca + z * sa, 0.0, -x * sa + z * ca);
        let thickness = 0.1;
        let offset = rot_v(0.0, thickness);
        let a = rot_v(fin_inner, 0.0) + Vec3::new(0.0, fin_y_top, 0.0);
        let b = rot_v(fin_inner, 0.0) + Vec3::new(0.0, fin_y_bot, 0.0);
        let c = rot_v(fin_outer, 0.0) + Vec3::new(0.0, fin_y_bot, 0.0);
        let vertices = vec![a, b, c, a + offset, b + offset, c + offset];
        let indices = vec![
            [0, 1, 2],
            [5, 4, 3],
            [0, 3, 4],
            [0, 4, 1],
            [1, 4, 5],
            [1, 5, 2],
            [2, 5, 3],
            [2, 3, 0],
        ];
        rocket
            .add_trimesh(vertices, indices, Vec3::ONE, true)
            .set_color(fin_color);
    }

    // 火焰。
    let flame_h = total_h * 0.08;
    let flame_r = nozzle_r * 0.6;
    let mut flame = rocket
        .add_cone(flame_r, flame_h)
        .set_color(Color::new(1.0, 0.65, 0.15, 0.85));
    flame.set_position(Vec3::new(0.0, nozzle_tip - flame_h / 2.0, 0.0));
    flame.set_rotation(Quat::from_axis_angle(Vec3::X, std::f32::consts::PI));
    flame.set_surface_rendering_activation(false);

    (rocket, flame)
}

/// 生成地面网格线（以 origin 为中心的 N×N 网格）。
fn build_ground_grid(origin_render: Vec3, up: Vec3, size: f32, step: f32) -> Vec<Polyline3d> {
    let mut lines = Vec::new();
    // 构造与 up 垂直的两个基向量。
    let right = if up.x.abs() < 0.9 {
        up.cross(Vec3::X).normalize_or_zero()
    } else {
        up.cross(Vec3::Z).normalize_or_zero()
    };
    let forward = up.cross(right).normalize_or_zero();

    let n = (size / step) as i32;
    for i in -n..=n {
        let t = i as f32 * step;
        // X 方向线。
        let a = origin_render + right * t - forward * size;
        let b = origin_render + right * t + forward * size;
        let mut p = Polyline3d::new(vec![a, b])
            .with_color(Color::new(0.2, 0.3, 0.2, 0.4))
            .with_width(1.0);
        p.perspective = true;
        lines.push(p);
        // Z 方向线。
        let a = origin_render + forward * t - right * size;
        let b = origin_render + forward * t + right * size;
        let mut p = Polyline3d::new(vec![a, b])
            .with_color(Color::new(0.2, 0.3, 0.2, 0.4))
            .with_width(1.0);
        p.perspective = true;
        lines.push(p);
    }
    lines
}

#[kiss3d::main]
async fn main() {
    eprintln!("orbitx 地球发射测试器（真实比例）");
    eprintln!("W 推力，A/D 俯仰(10km后)，G 自动转向，C 相机，R 重置，Space 暂停，Esc 退出。");

    let mut rocket_state = Rocket::on_pad();
    let mut paused = false;
    let mut last_instant = std::time::Instant::now();
    let mut frame_count = 0usize;
    let mut time_scale: f64 = 5.0;
    let mut keys_w = false;
    let mut keys_a = false;
    let mut keys_d = false;
    let mut chase_cam = true;
    let mut auto_gravity_turn = true;
    let mut crash_msg = String::new();
    let mut crash_timer = 0.0_f64;

    // 1:1 渲染比例 + 浮点原点。
    let mut frame = CameraFrame::new_with_scale(1.0);
    frame.set_origin([rocket_state.pos.x, rocket_state.pos.y, rocket_state.pos.z]);

    let mut window = Window::new("orbitx 发射").await;
    window.set_background_color(Color::new(0.0, 0.0, 0.02, 1.0));
    window.set_ambient(0.6);
    window.rebind_close_key(Some(Key::Escape));

    let font = load_cjk_font();

    let mut scene = SceneNode3d::empty();
    scene.add_light(Light::directional(Vec3::new(-0.5, -0.3, -0.8)).with_intensity(5.0));
    scene
        .add_light(Light::point(500.0))
        .set_position(Vec3::new(0.0, 100.0, 0.0));

    // 火箭模型（真实 70m 尺寸）。
    let (mut sc_node, mut flame_node) = build_rocket(&mut scene);

    // 轨迹。
    let mut trail: Vec<Vec3d> = Vec::with_capacity(TRAIL_MAX);
    let mut trail_poly = Polyline3d::new(vec![Vec3::ZERO])
        .with_color(Color::new(0.3, 1.0, 0.4, 0.9))
        .with_width(1.5);
    trail_poly.perspective = false;

    let mut camera = OrbitCamera3d::new(Vec3::new(0.0, 50.0, -200.0), Vec3::ZERO);

    while window.render_3d(&mut scene, &mut camera).await {
        let now = std::time::Instant::now();
        let dt_real = now.duration_since(last_instant).as_secs_f64();
        last_instant = now;

        for mut event in window.events().iter() {
            if chase_cam {
                match event.value {
                    WindowEvent::MouseButton(_, _, _)
                    | WindowEvent::CursorPos(_, _, _)
                    | WindowEvent::Scroll(_, _, _) => {
                        event.inhibited = true;
                        continue;
                    }
                    _ => {}
                }
            }
            match event.value {
                WindowEvent::Key(key, Action::Press, _) => match key {
                    Key::Space => {
                        paused = !paused;
                        event.inhibited = true;
                    }
                    Key::Add | Key::Equals => {
                        time_scale *= 2.0;
                        event.inhibited = true;
                    }
                    Key::Minus => {
                        time_scale /= 2.0;
                        event.inhibited = true;
                    }
                    Key::R => {
                        rocket_state = Rocket::on_pad();
                        trail.clear();
                        crash_msg.clear();
                        event.inhibited = true;
                    }
                    Key::C => {
                        chase_cam = !chase_cam;
                        event.inhibited = true;
                    }
                    Key::G => {
                        auto_gravity_turn = !auto_gravity_turn;
                        event.inhibited = true;
                    }
                    Key::W => {
                        keys_w = true;
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
                },
                WindowEvent::Key(key, Action::Release, _) => match key {
                    Key::W => {
                        keys_w = false;
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
        }

        if !paused {
            let dt = dt_real.min(0.1) * time_scale;
            let h = rocket_state.altitude();

            if h < LOCK_ALT {
                rocket_state.pitch = 0.0;
            } else if auto_gravity_turn {
                let target_pitch =
                    ((h - LOCK_ALT) / 70_000.0).min(1.0) * std::f64::consts::FRAC_PI_2;
                if rocket_state.pitch < target_pitch {
                    rocket_state.pitch = (rocket_state.pitch + PITCH_RATE * dt).min(target_pitch);
                }
            } else {
                if keys_a {
                    rocket_state.pitch -= PITCH_RATE * dt;
                }
                if keys_d {
                    rocket_state.pitch += PITCH_RATE * dt;
                }
            }
            rocket_state.pitch = rocket_state.pitch.clamp(0.0, std::f64::consts::FRAC_PI_2);

            let thrusting = keys_w && rocket_state.fuel > 0.0;

            // 在发射台上（无推力 + 无速度）时锁定不动，不应用重力。
            let on_pad = !thrusting && rocket_state.vel.length() < 1.0 && h < 100.0;
            if on_pad {
                rocket_state.pos = Rocket::on_pad().pos;
            }

            if thrusting {
                rocket_state.fuel = (rocket_state.fuel - FUEL_RATE * dt).max(0.0);
            }

            let thrust_dir = rocket_state.thrust_dir();

            // 在发射台上时跳过物理积分。
            if !on_pad {
                let n_sub = 10;
                let sub_dt = dt / n_sub as f64;
                for _ in 0..n_sub {
                    let pos = rocket_state.pos;
                    let vel = rocket_state.vel;
                    let td = thrust_dir;
                    let thrust = thrusting;
                    let mut force = move |s: &orbitx_math::StateVectors, _t: f64| {
                        let r = s.pos;
                        let r_mag = r.length();
                        let g_acc = r * (EARTH_GM / (r_mag * r_mag * r_mag));
                        let v = s.vel;
                        let v_mag = v.length();
                        let alt = r_mag - EARTH_R;
                        let rho = air_density(alt);
                        let drag_acc = if v_mag > 1e-3 && rho > 1e-10 {
                            let drag_mag = 0.5 * rho * v_mag * v_mag * DRAG_COEFF;
                            v * (-drag_mag / v_mag)
                        } else {
                            Vec3d::ZERO
                        };
                        let thrust_acc = if thrust {
                            td * THRUST_ACCEL
                        } else {
                            Vec3d::ZERO
                        };
                        (g_acc + drag_acc + thrust_acc, Vec3d::ZERO)
                    };
                    let sv = orbitx_math::StateVectors {
                        pos,
                        vel,
                        ..Default::default()
                    };
                    let next = rk4_step(sv, sub_dt, &mut force);
                    rocket_state.pos = next.pos;
                    rocket_state.vel = next.vel;
                }
            }

            // 碰撞。
            let h1 = rocket_state.altitude();
            if h1 < 0.0 {
                if rocket_state.speed() > 50.0 {
                    crash_msg = format!("坠毁！速度 {} m/s", rocket_state.speed() as u64);
                    crash_timer = 3.0;
                    rocket_state = Rocket::on_pad();
                    trail.clear();
                } else {
                    let r_mag = rocket_state.pos.length();
                    rocket_state.pos *= EARTH_R / r_mag;
                    rocket_state.vel = Vec3d::ZERO;
                }
            }

            if frame_count % TRAIL_INTERVAL == 0 {
                trail.push(rocket_state.pos);
                if trail.len() > TRAIL_MAX {
                    trail.remove(0);
                }
            }

            crash_timer -= dt_real;
            if crash_timer <= 0.0 {
                crash_msg.clear();
            }
        }

        frame_count += 1;

        // === 渲染 ===
        // 浮点原点 = 火箭位置。
        frame.set_origin([rocket_state.pos.x, rocket_state.pos.y, rocket_state.pos.z]);

        let sc_pos_render =
            frame.to_render([rocket_state.pos.x, rocket_state.pos.y, rocket_state.pos.z]);
        sc_node.set_position(sc_pos_render);

        // 火焰。
        let thrusting = keys_w && rocket_state.fuel > 0.0;
        if thrusting {
            flame_node.set_surface_rendering_activation(true);
            let flicker = 0.7 + rand_flicker(frame_count) * 0.5;
            flame_node.set_local_scale(1.0, flicker, 1.0);
        } else {
            flame_node.set_surface_rendering_activation(false);
        }

        // 相机距离随高度自适应。
        let h = rocket_state.altitude();
        let cam_dist = (h * 0.5 + 200.0).clamp(200.0, 500_000.0) as f32;

        // frustum 远裁剪面必须覆盖地面网格范围。
        let zfar = (cam_dist * 10.0).max(10_000.0);

        if chase_cam {
            let eye = sc_pos_render + Vec3::new(0.0, cam_dist * 0.2, -cam_dist);
            camera = OrbitCamera3d::new_with_frustum(
                std::f32::consts::FRAC_PI_4,
                0.1,
                zfar,
                eye,
                sc_pos_render,
            );
        } else {
            let eye = sc_pos_render + Vec3::new(0.0, cam_dist * 0.3, -cam_dist);
            camera = OrbitCamera3d::new_with_frustum(
                std::f32::consts::FRAC_PI_4,
                0.1,
                zfar,
                eye,
                sc_pos_render,
            );
        }

        // 地面网格（低空时渲染）。
        if h < 200_000.0 {
            // 发射点渲染位置。
            let pad_render = frame.to_render([LAUNCH_POS.x, LAUNCH_POS.y, LAUNCH_POS.z]);
            // "上"方向 = 径向（渲染系）。
            let radial_render = (pad_render - frame.to_render([0.0, 0.0, 0.0])).normalize_or_zero();
            // 网格大小随高度增加。
            let grid_size = (h.max(100.0) * 2.0).min(50_000.0) as f32;
            let grid_step = (grid_size / 20.0).max(10.0);
            for line in build_ground_grid(pad_render, radial_render, grid_size, grid_step) {
                window.draw_polyline(&line);
            }
        }

        // 轨迹。
        if trail.len() >= 2 {
            trail_poly.vertices.clear();
            for p in &trail {
                trail_poly.vertices.push(frame.to_render([p.x, p.y, p.z]));
            }
            window.draw_polyline(&trail_poly);
        }

        // 速度矢量。
        {
            let v = rocket_state.vel;
            let v_mag = v.length();
            if v_mag > 1e-3 {
                let len = v_mag.min((cam_dist * 0.3) as f64) as f32;
                let end = frame.to_render([
                    rocket_state.pos.x + v.x * (len as f64 / v_mag),
                    rocket_state.pos.y + v.y * (len as f64 / v_mag),
                    rocket_state.pos.z + v.z * (len as f64 / v_mag),
                ]);
                window.draw_line(
                    sc_pos_render,
                    end,
                    Color::new(0.2, 1.0, 1.0, 1.0),
                    2.0,
                    false,
                );
            }
        }

        // === HUD ===
        let v_mag = rocket_state.speed();
        let r = rocket_state.pos;
        let r_unit = r * (1.0 / r.length().max(1e-3));
        let v_vert = dot(rocket_state.vel, r_unit);
        let v_horiz = (rocket_state.vel - r_unit * v_vert).length();

        let mut lines: Vec<String> = Vec::new();
        if h > 1000.0 {
            lines.push(format!("高度 AGL = {:.1} km", h / 1e3));
        } else {
            lines.push(format!("高度 AGL = {:.0} m", h));
        }
        lines.push(format!("速度 Vel = {:.0} m/s", v_mag));
        lines.push(format!(
            "垂直 {}  水平 {}  m/s",
            v_vert as i64, v_horiz as i64
        ));
        lines.push(format!(
            "俯仰 Pitch = {:.0}°",
            rocket_state.pitch.to_degrees()
        ));
        lines.push(format!("燃料 Fuel = {:.0}%", rocket_state.fuel));

        if thrusting {
            lines.push("[ 推力开启 THRUST ON ]".to_string());
        }
        if h < LOCK_ALT {
            lines.push("[ 低空垂直锁定 VERT LOCK ]".to_string());
        }

        let r_mag = r.length();
        let v_circular = (EARTH_GM / r_mag).sqrt();
        let energy = v_mag * v_mag / 2.0 - EARTH_GM / r_mag;
        let energy_margin = EARTH_GM / r_mag * 0.01;
        if v_horiz > v_circular * 0.5 && energy < -energy_margin {
            let el = Elements::calculate(rocket_state.pos, rocket_state.vel, EARTH_GM, 0.0);
            let ap_alt = (el.ap_dist() - EARTH_R) / 1e3;
            let pe_alt = (el.pe_dist() - EARTH_R) / 1e3;
            if pe_alt > -1000.0 {
                lines.push(format!("远地点 ApD = {:.0} km", ap_alt));
                lines.push(format!("近地点 PeD = {:.0} km", pe_alt));
            } else {
                lines.push(format!(
                    "远地点 ApD = {:.0} km（亚轨道 suborbital）",
                    ap_alt
                ));
            }
        } else if energy > energy_margin && v_mag > 100.0 {
            lines.push("（逃逸轨道 escape trajectory）".to_string());
        }

        if !crash_msg.is_empty() {
            lines.push(format!("!!! {} !!!", crash_msg));
        }

        let text_scale = 26.0_f32;
        let line_h = text_scale + 4.0;
        let fg = Color::new(0.95, 1.0, 0.95, 1.0);
        let bg = Color::new(0.0, 0.0, 0.0, 0.85);
        for (i, line) in lines.iter().enumerate() {
            let y = window.height() as f32 - 10.0 - (i as f32 + 1.0) * line_h;
            for &(dx, dy) in &[(1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0)] {
                window.draw_text(line, Vec2::new(10.0 + dx, y + dy), text_scale, &font, bg);
            }
            window.draw_text(line, Vec2::new(10.0, y), text_scale, &font, fg);
        }
    }

    eprintln!("退出。");
}

fn rand_flicker(frame: usize) -> f32 {
    let s = (frame as u32).wrapping_mul(2654435761);
    (s >> 24) as f32 / 256.0
}
