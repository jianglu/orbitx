#![allow(dead_code)]

//! orbitx 地球发射测试器：火箭从地表垂直起飞，受引力+大气+推力作用。
//!
//! 用法：cargo run -p orbitx-launch
//!
//! 操作：
//! - W（按住）：主发动机推力
//! - A/D：调整俯仰角（从竖直到水平）
//! - G：切换自动重力转向
//! - C：切换相机（Chase/Orbit）
//! - R：重置到发射台
//! - Space：暂停
//! - +/-：时间加速
//! - Escape：退出

use kiss3d::event::{Action, Key, WindowEvent};
use kiss3d::light::Light;
use kiss3d::prelude::*;
use kiss3d::text::Font;

use orbitx_dynamics::{rk4_step, Elements};
use orbitx_math::{cross, dot, Vec3 as Vec3d};

// === 地球参数 ===
const EARTH_R: f64 = 6_371_000.0; // 半径 [m]
const EARTH_MASS: f64 = 5.972e24; // 质量 [kg]
const EARTH_GM: f64 = orbitx_math::consts::GGRAV * EARTH_MASS; // GM

// === 大气参数 ===
const RHO0: f64 = 1.225; // 海平面空气密度 [kg/m³]
const SCALE_H: f64 = 8500.0; // 大气标高 [m]
const ATM_TOP: f64 = 100_000.0; // 大气层顶 [m]
const DRAG_COEFF: f64 = 0.005; // Cd*A/m [m²/kg]

// === 火箭参数 ===
const THRUST_ACCEL: f64 = 25.0; // 推力加速度 [m/s²]
const FUEL_RATE: f64 = 5.0; // 燃料消耗 [%/s]
const PITCH_RATE: f64 = 0.523_598_775_598_298_8; // 30° in radians

// === 渲染参数 ===
/// 1 个渲染单位 = 多少米（100 km → 地球 R ≈ 63.7 单位）。
const RENDER_SCALE: f64 = 1.0 / 100_000.0;

/// 将 f64 米坐标转为 kiss3d f32 渲染坐标。
/// 以地球中心为原点（不需要浮点原点，因为距离不会超过几万公里）。
fn to_render(pos: Vec3d) -> Vec3 {
    Vec3::new(
        (pos.x * RENDER_SCALE) as f32,
        (pos.y * RENDER_SCALE) as f32,
        (pos.z * RENDER_SCALE) as f32,
    )
}

/// 大气密度 [kg/m³]（高度 h 处）。
fn air_density(h: f64) -> f64 {
    if !(0.0..=ATM_TOP).contains(&h) {
        0.0
    } else {
        RHO0 * (-h / SCALE_H).exp()
    }
}

/// 从向量 from 到 to 的最短旋转四元数。
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

/// 火箭状态。
struct Rocket {
    pos: Vec3d,
    vel: Vec3d,
    /// 俯仰角：0 = 径向（竖直向上），π/2 = 切向（水平）。
    pitch: f64,
    fuel: f64,
}

impl Rocket {
    fn new() -> Self {
        // 从赤道地表出发。
        Rocket {
            pos: Vec3d::new(EARTH_R, 0.0, 0.0),
            vel: Vec3d::ZERO,
            pitch: 0.0,
            fuel: 100.0,
        }
    }

    /// 高度（距地表）。
    fn altitude(&self) -> f64 {
        self.pos.length() - EARTH_R
    }

    /// 速度大小。
    fn speed(&self) -> f64 {
        self.vel.length()
    }

    /// 推力方向（左手系，f64）。
    /// pitch=0 时沿径向（向外），pitch=π/2 时沿切向。
    fn thrust_dir(&self) -> Vec3d {
        let r = self.pos;
        let r_mag = r.length();
        if r_mag < 1e-3 {
            return Vec3d::new(1.0, 0.0, 0.0);
        }
        let radial = r * (1.0 / r_mag); // 向外
                                        // 切向方向：垂直于径向，在轨道面内。
                                        // 用速度方向投影到水平面来确定切向基准。
        let tangent_base = if self.vel.length() > 1e-3 {
            // 从速度中减去径向分量，剩下的就是切向。
            let v_radial = radial * dot(self.vel, radial);
            let v_tan = self.vel - v_radial;
            if v_tan.length() > 1e-3 {
                v_tan.unit()
            } else {
                // 速度纯径向，用一个任意垂直方向。
                let up = Vec3d::new(0.0, 1.0, 0.0);
                cross(radial, up).unit()
            }
        } else {
            let up = Vec3d::new(0.0, 1.0, 0.0);
            cross(radial, up).unit()
        };

        // 线性插值：pitch=0 → 纯径向，pitch=π/2 → 纯切向。
        let cos_p = self.pitch.cos();
        let sin_p = self.pitch.sin();
        (radial * cos_p + tangent_base * sin_p).unit()
    }
}

const TRAIL_MAX: usize = 5000;
const TRAIL_INTERVAL: usize = 2;

#[kiss3d::main]
async fn main() {
    eprintln!("orbitx 地球发射测试器");
    eprintln!("W 推力，A/D 俯仰，G 自动转向，C 相机，R 重置，Space 暂停，Esc 退出。");

    let mut rocket = Rocket::new();
    let _initial = Rocket::new();

    let mut window = Window::new("orbitx 发射").await;
    window.set_background_color(Color::new(0.0, 0.0, 0.02, 1.0));
    window.set_ambient(0.6);
    window.rebind_close_key(Some(Key::Escape));

    let font = Font::default();

    let mut scene = SceneNode3d::empty();
    scene.add_light(Light::directional(Vec3::new(-0.5, -0.3, -0.8)).with_intensity(5.0));
    scene
        .add_light(Light::point(500.0))
        .set_position(Vec3::new(0.0, 0.0, 100.0));

    // 地球。
    let earth_r_render = (EARTH_R * RENDER_SCALE) as f32;
    let _earth = scene
        .add_sphere(earth_r_render)
        .set_color(Color::new(0.15, 0.3, 0.6, 1.0));

    // 大气层边界线（h=100km 的圆环，用多段线近似）。
    let atm_r = ((EARTH_R + ATM_TOP) * RENDER_SCALE) as f32;
    let mut atm_line = Polyline3d::new(
        (0..128)
            .map(|i| {
                let a = (i as f32 / 128.0) * std::f32::consts::TAU;
                Vec3::new(atm_r * a.cos(), 0.0, atm_r * a.sin())
            })
            .collect(),
    )
    .with_color(Color::new(0.2, 0.6, 0.3, 0.4))
    .with_width(1.5);
    atm_line.perspective = false;

    // 火箭造型。
    let mut sc_node = scene.add_group();
    let mut nose = sc_node
        .add_cone(0.08, 0.20)
        .set_color(Color::new(0.9, 0.9, 0.95, 1.0));
    nose.set_position(Vec3::new(0.0, 0.0, 0.15));
    nose.set_rotation(Quat::from_axis_angle(Vec3::X, -std::f32::consts::FRAC_PI_2));
    let mut body = sc_node
        .add_cylinder(0.07, 0.25)
        .set_color(Color::new(0.6, 0.7, 0.8, 1.0));
    body.set_rotation(Quat::from_axis_angle(Vec3::X, -std::f32::consts::FRAC_PI_2));
    let mut wing_l = sc_node
        .add_cube(0.25, 0.015, 0.12)
        .set_color(Color::new(0.4, 0.5, 0.6, 1.0));
    wing_l.set_position(Vec3::new(-0.13, 0.0, -0.04));
    let mut wing_r = sc_node
        .add_cube(0.25, 0.015, 0.12)
        .set_color(Color::new(0.4, 0.5, 0.6, 1.0));
    wing_r.set_position(Vec3::new(0.13, 0.0, -0.04));

    // 轨迹。
    let mut trail: Vec<Vec3d> = Vec::with_capacity(TRAIL_MAX);
    let mut trail_poly = Polyline3d::new(vec![Vec3::ZERO])
        .with_color(Color::new(0.3, 1.0, 0.4, 0.9))
        .with_width(1.5);
    trail_poly.perspective = false;

    // 状态。
    let mut camera = OrbitCamera3d::new(Vec3::new(0.0, 2.0, 6.0), Vec3::ZERO);
    let mut paused = false;
    let mut last_instant = std::time::Instant::now();
    let mut frame_count = 0usize;
    let mut time_scale: f64 = 10.0;
    let mut keys_w = false;
    let mut keys_a = false;
    let mut keys_d = false;
    let mut chase_cam = true;
    let mut auto_gravity_turn = false;
    let mut crash_msg = String::new();
    let mut crash_timer = 0.0_f64;

    while window.render_3d(&mut scene, &mut camera).await {
        let now = std::time::Instant::now();
        let dt_real = now.duration_since(last_instant).as_secs_f64();
        last_instant = now;

        // 事件处理。
        for mut event in window.events().iter() {
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
                        rocket = Rocket::new();
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
                        eprintln!(
                            "自动重力转向: {}",
                            if auto_gravity_turn { "开" } else { "关" }
                        );
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

            // 俯仰角控制。
            if auto_gravity_turn {
                // 自动重力转向：h < 10km 保持垂直，之后逐渐转水平。
                let h = rocket.altitude();
                if h > 10_000.0 {
                    let target_pitch =
                        ((h - 10_000.0) / 70_000.0).min(1.0) * std::f64::consts::FRAC_PI_2;
                    if rocket.pitch < target_pitch {
                        rocket.pitch += PITCH_RATE * dt;
                        if rocket.pitch > target_pitch {
                            rocket.pitch = target_pitch;
                        }
                    }
                }
            } else {
                // 手动控制。
                if keys_a {
                    rocket.pitch -= PITCH_RATE * dt;
                }
                if keys_d {
                    rocket.pitch += PITCH_RATE * dt;
                }
            }
            rocket.pitch = rocket.pitch.clamp(0.0, std::f64::consts::FRAC_PI_2);

            // 推力 + 燃料。
            let thrusting = keys_w && rocket.fuel > 0.0;
            if thrusting {
                rocket.fuel -= FUEL_RATE * dt;
                if rocket.fuel < 0.0 {
                    rocket.fuel = 0.0;
                }
            }

            let thrust_dir = rocket.thrust_dir();
            let h0 = rocket.altitude();

            // RK4 积分。
            let n_sub = 10;
            let sub_dt = dt / n_sub as f64;
            for _ in 0..n_sub {
                let pos = rocket.pos;
                let vel = rocket.vel;
                let td = thrust_dir;
                let thrust = thrusting;
                let mut force = move |s: &orbitx_math::StateVectors, _t: f64| {
                    let r = s.pos;
                    let r_mag = r.length();
                    // 重力加速度。
                    let g_acc = r * (EARTH_GM / (r_mag * r_mag * r_mag));
                    // 大气阻力。
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
                    // 推力。
                    let thrust_acc = if thrust {
                        td * THRUST_ACCEL
                    } else {
                        Vec3d::ZERO
                    };
                    let acc = g_acc + drag_acc + thrust_acc;
                    (acc, Vec3d::ZERO)
                };

                let sv = orbitx_math::StateVectors {
                    pos,
                    vel,
                    ..Default::default()
                };
                let next = rk4_step(sv, sub_dt, &mut force);
                rocket.pos = next.pos;
                rocket.vel = next.vel;
            }

            // 碰撞检测：高度 < 0 且速度大 → 坠毁；高度 < 0 且速度小 → 着陆。
            let h1 = rocket.altitude();
            if h1 < 0.0 {
                if rocket.speed() > 50.0 {
                    crash_msg = format!("坠毁！速度 {} m/s", rocket.speed() as u64);
                    crash_timer = 3.0;
                    eprintln!("{}", crash_msg);
                    rocket = Rocket::new();
                    trail.clear();
                } else {
                    // 安全着陆：贴在地表。
                    let r_mag = rocket.pos.length();
                    rocket.pos *= EARTH_R / r_mag;
                    rocket.vel = Vec3d::ZERO;
                }
            }

            // 采样轨迹。
            if frame_count % TRAIL_INTERVAL == 0 {
                trail.push(rocket.pos);
                if trail.len() > TRAIL_MAX {
                    trail.remove(0);
                }
            }

            crash_timer -= dt_real;
            if crash_timer <= 0.0 {
                crash_msg.clear();
            }

            let _ = h0;
        }

        frame_count += 1;

        // === 渲染更新 ===

        // 火箭位置。
        let sc_pos_render = to_render(rocket.pos);
        sc_node.set_position(sc_pos_render);

        // 火箭朝向：+Z 对齐推力方向。
        let thrust_render_end = to_render(rocket.pos + rocket.thrust_dir());
        let thrust_dir_render = (thrust_render_end - sc_pos_render).normalize_or_zero();
        if let Some(rot) = quat_from_to(Vec3::new(0.0, 0.0, 1.0), thrust_dir_render) {
            sc_node.set_rotation(rot);
        }

        // 相机。
        if chase_cam {
            let eye = sc_pos_render - thrust_dir_render * 3.0 + Vec3::new(0.0, 1.5, 0.0);
            camera = OrbitCamera3d::new(eye, sc_pos_render);
        } else {
            // Orbit 模式：看地球全貌。
            let r_render = (EARTH_R * RENDER_SCALE) as f32;
            camera = OrbitCamera3d::new(Vec3::new(0.0, r_render * 1.5, r_render * 3.0), Vec3::ZERO);
        }

        // 绘制大气层线。
        window.draw_polyline(&atm_line);

        // 绘制轨迹。
        if trail.len() >= 2 {
            trail_poly.vertices.clear();
            for p in &trail {
                trail_poly.vertices.push(to_render(*p));
            }
            window.draw_polyline(&trail_poly);
        }

        // 绘制速度矢量（青色）。
        {
            let v = rocket.vel;
            let v_mag = v.length();
            if v_mag > 1e-3 {
                let scale = (50_000.0 / v_mag.max(1.0)).min(1.0) * 3.0; // 最多 3 单位
                let end = to_render(rocket.pos + v * (scale / v_mag * 50_000.0));
                window.draw_line(
                    sc_pos_render,
                    end,
                    Color::new(0.2, 1.0, 1.0, 1.0),
                    2.0,
                    false,
                );
            }
        }

        // 绘制推力矢量（黄色）。
        if keys_w && rocket.fuel > 0.0 {
            let exh = rocket.thrust_dir() * (-1.0);
            let end = to_render(rocket.pos + exh * 200_000.0);
            window.draw_line(
                sc_pos_render,
                end,
                Color::new(1.0, 0.8, 0.2, 1.0),
                3.0,
                false,
            );
        }

        // === HUD ===
        let h = rocket.altitude();
        let v_mag = rocket.speed();
        let r = rocket.pos;
        let r_unit = r * (1.0 / r.length().max(1e-3));
        let v_vert = dot(rocket.vel, r_unit); // 径向（垂直）速度
        let v_horiz = (rocket.vel - r_unit * v_vert).length(); // 水平速度

        // 轨道根数（相对地球中心）。
        let mut lines: Vec<String> = Vec::new();
        lines.push(format!("Alt = {:.1} km", h / 1e3));
        lines.push(format!("Vel = {:.0} m/s", v_mag));
        lines.push(format!(
            "Vvert = {:.0} m/s   Vhoriz = {:.0} m/s",
            v_vert, v_horiz
        ));
        lines.push(format!("Pitch = {:.0}°", rocket.pitch.to_degrees()));
        lines.push(format!("Fuel = {:.0}%", rocket.fuel));

        // 轨道参数。
        let energy = v_mag * v_mag / 2.0 - EARTH_GM / r.length();
        if energy < 0.0 {
            let el = Elements::calculate(rocket.pos, rocket.vel, EARTH_GM, 0.0);
            let ap_alt = (el.ap_dist() - EARTH_R) / 1e3;
            let pe_alt = (el.pe_dist() - EARTH_R) / 1e3;
            lines.push(format!("ApD = {:.0} km", ap_alt));
            lines.push(format!("PeD = {:.0} km", pe_alt));
            let t_min = el.orbit_t() / 60.0;
            if t_min > 0.0 && t_min < 1e8 {
                lines.push(format!("T = {:.0} min", t_min));
            }
        } else {
            lines.push("(逃逸轨道)".to_string());
        }

        if !crash_msg.is_empty() {
            lines.push(format!("!!! {} !!!", crash_msg));
        }

        if auto_gravity_turn {
            lines.push("[自动重力转向]".to_string());
        }

        let text_scale = 26.0_f32;
        let line_h = text_scale + 4.0;
        let color = Color::new(0.9, 1.0, 0.9, 1.0);
        for (i, line) in lines.iter().enumerate() {
            let y = window.height() as f32 - 10.0 - (i as f32 + 1.0) * line_h;
            window.draw_text(line, Vec2::new(10.0, y), text_scale, &font, color);
        }
    }

    eprintln!("退出。");
}
