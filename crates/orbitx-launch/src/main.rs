#![allow(dead_code)]

//! orbitx 地球发射测试器：火箭从地表发射场起飞，受引力+大气+推力作用。
//!
//! 用法：cargo run -p orbitx-launch
//!
//! 操作：
//! - W（按住）：主发动机推力
//! - A/D：调整俯仰角（低空锁定，10km 后解锁）
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

// === 地球参数 ===
const EARTH_R: f64 = 6_371_000.0;
const EARTH_MASS: f64 = 5.972e24;
const EARTH_GM: f64 = orbitx_math::consts::GGRAV * EARTH_MASS;

// === 大气参数 ===
const RHO0: f64 = 1.225;
const SCALE_H: f64 = 8500.0;
const ATM_TOP: f64 = 100_000.0;
const DRAG_COEFF: f64 = 0.005;

// === 火箭参数 ===
const THRUST_ACCEL: f64 = 25.0;
const FUEL_RATE: f64 = 5.0;
const PITCH_RATE: f64 = 0.523_598_775_598_298_8; // 30°/s
const LOCK_ALT: f64 = 10_000.0; // 低空锁定高度

// === 渲染参数 ===
const RENDER_SCALE: f64 = 1.0 / 100_000.0;

/// 将 orbitx f64 位置（米，左手系）转为 kiss3d f32 位置（右手系 Y-up）。
/// 左手→右手转换：kiss3d.x = orbitx.x, kiss3d.y = orbitx.z, kiss3d.z = -orbitx.y
fn to_render(pos: Vec3d) -> Vec3 {
    Vec3::new(
        (pos.x * RENDER_SCALE) as f32,
        (pos.z * RENDER_SCALE) as f32,
        (-pos.y * RENDER_SCALE) as f32,
    )
}

/// 将 orbitx f64 方向向量（左手系）转为 kiss3d f32 方向（右手系 Y-up）。
/// 与 to_render 相同的轴变换，但不做缩放（方向向量不依赖长度）。
fn dir_to_render(dir: Vec3d) -> Vec3 {
    Vec3::new(dir.x as f32, dir.z as f32, -dir.y as f32)
}

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

/// 发射场位置（赤道）。
const LAUNCH_POS: Vec3d = Vec3d::new(EARTH_R, 0.0, 0.0);

impl Rocket {
    fn on_pad() -> Self {
        Rocket {
            pos: LAUNCH_POS,
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
            return Vec3d::new(1.0, 0.0, 0.0);
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

/// 构建火箭模型：返回 (group 节点, 火焰节点)。
///
/// 火箭沿 +Y 轴（与 kiss3d OrbitCamera3d 的 up 轴一致）：
///   头部在 +Y 端，尾部（喷管/火焰）在 -Y 端。
///
/// kiss3d cone(r,h) 和 cylinder(r,h) 默认沿 Y 轴，无需旋转！
///   cone：尖端在 y=+h/2，底面在 y=-h/2。
///   cylinder：中心在原点，沿 y 从 -h/2 到 +h/2。
fn build_rocket(scene: &mut SceneNode3d) -> (SceneNode3d, SceneNode3d) {
    let mut rocket = scene.add_group();

    // Y 轴布局（无需旋转，各部件中心位置）：
    //   y=+0.35  头部锥中心 (h=0.30, 尖端 y=+0.50, 底面 y=+0.20)
    //   y=+0.05  上段圆柱中心 (h=0.30, y 从 -0.10 到 +0.20)
    //   y=-0.12  红色条纹中心 (h=0.03)
    //   y=-0.21  下段圆柱中心 (h=0.34, y 从 -0.38 到 -0.04)
    //   y=-0.43  喷管中心 (h=0.10, 尖端 y=-0.48, 底面 y=-0.38)
    //   y=-0.58  火焰中心 (h=0.20, 尖端 y=-0.68, 底面 y=-0.48)

    // --- 头部锥（红色，尖端朝 +Y，无需旋转） ---
    let mut nose = rocket
        .add_cone(0.055, 0.30)
        .set_color(Color::new(0.85, 0.15, 0.1, 1.0));
    nose.set_position(Vec3::new(0.0, 0.35, 0.0));

    // --- 上段主体（白色圆柱，无需旋转） ---
    let mut upper = rocket
        .add_cylinder(0.055, 0.30)
        .set_color(Color::new(0.92, 0.92, 0.92, 1.0));
    upper.set_position(Vec3::new(0.0, 0.05, 0.0));

    // --- 红色条纹 ---
    let mut stripe = rocket
        .add_cylinder(0.057, 0.03)
        .set_color(Color::new(0.85, 0.15, 0.1, 1.0));
    stripe.set_position(Vec3::new(0.0, -0.12, 0.0));

    // --- 下段主体（白色，稍粗） ---
    let mut lower = rocket
        .add_cylinder(0.065, 0.34)
        .set_color(Color::new(0.85, 0.85, 0.85, 1.0));
    lower.set_position(Vec3::new(0.0, -0.21, 0.0));

    // --- 发动机喷管（深灰色，尖端朝 -Y，翻转 180°） ---
    let mut nozzle = rocket
        .add_cone(0.045, 0.10)
        .set_color(Color::new(0.2, 0.2, 0.23, 1.0));
    nozzle.set_position(Vec3::new(0.0, -0.43, 0.0));
    nozzle.set_rotation(Quat::from_axis_angle(Vec3::X, std::f32::consts::PI));

    // --- 4 片尾翼 ---
    let fin_color = Color::new(0.3, 0.3, 0.35, 1.0);
    for i in 0..4u32 {
        let angle = std::f32::consts::FRAC_PI_2 * i as f32;
        let mut fin = rocket.add_cube(0.10, 0.15, 0.01).set_color(fin_color);
        // 尾翼在 XZ 平面径向排列，Y 偏移到下段。
        let radial = Vec3::new(angle.cos(), 0.0, angle.sin());
        fin.set_position(radial * 0.065 + Vec3::new(0.0, -0.25, 0.0));
    }

    // --- 火焰节点（推力时可见，尖端朝 -Y，翻转 180°） ---
    let mut flame = rocket
        .add_cone(0.04, 0.20)
        .set_color(Color::new(1.0, 0.65, 0.15, 0.85));
    flame.set_position(Vec3::new(0.0, -0.58, 0.0));
    flame.set_rotation(Quat::from_axis_angle(Vec3::X, std::f32::consts::PI));
    flame.set_surface_rendering_activation(false);

    (rocket, flame)
}

#[kiss3d::main]
async fn main() {
    eprintln!("orbitx 地球发射测试器");
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

    let mut window = Window::new("orbitx 发射").await;
    window.set_background_color(Color::new(0.0, 0.0, 0.02, 1.0));
    window.set_ambient(0.6);
    window.rebind_close_key(Some(Key::Escape));

    let font = orbitx_scene::load_cjk_font();

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

    // 大气层边界线。
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

    // 发射场标记：扁圆柱平台。
    let pad_render = to_render(LAUNCH_POS);
    let mut pad = scene
        .add_cylinder(0.15, 0.02)
        .set_color(Color::new(0.5, 0.5, 0.55, 1.0));
    pad.set_position(pad_render);
    // 平台法线对齐径向。
    let radial_f32 = pad_render.normalize_or_zero();
    if let Some(rot) = quat_from_to(Vec3::Y, radial_f32) {
        pad.set_rotation(rot);
    }

    // 发射场圆环标记。
    let pad_ring_r = (5_000.0 * RENDER_SCALE) as f32; // 5km 半径标记
    let mut pad_ring = Polyline3d::new(
        (0..64)
            .map(|i| {
                let a = (i as f32 / 64.0) * std::f32::consts::TAU;
                let local = Vec3::new(pad_ring_r * a.cos(), 0.0, pad_ring_r * a.sin());
                pad_render + local
            })
            .collect(),
    )
    .with_color(Color::new(0.7, 0.6, 0.2, 0.6))
    .with_width(2.0);
    pad_ring.perspective = false;

    // 火箭 + 火焰。
    let (mut sc_node, mut flame_node) = build_rocket(&mut scene);

    // 轨迹。
    let mut trail: Vec<Vec3d> = Vec::with_capacity(TRAIL_MAX);
    let mut trail_poly = Polyline3d::new(vec![Vec3::ZERO])
        .with_color(Color::new(0.3, 1.0, 0.4, 0.9))
        .with_width(1.5);
    trail_poly.perspective = false;

    let mut camera = OrbitCamera3d::new(Vec3::new(0.0, 1.0, 3.0), Vec3::ZERO);

    while window.render_3d(&mut scene, &mut camera).await {
        let now = std::time::Instant::now();
        let dt_real = now.duration_since(last_instant).as_secs_f64();
        last_instant = now;

        for mut event in window.events().iter() {
            // Chase 模式下拦截鼠标事件，防止 OrbitCamera 默认处理与每帧
            // 重建相机冲突导致画面抖动。
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
            let h = rocket_state.altitude();

            // 俯仰角控制。
            if h < LOCK_ALT {
                // 低空锁定：强制垂直。
                rocket_state.pitch = 0.0;
            } else if auto_gravity_turn {
                let target_pitch =
                    ((h - LOCK_ALT) / 70_000.0).min(1.0) * std::f64::consts::FRAC_PI_2;
                if rocket_state.pitch < target_pitch {
                    rocket_state.pitch += PITCH_RATE * dt;
                    if rocket_state.pitch > target_pitch {
                        rocket_state.pitch = target_pitch;
                    }
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

            // 推力 + 燃料。
            let thrusting = keys_w && rocket_state.fuel > 0.0;
            if thrusting {
                rocket_state.fuel -= FUEL_RATE * dt;
                if rocket_state.fuel < 0.0 {
                    rocket_state.fuel = 0.0;
                }
            }

            let thrust_dir = rocket_state.thrust_dir();

            // RK4 积分。
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

            // 碰撞检测。
            let h1 = rocket_state.altitude();
            if h1 < 0.0 {
                if rocket_state.speed() > 50.0 {
                    crash_msg = format!("坠毁！速度 {} m/s", rocket_state.speed() as u64);
                    crash_timer = 3.0;
                    eprintln!("{}", crash_msg);
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

        let sc_pos_render = to_render(rocket_state.pos);
        sc_node.set_position(sc_pos_render);

        // 火箭朝向：将模型 +Z（头锥方向）对齐到渲染系的"上"（径向方向）。
        // 发射台上径向 = (1,0,0)，但为了让火箭在屏幕上竖直显示，
        // 不旋转模型（保持 +Z 朝上），由相机角度决定观察方向。
        let _thrust_dir_render = dir_to_render(rocket_state.thrust_dir()).normalize_or_zero();
        // 不旋转：模型保持 +Z 朝向，在默认相机视角中 +Z 就是屏幕上方。
        // 物理方向由 thrust_dir 计算，但视觉上火箭始终竖直显示。

        // 火焰效果：推力时显示并抖动。
        let thrusting = keys_w && rocket_state.fuel > 0.0;
        if thrusting {
            flame_node.set_surface_rendering_activation(true);
            // 火焰长度随机抖动。
            let flicker = 0.7 + rand_flicker(frame_count) * 0.5;
            flame_node.set_local_scale(1.0, flicker, 1.0);
        } else {
            flame_node.set_surface_rendering_activation(false);
        }

        // 相机：Chase 模式。
        // OrbitCamera3d up 轴 = +Y（源码确认）。
        // 火箭模型头锥朝 +Y（与相机 up 一致 = 屏幕上方）。
        // 相机放在 -Z 方向（正面），看 +Z。
        // screen_right = +X（径向 = 天空方向），screen_up = +Y（头锥方向）。
        // 地心方向（-X）→ 屏幕左侧，但因为火箭位置在 +X，地心实际在下方。
        if chase_cam {
            let dist = 3.0;
            let eye = sc_pos_render + Vec3::new(0.0, 0.0, -dist);
            camera = OrbitCamera3d::new(eye, sc_pos_render);
        } else {
            let r_render = (EARTH_R * RENDER_SCALE) as f32;
            camera = OrbitCamera3d::new(Vec3::new(0.0, r_render * 1.5, r_render * 3.0), Vec3::ZERO);
        }

        // 绘制大气层线 + 发射场圆环。
        window.draw_polyline(&atm_line);
        window.draw_polyline(&pad_ring);

        // 绘制轨迹。
        if trail.len() >= 2 {
            trail_poly.vertices.clear();
            for p in &trail {
                trail_poly.vertices.push(to_render(*p));
            }
            window.draw_polyline(&trail_poly);
        }

        // 速度矢量。
        {
            let v = rocket_state.vel;
            let v_mag = v.length();
            if v_mag > 1e-3 {
                let scale = (50_000.0 / v_mag.max(1.0)).min(1.0) * 3.0;
                let end = to_render(rocket_state.pos + v * (scale / v_mag * 50_000.0));
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
        let h = rocket_state.altitude();
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

        // 轨道根数：仅在水平速度足够大（真正进入轨道飞行）时才显示。
        // 垂直爬升阶段（水平速度低）轨道根数无意义且不稳定。
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
        let color = Color::new(0.9, 1.0, 0.9, 1.0);
        for (i, line) in lines.iter().enumerate() {
            let y = window.height() as f32 - 10.0 - (i as f32 + 1.0) * line_h;
            window.draw_text(line, Vec2::new(10.0, y), text_scale, &font, color);
        }
    }

    eprintln!("退出。");
}

/// 简单伪随机抖动（基于帧号），用于火焰长度。
fn rand_flicker(frame: usize) -> f32 {
    let s = (frame as u32).wrapping_mul(2654435761);
    (s >> 24) as f32 / 256.0
}
