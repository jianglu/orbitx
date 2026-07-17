//! Main application - window management, wgpu init, simulation loop, egui integration.

use std::num::NonZeroU32;
use std::sync::Arc;
use winit::{
    application::ApplicationHandler,
    event::{ElementState, KeyEvent, MouseButton, MouseScrollDelta, WindowEvent},
    event_loop::ActiveEventLoop,
    window::{Window, WindowAttributes},
};

use orbitx_math::vec3::Vec3;
use orbitx_render::{
    CameraSystem, CoordinateBridge, ExternalCamMode,
    SceneManager,
};
use orbitx_dynamics::PlanetarySystem;
use orbitx_gfx_hud::{FlightState, HudState, MfdPanel, MfdType, MfdSize};
use crate::flight_calc::{compute_flight_state, ParentBody};
use crate::input::{Action, key_to_action};
use crate::scene_renderer::{FrameScene, SceneCallback, SceneRenderer};
use crate::vessel::UserVessel;
use crate::ephem_bridge;

pub struct App {
    window: Option<Arc<Window>>,
    egui_ctx: egui::Context,
    painter: Option<egui_wgpu::winit::Painter>,
    egui_state: Option<egui_winit::State>,
    scene_renderer: Option<SceneRenderer>,
    camera: CameraSystem,
    coord_bridge: CoordinateBridge,
    scene: SceneManager,
    planetary: Option<PlanetarySystem>,
    has_ephemeris: bool,
    hud: HudState,
    mfd_left: MfdPanel,
    mfd_right: MfdPanel,
    flight_state: FlightState,
    /// Simulated user vessel (LEO by default; drives HUD/MFD).
    vessel: Option<UserVessel>,
    /// Scene index of the vessel node (None until spawned).
    vessel_node_idx: Option<usize>,
    sim_time: f64,
    time_warp: f64,
    paused: bool,
    dt: f64,
    focus_body: usize,
    running: bool,
    last_mouse_pos: Option<(f64, f64)>,
    /// True while the left mouse button is held (gates camera orbit drag).
    dragging: bool,
}

impl App {
    pub fn new() -> Self {
        // Frame the inner solar system: camera pulled back on the -x side
        // looking toward the Sun (target 0). At solar-system scale, planets
        // render as billboards. dist=6e11 m (~4 AU) with 60deg FOV covers
        // ~2.3 AU laterally, enough to see Mercury..Mars around the Sun.
        let mut camera = CameraSystem::new();
        camera.target = 0;
        camera.set_ext_mode(ExternalCamMode::TargetRelative {
            dist: 6.0e11,
            phi: std::f64::consts::PI,
            theta: 0.2,
        });
        Self {
            window: None,
            egui_ctx: egui::Context::default(),
            painter: None,
            egui_state: None,
            scene_renderer: None,
            camera,
            coord_bridge: CoordinateBridge::new_solar_system(20.0),
            scene: SceneManager::new(),
            planetary: None,
            has_ephemeris: false,
            hud: HudState::new(),
            mfd_left: MfdPanel::new(MfdType::Orbit, MfdSize::Left),
            mfd_right: MfdPanel::new(MfdType::Map, MfdSize::Right),
            flight_state: FlightState::default(),
            vessel: None,
            vessel_node_idx: None,
            sim_time: 0.0, time_warp: 1.0, paused: false, dt: 0.016,
            focus_body: 0, running: true, last_mouse_pos: None,
            dragging: false,
        }
    }

    fn init_scene(&mut self) {
        let orbiter_src = ephem_bridge::resolve_orbiter_src();
        let psys = ephem_bridge::create_planetary_system(&orbiter_src);
        self.has_ephemeris = psys.bodies.iter().any(|b| b.ephemeris.is_some());
        self.scene = ephem_bridge::create_scene_from_psys(&psys);

        // Spawn the user vessel in a 400 km circular LEO around Earth (if present).
        self.vessel = psys.bodies.iter().position(|b| b.name == "Earth").map(|idx| {
            let b = &psys.bodies[idx];
            UserVessel::leo(idx, b.radius_m, b.gm())
        });

        // Attach the vessel as a scene node so it's visible (cyan marker).
        // Its position is synced each frame from parent.pos + vessel.rel_pos.
        if self.vessel.is_some() {
            let idx = ephem_bridge::add_vessel_node(
                &mut self.scene,
                "user_vessel",
                [0.35, 0.95, 1.0, 1.0], // cyan
                40.0,                    // 40 m characteristic length
            );
            self.vessel_node_idx = Some(idx);
        }

        self.planetary = Some(psys);
    }

    fn handle_action(&mut self, action: Action) {
        match action {
            Action::CamModeNext => {
                let hint = (self.focus_body + 1) % self.scene.len().max(1);
                self.camera.cycle_ext_mode(true, hint);
            }
            Action::CamModePrev => {
                let hint = (self.focus_body + 1) % self.scene.len().max(1);
                self.camera.cycle_ext_mode(false, hint);
            }
            Action::CamModeSet(idx) => {
                // 直接跳到目标模式：先循环到 0，再前进 idx 次
                let hint = (self.focus_body + 1) % self.scene.len().max(1);
                for _ in 0..6 {
                    if self.camera.ext_mode_index() == 0 { break; }
                    self.camera.cycle_ext_mode(true, hint);
                }
                for _ in 0..(idx as usize) {
                    self.camera.cycle_ext_mode(true, hint);
                }
            }
            Action::CamToggleInternal => {
                self.camera.toggle_internal();
            }
            Action::CamCycleDirref => {
                let n = self.scene.len().max(1);
                let cur = self.camera.current_dist().map(|_| 0).unwrap_or(0);
                let _ = cur;
                // 从当前 dirref +1 循环
                let next = match self.camera.ext_mode {
                    orbitx_render::ExternalCamMode::TargetToObject { dirref, .. }
                    | orbitx_render::ExternalCamMode::TargetFromObject { dirref, .. } => {
                        (dirref + 1) % n
                    }
                    _ => (self.focus_body + 1) % n,
                };
                self.camera.set_dirref(next);
            }
            Action::CamGroundObserver => {
                self.camera.set_ext_mode(orbitx_render::ExternalCamMode::GroundObserver {
                    lng: 0.0, lat: 0.0, alt: 1.0e6,
                    terrain_follow: false, target_lock: None,
                });
            }
            Action::HudModeNext => self.hud.next_mode(),
            Action::HudColorNext => self.hud.next_color(),
            Action::MfdLeftNext => self.mfd_left.next_type(),
            Action::MfdRightNext => self.mfd_right.next_type(),
            Action::TimeWarpUp => self.time_warp = (self.time_warp * 2.0).min(1e6),
            Action::TimeWarpDown => self.time_warp = (self.time_warp / 2.0).max(0.125),
            Action::TimePause => self.paused = !self.paused,
            Action::ThrottleUp => {
                if let Some(v) = &mut self.vessel {
                    v.throttle = (v.throttle + 0.05).min(1.0);
                }
            }
            Action::ThrottleDown => {
                if let Some(v) = &mut self.vessel {
                    v.throttle = (v.throttle - 0.05).max(0.0);
                }
            }
            Action::ThrottleFull => {
                if let Some(v) = &mut self.vessel { v.throttle = 1.0; }
            }
            Action::ThrottleCut => {
                if let Some(v) = &mut self.vessel { v.throttle = 0.0; }
            }
            Action::FocusNextBody => {
                self.focus_body = (self.focus_body + 1) % self.scene.len().max(1);
                self.camera.target = self.focus_body;
            }
            Action::FocusPrevBody => {
                let len = self.scene.len().max(1);
                self.focus_body = if self.focus_body == 0 { len - 1 } else { self.focus_body - 1 };
                self.camera.target = self.focus_body;
            }
            Action::Quit => self.running = false,
            _ => {}
        }
    }

    fn render(&mut self) {
        let painter = match &mut self.painter { Some(p) => p, None => return };
        let egui_state = match &mut self.egui_state { Some(s) => s, None => return };
        let window = match &self.window { Some(w) => w, None => return };

        // Build per-frame scene data for the 3D renderer
        if let Some(sr) = &mut self.scene_renderer {
            let vp = window.inner_size();
            let viewport_size = [vp.width as f32, vp.height as f32];
            let mut frame = FrameScene::from_scene(&self.camera, &self.scene, viewport_size);
            frame.line_vertices = crate::scene_renderer::build_scene_lines(&self.scene, &self.coord_bridge);
            frame.time = self.sim_time as f32;
            sr.set_frame(frame);
        }

        let egui_input = egui_state.take_egui_input(window);
        let full_output = self.egui_ctx.run_ui(egui_input, |ui| {
            // 3D scene callback behind all UI
            if self.scene_renderer.is_some() {
                let rect = ui.max_rect();
                let callback = egui_wgpu::Callback::new_paint_callback(
                    rect,
                    SceneCallback,
                );
                ui.painter().add(callback);
            }

            // Central panel with transparent frame so 3D scene shows through
            egui::CentralPanel::default()
                .frame(egui::Frame::new().inner_margin(8).fill(egui::Color32::TRANSPARENT))
                .show(ui, |ui| {
                    // Cockpit bezel overlay when internal view is active
                    if self.camera.is_internal {
                        draw_cockpit_bezel(ui);
                    }
                    self.hud.draw(ui, &self.flight_state);
                });

            // Right info panel with semi-transparent background
            egui::Panel::right("info_panel")
                .frame(egui::Frame::new().inner_margin(8).fill(egui::Color32::from_black_alpha(180)))
                .show(ui, |ui| {
                ui.heading("orbitx");
                ui.separator();
                ui.label(format!("Time: {:.1} s", self.sim_time));
                ui.label(format!("Warp: {:.0}x", self.time_warp));
                ui.label(if self.paused { "PAUSED" } else { "RUNNING" });
                ui.separator();
                ui.label(format!("Focus: body #{}", self.focus_body));
                ui.label(if self.has_ephemeris { "Ephemeris: LIVE" } else { "Ephemeris: NONE" });
                ui.label(format!("Alt: {}", self.flight_state.fmt_altitude()));
                ui.label(format!("Spd: {}", self.flight_state.fmt_speed()));
                if let Some(v) = &self.vessel {
                    ui.label(format!("Thr: {:>3.0}%   fuel {:.0} kg",
                        v.throttle * 100.0, v.fuel_mass));
                }
                ui.separator();
                let view_label = if self.camera.is_internal { "INT" } else { "EXT" };
                ui.label(format!("Cam[{}] {}", view_label, self.camera.ext_mode.name()));
                ui.label(format!("  {}", self.camera.ext_mode.short_params()));
                ui.label(format!("Near/Far: {:.2e} / {:.2e} m",
                    self.camera.near_plane, self.camera.log_depth.far));
                ui.separator();
                ui.label(format!("HUD: {} ({:?})", self.hud.mode.name(), self.hud.color));
                ui.label(format!("MFD-L: {}", self.mfd_left.mfd_type.name()));
                ui.label(format!("MFD-R: {}", self.mfd_right.mfd_type.name()));
                ui.separator();
                ui.label("Controls:");
                ui.label("WASD: Camera orbit");
                ui.label("Q/E: Zoom in/out");
                ui.label("Tab: Cam mode next");
                ui.label("1-6: Cam mode select");
                ui.label("V: Internal/External");
                ui.label("R: Cycle dirref");
                ui.label("G: Ground observer");
                ui.label("[/]: Focus body (incl. vessel)");
                ui.label("H: HUD mode  C: HUD color");
                ui.label("O/M: MFD type");
                ui.label(",/.: Time warp");
                ui.label("\u{2191}/\u{2193}: Throttle  0/`: Full/Cut");
                ui.label("Space: Pause");
            });
            self.mfd_left.draw(ui, &self.flight_state);
            self.mfd_right.draw(ui, &self.flight_state);
        });

        egui_state.handle_platform_output(window, full_output.platform_output);

        // Inject scene_renderer and queue into callback_resources before painting
        if let Some(sr) = &self.scene_renderer {
            if let Some(rs) = painter.render_state() {
                let mut renderer = rs.renderer.write();
                renderer.callback_resources.insert(sr.clone());
                renderer.callback_resources.insert(rs.queue.clone());
            }
        }

        let clipped_primitives = self.egui_ctx.tessellate(
            full_output.shapes,
            window.scale_factor() as f32,
        );

        painter.paint_and_update_textures(
            egui::viewport::ViewportId::ROOT,
            window.scale_factor() as f32,
            [0.0, 0.0, 0.02, 1.0],
            &clipped_primitives,
            &full_output.textures_delta,
            vec![],
            window,
        );
    }
}

impl ApplicationHandler for App {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if self.window.is_none() {
            let window_attrs = WindowAttributes::default()
                .with_title("orbitx - Space Flight Simulator")
                .with_inner_size(winit::dpi::LogicalSize::new(1280, 720));
            let window = Arc::new(event_loop.create_window(window_attrs).unwrap());

            let egui_state = egui_winit::State::new(
                self.egui_ctx.clone(),
                egui::viewport::ViewportId::ROOT,
                &window,
                None,
                None,
                None,
            );

            let mut painter = pollster::block_on(egui_wgpu::winit::Painter::new(
                self.egui_ctx.clone(),
                egui_wgpu::WgpuConfiguration::default(),
                false,
                egui_wgpu::RendererOptions {
                    depth_stencil_format: Some(wgpu::TextureFormat::Depth32Float),
                    ..Default::default()
                },
            ));

            pollster::block_on(painter.set_window(
                egui::viewport::ViewportId::ROOT,
                Some(window.clone()),
            )).expect("Failed to set window on painter");

            // Create scene renderer using the painter's wgpu device
            let scene_renderer = if let Some(rs) = painter.render_state() {
                Some(SceneRenderer::new(&rs.device, &rs.queue, rs.target_format))
            } else {
                None
            };

            self.window = Some(window);
            self.egui_state = Some(egui_state);
            self.painter = Some(painter);
            self.scene_renderer = scene_renderer;
            self.init_scene();
        }
    }

    fn window_event(&mut self, event_loop: &ActiveEventLoop, _window_id: winit::window::WindowId, event: WindowEvent) {
        if let (Some(egui_state), Some(window)) = (&mut self.egui_state, &self.window) {
            let _ = egui_state.on_window_event(window, &event);
        }

        match event {
            WindowEvent::CloseRequested => {
                self.running = false;
                event_loop.exit();
            }
            WindowEvent::Resized(physical_size) => {
                if physical_size.width == 0 || physical_size.height == 0 { return; }
                if let Some(painter) = &mut self.painter {
                    if let (Some(w), Some(h)) = (
                        NonZeroU32::new(physical_size.width),
                        NonZeroU32::new(physical_size.height),
                    ) {
                        painter.on_window_resized(
                            egui::viewport::ViewportId::ROOT,
                            w,
                            h,
                        );
                    }
                }
                self.camera.set_aspect(physical_size.width as f64 / physical_size.height as f64);
            }
            WindowEvent::KeyboardInput { event: KeyEvent { physical_key, state, .. }, .. } => {
                if state == ElementState::Pressed {
                    if let winit::keyboard::PhysicalKey::Code(key_code) = physical_key {
                        if let Some(action) = key_to_action(key_code) {
                            self.handle_action(action);
                        }
                    }
                }
            }
            WindowEvent::MouseWheel { delta, .. } => {
                let scroll = match delta {
                    MouseScrollDelta::LineDelta(_, y) => y as f64,
                    MouseScrollDelta::PixelDelta(pos) => pos.y as f64 / 100.0,
                };
                self.camera.mouse_scroll(scroll);
            }
            WindowEvent::MouseInput { button: MouseButton::Left, state, .. } => {
                self.dragging = state == ElementState::Pressed;
                if !self.dragging {
                    self.last_mouse_pos = None;
                }
            }
            WindowEvent::CursorMoved { position, .. } => {
                if self.dragging {
                    if let Some((lx, ly)) = self.last_mouse_pos {
                        let dx = position.x - lx;
                        let dy = position.y - ly;
                        // mouse_drag applies a 0.005 sensitivity factor internally.
                        self.camera.mouse_drag(dx, dy);
                    }
                    self.last_mouse_pos = Some((position.x, position.y));
                }
            }
            WindowEvent::RedrawRequested => {
                if !self.paused {
                    self.sim_time += self.dt * self.time_warp;
                }
                // Advance ephemeris and sync positions
                if let Some(psys) = &mut self.planetary {
                    if self.has_ephemeris {
                        psys.mjd = ephem_bridge::sim_time_to_mjd(self.sim_time);
                        psys.update_positions();
                    }
                    ephem_bridge::sync_positions(psys, &mut self.scene);
                }

                // Propagate the user vessel + fill FlightState from real orbital data.
                // Must run *before* scene.update_all so the vessel scene node's
                // new position feeds into the f64→f32 render conversion this frame.
                let dt_sim = self.dt * self.time_warp;
                if let (Some(vessel), Some(psys)) = (&mut self.vessel, &self.planetary) {
                    if vessel.parent_idx < psys.bodies.len() {
                        let parent_body = &psys.bodies[vessel.parent_idx];
                        // Split large steps into sub-steps for stability at high time warp.
                        let max_sub = 5.0;
                        let n = (dt_sim / max_sub).ceil().max(1.0) as usize;
                        let sub_dt = dt_sim / n as f64;
                        let gm = parent_body.gm();
                        for _ in 0..n {
                            vessel.propagate(gm, sub_dt);
                        }
                        let parent = ParentBody {
                            name: parent_body.name.clone(),
                            abs_pos: parent_body.pos,
                            gm,
                            radius: parent_body.radius_m,
                        };
                        let mjd = ephem_bridge::sim_time_to_mjd(self.sim_time);
                        self.flight_state = compute_flight_state(
                            vessel, &parent, self.sim_time, mjd, self.time_warp,
                        );
                    }
                }
                // Sync vessel → scene node (position + throttle for exhaust plume).
                if let (Some(vessel), Some(psys), Some(node_idx)) =
                    (&self.vessel, &self.planetary, self.vessel_node_idx)
                {
                    ephem_bridge::sync_vessel_position(&mut self.scene, vessel, psys, node_idx);
                    if let Some(node) = self.scene.nodes_mut().get_mut(node_idx) {
                        if let orbitx_render::NodeType::Vessel(vs) = &mut node.node_type {
                            vs.throttle = vessel.throttle as f32;
                        }
                    }
                }

                let body_positions: Vec<Vec3> = self.scene.nodes()
                    .iter().map(|n| n.transform.position).collect();
                let body_radii: Vec<f64> = self.planetary.as_ref()
                    .map(|psys| psys.bodies.iter().map(|b| b.radius_m).collect())
                    .unwrap_or_default();
                // Body radii is one-per-planetary-body; scene may include the
                // vessel node at the tail, so only pass radii when lengths match.
                if body_radii.len() == body_positions.len() {
                    self.camera.update_with_radii(&body_positions, &body_radii, self.dt);
                } else {
                    self.camera.update(&body_positions, self.dt);
                }
                self.coord_bridge.set_origin(self.camera.cam_pos_sim());
                self.camera.set_render_scale(self.coord_bridge.scale());
                self.scene.update_all(&self.coord_bridge, &self.camera.cam_pos_sim());

                self.render();
                if let Some(window) = &self.window {
                    window.request_redraw();
                }
            }
            _ => {}
        }
    }
}

/// 驾驶舱面板叠加：内视图时在屏幕四周绘制半透明黑色边框，模拟座舱视窗遮挡。
///
/// 这是 GenericCockpit 的最小实现——不加载任何 mesh，只做视口裁切提示，
/// 让用户明确知道"当前是内部视图"。
fn draw_cockpit_bezel(ui: &egui::Ui) {
    let rect = ui.available_rect_before_wrap();
    let painter = ui.painter_at(rect);
    let bezel = egui::Color32::from_black_alpha(220);
    let panel_h = rect.height() * 0.16;
    let panel_w = rect.width() * 0.12;
    // 上下边框（保留中央视野）
    painter.rect_filled(
        egui::Rect::from_min_size(
            egui::pos2(rect.left(), rect.top()),
            egui::vec2(rect.width(), panel_h),
        ),
        0.0, bezel,
    );
    painter.rect_filled(
        egui::Rect::from_min_size(
            egui::pos2(rect.left(), rect.bottom() - panel_h),
            egui::vec2(rect.width(), panel_h),
        ),
        0.0, bezel,
    );
    // 左右边框
    painter.rect_filled(
        egui::Rect::from_min_size(
            egui::pos2(rect.left(), rect.top() + panel_h),
            egui::vec2(panel_w, rect.height() - 2.0 * panel_h),
        ),
        0.0, bezel,
    );
    painter.rect_filled(
        egui::Rect::from_min_size(
            egui::pos2(rect.right() - panel_w, rect.top() + panel_h),
            egui::vec2(panel_w, rect.height() - 2.0 * panel_h),
        ),
        0.0, bezel,
    );
    // 视窗轮廓（细绿边）
    let inner = egui::Rect::from_min_max(
        egui::pos2(rect.left() + panel_w, rect.top() + panel_h),
        egui::pos2(rect.right() - panel_w, rect.bottom() - panel_h),
    );
    painter.rect_stroke(inner, 4.0,
        egui::Stroke::new(1.5, egui::Color32::from_rgb(0, 200, 60)),
        egui::StrokeKind::Outside);
    // COCKPIT 标签
    painter.text(
        egui::pos2(rect.left() + panel_w + 8.0, rect.top() + panel_h + 8.0),
        egui::Align2::LEFT_TOP,
        "COCKPIT · GENERIC",
        egui::FontId::monospace(10.0),
        egui::Color32::from_rgb(0, 200, 60),
    );
}
