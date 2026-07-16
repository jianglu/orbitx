//! Main application - window management, wgpu init, simulation loop, egui integration.

use std::num::NonZeroU32;
use std::sync::Arc;
use winit::{
    application::ApplicationHandler,
    event::{ElementState, KeyEvent, MouseScrollDelta, WindowEvent},
    event_loop::ActiveEventLoop,
    window::{Window, WindowAttributes},
};

use orbitx_math::vec3::Vec3;
use orbitx_render::{
    CameraSystem, CoordinateBridge, ExternalCamMode,
    SceneNode, NodeType, SceneManager,
};
use orbitx_dynamics::PlanetarySystem;
use orbitx_gfx_hud::{FlightState, HudState, MfdPanel, MfdType, MfdSize};
use crate::input::{Action, key_to_action};
use crate::scene_renderer::{FrameScene, SceneCallback, SceneRenderer};
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
    sim_time: f64,
    time_warp: f64,
    paused: bool,
    dt: f64,
    focus_body: usize,
    running: bool,
    last_mouse_pos: Option<(f64, f64)>,
}

impl App {
    pub fn new() -> Self {
        Self {
            window: None,
            egui_ctx: egui::Context::default(),
            painter: None,
            egui_state: None,
            scene_renderer: None,
            camera: CameraSystem::new(),
            coord_bridge: CoordinateBridge::new_solar_system(20.0),
            scene: SceneManager::new(),
            planetary: None,
            has_ephemeris: false,
            hud: HudState::new(),
            mfd_left: MfdPanel::new(MfdType::Orbit, MfdSize::Left),
            mfd_right: MfdPanel::new(MfdType::Map, MfdSize::Right),
            flight_state: FlightState::default(),
            sim_time: 0.0, time_warp: 1.0, paused: false, dt: 0.016,
            focus_body: 3, running: true, last_mouse_pos: None,
        }
    }

    fn init_scene(&mut self) {
        let orbiter_src = ephem_bridge::resolve_orbiter_src();
        let psys = ephem_bridge::create_planetary_system(&orbiter_src);
        self.has_ephemeris = psys.bodies.iter().any(|b| b.ephemeris.is_some());
        self.scene = ephem_bridge::create_scene_from_psys(&psys);
        self.planetary = Some(psys);
    }

    fn handle_action(&mut self, action: Action) {
        match action {
            Action::CamModeNext => {
                let new_mode = match &self.camera.ext_mode {
                    ExternalCamMode::TargetRelative { .. } => ExternalCamMode::GlobalFrame {
                        pos: Vec3::ZERO, rot: orbitx_math::mat3::Matrix3::IDENTITY,
                    },
                    _ => ExternalCamMode::default(),
                };
                self.camera.set_ext_mode(new_mode);
            }
            Action::HudModeNext => self.hud.next_mode(),
            Action::HudColorNext => self.hud.next_color(),
            Action::MfdLeftNext => self.mfd_left.next_type(),
            Action::MfdRightNext => self.mfd_right.next_type(),
            Action::TimeWarpUp => self.time_warp = (self.time_warp * 2.0).min(1e6),
            Action::TimeWarpDown => self.time_warp = (self.time_warp / 2.0).max(0.125),
            Action::TimePause => self.paused = !self.paused,
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
            sr.set_frame(FrameScene::from_scene(&self.camera, &self.scene, viewport_size));
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

            egui::CentralPanel::default().show(ui, |ui| {
                self.hud.draw(ui, &self.flight_state);
            });
            egui::Panel::right("info_panel").show(ui, |ui| {
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
                ui.separator();
                ui.label(format!("HUD: {} ({:?})", self.hud.mode.name(), self.hud.color));
                ui.label(format!("MFD-L: {}", self.mfd_left.mfd_type.name()));
                ui.label(format!("MFD-R: {}", self.mfd_right.mfd_type.name()));
                ui.separator();
                ui.label("Controls:");
                ui.label("WASD: Camera orbit");
                ui.label("Q/E: Zoom in/out");
                ui.label("Tab: Camera mode");
                ui.label("[/]: Focus body");
                ui.label("H: HUD mode  C: HUD color");
                ui.label("O/M: MFD type");
                ui.label(",/.: Time warp");
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
                Some(SceneRenderer::new(&rs.device, rs.target_format))
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
            WindowEvent::CursorMoved { position, .. } => {
                if let Some((lx, ly)) = self.last_mouse_pos {
                    let dx = position.x - lx;
                    let dy = position.y - ly;
                    self.camera.mouse_drag(dx * 0.005, dy * 0.005);
                }
                self.last_mouse_pos = Some((position.x, position.y));
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
                let body_positions: Vec<Vec3> = self.scene.nodes()
                    .iter().map(|n| n.transform.position).collect();
                self.camera.update(&body_positions, self.dt);
                self.coord_bridge.set_origin(self.camera.cam_pos_sim());
                self.scene.update_all(&self.coord_bridge, &self.camera.cam_pos_sim());
                self.flight_state.sim_time = self.sim_time;
                self.flight_state.time_warp = self.time_warp;
                self.render();
                if let Some(window) = &self.window {
                    window.request_redraw();
                }
            }
            _ => {}
        }
    }
}
