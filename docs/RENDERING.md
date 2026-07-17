# orbitx 渲染架构

主 app（`orbitx-app`）的 wgpu + egui 3D 渲染栈架构记录。适合想理解或扩展渲染代码的人。

## 技术栈

| 层 | 选型 | 版本 |
|----|------|------|
| GPU 抽象 | wgpu | 29（Vulkan / Metal / D3D12 / WebGPU）|
| 窗口 / 输入 | winit | 0.30 |
| HUD / MFD / UI | egui | 0.35（egui-wgpu + egui-winit）|
| GPU 数学 | glam | 0.29（f32，SIMD）|
| 大世界数学 | orbitx-math | Vec3/Mat3 f64 |

## Crate 结构

```
orbitx-render     f64→f32 CoordinateBridge · CameraSystem · SceneNode · SceneManager
orbitx-gfx-hud    egui 绘制的 HUD / MFD 面板
orbitx-app        winit + wgpu 主循环 · 3D pipeline · 场景桥接
  ├─ shader/*.wgsl        planet / billboard / atmosphere / ring / cloud / sky / sun / corona / line
  ├─ sphere.rs             程序化 UV 球体 + 环生成
  ├─ scene_renderer.rs    wgpu 管线 + 每帧 FrameScene + egui_wgpu::CallbackTrait
  ├─ ephem_bridge.rs      PlanetarySystem ↔ SceneManager 位置同步
  ├─ vessel.rs             UserVessel + RK4 Kepler 传播
  ├─ flight_calc.rs        状态矢量 → FlightState（HUD/MFD 输入）
  ├─ app.rs               主 ApplicationHandler + 帧循环
  └─ input.rs             KeyCode ↔ Action 映射 + TOML 加载
```

## 大世界 → 渲染坐标：CoordinateBridge

太阳系尺度跨越 15 个数量级（Sun 半径 6.96×10⁸ m vs 冥王星 5.9×10¹² m 远日点），
f32 无法直接表示。`CoordinateBridge` 每帧把**相机位置设为浮点原点**：

```
render_pos = (sim_pos - camera_pos_sim) * scale
```

其中 `scale` 在 real-scale 模式为 1.0，太阳系缩放模式为 `1 AU = N render units`。
这样：
1. GPU 端 f32 精度总是围绕相机附近，不会失真
2. 视图矩阵简化为 `look_to_rh(ZERO, forward, up)`
3. 手性翻转 `render.z = -sim.z` 让黄道北（sim.y）映射为屏幕上方

## 相机：CameraSystem

- 6 外部模式（TargetRelative / AbsDirection / GlobalFrame / TargetToObject /
  TargetFromObject / GroundObserver）+ 3 内部模式（GenericCockpit / Panel2D / VirtualCockpit）
- 对数深度缓冲：`z_ndc = log2(C·w+1) / log2(C·far+1)`，近/远比 ≤ 1e15
- 动态近平面：`near = clamp(min(cam→表面距离) × 1e-3, [0.1 m, 1e6 m])`

## 渲染回调：egui_wgpu::CallbackTrait

**关键架构发现**：`egui_wgpu::CallbackTrait` 允许在 egui 的 RenderPass 内插入
自定义 wgpu 绘制。3D 场景与 egui UI **共享同一个 CommandEncoder / RenderPass**，
无需独立管理 GPU 生命周期。

```
egui_ctx.run_ui(|ui| {
    ui.painter().add(egui_wgpu::Callback::new_paint_callback(
        rect, SceneCallback,
    ));
    // 在这后面画 HUD / MFD egui 元素 → 天然叠在 3D 之上
});
```

`SceneCallback::paint()` 会从 `callback_resources` 拿到 `SceneRenderer` +
`FrameScene`（本帧的所有 draw call 数据），执行绘制。

## 帧循环（app.rs::RedrawRequested）

```
1. sim_time += dt × time_warp（若未暂停）
2. planetary.update_positions()（历表推进）
3. sync_positions(planetary → scene)
4. vessel.propagate(dt_sim)（含子步长，防高时间加速失稳）
5. flight_state = compute_flight_state(vessel, parent)  ← HUD/MFD 输入
6. sync_vessel_position(scene, vessel, planetary, node_idx)
7. camera.update_with_radii(positions, radii, dt) ← 动态近平面
8. coord_bridge.set_origin(camera.cam_pos_sim())
9. scene.update_all(coord_bridge, cam_pos)（f64 → f32 + 距离/屏幕角）
10. FrameScene::from_scene(camera, scene, viewport)  ← draw 列表 + 视锥剔除
11. egui 渲染回调 → wgpu 命令编码器 → 提交
```

## FrameScene::from_scene（本帧 draw 列表构建）

对每个可见场景节点：

1. **视锥剔除**（P3F-2）：`dot(pos_render, forward_render) + scale < 0` → 完全在相机后 → skip
2. **屏幕角计算**：`screen_px = (scale / render_dist) × viewport_h / fov_y`
3. **LOD 决策**：`screen_px < min_visible_px` → Billboard；否则 Sphere
4. **发射尾焰**（P3C-2）：如 Vessel 节点 `throttle > 0.01`，追加一枚亮橙色 billboard

## 每天体独立 Uniform Buffer 池（关键 bug 教训）

早期版本共享一个 uniform buffer + `queue.write_buffer()` 14 次 → 命令队列提交前
最后一次写入覆盖前面 13 次 → 14 天体全画在同一位置（"一个点"bug）。

修复：**每 draw 一个独立 buffer + 独立 BindGroup**，按需扩池：

```rust
sphere_slots: Vec<(Buffer, BindGroup)>,  // 每帧按 sphere draw 数量扩容
```

同样模式用于 billboard / atmosphere / ring / cloud slots。

## 双 BindGroup 模式

Group 0 = per-draw uniform（MVP、model、光照方向、log-depth 常数）
Group 1 = 每天体纹理 + 采样器（缓存在 `HashMap<String, BindGroup>`）

未贴图天体绑定 1×1 白色 fallback，shader 用 `use_texture` 布尔（藏在 `log_depth.w`）决定是否采样。

## 光照 / 发射标志

`planet.wgsl` 支持两种模式：
- 常规 Lambert：`ambient + diffuse × dot(N, L)`
- Emissive（Sun / Vessel）：直接返回纹理色，全亮无光照

Emissive flag 藏在 `light_dir.w`（float 打包成 bool 节省 uniform）。

## 大气 / 云层 / 环 层次

`atmosphere.wgsl` — Fresnel rim + 昼侧渐隐，预乘 alpha，×1.03 球壳
`cloud.wgsl` — 等距柱状云图亮度 → opacity，昼侧受光，×1.01 球壳，缓慢漂移
`ring.wgsl` — 径向 UV 纹理，双面、直式 alpha、不写深度

绘制顺序：Sky → 天体 sphere → Cloud → Atmosphere → Ring → Vessel → Exhaust →
Line（黄道网格 / 轨道环）→ HUD/MFD（egui overlay）

## 太阳（P3B-4b）

- **光球**：FBM 值噪声 granulation + 边缘暗化 + 黑体色 + 太阳黑子（`sun.wgsl`）
- **日冕**：相机朝向 billboard + 平滑径向衰减（`corona.wgsl`）
  避免了 Fresnel 双壳导致的"蛋壳分层"感

## 视锥剔除（P3F-2）

极简：只做**行为最重要**的 behind-camera cull：

```rust
let z_view = pos_render.dot(forward_render);
if z_view + scale < 0.0 { continue; }
```

保留 `scale` 边缘作为安全余量。侧向 / 顶部 / 底部裁切**未做**——GPU 硬件裁剪
已经处理得很好，且对 ~15 个天体的场景 CPU 裁剪收益微乎其微。

## 关键 shader

| 文件 | 用途 |
|------|------|
| `planet.wgsl` | 通用球体（Lambert 或 emissive）+ 对数深度 + 可选纹理 |
| `billboard.wgsl` | 相机朝向 disc / glow（远距 fallback + 尾焰 + 恒星）|
| `atmosphere.wgsl` | Fresnel 边缘辉光壳 |
| `cloud.wgsl` | 云层壳（equirect luminance → opacity）|
| `ring.wgsl` | 环带 annulus（径向 UV，双面）|
| `sky.wgsl` | 大天空盒（8K 银河）|
| `sun.wgsl` | 光球（FBM + 黑子 + 边缘暗化）|
| `corona.wgsl` | 日冕（billboard 径向衰减）|
| `line.wgsl` | LineList（黄道网格 / 轨道环 / 垂线）|

## 常见"看不见东西"的排查清单

1. **CentralPanel 不透明** → HUD 遮挡 3D。用 `Frame::new().fill(TRANSPARENT)`
2. **共享 uniform buffer** → 14 天体全画在同一位置。改成 per-draw slots
3. **屏幕像素单位不匹配** → 用 render 单位（不是 sim 米）算 screen_px
4. **相机在天体内部** → 检查默认 dist，需 > sun_radius
5. **球体绕序反了** → 里朝外全见双极。检查三角形 CCW-from-outside 绕序
6. **黄道面竖直** → 检查坐标映射，应 `render.y = sim.y`（黄道北为屏幕上方）
7. **CAD 拖拽方向反了** → `phi -= dx × SENS; theta += dy × SENS`

历史上这 7 条 bug 全部亲身踩过，本 crate 现在都做对了 :)

## 参考

- `crates/orbitx-app/examples/` — 8 个逐子系统隔离验证 demo
- `crates/orbitx-render/src/{coord,camera,scene}.rs` — 26 tests 覆盖核心变换
- `crates/orbitx-app/src/{vessel,flight_calc}.rs` — 9 tests 覆盖飞行数据管线
