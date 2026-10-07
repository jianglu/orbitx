# 火箭气动模型

产品火箭（sim-rocket 零件 → `sim.toml` → orbitx）的**气动算法权威**。字段表见 [`CONFIG_TOML.md`](CONFIG_TOML.md)；分阶段排期见 [`ROADMAP_AERO.md`](ROADMAP_AERO.md)。

P1.1 的 Orbiter 装配气动（`compute_aero_forces`：气流轴翼面 / 操纵面 / `DragElement`）保持独立；本文件描述的是**火箭筒体 + 翼面**路径（`compute_rocket_aero`）。同一刚体簇步进只走一套，禁止双计。

## 产品边界

- Orbiter 核心只做气动装配，系数由船 DLL 填写；自带运载件几乎只有 `SetCW`。
- SimRocket 没有每支箭一个 DLL：几何与能力来自零件投影；**力公式权威在 orbitx**。
- 几何输入来自 `sim.toml` / 运行时刚体包络，引擎不发明外形。

## 坐标

- Godot / 级体：**+Y 纵轴**（箭头大致 +Y），+X 展向，+Z 厚度。
- 体轴空速 `airvel` 是**船相对大气**（`tmul(R, vel−wind)`）。前飞 **`vy>0`**。
- 筒体用力：**体轴**——轴向力沿箭身（阻碍前进：`−sign(vy)`），法向力垂直箭身（逆侧向空速）。不用气流轴升阻拆筒体，避免与轴向力重复计侧面阻。

## 文件按级；力学按刚体

`sim.toml` 描述每一级（筒体系数 + `lifting_surfaces`）。气动对象是**一个完整刚体**：主组合体（对接连通簇）或分离后的独立体。

- **质心**：各船 `com_body` 变到刚体坐标后合成随油 **cg**（与推力同一参考）。
- **力矩**：`τ = cross(F, r − cg)`。
- **三轴投影面积、筒体压心、烘焙筒体系数**：仅拓扑变化（对接 / 分离）时算一次并按刚体缓存；与控制用的 `active` **无关**（`active` 只表示当前关注/控制哪一级）。
- **烘焙 `RocketBodyAero`**：取该刚体内带 `rocket_body` 且干重最大者的表（`cd_mach` / `cn_alpha` / damp）。
- 每步仍可变、不进静态缓存：`deploy`（步初 slew 后写入冻结翼副本）、随油 cg、大气状态、背风标志（步初按空速判定后冻结）。

## 三轴投影面积

整刚体外轮廓（筒体 / 罩 / 助推，**不含翼**）。

- **迎风 `Sy`**：各船横向圆盘在簇体 XZ 上的**面积并集**（重叠不双计）。串联同径共轴 ≈ 一盘；侧挂助推 `Sy` 增大。
- **侧视 `Sx` / `Sz`**：AABB（≈ L·D）；奇数侧挂随滚转改两档占比。光筒绕纵轴转：法向力大小不变、方向跟侧向来流。
- **法向力不乘侧视面积**；侧视只用于俯仰/偏航率阻尼。

来流体轴分量加权面积（`weighted_area`）**不进**受力路径，仅保留供 FFI / 工具。

## 筒体

- 小迎角轴向力：`cd_mach`（或默认火箭表）× 动压 × **`Sy`**。
- 法向力：`CN = cn_alpha · α`（细长体教学斜率，默认 `cn_alpha=2`，**定义在迎风圆盘 `Sy` 上**）。`Fn = CN · q · Sy`，方向**逆侧向空速**。禁止用侧视面积乘 `CN_α`。
- 作用点：当前刚体包络**几何中部**（外形固定，不随油）。
- 力矩对刚体随油质心。
- 角速度阻尼（无翼也有，不走压心的 `ω×r`）：`τ_x -= q·Sx·pitch_damp·ω_x`，`τ_z -= q·Sz·yaw_damp·ω_z`，`τ_y -= q·Sy·roll_damp·ω_y`。  
  俯仰 `ω_x` 运动在 YZ → 乘沿 +X 看的投影 **`Sx`**；偏航 `ω_z` 运动在 XY → 乘 **`Sz`**；滚转乘迎风 **`Sy`**。  
  默认 `pitch_damp = yaw_damp = 1`，`roll_damp = 0.1`（与旧 `rdrag` 相同）。`ρ` 或空速过低时与筒体气动一起为零。不用步长上限，也不用 `(空速+30)`。

## 翼面

每条 `lifting_surfaces` 独立：

- 局部迎角上的升阻**同一套**（失速、马赫、栅格 η、deploy）；不把翼阻并进筒体投影。
- 固定翼：`αs(M) = αs0 · f(M)`（亚音速默认约 18°，跨音速提前）。
- 栅格：较高失速角 + `η(M)` + 波阻；`deploy` 按 `deploy_rate` 整步限速走到 `deploy_target`（语义同 TVC `max_gimbal_rate`），RK 子步冻结。
- **背风（运行时）**：步初用体轴空速判定一次并冻结到子步。侧向空速 `v_lat=(vx,0,vz)` 过小（相对 `|V|` 或绝对阈值）→ 不减半。下风侧（压心 XZ 与 `v_lat̂` 同半平面）且径向未超出该站位刚体截面外半径 `R_body(y)` → 有效动压 **× 1/2**；伸出满算。`R_body(y)` 为覆盖该 y 的成员柱半径最大值（拓扑缓存）。不用翼法向当可见度。
- 压心：该面 `ref`（约翼面中部 / ¼ 弦投影到级 / 刚体坐标）。
- 弦向：**前缘→后缘**（零件 −Y）。零迎角时 `airvel ≈ −chord`（前飞 +Y）。`α = atan2(v_n, −v_c)`。
- 当地空速：`airvel_pt = airvel_cg + ω × (ref − cg)`（体轴）。迎角与阻力方向用 `airvel_pt`；**动压 / 马赫仍用质心空速**。筒体角速度阻尼见上一节，不通过压心的 `ω×r`。
- 升力取翼法向里垂直于当地空速的部分（不做功），阻力沿 `−airvel_pt̂`。

## 步进顺序

1. 步初：整步 `dt` 限速 `deploy`。
2. 取刚体静态缓存（包络 + 烘焙 body）；收集翼副本；按步初空速写背风标志并冻结。
3. 子步：只用冻结输入调用 `compute_rocket_aero`（力闭包内不再算并集 / 背风）。

## 明确不做

抛罩、本轮 Controller 下令展栅格、涡追踪、旧 sim.toml 兼容、DG 飞机 CL 表、侧飞关气动、像素轮廓求并、力路径加权面积。

## 参照（规则不抄数）

- Orbiter：装配与三轴投影面积；`oapiGetWaveDrag` 形状可借鉴。
- Jorgensen / DATCOM 部件叠加；Hoerner 阻力随迎角；Nielsen 导弹气动。

## FFI 对照

- 波阻 / 诱导阻力：`orbitx-dynamics-ffi/cpp/rocket_aero.cpp` 抄自 Orbiter `oapiGetWaveDrag` / `oapiGetInducedDrag`。
- 筒体 + 翼面合成：同文件为 `rocket.rs` 的 C++ 第二实现（非 Vessel.cpp）；`cargo test -p orbitx-dynamics --test ffi_oracle`。
