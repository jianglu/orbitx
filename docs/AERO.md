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
- **力矩**：`τ = (r − cg) × F`（推力、RCS、气动同一叉乘）。直接加的角速度阻尼不翻号。
- **三轴投影面积、筒体压心、烘焙筒体系数**：仅拓扑变化（对接 / 分离）时算一次并按刚体缓存；与控制用的 `active` **无关**（`active` 只表示当前关注/控制哪一级）。
- **烘焙 `RocketBodyAero`**：取该刚体内带 `rocket_body` 且干重最大者的表（`cd_mach` / `cn_alpha` / damp）。
- 每步仍可变、不进静态缓存：`deploy`（步初 slew 后写入冻结翼副本）、随油 cg、大气状态、背风标志（步初按空速判定后冻结）。

## 三轴投影面积

整刚体外轮廓（筒体 / 罩 / 助推，**不含翼**）。

- **迎风 `Sy`**：各船横向圆盘在簇体 XZ 上的**面积并集**（重叠不双计）。串联同径共轴 ≈ 一盘；侧挂助推 `Sy` 增大。翼不进圆盘。
- **侧视 `Sx` / `Sz`**：AABB（≈ L·D）。俯仰率阻尼用 `Sx`，偏航率阻尼用 `Sz`。横流面积 `S_lat = (|vx|·Sx + |vz|·Sz) / |v_lat|`。

## 筒体

全迎角同一式，三项相加（Allen、Perkins，NACA Report 1048，1951；Jorgensen，NTRS 19730006261，1973）。`α = atan2(|v_lat|, |vy|)`，`v_lat = (vx, 0, vz)`。

- 轴向：`Fy = −sign(vy) · Cd(M) · ½ρ · vy² · Sy`。
- 势流法向：`(cn_alpha/2) · q · Sy · sin(2α)`，逆侧向空速。默认 `cn_alpha = 2` 时小迎角为 `2·α·q·Sy`，90° 为 0。
- 横流：`1.1 · ½ρ · |v_lat|² · S_lat`，方向与势流相同。
- 作用点：当前刚体包络**几何中部**（外形固定，不随油）。
- 力矩对刚体随油质心，`(作用点 − cg) × F`。
- 角速度阻尼（无翼也有，不走压心的 `ω×r`，不随力矩叉乘翻号）：`τ_x -= q·Sx·pitch_damp·ω_x`，`τ_z -= q·Sz·yaw_damp·ω_z`，`τ_y -= q·Sy·roll_damp·ω_y`。  
  俯仰 `ω_x` 运动在 YZ → 乘沿 +X 看的投影 **`Sx`**；偏航 `ω_z` 运动在 XY → 乘 **`Sz`**；滚转乘迎风 **`Sy`**。  
  默认 `pitch_damp = yaw_damp = 1`，`roll_damp = 0.1`（与旧 `rdrag` 相同）。`ρ` 或空速过低时与筒体气动一起为零。不用步长上限，也不用 `(空速+30)`。

## 翼面

每条 `lifting_surfaces` 独立（不套筒体三项，面积不进 `Sy` / `S_lat`）。薄翼与筒体分开（Jorgensen，NASA TR R-474，1977：Nielsen、Kaattari、Pitts）。背风减半的 `q_eff` 乘在各项上：

- 附着升力：`Cl(α)·q_eff·area`，失速前 `Cl = Clα·α`。方向为低压侧 `n_lee = −sign(空速·法向)·法向` 里垂直于当地空速的分量。侧滑空速与法向都是 +Z 时，力指向 −Z。
- 分离法向力：`1.2·sin²α·q_eff·area`，同样指向低压侧。90° 时翼平面就是迎风面积。
- 零升阻力：`q_eff·edge_area·|cos α|`，沿来流反方向。`edge_area` = 厚度 × 展长。诱导阻力与波阻仍按翼平面系数。
- 固定翼：`αs(M) = αs0 · f(M)`（亚音速默认约 18°，跨音速提前）。
- 栅格：较高失速角 + `η(M)` + 波阻；`deploy` 按 `deploy_rate` 整步限速走到 `deploy_target`（语义同 TVC `max_gimbal_rate`），RK 子步冻结。
- **背风（运行时）**：步初用体轴空速判定一次并冻结到子步。`|V| < 5 m/s` 或 `β = atan2(|v_lat|, |vy|) < 10°` → 全部不遮蔽。两道都过，下风侧（压心 XZ 与 `v_lat̂` 同半平面）且径向未超出该站位筒体外半径 `R_body(y)` → 有效动压 **× 1/2**；伸出满算。不用翼法向当可见度。
- 压心：该面 `ref`（约翼面中部 / ¼ 弦投影到级 / 刚体坐标）。
- 弦向：**前缘→后缘**（零件 −Y）。零迎角时 `airvel ≈ −chord`（前飞 +Y）。`α = atan2(v_n, −v_c)`。
- 当地空速：`airvel_pt = airvel_cg + ω × (ref − cg)`（体轴）。迎角与阻力方向用 `airvel_pt`；**动压 / 马赫仍用质心空速**。筒体角速度阻尼见上一节，不通过压心的 `ω×r`。
- 力矩：`(ref − cg) × F`。

## 步进顺序

1. 步初：整步 `dt` 限速 `deploy`。
2. 取刚体静态缓存（包络 + 烘焙 body）；收集翼副本；按步初空速写背风标志并冻结。
3. 子步：只用冻结输入调用 `compute_rocket_aero`（力闭包内不再算并集 / 背风）。

## 明确不做

抛罩、本轮 Controller 下令展栅格、涡追踪、旧 sim.toml 兼容、DG 飞机 CL 表、侧飞关气动、像素轮廓求并。

## 参照（规则不抄数）

- Allen、Perkins，NACA Report 1048（1951）；Jorgensen，NTRS 19730006261（1973）：筒体轴向、势流法向与横流。
- Jorgensen，NASA TR R-474（1977）：薄翼与筒体分开。
- Orbiter：装配与三轴投影面积；`oapiGetWaveDrag` 形状可借鉴。

## FFI 对照

- 波阻 / 诱导阻力：`orbitx-dynamics-ffi/cpp/rocket_aero.cpp` 抄自 Orbiter `oapiGetWaveDrag` / `oapiGetInducedDrag`。
- 筒体 + 翼面合成：同文件为 `rocket.rs` 的 C++ 第二实现（非 Vessel.cpp）；`cargo test -p orbitx-dynamics --test ffi_oracle`。
