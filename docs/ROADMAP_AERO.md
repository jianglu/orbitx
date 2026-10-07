# 火箭气动排期（ROADMAP_AERO）

产品火箭筒体 + 翼面气动的分阶段排期与验收。算法权威见 [`AERO.md`](AERO.md)。阶段号用 **A\***，避免与主 [`ROADMAP.md`](ROADMAP.md) 的 P0–P6 撞号。

## 与 P1.1 的关系

**P1.1 已完成**：Orbiter 装配气动（`compute_aero_forces` / `Airfoil` / `DragElement`）。主 ROADMAP 中 P1.1 保持 ✅。

本文件是**产品侧补全**：现网火箭只用钉在原点的 `DragElement`（轴向 Cd），缺筒体法向、斜装/栅格翼、簇包络、对随油质心力矩。新公式在 `aero/rocket.rs`；不扩 `compute_aero_forces`。同一簇步进互斥：火箭簇只走 `compute_rocket_aero`，飞机簇只走 P1.1。

## 当前状态 / 目标

| | |
|--|--|
| 现状（本轮完成后） | `compute_rocket_aero` + A12 翼当地空速；A11 FFI 对拍；体轴筒体 + 簇包络 + `lifting_surfaces` |
| 目标 | 同上（教学级）；后续可调默认系数 / Controller 展翼 |
| 不做（仍成立） | 抛罩、Controller 展翼、涡追踪、旧 sim 兼容、DG 表 |

## 阶段表

入口条件：上一阶段对应 crate 测试绿。

| 阶段 | 内容 | 测什么 |
|------|------|--------|
| **A0** | `AERO.md` / 本文件；主 ROADMAP 引用；`aero.rs`→`aero/mod.rs`（零行为） | dynamics 既有气动测无回归 |
| **A1** | `rocket.rs` 仅筒体（三轴权重、体轴力、包络中部对 cg） | α=0 仅轴向；α↑ 法向↑；力矩臂 |
| **A2** | 翼升阻同套（失速、αs(M)、η、波阻） | 小 α 升力；过失速；翼不并进筒体面积 |
| **A3** | 背风 ×1/2、`slew_deploy` | 未伸出 ×1/2；限速全程时间 |
| **A4** | `StageConfig` / `StageSpec` 字段；CONFIG_TOML 链 AERO | 可读 `lifting_surfaces` |
| **A5** | `assembly/aero_geom.rs` 簇包络 | 三轴面积、几何中部、奇数侧挂占比 |
| **A6** | 步进互斥接线 | 火箭簇只 rocket；飞机簇只 P1.1；簇 cg 力矩 |
| **A7** | orbitx 预置改新模型 | vessel/config 测 |
| **A8** | sim-rocket 投影：`merge` 跳过 `fin_*`；导出翼面 | 导出 toml |
| **A9** | 设计端缩放、AeroComponent、capabilities；栅格无旋转通道 | 设计侧一致性 |
| **A10** | AGENTS / README / pending 交叉链接；本表打勾 | 相关测回归 |
| **A11** | `dynamics-ffi`：波阻/诱导抄 Orbiter；筒体+翼面 C++ 第二实现；`ffi_oracle` 对拍 | Rust vs C++ proptest |
| **A12** | 翼当地空速 `airvel+ω×(r−cg)`；q/Ma 仍质心；不搬 P1.1 `rdrag` | 生产弦向小 α；十字翼 ω=0 净 τy≈0；+ωy 时 τy 反向 |

## 进度

- [x] A0 文档 + 目录搬家
- [x] A1–A3（`compute_rocket_aero` 筒体/翼/背风·deploy）
- [x] A4 配置字段 + CONFIG_TOML
- [x] A5 簇包络 `aero_geom`
- [x] A6 步进互斥接线
- [x] A7 orbitx 预置
- [x] A8 sim-rocket 投影导出
- [x] A9 设计端 / capabilities；栅格 deploy 指令记 pending
- [x] A10 交叉链接与回归
- [x] A11 FFI 算法一致性（`rocket_aero.cpp` + `ffi_oracle`）
- [x] A12 翼当地空速（滚转阻尼）
