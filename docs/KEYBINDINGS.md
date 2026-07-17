# orbitx 键位速查

主 app（`cargo run -p orbitx-app`）的完整键位参考。

内置默认加载顺序（先命中先用）：
1. `$ORBITX_KEYBINDINGS`（环境变量，绝对路径）
2. `$HOME/.config/orbitx/keybindings.toml`
3. 编译时嵌入的 [`assets/keybindings.toml`](../assets/keybindings.toml)

TOML 语法极简：每行 `KeyCode = "ActionName"`；带参数用 `"CamModeSet(3)"` 形式。
`#` 行首为整行注释，值后 `#` 到行尾为行尾注释。未识别键名/动作名会被静默丢弃并保留默认。

## 相机

| 键 | 作用 |
|----|------|
| **鼠标左键 + 拖拽** | CAD 风格轨道旋转（水平 = 方位角 φ，竖直 = 俯仰 θ）|
| **鼠标滚轮** | 缩放（TargetRelative 模式的 dist）|
| **W / S** | 相机上/下轨道（未启用轨道键盘控制时保留）|
| **A / D** | 相机左/右轨道 |
| **Q / E** | 拉近 / 拉远 |
| **Tab** | 循环 6 种外部相机模式 |
| **1** | TargetRelative（默认，绕目标球面轨道）|
| **2** | AbsDirection（相机固定全局方向）|
| **3** | GlobalFrame（相机自由定位）|
| **4** | TargetToObject（目标 → 参考天体方向）|
| **5** | TargetFromObject（参考天体 → 目标方向）|
| **6** | GroundObserver（地面观察者）|
| **G** | 快切 GroundObserver 模式 |
| **R** | 循环 TargetToObject / TargetFromObject 的参考天体 |
| **V** | 切换内部（驾驶舱）/ 外部视图 |

## 焦点

| 键 | 作用 |
|----|------|
| **[** | 上一个焦点天体（含飞船节点）|
| **]** | 下一个焦点天体（含飞船节点）|

## 飞行控制

| 键 | 作用 |
|----|------|
| **↑** | 油门 +5% |
| **↓** | 油门 −5% |
| **0** | 油门全推（100%）|
| **`**（反引号）| 油门切断（0%）|

油门 > 1% 时飞船尾部出现橙色发射尾焰 billboard，大小与颜色随油门加深。

## 时间

| 键 | 作用 |
|----|------|
| **Space** | 暂停 / 恢复 |
| **.** | 时间加速 ×2（上限 1e6×）|
| **,** | 时间加速 /2（下限 0.125×）|

## HUD / MFD

| 键 | 作用 |
|----|------|
| **H** | HUD 模式循环（Orbit / Surface / Docking）|
| **C** | HUD 颜色循环（Green / Red / Yellow / Blue）|
| **O** | 左 MFD 类型循环（10 种）|
| **M** | 右 MFD 类型循环 |

## 通用

| 键 | 作用 |
|----|------|
| **Esc** | 退出 |

## 自定义示例

保存到 `$HOME/.config/orbitx/keybindings.toml`：

```toml
# 把 CAD 风格轨道旋转搬到 IJKL（Dvorak 键盘友好）
KeyJ = "CamOrbitLeft"
KeyL = "CamOrbitRight"
KeyI = "CamOrbitUp"
KeyK = "CamOrbitDown"

# 让 F1..F6 直接映射到相机模式
F1 = "CamModeSet(0)"
F2 = "CamModeSet(1)"
F3 = "CamModeSet(2)"
F4 = "CamModeSet(3)"
F5 = "CamModeSet(4)"
F6 = "CamModeSet(5)"

# 油门用 Z/X（模拟 KSP）
KeyZ = "ThrottleFull"
KeyX = "ThrottleCut"
```

其他未提及的键仍会加载内置默认（因为整个映射表以嵌入 TOML 为基础，然后叠加用户覆盖前先解析）——
当前实现是**完整替换**（TOML 有一条就替换，缺失键不再有默认）。如果你只想覆盖部分键，
把默认表复制到自定义文件再改需要的行。

## 完整动作枚举

见 `crates/orbitx-app/src/input.rs` 的 `Action`。可用动作名：

```
CamOrbitLeft / CamOrbitRight / CamOrbitUp / CamOrbitDown
CamZoomIn / CamZoomOut
CamModeNext / CamModePrev / CamGroundObserver
CamModeSet(0..5)
CamToggleInternal / CamCycleDirref
ThrottleUp / ThrottleDown / ThrottleFull / ThrottleCut
Prograde / Retrograde / RadialIn / RadialOut
RcsPitchUp / RcsPitchDown / RcsYawLeft / RcsYawRight
MfdLeftNext / MfdRightNext
TimeWarpUp / TimeWarpDown / TimePause
FocusNextBody / FocusPrevBody
HudModeNext / HudColorNext
Quit
```

## 可用 KeyCode

见 [winit KeyCode](https://docs.rs/winit/latest/winit/keyboard/enum.KeyCode.html)。
`parse_key()` 覆盖常用子集：字母 `KeyA..KeyZ`、数字 `Digit0..Digit9`、
`Space / Tab / Enter / Escape / Backspace / Backquote / Period / Comma`、
`BracketLeft/Right`、`Semicolon / Quote / Backslash / Slash / Minus / Equal`、
`ArrowUp/Down/Left/Right`、`PageUp/Down / Home / End / Insert / Delete`、
`F1..F12`、`ShiftLeft/Right / ControlLeft/Right / AltLeft/Right`。

需要更多变体时，在 `input.rs::parse_key()` 中追加一行即可。
