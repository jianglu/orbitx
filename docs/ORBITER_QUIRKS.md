# Orbiter 源码怪异行为与已知问题记录

在将 Orbiter 的 `Vecmat.h` / `Vecmat.cpp` / `Astro.h` / `Astro.cpp` 移植到 Rust
（`orbitx-math` crate）的过程中，发现了若干 C++ 源码中的怪异行为、可疑设计、
不符合标准的写法，以及潜在的 bug。

本文件记录这些发现，以便后续评估是否需要「修复」（偏离 C++ 行为以获得更正确的语义）
还是「忠实保留」（保持与 C++ 逐字节一致以确保数据兼容性）。

**当前策略**：`orbitx-math` Phase 1 全部忠实保留 C++ 行为，即使行为有问题。每个条目
标注了 `orbitx-math` 中的对应处理方式。

---

## 1. `date2mjd` — `tm_mon` 使用 1-based 而非标准 C 的 0-based

**文件**：`Src/Orbiter/Astro.cpp:12-24`

**问题**：C 标准规定 `struct tm::tm_mon` 是 0-based（0=Jan, 11=Dec）。但 Orbiter 的
`date2mjd` 函数与配套的 `mjddate` 函数之间形成一个自洽但**非标准**的约定：
`mjddate` 计算出的 `tm_mon` 是 1-based（1=Jan, 12=Dec），而 `date2mjd` 也期望
1-based 输入。

```cpp
// mjddate (Astro.cpp:45) 产生 1-based 月份:
date.tm_mon = f - 1 - 12 * (f/14);   // 对 1 月，f=2 → tm_mon=1 (不是 0)

// date2mjd (Astro.cpp:16) 直接使用 tm_mon，假设它是 1-based:
int m = date->tm_mon;                 // 1 月 → m=1，进入 m<=2 分支 → m=13
```

**影响**：如果外部代码用标准 C 的 `mktime`/`localtime` 填充 `struct tm` 后传入
`date2mjd`，结果会差 31 天。Orbiter 内部因为 `mjddate`/`date2mjd` 配对使用所以自洽，
但这是一个隐藏的兼容性陷阱。

**验证**：对 J2000 历元（2000-01-01 12:00）：
- 标准 MJD = 51544.5
- Orbiter `date2mjd(tm_mon=1)` = 51544.5 ✓（自洽）
- 若用标准 `tm_mon=0` 调用 = 51513.5 ✗（差 31 天）

**orbitx-math 处理**：`CivilDate.month` 是 1-based，直接传给算法，忠实复刻 C++ 行为。
已在 `date_to_mjd` 文档注释中明确标注此约定。

**是否修复**：⚠️ 暂不修复。修复会破坏与 Orbiter 场景文件（`.scn` 中 MJD 值）的兼容性。
未来若 orbitx 完全独立于 Orbiter 数据格式，可考虑修正为标准 0-based。

---

## 2. `Quaternion::conj` — 成员函数忽略 `this`

**文件**：`Src/Orbiter/Vecmat.h:390-392`

**问题**：`conj` 声明为 `const` 成员函数，但其实现**完全忽略 `this` 指针**，
返回的是**参数** `Q` 的共轭，而非 `*this` 的共轭：

```cpp
Quaternion conj (const Quaternion &Q) const // returns conjugate of Q
{ return Quaternion (-Q.qvx, -Q.qvy, -Q.qvz, Q.qs); }
//                            ^^^ 用的是参数 Q，不是 this->qvx
```

**影响**：调用 `q1.conj(q2)` 返回 `q2` 的共轭，而不是直觉上的 `q1` 的共轭。
这是一个容易误用的 API。

**orbitx-math 处理**：Rust 版 `Quat::conj(&self)` 返回 `self` 的共轭（更符合直觉）。
这是一个**有意偏离** C++ API 的地方，因为 C++ 的语义明显是笔误/设计错误。

**是否修复**：✅ 已修复（API 语义更合理）。如果未来需要与 C++ 插件 FFI 交互，
需注意此差异。

---

## 3. `Quaternion::premul` / `postmul` / `tpostmul` — 重归一化被 `#ifdef UNDEF` 禁用

**文件**：`Src/Orbiter/Vecmat.cpp:549-601`

**问题**：这三个函数都包含一个重归一化代码块，但被包裹在
`#ifdef UNDEF ... #endif` 中。由于 `UNDEF` 从未被定义，这些重归一化代码**永远不执行**：

```cpp
void Quaternion::premul (const Quaternion &Q)
{
    // ... 计算 ...
#ifdef UNDEF   // ← 永远为 false
    double inorm = ...;
    // 重归一化
#endif
}
```

**影响**：`premul`/`postmul`/`tpostmul` 操作后四元数**不会**被重归一化，可能累积
数值误差。相比之下，`operator+=`、`Rotate`、`interp` 都会重归一化。这可能是
开发者在调试时临时禁用了重归一化，之后忘记重新启用。

**orbitx-math 处理**：忠实保留——这三个函数不重归一化。

**是否修复**：⚠️ 暂不修复。修复可能改变物理仿真的数值轨迹。需先确认 Orbiter 的
调用者是否依赖了「不归一化」这个行为。

---

## 4. `Quaternion::Rotate` — desperation 分支重置为单位四元数

**文件**：`Src/Orbiter/Vecmat.cpp:499-522`

**问题**：`Rotate` 在重归一化时，如果 `norm2 <= 0`（四元数完全退化），会**静默重置
为单位四元数** `(0,0,0,1)`：

```cpp
double arg = qvx*qvx + qvy*qvy + qvz*qvz + qs*qs;
if (arg > 0.0) {
    double inorm = 1.0/sqrt(arg);
    // ... 归一化 ...
} else { // desperation
    qvx = qvy = qvz = 0.0;
    qs  = 1.0;
}
```

**影响**：这是一个「desperation」（绝望）处理——当四元数因数值爆炸退化到零时，
静默恢复为无旋转状态。问题在于：
1. **无任何警告/日志**——如果发生，用户完全不知道
2. **掩盖了上游问题**——四元数退化到零通常意味着积分器发散或角速度过大，
   重置为单位四元数只是隐藏症状
3. 物体姿态会突然「跳」回默认朝向

**orbitx-math 处理**：忠实保留（重置为单位四元数）。

**是否修复**：🔍 建议未来改进——至少添加日志警告，或返回一个标志让调用者知道发生了重置。

---

## 5. `Matrix::inv` — 无奇异性检查

**文件**：`Src/Orbiter/Vecmat.cpp:221-237`

**问题**：3×3 矩阵求逆直接除以行列式 `det`，**无任何零检查**：

```cpp
Matrix inv (const Matrix &A)
{
    double det = A.m11 * (A.m22*A.m33 - ...) - ...;
    return Matrix (
        (...) / det,   // ← det=0 时产生 inf/nan，无检查
        ...
    );
}
```

**影响**：对奇异矩阵（行列式为零）调用 `inv` 会产生 `inf`/`nan`，且无任何报错。
这些 `nan` 会静默传播到物理计算中，导致难以定位的 bug。

**orbitx-math 处理**：忠实保留（不检查，产生 inf/nan）。

**是否修复**：🔍 建议未来改进——可以返回 `Option<Matrix3>` 或使用 `Result`，
或在 debug 模式下 `assert!(det.abs() > eps)`。但需确认调用者是否依赖了「不 panic」
这个特性（Orbiter 中 `inv` 的调用点不多，主要在 QR 之外的少数地方）。

---

## 6. `Equ2Ecl` / `Ecl2Equ` — `asin` 无域检查

**文件**：`Src/Orbiter/Astro.cpp:91-105`

**问题**：赤道↔黄道坐标转换中的 `asin` 参数无 clamp 到 `[-1, 1]`：

```cpp
b = asin (sindc*cosob - sinra*cosdc*sinob);
//  ^^^^^ 若参数超出 [-1,1]，结果为 nan
```

**影响**：当输入角度接近边界值时（如 `dc` 接近 ±π/2），浮点误差可能使 `asin` 的
参数略微超出 `[-1, 1]`，产生 `nan`。这在 FFI 测试中实际观察到了（`prop_equ2ecl`
在 `dc` 接近 ±1.57 时产生 NaN）。

**orbitx-math 处理**：忠实保留（无 clamp）。FFI 测试中通过 `assert_close` 的
both-NaN 处理来容忍此行为。

**是否修复**：✅ 建议修复——添加 `.clamp(-1.0, 1.0)` 是无副作用的改进，
且能避免偶发 NaN。这是一个纯改进，不影响正常范围内的数值精度。

---

## 7. `mjddate` / `DateStr` / `FloatStr` / `DistStr` / `SciStr` — 静态返回缓冲区

**文件**：`Src/Orbiter/Astro.cpp:27, 54, 142, 169, 196`

**问题**：这五个函数都返回指向**函数内静态缓冲区**的指针：

```cpp
struct tm *mjddate (double mjd) {
    static struct tm date;    // ← 静态
    // ...
    return &date;
}

char *DateStr (double mjd) {
    static char datestr[256]; // ← 静态
    // ...
    return datestr;
}
```

**影响**：
1. **非线程安全**——多线程同时调用会数据竞争
2. **会被下一次调用覆盖**——`char *a = DateStr(m1); char *b = DateStr(m2);`
   此时 `a == b` 且都指向 `m2` 的结果

**orbitx-math 处理**：已修复——Rust 版返回 `CivilDate`（值类型）和 `String`（拥有
所有权），不存在此问题。

**是否修复**：✅ 已修复（Rust 天然无此问题）。

---

## 8. `qrdcmp` friend 声明使用默认参数（非标准 C++）

**文件**：`Src/Orbiter/Vecmat.h:230, 313`

**问题**：`qrdcmp` 的 friend 声明中使用了默认参数 `int *sing = 0`，
这在 C++ 标准中**不合法**（friend 声明若不是定义，不能有默认参数）：

```cpp
friend void qrdcmp (Matrix &a, Vector &c, Vector &d, int *sing = 0);
//                                                                  ^^^ 非标准
```

**影响**：MSVC 和 GCC 接受此扩展，但 **clang 严格模式拒绝编译**。在 macOS/Linux 上
用 clang 编译 Orbiter 源码时会产生硬错误。

**orbitx-math 处理**：`build.rs` 在编译前自动 strip 掉 `int *sing = 0` → `int *sing`
（这对 `orbitx-math` 的 Rust 实现无影响，仅影响 FFI oracle 编译）。

**是否修复**：🔧 应报告给 Orbiter 上游——只需删除 friend 声明中的 `= 0`，完全无副作用。

---

## 9. `Vecmat.h` — `Vector` 使用 `union` 含匿名 `struct`

**文件**：`Src/Orbiter/Vecmat.h:164`（及 Matrix:233, Quaternion:400, Vector4:272）

**问题**：Orbiter 的数学类型用 `union { double data[N]; struct { ... }; };` 实现
数组与命名字段的别名访问：

```cpp
union {
    double data[3];
    struct { double x, y, z; };  // ← 匿名 struct
};
```

**影响**：匿名结构体作为 union 成员是 C11 特性，在 C++ 中是**非标准扩展**
（虽然 MSVC/GCC/clang 都支持）。更重要的是，这导致 `Vector` 等**按值传递**时的
ABI 与 Rust 的 `#[repr(C)] struct` 不同——在 AArch64 上实测会导致 SIGSEGV。

**orbitx-math 处理**：Rust 用 `#[repr(C)] struct { x, y, z }` 实现相同内存布局，
FFI 接口全部改用**指针传递**避免 ABI 差异。

**是否修复**：🔧 不影响 Orbiter 本身（MSVC 编译），但影响跨语言 FFI。
可报告上游建议用命名 struct 成员替代匿名 struct。

---

## 10. `Orthodrome` 函数名拼写（非 bug）

**文件**：`Src/Orbiter/Astro.h:95, 100`；`Astro.cpp:107, 131`

**观察**：大圆距离函数命名为 `Orthodome`（少了一个 `r`），标准拼写是 `Orthodrome`
（源自希腊语 orthos + dromos）。这在 `orbitx-math` 的 Rust API 中已使用正确拼写
`orthodrome` / `orthodrome_dist`，但 FFI shim 中调用 C++ 时用了原始拼写 `Orthodome`。

**是否修复**：🔧 纯命名问题，不影响正确性。可报告上游。

---

## 11. VSOP87 `ReadData` — 精度过滤魔数 `tfac *= 5.0`

**文件**：`Src/Celbody/Vsop87/Vsop87.cpp:98`

**问题**：VSOP87 数据加载时的精度过滤使用一个未解释的魔数 `5.0` 来缩放误差估计：

```cpp
for (alpha = 0; alpha <= nalpha; alpha++) {
    // ...
    err = 2.0*sqrt(i+1.0)*a*tfac;
    if (err < prec) iused = i;
    // ...
    tfac *= 5.0; // don't ask
}
```

注释 `// don't ask`（别问）表明开发者自己也不确定这个 `5.0` 的来源。这可能是
一个经验近似——更高阶的时间幂项在远期会有更大的误差贡献，所以用 5 倍系数
递增过滤阈值。但这个选择缺乏理论依据。

**影响**：精度过滤的激进程度完全取决于这个魔数。如果改了它，会改变保留的项数，
从而改变历表精度和计算成本。

**orbitx-ephemeris 处理**：忠实保留 `tfac *= 5.0`，含原始注释。

**是否修复**：⚠️ 暂不修复。修改会影响保留的项数和历表精度。

---

## 12. ELP82 `INCLUDE_TIDAL_PERT` 默认禁用 — 潮汐/行星摄动项不计算

**文件**：`Src/Celbody/Moon/ELP82.cpp:6`

**问题**：ELP82 实现中，潮汐摄动、相对论修正和太阳偏心率摄动项被
`#ifdef INCLUDE_TIDAL_PERT ... #endif` 包裹，但 `INCLUDE_TIDAL_PERT`
**从未被定义**：

```cpp
// #define INCLUDE_TIDAL_PERT
// Uncomment this to add higher-order perturbation terms
// (tidal, relativistic, solar eccentricity)
// Warning: Using this can lead to inconsistencies since these
// effects are not currently modelled in Orbiter's dynamic model.
```

此外，即使在启用状态下，行星摄动部分（`PlanPerBin`）和图表/潮汐部分
（`FigurBin`）的读取循环也是**未完成的**——循环上限被硬编码为
`itab < 2/*12*/`（注释 12 表示本应到 12），说明只实现了 12 个摄动序列中的 1 个。

**影响**：月球历表精度低于完整 ELP2000-82 理论（约 0.1" vs 0.01"）。
对于 Orbiter 的用途（航天器仿真），这个精度足够。

**orbitx-ephemeris 处理**：忠实保留——只实现主问题序列（3 组），不包含摄动项。

**是否修复**：🔍 可在未来 Phase 4 中考虑补全，但需要完整的数据文件
（当前 `ELP82.dat` 只包含主问题项）。

---

## 13. `VsopEphem` — 直角坐标输出时 y↔z 互换

**文件**：`Src/Celbody/Vsop87/Vsop87.cpp:236-238`

**问题**：VSOP87 直角坐标（series A/E）输出时交换 y 和 z 分量：

```cpp
// swap y and z to map to orbiter system
tmp = ret[1]; ret[1] = ret[2]; ret[2] = tmp;
tmp = ret[4]; ret[4] = ret[5]; ret[5] = tmp;
```

这是因为 Orbiter 使用左手系（y 向黄道北极，z 正交），而标准天文学约定是
y 正交、z 向北极。

**影响**：这是 Orbiter 左手坐标系约定的一部分。极坐标输出（series B/D）不做
互换，因为极坐标不涉及轴顺序。

**orbitx-ephemeris 处理**：忠实保留——`vsop_eval_raw` 中对非极坐标做 y↔z 互换。

**是否修复**：❌ 不修复。这是左手系核心约定，全引擎统一。

---

## 14. ELP82 — 球坐标→直角坐标后 y↔z 互换

**文件**：`Src/Celbody/Moon/ELP82.cpp:409-416`

**问题**：ELP82 在完成岁差矩阵旋转后，将结果写入 `r[0..5]` 时**故意**把 y 和 z
分量互换（对比标准天文学约定）：

```cpp
// at this point we swap y and z components to conform with orbiter convention
// r[1] <-> r[2] and r[4] <-> r[5]
r[0] = pw2*x1+pwqw*x2+pw*x3;       // x (不换)
r[2] = pwqw*x1+qw2*x2-qw*x3;       // 本应是 y，写入 r[2]（z 位置）
r[1] = -pw*x1+qw*x2+(pw2+qw2-1)*x3;// 本应是 z，写入 r[1]（y 位置）
```

**影响**：与 #13 相同，这是 Orbiter 左手系约定。

**orbitx-ephemeris 处理**：忠实保留——`eval_into` 中按相同顺序写入 `ret`。

**是否修复**：❌ 不修复。

---

## 15. VSOP87 `ReadData` — `termlen` 哨兵行越界写入

**文件**：`Src/Celbody/Vsop87/Vsop87.cpp:109`

**问题**：`ReadData` 在每个坐标循环结束时，写入 `termlen` 的「哨兵行」：

```cpp
for (cooidx = 0; cooidx < 3; cooidx++) {
    for (alpha = 0; alpha <= nalpha; alpha++) {
        // ...
        termlen[alpha][cooidx] = iused;
    }
    termlen[alpha][cooidx] = 0; // ← alpha 此时 = nalpha+1，越界？
}
```

`termlen` 声明为 `IDX3 *termlen = new IDX3[nalpha+2]`，所以 `alpha=nalpha+1`
是合法的（分配了 `nalpha+2` 行）。这是**故意多分配一行**用于哨兵，但代码注释
中没有任何说明，初看像是越界写入。

**影响**：无实际 bug（分配了足够空间），但代码可读性差。`VsopEphem` 中的
`for (alpha = 0; termlen[alpha][cooidx]; alpha++)` 循环依赖这个哨兵行来终止。

**orbitx-ephemeris 处理**：使用 `termlen: [[usize; 3]; VSOP_MAXALPHA + 2]`，
哨兵行在 `from_reader` 中设置，逻辑清晰。

**是否修复**：🔧 Rust 版已通过显式数组大小和初始化改善了可读性。

---

## 16. Kepler `Calculate` — 左手系叉积 `H = V × R`

**文件**：`Src/Orbiter/Element.cpp:376+`

**问题**：角动量向量使用 `H = V × R`（而非标准的 `R × V`）：

```cpp
priv_H.Set (crossp (V, R)); // left-handed coordinates!
```

这是 Orbiter 左手坐标系的直接体现。在标准右手系中，角动量 `h = r × p = m(r × v)`。
在左手系中，叉积的方向反转，所以用 `V × R` 来补偿。

**影响**：所有从角动量导出的量（倾角 `i`、升交点 `theta`）都依赖这个约定。
用标准 `R × V` 会导致倾角符号反转。

**orbitx-dynamics 处理**：忠实保留——`kepler.rs` 中 `cross(v, r)` 并有注释标注。

**是否修复**：❌ 不修复。左手系核心约定。

---

## 17. Encke 方法 — 扰动项被 `#define` 禁用

**文件**：`Src/Orbiter/Rigidbody.cpp:368-404`；`BodyIntegrator.cpp:427-537`

**问题**：Encke 方法（轨道稳定化传播器）的扰动加速度函数
`GetIntermediateMoments_pert` 被宏定义为 `NO_GRAV_PERT`，返回零：

```cpp
#define GetIntermediateMoments_pert NO_GRAV_PERT  // returns zero
```

此外，完整的扰动 RK 驱动器（`RK2_Pert`/`RK4_Pert`/`RK8_Pert`/`RKdrv_Pert`）
被 `#ifdef UNDEF` 包裹，是死代码。

注释说明：「Encke's method in the current implementation doesn't seem very stable.
Therefore we simply disable gravitational perturbations altogether to revert to
a simple 2-body solution.」

**影响**：实际运行时 Encke 路径退化为纯 2-body Kepler 传播（使用 `Elements::PosVel`），
加上角运动的直接积分。完整 Encke 扰动修正从不执行。

**orbitx-dynamics 处理**：Phase 3 不实现 Encke 方法。

**是否修复**：🔍 未来阶段如果实现 Encke，应从 C++ 源码确认正确的扰动公式
（C++ 源码中的实现从未被验证过，因为有上述 bug）。

---

## 18. Pines 球谐重力 — km 单位 + y↔z 互换

**文件**：`Src/Orbiter/Psys.cpp:586-618`；`PinesGrav.cpp:185`

**问题**：Pines 球谐重力的调用涉及多重坐标/单位转换：

```cpp
Vector lpos = -tmul(rot, rpos) / 1000.0;  // m→km + 全局→体固
// swap y<->z (Orbiter left-handed → Pines right-handed)
double temp = lpos.y; lpos.y = lpos.z; lpos.z = temp;
dg = pinesAccel(lpos, maxDegreeOrder, maxDegreeOrder);
// swap back y<->z
temp = dg.y; dg.y = dg.z; dg.z = temp;
dg = mul(rot, dg) * 1000.0;  // km→m + 体固→全局
```

注意 `lpos` 前面的负号——这是从引力体指向测试点的向量取反（Pines 期望
从测试点指向引力体的向量）。

**影响**：任何 Pines 相关的测试/调试都需要正确处理这三层转换：
1. 米 → 千米（`/1000`）
2. 全局坐标 → 体固坐标（旋转矩阵 `GRot`）
3. 左手系 → 右手系（y↔z 互换）

**orbitx-dynamics 处理**：`PinesModel::accel` 接受 **km 单位**的右手系位置，
返回 **km/s²** 的右手系加速度。单位/坐标转换由调用者负责。

**是否修复**：🔧 Rust API 设计为显式接受 km/右手系，避免隐藏转换。
调用者需明确处理转换。

---

## 19. RK7/RK8 — 辛积分类角运动传播「靠猜」

**文件**：`Src/Orbiter/BodyIntegrator.cpp:296, 317, 350, 387`

**问题**：所有辛几何积分器（SY2/SY4/SY6/SY8）的角运动更新都使用一阶
`Q.Rotate(omega * step)`，而非常量更新中使用的 `EulerInv_full`：
```cpp
// Note: the propagation of angular state is guesswork ...
s1->Q.Rotate (s1->omega*step);
```

**影响**：辛几何积分器的线性部分（位置/速度）是真正辛的（能量守恒），
但角部分（四元数/角速度）只是一阶近似，不保辛性质。长时间运行时
姿态可能漂移。

**orbitx-dynamics 处理**：忠实保留——辛几何步进中使用一阶四元数更新。

**是否修复**：🔍 未来可改进角部分使用辛欧拉方程求解器。

---

## 汇总表

| # | 问题 | 类型 | orbitx-math 处理 | 建议 |
|---|------|------|-----------------|------|
| 1 | `date2mjd` tm_mon 1-based | 隐藏约定 | 忠实保留 | 暂不修复（数据兼容） |
| 2 | `conj` 忽略 this | API 设计错误 | ✅ 已修复 | — |
| 3 | premul/postmul 归一化禁用 | 可疑禁用 | 忠实保留 | 需调查 |
| 4 | Rotate desperation 重置 | 掩盖问题 | 忠实保留 | 加日志 |
| 5 | `inv` 无奇异检查 | 缺少检查 | 忠实保留 | 返回 Option |
| 6 | Equ2Ecl/Ecl2Equ 无 clamp | 域溢出 | 忠实保留 | ✅ 加 clamp |
| 7 | 静态返回缓冲区 | 线程不安全 | ✅ 已修复 | — |
| 8 | friend 默认参数 | 非标准 C++ | build.rs strip | 报告上游 |
| 9 | union 匿名 struct | 非标准扩展 | 指针 FFI | 报告上游 |
| 10 | Orthodome 拼写 | 命名 | Rust 用正确拼写 | 报告上游 |
| 11 | VSOP `tfac *= 5.0` 精度过滤 | 魔数/可疑 | 忠实保留 | 需调查 |
| 12 | ELP82 `INCLUDE_TIDAL_PERT` 默认禁用 | 功能不完整 | 忠实保留 | 需补全 |
| 13 | VsopEphem y↔z 互换 | 左手系约定 | 忠实保留 | — |
| 14 | ELP82 y↔z 互换 | 左手系约定 | 忠实保留 | — |
| 15 | VSOP `termlen` 哨兵越界写入 | 缓冲区复用 | 忠实保留 | 重构 |
| 16 | Kepler `H=V×R` 左手系叉积 | 左手系约定 | 忠实保留 | — |
| 17 | Encke 扰动项禁用 | 功能不完整 | 暂不实现 | 需重新设计 |
| 18 | Pines km 单位 + y↔z 互换 | 单位/坐标转换 | 显式 km/右手系 API | — |
| 19 | SY 角运动「靠猜」 | 一阶近似 | 忠实保留 | 可改进 |

---

## 维护说明

- 当发现新的 Orbiter 源码怪异行为时，请追加到本文件
- 当决定修复某个问题时，更新对应条目的「orbitx-math 处理」和「是否修复」列
- 修复决策原则：**不破坏与 Orbiter 数据格式的兼容性**（场景文件、历表数据、网格资产）
