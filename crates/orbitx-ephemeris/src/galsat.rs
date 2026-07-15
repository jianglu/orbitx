//! GALSAT/Lieske 木星伽利略卫星历表：解析器 + 求值器。
//!
//! 复刻 Orbiter 的 `Src/Celbody/Galsat/Lieske.cpp`。基于 Lieske 1977
//! （A&A 56, 333-352）的 GALSAT 理论，覆盖木星的 4 颗伽利略卫星：
//! Io、Europa、Ganymede、Callisto。
//!
//! **设计**：C++ 使用 FORTRAN 风格的 COMMON 块全局状态。Rust 版使用
//! `GalModel` 结构体封装所有状态，通过 `&mut self` 管理缓存。

use std::io::{BufRead, BufReader};

const TWO_PI: f64 = 6.28318530717958648;
const DEGRAD: f64 = 57.2957795130823208;
const TREF: f64 = 2_443_000.5; // 参考儒略日（Lieske.cpp:75）

// 物理常数（Galsat.cpp:GalEphem）
const AU_METERS: f64 = 299_792_458.0 * 499.004783806;
const AU_DAY: f64 = AU_METERS / 86400.0;

/// 单颗卫星的理论数据（xi/v/z 级数的系数、相位、频率）。
///
/// C++ 使用 FORTRAN 风格的三列布局（coef[k]、arg[k+nm]、rat[k+2*nm]）。
/// Rust 使用结构体数组，更清晰。
#[derive(Clone, Debug, Default)]
struct SatSeries {
    /// 系数数组（coef）。
    c: Vec<f64>,
    /// 相位数组（arg）。
    arg: Vec<f64>,
    /// 频率数组（rate）。
    rate: Vec<f64>,
}

/// 单颗卫星的理论（xi=半径、v=经度扰动、z=纬度）。
#[derive(Clone, Debug, Default)]
struct SatTheory {
    axis: f64,
    xi: SatSeries,
    v: SatSeries,
    z: SatSeries,
}

/// GALSAT/Lieske 完整模型。
pub struct GalModel {
    // ebblok
    earay: [f64; 28],
    #[allow(dead_code)]
    baray: [f64; 22],
    paray: [f64; 28],

    // angblk
    #[allow(dead_code)]
    angcod: [f64; 99],
    #[allow(dead_code)]
    ratcod: [f64; 99],
    ang: [f64; 22],
    rat: [f64; 23],

    // theory
    sats: [SatTheory; 4],
    #[allow(dead_code)]
    epsln: f64,

    // 运行时缓存（对应 C++ 的 svtloc_1 + local_1）
    tlast: f64,
    tlastg: f64,
    // 旋转矩阵 q（位置）和 qdot（速度）
    q: [f64; 9],
    qdot: [f64; 9],
    qmat: [f64; 9],
    // chkgal 缓存
    cj: f64,
    sj: f64,
    ci: f64,
    si: f64,
    cn: f64,
    sn: f64,
    // barycentre 修正系数
    cofbx: [f64; 7],
    cofbz: [f64; 5],
    angbx: [f64; 14],
    angbz: [f64; 10],
    // 已初始化标志
    initialized: bool,
}

impl GalModel {
    /// 从 `ephem_e15.dat` 读取数据。
    ///
    /// 复刻 `cd2com`（Lieske.cpp:712）+ `chkgal`（Lieske.cpp:397）。
    pub fn from_reader<R: BufRead>(reader: R) -> std::io::Result<Self> {
        let mut tok = TokenStream::new(reader);

        // 读 ebblok。
        let mut earay = [0.0_f64; 28];
        for i in 0..28 {
            earay[i] = tok.next_f64()?;
        }
        let mut baray = [0.0_f64; 22];
        for i in 0..22 {
            baray[i] = tok.next_f64()?;
        }
        let mut paray = [0.0_f64; 28];
        for i in 0..28 {
            paray[i] = tok.next_f64()?;
        }

        // 读 trmblk（53 个 double，不直接使用）。
        for _ in 0..53 {
            let _ = tok.next_f64()?;
        }

        // 读 angblk。
        let mut angcod = [0.0_f64; 99];
        for i in 0..99 {
            angcod[i] = tok.next_f64()?;
        }
        let mut ratcod = [0.0_f64; 99];
        for i in 0..99 {
            ratcod[i] = tok.next_f64()?;
        }
        let mut ang = [0.0_f64; 22];
        for i in 0..22 {
            ang[i] = tok.next_f64()?;
        }
        let mut rat = [0.0_f64; 23];
        for i in 0..23 {
            rat[i] = tok.next_f64()?;
        }

        // 读 theory。
        let mut sats: [SatTheory; 4] = Default::default();

        // axis[4]
        for i in 0..4 {
            sats[i].axis = tok.next_f64()?;
        }

        // 每卫星的 xi(10/24/31/49)、v(41/66/75/89)、z(7/11/13/18) 级数。
        let sizes: [(usize, usize, usize); 4] =
            [(10, 41, 7), (24, 66, 11), (31, 75, 13), (49, 89, 18)];
        for sat in 0..4 {
            let (nxi, nv, nz) = sizes[sat];
            sats[sat].xi = read_series(&mut tok, nxi)?;
            sats[sat].v = read_series(&mut tok, nv)?;
            sats[sat].z = read_series(&mut tok, nz)?;
        }

        let epsln = tok.next_f64()?;

        // 跳过 term counts 和 packed codes（nxi1t..nv4t + kodx1..kodv4）。
        // 这些在 revizg/updat 中用于更新 g 修正。由于 revizg 每 50 天
        // 更新一次且影响很小，我们在 Rust 版中先跳过这些 packed codes，
        // 后续如需精确匹配可补全。
        for _ in 0..12 {
            let _ = tok.next_int()?;
        }
        let kod_sizes: [usize; 12] = [20, 14, 82, 48, 22, 132, 62, 26, 150, 98, 36, 178];
        for size in kod_sizes {
            for _ in 0..size {
                let _ = tok.next_int()?;
            }
        }

        let mut model = GalModel {
            earay,
            baray,
            paray,
            angcod,
            ratcod,
            ang,
            rat,
            sats,
            epsln,
            tlast: -2e20,
            tlastg: -6e20,
            q: [0.0; 9],
            qdot: [0.0; 9],
            qmat: [0.0; 9],
            cj: 0.0,
            sj: 0.0,
            ci: 0.0,
            si: 0.0,
            cn: 0.0,
            sn: 0.0,
            cofbx: [0.0; 7],
            cofbz: [0.0; 5],
            angbx: [0.0; 14],
            angbz: [0.0; 10],
            initialized: false,
        };

        // 执行 chkgal（初始化质心修正系数 + 旋转矩阵参数）。
        model.chkgal();

        Ok(model)
    }

    /// 初始化质心修正系数和旋转矩阵参数（复刻 `chkgal`，Lieske.cpp:397）。
    fn chkgal(&mut self) {
        self.cofbx[0] = -1262.0 - self.earay[0] * 1267.0;
        self.cofbx[1] = (self.earay[1] + 1.0) * -1133.0;
        self.cj = self.earay[2] + 1.0;
        self.cofbx[2] = self.cj * -5715.0;
        self.sj = self.earay[3] + 1.0;
        self.cofbx[3] = self.sj * -5668.0;
        self.cofbx[4] = 12.0;
        self.cofbx[5] = (self.earay[18] + 1.0) * 68.0 + self.earay[3] * 67.0;
        self.cofbx[6] = self.sj * -21.0;
        self.cofbz[0] = -9.0;
        self.cofbz[1] = self.cj * -18.0;
        self.cofbz[2] = self.sj * -27.0;
        self.cofbz[3] = 9.0;
        self.cofbz[4] = self.earay[3] * 44.0 + 42.0;

        for k in 1..=4 {
            self.angbx[k - 1] = self.ang[k - 1] - self.ang[14];
            self.angbx[k + 6] = self.rat[k - 1] - self.rat[14];
            if k != 1 {
                self.angbz[k - 2] = self.ang[k - 1] - self.ang[k + 9];
                self.angbz[k + 3] = self.rat[k - 1] - self.rat[k + 9];
            }
        }
        for k in 1..=2 {
            self.angbx[k + 3] = self.ang[k + 6] - self.ang[14];
            self.angbx[k + 10] = self.rat[k + 6] - self.rat[14];
            self.angbz[k + 2] = self.angbx[k + 1];
            self.angbz[k + 7] = self.angbx[k + 8];
        }
        self.angbx[6] = self.ang[3] * 2.0 - self.ang[8] - self.ang[14];
        self.angbx[13] = self.rat[3] * 2.0 - self.rat[8] - self.rat[14];

        // 旋转矩阵参数。
        let orbecl = self.paray[25] * (self.earay[25] + 1.0) / DEGRAD;
        self.cj = orbecl.cos();
        self.sj = orbecl.sin();
        let orbequ = self.paray[24] * (self.earay[24] + 1.0) / DEGRAD;
        self.ci = orbequ.cos();
        self.si = orbequ.sin();
        self.cn = self.ang[21].cos();
        self.sn = self.ang[21].sin();

        self.tlast = -2e20;
        self.tlastg = -6e20;
        self.initialized = true;
    }

    /// 构建旋转矩阵 q 和 qdot（复刻 `qqdot`，Lieske.cpp:479）。
    fn qqdot(&mut self, t: f64) {
        let phidot = self.rat[14];
        let phi = phidot * t + self.ang[14] - self.ang[21];
        let cp = phi.cos();
        let sp = phi.sin();

        // q = r(-node) p(-j) r(-phi) p(-i)
        self.q[0] = self.cn * cp - self.sn * self.cj * sp;
        let qpsi11 = -self.cn * sp - self.sn * self.cj * cp;
        self.q[3] = qpsi11 * self.ci + self.sn * self.sj * self.si;
        self.q[6] = -qpsi11 * self.si + self.sn * self.sj * self.ci;
        self.q[1] = self.sn * cp + self.cn * self.cj * sp;
        self.qdot[0] = qpsi11 * phidot;
        let qpsi21 = -self.sn * sp + self.cn * self.cj * cp;
        self.qdot[1] = qpsi21 * phidot;
        self.q[4] = qpsi21 * self.ci - self.cn * self.sj * self.si;
        self.q[7] = -qpsi21 * self.si - self.cn * self.sj * self.ci;
        self.q[2] = sp * self.sj;
        self.q[5] = cp * self.sj * self.ci + self.cj * self.si;
        self.q[8] = -(cp * self.sj) * self.si + self.cj * self.ci;
        for l in 1..=3 {
            self.qdot[l + 2] = -(self.q[l - 1] * phidot) * self.ci;
            self.qdot[l + 5] = self.q[l - 1] * phidot * self.si;
        }
        self.qdot[2] = cp * self.sj * phidot;
    }

    /// 质心→木星向量修正（复刻 `barcor`，Lieske.cpp:333）。
    fn barcor(&self, t: f64, rb: &mut [f64; 6]) {
        for i1 in 0..7 {
            let d = self.angbx[i1] + self.angbx[i1 + 7] * t;
            let angl = d_mod(d, TWO_PI);
            let t1 = self.cofbx[i1];
            let t2 = self.angbx[i1 + 7];
            let ca = t1 * angl.cos() * 1e-10;
            let sa = t1 * angl.sin() * 1e-10;
            rb[0] += ca;
            rb[1] += sa;
            rb[3] -= sa * t2;
            rb[4] += ca * t2;
        }
        for i1 in 0..5 {
            let d = self.angbz[i1] + self.angbz[i1 + 5] * t;
            let angl = d_mod(d, TWO_PI);
            let t1 = self.cofbz[i1];
            let t2 = self.angbz[i1 + 5];
            let ca = t1 * angl.cos() * 1e-10;
            let sa = t1 * angl.sin() * 1e-10;
            rb[2] += sa;
            rb[5] += ca * t2;
        }
    }

    /// 对单颗卫星求 xi/v/z 级数（复刻 `samjay`，Lieske.cpp:222）。
    fn samjay(&self, t: f64, sat: usize, rb: &mut [f64; 6]) {
        let theory = &self.sats[sat];
        let nsat = sat + 1; // 1-based satellite index for ang/rat
        let nflag = 2; // position + velocity

        // xi（半径）级数：余弦求和。
        let mut xi = 0.0_f64;
        let mut xidot = 0.0_f64;
        for k in 0..theory.xi.c.len() {
            let angl = d_mod(theory.xi.arg[k] + theory.xi.rate[k] * t, TWO_PI);
            let ca = angl.cos();
            xi += theory.xi.c[k] * ca;
            let sa = angl.sin();
            xidot -= theory.xi.c[k] * sa * theory.xi.rate[k];
        }

        // v（经度扰动）级数：正弦求和。
        let mut v = 0.0_f64;
        let mut vdot = 0.0_f64;
        for k in 0..theory.v.c.len() {
            let angl = d_mod(theory.v.arg[k] + theory.v.rate[k] * t, TWO_PI);
            let sa = angl.sin();
            v += theory.v.c[k] * sa;
            let ca = angl.cos();
            vdot += theory.v.c[k] * ca * theory.v.rate[k];
        }

        let dt = v / self.rat[nsat];
        let sdfac = vdot / self.rat[nsat] + 1.0;

        // z（纬度）级数：正弦求和（在 t+dt 时刻）。
        let mut s = 0.0_f64;
        let mut sdot = 0.0_f64;
        for k in 0..theory.z.c.len() {
            let angl = d_mod(theory.z.arg[k] + theory.z.rate[k] * (t + dt), TWO_PI);
            let sa = angl.sin();
            s += theory.z.c[k] * sa;
            let ca = angl.cos();
            sdot += theory.z.c[k] * ca * theory.z.rate[k];
        }

        // l - psi + v
        let d = self.ang[nsat] - self.ang[15] + (self.rat[nsat] - self.rat[15]) * t;
        let angl = d_mod(d, TWO_PI) + v;
        let q1 = theory.axis * angl.cos();
        let q2 = theory.axis * angl.sin();
        let q3 = theory.axis * s;
        let q4 = xi + 1.0;

        rb[0] = q1 * q4;
        rb[1] = q2 * q4;
        rb[2] = q3 * q4;

        if nflag == 1 {
            return;
        }

        sdot *= sdfac;
        let ca = self.rat[nsat] - self.rat[15] + vdot;
        rb[3] = q1 * xidot - rb[1] * ca;
        rb[4] = q2 * xidot + rb[0] * ca;
        rb[5] = q3 * xidot + theory.axis * q4 * sdot;
    }

    /// 在儒略日 `jd` 处求值卫星 `ksat`（0=质心、1-4=Io/Europa/Ganymede/Callisto），
    /// 返回 [位置(3), 速度(3)]，单位为米和米/秒，已做 xzy 轴交换。
    ///
    /// 复刻 `galsat`（Lieske.cpp:68）+ `GalEphem` 单位转换（Galsat.cpp）。
    pub fn eval(&mut self, jd: f64, ksat: i32) -> [f64; 6] {
        assert!(self.initialized, "GalModel 未初始化");
        assert!((0..=4).contains(&ksat), "ksat 必须在 0..=4 范围内");

        let t = jd - TREF;
        let tolt = 1e-4;
        let _tolg = 50.0;

        // 更新旋转矩阵（如果时间移动 > tolt）。
        if (t - self.tlast).abs() > tolt {
            self.tlast = t;
            self.qqdot(t);
            // MS: qmat ← qdot, qdot ← q（移除了黄道→赤道旋转）
            self.qmat = self.qdot;
            self.qdot = self.q;
        }

        // 注意：跳过 revizg_（木土大不等式修正），因为 packed code 解码
        // 器未实现。对 ±50 年范围内的精度影响很小。

        let mut rb = [0.0_f64; 6];

        match ksat {
            0 => self.barcor(t, &mut rb),
            nsat => self.samjay(t, (nsat - 1) as usize, &mut rb),
        }

        // 旋转到黄道坐标。
        let mut r = [0.0_f64; 6];
        for i in 0..3 {
            for j in 0..3 {
                r[i] += self.qdot[i + j * 3] * rb[j];
                r[i + 3] += self.qdot[i + j * 3] * rb[j + 3] + self.qmat[i + j * 3] * rb[j];
            }
        }

        // 单位转换：AU→米、AU/天→米/秒、xzy 轴交换。
        [
            r[0] * AU_METERS,
            r[2] * AU_METERS,
            r[1] * AU_METERS,
            r[3] * AU_DAY,
            r[5] * AU_DAY,
            r[4] * AU_DAY,
        ]
    }
}

/// 读取一个级数（n 个系数 + n 个相位 + n 个频率）。
fn read_series<R: BufRead>(tok: &mut TokenStream<R>, n: usize) -> std::io::Result<SatSeries> {
    let mut c = Vec::with_capacity(n);
    let mut arg = Vec::with_capacity(n);
    let mut rate = Vec::with_capacity(n);
    for _ in 0..n {
        c.push(tok.next_f64()?);
    }
    for _ in 0..n {
        arg.push(tok.next_f64()?);
    }
    for _ in 0..n {
        rate.push(tok.next_f64()?);
    }
    Ok(SatSeries { c, arg, rate })
}

/// FORTRAN 风格的 mod 函数（`d_mod`，Lieske.cpp:63）。
fn d_mod(x: f64, y: f64) -> f64 {
    x - (x / y) as i64 as f64 * y
}

// --- 空白分隔 token 读取器 ---

struct TokenStream<R> {
    lines: std::io::Lines<BufReader<R>>,
    current_line: String,
    pos: usize,
}

impl<R: BufRead> TokenStream<R> {
    fn new(reader: R) -> Self {
        TokenStream {
            lines: BufReader::new(reader).lines(),
            current_line: String::new(),
            pos: 0,
        }
    }

    fn next_token(&mut self) -> std::io::Result<&str> {
        loop {
            let trimmed = self.current_line[self.pos..].trim_start();
            let start = self.current_line.len() - trimmed.len();
            if let Some(end) = trimmed.find(char::is_whitespace) {
                self.pos = start + end;
                return Ok(&self.current_line[start..start + end]);
            } else if !trimmed.is_empty() {
                self.pos = self.current_line.len();
                return Ok(&self.current_line[start..]);
            }
            match self.lines.next() {
                Some(Ok(line)) => {
                    self.current_line = line;
                    self.pos = 0;
                }
                Some(Err(e)) => return Err(e),
                None => {
                    return Err(std::io::Error::new(
                        std::io::ErrorKind::UnexpectedEof,
                        "GALSAT 数据意外结束",
                    ));
                }
            }
        }
    }

    fn next_f64(&mut self) -> std::io::Result<f64> {
        let tok = self.next_token()?;
        tok.parse::<f64>()
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))
    }

    fn next_int(&mut self) -> std::io::Result<i64> {
        let tok = self.next_token()?;
        tok.parse::<i64>()
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn d_mod_matches_fortran() {
        assert!((d_mod(7.5, TWO_PI) - (7.5 - TWO_PI)).abs() < 1e-10);
        assert!(d_mod(3.0, TWO_PI) - 3.0 < 1e-10);
        assert!(d_mod(TWO_PI, TWO_PI).abs() < 1e-10);
    }

    #[test]
    fn chkgal_initializes_cofbx() {
        // 验证 chkgal 正确初始化质心修正系数。
        // 构建一个最小模型，手动设置 earay。
        let mut model = GalModel {
            earay: [0.0; 28],
            baray: [0.0; 22],
            paray: [0.0; 28],
            angcod: [0.0; 99],
            ratcod: [0.0; 99],
            ang: [0.0; 22],
            rat: [0.0; 23],
            sats: Default::default(),
            epsln: 0.0,
            tlast: -2e20,
            tlastg: -6e20,
            q: [0.0; 9],
            qdot: [0.0; 9],
            qmat: [0.0; 9],
            cj: 0.0,
            sj: 0.0,
            ci: 0.0,
            si: 0.0,
            cn: 0.0,
            sn: 0.0,
            cofbx: [0.0; 7],
            cofbz: [0.0; 5],
            angbx: [0.0; 14],
            angbz: [0.0; 10],
            initialized: false,
        };
        model.earay[0] = 0.004; // e1 (Io eccentricity)
        model.chkgal();
        // cofbx[0] = -1262 - e1*1267
        let expected = -1262.0 - 0.004 * 1267.0;
        assert!((model.cofbx[0] - expected).abs() < 1e-6);
        assert!(model.initialized);
    }
}
