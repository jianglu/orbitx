//! TASS17 土星卫星历表：解析器 + 求值器。
//!
//! 复刻 Orbiter 的 `Src/Celbody/Satsat/Tass17.cpp`。TASS17 理论
//! （Vienne & Duriez 1995, A&A 297, 588-605）覆盖土星的 8 颗卫星：
//! Mimas、Enceladus、Tethys、Dione、Rhea、Titan、Hyperion、Iapetus。
//!
//! Hyperion 使用独立的解析理论（`elemhyp`/`lithyp`），其余 7 颗使用
//! 通用的级数求和（`calclon`/`calcelem`/`edered`）。

use std::io::{BufRead, BufReader};

const TWO_PI: f64 = 6.2831853071795862;

/// TASS17 项：(振幅 a, 相位 ph, 频率 fr)。
#[derive(Clone, Copy, Debug, Default)]
pub struct TasTerm {
    pub a: f64,
    pub ph: f64,
    pub fr: f64,
}

/// 8 个整数乘子（每卫星平经度的线性组合系数）。
#[derive(Clone, Copy, Debug, Default)]
pub struct TasIks(pub [i32; 8]);

/// 单颗卫星的级数数据（不含 Hyperion）。
#[derive(Clone, Debug, Default)]
pub struct TasSat {
    /// `ntr[0..3]` = 4 级数的项数；`ntr[4]` = 平经度项数（nt1）。
    pub ntr: [usize; 5],
    /// 4 级数的项数据。
    pub terms: [Vec<TasTerm>; 4],
    /// 4 级数对应的整数乘子。
    pub iks: [Vec<TasIks>; 4],
    /// 平经度常数项。
    pub al0: f64,
    /// 平经度线性项。
    pub an0: f64,
}

/// Hyperion 独立理论数据块。
#[derive(Clone, Debug)]
pub struct HyperionBlock {
    pub t0: f64,
    pub amm7: f64,
    pub cstp: f64,
    pub cstq: f64,
    pub serp: Vec<f64>,
    pub fap: Vec<f64>,
    pub frp: Vec<f64>,
    pub serq: Vec<f64>,
    pub faq: Vec<f64>,
    pub frq: Vec<f64>,
    pub serz: Vec<f64>,
    pub faz: Vec<f64>,
    pub frz: Vec<f64>,
    pub serzt: Vec<f64>,
    pub fazt: Vec<f64>,
    pub frzt: Vec<f64>,
}

impl Default for HyperionBlock {
    fn default() -> Self {
        Self {
            t0: 0.0,
            amm7: 0.0,
            cstp: 0.0,
            cstq: 0.0,
            serp: Vec::new(),
            fap: Vec::new(),
            frp: Vec::new(),
            serq: Vec::new(),
            faq: Vec::new(),
            frq: Vec::new(),
            serz: Vec::new(),
            faz: Vec::new(),
            frz: Vec::new(),
            serzt: Vec::new(),
            fazt: Vec::new(),
            frzt: Vec::new(),
        }
    }
}

/// 完整的 TASS17 模型。
pub struct TasModel {
    /// 8 颗卫星的级数数据（索引 6=Hyperion 的 TasSat 为空，使用 hyp）。
    pub sats: [TasSat; 8],
    /// Hyperion 独立理论。
    pub hyp: HyperionBlock,
    /// 全局参数。
    pub gk1: f64,
    pub aia: f64,
    pub oma: f64,
    pub aam: [f64; 9],
    pub tmas: [f64; 9],
}

// 物理常数（Satsat.cpp:106-107）
const AU_METERS: f64 = 299_792_458.0 * 499.004783806;
const AU_YEAR: f64 = AU_METERS / (86400.0 * 365.25);
// 历元参考日（Tass17.cpp:107, 162）
const TASS_EPOCH: f64 = 2_444_240.0;

impl TasModel {
    /// 从 `tass17.dat` 读取数据。
    ///
    /// 逐符号复刻 `ReadData`（Tass17.cpp:181）。
    pub fn from_reader<R: BufRead>(reader: R) -> std::io::Result<Self> {
        let mut tok = TokenStream::new(reader);

        let radsdg = std::f64::consts::FRAC_PI_4 / 45.0; // atan(1)/45 = pi/180

        let gk = tok.next_f64()?;
        let tas = tok.next_f64()?;
        let gk1 = (gk * 365.25).powf(2.0) / tas;
        let mut aia = tok.next_f64()?;
        let mut oma = tok.next_f64()?;
        aia *= radsdg;
        oma *= radsdg;

        let mut tam = [0.0_f64; 9];
        for i in 0..9 {
            tam[i] = tok.next_f64()?;
        }
        let mut tmas = [0.0_f64; 9];
        for i in 0..9 {
            tmas[i] = 1.0 / tam[i];
        }
        let mut am = [0.0_f64; 9];
        for i in 0..9 {
            am[i] = tok.next_f64()?;
        }
        let mut aam = [0.0_f64; 9];
        for i in 0..9 {
            aam[i] = am[i] * 365.25;
        }

        let mut sats: [TasSat; 8] = Default::default();

        // 读 8 卫星（跳过 Hyperion 索引 6）。
        for i in 0..8usize {
            if i == 6 {
                continue;
            }
            for j in 0..4usize {
                let _is = tok.next_int()?;
                let ieq = tok.next_int()?;
                let nt1 = tok.next_int()? as usize;
                let nt2 = tok.next_int()? as usize;
                let nt = nt2; // res=0 → 用 nt2（完整精度）

                sats[i].ntr[j] = nt;

                if ieq == 2 {
                    let _k = tok.next_int()?;
                    sats[i].al0 = tok.next_f64()?;
                    sats[i].an0 = tok.next_f64()?;
                    sats[i].ntr[4] = nt1;
                }

                // 预分配。
                sats[i].terms[j] = Vec::with_capacity(nt);
                sats[i].iks[j] = Vec::with_capacity(nt);

                for k in 0..nt2 {
                    let _n = tok.next_int()?;
                    let a = tok.next_f64()?;
                    let ph = tok.next_f64()?;
                    let fr = tok.next_f64()?;
                    let mut ik = [0i32; 8];
                    for m in 0..8 {
                        ik[m] = tok.next_int()? as i32;
                    }
                    if k < nt {
                        sats[i].terms[j].push(TasTerm { a, ph, fr });
                        sats[i].iks[j].push(TasIks(ik));
                    }
                }
            }
        }

        // 读 Hyperion 独立块。
        let hyp = read_hyperion(&mut tok)?;

        Ok(TasModel {
            sats,
            hyp,
            gk1,
            aia,
            oma,
            aam,
            tmas,
        })
    }

    /// 在儒略日 `jd` 处求值卫星 `isat`（0-7），返回 [位置(3), 速度(3)]。
    ///
    /// 单位为米和米/秒，已做 xzy 轴交换。
    /// 复刻 `posired`（Tass17.cpp:82）+ `SatEphem` 单位转换（Satsat.cpp:89）。
    pub fn eval(&self, jd: f64, isat: usize) -> [f64; 6] {
        let mut elem = [0.0_f64; 6];
        let mut dlo = [0.0_f64; 8];

        if isat == 6 {
            self.elemhyp(jd, &mut elem);
        } else {
            self.calclon(jd, &mut dlo);
            self.calcelem(jd, isat, &mut elem, &dlo);
        }

        let mut xyz = [0.0_f64; 3];
        let mut vxyz = [0.0_f64; 3];
        self.edered(&elem, &mut xyz, &mut vxyz, isat);

        // 单位转换：AU→米、AU/年→米/秒、xzy 轴交换（Satsat.cpp:108-113）
        [
            xyz[0] * AU_METERS,
            xyz[2] * AU_METERS,
            xyz[1] * AU_METERS,
            vxyz[0] * AU_YEAR,
            vxyz[2] * AU_YEAR,
            vxyz[1] * AU_YEAR,
        ]
    }

    /// 计算 8 颗卫星的平经度（复刻 `calclon`，Tass17.cpp:157）。
    fn calclon(&self, jd: f64, dlo: &mut [f64; 8]) {
        let t = (jd - TASS_EPOCH) / 365.25;
        for is in 0..8 {
            if is != 6 {
                let sat = &self.sats[is];
                let tm = &sat.terms[1];
                let mut s = 0.0;
                for i in 0..sat.ntr[4] {
                    s += tm[i].a * (tm[i].ph + t * tm[i].fr).sin();
                }
                dlo[is] = s;
            } else {
                dlo[is] = 0.0;
            }
        }
    }

    /// 计算单颗卫星的 6 元素（复刻 `calcelem`，Tass17.cpp:99）。
    fn calcelem(&self, jd: f64, is: usize, elem: &mut [f64; 6], dlo: &[f64; 8]) {
        let t = (jd - TASS_EPOCH) / 365.25;
        let sat = &self.sats[is];

        // elem[0] = 半长轴扰动：series[0] 余弦求和。
        let mut s = 0.0;
        let tm = &sat.terms[0];
        let ik = &sat.iks[0];
        for i in 0..sat.ntr[0] {
            let mut phas = tm[i].ph;
            for jk in 0..8 {
                phas += ik[i].0[jk] as f64 * dlo[jk];
            }
            s += tm[i].a * (phas + t * tm[i].fr).cos();
        }
        elem[0] = s;

        // elem[1] = 平经度：dlo + al0 + series[1] 正弦求和（从 ntr[4] 到 ntr[1]）。
        s = dlo[is] + sat.al0;
        let tm = &sat.terms[1];
        let ik = &sat.iks[1];
        for i in sat.ntr[4]..sat.ntr[1] {
            let mut phas = tm[i].ph;
            for jk in 0..8 {
                phas += ik[i].0[jk] as f64 * dlo[jk];
            }
            s += tm[i].a * (phas + t * tm[i].fr).sin();
        }
        s += sat.an0 * t;
        elem[1] = s.sin().atan2(s.cos());

        // elem[2,3] = 偏心率向量 (k*cos, k*sin)：series[2]。
        let mut s1 = 0.0;
        let mut s2 = 0.0;
        let tm = &sat.terms[2];
        let ik = &sat.iks[2];
        for i in 0..sat.ntr[2] {
            let mut phas = tm[i].ph;
            for jk in 0..8 {
                phas += ik[i].0[jk] as f64 * dlo[jk];
            }
            s1 += tm[i].a * (phas + t * tm[i].fr).cos();
            s2 += tm[i].a * (phas + t * tm[i].fr).sin();
        }
        elem[2] = s1;
        elem[3] = s2;

        // elem[4,5] = 倾角向量 (h*cos, h*sin)：series[3]。
        s1 = 0.0;
        s2 = 0.0;
        let tm = &sat.terms[3];
        let ik = &sat.iks[3];
        for i in 0..sat.ntr[3] {
            let mut phas = tm[i].ph;
            for jk in 0..8 {
                phas += ik[i].0[jk] as f64 * dlo[jk];
            }
            s1 += tm[i].a * (phas + t * tm[i].fr).cos();
            s2 += tm[i].a * (phas + t * tm[i].fr).sin();
        }
        elem[4] = s1;
        elem[5] = s2;
    }

    /// 元素→直角坐标（复刻 `edered`，Tass17.cpp:235）。
    ///
    /// 包含 Newton-Raphson Kepler 方程求解。
    fn edered(&self, elem: &[f64; 6], xyz: &mut [f64; 3], vxyz: &mut [f64; 3], isat: usize) {
        let amo = self.aam[isat] * (elem[0] + 1.0);
        let rmu = self.gk1 * (self.tmas[isat] + 1.0);
        let dga = (rmu / (amo * amo)).powf(0.33333333333333331);
        let rl = elem[1];
        let rk = elem[2];
        let rh = elem[3];

        // Newton-Raphson Kepler 方程求解。
        let mut fle = rl - rk * rl.sin() + rh * rl.cos();
        loop {
            let cf = fle.cos();
            let sf = fle.sin();
            let corf = (rl - fle + rk * sf - rh * cf) / (1.0 - rk * cf - rh * sf);
            fle += corf;
            if corf.abs() < 1e-14 {
                break;
            }
        }
        let cf = fle.cos();
        let sf = fle.sin();
        let dlf = -rk * sf + rh * cf;
        let rsam1 = -rk * cf - rh * sf;
        let asr = 1.0 / (rsam1 + 1.0);
        let phi = (1.0 - rk * rk - rh * rh).sqrt();
        let psi = 1.0 / (phi + 1.0);
        let x1 = dga * (cf - rk - psi * rh * dlf);
        let y1 = dga * (sf - rh + psi * rk * dlf);
        let vx1 = amo * asr * dga * (-sf - psi * rh * rsam1);
        let vy1 = amo * asr * dga * (cf + psi * rk * rsam1);

        // 倾角向量旋转。
        let dwho = (1.0 - elem[5] * elem[5] - elem[4] * elem[4]).sqrt() * 2.0;
        let rtp = 1.0 - elem[5] * 2.0 * elem[5];
        let rtq = 1.0 - elem[4] * 2.0 * elem[4];
        let rdg = elem[5] * 2.0 * elem[4];
        let xyz2_0 = x1 * rtp + y1 * rdg;
        let xyz2_1 = x1 * rdg + y1 * rtq;
        let xyz2_2 = (-x1 * elem[5] + y1 * elem[4]) * dwho;
        let vxyz2_0 = vx1 * rtp + vy1 * rdg;
        let vxyz2_1 = vx1 * rdg + vy1 * rtq;
        let vxyz2_2 = (-vx1 * elem[5] + vy1 * elem[4]) * dwho;

        // 轨道平面→黄道旋转（倾角 aia + 升交点 oma）。
        let ci = self.aia.cos();
        let si = self.aia.sin();
        let co = self.oma.cos();
        let so = self.oma.sin();
        xyz[0] = co * xyz2_0 - so * ci * xyz2_1 + so * si * xyz2_2;
        xyz[1] = so * xyz2_0 + co * ci * xyz2_1 - co * si * xyz2_2;
        xyz[2] = si * xyz2_1 + ci * xyz2_2;
        vxyz[0] = co * vxyz2_0 - so * ci * vxyz2_1 + so * si * vxyz2_2;
        vxyz[1] = so * vxyz2_0 + co * ci * vxyz2_1 - co * si * vxyz2_2;
        vxyz[2] = si * vxyz2_1 + ci * vxyz2_2;
    }

    /// Hyperion 独立理论求值（复刻 `elemhyp`，Tass17.cpp:319）。
    fn elemhyp(&self, dj: f64, elem: &mut [f64; 6]) {
        let hyp = &self.hyp;
        let t = dj - hyp.t0;

        let mut p = hyp.cstp;
        for i in 0..hyp.serp.len() {
            let wt = t * hyp.frp[i] + hyp.fap[i];
            p += hyp.serp[i] * wt.cos();
        }

        let mut q = hyp.cstq;
        for i in 0..hyp.serq.len() {
            let wt = t * hyp.frq[i] + hyp.faq[i];
            q += hyp.serq[i] * wt.sin();
        }

        let mut zr = 0.0;
        let mut zi = 0.0;
        for i in 0..hyp.serz.len() {
            let wt = t * hyp.frz[i] + hyp.faz[i];
            zr += hyp.serz[i] * wt.cos();
            zi += hyp.serz[i] * wt.sin();
        }

        let mut ztr = 0.0;
        let mut zti = 0.0;
        for i in 0..hyp.serzt.len() {
            let wt = t * hyp.frzt[i] + hyp.fazt[i];
            ztr += hyp.serzt[i] * wt.cos();
            zti += hyp.serzt[i] * wt.sin();
        }

        let mut vl = (hyp.amm7 * t + q) % TWO_PI;
        if vl < 0.0 {
            vl += TWO_PI;
        }

        elem[0] = p;
        elem[1] = vl;
        elem[2] = zr;
        elem[3] = zi;
        elem[4] = ztr;
        elem[5] = zti;
    }
}

/// 读取 Hyperion 独立块（复刻 `lithyp`，Tass17.cpp:291）。
fn read_hyperion<R: BufRead>(tok: &mut TokenStream<R>) -> std::io::Result<HyperionBlock> {
    let t0 = tok.next_f64()?;
    let amm7 = tok.next_f64()?;

    let nbtp = tok.next_int()? as usize;
    let cstp = tok.next_f64()?;
    let mut serp = Vec::with_capacity(nbtp);
    let mut fap = Vec::with_capacity(nbtp);
    let mut frp = Vec::with_capacity(nbtp);
    for _ in 0..nbtp {
        serp.push(tok.next_f64()?);
        fap.push(tok.next_f64()?);
        frp.push(tok.next_f64()?);
    }

    let nbtq = tok.next_int()? as usize;
    let cstq = tok.next_f64()?;
    let mut serq = Vec::with_capacity(nbtq);
    let mut faq = Vec::with_capacity(nbtq);
    let mut frq = Vec::with_capacity(nbtq);
    for _ in 0..nbtq {
        serq.push(tok.next_f64()?);
        faq.push(tok.next_f64()?);
        frq.push(tok.next_f64()?);
    }

    let nbtz = tok.next_int()? as usize;
    let mut serz = Vec::with_capacity(nbtz);
    let mut faz = Vec::with_capacity(nbtz);
    let mut frz = Vec::with_capacity(nbtz);
    for _ in 0..nbtz {
        serz.push(tok.next_f64()?);
        faz.push(tok.next_f64()?);
        frz.push(tok.next_f64()?);
    }

    let nbtzt = tok.next_int()? as usize;
    let mut serzt = Vec::with_capacity(nbtzt);
    let mut fazt = Vec::with_capacity(nbtzt);
    let mut frzt = Vec::with_capacity(nbtzt);
    for _ in 0..nbtzt {
        serzt.push(tok.next_f64()?);
        fazt.push(tok.next_f64()?);
        frzt.push(tok.next_f64()?);
    }

    Ok(HyperionBlock {
        t0,
        amm7,
        cstp,
        cstq,
        serp,
        fap,
        frp,
        serq,
        faq,
        frq,
        serz,
        faz,
        frz,
        serzt,
        fazt,
        frzt,
    })
}

// --- 空白分隔 token 读取器（与 vsop87.rs/elp82.rs 相同模式） ---

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
                        "TASS17 数据意外结束",
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
    fn parse_minimal_tass() {
        // 构建一个最小的 TASS17 数据文件进行解析测试。
        // 包含全局参数 + 1 颗卫星（Mimas）的 4 级数（各 1 项）+ Hyperion。
        let mut data = String::new();
        // gk, tas
        data.push_str("0.01720209895\n3498.790\n");
        // aia(度), oma(度)
        data.push_str("28.0512 169.5291\n");
        // 9 个逆质量
        data.push_str("0.1577287066246e+08 0.6666666666667e+07 0.9433962264151e+06 0.5094243504840e+06 0.2314814814815e+06 0.4225863977890e+04 0.3333333333333e+08 0.3225806451613e+06 0.2858130953844e-03\n");
        // 9 个平运动
        data.push_str("0.6667061728782e+01 0.4585536751534e+01 0.3328306445055e+01 0.2295717646433e+01 0.1390853715957e+01 0.3940425676910e+00 0.2953088138695e+00 0.7920197763193e-01 0.5839811452566e-03\n");

        // 卫星 0 (Mimas) 的 4 级数。
        // series 0: is=1 ieq=1 nt1=1 nt2=1
        data.push_str("1 1 1 1\n1 0.001 0.0 0.1\n0 0 0 0 0 0 0 0\n");
        // series 1: is=1 ieq=2 nt1=1 nt2=1
        data.push_str("1 2 1 1\n1 0.5 0.01\n1 0.75 0.0 0.089\n0 0 0 0 0 0 0 0\n");
        // series 2: is=1 ieq=3 nt1=1 nt2=1
        data.push_str("1 3 1 1\n1 0.02 0.0 0.2\n0 0 0 0 0 0 0 0\n");
        // series 3: is=1 ieq=4 nt1=1 nt2=1
        data.push_str("1 4 1 1\n1 0.03 0.0 0.3\n0 0 0 0 0 0 0 0\n");

        // 其余 6 颗非 Hyperion 卫星（索引 1-5, 7）各 4 级数（简化为各 1 项）。
        for sat_idx in [1, 2, 3, 4, 5, 7] {
            for series in 0..4 {
                let ieq = if series == 1 { 2 } else { series + 1 };
                data.push_str(&format!("{sat_idx} {ieq} 1 1\n"));
                if ieq == 2 {
                    data.push_str("1 0.5 0.01\n");
                }
                data.push_str("1 0.001 0.0 0.1\n0 0 0 0 0 0 0 0\n");
            }
        }

        // Hyperion 数据块。
        data.push_str("2444240.0\n"); // t0
        data.push_str("0.295\n"); // amm7
        data.push_str("2\n0.001\n"); // nbtp, cstp
        data.push_str("0.01 0.0 0.1\n0.02 0.5 0.2\n"); // 2 p-terms
        data.push_str("1\n0.005\n"); // nbtq, cstq
        data.push_str("0.03 0.0 0.3\n"); // 1 q-term
        data.push_str("1\n"); // nbtz
        data.push_str("0.04 0.0 0.4\n"); // 1 z-term
        data.push_str("1\n"); // nbtzt
        data.push_str("0.05 0.0 0.5\n"); // 1 zt-term

        let model = TasModel::from_reader(data.as_bytes()).unwrap();
        assert!(model.gk1 > 0.0);
        assert_eq!(model.sats[0].terms[0].len(), 1); // Mimas series 0
        assert_eq!(model.sats[0].ntr[4], 1); // 平经度项数
        assert_eq!(model.hyp.serp.len(), 2); // Hyperion p-terms
    }

    #[test]
    fn eval_returns_finite() {
        // 用上面的最小数据集，验证 eval 返回有限值。
        let mut data = String::new();
        data.push_str("0.01720209895\n3498.790\n28.0512 169.5291\n");
        data.push_str("0.1577287066246e+08 0.6666666666667e+07 0.9433962264151e+06 0.5094243504840e+06 0.2314814814815e+06 0.4225863977890e+04 0.3333333333333e+08 0.3225806451613e+06 0.2858130953844e-03\n");
        data.push_str("0.6667061728782e+01 0.4585536751534e+01 0.3328306445055e+01 0.2295717646433e+01 0.1390853715957e+01 0.3940425676910e+00 0.2953088138695e+00 0.7920197763193e-01 0.5839811452566e-03\n");
        data.push_str("1 1 1 1\n1 0.001 0.0 0.1\n0 0 0 0 0 0 0 0\n");
        data.push_str("1 2 1 1\n1 0.5 0.01\n1 0.75 0.0 0.089\n0 0 0 0 0 0 0 0\n");
        data.push_str("1 3 1 1\n1 0.02 0.0 0.2\n0 0 0 0 0 0 0 0\n");
        data.push_str("1 4 1 1\n1 0.03 0.0 0.3\n0 0 0 0 0 0 0 0\n");
        for sat_idx in [1, 2, 3, 4, 5, 7] {
            for series in 0..4 {
                let ieq = if series == 1 { 2 } else { series + 1 };
                data.push_str(&format!("{sat_idx} {ieq} 1 1\n"));
                if ieq == 2 {
                    data.push_str("1 0.5 0.01\n");
                }
                data.push_str("1 0.001 0.0 0.1\n0 0 0 0 0 0 0 0\n");
            }
        }
        data.push_str("2444240.0\n0.295\n2\n0.001\n0.01 0.0 0.1\n0.02 0.5 0.2\n1\n0.005\n0.03 0.0 0.3\n1\n0.04 0.0 0.4\n1\n0.05 0.0 0.5\n");

        let model = TasModel::from_reader(data.as_bytes()).unwrap();
        let ret = model.eval(2_451_545.0, 0); // Mimas at J2000
        for i in 0..6 {
            assert!(ret[i].is_finite(), "ret[{i}] = {} 不有限", ret[i]);
        }
    }
}
