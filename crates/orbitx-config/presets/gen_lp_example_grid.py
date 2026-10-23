#!/usr/bin/env python3
"""生成示例发射场 lp_cape.toml 的覆盖网格 lp_cape_grid.f32。

合成演示数据（非真实测绘）：中心 8 m 台面、向足迹边缘二次衰减到 0，
满足「最外圈向全球高程收敛」的数据契约示意。改格距/尺寸时同步改 lp。
"""
import math
import struct

W = H = 64
CELL = 10.0
half = (W - 1) * CELL / 2.0

out = bytearray()
for row in range(H):
    north = ((H - 1) / 2.0 - row) * CELL
    for col in range(W):
        east = (col - (W - 1) / 2.0) * CELL
        r = math.hypot(east, north) / half
        h = 8.0 * max(0.0, 1.0 - r * r)
        out += struct.pack("<f", h)

with open("lp_cape_grid.f32", "wb") as f:
    f.write(out)
print(f"wrote lp_cape_grid.f32 ({len(out)} bytes, {W}x{H} @ {CELL} m)")
