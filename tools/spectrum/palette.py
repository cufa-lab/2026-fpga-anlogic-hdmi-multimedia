# 频谱柱"深空"循环色表（32 项）：8 个关键色，相邻关键色之间 4 等分线性插值，再预乘 75%（柱身不透明度）
#   c = ((Ka*(4-f) + Kb*f) >> 2) * 3 >> 2，tb_goertzel_spectrum_bars 用同一公式独立计算
# 用法：python tools/spectrum/palette.py  → 打印 RTL case 项（粘进 goertzel_spectrum_bars.v 的 @@PAL 段）
KEYS = [
    (30, 60, 255),    # 深蓝（O 型星）
    (0, 170, 255),    # 天蓝
    (0, 230, 210),    # 青（OIII 发射线）
    (255, 215, 80),   # 金
    (255, 140, 30),   # 橙
    (255, 60, 90),    # 玫红（Hα）
    (230, 40, 200),   # 品红
    (140, 60, 255),   # 紫
]

def palette():
    out = []
    for i in range(32):
        a, f = KEYS[i >> 2], i & 3
        b = KEYS[((i >> 2) + 1) & 7]
        out.append(tuple((((a[c] * (4 - f) + b[c] * f) >> 2) * 3) >> 2 for c in range(3)))
    return out

if __name__ == "__main__":
    for i, (r, g, b) in enumerate(palette()):
        print("            5'd%-2d: pal = 24'h%02x%02x%02x;" % (i, r, g, b))
