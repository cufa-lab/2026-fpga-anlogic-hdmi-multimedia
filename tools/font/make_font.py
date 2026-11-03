#!/usr/bin/env python3
# -*- coding: utf-8 -*-
""" 讲解字幕：生成 TF 卡用的 HZK32 字库（32×32、每点 2 位灰度的抗锯齿字形，GB2312 汉字 + ASCII）。

字形：用 TrueType 字体按 4 倍超采样画出 32×32（ASCII 16×32），每点覆盖率量化成 0..3 四级。
      板上 2 倍大字（字幕 / 标题）直接用 32 点原样显示，1 倍小字（导语 / 全文页）取 2×2 平均，
      边缘半透明，和手机 / 电脑屏幕上的字一样平滑。
布局：每字 256 字节、一个扇区两个字。序号 fidx = (hi-0xA1)*94 + (lo-0xA1)，偏移 = fidx*256；
      ASCII 码 c 放在 GB2312 空着的第 10、11 区：fidx = 940 + c。共 87*94*256 = 2093568 字节。
      字内字节顺序与 FPGA 字形 RAM 一致（script_glyph_bram32k.v）：第 o 字节 →
        w = o>>4（第 2w、2w+1 行），h = (o>>3)&1（左 / 右 16 列），j = o&7（第 16h+2j、16h+2j+1 列）
        字节 = {p(2w,c), p(2w,c+1), p(2w+1,c), p(2w+1,c+1)}，c = 16h+2j，每点 2 位，高位在前
所有字形画在同一基线上（按"国"字墨迹框居中），标点 / 英文 / 数字在屏幕上高低一致。
默认字体：Windows 自带黑体 simhei.ttf（笔画均匀，小字也清楚）。

用法：
    python make_font.py                       # 输出到 ./font_out/HZK32
    python make_font.py --out D:/card_out
    python make_font.py --ttf C:/Windows/Fonts/msyh.ttc   # 换微软雅黑
"""
import argparse
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

DEFAULT_TTF = "C:/Windows/Fonts/simhei.ttf"
SS = 4                                   # 超采样倍数
ASC_BASE = 940                           # ASCII 的 fidx 起点（第 10 区）
NCHAR = 87 * 94


class Renderer:
    def __init__(self, ttf, asc_ttf=None, weight=1.0):
        self.font = ImageFont.truetype(str(ttf), 32 * SS)
        self.afont = ImageFont.truetype(str(asc_ttf or ttf), 32 * SS)
        self.weight = weight
        box = self.font.getbbox("国")
        self.hz_dy = (32 * SS - (box[3] - box[1])) // 2 - box[1]
        abox = self.afont.getbbox("Hg")
        self.asc_dy = (32 * SS - (abox[3] - abox[1])) // 2 - abox[1]

    def _render(self, ch, width, font, dy):
        """→ width×32 的 0..3 数组"""
        img = Image.new("L", (width * SS, 32 * SS), 0)
        draw = ImageDraw.Draw(img)
        w = draw.textlength(ch, font=font)
        draw.text((int(round((width * SS - w) / 2)), dy), ch, font=font, fill=255)
        cov = np.asarray(img, np.float32).reshape(32, SS, width, SS).mean(axis=(1, 3)) / 255.0
        return np.clip(np.round(cov * self.weight * 3), 0, 3).astype(np.uint8)

    def hanzi(self, ch):
        return self._render(ch, 32, self.font, self.hz_dy)

    def ascii(self, code):
        g = np.zeros((32, 32), np.uint8)
        if 33 <= code <= 126:
            g[:, :16] = self._render(chr(code), 16, self.afont, self.asc_dy)
        return g


def pack(g):
    """32×32 的 0..3 数组 → 256 字节（FPGA 字形 RAM 字序）"""
    out = bytearray(256)
    for o in range(256):
        w, h, j = o >> 4, (o >> 3) & 1, o & 7
        c = 16 * h + 2 * j
        r0, r1 = g[2 * w], g[2 * w + 1]
        out[o] = (int(r0[c]) << 6) | (int(r0[c + 1]) << 4) | (int(r1[c]) << 2) | int(r1[c + 1])
    return bytes(out)


def unpack(blob):
    """256 字节 → 32×32 的 0..3 数组（检查 / 仿真参考用）"""
    g = np.zeros((32, 32), np.uint8)
    for o in range(256):
        w, h, j = o >> 4, (o >> 3) & 1, o & 7
        c = 16 * h + 2 * j
        v = blob[o]
        g[2 * w, c], g[2 * w, c + 1], g[2 * w + 1, c], g[2 * w + 1, c + 1] = v >> 6, (v >> 4) & 3, (v >> 2) & 3, v & 3
    return g


def build(r, chars=None):
    """chars = None：整个字库；否则只画这些 fidx（其余全 0，仿真用的小卡）"""
    hzk = bytearray(NCHAR * 256)
    for fidx in range(NCHAR):
        if chars is not None and fidx not in chars:
            continue
        if ASC_BASE <= fidx < ASC_BASE + 128:
            g = r.ascii(fidx - ASC_BASE)
        else:
            hi, lo = 0xA1 + fidx // 94, 0xA1 + fidx % 94
            try:
                ch = bytes((hi, lo)).decode("gb2312")
            except UnicodeDecodeError:
                continue
            g = r.hanzi(ch)
        hzk[fidx * 256:(fidx + 1) * 256] = pack(g)
    return bytes(hzk)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ttf", default=DEFAULT_TTF, help="含 GB2312 汉字的字体（默认黑体）")
    ap.add_argument("--asc-ttf", default=None, help="ASCII 用的字体（默认同 --ttf）")
    ap.add_argument("--out", default="font_out", help="输出目录")
    ap.add_argument("--weight", type=float, default=1.0, help="笔画浓度（>1 更粗更实，默认 1.0）")
    a = ap.parse_args()
    r = Renderer(a.ttf, asc_ttf=a.asc_ttf, weight=a.weight)
    print("生成中（约 1 分钟）……", flush=True)
    hzk = build(r)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / "HZK32").write_bytes(hzk)
    print(f"HZK32 {len(hzk)} 字节 → {(out / 'HZK32').resolve()}")
    print("拷到 TF 卡根目录（文件名 HZK32，无扩展名）。旧的 HZK16 / ASC16 已不再使用，可以不拷。")


if __name__ == "__main__":
    main()
