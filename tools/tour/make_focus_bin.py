# -*- coding: utf-8 -*-
"""
make_focus_bin.py —— 把手写的 FOCUS.TXT（导览焦点表）转成 FPGA 直拷用的 FOCUS.BIN

用法（只用 Python 标准库）：
    python tools/make_focus_bin.py  FOCUS.TXT  FOCUS.BIN
    （不写参数时默认就是当前目录下的 FOCUS.TXT → FOCUS.BIN）
然后把 FOCUS.BIN 复制到 TF 卡根目录，卡上不要再放 FOCUS.TXT（名字是 FOCUS 的文件只取第一个）。

FOCUS.TXT 写法与以前完全一样（ASCII，大小写不敏感，每行一张图）：
    PICnn: x,y,zoom ; x,y,zoom ; ...      （1~4 个焦点）
  nn   = 播放序号 1..64（PIC1 / PIC01 都行）
  x, y = 原图像素坐标（左上角为原点，0..4095，超出按 4095）
  zoom = 放大倍数 1 / 1.5 / 2 / ... / 4（只看 1 位小数，x.0~x.4 → x，x.5~x.9 → x.5；< 1 按 1，> 4 按 4）
  行首空白 / UTF-8 BOM 忽略；不是 PIC 开头的行整行跳过（可写注释）；一组不完整（缺 y 或 zoom）就丢掉该组；
  遇到其它字符：已完整的组照收，本行余下跳过。只看文件前 4096 字节。

FOCUS.BIN 格式：固定 1024 字节，256 项 × 4 字节小端，第 a 项对应 FPGA 焦点表地址 a = 图片号·4 + 焦点号，
  内容 = x << 15 | y << 3 | z，z = 2·zoom - 1（1..7），z = 0 表示没有该焦点。

下面的解析器逐字节照搬原 FPGA focus_loader.v（第二批之前的版本）的状态机，结果与原来 FPGA 解析逐位相同
（sim/tb_opt2_focus_eqv.v 用原版 RTL 比对验证）。
"""
import sys

P_BOL, P_I, P_C, P_N, P_G, P_DEC, P_SKIP = range(7)
MAX_TEXT = 4096


def parse(data):
    tab = [0] * 256
    st = dict(pst=P_BOL, num=0, ndig=0, img=0, k=0, fld=0, val=0, vdig=0, fx=0, fy=0, dhi=0, dseen=0)

    def grp_reset(n):
        n.update(fld=0, val=0, vdig=0, dhi=0, dseen=0)

    def commit(o, n):
        # o = 本拍开始时的值（Verilog 非阻塞赋值读旧值），n = 本拍写入的新值
        grp_ok = o['fld'] == 2 and o['vdig'] and o['k'] != 4
        if grp_ok:
            z2r = (o['val'] * 2 + o['dhi']) & 0x1FFF
            zc = 1 if z2r < 2 else 7 if z2r >= 8 else ((z2r & 7) - 1) & 7
            tab[o['img'] * 4 + (o['k'] & 3)] = (o['fx'] << 15) | (o['fy'] << 3) | zc
            n['k'] = (o['k'] + 1) & 7
        grp_reset(n)

    for c in data[:MAX_TEXT]:
        o = dict(st)
        n = st
        cu = c - 0x20 if 0x61 <= c <= 0x7A else c
        is_dig = 0x30 <= c <= 0x39
        dv = c & 15
        is_sp = c in (0x20, 0x09)
        is_eol = c in (0x0A, 0x0D)
        pst = o['pst']
        if pst == P_BOL:
            if cu == 0x50:
                n['pst'] = P_I
            elif not (is_sp or is_eol or (c & 0x80)):
                n['pst'] = P_SKIP
        elif pst == P_I:
            n['pst'] = P_C if cu == 0x49 else (P_BOL if is_eol else P_SKIP)
        elif pst == P_C:
            n['pst'] = P_N if cu == 0x43 else (P_BOL if is_eol else P_SKIP)
            n['num'] = 0
            n['ndig'] = 0
        elif pst == P_N:
            num = o['num']
            if is_dig and o['ndig'] != 2:
                n['num'] = ((num & 15) * 8 + (num & 31) * 2 + dv) & 127
                n['ndig'] = o['ndig'] + 1
            elif c == 0x3A and o['ndig'] != 0 and num != 0 and num <= 64:
                n['img'] = (num - 1) & 63
                n['k'] = 0
                grp_reset(n)
                n['pst'] = P_G
            elif not is_sp:
                n['pst'] = P_BOL if is_eol else P_SKIP
        elif pst in (P_G, P_DEC):
            if is_dig:
                if pst == P_G:
                    n['val'] = min(o['val'] * 10 + dv, 4095)
                    n['vdig'] = 1
                elif not o['dseen']:
                    n['dseen'] = 1
                    n['dhi'] = 1 if dv >= 5 else 0
            elif c == 0x2C and pst == P_G and o['fld'] != 2:          # ','
                if o['fld'] == 0:
                    n['fx'] = o['val']
                else:
                    n['fy'] = o['val']
                n['fld'] = o['fld'] + 1
                n['val'] = 0
                n['vdig'] = 0
            elif c == 0x2E and pst == P_G and o['fld'] == 2:          # '.'
                n['pst'] = P_DEC
            elif c == 0x3B:                                           # ';'
                commit(o, n)
                n['pst'] = P_G
            elif is_eol:
                commit(o, n)
                n['pst'] = P_BOL
            elif not is_sp:
                commit(o, n)
                n['pst'] = P_SKIP
        else:                                                         # P_SKIP
            if is_eol:
                n['pst'] = P_BOL
    if st['pst'] in (P_G, P_DEC):                                     # 文件末尾没有换行：收最后一组
        commit(dict(st), st)
    return tab


def to_bin(tab):
    out = bytearray()
    for w in tab:
        out += (w & 0x7FFFFFF).to_bytes(4, 'little')
    return bytes(out)


def main():
    src = sys.argv[1] if len(sys.argv) > 1 else 'FOCUS.TXT'
    dst = sys.argv[2] if len(sys.argv) > 2 else 'FOCUS.BIN'
    data = open(src, 'rb').read()
    if len(data) > MAX_TEXT:
        print('注意：%s 有 %d 字节，只看前 %d 字节（与原 FPGA 解析一致）' % (src, len(data), MAX_TEXT))
    tab = parse(data)
    open(dst, 'wb').write(to_bin(tab))
    n = 0
    for img in range(64):
        foci = []
        for k in range(4):
            w = tab[img * 4 + k]
            if w & 7:
                foci.append('(%d,%d,%gx)' % (w >> 15, (w >> 3) & 4095, ((w & 7) + 1) / 2))
        if foci:
            n += len(foci)
            print('PIC%-2d: %s' % (img + 1, ' '.join(foci)))
    print('共 %d 个焦点 → %s（1024 字节）' % (n, dst))


if __name__ == '__main__':
    main()
