#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import os
import re
import sys

MAX_BYTES, MAX_W, MAX_LINES, MAX_GLYPH, MAX_CELLS, MAX_ROWS, ROW_W = 4096, 72, 110, 256, 512, 16, 72
NO_BREAK = {bytes.fromhex(h) for h in 'a1a2 a1a3 a1ad a1af a1b1 a1b3 a1b5 a1b7 a1b9 a1bb a1bd a1bf '
            'a3a1 a3a9 a3ac a3ae a3ba a3bb a3bf'.split()} | {c.encode() for c in ',.;:!?)]'}


def para_rows(cells):
    """全文页段落排版（与 script_loader 相同）：每行 ROW_W 半宽，首行缩进 4，行首不放标点（可挤出 2）"""
    rows, fill = 0, 4
    for c in cells:
        w = len(c)
        n = fill + w
        if n > ROW_W and not (c in NO_BREAK and n <= ROW_W + 2):
            rows += 1; fill = w
        else:
            fill = n
    return rows + (1 if fill != (4 if rows == 0 else 0) else 0)


def decode(raw):
    if raw.startswith(b'\xef\xbb\xbf'):
        return raw[3:].decode('utf-8'), 'UTF-8 带 BOM'
    for enc, name in (('gb2312', 'GB2312'), ('utf-8', 'UTF-8'), ('gbk', 'GBK')):
        try:
            return raw.decode(enc), name
        except UnicodeDecodeError:
            pass
    raise SystemExit('无法识别编码（不是 UTF-8 / GB2312 / GBK）')


def main():
    ap = argparse.ArgumentParser(description='检查 / 转换扩1 讲解稿 TRACKn.LRC')
    ap.add_argument('lrc')
    ap.add_argument('--out', help='转换后写到这里（GB2312 + CRLF）')
    ap.add_argument('--voc', help='对应的 TRACKn.VOC（检查时长）')
    a = ap.parse_args()
    raw = open(a.lrc, 'rb').read()
    text, enc = decode(raw)
    errs, warns = [], []
    title = intro = ''
    timed = []
    glyphs = set()
    cells = 0
    stream = []
    for no, line in enumerate(text.replace('\r\n', '\n').replace('\r', '\n').split('\n'), 1):
        s = line.strip()
        if not s:
            continue
        m = re.match(r'^\[(\d{1,3}):(\d{1,2})(?:\.(\d{1,3}))?\](.*)$', s)
        t = None
        if m:
            t = int(m.group(1)) * 6000 + int(m.group(2)) * 100 + int(((m.group(3) or '0') + '00')[:2])
            body = m.group(4)
        elif s.lower().startswith('[ti:') and s.endswith(']'):
            body = s[4:-1]
        elif s.lower().startswith('[ar:') and s.endswith(']'):
            body = s[4:-1]
        else:
            if not s.startswith('['):
                warns.append('第 %d 行没有时间标签，会被忽略：%s' % (no, s))
            continue
        try:
            gb = body.encode('gb2312')
        except UnicodeEncodeError as e:
            bad = body[e.start]
            errs.append('第 %d 行有 GB2312 里没有的字符 "%s"（U+%04X），请换成常用字 / 全角符号（ERR 71）' % (no, bad, ord(bad)))
            continue
        if len(gb) > MAX_W:
            warns.append('第 %d 行 %d 半宽 > %d，FPGA 会截断（ERR 76）：%s' % (no, len(gb), MAX_W, body))
            cut, i = 0, 0
            while i < len(gb):                     # 按字截断（不拆半个汉字），与 FPGA 一致
                n = 2 if gb[i] >= 0xA1 else 1
                if cut + n > MAX_W:
                    break
                cut += n; i += n
            gb = gb[:cut]
        i, cl = 0, []
        while i < len(gb):
            n = 2 if gb[i] >= 0xA1 else 1
            c = gb[i:i + n].replace(b'	', b' ')
            glyphs.add(c); cl.append(c); i += n
            cells += 1
        if t is None:
            if timed:
                errs.append('第 %d 行：[ti:] / [ar:] 必须写在所有时间行前面' % no)
            if len(gb) > 36:
                warns.append('第 %d 行标题 / 导语 %d 半宽 > 36，标题会显示不全' % (no, len(gb)))
            if s.lower().startswith('[ti:'):
                title = body
            else:
                intro = body
        else:
            timed.append((t, body, len(gb), no))
            stream += cl
    if len(raw) > MAX_BYTES:
        errs.append('文件 %d 字节 > %d（ERR 75），请精简讲解词' % (len(raw), MAX_BYTES))
    if enc != 'GB2312' and not a.out:
        errs.append('编码是 %s，FPGA 只认 GB2312 无 BOM（ERR 71）——加 --out TRACKn.LRC 自动转换' % enc)
    if len(timed) > MAX_LINES:
        errs.append('时间行 %d > %d（ERR 72）' % (len(timed), MAX_LINES))
    if len(glyphs) > MAX_GLYPH:
        errs.append('不同的字 %d 个 > %d（ERR 72），超出的字不显示，请精简或分段' % (len(glyphs), MAX_GLYPH))
    nrows = para_rows(stream)
    if nrows > MAX_ROWS:
        errs.append('全文页段落 %d 行 > %d（ERR 72），超出部分全文页不显示' % (nrows, MAX_ROWS))
    if cells > MAX_CELLS:
        errs.append('总字数 %d > %d（ERR 72）' % (cells, MAX_CELLS))
    for k in range(1, len(timed)):
        if timed[k][0] < timed[k - 1][0]:
            warns.append('第 %d 行时间比上一句早，FPGA 按文件顺序播放，会立刻跳过' % timed[k][3])
    if a.voc and timed:
        dur = os.path.getsize(a.voc) // 512 * 256 / 24000 * 100
        if timed[-1][0] >= dur:
            errs.append('最后一句时间 %.2f s ≥ 人声时长 %.2f s，永远显示不到' % (timed[-1][0] / 100, dur / 100))
    print('编码 %s，%d 字节；标题 "%s"；导语 "%s"' % (enc, len(raw), title, intro))
    print('讲解 %d 句，不同的字 %d / %d，总字数 %d / %d，全文页段落 %d / %d 行'
          % (len(timed), len(glyphs), MAX_GLYPH, cells, MAX_CELLS, nrows, MAX_ROWS))
    for t, body, w, no in timed:
        print('  [%02d:%05.2f] %s' % (t // 6000, t % 6000 / 100, body))
    for w in warns:
        print('提示：' + w)
    for e in errs:
        print('错误：' + e)
    if a.out and not [e for e in errs if 'ERR 71' in e]:
        lines = text.replace('\r\n', '\n').replace('\r', '\n').split('\n')
        out = '\r\n'.join(l.rstrip() for l in lines).strip('\r\n') + '\r\n'
        data = out.encode('gb2312')
        open(a.out, 'wb').write(data)
        print('已写出 %s（GB2312 + CRLF，%d 字节）' % (a.out, len(data)))
    print('结果：' + ('通过' if not errs else '有 %d 个错误' % len(errs)))
    return 1 if errs else 0


if __name__ == '__main__':
    sys.exit(main())
