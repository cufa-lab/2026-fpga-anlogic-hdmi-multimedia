#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""make_audio_corpus.py ：生成各种格式的 WAV → 转换 → 检查输出是否满足 FPGA 的 PCM 合同
运行：python test_make_audio_corpus.py      （只用标准库，不需要 ffmpeg）"""
import math
import os
import struct
import subprocess
import sys
import tempfile
import time
from array import array

HERE = os.path.dirname(os.path.abspath(__file__))
TOOL = os.path.join(HERE, 'make_audio_corpus.py')
fails = 0


def check(cond, msg):
    global fails
    print(('  ok   ' if cond else '  FAIL ') + msg)
    if not cond:
        fails += 1


def write_wav(path, rate, channels, bits, frames, tag=1, extensible=False):
    """frames: [[ch0, ch1, ...], ...]，值为 -1..1 浮点"""
    body = bytearray()
    for fr in frames:
        for v in fr:
            if tag == 3:
                body += struct.pack('<f' if bits == 32 else '<d', v)
            elif bits == 8:
                body += struct.pack('<B', max(0, min(255, round(v * 128) + 128)))
            elif bits == 16:
                body += struct.pack('<h', max(-32768, min(32767, round(v * 32768))))
            elif bits == 24:
                s = max(-8388608, min(8388607, round(v * 8388608)))
                body += struct.pack('<i', s)[0:3]
            elif bits == 32:
                body += struct.pack('<i', max(-2147483648, min(2147483647, round(v * 2147483648))))
    ba = channels * bits // 8
    if extensible:
        fmt = struct.pack('<HHIIHHHHI', 0xFFFE, channels, rate, rate * ba, ba, bits, 22, bits, 0)
        fmt += struct.pack('<H', tag) + b'\x00\x00\x00\x00\x10\x00\x80\x00\x00\xaa\x00\x38\x9b\x71'   # KSDATAFORMAT GUID 后 14 字节
    else:
        fmt = struct.pack('<HHIIHH', tag, channels, rate, rate * ba, ba, bits)
    riff = b'WAVE' + b'fmt ' + struct.pack('<I', len(fmt)) + fmt
    riff += b'LIST' + struct.pack('<I', 3) + b'abc\x00'          # 奇数长度的无关块（测填充字节）
    riff += b'data' + struct.pack('<I', len(body)) + bytes(body)
    with open(path, 'wb') as f:
        f.write(b'RIFF' + struct.pack('<I', len(riff)) + riff)


def run(args, cwd):
    r = subprocess.run([sys.executable, TOOL, '--no-ffmpeg'] + args, cwd=cwd,
                       stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    return r.returncode, r.stdout.decode('utf-8', 'replace') + r.stderr.decode('utf-8', 'replace')


def read_pcm(path):
    with open(path, 'rb') as f:
        raw = f.read()
    a = array('h'); a.frombytes(raw)
    if sys.byteorder == 'big': a.byteswap()
    return raw, a


def sine(rate, n, freq, amp, channels=1):
    return [[amp * math.sin(2 * math.pi * freq * i / rate)] * channels for i in range(n)]


def zero_cross_freq(a, n):
    z = sum(1 for i in range(1, n) if (a[i - 1] < 0) != (a[i] < 0))
    return z / 2 / (n / 48000)


def main():
    d = tempfile.mkdtemp(prefix='pcmtest_')
    print('临时目录', d)

    print('[1] 48k / 16bit / 单声道：样本原样、小端、补零到 512 整数倍')
    vals = [((i * 7919) % 65536 - 32768) / 32768.0 for i in range(1000)]
    write_wav(os.path.join(d, 'a.wav'), 48000, 1, 16, [[v] for v in vals])
    rc, out = run(['a.wav'], d)
    raw, a = read_pcm(os.path.join(d, 'TRACK1.PCM'))
    exp = [(i * 7919) % 65536 - 32768 for i in range(1000)]
    check(rc == 0, 'exit 0')
    check(list(a[:1000]) == exp, '1000 个样本逐个一致')
    check(len(raw) % 512 == 0 and len(raw) == 2048, '文件 %d 字节 = 整扇区' % len(raw))
    check(all(v == 0 for v in a[1000:]), '尾部补的是静音')
    check(raw[0:2] == struct.pack('<h', exp[0]), '小端：低字节在前')

    print('[2] 44.1k / 16bit / 立体声 1kHz → 48k 单声道，频率/幅度/时长不变')
    n = 44100
    write_wav(os.path.join(d, 'b.wav'), 44100, 2, 16, sine(44100, n, 1000, 0.5, 2))
    rc, out = run(['b.wav', '--first', '2'], d)
    raw, a = read_pcm(os.path.join(d, 'TRACK2.PCM'))
    f = zero_cross_freq(a, 48000)
    pk = max(abs(v) for v in a[100:47900]) / 32768
    check(rc == 0 and os.path.exists(os.path.join(d, 'TRACK2.PCM')), '--first 2 → TRACK2.PCM')
    check(abs(f - 1000) < 2, '频率 %.1f Hz' % f)
    check(abs(pk - 0.5) < 0.01, '峰值 %.3f' % pk)
    check(48000 <= len(a) < 48000 + 256, '1 秒 → %d 样本' % len(a))

    print('[3] 24bit / 32bit 整数 / float32 EXTENSIBLE 96k 立体声 / float64 / 8bit 22.05k')
    cases = [('c24.wav', 48000, 1, 24, 1, False), ('c32.wav', 48000, 1, 32, 1, False),
             ('cf32.wav', 96000, 2, 32, 3, True), ('cf64.wav', 48000, 1, 64, 3, False),
             ('c8.wav', 22050, 1, 8, 1, False)]
    for name, rate, ch, bits, tag, ext in cases:
        write_wav(os.path.join(d, name), rate, ch, bits, sine(rate, rate // 2, 440, 0.7, ch), tag, ext)
        for t in range(1, 5):
            p = os.path.join(d, 'TRACK%d.PCM' % t)
            if os.path.exists(p): os.remove(p)
        rc, out = run([name], d)
        raw, a = read_pcm(os.path.join(d, 'TRACK1.PCM'))
        f = zero_cross_freq(a, 24000)
        pk = max(abs(v) for v in a[200:23800]) / 32768
        check(rc == 0 and abs(f - 440) < 3 and abs(pk - 0.7) < 0.02,
              '%-9s %6d Hz %d 声道 %2d 位 → 440Hz 测得 %.1f Hz，峰值 %.3f' % (name, rate, ch, bits, f, pk))

    print('[4] 立体声取平均：L=+0.5、R=-0.5 → 0；L=0.6、R=0.2 → 0.4')
    write_wav(os.path.join(d, 'm.wav'), 48000, 2, 16, [[0.5, -0.5]] * 100 + [[0.6, 0.2]] * 100)
    rc, out = run(['m.wav'], d)
    raw, a = read_pcm(os.path.join(d, 'TRACK1.PCM'))
    check(all(v == 0 for v in a[:100]) and all(abs(v - 13107) <= 1 for v in a[100:200]), '混音正确')

    print('[5] --normalize：峰值拉到 -1 dBFS')
    write_wav(os.path.join(d, 'q.wav'), 48000, 1, 16, sine(48000, 4800, 1000, 0.1))
    rc, out = run(['q.wav', '--normalize'], d)
    raw, a = read_pcm(os.path.join(d, 'TRACK1.PCM'))
    pk = max(abs(v) for v in a) / 32768
    check(abs(pk - 0.891) < 0.005, '峰值 %.3f' % pk)

    print('[6] 出错处理')
    with open(os.path.join(d, 'adpcm.wav'), 'wb') as fh:
        fmt = struct.pack('<HHIIHH', 0x0011, 1, 48000, 24000, 256, 4)
        riff = b'WAVE' + b'fmt ' + struct.pack('<I', 16) + fmt + b'data' + struct.pack('<I', 256) + bytes(256)
        fh.write(b'RIFF' + struct.pack('<I', len(riff)) + riff)
    rc, out = run(['adpcm.wav'], d)
    check(rc == 1 and '需要 ffmpeg' in out, '压缩 WAV 无 ffmpeg → 报错退出')
    rc, out = run(['a.wav'] * 5, d)
    check(rc != 0 and 'TRACK1~TRACK4' in out, '超过 4 首 → 报错')
    rc, out = run(['a.wav', 'a.wav', '--first', '4'], d)
    check(rc != 0, '--first 4 再放 2 首 → 报错')
    rc, out = run(['不存在.wav'], d)
    check(rc == 1 and '不存在' in out, '文件不存在 → 报错')

    print('[7] 速度：3 分钟 44.1k 立体声 16bit')
    n = 44100 * 180
    one = struct.pack('<hh', 1000, -1000)
    body = one * n
    fmt = struct.pack('<HHIIHH', 1, 2, 44100, 44100 * 4, 4, 16)
    riff = b'WAVE' + b'fmt ' + struct.pack('<I', 16) + fmt + b'data' + struct.pack('<I', len(body)) + body
    with open(os.path.join(d, 'long.wav'), 'wb') as fh:
        fh.write(b'RIFF' + struct.pack('<I', len(riff)) + riff)
    t0 = time.time()
    rc, out = run(['long.wav'], d)
    dt = time.time() - t0
    size = os.path.getsize(os.path.join(d, 'TRACK1.PCM'))
    check(rc == 0 and size == ((48000 * 180 * 2 + 511) // 512) * 512, '用时 %.1f s，输出 %d 字节（%.1f MB）' % (dt, size, size / 1e6))
    print(out.strip().splitlines()[1] if rc == 0 else out)

    print('\n结果：' + ('全部通过' if fails == 0 else '%d 项失败' % fails))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
