#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FPGA（wav_stream_player）不解析文件头，只认一种裸数据：
    48000 Hz、16 位有符号、小端、单声道、无文件头
本脚本负责转成这种格式，并按 TF 卡上的命名规则输出 TRACK1.PCM ~ TRACK4.PCM。

用法（Windows 命令行，在本文件所在目录或写全路径）：
    python make_audio_corpus.py 歌1.wav 歌2.wav 歌3.wav          -> 当前目录生成 TRACK1/2/3.PCM
    python make_audio_corpus.py a.wav b.wav -o 卡的盘符根目录    -> 直接写到 TF 卡根目录（如卡是 E 盘就写 E 盘根目录）
    python make_audio_corpus.py c.wav --first 3                  -> 生成 TRACK3.PCM
    python make_audio_corpus.py song.mp3                         -> mp3/flac/m4a 等需要电脑装了 ffmpeg
    python make_audio_corpus.py a.wav --normalize                -> 顺便把音量拉到满幅（-1 dBFS）

输入格式：
  * WAV 用 Python 自带功能解码，不需要装任何东西：8/16/24/32 位整数、32/64 位浮点、
    任意声道数、任意采样率（含 WAVE_FORMAT_EXTENSIBLE）
  * 其他格式（mp3/flac/m4a/ogg/压缩 WAV…）：电脑上有 ffmpeg 时自动调用（在 PATH 里或 --ffmpeg 指定路径）
  * 装了 ffmpeg 时 WAV 也优先交给 ffmpeg 转（重采样质量更好）；--no-ffmpeg 强制用内置转换

处理：多声道取平均混成单声道 → 重采样到 48 kHz → （可选）归一化 → 16 位小端
      → 末尾补静音到 512 字节整数倍（FPGA 按扇区读，不满一扇区的尾巴会被丢弃，补齐后一个样本都不丢）

拷卡规则（重要，FPGA 只记文件起点和长度，不跟 FAT 碎片链，文件必须在卡上连续）：
  1. TF 卡格式化为 FAT32
  2. 先拷 TRACKn.PCM，再拷 BMP 图片
  3. 文件放根目录，名字必须是 TRACK1.PCM ~ TRACK4.PCM（可以缺号）
"""
import argparse
import os
import shutil
import struct
import subprocess
import sys
from array import array

TARGET_RATE = 48000
SECTOR = 512
MAX_TRACKS = 4

# WAV 格式码
WAVE_FORMAT_PCM = 0x0001
WAVE_FORMAT_IEEE_FLOAT = 0x0003
WAVE_FORMAT_EXTENSIBLE = 0xFFFE


class ConvertError(Exception):
    pass


# ----------------------------------------------------------------------------
# WAV 解码（只用标准库）
# ----------------------------------------------------------------------------
def read_wav(path):
    """返回 (采样率, 声道数, 交织的 float 样本 array('d')，范围约 -1..1)"""
    with open(path, 'rb') as f:
        data = f.read()
    if len(data) < 12 or data[0:4] != b'RIFF' or data[8:12] != b'WAVE':
        raise ConvertError('不是标准 RIFF/WAVE 文件')

    fmt = None
    pcm = None
    pos = 12
    while pos + 8 <= len(data):
        cid = data[pos:pos + 4]
        size = struct.unpack_from('<I', data, pos + 4)[0]
        body = data[pos + 8:pos + 8 + size]
        if cid == b'fmt ':
            fmt = body
        elif cid == b'data':
            pcm = body                       # 截断的文件：有多少用多少
        pos += 8 + size + (size & 1)         # 奇数长度的块后面有 1 个填充字节
    if fmt is None or len(fmt) < 16:
        raise ConvertError('缺少 fmt 块')
    if pcm is None:
        raise ConvertError('缺少 data 块')

    tag, channels, rate, _byte_rate, block_align, bits = struct.unpack_from('<HHIIHH', fmt, 0)
    if tag == WAVE_FORMAT_EXTENSIBLE:
        if len(fmt) < 40:
            raise ConvertError('EXTENSIBLE 格式的 fmt 块不完整')
        tag = struct.unpack_from('<H', fmt, 24)[0]       # SubFormat GUID 的前 2 字节就是格式码
    if channels < 1 or rate < 1:
        raise ConvertError('声道数/采样率非法')

    container = block_align // channels * 8 if block_align else bits
    nbytes = len(pcm) - len(pcm) % block_align
    pcm = pcm[:nbytes]

    if tag == WAVE_FORMAT_PCM:
        if container == 8:
            x = array('d', ((b - 128) / 128.0 for b in pcm))
        elif container == 16:
            a = array('h'); a.frombytes(pcm)
            if sys.byteorder == 'big': a.byteswap()
            x = array('d', (v / 32768.0 for v in a))
        elif container == 24:
            # 3 字节小端 → 4 字节（低位补 0）后按 32 位读，比逐个 int.from_bytes 快得多
            n = len(pcm) // 3
            b32 = bytearray(n * 4)
            b32[1::4] = pcm[0::3]; b32[2::4] = pcm[1::3]; b32[3::4] = pcm[2::3]
            a = array('i'); a.frombytes(bytes(b32))
            if sys.byteorder == 'big': a.byteswap()
            x = array('d', (v / 2147483648.0 for v in a))
        elif container == 32:
            a = array('i'); a.frombytes(pcm)
            if sys.byteorder == 'big': a.byteswap()
            x = array('d', (v / 2147483648.0 for v in a))
        else:
            raise ConvertError('不支持 %d 位整数 PCM' % container)
    elif tag == WAVE_FORMAT_IEEE_FLOAT:
        if container == 32:
            a = array('f')
        elif container == 64:
            a = array('d')
        else:
            raise ConvertError('不支持 %d 位浮点' % container)
        a.frombytes(pcm)
        if sys.byteorder == 'big': a.byteswap()
        x = array('d', a)
    else:
        raise ConvertError('压缩 WAV（格式码 0x%04X），需要 ffmpeg' % tag)
    return rate, channels, x


def to_mono(x, channels):
    if channels == 1:
        return x
    if channels == 2:
        return array('d', ((a + b) * 0.5 for a, b in zip(x[0::2], x[1::2])))
    k = 1.0 / channels
    return array('d', (sum(s) * k for s in zip(*[x[c::channels] for c in range(channels)])))


def resample(x, rate):
    """线性插值重采样到 48 kHz；降采样时先做 round(rate/48k) 点平均，压一下混叠"""
    if rate == TARGET_RATE or len(x) == 0:
        return x
    if rate > TARGET_RATE:
        k = max(1, round(rate / TARGET_RATE))
        if k > 1:
            acc = 0.0
            y = array('d', bytes(8 * len(x)))
            for i, v in enumerate(x):
                acc += v
                if i >= k:
                    acc -= x[i - k]
                y[i] = acc / k
            x = y
    n = len(x)
    out_n = n * TARGET_RATE // rate
    last = n - 1
    out = array('d', bytes(8 * out_n))
    for j in range(out_n):
        num = j * rate
        i = num // TARGET_RATE
        f = (num - i * TARGET_RATE) / TARGET_RATE
        a = x[i]
        out[j] = a + (x[i + 1] - a) * f if i < last else a
    return out


def float_to_s16(x, gain):
    out = array('h', bytes(2 * len(x)))
    clipped = 0
    for i, v in enumerate(x):
        s = round(v * gain * 32768.0)
        if s > 32767:
            s = 32767; clipped += 1
        elif s < -32768:
            s = -32768; clipped += 1
        out[i] = s
    return out, clipped


def convert_builtin(path, normalize):
    rate, channels, x = read_wav(path)
    x = to_mono(x, channels)
    x = resample(x, rate)
    peak = max((abs(v) for v in x), default=0.0)
    gain = (0.891 / peak) if (normalize and peak > 0) else 1.0      # -1 dBFS
    s16, clipped = float_to_s16(x, gain)
    return s16, clipped, '原文件 WAV %d Hz %d 声道，内置转换' % (rate, channels)


# ----------------------------------------------------------------------------
# ffmpeg 路径（任意格式，重采样质量更好）
# ----------------------------------------------------------------------------
def convert_ffmpeg(path, ffmpeg, normalize):
    cmd = [ffmpeg, '-v', 'error', '-nostdin', '-i', path,
           '-vn', '-f', 's16le', '-acodec', 'pcm_s16le', '-ar', str(TARGET_RATE), '-ac', '1', '-']
    r = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if r.returncode != 0:
        raise ConvertError('ffmpeg 失败：' + r.stderr.decode('utf-8', 'replace').strip())
    s16 = array('h'); s16.frombytes(r.stdout[:len(r.stdout) & ~1])
    if sys.byteorder == 'big': s16.byteswap()
    clipped = 0
    if normalize and len(s16):
        peak = max(max(s16), -min(s16))
        if peak > 0:
            g = 0.891 * 32768.0 / peak
            s16 = array('h', (max(-32768, min(32767, round(v * g))) for v in s16))
    return s16, clipped, '原文件由 ffmpeg 解码转换'


# ----------------------------------------------------------------------------
def write_pcm(s16, out_path):
    if sys.byteorder == 'big':
        s16 = array('h', s16); s16.byteswap()
    raw = s16.tobytes()
    pad = (-len(raw)) % SECTOR
    with open(out_path, 'wb') as f:
        f.write(raw)
        f.write(bytes(pad))                  # 补静音到整扇区
    return len(raw) + pad


def main():
    ap = argparse.ArgumentParser(
        description='把音乐转换成第10课 FPGA 播放用的 TRACKn.PCM（48kHz/16bit/小端/单声道/无文件头）')
    ap.add_argument('inputs', nargs='+', help='输入音乐文件（最多 4 个，按顺序对应 TRACK1..4）')
    ap.add_argument('-o', '--out', default='.', help='输出目录，可以直接写 TF 卡根目录（默认当前目录）')
    ap.add_argument('--first', type=int, default=1, choices=range(1, MAX_TRACKS + 1), metavar='N',
                    help='第一个输入对应 TRACKN（默认 1）')
    ap.add_argument('--normalize', action='store_true', help='把峰值音量拉到 -1 dBFS（原曲偏小声时用）')
    ap.add_argument('--ffmpeg', default=None, help='ffmpeg.exe 路径（不在 PATH 里时指定）')
    ap.add_argument('--no-ffmpeg', action='store_true', help='WAV 强制用内置转换')
    args = ap.parse_args()

    if args.first - 1 + len(args.inputs) > MAX_TRACKS:
        ap.error('最多 TRACK1~TRACK4：从 TRACK%d 开始只能再放 %d 个文件'
                 % (args.first, MAX_TRACKS - args.first + 1))
    if not os.path.isdir(args.out):
        ap.error('输出目录不存在：' + args.out)

    ffmpeg = None if args.no_ffmpeg else (args.ffmpeg or shutil.which('ffmpeg'))
    if args.ffmpeg and not os.path.isfile(args.ffmpeg) and not shutil.which(args.ffmpeg):
        ap.error('找不到 ffmpeg：' + args.ffmpeg)

    ok = 0
    for k, src in enumerate(args.inputs):
        name = 'TRACK%d.PCM' % (args.first + k)
        dst = os.path.join(args.out, name)
        try:
            if not os.path.isfile(src):
                raise ConvertError('文件不存在')
            is_wav = src.lower().endswith(('.wav', '.wave'))
            if ffmpeg:
                s16, clipped, how = convert_ffmpeg(src, ffmpeg, args.normalize)
            elif is_wav:
                s16, clipped, how = convert_builtin(src, args.normalize)
            else:
                raise ConvertError('不是 WAV，需要 ffmpeg（装好后放进 PATH，或用 --ffmpeg 指定路径）')
            if len(s16) == 0:
                raise ConvertError('没有音频数据')
            size = write_pcm(s16, dst)
            sec = len(s16) / TARGET_RATE
            peak = max(max(s16), -min(s16)) / 32768.0
            print('%s -> %s' % (src, dst))
            print('    输入：%s' % how)
            print('    输出：48000 Hz 单声道 16 位有符号小端，无文件头（FPGA 格式）')
            print('    时长 %d:%05.2f，%d 字节 = %d 扇区，峰值 %.0f%%%s'
                  % (sec // 60, sec % 60, size, size // SECTOR, peak * 100,
                     '，%d 个样本削波' % clipped if clipped else ''))
            if peak < 0.25 and not args.normalize:
                print('    提示：音量偏小，可以加 --normalize')
            ok += 1
        except ConvertError as e:
            print('%s：转换失败 —— %s' % (src, e), file=sys.stderr)

    if ok:
        print('\n拷卡：TF 卡格式化为 FAT32 → 先把 TRACKn.PCM 拷到根目录 → 再拷 BMP 图片 → 上板拨上 SW1 播放')
    return 0 if ok == len(args.inputs) else 1


if __name__ == '__main__':
    sys.exit(main())
