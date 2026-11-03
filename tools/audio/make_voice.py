#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
make_voice.py —— 讲解字幕：把讲解人声录音转换成 FPGA 直接播放的 TRACKn.VOC

FPGA（第二个 wav_stream_player）只认一种裸数据：
    24000 Hz、16 位有符号、小端、单声道、无文件头（比背景乐 TRACKn.PCM 的 48 kHz 省一半带宽 / 空间，人声足够）
解码 / 重采样 / 归一化复用同目录的 make_audio_corpus.py，只把目标采样率换成 24 kHz。

用法（Windows 命令行，在本文件所在目录）：
    python make_voice.py 讲解1.mp3 讲解2.mp3             -> 当前目录生成 TRACK1.VOC、TRACK2.VOC
    python make_voice.py 讲解3.wav --first 3 -o E:\\      -> 直接写到 TF 卡根目录的 TRACK3.VOC
    python make_voice.py 讲解1.mp3 --normalize            -> 录音偏小声时拉到 -1 dBFS（推荐）
  mp3 / m4a 等需要 ffmpeg（放进 PATH 或 --ffmpeg 指定 ffmpeg.exe）；没有 ffmpeg 时先用任意软件另存为 WAV 再转。

讲解时间轴：TRACKn.LRC 里的 [mm:ss.xx] 以 TRACKn.VOC 的第一个样本为 0 点（FPGA 按实际播出的人声样本数计时，
  暂停 / 卡顿时字幕跟着停，音字不会错开）。用任意音频软件打开原录音即可读出每句的开始时间。
人声只混在 WAV 音源里：SW1 拨上（播放 TRACKn.PCM）才听得到；只想要人声时放一个静音的 TRACKn.PCM 即可。
"""
import argparse
import os
import shutil
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import make_audio_corpus as mac  # noqa: E402

VOICE_RATE = 24000


def main():
    mac.TARGET_RATE = VOICE_RATE                    # make_audio_corpus 的解码 / 重采样 / ffmpeg 参数都读这个全局
    ap = argparse.ArgumentParser(
        description='把讲解录音转换成扩1 FPGA 播放用的 TRACKn.VOC（24kHz/16bit/小端/单声道/无文件头）')
    ap.add_argument('inputs', nargs='+', help='输入录音（最多 4 个，按顺序对应 TRACK1..4）')
    ap.add_argument('-o', '--out', default='.', help='输出目录，可以直接写 TF 卡根目录（默认当前目录）')
    ap.add_argument('--first', type=int, default=1, choices=range(1, 5), metavar='N', help='第一个输入对应 TRACKN（默认 1）')
    ap.add_argument('--normalize', action='store_true', help='把峰值音量拉到 -1 dBFS（推荐，人声和背景乐响度更匹配）')
    ap.add_argument('--ffmpeg', default=None, help='ffmpeg.exe 路径（不在 PATH 里时指定）')
    ap.add_argument('--no-ffmpeg', action='store_true', help='WAV 强制用内置转换')
    a = ap.parse_args()
    if a.first - 1 + len(a.inputs) > 4:
        ap.error('最多 TRACK1~TRACK4')
    if not os.path.isdir(a.out):
        ap.error('输出目录不存在：' + a.out)
    ffmpeg = None if a.no_ffmpeg else (a.ffmpeg or shutil.which('ffmpeg'))

    ok = 0
    for k, src in enumerate(a.inputs):
        dst = os.path.join(a.out, 'TRACK%d.VOC' % (a.first + k))
        try:
            if not os.path.isfile(src):
                raise mac.ConvertError('文件不存在')
            if ffmpeg:
                s16, clipped, how = mac.convert_ffmpeg(src, ffmpeg, a.normalize)
            elif src.lower().endswith(('.wav', '.wave')):
                s16, clipped, how = mac.convert_builtin(src, a.normalize)
            else:
                raise mac.ConvertError('不是 WAV，需要 ffmpeg（装好后放进 PATH，或用 --ffmpeg 指定路径；'
                                       '也可以先用任意音频软件另存为 WAV）')
            if len(s16) == 0:
                raise mac.ConvertError('没有音频数据')
            size = mac.write_pcm(s16, dst)          # 末尾补静音到整扇区（FPGA 丢弃不满一扇区的尾巴）
            sec = len(s16) / VOICE_RATE
            peak = max(max(s16), -min(s16)) / 32768.0
            print('%s -> %s' % (src, dst))
            print('    输入：%s' % how)
            print('    输出：24000 Hz 单声道 16 位有符号小端，无文件头')
            print('    时长 %d:%05.2f（LRC 最后一句的时间应小于它），%d 字节 = %d 扇区，峰值 %.0f%%%s'
                  % (sec // 60, sec % 60, size, size // 512, peak * 100,
                     '，%d 个样本削波' % clipped if clipped else ''))
            if peak < 0.25 and not a.normalize:
                print('    提示：音量偏小，建议加 --normalize')
            ok += 1
        except mac.ConvertError as e:
            print('%s：转换失败 —— %s' % (src, e), file=sys.stderr)
    if ok:
        print('\n拷卡：FAT32 卡根目录，刚格式化的空卡只往里拷（顺序无所谓，不要在卡上删改，否则文件可能不连续）；'
              '\n      上板拨上 SW1；SW2 暂停讲解、SW3 全文页、KEY3 长按开关人声')
    return 0 if ok == len(a.inputs) else 1


if __name__ == '__main__':
    sys.exit(main())
