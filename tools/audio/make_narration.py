#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
    python make_narration.py 星云介绍朗读.wav --track 1
        第一次运行：
          1. 录音转成 TRACK1.VOC（24 kHz/16 位/单声道，同 make_voice.py）
          2. 语音识别（faster-whisper）得到文字和每个字的时间
          3. 自动断句（每行默认不超过 18 个汉字，标点原样保留），写出 TRACK1.LRC
             板上字幕是一条连续滚动的跑马灯、全文页是一整段，行只决定卡拉OK 的时间节点，断得细一点高亮更跟嘴
          4. 同时写出字幕稿「星云介绍朗读_字幕稿.txt」，并用 check_lrc.py 的规则检查
        识别有错字 / 想改断行 / 想加标题：用记事本改字幕稿（一行就是一句字幕），然后原样再运行一次同一条命令，
          脚本会用改好的文字重新对齐时间（识别结果有缓存，第二次几秒就完成）。
    python make_narration.py 星云介绍朗读.wav --track 1 --title 星云 --intro "深空天体  第1展区"
        顺便写上左上角标题和导语（也可以直接写在字幕稿的 [ti:] [ar:] 行里）

输出默认放在录音所在的文件夹（-o 可改，可以直接写 TF 卡根目录）。
需要：pip install faster-whisper；第一次运行会下载识别模型（large-v3-turbo 约 1.6 GB，默认走 hf-mirror.com 镜像），
      之后离线可用。电脑慢可以 --model small（约 500 MB，错字多一些）。
音频：WAV 直接读；mp3 / m4a 等用 ffmpeg（在 PATH 里时）或 faster-whisper 自带的 PyAV 解码。
"""
import argparse
import difflib
import json
import os
import re
import shutil
import subprocess
import sys
from array import array

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import make_audio_corpus as mac  # noqa: E402

VOICE_RATE = 24000
RECOG_PROMPT = '以下是普通话的句子，使用简体中文。'
PUNCT = '，、；：。！？…—·“”‘’（）《》「」『』,.;:!?()"\' '
SPLIT_AFTER = '，、；：。！？…,;:!?'
SENT_END = '。！？；…!?;'
TO_FULL = {',': '，', '?': '？', '!': '！', ';': '；', ':': '：', '﹑': '、', '﹐': '，'}


# ----------------------------------------------------------------------------
# 文字处理
# ----------------------------------------------------------------------------
def is_cjk(c):
    return c >= '⺀'


def width(s):
    """GB2312 半宽数（汉字 / 全角符号 2，ASCII 1）"""
    return sum(2 if ord(c) > 0x7f else 1 for c in s)


def normalize(s):
    """识别结果 / 字幕稿 → 适合上板的写法：中文旁的半角标点换全角、去掉中文旁的空格"""
    s = s.replace('　', ' ').strip()
    out = []
    for i, c in enumerate(s):
        if c in TO_FULL:
            prev = s[i - 1] if i else ''
            nxt = s[i + 1] if i + 1 < len(s) else ''
            if c in '﹑﹐' or is_cjk(prev) or is_cjk(nxt) or not nxt.strip():
                c = TO_FULL[c]
        elif c == '.' and i and is_cjk(s[i - 1]):
            c = '。'
        out.append(c)
    s = ''.join(out)
    s = re.sub(r'\s+(?=[⺀-￿])|(?<=[⺀-￿])\s+', '', s)
    return re.sub(r'\s{2,}', ' ', s)


def tokens(s):
    """切成不可拆的小块：连续的英文字母 / 数字算一块，其它每个字一块"""
    return re.findall(r'[A-Za-z0-9%.\-+]+|.', s)


BREAK_AFTER = '的了着过地得'                    # 没有标点的长句优先在这些字后面断
BREAK_BEFORE = '和与及并或而但却就也都把被让从向对为在是'   # ……或者这些字前面断


def chunk(s, maxw):
    """一个没有标点可断的长句切成几段：每段不超过 maxw、尽量等长、尽量在虚词处断，不拆英文单词"""
    tk = tokens(s)
    w = [width(t) for t in tk]
    total = sum(w)
    if total <= maxw:
        return [s]
    n = -(-total // maxw)
    best, left = None, 0
    for i in range(1, len(tk)):
        left += w[i - 1]
        if left > maxw:
            break
        if total - left > (n - 1) * maxw:
            continue
        score = abs(left - total / n)
        if tk[i - 1] in BREAK_AFTER:
            score -= 6
        elif tk[i] in BREAK_BEFORE:
            score -= 4
        if best is None or score < best[0]:
            best = (score, i)
    i = best[1]
    return [''.join(tk[:i])] + chunk(''.join(tk[i:]), maxw)


def pack(s, maxw):
    """一句话 → 若干行字幕：优先在标点后断，每行不超过 maxw 半宽"""
    units = re.findall('[^%s]+[%s]*|[%s]+' % ((re.escape(SPLIT_AFTER),) * 3), s)
    lines, cur = [], ''
    for u in units:
        if width(cur + u) <= maxw:
            cur += u
            continue
        if cur:
            lines.append(cur); cur = ''
        if width(u) <= maxw:
            cur = u
        else:
            parts = chunk(u, maxw)
            lines += parts[:-1]
            cur = parts[-1]
    if cur:
        lines.append(cur)
    return [l for l in lines if l.strip(PUNCT)]


def sentences(text):
    return [m.strip() for m in re.findall('[^%s]+[%s]*' % ((re.escape(SENT_END),) * 2), text) if m.strip()]


def key(c):
    return '' if c in PUNCT or c.isspace() else c.lower()


# ----------------------------------------------------------------------------
# 音频
# ----------------------------------------------------------------------------
def convert_voice(src, normalize_vol, ffmpeg):
    mac.TARGET_RATE = VOICE_RATE
    if ffmpeg:
        return mac.convert_ffmpeg(src, ffmpeg, normalize_vol)
    if src.lower().endswith(('.wav', '.wave')):
        return mac.convert_builtin(src, normalize_vol)
    try:
        import av
    except ImportError:
        raise mac.ConvertError('不是 WAV，需要 ffmpeg 或 PyAV（pip install faster-whisper 会顺带装上 PyAV）')
    s16 = array('h')
    with av.open(src) as box:
        rs = av.AudioResampler(format='s16', layout='mono', rate=VOICE_RATE)
        for frame in box.decode(audio=0):
            for f in rs.resample(frame):
                s16.frombytes(bytes(f.planes[0])[:f.samples * 2])
        for f in rs.resample(None):
            s16.frombytes(bytes(f.planes[0])[:f.samples * 2])
    if sys.byteorder == 'big':
        s16.byteswap()
    clipped = 0
    if normalize_vol and len(s16):
        peak = max(max(s16), -min(s16))
        if peak:
            g = 0.891 * 32768 / peak
            s16 = array('h', (max(-32768, min(32767, round(v * g))) for v in s16))
    return s16, clipped, '%s，PyAV 解码' % os.path.splitext(src)[1]


# ----------------------------------------------------------------------------
# 语音识别（结果缓存成 json，改字幕稿后重跑不用再识别）
# ----------------------------------------------------------------------------
def recognize(src, model_name, cache):
    st = os.stat(src)
    tag = {'size': st.st_size, 'mtime': int(st.st_mtime), 'model': model_name, 'prompt': RECOG_PROMPT}
    if os.path.isfile(cache):
        try:
            d = json.load(open(cache, encoding='utf-8'))
            if d.get('tag') == tag:
                print('语音识别：用缓存 %s' % os.path.basename(cache))
                return d['text'], [tuple(c) for c in d['chars']]
        except (ValueError, KeyError):
            pass
    if os.path.isdir(os.path.join(sys.prefix, 'huggingface')):   # 模型放在 Python 安装目录下（move_whisper_to_D.bat 搬过去的）
        os.environ.setdefault('HF_HOME', os.path.join(sys.prefix, 'huggingface'))
    os.environ.setdefault('HF_ENDPOINT', 'https://hf-mirror.com')
    os.environ.setdefault('HF_HUB_DISABLE_SYMLINKS_WARNING', '1')
    try:
        from faster_whisper import WhisperModel
    except ImportError:
        raise SystemExit('缺少语音识别库：请先运行  pip install faster-whisper')
    print('语音识别：加载模型 %s（第一次会自动下载，large-v3-turbo 约 1.6 GB，请耐心等）……' % model_name)
    model = WhisperModel(model_name, device='cpu', compute_type='int8')
    print('语音识别：识别中……')
    segs, info = model.transcribe(src, language='zh', word_timestamps=True, vad_filter=False,
                                  initial_prompt=RECOG_PROMPT)
    text, chars = '', []
    for seg in segs:
        text += seg.text
        for w in seg.words:
            ks = [c for c in w.word if key(c)]
            for k, c in enumerate(ks):
                a = w.start + (w.end - w.start) * k / len(ks)
                b = w.start + (w.end - w.start) * (k + 1) / len(ks)
                chars.append((c.lower(), round(a, 3), round(b, 3)))
        print('  [%6.2f] %s' % (seg.start, seg.text.strip()))
    json.dump({'tag': tag, 'text': text, 'chars': chars}, open(cache, 'w', encoding='utf-8'), ensure_ascii=False)
    return text, chars


def align(lines, chars):
    """字幕行（可能被人改过字）对齐到识别出的逐字时间：返回每行 (开始时间, 结束时间)"""
    user, owner = [], []
    for n, l in enumerate(lines):
        for c in l:
            if key(c):
                user.append(key(c)); owner.append(n)
    asr = [c[0] for c in chars]
    t0 = [None] * len(user)
    t1 = [None] * len(user)
    for a, b, size in difflib.SequenceMatcher(None, user, asr, autojunk=False).get_matching_blocks():
        for k in range(size):
            t0[a + k] = chars[b + k][1]
            t1[a + k] = chars[b + k][2]
    known = [i for i in range(len(user)) if t0[i] is not None]
    if not known:
        raise SystemExit('字幕稿和录音完全对不上（是不是用错了字幕稿？可加 --redo 重新生成）')
    for arr in (t0, t1):                            # 没对上的字（改过的字）按位置插值
        for i in range(len(user)):
            if arr[i] is None:
                lo = max((k for k in known if k < i), default=None)
                hi = min((k for k in known if k > i), default=None)
                if lo is None:
                    arr[i] = arr[hi]
                elif hi is None:
                    arr[i] = arr[lo]
                else:
                    arr[i] = arr[lo] + (arr[hi] - arr[lo]) * (i - lo) / (hi - lo)
    span = {}
    for i, n in enumerate(owner):
        s, e = span.get(n, (t0[i], t1[i]))
        span[n] = (min(s, t0[i]), max(e, t1[i]))
    return [span.get(n) for n in range(len(lines))]


# ----------------------------------------------------------------------------
# 字幕稿
# ----------------------------------------------------------------------------
DRAFT_HEAD = """\
# 讲解字幕稿（由 make_narration.py 生成）。用记事本修改后保存，再运行一次同一条命令即可更新 LRC。
# - 一行就是一个卡拉OK 时间节点（板上所有行连成一条滚动字幕 / 一整段全文）；可以改错字、合并 / 拆分行，
#   标点照常写。超过 %d 个汉字的行会被自动再拆开。
# - [ti:] 后面写左上角大标题，[ar:] 后面写标题下的小字导语，不要可以空着。
# - 以 # 开头的行是说明，会被忽略。想重新识别生成本稿：删掉本文件或加 --redo。
"""


def read_draft(path):
    raw = open(path, 'rb').read()
    if raw.startswith(b'\xef\xbb\xbf'):
        raw = raw[3:]
    for enc in ('utf-8', 'gbk'):
        try:
            text = raw.decode(enc)
            break
        except UnicodeDecodeError:
            pass
    else:
        raise SystemExit('字幕稿编码无法识别，请用记事本另存为 UTF-8')
    title = intro = None
    lines = []
    for s in text.splitlines():
        s = s.strip()
        if not s or s.startswith('#'):
            continue
        low = s.lower()
        if low.startswith('[ti:'):
            title = s[4:].rstrip(']').strip()
        elif low.startswith('[ar:'):
            intro = s[4:].rstrip(']').strip()
        else:
            lines.append(normalize(s))
    return title or '', intro or '', lines


def write_draft(path, title, intro, lines, maxw):
    with open(path, 'w', encoding='utf-8', newline='\r\n') as f:
        f.write(DRAFT_HEAD % (maxw // 2))
        f.write('[ti:%s]\n[ar:%s]\n' % (title, intro))
        for l in lines:
            f.write(l + '\n')


def lrc_time(t):
    cs = max(0, int(round(t * 100)))
    return '[%02d:%02d.%02d]' % (cs // 6000, cs // 100 % 60, cs % 100)


# ----------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description='讲解朗读录音 → TRACKn.VOC + TRACKn.LRC（自动识别字幕和时间轴）')
    ap.add_argument('audio', help='讲解朗读录音（wav / mp3 / m4a ...）')
    ap.add_argument('--track', type=int, default=1, choices=range(1, 5), metavar='N',
                    help='生成 TRACKN.VOC / TRACKN.LRC，对应 PICN.BMP（默认 1）')
    ap.add_argument('-o', '--out', help='输出文件夹（默认录音所在文件夹，也可以直接写 TF 卡根目录）')
    ap.add_argument('--title', help='左上角大标题（写进字幕稿 [ti:]）')
    ap.add_argument('--intro', help='标题下的小字导语（写进字幕稿 [ar:]）')
    ap.add_argument('--max', type=int, default=18, metavar='字数',
                    help='每行最多几个汉字（默认 18；最大 36）。行越短，卡拉OK 高亮和声音对得越准')
    ap.add_argument('--lead', type=float, default=0.15, metavar='秒', help='字幕比声音提前出现的时间（默认 0.15）')
    ap.add_argument('--model', default='large-v3-turbo',
                    help='识别模型：large-v3-turbo（默认，准）/ small（快、小、错字多）/ medium ...')
    ap.add_argument('--redo', action='store_true', help='忽略已有字幕稿，按识别结果重新生成')
    ap.add_argument('--no-normalize', action='store_true', help='不把人声音量拉到 -1 dBFS')
    ap.add_argument('--ffmpeg', default=None, help='ffmpeg.exe 路径（可选）')
    a = ap.parse_args()
    if not os.path.isfile(a.audio):
        ap.error('录音文件不存在：' + a.audio)
    if not 1 <= a.max <= 36:
        ap.error('--max 只能是 1~36')
    maxw = a.max * 2
    out = a.out or os.path.dirname(os.path.abspath(a.audio))
    if not os.path.isdir(out):
        ap.error('输出文件夹不存在：' + out)
    stem = os.path.splitext(os.path.abspath(a.audio))[0]
    draft = stem + '_字幕稿.txt'
    voc = os.path.join(out, 'TRACK%d.VOC' % a.track)
    lrc = os.path.join(out, 'TRACK%d.LRC' % a.track)

    # 1. 人声 VOC
    try:
        s16, clipped, how = convert_voice(a.audio, not a.no_normalize, a.ffmpeg or shutil.which('ffmpeg'))
    except mac.ConvertError as e:
        raise SystemExit('录音转换失败：%s' % e)
    if not len(s16):
        raise SystemExit('录音里没有音频数据')
    size = mac.write_pcm(s16, voc)
    dur = size // 2 / VOICE_RATE
    print('人声：%s -> %s（%s；时长 %.2f s，%d 扇区%s）'
          % (os.path.basename(a.audio), voc, how, dur, size // 512, '，%d 个样本削波' % clipped if clipped else ''))

    # 2. 识别（或读缓存）
    asr_text, chars = recognize(a.audio, a.model, stem + '_识别缓存.json')
    if not chars:
        raise SystemExit('没有识别出任何文字（录音是不是没有人声？）')

    # 3. 字幕稿：已有就用（人可能改过），没有就按识别结果生成
    if os.path.isfile(draft) and not a.redo:
        title, intro, raw_lines = read_draft(draft)
        print('字幕稿：读取 %s（共 %d 行）' % (os.path.basename(draft), len(raw_lines)))
    else:
        title = intro = ''
        raw_lines = [l for s in sentences(normalize(asr_text)) for l in pack(s, maxw)]
        print('字幕稿：按识别结果新建 %s' % os.path.basename(draft))
    if a.title is not None:
        title = normalize(a.title)
    if a.intro is not None:
        intro = normalize(a.intro)
    lines = [l for r in raw_lines for l in pack(r, maxw)]
    if not lines:
        raise SystemExit('字幕稿里没有字幕行')
    write_draft(draft, title, intro, lines, maxw)

    # 4. 对齐时间，写 LRC
    spans = align(lines, chars)
    body, last = [], -1.0
    for l, (t, _e) in zip(lines, spans):
        t = max(t - a.lead, last + 0.1, 0.0)
        body.append(lrc_time(t) + l)
        last = t
    end = max(e for _s, e in spans) + 1.0           # 讲完 1 秒后清屏（录音末尾还有空白时）
    if end < dur - 0.2 and end > last + 0.1:
        body.append(lrc_time(end))
    head = []
    if title:
        head.append('[ti:%s]' % title)
    if intro:
        head.append('[ar:%s]' % intro)
    text = '\r\n'.join(head + body) + '\r\n'
    try:
        data = text.encode('gb2312')
    except UnicodeEncodeError as e:
        bad = text[e.start]
        raise SystemExit('字符 "%s"（U+%04X）不在 GB2312 字库里，FPGA 显示不了；请在字幕稿 %s 里把它换成常用字或全角符号后再运行'
                         % (bad, ord(bad), draft))
    open(lrc, 'wb').write(data)
    print('字幕：%s（%d 句，GB2312 + CRLF）' % (lrc, len(lines)))

    # 5. 用和 FPGA 相同的规则检查
    print('\n—— check_lrc 检查 ——', flush=True)
    r = subprocess.run([sys.executable, os.path.join(HERE, 'check_lrc.py'), lrc, '--voc', voc])
    print('\n识别结果可能有错字：请打开 %s 核对，改完再运行一次同一条命令。' % draft)
    print('拷卡：TRACK%d.VOC 和 TRACK%d.LRC 放 TF 卡根目录，对应 PIC%d.BMP（卡上还要有 HZK32 字库；卡要先格式化成 FAT32，见改动报告第 3.4 节）。'
          % (a.track, a.track, a.track))
    return r.returncode


if __name__ == '__main__':
    sys.exit(main())
