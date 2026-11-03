# tour_sequencer 微程序汇编器：生成 BRAM 初值（mem[32..] 常数、mem[256..] 微程序）（12 位指令 = op[11:8] src[7:5] a[4:0]；跳转目标 = [7:0]）
import re, sys

OPS = dict(LD=0, ADD=1, SUB=2, MAX=3, MIN=4, ST=5, MUL=6, DIV=7, BRN=8, BRNN=9, OUT=10, END=11)
SRC = dict(R=0, G=1, C=2, E=3, P16=4, P0=5)
RF = dict(D64=0, OX64=1, OY64=2, CMAX=3, RMAX=4, FX=5, FY=6, Z=7, W6=8, H6=9, TU=10, TV=11, TS=12,
          CU=13, CV=14, CS=15, UA=16, VA=17, SA=18, DU=19, DV=20, DS=21, FU=22, FV=23, T0=24, T1=25)
GEO = dict(C0=0, R0T=1, CW0=2, CH0=3, NN=4, DD=5, OX=10, OYT=11)
CONST = [0, 1, 4, 64, 20480, 30720, 32768, 40960, 49152, 65536]
EXT = dict(FTZ=0, FTX=1, FTY=2, E=3)
OUTS = dict(BU=0, BV=1, BS=2, FLAGS=7)

PROG = """
GEO:
  LD G.DD ; MUL C.64 ; LD P0 ; ST D64
  LD G.OX ; MUL C.64 ; LD P0 ; ST OX64
  LD G.OYT ; MUL C.64 ; LD P0 ; ST OY64
  LD G.C0 ; ADD G.CW0 ; SUB C.1 ; ST CMAX
  LD G.R0T ; ADD G.CH0 ; SUB C.1 ; ST RMAX
  LD C.0 ; ST CU ; ST CV ; ST UA ; ST VA ; ST DU ; ST DV ; ST DS
  LD C.65536 ; ST CS ; ST SA ; END
FRAME:
  LD R.DU ; MUL E.E ; LD P16 ; ADD R.UA ; ST CU ; OUT BU
  LD R.DV ; MUL E.E ; LD P16 ; ADD R.VA ; ST CV ; OUT BV
  LD R.DS ; MUL E.E ; LD P16 ; ADD R.SA ; ST CS ; OUT BS
FLAGS:
  OUT FLAGS ; END
CALC:
  LD E.FTX ; ST FX ; LD E.FTY ; ST FY ; LD E.FTZ ; ADD C.1 ; ST Z
  LD R.FX ; MAX G.C0 ; MIN R.CMAX ; SUB G.C0 ; MUL R.D64 ; DIV G.NN ; ADD R.OX64 ; ST FU
  LD R.FY ; MAX G.R0T ; MIN R.RMAX ; SUB G.R0T ; MUL R.D64 ; DIV G.NN ; ADD R.OY64 ; ST FV
  LD C.20480 ; MUL C.4 ; DIV R.Z ; ST W6
  LD C.32768 ; MUL C.4 ; DIV R.Z ; ST TS
  LD R.W6 ; MUL C.49152 ; LD P16 ; ST H6
  LD C.40960 ; SUB R.W6 ; ST T0
  LD R.W6 ; MUL C.32768 ; LD P16 ; ST T1
  LD R.FU ; SUB R.T1 ; MAX C.0 ; MIN R.T0 ; ST TU
  LD C.30720 ; SUB R.H6 ; ST T0
  LD R.H6 ; MUL C.32768 ; LD P16 ; ST T1
  LD R.FV ; SUB R.T1 ; MAX C.0 ; MIN R.T0 ; ST TV
fly:
  LD R.CU ; ST UA ; LD R.TU ; SUB R.CU ; ST DU
  LD R.CV ; ST VA ; LD R.TV ; SUB R.CV ; ST DV
  LD R.CS ; ST SA ; LD R.TS ; SUB R.CS ; ST DS
  END
HOME:
  LD C.0 ; ST TU ; ST TV ; LD C.65536 ; ST TS ; BRNN fly
"""

def operand(tok):
    if tok in ('P16', 'P0'):
        return SRC[tok], 0
    s, n = tok.split('.')
    if s == 'R': return 0, RF[n]
    if s == 'G': return 1, GEO[n]
    if s == 'C': return 2, CONST.index(int(n))
    if s == 'E': return 3, EXT[n]
    raise ValueError(tok)

lines = []
labels = {}
for raw in PROG.strip().splitlines():
    raw = raw.strip()
    if not raw:
        continue
    if raw.endswith(':'):
        labels[raw[:-1]] = len(lines)
        continue
    for ins in raw.split(';'):
        ins = ins.strip()
        if ins:
            lines.append(ins)
assert len(lines) <= 256
assert len(CONST) <= 32

words = []
for i, ins in enumerate(lines):
    p = ins.split()
    op = p[0]
    if op == 'END':
        w = OPS['END'] << 8
    elif op in ('BRN', 'BRNN'):
        w = (OPS[op] << 8) | labels[p[1]]
    elif op == 'ST':
        w = (OPS['ST'] << 8) | RF[p[1]]
    elif op == 'OUT':
        w = (OPS['OUT'] << 8) | OUTS[p[1]]
    else:
        s, a = operand(p[1])
        w = (OPS[op] << 8) | (s << 5) | a
    words.append((w, ins))

out = []
# 常数表和微程序放进引擎那块 BRAM 的空闲地址，initial 初值随比特流加载（TD 推断 BRAM 支持 INIT）
#   512×18：0..31 工作变量、32..63 常数、64..127 几何 {001, bank, idx}、256..511 微程序 {1, pc}
out.append('    // ---- 微程序 ROM（asm_seq.py 生成，%d 条）：入口 GEO=%d FRAME=%d FLAGS=%d CALC=%d HOME=%d ----' %
           (len(words), labels['GEO'], labels['FRAME'], labels['FLAGS'], labels['CALC'], labels['HOME']))
out.append('    // RAM 512×18（一块 BRAM9K）：0..31 工作变量、32..63 常数表、64..127 几何 {001, bank, idx}、256..511 微程序')
out.append('    (* ram_style = "bram" *) reg [17:0] mem [0:511];')
out.append('    initial begin')
for i, c in enumerate(CONST):
    out.append("        mem[%d] = 18'd%d;" % (32 + i, c))
for i, (w, ins) in enumerate(words):
    out.append("        mem[%d] = 18'h%05X;   // %s" % (256 + i, w, ins))
out.append('    end')
out.append("    localparam [7:0] PA_GEO = 8'd%d, PA_FRAME = 8'd%d, PA_FLAGS = 8'd%d, PA_CALC = 8'd%d, PA_HOME = 8'd%d;" %
           (labels['GEO'], labels['FRAME'], labels['FLAGS'], labels['CALC'], labels['HOME']))
txt = '\n'.join(out)

v = sys.argv[1]
s = open(v, encoding='utf-8').read()
a = s.index('    // @@UROM_BEGIN')
b = s.index('    // @@UROM_END')
s = s[:a] + '    // @@UROM_BEGIN\n' + txt + '\n' + s[b:]
open(v, 'w', encoding='utf-8', newline='').write(s)
print(len(words), labels)
