# goertzel_spectrum_bars 逐位参考模型 + ROM 初值生成
# 改频点 / 窗长 / 倾斜补偿后重新运行：python gsm.py ../../src/user_source/hdl_source/display/goertzel_spectrum_bars.v
import math, numpy as np
FS=48000.0; NB=32; SH=12
def _bins():
    f=[50.0]; r=1.1774
    for k in range(NB-1): f.append(max(f[-1]*r, f[-1]+24.0))
    out=[]
    for k,fk in enumerate(f):
        sp=(f[k+1]-fk) if k<NB-1 else fk*(r-1)
        out.append((fk,max(4,min(11,int(round(math.log2(FS/sp)))))))
    return out
BINS=_bins()
TILT=[10**(max(0,k-12)*0.8/20) for k in range(NB)]   # 高频倾斜补偿：第 12 档起每档 +0.8 dB
WIN=[int(round((0.5-0.5*math.cos(2*math.pi*(i+0.5)/256))*65535)) for i in range(256)]
EPS=[int(round(2*math.sin(math.pi*fk/FS)*65536)) for fk,m in BINS]
G=[int(round((1<<(17-m))*2.0*TILT[k])) for k,(fk,m) in enumerate(BINS)]
MASK=[(1<<m)-1 for fk,m in BINS]
SHF=[11-m for fk,m in BINS]
META=[(SHF[k]<<11)|MASK[k] for k in range(NB)]
def w18(v):
    v&=0x3ffff; return v-0x40000 if v&0x20000 else v
def sat17(v): return 0 if v<0 else min(v,0x1ffff)
def Lcode(p):
    if p<=0: return 0
    e=p.bit_length()-1; nrm=(p<<(16-e))&0x1ffff
    return 4*e+((nrm>>14)&3)
class Eng:
    def __init__(s):
        s.v=[0]*NB; s.S=[0]*NB; s.n=0
    def sample(s,x):
        """一次被接收的样本；返回 [(bin, pw)] 本样本结束窗口的功率"""
        n11=s.n&2047; ends=[]
        for k in range(NB):
            j8=((n11<<SHF[k])&2047)>>3
            xw=(x*WIN[j8])>>16
            xs=(xw*G[k])>>16
            u=w18(s.v[k]+xs)
            vn=w18(u+((-EPS[k]*s.S[k])>>16))
            Sn=w18(s.S[k]+((EPS[k]*vn)>>16))
            if (n11|(~MASK[k]&2047))==2047:
                q=sat17((vn*vn)>>SH)
                ends.append((k,sat17(q+((Sn*s.S[k])>>SH))))
                s.v[k]=0; s.S[k]=0
            else:
                s.v[k]=vn; s.S[k]=Sn
        s.n=(s.n+1)&2047
        return ends
NL = chr(10)
def rom_lines():
    L=["    initial begin"]
    for k in range(NB):
        a=k*8
        L.append("        mem[%3d] = 18'h00000;  mem[%3d] = 18'h00000;  mem[%3d] = 18'h00000;  mem[%3d] = 18'h00000;"%(a,a+1,a+2,a+3))
        L.append("        mem[%3d] = 18'h%05X;  mem[%3d] = 18'h%05X;  mem[%3d] = 18'h%05X;  mem[%3d] = 18'h%05X;   // %2d: %7.1f Hz  N=%4d"%(
            a+4,META[k],a+5,G[k],a+6,(-EPS[k])&0x3ffff,a+7,EPS[k],k,BINS[k][0],1<<BINS[k][1]))
    for i in range(0,256,4):
        L.append("        "+"  ".join("mem[%3d] = 18'h%05X;"%(256+i+j,WIN[i+j]) for j in range(4)))
    L.append("    end")
    return NL.join(L)

def patch_rtl(path):
    """把初值写进 RTL 的 @@ROM_BEGIN / @@ROM_END 之间"""
    s=open(path,encoding='utf-8').read()
    a=s.index("// @@ROM_BEGIN"); a=s.index(NL,a)+1
    b=s.index("    // @@ROM_END")
    s=s[:a]+rom_lines()+NL+s[b:]
    open(path,'w',encoding='utf-8',newline=NL).write(s)

if __name__=="__main__":
    import sys
    # 用法：python gsm.py <goertzel_spectrum_bars.v>
    patch_rtl(sys.argv[1])
