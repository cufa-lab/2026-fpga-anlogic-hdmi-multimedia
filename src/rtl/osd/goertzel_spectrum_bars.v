`timescale 1ns/1ps

module goertzel_spectrum_bars #(
    parameter integer BAR_X = 64,          // 带左端 x（16 的倍数，≥32：预读从左边两档开始），宽 512
    parameter integer BAR_Y = 368,         // 带顶端 y，高 64
    parameter integer FALL  = 2,           // 柱身每帧最多下落像素
    parameter integer HOLD  = 18,          // 峰值帽停留帧数（≤31）
    parameter integer QUIET = 60           // 连续无声多少帧后收拢（≤63）
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        frame_start,
    input  wire        de,
    input  wire [23:0] rgb_in,
    input  wire [9:0]  pixel_x,
    input  wire [9:0]  pixel_y,
    input  wire        enable,
    input  wire [4:0]  hue_shift,         // 色相流动（顶层帧计数位段，帧边界变化）
    input  wire [8:0]  bright_gain,       // 画面亮度 + 1（1..256，帧内不变）；256 = 不缩放
    input  wire        pcm_tap_valid,     // 每个样本一拍（握手拍）
    input  wire signed [15:0] pcm_tap_left,
    output wire [23:0] rgb_out
);
    localparam [5:0] SLOT0 = BAR_X / 16;
    localparam [9:0] Y0    = BAR_Y;
    localparam [4:0] HOLD5 = HOLD;
    localparam [5:0] QUIET6 = QUIET;

    // ============================ 共享 BRAM ============================
    reg  [17:0] mem [0:511];
    reg  [17:0] q;
    wire        re;
    wire [8:0]  ra;
    reg         we;
    reg  [8:0]  wa;
    reg  [17:0] wd;
    always @(posedge clk)
        if (we)
            mem[wa] <= wd;
    always @(posedge clk)
        if (re)
            q <= mem[ra];

    // ============================ 显示侧（读口优先）============================
    // 64 根柱：每档 16 像素画两根（8 像素一根，相位 1..6 画柱、0/7 为缝），左根 = 第 k 档，右根 = 第 k、k+1 档平均
    //   每档只读一次 D：ph=14 预读第 k+2 档（rb = bx+2），ph=15 移位 → 本档全程 dd0 = 第 k 档、dd1 = 第 k+1 档；
    //   预读从 bx=-2 开始，第 32 档不存在就不更新 dd1（末档右根 = 末档自身）。读次数与 32 柱时相同
    wire [9:0] dy      = pixel_y - Y0;
    wire       in_rows = (dy[9:6] == 4'd0);
    wire [5:0] r       = ~dy[5:0];                 // 距带底行数 0..63
    wire [5:0] bx      = pixel_x[9:4] - SLOT0;     // 档号
    wire [5:0] rb      = pixel_x[9:4] - (SLOT0 - 6'd2);
    wire [3:0] ph      = pixel_x[3:0];
    wire       zone    = enable && de && in_rows && !bx[5];
    wire       disp_rd = enable && de && in_rows && !rb[5] && (ph == 4'd14);
    reg        disp_rd_q;
    reg [11:0] dd0, dd1;                             // {cap, h}
    always @(posedge clk) begin
        disp_rd_q <= disp_rd;
        if (de && ph == 4'd15) begin
            dd0 <= dd1;
            if (disp_rd_q)
                dd1 <= q[11:0];
        end
    end

    reg  [4:0] wopen;                              // 展开半宽（档数，0..16）
    wire [3:0] dist   = bx[4] ? bx[3:0] : ~bx[3:0];
    wire       col_on = (ph[2:0] != 3'd0) && (ph[2:0] != 3'd7) && ({1'b0, dist} < wopen);
    wire [6:0] hsum   = {1'b0, dd0[5:0]}  + {1'b0, dd1[5:0]};
    wire [6:0] csum   = {1'b0, dd0[11:6]} + {1'b0, dd1[11:6]};
    wire [5:0] bh     = ph[3] ? hsum[6:1] : dd0[5:0];
    wire [5:0] bc     = ph[3] ? csum[6:1] : dd0[11:6];
    // 一个减法器同时给出柱身和柱顶：t = bh − r − 1，t ≥ 0 即 r < bh（柱身），t ∈ {0,1} 即柱顶 2 行
    wire [6:0] t      = {1'b0, bh} + {1'b1, ~r};
    wire       body   = zone && col_on && !t[6];
    wire       tip    = (t[5:1] == 5'd0);
    wire       capp   = zone && col_on && (bc[5:1] != 5'd0) && (r[5:1] == bc[5:1]);

    // "深空"循环色表 32 项（tools/spectrum/palette.py，已预乘 75% 不透明度）：
    //   深蓝 → 天蓝 → 青(OIII) → 金 → 橙 → 玫红(Hα) → 品红 → 紫 → 深蓝
    //   色相 = 档号 − 流动 + 高度/8：横向一档一色（整条带正好一圈色环）、整体向右流动（hue_shift 复用顶层帧计数）、
    //   越高颜色越往后走（振幅越大颜色越丰富）
    wire [4:0] hue = bx[4:0] - hue_shift + {2'b00, r[5:3]};
    reg  [23:0] pal;
    always @* begin
        case (hue)
            // @@PAL_BEGIN
            5'd0 : pal = 24'h162dbf;
            5'd1 : pal = 24'h1041bf;
            5'd2 : pal = 24'h0b56bf;
            5'd3 : pal = 24'h056abf;
            5'd4 : pal = 24'h007fbf;
            5'd5 : pal = 24'h008ab6;
            5'd6 : pal = 24'h0096ae;
            5'd7 : pal = 24'h00a1a5;
            5'd8 : pal = 24'h00ac9d;
            5'd9 : pal = 24'h2fa984;
            5'd10: pal = 24'h5fa66c;
            5'd11: pal = 24'h8fa354;
            5'd12: pal = 24'hbfa13c;
            5'd13: pal = 24'hbf9332;
            5'd14: pal = 24'hbf8429;
            5'd15: pal = 24'hbf761f;
            5'd16: pal = 24'hbf6916;
            5'd17: pal = 24'hbf5a21;
            5'd18: pal = 24'hbf4b2d;
            5'd19: pal = 24'hbf3c38;
            5'd20: pal = 24'hbf2d43;
            5'd21: pal = 24'hba2957;
            5'd22: pal = 24'hb5256c;
            5'd23: pal = 24'hb12181;
            5'd24: pal = 24'hac1e96;
            5'd25: pal = 24'h9b219f;
            5'd26: pal = 24'h8a25aa;
            5'd27: pal = 24'h7929b4;
            5'd28: pal = 24'h692dbf;
            5'd29: pal = 24'h542dbf;
            5'd30: pal = 24'h3f2dbf;
            default:pal = 24'h2a2dbf;
            // @@PAL_END
        endcase
    end
    // 柱身：bg/4 + 0.75c（每通道一次加法，≤ 63 + 191 不溢出）；柱顶辉光 0x80 + c/2；峰值帽 0xC0 + c/4（带色调的白），均为位拼接
    // 亮度同步：先按柱身 / 柱顶 / 峰值帽选出前景色，再 × bright_gain/256（3 个 8×9 乘法，DSP）。
    //   bg 已经在上游按亮度压暗，只缩前景色；bright_gain = 256 时乘积高 8 位 = 原色，逐位同改前
    wire [23:0] col  = capp ? {2'b11, pal[23:18], 2'b11, pal[15:10], 2'b11, pal[7:2]} :
                       tip  ? {1'b1, pal[23:17], 1'b1, pal[15:9], 1'b1, pal[7:1]} : pal;
    wire [16:0] sc_r = col[23:16] * bright_gain;
    wire [16:0] sc_g = col[15:8]  * bright_gain;
    wire [16:0] sc_b = col[7:0]   * bright_gain;
    wire [23:0] sc   = {sc_r[15:8], sc_g[15:8], sc_b[15:8]};
    wire [7:0] mix_r = {2'b00, rgb_in[23:18]} + sc[23:16];
    wire [7:0] mix_g = {2'b00, rgb_in[15:10]} + sc[15:8];
    wire [7:0] mix_b = {2'b00, rgb_in[7:2]}   + sc[7:0];

    assign rgb_out = capp  ? sc :
                     !body ? rgb_in :
                     tip   ? sc :
                             {mix_r, mix_g, mix_b};

    // ============================ 引擎 ============================
    // 每档步骤（st）：读 → 两拍后进捕获寄存器；乘积寄存器 p 晚一拍
    //   0 读 meta   1 读 v   2 读 win(j8)、记窗末   3 读 S   4 读 g、乘 w·x   5 读 P、R=xw
    //   6 读 -ε、乘 g·xw   7 读 D、R=v+xs   8 读 ε、乘 -ε·S   9 R=v'   10 乘 ε·v'、写 v
    //   11 乘 v'²、R=S'   12 乘 S'·S、Q=v'²、写 S、读下一档 meta   13 写 P、读下一档 v → 下一档从 2 开始
    //   帧更新：13 之后规格化 P（≤16 拍），再写 D
    wire       ce = !disp_rd;                      // 显示读的那拍引擎整体停一拍

    reg        busy, do_s, do_u, nrm_run;
    reg  [3:0] st;
    reg  [4:0] b;
    reg  signed [15:0] x;
    reg  [10:0] n11;
    reg        fid, seen, silent, loud, upd_pend;
    reg  [5:0] qcnt;

    reg  [17:0] cr, vr, sr, pr;
    reg  [16:0] dr;
    reg  [2:0]  rd_tag;
    reg         rd_v;
    reg  signed [17:0] rr;
    reg  [16:0] qv;
    reg  signed [35:0] p;
    reg         end_r;
    reg  [16:0] nrm;
    reg  [4:0]  ex;

    localparam [2:0] TG_C = 3'd0, TG_V = 3'd1, TG_S = 3'd2, TG_P = 3'd3, TG_D = 3'd4;

    wire       last_b = (b == 5'd31);
    wire [4:0] b_nx   = b + 1'b1;
    wire       eng    = busy && !nrm_run;

    // Hann 窗地址 / 窗末：meta = {s(3), mask(11)}，s = 11 - log2N
    wire [17:0] n_sh  = {7'd0, n11} << cr[13:11];
    wire [7:0]  j8    = n_sh[10:3];

    // 读口：引擎
    reg         e_rd;
    reg  [8:0]  e_ra;
    reg  [2:0]  e_tag;
    always @* begin
        e_rd  = 1'b0;
        e_ra  = {1'b0, b, 3'd4};
        e_tag = TG_C;
        case (st)
            4'd0:  begin e_rd = 1'b1;           e_ra = {1'b0, b, 3'd4};    e_tag = TG_C; end
            4'd1:  begin e_rd = 1'b1;           e_ra = {1'b0, b, 3'd0};    e_tag = TG_V; end
            4'd2:  begin e_rd = 1'b1;           e_ra = {1'b1, j8};         e_tag = TG_C; end
            4'd3:  begin e_rd = 1'b1;           e_ra = {1'b0, b, 3'd1};    e_tag = TG_S; end
            4'd4:  begin e_rd = 1'b1;           e_ra = {1'b0, b, 3'd5};    e_tag = TG_C; end
            4'd5:  begin e_rd = 1'b1;           e_ra = {1'b0, b, 3'd2};    e_tag = TG_P; end
            4'd6:  begin e_rd = 1'b1;           e_ra = {1'b0, b, 3'd6};    e_tag = TG_C; end
            4'd7:  begin e_rd = do_u;           e_ra = {1'b0, b, 3'd3};    e_tag = TG_D; end
            4'd8:  begin e_rd = 1'b1;           e_ra = {1'b0, b, 3'd7};    e_tag = TG_C; end
            4'd12: begin e_rd = !last_b;        e_ra = {1'b0, b_nx, 3'd4}; e_tag = TG_C; end
            4'd13: begin e_rd = !last_b;        e_ra = {1'b0, b_nx, 3'd0}; e_tag = TG_V; end
            default: ;
        endcase
    end
    wire e_go = eng && ce && e_rd;
    assign re = disp_rd || e_go;
    assign ra = disp_rd ? {1'b0, rb[4:0], 3'd3} : e_ra;

    // 读数据捕获（发读后一拍，与是否停顿无关）
    always @(posedge clk) begin
        rd_v   <= e_go;
        rd_tag <= e_tag;
        if (rd_v)
            case (rd_tag)
                TG_C: cr <= q;
                TG_V: vr <= q;
                TG_S: sr <= q;
                TG_P: pr <= q;
                default: dr <= q[16:0];
            endcase
    end

    // 乘法器（分时 6 用）：A ∈ {cr, R}，B ∈ {x, R, S}
    wire dsp_st = (st == 4'd4) || (st == 4'd6) || (st == 4'd8) || (st == 4'd10) || (st == 4'd11) || (st == 4'd12);
    wire signed [17:0] dsp_a = ((st == 4'd11) || (st == 4'd12)) ? rr : $signed(cr);
    wire signed [17:0] dsp_b = (st == 4'd4) ? {{2{x[15]}}, x} :
                               ((st == 4'd8) || (st == 4'd12)) ? $signed(sr) : rr;
    always @(posedge clk)
        if (eng && ce && dsp_st)
            p <= dsp_a * dsp_b;

    // 唯一的加法器：R = X + p[33:16]，X ∈ {0, v, R, S}
    wire signed [17:0] p_hi  = p[33:16];
    wire signed [17:0] add_x = (st == 4'd7) ? $signed(vr) :
                               (st == 4'd9) ? rr :
                               (st == 4'd11) ? $signed(sr) : 18'sd0;
    wire signed [17:0] add_o = add_x + p_hi;

    // 窗末功率：Q = sat(v'²>>12)，pw = sat(Q + (S'·S >>> 12))，非负 17 位
    wire [24:0] pw_sum = {8'd0, qv} + {p[35], p[35:12]};
    wire [16:0] pw     = pw_sum[24] ? 17'd0 : (pw_sum[23:17] != 7'd0) ? 17'h1ffff : pw_sum[16:0];
    wire        p_new  = (pr[17] != fid) || (pw > pr[16:0]);
    wire [17:0] pn     = p_new ? {fid, pw} : pr;
    wire        p_wr   = do_s && end_r;
    wire [16:0] pcur   = p_wr ? pn[16:0] : pr[16:0];

    // 帧更新：L = {ex, nrm[15:14]}，h_t = min(L, 63)
    wire        nrm_done = nrm[16] || (ex == 5'd0);
    wire [6:0]  lv       = {ex, nrm[15:14]};
    wire [5:0]  ht       = silent ? 6'd0 : (lv[6] ? 6'd63 : lv[5:0]);
    wire [5:0]  ho       = dr[5:0];
    wire [5:0]  co       = dr[11:6];
    wire [4:0]  hd       = dr[16:12];
    wire [5:0]  hn       = (ht >= ho) ? ht : (ho > FALL) ? ho - FALL : 6'd0;
    wire        cap_up   = (hn >= co);
    wire [5:0]  cn       = cap_up ? hn : (hd != 5'd0 || co == 6'd0) ? co : co - 1'b1;
    wire [4:0]  hdn      = cap_up ? HOLD5 : (hd != 5'd0) ? hd - 1'b1 : 5'd0;
    wire [16:0] dn       = {hdn, cn, hn};

    always @(posedge clk) begin
        we <= 1'b0;
        if (eng && ce) begin
            case (st)
                4'd2:  end_r <= &(n11 | ~cr[10:0]);
                4'd5:  rr <= add_o;                    // add_x = 0：R = xw
                4'd7, 4'd9, 4'd11: rr <= add_o;
                4'd12: qv <= (p[35:29] != 7'd0) ? 17'h1ffff : p[28:12];
                default: ;
            endcase
            // 写口：v / S 窗末写 0
            if (do_s && (st == 4'd10 || st == 4'd12)) begin
                we <= 1'b1;
                wa <= {1'b0, b, 2'b00, st[2]};          // 10 → v（字 0），12 → S（字 1）
                wd <= end_r ? 18'd0 : rr;
            end
            if (st == 4'd13 && p_wr) begin
                we <= 1'b1;
                wa <= {1'b0, b, 3'd2};
                wd <= pn;
            end
            if (st == 4'd13 && do_u) begin
                nrm <= pcur;
                ex  <= 5'd16;
            end
        end
        if (nrm_run) begin
            if (!nrm_done) begin
                nrm <= nrm << 1;
                ex  <= ex - 1'b1;
            end else begin
                we <= 1'b1;
                wa <= {1'b0, b, 3'd3};
                wd <= {1'b0, dn};
            end
        end
    end

    // 样本锁存（数据寄存器不复位）
    always @(posedge clk)
        if (!busy && enable && pcm_tap_valid)
            x <= pcm_tap_left;

    // 控制：启动 / 步进 / 帧末
    wire bin_done = (eng && ce && st == 4'd13 && !do_u) || (nrm_run && nrm_done);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy     <= 1'b0;
            nrm_run  <= 1'b0;
            st       <= 4'd0;
            b        <= 5'd0;
            do_s     <= 1'b0;
            do_u     <= 1'b0;
            n11      <= 11'd0;
            fid      <= 1'b0;
            seen     <= 1'b0;
            silent   <= 1'b1;
            loud     <= 1'b0;
            upd_pend <= 1'b0;
            qcnt     <= 6'd0;
            wopen    <= 5'd0;
        end else begin
            if (frame_start)
                upd_pend <= 1'b1;
            if (!busy) begin
                if (enable && (pcm_tap_valid || upd_pend)) begin
                    busy <= 1'b1;
                    st   <= 4'd0;
                    b    <= 5'd0;
                    do_s <= pcm_tap_valid;
                    do_u <= upd_pend;
                    if (upd_pend) begin
                        upd_pend <= frame_start;
                        silent   <= !seen;
                        seen     <= pcm_tap_valid;
                    end else
                        seen <= 1'b1;
                end
            end else begin
                if (eng && ce) begin
                    if (st == 4'd13) begin
                        if (do_u)
                            nrm_run <= 1'b1;
                    end else
                        st <= st + 1'b1;
                end
                if (nrm_run && nrm_done) begin
                    nrm_run <= 1'b0;
                    if (ht >= 6'd12)
                        loud <= 1'b1;
                end
                if (bin_done) begin
                    st <= 4'd2;
                    b  <= b_nx;
                    if (last_b) begin
                        busy <= 1'b0;
                        if (do_s)
                            n11 <= n11 + 1'b1;
                        if (do_u) begin
                            fid  <= ~fid;
                            loud <= 1'b0;
                            if (loud || (nrm_run && ht >= 6'd12)) begin
                                qcnt  <= 6'd0;
                                wopen <= (wopen >= 5'd14) ? 5'd16 : wopen + 5'd2;
                            end else if (qcnt != QUIET6)
                                qcnt <= qcnt + 1'b1;
                            else if (wopen != 5'd0)
                                wopen <= wopen - 1'b1;
                        end
                    end
                end
            end
        end
    end

    // ============================ 初值：状态清零 + ROM（tools/spectrum/gsm.py 生成）============================
    // @@ROM_BEGIN
    initial begin
        mem[  0] = 18'h00000;  mem[  1] = 18'h00000;  mem[  2] = 18'h00000;  mem[  3] = 18'h00000;
        mem[  4] = 18'h007FF;  mem[  5] = 18'h00080;  mem[  6] = 18'h3FE53;  mem[  7] = 18'h001AD;   //  0:    50.0 Hz  N=2048
        mem[  8] = 18'h00000;  mem[  9] = 18'h00000;  mem[ 10] = 18'h00000;  mem[ 11] = 18'h00000;
        mem[ 12] = 18'h007FF;  mem[ 13] = 18'h00080;  mem[ 14] = 18'h3FD85;  mem[ 15] = 18'h0027B;   //  1:    74.0 Hz  N=2048
        mem[ 16] = 18'h00000;  mem[ 17] = 18'h00000;  mem[ 18] = 18'h00000;  mem[ 19] = 18'h00000;
        mem[ 20] = 18'h007FF;  mem[ 21] = 18'h00080;  mem[ 22] = 18'h3FCB7;  mem[ 23] = 18'h00349;   //  2:    98.0 Hz  N=2048
        mem[ 24] = 18'h00000;  mem[ 25] = 18'h00000;  mem[ 26] = 18'h00000;  mem[ 27] = 18'h00000;
        mem[ 28] = 18'h007FF;  mem[ 29] = 18'h00080;  mem[ 30] = 18'h3FBE9;  mem[ 31] = 18'h00417;   //  3:   122.0 Hz  N=2048
        mem[ 32] = 18'h00000;  mem[ 33] = 18'h00000;  mem[ 34] = 18'h00000;  mem[ 35] = 18'h00000;
        mem[ 36] = 18'h007FF;  mem[ 37] = 18'h00080;  mem[ 38] = 18'h3FB1C;  mem[ 39] = 18'h004E4;   //  4:   146.0 Hz  N=2048
        mem[ 40] = 18'h00000;  mem[ 41] = 18'h00000;  mem[ 42] = 18'h00000;  mem[ 43] = 18'h00000;
        mem[ 44] = 18'h007FF;  mem[ 45] = 18'h00080;  mem[ 46] = 18'h3FA3D;  mem[ 47] = 18'h005C3;   //  5:   171.9 Hz  N=2048
        mem[ 48] = 18'h00000;  mem[ 49] = 18'h00000;  mem[ 50] = 18'h00000;  mem[ 51] = 18'h00000;
        mem[ 52] = 18'h00BFF;  mem[ 53] = 18'h00100;  mem[ 54] = 18'h3F938;  mem[ 55] = 18'h006C8;   //  6:   202.4 Hz  N=1024
        mem[ 56] = 18'h00000;  mem[ 57] = 18'h00000;  mem[ 58] = 18'h00000;  mem[ 59] = 18'h00000;
        mem[ 60] = 18'h00BFF;  mem[ 61] = 18'h00100;  mem[ 62] = 18'h3F804;  mem[ 63] = 18'h007FC;   //  7:   238.3 Hz  N=1024
        mem[ 64] = 18'h00000;  mem[ 65] = 18'h00000;  mem[ 66] = 18'h00000;  mem[ 67] = 18'h00000;
        mem[ 68] = 18'h00BFF;  mem[ 69] = 18'h00100;  mem[ 70] = 18'h3F699;  mem[ 71] = 18'h00967;   //  8:   280.6 Hz  N=1024
        mem[ 72] = 18'h00000;  mem[ 73] = 18'h00000;  mem[ 74] = 18'h00000;  mem[ 75] = 18'h00000;
        mem[ 76] = 18'h00BFF;  mem[ 77] = 18'h00100;  mem[ 78] = 18'h3F4EE;  mem[ 79] = 18'h00B12;   //  9:   330.3 Hz  N=1024
        mem[ 80] = 18'h00000;  mem[ 81] = 18'h00000;  mem[ 82] = 18'h00000;  mem[ 83] = 18'h00000;
        mem[ 84] = 18'h011FF;  mem[ 85] = 18'h00200;  mem[ 86] = 18'h3F2F8;  mem[ 87] = 18'h00D08;   // 10:   389.0 Hz  N= 512
        mem[ 88] = 18'h00000;  mem[ 89] = 18'h00000;  mem[ 90] = 18'h00000;  mem[ 91] = 18'h00000;
        mem[ 92] = 18'h011FF;  mem[ 93] = 18'h00200;  mem[ 94] = 18'h3F0A8;  mem[ 95] = 18'h00F58;   // 11:   458.0 Hz  N= 512
        mem[ 96] = 18'h00000;  mem[ 97] = 18'h00000;  mem[ 98] = 18'h00000;  mem[ 99] = 18'h00000;
        mem[100] = 18'h011FF;  mem[101] = 18'h00200;  mem[102] = 18'h3EDEF;  mem[103] = 18'h01211;   // 12:   539.2 Hz  N= 512
        mem[104] = 18'h00000;  mem[105] = 18'h00000;  mem[106] = 18'h00000;  mem[107] = 18'h00000;
        mem[108] = 18'h011FF;  mem[109] = 18'h00231;  mem[110] = 18'h3EABB;  mem[111] = 18'h01545;   // 13:   634.8 Hz  N= 512
        mem[112] = 18'h00000;  mem[113] = 18'h00000;  mem[114] = 18'h00000;  mem[115] = 18'h00000;
        mem[116] = 18'h018FF;  mem[117] = 18'h004CF;  mem[118] = 18'h3E6F6;  mem[119] = 18'h0190A;   // 14:   747.5 Hz  N= 256
        mem[120] = 18'h00000;  mem[121] = 18'h00000;  mem[122] = 18'h00000;  mem[123] = 18'h00000;
        mem[124] = 18'h018FF;  mem[125] = 18'h00546;  mem[126] = 18'h3E286;  mem[127] = 18'h01D7A;   // 15:   880.1 Hz  N= 256
        mem[128] = 18'h00000;  mem[129] = 18'h00000;  mem[130] = 18'h00000;  mem[131] = 18'h00000;
        mem[132] = 18'h018FF;  mem[133] = 18'h005C8;  mem[134] = 18'h3DD4E;  mem[135] = 18'h022B2;   // 16:  1036.2 Hz  N= 256
        mem[136] = 18'h00000;  mem[137] = 18'h00000;  mem[138] = 18'h00000;  mem[139] = 18'h00000;
        mem[140] = 18'h018FF;  mem[141] = 18'h00657;  mem[142] = 18'h3D729;  mem[143] = 18'h028D7;   // 17:  1220.0 Hz  N= 256
        mem[144] = 18'h00000;  mem[145] = 18'h00000;  mem[146] = 18'h00000;  mem[147] = 18'h00000;
        mem[148] = 18'h018FF;  mem[149] = 18'h006F4;  mem[150] = 18'h3CFEF;  mem[151] = 18'h03011;   // 18:  1436.4 Hz  N= 256
        mem[152] = 18'h00000;  mem[153] = 18'h00000;  mem[154] = 18'h00000;  mem[155] = 18'h00000;
        mem[156] = 18'h0207F;  mem[157] = 18'h00F3E;  mem[158] = 18'h3C771;  mem[159] = 18'h0388F;   // 19:  1691.3 Hz  N= 128
        mem[160] = 18'h00000;  mem[161] = 18'h00000;  mem[162] = 18'h00000;  mem[163] = 18'h00000;
        mem[164] = 18'h0207F;  mem[165] = 18'h010B7;  mem[166] = 18'h3BD76;  mem[167] = 18'h0428A;   // 20:  1991.3 Hz  N= 128
        mem[168] = 18'h00000;  mem[169] = 18'h00000;  mem[170] = 18'h00000;  mem[171] = 18'h00000;
        mem[172] = 18'h0207F;  mem[173] = 18'h01254;  mem[174] = 18'h3B1BE;  mem[175] = 18'h04E42;   // 21:  2344.6 Hz  N= 128
        mem[176] = 18'h00000;  mem[177] = 18'h00000;  mem[178] = 18'h00000;  mem[179] = 18'h00000;
        mem[180] = 18'h0207F;  mem[181] = 18'h01418;  mem[182] = 18'h3A3FF;  mem[183] = 18'h05C01;   // 22:  2760.5 Hz  N= 128
        mem[184] = 18'h00000;  mem[185] = 18'h00000;  mem[186] = 18'h00000;  mem[187] = 18'h00000;
        mem[188] = 18'h0283F;  mem[189] = 18'h02C11;  mem[190] = 18'h393E8;  mem[191] = 18'h06C18;   // 23:  3250.2 Hz  N=  64
        mem[192] = 18'h00000;  mem[193] = 18'h00000;  mem[194] = 18'h00000;  mem[195] = 18'h00000;
        mem[196] = 18'h0283F;  mem[197] = 18'h03052;  mem[198] = 18'h3811A;  mem[199] = 18'h07EE6;   // 24:  3826.8 Hz  N=  64
        mem[200] = 18'h00000;  mem[201] = 18'h00000;  mem[202] = 18'h00000;  mem[203] = 18'h00000;
        mem[204] = 18'h0283F;  mem[205] = 18'h034FB;  mem[206] = 18'h36B31;  mem[207] = 18'h094CF;   // 25:  4505.7 Hz  N=  64
        mem[208] = 18'h00000;  mem[209] = 18'h00000;  mem[210] = 18'h00000;  mem[211] = 18'h00000;
        mem[212] = 18'h0283F;  mem[213] = 18'h03A18;  mem[214] = 18'h351C8;  mem[215] = 18'h0AE38;   // 26:  5305.0 Hz  N=  64
        mem[216] = 18'h00000;  mem[217] = 18'h00000;  mem[218] = 18'h00000;  mem[219] = 18'h00000;
        mem[220] = 18'h0301F;  mem[221] = 18'h07F65;  mem[222] = 18'h33479;  mem[223] = 18'h0CB87;   // 27:  6246.1 Hz  N=  32
        mem[224] = 18'h00000;  mem[225] = 18'h00000;  mem[226] = 18'h00000;  mem[227] = 18'h00000;
        mem[228] = 18'h0301F;  mem[229] = 18'h08BAF;  mem[230] = 18'h312F8;  mem[231] = 18'h0ED08;   // 28:  7354.1 Hz  N=  32
        mem[232] = 18'h00000;  mem[233] = 18'h00000;  mem[234] = 18'h00000;  mem[235] = 18'h00000;
        mem[236] = 18'h0301F;  mem[237] = 18'h09929;  mem[238] = 18'h2ED21;  mem[239] = 18'h112DF;   // 29:  8658.7 Hz  N=  32
        mem[240] = 18'h00000;  mem[241] = 18'h00000;  mem[242] = 18'h00000;  mem[243] = 18'h00000;
        mem[244] = 18'h0301F;  mem[245] = 18'h0A7F0;  mem[246] = 18'h2C329;  mem[247] = 18'h13CD7;   // 30: 10194.8 Hz  N=  32
        mem[248] = 18'h00000;  mem[249] = 18'h00000;  mem[250] = 18'h00000;  mem[251] = 18'h00000;
        mem[252] = 18'h0380F;  mem[253] = 18'h17048;  mem[254] = 18'h295E2;  mem[255] = 18'h16A1E;   // 31: 12003.3 Hz  N=  16
        mem[256] = 18'h00002;  mem[257] = 18'h00016;  mem[258] = 18'h0003E;  mem[259] = 18'h00079;
        mem[260] = 18'h000C8;  mem[261] = 18'h0012A;  mem[262] = 18'h001A0;  mem[263] = 18'h0022A;
        mem[264] = 18'h002C6;  mem[265] = 18'h00377;  mem[266] = 18'h0043A;  mem[267] = 18'h00511;
        mem[268] = 18'h005FA;  mem[269] = 18'h006F6;  mem[270] = 18'h00805;  mem[271] = 18'h00927;
        mem[272] = 18'h00A5A;  mem[273] = 18'h00BA0;  mem[274] = 18'h00CF8;  mem[275] = 18'h00E62;
        mem[276] = 18'h00FDD;  mem[277] = 18'h01169;  mem[278] = 18'h01307;  mem[279] = 18'h014B5;
        mem[280] = 18'h01674;  mem[281] = 18'h01843;  mem[282] = 18'h01A22;  mem[283] = 18'h01C11;
        mem[284] = 18'h01E0F;  mem[285] = 18'h0201C;  mem[286] = 18'h02238;  mem[287] = 18'h02463;
        mem[288] = 18'h0269B;  mem[289] = 18'h028E2;  mem[290] = 18'h02B36;  mem[291] = 18'h02D97;
        mem[292] = 18'h03004;  mem[293] = 18'h0327E;  mem[294] = 18'h03504;  mem[295] = 18'h03796;
        mem[296] = 18'h03A33;  mem[297] = 18'h03CDA;  mem[298] = 18'h03F8C;  mem[299] = 18'h04248;
        mem[300] = 18'h0450D;  mem[301] = 18'h047DB;  mem[302] = 18'h04AB2;  mem[303] = 18'h04D91;
        mem[304] = 18'h05078;  mem[305] = 18'h05367;  mem[306] = 18'h0565C;  mem[307] = 18'h05958;
        mem[308] = 18'h05C59;  mem[309] = 18'h05F60;  mem[310] = 18'h0626C;  mem[311] = 18'h0657D;
        mem[312] = 18'h06892;  mem[313] = 18'h06BAA;  mem[314] = 18'h06EC6;  mem[315] = 18'h071E4;
        mem[316] = 18'h07504;  mem[317] = 18'h07826;  mem[318] = 18'h07B49;  mem[319] = 18'h07E6D;
        mem[320] = 18'h08192;  mem[321] = 18'h084B6;  mem[322] = 18'h087D9;  mem[323] = 18'h08AFB;
        mem[324] = 18'h08E1B;  mem[325] = 18'h09139;  mem[326] = 18'h09455;  mem[327] = 18'h0976D;
        mem[328] = 18'h09A82;  mem[329] = 18'h09D93;  mem[330] = 18'h0A09F;  mem[331] = 18'h0A3A6;
        mem[332] = 18'h0A6A7;  mem[333] = 18'h0A9A3;  mem[334] = 18'h0AC98;  mem[335] = 18'h0AF87;
        mem[336] = 18'h0B26E;  mem[337] = 18'h0B54D;  mem[338] = 18'h0B824;  mem[339] = 18'h0BAF2;
        mem[340] = 18'h0BDB7;  mem[341] = 18'h0C073;  mem[342] = 18'h0C325;  mem[343] = 18'h0C5CC;
        mem[344] = 18'h0C869;  mem[345] = 18'h0CAFB;  mem[346] = 18'h0CD81;  mem[347] = 18'h0CFFB;
        mem[348] = 18'h0D268;  mem[349] = 18'h0D4C9;  mem[350] = 18'h0D71D;  mem[351] = 18'h0D964;
        mem[352] = 18'h0DB9C;  mem[353] = 18'h0DDC7;  mem[354] = 18'h0DFE3;  mem[355] = 18'h0E1F0;
        mem[356] = 18'h0E3EE;  mem[357] = 18'h0E5DD;  mem[358] = 18'h0E7BC;  mem[359] = 18'h0E98B;
        mem[360] = 18'h0EB4A;  mem[361] = 18'h0ECF8;  mem[362] = 18'h0EE96;  mem[363] = 18'h0F022;
        mem[364] = 18'h0F19D;  mem[365] = 18'h0F307;  mem[366] = 18'h0F45F;  mem[367] = 18'h0F5A5;
        mem[368] = 18'h0F6D8;  mem[369] = 18'h0F7FA;  mem[370] = 18'h0F909;  mem[371] = 18'h0FA05;
        mem[372] = 18'h0FAEE;  mem[373] = 18'h0FBC5;  mem[374] = 18'h0FC88;  mem[375] = 18'h0FD39;
        mem[376] = 18'h0FDD5;  mem[377] = 18'h0FE5F;  mem[378] = 18'h0FED5;  mem[379] = 18'h0FF37;
        mem[380] = 18'h0FF86;  mem[381] = 18'h0FFC1;  mem[382] = 18'h0FFE9;  mem[383] = 18'h0FFFD;
        mem[384] = 18'h0FFFD;  mem[385] = 18'h0FFE9;  mem[386] = 18'h0FFC1;  mem[387] = 18'h0FF86;
        mem[388] = 18'h0FF37;  mem[389] = 18'h0FED5;  mem[390] = 18'h0FE5F;  mem[391] = 18'h0FDD5;
        mem[392] = 18'h0FD39;  mem[393] = 18'h0FC88;  mem[394] = 18'h0FBC5;  mem[395] = 18'h0FAEE;
        mem[396] = 18'h0FA05;  mem[397] = 18'h0F909;  mem[398] = 18'h0F7FA;  mem[399] = 18'h0F6D8;
        mem[400] = 18'h0F5A5;  mem[401] = 18'h0F45F;  mem[402] = 18'h0F307;  mem[403] = 18'h0F19D;
        mem[404] = 18'h0F022;  mem[405] = 18'h0EE96;  mem[406] = 18'h0ECF8;  mem[407] = 18'h0EB4A;
        mem[408] = 18'h0E98B;  mem[409] = 18'h0E7BC;  mem[410] = 18'h0E5DD;  mem[411] = 18'h0E3EE;
        mem[412] = 18'h0E1F0;  mem[413] = 18'h0DFE3;  mem[414] = 18'h0DDC7;  mem[415] = 18'h0DB9C;
        mem[416] = 18'h0D964;  mem[417] = 18'h0D71D;  mem[418] = 18'h0D4C9;  mem[419] = 18'h0D268;
        mem[420] = 18'h0CFFB;  mem[421] = 18'h0CD81;  mem[422] = 18'h0CAFB;  mem[423] = 18'h0C869;
        mem[424] = 18'h0C5CC;  mem[425] = 18'h0C325;  mem[426] = 18'h0C073;  mem[427] = 18'h0BDB7;
        mem[428] = 18'h0BAF2;  mem[429] = 18'h0B824;  mem[430] = 18'h0B54D;  mem[431] = 18'h0B26E;
        mem[432] = 18'h0AF87;  mem[433] = 18'h0AC98;  mem[434] = 18'h0A9A3;  mem[435] = 18'h0A6A7;
        mem[436] = 18'h0A3A6;  mem[437] = 18'h0A09F;  mem[438] = 18'h09D93;  mem[439] = 18'h09A82;
        mem[440] = 18'h0976D;  mem[441] = 18'h09455;  mem[442] = 18'h09139;  mem[443] = 18'h08E1B;
        mem[444] = 18'h08AFB;  mem[445] = 18'h087D9;  mem[446] = 18'h084B6;  mem[447] = 18'h08192;
        mem[448] = 18'h07E6D;  mem[449] = 18'h07B49;  mem[450] = 18'h07826;  mem[451] = 18'h07504;
        mem[452] = 18'h071E4;  mem[453] = 18'h06EC6;  mem[454] = 18'h06BAA;  mem[455] = 18'h06892;
        mem[456] = 18'h0657D;  mem[457] = 18'h0626C;  mem[458] = 18'h05F60;  mem[459] = 18'h05C59;
        mem[460] = 18'h05958;  mem[461] = 18'h0565C;  mem[462] = 18'h05367;  mem[463] = 18'h05078;
        mem[464] = 18'h04D91;  mem[465] = 18'h04AB2;  mem[466] = 18'h047DB;  mem[467] = 18'h0450D;
        mem[468] = 18'h04248;  mem[469] = 18'h03F8C;  mem[470] = 18'h03CDA;  mem[471] = 18'h03A33;
        mem[472] = 18'h03796;  mem[473] = 18'h03504;  mem[474] = 18'h0327E;  mem[475] = 18'h03004;
        mem[476] = 18'h02D97;  mem[477] = 18'h02B36;  mem[478] = 18'h028E2;  mem[479] = 18'h0269B;
        mem[480] = 18'h02463;  mem[481] = 18'h02238;  mem[482] = 18'h0201C;  mem[483] = 18'h01E0F;
        mem[484] = 18'h01C11;  mem[485] = 18'h01A22;  mem[486] = 18'h01843;  mem[487] = 18'h01674;
        mem[488] = 18'h014B5;  mem[489] = 18'h01307;  mem[490] = 18'h01169;  mem[491] = 18'h00FDD;
        mem[492] = 18'h00E62;  mem[493] = 18'h00CF8;  mem[494] = 18'h00BA0;  mem[495] = 18'h00A5A;
        mem[496] = 18'h00927;  mem[497] = 18'h00805;  mem[498] = 18'h006F6;  mem[499] = 18'h005FA;
        mem[500] = 18'h00511;  mem[501] = 18'h0043A;  mem[502] = 18'h00377;  mem[503] = 18'h002C6;
        mem[504] = 18'h0022A;  mem[505] = 18'h001A0;  mem[506] = 18'h0012A;  mem[507] = 18'h000C8;
        mem[508] = 18'h00079;  mem[509] = 18'h0003E;  mem[510] = 18'h00016;  mem[511] = 18'h00002;
    end
    // @@ROM_END
endmodule
