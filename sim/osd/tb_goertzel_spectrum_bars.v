`timescale 1ns/1ps
// tb_goertzel_spectrum_bars —— 频谱柱模块验证
//   激励 sim/spectrum/stim.hex（gen_stim.py：双音 200 Hz + 2.5 kHz → 真实曲目片段 → 静音），48 kHz 样本节拍（25 MHz 下 520/521 拍）
//   1. 引擎 / 帧更新：日志 sim/spectrum/rtl.log（每遍的样本、窗末功率、帧更新 Pcur/ht/D），
//      由 check.py 用 tools/spectrum/gsm.py 逐位比对
//   2. 像素：带内每个像素与本 tb 的独立参考比对（读模块 BRAM 里的 D 字 + wopen），带外 / enable=0 时必须原样透传
//   3. 收拢：静音段结束时 wopen 必须收到 0，所有柱高 0
module tb_goertzel_spectrum_bars;
    reg clk = 1'b0;
    always #20 clk = ~clk;
    reg rst_n = 1'b0;

    // 640x480 时序：800 x 525
    reg [9:0] hc = 10'd0, vc = 10'd0;
    always @(posedge clk) begin
        hc <= (hc == 10'd799) ? 10'd0 : hc + 1'b1;
        if (hc == 10'd799) vc <= (vc == 10'd524) ? 10'd0 : vc + 1'b1;
    end
    wire de          = (hc < 10'd640) && (vc < 10'd480);
    wire [9:0] px    = de ? hc : 10'd0;
    wire [9:0] py    = vc;
    wire frame_start = (vc == 10'd490) && (hc == 10'd0);
    reg dark = 1'b0;
    initial dark = $test$plusargs("dark");
    wire [23:0] rgb_in = dark ? {8'd6 + {4'd0, hc[7:4]}, 8'd8 + {5'd0, vc[6:4]}, 8'd24 + {4'd0, vc[5:2]}}
                              : {hc[7:0], vc[7:0], hc[7:0] ^ vc[7:0]};

    // 48 kHz 样本
    reg [15:0] stim [0:45599];
    initial $readmemh("sim/spectrum/stim.hex", stim);
    reg [24:0] acc = 25'd0;
    reg        tick = 1'b0;
    integer    si = 0;
    reg signed [15:0] smp = 16'sd0;
    always @(posedge clk) begin
        tick <= 1'b0;
        if (rst_n) begin
            if (acc + 25'd48000 >= 25'd25000000) begin
                acc  <= acc + 25'd48000 - 25'd25000000;
                tick <= (si < 45600);
                smp  <= (si < 45600) ? stim[si] : 16'sd0;
                si   <= si + 1;
            end else
                acc <= acc + 25'd48000;
        end
    end

    reg enable = 1'b1;
    reg [6:0] fcnt = 7'd0;                 // 色相流动：同顶层 ph_frame_cnt[6:2]
    always @(posedge clk) if (frame_start) fcnt <= fcnt + 1'b1;
    // 亮度同步：增益每帧轮换 256 / 192 / 128 / 64 / 1（帧边界变化，同顶层 bright_frame_pix + 1）
    reg [8:0] gain = 9'd256;
    always @(posedge clk)
        if (frame_start)
            case (fcnt % 5)
                0: gain <= 9'd192;  1: gain <= 9'd128;  2: gain <= 9'd64;  3: gain <= 9'd1;  default: gain <= 9'd256;
            endcase
    wire [23:0] rgb_out;
    goertzel_spectrum_bars #(
        .BAR_X (64), .BAR_Y (368), .FALL (2), .HOLD (6), .QUIET (8)
    ) dut (
        .clk (clk), .rst_n (rst_n), .frame_start (frame_start),
        .de (de), .rgb_in (rgb_in), .pixel_x (px), .pixel_y (py),
        .enable (enable), .hue_shift (fcnt[6:2]), .bright_gain (gain), .pcm_tap_valid (tick), .pcm_tap_left (smp),
        .rgb_out (rgb_out)
    );

    // ---------------- 日志（引擎 / 帧更新）----------------
    integer fd;
    reg busy_q = 1'b0;
    initial fd = $fopen("sim/spectrum/rtl.log", "w");
    always @(posedge clk) begin
        busy_q <= dut.busy;
        if (dut.busy && !busy_q)
            $fwrite(fd, "T %0d %0d %0d %0d\n", dut.do_s, dut.do_u, dut.x, dut.silent);
        if (dut.eng && dut.ce && dut.st == 4'd13 && dut.p_wr)
            $fwrite(fd, "W %0d %0d\n", dut.b, dut.pw);
        if (dut.eng && dut.ce && dut.st == 4'd13 && dut.do_u)
            $fwrite(fd, "N %0d %0d\n", dut.b, dut.pcur);
        if (dut.nrm_run && dut.nrm_done)
            $fwrite(fd, "D %0d %0d %0d %0d\n", dut.b, dut.ht, dut.dr, dut.dn);
    end

    // ---------------- 像素参考 ----------------
    // 64 柱：x-64 每 16 像素一档 k，前 8 像素左根 = 第 k 档，后 8 像素右根 = (第 k 档 + 第 min(k+1,31) 档)/2；根内 1..6 画柱
    // 色表按 tools/spectrum/palette.py 同一公式独立计算（插值后预乘 75%）；色相 = k − fcnt[6:2] + r/8
    function [23:0] key_c;
        input [2:0] i;
        case (i)
            3'd0: key_c = {8'd30,  8'd60,  8'd255};
            3'd1: key_c = {8'd0,   8'd170, 8'd255};
            3'd2: key_c = {8'd0,   8'd230, 8'd210};
            3'd3: key_c = {8'd255, 8'd215, 8'd80};
            3'd4: key_c = {8'd255, 8'd140, 8'd30};
            3'd5: key_c = {8'd255, 8'd60,  8'd90};
            3'd6: key_c = {8'd230, 8'd40,  8'd200};
            default: key_c = {8'd140, 8'd60, 8'd255};
        endcase
    endfunction
    function [23:0] pal_c;
        input [4:0] h;
        reg [23:0] a, b; integer f, c, v; reg [23:0] o;
        begin
            a = key_c(h[4:2]); b = key_c(h[4:2] + 3'd1); f = h[1:0];
            for (c = 0; c < 3; c = c + 1) begin
                v = ((a[c*8 +: 8] * (4 - f) + b[c*8 +: 8] * f) / 4) * 3 / 4;
                o[c*8 +: 8] = v;
            end
            pal_c = o;
        end
    endfunction
    function [23:0] ref_px;
        input [23:0] bg;
        input [9:0]  x, y;
        input        en;
        reg [9:0]  dy, xo; reg [5:0] r, h, c; reg [4:0] k, k1, dist, hue; reg [2:0] p8; reg sub;
        reg [17:0] d0, d1; reg [23:0] pc; integer ch, v, g;
        begin
            ref_px = bg;
            g = gain;
            dy = y - 10'd368; xo = x - 10'd64; r = 6'd63 - dy[5:0];
            k = xo[8:4]; sub = xo[3]; p8 = xo[2:0];
            if (en && dy < 10'd64 && x >= 10'd64 && x < 10'd576 && p8 >= 3'd1 && p8 <= 3'd6) begin
                dist = (k >= 5'd16) ? k - 5'd16 : 5'd15 - k;
                if (dist < dut.wopen) begin
                    k1 = (k == 5'd31) ? k : k + 5'd1;
                    d0 = dut.mem[{1'b0, k, 3'd3}]; d1 = dut.mem[{1'b0, k1, 3'd3}];
                    h = sub ? (d0[5:0] + d1[5:0]) / 2 : d0[5:0];
                    c = sub ? (d0[11:6] + d1[11:6]) / 2 : d0[11:6];
                    hue = k - fcnt[6:2] + r / 8;
                    pc = pal_c(hue);
                    // 亮度同步：前景色（峰值帽 / 柱顶 / 柱身色）× gain / 256，bg 不缩
                    if (c >= 6'd2 && (r >> 1) == (c >> 1)) begin
                        for (ch = 0; ch < 3; ch = ch + 1) begin v = ((8'hc0 + pc[ch*8 +: 8] / 4) * g) / 256; ref_px[ch*8 +: 8] = v; end
                    end else if (r < h) begin
                        for (ch = 0; ch < 3; ch = ch + 1) begin
                            if (r + 2 >= h) v = ((128 + pc[ch*8 +: 8] / 2) * g) / 256;
                            else            v = bg[ch*8 +: 8] / 4 + (pc[ch*8 +: 8] * g) / 256;
                            ref_px[ch*8 +: 8] = v;
                        end
                    end
                end
            end
        end
    endfunction

    integer px_err = 0, px_cnt = 0, lit_cnt = 0, frame_no = 0;
    always @(posedge clk) begin
        if (rst_n && de && frame_no >= 2) begin
            px_cnt = px_cnt + 1;
            if (rgb_out !== ref_px(rgb_in, px, py, enable)) begin
                if (px_err < 10)
                    $display("PIXEL MISMATCH f=%0d x=%0d y=%0d got=%h exp=%h dd0=%h dd1=%h bh=%0d t=%h m19=%h m20=%h", frame_no, px, py, rgb_out, ref_px(rgb_in, px, py, enable), dut.dd0, dut.dd1, dut.bh, dut.t, dut.mem[19*8+3], dut.mem[20*8+3]);
                px_err = px_err + 1;
            end
            if (rgb_out !== rgb_in) lit_cnt = lit_cnt + 1;
        end
        if (frame_start) frame_no = frame_no + 1;
    end

    // ---------------- 预览：+dump 时把若干帧的柱区（y 352..447）拼成 PPM（sim/spectrum/preview.ppm）----------------
    reg  dump_on = 1'b0;
    integer pf = 0;
    initial if ($test$plusargs("dump")) begin
        dump_on = 1'b1;
        pf = $fopen("sim/spectrum/preview.ppm", "w");
        $fwrite(pf, "P3\n640 %0d\n255\n", 96 * 6);
    end
    always @(posedge clk)
        if (dump_on && de && py >= 10'd352 && py < 10'd448 &&
            (frame_no == 10 || frame_no == 17 || frame_no == 20 || frame_no == 23 || frame_no == 26 || frame_no == 29))
            $fwrite(pf, "%0d %0d %0d\n", rgb_out[23:16], rgb_out[15:8], rgb_out[7:0]);

    // ---------------- 主流程 ----------------
    integer k, hsum, max_wopen, lit_music;
    initial begin
        max_wopen = 0; lit_music = 0;
        #400 rst_n = 1'b1;
        // 双音 + 曲目段（约 0.5 s = 30 帧）
        wait (si >= 24000);
        lit_music = lit_cnt;
        // 静音段：要求收拢
        wait (si >= 45600);
        repeat (2) @(posedge frame_start);
        hsum = 0;
        for (k = 0; k < 32; k = k + 1) hsum = hsum + dut.mem[k*8+3][5:0];
        $display("after silence: wopen=%0d sum(h)=%0d", dut.wopen, hsum);
        // enable=0 一帧：必须完全透传
        @(negedge clk) enable = 1'b0;
        lit_cnt = 0;
        @(posedge frame_start); @(posedge frame_start);
        $fclose(fd);
        if (dump_on) $fclose(pf);
        $display("pixels checked=%0d mismatches=%0d lit(music)=%0d lit(disabled)=%0d", px_cnt, px_err, lit_music, lit_cnt);
        if (px_err == 0 && lit_music > 20000 && lit_cnt == 0 && dut.wopen == 0 && hsum == 0)
            $display("PASS tb_goertzel_spectrum_bars pixel/collapse/passthrough (engine: python sim/spectrum/check.py sim/spectrum/rtl.log)");
        else
            $display("FAIL tb_goertzel_spectrum_bars");
        $finish;
    end
    always @(posedge clk) if (dut.wopen > max_wopen) max_wopen = dut.wopen;
endmodule
