`timescale 1ns/1ps

module tb_promax_text_overlay;
    reg clk = 1'b0;
    reg rst_n = 1'b0;
    always #20 clk = ~clk;

    // ---------------- 共享激励 ----------------
    reg [95:0] text_req   = 96'd0;
    reg [5:0]  len_req    = 6'd0;
    reg        sub_en_req = 1'b1;
    reg [1:0]  bright     = 2'd3;
    reg [7:0]  vol        = 8'd255;
    reg [8:0]  prd        = 9'd50;
    reg        auto_en    = 1'b0;
    reg        res_v      = 1'b1;    
    reg [11:0] res_w      = 12'd640;
    reg [11:0] res_h      = 12'd480;

    function [95:0] pack12;      // 字符串字面量（高字节在前）→ 第 i 个字符在 [i*8 +: 8]
        input [95:0] s;
        input integer n;
        integer i;
        begin
            pack12 = 96'd0;
            for (i = 0; i < n; i = i + 1)
                pack12[i*8 +: 8] = s[(n-1-i)*8 +: 8];
        end
    endfunction

    wire [31:0] err_a, err_b, frm_a, frm_b, px_a, px_b;

    tb_text_pair #(.H_ACT(640), .H_TOTAL(680), .TS_X(500)) u_a (
        .clk(clk), .rst_n(rst_n), .text_req(text_req), .len_req(len_req), .sub_en_req(sub_en_req),
        .bright(bright), .vol(vol), .prd(prd), .auto_en(auto_en), .res_v(res_v), .res_w(res_w), .res_h(res_h),
        .errors(err_a), .frames(frm_a), .checked(px_a));
    tb_text_pair #(.H_ACT(160), .H_TOTAL(200), .TS_X(104)) u_b (
        .clk(clk), .rst_n(rst_n), .text_req(text_req), .len_req(len_req), .sub_en_req(sub_en_req),
        .bright(bright), .vol(vol), .prd(prd), .auto_en(auto_en), .res_v(res_v), .res_w(res_w), .res_h(res_h),
        .errors(err_b), .frames(frm_b), .checked(px_b));

    // ---------------- BCD 计时单测 ----------------
    reg         ts_fs = 1'b0;
    integer     ts_cyc = 0;
    integer     ts_err = 0;
    integer     ts_ticks = 0;
    wire [23:0] ts_rgb_unused;
    timestamp_osd #(.OSD_X(0), .OSD_Y(0), .CLK_HZ(10)) u_ts_cnt (
        .pix_clk(clk), .pix_rst_n(rst_n), .enable(1'b0), .frame_start(ts_fs), .de(1'b0),
        .pixel_x(10'd0), .pixel_y(10'd0), .rgb_in(24'd0), .rgb_out(ts_rgb_unused),
        .ts_active(), .ts_pixel());
    wire [15:0] ts_digits = {u_ts_cnt.mm_tens, u_ts_cnt.mm_ones, u_ts_cnt.ss_tens, u_ts_cnt.ss_ones};
    reg  [15:0] ts_digits_q = 16'd0;
    integer     ts_shown;
    always @(posedge clk) if (rst_n) begin
        ts_cyc <= ts_cyc + 1;
        ts_fs  <= ((ts_cyc % 7) == 6);
        if (u_ts_cnt.sec_tick) ts_ticks = ts_ticks + 1;
        ts_digits_q <= ts_digits;
    end
    always @(negedge clk) if (rst_n && ts_cyc > 2) begin
        ts_shown = ((u_ts_cnt.mm_tens * 10 + u_ts_cnt.mm_ones) * 60 + u_ts_cnt.ss_tens * 10 + u_ts_cnt.ss_ones);
        if (!(ts_shown == ts_ticks % 6000 || ts_shown == (ts_ticks + 5999) % 6000)) begin
            if (ts_err < 5) $display("FAIL ts count shown=%0d ticks=%0d", ts_shown, ts_ticks);
            ts_err = ts_err + 1;
        end
        if (u_ts_cnt.ss_ones > 9 || u_ts_cnt.ss_tens > 5 || u_ts_cnt.mm_ones > 9 || u_ts_cnt.mm_tens > 9) begin
            if (ts_err < 5) $display("FAIL ts bcd range %h", ts_digits);
            ts_err = ts_err + 1;
        end
    end
    // 数字只能在 frame_start 后一拍变化
    reg ts_fs_q = 1'b0;
    always @(posedge clk) begin
        ts_fs_q <= ts_fs;
        if (rst_n && ts_cyc > 2 && ts_digits !== ts_digits_q && !ts_fs_q) begin
            if (ts_err < 5) $display("FAIL ts digits changed outside frame_start");
            ts_err = ts_err + 1;
        end
    end

    // ---------------- 激励时间表（按 B 的帧号推进；A/B 各自在本帧边界锁存字幕） ----------------
    // 分辨率与顶层约定一致：只在 A、B 的 frame_start 后 20 拍以外改变（帧中途仍然会变，检验帧锁存）
    task wait_res_window;
        begin
            while (u_a.fs_age < 20 || u_b.fs_age < 20) @(posedge clk);
        end
    endtask
    integer k;
    initial begin
        text_req = pack12("HELLO, FPGA!", 12); len_req = 12;
        #200 rst_n = 1'b1;
        for (k = 0; k < 950; k = k + 1) begin
            @(posedge u_b.frame_start);
            // 字幕：每 100 帧换一组（B 帧号）
            case (k)
                100: begin text_req = pack12("AB-9/x?", 7);       len_req = 7;  end
                200: begin text_req = pack12("Z", 1);             len_req = 1;  end
                300: begin                                        len_req = 0;  end
                350: begin text_req = pack12("0123456789:.", 12); len_req = 12; end
                400: sub_en_req = 1'b0;
                420: sub_en_req = 1'b1;
                600: begin text_req = pack12("QWERTYUIOP_,", 12); len_req = 12; end
                800: begin text_req = pack12("SCROLL TEST", 11);  len_req = 11; end
                default: ;
            endcase
            // 参数：帧中途（B 帧内随机时刻）改变，检验帧锁存
            if (k % 3 == 1) begin
                repeat ($urandom_range(50, 15000)) @(posedge clk);
                vol    = $urandom;
                bright = $urandom;
                case ($urandom_range(0, 5))
                    0: prd = 9'd5;  1: prd = 9'd10; 2: prd = 9'd25;
                    3: prd = 9'd50; 4: prd = 9'd100; default: prd = 9'd300;
                endcase
                if (k % 9 == 1) auto_en = ~auto_en;
                wait_res_window;
                case ($urandom_range(0, 15))
                    0: begin res_w = 12'd1;    res_h = 12'd1;    end
                    1: begin res_w = 12'd9;    res_h = 12'd10;   end
                    2: begin res_w = 12'd99;   res_h = 12'd100;  end
                    3: begin res_w = 12'd999;  res_h = 12'd1000; end
                    4: begin res_w = 12'd4095; res_h = 12'd4095; end
                    5: begin res_w = 12'd1000; res_h = 12'd9;    end
                    6: begin res_w = 12'd1920; res_h = 12'd1080; end
                    7: begin res_w = 12'd640;  res_h = 12'd480;  end
                    8: begin res_w = 12'd800;  res_h = 12'd600;  end
                    9: begin res_w = 12'd10;   res_h = 12'd99;   end
                    default: begin res_w = $urandom_range(1, 4095); res_h = $urandom_range(1, 4095); end
                endcase
                res_v = ($urandom_range(0, 7) != 0);
            end
            if (k % 3 == 2) begin                   // 另一个帧中途时刻再改一次（只改分辨率）
                repeat ($urandom_range(50, 15000)) @(posedge clk);
                wait_res_window;
                res_w = $urandom_range(1, 4095); res_h = $urandom_range(1, 4095);
            end
        end
        if (u_b.scroll_wraps < 1) begin
            $display("FAIL config B never wrapped the scroll cycle"); ts_err = ts_err + 1;
        end
        $display("INFO A: frames=%0d checked_px=%0d errors=%0d", frm_a, px_a, err_a);
        $display("INFO B: frames=%0d checked_px=%0d errors=%0d scroll_wraps=%0d", frm_b, px_b, err_b, u_b.scroll_wraps);
        $display("INFO ts counter: ticks=%0d errors=%0d", ts_ticks, ts_err);
        $display("INFO A res frames: w digits 1/2/3/4 = %0d/%0d/%0d/%0d, h = %0d/%0d/%0d/%0d, invalid = %0d",
                 u_a.cov_w1, u_a.cov_w2, u_a.cov_w3, u_a.cov_w4, u_a.cov_h1, u_a.cov_h2, u_a.cov_h3, u_a.cov_h4, u_a.cov_inv);
        if (u_a.cov_w1 == 0 || u_a.cov_w2 == 0 || u_a.cov_w3 == 0 || u_a.cov_w4 == 0 ||
            u_a.cov_h1 == 0 || u_a.cov_h2 == 0 || u_a.cov_h3 == 0 || u_a.cov_h4 == 0 || u_a.cov_inv == 0) begin
            $display("FAIL resolution coverage incomplete"); ts_err = ts_err + 1;
        end
        if (err_a == 0 && err_b == 0 && ts_err == 0 && px_a > 100000 && px_b > 100000 && ts_ticks > 6000)
            $display("PASS tb_promax_text_overlay");
        else
            $display("FAIL tb_promax_text_overlay");
        $finish;
    end
endmodule

// -----------------------------------------------------------------------------
// 一套视频时序 + DUT（本工程版）+ REF+ 期望模型
// -----------------------------------------------------------------------------
module tb_text_pair #(
    parameter integer H_ACT   = 640,
    parameter integer H_TOTAL = 680,
    parameter integer V_ACT   = 100,
    parameter integer V_FP    = 3,
    parameter integer V_SYNC  = 2,
    parameter integer V_BP    = 3,
    parameter integer TS_X    = 500
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [95:0] text_req,
    input  wire [5:0]  len_req,
    input  wire        sub_en_req,
    input  wire [1:0]  bright,
    input  wire [7:0]  vol,
    input  wire [8:0]  prd,
    input  wire        auto_en,
    input  wire        res_v,
    input  wire [11:0] res_w,
    input  wire [11:0] res_h,
    output reg  [31:0] errors,
    output reg  [31:0] frames,
    output reg  [31:0] checked
);
    localparam integer V_TOTAL = V_ACT + V_FP + V_SYNC + V_BP;
    localparam integer PX = 8, PY = 40, SY = 72;

    // ---- 时序 ----
    integer h = 0, v = V_ACT;
    wire de   = (h < H_ACT) && (v < V_ACT);
    wire vs_n = !((v >= V_ACT + V_FP) && (v < V_ACT + V_FP + V_SYNC));
    always @(posedge clk) begin
        if (h == H_TOTAL - 1) begin h <= 0; v <= (v == V_TOTAL - 1) ? 0 : v + 1; end
        else h <= h + 1;
    end

    // ---- 坐标重建（与 osd_layer_mixer 相同） ----
    reg [9:0] pixel_x = 0, pixel_y = 0;
    reg de_q = 0, vs_q = 1;
    wire frame_start = vs_q && !vs_n;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin pixel_x <= 0; pixel_y <= 0; de_q <= 0; vs_q <= 1; end
        else begin
            de_q <= de; vs_q <= vs_n;
            pixel_x <= de ? pixel_x + 10'd1 : 10'd0;
            if (!vs_n) pixel_y <= 0; else if (de_q && !de) pixel_y <= pixel_y + 10'd1;
        end
    end

    // ---- 字幕在本时序的帧边界锁存（模拟 subtitle_frame_commit 的 dst_*） ----
    reg [95:0] text = 0;
    reg [5:0]  len  = 0;
    reg        sub_en = 1;
    always @(posedge clk) if (frame_start) begin text <= text_req; len <= len_req; sub_en <= sub_en_req; end

    wire [23:0] rgb_in = {h[7:0], v[7:0], 8'h5a};
    reg  [7:0]  bright8;
    always @* case (bright) 2'd3: bright8 = 8'd255; 2'd2: bright8 = 8'd192; 2'd1: bright8 = 8'd128; default: bright8 = 8'd64; endcase
    // DUT 显示 8 位真实值；喂 100/75/50/25 让三位数字与参考模型（百分比）一致
    reg  [7:0]  dut_b8;
    always @* case (bright) 2'd3: dut_b8 = 8'd100; 2'd2: dut_b8 = 8'd75; 2'd1: dut_b8 = 8'd50; default: dut_b8 = 8'd25; endcase

    wire [23:0] dut_rgb, ref_rgb;
    wire dut_param, dut_sub, dut_band, dut_active, dut_changed;
    promax_text_overlay #(.PARAM_X(PX), .PARAM_Y(PY), .SUBTITLE_Y(SY), .H_ACTIVE(H_ACT)) dut (
        .clk(clk), .rst_n(rst_n), .frame_start(frame_start), .de(de), .rgb_in(rgb_in),
        .pixel_x(pixel_x), .pixel_y(pixel_y), .enable(1'b1), .subtitle_enable(sub_en),
        .subtitle_text(text), .subtitle_length(len), .brightness(dut_b8), .volume(vol),
        .period_tenths(prd), .auto_enabled(auto_en), .res_valid(res_v), .res_width(res_w), .res_height(res_h),
        .rgb_out(dut_rgb), .overlay_active(dut_active), .param_pixel(dut_param),
        .subtitle_pixel(dut_sub), .subtitle_band_pixel(dut_band), .param_changed(dut_changed));

    promax_text_overlay_ref #(.PARAM_X(PX), .PARAM_Y(PY), .SUBTITLE_Y(SY), .H_ACTIVE(H_ACT)) ref_u (
        .clk(clk), .rst_n(rst_n), .vsync(vs_n), .de(de), .rgb_in(rgb_in), .pixel_x(h[9:0]),
        .enable(1'b1), .subtitle_enable(sub_en), .subtitle_text(text), .subtitle_length(len),
        .brightness(bright8), .volume(vol), .period_ms(32'd5000), .auto_enabled(auto_en),
        .source_scale_2x(1'b0), .rgb_out(ref_rgb), .overlay_active());

    localparam integer TS_HZ = H_TOTAL * V_TOTAL * 3 / 2;
    wire [23:0] dut_ts_rgb, ref_ts_rgb;
    wire dut_ts_box, ref_ts_box, dut_ts_pix;
    timestamp_osd #(.OSD_X(TS_X), .OSD_Y(8), .CLK_HZ(TS_HZ)) dut_ts (
        .pix_clk(clk), .pix_rst_n(rst_n), .enable(1'b1), .frame_start(frame_start), .de(de),
        .pixel_x(pixel_x), .pixel_y(pixel_y), .rgb_in(rgb_in), .rgb_out(dut_ts_rgb),
        .ts_active(dut_ts_box), .ts_pixel(dut_ts_pix));
    timestamp_osd_ref #(.OSD_X(TS_X), .OSD_Y(8)) ref_ts (
        .pix_clk(clk), .pix_rst_n(rst_n), .enable(1'b1), .vsync(vs_n), .de(de),
        .pixel_x(h[9:0]), .pixel_y({2'b00, pixel_y}), .rgb_in(rgb_in), .rgb_out(ref_ts_rgb),
        .ts_active(ref_ts_box), .sys_rst_n(1'b1));
    initial force ref_ts.sec_cnt = (dut_ts.mm_tens * 10 + dut_ts.mm_ones) * 60 + dut_ts.ss_tens * 10 + dut_ts.ss_ones;


    reg [1:0] g_b = 3; reg [7:0] g_v = 255; reg [8:0] g_p = 50; reg g_a = 0;
    reg g_rv = 0; integer g_rw = 0, g_rh = 0;    
    integer cov_w1 = 0, cov_w2 = 0, cov_w3 = 0, cov_w4 = 0, cov_h1 = 0, cov_h2 = 0, cov_h3 = 0, cov_h4 = 0, cov_inv = 0;
    reg exp_changed = 0;
    always @(posedge clk) if (rst_n && frame_start) begin
        exp_changed <= (bright != g_b) || (vol != g_v) || (prd != g_p) || (auto_en != g_a);
        g_b <= bright; g_v <= vol; g_p <= prd; g_a <= auto_en;
        g_rv <= res_v; g_rw <= res_w; g_rh <= res_h;
        if (!res_v) cov_inv = cov_inv + 1;
        else begin
            if (res_w >= 1000) cov_w4 = cov_w4 + 1; else if (res_w >= 100) cov_w3 = cov_w3 + 1; else if (res_w >= 10) cov_w2 = cov_w2 + 1; else cov_w1 = cov_w1 + 1;
            if (res_h >= 1000) cov_h4 = cov_h4 + 1; else if (res_h >= 100) cov_h3 = cov_h3 + 1; else if (res_h >= 10) cov_h2 = cov_h2 + 1; else cov_h1 = cov_h1 + 1;
        end
    end

    function integer p10; input integer n; begin p10 = (n == 0) ? 1 : (n == 1) ? 10 : (n == 2) ? 100 : 1000; end endfunction
    function integer ndig; input integer n; begin ndig = (n >= 1000) ? 4 : (n >= 100) ? 3 : (n >= 10) ? 2 : 1; end endfunction
    // 宽：4 格右对齐（前导空白）；高：4 格左对齐（右补空白）；无效：----X----
    function [7:0] res_ch;
        input integer c;
        integer pos, pl;
        begin
            if (c == 29) res_ch = "X";
            else if (!g_rv) res_ch = "-";
            else if (c <= 28) begin
                pos = c - 25; pl = 3 - pos;
                res_ch = (pl > 0 && g_rw < p10(pl)) ? " " : dig((g_rw / p10(pl)) % 10);
            end else begin
                pos = c - 30;
                res_ch = (pos >= ndig(g_rh)) ? " " : dig((g_rh / p10(ndig(g_rh) - 1 - pos)) % 10);
            end
        end
    endfunction

    function [7:0] dig; input integer d; begin dig = "0" + d; end endfunction
    function [7:0] g_char;
        input integer c;
        integer bp;
        begin
            bp = (g_b == 3) ? 100 : (g_b == 2) ? 75 : (g_b == 1) ? 50 : 25;
            case (c)
                0: g_char = "B"; 1: g_char = "R"; 2: g_char = "T";
                3: g_char = dig(bp / 100); 4: g_char = dig((bp / 10) % 10); 5: g_char = dig(bp % 10);
                6: g_char = " "; 7: g_char = "V"; 8: g_char = "O"; 9: g_char = "L";
                10: g_char = dig(g_v / 100); 11: g_char = dig((g_v / 10) % 10); 12: g_char = dig(g_v % 10);
                13: g_char = " "; 14: g_char = "P";
                15: g_char = dig(g_p / 100); 16: g_char = dig((g_p / 10) % 10); 17: g_char = ".";
                18: g_char = dig(g_p % 10); 19: g_char = "S"; 20: g_char = " ";
                21: g_char = "M"; 22: g_char = ":"; 23: g_char = g_a ? "A" : "M";
                24: g_char = " ";
                default: g_char = (c >= 25 && c <= 33) ? res_ch(c) : " ";
            endcase
        end
    endfunction

    reg        exp_param;
    reg [7:0]  ch;
    reg [4:0]  gl;
    integer    cx, cc, cr;
    always @* begin
        exp_param = 1'b0;
        if (de && v >= PY && v < PY + 16 && h >= PX && h < PX + 408) begin
            cx = (h - PX) / 12; cc = ((h - PX) % 12) / 2; cr = (v - PY) / 2;
            ch = g_char(cx);
            gl = ref_u.glyph_row(ch, cr[2:0]);
            if (cc < 5 && cr < 7) exp_param = gl[4 - cc];
        end
    end

    integer fs_age = 0;
    always @(posedge clk) fs_age <= frame_start ? 0 : (fs_age < 1000 ? fs_age + 1 : fs_age);

    // ---- 逐像素比对 ----
    reg fs_q = 0;
    integer scroll_wraps = 0;
    always @(posedge clk) begin
        fs_q <= frame_start;
        if (!rst_n) begin
            errors <= 0; frames <= 0; checked <= 0;
        end else begin
            if (frame_start) frames <= frames + 1;
            if (frame_start && ref_u.scroll_pos == 10'd0 && frames > 3 && ref_u.scroll_div == 2'd0) scroll_wraps = scroll_wraps + 1;
            if (fs_q && frames >= 2 && dut_changed !== exp_changed) begin
                if (errors < 10) $display("FAIL[%0d] param_changed=%b exp=%b frame %0d", H_ACT, dut_changed, exp_changed, frames);
                errors <= errors + 1;
            end
            if (frames >= 2) begin
                if (!de && (dut_active || dut_ts_pix)) begin
                    if (errors < 10) $display("FAIL[%0d] overlay outside de h=%0d v=%0d", H_ACT, h, v);
                    errors <= errors + 1;
                end
                if (de) begin
                    checked <= checked + 1;
                    if (v >= PY && v < PY + 16 && h >= PX && h < PX + 408) begin
                        if (dut_param !== exp_param || dut_rgb !== (exp_param ? 24'hfff080 : rgb_in) ||
                            (h < PX + 14 * 12 && dut_param !== ref_u.param_pixel)) begin
                            if (errors < 10) $display("FAIL[%0d] param h=%0d v=%0d dut=%b exp=%b ref=%b char='%s' frame %0d",
                                                      H_ACT, h, v, dut_param, exp_param, ref_u.param_pixel, ch, frames);
                            errors <= errors + 1;
                        end
                    end else begin
                        if (dut_param !== 1'b0 || dut_rgb !== ref_rgb ||
                            dut_sub !== ref_u.subtitle_pixel || dut_band !== ref_u.subtitle_band_pixel) begin
                            if (errors < 10) $display("FAIL[%0d] subtitle h=%0d v=%0d sub %b/%b band %b/%b rgb %h/%h scroll=%0d frame %0d",
                                                      H_ACT, h, v, dut_sub, ref_u.subtitle_pixel, dut_band, ref_u.subtitle_band_pixel,
                                                      dut_rgb, ref_rgb, ref_u.scroll_pos, frames);
                            errors <= errors + 1;
                        end
                    end
                    if (dut_ts_rgb !== ref_ts_rgb || dut_ts_box !== ref_ts_box) begin
                        if (errors < 10) $display("FAIL[%0d] timestamp h=%0d v=%0d rgb %h/%h box %b/%b", H_ACT, h, v,
                                                  dut_ts_rgb, ref_ts_rgb, dut_ts_box, ref_ts_box);
                        errors <= errors + 1;
                    end
                end
            end
        end
    end
endmodule
