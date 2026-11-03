`timescale 1ns/1ps

module tb_osd_mixer_pipe;
    localparam integer H_ACT = 640, H_TOT = 700, V_ACT = 140, V_TOT = 150;
    reg clk = 0, rst_n = 0;
    always #20 clk = ~clk;

    // ---- 简化时序：每行 640 有效 + 60 消隐，每帧 140 行 + 10 行消隐（vsync 低 2 行）----
    integer hc = 0, vc = 0;
    reg de = 0, vsync = 1;
    always @(posedge clk) begin
        hc <= (hc == H_TOT - 1) ? 0 : hc + 1;
        if (hc == H_TOT - 1) vc <= (vc == V_TOT - 1) ? 0 : vc + 1;
        de    <= (hc < H_ACT) && (vc < V_ACT);
        vsync <= !((vc >= V_ACT + 4) && (vc < V_ACT + 6));
    end

    reg [23:0] rgb_in = 0;
    reg        ext_hit = 0, txt_hit = 0;
    reg [23:0] ext_color = 0, txt_color = 0;
    reg [7:0]  ext_alpha = 0, txt_alpha = 0;
    reg [7:0]  bri = 8'd200, vol = 8'd128;
    reg [8:0]  prd = 9'd50;
    reg        param_show = 0, param_hide = 0;
    integer    r;
    always @(posedge clk) begin
        rgb_in <= $urandom;
        r = $urandom % 100;
        ext_hit   <= (r < 15);
        ext_color <= $urandom;
        r = $urandom % 4;
        ext_alpha <= (r == 0) ? 8'd0 : (r == 1) ? 8'd255 : $urandom;
        r = $urandom % 100;
        txt_hit   <= (r < 20);
        txt_color <= $urandom;
        r = $urandom % 4;
        txt_alpha <= (r == 0) ? 8'd0 : (r == 1) ? 8'd255 : $urandom;
    end

    wire [23:0] out0, out1;
    wire        ov0, ov1;
    wire [9:0]  px0, py0, px1, py1;
    wire        fs0, fs1;

    osd_layer_mixer #(.PARAM_Y(82), .SUBTITLE_EN(0), .TS_ENABLE(0), .PARAM_HOLD_FRAMES(6), .FADE_STEP(8'd64), .PIPE(0)) u0 (
        .clk(clk), .rst_n(rst_n), .vsync(vsync), .de(de), .rgb_in(rgb_in),
        .subtitle_enable(1'b0), .subtitle_text(96'd0), .subtitle_length(6'd0),
        .brightness(bri), .volume(vol), .period_tenths(prd), .auto_enabled(1'b1),
        .res_valid(1'b1), .res_width(12'd640), .res_height(12'd480),
        .param_show(param_show), .param_hide(param_hide),
        .ext_hit(ext_hit), .ext_color(ext_color), .ext_alpha(ext_alpha),
        .txt_hit(txt_hit), .txt_color(txt_color), .txt_alpha(txt_alpha),
        .pix_x(px0), .pix_y(py0), .pix_frame_start(fs0), .rgb_out(out0), .overlay_active(ov0));
    osd_layer_mixer #(.PARAM_Y(82), .SUBTITLE_EN(0), .TS_ENABLE(0), .PARAM_HOLD_FRAMES(6), .FADE_STEP(8'd64), .PIPE(1)) u1 (
        .clk(clk), .rst_n(rst_n), .vsync(vsync), .de(de), .rgb_in(rgb_in),
        .subtitle_enable(1'b0), .subtitle_text(96'd0), .subtitle_length(6'd0),
        .brightness(bri), .volume(vol), .period_tenths(prd), .auto_enabled(1'b1),
        .res_valid(1'b1), .res_width(12'd640), .res_height(12'd480),
        .param_show(param_show), .param_hide(param_hide),
        .ext_hit(ext_hit), .ext_color(ext_color), .ext_alpha(ext_alpha),
        .txt_hit(txt_hit), .txt_color(txt_color), .txt_alpha(txt_alpha),
        .pix_x(px1), .pix_y(py1), .pix_frame_start(fs1), .rgb_out(out1), .overlay_active(ov1));

    // ---- 比对：PIPE=0 输出延迟 2 拍 ----
    reg [23:0] o0_d1, o0_d2;
    reg        v0_d1, v0_d2;
    integer    cyc = 0, errors = 0, checks = 0;
    integer    n_param_txt = 0, n_param_box = 0, n_ext = 0, n_txt = 0, n_txt_on_box = 0, n_txt_mid = 0;
    integer    n_hold = 0, n_hold_bad = 0;
    reg [55:0] t_prev;
    reg        txt_prev;
    always @(posedge clk) begin
        o0_d1 <= out0;  o0_d2 <= o0_d1;
        v0_d1 <= ov0;   v0_d2 <= v0_d1;
        cyc   <= cyc + 1;
        if (rst_n && cyc > 4) begin
            checks = checks + 1;
            if (out1 !== o0_d2 || ov1 !== v0_d2) begin
                if (errors < 10) $display("FAIL cyc=%0d x=%0d y=%0d pipe=%h/%b comb_d2=%h/%b", cyc, px0, py0, out1, ov1, o0_d2, v0_d2);
                errors = errors + 1;
            end
            if (px0 != px1 || py0 != py1 || fs0 != fs1) begin
                if (errors < 10) $display("FAIL cyc=%0d coord/frame_start differ", cyc);
                errors = errors + 1;
            end
            // 覆盖统计（看 PIPE=0 实例内部的图层选择）
            if (u0.param_pixel && u0.param_vis) n_param_txt = n_param_txt + 1;
            if (u0.param_box && u0.param_vis && !u0.param_pixel) n_param_box = n_param_box + 1;
            if (de && ext_hit && !(u0.param_box && u0.param_vis)) n_ext = n_ext + 1;
            if (de && txt_hit) n_txt = n_txt + 1;
            if (de && txt_hit && u0.layer_hit && u0.layer_alpha != 0) n_txt_on_box = n_txt_on_box + 1;
            if (de && txt_hit && txt_alpha != 0 && txt_alpha != 255) n_txt_mid = n_txt_mid + 1;
            // P9：上一拍和这一拍都没有 txt_hit → 操作数寄存器不变
            if (!txt_prev && !txt_hit) begin
                n_hold = n_hold + 1;
                if ({u1.g_pipe.t_bg, u1.g_pipe.t_color, u1.g_pipe.t_alpha} !== t_prev) n_hold_bad = n_hold_bad + 1;
            end
        end
        txt_prev <= txt_hit;
        t_prev   <= {u1.g_pipe.t_bg, u1.g_pipe.t_color, u1.g_pipe.t_alpha};
    end

    initial begin
        repeat (10) @(posedge clk);
        rst_n = 1;
        // 帧 0..: 参数条上电显示 6 帧后淡出；中途改参数 / param_show / param_hide
        repeat (H_TOT * V_TOT * 3) @(posedge clk);
        bri = 8'd17;  vol = 8'd255; prd = 9'd300;
        repeat (H_TOT * V_TOT * 2) @(posedge clk);
        param_hide = 1;
        repeat (H_TOT * V_TOT * 2) @(posedge clk);
        param_hide = 0;
        @(posedge clk) param_show = 1;
        @(posedge clk) param_show = 0;
        repeat (H_TOT * V_TOT * 9) @(posedge clk);
        $display("INFO checks=%0d param_txt=%0d param_box=%0d ext=%0d txt=%0d txt_on_layer=%0d txt_mid_alpha=%0d hold=%0d hold_bad=%0d",
                 checks, n_param_txt, n_param_box, n_ext, n_txt, n_txt_on_box, n_txt_mid, n_hold, n_hold_bad);
        if (n_param_txt == 0 || n_param_box == 0 || n_ext == 0 || n_txt_on_box == 0 || n_txt_mid == 0 || n_hold == 0) begin
            $display("FAIL coverage hole"); errors = errors + 1;
        end
        if (n_hold_bad != 0) begin
            $display("FAIL P9 operand regs toggled without txt_hit"); errors = errors + 1;
        end
        if (errors == 0) $display("PASS tb_osd_mixer_pipe: PIPE=1 == PIPE=0 delayed 2 cycles (%0d cycles), P9 operand hold", checks);
        else             $display("FAIL tb_osd_mixer_pipe: %0d errors", errors);
        $finish;
    end
endmodule
