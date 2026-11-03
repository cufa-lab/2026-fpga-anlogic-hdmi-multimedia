`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// tb_video_transition_pixel —— 像素级转场分模块验证（真实 640x480@60 时序，顶层同参）
//   video_timing_ctrl(ACTIVE_LAST=1, VS_ALIGN_HSYNC=1) → video_transition_pixel
//   DUT_A：顶层配置（FADE_FRAMES=16, PIPELINE=0）
//   DUT_8：FADE_FRAMES=8（溶解档位 lvl = ramp×2）
//   DUT_P：PIPELINE=1（检查 rgb 与 hs/vs/de 整组延迟 1 拍）
//   每帧在 frame_start 时换一组控制量（alpha / dissolve / ramp / seed），逐像素与参考模型比对：
//     P1 Alpha 乘法：alpha ∈ {0,1,16,127,128,129,240,254,255}；out = alpha==255 ? in : in*alpha>>8
//     P2 方块溶解：块序号取自 Python 模型生成的金表 sim/ref/dissolve_blk_idx.hex（独立于 RTL 的实现，
//        地址 {seed, y>>5 - 1, x>>5 - 5}）；序号 < lvl 的块显示，其余黑。seed 0/5/9/15 × ramp 0..16，
//        ramp=0 全黑、ramp=16 全显；每档点亮像素数单调递增；alpha≠255 时显示块仍按 alpha 压暗
//     P3 不同 seed 的点亮集合不同（每次换图亮灭顺序不同）
//     P4 PIPELINE=0：hs/vs/de 直通；PIPELINE=1：hs/vs/de/rgb 全部比输入晚 1 拍且彼此对齐
//     P5 de=0 期间不检查 rgb（下游 rgb_video_adapter 消隐清零），但 hs/vs/de 每拍都比
// -----------------------------------------------------------------------------
module tb_video_transition_pixel;
    reg clk = 0, rst_n = 0;
    always #20 clk = ~clk;

    wire raw_hs, raw_vs, raw_de;
    wire [11:0] raw_x, raw_y;
    video_timing_ctrl #(
        .H_ACT(640), .H_FP(16), .H_SYNC(96), .H_BP(48),
        .V_ACT(480), .V_FP(10), .V_SYNC(2), .V_BP(33),
        .ACTIVE_LAST(1), .VS_ALIGN_HSYNC(1)
    ) u_timing (.pclk(clk), .rst_n(rst_n), .hsync(raw_hs), .vsync(raw_vs), .de(raw_de), .x(raw_x), .y(raw_y));

    // 帧边界（同顶层 frame_start_raw）
    reg vs_q = 1;
    always @(posedge clk) vs_q <= raw_vs;
    wire frame_start = vs_q && !raw_vs;

    // 测试图：随坐标变化，覆盖全 0..255；最低位置 1 保证不是纯黑（黑 = 被溶解隐去）
    wire [9:0]  col = raw_x - 12'd160;
    wire [9:0]  row = raw_y - 12'd45;
    wire [23:0] new_rgb = raw_de ? {col[7:0] ^ row[7:0], col[9:2], (row[8:1] + col[3:0]) | 8'h01} : 24'h0;

    reg  [7:0] alpha = 255;
    reg        dis   = 0;
    reg  [4:0] ramp  = 0;
    reg  [3:0] seed  = 0;

    wire        a_hs, a_vs, a_de, e_hs, e_vs, e_de, p_hs, p_vs, p_de;
    wire [23:0] a_rgb, e_rgb, p_rgb;
    video_transition_pixel #(.FADE_FRAMES(16), .PIPELINE(0)) dut_a (
        .clk(clk), .rst_n(rst_n), .hs_in(raw_hs), .vs_in(raw_vs), .de_in(raw_de), .x_in(raw_x), .y_in(raw_y),
        .rgb_in(new_rgb), .alpha(alpha), .dissolve_active(dis), .ramp(ramp), .seed(seed),
        .hs_out(a_hs), .vs_out(a_vs), .de_out(a_de), .rgb_out(a_rgb));
    video_transition_pixel #(.FADE_FRAMES(8), .PIPELINE(0)) dut_8 (
        .clk(clk), .rst_n(rst_n), .hs_in(raw_hs), .vs_in(raw_vs), .de_in(raw_de), .x_in(raw_x), .y_in(raw_y),
        .rgb_in(new_rgb), .alpha(alpha), .dissolve_active(dis), .ramp(ramp >> 1), .seed(seed),
        .hs_out(e_hs), .vs_out(e_vs), .de_out(e_de), .rgb_out(e_rgb));
    video_transition_pixel #(.FADE_FRAMES(16), .PIPELINE(1)) dut_p (
        .clk(clk), .rst_n(rst_n), .hs_in(raw_hs), .vs_in(raw_vs), .de_in(raw_de), .x_in(raw_x), .y_in(raw_y),
        .rgb_in(new_rgb), .alpha(alpha), .dissolve_active(dis), .ramp(ramp), .seed(seed),
        .hs_out(p_hs), .vs_out(p_vs), .de_out(p_de), .rgb_out(p_rgb));

    // ---------------- 参考模型 ----------------
    reg [3:0] gold [0:8191];
    initial $readmemh("sim/ref/dissolve_blk_idx.hex", gold);

    function [7:0] fade8(input [7:0] c, input [7:0] a);
        reg [15:0] p;
        begin p = c * a; fade8 = (a == 8'd255) ? c : p[15:8]; end
    endfunction
    function [23:0] fade24(input [23:0] c, input [7:0] a);
        fade24 = {fade8(c[23:16], a), fade8(c[15:8], a), fade8(c[7:0], a)};
    endfunction
    function [3:0] blk(input [3:0] s, input [11:0] x, input [11:0] y);
        reg [4:0] bx; reg [3:0] by;
        begin bx = x[9:5] - 5'd5; by = y[9:5] - 5'd1; blk = gold[{s, by, bx}]; end
    endfunction
    function [23:0] model(input [23:0] nrgb, input [7:0] a, input d, input [4:0] lvl, input [3:0] s,
                          input [11:0] x, input [11:0] y);
        model = (d && !(blk(s, x, y) < lvl)) ? 24'h0 : fade24(nrgb, a);
    endfunction

    integer errors = 0, checked = 0, lit_cnt;
    reg [23:0] exp_a, exp_e;
    reg [23:0] exp_p;          // PIPELINE=1：上一拍的期望
    reg        hs_d, vs_d, de_d;
    reg        armed = 0;
    reg [7:0]  alpha_cur; reg dis_cur; reg [4:0] ramp_cur; reg [3:0] seed_cur;  // 本帧生效的控制量
    // P3：记录 seed 0 / seed 9 在 ramp=8 时的点亮图（20×16 块）
    reg        map0 [0:319];
    reg        map9 [0:319];
    always @(posedge clk) begin
        if (armed) begin
            exp_a = model(new_rgb, alpha_cur, dis_cur, ramp_cur, seed_cur, raw_x, raw_y);
            exp_e = model(new_rgb, alpha_cur, dis_cur, {ramp_cur[4:1], 1'b0}, seed_cur, raw_x, raw_y);
            if (a_hs !== raw_hs || a_vs !== raw_vs || a_de !== raw_de) begin
                errors = errors + 1; if (errors < 10) $display("FAIL: P4 comb sync not passthrough @%0t", $time);
            end
            if (raw_de) begin
                checked = checked + 1;
                if (a_rgb !== exp_a) begin errors = errors + 1;
                    if (errors < 10) $display("FAIL: dut_a c=%0d r=%0d a=%0d d=%0d rp=%0d s=%0d got %h exp %h", col, row, alpha_cur, dis_cur, ramp_cur, seed_cur, a_rgb, exp_a); end
                if (e_rgb !== exp_e) begin errors = errors + 1;
                    if (errors < 10) $display("FAIL: dut_8 c=%0d r=%0d rp=%0d s=%0d got %h exp %h", col, row, ramp_cur >> 1, seed_cur, e_rgb, exp_e); end
                if (a_rgb !== 24'h0) lit_cnt = lit_cnt + 1;
                if (dis_cur && ramp_cur == 8 && raw_x[4:0] == 5'd16 && raw_y[4:0] == 5'd16) begin
                    if (seed_cur == 0) map0[{raw_y[8:5] - 4'd1, 4'd0} + {raw_y[8:5] - 4'd1, 2'd0} + (raw_x[9:5] - 5'd5)] = (a_rgb !== 24'h0);
                    if (seed_cur == 9) map9[{raw_y[8:5] - 4'd1, 4'd0} + {raw_y[8:5] - 4'd1, 2'd0} + (raw_x[9:5] - 5'd5)] = (a_rgb !== 24'h0);
                end
            end
            // PIPELINE=1：输出 = 上一拍输入
            if (p_hs !== hs_d || p_vs !== vs_d || p_de !== de_d) begin
                errors = errors + 1; if (errors < 10) $display("FAIL: P4 pipelined sync misaligned @%0t", $time);
            end
            if (de_d && p_rgb !== exp_p) begin
                errors = errors + 1; if (errors < 10) $display("FAIL: P4 pipelined rgb got %h exp %h", p_rgb, exp_p);
            end
        end
        hs_d <= raw_hs; vs_d <= raw_vs; de_d <= raw_de;
        exp_p <= armed ? model(new_rgb, alpha_cur, dis_cur, ramp_cur, seed_cur, raw_x, raw_y) : 24'h0;
    end

    // 在 frame_start 换控制量（控制器只在帧边界变化），DUT 内部寄存 1 拍，消隐期 45 行足够
    task frame(input [7:0] a, input d, input [4:0] rp, input [3:0] s);
        begin
            @(posedge clk); while (!frame_start) @(posedge clk);
            alpha <= a; dis <= d; ramp <= rp; seed <= s;
            repeat (3) @(posedge clk);
            alpha_cur = a; dis_cur = d; ramp_cur = rp; seed_cur = s;
            armed = 1;
            lit_cnt = 0;
        end
    endtask
    task end_frame; begin @(posedge clk); while (!frame_start) @(posedge clk); end endtask

    integer i, k, si, prev_lit, ndiff;
    reg [7:0] alist [0:8];
    reg [3:0] slist [0:3];
    initial begin
        alist[0] = 0; alist[1] = 1; alist[2] = 16; alist[3] = 127; alist[4] = 128;
        alist[5] = 129; alist[6] = 240; alist[7] = 254; alist[8] = 255;
        slist[0] = 0; slist[1] = 5; slist[2] = 9; slist[3] = 15;
        repeat (4) @(posedge clk); rst_n = 1;

        // P1 Alpha
        for (i = 0; i < 9; i = i + 1) frame(alist[i], 1'b0, 5'd0, 4'd0);
        frame(8'd255, 1'b0, 5'd0, 4'd0);
        $display("INFO: P1 alpha 0/1/16/127/128/129/240/254/255 frames checked (%0d px so far)", checked);

        // P2 方块溶解：每个 seed 扫 ramp 0..16，点亮像素数单调不减，0 档全黑、16 档全显
        for (si = 0; si < 4; si = si + 1) begin
            prev_lit = -1;
            for (k = 0; k <= 16; k = k + 1) begin
                frame(8'd255, 1'b1, k[4:0], slist[si]);
                end_frame;
                if (lit_cnt < prev_lit) begin errors = errors + 1; $display("FAIL: P2 seed=%0d ramp=%0d lit %0d < previous %0d", slist[si], k, lit_cnt, prev_lit); end
                if (k == 0 && lit_cnt != 0)          begin errors = errors + 1; $display("FAIL: P2 ramp 0 not all black (%0d lit)", lit_cnt); end
                if (k == 16 && lit_cnt != 640 * 480) begin errors = errors + 1; $display("FAIL: P2 ramp 16 not all shown (%0d lit)", lit_cnt); end
                if (si == 0 && (k == 4 || k == 8 || k == 12))
                    $display("INFO: P2 seed=0 ramp=%0d: %0d / 307200 px shown (%0d%%)", k, lit_cnt, lit_cnt * 100 / 307200);
                prev_lit = lit_cnt;
            end
        end
        // 溶解 + 压暗同时生效（模型逐像素已比）
        frame(8'd128, 1'b1, 5'd9, 4'd3);
        frame(8'd255, 1'b0, 5'd9, 4'd3);         // dissolve=0 时 ramp/seed 不影响画面
        end_frame;
        $display("INFO: P2 dissolve 4 seeds x ramp 0..16 matched golden table; monotonic, 0=black, 16=full");

        // P3 不同 seed 点亮集合不同
        ndiff = 0;
        for (i = 0; i < 320; i = i + 1) if (map0[i] !== map9[i]) ndiff = ndiff + 1;
        if (ndiff < 60) begin errors = errors + 1; $display("FAIL: P3 seed 0 vs 9 differ in only %0d / 320 blocks", ndiff); end
        else $display("INFO: P3 seed 0 vs 9 at ramp 8: %0d / 320 blocks differ", ndiff);

        if (errors == 0) $display("PASS: tb_video_transition_pixel (%0d active pixels x 3 DUTs)", checked);
        else $display("FAIL: tb_video_transition_pixel errors=%0d", errors);
        $finish;
    end
endmodule
