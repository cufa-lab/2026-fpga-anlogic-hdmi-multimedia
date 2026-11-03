`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// tb_thumb_zoom_transition —— SW4 全屏 ⇄ 总览缩放转场控制器（模块级）
// 检查：
//   Z1 帧边界锁存：所有输出只在 frame_start 的下一拍变化，帧内保持不变（不撕裂）
//   Z2 SW4 拨上：12 帧收缩（裁剪矩形全屏 → 焦点格，alpha 255 → 64 单调）+ 12 帧展开（格 → 全屏，alpha 64 → 255）；
//      thumb_display 恰好在矩形 = 焦点格那一帧变 1；结束后 zoom_active=0、alpha=255、thumb_display=1
//   Z3 SW4 拨下：同样 24 帧，矩形 = 焦点格那一帧仍是缩略图（thumb_display=1），下一帧起 0；结束回全屏直通
//   Z4 4 个焦点格：切换帧的矩形 = 该格 160×120（左上 / 右上 / 左下 / 右下）
//   Z5 缓动：矩形边界逐帧单调，首尾两帧步长小、切换附近步长大（ease-in-out），没有一帧跳变 > 1/3 行程
//   Z6 去抖：SW4 只抖 1 帧 → 不动作
//   Z7 动画中来回拨 → 当前动画走完后按开关位置补做，最终与开关一致
//   Z8 display_valid = 0 不进入总览；总览中 display_valid 掉 → 退回全屏
// -----------------------------------------------------------------------------
module tb_thumb_zoom_transition;
    reg clk = 0, rst_n = 0;
    always #20 clk = ~clk;
    reg frame_start = 0, display_valid = 1, sw = 0;
    reg [1:0] focus = 0;
    wire thumb_display, zoom_active;
    wire [7:0] alpha;
    wire [11:0] x0, x1, y0, y1;
    thumb_zoom_transition dut (.clk(clk), .rst_n(rst_n), .frame_start(frame_start), .display_valid(display_valid),
        .sw_thumb(sw), .focus(focus), .thumb_display(thumb_display), .zoom_active(zoom_active),
        .zoom_alpha(alpha), .clip_x0(x0), .clip_x1(x1), .clip_y0(y0), .clip_y1(y1));

    integer errors = 0;
    task fail(input [8*72-1:0] msg); begin errors = errors + 1; if (errors < 30) $display("FAIL %0s @%0t", msg, $time); end endtask

    // Z1：输出在帧内不变
    reg [48:0] snap; reg fs_d = 0;
    wire [48:0] now = {thumb_display, zoom_active, alpha, x0, x1, y0[8:0], y1[8:0]};
    always @(posedge clk) begin
        fs_d <= frame_start;
        if (rst_n && !fs_d && !frame_start && now !== snap) fail("Z1 output changed mid-frame");
        snap <= now;
    end

    // 每帧记录（frame_start 后第 2 拍采样）
    integer nf = 0;
    reg  [11:0] fx0 [0:63], fx1 [0:63], fy0 [0:63], fy1 [0:63];
    reg  [7:0]  fa [0:63];
    reg         ft [0:63], fz [0:63];
    task frame; begin
        repeat (30) @(posedge clk); #1 frame_start = 1; @(posedge clk); #1 frame_start = 0;
        @(posedge clk); #1;
        if (nf < 64) begin fx0[nf] = x0; fx1[nf] = x1; fy0[nf] = y0; fy1[nf] = y1; fa[nf] = alpha; ft[nf] = thumb_display; fz[nf] = zoom_active; end
        nf = nf + 1;
    end endtask

    function integer cell_l(input [1:0] f); cell_l = 160 + (f[0] ? 160 : 0); endfunction
    function integer cell_t(input [1:0] f); cell_t = 45 + (f[1] ? 120 : 0); endfunction
    function integer absd(input integer a, input integer b); absd = a > b ? a - b : b - a; endfunction

    // 一次完整转场：返回切换帧号、检查曲线
    task run_zoom(input to_thumb, input [1:0] f);
        integer i, sw_frame, min_a, min_a_at, step, maxstep, cell_at, last;
        begin
            focus = f; nf = 0;
            sw = to_thumb;
            for (i = 0; i < 32; i = i + 1) frame;
            // 找矩形 = 焦点格的帧
            cell_at = -1; min_a = 256; min_a_at = -1; sw_frame = -1;
            for (i = 0; i < 32; i = i + 1) begin
                if (fx0[i] == cell_l(f) && fx1[i] == cell_l(f) + 160 && fy0[i] == cell_t(f) && fy1[i] == cell_t(f) + 120) cell_at = i;
                if (fa[i] < min_a) begin min_a = fa[i]; min_a_at = i; end
                if (i > 0 && ft[i] != ft[i-1]) sw_frame = i;
            end
            if (cell_at < 0) fail("Z4 rect never equals focus cell");
            if (min_a != 64 || min_a_at != cell_at) fail("Z2 alpha minimum not 64 at the cell frame");
            if (to_thumb && sw_frame != cell_at) fail("Z2 thumb_display not switched at the cell frame");
            if (!to_thumb && sw_frame != cell_at + 1) fail("Z3 thumb_display not switched one frame after cell frame");
            // 单调：cell 之前收缩、之后展开
            maxstep = 0;
            for (i = 1; i < 32; i = i + 1) begin
                if (i <= cell_at) begin
                    if (fx0[i] < fx0[i-1] || fx1[i] > fx1[i-1] || fy0[i] < fy0[i-1] || fy1[i] > fy1[i-1]) fail("Z5 shrink not monotonic");
                    if (fa[i] > fa[i-1]) fail("Z2 alpha not decreasing");
                end else begin
                    if (fx0[i] > fx0[i-1] || fx1[i] < fx1[i-1] || fy0[i] > fy0[i-1] || fy1[i] < fy1[i-1]) fail("Z5 expand not monotonic");
                    if (fa[i] < fa[i-1]) fail("Z2 alpha not increasing");
                end
                step = absd(fx1[i], fx1[i-1]) > absd(fx0[i], fx0[i-1]) ? absd(fx1[i], fx1[i-1]) : absd(fx0[i], fx0[i-1]);
                if (step > maxstep) maxstep = step;
            end
            // 缓动：第一步、最后一步小，切换附近大；单帧最大跳变不超过行程的 1/3
            if (absd(fx1[2], fx1[1]) > 10 || absd(fx1[cell_at + 12], fx1[cell_at + 11]) > 10) fail("Z5 ease start/end step too large");
            if (absd(fx1[cell_at], fx1[cell_at - 1]) < 50) fail("Z5 step at switch too small (not ease-in-out)");
            if (maxstep > (f[0] ? 320 : 480) / 3) fail("Z5 single-frame jump too large");
            // 结束状态
            last = 31;
            if (fz[last] !== 1'b0 || fa[last] !== 8'd255 || ft[last] !== to_thumb) fail("Z2/Z3 end state");
            if (fx0[last] != 160 || fx1[last] != 800 || fy0[last] != 45 || fy1[last] != 525) fail("Z2/Z3 end rect not full screen");
            $display("INFO %0s focus %0d: cell frame %0d, thumb_display switch frame %0d, alpha min %0d, max x-step %0d px, animation frames %0d",
                to_thumb ? "full->thumb" : "thumb->full", f, cell_at, sw_frame, min_a, maxstep, cell_at + 12);
            $display("INFO   x1 per frame: %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d | %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d",
                fx1[1], fx1[2], fx1[3], fx1[4], fx1[5], fx1[6], fx1[7], fx1[8], fx1[9], fx1[10], fx1[11], fx1[12], fx1[13],
                fx1[14], fx1[15], fx1[16], fx1[17], fx1[18], fx1[19], fx1[20], fx1[21], fx1[22], fx1[23], fx1[24], fx1[25]);
        end
    endtask

    integer i, e0, f;
    initial begin
        #100 rst_n = 1;
        for (i = 0; i < 3; i = i + 1) frame;
        // Z6 去抖：只拨 1 帧
        e0 = errors;
        sw = 1; frame; sw = 0;
        for (i = 0; i < 30; i = i + 1) begin frame; if (zoom_active || thumb_display) fail("Z6 1-frame glitch started zoom"); end
        $display("INFO Z6 debounce errors %0d", errors - e0);
        // Z2/Z3/Z4/Z5：4 个焦点，各拨上 / 拨下一次
        e0 = errors;
        for (f = 0; f < 4; f = f + 1) begin
            run_zoom(1'b1, f);
            run_zoom(1'b0, f);
        end
        $display("INFO Z1-Z5 errors %0d", errors - e0);
        // Z7 动画中来回拨
        e0 = errors;
        focus = 1;
        sw = 1; for (i = 0; i < 6; i = i + 1) frame;
        sw = 0; for (i = 0; i < 80; i = i + 1) frame;
        if (thumb_display || zoom_active || alpha != 255) fail("Z7 did not settle to full");
        sw = 1; for (i = 0; i < 20; i = i + 1) frame;
        sw = 0; frame; frame; sw = 1;
        for (i = 0; i < 80; i = i + 1) frame;
        if (!thumb_display || zoom_active || alpha != 255) fail("Z7 did not settle to thumb");
        $display("INFO Z7 errors %0d", errors - e0);
        // Z8 display_valid
        e0 = errors;
        display_valid = 0;
        for (i = 0; i < 40; i = i + 1) frame;
        if (thumb_display) fail("Z8 still thumb when display_valid=0");
        sw = 1; for (i = 0; i < 40; i = i + 1) frame;
        if (thumb_display || zoom_active) fail("Z8 entered thumb while display_valid=0");
        display_valid = 1; for (i = 0; i < 40; i = i + 1) frame;
        if (!thumb_display) fail("Z8 did not enter thumb after display_valid");
        $display("INFO Z8 errors %0d", errors - e0);

        if (errors == 0) $display("PASS tb_thumb_zoom_transition");
        else $display("FAIL tb_thumb_zoom_transition errors=%0d", errors);
        $finish;
    end
endmodule
