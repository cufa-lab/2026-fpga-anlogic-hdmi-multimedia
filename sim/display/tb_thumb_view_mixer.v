`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// tb_thumb_view_mixer —— 总览 / 转场像素选择（模块级，真实 video_timing_ctrl 时序 800×525）
// 检查：
//   M1 全屏、不转场：rgb_out 与 rgb_in 逐位相等
//   M2 总览：网格内（且该格完成）= 拼图像素；焦点格外圈 2 像素 = FOCUS_RGB；右侧 x≥320 / 下方 y≥240 空白 = 黑
//   M3 image_complete 门控：未完成格子整格黑（含焦点格时不画边框）
//   M4 4 个焦点格各一帧：边框像素数 1104、位置正确
//   M5 裁剪矩形（转场）：矩形外黑、矩形内 = 未裁剪时的值；坐标偏移（ACTIVE_LAST=1 的 160/45）对齐到有效区第 0 行第 0 列
// -----------------------------------------------------------------------------
module tb_thumb_view_mixer;
    reg clk = 0, rst_n = 0;
    always #20 clk = ~clk;
    wire hs, vs, de; wire [11:0] x, y;
    video_timing_ctrl #(.ACTIVE_LAST(1), .VS_ALIGN_HSYNC(1)) u_vtc (.pclk(clk), .rst_n(rst_n), .hsync(hs), .vsync(vs), .de(de), .x(x), .y(y));

    reg  [23:0] rgb_in = 0;
    reg         thumb = 0, clip_en = 0;
    reg  [1:0]  focus = 0;
    reg  [3:0]  done = 4'hF;
    reg  [11:0] cx0 = 160, cx1 = 800, cy0 = 45, cy1 = 525;
    wire [23:0] rgb_out;
    wire        tde, border; wire [1:0] sidx;
    thumb_view_mixer #(.FOCUS_RGB(24'hFFD000)) dut (.de_in(de), .x_in(x), .y_in(y), .rgb_in(rgb_in),
        .thumb_display(thumb), .focus_index(focus), .image_complete(done),
        .clip_en(clip_en), .clip_x0(cx0), .clip_x1(cx1), .clip_y0(cy0), .clip_y1(cy1),
        .rgb_out(rgb_out), .thumb_de(tde), .source_index(sidx), .focus_border(border));

    integer errors = 0, seed = 11;
    integer px, py, n_de, n_border, n_lit;
    reg vs_q = 1;
    task fail(input [8*48-1:0] msg); begin errors = errors + 1; if (errors < 20)
        $display("FAIL %0s px%0d py%0d in %h out %h (thumb%b focus%0d done%b clip%b)", msg, px, py, rgb_in, rgb_out, thumb, focus, done, clip_en); end endtask

    function [23:0] expect_rgb(input integer px, input integer py, input [23:0] in);
        integer s, lx, ly; reg [23:0] b;
        begin
            s = (py >= 120 ? 2 : 0) + (px >= 160 ? 1 : 0); lx = px % 160; ly = py % 120;
            if (!thumb) b = in;
            else if (px >= 320 || py >= 240 || !done[s]) b = 0;
            else if (s == focus && (lx < 2 || ly < 2 || lx >= 158 || ly >= 118)) b = 24'hFFD000;
            else b = in;
            if (clip_en && !(px + 160 >= cx0 && px + 160 < cx1 && py + 45 >= cy0 && py + 45 < cy1)) b = 0;
            expect_rgb = b;
        end
    endfunction

    // 一整帧：de 期间逐像素比较；de 计数 → (px, py) 与 VTC 的有效区第 0 点对齐
    task one_frame;
        integer cnt; begin
            @(negedge vs); cnt = 0; n_de = 0; n_border = 0; n_lit = 0;
            while (cnt < 640 * 480) begin
                @(negedge clk);
                if (de) begin
                    px = cnt % 640; py = cnt / 640;
                    rgb_in = $random(seed) | 24'h010101;     // 非 0
                    #1;
                    if (x != px + 160 || y != py + 45) fail("offset (x/y vs active index)");
                    if (rgb_out !== expect_rgb(px, py, rgb_in)) fail("pixel");
                    if (border) n_border = n_border + 1;
                    if (rgb_out !== 0) n_lit = n_lit + 1;
                    cnt = cnt + 1;
                end
            end
        end endtask

    integer f, e0;
    initial begin
        #100 rst_n = 1;
        // M1
        e0 = errors; thumb = 0; clip_en = 0;
        one_frame;
        if (n_lit != 640 * 480) fail("M1 not all passthrough");
        $display("INFO M1 full-screen passthrough: %0d/307200 pixels bit-exact, errors %0d", n_lit, errors - e0);
        // M2/M4
        e0 = errors; thumb = 1; done = 4'hF;
        for (f = 0; f < 4; f = f + 1) begin
            focus = f; one_frame;
            if (n_border != 1104) begin errors = errors + 1; $display("FAIL M4 focus %0d border %0d", f, n_border); end
            if (n_lit != 76800) begin errors = errors + 1; $display("FAIL M2 lit pixels %0d (exp 76800 = 320x240)", n_lit); end
            $display("INFO M2/M4 focus %0d: lit %0d px (320x240), border %0d px", f, n_lit, n_border);
        end
        $display("INFO M2/M4 errors %0d", errors - e0);
        // M3
        e0 = errors; focus = 2; done = 4'b1010; one_frame;
        if (n_lit != 2 * 19200 || n_border != 0) begin errors = errors + 1; $display("FAIL M3 lit %0d border %0d", n_lit, n_border); end
        done = 4'b0100; one_frame;
        if (n_lit != 19200 || n_border != 1104) begin errors = errors + 1; $display("FAIL M3b lit %0d border %0d", n_lit, n_border); end
        $display("INFO M3 gating errors %0d", errors - e0);
        // M5 裁剪
        e0 = errors; done = 4'hF; focus = 3;
        clip_en = 1; cx0 = 160 + 100; cx1 = 160 + 420; cy0 = 45 + 30; cy1 = 45 + 400;
        thumb = 0; one_frame;
        if (n_lit != 320 * 370) begin errors = errors + 1; $display("FAIL M5 full clip lit %0d (exp %0d)", n_lit, 320 * 370); end
        thumb = 1; cx0 = 160 + 160; cx1 = 160 + 320; cy0 = 45 + 120; cy1 = 45 + 240; one_frame;   // = 焦点格
        if (n_lit != 19200) begin errors = errors + 1; $display("FAIL M5 cell clip lit %0d", n_lit); end
        $display("INFO M5 clip errors %0d", errors - e0);

        if (errors == 0) $display("PASS tb_thumb_view_mixer");
        else $display("FAIL tb_thumb_view_mixer errors=%0d", errors);
        $finish;
    end
endmodule
