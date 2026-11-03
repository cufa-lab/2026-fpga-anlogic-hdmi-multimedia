`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// tb_thumb_fullgrid_view —— 全屏 2×2 总览显示侧（与顶层同参数）
//   thumb_view_mixer  CELL_W/H = 320/240、PLACEHOLDER=1、EMPTY_LEVEL=0x40
//   thumb_zoom_transition FULL_GRID=1
// 逐像素扫 640×480 对参考模型：
//   V1 四格铺满全屏：格子 = {y≥240, x≥320}；已加载格 = 拼图像素，焦点黄框只在焦点格最外 2 像素（每格 2224 点）
//   V2 有图未加载 = 呼吸占位（外框 ph_level、格内 ph_level/2）
//   V3 空位（本页没有图，如共 10 张的第 2 页格子 2/3）= 静态暗灰：外框 0x40、格内 0x10；与占位可区分
//   V4 全屏（thumb_display=0）逐位直通；网格内没有黑色空白区
//   V5 裁剪：clip_en 时矩形外黑
//   V6 缩放 FULL_GRID：拨上 24 帧，最小矩形 = 焦点格 320×240（4 个焦点各一次），首尾为全屏，边界单调
// -----------------------------------------------------------------------------
module tb_thumb_fullgrid_view;
    // ---------------- mixer ----------------
    reg  [11:0] x = 0, y = 0;
    reg         de = 1, thumb = 1, clip_en = 0;
    reg  [11:0] cx0 = 0, cx1 = 0, cy0 = 0, cy1 = 0;
    reg  [1:0]  focus = 0;
    reg  [3:0]  cpl = 0, ex = 0;
    reg  [7:0]  lvl = 8'h50;
    wire [9:0]  px = x - 160, py = y - 45;
    wire [23:0] rgb_in = {px[7:0], py[7:0], 8'hA5 ^ px[9:2]};
    wire [23:0] out;
    wire        tde, fb, ph, em;
    wire [1:0]  si;

    thumb_view_mixer #(.FOCUS_RGB(24'hFFD000), .PLACEHOLDER(1), .CELL_W(320), .CELL_H(240), .EMPTY_LEVEL(8'h40)) dm (
        .de_in(de), .x_in(x), .y_in(y), .rgb_in(rgb_in), .thumb_display(thumb), .focus_index(focus),
        .image_complete(cpl), .clip_en(clip_en), .clip_x0(cx0), .clip_x1(cx1), .clip_y0(cy0), .clip_y1(cy1),
        .cell_exists(ex), .ph_level(lvl), .rgb_out(out), .thumb_de(tde), .source_index(si),
        .focus_border(fb), .placeholder(ph), .empty_slot(em));

    integer errors = 0;
    task fail(input [8*72-1:0] msg); begin errors = errors + 1; if (errors < 30) $display("FAIL %0s x=%0d y=%0d cpl=%b ex=%b f=%0d @%0t", msg, px, py, cpl, ex, focus, $time); end endtask

    integer i, j, lx, ly, cel, n_focus, n_ph, n_em, n_black;
    reg bord, inr;
    reg [23:0] e;
    task scan; begin
        n_focus = 0; n_ph = 0; n_em = 0; n_black = 0;
        for (j = 0; j < 480; j = j + 1) for (i = 0; i < 640; i = i + 1) begin
            x = i + 160; y = j + 45; #1;
            cel  = (j >= 240 ? 2 : 0) + (i >= 320 ? 1 : 0);
            lx   = i - (i >= 320 ? 320 : 0);
            ly   = j - (j >= 240 ? 240 : 0);
            bord = (lx < 2) || (ly < 2) || (lx >= 318) || (ly >= 238);
            inr  = !clip_en || ((x >= cx0) && (x < cx1) && (y >= cy0) && (y < cy1));
            if (!thumb)               e = rgb_in;
            else if (cpl[cel] && ex[cel]) begin
                e = (cel == focus && bord) ? 24'hFFD000 : rgb_in;
                if (cel == focus && bord) n_focus = n_focus + 1;
            end else if (cpl[cel])    e = (cel == focus && bord) ? 24'hFFD000 : rgb_in;   // 不会出现：有缩略图必有图
            else if (ex[cel]) begin   e = bord ? {3{lvl}} : {3{1'b0, lvl[7:1]}}; n_ph = n_ph + 1; end
            else begin                e = bord ? {3{8'h40}} : {3{8'h10}}; n_em = n_em + 1; end
            if (!inr) e = 24'h0;
            if (out === 24'h0) n_black = n_black + 1;
            if (out !== e) fail("V pixel mismatch");
            if (thumb) begin
                if (si !== cel[1:0]) fail("V1 source_index");
                if (tde !== cpl[cel]) fail("V1 thumb_de");
                if (ph !== (!cpl[cel] && ex[cel])) fail("V2 placeholder flag");
                if (em !== !ex[cel]) fail("V3 empty_slot flag");
            end
        end
    end endtask

    // ---------------- zoom ----------------
    reg clk = 0, rst_n = 0;
    always #20 clk = ~clk;
    reg frame_start = 0, sw = 0;
    reg [1:0] zf = 0;
    wire thumb_display, zoom_active;
    wire [7:0] alpha;
    wire [11:0] x0, x1, y0, y1;
    thumb_zoom_transition #(.FULL_GRID(1)) dz (.clk(clk), .rst_n(rst_n), .frame_start(frame_start), .display_valid(1'b1),
        .sw_thumb(sw), .focus(zf), .thumb_display(thumb_display), .zoom_active(zoom_active),
        .zoom_alpha(alpha), .clip_x0(x0), .clip_x1(x1), .clip_y0(y0), .clip_y1(y1));
    task zframe; begin repeat (30) @(posedge clk); #1 frame_start = 1; @(posedge clk); #1 frame_start = 0; @(posedge clk); #1; end endtask

    task run_zoom(input [1:0] f);
        integer k, area, min_area, mx0, mx1, my0, my1, px0, px1, py0, py1, n_act;
        begin
            zf = f; sw = 1; min_area = 1 << 30; n_act = 0;
            px0 = 160; px1 = 800; py0 = 45; py1 = 525;
            for (k = 0; k < 40; k = k + 1) begin
                zframe;
                if (zoom_active) begin
                    n_act = n_act + 1;
                    area = (x1 - x0) * (y1 - y0);
                    if (area < min_area) begin min_area = area; mx0 = x0; mx1 = x1; my0 = y0; my1 = y1; end
                    // 单调：收缩段边界只向焦点格靠，展开段只向外
                    if (!thumb_display && (x0 < px0 || x1 > px1 || y0 < py0 || y1 > py1)) fail("V6 shrink not monotonic");
                    px0 = x0; px1 = x1; py0 = y0; py1 = y1;
                end
            end
            if (!thumb_display || zoom_active) fail("V6 settled in overview");
            if (n_act != 24) begin fail("V6 24 animation frames"); $display("  n_act=%0d", n_act); end
            if (mx0 != 160 + (f[0] ? 320 : 0) || mx1 != mx0 + 320 || my0 != 45 + (f[1] ? 240 : 0) || my1 != my0 + 240) begin
                fail("V6 smallest rect = 320x240 focus cell"); $display("  f=%0d rect x %0d..%0d y %0d..%0d", f, mx0, mx1, my0, my1);
            end
            if (x0 != 160 || x1 != 800 || y0 != 45 || y1 != 525) fail("V6 ends full screen");
            sw = 0;
            for (k = 0; k < 40; k = k + 1) zframe;
            if (thumb_display || zoom_active) fail("V6 back to full screen");
        end
    endtask

    integer s, k;
    initial begin
        // V1 全部已加载，焦点逐格
        thumb = 1; cpl = 4'hF; ex = 4'hF;
        for (k = 0; k < 4; k = k + 1) begin
            focus = k; scan;
            if (n_focus != 2224) begin fail("V1 focus border count"); $display("  n_focus=%0d", n_focus); end
            if (n_black != 0) fail("V4 grid covers full screen (no black area)");
        end
        // V2 / V3：10 张图第 2 页：格子 0/1 有图（0 已好、1 加载中），2/3 空位
        cpl = 4'b0001; ex = 4'b0011; focus = 0; lvl = 8'h6E; scan;
        if (n_ph != 76800 || n_em != 2 * 76800) begin fail("V2/V3 counts"); $display("  ph=%0d em=%0d", n_ph, n_em); end
        // 只有 1 张图在本页
        cpl = 4'b0000; ex = 4'b0001; lvl = 8'h30; scan;
        if (n_em != 3 * 76800) fail("V3 three empty slots");
        // V5 裁剪
        cpl = 4'hF; ex = 4'hF; focus = 3; clip_en = 1; cx0 = 160 + 100; cx1 = 160 + 500; cy0 = 45 + 50; cy1 = 45 + 400; scan;
        clip_en = 0;
        // V4 全屏直通
        thumb = 0; cpl = 4'h0; ex = 4'h0; scan;

        // V6
        #100 rst_n = 1;
        for (k = 0; k < 3; k = k + 1) zframe;
        for (s = 0; s < 4; s = s + 1) run_zoom(s);

        if (errors == 0) $display("PASS tb_thumb_fullgrid_view");
        else $display("FAIL tb_thumb_fullgrid_view errors=%0d", errors);
        $finish;
    end
endmodule
