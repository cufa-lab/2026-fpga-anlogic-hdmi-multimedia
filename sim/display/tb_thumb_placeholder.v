`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// tb_thumb_placeholder ——  加载状态显示（thumb_view_mixer PLACEHOLDER=1，纯组合）
// 逐像素扫 640×480（raw 坐标 = 屏幕坐标 + 160/45），与参考模型比较：
//   P1 四个区域的 source_index（左上 0 / 右上 1 / 左下 2 / 右下 3）与 thumb_de（只在已加载格子内）
//   P2 已加载格子 = 拼图像素；焦点黄框只在 焦点格 且 已加载 的最外 2 像素（每格 1104 点）
//   P3 有图未加载的格子 = 占位：最外 2 像素 ph_level 灰、格内 ph_level/2 灰，不出现旧图像素
//   P4 没有图的格子、网格外（右侧 / 下方）= 黑
//   P5 全屏（thumb_display=0）逐位直通
//   P6 PLACEHOLDER=0 的实例与未加载格子一律黑（向后兼容）相同
//   P7 image_complete 四个位与四个格子一一对应（逐位单独置 1）
// -----------------------------------------------------------------------------
module tb_thumb_placeholder;
    reg  [11:0] x = 0, y = 0;
    reg         de = 1, thumb = 1;
    reg  [1:0]  focus = 0;
    reg  [3:0]  cpl = 0, ex = 0;
    reg  [7:0]  lvl = 8'h50;
    wire [9:0]  px = x - 160, py = y - 45;
    wire [23:0] rgb_in = {px[7:0], py[7:0], 8'hA5 ^ px[9:2]};
    wire [23:0] out1, out0;
    wire        tde1, fb1, ph1;
    wire [1:0]  si1;

    thumb_view_mixer #(.FOCUS_RGB(24'hFFD000), .PLACEHOLDER(1)) d1 (
        .de_in(de), .x_in(x), .y_in(y), .rgb_in(rgb_in), .thumb_display(thumb), .focus_index(focus),
        .image_complete(cpl), .clip_en(1'b0), .clip_x0(12'd0), .clip_x1(12'd0), .clip_y0(12'd0), .clip_y1(12'd0),
        .cell_exists(ex), .ph_level(lvl), .rgb_out(out1), .thumb_de(tde1), .source_index(si1),
        .focus_border(fb1), .placeholder(ph1));
    thumb_view_mixer #(.FOCUS_RGB(24'hFFD000), .PLACEHOLDER(0)) d0 (
        .de_in(de), .x_in(x), .y_in(y), .rgb_in(rgb_in), .thumb_display(thumb), .focus_index(focus),
        .image_complete(cpl), .clip_en(1'b0), .clip_x0(12'd0), .clip_x1(12'd0), .clip_y0(12'd0), .clip_y1(12'd0),
        .cell_exists(ex), .ph_level(lvl), .rgb_out(out0), .thumb_de(), .source_index(), .focus_border(), .placeholder());

    integer errors = 0;
    task fail(input [8*72-1:0] msg); begin errors = errors + 1; if (errors < 30) $display("FAIL %0s x=%0d y=%0d @cfg cpl=%b ex=%b f=%0d", msg, px, py, cpl, ex, focus); end endtask

    integer i, j, lx, ly, cel, n_focus, n_ph, n_ph_edge;
    reg ingrid, bord;
    reg [23:0] exp1, exp0;
    task scan; begin
        n_focus = 0; n_ph = 0; n_ph_edge = 0;
        for (j = 0; j < 480; j = j + 1) for (i = 0; i < 640; i = i + 1) begin
            x = i + 160; y = j + 45; #1;
            cel   = (j >= 120 ? 2 : 0) + (i >= 160 ? 1 : 0);
            lx     = i - (i >= 160 ? 160 : 0);
            ly     = j - (j >= 120 ? 120 : 0);
            ingrid = (i < 320) && (j < 240);
            bord   = (lx < 2) || (ly < 2) || (lx >= 158) || (ly >= 118);
            if (!thumb) begin
                exp1 = rgb_in; exp0 = rgb_in;
            end else if (ingrid && cpl[cel]) begin
                exp1 = (cel == focus && bord) ? 24'hFFD000 : rgb_in; exp0 = exp1;
                if (cel == focus && bord) n_focus = n_focus + 1;
            end else if (ingrid && ex[cel]) begin
                exp1 = bord ? {3{lvl}} : {3{1'b0, lvl[7:1]}}; exp0 = 24'h0;
                n_ph = n_ph + 1; if (bord) n_ph_edge = n_ph_edge + 1;
            end else begin
                exp1 = 24'h0; exp0 = 24'h0;
            end
            if (out1 !== exp1) fail("P2-P5 pixel mismatch (PLACEHOLDER=1)");
            if (out0 !== exp0) fail("P6 pixel mismatch (PLACEHOLDER=0)");
            if (thumb) begin
                if (si1 !== cel[1:0] && i < 320 && j < 240) fail("P1 source_index");
                if (tde1 !== (ingrid && cpl[cel])) fail("P1 thumb_de");
                if (ph1 !== (ingrid && !cpl[cel] && ex[cel])) fail("P3 placeholder flag");
                if (fb1 && !(cpl[cel] && cel == focus)) fail("P2 focus border on unloaded / non-focus cel");
            end
        end
    end endtask

    integer s, k;
    initial begin
        // 全部已加载，焦点逐格
        thumb = 1; cpl = 4'hF; ex = 4'hF;
        for (k = 0; k < 4; k = k + 1) begin
            focus = k; scan;
            if (n_focus != 1104) begin fail("P2 focus border count"); $display("  n_focus=%0d", n_focus); end
            if (n_ph != 0) fail("P3 placeholder when all loaded");
        end
        // P7 逐位：只有格子 s 已加载，其余有图未加载；焦点在 s / 不在 s
        for (s = 0; s < 4; s = s + 1) begin
            cpl = 4'b0001 << s; ex = 4'hF; focus = s; lvl = 8'h30 + s * 16; scan;
            if (n_focus != 1104) fail("P7 focus on loaded cel");
            if (n_ph != 3 * 19200 || n_ph_edge != 3 * 1104) begin fail("P7 placeholder counts"); $display("  n_ph=%0d bord=%0d", n_ph, n_ph_edge); end
            focus = (s + 1) % 4; scan;
            if (n_focus != 0) fail("P7 focus border on unloaded focus cel");
        end
        // 只有 3 张图（格子 3 不存在）：前 2 张已加载、第 3 张加载中
        cpl = 4'b0011; ex = 4'b0111; focus = 1; lvl = 8'h6E; scan;
        if (n_ph != 19200) fail("P4 only existing cells get placeholder");
        // 全屏直通
        thumb = 0; cpl = 4'b0000; ex = 4'hF; scan;
        if (errors == 0) $display("PASS tb_thumb_placeholder");
        else $display("FAIL tb_thumb_placeholder errors=%0d", errors);
        $finish;
    end
endmodule
