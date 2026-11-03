`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// tb_thumbnail_layout ——2×2 缩略图布局（模块级，纯组合逐点扫全屏）
// 检查：
//   L1 四个格子的 source_index（左上 0 / 右上 1 / 左下 2 / 右下 3）
//   L2 局部坐标与 4:1 映射：source_x = 4·(x mod 160)、source_y = 4·(y mod 120)，范围 0..636 / 0..476
//   L3 显示边界：网格只占左上 320×240，右侧 x ≥ 320、下方 y ≥ 240 全部 thumb_de = 0（空白）
//   L4 image_complete 门控：16 种组合，未完成的格子 thumb_de = 0、focus_border = 0
//   L5 焦点边框：focus_index = 0..3，只在焦点格最外 2 像素为 1（每格 160·120 - 156·116 = 1104 个点），其它格子 0
//   L6 in_de = 0 时全部输出为 0
// -----------------------------------------------------------------------------
module tb_thumbnail_layout;
    reg         in_de = 0;
    reg  [11:0] px = 0, py = 0;
    reg  [1:0]  focus = 0;
    reg  [3:0]  done = 4'hF;
    wire        thumb_de, border;
    wire [1:0]  sidx;
    wire [9:0]  sx, sy;

    thumbnail_layout dut (.in_de(in_de), .pixel_x(px), .pixel_y(py), .focus_index(focus),
        .image_complete(done), .thumb_de(thumb_de), .source_index(sidx),
        .source_x(sx), .source_y(sy), .focus_border(border));

    integer errors = 0, x, y, f, m, border_cnt, de_cnt, max_sx, max_sy;
    integer exp_idx, lx, ly;
    reg exp_de, exp_border;
    task fail(input [8*64-1:0] msg); begin errors = errors + 1;
        if (errors < 20) $display("FAIL %0s at x%0d y%0d focus%0d done%b: de%b idx%0d sx%0d sy%0d border%b",
            msg, px, py, focus, done, thumb_de, sidx, sx, sy, border); end endtask

    task sweep(output integer nb, output integer nd);
        begin
            nb = 0; nd = 0;
            for (y = 0; y < 480; y = y + 1)
                for (x = 0; x < 640; x = x + 1) begin
                    px = x; py = y; #1;
                    exp_idx = (y >= 120 ? 2 : 0) + (x >= 160 ? 1 : 0);
                    lx = x % 160; ly = y % 120;
                    exp_de = (x < 320) && (y < 240) && done[exp_idx];
                    exp_border = exp_de && exp_idx == focus && (lx < 2 || ly < 2 || lx >= 158 || ly >= 118);
                    if (thumb_de !== exp_de) fail("thumb_de");
                    if (exp_de) begin
                        if (sidx !== exp_idx) fail("source_index");
                        if (sx !== 4 * lx || sy !== 4 * ly) fail("4:1 mapping");
                        if (sx > max_sx) max_sx = sx;
                        if (sy > max_sy) max_sy = sy;
                    end else if (sx !== 0 || sy !== 0) fail("source_x/y not 0 outside");
                    if (border !== exp_border) fail("focus_border");
                    if (border) nb = nb + 1;
                    if (thumb_de) nd = nd + 1;
                end
        end
    endtask

    integer e0;
    initial begin
        max_sx = 0; max_sy = 0;
        in_de = 1;
        // L1/L2/L3/L5：全部完成，4 个焦点
        e0 = errors;
        for (f = 0; f < 4; f = f + 1) begin
            focus = f; done = 4'hF;
            sweep(border_cnt, de_cnt);
            if (border_cnt != 1104) begin errors = errors + 1; $display("FAIL focus %0d border pixels %0d (exp 1104)", f, border_cnt); end
            if (de_cnt != 4 * 19200) begin errors = errors + 1; $display("FAIL thumb_de pixels %0d (exp 76800)", de_cnt); end
            $display("INFO focus %0d: border %0d px, thumb_de %0d px (320x240 grid, rest blank)", f, border_cnt, de_cnt);
        end
        if (max_sx != 636 || max_sy != 476) begin errors = errors + 1; $display("FAIL max source %0d/%0d", max_sx, max_sy); end
        $display("INFO L1/L2/L3/L5 errors %0d, source_x max %0d source_y max %0d", errors - e0, max_sx, max_sy);
        // L4：16 种 image_complete 组合
        e0 = errors;
        for (m = 0; m < 16; m = m + 1) begin
            done = m; focus = m[1:0];
            sweep(border_cnt, de_cnt);
            if (de_cnt != 19200 * (m[0] + m[1] + m[2] + m[3])) begin errors = errors + 1; $display("FAIL done %b thumb_de %0d", done, de_cnt); end
            if (border_cnt != (done[focus] ? 1104 : 0)) begin errors = errors + 1; $display("FAIL done %b border %0d", done, border_cnt); end
        end
        $display("INFO L4 image_complete gating (16 combos) errors %0d", errors - e0);
        // L6：in_de = 0
        e0 = errors;
        in_de = 0; done = 4'hF;
        for (y = 0; y < 480; y = y + 7)
            for (x = 0; x < 640; x = x + 3) begin
                px = x; py = y; #1;
                if (thumb_de || border || sx != 0 || sy != 0 || sidx != 0) fail("output while in_de=0");
            end
        // 有效区外的坐标（>= 640 / >= 480）
        in_de = 1;
        px = 700; py = 10; #1; if (thumb_de || border) fail("x >= 640");
        px = 10; py = 500; #1; if (thumb_de || border) fail("y >= 480");
        $display("INFO L6 errors %0d", errors - e0);

        if (errors == 0) $display("PASS tb_thumbnail_layout");
        else $display("FAIL tb_thumbnail_layout errors=%0d", errors);
        $finish;
    end
endmodule
