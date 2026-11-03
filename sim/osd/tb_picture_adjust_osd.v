`timescale 1ns/1ps
// picture_adjust_osd：小分辨率时序逐帧逐像素检查
//   每帧统计：面板外像素必须直通；面板内文字按 tb 自带 5x7 字库逐像素比对（b/C + 三位数，百位 0 消隐）；
//   进度条 ly=2 行填充像素数 = 值/2（255 填满 128）；当前项颜色 / 行首色块只在调节模式；对比度行中点刻度
//   O1 默认值 255/128 且不在模式：整帧直通（面板隐藏）
//   O2 对比度变化 → 显示 HOLD_FRAMES 帧后隐藏；数值 / 进度条正确（O2a：模式外只变亮度 = 环境自适应，不弹出）
//   O3 调节模式常显，当前项（亮度 / 对比度）黄色文字 + 青色进度条 + 行首色块，另一项白 / 灰
//   O4 帧中途改输入：本帧显示不变（frame_start 锁存），下一帧才更新
//   O5 端点 0 / 5 / 99 / 255；退出模式后保持 HOLD 帧再隐藏
module tb_picture_adjust_osd;
    localparam integer H_ACT = 256, V_ACT = 112, HOLD = 3;
    localparam integer PX = 8, PY = 64, TEXT_X = PX + 8, BAR_X = TEXT_X + 60, PANEL_L = PX - 2, PANEL_R = BAR_X + 128 + 4;
    localparam [23:0] SEL = 24'hffd000, TXT = 24'hf0f0f0, BSEL = 24'h00c0ff, BOTH = 24'h909090, TICK = 24'hff4040;
    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    wire hs, vs, de;
    wire [11:0] x, y;
    wire fs;
    video_timing_ctrl #(.H_ACT(H_ACT), .H_FP(4), .H_SYNC(4), .H_BP(4), .V_ACT(V_ACT), .V_FP(2), .V_SYNC(2), .V_BP(2)) u_vt (
        .pclk(clk), .rst_n(rst_n), .hsync(hs), .vsync(vs), .de(de), .x(x), .y(y));
    frame_boundary_pulse #(.V_ACTIVE(V_ACT)) u_fb (.pix_clk(clk), .pix_rst_n(rst_n), .vsync_n(vs), .de(de), .frame_start(fs), .blank_start());

    reg  [7:0] b_in = 255, c_in = 128;
    reg        m_in = 0, s_in = 0;
    wire [23:0] rgb_in = {x[7:0], y[7:0] ^ 8'h3c, x[7:0] + y[7:0]};
    wire [23:0] rgb_out;
    wire        active;
    picture_adjust_osd #(.PANEL_X(PX), .PANEL_Y(PY), .HOLD_FRAMES(HOLD)) dut (
        .clk(clk), .rst_n(rst_n), .frame_start(fs), .vsync(vs), .de(de), .rgb_in(rgb_in),
        .brightness(b_in), .contrast(c_in), .adj_mode(m_in), .adj_sel(s_in),
        .rgb_out(rgb_out), .osd_active(active));

    // ---------------- tb 字库（与状态条 5x7 字形一致，独立抄写） ----------------
    function [4:0] font(input integer g, input integer r);
        reg [34:0] bm;
        begin
            case (g)
                0: bm = {5'b11111,5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b11111};
                1: bm = {5'b00100,5'b01100,5'b00100,5'b00100,5'b00100,5'b00100,5'b11111};
                2: bm = {5'b11111,5'b00001,5'b00001,5'b11111,5'b10000,5'b10000,5'b11111};
                3: bm = {5'b11111,5'b00001,5'b00001,5'b11111,5'b00001,5'b00001,5'b11111};
                4: bm = {5'b10001,5'b10001,5'b10001,5'b11111,5'b00001,5'b00001,5'b00001};
                5: bm = {5'b11111,5'b10000,5'b10000,5'b11111,5'b00001,5'b00001,5'b11111};
                6: bm = {5'b11111,5'b10000,5'b10000,5'b11111,5'b10001,5'b10001,5'b11111};
                7: bm = {5'b11111,5'b00001,5'b00010,5'b00100,5'b01000,5'b01000,5'b01000};
                8: bm = {5'b11111,5'b10001,5'b10001,5'b11111,5'b10001,5'b10001,5'b11111};
                9: bm = {5'b11111,5'b10001,5'b10001,5'b11111,5'b00001,5'b00001,5'b11111};
                11: bm = {5'b10000,5'b10000,5'b11110,5'b10001,5'b10001,5'b10001,5'b11110};   // b
                12: bm = {5'b01111,5'b10000,5'b10000,5'b10000,5'b10000,5'b10000,5'b01111};   // C
                default: bm = 35'd0;
            endcase
            font = (r < 7) ? bm[34 - 5*r -: 5] : 5'd0;
        end
    endfunction
    function integer glyph_of(input integer cell_idx, input integer row, input integer v);
        case (cell_idx)
            0: glyph_of = row ? 12 : 11;
            2: glyph_of = (v / 100 == 0) ? 15 : v / 100;
            3: glyph_of = (v / 10) % 10;
            4: glyph_of = v % 10;
            default: glyph_of = 15;
        endcase
    endfunction

    // ---------------- 帧统计 ----------------
    reg  [7:0] lb = 255, lc = 128; reg lm = 0, ls = 0;          // frame_start 时的输入（参考锁存）
    integer outside_err, panel_px, panel_pass, text_err, tcolor_err, tick_cnt, bg_err;
    integer fill [0:1]; integer fill_sel [0:1]; integer marker [0:1];
    integer r_outside_err, r_panel_px, r_panel_pass, r_text_err, r_tcolor_err, r_tick, r_bg_err;
    integer r_fill [0:1]; integer r_fill_sel [0:1]; integer r_marker [0:1];
    reg [7:0] r_lb, r_lc; reg r_lm, r_ls;
    event frame_done;
    integer px, py, row, ly, cell_idx, col, v, g; reg on, sel_row;
    task clear_stats;
        begin
            outside_err = 0; panel_px = 0; panel_pass = 0; text_err = 0; tcolor_err = 0; tick_cnt = 0; bg_err = 0;
            fill[0] = 0; fill[1] = 0; fill_sel[0] = 0; fill_sel[1] = 0; marker[0] = 0; marker[1] = 0;
        end
    endtask
    initial clear_stats;
    always @(posedge clk) if (rst_n && fs) begin
        r_outside_err = outside_err; r_panel_px = panel_px; r_panel_pass = panel_pass; r_text_err = text_err;
        r_tcolor_err = tcolor_err; r_tick = tick_cnt; r_bg_err = bg_err;
        r_fill[0] = fill[0]; r_fill[1] = fill[1]; r_fill_sel[0] = fill_sel[0]; r_fill_sel[1] = fill_sel[1];
        r_marker[0] = marker[0]; r_marker[1] = marker[1];
        r_lb = lb; r_lc = lc; r_lm = lm; r_ls = ls;
        clear_stats;
        lb = b_in; lc = c_in; lm = m_in; ls = s_in;
        -> frame_done;
    end
    always @(negedge clk) if (rst_n && de) begin
        px = x; py = y;
        if (!(px >= PANEL_L && px < PANEL_R && py >= PY && py < PY + 32)) begin
            if (rgb_out !== rgb_in) outside_err = outside_err + 1;
        end else begin
            panel_px = panel_px + 1;
            if (rgb_out === rgb_in) panel_pass = panel_pass + 1;
            row = (py - PY) / 16; ly = (py - PY) % 16;
            v = row ? lc : lb;
            sel_row = lm && (row == ls);
            // 文字格
            if (px >= TEXT_X && px < TEXT_X + 60 && ly < 14) begin
                cell_idx = (px - TEXT_X) / 12; col = (px - TEXT_X) % 12;
                g = glyph_of(cell_idx, row, v);
                on = (col < 10) && font(g, ly / 2) >> (4 - col / 2) & 1;
                if (on !== ((rgb_out === TXT) || (rgb_out === SEL))) text_err = text_err + 1;
                if (on && rgb_out !== (sel_row ? SEL : TXT)) tcolor_err = tcolor_err + 1;
                if (!on && rgb_out !== {1'b0, rgb_in[23:17], 1'b0, rgb_in[15:9], 1'b0, rgb_in[7:1]}) bg_err = bg_err + 1;
            end
            // 进度条内部 ly=2 行
            if (ly == 2 && px > BAR_X && px <= BAR_X + 128) begin
                if (rgb_out === BSEL || rgb_out === BOTH) fill[row] = fill[row] + 1;
                if (rgb_out === BSEL) fill_sel[row] = fill_sel[row] + 1;
            end
            if (rgb_out === TICK) tick_cnt = tick_cnt + 1;
            if (px >= PX && px < PX + 4 && rgb_out === SEL) marker[row] = marker[row] + 1;
        end
    end

    integer errors = 0;
    task err(input [8*48-1:0] m); begin $display("FAIL %0s (frame lb=%0d lc=%0d m=%b s=%b)", m, r_lb, r_lc, r_lm, r_ls); errors = errors + 1; end endtask
    function integer fill_exp(input integer vv); fill_exp = (vv == 255) ? 128 : vv / 2; endfunction
    // 等一帧并检查它（输入在上一个 frame_start 之后已设好）
    task check_frame(input vis, input [8*24-1:0] tag);
        begin
            @frame_done;
            if (r_outside_err != 0) err({tag, " outside panel modified"});
            if (!vis) begin
                if (r_panel_pass != r_panel_px) err({tag, " panel must be hidden"});
            end else begin
                if (r_panel_pass != 0)  err({tag, " panel pixels passed through while visible"});
                if (r_text_err != 0)    begin err({tag, " text glyph mismatch"}); $display("  text_err=%0d", r_text_err); end
                if (r_tcolor_err != 0)  err({tag, " text colour"});
                if (r_bg_err != 0)      err({tag, " panel background not 50% dim"});
                if (r_fill[0] != fill_exp(r_lb) || r_fill[1] != fill_exp(r_lc)) begin
                    err({tag, " progress bar length"}); $display("  fill b=%0d c=%0d", r_fill[0], r_fill[1]);
                end
                if (r_fill_sel[0] != ((r_lm && !r_ls) ? r_fill[0] : 0) || r_fill_sel[1] != ((r_lm && r_ls) ? r_fill[1] : 0))
                    err({tag, " progress bar colour"});
                if ((r_marker[0] != 0) !== (r_lm && !r_ls) || (r_marker[1] != 0) !== (r_lm && r_ls))
                    err({tag, " marker"});
                if (r_tick != 9) err({tag, " contrast tick"});
            end
        end
    endtask
    task set_in(input [7:0] b, input [7:0] c, input m, input s);
        @(negedge clk) begin b_in = b; c_in = c; m_in = m; s_in = s; end
    endtask

    initial begin
        #33 rst_n = 1;
        @frame_done;                                     // 第一帧（复位后统计不完整）丢弃
        // O1 默认值隐藏
        check_frame(0, "O1 default hidden");
        check_frame(0, "O1 default hidden 2");
        // O2a 外接光照：模式外只有亮度变（环境自适应）→ 面板不弹出（由参数条提示）
        set_in(8'd100, 8'd128, 0, 0);
        @frame_done;
        check_frame(0, "O2a brightness-only change stays hidden");
        check_frame(0, "O2a brightness-only change stays hidden 2");
        // O2 模式外对比度变化 → 显示 HOLD 帧
        set_in(8'd100, 8'd140, 0, 0);
        @frame_done;                                     // 这一帧锁存 100/140
        check_frame(1, "O2 hold 1");
        check_frame(1, "O2 hold 2");
        check_frame(1, "O2 hold 3");
        check_frame(0, "O2 hidden after hold");
        // O3 调节模式常显 + 当前项
        set_in(8'd100, 8'd128, 1, 0);
        @frame_done;
        repeat (HOLD + 2) check_frame(1, "O3 mode sel=b");
        set_in(8'd100, 8'd200, 1, 1);
        @frame_done;
        check_frame(1, "O3 mode sel=C");
        // O4 帧中途改值：本帧不变
        @frame_done;                                     // 帧开始（锁存 100/200）
        repeat (H_ACT * 70) @(posedge clk);              // 走到面板中部（y≈64..95 附近）
        set_in(8'd37, 8'd9, 1, 0);
        @frame_done;                                     // 上一帧的统计已在这里结算
        if (r_lb !== 8'd100 || r_lc !== 8'd200 || r_text_err != 0 || r_fill[0] != 50 || r_fill[1] != 100)
            err("O4 mid-frame change leaked into the frame");
        check_frame(1, "O4 next frame updated");
        if (r_lb !== 8'd37 || r_fill[0] != 18 || r_fill[1] != 4) err("O4 next frame values");
        // O5 端点
        set_in(8'd0, 8'd255, 1, 1);   @frame_done; check_frame(1, "O5 0/255");
        set_in(8'd5, 8'd99, 1, 0);    @frame_done; check_frame(1, "O5 5/99");
        set_in(8'd255, 8'd128, 1, 1); @frame_done; check_frame(1, "O5 255/128");
        // 退出模式：值未变 → 保持 HOLD 帧（模式中每帧都重装）后隐藏
        set_in(8'd255, 8'd128, 0, 1); @frame_done;
        check_frame(1, "O5 exit hold 1");
        check_frame(1, "O5 exit hold 2");
        check_frame(1, "O5 exit hold 3");
        check_frame(0, "O5 hidden after exit");

        if (errors == 0) $display("PASS tb_picture_adjust_osd (O1-O5 + O2a: hide/hold, ambient brightness no-pop, glyphs, progress bar, mode colours, frame latch)");
        else             $display("FAIL tb_picture_adjust_osd errors=%0d", errors);
        $finish;
    end
endmodule
