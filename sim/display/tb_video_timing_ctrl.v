`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// tb_video_timing_ctrl —— video_timing_ctrl(ACTIVE_LAST=1) + rgb_video_adapter 替换官方
//   video_timing_data(color_bar) 之后，送下游的 hs/vs/de 波形必须与原来逐拍等价（只差一个固定相位）：
//   下游 video_x_tagger / hdmi_audio_symbol_core 按 HTOTAL=800 硬编码数据岛、前导、保护带的列位置。
//   T1 两路各从"第一帧第一个有效像素"起录 2 整帧 (hs,vs,de)，逐拍完全一致
//   T2 640x480@60：每行 800 拍、de 640 拍；每帧 525 行、有效 480 行；hs/vs 低有效，脉宽 96 拍 / 2 行
//   T3 新链路接 video_x_tagger：锁定后每个 de 上升沿 video_x=0，每个 hs 下降沿 video_x=656
//   T4 rgb_video_adapter：消隐期 RGB 强制 0、de 期间透传
// -----------------------------------------------------------------------------
module tb_video_timing_ctrl;
    reg clk = 0, rst_n = 0;
    always #20 clk = ~clk;

    // 官方链路
    wire o_hs, o_vs, o_de;
    video_timing_data u_old (.video_clk(clk), .rst_n(rst_n), .read_req(), .read_req_ack(1'b0),
                             .hs(o_hs), .vs(o_vs), .de(o_de));
    // 新链路
    wire r_hs, r_vs, r_de; wire [11:0] r_x, r_y;
    video_timing_ctrl #(.H_ACT(640), .H_FP(16), .H_SYNC(96), .H_BP(48),
                        .V_ACT(480), .V_FP(10), .V_SYNC(2), .V_BP(33), .ACTIVE_LAST(1), .VS_ALIGN_HSYNC(1)) u_vtc (
        .pclk(clk), .rst_n(rst_n), .hsync(r_hs), .vsync(r_vs), .de(r_de), .x(r_x), .y(r_y));
    wire [23:0] rgb_in = {r_x[7:0], r_y[7:0], 8'h5A};
    wire n_hs, n_vs, n_de; wire [23:0] n_rgb;
    rgb_video_adapter u_adp (.pclk(clk), .rst_n(rst_n), .hs_in(r_hs), .vs_in(r_vs), .de_in(r_de),
        .rgb_in(rgb_in), .video_clk(), .video_hs(n_hs), .video_vs(n_vs), .video_de(n_de), .video_rgb(n_rgb));
    wire t_hs, t_vs, t_de, t_locked; wire [23:0] t_rgb; wire [9:0] t_x;
    video_x_tagger u_tag (.clk(clk), .rst_n(rst_n), .hs_i(n_hs), .vs_i(n_vs), .de_i(n_de), .rgb_i(n_rgb),
        .hs_o(t_hs), .vs_o(t_vs), .de_o(t_de), .rgb_o(t_rgb), .video_x_o(t_x), .locked_o(t_locked));

    localparam integer FRAME = 800 * 525, N = 2 * FRAME;
    reg [2:0] rec_o [0:N-1];
    reg [2:0] rec_n [0:N-1];
    integer   no = -1, nn = -1;          // -1 = 等对齐点
    reg       o_vs_seen = 0, n_vs_seen = 0, o_de_q = 0, n_de_q = 0, o_vs_q = 1, n_vs_q = 1;
    integer   errors = 0;

    // 对齐点：复位后第一个 vs 下降沿之后的第一个 de 上升沿（即一帧第一个有效像素）
    always @(posedge clk) if (rst_n) begin
        o_vs_q <= o_vs; n_vs_q <= n_vs; o_de_q <= o_de; n_de_q <= n_de;
        if (o_vs_q && !o_vs) o_vs_seen <= 1;
        if (n_vs_q && !n_vs) n_vs_seen <= 1;
        if (no < 0 && o_vs_seen && o_de && !o_de_q) no = 0;
        if (nn < 0 && n_vs_seen && n_de && !n_de_q) nn = 0;
        if (no >= 0 && no < N) begin rec_o[no] = {o_hs, o_vs, o_de}; no = no + 1; end
        if (nn >= 0 && nn < N) begin rec_n[nn] = {n_hs, n_vs, n_de}; nn = nn + 1; end
        // T4
        if (!n_de && n_rgb !== 24'd0) begin
            if (errors < 10) $display("FAIL T4 rgb not blanked @%0t", $time); errors = errors + 1;
        end
    end

    // T3：x 对齐
    reg t_de_q = 0, t_hs_q = 1; integer n_lines = 0;
    always @(posedge clk) if (rst_n) begin
        t_de_q <= t_de; t_hs_q <= t_hs;
        if (t_locked && t_de && !t_de_q) begin
            n_lines = n_lines + 1;
            if (t_x !== 10'd0) begin if (errors < 10) $display("FAIL T3 de rise at x=%0d", t_x); errors = errors + 1; end
        end
        if (t_locked && t_hs_q && !t_hs && t_x !== 10'd656) begin
            if (errors < 10) $display("FAIL T3 hs fall at x=%0d", t_x); errors = errors + 1;
        end
    end

    integer i, f, l, c, de_cnt, hs_cnt, vs_lines, act_lines, line_de;
    initial begin
        #200 rst_n = 1;
        wait (no == N && nn == N);
        // T1
        for (i = 0; i < N; i = i + 1)
            if (rec_o[i] !== rec_n[i]) begin
                if (errors < 10) $display("FAIL T1 cycle %0d (line %0d col %0d): old hs/vs/de=%b new=%b", i, i / 800, i % 800, rec_o[i], rec_n[i]);
                errors = errors + 1;
            end
        // T2（在新链路记录上统计第一帧）
        act_lines = 0; vs_lines = 0;
        for (l = 0; l < 525; l = l + 1) begin
            line_de = 0; hs_cnt = 0;
            for (c = 0; c < 800; c = c + 1) begin
                if (rec_n[l * 800 + c][0]) line_de = line_de + 1;
                if (!rec_n[l * 800 + c][2]) hs_cnt = hs_cnt + 1;
            end
            if (line_de == 640) act_lines = act_lines + 1;
            else if (line_de != 0) begin $display("FAIL T2 line %0d de=%0d", l, line_de); errors = errors + 1; end
            if (hs_cnt != 96) begin $display("FAIL T2 line %0d hsync width %0d", l, hs_cnt); errors = errors + 1; end
            if (!rec_n[l * 800 + 700][1]) vs_lines = vs_lines + 1;
        end
        if (act_lines != 480 || vs_lines != 2) begin $display("FAIL T2 active lines %0d vsync lines %0d", act_lines, vs_lines); errors = errors + 1; end
        repeat (1000) @(posedge clk);
        $display("INFO compared %0d cycles (2 frames), active lines/frame %0d, tagger lines %0d", N, act_lines, n_lines);
        if (n_lines < 900) begin $display("FAIL T3 too few tagged lines"); errors = errors + 1; end
        if (errors == 0) $display("PASS tb_video_timing_ctrl");
        else $display("FAIL tb_video_timing_ctrl errors=%0d", errors);
        $finish;
    end
    initial begin #80_000_000; $display("FAIL timeout"); $finish; end
endmodule
