`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// tb_picture_res_osd —— 参数条分辨率 + 切到新画面自动显示（顶层同一条链、真实 640x480@60 时序）
//   sys 50MHz：load_done / loaded_buffer_idx / src_width / src_height（模拟 sd_card_bmp）
//     → picture_res_info（锁存 → 翻转 2FF → pix 域 4 bank 表）
//   pix 25MHz：video_timing_ctrl → frame_start_raw（同顶层）提交 {display_valid, bank, 图片号, 总览}（模拟
//     media_player_control / thumb_zoom_transition 的帧边界提交）→ rgb_video_adapter → osd_layer_mixer(TS_ENABLE=0)
//   检查：
//     1 每帧（mixer frame_start 后 BCD 转换完成时）参数条锁存的分辨率 = 本帧生效 bank 的分辨率（换图当帧就对，
//       不晚一帧）；宽 4 位 BCD、高左对齐补 F；总览 / 未加载 = 无效标志 0（"----X----"）
//     2 pic_show 只在：首图出现、全屏换图（KEY2 / 自动轮播）、总览回全屏（SW4 拨下）；
//       总览里换焦点、进总览、重复提交同一张图、后台加载其它 bank 都不出
//     3 每次 pic_show 后参数条保持计数重装（参数条重新显示、淡入），间隔足够长时会淡出到 0
//     4 load_done 那一拍采样宽高：下一拍 src_width/src_height 立刻变成别的值（下一张图的头）也不影响
//     5 去掉时间戳后：时间戳层恒为 0，原时间戳区域（右上角）不被改动
// -----------------------------------------------------------------------------
module tb_picture_res_osd;
    reg sys_clk = 0, pix_clk = 0, rst_n = 0;
    always #10 sys_clk = ~sys_clk;       // 50 MHz
    always #20 pix_clk = ~pix_clk;       // 25 MHz

    localparam integer HOLD = 4;         // 参数条保持帧数（缩短，便于看到重装 / 淡出）

    // ---------------- 视频时序（顶层同参） ----------------
    wire raw_hs, raw_vs, raw_de;
    wire [11:0] raw_x, raw_y;
    video_timing_ctrl #(
        .H_ACT(640), .H_FP(16), .H_SYNC(96), .H_BP(48),
        .V_ACT(480), .V_FP(10), .V_SYNC(2), .V_BP(33),
        .ACTIVE_LAST(1), .VS_ALIGN_HSYNC(1)
    ) u_timing (.pclk(pix_clk), .rst_n(rst_n), .hsync(raw_hs), .vsync(raw_vs), .de(raw_de), .x(raw_x), .y(raw_y));
    reg raw_vs_q = 1'b1;
    always @(posedge pix_clk) raw_vs_q <= raw_vs;
    wire frame_start_raw = raw_vs_q && !raw_vs;

    wire hs, vs, de;
    wire [23:0] bg_rgb;
    rgb_video_adapter u_adapter (
        .pclk(pix_clk), .rst_n(rst_n), .hs_in(raw_hs), .vs_in(raw_vs), .de_in(raw_de),
        .rgb_in({raw_x[7:0], raw_y[7:0], 8'h5a}), .video_clk(), .video_hs(hs), .video_vs(vs), .video_de(de), .video_rgb(bg_rgb));

    // ---------------- sys：加载完成 ----------------
    reg        load_done = 0;
    reg [1:0]  loaded_bank = 0;
    reg [11:0] src_w = 0, src_h = 0;

    // ---------------- pix：帧边界提交 ----------------
    reg        display_valid = 0, thumb_display = 0;
    reg [1:0]  active_bank = 0;
    reg [5:0]  active_img = 0;
    reg        pend = 0, p_dv = 0, p_th = 0;
    reg [1:0]  p_bank = 0;
    reg [5:0]  p_img = 0;
    always @(posedge pix_clk) if (frame_start_raw && pend) begin
        display_valid <= p_dv; thumb_display <= p_th; active_bank <= p_bank; active_img <= p_img; pend <= 1'b0;
    end

    // ---------------- DUT ----------------
    wire        res_valid, pic_show;
    wire [11:0] res_width, res_height;
    picture_res_info u_res (
        .sys_clk(sys_clk), .sys_rst_n(rst_n), .load_done(load_done), .loaded_buffer_idx(loaded_bank),
        .src_width(src_w), .src_height(src_h),
        .pix_clk(pix_clk), .pix_rst_n(rst_n), .display_valid(display_valid), .thumb_display(thumb_display),
        .active_buffer_idx(active_bank), .active_image_idx(active_img),
        .res_valid(res_valid), .res_width(res_width), .res_height(res_height), .pic_show(pic_show));

    wire [23:0] final_rgb;
    osd_layer_mixer #(.SUBTITLE_Y(452), .TS_ENABLE(0), .PARAM_HOLD_FRAMES(HOLD)) u_mix (
        .clk(pix_clk), .rst_n(rst_n), .vsync(vs), .de(de), .rgb_in(bg_rgb),
        .subtitle_enable(1'b0), .subtitle_text(96'd0), .subtitle_length(6'd0),
        .brightness(8'd255), .volume(8'd128), .period_tenths(9'd25), .auto_enabled(1'b1),
        .res_valid(res_valid), .res_width(res_width), .res_height(res_height), .param_show(pic_show), .param_hide(1'b0), .ext_hit(1'b0), .ext_color(24'd0), .ext_alpha(8'd0), .txt_hit(1'b0), .txt_color(24'd0), .txt_alpha(8'd0),
        .rgb_out(final_rgb), .overlay_active());

    integer errors = 0;
    task fail(input [8*96-1:0] msg); begin
        errors = errors + 1; if (errors < 20) $display("FAIL @%0t frame %0d: %0s", $time, frames, msg);
    end endtask

    // ---------------- 模型 ----------------
    integer tab_w [0:3];
    integer tab_h [0:3];
    integer i;
    initial for (i = 0; i < 4; i = i + 1) begin tab_w[i] = 0; tab_h[i] = 0; end

    function [15:0] bcd4; input integer n;
        reg [3:0] d3, d2, d1, d0;
        begin
            d3 = n / 1000 % 10; d2 = n / 100 % 10; d1 = n / 10 % 10; d0 = n % 10;
            bcd4 = {d3, d2, d1, d0};
        end
    endfunction
    function [15:0] enc_h; input integer n;
        integer nd, k;
        begin
            enc_h = bcd4(n);
            nd = (n >= 1000) ? 4 : (n >= 100) ? 3 : (n >= 10) ? 2 : 1;
            for (k = 0; k < 4 - nd; k = k + 1) enc_h = {enc_h[11:0], 4'hF};
        end
    endfunction

    // 1 每帧核对锁存的分辨率（mixer frame_start 后 40 拍：12 + 3 拍转换已完成，离参数条行还很远）
    integer frames = 0, res_checks = 0, res_valid_checks = 0;
    reg [15:0] exp_w, exp_h;
    always @(posedge pix_clk) if (rst_n && u_mix.frame_start) frames <= frames + 1;
    always @(posedge pix_clk) if (rst_n && u_mix.frame_start && frames >= 2) begin
        repeat (40) @(posedge pix_clk);
        res_checks = res_checks + 1;
        if (!display_valid || thumb_display || tab_w[active_bank] == 0) begin
            // 无效：只看 1 位标志（BCD 内容不显示，未写过的表项在仿真里是 X）
            if (u_mix.u_text.res_v_frame !== 1'b0) fail("resolution should be invalid (----X----)");
        end else begin
            exp_w = bcd4(tab_w[active_bank]); exp_h = enc_h(tab_h[active_bank]);
            res_valid_checks = res_valid_checks + 1;
            if (u_mix.u_text.res_v_frame !== 1'b1 || u_mix.u_text.res_w_bcd !== exp_w || u_mix.u_text.res_h_bcd !== exp_h) begin
                fail("latched resolution != active bank");
                $display("      got v%b %h X %h  exp %h X %h  (bank %0d)", u_mix.u_text.res_v_frame,
                         u_mix.u_text.res_w_bcd, u_mix.u_text.res_h_bcd, exp_w, exp_h, active_bank);
            end
        end
    end

    // 2/3 pic_show 期望：本帧提交使 {全屏, 图片号} 变成新画面
    reg     prev_full = 0;
    reg [5:0] prev_img = 0;
    integer shows = 0, exp_shows = 0;
    reg     show_pending = 0;
    always @(posedge pix_clk) if (rst_n) begin
        prev_full <= display_valid && !thumb_display;
        prev_img  <= active_img;
        if ((display_valid && !thumb_display) && (!prev_full || active_img != prev_img)) begin
            exp_shows = exp_shows + 1;
            if (!pic_show) fail("missing pic_show on new picture");
        end else if (pic_show) fail("unexpected pic_show");
        if (pic_show) begin shows = shows + 1; show_pending <= 1'b1; end
        if (show_pending) begin
            show_pending <= 1'b0;
            if (u_mix.param_hold !== HOLD) fail("param_hold not reloaded after pic_show");
        end
    end

    // 5 时间戳层不存在；右上角原时间戳区域原样透传（参数条只在 x < 416）
    always @(posedge pix_clk) if (rst_n && frames >= 2) begin
        if (u_mix.ts_pixel !== 1'b0 || u_mix.ts_box !== 1'b0) fail("timestamp layer still active");
        if (de && u_mix.pixel_x >= 416 && final_rgb !== bg_rgb) fail("pixel changed right of param bar");
    end

    // ---------------- 激励 ----------------
    task load(input [1:0] bank, input integer w, input integer h);
        begin
            @(posedge sys_clk);
            src_w <= w; src_h <= h;
            repeat (3) @(posedge sys_clk);
            load_done <= 1'b1; loaded_bank <= bank;
            @(posedge sys_clk);
            load_done <= 1'b0;
            src_w <= $urandom; src_h <= $urandom;           // 4 下一张图的头紧接着到
            tab_w[bank] = w; tab_h[bank] = h;
            // 真实 load_done 间隔 >= 一次整图加载（毫秒级）；picture_res_info 的准静态跨域依赖这一点，
            // 这里留 100 个 sys 时钟（远大于 2FF + 写表的 4 个 pix 时钟）
            repeat (100) @(posedge sys_clk);
        end
    endtask
    task commit(input dv, input th, input [1:0] bank, input [5:0] img);
        begin
            @(posedge pix_clk);
            p_dv <= dv; p_th <= th; p_bank <= bank; p_img <= img; pend <= 1'b1;
            @(posedge pix_clk);
            wait (!pend);
        end
    endtask
    task wait_frames(input integer n);
        integer f0;
        begin f0 = frames; wait (frames >= f0 + n); end
    endtask

    integer fade_max;
    initial begin
        #1000 rst_n = 1;
        wait_frames(2);
        load(2'd0, 1920, 1080);
        load(2'd1, 640, 480);
        load(2'd2, 800, 600);
        load(2'd3, 1, 4095);
        wait_frames(1);
        commit(1, 0, 2'd0, 6'd0);              // 首图出现 → 显示 1920X1080
        wait_frames(10);                        // 参数条保持 HOLD 帧后淡出
        if (u_mix.param_hold !== 0) fail("param bar hold did not expire");
        commit(1, 0, 2'd1, 6'd1);              // KEY2 下一张 → 640X480
        wait_frames(2);
        load(2'd2, 1024, 768);                  // 后台加载其它 bank：显示不变、不弹参数条
        wait_frames(2);
        commit(1, 1, 2'd1, 6'd1);              // 进总览：不弹，显示 ----X----
        wait_frames(2);
        commit(1, 1, 2'd3, 6'd2);              // 总览里换焦点（bank 可能是别的图）：不弹
        wait_frames(2);
        commit(1, 0, 2'd2, 6'd5);              // SW4 拨下选定 → 回全屏 1024X768
        wait_frames(2);
        commit(1, 0, 2'd2, 6'd5);              // 重复提交同一张图：不弹
        wait_frames(2);
        commit(1, 0, 2'd3, 6'd3);              // 自动轮播 → 1X4095（宽右对齐、高 4 位）
        wait_frames(2);
        // 加载完立刻在下一个帧边界提交（加载回执与提交几乎同时）→ 当帧就是新分辨率
        //   load_done 落在帧边界前约 60 个像素时钟（真实链路里控制层还要再过一次握手才提交，更宽松）
        @(posedge pix_clk);
        wait (frame_start_raw); repeat (800 * 525 - 60) @(posedge pix_clk);
        fork
            load(2'd0, 99, 10);
            commit(1, 0, 2'd0, 6'd7);
        join
        wait_frames(2);
        commit(1, 0, 2'd1, 6'd9);              // 换到旧 bank 1（640X480）
        wait_frames(2);
        commit(1, 0, 2'd0, 6'd7);              // 上一张：回 bank 0（99X10）
        fade_max = 0;                           // 前面连续换图已把参数条淡入到 255：保持 HOLD 帧 + 16 帧淡出
        repeat (30) begin
            wait_frames(1);
            if (u_mix.param_fade > fade_max) fade_max = u_mix.param_fade;
        end
        if (fade_max == 0) fail("param bar never faded in after pic_show");
        if (u_mix.param_fade !== 0) fail("param bar did not fade out");
        $display("INFO frames=%0d res_checks=%0d (valid %0d) pic_show=%0d (expected %0d) fade_max=%0d errors=%0d",
                 frames, res_checks, res_valid_checks, shows, exp_shows, fade_max, errors);
        if (shows != 7) fail("pic_show count != 7");
        if (errors == 0 && res_valid_checks > 15) $display("PASS tb_picture_res_osd");
        else                                      $display("FAIL tb_picture_res_osd");
        $finish;
    end
endmodule
