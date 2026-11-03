`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// tb_transition_chain —— 转场子系统集成验证（与顶层同一条连线，缩小视频时序）
//   按键事件 → video_fade_transition → media_policy → media_player_control（真实 RTL）
//     → frame_start 提交 active_buffer_idx / index_commit → video_fade_transition
//   reader 模型：frame_start 下一拍锁存 active_buffer_idx，按 bank 里的图片号生成测试图
//   → video_transition_pixel → rgb_video_adapter
//   两套链路同激励：FADE=16（ch）与 FADE=0 旁路（cb）。
//   检查：
//     C1 每帧逐像素 = 参考模型（本帧锁存的图片 × 本帧 alpha / 溶解档位 + 块序号）——帧内统一，无撕裂
//     C2 手动下一张/上一张：每次按键只产生 1 个 goto；换图那一帧 alpha=0（全黑时切换）；
//        淡出帧 alpha 单调下降、淡入帧单调上升；目标图片正确
//     C3 自动轮播（auto_tick 经转场控制器，next_auto_in=1）：每次换图都是方块溶解（2 倍速）——换图前一帧
//        HOLD 全黑，之前溶解档位 14→2 逐帧 -2，新图 2→14 逐帧 +2，回 IDLE 全显；全程相邻两帧可见档位差 ≤ 2（画面连续）；
//        溶解期间像素 alpha 恒 255，转场控制器 alpha 照常淡出淡入（音频用）
//     C4 图片数 = 1（策略层拒绝）：淡出 → error_event 结束全黑 → 淡入，图片不变，不卡黑
//     C5 旁路链：按键直接 goto、帧边界硬切、输出与输入逐位相等（= 原工程行为）
// -----------------------------------------------------------------------------
module trans_chain #(
    parameter integer FADE = 16,
    parameter integer H_ACT = 64, H_FP = 4, H_SYNC = 8, H_BP = 4,
    parameter integer V_ACT = 48, V_FP = 2, V_SYNC = 2, V_BP = 4
) (
    input  wire        sys_clk,
    input  wire        pix_clk,
    input  wire        rst_n,
    input  wire        key_next,
    input  wire        key_prev,
    input  wire        key_auto,
    input  wire [6:0]  image_count,
    output wire        hs, vs, de,
    output wire [23:0] rgb
);
    localparam integer H_TOTAL = H_ACT + H_FP + H_SYNC + H_BP;
    localparam integer V_TOTAL = V_ACT + V_FP + V_SYNC + V_BP;

    // ---------------- loader 模型 ----------------
    wire        load_request;
    wire [5:0]  load_image_idx;
    wire [1:0]  load_buffer_idx;
    reg         ld_busy = 0;
    integer     ld_cnt = 0;
    reg  [5:0]  ld_img = 0;
    reg  [1:0]  ld_buf = 0;
    reg         load_done = 0;
    reg  [5:0]  loaded_image_idx = 0;
    reg  [1:0]  loaded_buffer_idx = 0;
    reg  [3:0]  image_complete = 0;
    reg  [5:0]  bank_content [0:3];
    always @(posedge sys_clk) begin
        load_done <= 0;
        if (!rst_n) begin image_complete <= 0; ld_busy <= 0; end
        else if (load_request) begin
            ld_busy <= 1; ld_cnt <= 2000; ld_img <= load_image_idx; ld_buf <= load_buffer_idx;
            image_complete[load_buffer_idx] <= 1'b0;
        end else if (ld_busy) begin
            if (ld_cnt == 0) begin
                ld_busy <= 0; load_done <= 1; loaded_image_idx <= ld_img; loaded_buffer_idx <= ld_buf;
                image_complete[ld_buf] <= 1'b1; bank_content[ld_buf] = ld_img;
            end else ld_cnt <= ld_cnt - 1;
        end
    end

    // ---------------- 视频时序 ----------------
    wire raw_hs, raw_vs, raw_de;
    wire [11:0] raw_x, raw_y;
    video_timing_ctrl #(
        .H_ACT(H_ACT), .H_FP(H_FP), .H_SYNC(H_SYNC), .H_BP(H_BP),
        .V_ACT(V_ACT), .V_FP(V_FP), .V_SYNC(V_SYNC), .V_BP(V_BP),
        .ACTIVE_LAST(1), .VS_ALIGN_HSYNC(1)
    ) u_timing (.pclk(pix_clk), .rst_n(rst_n), .hsync(raw_hs), .vsync(raw_vs), .de(raw_de), .x(raw_x), .y(raw_y));
    reg raw_vs_q = 1;
    always @(posedge pix_clk) raw_vs_q <= raw_vs;
    wire frame_start_raw = raw_vs_q && !raw_vs;

    // ---------------- 控制链（与顶层同样的连线）----------------
    wire goto_valid, toggle_auto_event, period_cfg_valid, auto_tick, auto_enabled;
    wire [5:0] goto_image_idx, current_image_idx, active_image_idx;
    wire [1:0] goto_audio_idx, active_buffer_idx, active_audio_idx;
    wire [31:0] period_cfg_cycles;
    wire display_valid_sys, error_event, request_busy, display_valid, index_commit;
    wire fade_next_event, fade_next_is_prev, fade_busy, fade_active, dissolve_active;
    wire [7:0] fade_alpha;
    wire [4:0] fade_ramp;
    // 顶层同款：轮播到点经转场控制器（无总览 / 导览，auto_direct 恒 0）；按键同拍按键优先
    wire key_ev = key_next | key_prev;

    video_fade_transition #(.FADE_FRAMES(FADE), .HOLD_TIMEOUT_FRAMES(480)) u_fade (
        .ctrl_clk(sys_clk), .ctrl_rst_n(rst_n),
        .next_event_in(key_ev | auto_tick), .next_tag_in(key_prev), .next_auto_in(~key_ev),
        .next_block_in(request_busy), .hold_abort_in(error_event),
        .next_event_out(fade_next_event), .next_tag_out(fade_next_is_prev), .fade_busy(fade_busy),
        .pix_clk(pix_clk), .pix_rst_n(rst_n), .player_active(display_valid),
        .frame_start(frame_start_raw), .index_commit(index_commit),
        .alpha_out(fade_alpha), .fade_active(fade_active), .dissolve_active(dissolve_active),
        .ramp_out(fade_ramp));

    media_policy #(.CLK_HZ(50_000_000)) u_policy (
        .clk(sys_clk), .rst_n(rst_n), .enable(display_valid_sys),
        .next_event(fade_next_event & ~fade_next_is_prev), .prev_event(fade_next_event & fade_next_is_prev),
        .auto_event(key_auto), .bright_event(1'b0), .vol_event(1'b0), .period_event(1'b0), .save_event(1'b0),
        .auto_tick(1'b0),
        .adj_inc(1'b0), .adj_dec(1'b0), .adj_default(1'b0), .adj_sel(1'b0), .adj_show(1'b0),
        .ir_present(1'b0), .disp_brightness(8'd255), .bri_base(8'd255), .env_change(1'b0),
        .current_image_idx(current_image_idx), .image_count(image_count), .auto_enabled(auto_enabled),
        .cfg_load_valid(1'b0), .cfg_period_sel(3'd0), .cfg_brightness(9'd0), .cfg_contrast(8'd128), .cfg_volume(8'd0), .cfg_auto_mode(1'b0),
        .goto_valid(goto_valid), .goto_image_idx(goto_image_idx), .goto_audio_idx(goto_audio_idx),
        .toggle_auto_event(toggle_auto_event), .period_cfg_valid(period_cfg_valid), .period_cfg_cycles(period_cfg_cycles),
        .media_id(), .audio_id(), .linked_event(), .bri_offset(), .contrast(), .volume(), .period_tenths(),
        .pic_event(), .vol_page_event(), .prd_event(), .bri_event(), .con_event(), .save_done_event(),
        .saved_period_sel(), .saved_brightness(), .saved_contrast(), .saved_volume(), .saved_auto_mode(), .saved_valid());

    // 轮播周期 = 50 帧（帧长 H_TOTAL*V_TOTAL*40ns，sys 50MHz 每 pix 拍 2 个 sys 拍）
    media_player_control #(
        .DEFAULT_PERIOD_CYCLES (H_TOTAL * V_TOTAL * 2 * 50),
        .MIN_PERIOD_CYCLES     (100),
        .MAX_PERIOD_CYCLES     (32'h7fff_ffff)
    ) u_ctrl (
        .sys_clk(sys_clk), .sys_rst_n(rst_n), .tick(1'b1),
        .goto_valid(goto_valid), .goto_image_idx(goto_image_idx), .goto_audio_idx(goto_audio_idx),
        .toggle_auto_event(toggle_auto_event), .period_cfg_valid(period_cfg_valid), .period_cfg_cycles(period_cfg_cycles),
        .auto_tick(auto_tick), .scan_done(1'b1), .image_count(image_count), .loader_ready(!ld_busy),
        .load_done(load_done), .loaded_image_idx(loaded_image_idx), .loaded_buffer_idx(loaded_buffer_idx),
        .load_error(8'h00), .image_complete(image_complete),
        .load_request(load_request), .load_image_idx(load_image_idx), .load_buffer_idx(load_buffer_idx),
        .auto_enabled(auto_enabled), .active_period_cycles(), .current_image_idx(current_image_idx),
        .image_committed(), .display_valid_sys(display_valid_sys), .control_error(), .error_event(error_event),
        .request_busy(request_busy), .bank_resident(),
        .pix_clk(pix_clk), .pix_rst_n(rst_n), .frame_boundary(frame_start_raw),
        .active_image_idx(active_image_idx), .active_buffer_idx(active_buffer_idx), .active_audio_idx(active_audio_idx),
        .display_valid(display_valid), .index_commit(index_commit));

    // ---------------- reader 模型 ----------------
    reg       fs_d = 0;
    reg [5:0] shown_img = 0;       // 本帧读的图（frame_start 下一拍锁存，同 media_frame_reader）
    reg       shown_valid = 0;
    always @(posedge pix_clk) begin
        fs_d <= frame_start_raw;
        if (fs_d) begin shown_img <= bank_content[active_buffer_idx]; shown_valid <= display_valid; end
    end
    wire [9:0]  col = raw_x - (H_TOTAL - H_ACT);
    wire [9:0]  row = raw_y - (V_TOTAL - V_ACT);
    function [23:0] pattern(input [5:0] img, input [9:0] c, input [9:0] r);
        pattern = {2'b01, img, c[7:0] ^ {r[3:0], 4'h0}, r[7:0] + {img, 2'b11}};
    endfunction
    wire [23:0] reader_rgb = (raw_de && shown_valid) ? pattern(shown_img, col, row) : 24'h0;

    wire t_hs, t_vs, t_de;
    wire [23:0] t_rgb;
    video_transition_pixel #(.FADE_FRAMES(FADE), .PIPELINE(0)) u_pix (
        .clk(pix_clk), .rst_n(rst_n), .hs_in(raw_hs), .vs_in(raw_vs), .de_in(raw_de), .x_in(raw_x), .y_in(raw_y),
        .rgb_in(reader_rgb), .alpha(fade_alpha | {8{dissolve_active}}), .dissolve_active(dissolve_active), .ramp(fade_ramp),
        .seed(active_image_idx[3:0]), .hs_out(t_hs), .vs_out(t_vs), .de_out(t_de), .rgb_out(t_rgb));

    rgb_video_adapter u_adapter (
        .pclk(pix_clk), .rst_n(rst_n), .hs_in(t_hs), .vs_in(t_vs), .de_in(t_de), .rgb_in(t_rgb),
        .video_clk(), .video_hs(hs), .video_vs(vs), .video_de(de), .video_rgb(rgb));
endmodule

module tb_transition_chain;
    localparam integer H_ACT = 64, H_TOTAL = 80, V_ACT = 48, V_TOTAL = 56;
    localparam integer STEP = H_ACT / 16;
    reg sys_clk = 0, pix_clk = 0, rst_n = 0;
    always #10 sys_clk = ~sys_clk;
    always #20 pix_clk = ~pix_clk;

    reg key_next = 0, key_prev = 0, key_auto = 0;
    reg [6:0] image_count = 4;
    wire ch_hs, ch_vs, ch_de, cb_hs, cb_vs, cb_de;
    wire [23:0] ch_rgb, cb_rgb;
    trans_chain #(.FADE(16)) ch (.sys_clk(sys_clk), .pix_clk(pix_clk), .rst_n(rst_n),
        .key_next(key_next), .key_prev(key_prev), .key_auto(key_auto), .image_count(image_count),
        .hs(ch_hs), .vs(ch_vs), .de(ch_de), .rgb(ch_rgb));
    trans_chain #(.FADE(0)) cb (.sys_clk(sys_clk), .pix_clk(pix_clk), .rst_n(rst_n),
        .key_next(key_next), .key_prev(key_prev), .key_auto(key_auto), .image_count(image_count),
        .hs(cb_hs), .vs(cb_vs), .de(cb_de), .rgb(cb_rgb));

    integer errors = 0;

    // ---------------- C1 / C5 逐像素参考模型 ----------------
    // 适配器输出比 raw 晚 1 拍：用 1 拍前的坐标 / 图片 / 控制量（控制量取 DUT 内部寄存值）
    function [23:0] pattern(input [5:0] img, input [9:0] c, input [9:0] r);
        pattern = {2'b01, img, c[7:0] ^ {r[3:0], 4'h0}, r[7:0] + {img, 2'b11}};
    endfunction
    function [7:0] f8(input [7:0] c, input [7:0] a, input full);
        reg [15:0] p; begin p = c * a; f8 = full ? c : p[15:8]; end
    endfunction
    // 块序号参考模型：PRESENT S 盒查表 + 与 RTL 同式的坐标位异或（独立于 RTL 的 case 写法）
    reg [3:0] sb [0:15];
    initial begin
        sb[0]=4'hC; sb[1]=4'h5; sb[2]=4'h6; sb[3]=4'hB; sb[4]=4'h9; sb[5]=4'h0; sb[6]=4'hA; sb[7]=4'hD;
        sb[8]=4'h3; sb[9]=4'hE; sb[10]=4'hF; sb[11]=4'h8; sb[12]=4'h4; sb[13]=4'h7; sb[14]=4'h1; sb[15]=4'h2;
    end
    function [3:0] blk(input [3:0] s, input [11:0] x, input [11:0] y);
        reg [3:0] u, k;
        begin
            u = {y[5]^y[6]^x[9]^x[7], x[6]^y[7]^x[8]^x[5], x[7]^y[8]^x[8]^y[6], x[9]^x[5]^x[7]};
            k = {y[7]^x[7], y[9]^x[7], x[5]^x[7], y[5]^x[7]};
            blk = sb[sb[u] ^ k ^ s];
        end
    endfunction
    reg [9:0]  col_d, row_d;
    reg [5:0]  img_d, bimg_d;
    reg        valid_d, bvalid_d, de_raw_d;
    reg [7:0]  a_d; reg full_d, dis_d; reg [4:0] lvl_d; reg [3:0] seed_d; reg [11:0] x_d, y_d;
    reg [23:0] exp_ch, exp_cb, base;
    integer    px = 0;
    always @(posedge pix_clk) begin
        col_d <= ch.col; row_d <= ch.row; x_d <= ch.raw_x; y_d <= ch.raw_y; de_raw_d <= ch.raw_de;
        img_d <= ch.shown_img; valid_d <= ch.shown_valid;
        bimg_d <= cb.shown_img; bvalid_d <= cb.shown_valid;
        a_d <= ch.u_pix.alpha_q; full_d <= ch.u_pix.alpha_full_q; dis_d <= ch.u_pix.dis_q;
        lvl_d <= ch.u_pix.lvl_q; seed_d <= ch.u_pix.seed_q;
        if (rst_n && de_raw_d) begin
            base   = valid_d ? pattern(img_d, col_d, row_d) : 24'h0;
            exp_ch = (dis_d && !(blk(seed_d, x_d, y_d) < lvl_d)) ? 24'h0 :
                     {f8(base[23:16], a_d, full_d), f8(base[15:8], a_d, full_d), f8(base[7:0], a_d, full_d)};
            exp_cb = bvalid_d ? pattern(bimg_d, col_d, row_d) : 24'h0;
            px = px + 1;
            if (ch_rgb !== exp_ch) begin errors = errors + 1;
                if (errors < 10) $display("FAIL: C1 pixel c=%0d r=%0d got %h exp %h", col_d, row_d, ch_rgb, exp_ch); end
            if (cb_rgb !== exp_cb) begin errors = errors + 1;
                if (errors < 10) $display("FAIL: C5 bypass pixel c=%0d r=%0d got %h exp %h", col_d, row_d, cb_rgb, exp_cb); end
        end
        if (rst_n && (ch_hs !== cb_hs || ch_vs !== cb_vs || ch_de !== cb_de)) begin
            errors = errors + 1; if (errors < 10) $display("FAIL: sync differs between chains");
        end
    end

    // ---------------- 帧记录：每帧中点 ----------------
    integer nf = 0;
    reg [7:0] fa [0:4095];      // alpha
    reg       fw [0:4095];      // 溶解中
    reg [4:0] fb [0:4095];      // 可见档位：溶解 = lvl，否则 alpha 255 → 16、alpha 0 → 0（其余按 alpha/16）
    reg [7:0] fc [0:4095];      // 转场控制器 alpha（音频用）
    reg [5:0] fi [0:4095];      // 本帧图片
    reg [5:0] fbi [0:4095];     // 旁路链本帧图片
    always @(posedge pix_clk) begin
        if (rst_n && ch.raw_x == 0 && ch.raw_y == V_TOTAL - V_ACT / 2) begin
            fa[nf] = ch.u_pix.alpha_q; fc[nf] = ch.fade_alpha; fw[nf] = ch.u_pix.dis_q;
            fb[nf] = ch.u_pix.dis_q ? ch.u_pix.lvl_q : (ch.u_pix.alpha_q == 8'd255) ? 5'd16 : ch.u_pix.alpha_q[7:4];
            fi[nf] = ch.shown_img; fbi[nf] = cb.shown_img;
            nf = nf + 1;
        end
    end

    // goto 计数
    integer goto_ch = 0, goto_cb = 0;
    always @(posedge sys_clk) begin
        if (ch.goto_valid) goto_ch = goto_ch + 1;
        if (cb.goto_valid) goto_cb = goto_cb + 1;
    end

    task key(input integer which);  // 0 next, 1 prev, 2 auto
        begin
            @(posedge sys_clk);
            if (which == 0) key_next <= 1; else if (which == 1) key_prev <= 1; else key_auto <= 1;
            @(posedge sys_clk); key_next <= 0; key_prev <= 0; key_auto <= 0;
        end
    endtask
    task frames(input integer n); integer k; begin for (k = 0; k < n; k = k + 1) @(posedge ch.frame_start_raw); end endtask

    // 分析 [f0, f1) 帧段里的换图：手动 → 换图帧 alpha=0，且之前单调降、之后单调升；自动 → 方块溶解
    integer f, s, k;
    task analyse(input integer f0, input integer f1, input manual, output integer nchg, output integer nok);
        integer f2, j, q;
        reg ok;
        begin
            nchg = 0; nok = 0;
            for (f2 = f0 + 1; f2 < f1; f2 = f2 + 1) begin
                if (fi[f2] != fi[f2 - 1]) begin
                    nchg = nchg + 1; ok = 1;
                    if (manual) begin
                        if (fa[f2] != 0 || fa[f2 - 1] != 0 || fw[f2]) ok = 0;         // 全黑中换图
                        for (j = 1; j <= 15; j = j + 1)                                // 淡出 / 淡入曲线
                            if (fa[f2 - 1 - j] < fa[f2 - j] || fa[f2 + j] < fa[f2 + j - 1]) ok = 0;
                    end else begin
                        // 溶解 2 倍速：换图前一帧是 HOLD 全黑；之前 OUT 段档位 2,4..14（倒看），新图第一帧起
                        // IN 段 2,4..14（不出档位 0 的黑帧）；之后回 IDLE 全显
                        if (fb[f2 - 1] != 0 || fw[f2 - 1]) ok = 0;
                        j = f2 - 1; while (j > f0 && !fw[j]) j = j - 1;          // 跳过 HOLD，找 OUT 最后一帧
                        for (q = 0; q < 7; q = q + 1)
                            if (!fw[j - q] || fb[j - q] != 2 + 2 * q || fa[j - q] != 255) ok = 0;
                        for (j = 0; j < 7; j = j + 1)
                            if (!fw[f2 + j] || fb[f2 + j] != 2 + 2 * j || fa[f2 + j] != 255 || fc[f2 + j] != 32 + 32 * j) ok = 0;   // 画面不压暗，音频照常淡入
                        if (fw[f2 + 7] || fb[f2 + 7] != 16) ok = 0;
                    end
                    if (ok) nok = nok + 1;
                    else begin
                        $display("FAIL: change at frame %0d (%0d->%0d) manual=%0d", f2, fi[f2 - 1], fi[f2], manual);
                        for (j = f2 - 22; j < f2 + 18; j = j + 1)
                            $display("FAIL:      f%0d img=%0d a=%0d c=%0d d=%0d lvl=%0d", j, fi[j], fa[j], fc[j], fw[j], fb[j]);
                    end
                end
            end
        end
    endtask

    integer n0, g0, gb0, nchg, nok;
    initial begin
        repeat (10) @(posedge sys_clk);
        rst_n = 1;
        // 首图 + 预加载
        while (!ch.display_valid_sys || !cb.display_valid_sys) @(posedge sys_clk);
        frames(20);

        // ===== C2 手动：下一张 ×3，上一张 ×1 =====
        n0 = nf; g0 = goto_ch; gb0 = goto_cb;
        for (k = 0; k < 4; k = k + 1) begin
            key(k == 3 ? 1 : 0);
            // 连按（转场中重复按键要被丢弃）
            frames(3); key(0); frames(2); key(0);
            frames(45);
        end
        analyse(n0, nf, 1'b1, nchg, nok);
        if (nchg != 4 || nok != 4) begin errors = errors + 1; $display("FAIL: C2 manual changes=%0d ok=%0d", nchg, nok); end
        if (goto_ch - g0 != 4) begin errors = errors + 1; $display("FAIL: C2 goto count %0d (expect 4, repeats dropped)", goto_ch - g0); end
        if (ch.current_image_idx !== 6'd2) begin errors = errors + 1; $display("FAIL: C2 final image %0d expect 2", ch.current_image_idx); end
        $display("INFO: C2 manual next x3 + prev x1 -> 4 switches, all inside black frames, image=%0d, goto=%0d (bypass goto=%0d)",
                 ch.current_image_idx, goto_ch - g0, goto_cb - gb0);

        // ===== C5 旁路：同样按键全部生效、硬切 =====
        if (goto_cb - gb0 != 12) begin errors = errors + 1; $display("FAIL: C5 bypass goto count %0d expect 12", goto_cb - gb0); end

        // ===== C3 自动轮播 =====
        key(2);
        n0 = nf;
        frames(50 * 4 + 40);
        key(2);
        frames(30);
        analyse(n0, nf, 1'b0, nchg, nok);
        if (nchg < 3 || nok != nchg) begin errors = errors + 1; $display("FAIL: C3 auto changes=%0d ok=%0d", nchg, nok); end
        else $display("INFO: C3 auto slideshow: %0d switches, all random block dissolve x2 (out 14..2, black, in 2..14)", nchg);
        // 连续性：整段轮播里相邻两帧的可见档位差 ≤ 2（每帧最多变 2/16 的块，没有整幅跳变）
        s = 0;
        for (f = n0 + 1; f < nf; f = f + 1)
            if (fb[f] > fb[f - 1] + 2 || fb[f - 1] > fb[f] + 2) begin
                s = s + 1; if (s < 5) $display("FAIL: C3 visible level jump f%0d %0d -> %0d", f, fb[f - 1], fb[f]);
            end
        if (s != 0) errors = errors + s;
        else $display("INFO: C3 continuity: %0d frames, visible level never jumps more than 2/16 between frames", nf - n0);

        // ===== C4 策略层拒绝（图片数 1）=====
        image_count = 1;
        frames(2);
        g0 = ch.current_image_idx; n0 = nf;
        key(0);
        frames(60);
        if (ch.current_image_idx !== g0 || ch.u_fade.g_fade.state != 0 || ch.u_pix.alpha_q != 255) begin
            errors = errors + 1; $display("FAIL: C4 reject: stuck or image changed");
        end else begin
            s = 0; for (f = n0; f < nf; f = f + 1) if (fa[f] == 0) s = s + 1;
            $display("INFO: C4 rejected switch: faded out, %0d black frames, faded back in, image unchanged", s);
        end
        image_count = 4;

        frames(5);
        $display("INFO: frames=%0d pixels checked=%0d", nf, px);
        if (errors == 0) $display("PASS: tb_transition_chain");
        else $display("FAIL: tb_transition_chain errors=%0d", errors);
        $finish;
    end
    initial begin #2_000_000_000; $display("FAIL: timeout"); $finish; end
endmodule
