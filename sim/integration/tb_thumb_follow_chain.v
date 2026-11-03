`timescale 1ns/1ps

module tb_thumb_follow_chain;
    reg sys_clk = 0, pix_clk = 0, rst_n = 0;
    always #10 sys_clk = ~sys_clk;
    always #20 pix_clk = ~pix_clk;

    reg frame_start = 0;
    integer pcnt = 0;
    always @(posedge pix_clk) begin
        pcnt <= (pcnt == 299) ? 0 : pcnt + 1;
        frame_start <= (pcnt == 299);
    end

    integer errors = 0;
    task fail(input [8*80-1:0] msg); begin errors = errors + 1; if (errors < 40) $display("FAIL %0s @%0t", msg, $time); end endtask

    // ---------------- 激励 ----------------
    reg sw4 = 1'b1;                          // F6：上电前已拨上
    reg sw4_q = 1'b0;                        // 同顶层 sw4_stable_n_q 复位 = 拨下：复位后已拨上 → 出一个拨上沿
    reg key2_click = 0, key2_long = 0, key_auto = 0;
    always @(posedge sys_clk) sw4_q <= rst_n ? sw4 : 1'b0;
    wire thumb_key_mode    =  sw4 && !sw4_q;
    wire thumb_key_confirm = !sw4 &&  sw4_q;
    reg  [6:0] image_count = 7'd4;

    // ---------------- loader 模型 ----------------
    wire        load_request;
    wire [5:0]  load_image_idx;
    wire [1:0]  load_buffer_idx;
    wire        load_thumb_gen;
    reg  [63:0] gen_seen = 0;                // 图片 i 有过 gen=1 的加载
    integer     n_load = 0;
    reg         ld_busy = 0;
    reg  [5:0]  bank_c [0:3];                // F14：各 bank 里装的图
    integer     n_err = 0;
    integer     ld_cnt = 0;
    reg  [5:0]  ld_img = 0;
    reg  [1:0]  ld_buf = 0;
    reg         load_done = 0;
    reg  [5:0]  loaded_image_idx = 0;
    reg  [1:0]  loaded_buffer_idx = 0;
    reg  [3:0]  image_complete = 0;
    always @(posedge sys_clk) begin
        load_done <= 0;
        if (!rst_n) begin image_complete <= 0; ld_busy <= 0; end
        else if (load_request) begin
            ld_busy <= 1; ld_cnt <= 2000; ld_img <= load_image_idx; ld_buf <= load_buffer_idx;
            image_complete[load_buffer_idx] <= 1'b0;
            n_load = n_load + 1;
            if (load_thumb_gen) gen_seen[load_image_idx] <= 1'b1;
        end else if (ld_busy) begin
            if (ld_cnt == 0) begin
                ld_busy <= 0; load_done <= 1; loaded_image_idx <= ld_img; loaded_buffer_idx <= ld_buf;
                image_complete[ld_buf] <= 1'b1;
                bank_c[ld_buf] = ld_img;
            end else ld_cnt <= ld_cnt - 1;
        end
    end

    // ---------------- 链路（同顶层）----------------
    wire goto_valid, toggle_auto_event, period_cfg_valid, auto_tick, auto_enabled;
    wire [5:0] goto_image_idx, current_image_idx, active_image_idx;
    wire [1:0] goto_audio_idx, active_buffer_idx, active_audio_idx;
    wire [31:0] period_cfg_cycles;
    wire display_valid_sys, error_event, request_busy, display_valid, index_commit;
    wire fade_next_event, fade_next_is_prev, fade_busy, fade_active, dissolve_active;
    wire [7:0] fade_alpha;
    wire [4:0] fade_ramp;
    wire thumb_nav_enable, thumb_view_pix, thumb_view_sys, thumb_view_busy, thumb_view_hold;

    // 总览翻页规则（同顶层）
    reg        tv_s1 = 0, thumb_view_back = 0, thumb_view_settled = 0;
    always @(posedge pix_clk) if (!u_thumb_zoom.zoom_active && (u_thumb_zoom.thumb_display == thumb_view_pix)) thumb_view_settled <= thumb_view_pix;
    always @(posedge sys_clk) begin tv_s1 <= thumb_view_settled; thumb_view_back <= tv_s1; end
    wire       thumb_nav_ok      = thumb_nav_enable && (thumb_view_sys == thumb_view_back);
    wire [6:0] cur_plus1         = {1'b0, current_image_idx} + 7'd1;
    wire       thumb_next_cross  = (image_count > 7'd4) && ((current_image_idx[1:0] == 2'd3) || (cur_plus1 >= image_count));
    wire       thumb_prev_cross  = (image_count > 7'd4) && (current_image_idx[1:0] == 2'd0);
    wire       thumb_key_in      = thumb_view_sys && ((key2_click && !thumb_next_cross) || (key2_long && !thumb_prev_cross));
    wire       thumb_direct_ok   = thumb_nav_ok && !request_busy && !fade_busy;
    wire       thumb_direct_next = key2_click && thumb_view_sys && !thumb_next_cross && thumb_direct_ok;
    wire       thumb_direct_prev = key2_long  && thumb_view_sys && !thumb_prev_cross && thumb_direct_ok;
    reg  [3:0] commit_page_q = 0;
    always @(posedge pix_clk) commit_page_q <= active_image_idx[5:2];
    wire       fade_index_commit = index_commit && (!thumb_view_pix || (active_image_idx[5:2] != commit_page_q));
    // 轮播分流（同顶层）：总览页内直通策略层，其余经转场控制器方块溶解；按键同拍按键优先
    wire       auto_req          = auto_tick && thumb_nav_ok;
    wire       auto_in_page      = thumb_view_sys && !thumb_next_cross;
    wire       auto_direct       = auto_req && auto_in_page && !fade_busy;
    wire       key_fade_ev       = (key2_click | key2_long) & thumb_nav_ok & ~thumb_key_in;

    video_fade_transition #(.FADE_FRAMES(16), .HOLD_TIMEOUT_FRAMES(480)) u_fade (
        .ctrl_clk(sys_clk), .ctrl_rst_n(rst_n),
        .next_event_in(key_fade_ev | (auto_req & ~auto_in_page)), .next_tag_in(key2_long), .next_auto_in(~key_fade_ev),
        .next_block_in(request_busy), .hold_abort_in(error_event),
        .next_event_out(fade_next_event), .next_tag_out(fade_next_is_prev), .fade_busy(fade_busy),
        .pix_clk(pix_clk), .pix_rst_n(rst_n), .player_active(display_valid), .frame_start(frame_start),
        .index_commit(fade_index_commit),
        .alpha_out(fade_alpha), .fade_active(fade_active), .dissolve_active(dissolve_active),
        .ramp_out(fade_ramp));

    media_policy #(.CLK_HZ(50_000_000)) u_policy (
        .clk(sys_clk), .rst_n(rst_n), .enable(display_valid_sys),
        .next_event((fade_next_event & ~fade_next_is_prev) | thumb_direct_next),
        .prev_event((fade_next_event &  fade_next_is_prev) | thumb_direct_prev),
        .auto_event(key_auto), .bright_event(1'b0), .vol_event(1'b0), .period_event(1'b0), .save_event(1'b0),
        .auto_tick(auto_direct),
        .adj_inc(1'b0), .adj_dec(1'b0), .adj_default(1'b0), .adj_sel(1'b0), .adj_show(1'b0),
        .ir_present(1'b0), .disp_brightness(8'd255), .bri_base(8'd255), .env_change(1'b0),
        .current_image_idx(current_image_idx), .image_count(image_count), .auto_enabled(auto_enabled),
        .cfg_load_valid(1'b0), .cfg_period_sel(3'd0), .cfg_brightness(9'd0), .cfg_contrast(8'd128), .cfg_volume(8'd0), .cfg_auto_mode(1'b0),
        .goto_valid(goto_valid), .goto_image_idx(goto_image_idx), .goto_audio_idx(goto_audio_idx),
        .toggle_auto_event(toggle_auto_event), .period_cfg_valid(period_cfg_valid), .period_cfg_cycles(period_cfg_cycles),
        .media_id(), .audio_id(), .linked_event(), .bri_offset(), .contrast(), .volume(), .period_tenths(),
        .pic_event(), .vol_page_event(), .prd_event(), .bri_event(), .con_event(), .save_done_event(),
        .saved_period_sel(), .saved_brightness(), .saved_contrast(), .saved_volume(), .saved_auto_mode(), .saved_valid());

    media_player_control #(
        .DEFAULT_PERIOD_CYCLES (600 * 60),          // 60 帧
        .MIN_PERIOD_CYCLES     (100),
        .MAX_PERIOD_CYCLES     (32'h7fff_ffff),
        .LOOKAHEAD             (1)
    ) u_ctrl (
        .sys_clk(sys_clk), .sys_rst_n(rst_n), .tick(1'b1),
        .goto_valid(goto_valid), .goto_image_idx(goto_image_idx), .goto_audio_idx(goto_audio_idx),
        .toggle_auto_event(toggle_auto_event), .period_cfg_valid(period_cfg_valid), .period_cfg_cycles(period_cfg_cycles),
        .auto_tick(auto_tick), .scan_done(1'b1), .image_count(image_count), .loader_ready(!ld_busy),
        .load_done(load_done), .loaded_image_idx(loaded_image_idx), .loaded_buffer_idx(loaded_buffer_idx),
        .load_error(8'd0), .image_complete(image_complete), .fast_view(thumb_view_sys && thumb_view_back), .view_hold(thumb_view_hold),
        .load_request(load_request), .load_image_idx(load_image_idx), .load_buffer_idx(load_buffer_idx),
        .load_thumb_gen(load_thumb_gen), .auto_enabled(auto_enabled), .active_period_cycles(), .current_image_idx(current_image_idx),
        .image_committed(), .display_valid_sys(display_valid_sys), .control_error(), .error_event(error_event),
        .request_busy(request_busy), .bank_resident(),
        .pix_clk(pix_clk), .pix_rst_n(rst_n), .frame_boundary(frame_start),
        .active_image_idx(active_image_idx), .active_buffer_idx(active_buffer_idx), .active_audio_idx(active_audio_idx),
        .display_valid(display_valid), .index_commit(index_commit));

    thumbnail_controller #(.FOCUS_FOLLOW(1)) u_thumb_ctrl (
        .clk(sys_clk), .rst_n(rst_n), .enable(display_valid_sys),
        .key_right(key2_click), .key_left(key2_long), .key_confirm(thumb_key_confirm), .key_mode(thumb_key_mode),
        .image_complete(4'hF), .current_image_idx(current_image_idx), .image_count(image_count),
        .select_busy(request_busy | fade_busy | thumb_view_hold), .select_error(error_event),
        .view_mode(thumb_view_sys), .focus_index(), .active_index(), .transition_req(), .select_valid(), .select_image_idx(),
        .select_reject(), .nav_enable(thumb_nav_enable), .view_busy(thumb_view_busy),
        .pix_clk(pix_clk), .pix_rst_n(rst_n), .frame_start(frame_start),
        .view_mode_pix(thumb_view_pix), .focus_pix(), .view_commit());

    wire        thumb_display, zoom_active;
    wire [7:0]  zoom_alpha;
    wire [11:0] zoom_x0, zoom_x1, zoom_y0, zoom_y1;
    thumb_zoom_transition #(.X_OFFSET(160), .Y_OFFSET(45), .A_MID(8'd64), .STEPS(12), .FULL_GRID(1)) u_thumb_zoom (
        .clk(pix_clk), .rst_n(rst_n), .frame_start(frame_start), .display_valid(display_valid),
        .sw_thumb(thumb_view_pix), .focus(active_image_idx[1:0]),
        .thumb_display(thumb_display), .zoom_active(zoom_active), .zoom_alpha(zoom_alpha),
        .clip_x0(zoom_x0), .clip_x1(zoom_x1), .clip_y0(zoom_y0), .clip_y1(zoom_y1));

    wire [7:0] view_alpha   = zoom_active ? zoom_alpha : (fade_alpha | {8{dissolve_active}});
    wire [2:0] reader_index = thumb_display ? {2'b10, active_image_idx[2]} : {1'b0, active_buffer_idx};

    // ---------------- F10 全程监视：总览中页内提交后 4 帧内不得有溶解 / 淡入淡出 ----------------
    integer in_page_watch = 0, n_ov_wipe = 0, n_inpage = 0;
    reg wipe_q = 0; reg [7:0] fa_q = 8'd255;
    always @(posedge key2_click or posedge key2_long) in_page_watch = 0;   // 新按键（如紧接着跨页）自己会开淡出，不算页内提交引起
    always @(posedge pix_clk) begin
        if (rst_n) begin
            if (index_commit && thumb_view_pix && thumb_display && (active_image_idx[5:2] == commit_page_q)) begin
                in_page_watch = 4; n_inpage = n_inpage + 1;
            end else if (frame_start && in_page_watch > 0) in_page_watch = in_page_watch - 1;
            // 只抓页内提交之后"新开始"的转场（跨页转场进行中又来一次同页补提交不算）
            if (in_page_watch > 0 && ((dissolve_active && !wipe_q) || (fade_alpha != 8'd255 && fa_q == 8'd255))) begin
                fail("F10 in-page overview commit started a transition");
                $display("  dissolve=%b fade_alpha=%0d img=%0d", dissolve_active, fade_alpha, active_image_idx);
                in_page_watch = 0;
            end
            if (dissolve_active && !wipe_q && thumb_display) n_ov_wipe = n_ov_wipe + 1;
        end
        wipe_q <= dissolve_active; fa_q <= fade_alpha;
    end

    // ---------------- F1 / F7 全局监视 ----------------
    reg [63:0] pq = 0; reg fs_q = 0; reg ic_q = 0;
    reg za_q = 0, td_q = 0;
    reg       exp_valid = 0;               // F2：外扩起点期望的图片
    reg [5:0] exp_img = 0;
    integer   n_expand = 0, n_shrink = 0;
    wire [63:0] snap = {thumb_display, reader_index, active_image_idx, zoom_active, zoom_alpha, fade_alpha,
                        zoom_x0, zoom_x1, zoom_y0, zoom_y1[5:0]};
    always @(posedge pix_clk) begin
        if (rst_n) begin
            // 转场控制器在 index_commit 下一拍进 IN（溶解从档位 2 起步，alpha 32），仍在帧消隐期
            if (snap !== pq && !fs_q && !ic_q) begin fail("F1 frame-level value changed outside frame_start"); $display("  %h -> %h", pq, snap); end
            if (zoom_active && !za_q) begin
                if (fade_alpha !== 8'd255) begin fail("F7 zoom started while fade not idle"); $display("  fade_alpha=%0d", fade_alpha); end
                if (td_q) begin
                    n_expand = n_expand + 1;
                    if (exp_valid && active_image_idx !== exp_img) begin
                        fail("F2 expand started from old image"); $display("  active=%0d expect=%0d", active_image_idx, exp_img);
                    end
                end else n_shrink = n_shrink + 1;
            end
        end
        pq <= snap; fs_q <= frame_start; ic_q <= index_commit; za_q <= zoom_active; td_q <= thumb_display;
    end

    integer n_goto = 0, n_commit = 0;
    always @(posedge sys_clk) if (goto_valid) n_goto = n_goto + 1;
    always @(posedge sys_clk) if (error_event) begin n_err = n_err + 1; $display("INFO error_event code=%h @%0t", u_ctrl.control_error, $time); end
    always @(posedge pix_clk) if (index_commit) n_commit = n_commit + 1;

    // ---------------- 工具 ----------------
    // 释放后再等几拍：总览页内直通时 goto → request_busy 有 2~3 拍流水，之后 wait_idle 才看得到忙
    task click; begin @(posedge sys_clk); #1 key2_click = 1; @(posedge sys_clk); #1 key2_click = 0; repeat (6) @(posedge sys_clk); #1; end endtask
    task longp; begin @(posedge sys_clk); #1 key2_long  = 1; @(posedge sys_clk); #1 key2_long  = 0; repeat (6) @(posedge sys_clk); #1; end endtask
    task pulse_auto; begin @(posedge sys_clk); #1 key_auto = 1; @(posedge sys_clk); #1 key_auto = 0; end endtask
    task frames(input integer n); integer i; begin for (i = 0; i < n; i = i + 1) @(posedge frame_start); #1; end endtask
    // 等到稳定在总览 / 全屏且无动画（最多 n 帧）
    task wait_thumb(input integer n); integer i; begin
        for (i = 0; i < n && !(thumb_display && !zoom_active); i = i + 1) frames(1);
        frames(1);
    end endtask
    task wait_full(input integer n); integer i; begin
        for (i = 0; i < n && !(!thumb_display && !zoom_active && thumb_view_pix == 0); i = i + 1) frames(1);
        frames(1);
    end endtask
    task wait_idle(input integer n); integer i; begin
        for (i = 0; i < n && (fade_busy || request_busy || fade_alpha != 8'd255); i = i + 1) frames(1);
        frames(1);
    end endtask
    // F2 后：外扩完成后 n 帧内 view_alpha 必须一直 255
    task check_no_black(input integer n); integer i; begin
        for (i = 0; i < n; i = i + 1) begin
            frames(1);
            if (view_alpha !== 8'd255) begin fail("F2 black / fade after expand"); $display("  view_alpha=%0d", view_alpha); i = n; end
        end
    end endtask

    integer i, g0, c0, s0, e0, a0;
    reg [5:0] exp_seq;
    initial begin
        #200; rst_n = 1;

        // ===== F6 上电已拨上 =====
        wait (display_valid_sys);
        wait_thumb(80);
        if (!thumb_display || active_image_idx !== 0) fail("F6 auto enter overview after first image");
        if (n_shrink != 1) fail("F6 exactly one shrink");
        frames(30);                                  // 预加载 1..3 完成

        // ===== F3 总览焦点回绕 =====
        exp_seq = 0;
        for (i = 0; i < 4; i = i + 1) begin
            click; wait_idle(60);
            exp_seq = (exp_seq + 1) % 4;
            if (active_image_idx !== exp_seq || !thumb_display) begin fail("F3 right wrap"); $display("  got %0d exp %0d", active_image_idx, exp_seq); end
            if (view_alpha !== 8'd255) fail("F3 overview alpha");
        end
        for (i = 0; i < 4; i = i + 1) begin
            longp; wait_idle(60);
            exp_seq = (exp_seq + 3) % 4;
            if (active_image_idx !== exp_seq || !thumb_display) begin fail("F3 left wrap"); $display("  got %0d exp %0d", active_image_idx, exp_seq); end
        end
        if (n_expand != 0) fail("F3 overview must not exit on KEY2");

        // ===== F2 切图在途拨下 =====
        //总览页内 KEY2 直通（不淡出），切图在途 = goto 后、帧边界提交前：同一拍拨下 SW4
        g0 = n_goto; e0 = n_expand;
        exp_img = (active_image_idx + 1) % 4; exp_valid = 1;
        click; #1 sw4 = 0;
        repeat (4) @(posedge sys_clk); #1;
        if (!request_busy) fail("F2 switch should be in flight");
        if (fade_busy) fail("F10 in-page KEY2 must bypass fade");
        frames(2);
        // F8：退出等待中 KEY2 被屏蔽
        click; frames(2); longp;
        wait_full(120);
        if (n_expand != e0 + 1) fail("F2 one expand");
        if (active_image_idx !== exp_img) fail("F2 final image");
        if (n_goto != g0 + 1) begin fail("F8 KEY2 during exit-pending must be blocked"); $display("  goto +%0d", n_goto - g0); end
        exp_valid = 0;
        check_no_black(40);

        // ===== F4 全屏 KEY2 淡出淡入 =====
        a0 = active_image_idx; c0 = n_commit;
        click;
        begin : f4_out
            for (i = 0; i < 40 && fade_alpha != 0; i = i + 1) frames(1);
        end
        if (fade_alpha !== 8'd0) fail("F4 fade out reaches black");
        wait_idle(80);
        if (active_image_idx !== (a0 + 1) % 4 || n_commit != c0 + 1) fail("F4 next committed");
        if (fade_alpha !== 8'd255) fail("F4 fade back in");
        // 拨上落在淡出中途
        s0 = n_shrink; a0 = active_image_idx;
        click; frames(4);
        #1 sw4 = 1;
        frames(4);
        if (zoom_active || thumb_display) fail("F4 shrink must wait for fade");
        wait_thumb(120);
        if (n_shrink != s0 + 1 || active_image_idx !== (a0 + 1) % 4) fail("F4 shrink after fade with new image");
        #1 sw4 = 0; wait_full(60);

        // ===== F5 自动轮播 =====
        pulse_auto; frames(2);
        if (!auto_enabled) fail("F5 auto enabled");
        c0 = n_commit; frames(200);
        if (n_commit < c0 + 2) fail("F5 auto play in fullscreen");
        #1 sw4 = 1; wait_thumb(120);
        c0 = n_commit; frames(200);
        if (n_commit < c0 + 2) fail("F5 auto play keeps moving focus in overview (lesson 12 behaviour)");
        if (!thumb_display) fail("F5 still overview");
        pulse_auto; frames(2);
        wait_idle(80);
        #1 sw4 = 0; wait_full(120);
        check_no_black(20);

        // ===== 10 张图：前瞻缓存 + 总览翻页 =====
        image_count = 7'd10;
        frames(60);
        // F9 全屏走到第 3 张，停留后跨页到第 4 张
        for (i = 0; i < 4 && active_image_idx != 3; i = i + 1) begin click; wait_idle(80); end
        if (active_image_idx !== 3) fail("F9 reach image 3");
        frames(60);                                  // 停留：前瞻把第 4 张装进 bank
        g0 = n_goto; c0 = n_commit;
        click;
        wait (n_goto != g0); s0 = n_load;
        wait (n_commit != c0); e0 = n_load;
        if (e0 != s0) begin fail("F9 3->4 should hit lookahead cache (no load between goto and commit)"); $display("  loads %0d", e0 - s0); end
        wait_idle(80);
        if (active_image_idx !== 4) fail("F9 image 4 committed");
        frames(40);
        // F12（一部分）：停留后当前页 4..7 / 下一页 8..9 都有过 gen=1 的加载
        if (gen_seen[9:4] !== 6'b111111) begin fail("F12 thumbnails of current / next page generated"); $display("  gen_seen=%b", gen_seen[11:0]); end

        // 进总览（第 1 页）
        #1 sw4 = 1; wait_thumb(120);
        if (!thumb_display || reader_index !== 3'd5) begin fail("F11 overview page 1 reads area B"); $display("  reader=%0d", reader_index); end
        // F10 页内 4→5→6→7：不淡出、几帧内提交
        for (i = 5; i <= 7; i = i + 1) begin
            c0 = n_commit;
            click;
            begin : f10_wait
                integer k;
                for (k = 0; k < 4 && n_commit == c0; k = k + 1) frames(1);
            end
            if (n_commit == c0 || active_image_idx !== i) begin fail("F10 in-page move within 4 frames"); $display("  active=%0d exp=%0d", active_image_idx, i); end
            if (fade_busy || fade_alpha !== 8'd255) fail("F10 in-page move must not fade");
            frames(10);
        end
        // F11 跨页 7→8：淡出到 0 → 第 2 页 → 淡入
        click;
        begin : f11_out
            for (i = 0; i < 40 && fade_alpha != 0; i = i + 1) frames(1);
        end
        if (fade_alpha !== 8'd0) fail("F11 cross-page KEY2 fades out");
        wait_idle(120);
        if (active_image_idx !== 8 || !thumb_display || fade_alpha !== 8'd255 || reader_index !== 3'd4) begin
            fail("F11 page 2 shown, faded in"); $display("  active=%0d td=%0d alpha=%0d reader=%0d", active_image_idx, thumb_display, fade_alpha, reader_index);
        end
        // F11 上一张跨页 8→7
        longp;
        begin : f11_out2
            for (i = 0; i < 40 && fade_alpha != 0; i = i + 1) frames(1);
        end
        if (fade_alpha !== 8'd0) fail("F11 cross-page long press fades out");
        wait_idle(120);
        if (active_image_idx !== 7 || !thumb_display || fade_alpha !== 8'd255) fail("F11 back to page 1");

        // F12 总览自动轮播：跨页走方块溶解（页内提交由 F10 监视器检查）
        a0 = n_ov_wipe; s0 = n_inpage;
        pulse_auto; frames(2);
        if (!auto_enabled) fail("F12 auto enabled");
        frames(700);
        if (n_ov_wipe == a0) fail("F12 auto cross-page in overview should dissolve");
        if (n_inpage == s0) fail("F12 auto in-page moves expected");
        if (!thumb_display) fail("F12 still overview");
        pulse_auto; frames(2);
        wait_idle(120);
        #1 sw4 = 0; wait_full(120);
        check_no_black(20);

        // F14 上板问题复现：总览里 KEY2 连按（每 2 帧一次，比读一张图快得多，含跨页），全程不得报错；
        //     页内每次都立即动；随后拨下 SW4：外扩出来的是当前图，且显示的 bank 里确实整张是它
        #1 sw4 = 1; wait_thumb(120);
        e0 = n_err; c0 = n_commit; g0 = 0;
        for (i = 0; i < 14; i = i + 1) begin
            a0 = current_image_idx;
            if (thumb_next_cross) begin                  // 跨页：淡出淡入，等它走完
                click; wait_idle(120);
            end else begin
                click; frames(2);
                if (current_image_idx === a0) begin fail("F14 in-page rapid move did not respond"); $display("  cur=%0d", a0); end
            end
            if (current_image_idx !== a0) g0 = g0 + 1;
        end
        if (n_err != e0) begin fail("F14 rapid overview moves raised error"); $display("  errors=%0d", n_err - e0); end
        if (g0 < 12) begin fail("F14 moves accepted"); $display("  moved %0d / 14", g0); end
        a0 = current_image_idx;
        #1 sw4 = 0; wait_full(400);
        if (active_image_idx !== a0) fail("F14 exit shows the focused image");
        if (bank_c[active_buffer_idx] !== active_image_idx) begin
            fail("F14 full-screen bank holds the focused image"); $display("  bank%0d=%0d active=%0d", active_buffer_idx, bank_c[active_buffer_idx], active_image_idx);
        end
        check_no_black(20);

        // F13 页数为奇数（10 张 = 3 页）：停在末页（第 8 / 9 张），前瞻填满后不得再有加载
        //     （末页与回绕邻页第 0 页同一块缩略图区，不能互相覆盖、反复重写）
        for (i = 0; i < 12 && active_image_idx != 9; i = i + 1) begin click; wait_idle(80); end
        if (active_image_idx !== 9) fail("F13 reach image 9");
        frames(150);
        s0 = n_load; frames(150);
        if (n_load != s0) begin fail("F13 lookahead must settle on last page (no thrash)"); $display("  loads +%0d", n_load - s0); end
        if (!u_ctrl.th_v[0] || !u_ctrl.th_v[1] || u_ctrl.th_t0 !== 3'd1 || u_ctrl.th_t1 !== 3'd1)
            fail("F13 thumbnails of images 8/9 stay valid");

        if (errors == 0) $display("PASS tb_thumb_follow_chain (goto=%0d commit=%0d shrink=%0d expand=%0d loads=%0d ov_wipe=%0d)", n_goto, n_commit, n_shrink, n_expand, n_load, n_ov_wipe);
        else $display("FAIL tb_thumb_follow_chain errors=%0d", errors);
        $finish;
    end
    initial begin #400_000_000; $display("FAIL tb_thumb_follow_chain timeout"); $finish; end
endmodule
