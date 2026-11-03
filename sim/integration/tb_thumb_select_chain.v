`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// tb_thumb_select_chain —— 快速选择集成验证
//   thumbnail_controller → select_valid / select_image_idx ─┐
//   按键下一张 → media_policy → goto ──────────────────────┴→ media_player_control（真实 RTL：
//     bank 命中 / 未命中加载、commit_toggle / ack、frame_boundary 原子提交）→ active_image_idx / index_commit
//   loader 行为模型（2000 拍一张，fail_mask 的图加载报 0x56）；缩略图格子完整 = 该图成功加载过且在当前页
//   goto = policy.goto | select；缩略图中暂停轮播、方向键不进策略层
// 检查：
//   S1 active_image_idx / active_buffer_idx 只在 frame_start 提交（index_commit 同拍）；view_mode_pix 同样
//   S2 快速选择已加载格子：只有 1 个 goto，目标 = 页·4 + 焦点；目标先在帧边界提交，view_mode_pix 才回 0
//      （回全屏的第一帧显示的就是目标图，不会先回旧图）
//   S3 选择未加载格子（预加载失败的图）：select_reject，无 goto，active 不变，留在缩略图
//   S4 选择不在 bank 里的图（第 2 页，需要加载）：等加载完成提交后才回全屏，期间 request_busy=1
//   S5 全屏下一张、自动轮播照常工作；缩略图中轮播暂停（不提交），回全屏后恢复
//   S6 控制器 view_mode 与 media_player_control 的 active 没有多驱动：select 只走 goto 一条路
// -----------------------------------------------------------------------------
module tb_thumb_select_chain;
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
    reg key_next = 0, key_auto = 0, k_r = 0, k_l = 0, k_c = 0, k_m = 0;
    reg [6:0]  image_count = 7'd12;
    reg [63:0] fail_mask = 64'd0;

    // ---------------- loader 模型 ----------------
    wire        load_request;
    wire [5:0]  load_image_idx;
    wire [1:0]  load_buffer_idx;
    reg         ld_busy = 0;
    integer     ld_cnt = 0;
    reg  [5:0]  ld_img = 0;
    reg  [1:0]  ld_buf = 0;
    reg         load_done = 0;
    reg  [7:0]  load_error = 0;
    reg  [5:0]  loaded_image_idx = 0;
    reg  [1:0]  loaded_buffer_idx = 0;
    reg  [3:0]  image_complete = 0;
    reg  [63:0] loaded_once = 0;           // 缩略图写入器模型：成功加载过的图都有缩略图
    always @(posedge sys_clk) begin
        load_done <= 0; load_error <= 0;
        if (!rst_n) begin image_complete <= 0; ld_busy <= 0; loaded_once <= 0; end
        else if (load_request) begin
            ld_busy <= 1; ld_cnt <= 2000; ld_img <= load_image_idx; ld_buf <= load_buffer_idx;
            image_complete[load_buffer_idx] <= 1'b0;
        end else if (ld_busy) begin
            if (ld_cnt == 0) begin
                ld_busy <= 0;
                if (fail_mask[ld_img]) load_error <= 8'h56;
                else begin
                    load_done <= 1; loaded_image_idx <= ld_img; loaded_buffer_idx <= ld_buf;
                    image_complete[ld_buf] <= 1'b1; loaded_once[ld_img] <= 1'b1;
                end
            end else ld_cnt <= ld_cnt - 1;
        end
    end

    // ---------------- 控制链 ----------------
    wire goto_valid_p, toggle_auto_event, period_cfg_valid, auto_tick, auto_enabled;
    wire [5:0] goto_idx_p, current_image_idx, active_image_idx;
    wire [1:0] goto_aud_p, active_buffer_idx, active_audio_idx;
    wire [31:0] period_cfg_cycles;
    wire display_valid_sys, error_event, request_busy, display_valid, index_commit;
    wire vm, tr, sv, rj, nav, vb, vm_pix, vc;
    wire [1:0] fi, ai, fp;
    wire [5:0] si;

    wire [3:0] page_cells = { loaded_once[{current_image_idx[5:2], 2'd3}], loaded_once[{current_image_idx[5:2], 2'd2}],
                              loaded_once[{current_image_idx[5:2], 2'd1}], loaded_once[{current_image_idx[5:2], 2'd0}] };

    thumbnail_controller #(.FOCUS_FOLLOW(0)) u_tc (
        .clk(sys_clk), .rst_n(rst_n), .enable(display_valid_sys),
        .key_right(k_r), .key_left(k_l), .key_confirm(k_c), .key_mode(k_m),
        .image_complete(page_cells), .current_image_idx(current_image_idx), .image_count(image_count),
        .select_busy(request_busy), .select_error(error_event),
        .view_mode(vm), .focus_index(fi), .active_index(ai), .transition_req(tr),
        .select_valid(sv), .select_image_idx(si), .select_reject(rj), .nav_enable(nav), .view_busy(vb),
        .pix_clk(pix_clk), .pix_rst_n(rst_n), .frame_start(frame_start),
        .view_mode_pix(vm_pix), .focus_pix(fp), .view_commit(vc));

    media_policy #(.CLK_HZ(50_000_000)) u_policy (
        .clk(sys_clk), .rst_n(rst_n), .enable(display_valid_sys),
        .next_event(key_next & nav), .prev_event(1'b0),
        .auto_event(key_auto), .bright_event(1'b0), .vol_event(1'b0), .period_event(1'b0), .save_event(1'b0),
        .auto_tick(auto_tick & ~vm),
        .adj_inc(1'b0), .adj_dec(1'b0), .adj_default(1'b0), .adj_sel(1'b0), .adj_show(1'b0),
        .ir_present(1'b0), .disp_brightness(8'd255), .bri_base(8'd255), .env_change(1'b0),
        .current_image_idx(current_image_idx), .image_count(image_count), .auto_enabled(auto_enabled),
        .cfg_load_valid(1'b0), .cfg_period_sel(3'd0), .cfg_brightness(9'd0), .cfg_contrast(8'd128), .cfg_volume(8'd0), .cfg_auto_mode(1'b0),
        .goto_valid(goto_valid_p), .goto_image_idx(goto_idx_p), .goto_audio_idx(goto_aud_p),
        .toggle_auto_event(toggle_auto_event), .period_cfg_valid(period_cfg_valid), .period_cfg_cycles(period_cfg_cycles),
        .media_id(), .audio_id(), .linked_event(), .bri_offset(), .contrast(), .volume(), .period_tenths(),
        .pic_event(), .vol_page_event(), .prd_event(), .bri_event(), .con_event(), .save_done_event(),
        .saved_period_sel(), .saved_brightness(), .saved_contrast(), .saved_volume(), .saved_auto_mode(), .saved_valid());

    // 直接索引选择：select 与策略层 goto 合并成一条（两者互斥：缩略图中策略层不出 goto）
    wire       goto_valid = goto_valid_p | sv;
    wire [5:0] goto_idx   = sv ? si : goto_idx_p;
    wire [1:0] goto_aud   = sv ? si[1:0] : goto_aud_p;

    media_player_control #(
        .DEFAULT_PERIOD_CYCLES (600 * 40),          // 40 帧
        .MIN_PERIOD_CYCLES     (100),
        .MAX_PERIOD_CYCLES     (32'h7fff_ffff)
    ) u_ctrl (
        .sys_clk(sys_clk), .sys_rst_n(rst_n), .tick(1'b1),
        .goto_valid(goto_valid), .goto_image_idx(goto_idx), .goto_audio_idx(goto_aud),
        .toggle_auto_event(toggle_auto_event), .period_cfg_valid(period_cfg_valid), .period_cfg_cycles(period_cfg_cycles),
        .auto_tick(auto_tick), .scan_done(1'b1), .image_count(image_count), .loader_ready(!ld_busy),
        .load_done(load_done), .loaded_image_idx(loaded_image_idx), .loaded_buffer_idx(loaded_buffer_idx),
        .load_error(load_error), .image_complete(image_complete),
        .load_request(load_request), .load_image_idx(load_image_idx), .load_buffer_idx(load_buffer_idx),
        .auto_enabled(auto_enabled), .active_period_cycles(), .current_image_idx(current_image_idx),
        .image_committed(), .display_valid_sys(display_valid_sys), .control_error(), .error_event(error_event),
        .request_busy(request_busy), .bank_resident(),
        .pix_clk(pix_clk), .pix_rst_n(rst_n), .frame_boundary(frame_start),
        .active_image_idx(active_image_idx), .active_buffer_idx(active_buffer_idx), .active_audio_idx(active_audio_idx),
        .display_valid(display_valid), .index_commit(index_commit));

    // ---------------- S1：帧边界原子性 ----------------
    reg [8:0] pq = 0; reg fs_q = 0, vmp_q = 0;
    reg       exp_target_valid = 0;
    reg [5:0] exp_target = 0;
    always @(posedge pix_clk) begin
        if (rst_n) begin
            if ({active_image_idx, active_buffer_idx, vm_pix} !== pq && !fs_q) fail("S1 active/view changed outside frame_start");
            if (index_commit && !fs_q) fail("S1 index_commit outside frame_start");
            // S2：回全屏那一帧，active 必须已经是选中的目标
            if (vmp_q && !vm_pix && exp_target_valid && active_image_idx !== exp_target) begin
                fail("S2 returned to fullscreen before target committed");
                $display("  active=%0d target=%0d", active_image_idx, exp_target);
            end
        end
        pq <= {active_image_idx, active_buffer_idx, vm_pix}; fs_q <= frame_start; vmp_q <= vm_pix;
    end

    integer n_goto = 0, n_commit = 0, n_sv = 0, n_rj = 0;
    always @(posedge sys_clk) begin
        if (goto_valid) n_goto = n_goto + 1;
        if (sv) n_sv = n_sv + 1;
        if (rj) n_rj = n_rj + 1;
        if (goto_valid_p && sv) fail("S6 policy goto and select in the same cycle");
    end
    always @(posedge pix_clk) if (index_commit) n_commit = n_commit + 1;

    task pulse_r; begin @(posedge sys_clk); #1 k_r = 1; @(posedge sys_clk); #1 k_r = 0; end endtask
    task pulse_c; begin @(posedge sys_clk); #1 k_c = 1; @(posedge sys_clk); #1 k_c = 0; end endtask
    task pulse_m; begin @(posedge sys_clk); #1 k_m = 1; @(posedge sys_clk); #1 k_m = 0; end endtask
    task pulse_next; begin @(posedge sys_clk); #1 key_next = 1; @(posedge sys_clk); #1 key_next = 0; end endtask
    task pulse_auto; begin @(posedge sys_clk); #1 key_auto = 1; @(posedge sys_clk); #1 key_auto = 0; end endtask
    task frames(input integer n); integer i; begin for (i = 0; i < n; i = i + 1) @(posedge frame_start); #1; end endtask
    task focus_to(input [1:0] f); integer k; begin for (k = 0; k < 4 && fi != f; k = k + 1) pulse_r; end endtask
    task enter; begin pulse_m; frames(3); if (vm_pix !== 1) fail("enter thumbnail"); end endtask

    integer g0, c0, r0, a0, t_sel, k5;
    initial begin
        fail_mask[2] = 1'b1;                 // 图 2 坏：预加载失败 → 缩略图格子 2 一直未加载
        #200; rst_n = 1;
        // 开机：首图 0 + 预加载 1、3（2 失败）
        wait (display_valid_sys); frames(40);
        if (active_image_idx !== 0) fail("boot image 0");
        if (page_cells !== 4'b1011) begin fail("preload bitmap"); $display("  cells=%b", page_cells); end

        // ===== S2 快速选择已加载格子 3 =====
        enter;
        focus_to(2'd3); frames(2);
        g0 = n_goto; c0 = n_commit;
        exp_target = 6'd3; exp_target_valid = 1;
        pulse_c;
        frames(8);
        if (n_goto != g0 + 1) fail("S2 exactly one goto");
        if (active_image_idx !== 6'd3 || current_image_idx !== 6'd3) fail("S2 target committed");
        if (n_commit != c0 + 1) fail("S2 exactly one commit");
        if (vm_pix !== 0) fail("S2 back to fullscreen");
        exp_target_valid = 0;

        // ===== S3 选择未加载格子 2 =====
        enter;
        focus_to(2'd2); frames(2);
        g0 = n_goto; r0 = n_rj; a0 = active_image_idx;
        pulse_c; frames(6);
        if (n_rj != r0 + 1 || n_goto != g0) fail("S3 unloaded cell rejected without goto");
        if (active_image_idx !== a0 || vm_pix !== 1 || fi !== 2'd2) fail("S3 state unchanged");
        pulse_m; frames(3);                   // 模式键返回全屏，不切图
        if (vm_pix !== 0 || active_image_idx !== a0) fail("S3 mode key returns without switching");

        // ===== S5 全屏下一张 =====
        g0 = n_goto;
        pulse_next; frames(8);
        if (active_image_idx !== 6'd4) fail("S5 next goes 3 -> 4 (load)");   // 4 不在 bank：加载
        frames(10);
        if (active_image_idx !== 6'd4) fail("S5 next committed");

        // ===== S4 第 2 页（4..7）：4 已加载，选 4 以外的已加载格子需要先让它们加载过 =====
        pulse_next; frames(10); pulse_next; frames(10);           // 5、6 加载过
        if (active_image_idx !== 6'd6) fail("S4 prep next to 6");
        // 把 bank 里的 5 挤掉：再切 7、8、9（每次写非显示 bank）
        pulse_next; frames(10); pulse_next; frames(10); pulse_next; frames(10);
        pulse_next; frames(10);                                    // → 10
        // 回到第 2 页的 4：选 4 走加载（不在 bank 里时）
        enter; frames(2);
        // 当前页 = 2（图 8..11），8、9、10 加载过，11 没有
        if (page_cells !== 4'b0111) begin fail("S4 page 2 bitmap"); $display("  cells=%b cur=%0d", page_cells, current_image_idx); end
        focus_to(2'd0);                                            // 图 8
        exp_target = 6'd8; exp_target_valid = 1; g0 = n_goto;
        pulse_c;
        frames(3);
        if (request_busy !== 1 && active_image_idx !== 6'd8) fail("S4 request_busy while loading");
        if (vm_pix !== 1 && active_image_idx !== 6'd8) fail("S4 must stay thumbnail while loading");
        frames(12);
        if (active_image_idx !== 6'd8 || vm_pix !== 0 || n_goto != g0 + 1) fail("S4 loaded target committed then fullscreen");
        exp_target_valid = 0;

        // ===== S5 自动轮播 =====
        pulse_auto; frames(2);
        if (!auto_enabled) fail("S5 auto enabled");
        c0 = n_commit; frames(90);
        if (n_commit < c0 + 2) fail("S5 auto play advances in fullscreen");
        enter;
        c0 = n_commit; a0 = active_image_idx;
        frames(100);
        if (n_commit != c0 || active_image_idx !== a0) fail("S5 auto play paused in thumbnail");
        // 缩略图里快速选择与轮播不冲突
        t_sel = 4;
        for (k5 = 3; k5 >= 0; k5 = k5 - 1) if (page_cells[k5] && k5 != a0[1:0]) t_sel = k5;
        if (t_sel < 4) focus_to(t_sel[1:0]);
        else fail("S5 no loaded cell other than current");
        if (t_sel < 4) begin
            exp_target = {a0[5:2], fi}; exp_target_valid = 1;
            pulse_c; frames(15);
            if (active_image_idx !== exp_target || vm_pix !== 0) fail("S5 select with auto on");
            exp_target_valid = 0;
        end else begin
            pulse_m; frames(3);
        end
        c0 = n_commit; frames(90);
        if (n_commit < c0 + 1) fail("S5 auto play resumes after fullscreen");

        if (errors == 0) $display("PASS tb_thumb_select_chain (goto=%0d commit=%0d select=%0d reject=%0d)", n_goto, n_commit, n_sv, n_rj);
        else $display("FAIL tb_thumb_select_chain errors=%0d", errors);
        $finish;
    end
    initial begin #200_000_000; $display("FAIL tb_thumb_select_chain timeout"); $finish; end
endmodule
