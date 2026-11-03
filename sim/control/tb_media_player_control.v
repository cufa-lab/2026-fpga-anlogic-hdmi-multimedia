`timescale 1ns/1ps
// media_player_control 场景测试（sys 100MHz / pix 25MHz，带行为级 loader 模型）—— 四bank 版
//
// 全程断言（任何时刻违反即 FAIL）：
//   A1 loader 正在写的 bank ≠ 读端正在显示的 bank（防撕裂 / 防覆盖）
//   A2 active_buffer_idx / active_image_idx / display_valid 只在 frame_boundary 后一拍变化
//   A3 load_request 只在 loader_ready 时发出
//   A4 每次帧边界提交时，显示的 bank 里确实整张装着 active_image_idx（不显示错图 / 半张图）
// 场景：
//   S1 开机：首图 img0 → bank0；随后后台预加载 img1/img2 → bank1/bank2（image_count=3）
//   S2 命中零延迟：goto 1/2/0 不发加载，≤2 帧内在帧边界切换到对应 bank
//   S3 未命中（image_count=8）：goto 5 → 写 home bank1；加载中 goto 6 挂起、goto 7 报 0x52；
//      挂起的 6 随后执行（写 bank2）
//   S3b 取代：未命中加载 img7 途中 goto 0（命中）→ 立即显示 0；img7 加载完只入表不显示；再 goto 7 命中
//   S4 周期越界 0x51；合法周期生效
//   S5 自动轮播：auto_tick 周期正确；后台预加载期间也照常计时（request_busy 不含预加载）
//   S6 loader 报错（0x56）：错误码粘滞、显示不变；下次成功提交后清零
//   S6b 预加载失败静默：不报错、不重试；之后真正切过去才报 0x56
//   S6c image_complete=0 的 bank 不算命中→ 重新加载
//   S7 image_count=1 时 goto / auto → 0x53
//   S8 复位后首图坏：自动换 img1 作为首图
//   S9 快速启动：唯一已知图坏，后台扫描再找到一张 → 重新启动首图
//   S10 64 项目录：最高索引可加载并提交，音频按低 2 位循环
//   音视频联动：active_audio_idx 与画面同一帧边界切换，值为 goto 携带的音轨（本 tb 用 img^3）
module tb_media_player_control;
    reg sys_clk = 0, pix_clk = 0;
    reg sys_rst_n = 0, pix_rst_n = 0;
    always #5  sys_clk = ~sys_clk;
    always #20 pix_clk = ~pix_clk;

    // ---------------- 激励 ----------------
    reg        goto_valid = 0;
    reg [5:0]  goto_idx = 0;
    reg [1:0]  goto_aud = 0;
    reg        toggle_auto = 0;
    reg        period_valid = 0;
    reg [31:0] period_cycles = 0;
    reg        scan_done = 0;
    reg [6:0]  image_count = 3;
    reg [63:0] fail_mask = 64'b0;     // 这些图片加载返回 0x56
    reg [3:0]  ic_force_clear = 4'b0; // S6c：强制 loader 的 image_complete 某位为 0

    // ---------------- loader 模型（含 4 个 bank 的内容记账 + image_complete）----------------
    localparam integer LOAD_LAT = 300;
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
    reg  [7:0]  load_error = 0;
    reg  [3:0]  image_complete_r = 0;
    reg  [5:0]  bank_content [0:3];
    wire        loader_ready = scan_done && !ld_busy;
    wire [3:0]  image_complete = image_complete_r & ~ic_force_clear;
    integer     n_loads = 0;

    always @(posedge sys_clk) begin
        load_done  <= 0;
        load_error <= 0;
        if (!sys_rst_n) begin
            image_complete_r <= 0; ld_busy <= 0;
        end else if (load_request) begin
            ld_busy <= 1; ld_cnt <= LOAD_LAT; ld_img <= load_image_idx; ld_buf <= load_buffer_idx;
            image_complete_r[load_buffer_idx] <= 1'b0;
            n_loads = n_loads + 1;
        end else if (ld_busy) begin
            if (ld_cnt == 0) begin
                ld_busy <= 0;
                if (fail_mask[ld_img]) load_error <= 8'h56;
                else begin
                    load_done <= 1; loaded_image_idx <= ld_img; loaded_buffer_idx <= ld_buf;
                    image_complete_r[ld_buf] <= 1'b1;
                    bank_content[ld_buf] = ld_img;
                end
            end else ld_cnt <= ld_cnt - 1;
        end
    end

    // ---------------- 帧边界（缩小的帧：每 200 个 pix 周期） ----------------
    reg frame_boundary = 0;
    integer pix_cnt = 0;
    always @(posedge pix_clk) begin
        pix_cnt <= (pix_cnt == 199) ? 0 : pix_cnt + 1;
        frame_boundary <= (pix_cnt == 199);
    end

    wire auto_tick, auto_enabled, display_valid_sys, error_event, request_busy;
    wire [31:0] active_period_cycles;
    wire [5:0]  current_image_idx, active_image_idx;
    wire [1:0]  active_buffer_idx;
    wire [1:0]  active_audio_idx;
    wire        committed_map;          // image_committed：至少一张已提交
    wire [7:0]  control_error;
    wire [3:0]  bank_resident;
    wire        display_valid, index_commit;

    media_player_control #(
        .DEFAULT_PERIOD_CYCLES(32'd20000),
        .MIN_PERIOD_CYCLES    (32'd5000),
        .MAX_PERIOD_CYCLES    (32'd100000)
    ) dut (
        .sys_clk(sys_clk), .sys_rst_n(sys_rst_n), .tick(1'b1),
        .goto_valid(goto_valid), .goto_image_idx(goto_idx), .goto_audio_idx(goto_aud),
        .toggle_auto_event(toggle_auto),
        .period_cfg_valid(period_valid), .period_cfg_cycles(period_cycles), .auto_tick(auto_tick),
        .scan_done(scan_done), .image_count(image_count), .loader_ready(loader_ready),
        .load_done(load_done), .loaded_image_idx(loaded_image_idx), .loaded_buffer_idx(loaded_buffer_idx),
        .load_error(load_error), .image_complete(image_complete),
        .load_request(load_request), .load_image_idx(load_image_idx),
        .load_buffer_idx(load_buffer_idx), .auto_enabled(auto_enabled),
        .active_period_cycles(active_period_cycles), .current_image_idx(current_image_idx), .image_committed(committed_map),
        .display_valid_sys(display_valid_sys), .control_error(control_error),
        .error_event(error_event), .request_busy(request_busy), .bank_resident(bank_resident),
        .pix_clk(pix_clk), .pix_rst_n(pix_rst_n), .frame_boundary(frame_boundary),
        .active_image_idx(active_image_idx), .active_buffer_idx(active_buffer_idx), .active_audio_idx(active_audio_idx),
        .display_valid(display_valid), .index_commit(index_commit)
    );

    integer errors = 0;
    integer n_err_events = 0;
    always @(posedge sys_clk) if (sys_rst_n && error_event) n_err_events = n_err_events + 1;

    // ---------------- 全程断言 ----------------
    always @(posedge sys_clk) if (sys_rst_n) begin
        if (ld_busy && display_valid && (ld_buf == active_buffer_idx)) begin
            $display("FAIL A1 @%0t: writing displayed bank %0d", $time, ld_buf); errors = errors + 1;
        end
        if (load_request && !loader_ready) begin
            $display("FAIL A3 @%0t: load_request while loader busy", $time); errors = errors + 1;
        end
    end

    reg [1:0] ab_q = 0, aa_q = 0;
    reg [5:0] ai_q = 0;
    reg       dv_q = 0, fb_q = 0;
    integer   n_commits = 0;
    always @(posedge pix_clk) begin
        if (pix_rst_n && !fb_q &&
            (ab_q !== active_buffer_idx || ai_q !== active_image_idx || aa_q !== active_audio_idx || dv_q !== display_valid)) begin
            $display("FAIL A2 @%0t: display changed outside frame boundary", $time); errors = errors + 1;
        end
        ab_q <= active_buffer_idx; ai_q <= active_image_idx; aa_q <= active_audio_idx; dv_q <= display_valid; fb_q <= frame_boundary;
        if (pix_rst_n && index_commit) begin
            n_commits = n_commits + 1;
            if (!image_complete_r[active_buffer_idx] || bank_content[active_buffer_idx] !== active_image_idx ||
                (ld_busy && ld_buf == active_buffer_idx)) begin
                $display("FAIL A4 @%0t: committed img %0d on bank %0d holding %0d (complete=%b)", $time,
                         active_image_idx, active_buffer_idx, bank_content[active_buffer_idx], image_complete_r);
                errors = errors + 1;
            end
        end
    end

    // ---------------- 工具任务 ----------------
    task sys_wait(input integer c); repeat (c) @(posedge sys_clk); endtask
    task do_goto(input [5:0] idx);
        begin @(posedge sys_clk) begin goto_valid <= 1; goto_idx <= idx; goto_aud <= idx[1:0] ^ 2'b11; end
              @(posedge sys_clk) goto_valid <= 0; end
    endtask
    task do_period(input [31:0] c);
        begin @(posedge sys_clk) begin period_valid <= 1; period_cycles <= c; end
              @(posedge sys_clk) period_valid <= 0; end
    endtask
    task do_auto;
        begin @(posedge sys_clk) toggle_auto <= 1; @(posedge sys_clk) toggle_auto <= 0; end
    endtask
    task wait_idle;   // 等控制层完全空闲（加载含后台预加载 + 提交 + ack），连续 50 拍都空闲才算
        integer quiet;
        begin
            quiet = 0;
            sys_wait(4);
            while (quiet < 50) begin
                @(posedge sys_clk);
                if (request_busy || dut.req_pending || ld_busy || dut.transfer_busy || load_request) quiet = 0;
                else quiet = quiet + 1;
            end
        end
    endtask
    task expect_show(input [5:0] img, input [127:0] tag);
        begin
            if (current_image_idx !== img || active_image_idx !== img || !display_valid || !display_valid_sys) begin
                $display("FAIL %0s: cur=%0d active=%0d dv=%b/%b (exp %0d)", tag,
                         current_image_idx, active_image_idx, display_valid, display_valid_sys, img);
                errors = errors + 1;
            end
        end
    endtask
    task expect_bank(input [1:0] b, input [127:0] tag);
        begin
            if (active_buffer_idx !== b) begin
                $display("FAIL %0s: bank %0d (exp %0d)", tag, active_buffer_idx, b); errors = errors + 1;
            end
        end
    endtask
    task expect_av(input [1:0] aud, input [127:0] tag);
        begin
            if (active_audio_idx !== aud || committed_map !== 1'b1) begin
                $display("FAIL %0s: audio=%0d map=%b (exp %0d)", tag, active_audio_idx, committed_map, aud);
                errors = errors + 1;
            end
        end
    endtask
    task expect_err(input [7:0] e, input [127:0] tag);
        begin
            if (control_error !== e) begin
                $display("FAIL %0s: control_error=%h exp %h", tag, control_error, e); errors = errors + 1;
            end
        end
    endtask
    task expect_loads(input integer n, input [127:0] tag);
        begin
            if (n_loads !== n) begin $display("FAIL %0s: loads=%0d exp %0d", tag, n_loads, n); errors = errors + 1; end
        end
    endtask
    // 命中切换：不发加载，≤ 2 帧（2 x 200 pix 周期 = 16us）+ 同步余量内完成
    task hit_goto(input [5:0] idx, input [1:0] b, input [127:0] tag);
        integer nl; time ts;
        begin
            nl = n_loads; ts = $time;
            do_goto(idx);
            while (!(current_image_idx == idx && display_valid_sys && !request_busy) && ($time - ts) < 100_000) @(posedge sys_clk);
            if (($time - ts) > 18_000) begin $display("FAIL %0s: hit switch took %0d ns", tag, $time - ts); errors = errors + 1; end
            sys_wait(4);
            expect_show(idx, tag);
            expect_bank(b, tag);
            expect_av(idx[1:0] ^ 2'b11, tag);
            if (n_loads !== nl) begin $display("FAIL %0s: hit issued a load", tag); errors = errors + 1; end
        end
    endtask
    task do_reset;
        begin
            scan_done = 0;
            sys_rst_n = 0; pix_rst_n = 0;
            sys_wait(5);
            sys_rst_n = 1; pix_rst_n = 1;
            sys_wait(20);
        end
    endtask

    integer t0, t1, nl0, ne0;
    reg [1:0] buf_before;
    reg [1:0] aa_before;

    initial begin
        #100 sys_rst_n = 1;
        #80  pix_rst_n = 1;
        sys_wait(50);
        if (display_valid || load_request) begin $display("FAIL: activity before scan_done"); errors = errors + 1; end

        // ---------- S1 开机 + 后台预加载 ----------
        scan_done = 1;
        wait_idle;
        expect_show(0, "S1 boot");
        expect_bank(0, "S1 boot");
        expect_av(2'd0 ^ 2'd0, "S1 boot audio");      // 开机首图音轨 = 首图序号 0
        expect_err(0, "S1");
        expect_loads(3, "S1 boot + preload 1,2");
        if (bank_resident !== 4'b0111) begin $display("FAIL S1 bank_resident %b", bank_resident); errors = errors + 1; end

        // ---------- S2 命中零延迟 ----------
        hit_goto(1, 1, "S2 hit1");
        hit_goto(2, 2, "S2 hit2");
        hit_goto(0, 0, "S2 hit0");
        hit_goto(2, 2, "S2 hit2 again");
        expect_loads(3, "S2 no loads");

        // ---------- S3 未命中 + 忙时合并 ----------
        image_count = 8;
        wait_idle;                                  // 新图出现：预加载 img3 → bank3
        expect_loads(4, "S3 preload img3");
        if (bank_resident !== 4'b1111) begin $display("FAIL S3 bank_resident %b", bank_resident); errors = errors + 1; end
        // 当前显示 img2(bank2)；goto 5 → home bank1
        do_goto(5);
        sys_wait(20);
        if (!ld_busy || ld_buf !== 2'd1) begin $display("FAIL S3: miss load not started on bank1"); errors = errors + 1; end
        do_goto(6);                 // 挂起
        do_goto(7);                 // 已有挂起 → 0x52
        sys_wait(2);
        expect_err(8'h52, "S3 busy");
        wait_idle;
        // 挂起的 6：home bank2，此时显示 bank1 → 写 bank2
        expect_show(6, "S3 pending executed");
        expect_bank(2, "S3 pending bank");
        expect_av(2'd6 ^ 2'd3, "S3 pending audio");
        expect_err(0, "S3 cleared by commit");
        expect_loads(6, "S3 loads");

        // ---------- S3b 取代：未命中加载途中命中切换 ----------
        do_goto(7);                 // home bank3（显示 bank2）
        sys_wait(20);
        if (!ld_busy || ld_buf !== 2'd3) begin $display("FAIL S3b: miss load not started on bank3"); errors = errors + 1; end
        hit_goto(0, 0, "S3b hit0 during load");
        wait_idle;
        expect_show(0, "S3b stays on 0 after superseded load");
        if (bank_content[3] !== 6'd7 || !bank_resident[3]) begin $display("FAIL S3b img7 not entered in bank3"); errors = errors + 1; end
        hit_goto(7, 3, "S3b goto7 now hits");

        // ---------- S4 周期 ----------
        do_period(32'd100);    sys_wait(2); expect_err(8'h51, "S4 too short");
        do_period(32'd200000); sys_wait(2); expect_err(8'h51, "S4 too long");
        if (active_period_cycles !== 32'd20000) begin $display("FAIL S4 period changed"); errors = errors + 1; end
        do_period(32'd8000);   sys_wait(2);
        if (active_period_cycles !== 32'd8000) begin $display("FAIL S4 period not applied"); errors = errors + 1; end

        // ---------- S5 自动轮播 ----------
        image_count = 3;
        do_auto; sys_wait(2);
        if (!auto_enabled) begin $display("FAIL S5 auto not enabled"); errors = errors + 1; end
        @(posedge auto_tick); t0 = $time;
        // 模拟策略层：tick → goto 下一张
        @(posedge sys_clk) begin goto_valid <= 1; goto_idx <= (current_image_idx >= 2) ? 0 : current_image_idx + 1;
            goto_aud <= ((current_image_idx >= 2) ? 2'd0 : current_image_idx + 2'd1) ^ 2'b11; end
        @(posedge sys_clk) goto_valid <= 0;
        wait_idle;
        @(posedge auto_tick); t1 = $time;
        // 空闲后才开始计时，间隔 ≥ 周期
        if ((t1 - t0) < 8000 * 10) begin $display("FAIL S5 tick interval %0d", t1 - t0); errors = errors + 1; end
        do_auto; sys_wait(2);
        if (auto_enabled) begin $display("FAIL S5 auto not disabled"); errors = errors + 1; end
        wait_idle;

        // ---------- S6 loader 报错 ----------
        image_count = 8;
        do_goto(1); wait_idle;               // bank1 此时装 img5 → 未命中，img1 重新写进 home bank1
        expect_show(1, "S6 setup show 1"); expect_bank(1, "S6 setup bank");
        buf_before = active_buffer_idx;
        aa_before  = active_audio_idx;
        fail_mask = 64'h20;                  // img5 坏
        do_goto(5); wait_idle;               // home bank1 正在显示 → 写 bank0
        expect_err(8'h56, "S6 load error");
        if (active_buffer_idx !== buf_before) begin $display("FAIL S6 display changed"); errors = errors + 1; end
        if (active_audio_idx !== aa_before) begin $display("FAIL S6 audio changed"); errors = errors + 1; end
        // 失败的 bank 不能被当成装着 img5（之后后台预加载可能把 img0 补回 home bank0，这是允许的）
        if (bank_resident[0] && dut.bank_img0 == 6'd5) begin $display("FAIL S6 failed bank marked as holding img5"); errors = errors + 1; end
        fail_mask = 64'b0;
        do_goto(5); wait_idle;
        expect_show(5, "S6 recovered");
        expect_bank(0, "S6 recovered bank");
        expect_av(2'd5 ^ 2'd3, "S6 recovered audio");
        expect_err(0, "S6 cleared");

        // ---------- S6c image_complete=0 → 不算命中 ----------
        // bank3 有 img7；强制 loader 说 bank3 不完整 → goto 7 必须重新加载
        nl0 = n_loads;
        ic_force_clear = 4'b1000;
        do_goto(7); wait_idle;
        if (n_loads != nl0 + 1) begin $display("FAIL S6c incomplete bank was treated as hit (loads %0d)", n_loads - nl0); errors = errors + 1; end
        ic_force_clear = 4'b0000;
        expect_show(7, "S6c reload");

        // ---------- S7 资源不足 ----------
        image_count = 1;
        do_goto(0); sys_wait(2); expect_err(8'h53, "S7 goto");
        do_auto;    sys_wait(2); expect_err(8'h53, "S7 auto");
        if (auto_enabled) begin $display("FAIL S7 auto enabled"); errors = errors + 1; end
        image_count = 3;

        // ---------- S6b 预加载失败静默 ----------
        n_loads = 0;
        fail_mask = 64'h4;                    // img2 坏
        do_reset;
        ne0 = n_err_events;
        scan_done = 1;
        wait_idle;
        sys_wait(2000);
        wait_idle;
        expect_show(0, "S6b boot");
        expect_err(0, "S6b preload failure silent");
        if (n_err_events != ne0) begin $display("FAIL S6b preload failure raised error_event"); errors = errors + 1; end
        expect_loads(3, "S6b boot + preload1 + preload2(fail) once");
        if (bank_resident !== 4'b0011) begin $display("FAIL S6b bank_resident %b", bank_resident); errors = errors + 1; end
        do_goto(2); wait_idle;                // 真正切过去才报
        expect_err(8'h56, "S6b goto bad image reports");
        expect_show(0, "S6b display kept");
        fail_mask = 64'b0;

        // ---------- S8 首图坏 ----------
        n_loads = 0;
        fail_mask = 64'h1;
        do_reset;
        if (committed_map !== 1'b0 || display_valid) begin $display("FAIL S8 map not cleared %b", committed_map); errors = errors + 1; end
        scan_done = 1;
        wait_idle;
        sys_wait(2000);
        wait_idle;
        expect_show(1, "S8 boot skip bad");
        expect_bank(1, "S8 boot bank");
        expect_av(2'd1, "S8 boot audio");
        expect_err(0, "S8 error cleared by commit");
        // 首图 0 坏 + 1 成功；预加载：bank0(img0 再试一次失败后不再试) + bank2(img2)
        expect_loads(4, "S8 loads");

        // ---------- S9 快速启动：唯一已知图坏，后台扫描再找到一张 → 重新启动首图 ----------
        n_loads = 0;
        fail_mask = 64'h1;
        image_count = 1;
        do_reset;
        scan_done = 1;
        wait_idle;
        sys_wait(20);
        wait_idle;
        if (display_valid) begin $display("FAIL S9 displayed with only a bad image"); errors = errors + 1; end
        image_count = 2;                                  // 后台扫描确认第 2 张
        sys_wait(5);
        wait_idle;
        sys_wait(2000);
        wait_idle;
        expect_show(1, "S9 boot re-armed");
        expect_loads(3, "S9 loads (bad0, good1, preload0 once)");
        fail_mask = 64'b0; image_count = 64;

        // ---------- S10 64 项目录：最高索引可加载并提交，音频按低 2 位循环 ----------
        do_goto(63); wait_idle;
        expect_show(63, "S10 goto63");
        expect_av(2'd0, "S10 audio");

        sys_wait(100);
        $display("INFO commits checked by A4: %0d", n_commits);
        if (errors == 0) $display("PASS tb_media_player_control");
        else             $display("FAIL tb_media_player_control errors=%0d", errors);
        $finish;
    end

    initial begin
        #40_000_000;
        $display("FAIL tb_media_player_control timeout");
        $finish;
    end
endmodule
