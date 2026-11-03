`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// tb_media_player_lookahead —— media_player_control LOOKAHEAD=1 前瞻缓存 + 缩略图记账
//   sys 100MHz / pix 25MHz，帧缩小为 200 个 pix 周期；loader 行为模型记 bank 内容、可按图片注入失败
// 全程断言：
//   G1 后台加载（load_is_request=0）不写当前显示的 bank，也不写正驻留着 A（顺方向下一张）/ B（反方向）的 bank
//   G2 后台加载期间 request_busy 不因它变 1（没有用户切换在途时 request_busy=0）
//   G3 每次帧边界提交时，显示的 bank 里整张装着 active_image_idx（同 tb_media_player_control A4）
//   G4 写缩略图的加载（gen=1）不会盖掉当前页已完好的格子（条目 {img[2], img[1:0]} 的旧页号 = 当前页且新图不在当前页）
// 场景（10 张图 = 3 页，第 2 页只有 8、9）：
//   L1 开机首图 0：随后后台把 A=1、B=9 驻留；当前页 1~3、下一页 4~7 生成缩略图（gen=1）；稳定后不再加载
//   L2 goto 1（命中）：goto 到提交之间没有加载；方向 = 下一张，A=2 随后驻留
//   L3 走到 3 停留 → goto 4（跨页）命中零延迟
//   L4 goto 3（上一张）：方向翻转，A=2（上一张方向）驻留，B=4 保留
//   L5 已有缩略图的图被真切换加载时 gen=0（goto 6 未驻留 → 用户加载 gen=0），格子 valid 保持
//   L6 后台候选加载失败：不报错、本轮不重试；下一次提交后才再试
//   L7 末页（第 2 页）停留：回绕邻页第 0 页同一块区，不预生成；稳定后不再加载，8/9 缩略图保持
//   L9 全屏切到未驻留的图、排在后台加载后面时 request_busy 一直为 1，不报 0x52
//   L8 总览快速提交（零等待、补整图后补提交、快速连按不报错、坏图报错并退回）
//   L10 全屏匀速连按（节奏略慢于一次读卡）组内 / 跨页 / 回绕都命中滑动窗口 {B, 当前, A, A2}；停下后缩略图补齐、窗口不缺
//   G1b 全屏后台加载不挤掉 A2
//   G5 load_cancel 只打后台加载（不打用户 / 开机加载，不打未过期的补当前图）；0x59 不报错、不记 cand_fail
//   L11 全屏切到未驻留的远处图片、后台大图加载刚开始 → 立即取消，时延 ≈ 一次读卡（不再先等后台那张读完）
//   L12 总览跨到远处页：在途的旧页缩略图加载被取消，新页立即开始补
//   L13 全屏连按快于读卡（0.6 倍）：每次时延 ≤ 一次读卡 + 一帧（后台加载不挡路），全程不报错
//   LOOKAHEAD=0 默认实例行为由 tb_media_player_control 覆盖（本 tb 不重复）
// -----------------------------------------------------------------------------
module tb_media_player_lookahead;
    reg sys_clk = 0, pix_clk = 0;
    reg rst_n = 0;
    always #5  sys_clk = ~sys_clk;
    always #20 pix_clk = ~pix_clk;

    integer errors = 0;
    task fail(input [8*80-1:0] msg); begin errors = errors + 1; if (errors < 40) $display("FAIL %0s @%0t", msg, $time); end endtask

    reg        goto_valid = 0;
    reg [5:0]  goto_idx = 0;
    reg [6:0]  image_count = 7'd10;
    reg [63:0] fail_mask = 64'b0;
    reg        fast_view = 0;            // L8：总览已生效
    wire       view_hold;
    integer    n_err = 0;

    // ---------------- loader 模型 ----------------
    integer     ld_lat = 300;            // L10 改大：读卡比帧长（板上约 50 帧）
    wire        load_request, load_thumb_gen, load_cancel;
    integer     n_cancel = 0;
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
    reg  [3:0]  image_complete = 0;
    reg  [5:0]  bank_content [0:3];
    integer     n_loads = 0, n_gen = 0;
    integer     load_cnt_img [0:63];
    integer     gen_cnt_img  [0:63];
    integer     ii;
    initial for (ii = 0; ii < 64; ii = ii + 1) begin load_cnt_img[ii] = 0; gen_cnt_img[ii] = 0; end

    always @(posedge sys_clk) begin
        load_done  <= 0;
        load_error <= 0;
        if (!rst_n) begin
            image_complete <= 0; ld_busy <= 0;
        end else if (load_request) begin
            ld_busy <= 1; ld_cnt <= ld_lat; ld_img <= load_image_idx; ld_buf <= load_buffer_idx;
            image_complete[load_buffer_idx] <= 1'b0;
            n_loads = n_loads + 1;
            load_cnt_img[load_image_idx] = load_cnt_img[load_image_idx] + 1;
            if (load_thumb_gen) begin n_gen = n_gen + 1; gen_cnt_img[load_image_idx] = gen_cnt_img[load_image_idx] + 1; end
        end else if (load_cancel && ld_busy) begin       // CANCEL_EN：中止在途加载，回 0x59（目标 bank 保持不完整）
            ld_busy <= 0; load_error <= 8'h59; n_cancel = n_cancel + 1;
        end else if (ld_busy) begin
            if (ld_cnt == 0) begin
                ld_busy <= 0;
                if (fail_mask[ld_img]) load_error <= 8'h56;
                else begin
                    load_done <= 1; loaded_image_idx <= ld_img; loaded_buffer_idx <= ld_buf;
                    image_complete[ld_buf] <= 1'b1;
                    bank_content[ld_buf] = ld_img;
                end
            end else ld_cnt <= ld_cnt - 1;
        end
    end

    reg frame_boundary = 0;
    integer pix_cnt = 0;
    always @(posedge pix_clk) begin
        pix_cnt <= (pix_cnt == 199) ? 0 : pix_cnt + 1;
        frame_boundary <= (pix_cnt == 199);
    end

    wire auto_tick, auto_enabled, display_valid_sys, error_event, request_busy, display_valid, index_commit;
    wire [5:0] current_image_idx, active_image_idx;
    wire [1:0] active_buffer_idx, active_audio_idx;
    wire [7:0] control_error;
    wire [3:0] bank_resident;

    media_player_control #(
        .DEFAULT_PERIOD_CYCLES(32'd20000),
        .MIN_PERIOD_CYCLES    (32'd5000),
        .MAX_PERIOD_CYCLES    (32'd100000),
        .LOOKAHEAD            (1)
    ) dut (
        .sys_clk(sys_clk), .sys_rst_n(rst_n), .tick(1'b1),
        .goto_valid(goto_valid), .goto_image_idx(goto_idx), .goto_audio_idx(goto_idx[1:0]),
        .toggle_auto_event(1'b0), .period_cfg_valid(1'b0), .period_cfg_cycles(32'd0), .auto_tick(auto_tick),
        .scan_done(1'b1), .image_count(image_count), .loader_ready(!ld_busy),
        .load_done(load_done), .loaded_image_idx(loaded_image_idx), .loaded_buffer_idx(loaded_buffer_idx),
        .load_error(load_error), .image_complete(image_complete), .fast_view(fast_view), .view_hold(view_hold),
        .load_request(load_request), .load_image_idx(load_image_idx), .load_buffer_idx(load_buffer_idx),
        .load_thumb_gen(load_thumb_gen), .load_cancel(load_cancel),
        .auto_enabled(auto_enabled), .active_period_cycles(), .current_image_idx(current_image_idx),
        .image_committed(), .display_valid_sys(display_valid_sys), .control_error(control_error),
        .error_event(error_event), .request_busy(request_busy), .bank_resident(bank_resident),
        .pix_clk(pix_clk), .pix_rst_n(rst_n), .frame_boundary(frame_boundary),
        .active_image_idx(active_image_idx), .active_buffer_idx(active_buffer_idx), .active_audio_idx(active_audio_idx),
        .display_valid(display_valid), .index_commit(index_commit));

    // ---------------- 全程断言 ----------------
    function [5:0] bimg(input [1:0] b);
        case (b) 2'd0: bimg = dut.bank_img0; 2'd1: bimg = dut.bank_img1; 2'd2: bimg = dut.bank_img2; default: bimg = dut.bank_img3; endcase
    endfunction
    function [2:0] ttag(input [2:0] e);
        case (e) 3'd0: ttag = dut.th_t0; 3'd1: ttag = dut.th_t1; 3'd2: ttag = dut.th_t2; 3'd3: ttag = dut.th_t3;
                 3'd4: ttag = dut.th_t4; 3'd5: ttag = dut.th_t5; 3'd6: ttag = dut.th_t6; default: ttag = dut.th_t7; endcase
    endfunction
    // 某图在 loader 模型里完整驻留的份数（同一张图可能在两个 bank：挤掉其中一份不算挤掉）
    function integer copies(input [5:0] img); integer b; begin copies = 0; for (b = 0; b < 4; b = b + 1) if (image_complete[b] && bank_content[b] == img) copies = copies + 1; end endfunction
    reg user_pending = 0;     // tb 发了 goto 还没提交
    always @(posedge sys_clk) if (rst_n && load_request && !dut.load_is_request) begin
        // G1（load_request 这拍 DUT 的 bank_valid[victim] 已清，用 loader 模型自己的 bank 内容 / 完整位判断）
        if (dut.display_valid_sys && load_buffer_idx == dut.current_buffer_idx) fail("G1 background load into displayed bank");
        // B 只许全屏的缩略图加载借用（bg_slot 2~9）；总览 / 驻留型加载都不挤 B
        if (image_complete[load_buffer_idx] &&
            (bank_content[load_buffer_idx] == dut.la_a ||
             (bank_content[load_buffer_idx] == dut.la_b && copies(dut.la_b) == 1 && (fast_view || dut.bg_slot <= 4'd1 || dut.bg_slot == 4'd10))))
            begin fail("G1 background load evicts A/B"); $display("  buf=%0d holds %0d A=%0d B=%0d slot=%0d A2=%0d cur=%0d curbuf=%0d ok=%b imgs=%0d %0d %0d %0d unres=%b fv=%b", load_buffer_idx, bank_content[load_buffer_idx], dut.la_a, dut.la_b, dut.bg_slot, dut.la_a2, current_image_idx, dut.current_buffer_idx, dut.bank_ok, dut.bank_img0, dut.bank_img1, dut.bank_img2, dut.bank_img3, dut.cur_unres, fast_view); end
        // G1b 全屏滑动窗口：后台加载不挤掉 A2（缩略图加载没空闲 bank 时只能借 B 的）
        if (!fast_view && image_complete[load_buffer_idx] && bank_content[load_buffer_idx] == dut.la_a2 && dut.la_a2 != load_image_idx)
            begin fail("G1b background load evicts A2"); $display("  buf=%0d holds %0d", load_buffer_idx, bank_content[load_buffer_idx]); end
        // G4
        if (load_thumb_gen && dut.th_v[load_image_idx[2:0]] && ttag(load_image_idx[2:0]) == current_image_idx[5:3] &&
            current_image_idx[2] == load_image_idx[2] && load_image_idx[5:2] != current_image_idx[5:2])
            begin fail("G4 background thumbnail overwrites current page"); $display("  img=%0d cur=%0d", load_image_idx, current_image_idx); end
    end
    // G5
    always @(posedge sys_clk) if (rst_n && load_cancel) begin
        if (dut.load_is_request) fail("G5 cancel hit a user / boot load");
        if (dut.bg_cur && !dut.bg_stale) fail("G5 cancel hit the current-image fill");
    end
    always @(posedge sys_clk) if (rst_n && error_event && control_error == 8'h59) fail("G5 cancel reported as error");
    always @(posedge sys_clk) if (rst_n && dut.display_valid_sys && !user_pending && request_busy && !goto_valid && !dut.commit_busy)   // 总览快速提交后的补提交 / 退回提交本身占一次握手
        begin fail("G2 request_busy from background load"); end
    always @(posedge pix_clk) if (index_commit) begin
        // 总览快速提交（cur_unres=1）时 bank 里故意还是上一张驻留图（屏上读缩略图区）
        if (!dut.cur_unres && bank_content[active_buffer_idx] !== active_image_idx) begin fail("G3 committed bank content mismatch"); $display("  bank%0d=%0d active=%0d", active_buffer_idx, bank_content[active_buffer_idx], active_image_idx); end
    end

    // ---------------- 工具 ----------------
    task frames(input integer n); integer i; begin for (i = 0; i < n; i = i + 1) @(posedge frame_boundary); #1; end endtask
    function resident(input [5:0] img);
        resident = (dut.bank_ok[0] && dut.bank_img0 == img) || (dut.bank_ok[1] && dut.bank_img1 == img) ||
                   (dut.bank_ok[2] && dut.bank_img2 == img) || (dut.bank_ok[3] && dut.bank_img3 == img);
    endfunction
    function thumb_ok(input [5:0] img);
        thumb_ok = dut.th_v[img[2:0]] && ttag(img[2:0]) == img[5:3];
    endfunction
    // 发 goto，等提交；返回 goto 到提交之间的加载数
    integer gl, glat;                      // goto 到提交之间的加载数 / 时延（ns）
    task go(input [5:0] img);
        integer l0, k;
        begin
            @(posedge sys_clk); #1 user_pending = 1; goto_idx = img; goto_valid = 1; l0 = n_loads; glat = $time;
            @(posedge sys_clk); #1 goto_valid = 0;
            for (k = 0; k < 8000 && !(current_image_idx == img && !request_busy); k = k + 1) @(posedge sys_clk);
            gl = n_loads - l0;
            glat = $time - glat;
            if (current_image_idx !== img) begin fail("go: not committed"); $display("  img=%0d cur=%0d", img, current_image_idx); end
            #1 user_pending = 0;
        end
    endtask
    // 停留到不再有加载（最多 n 帧）
    task settle(input integer n); integer k, l0; begin
        l0 = -1;
        for (k = 0; k < n && l0 != n_loads; k = k + 1) begin l0 = n_loads; frames(16); end
        if (l0 != n_loads) fail("settle: lookahead keeps loading");
    end endtask

    integer l0, e0, i, tgt, a0, k;
    reg     busy_gap;
    always @(posedge sys_clk) if (error_event) n_err = n_err + 1;
    function [5:0] la_next_tb(input [5:0] c); la_next_tb = ({1'b0, c} + 7'd1 >= image_count) ? 6'd0 : c + 6'd1; endfunction
    initial begin
        #200; rst_n = 1; user_pending = 1;
        // ===== L1 =====
        wait (display_valid_sys); #1 user_pending = 0;
        if (current_image_idx !== 0) fail("L1 boot image 0");
        settle(40);
        if (!resident(0) || !resident(1) || !resident(9)) begin fail("L1 A=1 / B=9 resident"); $display("  banks %0d %0d %0d %0d ok=%b", dut.bank_img0, dut.bank_img1, dut.bank_img2, dut.bank_img3, dut.bank_ok); end
        for (i = 0; i < 8; i = i + 1)
            if (!thumb_ok(i)) begin fail("L1 thumbnails 0..7"); $display("  img %0d", i); end
        if (gen_cnt_img[0] != 1 || gen_cnt_img[1] != 1 || gen_cnt_img[5] != 1) fail("L1 each thumbnail generated once");
        if (gen_cnt_img[9] != 0) fail("L1 B (image 9, page 2) is bank-only: page 2 shares area with page 0");

        // ===== L2 =====
        go(1);
        if (gl != 0) fail("L2 goto 1 hit, no load");
        if (dut.la_dir !== 1'b1) fail("L2 direction next");
        settle(40);
        if (!resident(2)) fail("L2 A=2 resident");

        // ===== L3 =====
        go(2); settle(40); go(3); settle(40);
        if (!resident(4)) fail("L3 image 4 resident before crossing");
        go(4);
        if (gl != 0) begin fail("L3 3->4 cross page zero latency"); $display("  loads %0d", gl); end
        settle(40);
        for (i = 4; i < 10; i = i + 1)
            if (!thumb_ok(i)) begin fail("L3 thumbnails page 1 / 2"); $display("  img %0d", i); end

        // ===== L4 =====
        go(3);
        if (gl != 0) fail("L4 goto 3 (B) hit");
        if (dut.la_dir !== 1'b0) fail("L4 direction prev");
        settle(40);
        if (!resident(2) || !resident(4)) begin fail("L4 A=2 / B=4 resident"); $display("  banks %0d %0d %0d %0d", dut.bank_img0, dut.bank_img1, dut.bank_img2, dut.bank_img3); end

        // ===== L5 =====
        e0 = gen_cnt_img[6];
        if (resident(6)) fail("L5 precondition: image 6 not resident");
        go(6);
        if (gl == 0) fail("L5 image 6 needed a load");
        if (gen_cnt_img[6] != e0) fail("L5 thumbnail intact -> gen=0");
        if (!thumb_ok(6)) fail("L5 thumbnail 6 still valid");
        settle(40);

        // ===== L6 =====
        fail_mask[1] = 1'b1;                    // 第 0 页的 1 会在回到第 0 页时作为候选
        go(5); settle(40);
        go(4); settle(40);
        e0 = load_cnt_img[1];
        go(0);                                  // 当前页 0：1 既是 A 候选，也可能是当前页缺缩略图的候选（每个候选各试一次）
        settle(40);
        l0 = load_cnt_img[1] - e0;
        if (l0 < 1 || l0 > 2) begin fail("L6 failed candidate tried once per candidate slot"); $display("  tries %0d", l0); end
        frames(200);
        if (load_cnt_img[1] - e0 != l0) fail("L6 no retry before next commit");
        if (control_error != 8'h00 || error_event) fail("L6 background failure must be silent");
        go(2);                                  // 下一次提交（1 = B 候选，也是当前页缺缩略图的格子）
        settle(40);
        if (load_cnt_img[1] - e0 <= l0) begin fail("L6 retried after next commit"); $display("  tries %0d", load_cnt_img[1] - e0); end
        fail_mask[1] = 1'b0;

        // ===== L9 上板 0x52：全屏切到未驻留的图、排在后台加载后面时 request_busy 一直为 1 =====
        go(0); settle(40);
        tgt = 64;
        for (i = 4; i < 8; i = i + 1) if (tgt == 64 && !resident(i)) tgt = i;
        if (tgt == 64) fail("L9 precondition: a non-resident image");
        else begin
            // 等一个后台加载开始（停留后新提交会触发前瞻）
            go(1);
            wait (ld_busy && !dut.load_is_request);
            @(posedge sys_clk); #1 user_pending = 1; goto_idx = tgt; goto_valid = 1;
            @(posedge sys_clk); #1 goto_valid = 0;
            busy_gap = 0;
            while (current_image_idx != tgt) begin
                @(posedge sys_clk); #1;
                if (!request_busy && current_image_idx != tgt) busy_gap = 1;
            end
            if (busy_gap) fail("L9 request_busy dropped while switch was pending behind background load");
            wait (!request_busy); #1 user_pending = 0;
            if (control_error != 8'h00) fail("L9 no error");
        end
        settle(40);

        // ===== L8 总览快速提交 =====
        fast_view = 1;
        tgt = 64;
        for (i = 4; i < 10; i = i + 1) if (tgt == 64 && !resident(i)) tgt = i;
        if (tgt == 64) fail("L8 precondition: a non-resident image");
        else begin
            l0 = n_loads;
            go(tgt);
            if (gl != 0) begin fail("L8 overview miss commits without waiting for a load"); $display("  loads %0d", gl); end
            if (!view_hold) fail("L8 view_hold while full image missing");
            // 第一个后台加载 = 当前图
            wait (n_loads != l0); #1;
            if (ld_img !== tgt) begin fail("L8 first background load = current image"); $display("  loaded %0d", ld_img); end
            settle(40);
            if (view_hold || !resident(tgt)) fail("L8 full image resolved, view_hold cleared");
            if (bank_content[active_buffer_idx] !== tgt) fail("L8 re-commit to the real bank");
        end
        // 快速连按：每 3 帧一次，页内 + 跨页，全程不报错，每次都立即提交
        e0 = n_err;
        for (i = 0; i < 8; i = i + 1) begin
            go(la_next_tb(current_image_idx));
            if (gl != 0) fail("L8 rapid move without load wait");
            frames(3);
        end
        if (n_err != e0) begin fail("L8 rapid overview moves raised an error"); $display("  errors %0d code %h", n_err - e0, control_error); end
        settle(60);
        if (view_hold) fail("L8 settles");
        // 坏图：总览快速提交到坏图 → 后台补图失败 → 报错、退回上一张
        a0 = current_image_idx;
        tgt = 64;
        for (i = 0; i < 10; i = i + 1) if (tgt == 64 && !resident(i) && i != a0) tgt = i;
        if (tgt != 64) begin
            fail_mask[tgt] = 1'b1; e0 = n_err;
            @(posedge sys_clk); #1 user_pending = 1; goto_idx = tgt; goto_valid = 1;
            @(posedge sys_clk); #1 goto_valid = 0;
            wait (current_image_idx == tgt); #1 user_pending = 0;
            for (i = 0; i < 4000 && view_hold; i = i + 1) frames(1);
            frames(4);
            if (current_image_idx !== a0) begin fail("L8 bad image reverts to previous"); $display("  cur=%0d exp=%0d", current_image_idx, a0); end
            if (n_err == e0 || control_error !== 8'h56) begin fail("L8 bad image reported"); $display("  err=%h", control_error); end
            if (view_hold) fail("L8 view_hold released after revert");
            fail_mask[tgt] = 1'b0;
        end
        fast_view = 0;
        settle(60);

        // ===== L10 上板第二轮：全屏匀速连按下一张（节奏略慢于一次读卡），组内 / 跨页 / 回绕都零延迟 =====
        image_count = 7'd24; go(8); settle(80);  // 24 张 = 6 页：每进一页都要给邻页现读 4 张缩略图（板上卡顿的条件）
        ld_lat = 3000;                          // 读卡 3000 拍 ≈ 3.75 帧，按键间隔 3600 拍（1.2 次读卡）
        for (i = 0; i < 13; i = i + 1) begin
            l0 = $time;
            go(la_next_tb(current_image_idx));
            if (gl != 0 || glat > 24000) begin fail("L10 steady forward press must hit the sliding window"); $display("  img=%0d loads=%0d latency=%0d ns", current_image_idx, gl, glat); end
            while ($time < l0 + 36000) @(posedge sys_clk);
        end
        // 停下来后缩略图补齐（当前页 + 邻页），窗口 B / A / A2 都在
        settle(80);
        for (i = 0; i < 4; i = i + 1)
            if (!thumb_ok({current_image_idx[5:2], i[1:0]})) begin fail("L10 current page thumbnails after idle"); $display("  img %0d", {current_image_idx[5:2], i[1:0]}); end
        if (!resident(la_next_tb(current_image_idx)) || !resident(dut.la_a2) || !resident(dut.la_b))
            fail("L10 window B / A / A2 resident after idle thumbnails");

        $display("INFO L11 start t=%0t cur=%0d", $time, current_image_idx);
        // ===== L11 第四轮：全屏切到未驻留的远处图，后台加载刚开始 → 取消，时延 ≈ 一次读卡 =====
        go(la_next_tb(current_image_idx));      // 触发前瞻（A2 / 缩略图）
        wait (ld_busy && !dut.load_is_request && ld_cnt > 2800); #1;
        tgt = (current_image_idx + 12) % 24;
        if (resident(tgt)) fail("L11 precondition: far image not resident");
        a0 = n_cancel;
        go(tgt);
        if (n_cancel == a0) fail("L11 background load cancelled for the user switch");
        if (glat > 3000 * 10 + 12000) begin fail("L11 miss latency about one card read"); $display("  latency=%0d ns", glat); end
        if (control_error != 8'h00) fail("L11 no error");
        settle(80);
        if (!resident(la_next_tb(current_image_idx))) fail("L11 window refilled after cancel");

        $display("INFO L12 start t=%0t", $time);
        // ===== L12 总览跨到远处页：旧页缩略图加载取消 =====
        fast_view = 1;
        settle(80);
        a0 = current_image_idx[5:2];
        go(((a0 + 2) % 6) * 4);                 // 隔一页 = 同一块缩略图区，缩略图一定要重新生成
        for (k = 0; k < 200000 && !(ld_busy && !dut.load_is_request && ld_cnt > 2800 && !dut.bg_cur); k = k + 1) @(posedge sys_clk);
        #1 if (k >= 200000) fail("L12 precondition: thumbnail load in flight");
        tgt = ((a0 + 5) % 6) * 4 + 1;          // 新当前页 = a0-1，邻页 = a0：在途的 a0+2 / a0+3 页加载都没用了
        a0 = n_cancel; e0 = n_err; i = ld_img;
        go(tgt);
        for (k = 0; k < 400 && n_cancel == a0; k = k + 1) @(posedge sys_clk);
        if (n_cancel == a0) begin fail("L12 stale old-page load cancelled"); $display("  in-flight img %0d new cur %0d", i, tgt); end
        settle(120);
        for (i = 0; i < 4; i = i + 1)
            if (!thumb_ok({tgt[5:2], i[1:0]})) begin fail("L12 new page thumbnails filled"); $display("  img %0d", {tgt[5:2], i[1:0]}); end
        if (view_hold || n_err != e0) fail("L12 settles without error");
        fast_view = 0;
        settle(80);

        $display("INFO L13 start t=%0t", $time);
        // ===== L13 全屏连按快于读卡（间隔 0.6 次读卡）：每次 ≤ 一次读卡 + 一帧 =====
        e0 = n_err;
        for (i = 0; i < 10; i = i + 1) begin
            l0 = $time;
            go(la_next_tb(current_image_idx));
            if (glat > 3000 * 10 + 12000) begin fail("L13 fast presses bounded by one card read"); $display("  img=%0d latency=%0d ns", current_image_idx, glat); end
            while ($time < l0 + 18000) @(posedge sys_clk);
        end
        if (n_err != e0) fail("L13 no errors");
        $display("INFO L11-L13 cancels %0d", n_cancel);
        settle(80);
        ld_lat = 300;
        go(8); image_count = 7'd10;
        settle(60);

        // ===== L7 =====
        go(8); settle(60);
        l0 = n_loads; frames(200);
        if (n_loads != l0) fail("L7 last page settles");
        if (!thumb_ok(8) || !thumb_ok(9)) fail("L7 thumbnails 8/9 kept");

        if (errors == 0) $display("PASS tb_media_player_lookahead (loads=%0d gen=%0d)", n_loads, n_gen);
        else $display("FAIL tb_media_player_lookahead errors=%0d", errors);
        $finish;
    end
    initial begin #200_000_000; $display("FAIL tb_media_player_lookahead timeout"); $finish; end
endmodule
