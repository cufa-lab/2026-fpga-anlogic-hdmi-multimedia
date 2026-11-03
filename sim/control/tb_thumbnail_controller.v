`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// tb_thumbnail_controller —— 缩略图交互控制器（模块级，sys 50MHz / pix 25MHz）
// 两个实例同时跑：a = FOCUS_FOLLOW 0，b = FOCUS_FOLLOW 1（上板交互）
// 全程断言（两个实例都查）：
//   G1 view_mode_pix / focus_pix 只在 frame_start 那一拍更新（帧内不变 → 不撕裂）
//   G2 transition_req / select_valid / select_reject / view_commit 严格单拍
//   G3 握手不漏不重：静止后 pix 生效值 = sys 请求值，view_commit 次数 = req_toggle 翻转次数
// 练习模式 a：
//   A1 复位：全屏、焦点 0、当前格 0、pix 侧也为 0
//   A2 全屏下方向键不改 focus_index，nav_enable=1
//   A3 模式键：1 个 transition_req，view_mode=1，焦点 = 当前图格子；帧边界后 view_mode_pix=1
//   A4 右键 0→1→2→3→0、左键 0→3→2→1→0；pix 焦点跟随
//   A5 image_complete 四位逐一：只有 bit s 的格子能选；选中 → 1 个 select_valid、目标 = 页·4+s；
//      控制层提交后才回全屏（1 个 transition_req）；其它格子 → select_reject，状态不变
//   A6 目标 ≥ 图片数 → 拒绝；目标 = 当前图 → 不发 select，直接回全屏
//   A7 select_busy 期间确认 → 排队，不发；再确认 / 方向键不覆盖；空闲后恰好 1 个 select_valid
//   A8 选择失败（select_error）→ select_reject，留在缩略图，焦点不变
//   A9 select_busy 时模式键排队，空闲后才切；忙中按两次 = 取消
//   A10 一帧内连按 3 次右键：pix 焦点最终 = 请求值，提交次数 = 翻转次数
// 上板模式 b：
//   B1 enable=0 时拨上（key_mode）不动作但保留；enable 后进入缩略图
//   B2 焦点跟随 current_image_idx（方向键本身不改焦点），pix 帧边界跟随
//   B3 切图在途拨下（key_confirm）：nav_enable=0，等 select_busy 落下才 1 个 transition_req 回全屏
//   B4 等待中又拨上（抖动）→ 取消退出，仍在缩略图
//   B5 缩略图中 enable 掉 → 退回全屏
// -----------------------------------------------------------------------------
module tb_thumbnail_controller;
    reg clk = 0, pix_clk = 0, rst_n = 0;
    always #10 clk = ~clk;
    always #20 pix_clk = ~pix_clk;

    // 帧：每 300 个 pix 拍一个 frame_start（NBA 驱动，与 RTL 同一采样关系）
    reg frame_start = 0;
    integer pcnt = 0;
    always @(posedge pix_clk) begin
        pcnt <= (pcnt == 299) ? 0 : pcnt + 1;
        frame_start <= (pcnt == 299);
    end

    integer errors = 0;
    task fail(input [8*80-1:0] msg); begin errors = errors + 1; if (errors < 40) $display("FAIL %0s @%0t", msg, $time); end endtask

    // ---------------- 实例 a：练习模式 ----------------
    reg a_en = 1, a_r = 0, a_l = 0, a_c = 0, a_m = 0, a_busy = 0, a_err = 0;
    reg [3:0] a_cpl = 4'hF;
    reg [5:0] a_cur = 0;
    reg [6:0] a_cnt = 7'd8;
    wire a_vm, a_tr, a_sv, a_rj, a_nav, a_vb, a_vmp, a_vc;
    wire [1:0] a_fi, a_ai, a_fp;
    wire [5:0] a_si;
    thumbnail_controller #(.FOCUS_FOLLOW(0)) da (
        .clk(clk), .rst_n(rst_n), .enable(a_en), .key_right(a_r), .key_left(a_l), .key_confirm(a_c), .key_mode(a_m),
        .image_complete(a_cpl), .current_image_idx(a_cur), .image_count(a_cnt), .select_busy(a_busy), .select_error(a_err),
        .view_mode(a_vm), .focus_index(a_fi), .active_index(a_ai), .transition_req(a_tr),
        .select_valid(a_sv), .select_image_idx(a_si), .select_reject(a_rj), .nav_enable(a_nav), .view_busy(a_vb),
        .pix_clk(pix_clk), .pix_rst_n(rst_n), .frame_start(frame_start), .view_mode_pix(a_vmp), .focus_pix(a_fp), .view_commit(a_vc));

    // ---------------- 实例 b：上板模式 ----------------
    reg b_en = 0, b_r = 0, b_l = 0, b_c = 0, b_m = 0, b_busy = 0, b_err = 0;
    reg [5:0] b_cur = 0;
    wire b_vm, b_tr, b_sv, b_rj, b_nav, b_vb, b_vmp, b_vc;
    wire [1:0] b_fi, b_ai, b_fp;
    wire [5:0] b_si;
    thumbnail_controller #(.FOCUS_FOLLOW(1)) db (
        .clk(clk), .rst_n(rst_n), .enable(b_en), .key_right(b_r), .key_left(b_l), .key_confirm(b_c), .key_mode(b_m),
        .image_complete(4'hF), .current_image_idx(b_cur), .image_count(7'd4), .select_busy(b_busy), .select_error(b_err),
        .view_mode(b_vm), .focus_index(b_fi), .active_index(b_ai), .transition_req(b_tr),
        .select_valid(b_sv), .select_image_idx(b_si), .select_reject(b_rj), .nav_enable(b_nav), .view_busy(b_vb),
        .pix_clk(pix_clk), .pix_rst_n(rst_n), .frame_start(frame_start), .view_mode_pix(b_vmp), .focus_pix(b_fp), .view_commit(b_vc));

    // ---------------- G1：pix 生效值只在 frame_start 后变化 ----------------
    reg [2:0] a_pq = 0, b_pq = 0;
    reg fs_q = 0;
    always @(posedge pix_clk) begin
        if (rst_n) begin
            if ({a_vmp, a_fp} !== a_pq && !fs_q) fail("G1 a pix view/focus changed outside frame_start");
            if ({b_vmp, b_fp} !== b_pq && !fs_q) fail("G1 b pix view/focus changed outside frame_start");
            if ((a_vc || b_vc) && !fs_q) fail("G1 view_commit outside frame_start");
        end
        a_pq <= {a_vmp, a_fp}; b_pq <= {b_vmp, b_fp}; fs_q <= frame_start;
    end

    // ---------------- G2：单拍 + 计数 ----------------
    reg a_tr_q = 0, a_sv_q = 0, a_rj_q = 0, b_tr_q = 0, a_vc_q = 0, b_vc_q = 0;
    integer a_ntr = 0, a_nsv = 0, a_nrj = 0, b_ntr = 0;
    always @(posedge clk) begin
        if (a_tr && a_tr_q) fail("G2 a transition_req 2 cycles");
        if (a_sv && a_sv_q) fail("G2 a select_valid 2 cycles");
        if (a_rj && a_rj_q) fail("G2 a select_reject 2 cycles");
        if (b_tr && b_tr_q) fail("G2 b transition_req 2 cycles");
        if (b_sv || b_rj) fail("G2 b follow mode must never select/reject");
        a_tr_q <= a_tr; a_sv_q <= a_sv; a_rj_q <= a_rj; b_tr_q <= b_tr;
        if (a_tr) a_ntr = a_ntr + 1;
        if (a_sv) a_nsv = a_nsv + 1;
        if (a_rj) a_nrj = a_nrj + 1;
        if (b_tr) b_ntr = b_ntr + 1;
    end
    integer a_ntog = 0, a_ncom = 0, b_ntog = 0, b_ncom = 0;
    reg a_tog_q = 0, b_tog_q = 0;
    always @(posedge clk) begin
        if (da.req_toggle !== a_tog_q) a_ntog = a_ntog + 1;
        if (db.req_toggle !== b_tog_q) b_ntog = b_ntog + 1;
        a_tog_q <= da.req_toggle; b_tog_q <= db.req_toggle;
    end
    always @(posedge pix_clk) begin
        if (a_vc && a_vc_q) fail("G2 a view_commit 2 cycles");
        if (b_vc && b_vc_q) fail("G2 b view_commit 2 cycles");
        a_vc_q <= a_vc; b_vc_q <= b_vc;
        if (a_vc) a_ncom = a_ncom + 1;
        if (b_vc) b_ncom = b_ncom + 1;
    end

    // ---------------- 工具 ----------------
    task pa_r; begin @(posedge clk); #1 a_r = 1; @(posedge clk); #1 a_r = 0; end endtask
    task pa_l; begin @(posedge clk); #1 a_l = 1; @(posedge clk); #1 a_l = 0; end endtask
    task pa_c; begin @(posedge clk); #1 a_c = 1; @(posedge clk); #1 a_c = 0; end endtask
    task pa_m; begin @(posedge clk); #1 a_m = 1; @(posedge clk); #1 a_m = 0; end endtask
    task pa_err; begin @(posedge clk); #1 a_err = 1; @(posedge clk); #1 a_err = 0; end endtask
    task pb_c; begin @(posedge clk); #1 b_c = 1; @(posedge clk); #1 b_c = 0; end endtask
    task pb_m; begin @(posedge clk); #1 b_m = 1; @(posedge clk); #1 b_m = 0; end endtask
    task pb_r; begin @(posedge clk); #1 b_r = 1; @(posedge clk); #1 b_r = 0; end endtask
    task frames(input integer n); integer i; begin for (i = 0; i < n; i = i + 1) @(posedge frame_start); repeat (20) @(posedge clk); end endtask
    task sys(input integer n); begin repeat (n) @(posedge clk); #1; end endtask
    task settle; begin frames(3); end endtask
    task check_g3; begin
        settle;
        if ({a_vmp, a_fp} !== {a_vm, a_fi}) fail("G3 a pix != request after settle");
        if ({b_vmp, b_fp} !== {b_vm, b_fi}) fail("G3 b pix != request after settle");
        if (a_ntog != a_ncom) begin fail("G3 a toggles != commits"); $display("  a tog=%0d com=%0d", a_ntog, a_ncom); end
        if (b_ntog != b_ncom) begin fail("G3 b toggles != commits"); $display("  b tog=%0d com=%0d", b_ntog, b_ncom); end
    end endtask

    // 让 a 回到全屏，并把焦点移到 f（a 必须在缩略图里）
    task a_focus_to(input [1:0] f); integer k; begin
        for (k = 0; k < 4 && a_fi != f; k = k + 1) pa_r;
        if (a_fi != f) fail("a_focus_to");
    end endtask
    task a_enter; integer t0; begin
        t0 = a_ntr; pa_m; sys(4);
        if (a_vm !== 1'b1 || a_ntr != t0 + 1) fail("A3 enter thumbnail");
        settle;
        if (a_vmp !== 1'b1) fail("A3 view_mode_pix not 1 after frames");
    end endtask
    // 模拟控制层：忙一段 → 提交目标 → 空闲
    // 等 sig 变成 val，最多 2 帧（模式变化要排在上一次 {模式, 焦点} 帧边界生效之后）
    task wait_a_vm(input val); integer k; begin
        for (k = 0; k < 1200 && a_vm !== val; k = k + 1) @(posedge clk);
        #1;
    end endtask
    task wait_b_vm(input val); integer k; begin
        for (k = 0; k < 1200 && b_vm !== val; k = k + 1) @(posedge clk);
        #1;
    end endtask
    task a_player_commit(input [5:0] tgt); begin
        sys(6); a_busy = 1; sys(300); a_cur = tgt; sys(3); a_busy = 0; wait_a_vm(1'b0);
    end endtask

    integer s, f, t0, v0, r0, i;
    reg [5:0] tgt;
    initial begin
        #200; rst_n = 1; sys(5);

        // ===== A1 / B 复位 =====
        if (a_vm !== 0 || a_fi !== 0 || a_ai !== 0 || a_vmp !== 0 || a_fp !== 0) fail("A1 reset state");
        if (b_vm !== 0 || b_fi !== 0 || b_vmp !== 0 || b_fp !== 0) fail("B reset state");
        if (a_tr || a_sv || a_rj) fail("A1 pulses at reset");

        // ===== A2 全屏下方向键 =====
        pa_r; pa_r; pa_l; sys(4);
        if (a_fi !== 0) fail("A2 focus moved in fullscreen");
        if (a_nav !== 1) fail("A2 nav_enable should be 1 in fullscreen");

        // ===== A3 进缩略图（当前图在第 1 页第 0 格：idx 4）=====
        a_cur = 6'd4; sys(2);
        a_enter;
        if (a_fi !== 2'd0) fail("A3 focus should start at current cell");
        if (a_nav !== 0) fail("A3 nav_enable should be 0 in thumbnail");

        // ===== A4 焦点循环 =====
        for (i = 1; i <= 4; i = i + 1) begin pa_r; sys(2); if (a_fi !== i[1:0]) fail("A4 right wrap sequence"); end
        if (a_fi !== 0) fail("A4 right ends at 0");
        for (i = 3; i >= 0; i = i - 1) begin pa_l; sys(2); if (a_fi !== i[1:0]) fail("A4 left wrap sequence"); end
        pa_l; sys(2); if (a_fi !== 2'd3) fail("A4 left 0->3");
        pa_r; sys(2); if (a_fi !== 2'd0) fail("A4 right 3->0");
        check_g3;
        if (a_fp !== 2'd0) fail("A4 pix focus follow");

        // ===== A5 image_complete 四位逐一 =====
        for (s = 0; s < 4; s = s + 1) begin
            a_cpl = 4'b0001 << s;
            for (f = 0; f < 4; f = f + 1) begin
                if (a_vm !== 1) a_enter;
                a_focus_to(f[1:0]); settle;
                v0 = a_nsv; r0 = a_nrj; t0 = a_ntr;
                pa_c; sys(6);
                tgt = {a_cur[5:2], f[1:0]};
                if (f != s) begin
                    if (a_nrj != r0 + 1 || a_nsv != v0) fail("A5 unloaded cell must be rejected");
                    if (a_vm !== 1 || a_fi !== f[1:0]) fail("A5 state changed on reject");
                    settle;
                    if (a_vmp !== 1) fail("A5 pix left thumbnail on reject");
                end else if (tgt == a_cur) begin
                    // A6 目标 = 当前图：直接回全屏
                    if (a_nsv != v0) fail("A6 select issued for current image");
                    sys(4);
                    if (a_vm !== 0 || a_ntr != t0 + 1) fail("A6 should exit to fullscreen directly");
                    settle;
                end else begin
                    if (a_nsv != v0 + 1 || a_si !== tgt) begin fail("A5 select_valid / target"); $display("  si=%0d tgt=%0d", a_si, tgt); end
                    if (a_vm !== 1) fail("A5 must stay thumbnail until commit");
                    a_player_commit(tgt);
                    if (a_vm !== 0 || a_ntr != t0 + 1) fail("A5 exit to fullscreen after commit");
                    settle;
                    if (a_vmp !== 0) fail("A5 pix fullscreen after commit");
                    // 回到第 1 页第 0 格作为下一轮的当前图
                    a_cur = 6'd4; sys(2);
                end
            end
        end
        a_cpl = 4'hF;
        check_g3;

        // ===== A6 目标 >= 图片数 =====
        a_cnt = 7'd6;                              // 第 1 页只有 idx 4、5
        if (a_vm !== 1) a_enter;
        a_focus_to(2'd2); r0 = a_nrj; v0 = a_nsv;
        pa_c; sys(6);
        if (a_nrj != r0 + 1 || a_nsv != v0) fail("A6 nonexistent image must be rejected");
        a_cnt = 7'd8;

        // ===== A7 忙时排队、不覆盖 =====
        a_focus_to(2'd2); a_busy = 1; v0 = a_nsv; sys(2);
        pa_c; sys(50);
        if (a_nsv != v0) fail("A7 select issued while busy");
        pa_r; sys(4);
        if (a_fi !== 2'd2) fail("A7 focus moved while selection pending");
        pa_c; sys(4);
        if (a_nsv != v0) fail("A7 second confirm while pending");
        a_busy = 0; sys(6);
        if (a_nsv != v0 + 1 || a_si !== 6'd6) fail("A7 queued select after busy");
        a_player_commit(6'd6);
        if (a_vm !== 0) fail("A7 exit after commit");
        a_cur = 6'd4; settle;

        // ===== A8 选择失败 =====
        a_enter; a_focus_to(2'd3); v0 = a_nsv; r0 = a_nrj;
        pa_c; sys(6);
        if (a_nsv != v0 + 1) fail("A8 select not issued");
        sys(20); pa_err; sys(6);
        if (a_nrj != r0 + 1) fail("A8 reject on select_error");
        if (a_vm !== 1 || a_fi !== 2'd3) fail("A8 must stay thumbnail with focus");
        settle;

        // ===== A9 忙时模式键 =====
        a_busy = 1; t0 = a_ntr; sys(2);
        pa_m; sys(100);
        if (a_ntr != t0 || a_vm !== 1) fail("A9 mode switched while busy");
        a_busy = 0; wait_a_vm(1'b0);
        if (a_ntr != t0 + 1 || a_vm !== 0) fail("A9 queued mode after busy");
        settle;
        a_busy = 1; t0 = a_ntr; sys(2);
        pa_m; sys(10); pa_m; sys(10);                // 按两下 = 取消
        a_busy = 0; sys(20);
        if (a_ntr != t0 || a_vm !== 0) fail("A9 double mode press should cancel");
        settle;

        // ===== A10 一帧内连按 =====
        a_enter;
        @(posedge frame_start); sys(10);
        pa_r; sys(3); pa_r; sys(3); pa_r; sys(3);
        check_g3;
        if (a_fp !== a_fi) fail("A10 pix focus final");

        // ===== B1 上电时 SW4 已拨上 =====
        pb_m; sys(50); frames(2);
        if (b_vm !== 0 || b_ntr != 0) fail("B1 entered before enable");
        b_en = 1; sys(6);
        if (b_vm !== 1 || b_ntr != 1) fail("B1 enter after enable");
        settle;
        if (b_vmp !== 1) fail("B1 pix thumbnail");

        // ===== B2 焦点跟随 =====
        for (i = 1; i <= 4; i = i + 1) begin
            b_cur = i[5:0] & 6'd3; sys(3);
            if (b_fi !== b_cur[1:0]) fail("B2 focus follows current");
        end
        pb_r; sys(3);
        if (b_fi !== b_cur[1:0]) fail("B2 key_right must not move focus by itself");
        if (b_nav !== 1) fail("B2 nav_enable in idle thumbnail");
        b_cur = 6'd2; check_g3;
        if (b_fp !== 2'd2) fail("B2 pix focus");

        // ===== B3 切图在途拨下 =====
        b_busy = 1; t0 = b_ntr; sys(2);
        pb_c; sys(4);
        if (b_nav !== 0) fail("B3 nav_enable must be 0 while exit pending");
        sys(300);
        if (b_vm !== 1 || b_ntr != t0) fail("B3 exited while busy");
        b_cur = 6'd3; sys(2); b_busy = 0; wait_b_vm(1'b0);
        if (b_vm !== 0 || b_ntr != t0 + 1) fail("B3 exit after busy");
        if (b_nav !== 1) fail("B3 nav_enable after exit");
        settle;
        if (b_vmp !== 0) fail("B3 pix fullscreen");

        // ===== B4 等待中抖回 =====
        pb_m; sys(6); settle;
        if (b_vm !== 1) fail("B4 enter");
        b_busy = 1; t0 = b_ntr; sys(2);
        pb_c; sys(20); pb_m; sys(20);
        b_busy = 0; sys(20);
        if (b_vm !== 1 || b_ntr != t0) fail("B4 bounce should cancel exit");
        check_g3;

        // ===== B5 enable 掉 =====
        b_en = 0; sys(6);
        if (b_vm !== 0) fail("B5 exit on enable low");
        check_g3;

        if (errors == 0) $display("PASS tb_thumbnail_controller (a: tr=%0d sv=%0d rj=%0d commits=%0d, b: tr=%0d commits=%0d)",
                                  a_ntr, a_nsv, a_nrj, a_ncom, b_ntr, b_ncom);
        else $display("FAIL tb_thumbnail_controller errors=%0d", errors);
        $finish;
    end
    initial begin #400_000_000; $display("FAIL tb_thumbnail_controller timeout"); $finish; end
endmodule
