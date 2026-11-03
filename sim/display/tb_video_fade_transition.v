`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// tb_video_fade_transition —— 转场控制器分模块验证
//   DUT  : FADE_FRAMES=16、HOLD_TIMEOUT_FRAMES=40（ctrl 50MHz / pix 25MHz，缩短帧长 FRAME 拍）
//   DUT0 : FADE_FRAMES=0 旁路
//   控制层模型：next_event_out → CMD_DELAY 个 ctrl 拍后（可选 HOLD_EXTRA 帧）在下一个 frame_start
//   的下一拍打 index_commit（与 media_player_control 的 pix 域提交时序相同）；reject=1 时不提交、改发
//   hold_abort_in（模拟策略层拒绝 / 加载失败）。
//   检查项：
//     T1 旁路：next_event/tag 直通、alpha 恒 255、busy/dissolve 恒 0
//     T2 手动切换状态序列 IDLE→OUT→HOLD→IN→IDLE；逐帧 alpha：255,240..16,0 | 0.. | 0,16..240,255；全程 dissolve=0
//     T3 淡出完成后只放出 1 个 next_event_out，标签正确（上一张/下一张）
//     T4 HOLD 等 index_commit：延迟提交 HOLD_EXTRA 帧，期间 alpha=0、状态 HOLD，不提前淡入
//     T5 自动轮播请求（next_auto_in=1）→ 方块溶解：同样 OHNI、放出 1 个 next_event_out；OUT 段 dissolve=1、
//        ramp 16,14..2（2 倍速，不出 ramp=0 帧）；HOLD dissolve=0 全黑；IN 段 dissolve=1、ramp 2..14；
//        alpha 仍是真实淡入淡出值（给音频）。
//        T5b IDLE 里不经请求的 index_commit 不触发转场；T5c 溶解之后的手动切换 dissolve 恒 0
//     T6 转场忙期间的重复请求 / next_block_in=1 / player_active=0 时的请求全部丢弃，之后请求正常
//     T7 跨域：随机相位连续切换 N 次，每个被接受的请求恰好对应 1 次 next_event_out
//     T8 HOLD 出口：hold_abort（拒绝）→ 淡入；无提交无 abort → HOLD_TIMEOUT 帧后淡入
//     T9 各状态下复位都回到 IDLE（alpha 255、busy 0），复位后请求正常
//     全程：alpha / dissolve_active / 溶解档位只在 frame_start 或 index_commit 的下一拍变化
// -----------------------------------------------------------------------------
module tb_video_fade_transition;
    localparam integer FF      = 16;
    localparam integer FRAME   = 600;        // 每帧 pix 拍数（缩短）
    localparam integer HOLD_TO = 40;

    reg ctrl_clk = 0, pix_clk = 0;
    reg ctrl_rst_n = 0, pix_rst_n = 0;
    always #10 ctrl_clk = ~ctrl_clk;   // 50 MHz
    always #20 pix_clk  = ~pix_clk;    // 25 MHz

    integer errors = 0;
    task fail(input [8*96-1:0] msg); begin errors = errors + 1; $display("FAIL: %0s @%0t", msg, $time); end endtask

    // ---------------- 帧边界 ----------------
    integer fcnt = 0;
    reg frame_start = 0;
    always @(posedge pix_clk) begin
        fcnt <= (fcnt == FRAME - 1) ? 0 : fcnt + 1;
        frame_start <= (fcnt == FRAME - 1);
    end

    // ---------------- 激励 ----------------
    reg       ev = 0, tag = 0, aut = 0, block = 0, abort = 0;
    reg       player_active = 1;
    reg       auto_commit_req = 0;           // pix 域：模拟控制层自发提交（自动轮播）

    wire       ev_out, tag_out, busy;
    wire [7:0] alpha;
    wire       fade_active, dis;
    wire [4:0] ramp;
    reg        index_commit = 0;

    video_fade_transition #(.FADE_FRAMES(FF), .HOLD_TIMEOUT_FRAMES(HOLD_TO)) dut (
        .ctrl_clk(ctrl_clk), .ctrl_rst_n(ctrl_rst_n),
        .next_event_in(ev), .next_tag_in(tag), .next_auto_in(aut), .next_block_in(block), .hold_abort_in(abort),
        .next_event_out(ev_out), .next_tag_out(tag_out), .fade_busy(busy),
        .pix_clk(pix_clk), .pix_rst_n(pix_rst_n), .player_active(player_active),
        .frame_start(frame_start), .index_commit(index_commit),
        .alpha_out(alpha), .fade_active(fade_active), .dissolve_active(dis),
        .ramp_out(ramp));

    wire       b_ev_out, b_tag_out, b_busy, b_fade, b_dis;
    wire [7:0] b_alpha;
    wire [4:0] b_ramp;
    video_fade_transition #(.FADE_FRAMES(0)) dut0 (
        .ctrl_clk(ctrl_clk), .ctrl_rst_n(ctrl_rst_n),
        .next_event_in(ev), .next_tag_in(tag), .next_auto_in(aut), .next_block_in(block), .hold_abort_in(abort),
        .next_event_out(b_ev_out), .next_tag_out(b_tag_out), .fade_busy(b_busy),
        .pix_clk(pix_clk), .pix_rst_n(pix_rst_n), .player_active(player_active),
        .frame_start(frame_start), .index_commit(index_commit),
        .alpha_out(b_alpha), .fade_active(b_fade), .dissolve_active(b_dis),
        .ramp_out(b_ramp));

    // ---------------- 控制层模型 ----------------
    reg     reject = 0;
    integer hold_extra = 0;          // 收到请求后额外等几帧再提交（模拟 SD 加载）
    integer ev_out_count = 0, prev_count = 0;
    reg     cmd_toggle = 0;          // ctrl → pix：要提交
    always @(posedge ctrl_clk) begin
        if (ctrl_rst_n && ev_out) begin
            ev_out_count = ev_out_count + 1;
            if (tag_out) prev_count = prev_count + 1;
            if (reject) begin
                repeat (3) @(posedge ctrl_clk);
                abort <= 1'b1; @(posedge ctrl_clk); abort <= 1'b0;
            end else begin
                repeat (5) @(posedge ctrl_clk);
                cmd_toggle <= ~cmd_toggle;
            end
        end
    end
    reg cmd_s1 = 0, cmd_s2 = 0, cmd_seen = 0, pend = 0;
    integer wait_frames = 0;
    always @(posedge pix_clk) begin
        cmd_s1 <= cmd_toggle; cmd_s2 <= cmd_s1;
        index_commit <= 1'b0;
        if (cmd_s2 != cmd_seen) begin cmd_seen <= cmd_s2; pend <= 1'b1; wait_frames <= hold_extra; end
        if (auto_commit_req) begin pend <= 1'b1; wait_frames <= 0; auto_commit_req <= 1'b0; end
        if (frame_start && pend) begin
            if (wait_frames != 0) wait_frames <= wait_frames - 1;
            else begin index_commit <= 1'b1; pend <= 1'b0; end
        end
    end

    // ---------------- 全程检查：可见量只在帧边界变化 ----------------
    reg [7:0] alpha_q = 255;
    reg [4:0] bnd_q = FF;
    reg       fs_q = 0, ic_q = 0;
    reg       rst_seen = 0;
    always @(posedge pix_clk) begin
        fs_q <= frame_start; ic_q <= index_commit;
        alpha_q <= alpha; bnd_q <= dis ? ramp : FF;   // 有效档位：不溶解 = 全显 = FF
        rst_seen <= !pix_rst_n;
        if (pix_rst_n && !rst_seen && !fs_q && !ic_q) begin
            if (alpha !== alpha_q)          fail("alpha changed mid-frame");
            // 请求在帧中间进 OUT 时 dissolve 置 1、ramp=FF（全显），画面不变；只比可见的有效档位
            if ((dis ? ramp : FF) !== bnd_q) fail("dissolve visible level changed mid-frame");
        end
        // 旁路实例
        if (b_alpha !== 8'd255 || b_busy !== 1'b0 || b_dis !== 1'b0 || b_fade !== 1'b0 || b_ramp !== 0)
            fail("bypass outputs not constant");
    end
    always @(posedge ctrl_clk) begin
        if (b_ev_out !== ev || b_tag_out !== tag) fail("bypass next_event/tag not passthrough");
    end

    // 请求被接受的下一拍 fade_busy 必须已经为 1（顶层靠它屏蔽 auto_tick，不能等 busy 同步回来）
    reg acc_q = 0;
    always @(posedge ctrl_clk) begin
        acc_q <= ctrl_rst_n && dut.g_fade.accept;
        if (acc_q && !busy) fail("fade_busy not raised right after accept");
    end

    // HOLD 中收到 index_commit，下一拍必须进入 IN（不能靠超时 / abort 才离开）
    reg hold_commit_q = 0;
    always @(posedge pix_clk) begin
        hold_commit_q <= pix_rst_n && index_commit && (dut.g_fade.state == 2);
        if (hold_commit_q && dut.g_fade.state != 3) fail("T4 HOLD did not leave on index_commit");
    end

    // ---------------- 帧采样：每帧中点记录 alpha / dissolve / ramp ----------------
    integer nlog = 0;
    reg [7:0] alpha_log [0:255];
    reg       wipe_log  [0:255];
    reg [4:0] ramp_log  [0:255];
    reg [1:0] state_log [0:255];
    reg       logging = 0;
    always @(posedge pix_clk) begin
        if (logging && fcnt == FRAME / 2 && nlog < 256) begin
            alpha_log[nlog] = alpha; wipe_log[nlog] = dis; ramp_log[nlog] = ramp;
            state_log[nlog] = dut.g_fade.state;
            nlog = nlog + 1;
        end
    end

    // 状态序列记录
    reg [1:0] st_prev = 0;
    reg [8*64-1:0] seq = "";
    always @(posedge pix_clk) begin
        st_prev <= dut.g_fade.state;
        if (pix_rst_n && dut.g_fade.state != st_prev)
            seq = {seq[8*60-1:0], (dut.g_fade.state == 0) ? "I" : (dut.g_fade.state == 1) ? "O" :
                                  (dut.g_fade.state == 2) ? "H" : "N"};
    end

    task press(input t);
        begin
            @(posedge ctrl_clk); ev <= 1'b1; tag <= t;
            @(posedge ctrl_clk); ev <= 1'b0; tag <= 1'b0;
        end
    endtask
    task press_auto;
        begin
            @(posedge ctrl_clk); ev <= 1'b1; tag <= 1'b0; aut <= 1'b1;
            @(posedge ctrl_clk); ev <= 1'b0; aut <= 1'b0;
        end
    endtask
    task wait_frames_n(input integer n); integer k; begin for (k = 0; k < n; k = k + 1) @(posedge frame_start); end endtask
    // 等一次转场完整结束：先等 busy 拉高（最多 4 帧，被丢弃的请求不会拉高），再等回到 IDLE
    task wait_done; integer t; begin
        t = 0;
        while (!busy && t < 4 * FRAME) begin @(posedge ctrl_clk); t = t + 1; end
        while (dut.g_fade.state != 0 || busy) @(posedge ctrl_clk);
        repeat (10) @(posedge ctrl_clk);
    end endtask
    task wait_idle; begin
        while (dut.g_fade.state != 0 || busy) @(posedge ctrl_clk);
        repeat (10) @(posedge ctrl_clk);
    end endtask
    task wait_mid; begin while (fcnt != FRAME / 3) @(posedge pix_clk); end endtask

    integer i, k, base, exp, n0, n1, nacc;
    integer seed = 7;
    reg ok;

    initial begin
        repeat (5) @(posedge ctrl_clk);
        ctrl_rst_n = 1; pix_rst_n = 1;
        repeat (3) wait_frames_n(1);

        // ================= T1 旁路（逐拍由 always 检查）=================
        block = 1; press(1'b0); press(1'b1); repeat (400) @(posedge ctrl_clk); block = 0;
        if (busy || dut.g_fade.state != 0) fail("T1 blocked presses started a fade");
        $display("INFO: T1 bypass checked continuously");

        // ================= T2/T3/T4 手动切换（HOLD 额外等 3 帧）=================
        hold_extra = 3; seq = ""; nlog = 0;
        wait_mid; logging = 1;
        n0 = ev_out_count;
        press(1'b0);
        wait_done; wait_frames_n(2); logging = 0;
        if (seq[8*4-1:0] !== "OHNI") begin fail("T2 state sequence != OUT,HOLD,IN,IDLE"); $display("     seq=%0s", seq); end
        if (ev_out_count - n0 != 1) fail("T3 next_event_out count != 1");
        // 期望逐帧 alpha：请求帧 255 → 240..16 → 0(ramp0) → HOLD 0 ×(1+hold) → IN 0 → 16..240 → 255
        base = 0; ok = 1;
        if (alpha_log[0] !== 255) ok = 0;
        for (k = 0; k < nlog; k = k + 1) if (wipe_log[k] !== 0) ok = 0;
        for (k = 1; k <= FF - 1; k = k + 1) if (alpha_log[k] !== 256 - 16 * k) ok = 0;
        if (alpha_log[FF] !== 0) ok = 0;                         // ramp=0 帧
        i = FF + 1; n1 = 0;
        while (i < nlog && state_log[i] == 2) begin if (alpha_log[i] !== 0) ok = 0; n1 = n1 + 1; i = i + 1; end
        if (n1 < hold_extra) begin fail("T4 HOLD shorter than commit delay"); $display("     hold_frames=%0d", n1); end
        for (k = 0; k < FF; k = k + 1) begin
            if (state_log[i + k] !== 3 || alpha_log[i + k] !== 16 * k || wipe_log[i + k] !== 0) ok = 0;
        end
        if (alpha_log[i + FF] !== 255 || state_log[i + FF] !== 0) ok = 0;
        if (!ok) begin
            fail("T2 per-frame alpha sequence wrong");
            for (k = 0; k < nlog; k = k + 1) $display("     f%0d st=%0d a=%0d w=%0d", k, state_log[k], alpha_log[k], wipe_log[k]);
        end else
            $display("INFO: T2 manual fade alpha 255->0 (%0d frames), hold %0d frames, 0->255 (%0d frames); seq=%0s",
                     FF + 1, n1, FF, seq[8*4-1:0]);
        // 上一张标签
        hold_extra = 0; n0 = prev_count;
        press(1'b1); wait_done;
        if (prev_count - n0 != 1) fail("T3 prev tag not delivered");

        // ================= T5 自动轮播方块溶解 =================
        hold_extra = 2; seq = ""; nlog = 0; n0 = ev_out_count; n1 = prev_count;
        wait_mid; logging = 1;
        press_auto;
        wait_done; wait_frames_n(2); logging = 0;
        if (ev_out_count - n0 != 1 || prev_count != n1) fail("T5 auto request did not give exactly one next (not prev) event");
        if (seq[8*4-1:0] !== "OHNI") begin fail("T5 auto sequence != OUT,HOLD,IN,IDLE"); $display("     seq=%0s", seq); end
        // 溶解 2 倍速（DISSOLVE_STEP=2），两端不出 ramp=0 帧：
        // 请求帧：dissolve=1、ramp=16（全显）、alpha 255；OUT：ramp 14,12..2、音频 alpha 提前一档 192..0；
        // HOLD ≥1 帧全黑；IN：ramp 2,4..14、alpha 32..224；然后 IDLE
        ok = (wipe_log[0] === 1 && ramp_log[0] === FF && alpha_log[0] === 255);
        for (k = 1; k < FF / 2; k = k + 1)
            if (state_log[k] !== 1 || wipe_log[k] !== 1 || ramp_log[k] !== FF - 2 * k || alpha_log[k] !== 224 - 32 * k) ok = 0;
        i = FF / 2;
        n1 = 0;
        while (i < nlog && state_log[i] == 2) begin if (wipe_log[i] !== 0 || alpha_log[i] !== 0) ok = 0; i = i + 1; n1 = n1 + 1; end
        if (n1 < 1) ok = 0;
        for (k = 0; k < FF / 2 - 1; k = k + 1)
            if (state_log[i + k] !== 3 || wipe_log[i + k] !== 1 || ramp_log[i + k] !== 2 + 2 * k || alpha_log[i + k] !== 32 + 32 * k) ok = 0;
        if (wipe_log[i + FF / 2 - 1] !== 0 || alpha_log[i + FF / 2 - 1] !== 255 || state_log[i + FF / 2 - 1] !== 0) ok = 0;
        if (!ok) begin
            fail("T5 dissolve frame sequence wrong");
            for (k = 0; k < nlog; k = k + 1) $display("     f%0d st=%0d a=%0d d=%0d r=%0d", k, state_log[k], alpha_log[k], wipe_log[k], ramp_log[k]);
        end else
            $display("INFO: T5 auto dissolve x2: OUT ramp %0d..2 step 2, HOLD black %0d frame(s), IN ramp 2..%0d; audio alpha fades out/in", FF - 2, n1, FF - 2);
        // T5b：IDLE 里不经请求的 index_commit（如总览页内提交）不触发转场
        hold_extra = 0; seq = "";
        @(posedge pix_clk); auto_commit_req <= 1'b1;
        wait_frames_n(3);
        if (seq != "" || busy || dut.g_fade.state != 0 || alpha !== 255 || dis !== 0) fail("T5b bare index_commit in IDLE started a transition");
        // T5c：溶解之后的手动切换不带溶解
        nlog = 0; wait_mid; logging = 1;
        press(1'b0); wait_done; logging = 0;
        ok = 1;
        for (k = 0; k < nlog; k = k + 1) if (wipe_log[k] !== 0) ok = 0;
        if (!ok) fail("T5c manual fade after auto dissolve has dissolve_active");
        else $display("INFO: T5b bare commit ignored in IDLE; T5c manual after auto stays alpha fade");

        // ================= T6 忙期间 / block / 无图 时的请求被丢弃 =================
        n0 = ev_out_count;
        n1 = prev_count;
        press(1'b0);
        for (k = 0; k < 30; k = k + 1) begin           // 转场中狂按（含紧跟在第一个请求后的几拍）
            repeat (1 + k * 37) @(posedge ctrl_clk);
            press(k[0]);
        end
        wait_done;
        if (ev_out_count - n0 != 1) begin fail("T6 requests during busy not dropped"); $display("     outs=%0d", ev_out_count - n0); end
        if (prev_count != n1) fail("T6 tag overwritten by a request during busy");
        // 请求后紧接着第 2 个（busy 还没同步回来的窗口）
        n0 = ev_out_count;
        press(1'b0); press(1'b0); @(posedge ctrl_clk); press(1'b0); repeat (3) @(posedge ctrl_clk); press(1'b0);
        wait_done;
        if (ev_out_count - n0 != 1) begin fail("T6 second request in sync window not dropped"); $display("     outs=%0d", ev_out_count - n0); end
        n0 = ev_out_count;
        block = 1; press(1'b0); repeat (100) @(posedge ctrl_clk); block = 0;
        if (busy || dut.g_fade.state != 0) fail("T6 request accepted while blocked");
        player_active = 0; press(1'b0); wait_frames_n(2); player_active = 1;
        if (busy || dut.g_fade.state != 0) fail("T6 request accepted while player inactive");
        press(1'b0); wait_done;
        if (ev_out_count - n0 != 1) fail("T6 request after dropped ones failed");
        else $display("INFO: T6 busy/block/inactive requests dropped, following request OK");

        // 连续两拍的事件（两次翻转可能落在同一个 pix 采样间隔里）：两种相位各试一次
        for (k = 0; k < 2; k = k + 1) begin
            n0 = ev_out_count;
            @(posedge pix_clk); if (k) @(posedge ctrl_clk);
            @(posedge ctrl_clk); ev <= 1'b1; tag <= 1'b0;
            @(posedge ctrl_clk); @(posedge ctrl_clk); ev <= 1'b0;
            wait_done;
            if (ev_out_count - n0 != 1) begin fail("T6 double-pulse event lost or duplicated"); $display("     phase=%0d outs=%0d", k, ev_out_count - n0); end
        end

        // ================= T7 跨域：随机相位连续切换 =================
        n0 = ev_out_count; nacc = 0;
        for (k = 0; k < 20; k = k + 1) begin
            repeat ($unsigned($random(seed)) % 997 + 1) @(posedge ctrl_clk);
            @(posedge ctrl_clk); ev <= 1'b1; tag <= 0;
            @(posedge ctrl_clk); ev <= 1'b0;            @(posedge pix_clk);
            wait_done;
            nacc = nacc + 1;
        end
        if (ev_out_count - n0 != nacc) begin fail("T7 CDC request lost or duplicated"); $display("     acc=%0d outs=%0d", nacc, ev_out_count - n0); end
        else $display("INFO: T7 %0d random-phase requests -> %0d commits (no loss / no dup)", nacc, ev_out_count - n0);

        // ================= T8 HOLD 出口：拒绝 abort / 超时 =================
        reject = 1; seq = ""; n0 = ev_out_count;
        press(1'b0);
        while (dut.g_fade.state != 2) @(posedge pix_clk);
        n1 = 0;
        while (dut.g_fade.state == 2) begin @(posedge pix_clk); if (fcnt == FRAME / 2) n1 = n1 + 1; end
        if (n1 > 3) begin fail("T8 reject: HOLD not ended by abort"); $display("     hold=%0d", n1); end
        wait_idle;
        if (seq[8*4-1:0] !== "OHNI") fail("T8 reject did not leave HOLD");
        reject = 0;
        // 超时：控制层完全不响应（不提交也不 abort）
        hold_extra = 1000; seq = "";
        press(1'b0);
        while (dut.g_fade.state != 2) @(posedge pix_clk);
        n1 = 0;
        while (dut.g_fade.state == 2) begin @(posedge pix_clk); if (fcnt == FRAME / 2) n1 = n1 + 1; end
        if (n1 != HOLD_TO) begin fail("T8 HOLD timeout frame count wrong"); $display("     hold=%0d", n1); end
        wait_idle;
        // 超时后那个迟到的提交在 IDLE 到达 → 不触发转场，清掉模型
        hold_extra = 0;
        pend = 0;
        $display("INFO: T8 reject->abort fade-in OK, timeout after %0d frames", n1);

        // ================= T9 各状态下复位 =================
        for (k = 1; k <= 4; k = k + 1) begin
            wait_idle;
            if (k == 4) press_auto;
            else press(1'b0);
            hold_extra = 2;
            // 等到目标状态：1=OUT 2=HOLD 3=IN(淡入) 4=IN(溶解)
            while (dut.g_fade.state != ((k == 4) ? 3 : k)) @(posedge pix_clk);
            wait_frames_n(1); repeat (37) @(posedge pix_clk);
            ctrl_rst_n = 0; pix_rst_n = 0;
            repeat (4) @(posedge ctrl_clk);
            if (dut.g_fade.state != 0 || alpha !== 255 || busy !== 0 || dis !== 0)
                fail("T9 not IDLE after reset");
            ctrl_rst_n = 1; pix_rst_n = 1;
            cmd_seen = cmd_s2; pend = 0;
            repeat (4) @(posedge ctrl_clk);
            hold_extra = 0;
        end
        wait_frames_n(2);
        n0 = ev_out_count; seq = "";
        press(1'b0); wait_done;
        if (ev_out_count - n0 != 1 || seq[8*4-1:0] !== "OHNI") fail("T9 manual switch after reset failed");
        else $display("INFO: T9 reset in OUT/HOLD/IN/DISSOLVE -> IDLE, then manual switch OK");

        if (errors == 0) $display("PASS: tb_video_fade_transition");
        else $display("FAIL: tb_video_fade_transition errors=%0d", errors);
        $finish;
    end

    initial begin
        #200_000_000;
        $display("FAIL: timeout");
        $finish;
    end
endmodule
