`timescale 1ns/1ps
// tb_tour_sequencer ——几何表 / 焦点换算 / 缓动 / 停留 / 跳过 / 中途回家 / 中止
//   该图没有 FOCUS 条目 → chord 不响应
//   帧节拍 = 每 400 拍一个 frame_tick；DWELL 10 帧（参数缩小，加快仿真）
//   bundle 跨域约定：只允许在 frame_tick 之后 ≤ 2000 拍内变化（pix 在 frame_start 锁存）
module tb_tour_sequencer;
    reg clk = 0, rst_n = 0;
    always #10 clk = ~clk;
    integer errors = 0;
    task fail(input [8*72-1:0] m); begin $display("FAIL %0s  @%0t", m, $time); errors = errors + 1; end endtask

    reg chord = 0, tour_next = 0, enable = 1, block = 0, start_block = 0;
    reg [5:0] cur_img = 6'd2;
    reg frame_tick = 0;
    reg [1:0] disp_bank = 2'd1;
    reg fb_load_start = 0; reg [1:0] fb_load_bank = 0;
    reg plan_valid = 0;
    reg [9:0] g_ox = 0; reg [8:0] g_oyt = 0;
    reg [11:0] g_c0 = 0, g_r0t = 0, g_cw = 0, g_ch = 0, g_n = 1;
    reg [9:0] g_d = 1;
    wire [7:0] ft_addr; reg [26:0] ft_data = 0;
    reg [26:0] ftab [0:255];
    always @(posedge clk) ft_data <= ftab[ft_addr];
    wire tour_busy;
    wire [80:0] bundle;

    tour_sequencer #(.START_FRAMES(3), .DWELL_FRAMES(10)) dut (
        .clk(clk), .rst_n(rst_n), .chord(chord), .tour_next(tour_next), .enable(enable), .block(block), .start_block(start_block),
        .cur_img(cur_img), .frame_tick(frame_tick), .disp_bank(disp_bank),
        .fb_load_start(fb_load_start), .fb_load_bank(fb_load_bank), .plan_valid(plan_valid),
        .g_ox(g_ox), .g_oyt(g_oyt), .g_c0(g_c0), .g_r0t(g_r0t), .g_cw(g_cw), .g_ch(g_ch), .g_n(g_n), .g_d(g_d),
        .ft_addr(ft_addr), .ft_data(ft_data), .tour_busy(tour_busy), .bundle(bundle));

    wire        b_on = bundle[0], b_bit1 = bundle[1];
    wire [15:0] b_u = bundle[19:4];
    wire [14:0] b_v = bundle[34:20];
    wire [16:0] b_s = bundle[51:35];

    // 帧节拍
    integer frames = 0;
    always begin repeat (399) @(posedge clk); frame_tick <= 1; @(posedge clk); frame_tick <= 0; frames = frames + 1; end

    // bundle 只在提交窗口里变；bit1 恒 0
    integer since_tick = 99999, win_bad = 0, nchg = 0;
    reg [80:0] bprev = 0;
    always @(posedge clk) begin
        since_tick = frame_tick ? 0 : since_tick + 1;
        if (bundle !== bprev) begin
            nchg = nchg + 1;
            if (since_tick > 2000 && frames > 0) win_bad = win_bad + 1;
        end
        if (b_bit1) fail("bundle bit1 set");
        bprev <= bundle;
    end

    // 缓动单调性：一次飞行内 u 的每次变化都朝同一方向
    integer ease_bad = 0, ldir = 0;
    reg [15:0] lu; reg fly_q = 0;
    always @(posedge clk) begin
        if (dut.st == 3'd5 && !fly_q) begin fly_q <= 1; lu <= b_u; ldir = 0; end
        else if (dut.st == 3'd5 && b_u != lu) begin
            if (ldir == 0) ldir = (b_u > lu) ? 1 : -1;
            else if ((ldir > 0 && b_u < lu) || (ldir < 0 && b_u > lu)) ease_bad = ease_bad + 1;
            lu <= b_u;
        end
        if (dut.st != 3'd5) fly_q <= 0;                                  // 重新计算目标（T_CALC）= 新的一次飞行
    end

    task plan(input [1:0] bank, input [9:0] ox, input [8:0] oyt, input [11:0] c0, r0t, cw, ch, n, input [9:0] d);
        begin
            @(negedge clk); fb_load_start = 1; fb_load_bank = bank; @(negedge clk); fb_load_start = 0;
            repeat (50) @(negedge clk);
            g_ox = ox; g_oyt = oyt; g_c0 = c0; g_r0t = r0t; g_cw = cw; g_ch = ch; g_n = n; g_d = d;
            plan_valid = 1; @(negedge clk); plan_valid = 0;
            repeat (20) @(negedge clk);                                              // 几何 8 拍顺序写入（缩放器 g_* 保持）
            g_ox = 0; g_oyt = 0; g_c0 = 0; g_n = 1; g_d = 1;                           // 之后变化不应影响已写入的表
        end
    endtask
    task press_chord; begin @(negedge clk); chord = 1; @(negedge clk); chord = 0; end endtask
    task press_next;  begin @(negedge clk); tour_next = 1; @(negedge clk); tour_next = 0; end endtask

    task expect_view(input integer u, v, s, input [8*24-1:0] tag);
        begin
            $display("INFO %0s view u %0d v %0d s %0d (frame %0d)", tag, b_u, b_v, b_s, frames);
            if (bundle[80:52] !== 29'd0 || bundle[3:1] !== 3'd0) fail("unused bundle bits (no minimap) must be 0");
            if ((b_u > u + 2) || (b_u + 2 < u) || (b_v > v + 2) || (b_v + 2 < v) || (b_s > s + 2) || (b_s + 2 < s)) begin
                $display("INFO %0s expect %0d %0d %0d", tag, u, v, s);
                fail("view target");
            end
        end
    endtask
    task wait_st(input [2:0] s, input integer maxf); integer f0; begin
        f0 = frames;
        while (dut.st !== s && frames - f0 < maxf) @(posedge clk);
        if (dut.st !== s) begin $display("INFO wait st %0d, now %0d", s, dut.st); fail("state timeout"); end
    end endtask
    task commit; begin repeat (400 + 300) @(posedge clk); end endtask   // 等下一帧提交完

    integer f0;
    initial begin
        for (f0 = 0; f0 < 256; f0 = f0 + 1) ftab[f0] = 0;
        // 图 2：焦点 (1000,500,2.0)、(1800,1000,3.0)、(100,100,1.0)
        ftab[{6'd2, 2'd0}] = {12'd1000, 12'd500, 3'd3};
        ftab[{6'd2, 2'd1}] = {12'd1800, 12'd1000, 3'd5};
        ftab[{6'd2, 2'd2}] = {12'd100, 12'd100, 3'd1};
        // 图 5：一个焦点 (540,300,2.0)；图 6：没有条目
        ftab[{6'd5, 2'd0}] = {12'd540, 12'd300, 3'd3};
        repeat (5) @(posedge clk); rst_n = 1;
        repeat (20) @(posedge clk);

        // ---- 0. 没有几何不能进入 ----
        press_chord; repeat (50) @(posedge clk);
        if (tour_busy) fail("entered without geometry");

        // ---- 1. 1920×1080 FILL（bank 1）：窗口 1440×1080 (c0 = 240)，N = 1440 D = 640 ----
        plan(2'd1, 10'd0, 9'd0, 12'd240, 12'd0, 12'd1440, 12'd1080, 12'd1440, 10'd640);
        // bank 2：1080×1920 竖图 FIT，ox = 185，N = 1920 D = 480
        plan(2'd2, 10'd185, 9'd0, 12'd0, 12'd0, 12'd1080, 12'd1920, 12'd1920, 10'd480);
        start_block = 1; press_chord; repeat (50) @(posedge clk);
        if (tour_busy) fail("entered while start_block");
        start_block = 0;
        press_chord;
        repeat (20) @(posedge clk);
        if (!tour_busy) fail("start: busy");
        commit;
        if (!b_on) fail("start state");
        if (b_s !== 17'd65536 || b_u !== 0 || b_v !== 0) fail("home view at start");
        wait_st(3'd5, 10);
        commit;
        if (!b_on) fail("on after start");
        // 焦点 0：Fu6 = 760·40960/1440 = 21617，W6 = 20480 → u0 = 11377；Fv6 = 14222，H6 = 15360 → v0 = 6542
        wait_st(3'd6, 80);
        expect_view(11377, 6542, 32768, "focus0");
        if (ease_bad != 0) fail("easing not monotonic");
        f0 = frames;
        wait_st(3'd5, 30);                                                         // 停 10 帧后飞焦点 1
        if (frames - f0 < 9) fail("dwell too short");
        // 焦点 1：(1800,1000) z = 3.0 → x 夹到 1679 → Fu6 = 40931、W6 = 13653 → u0 夹到 27307；
        //   Fv6 = 28444、H6 = 10239 → v0 夹到 20481；s = 21845
        wait_st(3'd6, 80);
        expect_view(27307, 20481, 21845, "focus1 clamp");
        // KEY2 跳过：停留中立刻去下一个
        repeat (400 * 2) @(posedge clk);
        f0 = frames;
        press_next;
        repeat (10) @(posedge clk);
        if (dut.st == 3'd6) fail("skip ignored");
        // 焦点 2：zoom 1.0 = 整图视角
        wait_st(3'd6, 80);
        if (frames - f0 > 70) fail("skip too slow");
        expect_view(0, 0, 65536, "focus2 zoom1");
        // 焦点 3 不存在 → 回整图 → 结束
        wait_st(3'd7, 100);
        repeat (5) @(posedge clk);
        if (tour_busy) fail("not ended");
        commit;
        if (b_on) fail("bundle on after end");
        expect_view(0, 0, 65536, "home");
        if (ease_bad != 0) fail("easing not monotonic (tour 1)");
        $display("INFO tour 1 done: %0d frames, bundle changes %0d", frames, nchg);

        // ---- 2a. 该图没有 FOCUS 条目 → chord 不响应（照常播放）----
        disp_bank = 2'd2; cur_img = 6'd6;
        repeat (10) @(posedge clk);
        press_chord; repeat (50) @(posedge clk);
        if (tour_busy || dut.st != 3'd0) fail("entered without FOCUS entry");
        commit;
        if (b_on) fail("bundle on without FOCUS entry");
        // ---- 2b. 竖图 FIT（bank 2）、图 5 一个焦点 (540,300,2.0)：u0 = 185·64 + 540·30720/1920 - 10240 = 10240 ----
        //          回家途中再按 chord 不应重复触发
        cur_img = 6'd5;
        repeat (10) @(posedge clk);
        press_chord;
        wait_st(3'd5, 10);
        wait_st(3'd6, 80);
        expect_view(10240, 0, 32768, "FIT focus");                              // v：300·30720/1920 = 4800 - 7680 < 0 → 0
        wait_st(3'd5, 30);                                                         // 只有一个焦点 → 下一个 = 回家
        repeat (400 * 20) @(posedge clk);
        press_chord;                                                               // 回家途中再按：不应重复触发
        wait_st(3'd7, 100);
        repeat (5) @(posedge clk);
        if (tour_busy) fail("tour 2 not ended");
        commit;
        expect_view(0, 0, 65536, "home2");

        // ---- 3. 去焦点 0 的飞行中按 chord → 掉头回家 ----
        disp_bank = 2'd1; cur_img = 6'd2;
        repeat (10) @(posedge clk);
        press_chord;
        wait_st(3'd5, 10);
        repeat (400 * 20) @(posedge clk);
        press_chord;
        wait_st(3'd7, 100);
        commit;
        if (tour_busy || b_on) fail("chord home in flight");
        expect_view(0, 0, 65536, "home3");
        if (ease_bad != 0) fail("easing not monotonic (u-turn)");

        // ---- 4. 中止：停留中换图 → 立即结束 ----
        press_chord;
        wait_st(3'd6, 100);
        cur_img = 6'd3;
        repeat (3) @(posedge clk);
        if (tour_busy) fail("abort on image change");
        commit;
        if (b_on) fail("bundle on after abort");
        // 5. 总览 block 时不能进入；进入后 block → 中止
        cur_img = 6'd2; block = 1; press_chord; repeat (50) @(posedge clk);
        if (tour_busy) fail("entered while block");
        block = 0;
        press_chord;
        wait_st(3'd5, 10);
        block = 1; repeat (3) @(posedge clk);
        if (tour_busy) fail("abort on block");
        block = 0;
        // 6. 重新规划 bank 1 时（加载开始）几何作废 → 不能进入
        @(negedge clk); fb_load_start = 1; fb_load_bank = 2'd1; @(negedge clk); fb_load_start = 0;
        repeat (50) @(posedge clk);
        press_chord; repeat (50) @(posedge clk);
        if (tour_busy) fail("entered with stale geometry");
        commit;

        if (win_bad != 0) begin $display("INFO bundle changed outside commit window %0d times", win_bad); fail("commit window"); end
        if (errors == 0) $display("PASS tb_tour_sequencer");
        else $display("FAIL tb_tour_sequencer errors=%0d", errors);
        $finish;
    end
    initial begin #400_000_000; $display("FAIL timeout"); $finish; end
endmodule
