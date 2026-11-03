`timescale 1ns/1ps
// -----------------------------------------------------------------------------
//   音画同步版：曲目由 select（当前图片绑定的曲目号）决定，停在同一张图单曲循环。
//   tb 内置定位器模型（idx → {1000*(idx+1), idx+2}）与播放器写侧模型（wr_idle / wr_done / wr_error 时序与
//   wav_stream_player 一致：play_enable 拉低后若干拍 wr_idle 才回 1）
//   [P1] 没有曲目 / enable=0：不请求、不播放，switching=0
//   [P2] 停在一张图：播完（wr_done）和出错（wr_error）都重播同一曲；参数 = 定位结果；只有 wr_idle 后才启动
//   [P3] 换图：播放中 select 变化 → 立即停、开播新图的曲目；16 首一一对应（select=15 → TRACK16）；
//        本图曲目缺失 / select 无曲目位 = 1（第 17 张起）→ 停曲、不顺延、switching=0（回到琶音）；
//        LOOKUP / START 期间连续换图 → 只开播最后一张图的曲目；空档 switching=1
//   [P4] enable 中途拉低 → 停止、switching=0；再拉高从本图曲目开头播
//   [P5] LOOP=0：播完停在 HOLD（switching=0），不重播；换图才播新曲
// -----------------------------------------------------------------------------
module tb_wav_playlist_ctrl;
    reg clk = 0, rst_n = 0;
    always #10 clk = ~clk;

    reg  [15:0] track_valid = 16'd0;
    reg  [4:0] select = 5'd0;
    wire       has_tb = !select[4] && track_valid[select[3:0]];
    reg        wr_done = 0, wr_error = 0;
    reg        wr_idle_r = 1;
    wire [3:0] info_idx0, info_idx1, track0, track1;
    wire       info_req0, info_req1, pe0, pe1, playing0, playing1, sw0, sw1;
    wire [31:0] st0, st1, tot0, tot1;
    reg        info_valid0 = 0, info_valid1 = 0;
    reg [31:0] info_st0 = 0, info_tot0 = 0, info_st1 = 0, info_tot1 = 0;

    // 被测：LOOP=1 与 LOOP=0 各一个，共享曲目表、select 和播放器模型（同一时刻只有一个被 enable）
    reg en0 = 0, en1 = 0;
    wav_playlist_ctrl #(.LOOP(1)) dut0 (
        .clk(clk), .rst_n(rst_n), .enable(en0), .track_valid(track_valid), .select(select),
        .info_req(info_req0), .info_idx(info_idx0), .info_valid(info_valid0),
        .info_start_lba(info_st0), .info_total_sectors(info_tot0),
        .play_enable(pe0), .start_lba(st0), .total_sectors(tot0),
        .wr_idle(wr_idle_r), .wr_done(wr_done && pe0), .wr_error(wr_error && pe0), .track(track0), .playing(playing0),
        .switching(sw0)
    );
    wav_playlist_ctrl #(.LOOP(0)) dut1 (
        .clk(clk), .rst_n(rst_n), .enable(en1), .track_valid(track_valid), .select(select),
        .info_req(info_req1), .info_idx(info_idx1), .info_valid(info_valid1),
        .info_start_lba(info_st1), .info_total_sectors(info_tot1),
        .play_enable(pe1), .start_lba(st1), .total_sectors(tot1),
        .wr_idle(wr_idle_r), .wr_done(wr_done && pe1), .wr_error(wr_error && pe1), .track(track1), .playing(playing1),
        .switching(sw1)
    );
    wire pe = pe0 | pe1;

    integer errors = 0;
    task fail(input [8*64-1:0] m); begin errors = errors + 1; $display("FAIL @%0t: %0s", $time, m); end endtask

    // locator model: info_valid 6 cycles after req (counter-based, plain Verilog)
    integer lc0 = 0, lc1 = 0;
    always @(posedge clk) begin
        info_valid0 <= (lc0 == 1);
        info_valid1 <= (lc1 == 1);
        if (info_req0) begin lc0 <= 6; info_st0 <= 1000 * (info_idx0 + 1); info_tot0 <= info_idx0 + 2; end
        else if (lc0 != 0) lc0 <= lc0 - 1;
        if (info_req1) begin lc1 <= 6; info_st1 <= 1000 * (info_idx1 + 1); info_tot1 <= info_idx1 + 2; end
        else if (lc1 != 0) lc1 <= lc1 - 1;
    end

    // player write-side model: play_enable rise -> wr_idle=0 after 3 cycles; fall -> wr_idle=1 after 6 cycles
    reg pe_q = 0;
    integer pc = 0;
    always @(posedge clk) begin
        pe_q <= pe;
        if (pe && !pe_q) begin
            if (!wr_idle_r) fail("[P2] play_enable while player not idle");
            pc <= 3;
        end else if (!pe && pe_q) pc <= -6;
        else if (pc > 1) pc <= pc - 1;
        else if (pc < -1) pc <= pc + 1;
        else if (pc == 1) begin wr_idle_r <= 1'b0; pc <= 0; end
        else if (pc == -1) begin wr_idle_r <= 1'b1; pc <= 0; end
    end

    // record start track and parameters
    integer order [0:63];
    integer n_start = 0;
    always @(posedge clk) begin
        if (pe && !pe_q) begin
            if (n_start < 64) order[n_start] = pe0 ? track0 : track1;
            if ((pe0 && (st0 != 1000 * (track0 + 1) || tot0 != track0 + 2)) ||
                (pe1 && (st1 != 1000 * (track1 + 1) || tot1 != track1 + 2))) fail("[P2] start_lba/total_sectors != locator result");
            n_start = n_start + 1;
        end
    end

    // switching 必须覆盖 dut0 停曲 → 再开播的整个空档（en0 期间、有曲目时）
    integer gap_cyc = 0, gap_max = 0;
    always @(posedge clk) begin
        if (en0 && has_tb && rst_n && !playing0) begin
            if (!sw0) fail("[P3] gap without switching");
            gap_cyc = gap_cyc + 1;
            if (gap_cyc > gap_max) gap_max = gap_cyc;
        end else gap_cyc = 0;
        if (!en0 && sw0) fail("[P4] switching while disabled");
        if (!en1 && sw1) fail("[P5] switching while disabled");
        if (en0 && !has_tb && sw0) fail("[P3] switching on image without track");
    end

    task finish_track(input err);
        begin
            wait (pe); repeat (20) @(negedge clk);
            if (err) wr_error = 1; else wr_done = 1;
            wait (!pe); @(negedge clk); wr_done = 0; wr_error = 0;
        end
    endtask

    // 播放中换图：等开播后再换 select，返回时新曲已开播
    task change_to(input [4:0] s);
        integer n0;
        begin
            wait (pe); repeat (20) @(negedge clk);
            n0 = n_start; select = s;
            wait (n_start == n0 + 1); @(negedge clk);
        end
    endtask

    // 播放中换到没有曲目的图：停曲后不再开播
    task change_to_none(input [4:0] s);
        integer n0;
        begin
            wait (pe); repeat (20) @(negedge clk);
            n0 = n_start; select = s;
            wait (!pe); repeat (300) @(negedge clk);
            if (pe || sw0 || n_start != n0) begin fail("[P3] image without track started something"); $display("     select=%0d", s); end
        end
    endtask

    integer k;
    initial begin
        repeat (3) @(negedge clk); rst_n = 1;
        // [P1]
        en0 = 1; repeat (100) @(negedge clk);
        if (info_req0 || pe || n_start != 0 || sw0) fail("[P1] activity with no tracks");
        en0 = 0; track_valid = 16'b1000_0000_0000_1011;   // TRACK1/2/4/16 repeat (100) @(negedge clk);
        if (pe || n_start != 0) fail("[P1] activity with enable=0");
        $display("INFO [P1] idle without tracks / without enable");

        // [P2] select=0（第 1 张图）：播完、出错、播完 → 始终 TRACK1
        en0 = 1;
        for (k = 0; k < 3; k = k + 1) finish_track(k == 1);
        wait (pe); repeat (2) @(negedge clk);
        if (n_start != 4 || order[0] != 0 || order[1] != 0 || order[2] != 0 || order[3] != 0)
            begin fail("[P2] same image did not loop its own track"); $display("     n=%0d %0d %0d %0d %0d", n_start, order[0],order[1],order[2],order[3]); end
        $display("INFO [P2] stay on image 0: TRACK%0d x%0d (done / error both replay same track)", order[0]+1, n_start);

        // [P3] 换图：1 → TRACK2，15 → TRACK16，2（TRACK3 缺失）→ 不播，3 → TRACK4，无曲目位（第 17 张起）→ 不播，0 → TRACK1
        k = n_start;
        change_to(5'd1); change_to(5'd15); change_to_none(5'd2);
        select = 5'd3; wait (n_start == k + 3); @(negedge clk);
        change_to_none(5'h10);
        select = 5'd0; wait (n_start == k + 4); @(negedge clk);
        if (order[k] != 1 || order[k+1] != 15 || order[k+2] != 3 || order[k+3] != 0)
            begin fail("[P3] image->track mapping"); $display("     %0d %0d %0d %0d", order[k],order[k+1],order[k+2],order[k+3]); end
        // 播完后仍是本图曲目
        finish_track(0); wait (pe); repeat (2) @(negedge clk);
        if (order[k+4] != 0) fail("[P3] loop after switch not same track");
        // LOOKUP 期间连续换图：1 → 2 → 1，最终只播 TRACK2
        repeat (20) @(negedge clk);
        k = n_start; select = 5'd1;
        wait (info_req0); @(negedge clk); select = 5'd15;
        repeat (3) @(negedge clk); select = 5'd1;
        wait (n_start == k + 1); repeat (200) @(negedge clk);
        if (n_start != k + 1 || order[k] != 1) begin fail("[P3] rapid switch"); $display("     n=%0d TRACK%0d", n_start - k, order[k]+1); end
        $display("INFO [P3] image 1/15/3/0 -> TRACK%0d %0d %0d %0d (2, 16 -> none); rapid 1-15-1 -> TRACK%0d only; max gap %0d cycles (switching=1)",
                 order[k-5]+1, order[k-4]+1, order[k-3]+1, order[k-2]+1, order[k]+1, gap_max);

        // [P4] 中途关 → 停；再开 → 本图曲目从头播（当前 select=1 → TRACK2）
        repeat (30) @(negedge clk); en0 = 0;
        wait (!pe); repeat (50) @(negedge clk);
        if (pe || sw0) fail("[P4] still playing/switching after disable");
        k = n_start; en0 = 1; wait (pe); repeat (2) @(negedge clk);
        if (order[k] != 1) begin fail("[P4] resume not image track"); $display("     TRACK%0d", order[k]+1); end
        en0 = 0; wait (!pe); wait (wr_idle_r); repeat (10) @(negedge clk);
        $display("INFO [P4] disable mid-track stops, enable restarts TRACK%0d", order[k]+1);

        // [P5] LOOP=0：播完停住不重播；换图才播新曲
        track_valid = 16'h8001; select = 5'd15; k = n_start;   // TRACK16
        en1 = 1;
        finish_track(0);
        repeat (300) @(negedge clk);
        if (pe || sw1 || n_start != k + 1 || order[k] != 15) fail("[P5] LOOP=0 did not hold after track");
        select = 5'd0;
        wait (pe); repeat (2) @(negedge clk);
        if (order[k+1] != 0) fail("[P5] switch from HOLD not TRACK1");
        $display("INFO [P5] LOOP=0: TRACK%0d then hold; image change plays TRACK%0d", order[k]+1, order[k+1]+1);

        if (errors == 0) $display("PASS tb_wav_playlist_ctrl");
        else             $display("FAIL tb_wav_playlist_ctrl: %0d errors", errors);
        $finish;
    end
    initial begin #5_000_000; $display("FAIL tb_wav_playlist_ctrl: timeout"); $finish; end
endmodule
