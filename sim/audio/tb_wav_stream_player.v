`timescale 1ns/1ps
// -----------------------------------------------------------------------------
//   写侧 50MHz（SD 引擎）、读侧 25MHz（音频链），tb 内置扇区口行为模型（busy/req/done/error 协议同 sd_sector_adapter）
//   样本值 = hash(lba, k) | 1（永不为 0）→ 输出 0 即静音，非 0 必须严格按扇区顺序
//   覆盖：1 PCM 格式 + start_lba/total_sectors  2 play_enable 启动  3 小端拼字  4 握手  5 BRAM 写入/预填充
//         6 sample_tick 输出 + 指针推进  7 下溢静音  8 格雷码 CDC  9 低水位续读  10 排空 + 结束状态
//         以及：停止/换曲无残留、扇区错误结束、下游 ready 长时间拉低不丢样本
// -----------------------------------------------------------------------------
module tb_wav_stream_player;
    localparam integer AW    = 12;
    localparam integer DEPTH = 1 << AW;
    localparam integer TICK  = 40;          // tb 加速：每 40 个读时钟一个 sample_tick（真实 521）

    reg wr_clk = 0, rd_clk = 0;
    always #10 wr_clk = ~wr_clk;            // 50MHz
    always #20 rd_clk = ~rd_clk;            // 25MHz
    reg wr_rst_n = 0, rd_rst_n = 0;

    reg         play_enable = 0;
    reg  [31:0] start_lba = 0, total_sectors = 0;
    wire        sd_req;
    wire [31:0] sd_lba;
    reg         sd_busy = 0, sd_valid = 0, sd_done = 0;
    reg  [7:0]  sd_byte = 0, sd_error = 0;
    reg  [8:0]  sd_byte_index = 0;
    wire        wr_idle, wr_done, wr_error;
    reg         sample_tick = 0;
    wire signed [15:0] sample_out;
    wire        sample_valid;
    reg         sample_ready = 1;
    wire [31:0] sample_counter;
    wire        stream_active, stream_end, buffer_low;

    wav_stream_player #(.ADDR_WIDTH(AW)) dut (
        .wr_clk(wr_clk), .wr_rst_n(wr_rst_n), .play_enable(play_enable),
        .start_lba(start_lba), .total_sectors(total_sectors),
        .sd_req(sd_req), .sd_lba(sd_lba), .sd_busy(sd_busy), .sd_valid(sd_valid), .sd_byte(sd_byte),
        .sd_byte_index(sd_byte_index), .sd_done(sd_done), .sd_error(sd_error),
        .wr_idle(wr_idle), .wr_done(wr_done), .wr_error(wr_error),
        .rd_clk(rd_clk), .rd_rst_n(rd_rst_n), .sample_tick(sample_tick),
        .sample_out(sample_out), .sample_valid(sample_valid), .sample_ready(sample_ready),
        .sample_counter(sample_counter), .stream_active(stream_active), .stream_end(stream_end),
        .buffer_low(buffer_low)
    );

    integer errors = 0;
    task fail(input [8*72-1:0] msg); begin errors = errors + 1; $display("FAIL @%0t: %0s", $time, msg); end endtask

    // 样本值：48k/16bit/有符号/小端/单声道，PC 侧预处理后的裸 PCM
    function [15:0] pcm;
        input [31:0] lba; input [7:0] k;
        reg [31:0] h;
        begin
            h = (lba * 32'd256 + k) * 32'h9E3779B1;
            pcm = h[31:16] | 16'h0001;
        end
    endfunction

    // ---------------- 扇区口行为模型 ----------------
    integer lat_cycles = 20;           // req → 首字节
    integer byte_gap   = 1;            // 字节间隔（1 = 每 2 拍一个字节）
    reg [31:0] err_lba = 32'hFFFF_FFFF;
    integer n_req = 0;
    reg [31:0] req_log [0:255];
    reg [31:0] model_lba;
    integer i, g;

    always @(posedge wr_clk) begin
        if (sd_req && sd_busy) fail("sd_req while sd_busy");
    end
    initial begin
        forever begin
            @(posedge wr_clk);
            if (sd_req) begin
                model_lba = sd_lba;
                if (n_req < 256) req_log[n_req] = sd_lba;
                n_req = n_req + 1;
                #1 sd_busy = 1;
                repeat (lat_cycles) @(posedge wr_clk);
                for (i = 0; i < 512; i = i + 1) begin
                    if (sd_lba !== model_lba) fail("sd_lba changed while busy");
                    #1 sd_valid = 1; sd_byte_index = i;
                    sd_byte = i[0] ? pcm(model_lba, i >> 1) >> 8 : pcm(model_lba, i >> 1);   // 小端：低字节先到
                    @(posedge wr_clk); #1 sd_valid = 0;
                    for (g = 0; g < byte_gap; g = g + 1) @(posedge wr_clk);
                end
                #1 sd_done = 1; sd_error = (model_lba == err_lba) ? 8'h21 : 8'h00;
                @(posedge wr_clk); #1 sd_done = 0; sd_error = 0; sd_busy = 0;
            end
        end
    end

    // ---------------- sample_tick ----------------
    integer tick_cnt = 0, rd_cyc = 0;
    always @(posedge rd_clk) begin
        rd_cyc <= (rd_cyc == TICK-1) ? 0 : rd_cyc + 1;
        sample_tick <= (rd_cyc == TICK-1);
        if (sample_tick) tick_cnt = tick_cnt + 1;
    end

    // ---------------- 输出监视 ----------------
    reg         check_on = 0;
    reg [31:0]  exp_lba;
    integer     exp_k;
    integer     n_real = 0, n_silent = 0, n_take = 0, n_underflow = 0;
    integer     first_real_level = -1;
    reg         seen_real = 0;
    reg  signed [15:0] hold_s; reg hold_v = 0;
    always @(posedge rd_clk) begin
        // valid 保持期间数据不变（PCM 合同）
        if (hold_v && sample_valid && sample_out !== hold_s) fail("sample_out changed while valid && !ready");
        hold_v <= sample_valid && !sample_ready; hold_s <= sample_out;
        if (sample_valid && sample_ready) begin
            n_take = n_take + 1;
            if (check_on) begin
                if (sample_out == 16'sd0) begin
                    n_silent = n_silent + 1;
                    if (seen_real) n_underflow = n_underflow + 1;
                end else begin
                    if (!seen_real) first_real_level = dut.level_rd;
                    seen_real = 1;
                    if (sample_out !== pcm(exp_lba, exp_k)) begin
                        fail("sample mismatch");
                        $display("     lba=%0d k=%0d got %h exp %h", exp_lba, exp_k, sample_out, pcm(exp_lba, exp_k));
                    end
                    n_real = n_real + 1;
                    exp_k = exp_k + 1;
                    if (exp_k == 256) begin exp_k = 0; exp_lba = exp_lba + 1; end
                end
            end
        end
    end

    // ---------------- 结构断言：格雷码 CDC / 水位 / 流控 ----------------
    reg [AW:0] prev_wr_in_rd, prev_rd_in_wr, prev_wg2, prev_rg2;
    integer max_level_wr = 0;
    function integer popc; input [AW:0] v; integer j; begin popc = 0; for (j = 0; j <= AW; j = j + 1) popc = popc + v[j]; end endfunction
    always @(posedge rd_clk) if (rd_rst_n) begin
        if (popc(dut.wr_gray_sync2 ^ prev_wg2) > 1) fail("wr gray sync jumps > 1 bit");
        if (((dut.wr_ptr_in_rd - prev_wr_in_rd) & {(AW+1){1'b1}}) > 1) fail("wr_ptr_in_rd not monotonic");
        if (((dut.wr_ptr - dut.wr_ptr_in_rd) & {(AW+1){1'b1}}) > 8) fail("wr_ptr_in_rd ahead / lags too far");
        if (dut.level_rd > DEPTH) fail("level_rd > DEPTH");
        prev_wg2 <= dut.wr_gray_sync2; prev_wr_in_rd <= dut.wr_ptr_in_rd;
    end
    always @(posedge wr_clk) if (wr_rst_n) begin
        // 停止时读指针整体跳到写指针（多位变化），此时写侧在 IDLE/ABORT/END 不用 rd_ptr_in_wr；
        // 写侧只在 REQ/FILL 用它（space_ok），这段时间读指针只会 +1
        if (dut.wstate == 3'd1 || dut.wstate == 3'd2) begin
            if (popc(dut.rd_gray_sync2 ^ prev_rg2) > 1) fail("rd gray sync jumps > 1 bit in REQ/FILL");
            if (((dut.rd_ptr_in_wr - prev_rd_in_wr) & {(AW+1){1'b1}}) > 1) fail("rd_ptr_in_wr not monotonic in REQ/FILL");
        end
        // 新一曲开始时缓冲必须为空（读侧已就位）
        if (dut.wstate == 3'd0 && dut.play_enable && dut.armed_sync2 && start_lba != 0 && total_sectors != 0 && dut.rd_ptr_in_wr != dut.wr_ptr)
            fail("track start with non-empty buffer");
        if (dut.level_wr > DEPTH) fail("level_wr > DEPTH (overflow)");
        if (sd_req && dut.level_wr > DEPTH - 256) fail("sd_req with < 1 sector free");
        if (dut.level_wr > max_level_wr) max_level_wr = dut.level_wr;
        // BRAM 写入：写进去的就是小端拼好的样本
        if (dut.wstate == 3'd2 && sd_valid && dut.lo_pending && {sd_byte, dut.lo_byte} !== pcm(model_lba, sd_byte_index >> 1))
            fail("BRAM write data != little-endian sample");
        prev_rg2 <= dut.rd_gray_sync2; prev_rd_in_wr <= dut.rd_ptr_in_wr;
    end

    // 预填充：缓冲未到 1 扇区且写侧未结束时 can_play 不得成立
    always @(posedge rd_clk)
        if (dut.can_play && !dut.primed && dut.level_rd < 256 && !dut.ended_rd) fail("can_play before 1-sector prefill");

    // ---------------- 辅助 ----------------
    task start_track(input [31:0] lba, input [31:0] n);
        begin
            @(posedge wr_clk); #1;
            start_lba = lba; total_sectors = n;
            exp_lba = lba; exp_k = 0; seen_real = 0; n_real = 0; n_silent = 0; n_underflow = 0;
            first_real_level = -1; n_req = 0; max_level_wr = 0;
            check_on = 1; play_enable = 1;
        end
    endtask
    task stop_track;
        begin
            @(posedge wr_clk); #1 play_enable = 0;
            wait (!dut.en_rd); @(posedge rd_clk); check_on = 0;
            wait (wr_idle);
        end
    endtask
    task wait_done(input integer timeout_us);
        integer t;
        begin
            t = 0;
            while (!wr_done && !wr_error && t < timeout_us) begin #1000; t = t + 1; end
            if (t >= timeout_us) fail("timeout waiting wr_done");
        end
    endtask
    integer k, t0, takes0, ticks0;

    initial begin
        #200 wr_rst_n = 1; rd_rst_n = 1;
        repeat (20) @(posedge wr_clk);

        // ===== 2. play_enable 之前不读卡、不出真实样本；参数为 0 不启动 =====
        start_lba = 100; total_sectors = 0; play_enable = 1;
        #20000;
        if (n_req != 0) fail("started with total_sectors == 0");
        play_enable = 0; #2000;
        start_lba = 100; total_sectors = 40;
        #20000;
        if (n_req != 0) fail("sd_req before play_enable");
        if (!wr_idle) fail("wr_idle should be 1 when stopped");
        if (stream_active) fail("stream_active while stopped");
        if (sample_valid) fail("sample_valid while stopped");
        $display("INFO 2: no request before play_enable, wr_idle=%0d", wr_idle);

        // ===== 1/3/4/5/6/9/10. 40 扇区（10240 样本 > 4096 缓冲）完整播放 =====
        sample_ready = 1;
        start_track(100, 40);
        wait (n_req != 0);
        if (req_log[0] != 100) fail("first lba != start_lba");
        wait_done(60000);
        if (!wr_done) fail("wr_done not asserted after full play");
        if (wr_error) fail("unexpected wr_error");
        if (n_req != 40) begin fail("sector count != total_sectors"); $display("     n_req=%0d", n_req); end
        for (k = 0; k < 40 && k < n_req; k = k + 1) if (req_log[k] != 100 + k) fail("lba sequence wrong");
        if (n_real != 40*256) begin fail("played sample count != total_sectors*256"); $display("     n_real=%0d", n_real); end
        if (n_underflow != 0) fail("unexpected underflow with fast SD");
        if (first_real_level < 255) begin fail("first real sample before 1-sector prefill"); $display("     level=%0d", first_real_level); end
        if (max_level_wr < DEPTH - 256 - 8) begin fail("buffer never filled -> flow control untested"); $display("     max=%0d", max_level_wr); end
        if (sample_counter != 40*256) fail("sample_counter != played samples");
        if (!stream_end || stream_active) fail("stream_end/stream_active wrong after drain");
        $display("INFO 1-10: lba %0d..%0d, %0d sectors, %0d real samples, %0d prefill silence, prefill level %0d, max fill %0d/%0d",
                 req_log[0], req_log[n_req-1], n_req, n_real, n_silent, first_real_level, max_level_wr, DEPTH);
        // 10. 结束后不自动重播：停在 WS_END，继续出静音
        k = n_req; #50000;
        if (n_req != k) fail("re-requested after end");
        stop_track;
        if (sample_counter != 0) fail("sample_counter not cleared on stop");

        // ===== 6. 下游 ready 长时间拉低（> 3 个采样周期），额度保证不丢样本、平均速率 = tick =====
        start_track(300, 20);
        #600000;                                  // 播放中
        ticks0 = tick_cnt; takes0 = n_take;
        sample_ready = 0; #(TICK*40*5); sample_ready = 1;   // 5 个采样周期
        #(TICK*40*3);
        if ((tick_cnt - ticks0) - (n_take - takes0) > 1 || (n_take - takes0) > (tick_cnt - ticks0))
            begin fail("credit: takes != ticks after ready stall"); $display("     ticks=%0d takes=%0d", tick_cnt-ticks0, n_take-takes0); end
        // 随机 ready
        fork
            begin repeat (20000) begin @(posedge rd_clk); sample_ready <= ($random % 4) != 0; end sample_ready <= 1; end
        join
        wait_done(60000);
        if (n_real != 20*256) begin fail("random-ready: sample count"); $display("     n_real=%0d", n_real); end
        if (n_underflow != 0) fail("random-ready: underflow");
        $display("INFO 6: ready stall 5 periods -> ticks %0d takes %0d; random ready ok, %0d samples", tick_cnt-ticks0, n_take-takes0, n_real);
        stop_track;

        // ===== 7. 下溢：SD 慢于消费 → 中途静音、valid 不断，恢复后样本不跳 =====
        byte_gap = 20;                            // 512 字节 x 21 拍 ≈ 215us
        lat_cycles = 40000;                       // + 800us 延迟 ≈ 1ms/扇区 ≫ 256 样本 x 1.6us = 410us
        start_track(500, 8);
        wait_done(60000);
        if (n_real != 8*256) begin fail("underflow: sample count"); $display("     n_real=%0d", n_real); end
        if (n_underflow == 0) fail("underflow: no silence observed");
        if (!wr_done) fail("underflow: wr_done");
        $display("INFO 7: underflow silence %0d samples inside track, sequence intact (%0d real)", n_underflow, n_real);
        stop_track;
        byte_gap = 1; lat_cycles = 20;

        // ===== 10b. 尾部不足 1 扇区（不满 256 预填充）也要排空：单扇区曲目 =====
        start_track(700, 1);
        wait_done(20000);
        if (n_real != 256) begin fail("1-sector track not fully drained"); $display("     n_real=%0d", n_real); end
        stop_track;

        // ===== 停止/换曲：在途扇区中途停止 → ABORT，换曲后不播旧曲残留 =====
        lat_cycles = 200;
        start_track(1000, 30);
        wait (n_req == 6); #3000;                 // 第 6 个扇区在途
        if (dut.wstate != 3'd2) fail("expected WS_FILL mid sector");
        stop_track;
        if (dut.level_rd != 0) fail("buffer not empty after stop");
        start_track(2000, 3);                     // 新曲：从 2000 的第 0 个样本开始，期间任何 1000.. 样本都会比对失败
        wait_done(20000);
        if (n_real != 3*256) begin fail("restart: sample count"); $display("     n_real=%0d", n_real); end
        $display("INFO stop/restart: abort mid-sector, new track clean (%0d samples)", n_real);
        stop_track;

        // ===== 4. 扇区错误：wr_error，本曲结束，不再请求 =====
        err_lba = 3002;
        start_track(3000, 10);
        wait_done(20000);
        if (!wr_error) fail("wr_error not set on sector error");
        k = n_req; #20000;
        if (n_req != k || n_req != 3) begin fail("requests after error"); $display("     n_req=%0d", n_req); end
        $display("INFO 4: sector error at lba 3002 -> wr_error, %0d requests", n_req);
        stop_track;
        if (wr_error) ; // 下一曲启动时清除
        err_lba = 32'hFFFF_FFFF;
        start_track(4000, 1);
        #2000;
        if (wr_error) fail("wr_error not cleared on next track");
        wait_done(20000);
        stop_track;

        if (errors == 0) $display("PASS tb_wav_stream_player");
        else             $display("FAIL tb_wav_stream_player: %0d errors", errors);
        $finish;
    end
endmodule
