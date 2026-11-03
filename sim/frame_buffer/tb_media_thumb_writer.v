`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// tb_media_thumb_writer ——加载时 4:1 抽样写 2×2 拼图（模块级，写口 + 存储模型）
// 检查：
//   T1 4:1 最近邻映射：拼图 (slot_y·120+ly, slot_x·160+lx) = 源显示 (4·ly, 4·lx) = 流第 479-4·ly 行第 4·lx 列
//   T2 每个格子恰好写 19200 字，且只写自己 160×120 的区域（不越界、不写别的格子 / 右侧下方空白区）
//   T3 burst：每次连续 32 拍、起址 32 对齐、地址连续；只在 grant 之后开始（仲裁随机延迟放行）
//   T4 image_complete：第一个像素时清本格 valid，提交 + 写完才置 1；tag = 图片号 / 4（按页）
//   T5 中止（decode_error）：本格保持无效，进行中的 burst 照常写满 32 拍，FIFO 残余丢弃；随后重载同格正确
//   T6 clear 清全部 valid
// -----------------------------------------------------------------------------
module tb_media_thumb_writer;
    localparam integer BASE = 1228800;
    reg clk = 0, rst_n = 0;
    always #12.5 clk = ~clk;

    reg        clear = 0;
    reg  [5:0] image_idx = 0;
    reg        ld_start = 0, px_valid = 0, px_commit = 0;
    reg [23:0] px_rgb = 0;
    reg  [7:0] dec_err = 0;
    reg        grant_ok = 0;
    wire       want, busy, wr_en;
    wire [20:0] wr_addr; wire [3:0] wr_mask; wire [31:0] wr_data;
    wire [7:0] valid; wire [31:0] tags;   
    wire       grant = want && grant_ok;

    media_thumb_writer dut (.clk(clk), .rst_n(rst_n), .clear(clear), .image_idx(image_idx), .gen(1'b1),
        .ld_load_start(ld_start), .ld_pixel_valid(px_valid), .ld_pixel_rgb(px_rgb),
        .ld_frame_commit(px_commit), .ld_decode_error(dec_err),
        .controller_init_done(1'b1), .burst_want(want), .burst_grant(grant), .burst_busy(busy),
        .app_wr_en(wr_en), .app_wr_addr(wr_addr), .app_wr_mask(wr_mask), .app_wr_data(wr_data),
        .thumb_valid(valid), .thumb_tags(tags));

    function [23:0] pat(input [5:0] img, input integer s); pat = {img, 18'h0} ^ (s * 24'h9E37) ^ 24'h5A5A5A; endfunction

    // 存储模型：只建第 5 区（640×480）
    reg [23:0] mem [0:307199];
    integer    wcount [0:3];
    integer errors = 0;
    task fail(input [8*72-1:0] msg); begin errors = errors + 1; if (errors < 30) $display("FAIL %0s @%0t", msg, $time); end endtask

    // 仲裁：随机放行（有时长时间不放行，让 FIFO 积累）
    integer seed = 3, hold_off = 0;
    always @(posedge clk) begin
        if (hold_off > 0) begin hold_off <= hold_off - 1; grant_ok <= 0; end
        else begin
            grant_ok <= ($unsigned($random(seed)) % 4) == 0;
            if (($unsigned($random(seed)) % 4000) == 0) hold_off <= 3000;
        end
    end

    // 写口监视
    integer run = 0, bursts = 0, expect_slot = 0, off, row, colm, sl;
    reg wr_en_q = 0, granted = 0; reg [20:0] last_addr;
    always @(posedge clk) begin
        if (grant) granted <= 1;
        if (wr_en) begin
            if (!wr_en_q) begin
                if (!granted) fail("T3 burst without grant");
                granted <= 0;
                if (wr_addr[4:0] != 0) fail("T3 burst start not 32-aligned");
            end else if (wr_addr != last_addr + 1) fail("T3 address not contiguous inside burst");
            last_addr = wr_addr;
            run = run + 1;
            if (wr_addr < BASE || wr_addr >= BASE + 307200) fail("T2 write outside region 5");
            else begin
                off = wr_addr - BASE; row = off / 640; colm = off % 640;
                if (row >= 240 || colm >= 320) fail("T2 write in blank area");
                sl = (row >= 120 ? 2 : 0) + (colm >= 160 ? 1 : 0);
                if (sl != expect_slot) fail("T2 write into another slot");
                mem[off] = wr_data[23:0];
                wcount[sl] = wcount[sl] + 1;
            end
            if (wr_mask != 4'b0000) fail("mask");
        end else if (wr_en_q) begin
            bursts = bursts + 1;
            if (run != 32) begin fail("T3 burst length != 32"); $display("INFO burst len %0d", run); end
            run = 0;
        end
        wr_en_q <= wr_en;
    end

    task feed(input [5:0] img, input integer n_px, input integer abort_at);
        integer s;
        begin
            image_idx = img; expect_slot = img[1:0];
            @(posedge clk); #1 ld_start = 1; @(posedge clk); #1 ld_start = 0;
            repeat (50) @(posedge clk);
            wcount[img[1:0]] = 0;          // 上一次中止时在途的 burst 已在这 50 拍内写完
            for (s = 0; s < n_px; s = s + 1) begin
                repeat ($unsigned($random(seed)) % 12) @(posedge clk);
                if (s == abort_at) begin
                    #1 dec_err = 8'hAB; @(posedge clk); #1 dec_err = 0; s = n_px;
                end else begin
                    #1 px_valid = 1; px_rgb = pat(img, s); px_commit = (s == 307199);
                    @(posedge clk); #1 px_valid = 0; px_commit = 0;
                    if (s == 0) begin
                        @(posedge clk);
                        if (valid[img[1:0]]) fail("T4 valid not cleared at first pixel");
                    end
                end
            end
        end
    endtask

    task check_slot(input [5:0] img);
        integer lx, ly, sx, sy, bad; reg [23:0] e;
        begin
            sx = img[0]; sy = img[1]; bad = 0;
            for (ly = 0; ly < 120; ly = ly + 1)
                for (lx = 0; lx < 160; lx = lx + 1) begin
                    e = pat(img, (479 - 4 * ly) * 640 + 4 * lx);
                    if (mem[(sy * 120 + ly) * 640 + sx * 160 + lx] !== e) begin
                        bad = bad + 1;
                        if (bad < 4) $display("FAIL T1 img %0d slot (%0d,%0d) got %h exp %h", img, lx, ly, mem[(sy * 120 + ly) * 640 + sx * 160 + lx], e);
                    end
                end
            errors = errors + bad;
            if (wcount[img[1:0]] != 19200) begin fail("T2 word count != 19200"); $display("INFO count %0d", wcount[img[1:0]]); end
            if (!valid[img[1:0]]) fail("T4 valid not set");
            if (tags[img[1:0]*4 +: 4] != img[5:2]) fail("T4 tag wrong");
            $display("INFO image %0d -> slot %0d page %0d: %0d words, mapping errors %0d, valid=%b tags=%h",
                img, img[1:0], img[5:2], wcount[img[1:0]], bad, valid, tags);
        end
    endtask

    task wait_idle; begin
        repeat (200) @(posedge clk);
        while (busy || dut.cnt != 0 || dut.active) @(posedge clk);
        repeat (10) @(posedge clk);
    end endtask

    integer k, e0;
    initial begin
        for (k = 0; k < 4; k = k + 1) wcount[k] = 0;
        #100 rst_n = 1;
        // T1-T4：图片 0..3
        e0 = errors;
        for (k = 0; k < 4; k = k + 1) begin feed(k, 307200, -1); wait_idle; check_slot(k); end
        if (valid != 4'hF) fail("T4 all four valid");
        // 第 2 页：图片 5 → 格子 1，tag 1
        feed(6'd5, 307200, -1); wait_idle; check_slot(6'd5);
        $display("INFO T1-T4 errors %0d, bursts %0d", errors - e0, bursts);
        // T5：图片 2 加载中止 → 格子 2 无效；重载正确
        e0 = errors;
        feed(6'd2, 307200, 150000); wait_idle;
        if (valid[2]) fail("T5 valid set after abort");
        if (dut.cnt != 0) fail("T5 FIFO residue not flushed");
        $display("INFO T5 after abort: valid=%b, slot 2 words written before abort %0d (all in 32-word bursts)", valid, wcount[2]);
        feed(6'd2, 307200, -1); wait_idle; check_slot(6'd2);
        // 中止紧跟新加载（残余丢弃时 burst 还在进行）
        feed(6'd3, 307200, 2000);
        feed(6'd3, 307200, -1); wait_idle; check_slot(6'd3);
        $display("INFO T5 errors %0d", errors - e0);
        // T6
        e0 = errors;
        @(posedge clk); #1 clear = 1; @(posedge clk); #1 clear = 0; @(posedge clk);
        if (valid != 0) fail("T6 clear");
        $display("INFO T6 errors %0d", errors - e0);

        if (errors == 0) $display("PASS tb_media_thumb_writer");
        else $display("FAIL tb_media_thumb_writer errors=%0d", errors);
        $finish;
    end
    initial begin #2_000_000_000; $display("FAIL timeout"); $finish; end
endmodule
