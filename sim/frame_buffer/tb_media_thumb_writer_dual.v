`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// tb_media_thumb_writer_dual —— 全屏 2×2 + 双缓冲缩略图区（SUB_LOG2=1, DUAL=1）+ gen 跳过
// 检查：
//   D1 2:1 最近邻（隔一个取一个）：区 img[2] 的 (slot_y·240+ly, slot_x·320+lx) = 源显示 (2·ly, 2·lx)
//      = 流第 479-2·ly 行第 2·lx 列；每格恰好 76800 字
//   D2 只写本格：图 0~3 → 第一块区（BASE），图 4~7 → 第二块区（BASE+307200）；写第二块不碰第一块
//      （写完图 4 后图 0 的格子逐字不变）；burst 32 拍、32 对齐、连续、grant 之后
//   D3 条目 {img[2], img[1:0]} 的 valid / tag（tag = img[5:2]）；第 2 页图 8 回到第一块区格子 0，tag 2
//   D4 gen=0：整帧送完也不写一个字，valid / tag 不变（缩略图完好时格子不闪占位）
//   D5 中止：本条目无效、burst 写满；clear 清全部 8 个条目
// -----------------------------------------------------------------------------
module tb_media_thumb_writer_dual;
    localparam integer BASE = 1228800;
    localparam integer AREA = 307200;
    reg clk = 0, rst_n = 0;
    always #12.5 clk = ~clk;

    reg        clear = 0;
    reg  [5:0] image_idx = 0;
    reg        gen = 1;
    reg        ld_start = 0, px_valid = 0, px_commit = 0;
    reg [23:0] px_rgb = 0;
    reg  [7:0] dec_err = 0;
    reg        grant_ok = 0;
    wire       want, busy, wr_en;
    wire [20:0] wr_addr; wire [3:0] wr_mask; wire [31:0] wr_data;
    wire [7:0] valid; wire [31:0] tags;
    wire       grant = want && grant_ok;

    media_thumb_writer #(.SUB_LOG2(1), .DUAL(1), .AREA_WORDS(AREA)) dut (
        .clk(clk), .rst_n(rst_n), .clear(clear), .image_idx(image_idx), .gen(gen),
        .ld_load_start(ld_start), .ld_pixel_valid(px_valid), .ld_pixel_rgb(px_rgb),
        .ld_frame_commit(px_commit), .ld_decode_error(dec_err),
        .controller_init_done(1'b1), .burst_want(want), .burst_grant(grant), .burst_busy(busy),
        .app_wr_en(wr_en), .app_wr_addr(wr_addr), .app_wr_mask(wr_mask), .app_wr_data(wr_data),
        .thumb_valid(valid), .thumb_tags(tags));

    function [23:0] pat(input [5:0] img, input integer s); pat = {img, 18'h0} ^ (s * 24'h9E37) ^ 24'h5A5A5A; endfunction

    reg [23:0] mem [0:2*AREA-1];
    integer    wcount [0:7];
    integer    n_words = 0;
    integer errors = 0;
    task fail(input [8*72-1:0] msg); begin errors = errors + 1; if (errors < 30) $display("FAIL %0s @%0t", msg, $time); end endtask

    integer seed = 7, hold_off = 0;
    always @(posedge clk) begin
        if (hold_off > 0) begin hold_off <= hold_off - 1; grant_ok <= 0; end
        else begin
            grant_ok <= ($unsigned($random(seed)) % 4) == 0;
            if (($unsigned($random(seed)) % 4000) == 0) hold_off <= 1000;   // 2:1 抽样一行 320 个样本，板上 SD 速率下 FIFO 余量约 1 ms
        end
    end

    integer run = 0, bursts = 0, expect_ent = 0, off, row, colm, ent;
    reg wr_en_q = 0, granted = 0; reg [20:0] last_addr;
    always @(posedge clk) begin
        if (grant) granted <= 1;
        if (wr_en) begin
            n_words = n_words + 1;
            if (!wr_en_q) begin
                if (!granted) fail("D2 burst without grant");
                granted <= 0;
                if (wr_addr[4:0] != 0) fail("D2 burst start not 32-aligned");
            end else if (wr_addr != last_addr + 1) fail("D2 address not contiguous inside burst");
            last_addr = wr_addr;
            run = run + 1;
            if (wr_addr < BASE || wr_addr >= BASE + 2 * AREA) fail("D2 write outside thumbnail areas");
            else begin
                off = wr_addr - BASE;
                ent = (off >= AREA ? 4 : 0);
                row = (off % AREA) / 640; colm = (off % AREA) % 640;
                ent = ent + (row >= 240 ? 2 : 0) + (colm >= 320 ? 1 : 0);
                if (ent != expect_ent) begin fail("D2 write into another cell / area"); $display("  ent %0d exp %0d", ent, expect_ent); end
                mem[off] = wr_data[23:0];
                wcount[ent] = wcount[ent] + 1;
            end
        end else if (wr_en_q) begin
            bursts = bursts + 1;
            if (run != 32) begin fail("D2 burst length != 32"); $display("INFO burst len %0d", run); end
            run = 0;
        end
        wr_en_q <= wr_en;
    end

    task feed(input [5:0] img, input integer abort_at);
        integer s;
        begin
            image_idx = img; expect_ent = {img[2], img[1:0]};
            @(posedge clk); #1 ld_start = 1; @(posedge clk); #1 ld_start = 0;
            repeat (50) @(posedge clk);
            wcount[expect_ent] = 0;
            for (s = 0; s < 307200; s = s + 1) begin
                repeat ($unsigned($random(seed)) % 12) @(posedge clk);
                if (s == abort_at) begin
                    #1 dec_err = 8'hAB; @(posedge clk); #1 dec_err = 0; s = 307200;
                end else begin
                    #1 px_valid = 1; px_rgb = pat(img, s); px_commit = (s == 307199);
                    @(posedge clk); #1 px_valid = 0; px_commit = 0;
                end
            end
        end
    endtask

    // 返回不一致字数
    function integer cell_bad(input [5:0] img);
        integer lx, ly, base_off, b; reg [23:0] e;
        begin
            b = 0;
            base_off = (img[2] ? AREA : 0) + (img[1] ? 240 * 640 : 0) + (img[0] ? 320 : 0);
            for (ly = 0; ly < 240; ly = ly + 1)
                for (lx = 0; lx < 320; lx = lx + 1) begin
                    e = pat(img, (479 - 2 * ly) * 640 + 2 * lx);
                    if (mem[base_off + ly * 640 + lx] !== e) b = b + 1;
                end
            cell_bad = b;
        end
    endfunction

    task check_cell(input [5:0] img);
        integer bad, e;
        begin
            e = {img[2], img[1:0]};
            bad = cell_bad(img);
            if (bad != 0) begin fail("D1 2:1 mapping"); $display("  img %0d bad words %0d", img, bad); end
            if (wcount[e] != 76800) begin fail("D1 word count != 76800"); $display("  count %0d", wcount[e]); end
            if (!valid[e]) fail("D3 valid not set");
            if (tags[e*4 +: 4] != img[5:2]) fail("D3 tag wrong");
            $display("INFO image %0d -> entry %0d page %0d: %0d words, bad %0d, valid=%b tags=%h", img, e, img[5:2], wcount[e], bad, valid, tags);
        end
    endtask

    task wait_idle; begin
        repeat (200) @(posedge clk);
        while (busy || dut.cnt != 0 || dut.active) @(posedge clk);
        repeat (10) @(posedge clk);
    end endtask

    integer k, w0;
    reg [7:0]  v0; reg [31:0] t0;
    initial begin
        for (k = 0; k < 8; k = k + 1) wcount[k] = 0;
        #100 rst_n = 1;
        // D1 / D3：图 0（第一块区格子 0）、图 5（第二块区格子 1）、图 6、图 3
        feed(6'd0, -1); wait_idle; check_cell(6'd0);
        feed(6'd5, -1); wait_idle; check_cell(6'd5);
        feed(6'd6, -1); wait_idle; check_cell(6'd6);
        feed(6'd3, -1); wait_idle; check_cell(6'd3);
        // D2：写第二块区（图 4）后第一块区的图 0 不变
        feed(6'd4, -1); wait_idle; check_cell(6'd4);
        if (cell_bad(6'd0) != 0) fail("D2 area B write disturbed area A");
        if (valid !== 8'b0111_1001) begin fail("D3 valid map"); $display("  valid=%b", valid); end
        // D3：第 2 页图 8 → 第一块区格子 0，tag 2，覆盖图 0
        feed(6'd8, -1); wait_idle; check_cell(6'd8);
        if (cell_bad(6'd4) != 0) fail("D2 page 2 write disturbed area B");
        // D4：gen=0 不写
        v0 = valid; t0 = tags; w0 = n_words;
        gen = 0;
        feed(6'd5, -1); wait_idle;
        if (n_words != w0) begin fail("D4 gen=0 must not write"); $display("  words %0d", n_words - w0); end
        if (valid !== v0 || tags !== t0) fail("D4 gen=0 keeps valid / tags");
        if (cell_bad(6'd5) != 0) fail("D4 cell 5 intact");
        gen = 1;
        // D5：中止 → 本条目无效；之后重载正确
        feed(6'd7, 100000); wait_idle;
        if (valid[7]) fail("D5 valid after abort");
        if (dut.cnt != 0) fail("D5 FIFO residue not flushed");
        feed(6'd7, -1); wait_idle; check_cell(6'd7);
        @(posedge clk); #1 clear = 1; @(posedge clk); #1 clear = 0; @(posedge clk);
        if (valid != 0) fail("D5 clear all 8");

        if (errors == 0) $display("PASS tb_media_thumb_writer_dual (bursts=%0d)", bursts);
        else $display("FAIL tb_media_thumb_writer_dual errors=%0d", errors);
        $finish;
    end
    initial begin #2_000_000_000; $display("FAIL timeout cnt=%0d written=%0d", dut.cnt, dut.written); $finish; end
endmodule
