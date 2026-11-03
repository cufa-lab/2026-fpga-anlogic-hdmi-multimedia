`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// tb_sd_sector_adapter ——  SD 卡接口适配层单元测试
// sd_sector_adapter + sd_sec_fat32_model（官方 sd_card_top 扇区读接口的周期级模型）
//   A1 单扇区读：busy 包住整个读、恰好 512 个 valid、byte_index 0..511 连续、字节内容正确、单拍 done、error=0
//   A2 背靠背连读 3 个扇区（done 下一拍就发下一个 req）：每次都对、官方侧不多读（sd_sec_read 在 end 后撤销）
//   A3 busy 期间的 req 被忽略（地址不变、不多读）
//   A4 卡死（等令牌无出口）：TIMEOUT_CYCLES 后单拍 done + error=0x01，sd_sec_read 撤销；复位后恢复正常
//   A5 迟到数据：超时后卡才吐数据/end → 适配层不透传 valid/done；下一次读正常
// -----------------------------------------------------------------------------
module tb_sd_sector_adapter;
    localparam integer TMO = 4000;

    reg clk = 1'b0;
    reg rst_n = 1'b0;
    always #10 clk = ~clk;                     // 50 MHz

    reg         sector_req = 1'b0;
    reg  [31:0] sector_lba = 32'd0;
    wire        sector_busy, sector_valid, sector_done;
    wire [7:0]  sector_byte, sector_error;
    wire [8:0]  sector_byte_index;
    wire        sd_sec_read, sd_sec_read_data_valid, sd_sec_read_end;
    wire [31:0] sd_sec_read_addr;
    wire [7:0]  sd_sec_read_data;

    sd_sector_adapter #(.TIMEOUT_CYCLES(TMO)) dut (
        .clk(clk), .rst_n(rst_n),
        .sector_req(sector_req), .sector_lba(sector_lba), .sector_busy(sector_busy),
        .sector_valid(sector_valid), .sector_byte(sector_byte), .sector_byte_index(sector_byte_index),
        .sector_done(sector_done), .sector_error(sector_error),
        .sd_sec_read(sd_sec_read), .sd_sec_read_addr(sd_sec_read_addr),
        .sd_sec_read_data_valid(sd_sec_read_data_valid), .sd_sec_read_data(sd_sec_read_data),
        .sd_sec_read_end(sd_sec_read_end)
    );

    sd_sec_fat32_model #(.W(32), .H(24), .LATENCY(30), .GAP(2)) u_card (
        .clk(clk), .rst_n(rst_n),
        .sd_sec_read(sd_sec_read), .sd_sec_read_addr(sd_sec_read_addr),
        .sd_sec_read_data(sd_sec_read_data), .sd_sec_read_data_valid(sd_sec_read_data_valid),
        .sd_sec_read_end(sd_sec_read_end)
    );

    integer failures = 0;
    task fail(input [8*80-1:0] msg);
        begin failures = failures + 1; $display("FAIL @%0t: %0s", $time, msg); end
    endtask

    // 监视：valid / done 计数、索引连续性、内容
    integer n_valid = 0, n_done = 0, n_bad_idx = 0, n_bad_byte = 0, n_valid_idle = 0, n_done_idle = 0;
    integer exp_idx = 0;
    reg [31:0] cur_lba = 32'd0;
    reg [7:0]  last_err = 8'd0;
    always @(posedge clk) if (rst_n) begin
        if (sector_valid) begin
            n_valid = n_valid + 1;
            if (!sector_busy) n_valid_idle = n_valid_idle + 1;
            if (sector_byte_index != exp_idx[8:0]) n_bad_idx = n_bad_idx + 1;
            if (sector_byte !== u_card.card_byte(cur_lba, sector_byte_index)) n_bad_byte = n_bad_byte + 1;
            exp_idx = exp_idx + 1;
        end
        if (sector_done) begin
            n_done = n_done + 1;
            last_err = sector_error;
            if (!sector_busy && sector_error == 8'd0) n_done_idle = n_done_idle + 1;
        end
    end

    task clear_counts;
        begin
            n_valid = 0; n_done = 0; n_bad_idx = 0; n_bad_byte = 0; exp_idx = 0;
            n_valid_idle = 0; n_done_idle = 0;
        end
    endtask

    // 发一次读并等 done；返回周期数
    integer cyc;
    task read_sector(input [31:0] lba);
        begin
            @(negedge clk);
            sector_lba = lba; sector_req = 1'b1; cur_lba = lba; exp_idx = 0;
            @(negedge clk);
            sector_req = 1'b0;
            if (!sector_busy) fail("busy not asserted after req");
            cyc = 0;
            while (!sector_done && cyc < 200000) begin @(posedge clk); #1; cyc = cyc + 1; end
            // done 所在拍 busy 仍为 1，下一拍撤销（再过一个上升沿，监视器也已计入本次 done）
            @(posedge clk); @(negedge clk);
            if (sector_busy) fail("busy still high after done");
        end
    endtask

    integer reads0, k;
    initial begin
        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        repeat (5) @(posedge clk);

        // ---------------- A1 ----------------
        clear_counts;
        read_sector(32'd8192 + 32);             // FAT 首扇区（非零内容）
        if (n_valid != 512 || n_done != 1 || n_bad_idx != 0 || n_bad_byte != 0 || last_err != 0) begin
            failures = failures + 1;
            $display("FAIL A1 valid=%0d done=%0d bad_idx=%0d bad_byte=%0d err=%h", n_valid, n_done, n_bad_idx, n_bad_byte, last_err);
        end
        if (u_card.n_reads != 1) fail("A1 model saw != 1 read");
        if (sd_sec_read) fail("A1 sd_sec_read not released");

        // ---------------- A2 背靠背 ----------------
        clear_counts;
        reads0 = u_card.n_reads;
        for (k = 0; k < 3; k = k + 1) begin
            n_valid = 0; n_bad_idx = 0;
            @(negedge clk);
            sector_lba = 32'd0 + k; sector_req = 1'b1; cur_lba = k; exp_idx = 0;
            @(negedge clk) sector_req = 1'b0;
            while (!sector_done) @(posedge clk);
            if (n_valid != 512 || n_bad_idx != 0) begin
                failures = failures + 1; $display("FAIL A2 read %0d valid=%0d bad_idx=%0d", k, n_valid, n_bad_idx);
            end
        end
        repeat (200) @(posedge clk);
        if (u_card.n_reads - reads0 != 3) begin
            failures = failures + 1; $display("FAIL A2 model saw %0d reads (expect 3)", u_card.n_reads - reads0);
        end
        if (n_bad_byte != 0) fail("A2 data mismatch");

        // ---------------- A3 busy 期间的 req 被忽略 ----------------
        clear_counts;
        reads0 = u_card.n_reads;
        @(negedge clk);
        sector_lba = 32'd8192; sector_req = 1'b1; cur_lba = 32'd8192;
        @(negedge clk) sector_req = 1'b0;
        repeat (100) @(posedge clk);
        @(negedge clk);
        sector_lba = 32'd5; sector_req = 1'b1;        // busy 中再请求
        @(negedge clk) sector_req = 1'b0;
        sector_lba = 32'd0;
        if (sd_sec_read_addr != 32'd8192) fail("A3 address changed during busy");
        while (!sector_done) @(posedge clk);
        repeat (200) @(posedge clk);
        if (u_card.n_reads - reads0 != 1 || n_bad_byte != 0 || n_valid != 512) fail("A3 request during busy was not ignored");

        // ---------------- A4 卡死 → 超时 ----------------
        clear_counts;
        u_card.hang = 1'b1;
        @(negedge clk);
        sector_lba = 32'd1; sector_req = 1'b1; cur_lba = 32'd1;
        @(negedge clk) sector_req = 1'b0;
        cyc = 0;
        while (!sector_done && cyc < 3 * TMO) begin @(posedge clk); #1; cyc = cyc + 1; end
        if (!sector_done || sector_error != 8'h01) fail("A4 no timeout done/error");
        if (cyc < TMO - 10 || cyc > TMO + 10) begin
            failures = failures + 1; $display("FAIL A4 timeout after %0d cycles (expect ~%0d)", cyc, TMO);
        end
        @(posedge clk); @(negedge clk);
        if (sector_busy || sd_sec_read) fail("A4 busy/sd_sec_read not released after timeout");
        if (n_done != 1) fail("A4 done count");
        // 上层软复位 sd_card_top + 适配层
        u_card.hang = 1'b0;
        rst_n = 1'b0; repeat (3) @(posedge clk); rst_n = 1'b1; repeat (3) @(posedge clk);
        clear_counts;
        read_sector(32'd8192 + 33);
        if (n_valid != 512 || n_bad_byte != 0 || last_err != 0) fail("A4 read after reset failed");

        // ---------------- A5 超时后迟到的数据不透传 ----------------
        clear_counts;
        u_card.extra_latency = TMO + 500;
        @(negedge clk);
        sector_lba = 32'd2; sector_req = 1'b1; cur_lba = 32'd2;
        @(negedge clk) sector_req = 1'b0;
        while (!sector_done) begin @(posedge clk); #1; end
        if (sector_error != 8'h01) fail("A5 expected timeout");
        @(posedge clk);
        n_valid = 0; n_done = 0;
        repeat (TMO + 3000) @(posedge clk);         // 模型此时把整扇区吐完 + end
        if (n_valid != 0 || n_done != 0) begin
            failures = failures + 1; $display("FAIL A5 late data leaked valid=%0d done=%0d", n_valid, n_done);
        end
        u_card.extra_latency = 0;
        clear_counts;
        read_sector(32'd8192 + 32);
        if (n_valid != 512 || n_bad_byte != 0 || n_bad_idx != 0 || last_err != 0) fail("A5 read after late data failed");

        if (failures == 0)
            $display("PASS tb_sd_sector_adapter: busy/index/done/512 bytes, back-to-back, busy-ignore, timeout 0x01, late data isolated");
        else
            $display("FAIL tb_sd_sector_adapter failures=%0d", failures);
        $finish;
    end

    initial begin
        #50_000_000;
        $display("FAIL tb_sd_sector_adapter timeout");
        $finish;
    end
endmodule
