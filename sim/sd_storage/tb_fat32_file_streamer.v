`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// tb_fat32_file_streamer —— 文件流读取器单元测试（fat32_file_streamer + sd_sector_adapter + 扇区模型）
// 32x24 BMP = 2358 字节：spc=1 → 5 簇、spc=2 → 3 簇、spc=4 → 2 簇（最后一簇都不满）
//   F1 spc=1、交错倒序碎片化、簇号 > 65535、FAT 表项高 4 位 = 0xF（保留位必须屏蔽）→ 逐字节 = 文件内容、偏移连续、字节数 = 文件大小
//   F2 spc=2 交错；F3 spc=4 连续；每簇只查一次 FAT + 结尾查一次（读扇区数 = 数据扇区 + 簇数）
//   F4 第1簇 FAT=EOF → 0x25   F5 第1簇 FAT=0x0FFFFFF0 → 0x24   F6 自指 → 0x23
//   F7 最后一簇不是 EOF → 0x23   F8 FAT=1 → 0x24   F9 first_cluster=0 → 0x24（不读卡）
//   F10 目录项大小比链长 → 0x25   F11 MAX_VISITED_CLUSTERS=3 的实例读 5 簇文件 → 0x26
//   F12 扇区读卡死 → 重试后 0x10  F13 file_size=0 → 直接完成、无错误、不读卡
//   F14 单次读卡死 → 自动重试成功  F15 FAT 坏簇标记 → 0x27
//   F16 扇区读到第 300 字节断掉（连续 2 次）→ 重读时跳过已输出部分，字节流仍逐字节正确、偏移连续
// -----------------------------------------------------------------------------
module tb_fat32_file_streamer;
    localparam integer TMO = 20000;

    reg clk = 1'b0;
    reg rst_n = 1'b0;
    always #10 clk = ~clk;

    reg         file_start = 1'b0;
    reg  [31:0] first_cluster = 32'd0, file_size = 32'd0;
    reg  [7:0]  spc = 8'd1;
    reg  [31:0] fat_start = 32'd0, data_start = 32'd0, max_cluster = 32'd0;
    reg         use_small = 1'b0;

    wire        busy_a, done_a, valid_a;  wire [7:0] byte_a, err_a;  wire [31:0] off_a;
    wire        busy_b, done_b, valid_b;  wire [7:0] byte_b, err_b;  wire [31:0] off_b;
    wire        req_a, req_b;  wire [31:0] lba_a, lba_b;

    wire        sector_busy, sector_valid, sector_done;
    wire [7:0]  sector_byte, sector_error;
    wire [8:0]  sector_byte_index;
    wire        sd_sec_read, sd_sec_read_data_valid, sd_sec_read_end;
    wire [31:0] sd_sec_read_addr;
    wire [7:0]  sd_sec_read_data;

    fat32_file_streamer dut (
        .clk(clk), .rst_n(rst_n), .file_start(file_start && !use_small),
        .first_cluster(first_cluster), .file_size(file_size), .sectors_per_cluster(spc),
        .fat_start_lba(fat_start), .cluster_base_lba(data_start - 2 * spc), .max_cluster(max_cluster),
        .file_busy(busy_a), .file_done(done_a), .file_valid(valid_a), .file_byte(byte_a),
        .file_byte_offset(off_a), .fs_error(err_a),
        .sector_req(req_a), .sector_lba(lba_a), .sector_busy(use_small ? 1'b1 : sector_busy),
        .sector_valid(sector_valid && !use_small), .sector_byte(sector_byte),
        .sector_byte_index(sector_byte_index), .sector_done(sector_done && !use_small),
        .sector_error(sector_error)
    );

    fat32_file_streamer #(.MAX_VISITED_CLUSTERS(3)) dut_small (
        .clk(clk), .rst_n(rst_n), .file_start(file_start && use_small),
        .first_cluster(first_cluster), .file_size(file_size), .sectors_per_cluster(spc),
        .fat_start_lba(fat_start), .cluster_base_lba(data_start - 2 * spc), .max_cluster(max_cluster),
        .file_busy(busy_b), .file_done(done_b), .file_valid(valid_b), .file_byte(byte_b),
        .file_byte_offset(off_b), .fs_error(err_b),
        .sector_req(req_b), .sector_lba(lba_b), .sector_busy(use_small ? sector_busy : 1'b1),
        .sector_valid(sector_valid && use_small), .sector_byte(sector_byte),
        .sector_byte_index(sector_byte_index), .sector_done(sector_done && use_small),
        .sector_error(sector_error)
    );

    sd_sector_adapter #(.TIMEOUT_CYCLES(TMO)) u_adp (
        .clk(clk), .rst_n(rst_n),
        .sector_req(use_small ? req_b : req_a), .sector_lba(use_small ? lba_b : lba_a),
        .sector_busy(sector_busy), .sector_valid(sector_valid), .sector_byte(sector_byte),
        .sector_byte_index(sector_byte_index), .sector_done(sector_done), .sector_error(sector_error),
        .sd_sec_read(sd_sec_read), .sd_sec_read_addr(sd_sec_read_addr),
        .sd_sec_read_data_valid(sd_sec_read_data_valid), .sd_sec_read_data(sd_sec_read_data),
        .sd_sec_read_end(sd_sec_read_end)
    );

    sd_sec_fat32_model #(.W(32), .H(24), .LATENCY(10), .GAP(0)) u_card (
        .clk(clk), .rst_n(rst_n),
        .sd_sec_read(sd_sec_read), .sd_sec_read_addr(sd_sec_read_addr),
        .sd_sec_read_data(sd_sec_read_data), .sd_sec_read_data_valid(sd_sec_read_data_valid),
        .sd_sec_read_end(sd_sec_read_end)
    );

    wire        f_done  = use_small ? done_b  : done_a;
    wire        f_valid = use_small ? valid_b : valid_a;
    wire [7:0]  f_byte  = use_small ? byte_b  : byte_a;
    wire [31:0] f_off   = use_small ? off_b   : off_a;
    wire [7:0]  f_err   = use_small ? err_b   : err_a;
    wire        f_busy  = use_small ? busy_b  : busy_a;

    integer failures = 0;
    task fail(input [8*80-1:0] msg);
        begin failures = failures + 1; $display("FAIL @%0t: %0s", $time, msg); end
    endtask

    integer cur_img = 0, n_bytes = 0, n_bad = 0, n_off_bad = 0;
    always @(posedge clk) if (rst_n && f_valid) begin
        if (f_off != n_bytes) n_off_bad = n_off_bad + 1;
        if (f_byte !== u_card.bmp_byte(cur_img, f_off)) n_bad = n_bad + 1;
        n_bytes = n_bytes + 1;
    end

    // 按模型当前配置设置文件系统参数
    task setup_fs;
        begin
            spc = u_card.fi_spc; fat_start = u_card.fi_fat_start(0);
            data_start = u_card.fi_data_start(0); max_cluster = u_card.fi_data_clusters + 1;
        end
    endtask

    integer reads0, nreads;
    task stream(input integer img, input [31:0] clus, input [31:0] size);
        integer t;
        begin
            cur_img = img; n_bytes = 0; n_bad = 0; n_off_bad = 0;
            reads0 = u_card.n_reads;
            @(negedge clk);
            first_cluster = clus; file_size = size; file_start = 1'b1;
            @(negedge clk) file_start = 1'b0;
            t = 0;
            while (!f_done && t < 3_000_000) begin @(posedge clk); #1; t = t + 1; end
            if (!f_done) fail("file_done never came");
            nreads = u_card.n_reads - reads0;
            @(negedge clk);
            if (f_busy) fail("file_busy after done");
        end
    endtask

    task expect_ok(input integer img, input [8*8-1:0] tag, input integer exp_reads);
        begin
            stream(img, u_card.fi_clus(img, 0), u_card.fi_size(img));
            if (f_err != 8'h00 || n_bytes != u_card.fi_size(img) || n_bad != 0 || n_off_bad != 0 ||
                (exp_reads >= 0 && nreads != exp_reads)) begin
                failures = failures + 1;
                $display("FAIL %0s err=%h bytes=%0d/%0d bad=%0d off_bad=%0d reads=%0d/%0d", tag, f_err, n_bytes,
                         u_card.fi_size(img), n_bad, n_off_bad, nreads, exp_reads);
            end
        end
    endtask

    task expect_err(input integer img, input [7:0] err, input [8*8-1:0] tag);
        begin
            stream(img, u_card.fi_clus(img, 0), u_card.fi_size(img));
            if (f_err != err || n_bad != 0 || n_off_bad != 0) begin
                failures = failures + 1;
                $display("FAIL %0s err=%h (exp %h) bytes=%0d bad=%0d", tag, f_err, err, n_bytes, n_bad);
            end
        end
    endtask

    integer ns;
    initial begin
        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        repeat (5) @(posedge clk);

        // ---------------- F1 ----------------
        u_card.fi_mbr = 1; u_card.fi_part_lba = 32'd300000; u_card.fi_spc = 8'd1;
        u_card.fi_data_clusters = 32'd80000; u_card.fi_cbase = 32'd70000; u_card.fi_frag = 2'd2;
        u_card.fi_fat_hinib = 4'hF;
        setup_fs;
        ns = (u_card.FILE_LEN + 511) / 512;                         // 5 个数据扇区
        expect_ok(1, "F1", ns + u_card.fi_ncl(0));
        expect_ok(3, "F1b", ns + u_card.fi_ncl(0));
        $display("INFO F1 spc=1 clusters %0d..%0d (reverse interleaved), reads=%0d", u_card.fi_clus(1, 0),
                 u_card.fi_clus(1, u_card.fi_ncl(0) - 1), nreads);
        u_card.fi_fat_hinib = 4'h0;

        // ---------------- F2 / F3 ----------------
        u_card.fi_spc = 8'd2; u_card.fi_frag = 2'd1; u_card.fi_cbase = 32'd10; u_card.fi_data_clusters = 32'd2000;
        setup_fs;
        expect_ok(0, "F2", ns + u_card.fi_ncl(0));
        u_card.fi_spc = 8'd4; u_card.fi_frag = 2'd0; u_card.fi_mbr = 0;
        setup_fs;
        expect_ok(2, "F3", ns + u_card.fi_ncl(0));

        // ---------------- F4..F8 FAT 链错误 ----------------
        u_card.fi_spc = 8'd1; u_card.fi_frag = 2'd1; u_card.fi_mbr = 1;
        setup_fs;
        u_card.fi_fat_fault[0] = 3'd1; expect_err(0, 8'h25, "F4 eof");
        u_card.fi_fat_fault[0] = 3'd2; expect_err(0, 8'h24, "F5 rng");
        u_card.fi_fat_fault[0] = 3'd3; expect_err(0, 8'h23, "F6 self");
        u_card.fi_fat_fault[0] = 3'd4; expect_err(0, 8'h23, "F7 long");
        if (n_bytes != u_card.fi_size(0)) fail("F7 whole file should stream before final chain check");
        u_card.fi_fat_fault[0] = 3'd5; expect_err(0, 8'h24, "F8 one");
        u_card.fi_fat_fault[0] = 3'd0;

        // ---------------- F9 ----------------
        stream(0, 32'd0, u_card.fi_size(0));
        if (f_err != 8'h24 || nreads != 0 || n_bytes != 0) fail("F9 first_cluster=0 should give 0x24 without reads");
        stream(0, max_cluster + 1, u_card.fi_size(0));
        if (f_err != 8'h24 || nreads != 0) fail("F9 first_cluster>max should give 0x24");

        // ---------------- F10 ----------------
        u_card.fi_size_delta[2] = 600;
        expect_err(2, 8'h25, "F10 size");
        u_card.fi_size_delta[2] = 0;

        // ---------------- F11 ----------------
        use_small = 1'b1;
        expect_err(1, 8'h26, "F11 vis");
        use_small = 1'b0;

        // ---------------- F12 ----------------
        u_card.hang = 1'b1;
        expect_err(3, 8'h10, "F12 hang");
        u_card.hang = 1'b0;
        rst_n = 1'b0; repeat (3) @(posedge clk); rst_n = 1'b1; repeat (3) @(posedge clk);
        expect_ok(3, "F12 after", ns + u_card.fi_ncl(0));

        // ---------------- F13 ----------------
        stream(0, u_card.fi_clus(0, 0), 32'd0);
        if (f_err != 8'h00 || nreads != 0 || n_bytes != 0) fail("F13 empty file");

        // ---------------- F14 / F15 ----------------
        u_card.hang_reads = 1;
        expect_ok(1, "F14 retry", ns + u_card.fi_ncl(0) + 1);
        u_card.fi_fat_fault[0] = 3'd6; expect_err(0, 8'h27, "F15 bad");
        u_card.fi_fat_fault[0] = 3'd0;

        // ---------------- F16 扇区读到一半断掉：重读后跳过已输出的字节，不重复、不缺 ----------------
        u_card.cut_reads = 2; u_card.cut_at = 300;
        expect_ok(2, "F16 cut", ns + u_card.fi_ncl(0) + 2);

        if (failures == 0)
            $display("PASS tb_fat32_file_streamer: byte-exact streams (no sector buffer), retry without duplicate bytes, bad-cluster 0x27, FAT errors 0x23..0x26");
        else
            $display("FAIL tb_fat32_file_streamer failures=%0d", failures);
        $finish;
    end

    initial begin
        #500_000_000;
        $display("FAIL tb_fat32_file_streamer timeout");
        $finish;
    end
endmodule
