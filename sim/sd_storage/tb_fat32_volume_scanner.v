`timescale 1ns/1ps

module tb_fat32_volume_scanner;
    localparam integer TMO = 20000;

    reg clk = 1'b0;
    reg rst_n = 1'b0;
    always #10 clk = ~clk;

    reg         scan_start = 1'b0;
    wire        scan_busy, scan_done;
    wire [7:0]  fs_error, spc;
    wire [31:0] partition_lba, fat_start_lba, data_start_lba, max_cluster;
    wire [6:0]  resource_count;
    wire        resource_table_full, directory_limit_hit;
    wire [7:0]  bad_cluster_skip_count;
    wire [3:0]  resource_valid = (resource_count >= 7'd4) ? 4'hF : ((4'b1 << resource_count) - 4'b1);
    wire        cat_we;
    wire [8:0]  cat_waddr, cat_raddr;
    wire [31:0] cat_wdata, cat_rdata;

    wire        sector_req, sector_busy, sector_valid, sector_done;
    wire [31:0] sector_lba;
    wire [7:0]  sector_byte, sector_error;
    wire [8:0]  sector_byte_index;
    wire        sd_sec_read, sd_sec_read_data_valid, sd_sec_read_end;
    wire [31:0] sd_sec_read_addr;
    wire [7:0]  sd_sec_read_data;

    fat32_volume_scanner dut (
        .clk(clk), .rst_n(rst_n), .scan_start(scan_start), .scan_busy(scan_busy), .scan_done(scan_done),
        .fs_error(fs_error), .partition_lba(partition_lba), .sectors_per_cluster(spc),
        .fat_start_lba(fat_start_lba), .data_start_lba(data_start_lba), .max_cluster(max_cluster),
        .resource_count(resource_count), .resource_table_full(resource_table_full),
        .directory_limit_hit(directory_limit_hit), .bad_cluster_skip_count(bad_cluster_skip_count),
        .cat_we(cat_we), .cat_waddr(cat_waddr), .cat_wdata(cat_wdata), .cat_raddr(cat_raddr), .cat_rdata(cat_rdata),
        .sector_req(sector_req), .sector_lba(sector_lba), .sector_busy(sector_busy),
        .sector_valid(sector_valid), .sector_byte(sector_byte), .sector_byte_index(sector_byte_index),
        .sector_done(sector_done), .sector_error(sector_error)
    );

    fat32_sdp_ram #(.AW(9), .DW(32)) cat (
        .clk(clk), .we(cat_we), .waddr(cat_waddr), .wdata(cat_wdata), .re(1'b1), .raddr(cat_raddr), .rdata(cat_rdata)
    );

    sd_sector_adapter #(.TIMEOUT_CYCLES(TMO)) u_adp (
        .clk(clk), .rst_n(rst_n),
        .sector_req(sector_req), .sector_lba(sector_lba), .sector_busy(sector_busy),
        .sector_valid(sector_valid), .sector_byte(sector_byte), .sector_byte_index(sector_byte_index),
        .sector_done(sector_done), .sector_error(sector_error),
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

    integer failures = 0;
    task fail(input [8*80-1:0] msg);
        begin failures = failures + 1; $display("FAIL @%0t: %0s", $time, msg); end
    endtask

    integer reads0, nreads;
    task run_scan;
        integer t;
        begin
            reads0 = u_card.n_reads;
            @(negedge clk) scan_start = 1'b1;
            @(negedge clk) scan_start = 1'b0;
            t = 0;
            while (!scan_done && t < 2_000_000) begin @(posedge clk); #1; t = t + 1; end
            if (!scan_done) fail("scan_done never came");
            nreads = u_card.n_reads - reads0;
            @(negedge clk);
            if (scan_busy) fail("scan_busy after done");
        end
    endtask

    function [31:0] rc_of(input integer k);
        begin
            rc_of = {4'd0, cat.mem[2*k][27:0]};    // 每项 2 个字：{项号,0}=首簇 {项号,1}=大小
        end
    endfunction
    function [31:0] rs_of(input integer k);
        begin
            rs_of = cat.mem[2*k+1];
        end
    endfunction

    // 检查布局参数 + 已找到文件的簇/大小
    task check_layout(input [8*8-1:0] tag);
        integer k;
        begin
            if (spc != u_card.fi_spc || fat_start_lba != u_card.fi_fat_start(0) ||
                data_start_lba != u_card.fi_data_start(0) || max_cluster != u_card.fi_data_clusters + 1 ||
                partition_lba != u_card.fi_boot_lba(0)) begin
                failures = failures + 1;
                $display("FAIL %0s layout spc=%0d fat=%0d/%0d data=%0d/%0d max=%0d/%0d part=%0d", tag,
                         spc, fat_start_lba, u_card.fi_fat_start(0), data_start_lba, u_card.fi_data_start(0),
                         max_cluster, u_card.fi_data_clusters + 1, partition_lba);
            end
            for (k = 0; k < 4; k = k + 1)
                if (resource_valid[k] && (rc_of(k) != u_card.fi_clus(k, 0) || rs_of(k) != u_card.fi_size(k))) begin
                    failures = failures + 1;
                    $display("FAIL %0s PIC%0d cluster %0d (exp %0d) size %0d (exp %0d)", tag, k + 1,
                             rc_of(k), u_card.fi_clus(k, 0), rs_of(k), u_card.fi_size(k));
                end
        end
    endtask

    task set_extra(input integer i, input [8:0] slot, input [87:0] name, input [31:0] cl);
        begin
            u_card.fi_extra_valid[i] = 1'b1; u_card.fi_extra_slot[i] = slot; u_card.fi_extra_name[i] = name;
            u_card.fi_extra_cluster[i] = cl; u_card.fi_extra_size[i] = cl * 10;
        end
    endtask

    task expect_result(input [7:0] err, input [3:0] valid, input [8*8-1:0] tag);
        begin
            if (fs_error != err || resource_valid != valid) begin
                failures = failures + 1;
                $display("FAIL %0s fs_error=%h (exp %h) valid=%b (exp %b)", tag, fs_error, err, resource_valid, valid);
            end
        end
    endtask

    initial begin
        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        repeat (5) @(posedge clk);

        // ---------------- S1 ----------------
        u_card.fi_mbr = 1; u_card.fi_part_lba = 32'd200000; u_card.fi_spc = 8'd1;
        u_card.fi_data_clusters = 32'd90000; u_card.fi_cbase = 32'd70000; u_card.fi_frag = 2'd1;
        u_card.fi_slot[0] = 9'd13; u_card.fi_slot[1] = 9'd2; u_card.fi_slot[2] = 9'd9; u_card.fi_slot[3] = 9'd4;
        u_card.fi_end_slot = 9'd16;
        run_scan;
        expect_result(8'h00, 4'hF, "S1");
        check_layout("S1");
        if (resource_count != 4) fail("S1 resource_count != 4");
        if (nreads != 4) begin failures = failures + 1; $display("FAIL S1 reads=%0d (exp 4: MBR, boot, root, root FAT)", nreads); end
        if (u_card.addr_log[reads0 + 1] != 32'd200000) fail("S1 boot sector not read at partition LBA");
        $display("INFO S1 MBR part=200000 cluster0=%0d fat=%0d data=%0d max=%0d", rc_of(0), fat_start_lba, data_start_lba, max_cluster);

        // ---------------- S2 ----------------
        u_card.fi_mbr = 0; u_card.fi_spc = 8'd8; u_card.fi_data_clusters = 32'd3000;
        u_card.fi_cbase = 32'd0; u_card.fi_frag = 2'd0;
        u_card.fi_slot[0] = 9'd5; u_card.fi_slot[1] = 9'd40; u_card.fi_slot[2] = 9'd17; u_card.fi_slot[3] = 9'd100;
        u_card.fi_end_slot = 9'd120; u_card.fi_ghost_slot = 9'd125;
        run_scan;
        expect_result(8'h00, 4'hF, "S2");
        check_layout("S2");
        if (nreads != 1 + 8) begin failures = failures + 1; $display("FAIL S2 reads=%0d (exp 9: boot@0 + root sectors 0..7 through end marker)", nreads); end

        // ---------------- S3 ----------------
        u_card.fi_mbr = 1; u_card.fi_part_lba = 32'd2048; u_card.fi_spc = 8'd4;
        u_card.fi_present = 4'b1011;
        u_card.fi_slot[0] = 9'd3; u_card.fi_slot[1] = 9'd18; u_card.fi_slot[3] = 9'd19;
        u_card.fi_end_slot = 9'd20; u_card.fi_ghost_slot = 9'd25;
        run_scan;
        expect_result(8'h00, 4'b0111, "S3");
        if (resource_count != 3 ||
            rc_of(0) != u_card.fi_clus(0, 0) || rc_of(1) != u_card.fi_clus(1, 0) || rc_of(2) != u_card.fi_clus(3, 0))
            fail("S3 compact PIC1/PIC2/PIC4 order wrong");
        if (nreads != 2 + 2) begin failures = failures + 1; $display("FAIL S3 reads=%0d (exp 4: MBR, boot, root 0..1)", nreads); end

        // ---------------- S4 ----------------
        u_card.fi_present = 4'hF; u_card.fi_ghost_slot = 9'h1FF; u_card.fi_end_slot = 9'd64;
        u_card.fi_slot[2] = 9'd30;
        u_card.fi_bad_sig0 = 1'b1;
        run_scan;
        expect_result(8'h20, 4'h0, "S4 MBR");
        u_card.fi_mbr = 0;
        run_scan;
        expect_result(8'h20, 4'h0, "S4 VBR");
        u_card.fi_bad_sig0 = 1'b0;

        // ---------------- S5 ----------------
        u_card.fi_bad_bps = 1'b1;
        run_scan;
        expect_result(8'h21, 4'h0, "S5 bps");
        u_card.fi_bad_bps = 1'b0;
        u_card.fi_mbr = 1; u_card.fi_part_type = 8'h07;
        run_scan;
        expect_result(8'h21, 4'h0, "S5 type");
        u_card.fi_part_type = 8'h0B;
        run_scan;
        expect_result(8'h00, 4'hF, "S5 0x0B");
        check_layout("S5 0x0B");

        // ---------------- S6 ----------------
        u_card.fi_present = 4'b1000; u_card.fi_spc = 8'd2; u_card.fi_end_slot = 9'd32;
        u_card.fi_slot[3] = 9'd31;
        run_scan;
        expect_result(8'h00, 4'b0001, "S6");
        if (resource_count != 1 || rc_of(0) != u_card.fi_clus(3, 0) || rs_of(0) != u_card.fi_size(3))
            fail("S6 PIC4 was not compacted to slot0");

        // ---------------- S7 ----------------
        u_card.fi_present = 4'hF;
        u_card.hang = 1'b1;
        run_scan;
        expect_result(8'h10, 4'h0, "S7");
        u_card.hang = 1'b0;
        rst_n = 1'b0; repeat (3) @(posedge clk); rst_n = 1'b1; repeat (3) @(posedge clk);
        u_card.fi_slot[0] = 9'd0; u_card.fi_slot[1] = 9'd1; u_card.fi_slot[2] = 9'd2; u_card.fi_slot[3] = 9'd3;
        run_scan;
        expect_result(8'h00, 4'hF, "S7 after");
        check_layout("S7 after");

        // ---------------- S8 根目录跨簇（FAT 链）----------------
        u_card.fi_mbr = 1; u_card.fi_part_lba = 32'd8192; u_card.fi_spc = 8'd2;
        u_card.fi_data_clusters = 32'd4000; u_card.fi_cbase = 32'd0; u_card.fi_frag = 2'd0;
        u_card.fi_present = 4'hF;
        u_card.fi_slot[0] = 9'd3;  u_card.fi_slot[1] = 9'd10;      // 首簇（槽 0~31）
        u_card.fi_slot[2] = 9'd33; u_card.fi_slot[3] = 9'd34;      // 第二簇（槽 32~63）
        u_card.fi_end_slot = 9'd36;                                // 结束标记也在第二簇
        u_card.fi_ghost_slot = 9'h1FF;
        u_card.fi_root2 = 32'd20;                                  // 根目录链到簇 20
        run_scan;
        expect_result(8'h00, 4'hF, "S8");
        check_layout("S8");
        if (nreads < 5) begin
            failures = failures + 1;
            $display("FAIL S8 reads=%0d (expect >=5: MBR+boot+root cluster2 sectors+FAT lookup+root cluster20 sector)", nreads);
        end
        u_card.fi_root2 = 32'd0;

        // ---------------- S9 动态文件列表：非 PIC 命名 + 小写扩展名 ----------------
        u_card.fi_mbr = 1; u_card.fi_part_lba = 32'd8192; u_card.fi_spc = 8'd1;
        u_card.fi_data_clusters = 32'd4000; u_card.fi_cbase = 32'd0; u_card.fi_frag = 2'd0;
        u_card.fi_present = 4'h0;                                  // 没有任何 PIC1~4
        u_card.fi_end_slot = 9'd12;
        u_card.fi_ghost_slot = 9'h1FF;
        u_card.fi_extra_valid[0] = 1'b1; u_card.fi_extra_slot[0] = 9'd2;
        u_card.fi_extra_name[0]  = "PHOTO   BMP"; u_card.fi_extra_cluster[0] = 32'd100; u_card.fi_extra_size[0] = 32'd5000;
        u_card.fi_extra_valid[1] = 1'b1; u_card.fi_extra_slot[1] = 9'd5;
        u_card.fi_extra_name[1]  = "lower   bmp"; u_card.fi_extra_cluster[1] = 32'd101; u_card.fi_extra_size[1] = 32'd6000;
        u_card.fi_extra_valid[2] = 1'b1; u_card.fi_extra_slot[2] = 9'd9;
        u_card.fi_extra_name[2]  = "ZEBRA   BMP"; u_card.fi_extra_cluster[2] = 32'd102; u_card.fi_extra_size[2] = 32'd7000;
        run_scan;
        expect_result(8'h00, 4'b0111, "S9");
        if (resource_count != 3) fail("S9 resource_count != 3");
        if (rc_of(0) != 32'd100 || rs_of(0) != 32'd5000) begin
            failures = failures + 1; $display("FAIL S9 slot0 cluster=%0d size=%0d", rc_of(0), rs_of(0));
        end
        if (rc_of(1) != 32'd101 || rs_of(1) != 32'd6000) begin
            failures = failures + 1; $display("FAIL S9 slot1 (lowercase .bmp) cluster=%0d size=%0d", rc_of(1), rs_of(1));
        end
        if (rc_of(2) != 32'd102 || rs_of(2) != 32'd7000) begin
            failures = failures + 1; $display("FAIL S9 slot2 cluster=%0d size=%0d", rc_of(2), rs_of(2));
        end
        if (nreads != 3) begin
            failures = failures + 1; $display("FAIL S9 reads=%0d (expect 3: MBR+boot+root)", nreads);
        end
        u_card.fi_extra_valid[0] = 1'b0; u_card.fi_extra_valid[1] = 1'b0; u_card.fi_extra_valid[2] = 1'b0;

        // ---------------- S10 [PIC 编号排序] ----------------
        begin : s10
            integer q;
            reg [31:0] exp_cl [0:7];
            set_extra(0, 9'd1,  "PIC16   BMP", 32'd116);
            set_extra(1, 9'd2,  "ZEBRA   BMP", 32'd200);
            set_extra(2, 9'd3,  "PIC13   BMP", 32'd113);
            set_extra(3, 9'd4,  "PIC65   BMP", 32'd165);
            set_extra(4, 9'd6,  "PIC64   BMP", 32'd164);
            set_extra(5, 9'd7,  "PIC01   BMP", 32'd201);
            set_extra(6, 9'd8,  "pic5    bmp", 32'd105);
            set_extra(7, 9'd10, "PIC10   BMP", 32'd110);
            exp_cl[0] = 105; exp_cl[1] = 110; exp_cl[2] = 113; exp_cl[3] = 116;
            exp_cl[4] = 164; exp_cl[5] = 200; exp_cl[6] = 165; exp_cl[7] = 201;
            run_scan;
            if (fs_error != 8'h00 || resource_count != 8) begin
                failures = failures + 1; $display("FAIL S10 fs_error=%h resource_count=%0d (exp 8)", fs_error, resource_count);
            end
            for (q = 0; q < 8; q = q + 1)
                if (rc_of(q) != exp_cl[q] || rs_of(q) != exp_cl[q] * 10) begin
                    failures = failures + 1;
                    $display("FAIL S10 slot%0d cluster=%0d size=%0d (exp %0d)", q, rc_of(q), rs_of(q), exp_cl[q]);
                end
            for (q = 0; q < 8; q = q + 1) u_card.fi_extra_valid[q] = 1'b0;
        end

        if (failures == 0)
            $display("PASS tb_fat32_volume_scanner: MBR/superfloppy, complete directory scan, compact dynamic BMP list, PIC1..64 numeric order, 1-BRAM catalog {cluster,size}, root FAT chain, end marker and error handling");
        else
            $display("FAIL tb_fat32_volume_scanner failures=%0d", failures);
        $finish;
    end

    initial begin
        #500_000_000;
        $display("FAIL tb_fat32_volume_scanner timeout");
        $finish;
    end
endmodule
