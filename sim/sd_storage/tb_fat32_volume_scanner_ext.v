`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// tb_fat32_volume_scanner_ext —— 存储架构完善：递归子目录 / 64 项动态清单（资源版：只认 8.3 目录项，LFN 项跳过）
//   卡（sd_sec_fat32_ext_model，超级软盘 spc=4，数据区 LBA48）：
//     根目录 : LFN "LongRootPicture.BMP"（2 项 LFN + 8.3 名 LONGRO~1.BIN）、20 个 R*.BMP、子目录 SUBDIR(簇 10)
//     SUBDIR : . / .. / 子目录 NEST(簇 11) / 30 个 S*.BMP
//     NEST   : . / .. / 20 个 N*.BMP          共 71 个 BMP > 64 → 清单满
//   检查：布局参数、清单 64 项 + table_full、遍历顺序（根 → SUBDIR → NEST）、首簇/大小、
//         LFN 项本身和 8.3 扩展名不是 BMP 的 LONGRO~1.BIN 都不进清单、读扇区数
// -----------------------------------------------------------------------------
module tb_fat32_volume_scanner_ext;
    reg clk=0, rst_n=0, scan_start=0;
    always #10 clk=~clk;
    wire scan_busy,scan_done;
    wire [7:0] fs_error,sectors_per_cluster,bad_cluster_skip_count;
    wire [31:0] partition_lba,fat_start_lba,data_start_lba,max_cluster;
    wire [6:0] resource_count;
    wire resource_table_full,directory_limit_hit;
    wire cat_we;
    wire [7:0] cat_waddr, cat_raddr;
    wire [31:0] cat_wdata, cat_rdata;
    wire sector_req,sector_busy,sector_valid,sector_done;
    wire [31:0] sector_lba;
    wire [7:0] sector_byte,sector_error;
    wire [8:0] sector_byte_index;
    wire sd_sec_read,sd_sec_read_data_valid,sd_sec_read_end;
    wire [31:0] sd_sec_read_addr;
    wire [7:0] sd_sec_read_data;

    fat32_volume_scanner dut(
        .clk(clk),.rst_n(rst_n),.scan_start(scan_start),.scan_busy(scan_busy),.scan_done(scan_done),
        .fs_error(fs_error),.partition_lba(partition_lba),.sectors_per_cluster(sectors_per_cluster),
        .fat_start_lba(fat_start_lba),
        .data_start_lba(data_start_lba),.max_cluster(max_cluster),
        .resource_count(resource_count),.resource_table_full(resource_table_full),
        .directory_limit_hit(directory_limit_hit),.bad_cluster_skip_count(bad_cluster_skip_count),
        .cat_we(cat_we),.cat_waddr(cat_waddr),.cat_wdata(cat_wdata),.cat_raddr(cat_raddr),.cat_rdata(cat_rdata),
        .sector_req(sector_req),.sector_lba(sector_lba),.sector_busy(sector_busy),.sector_valid(sector_valid),
        .sector_byte(sector_byte),.sector_byte_index(sector_byte_index),.sector_done(sector_done),.sector_error(sector_error)
    );
    fat32_sdp_ram #(.AW(8),.DW(32)) cat(
        .clk(clk),.we(cat_we),.waddr(cat_waddr),.wdata(cat_wdata),.re(1'b1),.raddr(cat_raddr),.rdata(cat_rdata)
    );
    sd_sector_adapter #(.TIMEOUT_CYCLES(20000)) adapter(
        .clk(clk),.rst_n(rst_n),.sector_req(sector_req),.sector_lba(sector_lba),.sector_busy(sector_busy),
        .sector_valid(sector_valid),.sector_byte(sector_byte),.sector_byte_index(sector_byte_index),
        .sector_done(sector_done),.sector_error(sector_error),
        .sd_sec_read(sd_sec_read),
        .sd_sec_read_addr(sd_sec_read_addr),.sd_sec_read_data_valid(sd_sec_read_data_valid),
        .sd_sec_read_data(sd_sec_read_data),.sd_sec_read_end(sd_sec_read_end)
    );
    sd_sec_fat32_ext_model card(
        .clk(clk),.rst_n(rst_n),.sd_sec_read(sd_sec_read),.sd_sec_read_addr(sd_sec_read_addr),
        .sd_sec_read_data(sd_sec_read_data),.sd_sec_read_data_valid(sd_sec_read_data_valid),
        .sd_sec_read_end(sd_sec_read_end)
    );

    integer failures=0;
    task fail;
        input [8*100-1:0] message;
        begin failures=failures+1;$display("FAIL %0s",message);end
    endtask
    // 每项 2 个字：{项号,0}=首簇  {项号,1}=大小
    task check_entry;
        input [6:0] index;
        input [31:0] cluster;
        input [31:0] size;
        begin
            if(cat.mem[{index,1'b0}][27:0]!=cluster || cat.mem[{index,1'b1}]!=size) begin
                failures=failures+1;
                $display("FAIL index=%0d cluster=%0d/%0d size=%0d/%0d",index,
                         cat.mem[{index,1'b0}][27:0],cluster,cat.mem[{index,1'b1}],size);
            end
        end
    endtask

    integer timeout, k;
    initial begin
        repeat(5) @(posedge clk); rst_n=1; repeat(4) @(posedge clk);
        @(negedge clk) scan_start=1; @(negedge clk) scan_start=0;
        timeout=0;
        while(!scan_done && timeout<2000000) begin @(posedge clk);#1;timeout=timeout+1;end
        if(!scan_done) fail("scan timeout");
        if(fs_error!=0) begin failures=failures+1;$display("FAIL fs_error=%h",fs_error);end
        if(partition_lba!=0 || sectors_per_cluster!=4 || fat_start_lba!=32 || data_start_lba!=48 || max_cluster!=5001)
            fail("parsed FAT32 layout mismatch");
        // 70 个 *.BMP（根 20 + SUBDIR 30 + NEST 20）> 64 → 清单满
        if(resource_count!=64 || !resource_table_full || directory_limit_hit || bad_cluster_skip_count!=0)
            fail("64-entry catalog/diagnostics mismatch");
        // LFN 项（attr=0x0F）与 8.3 扩展名为 BIN 的 LONGRO~1.BIN（簇 100）都不能进清单
        for(k=0;k<64;k=k+1)
            if(cat.mem[2*k][27:0]==100) fail("non-BMP 8.3 entry / LFN entry leaked into catalog");
        check_entry(0,101,2000);          // 根目录 R00
        check_entry(19,120,2019);         // 根目录 R19
        check_entry(20,200,3000);         // SUBDIR S00（广度优先：根目录之后）
        check_entry(49,229,3029);         // SUBDIR S29
        check_entry(50,300,4000);         // NEST N00（SUBDIR 之后）
        check_entry(63,313,4013);         // NEST N13 = 第 64 项
        if(card.n_reads!=8) begin failures=failures+1;$display("FAIL sector reads=%0d expected=8",card.n_reads);end
        if(failures==0)
            $display("PASS tb_fat32_volume_scanner_ext: recursive subdirectories (BFS order), 8.3-only BMP matching (LFN entries skipped), 64-entry 1-BRAM catalog, table-full handling");
        else $display("FAIL tb_fat32_volume_scanner_ext failures=%0d",failures);
        $finish;
    end
    initial begin #200000000;$display("FAIL tb_fat32_volume_scanner_ext watchdog");$finish;end
endmodule
