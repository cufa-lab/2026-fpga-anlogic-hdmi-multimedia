`timescale 1ns/1ps

module tb_sd_card_bmp_fat32;
    localparam W = 32, H = 24;

    reg clk = 1'b0;
    reg rst = 1'b1;
    reg ingest_enable = 1'b1;
    reg load_request = 1'b0;
    reg [5:0] load_image_idx = 6'd0;
    reg [1:0] load_buffer_idx = 2'd0;
    wire scan_done, scan_all_done;
    wire [6:0] image_count;
    wire loader_ready;
    wire load_done;
    wire [5:0] loaded_image_idx;
    wire [1:0] loaded_buffer_idx;
    wire [7:0] load_error;
    wire [3:0] bmp_state_code;
    wire [7:0] sd_reinit_count;
    wire [7:0] fs_error;
    wire fb_load_start, fb_pixel_valid, fb_frame_commit, fb_load_abort, fb_done, fb_busy;
    wire [1:0] fb_load_bank;
    wire [23:0] fb_pixel_rgb;
    wire [7:0] fb_error;
    wire SD_nCS, SD_DCLK, SD_MOSI, SD_MISO;

    always #5 clk = ~clk;                       // 100 MHz

    sd_card_bmp #(
        .CLK_FREQ_HZ        (100_000_000),
        .BMP_WIDTH          (W),
        .BMP_HEIGHT         (H),
        .SD_READ_TIMEOUT_MS (16'd1),            // 扇区 ≈0.2ms
        .LOAD_TIMEOUT_MS    (20),               // 含一次 SD 重新初始化(≈5ms)
        .MAX_RETRIES        (2'd1),
        .SD_REINIT_RETRY_MS (8)                 // 仿真里缩短：初始化≈3ms
    ) dut (
        .clk(clk), .rst_n(~rst), .ingest_enable(ingest_enable), .sd_ready(),
        .scan_done(scan_done), .scan_all_done(scan_all_done), .image_count(image_count),
        .loader_ready(loader_ready), .load_request(load_request),
        .load_image_idx(load_image_idx), .load_buffer_idx(load_buffer_idx),
        .load_done(load_done), .loaded_image_idx(loaded_image_idx),
        .loaded_buffer_idx(loaded_buffer_idx), .load_error(load_error),
        .bmp_state_code(bmp_state_code), .sd_reinit_count(sd_reinit_count), .fs_error(fs_error),
        .fb_load_start(fb_load_start), .fb_load_bank(fb_load_bank), .pixel_valid(fb_pixel_valid),
        .pixel_rgb(fb_pixel_rgb), .frame_commit(fb_frame_commit), .fb_load_abort(fb_load_abort),
        .fb_done(fb_done), .fb_error(fb_error), .fb_busy(fb_busy),
        .SD_nCS(SD_nCS), .SD_DCLK(SD_DCLK), .SD_MOSI(SD_MOSI), .SD_MISO(SD_MISO)
    );

    sd_card_fat32_model #(.W(W), .H(H)) u_sd (
        .nCS(SD_nCS), .DCLK(SD_DCLK), .MOSI(SD_MOSI), .MISO(SD_MISO)
    );

    // ---------------- 真实 load CDC + loader，SDRAM 写口接行为存储器 ----------------
    reg mem_clk = 1'b0;
    always #12.5 mem_clk = ~mem_clk;            // 40 MHz，与 sys 异步
    wire ld_start, ld_pv, ld_commit, ld_busy, ld_done;
    wire [1:0] ld_index;
    wire [23:0] ld_rgb;
    wire [7:0] ld_dec_err, ld_mem_err;
    wire [3:0] ld_complete, complete_sys;
    wire app_wr_en; wire [20:0] app_wr_addr; wire [3:0] app_wr_mask; wire [31:0] app_wr_data;
    wire ld_want, ld_bbusy;

    media_frame_load_cdc u_cdc (
        .src_clk(clk), .src_rst_n(~rst),
        .load_start(fb_load_start), .load_bank(fb_load_bank), .pixel_valid(fb_pixel_valid),
        .pixel_rgb(fb_pixel_rgb), .frame_commit(fb_frame_commit), .load_abort(fb_load_abort),
        .done_event(fb_done), .done_error(fb_error), .image_complete_sys(complete_sys), .stream_overflow(),
        .dst_clk(mem_clk), .dst_rst_n(~rst),
        .ld_load_start(ld_start), .ld_image_index(ld_index), .ld_pixel_valid(ld_pv), .ld_pixel_rgb(ld_rgb),
        .ld_frame_commit(ld_commit), .ld_decode_error(ld_dec_err),
        .ld_load_done(ld_done), .ld_memory_error(ld_mem_err), .ld_image_complete(ld_complete)
    );
    reg grant_en = 1'b1;                     // L8：关掉仲裁放行 → loader 写停顿超时 0x46
    media_frame_loader #(.IMAGE_WORDS(W * H), .FIFO_DEPTH(512), .BURST_SIZE(128), .STALL_TIMEOUT(4000),
        .WRITE_V_FLIP(1), .FRAME_WIDTH(W), .FRAME_HEIGHT(H)) u_loader (
        .clk(mem_clk), .rst_n(~rst), .load_start(ld_start), .image_index(ld_index),
        .pixel_valid(ld_pv), .pixel_rgb(ld_rgb), .frame_commit(ld_commit), .decode_error(ld_dec_err),
        .load_busy(ld_busy), .load_done(ld_done), .image_complete(ld_complete), .memory_error(ld_mem_err),
        .controller_init_done(1'b1), .controller_busy(1'b0), .refresh_active(1'b0),
        .burst_want(ld_want), .burst_grant(grant_en), .burst_busy(ld_bbusy),
        .app_wr_en(app_wr_en), .app_wr_addr(app_wr_addr), .app_wr_mask(app_wr_mask), .app_wr_data(app_wr_data)
    );
    reg [31:0] sdram_mem [0:4*W*H-1];
    always @(posedge mem_clk) if (app_wr_en) begin
        if (app_wr_addr >= 4 * W * H || app_wr_mask != 4'b0000) $display("FAIL @%0t: bad SDRAM write addr %0d mask %b", $time, app_wr_addr, app_wr_mask);
        else sdram_mem[app_wr_addr] <= app_wr_data;
    end

    // ---------------- 像素流比对 + 握手协议检查（sys 域）----------------
    integer cur_img = 0;                        // 当前加载对应的卡模型图片号（PIC 编号 - 1）
    integer n_pix = 0, pix_bad = 0, n_frames = 0, n_wr_total = 0;
    integer n_start = 0, n_done = 0, n_abort = 0, n_done_err = 0;
    reg     frame_open = 1'b0, outstanding = 1'b0;
    reg [23:0] exp_rgb;

    always @(posedge clk) begin
        if (fb_pixel_valid) n_wr_total <= n_wr_total + 1;
        if (fb_load_start) begin
            if (outstanding) $display("FAIL @%0t: fb_load_start while previous load not acknowledged", $time);
            n_start = n_start + 1; outstanding = 1'b1;
            n_pix      <= 0;                    // 新的加载（含重试）从头写
            frame_open <= 1'b1;
        end else if (fb_pixel_valid && frame_open) begin
            exp_rgb = u_sd.pix_rgb(cur_img, n_pix % W, n_pix / W);
            if (fb_pixel_rgb !== exp_rgb) pix_bad <= pix_bad + 1;
            if (fb_frame_commit !== (n_pix == W * H - 1)) begin
                pix_bad <= pix_bad + 1; $display("FAIL @%0t: frame_commit at pixel %0d", $time, n_pix);
            end
            if (n_pix == W * H - 1) begin
                frame_open <= 1'b0;
                n_frames   <= n_frames + 1;
            end
            n_pix <= n_pix + 1;
        end
        if (fb_load_abort) begin
            if (!outstanding) $display("FAIL @%0t: fb_load_abort without load in flight", $time);
            n_abort = n_abort + 1; frame_open <= 1'b0;
        end
        if (fb_done) begin
            if (!outstanding) $display("FAIL @%0t: fb_done without load in flight", $time);
            n_done = n_done + 1; outstanding = 1'b0;
            if (fb_error != 8'h00) n_done_err = n_done_err + 1;
        end
    end

    // 加载成功后核对 bank 内容：流像素 n → SDRAM bank*W*H + (H-1-n/W)*W + n%W
    function integer check_bank(input integer bank, input integer img);
        integer n, e; begin
            e = 0;
            for (n = 0; n < W * H; n = n + 1)
                if (sdram_mem[bank * W * H + (H - 1 - n / W) * W + n % W] !== {8'h00, u_sd.pix_rgb(img, n % W, n / W)}) e = e + 1;
            check_bank = e;
        end endfunction

    // ---------------- 观测 ----------------
    integer failures = 0;
    integer n_load_err_events = 0;
    reg [7:0] last_load_error = 8'h00;
    always @(posedge clk) if (!rst && load_error != 8'h00) begin
        n_load_err_events = n_load_err_events + 1;
        last_load_error   = load_error;
    end

    task fail(input [8*72-1:0] msg);
        begin failures = failures + 1; $display("FAIL @%0t: %0s", $time, msg); end
    endtask

    task wait_cycles(input integer n);
        begin repeat (n) @(posedge clk); end
    endtask

    // 卡模型恢复默认内容
    task card_default;
        integer q;
        begin
            u_sd.fi_mbr = 1; u_sd.fi_part_lba = 32'd200000; u_sd.fi_part_type = 8'h0C;
            u_sd.fi_spc = 8'd1; u_sd.fi_data_clusters = 32'd90000; u_sd.fi_cbase = 32'd70000;
            u_sd.fi_frag = 2'd2; u_sd.fi_fat_hinib = 4'h0;
            u_sd.fi_present = 4'hF; u_sd.fi_badsig = 4'h0; u_sd.fi_bigsize = 4'h0;
            u_sd.fi_bad_sig0 = 1'b0; u_sd.fi_bad_bps = 1'b0;
            u_sd.fi_end_slot = 9'd16; u_sd.fi_ghost_slot = 9'h1FF;
            u_sd.fi_slot[0] = 9'd13; u_sd.fi_slot[1] = 9'd2; u_sd.fi_slot[2] = 9'd9; u_sd.fi_slot[3] = 9'd4;
            for (q = 0; q < 4; q = q + 1) begin u_sd.fi_fat_fault[q] = 3'd0; u_sd.fi_size_delta[q] = 0; end
        end
    endtask

    task reset_all;
        begin
            rst = 1'b1;
            load_request = 1'b0;
            u_sd.present = 1'b1;
            wait_cycles(20);
            rst = 1'b0;
            n_load_err_events = 0;
            last_load_error   = 8'h00;
        end
    endtask

    // 发一次加载请求，等 load_done 或终止错误；res: 0=done 1=error 2=挂死
    integer res, wr0, reinit_at_req;
    task do_load(input [5:0] idx, input integer img, input integer max_ms);
        integer t;
        begin
            t = 0;
            while (!loader_ready && t < max_ms * 100000) begin @(posedge clk); t = t + 1; end
            if (!loader_ready) fail("loader_ready never came");
            cur_img = img; pix_bad = 0; wr0 = n_wr_total; reinit_at_req = sd_reinit_count;
            @(negedge clk);
            load_request = 1'b1; load_image_idx = idx; load_buffer_idx = {1'b0, idx[0]};
            @(negedge clk) load_request = 1'b0;
            res = 2; t = 0;
            while (res == 2 && t < max_ms * 100000) begin
                @(posedge clk); t = t + 1;
                if (load_done) res = 0;
                else if (load_error != 8'h00) res = 1;
            end
            wait_cycles(4);
        end
    endtask

    integer bank_err;
    task expect_load_ok(input [5:0] idx, input integer img, input [8*24-1:0] tag);
        begin
            do_load(idx, img, 60);
            wait_cycles(10);                                       // image_complete 同步到 sys 域
            bank_err = check_bank(idx[0], img);
            if (bank_err != 0 || complete_sys[idx[0]] !== 1'b1) begin
                failures = failures + 1;
                $display("FAIL @%0t: %0s bank %0d content errors %0d image_complete %b", $time, tag, idx[0], bank_err, complete_sys);
            end
            if (res != 0 || pix_bad != 0 || loaded_image_idx != idx || n_wr_total - wr0 != W * H ||
                loaded_buffer_idx != {1'b0, idx[0]}) begin
                failures = failures + 1;
                $display("FAIL @%0t: %0s load idx %0d res=%0d err=%h pix_bad=%0d pixels=%0d", $time, tag, idx, res,
                         last_load_error, pix_bad, n_wr_total - wr0);
            end
        end
    endtask

    task expect_load_err(input [5:0] idx, input integer img, input [7:0] err, input [8*24-1:0] tag);
        begin
            do_load(idx, img, 60);
            if (res != 1 || last_load_error != err) begin
                failures = failures + 1;
                $display("FAIL @%0t: %0s load idx %0d res=%0d err=%h (exp %h)", $time, tag, idx, res, last_load_error, err);
            end
        end
    endtask

    task wait_scan(input integer max_ms);
        integer t;
        begin
            t = 0;
            while (!scan_done && t < max_ms * 100000) begin @(posedge clk); t = t + 1; end
            if (!scan_done) fail("scan_done never came");
        end
    endtask

    time t_rst, t_scan, t_load0;
    integer reinit0, cmd17_0;
    initial begin
        card_default;

        // ======================= L0：有序启动 =======================
        ingest_enable = 1'b0;
        reset_all;
        wait_cycles(800_000);                                      // 8 ms > SD 初始化 ≈3 ms
        if (!dut.sd_init_done) fail("L0 SD card should initialise while SDRAM is still initialising");
        if (u_sd.n_cmd17 != 0 || dut.scan_kicked || scan_done) fail("L0 scan started before ingest_enable");
        ingest_enable = 1'b1;
        wait_scan(50);
        if (image_count != 7'd4) fail("L0 scan after ingest_enable");

        // ======================= L1：碎片化 + 大分区偏移 =======================
        card_default;
        reset_all;
        t_rst = $time;
        wait_scan(50);
        t_scan = $time;
        if (image_count != 7'd4 || !scan_all_done || fs_error != 8'h00) begin
            failures = failures + 1; $display("FAIL L1 image_count=%0d all=%0d fs_error=%h", image_count, scan_all_done, fs_error);
        end
        if (dut.u_catalog.mem[0][27:0] != u_sd.fi_clus(0, 0) || dut.u_catalog.mem[6][27:0] != u_sd.fi_clus(3, 0) ||
            dut.u_catalog.mem[5] != u_sd.FILE_LEN) fail("L1 cluster/size table");
        if (dut.scanner_data_start != u_sd.fi_data_start(0) || dut.scanner_fat_start != u_sd.fi_fat_start(0))
            fail("L1 latched layout");
        expect_load_ok(2'd0, 0, "L1 PIC1");
        t_load0 = $time;
        expect_load_ok(2'd1, 1, "L1 PIC2");
        expect_load_ok(2'd2, 2, "L1 PIC3");
        expect_load_ok(2'd3, 3, "L1 PIC4");
        expect_load_ok(2'd0, 0, "L1 PIC1 again");
        $display("INFO L1 reset->scan_done %0d us, first image %0d us, CMD17 total %0d", (t_scan - t_rst) / 1000,
                 (t_load0 - t_rst) / 1000, u_sd.n_cmd17);

        // ======================= L2：缺 PIC3 =======================
        card_default;
        u_sd.fi_present = 4'b1011;
        reset_all;
        wait_scan(50);
        if (image_count != 7'd3 || fs_error != 8'h00) begin
            failures = failures + 1; $display("FAIL L2 image_count=%0d fs_error=%h", image_count, fs_error);
        end
        expect_load_ok(2'd0, 0, "L2 slot0=PIC1");
        expect_load_ok(2'd1, 1, "L2 slot1=PIC2");
        expect_load_ok(2'd2, 3, "L2 slot2=PIC4");
        expect_load_err(2'd3, 0, 8'h57, "L2 slot3");

        // ======================= L3：BMP 头 =======================
        card_default;
        u_sd.fi_badsig = 4'b0010; u_sd.fi_bigsize = 4'b0100; u_sd.fi_size_delta[3] = -10;
        reset_all;
        wait_scan(50);
        if (image_count != 7'd4) fail("L3 scan");
        reinit0 = sd_reinit_count;
        expect_load_err(2'd1, 1, 8'h56, "L3 sig XX");
        if (n_wr_total != wr0) fail("L3 pixels written for bad signature");
        expect_load_err(2'd2, 2, 8'h56, "L3 800x600");
        if (n_wr_total != wr0) fail("L3 pixels written for 800x600");
        expect_load_err(2'd3, 3, 8'h56, "L3 size too small");
        if (n_wr_total != wr0) fail("L3 pixels written for short file");
        if (sd_reinit_count != reinit0) fail("L3 header error should not re-init SD");
        expect_load_ok(2'd0, 0, "L3 good after bad");

        // ======================= L4：FAT 链错误 =======================
        card_default;
        u_sd.fi_fat_fault[0] = 3'd1; u_sd.fi_fat_fault[1] = 3'd3;
        reset_all;
        wait_scan(50);
        expect_load_err(2'd0, 0, 8'h25, "L4 early EOF");
        if (fs_error != 8'h25) fail("L4 fs_error not 0x25");
        expect_load_err(2'd1, 1, 8'h23, "L4 self loop");
        if (fs_error != 8'h23) fail("L4 fs_error not 0x23");
        expect_load_ok(2'd3, 3, "L4 good after bad");

        // ======================= L5：播放中拔卡 =======================
        card_default;
        reset_all;
        wait_scan(50);
        expect_load_ok(2'd0, 0, "L5 before pull");
        reinit0 = sd_reinit_count;
        u_sd.present = 1'b0;
        wait_cycles(1000);
        do_load(2'd1, 1, 80);
        if (res != 1 || last_load_error != 8'h55) begin
            failures = failures + 1; $display("FAIL L5 card pulled: res=%0d err=%h (expect 0x55)", res, last_load_error);
        end
        if (sd_reinit_count == reinit0) fail("L5 no SD re-init attempted");
        if (!scan_done || image_count != 7'd4) fail("L5 resource table lost after card pull");
        wait_cycles(2000);
        if (loader_ready) fail("L5 loader_ready while card absent");
        u_sd.present = 1'b1;
        expect_load_ok(2'd1, 1, "L5 after reinsert");
        expect_load_ok(2'd2, 2, "L5 after reinsert 2");
        reinit0 = sd_reinit_count;
        u_sd.present = 1'b0;
        wait_cycles(5000);
        u_sd.present = 1'b1;
        n_load_err_events = 0;
        expect_load_ok(2'd3, 3, "L5 quick swap");
        if (n_load_err_events != 0) fail("L5 quick swap leaked an error to the player");
        if (sd_reinit_count == reinit0) fail("L5 quick swap: no re-init");

        // ======================= L6：扫描中拔卡 =======================
        card_default;
        reset_all;
        cmd17_0 = u_sd.n_cmd17;
        wait (u_sd.n_cmd17 == cmd17_0 + 2);                        // 正在读引导扇区
        reinit0 = sd_reinit_count;
        u_sd.present = 1'b0;
        wait_cycles(300_000);
        if (scan_done) fail("L6 scan finished with card absent");
        u_sd.present = 1'b1;
        wait_scan(100);
        if (image_count != 7'd4 || fs_error != 8'h00) begin
            failures = failures + 1; $display("FAIL L6 image_count=%0d fs_error=%h", image_count, fs_error);
        end
        if (sd_reinit_count == reinit0) fail("L6 scan timeout did not re-init SD");
        if (n_load_err_events != 0) fail("L6 scan-side error leaked to load_error");
        expect_load_ok(2'd2, 2, "L6 after");

        // ======================= L8：loader 报错（写停顿 0x46）→ 重试 → 上报；恢复后正常 =======================
        card_default;
        reset_all;
        wait_scan(50);
        grant_en = 1'b0;
        reinit0 = sd_reinit_count;
        expect_load_err(2'd1, 1, 8'h46, "L8 loader write stall");
        if (sd_reinit_count != reinit0) fail("L8 loader error should not re-init SD");
        grant_en = 1'b1;
        expect_load_ok(2'd1, 1, "L8 after stall");

        // ======================= L7：非 FAT32 卡 =======================
        card_default;
        u_sd.fi_bad_sig0 = 1'b1;
        reset_all;
        wait_scan(50);
        if (image_count != 7'd0 || fs_error != 8'h20) begin
            failures = failures + 1; $display("FAIL L7 image_count=%0d fs_error=%h", image_count, fs_error);
        end
        expect_load_err(2'd0, 0, 8'h57, "L7 no images");
        wait_cycles(1000);
        if (n_start != n_done || outstanding) begin
            failures = failures + 1; $display("FAIL fb_load_start %0d vs fb_done %0d (outstanding=%b)", n_start, n_done, outstanding);
        end
        $display("INFO loader handshakes: start %0d, done %0d (with error %0d), abort %0d", n_start, n_done, n_done_err, n_abort);

        if (failures == 0)
            $display("PASS tb_sd_card_bmp_fat32: ingest gate, MBR@200000 fragmented 4 images pixel-exact, missing PIC3 compaction + 0x57, header 0x56 x3 (no pixels), FAT 0x25/0x23, card pull 0x55 + reinsert, quick swap, scan pull rescan, non-FAT32 0x20, loader 0x46 retry/report, real CDC+loader bank content");
        else
            $display("FAIL tb_sd_card_bmp_fat32 failures=%0d", failures);
        $finish;
    end

    initial begin
        #2_000_000_000;
        $display("FAIL tb_sd_card_bmp_fat32 timeout");
        $finish;
    end
endmodule
