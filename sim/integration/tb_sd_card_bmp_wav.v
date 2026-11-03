`timescale 1ns/1ps

module tb_sd_card_bmp_wav;
    localparam W = 32, H = 24;
    localparam integer TICK = 130;          // 读侧 25MHz 下每 130 拍一个 sample_tick（真实 521，加速 4 倍）

    reg clk = 1'b0;
    reg rst = 1'b1;
    reg load_request = 1'b0;
    reg [5:0] load_image_idx = 6'd0;
    wire scan_done, scan_all_done;
    wire [6:0] image_count;
    wire loader_ready, load_done;
    wire [5:0] loaded_image_idx;
    wire [1:0] loaded_buffer_idx;
    wire [7:0] load_error, sd_reinit_count, fs_error;
    wire fb_load_start, fb_pixel_valid, fb_frame_commit, fb_load_abort, fb_done, fb_busy;
    wire [1:0] fb_load_bank;
    wire [23:0] fb_pixel_rgb;
    wire [7:0] fb_error;
    wire SD_nCS, SD_DCLK, SD_MOSI, SD_MISO;
    always #5 clk = ~clk;                   // 100 MHz

    // 音频口
    wire [15:0] trk_valid;
    wire        info_req, info_valid;
    wire [3:0]  info_idx;
    wire [31:0] info_start, info_total;
    wire        a_req, a_busy, a_valid, a_done;
    wire [31:0] a_lba;
    wire [7:0]  a_byte, a_error;
    wire [8:0]  a_bidx;

    sd_card_bmp #(
        .CLK_FREQ_HZ        (100_000_000),
        .BMP_WIDTH          (W),
        .BMP_HEIGHT         (H),
        .SD_READ_TIMEOUT_MS (16'd1),
        .LOAD_TIMEOUT_MS    (20),
        .MAX_RETRIES        (2'd1),
        .SD_REINIT_RETRY_MS (8)
    ) dut (
        .clk(clk), .rst_n(~rst), .ingest_enable(1'b1), .sd_ready(),
        .scan_done(scan_done), .scan_all_done(scan_all_done), .image_count(image_count),
        .loader_ready(loader_ready), .load_request(load_request),
        .load_image_idx(load_image_idx), .load_buffer_idx({1'b0, load_image_idx[0]}),
        .load_done(load_done), .loaded_image_idx(loaded_image_idx),
        .loaded_buffer_idx(loaded_buffer_idx), .load_error(load_error),
        .bmp_state_code(), .sd_reinit_count(sd_reinit_count), .fs_error(fs_error),
        .fb_load_start(fb_load_start), .fb_load_bank(fb_load_bank), .pixel_valid(fb_pixel_valid),
        .pixel_rgb(fb_pixel_rgb), .frame_commit(fb_frame_commit), .fb_load_abort(fb_load_abort),
        .fb_done(fb_done), .fb_error(fb_error), .fb_busy(fb_busy),
        .audio_track_valid(trk_valid), .audio_info_req(info_req), .audio_info_idx(info_idx),
        .audio_info_valid(info_valid), .audio_info_start_lba(info_start), .audio_info_total_sectors(info_total),
        .audio_sector_req(a_req), .audio_sector_lba(a_lba), .audio_sector_busy(a_busy),
        .audio_sector_valid(a_valid), .audio_sector_byte(a_byte), .audio_sector_byte_index(a_bidx),
        .audio_sector_done(a_done), .audio_sector_error(a_error),
        .SD_nCS(SD_nCS), .SD_DCLK(SD_DCLK), .SD_MOSI(SD_MOSI), .SD_MISO(SD_MISO)
    );

    sd_card_fat32_model #(.W(W), .H(H)) u_sd (
        .nCS(SD_nCS), .DCLK(SD_DCLK), .MOSI(SD_MOSI), .MISO(SD_MISO)
    );

    // ---------------- 帧缓存写侧（同 tb_sd_card_bmp_fat32）----------------
    reg mem_clk = 1'b0;
    always #12.5 mem_clk = ~mem_clk;
    wire ld_start, ld_pv, ld_commit, ld_busy, ld_done;
    wire [1:0] ld_index;
    wire [23:0] ld_rgb;
    wire [7:0] ld_dec_err, ld_mem_err;
    wire [3:0] ld_complete, complete_sys;
    wire app_wr_en; wire [20:0] app_wr_addr; wire [3:0] app_wr_mask; wire [31:0] app_wr_data;
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
    media_frame_loader #(.IMAGE_WORDS(W * H), .FIFO_DEPTH(512), .BURST_SIZE(128), .STALL_TIMEOUT(4000),
        .WRITE_V_FLIP(1), .FRAME_WIDTH(W), .FRAME_HEIGHT(H)) u_loader (
        .clk(mem_clk), .rst_n(~rst), .load_start(ld_start), .image_index(ld_index),
        .pixel_valid(ld_pv), .pixel_rgb(ld_rgb), .frame_commit(ld_commit), .decode_error(ld_dec_err),
        .load_busy(ld_busy), .load_done(ld_done), .image_complete(ld_complete), .memory_error(ld_mem_err),
        .controller_init_done(1'b1), .controller_busy(1'b0), .refresh_active(1'b0),
        .burst_want(), .burst_grant(1'b1), .burst_busy(),
        .app_wr_en(app_wr_en), .app_wr_addr(app_wr_addr), .app_wr_mask(app_wr_mask), .app_wr_data(app_wr_data)
    );

    // ---------------- 播放列表 + 播放器（读侧 25MHz）----------------
    reg pix_clk = 1'b0;
    always #20 pix_clk = ~pix_clk;
    reg pl_enable = 1'b0;
    reg [4:0] pl_select = 5'd0;
    wire        play_enable, wr_idle, wr_done, wr_error;
    wire [31:0] pl_start, pl_total;
    wire [3:0]  pl_track;
    wire        raw_v;
    wire signed [15:0] raw_s;
    reg         tick = 1'b0;
    integer     tdiv = 0;
    always @(posedge pix_clk) begin tdiv <= (tdiv == TICK-1) ? 0 : tdiv + 1; tick <= (tdiv == TICK-1); end

    wav_playlist_ctrl #(.LOOP(1)) u_pl (
        .clk(clk), .rst_n(~rst), .enable(pl_enable), .track_valid(trk_valid), .select(pl_select),
        .info_req(info_req), .info_idx(info_idx), .info_valid(info_valid),
        .info_start_lba(info_start), .info_total_sectors(info_total),
        .play_enable(play_enable), .start_lba(pl_start), .total_sectors(pl_total),
        .wr_idle(wr_idle), .wr_done(wr_done), .wr_error(wr_error), .track(pl_track), .playing(), .switching()
    );
    wav_stream_player u_player (
        .wr_clk(clk), .wr_rst_n(~rst), .play_enable(play_enable),
        .start_lba(pl_start), .total_sectors(pl_total),
        .sd_req(a_req), .sd_lba(a_lba), .sd_busy(a_busy), .sd_valid(a_valid), .sd_byte(a_byte),
        .sd_byte_index(a_bidx), .sd_done(a_done), .sd_error(a_error),
        .wr_idle(wr_idle), .wr_done(wr_done), .wr_error(wr_error),
        .rd_clk(pix_clk), .rd_rst_n(~rst), .sample_tick(tick),
        .sample_out(raw_s), .sample_valid(raw_v), .sample_ready(1'b1),
        .sample_counter(), .stream_active(), .stream_end(), .buffer_low()
    );

    // ---------------- 检查 ----------------
    integer failures = 0;
    task fail(input [8*72-1:0] msg); begin failures = failures + 1; $display("FAIL @%0t: %0s", $time, msg); end endtask
    task wait_cycles(input integer n); begin repeat (n) @(posedge clk); end endtask

    function integer tm; input integer k; begin tm = (k == 15) ? 3 : k; end endfunction   // 曲目号 → 卡模型下标
    function [31:0] exp_start; input integer k; begin exp_start = u_sd.fi_data_start(0) + (u_sd.fi_trk_clus[tm(k)] - 2) * u_sd.fi_spc; end endfunction
    function [31:0] exp_total; input integer k; begin exp_total = u_sd.fi_trk_size[tm(k)] >> 9; end endfunction

    // 图片像素流
    integer cur_img = 0, n_pix = 0, pix_bad = 0;
    reg frame_open = 1'b0;
    always @(posedge clk) begin
        if (fb_load_start) begin n_pix <= 0; frame_open <= 1'b1; end
        else if (fb_pixel_valid && frame_open) begin
            if (fb_pixel_rgb !== u_sd.pix_rgb(cur_img, n_pix % W, n_pix / W)) pix_bad <= pix_bad + 1;
            if (n_pix == W * H - 1) frame_open <= 1'b0;
            n_pix <= n_pix + 1;
        end
    end

    // 定位结果
    integer n_info = 0;
    always @(posedge clk) if (info_valid) begin
        n_info = n_info + 1;
        if (info_start !== exp_start(info_idx) || info_total !== exp_total(info_idx)) begin
            fail("W1 locator start/total");
            $display("     idx=%0d start=%0d (exp %0d) total=%0d (exp %0d)", info_idx, info_start, exp_start(info_idx), info_total, exp_total(info_idx));
        end
    end

    // 曲目开始 / 结束，扇区范围
    integer order [0:31];
    integer n_tracks = 0, n_done_ok = 0, n_err_end = 0, n_aud_sec = 0;
    reg [31:0] cur_start, cur_total;
    reg [31:0] exp_lba; integer exp_k = 0, trk_samples = 0, mism = 0, underflow = 0;
    reg uf_on = 1'b1;                     // W5 拔卡期间的静音不算下溢
    reg seen_real = 1'b0, pe_q = 1'b0;
    always @(posedge clk) begin
        pe_q <= play_enable;
        if (play_enable && !pe_q) begin
            if (n_tracks < 32) order[n_tracks] = pl_track;
            n_tracks = n_tracks + 1;
            cur_start = pl_start; cur_total = pl_total;
            exp_lba = pl_start; exp_k = 0; trk_samples = 0; seen_real = 1'b0;
        end
        if (a_req) begin
            n_aud_sec = n_aud_sec + 1;
            if (a_lba < cur_start || a_lba >= cur_start + cur_total) begin fail("W2 audio sector outside track"); $display("     lba=%0d", a_lba); end
        end
        if (play_enable && wr_done && pe_q) begin
            if (trk_samples != cur_total * 256) begin fail("W2 track sample count"); $display("     got %0d exp %0d", trk_samples, cur_total*256); end
        end
    end
    always @(posedge clk) begin : end_count
        reg d_q, e_q;
        d_q <= wr_done; e_q <= wr_error;
        if (wr_done && !d_q) n_done_ok = n_done_ok + 1;
        if (wr_error && !e_q) n_err_end = n_err_end + 1;
    end
    // 读侧样本
    always @(posedge pix_clk) if (!rst && raw_v) begin
        if (u_player.cur_real) begin
            if (raw_s !== u_sd.fi_pcm_sample(exp_lba, exp_k)) begin
                mism = mism + 1;
                if (mism < 6) $display("FAIL @%0t: W2 sample lba=%0d k=%0d got %h exp %h", $time, exp_lba, exp_k, raw_s, u_sd.fi_pcm_sample(exp_lba, exp_k));
            end
            seen_real = 1'b1; trk_samples = trk_samples + 1;
            exp_k = exp_k + 1; if (exp_k == 256) begin exp_k = 0; exp_lba = exp_lba + 1; end
        end else if (uf_on && seen_real && !u_player.ended_rd && u_player.en_rd) underflow = underflow + 1;
    end

    // 调试时间线（+define+WAV_TRACE）
`ifdef WAV_TRACE
    reg uf_q = 0; reg de_q = 0, ee_q = 0, pe_t = 0;
    always @(posedge clk) begin
        pe_t <= play_enable; de_q <= wr_done; ee_q <= wr_error;
        if (play_enable && !pe_t) $display("TRACE %0t start TRACK%0d lba %0d n %0d", $time, pl_track+1, pl_start, pl_total);
        if (wr_done && !de_q) $display("TRACE %0t wr_done", $time);
        if (wr_error && !ee_q) $display("TRACE %0t wr_error", $time);
        if (load_request) $display("TRACE %0t load_request %0d", $time, load_image_idx);
        if (a_req) $display("TRACE %0t a_req lba %0d level %0d", $time, a_lba, u_player.level_wr);
        if (a_done) $display("TRACE %0t a_done err %h", $time, a_error);
    end
    always @(posedge pix_clk) begin
        uf_q <= (uf_on && seen_real && !u_player.ended_rd && u_player.en_rd && raw_v && !u_player.cur_real);
        if (raw_v && !u_player.cur_real && uf_on && seen_real && !u_player.ended_rd && u_player.en_rd && !uf_q) $display("TRACE %0t underflow level_rd %0d", $time, u_player.level_rd);
    end
`endif
    // 音频扇区等待时间（req → done），看仲裁让出
    integer t_req = 0, wait_max = 0, cyc = 0;
    always @(posedge clk) begin
        cyc = cyc + 1;
        if (a_req) t_req = cyc;
        if (a_done && cyc - t_req > wait_max) wait_max = cyc - t_req;
    end

    // ---------------- 任务 ----------------
    integer res;
    task do_load(input [5:0] idx, input integer img);
        integer t;
        begin
            t = 0;
            while (!loader_ready && t < 6000000) begin @(posedge clk); t = t + 1; end
            if (!loader_ready) fail("loader_ready never came");
            cur_img = img; pix_bad = 0;
            @(negedge clk) begin load_request = 1'b1; load_image_idx = idx; end
            @(negedge clk) load_request = 1'b0;
            res = 2; t = 0;
            while (res == 2 && t < 6000000) begin
                @(posedge clk); t = t + 1;
                if (load_done) res = 0; else if (load_error != 8'h00) res = 1;
            end
            wait_cycles(4);
        end
    endtask
    task expect_load_ok(input [5:0] idx, input integer img, input [8*16-1:0] tag);
        begin
            do_load(idx, img);
            if (res != 0 || pix_bad != 0 || loaded_image_idx != idx) begin
                failures = failures + 1; $display("FAIL @%0t: %0s load %0d res=%0d err=%h pix_bad=%0d", $time, tag, idx, res, load_error, pix_bad);
            end
        end
    endtask
    task wait_tracks(input integer n, input integer max_ms);
        integer t;
        begin
            t = 0;
            while (n_tracks < n && t < max_ms * 100) begin #10000; t = t + 1; end
            if (n_tracks < n) fail("track never started");
        end
    endtask

    integer q, k0, s0;
    initial begin
        // 卡：MBR @200000，spc=2，图片交错倒序碎片化，曲目连续存放
        u_sd.fi_mbr = 1; u_sd.fi_part_lba = 32'd200000; u_sd.fi_part_type = 8'h0C;
        u_sd.fi_spc = 8'd2; u_sd.fi_data_clusters = 32'd90000; u_sd.fi_cbase = 32'd70000; u_sd.fi_frag = 2'd2;
        u_sd.fi_end_slot = 9'd30;
        u_sd.fi_slot[0] = 9'd13; u_sd.fi_slot[1] = 9'd2; u_sd.fi_slot[2] = 9'd9; u_sd.fi_slot[3] = 9'd4;
        u_sd.fi_trk_valid[0] = 1; u_sd.fi_trk_slot[0] = 9'd5;  u_sd.fi_trk_clus[0] = 32'd1000; u_sd.fi_trk_size[0] = 3*512 + 300;
        u_sd.fi_trk_valid[1] = 1; u_sd.fi_trk_slot[1] = 9'd17; u_sd.fi_trk_clus[1] = 32'd1010; u_sd.fi_trk_size[1] = 2*512;
        u_sd.fi_trk_valid[3] = 1; u_sd.fi_trk_slot[3] = 9'd19; u_sd.fi_trk_clus[3] = 32'd1020; u_sd.fi_trk_size[3] = 6*512;
        u_sd.fi_trk_decoy = 1; u_sd.fi_decoy_slot = 9'd21;    // 21..27

        // ===== W0 扫描 =====
        wait_cycles(20); rst = 1'b0;
        q = 0; while (!scan_done && q < 5000) begin #10000; q = q + 1; end
        if (!scan_done) fail("W0 scan_done never came");
        if (trk_valid !== 16'b1000_0000_0000_0011) begin fail("W0 audio_track_valid"); $display("     got %b", trk_valid); end
        if (image_count != 7'd4) fail("W0 image_count != 4");
        for (q = 0; q < 16; q = q + 1) if (q < 2 || q == 15)
            // L9（第二批优化）：曲目条目字 0 = 首扇区 LBA（扫描时换算好），不再是首簇号
            if (dut.u_catalog.mem[(128+q)*2] !== exp_start(q) || dut.u_catalog.mem[(128+q)*2+1] !== u_sd.fi_trk_size[tm(q)])
                begin fail("W0 catalog entry 128+k"); $display("     k=%0d lba=%0d size=%0d", q, dut.u_catalog.mem[(128+q)*2], dut.u_catalog.mem[(128+q)*2+1]); end
        $display("INFO W0: scan ok, image_count=%0d audio_track_valid=%b, data_start=%0d", image_count, trk_valid, u_sd.fi_data_start(0));
        expect_load_ok(6'd0, 0, "W0 load");

        // ===== W1/W2 音画同步：曲目跟随 select（当前图片） =====
        @(negedge clk) pl_enable = 1'b1;                  // select=0
        wait_tracks(2, 200);                              // TRACK1 播完 → 再播 TRACK1
        wait_cycles(20000);
        @(negedge clk) pl_select = 5'd1;                  // 播放中换图 → TRACK2
        wait_tracks(3, 50);
        wait_cycles(20000);
        @(negedge clk) pl_select = 5'd2;                  // TRACK3 缺失 → 停曲、不顺延
        wait (!play_enable); wait_cycles(2000);
        s0 = n_aud_sec; q = n_tracks;
        wait_cycles(200000);
        if (play_enable || n_tracks != q || n_aud_sec != s0) fail("W2 missing track should play nothing");
        @(negedge clk) pl_select = 5'd15;                 // → TRACK16
        wait_tracks(4, 50);
        // ===== W3 播 TRACK16 时连续加载 4 张图 =====
        k0 = n_aud_sec;
        expect_load_ok(6'd1, 1, "W3 load1");
        expect_load_ok(6'd2, 2, "W3 load2");
        expect_load_ok(6'd3, 3, "W3 load3");
        expect_load_ok(6'd0, 0, "W3 load0");
        $display("INFO W3: 4 image loads during TRACK16, %0d audio sectors interleaved, max audio sector latency %0d cycles",
                 n_aud_sec - k0, wait_max);
        if (n_aud_sec == k0) fail("W3 no audio sector interleaved with image loads");
        wait_tracks(5, 300);                              // 停在图 15：TRACK16 单曲循环
        wait_cycles(20000);
        @(negedge clk) pl_select = 5'd0;                  // 回到图 0 → TRACK1
        wait_tracks(6, 50);
        if (order[0] != 0 || order[1] != 0 || order[2] != 1 || order[3] != 15 || order[4] != 15 || order[5] != 0) begin
            fail("W2 image->track order"); $display("     %0d %0d %0d %0d %0d %0d", order[0], order[1], order[2], order[3], order[4], order[5]);
        end
        if (n_info < 6) fail("W1 locator not queried per track");
        $display("INFO W2: sel 0,0(loop),1,(2 none),15,15(loop),0 -> TRACK%0d %0d %0d %0d %0d %0d, %0d tracks completed, samples ok (mismatch %0d, underflow %0d)",
                 order[0]+1, order[1]+1, order[2]+1, order[3]+1, order[4]+1, order[5]+1, n_done_ok, mism, underflow);
        if (n_done_ok < 2) fail("W2 loop: fewer than 2 tracks completed");

        // ===== W4 SW1 关 / 开 =====
        wait_cycles(20000);                               // TRACK1 播放中
        @(negedge clk) pl_enable = 1'b0;
        wait (!play_enable); wait_cycles(2000);
        s0 = n_aud_sec;
        expect_load_ok(6'd2, 2, "W4 load while off");
        wait_cycles(200000);
        if (n_aud_sec != s0) fail("W4 audio sectors read while disabled");
        if (!wr_idle) fail("W4 player not idle after disable");
        q = n_tracks;
        @(negedge clk) pl_enable = 1'b1;
        wait_tracks(q + 1, 50);
        if (order[q] != 0) begin fail("W4 resume should restart image track (TRACK1)"); $display("     got TRACK%0d", order[q]+1); end
        $display("INFO W4: disable -> no audio sectors, image load ok; enable -> TRACK%0d restarts", order[q]+1);
        wait_cycles(20000);
        @(negedge clk) pl_select = 5'd15;                 // W5 用最长的 TRACK16

        // ===== W5 播放中拔卡 =====
        wait_tracks(q + 2, 200);                          // TRACK16（最长）
        if (order[q+1] != 15) fail("W5 expected TRACK16");
        wait_cycles(50000);
        uf_on = 1'b0;
        q = n_err_end;
        u_sd.present = 1'b0;
        wait_cycles(400000);
        if (n_err_end == q) fail("W5 no wr_error after card pull");
        u_sd.present = 1'b1;
        q = n_done_ok;
        k0 = 0; while (n_done_ok == q && k0 < 3000) begin #10000; k0 = k0 + 1; end
        if (n_done_ok == q) fail("W5 playback did not recover after reinsertion");
        expect_load_ok(6'd3, 3, "W5 load after reinsertion");
        $display("INFO W5: card pull -> audio sector error, wr_error events %0d, recovered (tracks %0d done), reinit %0d",
                 n_err_end, n_done_ok, sd_reinit_count);

        if (mism != 0) fail("W2 sample mismatches");
        if (underflow != 0) begin fail("W3 audio underflow"); $display("     %0d silent samples", underflow); end
        if (failures == 0) $display("PASS tb_sd_card_bmp_wav");
        else               $display("FAIL tb_sd_card_bmp_wav: %0d failures", failures);
        $finish;
    end
    initial begin #400_000_000; $display("FAIL tb_sd_card_bmp_wav: timeout"); $finish; end
endmodule
