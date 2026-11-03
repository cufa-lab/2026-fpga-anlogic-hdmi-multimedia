`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// tb_smart_load ——  智能构图（加载链路模块级子系统；资源取舍：无局部重读、无主体检测、无贴边阴影）
//   真实 sd_card_bmp(ANY_SIZE=1, ROW_SKIP=1, CANCEL_EN=1, SMART=1) + FAT32 扫描 / 流读 + 适配层 + sd_card_top 快速替身
//   → bmp_smart_scaler → media_frame_load_cdc → media_frame_loader（与板上同配置），SDRAM 为行为模型
// 参考模型逐位复现硬件定点：构图（INT 整数倍 / FILL 4q×3q 居中裁切 / FIT 原样居中）、
//   轻量柔化背景（主图边缘延展 → 竖向逐列 + 横向沿行一阶低通 α=1/16 四舍五入 → ×0.625）
// 检查：
//   S1 bank 640×480 逐像素 = 参考（主图区 = 最近邻窗口映射；边框 = 模糊背景；FIT 主图与第十三讲逐位相同）
//   S2 构图几何（plan_*）= 参考；FILL 窗口居中
//   S3 sd_card_bmp 像素数：第 1 遍（判头）= 0，第 2 遍 = 窗口用到的行；读卡扇区打印
//   S4 缩放器 / CDC 不溢出
//   S5 所有写入都落在 4 个 bank 内（< 1228800）
//   X1 加载中途取消 → 0x59，随后加载正确
// -----------------------------------------------------------------------------
module tb_smart_load;
    localparam IMG_WORDS = 307200;

    reg clk = 1'b0, rst = 1'b1;
    always #10 clk = ~clk;                      // 50 MHz
    reg mem_clk = 1'b0;
    always #12.5 mem_clk = ~mem_clk;            // 40 MHz

    reg load_request = 1'b0; reg [5:0] load_image_idx = 6'd0; reg [1:0] load_buffer_idx = 2'd0;
    reg load_cancel = 1'b0;
    wire scan_done, loader_ready, load_done; wire [6:0] image_count; wire [7:0] load_error;
    wire [5:0] loaded_image_idx; wire [1:0] loaded_buffer_idx;
    wire fb_load_start, fb_pixel_valid, fb_frame_commit, fb_load_abort, fb_done, fb_busy;
    wire [1:0] fb_load_bank; wire [23:0] fb_pixel_rgb; wire [7:0] fb_error; wire [5:0] fb_load_image;
    wire geom_valid; wire [11:0] src_w, src_h; wire src_hold;
    wire sc_valid, sc_commit, sc_busy, sc_overflow; wire [23:0] sc_rgb;
    wire map_valid, map_direct; wire [11:0] map_c0, map_r0b, map_cw, map_ch, map_n; wire [9:0] map_d;
    wire plan_valid; wire [9:0] g_ox, g_d; wire [8:0] g_oyt; wire [11:0] g_c0, g_r0t, g_cw, g_ch, g_n;

    sd_card_bmp #(
        .CLK_FREQ_HZ(50_000_000), .BMP_WIDTH(640), .BMP_HEIGHT(480),
        .SD_READ_TIMEOUT_MS(16'd2), .LOAD_TIMEOUT_MS(40), .MAX_RETRIES(2'd1), .SD_REINIT_RETRY_MS(16),
        .ANY_SIZE(1), .ROW_SKIP(1), .CANCEL_EN(1), .SMART(1)
    ) dut (
        .clk(clk), .rst_n(~rst), .ingest_enable(1'b1), .sd_ready(),
        .scan_done(scan_done), .scan_all_done(), .image_count(image_count),
        .loader_ready(loader_ready), .load_request(load_request),
        .load_image_idx(load_image_idx), .load_buffer_idx(load_buffer_idx),
        .load_done(load_done), .loaded_image_idx(loaded_image_idx), .loaded_buffer_idx(loaded_buffer_idx),
        .load_error(load_error), .load_cancel(load_cancel), .bmp_state_code(), .sd_reinit_count(), .fs_error(),
        .fb_load_start(fb_load_start), .fb_load_bank(fb_load_bank), .pixel_valid(fb_pixel_valid),
        .pixel_rgb(fb_pixel_rgb), .frame_commit(fb_frame_commit), .fb_load_abort(fb_load_abort),
        .fb_done(fb_done), .fb_error(fb_error), .fb_busy(fb_busy), .fb_load_image(fb_load_image),
        .load_refine(1'b0), .fb_load_refine(),
        .sm_map_valid(map_valid), .sm_c0(map_c0), .sm_r0b(map_r0b), .sm_cw(map_cw), .sm_ch(map_ch),
        .sm_n(map_n), .sm_d(map_d),
        .src_geom_valid(geom_valid), .src_width(src_w), .src_height(src_h), .src_hold(src_hold), .src_progress(sc_valid),
        .audio_track_valid(), .audio_info_req(1'b0), .audio_info_idx(2'd0), .audio_info_valid(),
        .audio_info_start_lba(), .audio_info_total_sectors(),
        .audio_sector_req(1'b0), .audio_sector_lba(32'd0), .audio_sector_busy(), .audio_sector_valid(),
        .audio_sector_byte(), .audio_sector_byte_index(), .audio_sector_done(), .audio_sector_error(),
        .narr_info_req(1'b0), .narr_info_idx(4'd0), .narr_sector_req(1'b0), .narr_sector_lba(32'd0),
        .SD_nCS(), .SD_DCLK(), .SD_MOSI(), .SD_MISO(1'b1));

    `define SDM dut.sd_card_top_m0.u_m

    bmp_smart_scaler #(.PACE(16)) u_sc (
        .clk(clk), .rst_n(~rst), .load_start(fb_load_start), .load_abort(fb_load_abort),
        .refine(1'b0), .ref_c0(12'd0), .ref_r0b(12'd0), .ref_cw(12'd576), .ref_ch(12'd432),
        .geom_valid(geom_valid), .src_w(src_w), .src_h(src_h),
        .in_valid(fb_pixel_valid), .in_rgb(fb_pixel_rgb), .in_hold(src_hold),
        .out_valid(sc_valid), .out_rgb(sc_rgb), .out_commit(sc_commit), .busy(sc_busy), .overflow(sc_overflow),
        .map_valid(map_valid), .map_direct(map_direct), .map_c0(map_c0), .map_r0b(map_r0b), .map_cw(map_cw),
        .map_ch(map_ch), .map_n(map_n), .map_d(map_d),
        .plan_valid(plan_valid), .g_ox(g_ox), .g_oyt(g_oyt), .g_c0(g_c0), .g_r0t(g_r0t), .g_cw(g_cw), .g_ch(g_ch),
        .g_n(g_n), .g_d(g_d));

    wire ld_start, ld_pv, ld_commit, ld_busy, ld_done, cdc_ovf;
    wire [1:0] ld_index; wire [23:0] ld_rgb; wire [7:0] ld_dec_err, ld_mem_err; wire [3:0] ld_complete, complete_sys;
    media_frame_load_cdc u_cdc (
        .src_clk(clk), .src_rst_n(~rst),
        .load_start(fb_load_start), .load_bank(fb_load_bank), .pixel_valid(sc_valid),
        .pixel_rgb(sc_rgb), .frame_commit(sc_commit), .load_abort(fb_load_abort),
        .done_event(fb_done), .done_error(fb_error), .image_complete_sys(complete_sys), .stream_overflow(cdc_ovf),
        .dst_clk(mem_clk), .dst_rst_n(~rst),
        .ld_load_start(ld_start), .ld_image_index(ld_index), .ld_pixel_valid(ld_pv), .ld_pixel_rgb(ld_rgb),
        .ld_frame_commit(ld_commit), .ld_decode_error(ld_dec_err),
        .ld_load_done(ld_done), .ld_memory_error(ld_mem_err), .ld_image_complete(ld_complete));

    wire ld_want, ld_bbusy, ld_wr_en; wire [20:0] ld_wr_addr; wire [3:0] ld_wr_mask; wire [31:0] ld_wr_data;
    media_frame_loader #(.IMAGE_WORDS(IMG_WORDS), .FIFO_DEPTH(512), .BURST_SIZE(128), .STALL_TIMEOUT(65535),
        .WRITE_V_FLIP(1), .FRAME_WIDTH(640), .FRAME_HEIGHT(480)) u_loader (
        .clk(mem_clk), .rst_n(~rst), .load_start(ld_start), .image_index(ld_index),
        .pixel_valid(ld_pv), .pixel_rgb(ld_rgb), .frame_commit(ld_commit), .decode_error(ld_dec_err),
        .load_busy(ld_busy), .load_done(ld_done), .image_complete(ld_complete), .memory_error(ld_mem_err),
        .controller_init_done(1'b1), .controller_busy(1'b0), .refresh_active(1'b0),
        .burst_want(ld_want), .burst_grant(ld_want && !ld_bbusy), .burst_busy(ld_bbusy),
        .app_wr_en(ld_wr_en), .app_wr_addr(ld_wr_addr), .app_wr_mask(ld_wr_mask), .app_wr_data(ld_wr_data));

    reg [23:0] sdram_mem [0:2097151];
    integer errors = 0;
    task fail(input [8*72-1:0] msg); begin errors = errors + 1; if (errors < 40) $display("FAIL @%0t: %0s", $time, msg); end endtask
    integer burst_bad = 0;
    always @(posedge mem_clk) begin
        if (ld_wr_en) sdram_mem[ld_wr_addr] <= ld_wr_data[23:0];
        if (ld_wr_en && ld_wr_addr >= 21'd1228800) burst_bad = burst_bad + 1;
        if (cdc_ovf) fail("S4 CDC stream overflow");
    end
    integer sc_peak = 0;
    always @(posedge clk) begin
        if (u_sc.fifo_cnt > sc_peak) sc_peak = u_sc.fifo_cnt;
        if (sc_overflow) fail("S4 scaler FIFO overflow");
    end

    // 调试跟踪（+TRACE）
    reg [1:0] tr_ld; reg [5:0] tr_st; reg tr_on = 0;
    initial tr_on = $test$plusargs("TRACE");
    always @(posedge clk) if (tr_on) begin
        if (dut.ld_state != tr_ld) $display("TR %0t ld_state %0d -> %0d rows %0d", $time, tr_ld, dut.ld_state, dut.any_row);
        if (u_sc.st != tr_st && (u_sc.st < 6'd24 || tr_st < 6'd24)) $display("TR %0t sc %0d -> %0d", $time, tr_st, u_sc.st);
        if (dut.load_abort) $display("TR %0t load_abort timeout_ms %0d", $time, dut.load_timeout_ms);
        if (dut.ld_restart) $display("TR %0t restart", $time);
        tr_ld <= dut.ld_state; tr_st <= u_sc.st;
    end

    // ---------------- 读卡扇区计数 ----------------
    integer n_data_rd = 0;
    always @(posedge clk)
        if (`SDM.state == 0 && `SDM.sd_sec_read && `SDM.sd_sec_read_addr >= `SDM.fi_data_start(0)) n_data_rd = n_data_rd + 1;
    integer n_pix = 0, n_pix_p1 = 0;
    always @(posedge clk) begin
        if (fb_load_start) n_pix = 0;
        if (dut.ld_restart) n_pix_p1 = n_pix;
        if (fb_pixel_valid) n_pix = n_pix + 1;
    end
    integer rd_p1 = 0;
    always @(posedge clk) if (dut.ld_restart) rd_p1 = n_data_rd;
    // 规划几何
    reg [9:0] cp_ox, cp_d; reg [8:0] cp_oyt; reg [11:0] cp_c0, cp_r0t, cp_cw, cp_ch, cp_n;
    integer n_plan = 0;
    always @(posedge clk) if (plan_valid) begin
        n_plan = n_plan + 1;
        cp_ox = g_ox; cp_oyt = g_oyt; cp_c0 = g_c0; cp_r0t = g_r0t; cp_cw = g_cw; cp_ch = g_ch;
        cp_n = g_n; cp_d = g_d;
    end

    // ================= 参考模型 =================
    integer MODE, WIDE, Q, C0, R0B, CW, CH, N, D, DW, DH, OX, PRE, KK, BLUR;
    reg [23:0] vm [0:639];
    // 一阶低通 a + floor((b - a + 8) / 16)（与 bmp_smart_scaler.lp16 相同）
    function [7:0] lp16(input integer a, input integer b);
        begin lp16 = (a + ((b - a + 8) >>> 4)) & 255; end
    endfunction
    function [23:0] lp16x3(input [23:0] a, input [23:0] b);
        begin lp16x3 = {lp16(a[23:16], b[23:16]), lp16(a[15:8], b[15:8]), lp16(a[7:0], b[7:0])}; end
    endfunction

    task plan_ref(input integer img, input integer w, input integer h);
        begin
            WIDE = (3 * w >= 4 * h);
            Q = WIDE ? (h * 21846) >> 16 : w >> 2;
            if (w <= 320 && h <= 240) MODE = 2;
            else if (h <= w && 9 * w <= 16 * h) MODE = 1;
            else MODE = 0;
            C0 = 0; R0B = 0; CW = w; CH = h; BLUR = 0;
            if (MODE == 1) begin
                CW = 4 * Q; CH = 3 * Q; N = CW; D = 640; DW = 640; DH = 480;
                C0 = (w - CW) >> 1; R0B = (h - CH) >> 1;
            end else if (MODE == 2) begin
                KK = 1; DW = w; DH = h;
                while (DW + w <= 640 && DH + h <= 480 && KK != 8) begin DW = DW + w; DH = DH + h; KK = KK + 1; end
                N = 1; D = KK;
            end else begin
                if (WIDE) begin N = w; D = 640; end else begin N = h; D = 480; end
                DW = (D * w + N - 1) / N; DH = (D * h + N - 1) / N;
                if (DW > 640) DW = 640; if (DH > 480) DH = 480;
            end
            OX = (640 - DW) >> 1; PRE = 480 - DH - ((480 - DH) >> 1);
            BLUR = (MODE != 1) && (DW != 640 || DH != 480);
        end
    endtask

    function [23:0] ref_main(input integer img, input integer o, input integer x);
        begin ref_main = `SDM.pix_rgb(img, C0 + ((x - OX) * N) / D, R0B + ((o - PRE) * N) / D); end
    endfunction

    // ---------------- 普通加载 ----------------
    integer res, t;
    task start_load(input [5:0] idx, input [1:0] bank);
        begin
            t = 0;
            while (!loader_ready && t < 5_000_000) begin @(posedge clk); t = t + 1; end
            sc_peak = 0;
            @(negedge clk); load_request = 1; load_image_idx = idx; load_buffer_idx = bank;
            @(negedge clk); load_request = 0;
        end
    endtask
    task wait_result;
        begin
            res = 2; t = 0;
            while (res == 2 && t < 80_000_000) begin
                @(posedge clk); t = t + 1;
                if (load_done) res = 0; else if (load_error != 8'h00) res = load_error;
            end
        end
    endtask

    integer rows2;
    task dda_rows(input integer n, input integer d, input integer ch);
        integer r, yr;
        begin
            rows2 = 0; yr = d;
            for (r = 0; r < ch; r = r + 1)
                if (n <= d || yr > 0) begin rows2 = rows2 + 1; yr = yr + d - n; end else yr = yr + d;
        end
    endtask

    task check_smart(input [5:0] idx, input [1:0] bank, input integer img, input integer w, input integer h,
                     input integer exp_mode, input [8*24-1:0] tag);
        integer o, x, bad, bad_main, bad_bg, d0, c, dk, p0, orow, xc;
        reg [23:0] e, got, src, hh; reg inside;
        begin
            plan_ref(img, w, h);
            d0 = n_data_rd; p0 = n_plan;
            start_load(idx, bank);
            wait_result;
            repeat (300) @(posedge clk);
            if (res != 0) begin fail("load did not complete"); $display("INFO %0s res %h", tag, res); end
            if (MODE != exp_mode) begin fail("test setup: mode"); $display("INFO %0s ref mode %0d exp %0d", tag, MODE, exp_mode); end
            if (u_sc.mode != MODE) begin fail("S2 mode"); $display("INFO %0s mode %0d ref %0d", tag, u_sc.mode, MODE); end
            // S2 几何
            if (n_plan != p0 + 1) fail("S2 plan_valid once");
            if (cp_ox != OX || cp_oyt != 480 - DH - PRE || cp_c0 != C0 || cp_r0t != h - CH - R0B || cp_cw != CW ||
                cp_ch != CH || cp_n != N || cp_d != D) begin
                fail("S2 plan geometry");
                $display("INFO %0s got ox%0d oyt%0d c0 %0d r0t %0d cw%0d ch%0d n%0d d%0d", tag,
                         cp_ox, cp_oyt, cp_c0, cp_r0t, cp_cw, cp_ch, cp_n, cp_d);
                $display("INFO %0s ref ox%0d oyt%0d c0 %0d r0t %0d cw%0d ch%0d n%0d d%0d", tag,
                         OX, 480 - DH - PRE, C0, h - CH - R0B, CW, CH, N, D);
            end
            // S3 像素数
            dda_rows(N, D, CH);
            if (n_pix_p1 != 0 || n_pix - n_pix_p1 != CW * rows2) begin
                fail("S3 pixel counts (header pass / window)");
                $display("INFO %0s pass1 %0d exp 0, pass2 %0d exp %0d", tag, n_pix_p1, n_pix - n_pix_p1, CW * rows2);
            end
            if (!complete_sys[bank]) fail("S1 bank complete");
            // S1 逐像素
            bad = 0; bad_main = 0; bad_bg = 0;
            // o = 输出流序号（自下而上，与硬件 out_rows 同序）；边框源 = 主图边缘延展
            for (o = 0; o < 480; o = o + 1) begin
                orow = (o < PRE) ? PRE : (o >= PRE + DH) ? PRE + DH - 1 : o;
                for (x = 0; x < 640; x = x + 1) begin
                    if (BLUR) begin
                        xc  = (x < OX) ? OX : (x >= OX + DW) ? OX + DW - 1 : x;
                        src = ref_main(img, orow, xc);
                        vm[x] = (o == 0) ? src : lp16x3(vm[x], src);
                        hh  = (x == 0) ? vm[x] : lp16x3(hh, vm[x]);
                    end
                    inside = (o >= PRE) && (o < PRE + DH) && (x >= OX) && (x < OX + DW);
                    if (inside) e = ref_main(img, o, x);
                    else if (!BLUR) e = 24'd0;
                    else
                        for (c = 0; c < 3; c = c + 1) begin
                            dk = (hh[c*8 +: 8] >> 1) + (hh[c*8 +: 8] >> 3);
                            e[c*8 +: 8] = dk;
                        end
                    got = sdram_mem[bank * IMG_WORDS + (479 - o) * 640 + x];
                    if (got !== e) begin
                        bad = bad + 1;
                        if (inside) bad_main = bad_main + 1; else bad_bg = bad_bg + 1;
                        if (bad < 4) $display("FAIL S1 %0s o%0d x%0d got %h exp %h (%0s)", tag, o, x, got, e, inside ? "main" : "bg");
                    end
                end
            end
            errors = errors + bad;
            $display("INFO %0s %0dx%0d mode %0d: win c0 %0d r0b %0d %0dx%0d N/D %0d/%0d -> %0dx%0d @(%0d,%0d) blur %0d",
                tag, w, h, MODE, C0, R0B, CW, CH, N, D, DW, DH, OX, 480 - DH - PRE, BLUR);
            $display("INFO %0s   sectors pass1 %0d + pass2 %0d / file %0d, bank errors %0d (main %0d bg %0d), FIFO peak %0d, %0d us",
                tag, rd_p1 - d0, n_data_rd - rd_p1, (`SDM.fi_klen(img) + 511) / 512, bad, bad_main, bad_bg, sc_peak, t / 50);
        end
    endtask

    integer q, e0, k, wsum;
    reg [3:0] cs;
    initial begin
        `SDM.fi_mbr = 1; `SDM.fi_part_lba = 32'd8192; `SDM.fi_spc = 8'd64; `SDM.fi_data_clusters = 32'd4000;
        `SDM.fi_cbase = 32'd0; `SDM.fi_frag = 2'd1; `SDM.fi_present = 4'hF;
        `SDM.fi_slot[0] = 9'd3; `SDM.fi_slot[1] = 9'd4; `SDM.fi_slot[2] = 9'd5; `SDM.fi_slot[3] = 9'd6;
        for (q = 0; q < 4; q = q + 1) begin `SDM.fi_fat_fault[q] = 3'd0; `SDM.fi_size_delta[q] = 0; end
        `SDM.fi_any = 1;
        `SDM.fi_any_w[0] = 1920; `SDM.fi_any_h[0] = 1080;      // FILL 宽受限：主体在右侧
        `SDM.fi_any_w[1] = 1080; `SDM.fi_any_h[1] = 1920;      // FIT 竖图：模糊背景
        `SDM.fi_any_w[2] = 1000; `SDM.fi_any_h[2] = 900;       // FILL 高受限：主体靠上
        `SDM.fi_any_w[3] = 640;  `SDM.fi_any_h[3] = 480;       // 640×480：与原来逐位相同
        for (q = 0; q < 4; q = q + 1)
            `SDM.fi_len[q] = 54 + (3 * `SDM.fi_any_w[q] + `SDM.fi_any_w[q] % 4) * `SDM.fi_any_h[q];
        `SDM.fi_subj = 1'b1;
        `SDM.fi_sx[0] = 1500; `SDM.fi_sy[0] = 600; `SDM.fi_sr[0] = 160;
        `SDM.fi_sx[1] = 0;    `SDM.fi_sy[1] = 0;   `SDM.fi_sr[1] = 100000;   // 整幅纹理
        `SDM.fi_sx[2] = 500;  `SDM.fi_sy[2] = 780; `SDM.fi_sr[2] = 90;
        `SDM.fi_sx[3] = 0;    `SDM.fi_sy[3] = 0;   `SDM.fi_sr[3] = 100000;
        #200 rst = 0;
        t = 0; while (!scan_done && t < 10_000_000) begin @(posedge clk); t = t + 1; end
        $display("INFO scan_done image_count %0d", image_count);

        e0 = errors;
        check_smart(6'd0, 2'd0, 0, 1920, 1080, 1, "FILL-wide");
        if ($test$plusargs("ONLY1")) $finish;
        if (C0 != (1920 - CW) / 2 || R0B != 0) fail("S2 FILL window centered (x)");
        check_smart(6'd1, 2'd1, 1, 1080, 1920, 0, "FIT-tall");
        check_smart(6'd2, 2'd2, 2, 1000, 900, 1, "FILL-tall");
        if (C0 != 0 || R0B != (900 - CH) / 2) fail("S2 FILL window centered (y)");
        check_smart(6'd3, 2'd3, 3, 640, 480, 1, "640x480");
        if (C0 != 0 || R0B != 0 || N != 640 || D != 640) fail("S2 640x480 = identity");
        $display("INFO round 1 errors %0d", errors - e0);

        // ---- X1 加载中途取消（第 2 遍）----
        e0 = errors;
        start_load(6'd1, 2'd1);
        t = 0; while (n_pix < 1080 * 10 && t < 20_000_000) begin @(posedge clk); t = t + 1; end
        @(negedge clk); load_cancel = 1; @(negedge clk); load_cancel = 0;
        wait_result;
        if (res != 8'h59) begin fail("X1 cancel during load -> 0x59"); $display("INFO res %h", res); end
        t = 0; while ((fb_busy || !loader_ready) && t < 2_000_000) begin @(posedge clk); t = t + 1; end
        repeat (200) @(posedge clk);
        if (u_sc.busy) fail("X1 scaler idle after cancel");
        $display("INFO X1 cancel errors %0d", errors - e0);

        // ---- 第二轮：极端比例 / 小图（目录项大小在扫描时已定，只换头里的宽高）----
        e0 = errors;
        `SDM.fi_sr[0] = 100000; `SDM.fi_sr[2] = 100000;
        `SDM.fi_any_w[0] = 3000; `SDM.fi_any_h[0] = 690;  check_smart(6'd0, 2'd1, 0, 3000, 690, 0, "FIT-panorama");
        `SDM.fi_any_w[2] = 200;  `SDM.fi_any_h[2] = 150;  check_smart(6'd2, 2'd2, 2, 200, 150, 2, "INT-200x150");
        `SDM.fi_any_w[3] = 300;  `SDM.fi_any_h[3] = 100;  check_smart(6'd3, 2'd3, 3, 300, 100, 2, "INT-300x100");
        `SDM.fi_any_w[3] = 41;   `SDM.fi_any_h[3] = 333;  check_smart(6'd3, 2'd0, 3, 41, 333, 0, "FIT-41x333");
        `SDM.fi_any_w[2] = 700;  `SDM.fi_any_h[2] = 520;  check_smart(6'd2, 2'd1, 2, 700, 520, 1, "FILL-700x520");
        $display("INFO round 2 errors %0d", errors - e0);
        if (burst_bad != 0) fail("S5 writes beyond frame banks");

        if (errors == 0) $display("PASS tb_smart_load");
        else $display("FAIL tb_smart_load errors=%0d", errors);
        $finish;
    end
    initial begin #4_000_000_000; $display("FAIL timeout"); $finish; end
endmodule
