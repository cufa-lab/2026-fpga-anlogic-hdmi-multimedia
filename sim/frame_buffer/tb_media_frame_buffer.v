`timescale 1ns/1ps

module tb_media_frame_buffer;
`ifdef FULL_SIZE
    // 全尺寸：640x480 一张图 307200 字，bank 基址 0/307200/614400/921600，标准 800x525 时序
    localparam integer W = 640, H = 480, WORDS = W * H;
    localparam integer HFP = 16, HSY = 96, HBP = 48, VFP = 10, VSY = 2, VBP = 33;
    localparam integer N_SWITCH = 5, MIN_CHECKED = 8, MIN_WHILE_LOADING = 3;
`else
    localparam integer W = 64, H = 32, WORDS = W * H;
    localparam integer HFP = 4, HSY = 8, HBP = 12, VFP = 2, VSY = 2, VBP = 4;
    localparam integer N_SWITCH = 10, MIN_CHECKED = 14, MIN_WHILE_LOADING = 3;
`endif

    reg mem_clk = 0, mem_clk_sft = 0, pix_clk = 0, rst_n = 0;
    always #12.5 mem_clk = ~mem_clk;                                   // 40MHz
    initial begin #10; forever #12.5 mem_clk_sft = ~mem_clk_sft; end   // 滞后 10ns
    always #20 pix_clk = ~pix_clk;                                     // 25MHz

    // ---------------- SDRAM ----------------
    wire init_done, ref_vld, sdr_busy, ref_req;
    wire wr_en; wire [20:0] wr_addr; wire [3:0] wr_mask; wire [31:0] wr_data;
    wire rd_en; wire [20:0] rd_addr; wire rd_vld; wire [31:0] rd_dout;
    sdram U3(.Clk(mem_clk), .Clk_sft(mem_clk_sft), .Rst_n(rst_n),
        .Sdr_init_done(init_done), .Sdr_init_ref_vld(ref_vld), .Sdr_busy(sdr_busy), .App_ref_req(ref_req),
        .App_wr_en(wr_en), .App_wr_addr(wr_addr), .App_wr_dm(wr_mask), .App_wr_din(wr_data),
        .App_rd_en(rd_en), .App_rd_addr(rd_addr), .Sdr_rd_en(rd_vld), .Sdr_rd_dout(rd_dout));

    // ---------------- loader ----------------
    reg        ld_start = 0; reg [1:0] ld_index = 0;
    reg        px_valid = 0; reg [23:0] px_rgb = 0; reg px_commit = 0; reg [7:0] dec_err = 0;
    wire       ld_busy, ld_done; wire [3:0] img_complete; wire [7:0] mem_err;
    wire       wr_want, wr_grant, wr_busy_w, rd_want, rd_grant, rd_busy_w;
    media_frame_loader #(.IMAGE_WORDS(WORDS), .FIFO_DEPTH(512), .BURST_SIZE(128),
        .STALL_TIMEOUT(65535), .WRITE_V_FLIP(1), .FRAME_WIDTH(W), .FRAME_HEIGHT(H)) u_loader (
        .clk(mem_clk), .rst_n(rst_n), .load_start(ld_start), .image_index(ld_index),
        .pixel_valid(px_valid), .pixel_rgb(px_rgb), .frame_commit(px_commit), .decode_error(dec_err),
        .load_busy(ld_busy), .load_done(ld_done), .image_complete(img_complete), .memory_error(mem_err),
        .controller_init_done(init_done), .controller_busy(sdr_busy), .refresh_active(ref_vld),
        .burst_want(wr_want), .burst_grant(wr_grant), .burst_busy(wr_busy_w),
        .app_wr_en(wr_en), .app_wr_addr(wr_addr), .app_wr_mask(wr_mask), .app_wr_data(wr_data));

    media_sdram_arbiter #(.IDLE_CYCLES(8), .REF_INTERVAL(576)) u_arb (
        .clk(mem_clk), .rst_n(rst_n), .ctrl_busy(sdr_busy),
        .ctrl_init_done(init_done), .ref_vld(ref_vld), .ref_req(ref_req),
        .wr_want(wr_want), .wr_busy(wr_busy_w), .wr_grant(wr_grant),
        .rd_want(rd_want), .rd_busy(rd_busy_w), .rd_grant(rd_grant));

    // ---------------- 视频时序 + reader ----------------
    wire hs, vs, de; wire [11:0] vx, vy;
    video_timing_ctrl #(.H_ACT(W), .H_FP(HFP), .H_SYNC(HSY), .H_BP(HBP),
        .V_ACT(H), .V_FP(VFP), .V_SYNC(VSY), .V_BP(VBP), .ACTIVE_LAST(1)) u_vtc (
        .pclk(pix_clk), .rst_n(rst_n), .hsync(hs), .vsync(vs), .de(de), .x(vx), .y(vy));
    reg vs_q = 1;
    always @(posedge pix_clk) vs_q <= vs;
    wire frame_start = vs_q && !vs;

    reg init_done_pix = 0;
    always @(posedge pix_clk) init_done_pix <= init_done;
    reg  [1:0] active_index = 0;
    wire [23:0] pix_rgb; wire pix_valid; wire underflow;
    wire rd_busy_flag, frame_done; wire [1:0] reader_index; wire [7:0] reader_err;
    media_frame_reader #(.IMAGE_WORDS(WORDS), .FIFO_DEPTH(512), .FIFO_MARGIN(8),
        .BURST_SIZE(128), .BURST_GAP_CYCLES(12), .MAX_INFLIGHT_BURSTS(2)) u_reader (
        .mem_clk(mem_clk), .mem_rst_n(rst_n), .controller_init_done(init_done),
        .controller_busy(sdr_busy), .refresh_active(ref_vld),
        .burst_want(rd_want), .burst_grant(rd_grant), .burst_busy(rd_busy_w),
        .app_rd_en(rd_en), .app_rd_addr(rd_addr), .sdram_rd_valid(rd_vld), .sdram_rd_data(rd_dout),
        .pix_clk(pix_clk), .pix_rst_n(rst_n), .frame_start(frame_start & init_done_pix),
        .active_index(active_index), .pixel_consume(de & init_done_pix),
        .pixel_rgb(pix_rgb), .pixel_valid(pix_valid), .underflow(underflow),
        .reader_busy(rd_busy_flag), .frame_read_done(frame_done), .reader_index(reader_index),
        .reader_error(reader_err));

    // ---------------- 期望值 ----------------
    function [23:0] pat(input [1:0] b, input [18:0] s); pat = {b, 3'b101, s} ^ 24'h00A5C3; endfunction
    // 显示第 r 行第 c 列 ← 流像素 s = (H-1-r)*W + c（BMP 自下而上，loader 翻转写）
    function [23:0] expect_pix(input [1:0] b, input integer r, input integer c); expect_pix = pat(b, (H - 1 - r) * W + c); endfunction

    integer errors = 0;
    task fail(input [8*80-1:0] msg); begin errors = errors + 1; if (errors < 20) $display("FAIL %0s @%0t", msg, $time); end endtask

    // ---------------- 显示逐像素检查（pix 域）----------------
    reg allow_rd_err = 0; integer seen_underflow = 0, seen_overrun = 0;   // T8
    reg [3:0] bank_ready = 0;          // tb 视角：已完整加载、此后没被重写
    reg [3:0] frame_checkable;
    reg [1:0] frame_bank;
    integer   pix_idx = 0, frames = 0, frames_checked = 0, frames_while_loading = 0;
    reg       frame_ok, frame_live = 0, loading_during_frame, fs_d = 0;
    event     fs_ev;                   // frame_start 沿：tb 在这一拍用非阻塞赋值提交 active_index（同控制器）
    always @(posedge pix_clk) begin
        if (frame_start && init_done_pix) begin
            if (frame_live && frame_checkable[frame_bank]) begin
                if (pix_idx != WORDS) fail("frame pixel count");
                if (frame_ok && pix_idx == WORDS) begin
                    frames_checked = frames_checked + 1;
                    if (loading_during_frame) frames_while_loading = frames_while_loading + 1;
                end
            end
            frames = frames + 1;
            frame_live = 1; pix_idx = 0; frame_ok = 1; loading_during_frame = 0;
            frame_checkable = allow_rd_err ? 4'b0000 : bank_ready;
            -> fs_ev;
        end
        if (fs_d) frame_bank = active_index;    // 与 reader 同一拍锁存（frame_start 的下一拍）
        fs_d <= frame_start && init_done_pix;
        if (ld_busy) loading_during_frame = 1;
        if (de && init_done_pix && frame_live) begin
            if (frame_checkable[frame_bank]) begin
                if (!pix_valid) begin fail("display underflow (pixel_valid=0 on de)"); frame_ok = 0; end
                else if (pix_rgb !== expect_pix(frame_bank, pix_idx / W, pix_idx % W)) begin
                    if (frame_ok) $display("FAIL bank %0d pixel %0d (r%0d c%0d) got %h exp %h @%0t", frame_bank, pix_idx,
                        pix_idx / W, pix_idx % W, pix_rgb, expect_pix(frame_bank, pix_idx / W, pix_idx % W), $time);
                    frame_ok = 0; errors = errors + 1;
                end
            end
            pix_idx = pix_idx + 1;
        end
    end

    // ---------------- burst / 仲裁检查（mem 域）----------------
    integer wr_run = 0, rd_run = 0, wr_bursts = 0, rd_bursts = 0, bad_runs = 0;
    integer since_wr = 1000, since_rd = 1000, min_wr_to_rd = 1000, min_rd_to_wr = 1000;
    reg wr_en_q = 0, rd_en_q = 0;
    realtime last_phy_ref = -1.0e9;
    realtime prev_phy_ref_gap = 0.0;
    integer  ref_slot_miss = 0, ref_since_init = 0;
    realtime t_init_ref = -1.0;
    wire [3:0] phy_cmd = {1'b0, U3.SDR_RAS, U3.SDR_CAS, U3.SDR_WE};
    always @(posedge mem_clk) begin
        if (phy_cmd == 4'b0001) begin
            if ($realtime - last_phy_ref < 60.0)
                $display("TRACE double REF @%0t dt=%0.1fns previous_gap=%0.2fus ref_vld=%b busy=%b wr=%b rd=%b wr_busy=%b rd_busy=%b quiet=%0d wr_want=%b rd_want=%b",
                    $time, $realtime - last_phy_ref, prev_phy_ref_gap / 1000.0, ref_vld, sdr_busy, wr_en, rd_en,
                    wr_busy_w, rd_busy_w, u_arb.quiet, wr_want, rd_want);
            // 定时刷新检查：SDRAM 要求 64ms 内 4096 次（平均 15.625us）。单次可因等 burst 结束推迟，
            // 但不能整槽丢失（间隔 >= 31.25us），累计次数也不能落后（见结尾汇总）
            if (last_phy_ref > 0.0 && init_done && $realtime - last_phy_ref >= 31250.0) ref_slot_miss = ref_slot_miss + 1;
            if (init_done) begin
                if (t_init_ref < 0.0) t_init_ref = $realtime;
                ref_since_init = ref_since_init + 1;
            end
            prev_phy_ref_gap = $realtime - last_phy_ref;
            last_phy_ref = $realtime;
        end
        if (wr_en && rd_en) fail("App_wr_en and App_rd_en overlap");
        if (wr_en && !wr_en_q && since_rd < min_rd_to_wr) min_rd_to_wr = since_rd;
        if (rd_en && !rd_en_q && since_wr < min_wr_to_rd) min_wr_to_rd = since_wr;
        since_wr = wr_en ? 0 : since_wr + 1;
        since_rd = rd_en ? 0 : since_rd + 1;
        if (wr_en) wr_run = wr_run + 1;
        else if (wr_en_q) begin wr_bursts = wr_bursts + 1; if (wr_run != 128) begin bad_runs = bad_runs + 1; $display("INFO wr burst len %0d", wr_run); end wr_run = 0; end
        if (rd_en) rd_run = rd_run + 1;
        else if (rd_en_q) begin rd_bursts = rd_bursts + 1; if (rd_run != 128) begin bad_runs = bad_runs + 1; $display("INFO rd burst len %0d", rd_run); end rd_run = 0; end
        wr_en_q <= wr_en; rd_en_q <= rd_en;
    end
    always @(posedge pix_clk) if (underflow) begin if (allow_rd_err) seen_underflow = seen_underflow + 1; else fail("reader underflow flag"); end
    always @(posedge mem_clk) if (init_done && reader_err != 8'h00) begin
        if (allow_rd_err && reader_err == 8'h61) seen_overrun = seen_overrun + 1; else fail("reader_error != 0");
    end
    always @(posedge mem_clk) begin
        if (wr_en && !wr_en_q && wr_addr[1:0] != 2'b00) fail("write burst start not 4-aligned");
        if (rd_en && !rd_en_q && rd_addr[1:0] != 2'b00) fail("read burst start not 4-aligned");
    end

    // ---------------- loader 激励（mem 域，像素间随机 0..7 拍空闲）----------------
    integer seed = 32'h1234_5678;
    task feed_frame(input [1:0] b, input integer n, input integer commit_at, input integer err_at);
        integer s, g; begin
            @(posedge mem_clk); #1 ld_start = 1; ld_index = b;
            @(posedge mem_clk); #1 ld_start = 0;
            for (s = 0; s < n; s = s + 1) begin
                g = $unsigned($random(seed)) % 8;
                repeat (g) @(posedge mem_clk);
                #1 px_valid = 1; px_rgb = pat(b, s); px_commit = (s == commit_at); dec_err = (s == err_at) ? 8'h56 : 8'h00;
                @(posedge mem_clk); #1 px_valid = 0; px_commit = 0; dec_err = 0;
            end
        end endtask
    // load_done 可能在喂像素途中就出现（坏帧），用监视器计数
    integer done_cnt = 0, done_seen = 0; reg [7:0] done_code = 0;
    always @(posedge mem_clk) if (ld_done) begin done_cnt = done_cnt + 1; done_code = mem_err; end
    task wait_done(output [7:0] code);
        begin
            while (done_cnt == done_seen) @(posedge mem_clk);
            done_seen = done_cnt; code = done_code;
            @(posedge mem_clk);
        end endtask

    reg [7:0] code;
    integer k, f;
    initial begin
        #300 rst_n = 1;
        wait (init_done);
        $display("INFO SDRAM init_done @%0t", $time);
        // T1/T2：依次加载 4 个 bank；加载 bank k 时显示 bank k-1（读写并发）
        for (k = 0; k < 4; k = k + 1) begin
            if (k > 0) begin @(fs_ev); active_index <= k - 1; end
            feed_frame(k[1:0], WORDS, WORDS - 1, -1);
            wait_done(code);
            if (code != 8'h00) begin $display("FAIL load bank %0d memory_error %h", k, code); errors = errors + 1; end
            if (img_complete[k] !== 1'b1) begin $display("FAIL image_complete[%0d] not set (%b)", k, img_complete); errors = errors + 1; end
            bank_ready[k] = 1'b1;
            $display("INFO bank %0d loaded @%0t image_complete=%b", k, $time, img_complete);
        end
        // 让最后一个 bank 至少显示一帧
        repeat (2) @(fs_ev);
        // T3：每帧切 bank（控制器在 frame_start 同拍提交）
        for (f = 0; f < N_SWITCH; f = f + 1) begin
            @(fs_ev);
            active_index <= f[1:0];
        end
        repeat (2) @(fs_ev);
        // T6：错误码（加载 bank 3 的各种坏帧；bank 3 先标记为不可检查）
        @(fs_ev); active_index <= 0; bank_ready[3] = 1'b0;
        feed_frame(2'd3, WORDS - 100, WORDS - 101, -1);          // 短帧
        wait_done(code); if (code != 8'h45) begin $display("FAIL short frame code %h", code); errors = errors + 1; end
        if (img_complete[3] !== 1'b0) begin $display("FAIL image_complete[3] should clear on failed load"); errors = errors + 1; end
        feed_frame(2'd3, 500, -1, 300);                          // 解码错
        wait_done(code); if (code != 8'h44) begin $display("FAIL decode error code %h", code); errors = errors + 1; end
        repeat (40) @(posedge mem_clk);
        #1 px_valid = 1; @(posedge mem_clk); #1 px_valid = 0;    // 未激活收到像素
        @(posedge mem_clk); if (mem_err != 8'h41) begin $display("FAIL not-active code %h", mem_err); errors = errors + 1; end
        feed_frame(2'd3, WORDS + 1, WORDS, -1);                  // 超量
        wait_done(code); if (code != 8'h43) begin $display("FAIL pixel count code %h", code); errors = errors + 1; end
        // 坏帧之后 bank 3 重新正常加载，仍能显示正确
        feed_frame(2'd3, WORDS, WORDS - 1, -1);
        wait_done(code); if (code != 8'h00) begin $display("FAIL reload bank 3 code %h", code); errors = errors + 1; end
        bank_ready[3] = 1'b1;
        @(fs_ev); active_index <= 3;
        repeat (3) @(fs_ev);
        // T8：帧中途卡住 reader（仲裁不放行），跨过下一个 frame_start 后松开
        @(fs_ev); allow_rd_err = 1;
        @(fs_ev);                                   // 从这一帧起不做逐像素检查
        repeat ((VSY + VBP + H / 2) * (W + HFP + HSY + HBP)) @(posedge pix_clk);
        force u_arb.rd_grant = 1'b0;
        @(fs_ev); repeat (40) @(posedge pix_clk);
        release u_arb.rd_grant;
        repeat (3) @(fs_ev); @(posedge pix_clk); allow_rd_err = 0;
        repeat (3) @(fs_ev);
        if (seen_underflow == 0 || seen_overrun == 0) begin $display("FAIL T8 underflow %0d overrun %0d not detected", seen_underflow, seen_overrun); errors = errors + 1; end

        $display("INFO frames %0d, fully checked %0d (while loading %0d)", frames, frames_checked, frames_while_loading);
        $display("INFO wr bursts %0d, rd bursts %0d, non-128 bursts %0d", wr_bursts, rd_bursts, bad_runs);
        $display("INFO T8 underflow frames %0d, overrun cycles %0d", seen_underflow, seen_overrun);
        $display("INFO min gap wr->rd %0d, rd->wr %0d cycles", min_wr_to_rd, min_rd_to_wr);
        if (frames_checked < MIN_CHECKED) begin $display("FAIL too few checked frames"); errors = errors + 1; end
        if (frames_while_loading < MIN_WHILE_LOADING) begin $display("FAIL too few frames checked during loading"); errors = errors + 1; end
        if (bad_runs != 0) begin $display("FAIL burst length not 128"); errors = errors + 1; end
        $display("INFO SDRAM auto refresh: %0d commands, max gap %0.2f us, lost slots %0d", U3.sdram.ref_count, U3.sdram.ref_max_gap_ns / 1000.0, ref_slot_miss);
        $display("INFO refresh since init: %0d commands in %0.1f us, need >= %0d (15.625us each)",
            ref_since_init, ($realtime - t_init_ref) / 1000.0, $rtoi(($realtime - t_init_ref) / 15625.0));
        if (ref_slot_miss != 0) begin $display("FAIL refresh slot lost (gap >= 31.25us)"); errors = errors + 1; end
        if (ref_since_init < $rtoi(($realtime - t_init_ref) / 15625.0)) begin $display("FAIL refresh rate below 4096/64ms"); errors = errors + 1; end
        if (min_wr_to_rd < 20 || min_rd_to_wr < 20) begin $display("FAIL turnaround gap < 20"); errors = errors + 1; end
        if (errors == 0) $display("PASS tb_media_frame_buffer");
        else $display("FAIL tb_media_frame_buffer errors=%0d", errors);
        $finish;
    end
`ifdef FULL_SIZE
    initial begin #1_500_000_000; $display("FAIL timeout"); $finish; end
`else
    initial begin #60_000_000; $display("FAIL timeout"); $finish; end
`endif
endmodule
