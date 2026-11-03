`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// tb_k1_chain ——  讲解字幕整条链路（与顶层同一接法，sys 50MHz / pix 25MHz 真双时钟）
//   卡：sim/k1/card.hex（gen_card.py：真 FAT32 + 真 HZK32 + TRACK1.PCM/LRC/VOC），sd_card_top 用扇区级替身
//   sys：真 sd_card_bmp（扫描器 aux 清单 + locator B 口 + 仲裁器 A/B/C 三口）+ wav_playlist_ctrl + 背景乐播放器
//        + script_loader + 人声播放器（C 口分时）+ 讲解 RAM（字形用真 script_glyph_bram32k / EG_PHY_BRAM32K 模型，
//        去重表 script_dedup_ram）
//   pix：audio_sample_tick（2 倍速 96k，讲解时钟按人声样本走，整体同比加速）→ 两个播放器 → narration_audio_mixer
//        video_timing_ctrl → rgb_video_adapter → osd_layer_mixer(ext + txt) ← script_osd
//   检查（本 tb）：
//     C1 扫描：narr_aux_valid = {HZK32, VOC1, LRC1} = bit 32 / 16 / 0，script_error 始终 0
//     C2 装稿：ready 翻转、人声开播（epoch），loader 回到 RUN；写入的格 / 行表与 LRC 一致（chk_chain.py 逐项比字形）
//     C3 人声：播出的真实样本 = 卡上 TRACK1.VOC 逐个相同（含循环回绕），开播后不下溢；背景乐不下溢
//     C4 讲解时钟：now 与人声样本数 / 240 一致；SW2 暂停期间 now 不动、人声不出样本、背景乐照常
//     C5 混音：说话时 ducking=1 且背景乐增益压到一半；关人声（KEY3 长按）后人声增益收到 0
//   像素：选定帧转储到 sim/out_k1c/，chk_chain.py 用写口影子 RAM 作参考跑 chk_osd 的逐像素模型（经真 BRAM32K 读出）
// -----------------------------------------------------------------------------
module tb_k1_chain;
`include "sim/k1/card_meta.vh"
    reg sys_clk = 0, pix_clk = 0, rst_n = 0;
    always #10 sys_clk = ~sys_clk;
    always #20 pix_clk = ~pix_clk;

    // ---------------- SD ----------------
    wire scan_done, scan_all_done;
    wire [6:0] image_count;
    wire [7:0] fs_error;
    wire [15:0] trk_valid;
    wire        info_req, info_valid;
    wire [3:0]  info_idx;
    wire [31:0] info_start, info_total;
    wire        a_req, a_busy, a_valid, a_done;
    wire [31:0] a_lba;
    wire [7:0]  a_byte, a_error;
    wire [8:0]  a_bidx;
    wire [33:0] narr_aux_valid;
    wire        narr_info_req, narr_info_valid;
    wire [5:0]  narr_info_idx;
    wire [31:0] narr_info_size;
    wire        narr_sector_req, narr_sector_busy, narr_sector_valid, narr_sector_done;
    wire [31:0] narr_sector_lba;
    wire [7:0]  narr_sector_error;

    sd_card_bmp #(
        .CLK_FREQ_HZ (50_000_000),
        .BMP_WIDTH   (16'd32),
        .BMP_HEIGHT  (16'd24)
    ) u_sd (
        .clk(sys_clk), .rst_n(rst_n), .ingest_enable(1'b1), .sd_ready(),
        .scan_done(scan_done), .scan_all_done(scan_all_done), .image_count(image_count),
        .loader_ready(), .load_request(1'b0), .load_image_idx(6'd0), .load_buffer_idx(2'd0),
        .load_done(), .loaded_image_idx(), .loaded_buffer_idx(), .load_error(), .load_cancel(1'b0),
        .bmp_state_code(), .sd_reinit_count(), .fs_error(fs_error),
        .fb_load_start(), .fb_load_bank(), .pixel_valid(), .pixel_rgb(), .frame_commit(), .fb_load_abort(),
        .fb_done(1'b0), .fb_error(8'd0), .fb_busy(),
        .fb_load_image(), .src_geom_valid(), .src_width(), .src_height(), .src_hold(1'b0), .src_progress(1'b0),
        .audio_track_valid(trk_valid), .audio_info_req(info_req), .audio_info_idx(info_idx),
        .audio_info_valid(info_valid), .audio_info_start_lba(info_start), .audio_info_total_sectors(info_total),
        .audio_sector_req(a_req), .audio_sector_lba(a_lba), .audio_sector_busy(a_busy),
        .audio_sector_valid(a_valid), .audio_sector_byte(a_byte), .audio_sector_byte_index(a_bidx),
        .audio_sector_done(a_done), .audio_sector_error(a_error),
        .narr_aux_valid(narr_aux_valid), .narr_info_req(narr_info_req), .narr_info_idx(narr_info_idx),
        .narr_info_valid(narr_info_valid), .narr_info_size(narr_info_size),
        .narr_sector_req(narr_sector_req), .narr_sector_lba(narr_sector_lba), .narr_sector_busy(narr_sector_busy),
        .narr_sector_valid(narr_sector_valid), .narr_sector_done(narr_sector_done), .narr_sector_error(narr_sector_error),
        .SD_nCS(), .SD_DCLK(), .SD_MOSI(), .SD_MISO(1'b1)
    );

    reg enable_app = 1'b0;                    // = player_enable_fast（首图提交）；本 tb 扫描完即放行
    // ---------------- 背景乐 ----------------
    wire        play_enable, wr_idle, wr_done, wr_error;
    wire [31:0] pl_start, pl_total;
    wav_playlist_ctrl #(.LOOP(1)) u_pl (
        .clk(sys_clk), .rst_n(rst_n), .enable(enable_app), .track_valid(trk_valid), .select(5'd0),
        .info_req(info_req), .info_idx(info_idx), .info_valid(info_valid),
        .info_start_lba(info_start), .info_total_sectors(info_total),
        .play_enable(play_enable), .start_lba(pl_start), .total_sectors(pl_total),
        .wr_idle(wr_idle), .wr_done(wr_done), .wr_error(wr_error), .track(), .playing(), .switching()
    );
    wire               wav_tick;
    audio_sample_tick #(.CLOCK_HZ(25_000_000), .SAMPLE_HZ(96_000)) u_tick (.clk(pix_clk), .rst_n(rst_n), .tick(wav_tick));
    wire signed [15:0] m_s;
    wire               m_v, m_r, m_active;
    wav_stream_player #(.ADDR_WIDTH(12), .SAMPLES_PER_SECTOR(256)) u_bg (
        .wr_clk(sys_clk), .wr_rst_n(rst_n), .play_enable(play_enable),
        .start_lba(pl_start), .total_sectors(pl_total),
        .sd_req(a_req), .sd_lba(a_lba), .sd_busy(a_busy), .sd_valid(a_valid), .sd_byte(a_byte),
        .sd_byte_index(a_bidx), .sd_done(a_done), .sd_error(a_error),
        .wr_idle(wr_idle), .wr_done(wr_done), .wr_error(wr_error),
        .rd_clk(pix_clk), .rd_rst_n(rst_n), .sample_tick(wav_tick),
        .sample_out(m_s), .sample_valid(m_v), .sample_ready(m_r),
        .sample_counter(), .stream_active(m_active), .stream_end(), .buffer_low()
    );

    // ---------------- 讲解 sys 侧 ----------------
    wire        voc_sd_req, voc_sd_busy, voc_sd_valid, voc_sd_done;
    wire [31:0] voc_sd_lba;
    wire        voc_play_enable, voc_wr_idle, voc_wr_done, voc_wr_error;
    wire [31:0] voc_start_lba, voc_total_sectors;
    wire        g_we, g_park, c_we, l_we;
    wire [2:0]  g_blk;
    wire [12:0] g_addr;
    wire        dd_we, dd_re;
    wire [11:0] dd_addr;
    wire [7:0]  dd_data, dd_q;
    wire [15:0] g_data;
    wire        g_re;
    wire [15:0] g_q;
    wire [8:0]  c_addr, l_addr;
    wire [9:0]  c_data;
    wire [17:0] l_data;
    wire        rel_req_t, rel_ack_t, ready_t, epoch_t;
    wire [7:0]  script_error;
    script_loader #(.MAX_W(72), .ROW_W(72)) u_ld (
        .clk(sys_clk), .rst_n(rst_n), .enable(scan_all_done & enable_app), .aux_valid(narr_aux_valid), .select(5'd0),
        .loc_req(narr_info_req), .loc_idx(narr_info_idx), .loc_valid(narr_info_valid),
        .loc_lba(info_start), .loc_size(narr_info_size),
        .sec_req(narr_sector_req), .sec_lba(narr_sector_lba), .sec_busy(narr_sector_busy),
        .sec_valid(narr_sector_valid), .sec_byte(a_byte), .sec_byte_index(a_bidx),
        .sec_done(narr_sector_done), .sec_error(narr_sector_error),
        .voc_sd_req(voc_sd_req), .voc_sd_lba(voc_sd_lba), .voc_sd_busy(voc_sd_busy), .voc_sd_valid(voc_sd_valid),
        .voc_sd_done(voc_sd_done), .voc_play_enable(voc_play_enable), .voc_start_lba(voc_start_lba),
        .voc_total_sectors(voc_total_sectors), .voc_wr_idle(voc_wr_idle), .voc_wr_done(voc_wr_done),
        .voc_wr_error(voc_wr_error),
        .glyph_we(g_we), .glyph_park(g_park), .glyph_blk(g_blk), .glyph_addr(g_addr), .glyph_data(g_data),
        .glyph_re(g_re), .glyph_q(g_q),
        .dd_we(dd_we), .dd_re(dd_re), .dd_addr(dd_addr), .dd_data(dd_data), .dd_q(dd_q),
        .code_we(c_we), .code_addr(c_addr), .code_data(c_data),
        .line_we(l_we), .line_addr(l_addr), .line_data(l_data),
        .rel_req_t(rel_req_t), .rel_ack_t(rel_ack_t), .ready_t(ready_t), .epoch_t(epoch_t),
        .script_error(script_error)
    );
    wire        l_re, c_re, gl_re;
    wire [8:0]  l_raddr, c_raddr;
    wire [12:0] gl_raddr;
    wire [17:0] l_rdata;
    wire [9:0]  c_rdata;
    wire [63:0] gl_rdata;
    script_glyph_bram32k u_glyph (
        .sys_clk(sys_clk), .sys_we(g_we), .sys_park(g_park), .sys_blk(g_blk), .sys_re(g_re), .sys_addr(g_addr),
        .sys_data(g_data), .sys_q(g_q),
        .pix_clk(pix_clk), .pix_re(gl_re), .pix_addr(gl_raddr), .pix_q(gl_rdata));
    script_dedup_ram u_dedup (.clk(sys_clk), .we(dd_we), .re(dd_re), .addr(dd_addr), .data(dd_data), .q(dd_q));
    script_dc_ram #(.AW(9), .DW(10)) u_code (.wclk(sys_clk), .we(c_we), .waddr(c_addr), .wdata(c_data),
        .rclk(pix_clk), .re(c_re), .raddr(c_raddr), .rdata(c_rdata));
    script_dc_ram #(.AW(9), .DW(18)) u_line (.wclk(sys_clk), .we(l_we), .waddr(l_addr), .wdata(l_data),
        .rclk(pix_clk), .re(l_re), .raddr(l_raddr), .rdata(l_rdata));

    // ---------------- 人声 + 混音（pix）----------------
    wire mfs;                                  // osd_layer_mixer 帧起点（混音器增益也按帧更新）
    reg pause = 0, page_on = 0, marq_hide = 0;
    reg voc_div = 0;
    always @(posedge pix_clk) if (wav_tick) voc_div <= ~voc_div;
    wire voc_tick = wav_tick && voc_div && !pause;
    wire signed [15:0] v_s;
    wire        v_v, v_active;
    wire [31:0] v_cnt;
    wav_stream_player #(.ADDR_WIDTH(10), .SAMPLES_PER_SECTOR(256)) u_voc (
        .wr_clk(sys_clk), .wr_rst_n(rst_n), .play_enable(voc_play_enable),
        .start_lba(voc_start_lba), .total_sectors(voc_total_sectors),
        .sd_req(voc_sd_req), .sd_lba(voc_sd_lba), .sd_busy(voc_sd_busy), .sd_valid(voc_sd_valid), .sd_byte(a_byte),
        .sd_byte_index(a_bidx), .sd_done(voc_sd_done), .sd_error(narr_sector_error),
        .wr_idle(voc_wr_idle), .wr_done(voc_wr_done), .wr_error(voc_wr_error),
        .rd_clk(pix_clk), .rd_rst_n(rst_n), .sample_tick(voc_tick),
        .sample_out(v_s), .sample_valid(v_v), .sample_ready(1'b1),
        .sample_counter(v_cnt), .stream_active(v_active), .stream_end(), .buffer_low()
    );
    wire signed [15:0] o_s;
    wire o_v, ducking;
    narration_audio_mixer u_mix_a (
        .clk(pix_clk), .rst_n(rst_n), .frame_start(mfs), .volume(8'd255), .fade_gain(8'd255),
        .pause(pause), .voice_active(v_active),
        .m_valid(m_v), .m_ready(m_r), .m_sample(m_s), .v_valid(v_v), .v_ready(), .v_sample(v_s),
        .out_valid(o_v), .out_ready(1'b1), .out_sample(o_s), .ducking(ducking));

    // ---------------- 视频 ----------------
    wire raw_hs, raw_vs, raw_de;
    video_timing_ctrl #(.H_ACT(640), .H_FP(16), .H_SYNC(96), .H_BP(48), .V_ACT(480), .V_FP(10), .V_SYNC(2), .V_BP(33),
        .ACTIVE_LAST(1), .VS_ALIGN_HSYNC(1)) u_timing (.pclk(pix_clk), .rst_n(rst_n), .hsync(raw_hs), .vsync(raw_vs),
        .de(raw_de), .x(), .y());
    wire hs, vs, de;
    rgb_video_adapter u_adapter (.pclk(pix_clk), .rst_n(rst_n), .hs_in(raw_hs), .vs_in(raw_vs), .de_in(raw_de),
        .rgb_in(24'h0), .video_clk(), .video_hs(hs), .video_vs(vs), .video_de(de), .video_rgb());
    reg [9:0] tx = 0, ty = 0;
    reg tde_q = 0;
    always @(posedge pix_clk) begin
        tde_q <= de; tx <= de ? tx + 10'd1 : 10'd0;
        if (!vs) ty <= 0; else if (tde_q && !de) ty <= ty + 10'd1;
    end
    wire [23:0] bg_rgb = de ? {8'd30 + {1'b0, ty[8:2]}, 8'd90 + {2'd0, tx[9:4]}, 8'd140} : 24'h0;
    wire [9:0]  mx, my;
    wire        s_hit, page_active, t_hit;
    wire [23:0] s_color, final_rgb, t_color;
    wire [7:0]  s_alpha, t_alpha;
    script_osd u_osd (
        .clk(pix_clk), .rst_n(rst_n), .frame_start(mfs), .de(de), .pixel_x(mx), .pixel_y(my), .rgb_in(bg_rgb),
        .rel_req_t(rel_req_t), .rel_ack_t(rel_ack_t), .ready_t(ready_t), .epoch_t(epoch_t),
        .line_re(l_re), .line_raddr(l_raddr), .line_rdata(l_rdata),
        .code_re(c_re), .code_raddr(c_raddr), .code_rdata(c_rdata),
        .glyph_re(gl_re), .glyph_raddr(gl_raddr), .glyph_rdata(gl_rdata),
        .voice_cnt0(v_cnt[0]), .tick48(wav_tick), .pause(pause), .page_on(page_on),
        .hide(1'b0), .marq_hide(marq_hide), .title_show(1'b0), .bright_gain(9'h100), .ext_hit(s_hit), .ext_color(s_color), .ext_alpha(s_alpha),
        .txt_hit(t_hit), .txt_color(t_color), .txt_alpha(t_alpha), .page_active(page_active));
    osd_layer_mixer #(.TS_ENABLE(0), .SUBTITLE_EN(0), .PARAM_Y(82), .PARAM_HOLD_FRAMES(240)) u_mix_v (
        .clk(pix_clk), .rst_n(rst_n), .vsync(vs), .de(de), .rgb_in(bg_rgb),
        .subtitle_enable(1'b0), .subtitle_text(96'd0), .subtitle_length(6'd0),
        .brightness(8'd255), .volume(8'd255), .period_tenths(9'd25), .auto_enabled(1'b0),
        .res_valid(1'b1), .res_width(12'd32), .res_height(12'd24), .param_show(1'b0), .param_hide(page_active),
        .ext_hit(s_hit), .ext_color(s_color), .ext_alpha(s_alpha),
        .txt_hit(t_hit), .txt_color(t_color), .txt_alpha(t_alpha),
        .pix_x(mx), .pix_y(my), .pix_frame_start(mfs), .rgb_out(final_rgb), .overlay_active());

    // ================= 检查 =================
    integer errors = 0;
    reg [7:0] card [0:CARD_BYTES-1];
    initial $readmemh("sim/k1/card.hex", card);
    localparam integer VLOOP = VOC_SECTORS * 256;      // 播放器只播整扇区（尾部不足 1 扇区丢弃，同 PCM）
    function signed [15:0] voc_exp; input integer n; integer b;
        begin b = (VOC_LBA * 512) + 2 * (n % VLOOP); voc_exp = {card[b + 1], card[b]}; end endfunction

    // 影子 RAM（写口旁听）→ 结束时写文件给 chk_chain.py（字形按 script_glyph_bram32k 的字节写语义）
    reg [63:0] sh_g [0:8191];
    reg [9:0]  sh_c [0:511];
    reg [17:0] sh_l [0:511];
    integer i;
    initial begin
        for (i = 0; i < 8192; i = i + 1) sh_g[i] = 64'h0;
        for (i = 0; i < 512; i = i + 1) begin sh_c[i] = 10'h0; sh_l[i] = 18'h0; end
    end
    always @(posedge sys_clk) begin
        if (g_we) begin
            if (g_park) sh_g[g_addr][15:0] <= g_data;
            else        sh_g[g_addr][63 - 8 * g_blk -: 8] <= g_blk[0] ? g_data[7:0] : g_data[15:8];
        end
        if (c_we) sh_c[c_addr] <= c_data;
        if (l_we) sh_l[l_addr] <= l_data;
    end

    // C1 script_error 必须一直为 0
    always @(posedge sys_clk) if (rst_n && script_error != 8'h00) begin
        errors = errors + 1; $display("FAIL script_error=%02h @%0t", script_error, $time); #1000 $finish;
    end

    // C3 人声样本
    integer v_n = 0, v_bad = 0, v_under = 0, v_started = 0;
    always @(posedge pix_clk) if (v_v) begin
        if (u_voc.cur_real) begin
            if (v_s !== voc_exp(v_n)) begin
                v_bad = v_bad + 1;
                if (v_bad < 6) $display("FAIL voice sample %0d got %0d exp %0d", v_n, v_s, voc_exp(v_n));
            end
            v_n = v_n + 1; v_started = 1;
        end else if (v_started && v_active && (v_n % VLOOP) != 0)
            v_under = v_under + 1;
    end
    integer m_n = 0, m_under = 0, m_started = 0;
    always @(posedge pix_clk) if (m_v && m_r) begin
        if (u_bg.cur_real) begin m_n = m_n + 1; m_started = 1; end
        else if (m_started && m_active && (m_n % (24 * 256)) != 0) m_under = m_under + 1;   // 曲尾重播的空档不算
    end

    // 帧 / 状态快照 / 转储（格式同 tb_script_osd，chk_osd 可直接用）
    integer frame = 0, fst, fpx = 0;
    reg dumping = 0;
    reg [1023:0] dump_mask = 0;
    always @(posedge pix_clk) if (mfs) frame <= frame + 1;
    always @(posedge pix_clk) if (rst_n && u_osd.tl_done) begin
        #1;
        $fwrite(fst, "%0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d\n",
            frame, u_osd.now, u_osd.loaded, u_osd.has_voice, u_osd.cur, u_osd.gc, u_osd.s_w, u_osd.ta, u_osd.ia,
            u_osd.sa, u_osd.pa, u_osd.xp, u_osd.off, u_osd.rr, u_osd.page_y, u_osd.ptop, u_osd.psub, u_osd.lvl_t_a,
            u_osd.lvl_s_a, u_osd.releasing, u_osd.rel_ack_t, pause, marq_hide, 0, u_osd.line_end, u_osd.rows_n,
            u_osd.gend, u_osd.s0, u_osd.t_w, u_osd.t_start, u_osd.i_w, u_osd.i_start, u_osd.itim, u_osd.ended,
            u_osd.hx, u_osd.t_cur, u_osd.xd, u_osd.tl, page_active, v_n);
    end
    always @(negedge pix_clk) if (rst_n && dumping && de && my < 10'd480)
        $fwrite(fpx, "%06h %06h %0d %06h %02h %0d %06h %02h\n", final_rgb, u_mix_v.stage1_rgb,
                !s_hit ? 0 : u_osd.ic_px ? 2 : u_osd.ic_bg ? 3 : u_osd.dc_px ? 4 : 5,
                s_hit ? s_color : 24'h0, s_hit ? s_alpha : 8'h0,
                t_hit, t_hit ? t_color : 24'h0, t_hit ? t_alpha : 8'h0);
    reg [8*40-1:0] fname;
    always @(posedge pix_clk) if (mfs) begin
        if (dumping) begin $fclose(fpx); dumping = 0; end
        if (dump_mask[frame + 1]) begin
            $sformat(fname, "sim/out_k1c/frame_%03d.txt", frame + 1);
            fpx = $fopen(fname, "w"); dumping = 1;
        end
    end

    task wait_frames; input integer n; integer f0; begin f0 = frame; while (frame < f0 + n) @(posedge pix_clk); end endtask

    integer f_ready, now0, vn0, mn0, k;
    reg rdy_q;
    initial begin
        fst = $fopen("sim/out_k1c/state.txt", "w");
        repeat (20) @(posedge sys_clk); rst_n = 1;
        // ---- C1 扫描 ----
        wait (scan_all_done);
        $display("INFO scan_all_done @%0t aux=%b fs_error=%02h images=%0d tracks=%b", $time, narr_aux_valid, fs_error, image_count, trk_valid);
        @(posedge sys_clk); @(posedge sys_clk);
        if (narr_aux_valid !== ((34'd1 << 32) | (34'd1 << 16) | 34'd1)) begin errors = errors + 1; $display("FAIL aux_valid=%b", narr_aux_valid); $finish; end
        enable_app = 1;
        // ---- C2 装稿 ----
        rdy_q = ready_t;
        k = 0;
        while (ready_t == rdy_q && k < 10_000_000) begin @(posedge sys_clk); k = k + 1; end
        if (ready_t == rdy_q) begin errors = errors + 1; $display("FAIL no ready within 200 ms"); $finish; end
        $display("INFO ready @%0t (glyph writes done, header written)", $time);
        f_ready = frame;
        for (k = 1; k <= 60; k = k + 6) dump_mask[f_ready + 8 + k] = 1;
        dump_mask[f_ready + 75] = 1; dump_mask[f_ready + 92] = 1;
        // 第一遍讲解（120 cs ≈ 36 帧 @2 倍速）
        wait_frames(24);
        if (!ducking) begin errors = errors + 1; $display("FAIL not ducking while speaking"); end
        if (u_mix_a.gm > 8'd140 || u_mix_a.gv < 8'd200) begin errors = errors + 1; $display("FAIL duck gains gm=%0d gv=%0d", u_mix_a.gm, u_mix_a.gv); end
        // ---- C4 暂停 ----
        pause = 1; wait_frames(2);
        now0 = u_osd.now; vn0 = v_n; mn0 = m_n;
        wait_frames(8);
        if (u_osd.now != now0 || v_n != vn0) begin errors = errors + 1; $display("FAIL pause: now %0d->%0d voice %0d->%0d", now0, u_osd.now, vn0, v_n); end
        if (m_n - mn0 < 10000) begin errors = errors + 1; $display("FAIL pause: background stalled (%0d samples)", m_n - mn0); end
        pause = 0;
        // 讲解时钟 = 人声样本 / 240（暂停不影响对应关系；允许 1 cs 的采样相位差）
        wait_frames(10);
        now0 = u_osd.now; vn0 = (v_n % VLOOP) / 240;
        if (now0 - vn0 > 1 || vn0 - now0 > 1) begin
            errors = errors + 1; $display("FAIL clock now=%0d voice=%0d", u_osd.now, v_n);
        end
        // ---- 全文页 ----
        wait_frames(30); page_on = 1;
        wait_frames(25); page_on = 0;
        // ---- C5 KEY3 长按：隐藏跑马灯（人声 / 讲解时钟照走）----
        marq_hide = 1; vn0 = v_n; wait_frames(40);
        if (u_osd.sa != 0) begin errors = errors + 1; $display("FAIL marq_hide sa=%0d", u_osd.sa); end
        if (v_n - vn0 < 10000) begin errors = errors + 1; $display("FAIL marq_hide stalled voice (%0d samples)", v_n - vn0); end
        marq_hide = 0;
        $display("INFO voice samples=%0d (loops %0d) bad=%0d underflow=%0d; bg samples=%0d underflow=%0d; sd reads=%0d",
                 v_n, v_n / VLOOP, v_bad, v_under, m_n, m_under, u_sd.sd_card_top_m0.n_reads);
        if (v_bad != 0 || v_n < VLOOP + 1000) begin errors = errors + 1; $display("FAIL voice stream"); end
        if (v_under > 0 || m_under > 0) begin errors = errors + 1; $display("FAIL underflow voice=%0d bg=%0d", v_under, m_under); end
        // 影子 RAM
        fpx = $fopen("sim/out_k1c/sh_line.hex", "w");  for (i = 0; i < 512; i = i + 1)  $fwrite(fpx, "%05h\n", sh_l[i]); $fclose(fpx);
        fpx = $fopen("sim/out_k1c/sh_code.hex", "w");  for (i = 0; i < 512; i = i + 1)  $fwrite(fpx, "%03h\n", sh_c[i]); $fclose(fpx);
        fpx = $fopen("sim/out_k1c/sh_glyph.hex", "w"); for (i = 0; i < 8192; i = i + 1) $fwrite(fpx, "%016h\n", sh_g[i]); $fclose(fpx);
        $fclose(fst);
        if (errors == 0) $display("PASS tb_k1_chain (run sim/k1/chk_chain.py for script content + pixels)");
        else             $display("FAIL tb_k1_chain errors=%0d", errors);
        $finish;
    end
endmodule
