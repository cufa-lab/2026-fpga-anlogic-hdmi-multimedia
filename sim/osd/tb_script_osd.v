`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// tb_script_osd —— 讲解字幕图层（真实 640x480@60 时序，与顶层同一条链）
//   video_timing_ctrl → rgb_video_adapter → [测试图] → osd_layer_mixer(SUBTITLE_EN=0, ext_*) ← script_osd
//   三块 RAM 预装 sim/k1/o_*.hex（sim/k1/gen_osd.py 用 k1lib 按 script_loader 布局生成：黑体 32×32 灰度字形，
//   17 句连成一段讲解流 + 段落行表）；字形 RAM 用行为模型（8192×64，同 1 拍读延迟），line / code 用真 script_dc_ram。
//   sys 侧握手由 tb 直接翻 toggle。扩1b：两级混合（ext 底板 / 装饰 / 图标 + txt 灰度文字），title_show 重新显示标题。
//   讲解时钟加速：人声样本节拍 vper 拍一次（真实 1042），无人声段 tick48 每 44 拍一次。
//   本 tb 只做时序 / 握手类断言；逐像素对照由 sim/k1/chk_osd.py 根据
//     sim/out_k1/state.txt（每帧 tl_done 时的状态快照）+ frame_NNN.txt（选定帧逐像素：rgb_out / 第 1 级结果 /
//     ext 类别、颜色、alpha / txt 命中、颜色、alpha）
//   重新渲染期望的文字点阵后比对，并转出 PNG 目检。
// -----------------------------------------------------------------------------
module tb_script_osd;
    reg pix_clk = 0, rst_n = 0;
    always #20 pix_clk = ~pix_clk;       // 25 MHz

    // ---------------- 视频时序（顶层同参） ----------------
    wire raw_hs, raw_vs, raw_de;
    wire [11:0] raw_x, raw_y;
    video_timing_ctrl #(
        .H_ACT(640), .H_FP(16), .H_SYNC(96), .H_BP(48),
        .V_ACT(480), .V_FP(10), .V_SYNC(2), .V_BP(33),
        .ACTIVE_LAST(1), .VS_ALIGN_HSYNC(1)
    ) u_timing (.pclk(pix_clk), .rst_n(rst_n), .hsync(raw_hs), .vsync(raw_vs), .de(raw_de), .x(raw_x), .y(raw_y));

    wire hs, vs, de;
    wire [23:0] adapter_rgb;
    rgb_video_adapter u_adapter (
        .pclk(pix_clk), .rst_n(rst_n), .hs_in(raw_hs), .vs_in(raw_vs), .de_in(raw_de),
        .rgb_in(24'h000000), .video_clk(), .video_hs(hs), .video_vs(vs), .video_de(de), .video_rgb(adapter_rgb));

    // 测试图：上半暗蓝渐变（暗-平），下半按阶段切换（亮 + 条纹 = 亮-花 / 暗-平）
    reg [9:0] tx = 0, ty = 0;
    reg       tde_q = 0;
    reg       busy_pic = 0;
    always @(posedge pix_clk) begin
        tde_q <= de;
        tx <= de ? tx + 10'd1 : 10'd0;
        if (!vs) ty <= 0; else if (tde_q && !de) ty <= ty + 10'd1;
    end
    wire [7:0]  gx = tx[9:2];
    wire [23:0] sky  = {8'd20 + {2'd0, ty[8:3]}, 8'd60 + {1'b0, ty[8:2]}, 8'd110 + {1'b0, ty[8:2]}};
    wire [23:0] reef = (tx[2] ^ ty[3]) ? {8'd250, 8'd200, 8'd150} : {8'd120, 8'd200, 8'd210};
    wire [23:0] sea  = {8'd10, 8'd40 + {2'd0, gx[7:2]}, 8'd80 + {1'b0, ty[8:2]} - 8'd60};
    wire [23:0] pattern = (ty < 10'd240) ? sky : (busy_pic ? reef : sea);
    wire [23:0] bg_rgb  = de ? pattern : 24'h000000;

    // ---------------- RAM ----------------
    wire        line_re, code_re, glyph_re;
    wire [8:0]  line_raddr, code_raddr;
    wire [12:0] glyph_raddr;
    wire [17:0] line_rdata;
    wire [9:0]  code_rdata;
    reg  [63:0] glyph_rdata;
    reg         lw_we = 0;
    reg  [8:0]  lw_addr = 0;
    reg  [17:0] lw_data = 0;
    script_dc_ram #(.AW(9), .DW(18)) u_line (
        .wclk(pix_clk), .we(lw_we), .waddr(lw_addr), .wdata(lw_data),
        .rclk(pix_clk), .re(line_re), .raddr(line_raddr), .rdata(line_rdata));
    script_dc_ram #(.AW(9), .DW(10)) u_code (
        .wclk(pix_clk), .we(1'b0), .waddr(9'd0), .wdata(10'd0),
        .rclk(pix_clk), .re(code_re), .raddr(code_raddr), .rdata(code_rdata));
    reg [63:0] gmem [0:8191];
    always @(posedge pix_clk) if (glyph_re) glyph_rdata <= gmem[glyph_raddr];
    initial begin
        $readmemh("sim/k1/o_line.hex",  u_line.mem);
        $readmemh("sim/k1/o_code.hex",  u_code.mem);
        $readmemh("sim/k1/o_glyph.hex", gmem);
    end

    // ---------------- 控制 ----------------
    reg rel_req_t = 0, ready_t = 0, epoch_t = 0;
    wire rel_ack_t;
    reg  pause = 0, page_on = 0, marq_hide = 0, hide = 0, title_show = 0;
    reg  [8:0] gain = 9'd256;                     // 亮度同步：+gain=N（1..256）；chk_osd.py 用 K1_GAIN=N 对照
    initial if ($value$plusargs("gain=%d", gain)) $display("bright_gain = %0d", gain);
    reg  voice_cnt0 = 0, tick48 = 0, voice_run = 0;
    integer vper = 1042, tper = 44, vc = 0, tc = 0;
    always @(posedge pix_clk) begin
        tick48 <= 1'b0;
        vc = vc + 1; tc = tc + 1;
        if (vc >= vper) begin vc = 0; if (voice_run && !pause) voice_cnt0 <= ~voice_cnt0; end
        if (tc >= tper) begin tc = 0; tick48 <= 1'b1; end
    end

    // ---------------- DUT ----------------
    wire [9:0]  mx, my;
    wire        mfs;
    wire        ext_hit, page_active, txt_hit;
    wire [23:0] ext_color, final_rgb, txt_color;
    wire [7:0]  ext_alpha, txt_alpha;
    script_osd dut (
        .clk(pix_clk), .rst_n(rst_n), .frame_start(mfs), .de(de), .pixel_x(mx), .pixel_y(my), .rgb_in(bg_rgb),
        .rel_req_t(rel_req_t), .rel_ack_t(rel_ack_t), .ready_t(ready_t), .epoch_t(epoch_t),
        .line_re(line_re), .line_raddr(line_raddr), .line_rdata(line_rdata),
        .code_re(code_re), .code_raddr(code_raddr), .code_rdata(code_rdata),
        .glyph_re(glyph_re), .glyph_raddr(glyph_raddr), .glyph_rdata(glyph_rdata),
        .voice_cnt0(voice_cnt0), .tick48(tick48), .pause(pause), .page_on(page_on),
        .hide(hide), .marq_hide(marq_hide), .title_show(title_show), .bright_gain(gain), .ext_hit(ext_hit), .ext_color(ext_color), .ext_alpha(ext_alpha),
        .txt_hit(txt_hit), .txt_color(txt_color), .txt_alpha(txt_alpha), .page_active(page_active));

    osd_layer_mixer #(.TS_ENABLE(0), .SUBTITLE_EN(0), .PARAM_Y(82), .PARAM_HOLD_FRAMES(0)) u_mix (
        .clk(pix_clk), .rst_n(rst_n), .vsync(vs), .de(de), .rgb_in(bg_rgb),
        .subtitle_enable(1'b0), .subtitle_text(96'd0), .subtitle_length(6'd0),
        .brightness(8'd200), .volume(8'd128), .period_tenths(9'd25), .auto_enabled(1'b1),
        .res_valid(1'b1), .res_width(12'd640), .res_height(12'd480), .param_show(1'b0), .param_hide(page_active),
        .ext_hit(ext_hit), .ext_color(ext_color), .ext_alpha(ext_alpha),
        .txt_hit(txt_hit), .txt_color(txt_color), .txt_alpha(txt_alpha),
        .pix_x(mx), .pix_y(my), .pix_frame_start(mfs), .rgb_out(final_rgb), .overlay_active());

    // ---------------- 帧计数 / 状态快照 / 帧转储 ----------------
    integer frame = 0, fst, fpx = 0, errors = 0;
    reg     dumping = 0;
    reg [1023:0] dump_mask = 0;     // 由 initial 置位
    integer hit_in_frame = 0, line_reads_in_frame = 0;
    always @(posedge pix_clk) if (mfs) frame <= frame + 1;

    always @(posedge pix_clk) if (rst_n && dut.tl_done) begin
        #1;
        $fwrite(fst, "%0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d\n",
            frame, dut.now, dut.loaded, dut.has_voice, dut.cur, dut.gc, dut.s_w, dut.ta, dut.ia, dut.sa, dut.pa,
            dut.xp, dut.off, dut.rr, dut.page_y, dut.ptop, dut.psub, dut.lvl_t_a, dut.lvl_s_a, dut.releasing,
            dut.rel_ack_t, pause, marq_hide, hide, dut.line_end, dut.rows_n, dut.gend, dut.s0, dut.t_w, dut.t_start,
            dut.i_w, dut.i_start, dut.itim, dut.ended, dut.hx, dut.t_cur, dut.xd, dut.tl, page_active);
    end

    // 像素在 negedge 采：此时 mixer 坐标、ext_*、rgb_out 都已稳定且对应同一像素
    always @(negedge pix_clk) if (rst_n) begin
        if (de && (ext_hit || txt_hit)) hit_in_frame = hit_in_frame + 1;
        if (dumping && de && my < 10'd480)
            $fwrite(fpx, "%06h %06h %0d %06h %02h %0d %06h %02h\n", final_rgb, u_mix.stage1_rgb,
                    !ext_hit ? 0 : dut.ic_px ? 2 : dut.ic_bg ? 3 : dut.dc_px ? 4 : 5,
                    ext_hit ? ext_color : 24'h0, ext_hit ? ext_alpha : 8'h0,
                    txt_hit, txt_hit ? txt_color : 24'h0, txt_hit ? txt_alpha : 8'h0);
    end
    // 隐藏 / 释放后：一帧里不得有任何讲解图层像素；空闲（未加载）时不读 RAM
    always @(posedge pix_clk) if (rst_n && line_re) line_reads_in_frame = line_reads_in_frame + 1;
    reg [8*32-1:0] fname;
    reg expect_blank = 0;
    always @(posedge pix_clk) if (mfs) begin
        if (expect_blank && hit_in_frame != 0) begin
            errors = errors + 1; $display("FAIL frame %0d: %0d overlay pixels while hidden/unloaded", frame, hit_in_frame);
        end
        hit_in_frame = 0; line_reads_in_frame = 0;
        if (dumping) begin $fclose(fpx); dumping = 0; end
        if (dump_mask[frame + 1]) begin
            $sformat(fname, "sim/out_k1/frame_%03d.txt", frame + 1);
            fpx = $fopen(fname, "w"); dumping = 1;
        end
    end

    task wait_to; input integer f; begin while (frame < f) @(posedge pix_clk); end endtask

    // ---------------- 场景 ----------------
    integer f_rel, k;
    reg [17:0] now_a;
    reg [7:0]  cur_a;
    initial begin
        fst = $fopen("sim/out_k1/state.txt", "w");
        // 选定转储帧
        dump_mask[12] = 1; dump_mask[40] = 1; dump_mask[105] = 1; dump_mask[118] = 1;
        dump_mask[150] = 1; dump_mask[130] = 1; dump_mask[166] = 1; dump_mask[178] = 1;
        dump_mask[203] = 1; dump_mask[207] = 1; dump_mask[216] = 1; dump_mask[250] = 1; dump_mask[258] = 1;
        dump_mask[272] = 1; dump_mask[290] = 1; dump_mask[320] = 1; dump_mask[345] = 1; dump_mask[380] = 1;
        dump_mask[442] = 1; dump_mask[455] = 1; dump_mask[470] = 1; dump_mask[488] = 1;
        repeat (10) @(posedge pix_clk); rst_n = 1;

        // 0..2 帧：没装字幕 → 图层全空
        expect_blank = 1;
        wait_to(3);
        expect_blank = 0;
        // sys 侧：表头写完（ready）→ 人声开始（epoch）
        ready_t = ~ready_t;
        repeat (2000) @(posedge pix_clk);
        epoch_t = ~epoch_t; voice_run = 1;

        wait_to(100); busy_pic = 1;
        wait_to(120); vper = 130;                     // 8 倍速
        wait_to(190); busy_pic = 0;
        wait_to(200); page_on = 1;                    // 全文页滑出（有滚动）
        wait_to(262); page_on = 0;
        wait_to(282);
        pause = 1;                                    // 暂停：人声停 → now 冻结
        wait_to(286); now_a = dut.now;
        wait_to(300);
        if (dut.now != now_a) begin errors = errors + 1; $display("FAIL pause: now moved %0d -> %0d", now_a, dut.now); end
        pause = 0;
        wait_to(310); marq_hide = 1; cur_a = dut.cur;  // KEY3 长按：跑马灯淡出，时间轴照走
        wait_to(316);                                 // 标题 / 导语：显示 HOLD 帧后已淡出
        if (dut.ta != 8'd0 || dut.ia != 8'd0) begin errors = errors + 1; $display("FAIL title/intro not hidden: ta=%0d ia=%0d", dut.ta, dut.ia); end
        @(posedge pix_clk); title_show = 1; @(posedge pix_clk); title_show = 0;   // 切到新画面 → 重新显示（不受 marq_hide 影响）
        wait_to(328);
        if (dut.sa != 8'd0) begin errors = errors + 1; $display("FAIL marq_hide: sa=%0d", dut.sa); end
        if (dut.ta == 8'd0) begin errors = errors + 1; $display("FAIL marq_hide hid title: ta=%0d", dut.ta); end
        if (dut.cur == cur_a) begin errors = errors + 1; $display("FAIL marq_hide froze timeline: cur=%0d", dut.cur); end
        $display("INFO marq_hide: sa -> 0, cur %0d -> %0d (timeline runs), ta=%0d", cur_a, dut.cur, dut.ta);
        marq_hide = 0;                                // 再次长按：淡入，位置与人声一致
        wait_to(330);
        epoch_t = ~epoch_t; vper = 32;                // 人声重播（单曲循环）→ 从头，32 倍速追完全部
        wait_to(430);
        if (!dut.ended || dut.sa != 8'd0) begin errors = errors + 1; $display("FAIL end: ended=%0d sa=%0d", dut.ended, dut.sa); end
        if (dut.ta == 8'd0 || dut.ia == 8'd0) begin errors = errors + 1; $display("FAIL title_show: ta=%0d ia=%0d", dut.ta, dut.ia); end
        page_on = 1;                                  // 页面开着时换曲：页面文字也要淡出
        wait_to(432);
        // 换曲：释放握手
        f_rel = frame; rel_req_t = ~rel_req_t;
        while (rel_ack_t == 1'b0 && frame < f_rel + 30) @(posedge pix_clk);
        if (rel_ack_t != 1'b1) begin errors = errors + 1; $display("FAIL release: no ack"); end
        else $display("INFO release ack after %0d frames", frame - f_rel);
        if (dut.loaded || dut.ta || dut.ia || dut.sa || dut.pa) begin errors = errors + 1; $display("FAIL release: still visible"); end
        voice_run = 0;
        wait_to(444);                                 // 页面仍开着：只剩空底板（无文字）
        // 新曲目：无人声（has_voice=0），loader 改写表头后给 ready
        @(posedge pix_clk); lw_we = 1; lw_addr = 9'd3; lw_data = u_line.mem[3] & ~18'h200;
        @(posedge pix_clk); lw_we = 0;
        ready_t = ~ready_t;
        wait_to(450); page_on = 0;
        wait_to(475); pause = 1;
        wait_to(477); now_a = dut.now;
        wait_to(482);
        if (dut.now != now_a) begin errors = errors + 1; $display("FAIL no-voice pause: now moved"); end
        pause = 0;
        wait_to(490); hide = 1;
        wait_to(492); expect_blank = 1;
        wait_to(496); expect_blank = 0; hide = 0;
        wait_to(500);
        $fclose(fst);
        if (errors == 0) $display("PASS tb_script_osd (handshake/timing; run sim/k1/chk_osd.py for pixels)");
        else             $display("FAIL tb_script_osd errors=%0d", errors);
        $finish;
    end
endmodule
