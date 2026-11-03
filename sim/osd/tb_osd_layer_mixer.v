`timescale 1ns/1ps

module tb_osd_layer_mixer;
    reg sys_clk = 0, pix_clk = 0, rst_n = 0;
    always #10 sys_clk = ~sys_clk;       // 50 MHz
    always #20 pix_clk = ~pix_clk;       // 25 MHz

    localparam integer HOLD = 30;        // 参数条显示帧数（缩短，便于看到淡出）

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

    // 测试图：用送 HDMI 那组 vs/de 自己数坐标（等价于 reader 在 de 期间送出的像素）
    reg [9:0] tx = 0, ty = 0;
    reg       tde_q = 0;
    always @(posedge pix_clk) begin
        tde_q <= de;
        tx <= de ? tx + 10'd1 : 10'd0;
        if (!vs) ty <= 0; else if (tde_q && !de) ty <= ty + 10'd1;
    end
    wire [23:0] pattern = {tx[9:2], ty[8:1], (tx[5] ^ ty[5]) ? 8'hc0 : 8'h30};
    wire [23:0] bg_rgb  = de ? pattern : 24'h000000;

    wire frame_start;
    frame_boundary_pulse #(.V_ACTIVE(480)) u_fb (
        .pix_clk(pix_clk), .pix_rst_n(rst_n), .vsync_n(vs), .de(de), .frame_start(frame_start), .blank_start());

    // ---------------- sys 域字幕源 + 跨域提交 ----------------
    reg  [5:0]  image_idx = 0;
    reg         show = 0;
    wire        commit, src_en;
    wire [95:0] src_text;
    wire [5:0]  src_len;
    subtitle_text_source #(.CLK_HZ(50_000_000)) u_src (
        .clk(sys_clk), .rst_n(rst_n), .image_idx(image_idx), .show(show),
        .commit(commit), .text(src_text), .length(src_len), .enable(src_en));
    wire [95:0] sub_text;
    wire [5:0]  sub_len;
    wire        sub_en;
    subtitle_frame_commit u_commit (
        .src_clk(sys_clk), .src_rst_n(rst_n), .src_commit(commit), .src_text(src_text),
        .src_length(src_len), .src_enable(src_en), .dst_clk(pix_clk), .dst_rst_n(rst_n),
        .frame_start(frame_start), .dst_text(sub_text), .dst_length(sub_len), .dst_enable(sub_en));

    // ---------------- 多图层（第十六讲：状态行已去掉，mixer 直接接图片） ----------------
    reg  [7:0] volume = 8'd128;
    reg  [8:0] period = 9'd25;
    reg        pshow  = 1'b0;
    wire [23:0] status_rgb = bg_rgb;
    wire [23:0] final_rgb;
    wire        status_active = 1'b0;
    wire        mix_active;

    osd_layer_mixer #(.TS_ENABLE(0), .PARAM_HOLD_FRAMES(HOLD)) u_mix (
        .clk(pix_clk), .rst_n(rst_n), .vsync(vs), .de(de), .rgb_in(status_rgb),
        .subtitle_enable(sub_en), .subtitle_text(sub_text), .subtitle_length(sub_len),
        .brightness(8'd100), .volume(volume), .period_tenths(period), .auto_enabled(1'b1),
        .res_valid(1'b1), .res_width(12'd1920), .res_height(12'd1080), .param_show(pshow), .param_hide(1'b0), .ext_hit(1'b0), .ext_color(24'd0), .ext_alpha(8'd0), .txt_hit(1'b0), .txt_color(24'd0), .txt_alpha(8'd0),
        .rgb_out(final_rgb), .overlay_active(mix_active));

    // ---------------- 期望模型 ----------------
    integer frames = 0;                  // 按 mixer 的 frame_start 计
    integer errors = 0, checked = 0, layer_px = 0;
    reg  [7:0] m_sfade = 0, m_pfade = 0;
    integer    m_hold = HOLD;

    function [7:0] step16; input [7:0] c; input up;
        begin
            if (up) step16 = (c > 239) ? 8'd255 : c + 8'd16;
            else    step16 = (c < 16)  ? 8'd0   : c - 8'd16;
        end
    endfunction

    // 淡入淡出模型（与 DUT 同拍更新）
    always @(posedge pix_clk) if (rst_n) begin
        if (u_mix.param_changed || pshow) m_hold <= HOLD;
        else if (u_mix.frame_start && m_hold > 0) m_hold <= m_hold - 1;
        if (u_mix.frame_start) begin
            frames  <= frames + 1;
            m_sfade <= step16(m_sfade, sub_en);
            m_pfade <= step16(m_pfade, m_hold > 0);
        end
    end
    always @(negedge pix_clk) if (rst_n && (u_mix.subtitle_fade !== m_sfade || u_mix.param_fade !== m_pfade)) begin
        if (errors < 10) $display("FAIL fade dut s=%0d p=%0d model s=%0d p=%0d frame %0d",
                                  u_mix.subtitle_fade, u_mix.param_fade, m_sfade, m_pfade, frames);
        errors = errors + 1;
    end

    function real blendf; input integer b, f, a; begin blendf = (f * a + b * (255 - a)) / 255.0; end endfunction
    function ok24;
        input [23:0] got, bgc, fgc; input integer a;
        integer c; real e;
        begin
            ok24 = 1'b1;
            for (c = 0; c < 3; c = c + 1) begin
                e = blendf(bgc[c*8 +: 8], fgc[c*8 +: 8], a);
                if (got[c*8 +: 8] - e > 1.0 || e - got[c*8 +: 8] > 1.0) ok24 = 1'b0;
            end
        end
    endfunction

    integer px, py;
    reg [23:0] e_col;
    integer    e_a;
    reg        e_hit, in_boxes;
    always @(posedge pix_clk) if (rst_n && frames >= 2) begin
        px = u_mix.pixel_x; py = u_mix.pixel_y;
        if (!de) begin
            if (mix_active || final_rgb !== status_rgb) begin
                if (errors < 10) $display("FAIL activity outside de"); errors = errors + 1;
            end
        end else begin
            checked = checked + 1;
            if (px !== tx || py !== ty) begin
                if (errors < 10) $display("FAIL coord %0d,%0d vs %0d,%0d", px, py, tx, ty); errors = errors + 1;
            end
            in_boxes = (px >= 4 && px < 416 && py >= 37 && py < 56) ||
                       (px >= 8 && px < 632 && py >= 452 && py < 472);
            e_hit = 1'b1; e_col = 0; e_a = 0;
            if      (u_mix.ts_pixel)                   begin e_col = 24'he0f0ff; e_a = 255; end
            else if (u_mix.ts_box)                     begin e_col = 24'h000000; e_a = 128; end
            else if (u_mix.param_pixel && m_pfade != 0) begin e_col = 24'hfff080; e_a = m_pfade; end      // 扩1：淡出到 0 后不占层
            else if (u_mix.param_box && m_pfade != 0)   begin e_col = 24'h000000; e_a = m_pfade / 2; end
            else if (u_mix.subtitle_pixel)             begin e_col = 24'h000000; e_a = m_sfade; end
            else if (u_mix.subtitle_band_pixel)        begin e_col = 24'he8eef2; e_a = m_sfade - m_sfade / 4; end
            else e_hit = 1'b0;
            if (mix_active !== e_hit || (e_hit && !in_boxes)) begin
                if (errors < 10) $display("FAIL layer region (%0d,%0d) hit=%b exp=%b", px, py, mix_active, e_hit);
                errors = errors + 1;
            end else if (!e_hit) begin
                if (final_rgb !== status_rgb) begin
                    if (errors < 10) $display("FAIL passthrough (%0d,%0d) %h vs %h", px, py, final_rgb, status_rgb);
                    errors = errors + 1;
                end
            end else begin
                layer_px = layer_px + 1;
                if (!ok24(final_rgb, status_rgb, e_col, e_a)) begin
                    if (errors < 10) $display("FAIL blend (%0d,%0d) got %h bg %h fg %h a %0d", px, py, final_rgb, status_rgb, e_col, e_a);
                    errors = errors + 1;
                end
            end
            // 状态条和新图层互不重叠
            if (status_active && e_hit) begin
                if (errors < 10) $display("FAIL status OSD overlaps layer (%0d,%0d)", px, py); errors = errors + 1;
            end
        end
    end

    // ---------------- 导出帧 ----------------
    integer fd = 0;
    reg     dumping = 0;
    task dump_frame; input integer n;
        reg [8*64-1:0] fname;
        begin
            @(posedge u_mix.frame_start);
            $sformat(fname, "sim/out_l9/frame_%0d%0d%0d.hex", n / 100, (n / 10) % 10, n % 10);
            fd = $fopen(fname, "w");
            dumping = 1;
            @(posedge u_mix.frame_start);
            dumping = 0;
            $fclose(fd);
            $display("INFO dumped frame %0d", n);
        end
    endtask
    always @(posedge pix_clk) if (dumping && de) $fwrite(fd, "%h\n", final_rgb);

    // ---------------- 激励 ----------------
    reg [5:0]  s_cell0;
    reg [4:0]  s_col0;
    integer    pos0, pos1;
    integer    f0;
    initial begin
        #1000 rst_n = 1;
        @(posedge sys_clk) show = 1;
        wait (frames == 6);  dump_frame(6);                        // 字幕带、参数条正在淡入
        wait (frames == 20); dump_frame(20);                       // 全部显示
        f0 = frames; #100 pos0 = u_mix.u_text.scroll_cell * 18 + u_mix.u_text.scroll_col;
        wait (frames == f0 + 6);
        #100 pos1 = u_mix.u_text.scroll_cell * 18 + u_mix.u_text.scroll_col;
        if (pos1 - pos0 != 2) begin
            $display("FAIL scroll speed: %0d px in 6 frames (expect 2)", pos1 - pos0); errors = errors + 1;
        end
        @(posedge sys_clk) image_idx = 6'd1;                       // 换图 → 字幕换成 CAPTION1
        wait (frames == f0 + 12);
        if (sub_text[7:0] !== "E" || sub_len !== 6'd11) begin
            $display("FAIL caption not switched: %h len %0d", sub_text, sub_len); errors = errors + 1;
        end
        wait (frames == 50);
        if (u_mix.param_fade !== 0) begin $display("FAIL param bar not hidden after hold"); errors = errors + 1; end
        dump_frame(50);                                            // 参数条已隐藏
        volume = 8'd192;                                           // 调音量 → 参数条重新出现
        wait (frames == 70);
        if (u_mix.param_fade !== 255) begin $display("FAIL param bar not re-shown"); errors = errors + 1; end
        dump_frame(70);
        @(posedge sys_clk) show = 0;                               // 关字幕 → 淡出
        wait (frames == 76); dump_frame(76);
        wait (frames == 90);
        if (u_mix.subtitle_fade !== 0) begin $display("FAIL subtitle not faded out"); errors = errors + 1; end
        wait (frames == 120);
        if (u_mix.param_fade !== 0) begin $display("FAIL param bar not hidden before param_show"); errors = errors + 1; end
        @(posedge pix_clk) pshow <= 1'b1;                         // 切到新画面（单拍）→ 参数条重新出现
        @(posedge pix_clk) pshow <= 1'b0;
        wait (frames == 140);
        if (u_mix.param_fade !== 255) begin $display("FAIL param bar not re-shown by param_show"); errors = errors + 1; end
        dump_frame(140);                                           // 参数条带分辨率 1920X1080
        $display("INFO frames=%0d checked=%0d layer_px=%0d errors=%0d", frames, checked, layer_px, errors);
        if (errors == 0 && layer_px > 100000) $display("PASS tb_osd_layer_mixer");
        else                                   $display("FAIL tb_osd_layer_mixer");
        $finish;
    end
endmodule
