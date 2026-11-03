`timescale 1ns/1ps

module tb_param_bar_ambient;
    reg pix_clk = 0, rst_n = 0;
    always #20 pix_clk = ~pix_clk;       // 25 MHz
    localparam integer HOLD = 30;

    wire raw_hs, raw_vs, raw_de;
    wire [11:0] raw_x, raw_y;
    video_timing_ctrl #(
        .H_ACT(640), .H_FP(16), .H_SYNC(96), .H_BP(48),
        .V_ACT(480), .V_FP(10), .V_SYNC(2), .V_BP(33),
        .ACTIVE_LAST(1), .VS_ALIGN_HSYNC(1)
    ) u_timing (.pclk(pix_clk), .rst_n(rst_n), .hsync(raw_hs), .vsync(raw_vs), .de(raw_de), .x(raw_x), .y(raw_y));
    wire hs, vs, de;
    rgb_video_adapter u_adapter (
        .pclk(pix_clk), .rst_n(rst_n), .hs_in(raw_hs), .vs_in(raw_vs), .de_in(raw_de),
        .rgb_in(24'h000000), .video_clk(), .video_hs(hs), .video_vs(vs), .video_de(de), .video_rgb());

    reg  [7:0] bri = 8'd255;
    reg        pshow = 1'b0;
    wire [23:0] final_rgb;
    osd_layer_mixer #(.TS_ENABLE(0), .PARAM_HOLD_FRAMES(HOLD)) u_mix (
        .clk(pix_clk), .rst_n(rst_n), .vsync(vs), .de(de), .rgb_in(24'h404040),
        .subtitle_enable(1'b0), .subtitle_text(96'd0), .subtitle_length(6'd0),
        .brightness(bri), .volume(8'd128), .period_tenths(9'd50), .auto_enabled(1'b1),
        .res_valid(1'b1), .res_width(12'd640), .res_height(12'd480), .param_show(pshow), .param_hide(1'b0),
        .ext_hit(1'b0), .ext_color(24'd0), .ext_alpha(8'd0), .txt_hit(1'b0), .txt_color(24'd0), .txt_alpha(8'd0),
        .rgb_out(final_rgb), .overlay_active());

    // ---------------- 参考模型 ----------------
    integer frames = 0, errors = 0;
    reg  [7:0] m_pfade = 0;
    integer    m_hold = HOLD;
    function [7:0] step16; input [7:0] c; input up;
        begin
            if (up) step16 = (c > 239) ? 8'd255 : c + 8'd16;
            else    step16 = (c < 16)  ? 8'd0   : c - 8'd16;
        end
    endfunction
    always @(posedge pix_clk) if (rst_n) begin
        if (u_mix.param_changed || pshow) m_hold <= HOLD;
        else if (u_mix.frame_start && m_hold > 0) m_hold <= m_hold - 1;
        if (u_mix.frame_start) begin
            frames  <= frames + 1;
            m_pfade <= step16(m_pfade, m_hold > 0);
        end
    end
    always @(negedge pix_clk) if (rst_n && u_mix.param_fade !== m_pfade) begin
        if (errors < 10) $display("FAIL fade dut=%0d model=%0d frame %0d", u_mix.param_fade, m_pfade, frames);
        errors = errors + 1;
    end

    // 传感器斜坡：在帧中部（有效区内）改值，下一帧边界才进参数条
    reg        ramp_on = 0;
    integer    ramp_left = 0;
    reg [7:0]  last_sent;
    always @(posedge pix_clk) if (rst_n && raw_y == 12'd200 && raw_x == 12'd300 && ramp_on && ramp_left > 0) begin
        last_sent <= bri;
        bri       <= bri - 8'd2;
        ramp_left <= ramp_left - 1;
    end
    // 实时：每个 frame_start 之后 BRT 锁存 == 这一帧开始时的输入
    reg [7:0] bri_at_fs;
    always @(posedge pix_clk) if (u_mix.frame_start) bri_at_fs <= bri;
    integer live_checked = 0;
    always @(posedge pix_clk) if (rst_n && frames > 2 && raw_y == 12'd100 && raw_x == 12'd10) begin
        if (u_mix.u_text.bright_frame !== bri_at_fs) begin
            $display("FAIL live BRT latch %0d != %0d (frame %0d)", u_mix.u_text.bright_frame, bri_at_fs, frames); errors = errors + 1;
        end
        live_checked = live_checked + 1;
    end

    integer f_stop;
    initial begin
        #1000 rst_n = 1;
        // P1
        wait (frames == 55);
        if (u_mix.param_fade !== 0) begin $display("FAIL P1 power-up bar not hidden"); errors = errors + 1; end
        else $display("PASS P1 power-up bar shown then hidden");
        // P2 / P3 斜坡 40 帧
        ramp_on = 1; ramp_left = 40;
        wait (frames == 55 + 20);
        if (u_mix.param_fade !== 255) begin $display("FAIL P2 bar not shown by brightness change"); errors = errors + 1; end
        else $display("PASS P2 brightness change -> bar shown (BRT=%0d)", u_mix.u_text.bright_frame);
        // P5 显示期间切图
        @(posedge pix_clk) pshow <= 1'b1;
        @(posedge pix_clk) pshow <= 1'b0;
        wait (ramp_left == 0);
        f_stop = frames;
        $display("INFO ramp finished at frame %0d, BRT=%0d", f_stop, bri);
        wait (frames == f_stop + 2);
        if (u_mix.u_text.bright_frame !== bri) begin $display("FAIL P3 final BRT %0d != %0d", u_mix.u_text.bright_frame, bri); errors = errors + 1; end
        else $display("PASS P3 BRT followed the ramp live to %0d", bri);
        // P4
        wait (frames == f_stop + HOLD - 2);
        if (u_mix.param_fade !== 255) begin $display("FAIL P4 bar hid too early"); errors = errors + 1; end
        wait (frames == f_stop + HOLD + 20);
        if (u_mix.param_fade !== 0) begin $display("FAIL P4 bar not hidden after idle"); errors = errors + 1; end
        else $display("PASS P4 bar stayed %0d frames after last change, then hid", HOLD);
        if (errors == 0 && live_checked > 100) $display("PASS tb_param_bar_ambient (live checks %0d)", live_checked);
        else                                    $display("FAIL tb_param_bar_ambient errors=%0d live=%0d", errors, live_checked);
        $finish;
    end
endmodule
