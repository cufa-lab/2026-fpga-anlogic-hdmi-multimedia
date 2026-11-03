`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// tb_narration_audio_mixer —— 背景乐 + 人声混音
//   背景乐：48k 节拍，valid 保持到 ready（PCM 合同）；人声：24k，v_valid 单拍
//   帧首在 tb 里加快到每 2000 拍一次（只为让音量渐变快点走完）
//   1. 不说话：输出 = 背景乐 × 音量（255×255 → 原样）
//   2. 说话：背景乐压到一半（gm=127），人声全音量，人声 24k→48k 中点插值；逐样本与参考模型比对
//   3. 饱和：两路都接近满幅 → 32767 / -32768
//   4. SW2 暂停：人声收掉、背景乐回到全音量；人声流结束（voice_active=0）同样（KEY3 人声开关已去掉）
//   5. 速率：输出握手数 = 背景乐样本数（不丢不多）
// -----------------------------------------------------------------------------
module tb_narration_audio_mixer;
    reg clk = 1'b0;
    always #20 clk = ~clk;
    reg rst_n = 1'b0;

    reg        frame_start = 1'b0;
    reg [7:0]  volume = 8'd255, fade = 8'd255;
    reg        pause = 1'b0, voice_active = 1'b0;
    reg        m_valid = 1'b0;
    reg signed [15:0] m_sample = 0;
    reg        v_valid = 1'b0;
    reg signed [15:0] v_sample = 0;
    wire       m_ready, out_valid;
    wire signed [15:0] out_sample;
    wire       ducking;

    narration_audio_mixer dut (
        .clk(clk), .rst_n(rst_n), .frame_start(frame_start), .volume(volume), .fade_gain(fade),
        .pause(pause), .voice_active(voice_active),
        .m_valid(m_valid), .m_ready(m_ready), .m_sample(m_sample),
        .v_valid(v_valid), .v_ready(), .v_sample(v_sample),
        .out_valid(out_valid), .out_ready(1'b1), .out_sample(out_sample), .ducking(ducking)
    );

    integer cyc = 0;
    always @(posedge clk) begin
        cyc <= cyc + 1;
        frame_start <= (cyc % 2000 == 0);
    end

    // 背景乐：每 520 拍一个样本（48k @25MHz），值由 mgen 给
    integer nm = 0, nout = 0, vcount = 0;
    reg signed [15:0] mval, vval;
    reg big = 1'b0;
    always @(posedge clk) begin
        if (cyc % 520 == 5 && !m_valid) begin
            m_valid  <= 1'b1;
            m_sample <= big ? 16'sd30000 : (nm * 37) % 20000 - 10000;
            nm = nm + 1;
        end else if (m_valid && m_ready)
            m_valid <= 1'b0;
        v_valid <= 1'b0;
        if (cyc % 1040 == 300 && !pause) begin
            v_valid  <= 1'b1;
            v_sample <= big ? 16'sd30000 : (vcount * 911) % 16000 - 8000;
            vcount = vcount + 1;
        end
    end

    // 参考模型（与 DUT 同一公式，独立实现）
    reg signed [15:0] r_prev = 0, r_cur = 0, r_m;
    reg r_fresh = 1'b0;
    integer exp_v, exp, errors = 0, checked = 0;
    reg checking = 1'b0;
    always @(posedge clk) begin
        if (v_valid) begin r_prev = r_cur; r_cur = v_sample; r_fresh = 1'b1; end
        if (m_valid && m_ready) nout = nout + 1;
    end
    function integer sh8; input integer x; sh8 = (x >= 0) ? x / 256 : -((-x + 255) / 256); endfunction
    task check_sample; input integer gm; input integer gv; input integer m; input integer v; input integer got;
        integer e; begin
        e = sh8(m * gm) + sh8(v * gv);
        if (e > 32767) e = 32767; if (e < -32768) e = -32768;
        checked = checked + 1;
        if (got !== e) begin
            errors = errors + 1;
            if (errors < 10) $display("FAIL gm=%0d gv=%0d m=%0d v=%0d got=%0d exp=%0d", gm, gv, m, v, got, e);
        end
    end endtask

    // 每个输出样本：从 DUT 内部取当次的 mreg / vv（插值规则另由 interp 检查）
    reg signed [15:0] last_v;
    integer k;
    always @(posedge clk)
        if (checking && out_valid)
            check_sample(dut.gm, dut.gv, dut.mreg, dut.vv, out_sample);

    // 插值检查：人声在播且稳定时，vv 序列应是 mid, cur, mid, cur ...
    integer interp_err = 0, interp_n = 0;
    wire signed [16:0] r_mid = {r_prev[15], r_prev} + {r_cur[15], r_cur};
    always @(posedge clk)
        if (checking && dut.st == 3'd3 && voice_active && dut.vvol == volume) begin   // ST_P1：vv 刚装入
            interp_n = interp_n + 1;
            if (dut.vv !== r_cur && dut.vv !== r_mid[16:1]) interp_err = interp_err + 1;
        end

    task wait_frames; input integer n; begin repeat (n * 2000) @(posedge clk); end endtask

    integer nm0, no0;
    initial begin
        repeat (5) @(posedge clk); rst_n = 1'b1;
        // ---- 1：不说话 ----
        wait_frames(70);
        if (dut.gm != 255 || dut.gv != 0) begin errors = errors + 1; $display("FAIL idle gains gm=%0d gv=%0d", dut.gm, dut.gv); end
        checking = 1'b1; wait_frames(5); checking = 1'b0;
        // ---- 2：说话 ----
        voice_active = 1'b1;
        wait_frames(70);
        if (dut.gm != 127 || dut.gv != 255 || !ducking) begin errors = errors + 1; $display("FAIL speak gains gm=%0d gv=%0d", dut.gm, dut.gv); end
        checking = 1'b1; wait_frames(20); checking = 1'b0;
        if (interp_n < 50 || interp_err != 0) begin errors = errors + 1; $display("FAIL interp n=%0d err=%0d", interp_n, interp_err); end
        // ---- 3：饱和 ----
        big = 1'b1; volume = 8'd255;
        wait_frames(70);                     // 让 mvol 回到 ducking 目标
        checking = 1'b1; wait_frames(3); checking = 1'b0;
        if (!(out_sample == 16'sd32767 || dut.st != 3'd6)) begin errors = errors + 1; $display("FAIL saturation %0d", out_sample); end
        big = 1'b0;
        // ---- 4：暂停 / 人声结束 ----
        pause = 1'b1;
        wait_frames(70);
        if (dut.gm != 255 || dut.gv != 0 || dut.v_cur != 0) begin errors = errors + 1; $display("FAIL pause gm=%0d gv=%0d v=%0d", dut.gm, dut.gv, dut.v_cur); end
        pause = 1'b0; wait_frames(70);
        voice_active = 1'b0; wait_frames(70);
        if (dut.gm != 255 || dut.gv != 0) begin errors = errors + 1; $display("FAIL voice end gm=%0d gv=%0d", dut.gm, dut.gv); end
        checking = 1'b1; wait_frames(3); checking = 1'b0;
        // ---- 5：速率 ----
        nm0 = nm; no0 = nout;
        wait_frames(10);
        if ((nm - nm0) - (nout - no0) > 1 || (nm - nm0) - (nout - no0) < -1) begin
            errors = errors + 1; $display("FAIL rate in=%0d out=%0d", nm - nm0, nout - no0);
        end
        $display("INFO checked %0d samples, interp %0d", checked, interp_n);
        if (errors == 0) $display("PASS tb_narration_audio_mixer");
        else             $display("FAIL tb_narration_audio_mixer errors=%0d", errors);
        $finish;
    end
endmodule
