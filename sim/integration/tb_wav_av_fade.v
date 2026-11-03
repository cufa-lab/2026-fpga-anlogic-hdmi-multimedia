`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// tb_wav_av_fade ——转场 → WAV 增益（真实 video_fade_transition + sync_fade_gain，
//   中间的音频 alpha 选择与 top_tf_hdmi_audio 同一表达式）
//   [F1] 手动切换：gain 255 → 单调降到 0 → 全黑期保持 0（此时才放出切换请求、提交新图、换曲）
//        → 提交后单调升回 255
//   [F2] 自动轮播（next_auto_in=1 → 随机方块溶解，2026-10-18 起替代 Wipe）：gain 与手动相同——淡出到 0、
//        全黑期 0、提交后逐帧 32,64,..,224 单调上升回 255（溶解 2 倍速）；期间 dissolve_active 拉高（画面走溶解、不压暗）
// -----------------------------------------------------------------------------
module tb_wav_av_fade;
    localparam integer N = 16;                 // = top TRANSITION_FRAMES
    localparam integer FRAME = 200;            // 加速：每 200 拍一帧
    reg clk = 0, cclk = 0, rst_n = 0;
    always #20 clk = ~clk;                     // pixel_clk 25MHz
    always #10 cclk = ~cclk;                   // sys_clk 50MHz

    integer fc = 0;
    reg frame_start = 0;
    always @(posedge clk) begin fc <= (fc == FRAME-1) ? 0 : fc + 1; frame_start <= (fc == FRAME-1); end

    reg next_ev = 0, next_auto = 0, commit = 0;
    wire next_out, next_tag_out, busy, fade_active, dissolve_active;
    wire [7:0] fade_alpha;
    wire [4:0] fade_ramp;
    video_fade_transition #(.FADE_FRAMES(N), .HOLD_TIMEOUT_FRAMES(480)) u_fade (
        .ctrl_clk(cclk), .ctrl_rst_n(rst_n), .next_event_in(next_ev), .next_tag_in(1'b0), .next_auto_in(next_auto), .next_block_in(1'b0),
        .hold_abort_in(1'b0), .next_event_out(next_out), .next_tag_out(next_tag_out), .fade_busy(busy),
        .pix_clk(clk), .pix_rst_n(rst_n), .player_active(1'b1), .frame_start(frame_start), .index_commit(commit),
        .alpha_out(fade_alpha), .fade_active(fade_active), .dissolve_active(dissolve_active),
        .ramp_out(fade_ramp)
    );
    // ---- 与 top_tf_hdmi_audio 相同 ----
    wire [7:0]  wav_alpha_pix      = fade_alpha;
    reg         dis_seen = 0;
    always @(posedge clk) if (dissolve_active) dis_seen <= 1'b1;
    wire [7:0]  gain;
    sync_fade_gain #(.SYNC_STAGES(2)) u_gain (
        .pix_clk(clk), .pix_rst_n(rst_n), .frame_start(frame_start), .alpha_pix(wav_alpha_pix),
        .fade_active(fade_active), .aud_clk(clk), .aud_rst_n(rst_n), .gain_aud(gain)
    );

    integer errors = 0;
    task fail(input [8*64-1:0] m); begin errors = errors + 1; $display("FAIL @%0t: %0s", $time, m); end endtask

    // 每帧采样一次 gain（帧中间）
    integer fs [0:127];
    integer nf = 0;
    reg sampling = 0;
    always @(posedge clk) if (sampling && fc == FRAME/2) begin if (nf < 128) fs[nf] = gain; nf = nf + 1; end

    integer k, i_commit, mono, steps;
    initial begin
        repeat (5) @(negedge clk); rst_n = 1;
        repeat (3 * FRAME) @(negedge clk);
        if (gain != 255) fail("idle gain != 255");

        // [F1] 手动
        nf = 0; sampling = 1;
        @(negedge cclk) next_ev = 1; @(negedge cclk) next_ev = 0;
        wait (next_out);                                  // 全黑：控制层收到切换请求
        repeat (3 * FRAME) @(negedge clk);                // 模拟加载 3 帧
        if (gain != 0) fail("[F1] gain not 0 in HOLD");
        wait (frame_start); @(negedge clk) commit = 1; @(negedge clk) commit = 0;
        i_commit = nf;
        wait (!fade_active); repeat (3 * FRAME) @(negedge clk);
        sampling = 0;
        mono = 1;
        for (k = 1; k < i_commit; k = k + 1) if (fs[k] > fs[k-1]) mono = 0;
        if (!mono) fail("[F1] fade-out not monotonic");
        for (k = i_commit + 1; k < nf; k = k + 1) if (fs[k] < fs[k-1]) mono = 0;
        if (!mono) fail("[F1] fade-in not monotonic");
        if (fs[0] != 255 || fs[i_commit - 1] != 0 || fs[i_commit] > 16 || fs[nf-1] != 255) fail("[F1] endpoints");
        $display("INFO [F1] manual: gain %0d -> 0 (%0d frames to commit) -> %0d after %0d frames",
                 fs[0], i_commit, fs[nf-1], nf - i_commit);

        // [F2] 自动轮播（next_auto_in=1 → 方块溶解）：音频与手动相同的 淡出 → 0 → 淡入，画面走溶解
        repeat (2 * FRAME) @(negedge clk);
        nf = 0; sampling = 1; dis_seen = 0;
        @(negedge cclk) begin next_ev = 1; next_auto = 1; end @(negedge cclk) begin next_ev = 0; next_auto = 0; end
        wait (next_out);
        repeat (1 * FRAME) @(negedge clk);                // 前瞻命中：只等 1 帧
        if (gain != 0) fail("[F2] gain not 0 in HOLD");
        wait (frame_start); @(negedge clk) commit = 1; @(negedge clk) commit = 0;
        i_commit = nf;
        wait (!fade_active); repeat (3 * FRAME) @(negedge clk);
        sampling = 0;
        if (!dis_seen) fail("[F2] dissolve_active never raised");
        mono = 1; steps = 0;
        for (k = 1; k < i_commit; k = k + 1) if (fs[k] > fs[k-1]) mono = 0;
        if (!mono) fail("[F2] fade-out not monotonic");
        for (k = i_commit + 1; k < nf; k = k + 1) begin
            if (fs[k] < fs[k-1]) mono = 0;
            if (fs[k] != fs[k-1]) steps = steps + 1;
        end
        if (!mono) fail("[F2] fade-in not monotonic");
        // 溶解加速（DISSOLVE_STEP=2）：淡入从 32 起、每帧 +32，7 步回 255
        if (steps < N / 2 - 1) begin fail("[F2] too few fade steps"); $display("     steps=%0d", steps); end
        if (fs[0] != 255 || fs[i_commit - 1] != 0 || fs[i_commit] > 32 || fs[nf-1] != 255) fail("[F2] endpoints");
        $display("INFO [F2] auto dissolve: gain %0d -> 0 (%0d frames to commit) -> %0d after %0d frames (%0d steps)",
                 fs[0], i_commit, fs[nf-1], nf - i_commit, steps);

        if (errors == 0) $display("PASS tb_wav_av_fade");
        else             $display("FAIL tb_wav_av_fade: %0d errors", errors);
        $finish;
    end
    initial begin #20_000_000; $display("FAIL tb_wav_av_fade: timeout"); $finish; end
endmodule
