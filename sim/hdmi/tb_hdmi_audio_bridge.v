`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// tb_hdmi_audio_bridge：音频数据源 → PCM 合同 → HDMI 符号层 的系统级验证
// 激励：真实的 video_timing_data + video_delay(与顶层同一条时序链)，
//       像素数据为随机数(read_en 推进)，经 video_x_tagger 送入 media_hdmi_audio_bridge。
// 监视：对 3 个 TMDS 通道输出逐字符做独立解析(参考模型见 tb_hdmi_common.vh)：
//   [V1] 有效像素：TMDS 解码 == 输入 RGB；且与独立 TMDS 编码器参考模型逐字符一致
//   [V2] 每行 640 像素、每帧 480 行；视频前导 8 字符 + 保护带 2 字符 只出现在有效行之前
//   [V3] 控制周期 ch0 = {VS,HS}；只允许 3 种 CTL 组合；前导之前控制周期 >= 12 字符
//   [D1] 数据岛：前导 8 + 保护带 2 + 32*n TERC4 + 保护带 2；岛内 ch0 D1:D0 = {VS,HS}
//   [D2] 包头/子包 ECC == 独立 BCH 参考模型
//   [A1] Audio Sample 包：16bit 左对齐、P 校验、C 位(48k/16bit)、B 帧起点、192 帧计数
//   [A2] 解出的 PCM 序列 == symbol core 实际接收的 PCM 序列(不丢、不重、不乱序)
//   [A3] 采样率 48 kHz；ACR: N=6144 CTS=25000，每 25000 拍一次
//   [A4] Audio InfoFrame：每帧 1 个、校验和为 0、CC=2ch
//   [E ] sequence_error / tone_overflow / pcm_contract_error 始终为 0
// -----------------------------------------------------------------------------
module tb_hdmi_audio_bridge;
    `include "tb_hdmi_common.vh"

    parameter integer FRAMES = 4;
    localparam integer LAT = 2;      // bridge 输入寄存 1 拍 + 编码器输出寄存 1 拍

    reg clk = 1'b0;
    always #20 clk = ~clk;           // 25 MHz 像素时钟
    reg rst_n = 1'b0;

    // ---------------- 与顶层一致的视频时序链 ----------------
    wire read_req, hs_0, vs_0, de_0, hs_d, vs_d, de_d, read_en;
    reg  read_req_ack = 1'b0;
    reg  [31:0] pix_lfsr = 32'h1234_5678;
    wire [23:0] vout_raw;
    always @(posedge clk) begin
        read_req_ack <= read_req;
        if (read_en) pix_lfsr <= {pix_lfsr[30:0], pix_lfsr[31] ^ pix_lfsr[21] ^ pix_lfsr[1] ^ pix_lfsr[0]};
    end

    video_timing_data u_timing (
        .video_clk(clk), .rst_n(rst_n), .read_req(read_req), .read_req_ack(read_req_ack),
        .hs(hs_0), .vs(vs_0), .de(de_0)
    );
    video_delay u_delay (
        .video_clk(clk), .rst_n(rst_n), .read_en(read_en), .read_data(pix_lfsr[31:8]),
        .hs(hs_0), .vs(vs_0), .de(de_0), .hs_r(hs_d), .vs_r(vs_d), .de_r(de_d), .vout_data(vout_raw)
    );

    wire hs, vs, de;
    wire [23:0] rgb;
    wire [9:0] vx;
    video_x_tagger u_tag (
        .clk(clk), .rst_n(rst_n), .hs_i(hs_d), .vs_i(vs_d), .de_i(de_d), .rgb_i(vout_raw),
        .hs_o(hs), .vs_o(vs), .de_o(de), .rgb_o(rgb), .video_x_o(vx), .locked_o()
    );

    reg [1:0] media_id = 2'd0;
    wire [9:0] c0, c1, c2;
    wire seq_err, tone_ovf, pcm_err, blip, tap_valid;
    wire signed [15:0] tap_left;
    media_hdmi_audio_bridge dut (
        .pixel_clk(clk), .pixel_rst_n(rst_n),
        .video_hsync(hs), .video_vsync(vs), .video_de(de), .video_x(vx), .video_rgb(rgb),
        .media_id(media_id), .volume(8'd255),
        .wav_valid(1'b0), .wav_ready(), .wav_sample(16'sd0), .wav_active(1'b0), .force_arpeggio(1'b0),
        .test_tone_enable(1'b0), .audio_source(),
        .channel0_symbol(c0), .channel1_symbol(c1), .channel2_symbol(c2),
        .sequence_error(seq_err), .tone_overflow(tone_ovf), .pcm_contract_error(pcm_err),
        .blip_active(blip), .pcm_tap_valid(tap_valid), .pcm_tap_left(tap_left)
    );

    // ---------------- 参考对齐 ----------------
    reg [LAT*27-1:0] ref_pipe;
    always @(posedge clk) ref_pipe <= {ref_pipe[(LAT-1)*27-1:0], hs, vs, de, rgb};
    wire        hs_r  = ref_pipe[LAT*27-1];
    wire        vs_r  = ref_pipe[LAT*27-2];
    wire        de_r  = ref_pipe[LAT*27-3];
    wire [23:0] rgb_r = ref_pipe[LAT*27-4 -: 24];

    // ---------------- 独立 TMDS 视频编码参考模型(DVI 1.0 3.3.3) ----------------
    integer rd [0:2];
    function [9:0] tmds_enc_ref;
        input [7:0] d;
        input integer ch;
        integer n1, n1q, n0q, i;
        reg [8:0] q_m;
        reg [9:0] q_out;
        begin
            n1 = 0; for (i = 0; i < 8; i = i + 1) n1 = n1 + d[i];
            q_m[0] = d[0];
            if (n1 > 4 || (n1 == 4 && d[0] == 1'b0)) begin
                for (i = 1; i < 8; i = i + 1) q_m[i] = ~(q_m[i-1] ^ d[i]);
                q_m[8] = 1'b0;
            end else begin
                for (i = 1; i < 8; i = i + 1) q_m[i] = q_m[i-1] ^ d[i];
                q_m[8] = 1'b1;
            end
            n1q = 0; for (i = 0; i < 8; i = i + 1) n1q = n1q + q_m[i];
            n0q = 8 - n1q;
            if (rd[ch] == 0 || n1q == n0q) begin
                q_out = {~q_m[8], q_m[8], q_m[8] ? q_m[7:0] : ~q_m[7:0]};
                if (q_m[8] == 1'b0) rd[ch] = rd[ch] + (n0q - n1q);
                else                rd[ch] = rd[ch] + (n1q - n0q);
            end else if ((rd[ch] > 0 && n1q > n0q) || (rd[ch] < 0 && n0q > n1q)) begin
                q_out = {1'b1, q_m[8], ~q_m[7:0]};
                rd[ch] = rd[ch] + 2 * q_m[8] + (n0q - n1q);
            end else begin
                q_out = {1'b0, q_m[8], q_m[7:0]};
                rd[ch] = rd[ch] - 2 * (~q_m[8] & 1'b1) + (n1q - n0q);
            end
            tmds_enc_ref = q_out;
        end
    endfunction

    // ---------------- 字符分类 ----------------
    localparam [9:0] VGB0 = 10'b1011001100, VGB1 = 10'b0100110011, VGB2 = 10'b1011001100;
    localparam [9:0] DGB  = 10'b0100110011;
    wire [4:0] t0 = terc4_dec(c0), t1 = terc4_dec(c1), t2 = terc4_dec(c2);
    wire [2:0] k0 = ctrl_dec(c0),  k1 = ctrl_dec(c1),  k2 = ctrl_dec(c2);
    wire is_vgb  = (c0 == VGB0) && (c1 == VGB1) && (c2 == VGB2);
    wire is_dgb  = t0[4] && (c1 == DGB) && (c2 == DGB);
    wire is_ctrl = k0[2] && k1[2] && k2[2];
    wire is_isl  = t0[4] && t1[4] && t2[4];

    // ---------------- 统计 ----------------
    reg started = 1'b0;             // 第 1 个 vsync 下降沿之后开始严格检查
    integer vs_edges = 0;           // video_x_tagger 锁定前输出空闲，复位后不再有伪 vsync 沿
    integer errors = 0, max_err_print = 30;
    task err(input [8*96-1:0] msg);
        begin
            if (started) begin errors = errors + 1;
            if (errors <= max_err_print) $display("FAIL @%0t: %0s", $time, msg); end
        end
    endtask

    localparam S_CTRL = 0, S_VGB = 1, S_VID = 2, S_DLEAD = 3, S_ISL = 4, S_DTRAIL = 5;
    integer st = S_CTRL, run = 0, ctrl_run = 0, vpre_run = 0, dpre_run = 0;
    integer vid_px = 0, vid_lines = 0, vgb_chars = 0, vpre_chars = 0;
    integer frames = 0, lines_in_frame = 0, bad_line_px = 0;
    integer isl_idx = 0, n_islands = 0, n_pkts = 0, n_audio = 0, n_acr = 0, n_info = 0, info_in_frame = 0;
    integer vid_chars = 0, vid_ref_mismatch = 0, vid_dec_mismatch = 0;
    reg vs_r_q = 1'b1;
    integer ticks = 0;
    always @(posedge clk) if (rst_n && dut.u_tone.sample_tick) ticks = ticks + 1;

    reg [31:0] hdr;
    reg [63:0] sp [0:3];

    // PCM 参考队列(symbol core 实际接收的采样)
    reg signed [15:0] pcmq [0:1023];
    integer q_wr = 0, q_rd = 0, pcm_ok = 0;
    always @(posedge clk) if (rst_n && tap_valid) begin
        pcmq[q_wr % 1024] = tap_left;
        q_wr = q_wr + 1;
    end

    // 音频帧计数
    integer frame_idx = -1;
    integer acr_last_t = -1, acr_int_min = 1 << 30, acr_int_max = 0, cyc = 0;
    integer n_acr_started = 0, acr_span = 0, acr_t0 = -1;
    integer first_frame_cyc = 0, first_frame_samples = 0, last_frame_cyc = 0, last_frame_samples = 0;

    task check_packet;
        integer k, j, sum, fi;
        reg [23:0] lw, rw;
        reg [7:0] b6;
        reg cl, cr;
        begin
            n_pkts = n_pkts + 1;
            if (hdr[31:24] !== ref_bch8({32'd0, hdr[23:0]}, 24)) err("header ECC mismatch");
            for (k = 0; k < 4; k = k + 1)
                if (sp[k][63:56] !== ref_bch8(sp[k][55:0], 56)) err("subpacket ECC mismatch");
            case (hdr[7:0])
                8'h02: begin
                    n_audio = n_audio + 1;
                    if (hdr[15:8] !== 8'h0f) err("audio sample HB1 != 0x0F (4 samples, layout 0)");
                    if (hdr[23:16] & 8'h0f) err("audio sample HB2 reserved bits");
                    for (k = 0; k < 4; k = k + 1) begin
                        lw = sp[k][23:0]; rw = sp[k][47:24]; b6 = sp[k][55:48];
                        if (lw[7:0] !== 8'h00 || rw[7:0] !== 8'h00) err("16-bit sample not left-justified in 24 bits");
                        if (lw !== rw) err("L != R (mono tone)");
                        // B 位 / 192 帧计数
                        if (hdr[16 + 4 + k]) begin
                            if (frame_idx >= 0 && frame_idx != 191) err("B flag not at frame 192 boundary");
                            frame_idx = 0;
                        end else if (frame_idx >= 0) begin
                            frame_idx = frame_idx + 1;
                            if (frame_idx > 191) err("192-frame block without B flag");
                        end
                        fi = frame_idx;
                        cl = b6[2]; cr = b6[6];
                        if (b6[1] || b6[0] || b6[5] || b6[4]) err("U/V bits must be 0");
                        if (b6[3] !== ^{lw, b6[2:0]}) err("left parity");
                        if (b6[7] !== ^{rw, b6[6:4]}) err("right parity");
                        if (fi >= 0 && (cl !== (fi == 25 || fi == 33) || cr !== cl)) err("channel status bit (C25=48kHz, C33=16bit)");
                        // PCM 连续性
                        if (q_rd >= q_wr) err("audio packet carries a sample never accepted");
                        else begin
                            if ($signed(lw[23:8]) !== pcmq[q_rd % 1024]) err("PCM sample order/value mismatch");
                            else pcm_ok = pcm_ok + 1;
                            q_rd = q_rd + 1;
                        end
                    end
                end
                8'h01: begin
                    n_acr = n_acr + 1;
                    if (hdr[23:8] !== 16'h0000) err("ACR HB1/HB2");
                    for (k = 0; k < 4; k = k + 1) begin
                        if (sp[k][7:0] !== 8'h00) err("ACR SB0");
                        if ({sp[k][11:8], sp[k][23:16], sp[k][31:24]} !== 20'd25000) err("ACR CTS != 25000");
                        if ({sp[k][35:32], sp[k][47:40], sp[k][55:48]} !== 20'd6144) err("ACR N != 6144");
                        if (sp[k][15:12] !== 4'h0 || sp[k][39:36] !== 4'h0) err("ACR reserved nibbles");
                    end
                    if (started) begin
                        if (acr_t0 < 0) acr_t0 = cyc; else n_acr_started = n_acr_started + 1;
                        acr_span = cyc - acr_t0;
                    end
                    if (acr_last_t >= 0 && started) begin
                        if (cyc - acr_last_t < acr_int_min) acr_int_min = cyc - acr_last_t;
                        if (cyc - acr_last_t > acr_int_max) acr_int_max = cyc - acr_last_t;
                    end
                    acr_last_t = cyc;
                end
                8'h84: begin
                    n_info = n_info + 1;
                    info_in_frame = info_in_frame + 1;
                    if (hdr[23:8] !== 16'h0a01) err("Audio InfoFrame HB1/HB2 (version 1, length 10)");
                    sum = hdr[7:0] + hdr[15:8] + hdr[23:16];
                    for (k = 0; k < 4; k = k + 1)
                        for (j = 0; j < 7; j = j + 1) sum = sum + sp[k][j*8 +: 8];
                    if (sum % 256 != 0) err("Audio InfoFrame checksum");
                    if (sp[0][15:8] !== 8'h01) err("Audio InfoFrame PB1 (CT=0, CC=1 -> 2ch)");
                end
                default: err("unexpected packet type");
            endcase
        end
    endtask

    integer ch;
    reg [9:0] ref_sym [0:2];
    always @(posedge clk) begin
        cyc = cyc + 1;
        if (rst_n && cyc > LAT + 2) begin
            // 帧边界(vsync 下降沿)
            if (vs_r_q && !vs_r) begin
                if (started) begin
                    frames = frames + 1;
                    if (lines_in_frame != 480) err("active lines per frame != 480");
                    if (info_in_frame != 1) err("Audio InfoFrame count per frame != 1");
                    if (frames == 1) begin first_frame_cyc = cyc; first_frame_samples = ticks; end
                    last_frame_cyc = cyc; last_frame_samples = ticks;
                end
                vs_edges = vs_edges + 1;
                if (vs_edges >= 1) started = 1'b1;
                lines_in_frame = 0;
                info_in_frame = 0;
            end
            vs_r_q = vs_r;

            // 视频字符：解码 + 参考编码
            if (de_r) begin
                ref_sym[0] = tmds_enc_ref(rgb_r[7:0], 0);
                ref_sym[1] = tmds_enc_ref(rgb_r[15:8], 1);
                ref_sym[2] = tmds_enc_ref(rgb_r[23:16], 2);
                vid_chars = vid_chars + 1;
                if ({tmds_dec(c2), tmds_dec(c1), tmds_dec(c0)} !== rgb_r) begin
                    vid_dec_mismatch = vid_dec_mismatch + 1; err("video TMDS decode != input RGB");
                end
                if (c0 !== ref_sym[0] || c1 !== ref_sym[1] || c2 !== ref_sym[2]) begin
                    vid_ref_mismatch = vid_ref_mismatch + 1; err("video symbol != reference TMDS encoder");
                end
            end else begin
                for (ch = 0; ch < 3; ch = ch + 1) rd[ch] = 0;
            end

            // 字符语法
            case (st)
                S_CTRL: begin
                    if (de_r) begin
                        if (started) err("video period without preamble/guard");
                        st = S_VID; vid_px = 1;
                    end else if (is_vgb) begin
                        if (started && vpre_run != 8) err("video guard not preceded by exactly 8 video preamble chars");
                        if (started && ctrl_run < 12) err("control period before video < 12 chars");
                        vgb_chars = vgb_chars + 1; st = S_VGB; run = 1;
                    end else if (is_dgb) begin
                        if (dpre_run != 8) err("data island guard not preceded by exactly 8 preamble chars");
                        if (ctrl_run < 12) err("control period before data island < 12 chars");
                        if (t0[3:2] !== 2'b11 || t0[1:0] !== {vs_r, hs_r}) err("data guard ch0 != TERC4{1,1,VS,HS}");
                        st = S_DLEAD; run = 1;
                    end else if (is_ctrl) begin
                        if (k0[1:0] !== {vs_r, hs_r}) err("control ch0 != {VS,HS}");
                        if ({k1[1:0], k2[1:0]} == 4'b0100) begin
                            vpre_run = vpre_run + 1; dpre_run = 0; vpre_chars = vpre_chars + 1;
                        end else if ({k1[1:0], k2[1:0]} == 4'b0101) begin
                            dpre_run = dpre_run + 1; vpre_run = 0;
                        end else if ({k1[1:0], k2[1:0]} == 4'b0000) begin
                            if (vpre_run != 0 || dpre_run != 0) err("preamble not followed by guard band");
                            vpre_run = 0; dpre_run = 0;
                        end else err("illegal CTL combination");
                        ctrl_run = ctrl_run + 1;
                    end else err("illegal character in control period");
                end
                S_VGB: begin
                    if (is_vgb) begin
                        run = run + 1; vgb_chars = vgb_chars + 1;
                        if (run > 2) err("video guard band longer than 2");
                    end else if (de_r) begin
                        if (run != 2) err("video guard band length != 2");
                        st = S_VID; vid_px = 1;
                    end else begin
                        err("video guard band not followed by video"); st = S_CTRL;
                    end
                end
                S_VID: begin
                    if (de_r) vid_px = vid_px + 1;
                    else begin
                        if (vid_px != 640) begin bad_line_px = bad_line_px + 1; err("active line length != 640"); end
                        vid_lines = vid_lines + 1; lines_in_frame = lines_in_frame + 1;
                        if (!is_ctrl) err("video period not followed by control period");
                        st = S_CTRL; ctrl_run = 1; vpre_run = 0; dpre_run = 0;
                        if (is_ctrl && k0[1:0] !== {vs_r, hs_r}) err("control ch0 != {VS,HS}");
                    end
                end
                S_DLEAD: begin
                    if (is_dgb) begin
                        run = run + 1;
                        if (run > 2) err("data island leading guard > 2");
                        if (t0[3:2] !== 2'b11 || t0[1:0] !== {vs_r, hs_r}) err("data guard ch0 != TERC4{1,1,VS,HS}");
                    end else if (is_isl) begin
                        if (run != 2) err("data island leading guard != 2");
                        st = S_ISL; isl_idx = 0; n_islands = n_islands + 1;
                    end else begin err("data island guard not followed by TERC4"); st = S_CTRL; end
                end
                S_DTRAIL: begin
                    if (is_dgb) begin
                        run = run + 1;
                        if (run > 2) err("data island trailing guard > 2");
                        if (t0[3:2] !== 2'b11 || t0[1:0] !== {vs_r, hs_r}) err("data guard ch0 != TERC4{1,1,VS,HS}");
                    end else begin
                        if (run != 2) err("data island trailing guard != 2");
                        if (!is_ctrl || de_r) err("data island not followed by control period");
                        st = S_CTRL; ctrl_run = 1; vpre_run = 0; dpre_run = 0;
                    end
                end
                default: ;
            endcase

            // 数据岛负载(与语法状态机同拍处理，S_DLEAD→S_ISL 的第一个字符也在此处)
            if (st == S_ISL) begin
                if (is_isl && !de_r) begin
                    if (t0[1:0] !== {vs_r, hs_r}) err("data island ch0 D1:D0 != {VS,HS}");
                    hdr[isl_idx % 32] = t0[2];
                    for (ch = 0; ch < 4; ch = ch + 1) begin
                        sp[ch][2 * (isl_idx % 32)]     = t1[ch];
                        sp[ch][2 * (isl_idx % 32) + 1] = t2[ch];
                    end
                    if (isl_idx % 32 == 31) check_packet;
                    isl_idx = isl_idx + 1;
                end else if (is_dgb) begin
                    if (isl_idx % 32 != 0) err("data island payload not a multiple of 32 chars");
                    st = S_DTRAIL; run = 1;
                    if (t0[3:2] !== 2'b11 || t0[1:0] !== {vs_r, hs_r}) err("data guard ch0 != TERC4{1,1,VS,HS}");
                end else begin
                    err("illegal character inside data island"); st = S_CTRL;
                end
            end

            if (seq_err)  err("sequence_error asserted");
            if (tone_ovf) err("tone_overflow asserted");
            if (pcm_err)  err("pcm_contract_error asserted");
        end
    end

    real fs;
    initial begin
        rd[0] = 0; rd[1] = 0; rd[2] = 0;
        repeat (10) @(posedge clk);
        rst_n = 1'b1;
        // 跑到第 2 帧中间切换 media_id(触发提示音 + 换旋律)
        wait (frames == 2);
        repeat (200000) @(posedge clk);
        @(negedge clk) media_id = 2'd3;
        wait (frames == FRAMES + 1);
        repeat (10) @(posedge clk);

        fs = (last_frame_samples - first_frame_samples) * 25.0e6 / (last_frame_cyc - first_frame_cyc);
        $display("INFO frames=%0d active_lines=%0d video_chars=%0d vgb_chars=%0d vpre_chars=%0d",
                 frames, vid_lines, vid_chars, vgb_chars, vpre_chars);
        $display("INFO islands=%0d packets=%0d audio=%0d acr=%0d infoframe=%0d pcm_matched=%0d (queued %0d)",
                 n_islands, n_pkts, n_audio, n_acr, n_info, pcm_ok, q_wr - q_rd);
        $display("INFO sample_tick rate %0.3f Hz, ACR interval min=%0d max=%0d cycles, %0d ACR in %0d cycles", fs, acr_int_min, acr_int_max, n_acr_started, acr_span);
        // 汇总判据
        // 含上电后第一条有效行：不允许任何一行缺视频前导/保护带
        if (vgb_chars != 2 * vid_lines) err("video guard count != 2 per active line");
        if (vpre_chars != 8 * vid_lines) err("video preamble count != 8 per active line");
        if ((last_frame_samples - first_frame_samples) * 25000000.0 < (last_frame_cyc - first_frame_cyc) * 48000.0 - 25000000.0 ||
            (last_frame_samples - first_frame_samples) * 25000000.0 > (last_frame_cyc - first_frame_cyc) * 48000.0 + 25000000.0)
            err("audio sample tick count != cycles*48000/25e6 (+-1)");
        if (pcm_ok < (last_frame_samples - first_frame_samples)) err("fewer samples packetized than generated");
        // ACR 每 25000 拍排队一次，等岛位(每行 1 个)最多延迟数行
        if (acr_int_min < 25000 - 3 * 800 || acr_int_max > 25000 + 3 * 800) err("ACR interval not ~25000 cycles");
        if (n_acr_started * 25000 < acr_span - 25000 || n_acr_started * 25000 > acr_span + 25000) err("ACR average != 1 per 25000 cycles");
        if (n_acr == 0 || n_info == 0 || n_audio == 0) err("missing packet type");
        if (q_wr - q_rd > 16) err("PCM backlog grows (packets cannot keep up)");
        if (errors == 0) $display("PASS tb_hdmi_audio_bridge: video integrity + HDMI grammar + packets/ECC + PCM continuity + 48 kHz/ACR/InfoFrame");
        else             $display("FAIL tb_hdmi_audio_bridge: %0d errors", errors);
        $finish;
    end

    initial begin
        #(40.0 * 800 * 525 * (FRAMES + 3));
        $display("FAIL tb_hdmi_audio_bridge: timeout");
        $finish;
    end
endmodule
