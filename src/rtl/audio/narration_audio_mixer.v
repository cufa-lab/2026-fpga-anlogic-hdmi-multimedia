`timescale 1ns/1ps
// -----------------------------------------------------------------------------
//   背景乐 m（48kHz，wav_stream_player 握手样本） × gm  +  人声 v（24kHz，第二个 wav_stream_player） × gv  → 饱和到 16 位
//   gm = (mvol × fade + 255) >> 8，gv = (vvol × fade + 255) >> 8，每帧（frame_start）算一次；fade = sync_fade_gain 转场增益
//   mvol 目标：人声在播（voice_active && !pause）时 volume/2（ducking），否则 volume；每帧 ±MSTEP 平滑
//   vvol 目标：人声在播时 volume，否则 0；每帧 ±VSTEP（暂停约 0.27 s 收掉）
//   人声 24k → 48k：每来一个人声样本，下一个背景乐样本用 (上一个 + 这一个)/2，再下一个用这一个（线性插值，0 乘法）
//   人声不在播（voice_active=0）或 vvol 已降到 0 → 人声寄存器清 0（不留直流）
//
// 握手：背景乐侧 PCM 合同不变（valid 保持到 ready、数据期间不变）。本模块收下样本后 3 拍算完再给 out_valid，
//   下游 ready 时同拍回 m_ready（播放器在这一拍推进），平均速率仍等于背景乐的 48kHz。
//   人声侧 v_ready 恒 1（每个 24k 节拍收一个）。
// 乘法器：同一个 16x9 有符号乘法（TD 放 1 个 DSP）：帧首 2 次算增益，每个样本 2 次缩放。
// -----------------------------------------------------------------------------
module narration_audio_mixer #(
    parameter [7:0] MSTEP = 8'd4,             // 背景乐 ducking 每帧步进（约 1 s 压到一半）
    parameter [7:0] VSTEP = 8'd16
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               frame_start,
    input  wire [7:0]         volume,        // 用户音量（已含"首图提交前静音"）
    input  wire [7:0]         fade_gain,
    input  wire               pause,         // SW2
    input  wire               voice_active,  // 人声播放器 stream_active
    // 背景乐
    input  wire               m_valid,
    output wire               m_ready,
    input  wire signed [15:0] m_sample,
    // 人声
    input  wire               v_valid,
    output wire               v_ready,
    input  wire signed [15:0] v_sample,
    // 输出
    output wire               out_valid,
    input  wire               out_ready,
    output reg  signed [15:0] out_sample,
    output wire               ducking
);
    localparam [2:0] ST_IDLE = 3'd0, ST_G1 = 3'd1, ST_G2 = 3'd2, ST_P1 = 3'd3, ST_P2 = 3'd4,
                     ST_P3 = 3'd5, ST_OUT = 3'd6;

    reg [2:0]  st;
    reg        gain_pend;
    reg [7:0]  mvol, vvol, gm, gv;
    reg signed [15:0] v_prev, v_cur, mreg, vv;
    reg        fresh;
    reg signed [24:0] prod;
    reg signed [16:0] sum_m;

    wire speak = voice_active && !pause;
    wire [7:0] m_tgt = speak ? {1'b0, volume[7:1]} : volume;
    wire [7:0] v_tgt = speak ? volume : 8'd0;
    // 一个 9 位差 d = tgt - cur 同时给出方向和距离（原写法 4 次比较 / 减法）：
    //   d > stp → cur + stp；d < -stp → cur - stp；否则（含 d = 0）直接到 tgt（stp >= 1）
    function [7:0] ramp; input [7:0] cur; input [7:0] tgt; input [7:0] stp;
        reg [8:0] d;
        begin
            d = {1'b0, tgt} - {1'b0, cur};
            ramp = (!d[8] && d[7:0] > stp)                 ? cur + stp :
                   ( d[8] && d < 9'd0 - {1'b0, stp})       ? cur - stp : tgt;
        end
    endfunction
    assign ducking   = speak;
    assign v_ready   = 1'b1;
    assign out_valid = (st == ST_OUT);
    assign m_ready   = (st == ST_OUT) && out_ready;

    // ---- 共享乘法器 ----
    reg signed [15:0] mul_a;
    reg        [7:0]  mul_b;
    always @* begin
        case (st)
            ST_IDLE: begin mul_a = {8'd0, mvol}; mul_b = fade_gain; end   // 进 G1 时算 gm
            ST_G1:   begin mul_a = {8'd0, vvol}; mul_b = fade_gain; end
            ST_P1:   begin mul_a = mreg;         mul_b = gm;        end
            default: begin mul_a = vv;           mul_b = gv;        end   // ST_P2
        endcase
    end
    wire signed [24:0] mul_p = mul_a * $signed({1'b0, mul_b});
    wire [7:0]  g_q    = (prod[15:0] + 16'd255) >> 8;                  // 增益：(a*b + 255) >> 8
    wire signed [16:0] p_sh = prod[24:8];                              // 样本：>>> 8
    wire signed [17:0] mix_s = {sum_m[16], sum_m} + {p_sh[16], p_sh};

    // ---- 人声寄存器 + 插值 ----
    wire signed [16:0] mid = {v_prev[15], v_prev} + {v_cur[15], v_cur};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= ST_IDLE; gain_pend <= 1'b1;
            mvol <= 8'd0; vvol <= 8'd0; gm <= 8'd0; gv <= 8'd0;
            v_prev <= 16'sd0; v_cur <= 16'sd0; mreg <= 16'sd0; vv <= 16'sd0; fresh <= 1'b0;
            prod <= 25'sd0; sum_m <= 17'sd0; out_sample <= 16'sd0;
        end else begin
            if (frame_start) begin
                gain_pend <= 1'b1;
                mvol <= ramp(mvol, m_tgt, MSTEP);              // 音量目标平滑
                vvol <= ramp(vvol, v_tgt, VSTEP);
            end

            if (!voice_active || (vvol == 8'd0 && !speak)) begin
                v_prev <= 16'sd0; v_cur <= 16'sd0; fresh <= 1'b0;
            end else if (v_valid) begin
                v_prev <= v_cur; v_cur <= v_sample; fresh <= 1'b1;
            end

            prod <= mul_p;
            case (st)
            ST_IDLE: begin
                if (gain_pend && !frame_start) begin
                    gain_pend <= 1'b0;
                    st <= ST_G1;                              // 本拍乘法 = mvol*fade
                end else if (m_valid) begin
                    mreg <= m_sample;
                    if (fresh) begin vv <= mid[16:1]; fresh <= v_valid; end
                    else          vv <= v_cur;
                    st <= ST_P1;
                end
            end
            ST_G1: begin gm <= g_q; st <= ST_G2; end          // prod = mvol*fade；本拍乘法 = vvol*fade
            ST_G2: begin gv <= g_q; st <= ST_IDLE; end        // prod = vvol*fade
            ST_P1: st <= ST_P2;                               // 本拍乘法 = m*gm
            ST_P2: begin sum_m <= p_sh; st <= ST_P3; end      // prod = m*gm；本拍乘法 = vv*gv
            ST_P3: begin
                // prod = vv*gv；和只算一次，高 3 位不一致 = 超出 16 位 → 按符号饱和
                if (mix_s[17:15] == 3'b000 || mix_s[17:15] == 3'b111) out_sample <= mix_s[15:0];
                else out_sample <= mix_s[17] ? -16'sd32768 : 16'sd32767;
                st <= ST_OUT;
            end
            ST_OUT: if (out_ready) st <= ST_IDLE;
            default: st <= ST_IDLE;
            endcase
        end
    end
endmodule
