`timescale 1ns/1ps

// Audio-video linked melody source for the player. Each loaded image bank
// plays its own six-note arpeggio loop (C6 chord, Am7, pentatonic fall,
// Fmaj7), so a picture switch is audible as a new melody after a
// short 1.5 kHz cue blip, and the persisted volume setting scales the output.
// The FIFO/valid/ready contract matches hdmi_pcm_tone_fifo so the audio
// packetizer chain is unchanged.
//
// L15（第二批优化）：
//   * EXT_TICK=1 时用外部 48 kHz 节拍 tick48（顶层 wav_tick，同钟同复位、逐拍相同），去掉本地 25 位累加器；
//   * 8 深样本 FIFO 改为"节拍欠账计数"：样本只由 phase/音量决定，所以相位、音符推进挪到样本被取走时，
//     样本在输出端现算。欠账同样封顶 8（满后丢节拍，与原 FIFO 满时相同）。音量与 media_id 在
//     欠账窗口（≤8 个样本）内不变时，输出序列与原版逐样本相同；
//   * 删掉不可能触发的饱和：|12000·255| = 3 060 000 < 8 388 352，scaled>>>8 恒在 16 位内。
//   不被选中时（WAV 播放）ready=0，欠账满 8 后状态完全冻结（P4：琶音停振，原版即如此）。
// 换旋律（仍是 16 点正弦）：4 段旋律整体降到 A3~E5 中低音区、每音 250 ms → 300 ms，听感更柔和；
//   相位增量按平均律精确值重算。结构、提示音、48 kHz 节拍、握手合同都不变，资源持平。
module pcm_media_tone #(
    parameter integer EXT_TICK = 0,
    parameter integer CLOCK_HZ = 25000000,
    parameter integer SAMPLE_HZ = 48000,
    parameter [19:0] BLIP_INC = 20'd32768,   // ~1500 Hz switch cue
    parameter integer BLIP_SAMPLES = 4800,   // 100 ms at 48 kHz
    parameter integer NOTE_SAMPLES = 14400   // 300 ms per melody note
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               tick48,         // EXT_TICK=1 时的外部 48 kHz 节拍
    input  wire [1:0]         media_id,
    input  wire [7:0]         volume,
    output wire               sample_valid,
    input  wire               sample_ready,
    output wire signed [15:0] sample_left,
    output wire signed [15:0] sample_right,
    output reg                overflow,
    output wire               blip_active
);
    reg [19:0] phase;
    reg [1:0] media_id_q;
    reg [12:0] blip_countdown;
    reg [2:0] note_index;
    reg [14:0] note_sample_count;
    reg [3:0] credits;                       // 已到节拍、尚未被取走的样本数（0..8）
    wire sample_pop;
    wire sample_tick;
    wire sample_push;                        // 状态推进（= 取走一个样本）

    assign sample_valid = credits != 0;
    assign sample_pop = sample_valid && sample_ready;
    assign sample_push = sample_pop;
    assign blip_active = blip_countdown != 0;

    generate
        if (EXT_TICK != 0) begin : g_ext_tick
            assign sample_tick = tick48;
        end else begin : g_int_tick
            reg [24:0] rate_accumulator;
            assign sample_tick = rate_accumulator >= CLOCK_HZ - SAMPLE_HZ;
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n)
                    rate_accumulator <= 25'd0;
                else if (sample_tick)
                    rate_accumulator <= rate_accumulator + SAMPLE_HZ - CLOCK_HZ;
                else
                    rate_accumulator <= rate_accumulator + SAMPLE_HZ;
            end
        end
    endgenerate
    wire tick_take = sample_tick && (credits != 4'd8 || sample_pop);

    // phase_inc = freq * 2**20 / SAMPLE_HZ (freq * 21.8453)
    function [19:0] melody_inc;
        input [1:0] melody;
        input [2:0] note;
        begin
            case ({melody, note})
                // 0: C 大调六和弦起伏 C4 E4 G4 A4 G4 E4
                5'b00_000: melody_inc = 20'd5715;   // C4 261.6 Hz
                5'b00_001: melody_inc = 20'd7201;   // E4 329.6 Hz
                5'b00_010: melody_inc = 20'd8563;   // G4 392.0 Hz
                5'b00_011: melody_inc = 20'd9612;   // A4 440.0 Hz
                5'b00_100: melody_inc = 20'd8563;   // G4
                5'b00_101: melody_inc = 20'd7201;   // E4
                // 1: a 小调七和弦 A3 C4 E4 G4 E4 C4
                5'b01_000: melody_inc = 20'd4806;   // A3 220.0 Hz
                5'b01_001: melody_inc = 20'd5715;   // C4
                5'b01_010: melody_inc = 20'd7201;   // E4
                5'b01_011: melody_inc = 20'd8563;   // G4
                5'b01_100: melody_inc = 20'd7201;   // E4
                5'b01_101: melody_inc = 20'd5715;   // C4
                // 2: 五声音阶下行 E5 D5 C5 A4 G4 A4
                5'b10_000: melody_inc = 20'd14402;  // E5 659.3 Hz
                5'b10_001: melody_inc = 20'd12830;  // D5 587.3 Hz
                5'b10_010: melody_inc = 20'd11431;  // C5 523.3 Hz
                5'b10_011: melody_inc = 20'd9612;   // A4
                5'b10_100: melody_inc = 20'd8563;   // G4
                5'b10_101: melody_inc = 20'd9612;   // A4
                // 3: F 大七和弦 F4 A4 C5 E5 C5 A4
                5'b11_000: melody_inc = 20'd7629;   // F4 349.2 Hz
                5'b11_001: melody_inc = 20'd9612;   // A4
                5'b11_010: melody_inc = 20'd11431;  // C5
                5'b11_011: melody_inc = 20'd14402;  // E5
                5'b11_100: melody_inc = 20'd11431;  // C5
                5'b11_101: melody_inc = 20'd9612;   // A4
                default:   melody_inc = 20'd5715;
            endcase
        end
    endfunction

    wire [19:0] melody_note_inc = melody_inc(media_id_q, note_index);
    wire [19:0] phase_inc = (blip_countdown != 0) ? BLIP_INC : melody_note_inc;

    function signed [15:0] sine_sample;
        input [3:0] index;
        begin
            case (index)
                4'd0: sine_sample = 16'sd0;
                4'd1: sine_sample = 16'sd4592;
                4'd2: sine_sample = 16'sd8485;
                4'd3: sine_sample = 16'sd11087;
                4'd4: sine_sample = 16'sd12000;
                4'd5: sine_sample = 16'sd11087;
                4'd6: sine_sample = 16'sd8485;
                4'd7: sine_sample = 16'sd4592;
                4'd8: sine_sample = 16'sd0;
                4'd9: sine_sample = -16'sd4592;
                4'd10: sine_sample = -16'sd8485;
                4'd11: sine_sample = -16'sd11087;
                4'd12: sine_sample = -16'sd12000;
                4'd13: sine_sample = -16'sd11087;
                4'd14: sine_sample = -16'sd8485;
                default: sine_sample = -16'sd4592;
            endcase
        end
    endfunction

    wire signed [15:0] raw_sample = sine_sample(phase[19:16]);
    wire signed [23:0] scaled = raw_sample * $signed({1'b0, volume});
    wire signed [15:0] scaled_clamped = scaled[23:8];   // 名字保留给 tb；不会越界，无需饱和
    assign sample_left = scaled_clamped;
    assign sample_right = scaled_clamped;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            phase <= 20'd0;
            media_id_q <= 2'd0;
            blip_countdown <= 13'd0;
            note_index <= 3'd0;
            note_sample_count <= 15'd0;
            credits <= 4'd0;
            overflow <= 1'b0;
        end else begin
            media_id_q <= media_id;

            if (sample_push) begin
                phase <= phase + phase_inc;
                if (blip_countdown != 0) begin
                    blip_countdown <= blip_countdown - 1'b1;
                end else if (note_sample_count >= NOTE_SAMPLES - 1) begin
                    note_sample_count <= 15'd0;
                    note_index <= (note_index == 3'd5) ? 3'd0 : note_index + 1'b1;
                end else begin
                    note_sample_count <= note_sample_count + 1'b1;
                end
            end
            if (sample_tick && !tick_take)
                overflow <= 1'b1;

            case ({tick_take, sample_pop})
                2'b10: credits <= credits + 1'b1;
                2'b01: credits <= credits - 1'b1;
                default: credits <= credits;
            endcase

            // A switch reloads the cue blip and restarts the melody from its
            // first note. Load last so a reload wins over the same-edge
            // decrement or note advance.
            if (media_id != media_id_q) begin
                blip_countdown <= BLIP_SAMPLES[12:0];
                note_index <= 3'd0;
                note_sample_count <= 15'd0;
            end
        end
    end
endmodule
