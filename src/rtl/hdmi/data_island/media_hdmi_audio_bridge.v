`timescale 1ns/1ps

// Player-owned boundary between the registered video bundle and the verified
// Route B HDMI packet/symbol chain. The EG4 DDR PHY remains in the board Top so
// this bridge can be exercised in ordinary RTL simulation without vendor pads.
// media_id drives the per-image tone selection and volume scales the output,
// turning the tone source into the audio half of the AV-linked feature.

module media_hdmi_audio_bridge #(
    parameter integer TONE_EXT_TICK = 0
) (
    input  wire        pixel_clk,
    input  wire        pixel_rst_n,
    input  wire        tone_tick,
    input  wire        video_hsync,
    input  wire        video_vsync,
    input  wire        video_de,
    input  wire [9:0]  video_x,
    input  wire [23:0] video_rgb,
    input  wire [1:0]  media_id,
    input  wire [7:0]  volume,
    input  wire               wav_valid,
    output wire               wav_ready,
    input  wire signed [15:0] wav_sample,
    input  wire               wav_active,
    input  wire               force_arpeggio,
    input  wire               test_tone_enable,
    output wire [1:0]         audio_source,     // 0 = 琶音, 1 = WAV, 2 = 测试音
    output wire [9:0]  channel0_symbol,
    output wire [9:0]  channel1_symbol,
    output wire [9:0]  channel2_symbol,
    output wire        sequence_error,
    output wire        tone_overflow,
    output wire        pcm_contract_error,
    output wire        blip_active,
    output wire        pcm_tap_valid,
    output wire signed [15:0] pcm_tap_left
);
    wire source_valid;
    wire source_ready;
    wire signed [15:0] source_left;
    wire signed [15:0] source_right;
    wire sample_valid;
    wire sample_ready;
    wire signed [15:0] sample_left;
    wire signed [15:0] sample_right;

    reg video_hsync_q;
    reg video_vsync_q;
    reg video_de_q;
    reg [9:0] video_x_q;
    reg [23:0] video_rgb_q;
    always @(posedge pixel_clk or negedge pixel_rst_n) begin
        if (!pixel_rst_n) begin
            video_hsync_q <= 1'b1;
            video_vsync_q <= 1'b1;
            video_de_q <= 1'b0;
            video_x_q <= 10'd0;
        end else begin
            video_hsync_q <= video_hsync;
            video_vsync_q <= video_vsync;
            video_de_q <= video_de;
            video_x_q <= video_x;
        end
    end
    // 纯数据寄存器不接复位（下游/本级已按 de、锁定门控，复位期间时钟照常冲成 0）
    always @(posedge pixel_clk)
        video_rgb_q <= video_de ? video_rgb : 24'h000000;

    // ---- 琶音（第3课，原样）----
    wire arp_valid;
    wire arp_ready;
    wire signed [15:0] arp_left;
    wire signed [15:0] arp_right;

    pcm_media_tone #(
        .EXT_TICK(TONE_EXT_TICK)
    ) u_tone (
        .clk(pixel_clk), .rst_n(pixel_rst_n), .tick48(tone_tick),
        .media_id(media_id), .volume(volume),
        .sample_valid(arp_valid), .sample_ready(arp_ready),
        .sample_left(arp_left), .sample_right(arp_right),
        .overflow(tone_overflow), .blip_active(blip_active)
    );

    wire mix_valid;
    wire mix_ready;
    wire signed [15:0] mix_left;
    wire signed [15:0] mix_right;

    audio_source_mux u_src_mux (
        .clk(pixel_clk), .rst_n(pixel_rst_n),
        .wav_valid(wav_valid), .wav_ready(wav_ready),
        .wav_left(wav_sample), .wav_right(wav_sample),     // 单声道 PCM → 左右声道相同
        .wav_active(wav_active),
        .arp_valid(arp_valid), .arp_ready(arp_ready),
        .arp_left(arp_left), .arp_right(arp_right),
        .force_arpeggio(force_arpeggio),
        .out_valid(mix_valid), .out_ready(mix_ready),
        .out_left(mix_left), .out_right(mix_right)
    );


    wire test_valid;
    wire test_ready;
    wire signed [15:0] test_left;
    wire signed [15:0] test_right;

    pcm_test_tone #(
        .SAMPLE_DIV(521)                 // 25MHz / 521 ≈ 48kHz，16 点正弦 ≈ 3kHz
    ) u_test_tone (
        .clk(pixel_clk), .rst_n(pixel_rst_n), .enable(test_tone_enable),
        .sample_valid(test_valid), .sample_ready(test_ready),
        .sample_left(test_left), .sample_right(test_right),
        .rom_index(), .sample_accepted()
    );

    audio_source_mux u_test_mux (
        .clk(pixel_clk), .rst_n(pixel_rst_n),
        .wav_valid(test_valid), .wav_ready(test_ready),
        .wav_left(test_left), .wav_right(test_right),
        .wav_active(test_tone_enable),
        .arp_valid(mix_valid), .arp_ready(mix_ready),
        .arp_left(mix_left), .arp_right(mix_right),
        .force_arpeggio(1'b0),
        .out_valid(source_valid), .out_ready(source_ready),
        .out_left(source_left), .out_right(source_right)
    );

    assign audio_source = test_tone_enable                ? 2'd2 :
                          (wav_active && !force_arpeggio) ? 2'd1 : 2'd0;

    m06_pcm_contract_adapter u_m06_pcm_contract (
        .clk(pixel_clk), .rst_n(pixel_rst_n),
        .source_valid(source_valid), .source_ready(source_ready),
        .source_left(source_left), .source_right(source_right),
        .pcm_valid(sample_valid), .pcm_ready(sample_ready),
        .pcm_left(sample_left), .pcm_right(sample_right),
        .contract_error(pcm_contract_error)
    );

    hdmi_audio_symbol_core u_symbol_core (
        .pixel_clk(pixel_clk), .rst_n(pixel_rst_n),
        .video_hsync(video_hsync_q), .video_vsync(video_vsync_q),
        .video_de(video_de_q), .video_x(video_x_q), .video_rgb(video_rgb_q),
        .sample_valid(sample_valid), .sample_ready(sample_ready),
        .sample_left(sample_left), .sample_right(sample_right),
        .channel0_symbol(channel0_symbol),
        .channel1_symbol(channel1_symbol),
        .channel2_symbol(channel2_symbol),
        .sequence_error(sequence_error)
    );

    assign pcm_tap_valid = sample_valid && sample_ready;
    assign pcm_tap_left = sample_left;
endmodule
