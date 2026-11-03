`timescale 1ns/1ps

module hdmi_audio_symbol_core #(
    parameter integer ACR_INTERVAL_CYCLES = 25000,
    parameter [9:0]   V_ACTIVE = 10'd480,
    // x wraps (799->0) counted from the vsync falling edge until the line
    // whose x=790..799 precedes the first active line: VSYNC(2)+VBP(33)=35
    // for 640x480 with the vsync edge in horizontal blanking.
    parameter [9:0]   FIRST_ACTIVE_WRAP = 10'd35
) (
    input  wire               pixel_clk,
    input  wire               rst_n,
    input  wire               video_hsync,
    input  wire               video_vsync,
    input  wire               video_de,
    input  wire [9:0]         video_x,
    input  wire [23:0]        video_rgb,
    input  wire               sample_valid,
    output wire               sample_ready,
    input  wire signed [15:0] sample_left,
    input  wire signed [15:0] sample_right,
    output wire [9:0]         channel0_symbol,
    output wire [9:0]         channel1_symbol,
    output wire [9:0]         channel2_symbol,
    output wire               sequence_error
);
    reg previous_vsync;
    reg previous_de;
    reg line_has_de;
    reg [9:0] active_line_count;
    reg [9:0] lines_since_vsync;

    // HDMI 1.4 5.2.2: a Video Preamble + Guard Band may only precede a Video
    // Data Period. Emitting them on the 45 blanking lines (original course
    // code) leaves a guard band with no video after it. Predict whether the
    // next line carries video:
    //   * current line had DE and fewer than V_ACTIVE lines were sent, or
    //   * this is the line right before the first active line, i.e. the
    //     FIRST_ACTIVE_WRAP-th x wrap after the vsync falling edge.
    wire vsync_start = previous_vsync && !video_vsync;
    wire next_line_active =
        (line_has_de && active_line_count < V_ACTIVE) ||
        (lines_since_vsync == FIRST_ACTIVE_WRAP);

    always @(posedge pixel_clk or negedge rst_n) begin
        if (!rst_n) begin
            previous_vsync <= 1'b1;
            previous_de <= 1'b0;
            line_has_de <= 1'b0;
            active_line_count <= 10'd0;
            lines_since_vsync <= 10'h3ff;
        end else begin
            previous_vsync <= video_vsync;
            previous_de <= video_de;

            if (video_de && !previous_de)
                active_line_count <= active_line_count + 1'b1;
            else if (vsync_start)
                active_line_count <= 10'd0;

            if (video_de)
                line_has_de <= 1'b1;
            else if (video_x == 10'd799)
                line_has_de <= 1'b0;

            if (vsync_start)
                lines_since_vsync <= 10'd0;
            else if (video_x == 10'd799 && lines_since_vsync != 10'h3ff)
                lines_since_vsync <= lines_since_vsync + 1'b1;
        end
    end

    wire video_preamble = next_line_active &&
                          video_x >= 10'd790 && video_x <= 10'd797;
    wire video_guard = next_line_active && video_x >= 10'd798;
    wire island_slot = video_x == 10'd652;
    wire info_emit = previous_vsync && !video_vsync;

    wire audio_valid, audio_ready;
    wire [23:0] audio_header;
    wire [7:0] audio_header_ecc;
    wire [223:0] audio_body;
    wire [31:0] audio_body_ecc;
    wire [7:0] packet_frame_base_unused;
    hdmi_audio_sample_packetizer u_audio_packetizer (
        .clk(pixel_clk), .rst_n(rst_n),
        .sample_valid(sample_valid), .sample_ready(sample_ready),
        .sample_left(sample_left), .sample_right(sample_right),
        .packet_valid(audio_valid), .packet_ready(audio_ready),
        .packet_header(audio_header),
        .packet_header_ecc(audio_header_ecc),
        .packet_body(audio_body), .packet_body_ecc(audio_body_ecc),
        .packet_frame_base(packet_frame_base_unused)
    );

    wire acr_valid, acr_ready;
    wire [23:0] acr_header;
    wire [7:0] acr_header_ecc;
    wire [223:0] acr_body;
    wire [31:0] acr_body_ecc;
    hdmi_audio_acr_packetizer #(
        .INTERVAL_CYCLES(ACR_INTERVAL_CYCLES),
        .ACR_N(20'd6144),
        .ACR_CTS(20'd25000)
    ) u_acr_packetizer (
        .clk(pixel_clk), .rst_n(rst_n),
        .packet_valid(acr_valid), .packet_ready(acr_ready),
        .packet_header(acr_header), .packet_header_ecc(acr_header_ecc),
        .packet_body(acr_body), .packet_body_ecc(acr_body_ecc)
    );

    wire info_valid, info_ready;
    wire [23:0] info_header;
    wire [7:0] info_header_ecc;
    wire [223:0] info_body;
    wire [31:0] info_body_ecc;
    hdmi_audio_infoframe_packetizer u_info_packetizer (
        .clk(pixel_clk), .rst_n(rst_n), .emit(info_emit),
        .packet_valid(info_valid), .packet_ready(info_ready),
        .packet_header(info_header),
        .packet_header_ecc(info_header_ecc),
        .packet_body(info_body), .packet_body_ecc(info_body_ecc)
    );

    // L8：包调度器只给出 valid/ready/来源号（其整包选择输出不接，综合裁掉）；
    //   数据岛调度器锁存来源号，mapper 直接读三个包生成器的内容（锁定期间它们保持不变）
    wire packet_valid;
    wire packet_claim;
    wire packet_ready;
    wire [1:0] packet_source;
    hdmi_audio_packet_scheduler u_packet_scheduler (
        .clk(pixel_clk), .rst_n(rst_n),
        .acr_valid(acr_valid), .acr_ready(acr_ready),
        .acr_header(acr_header), .acr_header_ecc(acr_header_ecc),
        .acr_body(acr_body), .acr_body_ecc(acr_body_ecc),
        .info_valid(info_valid), .info_ready(info_ready),
        .info_header(info_header), .info_header_ecc(info_header_ecc),
        .info_body(info_body), .info_body_ecc(info_body_ecc),
        .audio_valid(audio_valid), .audio_ready(audio_ready),
        .audio_header(audio_header), .audio_header_ecc(audio_header_ecc),
        .audio_body(audio_body), .audio_body_ecc(audio_body_ecc),
        .packet_valid(packet_valid), .packet_lock(packet_claim),
        .packet_ready(packet_ready),
        .packet_source(packet_source),
        .packet_header(),
        .packet_header_ecc(),
        .packet_body(), .packet_body_ecc()
    );

    wire [2:0] mode;
    wire [1:0] control0, control1, control2;
    wire [4:0] packet_pixel_index;
    wire [1:0] active_source;
    wire sequence_active_unused;
    hdmi_data_island_scheduler u_island_scheduler (
        .clk(pixel_clk), .rst_n(rst_n),
        .video_active(video_de), .video_preamble(video_preamble),
        .video_guard(video_guard), .island_slot(island_slot),
        .hsync(video_hsync), .vsync(video_vsync),
        .packet_valid(packet_valid), .packet_ready(packet_ready),
        .packet_claim(packet_claim),
        .packet_source(packet_source),
        .mode(mode), .channel0_control(control0),
        .channel1_control(control1), .channel2_control(control2),
        .packet_pixel_index(packet_pixel_index),
        .active_source(active_source),
        .sequence_active(sequence_active_unused),
        .sequence_error(sequence_error)
    );

    wire [3:0] island0, island1, island2;
    hdmi_data_island_mapper u_island_mapper (
        .pixel_index(packet_pixel_index),
        .hsync(video_hsync), .vsync(video_vsync),
        .active_source(active_source),
        .acr_header(acr_header), .acr_header_ecc(acr_header_ecc),
        .acr_body(acr_body), .acr_body_ecc(acr_body_ecc),
        .info_header(info_header), .info_header_ecc(info_header_ecc),
        .info_body(info_body), .info_body_ecc(info_body_ecc),
        .audio_header(audio_header), .audio_header_ecc(audio_header_ecc),
        .audio_body(audio_body), .audio_body_ecc(audio_body_ecc),
        .channel0_terc4(island0), .channel1_terc4(island1),
        .channel2_terc4(island2)
    );

    hdmi_tmds_channel_encoder #(.CHANNEL(0)) u_encoder0 (
        .clk(pixel_clk), .rst_n(rst_n), .mode(mode),
        .video_data(video_rgb[7:0]), .data_island_data(island0),
        .control_data(control0), .symbol(channel0_symbol)
    );
    hdmi_tmds_channel_encoder #(.CHANNEL(1)) u_encoder1 (
        .clk(pixel_clk), .rst_n(rst_n), .mode(mode),
        .video_data(video_rgb[15:8]), .data_island_data(island1),
        .control_data(control1), .symbol(channel1_symbol)
    );
    hdmi_tmds_channel_encoder #(.CHANNEL(2)) u_encoder2 (
        .clk(pixel_clk), .rst_n(rst_n), .mode(mode),
        .video_data(video_rgb[23:16]), .data_island_data(island2),
        .control_data(control2), .symbol(channel2_symbol)
    );
endmodule
