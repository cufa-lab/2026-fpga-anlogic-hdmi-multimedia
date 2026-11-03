`timescale 1ns/1ps

// 三路包（ACR / InfoFrame / 音频）各自按 pixel_index 取位，再按
//   active_source 三选一。ACR、InfoFrame 是常量包，取位逻辑被综合成 5 输入小表；
//   原来"整包三选一 + 整包拷贝"的 288 位选择器不再需要。
module hdmi_data_island_mapper (
    input  wire [4:0]   pixel_index,
    input  wire         hsync,
    input  wire         vsync,
    input  wire [1:0]   active_source,     // 0 = ACR, 1 = InfoFrame, 2 = 音频
    input  wire [23:0]  acr_header,
    input  wire [7:0]   acr_header_ecc,
    input  wire [223:0] acr_body,
    input  wire [31:0]  acr_body_ecc,
    input  wire [23:0]  info_header,
    input  wire [7:0]   info_header_ecc,
    input  wire [223:0] info_body,
    input  wire [31:0]  info_body_ecc,
    input  wire [23:0]  audio_header,
    input  wire [7:0]   audio_header_ecc,
    input  wire [223:0] audio_body,
    input  wire [31:0]  audio_body_ecc,
    output wire [3:0]   channel0_terc4,
    output wire [3:0]   channel1_terc4,
    output wire [3:0]   channel2_terc4
);
    wire [5:0] even_bit = {pixel_index, 1'b0};
    wire [5:0] odd_bit = {pixel_index, 1'b1};

    // 一个包在本像素上的 9 个数据位：{D2(header), ch2 的 4 位(奇), ch1 的 4 位(偶)}
    function [8:0] lane_bits;
        input [23:0]  header;
        input [7:0]   header_ecc;
        input [223:0] body;
        input [31:0]  body_ecc;
        input [4:0]   index;
        reg [31:0] hb;
        reg [63:0] b0, b1, b2, b3;
        reg [5:0] e, o;
        begin
            hb = {header_ecc, header};
            b0 = {body_ecc[7:0], body[55:0]};
            b1 = {body_ecc[15:8], body[111:56]};
            b2 = {body_ecc[23:16], body[167:112]};
            b3 = {body_ecc[31:24], body[223:168]};
            e = {index, 1'b0};
            o = {index, 1'b1};
            lane_bits = {hb[index], b3[o], b2[o], b1[o], b0[o],
                         b3[e], b2[e], b1[e], b0[e]};
        end
    endfunction

    wire [8:0] acr_bits   = lane_bits(acr_header, acr_header_ecc, acr_body, acr_body_ecc, pixel_index);
    wire [8:0] info_bits  = lane_bits(info_header, info_header_ecc, info_body, info_body_ecc, pixel_index);
    wire [8:0] audio_bits = lane_bits(audio_header, audio_header_ecc, audio_body, audio_body_ecc, pixel_index);
    wire [8:0] bits = (active_source == 2'd0) ? acr_bits :
                      (active_source == 2'd1) ? info_bits : audio_bits;

    // D3 is the allowed packet-sync don't-care value, held at one for a
    // deterministic stream. D2 carries the header BCH block; D1/D0 carry
    // VSYNC/HSYNC throughout the Data Island and its guard bands.
    assign channel0_terc4 = {1'b1, bits[8], vsync, hsync};
    assign channel2_terc4 = bits[7:4];
    assign channel1_terc4 = bits[3:0];
endmodule
