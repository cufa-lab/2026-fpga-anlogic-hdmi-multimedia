`timescale 1ns/1ps

module brightness_contrast_adjust #(
    parameter integer BRIGHT_STANDALONE = 0,
    parameter integer PIPELINE          = 0
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        frame_start,     // 帧消隐期单拍（与转场控制器同一个边界）
    input  wire [7:0]  brightness_in,   // 0..255，255 = 正常
    input  wire [7:0]  contrast_in,     // 0..255，128 = 正常
    input  wire [7:0]  alpha_in,        // 转场 alpha（255 = 不衰减）
    input  wire        in_de,
    input  wire [23:0] rgb_in,
    output wire        out_de,
    output wire [23:0] rgb_out,
    output wire [7:0]  alpha_out,       // 亮度折进去之后的 alpha（BRIGHT_STANDALONE=1 时 = alpha_in）
    output reg  [7:0]  brightness_q,    // 本帧生效值（给 OSD 显示 / 观测）
    output reg  [7:0]  contrast_q
);
    // ---------------- 帧边界锁存 ----------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            brightness_q <= 8'd255;
            contrast_q   <= 8'd128;
        end else if (frame_start) begin
            brightness_q <= brightness_in;
            contrast_q   <= contrast_in;
        end
    end

    wire [8:0] bright_gain = {1'b0, brightness_q} + 9'd1;     // 1..256

    // ---------------- 对比度 + 饱和截断（每通道） ----------------
    function [7:0] contrast_channel;
        input [7:0] x;
        input [7:0] c;
        reg signed [8:0]  center;    // x - 128：[-128, 127]
        reg signed [17:0] product;   // center × c：[-32640, 32385]
        reg signed [17:0] sum;       // (product >>> 7) + 128：[-127, 381]
        begin
            center  = $signed({1'b0, x}) - 9'sd128;
            product = center * $signed({1'b0, c});
            sum     = (product >>> 7) + 18'sd128;
            if (sum < 18'sd0)
                contrast_channel = 8'd0;
            else if (sum > 18'sd255)
                contrast_channel = 8'd255;
            else
                contrast_channel = sum[7:0];
        end
    endfunction

    wire [23:0] pre_rgb;
    generate
        if (BRIGHT_STANDALONE != 0) begin : g_bright_pix
            wire [16:0] mul_r = rgb_in[23:16] * bright_gain;
            wire [16:0] mul_g = rgb_in[15:8]  * bright_gain;
            wire [16:0] mul_b = rgb_in[7:0]   * bright_gain;
            assign pre_rgb   = {mul_r[15:8], mul_g[15:8], mul_b[15:8]};
            assign alpha_out = alpha_in;
        end else begin : g_bright_alpha
            //(brightness+1)/256 折进转场 alpha，像素上不再乘亮度
            wire [16:0] eff_alpha = alpha_in * bright_gain;      // ≤ 255 × 256 = 65280
            assign pre_rgb   = rgb_in;
            assign alpha_out = eff_alpha[15:8];
        end
    endgenerate

    wire [23:0] adj_rgb = {
        contrast_channel(pre_rgb[23:16], contrast_q),
        contrast_channel(pre_rgb[15:8],  contrast_q),
        contrast_channel(pre_rgb[7:0],   contrast_q)
    };

    // ---------------- 输出：组合直通或 rgb/de 同拍寄存一级 ----------------
    generate
        if (PIPELINE != 0) begin : g_pipe
            reg        de_q;
            reg [23:0] rgb_q;
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n)
                    de_q <= 1'b0;
                else
                    de_q <= in_de;
            end
            always @(posedge clk)
                rgb_q <= adj_rgb;               // 纯数据寄存器不复位
            assign out_de  = de_q;
            assign rgb_out = rgb_q;
        end else begin : g_comb
            assign out_de  = in_de;
            assign rgb_out = adj_rgb;
        end
    endgenerate
endmodule
