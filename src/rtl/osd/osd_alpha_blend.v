`timescale 1ns/1ps

module osd_alpha_blend (
    input  wire [23:0] bg,
    input  wire [23:0] fg,
    input  wire [7:0]  alpha,
    output wire [23:0] out
);
    wire signed [9:0] a = $signed({2'b00, alpha} + {9'd0, alpha[7]});

    function [7:0] blend_channel;
        input [7:0]        b;
        input [7:0]        f;
        input signed [9:0] k;
        reg signed [9:0]   diff;
        reg signed [19:0]  prod;
        reg signed [19:0]  acc;
        begin
            diff = $signed({2'b00, f}) - $signed({2'b00, b});
            prod = diff * k;
            // bg + ((prod + 128) >>> 8) == ({bg, 8'h80} + prod) >>> 8（bg<<8 低 8 位为 0，逐位相同），
            // 两级加法（+128 舍入、加 bg）合成一个 11 位有效进位链（低 7 位常量无进位）
            acc  = $signed({4'b0000, b, 8'h80}) + prod;
            blend_channel = acc[15:8];
        end
    endfunction

    assign out = {blend_channel(bg[23:16], fg[23:16], a),
                  blend_channel(bg[15:8],  fg[15:8],  a),
                  blend_channel(bg[7:0],   fg[7:0],   a)};
endmodule
