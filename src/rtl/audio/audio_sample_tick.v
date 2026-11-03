`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// audio_sample_tick —— 48kHz 采样节拍
//   每拍 +SAMPLE_HZ，累加到 >= CLOCK_HZ - SAMPLE_HZ 时出一个单拍 tick 并减 CLOCK_HZ
//   → 25MHz / 48kHz = 520.83 拍，tick 间隔 520/521 交替，长期平均严格 48kHz（与 HDMI ACR N=6144/CTS=25000 一致）
// -----------------------------------------------------------------------------
module audio_sample_tick #(
    parameter integer CLOCK_HZ  = 25000000,
    parameter integer SAMPLE_HZ = 48000
) (
    input  wire clk,
    input  wire rst_n,
    output wire tick
);
    localparam integer AW = $clog2(CLOCK_HZ + 1);
    reg [AW-1:0] acc;

    assign tick = (acc >= CLOCK_HZ - SAMPLE_HZ);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            acc <= {AW{1'b0}};
        else if (tick)
            acc <= acc + SAMPLE_HZ - CLOCK_HZ;
        else
            acc <= acc + SAMPLE_HZ;
    end
endmodule
