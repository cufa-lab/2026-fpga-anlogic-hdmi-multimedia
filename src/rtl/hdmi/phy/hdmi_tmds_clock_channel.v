`timescale 1ns/1ps

// Repeat one 5-high/5-low symbol per pixel for a 25 MHz TMDS clock.
module hdmi_tmds_clock_channel (
    input  wire       pixel_clk,
    input  wire       rst_n,
    output reg [9:0]  symbol
);

    always @(posedge pixel_clk or negedge rst_n) begin
        if (!rst_n) begin
            symbol <= 10'b0000011111;
        end else begin
            symbol <= 10'b0000011111;
        end
    end

endmodule
