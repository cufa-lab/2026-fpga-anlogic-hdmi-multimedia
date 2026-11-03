`timescale 1ns/1ps

module script_dc_ram #(
    parameter integer AW = 9,
    parameter integer DW = 18
) (
    input  wire          wclk,
    input  wire          we,
    input  wire [AW-1:0] waddr,
    input  wire [DW-1:0] wdata,
    input  wire          rclk,
    input  wire          re,
    input  wire [AW-1:0] raddr,
    output reg  [DW-1:0] rdata
);
    (* ram_style = "bram" *) reg [DW-1:0] mem [0:(1<<AW)-1];

    always @(posedge wclk)
        if (we)
            mem[waddr] <= wdata;

    always @(posedge rclk)
        if (re)
            rdata <= mem[raddr];
endmodule
