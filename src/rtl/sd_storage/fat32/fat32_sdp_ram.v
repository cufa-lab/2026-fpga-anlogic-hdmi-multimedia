`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// fat32_sdp_ram —— 简单双口 RAM（一写一读，同步读），按 TD SWUG106 5.6.5.2 写法推断为 BRAM9K
//   存储子系统里所有“大表”都放这里，不再用寄存器数组：
//     * sd_sector_adapter 的 512x8 共享扇区缓冲
//     * sd_card_bmp 的 128x136 文件目录表（低 68 项 = BMP 清单，96..127 = 扫描器的子目录队列）
//   读端：re=1 时下一拍 rdata = mem[raddr]；re=0 时 rdata 保持（扇区写回逐字节取数要用这个保持）
//   不带复位（BRAM 没有复位，仿真里未写过的单元是 X，使用者只读写过的地址）
// -----------------------------------------------------------------------------
module fat32_sdp_ram #(
    parameter integer AW = 9,
    parameter integer DW = 8
) (
    input  wire          clk,
    input  wire          we,
    input  wire [AW-1:0] waddr,
    input  wire [DW-1:0] wdata,
    input  wire          re,
    input  wire [AW-1:0] raddr,
    output reg  [DW-1:0] rdata
);
    (* ram_style = "bram" *) reg [DW-1:0] mem [0:(1<<AW)-1];

    always @(posedge clk)
        if (we)
            mem[waddr] <= wdata;

    always @(posedge clk)
        if (re)
            rdata <= mem[raddr];
endmodule
