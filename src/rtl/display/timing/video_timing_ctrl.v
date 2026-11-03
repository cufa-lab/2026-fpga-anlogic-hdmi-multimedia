`timescale 1ns / 1ps

module video_timing_ctrl #(
    parameter H_ACT  = 640,  // 水平有效像素
    parameter H_FP   = 16,   // 水平前肩
    parameter H_SYNC = 96,   // 水平同步脉宽
    parameter H_BP   = 48,   // 水平后肩
    parameter V_ACT  = 480,  // 垂直有效行
    parameter V_FP   = 10,   // 垂直前肩
    parameter V_SYNC = 2,    // 垂直同步脉宽
    parameter V_BP   = 33,   // 垂直后肩
    parameter ACTIVE_LAST = 0,
    parameter VS_ALIGN_HSYNC = 0   // 1：vsync 边沿对齐 hsync 前沿
)(
    input  wire        pclk,   // 像素时钟（生产环境25 MHz，由TD PLL从50 MHz生成）
    input  wire        rst_n,  // 低有效复位
    output wire        hsync,  // 低有效
    output wire        vsync,  // 低有效
    output wire        de,     // 有效显示区
    output reg  [11:0] x,      // 行计数器；de=1时为有效像素列坐标
    output reg  [11:0] y       // 场计数器；de=1时为有效像素行坐标
);

    localparam H_TOTAL = H_ACT + H_FP + H_SYNC + H_BP;
    localparam V_TOTAL = V_ACT + V_FP + V_SYNC + V_BP;

    // 行/场计数：x走完一行，y走一步；两个都走完回到帧起点(0,0)
    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            x <= 12'd0;
            y <= 12'd0;
        end else if (x == H_TOTAL - 1) begin
            x <= 12'd0;
            y <= (y == V_TOTAL - 1) ? 12'd0 : y + 12'd1;
        end else begin
            x <= x + 12'd1;
        end
    end

    // 同拍组合译码：de/hsync/vsync与x/y永远对齐，不存在错拍问题
    assign de    = ACTIVE_LAST ?
                   ((x >= H_TOTAL - H_ACT) && (y >= V_TOTAL - V_ACT)) :
                   ((x < H_ACT) && (y < V_ACT));
    assign hsync = ACTIVE_LAST ?
                   ~((x >= H_FP) && (x < H_FP + H_SYNC)) :
                   ~((x >= H_ACT + H_FP) && (x < H_ACT + H_FP + H_SYNC));
    // 场同步区间 [VS0, VS1) 行；VS_ALIGN_HSYNC=1 时两个边沿都推迟到所在行的 hsync 前沿：
    // 起始行在 hsync 前沿之前仍不算、结束行在 hsync 前沿之前仍算（只多两个等值比较）
    localparam VS0      = ACTIVE_LAST ? V_FP : V_ACT + V_FP;
    localparam VS1      = VS0 + V_SYNC;
    localparam HS_START = ACTIVE_LAST ? H_FP : H_ACT + H_FP;
    wire vs_rows   = (y >= VS0) && (y < VS1);
    wire before_hs = (x < HS_START);
    wire vs_active = !VS_ALIGN_HSYNC ? vs_rows :
                     (vs_rows && !((y == VS0) && before_hs)) || ((y == VS1) && before_hs);
    assign vsync = ~vs_active;

endmodule
