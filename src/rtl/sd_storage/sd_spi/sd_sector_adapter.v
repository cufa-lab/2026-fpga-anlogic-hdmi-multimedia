`timescale 1ns/1ps

module sd_sector_adapter #(
    parameter integer TIMEOUT_CYCLES = 0,         // 0 = 不做超时；TICK_EN=1 时单位是 tick
    parameter integer TICK_EN        = 0          // 逻辑优化 L2：1 = 超时计数只在 tick 那拍加 1（sd_card_bmp 接 1ms 节拍，500ms → 9 位）
)(
    input  wire        clk,
    input  wire        rst_n,                     // 低有效复位
    input  wire        tick,                      // TICK_EN=1 时的计时节拍；=0 不用（tb 可不接）

    input  wire        sector_req,                // 读扇区请求（FAT32 模块给单拍脉冲，电平也可）
    input  wire [31:0] sector_lba,                // 扇区号
    output wire        sector_busy,               // 忙（正在读）
    output wire        sector_valid,              // 字节有效脉冲
    output wire [7:0]  sector_byte,               // 当前字节
    output wire [8:0]  sector_byte_index,         // 字节索引（0~511）
    output wire        sector_done,               // 读完成脉冲
    output wire [7:0]  sector_error,              // 错误码（0 = 正常，与 sector_done 同拍有效）

    output reg         sd_sec_read,               // 读扇区请求（电平）
    output reg  [31:0] sd_sec_read_addr,          // 扇区号
    input  wire        sd_sec_read_data_valid,    // 字节有效
    input  wire [7:0]  sd_sec_read_data,          // 当前字节
    input  wire        sd_sec_read_end            // 读完成
);
    localparam [7:0]  ERR_NONE    = 8'h00;
    localparam [7:0]  ERR_TIMEOUT = 8'h01;
    // TICK_EN=1：数满 TIMEOUT_CYCLES 个 tick 后再来一个 tick 才超时 → 实际 (N, N+1] ms，不会比按周期计短
    localparam integer TLIM = TICK_EN ? TIMEOUT_CYCLES + 2 : TIMEOUT_CYCLES;   // 命中条件 cnt >= TLIM-1 = N+1
    localparam integer TW = (TLIM < 2) ? 1 : $clog2(TLIM + 1);

    reg [8:0]    byte_cnt;       // 字节索引计数器（0~511）
    reg          busy_reg;       // 忙寄存器
    reg [TW-1:0] timeout_cnt;
    reg          timeout_done;   // 超时收尾：单拍 done + error

    wire timeout_hit = (TIMEOUT_CYCLES != 0) && busy_reg &&
                       (timeout_cnt >= TLIM - 1) && !sd_sec_read_end;

    assign sector_busy       = busy_reg;
    assign sector_valid      = sd_sec_read_data_valid && busy_reg;
    assign sector_byte       = sd_sec_read_data;
    assign sector_byte_index = byte_cnt;
    assign sector_done       = (sd_sec_read_end && busy_reg) || timeout_done;
    assign sector_error      = timeout_done ? ERR_TIMEOUT : ERR_NONE;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sd_sec_read      <= 1'b0;
            sd_sec_read_addr <= 32'd0;
            byte_cnt         <= 9'd0;
            busy_reg         <= 1'b0;
            timeout_cnt      <= {TW{1'b0}};
            timeout_done     <= 1'b0;
        end else begin
            timeout_done <= 1'b0;
            // 请求到来且不忙时，锁存扇区号，发出读请求
            if (sector_req && !busy_reg && !timeout_done) begin
                sd_sec_read      <= 1'b1;
                sd_sec_read_addr <= sector_lba;
                busy_reg         <= 1'b1;
                byte_cnt         <= 9'd0;
                timeout_cnt      <= {TW{1'b0}};
            end
            // 读完成：撤销请求，清零计数器和忙
            else if (sd_sec_read_end && busy_reg) begin
                sd_sec_read <= 1'b0;
                busy_reg    <= 1'b0;
                byte_cnt    <= 9'd0;
            end
            // 读超时：撤销请求，下一拍给出 done + error
            else if (timeout_hit) begin
                sd_sec_read  <= 1'b0;
                busy_reg     <= 1'b0;
                byte_cnt     <= 9'd0;
                timeout_done <= 1'b1;
            end
            // 读进行中：字节有效时计数器递增
            else if (busy_reg) begin
                if (!TICK_EN || tick)
                    timeout_cnt <= timeout_cnt + 1'b1;
                if (sd_sec_read_data_valid)
                    byte_cnt <= byte_cnt + 9'd1;
            end
        end
    end
endmodule
