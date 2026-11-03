`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// cdc_bus_handshake —— 多比特慢变状态的跨时钟域（toggle 请求/应答）
//
// 多位总线不能逐位 2FF（各位到达拍数不同 → 撕裂）。本模块：
//   源域：数据变化且上一次已被应答 → 把数据锁进 src_hold，翻转 req
//   目的域：req 经 2FF 同步，看到翻转时 src_hold 已稳定 ≥2 个目的时钟，直接采样，
//   然后翻转 ack
//   源域：ack 经 2FF 同步回来前 src_hold 不变
// 连续变化会被合并成“最新值”，适合状态/配置类信号，不适合事件流。
// -----------------------------------------------------------------------------
module cdc_bus_handshake #(
    parameter integer   WIDTH     = 8,
    parameter [WIDTH-1:0] RESET_VAL = {WIDTH{1'b0}}
) (
    input  wire             src_clk,
    input  wire             src_rst_n,
    input  wire [WIDTH-1:0] src_data,

    input  wire             dst_clk,
    input  wire             dst_rst_n,
    output reg  [WIDTH-1:0] dst_data
);
    reg [WIDTH-1:0] src_hold;
    reg             src_req;
    (* async_reg = "true" *) reg ack_sync1;
    (* async_reg = "true" *) reg ack_sync2;

    (* async_reg = "true" *) reg req_sync1;
    (* async_reg = "true" *) reg req_sync2;
    reg             dst_ack;

    wire src_idle = (src_req == ack_sync2);

    always @(posedge src_clk or negedge src_rst_n) begin
        if (!src_rst_n) begin
            src_hold  <= RESET_VAL;
            src_req   <= 1'b0;
            ack_sync1 <= 1'b0;
            ack_sync2 <= 1'b0;
        end else begin
            ack_sync1 <= dst_ack;
            ack_sync2 <= ack_sync1;
            if (src_idle && (src_data != src_hold)) begin
                src_hold <= src_data;
                src_req  <= ~src_req;
            end
        end
    end

    always @(posedge dst_clk or negedge dst_rst_n) begin
        if (!dst_rst_n) begin
            req_sync1 <= 1'b0;
            req_sync2 <= 1'b0;
            dst_ack   <= 1'b0;
            dst_data  <= RESET_VAL;
        end else begin
            req_sync1 <= src_req;
            req_sync2 <= req_sync1;
            if (req_sync2 != dst_ack) begin
                dst_data <= src_hold;
                dst_ack  <= req_sync2;
            end
        end
    end
endmodule
