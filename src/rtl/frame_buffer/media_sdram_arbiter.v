`timescale 1ns/1ps

module media_sdram_arbiter #(
    parameter integer IDLE_CYCLES     = 8,
    parameter integer REF_INTERVAL    = 576,   // 40MHz 下 14.4us（规格 15.625us，留约 8% 余量）
    parameter integer REF_ACK_TIMEOUT = 31
) (
    input  wire clk,
    input  wire rst_n,
    input  wire ctrl_busy,     // SDRAM 控制器 Sdr_busy（burst 收尾 / 刷新期间为 1）
    input  wire ctrl_init_done,// SDRAM 控制器 Sdr_init_done
    input  wire ref_vld,       // SDRAM 控制器 Sdr_init_ref_vld（刷新被受理后约 20 拍为 1）
    output reg  ref_req,       // → App_ref_req，单拍
    input  wire wr_want,       // loader 想开新 burst
    input  wire wr_busy,       // loader burst 进行中（含最后一拍 App_wr_en）
    output wire wr_grant,
    input  wire rd_want,       // reader 想开新 burst
    input  wire rd_busy,       // reader burst 进行中（含最后一拍 App_rd_en）
    output wire rd_grant
);
    localparam integer QW = $clog2(IDLE_CYCLES + 1);
    localparam integer RW = $clog2(REF_INTERVAL);
    localparam integer TW = $clog2(REF_ACK_TIMEOUT + 1);

    reg [QW-1:0] quiet;        // 读写口与控制器都空闲的连续拍数（饱和）
    reg [RW-1:0] ref_cnt;      // 刷新节拍，自由计数，与请求被受理的早晚无关
    reg          ref_pend;     // 本时隙的刷新还没完成
    reg          ref_wait;     // 已发请求，等应答
    reg          ref_acked;    // 已看到 ref_vld=1
    reg [TW-1:0] ref_to;

    wire settled  = !wr_busy && !rd_busy && !ctrl_busy && (quiet == IDLE_CYCLES);
    wire ref_fire = ref_pend && !ref_wait && settled;

    assign rd_grant = settled && !ref_pend;
    assign wr_grant = settled && !ref_pend && !rd_want;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            quiet <= {QW{1'b0}};
        else if (wr_busy || rd_busy || ctrl_busy || ref_wait || ref_fire ||
                 (rd_want && rd_grant) || (wr_want && wr_grant))
            quiet <= {QW{1'b0}};
        else if (quiet != IDLE_CYCLES)
            quiet <= quiet + 1'b1;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ref_cnt <= {RW{1'b0}}; ref_pend <= 1'b0; ref_wait <= 1'b0;
            ref_acked <= 1'b0; ref_to <= {TW{1'b0}}; ref_req <= 1'b0;
        end else if (!ctrl_init_done) begin
            ref_cnt <= {RW{1'b0}}; ref_pend <= 1'b0; ref_wait <= 1'b0;
            ref_acked <= 1'b0; ref_req <= 1'b0;
        end else begin
            ref_req <= ref_fire;
            ref_cnt <= (ref_cnt == REF_INTERVAL - 1) ? {RW{1'b0}} : ref_cnt + 1'b1;
            if (ref_fire) begin
                ref_wait <= 1'b1; ref_acked <= 1'b0; ref_to <= {TW{1'b0}};
            end else if (ref_wait) begin
                ref_to <= ref_to + 1'b1;
                if (ref_vld)
                    ref_acked <= 1'b1;
                else if (ref_acked) begin               // 应答结束：本次刷新完成
                    ref_wait <= 1'b0; ref_pend <= 1'b0;
                end else if (ref_to == REF_ACK_TIMEOUT)
                    ref_wait <= 1'b0;                   // 没被受理，下次安静时重发
            end
            if (ref_cnt == REF_INTERVAL - 1)
                ref_pend <= 1'b1;
        end
    end
endmodule
