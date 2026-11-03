`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// frame_boundary_pulse —— pix_clk 域帧边界（输入为送往 HDMI 的那组 vs/de）
//
//   frame_start : 低有效 VSYNC 下降沿（进入场同步）——OSD 每帧锁存状态用
//   blank_start : 第 V_ACTIVE 行有效数据结束（de 下降沿）——显示缓冲区提交点
//
// 为什么提交点不用 VSYNC 边沿：video_timing_data 在“未延时”的 VSYNC 下降沿发
// read_req，frame_fifo_read 在那一刻锁存 read_addr_index；送 HDMI 的 vs 比它晚
// video_delay 的 21 拍，在这个边沿提交已经赶不上本帧的 read_req（新图要晚一帧，
// 且 ack 回去后下一次加载可能覆盖“仍在被读”的缓冲）。
// 场消隐开始点：本帧像素已全部读出，离下一次 read_req 还有 VFP（10 行 ≈ 320us），
// 足够 read_addr_index 在 mem_clk 域两级同步后稳定。
// -----------------------------------------------------------------------------
module frame_boundary_pulse #(
    parameter integer V_ACTIVE = 480
) (
    input  wire pix_clk,
    input  wire pix_rst_n,
    input  wire vsync_n,
    input  wire de,
    output reg  frame_start,
    output reg  blank_start
);
    reg        vsync_n_d;
    reg        de_d;
    reg        armed;          // 复位后先见到一次 VSYNC，blank_start 才输出
    reg [10:0] line_cnt;

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            vsync_n_d   <= 1'b1;
            de_d        <= 1'b0;
            armed       <= 1'b0;
            line_cnt    <= 11'd0;
            frame_start <= 1'b0;
            blank_start <= 1'b0;
        end else begin
            vsync_n_d   <= vsync_n;
            de_d        <= de;
            frame_start <= 1'b0;
            blank_start <= 1'b0;

            if (!vsync_n) begin
                line_cnt <= 11'd0;
                if (vsync_n_d) begin
                    // VSYNC 边沿本身总是可信的；armed 只挡 blank_start（行计数要从 VSYNC 开始才准）
                    frame_start <= 1'b1;
                    armed       <= 1'b1;
                end
            end else if (de_d && !de) begin
                line_cnt <= line_cnt + 11'd1;
                if (armed && (line_cnt == V_ACTIVE - 1))
                    blank_start <= 1'b1;
            end
        end
    end
endmodule
