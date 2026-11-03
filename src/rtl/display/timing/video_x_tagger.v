`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// video_x_tagger —— 给视频流打上列号 video_x
// hdmi_audio_symbol_core 按 HTOTAL=800 硬编码了列位置：
//   x=0..639 有效像素, x==652 数据岛起点, 790..797 视频前导, 798..799 视频保护带
// 因此要求：输出 de 第一个有效像素那一拍 video_x_o == 0，之后逐拍 +1，799 回 0。
// 本模块把 hs/vs/de/rgb 与 video_x 一起打一拍输出，保证同一拍对齐(流水线 valid 对齐)。
// 上电锁定：复位期间 color_bar 把 hs/vs 拉低，复位释放瞬间会出现一个"伪 vs 下降沿"，
// 且此时 x 与行不同步。若直接透传，symbol core 会从该伪沿开始数行，
// 第一帧在无视频的行前发出视频前导/保护带，数据岛也可能与第一条有效行重叠。
// 因此：在第一个真实 hs 下降沿(之前观察到过 hs 为高)锁定 x = HS_FALL_X，
// vs 同理：复位后 vs 一直为低直到 color_bar 第一次"场同步结束"(约 12 行)，此低电平不是场同步，
// 故 vs 在观察到过高电平之后才透传，否则输出空闲高电平。
// 帧锁定：color_bar 复位后第一帧有效行出现在任何 vs 下降沿之前，symbol core 无从预测第一行
// (该行缺视频前导/保护带)。故 de/rgb 在第一个真实 vs 下降沿之后才透传(上电最多多黑一帧)。
// 锁定前输出保持空闲(hs=vs=1, de=0, rgb=0，纯控制期)；锁定后 de 上升沿仍把 x 对齐到 0。
// -----------------------------------------------------------------------------
module video_x_tagger #(
    parameter [9:0] H_TOTAL   = 10'd800,
    parameter [9:0] HS_FALL_X = 10'd656      // H_ACTIVE + H_FP
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        hs_i,
    input  wire        vs_i,
    input  wire        de_i,
    input  wire [23:0] rgb_i,
    output reg         hs_o,
    output reg         vs_o,
    output reg         de_o,
    output reg  [23:0] rgb_o,
    output reg  [9:0]  video_x_o,
    output reg         locked_o
);
    reg hs_seen_high;
    reg hs_prev;
    reg vs_seen_high;
    reg vs_prev;
    reg frame_locked;
    wire vs_fall = locked_o && vs_seen_high && vs_prev && !vs_i;
    wire frame_now = frame_locked || vs_fall;
    wire hs_fall = hs_seen_high && hs_prev && !hs_i;
    wire lock_now = locked_o || hs_fall;

    // 纯数据寄存器不接复位（按锁定门控，复位期间 frame_now=0、时钟照常冲成 0）
    always @(posedge clk)
        rgb_o <= frame_now ? rgb_i : 24'd0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hs_o         <= 1'b1;   // 640x480 同步低有效，空闲为高
            vs_o         <= 1'b1;
            de_o         <= 1'b0;
            video_x_o    <= 10'd0;
            locked_o     <= 1'b0;
            hs_seen_high <= 1'b0;
            hs_prev      <= 1'b0;
            vs_seen_high <= 1'b0;
            vs_prev      <= 1'b1;
            frame_locked <= 1'b0;
        end else begin
            hs_prev <= hs_i;
            if (hs_i)
                hs_seen_high <= 1'b1;
            vs_prev <= vs_i;
            if (vs_i)
                vs_seen_high <= 1'b1;
            if (vs_fall)
                frame_locked <= 1'b1;
            if (hs_fall)
                locked_o <= 1'b1;

            hs_o  <= lock_now ? hs_i  : 1'b1;
            vs_o  <= (lock_now && vs_seen_high) ? vs_i : 1'b1;
            de_o  <= frame_now ? de_i  : 1'b0;

            if (hs_fall && !locked_o)
                video_x_o <= HS_FALL_X;
            else if (de_i && !de_o && frame_now)
                video_x_o <= 10'd0;
            else if (video_x_o == H_TOTAL - 1'b1)
                video_x_o <= 10'd0;
            else
                video_x_o <= video_x_o + 1'b1;
        end
    end
endmodule
