`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// picture_adjust_ctrl 模块：KEY3/KEY4 组合键调节亮度/对比度
// 亮度、对比度用按键调节；本板只有 KEY1~KEY4 + SW1~SW4 且都已分配，
//   KEY3+KEY4 同按            → 进入 / 退出调节模式（IDLE_EXIT_MS 内没有按键自动退出）
//   模式内  KEY3 单击 = 当前项 +STEP     KEY4 单击 = 当前项 -STEP
//           KEY3 长按 = 切换 亮度 / 对比度   KEY4 长按 = 当前项恢复默认（亮度 255 / 对比度 128）
//   模式外  所有按键事件原样交给原有功能（KEY3 自动轮播/亮度阶梯、KEY4 音量/周期）
//   KEY2（切图）、KEY2+KEY3（保存）不经过本模块，任何时候都可用。
// 输入沿用已有的消抖模块（dual_key_chord_router / single_key_event）：它们给出消抖后的电平
// key*_stable_n 和"松开时判定"的单击/长按事件
// 2FF 同步 → 20ms 消抖 → 按住计时 → 松开沿统一区分单击/长按），这里不再重复消抖。
//
// 组合键过滤：两个键都按下的那一刻起到两个键都松开后再过 1 拍，KEY3/KEY4 的单击/长按事件
// 全部吞掉（router 的 KEY3 事件比电平晚 1 拍，single_key_event 的 KEY4 事件与电平同拍），
// 所以同按不会顺带触发自动轮播 / 音量 / 周期，也不会被当成模式内的加减。
// 事件输出都是单拍；数值本身由 media_policy 保存（与亮度阶梯、Flash 加载/保存在同一处）。
// -----------------------------------------------------------------------------
// 逻辑优化：空闲退出计时只在 tick（公共 1ms 节拍）那拍前进，13 位（原按 50MHz 周期计 28 位）
module picture_adjust_ctrl #(
    parameter integer IDLE_EXIT_MS = 5000
) (
    input  wire clk,
    input  wire rst_n,
    input  wire tick,               // 公共 1ms 节拍（tb 接 1 = 每拍 1 ms）
    input  wire enable,             // 首图出来之后才允许进入（同 media_policy.enable）

    input  wire key3_stable_n,      // 消抖后电平，低 = 按下
    input  wire key4_stable_n,
    input  wire key3_click_in,
    input  wire key3_long_in,
    input  wire key4_click_in,
    input  wire key4_long_in,

    // → 原有功能（模式外、且不是组合键的一部分）
    output wire key3_click_out,
    output wire key3_long_out,
    output wire key4_click_out,
    output wire key4_long_out,

    // → media_policy（单拍）
    output reg  adj_mode,           // 1 = 调节模式中
    output reg  adj_sel,            // 0 = 亮度，1 = 对比度
    output wire adj_inc,
    output wire adj_dec,
    output wire adj_default,
    output reg  adj_show            // 进入模式 / 切换调节项（数码管切到对应页）
);
    localparam integer IDLE_W = (IDLE_EXIT_MS <= 2) ? 1 : $clog2(IDLE_EXIT_MS + 1);

    wire both_down = !key3_stable_n && !key4_stable_n;
    wire both_up   =  key3_stable_n &&  key4_stable_n;

    reg  chord_hold;                 // 组合键进行中（含松开后 1 拍）
    reg  both_up_q;
    reg  [IDLE_W-1:0] idle_count;

    wire chord_start = both_down && !chord_hold;
    wire swallow     = chord_hold || both_down;
    wire ev3c = key3_click_in & ~swallow;
    wire ev3l = key3_long_in  & ~swallow;
    wire ev4c = key4_click_in & ~swallow;
    wire ev4l = key4_long_in  & ~swallow;
    wire any_key = !key3_stable_n || !key4_stable_n || ev3c || ev3l || ev4c || ev4l;

    assign key3_click_out = ev3c & ~adj_mode;
    assign key3_long_out  = ev3l & ~adj_mode;
    assign key4_click_out = ev4c & ~adj_mode;
    assign key4_long_out  = ev4l & ~adj_mode;
    assign adj_inc        = ev3c &  adj_mode;
    assign adj_dec        = ev4c &  adj_mode;
    assign adj_default    = ev4l &  adj_mode;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            chord_hold <= 1'b0;
            both_up_q  <= 1'b1;
            adj_mode   <= 1'b0;
            adj_sel    <= 1'b0;
            adj_show   <= 1'b0;
            idle_count <= {IDLE_W{1'b0}};
        end else begin
            adj_show  <= 1'b0;
            both_up_q <= both_up;
            if (both_down)
                chord_hold <= 1'b1;
            else if (both_up && both_up_q)
                chord_hold <= 1'b0;

            if (!enable) begin
                adj_mode <= 1'b0;
            end else if (chord_start) begin
                adj_mode   <= ~adj_mode;
                adj_show   <= ~adj_mode;            // 进入时显示当前项
                idle_count <= {IDLE_W{1'b0}};
            end else if (adj_mode) begin
                if (ev3l) begin
                    adj_sel  <= ~adj_sel;
                    adj_show <= 1'b1;
                end
                if (any_key)
                    idle_count <= {IDLE_W{1'b0}};
                else if (!tick)
                    idle_count <= idle_count;
                else if (idle_count == IDLE_EXIT_MS - 1)
                    adj_mode   <= 1'b0;              // 超时自动退出
                else
                    idle_count <= idle_count + 1'b1;
            end
        end
    end
endmodule
