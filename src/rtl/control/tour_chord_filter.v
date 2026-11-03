`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// tour_chord_filter —— 拓展3做深 ② 导览运镜：KEY2+KEY4 组合键（sys_clk 域）
//
// 做法同 picture_adjust_ctrl：两个键都按下的那一刻起，到两个键都松开后再过 1 拍
// （router 的 KEY2 事件比电平晚 1 拍），KEY2 / KEY4 的单击、长按事件全部吞掉，
// 所以同按不会顺带切图 / 调音量 / 改周期。组合键按下沿给 chord（单拍）。
// 导览中（tour_on）：KEY2 单击 → tour_next（跳到下一个焦点），KEY2 长按吞掉（不切图）；
// KEY4 照常（音量 / 周期），KEY3 不经过本模块。导览外所有事件原样透传。
// 纯组合 + 2 个触发器，没有计数器。
// -----------------------------------------------------------------------------
module tour_chord_filter (
    input  wire clk,
    input  wire rst_n,
    input  wire tour_on,

    input  wire key2_stable_n,      // 消抖后电平，低 = 按下
    input  wire key4_stable_n,
    input  wire key2_click_in,
    input  wire key2_long_in,
    input  wire key4_click_in,
    input  wire key4_long_in,

    output wire key2_click_out,
    output wire key2_long_out,
    output wire key4_click_out,
    output wire key4_long_out,
    output wire chord,              // 组合键按下（单拍）
    output wire tour_next           // 导览中 KEY2 单击（单拍）
);
    wire both_down = !key2_stable_n && !key4_stable_n;
    wire both_up   =  key2_stable_n &&  key4_stable_n;

    reg chord_hold;
    reg both_up_q;

    wire swallow = chord_hold || both_down;
    wire ev2c = key2_click_in & ~swallow;
    wire ev2l = key2_long_in  & ~swallow;

    assign chord          = both_down && !chord_hold;
    assign key2_click_out = ev2c & ~tour_on;
    assign key2_long_out  = ev2l & ~tour_on;
    assign tour_next      = ev2c &  tour_on;
    assign key4_click_out = key4_click_in & ~swallow;
    assign key4_long_out  = key4_long_in  & ~swallow;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            chord_hold <= 1'b0;
            both_up_q  <= 1'b1;
        end else begin
            both_up_q <= both_up;
            if (both_down)
                chord_hold <= 1'b1;
            else if (both_up && both_up_q)
                chord_hold <= 1'b0;
        end
    end
endmodule
