`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// ir_presence_filter —— 外接红外反射避障模块 YL-62 的 OUT 消抖（sys_clk 域）
//
// YL-62：LM393 比较器开集输出 + 模块上 10K 上拉，检测到障碍（手遮挡）OUT = 低，同时模块绿灯亮。
//   接线：VCC 必须接 3.3V（J2-29），OUT = J2-31(J12)。FPGA 引脚再开内部上拉：模块没接时读到 1 = 无遮挡。
// 处理：2FF 同步 → 用公共 1ms 节拍计数，电平连续 STABLE_MS 不变才改变输出（比较器在门限附近会抖动）。
// present = 1 表示有手 / 物体遮挡。
// -----------------------------------------------------------------------------
module ir_presence_filter #(
    parameter integer STABLE_MS = 50
) (
    input  wire clk,
    input  wire rst_n,
    input  wire tick_ms,
    input  wire ir_out_n,       // 模块 OUT 引脚（低 = 检测到）
    output reg  present
);
    localparam integer CNT_W = $clog2(STABLE_MS + 1);

    (* async_reg = "true" *) reg s1, s2;
    reg [CNT_W-1:0] cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s1      <= 1'b1;
            s2      <= 1'b1;
            cnt     <= {CNT_W{1'b0}};
            present <= 1'b0;
        end else begin
            s1 <= ir_out_n;
            s2 <= s1;
            if (present == s2)                       // 新电平（present 与 OUT 同值 = 状态要翻转）
                begin
                    if (tick_ms) begin
                        if (cnt == STABLE_MS - 1) begin
                            cnt     <= {CNT_W{1'b0}};
                            present <= ~s2;
                        end else
                            cnt <= cnt + 1'b1;
                    end
                end
            else
                cnt <= {CNT_W{1'b0}};
        end
    end
endmodule
