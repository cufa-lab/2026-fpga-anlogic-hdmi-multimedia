`timescale 1ns / 1ps

module rgb_video_adapter #(
    parameter HS_IN_ACTIVE_LOW  = 1,  // 低有效
    parameter VS_IN_ACTIVE_LOW  = 1,
    parameter HS_OUT_ACTIVE_LOW = 1,  // 按厂商IP要求配置；1=维持低有效
    parameter VS_OUT_ACTIVE_LOW = 1
)(
    input  wire        pclk,       // 像素时钟
    input  wire        rst_n,      // 低有效复位
    input  wire        hs_in,      // 课程视频流
    input  wire        vs_in,
    input  wire        de_in,
    input  wire [23:0] rgb_in,
    output wire        video_clk,  // = pclk，直通不寄存
    output reg         video_hs,   // 极性由输出参数决定
    output reg         video_vs,
    output reg         video_de,
    output reg  [23:0] video_rgb   // 消隐期恒0
);

    // 先归一化成"高有效"内部语义，再按输出参数生成——两端极性互相独立
    wire hs_act = HS_IN_ACTIVE_LOW ? ~hs_in : hs_in;
    wire vs_act = VS_IN_ACTIVE_LOW ? ~vs_in : vs_in;

    assign video_clk = pclk;

    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            // 复位=空闲电平：不在同步脉冲里、无有效像素、全黑
            video_hs  <= HS_OUT_ACTIVE_LOW ? 1'b1 : 1'b0;
            video_vs  <= VS_OUT_ACTIVE_LOW ? 1'b1 : 1'b0;
            video_de  <= 1'b0;
        end else begin
            video_hs  <= HS_OUT_ACTIVE_LOW ? ~hs_act : hs_act;
            video_vs  <= VS_OUT_ACTIVE_LOW ? ~vs_act : vs_act;
            video_de  <= de_in;
        end
    end

    // 纯数据寄存器不接复位（下游/本级已按 de、锁定门控，复位期间时钟照常冲成 0）
    always @(posedge pclk)
        video_rgb <= de_in ? rgb_in : 24'h000000;  // 消隐防护

endmodule
