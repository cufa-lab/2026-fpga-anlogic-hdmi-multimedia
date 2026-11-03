`timescale 1ns/1ps

module ambient_brightness #(
    parameter integer MIN_BASE = 64,        // 最暗环境下的基准亮度
    parameter integer L_LO     = 12,        // 4·log2(raw) 低于此值都按最暗处理
    parameter integer HYST     = 8,
    parameter integer RAMP_MS  = 8
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        tick_ms,
    input  wire        sensor_ok,
    input  wire        raw_valid,
    input  wire [15:0] raw,
    input  wire [8:0]  offset,             // 有符号：按键偏移
    output reg  [7:0]  base,               // 当前环境基准（policy 换算偏移用）
    output reg  [7:0]  eff_brightness,     // 送显示链路 / 参数条 / 数码管的真实亮度
    output reg         env_change
);
    localparam integer RAMP_W = (RAMP_MS <= 1) ? 1 : $clog2(RAMP_MS);

    // ---------------- 4·log2(raw)：前导 1 位置 + 其后 2 位 ----------------
    reg [3:0] msb;
    reg [1:0] frac;
    integer   i;
    always @* begin
        msb = 4'd0;
        for (i = 1; i < 16; i = i + 1)
            if (raw[i]) msb = i[3:0];
        case (msb)
            4'd0:    frac = 2'd0;
            4'd1:    frac = {raw[0], 1'b0};
            default: frac = raw[msb-1 -: 2];
        endcase
    end
    wire [5:0] lval = {msb, frac};

    // ---------------- 目标基准 ----------------
    wire [5:0] over  = (lval > L_LO) ? lval - L_LO : 6'd0;
    wire [8:0] slope = {over, 3'b000} - {3'b000, over};                // ×7，≤ 51×7 = 357
    wire [9:0] tsum  = MIN_BASE + slope;
    wire [7:0] map_base = (tsum > 10'd255) ? 8'd255 : tsum[7:0];

    reg  [7:0] target;
    reg        seen;            // 本次插上后已收到过读数
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            target <= 8'd255;
            seen   <= 1'b0;
        end else if (!sensor_ok) begin
            target <= 8'd255;
            seen   <= 1'b0;
        end else if (raw_valid) begin
            target <= map_base;
            seen   <= 1'b1;
        end
    end

    // ---------------- 迟滞 + 斜坡 ----------------
    reg              ramping;
    reg              armed;     // 第一次读数已落定，之后的斜坡启动才算"新环境"
    reg [RAMP_W-1:0] ramp_div;
    wire       up   = target > base;
    wire [7:0] diff = up ? target - base : base - target;
    wire       start = !ramping && (diff > HYST);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            base       <= 8'd255;
            ramping    <= 1'b0;
            armed      <= 1'b0;
            ramp_div   <= {RAMP_W{1'b0}};
            env_change <= 1'b0;
        end else begin
            env_change <= 1'b0;
            if (!sensor_ok)
                armed <= 1'b0;
            if (tick_ms) begin
                ramp_div <= (ramp_div == RAMP_MS - 1) ? {RAMP_W{1'b0}} : ramp_div + 1'b1;
                if (diff == 8'd0)
                    ramping <= 1'b0;
                else if (start)
                    ramping <= 1'b1;
                if (start && armed && sensor_ok)
                    env_change <= 1'b1;
                if (seen && sensor_ok && !start && !ramping)
                    armed <= 1'b1;              // 第一次读数没引起斜坡，或第一段斜坡已走完
                if (ramping && (ramp_div == 0) && (diff != 8'd0))
                    base <= up ? base + 8'd1 : base - 8'd1;
            end
        end
    end

    // ---------------- 显示亮度 = clamp(base + offset) ----------------
    wire signed [9:0] sum = $signed({2'b00, base}) + $signed({offset[8], offset});
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            eff_brightness <= 8'd255;
        else if (sum[9])
            eff_brightness <= 8'd0;
        else if (sum[8])
            eff_brightness <= 8'd255;
        else
            eff_brightness <= sum[7:0];
    end
endmodule
