`timescale 1ns/1ps

module bh1750_reader #(
    parameter integer CLK_HZ    = 50_000_000,
    parameter integer IIC_HZ    = 100_000,
    parameter [6:0]   DEV_ADDR  = 7'h23,
    parameter integer MEAS_MS   = 180,
    parameter integer PERIOD_MS = 500
) (
    input  wire        clk,
    input  wire        rst_n,
    output wire        scl_o_low,     // 1 = 拉低 SCL（顶层接三态，0 = 释放）
    output wire        sda_o_low,     // 1 = 拉低 SDA
    input  wire        sda_i,
    output reg         tick_ms,
    output reg         raw_valid,
    output reg  [15:0] raw,
    output reg         sensor_ok
);
    localparam integer QTR_CYCLES = CLK_HZ / (IIC_HZ * 4);           // 50MHz/400k = 125
    localparam integer QTR_W      = $clog2(QTR_CYCLES);
    localparam integer QPMS       = IIC_HZ * 4 / 1000;                // 每毫秒 400 个 1/4 位
    localparam integer QPMS_W     = $clog2(QPMS);
    localparam integer WAIT_W     = $clog2(PERIOD_MS + 1);
    localparam integer REST_MS    = PERIOD_MS - MEAS_MS;

    // 操作码：START / STOP / 发送 / 接收+ACK / 接收+NACK / 等待
    localparam [2:0] OP_START = 3'd0, OP_STOP = 3'd1, OP_TX = 3'd2,
                     OP_RXA   = 3'd3, OP_RXN  = 3'd4, OP_WAIT = 3'd5;

    // ---------------- 分频：1/4 位节拍 + 1ms 节拍 ----------------
    reg [QTR_W-1:0]  qdiv;
    reg [QPMS_W-1:0] msdiv;
    wire qtick = (qdiv == QTR_CYCLES - 1);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            qdiv    <= {QTR_W{1'b0}};
            msdiv   <= {QPMS_W{1'b0}};
            tick_ms <= 1'b0;
        end else begin
            tick_ms <= 1'b0;
            qdiv    <= qtick ? {QTR_W{1'b0}} : qdiv + 1'b1;
            if (qtick) begin
                if (msdiv == QPMS - 1) begin
                    msdiv   <= {QPMS_W{1'b0}};
                    tick_ms <= 1'b1;
                end else
                    msdiv <= msdiv + 1'b1;
            end
        end
    end

    // ---------------- 操作序列（step → 操作码 / 数据） ----------------
    //  0 START  1 TX addr+W  2 TX 0x01  3 STOP  4 START  5 TX addr+W  6 TX 0x20  7 STOP
    //  8 WAIT MEAS_MS  9 START  10 TX addr+R  11 RX+ACK(高字节)  12 RX+NACK(低字节)  13 STOP  14 WAIT 余下时间
    reg [3:0] step;
    reg [2:0] op;
    reg [7:0] op_data;
    always @* begin
        op_data = {DEV_ADDR, 1'b0};
        case (step)
            4'd0, 4'd4, 4'd9: op = OP_START;
            4'd3, 4'd7, 4'd13: op = OP_STOP;
            4'd2:  begin op = OP_TX; op_data = 8'h01; end
            4'd6:  begin op = OP_TX; op_data = 8'h20; end
            4'd10: begin op = OP_TX; op_data = {DEV_ADDR, 1'b1}; end
            4'd11: op = OP_RXA;
            4'd12: op = OP_RXN;
            4'd8, 4'd14: op = OP_WAIT;
            default: op = OP_TX;                // 1 / 5：地址 + 写
        endcase
    end

    reg  [1:0]  q;
    reg  [3:0]  bitn;            // 数据字节内位号 0..8
    reg  [8:0]  sr;
    reg         scl_low, sda_low;
    reg  [WAIT_W-1:0] wait_ms;
    reg         loaded;          // 本操作的 sr 已装载（字节类操作第 0 拍装）
    reg  [7:0]  hi_byte;

    assign scl_o_low = scl_low;
    assign sda_o_low = sda_low;

    wire byte_op = (op == OP_TX) || (op == OP_RXA) || (op == OP_RXN);
    wire [WAIT_W-1:0] wait_target = (step == 4'd8) ? MEAS_MS[WAIT_W-1:0] : REST_MS[WAIT_W-1:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            step      <= 4'd14;           // 上电先等一个间隔再开始（模块上电稳定）
            q         <= 2'd0;
            bitn      <= 4'd0;
            sr        <= 9'h1FF;
            scl_low   <= 1'b0;
            sda_low   <= 1'b0;
            wait_ms   <= {WAIT_W{1'b0}};
            loaded    <= 1'b0;
            hi_byte   <= 8'd0;
            raw       <= 16'd0;
            raw_valid <= 1'b0;
            sensor_ok <= 1'b0;
        end else begin
            raw_valid <= 1'b0;
            if (op == OP_WAIT) begin
                scl_low <= 1'b0;
                sda_low <= 1'b0;
                if (tick_ms) begin
                    if (wait_ms == wait_target) begin
                        wait_ms <= {WAIT_W{1'b0}};
                        step    <= (step == 4'd14) ? 4'd0 : step + 4'd1;
                    end else
                        wait_ms <= wait_ms + 1'b1;
                end
            end else if (byte_op && !loaded) begin
                // 字节类操作先装移位寄存器（不占 1/4 位节拍）
                sr     <= (op == OP_TX) ? {op_data, 1'b1} : {8'hFF, (op == OP_RXN)};
                bitn   <= 4'd0;
                loaded <= 1'b1;
            end else if (qtick) begin
                q <= q + 2'd1;
                case (op)
                    OP_START: begin
                        case (q)
                            2'd0: begin scl_low <= 1'b0; sda_low <= 1'b0; end
                            2'd1: sda_low <= 1'b1;
                            2'd2: scl_low <= 1'b1;
                            default: step <= step + 4'd1;
                        endcase
                    end
                    OP_STOP: begin
                        case (q)
                            2'd0: begin scl_low <= 1'b1; sda_low <= 1'b1; end
                            2'd1: scl_low <= 1'b0;
                            2'd2: sda_low <= 1'b0;
                            default: step <= step + 4'd1;
                        endcase
                    end
                    default: begin            // 发送 / 接收：9 位
                        case (q)
                            2'd0: begin scl_low <= 1'b1; sda_low <= ~sr[8]; end
                            2'd1: scl_low <= 1'b0;
                            2'd2: sr <= {sr[7:0], sda_i};
                            default: begin
                                scl_low <= 1'b1;
                                if (bitn == 4'd8) begin
                                    loaded <= 1'b0;
                                    if (op == OP_TX && sr[0]) begin
                                        // 从机 NACK：模块没接 / 地址不对 → 发 STOP，跳到周期末尾等待重试
                                        sensor_ok <= 1'b0;
                                        step      <= 4'd13;
                                    end else begin
                                        step <= step + 4'd1;
                                        if (op == OP_RXA)
                                            hi_byte <= sr[8:1];
                                        if (op == OP_RXN) begin
                                            raw       <= {hi_byte, sr[8:1]};
                                            raw_valid <= 1'b1;
                                            sensor_ok <= 1'b1;
                                        end
                                    end
                                end else
                                    bitn <= bitn + 4'd1;
                            end
                        endcase
                    end
                endcase
            end
        end
    end
endmodule
