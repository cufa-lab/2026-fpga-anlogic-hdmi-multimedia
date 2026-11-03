`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// single_key_event —— 单键事件识别（KEY4 扩展键）
// 与 dual_key_chord_router 同一套约定：2FF 同步 → 计数消抖 → 松开时判定
//   按住 < LONG_CYCLES 松开 → click_event
//   按住 ≥ LONG_CYCLES 松开 → long_event
// 按键低有效（按下=0）。
// 逻辑优化：计数只在 tick 那拍前进（顶层接公共 1ms 节拍，参数按毫秒给；tb 接 1 = 逐拍，同原版）。
// -----------------------------------------------------------------------------
module single_key_event #(
    parameter integer DEBOUNCE_CYCLES = 20,    // tick 数（1ms 节拍 → 20ms）
    parameter integer LONG_CYCLES     = 500    // tick 数（0.5s）
) (
    input  wire clk,
    input  wire rst_n,
    input  wire tick,          // 时基使能（1ms 节拍；接 1 = 按时钟周期计）
    input  wire key_n,
    output reg  click_event,
    output reg  long_event,
    output reg  key_stable_n
);
    localparam integer DB_BITS   = (DEBOUNCE_CYCLES <= 2) ? 1 : $clog2(DEBOUNCE_CYCLES);
    localparam integer LONG_BITS = (LONG_CYCLES <= 2) ? 1 : $clog2(LONG_CYCLES + 1);

    (* async_reg = "true" *) reg key_sync1;
    (* async_reg = "true" *) reg key_sync2;
    reg [DB_BITS-1:0]   db_count;
    reg [LONG_BITS-1:0] hold_count;
    reg                 long_marked;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            key_sync1    <= 1'b1;
            key_sync2    <= 1'b1;
            db_count     <= {DB_BITS{1'b0}};
            key_stable_n <= 1'b1;
            hold_count   <= {LONG_BITS{1'b0}};
            long_marked  <= 1'b0;
            click_event  <= 1'b0;
            long_event   <= 1'b0;
        end else begin
            key_sync1   <= key_n;
            key_sync2   <= key_sync1;
            click_event <= 1'b0;
            long_event  <= 1'b0;

            if (key_sync2 == key_stable_n)
                db_count <= {DB_BITS{1'b0}};
            else if (!tick)
                db_count <= db_count;
            else if (db_count + 1 >= DEBOUNCE_CYCLES) begin
                db_count     <= {DB_BITS{1'b0}};
                key_stable_n <= key_sync2;
                if (!key_stable_n && key_sync2) begin
                    // 稳定地松开：按住时长决定事件类型
                    if (long_marked)
                        long_event  <= 1'b1;
                    else
                        click_event <= 1'b1;
                end
            end else
                db_count <= db_count + 1'b1;

            if (key_stable_n) begin
                hold_count  <= {LONG_BITS{1'b0}};
                long_marked <= 1'b0;
            end else if (!long_marked && tick) begin
                if (hold_count + 1 >= LONG_CYCLES)
                    long_marked <= 1'b1;
                else
                    hold_count <= hold_count + 1'b1;
            end
        end
    end
endmodule
