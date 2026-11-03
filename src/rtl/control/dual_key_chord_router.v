`timescale 1ns/1ps
//   KEY2 单击 = 下一张（§3.2）  KEY2 长按 KEY2_LONG_CYCLES 后松开 = 上一张（§2.9 的 1s 长按上一张）
//   KEY3 单击 = 自动轮播        KEY3 长按 KEY3_LONG_CYCLES 后松开 = 亮度
//   KEY2+KEY3 同按 HOLD_CYCLES  = 保存（dual_key_long_hold_guard）
// 长按都在松开时判定，按住不放只出一次事件。
// 逻辑优化 L2：所有计数只在 tick 那拍前进（顶层接公共 1ms 节拍、参数按毫秒给；tb 接 1 = 逐拍，同原版），
//   长按计数位宽按参数取（原 2 个 32 位）。
module dual_key_chord_router #(
    parameter integer DEBOUNCE_CYCLES = 20,      // tick 数
    parameter integer HOLD_CYCLES = 3000,
    // KEY3 held alone for this long, then released, emits key3_long_event
    // (brightness ladder) instead of the single-click auto toggle.
    parameter integer KEY3_LONG_CYCLES = 500,
    // KEY2 held alone for this long, then released, emits key2_long_event
    // (previous image) instead of the single-click next.
    parameter integer KEY2_LONG_CYCLES = 1000
) (
    input  wire clk,
    input  wire rst_n,
    input  wire tick,            // 时基使能（1ms 节拍；接 1 = 按时钟周期计）
    input  wire save_enabled,
    input  wire key2_n,
    input  wire key3_n,
    output reg  key2_event,
    output reg  key2_long_event,
    output reg  key3_event,
    output reg  key3_long_event,
    output wire save_event,
    output wire save_locked,
    output wire key2_stable_n,
    output wire key3_stable_n
);
    localparam [1:0] ST_IDLE = 2'd0;
    localparam [1:0] ST_KEY2 = 2'd1;
    localparam [1:0] ST_KEY3 = 2'd2;
    localparam [1:0] ST_CHORD = 2'd3;

    reg [1:0] state;
    reg save_armed;
    wire guard_trigger;
    // 存全部设置（顺带省资源）：KEY2 / KEY3 长按计数只在各自的 ST_KEY2 / ST_KEY3 里走、进状态都清零，
    //   两者从不同时使用 → 共用一个计数器 + 一个"已到长按"标志（原各一套）；到长按后停计（原 < LONG 比较）
    localparam integer K3W = $clog2(KEY3_LONG_CYCLES + 1);
    localparam integer K2W = $clog2(KEY2_LONG_CYCLES + 1);
    localparam integer KW  = (K2W > K3W) ? K2W : K3W;
    reg [KW-1:0] hold_count;
    reg long_marked;

    initial begin
        if (DEBOUNCE_CYCLES < 1 || HOLD_CYCLES < 1 || KEY2_LONG_CYCLES < 1 || KEY3_LONG_CYCLES < 1) begin
            $display("DUAL_KEY_CHORD_ROUTER_CONFIG_ERROR invalid timing parameter");
            $finish;
        end
    end

    dual_key_long_hold_guard #(
        .DEBOUNCE_CYCLES(DEBOUNCE_CYCLES),
        .HOLD_CYCLES(HOLD_CYCLES)
    ) u_guard (
        .clk(clk), .rst_n(rst_n), .tick(tick),
        .enabled(save_enabled && save_armed && state == ST_CHORD),
        .key2_n(key2_n), .key3_n(key3_n), .trigger(guard_trigger),
        .locked(save_locked), .key2_stable_n(key2_stable_n),
        .key3_stable_n(key3_stable_n)
    );

    assign save_event = guard_trigger;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            save_armed <= 1'b0;
            key2_event <= 1'b0;
            key3_event <= 1'b0;
            key3_long_event <= 1'b0;
            hold_count <= 0;
            long_marked <= 1'b0;
            key2_long_event <= 1'b0;
        end else begin
            key2_event <= 1'b0;
            key3_event <= 1'b0;
            key3_long_event <= 1'b0;
            key2_long_event <= 1'b0;

            if (!save_enabled)
                save_armed <= 1'b0;
            else if (key2_stable_n && key3_stable_n)
                save_armed <= 1'b1;

            case (state)
                ST_IDLE: begin
                    if (!key2_stable_n && !key3_stable_n)
                        state <= ST_CHORD;
                    else if (!key2_stable_n) begin
                        state <= ST_KEY2;
                        hold_count <= 0;
                        long_marked <= 1'b0;
                    end
                    else if (!key3_stable_n) begin
                        state <= ST_KEY3;
                        hold_count <= 0;
                        long_marked <= 1'b0;
                    end
                end
                ST_KEY2: begin
                    if (!key2_stable_n && !key3_stable_n) begin
                        // KEY3 joining cancels the KEY2 long press and hands the
                        // pair to the save chord.
                        state <= ST_CHORD;
                        hold_count <= 0;
                        long_marked <= 1'b0;
                    end else if (key2_stable_n) begin
                        if (long_marked)
                            key2_long_event <= 1'b1;
                        else
                            key2_event <= 1'b1;
                        hold_count <= 0;
                        long_marked <= 1'b0;
                        state <= key3_stable_n ? ST_IDLE : ST_KEY3;
                    end else if (tick && !long_marked) begin
                        hold_count <= hold_count + 1'b1;
                        if (hold_count == KEY2_LONG_CYCLES - 1)
                            long_marked <= 1'b1;
                    end
                end
                ST_KEY3: begin
                    if (!key2_stable_n && !key3_stable_n) begin
                        // KEY2 joining cancels the long-press ladder and
                        // hands the pair to the save chord.
                        state <= ST_CHORD;
                        hold_count <= 0;
                        long_marked <= 1'b0;
                    end else if (key3_stable_n) begin
                        if (long_marked)
                            key3_long_event <= 1'b1;
                        else
                            key3_event <= 1'b1;
                        hold_count <= 0;
                        long_marked <= 1'b0;
                        state <= key2_stable_n ? ST_IDLE : ST_KEY2;
                    end else if (tick && !long_marked) begin
                        hold_count <= hold_count + 1'b1;
                        if (hold_count == KEY3_LONG_CYCLES - 1)
                            long_marked <= 1'b1;
                    end
                end
                ST_CHORD: begin
                    if (key2_stable_n && key3_stable_n)
                        state <= ST_IDLE;
                end
                default: state <= ST_IDLE;
            endcase
            end
    end
endmodule
