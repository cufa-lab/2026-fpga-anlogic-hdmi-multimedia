`timescale 1ns/1ps
// dual_key_chord_router + dual_key_long_hold_guard
// DEBOUNCE=8, HOLD=200, KEY3_LONG=100, KEY2_LONG=150
//   KEY2 短按 → key2_event；KEY2 长按 → key2_long_event（上一张）
//   KEY3 短按 → key3_event；KEY3 长按 → key3_long_event（亮度）
//   长按在松开时判定：按住不放只出一次，按住期间不出事件
//   KEY2+KEY3 同按 ≥HOLD → 恰好 1 次 save，且不产生任何单键事件（含长按）
//   松开后再次同按 → 能再次 save（guard 重新武装）
//   同按但不足 HOLD → 无任何事件
//   全程检查：每拍最多一个事件、事件均为单拍脉冲
module tb_dual_key_chord_router;
    reg clk = 0, rst_n = 0, k2 = 1, k3 = 1;
    always #5 clk = ~clk;
    wire e2, e2l, e3, e3l, save, locked, s2, s3;
    integer n2 = 0, n2l = 0, n3 = 0, n3l = 0, ns = 0, errors = 0;
    reg e2_d = 0, e2l_d = 0, e3_d = 0, e3l_d = 0, save_d = 0;

    dual_key_chord_router #(.DEBOUNCE_CYCLES(8), .HOLD_CYCLES(200),
                            .KEY3_LONG_CYCLES(100), .KEY2_LONG_CYCLES(150)) dut (
        .clk(clk), .rst_n(rst_n), .tick(1'b1), .save_enabled(1'b1), .key2_n(k2), .key3_n(k3),
        .key2_event(e2), .key2_long_event(e2l), .key3_event(e3), .key3_long_event(e3l),
        .save_event(save), .save_locked(locked), .key2_stable_n(s2), .key3_stable_n(s3)
    );
    always @(posedge clk) begin
        n2 = n2 + e2; n2l = n2l + e2l; n3 = n3 + e3; n3l = n3l + e3l; ns = ns + save;
        if (e2 + e2l + e3 + e3l + save > 1) begin
            $display("FAIL: two events in one cycle @%0t", $time); errors = errors + 1;
        end
        if ((e2 && e2_d) || (e2l && e2l_d) || (e3 && e3_d) || (e3l && e3l_d) || (save && save_d)) begin
            $display("FAIL: event wider than one cycle @%0t", $time); errors = errors + 1;
        end
        e2_d <= e2; e2l_d <= e2l; e3_d <= e3; e3l_d <= e3l; save_d <= save;
    end

    task idle(input integer c); repeat (c) @(posedge clk); endtask
    task check(input integer a, input integer al, input integer b, input integer c,
               input integer d, input [127:0] tag);
        begin
            if (n2 !== a || n2l !== al || n3 !== b || n3l !== c || ns !== d) begin
                $display("FAIL %0s: k2=%0d k2l=%0d k3=%0d k3l=%0d save=%0d (exp %0d %0d %0d %0d %0d)",
                         tag, n2, n2l, n3, n3l, ns, a, al, b, c, d);
                errors = errors + 1;
            end
        end
    endtask

    initial begin
        #50 rst_n = 1; idle(10);
        @(posedge clk) k2 <= 0; idle(40); k2 <= 1; idle(40);
        check(1, 0, 0, 0, 0, "key2 click");
        @(posedge clk) k3 <= 0; idle(40); k3 <= 1; idle(40);
        check(1, 0, 1, 0, 0, "key3 click");
        @(posedge clk) k3 <= 0; idle(150); k3 <= 1; idle(40);
        check(1, 0, 1, 1, 0, "key3 long");
        // KEY2 长按：按住期间不出事件，松开后出 1 次 key2_long
        @(posedge clk) k2 <= 0; idle(400);
        check(1, 0, 1, 1, 0, "key2 held (no event yet)");
        k2 <= 1; idle(40);
        check(1, 1, 1, 1, 0, "key2 long");
        // KEY2 按住刚好不足长按阈值 → 仍是单击
        @(posedge clk) k2 <= 0; idle(120); k2 <= 1; idle(40);
        check(2, 1, 1, 1, 0, "key2 just below long");
        // 同按 1 次（按住远超 HOLD）
        @(posedge clk) k2 <= 0; idle(3); k3 <= 0; idle(600);
        check(2, 1, 1, 1, 1, "chord hold");
        k2 <= 1; k3 <= 1; idle(40);
        check(2, 1, 1, 1, 1, "chord release");
        // 再次同按 → 重新武装
        @(posedge clk) k3 <= 0; idle(2); k2 <= 0; idle(300); k2 <= 1; k3 <= 1; idle(40);
        check(2, 1, 1, 1, 2, "chord again");
        // 同按不足 HOLD
        @(posedge clk) k2 <= 0; k3 <= 0; idle(80); k2 <= 1; k3 <= 1; idle(40);
        check(2, 1, 1, 1, 2, "short chord");
        // KEY2 先长按超过阈值，再按 KEY3 组合 → 只出 save，不出 key2_long
        @(posedge clk) k2 <= 0; idle(200); k3 <= 0; idle(400); k2 <= 1; k3 <= 1; idle(40);
        check(2, 1, 1, 1, 3, "long key2 then chord");
        // 复位后按键抖动（每 3 拍翻转，短于消抖）→ 无事件
        repeat (10) begin @(posedge clk) k2 <= ~k2; idle(2); end
        k2 <= 1; idle(40);
        check(2, 1, 1, 1, 3, "bounce ignored");
        if (errors == 0) $display("PASS tb_dual_key_chord_router");
        else             $display("FAIL tb_dual_key_chord_router errors=%0d", errors);
        $finish;
    end
endmodule
