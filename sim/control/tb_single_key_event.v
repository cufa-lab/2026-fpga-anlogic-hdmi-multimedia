`timescale 1ns/1ps
// single_key_event：DEBOUNCE=8, LONG=100
// 抖动 → 无事件；短按 → 松开时 1 个 click；长按 → 松开时 1 个 long（按住期间不出事件）
module tb_single_key_event;
    reg clk = 0, rst_n = 0, key_n = 1;
    always #5 clk = ~clk;
    wire click, long_ev, stable_n;
    integer n_click = 0, n_long = 0, errors = 0;

    single_key_event #(.DEBOUNCE_CYCLES(8), .LONG_CYCLES(100)) dut (
        .clk(clk), .rst_n(rst_n), .tick(1'b1), .key_n(key_n),
        .click_event(click), .long_event(long_ev), .key_stable_n(stable_n)
    );
    always @(posedge clk) begin
        if (click)   n_click = n_click + 1;
        if (long_ev) n_long  = n_long + 1;
    end

    task press(input integer cycles);
        begin
            @(posedge clk) key_n <= 0;
            repeat (cycles) @(posedge clk);
            key_n <= 1;
            repeat (30) @(posedge clk);
        end
    endtask

    task check(input integer c, input integer l, input [127:0] tag);
        begin
            if (n_click !== c || n_long !== l) begin
                $display("FAIL %0s: click=%0d long=%0d (exp %0d %0d)", tag, n_click, n_long, c, l);
                errors = errors + 1;
            end
        end
    endtask

    integer i;
    initial begin
        #50 rst_n = 1;
        repeat (5) @(posedge clk);
        // 抖动：每次只低 3 拍
        for (i = 0; i < 6; i = i + 1) begin
            @(posedge clk) key_n <= 0; repeat (3) @(posedge clk); key_n <= 1; repeat (3) @(posedge clk);
        end
        repeat (30) @(posedge clk);
        check(0, 0, "bounce");
        press(40);            check(1, 0, "short");
        @(posedge clk) key_n <= 0;
        repeat (200) @(posedge clk);
        check(1, 0, "holding");   // 按住期间不出事件
        key_n <= 1; repeat (30) @(posedge clk);
        check(1, 1, "long");
        press(40);            check(2, 1, "short2");
        if (errors == 0) $display("PASS tb_single_key_event");
        else             $display("FAIL tb_single_key_event errors=%0d", errors);
        $finish;
    end
endmodule
