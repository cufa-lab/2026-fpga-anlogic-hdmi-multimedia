`timescale 1ns/1ps
// -----------------------------------------------------------------------------
//   R1 异步压下：ext_rst_n 下降后不等时钟沿，rst_n_sync 立即为 0（时钟停振时也一样）
//   R2 同步释放：ext_rst_n 在时钟周期内任意时刻释放，rst_n_sync 只在时钟上升沿变 1，
//      且恰好在释放后的第 2 个上升沿（2 级触发器）
//   R3 窄脉冲：比一个时钟周期还短的复位脉冲也能完整复位一次
//   R4 两个异频时钟域同时释放：各自在本域时钟沿释放
//   S1 sync_2ff：电平变化 2 个目的沿后出现在输出；复位值 = RESET_VAL(0/1 各测一次)
// -----------------------------------------------------------------------------
module tb_reset_sync;
    reg clk_a = 1'b0, clk_b = 1'b0;
    reg run_a = 1'b1;
    always #5    if (run_a) clk_a = ~clk_a;      // 100 MHz，可停振
    always #6.25 clk_b = ~clk_b;                 // 80 MHz
    reg ext_rst_n = 1'b0;
    wire rst_a, rst_b;
    reset_sync u_a (.clk(clk_a), .ext_rst_n(ext_rst_n), .rst_n_sync(rst_a));
    reset_sync u_b (.clk(clk_b), .ext_rst_n(ext_rst_n), .rst_n_sync(rst_b));

    reg  s_rst_n = 1'b0, s_in = 1'b0;
    wire s_out0, s_out1;
    sync_2ff #(.RESET_VAL(1'b0)) u_s0 (.clk(clk_b), .rst_n(s_rst_n), .async_in(s_in), .sync_out(s_out0));
    sync_2ff #(.RESET_VAL(1'b1)) u_s1 (.clk(clk_b), .rst_n(s_rst_n), .async_in(s_in), .sync_out(s_out1));

    integer errors = 0;
    task fail(input [8*80-1:0] msg);
        begin errors = errors + 1; $display("FAIL @%0t: %0s", $time, msg); end
    endtask

    realtime t_pos_a = 0, t_pos_b = 0;
    // R2：rst_a 只能在 clk_a 上升沿之后变化
    always @(rst_a) if (rst_a === 1'b1 && !(clk_a === 1'b1 && $realtime == t_pos_a)) fail("R2 rst_a released off a clk_a rising edge");
    always @(posedge clk_a) t_pos_a = $realtime;
    always @(posedge clk_b) t_pos_b = $realtime;
    always @(rst_b) if (rst_b === 1'b1 && !(clk_b === 1'b1 && $realtime == t_pos_b)) fail("R4 rst_b released off a clk_b rising edge");

    integer edges, k;
    initial begin
        #100;
        if (rst_a !== 1'b0 || rst_b !== 1'b0) fail("outputs not in reset while ext_rst_n=0");
        // ---- R2：释放点扫过一个时钟周期内 0.3 ns 步进的各相位 ----
        for (k = 0; k < 33; k = k + 1) begin
            ext_rst_n = 1'b0; #1;
            if (rst_a !== 1'b0 || rst_b !== 1'b0) fail("R1 reset not asserted immediately");
            #40; @(posedge clk_a); #(0.15 + k * 0.3);          // 释放点相对 clk_a 上升沿 0.15..9.75 ns
            ext_rst_n = 1'b1;
            edges = 0;
            while (rst_a !== 1'b1) begin @(posedge clk_a); #0.01; edges = edges + 1; if (edges > 5) begin fail("R2 no release"); rst_a_force_exit; end end
            if (edges != 2) begin $display("  k=%0d release after %0d edges", k, edges); fail("R2 release not on the 2nd rising edge"); end
            wait (rst_b === 1'b1);
            #50;
        end
        // ---- R1：时钟停振时压下复位也必须立即生效 ----
        @(negedge clk_a); run_a = 1'b0; #20;
        ext_rst_n = 1'b0; #0.1;
        if (rst_a !== 1'b0) fail("R1 reset needs a clock edge to assert");
        #30; ext_rst_n = 1'b1; #30;
        if (rst_a !== 1'b0) fail("R1 released without a clock");
        run_a = 1'b1; clk_a = 1'b0;
        wait (rst_a === 1'b1);
        // ---- R3：2 ns 窄脉冲(< 10 ns 周期)也复位 ----
        #33.3; ext_rst_n = 1'b0; #2; ext_rst_n = 1'b1; #0.1;
        if (rst_a !== 1'b0 || rst_b !== 1'b0) fail("R3 narrow reset pulse was missed");
        wait (rst_a === 1'b1 && rst_b === 1'b1);

        // ---- S1：sync_2ff ----
        #10;
        if (s_out0 !== 1'b0 || s_out1 !== 1'b1) fail("S1 sync_2ff reset value != RESET_VAL");
        s_rst_n = 1'b1; s_in = 1'b1;
        @(posedge clk_b); #0.1; @(posedge clk_b); #0.1;
        if (s_out0 !== 1'b1 || s_out1 !== 1'b1) fail("S1 sync_2ff: level not through after 2 edges");
        s_in = 1'b0; #0.1;
        @(posedge clk_b); #0.1;
        if (s_out0 !== 1'b1) fail("S1 sync_2ff: output changed after only 1 edge");
        @(posedge clk_b); #0.1;
        if (s_out0 !== 1'b0 || s_out1 !== 1'b0) fail("S1 sync_2ff: falling level not through after 2 edges");
        s_in = 1'b1; #20; s_rst_n = 1'b0; #0.1;
        if (s_out0 !== 1'b0 || s_out1 !== 1'b1) fail("S1 sync_2ff async reset");

        if (errors == 0) $display("PASS tb_reset_sync: async assert (even with clock stopped), release on exactly the 2nd own-domain edge at 33 phases, narrow pulse, 2 domains, sync_2ff");
        else             $display("FAIL tb_reset_sync: %0d errors", errors);
        $finish;
    end
    task rst_a_force_exit; begin $display("FAIL tb_reset_sync: hang"); $finish; end endtask
endmodule
