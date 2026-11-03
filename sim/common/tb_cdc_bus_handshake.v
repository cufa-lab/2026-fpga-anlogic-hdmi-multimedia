`timescale 1ns/1ps
// cdc_bus_handshake：100MHz → 25MHz，检查
//   1) 复位值 = RESET_VAL
//   2) 目的端只会出现源端真实写过的值（多比特不撕裂）
//   3) 源端停止变化后目的端收敛到最终值
module tb_cdc_bus_handshake;
    reg src_clk = 0, dst_clk = 0;
    reg src_rst_n = 0, dst_rst_n = 0;
    reg  [11:0] src_data = 12'h006;
    wire [11:0] dst_data;
    integer errors = 0;
    integer i;
    reg [4095:0] written;   // 记录源端出现过的值

    always #5  src_clk = ~src_clk;
    always #20 dst_clk = ~dst_clk;

    cdc_bus_handshake #(.WIDTH(12), .RESET_VAL(12'h006)) dut (
        .src_clk(src_clk), .src_rst_n(src_rst_n), .src_data(src_data),
        .dst_clk(dst_clk), .dst_rst_n(dst_rst_n), .dst_data(dst_data)
    );

    always @(posedge src_clk) written[src_data] <= 1'b1;

    always @(posedge dst_clk) if (dst_rst_n && !written[dst_data]) begin
        $display("FAIL: torn value %h", dst_data);
        errors = errors + 1;
    end

    initial begin
        written = 0;
        written[12'h006] = 1'b1;
        #33 src_rst_n = 1;
        #57 dst_rst_n = 1;
        if (dst_data !== 12'h006) begin $display("FAIL: reset value %h", dst_data); errors = errors + 1; end
        // 随机、快速变化（很多值会被合并）
        for (i = 0; i < 300; i = i + 1) begin
            @(posedge src_clk);
            if ($random % 7 == 0) src_data <= $random;
        end
        @(posedge src_clk) src_data <= 12'hA5C;
        repeat (40) @(posedge dst_clk);
        if (dst_data !== 12'hA5C) begin $display("FAIL: final %h", dst_data); errors = errors + 1; end
        // 单比特慢变化
        @(posedge src_clk) src_data <= 12'hA5D;
        repeat (20) @(posedge dst_clk);
        if (dst_data !== 12'hA5D) begin $display("FAIL: slow change %h", dst_data); errors = errors + 1; end
        if (errors == 0) $display("PASS tb_cdc_bus_handshake");
        else             $display("FAIL tb_cdc_bus_handshake errors=%0d", errors);
        $finish;
    end
endmodule
