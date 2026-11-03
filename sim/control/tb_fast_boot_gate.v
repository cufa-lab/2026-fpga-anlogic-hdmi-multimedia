`timescale 1ns/1ps
// fast_boot_gate：穷举 {ingest_done, image_complete[3:0], player_enable_raw}
//   first_image_ready  = |image_complete（本工程适配：任意一张已提交）
//   player_enable_fast = player_enable_raw && first_image_ready
//   ingest_done 只作信息，不影响输出
module tb_fast_boot_gate;
    reg clk = 0, rst_n = 0;
    reg ingest = 0, raw = 0;
    reg [3:0] comp = 0;
    wire fast, first;
    integer i, errors = 0;

    fast_boot_gate dut (
        .clk(clk), .rst_n(rst_n), .ingest_done(ingest), .image_complete(|comp),
        .player_enable_raw(raw), .player_enable_fast(fast), .first_image_ready(first)
    );

    initial begin
        for (i = 0; i < 64; i = i + 1) begin
            {ingest, comp, raw} = i[5:0];
            #1;
            if (first !== (|comp) || fast !== (raw && (|comp))) begin
                $display("FAIL in=%b comp=%b raw=%b -> first=%b fast=%b", ingest, comp, raw, first, fast);
                errors = errors + 1;
            end
        end
        // 首图坏、第二张先完成（bit1）也必须放行
        ingest = 0; comp = 4'b0010; raw = 1; #1;
        if (!fast) begin $display("FAIL bad first image blocks player"); errors = errors + 1; end
        if (errors == 0) $display("PASS tb_fast_boot_gate");
        else             $display("FAIL tb_fast_boot_gate errors=%0d", errors);
        $finish;
    end
endmodule
