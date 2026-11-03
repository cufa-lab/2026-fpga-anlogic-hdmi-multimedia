`timescale 1ns/1ps
// tb_pcm_contract_adapter：PCM 合同适配层
//   * valid/ready/数据直通(零延迟、不丢不重)
//   * 合法生产者(反压期间数据保持)不报错
//   * 非法生产者(反压期间改数据)contract_error 置位并保持
//   * pcm_test_tone(SAMPLE_DIV=521) 作为合法生产者：速率 25e6/521 Hz、16 点 LUT 循环
module tb_pcm_contract_adapter;
    reg clk = 1'b0;
    always #20 clk = ~clk;
    reg rst_n = 1'b0;

    // ---------------- 合法生产者：pcm_test_tone ----------------
    reg ready_a = 1'b1;
    wire src_valid, src_ready, pcm_valid;
    wire signed [15:0] src_l, src_r, pcm_l, pcm_r;
    wire [3:0] rom_index;
    wire accepted;
    wire err_a;

    pcm_test_tone u_tone (
        .clk(clk), .rst_n(rst_n), .enable(1'b1),
        .sample_valid(src_valid), .sample_ready(src_ready),
        .sample_left(src_l), .sample_right(src_r),
        .rom_index(rom_index), .sample_accepted(accepted)
    );
    m06_pcm_contract_adapter u_ok (
        .clk(clk), .rst_n(rst_n),
        .source_valid(src_valid), .source_ready(src_ready),
        .source_left(src_l), .source_right(src_r),
        .pcm_valid(pcm_valid), .pcm_ready(ready_a),
        .pcm_left(pcm_l), .pcm_right(pcm_r),
        .contract_error(err_a)
    );

    // ---------------- 非法生产者：反压时改数据 ----------------
    reg bad_valid = 1'b0;
    reg signed [15:0] bad_l = 16'sd0;
    wire bad_src_ready, bad_pcm_valid, err_b;
    wire signed [15:0] bad_pcm_l, bad_pcm_r;
    reg ready_b = 1'b0;
    m06_pcm_contract_adapter u_bad (
        .clk(clk), .rst_n(rst_n),
        .source_valid(bad_valid), .source_ready(bad_src_ready),
        .source_left(bad_l), .source_right(bad_l),
        .pcm_valid(bad_pcm_valid), .pcm_ready(ready_b),
        .pcm_left(bad_pcm_l), .pcm_right(bad_pcm_r),
        .contract_error(err_b)
    );

    integer errors = 0;
    integer n_acc = 0, first_acc = -1, last_acc = -1, cyc = 0;
    reg [3:0] expect_idx = 4'd0;
    reg signed [15:0] lut [0:15];
    integer i;

    always @(posedge clk) if (rst_n) begin
        cyc = cyc + 1;
        if (pcm_valid !== src_valid || pcm_l !== src_l || pcm_r !== src_r || src_ready !== ready_a) begin
            errors = errors + 1; $display("FAIL adapter not a pass-through at cycle %0d", cyc);
        end
        if (pcm_valid && ready_a) begin
            if (pcm_l !== lut[expect_idx]) begin
                errors = errors + 1; $display("FAIL sample %0d = %0d expect LUT[%0d]=%0d", n_acc, pcm_l, expect_idx, lut[expect_idx]);
            end
            expect_idx = expect_idx + 1'b1;
            if (first_acc < 0) first_acc = cyc;
            last_acc = cyc;
            n_acc = n_acc + 1;
        end
    end

    initial begin
        lut[0]=0; lut[1]=4592; lut[2]=8485; lut[3]=11087; lut[4]=12000; lut[5]=11087; lut[6]=8485; lut[7]=4592;
        lut[8]=0; lut[9]=-4592; lut[10]=-8485; lut[11]=-11087; lut[12]=-12000; lut[13]=-11087; lut[14]=-8485; lut[15]=-4592;
        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        repeat (521 * 200) @(posedge clk);
        // 周期：相邻接收间隔恒为 521 拍
        if (n_acc < 199 || (last_acc - first_acc) != 521 * (n_acc - 1)) begin
            errors = errors + 1;
            $display("FAIL pcm_test_tone period: n=%0d span=%0d", n_acc, last_acc - first_acc);
        end
        // 随机反压(合法生产者)
        for (i = 0; i < 521 * 100; i = i + 1) begin @(negedge clk); ready_a = ($random % 3) != 0; end
        @(negedge clk); ready_a = 1'b1;
        repeat (1000) @(posedge clk);
        if (err_a) begin errors = errors + 1; $display("FAIL contract_error on a legal producer"); end

        // 非法生产者：valid 保持但数据在 !ready 期间变化
        @(negedge clk); bad_valid = 1'b1; bad_l = 16'sd100;
        repeat (3) @(posedge clk);
        if (err_b) begin errors = errors + 1; $display("FAIL error before violation"); end
        @(negedge clk); bad_l = 16'sd200;
        repeat (2) @(posedge clk);
        if (!err_b) begin errors = errors + 1; $display("FAIL contract violation not detected"); end
        @(negedge clk); ready_b = 1'b1; bad_valid = 1'b0;
        repeat (5) @(posedge clk);
        if (!err_b) begin errors = errors + 1; $display("FAIL contract_error not sticky"); end

        $display("INFO pcm_test_tone: %0d samples, period 521 cycles = %0d Hz", n_acc, 25000000 / 521);
        if (errors == 0) $display("PASS tb_pcm_contract_adapter: pass-through, legal/illegal producer, test tone 521-cycle period + LUT order");
        else             $display("FAIL tb_pcm_contract_adapter: %0d errors", errors);
        $finish;
    end
endmodule
