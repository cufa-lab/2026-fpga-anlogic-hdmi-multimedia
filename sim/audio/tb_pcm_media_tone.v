`timescale 1ns/1ps
// tb_pcm_media_tone：音频数据源
//   * 25 MHz 分数累加 → 48 kHz：2 500 000 拍内恰好 4800 个采样节拍(无丢失)
//   * 每个输出都是 16 点正弦 LUT 的值(volume=255 时为 LUT*255>>>8)
//   * 波形确为正弦：一个周期内依次覆盖 LUT，正负对称
//   * media_id 变化 → 1.5 kHz 提示音(4800 采样) → 该图的旋律
//   * 下游反压时 valid/数据保持(M06 合同)，FIFO 不溢出
module tb_pcm_media_tone;
    reg clk = 1'b0;
    always #20 clk = ~clk;                      // 25 MHz
    reg rst_n = 1'b0;

    reg  [1:0] media_id = 2'd0;
    reg  [7:0] volume = 8'd255;
    reg        ready = 1'b1;
    wire       valid;
    wire signed [15:0] left, right;
    wire       overflow, blip_active;

    pcm_media_tone dut (
        .clk(clk), .rst_n(rst_n), .media_id(media_id), .volume(volume),
        .sample_valid(valid), .sample_ready(ready),
        .sample_left(left), .sample_right(right),
        .overflow(overflow), .blip_active(blip_active)
    );

    integer errors = 0;
    integer pops = 0, ticks = 0, cyc = 0;
    integer lut_hits [0:15];
    integer i, k;
    reg signed [15:0] lut [0:15];
    reg signed [15:0] held_l;
    reg held_valid;
    integer pos_max, neg_min;

    function integer lut_index;         // 返回 -1 表示不在表内
        input signed [15:0] v;
        integer j;
        begin
            lut_index = -1;
            for (j = 0; j < 16; j = j + 1)
                if (v == (lut[j] * 255) >>> 8) lut_index = j;
        end
    endfunction

    always @(posedge clk) if (rst_n) begin
        cyc = cyc + 1;
        if (dut.sample_tick) ticks = ticks + 1;
        if (valid && ready) begin
            pops = pops + 1;
            if (left !== right) begin errors = errors + 1; $display("FAIL L!=R"); end
            k = lut_index(left);
            if (k < 0) begin errors = errors + 1; $display("FAIL sample %0d not in 16-point LUT", left); end
            if (left > pos_max) pos_max = left;
            if (left < neg_min) neg_min = left;
        end
        // 合同：valid && !ready 时下一拍 valid 保持、数据不变
        if (held_valid && !(valid && left == held_l)) begin
            errors = errors + 1; $display("FAIL valid/data changed under back-pressure");
        end
        held_valid = valid && !ready;
        held_l = left;
        if (overflow) begin errors = errors + 1; $display("FAIL overflow"); end
        // LUT 对称(0/8、1/7…取值相同)，按写入时的相位索引逐项核对并计数
        if (dut.sample_push) begin
            k = dut.phase[19:16];
            if (dut.scaled_clamped !== (lut[k] * 255) >>> 8) begin
                errors = errors + 1; $display("FAIL push idx %0d value %0d", k, dut.scaled_clamped);
            end
            lut_hits[k] = lut_hits[k] + 1;
        end
    end

    integer blip_len;
    initial begin
        lut[0]=0; lut[1]=4592; lut[2]=8485; lut[3]=11087; lut[4]=12000; lut[5]=11087; lut[6]=8485; lut[7]=4592;
        lut[8]=0; lut[9]=-4592; lut[10]=-8485; lut[11]=-11087; lut[12]=-12000; lut[13]=-11087; lut[14]=-8485; lut[15]=-4592;
        for (i = 0; i < 16; i = i + 1) lut_hits[i] = 0;
        pos_max = -40000; neg_min = 40000; held_valid = 0;
        repeat (5) @(posedge clk);
        rst_n = 1'b1;

        // 1) 48 kHz 速率：100 ms = 2 500 000 拍 → 4800 个节拍
        repeat (2500000) @(posedge clk);
        if (ticks != 4800) begin errors = errors + 1; $display("FAIL ticks=%0d in 100 ms (expect 4800)", ticks); end
        if (pops < 4792 || pops > 4800) begin errors = errors + 1; $display("FAIL pops=%0d", pops); end
        $display("INFO 100 ms: sample_tick=%0d accepted=%0d -> %0d Hz", ticks, pops, ticks * 10);
        for (i = 0; i < 16; i = i + 1)
            if (lut_hits[i] == 0) begin errors = errors + 1; $display("FAIL LUT entry %0d never output", i); end
        if (pos_max != (12000*255)>>>8 || neg_min != (-12000*255)>>>8) begin
            errors = errors + 1; $display("FAIL peak %0d/%0d", pos_max, neg_min);
        end

        // 2) 反压：ready 随机拉低，合同检查在 always 块里
        for (i = 0; i < 200000; i = i + 1) begin
            @(negedge clk); ready = ($random % 4) != 0;
        end
        // 长时间停住(<8 个采样 = 166 us)不溢出
        @(negedge clk); ready = 1'b0;
        repeat (3000) @(posedge clk);
        @(negedge clk); ready = 1'b1;
        repeat (2000) @(posedge clk);

        // 3) media_id 切换 → 提示音
        @(negedge clk); media_id = 2'd2;
        repeat (3) @(posedge clk);
        if (!blip_active) begin errors = errors + 1; $display("FAIL no blip after media_id change"); end
        if (dut.phase_inc !== 20'd32768) begin errors = errors + 1; $display("FAIL blip inc %0d", dut.phase_inc); end
        blip_len = 0;
        while (blip_active) begin @(posedge clk); if (dut.sample_tick) blip_len = blip_len + 1; end
        if (blip_len < 4798 || blip_len > 4801) begin errors = errors + 1; $display("FAIL blip length %0d samples", blip_len); end
        repeat (2) @(posedge clk);
        if (dut.phase_inc !== 20'd14402) begin errors = errors + 1; $display("FAIL melody 2 note0 inc %0d", dut.phase_inc); end

        if (errors == 0) $display("PASS tb_pcm_media_tone: 48 kHz exact, 16-point sine LUT, blip + melody, back-pressure contract");
        else             $display("FAIL tb_pcm_media_tone: %0d errors", errors);
        $finish;
    end
endmodule
