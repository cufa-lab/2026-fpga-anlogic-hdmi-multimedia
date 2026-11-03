`timescale 1ns/1ps
// -----------------------------------------------------------------------------
//   B1 复位后 WAIT_SDRAM：ingest/player 都为 0；SDRAM 超时 → 0x59 + error_event，
//      SDRAM 晚到仍能继续（0x59 清除，进入 INGEST，ingest_enable=1）
//   B2 INGEST 中 SD 卡一直未就绪 → 0x5A；SD 就绪后清除
//   B3 scan_done 但 image_count==0 → 0x53，停在 CHECK，player_enable 永远不放行；
//      错误持续期间 error_event 每 ERR_REPEAT_CYCLES 重发
//   B4 正常路径：写口忙期间 DRAIN 计数一直清零；写口最后一次忙之后恰好 DRAIN_CYCLES 拍放行
//   B5 RUN 只有复位才离开（scan_done/image_count 变化不影响）；复位异步清 player_enable
//   M  连续监视：player_enable=1 时 ingest_enable、sdram_init_done 必须都已为 1
// -----------------------------------------------------------------------------
module tb_boot_sequencer;
    localparam integer CLK_HZ = 10_000;
    localparam integer DRAIN  = 31;
    localparam integer REP    = 50;

    reg clk = 1'b0;
    always #5 clk = ~clk;
    reg rst_n = 1'b0;
    reg sdram_init_done = 1'b0, sd_init_done = 1'b0, scan_done = 1'b0, write_busy = 1'b0;
    reg [6:0] image_count = 7'd0;
    wire ingest_enable, player_enable, error_event;
    wire [7:0] boot_error;
    wire [2:0] boot_state;

    boot_sequencer #(
        .SDRAM_TIMEOUT_MS(50), .SD_READY_MS(200),     // L2：tick=1 → 1 "ms" = 1 拍，与原 CLK_HZ=10k 时的 50 / 200 拍相同
        .DRAIN_CYCLES(DRAIN), .ERR_REPEAT_MS(REP)
    ) dut (
        .clk(clk), .rst_n(rst_n), .tick(1'b1), .sdram_init_done(sdram_init_done), .sd_init_done(sd_init_done),
        .scan_done(scan_done), .image_count(image_count), .write_busy(write_busy),
        .ingest_enable(ingest_enable), .player_enable(player_enable),
        .boot_error(boot_error), .error_event(error_event), .boot_state(boot_state)
    );

    integer errors = 0;
    task fail(input [8*80-1:0] msg);
        begin errors = errors + 1; $display("FAIL @%0t: %0s", $time, msg); end
    endtask

    // M：放行必须在 SDRAM 初始化完成、扫描已放行之后
    always @(posedge clk) if (player_enable === 1'b1 && !(ingest_enable === 1'b1 && sdram_init_done === 1'b1))
        fail("M player_enable without ingest_enable / sdram_init_done");

    integer n_evt = 0;
    always @(posedge clk) if (error_event) n_evt = n_evt + 1;

    task tick(input integer n); begin repeat (n) @(posedge clk); #1; end endtask

    integer k, ev0, t_last_busy, t_pe;
    initial begin
        #23 rst_n = 1'b1;
        tick(2);
        // ---- B1 ----
        if (boot_state !== 3'd0 || ingest_enable || player_enable || boot_error !== 8'h00)
            fail("B1 bad state after reset");
        tick(40);
        if (boot_error !== 8'h00) fail("B1 0x59 raised before the SDRAM timeout");
        ev0 = n_evt;
        tick(20);
        if (boot_error !== 8'h59) fail("B1 SDRAM timeout did not raise 0x59");
        if (n_evt == ev0) fail("B1 no error_event for 0x59");
        if (ingest_enable) fail("B1 ingest enabled while SDRAM not initialised");
        sdram_init_done = 1'b1;
        tick(3);
        if (boot_error !== 8'h00) fail("B1 0x59 not cleared after late SDRAM init");
        if (boot_state !== 3'd1 || !ingest_enable) fail("B1 did not enter INGEST after SDRAM init");

        // ---- B2 ----
        tick(190);
        if (boot_error !== 8'h00) fail("B2 0x5A before the SD ready timeout");
        tick(20);
        if (boot_error !== 8'h5A) fail("B2 SD never ready did not raise 0x5A");
        sd_init_done = 1'b1;
        tick(3);
        if (boot_error !== 8'h00) fail("B2 0x5A not cleared after SD ready");
        tick(400);                                   // SD 就绪期间不超时
        if (boot_error !== 8'h00) fail("B2 0x5A while SD ready");

        // ---- B3 ----
        scan_done = 1'b1; image_count = 7'd0;
        tick(3);
        if (boot_error !== 8'h53 || boot_state !== 3'd2) fail("B3 empty card did not raise 0x53 in CHECK");
        ev0 = n_evt;
        tick(REP * 4);
        if (n_evt - ev0 < 3 || n_evt - ev0 > 5) begin
            $display("  events in %0d cycles: %0d", REP * 4, n_evt - ev0);
            fail("B3 error_event not repeated every ERR_REPEAT_CYCLES");
        end
        if (player_enable) fail("B3 player enabled without resources");

        // ---- B4 ----
        rst_n = 1'b0; #1;
        if (player_enable || ingest_enable || boot_state !== 3'd0) fail("B4 reset not asynchronous");
        scan_done = 1'b0; image_count = 7'd0; tick(2);
        rst_n = 1'b1; tick(3);
        if (boot_state !== 3'd1) fail("B4 did not go straight to INGEST with SDRAM already ready");
        tick(10);
        write_busy = 1'b1; image_count = 7'd1; scan_done = 1'b1;
        tick(3);
        if (boot_state !== 3'd3) fail("B4 not in DRAIN after scan_done with an image");
        for (k = 0; k < 6; k = k + 1) begin          // 写口断续忙：每段空闲 < DRAIN
            write_busy = 1'b1; tick(7);
            write_busy = 1'b0; tick(DRAIN - 5);
            if (player_enable) fail("B4 released while the write port was still active");
        end
        write_busy = 1'b1; tick(4);
        @(negedge clk) write_busy = 1'b0; t_last_busy = $time;
        wait (player_enable === 1'b1); t_pe = $time;               // 从下降处的 negedge 算，+半拍取整 = 采到的空闲沿数
        if ((t_pe - t_last_busy + 5) / 10 < DRAIN || (t_pe - t_last_busy + 5) / 10 > DRAIN + 1) begin
            $display("  release %0d cycles after the last write", (t_pe - t_last_busy + 5) / 10);
            fail("B4 release not DRAIN_CYCLES after the write port went idle");
        end
        if (boot_state !== 3'd4 || boot_error !== 8'h00) fail("B4 not in RUN / error set");

        // ---- B5 ----
        tick(2);
        scan_done = 1'b0; image_count = 7'd0; write_busy = 1'b1; tick(20);
        if (!player_enable || boot_state !== 3'd4) fail("B5 RUN left without reset");
        if (boot_error !== 8'h00) fail("B5 error raised in RUN");
        rst_n = 1'b0; #1;
        if (player_enable) fail("B5 reset did not clear player_enable immediately");

        if (errors == 0) $display("PASS tb_boot_sequencer: SDRAM timeout 0x59 + late recovery, SD not ready 0x5A, empty card 0x53 + event repeat, write-drain %0d cycles, RUN sticky, async reset", DRAIN);
        else             $display("FAIL tb_boot_sequencer: %0d errors", errors);
        $finish;
    end
    initial begin #200_000; $display("FAIL tb_boot_sequencer: timeout"); $finish; end
endmodule
