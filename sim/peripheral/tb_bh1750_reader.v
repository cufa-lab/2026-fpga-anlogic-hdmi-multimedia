`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// tb_bh1750_reader —— bh1750_reader + BH1750 IIC 从机行为模型（开漏总线 + 上拉）
//   检查：① 每个周期命令序列 = START 0x46 0x01 STOP / START 0x46 0x20 STOP / 等测量 / START 0x47 读 2 字节
//         ② 读回值逐位正确、raw_valid 单拍、sensor_ok
//         ③ 0x20 命令 STOP 到读事务 START 间隔 ≥ MEAS_MS
//         ④ SCL 周期 = 10us（100kHz）、高电平期间 SDA 只在 START/STOP 时变化（从机模型检查）
//         ⑤ 主机第 1 字节 ACK、第 2 字节 NACK
//         ⑥ 拔掉模块（地址 NACK）→ sensor_ok 清零、不出 raw_valid、总线空闲释放；插回自动恢复
//         ⑦ tick_ms 周期 1ms
// -----------------------------------------------------------------------------
module tb_bh1750_reader;
    reg clk = 0, rst_n = 0;
    always #10 clk = ~clk;                  // 50MHz

    wire scl_low, sda_low, tick_ms, raw_valid, sensor_ok;
    wire [15:0] raw;
    tri1 scl_w, sda_w;                      // 模块上 4.7K 上拉
    reg  s_low = 1'b0;
    assign scl_w = scl_low ? 1'b0 : 1'bz;
    assign sda_w = sda_low ? 1'b0 : 1'bz;
    assign sda_w = s_low   ? 1'b0 : 1'bz;

    bh1750_reader #(.CLK_HZ(50_000_000), .IIC_HZ(100_000), .DEV_ADDR(7'h23), .MEAS_MS(2), .PERIOD_MS(5)) dut (
        .clk(clk), .rst_n(rst_n), .scl_o_low(scl_low), .sda_o_low(sda_low), .sda_i(sda_w),
        .tick_ms(tick_ms), .raw_valid(raw_valid), .raw(raw), .sensor_ok(sensor_ok));

    integer errors = 0;

    // ---------------- 从机模型 ----------------
    reg        present = 1'b1;
    reg [15:0] lux_val = 16'h1234;
    reg        active = 0, ignore_fall = 0, reading = 0, addr_phase = 0, master_ack = 0;
    integer    bitpos = 0, rd_idx = 0;
    reg [7:0]  sh = 0, txb = 0;
    reg [7:0]  cmd_log [0:7];
    integer    ncmd = 0;
    integer    n_start = 0, n_stop = 0, n_rx_ack = 0, n_rx_nack = 0;
    reg        got_on = 0, got_meas = 0;
    time       t_meas_stop = 0, t_read_start = 0, t_last_scl_rise = 0;
    reg        meas_cmd_seen = 0;

    always @(negedge sda_w) if (scl_w === 1'b1) begin          // START
        active = 1; ignore_fall = 1; bitpos = 0; addr_phase = 1; reading = 0; s_low = 0; n_start = n_start + 1;
        t_read_start = $time;
    end
    always @(posedge sda_w) if (scl_w === 1'b1) begin          // STOP
        if (active) n_stop = n_stop + 1; active = 0; s_low = 0;
        if (meas_cmd_seen) begin t_meas_stop = $time; meas_cmd_seen = 0; end
    end
    always @(posedge scl_w) if (active) begin
        if (t_last_scl_rise != 0 && ($time - t_last_scl_rise) < 9900 && bitpos != 0) begin
            $display("FAIL SCL period %0t", $time - t_last_scl_rise); errors = errors + 1;
        end
        t_last_scl_rise = $time;
        if (bitpos < 8 && !reading) sh = {sh[6:0], sda_w};
        if (bitpos == 8 && reading) begin
            master_ack = (sda_w === 1'b0);
            if (master_ack) n_rx_ack = n_rx_ack + 1; else n_rx_nack = n_rx_nack + 1;
        end
    end
    always @(negedge scl_w) if (active) begin
        if (ignore_fall) begin
            ignore_fall = 0;
        end else begin
            bitpos = bitpos + 1;
            if (bitpos == 8) begin
                if (!reading) begin
                    if (addr_phase) begin
                        if (present && sh[7:1] == 7'h23) begin
                            s_low = 1;                         // ACK
                            if (sh[0]) begin
                                if (t_read_start - t_meas_stop < 2_000_000) begin
                                    $display("FAIL read %0t ns after measure cmd (< MEAS 2ms)", t_read_start - t_meas_stop); errors = errors + 1;
                                end
                                if (!got_on || !got_meas) begin
                                    $display("FAIL read before 0x01/0x20 (on=%b meas=%b)", got_on, got_meas); errors = errors + 1;
                                end
                            end
                        end else
                            s_low = 0;                         // NACK（模块不在）
                    end else begin
                        s_low = 1;
                        cmd_log[ncmd % 8] = sh; ncmd = ncmd + 1;
                        if (sh == 8'h01) got_on = 1;
                        if (sh == 8'h20) begin got_meas = got_on; meas_cmd_seen = 1; end
                    end
                end else
                    s_low = 0;                                 // 放开给主机 ACK/NACK
            end else if (bitpos == 9) begin
                bitpos = 0;
                if (!reading) begin
                    if (addr_phase && sh[0] && present && sh[7:1] == 7'h23) begin
                        reading = 1; rd_idx = 0; txb = lux_val[15:8]; s_low = ~txb[7];
                    end else
                        s_low = 0;
                    addr_phase = 0;
                end else begin
                    if (master_ack && rd_idx == 0) begin
                        rd_idx = 1; txb = lux_val[7:0]; s_low = ~txb[7];
                    end else begin
                        s_low = 0; reading = 0; got_on = 0; got_meas = 0;
                    end
                end
            end else if (reading)
                s_low = ~txb[7 - bitpos];
        end
    end

    // ---------------- 观察 ----------------
    integer n_valid = 0;
    reg [15:0] last_raw;
    always @(posedge clk) if (raw_valid) begin n_valid = n_valid + 1; last_raw = raw; end
    reg rv_q = 0;
    always @(posedge clk) begin
        rv_q <= raw_valid;
        if (raw_valid && rv_q) begin $display("FAIL raw_valid not single cycle"); errors = errors + 1; end
    end
    time t_tick = 0;
    integer n_tick = 0;
    always @(posedge clk) if (tick_ms) begin
        if (t_tick != 0 && ($time - t_tick) != 1_000_000) begin $display("FAIL tick_ms period %0t", $time - t_tick); errors = errors + 1; end
        t_tick = $time; n_tick = n_tick + 1;
    end

    task wait_valid(input [15:0] exp, input [8*24-1:0] tag);
        integer k;
        begin
            k = n_valid;
            fork : w
                begin wait (n_valid != k); disable w; end
                begin #20_000_000; disable w; end
            join
            if (n_valid == k) begin $display("FAIL %0s no raw_valid", tag); errors = errors + 1; end
            else if (last_raw !== exp || !sensor_ok) begin
                $display("FAIL %0s raw=%h exp=%h ok=%b", tag, last_raw, exp, sensor_ok); errors = errors + 1;
            end else
                $display("PASS %0s raw=%h", tag, last_raw);
        end
    endtask

    integer v0, s0, a0, n0;
    initial begin
        #200 rst_n = 1;
        wait_valid(16'h1234, "first read");
        if (cmd_log[0] !== 8'h01 || cmd_log[1] !== 8'h20) begin
            $display("FAIL cmd seq %h %h", cmd_log[0], cmd_log[1]); errors = errors + 1;
        end
        if (n_rx_ack != 1 || n_rx_nack != 1) begin
            $display("FAIL master ack/nack %0d/%0d", n_rx_ack, n_rx_nack); errors = errors + 1;
        end
        if (n_start != 3 || n_stop != 2) begin $display("FAIL start/stop %0d/%0d", n_start, n_stop); errors = errors + 1; end
        lux_val = 16'h00A5;
        wait_valid(16'h00A5, "second read");
        lux_val = 16'hFFFE;
        wait_valid(16'hFFFE, "full-scale read");

        // 拔掉模块
        present = 0;
        #12_000_000;
        v0 = n_valid;
        if (sensor_ok) begin $display("FAIL sensor_ok still 1 after unplug"); errors = errors + 1; end
        #10_000_000;
        if (n_valid != v0) begin $display("FAIL raw_valid while unplugged"); errors = errors + 1; end
        if (scl_low || sda_low) begin
            // 允许落在一次重试事务中间：等到等待段再查
            wait (dut.op == 3'd5); #100;
        end
        if (scl_low || sda_low) begin $display("FAIL bus not released when idle"); errors = errors + 1; end
        else $display("PASS unplug: sensor_ok=0, bus released");

        present = 1; lux_val = 16'h0042;
        wait_valid(16'h0042, "replug");
        if (n_tick < 30) begin $display("FAIL tick count %0d", n_tick); errors = errors + 1; end
        if (errors == 0) $display("TB_BH1750_READER PASS");
        else             $display("TB_BH1750_READER FAIL errors=%0d", errors);
        $finish;
    end
endmodule
