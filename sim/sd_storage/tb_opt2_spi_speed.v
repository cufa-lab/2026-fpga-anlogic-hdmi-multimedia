`timescale 1ns/1ps
// 单扇区读耗时测量（sd_card_top + sd_card_fat32_model，sys_clk 50 MHz），连续读 8 个扇区取平均，并核对数据
module tb_opt2_spi_speed;
    reg clk = 0, rst_n = 0;
    always #10 clk = ~clk;
    wire nCS, DCLK, MOSI, MISO;
    wire init_done, dvalid, rend;
    wire [7:0] dat;
    reg  rd = 0;
    reg  [31:0] lba = 0;
    sd_card_top dut (.clk(clk), .rst_n(rst_n), .SD_nCS(nCS), .SD_DCLK(DCLK), .SD_MOSI(MOSI), .SD_MISO(MISO),
        .sd_init_done(init_done), .sd_sec_read(rd), .sd_sec_read_addr(lba), .sd_sec_read_data(dat),
        .sd_sec_read_data_valid(dvalid), .sd_sec_read_end(rend),
        .sd_sec_write(1'b0), .sd_sec_write_addr(32'd0), .sd_sec_write_data(8'd0), .sd_sec_write_data_req(), .sd_sec_write_end());
    sd_card_fat32_model card (.nCS(nCS), .DCLK(DCLK), .MOSI(MOSI), .MISO(MISO));
    integer k, nb, err = 0, nedge = 0;
    always @(posedge DCLK) if (rd) nedge = nedge + 1;
    realtime t0, t1;
    always @(posedge clk) if (dvalid) begin
        if (dat !== card.card_byte(lba, nb)) begin err = err + 1; if (err < 5) $display("ERR lba %0d byte %0d got %h", lba, nb, dat); end
        nb = nb + 1;
    end
    initial begin
        repeat (5) @(posedge clk); rst_n = 1;
        wait (init_done); repeat (10) @(posedge clk);
        t0 = $realtime;
        for (k = 0; k < 8; k = k + 1) begin
            lba = k; nb = 0;
            @(negedge clk); rd = 1;
            @(posedge rend); @(negedge clk); rd = 0;
            if (nb != 512) begin err = err + 1; $display("ERR lba %0d bytes %0d", k, nb); end
            repeat (4) @(posedge clk);
        end
        t1 = $realtime;
        $display("INFO avg sector read %0.1f us (8 sectors), DCLK/sector %0d, byte errors %0d", (t1 - t0) / 8000.0, nedge / 8, err);
        if (err == 0) $display("PASS tb_opt2_spi_speed"); else $display("FAIL tb_opt2_spi_speed");
        $finish;
    end
    initial begin #200_000_000; $display("FAIL timeout"); $finish; end
endmodule
