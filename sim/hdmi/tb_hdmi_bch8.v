`timescale 1ns/1ps

module tb_hdmi_bch8;
    `include "tb_hdmi_common.vh"

    reg  [23:0] hdr;
    reg  [55:0] sb;
    wire [7:0]  hdr_ecc, sb_ecc;
    hdmi_bch8 #(.DATA_BYTES(3)) u_hdr (.data(hdr), .ecc(hdr_ecc));
    hdmi_bch8 #(.DATA_BYTES(7)) u_sb  (.data(sb),  .ecc(sb_ecc));

    integer errors = 0;
    integer i;

    task check_hdr(input [23:0] d, input [7:0] golden);
        begin
            hdr = d; #1;
            if (hdr_ecc !== golden) begin
                errors = errors + 1;
                $display("FAIL golden header %h: got %h expect %h", d, hdr_ecc, golden);
            end
        end
    endtask

    task check_sb(input [55:0] d, input [7:0] golden);
        begin
            sb = d; #1;
            if (sb_ecc !== golden) begin
                errors = errors + 1;
                $display("FAIL golden subpacket %h: got %h expect %h", d, sb_ecc, golden);
            end
        end
    endtask

    initial begin
        // header 以 {HB2,HB1,HB0} 表示；ECC 取自官方 IP 数据岛解码
        check_hdr(24'h0a0184, 8'h4a);   // Audio InfoFrame
        check_hdr(24'h190183, 8'h64);   // SPD InfoFrame
        check_hdr(24'h000102, 8'h65);   // Audio Sample, 1 sample
        check_sb (56'h00000000000170, 8'h7d);
        check_sb (56'h006c65746e49cb, 8'hfd);
        check_sb (56'h88fdb97502468a, 8'h27);
        check_sb (56'h88fb72eb048d14, 8'hd1);
        check_sb (56'h88fa4fa605b059, 8'h9a);
        check_sb (56'h00f8091c07f6e3, 8'h05);
        check_sb (56'h88eeeef411110b, 8'h1d);

        for (i = 0; i < 20000; i = i + 1) begin
            hdr = $random; sb = {$random, $random}; #1;
            if (hdr_ecc !== ref_bch8({32'd0, hdr}, 24)) errors = errors + 1;
            if (sb_ecc !== ref_bch8(sb, 56)) errors = errors + 1;
        end

        if (errors == 0) $display("PASS tb_hdmi_bch8: 10 golden vectors + 40000 random vectors");
        else             $display("FAIL tb_hdmi_bch8: %0d errors", errors);
        $finish;
    end
endmodule
