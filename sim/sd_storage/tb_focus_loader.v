`timescale 1ns/1ps
// tb_focus_loader —— 焦点表加载（focus_loader）+ KEY2+KEY4 组合键过滤（tour_chord_filter）
//   卡上是 tools/make_focus_bin.py 生成的 FOCUS.BIN（1024 字节，256 项 × 4 字节小端），FPGA 直拷；
//   文本解析规则的验证移到 tb_opt2_focus_eqv（脚本 vs 原版 RTL 逐位比对）。
//   卡模型：定位器 5 拍后给 lba/size；扇区口 busy → 每 6 拍一个字节 → done；第 2 个扇区第一次读给错误（测整份重读）
module tb_focus_loader;
    reg clk = 0, rst_n = 0;
    always #10 clk = ~clk;
    integer errors = 0;

    // ---------------- 文件内容 ----------------
    reg [7:0] file [0:4095];
    integer   fsize;
    task ent; input [7:0] a; input [11:0] x, y; input [2:0] z; begin   // 写一项（小端 4 字节）
        {file[a*4+3], file[a*4+2], file[a*4+1], file[a*4]} = {5'd0, x, y, z};
    end endtask

    // ---------------- DUT ----------------
    reg enable = 0, aux_ok = 1;
    wire loc_req; wire [5:0] loc_idx; reg loc_valid = 0;
    wire sec_req; wire [31:0] sec_lba;
    reg sec_busy = 0, sec_valid = 0, sec_done = 0; reg [7:0] sec_byte = 0, sec_error = 0;
    wire done; reg [7:0] rd_addr = 0; wire [26:0] rd_data;
    focus_loader dut (
        .clk(clk), .rst_n(rst_n), .enable(enable), .aux_ok(aux_ok),
        .loc_req(loc_req), .loc_idx(loc_idx), .loc_valid(loc_valid), .loc_lba(32'd1000), .loc_size(fsize),
        .sec_req(sec_req), .sec_lba(sec_lba), .sec_busy(sec_busy), .sec_valid(sec_valid), .sec_byte(sec_byte),
        .sec_done(sec_done), .sec_error(sec_error), .done(done), .rd_addr(rd_addr), .rd_data(rd_data));

    integer nreq = 0, err_once = 1;
    always @(posedge clk) if (loc_req) begin
        if (loc_idx !== 6'h31) begin $display("FAIL loc_idx %0d", loc_idx); errors = errors + 1; end
        repeat (5) @(posedge clk); loc_valid <= 1; @(posedge clk); loc_valid <= 0;
    end
    always @(posedge clk) if (sec_req) begin : srv
        integer b, base; reg bad;
        nreq = nreq + 1;
        base = (sec_lba - 1000) * 512;
        bad  = (sec_lba == 1001) && err_once;
        if (bad) err_once = 0;
        sec_busy <= 1;
        repeat (3) @(posedge clk);
        for (b = 0; b < 512; b = b + 1) begin
            sec_byte <= (base + b < fsize) ? file[base + b] : 8'h00; sec_valid <= 1; @(posedge clk);
            sec_valid <= 0; repeat (5) @(posedge clk);
            if (bad && b == 100) b = 512;
        end
        sec_error <= bad ? 8'h31 : 8'h00; sec_done <= 1; sec_busy <= 0; @(posedge clk);
        sec_done <= 0; sec_error <= 0;
    end

    task chk; input [7:0] a; input [11:0] x, y; input [2:0] z; begin
        rd_addr = a; @(posedge clk); @(posedge clk); #1;
        if (rd_data !== {x, y, z}) begin
            $display("FAIL tab[%0d] = %0d,%0d,%0d  expect %0d,%0d,%0d", a, rd_data[26:15], rd_data[14:3], rd_data[2:0], x, y, z);
            errors = errors + 1;
        end
    end endtask

    // ---------------- 组合键 ----------------
    reg k2 = 1, k4 = 1, c2 = 0, l2 = 0, c4 = 0, l4 = 0, ton = 0;
    wire o2c, o2l, o4c, o4l, chord, tnext;
    tour_chord_filter u_cf (.clk(clk), .rst_n(rst_n), .tour_on(ton),
        .key2_stable_n(k2), .key4_stable_n(k4), .key2_click_in(c2), .key2_long_in(l2), .key4_click_in(c4), .key4_long_in(l4),
        .key2_click_out(o2c), .key2_long_out(o2l), .key4_click_out(o4c), .key4_long_out(o4l), .chord(chord), .tour_next(tnext));
    integer n_chord = 0, n_o2c = 0, n_o2l = 0, n_o4c = 0, n_o4l = 0, n_next = 0;
    always @(posedge clk) begin
        n_chord = n_chord + chord; n_o2c = n_o2c + o2c; n_o2l = n_o2l + o2l;
        n_o4c = n_o4c + o4c; n_o4l = n_o4l + o4l; n_next = n_next + tnext;
    end
    // router：KEY2 事件晚电平 1 拍；single_key_event：KEY4 事件与电平同拍
    task k2_rel; input lng; begin k2 <= 1; @(posedge clk); if (lng) l2 <= 1; else c2 <= 1; @(posedge clk); c2 <= 0; l2 <= 0; end endtask
    task k4_rel; input lng; begin k4 <= 1; if (lng) l4 <= 1; else c4 <= 1; @(posedge clk); c4 <= 0; l4 <= 0; end endtask
    task cnt_chk; input integer a, b, c, d, e, f; begin
        repeat (3) @(posedge clk);
        if (n_chord != a || n_o2c != b || n_o2l != c || n_o4c != d || n_o4l != e || n_next != f) begin
            $display("FAIL chord cnt %0d %0d %0d %0d %0d %0d  expect %0d %0d %0d %0d %0d %0d", n_chord, n_o2c, n_o2l, n_o4c, n_o4l, n_next, a, b, c, d, e, f);
            errors = errors + 1;
        end
    end endtask

    integer i;
    initial begin
        // 与原文本用例解析结果相同的表（make_focus_bin.py 对原 tb 文本的输出）；其余字节用 0xA5 测高位被丢弃
        for (i = 0; i < 4096; i = i + 1) file[i] = 8'h00;
        fsize = 1024;
        ent({6'd0, 2'd0}, 100, 200, 3);   ent({6'd0, 2'd1}, 300, 400, 2);
        ent({6'd1, 2'd0}, 5, 6, 7);       ent({6'd1, 2'd1}, 7, 8, 6);
        ent({6'd1, 2'd2}, 9, 10, 7);      ent({6'd1, 2'd3}, 11, 12, 1);
        ent({6'd3, 2'd0}, 10, 20, 5);     ent({6'd4, 2'd0}, 1, 2, 5);
        ent({6'd63, 2'd0}, 4095, 4095, 4);
        file[{6'd4, 2'd0}*4+3] = file[{6'd4, 2'd0}*4+3] | 8'hF8;   // 未用的高 5 位
        $display("INFO file %0d bytes", fsize);

        repeat (5) @(posedge clk); rst_n = 1;
        repeat (300) @(posedge clk);
        if (done) begin $display("FAIL done before enable"); errors = errors + 1; end
        enable = 1;
        wait (done);
        $display("INFO done, %0d sector requests", nreq);
        if (nreq != 4) begin $display("FAIL expect 4 sector requests (2 + retry 2)"); errors = errors + 1; end
        chk({6'd0, 2'd0}, 100, 200, 3);
        chk({6'd0, 2'd1}, 300, 400, 2);
        chk({6'd0, 2'd2}, 0, 0, 0);
        chk({6'd1, 2'd0}, 5, 6, 7);
        chk({6'd1, 2'd1}, 7, 8, 6);
        chk({6'd1, 2'd2}, 9, 10, 7);
        chk({6'd1, 2'd3}, 11, 12, 1);
        chk({6'd2, 2'd0}, 0, 0, 0);
        chk({6'd3, 2'd0}, 10, 20, 5);
        chk({6'd3, 2'd1}, 0, 0, 0);
        chk({6'd4, 2'd0}, 1, 2, 5);
        chk({6'd63, 2'd0}, 4095, 4095, 4);
        chk({6'd63, 2'd1}, 0, 0, 0);
        chk({6'd10, 2'd0}, 0, 0, 0);
        chk({6'd63, 2'd3}, 0, 0, 0);

        // 文件大小不是 1024（误放了文本 FOCUS.TXT）→ 表为空、不读扇区
        fsize = 936; nreq = 0;
        rst_n = 0; repeat (3) @(posedge clk); rst_n = 1;
        wait (done);
        if (nreq != 0) begin $display("FAIL size != 1024 still read sectors"); errors = errors + 1; end
        chk({6'd0, 2'd0}, 0, 0, 0);
        chk({6'd63, 2'd0}, 0, 0, 0);
        fsize = 1024;

        // -------- 组合键 --------
        // 1. KEY2 单击 / KEY4 单击 / KEY2 长按 → 透传
        k2 <= 0; repeat (5) @(posedge clk); k2_rel(0);
        k4 <= 0; repeat (5) @(posedge clk); k4_rel(0);
        k2 <= 0; repeat (5) @(posedge clk); k2_rel(1);
        cnt_chk(0, 1, 1, 1, 0, 0);
        // 2. 同按：先 KEY2 后 KEY4，先放 KEY2（事件都吞掉），chord 一次
        k2 <= 0; repeat (3) @(posedge clk); k4 <= 0; repeat (10) @(posedge clk);
        k2_rel(0); repeat (4) @(posedge clk); k4_rel(1);
        cnt_chk(1, 1, 1, 1, 0, 0);
        // 3. 同按：先 KEY4 后 KEY2，先放 KEY4，再放 KEY2
        k4 <= 0; repeat (3) @(posedge clk); k2 <= 0; repeat (10) @(posedge clk);
        k4_rel(0); repeat (4) @(posedge clk); k2_rel(1);
        cnt_chk(2, 1, 1, 1, 0, 0);
        // 4. 导览中：KEY2 单击 → tour_next，KEY2 长按吞掉，KEY4 照常
        ton <= 1;
        k2 <= 0; repeat (5) @(posedge clk); k2_rel(0);
        k2 <= 0; repeat (5) @(posedge clk); k2_rel(1);
        k4 <= 0; repeat (5) @(posedge clk); k4_rel(1);
        cnt_chk(2, 1, 1, 1, 1, 1);
        // 5. 导览中同按：只出 chord
        k4 <= 0; k2 <= 0; repeat (10) @(posedge clk); k2_rel(0); k4_rel(0);
        cnt_chk(3, 1, 1, 1, 1, 1);

        if (errors == 0) $display("PASS tb_focus_loader");
        else $display("FAIL tb_focus_loader errors=%0d", errors);
        $finish;
    end
    initial begin #50_000_000; $display("FAIL timeout"); $finish; end
endmodule
