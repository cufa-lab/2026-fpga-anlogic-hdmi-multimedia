`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// tb_script_loader —— script_loader 单元测试
//   定位器 / 扇区口 / 人声播放器 / pix 释放握手都用行为模型；字形 RAM（8192×64，按字节写、块 6/7 暂存）、
//   去重表（4096×8）是与 script_glyph_bram32k.v 同语义的行为模型，code / line 写口收进 tb 数组比对。
//   期望值由 sim/k1/gen_loader.py（k1lib 参考模型）生成：t_exp1.hex / t_exp4.hex = line[512] + code[512] + slot fidx[256]。
//   LRC 扇区字节间隔 32 拍（与硬件相同），字库扇区 8 拍（只做按字节直写）。
//   用例：
//     1. 上电定位 HZK32（项 0x30）；TRACK1：ti / ar / 未知标签 / 4 种时间格式 / 空行 / 制表符 / 行尾挤出标点 /
//        截断（0x76）/ hash 冲突的两个字；行表（含段落行）、code、表头、每槽 32 字字形全比
//     2. 人声：定位 VOC → play_enable + epoch；播完 wr_done → 自动重播 + 第二次 epoch
//     3. TRACK2（无 LRC）：表头 valid=1 / line_end=2，标题导语宽度清 0，人声停止
//     4. TRACK3（UTF-8 BOM）：0x71，表头 valid=0，不取字形
//     5. TRACK16（参考模型的第 4 份稿，放在第 16 首验证 16 曲编码）：306 个不同字 → 槽满 0x72，结果与参考模型一致
//     5b. select = 5'h1F（第 17 张起，无曲目位 = 1，低 4 位 = 15 且 LRC16 存在）：同无 LRC，表头 valid=1，人声不启动
//     6. 回到 TRACK1：第一个 LRC 扇区报错（整份重来）+ 第 3 个字库扇区报错（重读）；去重表残留着
//        TRACK4 的内容，结果必须与用例 1 完全相同
// -----------------------------------------------------------------------------
`include "sim/k1/t_meta.vh"
module tb_script_loader;
    reg clk = 1'b0;
    always #10 clk = ~clk;
    reg rst_n = 1'b0;

    reg  [4:0] select = 5'd0;
    wire [33:0] aux_valid = (34'd1 << 32) | (34'd1 << 16) | (34'd1 << 15) | (34'd1 << 2) | 34'd1;   // HZK32 VOC1 LRC16 LRC3 LRC1

    // ---------------- DUT ----------------
    wire        loc_req;
    wire [5:0]  loc_idx;
    reg         loc_valid = 1'b0;
    reg  [31:0] loc_lba = 0, loc_size = 0;
    wire        sec_req;
    wire [31:0] sec_lba;
    reg         sec_busy = 1'b0, sec_valid = 1'b0, sec_done = 1'b0;
    reg  [7:0]  sec_byte = 0, sec_error = 0;
    reg  [8:0]  sec_byte_index = 0;
    wire        voc_play_enable;
    wire [31:0] voc_start_lba, voc_total_sectors;
    reg         voc_wr_done = 1'b0;
    wire        glyph_we, glyph_park, glyph_re, code_we, line_we;
    wire [2:0]  glyph_blk;
    wire [12:0] glyph_addr;
    wire [15:0] glyph_data;
    reg  [15:0] glyph_q = 16'd0;
    wire        dd_we, dd_re;
    wire [11:0] dd_addr;
    wire [7:0]  dd_data;
    reg  [7:0]  dd_q = 8'd0;
    wire [8:0]  code_addr, line_addr;
    wire [9:0]  code_data;
    wire [17:0] line_data;
    wire        rel_req_t, ready_t, epoch_t;
    reg         rel_ack_t = 1'b0;
    wire [7:0]  script_error;

    script_loader #(.MAX_W(72), .ROW_W(72)) dut (
        .clk(clk), .rst_n(rst_n), .enable(1'b1), .aux_valid(aux_valid), .select(select),
        .loc_req(loc_req), .loc_idx(loc_idx), .loc_valid(loc_valid), .loc_lba(loc_lba), .loc_size(loc_size),
        .sec_req(sec_req), .sec_lba(sec_lba), .sec_busy(sec_busy), .sec_valid(sec_valid), .sec_byte(sec_byte),
        .sec_byte_index(sec_byte_index), .sec_done(sec_done), .sec_error(sec_error),
        .voc_sd_req(1'b0), .voc_sd_lba(32'd0), .voc_sd_busy(), .voc_sd_valid(), .voc_sd_done(),
        .voc_play_enable(voc_play_enable), .voc_start_lba(voc_start_lba), .voc_total_sectors(voc_total_sectors),
        .voc_wr_idle(!voc_play_enable), .voc_wr_done(voc_wr_done), .voc_wr_error(1'b0),
        .glyph_we(glyph_we), .glyph_park(glyph_park), .glyph_blk(glyph_blk), .glyph_addr(glyph_addr),
        .glyph_data(glyph_data), .glyph_re(glyph_re), .glyph_q(glyph_q),
        .dd_we(dd_we), .dd_re(dd_re), .dd_addr(dd_addr), .dd_data(dd_data), .dd_q(dd_q),
        .code_we(code_we), .code_addr(code_addr), .code_data(code_data),
        .line_we(line_we), .line_addr(line_addr), .line_data(line_data),
        .rel_req_t(rel_req_t), .rel_ack_t(rel_ack_t), .ready_t(ready_t), .epoch_t(epoch_t),
        .script_error(script_error)
    );

    // ---------------- 内容 ----------------
    reg [7:0]  lrc1 [0:4095];
    reg [7:0]  lrc4 [0:4095];
    reg [17:0] exp1 [0:1279];
    reg [17:0] exp4 [0:1279];
    // 字库：第 o 字节的内容（随 o 的各段变化，不同字 / 不同字节都不同）
    function [7:0] hzk_byte; input [31:0] o; hzk_byte = o[7:0] * 8'd7 + o[15:8] + o[20:16] * 8'd13 + 8'h3C; endfunction

    localparam [31:0] LBA_HZK = 10000, LBA_LRC1 = 100, LBA_LRC3 = 200, LBA_LRC4 = 300, LBA_VOC = 3000;
    localparam [31:0] HZK_SIZE = 32'd2093568;

    // ---------------- 定位器模型 ----------------
    always @(posedge clk) begin
        loc_valid <= 1'b0;
        if (loc_req) begin
            repeat (12) @(posedge clk);
            case (loc_idx)
                6'h30: begin loc_lba <= LBA_HZK;  loc_size <= HZK_SIZE;  end
                6'h10: begin loc_lba <= LBA_LRC1; loc_size <= `T1_LEN;   end
                6'h12: begin loc_lba <= LBA_LRC3; loc_size <= 10;        end
                6'h1F: begin loc_lba <= LBA_LRC4; loc_size <= `T4_LEN;   end   // TRACK16.LRC
                6'h20: begin loc_lba <= LBA_VOC;  loc_size <= 512*20 + 100; end
                default: begin $display("FAIL unexpected loc_idx %0d", loc_idx); $finish; end
            endcase
            loc_valid <= 1'b1;
        end
    end

    // ---------------- 扇区模型 ----------------
    integer bi, nsec_rd = 0, nhzk_rd = 0, gap;
    reg     inject_err = 1'b0;               // 下一个扇区报错
    integer inject_hzk = -1;                 // 第 N 个字库扇区报错（-1 = 不注入）
    reg [31:0] cur, o;
    function [7:0] sbyte; input [31:0] l; input [8:0] i; reg [31:0] q; begin
        if (l >= LBA_HZK && l < LBA_HZK + 4089) begin q = (l - LBA_HZK) * 512 + i; sbyte = hzk_byte(q); end
        else if (l >= LBA_LRC1 && l < LBA_LRC1 + 8) begin
            q = (l - LBA_LRC1) * 512 + i; sbyte = (q < `T1_LEN) ? lrc1[q] : 8'hEE;   // 文件尾之后是垃圾
        end else if (l >= LBA_LRC4 && l < LBA_LRC4 + 8) begin
            q = (l - LBA_LRC4) * 512 + i; sbyte = (q < `T4_LEN) ? lrc4[q] : 8'hEE;
        end else if (l == LBA_LRC3) sbyte = (i == 0) ? 8'hEF : (i == 1) ? 8'hBB : (i == 2) ? 8'hBF : 8'h41;
        else sbyte = 8'h00;
    end endfunction
    reg err_now;
    always @(posedge clk) begin
        if (sec_req) begin
            if (sec_busy) begin $display("FAIL req while busy"); $finish; end
            cur = sec_lba; sec_busy <= 1'b1;
            nsec_rd = nsec_rd + 1;
            err_now = inject_err;
            if (cur >= LBA_HZK) begin
                if (nhzk_rd == inject_hzk) err_now = 1'b1;
                nhzk_rd = nhzk_rd + 1;
            end
            gap = (cur >= LBA_HZK) ? 8 : 32;
            repeat (20) @(posedge clk);
            for (bi = 0; bi < 512; bi = bi + 1) begin
                sec_valid <= 1'b1; sec_byte <= sbyte(cur, bi[8:0]); sec_byte_index <= bi[8:0];
                @(posedge clk); sec_valid <= 1'b0;
                repeat (gap - 1) @(posedge clk);
            end
            repeat (4) @(posedge clk);
            sec_done <= 1'b1; sec_error <= err_now ? 8'h10 : 8'h00; inject_err <= 1'b0;
            @(posedge clk); sec_done <= 1'b0; sec_error <= 8'h00; sec_busy <= 1'b0;
        end
    end

    // ---------------- pix 释放握手模型 ----------------
    always @(rel_req_t) begin
        repeat (200) @(posedge clk);
        rel_ack_t <= rel_req_t;
    end

    // ---------------- RAM 模型 ----------------
    reg [63:0] gmem [0:8191];
    reg [7:0]  dmem [0:4095];
    reg [9:0]  cmem [0:511];
    reg [17:0] lmem [0:511];
    integer k, r, j;
    initial for (k = 0; k < 4096; k = k + 1) dmem[k] = 8'd0;
    always @(posedge clk) begin
        if (glyph_we) begin
            if (glyph_park) begin
                gmem[glyph_addr][15:8] <= glyph_data[15:8];
                gmem[glyph_addr][7:0]  <= glyph_data[7:0];
            end else
                gmem[glyph_addr][63 - 8 * glyph_blk -: 8] <= glyph_blk[0] ? glyph_data[7:0] : glyph_data[15:8];
        end
        if (glyph_re) glyph_q <= gmem[glyph_addr][15:0];
        if (dd_we) dmem[dd_addr] <= dd_data;
        if (dd_re) dd_q <= dmem[dd_addr];
        if (code_we) cmem[code_addr] <= code_data;
        if (line_we) lmem[line_addr] <= line_data;
    end

    integer errors = 0;
    task check; input cond; input [8*64-1:0] msg; begin
        if (!cond) begin errors = errors + 1; $display("FAIL %0s", msg); end
    end endtask

    task wait_ready; reg r0; integer to; begin
        r0 = ready_t; to = 0;
        while (ready_t == r0 && to < 20000000) begin @(posedge clk); to = to + 1; end
        if (to >= 20000000) begin $display("FAIL ready timeout state=%0d sel_q=%0d rel_done=%0d", dut.state, dut.sel_q, dut.rel_done); $display("  t=%0t nsec=%0d nhzk=%0d hz_k=%0d hz_n=%0d busy=%0d own=%0d", $time, nsec_rd, nhzk_rd, dut.hz_k, dut.hz_n, sec_busy, dut.own_ld); $finish; end
        repeat (5) @(posedge clk);
    end endtask

    // 对照参考模型：行表（条 0/1 全部、时间行 w0..w2、段落行 w0..w2）、code、字形
    integer nbad;
    reg [17:0] ev;
    reg [63:0] gw;
    reg [12:0] fx;
    task cmp_line; input integer a; input [17:0] e; begin
        if (lmem[a] !== e) begin
            nbad = nbad + 1;
            if (nbad < 10) $display("FAIL line[%0d] (entry %0d w%0d) got %h exp %h", a, a / 4, a % 4, lmem[a], e);
        end
    end endtask
    function [17:0] exp_w; input integer t; input integer a; exp_w = (t == 1) ? exp1[a] : exp4[a]; endfunction
    task check_track; input integer t; input integer lines; input integer rows; input integer cells;
        input integer slots; input [7:0] err; begin
        nbad = 0;
        for (k = 0; k < 512; k = k + 1) begin
            ev = (t == 1) ? exp1[k] : exp4[k];
            if ((k < 8 && k != 7 && !(k == 1 && exp_w(t, 2) == 0) && !(k == 5 && exp_w(t, 6) == 0)) || ((k / 4) >= 2 && (k / 4) < lines && (k % 4) != 3) ||
                ((k / 4) >= 112 && (k / 4) < 112 + rows && (k % 4) != 3))
                cmp_line(k, ev);
        end
        for (k = 0; k < cells; k = k + 1) begin
            ev = (t == 1) ? exp1[512 + k] : exp4[512 + k];
            if (cmem[k] !== ev[9:0]) begin
                nbad = nbad + 1;
                if (nbad < 10) $display("FAIL code[%0d] got %h exp %h", k, cmem[k], ev[9:0]);
            end
        end
        for (k = 0; k < slots; k = k + 1) begin
            ev = (t == 1) ? exp1[1024 + k] : exp4[1024 + k];
            fx = ev[12:0];
            for (r = 0; r < 32; r = r + 1) begin
                for (j = 0; j < 8; j = j + 1) begin
                    o = {fx[12:1], 9'd0} + {fx[0], 8'd0} + r * 8 + j;
                    gw[63 - 8 * j -: 8] = hzk_byte(o);
                end
                if (gmem[k * 32 + r] !== gw) begin
                    nbad = nbad + 1;
                    if (nbad < 10) $display("FAIL glyph slot%0d (fidx %0d) word%0d got %h exp %h", k, fx, r, gmem[k * 32 + r], gw);
                end
            end
        end
        if (script_error != err) begin nbad = nbad + 1; $display("FAIL track%0d error %h exp %h", t, script_error, err); end
        errors = errors + nbad;
        $display("INFO track%0d: %0d cells %0d slots %0d rows, error %h, %0d mismatches", t, cells, slots, rows, script_error, nbad);
    end endtask

    integer e0, ep, n0;
    initial begin
        $readmemh("sim/k1/t_lrc.hex",  lrc1);
        $readmemh("sim/k1/t_lrc4.hex", lrc4);
        $readmemh("sim/k1/t_exp1.hex", exp1);
        $readmemh("sim/k1/t_exp4.hex", exp4);
        repeat (5) @(posedge clk);
        rst_n = 1'b1;

        // ---- 1：开机 → TRACK1 ----
        e0 = epoch_t;
        wait_ready;
        check(dut.hzk_lba == LBA_HZK, "hzk lba");
        check_track(1, `T1_LINES, `T1_ROWS, `T1_CELLS, `T1_SLOTS, `T1_ERR);
        $display("INFO track1 loaded, sectors read %0d", nsec_rd);

        // ---- 2：人声 ----
        wait (voc_play_enable);
        check(voc_start_lba == LBA_VOC && voc_total_sectors == 20, "voc lba/sectors");
        check(epoch_t != e0, "epoch toggled on voice start");
        ep = epoch_t;
        repeat (500) @(posedge clk);
        voc_wr_done <= 1'b1;
        wait (!voc_play_enable);
        voc_wr_done <= 1'b0;
        wait (voc_play_enable);
        check(epoch_t != ep, "epoch toggled on voice loop");

        // ---- 3：TRACK2（无 LRC）----
        select = 5'd1;
        wait_ready;
        check(lmem[{7'd0, 2'd3}] == {3'd0, 5'd0, 1'b0, 1'b1, 8'd2}, "header track2");
        check(lmem[{7'd0, 2'd2}] == 0 && lmem[{7'd1, 2'd2}] == 0, "track2 title/intro cleared");
        check(lmem[{7'd0, 2'd0}] == 0, "track2 gend 0");
        repeat (100) @(posedge clk);
        check(!voc_play_enable, "track2 no voice");
        check(script_error == 8'h00, "track2 no error");

        // ---- 4：TRACK3（UTF-8 BOM）----
        n0 = nhzk_rd;
        select = 5'd2;
        wait_ready;
        check(script_error == 8'h71, "track3 0x71");
        check(lmem[{7'd0, 2'd3}][8] == 1'b0, "track3 header invalid");
        check(nhzk_rd == n0, "track3 no glyph reads");

        // ---- 5：TRACK16（槽满）----
        select = 5'd15;
        wait_ready;
        check_track(4, `T4_LINES, `T4_ROWS, `T4_CELLS, `T4_SLOTS, `T4_ERR);

        // ---- 5b：第 17 张起（无曲目位；低 4 位 15 = TRACK16 存在，必须仍不加载）----
        n0 = nhzk_rd;
        select = 5'h1F;
        wait_ready;
        check(lmem[{7'd0, 2'd3}] == {3'd0, 5'd0, 1'b0, 1'b1, 8'd2}, "header no-track");
        repeat (100) @(posedge clk);
        check(!voc_play_enable && script_error == 8'h00 && nhzk_rd == n0, "no-track: no voice / error / glyph reads");

        // ---- 6：回到 TRACK1，LRC 第 0 扇区 + 第 3 个字库扇区报错；去重表里是 TRACK4 的残留 ----
        inject_err = 1'b1;
        inject_hzk = nhzk_rd + 2;
        select = 5'd0;
        wait_ready;
        check(nhzk_rd == inject_hzk + 1 + `T1_SLOTS - 2, "hzk sector re-read once");
        check_track(1, `T1_LINES, `T1_ROWS, `T1_CELLS, `T1_SLOTS, `T1_ERR);

        if (errors == 0) $display("PASS tb_script_loader");
        else             $display("FAIL tb_script_loader errors=%0d", errors);
        $finish;
    end

    initial begin #2_000_000_000; $display("FAIL global timeout"); $finish; end
endmodule
