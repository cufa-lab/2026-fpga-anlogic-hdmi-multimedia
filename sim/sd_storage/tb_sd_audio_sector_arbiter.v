`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// tb_sd_audio_sector_arbiter —— 扇区读口二选一（A = FAT32 客户端，B = 音频）
//   [R1] 单口请求：req 单拍 → s_req 单拍、s_lba = 该口 lba，valid/done 只回给该口
//   [R2] 同时请求 / 连续请求：按扇区轮转（A,B 都在等时交替），任何一口不会连续拿两次而另一口在等
//   [R3] 协议：s_req 只在 s_busy=0 且无在途时发出；busy = 挂起或在途
//   [R4] b_enable=0：B 只锁存不放行；A 照常；b_enable 恢复后放行
//   [R5] flush：B 在途 → b_done + b_flushed；B 挂起（未放行）→ 保留，flush 后照常读；A 挂起/在途清除
//   随机压力：两口随机发请求，模型校验每个请求恰好一个 done、lba 不串
// -----------------------------------------------------------------------------
module tb_sd_audio_sector_arbiter;
    reg clk = 0, rst_n = 0;
    always #10 clk = ~clk;

    reg flush = 0, b_enable = 1;
    reg a_req = 0, b_req = 0;
    reg [31:0] a_lba = 0, b_lba = 0;
    wire a_busy, a_valid, a_done, b_busy, b_valid, b_done, b_flushed, s_req;
    wire [31:0] s_lba;
    reg s_busy = 0, s_valid = 0, s_done = 0;

    sd_audio_sector_arbiter dut (
        .clk(clk), .rst_n(rst_n), .flush(flush), .b_enable(b_enable),
        .a_req(a_req), .a_lba(a_lba), .a_busy(a_busy), .a_valid(a_valid), .a_done(a_done),
        .b_req(b_req), .b_lba(b_lba), .b_busy(b_busy), .b_valid(b_valid), .b_done(b_done), .b_flushed(b_flushed),
        .s_req(s_req), .s_lba(s_lba), .s_busy(s_busy), .s_valid(s_valid), .s_done(s_done)
    );

    integer errors = 0;
    task fail(input [8*64-1:0] m); begin errors = errors + 1; $display("FAIL @%0t: %0s", $time, m); end endtask

    // 适配层模型：s_req → busy，LAT 拍后 4 个 valid，done
    integer lat = 8, n_s = 0;
    reg [31:0] s_log [0:1023];
    reg [31:0] cur;
    reg kill = 0;                          // flush 时适配层被复位
    always @(posedge clk) if (s_req && s_busy) fail("[R3] s_req while s_busy");
    initial forever begin : adp
        integer i;
        @(posedge clk);
        if (s_req) begin
            cur = s_lba; if (n_s < 1024) s_log[n_s] = s_lba; n_s = n_s + 1;
            #1 s_busy = 1;
            for (i = 0; i < lat && !kill; i = i + 1) @(posedge clk);
            for (i = 0; i < 4 && !kill; i = i + 1) begin
                if (s_lba !== cur) fail("[R1] s_lba changed in flight");
                #1 s_valid = 1; @(posedge clk); #1 s_valid = 0;
            end
            if (!kill) begin #1 s_done = 1; @(posedge clk); end
            #1 s_done = 0; s_busy = 0;
        end
    end

    // 客户端计数
    integer a_dn = 0, b_dn = 0, b_fl = 0, a_v = 0, b_v = 0;
    always @(posedge clk) begin
        if (a_done) a_dn = a_dn + 1;
        if (b_done) b_dn = b_dn + 1;
        if (b_flushed) begin b_fl = b_fl + 1; if (!b_done) fail("[R5] b_flushed without b_done"); end
        if (a_valid) a_v = a_v + 1;
        if (b_valid) b_v = b_v + 1;
        if (a_valid && b_valid) fail("[R1] valid to both");
        if (a_done && b_done) fail("[R1] done to both");
    end

    task req_a(input [31:0] l); begin @(negedge clk); a_lba = l; a_req = 1; @(negedge clk); a_req = 0; end endtask
    task req_b(input [31:0] l); begin @(negedge clk); b_lba = l; b_req = 1; @(negedge clk); b_req = 0; end endtask
    task wait_idle; begin @(negedge clk); while (a_busy || b_busy || s_busy) @(negedge clk); repeat (3) @(negedge clk); end endtask

    integer k, ka, kb, n0, a_out, b_out, a_cnt, b_cnt;
    initial begin
        repeat (3) @(negedge clk); rst_n = 1;

        // [R1] 单口
        req_a(32'd100); if (!a_busy) fail("[R3] a_busy after req"); wait_idle;
        if (n_s != 1 || s_log[0] != 100 || a_dn != 1 || a_v != 4 || b_dn != 0 || b_v != 0) fail("[R1] A single");
        req_b(32'd200); wait_idle;
        if (n_s != 2 || s_log[1] != 200 || b_dn != 1 || b_v != 4 || a_dn != 1) fail("[R1] B single");
        $display("INFO [R1] single requests routed, valid/done only to owner");

        // [R2] 同时请求 + 背靠背：A 连续读 6 扇区（如图片加载），B 每次 done 后立即再请求
        n0 = n_s;
        fork
            begin for (ka = 0; ka < 6; ka = ka + 1) begin req_a(32'd1000 + ka); @(negedge clk); while (a_busy) @(negedge clk); end end
            begin for (kb = 0; kb < 6; kb = kb + 1) begin req_b(32'd2000 + kb); @(negedge clk); while (b_busy) @(negedge clk); end end
        join
        wait_idle;
        // 两口都在等时必须交替：不出现 A A（B 在等）这种连续
        a_cnt = 0; b_cnt = 0;
        for (k = n0; k < n_s; k = k + 1) begin
            if (s_log[k] >= 1000 && s_log[k] < 1006) begin if (s_log[k] != 1000 + a_cnt) fail("[R2] A order"); a_cnt = a_cnt + 1; end
            else if (s_log[k] >= 2000 && s_log[k] < 2006) begin if (s_log[k] != 2000 + b_cnt) fail("[R2] B order"); b_cnt = b_cnt + 1; end
            else fail("[R2] unknown lba");
            if (k > n0 && k < n_s - 1 && ((s_log[k] < 2000) == (s_log[k-1] < 2000))) fail("[R2] same port twice while other waiting");
        end
        if (a_cnt != 6 || b_cnt != 6) fail("[R2] count");
        $display("INFO [R2] 6+6 back-to-back sectors interleaved: %0d %0d %0d %0d %0d %0d ...",
                 s_log[n0], s_log[n0+1], s_log[n0+2], s_log[n0+3], s_log[n0+4], s_log[n0+5]);

        // [R4] b_enable=0
        b_enable = 0; n0 = n_s; b_dn = 0;
        req_b(32'd300);
        repeat (50) @(negedge clk);
        if (n_s != n0 || !b_busy) fail("[R4] B released while disabled");
        req_a(32'd301); @(negedge clk); while (a_busy) @(negedge clk);
        if (n_s != n0 + 1 || s_log[n0] != 301) fail("[R4] A blocked by disabled B");
        b_enable = 1; wait_idle;
        if (n_s != n0 + 2 || s_log[n0+1] != 300 || b_dn != 1) fail("[R4] B not released after enable");
        $display("INFO [R4] b_enable=0 latches B only, A unaffected, released on enable");

        // [R5a] flush 时 B 在途 → b_done + b_flushed
        b_fl = 0; b_dn = 0; lat = 40;
        req_b(32'd400); repeat (10) @(negedge clk);
        if (!s_busy) fail("[R5] B not in flight");
        kill = 1; flush = 1; @(negedge clk); flush = 0;
        repeat (3) @(negedge clk); kill = 0;
        wait_idle;
        if (b_fl != 1 || b_dn != 1 || b_busy) fail("[R5] in-flight B not flushed with done");
        // [R5b] flush 时 B 挂起未放行 → 保留，之后照常读
        b_enable = 0; n0 = n_s; b_fl = 0; b_dn = 0;
        req_b(32'd500); req_a(32'd501);           // A 在途
        repeat (5) @(negedge clk);
        kill = 1; flush = 1; @(negedge clk); flush = 0; repeat (3) @(negedge clk); kill = 0;
        if (a_busy) fail("[R5] A not cleared by flush");
        if (!b_busy || b_fl != 0) fail("[R5] pending B dropped by flush");
        b_enable = 1; wait_idle;
        if (b_dn != 1 || b_fl != 0 || s_log[n_s-1] != 500) fail("[R5] pending B not served after flush");
        $display("INFO [R5] flush: in-flight B -> done+flushed; pending B kept and served; A cleared");
        lat = 8;

        // 随机压力
        a_out = 0; b_out = 0; a_dn = 0; b_dn = 0; n0 = n_s;
        for (k = 0; k < 3000; k = k + 1) begin
            @(negedge clk);
            a_req = (!a_busy && !a_req && ($random % 5 == 0)); if (a_req) begin a_lba = 32'h0A000000 + k; a_out = a_out + 1; end
            b_req = (!b_busy && !b_req && ($random % 7 == 0)); if (b_req) begin b_lba = 32'h0B000000 + k; b_out = b_out + 1; end
            lat = 2 + ($random & 15);
        end
        @(negedge clk) begin a_req = 0; b_req = 0; end
        wait_idle;
        if (a_dn != a_out || b_dn != b_out) begin fail("[stress] done count"); $display("     a %0d/%0d b %0d/%0d", a_dn, a_out, b_dn, b_out); end
        $display("INFO [stress] %0d A + %0d B random requests, each exactly one done", a_out, b_out);

        if (errors == 0) $display("PASS tb_sd_audio_sector_arbiter");
        else             $display("FAIL tb_sd_audio_sector_arbiter: %0d errors", errors);
        $finish;
    end
endmodule
