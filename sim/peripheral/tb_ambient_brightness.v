`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// tb_ambient_brightness —— 环境光自适应亮度：显示亮度 = 限幅(base + offset)
//   ① 传感器不在：base = 255，512 个偏移逐个检查 eff == 限幅(255 + offset)（= 原工程 8 位亮度，逐位相同）
//   ② 一组照度：base 收敛到参考模型（整数精确 1/4 倍程）；每档多个偏移检查 eff 限幅加法
//   ③ 斜坡：每步 ±1、步间隔 ≥ RAMP_MS 个 tick
//   ④ 迟滞：目标差 ≤ HYST 不动、不发 env_change
//   ⑤ env_change：上电第一次读数不发；之后每次新环境（斜坡启动）恰好 1 个单拍；掉线不发；重新插上第一次读数又不发
//   ⑥ 随机 base × offset 组合（含溢出两端）
// tick_ms 在 tb 里用 16 拍一次加速（模块只数 tick，不关心真实时间）。
// -----------------------------------------------------------------------------
module tb_ambient_brightness;
    reg clk = 0, rst_n = 0;
    always #10 clk = ~clk;
    reg tick = 0;
    integer tdiv = 0;
    always @(posedge clk) begin
        tdiv <= (tdiv == 15) ? 0 : tdiv + 1;
        tick <= (tdiv == 15);
    end

    reg        ok = 0, rv = 0;
    reg [15:0] raw = 0;
    reg [8:0]  off = 9'd0;
    wire [7:0] base, eff;
    wire       envc;
    ambient_brightness #(.MIN_BASE(64), .L_LO(12), .HYST(8), .RAMP_MS(8)) dut (
        .clk(clk), .rst_n(rst_n), .tick_ms(tick), .sensor_ok(ok), .raw_valid(rv), .raw(raw),
        .offset(off), .base(base), .eff_brightness(eff), .env_change(envc));

    integer errors = 0;

    function integer clamp_add(input integer b, input [8:0] o);
        integer s;
        begin
            s = b + $signed(o);
            clamp_add = (s < 0) ? 0 : ((s > 255) ? 255 : s);
        end
    endfunction
    function integer ref_L(input integer r);
        integer m;
        begin
            if (r <= 1) ref_L = 0;
            else begin
                m = 0;
                while ((r >> (m + 1)) != 0) m = m + 1;
                if (m == 1) ref_L = 4 + ((r & 1) << 1);
                else        ref_L = 4 * m + ((r >> (m - 2)) & 3);
            end
        end
    endfunction
    function integer ref_base(input integer r);
        integer l, g;
        begin
            l = ref_L(r);
            g = (l > 12) ? 64 + (l - 12) * 7 : 64;
            ref_base = (g > 255) ? 255 : g;
        end
    endfunction

    integer n_env = 0;
    reg envc_q = 0;
    always @(posedge clk) begin
        envc_q <= envc;
        if (envc) n_env = n_env + 1;
        if (envc && envc_q) begin $display("FAIL env_change not single cycle"); errors = errors + 1; end
    end

    task send(input [15:0] r);
        begin
            @(posedge clk) begin raw <= r; rv <= 1; end
            @(posedge clk) rv <= 0;
        end
    endtask
    task settle;
        begin repeat (16 * 8 * 260) @(posedge clk); end
    endtask
    task check_offsets(input [8*24-1:0] tag);
        integer k, e;
        reg [8:0] o;
        begin
            for (k = -256; k < 256; k = k + 37) begin
                o = k; off = o;
                repeat (3) @(posedge clk);
                e = clamp_add(base, o);
                if (eff !== e[7:0]) begin $display("FAIL %0s base=%0d off=%0d eff=%0d exp=%0d", tag, base, k, eff, e); errors = errors + 1; end
            end
            off = 9'd0;
        end
    endtask

    // 斜坡监视
    reg [7:0] base_q = 8'd255;
    integer   ticks_since = 1000;
    always @(posedge clk) begin
        if (tick) ticks_since = ticks_since + 1;
        if (rst_n && base !== base_q) begin
            if (base != base_q + 8'd1 && base != base_q - 8'd1) begin $display("FAIL ramp jump %0d -> %0d", base_q, base); errors = errors + 1; end
            if (ticks_since < 8) begin $display("FAIL ramp too fast (%0d ticks)", ticks_since); errors = errors + 1; end
            ticks_since = 0;
        end
        base_q <= base;
    end

    integer k, r, e, b0, n0;
    integer raws [0:9];
    reg [8:0] o;
    initial begin
        raws[0] = 0; raws[1] = 5; raws[2] = 9; raws[3] = 30; raws[4] = 120;
        raws[5] = 360; raws[6] = 700; raws[7] = 1200; raws[8] = 54612; raws[9] = 20;
        #100 rst_n = 1;

        // ① 传感器不在：所有偏移
        for (k = -256; k < 256; k = k + 1) begin
            o = k; off = o;
            repeat (3) @(posedge clk);
            e = clamp_add(255, o);
            if (eff !== e[7:0] || base !== 8'd255) begin $display("FAIL bypass off=%0d eff=%0d", k, eff); errors = errors + 1; end
        end
        off = 9'd0;
        $display("PASS bypass: eff == clamp(255+offset) for all 512 offsets");

        // ⑤a 第一次读数不发 env_change
        ok = 1;
        send(raws[0]); settle;
        if (n_env != 0) begin $display("FAIL first reading raised env_change"); errors = errors + 1; end
        else $display("PASS first reading after plug-in: no env_change (Flash offset kept)");
        if (base !== 8'd64) begin $display("FAIL first reading base=%0d", base); errors = errors + 1; end

        // ② 各照度 + ⑤b 每个新环境恰好一个 env_change
        for (k = 1; k < 10; k = k + 1) begin
            n0 = n_env; b0 = base;
            send(raws[k]);
            settle;
            r = ref_base(raws[k]);
            if (base != r && !(((base > r) ? base - r : r - base) <= 8)) begin
                $display("FAIL raw=%0d base=%0d exp=%0d", raws[k], base, r); errors = errors + 1;
            end
            e = (((b0 > r) ? b0 - r : r - b0) > 8) ? 1 : 0;
            if (n_env - n0 != e) begin $display("FAIL raw=%0d env_change count %0d exp %0d", raws[k], n_env - n0, e); errors = errors + 1; end
            $display("PASS raw=%0d (~%0d lux) L=%0d base=%0d (exp %0d) env_change=%0d", raws[k], raws[k] * 10 / 12, ref_L(raws[k]), base, r, n_env - n0);
            check_offsets("eff");
        end

        // ④ 迟滞
        send(120); settle;
        b0 = base; n0 = n_env;
        send(130); settle;        // L 27 → 28：差 7
        if (base !== b0 || n_env != n0) begin $display("FAIL hysteresis moved %0d -> %0d env=%0d", b0, base, n_env - n0); errors = errors + 1; end
        else $display("PASS hysteresis: small change ignored (base %0d, no env_change)", base);
        send(200); settle;
        if (base !== ref_base(200) || n_env != n0 + 1) begin $display("FAIL after big change base=%0d env=%0d", base, n_env - n0); errors = errors + 1; end

        // ⑥ 随机偏移
        for (k = 0; k < 300; k = k + 1) begin
            off = $random;
            repeat (3) @(posedge clk);
            e = clamp_add(base, off);
            if (eff !== e[7:0]) begin $display("FAIL random base=%0d off=%0d eff=%0d", base, $signed(off), eff); errors = errors + 1; end
        end
        off = 9'd0;
        $display("PASS random offsets");

        // ⑤c 掉线：回到 255、不发 env_change
        n0 = n_env;
        ok = 0; settle;
        if (base !== 8'd255 || n_env != n0) begin $display("FAIL sensor lost base=%0d env=%0d", base, n_env - n0); errors = errors + 1; end
        else $display("PASS sensor lost: back to base 255, no env_change");
        // ⑤d 重新插上：第一次读数又不发，之后的新环境发
        ok = 1; send(9); settle;
        if (n_env != n0 || base !== 8'd64) begin $display("FAIL replug first reading env=%0d base=%0d", n_env - n0, base); errors = errors + 1; end
        send(1200); settle;
        if (n_env != n0 + 1 || base !== 8'd255) begin $display("FAIL replug second reading env=%0d base=%0d", n_env - n0, base); errors = errors + 1; end
        else $display("PASS replug: first reading quiet, next environment raises env_change");

        if (errors == 0) $display("TB_AMBIENT_BRIGHTNESS PASS");
        else             $display("TB_AMBIENT_BRIGHTNESS FAIL errors=%0d", errors);
        $finish;
    end
endmodule
