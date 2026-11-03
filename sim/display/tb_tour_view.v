`timescale 1ns/1ps
// tb_tour_view —— 视角渲染 + 行 DMA（逐像素与整数参考模型比对；资源取舍：无小地图）
//   时序：真实 video_timing_ctrl（640×480@ACTIVE_LAST）；仲裁：真实 media_sdram_arbiter；
//   SDRAM：读延迟 11 拍、burst 后 busy 15 拍的行为模型（数据 = 地址的函数，不存数组）；
//   另有一个每 2000 拍抢一次总线的 128 字写口（模拟播放器后台加载的写入）。
//   帧序列：整图（必须与帧缓存逐点相同）→ 2 倍带小数偏移 → 4 倍贴右下边 → 1.5 倍 → 2 倍（bundle[1] 置 1 应无影响）
//          → 原小地图位（show / cap / 视框）置满应无影响 → 关闭
module tb_tour_view;
    reg pclk = 0, mclk = 0, rst_n = 0;
    always #20 pclk = ~pclk;       // 25 MHz
    always #12.5 mclk = ~mclk;     // 40 MHz
    integer errors = 0;

    // ---------------- 时序 ----------------
    wire hs, vs, de; wire [11:0] rx, ry;
    video_timing_ctrl #(.H_ACT(640), .H_FP(16), .H_SYNC(96), .H_BP(48), .V_ACT(480), .V_FP(10), .V_SYNC(2), .V_BP(33),
        .ACTIVE_LAST(1), .VS_ALIGN_HSYNC(1)) u_t (.pclk(pclk), .rst_n(rst_n), .hsync(hs), .vsync(vs), .de(de), .x(rx), .y(ry));
    reg vs_q = 1; always @(posedge pclk) vs_q <= vs;
    wire fs = vs_q && !vs;

    // ---------------- 像素内容 ----------------
    function [23:0] fbpix(input integer bank, input integer row, input integer col);
        integer r, g, b;
        begin
            r = (col * 3 + row + bank * 40) & 255;
            g = (row * 2 + (col >> 1)) & 255;
            b = ((col * col) / 7 + row * 5) & 255;
            fbpix = {r[7:0], g[7:0], b[7:0]};
        end
    endfunction
    function [31:0] memword(input integer addr);
        begin
            if (addr >= 1228800) memword = 32'hDEAD_BEEF;              // 不应读到帧缓存以外
            else memword = {8'h00, fbpix(addr / 307200, (addr % 307200) / 640, addr % 640)};
        end
    endfunction

    // ---------------- DUT ----------------
    reg  [80:0] bundle = 0;
    reg  [1:0]  bank = 2'd2;
    wire        tour_on; wire [23:0] trgb;
    wire dma_want, dma_busy, dma_own, app_rd_en; wire [20:0] app_rd_addr;
    wire rd_grant, wr_grant, ref_req;
    reg  ctrl_busy = 0, ref_vld = 0;
    reg  sdr_rd_en = 0; reg [31:0] sdr_rd_dout = 0;
    reg  wr_want = 0, wr_busy = 0;
    tour_view dut (
        .pix_clk(pclk), .pix_rst_n(rst_n), .bundle(bundle), .frame_start(fs), .raw_x(rx), .raw_y(ry), .bank(bank),
        .tour_on(tour_on), .tour_rgb(trgb),
        .mem_clk(mclk), .mem_rst_n(rst_n), .dma_want(dma_want), .dma_grant(rd_grant), .dma_busy(dma_busy), .dma_own(dma_own),
        .app_rd_en(app_rd_en), .app_rd_addr(app_rd_addr), .sdram_rd_valid(sdr_rd_en), .sdram_rd_data(sdr_rd_dout));
    media_sdram_arbiter #(.IDLE_CYCLES(8), .REF_INTERVAL(576)) u_arb (
        .clk(mclk), .rst_n(rst_n), .ctrl_busy(ctrl_busy), .ctrl_init_done(1'b1), .ref_vld(ref_vld), .ref_req(ref_req),
        .wr_want(wr_want), .wr_busy(wr_busy), .wr_grant(wr_grant), .rd_want(dma_want), .rd_busy(dma_busy), .rd_grant(rd_grant));

    // SDRAM 行为：读 11 拍延迟；burst 结束后 busy 15 拍；刷新 20 拍
    reg [31:0] pipe_d [0:15]; reg [15:0] pipe_v = 0;
    integer bcnt = 0, i, addr_chk = 0, last_rd = -1, n_bursts = 0;
    always @(posedge mclk) begin
        pipe_v <= {pipe_v[14:0], app_rd_en};
        for (i = 15; i > 0; i = i - 1) pipe_d[i] <= pipe_d[i - 1];
        pipe_d[0] <= memword(app_rd_addr);
        sdr_rd_en   <= pipe_v[9];
        sdr_rd_dout <= pipe_d[9];
        if (app_rd_en) begin
            if (wr_busy) begin $display("FAIL read during write"); errors = errors + 1; end
            if (app_rd_addr >= 1228800) begin $display("FAIL read outside frame buffers %0d", app_rd_addr); errors = errors + 1; end
            if (last_rd != app_rd_addr - 1 && (app_rd_addr % 4) != 0) begin $display("FAIL burst start not 4-aligned %0d", app_rd_addr); errors = errors + 1; end
            last_rd = app_rd_addr;
        end
    end
    integer quiet = 99;
    always @(posedge mclk) begin
        if (app_rd_en || wr_busy || ref_vld) quiet = 0; else if (quiet < 99) quiet = quiet + 1;
        ctrl_busy <= (quiet < 15) || (pipe_v != 0);
    end
    always @(posedge mclk) if (ref_req) begin repeat (2) @(posedge mclk); ref_vld <= 1; repeat (20) @(posedge mclk); ref_vld <= 0; end
    // 写口：每 2000 拍一个 128 字 burst
    integer wtick = 0;
    always @(posedge mclk) begin
        wtick = wtick + 1;
        if (wtick >= 2000 && !wr_busy) wr_want <= 1;
        if (wr_want && wr_grant) begin wr_want <= 0; wtick = 0; wr_busy <= 1; end
        else if (wr_busy && wtick == 128) wr_busy <= 0;
    end
    // burst 长度检查（4 的倍数）
    integer blen = 0;
    always @(posedge mclk) begin
        if (app_rd_en) blen = blen + 1;
        else if (blen != 0) begin
            if (blen % 4 != 0 || blen > 128) begin $display("FAIL burst length %0d", blen); errors = errors + 1; end
            n_bursts = n_bursts + 1;
            blen = 0;
        end
    end

    // ---------------- 参考模型 ----------------
    integer v_u0, v_v0, v_s, v_bank;
    function integer lerp(input integer a, input integer b, input integer f);
        integer d;
        begin d = (b - a) * f; lerp = a + ((d >= 0) ? (d >> 6) : -((-d + 63) >> 6)); end
    endfunction
    function [23:0] lerp3(input [23:0] a, input [23:0] b, input integer f);
        integer r, g, bb;
        begin
            r = lerp(a[23:16], b[23:16], f); g = lerp(a[15:8], b[15:8], f); bb = lerp(a[7:0], b[7:0], f);
            lerp3 = {r[7:0], g[7:0], bb[7:0]};
        end
    endfunction
    function [23:0] src_pix(input integer row, input integer col);
        begin src_pix = fbpix(v_bank, row, col); end
    endfunction
    function [23:0] expect_pix(input integer X, input integer Y);
        integer U, V, a, b, fy, c, c1, fx, mx, mr;
        reg [23:0] L, R;
        begin
            V = v_v0 * 1024 + Y * v_s; U = v_u0 * 1024 + X * v_s; mr = 479; mx = 639;
            a = V >> 16; if (a > mr) a = mr;
            b = (a == mr) ? a : a + 1;
            fy = (V >> 10) & 63;
            if (!dut.c_bok) begin fy = 0; b = a; end      // 降级：下一行没到 → 本行只用 a 行
            c = U >> 16; c1 = (c >= mx) ? mx : c + 1;
            fx = (U >> 10) & 63;
            L = lerp3(src_pix(a, c),  src_pix(b, c),  fy);
            R = lerp3(src_pix(a, c1), src_pix(b, c1), fy);
            expect_pix = lerp3(L, R, fx);
        end
    endfunction

    // 视角：u0 / v0 Q.6，s Q.16
    task set_view(input integer u0, input integer v0, input integer s, input integer sharp, input integer show, input integer cap,
                  input integer mx0, input integer mx1, input integer my0, input integer my1);
        begin
            bundle = 0;
            bundle[0] = 1; bundle[1] = sharp; bundle[2] = show; bundle[3] = cap;
            bundle[19:4] = u0; bundle[34:20] = v0; bundle[51:35] = s;
            bundle[58:52] = mx0; bundle[66:59] = mx1; bundle[73:67] = my0; bundle[80:74] = my1;
        end
    endtask

    // 每帧：frame_start 时把视角参数记下（与 DUT 同一拍锁存）
    integer fr = 0;
    always @(posedge pclk) if (fs) begin
        fr = fr + 1;
        v_u0 = bundle[19:4]; v_v0 = bundle[34:20]; v_s = bundle[51:35];
        v_bank = bank;
    end

    // 逐像素比对（全屏）
    integer chk = 0, mism = 0, mism_lines = 0, line_bad = 0, X, Y, ck_frames = 0;
    reg [23:0] e;
    integer nfb = 0;
    always @(posedge pclk) begin
        if (de && tour_on && chk) begin
            X = rx - 160; Y = ry - 45;
            e = expect_pix(X, Y);
            if (trgb !== e) begin
                mism = mism + 1; line_bad = 1;
                if (mism <= 8) $display("FAIL frame %0d (%0d,%0d) got %h expect %h", fr, X, Y, trgb, e);
            end
            if (X == 639) begin mism_lines = mism_lines + line_bad; line_bad = 0; nfb = nfb + !dut.c_bok; end
        end
    end

    task run_frame(input [8*40-1:0] tag); integer m0; begin
        m0 = mism;
        @(posedge fs); chk = 1;          // 本帧开始用新视角
        @(posedge pclk);
        wait (ry == 525 - 1 && rx == 799);
        chk = 0;
        $display("INFO %0s: frame %0d mismatches %0d (lines %0d), b-fallback lines %0d, bursts so far %0d", tag, fr, mism - m0, mism_lines, nfb, n_bursts);
        ck_frames = ck_frames + 1;
    end endtask

    initial begin
        repeat (5) @(posedge pclk); rst_n = 1;
        // 第 1 帧：tour 关 → 不出图
        @(posedge fs); repeat (100) @(posedge pclk);
        if (tour_on) begin $display("FAIL tour_on without bundle"); errors = errors + 1; end
        // 1. 整图
        set_view(0, 0, 65536, 0, 0, 0, 0, 127, 0, 95);
        run_frame("home s=1");
        // 2. 2 倍、u0 = 123.4 px、v0 = 77.7 px
        set_view(7898, 4973, 32768, 0, 0, 0, 0, 0, 0, 0);
        run_frame("zoom2 frac");
        // 3. 4 倍贴右下（u0 = 480 px、v0 = 360 px），s = 16384
        set_view(30720, 23040, 16384, 0, 0, 0, 0, 0, 0, 0);
        run_frame("zoom4 corner");
        // 3b. 1.5 倍任意小数
        set_view(5001, 3333, 43690, 0, 0, 0, 0, 0, 0, 0);
        run_frame("zoom1.5");
        // 4. bundle[1]（原 sharp 位）置 1：应与普通 2 倍相同
        set_view(7898, 4973, 32768, 1, 0, 0, 0, 0, 0, 0);
        run_frame("bit1 ignored");
        // 5. 原小地图位（show / cap / 视框）置满：应与普通视角相同（右下角不应出现小地图）
        set_view(11349, 6513, 32768, 1, 1, 1, 127, 255, 127, 127);
        run_frame("old minimap bits ignored");
        // 7. 关闭
        bundle = 0;
        @(posedge fs); repeat (10) @(posedge pclk);
        if (tour_on) begin $display("FAIL tour_on after off"); errors = errors + 1; end
        repeat (2000) @(posedge mclk);
        if (dma_busy || dma_want) begin $display("FAIL dma still busy"); errors = errors + 1; end

        errors = errors + mism;
        if (errors == 0) $display("PASS tb_tour_view");
        else $display("FAIL tb_tour_view errors=%0d", errors);
        $finish;
    end
    initial begin #400_000_000; $display("FAIL timeout"); $finish; end
endmodule
