`timescale 1ns/1ps
// brightness_contrast_adjust：逐位比对参考模型
//   C1 对比度穷举 x(0..255) × c(0..255)，RGB 三通道各喂不同值；参考 = floor((x-128)·c/128)+128 饱和
//      （floor 用整数显式实现，等价于 >>>7；若 RTL 用逻辑右移 / 截断除法会在负数侧失配）
//   C2 c=128 逐位直通；c=0 整屏 128；端点 / 中间位宽极值（-127 → 0、381 → 255）
//   B1 eff_alpha 穷举 alpha × b：alpha_out = alpha·(b+1)>>8；b=255 时 alpha_out == alpha_in
//   F1 参数只在 frame_start 生效：中途改 brightness_in / contrast_in 输出不变，frame_start 后才变
//   P1 PIPELINE=1：rgb_out / out_de 同拍晚 1 拍，与输入逐拍对齐
//   S1 BRIGHT_STANDALONE=1：先 x·(b+1)>>8 再对比度，alpha 直通；b=255 & c=128 逐位旁路；三通道
//   R  复位值 255 / 128（复位后未见 frame_start 也是全亮直通）
module tb_brightness_contrast_adjust;
    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    reg        fs = 0;
    reg  [7:0] b_in = 255, c_in = 128, a_in = 255;
    reg        de_in = 0;
    reg [23:0] rgb_in = 0;

    wire        de0, dep, des;
    wire [23:0] rgb0, rgbp, rgbs;
    wire [7:0]  a0, ap, as_;
    wire [7:0]  bq, cq;

    brightness_contrast_adjust #(.BRIGHT_STANDALONE(0), .PIPELINE(0)) dut (
        .clk(clk), .rst_n(rst_n), .frame_start(fs), .brightness_in(b_in), .contrast_in(c_in),
        .alpha_in(a_in), .in_de(de_in), .rgb_in(rgb_in),
        .out_de(de0), .rgb_out(rgb0), .alpha_out(a0), .brightness_q(bq), .contrast_q(cq));
    brightness_contrast_adjust #(.BRIGHT_STANDALONE(0), .PIPELINE(1)) dut_p (
        .clk(clk), .rst_n(rst_n), .frame_start(fs), .brightness_in(b_in), .contrast_in(c_in),
        .alpha_in(a_in), .in_de(de_in), .rgb_in(rgb_in),
        .out_de(dep), .rgb_out(rgbp), .alpha_out(ap), .brightness_q(), .contrast_q());
    brightness_contrast_adjust #(.BRIGHT_STANDALONE(1), .PIPELINE(0)) dut_s (
        .clk(clk), .rst_n(rst_n), .frame_start(fs), .brightness_in(b_in), .contrast_in(c_in),
        .alpha_in(a_in), .in_de(de_in), .rgb_in(rgb_in),
        .out_de(des), .rgb_out(rgbs), .alpha_out(as_), .brightness_q(), .contrast_q());

    integer errors = 0, checks = 0;
    integer x, c, a, b, k;

    // ---------------- 参考模型 ----------------
    function integer floor_div128(input integer p);
        floor_div128 = (p >= 0) ? (p / 128) : -((-p + 127) / 128);
    endfunction
    function [7:0] ref_con(input integer xv, input integer cv);
        integer s;
        begin
            s = floor_div128((xv - 128) * cv) + 128;
            ref_con = (s < 0) ? 8'd0 : (s > 255) ? 8'd255 : s[7:0];
        end
    endfunction
    function [7:0] ref_bri(input integer xv, input integer bv);
        ref_bri = (xv * (bv + 1)) >> 8;
    endfunction
    function [23:0] ref_rgb_def(input [23:0] p, input integer cv);
        ref_rgb_def = {ref_con(p[23:16], cv), ref_con(p[15:8], cv), ref_con(p[7:0], cv)};
    endfunction
    function [23:0] ref_rgb_sa(input [23:0] p, input integer bv, input integer cv);
        ref_rgb_sa = {ref_con(ref_bri(p[23:16], bv), cv), ref_con(ref_bri(p[15:8], bv), cv), ref_con(ref_bri(p[7:0], bv), cv)};
    endfunction

    task commit(input [7:0] bv, input [7:0] cv);   // 改参数 + 一个 frame_start
        begin
            @(negedge clk) begin b_in = bv; c_in = cv; fs = 1; end
            @(negedge clk) fs = 0;
        end
    endtask
    task fail(input [8*40-1:0] tag, input [23:0] got, input [23:0] exp);
        begin
            if (errors < 20) $display("FAIL %0s got %h exp %h (b=%0d c=%0d a=%0d rgb=%h)", tag, got, exp, bq, cq, a_in, rgb_in);
            errors = errors + 1;
        end
    endtask

    // ---------------- P1 PIPELINE=1 逐拍对齐检查（后台常开） ----------------
    reg [23:0] exp_p_rgb; reg exp_p_de = 0; reg p_armed = 0;
    always @(posedge clk) begin
        if (p_armed) begin
            if (dep !== exp_p_de) fail("P1 PIPELINE de align", {23'd0, dep}, {23'd0, exp_p_de});
            if (exp_p_de && rgbp !== exp_p_rgb) fail("P1 PIPELINE rgb align", rgbp, exp_p_rgb);
        end
        exp_p_de  <= de_in;
        exp_p_rgb <= ref_rgb_def(rgb_in, cq);
        p_armed   <= rst_n;
    end

    initial begin
        #23 rst_n = 1;
        // ---- R 复位值 ----
        @(negedge clk);
        if (bq !== 8'd255 || cq !== 8'd128) fail("R reset q", {bq, cq}, {8'd255, 8'd128});
        rgb_in = 24'h12_80_fe; de_in = 1; #1;
        if (rgb0 !== 24'h12_80_fe || a0 !== 8'd255 || rgbs !== 24'h12_80_fe) fail("R reset bypass", rgb0, 24'h12_80_fe);

        // ---- C1/C2 对比度穷举（三通道） ----
        for (c = 0; c < 256; c = c + 1) begin
            commit(8'd255, c[7:0]);
            for (x = 0; x < 256; x = x + 1) begin
                @(negedge clk);
                rgb_in = {x[7:0], ~x[7:0], x[7:0] ^ c[7:0]};
                de_in  = x[0] | x[3];                 // de 跟随变化，检查组合 de 直通
                #1;
                checks = checks + 1;
                if (rgb0 !== ref_rgb_def(rgb_in, c)) fail("C1 contrast", rgb0, ref_rgb_def(rgb_in, c));
                if (de0 !== de_in) fail("C1 de passthrough", {23'd0, de0}, {23'd0, de_in});
                if (c == 128 && rgb0 !== rgb_in) fail("C2 c=128 bypass", rgb0, rgb_in);
                if (c == 0 && rgb0 !== 24'h80_80_80) fail("C2 c=0 mid gray", rgb0, 24'h808080);
                // 亮度 255 + 对比度 c：standalone 与默认像素一致
                if (rgbs !== rgb0) fail("S1 b=255 standalone == default", rgbs, rgb0);
            end
        end
        // 端点 / 中间位宽：(0-128)·255 = -32640 >>>7 = -255 → -127 → 0；127·255 = 32385 → 253+128 = 381 → 255
        commit(8'd255, 8'd255);
        @(negedge clk) rgb_in = {8'd0, 8'd255, 8'd128}; #1;
        if (rgb0 !== {8'd0, 8'd255, 8'd128}) fail("C2 extreme c=255", rgb0, {8'd0, 8'd255, 8'd128});
        // 负数侧算术移位：x=100,c=64 → (-28·64)>>>7 = -14 → 114；x=127,c=1 → (-1)>>>7 = -1 → 127（逻辑移位/截断会得 128）
        commit(8'd255, 8'd64);
        @(negedge clk) rgb_in = {8'd100, 8'd156, 8'd127}; #1;
        if (rgb0 !== {8'd114, 8'd142, 8'd127}) fail("C2 arithmetic shift", rgb0, {8'd114, 8'd142, 8'd127});
        commit(8'd255, 8'd1);
        @(negedge clk) rgb_in = {8'd127, 8'd0, 8'd255}; #1;
        if (rgb0 !== {8'd127, 8'd127, 8'd128}) fail("C2 floor of -1/128", rgb0, {8'd127, 8'd127, 8'd128});
        $display("INFO C1/C2 contrast exhaustive %0d pixels x 3 channels", checks);

        // ---- B1 eff_alpha 穷举 ----
        for (b = 0; b < 256; b = b + 1) begin
            commit(b[7:0], 8'd128);
            for (a = 0; a < 256; a = a + 1) begin
                @(negedge clk) a_in = a[7:0]; #1;
                if (a0 !== ((a * (b + 1)) >> 8)) fail("B1 eff_alpha", {16'd0, a0}, (a * (b + 1)) >> 8);
                if (b == 255 && a0 !== a_in) fail("B1 b=255 alpha bypass", {16'd0, a0}, {16'd0, a_in});
                if (as_ !== a_in) fail("S1 standalone alpha passthrough", {16'd0, as_}, {16'd0, a_in});
                if (rgb0 !== rgb_in) fail("B1 brightness must not touch pixels (c=128)", rgb0, rgb_in);
            end
        end
        a_in = 255;
        commit(8'd0, 8'd128); #1;
        if (a0 !== 8'd0) fail("B1 b=0 alpha=255 -> 0", {16'd0, a0}, 0);
        commit(8'd127, 8'd128); #1;
        if (a0 !== 8'd127) fail("B1 b=127 alpha=255 -> 127", {16'd0, a0}, 127);
        commit(8'd191, 8'd128); #1;
        if (a0 !== 8'd191) fail("B1 b=191 alpha=255 -> 191", {16'd0, a0}, 191);

        // ---- S1 standalone：亮度 × 对比度组合（随机 + 端点），三通道 ----
        for (k = 0; k < 4000; k = k + 1) begin
            if (k % 100 == 0) commit($urandom, (k < 400) ? 8'd128 : $urandom);
            @(negedge clk) rgb_in = $urandom; #1;
            if (rgbs !== ref_rgb_sa(rgb_in, bq, cq)) fail("S1 standalone bri+con", rgbs, ref_rgb_sa(rgb_in, bq, cq));
            if (rgb0 !== ref_rgb_def(rgb_in, cq)) fail("S1 default ignores bri on pixels", rgb0, ref_rgb_def(rgb_in, cq));
        end
        commit(8'd0, 8'd128);
        @(negedge clk) rgb_in = 24'hff_ff_ff; #1;
        if (rgbs !== 24'h00_00_00) fail("S1 standalone b=0 -> 0", rgbs, 0);
        commit(8'd255, 8'd128);
        @(negedge clk) rgb_in = 24'hff_01_80; #1;
        if (rgbs !== 24'hff_01_80) fail("S1 standalone b=255 bypass", rgbs, 24'hff0180);

        // ---- F1 帧边界锁存 ----
        commit(8'd200, 8'd100);
        @(negedge clk) begin b_in = 8'd10; c_in = 8'd250; rgb_in = 24'h20_80_e0; a_in = 255; end
        repeat (50) begin
            @(negedge clk); #1;
            if (bq !== 8'd200 || cq !== 8'd100) fail("F1 q changed without frame_start", {bq, cq}, {8'd200, 8'd100});
            if (rgb0 !== ref_rgb_def(rgb_in, 100) || a0 !== ((255 * 201) >> 8)) fail("F1 output changed mid-frame", rgb0, ref_rgb_def(rgb_in, 100));
        end
        @(negedge clk) fs = 1;
        @(posedge clk); #1;
        if (bq !== 8'd10 || cq !== 8'd250 || rgb0 !== ref_rgb_def(rgb_in, 250) || a0 !== ((255 * 11) >> 8))
            fail("F1 frame_start latch", {bq, cq}, {8'd10, 8'd250});
        @(negedge clk) fs = 0;

        // ---- P1：再跑一段 de 方波 + 随机像素，后台 always 块逐拍比对 ----
        commit(8'd255, 8'd77);
        for (k = 0; k < 2000; k = k + 1) begin
            @(negedge clk) begin rgb_in = $urandom; de_in = (k % 37) < 30; end
        end
        @(negedge clk);

        if (errors == 0) $display("PASS tb_brightness_contrast_adjust (contrast 65536x3 exhaustive, eff_alpha 65536 exhaustive, frame latch, PIPELINE align, STANDALONE)");
        else             $display("FAIL tb_brightness_contrast_adjust errors=%0d", errors);
        $finish;
    end
endmodule
