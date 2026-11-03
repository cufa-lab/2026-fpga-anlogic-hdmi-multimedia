`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// tb_osd_alpha_blend ——  Alpha 混合器
//   每个 alpha（0..255）x 400 组随机/边界 (bg, fg)，逐通道检查：
//     1) alpha=0 → 输出 == bg；alpha=255 → 输出 == fg（逐位相等）
//     2) 与精确公式 fg*a/255 + bg*(255-a)/255 相差 <= 1
//     3) 输出落在 [min(bg,fg), max(bg,fg)]（不溢出、不反向）
//     4) alpha 单调：alpha 增大时输出向 fg 单调靠近
// -----------------------------------------------------------------------------
module tb_osd_alpha_blend;
    reg  [23:0] bg, fg;
    reg  [7:0]  alpha;
    wire [23:0] out;
    osd_alpha_blend dut (.bg(bg), .fg(fg), .alpha(alpha), .out(out));

    integer a, n, c, errors = 0, checks = 0;
    integer b8, f8, o8, lo, hi, prev;
    real    exact;
    reg [23:0] prev_out;

    initial begin
        for (n = 0; n < 400; n = n + 1) begin
            case (n)
                0: begin bg = 24'h000000; fg = 24'hffffff; end
                1: begin bg = 24'hffffff; fg = 24'h000000; end
                2: begin bg = 24'h123456; fg = 24'h123456; end
                3: begin bg = 24'hff00ff; fg = 24'h00ff00; end
                default: begin bg = $urandom; fg = $urandom; end
            endcase
            for (a = 0; a < 256; a = a + 1) begin
                alpha = a;
                #1;
                for (c = 0; c < 3; c = c + 1) begin
                    b8 = bg[c*8 +: 8]; f8 = fg[c*8 +: 8]; o8 = out[c*8 +: 8];
                    exact = (f8 * a + b8 * (255 - a)) / 255.0;
                    lo = (b8 < f8) ? b8 : f8;
                    hi = (b8 < f8) ? f8 : b8;
                    checks = checks + 1;
                    if ((a == 0 && o8 != b8) || (a == 255 && o8 != f8) ||
                        (o8 - exact > 1.0) || (exact - o8 > 1.0) || o8 < lo || o8 > hi) begin
                        if (errors < 10) $display("FAIL bg=%h fg=%h a=%0d ch%0d out=%0d exact=%f", bg, fg, a, c, o8, exact);
                        errors = errors + 1;
                    end
                    if (a > 0) begin
                        prev = prev_out[c*8 +: 8];
                        if ((f8 >= b8 && o8 < prev) || (f8 < b8 && o8 > prev)) begin
                            if (errors < 10) $display("FAIL non-monotonic bg=%h fg=%h a=%0d ch%0d", bg, fg, a, c);
                            errors = errors + 1;
                        end
                    end
                end
                prev_out = out;
            end
        end
        $display("INFO checks=%0d errors=%0d", checks, errors);
        if (errors == 0) $display("PASS tb_osd_alpha_blend");
        else             $display("FAIL tb_osd_alpha_blend");
        $finish;
    end
endmodule
