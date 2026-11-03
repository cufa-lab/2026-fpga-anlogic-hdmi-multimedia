`timescale 1ns/1ps
// frame_boundary_pulse：缩小的视频时序（H 20 拍 / V_ACTIVE 6 行 / VFP 2 / VSA 1 / VBP 2）
// 检查 frame_start 在 vs 下降沿、blank_start 在最后一行 de 下降沿，每帧各一次，
// 并且复位后第一次不完整的帧不输出。
module tb_frame_boundary_pulse;
    localparam H_TOTAL = 20, H_ACT = 12;
    localparam V_ACT = 6, V_FP = 2, V_SA = 1, V_BP = 2;
    localparam V_TOTAL = V_ACT + V_FP + V_SA + V_BP;

    reg clk = 0, rst_n = 0;
    always #20 clk = ~clk;

    integer h = 0, v = 3;           // 从帧中间开始（不完整帧）
    wire de      = (h < H_ACT) && (v < V_ACT);
    wire vsync_n = !((v >= V_ACT + V_FP) && (v < V_ACT + V_FP + V_SA));
    always @(posedge clk) begin
        if (h == H_TOTAL - 1) begin
            h <= 0;
            v <= (v == V_TOTAL - 1) ? 0 : v + 1;
        end else h <= h + 1;
    end

    wire frame_start, blank_start;
    frame_boundary_pulse #(.V_ACTIVE(V_ACT)) dut (
        .pix_clk(clk), .pix_rst_n(rst_n), .vsync_n(vsync_n), .de(de),
        .frame_start(frame_start), .blank_start(blank_start)
    );

    integer n_fs = 0, n_bs = 0, errors = 0;
    reg vs_d = 1, de_d = 0;
    always @(posedge clk) begin
        vs_d <= vsync_n; de_d <= de;
        if (frame_start) begin
            n_fs = n_fs + 1;
            // frame_start 比 vs 下降沿晚 1 拍：此刻 vs 已低、上一拍低
            if (!(vs_d == 0 && v == V_ACT + V_FP)) begin
                $display("FAIL: frame_start at v=%0d h=%0d", v, h); errors = errors + 1;
            end
        end
        if (blank_start) begin
            n_bs = n_bs + 1;
            if (!(v == V_ACT - 1 && h == H_ACT + 1)) begin
                $display("FAIL: blank_start at v=%0d h=%0d", v, h); errors = errors + 1;
            end
            if (n_fs == 0) begin
                $display("FAIL: blank_start before first vsync"); errors = errors + 1;
            end
        end
    end

    initial begin
        #100 rst_n = 1;
        repeat (H_TOTAL * V_TOTAL * 4) @(posedge clk);
        // 起点 v=3 → 第一次 vs 前的半帧不算
        if (n_fs < 3 || n_bs < 3 || (n_fs - n_bs) > 1 || (n_bs - n_fs) > 0) begin
            $display("FAIL: counts fs=%0d bs=%0d", n_fs, n_bs); errors = errors + 1;
        end
        if (errors == 0) $display("PASS tb_frame_boundary_pulse fs=%0d bs=%0d", n_fs, n_bs);
        else             $display("FAIL tb_frame_boundary_pulse errors=%0d", errors);
        $finish;
    end
endmodule
