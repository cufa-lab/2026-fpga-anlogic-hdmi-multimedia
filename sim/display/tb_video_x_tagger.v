`timescale 1ns/1ps
// tb_video_x_tagger：列号标注 + 流水线对齐
// 激励为与顶层相同的 video_timing_data + video_delay 时序链(640x480@60)。
//   * 上电锁定：第一个输出 hs 下降沿之前输出保持空闲(hs=vs=1,de=0,rgb=0，无伪 vs 沿)，
//     该下降沿 video_x == 656 且出现在复位释放后两行(1600 拍)内(复位后 hs 先为低，第一个真下降沿在约 840 拍)；
//     vs 在见到高电平前保持 1(复位后 color_bar 的 vs 低电平不是场同步，否则首帧会多数 8 行)，同拍 locked_o 置 1
//     de/rgb 在第一个真实 vs 下降沿之后才透传(首帧完整，symbol core 可预测第一条有效行)
//   * 锁定后 hs/vs/de/rgb 输出 == 输入延迟 1 拍(同拍对齐)
//   * 每条有效行第一个 de_o 像素 video_x == 0，行内逐拍 +1，799 → 0
//   * 锁定后 video_x 连续递增 mod 800(场消隐期间不跳变)，de_o == (x < 640)
//   * hs 下降沿固定在 x == 640+16 = 656；vs 下降沿固定在同一 x
//   * vs 下降沿到第一条有效行之间恰好 36 次 x 回绕：symbol core 在计数 == FIRST_ACTIVE_WRAP(35)
//     的那条消隐行(即最后一条消隐行)末尾预测下一行有效，回绕后计数变 36 时 de 到来
//   * 每个完整帧恰好 480 条有效行
module tb_video_x_tagger;
    parameter integer FRAMES = 3;
    reg clk = 1'b0;
    always #20 clk = ~clk;
    reg rst_n = 1'b0;

    wire read_req, hs_0, vs_0, de_0, hs_d, vs_d, de_d, read_en;
    reg  read_req_ack = 1'b0;
    reg  [23:0] pix = 24'd0;
    wire [23:0] vout_raw;
    always @(posedge clk) begin
        read_req_ack <= read_req;
        if (read_en) pix <= pix + 24'h010203;
    end
    video_timing_data u_timing (
        .video_clk(clk), .rst_n(rst_n), .read_req(read_req), .read_req_ack(read_req_ack),
        .hs(hs_0), .vs(vs_0), .de(de_0)
    );
    video_delay u_delay (
        .video_clk(clk), .rst_n(rst_n), .read_en(read_en), .read_data(pix),
        .hs(hs_0), .vs(vs_0), .de(de_0), .hs_r(hs_d), .vs_r(vs_d), .de_r(de_d), .vout_data(vout_raw)
    );

    wire hs, vs, de;
    wire [23:0] rgb;
    wire [9:0] vx;
    wire locked_o;
    video_x_tagger dut (
        .clk(clk), .rst_n(rst_n), .hs_i(hs_d), .vs_i(vs_d), .de_i(de_d), .rgb_i(vout_raw),
        .hs_o(hs), .vs_o(vs), .de_o(de), .rgb_o(rgb), .video_x_o(vx), .locked_o(locked_o)
    );

    integer errors = 0, lines = 0, frames = 0, wraps = 0, vs_x = -1;
    reg locked = 1'b0;
    reg hs_q = 1'b1, vs_q = 1'b1, de_q = 1'b0;
    reg [26:0] in_q = {1'b1, 1'b1, 1'b0, 24'd0};   // 与 DUT 复位值一致
    integer lines_in_frame = 0;
    reg first_edge;
    reg rst_q = 1'b0;
    integer cyc = 0;
    reg vs_seen = 1'b0, vs_d_q = 1'b1, fl = 1'b0;   // 参考模型：vs 见高后透传；第一个 vs 下降沿后 de/rgb 透传
    wire fl_now = fl || (vs_seen && vs_d_q && !vs_d && locked);
    reg [9:0] vx_q;
    integer wraps_since_vs = -1;
    task err(input [8*80-1:0] m);
        begin errors = errors + 1; if (errors < 20) $display("FAIL @%0t x=%0d: %0s", $time, vx, m); end
    endtask

    always @(posedge clk) begin
        if (rst_n) begin
            cyc = cyc + 1;
            first_edge = !locked && !hs && hs_q;               // 第一个输出 hs 下降沿 = 锁定点
            if (first_edge) begin
                if (vx !== 10'd656) err("x at lock (first hs fall) != 656");
                if (cyc > 1600) err("tagger did not lock within the first two lines");
                if (locked_o !== 1'b1) err("locked_o not set at first hs fall");
                locked = 1'b1;
            end
            if (!locked && !first_edge) begin
                if ({hs, vs, de, rgb} !== {1'b1, 1'b1, 1'b0, 24'd0}) err("outputs not idle before lock");
                if (locked_o !== 1'b0) err("locked_o set before first hs fall");
            end else if (rst_q && {hs, vs, de, rgb} !== in_q) begin
                err("outputs are not input delayed by 1 cycle"); if (errors < 3) $display("  got %b exp %b", {hs, vs, de, rgb}, in_q);
            end
            if (de && !de_q) begin
                if (vx !== 10'd0) err("first pixel of line has x != 0");
                if (wraps_since_vs >= 0) begin
                    if (wraps_since_vs != 36) begin
                        errors = errors + 1; $display("FAIL vs->first active line = %0d wraps (expect 36)", wraps_since_vs);
                    end
                    wraps_since_vs = -1;
                end
                lines = lines + 1; lines_in_frame = lines_in_frame + 1;
            end
            if (locked) begin
                if (!first_edge && vx !== ((vx_q == 10'd799) ? 10'd0 : vx_q + 1'b1)) err("x not continuous mod 800");
                if (de && vx >= 640) err("de beyond x=639");
                if (!de && de_q && vx !== 10'd640) err("de fell at x != 640");
                if (!hs && hs_q && vx !== 10'd656) err("hs falling edge not at x=656");
                if (!vs && vs_q) begin
                    if (frames > 0 && lines_in_frame != 480) begin
                        errors = errors + 1; $display("FAIL frame %0d has %0d active lines", frames, lines_in_frame);
                    end
                    frames = frames + 1; wraps_since_vs = 0; lines_in_frame = 0;
                    if (vs_x < 0) vs_x = vx; else if (vx != vs_x) err("vs falling edge moved");
                end
                if (vx == 10'd799) begin wraps = wraps + 1; if (wraps_since_vs >= 0) wraps_since_vs = wraps_since_vs + 1; end
            end
        end
        in_q <= {hs_d, vs_seen ? vs_d : 1'b1, fl_now & de_d, fl_now ? vout_raw : 24'd0};
        if (rst_n) begin if (vs_d === 1'b1) vs_seen <= 1'b1; vs_d_q <= vs_d; if (fl_now) fl <= 1'b1; end rst_q <= rst_n;
        hs_q <= hs; vs_q <= vs; de_q <= de; vx_q <= vx;
    end

    initial begin
        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        wait (frames == FRAMES + 1);
        repeat (800 * 40) @(posedge clk);
        $display("INFO lines=%0d frames=%0d vs falling edge at x=%0d, x wraps=%0d", lines, frames, vs_x, wraps);
        if (errors == 0) $display("PASS tb_video_x_tagger: 1-cycle aligned, x=0 at first pixel, continuous mod 800, hs@656, vs->active = 36 wraps (WRAP=35), 480 lines/frame");
        else             $display("FAIL tb_video_x_tagger: %0d errors", errors);
        $finish;
    end
    initial begin #(40.0 * 800 * 525 * (FRAMES + 3)); $display("FAIL tb_video_x_tagger: timeout"); $finish; end
endmodule
