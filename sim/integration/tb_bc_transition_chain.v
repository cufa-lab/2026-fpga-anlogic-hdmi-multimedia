`timescale 1ns/1ps
// 亮度/对比度 × 转场 链路（与顶层同接法）：
//   sys 域 {contrast, brightness} → cdc_bus_handshake → pix 域
//   reader 像素 → brightness_contrast_adjust（frame_start 锁存，对比度在像素上，亮度折进 alpha）
//   → video_transition_pixel（Alpha 乘法 + 随机方块溶解）
//   逐像素参考：eff = alpha·(b+1)>>8；out = (eff==255) ? con(x) : con(x)·eff>>8；溶解中序号 ≥ 档位的块 = 黑
//   T1 b=255 c=128 alpha=255：逐位直通（无二次衰减、无舍入损失）
//   T2 淡出淡入序列 alpha 255→0→255 × 亮度 {255,191,127,63} × 对比度 {128,40,230}：逐像素比对；alpha=0 必须全黑
//      （低对比度下也是黑场，不会被拉回灰）
//   T3 亮度只处理一次：alpha=255 时 out = con(x)·(b+1)>>8（单次乘法）
//   T4 sys 域随机时刻改参数（异步时钟）：每帧内输出恒定对应同一组参数（帧内无撕裂），最终收敛到最新值
//   T5 方块溶解期间（低亮度 + 高对比度，seed 6）：显示块 = 已调整像素（亮度仍生效），隐去块 = 黑
module tb_bc_transition_chain;
    localparam integer H_ACT = 64, V_ACT = 24;
    reg pclk = 0, sclk = 0, rst_n = 0;
    always #5 pclk = ~pclk;              // 100 MHz（相对比例即可）
    always #7.3 sclk = ~sclk;            // 异步 sys 时钟

    wire hs, vs, de;
    wire [11:0] x, y;
    video_timing_ctrl #(.H_ACT(H_ACT), .H_FP(2), .H_SYNC(2), .H_BP(2), .V_ACT(V_ACT), .V_FP(1), .V_SYNC(1), .V_BP(1)) u_vt (
        .pclk(pclk), .rst_n(rst_n), .hsync(hs), .vsync(vs), .de(de), .x(x), .y(y));
    reg vs_q = 1;
    always @(posedge pclk) vs_q <= vs;
    wire fs = vs_q && !vs;               // 场同步下降沿（同 frame_start_raw）

    // sys 域参数
    reg [7:0] s_bri = 255, s_con = 128;
    wire [17:0] pic;
    cdc_bus_handshake #(.WIDTH(18), .RESET_VAL({1'b0, 1'b0, 8'd128, 8'd255})) u_cdc (
        .src_clk(sclk), .src_rst_n(rst_n), .src_data({2'b00, s_con, s_bri}),
        .dst_clk(pclk), .dst_rst_n(rst_n), .dst_data(pic));

    // 转场控制量（只在 frame_start 变化，同 video_fade_transition）
    reg [7:0] alpha = 255, alpha_next = 255;
    reg       dis = 0, dis_next = 0;
    reg [4:0] ramp = 0, ramp_next = 0;
    always @(posedge pclk) if (fs) begin alpha <= alpha_next; dis <= dis_next; ramp <= ramp_next; end

    wire [23:0] src = {x[5:0], 2'b00} | {8'd0, y[4:0] * 8'd11, x[7:0] ^ y[7:0]} | (x[0] ? 24'hff0000 : 24'h0);
    wire [23:0] bc_rgb, out_rgb;
    wire [7:0]  bc_alpha, bq, cq;
    wire        de_o;
    brightness_contrast_adjust #(.BRIGHT_STANDALONE(0), .PIPELINE(0)) u_bc (
        .clk(pclk), .rst_n(rst_n), .frame_start(fs), .brightness_in(pic[7:0]), .contrast_in(pic[15:8]),
        .alpha_in(alpha), .in_de(de), .rgb_in(src), .out_de(), .rgb_out(bc_rgb), .alpha_out(bc_alpha),
        .brightness_q(bq), .contrast_q(cq));
    video_transition_pixel #(.FADE_FRAMES(16), .PIPELINE(0)) u_tp (
        .clk(pclk), .rst_n(rst_n), .hs_in(hs), .vs_in(vs), .de_in(de), .x_in(x), .y_in(y),
        .rgb_in(bc_rgb), .alpha(bc_alpha), .dissolve_active(dis), .ramp(ramp),
        .seed(4'd6), .hs_out(), .vs_out(), .de_out(de_o), .rgb_out(out_rgb));

    // ---------------- 参考 ----------------
    function integer fdiv128(input integer p); fdiv128 = (p >= 0) ? p / 128 : -((-p + 127) / 128); endfunction
    function [7:0] con(input integer xv, input integer cv);
        integer s; begin s = fdiv128((xv - 128) * cv) + 128; con = (s < 0) ? 0 : (s > 255) ? 255 : s; end
    endfunction
    function [7:0] ch(input integer xv, input integer bv, input integer cv, input integer av);
        integer e; begin e = (av * (bv + 1)) >> 8; ch = (e == 255) ? con(xv, cv) : (con(xv, cv) * e) >> 8; end
    endfunction
    // 块序号参考（PRESENT S 盒查表 + 坐标位异或，同 tb_transition_chain）
    reg [3:0] sb [0:15];
    initial begin
        sb[0]=4'hC; sb[1]=4'h5; sb[2]=4'h6; sb[3]=4'hB; sb[4]=4'h9; sb[5]=4'h0; sb[6]=4'hA; sb[7]=4'hD;
        sb[8]=4'h3; sb[9]=4'hE; sb[10]=4'hF; sb[11]=4'h8; sb[12]=4'h4; sb[13]=4'h7; sb[14]=4'h1; sb[15]=4'h2;
    end
    function [3:0] blk(input [3:0] s, input [11:0] xx, input [11:0] yy);
        reg [3:0] u, k;
        begin
            u = {yy[5]^yy[6]^xx[9]^xx[7], xx[6]^yy[7]^xx[8]^xx[5], xx[7]^yy[8]^xx[8]^yy[6], xx[9]^xx[5]^xx[7]};
            k = {yy[7]^xx[7], yy[9]^xx[7], xx[5]^xx[7], yy[5]^xx[7]};
            blk = sb[sb[u] ^ k ^ s];
        end
    endfunction
    function [23:0] ref_px(input [23:0] p, input integer bv, input integer cv, input integer av);
        ref_px = {ch(p[23:16], bv, cv, av), ch(p[15:8], bv, cv, av), ch(p[7:0], bv, cv, av)};
    endfunction

    // 帧参数（本帧实际锁存值：在 fs 后 1 拍读 bq/cq/alpha）
    integer errors = 0, frames = 0, px_checked = 0, nonblack = 0;
    reg [7:0] f_b, f_c, f_a; reg f_w; reg [4:0] f_r;
    reg       chk_en = 0;
    always @(posedge pclk) if (fs) begin
        #1; f_b = bq; f_c = cq; f_a = alpha; f_w = dis; f_r = ramp;
        frames = frames + 1;
    end
    always @(negedge pclk) if (rst_n && chk_en && de_o) begin : chk
        reg [23:0] e;
        e = ref_px(src, f_b, f_c, f_a);
        if (f_w && !(blk(4'd6, x, y) < f_r)) e = 24'h0;
        px_checked = px_checked + 1;
        if (out_rgb !== 24'h0) nonblack = nonblack + 1;
        if (out_rgb !== e) begin
            if (errors < 15) $display("FAIL px x=%0d y=%0d src=%h b=%0d c=%0d a=%0d w=%b r=%0d: got %h exp %h",
                                      x, y, src, f_b, f_c, f_a, f_w, f_r, out_rgb, e);
            errors = errors + 1;
        end
        // 帧内参数不允许变化（锁存在 frame_start）
        if (bq !== f_b || cq !== f_c) begin
            if (errors < 15) $display("FAIL mid-frame parameter change b %0d->%0d c %0d->%0d", f_b, bq, f_c, cq);
            errors = errors + 1;
        end
    end

    task frame; begin @(posedge pclk); while (!fs) @(posedge pclk); end endtask
    task set_bc(input [7:0] b, input [7:0] c);          // sys 域改值，等 CDC + 下一帧锁存
        begin
            @(posedge sclk) begin s_bri <= b; s_con <= c; end
            frame; frame; frame;
            if (bq !== b || cq !== c) begin $display("FAIL set_bc not converged b=%0d c=%0d", bq, cq); errors = errors + 1; end
        end
    endtask

    integer bi, ci, k, nb0;
    reg [7:0] blist [0:3]; reg [7:0] clist [0:2];
    initial begin
        blist[0] = 255; blist[1] = 191; blist[2] = 127; blist[3] = 63;
        clist[0] = 128; clist[1] = 40; clist[2] = 230;
        #40 rst_n = 1;
        frame; frame; frame;
        chk_en = 1;
        // T1 直通
        frame; frame;
        if (bq !== 255 || cq !== 128) begin $display("FAIL T1 defaults"); errors = errors + 1; end
        // T2 / T3：亮度 × 对比度 × 淡出淡入
        for (bi = 0; bi < 4; bi = bi + 1)
            for (ci = 0; ci < 3; ci = ci + 1) begin
                set_bc(blist[bi], clist[ci]);
                for (k = 0; k <= 16; k = k + 1) begin              // 淡出 255 → 0
                    @(negedge pclk) alpha_next = (k == 16) ? 8'd0 : 8'd255 - k * 16;
                    frame;
                end
                frame;                                              // alpha=0 这一帧
                nb0 = nonblack; frame;
                if (nonblack != nb0) begin $display("FAIL T2 alpha=0 must be black (b=%0d c=%0d)", bq, cq); errors = errors + 1; end
                for (k = 1; k <= 16; k = k + 1) begin              // 淡入 0 → 255
                    @(negedge pclk) alpha_next = (k == 16) ? 8'd255 : k * 16;
                    frame;
                end
                frame;
            end
        // T4 sys 域任意时刻改参数（帧内检查由后台 always 完成）
        for (k = 0; k < 40; k = k + 1) begin
            repeat ($urandom % 900) @(posedge sclk);
            s_bri <= $urandom; s_con <= $urandom;
        end
        frame; frame; frame;
        if (bq !== s_bri || cq !== s_con) begin $display("FAIL T4 final convergence"); errors = errors + 1; end
        // T5 方块溶解：低亮度 + 高对比度，档位 16→0→16
        set_bc(8'd127, 8'd200);
        @(negedge pclk) begin dis_next = 1; alpha_next = 255; end
        for (k = 16; k >= 0; k = k - 1) begin @(negedge pclk) ramp_next = k; frame; end
        for (k = 0; k <= 16; k = k + 1) begin @(negedge pclk) ramp_next = k; frame; end
        @(negedge pclk) begin dis_next = 0; ramp_next = 0; end
        frame; frame;

        $display("INFO frames=%0d pixels checked=%0d", frames, px_checked);
        if (errors == 0 && px_checked > 100000) $display("PASS tb_bc_transition_chain (T1-T5: bypass, fade x brightness x contrast, black at alpha 0, single brightness, CDC frame latch, dissolve)");
        else                                    $display("FAIL tb_bc_transition_chain errors=%0d checked=%0d", errors, px_checked);
        $finish;
    end
endmodule
