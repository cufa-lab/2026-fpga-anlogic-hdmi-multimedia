`timescale 1ns/1ps

module video_transition_pixel #(
    parameter integer FADE_FRAMES = 16,    // 与 video_fade_transition 同参；0 = 旁路；溶解要求 ≤ 16
    parameter integer PIPELINE    = 0
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        hs_in,
    input  wire        vs_in,
    input  wire        de_in,
    input  wire [11:0] x_in,
    input  wire [11:0] y_in,
    input  wire [23:0] rgb_in,
    input  wire [7:0]  alpha,
    input  wire        dissolve_active,
    input  wire [4:0]  ramp,
    input  wire [3:0]  seed,
    output wire        hs_out,
    output wire        vs_out,
    output wire        de_out,
    output wire [23:0] rgb_out
);
    // lvl = ramp × 16 / FADE_FRAMES（FADE_FRAMES 整除 256，≤ 16 时是左移，常数折叠）
    localparam integer FF     = (FADE_FRAMES == 0) ? 1 : FADE_FRAMES;
    localparam integer LV_MUL = (FF >= 16) ? 1 : 16 / FF;

    // synthesis translate_off
    initial if (FADE_FRAMES > 16)
        $display("ERROR: video_transition_pixel FADE_FRAMES=%0d 方块溶解要求 <= 16", FADE_FRAMES);
    // synthesis translate_on

    // ---------------- 控制量：每拍寄存（只在消隐期变化）----------------
    reg [7:0] alpha_q;
    reg       alpha_full_q;
    reg       dis_q;
    reg [4:0] lvl_q;             // 0..16：序号 < lvl 的块显示
    reg [3:0] seed_q;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            alpha_q      <= 8'd255;
            alpha_full_q <= 1'b1;
            dis_q        <= 1'b0;
        end else begin
            alpha_q      <= alpha;
            alpha_full_q <= (alpha == 8'd255) || (FADE_FRAMES == 0);
            dis_q        <= dissolve_active && (FADE_FRAMES != 0);
        end
    end
    // L7：纯数据寄存器不接复位（只在 dis_q=1 时有意义，dis_q 复位为 0）
    always @(posedge clk) begin
        lvl_q  <= ramp * LV_MUL;
        seed_q <= seed;
    end

    // ---------------- Alpha 乘法（/256 用右移）----------------
    wire [15:0] mul_r = rgb_in[23:16] * alpha_q;
    wire [15:0] mul_g = rgb_in[15:8]  * alpha_q;
    wire [15:0] mul_b = rgb_in[7:0]   * alpha_q;
    wire [23:0] faded_rgb = alpha_full_q ? rgb_in : {mul_r[15:8], mul_g[15:8], mul_b[15:8]};

    // ---------------- 方块序号 ----------------
    function [3:0] sbox;                      // PRESENT S 盒：C 5 6 B 9 0 A D 3 E F 8 4 7 1 2
        input [3:0] v;
        case (v)
            4'h0: sbox = 4'hC; 4'h1: sbox = 4'h5; 4'h2: sbox = 4'h6; 4'h3: sbox = 4'hB;
            4'h4: sbox = 4'h9; 4'h5: sbox = 4'h0; 4'h6: sbox = 4'hA; 4'h7: sbox = 4'hD;
            4'h8: sbox = 4'h3; 4'h9: sbox = 4'hE; 4'hA: sbox = 4'hF; 4'hB: sbox = 4'h8;
            4'hC: sbox = 4'h4; 4'hD: sbox = 4'h7; 4'hE: sbox = 4'h1; default: sbox = 4'h2;
        endcase
    endfunction

    wire       x5 = x_in[5], x6 = x_in[6], x7 = x_in[7], x8 = x_in[8], x9 = x_in[9];
    wire       y5 = y_in[5], y6 = y_in[6], y7 = y_in[7], y8 = y_in[8], y9 = y_in[9];
    wire [3:0] mix1 = {y5 ^ y6 ^ x9 ^ x7, x6 ^ y7 ^ x8 ^ x5, x7 ^ y8 ^ x8 ^ y6, x9 ^ x5 ^ x7};
    wire [3:0] mix2 = {y7 ^ x7, y9 ^ x7, x5 ^ x7, y5 ^ x7} ^ seed_q;
    wire [3:0] blk_idx  = sbox(sbox(mix1) ^ mix2);
    wire       blk_show = ({1'b0, blk_idx} < lvl_q);

    wire [23:0] trans_rgb = (dis_q && !blk_show) ? 24'h000000 : faded_rgb;

    // ---------------- 输出：组合直通或整组寄存一级 ----------------
    generate
        if (PIPELINE != 0) begin : g_pipe
            reg        hs_q, vs_q, de_q;
            reg [23:0] rgb_q;
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    hs_q  <= 1'b1;
                    vs_q  <= 1'b1;
                    de_q  <= 1'b0;
                end else begin
                    hs_q  <= hs_in;
                    vs_q  <= vs_in;
                    de_q  <= de_in;
                end
            end
            // L7：纯数据寄存器不接复位（下游/本级已按 de、锁定门控，复位期间时钟照常冲成 0）
            always @(posedge clk)
                rgb_q <= trans_rgb;
            assign hs_out  = hs_q;
            assign vs_out  = vs_q;
            assign de_out  = de_q;
            assign rgb_out = rgb_q;
        end else begin : g_comb
            assign hs_out  = hs_in;
            assign vs_out  = vs_in;
            assign de_out  = de_in;
            assign rgb_out = trans_rgb;
        end
    endgenerate
endmodule
