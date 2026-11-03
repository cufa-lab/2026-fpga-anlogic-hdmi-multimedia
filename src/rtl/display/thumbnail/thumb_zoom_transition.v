`timescale 1ns/1ps

module thumb_zoom_transition #(
    parameter integer X_OFFSET = 160,
    parameter integer Y_OFFSET = 45,
    parameter [7:0]   A_MID    = 8'd64,
    parameter integer STEPS    = 12,       
    parameter integer FULL_GRID = 0        
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        frame_start,
    input  wire        display_valid,
    input  wire        sw_thumb,          // 已同步的 SW4 电平
    input  wire [1:0]  focus,             // 当前图片所在格子（active_image_idx % 4）
    output reg         thumb_display,     // 1 = 总览网格（reader 读第 5 区）
    output reg         zoom_active,       // 1 = 动画中（alpha / 裁剪有效）
    output reg  [7:0]  zoom_alpha,
    output reg  [11:0] clip_x0, clip_x1,  // raw 计数坐标（已加 X/Y_OFFSET），x0 ≤ x < x1
    output reg  [11:0] clip_y0, clip_y1
);
    localparam [2:0] Z_FULL = 3'd0, Z_SHRINK_OUT = 3'd1, Z_SHRINK_IN = 3'd2,
                     Z_THUMB = 3'd3, Z_EXPAND_OUT = 3'd4, Z_EXPAND_IN = 3'd5;

    reg [2:0] zst;
    reg [3:0] k;
    reg       sw_q;          // 上一帧采样
    reg       sw_stable;     // 去抖后
    reg [1:0] fz;            // 动画焦点

    wire       out_phase = (zst == Z_SHRINK_OUT) || (zst == Z_EXPAND_OUT);
    wire       idle      = (zst == Z_FULL) || (zst == Z_THUMB);
    wire [3:0] j         = idle ? 4'd12 : out_phase ? 4'd12 - k : k;

    function [11:0] fx0_full(input [3:0] j);
        case (j)
            4'd0: fx0_full = X_OFFSET + 320;
            4'd1: fx0_full = X_OFFSET + 246;
            4'd2: fx0_full = X_OFFSET + 185;
            4'd3: fx0_full = X_OFFSET + 135;
            4'd4: fx0_full = X_OFFSET + 95;
            4'd5: fx0_full = X_OFFSET + 63;
            4'd6: fx0_full = X_OFFSET + 40;
            4'd7: fx0_full = X_OFFSET + 23;
            4'd8: fx0_full = X_OFFSET + 11;
            4'd9: fx0_full = X_OFFSET + 5;
            4'd10: fx0_full = X_OFFSET + 1;
            default: fx0_full = X_OFFSET + 0;
        endcase
    endfunction
    function [11:0] fx1_full(input [3:0] j);
        case (j)
            4'd0: fx1_full = X_OFFSET + 320;
            4'd1: fx1_full = X_OFFSET + 394;
            4'd2: fx1_full = X_OFFSET + 455;
            4'd3: fx1_full = X_OFFSET + 505;
            4'd4: fx1_full = X_OFFSET + 545;
            4'd5: fx1_full = X_OFFSET + 577;
            4'd6: fx1_full = X_OFFSET + 600;
            4'd7: fx1_full = X_OFFSET + 617;
            4'd8: fx1_full = X_OFFSET + 629;
            4'd9: fx1_full = X_OFFSET + 635;
            4'd10: fx1_full = X_OFFSET + 639;
            default: fx1_full = X_OFFSET + 640;
        endcase
    endfunction
    function [11:0] fy0_full(input [3:0] j);
        case (j)
            4'd0: fy0_full = Y_OFFSET + 240;
            4'd1: fy0_full = Y_OFFSET + 184;
            4'd2: fy0_full = Y_OFFSET + 138;
            4'd3: fy0_full = Y_OFFSET + 101;
            4'd4: fy0_full = Y_OFFSET + 71;
            4'd5: fy0_full = Y_OFFSET + 47;
            4'd6: fy0_full = Y_OFFSET + 30;
            4'd7: fy0_full = Y_OFFSET + 17;
            4'd8: fy0_full = Y_OFFSET + 8;
            4'd9: fy0_full = Y_OFFSET + 3;
            4'd10: fy0_full = Y_OFFSET + 0;
            default: fy0_full = Y_OFFSET + 0;
        endcase
    endfunction
    function [11:0] fy1_full(input [3:0] j);
        case (j)
            4'd0: fy1_full = Y_OFFSET + 240;
            4'd1: fy1_full = Y_OFFSET + 296;
            4'd2: fy1_full = Y_OFFSET + 342;
            4'd3: fy1_full = Y_OFFSET + 379;
            4'd4: fy1_full = Y_OFFSET + 409;
            4'd5: fy1_full = Y_OFFSET + 433;
            4'd6: fy1_full = Y_OFFSET + 450;
            4'd7: fy1_full = Y_OFFSET + 463;
            4'd8: fy1_full = Y_OFFSET + 472;
            4'd9: fy1_full = Y_OFFSET + 477;
            4'd10: fy1_full = Y_OFFSET + 480;
            default: fy1_full = Y_OFFSET + 480;
        endcase
    endfunction
    function [11:0] fx0_l12(input [3:0] j);
        case (j)
            4'd0: fx0_l12 = X_OFFSET + 160;
            4'd1: fx0_l12 = X_OFFSET + 123;
            4'd2: fx0_l12 = X_OFFSET + 92;
            4'd3: fx0_l12 = X_OFFSET + 67;
            4'd4: fx0_l12 = X_OFFSET + 47;
            4'd5: fx0_l12 = X_OFFSET + 31;
            4'd6: fx0_l12 = X_OFFSET + 20;
            4'd7: fx0_l12 = X_OFFSET + 11;
            4'd8: fx0_l12 = X_OFFSET + 5;
            4'd9: fx0_l12 = X_OFFSET + 2;
            4'd10: fx0_l12 = X_OFFSET + 0;
            default: fx0_l12 = X_OFFSET + 0;
        endcase
    endfunction
    function [11:0] fx1_l12a(input [3:0] j);
        case (j)
            4'd0: fx1_l12a = X_OFFSET + 320;
            4'd1: fx1_l12a = X_OFFSET + 394;
            4'd2: fx1_l12a = X_OFFSET + 455;
            4'd3: fx1_l12a = X_OFFSET + 505;
            4'd4: fx1_l12a = X_OFFSET + 545;
            4'd5: fx1_l12a = X_OFFSET + 577;
            4'd6: fx1_l12a = X_OFFSET + 600;
            4'd7: fx1_l12a = X_OFFSET + 617;
            4'd8: fx1_l12a = X_OFFSET + 629;
            4'd9: fx1_l12a = X_OFFSET + 635;
            4'd10: fx1_l12a = X_OFFSET + 639;
            default: fx1_l12a = X_OFFSET + 640;
        endcase
    endfunction
    function [11:0] fx1_l12b(input [3:0] j);
        case (j)
            4'd0: fx1_l12b = X_OFFSET + 160;
            4'd1: fx1_l12b = X_OFFSET + 271;
            4'd2: fx1_l12b = X_OFFSET + 363;
            4'd3: fx1_l12b = X_OFFSET + 438;
            4'd4: fx1_l12b = X_OFFSET + 498;
            4'd5: fx1_l12b = X_OFFSET + 545;
            4'd6: fx1_l12b = X_OFFSET + 580;
            4'd7: fx1_l12b = X_OFFSET + 605;
            4'd8: fx1_l12b = X_OFFSET + 624;
            4'd9: fx1_l12b = X_OFFSET + 633;
            4'd10: fx1_l12b = X_OFFSET + 639;
            default: fx1_l12b = X_OFFSET + 640;
        endcase
    endfunction
    function [11:0] fy0_l12(input [3:0] j);
        case (j)
            4'd0: fy0_l12 = Y_OFFSET + 120;
            4'd1: fy0_l12 = Y_OFFSET + 92;
            4'd2: fy0_l12 = Y_OFFSET + 69;
            4'd3: fy0_l12 = Y_OFFSET + 50;
            4'd4: fy0_l12 = Y_OFFSET + 35;
            4'd5: fy0_l12 = Y_OFFSET + 23;
            4'd6: fy0_l12 = Y_OFFSET + 15;
            4'd7: fy0_l12 = Y_OFFSET + 8;
            4'd8: fy0_l12 = Y_OFFSET + 4;
            4'd9: fy0_l12 = Y_OFFSET + 1;
            4'd10: fy0_l12 = Y_OFFSET + 0;
            default: fy0_l12 = Y_OFFSET + 0;
        endcase
    endfunction
    function [11:0] fy1_l12a(input [3:0] j);
        case (j)
            4'd0: fy1_l12a = Y_OFFSET + 240;
            4'd1: fy1_l12a = Y_OFFSET + 296;
            4'd2: fy1_l12a = Y_OFFSET + 342;
            4'd3: fy1_l12a = Y_OFFSET + 379;
            4'd4: fy1_l12a = Y_OFFSET + 409;
            4'd5: fy1_l12a = Y_OFFSET + 433;
            4'd6: fy1_l12a = Y_OFFSET + 450;
            4'd7: fy1_l12a = Y_OFFSET + 463;
            4'd8: fy1_l12a = Y_OFFSET + 472;
            4'd9: fy1_l12a = Y_OFFSET + 477;
            4'd10: fy1_l12a = Y_OFFSET + 480;
            default: fy1_l12a = Y_OFFSET + 480;
        endcase
    endfunction
    function [11:0] fy1_l12b(input [3:0] j);
        case (j)
            4'd0: fy1_l12b = Y_OFFSET + 120;
            4'd1: fy1_l12b = Y_OFFSET + 203;
            4'd2: fy1_l12b = Y_OFFSET + 272;
            4'd3: fy1_l12b = Y_OFFSET + 329;
            4'd4: fy1_l12b = Y_OFFSET + 374;
            4'd5: fy1_l12b = Y_OFFSET + 409;
            4'd6: fy1_l12b = Y_OFFSET + 435;
            4'd7: fy1_l12b = Y_OFFSET + 454;
            4'd8: fy1_l12b = Y_OFFSET + 468;
            4'd9: fy1_l12b = Y_OFFSET + 475;
            4'd10: fy1_l12b = Y_OFFSET + 479;
            default: fy1_l12b = Y_OFFSET + 480;
        endcase
    endfunction
    function [7:0] f_alpha(input [3:0] j);
        case (j)
            4'd0: f_alpha = 8'd64;
            4'd1: f_alpha = 8'd109;
            4'd2: f_alpha = 8'd145;
            4'd3: f_alpha = 8'd175;
            4'd4: f_alpha = 8'd199;
            4'd5: f_alpha = 8'd217;
            4'd6: f_alpha = 8'd232;
            4'd7: f_alpha = 8'd241;
            4'd8: f_alpha = 8'd249;
            4'd9: f_alpha = 8'd253;
            4'd10: f_alpha = 8'd255;
            default: f_alpha = 8'd255;
        endcase
    endfunction

    // FULL_GRID=1：焦点格 320×240，左/右格 x = 0..320 / 320..640，上/下格 y = 0..240 / 240..480
    wire [11:0] x0_now = !fz[0] ? X_OFFSET : FULL_GRID ? fx0_full(j) : fx0_l12(j);
    wire [11:0] x1_now = FULL_GRID ? (fz[0] ? X_OFFSET + 640 : fx1_full(j)) : (fz[0] ? fx1_l12a(j) : fx1_l12b(j));
    wire [11:0] y0_now = !fz[1] ? Y_OFFSET : FULL_GRID ? fy0_full(j) : fy0_l12(j);
    wire [11:0] y1_now = FULL_GRID ? (fz[1] ? Y_OFFSET + 480 : fy1_full(j)) : (fz[1] ? fy1_l12a(j) : fy1_l12b(j));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            zst <= Z_FULL; k <= 4'd0; sw_q <= 1'b0; sw_stable <= 1'b0; fz <= 2'd0;
            thumb_display <= 1'b0; zoom_active <= 1'b0; zoom_alpha <= 8'd255;
            clip_x0 <= X_OFFSET; clip_x1 <= X_OFFSET + 640;
            clip_y0 <= Y_OFFSET; clip_y1 <= Y_OFFSET + 480;
        end else if (frame_start) begin
            sw_q <= sw_thumb;
            if (sw_q == sw_thumb) sw_stable <= sw_thumb;

            // ---- 本帧输出（用本帧 zst / k）----
            zoom_active <= !idle;
            zoom_alpha  <= f_alpha(j);                         // 255 - (191s >> 8)，空闲 s=0 → 255
            clip_x0 <= x0_now;  clip_x1 <= x1_now;
            clip_y0 <= y0_now;  clip_y1 <= y1_now;

            // ---- 状态推进 ----
            case (zst)
                Z_FULL: begin
                    thumb_display <= 1'b0;
                    if (display_valid && sw_stable) begin
                        zst <= Z_SHRINK_OUT; k <= 4'd1; fz <= focus;
                    end
                end
                Z_SHRINK_OUT: begin
                    if (k == STEPS) begin
                        zst <= Z_SHRINK_IN; k <= 4'd1;
                        thumb_display <= 1'b1;        // 下一帧起读总览
                    end else k <= k + 1'b1;
                end
                Z_SHRINK_IN: begin
                    if (k == STEPS) zst <= Z_THUMB;
                    else k <= k + 1'b1;
                end
                Z_THUMB: begin
                    thumb_display <= 1'b1;
                    if (!sw_stable || !display_valid) begin
                        zst <= Z_EXPAND_OUT; k <= 4'd1; fz <= focus;
                    end
                end
                Z_EXPAND_OUT: begin
                    if (k == STEPS) begin
                        zst <= Z_EXPAND_IN; k <= 4'd1;   // s=256 这一帧仍显示焦点格缩略图
                    end else k <= k + 1'b1;
                end
                Z_EXPAND_IN: begin
                    thumb_display <= 1'b0;             // 从 s 开始变小的第一帧起读全屏图
                    if (k == STEPS) zst <= Z_FULL;
                    else k <= k + 1'b1;
                end
                default: zst <= Z_FULL;
            endcase
        end
    end
endmodule
