`timescale 1ns/1ps

module timestamp_osd #(
    parameter integer OSD_X  = 500,
    parameter integer OSD_Y  = 8,
    parameter integer CHAR_W = 8,                  // 8x16 像素字符
    parameter integer CHAR_H = 16,
    parameter integer CLK_HZ = 25_000_000          // pix_clk 频率
) (
    input  wire        pix_clk,
    input  wire        pix_rst_n,
    input  wire        enable,
    input  wire        frame_start,
    input  wire        de,
    input  wire [9:0]  pixel_x,
    input  wire [9:0]  pixel_y,
    input  wire [23:0] rgb_in,
    output wire [23:0] rgb_out,
    output wire        ts_active,                  // 5 字符区域（整块，作底板用）
    output reg         ts_pixel                    // 数字/冒号笔画像素
);
    localparam integer SEC_W = (CLK_HZ > 1) ? $clog2(CLK_HZ) : 1;

    // ---------------- 计时：CLK_HZ 拍 = 1 秒，秒进位推迟到下一个 frame_start ----------------
    reg [SEC_W-1:0] tick_cnt;
    reg             sec_pending;
    reg [3:0]       ss_ones, ss_tens, mm_ones, mm_tens;
    wire            sec_tick = (tick_cnt == CLK_HZ - 1);

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            tick_cnt    <= {SEC_W{1'b0}};
            sec_pending <= 1'b0;
            ss_ones     <= 4'd0;
            ss_tens     <= 4'd0;
            mm_ones     <= 4'd0;
            mm_tens     <= 4'd0;
        end else begin
            tick_cnt <= sec_tick ? {SEC_W{1'b0}} : tick_cnt + 1'b1;
            if (frame_start && (sec_pending || sec_tick)) begin
                sec_pending <= 1'b0;
                if (ss_ones != 4'd9) begin
                    ss_ones <= ss_ones + 4'd1;
                end else begin
                    ss_ones <= 4'd0;
                    if (ss_tens != 4'd5) begin
                        ss_tens <= ss_tens + 4'd1;
                    end else begin
                        ss_tens <= 4'd0;
                        if (mm_ones != 4'd9) begin
                            mm_ones <= mm_ones + 4'd1;
                        end else begin
                            mm_ones <= 4'd0;
                            mm_tens <= (mm_tens == 4'd9) ? 4'd0 : mm_tens + 4'd1;
                        end
                    end
                end
            end else if (sec_tick) begin
                sec_pending <= 1'b1;
            end
        end
    end

    // ---------------- 渲染（与课程原版相同） ----------------
    wire [9:0] dx = pixel_x - OSD_X;
    wire [9:0] dy = pixel_y - OSD_Y;

    wire in_region = enable && de &&
                     pixel_x >= OSD_X && pixel_x < OSD_X + 5 * CHAR_W &&
                     pixel_y >= OSD_Y && pixel_y < OSD_Y + CHAR_H;

    assign ts_active = in_region;

    // 4x6 点阵（纵向放大 2 倍，有效 4x12，字符格内留空）
    function font_pixel;
        input [3:0] digit;
        input [3:0] col;   // 0..3
        input [3:0] row;   // 0..5（放大前的逻辑行）
        begin
            case (digit)
                4'd0: font_pixel = (row==0 && (col==1||col==2)) ||
                                   (row==1 && (col==0||col==3)) ||
                                   (row==2 && (col==0||col==3)) ||
                                   (row==3 && (col==0||col==3)) ||
                                   (row==4 && (col==0||col==3)) ||
                                   (row==5 && (col==1||col==2));
                4'd1: font_pixel = (col==2) ||
                                   (row==0 && col==1) ||
                                   (row==2 && (col==1||col==3)) ||
                                   (row==5 && (col==1||col==2||col==3));
                4'd2: font_pixel = (row==0 && (col==1||col==2)) ||
                                   (row==1 && col==3) ||
                                   (row==2 && (col==1||col==2)) ||
                                   (row==3 && col==1) ||
                                   (row==4 && col==0) ||
                                   (row==5 && (col==1||col==2||col==3));
                4'd3: font_pixel = (row==0 && (col==1||col==2)) ||
                                   (row==1 && col==3) ||
                                   (row==2 && (col==1||col==2)) ||
                                   (row==3 && col==3) ||
                                   (row==4 && col==3) ||
                                   (row==5 && (col==1||col==2));
                4'd4: font_pixel = (row==0 && (col==0||col==3)) ||
                                   (row==1 && (col==0||col==3)) ||
                                   (row==2 && (col==0||col==1||col==2||col==3)) ||
                                   (row==3 && col==3) ||
                                   (row==4 && col==3) ||
                                   (row==5 && col==3);
                4'd5: font_pixel = (row==0 && (col==0||col==1||col==2||col==3)) ||
                                   (row==1 && col==0) ||
                                   (row==2 && (col==0||col==1||col==2)) ||
                                   (row==3 && col==3) ||
                                   (row==4 && col==3) ||
                                   (row==5 && (col==1||col==2));
                4'd6: font_pixel = (row==0 && (col==1||col==2)) ||
                                   (row==1 && col==0) ||
                                   (row==2 && (col==0||col==1||col==2)) ||
                                   (row==3 && (col==0||col==3)) ||
                                   (row==4 && (col==0||col==3)) ||
                                   (row==5 && (col==1||col==2));
                4'd7: font_pixel = (row==0 && (col==0||col==1||col==2||col==3)) ||
                                   (row==1 && col==3) ||
                                   (row==2 && col==2) ||
                                   (row==3 && col==2) ||
                                   (row==4 && col==2) ||
                                   (row==5 && col==2);
                4'd8: font_pixel = (row==0 && (col==1||col==2)) ||
                                   (row==1 && (col==0||col==3)) ||
                                   (row==2 && (col==1||col==2)) ||
                                   (row==3 && (col==0||col==3)) ||
                                   (row==4 && (col==0||col==3)) ||
                                   (row==5 && (col==1||col==2));
                4'd9: font_pixel = (row==0 && (col==1||col==2)) ||
                                   (row==1 && (col==0||col==3)) ||
                                   (row==2 && (col==1||col==2||col==3)) ||
                                   (row==3 && col==3) ||
                                   (row==4 && col==3) ||
                                   (row==5 && (col==1||col==2));
                default: font_pixel = 1'b0;
            endcase
        end
    endfunction

    // MM 与 SS 之间的冒号
    function colon_pixel;
        input [3:0] col;
        input [3:0] row;
        begin
            colon_pixel = (col >= 1 && col <= 2) &&
                          (row == 3'd1 || row == 3'd4);
        end
    endfunction

    wire [3:0] fcol = dx % CHAR_W;
    wire [3:0] frow = (dy % CHAR_H) / 2;
    always @* begin
        ts_pixel = 1'b0;
        if (in_region && dx < 5 * CHAR_W && dy < CHAR_H) begin
            if (frow < 6) begin
                case (dx / CHAR_W)
                3'd0: ts_pixel = font_pixel(mm_tens, fcol, frow);
                3'd1: ts_pixel = font_pixel(mm_ones, fcol, frow);
                3'd2: ts_pixel = colon_pixel(fcol, frow);
                3'd3: ts_pixel = font_pixel(ss_tens, fcol, frow);
                3'd4: ts_pixel = font_pixel(ss_ones, fcol, frow);
                default: ts_pixel = 1'b0;
                endcase
            end
        end
    end

    assign rgb_out = ts_pixel ? 24'hE0F0FF : rgb_in;
endmodule
