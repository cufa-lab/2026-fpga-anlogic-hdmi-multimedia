`timescale 1ns/1ps

module promax_text_overlay #(
    parameter integer PARAM_X    = 8,
    parameter integer PARAM_Y    = 40,
    parameter integer SUBTITLE_Y = 72,
    parameter integer H_ACTIVE   = 640,
    parameter integer SCROLL_DIV = 3        // 每 SCROLL_DIV 帧滚 1 像素
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        frame_start,         // 低有效 vsync 下降沿
    input  wire        de,
    input  wire [23:0] rgb_in,
    input  wire [9:0]  pixel_x,             // 本拍像素列号（de 期间有效）
    input  wire [9:0]  pixel_y,             // 本拍像素行号
    input  wire        enable,
    input  wire        subtitle_enable,
    input  wire [95:0] subtitle_text,       // 第 i 个字符在 [i*8 +: 8]，最多 12 个
    input  wire [5:0]  subtitle_length,
    input  wire [7:0]  brightness,          // 0..255
    input  wire [7:0]  volume,
    input  wire [8:0]  period_tenths,       // 轮播周期，单位 0.1s
    input  wire        auto_enabled,
    input  wire        res_valid,           // 1 = res_width/res_height 是当前画面的图片分辨率
    input  wire [11:0] res_width,           // 1..4095
    input  wire [11:0] res_height,
    output wire [23:0] rgb_out,             
    output wire        overlay_active,
    output wire        param_pixel,         // 参数条文字像素
    output wire        subtitle_pixel,      // 字幕文字像素
    output wire        subtitle_band_pixel, // 字幕底色带像素
    output reg         param_changed        // 本帧锁存的参数和上一帧不同（frame_start 后一拍，单拍）
);
    localparam integer CELL_W          = 12;
    localparam integer CELL_H          = 16;
    localparam integer SUBTITLE_CELL_W = 18;
    localparam integer SUBTITLE_CELL_H = 18;
    localparam integer PARAM_CHARS     = 34;    // 32 → 34（分辨率最长 "1920X1080" 加前导空格）
    localparam integer MAX_TEXT        = 12;
    localparam integer SCROLL_CYCLE    = H_ACTIVE + 128;
    localparam integer SUBTITLE_BAND_BOTTOM = SUBTITLE_Y + SUBTITLE_CELL_H + 2;
    localparam [5:0]   SCROLL_LAST_CELL = (SCROLL_CYCLE - 1) / SUBTITLE_CELL_W;
    localparam [4:0]   SCROLL_LAST_COL  = (SCROLL_CYCLE - 1) % SUBTITLE_CELL_W;

    generate
        if (!((PARAM_Y + CELL_H <= SUBTITLE_Y + 2) || (SUBTITLE_Y + 16 <= PARAM_Y)) || PARAM_X < 1) begin : g_bad_layout
            promax_text_overlay_param_rows_overlap_subtitle_rows u_layout_error ();
        end
    endgenerate

    // ---------------- 帧锁存（一帧内参数不变，避免撕裂）+ 滚动位置 ----------------
    reg [7:0] bright_frame;
    reg [7:0] volume_frame;
    reg [8:0] period_frame;
    reg       auto_frame;
    reg [5:0] scroll_cell;                // 滚动位置 scroll_pos 的 /18 与 %18
    reg [4:0] scroll_col;
    reg [3:0] scroll_div;             // SCROLL_DIV <= 16

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bright_frame          <= 8'd255;
            volume_frame          <= 8'd255;
            period_frame          <= 9'd50;
            auto_frame            <= 1'b0;
            scroll_cell           <= 6'd0;
            scroll_col            <= 5'd0;
            scroll_div            <= 4'd0;
            param_changed         <= 1'b0;
        end else begin
            param_changed <= 1'b0;
            if (frame_start) begin
                bright_frame          <= brightness;
                volume_frame          <= volume;
                period_frame          <= period_tenths;
                auto_frame            <= auto_enabled;
                param_changed <= (brightness != bright_frame) || (volume != volume_frame) ||
                                 (period_tenths != period_frame) || (auto_enabled != auto_frame);
                if (scroll_div == SCROLL_DIV - 1) begin
                    scroll_div <= 4'd0;
                    if (scroll_cell == SCROLL_LAST_CELL && scroll_col == SCROLL_LAST_COL) begin
                        scroll_cell <= 6'd0;
                        scroll_col  <= 5'd0;
                    end else if (scroll_col == SUBTITLE_CELL_W - 1) begin
                        scroll_cell <= scroll_cell + 6'd1;
                        scroll_col  <= 5'd0;
                    end else begin
                        scroll_col  <= scroll_col + 5'd1;
                    end
                end else begin
                    scroll_div <= scroll_div + 4'd1;
                end
            end
        end
    end

    // ---------------- 二进制转 BCD（逻辑优化 L4：逐拍移位加 3，与分辨率共用 cv_cnt 序列） ----------------
    // 读帧锁存值的第 cv_cnt-4 位移位加 3，每个数只用 2 个加 3 单元（9 位数移位前百位 <= 2，不需要加 3）。
    // 帧锁存值在 frame_start 那拍更新，cv_cnt = 12 时已稳定；转换在场消隐期内完成（参数条在第 PARAM_Y 行）。
    // 复位值 = 帧锁存复位值 255 / 255 / 50 的 BCD，第一个 frame_start 之前显示与原版相同。
    reg [11:0] bright_bcd;
    reg [11:0] volume_bcd;
    reg [11:0] period_bcd;

    function [11:0] add3_low2;
        input [11:0] b;
        begin
            add3_low2[11:8] = b[11:8];
            add3_low2[7:4]  = (b[7:4] >= 4'd5) ? b[7:4] + 4'd3 : b[7:4];
            add3_low2[3:0]  = (b[3:0] >= 4'd5) ? b[3:0] + 4'd3 : b[3:0];
        end
    endfunction

    wire [11:0] bri_adj = add3_low2(bright_bcd);
    wire [11:0] vol_adj = add3_low2(volume_bcd);
    wire [11:0] prd_adj = add3_low2(period_bcd);

    function [7:0] digit_char;
        input [3:0] d;
        begin
            digit_char = 8'h30 + {4'd0, d};
        end
    endfunction

    // ---------------- 分辨率：逐拍移位加 3（frame_start 锁存，场消隐期内完成） ----------------
    // 数字码：0..9 = 数字，4'hF = 空白（高度左对齐时补在右边）；无效用 1 位 res_v_frame，显示 '-'
    // 12 位数的千位最终 <= 4，移位前 <= 2，所以只有低 3 位需要加 3 单元
    // 输入位直接按 cv_cnt 索引读（不存移位副本）：res_width/res_height 只在帧边界附近变化，
    // frame_start 后 15 拍内不变（见 picture_res_info 的约定）
    reg [15:0] res_w_bcd;
    reg [15:0] res_h_bcd;
    reg        res_v_frame;
    reg [3:0]  cv_cnt;                      // 15..4 移位加 3（读第 cv_cnt-4 位），3..1 高度左对齐，0 空闲
    wire [3:0] cv_bit = cv_cnt - 4'd4;

    function [15:0] add3_low3;
        input [15:0] b;
        begin
            add3_low3[15:12] = b[15:12];
            add3_low3[11:8]  = (b[11:8] >= 4'd5) ? b[11:8] + 4'd3 : b[11:8];
            add3_low3[7:4]   = (b[7:4]  >= 4'd5) ? b[7:4]  + 4'd3 : b[7:4];
            add3_low3[3:0]   = (b[3:0]  >= 4'd5) ? b[3:0]  + 4'd3 : b[3:0];
        end
    endfunction

    wire [15:0] w_adj = add3_low3(res_w_bcd);
    wire [15:0] h_adj = add3_low3(res_h_bcd);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            res_w_bcd   <= 16'h0000;
            res_h_bcd   <= 16'h0000;
            res_v_frame <= 1'b0;
            cv_cnt      <= 4'd0;
        end else if (frame_start) begin
            res_w_bcd   <= 16'h0000;
            res_h_bcd   <= 16'h0000;
            res_v_frame <= res_valid;
            cv_cnt      <= 4'd15;
        end else if (cv_cnt[3] || cv_cnt[2]) begin            // cv_cnt >= 4
            res_w_bcd <= {w_adj[14:0], res_width[cv_bit]};
            res_h_bcd <= {h_adj[14:0], res_height[cv_bit]};
            cv_cnt    <= cv_cnt - 4'd1;
        end else if (cv_cnt != 4'd0) begin
            if (res_h_bcd[15:12] == 4'd0)
                res_h_bcd <= {res_h_bcd[11:0], 4'hF};
            cv_cnt    <= cv_cnt - 4'd1;
        end
    end

    // 亮度 / 音量 / 周期 BCD，cv_cnt = 12..4 读第 cv_bit（8..0）位
    wire [8:0] bri9 = {1'b0, bright_frame};
    wire [8:0] vol9 = {1'b0, volume_frame};
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bright_bcd <= 12'h255;
            volume_bcd <= 12'h255;
            period_bcd <= 12'h050;
        end else if (frame_start) begin
            bright_bcd <= 12'h000;
            volume_bcd <= 12'h000;
            period_bcd <= 12'h000;
        end else if ((cv_cnt[3] || cv_cnt[2]) && cv_cnt <= 4'd12) begin
            bright_bcd <= {bri_adj[10:0], bri9[cv_bit]};
            volume_bcd <= {vol_adj[10:0], vol9[cv_bit]};
            period_bcd <= {prd_adj[10:0], period_frame[cv_bit]};
        end
    end

    // 无效 → '-'；F → 空白（高度右侧补位）
    function [7:0] res_char;
        input       v;
        input [3:0] d;
        begin
            if (!v)
                res_char = "-";
            else if (d == 4'hF)
                res_char = " ";
            else
                res_char = 8'h30 + {4'd0, d};
        end
    endfunction

    // 宽右对齐：高位为 0 的前导数字显示空白（个位总显示）；无效时不消隐（整段 '-'）
    wire w_lz3 = res_v_frame && (res_w_bcd[15:12] == 4'd0);
    wire w_lz2 = w_lz3 && (res_w_bcd[11:8] == 4'd0);
    wire w_lz1 = w_lz2 && (res_w_bcd[7:4]  == 4'd0);

    // 参数条字符：BRT100 VOL128 P02.5S M:A 1920X1080
    function [7:0] param_char;
        input [5:0] index;
        begin
            case (index)
                6'd0:  param_char = "B";
                6'd1:  param_char = "R";
                6'd2:  param_char = "T";
                6'd3:  param_char = digit_char(bright_bcd[11:8]);
                6'd4:  param_char = digit_char(bright_bcd[7:4]);
                6'd5:  param_char = digit_char(bright_bcd[3:0]);
                6'd6:  param_char = " ";
                6'd7:  param_char = "V";
                6'd8:  param_char = "O";
                6'd9:  param_char = "L";
                6'd10: param_char = digit_char(volume_bcd[11:8]);
                6'd11: param_char = digit_char(volume_bcd[7:4]);
                6'd12: param_char = digit_char(volume_bcd[3:0]);
                6'd13: param_char = " ";
                6'd14: param_char = "P";
                6'd15: param_char = digit_char(period_bcd[11:8]);
                6'd16: param_char = digit_char(period_bcd[7:4]);
                6'd17: param_char = ".";
                6'd18: param_char = digit_char(period_bcd[3:0]);
                6'd19: param_char = "S";
                6'd20: param_char = " ";
                6'd21: param_char = "M";
                6'd22: param_char = ":";
                6'd23: param_char = auto_frame ? "A" : "M";
                6'd25: param_char = w_lz3 ? " " : res_char(res_v_frame, res_w_bcd[15:12]);
                6'd26: param_char = w_lz2 ? " " : res_char(res_v_frame, res_w_bcd[11:8]);
                6'd27: param_char = w_lz1 ? " " : res_char(res_v_frame, res_w_bcd[7:4]);
                6'd28: param_char = res_char(res_v_frame, res_w_bcd[3:0]);
                6'd29: param_char = "X";
                6'd30: param_char = res_char(res_v_frame, res_h_bcd[15:12]);
                6'd31: param_char = res_char(res_v_frame, res_h_bcd[11:8]);
                6'd32: param_char = res_char(res_v_frame, res_h_bcd[7:4]);
                6'd33: param_char = res_char(res_v_frame, res_h_bcd[3:0]);
                default: param_char = " ";               // 24 = 空格；34 以后在框外
            endcase
        end
    endfunction

    // ASCII 0x20..0x5F → 字库字符码 0..63；其余（小写、控制字符、>=0x80）按空白
    function [5:0] font_code;
        input [7:0] c;
        begin
            if (!c[7] && (c[6] ^ c[5]))
                font_code = {c[6], c[4:0]};
            else
                font_code = 6'd0;
        end
    endfunction

    // ---------------- 沿扫描行的格计数器（描述"本拍像素"） ----------------
    // 参数条：pixel_x == PARAM_X-1 时下一拍进入第 0 格第 0 列
    reg  [5:0] p_cell;
    reg  [3:0] p_col;
    reg  [5:0] p_cell_n;
    reg  [3:0] p_col_n;
    // 字幕流位置 s = (pixel_x + scroll_pos) mod SCROLL_CYCLE 的 /18、%18：消隐期装入滚动位置，
    // de 期间逐像素 +1，到 SCROLL_CYCLE-1 回 0
    reg  [5:0] s_cell;
    reg  [4:0] s_col;
    reg  [5:0] s_cell_n;
    reg  [4:0] s_col_n;

    always @* begin
        if (de && pixel_x == PARAM_X - 1) begin
            p_cell_n = 6'd0;
            p_col_n  = 4'd0;
        end else if (de && p_col == CELL_W - 1) begin
            p_cell_n = p_cell + 6'd1;
            p_col_n  = 4'd0;
        end else if (de) begin
            p_cell_n = p_cell;
            p_col_n  = p_col + 4'd1;
        end else begin
            p_cell_n = p_cell;
            p_col_n  = p_col;
        end

        if (!de) begin
            s_cell_n = scroll_cell;
            s_col_n  = scroll_col;
        end else if (s_cell == SCROLL_LAST_CELL && s_col == SCROLL_LAST_COL) begin
            s_cell_n = 6'd0;
            s_col_n  = 5'd0;
        end else if (s_col == SUBTITLE_CELL_W - 1) begin
            s_cell_n = s_cell + 6'd1;
            s_col_n  = 5'd0;
        end else begin
            s_cell_n = s_cell;
            s_col_n  = s_col + 5'd1;
        end
    end

    always @(posedge clk) begin
        p_cell <= p_cell_n;
        p_col  <= p_col_n;
        s_cell <= s_cell_n;
        s_col  <= s_col_n;
    end

    // ---------------- 字库：用"下一个像素"的格号取字形行，下一拍对上本拍像素 ----------------
    wire       in_param_rows = (pixel_y >= PARAM_Y) && (pixel_y < PARAM_Y + CELL_H);
    wire [9:0] param_dy      = pixel_y - PARAM_Y;
    wire [9:0] subtitle_dy   = pixel_y - (SUBTITLE_Y + 2);
    wire [2:0] param_row     = param_dy[3:1];
    wire [2:0] subtitle_row  = subtitle_dy[3:1];

    wire       next_sub_valid = (s_cell_n < subtitle_length) && (s_cell_n < MAX_TEXT);
    wire [7:0] next_sub_char  = next_sub_valid ? subtitle_text[s_cell_n[3:0]*8 +: 8] : " ";
    wire [8:0] font_addr      = in_param_rows ? {font_code(param_char(p_cell_n)), param_row}
                                              : {font_code(next_sub_char), subtitle_row};
    wire [4:0] font_q;

    // P4（第二批低功耗）：只在参数条行 / 字幕字形行（且字幕开着）读字库，其余行 BRAM 读口不翻转。
    //   这两类行里用到的 font_q 都是同一行内前一拍读的（PARAM_X >= 1、字幕字形从 x >= 12 起），输出逐拍不变
    wire       in_sub_rows = enable && subtitle_enable && (pixel_y >= SUBTITLE_Y + 2) && (pixel_y < SUBTITLE_Y + 16);
    osd_font_rom u_font (
        .clk  (clk),
        .re   (in_param_rows || in_sub_rows),
        .addr (font_addr),
        .q    (font_q)
    );

    // ---------------- 本拍像素命中 ----------------
    wire in_param_box = enable && de &&
                        (pixel_x >= PARAM_X) && (pixel_x < PARAM_X + PARAM_CHARS * CELL_W) &&
                        in_param_rows;
    assign param_pixel = in_param_box && (p_col < 4'd10) && (param_row != 3'd7) &&
                         font_q[3'd4 - p_col[3:1]];

    wire in_sub_columns = enable && subtitle_enable && de &&
                          (pixel_x >= 8) && (pixel_x < H_ACTIVE - 8);
    assign subtitle_band_pixel = in_sub_columns &&
                                 (pixel_y >= SUBTITLE_Y) && (pixel_y < SUBTITLE_BAND_BOTTOM);

    wire [4:0] glyph_col = s_col - 5'd4;
    assign subtitle_pixel = in_sub_columns &&
                            (pixel_y >= SUBTITLE_Y + 2) && (pixel_y < SUBTITLE_Y + 16) &&
                            (s_cell < subtitle_length) && (s_cell < MAX_TEXT) &&
                            (s_col >= 5'd4) && (s_col < 5'd14) &&
                            font_q[3'd4 - glyph_col[3:1]];

    assign overlay_active = param_pixel || subtitle_pixel || subtitle_band_pixel;
    assign rgb_out = overlay_active ?
                     (subtitle_pixel ? 24'h000000 :
                      subtitle_band_pixel ? 24'he8eef2 : 24'hfff080) : rgb_in;
endmodule
