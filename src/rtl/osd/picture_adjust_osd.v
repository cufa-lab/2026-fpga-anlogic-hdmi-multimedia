`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// picture_adjust_osd —— 亮度 / 对比度 OSD 参数面板
//
// 两行，每行 16 像素高（5x7 字形横竖各放大 2 倍，与参数条同尺寸）：
//     ▌b 255 [██████████████████]      亮度   0..255（255 = 正常）
//      C 128 [█████████         ]      对比度 0..255（128 = 正常，进度条中点有刻度）
//   行首色块 = 调节模式中的当前项；当前项文字黄色（FFD000）/ 进度条青色，另一项白 / 灰。
//   进度条 128 像素宽，填充长度 = 值 / 2（255 填满）；面板底色为背景 50% 压暗（移位，无乘法器）。
// 显示时机：调节模式中常显；模式外任何一项数值变化（如 KEY3 长按亮度阶梯）后显示 HOLD_FRAMES 帧
//   再隐藏（退出调节模式后同样保持 HOLD_FRAMES 帧）；HOLD_FRAMES = 0 时常显。
// 帧边界：显示值、模式、当前项都在 frame_start 锁存，一帧内不变。
// 输入：brightness / contrast 接 brightness_contrast_adjust 的本帧生效值，adj_mode / adj_sel 已在
//   pix_clk 域（sys 域经 cdc_bus_handshake）。字形沿用 status_osd_overlay 的 5x7 十六进制字库
//   （0..9、b、C 都在里面）。坐标本地用 vs/de 重建（同 osd_overlay_24）；像素通路纯组合，0 拍延迟。
// 版面：默认 (8,64)..(203,99)，在状态条 (8..63, 8..15)、参数条 (4..391, 37..55) 之下，
//   不碰时间戳 / 波形 (y ≥ 384) / 字幕带 (y ≥ 452)。
// -----------------------------------------------------------------------------
module picture_adjust_osd #(
    parameter integer PANEL_X     = 8,
    parameter integer PANEL_Y     = 64,
    parameter integer HOLD_FRAMES = 240,               // 约 4 s；0 = 常显
    parameter [23:0]  SEL_COLOR   = 24'hffd000,
    parameter [23:0]  TEXT_COLOR  = 24'hf0f0f0,
    parameter [23:0]  BAR_SEL     = 24'h00c0ff,
    parameter [23:0]  BAR_OTHER   = 24'h909090
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        frame_start,
    input  wire        vsync,          // 低有效
    input  wire        de,
    input  wire [23:0] rgb_in,
    input  wire [7:0]  brightness,
    input  wire [7:0]  contrast,
    input  wire        adj_mode,
    input  wire        adj_sel,        // 0 = 亮度，1 = 对比度
    output wire [23:0] rgb_out,
    output wire        osd_active
);
    localparam integer CELL   = 12;                   // 字符格宽（10 像素字形 + 2 间隔）
    localparam integer TEXT_X = PANEL_X + 8;          // 行首色块 4 像素 + 4 间隔
    localparam integer BAR_X  = TEXT_X + 5 * CELL;    // "b 255 " 之后
    localparam integer BAR_W  = 128;
    localparam integer PANEL_W = BAR_X + BAR_W + 4 - (PANEL_X - 2);
    localparam integer HOLD_W = (HOLD_FRAMES > 0) ? $clog2(HOLD_FRAMES + 1) : 1;

    // ---------------- 坐标重建 ----------------
    reg [9:0] pixel_x;
    reg [9:0] pixel_y;
    reg       de_q;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pixel_x <= 10'd0;
            pixel_y <= 10'd0;
            de_q    <= 1'b0;
        end else begin
            de_q    <= de;
            pixel_x <= de ? pixel_x + 10'd1 : 10'd0;
            if (!vsync)
                pixel_y <= 10'd0;
            else if (de_q && !de)
                pixel_y <= pixel_y + 10'd1;
        end
    end

    // ---------------- 帧锁存 + 显示计时 ----------------
    reg [7:0]  bri_q, con_q;
    reg        mode_q, sel_q;
    reg [HOLD_W-1:0] hold;
    // 外接光照：亮度会随环境自动变，不再触发面板弹出（参数条负责提示）；按键调节都在调节模式内，模式中常显
    wire value_changed = (contrast != con_q);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bri_q  <= 8'd255;
            con_q  <= 8'd128;
            mode_q <= 1'b0;
            sel_q  <= 1'b0;
            hold   <= {HOLD_W{1'b0}};
        end else if (frame_start) begin
            bri_q  <= brightness;
            con_q  <= contrast;
            mode_q <= adj_mode;
            sel_q  <= adj_sel;
            if (value_changed || adj_mode || mode_q)     // 退出模式那一帧也重装：退出后同样保持 HOLD_FRAMES 帧
                hold <= HOLD_FRAMES;
            else if (hold != {HOLD_W{1'b0}})
                hold <= hold - 1'b1;
        end
    end
    wire visible = (HOLD_FRAMES == 0) || mode_q || (hold != {HOLD_W{1'b0}});

    // ---------------- 字库（5x7，0..F；与 status_osd_overlay 相同字形）----------------
    function [4:0] font_row;
        input [3:0] value;
        input [2:0] row;
        begin
            case (value)
                4'h0: case(row) 0:font_row=5'b11111;1:font_row=5'b10001;2:font_row=5'b10001;3:font_row=5'b10001;4:font_row=5'b10001;5:font_row=5'b10001;default:font_row=5'b11111; endcase
                4'h1: case(row) 0:font_row=5'b00100;1:font_row=5'b01100;2:font_row=5'b00100;3:font_row=5'b00100;4:font_row=5'b00100;5:font_row=5'b00100;default:font_row=5'b11111; endcase
                4'h2: case(row) 0:font_row=5'b11111;1:font_row=5'b00001;2:font_row=5'b00001;3:font_row=5'b11111;4:font_row=5'b10000;5:font_row=5'b10000;default:font_row=5'b11111; endcase
                4'h3: case(row) 0:font_row=5'b11111;1:font_row=5'b00001;2:font_row=5'b00001;3:font_row=5'b11111;4:font_row=5'b00001;5:font_row=5'b00001;default:font_row=5'b11111; endcase
                4'h4: case(row) 0:font_row=5'b10001;1:font_row=5'b10001;2:font_row=5'b10001;3:font_row=5'b11111;4:font_row=5'b00001;5:font_row=5'b00001;default:font_row=5'b00001; endcase
                4'h5: case(row) 0:font_row=5'b11111;1:font_row=5'b10000;2:font_row=5'b10000;3:font_row=5'b11111;4:font_row=5'b00001;5:font_row=5'b00001;default:font_row=5'b11111; endcase
                4'h6: case(row) 0:font_row=5'b11111;1:font_row=5'b10000;2:font_row=5'b10000;3:font_row=5'b11111;4:font_row=5'b10001;5:font_row=5'b10001;default:font_row=5'b11111; endcase
                4'h7: case(row) 0:font_row=5'b11111;1:font_row=5'b00001;2:font_row=5'b00010;3:font_row=5'b00100;4:font_row=5'b01000;5:font_row=5'b01000;default:font_row=5'b01000; endcase
                4'h8: case(row) 0:font_row=5'b11111;1:font_row=5'b10001;2:font_row=5'b10001;3:font_row=5'b11111;4:font_row=5'b10001;5:font_row=5'b10001;default:font_row=5'b11111; endcase
                4'h9: case(row) 0:font_row=5'b11111;1:font_row=5'b10001;2:font_row=5'b10001;3:font_row=5'b11111;4:font_row=5'b00001;5:font_row=5'b00001;default:font_row=5'b11111; endcase
                4'hb: case(row) 0:font_row=5'b10000;1:font_row=5'b10000;2:font_row=5'b11110;3:font_row=5'b10001;4:font_row=5'b10001;5:font_row=5'b10001;default:font_row=5'b11110; endcase
                4'hc: case(row) 0:font_row=5'b01111;1:font_row=5'b10000;2:font_row=5'b10000;3:font_row=5'b10000;4:font_row=5'b10000;5:font_row=5'b10000;default:font_row=5'b01111; endcase
                default: font_row = 5'b00000;          // 空白
            endcase
        end
    endfunction

    // 二进制转 BCD（移位加 3，输入是帧锁存值，静态）
    function [11:0] bin2bcd8;
        input [7:0] v;
        integer n;
        reg [11:0] bcd;
        begin
            bcd = 12'd0;
            for (n = 7; n >= 0; n = n - 1) begin
                if (bcd[3:0] >= 4'd5) bcd[3:0] = bcd[3:0] + 4'd3;
                if (bcd[7:4] >= 4'd5) bcd[7:4] = bcd[7:4] + 4'd3;
                bcd = {bcd[10:0], v[n]};
            end
            bin2bcd8 = bcd;
        end
    endfunction

    // ---------------- 本拍像素命中 ----------------
    wire [9:0] dy       = pixel_y - PANEL_Y;
    wire       in_rows  = (pixel_y >= PANEL_Y) && (pixel_y < PANEL_Y + 32);
    wire       line     = dy[4];                       // 0 = 亮度行，1 = 对比度行
    wire [3:0] ly       = dy[3:0];
    wire [7:0] value    = line ? con_q : bri_q;
    wire [11:0] bcd     = bin2bcd8(value);
    wire       line_sel = (line == sel_q);

    wire       in_panel = visible && de && in_rows &&
                          (pixel_x >= PANEL_X - 2) && (pixel_x < PANEL_X - 2 + PANEL_W);

    // 行首色块：调节模式中标出当前项
    wire marker = mode_q && line_sel && (pixel_x >= PANEL_X) && (pixel_x < PANEL_X + 4) &&
                  (ly >= 4'd1) && (ly < 4'd14);

    // 文字：格 0 = b/C，格 1 空，格 2..4 = 百/十/个位（百位为 0 时空白）
    wire [9:0] tx     = pixel_x - TEXT_X;
    wire       in_txt = (pixel_x >= TEXT_X) && (pixel_x < TEXT_X + 5 * CELL) && (ly < 4'd14);
    wire [9:0] tx12 = tx - 10'd12;
    wire [9:0] tx24 = tx - 10'd24;
    wire [9:0] tx36 = tx - 10'd36;
    wire [9:0] tx48 = tx - 10'd48;
    reg  [2:0] cell_idx;
    reg  [3:0] col;
    always @* begin                                    // tx / 12（0..59 → 5 格，常数比较，不用除法器）
        if      (tx < 10'd12) begin cell_idx = 3'd0; col = tx[3:0];   end
        else if (tx < 10'd24) begin cell_idx = 3'd1; col = tx12[3:0]; end
        else if (tx < 10'd36) begin cell_idx = 3'd2; col = tx24[3:0]; end
        else if (tx < 10'd48) begin cell_idx = 3'd3; col = tx36[3:0]; end
        else                  begin cell_idx = 3'd4; col = tx48[3:0]; end
    end
    reg [3:0] glyph;
    always @* begin
        case (cell_idx)
            3'd0:    glyph = line ? 4'hc : 4'hb;
            3'd2:    glyph = (bcd[11:8] == 4'd0) ? 4'hf : bcd[11:8];
            3'd3:    glyph = bcd[7:4];
            3'd4:    glyph = bcd[3:0];
            default: glyph = 4'hf;                     // 空白（字库里 F 已改为空）
        endcase
    end
    wire [4:0] grow    = font_row(glyph, ly[3:1]);
    wire       txt_pix = in_txt && (col < 4'd10) && grow[3'd4 - col[3:1]];

    // 进度条：外框 1 像素，内宽 128；填充 = 值/2；对比度行在中点 64 画刻度
    wire [9:0] bx      = pixel_x - BAR_X;
    wire       in_bar  = (pixel_x >= BAR_X) && (pixel_x < BAR_X + BAR_W + 2) && (ly >= 4'd1) && (ly < 4'd14);
    wire       bar_edge = in_bar && ((bx == 10'd0) || (bx == BAR_W + 1) || (ly == 4'd1) || (ly == 4'd13));
    wire [9:0] fx      = bx - 10'd1;                   // 0..127
    wire       bar_fill = in_bar && !bar_edge && (({1'b0, fx[6:0]} < {1'b0, value[7:1]}) || (value == 8'd255));
    wire       bar_tick = in_bar && !bar_edge && line && (fx == 10'd64) && (ly >= 4'd3) && (ly < 4'd12);

    // ---------------- 颜色 ----------------
    wire [23:0] dim_bg = {1'b0, rgb_in[23:17], 1'b0, rgb_in[15:9], 1'b0, rgb_in[7:1]};
    reg  [23:0] pix;
    always @* begin
        pix = rgb_in;
        if (in_panel) begin
            pix = dim_bg;
            if (marker)
                pix = SEL_COLOR;
            else if (txt_pix)
                pix = (line_sel && mode_q) ? SEL_COLOR : TEXT_COLOR;
            else if (bar_edge)
                pix = TEXT_COLOR;
            else if (bar_tick)
                pix = 24'hff4040;
            else if (bar_fill)
                pix = (line_sel && mode_q) ? BAR_SEL : BAR_OTHER;
        end
    end

    assign rgb_out    = pix;
    assign osd_active = in_panel;
endmodule
