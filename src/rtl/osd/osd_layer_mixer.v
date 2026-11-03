`timescale 1ns/1ps

module osd_layer_mixer #(
    parameter integer H_ACTIVE          = 640,
    parameter integer PARAM_X           = 8,
    parameter integer PARAM_Y           = 40,
    parameter integer SUBTITLE_Y        = 452,
    parameter integer TS_X              = 592,
    parameter integer TS_Y              = 8,
    parameter integer SCROLL_DIV        = 3,
    parameter integer CLK_HZ            = 25_000_000,
    parameter integer PARAM_HOLD_FRAMES = 240,          // 约 4 秒；0 = 参数条常显
    parameter integer TS_ENABLE         = 1,            // 0 = 不要右上角时间戳（不例化计时器）
    parameter integer SUBTITLE_EN       = 1,            // 0 = 去掉旧滚动字幕
    parameter [7:0]   FADE_STEP         = 8'd16,        // 每帧 alpha 步进（16 帧约 0.27 秒渐变）
    parameter [7:0]   PANEL_ALPHA       = 8'd128,       // 时间戳底板不透明度
    parameter [23:0]  PANEL_COLOR       = 24'h000000,
    parameter [23:0]  PARAM_COLOR       = 24'hfff080,
    parameter [23:0]  BAND_COLOR        = 24'he8eef2,
    parameter [23:0]  SUB_TEXT_COLOR    = 24'h000000,
    parameter [23:0]  TS_COLOR          = 24'he0f0ff,
    parameter integer PIPE              = 0             // 逻辑优化 L21+P9：0 = 纯组合（原版，tb 默认）；1 = 两级流水，输出晚 2 拍
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        vsync,               // 低有效
    input  wire        de,
    input  wire [23:0] rgb_in,
    input  wire        subtitle_enable,
    input  wire [95:0] subtitle_text,
    input  wire [5:0]  subtitle_length,
    input  wire [7:0]  brightness,           // 0..255（参数条 BRT 显示真实值）
    input  wire [7:0]  volume,
    input  wire [8:0]  period_tenths,
    input  wire        auto_enabled,
    input  wire        res_valid,           // 当前图片分辨率（pix 域，画面换图同帧更新）
    input  wire [11:0] res_width,
    input  wire [11:0] res_height,
    input  wire        param_show,          // 单拍，切到新画面 → 参数条重新显示
    input  wire        param_hide,          // 全文讲解页打开期间参数条淡出（关页后若仍在显示期则淡回）
    input  wire        ext_hit,             // 外部图层（与本拍像素对齐，组合）
    input  wire [23:0] ext_color,
    input  wire [7:0]  ext_alpha,
    input  wire        txt_hit,             
    input  wire [23:0] txt_color,
    input  wire [7:0]  txt_alpha,
    output wire [9:0]  pix_x,
    output wire [9:0]  pix_y,
    output wire        pix_frame_start,
    output wire [23:0] rgb_out,
    output wire        overlay_active
);
    localparam integer PARAM_W = 34 * 12;      // 34 格（含分辨率），与 promax_text_overlay PARAM_CHARS 一致
    localparam integer HOLD_W  = (PARAM_HOLD_FRAMES > 0) ? $clog2(PARAM_HOLD_FRAMES + 1) : 1;

    // ---------------- 坐标重建 + 帧起点 ----------------
    reg [9:0] pixel_x;
    reg [9:0] pixel_y;
    reg       de_q;
    reg       vs_q;
    wire      frame_start = vs_q && !vsync;
    assign pix_x           = pixel_x;
    assign pix_y           = pixel_y;
    assign pix_frame_start = frame_start;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pixel_x <= 10'd0;
            pixel_y <= 10'd0;
            de_q    <= 1'b0;
            vs_q    <= 1'b1;
        end else begin
            de_q    <= de;
            vs_q    <= vsync;
            pixel_x <= de ? pixel_x + 10'd1 : 10'd0;
            if (!vsync)
                pixel_y <= 10'd0;
            else if (de_q && !de)
                pixel_y <= pixel_y + 10'd1;
        end
    end

    // ---------------- 淡入淡出 ----------------
    reg  [7:0]        param_fade;
    reg  [7:0]        subtitle_fade;
    reg  [HOLD_W-1:0] param_hold;
    wire              param_changed;
    wire              param_target    = !param_hide && ((PARAM_HOLD_FRAMES == 0) || (param_hold != {HOLD_W{1'b0}}));

    function [7:0] fade_step;
        input [7:0] cur;
        input       up;
        begin
            if (up)
                fade_step = (cur > 8'd255 - FADE_STEP) ? 8'd255 : cur + FADE_STEP;
            else
                fade_step = (cur < FADE_STEP) ? 8'd0 : cur - FADE_STEP;
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            param_fade    <= 8'd0;
            subtitle_fade <= 8'd0;
            param_hold    <= PARAM_HOLD_FRAMES;           // 上电先显示一次参数条
        end else begin
            if (param_changed || param_show)
                param_hold <= PARAM_HOLD_FRAMES;
            else if (frame_start && param_hold != {HOLD_W{1'b0}})
                param_hold <= param_hold - 1'b1;
            if (frame_start) begin
                param_fade    <= fade_step(param_fade, param_target);
                subtitle_fade <= fade_step(subtitle_fade, subtitle_enable && (SUBTITLE_EN != 0));
            end
        end
    end

    // ---------------- 文字层：参数条 + 滚动字幕 ----------------
    wire param_pixel;
    wire subtitle_pixel;
    wire subtitle_band_pixel;

    promax_text_overlay #(
        .PARAM_X    (PARAM_X),
        .PARAM_Y    (PARAM_Y),
        .SUBTITLE_Y (SUBTITLE_Y),
        .H_ACTIVE   (H_ACTIVE),
        .SCROLL_DIV (SCROLL_DIV)
    ) u_text (
        .clk                 (clk),
        .rst_n               (rst_n),
        .frame_start         (frame_start),
        .de                  (de),
        .rgb_in              (rgb_in),
        .pixel_x             (pixel_x),
        .pixel_y             (pixel_y),
        .enable              (1'b1),
        .subtitle_enable     (subtitle_fade != 8'd0),     // 淡出期间继续画，alpha 降到 0 才关
        .subtitle_text       (subtitle_text),
        .subtitle_length     (subtitle_length),
        .brightness          (brightness),
        .volume              (volume),
        .period_tenths       (period_tenths),
        .auto_enabled        (auto_enabled),
        .res_valid           (res_valid),
        .res_width           (res_width),
        .res_height          (res_height),
        .rgb_out             (),
        .overlay_active      (),
        .param_pixel         (param_pixel),
        .subtitle_pixel      (subtitle_pixel),
        .subtitle_band_pixel (subtitle_band_pixel),
        .param_changed       (param_changed)
    );

    // ---------------- 时间戳层 ----------------
    wire ts_box;
    wire ts_pixel;

    generate
        if (TS_ENABLE != 0) begin : g_ts
            timestamp_osd #(
                .OSD_X  (TS_X),
                .OSD_Y  (TS_Y),
                .CLK_HZ (CLK_HZ)
            ) u_timestamp (
                .pix_clk     (clk),
                .pix_rst_n   (rst_n),
                .enable      (1'b1),
                .frame_start (frame_start),
                .de          (de),
                .pixel_x     (pixel_x),
                .pixel_y     (pixel_y),
                .rgb_in      (rgb_in),
                .rgb_out     (),
                .ts_active   (),
                .ts_pixel    (ts_pixel)
            );
            // 时间戳底板："MM:SS" 实际笔画 x 0..35、y 0..11
            assign ts_box = de && (pixel_x >= TS_X - 3) && (pixel_x < TS_X + 39) &&
                            (pixel_y >= TS_Y - 2) && (pixel_y < TS_Y + 14);
        end else begin : g_no_ts
            assign ts_pixel = 1'b0;
            assign ts_box   = 1'b0;
        end
    endgenerate

    // ---------------- 半透明底板区域 ----------------
    // 参数条底板：文字框四周留边（文字占 14 行 / 每格前 10 列）
    wire param_box = de && (pixel_x >= PARAM_X - 4) && (pixel_x < PARAM_X + PARAM_W) &&
                     (pixel_y >= PARAM_Y - 3) && (pixel_y < PARAM_Y + 16);

    wire param_vis = (param_fade != 8'd0);

    // ---------------- 图层选择 + Alpha 混合 ----------------
    reg        layer_hit;
    reg [23:0] layer_color;
    reg [7:0]  layer_alpha;

    always @* begin
        layer_hit   = 1'b1;
        layer_color = PANEL_COLOR;
        layer_alpha = 8'd0;
        if (ts_pixel) begin
            layer_color = TS_COLOR;
            layer_alpha = 8'd255;
        end else if (ts_box) begin
            layer_color = PANEL_COLOR;
            layer_alpha = PANEL_ALPHA;
        end else if (param_pixel && param_vis) begin
            layer_color = PARAM_COLOR;
            layer_alpha = param_fade;
        end else if (param_box && param_vis) begin   // 扩1：淡出到 0 后不再挡住下层（讲解全文页）
            layer_color = PANEL_COLOR;
            layer_alpha = {1'b0, param_fade[7:1]};
        end else if (ext_hit) begin
            layer_color = ext_color;
            layer_alpha = ext_alpha;
        end else if (subtitle_pixel) begin
            layer_color = SUB_TEXT_COLOR;
            layer_alpha = subtitle_fade;
        end else if (subtitle_band_pixel) begin
            layer_color = BAND_COLOR;
            layer_alpha = subtitle_fade - {2'b00, subtitle_fade[7:2]};
        end else begin
            layer_hit   = 1'b0;
        end
    end

    wire [23:0] blended_rgb;
    osd_alpha_blend u_blend (
        .bg    (rgb_in),
        .fg    (layer_color),
        .alpha (layer_alpha),           // 不命中时 alpha = 0：乘法器一个操作数恒 0，部分积不翻转
        .out   (blended_rgb)
    );

    wire [23:0] stage1_rgb = layer_hit ? blended_rgb : rgb_in;
    wire [23:0] text_rgb;

    generate
        if (PIPE == 0) begin : g_comb
            // 原版：纯组合，输出与输入同拍
            osd_alpha_blend u_blend_txt (
                .bg    (stage1_rgb),
                .fg    (txt_color),
                .alpha (txt_alpha),
                .out   (text_rgb)
            );
            assign rgb_out        = txt_hit ? text_rgb : stage1_rgb;
            assign overlay_active = layer_hit | txt_hit;
        end else begin : g_pipe
            // L21：两级混合之间、输出处各打一拍（输出晚 2 拍，顶层 hs/vs/de 同步延迟 2 拍）
            // P9：第 2 级混合器的 3 个操作数寄存器只在 txt_hit 那拍装入，没有讲解文字的像素
            //     乘法器输入保持不变、停止翻转；输出选择器保留（不命中走 s1_rgb）
            // L7：纯数据流水寄存器不接复位（上电首帧前 de=0，垃圾数据被消隐挡住）
            reg [23:0] s1_rgb;
            reg        s1_txt_hit;
            reg        s1_layer_hit;
            reg [23:0] t_bg;
            reg [23:0] t_color;
            reg [7:0]  t_alpha;
            reg [23:0] rgb_q;
            reg        ovl_q;
            always @(posedge clk) begin
                s1_rgb       <= stage1_rgb;
                s1_txt_hit   <= txt_hit;
                s1_layer_hit <= layer_hit;
                if (txt_hit) begin
                    t_bg    <= stage1_rgb;
                    t_color <= txt_color;
                    t_alpha <= txt_alpha;
                end
                rgb_q <= s1_txt_hit ? text_rgb : s1_rgb;
                ovl_q <= s1_layer_hit | s1_txt_hit;
            end
            osd_alpha_blend u_blend_txt (
                .bg    (t_bg),
                .fg    (t_color),
                .alpha (t_alpha),
                .out   (text_rgb)
            );
            assign rgb_out        = rgb_q;
            assign overlay_active = ovl_q;
        end
    endgenerate
endmodule
