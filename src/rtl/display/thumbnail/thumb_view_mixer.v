`timescale 1ns/1ps

module thumb_view_mixer #(
    parameter integer X_OFFSET    = 160,
    parameter integer Y_OFFSET    = 45,
    parameter [23:0]  FOCUS_RGB   = 24'hFFD000,
    parameter integer PLACEHOLDER = 0,
    parameter integer CELL_W      = 160,     // 格子宽（320 = 全屏 2×2）
    parameter integer CELL_H      = 120,     // 格子高（240 = 全屏 2×2）
    parameter [7:0]   EMPTY_LEVEL = 8'd0     // 空位暗灰框灰度（0 = 黑，原行为）
) (
    input  wire        de_in,
    input  wire [11:0] x_in,
    input  wire [11:0] y_in,
    input  wire [23:0] rgb_in,
    input  wire        thumb_display,
    input  wire [1:0]  focus_index,
    input  wire [3:0]  image_complete,
    input  wire        clip_en,
    input  wire [11:0] clip_x0, clip_x1, clip_y0, clip_y1,
    input  wire [3:0]  cell_exists,      
    input  wire [7:0]  ph_level,         
    output wire [23:0] rgb_out,
    // 观测 / 测试
    output wire        thumb_de,
    output wire [1:0]  source_index,
    output wire        focus_border,
    output wire        placeholder,     
    output wire        empty_slot        
);
    wire [11:0] px = x_in - X_OFFSET;
    wire [11:0] py = y_in - Y_OFFSET;
    wire [9:0]  src_x_unused, src_y_unused;   // 第5区拼图已在加载时完成 4:1 映射（见 media_thumb_writer）

    thumbnail_layout #(
        .DISPLAY_WIDTH(640), .DISPLAY_HEIGHT(480), .THUMB_WIDTH(CELL_W), .THUMB_HEIGHT(CELL_H)
    ) u_layout (
        .in_de(de_in && thumb_display), .pixel_x(px), .pixel_y(py),
        .focus_index(focus_index), .image_complete(image_complete),
        .thumb_de(thumb_de), .source_index(source_index),
        .source_x(src_x_unused), .source_y(src_y_unused), .focus_border(focus_border));

    wire [1:0]  ph_cell = {py >= CELL_H, px >= CELL_W};
    wire [9:0]  ph_lx   = px[9:0] - (ph_cell[0] ? CELL_W : 10'd0);
    wire [8:0]  ph_ly   = py[8:0] - (ph_cell[1] ? CELL_H : 9'd0);
    wire        ph_edge = (ph_lx < 10'd2) || (ph_lx >= CELL_W - 2) || (ph_ly < 9'd2) || (ph_ly >= CELL_H - 2);
    wire        in_grid = de_in && thumb_display && (px < 2 * CELL_W) && (py < 2 * CELL_H);
    assign placeholder  = (PLACEHOLDER != 0) && in_grid && !image_complete[ph_cell] && cell_exists[ph_cell];
    assign empty_slot   = (EMPTY_LEVEL != 0) && in_grid && !cell_exists[ph_cell];
    wire [7:0]  ph_gray = ph_edge ? ph_level : {1'b0, ph_level[7:1]};
    wire [7:0]  em_gray = ph_edge ? EMPTY_LEVEL : {2'b00, EMPTY_LEVEL[7:2]};

    wire        in_rect = !clip_en ||
                          ((x_in >= clip_x0) && (x_in < clip_x1) && (y_in >= clip_y0) && (y_in < clip_y1));
    wire [23:0] base    = !thumb_display ? rgb_in :
                          thumb_de       ? (focus_border ? FOCUS_RGB : rgb_in) :
                          placeholder    ? {3{ph_gray}} :
                          empty_slot     ? {3{em_gray}} : 24'h000000;
    assign rgb_out = in_rect ? base : 24'h000000;
endmodule
