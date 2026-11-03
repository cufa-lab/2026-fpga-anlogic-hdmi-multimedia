`timescale 1ns/1ps

module media_frame_loader #(
    parameter integer ADDR_WIDTH    = 21,
    parameter integer IMAGE_WORDS   = 307200,
    parameter integer FIFO_DEPTH    = 512,
    parameter integer BURST_SIZE    = 128,
    parameter integer STALL_TIMEOUT = 65535,
    parameter integer WRITE_V_FLIP  = 1,          // 1：BMP 自下而上 → SDRAM 自上而下
    parameter integer FRAME_WIDTH   = 640,
    parameter integer FRAME_HEIGHT  = 480
) (
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  load_start,
    input  wire [1:0]            image_index,
    input  wire                  pixel_valid,
    input  wire [23:0]           pixel_rgb,
    input  wire                  frame_commit,
    input  wire [7:0]            decode_error,
    output wire                  load_busy,
    output reg                   load_done,
    output reg  [3:0]            image_complete,
    output reg  [7:0]            memory_error,
    input  wire                  controller_init_done,
    input  wire                  controller_busy,
    input  wire                  refresh_active,
    // SDRAM 读写仲裁（media_sdram_arbiter）
    output wire                  burst_want,
    input  wire                  burst_grant,
    output wire                  burst_busy,
    output reg                   app_wr_en,
    output reg  [ADDR_WIDTH-1:0] app_wr_addr,
    output reg  [3:0]            app_wr_mask,
    output wire [31:0]           app_wr_data
);
    localparam ERR_NONE          = 8'h00;
    localparam ERR_NOT_ACTIVE    = 8'h41;
    localparam ERR_FIFO_OVERFLOW = 8'h42;
    localparam ERR_PIXEL_COUNT   = 8'h43;
    localparam ERR_DECODE        = 8'h44;
    localparam ERR_SHORT_FRAME   = 8'h45;
    localparam ERR_WRITE_STALL   = 8'h46;

    localparam integer PTR_WIDTH = (FIFO_DEPTH <= 2) ? 1 : $clog2(FIFO_DEPTH);
    localparam integer PCW       = $clog2(IMAGE_WORDS + 1);
    localparam integer STW       = $clog2(STALL_TIMEOUT + 1);
    localparam integer CW        = (FRAME_WIDTH <= 2) ? 1 : $clog2(FRAME_WIDTH);
    localparam [PTR_WIDTH:0] EFFECTIVE_BURST =
        (BURST_SIZE < FIFO_DEPTH) ? BURST_SIZE : FIFO_DEPTH;
    // 首像素地址偏移：翻转时是最后一行行首
    localparam [ADDR_WIDTH-1:0] FIRST_OFFSET = WRITE_V_FLIP ? (FRAME_HEIGHT - 1) * FRAME_WIDTH : 0;
    localparam [ADDR_WIDTH-1:0] LINE_JUMP    = FRAME_WIDTH * 2 - 1;
    localparam [ADDR_WIDTH-1:0] BANK0_BASE = 0;
    localparam [ADDR_WIDTH-1:0] BANK1_BASE = IMAGE_WORDS;
    localparam [ADDR_WIDTH-1:0] BANK2_BASE = IMAGE_WORDS * 2;
    localparam [ADDR_WIDTH-1:0] BANK3_BASE = IMAGE_WORDS * 3;

    reg active;
    reg aborting;                            // 已报错：写完进行中的 burst 就结束
    reg [1:0] active_image;
    reg commit_seen;
    reg [PCW-1:0] pixels_received;
    reg [PCW-1:0] pixels_written;
    reg [STW-1:0] stall_count;
    reg [PTR_WIDTH-1:0] wr_ptr;
    reg [PTR_WIDTH-1:0] rd_ptr;
    reg [PTR_WIDTH:0] fifo_count;
    reg burst_active;
    reg [PTR_WIDTH:0] burst_remaining;
    reg [ADDR_WIDTH-1:0] wr_addr;            // 下一个要写的 SDRAM 地址
    reg [CW-1:0] wr_col;                     // 当前行内列号（翻转写时用来判断行尾）
    wire [23:0] fifo_rdata;

    wire receiving    = active && !aborting;
    wire frame_full   = (pixels_received == IMAGE_WORDS);
    wire err_decode   = decode_error != 0;
    wire err_count    = pixel_valid && (pixels_received >= IMAGE_WORDS);
    wire err_overflow = pixel_valid && (fifo_count >= FIFO_DEPTH);
    wire push_allowed = receiving && pixel_valid && !err_decode && !err_count && !err_overflow;
    wire pop_allowed  = active && controller_init_done && burst_active && fifo_count != 0;
    wire burst_last   = pop_allowed && (burst_remaining == 1);
    wire [PTR_WIDTH:0] next_burst_words = (fifo_count >= EFFECTIVE_BURST) ?
                                          EFFECTIVE_BURST : fifo_count;

    assign load_busy = active;
    // The inherited controller consumes a continuous App_wr_en burst and
    // handles busy/refresh internally. Those status outputs are not per-word
    // ready signals and must not split an active write burst.
    // 零头只在整帧像素都到齐之后才写（短帧不写零头，保证 burst 长度是 4 的倍数）
    assign burst_want  = receiving && controller_init_done && !burst_active &&
                         fifo_count != 0 &&
                         (fifo_count >= EFFECTIVE_BURST || (commit_seen && frame_full));
    wire   start_burst = burst_want && burst_grant;
    assign burst_busy  = burst_active || app_wr_en;
    assign app_wr_data = {8'h00, fifo_rdata};

    // 像素 FIFO：BRAM，pop 拍给读地址，下一拍 fifo_rdata 与 app_wr_en 同拍有效
    fat32_sdp_ram #(.AW(PTR_WIDTH), .DW(24)) u_fifo_ram (
        .clk   (clk),
        .we    (push_allowed),
        .waddr (wr_ptr),
        .wdata (pixel_rgb),
        .re    (pop_allowed),
        .raddr (rd_ptr),
        .rdata (fifo_rdata)
    );

    function [ADDR_WIDTH-1:0] bank_first_addr;
        input [1:0] index;
        begin
            case (index)
                2'd0: bank_first_addr = BANK0_BASE + FIRST_OFFSET;
                2'd1: bank_first_addr = BANK1_BASE + FIRST_OFFSET;
                2'd2: bank_first_addr = BANK2_BASE + FIRST_OFFSET;
                default: bank_first_addr = BANK3_BASE + FIRST_OFFSET;
            endcase
        end
    endfunction

    task fail_load;
        input [7:0] code;
        begin
            memory_error <= code;
            aborting <= 1'b1;
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            active <= 1'b0;
            aborting <= 1'b0;
            active_image <= 2'd0;
            commit_seen <= 1'b0;
            pixels_received <= {PCW{1'b0}};
            pixels_written <= {PCW{1'b0}};
            stall_count <= {STW{1'b0}};
            wr_ptr <= {PTR_WIDTH{1'b0}};
            rd_ptr <= {PTR_WIDTH{1'b0}};
            fifo_count <= {(PTR_WIDTH+1){1'b0}};
            burst_active <= 1'b0;
            burst_remaining <= 0;
            wr_addr <= {ADDR_WIDTH{1'b0}};
            wr_col <= {CW{1'b0}};
            load_done <= 1'b0;
            image_complete <= 4'd0;
            memory_error <= ERR_NONE;
            app_wr_en <= 1'b0;
            app_wr_addr <= {ADDR_WIDTH{1'b0}};
            app_wr_mask <= 4'b0000;
        end else begin
            load_done <= 1'b0;
            app_wr_en <= 1'b0;

            if (load_start && !active) begin
                active <= 1'b1;
                aborting <= 1'b0;
                active_image <= image_index;
                commit_seen <= 1'b0;
                pixels_received <= {PCW{1'b0}};
                pixels_written <= {PCW{1'b0}};
                stall_count <= {STW{1'b0}};
                wr_ptr <= {PTR_WIDTH{1'b0}};
                rd_ptr <= {PTR_WIDTH{1'b0}};
                fifo_count <= {(PTR_WIDTH+1){1'b0}};
                burst_active <= 1'b0;
                burst_remaining <= 0;
                wr_addr <= bank_first_addr(image_index);
                wr_col <= {CW{1'b0}};
                memory_error <= ERR_NONE;
                case (image_index)
                    2'd0: image_complete[0] <= 1'b0;
                    2'd1: image_complete[1] <= 1'b0;
                    2'd2: image_complete[2] <= 1'b0;
                    default: image_complete[3] <= 1'b0;
                endcase
            end

            if (active) begin
                // ---- 写 burst（报错后也照常走完）----
                if (start_burst) begin
                    burst_active <= 1'b1;
                    burst_remaining <= next_burst_words;
                end

                if (push_allowed) begin
                    wr_ptr <= wr_ptr + 1'b1;
                    pixels_received <= pixels_received + 1'b1;
                end

                if (pop_allowed) begin
                    app_wr_en <= 1'b1;
                    app_wr_addr <= wr_addr;
                    app_wr_mask <= 4'b0000;
                    rd_ptr <= rd_ptr + 1'b1;
                    pixels_written <= pixels_written + 1'b1;
                    if (WRITE_V_FLIP && wr_col == FRAME_WIDTH - 1) begin
                        wr_col  <= {CW{1'b0}};
                        wr_addr <= wr_addr - LINE_JUMP;
                    end else begin
                        wr_col  <= wr_col + 1'b1;
                        wr_addr <= wr_addr + 1'b1;
                    end
                    if (burst_remaining == 1) begin
                        burst_active <= 1'b0;
                        burst_remaining <= 0;
                    end else begin
                        burst_remaining <= burst_remaining - 1'b1;
                    end
                end

                case ({push_allowed, pop_allowed})
                    2'b10: fifo_count <= fifo_count + 1'b1;
                    2'b01: fifo_count <= fifo_count - 1'b1;
                    default: fifo_count <= fifo_count;
                endcase

                // ---- 错误检测 / 结束判定 ----
                if (aborting) begin
                    if (!burst_active || burst_last) begin
                        active <= 1'b0;
                        aborting <= 1'b0;
                        load_done <= 1'b1;
                    end
                end else if (err_decode) begin
                    fail_load(ERR_DECODE);
                end else if (err_count) begin
                    fail_load(ERR_PIXEL_COUNT);
                end else if (err_overflow) begin
                    fail_load(ERR_FIFO_OVERFLOW);
                end else if (commit_seen && !frame_full) begin
                    fail_load(ERR_SHORT_FRAME);
                end else begin
                    if (pop_allowed) begin
                        stall_count <= {STW{1'b0}};
                    end else if (fifo_count != 0 &&
                                 (commit_seen || fifo_count >= EFFECTIVE_BURST)) begin
                        // 等仲裁放行也算停顿：读侧长期霸占 SDRAM 时报 0x46
                        if (stall_count == STALL_TIMEOUT)
                            fail_load(ERR_WRITE_STALL);
                        else
                            stall_count <= stall_count + 1'b1;
                    end else begin
                        stall_count <= {STW{1'b0}};
                    end

                    if (frame_commit)
                        commit_seen <= 1'b1;

                    if (commit_seen && fifo_count == 0 && !pop_allowed &&
                        !burst_active) begin
                        if (pixels_written != IMAGE_WORDS) begin
                            fail_load(ERR_SHORT_FRAME);
                        end else begin
                            case (active_image)
                                2'd0: image_complete[0] <= 1'b1;
                                2'd1: image_complete[1] <= 1'b1;
                                2'd2: image_complete[2] <= 1'b1;
                                default: image_complete[3] <= 1'b1;
                            endcase
                            active <= 1'b0;
                            load_done <= 1'b1;
                        end
                    end
                end
            end else if (pixel_valid) begin
                memory_error <= ERR_NOT_ACTIVE;
            end
        end
    end
endmodule
