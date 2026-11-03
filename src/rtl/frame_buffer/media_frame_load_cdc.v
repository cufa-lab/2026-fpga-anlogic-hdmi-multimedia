`timescale 1ns/1ps

module media_frame_load_cdc (
    // ---------------- sys_clk 域（sd_card_bmp）----------------
    input  wire        src_clk,
    input  wire        src_rst_n,
    input  wire        load_start,
    input  wire [1:0]  load_bank,
    input  wire        pixel_valid,
    input  wire [23:0] pixel_rgb,
    input  wire        frame_commit,
    input  wire        load_abort,
    output reg         done_event,          // 单拍：loader 结束了一次加载
    output reg  [7:0]  done_error,          // 与 done_event 同拍有效：0 = 整帧写完
    output wire [3:0]  image_complete_sys,
    output reg         stream_overflow,     // 粘滞：命令 FIFO 满时还在写（观测用）

    // ---------------- mem_clk 域（media_frame_loader）----------------
    input  wire        dst_clk,
    input  wire        dst_rst_n,
    output reg         ld_load_start,
    output reg  [1:0]  ld_image_index,
    output reg         ld_pixel_valid,
    output reg  [23:0] ld_pixel_rgb,
    output reg         ld_frame_commit,
    output reg  [7:0]  ld_decode_error,
    input  wire        ld_load_done,
    input  wire [7:0]  ld_memory_error,
    input  wire [3:0]  ld_image_complete
);
    localparam [1:0] T_START = 2'b01;
    localparam [1:0] T_PIXEL = 2'b10;
    localparam [1:0] T_ABORT = 2'b11;

    // ---------------- 正向命令流 ----------------
    wire        fifo_we = load_start || pixel_valid || load_abort;
    wire [31:0] fifo_di = load_abort ? {T_ABORT, 30'd0} :
                          load_start ? {T_START, 28'd0, load_bank} :
                                       {T_PIXEL, 5'd0, frame_commit, pixel_rgb};
    wire        fifo_full;
    wire        fifo_empty;
    wire        fifo_valid;
    wire [31:0] fifo_do;

    wfifo_32_32_512 #(
        .DATA_WIDTH_W (32), .DATA_WIDTH_R (32),
        .ADDR_WIDTH_W (8),  .ADDR_WIDTH_R (8),
        .AL_FULL_NUM  (250), .AL_EMPTY_NUM (2),
        .SHOW_AHEAD_EN(1'b0), .OUTREG_EN("NOREG")
    ) u_cmd_fifo (
        .rst        (!src_rst_n || !dst_rst_n),
        .clkw       (src_clk),
        .clkr       (dst_clk),
        .we         (fifo_we),
        .di         (fifo_di),
        .re         (!fifo_empty),
        .dout       (fifo_do),
        .valid      (fifo_valid),
        .full_flag  (fifo_full),
        .empty_flag (fifo_empty),
        .afull      (),
        .aempty     (),
        .wrusedw    (),
        .rdusedw    ()
    );

    always @(posedge src_clk or negedge src_rst_n) begin
        if (!src_rst_n)
            stream_overflow <= 1'b0;
        else if (fifo_we && fifo_full)
            stream_overflow <= 1'b1;
    end

    // mem 域：每拍至多弹出一条命令，寄存一拍再给 loader
    always @(posedge dst_clk or negedge dst_rst_n) begin
        if (!dst_rst_n) begin
            ld_load_start   <= 1'b0;
            ld_image_index  <= 2'd0;
            ld_pixel_valid  <= 1'b0;
            ld_frame_commit <= 1'b0;
            ld_decode_error <= 8'h00;
        end else begin
            ld_load_start   <= fifo_valid && (fifo_do[31:30] == T_START);
            ld_pixel_valid  <= fifo_valid && (fifo_do[31:30] == T_PIXEL);
            ld_frame_commit <= fifo_valid && (fifo_do[31:30] == T_PIXEL) && fifo_do[24];
            ld_decode_error <= (fifo_valid && (fifo_do[31:30] == T_ABORT)) ? 8'hAB : 8'h00;
            if (fifo_valid && (fifo_do[31:30] == T_START))
                ld_image_index <= fifo_do[1:0];
        end
    end

    // 纯数据寄存器不复位、不带使能（只在 ld_pixel_valid 时被 loader 使用）
    always @(posedge dst_clk)
        ld_pixel_rgb <= fifo_do[23:0];

    // ---------------- 反向 done + 错误码 ----------------
    reg       done_toggle_mem;
    reg [7:0] done_error_mem;
    always @(posedge dst_clk or negedge dst_rst_n) begin
        if (!dst_rst_n) begin
            done_toggle_mem <= 1'b0;
            done_error_mem  <= 8'h00;
        end else if (ld_load_done) begin
            done_toggle_mem <= ~done_toggle_mem;
            done_error_mem  <= ld_memory_error;
        end
    end

    (* async_reg = "true" *) reg done_sync1;
    (* async_reg = "true" *) reg done_sync2;
    reg done_sync3;
    always @(posedge src_clk or negedge src_rst_n) begin
        if (!src_rst_n) begin
            done_sync1 <= 1'b0;
            done_sync2 <= 1'b0;
            done_sync3 <= 1'b0;
            done_event <= 1'b0;
            done_error <= 8'h00;
        end else begin
            done_sync1 <= done_toggle_mem;
            done_sync2 <= done_sync1;
            done_sync3 <= done_sync2;
            done_event <= done_sync2 ^ done_sync3;
            if (done_sync2 ^ done_sync3)
                done_error <= done_error_mem;
        end
    end

    genvar gi;
    generate
        for (gi = 0; gi < 4; gi = gi + 1) begin : g_complete_sync
            sync_2ff u_sync (
                .clk      (src_clk),
                .rst_n    (src_rst_n),
                .async_in (ld_image_complete[gi]),
                .sync_out (image_complete_sys[gi])
            );
        end
    endgenerate
endmodule
