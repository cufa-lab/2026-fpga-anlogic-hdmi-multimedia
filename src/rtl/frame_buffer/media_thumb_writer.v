`timescale 1ns/1ps

module media_thumb_writer #(
    parameter integer ADDR_WIDTH = 21,
    parameter integer BASE       = 1228800,
    parameter integer BURST      = 32,
    parameter integer SUB_LOG2   = 2,        
    parameter integer DUAL       = 0,        // 1 = 两块缩略图区按页号奇偶交替
    parameter integer AREA_WORDS = 307200    // 一块缩略图区的大小（第二块基址 = BASE + AREA_WORDS）
) (
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  clear,
    input  wire [5:0]            image_idx,      // 准静态：本次加载的图片号（第一个像素时采样）
    input  wire                  gen,            // 准静态：1 = 本次加载要写缩略图（与 image_idx 一起采样）
    // 旁听 media_frame_load_cdc 输出（与 loader 同一条流）
    input  wire                  ld_load_start,
    input  wire                  ld_pixel_valid,
    input  wire [23:0]           ld_pixel_rgb,
    input  wire                  ld_frame_commit,
    input  wire [7:0]            ld_decode_error,
    // 写口
    input  wire                  controller_init_done,
    output wire                  burst_want,
    input  wire                  burst_grant,
    output wire                  burst_busy,
    output reg                   app_wr_en,
    output reg  [ADDR_WIDTH-1:0] app_wr_addr,
    output wire [3:0]            app_wr_mask,
    output wire [31:0]           app_wr_data,
    // 状态：条目 e = {区, slot}
    output reg  [7:0]            thumb_valid,
    output reg  [31:0]           thumb_tags       // {tag7, ..., tag0}
);
    localparam integer F     = 1 << SUB_LOG2;
    localparam integer CW    = 640 / F;
    localparam integer CH    = 480 / F;
    localparam integer WORDS = CW * CH;
    localparam [ADDR_WIDTH-1:0] ROW_BACK = 640 + CW - 1;   // 本行末 +1 → 上一缩略图行行首

    reg        active;          // 本次加载在收像素
    reg        first_px;        // 下一个像素是本帧第一个
    reg        commit_seen;
    reg        flush_pend;      // 丢弃 FIFO 残余（中止 / 新加载），等进行中的 burst 写完再清
    reg [2:0]  ent;             // 当前条目 {区, slot}
    reg [3:0]  tag;
    reg [9:0]  col;
    reg [8:0]  row;
    reg [7:0]  wptr, rptr;
    reg [8:0]  cnt;             // FIFO 占用
    reg [16:0] written;         // 已写字数
    reg        burst_active;
    reg [5:0]  burst_left;
    reg [8:0]  lx;              // 当前缩略图行内已写字数（到 CW 跳行）
    reg [ADDR_WIDTH-1:0] addr;
    wire [23:0] fifo_rdata;

    wire sample   = active && !flush_pend && ld_pixel_valid &&
                    ((col & (F - 1)) == 0) && ((row & (F - 1)) == F - 1);
    wire pop      = burst_active && (cnt != 9'd0);
    assign burst_want  = controller_init_done && !burst_active && !flush_pend && (cnt >= BURST);
    assign burst_busy  = burst_active || app_wr_en;
    assign app_wr_mask = 4'b0000;
    assign app_wr_data = {8'h00, fifo_rdata};

    fat32_sdp_ram #(.AW(8), .DW(24)) u_fifo (
        .clk(clk), .we(sample && cnt != 9'd256), .waddr(wptr), .wdata(ld_pixel_rgb),
        .re(pop), .raddr(rptr), .rdata(fifo_rdata));

    // 第一个缩略图行（ly = CH-1）的起址：8 个常数直接选
    // （写成 (s[1]?CH:0 + CH-1)*640 会被综合成变量乘法占 1 个 DSP）
    function [ADDR_WIDTH-1:0] first_addr(input [2:0] e);
        case (e)
            3'd0: first_addr = BASE + (CH - 1) * 640;
            3'd1: first_addr = BASE + (CH - 1) * 640 + CW;
            3'd2: first_addr = BASE + (2 * CH - 1) * 640;
            3'd3: first_addr = BASE + (2 * CH - 1) * 640 + CW;
            3'd4: first_addr = BASE + AREA_WORDS + (CH - 1) * 640;
            3'd5: first_addr = BASE + AREA_WORDS + (CH - 1) * 640 + CW;
            3'd6: first_addr = BASE + AREA_WORDS + (2 * CH - 1) * 640;
            default: first_addr = BASE + AREA_WORDS + (2 * CH - 1) * 640 + CW;
        endcase
    endfunction

    wire [2:0] new_ent = {(DUAL != 0) && image_idx[2], image_idx[1:0]};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            active <= 1'b0; first_px <= 1'b0; commit_seen <= 1'b0; flush_pend <= 1'b0; ent <= 3'd0; tag <= 4'd0;
            col <= 10'd0; row <= 9'd0; wptr <= 8'd0; rptr <= 8'd0; cnt <= 9'd0; written <= 17'd0;
            burst_active <= 1'b0; burst_left <= 6'd0; lx <= 9'd0; addr <= {ADDR_WIDTH{1'b0}};
            app_wr_en <= 1'b0; app_wr_addr <= {ADDR_WIDTH{1'b0}};
            thumb_valid <= 8'd0; thumb_tags <= 32'd0;
        end else begin
            app_wr_en <= 1'b0;

            // ---- burst：放行后连续 BURST 拍（中止时也照常写完）----
            if (burst_want && burst_grant) begin
                burst_active <= 1'b1;
                burst_left   <= BURST;
            end
            if (pop) begin
                app_wr_en   <= 1'b1;
                app_wr_addr <= addr;
                rptr        <= rptr + 1'b1;
                written     <= written + 1'b1;
                if (lx == CW - 1) begin
                    lx   <= 9'd0;
                    addr <= addr - ROW_BACK;
                end else begin
                    lx   <= lx + 1'b1;
                    addr <= addr + 1'b1;
                end
                if (burst_left == 6'd1) burst_active <= 1'b0;
                burst_left <= burst_left - 1'b1;
            end
            cnt <= cnt + (sample && cnt != 9'd256) - pop;
            if (sample && cnt != 9'd256) wptr <= wptr + 1'b1;

            // ---- 像素流计数 ----
            if (ld_load_start) begin
                active <= 1'b1; first_px <= 1'b1; commit_seen <= 1'b0; flush_pend <= 1'b1;
                col <= 10'd0; row <= 9'd0;
            end else if (ld_decode_error != 8'h00) begin
                active <= 1'b0;                      // 中止：已采样的残余丢弃，格子保持无效
                flush_pend <= 1'b1;
            end else if (active && ld_pixel_valid) begin
                if (first_px) begin
                    first_px <= 1'b0;
                    if (!gen) begin
                        active <= 1'b0;              // 缩略图完好，本次不写，格子不动
                    end else begin
                        // 第一个像素：锁存条目 / 页号，作废该格旧图；上一帧没写完的残余丢弃
                        ent <= new_ent;
                        tag <= image_idx[5:2];
                        thumb_valid[new_ent] <= 1'b0;
                        addr    <= first_addr(new_ent);
                        lx      <= 9'd0;
                        written <= 17'd0;
                    end
                end
                if (col == 10'd639) begin
                    col <= 10'd0;
                    row <= row + 1'b1;
                end else
                    col <= col + 1'b1;
                if (ld_frame_commit) commit_seen <= 1'b1;
            end

            // 全部写完 → 置格子有效
            if (active && !first_px && commit_seen && written == WORDS && !burst_active) begin
                active <= 1'b0;
                thumb_valid[ent] <= 1'b1;
                case (ent)
                    3'd0: thumb_tags[3:0]   <= tag;
                    3'd1: thumb_tags[7:4]   <= tag;
                    3'd2: thumb_tags[11:8]  <= tag;
                    3'd3: thumb_tags[15:12] <= tag;
                    3'd4: thumb_tags[19:16] <= tag;
                    3'd5: thumb_tags[23:20] <= tag;
                    3'd6: thumb_tags[27:24] <= tag;
                    default: thumb_tags[31:28] <= tag;
                endcase
            end

            // 丢弃残余：进行中的 burst（放行时 FIFO ≥ BURST，一定写得完整）结束后再清指针
            if (flush_pend && !burst_active) begin
                rptr <= wptr; cnt <= 9'd0; flush_pend <= 1'b0;
            end

            if (clear) thumb_valid <= 8'd0;
        end
    end
endmodule
