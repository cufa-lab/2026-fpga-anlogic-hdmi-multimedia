`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// picture_res_info —— 参数条的"当前图片分辨率"和"切到新画面"事件
//
// 分辨率：每个 bank 记一条 {宽, 高}。
//   sys 域：sd_card_bmp 报 load_done 的那一拍，头寄存器 src_width/src_height 仍是本次加载的
//     （load_busy 同拍才清，下一次加载的 BMP 头要很久以后才到），连同 loaded_buffer_idx 一起锁存一条，
//     同拍翻转 entry_tog
//   → 跨域：一条记录要等下一次 load_done（>= 一次整图加载，毫秒级）才会再变，所以不用通用握手的两份副本：
//     entry_tog 经 2FF（req_sync1/2，已被 SDC */req_sync1 覆盖）到 pix 域，看到翻转时 entry_sys 早已稳定，
//     直接写表（entry_sys → 表是准静态载荷，SDC 第 14 节 false path）
//   → pix 域 4×24 位表（不复位，可映射成分布式 RAM）
//   读：地址 = active_buffer_idx（与 reader 同一个 frame_start 提交），换图那一帧参数条锁存的就是新图分辨率。
//   全屏时当前 bank 一定装着当前图（总览快速提交未驻留的图，会先补齐整图才放回全屏，见 view_hold）；
//   总览里 bank 不一定对应焦点图 → res_valid = 0（参数条显示 "----X----"）。首图出现前同样无效。
//   约定：res_width/res_height 只在帧边界附近变化（提交）或写非当前 bank，promax_text_overlay 在 frame_start
//   后 12 拍内逐位读取它们
// 切到新画面：全屏下图片号变化（KEY2 下一张 / 上一张、自动轮播）、从总览回到全屏（SW4 拨下选定）、首图出现。
//   active_image_idx / thumb_display / display_valid 都只在帧边界变化 → pic_show 单拍
// -----------------------------------------------------------------------------
module picture_res_info (
    input  wire        sys_clk,
    input  wire        sys_rst_n,
    input  wire        load_done,          // sd_card_bmp：单拍，加载成功
    input  wire [1:0]  loaded_buffer_idx,
    input  wire [11:0] src_width,          // sd_card_bmp 当前（最近一次）BMP 头的宽 / 高
    input  wire [11:0] src_height,

    input  wire        pix_clk,
    input  wire        pix_rst_n,
    input  wire        display_valid,
    input  wire        thumb_display,      // 1 = 总览（reader 读缩略图区）
    input  wire [1:0]  active_buffer_idx,
    input  wire [5:0]  active_image_idx,
    output wire        res_valid,
    output wire [11:0] res_width,
    output wire [11:0] res_height,
    output wire        pic_show
);
    // ---------------- sys：加载成功时锁存 {bank, 宽, 高} + 翻转 ----------------
    reg  [25:0] entry_sys;                 // 数据，不复位
    reg         entry_tog;
    always @(posedge sys_clk) begin
        if (load_done)
            entry_sys <= {loaded_buffer_idx, src_width, src_height};
    end
    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n)
            entry_tog <= 1'b0;
        else if (load_done)
            entry_tog <= ~entry_tog;
    end

    // ---------------- pix：翻转同步 → 写表 ----------------
    (* async_reg = "true" *) reg req_sync1;
    (* async_reg = "true" *) reg req_sync2;
    reg         req_q;
    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            req_sync1 <= 1'b0;
            req_sync2 <= 1'b0;
            req_q     <= 1'b0;
        end else begin
            req_sync1 <= entry_tog;
            req_sync2 <= req_sync1;
            req_q     <= req_sync2;
        end
    end

    reg  [23:0] res_table [0:3];
    always @(posedge pix_clk) begin
        if (req_sync2 != req_q)
            res_table[entry_sys[25:24]] <= entry_sys[23:0];
    end

    wire [23:0] res_cur = res_table[active_buffer_idx];
    assign res_width  = res_cur[23:12];
    assign res_height = res_cur[11:0];
    assign res_valid  = display_valid && !thumb_display && (res_cur[23:12] != 12'd0);

    // ---------------- 切到新画面 ----------------
    wire       pic_full = display_valid && !thumb_display;
    reg        pic_full_q;
    reg  [5:0] pic_idx_q;
    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            pic_full_q <= 1'b0;
            pic_idx_q  <= 6'd0;
        end else begin
            pic_full_q <= pic_full;
            pic_idx_q  <= active_image_idx;
        end
    end
    assign pic_show = pic_full && (!pic_full_q || (active_image_idx != pic_idx_q));
endmodule
