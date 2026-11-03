`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// wav_playlist_ctrl —— 播放列表控制（课程演进路径"音频功能完善：播放列表"），SD 引擎时钟域
//   产生 wav_stream_player 的 play_enable / start_lba / total_sectors
// 曲目 = TF 卡上的 TRACK1~TRACK16.PCM（fat32_volume_scanner 识别，
//   pcm_track_locator 换算扇区）。
// 音画同步：曲目由当前图片决定，一一对应。
//   select = 当前图片的 {无曲目, 曲号}（顶层：第 k 张图（k = 0..15）→ TRACK(k+1)，k >= 16 → 无曲目位 = 1）；
//   卡上没有这首（或 select = 无曲目）→ 不播，switching = 0，音源回到琶音（不顺延到别的曲目）。
//   select 变化（KEY2 切图 / 自动轮播，图片提交之后）→ 停掉当前曲，从新曲目开头播放。
//   停留在同一张图：LOOP=1 本曲播完从头再播（单曲循环）；LOOP=0 播完停住，直到换图或 enable 重新拉高。
//
//   S_IDLE   enable 且本图有曲目 → 锁存 select → S_LOOKUP
//   S_LOOKUP 向 locator 请求该曲目的 {start_lba, total_sectors}，等 info_valid → S_START（期间换图 → S_IDLE 重算）
//   S_START  等播放器空闲（wr_idle：写侧 IDLE 且读侧已停止）→ play_enable=1 → S_PLAY
//   S_PLAY   换图 / enable 拉低 / wr_done（读完且播空）/ wr_error（扇区错误）→ S_STOP
//   S_STOP   play_enable=0，等 wr_idle → S_IDLE（同曲重播或换曲），LOOP=0 播完 → S_HOLD
//   出错也是重播本曲：拔卡时仲裁器把请求留到 SD 恢复，插回后本图的曲子接着从头播。
// switching：停曲 → 新曲开播之间的空档（几十拍到约 1ms）为 1。顶层把它并进 wav_active，空档里音源选择器
// -----------------------------------------------------------------------------
module wav_playlist_ctrl #(
    parameter integer LOOP = 1
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        enable,            // SW1 & 启动序列放行
    input  wire [15:0] track_valid,       // 扫描结果：TRACK1~16.PCM 是否存在
    input  wire [4:0]  select,            // 当前图片的 {无曲目, 曲号 0..15 = TRACK1..16}（电平）
    // pcm_track_locator
    output reg         info_req,          // 单拍
    output reg  [3:0]  info_idx,
    input  wire        info_valid,        // 单拍，start/total 同拍有效且保持
    input  wire [31:0] info_start_lba,
    input  wire [31:0] info_total_sectors,
    // wav_stream_player 写侧
    output reg         play_enable,
    output reg  [31:0] start_lba,
    output reg  [31:0] total_sectors,
    input  wire        wr_idle,
    input  wire        wr_done,
    input  wire        wr_error,
    // 状态
    output wire [3:0]  track,             // 当前曲目号 0..15 = TRACK1..16
    output wire        playing,
    output wire        switching
);
    localparam [2:0] S_IDLE   = 3'd0;
    localparam [2:0] S_LOOKUP = 3'd1;
    localparam [2:0] S_START  = 3'd2;
    localparam [2:0] S_PLAY   = 3'd3;
    localparam [2:0] S_STOP   = 3'd4;
    localparam [2:0] S_HOLD   = 3'd5;

    reg [2:0] state;
    reg [4:0] sel_q;                      // 本曲对应的 select，与 select 不等 = 换图了
    reg       finished;                   // 本曲是播完停的（LOOP=0 才用到）

    wire        has        = !select[4] && track_valid[select[3:0]];   // 本图的曲目在卡上
    wire        sel_change = (select != sel_q);

    assign track     = sel_q[3:0];
    assign playing   = (state == S_PLAY);
    assign switching = enable && has && (state != S_PLAY) && (state != S_HOLD);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= S_IDLE;
            sel_q         <= 5'd0;
            finished      <= 1'b0;
            info_req      <= 1'b0;
            info_idx      <= 4'd0;
            play_enable   <= 1'b0;
            start_lba     <= 32'd0;
            total_sectors <= 32'd0;
        end else begin
            info_req <= 1'b0;
            case (state)
            S_IDLE: begin
                if (enable && has) begin
                    sel_q    <= select;
                    info_idx <= select[3:0];
                    info_req <= 1'b1;
                    state    <= S_LOOKUP;
                end
            end
            S_LOOKUP: begin
                if (info_valid) begin
                    start_lba     <= info_start_lba;
                    total_sectors <= info_total_sectors;
                    state         <= sel_change ? S_IDLE : S_START;
                end
            end
            S_START: begin
                if (!enable || sel_change)
                    state <= S_IDLE;
                else if (wr_idle) begin
                    play_enable <= 1'b1;
                    state       <= S_PLAY;
                end
            end
            S_PLAY: begin
                if (!enable || sel_change || wr_done || wr_error) begin
                    play_enable <= 1'b0;
                    finished    <= wr_done && enable && !sel_change;
                    state       <= S_STOP;
                end
            end
            S_STOP: begin
                // wr_idle 需要读侧确认已停止：play_enable 拉低后至少几拍才会成立
                if (wr_idle && !play_enable) begin
                    state    <= (LOOP == 0 && finished) ? S_HOLD : S_IDLE;
                    finished <= 1'b0;
                end
            end
            S_HOLD: begin
                if (!enable || sel_change)
                    state <= S_IDLE;
            end
            default: state <= S_IDLE;
            endcase
        end
    end
endmodule
