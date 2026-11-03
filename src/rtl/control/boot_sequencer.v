`timescale 1ns/1ps
// -----------------------------------------------------------------------------
//   复位释放 → S_WAIT_SDRAM ──sdram_init_done──→ S_INGEST ──scan_done──→ S_CHECK
//                  │超时 0x59(仍等待)            │SD 3s 未就绪 0x5A(恢复即清)   │image_count==0 → 0x53(停在此)
//                                                                              ▼
//                                     S_RUN(player_enable=1) ←─31 拍写口空闲── S_DRAIN
// -----------------------------------------------------------------------------
module boot_sequencer #(
    parameter integer SDRAM_TIMEOUT_MS  = 100,     // EG4 片内 SDRAM 初始化约 0.2 ms
    parameter integer SD_READY_MS       = 3000,    // 包含 sd_card_bmp 的 1.5 s 重新初始化看门狗 1 次
    parameter integer DRAIN_CYCLES      = 31,
    parameter integer ERR_REPEAT_MS     = 2000
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       tick,                // 公共 1ms 节拍（tb 接 1 = 每拍 1 ms）
    input  wire       sdram_init_done,     // 已同步到 clk 域
    input  wire       sd_init_done,        // sd_card_bmp：SD 卡初始化完成且未在软复位
    input  wire       scan_done,           // 资源可用（首图确认或整卡扫完）
    input  wire [6:0] image_count,
    input  wire       write_busy,          // 写口活动（write_req / write_en / 加载进行中）
    output reg        ingest_enable,
    output reg        player_enable,
    output reg  [7:0] boot_error,
    output reg        error_event,
    output reg  [2:0] boot_state
);

localparam [2:0] S_WAIT_SDRAM = 3'd0,
                 S_INGEST     = 3'd1,
                 S_CHECK      = 3'd2,
                 S_DRAIN      = 3'd3,
                 S_RUN        = 3'd4;

localparam [7:0] ERR_NONE          = 8'h00;
localparam [7:0] ERR_NO_RESOURCE   = 8'h53;    // 与 media_player_control 同码：整卡没有可用 BMP
localparam [7:0] ERR_SDRAM_TIMEOUT = 8'h59;
localparam [7:0] ERR_SD_NOT_READY  = 8'h5A;

localparam integer TMAX = (SD_READY_MS > SDRAM_TIMEOUT_MS) ? SD_READY_MS : SDRAM_TIMEOUT_MS;
localparam integer TW   = $clog2(TMAX + 1);
localparam integer RW   = $clog2(ERR_REPEAT_MS + 1);

reg [TW-1:0] timer;
reg [RW-1:0] err_repeat;
reg [5:0]  drain_cnt;
reg [7:0]  err_next;

always @(*) begin
    err_next = ERR_NONE;
    case (boot_state)
        S_WAIT_SDRAM: if (timer >= SDRAM_TIMEOUT_MS) err_next = ERR_SDRAM_TIMEOUT;
        S_INGEST:     if (timer >= SD_READY_MS)      err_next = ERR_SD_NOT_READY;
        S_CHECK:      if (image_count == 7'd0)           err_next = ERR_NO_RESOURCE;
        default:      err_next = ERR_NONE;
    endcase
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        boot_state    <= S_WAIT_SDRAM;
        ingest_enable <= 1'b0;
        player_enable <= 1'b0;
        boot_error    <= ERR_NONE;
        error_event   <= 1'b0;
        timer         <= 0;
        err_repeat    <= 0;
        drain_cnt     <= 6'd0;
    end else begin
        // ---- 错误码：电平 + 事件（新错误立即、持续期间周期重发）----
        boot_error  <= err_next;
        error_event <= 1'b0;
        if (err_next != ERR_NONE) begin
            if (err_next != boot_error || (tick && err_repeat >= ERR_REPEAT_MS - 1)) begin
                error_event <= 1'b1;
                err_repeat  <= 0;
            end else if (tick) begin
                err_repeat  <= err_repeat + 1'b1;
            end
        end else begin
            err_repeat <= 0;
        end

        case (boot_state)
            S_WAIT_SDRAM: begin
                if (sdram_init_done) begin
                    boot_state    <= S_INGEST;
                    ingest_enable <= 1'b1;
                    timer         <= 0;
                end else if (tick && timer < SDRAM_TIMEOUT_MS) begin
                    timer <= timer + 1'b1;
                end
            end
            S_INGEST: begin
                if (scan_done) begin
                    boot_state <= S_CHECK;
                    timer      <= 0;
                end else if (sd_init_done) begin
                    timer <= 0;                    // SD 就绪（含拔插卡后恢复）→ 清 0x5A
                end else if (tick && timer < SD_READY_MS) begin
                    timer <= timer + 1'b1;
                end
            end
            S_CHECK: begin
                // scan_done 且 image_count==0 只可能是整卡扫完没有图：停在这里报 0x53，等复位
                if (image_count != 7'd0) begin
                    boot_state <= S_DRAIN;
                    drain_cnt  <= 6'd0;
                end
            end
            S_DRAIN: begin
                if (write_busy)
                    drain_cnt <= 6'd0;
                else if (drain_cnt >= DRAIN_CYCLES - 1) begin
                    boot_state    <= S_RUN;
                    player_enable <= 1'b1;
                end else
                    drain_cnt <= drain_cnt + 6'd1;
            end
            default: begin                             // S_RUN：只有复位才离开
                boot_state    <= S_RUN;
                ingest_enable <= 1'b1;
                player_enable <= 1'b1;
            end
        endcase
    end
end

endmodule
