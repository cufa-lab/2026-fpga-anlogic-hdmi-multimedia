`timescale 1ns/1ps
// -----------------------------------------------------------------------------
//   TF 扇区字节（写侧 wr_clk = 本工程 SD 引擎时钟 sys_clk 50MHz，字节串行）
//     → 小端两字节拼 16 位样本 → 双时钟 BRAM 循环缓冲（4096x16 = 8KB ≈ 85ms @48kHz）
//   48kHz sample_tick（读侧 rd_clk = pixel_clk 25MHz，音频链所在时钟）
//     → 从 BRAM 取样 → valid/ready PCM 接口
// 数据格式：PC 预处理成 48kHz / 16 位有符号 / 小端 / 单声道的
// 裸 PCM，FPGA 只需要 start_lba + total_sectors，按扇区顺序流式读取。
// 状态口（读侧 rd_clk 域）：sample_counter 已播放的真实样本数（OSD 进度用）、stream_active、stream_end、
// buffer_low（< 25%）。写侧 wr_clk 域：wr_idle（可以开始下一曲）、wr_done（本曲已读完且全部播完）、
// wr_error（本曲因扇区错误提前结束）——播放列表控制器用。
// -----------------------------------------------------------------------------
module wav_stream_player #(
    parameter integer ADDR_WIDTH = 12,         // 4096 x 16-bit = 8 KB
    parameter integer SAMPLES_PER_SECTOR = 256 // 512 bytes / 2
) (
    // write side: SD 引擎时钟域
    input  wire        wr_clk,
    input  wire        wr_rst_n,
    input  wire        play_enable,        // 电平：播放本曲；拉低 = 停止
    input  wire [31:0] start_lba,          // PCM 数据首扇区
    input  wire [31:0] total_sectors,      // PCM 数据扇区数
    output wire        sd_req,             // 单拍：读一个扇区（sd_lba 同拍有效，保持到 sd_done）
    output wire [31:0] sd_lba,
    input  wire        sd_busy,
    input  wire        sd_valid,           // 每个有效字节一个脉冲
    input  wire [7:0]  sd_byte,
    input  wire [8:0]  sd_byte_index,      
    input  wire        sd_done,
    input  wire [7:0]  sd_error,
    output wire        wr_idle,
    output wire        wr_done,
    output reg         wr_error,

    // read side: 音频链时钟域
    input  wire        rd_clk,
    input  wire        rd_rst_n,
    input  wire        sample_tick,        // ~48 kHz 单拍：允许输出一个样本
    output reg  signed [15:0] sample_out,
    output reg         sample_valid,
    input  wire        sample_ready,

    // status (read side)
    output reg  [31:0] sample_counter,
    output wire        stream_active,
    output wire        stream_end,
    output wire        buffer_low
);
    localparam integer PW    = ADDR_WIDTH + 1;              // 指针位宽（多 1 位区分满/空）
    localparam integer DEPTH = 1 << ADDR_WIDTH;

    // ---------------- 双时钟 BRAM 循环缓冲 ----------------
    (* ram_style = "bram" *) reg [15:0] buffer [0:DEPTH-1];
    reg [PW-1:0] wr_ptr;                // wr_clk 域
    reg [PW-1:0] rd_ptr;                // rd_clk 域

    function [PW-1:0] to_gray;
        input [PW-1:0] b;
        begin
            to_gray = b ^ (b >> 1);
        end
    endfunction

    function [PW-1:0] from_gray;
        input [PW-1:0] g;
        reg [PW-1:0] b;
        integer i;
        begin
            b[PW-1] = g[PW-1];
            for (i = PW-2; i >= 0; i = i - 1)
                b[i] = b[i+1] ^ g[i];
            from_gray = b;
        end
    endfunction

    // 格雷码：源域寄存一拍（无毛刺、每次只变 1 位）→ 目的域 2FF
    reg [PW-1:0] wr_gray;                               // wr_clk 域
    reg [PW-1:0] wr_gray_sync1, wr_gray_sync2;          // rd_clk 域
    reg [PW-1:0] rd_gray;                               // rd_clk 域
    reg [PW-1:0] rd_gray_sync1, rd_gray_sync2;          // wr_clk 域

    wire [PW-1:0] wr_ptr_in_rd = from_gray(wr_gray_sync2);
    wire [PW-1:0] rd_ptr_in_wr = from_gray(rd_gray_sync2);

    // 写域水位（流控用，读指针滞后 → 低估空余，安全）；读域水位（播放/下溢判断，写指针滞后 → 低估可读，安全）
    wire [PW-1:0] level_wr = wr_ptr - rd_ptr_in_wr;
    wire [PW-1:0] level_rd = wr_ptr_in_rd - rd_ptr;
    wire          space_ok = (level_wr <= DEPTH - SAMPLES_PER_SECTOR);

    // ---------------- 写侧状态机----------------
    localparam [2:0] WS_IDLE  = 3'd0;
    localparam [2:0] WS_REQ   = 3'd1;
    localparam [2:0] WS_FILL  = 3'd2;
    localparam [2:0] WS_DONE  = 3'd3;
    localparam [2:0] WS_END   = 3'd4;
    localparam [2:0] WS_ABORT = 3'd5;   // 停止时在途扇区还没 done：丢弃剩余字节，等 done

    reg [2:0]  wstate;
    reg [31:0] cur_lba;
    // L15：剩余扇区 20 位 = 最多 2^20 扇区（512 MB，单声道 48 kHz 约 91 分钟），远超实际曲目；
    //   更大的文件只播前 (扇区数 mod 2^20) 个扇区。起播判据 total_sectors != 0 仍看完整 32 位
    localparam integer SL_W = 20;
    reg [SL_W-1:0] sectors_left;        // 剩余扇区
    reg [7:0]  lo_byte;                 // 小端拼字：先到的是低字节
    reg        lo_pending;
    reg        wr_ended;                // = (wstate == WS_END)，寄存后跨域

    // 读侧 → 写侧回执
    reg        rd_armed;                // rd 域：已看到 play_enable，读指针停止跟随
    reg        rd_drained;              // rd 域：写侧已结束且缓冲已播空
    reg        armed_sync1, armed_sync2;
    reg        drained_sync1, drained_sync2;

    assign sd_req  = (wstate == WS_REQ) && !sd_busy && space_ok;
    assign sd_lba  = cur_lba;
    assign wr_idle = (wstate == WS_IDLE) && !armed_sync2;
    assign wr_done = wr_ended && drained_sync2;

    always @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) begin
            wstate        <= WS_IDLE;
            cur_lba       <= 32'd0;
            sectors_left  <= {SL_W{1'b0}};
            wr_ptr        <= {PW{1'b0}};
            wr_gray       <= {PW{1'b0}};
            lo_pending    <= 1'b0;
            wr_ended      <= 1'b0;
            wr_error      <= 1'b0;
            rd_gray_sync1 <= {PW{1'b0}};
            rd_gray_sync2 <= {PW{1'b0}};
            armed_sync1   <= 1'b0;
            armed_sync2   <= 1'b0;
            drained_sync1 <= 1'b0;
            drained_sync2 <= 1'b0;
        end else begin
            wr_gray       <= to_gray(wr_ptr);
            rd_gray_sync1 <= rd_gray;
            rd_gray_sync2 <= rd_gray_sync1;
            armed_sync1   <= rd_armed;
            armed_sync2   <= armed_sync1;
            drained_sync1 <= rd_drained;
            drained_sync2 <= drained_sync1;

            case (wstate)
            WS_IDLE: begin
                // 读侧已就位（读指针 = 写指针，缓冲为空）才开始；参数为 0 不播
                if (play_enable && armed_sync2 && start_lba != 32'd0 && total_sectors != 32'd0) begin
                    cur_lba      <= start_lba;
                    sectors_left <= total_sectors[SL_W-1:0];
                    lo_pending   <= 1'b0;
                    wr_error     <= 1'b0;
                    wstate       <= WS_REQ;
                end
            end
            WS_REQ: begin
                if (!play_enable)
                    wstate <= WS_IDLE;
                else if (sd_req)                // 空余 >= 1 扇区且 SD 口空闲：发请求
                    wstate <= WS_FILL;
            end
            WS_FILL: begin
                if (sd_valid) begin
                    if (!lo_pending) begin
                        lo_byte    <= sd_byte;
                        lo_pending <= 1'b1;
                    end else begin
                        wr_ptr     <= wr_ptr + 1'b1;     // 数据写在下面的 BRAM 写口
                        lo_pending <= 1'b0;
                    end
                end
                if (sd_done) begin
                    lo_pending <= 1'b0;
                    if (sd_error != 8'd0) begin
                        wr_error <= 1'b1;
                        wstate   <= WS_END;             // 扇区读错：结束本曲
                    end else begin
                        cur_lba      <= cur_lba + 1'b1;
                        sectors_left <= sectors_left - 1'b1;
                        if (sectors_left == {{(SL_W-1){1'b0}}, 1'b1})
                            wstate <= WS_END;           // 全部读完
                        else if (!play_enable)
                            wstate <= WS_IDLE;
                        else
                            wstate <= WS_REQ;           // 读下一个扇区
                    end
                end else if (!play_enable) begin
                    wstate <= WS_ABORT;
                end
            end
            WS_ABORT: begin
                if (sd_done)
                    wstate <= WS_IDLE;
            end
            WS_DONE: wstate <= WS_REQ;                  // 保留未用
            WS_END: begin
                if (!play_enable) begin
                    wstate   <= WS_IDLE;                // 等播放停止，不自动重播
                    wr_error <= 1'b0;                   // 与 wr_done 同时撤销：播放列表启动下一曲时不会看到上一曲的错误
                end
            end
            default: wstate <= WS_IDLE;
            endcase

            wr_ended <= (wstate == WS_END) && play_enable;
        end
    end

    // BRAM 写口（无复位）
    always @(posedge wr_clk) begin
        if (wstate == WS_FILL && sd_valid && lo_pending)
            buffer[wr_ptr[ADDR_WIDTH-1:0]] <= {sd_byte, lo_byte};
    end

    // ---------------- 读侧 ----------------
    reg        en_sync1, en_rd;
    reg        ended_sync1, ended_rd;
    reg [2:0]  credits;                 // 未送出的采样节拍额度（饱和 7）
    reg [1:0]  settle;                  // 读指针变化后等 BRAM 读数据跟上（同步读 1 拍 + 余量 1 拍）
    reg        cur_real;                // 当前 sample_out 来自缓冲（握手后推进读指针）
    reg [15:0] rd_data;

    wire take      = sample_valid && sample_ready;
    wire credit_up;

    // BRAM 读口（无复位，同步读）
    // P3 低功耗：只有 sample_out 装入 rd_data 的那一拍（!sample_valid && credits != 0 && settle == 0）用得到
    //   上一拍读出的数据。那一拍 credits != 0 → 上一拍 credits != 0 或 credit_up；读指针只在 take / 停止时变，
    //   同时 settle 置 2 挡住装入。所以只在 credit_up || credits != 0 || settle != 0 时读，装入的数据与
    //   每拍都读逐拍相同；缓冲空时写进来的数据由 level_rd（格雷码同步，晚于写入）放行，同样能读到。
    //   48kHz 采样、25MHz 读时钟，读口绝大部分时间停住
    wire rd_en = credit_up || (credits != 3'd0) || (settle != 2'd0);
    always @(posedge rd_clk)
        if (rd_en)
            rd_data <= buffer[rd_ptr[ADDR_WIDTH-1:0]];

    // 可读 >= 1 扇区才播（预填充 256），或写侧已结束且还有残余 → 排空
    // primed：预填充达标后置 1，缓冲播空（下溢）或停止时清 0 → 下溢后重新攒够 1 扇区再播，不会一个样本一个样本地断续
    reg        primed;
    wire can_play  = en_rd && (level_rd != 0) &&
                     (primed || (level_rd >= SAMPLES_PER_SECTOR) || ended_rd);
    assign credit_up = sample_tick && (credits != 3'd7);

    assign stream_active = en_rd && !rd_drained;
    assign stream_end    = rd_drained;
    assign buffer_low    = en_rd && (level_rd < (DEPTH >> 2));      // < 25%

    always @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) begin
            rd_ptr         <= {PW{1'b0}};
            rd_gray        <= {PW{1'b0}};
            wr_gray_sync1  <= {PW{1'b0}};
            wr_gray_sync2  <= {PW{1'b0}};
            en_sync1       <= 1'b0;
            en_rd          <= 1'b0;
            ended_sync1    <= 1'b0;
            ended_rd       <= 1'b0;
            rd_armed       <= 1'b0;
            rd_drained     <= 1'b0;
            primed         <= 1'b0;
            credits        <= 3'd0;
            settle         <= 2'd0;
            cur_real       <= 1'b0;
            sample_out     <= 16'sd0;
            sample_valid   <= 1'b0;
            sample_counter <= 32'd0;
        end else begin
            wr_gray_sync1 <= wr_gray;
            wr_gray_sync2 <= wr_gray_sync1;
            en_sync1      <= play_enable;
            en_rd         <= en_sync1;
            ended_sync1   <= wr_ended;
            ended_rd      <= ended_sync1;
            rd_armed      <= en_rd;
            rd_drained    <= en_rd && ended_rd && (level_rd == 0);
            rd_gray       <= to_gray(rd_ptr);

            if (!en_rd || level_rd == {PW{1'b0}})
                primed <= 1'b0;                     // 停止 / 播空
            else if (level_rd >= SAMPLES_PER_SECTOR)
                primed <= 1'b1;                     // 预填充 1 扇区

            if (settle != 2'd0)
                settle <= settle - 2'd1;

            // 节拍额度：tick +1，握手 -1
            case ({credit_up, take})
                2'b10: credits <= credits + 3'd1;
                2'b01: credits <= credits - 3'd1;
                default: ;
            endcase

            if (take) begin
                sample_valid <= 1'b0;
                if (cur_real) begin
                    rd_ptr         <= rd_ptr + 1'b1;
                    sample_counter <= sample_counter + 1'b1;
                    settle         <= 2'd2;
                end
            end else if (!sample_valid && credits != 3'd0 && settle == 2'd0) begin
                // 一旦给出 valid，数据保持到握手（PCM 合同）
                sample_valid <= 1'b1;
                if (can_play) begin
                    sample_out <= rd_data;          // 正常播放 / 排空残余
                    cur_real   <= 1'b1;
                end else begin
                    sample_out <= 16'sd0;           // 预填充或下溢：静音，不消费读指针
                    cur_real   <= 1'b0;
                end
            end

            if (!en_rd) begin
                // 停止：读指针跟随写指针（缓冲视为空），计数/额度清零。stream_active 与 en_rd 同拍拉低，
                // 音源选择器同拍切走，所以这里撤掉未被接收的样本对下游不可见
                rd_ptr         <= wr_ptr_in_rd;
                sample_counter <= 32'd0;
                credits        <= 3'd0;
                cur_real       <= 1'b0;
                settle         <= 2'd2;
                sample_valid   <= 1'b0;
                sample_out     <= 16'sd0;
            end
        end
    end
endmodule
