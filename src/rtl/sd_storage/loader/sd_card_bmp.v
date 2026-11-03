`timescale 1ns/1ps

module sd_card_bmp #(
    // ---------------- 2.6 参数集中化 ----------------
    parameter integer CLK_FREQ_HZ        = 100_000_000,
    parameter [15:0]  BMP_WIDTH          = 16'd640,    // 只接受这个尺寸的 24bit BMP
    parameter [15:0]  BMP_HEIGHT         = 16'd480,

    parameter [31:0]  SCAN_START_SECTOR  = 32'd0,
    parameter [31:0]  SCAN_MAX_SECTOR    = 32'd131071,
    parameter [2:0]   SCAN_TARGET_COUNT  = 3'd4,
    parameter [15:0]  SD_READ_TIMEOUT_MS = 16'd500,    // 单扇区读超时（sd_sector_adapter）/ 帧写口应答超时
    parameter integer LOAD_TIMEOUT_MS    = 2000,       // 单张加载看门狗（640x480x24bit 正常约 0.6~0.8s）
    parameter [1:0]   MAX_RETRIES        = 2'd1,       // 失败后重试次数（总尝试 = 1 + MAX_RETRIES）
    parameter integer SD_REINIT_CYCLES   = 64,         // SD 软复位脉宽
    // 初始化迟迟不完成（无卡时 sd_card_cmd 要等 0xFFFF 字节≈2.6s 才重发 CMD0）→ 再复位一次；
    // 取值大于 SD 规范 ACMD41 上电初始化最长 1s，避免打断慢卡
    parameter integer SD_REINIT_RETRY_MS = 1500,
    // load_done，覆盖 SDRAM 控制器内最后一次突发写的流水
    parameter integer WRITE_DRAIN_CYCLES = 31,
    //  原则 6 防无限循环：单个文件 FAT 链最多遍历的簇数（640x480 图 900KB，4KB 簇也只要 226 簇）
    parameter integer MAX_VISITED_CLUSTERS = 8192,
    // 1 = 任意分辨率（1..4095），0 = 只收 BMP_WIDTH×BMP_HEIGHT
    parameter integer ANY_SIZE           = 0,
    // （第十五讲）第四轮：1 = 大图跳行读（需 ANY_SIZE=1）；1 = 接受 load_cancel
    parameter integer ROW_SKIP           = 0,
    parameter integer CANCEL_EN          = 0,
    parameter integer SMART              = 0,          // 两遍读（探测 + 构图窗口）
    parameter integer REFINE_EN          = 0,          // 1 = 接受 load_refine（导览局部重读，需 SMART=1）
    parameter integer EXT_MS_TICK        = 0           // 逻辑优化 L2：1 = 用外部公共 1ms 节拍 ext_ms_tick，去掉本地分频器
)(
    input                       clk,
    input                       rst_n,
    input                       ext_ms_tick,        // EXT_MS_TICK=1：顶层 sensor_tick_ms（同 clk 域单拍）；=0 不用，tb 可不接
    //  boot_sequencer 在 SDRAM 初始化完成(sync_2ff 同步到本域)后置 1，才开始上电扫描
    input                       ingest_enable,
    output                      sd_ready,           // SD 卡已初始化且不在软复位（boot_sequencer 0x5A 判断）

    // ---- 资源发现 ----
    output reg                  scan_done,
    output reg                  scan_all_done,
    output reg [6:0]            image_count,

    // ---- 加载请求/完成 ----
    output                      loader_ready,
    input                       load_request,
    input  [5:0]                load_image_idx,
    input  [1:0]                load_buffer_idx,
    output reg                  load_done,
    output reg [5:0]            loaded_image_idx,
    output reg [1:0]            loaded_buffer_idx,
    output reg [7:0]            load_error,
    input                       load_cancel,        
    output     [3:0]            bmp_state_code,     // {扫描器忙, 流读器忙, 加载引擎状态}，仅作观测
    output reg [7:0]            sd_reinit_count,    // SD 自恢复次数（观测/调试）
    output reg [7:0]            fs_error,           

    // ---- 帧加载器----
    output reg                  fb_load_start,      // 单拍：开始写 bank fb_load_bank
    output     [1:0]            fb_load_bank,
    output                      pixel_valid,
    output     [23:0]           pixel_rgb,          // {R, G, B}
    output                      frame_commit,       // 与本帧最后一个像素同拍
    output reg                  fb_load_abort,      // 单拍：本次加载作废
    input                       fb_done,            // 单拍：loader 结束一次加载（已在 clk 域）
    input      [7:0]            fb_error,           // 与 fb_done 同拍：0 = 整帧写完
    output                      fb_busy,            // loader 侧还有未结束的加载（boot_sequencer 排空用）
    output     [5:0]            fb_load_image,      // 本次加载的图片编号（fb_load_start 前稳定）
    input                       load_refine,        // SMART：与 load_request 同拍，1 = 导览局部重读
    output reg                  fb_load_refine,     // SMART：本次加载是局部重读（fb_load_start 前稳定）
    // ---- SMART：构图窗口（← bmp_smart_scaler，sm_map_valid 之后稳定到下一次 load_start）----
    input                       sm_map_valid,
    input      [11:0]           sm_c0,
    input      [11:0]           sm_r0b,
    input      [11:0]           sm_cw,
    input      [11:0]           sm_ch,
    input      [11:0]           sm_n,
    input      [9:0]            sm_d,

    // ---- 源图尺寸 + 反压（→ / ← bmp_scale_letterbox）----
    output reg                  src_geom_valid,     // 单拍：头校验通过，src_width/src_height 有效
    output     [11:0]           src_width,
    output     [11:0]           src_height,
    input                       src_hold,           // 1 = 流读器在扇区边界暂停
    input                       src_progress,       // 缩放器输出了一个像素（文件读完后它还要发完黑边，也算进展）

    // ---- WAV 音频：曲目信息 + 音频扇区读口（复用本模块唯一的 SD 控制器，与图片加载按扇区轮转）----
    output reg [15:0]           audio_track_valid,  // 扫描找到的 TRACK1~16.PCM（扫描结束时锁存，SD 软复位不丢）
    input                       audio_info_req,     // 单拍：查询曲目 audio_info_idx 的扇区范围
    input      [3:0]            audio_info_idx,
    output                      audio_info_valid,   // 单拍：下面两个值有效（保持到下一次查询）
    output     [31:0]           audio_info_start_lba,
    output     [31:0]           audio_info_total_sectors,
    input                       audio_sector_req,   // 扇区口协议同 sd_sector_adapter（busy=0 时单拍 req）
    input      [31:0]           audio_sector_lba,
    output                      audio_sector_busy,
    output                      audio_sector_valid,
    output     [7:0]            audio_sector_byte,
    output     [8:0]            audio_sector_byte_index,
    output                      audio_sector_done,
    output     [7:0]            audio_sector_error,
    //  讲解
    output reg [33:0]           narr_aux_valid,     // {FOCUS, HZK32, VOC16..1, LRC16..1}（扫描结束锁存）
    input                       narr_info_req,      // 单拍
    input      [5:0]            narr_info_idx,      // {类型, 曲号}，见 pcm_track_locator
    output                      narr_info_valid,    // 单拍；audio_info_start_lba / total_sectors / narr_info_size 有效
    output     [31:0]           narr_info_size,     // 文件字节数
    input                       narr_sector_req,
    input      [31:0]           narr_sector_lba,
    output                      narr_sector_busy,
    output                      narr_sector_valid,
    output                      narr_sector_done,
    output     [7:0]            narr_sector_error,

    // ---- TF 卡 SPI ----
    output                      SD_nCS,
    output                      SD_DCLK,
    output                      SD_MOSI,
    input                       SD_MISO
);

localparam [7:0] ERR_NONE          = 8'h00;
localparam [7:0] ERR_LOAD_TIMEOUT  = 8'h55;
localparam [7:0] ERR_BMP_HEADER    = 8'h56;
localparam [7:0] ERR_BAD_INDEX     = 8'h57;
localparam [7:0] ERR_BMP_TRUNCATED = 8'h58;
localparam [7:0] ERR_LOAD_CANCEL   = 8'h59;         
localparam [7:0] FS_ERR_SECTOR     = 8'h10;          // FAT32 模块：扇区读错（适配层超时）

localparam integer MS_CYCLES             = CLK_FREQ_HZ / 1000;
localparam integer SECTOR_TIMEOUT_CYCLES = MS_CYCLES * SD_READ_TIMEOUT_MS;
localparam integer MSW  = $clog2(MS_CYCLES + 1);                  // 1ms 分频计数器
localparam integer LTW  = $clog2(LOAD_TIMEOUT_MS + 2);            // 加载看门狗（毫秒）
localparam integer RTW  = $clog2(SD_REINIT_RETRY_MS + 2);         // 重新初始化看门狗（毫秒）
localparam [31:0] EXPECTED_PIXEL_COUNT = BMP_WIDTH * BMP_HEIGHT;
localparam [31:0] EXPECTED_PIXEL_BYTES = BMP_WIDTH * BMP_HEIGHT * 3;
localparam integer PCW  = $clog2(BMP_WIDTH * BMP_HEIGHT + 1);    // 像素计数位宽（640x480 → 19 位）

// 加载引擎（取代 bmp_read 的加载路径）
// 不再有帧写口 write_req/ack 握手（LD_ACK 状态删除），START 命令与像素走同一条有序流
localparam [1:0] LD_IDLE   = 2'd0;
localparam [1:0] LD_REST   = 2'd1;                   // SMART：第 1 遍读完，等构图窗口后重启流读器
localparam [1:0] LD_STREAM = 2'd2;                   // 文件字节流：头校验 → 拼像素

wire             sd_sec_read;
wire [31:0]      sd_sec_read_addr;
wire [7:0]       sd_sec_read_data;
wire             sd_sec_read_data_valid;
wire             sd_sec_read_end;
wire             sd_init_done;

reg              scan_start_pulse;
reg              scan_kicked;
reg              load_start_pulse;
reg              load_abort;

reg              load_busy;          // 已接收请求，尚未 done/终止错误
reg              start_pending;      // 等加载引擎空闲 / SD 初始化完成后发 load_start
reg [1:0]        retry_count;
reg [5:0]        busy_image_idx;
reg              source_started;     // 已看到加载引擎离开 IDLE
reg              fb_live;            // 本次尝试的 fb_load_start 已发出、loader 还没回执
reg              fb_outstanding;     // 有 fb_load_start 的回执(fb_done)还没收到（含已中止的）
reg              write_done_seen;    // 已收到本次尝试的 fb_done 且无错（整帧已写进 SDRAM）
reg [LTW-1:0]    load_timeout_ms;
reg              drain_active;       // 源读完 + SDRAM 写完，正在排空
reg [5:0]        drain_cnt;

reg              sd_soft_rst;        // → sd_card_top / 适配层 / FAT 模块，重新初始化 SD 卡
reg [7:0]        sd_rst_cnt;
reg              reinit_wait;        // 等 SD 初始化完成（上电或软复位之后）
reg [RTW-1:0]    reinit_ms;

// ---- 1ms 节拍（所有毫秒级超时共用）----
reg [MSW-1:0]    ms_div;
reg              ms_tick;

//=== FAT32 适配层信号 ===
wire        scanner_busy;
wire        scanner_done;
wire [7:0]  scanner_fs_error;
wire [7:0]  scanner_spc_w;
wire [31:0] scanner_fat_start_w;
wire [31:0] scanner_data_start_w;
wire [31:0] scanner_cl_base_w;        // L9：data_start - 2·spc
wire [31:0] scanner_max_cluster_w;
wire [6:0]  scanner_resource_count;
wire [15:0] scanner_track_valid;      // TRACK1~16.PCM
wire [33:0] scanner_aux_valid;        
wire        narr_flushed;             
wire        trk_cat_own;              // pcm_track_locator 正在用清单读口
wire [8:0]  trk_cat_raddr;
// 扫描器随 SD 软复位一起复位，卷布局参数锁存在这里；清单本身在 BRAM 里（不随复位丢失）
reg  [7:0]  scanner_spc;
reg  [31:0] scanner_fat_start;
reg  [31:0] scanner_data_start;      // 只给 tb 观察（综合时无负载被裁掉）
reg  [31:0] scanner_cl_base;
reg  [31:0] scanner_max_cluster;

// ---- 文件清单 BRAM：扫描期间归扫描器，之后本模块只读 ----
wire        scn_cat_we;
wire [8:0]  scn_cat_waddr;
wire [31:0] scn_cat_wdata;
wire [8:0]  scn_cat_raddr;
wire [31:0] cat_rdata;
reg  [8:0]  cat_raddr;
reg  [2:0]  cat_fetch;             // 取一项 {首簇, 大小}：4 → 0（BRAM 读 2 拍延迟 × 2 个字）

reg  [31:0] load_cluster;
reg  [31:0] load_file_size;
reg         use_streamer;          // 0=扇区读口给扫描器, 1=给流读器
// 流读器
reg         file_start_pulse;
wire        streamer_busy;
wire        streamer_done;
wire        streamer_byte_valid;
wire [7:0]  streamer_byte;
wire [31:0] streamer_offset;
wire [7:0]  streamer_fs_error;
reg         stream_rst;            // 流读器同步中止（寄存器输出做异步复位，无毛刺）
// 加载引擎 + BMP 头校验 + RGB 拼装
reg  [1:0]  ld_state;
reg         ld_error;              // 单拍：本次加载失败
reg  [7:0]  ld_error_code;
reg         ld_error_reinit;       // 失败原因需要重新初始化 SD（扇区读超时/应答超时）
reg         ld_abort;              // 头校验失败：立即中止流读器
reg  [1:0]  byte_state;            // 0=空闲, 1=读BMP头, 2=拼RGB
reg         hdr_fixed_ok;          // 签名/宽/高/平面/位深/压缩 逐字节与期望值一致
reg         hdr_dib_ok;            // DIB 头长度 >= 40
reg  [31:0] hdr_file_len;
reg  [31:0] data_offset;           // BMP 头偏移 10 的像素数据偏移量
reg         pix_on;                // 已到达像素区
reg  [1:0]  rgb_byte_cnt;          // 0=B, 1=G, 2=R
reg  [23:0] rgb_pixel;             // 拼装中的像素 {R, G, B}
reg         bmp_data_wr_en;
reg         bmp_frame_commit;      // 与最后一个像素同拍
reg  [PCW-1:0] pixel_count;
//=== 新增结束 ===
// ANY_SIZE：真实宽高 + 行内列/填充/行计数（ANY_SIZE=0 时这些逻辑全部被裁掉）
reg  [15:0] hdr_w;                 // 头偏移 18..19
reg  [15:0] hdr_h;                 // 头偏移 22..23
reg  [13:0] row_bytes_q;           // 3w + w%4（每行 4 字节对齐）
reg  [25:0] pix_bytes_q;           // row_bytes × h（1 个 DSP）
reg  [11:0] any_col;
reg  [11:0] any_row;
reg  [1:0]  any_pad;               // 本行剩余填充字节
wire        any_size_ok   = (hdr_w != 16'd0) && (hdr_w <= 16'd4095) &&
                            (hdr_h != 16'd0) && (hdr_h <= 16'd4095);
//  ROW_SKIP：缩放器同一套行 DDA（宽受限 N = w、D = 640，否则 N = h、D = 480），头字节间隙里逐级算好
reg         rs_wlim, rs_d640, rs_vskip;   // 宽受限 / D = 640 / 缩小（N > D，才会有没用到的行）
reg  [11:0] rs_n;
reg  signed [13:0] rs_dmn;         // D - N
reg  signed [13:0] rs_yrem;        // 当前行（any_row）的 DDA 余量：> 0 = 缩放器会用到这一行
reg  [31:0] rs_row_off;            // 下一个要的字节偏移（当前用到的行首；行末跳到下一用到的行首）= 流读器 skip_until
// SMART：窗口 / 两遍
reg         sm_pass2;              // 第 2 遍（按构图窗口）
reg  [1:0]  sm_rcnt;               // LD_REST：复位流读器后再等 2 拍发 file_start
reg         ld_restart;            // 单拍：复位流读器（第 2 遍重读）
reg  [9:0]  rs_dv;                 // SMART 的 D
reg  [13:0] rs_c0b;                // 3·c0（窗口行内起始字节）
reg  [11:0] rs_wpre;               // 窗口前还要跳过的行数（每拍一行）
reg         busy_refine;
wire        sm_use  = (SMART != 0) && (sm_pass2 || busy_refine);   // 本遍按窗口读
wire [11:0] eff_w   = sm_use ? sm_cw : hdr_w[11:0];
wire [11:0] eff_h   = sm_use ? sm_ch : 12'd1;          // SMART 第 1 遍只读 1 行
wire        sm_win_ok = !sm_use || ((sm_cw != 12'd0) && (sm_ch != 12'd0) &&
                                    ({1'b0, sm_c0}  + {1'b0, sm_cw} <= {1'b0, hdr_w[11:0]}) &&
                                    ({1'b0, sm_r0b} + {1'b0, sm_ch} <= {1'b0, hdr_h[11:0]}));
wire signed [13:0] rs_d = (SMART != 0) ? $signed({4'd0, rs_dv}) : rs_d640 ? 14'sd640 : 14'sd480;
wire        rs_used = !rs_vskip || (rs_yrem > 14'sd0);
wire        any_rows_done = (any_row == ((SMART != 0) ? eff_h : hdr_h[11:0]));
wire        any_last_row  = (any_row == ((SMART != 0) ? eff_h : hdr_h[11:0]) - 1'b1);
wire        src_hold_eff  = (ANY_SIZE != 0) && src_hold;   // 老 tb 不接这个口（z）时不影响
assign src_width  = hdr_w[11:0];
assign src_height = hdr_h[11:0];

reg  [1:0]       write_buf_idx;      // 本次加载的目标 bank
wire             sd_usable          = sd_init_done && !sd_soft_rst;
wire             aud_flushed;        // 音频扇区结束是 SD 软复位补发的（仲裁器 b_flushed）
// 加载引擎空闲（相当于原 bmp_read 的 ready）
wire             ld_ready           = (ld_state == LD_IDLE) && !streamer_busy && !stream_rst;
wire             source_done        = source_started && ld_ready;
//loader 回执分类（只认本次尝试的回执；被中止的上一次尝试的回执只清 fb_outstanding）
wire             fb_ok_now          = fb_done && fb_live && (fb_error == ERR_NONE);
wire             fb_fail            = fb_done && fb_live && (fb_error != ERR_NONE);

assign sd_ready       = sd_usable;
assign pixel_valid    = bmp_data_wr_en;
assign pixel_rgb      = rgb_pixel;
assign frame_commit   = bmp_frame_commit;
assign fb_load_bank   = write_buf_idx;
assign fb_busy        = fb_outstanding;
assign fb_load_image  = busy_image_idx;
assign loader_ready   = sd_usable && scan_done && !load_busy && !load_start_pulse && !trk_cat_own;   // 曲目查询占用清单读口时暂缓（几拍）
assign bmp_state_code = {scanner_busy, streamer_busy, ld_state};

// =============================================================================
// 上层状态机：资源发现 + 单次加载 + 重试/看门狗/SD 自恢复
// =============================================================================
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        ms_div            <= {MSW{1'b0}};
        ms_tick           <= 1'b0;
        scan_start_pulse  <= 1'b0;
        scan_kicked       <= 1'b0;
        scan_done         <= 1'b0;
        scan_all_done     <= 1'b0;
        image_count       <= 7'd0;
        load_start_pulse  <= 1'b0;
        load_abort        <= 1'b0;
        load_cluster      <= 32'd0;
        load_file_size    <= 32'd0;
        cat_raddr         <= 9'd0;
        cat_fetch         <= 3'd0;
        scanner_spc       <= 8'd0;
        scanner_fat_start <= 32'd0;
        scanner_data_start<= 32'd0;
        scanner_cl_base   <= 32'd0;
        scanner_max_cluster <= 32'd0;
        audio_track_valid <= 16'd0;
        narr_aux_valid    <= 34'd0;
        use_streamer      <= 1'b0;
        fs_error          <= ERR_NONE;
        load_busy         <= 1'b0;
        start_pending     <= 1'b0;
        retry_count       <= 2'd0;
        busy_image_idx    <= 6'd0;
        busy_refine       <= 1'b0;
        fb_load_refine    <= 1'b0;
        source_started    <= 1'b0;
        fb_live           <= 1'b0;
        fb_outstanding    <= 1'b0;
        fb_load_start     <= 1'b0;
        fb_load_abort     <= 1'b0;
        write_done_seen   <= 1'b0;
        load_timeout_ms   <= {LTW{1'b0}};
        drain_active      <= 1'b0;
        drain_cnt         <= 6'd0;
        write_buf_idx     <= 2'd0;
        load_done         <= 1'b0;
        loaded_image_idx  <= 6'd0;
        loaded_buffer_idx <= 2'd0;
        load_error        <= ERR_NONE;
        sd_soft_rst       <= 1'b0;
        sd_rst_cnt        <= 8'd0;
        sd_reinit_count   <= 8'd0;
        reinit_wait       <= 1'b1;
        reinit_ms         <= {RTW{1'b0}};
    end else begin
        scan_start_pulse  <= 1'b0;
        fb_load_start     <= 1'b0;
        fb_load_abort     <= 1'b0;

        // loader 回执。任何回执都说明 loader 那边这次加载已结束
        if (fb_done)
            fb_outstanding <= 1'b0;
        load_start_pulse  <= 1'b0;
        load_abort        <= 1'b0;
        load_done         <= 1'b0;
        load_error        <= ERR_NONE;

        // ---------------- 1ms 节拍 ----------------
        ms_tick <= EXT_MS_TICK ? ext_ms_tick : (ms_div == MS_CYCLES - 1);
        ms_div  <= (EXT_MS_TICK || ms_div == MS_CYCLES - 1) ? {MSW{1'b0}} : ms_div + 1'b1;

        // ---------------- SD 软复位脉冲 ----------------
        if (sd_soft_rst) begin
            if (sd_rst_cnt >= SD_REINIT_CYCLES - 1)
                sd_soft_rst <= 1'b0;
            else
                sd_rst_cnt <= sd_rst_cnt + 8'd1;
        end

        // 初始化看门狗：拔卡期间周期性重新初始化，插回后最多 SD_REINIT_RETRY_MS 内恢复
        if (sd_soft_rst) begin
            reinit_wait <= 1'b1;
            reinit_ms   <= {RTW{1'b0}};
        end else if (reinit_wait) begin
            if (sd_init_done) begin
                reinit_wait <= 1'b0;
            end else if (reinit_ms > SD_REINIT_RETRY_MS) begin
                sd_soft_rst <= 1'b1;
                sd_rst_cnt  <= 8'd0;
                reinit_ms   <= {RTW{1'b0}};
            end else if (ms_tick) begin
                reinit_ms <= reinit_ms + 1'b1;
            end
        end

        // ---------------- 资源发现 ----------------
        // 扫描器与 SD 一起软复位：扫描没完成前被复位 → 初始化完成后重新扫
        if (sd_soft_rst && !scan_done)
            scan_kicked <= 1'b0;
        else if (!scan_kicked && !scan_done && sd_usable && ingest_enable && !scanner_busy) begin
            scan_start_pulse <= 1'b1;
            scan_kicked      <= 1'b1;
        end

        if (scanner_done && !scan_done) begin
            if (scanner_fs_error == FS_ERR_SECTOR) begin
                // 扇区读超时：重新初始化 SD 后重扫
                scan_kicked     <= 1'b0;
                sd_soft_rst     <= 1'b1;
                sd_rst_cnt      <= 8'd0;
                sd_reinit_count <= sd_reinit_count + 8'd1;
            end else begin
                // 扫描结束：锁存文件系统布局；清单已经由扫描器写在 BRAM 0..count-1
                scanner_spc         <= scanner_spc_w;
                scanner_fat_start   <= scanner_fat_start_w;
                scanner_data_start  <= scanner_data_start_w;
                scanner_cl_base     <= scanner_cl_base_w;
                scanner_max_cluster <= scanner_max_cluster_w;
                fs_error      <= scanner_fs_error;
                image_count   <= scanner_resource_count;
                audio_track_valid <= scanner_track_valid;   
                narr_aux_valid    <= scanner_aux_valid;     
                use_streamer  <= (scanner_resource_count != 7'd0);   // 找到文件才切给流读器
                scan_done     <= 1'b1;         // 没找到图也要结束等待（控制层/启动器报 0x53）
                scan_all_done <= 1'b1;
            end
        end

        // 音频扇区读超时（拔卡等）且没有图片加载在进行 → 同加载失败，软复位重新初始化 SD
        // （加载进行中由加载自己的失败路径处理；flush 补发的结束不算）
        if (((audio_sector_done && audio_sector_error != 8'h00 && !aud_flushed) ||
             (narr_sector_done && narr_sector_error != 8'h00 && !narr_flushed)) && !load_busy && scan_done && !sd_soft_rst) begin
            sd_soft_rst     <= 1'b1;
            sd_rst_cnt      <= 8'd0;
            sd_reinit_count <= sd_reinit_count + 8'd1;
        end

        // ---------------- 取清单项 {首簇, 大小}（BRAM 读 2 拍延迟）----------------
        if (cat_fetch != 3'd0) begin
            cat_fetch <= cat_fetch - 3'd1;
            if (cat_fetch == 3'd3) begin
                load_cluster <= {4'd0, cat_rdata[27:0]};
                cat_raddr    <= {cat_raddr[8:1], 1'b1};
            end
            if (cat_fetch == 3'd1)
                load_file_size <= cat_rdata;
        end

        // ---------------- 执行一次加载 ----------------
        if (!load_busy) begin
            load_timeout_ms <= {LTW{1'b0}};
            if (load_request && loader_ready) begin
                retry_count <= 2'd0;
                if ({1'b0, load_image_idx} >= image_count) begin
                    load_error <= ERR_BAD_INDEX;
                end else begin
                    // 修改点3：加载参数由扇区号换成 起始簇 + 文件大小（从清单 BRAM 取）
                    cat_raddr       <= {2'b00, load_image_idx, 1'b0};
                    cat_fetch       <= 3'd4;
                    write_buf_idx   <= load_buffer_idx;  // 在 fb_load_start 之前稳定
                    busy_image_idx  <= load_image_idx;
                    busy_refine     <= (SMART != 0) && (REFINE_EN != 0) && load_refine;
                    fb_load_refine  <= (SMART != 0) && (REFINE_EN != 0) && load_refine;
                    load_busy       <= 1'b1;
                    drain_active    <= 1'b0;
                    start_pending   <= 1'b1;
                    source_started  <= 1'b0;
                    write_done_seen <= 1'b0;
                end
            end
        end else begin
            // 看门狗覆盖整个加载过程：等 SD 重新初始化、实际读写
            // 第12 ANY_SIZE：有字节进展就清零（= 连续 LOAD_TIMEOUT_MS 没进展才超时，大图不误判）
            if ((ANY_SIZE != 0) && (streamer_byte_valid || src_progress)) load_timeout_ms <= {LTW{1'b0}};
            else if (ms_tick) load_timeout_ms <= load_timeout_ms + 1'b1;

            if (start_pending) begin
                // 上一次 fb_load_start 的回执（含被中止的）回来之后才发下一个
                if (ld_ready && sd_usable && use_streamer && cat_fetch == 3'd0 && !fb_outstanding &&
                    !((CANCEL_EN != 0) && load_cancel)) begin   // 取消那拍不启动（否则 fb_load_start 没人中止）
                    load_start_pulse <= 1'b1;
                    start_pending    <= 1'b0;
                    fb_load_start    <= 1'b1;      // 与加载引擎启动同拍，先于本帧所有像素进命令流
                    fb_live          <= 1'b1;
                    fb_outstanding   <= 1'b1;
                end
            end else begin
                if (!ld_ready && !load_start_pulse)
                    source_started <= 1'b1;
                // 只认本次 fb_load_start 之后的回执
                if (fb_ok_now) begin
                    write_done_seen <= 1'b1;
                    fb_live         <= 1'b0;
                end
            end

            if (ld_error || fb_fail || (load_timeout_ms > LOAD_TIMEOUT_MS)) begin
                // 失败：超时类先重新初始化 SD；剩余重试次数用完才上报
                // 看门狗超时 / loader 报错：把加载引擎和流读器拉回 IDLE（本地错误时引擎已自己回 IDLE）
                load_abort      <= !ld_error;
                // loader 还在等像素 → 发中止，它以 0x44 结束并回执
                fb_load_abort   <= fb_live && !fb_done;
                fb_live         <= 1'b0;
                source_started  <= 1'b0;
                write_done_seen <= 1'b0;
                load_timeout_ms <= {LTW{1'b0}};
                drain_active    <= 1'b0;
                if ((!ld_error && !fb_fail) || (ld_error && ld_error_reinit)) begin
                    sd_soft_rst     <= 1'b1;
                    sd_rst_cnt      <= 8'd0;
                    sd_reinit_count <= sd_reinit_count + 8'd1;
                end
                if (ld_error && (ld_error_code[7:4] == 4'h2))
                    fs_error <= ld_error_code;   // FAT 链错误同时记到 fs_error 观测口
                if (retry_count < MAX_RETRIES) begin
                    retry_count   <= retry_count + 2'd1;
                    start_pending <= 1'b1;
                end else begin
                    load_busy     <= 1'b0;
                    start_pending <= 1'b0;
                    load_error    <= ld_error ? ld_error_code : fb_fail ? fb_error : ERR_LOAD_TIMEOUT;
                end
            end else if (!start_pending && source_done && (write_done_seen || fb_ok_now)) begin
                // 写CDC排空 WRITE_DRAIN_CYCLES 拍后才报 load_done（播放器才会提交显示）
                if (!drain_active) begin
                    drain_active <= 1'b1;
                    drain_cnt    <= 6'd0;
                end else if (drain_cnt >= WRITE_DRAIN_CYCLES - 1) begin
                    drain_active      <= 1'b0;
                    load_busy         <= 1'b0;
                    load_done         <= 1'b1;
                    loaded_image_idx  <= busy_image_idx;
                    loaded_buffer_idx <= write_buf_idx;
                    retry_count       <= 2'd0;
                end else begin
                    drain_cnt <= drain_cnt + 6'd1;
                end
            end else if ((CANCEL_EN != 0) && load_cancel) begin
                // 上层取消（后台加载被更急的切图取代）。同 loader 报错的失败路径，但不重试、不复位 SD；
                // 已在排空的走上一分支照常完成。在途扇区的迟到 done 由复位后的流读器忽略，下一次 fb_load_start 等 fb_outstanding 清
                load_abort      <= 1'b1;
                fb_load_abort   <= fb_live && !fb_done;
                fb_live         <= 1'b0;
                source_started  <= 1'b0;
                write_done_seen <= 1'b0;
                drain_active    <= 1'b0;
                load_busy       <= 1'b0;
                start_pending   <= 1'b0;
                load_error      <= ERR_LOAD_CANCEL;
            end
        end
    end
end

// =============================================================================
// 加载引擎：启动流读器 → BMP 头流式校验（严谨版）→ BGR 字节拼像素
// 头校验：偏移 0..33 的字节到达时逐个和期望值比（签名/宽/高/平面/位深/压缩），只锁存
// 文件长度和像素偏移两个字段；第 34 字节到达时做范围判定。像素区从 data_offset(≥54)开始，
// 所以一定先判头、后出像素；头不合格立即中止流读器，一个像素都不写。
// =============================================================================
wire [31:0] pixel_bytes_need = (ANY_SIZE != 0) ? {6'd0, pix_bytes_q} : EXPECTED_PIXEL_BYTES;
wire [31:0] pixel_data_end = data_offset + pixel_bytes_need;
// data_offset < 文件长度 由 “文件长度 >= 像素区末尾 且 末尾不溢出” 蕴含，不再单独比较
wire header_match = hdr_fixed_ok && hdr_dib_ok &&
                    ((ANY_SIZE == 0) || any_size_ok) &&
                    (data_offset >= 32'd54) &&
                    (pixel_data_end >= data_offset) &&
                    (hdr_file_len >= pixel_data_end) &&
                    (load_file_size >= pixel_data_end);   // 像素区必须在 FAT 文件大小之内
wire [5:0]  hoff     = streamer_offset[5:0];
wire        in_hdr   = (streamer_offset[31:6] == 26'd0) && (hoff < 6'd34);
wire        at_pix   = pix_on || (streamer_offset == data_offset);
wire        sm_preseek = (SMART != 0) && (rs_wpre != 12'd0);   // 窗口前的行：每拍跳一行
wire        pixel_byte = (byte_state == 2'd2) &&
                         ((ROW_SKIP != 0) ? (!any_rows_done && rs_used && !sm_preseek && (streamer_offset >= rs_row_off)) :
                          at_pix && ((ANY_SIZE != 0) ? !any_rows_done : (pixel_count < EXPECTED_PIXEL_COUNT)));
// ROW_SKIP：当前行缩放器用不到 → 每拍跳一行（不等字节；跳过的行的字节偏移都 < 新 row_off，到了也会丢）
wire        rs_seek    = (ROW_SKIP != 0) && (byte_state == 2'd2) && !any_rows_done && !rs_used && !sm_preseek;
// SMART：最后一个窗口行结束（行末或跳行）→ skip_until 直接到像素区末尾（后面的扇区都不读）
wire [31:0] rs_next_off = ((SMART != 0) && any_last_row) ? pixel_data_end : rs_row_off + row_bytes_q;

// 偏移 0..33 中固定内容的字节：{需要比较, 期望值}
//  ANY_SIZE：宽高低 16 位不比对（锁存到 hdr_w/hdr_h，范围在 header_match 里查），高 16 位仍须为 0
reg        exp_care;
reg [7:0]  exp_byte;
always @* begin
    exp_care = 1'b1;
    case (hoff)
        6'd0:  exp_byte = "B";
        6'd1:  exp_byte = "M";
        6'd18: begin exp_byte = BMP_WIDTH[7:0];   exp_care = (ANY_SIZE == 0); end
        6'd19: begin exp_byte = BMP_WIDTH[15:8];  exp_care = (ANY_SIZE == 0); end
        6'd22: begin exp_byte = BMP_HEIGHT[7:0];  exp_care = (ANY_SIZE == 0); end
        6'd23: begin exp_byte = BMP_HEIGHT[15:8]; exp_care = (ANY_SIZE == 0); end
        6'd26: exp_byte = 8'd1;                       // 平面数 = 1
        6'd28: exp_byte = 8'd24;                      // 位深 = 24
        6'd20, 6'd21, 6'd24, 6'd25, 6'd27, 6'd29,
        6'd30, 6'd31, 6'd32, 6'd33: exp_byte = 8'd0;  // 宽/高高 16 位、平面/位深高字节、压缩 = 0
        default: begin exp_care = 1'b0; exp_byte = 8'd0; end
    endcase
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        row_bytes_q <= 14'd0;
        pix_bytes_q <= 26'd0;
        rs_wlim     <= 1'b1;
        rs_d640     <= 1'b1;
        rs_vskip    <= 1'b0;
        rs_n        <= 12'd1;
        rs_dmn      <= 14'sd0;
        rs_dv       <= 10'd640;
        rs_c0b      <= 14'd0;
    end else begin
        row_bytes_q <= {hdr_w[11:0], 1'b0} + hdr_w[11:0] + hdr_w[1:0];
        pix_bytes_q <= row_bytes_q * hdr_h[11:0];
        // ROW_SKIP：hdr_w 在偏移 19、hdr_h 在 23 锁存，第 34 字节判头前三级早已稳定
        rs_wlim     <= ({hdr_w[11:0], 1'b0} + hdr_w[11:0]) >= {hdr_h[11:0], 2'b00};
        rs_d640     <= rs_wlim;
        if (SMART != 0) begin
            // 第 1 遍：N = D = 1（只读第 0 行）；按窗口读：缩放器给的 N / D
            rs_n     <= sm_use ? sm_n : 12'd1;
            rs_dv    <= sm_use ? sm_d : 10'd1;
            rs_c0b   <= sm_use ? ({sm_c0, 1'b0} + sm_c0) : 14'd0;
            rs_vskip <= {1'b0, rs_n} > {3'd0, rs_dv};
        end else begin
            rs_n     <= rs_wlim ? hdr_w[11:0] : hdr_h[11:0];
            rs_vskip <= rs_n > (rs_d640 ? 12'd640 : 12'd480);
        end
        rs_dmn      <= rs_d - $signed({2'd0, rs_n});
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        hdr_w            <= 16'd0;
        hdr_h            <= 16'd0;
        any_col          <= 12'd0;
        any_row          <= 12'd0;
        any_pad          <= 2'd0;
        src_geom_valid   <= 1'b0;
        ld_state         <= LD_IDLE;
        ld_error         <= 1'b0;
        ld_error_code    <= ERR_NONE;
        ld_error_reinit  <= 1'b0;
        ld_abort         <= 1'b0;
        stream_rst       <= 1'b0;
        file_start_pulse <= 1'b0;
        byte_state       <= 2'd0;
        hdr_fixed_ok     <= 1'b0;
        hdr_dib_ok       <= 1'b0;
        hdr_file_len     <= 32'd0;
        data_offset      <= 32'd54;
        pix_on           <= 1'b0;
        rgb_byte_cnt     <= 2'd0;
        rgb_pixel        <= 24'd0;
        bmp_data_wr_en   <= 1'b0;
        bmp_frame_commit <= 1'b0;
        pixel_count      <= {PCW{1'b0}};
        rs_yrem          <= 14'sd0;
        rs_row_off       <= 32'd0;
        sm_pass2         <= 1'b0;
        sm_rcnt          <= 2'd0;
        ld_restart       <= 1'b0;
        rs_wpre          <= 12'd0;
    end else begin
        ld_restart       <= 1'b0;
        ld_error         <= 1'b0;
        ld_abort         <= 1'b0;
        file_start_pulse <= 1'b0;
        bmp_data_wr_en   <= 1'b0;
        bmp_frame_commit <= 1'b0;
        src_geom_valid   <= 1'b0;
        stream_rst       <= sd_soft_rst || load_abort || ld_abort || ld_restart;

        if (sd_soft_rst || load_abort) begin
            // SD 重新初始化 / 上层看门狗中止：丢弃本次加载回 IDLE（不报错，由上层重试）
            ld_state   <= LD_IDLE;
            byte_state <= 2'd0;
            rs_wpre    <= 12'd0;
            sm_rcnt    <= 2'd0;
        end else begin
            case (ld_state)
                LD_IDLE: begin
                    if (load_start_pulse) begin
                        sm_pass2     <= 1'b0;
                        // 同拍 fb_load_start 已进命令流（loader 开始接收本 bank），直接读文件；
                        // 头不合格时一个像素都不写，上层发中止，这个 bank 不会被标记为已加载
                        file_start_pulse <= 1'b1;     // 流读器：first_cluster + file_size
                        byte_state   <= 2'd1;           // 开始读 BMP 头
                        hdr_fixed_ok <= 1'b1;
                        hdr_dib_ok   <= 1'b0;
                        data_offset  <= 32'd54;
                        pix_on       <= 1'b0;
                        rgb_byte_cnt <= 2'd0;
                        pixel_count  <= {PCW{1'b0}};
                        any_col      <= 12'd0;
                        any_row      <= 12'd0;
                        any_pad      <= 2'd0;
                        rs_row_off   <= 32'd0;        // 判头之前不跳扇区
                        rs_wpre      <= 12'd0;
                        ld_state     <= LD_STREAM;
                    end
                end

                // SMART：第 1 遍（探测）读完 → 等缩放器定出构图窗口 → 复位流读器 → 从头读第 2 遍
                LD_REST: begin
                    if (sm_rcnt == 2'd0) begin
                        if (sm_map_valid) begin
                            ld_restart <= 1'b1;
                            sm_rcnt    <= 2'd1;
                        end
                    end else if (sm_rcnt != 2'd3) begin
                        sm_rcnt <= sm_rcnt + 1'b1;
                    end else begin
                        sm_rcnt          <= 2'd0;
                        sm_pass2         <= 1'b1;
                        file_start_pulse <= 1'b1;
                        byte_state   <= 2'd1;
                        hdr_fixed_ok <= 1'b1;
                        hdr_dib_ok   <= 1'b0;
                        data_offset  <= 32'd54;
                        pix_on       <= 1'b0;
                        rgb_byte_cnt <= 2'd0;
                        pixel_count  <= {PCW{1'b0}};
                        any_col      <= 12'd0;
                        any_row      <= 12'd0;
                        any_pad      <= 2'd0;
                        rs_row_off   <= 32'd0;
                        rs_wpre      <= 12'd0;
                        ld_state     <= LD_STREAM;
                    end
                end

                LD_STREAM: begin
                    if (streamer_byte_valid) begin
                        // ---- BMP 头：固定字段逐字节比对，两个范围字段小端拼接 ----
                        if (in_hdr) begin
                            if (exp_care && streamer_byte != exp_byte)
                                hdr_fixed_ok <= 1'b0;
                            if (hoff >= 6'd2 && hoff <= 6'd5)
                                hdr_file_len <= {streamer_byte, hdr_file_len[31:8]};
                            if (hoff >= 6'd10 && hoff <= 6'd13)
                                data_offset <= {streamer_byte, data_offset[31:8]};
                            if (hoff == 6'd14)
                                hdr_dib_ok <= (streamer_byte >= 8'd40);
                            else if (hoff >= 6'd15 && hoff <= 6'd17 && streamer_byte != 8'd0)
                                hdr_dib_ok <= 1'b1;     // DIB 头长度 >= 256 也算 >= 40
                            if (hoff == 6'd18 || hoff == 6'd19)
                                hdr_w <= {streamer_byte, hdr_w[15:8]};
                            if (hoff == 6'd22 || hoff == 6'd23)
                                hdr_h <= {streamer_byte, hdr_h[15:8]};
                        end
                        if (streamer_offset == 32'd34 && byte_state == 2'd1) begin
                            // 偏移 0..33 已全部到齐：严格头校验
                            if (header_match && ((SMART == 0) || sm_win_ok)) begin
                                byte_state     <= 2'd2;
                                src_geom_valid <= !((SMART != 0) && sm_pass2);   // 尺寸给缩放器（先于第一个像素）；SMART 第 2 遍不再发
                                rs_yrem        <= rs_d;          // 第 0 行一定用到
                                rs_row_off     <= data_offset + rs_c0b;   // SMART：窗口行内从第 c0 列起（否则 rs_c0b = 0）
                                if (SMART != 0) rs_wpre <= sm_use ? sm_r0b : 12'd0;
                            end else begin
                                ld_error        <= 1'b1;
                                ld_error_code   <= ERR_BMP_HEADER;
                                ld_error_reinit <= 1'b0;
                                ld_abort        <= 1'b1;
                                byte_state      <= 2'd0;
                                ld_state        <= LD_IDLE;
                            end
                        end

                        // ---- 拼 RGB：BMP 按 B,G,R 存，第 3 个字节到达时输出一个像素 ----
                        if (pixel_byte && (ANY_SIZE != 0)) begin
                            //  ANY_SIZE：B,G,R 拼像素；每行 w 个像素后跳过 w%4 个填充字节
                            pix_on <= 1'b1;
                            if (any_pad != 2'd0) begin
                                any_pad <= any_pad - 2'd1;
                            end else begin
                                case (rgb_byte_cnt)
                                    2'd0: begin rgb_pixel[7:0]   <= streamer_byte; rgb_byte_cnt <= 2'd1; end
                                    2'd1: begin rgb_pixel[15:8]  <= streamer_byte; rgb_byte_cnt <= 2'd2; end
                                    default: begin
                                        rgb_pixel[23:16] <= streamer_byte;
                                        rgb_byte_cnt     <= 2'd0;
                                        bmp_data_wr_en   <= (SMART == 0) || sm_use;   // SMART 第 1 遍不送像素
                                        bmp_frame_commit <= ((SMART == 0) || sm_use) && any_last_row &&
                                                            (any_col == ((SMART != 0) ? eff_w : hdr_w[11:0]) - 1'b1);
                                        if (any_col == ((SMART != 0) ? eff_w : hdr_w[11:0]) - 1'b1) begin
                                            any_col <= 12'd0;
                                            any_row <= any_row + 1'b1;
                                            any_pad <= (ROW_SKIP != 0) ? 2'd0 : hdr_w[1:0];   // ROW_SKIP：填充按偏移丢
                                            if (ROW_SKIP != 0) begin
                                                rs_row_off <= rs_next_off;
                                                rs_yrem    <= rs_yrem + rs_dmn;   // 缩小时每个用到的行恰好出一个目标行（不缩小时不看 yrem）
                                            end
                                        end else begin
                                            any_col <= any_col + 1'b1;
                                        end
                                    end
                                endcase
                            end
                        end else if (pixel_byte) begin
                            pix_on <= 1'b1;
                            case (rgb_byte_cnt)
                                2'd0: begin rgb_pixel[7:0]   <= streamer_byte; rgb_byte_cnt <= 2'd1; end
                                2'd1: begin rgb_pixel[15:8]  <= streamer_byte; rgb_byte_cnt <= 2'd2; end
                                default: begin
                                    rgb_pixel[23:16] <= streamer_byte;
                                    rgb_byte_cnt     <= 2'd0;
                                    bmp_data_wr_en   <= 1'b1;
                                    // 最后一个像素同拍提交（不用"文件读完"当提交）
                                    bmp_frame_commit <= (pixel_count == EXPECTED_PIXEL_COUNT - 1);
                                    pixel_count      <= pixel_count + 1'b1;
                                end
                            endcase
                        end
                    end

                    if (rs_seek) begin
                        any_row    <= any_row + 1'b1;
                        rs_row_off <= rs_next_off;
                        rs_yrem    <= rs_yrem + rs_d;
                    end
                    if (sm_preseek && (byte_state == 2'd2)) begin
                        rs_wpre    <= rs_wpre - 1'b1;
                        rs_row_off <= rs_row_off + row_bytes_q;
                    end

                    if (streamer_done) begin
                        // SMART 普通加载的第 1 遍正常读完 → LD_REST（错误判定同下，出错仍回 IDLE）
                        ld_state   <= ((SMART != 0) && !sm_use && (streamer_fs_error == ERR_NONE) &&
                                       (byte_state == 2'd2) && any_rows_done && (rgb_byte_cnt == 2'd0)) ? LD_REST : LD_IDLE;
                        byte_state <= 2'd0;
                        if (streamer_fs_error != ERR_NONE) begin
                            // 0x10 = 扇区读超时（需要重新初始化 SD）；0x23~0x27 FAT 链错误原样上报
                            ld_error        <= 1'b1;
                            ld_error_code   <= (streamer_fs_error == FS_ERR_SECTOR) ? ERR_LOAD_TIMEOUT
                                                                                    : streamer_fs_error;
                            ld_error_reinit <= (streamer_fs_error == FS_ERR_SECTOR);
                        end else if (byte_state != 2'd2) begin
                            // 文件不足 35 字节，头都没读完
                            ld_error        <= 1'b1;
                            ld_error_code   <= ERR_BMP_HEADER;
                            ld_error_reinit <= 1'b0;
                        end else if (((ANY_SIZE != 0) ? !any_rows_done : (pixel_count != EXPECTED_PIXEL_COUNT)) ||
                                     (rgb_byte_cnt != 2'd0)) begin
                            ld_error        <= 1'b1;
                            ld_error_code   <= ERR_BMP_TRUNCATED;
                            ld_error_reinit <= 1'b0;
                        end
                    end
                end

                default: begin
                    ld_state   <= LD_IDLE;
                    byte_state <= 2'd0;
                end
            endcase
        end
    end
end

// =============================================================================
// 扇区读仲裁：扫描和加载不会同时进行（扫描成功后才会有加载），按 use_streamer 分时复用。
// 未选中的一方看到 busy=1、收不到 valid/done，不可能误发请求或误收数据。
// =============================================================================
wire        scanner_sector_req;
wire [31:0] scanner_sector_lba;
wire        streamer_sector_req;
wire [31:0] streamer_sector_lba;
wire        arb_sector_busy;
wire        arb_sector_valid;
wire [7:0]  arb_sector_byte;
wire [8:0]  arb_sector_byte_index;
wire        arb_sector_done;
wire [7:0]  arb_sector_error;

wire        arb_sector_req = use_streamer ? streamer_sector_req : scanner_sector_req;
wire [31:0] arb_sector_lba = use_streamer ? streamer_sector_lba : scanner_sector_lba;

// 扫描器 / 适配层随 SD 软复位一起复位（sd_soft_rst 是单个寄存器输出，无毛刺）
wire fs_rst_n     = rst_n & ~sd_soft_rst;
wire stream_rst_n = rst_n & ~stream_rst;

// ---------- 文件清单 BRAM（512x32 = 2 块 BRAM9K：低 128 项图片 + 子目录队列，高 128 项曲目类文件）----------
fat32_sdp_ram #(.AW(9), .DW(32)) u_catalog (
    .clk   (clk),
    .we    (scanner_busy & scn_cat_we),
    .waddr (scn_cat_waddr),
    .wdata (scn_cat_wdata),
    .re    (1'b1),
    .raddr (scanner_busy ? scn_cat_raddr : trk_cat_own ? trk_cat_raddr : cat_raddr),   // 曲目查询
    .rdata (cat_rdata)
);

// ---------- 卷扫描器 ----------
fat32_volume_scanner u_scanner (
    .clk                 (clk),
    .rst_n               (fs_rst_n),
    .scan_start          (scan_start_pulse),
    .scan_busy           (scanner_busy),
    .scan_done           (scanner_done),
    .fs_error            (scanner_fs_error),
    .partition_lba       (),
    .sectors_per_cluster (scanner_spc_w),
    .fat_start_lba       (scanner_fat_start_w),
    .data_start_lba      (scanner_data_start_w),
    .cluster_base_lba    (scanner_cl_base_w),
    .max_cluster         (scanner_max_cluster_w),
    .resource_count      (scanner_resource_count),
    .resource_table_full (),
    .directory_limit_hit (),
    .bad_cluster_skip_count(),
    .track_valid         (scanner_track_valid),
    .aux_valid           (scanner_aux_valid),
    .cat_we              (scn_cat_we),
    .cat_waddr           (scn_cat_waddr),
    .cat_wdata           (scn_cat_wdata),
    .cat_raddr           (scn_cat_raddr),
    .cat_rdata           (cat_rdata),
    .sector_req          (scanner_sector_req),
    .sector_lba          (scanner_sector_lba),
    .sector_busy         (use_streamer ? 1'b1 : arb_sector_busy),
    .sector_valid        (!use_streamer && arb_sector_valid),
    .sector_byte         (arb_sector_byte),
    .sector_byte_index   (arb_sector_byte_index),
    .sector_done         (!use_streamer && arb_sector_done),
    .sector_error        (arb_sector_error)
);

// ---------- 文件流读取器 ----------
fat32_file_streamer #(
    .MAX_VISITED_CLUSTERS (MAX_VISITED_CLUSTERS),
    .SKIP_EN              (ROW_SKIP)            // 整扇区都在下一个要用的行之前就不读
) u_streamer (
    .clk                 (clk),
    .rst_n               (stream_rst_n),
    .file_start          (file_start_pulse),
    .first_cluster       (load_cluster),
    .file_size           (load_file_size),
    .sectors_per_cluster (scanner_spc),
    .fat_start_lba       (scanner_fat_start),
    .cluster_base_lba    (scanner_cl_base),
    .max_cluster         (scanner_max_cluster),
    .file_busy           (streamer_busy),
    .file_done           (streamer_done),
    .file_valid          (streamer_byte_valid),
    .file_byte           (streamer_byte),
    .file_byte_offset    (streamer_offset),
    .fs_error            (streamer_fs_error),
    .sector_req          (streamer_sector_req),
    .sector_lba          (streamer_sector_lba),
    .sector_busy         (use_streamer ? (arb_sector_busy | src_hold_eff) : 1'b1),   // 缩放器反压
    .sector_valid        (use_streamer && arb_sector_valid),
    .sector_byte         (arb_sector_byte),
    .sector_byte_index   (arb_sector_byte_index),
    .sector_done         (use_streamer && arb_sector_done),
    .sector_error        (arb_sector_error),
    .skip_until          (rs_row_off)
);

// ---------- 扇区读口仲裁（A = 扫描器/流读器，B = WAV 音频）----------
// 原来 arb_sector_* 直接接适配层；现在经 sd_audio_sector_arbiter 按扇区轮转。扫描阶段（use_streamer=0）不放行音频；
// SD 软复位时音频在途扇区补 done + 0x10（播放器结束本曲，不会卡住），未放行的请求等 SD 恢复后继续；
// 音频扇区读超时（无图片加载时）同样触发 SD 重新初始化，拔卡插回后音频自行恢复
wire        adp_sector_req;
wire [31:0] adp_sector_lba;
wire        adp_sector_busy;
wire        adp_sector_valid;
wire        adp_sector_done;

sd_audio_sector_arbiter u_sector_arb (
    .clk       (clk),
    .rst_n     (rst_n),
    .flush     (sd_soft_rst),
    .b_enable  (use_streamer && sd_usable),   // 扫描阶段 / SD 重新初始化期间音频请求只挂起
    .a_req     (arb_sector_req),
    .a_lba     (arb_sector_lba),
    .a_busy    (arb_sector_busy),
    .a_valid   (arb_sector_valid),
    .a_done    (arb_sector_done),
    .b_req     (audio_sector_req),
    .b_lba     (audio_sector_lba),
    .b_busy    (audio_sector_busy),
    .b_valid   (audio_sector_valid),
    .b_done    (audio_sector_done),
    .b_flushed (aud_flushed),
    .c_req     (narr_sector_req),
    .c_lba     (narr_sector_lba),
    .c_busy    (narr_sector_busy),
    .c_valid   (narr_sector_valid),
    .c_done    (narr_sector_done),
    .c_flushed (narr_flushed),
    .s_req     (adp_sector_req),
    .s_lba     (adp_sector_lba),
    .s_busy    (adp_sector_busy),
    .s_valid   (adp_sector_valid),
    .s_done    (adp_sector_done)
);
assign audio_sector_byte       = arb_sector_byte;
assign audio_sector_byte_index = arb_sector_byte_index;
assign audio_sector_error      = aud_flushed ? FS_ERR_SECTOR : arb_sector_error;
assign narr_sector_error       = narr_flushed ? FS_ERR_SECTOR : arb_sector_error;   // 字节/序号与音频口共用

// ---------- 曲目号 → 扇区范围（清单 BRAM 第 128.. 项，与加载取项分时共用读口）----------
wire trk_cat_free = scan_done && !scanner_busy && (cat_fetch == 3'd0) &&
                    !(!load_busy && load_request && loader_ready);
pcm_track_locator u_track_locator (
    .clk                 (clk),
    .rst_n               (rst_n),
    .req                 (audio_info_req),
    .idx                 ({2'b00, audio_info_idx}),        // 类型 0 = PCM
    .req_b               (narr_info_req),
    .idx_b               (narr_info_idx),
    .info_valid_b        (narr_info_valid),
    .size_bytes          (narr_info_size),
    .cat_free            (trk_cat_free),
    .cat_own             (trk_cat_own),
    .cat_raddr           (trk_cat_raddr),
    .cat_rdata           (cat_rdata),
    .info_valid          (audio_info_valid),
    .start_lba           (audio_info_start_lba),
    .total_sectors       (audio_info_total_sectors)
);

// ---------- SD 卡接口适配层（对接官方 sd_card_top）----------
sd_sector_adapter #(
    .TIMEOUT_CYCLES (SD_READ_TIMEOUT_MS),           // L2：按 1ms 节拍计（原 MS_CYCLES*500 拍，25 位）
    .TICK_EN        (1)
) u_adapter (
    .clk                    (clk),
    .rst_n                  (fs_rst_n),
    .tick                   (ms_tick),
    .sector_req             (adp_sector_req),       // 经仲裁器
    .sector_lba             (adp_sector_lba),
    .sector_busy            (adp_sector_busy),
    .sector_valid           (adp_sector_valid),
    .sector_byte            (arb_sector_byte),
    .sector_byte_index      (arb_sector_byte_index),
    .sector_done            (adp_sector_done),
    .sector_error           (arb_sector_error),
    .sd_sec_read            (sd_sec_read),
    .sd_sec_read_addr       (sd_sec_read_addr),
    .sd_sec_read_data_valid (sd_sec_read_data_valid),
    .sd_sec_read_data       (sd_sec_read_data),
    .sd_sec_read_end        (sd_sec_read_end)
);

// 官方驱动原样使用；本工程只读卡，写口接 0（综合时写路径被裁掉）
sd_card_top sd_card_top_m0(
    .clk                    (clk),
    .rst_n                  (rst_n & ~sd_soft_rst),  // 软复位 = 单个寄存器输出，无毛刺
    .SD_nCS                 (SD_nCS),
    .SD_DCLK                (SD_DCLK),
    .SD_MOSI                (SD_MOSI),
    .SD_MISO                (SD_MISO),
    .sd_init_done           (sd_init_done),
    .sd_sec_read            (sd_sec_read),
    .sd_sec_read_addr       (sd_sec_read_addr),
    .sd_sec_read_data       (sd_sec_read_data),
    .sd_sec_read_data_valid (sd_sec_read_data_valid),
    .sd_sec_read_end        (sd_sec_read_end),
    .sd_sec_write           (1'b0),
    .sd_sec_write_addr      (32'd0),
    .sd_sec_write_data      (8'd0),
    .sd_sec_write_data_req  (),
    .sd_sec_write_end       ()
);

endmodule
