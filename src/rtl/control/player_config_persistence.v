`timescale 1ns/1ps

module player_config_persistence #(
    parameter [23:0]  SLOT0_ADDRESS        = 24'h7FE000,
    parameter [23:0]  SLOT1_ADDRESS        = 24'h7FF000,
    parameter [7:0]   EXPECT_CAPACITY      = 8'h17,      // W25Q64：JEDEC EF 40 17
    parameter integer ERASE_TIMEOUT_MS     = 5000,       // tick 数：5 s（W25Q64 扇区擦除最长 400ms）
    parameter integer PROGRAM_TIMEOUT_MS   = 100,        // tick 数：100 ms（页编程最长 3ms）
    parameter integer STARTUP_MS           = 1,          // tick 数：上电后等 1ms 再访问 Flash
    parameter integer WAKE_MS              = 1           // tick 数：ABh 唤醒后等待（tRES1 3us）
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        tick,               // 公共 1ms 节拍（tb 接 1 = 按时钟周期计）

    // ---- 保存请求（media_policy.save_done_event + 快照）----
    input  wire        save_request,
    input  wire [2:0]  save_period_sel,
    input  wire [8:0]  save_brightness,    // 外接传感器：亮度偏移（有符号 9 位，-256..255）
    input  wire [7:0]  save_contrast,
    input  wire [7:0]  save_volume,
    input  wire        save_auto_mode,
    input  wire        save_ui_hide,       // KEY3 长按隐藏底部跑马灯 / 频谱柱

    // ---- 上电加载结果（cfg_load_valid 单拍时有效；其余时间 cfg_* 兼作待写记录）----
    output reg         cfg_load_valid,
    output reg  [2:0]  cfg_period_sel,
    output reg  [8:0]  cfg_brightness,     // 外接传感器：亮度偏移（有符号）
    output reg  [7:0]  cfg_contrast,
    output reg  [7:0]  cfg_volume,
    output reg         cfg_auto_mode,
    output reg         cfg_ui_hide,

    // ---- 状态 ----
    output reg         save_ok,          // 单拍：写入并读回校验成功
    output reg         error_event,      // 单拍
    output reg  [7:0]  error_code,
    output reg         flash_ok,         // JEDEC ID 正确
    output wire        busy,
    output reg         active_valid,     // Flash 中至少有一个有效记录
    output reg         active_slot,
    output reg  [7:0]  active_seq,
    output reg  [4:0]  state_code,       // 观测

    // ---- W25Q64 SPI ----
    output reg         flash_cs_n,
    output reg         flash_sclk,
    output reg         flash_mosi,
    input  wire        flash_miso,
    output wire        flash_wp_n,
    output wire        flash_hold_n
);
    localparam [7:0] ERR_FLASH_ABSENT = 8'h5B;
    localparam [7:0] ERR_SAVE_FAILED  = 8'h5C;

    localparam [7:0] CMD_WREN  = 8'h06;
    localparam [7:0] CMD_RDSR1 = 8'h05;
    localparam [7:0] CMD_READ  = 8'h03;
    localparam [7:0] CMD_PP    = 8'h02;
    localparam [7:0] CMD_SE4K  = 8'h20;
    localparam [7:0] CMD_JEDEC = 8'h9F;
    localparam [7:0] CMD_WAKE  = 8'hAB;

    localparam [7:0] REC_VERSION = 8'h03;      // 外接传感器：v3 = 亮度偏移 9 位（byte7 低 8 位 + byte11 bit0 符号）
    localparam [7:0] REC_V2      = 8'h02;
    localparam [7:0] REC_V1      = 8'h01;
    localparam integer GAP_CYCLES = 8;       // 片选高 160ns（tSHSL 读 10ns / 写 50ns）

    // ---- 主状态机（编码沿用原版；5 / 10 两个 CRC 状态已不需要，7 LOAD_APPLY / 9 BUILD 已并入前一状态）----
    localparam [4:0] ST_STARTUP     = 5'd0,
                     ST_WAKE        = 5'd1,
                     ST_WAKE_WAIT   = 5'd2,
                     ST_PREFLIGHT   = 5'd3,
                     ST_LOAD        = 5'd4,
                     ST_LOAD_CHK    = 5'd6,
                     ST_READY       = 5'd8,
                     ST_WREN        = 5'd11,
                     ST_WEL         = 5'd12,
                     ST_ERASE       = 5'd13,
                     ST_ERASE_POLL  = 5'd14,
                     ST_PROGRAM     = 5'd15,
                     ST_PROG_POLL   = 5'd16,
                     ST_VERIFY      = 5'd17,
                     ST_VERIFY_CHK  = 5'd18,
                     ST_FAIL        = 5'd19;

    assign flash_wp_n   = 1'b1;
    assign flash_hold_n = 1'b1;

    wire [7:0]  seq_next = active_seq + 8'd1;   // 本次写入的 seq（无有效记录时 active_seq = 0 → 1）
    reg  [7:0]  crc_r;                 // CRC 累加（每次事务从 0 开始，逐位更新）

    // 记录第 i 字节（取自 cfg_*：保存开工时已锁存）；第 15 字节 = 当前 CRC 累加值
    function [7:0] rec_byte;
        input [3:0] i;
        input [7:0] crc;
        begin
            case (i)
                4'd0:    rec_byte = 8'h50;                  // 'P'
                4'd1:    rec_byte = 8'h43;                  // 'C'
                4'd2:    rec_byte = 8'h46;                  // 'F'
                4'd3:    rec_byte = 8'h47;                  // 'G'
                4'd4:    rec_byte = REC_VERSION;
                4'd5:    rec_byte = seq_next;
                4'd6:    rec_byte = {5'd0, cfg_period_sel};
                4'd7:    rec_byte = cfg_brightness[7:0];
                4'd8:    rec_byte = cfg_volume;
                4'd9:    rec_byte = {7'd0, cfg_auto_mode};
                4'd10:   rec_byte = cfg_contrast;
                4'd11:   rec_byte = {6'd0, cfg_ui_hide, cfg_brightness[8]};
                4'd15:   rec_byte = crc;
                default: rec_byte = 8'h00;
            endcase
        end
    endfunction

    // =========================== SPI 事务引擎 ===========================
    // 一次事务：命令字节 + (可选)3 字节槽地址 + x_ndata 个数据字节
    //   写：数据字节 = rec_byte(序号)；读：每收完一个数据字节出一拍 rx_stb（rx_idx / rx_byte）给主状态机
    localparam [2:0] XS_IDLE  = 3'd0,
                     XS_SETUP = 3'd1,
                     XS_SHIFT = 3'd2,
                     XS_HOLD  = 3'd3,
                     XS_GAP   = 3'd4;

    reg         x_go;
    reg  [7:0]  x_cmd;
    reg         x_slot;        // 地址 = 槽 0 / 槽 1
    reg         x_has_addr;
    reg  [4:0]  x_ndata;
    reg         x_done;
    reg  [2:0]  xs;
    reg  [4:0]  x_bi;          // 当前字节序号
    reg  [2:0]  x_bit;
    reg  [1:0]  x_ph;
    reg  [6:0]  x_rx;
    reg  [3:0]  x_cnt;
    reg         miso_r;
    reg         x_match;       // 本次事务各数据位都与 rec_byte 相等（读回校验）
    reg         rx_stb;
    reg  [3:0]  rx_idx;
    reg  [7:0]  rx_byte;

    wire [23:0] x_addr  = x_slot ? SLOT1_ADDRESS : SLOT0_ADDRESS;
    wire [4:0] x_hdr   = x_has_addr ? 5'd4 : 5'd1;
    wire [4:0] x_total = x_hdr + x_ndata;
    wire [4:0] x_didx  = x_bi - x_hdr;
    wire       x_data  = (x_bi >= x_hdr);
    wire [7:0] x_dbyte = rec_byte(x_didx[3:0], crc_r);
    // 当前字节（读命令时数据字节照发 rec_byte，Flash 忽略 MOSI；读回校验靠它逐位比较）
    wire [7:0] x_cur   = (x_bi == 5'd0)                  ? x_cmd :
                         (x_has_addr && x_bi == 5'd1)    ? x_addr[23:16] :
                         (x_has_addr && x_bi == 5'd2)    ? x_addr[15:8] :
                         (x_has_addr && x_bi == 5'd3)    ? x_addr[7:0] : x_dbyte;
    wire [7:0] x_rx_byte = {x_rx, miso_r};
    wire       x_write   = (x_cmd == CMD_PP);
    wire       crc_in    = x_write ? flash_mosi : miso_r;     // 本位：写 = 刚发出的，读 = 收到的

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            xs         <= XS_IDLE;
            x_done     <= 1'b0;
            x_bi       <= 5'd0;
            x_bit      <= 3'd7;
            x_ph       <= 2'd0;
            x_rx       <= 7'd0;
            x_cnt      <= 4'd0;
            miso_r     <= 1'b0;
            x_match    <= 1'b0;
            crc_r      <= 8'd0;
            rx_stb     <= 1'b0;
            rx_idx     <= 4'd0;
            rx_byte    <= 8'd0;
            flash_cs_n <= 1'b1;
            flash_sclk <= 1'b0;
            flash_mosi <= 1'b0;
        end else begin
            miso_r <= flash_miso;
            x_done <= 1'b0;
            rx_stb <= 1'b0;
            case (xs)
                XS_IDLE: begin
                    flash_cs_n <= 1'b1;
                    flash_sclk <= 1'b0;
                    if (x_go) begin
                        flash_cs_n <= 1'b0;
                        x_bi       <= 5'd0;
                        x_cnt      <= 4'd0;
                        x_match    <= 1'b1;
                        crc_r      <= 8'd0;
                        xs         <= XS_SETUP;
                    end
                end
                XS_SETUP: begin                    // CS 低到第一个 SCK 上升沿 ≥ 60ns
                    x_bit <= 3'd7;
                    x_ph  <= 2'd0;
                    xs    <= XS_SHIFT;
                end
                XS_SHIFT: begin
                    x_ph <= x_ph + 2'd1;
                    case (x_ph)
                        2'd0: flash_mosi <= x_cur[x_bit];
                        2'd1: flash_sclk <= 1'b1;           // 上升沿：Flash 采 MOSI
                        2'd2: ;
                        default: begin                     // 下降沿前一拍采 MISO
                            flash_sclk <= 1'b0;
                            x_rx       <= x_rx_byte[6:0];
                            if (x_data) begin
                                if (miso_r != flash_mosi)
                                    x_match <= 1'b0;
                                if (x_didx[3:0] != 4'd15)     // byte 0..14 进 CRC（逐位，MSB 先）
                                    crc_r <= {crc_r[6:0], 1'b0} ^ ({8{crc_r[7] ^ crc_in}} & 8'h07);
                            end
                            if (x_bit != 3'd0) begin
                                x_bit <= x_bit - 3'd1;
                            end else begin
                                if (x_data) begin
                                    rx_stb  <= 1'b1;
                                    rx_idx  <= x_didx[3:0];
                                    rx_byte <= x_rx_byte;
                                end
                                if (x_bi + 5'd1 >= x_total) begin
                                    xs    <= XS_HOLD;
                                    x_cnt <= 4'd0;
                                end else begin
                                    x_bi  <= x_bi + 5'd1;
                                    x_bit <= 3'd7;
                                end
                            end
                        end
                    endcase
                end
                XS_HOLD: begin                     // 最后一个下降沿之后再保持 CS 低 2 拍
                    x_cnt <= x_cnt + 4'd1;
                    if (x_cnt == 4'd1) begin
                        flash_cs_n <= 1'b1;
                        x_cnt      <= 4'd0;
                        xs         <= XS_GAP;
                    end
                end
                default: begin                     // XS_GAP：CS 高 GAP_CYCLES 拍后报完成
                    x_cnt <= x_cnt + 4'd1;
                    if (x_cnt == GAP_CYCLES - 1) begin
                        x_done <= 1'b1;
                        xs     <= XS_IDLE;
                    end
                end
            endcase
        end
    end

    // =========================== 主状态机 ===========================
    localparam integer TM1  = (ERASE_TIMEOUT_MS > PROGRAM_TIMEOUT_MS) ? ERASE_TIMEOUT_MS : PROGRAM_TIMEOUT_MS;
    localparam integer TM2  = (STARTUP_MS > WAKE_MS) ? STARTUP_MS : WAKE_MS;
    localparam integer TMAX = (TM1 > TM2) ? TM1 : TM2;
    localparam integer TW   = $clog2(TMAX + 1);

    reg  [4:0]  st;
    reg         x_launched;
    reg  [TW-1:0] timer;
    reg         load_slot;           // 正在读的槽
    reg         save_pending;
    reg         target_slot;
    reg         prog_phase;          // 0：擦除前的 WREN；1：编程前的 WREN
    reg         rx_ok;               // 本次读事务逐字节检查都通过（JEDEC / 记录）
    reg  [1:0]  sr_lo;               // 最近一次 SR1 的 {WEL, WIP}

    // 读记录时逐字节锁存的字段（CRC 通过后才拷进 cfg_*）
    reg         rd_v1, rd_v3;
    reg  [7:0]  rd_seq;
    reg  [2:0]  rd_psel;
    reg  [7:0]  rd_b;                // byte 7
    reg  [7:0]  rd_vol;
    reg         rd_auto;
    reg  [7:0]  rd_con;
    reg  [1:0]  rd_flags;            // byte 11 {隐藏, 亮度偏移符号}

    assign busy = (st != ST_READY) || save_pending;

    // 亮度 → 偏移：v3 直接取 {byte11[0], byte7}；v2 的 8 位亮度 b（v1 档位先换成 b）= 无传感器时的显示值，
    // 偏移 = b - 255（9 位补码 = {1,b} + 1），无传感器时基准 255 + 偏移 = b，与旧版逐位相同
    wire [7:0] rec_b   = rd_v1 ? {rd_b[1:0], 6'h3f} : rd_b;
    wire [8:0] rec_off = rd_v3 ? {rd_flags[0], rd_b} : {1'b1, rec_b} + 9'd1;

    // 刚读到的槽是否比已采用的新（模 256 比较：差 1..127 = 较新）；槽 0 时还没有采用的记录，此项不看
    wire [7:0] seq_diff = rd_seq - active_seq;
    wire       rd_newer = (seq_diff != 8'd0) && !seq_diff[7];

    // 发起一次事务
    task launch(input [7:0] cmd, input has_addr, input slot, input [4:0] ndata);
        begin
            x_cmd      <= cmd;
            x_has_addr <= has_addr;
            x_slot     <= slot;
            x_ndata    <= ndata;
            x_go       <= 1'b1;
            x_launched <= 1'b1;
            rx_ok      <= 1'b1;
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st              <= ST_STARTUP;
            x_go            <= 1'b0;
            x_cmd           <= 8'd0;
            x_slot          <= 1'b0;
            x_has_addr      <= 1'b0;
            x_ndata         <= 5'd0;
            x_launched      <= 1'b0;
            timer           <= {TW{1'b0}};
            load_slot       <= 1'b0;
            save_pending    <= 1'b0;
            target_slot     <= 1'b0;
            prog_phase      <= 1'b0;
            rx_ok           <= 1'b0;
            sr_lo           <= 2'b00;
            rd_v1           <= 1'b0;  rd_v3   <= 1'b0;
            rd_seq          <= 8'd0;  rd_psel <= 3'd0;
            rd_b            <= 8'd0;  rd_vol  <= 8'd0;
            rd_auto         <= 1'b0;  rd_con  <= 8'd0;
            rd_flags        <= 2'b00;
            cfg_load_valid  <= 1'b0;
            cfg_period_sel  <= 3'd0;
            cfg_brightness  <= 9'd0;
            cfg_contrast    <= 8'd0;
            cfg_volume      <= 8'd0;
            cfg_auto_mode   <= 1'b0;
            cfg_ui_hide     <= 1'b0;
            save_ok         <= 1'b0;
            error_event     <= 1'b0;
            error_code      <= 8'h00;
            flash_ok        <= 1'b0;
            active_valid    <= 1'b0;
            active_slot     <= 1'b0;
            active_seq    <= 8'd0;
            state_code      <= 5'd0;
        end else begin
            x_go           <= 1'b0;
            cfg_load_valid <= 1'b0;
            save_ok        <= 1'b0;
            error_event    <= 1'b0;
            state_code     <= st;

            if (save_request)
                save_pending <= 1'b1;                          // 忙时再来请求只排队一次，开工时取当时的现值

            // ---- 读到的字节逐个处理（引擎每个数据字节一拍 rx_stb）----
            if (rx_stb) begin
                if (rx_idx == 4'd0)
                    sr_lo <= rx_byte[1:0];                     // SR1 {WEL, WIP}（其它命令的第 0 字节写进来也无妨）
                case (st)
                    ST_PREFLIGHT: begin                        // 9Fh JEDEC ID：厂商非 00/FF 且容量 = 8MB
                        if ((rx_idx == 4'd0 && (rx_byte == 8'h00 || rx_byte == 8'hFF)) ||
                            (rx_idx == 4'd2 && rx_byte != EXPECT_CAPACITY))
                            rx_ok <= 1'b0;
                    end
                    ST_LOAD: begin
                        case (rx_idx)
                            4'd0:  if (rx_byte != 8'h50) rx_ok <= 1'b0;   // "PCFG"
                            4'd1:  if (rx_byte != 8'h43) rx_ok <= 1'b0;
                            4'd2:  if (rx_byte != 8'h46) rx_ok <= 1'b0;
                            4'd3:  if (rx_byte != 8'h47) rx_ok <= 1'b0;
                            4'd4: begin
                                rd_v1 <= (rx_byte == REC_V1);
                                rd_v3 <= (rx_byte == REC_VERSION);
                                if (rx_byte != REC_VERSION && rx_byte != REC_V2 && rx_byte != REC_V1)
                                    rx_ok <= 1'b0;
                            end
                            4'd5:  rd_seq <= rx_byte;
                            4'd6: begin
                                rd_psel <= rx_byte[2:0];
                                if (rx_byte > 8'd5) rx_ok <= 1'b0;
                            end
                            4'd7: begin
                                rd_b <= rx_byte;
                                if (rd_v1 && rx_byte > 8'd3) rx_ok <= 1'b0;
                            end
                            4'd8:  rd_vol <= rx_byte;
                            4'd9: begin
                                rd_auto <= rx_byte[0];
                                if (rx_byte > 8'd1) rx_ok <= 1'b0;
                            end
                            4'd10: rd_con   <= rx_byte;
                            4'd11: rd_flags <= rx_byte[1:0];           // v1/v2 这一字节保留 0 → 不隐藏
                            4'd15: if (rx_byte != crc_r) rx_ok <= 1'b0;  // crc_r = CRC(byte 0..14)
                            default: ;
                        endcase
                    end
                    default: ;
                endcase
            end

            case (st)
                // ---------------- 上电 ----------------
                ST_STARTUP: begin
                    if (tick) timer <= timer + 1'b1;
                    if (timer >= STARTUP_MS) begin
                        timer <= {TW{1'b0}};
                        st    <= ST_WAKE;
                    end
                end
                ST_WAKE: begin                        // ABh：若 Flash 处于掉电模式则唤醒
                    if (!x_launched) begin
                        launch(CMD_WAKE, 1'b0, 1'b0, 5'd0);
                    end else if (x_done) begin
                        x_launched <= 1'b0;
                        timer      <= {TW{1'b0}};
                        st         <= ST_WAKE_WAIT;
                    end
                end
                ST_WAKE_WAIT: begin
                    if (tick) timer <= timer + 1'b1;
                    if (timer >= WAKE_MS) begin
                        timer <= {TW{1'b0}};
                        st    <= ST_PREFLIGHT;
                    end
                end
                ST_PREFLIGHT: begin
                    if (!x_launched) begin
                        launch(CMD_JEDEC, 1'b0, 1'b0, 5'd3);
                    end else if (x_done) begin
                        x_launched <= 1'b0;
                        if (rx_ok) begin
                            flash_ok  <= 1'b1;
                            load_slot <= 1'b0;
                            st        <= ST_LOAD;
                        end else begin
                            flash_ok  <= 1'b0;        // 没有 Flash：不加载，保持默认值
                            st        <= ST_READY;
                        end
                    end
                end

                // ---------------- 加载两个槽：cfg_* 保留有效且 seq 较新的一条 ----------------
                ST_LOAD: begin
                    if (!x_launched) begin
                        launch(CMD_READ, 1'b1, load_slot, 5'd16);
                    end else if (x_done) begin
                        x_launched <= 1'b0;
                        st         <= ST_LOAD_CHK;
                    end
                end
                ST_LOAD_CHK: begin
                    if (rx_ok && (!active_valid || rd_newer)) begin
                        active_valid   <= 1'b1;
                        active_slot    <= load_slot;
                        active_seq   <= rd_seq;
                        cfg_period_sel <= rd_psel;
                        cfg_brightness <= rec_off;
                        cfg_contrast   <= rd_v1 ? 8'd128 : rd_con;
                        cfg_volume     <= rd_vol;
                        cfg_auto_mode  <= rd_auto;
                        cfg_ui_hide    <= rd_flags[1];
                    end
                    load_slot <= 1'b1;
                    if (!load_slot) begin
                        st <= ST_LOAD;
                    end else begin
                        cfg_load_valid <= active_valid | rx_ok;   // 两槽有一个有效就下发（都无效 → 播放器用默认值）
                        st             <= ST_READY;
                    end
                end

                // ---------------- 空闲：等保存请求；开工时冻结本次要写的记录（CRC 在编程时边发边算）----------------
                ST_READY: begin
                    if (save_pending && !save_request) begin
                        save_pending <= 1'b0;
                        if (!flash_ok) begin
                            error_code  <= ERR_FLASH_ABSENT;
                            error_event <= 1'b1;
                        end else begin
                            target_slot    <= active_valid ? ~active_slot : 1'b0;
                            cfg_period_sel <= save_period_sel;
                            cfg_brightness <= save_brightness;
                            cfg_contrast   <= save_contrast;
                            cfg_volume     <= save_volume;
                            cfg_auto_mode  <= save_auto_mode;
                            cfg_ui_hide    <= save_ui_hide;
                            prog_phase     <= 1'b0;
                            st             <= ST_WREN;
                        end
                    end
                end

                // ---------------- 写使能（擦除前、编程前各一次）----------------
                ST_WREN: begin
                    if (!x_launched) begin
                        launch(CMD_WREN, 1'b0, 1'b0, 5'd0);
                    end else if (x_done) begin
                        x_launched <= 1'b0;
                        st         <= ST_WEL;
                    end
                end
                ST_WEL: begin                         // 读 SR1 确认 WEL=1、WIP=0
                    if (!x_launched) begin
                        launch(CMD_RDSR1, 1'b0, 1'b0, 5'd1);
                    end else if (x_done) begin
                        x_launched <= 1'b0;
                        timer      <= {TW{1'b0}};
                        if (sr_lo != 2'b10)
                            st <= ST_FAIL;
                        else if (prog_phase)
                            st <= ST_PROGRAM;
                        else
                            st <= ST_ERASE;
                    end
                end
                ST_ERASE: begin                       // 20h 4KB 扇区擦除（目标槽）
                    if (!x_launched) begin
                        launch(CMD_SE4K, 1'b1, target_slot, 5'd0);
                    end else if (x_done) begin
                        x_launched <= 1'b0;
                        timer      <= {TW{1'b0}};
                        st         <= ST_ERASE_POLL;
                    end
                end
                ST_ERASE_POLL: begin
                    if (tick) timer <= timer + 1'b1;
                    if (!x_launched) begin
                        launch(CMD_RDSR1, 1'b0, 1'b0, 5'd1);
                    end else if (x_done) begin
                        x_launched <= 1'b0;
                        if (!sr_lo[0]) begin
                            prog_phase <= 1'b1;
                            st         <= ST_WREN;
                        end else if (timer >= ERASE_TIMEOUT_MS) begin
                            st         <= ST_FAIL;
                        end
                    end
                end
                ST_PROGRAM: begin                     // 02h 页编程 16 字节
                    if (!x_launched) begin
                        launch(CMD_PP, 1'b1, target_slot, 5'd16);
                    end else if (x_done) begin
                        x_launched <= 1'b0;
                        timer      <= {TW{1'b0}};
                        st         <= ST_PROG_POLL;
                    end
                end
                ST_PROG_POLL: begin
                    if (tick) timer <= timer + 1'b1;
                    if (!x_launched) begin
                        launch(CMD_RDSR1, 1'b0, 1'b0, 5'd1);
                    end else if (x_done) begin
                        x_launched <= 1'b0;
                        if (!sr_lo[0])
                            st <= ST_VERIFY;
                        else if (timer >= PROGRAM_TIMEOUT_MS)
                            st <= ST_FAIL;
                    end
                end
                ST_VERIFY: begin                      // 03h 读回 16 字节，引擎逐位与记录比较
                    if (!x_launched) begin
                        launch(CMD_READ, 1'b1, target_slot, 5'd16);
                    end else if (x_done) begin
                        x_launched <= 1'b0;
                        st         <= ST_VERIFY_CHK;
                    end
                end
                ST_VERIFY_CHK: begin
                    if (x_match) begin
                        active_valid <= 1'b1;
                        active_slot  <= target_slot;
                        active_seq <= seq_next;
                        save_ok      <= 1'b1;
                        st           <= ST_READY;
                    end else begin
                        st <= ST_FAIL;
                    end
                end
                default: begin                        // ST_FAIL：旧槽不动，当前槽不切换
                    error_code  <= ERR_SAVE_FAILED;
                    error_event <= 1'b1;
                    x_launched  <= 1'b0;
                    st          <= ST_READY;
                end
            endcase
        end
    end
endmodule
