`timescale 1ns/1ps

module fat32_volume_scanner #(
    parameter integer MAX_FILES        = 64,
    parameter integer MAX_DIRS         = 32,
    parameter integer MAX_DIR_CLUSTERS = 512
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        scan_start,
    output wire        scan_busy,
    output reg         scan_done,
    output reg  [7:0]  fs_error,
    output reg  [31:0] partition_lba,
    output reg  [7:0]  sectors_per_cluster,
    output reg  [31:0] fat_start_lba,
    output reg  [31:0] data_start_lba,
    output reg  [31:0] cluster_base_lba,     // data_start_lba - 2·spc，给 fat32_cluster_lba
    output wire [31:0] max_cluster,

    output reg  [6:0]  resource_count,
    output reg         resource_table_full,
    output reg         directory_limit_hit,
    output reg  [7:0]  bad_cluster_skip_count,

    // TRACK1~TRACK16.PCM 的 {首扇区, 大小} 写在清单第 128+k 项
    output reg  [15:0] track_valid,
    output reg  [33:0] aux_valid,            // {FOCUS, HZK32, VOC[15:0], LRC[15:0]}

    // 清单 BRAM 端口（fat32_sdp_ram AW=9 DW=32）
    output reg         cat_we,
    output reg  [8:0]  cat_waddr,
    output reg  [31:0] cat_wdata,
    output reg  [8:0]  cat_raddr,
    input  wire [31:0] cat_rdata,

    output reg         sector_req,
    output reg  [31:0] sector_lba,
    input  wire        sector_busy,
    input  wire        sector_valid,
    input  wire [7:0]  sector_byte,
    input  wire [8:0]  sector_byte_index,
    input  wire        sector_done,
    input  wire [7:0]  sector_error
);
    localparam [7:0] ERR_NONE             = 8'h00;
    localparam [7:0] ERR_SECTOR           = 8'h10;
    localparam [7:0] ERR_BOOT_SIGNATURE   = 8'h20;
    localparam [7:0] ERR_UNSUPPORTED_BPB  = 8'h21;
    localparam [7:0] ERR_RESOURCE_MISSING = 8'h22;

    localparam [4:0] ST_IDLE        = 5'd0;
    localparam [4:0] ST_READ0_REQ   = 5'd1;
    localparam [4:0] ST_READ0_WAIT  = 5'd2;
    localparam [4:0] ST_BOOT_REQ    = 5'd3;
    localparam [4:0] ST_BOOT_WAIT   = 5'd4;
    localparam [4:0] ST_BOOT_PARSE  = 5'd5;
    localparam [4:0] ST_BOOT_LAYOUT = 5'd6;
    localparam [4:0] ST_BOOT_MAX    = 5'd7;
    localparam [4:0] ST_START_DIR   = 5'd8;
    localparam [4:0] ST_DQ_WAIT     = 5'd9;
    localparam [4:0] ST_DQ_TAKE     = 5'd10;
    localparam [4:0] ST_C2L         = 5'd11;
    localparam [4:0] ST_DIR_REQ     = 5'd12;
    localparam [4:0] ST_DIR_WAIT    = 5'd13;
    localparam [4:0] ST_DIR_NEXT    = 5'd14;
    localparam [4:0] ST_FAT_REQ     = 5'd15;
    localparam [4:0] ST_FAT_WAIT    = 5'd16;
    localparam [4:0] ST_FAT_DECIDE  = 5'd17;
    localparam [4:0] ST_BUILD       = 5'd18;
    localparam [4:0] ST_BUILD_WAIT  = 5'd19;
    localparam [4:0] ST_BUILD_TAKE  = 5'd20;
    localparam [4:0] ST_FINISH      = 5'd21;
    localparam [4:0] ST_CLEAR       = 5'd22;     // 清编号槽有效位

    reg [4:0] state;
    localparam integer DCW = $clog2(MAX_DIR_CLUSTERS + 1);
    reg [27:0] max_cluster_r;
    assign max_cluster = {4'd0, max_cluster_r};

    // ---------------- 引导扇区 / MBR 字段（字节流里现场锁存）----------------
    reg [7:0]  b_jump;
    reg        sig_ok_lo, sig_ok_hi;     // 510 / 511 = 55 AA
    reg [7:0]  part_type;
    reg [31:0] part_lba_cap;
    reg        bps_ok;                   // 11/12 = 00 02
    reg [7:0]  spc_cap;
    reg [15:0] reserved_sectors;
    reg [7:0]  fat_count;
    reg        fat16_zero;               // 17,18(根目录项数) 22,23(FAT16 大小) 都必须为 0
    reg [31:0] total_sectors;
    reg [31:0] fat_sectors;
    reg [31:0] root_word;                // 44..47 根目录簇号（低 28 位有效）
    wire [27:0] root_cluster = root_word[27:0];
    reg [31:0] data_sector_count;
    reg [7:0]  spc_left;

    // ---------------- 目录遍历 ----------------
    reg [5:0]  dir_head, dir_tail;
    reg [27:0] current_dir_start;
    reg [27:0] dir_cluster;
    reg [DCW-1:0] dir_chain_count;
    reg [7:0]  dir_sector_index;
    reg        dir_end_seen;
    reg [31:0] dir_fat_word;
    reg        c2l_start;
    reg        c2l_ent;                  // L9：这次 c2l 算的是曲目条目的首扇区（完成后覆盖清单字 0）

    // ---------------- 当前目录项（逐字节）----------------
    reg [7:0]  e0;
    reg [7:0]  e_attr;
    reg [15:0] e_cl_hi, e_cl_lo;
    reg [31:0] e_size;
    reg        e_pic;                    // 名字 = PICn / PICnn + 空格补齐
    reg [6:0]  e_pic_n;                  // 编号 - 1；bit6 = 1 表示超过 64
    reg        e_ext_bmp;                // 扩展名 BMP（大小写不敏感）
    reg        size_pending;             // 下一拍写清单项的第 2 个字（文件大小）
    reg        e_trk;                    // 名字 = TRACK1~TRACK16 + 空格补齐
    reg [3:0]  e_trk_digit;              // 曲号 0..15
    reg        e_ext_pcm;                // 扩展名 PCM（大小写不敏感）
    reg        e_ext_lrc;                // 扩1：扩展名 LRC
    reg        e_ext_voc;                // 扩1：扩展名 VOC
    reg        e_hzk, e_asc;             // 扩1：名字 = HZK32 + 3 空格 / 拓展3做深：FOCUS + 3 空格（扩展名不限）

    // ---------------- 清单 ----------------
    reg [6:0]  generic_count;
    reg [7:0]  build_r;                  // 0..63 编号槽，64..127 其余 BMP
    reg [6:0]  build_w;
    reg        build_word;               // 压紧时正在搬第几个字

    assign scan_busy = (state != ST_IDLE);

    wire        c2l_done;
    wire [31:0] c2l_lba;
    wire [27:0] ent_cluster;
    fat32_cluster_lba u_c2l (
        .clk(clk), .rst_n(rst_n), .start(c2l_start), .cluster(c2l_ent ? ent_cluster : dir_cluster),
        .sectors_per_cluster(sectors_per_cluster), .cluster_base_lba(cluster_base_lba),
        .busy(), .done(c2l_done), .lba(c2l_lba)
    );

    // ---------------- 字节流解码 ----------------
    wire        boot_cap  = sector_valid && (state == ST_READ0_WAIT || state == ST_BOOT_WAIT);
    wire        dir_cap   = sector_valid && (state == ST_DIR_WAIT) && !dir_end_seen;
    wire [4:0]  eoff      = sector_byte_index[4:0];
    wire [7:0]  ubyte     = (sector_byte >= "a" && sector_byte <= "z") ? (sector_byte & 8'hDF) : sector_byte;

    // 第 31 字节到达时的完整字段
    assign      ent_cluster = {e_cl_hi[11:0], e_cl_lo};
    wire [31:0] ent_size    = {sector_byte, e_size[23:0]};
    wire        ent_valid   = (e0 != 8'h00) && (e0 != 8'hE5);
    wire        ent_is_dir  = e_attr[4] && !e_attr[3];          // 子目录（排除卷标/LFN）
    wire        ent_regular = (e_attr[4:3] == 2'b00);            // 普通文件（LFN 项 attr=0x0F 被排除）
    wire        cl_in_range = (ent_cluster >= 28'd2) && ({4'd0, ent_cluster} <= max_cluster);
    wire        ent_bmp     = ent_regular && e_ext_bmp && cl_in_range && (ent_size != 32'd0);
    wire        ent_pic_num = e_pic && !e_pic_n[6];                 // PIC1..PIC64 → 编号槽 e_pic_n
    // 曲目类文件 → 清单项 128 + {m_type, m_trk}；PCM/VOC 至少 1 个完整扇区（不满一扇区的尾部不播）
    wire        ent_sized   = cl_in_range && (ent_size != 32'd0);
    wire        ent_media   = ent_regular && ent_sized &&
                              ((e_trk && (e_ext_lrc || ((e_ext_pcm || e_ext_voc) && ent_size[31:9] != 23'd0))) ||
                               ((e_hzk || e_asc) && !e_ext_bmp));      // 不与 BMP 清单写同拍冲突
    wire [1:0]  m_type      = !e_trk ? 2'd3 : e_ext_pcm ? 2'd0 : e_ext_lrc ? 2'd1 : 2'd2;
    wire [3:0]  m_trk       = e_trk ? e_trk_digit : {3'd0, e_asc};
    wire        m_seen      = (m_type == 2'd0) ? track_valid[m_trk] : (m_type == 2'd1) ? aux_valid[{2'b00, m_trk}] :
                              (m_type == 2'd2) ? aux_valid[{2'b01, m_trk}] : aux_valid[{5'b10000, e_asc}];
    wire [31:0] fat_mul2    = fat_count[1] ? {fat_sectors[30:0], 1'b0} : fat_sectors;
    // 两位编号：(10·d1 + d2) - 1 = 10·s + 9 + d2，s = d1 - 1（0..8，查表）；结果 bit6 = 1 即 > 64
    reg  [6:0]  pic_t10;
    always @* begin
        case (e_pic_n[3:0])
            4'd0: pic_t10 = 7'd9;   4'd1: pic_t10 = 7'd19;  4'd2: pic_t10 = 7'd29;
            4'd3: pic_t10 = 7'd39;  4'd4: pic_t10 = 7'd49;  4'd5: pic_t10 = 7'd59;
            4'd6: pic_t10 = 7'd69;  4'd7: pic_t10 = 7'd79;  default: pic_t10 = 7'd89;
        endcase
    end
    // 压紧终点 = 64 + generic_count（generic_count <= 64）：拼位得到，无加法
    wire [7:0]  build_end   = {generic_count[6], ~generic_count[6], generic_count[5:0]};

    task finish_with_error;
        input [7:0] code;
        begin fs_error <= code; state <= ST_FINISH; end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            scan_done <= 1'b0; fs_error <= ERR_NONE;
            sector_req <= 1'b0; sector_lba <= 32'd0;
            partition_lba <= 32'd0; sectors_per_cluster <= 8'd0;
            fat_start_lba <= 32'd0; data_start_lba <= 32'd0; cluster_base_lba <= 32'd0; max_cluster_r <= 28'd0;
            resource_count <= 7'd0; resource_table_full <= 1'b0;
            directory_limit_hit <= 1'b0; bad_cluster_skip_count <= 8'd0;
            cat_we <= 1'b0; cat_waddr <= 9'd0; cat_wdata <= 32'd0; cat_raddr <= 9'd0;
            b_jump <= 8'd0; sig_ok_lo <= 1'b0; sig_ok_hi <= 1'b0;
            part_type <= 8'd0; part_lba_cap <= 32'd0; bps_ok <= 1'b0; spc_cap <= 8'd0;
            reserved_sectors <= 16'd0; fat_count <= 8'd0; fat16_zero <= 1'b0;
            total_sectors <= 32'd0; fat_sectors <= 32'd0; root_word <= 32'd0;
            data_sector_count <= 32'd0; spc_left <= 8'd0;
            dir_head <= 6'd0; dir_tail <= 6'd0; current_dir_start <= 28'd0;
            dir_cluster <= 28'd0; dir_chain_count <= {DCW{1'b0}}; dir_sector_index <= 8'd0;
            dir_end_seen <= 1'b0; dir_fat_word <= 32'd0; c2l_start <= 1'b0; c2l_ent <= 1'b0;
            e0 <= 8'd0; e_attr <= 8'd0; e_cl_hi <= 16'd0; e_cl_lo <= 16'd0; e_size <= 32'd0;
            e_pic <= 1'b0; e_pic_n <= 7'd0; e_ext_bmp <= 1'b0; size_pending <= 1'b0;
            generic_count <= 7'd0; build_r <= 8'd0; build_w <= 7'd0;
            build_word <= 1'b0;
            e_trk <= 1'b0; e_trk_digit <= 4'd0; e_ext_pcm <= 1'b0; track_valid <= 16'd0;
            e_ext_lrc <= 1'b0; e_ext_voc <= 1'b0; e_hzk <= 1'b0; e_asc <= 1'b0; aux_valid <= 34'd0;
        end else begin
            scan_done  <= 1'b0;
            sector_req <= 1'b0;
            cat_we     <= 1'b0;
            c2l_start  <= 1'b0;

            // 清单项第 2 个字（文件大小）紧跟在首簇之后一拍写入
            if (size_pending) begin
                size_pending <= 1'b0;
                cat_we       <= 1'b1;
                cat_waddr    <= {cat_waddr[8:1], 1'b1};
                cat_wdata    <= e_size;
            end

            // L9：曲目条目的首扇区算完 → 覆盖字 0（第 31 字节先照常写了首簇，下一拍 size_pending 写了大小，
            //   cat_waddr 此时还指着该条目的字 1）。c2l 最多 8 拍；下一次目录/FAT 用 c2l 至少隔一个扇区读，
            //   目录项写入至少隔 32 字节，期间不会有别的清单写入
            if (c2l_done && c2l_ent) begin
                c2l_ent   <= 1'b0;
                cat_we    <= 1'b1;
                cat_waddr <= {cat_waddr[8:1], 1'b0};
                cat_wdata <= c2l_lba;
            end

            // ================= 引导扇区 / MBR 字段 =================
            if (boot_cap) begin
                case (sector_byte_index)
                    9'd0:   b_jump <= sector_byte;
                    9'd11:  bps_ok <= (sector_byte == 8'h00);
                    9'd12:  bps_ok <= bps_ok && (sector_byte == 8'h02);
                    9'd13:  spc_cap <= sector_byte;
                    9'd16:  fat_count <= sector_byte;
                    9'd450: part_type <= sector_byte;
                    9'd510: sig_ok_lo <= (sector_byte == 8'h55);
                    9'd511: sig_ok_hi <= (sector_byte == 8'hAA);
                    default: ;
                endcase
                if (sector_byte_index[8:1] == 8'd7)                     // 14,15
                    reserved_sectors <= {sector_byte, reserved_sectors[15:8]};
                if ((sector_byte_index == 9'd17 || sector_byte_index == 9'd18 ||
                     sector_byte_index == 9'd22 || sector_byte_index == 9'd23) && sector_byte != 8'h00)
                    fat16_zero <= 1'b0;
                if (sector_byte_index[8:2] == 7'd8)                     // 32..35
                    total_sectors <= {sector_byte, total_sectors[31:8]};
                if (sector_byte_index[8:2] == 7'd9)                     // 36..39
                    fat_sectors <= {sector_byte, fat_sectors[31:8]};
                if (sector_byte_index[8:2] == 7'd11)                    // 44..47
                    root_word <= {sector_byte, root_word[31:8]};
                if (sector_byte_index >= 9'd454 && sector_byte_index <= 9'd457)
                    part_lba_cap <= {sector_byte, part_lba_cap[31:8]};
            end

            // ================= 目录项字节流 =================
            if (dir_cap) begin
                case (eoff)
                    5'd0:  begin e0 <= sector_byte; e_pic <= (ubyte == "P"); end
                    5'd1:  e_pic <= e_pic && (ubyte == "I");
                    5'd2:  e_pic <= e_pic && (ubyte == "C");
                    5'd3:  begin
                               e_pic <= e_pic && (sector_byte >= "1") && (sector_byte <= "9");
                               e_pic_n <= {3'd0, sector_byte[3:0] - 4'd1}; // "1"=0x31 → 0
                           end
                    5'd4:  if (sector_byte != " ") begin                    // 第二位 "0".."9"
                               e_pic <= e_pic && (sector_byte >= "0") && (sector_byte <= "9");
                               e_pic_n <= pic_t10 + {3'd0, sector_byte[3:0]};
                           end
                    5'd5, 5'd6, 5'd7: e_pic <= e_pic && (sector_byte == " ");
                    5'd8:  e_ext_bmp <= (ubyte == "B");
                    5'd9:  e_ext_bmp <= e_ext_bmp && (ubyte == "M");
                    5'd10: e_ext_bmp <= e_ext_bmp && (ubyte == "P");
                    5'd11: e_attr <= sector_byte;
                    5'd20: e_cl_hi[7:0]  <= sector_byte;
                    5'd21: e_cl_hi[15:8] <= sector_byte;
                    5'd26: e_cl_lo[7:0]  <= sector_byte;
                    5'd27: e_cl_lo[15:8] <= sector_byte;
                    5'd28: e_size[7:0]   <= sector_byte;
                    5'd29: e_size[15:8]  <= sector_byte;
                    5'd30: e_size[23:16] <= sector_byte;
                    5'd31: e_size[31:24] <= sector_byte;
                    default: ;
                endcase
                // 曲目名 "TRACKn  " / "TRACKnn "（n = 1~16），与上面 PIC 名并行比对
                case (eoff)
                    5'd0:  e_trk <= (ubyte == "T");
                    5'd1:  e_trk <= e_trk && (ubyte == "R");
                    5'd2:  e_trk <= e_trk && (ubyte == "A");
                    5'd3:  e_trk <= e_trk && (ubyte == "C");
                    5'd4:  e_trk <= e_trk && (ubyte == "K");
                    5'd5:  begin
                               e_trk <= e_trk && (sector_byte >= "1") && (sector_byte <= "9");
                               e_trk_digit <= sector_byte[3:0] - 4'd1;          // "1" → 0
                           end
                    5'd6:  if (sector_byte != " ") begin                        // 第二位：只允许 10~16
                               e_trk <= e_trk && (e_trk_digit == 4'd0) && (sector_byte >= "0") && (sector_byte <= "6");
                               e_trk_digit <= 4'd9 + {1'b0, sector_byte[2:0]};  // "10" → 9 … "16" → 15
                           end
                    5'd7:  e_trk <= e_trk && (sector_byte == " ");
                    5'd8:  e_ext_pcm <= (ubyte == "P");
                    5'd9:  e_ext_pcm <= e_ext_pcm && (ubyte == "C");
                    5'd10: e_ext_pcm <= e_ext_pcm && (ubyte == "M");
                    default: ;
                endcase
                // 扩1：扩展名 LRC / VOC，字库名 "HZK32   " / 焦点表 "FOCUS   "
                case (eoff)
                    5'd0:  begin e_hzk <= (ubyte == "H"); e_asc <= (ubyte == "F"); end
                    5'd1:  begin e_hzk <= e_hzk && (ubyte == "Z"); e_asc <= e_asc && (ubyte == "O"); end
                    5'd2:  begin e_hzk <= e_hzk && (ubyte == "K"); e_asc <= e_asc && (ubyte == "C"); end
                    5'd3:  begin e_hzk <= e_hzk && (ubyte == "3"); e_asc <= e_asc && (ubyte == "U"); end
                    5'd4:  begin e_hzk <= e_hzk && (ubyte == "2"); e_asc <= e_asc && (ubyte == "S"); end
                    5'd5, 5'd6, 5'd7: begin e_hzk <= e_hzk && (ubyte == " "); e_asc <= e_asc && (ubyte == " "); end
                    5'd8:  begin e_ext_lrc <= (ubyte == "L"); e_ext_voc <= (ubyte == "V"); end
                    5'd9:  begin e_ext_lrc <= e_ext_lrc && (ubyte == "R"); e_ext_voc <= e_ext_voc && (ubyte == "O"); end
                    5'd10: begin e_ext_lrc <= e_ext_lrc && (ubyte == "C"); e_ext_voc <= e_ext_voc && (ubyte == "C"); end
                    default: ;
                endcase

                // ---------- 目录项收齐（第 31 字节）----------
                if (eoff == 5'd31) begin
                    if (e0 == 8'h00) begin
                        dir_end_seen <= 1'b1;                       // 目录结束标记，其后全是空项
                    end else if (ent_valid) begin
                        if (ent_is_dir && (e0 != ".") && cl_in_range &&
                            ent_cluster != root_cluster && ent_cluster != current_dir_start) begin
                            if (dir_tail < MAX_DIRS) begin
                                cat_we    <= 1'b1;
                                cat_waddr <= {3'b110, dir_tail[4:0], 1'b0};          // 项 192 + dir_tail
                                cat_wdata <= {4'd0, ent_cluster};
                                dir_tail  <= dir_tail + 6'd1;
                            end else
                                directory_limit_hit <= 1'b1;
                        end
                        if (ent_bmp) begin
                            if (ent_pic_num || generic_count < MAX_FILES) begin
                                cat_we       <= 1'b1;
                                // 编号槽 n-1 / 64 + generic_count
                                cat_waddr    <= {1'b0, !ent_pic_num, ent_pic_num ? e_pic_n[5:0] : generic_count[5:0], 1'b0};
                                cat_wdata    <= {1'b1, 3'd0, ent_cluster};      // bit31 = 槽位有效
                                size_pending <= 1'b1;
                                if (!ent_pic_num) generic_count <= generic_count + 7'd1;
                            end else
                                resource_table_full <= 1'b1;
                        end
                        if (ent_media && !m_seen) begin
                            // PCM / LRC / VOC / HZK32 / FOCUS：同名只取第一个（同 PIC）；大小在下一拍由 size_pending 写入，
                            // 同时启动 c2l，算完把字 0 覆盖成首扇区 LBA
                            cat_we       <= 1'b1;
                            cat_waddr    <= {2'b10, m_type, m_trk, 1'b0};
                            cat_wdata    <= {4'd0, ent_cluster};
                            size_pending <= 1'b1;
                            c2l_start    <= 1'b1;
                            c2l_ent      <= 1'b1;
                            case (m_type)
                                2'd0:    track_valid[m_trk]                <= 1'b1;
                                2'd1:    aux_valid[{2'b00, m_trk}]         <= 1'b1;
                                2'd2:    aux_valid[{2'b01, m_trk}]         <= 1'b1;
                                default: aux_valid[{5'b10000, e_asc}]      <= 1'b1;   // 32 = HZK32, 33 = FOCUS
                            endcase
                        end
                    end
                end
            end

            // 目录簇链：FAT 表项现场拼
            if (sector_valid && state == ST_FAT_WAIT &&
                sector_byte_index[8:2] == dir_cluster[6:0])
                dir_fat_word <= {sector_byte, dir_fat_word[31:8]};

            case (state)
                ST_IDLE: if (scan_start) begin
                    fs_error <= ERR_NONE; partition_lba <= 32'd0;
                    resource_count <= 7'd0; resource_table_full <= 1'b0;
                    directory_limit_hit <= 1'b0; bad_cluster_skip_count <= 8'd0;
                    generic_count <= 7'd0; build_w <= 7'd0;
                    track_valid <= 16'd0; aux_valid <= 34'd0;
                    dir_head <= 6'd0; dir_tail <= 6'd0;
                    state <= ST_CLEAR;
                end
                // 编号槽 0..63 的字 0 清零（bit31 有效位）；build_w 借作计数，数到 64 进入读 LBA0
                ST_CLEAR: begin
                    if (build_w[6]) begin
                        build_w <= 7'd0;
                        state   <= ST_READ0_REQ;
                    end else begin
                        cat_we    <= 1'b1;
                        cat_waddr <= {2'b00, build_w[5:0], 1'b0};
                        cat_wdata[31] <= 1'b0;          // 只有 bit31 有意义，其余位保持（少一路选择器输入）
                        build_w   <= build_w + 7'd1;
                    end
                end
                ST_READ0_REQ: if (!sector_busy) begin
                    fat16_zero <= 1'b1;
                    sector_lba <= 32'd0; sector_req <= 1'b1; state <= ST_READ0_WAIT;
                end
                ST_READ0_WAIT: if (sector_done) begin
                    if (sector_error != 0) finish_with_error(ERR_SECTOR);
                    else if (!sig_ok_lo || !sig_ok_hi) finish_with_error(ERR_BOOT_SIGNATURE);
                    else if (b_jump == 8'hEB || b_jump == 8'hE9) begin
                        partition_lba <= 32'd0; state <= ST_BOOT_PARSE;     // 超级软盘：本扇区就是引导扇区
                    end else if (part_type == 8'h0B || part_type == 8'h0C) begin
                        partition_lba <= part_lba_cap; sector_lba <= part_lba_cap; state <= ST_BOOT_REQ;
                    end else finish_with_error(ERR_UNSUPPORTED_BPB);
                end
                ST_BOOT_REQ: if (!sector_busy) begin
                    fat16_zero <= 1'b1;
                    sector_req <= 1'b1; state <= ST_BOOT_WAIT;
                end
                ST_BOOT_WAIT: if (sector_done) begin
                    if (sector_error != 0) finish_with_error(ERR_SECTOR);
                    else if (!sig_ok_lo || !sig_ok_hi) finish_with_error(ERR_BOOT_SIGNATURE);
                    else state <= ST_BOOT_PARSE;
                end
                ST_BOOT_PARSE: begin
                    if (!bps_ok || spc_cap == 8'd0 || ((spc_cap & (spc_cap - 8'd1)) != 8'd0) ||
                        reserved_sectors == 16'd0 || (fat_count != 8'd1 && fat_count != 8'd2) ||
                        !fat16_zero || total_sectors == 32'd0 || fat_sectors == 32'd0 ||
                        root_cluster < 28'd2) begin
                        finish_with_error(ERR_UNSUPPORTED_BPB);
                    end else begin
                        sectors_per_cluster <= spc_cap;
                        fat_start_lba <= partition_lba + {16'd0, reserved_sectors};
                        data_sector_count <= total_sectors - {16'd0, reserved_sectors};
                        state <= ST_BOOT_LAYOUT;
                    end
                end
                ST_BOOT_LAYOUT: begin
                    data_start_lba    <= fat_start_lba + fat_mul2;
                    data_sector_count <= data_sector_count - fat_mul2;
                    spc_left <= sectors_per_cluster;
                    state <= ST_BOOT_MAX;
                end
                ST_BOOT_MAX: begin
                    // 数据簇数 = 数据扇区数 / spc：spc 是 2 的幂，每拍右移 1 位
                    if (spc_left[7:1] != 7'd0) begin
                        data_sector_count <= {1'b0, data_sector_count[31:1]};
                        spc_left <= {1'b0, spc_left[7:1]};
                    end else begin
                        max_cluster_r <= data_sector_count[27:0] + 28'd1;
                        cluster_base_lba <= data_start_lba - {23'd0, sectors_per_cluster, 1'b0};
                        cat_we <= 1'b1; cat_waddr <= {3'b110, 5'd0, 1'b0}; cat_wdata <= {4'd0, root_cluster};
                        dir_head <= 6'd0; dir_tail <= 6'd1;
                        state <= ST_START_DIR;
                    end
                end

                // -------- 广度优先：取队首目录 --------
                ST_START_DIR: begin
                    if (dir_head < dir_tail) begin
                        cat_raddr <= {3'b110, dir_head[4:0], 1'b0};
                        dir_head <= dir_head + 6'd1;
                        state <= ST_DQ_WAIT;
                    end else begin
                        build_r <= 8'd0; build_w <= 7'd0; build_word <= 1'b0;
                        state <= ST_BUILD;
                    end
                end
                ST_DQ_WAIT: state <= ST_DQ_TAKE;
                ST_DQ_TAKE: begin
                    current_dir_start <= cat_rdata[27:0];
                    dir_cluster <= cat_rdata[27:0];
                    dir_chain_count <= {{(DCW-1){1'b0}}, 1'b1};
                    dir_sector_index <= 8'd0;
                    dir_end_seen <= 1'b0;
                    c2l_start <= 1'b1;
                    state <= ST_C2L;
                end
                ST_C2L: if (c2l_done) begin
                    sector_lba <= c2l_lba;
                    state <= ST_DIR_REQ;
                end
                ST_DIR_REQ: if (!sector_busy) begin sector_req <= 1'b1; state <= ST_DIR_WAIT; end
                ST_DIR_WAIT: if (sector_done) begin
                    if (sector_error != 0) finish_with_error(ERR_SECTOR); else state <= ST_DIR_NEXT;
                end
                ST_DIR_NEXT: begin
                    if (dir_end_seen) state <= ST_START_DIR;
                    else if (dir_sector_index + 8'd1 < sectors_per_cluster) begin
                        dir_sector_index <= dir_sector_index + 8'd1;
                        sector_lba <= sector_lba + 32'd1; state <= ST_DIR_REQ;
                    end else begin
                        sector_lba <= fat_start_lba + {11'd0, dir_cluster[27:7]};
                        state <= ST_FAT_REQ;
                    end
                end
                ST_FAT_REQ: if (!sector_busy) begin
                    dir_fat_word <= 32'd0;
                    sector_req <= 1'b1; state <= ST_FAT_WAIT;
                end
                ST_FAT_WAIT: if (sector_done) begin
                    if (sector_error != 0) finish_with_error(ERR_SECTOR); else state <= ST_FAT_DECIDE;
                end
                ST_FAT_DECIDE: begin
                    if (dir_fat_word[27:0] == 28'hFFFFFF7) begin
                        // 坏簇：跳过这个目录剩余部分，继续扫描其它目录
                        bad_cluster_skip_count <= bad_cluster_skip_count + 8'd1; state <= ST_START_DIR;
                    end else if (dir_fat_word[27:0] >= 28'hFFFFFF8 || dir_fat_word[27:0] < 28'd2 ||
                                 {4'd0, dir_fat_word[27:0]} > max_cluster ||
                                 dir_fat_word[27:0] == dir_cluster || dir_chain_count >= MAX_DIR_CLUSTERS) begin
                        state <= ST_START_DIR;
                    end else begin
                        dir_cluster <= dir_fat_word[27:0];
                        dir_chain_count <= dir_chain_count + 1'b1;
                        dir_sector_index <= 8'd0;
                        c2l_start <= 1'b1;
                        state <= ST_C2L;
                    end
                end

                // -------- 压紧清单：编号槽 0..63 中有效的（按编号）+ 其余 BMP，按 2 个字顺序搬到 0..n-1 --------
                //   编号槽先读字 0，bit31 = 0 是空槽，跳过不写；w <= r，写入只覆盖已读过的项
                ST_BUILD: begin
                    if (build_r == build_end) begin
                        state <= ST_FINISH;
                    end else if (build_w >= MAX_FILES) begin
                        resource_table_full <= 1'b1;
                        state <= ST_FINISH;
                    end else begin
                        cat_raddr <= {build_r, build_word};
                        state <= ST_BUILD_WAIT;
                    end
                end
                ST_BUILD_WAIT: state <= ST_BUILD_TAKE;
                ST_BUILD_TAKE: begin
                    if (!build_word && !build_r[6] && !cat_rdata[31]) begin
                        build_r <= build_r + 8'd1;                      // 空编号槽
                    end else begin
                        cat_we     <= 1'b1;
                        cat_waddr  <= {1'b0, build_w, build_word};
                        cat_wdata  <= cat_rdata;
                        build_word <= ~build_word;
                        if (build_word) begin
                            build_w <= build_w + 7'd1;
                            build_r <= build_r + 8'd1;
                        end
                    end
                    state <= ST_BUILD;
                end
                ST_FINISH: begin
                    resource_count <= build_w;
                    if (fs_error == ERR_NONE && build_w == 7'd0) fs_error <= ERR_RESOURCE_MISSING;
                    scan_done <= 1'b1; state <= ST_IDLE;
                end
                default: state <= ST_IDLE;
            endcase
        end
    end
endmodule
