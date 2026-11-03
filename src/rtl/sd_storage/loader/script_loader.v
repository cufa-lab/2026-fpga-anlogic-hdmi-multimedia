`timescale 1ns/1ps

module script_loader #(
    parameter integer MAX_W  = 72,           // 单行最多半宽字数
    parameter integer ROW_W  = 72            // 全文页段落每行半宽（1× 字 576 像素）
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        enable,               // 扫描完成且启动序列放行
    input  wire [33:0] aux_valid,            // {FOCUS, HZK32, VOC16..1, LRC16..1}
    input  wire [4:0]  select,               // 当前图片的 {无曲目, 曲号 0..15}（sys 域电平）
    // pcm_track_locator B 口
    output reg         loc_req,
    output reg  [5:0]  loc_idx,
    input  wire        loc_valid,
    input  wire [31:0] loc_lba,
    input  wire [31:0] loc_size,
    // 仲裁器 C 口
    output wire        sec_req,
    output wire [31:0] sec_lba,
    input  wire        sec_busy,
    input  wire        sec_valid,
    input  wire [7:0]  sec_byte,
    input  wire [8:0]  sec_byte_index,
    input  wire        sec_done,
    input  wire [7:0]  sec_error,
    // 人声播放器（wav_stream_player 写侧）
    input  wire        voc_sd_req,
    input  wire [31:0] voc_sd_lba,
    output wire        voc_sd_busy,
    output wire        voc_sd_valid,
    output wire        voc_sd_done,
    output reg         voc_play_enable,
    output reg  [31:0] voc_start_lba,
    output reg  [31:0] voc_total_sectors,
    input  wire        voc_wr_idle,
    input  wire        voc_wr_done,
    input  wire        voc_wr_error,
    // 字形 RAM 写口（script_glyph_bram32k A 口）
    output reg         glyph_we,
    output reg         glyph_park,           // 1 = 写块 6、7（暂存 fidx），0 = 写块 glyph_blk
    output reg  [2:0]  glyph_blk,
    output reg  [12:0] glyph_addr,
    output reg  [15:0] glyph_data,
    output reg         glyph_re,             // 读回暂存的 fidx（下一拍之后 glyph_q 有效）
    input  wire [15:0] glyph_q,
    // 去重表（script_dedup_bram32k）
    output reg         dd_we,
    output reg         dd_re,
    output reg  [11:0] dd_addr,
    output reg  [7:0]  dd_data,
    input  wire [7:0]  dd_q,
    output reg         code_we,
    output reg  [8:0]  code_addr,
    output reg  [9:0]  code_data,
    output reg         line_we,
    output reg  [8:0]  line_addr,
    output reg  [17:0] line_data,
    // 与 pix 的握手（toggle）
    output reg         rel_req_t,
    input  wire        rel_ack_t,            // pix 域 toggle，这里 2FF
    output reg         ready_t,
    output reg         epoch_t,
    output reg  [7:0]  script_error
);
    // ---------------- 主状态 ----------------
    localparam [4:0] S_INIT    = 5'd0;
    localparam [4:0] S_HZKLOC  = 5'd1;
    localparam [4:0] S_RUN     = 5'd2;
    localparam [4:0] S_VLOOP   = 5'd3;
    localparam [4:0] S_REL     = 5'd4;
    localparam [4:0] S_RELW    = 5'd5;
    localparam [4:0] S_CLR0    = 5'd6;
    localparam [4:0] S_CLR1    = 5'd7;
    localparam [4:0] S_LRCLOC  = 5'd8;
    localparam [4:0] S_RQ      = 5'd9;
    localparam [4:0] S_WT      = 5'd10;
    localparam [4:0] S_LEOF    = 5'd11;
    localparam [4:0] S_LEOF2   = 5'd12;
    localparam [4:0] S_LEOF3   = 5'd13;
    localparam [4:0] S_FRD     = 5'd14;
    localparam [4:0] S_FRD2    = 5'd15;
    localparam [4:0] S_FC      = 5'd16;
    localparam [4:0] S_HDR     = 5'd17;
    localparam [4:0] S_HDR1    = 5'd18;
    localparam [4:0] S_HDR2    = 5'd19;
    localparam [4:0] S_VOCLOC  = 5'd20;
    localparam [4:0] S_VSTART  = 5'd21;

    // ---------------- 解析器状态 ----------------
    localparam [3:0] P_LS     = 4'd0;        // 行首
    localparam [3:0] P_TAG    = 4'd1;        // '[' 之后第一个字符
    localparam [3:0] P_TI     = 4'd2;        // 't' 之后等 'i'
    localparam [3:0] P_AR     = 4'd3;        // 'a' 之后等 'r'
    localparam [3:0] P_COLON  = 4'd4;        // ti / ar 之后等 ':'
    localparam [3:0] P_TIME   = 4'd5;
    localparam [3:0] P_SKIP   = 4'd6;        // 跳到行尾
    localparam [3:0] P_TEXT   = 4'd7;
    localparam [3:0] P_TEXTLO = 4'd8;
    localparam [3:0] P_STOP   = 4'd9;        // 编码错，忽略余下字节

    reg [4:0]  state;
    reg        hzk_phase;                    // 0 = LRC 1 = 字库
    reg        first;
    reg [4:0]  sel_q;
    reg        font_ok;
    reg [31:0] hzk_lba;
    reg [31:0] lrc_lba;
    reg [12:0] lrc_size;
    reg [31:0] rd_lba;
    reg [2:0]  secn;                         // 本文件内的扇区序号
    reg [3:0]  nsec;                         // 要读的扇区数
    reg        own_ld;
    reg        ld_req;
    (* async_reg = "true" *) reg ack_sync1, ack_sync2;   // rel_ack_t 2FF（SDC 已有 */ack_sync1 规则）
    // 字库
    reg [8:0]  hz_k;                         // 正在取的槽（9 位：256 个槽满时要能数到 256）
    reg        hz_half;                      // 本槽字在扇区的前 / 后 256 字节
    // 解析器
    reg [3:0]  pst;
    reg [1:0]  tf;                           // 0 = 分 1 = 秒 2 = 小数
    reg [1:0]  fdig;
    reg [6:0]  num;
    reg [17:0] tcs;
    reg        tfix;                         // ']' 之后一拍做 *100
    reg        ttag;                         // 本行是 ti / ar（']' 结束）
    reg        tis_ar;
    reg [6:0]  tgt;                          // 目标条号
    reg [7:0]  line_n;                       // 下一条时间行下标（2..112）
    reg [9:0]  ci;                           // code_ram 写指针
    reg [9:0]  lstart;
    reg [8:0]  s0;                           // 讲解流起点（第一条时间行的 code 下标）
    reg [6:0]  lwidth;
    reg [8:0]  hz_n;                         // 已分配的槽数
    reg [7:0]  hi_b;
    reg        full;
    reg        p_bad;
    // 讲解流 / 段落行
    reg [10:0] gsum;                         // 流内半宽偏移（下一个字的 G）
    reg [10:0] gline;                        // 本时间行起点 G
    reg [6:0]  row_fill;                     // 本段落行已用半宽（首行含 4 缩进）
    reg [8:0]  row_c0;
    reg [10:0] row_g0;
    reg [4:0]  rows;
    // 行表写（字节间隙里 3 拍）
    reg [1:0]  lw_step;                      // 3,2,1 = w0,w1,w2
    reg        lw_go;
    reg [17:0] lw_t;
    reg [10:0] lw_s;
    reg [6:0]  lw_w;
    reg [6:0]  lw_tgt;
    // 查重流水
    reg        cq_go, cq_wide, cq_nb;
    reg [6:0]  cq_a, cq_b;
    reg [12:0] fidx;
    reg [2:0]  dq;
    reg [7:0]  dslot;

    // ---------------- 扇区口二选一 ----------------
    assign sec_req      = own_ld ? ld_req : voc_sd_req;
    assign sec_lba      = own_ld ? rd_lba : voc_sd_lba;
    assign voc_sd_busy  = own_ld | sec_busy;
    assign voc_sd_valid = !own_ld && sec_valid;
    assign voc_sd_done  = !own_ld && sec_done;

    wire        ld_byte  = own_ld && sec_valid;
    wire [11:0] foff     = {secn, sec_byte_index};
    wire        lrc_byte = ld_byte && !hzk_phase && ({1'b0, foff} < lrc_size);
    wire        rel_done = (ack_sync2 == rel_req_t);
    wire        lrc_ok   = !sel_q[4] && aux_valid[{2'b00, sel_q[3:0]}];   // TRACKn.LRC 存在（无曲目 = 0）
    wire        voc_ok   = !sel_q[4] && aux_valid[{2'b01, sel_q[3:0]}];   // TRACKn.VOC 存在

    // 字节分类
    wire [7:0]  b       = sec_byte;
    wire        is_dig  = (b >= "0") && (b <= "9");
    wire [3:0]  dval    = b[3:0];
    wire        is_eol  = (b == 8'h0D) || (b == 8'h0A);
    wire        is_asc  = (b >= 8'h20) && (b <= 8'h7E);
    wire        is_hi   = (b >= 8'hA1) && (b <= 8'hF7);
    wire        is_lo   = (b >= 8'hA1) && (b <= 8'hFE);
    wire [7:0]  lb      = b | 8'h20;                       // 小写（只用于字母比较）
    wire [6:0]  num10   = {num[3:0], 3'b000} + {num[5:0], 1'b0} + {3'd0, dval};   // num*10 + d（num <= 9 时）
    wire [17:0] t60     = {tcs[11:0], 6'd0} - {tcs[15:0], 2'd0} + {11'd0, num};   // T*60 + num
    wire [6:0]  frac    = (fdig == 2'd1) ? {num[3:0], 3'b000} + {num[5:0], 1'b0} : num;
    wire [17:0] t100    = {tcs[11:0], 6'd0} + {tcs[12:0], 5'd0} + {tcs[15:0], 2'd0} + {11'd0, frac};
    // 本字符半宽
    wire        ch_wide = (pst == P_TEXTLO);
    wire [7:0]  w_next  = {1'b0, lwidth} + (ch_wide ? 8'd2 : 8'd1);
    // 不放行首的标点：、。…’” 〕〉》」』〗】 ！），．：；？ 以及 ASCII , . ; : ! ? ) ]
    wire        nb_a1   = (b == 8'hA2) || (b == 8'hA3) || (b == 8'hAD) || (b == 8'hAF) ||
                          (b[0] && b >= 8'hB1 && b <= 8'hBF);
    wire        nb_a3   = (b == 8'hA1) || (b == 8'hA9) || (b == 8'hAC) || (b == 8'hAE) ||
                          (b == 8'hBA) || (b == 8'hBB) || (b == 8'hBF);
    wire        nb_asc  = (b == ",") || (b == ".") || (b == ";") || (b == ":") || (b == "!") ||
                          (b == "?") || (b == ")") || (b == "]");
    wire        ch_nb   = ch_wide ? ((hi_b == 8'hA1) ? nb_a1 : (hi_b == 8'hA3) ? nb_a3 : 1'b0) : nb_asc;
    // 提交（查重结束那一拍）
    wire        cm_hit  = (dq == 3'd5) && (glyph_q[12:0] == fidx);
    wire        dd_hit  = ({1'b0, dd_q} < hz_n);
    wire        cm_miss = ((dq == 3'd3) && !dd_hit) || ((dq == 3'd5) && !(glyph_q[12:0] == fidx));
    wire        cm_drop = cm_miss && hz_n[8];                                // 256 个槽已满
    wire        cm_ok   = (cm_hit || cm_miss) && !cm_drop;
    wire [7:0]  cm_slot = cm_hit ? dslot : hz_n[7:0];
    wire [1:0]  cw      = cq_wide ? 2'd2 : 2'd1;
    wire [6:0]  rf_n    = row_fill + {5'd0, cw};
    wire        rf_brk  = (rf_n > ROW_W) && !(cq_nb && rf_n <= ROW_W + 2);
    wire [6:0]  rf_ind  = (rows == 5'd0) ? 7'd4 : 7'd0;                    // 首行缩进 2 字

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_INIT; hzk_phase <= 1'b0; first <= 1'b1; sel_q <= 5'd0; font_ok <= 1'b0;
            hzk_lba <= 32'd0; lrc_lba <= 32'd0; lrc_size <= 13'd0; rd_lba <= 32'd0;
            secn <= 3'd0; nsec <= 4'd0; own_ld <= 1'b0; ld_req <= 1'b0;
            ack_sync1 <= 1'b0; ack_sync2 <= 1'b0;
            hz_k <= 9'd0; hz_half <= 1'b0;
            pst <= P_LS; tf <= 2'd0; fdig <= 2'd0; num <= 7'd0; tcs <= 18'd0; tfix <= 1'b0;
            ttag <= 1'b0; tis_ar <= 1'b0; tgt <= 7'd0; line_n <= 8'd2; ci <= 10'd0; lstart <= 10'd0; s0 <= 9'd0;
            lwidth <= 7'd0; hz_n <= 9'd0; hi_b <= 8'd0; full <= 1'b0; p_bad <= 1'b0;
            gsum <= 11'd0; gline <= 11'd0; row_fill <= 7'd4; row_c0 <= 9'd0; row_g0 <= 11'd0; rows <= 5'd0;
            lw_step <= 2'd0; lw_go <= 1'b0; lw_t <= 18'd0; lw_s <= 11'd0; lw_w <= 7'd0; lw_tgt <= 7'd0;
            cq_go <= 1'b0; cq_wide <= 1'b0; cq_nb <= 1'b0; cq_a <= 7'd0; cq_b <= 7'd0; fidx <= 13'd0;
            dq <= 3'd0; dslot <= 8'd0;
            loc_req <= 1'b0; loc_idx <= 6'd0;
            voc_play_enable <= 1'b0; voc_start_lba <= 32'd0; voc_total_sectors <= 32'd0;
            glyph_we <= 1'b0; glyph_park <= 1'b0; glyph_blk <= 3'd0; glyph_addr <= 13'd0; glyph_data <= 16'd0;
            glyph_re <= 1'b0;
            dd_we <= 1'b0; dd_re <= 1'b0; dd_addr <= 12'd0; dd_data <= 8'd0;
            code_we <= 1'b0; code_addr <= 9'd0; code_data <= 10'd0;
            line_we <= 1'b0; line_addr <= 9'd0; line_data <= 18'd0;
            rel_req_t <= 1'b0; ready_t <= 1'b0; epoch_t <= 1'b0; script_error <= 8'h00;
        end else begin
            loc_req  <= 1'b0;
            ld_req   <= 1'b0;
            glyph_we <= 1'b0;
            glyph_re <= 1'b0;
            dd_we    <= 1'b0;
            dd_re    <= 1'b0;
            code_we  <= 1'b0;
            line_we  <= 1'b0;
            cq_go    <= 1'b0;
            ack_sync1 <= rel_ack_t;
            ack_sync2 <= ack_sync1;

            // ================= 字形字节（本槽所在的半个扇区，按字节直写）=================
            if (ld_byte && hzk_phase && sec_byte_index[8] == hz_half) begin
                glyph_we   <= 1'b1;
                glyph_park <= 1'b0;
                glyph_blk  <= sec_byte_index[2:0];
                glyph_addr <= {hz_k[7:0], sec_byte_index[7:3]};
                glyph_data <= {sec_byte, sec_byte};
            end

            // ================= 行表写（3 拍）=================
            if (lw_go) begin
                lw_go   <= 1'b0;
                lw_step <= 2'd3;
            end
            if (lw_step != 2'd0) begin
                lw_step   <= lw_step - 2'd1;
                line_we   <= 1'b1;
                line_addr <= {lw_tgt, 2'd3 - lw_step};                     // 3→w0 2→w1 1→w2
                line_data <= (lw_step == 2'd3) ? lw_t :
                             (lw_step == 2'd2) ? {7'd0, lw_s} : {11'd0, lw_w};
            end

            // ================= 时间 *100（']' 之后一拍）=================
            if (tfix) begin
                tfix <= 1'b0;
                tcs  <= t100;
            end

            // ================= 查重：fidx → hash 表 → 核对暂存 fidx → 提交 =================
            case (dq)
            3'd0: if (cq_go) begin
                fidx <= {cq_a, 6'd0} + {1'b0, cq_a, 5'd0} - {5'd0, cq_a, 1'b0} + {6'd0, cq_b};   // a*94 + b
                dq   <= 3'd1;
            end
            3'd1: begin dd_re <= 1'b1; dd_addr <= fidx[11:0]; dq <= 3'd2; end
            3'd2: dq <= 3'd3;
            3'd3: begin
                dslot <= dd_q;
                if (dd_hit) begin
                    glyph_re <= 1'b1; glyph_addr <= {dd_q, 5'd0}; dq <= 3'd4;
                end else
                    dq <= 3'd0;
            end
            3'd4: dq <= 3'd5;
            default: dq <= 3'd0;
            endcase
            if (cm_drop) full <= 1'b1;
            if (cm_miss && !cm_drop) begin
                dd_we <= 1'b1; dd_addr <= fidx[11:0]; dd_data <= hz_n[7:0];
                glyph_we <= 1'b1; glyph_park <= 1'b1; glyph_addr <= {hz_n[7:0], 5'd0}; glyph_data <= {3'd0, fidx};
                hz_n <= hz_n + 9'd1;
            end
            if (cm_ok) begin
                code_we   <= 1'b1;
                code_addr <= ci[8:0];
                code_data <= {cq_wide, 1'b0, cm_slot};
                ci        <= ci + 10'd1;
                lwidth    <= lwidth + {5'd0, cw};
                if (!ttag) begin
                    gsum <= gsum + {9'd0, cw};
                    if (rf_brk) begin                                       // 换段落行：写上一行
                        if (rows[4]) full <= 1'b1;
                        else begin
                            lw_go <= 1'b1; lw_tgt <= {3'b111, rows[3:0]};
                            lw_t <= {9'd0, row_c0}; lw_s <= row_g0; lw_w <= row_fill - rf_ind;
                            rows <= rows + 5'd1;
                        end
                        row_c0 <= ci[8:0]; row_g0 <= gsum; row_fill <= {5'd0, cw};
                    end else
                        row_fill <= rf_n;
                end
            end

            // ================= LRC 字节流 =================
            if (lrc_byte) begin
                case (pst)
                P_LS: begin
                    if (b == "[") begin
                        pst <= P_TAG; tf <= 2'd0; fdig <= 2'd0; num <= 7'd0; tcs <= 18'd0;
                    end else if (b[7]) begin
                        pst <= P_STOP; p_bad <= 1'b1;                    // UTF-8 BOM / 非 ASCII 行首
                    end else if (!is_eol)
                        pst <= P_SKIP;
                end
                P_TAG: begin
                    tis_ar <= (lb == "a");
                    if (is_dig) begin num <= {3'd0, dval}; pst <= P_TIME; end
                    else if (lb == "t") pst <= P_TI;
                    else if (lb == "a") pst <= P_AR;
                    else pst <= is_eol ? P_LS : P_SKIP;
                end
                P_TI: pst <= (lb == "i") ? P_COLON : is_eol ? P_LS : P_SKIP;
                P_AR: pst <= (lb == "r") ? P_COLON : is_eol ? P_LS : P_SKIP;
                P_COLON: begin
                    if (b == ":") begin
                        pst <= P_TEXT; ttag <= 1'b1; tgt <= {6'd0, tis_ar};
                        lstart <= ci; lwidth <= 7'd0;
                    end else pst <= is_eol ? P_LS : P_SKIP;
                end
                P_TIME: begin
                    if (is_dig) begin
                        if (tf != 2'd2 || fdig != 2'd2) begin
                            num <= num10;
                            if (tf == 2'd2) fdig <= fdig + 2'd1;
                        end
                    end else if (b == ":" && tf == 2'd0) begin
                        tcs <= {11'd0, num}; num <= 7'd0; tf <= 2'd1;
                    end else if (b == "." && tf == 2'd1) begin
                        tcs <= t60; num <= 7'd0; tf <= 2'd2;
                    end else if (b == "]" && tf != 2'd0) begin
                        if (tf == 2'd1) begin tcs <= t60; num <= 7'd0; fdig <= 2'd0; end
                        tfix <= 1'b1;
                        pst <= P_TEXT; ttag <= 1'b0; tgt <= line_n[6:0];
                        lstart <= ci; lwidth <= 7'd0; gline <= gsum;
                        if (line_n == 8'd2) begin s0 <= ci[8:0]; row_c0 <= ci[8:0]; end
                    end else
                        pst <= is_eol ? P_LS : P_SKIP;
                end
                P_SKIP: if (b == 8'h0A) pst <= P_LS;
                P_TEXT, P_TEXTLO: begin
                    if (pst == P_TEXT && (is_eol || (ttag && b == "]"))) begin
                        // 行尾：写 {时间, 起点, 宽度}
                        lw_t <= tcs; lw_s <= ttag ? {1'b0, lstart} : gline; lw_w <= lwidth; lw_tgt <= tgt;
                        if (ttag) lw_go <= 1'b1;
                        else if (line_n[6:4] == 3'b111) full <= 1'b1;   // 第 111 条之后的时间行丢弃
                        else begin lw_go <= 1'b1; line_n <= line_n + 8'd1; end
                        pst <= (ttag && b == "]") ? P_SKIP : P_LS;
                    end else if (pst == P_TEXT && is_hi) begin
                        hi_b <= b; pst <= P_TEXTLO;
                    end else if ((pst == P_TEXT && (is_asc || b == 8'h09)) || (pst == P_TEXTLO && is_lo)) begin
                        pst <= P_TEXT;
                        if (w_next > MAX_W) begin
                            if (script_error == 8'h00) script_error <= 8'h76;
                        end else if (ci[9] || full) begin
                            full <= 1'b1;
                        end else begin
                            cq_go   <= 1'b1;
                            cq_wide <= ch_wide;
                            cq_nb   <= ch_nb;
                            cq_a    <= ch_wide ? hi_b[6:0] - 7'h21 : 7'd10;
                            cq_b    <= ch_wide ? b[6:0] - 7'h21 : (b == 8'h09) ? 7'h20 : b[6:0];
                        end
                    end else begin
                        pst <= P_STOP; p_bad <= 1'b1;                    // 非 GB2312
                    end
                end
                default: ;                                              // P_STOP
                endcase
            end

            // ================= 主状态机 =================
            case (state)
            S_INIT: if (enable) begin
                if (aux_valid[32]) begin loc_req <= 1'b1; loc_idx <= 6'h30; state <= S_HZKLOC; end
                else state <= S_RUN;
            end
            S_HZKLOC: if (loc_valid) begin
                hzk_lba <= loc_lba;
                font_ok <= 1'b1;
                state   <= S_RUN;
            end

            S_RUN: begin
                if (first || select != sel_q) begin
                    first <= 1'b0;
                    state <= S_REL;
                end else if (voc_play_enable && (voc_wr_done || voc_wr_error)) begin
                    voc_play_enable <= 1'b0;                            // 人声单曲循环
                    state <= S_VLOOP;
                end
            end
            S_VLOOP: if (voc_wr_idle && !voc_play_enable) state <= S_VSTART;

            // -------- 换图：停人声 + 释放握手 --------
            S_REL: begin
                voc_play_enable <= 1'b0;
                rel_req_t <= ~rel_req_t;
                state <= S_RELW;
            end
            S_RELW: if (rel_done && voc_wr_idle) begin
                sel_q <= select;
                script_error <= 8'h00;
                own_ld <= 1'b1;
                state <= S_CLR0;
            end
            S_CLR0: begin                                               // 条 0 / 条 1 宽度清 0（没有 ti/ar 时）
                line_we <= 1'b1; line_addr <= {7'd0, 2'd2}; line_data <= 18'd0;
                state <= S_CLR1;
            end
            S_CLR1: begin
                line_we <= 1'b1; line_addr <= {7'd1, 2'd2}; line_data <= 18'd0;
                line_n <= 8'd2; ci <= 10'd0; hz_n <= 9'd0; full <= 1'b0; p_bad <= 1'b0; s0 <= 9'd0;
                gsum <= 11'd0; row_fill <= 7'd4; row_c0 <= 9'd0; row_g0 <= 11'd0; rows <= 5'd0;
                if (!lrc_ok)
                    state <= S_HDR;                                     // 本图没有讲解稿
                else if (!font_ok) begin
                    script_error <= 8'h73; p_bad <= 1'b1; state <= S_HDR;
                end else begin
                    loc_req <= 1'b1; loc_idx <= {2'b01, sel_q[3:0]}; state <= S_LRCLOC;
                end
            end
            S_LRCLOC: if (loc_valid) begin
                if (loc_size > 32'd4096) begin
                    script_error <= 8'h75; p_bad <= 1'b1; state <= S_HDR;
                end else begin
                    lrc_lba  <= loc_lba;
                    lrc_size <= loc_size[12:0];
                    rd_lba   <= loc_lba;
                    secn     <= 3'd0;
                    nsec     <= {1'b0, loc_size[12:9]} + {3'd0, (loc_size[8:0] != 9'd0)};
                    hzk_phase <= 1'b0;
                    pst      <= P_LS;
                    state    <= S_RQ;
                end
            end

            // -------- 扇区读（LRC / 字库共用）--------
            S_RQ: if (!sec_busy) begin
                ld_req <= 1'b1;
                state  <= S_WT;
            end
            S_WT: if (sec_done) begin
                if (sec_error != 8'h00) begin
                    if (!hzk_phase) begin                               // 解析不幂等：整份重来
                        rd_lba <= lrc_lba; secn <= 3'd0;
                        nsec   <= {1'b0, lrc_size[12:9]} + {3'd0, (lrc_size[8:0] != 9'd0)};
                        line_n <= 8'd2; ci <= 10'd0; hz_n <= 9'd0; full <= 1'b0; p_bad <= 1'b0; s0 <= 9'd0;
                        gsum <= 11'd0; row_fill <= 7'd4; row_c0 <= 9'd0; row_g0 <= 11'd0; rows <= 5'd0;
                        pst    <= P_LS;
                        if (script_error == 8'h76) script_error <= 8'h00;
                    end
                    state <= S_RQ;                                      // 字库扇区：重读本扇区
                end else if (hzk_phase) begin
                    hz_k  <= hz_k + 9'd1;
                    state <= S_FRD;
                end else if (nsec > 4'd1) begin
                    nsec   <= nsec - 4'd1;
                    secn   <= secn + 3'd1;
                    rd_lba <= rd_lba + 32'd1;
                    state  <= S_RQ;
                end else
                    state <= S_LEOF;
            end
            S_LEOF: if (dq == 3'd0 && !cq_go && lw_step == 2'd0 && !lw_go) begin   // 文件结尾没有换行
                if (pst == P_TEXT || pst == P_TEXTLO) begin
                    lw_t <= tcs; lw_s <= ttag ? {1'b0, lstart} : gline; lw_w <= lwidth; lw_tgt <= tgt;
                    if (ttag) lw_go <= 1'b1;
                    else if (line_n[6:4] == 3'b111) full <= 1'b1;
                    else begin lw_go <= 1'b1; line_n <= line_n + 8'd1; end
                end
                pst   <= P_LS;
                state <= S_LEOF2;
            end
            S_LEOF2: if (!lw_go && lw_step == 2'd0 && !tfix) begin       // 最后一个段落行
                if (row_fill != rf_ind) begin
                    if (rows[4]) full <= 1'b1;
                    else begin
                        lw_go <= 1'b1; lw_tgt <= {3'b111, rows[3:0]};
                        lw_t <= {9'd0, row_c0}; lw_s <= row_g0; lw_w <= row_fill - rf_ind;
                        rows <= rows + 5'd1;
                    end
                end
                state <= S_LEOF3;
            end
            S_LEOF3: if (!lw_go && lw_step == 2'd0) begin
                if (p_bad && script_error != 8'h71) script_error <= 8'h71;
                else if (full && (script_error == 8'h00 || script_error == 8'h76)) script_error <= 8'h72;
                hz_k  <= 9'd0;
                hzk_phase <= 1'b1;
                state <= S_FRD;
            end

            // -------- 取字形：每槽读半个扇区 --------
            S_FRD: begin
                if (hz_k == hz_n || p_bad) state <= S_HDR;
                else begin glyph_re <= 1'b1; glyph_addr <= {hz_k[7:0], 5'd0}; state <= S_FRD2; end
            end
            S_FRD2: state <= S_FC;
            S_FC: begin
                rd_lba  <= hzk_lba + {20'd0, glyph_q[12:1]};
                hz_half <= glyph_q[0];
                state   <= S_RQ;
            end

            // -------- 表头 → 通知 pix → 人声 --------
            S_HDR: begin
                hzk_phase <= 1'b0;
                line_we   <= 1'b1;
                line_addr <= {7'd0, 2'd3};
                line_data <= {3'd0, rows, voc_ok, !p_bad, line_n};
                state     <= S_HDR1;
            end
            S_HDR1: begin
                line_we <= 1'b1; line_addr <= {7'd0, 2'd0}; line_data <= {7'd0, gsum};
                state <= S_HDR2;
            end
            S_HDR2: begin
                own_ld  <= 1'b0;
                line_we <= 1'b1; line_addr <= {7'd1, 2'd0}; line_data <= {9'd0, s0};
                state <= S_VOCLOC;
            end
            S_VOCLOC: begin
                ready_t <= ~ready_t;
                if (voc_ok) begin
                    loc_req <= 1'b1; loc_idx <= {2'b10, sel_q[3:0]}; state <= S_VSTART;
                    voc_total_sectors <= 32'd0;
                end else
                    state <= S_RUN;
            end
            S_VSTART: begin
                if (loc_valid) begin
                    voc_start_lba     <= loc_lba;
                    voc_total_sectors <= {9'd0, loc_size[31:9]};
                end
                if (voc_total_sectors != 32'd0 && voc_wr_idle && !loc_valid) begin
                    voc_play_enable <= 1'b1;
                    epoch_t <= ~epoch_t;
                    state <= S_RUN;
                end
            end
            default: state <= S_INIT;
            endcase
        end
    end
endmodule
