`timescale 1ns/1ps

module seg7_panel #(
    parameter integer CLK_HZ             = 100_000_000,
    parameter integer SCAN_STEP_CYCLES   = 16384,        // 164us/位 @100MHz，8 位一轮 1.31ms
    parameter integer PAGE_PERIOD_CYCLES = 1000,         // tick 数：1s 轮播
    parameter integer HOLD_CYCLES        = 2000,         // tick 数：2s 强制页
    parameter integer DIGIT_REVERSE      = 0
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       tick,            // 公共 1ms 节拍（tb 接 1 = 按时钟周期计）
    input  wire       enable,
    input  wire [5:0] media_id,        // 当前显示图片 0..63
    input  wire       media_valid,     // 已有图片提交
    input  wire       auto_enabled,
    input  wire [7:0] brightness,      
    input  wire [7:0] contrast,        
    input  wire [7:0] volume,          // 0..255
    input  wire [8:0] period_tenths,   // 周期 0.1s 单位
    input  wire [7:0] error_code,
    input  wire       pic_event,
    input  wire       vol_event,
    input  wire       bri_event,
    input  wire       con_event,       
    input  wire       prd_event,
    input  wire       save_event,
    input  wire       err_event,
    output reg  [7:0] seg_n,           // {dp,g,f,e,d,c,b,a} 低有效
    output reg  [7:0] dig_n            // 位选低有效 one-hot
);
    localparam [2:0] PAGE_PIC  = 3'd0;   // 轮播页
    localparam [2:0] PAGE_VOL  = 3'd1;   // 轮播页
    localparam [2:0] PAGE_PRD  = 3'd2;   // 轮播页
    localparam [2:0] PAGE_BRI  = 3'd3;   // 事件页
    localparam [2:0] PAGE_SAVE = 3'd4;   // 事件页
    localparam [2:0] PAGE_ERR  = 3'd5;   // 事件页
    localparam [2:0] PAGE_CON  = 3'd6;   // 事件页

    localparam integer SCAN_BITS = (SCAN_STEP_CYCLES <= 2) ? 1 : $clog2(SCAN_STEP_CYCLES);

    localparam integer PTW = (PAGE_PERIOD_CYCLES <= 2) ? 1 : $clog2(PAGE_PERIOD_CYCLES);
    localparam integer HTW = (HOLD_CYCLES <= 2) ? 1 : $clog2(HOLD_CYCLES);
    reg [PTW-1:0] page_timer;
    reg [HTW-1:0] hold_timer;
    reg [2:0]  page;
    reg        holding;

    // ---------------- 第一步：页面选择 ----------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            page       <= PAGE_PIC;
            page_timer <= 0;
            hold_timer <= 0;
            holding    <= 1'b0;
        end else begin
            if (err_event) begin
                page <= PAGE_ERR;  holding <= 1'b1; hold_timer <= 0;
            end else if (save_event) begin
                page <= PAGE_SAVE; holding <= 1'b1; hold_timer <= 0;
            end else if (bri_event) begin
                page <= PAGE_BRI;  holding <= 1'b1; hold_timer <= 0;
            end else if (con_event) begin
                page <= PAGE_CON;  holding <= 1'b1; hold_timer <= 0;
            end else if (vol_event) begin
                page <= PAGE_VOL;  holding <= 1'b1; hold_timer <= 0;
            end else if (prd_event) begin
                page <= PAGE_PRD;  holding <= 1'b1; hold_timer <= 0;
            end else if (pic_event) begin
                page <= PAGE_PIC;  holding <= 1'b1; hold_timer <= 0;
            end else if (!tick) begin
                page <= page;
            end else if (holding) begin
                if (hold_timer >= HOLD_CYCLES - 1) begin
                    holding    <= 1'b0;
                    page_timer <= 0;
                    if (page > PAGE_PRD)
                        page <= PAGE_PIC;      // 事件页结束回到轮播起点
                end else begin
                    hold_timer <= hold_timer + 1'b1;
                end
            end else begin
                if (page_timer >= PAGE_PERIOD_CYCLES - 1) begin
                    page_timer <= 0;
                    page <= (page >= PAGE_PRD) ? PAGE_PIC : page + 3'd1;
                end else begin
                    page_timer <= page_timer + 1'b1;
                end
            end
        end
    end

    // ---------------- 第三步：BCD（double-dabble，9bit → 3 位）----------------
    function [11:0] bcd9;
        input [8:0] v;
        integer i;
        reg [3:0] n0, n1, n2;
        begin
            n0 = 4'd0; n1 = 4'd0; n2 = 4'd0;
            for (i = 8; i >= 0; i = i - 1) begin
                if (n0 >= 4'd5) n0 = n0 + 4'd3;
                if (n1 >= 4'd5) n1 = n1 + 4'd3;
                if (n2 >= 4'd5) n2 = n2 + 4'd3;
                n2 = {n2[2:0], n1[3]};
                n1 = {n1[2:0], n0[3]};
                n0 = {n0[2:0], v[i]};
            end
            bcd9 = {n2, n1, n0};
        end
    endfunction

    // L4：只有一个 BCD 转换，按页选输入（ERR / SAVE 页不用，随便给）
    reg  [8:0]  bcd_in;
    always @* begin
        case (page)
            PAGE_PIC: bcd_in = {3'b000, media_id} + 9'd1;
            PAGE_VOL: bcd_in = {1'b0, volume};
            PAGE_PRD: bcd_in = period_tenths;
            PAGE_CON: bcd_in = {1'b0, contrast};
            default:  bcd_in = {1'b0, brightness};
        endcase
    end
    wire [11:0] page_bcd = bcd9(bcd_in);
    wire [11:0] prd_bcd = page_bcd;          // {十位秒, 个位秒, 十分位}
    wire [11:0] bri_bcd = page_bcd;
    wire [11:0] vol_bcd = page_bcd;
    wire [11:0] pic_bcd = page_bcd;

    // ---------------- 第四步：字形（高有效，bit0=A..bit6=G）----------------
    localparam [4:0] F_BLANK = 5'h10;
    localparam [4:0] F_P     = 5'h11;
    localparam [4:0] F_I     = 5'h12;
    localparam [4:0] F_C     = 5'h13;
    localparam [4:0] F_A     = 5'h14;
    localparam [4:0] F_B     = 5'h15;   // 小写 b
    localparam [4:0] F_R     = 5'h16;   // 小写 r
    localparam [4:0] F_D     = 5'h17;   // 小写 d
    localparam [4:0] F_S     = 5'h18;
    localparam [4:0] F_U     = 5'h19;   // U，也用来表示 V
    localparam [4:0] F_E     = 5'h1a;
    localparam [4:0] F_DASH  = 5'h1b;
    localparam [4:0] F_L     = 5'h1c;
    localparam [4:0] F_O     = 5'h1d;   // 小写 o
    localparam [4:0] F_N     = 5'h1e;   // 小写 n

    function [6:0] seg_font;
        input [4:0] sym;
        begin
            case (sym)
                5'h00: seg_font = 7'h3f;  5'h01: seg_font = 7'h06;
                5'h02: seg_font = 7'h5b;  5'h03: seg_font = 7'h4f;
                5'h04: seg_font = 7'h66;  5'h05: seg_font = 7'h6d;
                5'h06: seg_font = 7'h7d;  5'h07: seg_font = 7'h07;
                5'h08: seg_font = 7'h7f;  5'h09: seg_font = 7'h6f;
                5'h0a: seg_font = 7'h77;  5'h0b: seg_font = 7'h7c;
                5'h0c: seg_font = 7'h39;  5'h0d: seg_font = 7'h5e;
                5'h0e: seg_font = 7'h79;  5'h0f: seg_font = 7'h71;
                F_P:    seg_font = 7'h73;
                F_I:    seg_font = 7'h06;
                F_C:    seg_font = 7'h39;
                F_A:    seg_font = 7'h77;
                F_B:    seg_font = 7'h7c;
                F_R:    seg_font = 7'h50;
                F_D:    seg_font = 7'h5e;
                F_S:    seg_font = 7'h6d;
                F_U:    seg_font = 7'h3e;
                F_E:    seg_font = 7'h79;
                F_DASH: seg_font = 7'h40;
                F_L:    seg_font = 7'h38;
                F_O:    seg_font = 7'h5c;
                F_N:    seg_font = 7'h54;
                default: seg_font = 7'h00;  // blank
            endcase
        end
    endfunction

    // 逻辑位 7 = 最左，0 = 最右
    reg [4:0] sym [0:7];
    reg [7:0] dp;
    integer k;
    always @* begin
        for (k = 0; k < 8; k = k + 1)
            sym[k] = F_BLANK;
        dp = 8'b0000_0000;
        case (page)
            PAGE_PIC: begin
                sym[4] = F_P; sym[3] = F_I; sym[2] = F_C;
                if (media_valid) begin
                    sym[1] = (pic_bcd[7:4] == 4'd0) ? F_BLANK : {1'b0,pic_bcd[7:4]};
                    sym[0] = {1'b0,pic_bcd[3:0]};
                end else begin
                    sym[0] = F_DASH;
                end
                dp[0]  = auto_enabled;
            end
            PAGE_VOL: begin
                sym[7] = F_U; sym[6] = 5'h00; sym[5] = F_L;   // 'O' 与 0 同字形
                sym[2] = {1'b0, vol_bcd[11:8]};
                sym[1] = {1'b0, vol_bcd[7:4]};
                sym[0] = {1'b0, vol_bcd[3:0]};
            end
            PAGE_PRD: begin
                sym[7] = F_P; sym[6] = F_R; sym[5] = F_D;
                // 不带单位 S：七段的 S 与 5 同字形，"2.5S" 会读成 "2.55"
                sym[2] = (prd_bcd[11:8] == 4'd0) ? F_BLANK : {1'b0, prd_bcd[11:8]};
                sym[1] = {1'b0, prd_bcd[7:4]};
                sym[0] = {1'b0, prd_bcd[3:0]};
                dp[1]  = 1'b1;
            end
            PAGE_BRI: begin
                sym[7] = F_B; sym[6] = F_R;
                sym[2] = (bri_bcd[11:8] == 4'd0) ? F_BLANK : {1'b0, bri_bcd[11:8]};
                sym[1] = (bri_bcd[11:8] == 4'd0 && bri_bcd[7:4] == 4'd0) ? F_BLANK : {1'b0, bri_bcd[7:4]};
                sym[0] = {1'b0, bri_bcd[3:0]};
            end
            PAGE_CON: begin
                sym[7] = F_C; sym[6] = F_O; sym[5] = F_N;
                sym[2] = (bri_bcd[11:8] == 4'd0) ? F_BLANK : {1'b0, bri_bcd[11:8]};
                sym[1] = (bri_bcd[11:8] == 4'd0 && bri_bcd[7:4] == 4'd0) ? F_BLANK : {1'b0, bri_bcd[7:4]};
                sym[0] = {1'b0, bri_bcd[3:0]};
            end
            PAGE_SAVE: begin
                sym[7] = F_S; sym[6] = F_A; sym[5] = F_U; sym[4] = F_E;
            end
            default: begin // PAGE_ERR
                sym[7] = F_E; sym[6] = F_R; sym[5] = F_R;
                sym[1] = {1'b0, error_code[7:4]};
                sym[0] = {1'b0, error_code[3:0]};
            end
        endcase
    end

    // ---------------- 第二步：动态扫描 ----------------
    reg [SCAN_BITS-1:0] scan_cnt;
    reg [2:0]           scan_idx;     // 逻辑位 0..7
    wire [2:0]          sel_bit = DIGIT_REVERSE ? (3'd7 - scan_idx) : scan_idx;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            scan_cnt <= {SCAN_BITS{1'b0}};
            scan_idx <= 3'd0;
            dig_n    <= 8'hff;
            seg_n    <= 8'hff;
        end else if (!enable) begin
            dig_n <= 8'hff;
            seg_n <= 8'hff;
        end else begin
            if (scan_cnt == SCAN_STEP_CYCLES - 1) begin
                scan_cnt <= {SCAN_BITS{1'b0}};
                scan_idx <= scan_idx + 3'd1;
            end else begin
                scan_cnt <= scan_cnt + 1'b1;
            end
            dig_n <= ~(8'b0000_0001 << sel_bit);
            seg_n <= ~{dp[scan_idx], seg_font(sym[scan_idx])};
        end
    end
endmodule
