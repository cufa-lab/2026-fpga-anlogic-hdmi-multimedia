`timescale 1ns/1ps

module bmp_smart_scaler #(
    parameter integer PACE       = 16,     // 每个输出像素的 sys 时钟数（≥ 8：背景低通分时用前 4 拍）
    parameter integer HOLD_LEVEL = 32,
    parameter integer REFINE_EN  = 0       // 1 = 支持导览局部重读（REF 模式）；0 = refine 输入忽略、REF 逻辑裁掉
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        load_start,
    input  wire        load_abort,
    input  wire        refine,             // 准静态：load_start 时采样（1 = 导览局部重读）
    input  wire [11:0] ref_c0,             // 局部重读窗口（原图坐标，行号自下而上）
    input  wire [11:0] ref_r0b,
    input  wire [11:0] ref_cw,             // = 4q（≥ 576）
    input  wire [11:0] ref_ch,             // = 3q
    input  wire        geom_valid,
    input  wire [11:0] src_w,
    input  wire [11:0] src_h,
    input  wire        in_valid,
    input  wire [23:0] in_rgb,
    output wire        in_hold,
    output reg         out_valid,
    output reg  [23:0] out_rgb,
    output reg         out_commit,
    output wire        busy,
    output reg         overflow,           // 粘滞到下一次 load_start（观测用）
    // ---- → sd_card_bmp（SMART=1）----
    output reg         map_valid,          // 第 2 遍窗口已定（电平，到下一次 load_start）
    output reg         map_direct,         // 1 = 窗口在 load_start 时已知（REF）
    output reg  [11:0] map_c0,
    output reg  [11:0] map_r0b,
    output reg  [11:0] map_cw,
    output reg  [11:0] map_ch,
    output reg  [11:0] map_n,
    output reg  [9:0]  map_d,
    // ---- → 导览运镜几何表（plan_valid 时有效，保持到下一次规划）----
    output reg         plan_valid,
    output wire [9:0]  g_ox,               // 主图在帧里的左上角（显示坐标，自上而下）
    output wire [8:0]  g_oyt,
    output wire [11:0] g_c0,               // 主图对应的原图窗口（左上角，自上而下）
    output wire [11:0] g_r0t,
    output wire [11:0] g_cw,
    output wire [11:0] g_ch,
    output wire [11:0] g_n,                // 比例：显示 = 原图 × D / N
    output wire [9:0]  g_d
);
    localparam [4:0] S_IDLE  = 5'd0,  S_GEOM  = 5'd1,  S_PL0   = 5'd2,  S_PL1   = 5'd3,
                     S_PL2   = 5'd4,  S_PL3   = 5'd5,  S_INT   = 5'd6,  S_SET1  = 5'd7,
                     S_DIV   = 5'd8,  S_SET2  = 5'd9,  S_RINIT = 5'd10, S_FETCH = 5'd11,
                     S_PIX   = 5'd12, S_RDONE = 5'd13, S_NEXT  = 5'd14, S_CHK   = 5'd15,
                     S_ADV   = 5'd16, S_POST  = 5'd17, S_EMIT  = 5'd18;
    localparam [1:0] M_FIT = 2'd0, M_FILL = 2'd1, M_INT = 2'd2, M_REF = 2'd3;

    reg [4:0]  st, emit_ret;
    reg        emit_black;
    reg        refine_q;
    wire       refine_e = (REFINE_EN != 0) && refine;
    wire [11:0] rf_c0  = (REFINE_EN != 0) ? ref_c0  : 12'd0;
    wire [11:0] rf_r0b = (REFINE_EN != 0) ? ref_r0b : 12'd0;
    wire [11:0] rf_cw  = (REFINE_EN != 0) ? ref_cw  : 12'd576;
    wire [11:0] rf_ch  = (REFINE_EN != 0) ? ref_ch  : 12'd432;
    reg [1:0]  mode;
    reg [11:0] ih;                       // 原图高（几何输出用）
    reg [11:0] sw, sh;                   // 构图时 = 原图尺寸；第 2 遍 = 窗口
    reg        wlim;
    reg [11:0] n_step;                   // N
    reg [9:0]  dd;                       // D（640 / 480 / 576 / k）
    reg signed [22:0] div_rem;
    reg [9:0]  div_cnt;
    reg [9:0]  dw;
    reg [8:0]  dh;
    reg [9:0]  ox;
    reg [8:0]  pre_rows;
    reg signed [13:0] xrem, yrem;
    reg [9:0]  dx;
    reg [11:0] cx, ry;
    reg [8:0]  dy;
    reg        row_used;
    reg [9:0]  col;
    reg [8:0]  out_rows;
    reg [4:0]  pc;
    reg        in_range_q;
    reg [23:0] pix;
    reg [9:0]  ow;                       // 输出帧宽 / 高（REF 576×432）
    reg [8:0]  oh;
    reg [10:0] q;
    reg [3:0]  kk;                       // INT 倍数（≤ 8）
    // ---- 柔化背景 ----
    reg        blur_en, blur_row;
    reg [23:0] vn;                       // 竖向低通后（本列）
    reg [23:0] hb;                       // 横向低通后

    // ---------------- 一个 DSP：寄存输出（q = floor(h/3)）----------------
    reg  signed [17:0] mul_a, mul_b;
    reg  signed [35:0] prod;
    always @(posedge clk) prod <= mul_a * mul_b;

    // ---------------- 输入 FIFO ----------------
    reg  [8:0]  wptr, rptr;
    wire [8:0]  fifo_cnt   = wptr - rptr;
    wire        fifo_empty = (fifo_cnt == 9'd0);
    wire        accept     = (st != S_IDLE) && !overflow;
    wire        fifo_re    = (st == S_FETCH) && !fifo_empty;
    wire [23:0] fifo_rdata;
    assign in_hold = (fifo_cnt >= HOLD_LEVEL);
    assign busy    = (st != S_IDLE);

    fat32_sdp_ram #(.AW(8), .DW(24)) u_in_fifo (
        .clk(clk), .we(in_valid && accept && fifo_cnt != 9'd256), .waddr(wptr[7:0]), .wdata(in_rgb),
        .re(fifo_re), .raddr(rptr[7:0]), .rdata(fifo_rdata));

    // ---------------- 行缓冲（读地址 = 列号夹到主图范围：边框读到边缘像素）----------------
    wire [9:0]  oxe      = ox + dw;
    wire [9:0]  col_c    = (col < ox) ? ox : (col >= oxe) ? oxe - 1'b1 : col;
    wire        lb_we    = (st == S_PIX) && (pc != 5'd0) && row_used && (xrem > 0) && (dx < dw);
    wire [9:0]  lb_waddr = ox + dx;
    wire [23:0] lb_rdata;
    fat32_sdp_ram #(.AW(10), .DW(24)) u_line (
        .clk(clk), .we(lb_we), .waddr(lb_waddr), .wdata(pix),
        .re(st == S_EMIT && pc == 5'd0), .raddr(col_c), .rdata(lb_rdata));

    // ---------------- 竖向低通状态 640×24（每输出行每列读改写一次）----------------
    wire [23:0] vm_q;
    fat32_sdp_ram #(.AW(10), .DW(24)) u_vlp (
        .clk(clk), .we(st == S_EMIT && pc == 5'd3 && blur_en), .waddr(col), .wdata(vn),
        .re(st == S_EMIT && pc == 5'd0), .raddr(col), .rdata(vm_q));

    // 一阶低通 a + round((b - a) / 16)：结果总在 [min(a,b), max(a,b)] 内，不溢出
    function [7:0] lp16;
        input [7:0] a, b;
        reg signed [9:0] t;
        begin
            t    = $signed({2'b00, b}) - $signed({2'b00, a}) + 10'sd8;
            lp16 = a + {{2{t[9]}}, t[9:4]};
        end
    endfunction
    function [23:0] lp16x3;
        input [23:0] a, b;
        begin lp16x3 = {lp16(a[23:16], b[23:16]), lp16(a[15:8], b[15:8]), lp16(a[7:0], b[7:0])}; end
    endfunction

    // ---------------- 组合小件 ----------------
    // 构图判定
    wire        int_ok  = ({1'b0, sw} <= 13'd320) && ({1'b0, sh} <= 13'd240);
    wire [15:0] sw9     = {sw, 3'd0} + sw;
    wire [15:0] sh16    = {sh, 4'd0};
    wire        fill_ok = (sh <= sw) && (sw9 <= sh16);
    wire [13:0] sw3     = {sw, 1'b0} + sw;
    wire        wide    = sw3 >= {sh, 2'b00};
    wire [11:0] cw_f    = {q[9:0], 2'b00};
    wire [11:0] ch_f    = {q, 1'b0} + q;
    // 压暗 ×0.625
    wire [7:0]  dk_r    = {1'b0, hb[23:17]} + {3'b000, hb[23:19]};
    wire [7:0]  dk_g    = {1'b0, hb[15:9]}  + {3'b000, hb[15:11]};
    wire [7:0]  dk_b    = {1'b0, hb[7:1]}   + {3'b000, hb[7:3]};

    assign g_ox  = ox;
    assign g_oyt = oh - dh - pre_rows;
    assign g_c0  = map_c0;
    assign g_r0t = ih - map_ch - map_r0b;
    assign g_cw  = map_cw;
    assign g_ch  = map_ch;
    assign g_n   = map_n;
    assign g_d   = map_d;

    // 发一行：返回状态 + 是否全边框
    task emit_row(input [4:0] ret, input black);
        begin
            emit_ret   <= ret;
            emit_black <= black;
            col        <= 10'd0;
            pc         <= 5'd0;
            blur_row   <= blur_en && (black || (dw != ow));
            st         <= (out_rows == oh) ? S_IDLE : S_EMIT;
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_IDLE; emit_ret <= S_IDLE; emit_black <= 1'b1;
            refine_q <= 1'b0; mode <= M_FIT;
            ih <= 12'd1; sw <= 12'd1; sh <= 12'd1; wlim <= 1'b1; n_step <= 12'd1; dd <= 10'd640;
            div_rem <= 23'sd0; div_cnt <= 10'd0;
            dw <= 10'd0; dh <= 9'd0; ox <= 10'd0; pre_rows <= 9'd0;
            xrem <= 14'sd0; yrem <= 14'sd0;
            dx <= 10'd0; cx <= 12'd0; ry <= 12'd0; dy <= 9'd0; row_used <= 1'b0;
            col <= 10'd0; out_rows <= 9'd0; pc <= 5'd0; in_range_q <= 1'b0; pix <= 24'd0;
            ow <= 10'd640; oh <= 9'd480; q <= 11'd0; kk <= 4'd1;
            wptr <= 9'd0; rptr <= 9'd0;
            out_valid <= 1'b0; out_rgb <= 24'd0; out_commit <= 1'b0; overflow <= 1'b0;
            map_valid <= 1'b0; map_direct <= 1'b0;
            map_c0 <= 12'd0; map_r0b <= 12'd0; map_cw <= 12'd1; map_ch <= 12'd1; map_n <= 12'd1; map_d <= 10'd640;
            plan_valid <= 1'b0;
            blur_en <= 1'b0; blur_row <= 1'b0; vn <= 24'd0; hb <= 24'd0;
            mul_a <= 18'sd0; mul_b <= 18'sd0;
        end else if (load_start || load_abort) begin
            st <= load_start ? S_GEOM : S_IDLE;
            wptr <= 9'd0; rptr <= 9'd0; out_rows <= 9'd0;
            pc <= 5'd0;
            out_valid <= 1'b0; out_commit <= 1'b0; plan_valid <= 1'b0;
            if (load_start) begin
                overflow   <= 1'b0;
                refine_q   <= refine_e;
                // REF：窗口由导览给出；普通加载判头后定构图
                map_valid  <= refine_e;
                map_direct <= refine_e;
                map_c0  <= rf_c0;  map_r0b <= rf_r0b;
                map_cw  <= rf_cw;  map_ch  <= rf_ch;
                map_n   <= rf_cw;  map_d   <= 10'd576;
            end
        end else begin
            out_valid  <= 1'b0;
            out_commit <= 1'b0;
            plan_valid <= 1'b0;
            if (in_valid && accept && fifo_cnt != 9'd256)
                wptr <= wptr + 1'b1;
            if (fifo_re)
                rptr <= rptr + 1'b1;

            case (st)
                S_IDLE: ;
                S_GEOM: if (geom_valid) begin
                    ih <= src_h;
                    sw <= refine_q ? rf_cw : src_w;
                    sh <= refine_q ? rf_ch : src_h;
                    st <= S_PL0;
                end
                // ================= 构图 =================
                S_PL0: begin
                    mode  <= refine_q ? M_REF : int_ok ? M_INT : fill_ok ? M_FILL : M_FIT;
                    wlim  <= wide;
                    mul_a <= {6'd0, sh};
                    mul_b <= 18'sd21846;                     // floor(h/3) = (h·21846) >> 16（h ≤ 4095 精确）
                    st    <= S_PL1;
                end
                S_PL1: st <= S_PL2;                          // 乘法寄存 1 拍
                S_PL2: begin
                    q  <= wlim ? prod[26:16] : sw[11:2];
                    st <= S_PL3;
                end
                S_PL3: begin
                    case (mode)
                        M_REF: if (REFINE_EN != 0) begin
                            n_step <= sw; dd <= 10'd576;
                            dw <= 10'd576; dh <= 9'd432; ow <= 10'd576; oh <= 9'd432;
                            st <= S_SET2;
                        end
                        M_FILL: begin
                            // 4q×3q 居中窗口（铺满 640×480）
                            map_cw  <= cw_f; map_ch <= ch_f;
                            n_step  <= cw_f; dd <= 10'd640;
                            dw <= 10'd640; dh <= 9'd480; ow <= 10'd640; oh <= 9'd480;
                            map_c0  <= (sw - cw_f) >> 1;
                            map_r0b <= (sh - ch_f) >> 1;
                            st <= S_SET2;
                        end
                        M_INT: begin
                            map_c0 <= 12'd0; map_r0b <= 12'd0; map_cw <= sw; map_ch <= sh;
                            n_step <= 12'd1; kk <= 4'd1;
                            dw <= sw[9:0]; dh <= sh[8:0]; ow <= 10'd640; oh <= 9'd480;
                            st <= S_INT;
                        end
                        default: begin
                            map_c0 <= 12'd0; map_r0b <= 12'd0; map_cw <= sw; map_ch <= sh;
                            ow <= 10'd640; oh <= 9'd480;
                            st <= S_SET1;
                        end
                    endcase
                end
                S_INT: begin
                    // k 倍：再加一份仍放得下且 k < 8 就继续
                    if (({1'b0, dw} + sw <= 13'd640) && ({1'b0, dh} + sh <= 13'd480) && (kk != 4'd8)) begin
                        dw <= dw + sw[9:0]; dh <= dh + sh[8:0]; kk <= kk + 1'b1;
                    end else begin
                        dd <= {6'd0, kk};
                        st <= S_SET2;
                    end
                end
                S_SET1: begin
                    // FIT（原逻辑）：另一轴 ceil(D·s_other / N)；D·s 用移位加：640s = 512s+128s，480s = 512s-32s
                    n_step     <= wlim ? sw : sh;
                    dd         <= wlim ? 10'd640 : 10'd480;
                    div_rem    <= wlim ? ({sh, 9'd0} + {sh, 7'd0}) : ({sw, 9'd0} - {sw, 5'd0});
                    div_cnt    <= 10'd0;
                    st         <= S_DIV;
                end
                S_DIV: begin
                    if (div_rem > 0 && div_cnt != (wlim ? 10'd480 : 10'd640)) begin
                        div_rem <= div_rem - $signed({11'd0, n_step});
                        div_cnt <= div_cnt + 1'b1;
                    end else begin
                        dw <= wlim ? 10'd640 : div_cnt;
                        dh <= wlim ? div_cnt[8:0] : 9'd480;
                        st <= S_SET2;
                    end
                end
                S_SET2: begin
                    ox       <= (ow - dw) >> 1;
                    pre_rows <= oh - dh - ((oh - dh) >> 1);
                    yrem     <= $signed({4'd0, dd});
                    dy       <= 9'd0;
                    ry       <= 12'd0;
                    if (mode != M_REF) begin
                        map_n <= n_step;
                        map_d <= dd;
                    end
                    blur_en  <= ((mode == M_FIT) || (mode == M_INT)) && ((dw != 10'd640) || (dh != 9'd480));
                    sw       <= map_cw;                      // 第 2 遍的流 = 窗口
                    sh       <= map_ch;
                    if (!refine_q) begin
                        map_valid  <= 1'b1;
                        plan_valid <= 1'b1;
                    end
                    st       <= S_RINIT;                     // 下方边框行等第一行主图进了行缓冲再发（S_RDONE）
                end
                // ================= 第 2 遍：输出（原 bmp_scale_letterbox）=================
                S_RINIT: begin
                    xrem     <= $signed({4'd0, dd});
                    dx       <= 10'd0;
                    cx       <= 12'd0;
                    row_used <= (yrem > 0) && (dy < dh);
                    st       <= !(yrem > 0) ? S_ADV : S_FETCH;   // 上游没送这一行（ROW_SKIP）
                end
                S_FETCH: if (!fifo_empty) st <= S_PIX;
                S_PIX: begin
                    if (pc == 5'd0) begin
                        pix <= fifo_rdata;
                        pc  <= 5'd1;
                    end else if (lb_we) begin
                        dx   <= dx + 1'b1;
                        xrem <= xrem - $signed({2'd0, n_step});
                    end else begin
                        pc     <= 5'd0;
                        xrem   <= xrem + $signed({4'd0, dd});
                        cx     <= cx + 1'b1;
                        st     <= (cx == sw - 1'b1) ? S_RDONE : S_FETCH;
                    end
                end
                S_RDONE: begin
                    // 第一行主图在行缓冲里：先发完下方边框（out_rows < pre_rows），再发主图行
                    if (row_used) begin
                        if (out_rows < pre_rows) emit_row(S_RDONE, 1'b1);
                        else emit_row(S_NEXT, 1'b0);
                    end else
                        st <= S_ADV;
                end
                S_NEXT: begin
                    dy   <= dy + 1'b1;
                    yrem <= yrem - $signed({2'd0, n_step});
                    st   <= S_CHK;
                end
                S_CHK: begin
                    if ((yrem > 0) && (dy < dh)) emit_row(S_NEXT, 1'b0);
                    else st <= S_ADV;
                end
                S_ADV: begin
                    yrem   <= yrem + $signed({4'd0, dd});
                    ry     <= ry + 1'b1;
                    st     <= (ry == sh - 1'b1) ? S_POST : S_RINIT;
                end
                S_POST: begin
                    if (out_rows != oh) emit_row(S_POST, 1'b1);
                    else st <= S_IDLE;
                end
                S_EMIT: begin
                    // pc0 读行缓冲（夹列）+ 竖向状态 → pc2 竖向低通（第一行直接取源）→ pc3 写回 + 横向低通
                    case (pc)
                        5'd0: in_range_q <= (col >= ox) && (col < oxe);
                        5'd2: vn <= (out_rows == 9'd0) ? lb_rdata : lp16x3(vm_q, lb_rdata);
                        5'd3: hb <= (col == 10'd0) ? vn : lp16x3(hb, vn);
                        default: ;
                    endcase
                    if (pc == PACE - 1) begin
                        pc         <= 5'd0;
                        out_valid  <= 1'b1;
                        out_rgb    <= (emit_black || !in_range_q) ? (blur_row ? {dk_r, dk_g, dk_b} : 24'd0) : lb_rdata;
                        out_commit <= (out_rows == oh - 1'b1) && (col == ow - 1'b1);
                        if (col == ow - 1'b1) begin
                            out_rows <= out_rows + 1'b1;
                            st       <= emit_ret;
                        end
                        col <= col + 1'b1;
                    end else
                        pc <= pc + 1'b1;
                end
                default: st <= S_IDLE;
            endcase
            if (in_valid && accept && fifo_cnt == 9'd256) begin
                overflow <= 1'b1;      // 放在 case 之后：停机优先于本拍的状态转移
                st       <= S_IDLE;
            end
        end
    end
endmodule
