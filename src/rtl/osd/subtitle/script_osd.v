`timescale 1ns/1ps

module script_osd #(
    parameter [17:0] LAST_HOLD = 18'd400,     // 最后一句的朗读时长
    parameter [9:0]  PAGE_TOP  = 10'd72,
    parameter [7:0]  HOLD      = 8'd240,      // 标题 / 导语显示帧数（与参数条 PARAM_HOLD_FRAMES 相同）
    parameter [9:0]  XF        = 10'd512,     // 跑马灯朗读点（源像素，屏幕 ×3/4 = 384）
    parameter [9:0]  XS        = 10'd16,      // 跑马灯左右留边
    parameter [7:0]  PANEL_A   = 8'd200,
    parameter [7:0]  PL_A0     = 8'd64,       // 暗-平
    parameter [7:0]  PL_A1     = 8'd160,      // 暗-花 / 亮-平
    parameter [7:0]  PL_A2     = 8'd184,      // 亮-花
    parameter [23:0] C_TITLE   = 24'hFFFFFF,
    parameter [23:0] C_INTRO   = 24'hC8DCEB,
    parameter [23:0] C_READ    = 24'hFFD54F,
    parameter [23:0] C_UNREAD  = 24'hF2F2F2,
    parameter [23:0] C_PAST    = 24'hA0A8B4,
    parameter [23:0] C_PLATE   = 24'h0A1424,
    parameter [23:0] C_PANEL   = 24'h0B1220,
    parameter [23:0] C_HILITE  = 24'h5A7AA0,
    parameter [23:0] C_GRAB    = 24'h8090A8,
    parameter [23:0] C_DIV     = 24'h3A4A64
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        frame_start,
    input  wire        de,
    input  wire [9:0]  pixel_x,
    input  wire [9:0]  pixel_y,
    input  wire [23:0] rgb_in,
    // sys 握手（toggle，内部 2FF）
    input  wire        rel_req_t,
    output reg         rel_ack_t,
    input  wire        ready_t,
    input  wire        epoch_t,
    // RAM 读口（1 拍延迟）
    output wire        line_re,
    output wire [8:0]  line_raddr,
    input  wire [17:0] line_rdata,
    output wire        code_re,
    output wire [8:0]  code_raddr,
    input  wire [9:0]  code_rdata,
    output reg         glyph_re,
    output reg  [12:0] glyph_raddr,
    input  wire [63:0] glyph_rdata,
    // 控制
    input  wire        voice_cnt0,            // 人声播放器 sample_counter[0]（pix 域）
    input  wire        tick48,                // 48k 采样节拍（单拍）
    input  wire        pause,                 // SW2
    input  wire        page_on,               // SW3
    input  wire        hide,                  // 总览 / 无画面
    input  wire        marq_hide,             // KEY3 长按切换：底部跑马灯淡出（标题 / 导语 / 全文页不受影响）
    input  wire        title_show,            // 单拍：切到新画面（与参数条 param_show 同一信号）
    input  wire [8:0]  bright_gain,           // 画面亮度 + 1（1..256）：底部跑马灯（底板 + 文字）随画面亮度缩放
    output wire        ext_hit,
    output wire [23:0] ext_color,
    output wire [7:0]  ext_alpha,
    output wire        txt_hit,
    output reg  [23:0] txt_color,
    output wire [7:0]  txt_alpha,
    output wire        page_active
);
    // ================= 跨域 =================
    (* async_reg = "true" *) reg req_sync1, req_sync2;
    (* async_reg = "true" *) reg rdy_sync1, rdy_sync2;
    (* async_reg = "true" *) reg epo_sync1, epo_sync2;
    reg req_seen, rdy_seen, epo_seen;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin
            req_sync1 <= 1'b0; req_sync2 <= 1'b0; rdy_sync1 <= 1'b0; rdy_sync2 <= 1'b0;
            epo_sync1 <= 1'b0; epo_sync2 <= 1'b0;
        end else begin
            req_sync1 <= rel_req_t; req_sync2 <= req_sync1;
            rdy_sync1 <= ready_t;   rdy_sync2 <= rdy_sync1;
            epo_sync1 <= epoch_t;   epo_sync2 <= epo_sync1;
        end

    // ================= 状态 =================
    reg        loaded, releasing, has_voice, ready_pend;
    reg [7:0]  line_end;
    reg [4:0]  rows_n;                        // 段落行数
    reg [10:0] gend;                          // 讲解流总宽（半宽）
    reg [8:0]  s0;                            // 讲解流起点 code 下标
    reg [8:0]  t_start, i_start;
    reg [6:0]  t_w, i_w, s_w;
    reg [10:0] gc;                            // 当前句起点 G
    reg [7:0]  cur;                           // 当前句条号（1 = 还没有）
    reg [17:0] t_cur, t_n, t_next;
    reg        nx_inf;
    reg [17:0] now;
    reg [8:0]  cdiv;
    reg        vc_q;
    reg [7:0]  ta, ia, sa, pa;                // 标题 / 导语 / 字幕 / 全文页 文字 alpha
    reg [7:0]  itim;                          // 标题 / 导语计时（帧，饱和）
    reg [17:0] dur;
    reg [17:0] acc;
    wire [18:0] acc_d = {1'b0, acc} - {1'b0, dur};          // 借位 = acc < dur
    reg [10:0] hx;                            // 本句卡拉OK 进度（2× 像素）
    reg        cs_evt;
    reg [17:0] xd;                            // 低通后的朗读位置（2× 像素，低 3 位小数）
    wire [14:0] xp = xd[17:3];
    reg signed [15:0] off;                    // 跑马灯滚动量：屏幕 x = 流位置 - off
    reg [8:0]  wi;                            // 找首字：wi 号字的流位置 = wg
    reg [10:0] wg;
    reg [10:0] u0;
    reg [8:0]  m_i0;                          // 跑马灯本帧参数：首字、首字的半、首块内起始列、起止 x、金白分界
    reg        m_h0;
    reg [3:0]  m_pu0;
    reg [9:0]  m_rx0, m_split;
    reg [10:0] m_rx1;
    reg [3:0]  rr;                            // 全文页朗读所在段落行
    reg [9:0]  page_y;
    reg [7:0]  ptop;                          // 段落滚动：首行
    reg [4:0]  psub;                          //            行内像素 0..23
    reg [4:0]  lvl_t_q, lvl_s_q;              // 底板 alpha / 8（渐变后；PL_A* 必须是 8 的倍数）
    wire [7:0] lvl_t_a = {lvl_t_q, 3'b000};
    wire [7:0] lvl_s_a = {lvl_s_q, 3'b000};
    reg [1:0]  lvl_t, lvl_s;                  // 带统计等级 {亮, 花}

    wire        page_vis = (page_y != 10'd480);
    // 全文页滑动（缓出）：离终点距离 d，每帧走 d/4 + 1，d <= 1 时到位（page_y 恒在 PAGE_TOP..480）
    wire        pg_open  = page_on && !hide;
    wire [9:0]  pg_dist  = pg_open ? page_y - PAGE_TOP : 10'd480 - page_y;
    wire [9:0]  pg_step  = {2'b00, pg_dist[9:2]} + 10'd1;
    assign page_active = page_vis && !hide;

    // ================= 讲解时钟 =================
    wire adv = has_voice ? (voice_cnt0 != vc_q) : (tick48 && !pause);
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin
            vc_q <= 1'b0; cdiv <= 9'd0; now <= 18'd0; cs_evt <= 1'b0; epo_seen <= 1'b0;
        end else begin
            vc_q   <= voice_cnt0;
            cs_evt <= 1'b0;
            if (epo_sync2 != epo_seen || (rdy_sync2 != rdy_seen)) begin
                epo_seen <= epo_sync2;
                cdiv <= 9'd0; now <= 18'd0;
            end else if (adv) begin                    // 不等 loaded：人声可能在读表头那一帧之前就开播
                if (cdiv == (has_voice ? 9'd239 : 9'd479)) begin
                    cdiv <= 9'd0; now <= now + 18'd1; cs_evt <= 1'b1;
                end else
                    cdiv <= cdiv + 9'd1;
            end
        end

    // ================= 时间轴（场消隐）=================
    localparam [4:0] TL_IDLE = 5'd0,  TL_EV = 5'd1,  TL_H1 = 5'd2,  TL_H2 = 5'd3,  TL_H3 = 5'd4,
                     TL_H4 = 5'd5,    TL_H5 = 5'd6,  TL_H6 = 5'd7,  TL_H7 = 5'd8,  TL_H8 = 5'd9,
                     TL_N = 5'd10,    TL_N1 = 5'd11, TL_WT = 5'd12, TL_N2 = 5'd13, TL_SW = 5'd14,
                     TL_S1 = 5'd15,   TL_S2 = 5'd16, TL_S3 = 5'd17, TL_S4 = 5'd18, TL_S5 = 5'd19,
                     TL_X0 = 5'd20,   TL_X1 = 5'd21, TL_X2 = 5'd22, TL_W0 = 5'd23, TL_W1 = 5'd24,
                     TL_W2 = 5'd25,   TL_R0 = 5'd26, TL_R1 = 5'd27, TL_R2 = 5'd28, TL_FR = 5'd29;
    reg [4:0]  tl;
    reg        tl_done;                       // 单拍：时间轴结束，给第 0 行做行准备
    reg        tl_re;                         // line 口：时间轴（场消隐）与行准备（行消隐）时间上不重叠
    reg [8:0]  tl_addr;
    reg        tl_cre;                        // code 口：找首字（场消隐）与格引擎（有效行）不重叠
    reg [8:0]  tl_caddr;

    wire [7:0]  n_idx    = cur + 8'd1;
    wire        n_ok     = (n_idx < line_end);
    wire        due      = n_ok && (t_n <= now);
    wire [17:0] gap      = t_next - t_cur;
    wire [17:0] since    = now - t_cur;
    wire        ended    = (cur >= 8'd2) && !n_ok && (s_w == 7'd0 || since >= LAST_HOLD);
    wire        all_zero = (ta == 8'd0) && (ia == 8'd0) && (sa == 8'd0) && (pa == 8'd0);

    function [7:0] up8;   input [7:0] v; input [7:0] st; up8 = (v > 8'd255 - st) ? 8'd255 : v + st; endfunction
    function [7:0] dn8;   input [7:0] v; input [7:0] st; dn8 = (v < st) ? 8'd0 : v - st; endfunction

    // 低通：Xd += (X - Xd) / 8（带 3 位小数，滚动速度连续；差不足 1/8 像素时每帧走 1/8 像素）
    wire [14:0] x2       = {gc, 4'b0000} + {4'd0, hx};
    wire [18:0] xdif     = {1'b0, x2, 3'b000} - {1'b0, xd};
    wire [18:0] xstep    = (xdif[18:3] == 16'd0 || xdif[18:3] == 16'hFFFF) ?
                           ((xdif == 19'd0) ? 19'd0 : {{18{xdif[18]}}, 1'b1}) : {{3{xdif[18]}}, xdif[18:3]};
    // 滚动量 off = max(-XS, min(Xd - XF, 16*Gend - (SW_SRC - XS)))；SW_SRC = 640 ×4/3（源坐标下的屏宽）
    localparam [15:0] SW_SRC = 16'd853;
    wire signed [15:0] o_a    = $signed({1'b0, xp}) - $signed({6'd0, XF});
    wire signed [15:0] o_max  = $signed({1'b0, gend, 4'b0000}) - $signed(SW_SRC - {6'd0, XS});
    wire signed [15:0] o_min  = (o_a < o_max) ? o_a : o_max;
    wire        [15:0] scr_end = {1'b0, gend, 4'b0000} - off;              // 流末尾（源坐标，相对屏幕左边）
    // 源 → 屏幕 ×3/4（v - v/4）：起点（off < 0，-off <= XS）、终点（> SW_SRC 时钳到 640）、金白分界
    wire        [4:0]  nof     = 5'd0 - off[4:0];
    wire        [9:0]  sp_src  = xp[9:0] - off[9:0];
    wire               se_big  = (scr_end[15:10] != 6'd0) || (scr_end[9:0] > SW_SRC[9:0]);
    // 全文页：自动滚动目标 = clamp(rr - 4, 0, rows - 14)
    wire [4:0] pt_max   = (rows_n > 5'd14) ? rows_n - 5'd14 : 5'd0;
    wire [4:0] rr_m4    = (rr > 4'd4) ? {1'b0, rr} - 5'd4 : 5'd0;
    wire [7:0] ptgt     = {3'd0, (rr_m4 < pt_max) ? rr_m4 : pt_max};
    wire [8:0] pdiff    = {1'b0, ptgt} - {1'b0, ptop};
    wire       p_jump   = pdiff[8] ? (pdiff[7:0] <= 8'hFC) : (pdiff[7:2] != 6'd0);   // 差 >= 4 / <= -4
    wire [4:0] rr_n     = {1'b0, rr} + 5'd1;

    // 带统计等级 → 目标 alpha
    function [4:0] lvl_alpha; input [1:0] l;
        lvl_alpha = (l == 2'b11) ? PL_A2[7:3] : (l == 2'b00) ? PL_A0[7:3] : PL_A1[7:3];
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tl <= TL_IDLE; tl_done <= 1'b0;
            loaded <= 1'b0; releasing <= 1'b0; has_voice <= 1'b0; ready_pend <= 1'b0;
            req_seen <= 1'b0; rdy_seen <= 1'b0; rel_ack_t <= 1'b0;
            line_end <= 8'd2; rows_n <= 5'd0; gend <= 11'd0; s0 <= 9'd0;
            t_start <= 9'd0; i_start <= 9'd0; t_w <= 7'd0; i_w <= 7'd0; s_w <= 7'd0; gc <= 11'd0; cur <= 8'd1;
            t_cur <= 18'd0; t_n <= 18'd0; t_next <= 18'd0; nx_inf <= 1'b1;
            ta <= 8'd0; ia <= 8'd0; sa <= 8'd0; pa <= 8'd0; itim <= 8'd0;
            dur <= 18'd1; acc <= 18'd0; hx <= 11'd0; xd <= 18'd0; off <= 16'd0;
            wi <= 9'd0; wg <= 11'd0; u0 <= 11'd0;
            m_i0 <= 9'd0; m_h0 <= 1'b0; m_pu0 <= 4'd0; m_rx0 <= 10'd0; m_rx1 <= 11'd0; m_split <= 10'd0;
            rr <= 4'd0; page_y <= 10'd480; ptop <= 8'd0; psub <= 5'd0;
            lvl_t_q <= PL_A1[7:3]; lvl_s_q <= PL_A1[7:3];
            tl_re <= 1'b0; tl_addr <= 9'd0; tl_cre <= 1'b0; tl_caddr <= 9'd0;
        end else begin
            tl_done <= 1'b0;
            tl_re   <= 1'b0;
            tl_cre  <= 1'b0;

            // 卡拉OK DDA：每 1/100 s 累加一次宽度（2× 像素），随后每拍最多减一次时长
            if (cs_evt)
                acc <= acc + {7'd0, s_w, 4'b0000};
            else if (!acc_d[18] && hx < {s_w, 4'b0000}) begin
                acc <= acc_d[17:0];
                hx  <= hx + 11'd1;
            end

            if (rdy_sync2 != rdy_seen) begin rdy_seen <= rdy_sync2; ready_pend <= 1'b1; end
            if (req_sync2 != req_seen) begin req_seen <= req_sync2; releasing <= 1'b1; ready_pend <= 1'b0; end

            case (tl)
            TL_IDLE: if (frame_start) tl <= TL_EV;
            TL_EV: begin
                if (ready_pend && !releasing) begin
                    ready_pend <= 1'b0;
                    tl_re <= 1'b1; tl_addr <= {7'd0, 2'd3};
                    tl <= TL_H1;
                end else if (loaded && !releasing)
                    tl <= TL_N;
                else
                    tl <= TL_FR;
            end
            // ---- 表头 + 标题 + 导语（读口 1 拍延迟：本状态发地址，隔一个状态拿数据）----
            TL_H1: begin tl_re <= 1'b1; tl_addr <= {7'd0, 2'd0}; tl <= TL_H2; end
            TL_H2: begin
                loaded    <= line_rdata[8];
                has_voice <= line_rdata[9];
                rows_n    <= line_rdata[14:10];
                line_end  <= (line_rdata[7:0] < 8'd2) ? 8'd2 : line_rdata[7:0];
                tl_re <= 1'b1; tl_addr <= {7'd1, 2'd0}; tl <= TL_H3;
            end
            TL_H3: begin gend <= line_rdata[10:0]; tl_re <= 1'b1; tl_addr <= {7'd0, 2'd1}; tl <= TL_H4; end
            TL_H4: begin s0 <= line_rdata[8:0];    tl_re <= 1'b1; tl_addr <= {7'd0, 2'd2}; tl <= TL_H5; end
            TL_H5: begin t_start <= line_rdata[8:0]; tl_re <= 1'b1; tl_addr <= {7'd1, 2'd1}; tl <= TL_H6; end
            TL_H6: begin t_w <= line_rdata[6:0];     tl_re <= 1'b1; tl_addr <= {7'd1, 2'd2}; tl <= TL_H7; end
            TL_H7: begin i_start <= line_rdata[8:0]; tl <= TL_H8; end
            TL_H8: begin
                i_w <= line_rdata[6:0];
                cur <= 8'd1; t_cur <= 18'd0; s_w <= 7'd0; gc <= 11'd0; hx <= 11'd0; acc <= 18'd0;
                xd <= 18'd0; wi <= s0; wg <= 11'd0; rr <= 4'd0;
                sa <= 8'd0; ta <= 8'd0; ia <= 8'd0; itim <= 8'd0; ptop <= 8'd0; psub <= 5'd0;
                tl <= TL_N;
            end
            // ---- 每帧：回退 / 前进判断 ----
            TL_N: begin
                if (cur >= 8'd2 && now < t_cur) begin             // epoch 归零（人声重播）→ 从头
                    cur <= 8'd1; t_cur <= 18'd0; s_w <= 7'd0; gc <= 11'd0; hx <= 11'd0; acc <= 18'd0;
                    xd <= 18'd0; wi <= s0; wg <= 11'd0; rr <= 4'd0;
                    tl <= TL_N;
                end else if (n_ok) begin
                    tl_re <= 1'b1; tl_addr <= {n_idx[6:0], 2'd0};
                    tl <= TL_N1;
                end else begin
                    t_n <= 18'h3FFFF;
                    tl  <= TL_N2;
                end
            end
            TL_N1: tl <= TL_WT;
            TL_WT: begin t_n <= line_rdata; tl <= TL_N2; end
            TL_N2: tl <= due ? TL_SW : TL_X0;
            // ---- 换句（可连换多句追上 now）----
            TL_SW: begin
                cur <= n_idx; t_cur <= t_n;
                tl_re <= 1'b1; tl_addr <= {n_idx[6:0], 2'd1};
                tl <= TL_S1;
            end
            TL_S1: begin tl_re <= 1'b1; tl_addr <= {cur[6:0], 2'd2}; tl <= TL_S2; end
            TL_S2: begin
                gc <= line_rdata[10:0];
                if (n_ok) begin tl_re <= 1'b1; tl_addr <= {n_idx[6:0], 2'd0}; end   // n_idx 已是下一句
                tl <= TL_S3;
            end
            TL_S3: begin s_w <= line_rdata[6:0]; tl <= TL_S4; end
            TL_S4: begin
                nx_inf <= !n_ok;
                t_next <= line_rdata;
                t_n    <= n_ok ? line_rdata : 18'h3FFFF;
                tl     <= TL_S5;
            end
            TL_S5: begin
                if (due) tl <= TL_SW;                              // 下一句也已到点：直接再换
                else begin
                    hx <= 11'd0; acc <= 18'd0;
                    dur <= nx_inf ? LAST_HOLD : (gap != 18'd0) ? gap : 18'd1;
                    tl <= TL_X0;
                end
            end
            // ---- 跑马灯：低通 → 滚动量 → 本帧参数 ----
            TL_X0: begin xd <= xd + xstep[17:0]; tl <= TL_X1; end
            TL_X1: begin off <= (o_min < -$signed({6'd0, XS})) ? -$signed({6'd0, XS}) : o_min; tl <= TL_X2; end
            TL_X2: begin
                u0      <= off[15] ? 11'd0 : off[14:4];
                m_pu0   <= off[15] ? 4'd0 : {off[3:2], off[1], ~off[1] & off[0]};   // 源列 → {组, 相位 0..2}
                m_rx0   <= off[15] ? {5'd0, nof} - {7'd0, nof[4:2]} : 10'd0;
                m_rx1   <= se_big ? 11'd640 : {1'b0, scr_end[9:0] - {2'd0, scr_end[9:2]}};
                m_split <= sp_src - {2'd0, sp_src[9:2]};
                tl <= TL_W0;
            end
            // ---- 找首字：wi 从上次的位置往后走到覆盖 u0 的字（每帧通常 0~1 步）----
            TL_W0: begin
                if (wg >= u0) begin m_i0 <= wi; m_h0 <= 1'b0; tl <= TL_R0; end
                else begin tl_cre <= 1'b1; tl_caddr <= wi; tl <= TL_W1; end
            end
            TL_W1: tl <= TL_W2;
            TL_W2: begin
                if (code_rdata[9] && wg + 11'd1 == u0) begin m_i0 <= wi; m_h0 <= 1'b1; tl <= TL_R0; end
                else begin
                    wg <= wg + (code_rdata[9] ? 11'd2 : 11'd1);
                    wi <= wi + 9'd1;
                    tl <= TL_W0;
                end
            end
            // ---- 全文页：朗读所在段落行 rr 往后走 ----
            TL_R0: begin
                if (rr_n < rows_n) begin tl_re <= 1'b1; tl_addr <= {3'b111, rr_n[3:0], 2'd1}; tl <= TL_R1; end
                else tl <= TL_FR;
            end
            TL_R1: tl <= TL_R2;
            TL_R2: begin
                if ({line_rdata[10:0], 4'b0000} <= xp) begin rr <= rr_n[3:0]; tl <= TL_R0; end
                else tl <= TL_FR;
            end
            // ---- 每帧动画 ----
            TL_FR: begin
                tl <= TL_IDLE; tl_done <= 1'b1;
                if (releasing) begin
                    ta <= dn8(ta, 8'd32); ia <= dn8(ia, 8'd32); sa <= dn8(sa, 8'd32); pa <= dn8(pa, 8'd32);
                    if (all_zero) begin
                        releasing <= 1'b0; loaded <= 1'b0; rel_ack_t <= ~rel_ack_t;
                    end
                end else if (loaded) begin
                    if (itim != 8'd255) itim <= itim + 8'd1;
                    if (itim < HOLD && t_w != 7'd0)
                        ta <= (ta == 8'd255) ? 8'd255 : ta + ({1'b0, ~ta[7:1]} >> 2) + 8'd1;   // 缓出
                    else
                        ta <= dn8(ta, 8'd16);
                    ia <= (itim >= 8'd24 && itim < HOLD && i_w != 7'd0) ? up8(ia, 8'd16) : dn8(ia, 8'd16);
                    sa <= (gend != 11'd0 && !ended && !marq_hide) ? up8(sa, 8'd16) : dn8(sa, 8'd16);
                    pa <= up8(pa, 8'd32);
                end
                // 全文页滑动（缓出，开 / 合对称）
                if (pg_dist <= 10'd1) page_y <= pg_open ? PAGE_TOP : 10'd480;
                else                  page_y <= pg_open ? page_y - pg_step : page_y + pg_step;
                // 段落滚动：差 3 行以上直接跳，否则每帧 2 像素
                if (!page_vis || p_jump) begin
                    ptop <= ptgt; psub <= 5'd0;
                end else if (!pdiff[8] && pdiff != 9'd0) begin
                    if (psub >= 5'd22) begin psub <= psub - 5'd22; ptop <= ptop + 8'd1; end
                    else psub <= psub + 5'd2;
                end else if (pdiff[8] || psub != 5'd0) begin
                    if (psub == 5'd0) begin psub <= 5'd22; ptop <= ptop - 8'd1; end
                    else psub <= (psub == 5'd1) ? 5'd0 : psub - 5'd2;
                end
                // 底板等级渐变
                lvl_t_q <= (lvl_t_q < lvl_alpha(lvl_t)) ? lvl_t_q + 5'd1 : (lvl_t_q > lvl_alpha(lvl_t)) ? lvl_t_q - 5'd1 : lvl_t_q;
                lvl_s_q <= (lvl_s_q < lvl_alpha(lvl_s)) ? lvl_s_q + 5'd1 : (lvl_s_q > lvl_alpha(lvl_s)) ? lvl_s_q - 5'd1 : lvl_s_q;
            end
            default: tl <= TL_IDLE;
            endcase
            if (title_show) itim <= 8'd0;                          // 切到新画面：标题 / 导语重新显示
        end
    end

    // ================= 带统计（rgb_in，标题带 4..47 / 字幕带 446..477）=================
    reg [19:0] sum_y, sum_d;
    reg [7:0]  y_prev;
    wire [9:0] y_sum10 = {2'd0, rgb_in[23:16]} + {1'b0, rgb_in[15:8], 1'b0} + {2'd0, rgb_in[7:0]};
    wire [7:0] y_cur   = y_sum10[9:2];
    wire [7:0] y_dif   = (y_cur > y_prev) ? y_cur - y_prev : y_prev - y_cur;
    wire       in_tb   = (pixel_y >= 10'd4)   && (pixel_y < 10'd48);
    wire       in_sb   = (pixel_y >= 10'd446) && (pixel_y < 10'd478);
    // P4（第二批）：没装讲稿或画面隐藏时停止统计（累加器 / 差分不翻转；重新显示后下一帧即恢复，底板 alpha 本来就逐帧渐变）
    wire       st_en   = loaded && !hide;
    wire       samp    = st_en && de && (pixel_x[1:0] == 2'b00) && !pixel_y[0] && (in_tb || in_sb);
    localparam [19:0] N_T = 20'd3520, N_S = 20'd2560;     // 22 × 160 / 16 × 160
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin
            sum_y <= 20'd0; sum_d <= 20'd0; y_prev <= 8'd0; lvl_t <= 2'b01; lvl_s <= 2'b01;
        end else begin
            if (de && pixel_x == 10'd0 && (pixel_y == 10'd4 || pixel_y == 10'd446)) begin
                sum_y <= 20'd0; sum_d <= 20'd0; y_prev <= y_cur;
            end else if (samp) begin
                sum_y <= sum_y + {12'd0, y_cur};
                sum_d <= sum_d + {12'd0, y_dif};
                y_prev <= y_cur;
            end
            if (de && pixel_x == 10'd0 && pixel_y == 10'd48)
                lvl_t <= {sum_y >= N_T * 20'd112, sum_d >= N_T * 20'd12};
            if (frame_start)
                lvl_s <= {sum_y >= N_S * 20'd112, sum_d >= N_S * 20'd12};
        end

    // ================= 行准备（行消隐）+ 格引擎 =================
    localparam [1:0] M_TITLE = 2'd0, M_INTRO = 2'd1, M_KARA = 2'd2;
    localparam [3:0] HS_IDLE = 4'd0, HS_0 = 4'd1, HS_1 = 4'd2, HS_2 = 4'd3, HS_3 = 4'd4, HS_4 = 4'd5,
                     HS_5 = 4'd6, HS_A = 4'd7, HS_B = 4'd8, HS_C = 4'd9;
    reg        de_q, de_fall;
    reg [3:0]  hs;
    reg        hs_re;
    reg [8:0]  hs_addr;
    reg        run_on, run_s, rmarq;
    reg        rboth;                         // 跑马灯 3/4：本行 = 两源行平均（r2 + r3）
    reg [9:0]  rx0;
    reg [10:0] rx1;
    reg [9:0]  rs1, rs2;                      // 颜色分界：x < rs1 已读句灰，x < rs2 金，其余白
    reg signed [14:0] rbase;                  // 全文页本行：屏幕 x = 8*G + rbase（1× 像素）
    reg [3:0]  rrow;
    reg        rpar;
    reg [1:0]  rmode;
    reg [7:0]  ralpha;
    reg [6:0]  rw;
    reg [3:0]  rpu0;
    reg        pl_on;
    reg [9:0]  plx0;
    reg [10:0] plx1;
    reg [7:0]  pl_a;
    reg [1:0]  pl_k;                          // 底板颜色编号：0 底板 1 面板 2 高光 3 黑（压暗）
    reg        pk_on;                         // 标题 / 导语底板：x 范围随文字宽度（HS_B 算）
    reg [1:0]  pk_in;                         // 切角内缩
    reg        dc_on;
    reg [9:0]  dcx0, dcx1;
    reg [1:0]  dc_k;                          // 装饰颜色编号：0 拖动条 1 分隔线 2 已读金
    reg [4:0]  lsub;                          //             行内第几像素 0..23
    reg [7:0]  lent_q;                        // 全文页：本行的段落行号（ptop + 行）
    reg [8:0]  fa;                            // 预取指针：code 下标 + 半
    reg        fh;
    reg        fh_p;
    reg [3:0]  pu;                            // 字块内第几像素
    reg [63:0] word, nword;
    reg [4:0]  fp;                            // 预取流水
    reg        ic_row, ic_bk;                 // 本行在图标 / 图标底框行
    reg [3:0]  ic_r;
    reg        eng_cre;
    reg [8:0]  eng_caddr;

    assign line_re    = tl_re | hs_re;
    assign line_raddr = hs_re ? hs_addr : tl_addr;
    assign code_re    = tl_cre | eng_cre;
    assign code_raddr = tl_cre ? tl_caddr : eng_caddr;

    // ---- 本行几何（HS_0 用，pixel_y 已是新行）----
    wire [9:0] yy      = pixel_y;
    wire [9:0] rel     = yy - page_y;
    wire       pg_line = page_vis && !hide && (yy >= page_y);
    wire       t_band  = (yy >= 10'd4)   && (yy < 10'd48);
    wire       i_band  = (yy >= 10'd50)  && (yy < 10'd74);
    wire       s_band  = (yy >= 10'd446) && (yy < 10'd478);
    // 主画面三个带 y 范围互不重叠，每行只会画其中一个：共用一个 dy 减法器和一套字号选择
    wire [9:0] b_dy    = yy - (t_band ? 10'd10 : i_band ? 10'd54 : 10'd450);
    wire       b_on    = i_band ? (b_dy < 10'd16) : s_band ? (b_dy < 10'd24) : (b_dy < 10'd32);
    // 跑马灯行映射：屏幕行 dy（0..23）= 3g + ph → 源行对 2g + (ph == 2)，ph 0 / 1 取上 / 下源行，ph 2 两行平均
    reg  [4:0] s_rm;                          // {g[2:0], ph[1:0]}
    always @* begin
        case (b_dy[4:0])
            5'd0:  s_rm = 5'b000_00; 5'd1:  s_rm = 5'b000_01; 5'd2:  s_rm = 5'b000_10;
            5'd3:  s_rm = 5'b001_00; 5'd4:  s_rm = 5'b001_01; 5'd5:  s_rm = 5'b001_10;
            5'd6:  s_rm = 5'b010_00; 5'd7:  s_rm = 5'b010_01; 5'd8:  s_rm = 5'b010_10;
            5'd9:  s_rm = 5'b011_00; 5'd10: s_rm = 5'b011_01; 5'd11: s_rm = 5'b011_10;
            5'd12: s_rm = 5'b100_00; 5'd13: s_rm = 5'b100_01; 5'd14: s_rm = 5'b100_10;
            5'd15: s_rm = 5'b101_00; 5'd16: s_rm = 5'b101_01; 5'd17: s_rm = 5'b101_10;
            5'd18: s_rm = 5'b110_00; 5'd19: s_rm = 5'b110_01; 5'd20: s_rm = 5'b110_10;
            5'd21: s_rm = 5'b111_00; 5'd22: s_rm = 5'b111_01; default: s_rm = 5'b111_10;
        endcase
    end
    wire [7:0] b_a     = t_band ? ta : ia;
    wire [7:0] b_pl    = (lvl_t_a < b_a) ? lvl_t_a : b_a;
    wire [7:0] s_pl    = (lvl_s_a < sa) ? lvl_s_a : sa;
    wire [9:0] ptl_dy  = rel - 10'd12;
    wire [9:0] t_x0    = 10'd16 + {5'd0, ~ta[7:5], 2'b00};       // 标题从右向左滑入 28 像素
    wire [9:0] veil_a  = (10'd480 - page_y) >> 2;
    wire       main_ok = !page_vis && !hide && loaded;
    wire       pg_ok   = loaded && pa != 8'd0;
    wire [4:0] lsub_n  = (rel == 10'd56) ? psub : (lsub == 5'd23) ? 5'd0 : lsub + 5'd1;
    wire [7:0] lent    = (rel == 10'd56) ? ptop : (lsub == 5'd23) ? lent_q + 8'd1 : lent_q;   // 本行段落行号
    wire       in_list = pg_line && rel >= 10'd56;
    // 底板切角：带的首末行内缩 2、次首次末行内缩 1
    wire       ck_2    = t_band ? (yy == 10'd4  || yy == 10'd47) : (yy == 10'd50 || yy == 10'd73);
    wire       ck_1    = t_band ? (yy == 10'd5  || yy == 10'd46) : (yy == 10'd51 || yy == 10'd72);
    wire [1:0] chamf   = ck_2 ? 2'd2 : ck_1 ? 2'd1 : 2'd0;
    // 全文页本行颜色分界（1× 像素）：rbase + 8*Gc（本句起点）/ rbase + Xd/2（朗读点）
    function [9:0] clampx; input signed [14:0] v;
        clampx = v[14] ? 10'd0 : (v[13:10] != 4'd0) ? 10'd1023 : v[9:0];
    endfunction
    wire signed [14:0] pg_s1 = rbase + $signed({1'b0, gc, 3'b000});
    wire signed [14:0] pg_s2 = rbase + $signed({1'b0, xp[14:1]});

    // ---- 格引擎 ----
    wire       in_run    = run_on && de && (pixel_x >= rx0) && ({1'b0, pixel_x} < rx1);
    wire       pu_skip   = rmarq && pu[1];                                  // 3/4：相位 2 之后跳到下一组
    wire       unit_end  = run_s ? (pu[3:1] == 3'b111 && (rmarq || pu[0])) : (pu[2:0] == 3'd7);
    // 预取：发 code 地址 → 2 拍后拿 code、发 glyph 地址 → 2 拍后拿字形
    wire       fetch_go  = (hs == HS_A && run_on) || (hs == HS_C && fp[4]) || (in_run && unit_end);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            de_q <= 1'b0; de_fall <= 1'b0; hs <= HS_IDLE; hs_re <= 1'b0; hs_addr <= 9'd0;
            run_on <= 1'b0; run_s <= 1'b0; rmarq <= 1'b0; rboth <= 1'b0; rx0 <= 10'd0; rx1 <= 11'd0;
            rs1 <= 10'd0; rs2 <= 10'd0; rbase <= 15'sd0; rrow <= 4'd0; rpar <= 1'b0;
            rmode <= M_TITLE; ralpha <= 8'd0; rw <= 7'd0; rpu0 <= 4'd0;
            pl_on <= 1'b0; plx0 <= 10'd0; plx1 <= 11'd0; pl_a <= 8'd0; pl_k <= 2'd3; pk_on <= 1'b0; pk_in <= 2'd0;
            dc_on <= 1'b0; dcx0 <= 10'd0; dcx1 <= 10'd0; dc_k <= 2'd0;
            lsub <= 5'd0; lent_q <= 8'd0;
            fa <= 9'd0; fh <= 1'b0; fh_p <= 1'b0; pu <= 4'd0; word <= 64'd0; nword <= 64'd0; fp <= 5'd0;
            ic_row <= 1'b0; ic_bk <= 1'b0; ic_r <= 4'd0;
            eng_cre <= 1'b0; eng_caddr <= 9'd0; glyph_re <= 1'b0; glyph_raddr <= 13'd0;
        end else begin
            de_q     <= de;
            de_fall  <= de_q && !de;                // 下一拍 pixel_y 已是新行
            hs_re    <= 1'b0;
            eng_cre  <= 1'b0;
            glyph_re <= 1'b0;

            // ---- 预取流水：取完 code 就把指针移到下一块（宽字先走第 1 半）----
            fp <= {fp[3:0], fetch_go};
            if (fetch_go) begin
                eng_cre   <= 1'b1;
                eng_caddr <= fa;
                fh_p      <= fh;
            end
            if (fp[1]) begin
                glyph_re    <= 1'b1;
                glyph_raddr <= {code_rdata[7:0], rrow, fh_p};
                if (code_rdata[9] && !fh_p) fh <= 1'b1;
                else begin fa <= fa + 9'd1; fh <= 1'b0; end
            end
            if (fp[3])
                nword <= glyph_rdata;

            // ---- 格引擎 ----
            if (in_run) begin
                if (unit_end) begin
                    pu   <= 4'd0;
                    word <= nword;
                end else
                    pu <= pu + {2'b00, pu_skip, ~pu_skip};
            end

            // ---- 行准备 ----
            case (hs)
            HS_IDLE: if (de_fall || tl_done) hs <= HS_0;
            HS_0: begin
                run_on <= 1'b0; pl_on <= 1'b0; pk_on <= 1'b0; dc_on <= 1'b0; rmarq <= 1'b0; rboth <= 1'b0;
                rpu0 <= 4'd0; rs1 <= 10'd0; rs2 <= 10'd0;
                lsub <= lsub_n; lent_q <= lent;
                ic_row <= !hide && loaded && (yy >= 10'd12) && (yy < 10'd28);
                ic_bk  <= !hide && loaded && (yy >= 10'd10) && (yy < 10'd30);
                ic_r   <= yy[3:0] - 4'd12;
                pk_in  <= chamf;
                hs <= HS_A;
                if (pg_line) begin
                    // 全文页：底板（顶边 1 像素高光）+ 拖动条 / 分隔线 / 朗读行标记
                    pl_on <= 1'b1; plx0 <= 10'd0; plx1 <= 11'd640;
                    pl_a  <= (rel == 10'd0) ? 8'd255 : PANEL_A;
                    pl_k  <= (rel == 10'd0) ? 2'd2 : 2'd1;
                    if (rel >= 10'd4 && rel < 10'd8) begin
                        dc_on <= 1'b1; dcx0 <= 10'd296; dcx1 <= 10'd344; dc_k <= 2'd0;
                    end else if (rel == 10'd52) begin
                        dc_on <= 1'b1; dcx0 <= 10'd16; dcx1 <= 10'd624; dc_k <= 2'd1;
                    end else if (in_list && pg_ok && lent == {4'd0, rr} && lsub_n >= 5'd2 && lsub_n < 5'd22) begin
                        dc_on <= 1'b1; dcx0 <= 10'd20; dcx1 <= 10'd23; dc_k <= 2'd2;
                    end
                    ralpha <= pa;
                    if (in_list && pg_ok && lent < {3'd0, rows_n} && lsub_n >= 5'd4 && lsub_n < 5'd20) begin
                        hs_re <= 1'b1; hs_addr <= {3'b111, lent[3:0], 2'd0};
                        rx0   <= (lent == 8'd0) ? 10'd64 : 10'd32;      // 首行缩进 2 字
                        rrow  <= lsub_n[3:0] - 4'd4;
                        run_s <= 1'b0; rmode <= M_KARA;
                        hs <= HS_1;
                    end else if (pg_ok && t_w != 7'd0 && ptl_dy < 10'd32) begin
                        run_on <= 1'b1; run_s <= 1'b1; fa <= t_start; fh <= 1'b0; rw <= t_w; rx0 <= 10'd24;
                        rrow <= ptl_dy[4:1]; rpar <= ptl_dy[0]; rmode <= M_TITLE;
                    end
                end else if (page_vis && !hide) begin
                    pl_on <= 1'b1; plx0 <= 10'd0; plx1 <= 11'd640; pl_k <= 2'd3;  // 页面上方压暗
                    pl_a  <= veil_a[7:0];
                end else if (main_ok) begin
                    rrow <= i_band ? b_dy[3:0] : s_band ? {s_rm[4:2], s_rm[1]} : b_dy[4:1];
                    rpar <= s_band ? s_rm[0] : b_dy[0];
                    run_s <= !i_band;
                    if (s_band) begin
                        // 跑马灯：满宽底板 + 本帧参数
                        if (gend != 11'd0 && sa != 8'd0) begin
                            pl_on <= 1'b1; plx0 <= 10'd0; plx1 <= 11'd640; pl_k <= 2'd0; pl_a <= s_pl;
                            run_on <= b_on; rmarq <= 1'b1; rboth <= s_rm[1]; ralpha <= sa; rmode <= M_KARA;
                            fa <= m_i0; fh <= m_h0; rpu0 <= m_pu0;
                            rx0 <= m_rx0; rx1 <= m_rx1; rs2 <= m_split;
                        end
                    end else if ((t_band && t_w != 7'd0) || (i_band && i_w != 7'd0)) begin
                        if (b_a != 8'd0) begin
                            pk_on <= 1'b1; pl_a <= b_pl; ralpha <= b_a;
                            run_on <= b_on;
                        end
                        if (t_band) begin
                            fa <= t_start; fh <= 1'b0; rw <= t_w; rx0 <= t_x0; rmode <= M_TITLE;
                        end else begin
                            fa <= i_start; fh <= 1'b0; rw <= i_w; rx0 <= 10'd16; rmode <= M_INTRO;
                        end
                    end
                end
            end
            // ---- 全文页段落行：读 {起点, 行首 G, 宽度}（读口 1 拍延迟）----
            HS_1: begin hs_re <= 1'b1; hs_addr <= {3'b111, lent_q[3:0], 2'd1}; hs <= HS_2; end
            HS_2: begin
                fa <= line_rdata[8:0]; fh <= 1'b0;
                hs_re <= 1'b1; hs_addr <= {3'b111, lent_q[3:0], 2'd2}; hs <= HS_3;
            end
            HS_3: begin rbase <= $signed({5'd0, rx0}) - $signed({1'b0, line_rdata[10:0], 3'b000}); hs <= HS_4; end
            HS_4: begin
                rw <= line_rdata[6:0]; run_on <= (line_rdata[6:0] != 7'd0);
                rs1 <= clampx(pg_s1);
                hs <= HS_5;
            end
            HS_5: begin rs2 <= clampx(pg_s2); hs <= HS_A; end
            // ---- 本段终点 / 预取第 0 块（fetch_go）----
            HS_A: begin
                if (!rmarq) rx1 <= {1'b0, rx0} + (run_s ? {rw, 4'b0000} : {1'b0, rw, 3'b000});
                pu <= rpu0;
                hs <= HS_B;
            end
            HS_B: begin
                if (pk_on) begin
                    pl_on <= 1'b1; pl_k <= 2'd0;
                    plx0  <= rx0 - 10'd12 + {8'd0, pk_in};
                    plx1  <= rx1 + 11'd12 - {9'd0, pk_in};
                end
                hs <= run_on ? HS_C : HS_IDLE;
            end
            HS_C: if (fp[4]) begin                              // 第 0 块到 nword → 装入，并预取第 1 块
                word <= nword;
                hs <= HS_IDLE;
            end
            default: hs <= HS_IDLE;
            endcase
        end
    end

    // ================= 图标（16×16 点阵）=================
    // 暂停图标（两竖条）：ic_r 2..13 行、列 3..5 / 10..12（16'h1C38）
    wire       show_p  = pause && loaded;
    // 暂停 x 616..631（616 = 16k + 8 → 列号 = x[3:0] ^ 8）
    wire [3:0] dxp     = pixel_x[3:0] ^ 4'b1000;
    wire       in_p    = (pixel_x >= 10'd616) && (pixel_x < 10'd632);
    wire       ic_on_r = (ic_r >= 4'd2) && (ic_r <= 4'd13);
    wire [15:0] ic_pause = 16'h1C38;
    wire       ic_px   = ic_row && show_p && in_p && ic_on_r && ic_pause[4'd15 - dxp];
    wire       ic_bg   = ic_bk && show_p && pixel_x >= 10'd614 && pixel_x < 10'd634;

    // ================= 文字像素（第 2 级混合）=================
    // 字块 8 字节 = 8 个 2×2 小块；2×：取 1 点（2 位）；1×：4 点求和（0..12）；
    // 跑马灯 3/4：相位 2 的列取两点和、rboth 行取两行和（bsel = pu[3:1] 对 3/4 编码同样成立）。覆盖率 → m（0..16）
    wire [2:0] bsel  = run_s ? pu[3:1] : pu[2:0];
    wire [7:0] gbyte = word[6'd63 - {bsel, 3'b000} -: 8];
    wire       rb    = !run_s || rboth;                                     // 两行求和
    wire       cb    = !run_s || pu_skip;                                   // 两列求和
    wire [2:0] gc0   = rb ? {1'b0, gbyte[7:6]} + {1'b0, gbyte[3:2]} : {1'b0, rpar ? gbyte[3:2] : gbyte[7:6]};
    wire [2:0] gc1   = rb ? {1'b0, gbyte[5:4]} + {1'b0, gbyte[1:0]} : {1'b0, rpar ? gbyte[1:0] : gbyte[5:4]};
    wire [3:0] gs    = cb ? {1'b0, gc0} + {1'b0, gc1} : {1'b0, pu[0] ? gc1 : gc0};
    wire [3:0] cov   = (rb && cb) ? gs : (rb || cb) ? {gs[2:0], 1'b0} : {gs[1:0], 2'b00};   // 0..12
    reg  [4:0] cov_m;                                                       // round(cov * 16 / 12)
    always @* begin
        case (cov)
            4'd0: cov_m = 5'd0;   4'd1: cov_m = 5'd1;   4'd2: cov_m = 5'd3;   4'd3: cov_m = 5'd4;
            4'd4: cov_m = 5'd5;   4'd5: cov_m = 5'd7;   4'd6: cov_m = 5'd8;   4'd7: cov_m = 5'd9;
            4'd8: cov_m = 5'd11;  4'd9: cov_m = 5'd12;  4'd10: cov_m = 5'd13; 4'd11: cov_m = 5'd15;
            default: cov_m = 5'd16;
        endcase
    end
    wire [12:0] t_am = ralpha * cov_m;
    assign txt_hit   = in_run && (cov != 4'd0) && (ralpha != 8'd0);
    assign txt_alpha = t_am[12] ? 8'd255 : t_am[11:4];

    // ================= 亮度同步：跑马灯 3 色 × bright_gain/256 =================
    // 画面已在上游按亮度压暗，这里只缩前景色（底板 C_PLATE、已读 C_READ、未读 C_UNREAD），标题 / 导语 / 图标不变。
    // 颜色是常数、增益帧内不变 → 每帧时间轴结束（tl_done，场消隐内）后串行算一遍：9 个通道 × 9 拍移位加（高位先），
    //   结果依次移入 72 位移位寄存器，不占 DSP、不在像素通路上加乘法器。
    // 全文页（page_vis）与主画面跑马灯不会同帧出现：页面打开时增益取 256（逐位原色），M_KARA 文字直接共用这两色，
    //   全文页亮度维持不变，像素端不用多一级选择。
    // 用 tl_done 启动：page_y 与它同拍更新，读到的是本帧的 page_vis；81 拍算完，最早用到的行（全文页 y>=128）远在其后。
    reg  [8:0]  bm_g;
    reg  [3:0]  bm_i;                         // 通道 0..8：底板 RGB、已读 RGB、未读 RGB
    reg  [3:0]  bm_b;                         // 当前乘数位 8..0
    reg         bm_run;
    reg  [15:0] bm_acc;
    reg  [71:0] bm_sh;                        // {s_plate, s_read, s_unread}，纯数据不复位
    reg  [7:0]  bm_c;
    always @* begin
        case (bm_i)
            4'd0: bm_c = C_PLATE[23:16];  4'd1: bm_c = C_PLATE[15:8];  4'd2: bm_c = C_PLATE[7:0];
            4'd3: bm_c = C_READ[23:16];   4'd4: bm_c = C_READ[15:8];   4'd5: bm_c = C_READ[7:0];
            4'd6: bm_c = C_UNREAD[23:16]; 4'd7: bm_c = C_UNREAD[15:8]; default: bm_c = C_UNREAD[7:0];
        endcase
    end
    wire [15:0] bm_nx = {bm_acc[14:0], 1'b0} + (bm_g[bm_b] ? {8'd0, bm_c} : 16'd0);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bm_run <= 1'b0; bm_i <= 4'd0; bm_b <= 4'd8; bm_g <= 9'h100;
        end else if (tl_done) begin
            bm_run <= 1'b1; bm_i <= 4'd0; bm_b <= 4'd8;
            bm_g   <= page_vis ? 9'h100 : bright_gain;
        end else if (bm_run) begin
            bm_b <= (bm_b == 4'd0) ? 4'd8 : bm_b - 4'd1;
            if (bm_b == 4'd0) begin
                bm_i <= bm_i + 4'd1;
                if (bm_i == 4'd8) bm_run <= 1'b0;
            end
        end
    end
    always @(posedge clk) begin
        if (tl_done)
            bm_acc <= 16'd0;
        else if (bm_run) begin
            bm_acc <= (bm_b == 4'd0) ? 16'd0 : bm_nx;
            if (bm_b == 4'd0)
                bm_sh <= {bm_sh[63:0], bm_nx[15:8]};
        end
    end
    wire [23:0] s_plate  = bm_sh[71:48];
    wire [23:0] s_read   = bm_sh[47:24];
    wire [23:0] s_unread = bm_sh[23:0];

    always @* begin
        case (rmode)
            M_TITLE: txt_color = C_TITLE;
            M_INTRO: txt_color = C_INTRO;
            default: txt_color = (pixel_x < rs1) ? C_PAST : (pixel_x < rs2) ? s_read : s_unread;
        endcase
    end

    // ================= 底板 / 装饰 / 图标（第 1 级，优先级：图标 > 图标底框 > 装饰 > 底板）=================
    wire       dc_px  = dc_on && de && pixel_x >= dcx0 && pixel_x < dcx1;
    wire       pl_px  = pl_on && de && pixel_x >= plx0 && {1'b0, pixel_x} < plx1;
    // 颜色只在输出端查一次表（3 位编号 → 24 位常数）：0..3 = {1'b0, pl_k}  4..6 = {1'b1, dc_k}  7 = 图标白
    wire [2:0] col_k  = ic_px ? 3'd7 : ic_bg ? 3'd3 : dc_px ? {1'b1, dc_k} : {1'b0, pl_k};
    reg [23:0] col_c;
    always @* begin
        case (col_k)
            3'd0:    col_c = rmarq ? s_plate : C_PLATE;   // 跑马灯行用亮度同步后的底板色，标题 / 导语底板不变
            3'd1:    col_c = C_PANEL;
            3'd2:    col_c = C_HILITE;
            3'd4:    col_c = C_GRAB;
            3'd5:    col_c = C_DIV;
            3'd6:    col_c = C_READ;
            3'd7:    col_c = 24'hF4F6F8;
            default: col_c = 24'h000000;          // 3 = 压暗 / 图标底框
        endcase
    end
    assign ext_hit   = de && (ic_px || ic_bg || dc_px || pl_px);
    assign ext_color = col_c;
    assign ext_alpha = ic_px ? 8'd240 : ic_bg ? 8'd120 : dc_px ? 8'd255 : pl_a;
endmodule
