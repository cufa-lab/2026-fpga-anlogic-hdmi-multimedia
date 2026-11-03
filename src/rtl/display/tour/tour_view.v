`timescale 1ns/1ps

module tour_view #(
    parameter integer H_START  = 160,
    parameter integer V_START  = 45
) (
    // ---------------- pix ----------------
    input  wire        pix_clk,
    input  wire        pix_rst_n,
    input  wire [80:0] bundle,             // tour_sequencer → cdc_bus_handshake（pix 域准静态）
    input  wire        frame_start,
    input  wire [11:0] raw_x,
    input  wire [11:0] raw_y,
    input  wire [1:0]  bank,               // 当前显示 bank（pix）
    output reg         tour_on,            // 本帧由本模块出图
    output reg  [23:0] tour_rgb,
    // ---------------- mem ----------------
    input  wire        mem_clk,
    input  wire        mem_rst_n,
    output wire        dma_want,
    input  wire        dma_grant,
    output wire        dma_busy,
    output wire        dma_own,
    output reg         app_rd_en,
    output reg  [20:0] app_rd_addr,
    input  wire        sdram_rd_valid,
    input  wire [31:0] sdram_rd_data
);
    localparam [11:0] PRE_X  = H_START - 10;         // 这一拍末启动，下一拍（raw_x = 151）是 k = -1 的读地址拍

    // ======================= 帧锁存 =======================
    reg [15:0] u0;  reg [14:0] v0;  reg [16:0] s;
    reg [1:0]  bk;
    reg [9:0]  b_clo, b_n;                           // B 取列范围
    // c_hi = min(639, ⌊u0/64⌋ + 640s/65536 + 3) | 3（640s/65536 = s>>7 + s>>9，偏大不偏小）
    wire [9:0]  bu_lo  = bundle[19:10];
    wire [11:0] bu_hi0 = {2'd0, bu_lo} + {2'd0, bundle[51:42]} + {4'd0, bundle[51:44]} + 12'd3;   // s>>7 + s>>9
    wire [9:0]  bu_hi  = (bu_hi0 > 12'd639) ? 10'd639 : (bu_hi0[9:0] | 10'd3);
    wire [9:0]  bu_clo = {bu_lo[9:2], 2'b00};

    // ======================= 行描述 =======================
    reg [25:0] vb;                                   // 帧缓存 v（Q.16）
    reg [9:0]  ly;                                   // nxt 对应的屏幕行
    reg        upd;
    reg        n_val;  reg [8:0] n_a, n_b;  reg [5:0] n_fy;
    reg        c_val, c_bok;  reg [1:0] c_sa, c_sb;  reg [5:0] c_fy;

    wire [9:0] vb_a = vb[25:16];
    wire [8:0] f_a  = (vb_a > 10'd479) ? 9'd479 : vb_a[8:0];

    // ======================= 行槽标签 =======================
    reg [2:0]  t_v;
    reg [8:0]  t_row0, t_row1, t_row2;
    wire [8:0] t_row [0:2];
    assign t_row[0] = t_row0; assign t_row[1] = t_row1; assign t_row[2] = t_row2;
    wire [2:0] h_a, h_b;
    genvar gi;
    generate for (gi = 0; gi < 3; gi = gi + 1) begin : g_hit
        assign h_a[gi] = t_v[gi] && (t_row[gi] == n_a);
        assign h_b[gi] = t_v[gi] && (t_row[gi] == n_b);
    end endgenerate
    function [1:0] enc; input [2:0] h; begin enc = h[0] ? 2'd0 : h[1] ? 2'd1 : 2'd2; end endfunction
    wire [2:0] in_use;
    generate for (gi = 0; gi < 3; gi = gi + 1) begin : g_use
        assign in_use[gi] = (c_val && (c_sa == gi || c_sb == gi)) || h_a[gi] || h_b[gi];
    end endgenerate
    wire       any_free = in_use != 3'b111;
    wire [1:0] victim   = !in_use[0] ? 2'd0 : !in_use[1] ? 2'd1 : 2'd2;

    // ======================= DMA 请求（pix → mem toggle）=======================
    reg        rq_t, pend, stale;
    reg        ack_t;                                // mem 域：DMA 完成 toggle
    reg [1:0]  rq_slot;  reg [8:0] rq_row;  reg [9:0] rq_clo, rq_n;  reg [1:0] rq_bank;
    (* async_reg = "true" *) reg ack_sync1, ack_sync2;
    reg        ack_seen;
    wire       ack_ev = (ack_sync2 != ack_seen);

    wire        line_go = tour_on && (raw_x == 12'd0) && (raw_y >= V_START) && (raw_y < V_START + 480);
    wire        need = tour_on && n_val && !upd && !pend && (!(|h_a) || !(|h_b)) && any_free;

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            tour_on <= 1'b0;
            u0 <= 16'd0; v0 <= 15'd0; s <= 17'd65536;
            bk <= 2'd0; b_clo <= 10'd0; b_n <= 10'd640;
            vb <= 26'd0; ly <= 10'd0; upd <= 1'b0;
            n_val <= 1'b0; n_a <= 9'd0; n_b <= 9'd0; n_fy <= 6'd0;
            c_val <= 1'b0; c_bok <= 1'b0; c_sa <= 2'd0; c_sb <= 2'd0; c_fy <= 6'd0;
            t_v <= 3'd0; t_row0 <= 9'd0; t_row1 <= 9'd0; t_row2 <= 9'd0;
            rq_t <= 1'b0; pend <= 1'b0; stale <= 1'b0;
            rq_slot <= 2'd0; rq_row <= 9'd0; rq_clo <= 10'd0; rq_n <= 10'd0; rq_bank <= 2'd0;
            ack_sync1 <= 1'b0; ack_sync2 <= 1'b0; ack_seen <= 1'b0;
        end else begin
            ack_sync1 <= ack_t;
            ack_sync2 <= ack_sync1;
            if (ack_ev) begin
                ack_seen <= ack_sync2;
                pend     <= 1'b0;
                stale    <= 1'b0;
                if (!stale && !frame_start) begin
                    t_v[rq_slot]   <= 1'b1;
                    case (rq_slot) 2'd0: t_row0 <= rq_row; 2'd1: t_row1 <= rq_row; default: t_row2 <= rq_row; endcase
                end
            end

            if (frame_start) begin
                tour_on <= bundle[0];
                u0 <= bundle[19:4]; v0 <= bundle[34:20]; s <= bundle[51:35];
                bk  <= bank;
                b_clo <= bu_clo;
                b_n   <= bu_hi - bu_clo + 10'd1;
                vb  <= {bundle[34:20], 10'd0};
                ly  <= 10'd0;
                upd <= 1'b1;
                t_v <= 3'd0;
                c_val <= 1'b0;
                if (pend && !ack_ev) stale <= 1'b1;
            end else if (upd) begin
                upd   <= 1'b0;
                n_val <= (ly < 10'd480);
                n_a   <= f_a;
                n_b   <= (f_a == 9'd479) ? f_a : f_a + 9'd1;
                n_fy  <= vb[15:10];
            end else if (line_go) begin
                // 行首：nxt（= 本行）a 在槽里 → 换上；否则重复上一行
                if (n_val && (|h_a)) begin
                    c_val <= 1'b1;
                    c_sa  <= enc(h_a);
                    c_sb  <= (|h_b) ? enc(h_b) : enc(h_a);
                    c_bok <= |h_b;
                    c_fy  <= (|h_b) ? n_fy : 6'd0;
                end
                vb  <= vb + {9'd0, s};
                ly  <= ly + 10'd1;
                upd <= 1'b1;
            end else if (need) begin
                rq_slot <= victim;
                rq_row  <= !(|h_a) ? n_a : n_b;
                rq_clo  <= b_clo;
                rq_n    <= b_n;
                rq_bank <= bk;
                rq_t    <= ~rq_t;
                pend    <= 1'b1;
                t_v[victim] <= 1'b0;
            end
        end
    end

    // ======================= 渲染流水 =======================
    // t_k = 152 + k（k = -1..639）：
    //   t_k   读地址（k = -1：a0；否则 min(⌊u⌋+1, maxcol)）
    //   +1    RAM 输出 → 选上下两行
    //   +2    竖插乘法    +3 竖插加     +4 L / R 移位    +5 横插乘法    +6 横插加（hv）
    //   +7    tour_rgb（raw_x = 160 + k 时可见）
    reg        r_act;
    reg [10:0] r_k;                                  // k + 1（0 = 预读）
    reg [25:0] r_u;
    reg [9:0]  ia_q;
    wire [9:0] ia = r_u[25:16];
    wire [9:0] ia1 = (ia >= 10'd639) ? 10'd639 : ia + 10'd1;
    wire [9:0] rd_addr = (r_k == 11'd0) ? ia : ia1;
    wire       adv0 = (r_k == 11'd1) || (ia != ia_q);   // k = 0 强制移位；之后列号前进时移位

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            r_act <= 1'b0; r_k <= 11'd0; r_u <= 26'd0; ia_q <= 10'd0;
        end else begin
            if (raw_x == PRE_X && c_val && tour_on) begin
                r_act  <= 1'b1;
                r_k    <= 11'd0;
                r_u    <= {u0, 10'd0};
            end else if (r_act) begin
                r_k  <= r_k + 11'd1;
                if (r_k != 11'd0) r_u <= r_u + {9'd0, s};
                if (r_k == 11'd640) r_act <= 1'b0;
            end
            ia_q <= ia;
        end
    end

    // 行槽 RAM（mem 写 / pix 读）
    wire        dm_we;
    wire [1:0]  dm_slot;
    wire [9:0]  dm_wa;
    wire [23:0] q0, q1, q2;
    script_dc_ram #(.AW(10), .DW(24)) u_slot0 (.wclk(mem_clk), .we(dm_we && dm_slot == 2'd0), .waddr(dm_wa), .wdata(sdram_rd_data[23:0]),
                                                .rclk(pix_clk), .re(r_act), .raddr(rd_addr), .rdata(q0));
    script_dc_ram #(.AW(10), .DW(24)) u_slot1 (.wclk(mem_clk), .we(dm_we && dm_slot == 2'd1), .waddr(dm_wa), .wdata(sdram_rd_data[23:0]),
                                                .rclk(pix_clk), .re(r_act), .raddr(rd_addr), .rdata(q1));
    script_dc_ram #(.AW(10), .DW(24)) u_slot2 (.wclk(mem_clk), .we(dm_we && dm_slot == 2'd2), .waddr(dm_wa), .wdata(sdram_rd_data[23:0]),
                                                .rclk(pix_clk), .re(r_act), .raddr(rd_addr), .rdata(q2));

    reg [6:0]  p_act;                                // 有效位流水 [0] = +1 ... [6] = +7
    reg [5:0]  p_fx1, p_fx2, p_fx3, p_fx4, p_fx5;
    reg [3:0]  p_adv;                                // [0] = +1 ... [3] = +4
    reg [23:0] s1_t, s1_b, s2_t, s5_l;
    reg [23:0] vr_q, lq, rq;
    (* dsp_style = "yes" *) reg signed [15:0] vp_r, vp_g, vp_b;
    (* dsp_style = "yes" *) reg signed [15:0] hp_r, hp_g, hp_b;
    reg [23:0] hv;
    wire [23:0] sel_t = (c_sa == 2'd0) ? q0 : (c_sa == 2'd1) ? q1 : q2;
    wire [23:0] sel_b = (c_sb == 2'd0) ? q0 : (c_sb == 2'd1) ? q1 : q2;

    function signed [8:0] dif; input [7:0] a, b; begin dif = $signed({1'b0, b}) - $signed({1'b0, a}); end endfunction
    function [7:0] addp; input [7:0] a; input signed [15:0] p; reg signed [9:0] t; begin
        t = $signed({2'b00, a}) + (p >>> 6); addp = t[7:0];
    end endfunction

    always @(posedge pix_clk) begin
        p_act <= {p_act[5:0], r_act};
        p_adv <= {p_adv[2:0], adv0};
        p_fx1 <= r_u[15:10]; p_fx2 <= p_fx1; p_fx3 <= p_fx2; p_fx4 <= p_fx3; p_fx5 <= p_fx4;
        // +1：选行
        s1_t <= sel_t;
        s1_b <= sel_b;
        // +2：竖插乘法
        vp_r <= dif(s1_t[23:16], s1_b[23:16]) * $signed({1'b0, c_fy});
        vp_g <= dif(s1_t[15:8],  s1_b[15:8])  * $signed({1'b0, c_fy});
        vp_b <= dif(s1_t[7:0],   s1_b[7:0])   * $signed({1'b0, c_fy});
        s2_t <= s1_t;
        // +3：竖插加
        vr_q <= {addp(s2_t[23:16], vp_r), addp(s2_t[15:8], vp_g), addp(s2_t[7:0], vp_b)};
        // +4：L / R
        if (p_act[3]) begin
            rq <= vr_q;
            if (p_adv[3]) lq <= rq;
        end
        // +5：横插乘法（fx 跟随 k：t_k 的 r_u 低位，延迟 5 拍）
        hp_r <= dif(lq[23:16], rq[23:16]) * $signed({1'b0, p_fx5});
        hp_g <= dif(lq[15:8],  rq[15:8])  * $signed({1'b0, p_fx5});
        hp_b <= dif(lq[7:0],   rq[7:0])   * $signed({1'b0, p_fx5});
        s5_l <= lq;
        // +6：横插加
        hv <= {addp(s5_l[23:16], hp_r), addp(s5_l[15:8], hp_g), addp(s5_l[7:0], hp_b)};
    end

    // L7：纯数据寄存器不接复位（下游/本级已按 de、锁定门控，复位期间时钟照常冲成 0）
    always @(posedge pix_clk)
        tour_rgb <= p_act[6] ? hv : 24'd0;

    // ======================= DMA（mem）=======================
    (* async_reg = "true" *) reg req_sync1, req_sync2;
    reg        req_seen;
    reg [1:0]  mq_slot;  reg [8:0] mq_row;  reg [9:0] mq_clo, mq_n;  reg [1:0] mq_bank;
    reg        m_go, m_act, b_act;
    reg [9:0]  m_iss, m_ret, b_left;
    reg [9:0]  m_wa;
    reg [20:0] m_addr;
    wire [20:0] base_b = (mq_bank == 2'd0) ? 21'd0 : (mq_bank == 2'd1) ? 21'd307200 :
                         (mq_bank == 2'd2) ? 21'd614400 : 21'd921600;
    wire [20:0] row640 = {3'd0, mq_row, 9'd0} + {5'd0, mq_row, 7'd0};
    wire [9:0]  m_left = mq_n - m_iss;
    assign dma_own  = (m_iss != m_ret);
    assign dma_want = m_act && !b_act && !app_rd_en && !dma_own && (m_iss != mq_n);
    assign dma_busy = b_act || app_rd_en || dma_own;
    assign dm_we    = sdram_rd_valid && dma_own;
    assign dm_slot  = mq_slot;
    assign dm_wa    = m_wa;

    always @(posedge mem_clk or negedge mem_rst_n) begin
        if (!mem_rst_n) begin
            req_sync1 <= 1'b0; req_sync2 <= 1'b0; req_seen <= 1'b0; ack_t <= 1'b0;
            mq_slot <= 2'd0; mq_row <= 9'd0; mq_clo <= 10'd0; mq_n <= 10'd0; mq_bank <= 2'd0;
            m_go <= 1'b0; m_act <= 1'b0; b_act <= 1'b0; m_iss <= 10'd0; m_ret <= 10'd0; b_left <= 10'd0;
            m_wa <= 10'd0; m_addr <= 21'd0; app_rd_en <= 1'b0; app_rd_addr <= 21'd0;
        end else begin
            req_sync1 <= rq_t;
            req_sync2 <= req_sync1;
            app_rd_en <= 1'b0;
            if (!m_act && !m_go && (req_sync2 != req_seen)) begin
                req_seen <= req_sync2;
                mq_slot <= rq_slot; mq_row <= rq_row; mq_clo <= rq_clo; mq_n <= rq_n; mq_bank <= rq_bank;
                m_go <= 1'b1;
            end
            if (m_go) begin                                  // 载荷已稳定 1 拍：算首地址
                m_go   <= 1'b0;
                m_act  <= 1'b1;
                m_iss  <= 10'd0;
                m_ret  <= 10'd0;
                m_wa   <= mq_clo;
                m_addr <= base_b + row640 + {11'd0, mq_clo};
            end
            if (dma_want && dma_grant) begin
                b_act  <= 1'b1;
                b_left <= (m_left > 10'd128) ? 10'd128 : m_left;
            end
            if (b_act) begin
                app_rd_en   <= 1'b1;
                app_rd_addr <= m_addr;
                m_addr      <= m_addr + 21'd1;
                m_iss       <= m_iss + 10'd1;
                b_left      <= b_left - 10'd1;
                if (b_left == 10'd1) b_act <= 1'b0;
            end
            if (dm_we) begin
                m_wa  <= m_wa + 10'd1;
                m_ret <= m_ret + 10'd1;
            end
            if (m_act && !b_act && !app_rd_en && !dma_own && (m_iss == mq_n)) begin
                m_act <= 1'b0;
                ack_t <= ~ack_t;
            end
        end
    end
endmodule
