`timescale 1ns/1ps

module video_fade_transition #(
    parameter integer FADE_FRAMES         = 0,   // 每段（淡出/淡入）帧数；0 = 旁路；须整除 256
    parameter integer HOLD_TIMEOUT_FRAMES = 0,   // HOLD 最长等待帧数；0 = 不超时
    parameter integer DISSOLVE_STEP       = 2    // 溶解每帧 ramp 步进（须整除 FADE_FRAMES）；1 = 与淡入淡出同速
) (
    // ---------------- 控制域 ----------------
    input  wire       ctrl_clk,
    input  wire       ctrl_rst_n,
    input  wire       next_event_in,    // 手动切换请求（单拍）
    input  wire       next_tag_in,      // 请求标签（与 next_event_in 同拍有效）
    input  wire       next_auto_in,     // 1 = 自动轮播请求 → 方块溶解（与 next_event_in 同拍有效）
    input  wire       next_block_in,    // 1 = 控制层忙，丢弃新请求
    input  wire       hold_abort_in,    // 单拍：控制层拒绝/切换失败 → 结束 HOLD
    output wire       next_event_out,   // 全黑时放出的切换请求（单拍）
    output wire       next_tag_out,     // 与 next_event_out 对应的标签
    output wire       fade_busy,        // 转场进行中（已同步到控制域）

    // ---------------- 像素域 ----------------
    input  wire       pix_clk,
    input  wire       pix_rst_n,
    input  wire       player_active,    // 已有图片在显示
    input  wire       frame_start,      // 帧边界（单拍，帧消隐期）
    input  wire       index_commit,     // 控制层在帧边界提交了新图（单拍）
    output wire [7:0] alpha_out,
    output wire       fade_active,
    // 方块溶解：dissolve_active 打开像素域块序号比较，ramp_out 推进显示档位
    output wire       dissolve_active,
    output wire [4:0] ramp_out
);
    generate
        if (FADE_FRAMES == 0) begin : g_bypass
            assign next_event_out  = next_event_in;
            assign next_tag_out    = next_tag_in;
            assign fade_busy       = 1'b0;
            assign alpha_out       = 8'd255;
            assign fade_active     = 1'b0;
            assign dissolve_active = 1'b0;
            assign ramp_out        = 5'd0;
        end else begin : g_fade
            // 256 % FADE_FRAMES 必须为 0，ramp×STEP 才正好覆盖 0..255
            localparam integer STEP  = 256 / FADE_FRAMES;
            // OUT 刚进入时 ramp = FADE_FRAMES（请求在帧中间到达，本帧保持全亮），要多一个值
            localparam integer CNT_W = (FADE_FRAMES <= 2) ? 2 : $clog2(FADE_FRAMES + 1);
            localparam integer HTO_W = (HOLD_TIMEOUT_FRAMES < 2) ? 1 : $clog2(HOLD_TIMEOUT_FRAMES + 1);

            localparam [1:0] S_IDLE = 2'd0;
            localparam [1:0] S_OUT  = 2'd1;
            localparam [1:0] S_HOLD = 2'd2;
            localparam [1:0] S_IN   = 2'd3;

            // synthesis translate_off
            initial begin
                if ((256 % FADE_FRAMES) != 0 || FADE_FRAMES > 64)
                    $display("ERROR: video_fade_transition FADE_FRAMES=%0d 必须整除 256 且 <= 64", FADE_FRAMES);
            end
            // synthesis translate_on

            // ---- 两个 always 块共用的寄存器（先声明）----
            reg req_toggle;                                  // ctrl → pix：切换请求
            reg abort_toggle;                                // ctrl → pix：结束 HOLD
            reg ack_seen;                                    // ctrl：已处理的 commit 值
            reg req_auto;                                    // ctrl：本次请求是自动轮播（准静态 → pix）
            (* async_reg = "true" *) reg commit_sync1, commit_sync2;
            (* async_reg = "true" *) reg busy_sync1, busy_sync2;
            (* async_reg = "true" *) reg seen_sync1, seen_sync2;

            // ---- 像素域 ----
            reg [1:0]       state;
            reg [CNT_W-1:0] ramp;
            (* async_reg = "true" *) reg req_sync1, req_sync2;
            (* async_reg = "true" *) reg abort_sync1, abort_sync2;
            reg             req_seen;
            reg             abort_seen;
            reg             commit_toggle;                   // pix → ctrl：淡出完成
            reg             dissolve_mode;                   // 本次转场走方块溶解
            reg [HTO_W-1:0] hold_frames;

            // 溶解：每帧步进 DS，两端停在 DS（不出 ramp=0 的黑帧）；淡入淡出：步进 1，停在 0 / FADE_FRAMES-1
            localparam integer DS = (DISSOLVE_STEP < 1) ? 1 : DISSOLVE_STEP;
            wire [CNT_W-1:0] r_step   = dissolve_mode ? DS[CNT_W-1:0] : {{(CNT_W-1){1'b0}}, 1'b1};
            wire             out_last = dissolve_mode ? (ramp <= DS) : (ramp == 0);
            wire             in_last  = (ramp >= FADE_FRAMES - r_step);

            wire fade_running = (state != S_IDLE);
            wire req_new      = (req_sync2 != req_seen);
            wire abort_new    = (abort_sync2 != abort_seen);
            wire hold_timeout = (HOLD_TIMEOUT_FRAMES != 0) && frame_start &&
                                (hold_frames == HOLD_TIMEOUT_FRAMES - 1);

            assign fade_active     = fade_running;
            assign dissolve_active = dissolve_mode && state[0];   // S_OUT=01 / S_IN=11
            if (CNT_W <= 5) begin : g_ramp_ext
                // 显式零扩展：越界的部分选择会读出 X
                assign ramp_out = {{(5-CNT_W){1'b0}}, ramp};
            end else begin : g_ramp_trunc
                assign ramp_out = ramp[4:0];
            end

            // 溶解 OUT 段音频 alpha 提前一档（ramp - DS）：最后一个 OUT 帧已是 0，sync_fade_gain 帧边界锁存后
            // HOLD 那 1 帧静音（与手动"淡出到 0 再全黑"同样的黑即静音）；画面 alpha 由顶层置 255，不受影响
            wire [CNT_W-1:0] a_ramp      = (dissolve_mode && state == S_OUT) ? ramp - DS[CNT_W-1:0] : ramp;
            wire [CNT_W+8:0] ramp_alpha  = a_ramp * STEP;
            wire [7:0]       ramp_alpha8 = ramp_alpha[7:0];
            // ramp == FADE_FRAMES：请求在帧中间到达，本帧仍全亮，下一完整帧才开始变暗
            assign alpha_out = (state == S_IDLE)       ? 8'd255 :
                               (state == S_HOLD)       ? 8'd0   :
                               (ramp == FADE_FRAMES)   ? 8'd255 : ramp_alpha8;

            // ======================== 像素域 4 状态机 ========================
            always @(posedge pix_clk or negedge pix_rst_n) begin
                if (!pix_rst_n) begin
                    state         <= S_IDLE;
                    ramp          <= {CNT_W{1'b0}};
                    req_sync1     <= 1'b0;
                    req_sync2     <= 1'b0;
                    req_seen      <= 1'b0;
                    abort_sync1   <= 1'b0;
                    abort_sync2   <= 1'b0;
                    abort_seen    <= 1'b0;
                    commit_toggle <= 1'b0;
                    dissolve_mode <= 1'b0;
                    hold_frames   <= {HTO_W{1'b0}};
                end else begin
                    req_sync1   <= req_toggle;
                    req_sync2   <= req_sync1;
                    abort_sync1 <= abort_toggle;
                    abort_sync2 <= abort_sync1;
                    // abort 只在 HOLD 有意义；其它状态到达的也记账，避免过期 abort 留到下一次 HOLD
                    if (abort_new)
                        abort_seen <= abort_sync2;

                    case (state)
                    S_IDLE: begin
                        if (req_new) begin
                            // 手动切换请求；没图在显示就丢弃（策略层此时同样不响应）
                            req_seen <= req_sync2;
                            if (player_active) begin
                                state         <= S_OUT;
                                ramp          <= FADE_FRAMES[CNT_W-1:0];
                                dissolve_mode <= req_auto;
                            end
                        end
                    end
                    S_OUT: begin
                        if (frame_start) begin
                            if (out_last) begin
                                // 淡出到全黑 → 翻转 commit_toggle，控制域放出真实切换请求
                                state         <= S_HOLD;
                                hold_frames   <= {HTO_W{1'b0}};
                                commit_toggle <= ~commit_toggle;
                            end else begin
                                ramp <= ramp - r_step;
                            end
                        end
                        // 淡出中也跟踪请求握手，提前到达的第二个事件不会让 req_seen 失配
                        if (req_new)
                            req_seen <= req_sync2;
                    end
                    S_HOLD: begin
                        if (req_new)
                            req_seen <= req_sync2;
                        if (frame_start)
                            hold_frames <= hold_frames + 1'b1;
                        // 新图已提交 → 淡入新图；切换被拒/失败/超时 → 淡入（仍是旧图）
                        if (index_commit || abort_new || hold_timeout) begin
                            state <= S_IN;
                            ramp  <= dissolve_mode ? DS[CNT_W-1:0] : {CNT_W{1'b0}};
                        end
                    end
                    S_IN: begin
                        if (req_new)
                            req_seen <= req_sync2;
                        if (frame_start) begin
                            if (in_last)
                                state <= S_IDLE;
                            else
                                ramp <= ramp + r_step;
                        end
                    end
                    endcase
                end
            end

            // ======================== 控制域握手 ========================
            reg req_wait_d;
            reg tag_hold;
            wire req_wait = (req_toggle != seen_sync2);    // 上一个请求还没被像素域收下
            wire accept   = next_event_in && !next_block_in && !busy_sync2 &&
                            !req_wait && !req_wait_d;

            // 已接受、像素域还没收下的请求也算忙：顶层用 fade_busy 屏蔽 auto_tick，
            // 否则接受请求后 busy 回来前的几拍里，轮播提交可能落在淡出中途
            assign fade_busy      = busy_sync2 | req_wait | req_wait_d;
            assign next_event_out = (commit_sync2 != ack_seen);
            assign next_tag_out   = tag_hold;

            always @(posedge ctrl_clk or negedge ctrl_rst_n) begin
                if (!ctrl_rst_n) begin
                    req_toggle   <= 1'b0;
                    abort_toggle <= 1'b0;
                    ack_seen     <= 1'b0;
                    commit_sync1 <= 1'b0;
                    commit_sync2 <= 1'b0;
                    busy_sync1   <= 1'b0;
                    busy_sync2   <= 1'b0;
                    seen_sync1   <= 1'b0;
                    seen_sync2   <= 1'b0;
                    req_wait_d   <= 1'b0;
                    tag_hold     <= 1'b0;
                    req_auto     <= 1'b0;
                end else begin
                    busy_sync1   <= fade_running;
                    busy_sync2   <= busy_sync1;
                    commit_sync1 <= commit_toggle;
                    commit_sync2 <= commit_sync1;
                    seen_sync1   <= req_seen;
                    seen_sync2   <= seen_sync1;
                    // req_seen 与 state 同拍更新，但两路 2FF 落地可能差 1 拍：多等 1 拍再看 busy
                    req_wait_d   <= req_wait;
                    // 转场进行中的事件直接丢弃：画面已在过渡，控制层在全黑前也不能接第二个请求
                    if (accept) begin
                        req_toggle <= ~req_toggle;
                        tag_hold   <= next_tag_in;
                        req_auto   <= next_auto_in;
                    end
                    if (hold_abort_in && busy_sync2)
                        abort_toggle <= ~abort_toggle;
                    if (commit_sync2 != ack_seen)
                        ack_seen <= commit_sync2;
                end
            end
        end
    endgenerate
endmodule
