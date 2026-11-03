`timescale 1ns/1ps

module thumbnail_controller #(
    parameter integer FOCUS_FOLLOW = 1
) (
    // ---------------- sys_clk 域（请求）----------------
    input  wire        clk,
    input  wire        rst_n,
    input  wire        enable,             // 有图在显示（display_valid_sys）；0 时只能是全屏
    input  wire        key_right,          // 单拍：焦点右移
    input  wire        key_left,           // 单拍：焦点左移
    input  wire        key_confirm,        // 单拍：选中当前焦点
    input  wire        key_mode,           // 单拍：全屏 / 缩略图切换
    input  wire [3:0]  image_complete,     // 本页 4 格缩略图完整，bit s = 格 s（左上 0 / 右上 1 / 左下 2 / 右下 3）
    input  wire [5:0]  current_image_idx,  // 已提交（ack 回来）的图片；页 = [5:2]
    input  wire [6:0]  image_count,
    input  wire        select_busy,        // 切图 / 转场在途（request_busy | fade_busy）
    input  wire        select_error,       // 控制层报错（单拍）：快速选择被拒 / 加载失败

    output reg         view_mode,          // 请求值：0 = 全屏，1 = 缩略图
    output reg  [1:0]  focus_index,        // 请求值：焦点格
    output wire [1:0]  active_index,       // 已提交图片的格子号（只读，active_image_idx 由控制层提交）
    output reg         transition_req,     // 单拍：发出一次模式切换（→ pix 域生效后由 thumb_zoom_transition 做动画）
    output reg         select_valid,       // 单拍：快速选择 → goto
    output reg  [5:0]  select_image_idx,
    output reg         select_reject,      // 单拍：确认了未加载 / 不存在的格子，或选择失败
    output wire        nav_enable,         // 1 = 方向键走原来的下一张 / 上一张
    output wire        view_busy,          // {模式, 焦点} 还没在帧边界生效

    // ---------------- pix_clk 域（生效）----------------
    input  wire        pix_clk,
    input  wire        pix_rst_n,
    input  wire        frame_start,        // 帧边界（单拍，场同步下降沿 = 控制层提交点）
    output reg         view_mode_pix,      // 生效值，只在 frame_start 变化
    output reg  [1:0]  focus_pix,
    output reg         view_commit         // 单拍：本帧边界提交了新的 {模式, 焦点}
);
    localparam [1:0] ST_FULL = 2'd0, ST_THUMB = 2'd1, ST_SELECT = 2'd2;

    reg [1:0] st;
    reg       enter_pend;
    reg       exit_pend;
    reg       sel_pend;

    // ---- sys → pix 握手（同 media_player_control 的 commit_toggle / ack_toggle）----
    reg [2:0] src_hold;                    // 在途载荷，保持到 ack 回来
    reg       req_toggle;
    (* async_reg = "true" *) reg ack_sync1;
    (* async_reg = "true" *) reg ack_sync2;
    reg       ack_toggle_pix;

    wire [2:0] payload  = {view_mode, focus_index};
    wire       hs_busy  = (req_toggle != ack_sync2);
    assign view_busy    = hs_busy || (payload != src_hold);
    wire       can_view = !view_busy && !select_busy;

    wire [5:0] focus_img = {current_image_idx[5:2], focus_index};
    wire       focus_ok  = image_complete[focus_index] && ({1'b0, focus_img} < image_count);

    assign active_index = current_image_idx[1:0];
    assign nav_enable   = FOCUS_FOLLOW ? !(enter_pend || exit_pend)
                                       : (st == ST_FULL) && !view_mode && !enter_pend;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st               <= ST_FULL;
            view_mode        <= 1'b0;
            focus_index      <= 2'd0;
            enter_pend       <= 1'b0;
            exit_pend        <= 1'b0;
            sel_pend         <= 1'b0;
            transition_req   <= 1'b0;
            select_valid     <= 1'b0;
            select_image_idx <= 6'd0;
            select_reject    <= 1'b0;
            src_hold         <= 3'd0;
            req_toggle       <= 1'b0;
            ack_sync1        <= 1'b0;
            ack_sync2        <= 1'b0;
        end else begin
            transition_req <= 1'b0;
            select_valid   <= 1'b0;
            select_reject  <= 1'b0;
            ack_sync1      <= ack_toggle_pix;
            ack_sync2      <= ack_sync1;

            // 握手空闲且请求值变了 → 锁存载荷、翻转 toggle（载荷保持到 ack）
            if (!hs_busy && (payload != src_hold)) begin
                src_hold   <= payload;
                req_toggle <= ~req_toggle;
            end

            if (!enable) begin
                // 没图在显示：缩略图请求退回全屏，撤销退出 / 选择挂起。
                // 进入挂起：FOCUS_FOLLOW 下 SW4 是电平，上电时已拨上的那一个拨上沿要保留到首图出来
                exit_pend  <= 1'b0;
                sel_pend   <= 1'b0;
                if (FOCUS_FOLLOW) begin
                    if (key_mode)
                        enter_pend <= ~enter_pend;
                    if (key_confirm)
                        enter_pend <= 1'b0;
                end else
                    enter_pend <= 1'b0;
                if (view_mode && !view_busy) begin
                    view_mode      <= 1'b0;
                    transition_req <= 1'b1;
                end
                if (!view_mode)
                    st <= ST_FULL;
            end else begin
                case (st)
                ST_FULL: begin
                    if (key_mode)
                        enter_pend <= ~enter_pend;
                    if (key_confirm)
                        enter_pend <= 1'b0;                  // 全屏下确认无意义；SW4 进 / 出抵消
                    if (enter_pend && can_view && !key_mode && !key_confirm) begin
                        view_mode      <= 1'b1;              // TODO1：进缩略图，焦点从当前图开始
                        focus_index    <= current_image_idx[1:0];
                        transition_req <= 1'b1;
                        enter_pend     <= 1'b0;
                        st             <= ST_THUMB;
                    end
                end

                ST_THUMB: begin
                    if (FOCUS_FOLLOW) begin
                        focus_index <= current_image_idx[1:0];
                        if (key_mode)
                            exit_pend <= ~exit_pend;
                        if (key_confirm)
                            exit_pend <= 1'b1;
                    end else begin
                        // TODO2：只在缩略图且没有挂起时动焦点
                        if (!exit_pend && !sel_pend) begin
                            if (key_right)
                                focus_index <= focus_index + 2'd1;   // 3 → 0 自然回绕
                            else if (key_left)
                                focus_index <= focus_index - 2'd1;   // 0 → 3 自然回绕
                        end
                        if (key_mode) begin
                            exit_pend <= ~exit_pend;                 // 回全屏，不切图
                            sel_pend  <= 1'b0;
                        end else if (key_confirm && !exit_pend && !sel_pend) begin
                            // TODO3：只有已加载的格子能选
                            if (!focus_ok)
                                select_reject <= 1'b1;
                            else if (focus_img == current_image_idx)
                                exit_pend <= 1'b1;
                            else begin
                                sel_pend         <= 1'b1;
                                select_image_idx <= focus_img;
                            end
                        end
                        if (sel_pend && !select_busy && !key_mode) begin
                            select_valid <= 1'b1;
                            sel_pend     <= 1'b0;
                            st           <= ST_SELECT;
                        end
                    end
                    if (exit_pend && can_view && !key_mode && !key_confirm) begin
                        view_mode      <= 1'b0;
                        transition_req <= 1'b1;
                        exit_pend      <= 1'b0;
                        st             <= ST_FULL;
                    end
                end

                ST_SELECT: begin
                    // 等控制层把目标图在帧边界提交（current_image_idx 在 ack 后更新），再回全屏
                    if (key_mode) begin
                        exit_pend <= 1'b1;                   // 放弃等待，直接回全屏
                        st        <= ST_THUMB;
                    end else if (select_error) begin
                        select_reject <= 1'b1;               // 被拒 / 加载失败：留在缩略图，焦点不动
                        st            <= ST_THUMB;
                    end else if ((current_image_idx == select_image_idx) && !select_busy) begin
                        exit_pend <= 1'b1;
                        st        <= ST_THUMB;
                    end
                end

                default: st <= ST_FULL;
                endcase
            end
        end
    end

    // ============================ pix_clk 域：帧边界生效 ============================
    (* async_reg = "true" *) reg req_sync1;
    (* async_reg = "true" *) reg req_sync2;
    reg       req_seen_pix;
    reg       pend_pix;
    reg [2:0] lat_pix;

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            req_sync1      <= 1'b0;
            req_sync2      <= 1'b0;
            req_seen_pix   <= 1'b0;
            pend_pix       <= 1'b0;
            lat_pix        <= 3'd0;
            ack_toggle_pix <= 1'b0;
            view_mode_pix  <= 1'b0;
            focus_pix      <= 2'd0;
            view_commit    <= 1'b0;
        end else begin
            req_sync1   <= req_toggle;
            req_sync2   <= req_sync1;
            view_commit <= 1'b0;
            // toggle 经两级同步才看到，此时 src_hold 已稳定 ≥2 个 pix 周期，直接采样
            if ((req_sync2 != req_seen_pix) && !pend_pix) begin
                req_seen_pix <= req_sync2;
                lat_pix      <= src_hold;
                pend_pix     <= 1'b1;
            end
            if (frame_start && pend_pix) begin
                {view_mode_pix, focus_pix} <= lat_pix;
                view_commit    <= 1'b1;
                pend_pix       <= 1'b0;
                ack_toggle_pix <= req_seen_pix;
            end
        end
    end
endmodule
