`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// media_player_control —— 控制层：负责“什么时候做”
// -----------------------------------------------------------------------------
module media_player_control #(
    parameter [31:0] DEFAULT_PERIOD_CYCLES = 32'd250_000_000,   // 2.5s @100MHz
    parameter [31:0] MIN_PERIOD_CYCLES     = 32'd50_000_000,    // 0.5s
    parameter [31:0] MAX_PERIOD_CYCLES     = 32'd3_000_000_000, // 30s
    parameter integer LOOKAHEAD            = 0,                 
    parameter integer PERIOD_W             = 32                 // 周期 / 轮播计数位宽（须容纳 MAX_PERIOD_CYCLES）
) (
    // ---------------- sys_clk 域 ----------------
    input  wire        sys_clk,
    input  wire        sys_rst_n,
    input  wire        tick,                // 轮播计时时基（顶层 1ms 节拍；接 1 = 按时钟周期计）
    input  wire        goto_valid,          // 策略层：切到 goto_image_idx
    input  wire [5:0]  goto_image_idx,
    input  wire [1:0]  goto_audio_idx,      // 策略层：与该图联动的音轨
    input  wire        toggle_auto_event,
    input  wire        period_cfg_valid,
    input  wire [31:0] period_cfg_cycles,
    output reg         auto_tick,           // 轮播到点（单拍）→ 策略层

    input  wire        scan_done,
    input  wire [6:0]  image_count,
    input  wire        loader_ready,
    input  wire        load_done,
    input  wire [5:0]  loaded_image_idx,
    input  wire [1:0]  loaded_buffer_idx,
    input  wire [7:0]  load_error,          // 单拍非零
    input  wire [3:0]  image_complete,      
    input  wire        fast_view,           

    output reg         load_request,
    output reg  [5:0]  load_image_idx,
    output reg  [1:0]  load_buffer_idx,
    output reg         load_thumb_gen,      
    output reg         auto_enabled,
    output wire [31:0] active_period_cycles,
    output reg  [5:0]  current_image_idx,   // 已提交（ack 回来）的图片
    output reg         image_committed,     // 至少有一张图成功提交过（fast_boot_gate 只需要“任意一张”）
    output reg         display_valid_sys,
    output reg  [7:0]  control_error,       // 粘滞，下一次成功提交清零
    output reg         error_event,         // 本拍报出新错误（单拍）
    output wire        request_busy,
    output wire [3:0]  bank_resident,       
    output wire        view_hold,           
    output reg         load_cancel,        

    // ---------------- pix_clk 域 ----------------
    input  wire        pix_clk,
    input  wire        pix_rst_n,
    input  wire        frame_boundary,      // 帧边界（单拍）
    output reg  [5:0]  active_image_idx,
    output reg  [1:0]  active_buffer_idx,
    output reg  [1:0]  active_audio_idx,    // 与 active_image_idx 同一帧边界切换
    output reg         display_valid,
    output reg         index_commit
);
    localparam [7:0] ERR_NONE          = 8'h00;
    localparam [7:0] ERR_PERIOD_RANGE  = 8'h51;
    localparam [7:0] ERR_REQUEST_BUSY  = 8'h52;
    localparam [7:0] ERR_NO_RESOURCE   = 8'h53;
    localparam [7:0] ERR_LOAD_MISMATCH = 8'h54;

    // ============================ sys_clk 域 ============================
    reg [PERIOD_W-1:0] auto_counter;
    reg [PERIOD_W-1:0] period_q;
    assign active_period_cycles = period_q;            // 高位补 0
    reg        scan_done_d;
    reg        req_pending;
    reg [5:0]  req_image_idx;
    reg [1:0]  req_audio_idx;
    reg [1:0]  load_audio_idx;       // 在途加载对应的音轨
    reg        transfer_busy;
    reg        load_is_request;      // 在途加载完成后要提交显示（用户/开机加载）；0 = 后台预加载
    reg [1:0]  current_buffer_idx;
    reg [5:0]  boot_image_idx;       // 首图尝试序号：坏图就换下一张
    reg        boot_exhausted;
    reg [6:0]  image_count_d;        

    // bank 表
    reg [3:0]  bank_valid;
    reg [5:0]  bank_img0, bank_img1, bank_img2, bank_img3;
    reg [1:0]  preload_ptr;
    reg [3:0]  preload_failed;

    reg [5:0]  commit_image_idx_sys;
    reg [1:0]  commit_buffer_idx_sys;
    reg [1:0]  commit_audio_idx_sys;
    reg        commit_toggle_sys;
    (* async_reg = "true" *) reg ack_sync1;
    (* async_reg = "true" *) reg ack_sync2;
    reg        ack_seen_sys;
    reg        ack_toggle_pix;

    wire commit_busy      = (commit_toggle_sys != ack_seen_sys);
    wire commit_ack_event = (ack_sync2 != ack_seen_sys);
    wire load_done_ok     = load_done &&
                            (loaded_image_idx  == load_image_idx) &&
                            (loaded_buffer_idx == load_buffer_idx);
    wire user_busy        = (transfer_busy && load_is_request) || commit_busy;

    // ---- bank 命中：挂起请求的图已经整张在某个 bank 里 ----
    wire [3:0] bank_ok = bank_valid & image_complete;
    wire [3:0] req_hit_vec = { bank_ok[3] && (bank_img3 == req_image_idx),
                               bank_ok[2] && (bank_img2 == req_image_idx),
                               bank_ok[1] && (bank_img1 == req_image_idx),
                               bank_ok[0] && (bank_img0 == req_image_idx) };
    wire       req_hit      = |req_hit_vec;
    wire [1:0] req_hit_bank = req_hit_vec[0] ? 2'd0 : req_hit_vec[1] ? 2'd1 :
                              req_hit_vec[2] ? 2'd2 : 2'd3;
    // ---- 未命中：写 home bank，home 正在显示就写 home^1 ----
    wire [1:0] req_home     = req_image_idx[1:0];
    wire [1:0] req_victim   = (req_home == current_buffer_idx) ? (req_home ^ 2'd1) : req_home;
    // ---- 后台预加载候选：第 preload_ptr 张图 → bank preload_ptr（空 bank、不在显示、没失败过）----
    wire       preload_ok   = ({5'd0, preload_ptr} < image_count) &&
                              !bank_valid[preload_ptr] && !preload_failed[preload_ptr] &&
                              (preload_ptr != current_buffer_idx);
    // ================= 前瞻缓存（LOOKAHEAD=1 才用到，=0 时综合裁掉）=================
    reg        la_dir;               // 1 = 顺方向是"下一张"
    reg [3:0]  cand_ptr;             // 0..10 候选指针
    reg [10:0] cand_fail;            // 后台加载失败的候选（提交时清）
    reg [3:0]  bg_slot;              // 在途后台加载对应的候选
    reg [7:0]  th_v;                 // 缩略图记账：条目 {img[2], img[1:0]} 有缩略图
    reg [2:0]  th_t0, th_t1, th_t2, th_t3, th_t4, th_t5, th_t6, th_t7;   // 条目里是第几页（img[5:3]）
    reg        cur_unres;            // 当前图是总览快速提交的（current_buffer 里还是上一张驻留图）
    reg        cur_bad;              // 快速提交的当前图后台加载失败 → 退回 current_buffer 里的图
    reg        bg_stale;             // 在途后台加载发起后又提交过（它的候选号已过期，失败不记账）
    reg        bg_cur;               // 在途后台加载 = 补当前图的全尺寸图
    reg        keep_err;             // 在途提交是坏图退回：应答时不清错误码
    reg  [5:0] res_img;              // current_buffer 里驻留的图（一串快速提交的第一次时锁存；坏图退回它）
    reg        bg_cancel;            // 已对在途后台加载发过取消（完成 / 0x59 回来清）

    wire       fast_ok  = (LOOKAHEAD != 0) && fast_view;

    wire [5:0] la_next  = (image_count <= 7'd1 || ({1'b0, current_image_idx} + 7'd1 >= image_count)) ?
                          6'd0 : current_image_idx + 6'd1;
    wire [5:0] la_prev  = (image_count <= 7'd1) ? 6'd0 :
                          (current_image_idx == 6'd0) ? image_count[5:0] - 6'd1 : current_image_idx - 6'd1;
    wire [5:0] la_a     = la_dir ? la_next : la_prev;      // 顺方向
    wire [5:0] la_b     = la_dir ? la_prev : la_next;      // 反方向
    wire [5:0] la_a2    = la_dir ? (({1'b0, la_next} + 7'd1 >= image_count) ? 6'd0 : la_next + 6'd1)   // 顺方向第二张
                                 : ((la_prev == 6'd0) ? image_count[5:0] - 6'd1 : la_prev - 6'd1);
    wire [6:0] cnt_m1   = image_count - 7'd1;
    wire [3:0] pg_cur   = current_image_idx[5:2];
    wire [3:0] pg_last  = cnt_m1[5:2];
    wire [3:0] pg_nb    = la_dir ? ((pg_cur == pg_last) ? 4'd0 : pg_cur + 4'd1)
                                 : ((pg_cur == 4'd0) ? pg_last : pg_cur - 4'd1);
    wire       cand_res_type = (cand_ptr <= 4'd1) || (cand_ptr == 4'd10);  // 1 = 要驻留，0 = 要缩略图
    reg  [5:0] cand_img;
    always @* begin
        case (cand_ptr)
            4'd0:    cand_img = (fast_ok || cur_unres) ? current_image_idx : la_a;   // 总览：先补当前图
            4'd1:    cand_img = la_a2;
            4'd2:    cand_img = {pg_cur, 2'd0};
            4'd3:    cand_img = {pg_cur, 2'd1};
            4'd4:    cand_img = {pg_cur, 2'd2};
            4'd5:    cand_img = {pg_cur, 2'd3};
            4'd6:    cand_img = {pg_nb, 2'd0};
            4'd7:    cand_img = {pg_nb, 2'd1};
            4'd8:    cand_img = {pg_nb, 2'd2};
            4'd9:    cand_img = {pg_nb, 2'd3};
            default: cand_img = la_b;
        endcase
    end
    // 在哪些 bank 里（整张完整）
    function [3:0] hit_vec(input [5:0] img, input [3:0] ok,
                           input [5:0] b0, input [5:0] b1, input [5:0] b2, input [5:0] b3);
        hit_vec = {ok[3] && (b3 == img), ok[2] && (b2 == img), ok[1] && (b1 == img), ok[0] && (b0 == img)};
    endfunction
    // 缩略图记账查询：该图的缩略图完好
    function th_have(input [5:0] img, input [7:0] v,
                     input [2:0] t0, input [2:0] t1, input [2:0] t2, input [2:0] t3,
                     input [2:0] t4, input [2:0] t5, input [2:0] t6, input [2:0] t7);
        reg [2:0] t;
        begin
            case (img[2:0])
                3'd0: t = t0; 3'd1: t = t1; 3'd2: t = t2; 3'd3: t = t3;
                3'd4: t = t4; 3'd5: t = t5; 3'd6: t = t6; default: t = t7;
            endcase
            th_have = v[img[2:0]] && (t == img[5:3]);
        end
    endfunction
    wire [3:0] cand_hit  = hit_vec(cand_img, bank_ok, bank_img0, bank_img1, bank_img2, bank_img3);
    wire       cand_thok = th_have(cand_img, th_v, th_t0, th_t1, th_t2, th_t3, th_t4, th_t5, th_t6, th_t7);
    // 后台写缩略图不能盖掉当前页：只写当前页的格子，或另一块区（页号奇偶不同）。
    //   页数为奇数时末页与第 0 页同奇偶（如 10 张：第 2 页 ↔ 第 0 页），回绕邻页就不预生成，否则两页互相覆盖、反复重写
    //   只给当前页 / 邻页写：B 在上一页时，上一页与邻页（下一页）同一块区、同一格（如 7 与 15），
    //   B 顺带写缩略图会盖掉邻页格子，补邻页又借走 B 的 bank → 反复互相覆盖（24 张仿真抓到）
    wire       cand_safe = ((cand_img[5:2] == pg_cur) || (cand_img[2] != pg_cur[0])) &&
                           ((cand_img[5:2] == pg_cur) || (cand_img[5:2] == pg_nb));
    wire       cand_gen  = !cand_thok && cand_safe;
    // 写哪个 bank：当前显示 / A / A2（全屏）/ B 所在的 bank 不动，其余里优先不完整（空）的；
    //   全屏 4 个 bank 正好是滑动窗口 {B, 当前, A, A2}，没有空闲时缩略图加载借 B 的 bank（B 排在候选最后补回），
    //   当前 / A / A2 永远不挤 → 缩略图加载在途时连按，下一张、下下张仍命中
    wire [3:0] a_hit_v   = hit_vec(la_a,  bank_ok, bank_img0, bank_img1, bank_img2, bank_img3);
    wire [3:0] a2_hit_v  = hit_vec(la_a2, bank_ok, bank_img0, bank_img1, bank_img2, bank_img3);
    wire [3:0] la_core   = (4'b0001 << current_buffer_idx) | a_hit_v | (fast_ok ? 4'd0 : a2_hit_v);
    wire [3:0] la_prot   = la_core | hit_vec(la_b, bank_ok, bank_img0, bank_img1, bank_img2, bank_img3);
    wire [3:0] la_free   = ~la_prot;
    wire [3:0] la_empty  = la_free & ~bank_ok;
    wire [3:0] la_pick_v = (|la_empty) ? la_empty : (|la_free) ? la_free : ~la_core;   // la_core 至多 3 位
    // 总览中 A2 / B 不驻留：总览切图不需要全尺寸图，读卡时间给当前图和缩略图（候选 0 = 当前图，补全尺寸图）
    // 全屏：A、A2 先驻留（读时顺带写缩略图），驻留好了缩略图加载立即开始
    wire       cand_need = ({1'b0, cand_img} < image_count) && (image_count > 7'd1) && !cand_fail[cand_ptr] &&
                           !(fast_ok && ((cand_ptr == 4'd1) || (cand_ptr == 4'd10))) &&
                           (cand_res_type ? !(|cand_hit) : cand_gen);
    wire [1:0] la_victim = la_pick_v[0] ? 2'd0 : la_pick_v[1] ? 2'd1 : la_pick_v[2] ? 2'd2 : 2'd3;
    wire       req_thok  = th_have(req_image_idx, th_v, th_t0, th_t1, th_t2, th_t3, th_t4, th_t5, th_t6, th_t7);
    wire       boot_thok = th_have(boot_image_idx, th_v, th_t0, th_t1, th_t2, th_t3, th_t4, th_t5, th_t6, th_t7);

    // 提交与“加载完成提交”同拍冲突时让加载完成优先（下一拍再处理命中）
    wire       load_commit_now = transfer_busy && scan_done && load_done_ok && load_is_request;

    wire period_in_range  = (period_cfg_cycles >= MIN_PERIOD_CYCLES) &&
                            (period_cfg_cycles <= MAX_PERIOD_CYCLES);
    wire auto_resource_ok = scan_done && (image_count > 7'd1);
    wire goto_resource_ok = scan_done && display_valid_sys && (image_count > 7'd1) &&
                            ({1'b0, goto_image_idx} < image_count);

    // LOOKAHEAD 下后台加载几乎一直在跑，用户切图常要挂起排在它后面（req_pending）；
    //   挂起也算忙，上游按"忙时丢弃"规则丢键，否则下一次按键的 goto 撞上挂起 → 0x52（上板实测的 error）
    assign request_busy  = user_busy || ((LOOKAHEAD != 0) && req_pending);
    assign bank_resident = bank_ok;
    assign view_hold     = cur_unres;
    // 当前图（快速提交）的全尺寸图已在某个 bank → 补一次提交换到真 bank。
    //   cur_unres 时候选 0 就是当前图，直接复用候选的 cand_hit（不另做 4 个比较器）；
    //   候选指针空闲时一直轮转、加载完成会回 0，所以最多几拍就会看到
    wire       cur_res_now  = cur_unres && (cand_ptr == 4'd0) && (|cand_hit);
    wire [1:0] cur_hit_bank = cand_hit[0] ? 2'd0 : cand_hit[1] ? 2'd1 : cand_hit[2] ? 2'd2 : 2'd3;
    wire       commit_free = scan_done && display_valid_sys && !commit_busy && !commit_ack_event && !load_commit_now;
    // 快速提交的当前图后台加载失败（候选未过期）
    wire       cur_fail_now = (LOOKAHEAD != 0) && transfer_busy && !load_is_request && bg_cur && !bg_stale &&
                              cur_unres && !load_done_ok && (load_done || (load_error != ERR_NONE));
    // 在途后台加载可取消（见文件头 (a)(b)）
    wire       ld_pg_ok     = (load_image_idx[5:2] == pg_cur) || (load_image_idx[5:2] == pg_nb);
    wire       cancel_user  = !fast_ok && req_pending && !req_hit && (load_image_idx != req_image_idx);
    wire       cancel_stale = bg_stale && (fast_ok ? !ld_pg_ok :
                              ((load_image_idx != la_a) && (load_image_idx != la_a2) && (!ld_pg_ok || !(|a_hit_v))));
    wire       cancel_now   = (LOOKAHEAD != 0) && transfer_busy && !load_is_request && !(bg_cur && !bg_stale) &&
                              !bg_cancel && !load_request && !load_done && (load_error == ERR_NONE) &&
                              (cancel_user || cancel_stale);

    // 同一拍可能有多个来源报错：组合逻辑按优先级（越靠后越优先）选出一个，
    // 时序块里只用非阻塞赋值
    reg [7:0] err_next;
    reg       err_set;
    always @* begin
        err_set  = 1'b0;
        err_next = ERR_NONE;
        if (scan_done && !scan_done_d && (image_count == 7'd0)) begin
            err_set = 1'b1; err_next = ERR_NO_RESOURCE;
        end
        if (period_cfg_valid && !period_in_range) begin
            err_set = 1'b1; err_next = ERR_PERIOD_RANGE;
        end
        if (toggle_auto_event && !auto_resource_ok) begin
            err_set = 1'b1; err_next = ERR_NO_RESOURCE;
        end
        if (goto_valid) begin
            if (!goto_resource_ok) begin
                err_set = 1'b1; err_next = ERR_NO_RESOURCE;
            end else if (req_pending) begin
                err_set = 1'b1; err_next = ERR_REQUEST_BUSY;
            end
        end
        // 只有用户/开机加载的失败才上报；后台预加载失败静默记账
        // 总览快速提交的当前图坏了，同用户加载失败一样上报（随后退回上一张）
        if ((transfer_busy && load_is_request && !load_done_ok) || cur_fail_now) begin
            if (load_done) begin
                err_set = 1'b1; err_next = ERR_LOAD_MISMATCH;
            end else if (load_error != ERR_NONE) begin
                err_set = 1'b1; err_next = load_error;
            end
        end
    end

    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            load_request          <= 1'b0;
            load_image_idx        <= 6'd0;
            load_buffer_idx       <= 2'd0;
            load_thumb_gen        <= 1'b1;
            auto_enabled          <= 1'b0;
            period_q              <= DEFAULT_PERIOD_CYCLES[PERIOD_W-1:0];
            current_image_idx     <= 6'd0;
            image_committed       <= 1'b0;
            current_buffer_idx    <= 2'd0;
            display_valid_sys     <= 1'b0;
            control_error         <= ERR_NONE;
            error_event           <= 1'b0;
            auto_tick             <= 1'b0;
            auto_counter          <= {PERIOD_W{1'b0}};
            scan_done_d           <= 1'b0;
            req_pending           <= 1'b0;
            req_image_idx         <= 6'd0;
            req_audio_idx         <= 2'd0;
            load_audio_idx        <= 2'd0;
            transfer_busy         <= 1'b0;
            load_is_request       <= 1'b0;
            boot_image_idx        <= 6'd0;
            boot_exhausted        <= 1'b0;
            image_count_d         <= 7'd0;
            bank_valid            <= 4'd0;
            bank_img0             <= 6'd0;
            bank_img1             <= 6'd0;
            bank_img2             <= 6'd0;
            bank_img3             <= 6'd0;
            preload_ptr           <= 2'd0;
            preload_failed        <= 4'd0;
            commit_image_idx_sys  <= 6'd0;
            commit_buffer_idx_sys <= 2'd0;
            commit_audio_idx_sys  <= 2'd0;
            commit_toggle_sys     <= 1'b0;
            ack_sync1             <= 1'b0;
            ack_sync2             <= 1'b0;
            ack_seen_sys          <= 1'b0;
            la_dir                <= 1'b1;
            cand_ptr              <= 4'd0;
            cand_fail             <= 11'd0;
            bg_slot               <= 4'd0;
            th_v                  <= 8'd0;
            th_t0 <= 3'd0; th_t1 <= 3'd0; th_t2 <= 3'd0; th_t3 <= 3'd0;
            th_t4 <= 3'd0; th_t5 <= 3'd0; th_t6 <= 3'd0; th_t7 <= 3'd0;
            cur_unres             <= 1'b0;
            cur_bad               <= 1'b0;
            bg_stale              <= 1'b0;
            bg_cur                <= 1'b0;
            keep_err              <= 1'b0;
            res_img               <= 6'd0;
            load_cancel           <= 1'b0;
            bg_cancel             <= 1'b0;
        end else begin
            load_request <= 1'b0;
            auto_tick    <= 1'b0;
            load_cancel  <= 1'b0;
            error_event  <= err_set;
            scan_done_d  <= scan_done;
            image_count_d <= image_count;
            ack_sync1    <= ack_toggle_pix;
            ack_sync2    <= ack_sync1;

            // TF 卡重新初始化 → loader 丢弃发现结果和在途加载，播放器回到开机状态
            if (!scan_done) begin
                req_pending    <= 1'b0;
                transfer_busy  <= 1'b0;
                boot_image_idx <= 6'd0;
                boot_exhausted <= 1'b0;
                auto_enabled   <= 1'b0;
                image_committed <= 1'b0;
                bank_valid     <= 4'd0;
                preload_failed <= 4'd0;
                th_v           <= 8'd0;        // 缩略图记账随重新扫描清零
                cand_fail      <= 11'd0;
                cur_unres      <= 1'b0;
                cur_bad        <= 1'b0;
                bg_cancel      <= 1'b0;
            end

            //取消在途后台加载（loader 回 0x59 后下面的完成分支收尾，发起块再读更急的图）
            if (cancel_now) begin
                load_cancel <= 1'b1;
                bg_cancel   <= 1'b1;
            end

            // 2.7 快速启动：首图阶段已有图全部失败时，后台扫描又确认了新图 → 接着试新图
            if (scan_done && boot_exhausted && !display_valid_sys && (image_count > image_count_d)) begin
                boot_exhausted <= 1'b0;
                boot_image_idx <= image_count_d[5:0];
            end

            // -------- 提交完成（pix 域 ack 回来）--------
            if (commit_ack_event) begin
                ack_seen_sys       <= ack_sync2;
                current_image_idx  <= commit_image_idx_sys;
                current_buffer_idx <= commit_buffer_idx_sys;
                display_valid_sys  <= 1'b1;
                image_committed    <= 1'b1;
                cand_ptr           <= 4'd0;     // 换了当前图，候选从头检查
                cand_fail          <= 11'd0;
                bg_stale           <= 1'b1;     // 在途后台加载的候选号已过期
            end

            // 错误码粘滞；成功提交清零；同拍新错误优先
            if (err_set)
                control_error <= err_next;
            else if (commit_ack_event && !keep_err)
                control_error <= ERR_NONE;           // 坏图退回的那次提交不清错误码（同"切图失败显示不变"）
            if (commit_ack_event)
                keep_err <= 1'b0;

            // -------- 运行时周期配置 --------
            if (period_cfg_valid && period_in_range)
                period_q <= period_cfg_cycles[PERIOD_W-1:0];

            // -------- 自动轮播开关 --------
            if (toggle_auto_event && auto_resource_ok)
                auto_enabled <= ~auto_enabled;

            // -------- 切图请求：只挂起一个 --------
            if (goto_valid && goto_resource_ok && !req_pending) begin
                req_pending   <= 1'b1;
                req_image_idx <= goto_image_idx;
                req_audio_idx <= goto_audio_idx;
                // 切到的是上一张（且上一张 ≠ 下一张）= 反方向，其余（下一张 / 轮播 / 直接跳）= 顺方向
                la_dir        <= !((goto_image_idx == la_prev) && (la_prev != la_next));
            end

            // -------- 轮播计时：只在“空闲且在显示”时计，任何操作都重新计 --------
            // 倒计数：空闲时装入周期值，减到 1 那拍出 auto_tick（共 period 个 tick），省掉减法器和比较器
            if (auto_enabled && display_valid_sys && !request_busy && !req_pending &&
                !goto_valid && !toggle_auto_event && !period_cfg_valid) begin
                if (!tick) begin
                    auto_counter <= auto_counter;
                end else if (auto_counter <= 1) begin
                    auto_counter <= period_q;
                    auto_tick    <= 1'b1;
                end else begin
                    auto_counter <= auto_counter - 1'b1;
                end
            end else begin
                auto_counter <= period_q;
            end

            // -------- 命中 → 不加载，直接提交（零延迟切换）--------
            if (scan_done && display_valid_sys && req_pending && req_hit &&
                !commit_busy && !commit_ack_event && !load_commit_now) begin
                commit_image_idx_sys  <= req_image_idx;
                commit_buffer_idx_sys <= req_hit_bank;
                commit_audio_idx_sys  <= req_audio_idx;
                commit_toggle_sys     <= ~commit_toggle_sys;
                req_pending           <= 1'b0;
                load_is_request       <= 1'b0;       // 在途的用户加载（如果有）被这次切换取代，完成后只入表
                cur_unres             <= 1'b0;
            end else if (req_pending && fast_ok && commit_free) begin
                // 总览快速提交：总览读的是缩略图区，全尺寸图不在屏上 → 不等加载，bank 不变直接提交，
                //   全尺寸图由前瞻候选 0 在后台补（回全屏前 view_hold 拦住 SW4 拨下）
                commit_image_idx_sys  <= req_image_idx;
                commit_buffer_idx_sys <= current_buffer_idx;
                commit_audio_idx_sys  <= req_audio_idx;
                commit_toggle_sys     <= ~commit_toggle_sys;
                req_pending           <= 1'b0;
                cur_unres             <= 1'b1;
                cur_bad               <= 1'b0;
                if (!cur_unres) res_img <= current_image_idx;   // 此刻它一定整张在 current_buffer
            end else if (!req_pending && commit_free && (cur_bad || cur_res_now)) begin
                // 全尺寸图到了 → 同一张图换到真 bank 再提交一次（总览页内提交，不触发转场）；
                // 坏图 → 退回 current_buffer 里驻留的那一张
                commit_image_idx_sys  <= cur_bad ? res_img : current_image_idx;
                commit_buffer_idx_sys <= cur_bad ? current_buffer_idx : cur_hit_bank;
                if (cur_bad) commit_audio_idx_sys <= res_img[1:0];   // 音轨按图片号低 2 位（同开机 / 前瞻）
                keep_err              <= cur_bad;
                commit_toggle_sys     <= ~commit_toggle_sys;
                cur_unres             <= 1'b0;
                cur_bad               <= 1'b0;
            end

            // -------- 发起加载（一次只有一个；提交未应答时不发，保证 current_buffer_idx 已稳定）--------
            if (scan_done && !transfer_busy && !commit_busy && !commit_ack_event &&
                loader_ready && !load_request) begin
                if (!display_valid_sys) begin
                    // 首图：写它的 home bank（此时读端输出被 display_valid 压黑）
                    if ((image_count != 7'd0) && !boot_exhausted) begin
                        load_image_idx  <= boot_image_idx;
                        load_audio_idx  <= boot_image_idx[1:0]; // 音轨按 4 路循环
                        load_buffer_idx <= boot_image_idx[1:0];
                        load_request    <= 1'b1;
                        transfer_busy   <= 1'b1;
                        load_is_request <= 1'b1;
                        bank_valid[boot_image_idx[1:0]] <= 1'b0;
                        if (LOOKAHEAD != 0) begin
                            load_thumb_gen <= !boot_thok;
                            if (!boot_thok) th_v[boot_image_idx[2:0]] <= 1'b0;
                        end
                    end
                end else if (req_pending) begin
                    if (!req_hit && !fast_ok) begin      // 总览中由快速提交处理，不加载
                        load_image_idx  <= req_image_idx;
                        load_audio_idx  <= req_audio_idx;
                        load_buffer_idx <= (LOOKAHEAD != 0) ? la_victim : req_victim;
                        load_request    <= 1'b1;
                        transfer_busy   <= 1'b1;
                        load_is_request <= 1'b1;
                        req_pending     <= 1'b0;
                        if (LOOKAHEAD != 0) begin
                            bank_valid[la_victim] <= 1'b0;
                            load_thumb_gen <= !req_thok;
                            if (!req_thok) th_v[req_image_idx[2:0]] <= 1'b0;
                        end else
                            bank_valid[req_victim] <= 1'b0;
                    end
                end else if (LOOKAHEAD != 0) begin
                    // 前瞻：当前候选需要就发后台加载，否则看下一个候选
                    if (cand_need) begin
                        load_image_idx  <= cand_img;
                        load_audio_idx  <= cand_img[1:0];
                        load_buffer_idx <= la_victim;
                        load_request    <= 1'b1;
                        transfer_busy   <= 1'b1;
                        load_is_request <= 1'b0;
                        bank_valid[la_victim] <= 1'b0;
                        bg_slot         <= cand_ptr;
                        bg_stale        <= 1'b0;
                        bg_cur          <= (cand_ptr == 4'd0) && (fast_ok || cur_unres);
                        load_thumb_gen  <= cand_gen;
                        if (cand_gen) th_v[cand_img[2:0]] <= 1'b0;
                    end else begin
                        cand_ptr <= (cand_ptr == 4'd10) ? 4'd0 : cand_ptr + 4'd1;
                    end
                end else if (preload_ok) begin
                    load_image_idx  <= {4'd0, preload_ptr};
                    load_audio_idx  <= preload_ptr;
                    load_buffer_idx <= preload_ptr;
                    load_request    <= 1'b1;
                    transfer_busy   <= 1'b1;
                    load_is_request <= 1'b0;
                    bank_valid[preload_ptr] <= 1'b0;
                end else begin
                    preload_ptr <= preload_ptr + 2'd1;   // 轮询下一个候选 bank
                end
            end

            // -------- 加载完成 → 入 bank 表；用户/开机加载再发提交（载荷与 toggle 同拍写入，保持到 ack）--------
            if (transfer_busy && scan_done) begin
                if (load_done_ok) begin
                    transfer_busy <= 1'b0;
                    bg_cancel     <= 1'b0;          // 取消没赶上（loader 已在排空）：照常入表
                    bank_valid[load_buffer_idx] <= 1'b1;
                    case (load_buffer_idx)
                        2'd0: bank_img0 <= load_image_idx;
                        2'd1: bank_img1 <= load_image_idx;
                        2'd2: bank_img2 <= load_image_idx;
                        default: bank_img3 <= load_image_idx;
                    endcase
                    if (LOOKAHEAD != 0) begin
                        cand_ptr <= 4'd0;
                        if (load_thumb_gen) begin
                            th_v[load_image_idx[2:0]] <= 1'b1;
                            case (load_image_idx[2:0])
                                3'd0: th_t0 <= load_image_idx[5:3];
                                3'd1: th_t1 <= load_image_idx[5:3];
                                3'd2: th_t2 <= load_image_idx[5:3];
                                3'd3: th_t3 <= load_image_idx[5:3];
                                3'd4: th_t4 <= load_image_idx[5:3];
                                3'd5: th_t5 <= load_image_idx[5:3];
                                3'd6: th_t6 <= load_image_idx[5:3];
                                default: th_t7 <= load_image_idx[5:3];
                            endcase
                        end
                    end
                    if (load_is_request) begin
                        commit_image_idx_sys  <= load_image_idx;
                        commit_buffer_idx_sys <= load_buffer_idx;
                        commit_audio_idx_sys  <= load_audio_idx;
                        commit_toggle_sys     <= ~commit_toggle_sys;
                    end
                end else if (load_done || (load_error != ERR_NONE)) begin
                    transfer_busy <= 1'b0;
                    bg_cancel     <= 1'b0;
                    if (!load_is_request)
                        preload_failed[load_buffer_idx] <= 1'b1;
                    if ((LOOKAHEAD != 0) && !load_is_request && !bg_stale && !bg_cancel)
                        cand_fail[bg_slot] <= 1'b1;      // 本轮不再重试这个候选（过期的候选号 / 被取消的不记）
                    if (cur_fail_now)
                        cur_bad <= 1'b1;                 // 快速提交的当前图坏了 → 退回上一张
                    if (load_is_request && !display_valid_sys && !load_done) begin
                        // 首图坏了：试下一张，全部失败就停下来（错误码保留）
                        if ({1'b0, boot_image_idx} + 7'd1 >= image_count)
                            boot_exhausted <= 1'b1;
                        else
                            boot_image_idx <= boot_image_idx + 6'd1;
                    end
                end
            end
        end
    end

    // ============================ pix_clk 域 ============================
    (* async_reg = "true" *) reg commit_sync1;
    (* async_reg = "true" *) reg commit_sync2;
    reg       commit_seen_pix;
    reg       commit_pending_pix;
    reg [5:0] requested_image_pix;
    reg [1:0] requested_buffer_pix;
    reg [1:0] requested_audio_pix;

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            commit_sync1         <= 1'b0;
            commit_sync2         <= 1'b0;
            commit_seen_pix      <= 1'b0;
            commit_pending_pix   <= 1'b0;
            requested_image_pix  <= 6'd0;
            requested_buffer_pix <= 2'd0;
            requested_audio_pix  <= 2'd0;
            ack_toggle_pix       <= 1'b0;
            active_image_idx     <= 6'd0;
            active_buffer_idx    <= 2'd0;
            active_audio_idx     <= 2'd0;
            display_valid        <= 1'b0;
            index_commit         <= 1'b0;
        end else begin
            commit_sync1 <= commit_toggle_sys;
            commit_sync2 <= commit_sync1;
            index_commit <= 1'b0;

            // 载荷与 toggle 在 sys 域同拍写入，并一直保持到 ack 回来；toggle 经
            // 两级同步才被看到，此时载荷已稳定 ≥2 个 pix 周期，直接采样安全（准静态总线）
            if ((commit_sync2 != commit_seen_pix) && !commit_pending_pix) begin
                commit_seen_pix      <= commit_sync2;
                requested_image_pix  <= commit_image_idx_sys;
                requested_buffer_pix <= commit_buffer_idx_sys;
                requested_audio_pix  <= commit_audio_idx_sys;
                commit_pending_pix   <= 1'b1;
            end

            if (frame_boundary && commit_pending_pix) begin
                active_image_idx   <= requested_image_pix;
                active_buffer_idx  <= requested_buffer_pix;
                active_audio_idx   <= requested_audio_pix;
                display_valid      <= 1'b1;
                index_commit       <= 1'b1;
                commit_pending_pix <= 1'b0;
                ack_toggle_pix     <= commit_seen_pix;
            end
        end
    end
endmodule
