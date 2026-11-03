`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// media_policy —— 策略层：决定“下一张切到哪张图”、“图片和音频怎么联动”
//
// 输入是已经路由好的“语义事件”（单击/长按/组合键），输出是给控制层的命令和
// 给显示层的用户设置。本模块不关心加载时序、缓冲区、跨时钟域。
//
//   KEY2 单击          next_event    → 下一张      goto (cur+1) mod N
//   KEY2 长按 1s       prev_event    → 上一张      goto (cur-1) mod N
//   KEY3 单击          auto_event    → 自动轮播开/关
//   KEY3 长按 0.5s     bright_event  → 亮度阶梯 100→75→50→25→100 %（ 8 位：255/191/127/63，(b+1)/256 正好是这四档）
//   KEY2+KEY3 长按 3s  save_event    → 保存当前设置快照 → player_config_persistence 写 Flash
//   KEY4 单击 (扩展)   vol_event     → 音量阶梯 +VOL_STEP，到 255 后回 0
//   KEY4 长按 (扩展)   period_event  → 轮播周期阶梯 0.5/1/2.5/5/10/30 s
//   轮播到点           auto_tick     → 与“下一张”同一条策略
//   画面调节模式（picture_adjust_ctrl）：adj_inc / adj_dec 当前项 ±BC_STEP（饱和）、adj_default 恢复默认，
//                      adj_sel 0 = 亮度（默认 255）/ 1 = 对比度（默认 128）；对比度与亮度一起存 Flash
//
// 图片与音频联动：
//   每张图片绑定一条音轨 audio_for_image(img)；切图命令同时带上目标音轨
//   goto_audio_idx，由控制层与图片在同一个帧边界一起提交 → 画面和声音同步切换。
//
// enable = fast_boot_gate.player_enable_fast：首图显示之前丢弃所有事件。
// “下一张”以已提交的 current_image_idx 为基准计算，连按不会累积成连跳。
//
// 配置持久化：cfg_load_valid（上电从 Flash 读到有效记录，单拍）→ 周期/亮度/音量
//   立即生效（不受 enable 门控，周期经 period_cfg_valid 下发控制层）；自动轮播模式要等
//   播放器放行且图片数 > 1 才能打开 → 挂起 auto_restore_pending，条件满足时发一次
//   toggle_auto_event；恢复前用户自己按了 KEY3 则取消恢复。保存快照同时记录自动模式。
// 外接传感器（扩展）：DEFAULT_AUTO = 1 时 Flash 无记录也在首图后自动开轮播（复位时恢复挂起 = 1，
//   有 Flash 记录则按记录）；ir_present 电平变化 → 遮挡 = 关自动（手动浏览）、移开 = 开自动，KEY3 仍可随时开关。
//   外接光照：亮度改存"偏移"bri_offset（有符号 9 位），显示亮度 = 限幅(环境基准 + 偏移)（ambient_brightness 算）。
//   按键 ±BC_STEP / 阶梯作用在显示亮度上再换算回偏移，"默认" = 偏移 0（跟随环境）；BRI_ENV_RESET = 1 时
//   env_change（新的环境亮度生效）清零偏移。无传感器时基准 255：偏移 = 旧 8 位亮度 - 255，行为与原来一致。
// -----------------------------------------------------------------------------
module media_policy #(
    parameter integer CLK_HZ             = 100_000_000,
    parameter [2:0]   DEFAULT_PERIOD_SEL = 3'd2,         // 2.5s
    parameter [7:0]   DEFAULT_VOLUME     = 8'd128,
    parameter [7:0]   VOL_STEP           = 8'd32,
    parameter [7:0]   BC_STEP            = 8'd16,         // 亮度/对比度细调步长
    parameter         DEFAULT_AUTO       = 1'b0,          // 外接传感器：1 = 上电（Flash 无记录时）默认开自动轮播
    parameter         BRI_ENV_RESET      = 1'b0          // 外接光照：1 = 新的环境亮度生效时清零按键偏移（显示回到该环境对应的亮度）
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        enable,

    // 语义事件（单拍）
    input  wire        next_event,
    input  wire        prev_event,
    input  wire        auto_event,
    input  wire        bright_event,
    input  wire        vol_event,
    input  wire        period_event,
    input  wire        save_event,
    input  wire        auto_tick,
    // 画面调节（picture_adjust_ctrl，单拍）
    input  wire        adj_inc,
    input  wire        adj_dec,
    input  wire        adj_default,
    input  wire        adj_sel,           // 0 = 亮度，1 = 对比度
    input  wire        adj_show,          // 进入调节 / 切换项：数码管切到对应页
    // 外接红外：1 = 有遮挡（已消抖电平）
    input  wire        ir_present,
    // 外接光照（ambient_brightness）：显示亮度 = 限幅(base + bri_offset)
    input  wire [7:0]  disp_brightness,   // 当前真实显示亮度
    input  wire [7:0]  bri_base,          // 当前环境基准（无传感器 = 255）
    input  wire        env_change,        // 单拍：新的环境亮度开始生效

    // 控制层反馈
    input  wire [5:0]  current_image_idx,
    input  wire [6:0]  image_count,
    input  wire        auto_enabled,      // 控制层当前自动轮播状态（保存快照 / 恢复判断）

    // 上电从 Flash 加载的配置（单拍）
    input  wire        cfg_load_valid,
    input  wire [2:0]  cfg_period_sel,
    input  wire [8:0]  cfg_brightness,    // 外接传感器：亮度偏移（有符号；v1/v2 记录由 persistence 换算）
    input  wire [7:0]  cfg_contrast,
    input  wire [7:0]  cfg_volume,
    input  wire        cfg_auto_mode,

    // → 控制层命令
    output reg         goto_valid,
    output reg  [5:0]  goto_image_idx,
    output reg  [1:0]  goto_audio_idx,
    output reg         toggle_auto_event,
    output reg         period_cfg_valid,
    output wire [31:0] period_cfg_cycles, // = cycles_lut(period_sel)，period_cfg_valid 那拍被采用

    // → 显示层 / 音频
    output reg  [5:0]  media_id,          // 最近一次请求的目标图片
    output reg  [1:0]  audio_id,          // 最近一次请求的目标音轨
    output reg         linked_event,      // 图片 + 音频一起切换（单拍）
    output reg  [8:0]  bri_offset,        // 外接传感器：亮度偏移（有符号 9 位，0 = 跟随环境；无传感器时 = 亮度 - 255）
    output reg  [7:0]  contrast,          // 0..255，128 = 正常
    output reg  [7:0]  volume,            // 0..255
    output wire [8:0]  period_tenths,     // 周期，单位 0.1s（5..300）= tenths_lut(period_sel)
    output reg         pic_event,         // 数码管强制 PIC 页
    output reg         vol_page_event,    // 数码管强制 VOL 页
    output reg         prd_event,         // 数码管强制 PRD 页
    output reg         bri_event,         // 数码管强制 BRI 事件页
    output reg         con_event,         // 数码管强制 CON 事件页
    output reg         save_done_event,   // 保存请求 → player_config_persistence

    // 要保存的设置 = 当前值（存全部设置：不再另存快照，persistence 开工那拍锁存一次）
    output wire [2:0]  saved_period_sel,
    output wire [8:0]  saved_brightness,  // 亮度偏移
    output wire [7:0]  saved_contrast,
    output wire [7:0]  saved_volume,
    output wire        saved_auto_mode,
    output reg         saved_valid        // 保存 / 加载过（仅观测）
);
    localparam [31:0] CYC_PER_TENTH = CLK_HZ / 10;

    reg [2:0] period_sel;
    reg       auto_restore_pending;
    reg       ir_prev;               // 上一次处理过的红外电平（只在 enable 期间更新）
    reg       ir_req;                // 红外请求的自动轮播状态还没落实
    reg       ir_want_on;            // 红外请求：1 = 开自动轮播，0 = 关

    function [8:0] tenths_lut;
        input [2:0] sel;
        begin
            case (sel)
                3'd0: tenths_lut = 9'd5;
                3'd1: tenths_lut = 9'd10;
                3'd2: tenths_lut = 9'd25;
                3'd3: tenths_lut = 9'd50;
                3'd4: tenths_lut = 9'd100;
                default: tenths_lut = 9'd300;
            endcase
        end
    endfunction

    function [31:0] cycles_lut;
        input [2:0] sel;
        begin
            case (sel)
                3'd0: cycles_lut = CYC_PER_TENTH * 5;
                3'd1: cycles_lut = CYC_PER_TENTH * 10;
                3'd2: cycles_lut = CYC_PER_TENTH * 25;
                3'd3: cycles_lut = CYC_PER_TENTH * 50;
                3'd4: cycles_lut = CYC_PER_TENTH * 100;
                default: cycles_lut = CYC_PER_TENTH * 300;
            endcase
        end
    endfunction

    function [5:0] wrap_next;
        input [5:0] cur;
        input [6:0] count;
        begin
            if (count <= 7'd1 || ({1'b0, cur} + 7'd1 >= count))
                wrap_next = 6'd0;
            else
                wrap_next = cur + 6'd1;
        end
    endfunction

    function [5:0] wrap_prev;
        input [5:0] cur;
        input [6:0] count;
        begin
            if (count <= 7'd1)
                wrap_prev = 6'd0;
            else if (cur == 6'd0)
                wrap_prev = count[5:0] - 6'd1;   // count=64 → 63
            else
                wrap_prev = cur - 6'd1;
        end
    endfunction

    // 联动规则：图片 n 播放音轨 n。改联动方式只改这里。
    function [1:0] audio_for_image;
        input [5:0] img;
        begin
            audio_for_image = img[1:0];
        end
    endfunction

    function [7:0] volume_next;
        input [7:0] v;
        begin
            if (v == 8'd255)
                volume_next = 8'd0;
            else if ({1'b0, v} + {1'b0, VOL_STEP} >= 9'd255)
                volume_next = 8'd255;
            else
                volume_next = v + VOL_STEP;
        end
    endfunction

    // KEY3 长按亮度阶梯：降到下一个更低的档位（255→191→127→63→255），细调后的值也落回阶梯
    function [7:0] bright_ladder_next;
        input [7:0] b;
        begin
            if (b > 8'd191)      bright_ladder_next = 8'd191;
            else if (b > 8'd127) bright_ladder_next = 8'd127;
            else if (b > 8'd63)  bright_ladder_next = 8'd63;
            else                 bright_ladder_next = 8'd255;
        end
    endfunction

    //  细调：±BC_STEP，饱和在 0 / 255（不回绕）
    function [7:0] step_sat;
        input [7:0] v;
        input       up;
        begin
            if (up)
                step_sat = ({1'b0, v} + {1'b0, BC_STEP} > 9'd255) ? 8'd255 : v + BC_STEP;
            else
                step_sat = (v < BC_STEP) ? 8'd0 : v - BC_STEP;
        end
    endfunction

    wire       adj_step  = adj_inc | adj_dec;
    wire [7:0] adj_value = adj_default ? 8'd128 : step_sat(contrast, adj_inc);       // 对比度
    // 亮度：按键改的是"显示亮度"（±BC_STEP 饱和 / 阶梯），再换算回偏移 = 新显示值 - 环境基准；默认 = 偏移 0（跟随环境）
    wire [7:0] bri_target = bright_event ? bright_ladder_next(disp_brightness) : step_sat(disp_brightness, adj_inc);
    wire [8:0] bri_off_new = (adj_default && !bright_event) ? 9'd0 : {1'b0, bri_target} - {1'b0, bri_base};

    wire [2:0] period_sel_next = (period_sel >= 3'd5) ? 3'd0 : period_sel + 3'd1;
    wire [5:0] next_img = wrap_next(current_image_idx, image_count);
    wire [5:0] prev_img = wrap_prev(current_image_idx, image_count);

    // 周期两种单位直接由 period_sel 查表（原各存一份寄存器，与 period_sel 同拍更新，值逐拍相同）
    assign period_tenths     = tenths_lut(period_sel);
    assign period_cfg_cycles = cycles_lut(period_sel);

    assign saved_period_sel = period_sel;
    assign saved_brightness = bri_offset;
    assign saved_contrast   = contrast;
    assign saved_volume     = volume;
    assign saved_auto_mode  = auto_enabled | auto_restore_pending;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            goto_valid        <= 1'b0;
            goto_image_idx    <= 6'd0;
            goto_audio_idx    <= 2'd0;
            toggle_auto_event <= 1'b0;
            period_cfg_valid  <= 1'b0;
            media_id          <= 6'd0;
            audio_id          <= 2'd0;
            linked_event      <= 1'b0;
            bri_offset        <= 9'd0;
            contrast          <= 8'd128;
            volume            <= DEFAULT_VOLUME;
            period_sel        <= DEFAULT_PERIOD_SEL;
            pic_event         <= 1'b0;
            vol_page_event    <= 1'b0;
            prd_event         <= 1'b0;
            bri_event         <= 1'b0;
            con_event         <= 1'b0;
            save_done_event   <= 1'b0;
            saved_valid       <= 1'b0;
            auto_restore_pending <= DEFAULT_AUTO;
            ir_prev           <= 1'b0;
            ir_req            <= 1'b0;
            ir_want_on        <= 1'b0;
        end else begin
            goto_valid        <= 1'b0;
            toggle_auto_event <= 1'b0;
            period_cfg_valid  <= 1'b0;
            linked_event      <= 1'b0;
            pic_event         <= 1'b0;
            vol_page_event    <= 1'b0;
            prd_event         <= 1'b0;
            bri_event         <= 1'b0;
            con_event         <= 1'b0;
            save_done_event   <= 1'b0;

            // 上电加载 Flash 配置（启动期间就生效，不等 enable）
            if (cfg_load_valid) begin
                period_sel           <= (cfg_period_sel > 3'd5) ? 3'd5 : cfg_period_sel;
                period_cfg_valid     <= 1'b1;
                bri_offset           <= cfg_brightness;
                contrast             <= cfg_contrast;
                volume               <= cfg_volume;
                saved_valid          <= 1'b1;
                auto_restore_pending <= cfg_auto_mode;
            end

            // 外接光照：新的环境亮度生效 → 清掉按键偏移，显示亮度跟到新环境对应的值（同拍按键调节在后面覆盖）
            if (BRI_ENV_RESET && env_change)
                bri_offset <= 9'd0;

            if (enable) begin
                // 恢复自动轮播：已打开 / 用户先按了 KEY3 → 放弃；图片数 > 1 才发（否则控制层报 0x53）
                if (auto_restore_pending && (ir_present == ir_prev)) begin   // 红外同拍变化时让红外决定
                    if (auto_event || auto_enabled)
                        auto_restore_pending <= 1'b0;
                    else if (image_count > 7'd1) begin
                        toggle_auto_event    <= 1'b1;
                        auto_restore_pending <= 1'b0;
                    end
                end

                // 外接红外：遮挡沿 → 请求关自动轮播（手动），移开沿 → 请求开自动轮播。只在电平变化时
                //   请求一次，之后 KEY3 照常可以开 / 关（不会被红外电平一直压着）。请求在没有切换在途
                //   （toggle_auto_event 已落到控制层、auto_enabled 已更新）时才对照当前状态决定是否翻转；
                //   同拍按了 KEY3 以按键为准；开轮播要等图片数 > 1（同上面的恢复规则）。
                if (ir_present != ir_prev) begin
                    ir_prev              <= ir_present;
                    ir_req               <= 1'b1;
                    ir_want_on           <= ~ir_present;
                    auto_restore_pending <= 1'b0;
                end else if (ir_req && !toggle_auto_event) begin
                    if (auto_event || (auto_enabled == ir_want_on))
                        ir_req <= 1'b0;
                    else if (!ir_want_on || image_count > 7'd1) begin
                        toggle_auto_event <= 1'b1;
                        pic_event         <= 1'b1;
                        ir_req            <= 1'b0;
                    end
                end

                // 切图：下一张（按键/轮播）优先于上一张；图片和音轨一起下发
                if (next_event || auto_tick) begin
                    goto_valid     <= 1'b1;
                    goto_image_idx <= next_img;
                    goto_audio_idx <= audio_for_image(next_img);
                    media_id       <= next_img;
                    audio_id       <= audio_for_image(next_img);
                    linked_event   <= 1'b1;
                    pic_event      <= 1'b1;
                end else if (prev_event) begin
                    goto_valid     <= 1'b1;
                    goto_image_idx <= prev_img;
                    goto_audio_idx <= audio_for_image(prev_img);
                    media_id       <= prev_img;
                    audio_id       <= audio_for_image(prev_img);
                    linked_event   <= 1'b1;
                    pic_event      <= 1'b1;
                end

                if (auto_event) begin
                    toggle_auto_event <= 1'b1;
                    pic_event         <= 1'b1;
                end

                if (bright_event) begin
                    bri_offset <= bri_off_new;
                    bri_event  <= 1'b1;
                end

                // 画面调节模式（与 bright_event 互斥：模式内 KEY3 事件不会再给阶梯）
                if (adj_step || adj_default) begin
                    if (adj_sel)
                        contrast   <= adj_value;
                    else
                        bri_offset <= bri_off_new;
                end
                if (adj_step || adj_default || adj_show) begin
                    bri_event <= ~adj_sel;
                    con_event <=  adj_sel;
                end

                if (vol_event) begin
                    volume         <= volume_next(volume);
                    vol_page_event <= 1'b1;
                end

                if (period_event) begin
                    period_sel        <= period_sel_next;
                    period_cfg_valid  <= 1'b1;
                    prd_event         <= 1'b1;
                end

                if (save_event) begin
                    saved_valid      <= 1'b1;
                    save_done_event  <= 1'b1;
                end
            end
        end
    end
endmodule
