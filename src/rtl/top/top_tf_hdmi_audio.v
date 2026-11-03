module top(
    input                       clk,
    input                       key1_rst_n,     // KEY1 A2：复位
    input                       key2_n,         // KEY2 B2
    input                       key3_n,         // KEY3 B1
    input                       key4_n,         // KEY4 C1
    input  [3:0]                sw,            
    output [7:0]                seg_sel,
    output [7:0]                seg_data,

    // HDMI TMDS
    output                      HDMI_CLK_P,
    output                      HDMI_D2_P,
    output                      HDMI_D1_P,
    output                      HDMI_D0_P,

    // HDMI DDC
    output                      HDMI_DDC_SCL,
    inout                       HDMI_DDC_SDA,

    // 外接传感器（全部 3.3V 供电！模块 VCC 禁止接 5V）：J1 光照 GY-302/BH1750、J2 红外避障 YL-62
    output                      als_scl,        // J1-31 M15（开漏，0 = 拉低 / Z = 释放）
    inout                       als_sda,        // J1-32 L16（开漏）
    input                       ir_out_n,       // J2-31 J12（低 = 有遮挡）

    // TF card SPI
    output                      sd_ncs,
    output                      sd_dclk,
    output                      sd_mosi,
    input                       sd_miso,

    // 用户 Flash W25Q64（U33）SPI：配置持久化
    output                      flash_cs_n,     // P8
    output                      flash_sclk,     // M9
    output                      flash_mosi,     // P7  FLASH_SDI
    input                       flash_miso,     // N8  FLASH_SDO
    output                      flash_wp_n,     // P9
    output                      flash_hold_n    // R9
);

parameter MEM_DATA_BITS = 32;
parameter ADDR_BITS     = 21;
parameter BUSRT_BITS    = 10;
parameter FRAME_PIXELS  = 24'd307200;   // 640*480，每个 bank 一张图

// 每张图的滚动字幕（PIC1~4 依次对应；最多 12 个字符：空格、0-9、A-Z、: . , ! ? - _ /）
parameter [95:0] SUBTITLE_CAPTION0 = "ANLOGIC FPGA";
parameter [95:0] SUBTITLE_CAPTION1 = "EG4S20 HDMI";
parameter [95:0] SUBTITLE_CAPTION2 = "OSD SUBTITLE";
parameter [95:0] SUBTITLE_CAPTION3 = "ALPHA BLEND";

// 转场：每段（淡出 / 淡入，手动淡入淡出与自动方块溶解共用）帧数，须整除 256；16 帧 ≈ 0.27 s @60fps。0 = 旁路（硬切）
parameter integer TRANSITION_FRAMES = 16;

localparam integer SYS_CLK_HZ = 50_000_000;    // sys_clk = 板载 50MHz 晶振：应用层控制时钟

// ============================ 时钟============================
wire sys_clk = clk;
wire pixel_clk_5x;
wire mem_clk;
wire mem_clk_shift;
wire pixel_clk;
wire mem_locked;

sys_pll_promax_40 u_mem_pll (
    .refclk   (sys_clk),
    .reset    (~key1_rst_n),      // KEY1 按下时 PLL 也复位，松开后重新锁定（PLLRST_ENA=ENABLE）
    .clk0_out (pixel_clk_5x),
    .clk1_out (mem_clk),
    .clk2_out (mem_clk_shift),
    .clk3_out (pixel_clk),
    .clk4_out (),
    .locked   (mem_locked)
);

//音频由 25MHz 像素时钟分数分频得到 48kHz 采样节拍，不需要音频 PLL

// ============================ 复位============================
// 复位源 = KEY1 & PLL 锁定（任何时刻失锁都重新压下复位）；PLL 锁定过程远长于电源稳定，
// 不需要 POR 计数。每个时钟域一个 reset_sync：异步压下、本域两拍同步释放。
// 全部模块低有效复位，直接接 *_rst_n；只有加密 IP/原语(SDRAM 核、HDMI DDC 核、ODDR、
// 异步 FIFO)保留高有效，在实例处取反。
wire ext_rst_n = key1_rst_n & mem_locked;

wire ctrl_rst_n;    // sys_clk 域
wire mem_rst_n;     // mem_clk 域
wire pix_rst_n;     // pixel_clk 域
wire pix_5x_rst_n;  // pixel_clk_5x 域（10:1 串化）

reset_sync u_ctrl_reset   (.clk(sys_clk),      .ext_rst_n(ext_rst_n), .rst_n_sync(ctrl_rst_n));
reset_sync u_mem_reset    (.clk(mem_clk),      .ext_rst_n(ext_rst_n), .rst_n_sync(mem_rst_n));
reset_sync u_pix_reset    (.clk(pixel_clk),    .ext_rst_n(ext_rst_n), .rst_n_sync(pix_rst_n));
reset_sync u_pix_5x_reset (.clk(pixel_clk_5x), .ext_rst_n(ext_rst_n), .rst_n_sync(pix_5x_rst_n));

// ============================ 按键处理层 ============================
wire key2_click;
wire key2_long;
wire key3_click;
wire key3_long;
wire save_chord;
wire key4_click;
wire key4_long;
wire key3_stable_n;       // 消抖后电平 → picture_adjust_ctrl 组合键检测
wire key4_stable_n;
wire player_enable_fast;

// ----导览运镜：跨段使用的线（模块实例在视频时序段之后）----
wire        key2_click_raw, key2_long_raw, key4_click_raw, key4_long_raw;
wire        key2_stable_n;
wire        tour_chord, tour_next, tour_busy;
wire        arb_rd_grant;
wire        tv_dma_want, tv_dma_grant, tv_dma_busy, tv_dma_own, tv_app_rd_en;
wire [20:0] tv_app_rd_addr;
wire        tour_on_pix;
wire [23:0] tour_rgb;
wire [80:0] tour_bundle_pix;
wire        sm_map_valid;
wire [11:0] sm_c0, sm_r0b, sm_cw, sm_ch, sm_n;
wire [9:0]  sm_d;
wire        sc_plan_valid;
wire [9:0]  sc_g_ox, sc_g_d;
wire [8:0]  sc_g_oyt;
wire [11:0] sc_g_c0, sc_g_r0t, sc_g_cw, sc_g_ch, sc_g_n;
wire        focus_done;
wire        fl_loc_req, fl_sec_req;
wire [5:0]  fl_loc_idx;
wire [31:0] fl_sec_lba;
wire        sl_loc_req, sl_sec_req;
wire [5:0]  sl_loc_idx;
wire [31:0] sl_sec_lba;

//公共 1ms 节拍（bh1750_reader 产生，sys 域单拍），所有毫秒级定时器只在这一拍计数
wire        sensor_tick_ms;

dual_key_chord_router #(
    .DEBOUNCE_CYCLES  (20),               // 20ms（L2：单位 = 1ms tick）
    .HOLD_CYCLES      (3000),             // 双键同按 3s
    .KEY3_LONG_CYCLES (500),              // KEY3 长按 0.5s
    .KEY2_LONG_CYCLES (1000)              // KEY2 长按 1s
) u_key_router (
    .clk             (sys_clk),
    .rst_n           (ctrl_rst_n),
    .tick            (sensor_tick_ms),
    .save_enabled    (1'b1),
    .key2_n          (key2_n),
    .key3_n          (key3_n),
    .key2_event      (key2_click_raw),
    .key2_long_event (key2_long_raw),
    .key3_event      (key3_click),
    .key3_long_event (key3_long),
    .save_event      (save_chord),
    .save_locked     (),
    .key2_stable_n   (key2_stable_n),             
    .key3_stable_n   (key3_stable_n)
);

single_key_event #(
    .DEBOUNCE_CYCLES (20),                 // 1ms tick 数
    .LONG_CYCLES     (500)
) u_key4 (
    .clk          (sys_clk),
    .rst_n        (ctrl_rst_n),
    .tick         (sensor_tick_ms),
    .key_n        (key4_n),
    .click_event  (key4_click_raw),
    .long_event   (key4_long_raw),
    .key_stable_n (key4_stable_n)
);

// KEY2+KEY4 同按 = 进入 / 退出导览（同按期间两键的单击 / 长按全部吞掉）；
//   导览中 KEY2 单击 = 下一个焦点、KEY2 长按吞掉；导览外所有事件原样透传（下游仍用 key2_click 等原名）
tour_chord_filter u_tour_keys (
    .clk            (sys_clk),
    .rst_n          (ctrl_rst_n),
    .tour_on        (tour_busy),
    .key2_stable_n  (key2_stable_n),
    .key4_stable_n  (key4_stable_n),
    .key2_click_in  (key2_click_raw),
    .key2_long_in   (key2_long_raw),
    .key4_click_in  (key4_click_raw),
    .key4_long_in   (key4_long_raw),
    .key2_click_out (key2_click),
    .key2_long_out  (key2_long),
    .key4_click_out (key4_click),
    .key4_long_out  (key4_long),
    .chord          (tour_chord),
    .tour_next      (tour_next)
);

wire        sw4_stable_n;        // 0 = SW4 拨上（稳定后）
single_key_event #(
    .DEBOUNCE_CYCLES (20),                 // 1ms tick 数
    .LONG_CYCLES     (2)                   // 不用单击 / 长按事件，只用消抖电平
) u_sw4_debounce (
    .clk          (sys_clk),
    .rst_n        (ctrl_rst_n),
    .tick         (sensor_tick_ms),
    .key_n        (~sw[3]),
    .click_event  (),
    .long_event   (),
    .key_stable_n (sw4_stable_n)
);
reg  sw4_stable_n_q;
always @(posedge sys_clk or negedge ctrl_rst_n) begin
    if (!ctrl_rst_n)
        sw4_stable_n_q <= 1'b1;
    else
        sw4_stable_n_q <= sw4_stable_n;
end
wire        thumb_key_mode    =  sw4_stable_n_q && !sw4_stable_n;   // 拨上
wire        thumb_key_confirm = !sw4_stable_n_q &&  sw4_stable_n;   // 拨下 = 选定焦点图、回全屏
wire        thumb_nav_enable;    // 0 = 总览进 / 出正在等待生效，KEY2 暂不接收

// ============================ 控制策略层 ============================
wire        scan_done;
wire        scan_all_done;       // 整卡扫描结束（fast_boot_gate.ingest_done）
wire [6:0]  image_count;
wire        loader_ready;
wire        load_done;
wire [5:0]  loaded_image_idx;
wire [1:0]  loaded_buffer_idx;
wire [7:0]  load_error;
wire        load_cancel;        
wire        load_request;
wire [5:0]  load_image_idx;
wire [1:0]  load_buffer_idx;
wire        load_thumb_gen;      

wire        goto_valid;
wire [5:0]  goto_image_idx;
wire [1:0]  goto_audio_idx;
wire        toggle_auto_event;
wire        period_cfg_valid;
wire [31:0] period_cfg_cycles;
wire        auto_tick;
wire        auto_enabled;
wire [31:0] active_period_cycles;
wire [5:0]  current_image_idx;
wire        display_valid_sys;
wire [7:0]  control_error;
wire        error_event;
wire        request_busy;

wire [5:0]  media_id;
wire [1:0]  audio_id;
wire        linked_event;
wire [8:0]  bri_offset;          // 外接传感器：亮度偏移（有符号），显示亮度 = 限幅(环境基准 + 偏移)
wire [7:0]  contrast;            
wire [7:0]  volume;
wire [8:0]  period_tenths;
wire        pic_event;
wire        vol_page_event;
wire        bri_event;
wire        con_event;
wire        prd_event;
wire        save_done_event;     // media_policy：保存快照已更新 → 请求写 Flash
wire [2:0]  saved_period_sel;
wire [8:0]  saved_brightness;    // 亮度偏移
wire [7:0]  saved_contrast;
wire [7:0]  saved_volume;
wire        saved_auto_mode;
wire        cfg_load_valid;
wire [2:0]  cfg_period_sel;
wire [8:0]  cfg_brightness;      // 亮度偏移
wire [7:0]  cfg_contrast;
wire [7:0]  cfg_volume;
wire        cfg_auto_mode;
wire        cfg_ui_hide;         // KEY3 长按隐藏底部跑马灯 / 频谱柱
wire        cfg_save_ok;        // Flash 写入 + 读回校验成功 → 数码管 SAVE 页
wire        cfg_error_event;
wire [7:0]  cfg_error_code;

wire        frame_start;
wire        blank_start;
wire        frame_start_raw;     
wire [3:0]  fb_image_complete_sys;
wire [5:0]  active_image_idx;
wire [1:0]  active_buffer_idx;
wire [1:0]  active_audio_idx;
wire        display_valid;

wire        image_committed;     // 至少一张图已提交显示（fast_boot_gate）
wire        ingest_enable;       // boot_sequencer 输出
wire        player_enable;
wire        sd_ready;
wire [7:0]  boot_error;
wire        boot_error_event;
wire [2:0]  boot_state;
wire        first_image_ready;
wire [7:0]  sd_fs_error;         

// 扫描确认第一张图 → scan_done → 控制层立即加载首图；首图一提交就放行
// 按键/轮播；剩余图片在后台继续扫描（scan_all_done 之前 image_count 可能继续增长）
fast_boot_gate u_fast_boot_gate (
    .clk                (sys_clk),
    .rst_n              (ctrl_rst_n),
    .ingest_done        (scan_all_done),
    .image_complete     (image_committed),
    .player_enable_raw  (player_enable),       // boot_sequencer 放行（已含首图确认）
    .player_enable_fast (player_enable_fast),
    .first_image_ready  (first_image_ready)
);

// 事件先进 video_fade_transition：旧图淡出到全黑 → 才放出 fade_next_event（标签 = 上一张）给策略层
// → 控制层帧边界提交 → index_commit → 新图淡入。
// 自动轮播（2026-10-18）：auto_tick 不再直接进策略层，同按键一样先进转场控制器（next_auto_in=1）
//   → 旧图随机方块溶解到全黑 → 才放出"下一张" → 提交 → 新图方块溶解亮起；全程每帧只变 1/16 的块。
//   总览页内的轮播（焦点框移动、不换页）不进转场，直接交给策略层（同 thumb_direct_next）。
//   next_block_in = request_busy：控制层已有用户切换在途时丢弃按键（在途切换会在淡出中途硬切）
//   hold_abort_in = error_event：策略/控制层拒绝或加载失败 → 结束全黑，淡入（仍是旧图）
//   转场忙 / 总览进出未生效 / 导览中到点的轮播丢弃：控制层到点已重装周期，下个周期再轮播
wire        index_commit;        // media_player_control（pix 域）：帧边界提交新图（单拍）
wire        fade_next_event;
wire        fade_next_is_prev;
wire        fade_busy;
wire [7:0]  fade_alpha;
wire        fade_active;
wire        dissolve_active;     // 自动轮播的方块溶解进行中（淡出 / 淡入段）
wire [4:0]  fade_ramp;

// 总览翻页规则（sys 域）：总览里 KEY2 不跨页 = 直接进 media_policy（焦点框立即移动，不淡出），
//   跨页（下一张回到第 0 格 / 回绕，上一张离开第 0 格）才走转场控制器淡出 → 新页 → 淡入。
//   只在模式请求值已在 pix 域完全生效（缩放动画走完，thumb_view_settled 同步回 sys 后与请求值相等）时接 KEY2：
//   ① pix 域 fade_index_commit 的屏蔽条件（thumb_view_pix）与这里一致：走转场的切图一定会被转场控制器
//      看到提交，HOLD 不会空等；② 进 / 出总览的 0.4 s 缩放动画期间 KEY2 丢弃（与"忙时丢弃"原规则一致），
//      淡入淡出不会和缩放叠在一起。焦点握手不影响按键。
//   切图 / 转场在途时总览里的页内 KEY2 丢弃（与 next_block_in = request_busy 的原规则一致）。
wire        thumb_view_sys;      // u_thumb_ctrl 请求值（sys 域）
wire        thumb_view_pix;      // pix 域生效的模式（只在 frame_start 变化）
reg         thumb_view_settled;  // pix 域：缩放动画走完后的模式（单比特寄存器，见缩放转场处）
wire        thumb_view_back;     // thumb_view_settled 同步回 sys 域
wire        thumb_view_hold;     // media_player_control：总览快速提交的当前图还没有全尺寸图
sync_2ff u_thumb_view_back (
    .clk      (sys_clk),
    .rst_n    (ctrl_rst_n),
    .async_in (thumb_view_settled),
    .sync_out (thumb_view_back)
);
wire        thumb_nav_ok    = thumb_nav_enable && (thumb_view_sys == thumb_view_back);
wire        thumb_in_view   = thumb_view_sys;
wire [6:0]  cur_plus1       = {1'b0, current_image_idx} + 7'd1;
wire        thumb_next_cross = (image_count > 7'd4) && ((current_image_idx[1:0] == 2'd3) || (cur_plus1 >= image_count));
wire        thumb_prev_cross = (image_count > 7'd4) && (current_image_idx[1:0] == 2'd0);
wire        thumb_key_in    = thumb_in_view && ((key2_click && !thumb_next_cross) || (key2_long && !thumb_prev_cross));
wire        thumb_direct_ok = thumb_nav_ok && !request_busy && !fade_busy;
wire        thumb_direct_next = key2_click && thumb_in_view && !thumb_next_cross && thumb_direct_ok;
wire        thumb_direct_prev = key2_long  && thumb_in_view && !thumb_prev_cross && thumb_direct_ok;
wire        fade_index_commit;   // pix 域：总览中页内提交不进转场控制器
// 轮播到点的分流：总览页内（不跨页）= 直通策略层；其余 = 转场控制器方块溶解。按键同拍时按键优先、不算自动
wire        auto_req     = auto_tick && thumb_nav_ok && !tour_busy;   
wire        auto_in_page = thumb_in_view && !thumb_next_cross;
wire        auto_direct  = auto_req && auto_in_page && !fade_busy;
wire        key_fade_ev  = (key2_click | key2_long) & thumb_nav_ok & ~thumb_key_in;   //总览进 / 出生效前不接；总览页内另走直通

video_fade_transition #(
    .FADE_FRAMES         (TRANSITION_FRAMES),
    .HOLD_TIMEOUT_FRAMES (480)                // 全黑最多 8 s 兜底（加载超时 2 s × 重试 1 次 + 加载时间）
) u_fade_transition (
    .ctrl_clk        (sys_clk),
    .ctrl_rst_n      (ctrl_rst_n),
    .next_event_in   (key_fade_ev | (auto_req & ~auto_in_page)),
    .next_tag_in     (key2_long),
    .next_auto_in    (~key_fade_ev),
    .next_block_in   (request_busy),
    .hold_abort_in   (error_event),
    .next_event_out  (fade_next_event),
    .next_tag_out    (fade_next_is_prev),
    .fade_busy       (fade_busy),
    .pix_clk         (pixel_clk),
    .pix_rst_n       (pix_rst_n),
    .player_active   (display_valid),
    .frame_start     (frame_start_raw),       // 与控制层提交、reader 帧请求同一个边界（场同步下降沿）
    .index_commit    (fade_index_commit),
    .alpha_out       (fade_alpha),
    .fade_active     (fade_active),
    .dissolve_active (dissolve_active),
    .ramp_out        (fade_ramp)
);

// ---- 画面调节模式：KEY3+KEY4 同按进入/退出；模式外 KEY3/KEY4 事件原样交给 media_policy ----
wire key3_click_f, key3_long_f, key4_click_f, key4_long_f;
wire adj_mode, adj_sel, adj_inc, adj_dec, adj_default, adj_show;
picture_adjust_ctrl #(
    .IDLE_EXIT_MS (5000)
) u_picture_adjust (
    .clk            (sys_clk),
    .rst_n          (ctrl_rst_n),
    .tick           (sensor_tick_ms),
    .enable         (player_enable_fast),
    .key3_stable_n  (key3_stable_n),
    .key4_stable_n  (key4_stable_n),
    .key3_click_in  (key3_click),
    .key3_long_in   (key3_long),
    .key4_click_in  (key4_click),
    .key4_long_in   (key4_long),
    .key3_click_out (key3_click_f),
    .key3_long_out  (key3_long_f),
    .key4_click_out (key4_click_f),
    .key4_long_out  (key4_long_f),
    .adj_mode       (adj_mode),
    .adj_sel        (adj_sel),
    .adj_inc        (adj_inc),
    .adj_dec        (adj_dec),
    .adj_default    (adj_default),
    .adj_show       (adj_show)
);

// trk16：KEY3 长按（模式外，经调节模式过滤）= 隐藏 / 显示底部跑马灯 + 频谱柱
//   存全部设置：KEY2+KEY3 保存时一起写 Flash，上电读到有效记录就恢复；无记录 = 显示
reg ui_hide_sys;
always @(posedge sys_clk or negedge ctrl_rst_n) begin
    if (!ctrl_rst_n)
        ui_hide_sys <= 1'b0;
    else if (cfg_load_valid)
        ui_hide_sys <= cfg_ui_hide;
    else if (key3_long_f)
        ui_hide_sys <= ~ui_hide_sys;
end

// ---- 外接传感器（sys 域）：BH1750 环境光 → 自适应亮度；YL-62 红外遮挡 → 手动 / 自动轮播 ----
//   bh1750_reader：100kHz 开漏 IIC，每 0.5 s 单次高分辨率测量（测完芯片自动掉电），顺带给出公共 1ms 节拍
//   ambient_brightness：eff = 限幅(base + bri_offset)（对数映射 + 迟滞 + 斜坡渐变，一个加法器，无乘法），
//     eff 送像素链路、参数条、调节面板、数码管；env_change → media_policy 清零偏移。传感器没接 → base 255，与原来逐位相同
//   ir_presence_filter：2FF + 50ms 消抖 → media_policy（电平变化时请求一次，KEY3 照常可开关）
wire        als_scl_low, als_sda_low;
wire        als_raw_valid;
wire [15:0] als_raw;
wire        als_ok;
wire [7:0]  eff_brightness;      // 真实显示亮度 → 像素链路 / 参数条 BRT / 调节面板 / 数码管
wire [7:0]  als_base;
wire        als_env_change;
wire        ir_present;
assign als_scl = als_scl_low ? 1'b0 : 1'bz;
assign als_sda = als_sda_low ? 1'b0 : 1'bz;

bh1750_reader #(
    .CLK_HZ    (SYS_CLK_HZ),
    .IIC_HZ    (100_000),
    .DEV_ADDR  (7'h23),                 // ADDR 悬空（模块下拉）
    .MEAS_MS   (180),
    .PERIOD_MS (500)
) u_als (
    .clk       (sys_clk),
    .rst_n     (ctrl_rst_n),
    .scl_o_low (als_scl_low),
    .sda_o_low (als_sda_low),
    .sda_i     (als_sda),
    .tick_ms   (sensor_tick_ms),
    .raw_valid (als_raw_valid),
    .raw       (als_raw),
    .sensor_ok (als_ok)
);

ambient_brightness #(
    .MIN_BASE (64),
    .L_LO     (12),
    .HYST     (8),
    .RAMP_MS  (8)
) u_auto_bright (
    .clk             (sys_clk),
    .rst_n           (ctrl_rst_n),
    .tick_ms         (sensor_tick_ms),
    .sensor_ok       (als_ok),
    .raw_valid       (als_raw_valid),
    .raw             (als_raw),
    .offset          (bri_offset),
    .base            (als_base),
    .eff_brightness  (eff_brightness),
    .env_change      (als_env_change)
);

ir_presence_filter #(
    .STABLE_MS (50)
) u_ir_filter (
    .clk      (sys_clk),
    .rst_n    (ctrl_rst_n),
    .tick_ms  (sensor_tick_ms),
    .ir_out_n (ir_out_n),
    .present  (ir_present)
);

media_policy #(
    .CLK_HZ         (1000),                    // period_cfg_cycles 单位 = 1ms tick（播放器轮播只在 tick 计数）
    .DEFAULT_VOLUME (8'd128),
    .VOL_STEP       (8'd32),
    .BC_STEP        (8'd16),
    .DEFAULT_PERIOD_SEL (3'd3),                // 外接传感器：上电默认 5 s
    .DEFAULT_AUTO   (1'b1),                     // 外接传感器：Flash 无记录时上电默认自动轮播
    .BRI_ENV_RESET  (1'b1)                     // 外接光照：新环境亮度生效时回到该环境对应的亮度（0 = 一直保留按键偏移）
) u_media_policy (
    .clk               (sys_clk),
    .rst_n             (ctrl_rst_n),
    .enable            (player_enable_fast),
    .next_event        ((fade_next_event & ~fade_next_is_prev) | thumb_direct_next),   //经转场控制器
    .prev_event        ((fade_next_event &  fade_next_is_prev) | thumb_direct_prev),
    .auto_event        (key3_click_f),                           // 经组合键 / 调节模式过滤
    .bright_event      (1'b0),                                   // KEY3 长按改为讲解人声开关
    .vol_event         (key4_click_f),
    .period_event      (key4_long_f),
    .save_event        (save_chord),
    .auto_tick         (auto_direct),                            // 总览页内轮播直通；其余轮播经转场控制器（next_event）
    .adj_inc           (adj_inc),
    .adj_dec           (adj_dec),
    .adj_default       (adj_default),
    .adj_sel           (adj_sel),
    .adj_show          (adj_show),
    .ir_present        (ir_present),                             // 外接红外：遮挡 = 手动、移开 = 自动
    .disp_brightness   (eff_brightness),
    .bri_base          (als_base),
    .env_change        (als_env_change),                         // 新环境亮度生效 → 清零按键偏移
    .current_image_idx (current_image_idx),
    .image_count       (image_count),
    .auto_enabled      (auto_enabled),
    .cfg_load_valid    (cfg_load_valid),
    .cfg_period_sel    (cfg_period_sel),
    .cfg_brightness    (cfg_brightness),
    .cfg_contrast      (cfg_contrast),
    .cfg_volume        (cfg_volume),
    .cfg_auto_mode     (cfg_auto_mode),
    .goto_valid        (goto_valid),
    .goto_image_idx    (goto_image_idx),
    .goto_audio_idx    (goto_audio_idx),
    .toggle_auto_event (toggle_auto_event),
    .period_cfg_valid  (period_cfg_valid),
    .period_cfg_cycles (period_cfg_cycles),
    .media_id          (media_id),
    .audio_id          (audio_id),
    .linked_event      (linked_event),
    .bri_offset        (bri_offset),
    .contrast          (contrast),
    .volume            (volume),
    .period_tenths     (period_tenths),
    .pic_event         (pic_event),
    .vol_page_event    (vol_page_event),
    .prd_event         (prd_event),
    .bri_event         (bri_event),
    .con_event         (con_event),
    .save_done_event   (save_done_event),
    .saved_period_sel  (saved_period_sel),
    .saved_brightness  (saved_brightness),
    .saved_contrast    (saved_contrast),
    .saved_volume      (saved_volume),
    .saved_auto_mode   (saved_auto_mode),
    .saved_valid       ()
);

// 配置持久化：W25Q64 双槽 0x7FE000 / 0x7FF000 交替写，上电加载较新的有效记录
player_config_persistence #(
    .SLOT0_ADDRESS          (24'h7FE000),
    .SLOT1_ADDRESS          (24'h7FF000),
    .ERASE_TIMEOUT_MS       (5000),               // 5 s（L2：1ms tick 数）
    .PROGRAM_TIMEOUT_MS     (100)                 // 100 ms
) u_cfg_persist (
    .clk             (sys_clk),
    .rst_n           (ctrl_rst_n),
    .tick            (sensor_tick_ms),
    .save_request    (save_done_event),
    .save_period_sel (saved_period_sel),
    .save_brightness (saved_brightness),
    .save_contrast   (saved_contrast),
    .save_volume     (saved_volume),
    .save_auto_mode  (saved_auto_mode),
    .save_ui_hide    (ui_hide_sys),
    .cfg_load_valid  (cfg_load_valid),
    .cfg_period_sel  (cfg_period_sel),
    .cfg_brightness  (cfg_brightness),
    .cfg_contrast    (cfg_contrast),
    .cfg_volume      (cfg_volume),
    .cfg_auto_mode   (cfg_auto_mode),
    .cfg_ui_hide     (cfg_ui_hide),
    .save_ok        (cfg_save_ok),
    .error_event     (cfg_error_event),
    .error_code      (cfg_error_code),
    .flash_ok        (),
    .busy            (),
    .active_valid    (),
    .active_slot     (),
    .active_seq      (),
    .state_code      (),
    .flash_cs_n      (flash_cs_n),
    .flash_sclk      (flash_sclk),
    .flash_mosi      (flash_mosi),
    .flash_miso      (flash_miso),
    .flash_wp_n      (flash_wp_n),
    .flash_hold_n    (flash_hold_n)
);

media_player_control #(
    .DEFAULT_PERIOD_CYCLES (5000),                  // 外接传感器：默认 5 s（L5：单位 = 1ms tick）
    .MIN_PERIOD_CYCLES     (500),                   // 0.5s
    .MAX_PERIOD_CYCLES     (30000),                 // 30s
    .PERIOD_W              (15),                    // L5：30000 → 15 位
    .LOOKAHEAD             (1)                       // 前瞻缓存（下一张 / 上一张常驻 + 缩略图提前生成）
) u_media_player_control (
    .sys_clk              (sys_clk),
    .sys_rst_n            (ctrl_rst_n),
    .tick                 (sensor_tick_ms),
    .goto_valid           (goto_valid),
    .goto_image_idx       (goto_image_idx),
    .goto_audio_idx       (goto_audio_idx),
    .toggle_auto_event    (toggle_auto_event),
    .period_cfg_valid     (period_cfg_valid),
    .period_cfg_cycles    (period_cfg_cycles),
    .auto_tick            (auto_tick),
    .scan_done            (scan_done & player_enable),   // 启动序列放行前播放器不工作
    .image_count          (image_count),
    .loader_ready         (loader_ready),
    .load_done            (load_done),
    .loaded_image_idx     (loaded_image_idx),
    .loaded_buffer_idx    (loaded_buffer_idx),
    .load_error           (load_error),
    .image_complete       (fb_image_complete_sys),   
    .load_cancel          (load_cancel),             
    .fast_view            (thumb_view_sys && thumb_view_back),   // 总览已生效 → 切图快速提交
    .load_request         (load_request),
    .load_image_idx       (load_image_idx),
    .load_buffer_idx      (load_buffer_idx),
    .load_thumb_gen       (load_thumb_gen),          
    .auto_enabled         (auto_enabled),
    .active_period_cycles (active_period_cycles),
    .current_image_idx    (current_image_idx),
    .image_committed      (image_committed),
    .display_valid_sys    (display_valid_sys),
    .control_error        (control_error),
    .error_event          (error_event),
    .request_busy         (request_busy),
    .bank_resident        (),
    .view_hold            (thumb_view_hold),       
    .pix_clk              (pixel_clk),
    .pix_rst_n            (pix_rst_n),
    .frame_boundary       (frame_start_raw),         
    .active_image_idx     (active_image_idx),
    .active_buffer_idx    (active_buffer_idx),
    .active_audio_idx     (active_audio_idx),
    .display_valid        (display_valid),
    .index_commit         (index_commit)             
);

thumbnail_controller #(
    .FOCUS_FOLLOW (1)
) u_thumb_ctrl (
    .clk               (sys_clk),
    .rst_n             (ctrl_rst_n),
    .enable            (display_valid_sys),
    .key_right         (key2_click),
    .key_left          (key2_long),
    .key_confirm       (thumb_key_confirm),
    .key_mode          (thumb_key_mode),
    .image_complete    (4'hF),
    .current_image_idx (current_image_idx),
    .image_count       (image_count),
    .select_busy       (request_busy | fade_busy | thumb_view_hold),   // 总览快速提交的图要先补齐全尺寸图
    .select_error      (error_event),
    .view_mode         (thumb_view_sys),
    .focus_index       (),
    .active_index      (),
    .transition_req    (),
    .select_valid      (),
    .select_image_idx  (),
    .select_reject     (),
    .nav_enable        (thumb_nav_enable),
    .view_busy         (),
    .pix_clk           (pixel_clk),
    .pix_rst_n         (pix_rst_n),
    .frame_start       (frame_start_raw),        // 与控制层提交、reader 帧请求、缩放转场同一个边界
    .view_mode_pix     (thumb_view_pix),
    .focus_pix         (),
    .view_commit       ()
);

// ============================ 加载服务 + 帧缓存 ============================
// sd_card_bmp 的像素流（sys 域）→ media_frame_load_cdc → media_frame_loader（mem 域）
wire        fb_load_start;
wire [1:0]  fb_load_bank;
wire        fb_pixel_valid;
wire [23:0] fb_pixel_rgb;
wire        fb_frame_commit;
wire        fb_load_abort;
wire        fb_done;
wire [7:0]  fb_error;
wire        fb_busy;
// 任意分辨率 + 缩略图
wire [5:0]  fb_load_image;      // 本次加载的图片号（→ 缩略图格子 / 页）
wire        src_geom_valid;
wire [11:0] src_width;
wire [11:0] src_height;
wire        src_hold;
wire        sc_pixel_valid;     // 缩放器输出（640×480 整帧）→ CDC
wire [23:0] sc_pixel_rgb;
wire        sc_frame_commit;
wire        sc_busy;

wire Sdr_init_done;
wire sdram_init_done_sys;
sync_2ff u_sdram_init_sync (
    .clk        (sys_clk),
    .rst_n      (ctrl_rst_n),
    .async_in   (Sdr_init_done),
    .sync_out   (sdram_init_done_sys)
);

wire        sw1_wav_sys;
sync_2ff u_sw1_sync (
    .clk      (sys_clk),
    .rst_n    (ctrl_rst_n),
    .async_in (sw[0]),
    .sync_out (sw1_wav_sys)
);

// 曲目号 {无曲目, 曲号}：图片 0..15 → {0, 0..15}，16 起 → 无曲目位 = 1；PCM 与讲解（LRC/VOC）共用
wire [4:0]  media_trk = {current_image_idx[5:4] != 2'd0, current_image_idx[3:0]};

wire [15:0] wav_track_valid;
wire        wav_info_req;
wire [3:0]  wav_info_idx;
wire        wav_info_valid;
wire [31:0] wav_info_start_lba;
wire [31:0] wav_info_total_sectors;
wire        wav_sd_req;
wire [31:0] wav_sd_lba;
wire        wav_sd_busy;
wire        wav_sd_valid;
wire [7:0]  wav_sd_byte;
wire [8:0]  wav_sd_byte_index;
wire        wav_sd_done;
wire [7:0]  wav_sd_error;
wire        wav_play_enable;
wire [31:0] wav_start_lba;
wire [31:0] wav_total_sectors;
wire        wav_wr_idle;
wire        wav_wr_done;
wire        wav_wr_error;
wire        wav_switching;

wav_playlist_ctrl #(
    .LOOP (1)
) u_wav_playlist (
    .clk                (sys_clk),
    .rst_n              (ctrl_rst_n),
    .enable             (sw1_wav_sys & player_enable_fast),
    .track_valid        (wav_track_valid),
    .select             (media_trk),                // sys 域，提交回执后更新（画面已切到新图）
    .info_req           (wav_info_req),
    .info_idx           (wav_info_idx),
    .info_valid         (wav_info_valid),
    .info_start_lba     (wav_info_start_lba),
    .info_total_sectors (wav_info_total_sectors),
    .play_enable        (wav_play_enable),
    .start_lba          (wav_start_lba),
    .total_sectors      (wav_total_sectors),
    .wr_idle            (wav_wr_idle),
    .wr_done            (wav_wr_done),
    .wr_error           (wav_wr_error),
    .track              (),
    .playing            (),
    .switching          (wav_switching)
);

// sd_card_bmp 的讲解口（曲目类清单项查询 + 仲裁器 C 口）
wire [33:0] narr_aux_valid;
wire        narr_info_req;
wire [5:0]  narr_info_idx;
wire        narr_info_valid;
wire [31:0] narr_info_size;
wire        narr_sector_req;
wire [31:0] narr_sector_lba;
wire        narr_sector_busy;
wire        narr_sector_valid;
wire        narr_sector_done;
wire [7:0]  narr_sector_error;

// 参数集中化：图片尺寸、扫描范围、超时、重试次数全部在这里一处修改
sd_card_bmp #(
    .CLK_FREQ_HZ        (SYS_CLK_HZ),
    .BMP_WIDTH          (16'd640),
    .BMP_HEIGHT         (16'd480),
    .SCAN_START_SECTOR  (32'd0),
    .SCAN_MAX_SECTOR    (32'd131071),
    .SCAN_TARGET_COUNT  (3'd4),
    .SD_READ_TIMEOUT_MS (16'd500),
    .LOAD_TIMEOUT_MS    (2000),        
    .MAX_RETRIES        (2'd1),
    .ANY_SIZE           (1),           
    .ROW_SKIP           (1),           
    .CANCEL_EN          (1),           
    .SMART              (1),           
    .EXT_MS_TICK        (1)            
) sd_card_bmp_m0(
    .clk                 (sys_clk),
    .rst_n               (ctrl_rst_n),
    .ext_ms_tick         (sensor_tick_ms),
    .ingest_enable       (ingest_enable),
    .sd_ready            (sd_ready),
    .scan_done           (scan_done),
    .scan_all_done       (scan_all_done),
    .image_count         (image_count),
    .loader_ready        (loader_ready),
    .load_request        (load_request),
    .load_image_idx      (load_image_idx),
    .load_buffer_idx     (load_buffer_idx),
    .load_done           (load_done),
    .loaded_image_idx    (loaded_image_idx),
    .loaded_buffer_idx   (loaded_buffer_idx),
    .load_error          (load_error),
    .load_cancel         (load_cancel),
    .bmp_state_code      (),
    .sd_reinit_count     (),
    .fs_error            (sd_fs_error),
    .fb_load_start       (fb_load_start),
    .fb_load_bank        (fb_load_bank),
    .pixel_valid         (fb_pixel_valid),
    .pixel_rgb           (fb_pixel_rgb),
    .frame_commit        (fb_frame_commit),
    .fb_load_abort       (fb_load_abort),
    .fb_done             (fb_done),
    .fb_error            (fb_error),
    .fb_busy             (fb_busy),
    .fb_load_image       (fb_load_image),            
    .load_refine         (1'b0),                     
    .fb_load_refine      (),
    .sm_map_valid        (sm_map_valid),             
    .sm_c0               (sm_c0),
    .sm_r0b              (sm_r0b),
    .sm_cw               (sm_cw),
    .sm_ch               (sm_ch),
    .sm_n                (sm_n),
    .sm_d                (sm_d),
    .src_geom_valid      (src_geom_valid),
    .src_width           (src_width),
    .src_height          (src_height),
    .src_hold            (src_hold),
    .src_progress        (sc_pixel_valid),
    .audio_track_valid        (wav_track_valid),          
    .audio_info_req           (wav_info_req),
    .audio_info_idx           (wav_info_idx),
    .audio_info_valid         (wav_info_valid),
    .audio_info_start_lba     (wav_info_start_lba),
    .audio_info_total_sectors (wav_info_total_sectors),
    .audio_sector_req         (wav_sd_req),
    .audio_sector_lba         (wav_sd_lba),
    .audio_sector_busy        (wav_sd_busy),
    .audio_sector_valid       (wav_sd_valid),
    .audio_sector_byte        (wav_sd_byte),
    .audio_sector_byte_index  (wav_sd_byte_index),
    .audio_sector_done        (wav_sd_done),
    .audio_sector_error       (wav_sd_error),
    .narr_aux_valid           (narr_aux_valid),           
    .narr_info_req            (narr_info_req),
    .narr_info_idx            (narr_info_idx),
    .narr_info_valid          (narr_info_valid),
    .narr_info_size           (narr_info_size),
    .narr_sector_req          (narr_sector_req),
    .narr_sector_lba          (narr_sector_lba),
    .narr_sector_busy         (narr_sector_busy),
    .narr_sector_valid        (narr_sector_valid),
    .narr_sector_done         (narr_sector_done),
    .narr_sector_error        (narr_sector_error),
    .SD_nCS              (sd_ncs),
    .SD_DCLK             (sd_dclk),
    .SD_MOSI             (sd_mosi),
    .SD_MISO             (sd_miso)
);

// ============================ 讲解字幕：sys 侧 ============================
// script_loader：扫描结束后记下 HZK32 位置，之后每换一次曲目号（media_trk）：
//   释放握手（pix 侧文字全部淡出、不再读 RAM）→ 解析 TRACKn.LRC（≤ 4KB，字去重）→ 每个不同的字从 HZK32 取 32×32 灰度字形
//   → 写表头 → ready → 定位 TRACKn.VOC → 人声开播（epoch，pix 侧讲解时钟归零）；人声播完从头循环（同 PCM）。
//   仲裁器 C 口由 loader 与人声播放器分时（loader 装稿期间人声停）。错误码 0x71..0x76 进数码管 ERR 页。
wire        voc_sd_req;
wire [31:0] voc_sd_lba;
wire        voc_sd_busy;
wire        voc_sd_valid;
wire        voc_sd_done;
wire        voc_play_enable;
wire [31:0] voc_start_lba;
wire [31:0] voc_total_sectors;
wire        voc_wr_idle;
wire        voc_wr_done;
wire        voc_wr_error;
wire        scr_glyph_we, scr_code_we, scr_line_we;
wire        scr_glyph_park;
wire [2:0]  scr_glyph_blk;
wire [12:0] scr_glyph_addr;
wire [15:0] scr_glyph_data;
wire        scr_glyph_sys_re;
wire [15:0] scr_glyph_sys_q;
wire        scr_dd_we, scr_dd_re;
wire [11:0] scr_dd_addr;
wire [7:0]  scr_dd_data, scr_dd_q;
wire [8:0]  scr_code_addr, scr_line_addr;
wire [9:0]  scr_code_data;
wire [17:0] scr_line_data;
wire        scr_rel_req_t, scr_rel_ack_t, scr_ready_t, scr_epoch_t;
wire [7:0]  script_error;

script_loader #(
    .MAX_W  (72),
    .ROW_W  (72)
) u_script_loader (
    .clk               (sys_clk),
    .rst_n             (ctrl_rst_n),
    .enable            (scan_all_done & player_enable_fast & focus_done),   // 整卡扫描完（清单齐）且首图已提交
    .aux_valid         (narr_aux_valid),
    .select            (media_trk),                            // 与 TRACKn.PCM 同一绑定
    .loc_req           (sl_loc_req),
    .loc_idx           (sl_loc_idx),
    .loc_valid         (narr_info_valid & focus_done),
    .loc_lba           (wav_info_start_lba),                   // locator 输出与音频口共用
    .loc_size          (narr_info_size),
    .sec_req           (sl_sec_req),
    .sec_lba           (sl_sec_lba),
    .sec_busy          (narr_sector_busy | ~focus_done),
    .sec_valid         (narr_sector_valid & focus_done),
    .sec_byte          (wav_sd_byte),                          // 字节 / 序号与音频口共用
    .sec_byte_index    (wav_sd_byte_index),
    .sec_done          (narr_sector_done & focus_done),
    .sec_error         (narr_sector_error),
    .voc_sd_req        (voc_sd_req),
    .voc_sd_lba        (voc_sd_lba),
    .voc_sd_busy       (voc_sd_busy),
    .voc_sd_valid      (voc_sd_valid),
    .voc_sd_done       (voc_sd_done),
    .voc_play_enable   (voc_play_enable),
    .voc_start_lba     (voc_start_lba),
    .voc_total_sectors (voc_total_sectors),
    .voc_wr_idle       (voc_wr_idle),
    .voc_wr_done       (voc_wr_done),
    .voc_wr_error      (voc_wr_error),
    .glyph_we          (scr_glyph_we),
    .glyph_park        (scr_glyph_park),
    .glyph_blk         (scr_glyph_blk),
    .glyph_addr        (scr_glyph_addr),
    .glyph_data        (scr_glyph_data),
    .glyph_re          (scr_glyph_sys_re),
    .glyph_q           (scr_glyph_sys_q),
    .dd_we             (scr_dd_we),
    .dd_re             (scr_dd_re),
    .dd_addr           (scr_dd_addr),
    .dd_data           (scr_dd_data),
    .dd_q              (scr_dd_q),
    .code_we           (scr_code_we),
    .code_addr         (scr_code_addr),
    .code_data         (scr_code_data),
    .line_we           (scr_line_we),
    .line_addr         (scr_line_addr),
    .line_data         (scr_line_data),
    .rel_req_t         (scr_rel_req_t),
    .rel_ack_t         (scr_rel_ack_t),
    .ready_t           (scr_ready_t),
    .epoch_t           (scr_epoch_t),
    .script_error      (script_error)
);

// 讲解口（定位器 B 口 + 仲裁器 C 口）开机先给 focus_loader 读 FOCUS.TXT，done 之后归 script_loader
assign narr_info_req   = focus_done ? sl_loc_req : fl_loc_req;
assign narr_info_idx   = focus_done ? sl_loc_idx : fl_loc_idx;
assign narr_sector_req = focus_done ? sl_sec_req : fl_sec_req;
assign narr_sector_lba = focus_done ? sl_sec_lba : fl_sec_lba;

// 讲解 RAM（sys 写 / pix 读）：字形 8192×64（16 个 BRAM32K，256 个 32×32 灰度字）、去重表 4096×8（sys 单时钟，
//   4 个 BRAM9K）、格 512×10、行表 512×18（各 1 个 BRAM9K）
wire        scr_line_re, scr_code_re, scr_glyph_re;
wire [8:0]  scr_line_raddr, scr_code_raddr;
wire [12:0] scr_glyph_raddr;
wire [17:0] scr_line_rdata;
wire [9:0]  scr_code_rdata;
wire [63:0] scr_glyph_rdata;
script_dedup_ram u_script_dedup (
    .clk  (sys_clk),
    .we   (scr_dd_we),
    .re   (scr_dd_re),
    .addr (scr_dd_addr),
    .data (scr_dd_data),
    .q    (scr_dd_q)
);
script_glyph_bram32k u_script_glyph (
    .sys_clk  (sys_clk),
    .sys_we   (scr_glyph_we),
    .sys_park (scr_glyph_park),
    .sys_blk  (scr_glyph_blk),
    .sys_re   (scr_glyph_sys_re),
    .sys_addr (scr_glyph_addr),
    .sys_data (scr_glyph_data),
    .sys_q    (scr_glyph_sys_q),
    .pix_clk  (pixel_clk),
    .pix_re   (scr_glyph_re),
    .pix_addr (scr_glyph_raddr),
    .pix_q    (scr_glyph_rdata)
);
script_dc_ram #(.AW(9), .DW(10)) u_script_code (
    .wclk (sys_clk),   .we (scr_code_we),   .waddr (scr_code_addr),   .wdata (scr_code_data),
    .rclk (pixel_clk), .re (scr_code_re),   .raddr (scr_code_raddr),  .rdata (scr_code_rdata)
);
script_dc_ram #(.AW(9), .DW(18)) u_script_line (
    .wclk (sys_clk),   .we (scr_line_we),   .waddr (scr_line_addr),   .wdata (scr_line_data),
    .rclk (pixel_clk), .re (scr_line_re),   .raddr (scr_line_raddr),  .rdata (scr_line_rdata)
);

bmp_smart_scaler #(
    .PACE  (16)          // 50MHz/16 = 3.1M 像素/s，整帧 0.1s，SDRAM 写带宽余量 > 4 倍
) u_bmp_scale (
    .clk        (sys_clk),
    .rst_n      (ctrl_rst_n),
    .load_start (fb_load_start),
    .load_abort (fb_load_abort),
    .refine     (1'b0),          // REFINE_EN = 0：REF 模式裁掉
    .ref_c0     (12'd0),
    .ref_r0b    (12'd0),
    .ref_cw     (12'd576),
    .ref_ch     (12'd432),
    .geom_valid (src_geom_valid),
    .src_w      (src_width),
    .src_h      (src_height),
    .in_valid   (fb_pixel_valid),
    .in_rgb     (fb_pixel_rgb),
    .in_hold    (src_hold),
    .out_valid  (sc_pixel_valid),
    .out_rgb    (sc_pixel_rgb),
    .out_commit (sc_frame_commit),
    .busy       (sc_busy),
    .overflow   (),
    .map_valid  (sm_map_valid),
    .map_direct (),
    .map_c0     (sm_c0),
    .map_r0b    (sm_r0b),
    .map_cw     (sm_cw),
    .map_ch     (sm_ch),
    .map_n      (sm_n),
    .map_d      (sm_d),
    .plan_valid (sc_plan_valid),
    .g_ox       (sc_g_ox),
    .g_oyt      (sc_g_oyt),
    .g_c0       (sc_g_c0),
    .g_r0t      (sc_g_r0t),
    .g_cw       (sc_g_cw),
    .g_ch       (sc_g_ch),
    .g_n        (sc_g_n),
    .g_d        (sc_g_d)
);

boot_sequencer #(
    .DRAIN_CYCLES (31)
) u_boot_seq (
    .clk             (sys_clk),
    .rst_n           (ctrl_rst_n),
    .tick            (sensor_tick_ms),
    .sdram_init_done (sdram_init_done_sys),
    .sd_init_done    (sd_ready),
    .scan_done       (scan_done),
    .image_count     (image_count),
    .write_busy      (fb_busy | fb_pixel_valid | sc_pixel_valid),   
    .ingest_enable   (ingest_enable),
    .player_enable   (player_enable),
    .boot_error      (boot_error),
    .error_event     (boot_error_event),
    .boot_state      (boot_state)
);

// ============================  四bank帧缓存 ============================
wire Sdr_init_ref_vld;
wire Sdr_busy;
wire App_rd_en;
wire [ADDR_BITS-1:0] App_rd_addr;
wire Sdr_rd_en;
wire [MEM_DATA_BITS-1:0] Sdr_rd_dout;
wire App_wr_en;
wire [ADDR_BITS-1:0] App_wr_addr;
wire [MEM_DATA_BITS-1:0] App_wr_din;
wire [3:0] App_wr_dm;

// ---- loader（mem_clk 域）----
wire        loader_load_start;
wire [1:0]  loader_image_index;
wire        loader_pixel_valid;
wire [23:0] loader_pixel_rgb;
wire        loader_frame_commit;
wire [7:0]  loader_decode_error;
wire        loader_load_busy;
wire        loader_load_done;
wire [3:0]  loader_image_complete;
wire [7:0]  loader_memory_error;
wire        loader_app_wr_en;
wire [20:0] loader_app_wr_addr;
wire [3:0]  loader_app_wr_mask;
wire [31:0] loader_app_wr_data;
wire        loader_burst_want;
wire        loader_burst_grant;
wire        loader_burst_busy;

media_frame_load_cdc u_load_cdc (
    .src_clk            (sys_clk),
    .src_rst_n          (ctrl_rst_n),
    .load_start         (fb_load_start),
    .load_bank          (fb_load_bank),
    .pixel_valid        (sc_pixel_valid),      // 缩放器输出
    .pixel_rgb          (sc_pixel_rgb),
    .frame_commit       (sc_frame_commit),
    .load_abort         (fb_load_abort),
    .done_event         (fb_done),
    .done_error         (fb_error),
    .image_complete_sys (fb_image_complete_sys),
    .stream_overflow    (),
    .dst_clk            (mem_clk),
    .dst_rst_n          (mem_rst_n),
    .ld_load_start      (loader_load_start),
    .ld_image_index     (loader_image_index),
    .ld_pixel_valid     (loader_pixel_valid),
    .ld_pixel_rgb       (loader_pixel_rgb),
    .ld_frame_commit    (loader_frame_commit),
    .ld_decode_error    (loader_decode_error),
    .ld_load_done       (loader_load_done),
    .ld_memory_error    (loader_memory_error),
    .ld_image_complete  (loader_image_complete)
);

media_frame_loader #(
    .ADDR_WIDTH    (21),
    .IMAGE_WORDS   (307200),
    .FIFO_DEPTH    (512),
    .BURST_SIZE    (128),
    .STALL_TIMEOUT (65535),
    .WRITE_V_FLIP  (1),            // BMP 自下而上存储（同官方 frame_fifo_write 的 WRITE_V_FLIP）
    .FRAME_WIDTH   (640),
    .FRAME_HEIGHT  (480)
) u_loader (
    .clk                  (mem_clk),          // 接 SDRAM 控制器时钟
    .rst_n                (mem_rst_n),
    .load_start           (loader_load_start),
    .image_index          (loader_image_index),
    .pixel_valid          (loader_pixel_valid),
    .pixel_rgb            (loader_pixel_rgb),
    .frame_commit         (loader_frame_commit),
    .decode_error         (loader_decode_error),
    .load_busy            (loader_load_busy),
    .load_done            (loader_load_done),
    .image_complete       (loader_image_complete),
    .memory_error         (loader_memory_error),
    .controller_init_done (Sdr_init_done),
    .controller_busy      (Sdr_busy),
    .refresh_active       (Sdr_init_ref_vld),
    .burst_want           (loader_burst_want),
    .burst_grant          (loader_burst_grant),
    .burst_busy           (loader_burst_busy),
    .app_wr_en            (loader_app_wr_en),
    .app_wr_addr          (loader_app_wr_addr),
    .app_wr_mask          (loader_app_wr_mask),
    .app_wr_data          (loader_app_wr_data)
);

// ----  缩略图写入器（mem_clk 域）：旁听 loader 像素流，4:1 抽样写第 5 区 2×2 拼图 ----
// 图片号：sd_card_bmp 在 load_request 时锁存（远早于第一个像素），cdc_bus_handshake 送 mem 域，
// 写入器在第一个像素时采样。clear 接 0：scan_done 只有全局复位才清（复位已清 valid）
// 2:1 抽样（格子 320×240，4 格铺满全屏）+ 双缓冲（区 = 页号奇偶，第 6 区基址 307200×5）；
//   load_thumb_gen（控制层：该图缩略图已完好就不写）与图片号同一条握手送过去，两者都在 load_request 时更新
wire [5:0]  thumb_image_idx_mem;
wire        thumb_gen_mem;
wire [7:0]  thumb_valid_mem;
wire [31:0] thumb_tags_mem;
wire        thumb_burst_want;
wire        thumb_burst_grant;
wire        thumb_burst_busy;
wire        thumb_app_wr_en;
wire [20:0] thumb_app_wr_addr;
wire [3:0]  thumb_app_wr_mask;
wire [31:0] thumb_app_wr_data;
wire        arb_wr_grant;

cdc_bus_handshake #(.WIDTH(7)) u_thumb_img_cdc (
    .src_clk   (sys_clk),
    .src_rst_n (ctrl_rst_n),
    .src_data  ({load_thumb_gen, fb_load_image}),
    .dst_clk   (mem_clk),
    .dst_rst_n (mem_rst_n),
    .dst_data  ({thumb_gen_mem, thumb_image_idx_mem})
);

media_thumb_writer #(
    .ADDR_WIDTH (21),
    .BASE       (307200 * 4),
    .BURST      (32),
    .SUB_LOG2   (1),            
    .DUAL       (1),           
    .AREA_WORDS (307200)
) u_thumb_writer (
    .clk                  (mem_clk),
    .rst_n                (mem_rst_n),
    .clear                (1'b0),
    .image_idx            (thumb_image_idx_mem),
    .gen                  (thumb_gen_mem),
    .ld_load_start        (loader_load_start),
    .ld_pixel_valid       (loader_pixel_valid),
    .ld_pixel_rgb         (loader_pixel_rgb),
    .ld_frame_commit      (loader_frame_commit),
    .ld_decode_error      (loader_decode_error),
    .controller_init_done (Sdr_init_done),
    .burst_want           (thumb_burst_want),
    .burst_grant          (thumb_burst_grant),
    .burst_busy           (thumb_burst_busy),
    .app_wr_en            (thumb_app_wr_en),
    .app_wr_addr          (thumb_app_wr_addr),
    .app_wr_mask          (thumb_app_wr_mask),
    .app_wr_data          (thumb_app_wr_data),
    .thumb_valid          (thumb_valid_mem),
    .thumb_tags           (thumb_tags_mem)
);

// 写口合并：仲裁器只看到一个写口；loader 优先，两者的 burst 不会同时进行（busy 互相挡住 settled）
assign loader_burst_grant = arb_wr_grant && loader_burst_want;
assign thumb_burst_grant  = arb_wr_grant && !loader_burst_want && thumb_burst_want;

// ---- reader（mem_clk + pixel_clk 双时钟域）----
wire        reader_app_rd_en;
wire [20:0] reader_app_rd_addr;
wire        reader_sdram_rd_valid;
wire [31:0] reader_sdram_rd_data;
wire        reader_burst_want;
wire        reader_burst_grant;
wire        reader_burst_busy;
wire        reader_frame_start;
wire        reader_pixel_consume;
wire [23:0] reader_pixel_rgb;
wire        reader_pixel_valid;
wire        reader_underflow;
wire        reader_reader_busy;
wire        reader_frame_read_done;
wire [2:0]  reader_reader_index;
wire        thumb_display;       //1 = 总览（reader 读第 5 区拼图），pix 域，frame_start 同拍更新
wire [7:0]  reader_reader_error;
wire        reader_enable_mem;

// 首图完整写入并在帧边界提交之前，禁止 reader 访问 SDRAM。
// Sdr_init_done 放行，reader 会在 loader 写 bank0 的同时反复读取同一 bank；
// 行为仿真能被 burst 仲裁掩盖，但真实 SDRAM 上这是没有必要的读写扰动。
// display_valid_sys 在首图提交应答后才置位，2FF 同步到 mem 域后再开放读 burst。
sync_2ff u_reader_enable_mem_sync (
    .clk      (mem_clk),
    .rst_n    (mem_rst_n),
    .async_in (display_valid_sys),
    .sync_out (reader_enable_mem)
);

media_frame_reader #(
    .ADDR_WIDTH          (21),
    .IMAGE_WORDS         (307200),
    .FIFO_DEPTH          (512),
    .FIFO_MARGIN         (8),
    .BURST_SIZE          (128),
    .BURST_GAP_CYCLES    (12),
    .MAX_INFLIGHT_BURSTS (2),
    .INDEX_W             (3)       // index 4 = 第 5 区缩略图拼图
) u_reader (
    .mem_clk              (mem_clk),
    .mem_rst_n            (mem_rst_n),
    .controller_init_done (Sdr_init_done && reader_enable_mem),
    .controller_busy      (Sdr_busy),
    .refresh_active       (Sdr_init_ref_vld),
    .burst_want           (reader_burst_want),
    .burst_grant          (reader_burst_grant),
    .burst_busy           (reader_burst_busy),
    .app_rd_en            (reader_app_rd_en),
    .app_rd_addr          (reader_app_rd_addr),
    .sdram_rd_valid       (reader_sdram_rd_valid),
    .sdram_rd_data        (reader_sdram_rd_data),
    .pix_clk              (pixel_clk),
    .pix_rst_n            (pix_rst_n),
    .frame_start          (reader_frame_start),
    // 两者都在 frame_start 同拍提交；
    .active_index         (thumb_display ? {2'b10, active_image_idx[2]} : {1'b0, active_buffer_idx}),
    .pixel_consume        (reader_pixel_consume),
    .pixel_rgb            (reader_pixel_rgb),
    .pixel_valid          (reader_pixel_valid),
    .underflow            (reader_underflow),
    .reader_busy          (reader_reader_busy),
    .frame_read_done      (reader_frame_read_done),
    .reader_index         (reader_reader_index),
    .reader_error         (reader_reader_error)
);

// 定时刷新也由仲裁器发起（每 576 拍 = 14.4us，总线安静时插入），控制器内部定时器已关闭
wire App_ref_req;
media_sdram_arbiter #(
    .IDLE_CYCLES  (8),
    .REF_INTERVAL (576)
) u_sdram_arb (
    .clk       (mem_clk),
    .rst_n     (mem_rst_n),
    .ctrl_busy (Sdr_busy),
    .ctrl_init_done (Sdr_init_done),
    .ref_vld   (Sdr_init_ref_vld),
    .ref_req   (App_ref_req),
    .wr_want   (loader_burst_want | thumb_burst_want),   // loader + 缩略图写入器合并
    .wr_busy   (loader_burst_busy | thumb_burst_busy),
    .wr_grant  (arb_wr_grant),
    .rd_want   (reader_burst_want | tv_dma_want),   // 导览行 DMA 与 reader 共用读口（不同帧，reader 优先）
    .rd_busy   (reader_burst_busy | tv_dma_busy),
    .rd_grant  (arb_rd_grant)
);
assign reader_burst_grant = arb_rd_grant;
assign tv_dma_grant       = arb_rd_grant & ~reader_burst_want;

// ---- SDRAM 控制器信号名映射 ----
// 写端口：loader / 缩略图写入器 → sdram（同一时刻只有一个在 burst）
assign App_wr_en   = loader_app_wr_en | thumb_app_wr_en;
assign App_wr_addr = thumb_app_wr_en ? thumb_app_wr_addr : loader_app_wr_addr;
assign App_wr_din  = thumb_app_wr_en ? thumb_app_wr_data : loader_app_wr_data;
assign App_wr_dm   = thumb_app_wr_en ? thumb_app_wr_mask : loader_app_wr_mask;
// 读端口：reader → sdram
assign App_rd_en   = reader_app_rd_en | tv_app_rd_en;
assign App_rd_addr = tv_app_rd_en ? tv_app_rd_addr : reader_app_rd_addr;
// sdram → reader：Sdr_rd_en 是与 Sdr_rd_dout 同拍的"读数据有效"
assign reader_sdram_rd_valid = Sdr_rd_en & ~tv_dma_own;    
assign reader_sdram_rd_data  = Sdr_rd_dout;

sdram U3(
    .Clk               (mem_clk),
    .Clk_sft           (mem_clk_shift),
    .Rst_n             (mem_rst_n),
    .Sdr_init_done     (Sdr_init_done),
    .Sdr_init_ref_vld  (Sdr_init_ref_vld),
    .Sdr_busy          (Sdr_busy),
    .App_ref_req       (App_ref_req),
    .App_wr_en         (App_wr_en),
    .App_wr_addr       (App_wr_addr),
    .App_wr_dm         (App_wr_dm),
    .App_wr_din        (App_wr_din),
    .App_rd_en         (App_rd_en),
    .App_rd_addr       (App_rd_addr),
    .Sdr_rd_en         (Sdr_rd_en),
    .Sdr_rd_dout       (Sdr_rd_dout)
);

wire raw_hs, raw_vs, raw_de;
wire [11:0] raw_x, raw_y;
wire hs, vs, de;
wire [23:0] vout_data_raw;
wire [23:0] vout_data_gated;
wire [23:0] vout_status;        
wire [23:0] vout_data;          

video_timing_ctrl #(
    .H_ACT (640), .H_FP (16), .H_SYNC (96), .H_BP (48),
    .V_ACT (480), .V_FP (10), .V_SYNC (2),  .V_BP (33),
    .ACTIVE_LAST (1), .VS_ALIGN_HSYNC (1)     // vsync 边沿对齐 hsync 前沿 = 官方 color_bar 波形
) u_timing (
    .pclk  (pixel_clk),
    .rst_n (pix_rst_n),
    .hsync (raw_hs),
    .vsync (raw_vs),
    .de    (raw_de),
    .x     (raw_x),
    .y     (raw_y)
);

// frame_start = 场同步下降沿（帧消隐期开始）：播放控制器提交点 + reader 帧请求
reg raw_vs_q;
always @(posedge pixel_clk or negedge pix_rst_n) begin
    if (!pix_rst_n)
        raw_vs_q <= 1'b1;
    else
        raw_vs_q <= raw_vs;
end
assign frame_start_raw = raw_vs_q && !raw_vs;

// SDRAM 初始化完成之前 reader 不接帧请求、不消费像素（否则每帧都会报下溢）
wire sdram_init_done_pix;
sync_2ff u_sdram_init_pix_sync (
    .clk      (pixel_clk),
    .rst_n    (pix_rst_n),
    .async_in (Sdr_init_done),
    .sync_out (sdram_init_done_pix)
);
// 导览帧（本次 frame_start 锁存的 tour_on = 1）reader 不取帧、不消费，由 tour_view 出图
assign reader_frame_start   = frame_start_raw && sdram_init_done_pix && !tour_bundle_pix[0];
assign reader_pixel_consume = raw_de && sdram_init_done_pix && display_valid && !tour_on_pix;

// ============================导览运镜 ============================
// sys：focus_loader（开机读一次 FOCUS.TXT 焦点表）→ tour_sequencer（几何表 / 焦点换算 / 缓动，微程序引擎 1 个 DSP）
// pix + mem：tour_view（视角双线性渲染 + 行 DMA，6 个 DSP；资源取舍：无小地图）
// 跨域：显示 bank pix→sys（cdc_bus_handshake）、帧节拍 pix→sys（toggle + 2FF）；
//       bundle sys→pix 直连（写入窗口与 frame_start 错开）
wire [7:0]  tour_ft_addr;
wire [26:0] tour_ft_data;
focus_loader u_focus_loader (
    .clk       (sys_clk),
    .rst_n     (ctrl_rst_n),
    .enable    (scan_all_done & player_enable_fast),
    .aux_ok    (narr_aux_valid[33]),
    .loc_req   (fl_loc_req),
    .loc_idx   (fl_loc_idx),
    .loc_valid (narr_info_valid & ~focus_done),
    .loc_lba   (wav_info_start_lba),
    .loc_size  (narr_info_size),
    .sec_req   (fl_sec_req),
    .sec_lba   (fl_sec_lba),
    .sec_busy  (narr_sector_busy),
    .sec_valid (narr_sector_valid & ~focus_done),
    .sec_byte  (wav_sd_byte),
    .sec_done  (narr_sector_done & ~focus_done),
    .sec_error (narr_sector_error),
    .done      (focus_done),
    .rd_addr   (tour_ft_addr),
    .rd_data   (tour_ft_data)
);

reg tour_ftog;
always @(posedge pixel_clk or negedge pix_rst_n) begin
    if (!pix_rst_n)
        tour_ftog <= 1'b0;
    else if (frame_start_raw)
        tour_ftog <= ~tour_ftog;
end
wire tour_ftog_sys;
sync_2ff u_tour_ftick_sync (
    .clk      (sys_clk),
    .rst_n    (ctrl_rst_n),
    .async_in (tour_ftog),
    .sync_out (tour_ftog_sys)
);
reg tour_ftog_q;
always @(posedge sys_clk or negedge ctrl_rst_n) begin
    if (!ctrl_rst_n)
        tour_ftog_q <= 1'b0;
    else
        tour_ftog_q <= tour_ftog_sys;
end
wire tour_frame_tick = tour_ftog_sys ^ tour_ftog_q;

wire [1:0] tour_disp_bank;
cdc_bus_handshake #(.WIDTH(2)) u_tour_bank_cdc (
    .src_clk   (pixel_clk),
    .src_rst_n (pix_rst_n),
    .src_data  (active_buffer_idx),
    .dst_clk   (sys_clk),
    .dst_rst_n (ctrl_rst_n),
    .dst_data  (tour_disp_bank)
);

wire [80:0] tour_bundle_sys;
tour_sequencer u_tour_seq (
    .clk            (sys_clk),
    .rst_n          (ctrl_rst_n),
    .chord          (tour_chord),
    .tour_next      (tour_next),
    .enable         (display_valid_sys),
    .block          (thumb_view_sys | thumb_view_back),
    .start_block    (request_busy | fade_busy | thumb_view_hold),
    .cur_img        (current_image_idx),
    .frame_tick     (tour_frame_tick),
    .disp_bank      (tour_disp_bank),
    .fb_load_start  (fb_load_start),
    .fb_load_bank   (fb_load_bank),
    .plan_valid     (sc_plan_valid),
    .g_ox           (sc_g_ox),
    .g_oyt          (sc_g_oyt),
    .g_c0           (sc_g_c0),
    .g_r0t          (sc_g_r0t),
    .g_cw           (sc_g_cw),
    .g_ch           (sc_g_ch),
    .g_n            (sc_g_n),
    .g_d            (sc_g_d),
    .ft_addr        (tour_ft_addr),
    .ft_data        (tour_ft_data),
    .tour_busy      (tour_busy),
    .bundle         (tour_bundle_sys)
);

// bundle 不经握手 CDC：tour_sequencer 只在 frame_tick 之后几 µs 内改 bq_*，pix 在 frame_start 锁存（协议上准静态，SDC -from bq_*）
assign tour_bundle_pix = tour_bundle_sys;

tour_view #(
    .H_START  (160),
    .V_START  (45)
) u_tour_view (
    .pix_clk        (pixel_clk),
    .pix_rst_n      (pix_rst_n),
    .bundle         (tour_bundle_pix),
    .frame_start    (frame_start_raw),
    .raw_x          (raw_x),
    .raw_y          (raw_y),
    .bank           (active_buffer_idx),
    .tour_on        (tour_on_pix),
    .tour_rgb       (tour_rgb),
    .mem_clk        (mem_clk),
    .mem_rst_n      (mem_rst_n),
    .dma_want       (tv_dma_want),
    .dma_grant      (tv_dma_grant),
    .dma_busy       (tv_dma_busy),
    .dma_own        (tv_dma_own),
    .app_rd_en      (tv_app_rd_en),
    .app_rd_addr    (tv_app_rd_addr),
    .sdram_rd_valid (Sdr_rd_en),
    .sdram_rd_data  (Sdr_rd_dout)
);

// ============================ 缩略图总览（pix 域）============================
// 格子状态 mem → pix（准静态，cdc_bus_handshake）；格子 s 显示的条件 = 有完整缩略图且页号 = 当前页
// 缩略图区双缓冲，8 个条目 {区, slot}，当前页读 区 = 页号奇偶（active_image_idx[2]）
wire [39:0] thumb_status_pix;
cdc_bus_handshake #(.WIDTH(40)) u_thumb_status_cdc (
    .src_clk   (mem_clk),
    .src_rst_n (mem_rst_n),
    .src_data  ({thumb_valid_mem, thumb_tags_mem}),
    .dst_clk   (pixel_clk),
    .dst_rst_n (pix_rst_n),
    .dst_data  (thumb_status_pix)
);
wire [3:0]  thumb_page_pix = active_image_idx[5:2];
wire [3:0]  thumb_valid_pg = active_image_idx[2] ? thumb_status_pix[39:36] : thumb_status_pix[35:32];
wire [15:0] thumb_tags_pg  = active_image_idx[2] ? thumb_status_pix[31:16] : thumb_status_pix[15:0];
wire [3:0]  thumb_complete = thumb_valid_pg & {
    thumb_tags_pg[15:12] == thumb_page_pix, thumb_tags_pg[11:8] == thumb_page_pix,
    thumb_tags_pg[7:4]   == thumb_page_pix, thumb_tags_pg[3:0]  == thumb_page_pix};

wire sw4_thumb_pix = thumb_view_pix;

//   页号寄存一拍即"提交前的页"；总览（生效值）中只有换页的提交才交给转场控制器（KEY2 跨页淡入 / 自动轮播跨页方块溶解）
reg  [3:0] commit_page_q;
always @(posedge pixel_clk or negedge pix_rst_n) begin
    if (!pix_rst_n)
        commit_page_q <= 4'd0;
    else
        commit_page_q <= active_image_idx[5:2];
end
assign fade_index_commit = index_commit && (!thumb_view_pix || (active_image_idx[5:2] != commit_page_q));

// 图片数 sys → pix（慢变，后台扫描期间会增长）；格子 s 有图 = 页号·4 + s < 图片数
wire [6:0] image_count_pix;
cdc_bus_handshake #(.WIDTH(7)) u_img_count_cdc (
    .src_clk   (sys_clk),
    .src_rst_n (ctrl_rst_n),
    .src_data  (image_count),
    .dst_clk   (pixel_clk),
    .dst_rst_n (pix_rst_n),
    .dst_data  (image_count_pix)
);
wire [3:0] thumb_cell_exists = {
    {1'b0, thumb_page_pix, 2'd3} < image_count_pix, {1'b0, thumb_page_pix, 2'd2} < image_count_pix,
    {1'b0, thumb_page_pix, 2'd1} < image_count_pix, {1'b0, thumb_page_pix, 2'd0} < image_count_pix};
// 呼吸：64 帧一个来回（约 1.07 s），灰度 0x30..0x6E，只在帧边界变化；[6:2] 另给频谱柱做色相流动
reg  [6:0] ph_frame_cnt;
always @(posedge pixel_clk or negedge pix_rst_n) begin
    if (!pix_rst_n)
        ph_frame_cnt <= 7'd0;
    else if (frame_start_raw)
        ph_frame_cnt <= ph_frame_cnt + 7'd1;
end
wire [4:0] ph_tri   = ph_frame_cnt[5] ? ~ph_frame_cnt[4:0] : ph_frame_cnt[4:0];
wire [7:0] ph_level = 8'h30 + {2'b00, ph_tri, 1'b0};

// SW4 拨上：全屏图向焦点格聚缩 + 淡出 → 切总览 → 网格从焦点格外扩 + 淡入；拨下反向（各 12 帧，共 0.4s）
wire        zoom_active;
wire [7:0]  zoom_alpha;
wire [11:0] zoom_x0, zoom_x1, zoom_y0, zoom_y1;
thumb_zoom_transition #(
    .X_OFFSET (160),
    .Y_OFFSET (45),
    .A_MID    (8'd64),
    .STEPS    (12),
    .FULL_GRID (1)                             
) u_thumb_zoom (
    .clk           (pixel_clk),
    .rst_n         (pix_rst_n),
    .frame_start   (frame_start_raw),
    .display_valid (display_valid),
    .sw_thumb      (sw4_thumb_pix),
    .focus         (active_image_idx[1:0]),
    .thumb_display (thumb_display),
    .zoom_active   (zoom_active),
    .zoom_alpha    (zoom_alpha),
    .clip_x0       (zoom_x0),
    .clip_x1       (zoom_x1),
    .clip_y0       (zoom_y0),
    .clip_y1       (zoom_y1)
);

always @(posedge pixel_clk or negedge pix_rst_n) begin
    if (!pix_rst_n)
        thumb_view_settled <= 1'b0;
    else if (!zoom_active && (thumb_display == sw4_thumb_pix))
        thumb_view_settled <= sw4_thumb_pix;
end

//   [17] 调节模式  [16] 当前项(0 亮度/1 对比度)  [15:8] 对比度  [7:0] 真实显示亮度（外接光照：环境基准 + 按键偏移）
wire [17:0] picture_pix;
cdc_bus_handshake #(
    .WIDTH     (18),
    .RESET_VAL ({1'b0, 1'b0, 8'd128, 8'd255})
) u_picture_cdc (
    .src_clk   (sys_clk),
    .src_rst_n (ctrl_rst_n),
    .src_data  ({adj_mode, adj_sel, contrast, eff_brightness}),
    .dst_clk   (pixel_clk),
    .dst_rst_n (pix_rst_n),
    .dst_data  (picture_pix)
);

wire [7:0]  view_alpha    = zoom_active ? zoom_alpha : (fade_alpha | {8{dissolve_active}});
wire        view_dissolve = dissolve_active && !zoom_active;

wire        trans_hs, trans_vs, trans_de;
wire [23:0] trans_rgb;

wire [23:0] view_rgb;

wire [23:0] bc_rgb;
wire [7:0]  bc_alpha;
wire [7:0]  bright_frame_pix;
wire [7:0]  contrast_frame_pix;
brightness_contrast_adjust #(
    .BRIGHT_STANDALONE (0),       
    .PIPELINE          (0)
) u_bc_adjust (
    .clk           (pixel_clk),
    .rst_n         (pix_rst_n),
    .frame_start   (frame_start_raw),
    .brightness_in (picture_pix[7:0]),    // 外接传感器：真实显示亮度（环境基准 + 按键偏移）
    .contrast_in   (picture_pix[15:8]),
    .alpha_in      (view_alpha),
    .in_de         (raw_de),
    .rgb_in        (tour_on_pix ? tour_rgb : reader_pixel_valid ? reader_pixel_rgb : 24'h000000),   // 导览帧出 tour_view
    .out_de        (),
    .rgb_out       (bc_rgb),
    .alpha_out     (bc_alpha),
    .brightness_q  (bright_frame_pix),
    .contrast_q    (contrast_frame_pix)
);
// 亮度同步：底部频谱柱 / 讲解跑马灯的前景色 × (本帧亮度 + 1)/256，与图片用同一个帧锁存值；参数条 / 标题不接
wire [8:0]  bri_gain_pix = {1'b0, bright_frame_pix} + 9'd1;
thumb_view_mixer #(
    .X_OFFSET    (160),
    .Y_OFFSET    (45),
    .FOCUS_RGB   (24'hFFD000),
    .PLACEHOLDER (1),                        
    .CELL_W      (320),                        
    .CELL_H      (240),
    .EMPTY_LEVEL (8'h40)                 
) u_thumb_view (
    .de_in          (raw_de),
    .x_in           (raw_x),
    .y_in           (raw_y),
    .rgb_in         (bc_rgb),                  
    .thumb_display  (thumb_display),
    .focus_index    (active_image_idx[1:0]),
    .image_complete (thumb_complete),
    .clip_en        (zoom_active),
    .clip_x0        (zoom_x0),
    .clip_x1        (zoom_x1),
    .clip_y0        (zoom_y0),
    .clip_y1        (zoom_y1),
    .cell_exists    (thumb_cell_exists),   
    .ph_level       (ph_level),
    .rgb_out        (view_rgb),
    .thumb_de       (),
    .source_index   (),
    .focus_border   (),
    .placeholder    ()
);
video_transition_pixel #(
    .FADE_FRAMES     (TRANSITION_FRAMES),
    .PIPELINE        (0)
) u_transition_pixel (
    .clk             (pixel_clk),
    .rst_n           (pix_rst_n),
    .hs_in           (raw_hs),
    .vs_in           (raw_vs),
    .de_in           (raw_de),
    .x_in            (raw_x),
    .y_in            (raw_y),
    .rgb_in          (view_rgb),               
    .alpha           (bc_alpha),                
    .dissolve_active (view_dissolve),
    .ramp            (fade_ramp),
    .seed            (active_image_idx[3:0]),
    .hs_out          (trans_hs),
    .vs_out          (trans_vs),
    .de_out          (trans_de),
    .rgb_out         (trans_rgb)
);

rgb_video_adapter u_video_adapter (
    .pclk      (pixel_clk),
    .rst_n     (pix_rst_n),
    .hs_in     (trans_hs),
    .vs_in     (trans_vs),
    .de_in     (trans_de),
    .rgb_in    (trans_rgb),
    .video_clk (),
    .video_hs  (hs),
    .video_vs  (vs),
    .video_de  (de),
    .video_rgb (vout_data_raw)
);


frame_boundary_pulse #(
    .V_ACTIVE (480)
) u_frame_boundary (
    .pix_clk     (pixel_clk),
    .pix_rst_n   (pix_rst_n),
    .vsync_n     (vs),
    .de          (de),
    .frame_start (frame_start),
    .blank_start (blank_start)
);

// ---- 帧缓存读侧错误 → sys 域，并入数码管/OSD 错误码----
//   reader_error（mem 域多位慢变）走 toggle 握手；underflow（pix 域，"上一帧下溢过"至少保持一帧）走 2FF
//   0x61 帧溢出 / 0x62 读 FIFO 溢出 / 0x64 短帧；underflow 合并显示为 0x63。首图提交前不报（读的是空 bank）
wire [7:0] reader_error_sys;
wire       reader_underflow_sys;
cdc_bus_handshake #(
    .WIDTH     (8),
    .RESET_VAL (8'h00)
) u_reader_err_cdc (
    .src_clk   (mem_clk),
    .src_rst_n (mem_rst_n),
    .src_data  (reader_reader_error),
    .dst_clk   (sys_clk),
    .dst_rst_n (ctrl_rst_n),
    .dst_data  (reader_error_sys)
);
sync_2ff u_underflow_sync (
    .clk      (sys_clk),
    .rst_n    (ctrl_rst_n),
    .async_in (reader_underflow),
    .sync_out (reader_underflow_sys)
);
wire [7:0] fb_read_error_now = !display_valid_sys        ? 8'h00 :
                               (reader_error_sys != 8'h00) ? reader_error_sys :
                               reader_underflow_sys        ? 8'h63 : 8'h00;
// 锁存最近一次读侧错误（单帧下溢也能在数码管/OSD 上看到），下一次切图请求或新的控制错误时清除
reg  [7:0] fb_error_shown;
reg        fb_error_event;
always @(posedge sys_clk or negedge ctrl_rst_n) begin
    if (!ctrl_rst_n) begin
        fb_error_shown <= 8'h00;
        fb_error_event <= 1'b0;
    end else begin
        fb_error_event <= 1'b0;
        if (fb_read_error_now != 8'h00) begin
            fb_error_shown <= fb_read_error_now;
            fb_error_event <= (fb_read_error_now != fb_error_shown);
        end else if (goto_valid || error_event) begin
            fb_error_shown <= 8'h00;
        end
    end
end

// 错误码合并：启动错误(0x59/0x5A/0x53)优先；其次最近一次 Flash 保存错误(0x5B/0x5C，
// 保存成功或出现新的播放控制错误时清除)；再次帧缓存读侧错误；
// （loader 的 0x41~0x46 经 sd_card_bmp 重试后作为 load_error 进入播放控制错误）。数码管 ERR 页与 OSD 共用
reg  [7:0] cfg_error_shown;
always @(posedge sys_clk or negedge ctrl_rst_n) begin
    if (!ctrl_rst_n)
        cfg_error_shown <= 8'h00;
    else if (cfg_error_event)
        cfg_error_shown <= cfg_error_code;
    else if (cfg_save_ok || error_event)
        cfg_error_shown <= 8'h00;
end
// boot_sequencer 的 0x53（image_count==0，"没有可用 BMP"）本身不可诊断；
// 有 fs_error 时顶替显示成具体原因——0x20 引导签名错/非 FAT32、0x21 BPB 或分区类型不支持、
// 0x22 根目录扫完但没凑齐 PIC1~4.BMP（可能是命名、位置、或子目录问题）。fs_error 不清零，
// 但只在 boot_error==0x53 时才会被看到，不会覆盖 0x59/0x5A 等其它启动错误。
wire [7:0] boot_error_shown   = (boot_error == 8'h53 && sd_fs_error != 8'h00) ? sd_fs_error : boot_error;
wire [7:0] status_error       = (boot_error_shown != 8'h00) ? boot_error_shown :
                                (cfg_error_shown != 8'h00) ? cfg_error_shown :
                                (fb_error_shown != 8'h00)  ? fb_error_shown  :
                                (script_error != 8'h00)    ? script_error    : control_error;   
reg  [7:0] script_error_q;              // 讲解稿错误出现 → 数码管跳到 ERR 页
always @(posedge sys_clk or negedge ctrl_rst_n) begin
    if (!ctrl_rst_n) script_error_q <= 8'h00;
    else             script_error_q <= script_error;
end
wire       script_error_event = (script_error != 8'h00) && (script_error != script_error_q);
wire       status_error_event = error_event | boot_error_event | cfg_error_event | fb_error_event | script_error_event;

// 首图提交前黑屏；display_valid 只在场消隐期变化
assign vout_data_gated = display_valid ? vout_data_raw : 24'd0;

// ============================ 显示输出层 ============================
// sys 域状态 → pix 域（多比特走 toggle 握手，不逐位 2FF）
//   [8:1] 音量  [0] 自动轮播
// 去掉左上角状态行（osd_overlay_24 / status_osd_overlay：图片号、A/E、亮度档、忙、错误码），
//   图片号和手动 / 自动在参数条、数码管里都有，错误码仍在数码管 ERR 页；DIM_ENABLE=0 后它只剩状态行，
//   整个实例删掉，握手也只留音量 + 自动两项（原 20 位 → 9 位）
wire [8:0] status_pix;
cdc_bus_handshake #(
    .WIDTH     (9),
    .RESET_VAL ({8'd128, 1'b0})
) u_status_cdc (
    .src_clk   (sys_clk),
    .src_rst_n (ctrl_rst_n),
    .src_data  ({volume, auto_enabled}),
    .dst_clk   (pixel_clk),
    .dst_rst_n (pix_rst_n),
    .dst_data  (status_pix)
);
wire [7:0] volume_pix = status_pix[8:1];
assign vout_status = vout_data_gated;      

// ----  亮度 / 对比度参数面板（纯组合，0 拍）：调节模式中常显，数值变化后显示约 4s ----
//   显示 brightness_contrast_adjust 的本帧生效值（不是控制域的值），帧边界再锁存一次
wire [23:0] vout_adjust;
picture_adjust_osd #(
    .PANEL_X     (8),
    .PANEL_Y     (106),              
    .HOLD_FRAMES (240)
) u_adjust_osd (
    .clk         (pixel_clk),
    .rst_n       (pix_rst_n),
    .frame_start (frame_start),
    .vsync       (vs),
    .de          (de),
    .rgb_in      (vout_status),
    .brightness  (picture_pix[7:0]),   // 外接传感器：真实显示亮度（面板只在按键调节时弹出，环境自适应不弹），面板自己帧锁存
    .contrast    (contrast_frame_pix),
    .adj_mode    (picture_pix[17]),
    .adj_sel     (picture_pix[16]),
    .rgb_out     (vout_adjust),
    .osd_active  ()
);

// ----  多图层 OSD：参数条 + 滚动字幕 + 时间戳（Alpha 混合，纯组合，不改 vs/de 对齐）----
// 周期（0.1s 单位，sys 域慢变）单独走一个握手 CDC；亮度/音量/自动沿用上面的 status_pix
wire [8:0] period_tenths_pix;
cdc_bus_handshake #(
    .WIDTH     (9),
    .RESET_VAL (9'd50)
) u_period_cdc (
    .src_clk   (sys_clk),
    .src_rst_n (ctrl_rst_n),
    .src_data  (period_tenths),
    .dst_clk   (pixel_clk),
    .dst_rst_n (pix_rst_n),
    .dst_data  (period_tenths_pix)
);

wire        res_valid;
wire [11:0] res_width;
wire [11:0] res_height;
wire        pic_show;
picture_res_info u_pic_res (
    .sys_clk           (sys_clk),
    .sys_rst_n         (ctrl_rst_n),
    .load_done         (load_done),
    .loaded_buffer_idx (loaded_buffer_idx),
    .src_width         (src_width),
    .src_height        (src_height),
    .pix_clk           (pixel_clk),
    .pix_rst_n         (pix_rst_n),
    .display_valid     (display_valid),
    .thumb_display     (thumb_display),
    .active_buffer_idx (active_buffer_idx),
    .active_image_idx  (active_image_idx),
    .res_valid         (res_valid),
    .res_width         (res_width),
    .res_height        (res_height),
    .pic_show          (pic_show)
);

wire [9:0]  osd_pix_x, osd_pix_y;
wire        osd_frame_start;
wire        script_hit;
wire [23:0] script_color;
wire [7:0]  script_alpha;
wire        script_page_active;
wire        script_txt_hit;
wire [23:0] script_txt_color;
wire [7:0]  script_txt_alpha;
wire        voc_sample_cnt0;
wire        narr_pause_pix, narr_page_pix, ui_hide_pix;
wire        wav_tick;

osd_layer_mixer #(
    .H_ACTIVE          (640),
    .PARAM_X           (8),
    .PARAM_Y           (82),              
    .SUBTITLE_Y        (452),             // （SUBTITLE_EN=0 不用）
    .SUBTITLE_EN       (0),               
    .TS_ENABLE         (0),               // 去掉右上角 MM:SS 时间戳（连同计时器）
    .SCROLL_DIV        (3),               // 每 3 帧左移 1 像素（约 20 像素/秒）
    .CLK_HZ            (25_000_000),
    .PARAM_HOLD_FRAMES (240),              // 参数变化 / 切到新画面后显示约 4 秒再淡出；0 = 常显
    .PIPE              (1)               
) u_osd_layers (
    .clk              (pixel_clk),
    .rst_n            (pix_rst_n),
    .vsync            (vs),
    .de               (de),
    .rgb_in           (vout_adjust),        
    .subtitle_enable  (1'b0),
    .subtitle_text    (96'd0),
    .subtitle_length  (6'd0),
    .brightness       (picture_pix[7:0]),    // 外接传感器：真实显示亮度；变化即重新显示参数条，空闲约 4 s 隐藏
    .volume           (volume_pix),
    .period_tenths    (period_tenths_pix),
    .auto_enabled     (status_pix[0]),
    .res_valid        (res_valid),        
    .res_width        (res_width),
    .res_height       (res_height),
    .param_show       (pic_show),
    .param_hide       (script_page_active),  
    .ext_hit          (script_hit),          
    .ext_color        (script_color),
    .ext_alpha        (script_alpha),
    .txt_hit          (script_txt_hit),      
    .txt_color        (script_txt_color),
    .txt_alpha        (script_txt_alpha),
    .pix_x            (osd_pix_x),
    .pix_y            (osd_pix_y),
    .pix_frame_start  (osd_frame_start),
    .rgb_out          (vout_data),
    .overlay_active   ()
);

// ---- 讲解字幕图层（pix 域）：坐标 / 帧起点取 mixer 内的同一套，输出与像素同拍 ----
//   hide：总览（SW4）或首图提交前不显示；rgb_in = 混合前画面（自适应底板统计）
//   title_show = pic_show：切到新画面时标题 / 导语与参数条一起显示 HOLD 帧后淡出
script_osd #(
    .LAST_HOLD (18'd400),
    .PAGE_TOP  (10'd72),
    .HOLD      (8'd240)                   // = PARAM_HOLD_FRAMES
) u_script_osd (
    .clk         (pixel_clk),
    .rst_n       (pix_rst_n),
    .frame_start (osd_frame_start),
    .de          (de),
    .pixel_x     (osd_pix_x),
    .pixel_y     (osd_pix_y),
    .rgb_in      (vout_adjust),
    .rel_req_t   (scr_rel_req_t),
    .rel_ack_t   (scr_rel_ack_t),
    .ready_t     (scr_ready_t),
    .epoch_t     (scr_epoch_t),
    .line_re     (scr_line_re),
    .line_raddr  (scr_line_raddr),
    .line_rdata  (scr_line_rdata),
    .code_re     (scr_code_re),
    .code_raddr  (scr_code_raddr),
    .code_rdata  (scr_code_rdata),
    .glyph_re    (scr_glyph_re),
    .glyph_raddr (scr_glyph_raddr),
    .glyph_rdata (scr_glyph_rdata),
    .voice_cnt0  (voc_sample_cnt0),
    .tick48      (wav_tick),
    .pause       (narr_pause_pix),
    .page_on     (narr_page_pix),
    .hide        (thumb_display | ~display_valid),
    .marq_hide   (ui_hide_pix),           // trk16：KEY3 长按隐藏底部跑马灯
    .title_show  (pic_show),
    .bright_gain (bri_gain_pix),          // 亮度同步：跑马灯底板 / 文字随画面亮度
    .ext_hit     (script_hit),
    .ext_color   (script_color),
    .ext_alpha   (script_alpha),
    .txt_hit     (script_txt_hit),
    .txt_color   (script_txt_color),
    .txt_alpha   (script_txt_alpha),
    .page_active (script_page_active)
);

seg7_panel #(
    .CLK_HZ             (SYS_CLK_HZ),
    .SCAN_STEP_CYCLES   (16384),
    .PAGE_PERIOD_CYCLES (1000),              // 1s（L2：1ms tick 数）
    .HOLD_CYCLES        (2000),              // 2s
    .DIGIT_REVERSE      (0)
) u_seg7_panel (
    .clk              (sys_clk),
    .rst_n            (ctrl_rst_n),
    .tick             (sensor_tick_ms),
    .enable           (1'b1),
    .media_id         (current_image_idx),
    .media_valid      (display_valid_sys),
    .auto_enabled     (auto_enabled),
    .brightness       (eff_brightness),    // 外接传感器：数码管 BRI 页显示真实显示亮度
    .contrast         (contrast),
    .volume           (volume),
    .period_tenths    (period_tenths),
    .error_code       (status_error),
    .pic_event        (pic_event),
    .vol_event        (vol_page_event),
    .bri_event        (bri_event),
    .con_event        (con_event),
    .prd_event        (prd_event),
    .save_event       (cfg_save_ok),       // Flash 写入并读回校验成功才显示 SAVE
    .err_event        (status_error_event),
    .seg_n            (seg_data),
    .dig_n            (seg_sel)
);

// ===================== HDMI 音频 + 符号层====================
// video_x_tagger：把 hs/vs/de/rgb 打一拍并给出同拍列号 video_x；上电先等第一个 hs 下降沿锁定行、
//   第一个 vs 下降沿锁定帧，锁定前输出空闲电平（避免 color_bar 复位期的伪 vs / 不完整首帧）
// L21：osd_layer_mixer（PIPE=1）输出比 hs/vs/de 晚 2 拍，送 video_x_tagger 前同步延迟 2 拍
reg  [1:0]  hs_dly, vs_dly, de_dly;
always @(posedge pixel_clk or negedge pix_rst_n) begin
    if (!pix_rst_n) begin
        hs_dly <= 2'b11;                    // 与 rgb_video_adapter 复位电平一致（低有效同步，空闲 1）
        vs_dly <= 2'b11;
        de_dly <= 2'b00;
    end else begin
        hs_dly <= {hs_dly[0], hs};
        vs_dly <= {vs_dly[0], vs};
        de_dly <= {de_dly[0], de};
    end
end
wire        hs_t, vs_t, de_t;
wire [23:0] rgb_t;
wire [9:0]  video_x;

video_x_tagger u_video_x (
    .clk       (pixel_clk),
    .rst_n     (pix_rst_n),
    .hs_i      (hs_dly[1]),
    .vs_i      (vs_dly[1]),
    .de_i      (de_dly[1]),
    .rgb_i     (vout_data),
    .hs_o      (hs_t),
    .vs_o      (vs_t),
    .de_o      (de_t),
    .rgb_o     (rgb_t),
    .video_x_o (video_x),
    .locked_o  ()
);

// 音频全部在 pixel_clk(25MHz) 域：pcm_media_tone 分数累加得 48kHz 节拍 + 16 点正弦 LUT，
//   音色由 active_audio_idx 选择，换音轨时有提示音 blip；
//   m06_pcm_contract_adapter 检查 PCM 合同后交给 hdmi_audio_symbol_core 生成
//   视频/数据岛(音频采样包、ACR N=6144 CTS=25000、音频 InfoFrame)/控制符。
//   首图提交前 display_valid=0 → 音量 0 静音
wire [9:0] tmds_ch0_data;
wire [9:0] tmds_ch1_data;
wire [9:0] tmds_ch2_data;
wire [9:0] tmds_clk_data;

// ============================  WAV 音频播放：读侧（pixel_clk 域 = 音频链时钟）============================
sync_2ff u_sw2_sync (
    .clk      (pixel_clk),
    .rst_n    (pix_rst_n),
    .async_in (sw[1]),
    .sync_out (narr_pause_pix)
);
sync_2ff u_sw3_sync (
    .clk      (pixel_clk),
    .rst_n    (pix_rst_n),
    .async_in (sw[2]),
    .sync_out (narr_page_pix)
);
sync_2ff u_ui_hide_sync (
    .clk      (pixel_clk),
    .rst_n    (pix_rst_n),
    .async_in (ui_hide_sys),
    .sync_out (ui_hide_pix)
);

wire signed [15:0] wav_raw_sample;
wire               wav_raw_valid;
wire               wav_raw_ready;
wire               wav_stream_active;
wire               wav_stream_end;
wire               wav_switching_pix;
sync_2ff u_wav_switch_sync (
    .clk      (pixel_clk),
    .rst_n    (pix_rst_n),
    .async_in (wav_switching),
    .sync_out (wav_switching_pix)
);
// 换曲空档（停旧曲 → 新曲开播，以及曲终排空 → 重播）保持选 WAV：没有样本 = 静音，不漏出琶音
wire               wav_source_active = wav_stream_active | wav_stream_end | wav_switching_pix;
// 转场 alpha → 音频：手动淡入淡出与自动方块溶解都走 淡出 → 全黑 → 淡入，旧曲随画面淡出、新曲随画面淡入
wire [7:0]         wav_alpha_pix      = fade_alpha;
wire [7:0]         wav_fade_gain;
wire               wav_valid;
wire               wav_ready;
wire signed [15:0] wav_sample;

audio_sample_tick #(
    .CLOCK_HZ  (25_000_000),
    .SAMPLE_HZ (48_000)
) u_wav_tick (
    .clk   (pixel_clk),
    .rst_n (pix_rst_n),
    .tick  (wav_tick)
);

wav_stream_player #(
    .ADDR_WIDTH         (12),          // 4096 x 16 = 8KB ≈ 85ms @48kHz
    .SAMPLES_PER_SECTOR (256)
) u_wav_player (
    .wr_clk         (sys_clk),
    .wr_rst_n       (ctrl_rst_n),
    .play_enable    (wav_play_enable),
    .start_lba      (wav_start_lba),
    .total_sectors  (wav_total_sectors),
    .sd_req         (wav_sd_req),
    .sd_lba         (wav_sd_lba),
    .sd_busy        (wav_sd_busy),
    .sd_valid       (wav_sd_valid),
    .sd_byte        (wav_sd_byte),
    .sd_byte_index  (wav_sd_byte_index),
    .sd_done        (wav_sd_done),
    .sd_error       (wav_sd_error),
    .wr_idle        (wav_wr_idle),
    .wr_done        (wav_wr_done),
    .wr_error       (wav_wr_error),
    .rd_clk         (pixel_clk),
    .rd_rst_n       (pix_rst_n),
    .sample_tick    (wav_tick),
    .sample_out     (wav_raw_sample),
    .sample_valid   (wav_raw_valid),
    .sample_ready   (wav_raw_ready),
    .sample_counter (),                // OSD 进度显示接点
    .stream_active  (wav_stream_active),
    .stream_end     (wav_stream_end),
    .buffer_low     ()
);

sync_fade_gain #(
    .SYNC_STAGES (2)
) u_wav_fade_gain (
    .pix_clk     (pixel_clk),
    .pix_rst_n   (pix_rst_n),
    .frame_start (frame_start_raw),    // 与转场控制器同一个帧边界
    .alpha_pix   (wav_alpha_pix),
    .fade_active (fade_active),
    .aud_clk     (pixel_clk),
    .aud_rst_n   (pix_rst_n),
    .gain_aud    (wav_fade_gain)
);

// ---- 讲解人声：第二个 wav_stream_player（24kHz：每 2 个 48k 节拍取 1 个；SW2 暂停时不给节拍 → 停住）----
//   写侧由 script_loader 控制（仲裁器 C 口分时）；2KB 缓冲 ≈ 43 ms（24k）
reg  voc_tick_div;
always @(posedge pixel_clk or negedge pix_rst_n) begin
    if (!pix_rst_n)    voc_tick_div <= 1'b0;
    else if (wav_tick) voc_tick_div <= ~voc_tick_div;
end
wire               voc_tick = wav_tick && voc_tick_div && !narr_pause_pix;
wire signed [15:0] voc_sample;
wire               voc_valid;
wire               voc_stream_active;
wire [31:0]        voc_sample_counter;
assign voc_sample_cnt0 = voc_sample_counter[0];

wav_stream_player #(
    .ADDR_WIDTH         (10),          // 1024 x 16 = 2KB（2 个 BRAM9K）
    .SAMPLES_PER_SECTOR (256)
) u_voc_player (
    .wr_clk         (sys_clk),
    .wr_rst_n       (ctrl_rst_n),
    .play_enable    (voc_play_enable),
    .start_lba      (voc_start_lba),
    .total_sectors  (voc_total_sectors),
    .sd_req         (voc_sd_req),
    .sd_lba         (voc_sd_lba),
    .sd_busy        (voc_sd_busy),
    .sd_valid       (voc_sd_valid),
    .sd_byte        (wav_sd_byte),
    .sd_byte_index  (wav_sd_byte_index),
    .sd_done        (voc_sd_done),
    .sd_error       (narr_sector_error),
    .wr_idle        (voc_wr_idle),
    .wr_done        (voc_wr_done),
    .wr_error       (voc_wr_error),
    .rd_clk         (pixel_clk),
    .rd_rst_n       (pix_rst_n),
    .sample_tick    (voc_tick),
    .sample_out     (voc_sample),
    .sample_valid   (voc_valid),
    .sample_ready   (1'b1),            // 混音器每个 24k 节拍收一个
    .sample_counter (voc_sample_counter),   // [0] → 讲解时钟（只数真实样本）
    .stream_active  (voc_stream_active),
    .stream_end     (),
    .buffer_low     ()
);

//背景乐 × 音量 × 转场增益 + 人声（替代 wav_gain_stage；说话时背景乐平滑压到一半，1 个乘法器分时）
narration_audio_mixer #(
    .MSTEP (8'd4),
    .VSTEP (8'd16)
) u_narr_mix (
    .clk          (pixel_clk),
    .rst_n        (pix_rst_n),
    .frame_start  (frame_start_raw),
    .volume       (display_valid ? volume_pix : 8'd0),
    .fade_gain    (wav_fade_gain),
    .pause        (narr_pause_pix),
    .voice_active (voc_stream_active),
    .m_valid      (wav_raw_valid),
    .m_ready      (wav_raw_ready),
    .m_sample     (wav_raw_sample),
    .v_valid      (voc_valid),
    .v_ready      (),
    .v_sample     (voc_sample),
    .out_valid    (wav_valid),
    .out_ready    (wav_ready),
    .out_sample   (wav_sample),
    .ducking      ()
);

// ============================ 音频可视化============================

wire [23:0] rgb_wave;
wire [1:0]  audio_source_pix;          // 0 = 琶音, 1 = WAV, 2 = 测试音
wire        wave_pcm_valid = wav_valid & wav_ready;        // 混音器输出握手（含人声）
wire        wave_enable    = display_valid && (audio_source_pix == 2'd1) && !script_page_active && !tour_on_pix   
                            && !thumb_display && !zoom_active    // 缩略图总览 / SW4 缩放动画期间不画，只在全屏看图时显示
                            && !ui_hide_pix;                     // trk16：KEY3 长按隐藏

goertzel_spectrum_bars #(
    .BAR_X       (64),
    .BAR_Y       (380),               // trk16：380..443，让出底部讲解字幕带 446..477（字号 24）
    .FALL        (2),                 // 柱身每帧最多落 2 像素
    .HOLD        (18),                // 峰值帽停 0.3 s
    .QUIET       (60)                 // 静音 1 s 后收拢
) u_audio_wave (
    .clk             (pixel_clk),
    .rst_n           (pix_rst_n),
    .frame_start     (osd_frame_start),
    .de              (de_t),
    .rgb_in          (rgb_t),
    .pixel_x         (video_x),
    .pixel_y         (osd_pix_y),
    .enable          (wave_enable),
    .hue_shift       (ph_frame_cnt[6:2]),     // 复用总览呼吸帧计数：4 帧一步，32 步一圈约 2.1 s
    .bright_gain     (bri_gain_pix),          // 亮度同步：柱色随画面亮度
    .pcm_tap_valid   (wave_pcm_valid),
    .pcm_tap_left    (wav_sample),             
    .rgb_out         (rgb_wave)
);

media_hdmi_audio_bridge #(
    .TONE_EXT_TICK      (1)                   
) u_audio (
    .pixel_clk          (pixel_clk),
    .pixel_rst_n        (pix_rst_n),
    .tone_tick          (wav_tick),
    .video_hsync        (hs_t),
    .video_vsync        (vs_t),
    .video_de           (de_t),
    .video_x            (video_x),
    .video_rgb          (rgb_wave),           
    .media_id           (active_audio_idx),
    .volume             (display_valid ? volume_pix : 8'd0),
    .wav_valid          (wav_valid),          
    .wav_ready          (wav_ready),
    .wav_sample         (wav_sample),
    .wav_active         (wav_source_active),
    .force_arpeggio     (1'b0),               
    .test_tone_enable   (1'b0),
    .audio_source       (audio_source_pix),   
    .channel0_symbol    (tmds_ch0_data),
    .channel1_symbol    (tmds_ch1_data),
    .channel2_symbol    (tmds_ch2_data),
    .sequence_error     (),
    .tone_overflow      (),
    .pcm_contract_error (),
    .blip_active        (),
    .pcm_tap_valid      (),
    .pcm_tap_left       ()
);

hdmi_tmds_clock_channel u_tmds_clk (
    .pixel_clk (pixel_clk),
    .rst_n     (pix_rst_n),
    .symbol    (tmds_clk_data)
);

hdmi_phy_wrapper #(
    .DEVICE ( "EG" )
) u_hdmi2phy_wrapper(
    .I_pixel_clk        (pixel_clk),
    .I_serial_clk       (pixel_clk_5x),
    .I_rst_n            (pix_rst_n),
    .I_serial_rst_n     (pix_5x_rst_n),
    .I_tmds_channel_0   (tmds_ch0_data),
    .I_tmds_channel_1   (tmds_ch1_data),
    .I_tmds_channel_2   (tmds_ch2_data),
    .I_tmds_channel_clk (tmds_clk_data),
    .O_tmds_ch0_p       (HDMI_D0_P),
    .O_tmds_ch1_p       (HDMI_D1_P),
    .O_tmds_ch2_p       (HDMI_D2_P),
    .O_tmds_clk_p       (HDMI_CLK_P)
);

// ===================== EDID / DDC =====================
assign HDMI_DDC_SCL = 1'bz;
assign HDMI_DDC_SDA = 1'bz;

endmodule
