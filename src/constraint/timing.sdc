# ============================================================
# 单 PLL sys_pll_promax_40（实例 u_mem_pll，原语 pll_inst）：
#   refclk 50MHz → VCO 1000MHz
#   clkc[0] 125MHz 0°   pixel_clk_5x（HDMI 10:1 串行，反馈输出）
#   clkc[1]  40MHz 0°   mem_clk（SDRAM 控制器）
#   clkc[2]  40MHz 144° mem_clk_shift（只驱动片内 SDRAM 硬核采样时钟）
#   clkc[3]  25MHz 0°   pixel_clk
#   clkc[4] 未用
# sys_clk = 板载 50MHz 直接用（控制 / SD / 数码管）
#
# 只对每一个"已经走同步器"的跨域点单独 false_path / max_delay，其余跨域全部照常分析
# （新增的跨域逻辑没走同步器会直接报违例，而不是被整组藏掉）。
# 相位说明： txt/sdc 写 clk2 172.8°，与 IP 参数(CPHASE 9 / DIV 25 → 10ns)不符；
# ============================================================

# ------------------------------------------------------------
# 1. 时钟
# ------------------------------------------------------------
create_clock -name sys_clk -period 20.000 -waveform {0.000 10.000} [get_ports {clk}]
create_generated_clock -name pixel_clk_5x  -source [get_ports {clk}] -master_clock sys_clk -multiply_by 2.5 [get_pins {u_mem_pll/pll_inst.clkc[0]}]
create_generated_clock -name mem_clk       -source [get_ports {clk}] -master_clock sys_clk -divide_by 1.25 -phase 0 [get_pins {u_mem_pll/pll_inst.clkc[1]}]
create_generated_clock -name mem_clk_shift -source [get_ports {clk}] -master_clock sys_clk -divide_by 1.25 -phase 144 [get_pins {u_mem_pll/pll_inst.clkc[2]}]
create_generated_clock -name pixel_clk     -source [get_ports {clk}] -master_clock sys_clk -divide_by 2 [get_pins {u_mem_pll/pll_inst.clkc[3]}]

# pixel_clk ↔ pixel_clk_5x：同一 PLL 同相 25/125MHz，lane_lvds_10_1 在两者间搬 TMDS 符号，
# 必须做同步时序分析 —— 不加任何例外。

# ------------------------------------------------------------
# 2. SDRAM 移位时钟
# ------------------------------------------------------------
set_false_path -from [get_clocks {mem_clk}]       -to [get_clocks {mem_clk_shift}]
set_false_path -from [get_clocks {mem_clk_shift}] -to [get_clocks {mem_clk}]

# ------------------------------------------------------------
# 3. 保持时间不确定性：单 PLL 同一全局树，0.05ns 仍留裕量
# ------------------------------------------------------------
set_clock_uncertainty -hold 0.050 [get_clocks {sys_clk pixel_clk pixel_clk_5x mem_clk mem_clk_shift}]

# ------------------------------------------------------------
# 4. 异步输入与复位
#    ext_rst_n = key1_rst_n & locked，只进各域 reset_sync 的异步端（异步压下、本域同步释放）
#    KEY2..KEY4 进 single_key_event / dual_key_long_hold_guard 的 2FF
# ------------------------------------------------------------
set_false_path -from [get_ports {key1_rst_n}]
set_false_path -from [get_ports {key2_n key3_n key4_n}]
set_false_path -from [get_ports {sd_miso}]
# 用户 Flash MISO：player_config_persistence 在 SCK 下降沿后 60ns（3 拍）才采样，时序由协议保证
set_false_path -from [get_ports {flash_miso}]
# 外接传感器：BH1750 IIC 100kHz（每位 4 个 2.5us 相位，SDA 在 SCL 高的中点采样）、YL-62 OUT 进 2FF + 50ms 消抖
set_false_path -from [get_ports {als_sda ir_out_n}]
set_false_path -to   [get_ports {als_scl als_sda}]

# ------------------------------------------------------------
# 5. 同步器第一级（异步采样点）
#    reset_sync / sync_2ff                         : */s1
#    cdc_bus_handshake / media_player_control      : req_sync1 / ack_sync1 / commit_sync1

# ------------------------------------------------------------
set_false_path -to [get_regs -hier {*/s1}]
set_false_path -to [get_regs -hier {*/req_sync1}]
set_false_path -to [get_regs -hier {*/ack_sync1}]
set_false_path -to [get_regs -hier {*/commit_sync1}]
set_false_path -to [get_regs -hier {*/request_sync1_mem}]
set_false_path -to [get_regs -hier {*/index_sync1_mem[*]}]
set_false_path -to [get_regs -hier {*/done_sync1}]
# 准静态载荷：toggle 同步后（≥2 拍）才采样，源端在下一次 toggle 前不变
#   load CDC 的错误码（mem → sys）、reader 错误码握手的 dst_data（mem → sys）
set_false_path -from [get_regs -hier {u_load_cdc/done_error_mem[*]}] -to [get_regs -hier {u_load_cdc/done_error[*]}]
set_false_path -to [get_regs -hier {u_reader_err_cdc/dst_data[*]}]

# ------------------------------------------------------------
# 6. HDMI 串行输出：5x 时钟 → ODDR → 引脚，逐通道限定数字延迟
# ------------------------------------------------------------
set_max_delay -from [get_clocks {pixel_clk_5x}] \
    -to [get_ports {HDMI_CLK_P HDMI_D0_P HDMI_D1_P HDMI_D2_P}] \
    8.000 -datapath_only

# ------------------------------------------------------------
#    （替换原 frame_read_write 的 write_buf / read_buf 两条）
#    u_load_cdc/u_cmd_fifo   wfifo_32_32_512(256 深) : 写 sys_clk(sd_card_bmp 像素流) / 读 mem_clk(loader)
#    u_reader/u_read_fifo    rfifo_32_32_512(512 深) : 写 mem_clk(SDRAM 读数据)       / 读 pixel_clk
#    不整域屏蔽，只把 Gray 指针跨域限定在一个源时钟周期内(-datapath_only)，
#    保证多 bit 指针到达第一级同步器时的偏斜 < 1 个源周期（Gray 码只变 1 位）。
#    保持检查：sys_clk 直接来自引脚、PLL 输出有反馈补偿，两者插入延迟差 ~1.9ns，
#    异步第一级的 hold 没有意义，一并去掉（只去 hold，max_delay 照常检查）。
# ------------------------------------------------------------
set_max_delay -from [get_regs -hier {u_load_cdc/u_cmd_fifo/wr_to_rd_cross_inst/primary_addr_gray_reg[*]}] -to [get_regs -hier {u_load_cdc/u_cmd_fifo/wr_to_rd_cross_inst/sync_r1[*]}] 20.000 -datapath_only
set_max_delay -from [get_regs -hier {u_load_cdc/u_cmd_fifo/rd_to_wr_cross_inst/primary_addr_gray_reg[*]}] -to [get_regs -hier {u_load_cdc/u_cmd_fifo/rd_to_wr_cross_inst/sync_r1[*]}] 25.000 -datapath_only
set_max_delay -from [get_regs -hier {u_reader/u_read_fifo/wr_to_rd_cross_inst/primary_addr_gray_reg[*]}]  -to [get_regs -hier {u_reader/u_read_fifo/wr_to_rd_cross_inst/sync_r1[*]}]  25.000 -datapath_only
set_max_delay -from [get_regs -hier {u_reader/u_read_fifo/rd_to_wr_cross_inst/primary_addr_gray_reg[*]}]  -to [get_regs -hier {u_reader/u_read_fifo/rd_to_wr_cross_inst/sync_r1[*]}]  40.000 -datapath_only
set_false_path -hold -to [get_regs -hier {u_load_cdc/u_cmd_fifo/*_cross_inst/sync_r1[*]}]
set_false_path -hold -to [get_regs -hier {u_reader/u_read_fifo/*_cross_inst/sync_r1[*]}]

# FIFO 异步清零：只进 afifo 内部 asy_w_rst0/1、asy_r_rst0/1（异步置位、本侧时钟同步释放）和
# reader 的 2 级预取缓冲（释放由 fifo_release_pix 在 pix 域同步给出）。
#   u_load_cdc  : rst = !ctrl_rst_n | !mem_rst_n（上电/KEY1 复位时两侧都在复位）
#   u_reader    : rst = !mem_rst_n | !pix_rst_n | 帧重载清零计数(mem) | 两侧释放握手
set_false_path -from [get_regs -hier {u_ctrl_reset/s2}] -to [get_clocks {mem_clk}]
set_false_path -from [get_regs -hier {u_mem_reset/s2}]  -to [get_clocks {sys_clk pixel_clk}]
set_false_path -from [get_regs -hier {u_pix_reset/s2}]  -to [get_clocks {mem_clk}]
set_false_path -from [get_regs -hier {u_reader/fifo_reset_count_mem[*]}] -to [get_clocks {pixel_clk}]
set_false_path -from [get_regs -hier {u_reader/fifo_release_mem[*]}]     -to [get_clocks {pixel_clk}]
set_false_path -from [get_regs -hier {u_reader/fifo_release_pix[*]}]     -to [get_clocks {mem_clk}]

# ------------------------------------------------------------
#    指针跨域限定在一个源时钟周期内（setup 照常检查），第一级去掉 hold。
#    单 bit 电平握手（play_enable/写结束 → 读侧，读侧就位/排空 → 写侧）只进第一级，同 */s1。
# ------------------------------------------------------------
set_max_delay -from [get_regs -hier {u_wav_player/wr_gray[*]}] -to [get_regs -hier {u_wav_player/wr_gray_sync1[*]}] 20.000 -datapath_only
set_max_delay -from [get_regs -hier {u_wav_player/rd_gray[*]}] -to [get_regs -hier {u_wav_player/rd_gray_sync1[*]}] 40.000 -datapath_only
set_false_path -hold -to [get_regs -hier {u_wav_player/wr_gray_sync1[*]}]
set_false_path -hold -to [get_regs -hier {u_wav_player/rd_gray_sync1[*]}]
set_false_path -to [get_regs -hier {u_wav_player/en_sync1}]
set_false_path -to [get_regs -hier {u_wav_player/ended_sync1}]
set_false_path -to [get_regs -hier {u_wav_player/armed_sync1}]
set_false_path -to [get_regs -hier {u_wav_player/drained_sync1}]

# ------------------------------------------------------------
#    寄存器在 generate 块 g_fade 里，TD 网表名是 g_fade$commit_sync1 之类，第 5 节的 */commit_sync1、
#    */req_sync1 匹配不到（分隔符是 $ 不是 /），GUI 默认种子下报 hold -1.42ns。这里按实例单独列出第一级：
#    ctrl(sys) 侧 commit/busy/seen_sync1，pix 侧 req/abort_sync1；toggle 源在下一次翻转前不变。
# ------------------------------------------------------------
set_false_path -to [get_regs -hier {u_fade_transition/*commit_sync1}]
set_false_path -to [get_regs -hier {u_fade_transition/*busy_sync1}]
set_false_path -to [get_regs -hier {u_fade_transition/*seen_sync1}]
set_false_path -to [get_regs -hier {u_fade_transition/*req_sync1}]
set_false_path -to [get_regs -hier {u_fade_transition/*abort_sync1}]
# 2026-10-18 自动轮播方块溶解：req_auto（sys）在接受请求时与 req_toggle 同拍写入，像素域等 toggle 过 2FF 后才采样
#   进 dissolve_mode（准静态，下次接受请求前不变）
set_false_path -from [get_regs -hier {u_fade_transition/*req_auto}] -to [get_clocks {pixel_clk}]

# ------------------------------------------------------------
#    u_thumb_img_cdc    cdc_bus_handshake 6 位  : 本次加载图片号 sys_clk → mem_clk（写入器在第一个像素才采样）
#    u_thumb_status_cdc cdc_bus_handshake 20 位 : 格子 valid + 页号 mem_clk → pixel_clk
#    req/ack 第一级已被第 5 节 */req_sync1、*/ack_sync1 覆盖；载荷 dst_data 是准静态总线
#    （req 同步 2 拍后才采样，源端 src_hold 在应答前不变），同 u_reader_err_cdc/dst_data
#    SW4（u_sw4_sync）走 sync_2ff，已被 */s1 覆盖；thumb_display → reader 帧请求走 index_sync1_mem（已覆盖）
# ------------------------------------------------------------
set_false_path -to [get_regs -hier {u_thumb_img_cdc/dst_data[*]}]
set_false_path -to [get_regs -hier {u_thumb_status_cdc/dst_data[*]}]

# ------------------------------------------------------------
#    u_picture_cdc cdc_bus_handshake 18 位 : {调节模式, 当前项, 对比度, 亮度} sys_clk → pixel_clk
#    req/ack 第一级已被第 5 节 */req_sync1、*/ack_sync1 覆盖；载荷 dst_data 是准静态总线
#    （req 同步 2 拍后才采样，源端 src_hold 在应答前不变），pix 侧再由 brightness_contrast_adjust
#    在 frame_start 锁存一次。按键电平 key3/key4_stable_n 本来就在 sys 域，不跨域
# ------------------------------------------------------------
set_false_path -to [get_regs -hier {u_picture_cdc/dst_data[*]}]

# ------------------------------------------------------------
#    u_thumb_ctrl   thumbnail_controller 自带 toggle 握手 : {view_mode, focus} sys_clk → pixel_clk
#                   req_sync1 / ack_sync1 第一级已被第 5 节 */req_sync1、*/ack_sync1 覆盖；
#                   lat_pix 在 req 同步 2 拍后才采样 src_hold（应答前不变），准静态载荷
#    u_img_count_cdc cdc_bus_handshake 7 位 : 图片数 sys_clk → pixel_clk（占位格判定，慢变）
# ------------------------------------------------------------
set_false_path -to [get_regs -hier {u_thumb_ctrl/lat_pix[*]}]
set_false_path -to [get_regs -hier {u_img_count_cdc/dst_data[*]}]

# ------------------------------------------------------------
#    u_thumb_img_cdc 加宽到 7 位（{load_thumb_gen, 图片号}）、u_thumb_status_cdc 加宽到 40 位（8 条目 valid + 页号），
#      仍由第 10 节 dst_data[*] 覆盖
#    u_thumb_view_back：pix 域 thumb_view_settled → sys 域 sync_2ff（总览翻页规则用），已被第 5 节 */s1 覆盖
# ------------------------------------------------------------

# ------------------------------------------------------------
# 14. 第十六讲 参数条分辨率的跨域点
#    u_pic_res（picture_res_info）: entry_sys {bank, 宽, 高}（sys_clk，每次加载成功才变）→ pixel_clk 写分辨率表
#    翻转位 entry_tog → req_sync1/req_sync2，第一级已被第 5 节 */req_sync1 覆盖；pix 侧看到翻转时 entry_sys
#    已稳定 >= 2 个像素时钟，到下一次 load_done（毫秒级）之前不变 → 准静态载荷，同第 10 节 dst_data
#    u_status_cdc 缩成 9 位（{音量, 自动}），沿用原约束
# ------------------------------------------------------------
set_false_path -from [get_regs -hier {u_pic_res/entry_sys[*]}]

# ------------------------------------------------------------
# 15. 扩1 讲解字幕的跨域点
#    u_voc_player（第二个 wav_stream_player，人声 24k）：写侧 sys_clk / 读侧 pixel_clk，做法同第 8 节
#    u_script_osd（pix）← u_script_loader（sys）三个 toggle：rel_req → req_sync1（第 5 节已覆盖）、
#      ready → rdy_sync1、epoch → epo_sync1；回执 rel_ack → loader 的 ack_sync1（第 5 节已覆盖）
#    SW2 / SW3 / 隐藏开关（u_sw2_sync / u_sw3_sync / u_ui_hide_sync）走 sync_2ff，已被第 5 节 */s1 覆盖
#    三块讲解 RAM（script_dc_ram ×2、script_glyph_bram32k）sys 写 / pix 读：loader 只在 pix 回了释放握手
#      （文字全淡出、不再读）之后才写，ready 翻转后 pix 才读 → 两侧时间上不重叠，BRAM 内部不跨域分析
# ------------------------------------------------------------
set_max_delay -from [get_regs -hier {u_voc_player/wr_gray[*]}] -to [get_regs -hier {u_voc_player/wr_gray_sync1[*]}] 20.000 -datapath_only
set_max_delay -from [get_regs -hier {u_voc_player/rd_gray[*]}] -to [get_regs -hier {u_voc_player/rd_gray_sync1[*]}] 40.000 -datapath_only
set_false_path -hold -to [get_regs -hier {u_voc_player/wr_gray_sync1[*]}]
set_false_path -hold -to [get_regs -hier {u_voc_player/rd_gray_sync1[*]}]
set_false_path -to [get_regs -hier {u_voc_player/en_sync1}]
set_false_path -to [get_regs -hier {u_voc_player/ended_sync1}]
set_false_path -to [get_regs -hier {u_voc_player/armed_sync1}]
set_false_path -to [get_regs -hier {u_voc_player/drained_sync1}]
set_false_path -to [get_regs -hier {u_script_osd/rdy_sync1}]
set_false_path -to [get_regs -hier {u_script_osd/epo_sync1}]

# ------------------------------------------------------------
# 拓展3做深 ② 导览运镜
#    u_tour_seq/bq_* 视角 bundle 81 位 sys_clk → pixel_clk 直连：sequencer 只在 frame_tick（= pix frame_start
#                    经 2FF 同步）之后几 µs 内写，pix 在 frame_start 锁存 → 写入与采样相隔 ≥ 2 个 sys 周期、≤ 一帧
#    u_tour_bank_cdc cdc_bus_handshake 2 位  : 显示 bank pixel_clk → sys_clk
#    u_tour_view/mq_* : 行 DMA 请求载荷 pixel_clk → mem_clk（rq_t 翻转同步 2 拍后才采样，pix 侧在 ack 前不改）
#    req/ack 第一级已被第 5 节 */req_sync1、*/ack_sync1 覆盖；帧节拍 toggle 走 sync_2ff（*/s1）
# ------------------------------------------------------------
set_false_path -from [get_regs -hier {u_tour_seq/bq_*}]
set_false_path -to [get_regs -hier {u_tour_bank_cdc/dst_data[*]}]
set_false_path -to [get_regs -hier {u_tour_view/mq_*}]
