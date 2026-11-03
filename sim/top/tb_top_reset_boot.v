`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// tb_top_reset_boot：验证用例的整板复现（DUT = 完整 top，环境同 tb_top_ex4_hdmi_output）
//   引脚级 HDMI 检查器 models/tb_top_hdmi_checker.vh 全程在线：每次启动后每一帧都
//   像素级比对 BMP、提交前黑屏+静音、数据岛 ECC、时钟通道；复位窗口内暂停(chk_en=0)。
//   用例对应（2.8 表）：
//   [4] PLL 未锁定复位 : 上电 KEY1 按住期间 PLL 也在复位(不锁定)；任何时刻
//                        !(KEY1 & 全部 PLL locked) → 4 个域复位立即(异步)有效（连续监视 M1）
//                        复位只在全部锁定后、各域本域时钟沿释放（M2）
//   [5] SDRAM 初始化等待: 上电时让 SDRAM 初始化完成信号晚到 25 ms（SD 卡早已初始化完）
//                        期间不得开始扫描/发 CMD17；启动顺序时间戳全部单调（M3/M4）
//   [1] 正常启动        : 首图像素级正确
//   [2] 运行中复位      : 按住 KEY1 2 ms → 画面消失(display_valid=0、TMDS 引脚停摆)，
//                        松开后重新启动并正确显示
//   [3] 连续复位 10 次  : 间隔 5 us..100 ms，覆盖 PLL 锁定/SDRAM 初始化/SD 初始化/扫描/
//                        加载中各阶段，含一次按键抖动(1 us 级毛刺串)；最后一次松开后正常显示
//   [6] 失锁复位        : 运行中 PLL 30 ns 失锁毛刺 → 全系统复位；重启途中同一 PLL
//                        再失锁 5 us → 再次复位；恢复后正常显示（整板仿真里代替"断电重启"）
//     启动顺序加入 player_enable（扫描确认首图 + 写口排空后）；结尾检查 SDRAM 自刷新间隔
// -----------------------------------------------------------------------------
module tb_top_reset_boot;
    `include "tb_hdmi_common.vh"

    //   最小改动版是 sys_pll_m0/video_pll_m0 两个 PLL(extlock)、Sdr_init_done_sd、hdmi_5x_clk/video_clk
    `define TB_LOCKED      (dut.mem_locked === 1'b1)
    `define TB_RSTN        {dut.ctrl_rst_n, dut.mem_rst_n, dut.pix_rst_n, dut.pix_5x_rst_n}
    `define TB_SDR_DONE    dut.Sdr_init_done
    `define TB_SDR_DONE_S  dut.sdram_init_done_sys
    `define TB_SCAN_KICK   dut.sd_card_bmp_m0.scan_kicked
    `define TB_SD_INIT     dut.sd_card_bmp_m0.sd_init_done
    `define TB_PLL         dut.u_mem_pll
    `define TB_PLL_LOCK    dut.u_mem_pll.locked
    `define TB_PLAYER_EN   dut.player_enable
    `define TB_SER_CLK     dut.pixel_clk_5x
    `define TB_PIX_CLK     dut.pixel_clk
    // sys_clk = 板载 50MHz → SPI 高速 12.5MHz(最小改动版 100MHz/25MHz)，首图约 1.3 s
    localparam [63:0] BOOT_TIMEOUT_NS = 64'd1_600_000_000;

    reg clk = 1'b0;
    always #10 clk = ~clk;                   // 板载 50 MHz
    reg key1_rst_n = 1'b0;
    reg key2_n = 1'b1, key3_n = 1'b1, key4_n = 1'b1;
    wire [7:0] seg_sel, seg_data;
    wire HDMI_CLK_P, HDMI_D2_P, HDMI_D1_P, HDMI_D0_P, HDMI_DDC_SCL;
    wire HDMI_DDC_SDA;
    wire sd_ncs, sd_dclk, sd_mosi, sd_miso;
    pullup (HDMI_DDC_SDA);

    top dut (
        .clk(clk), .key1_rst_n(key1_rst_n), .key2_n(key2_n), .key3_n(key3_n), .key4_n(key4_n), .sw(4'b0000),
        .seg_sel(seg_sel), .seg_data(seg_data),
        .HDMI_CLK_P(HDMI_CLK_P), .HDMI_D2_P(HDMI_D2_P), .HDMI_D1_P(HDMI_D1_P), .HDMI_D0_P(HDMI_D0_P),
        .HDMI_DDC_SCL(HDMI_DDC_SCL), .HDMI_DDC_SDA(HDMI_DDC_SDA),
        .sd_ncs(sd_ncs), .sd_dclk(sd_dclk), .sd_mosi(sd_mosi), .sd_miso(sd_miso)
    );

    sd_card_spi_model #(.IMG0_SECTOR(32'd0), .IMG1_SECTOR(32'd1811)) u_sd (
        .nCS(sd_ncs), .DCLK(sd_dclk), .MOSI(sd_mosi), .MISO(sd_miso)
    );

    // 与 tb_top_ex4_hdmi_output 相同的仿真加速/悬空位钳位（RTL 不变）
    defparam dut.sd_card_bmp_m0.SCAN_MAX_SECTOR = 32'd3620;
    initial begin
        force dut.frame_read_write_m0.frame_fifo_write_m0.rdusedw[9] = 1'b0;
        force dut.frame_read_write_m0.frame_fifo_read_m0.wrusedw[9]  = 1'b0;
    end

    `include "tb_top_hdmi_checker.vh"

    // ======================= 连续监视 =======================
    // M1：复位源无效 → 所有域复位 0.5 ns 内有效（异步压下，不等时钟）
    wire src_ok = (key1_rst_n === 1'b1) && `TB_LOCKED;
    integer m1_bad = 0, m2_bad = 0, m3_bad = 0, m4_bad = 0;
    always @(negedge src_ok) begin
        #0.5;
        if (!src_ok && `TB_RSTN !== 4'b0000) begin
            m1_bad = m1_bad + 1;
            if (m1_bad <= 5) $display("FAIL @%0t: M1 reset source dropped but domain resets = %b", $time, `TB_RSTN);
        end
    end
    // M2：任何一个域退出复位时，复位源必须有效（PLL 已锁定）
    always @(`TB_RSTN) if (`TB_RSTN !== 4'b0000 && !src_ok) begin
        m2_bad = m2_bad + 1;
        if (m2_bad <= 5) $display("FAIL @%0t: M2 domain reset released while PLL unlocked / KEY1 low (%b)", $time, `TB_RSTN);
    end
    // M3：扫描启动时 SDRAM 必须已初始化完成（原始信号与同步后信号都为 1）
    always @(posedge `TB_SCAN_KICK) if (`TB_SDR_DONE !== 1'b1 || `TB_SDR_DONE_S !== 1'b1) begin
        m3_bad = m3_bad + 1; $display("FAIL @%0t: M3 scan started before SDRAM init done", $time);
    end
    // M4：CMD17（读扇区）只能在扫描启动之后出现
    always @(u_sd.n_cmd17) if (u_sd.n_cmd17 != 0 && `TB_SCAN_KICK !== 1'b1) begin
        m4_bad = m4_bad + 1; if (m4_bad <= 5) $display("FAIL @%0t: M4 CMD17 issued before the scan was started", $time);
    end

    // M5：画面/声音只能在启动序列放行之后出现
    integer m5_bad = 0;
    always @(posedge dut.display_valid) if (`TB_PLAYER_EN !== 1'b1) begin
        m5_bad = m5_bad + 1; $display("FAIL @%0t: M5 display_valid before player_enable", $time);
    end

    // 启动顺序时间戳（每次复位源下降清零）
    time ts_lock = 0, ts_rst = 0, ts_sdr = 0, ts_sdr_s = 0, ts_kick = 0, ts_cmd17 = 0, ts_pe = 0, ts_dv = 0;
    always @(negedge src_ok) begin ts_lock = 0; ts_rst = 0; ts_sdr = 0; ts_sdr_s = 0; ts_kick = 0; ts_cmd17 = 0; ts_pe = 0; ts_dv = 0; end
    always @(posedge `TB_PLAYER_EN)     if (ts_pe    == 0) ts_pe    = $time;
    always @(posedge src_ok)            if (ts_lock  == 0) ts_lock  = $time;
    always @(`TB_RSTN) if (`TB_RSTN === 4'b1111 && ts_rst == 0) ts_rst = $time;
    always @(posedge `TB_SDR_DONE)      if (ts_sdr   == 0) ts_sdr   = $time;
    always @(posedge `TB_SDR_DONE_S)    if (ts_sdr_s == 0) ts_sdr_s = $time;
    always @(posedge `TB_SCAN_KICK)     if (ts_kick  == 0) ts_kick  = $time;
    always @(u_sd.n_cmd17)              if (ts_cmd17 == 0 && src_ok) ts_cmd17 = $time;
    always @(posedge dut.display_valid) if (ts_dv    == 0) ts_dv    = $time;

    // 复位保持期间 TMDS 引脚翻转计数
    integer pin_toggles = 0;
    always @(HDMI_CLK_P or HDMI_D0_P or HDMI_D1_P or HDMI_D2_P) pin_toggles = pin_toggles + 1;

    initial forever begin
        #20_000_000;
        $display("INFO t=%0d ms rstn=%b locked=%0d sdr=%0d kick=%0d CMD17=%0d scan_done=%0d images=%0d dv=%0d chars=%0d",
                 $time / 1000000, `TB_RSTN, `TB_LOCKED, `TB_SDR_DONE, `TB_SCAN_KICK, u_sd.n_cmd17, dut.scan_done,
                 dut.image_count, dut.display_valid, n_chars);
    end

    // ======================= 任务 =======================
    integer boots_ok = 0, img0_before, k, pt;
    time t_to;

    // 等一次完整启动：顺序正确、首图提交、之后 3 帧纯图 0 像素级正确
    task boot_and_check(input [8*24-1:0] tag);
        begin
            chk_en = 1'b1;
            t_to = $time + BOOT_TIMEOUT_NS;
            wait (dut.display_valid === 1'b1 || $time > t_to);
            if (dut.display_valid !== 1'b1) begin
                err("boot timeout: no image committed");
            end else begin
                img0_before = frames_img0;
                wait (frames_img0 >= img0_before + 3 || $time > t_to + 100_000_000);
                if (frames_img0 < img0_before + 3) err("booted but fewer than 3 pixel-exact image-0 frames");
                else boots_ok = boots_ok + 1;
            end
            if (!(ts_lock > 0 && ts_rst > ts_lock && ts_sdr > ts_rst && ts_sdr_s > ts_sdr && ts_kick > ts_sdr_s &&
                  ts_cmd17 > ts_kick && ts_pe > ts_cmd17 && ts_dv > ts_pe))
                err("boot order violated (lock < rst < sdram < sdram_sys < scan < CMD17 < player_enable < display)");
            $display("INFO boot %0s: lock %0d ns | all rst released +%0d ns | sdram init +%0d us | sync +%0d ns | scan +%0d us | CMD17 +%0d us | player_enable +%0d ms | display +%0d ms",
                     tag, ts_lock, ts_rst - ts_lock, (ts_sdr - ts_rst) / 1000, ts_sdr_s - ts_sdr,
                     (ts_kick - ts_sdr_s) / 1000, (ts_cmd17 - ts_kick) / 1000, (ts_pe - ts_cmd17) / 1000000,
                     (ts_dv - ts_pe) / 1000000);
        end
    endtask

    // 运行中按住 KEY1 hold_ns：画面必须消失、PLL 复位失锁、TMDS 引脚停摆
    task press_reset(input integer hold_ns);
        begin
            chk_en = 1'b0;
            key1_rst_n = 1'b0;
            #1000;
            if (dut.display_valid !== 1'b0) err("display still valid 1 us after KEY1 pressed");
            if (`TB_PLL_LOCK !== 1'b0) err("PLL not held in reset while KEY1 pressed");
            pt = pin_toggles;
            #(hold_ns - 1000);
            if (pin_toggles != pt) err("TMDS pins still toggling while held in reset");
            key1_rst_n = 1'b1;
        end
    endtask

    integer highs [0:9];
    initial begin
        highs[0] = 5_000;       highs[1] = 50_000;      highs[2] = 200_000;     // PLL 锁定前后 / SDRAM 初始化中
        highs[3] = 1_000_000;   highs[4] = 3_000_000;   highs[5] = 8_000_000;   // SD 初始化中
        highs[6] = 15_000_000;  highs[7] = 30_000_000;  highs[8] = 60_000_000;  // 扫描 / 加载中
        highs[9] = 100_000_000;
    end

    initial begin
        // ---------- [4] 上电：KEY1 按住 5 us，期间 PLL 复位不锁定 ----------
        chk_en = 1'b1;
        force `TB_SDR_DONE = 1'b0;                 // [5] SDRAM 初始化完成晚到
        #4000;
        if (`TB_PLL_LOCK !== 1'b0) err("[4] PLL locked while its reset (KEY1) was held");
        if (`TB_RSTN !== 4'b0000) err("[4] domains out of reset while KEY1 held");
        #1000 key1_rst_n = 1'b1;
        #500;
        if (`TB_RSTN !== 4'b0000) err("[4] domains out of reset before PLL lock");
        // ---------- [5] SDRAM 初始化等待 ----------
        //   SDRAM 超时)，再多压 5 ms；期间 boot_state 必须停在 WAIT_SDRAM、ingest_enable=0
        t_to = $time + 80_000_000;
        wait (`TB_SD_INIT === 1'b1 || $time > t_to);
        #5_000_000;
        if (`TB_SD_INIT !== 1'b1) err("[5] SD card should have initialised within 80 ms");
        if (`TB_SCAN_KICK !== 1'b0 || u_sd.n_cmd17 != 0) err("[5] scan / CMD17 before SDRAM init done");
        if (dut.boot_state !== 3'd0 || dut.ingest_enable !== 1'b0) err("[5] boot_sequencer left WAIT_SDRAM before SDRAM init");
        if (dut.boot_error !== 8'h00) err("[5] boot error raised within the SDRAM timeout");
        $display("INFO [5] at %0d ms: SD init done=%0d, scan_kicked=%0d, CMD17=%0d, boot_state=%0d while SDRAM init held off",
                 $time / 1000000, `TB_SD_INIT, `TB_SCAN_KICK, u_sd.n_cmd17, dut.boot_state);
        release `TB_SDR_DONE;
        // ---------- [1] 正常启动 ----------
        boot_and_check("[1] power-on");

        // ---------- [2] 运行中复位 ----------
        press_reset(2_000_000);
        boot_and_check("[2] KEY1 during run");

        // ---------- [3] 连续复位 10 次（覆盖启动各阶段），第 6 次带按键抖动 ----------
        for (k = 0; k < 10; k = k + 1) begin
            chk_en = 1'b0;
            if (k == 5) begin
                repeat (6) begin key1_rst_n = 1'b0; #700; key1_rst_n = 1'b1; #1300; end
            end
            key1_rst_n = 1'b0;
            #(20_000 + k * 37_000);
            if (`TB_RSTN !== 4'b0000) err("[3] not in reset during a fast press");
            if (`TB_PLL_LOCK !== 1'b0) err("[3] PLL not reset during a fast press");
            key1_rst_n = 1'b1;
            #(highs[k]);
            $display("INFO [3] press %0d released for %0d us: rstn=%b sdram=%0d sd_init=%0d kick=%0d CMD17=%0d loaded_dv=%0d",
                     k + 1, highs[k] / 1000, `TB_RSTN, `TB_SDR_DONE, `TB_SD_INIT, `TB_SCAN_KICK, u_sd.n_cmd17, dut.display_valid);
        end
        chk_en = 1'b0;
        key1_rst_n = 1'b0; #50_000; key1_rst_n = 1'b1;
        boot_and_check("[3] after 10 fast resets");

        // ---------- [6] 失锁：运行中 30 ns 失锁毛刺；重启途中(扫描已开始)再失锁 5 us ----------
        chk_en = 1'b0;
        `TB_PLL.tb_unlock = 1'b1; #30; `TB_PLL.tb_unlock = 1'b0;
        #1;
        if (`TB_RSTN !== 4'b0000) err("[6] 30 ns loss of lock did not reset every domain");
        if (dut.display_valid !== 1'b0) err("[6] display still valid after loss of lock");
        if (`TB_PLAYER_EN !== 1'b0) err("[6] player_enable not cleared by loss of lock");
        wait (`TB_SCAN_KICK === 1'b1);
        #2_000_000;
        `TB_PLL.tb_unlock = 1'b1; #1;
        if (`TB_RSTN !== 4'b0000) err("[6] second loss of lock during restart did not reset every domain");
        #5_000; `TB_PLL.tb_unlock = 1'b0;
        boot_and_check("[6] after PLL unlocks");

        // ---------- 汇总 ----------
        if (m1_bad) err("M1: domain reset not asserted immediately on reset-source loss");
        if (m2_bad) err("M2: domain reset released while PLL unlocked");
        if (m3_bad) err("M3: scan started before SDRAM init");
        if (m4_bad) err("M4: CMD17 before scan start");
        if (m5_bad) err("M5: display before player_enable");
        $display("INFO SDRAM auto refresh: %0d commands, max gap %0.2f us, late %0d", dut.U3.sdram.ref_count, dut.U3.sdram.ref_max_gap_ns / 1000.0, dut.U3.sdram.ref_late);
        if (dut.U3.sdram.ref_late != 0 || dut.U3.sdram.ref_max_gap_ns > 50000.0) err("SDRAM refresh interval too long");
        if (clk_lane_bad != 0) begin errors = errors + 1; $display("FAIL clock lane: %0d chars != 0000011111", clk_lane_bad); end
        if (total_ecc_bad != 0) begin errors = errors + 1; $display("FAIL %0d data island packets with bad ECC", total_ecc_bad); end
        if (audio_short_frames != 0) begin errors = errors + 1; $display("FAIL %0d frames with audio packet count outside 195..205", audio_short_frames); end
        if (mute_bad_frames != 0) begin errors = errors + 1; $display("FAIL %0d frames before commit carry non-zero audio", mute_bad_frames); end
        if (silent_bad_frames != 0) begin errors = errors + 1; $display("FAIL %0d displayed frames carry only zero audio", silent_bad_frames); end
        if (av_mismatch != 0) begin errors = errors + 1; $display("FAIL active_audio_idx != active_image_idx for %0d pixel clocks", av_mismatch); end
        if (boots_ok != 4) begin errors = errors + 1; $display("FAIL only %0d of 4 boots displayed the image correctly", boots_ok); end
        $display("INFO frames: black=%0d img0=%0d img1=%0d mixed=%0d corrupted pixels=%0d muted=%0d audible=%0d chars=%0d",
                 frames_black, frames_img0, frames_img1, frames_mixed, total_other, muted_frames, audible_frames, n_chars);
        if (errors == 0) $display("PASS tb_top_reset_boot: 2.8 cases 1-6 + lesson-5 final boot sequencer (PLL-held reset, SDRAM-first boot order, player_enable gate, KEY1 during run, 10 fast resets incl. bounce, loss-of-lock reset), 4 boots pixel-exact");
        else             $display("FAIL tb_top_reset_boot: %0d errors", errors);
        $finish;
    end
    initial begin repeat (6) #1_500_000_000; $display("FAIL tb_top_reset_boot: timeout"); $finish; end
endmodule
