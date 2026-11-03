`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// tb_top_ex4_hdmi_output：lab_ex4_tf 整板顶层(top)端到端验证
// DUT 为完整 top：复位、按键/策略/播放控制、SD 卡 SPI、四 bank loader/reader、
//   SDRAM(安路 SIP 模型)、video_timing_ctrl、OSD、video_x_tagger、media_hdmi_audio_bridge、
//   TMDS 时钟通道、10:1 串化 PHY(EG_LOGIC_ODDR)、DDC 核。PLL 用 models/pll_behav_models.v。
// 外部：models/sd_card_fat32_model.v（FAT32 卡上 2 张 640x480 BMP），KEY2 单击 = 下一张。
// 监视：直接对 HDMI_D0/D1/D2/CLK_P 串行引脚解串(5x 时钟双沿采样)，以时钟通道上升沿
//   定字符边界，按 LSB first 还原 10bit 字符，再做以下检查：
//   [P0] 时钟通道每个字符都是 0000011111
//   [P1] 首图提交前每个有效像素都是黑色(OSD 区域除外)
//   [P2] 提交后每一帧每个像素 == BMP 期望像素；OSD 区域(x 8..63, y 8..15)允许 OSD 黄/红字
//        —— 任一像素不等于 黑/图0/图1(/OSD) 即判"花屏"
//   [P3] 每帧 480 有效行 x 640 像素，前面 2 字符视频保护带
//   [P4] 数据岛包头/子包 ECC 全部正确；每帧 Audio Sample 包 195..205 个、有 ACR 与 InfoFrame
//   [P5] 首图提交前音频采样全 0(静音)，提交后的完整帧音频非 0
//   [P6] 切图：active_audio_idx 与 active_image_idx 同步切换，提示音 blip 触发，音频包不断流
// -----------------------------------------------------------------------------
module tb_top_ex4_hdmi_output;
    `include "tb_hdmi_common.vh"

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

    sd_card_fat32_model #(.W(640), .H(480)) u_sd (
        .nCS(sd_ncs), .DCLK(sd_dclk), .MOSI(sd_mosi), .MISO(sd_miso)
    );

    // ---- 仅仿真加速/模型需要，RTL 不变 ----
    //  * 只保留 PIC1/PIC2；使用 64 KiB 簇减少 FAT 链查询次数，BMP 数据量和像素检查不变。
    //  * PLL 用 models/pll_behav_models.v
    initial begin
        #1;
        u_sd.fi_present       = 4'b0011;
        u_sd.fi_spc           = 8'd128;
        u_sd.fi_data_clusters = 32'd90000;
    end
    // 引脚级解串 + 字符/像素/数据岛检查(与 tb_top_reset_boot 共用)
    `define TB_SER_CLK dut.pixel_clk_5x         
    `define TB_PIX_CLK dut.pixel_clk             // 单 PLL clk3 25MHz
    `include "tb_top_hdmi_checker.vh"

    integer precommit_reads = 0;
    integer normal_reader_faults = 0;
    always @(posedge dut.mem_clk) begin
        if (dut.App_rd_en && !dut.reader_enable_mem)
            precommit_reads = precommit_reads + 1;
        if (dut.display_valid_sys && dut.reader_reader_error != 8'h00)
            normal_reader_faults = normal_reader_faults + 1;
    end
    always @(posedge dut.pixel_clk)
        if (dut.display_valid && dut.reader_underflow)
            normal_reader_faults = normal_reader_faults + 1;

    initial forever begin
        #10_000_000;
        $display("INFO t=%0d ms bmp_state=%0d CMD17=%0d scan_done=%0d images=%0d display_valid=%0d buf=%0d chars=%0d",
                 $time / 1000000, dut.sd_card_bmp_m0.bmp_state_code, u_sd.n_cmd17, dut.scan_done, dut.image_count,
                 dut.display_valid_sys, dut.active_buffer_idx, n_chars);
    end

    integer f_mark, blip_before;
    initial begin
        #1000 key1_rst_n = 1'b1;
        // 1) 等首图提交(PLL 锁定 + SDRAM 初始化 + SD 初始化 + 扫描 + 读 1801 扇区 + SDRAM 写完)
        wait (dut.display_valid === 1'b1);
        $display("INFO first image committed at %0d ms (img=%0d trk=%0d)", $time / 1000000, dut.active_image_idx, dut.active_audio_idx);
        // FAT32 合同：目录扫描一次生成完整动态清单，scan_all_done == scan_done。
        if (dut.scan_all_done !== 1'b1 || dut.image_count < 3'd2) begin
            errors = errors + 1;
            $display("FAIL FAT32 catalog was not complete before first-image load");
        end
        $display("INFO FAT32 catalog ready with %0d images", dut.image_count);
        f_mark = frames;
        wait (frames >= f_mark + 3);
        // 2) KEY2 单击(下一张)：按下 60 ms > 20 ms 消抖，远小于 1 s 长按
        blip_before = blip_seen;
        key2_n = 1'b0; #60_000_000; key2_n = 1'b1;
        wait (dut.active_image_idx === 2'd1);
        $display("INFO image 1 committed at %0d ms (img=%0d trk=%0d buf=%0d)", $time / 1000000,
                 dut.active_image_idx, dut.active_audio_idx, dut.active_buffer_idx);
        f_mark = frames;
        wait (frames >= f_mark + 3);
        if (blip_seen <= blip_before) err("no audio cue blip after the image switch");

        if (clk_lane_bad != 0) begin errors = errors + 1; $display("FAIL clock lane: %0d chars != 0000011111", clk_lane_bad); end
        if (total_ecc_bad != 0) begin errors = errors + 1; $display("FAIL %0d data island packets with bad ECC", total_ecc_bad); end
        if (audio_short_frames != 0) begin errors = errors + 1; $display("FAIL %0d frames with audio packet count outside 195..205", audio_short_frames); end
        if (frames_img0 < 2 || frames_img1 < 2) begin errors = errors + 1; $display("FAIL not enough pure image frames"); end
        if (frames_mixed > 2) begin errors = errors + 1; $display("FAIL %0d mixed frames (only commit/switch frames may mix)", frames_mixed); end
        if (mute_bad_frames != 0) begin errors = errors + 1; $display("FAIL %0d frames before first commit carry non-zero audio", mute_bad_frames); end
        if (silent_bad_frames != 0) begin errors = errors + 1; $display("FAIL %0d displayed frames carry only zero audio", silent_bad_frames); end
        if (muted_frames < 2 || audible_frames < 4) begin errors = errors + 1; $display("FAIL too few muted (%0d) / audible (%0d) frames observed", muted_frames, audible_frames); end
        if (av_mismatch != 0) begin errors = errors + 1; $display("FAIL active_audio_idx != active_image_idx for %0d pixel clocks", av_mismatch); end
        if (precommit_reads != 0) begin errors = errors + 1; $display("FAIL %0d SDRAM reads occurred before first image commit", precommit_reads); end
        if (normal_reader_faults != 0) begin errors = errors + 1; $display("FAIL reader fault/underflow seen during normal display (%0d cycles)", normal_reader_faults); end
        // SDRAM 40MHz、15.625us/次；最大间隔(含偶发丢 <= 3 次)必须 <= 50us，否则 2048 行 64ms 不满足
        $display("INFO SDRAM auto refresh: %0d commands, max gap %0.2f us, late %0d", dut.U3.sdram.ref_count, dut.U3.sdram.ref_max_gap_ns / 1000.0, dut.U3.sdram.ref_late);
        if (dut.U3.sdram.ref_late != 0 || dut.U3.sdram.ref_max_gap_ns > 50000.0) begin errors = errors + 1; $display("FAIL SDRAM refresh interval too long"); end
        $display("INFO frames: black=%0d img0=%0d img1=%0d mixed=%0d, corrupted pixels=%0d, audio packets=%0d, muted=%0d audible=%0d, chars=%0d",
                 frames_black, frames_img0, frames_img1, frames_mixed, total_other, total_audio, muted_frames, audible_frames, n_chars);
        if (errors == 0) $display("PASS tb_top_ex4_hdmi_output: pin-level TMDS decode, BMP pixel-exact (OSD masked) on every frame, black+mute before commit, audio ECC ok, AV-linked switch + blip");
        else             $display("FAIL tb_top_ex4_hdmi_output: %0d errors", errors);
        $finish;
    end
    initial begin repeat (4) #1_500_000_000; $display("FAIL tb_top_ex4_hdmi_output: timeout (bmp_state=%0d)", dut.sd_card_bmp_m0.bmp_state_code); $finish; end
endmodule
