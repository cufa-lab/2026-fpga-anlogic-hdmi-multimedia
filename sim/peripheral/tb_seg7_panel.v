`timescale 1ns/1ps
// seg7_panel（8 位）：扫描解码回 8 位显示内容，逐页比对
//   * 页面轮播 PIC→VOL→PRD→PIC（讲义 2.8）
//   * 事件强制页 + 保持时间：vol_event→VOL、prd_event→PRD、pic_event→PIC；
//     BRI/SAVE/ERR 事件页结束后回 PIC；保持期间不翻页
//   * 段码/位选低有效、位选 one-hot、dig_n[k] = 第 k 位（0 最右）
//   * DIGIT_REVERSE=1 的实例：位选镜像、段码相同
//   * enable=0：全灭
module tb_seg7_panel;
    localparam STEP = 4, PAGE = 1000, HOLD = 3000;
    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    reg       en = 1;
    reg [5:0] media_id = 0;
    reg       media_valid = 0, auto_en = 0;
    reg [7:0] bright = 255;      
    reg [7:0] contrast = 128;
    reg [7:0] volume = 128;
    reg [8:0] tenths = 25;
    reg [7:0] err = 0;
    reg pic_e = 0, vol_e = 0, bri_e = 0, con_e = 0, prd_e = 0, save_e = 0, err_e = 0;
    wire [7:0] seg_n, seg_n_r;
    wire [7:0] dig_n, dig_n_r;
    integer errors = 0;

    seg7_panel #(.CLK_HZ(1000), .SCAN_STEP_CYCLES(STEP), .PAGE_PERIOD_CYCLES(PAGE),
                 .HOLD_CYCLES(HOLD), .DIGIT_REVERSE(0)) dut (
        .clk(clk), .rst_n(rst_n), .tick(1'b1), .enable(en),
        .media_id(media_id), .media_valid(media_valid), .auto_enabled(auto_en),
        .brightness(bright), .contrast(contrast), .volume(volume), .period_tenths(tenths), .error_code(err),
        .pic_event(pic_e), .vol_event(vol_e), .bri_event(bri_e), .con_event(con_e), .prd_event(prd_e),
        .save_event(save_e), .err_event(err_e),
        .seg_n(seg_n), .dig_n(dig_n)
    );
    seg7_panel #(.CLK_HZ(1000), .SCAN_STEP_CYCLES(STEP), .PAGE_PERIOD_CYCLES(PAGE),
                 .HOLD_CYCLES(HOLD), .DIGIT_REVERSE(1)) dut_rev (
        .clk(clk), .rst_n(rst_n), .tick(1'b1), .enable(en),
        .media_id(media_id), .media_valid(media_valid), .auto_enabled(auto_en),
        .brightness(bright), .contrast(contrast), .volume(volume), .period_tenths(tenths), .error_code(err),
        .pic_event(pic_e), .vol_event(vol_e), .bri_event(bri_e), .con_event(con_e), .prd_event(prd_e),
        .save_event(save_e), .err_event(err_e),
        .seg_n(seg_n_r), .dig_n(dig_n_r)
    );

    // 解码：disp[7]=最左 … disp[0]=最右（高有效段码 {dp,g..a}）
    reg [7:0] disp [0:7];
    integer j, ones;
    reg [7:0] mirrored;
    always @(posedge clk) if (rst_n) begin
        ones = 0;
        for (j = 0; j < 8; j = j + 1) if (!dig_n[j]) ones = ones + 1;
        if (ones > 1) begin $display("FAIL: dig_n not one-hot %b", dig_n); errors = errors + 1; end
        for (j = 0; j < 8; j = j + 1) if (!dig_n[j]) disp[j] = ~seg_n;
        for (j = 0; j < 8; j = j + 1) mirrored[j] = dig_n[7 - j];
        if (dig_n_r !== mirrored || seg_n_r !== seg_n) begin
            $display("FAIL: DIGIT_REVERSE mismatch dig=%b rev=%b", dig_n, dig_n_r); errors = errors + 1;
        end
    end

    task settle; repeat (STEP * 8 * 2 + 4) @(posedge clk); endtask
    task expect8(input [63:0] exp, input [127:0] tag);
        reg [63:0] got;
        begin
            got = {disp[7], disp[6], disp[5], disp[4], disp[3], disp[2], disp[1], disp[0]};
            if (got !== exp) begin
                $display("FAIL %0s: got %h exp %h", tag, got, exp); errors = errors + 1;
            end
        end
    endtask
    task ev(input integer which);
        begin
            @(posedge clk) case (which)
                0: pic_e <= 1; 1: bri_e <= 1; 2: prd_e <= 1; 3: save_e <= 1; 4: err_e <= 1; 6: con_e <= 1;
                default: vol_e <= 1;
            endcase
            @(posedge clk) begin pic_e <= 0; bri_e <= 0; con_e <= 0; prd_e <= 0; save_e <= 0; err_e <= 0; vol_e <= 0; end
        end
    endtask

    initial begin
        #30 rst_n = 1;
        settle;
        expect8(64'h00_00_00_73_06_39_00_40, "PIC no image");     // _ _ _ P I C _ -
        media_valid = 1; media_id = 2; auto_en = 1;
        settle;
        expect8(64'h00_00_00_73_06_39_00_cf, "PIC 3 auto");
        auto_en = 0; settle;
        expect8(64'h00_00_00_73_06_39_00_4f, "PIC 3");
        // 自动翻页 → VOL 128
        repeat (PAGE) @(posedge clk); settle;
        expect8(64'h3e_3f_38_00_00_06_5b_7f, "VOL 128");          // V O L _ _ 1 2 8
        // → PRD 2.5s
        repeat (PAGE) @(posedge clk); settle;
        expect8(64'h73_50_5e_00_00_00_db_6d, "PRD 2.5");          // P r d _ _ _ 2. 5（不带 S，避免读成 2.55）
        // → PIC
        repeat (PAGE) @(posedge clk); settle;
        expect8(64'h00_00_00_73_06_39_00_4f, "PIC wrap");
        // 音量事件 → 强制 VOL，保持期间不翻页
        volume = 0; ev(5); settle;
        expect8(64'h3e_3f_38_00_00_3f_3f_3f, "VOL 000");
        repeat (PAGE + 100) @(posedge clk);
        expect8(64'h3e_3f_38_00_00_3f_3f_3f, "VOL held");
        volume = 255; settle;
        expect8(64'h3e_3f_38_00_00_5b_6d_6d, "VOL 255");
        // VOL 保持结束后继续轮播（VOL 是轮播页，不回 PIC）
        repeat (HOLD - PAGE - 100) @(posedge clk); settle;    // 事件后约 HOLD+200 拍
        expect8(64'h3e_3f_38_00_00_5b_6d_6d, "VOL after hold");
        repeat (PAGE) @(posedge clk); settle;
        expect8(64'h73_50_5e_00_00_00_db_6d, "rotate VOL->PRD");
        // ERR 事件页：保持 HOLD，期间不翻页，之后回 PIC
        err = 8'h52; ev(4); settle;
        expect8(64'h79_50_50_00_00_00_6d_5b, "ERR 52");           // E r r _ _ _ 5 2
        repeat (PAGE + 100) @(posedge clk);
        expect8(64'h79_50_50_00_00_00_6d_5b, "ERR held");
        repeat (HOLD - PAGE) @(posedge clk); settle;
        expect8(64'h00_00_00_73_06_39_00_4f, "back to PIC");
        // SAVE 事件页
        ev(3); settle;
        expect8(64'h6d_77_3e_79_00_00_00_00, "SAVE");             // S A U E _ _ _ _
        // 周期事件 30.0s → 强制 PRD
        tenths = 300; ev(2); settle;
        expect8(64'h73_50_5e_00_00_4f_bf_3f, "PRD 30.0");         // P r d _ _ 3 0. 0
        tenths = 5; settle;
        expect8(64'h73_50_5e_00_00_00_bf_6d, "PRD 0.5");          // P r d _ _ _ 0. 5
        // 亮度事件
        bright = 255; ev(1); settle;
        expect8(64'h7c_50_00_00_00_5b_6d_6d, "BRI 255");          // b r _ _ _ 2 5 5
        bright = 100; settle;
        expect8(64'h7c_50_00_00_00_06_3f_3f, "BRI 100");
        bright = 75; settle;
        expect8(64'h7c_50_00_00_00_00_07_6d, "BRI 75");
        bright = 5; settle;
        expect8(64'h7c_50_00_00_00_00_00_6d, "BRI 5");
        bright = 191;
        // 对比度事件页
        contrast = 128; ev(6); settle;
        expect8(64'h39_5c_54_00_00_06_5b_7f, "CON 128");          // C o n _ _ 1 2 8
        contrast = 0; settle;
        expect8(64'h39_5c_54_00_00_00_00_3f, "CON 0");
        contrast = 255; settle;
        expect8(64'h39_5c_54_00_00_5b_6d_6d, "CON 255");
        ev(1); settle;
        expect8(64'h7c_50_00_00_00_06_6f_06, "BRI 191 after CON");
        // 同拍多个事件：ERR 优先
        @(posedge clk) begin pic_e <= 1; vol_e <= 1; err_e <= 1; end
        @(posedge clk) begin pic_e <= 0; vol_e <= 0; err_e <= 0; end
        settle;
        expect8(64'h79_50_50_00_00_00_6d_5b, "ERR priority");
        // 切图事件 → PIC，图片 4
        media_id = 3; ev(0); settle;
        expect8(64'h00_00_00_73_06_39_00_66, "PIC 4");
        media_id = 63; settle;
        expect8(64'h00_00_00_73_06_39_7d_66, "PIC 64");
        // enable=0 → 全灭
        en = 0; repeat (4) @(posedge clk); #1;
        if (dig_n !== 8'hff || seg_n !== 8'hff) begin
            $display("FAIL enable=0 dig=%b seg=%b", dig_n, seg_n); errors = errors + 1;
        end
        if (errors == 0) $display("PASS tb_seg7_panel");
        else             $display("FAIL tb_seg7_panel errors=%0d", errors);
        $finish;
    end
endmodule
