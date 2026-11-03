`timescale 1ns/1ps

module tb_media_policy;
    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    reg en = 1;
    reg next_e = 0, prev_e = 0, auto_e = 0, bri_e = 0, vol_e = 0, prd_e = 0, save_e = 0, tick = 0;
    reg [5:0] cur = 0;
    reg [6:0] cnt = 3;
    reg auto_fb = 0;
    reg ld_v = 0; reg [2:0] ld_sel = 0; reg [7:0] ld_bri = 0; reg [7:0] ld_con = 0;
    reg a_inc = 0, a_dec = 0, a_def = 0, a_sel = 0, a_show = 0; reg [7:0] ld_vol = 0; reg ld_auto = 0;
    wire saved_auto;
    wire goto_valid, toggle_auto, period_valid, linked;
    wire [5:0] goto_idx, media_id;
    wire [1:0] goto_aud, audio_id;
    // 外接传感器：policy 输出亮度偏移；这里按"传感器没接"建模（基准 255，显示 = 限幅(255 + 偏移)），
    // 原有亮度期望全部不改 → 证明无传感器时行为与原来逐位相同
    function [7:0] disp;
        input [8:0] off;
        reg signed [9:0] s;
        begin
            s = 10'sd255 + $signed({off[8], off});
            disp = s[9] ? 8'd0 : (s[8] ? 8'd255 : s[7:0]);
        end
    endfunction
    wire [8:0] bri_off, saved_off;
    wire [7:0] bright = disp(bri_off);
    wire [7:0] contrast;
    wire [31:0] period_cycles;
    wire [8:0] tenths;
    wire [7:0] volume, saved_vol;
    wire pic_ev, vol_ev, bri_ev, con_ev, prd_ev, save_ev, saved_valid;
    wire [2:0] saved_sel;
    wire [7:0] saved_con;
    wire [7:0] saved_bri = disp(saved_off);
    integer errors = 0;

    media_policy #(.CLK_HZ(1000), .DEFAULT_PERIOD_SEL(3'd2),
                   .DEFAULT_VOLUME(8'd128), .VOL_STEP(8'd32), .BC_STEP(8'd16)) dut (
        .clk(clk), .rst_n(rst_n), .enable(en),
        .next_event(next_e), .prev_event(prev_e), .auto_event(auto_e), .bright_event(bri_e),
        .vol_event(vol_e), .period_event(prd_e), .save_event(save_e), .auto_tick(tick),
        .adj_inc(a_inc), .adj_dec(a_dec), .adj_default(a_def), .adj_sel(a_sel), .adj_show(a_show), .ir_present(1'b0), .disp_brightness(bright), .bri_base(8'd255), .env_change(1'b0),
        .current_image_idx(cur), .image_count(cnt), .auto_enabled(auto_fb),
        .cfg_load_valid(ld_v), .cfg_period_sel(ld_sel), .cfg_brightness({1'b1, ld_bri} + 9'd1), .cfg_contrast(ld_con), .cfg_volume(ld_vol), .cfg_auto_mode(ld_auto),
        .goto_valid(goto_valid), .goto_image_idx(goto_idx), .goto_audio_idx(goto_aud),
        .toggle_auto_event(toggle_auto),
        .period_cfg_valid(period_valid), .period_cfg_cycles(period_cycles),
        .media_id(media_id), .audio_id(audio_id), .linked_event(linked),
        .bri_offset(bri_off), .contrast(contrast), .volume(volume), .period_tenths(tenths),
        .pic_event(pic_ev), .vol_page_event(vol_ev), .prd_event(prd_ev), .bri_event(bri_ev), .con_event(con_ev),
        .save_done_event(save_ev),
        .saved_period_sel(saved_sel), .saved_brightness(saved_off), .saved_contrast(saved_con), .saved_volume(saved_vol), .saved_auto_mode(saved_auto),
        .saved_valid(saved_valid)
    );

    // goto 的每一拍都必须带着联动音轨，且 linked_event 与 goto_valid 同拍
    always @(posedge clk) if (rst_n) begin
        if (goto_valid && goto_aud !== goto_idx[1:0]) begin
            $display("FAIL link: img=%0d aud=%0d", goto_idx, goto_aud); errors = errors + 1;
        end
        if (goto_valid !== linked) begin
            $display("FAIL linked_event not aligned with goto_valid @%0t", $time); errors = errors + 1;
        end
    end

    task check_goto(input [5:0] exp, input [127:0] tag);
        begin
            if (!goto_valid || goto_idx !== exp || goto_aud !== exp[1:0] || !pic_ev ||
                media_id !== exp || audio_id !== exp[1:0]) begin
                $display("FAIL %0s: valid=%b idx=%0d aud=%0d media=%0d audio=%0d exp=%0d",
                         tag, goto_valid, goto_idx, goto_aud, media_id, audio_id, exp);
                errors = errors + 1;
            end
        end
    endtask
    task pulse_next(input [5:0] c, input [6:0] n, input [5:0] exp, input [127:0] tag);
        begin
            cur = c; cnt = n;
            @(posedge clk) next_e <= 1; @(posedge clk) next_e <= 0; #1;
            check_goto(exp, tag);
        end
    endtask
    task pulse_prev(input [5:0] c, input [6:0] n, input [5:0] exp, input [127:0] tag);
        begin
            cur = c; cnt = n;
            @(posedge clk) prev_e <= 1; @(posedge clk) prev_e <= 0; #1;
            check_goto(exp, tag);
        end
    endtask
    task pulse_vol(input [7:0] exp);
        begin
            @(posedge clk) vol_e <= 1; @(posedge clk) vol_e <= 0; #1;
            if (volume !== exp || !vol_ev) begin
                $display("FAIL volume step got %0d exp %0d", volume, exp); errors = errors + 1;
            end
        end
    endtask

    integer n_toggle = 0, n_pic_toggle = 0;
    task adj(input inc, input dec, input def, input sel);
        begin
            @(posedge clk) begin a_inc <= inc; a_dec <= dec; a_def <= def; a_sel <= sel; end
            @(posedge clk) begin a_inc <= 0; a_dec <= 0; a_def <= 0; end
        end
    endtask
    reg ev_b_seen, ev_c_seen;
    always @(posedge clk) begin if (bri_ev) ev_b_seen <= 1; if (con_ev) ev_c_seen <= 1; end
    task expect_bc(input [7:0] b, input [7:0] c, input eb, input ec, input [8*32-1:0] tag);
        begin
            #1;
            if (bright !== b || contrast !== c || bri_ev !== eb || con_ev !== ec) begin
                $display("FAIL %0s bri=%0d con=%0d bri_ev=%b con_ev=%b", tag, bright, contrast, bri_ev, con_ev); errors = errors + 1;
            end
        end
    endtask
    always @(posedge clk) if (toggle_auto) begin n_toggle = n_toggle + 1; if (pic_ev) n_pic_toggle = n_pic_toggle + 1; end
    integer i;
    reg [8:0] exp_tenths [0:6];
    initial begin
        exp_tenths[0] = 50; exp_tenths[1] = 100; exp_tenths[2] = 300; exp_tenths[3] = 5;
        exp_tenths[4] = 10; exp_tenths[5] = 25; exp_tenths[6] = 50;
        #30 rst_n = 1; #1;
        if (tenths !== 9'd25 || bright !== 8'd255 || contrast !== 8'd128 || period_cycles !== 32'd2500 || volume !== 8'd128) begin
            $display("FAIL reset defaults tenths=%0d bright=%0d cyc=%0d vol=%0d",
                     tenths, bright, period_cycles, volume); errors = errors + 1;
        end

        // ---- enable=0：首图出来之前所有事件都被丢弃 ----
        en = 0;
        @(posedge clk) begin next_e <= 1; prev_e <= 1; auto_e <= 1; bri_e <= 1;
                             vol_e <= 1; prd_e <= 1; save_e <= 1; tick <= 1; end
        @(posedge clk) begin next_e <= 0; prev_e <= 0; auto_e <= 0; bri_e <= 0;
                             vol_e <= 0; prd_e <= 0; save_e <= 0; tick <= 0; end
        #1;
        if (goto_valid || toggle_auto || period_valid || linked || pic_ev || vol_ev || bri_ev ||
            prd_ev || save_ev || saved_valid || volume !== 128 || bright !== 255 || tenths !== 25) begin
            $display("FAIL enable=0 must drop all events"); errors = errors + 1;
        end
        en = 1;

        pulse_next(0, 3, 1, "next 0/3");
        pulse_next(2, 3, 0, "next wrap 2/3");
        pulse_next(3, 4, 0, "next wrap 3/4");
        pulse_next(0, 1, 0, "next single");
        pulse_next(1, 2, 0, "next wrap 1/2");
        pulse_prev(0, 3, 2, "prev wrap 0/3");
        pulse_prev(0, 4, 3, "prev wrap 0/4");
        pulse_prev(2, 4, 1, "prev 2/4");
        pulse_prev(0, 1, 0, "prev single");
        pulse_next(62, 64, 63, "next 62/64");
        pulse_next(63, 64, 0, "next wrap 63/64");
        pulse_prev(0, 64, 63, "prev wrap 0/64");
        // auto_tick 走 next 策略（同样联动音轨）
        cur = 1; cnt = 4;
        @(posedge clk) tick <= 1; @(posedge clk) tick <= 0; #1;
        check_goto(2, "auto_tick");
        // next 与 prev 同拍：next 优先
        cur = 1; cnt = 4;
        @(posedge clk) begin next_e <= 1; prev_e <= 1; end
        @(posedge clk) begin next_e <= 0; prev_e <= 0; end #1;
        check_goto(2, "next/prev priority");
        // 没有事件时 goto 不出现
        @(posedge clk); #1;
        if (goto_valid || linked) begin $display("FAIL goto not single-cycle"); errors = errors + 1; end
        // auto toggle
        @(posedge clk) auto_e <= 1; @(posedge clk) auto_e <= 0; #1;
        if (!toggle_auto || !pic_ev) begin $display("FAIL toggle auto"); errors = errors + 1; end
        // 亮度 255→191→127→63→255（(b+1)/256 = 75/50/25/100 %）
        for (i = 0; i < 4; i = i + 1) begin
            @(posedge clk) bri_e <= 1; @(posedge clk) bri_e <= 0; #1;
            if (bright !== ((i == 3) ? 8'd255 : 8'd191 - 8'd64 * i) || !bri_ev) begin
                $display("FAIL bright step %0d -> %0d", i, bright); errors = errors + 1;
            end
        end
        // 音量 128→160→192→224→255→0→32
        pulse_vol(160); pulse_vol(192); pulse_vol(224); pulse_vol(255); pulse_vol(0); pulse_vol(32);
        // 周期阶梯 2.5→5→10→30→0.5→1→2.5→5
        for (i = 0; i < 7; i = i + 1) begin
            @(posedge clk) prd_e <= 1; @(posedge clk) prd_e <= 0; #1;
            if (tenths !== exp_tenths[i] || !period_valid || period_cycles !== exp_tenths[i] * 100 || !prd_ev) begin
                $display("FAIL period step %0d tenths=%0d cyc=%0d", i, tenths, period_cycles); errors = errors + 1;
            end
        end
        // 保存快照（当前 sel=3 即 5s，亮度 191，对比度 128，音量 32）
        @(posedge clk) bri_e <= 1; @(posedge clk) bri_e <= 0;
        @(posedge clk) save_e <= 1; @(posedge clk) save_e <= 0; #1;
        if (!save_ev || !saved_valid || saved_sel !== 3'd3 || saved_bri !== 8'd191 || saved_con !== 8'd128 || saved_vol !== 8'd32) begin
            $display("FAIL save sel=%0d bri=%0d vol=%0d", saved_sel, saved_bri, saved_vol); errors = errors + 1;
        end
        // 多个事件同拍：各自生效（切图 + 调音量）
        cur = 0; cnt = 4;
        @(posedge clk) begin next_e <= 1; vol_e <= 1; end
        @(posedge clk) begin next_e <= 0; vol_e <= 0; end #1;
        check_goto(1, "next + vol same cycle");
        if (volume !== 8'd64) begin $display("FAIL vol in same cycle %0d", volume); errors = errors + 1; end

        // enable=0 时加载：周期/亮度/音量立即生效，period_cfg 下发；自动模式不恢复
        en = 0; cnt = 1; auto_fb = 0;
        @(posedge clk) begin ld_v <= 1; ld_sel <= 3'd4; ld_bri <= 8'd77; ld_con <= 8'd200; ld_vol <= 8'd200; ld_auto <= 1'b1; end
        @(posedge clk) ld_v <= 0; #1;
        if (tenths !== 9'd100 || !period_valid || period_cycles !== 32'd10000 || bright !== 8'd77 || contrast !== 8'd200 || volume !== 8'd200 ||
            saved_sel !== 3'd4 || saved_bri !== 8'd77 || saved_con !== 8'd200 || saved_vol !== 8'd200 || !saved_auto || !saved_valid || toggle_auto ||
            vol_ev || prd_ev || bri_ev || con_ev || pic_ev || save_ev) begin
            $display("FAIL L1 load tenths=%0d cyc=%0d bri=%0d vol=%0d toggle=%b", tenths, period_cycles, bright, volume, toggle_auto); errors = errors + 1;
        end
        n_toggle = 0; n_pic_toggle = 0;
        repeat (5) @(posedge clk);
        // L2 enable=1 但只有 1 张图：不能恢复（控制层会报 0x53）；此时保存仍记录“自动”
        en = 1;
        repeat (5) @(posedge clk);
        @(posedge clk) save_e <= 1; @(posedge clk) save_e <= 0; #1;
        if (n_toggle != 0 || !saved_auto || saved_vol !== 8'd200) begin
            $display("FAIL L2 no restore with 1 image / pending auto saved (toggles=%0d saved_auto=%b)", n_toggle, saved_auto); errors = errors + 1;
        end
        // L3 图片数 > 1：恢复一次，不弹 PIC 页
        cnt = 3;
        repeat (3) @(posedge clk); #1;
        if (n_toggle != 1 || n_pic_toggle != 0) begin
            $display("FAIL L3 auto restore toggles=%0d (pic with toggle %0d)", n_toggle, n_pic_toggle); errors = errors + 1;
        end
        auto_fb = 1;
        repeat (5) @(posedge clk);
        if (n_toggle != 1) begin $display("FAIL L3 restore must fire only once"); errors = errors + 1; end
        // L4 自动开着保存 → saved_auto=1；关掉后保存 → 0
        @(posedge clk) save_e <= 1; @(posedge clk) save_e <= 0; #1;
        if (!saved_auto) begin $display("FAIL L4 saved_auto with auto on"); errors = errors + 1; end
        auto_fb = 0;
        @(posedge clk) save_e <= 1; @(posedge clk) save_e <= 0; #1;
        if (saved_auto) begin $display("FAIL L4 saved_auto with auto off"); errors = errors + 1; end
        // L5 恢复前用户先按 KEY3：只有用户那一次，之后不再恢复
        cnt = 1; n_toggle = 0;
        @(posedge clk) begin ld_v <= 1; ld_auto <= 1'b1; end
        @(posedge clk) ld_v <= 0;
        @(posedge clk) auto_e <= 1; @(posedge clk) auto_e <= 0;
        cnt = 3;
        repeat (5) @(posedge clk);
        if (n_toggle != 1) begin $display("FAIL L5 user KEY3 must cancel restore (toggles=%0d)", n_toggle); errors = errors + 1; end
        // L6 已经是自动：不再翻转
        n_toggle = 0; auto_fb = 1;
        @(posedge clk) begin ld_v <= 1; ld_auto <= 1'b1; end
        @(posedge clk) ld_v <= 0;
        repeat (5) @(posedge clk);
        if (n_toggle != 0) begin $display("FAIL L6 restore while already auto"); errors = errors + 1; end
        // L7 加载 auto=0：不翻转
        auto_fb = 0;
        @(posedge clk) begin ld_v <= 1; ld_auto <= 1'b0; end
        @(posedge clk) ld_v <= 0;
        repeat (5) @(posedge clk);
        if (n_toggle != 0) begin $display("FAIL L7 manual mode must not toggle"); errors = errors + 1; end

        // A1 亮度：从加载值 77 起 +16 → 93；-16 ×6 → 0 饱和；+16 ×17 → 255 饱和
        adj(1, 0, 0, 0); expect_bc(8'd93, 8'd200, 1, 0, "A1 bri +16");
        for (i = 0; i < 6; i = i + 1) adj(0, 1, 0, 0);
        expect_bc(8'd0, 8'd200, 1, 0, "A1 bri saturate 0");
        for (i = 0; i < 17; i = i + 1) adj(1, 0, 0, 0);
        expect_bc(8'd255, 8'd200, 1, 0, "A1 bri saturate 255");
        // A2 对比度：200 +16 ×4 → 255 饱和；默认 → 128；-16 ×8 → 0；再 -16 仍 0
        for (i = 0; i < 4; i = i + 1) adj(1, 0, 0, 1);
        expect_bc(8'd255, 8'd255, 0, 1, "A2 con saturate 255");
        adj(0, 0, 1, 1); expect_bc(8'd255, 8'd128, 0, 1, "A2 con default 128");
        for (i = 0; i < 9; i = i + 1) adj(0, 1, 0, 1);
        expect_bc(8'd255, 8'd0, 0, 1, "A2 con saturate 0");
        // A3 亮度恢复默认（先调暗）
        adj(0, 1, 0, 0); adj(0, 1, 0, 0); expect_bc(8'd223, 8'd0, 1, 0, "A3 bri -32");
        adj(0, 0, 1, 0); expect_bc(8'd255, 8'd0, 1, 0, "A3 bri default 255");
        // A4 adj_show：只弹页，不改值
        @(posedge clk) begin a_show <= 1; a_sel <= 1; end @(posedge clk) a_show <= 0; #1;
        if (!con_ev || bri_ev || bright !== 8'd255 || contrast !== 8'd0) begin $display("FAIL A4 adj_show"); errors = errors + 1; end
        // A5 细调值经阶梯：200 → 191 → 127
        adj(0, 1, 0, 0); adj(0, 1, 0, 0); adj(0, 1, 0, 0);       // 255-48 = 207
        @(posedge clk) bri_e <= 1; @(posedge clk) bri_e <= 0; #1;
        if (bright !== 8'd191) begin $display("FAIL A5 ladder from 207 -> %0d", bright); errors = errors + 1; end
        // A6 enable=0：调节事件丢弃；快照含对比度
        en = 0; adj(1, 0, 0, 1); @(negedge clk) en = 1;
        if (contrast !== 8'd0 || con_ev) begin $display("FAIL A6 enable=0 must drop adj"); errors = errors + 1; end
        adj(1, 0, 0, 1);
        @(posedge clk) save_e <= 1; @(posedge clk) save_e <= 0; #1;
        if (saved_bri !== 8'd191 || saved_con !== 8'd16) begin $display("FAIL A6 snapshot bri=%0d con=%0d", saved_bri, saved_con); errors = errors + 1; end

        if (errors == 0) $display("PASS tb_media_policy (incl. 2.11 Flash config load + auto restore L1-L7, lesson13 8-bit brightness/contrast A1-A6)");
        else             $display("FAIL tb_media_policy errors=%0d", errors);
        $finish;
    end
endmodule
