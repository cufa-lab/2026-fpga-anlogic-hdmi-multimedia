`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// tb_sensor_policy —— ir_presence_filter + media_policy（DEFAULT_AUTO=1、默认 5 s）+ 控制层自动开关模型
//   A 上电默认：周期 5 s（tenths 50、period_cfg_cycles = 5×CLK），首图放行且图片 > 1 后自动开轮播
//   B 遮挡 < 50ms 的抖动不动；遮挡稳定 → 关自动（手动）；移开稳定 → 开自动
//   C 遮挡期间按 KEY3 仍可开自动；移开时已是自动 → 不再翻转
//   D 无遮挡时 KEY3 关自动 → 保持关（红外不变化就不干预）
//   E 保存快照：自动状态 + 周期 5 s 进快照
//   F Flash 记录"手动"：上电不开自动；之后遮挡 → 移开 → 开自动
//   G 上电时手已挡着：始终不开自动（无 开→关 闪烁，toggle 次数 0）
//   H 只有 1 张图：移开请求挂起，图片数 > 1 时再开
// 控制层模型：toggle_auto_event 且图片数 > 1 时下一拍翻转 auto_enabled（与 media_player_control 一致）。
// tick_ms 每 8 拍一次加速。
// -----------------------------------------------------------------------------
module tb_sensor_policy;
    localparam integer CLK_HZ = 1000;      // 小时钟数便于检查 period_cfg_cycles
    reg clk = 0, rst_n = 0;
    always #10 clk = ~clk;
    reg tick = 0;
    integer tdiv = 0;
    always @(posedge clk) begin
        tdiv <= (tdiv == 7) ? 0 : tdiv + 1;
        tick <= (tdiv == 7);
    end

    reg        ir_n = 1'b1;
    wire       ir_present;
    ir_presence_filter #(.STABLE_MS(50)) u_ir (.clk(clk), .rst_n(rst_n), .tick_ms(tick), .ir_out_n(ir_n), .present(ir_present));

    reg        en = 0, auto_key = 0, save_e = 0;
    reg [6:0]  cnt = 7'd4;
    reg        ld_v = 0, ld_auto = 0;
    reg        auto_fb = 0;
    wire       toggle_auto, period_valid, pic_ev, saved_auto;
    wire [31:0] period_cycles;
    wire [8:0] tenths;
    wire [2:0] saved_sel;

    // ---- 外接光照闭环：ambient_brightness ↔ media_policy（dut = BRI_ENV_RESET 1，dut0 = 0 对照）----
    reg        als_ok = 0, als_rv = 0;
    reg [15:0] als_raw = 0;
    reg        a_inc = 0, a_dec = 0, a_def = 0, ld_neg = 0;
    wire [8:0] off1, off0, sv_off;
    wire [7:0] base1, eff1, base0, eff0;
    wire       env1, env0;
    ambient_brightness u_amb  (.clk(clk), .rst_n(rst_n), .tick_ms(tick), .sensor_ok(als_ok), .raw_valid(als_rv), .raw(als_raw),
                               .offset(off1), .base(base1), .eff_brightness(eff1), .env_change(env1));
    ambient_brightness u_amb0 (.clk(clk), .rst_n(rst_n), .tick_ms(tick), .sensor_ok(als_ok), .raw_valid(als_rv), .raw(als_raw),
                               .offset(off0), .base(base0), .eff_brightness(eff0), .env_change(env0));
    media_policy #(.CLK_HZ(CLK_HZ), .DEFAULT_PERIOD_SEL(3'd3), .DEFAULT_AUTO(1'b1), .BRI_ENV_RESET(1'b0)) dut0 (
        .clk(clk), .rst_n(rst_n), .enable(en),
        .next_event(1'b0), .prev_event(1'b0), .auto_event(1'b0), .bright_event(1'b0),
        .vol_event(1'b0), .period_event(1'b0), .save_event(1'b0), .auto_tick(1'b0),
        .adj_inc(a_inc), .adj_dec(a_dec), .adj_default(a_def), .adj_sel(1'b0), .adj_show(1'b0),
        .ir_present(1'b0), .disp_brightness(eff0), .bri_base(base0), .env_change(env0),
        .current_image_idx(6'd0), .image_count(cnt), .auto_enabled(1'b0),
        .cfg_load_valid(ld_v), .cfg_period_sel(3'd3), .cfg_brightness(ld_neg ? 9'h19C : 9'd0), .cfg_contrast(8'd128),
        .cfg_volume(8'd128), .cfg_auto_mode(ld_auto),
        .goto_valid(), .goto_image_idx(), .goto_audio_idx(), .toggle_auto_event(),
        .period_cfg_valid(), .period_cfg_cycles(),
        .media_id(), .audio_id(), .linked_event(), .bri_offset(off0), .contrast(), .volume(),
        .period_tenths(), .pic_event(), .vol_page_event(), .prd_event(), .bri_event(), .con_event(),
        .save_done_event(), .saved_period_sel(), .saved_brightness(), .saved_contrast(), .saved_volume(),
        .saved_auto_mode(), .saved_valid());

    media_policy #(.CLK_HZ(CLK_HZ), .DEFAULT_PERIOD_SEL(3'd3), .DEFAULT_AUTO(1'b1), .BRI_ENV_RESET(1'b1)) dut (
        .clk(clk), .rst_n(rst_n), .enable(en),
        .next_event(1'b0), .prev_event(1'b0), .auto_event(auto_key), .bright_event(1'b0),
        .vol_event(1'b0), .period_event(1'b0), .save_event(save_e), .auto_tick(1'b0),
        .adj_inc(a_inc), .adj_dec(a_dec), .adj_default(a_def), .adj_sel(1'b0), .adj_show(1'b0),
        .ir_present(ir_present), .disp_brightness(eff1), .bri_base(base1), .env_change(env1),
        .current_image_idx(6'd0), .image_count(cnt), .auto_enabled(auto_fb),
        .cfg_load_valid(ld_v), .cfg_period_sel(3'd3), .cfg_brightness(ld_neg ? 9'h19C : 9'd0), .cfg_contrast(8'd128),
        .cfg_volume(8'd128), .cfg_auto_mode(ld_auto),
        .goto_valid(), .goto_image_idx(), .goto_audio_idx(), .toggle_auto_event(toggle_auto),
        .period_cfg_valid(period_valid), .period_cfg_cycles(period_cycles),
        .media_id(), .audio_id(), .linked_event(), .bri_offset(off1), .contrast(), .volume(),
        .period_tenths(tenths), .pic_event(pic_ev), .vol_page_event(), .prd_event(), .bri_event(), .con_event(),
        .save_done_event(), .saved_period_sel(saved_sel), .saved_brightness(sv_off), .saved_contrast(), .saved_volume(),
        .saved_auto_mode(saved_auto), .saved_valid());

    integer n_toggle = 0;
    always @(posedge clk) begin
        if (toggle_auto) n_toggle = n_toggle + 1;
        if (!rst_n) auto_fb <= 1'b0;
        else if (toggle_auto && cnt > 7'd1) auto_fb <= ~auto_fb;
    end

    integer errors = 0;
    task expect_auto(input exp, input [8*40-1:0] tag);
        begin
            if (auto_fb !== exp) begin $display("FAIL %0s auto=%b exp=%b", tag, auto_fb, exp); errors = errors + 1; end
            else $display("PASS %0s (auto=%b toggles=%0d)", tag, auto_fb, n_toggle);
        end
    endtask
    task hand(input on, input integer ticks);
        begin ir_n = ~on; repeat (8 * ticks) @(posedge clk); end
    endtask
    task key3;
        begin @(posedge clk) auto_key <= 1; @(posedge clk) auto_key <= 0; repeat (10) @(posedge clk); end
    endtask
    task reboot;
        begin
            rst_n = 0; en = 0; n_toggle = 0; repeat (5) @(posedge clk); rst_n = 1; repeat (5) @(posedge clk);
        end
    endtask

    task adjb(input i, input d, input df);
        begin
            @(posedge clk) begin a_inc <= i; a_dec <= d; a_def <= df; end
            @(posedge clk) begin a_inc <= 0; a_dec <= 0; a_def <= 0; end
            repeat (4) @(posedge clk);
        end
    endtask
    task lux(input [15:0] r);
        begin @(posedge clk) begin als_raw <= r; als_rv <= 1; end @(posedge clk) als_rv <= 0; end
    endtask
    task amb_settle;
        begin repeat (8 * 8 * 220) @(posedge clk); end
    endtask
    task expect_bri(input [7:0] e1, input [7:0] e0, input [8:0] o1, input [8:0] o0, input [8*64-1:0] tag);
        begin
            if (eff1 !== e1 || eff0 !== e0 || off1 !== o1 || off0 !== o0) begin
                $display("FAIL %0s: eff %0d/%0d off %0d/%0d (exp %0d/%0d off %0d/%0d) base %0d", tag, eff1, eff0,
                         $signed(off1), $signed(off0), e1, e0, $signed(o1), $signed(o0), base1);
                errors = errors + 1;
            end else
                $display("PASS %0s (eff %0d/%0d)", tag, eff1, eff0);
        end
    endtask

    integer t0;
    initial begin
        // ---------------- A ----------------
        reboot;
        if (tenths !== 9'd50 || period_cycles !== 32'd5000) begin
            $display("FAIL default period tenths=%0d cycles=%0d", tenths, period_cycles); errors = errors + 1;
        end else $display("PASS default period 5.0 s");
        repeat (100) @(posedge clk);
        expect_auto(0, "A before enable: still off");
        en = 1; repeat (20) @(posedge clk);
        expect_auto(1, "A boot default auto on");

        // ---------------- B ----------------
        t0 = n_toggle;
        hand(1, 20); hand(0, 80);
        if (n_toggle != t0) begin $display("FAIL glitch 20ms toggled"); errors = errors + 1; end
        else $display("PASS 20ms glitch ignored");
        hand(1, 70);
        expect_auto(0, "B hand -> manual");
        hand(0, 70);
        expect_auto(1, "B release -> auto");

        // ---------------- C ----------------
        hand(1, 70);
        expect_auto(0, "C hand -> manual");
        key3;
        expect_auto(1, "C KEY3 during hand -> auto");
        t0 = n_toggle;
        hand(0, 70);
        expect_auto(1, "C release while auto: stays");
        if (n_toggle != t0) begin $display("FAIL C extra toggle"); errors = errors + 1; end

        // ---------------- D ----------------
        key3;
        expect_auto(0, "D KEY3 no hand -> manual");
        repeat (2000) @(posedge clk);
        expect_auto(0, "D stays manual");

        // ---------------- E ----------------
        key3;
        @(posedge clk) save_e <= 1; @(posedge clk) save_e <= 0; repeat (3) @(posedge clk);
        if (!saved_auto || saved_sel !== 3'd3) begin $display("FAIL E snapshot auto=%b sel=%0d", saved_auto, saved_sel); errors = errors + 1; end
        else $display("PASS E snapshot auto=1 period_sel=3 (5 s)");

        // ---------------- F ----------------
        reboot;
        @(posedge clk) begin ld_v <= 1; ld_auto <= 0; end
        @(posedge clk) ld_v <= 0;
        repeat (50) @(posedge clk); en = 1; repeat (200) @(posedge clk);
        expect_auto(0, "F flash=manual: boot manual");
        hand(1, 70); hand(0, 70);
        expect_auto(1, "F hand+release -> auto");

        // ---------------- G ----------------
        ir_n = 0;
        reboot;
        repeat (8 * 70) @(posedge clk);       // 滤波器稳定为"有遮挡"
        en = 1; repeat (200) @(posedge clk);
        expect_auto(0, "G boot with hand: manual");
        if (n_toggle != 0) begin $display("FAIL G toggles=%0d (flicker)", n_toggle); errors = errors + 1; end
        hand(0, 70);
        expect_auto(1, "G release -> auto");

        // ---------------- H ----------------
        hand(1, 70);
        expect_auto(0, "H hand -> manual");
        cnt = 7'd1;
        hand(0, 70);
        expect_auto(0, "H 1 image: request pending");
        cnt = 7'd3; repeat (20) @(posedge clk);
        expect_auto(1, "H images arrive -> auto");

        // ================= K 外接光照闭环：显示 = 限幅(基准 + 偏移) =================
        ir_n = 1'b1;
        reboot; als_ok = 0; en = 1; repeat (20) @(posedge clk);
        expect_bri(255, 255, 9'd0, 9'd0, "K1 no sensor: 255");
        adjb(0, 1, 0);
        expect_bri(239, 239, -9'sd16, -9'sd16, "K1 no sensor: KEY dec -> 239");
        als_ok = 1; lux(9); amb_settle;
        expect_bri(48, 48, -9'sd16, -9'sd16, "K2 first reading (dark 64): offset kept -> 48");
        adjb(1, 0, 0); adjb(1, 0, 0); adjb(1, 0, 0);
        expect_bri(96, 96, 9'sd32, 9'sd32, "K3 KEY inc x3 in dark -> 96 (above base)");
        lux(1200); amb_settle;
        expect_bri(255, 255, 9'd0, 9'sd32, "K4 bright env: RESET=1 offset 0 / RESET=0 keeps +32");
        lux(30); amb_settle;
        expect_bri(113, 145, 9'd0, 9'sd32, "K4 dim env 113: RESET=1 -> 113 / RESET=0 -> 145");
        adjb(0, 1, 0);
        expect_bri(97, 129, -9'sd16, 9'sd16, "K5 KEY dec at 113");
        @(posedge clk) save_e <= 1; @(posedge clk) save_e <= 0; repeat (3) @(posedge clk);
        if (sv_off !== 9'h1F0) begin $display("FAIL K5 saved offset %h (exp -16)", sv_off); errors = errors + 1; end
        else $display("PASS K5 save stores offset -16");
        adjb(0, 0, 1);
        expect_bri(113, 113, 9'd0, 9'd0, "K6 KEY default -> follow environment");
        repeat (20) adjb(1, 0, 0);
        expect_bri(255, 255, 9'sd142, 9'sd142, "K7 KEY inc saturates at 255 (offset 255-113)");
        // K8 Flash 偏移 -100 上电：第一次读数不冲掉；之后新环境才清零
        ld_neg = 1;
        reboot;
        @(posedge clk) ld_v <= 1; @(posedge clk) ld_v <= 0;
        en = 1;
        lux(120); amb_settle;
        expect_bri(69, 69, -9'sd100, -9'sd100, "K8 boot: Flash offset -100 kept on first reading (169-100)");
        lux(1200); amb_settle;
        expect_bri(255, 155, 9'd0, -9'sd100, "K8 new env: RESET=1 -> 255 / RESET=0 -> 155");
        ld_neg = 0;

        if (errors == 0) $display("TB_SENSOR_POLICY PASS");
        else             $display("TB_SENSOR_POLICY FAIL errors=%0d", errors);
        $finish;
    end
endmodule
