`timescale 1ns/1ps
// picture_adjust_ctrl集成：真实消抖模块 + 组合键过滤 + media_policy
//   原始按键带抖动 → dual_key_chord_router（KEY2/KEY3）/ single_key_event（KEY4）
//   → picture_adjust_ctrl → media_policy；按事件计数比对
//   K1 模式外：KEY3 单击=自动轮播、KEY3 长按=亮度阶梯 255→191、KEY4 单击=音量 +32、KEY4 长按=周期；无调节事件
//   K2 KEY3+KEY4 同按（任意先后、任意先松）→ 进入调节模式；组合过程不漏出任何 KEY3/KEY4 事件；进入时弹 BRI 页
//   K3 模式内：KEY3 单击 +16、KEY4 单击 -16、KEY3 长按切换 b/C（弹 CON 页）、KEY4 长按恢复默认；
//      原功能（自动轮播 / 阶梯 / 音量 / 周期）不触发
//   K4 模式内 KEY2 切图、KEY2+KEY3 保存仍可用（快照含 8 位亮度 / 对比度），保存组合不被当成 +16
//   K5 同按并长按（两个键都超过长按阈值）退出：不漏出长按事件（不恢复默认、不切项），数值保持
//   K6 退出后 KEY4 单击恢复音量功能
//   K7 空闲超时自动退出；退出后 KEY3 单击 = 自动轮播
//   K8 enable=0：强制退出且同按不能进入；media_policy 同时丢弃事件
module tb_picture_adjust_ctrl;
    localparam integer DEB = 20, LONG = 200, HOLD = 600, K2LONG = 400;
    localparam integer CLK_HZ = 1000, IDLE_MS = 1500;          // 1 拍 = 1 ms → 空闲 1500 拍
    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    reg key2_n = 1, key3_n = 1, key4_n = 1;
    reg en = 1;

    wire k2c, k2l, k3c, k3l, save_ev, k2s, k3s;
    wire k4c, k4l, k4s;
    dual_key_chord_router #(.DEBOUNCE_CYCLES(DEB), .HOLD_CYCLES(HOLD), .KEY3_LONG_CYCLES(LONG), .KEY2_LONG_CYCLES(K2LONG)) u_router (
        .clk(clk), .rst_n(rst_n), .tick(1'b1), .save_enabled(1'b1), .key2_n(key2_n), .key3_n(key3_n),
        .key2_event(k2c), .key2_long_event(k2l), .key3_event(k3c), .key3_long_event(k3l),
        .save_event(save_ev), .save_locked(), .key2_stable_n(k2s), .key3_stable_n(k3s));
    single_key_event #(.DEBOUNCE_CYCLES(DEB), .LONG_CYCLES(LONG)) u_key4 (
        .clk(clk), .rst_n(rst_n), .tick(1'b1), .key_n(key4_n), .click_event(k4c), .long_event(k4l), .key_stable_n(k4s));

    wire k3c_f, k3l_f, k4c_f, k4l_f;
    wire adj_mode, adj_sel, adj_inc, adj_dec, adj_def, adj_show;
    picture_adjust_ctrl #(.IDLE_EXIT_MS((CLK_HZ / 1000) * IDLE_MS)) dut (    // L2：tick=1，按拍数给
        .clk(clk), .rst_n(rst_n), .tick(1'b1), .enable(en),
        .key3_stable_n(k3s), .key4_stable_n(k4s),
        .key3_click_in(k3c), .key3_long_in(k3l), .key4_click_in(k4c), .key4_long_in(k4l),
        .key3_click_out(k3c_f), .key3_long_out(k3l_f), .key4_click_out(k4c_f), .key4_long_out(k4l_f),
        .adj_mode(adj_mode), .adj_sel(adj_sel), .adj_inc(adj_inc), .adj_dec(adj_dec),
        .adj_default(adj_def), .adj_show(adj_show));

    reg  auto_enabled = 0;
    wire toggle_auto, goto_valid, bri_ev, con_ev, vol_pg, prd_ev, save_done;
    wire [7:0] bright, contrast, volume, s_bri, s_con;
    wire [8:0] bri_off, s_off;
    // 外接传感器之后 media_policy 输出亮度偏移：无传感器（基准 255）时显示亮度 = 255 + 偏移
    assign bright = bri_off[7:0] + 8'hFF;
    assign s_bri  = s_off[7:0] + 8'hFF;
    wire [8:0] tenths;
    media_policy #(.CLK_HZ(CLK_HZ), .BC_STEP(8'd16)) u_policy (
        .clk(clk), .rst_n(rst_n), .enable(en),
        .next_event(k2c), .prev_event(k2l), .auto_event(k3c_f), .bright_event(k3l_f),
        .vol_event(k4c_f), .period_event(k4l_f), .save_event(save_ev), .auto_tick(1'b0),
        .adj_inc(adj_inc), .adj_dec(adj_dec), .adj_default(adj_def), .adj_sel(adj_sel), .adj_show(adj_show),
        .ir_present(1'b0), .disp_brightness(bright), .bri_base(8'hFF), .env_change(1'b0),
        .current_image_idx(6'd0), .image_count(7'd4), .auto_enabled(auto_enabled),
        .cfg_load_valid(1'b0), .cfg_period_sel(3'd0), .cfg_brightness(9'd0), .cfg_contrast(8'd0),
        .cfg_volume(8'd0), .cfg_auto_mode(1'b0),
        .goto_valid(goto_valid), .goto_image_idx(), .goto_audio_idx(), .toggle_auto_event(toggle_auto),
        .period_cfg_valid(), .period_cfg_cycles(),
        .media_id(), .audio_id(), .linked_event(), .bri_offset(bri_off), .contrast(contrast), .volume(volume),
        .period_tenths(tenths), .pic_event(), .vol_page_event(vol_pg), .prd_event(prd_ev), .bri_event(bri_ev),
        .con_event(con_ev), .save_done_event(save_done),
        .saved_period_sel(), .saved_brightness(s_off), .saved_contrast(s_con), .saved_volume(), .saved_auto_mode(),
        .saved_valid());
    always @(posedge clk) if (toggle_auto) auto_enabled <= ~auto_enabled;

    // ---------------- 事件计数 ----------------
    integer n_tog = 0, n_vol = 0, n_prd = 0, n_ladder = 0, n_inc = 0, n_dec = 0, n_def = 0, n_show = 0;
    integer n_goto = 0, n_save = 0, n_bri = 0, n_con = 0;
    always @(posedge clk) begin
        if (toggle_auto) n_tog = n_tog + 1;
        if (k4c_f) n_vol = n_vol + 1;
        if (k4l_f) n_prd = n_prd + 1;
        if (k3l_f) n_ladder = n_ladder + 1;
        if (adj_inc) n_inc = n_inc + 1;
        if (adj_dec) n_dec = n_dec + 1;
        if (adj_def) n_def = n_def + 1;
        if (adj_show) n_show = n_show + 1;
        if (goto_valid) n_goto = n_goto + 1;
        if (save_done) n_save = n_save + 1;
        if (bri_ev) n_bri = n_bri + 1;
        if (con_ev) n_con = n_con + 1;
    end
    // 单拍检查：调节事件只在模式内、原功能事件只在模式外
    integer errors = 0;
    always @(posedge clk) if (rst_n) begin
        if ((adj_inc || adj_dec || adj_def) && !adj_mode) begin $display("FAIL adj event outside mode @%0t", $time); errors = errors + 1; end
        if ((k3c_f || k3l_f || k4c_f || k4l_f) && adj_mode) begin $display("FAIL legacy event inside mode @%0t", $time); errors = errors + 1; end
    end

    // ---------------- 按键动作（带抖动） ----------------
    task setkey(input integer k, input v);
        case (k) 2: key2_n = v; 3: key3_n = v; default: key4_n = v; endcase
    endtask
    task bounce(input integer k, input v);       // 抖动 3 次（每段 < DEB）后稳定到 v
        begin
            setkey(k, v);  repeat (3) @(posedge clk);
            setkey(k, ~v); repeat (2) @(posedge clk);
            setkey(k, v);  repeat (4) @(posedge clk);
            setkey(k, ~v); repeat (3) @(posedge clk);
            setkey(k, v);
        end
    endtask
    task down(input integer k); bounce(k, 1'b0); endtask
    task up(input integer k);   bounce(k, 1'b1); endtask
    task wait_c(input integer n); repeat (n) @(posedge clk); endtask
    task click(input integer k); begin down(k); wait_c(60); up(k); wait_c(60); end endtask
    task longp(input integer k); begin down(k); wait_c(LONG + 80); up(k); wait_c(60); end endtask

    integer t0, v0, p0, l0, i0, d0, f0, s0, g0, sv0, b0, c0;
    task snap; begin t0 = n_tog; v0 = n_vol; p0 = n_prd; l0 = n_ladder; i0 = n_inc; d0 = n_dec; f0 = n_def;
                      s0 = n_show; g0 = n_goto; sv0 = n_save; b0 = n_bri; c0 = n_con; end endtask
    task expect_delta(input integer tog, input integer vol, input integer prd, input integer lad,
                      input integer inc, input integer dec, input integer def, input [8*40-1:0] tag);
        begin
            if (n_tog - t0 != tog || n_vol - v0 != vol || n_prd - p0 != prd || n_ladder - l0 != lad ||
                n_inc - i0 != inc || n_dec - d0 != dec || n_def - f0 != def) begin
                $display("FAIL %0s: tog=%0d vol=%0d prd=%0d ladder=%0d inc=%0d dec=%0d def=%0d", tag,
                         n_tog - t0, n_vol - v0, n_prd - p0, n_ladder - l0, n_inc - i0, n_dec - d0, n_def - f0);
                errors = errors + 1;
            end
        end
    endtask
    task expect_state(input mode, input sel, input [7:0] b, input [7:0] c, input [8*40-1:0] tag);
        if (adj_mode !== mode || adj_sel !== sel || bright !== b || contrast !== c) begin
            $display("FAIL %0s: mode=%b sel=%b bri=%0d con=%0d (exp %b %b %0d %0d)", tag, adj_mode, adj_sel, bright, contrast, mode, sel, b, c);
            errors = errors + 1;
        end
    endtask

    initial begin
        #33 rst_n = 1;
        wait_c(10);
        // ---- K1 模式外：原功能 ----
        snap; click(3);  expect_delta(1, 0, 0, 0, 0, 0, 0, "K1 KEY3 click = auto");
        if (auto_enabled !== 1'b1) begin $display("FAIL K1 auto toggle"); errors = errors + 1; end
        snap; longp(3);  expect_delta(0, 0, 0, 1, 0, 0, 0, "K1 KEY3 long = ladder");
        expect_state(0, 0, 8'd191, 8'd128, "K1 ladder 191");
        snap; click(4);  expect_delta(0, 1, 0, 0, 0, 0, 0, "K1 KEY4 click = volume");
        if (volume !== 8'd160) begin $display("FAIL K1 volume %0d", volume); errors = errors + 1; end
        snap; longp(4);  expect_delta(0, 0, 1, 0, 0, 0, 0, "K1 KEY4 long = period");
        if (tenths !== 9'd50) begin $display("FAIL K1 period tenths %0d", tenths); errors = errors + 1; end

        // ---- K2 同按进入：KEY3 先按、KEY4 先松 ----
        snap;
        down(3); wait_c(30); down(4); wait_c(60); up(4); wait_c(30); up(3); wait_c(60);
        expect_delta(0, 0, 0, 0, 0, 0, 0, "K2 chord enter leaks nothing");
        expect_state(1, 0, 8'd191, 8'd128, "K2 entered");
        if (n_show - s0 != 1 || n_bri - b0 != 1) begin $display("FAIL K2 adj_show/BRI page on entry"); errors = errors + 1; end

        // ---- K3 模式内 ----
        snap; click(3); expect_state(1, 0, 8'd207, 8'd128, "K3 bri +16");
        click(4); click(4); expect_state(1, 0, 8'd175, 8'd128, "K3 bri -32");
        expect_delta(0, 0, 0, 0, 1, 2, 0, "K3 inc/dec only");
        if (auto_enabled !== 1'b1 || volume !== 8'd160) begin $display("FAIL K3 legacy state touched"); errors = errors + 1; end
        snap; longp(3); expect_state(1, 1, 8'd175, 8'd128, "K3 KEY3 long -> sel contrast");
        if (n_con - c0 != 1) begin $display("FAIL K3 CON page on sel toggle"); errors = errors + 1; end
        click(3); click(3); expect_state(1, 1, 8'd175, 8'd160, "K3 con +32");
        longp(4); expect_state(1, 1, 8'd175, 8'd128, "K3 KEY4 long -> con default");
        click(4); expect_state(1, 1, 8'd175, 8'd112, "K3 con -16");
        expect_delta(0, 0, 0, 0, 2, 1, 1, "K3 contrast events");
        longp(3); longp(4); expect_state(1, 0, 8'd255, 8'd112, "K3 sel back, bri default");

        // ---- K4 模式内 KEY2 切图 / KEY2+KEY3 保存 ----
        snap; click(2);
        if (n_goto - g0 != 1) begin $display("FAIL K4 KEY2 next in mode"); errors = errors + 1; end
        click(4); expect_state(1, 0, 8'd239, 8'd112, "K4 bri -16 before save");
        snap;
        down(2); wait_c(10); down(3); wait_c(HOLD + 80); up(3); wait_c(10); up(2); wait_c(60);
        if (n_save - sv0 != 1 || s_bri !== 8'd239 || s_con !== 8'd112) begin
            $display("FAIL K4 save in mode: saves=%0d bri=%0d con=%0d", n_save - sv0, s_bri, s_con); errors = errors + 1;
        end
        expect_delta(0, 0, 0, 0, 0, 0, 0, "K4 save chord leaks no KEY3");
        expect_state(1, 0, 8'd239, 8'd112, "K4 still in mode");

        // ---- K5 同按并长按退出（KEY4 先按、KEY3 先松） ----
        snap;
        down(4); wait_c(15); down(3); wait_c(LONG + 120); up(3); wait_c(10); up(4); wait_c(60);
        expect_delta(0, 0, 0, 0, 0, 0, 0, "K5 long chord exit leaks nothing");
        expect_state(0, 0, 8'd239, 8'd112, "K5 exited, values kept");

        // ---- K6 退出后原功能恢复 ----
        snap; click(4); expect_delta(0, 1, 0, 0, 0, 0, 0, "K6 KEY4 click = volume again");
        if (volume !== 8'd192) begin $display("FAIL K6 volume %0d", volume); errors = errors + 1; end

        // ---- K7 空闲超时 ----
        down(3); down(4); wait_c(40); up(3); up(4); wait_c(40);
        expect_state(1, 0, 8'd239, 8'd112, "K7 re-entered");
        wait_c(IDLE_MS - 200);
        expect_state(1, 0, 8'd239, 8'd112, "K7 not yet timed out");
        click(3);                                     // 按键重置空闲计时
        wait_c(IDLE_MS - 200);
        expect_state(1, 0, 8'd255, 8'd112, "K7 key resets idle");
        wait_c(300);
        expect_state(0, 0, 8'd255, 8'd112, "K7 idle timeout exit");
        snap; click(3); expect_delta(1, 0, 0, 0, 0, 0, 0, "K7 KEY3 click = auto after timeout");

        // ---- K8 enable=0 ----
        down(3); down(4); wait_c(40); up(3); up(4); wait_c(40);
        expect_state(1, 0, 8'd255, 8'd112, "K8 entered");
        @(negedge clk) en = 0; wait_c(3);
        expect_state(0, 0, 8'd255, 8'd112, "K8 enable=0 forces exit");
        down(3); down(4); wait_c(40); up(3); up(4); wait_c(40);
        if (adj_mode !== 1'b0) begin $display("FAIL K8 chord must not enter when disabled"); errors = errors + 1; end
        @(negedge clk) en = 1; wait_c(5);

        if (errors == 0) $display("PASS tb_picture_adjust_ctrl (K1-K8: legacy keys, chord enter/exit swallow, inc/dec/sel/default, KEY2/save in mode, idle timeout, enable)");
        else             $display("FAIL tb_picture_adjust_ctrl errors=%0d", errors);
        $finish;
    end
endmodule
