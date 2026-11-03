`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// tb_player_config_persistence： 配置持久化（DUT + W25Q64 行为模型）
//   P1  空片上电：JEDEC 通过、两槽无效 → 不下发 cfg_load_valid（播放器用默认值）
//   P2  第一次保存 → SLOT0(0x7FE000) seq=1，记录逐字节正确（魔数/版本/字段/CRC-8）
//   P3  第二次保存 → SLOT1(0x7FF000) seq=2，SLOT0 不动（双槽交替）
//   P4  重新上电 → 加载 seq 较新的 SLOT1
//   P5  SLOT1 损坏（CRC 错）→ 回退 SLOT0
//   P6  seq 模 256：FF 与 00 → 00 较新；05 与 04 → 05 较新
//   P7  擦除中途掉电 → 重新上电加载另一个槽（旧配置），被写槽作废
//   P8  擦除超时（WIP 不清）→ 0x5C，当前槽不变；恢复后可正常保存
//   P9  读回校验失败（编程无效）→ 0x5C；P10 WEL 未置位 → 0x5C
//   P11 忙时连续保存请求 → 排队一次、用最新快照；P12 上电加载期间的保存请求 → 加载后执行
//   P13 Flash 不存在（MISO 悬空）→ 不加载，保存报 0x5B；P14 掉电模式 → ABh 唤醒后正常
//   P15 记录 v2：亮度 8 位 + 对比度（对比度取 vol^5A 让每条记录不同）；v1 旧记录仍可加载
//       （亮度档位 n → {n,111111}，对比度 128），v1 档位 > 3 拒收；v1 与 v2 槽按 seq 比较
//   P16 存全部设置：byte11 bit1 = KEY3 底部隐藏，存了读回为 1；旧 v3 / v2 记录（该位 0）加载为 0；
//       保存进行中输入再变 → 记录按开工那拍的值写、读回校验照常通过
//   全程：只擦写两个配置扇区、WP#/HOLD# 恒高
// -----------------------------------------------------------------------------
module tb_player_config_persistence;
    reg clk = 1'b0;
    always #10 clk = ~clk;                      // 50 MHz
    reg rst_n = 1'b0;

    reg        save_req = 1'b0;
    reg [2:0]  s_psel = 3'd0;
    reg [8:0]  s_bri  = 9'd0;          // 外接传感器：亮度偏移（有符号 9 位）
    reg [7:0]  s_con  = 8'd0;
    reg [7:0]  s_vol  = 8'd0;
    reg        s_auto = 1'b0;
    reg        s_hide = 1'b0;

    wire       ld_valid, save_ok, err_ev, flash_ok, busy, act_valid, act_slot, ld_hide;
    wire [2:0] ld_psel;
    wire [8:0] ld_bri;
    wire [7:0] ld_con;
    wire [7:0] ld_vol, err_code, act_seq;
    wire       ld_auto;
    wire [4:0] st_code;
    wire       cs_n, sclk, mosi, miso, wp_n, hold_n;
    pullup (miso);

    player_config_persistence #(
        .ERASE_TIMEOUT_MS       (20_000),      // tick 接 1 → 按周期：400 us（模型擦除 50 us）
        .PROGRAM_TIMEOUT_MS     (2_000),       // 40 us（模型编程 5 us）
        .STARTUP_MS             (20),
        .WAKE_MS                (20)
    ) dut (
        .clk(clk), .rst_n(rst_n), .tick(1'b1),
        .save_request(save_req), .save_period_sel(s_psel), .save_brightness(s_bri), .save_contrast(s_con),
        .save_volume(s_vol), .save_auto_mode(s_auto), .save_ui_hide(s_hide),
        .cfg_load_valid(ld_valid), .cfg_period_sel(ld_psel), .cfg_brightness(ld_bri), .cfg_contrast(ld_con),
        .cfg_volume(ld_vol), .cfg_auto_mode(ld_auto), .cfg_ui_hide(ld_hide),
        .save_ok(save_ok), .error_event(err_ev), .error_code(err_code),
        .flash_ok(flash_ok), .busy(busy), .active_valid(act_valid), .active_slot(act_slot),
        .active_seq(act_seq), .state_code(st_code),
        .flash_cs_n(cs_n), .flash_sclk(sclk), .flash_mosi(mosi), .flash_miso(miso),
        .flash_wp_n(wp_n), .flash_hold_n(hold_n)
    );

    w25q64_model #(.ERASE_NS(50_000), .PROG_NS(5_000)) flash (
        .cs_n(cs_n), .sclk(sclk), .mosi(mosi), .miso(miso), .wp_n(wp_n), .hold_n(hold_n)
    );

    integer errors = 0;
    task err(input [8*72-1:0] msg);
        begin errors = errors + 1; $display("FAIL @%0t: %0s", $time, msg); end
    endtask

    // 事件计数与最后一次加载值
    integer n_load = 0, n_ok = 0, n_err = 0;
    reg [7:0] last_err = 8'h00;
    reg [2:0] l_psel; reg [8:0] l_bri; reg [7:0] l_con; reg [7:0] l_vol; reg l_auto; reg l_hide;
    always @(posedge clk) begin
        if (ld_valid) begin n_load = n_load + 1; l_psel = ld_psel; l_bri = ld_bri; l_con = ld_con; l_vol = ld_vol; l_auto = ld_auto; l_hide = ld_hide; end
        if (save_ok) n_ok = n_ok + 1;
        if (err_ev) begin n_err = n_err + 1; last_err = err_code; end
    end

    // SCK 最短半周期（12.5 MHz → 40 ns）
    realtime t_edge = 0.0, min_half = 1.0e9;
    always @(sclk) if (!cs_n) begin
        if (t_edge > 0.0 && ($realtime - t_edge) < min_half) min_half = $realtime - t_edge;
        t_edge = $realtime;
    end
    always @(posedge cs_n) t_edge = 0.0;

    // ---------------- 参考 CRC / 记录 ----------------
    function [7:0] crc8(input [127:0] r);
        integer i, j; reg [7:0] c;
        begin
            c = 8'h00;
            for (i = 0; i < 15; i = i + 1) begin
                c = c ^ r[8*i +: 8];
                for (j = 0; j < 8; j = j + 1) c = c[7] ? ({c[6:0], 1'b0} ^ 8'h07) : {c[6:0], 1'b0};
            end
            crc8 = c;
        end
    endfunction
    // DUT 逐位串行 CRC 的参考：每字节 MSB 先逐位移入（与逐字节 crc8 等价）
    function [7:0] crc8_serial(input [127:0] r);
        integer i, j; reg [7:0] c; reg b;
        begin
            c = 8'h00;
            for (i = 0; i < 15; i = i + 1)
                for (j = 7; j >= 0; j = j - 1) begin
                    b = r[8*i + j];
                    c = {c[6:0], 1'b0} ^ ({8{c[7] ^ b}} & 8'h07);
                end
            crc8_serial = c;
        end
    endfunction
    // v3（外接传感器）：byte7 = 亮度偏移低 8 位，byte11 bit0 = 偏移符号、bit1 = 底部隐藏，byte10 = 对比度（= vol ^ 5A）
    function [127:0] mkrec_h(input [7:0] seq, input [2:0] psel, input [8:0] bri, input [7:0] vol, input au, input hide);
        reg [127:0] r;
        begin
            r = {8'h00, 30'd0, hide, bri[8], vol ^ 8'h5A, 7'd0, au, vol, bri[7:0], 5'd0, psel, seq, 8'h03, 32'h4746_4350};
            r[127:120] = crc8(r);
            mkrec_h = r;
        end
    endfunction
    function [127:0] mkrec(input [7:0] seq, input [2:0] psel, input [8:0] bri, input [7:0] vol, input au);
        mkrec = mkrec_h(seq, psel, bri, vol, au, 1'b0);
    endfunction
    // v2：byte7 = 8 位亮度
    function [127:0] mkrec_v2(input [7:0] seq, input [2:0] psel, input [7:0] bri, input [7:0] vol, input au);
        reg [127:0] r;
        begin
            r = {8'h00, 32'd0, vol ^ 8'h5A, 7'd0, au, vol, bri, 5'd0, psel, seq, 8'h02, 32'h4746_4350};
            r[127:120] = crc8(r);
            mkrec_v2 = r;
        end
    endfunction
    // v1：byte7 = 2 位档位
    function [127:0] mkrec_v1(input [7:0] seq, input [2:0] psel, input [7:0] lvl, input [7:0] vol, input au);
        reg [127:0] r;
        begin
            r = {8'h00, 40'd0, 7'd0, au, vol, lvl, 5'd0, psel, seq, 8'h01, 32'h4746_4350};
            r[127:120] = crc8(r);
            mkrec_v1 = r;
        end
    endfunction
    function [127:0] slot_rec(input s);
        integer k; reg [127:0] r;
        begin
            for (k = 0; k < 16; k = k + 1) r[8*k +: 8] = flash.mem[{s, 12'd0} + k];
            slot_rec = r;
        end
    endfunction
    task put_rec(input s, input [127:0] r);
        integer k;
        begin
            for (k = 0; k < 4096; k = k + 1) flash.mem[{s, 12'd0} + k] = 8'hFF;
            for (k = 0; k < 16; k = k + 1) flash.mem[{s, 12'd0} + k] = r[8*k +: 8];
        end
    endtask
    task erase_all;
        integer k;
        begin for (k = 0; k < 8192; k = k + 1) flash.mem[k] = 8'hFF; end
    endtask

    // ---------------- 过程 ----------------
    task power_on;                              // 复位后等回到 READY（空闲）
        begin
            rst_n = 1'b0; #200; rst_n = 1'b1;
            #100;
            wait (!busy);
            #200;
        end
    endtask
    task request(input [2:0] p, input [8:0] b, input [7:0] v, input a);
        begin
            @(negedge clk);
            s_psel = p; s_bri = b; s_con = v ^ 8'h5A; s_vol = v; s_auto = a;
            save_req = 1'b1;
            @(negedge clk);
            save_req = 1'b0;
        end
    endtask
    task save_and_wait(input [2:0] p, input [8:0] b, input [7:0] v, input a);
        integer ok0, err0;
        begin
            ok0 = n_ok; err0 = n_err;
            request(p, b, v, a);
            wait (n_ok != ok0 || n_err != err0 || $time > 64'd100_000_000_000);
            #200;
        end
    endtask

    integer ok0, err0, er0, ld0;
    reg [127:0] r0, r1, rA, rB;
    reg [7:0]   crc_d;
    integer     ci;
    initial begin
        // CRC-8/SMBUS 标准校验值："123456789" → F4（验证 TB 参考函数与 DUT 同一算法且算法本身正确）
        r0 = 128'd0;
        r0[119:48] = {8'h39, 8'h38, 8'h37, 8'h36, 8'h35, 8'h34, 8'h33, 8'h32, 8'h31};   // 前导 0 字节不影响 init=0 的 CRC
        crc_d = crc8_serial(r0);                           // 存全部设置：DUT 改逐位串行 CRC，与逐字节结果须相同
        if (crc8(r0) !== crc_d || crc_d !== 8'hF4) err("CRC-8 check value \"123456789\" != F4");
        for (ci = 0; ci < 64; ci = ci + 1) begin           // 随机记录：逐位 = 逐字节
            r0 = {$random, $random, $random, $random};
            if (crc8_serial(r0) !== crc8(r0)) err("serial CRC != bytewise CRC");
        end
        // ---------------- P1 空片 ----------------
        power_on;
        if (!flash_ok) err("P1 JEDEC EF4017 not accepted");
        if (n_load != 0 || act_valid) err("P1 blank flash must not load a config");
        if (flash.n_jedec != 1) err("P1 expected exactly one JEDEC read");
        $display("INFO P1 blank: flash_ok=%0d loads=%0d active_valid=%0d", flash_ok, n_load, act_valid);

        // ---------------- P2 第一次保存 → SLOT0 ----------------
        save_and_wait(3'd4, 2'd1, 8'd200, 1'b1);
        rA = mkrec(8'd1, 3'd4, 2'd1, 8'd200, 1'b1);
        if (n_ok != 1 || n_err != 0) err("P2 first save did not complete OK");
        if (slot_rec(0) !== rA) begin err("P2 SLOT0 record wrong"); $display("  got %h exp %h", slot_rec(0), rA); end
        if (slot_rec(1) !== {128{1'b1}}) err("P2 SLOT1 must stay blank");
        if (!act_valid || act_slot !== 1'b0 || act_seq !== 8'd1) err("P2 active slot/seq wrong");
        if (flash.n_erase != 1 || flash.n_program != 1) err("P2 expected 1 erase + 1 program");
        $display("INFO P2 saved slot0 %h", slot_rec(0));

        // ---------------- P3 第二次保存 → SLOT1 ----------------
        save_and_wait(3'd0, 2'd2, 8'd7, 1'b0);
        rB = mkrec(8'd2, 3'd0, 2'd2, 8'd7, 1'b0);
        if (n_ok != 2 || n_err != 0) err("P3 second save did not complete OK");
        if (slot_rec(1) !== rB) err("P3 SLOT1 record wrong");
        if (slot_rec(0) !== rA) err("P3 SLOT0 must be untouched");
        if (act_slot !== 1'b1 || act_seq !== 8'd2) err("P3 active slot/seq wrong");

        // ---------------- P4 重新上电 → 较新的 SLOT1 ----------------
        ld0 = n_load;
        power_on;
        if (n_load != ld0 + 1) err("P4 expected one cfg_load_valid");
        if ({l_psel, l_bri, l_con, l_vol, l_auto} !== {3'd0, 9'd2, 8'd7 ^ 8'h5A, 8'd7, 1'b0}) err("P4 loaded values != SLOT1");
        if (act_slot !== 1'b1 || act_seq !== 8'd2) err("P4 active slot/seq wrong");
        $display("INFO P4 reload: period_sel=%0d bri=%0d vol=%0d auto=%0d from slot%0d seq=%0d", l_psel, l_bri, l_vol, l_auto, act_slot, act_seq);

        // ---------------- P5 SLOT1 CRC 错 → 回退 SLOT0 ----------------
        flash.mem[13'h1008] = 8'd8;              // SLOT1 volume 7 → 8，CRC 不再匹配
        ld0 = n_load;
        power_on;
        if (n_load != ld0 + 1 || {l_psel, l_bri, l_vol, l_auto} !== {3'd4, 9'd1, 8'd200, 1'b1}) err("P5 corrupt SLOT1 must fall back to SLOT0");
        if (act_slot !== 1'b0 || act_seq !== 8'd1) err("P5 active slot/seq wrong");
        // 接着保存：必须写“非当前槽”SLOT1（覆盖损坏记录），seq=2
        save_and_wait(3'd5, 2'd0, 8'd255, 1'b0);
        if (slot_rec(1) !== mkrec(8'd2, 3'd5, 2'd0, 8'd255, 1'b0) || slot_rec(0) !== rA) err("P5 save after fallback wrote the wrong slot");

        // 字段越界（CRC 正确但 period_sel=6）也视为无效
        r1 = mkrec(8'd9, 3'd6, 2'd0, 8'd1, 1'b0);
        put_rec(1, r1);
        ld0 = n_load;
        power_on;
        if (act_slot !== 1'b0 || l_vol !== 8'd200) err("P5 out-of-range record must be rejected");

        // ---------------- P6 seq 模 256 ----------------
        put_rec(0, mkrec(8'hFF, 3'd1, 2'd1, 8'd11, 1'b0));
        put_rec(1, mkrec(8'h00, 3'd2, 2'd2, 8'd22, 1'b1));
        power_on;
        if (act_slot !== 1'b1 || l_vol !== 8'd22) err("P6 seq 00 must be newer than FF");
        put_rec(0, mkrec(8'h05, 3'd3, 2'd3, 8'd33, 1'b0));
        put_rec(1, mkrec(8'h04, 3'd2, 2'd2, 8'd44, 1'b1));
        power_on;
        if (act_slot !== 1'b0 || l_vol !== 8'd33) err("P6 seq 05 must be newer than 04");
        save_and_wait(3'd1, 2'd1, 8'd66, 1'b1);
        if (slot_rec(1) !== mkrec(8'h06, 3'd1, 2'd1, 8'd66, 1'b1)) err("P6 next seq must be 06 into SLOT1");

        // ---------------- P7 擦除中途掉电 ----------------
        put_rec(0, rA);                           // seq 1
        put_rec(1, rB);                           // seq 2（当前）
        power_on;
        if (act_slot !== 1'b1) err("P7 setup: SLOT1 should be active");
        er0 = flash.n_erase;
        request(3'd3, 2'd3, 8'd99, 1'b1);        // 目标 SLOT0
        wait (flash.n_erase == er0 + 1);
        #40_000;                                  // 擦除进行到一半以后（前半扇区已擦）
        flash.power_cut;
        rst_n = 1'b0;                             // 同时 FPGA 掉电
        if (slot_rec(0) === rA) err("P7 setup: SLOT0 should be half-erased");
        ld0 = n_load;
        power_on;
        if (n_load != ld0 + 1 || act_slot !== 1'b1 || l_vol !== 8'd7) err("P7 must reload the old SLOT1 after power loss mid-erase");
        $display("INFO P7 power cut mid-erase: reloaded slot%0d vol=%0d", act_slot, l_vol);

        // ---------------- P8 擦除超时 ----------------
        flash.stuck_busy = 1'b1;
        err0 = n_err;
        save_and_wait(3'd2, 2'd2, 8'd1, 1'b0);
        if (n_err != err0 + 1 || last_err !== 8'h5C) err("P8 erase timeout must report 0x5C");
        if (act_slot !== 1'b1 || act_seq !== 8'd2) err("P8 active slot must not change on failure");
        flash.stuck_busy = 1'b0;
        #100;
        ok0 = n_ok;
        save_and_wait(3'd2, 2'd2, 8'd1, 1'b0);
        if (n_ok != ok0 + 1 || slot_rec(0) !== mkrec(8'd3, 3'd2, 2'd2, 8'd1, 1'b0)) err("P8 save after recovery failed");

        // ---------------- P9 读回不一致 / P10 WEL 未置位 ----------------
        flash.ignore_program = 1'b1;
        err0 = n_err;
        save_and_wait(3'd1, 2'd1, 8'd2, 1'b0);
        if (n_err != err0 + 1 || last_err !== 8'h5C || act_slot !== 1'b0) err("P9 verify failure must report 0x5C and keep slot");
        flash.ignore_program = 1'b0;
        flash.no_wel = 1'b1;
        er0 = flash.n_erase;
        err0 = n_err;
        save_and_wait(3'd1, 2'd1, 8'd2, 1'b0);
        if (n_err != err0 + 1 || last_err !== 8'h5C || flash.n_erase != er0) err("P10 missing WEL must report 0x5C without erasing");
        flash.no_wel = 1'b0;
        // 失败过的 SLOT1 已被擦成 FF → 重新上电仍然是 SLOT0 seq 3
        ld0 = n_load;
        power_on;
        if (act_slot !== 1'b0 || act_seq !== 8'd3 || l_vol !== 8'd1) err("P9/P10 reload after failed saves wrong");

        // ---------------- P11 忙时连续请求 ----------------
        ok0 = n_ok;
        request(3'd0, 2'd0, 8'd10, 1'b0);
        #2000;
        request(3'd1, 2'd1, 8'd20, 1'b1);
        #2000;
        request(3'd2, 2'd2, 8'd30, 1'b0);        // 最新快照
        wait (n_ok == ok0 + 2 || $time > 64'd100_000_000_000);
        #300_000;
        if (n_ok != ok0 + 2) err("P11 three requests during a save must give exactly two saves");
        if (slot_rec(0) !== mkrec(8'd5, 3'd2, 2'd2, 8'd30, 1'b0) || act_seq !== 8'd5) err("P11 queued save must use the latest snapshot");

        // ---------------- P12 上电加载期间的保存请求 ----------------
        ok0 = n_ok;
        rst_n = 1'b0; #200; rst_n = 1'b1;
        #300;
        request(3'd4, 2'd3, 8'd77, 1'b1);
        wait (n_ok == ok0 + 1 || $time > 64'd100_000_000_000);
        if (n_load < 1 || slot_rec(1) !== mkrec(8'd6, 3'd4, 2'd3, 8'd77, 1'b1)) err("P12 save during boot load must run after the load");
        #1000;

        // ---------------- P13 Flash 不存在 ----------------
        flash.absent = 1'b1;
        ld0 = n_load;
        power_on;
        if (flash_ok || n_load != ld0) err("P13 absent flash must not be accepted / loaded");
        err0 = n_err; er0 = flash.n_erase;
        save_and_wait(3'd1, 2'd1, 8'd1, 1'b1);
        if (n_err != err0 + 1 || last_err !== 8'h5B || flash.n_erase != er0) err("P13 save without flash must report 0x5B");
        flash.absent = 1'b0;

        // ---------------- P14 掉电模式 → ABh 唤醒 ----------------
        flash.powered_down = 1'b1;
        ld0 = n_load;
        power_on;
        if (!flash_ok || n_load != ld0 + 1 || l_vol !== 8'd77) err("P14 flash in power-down must be woken by ABh");

        // ---------------- P15 外接传感器 v3 有符号偏移 + v2 / v1 兼容 ----------------
        // v3：偏移 -82（9'h1AE，符号位进 byte11 bit0）存了读回逐位一致；正偏移 +191 同样
        save_and_wait(3'd2, 9'h1AE, 8'd91, 1'b0);
        rA = slot_rec(act_slot);
        if (rA !== mkrec(act_seq, 3'd2, 9'h1AE, 8'd91, 1'b0) || rA[88] !== 1'b1) begin err("P15 v3 record layout wrong"); $display("  got %h", rA); end
        ld0 = n_load;
        power_on;
        if (n_load != ld0 + 1 || l_bri !== 9'h1AE || l_con !== (8'd91 ^ 8'h5A)) err("P15 v3 reload of negative offset wrong");
        save_and_wait(3'd2, 9'd191, 8'd91, 1'b0);
        power_on;
        if (l_bri !== 9'd191) err("P15 v3 reload of positive offset wrong");
        // v2：8 位亮度 b → 偏移 b-255（无传感器时显示 255+偏移 = b，与旧版一致）
        put_rec(0, mkrec_v2(8'd30, 3'd2, 8'd173, 8'd91, 1'b0));
        put_rec(1, mkrec_v2(8'd29, 3'd2, 8'd255, 8'd91, 1'b0));
        power_on;
        if (act_slot !== 1'b0 || l_bri !== 9'h1AE || l_con !== (8'd91 ^ 8'h5A)) err("P15 v2 173 must load as offset -82");
        put_rec(0, mkrec_v2(8'd28, 3'd2, 8'd173, 8'd91, 1'b0));   // SLOT1(255) 较新
        power_on;
        if (act_slot !== 1'b1 || l_bri !== 9'd0) err("P15 v2 255 must load as offset 0");
        // v1：档位 → 8 位亮度 → 偏移
        put_rec(0, mkrec_v1(8'd40, 3'd1, 8'd2, 8'd44, 1'b0));   // 档位 2 → 191 → -64
        put_rec(1, mkrec(8'd39, 3'd3, 9'd9, 8'd55, 1'b0));
        ld0 = n_load;
        power_on;
        if (n_load != ld0 + 1 || act_slot !== 1'b0 || l_bri !== 9'h1C0 || l_con !== 8'd128 || l_vol !== 8'd44 || l_psel !== 3'd1)
            err("P15 v1 level 2 must load as offset -64 and contrast 128");
        put_rec(0, mkrec_v1(8'd40, 3'd1, 8'd0, 8'd44, 1'b0));   // 档位 0 → 63 → -192
        power_on;
        if (l_bri !== 9'h140) err("P15 v1 level 0 must load as offset -192");
        put_rec(0, mkrec_v1(8'd41, 3'd1, 8'd4, 8'd44, 1'b0));   // v1 档位 4 越界 → 拒收，用 SLOT1
        power_on;
        if (act_slot !== 1'b1 || l_vol !== 8'd55 || l_bri !== 9'd9) err("P15 v1 level > 3 must be rejected");
        $display("INFO P15 v3 offsets -82 / +191 round-trip; v2 173 -> -82, 255 -> 0; v1 level2 -> -64, level0 -> -192");

        // ---------------- P16 存全部设置：底部隐藏位 ----------------
        s_hide = 1'b1;
        save_and_wait(3'd2, 9'd5, 8'd123, 1'b1);
        rA = slot_rec(act_slot);
        if (rA !== mkrec_h(act_seq, 3'd2, 9'd5, 8'd123, 1'b1, 1'b1) || rA[89] !== 1'b1) begin err("P16 hide bit must go to byte11 bit1"); $display("  got %h", rA); end
        s_hide = 1'b0;
        ld0 = n_load;
        power_on;
        if (n_load != ld0 + 1 || l_hide !== 1'b1 || l_vol !== 8'd123 || l_auto !== 1'b1) err("P16 hide=1 must reload");
        put_rec(~act_slot, mkrec(act_seq + 8'd1, 3'd1, 9'd0, 8'd50, 1'b0));   // 较新的旧 v3 记录（隐藏位 0）
        power_on;
        if (l_hide !== 1'b0 || l_vol !== 8'd50) err("P16 old v3 record must load hide=0");
        put_rec(0, mkrec_v2(8'd60, 3'd1, 8'd255, 8'd51, 1'b0));
        put_rec(1, mkrec_v2(8'd59, 3'd1, 8'd255, 8'd52, 1'b0));
        power_on;
        if (act_slot !== 1'b0 || l_hide !== 1'b0 || l_vol !== 8'd51) err("P16 v2 record must load hide=0");
        // 槽 0 较新且隐藏 = 1、槽 1 较旧隐藏 = 0：后读的旧槽不能覆盖已采用的新槽
        put_rec(0, mkrec_h(8'd70, 3'd4, 9'd3, 8'd61, 1'b1, 1'b1));
        put_rec(1, mkrec(8'd69, 3'd0, 9'd0, 8'd62, 1'b0));
        power_on;
        if (act_slot !== 1'b0 || act_seq !== 8'd70 || l_hide !== 1'b1 || l_vol !== 8'd61 || l_psel !== 3'd4) err("P16 older SLOT1 must not override newer SLOT0");
        // 槽 0 较旧隐藏 = 1、槽 1 较新隐藏 = 0 → 用槽 1
        put_rec(1, mkrec(8'd71, 3'd0, 9'd0, 8'd62, 1'b0));
        power_on;
        if (act_slot !== 1'b1 || l_hide !== 1'b0 || l_vol !== 8'd62) err("P16 newer SLOT1 must override SLOT0");
        // 保存进行中输入再变：记录按开工那拍的值写，读回校验通过
        ok0 = n_ok; err0 = n_err; er0 = flash.n_erase;
        request(3'd3, 9'd7, 8'd88, 1'b0);
        wait (flash.n_erase == er0 + 1);
        @(negedge clk) begin s_vol = 8'd1; s_psel = 3'd0; s_hide = 1'b1; s_bri = 9'd0; end
        wait (n_ok != ok0 || n_err != err0 || $time > 64'd100_000_000_000);
        #200;
        if (n_ok != ok0 + 1 || n_err != err0 || slot_rec(act_slot) !== mkrec(act_seq, 3'd3, 9'd7, 8'd88, 1'b0))
            err("P16 inputs changing mid-save must not corrupt the record / verify");
        s_hide = 1'b0;
        $display("INFO P16 hide bit round-trip; v3/v2 old records -> hide 0; newest-slot selection; mid-save input change OK");

        // ---------------- 汇总 ----------------
        if (flash.bad_access != 0) err("erase/program outside the two config sectors");
        if (flash.proto_err != 0) err("WP#/HOLD# not held high");
        if (flash.n_busy_cmd != 0) err("non-status command sent while flash busy");
        if (min_half < 39.0) err("SCK half period below 40 ns");
        $display("INFO erases=%0d programs=%0d reads=%0d saves_ok=%0d errors_ev=%0d min SCK half=%0.1f ns",
                 flash.n_erase, flash.n_program, flash.n_read, n_ok, n_err, min_half);
        if (errors == 0) $display("PASS tb_player_config_persistence: P1-P15 (blank, dual-slot alternate, newest-seq load, CRC/range fallback, seq wrap, power cut mid-erase, erase timeout, verify/WEL failure, queued save, save during load, absent flash 0x5B, ABh wake, v3 signed brightness offset, v2/v1 compat, P16 ui-hide bit)");
        else             $display("FAIL tb_player_config_persistence: %0d errors", errors);
        $finish;
    end
    initial begin #200_000_000; $display("FAIL tb_player_config_persistence: timeout"); $finish; end
endmodule
