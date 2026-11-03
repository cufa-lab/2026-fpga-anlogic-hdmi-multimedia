`timescale 1ns/1ps

module tour_sequencer #(
    parameter integer START_FRAMES = 3,
    parameter integer DWELL_FRAMES = 150
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        chord,
    input  wire        tour_next,
    input  wire        enable,               // display_valid_sys
    input  wire        block,                // 总览请求 / 已生效
    input  wire        start_block,          // 切图 / 转场在途：不进入
    input  wire [5:0]  cur_img,
    input  wire        frame_tick,           // 每帧一拍（pix frame_start 同步过来）
    input  wire [1:0]  disp_bank,            // 当前显示 bank（pix → sys）
    // 缩放器规划结果（g_* 在 plan_valid 之后保持到下一次规划）
    input  wire        fb_load_start,
    input  wire [1:0]  fb_load_bank,
    input  wire        plan_valid,
    input  wire [9:0]  g_ox,
    input  wire [8:0]  g_oyt,
    input  wire [11:0] g_c0,
    input  wire [11:0] g_r0t,
    input  wire [11:0] g_cw,
    input  wire [11:0] g_ch,
    input  wire [11:0] g_n,
    input  wire [9:0]  g_d,
    // 焦点表（focus_loader，同步读）
    output reg  [7:0]  ft_addr,
    input  wire [26:0] ft_data,
    output wire        tour_busy,            // 导览中（自动轮播暂停）
    output wire [80:0] bundle
);
    localparam [2:0] T_IDLE = 3'd0, T_GEO = 3'd1, T_START = 3'd2, T_FT = 3'd3, T_CALC = 3'd4,
                     T_FLY = 3'd5, T_DWELL = 3'd6, T_END = 3'd7;
    localparam [3:0] O_LD = 4'd0, O_ADD = 4'd1, O_SUB = 4'd2, O_MAX = 4'd3, O_MIN = 4'd4, O_ST = 4'd5,
                     O_MUL = 4'd6, O_DIV = 4'd7, O_BRN = 4'd8, O_BRNN = 4'd9, O_OUT = 4'd10, O_END = 4'd11;

    // @@UROM_BEGIN
    // ---- 微程序 ROM（asm_seq.py 生成，133 条）：入口 GEO=0 FRAME=32 FLAGS=50 CALC=52 HOME=127 ----
    // RAM 512×18（一块 BRAM9K）：0..31 工作变量、32..63 常数表、64..127 几何 {001, bank, idx}、256..511 微程序
    (* ram_style = "bram" *) reg [17:0] mem [0:511];
    initial begin
        mem[32] = 18'd0;
        mem[33] = 18'd1;
        mem[34] = 18'd4;
        mem[35] = 18'd64;
        mem[36] = 18'd20480;
        mem[37] = 18'd30720;
        mem[38] = 18'd32768;
        mem[39] = 18'd40960;
        mem[40] = 18'd49152;
        mem[41] = 18'd65536;
        mem[256] = 18'h00025;   // LD G.DD
        mem[257] = 18'h00643;   // MUL C.64
        mem[258] = 18'h000A0;   // LD P0
        mem[259] = 18'h00500;   // ST D64
        mem[260] = 18'h0002A;   // LD G.OX
        mem[261] = 18'h00643;   // MUL C.64
        mem[262] = 18'h000A0;   // LD P0
        mem[263] = 18'h00501;   // ST OX64
        mem[264] = 18'h0002B;   // LD G.OYT
        mem[265] = 18'h00643;   // MUL C.64
        mem[266] = 18'h000A0;   // LD P0
        mem[267] = 18'h00502;   // ST OY64
        mem[268] = 18'h00020;   // LD G.C0
        mem[269] = 18'h00122;   // ADD G.CW0
        mem[270] = 18'h00241;   // SUB C.1
        mem[271] = 18'h00503;   // ST CMAX
        mem[272] = 18'h00021;   // LD G.R0T
        mem[273] = 18'h00123;   // ADD G.CH0
        mem[274] = 18'h00241;   // SUB C.1
        mem[275] = 18'h00504;   // ST RMAX
        mem[276] = 18'h00040;   // LD C.0
        mem[277] = 18'h0050D;   // ST CU
        mem[278] = 18'h0050E;   // ST CV
        mem[279] = 18'h00510;   // ST UA
        mem[280] = 18'h00511;   // ST VA
        mem[281] = 18'h00513;   // ST DU
        mem[282] = 18'h00514;   // ST DV
        mem[283] = 18'h00515;   // ST DS
        mem[284] = 18'h00049;   // LD C.65536
        mem[285] = 18'h0050F;   // ST CS
        mem[286] = 18'h00512;   // ST SA
        mem[287] = 18'h00B00;   // END
        mem[288] = 18'h00013;   // LD R.DU
        mem[289] = 18'h00663;   // MUL E.E
        mem[290] = 18'h00080;   // LD P16
        mem[291] = 18'h00110;   // ADD R.UA
        mem[292] = 18'h0050D;   // ST CU
        mem[293] = 18'h00A00;   // OUT BU
        mem[294] = 18'h00014;   // LD R.DV
        mem[295] = 18'h00663;   // MUL E.E
        mem[296] = 18'h00080;   // LD P16
        mem[297] = 18'h00111;   // ADD R.VA
        mem[298] = 18'h0050E;   // ST CV
        mem[299] = 18'h00A01;   // OUT BV
        mem[300] = 18'h00015;   // LD R.DS
        mem[301] = 18'h00663;   // MUL E.E
        mem[302] = 18'h00080;   // LD P16
        mem[303] = 18'h00112;   // ADD R.SA
        mem[304] = 18'h0050F;   // ST CS
        mem[305] = 18'h00A02;   // OUT BS
        mem[306] = 18'h00A07;   // OUT FLAGS
        mem[307] = 18'h00B00;   // END
        mem[308] = 18'h00061;   // LD E.FTX
        mem[309] = 18'h00505;   // ST FX
        mem[310] = 18'h00062;   // LD E.FTY
        mem[311] = 18'h00506;   // ST FY
        mem[312] = 18'h00060;   // LD E.FTZ
        mem[313] = 18'h00141;   // ADD C.1
        mem[314] = 18'h00507;   // ST Z
        mem[315] = 18'h00005;   // LD R.FX
        mem[316] = 18'h00320;   // MAX G.C0
        mem[317] = 18'h00403;   // MIN R.CMAX
        mem[318] = 18'h00220;   // SUB G.C0
        mem[319] = 18'h00600;   // MUL R.D64
        mem[320] = 18'h00724;   // DIV G.NN
        mem[321] = 18'h00101;   // ADD R.OX64
        mem[322] = 18'h00516;   // ST FU
        mem[323] = 18'h00006;   // LD R.FY
        mem[324] = 18'h00321;   // MAX G.R0T
        mem[325] = 18'h00404;   // MIN R.RMAX
        mem[326] = 18'h00221;   // SUB G.R0T
        mem[327] = 18'h00600;   // MUL R.D64
        mem[328] = 18'h00724;   // DIV G.NN
        mem[329] = 18'h00102;   // ADD R.OY64
        mem[330] = 18'h00517;   // ST FV
        mem[331] = 18'h00044;   // LD C.20480
        mem[332] = 18'h00642;   // MUL C.4
        mem[333] = 18'h00707;   // DIV R.Z
        mem[334] = 18'h00508;   // ST W6
        mem[335] = 18'h00046;   // LD C.32768
        mem[336] = 18'h00642;   // MUL C.4
        mem[337] = 18'h00707;   // DIV R.Z
        mem[338] = 18'h0050C;   // ST TS
        mem[339] = 18'h00008;   // LD R.W6
        mem[340] = 18'h00648;   // MUL C.49152
        mem[341] = 18'h00080;   // LD P16
        mem[342] = 18'h00509;   // ST H6
        mem[343] = 18'h00047;   // LD C.40960
        mem[344] = 18'h00208;   // SUB R.W6
        mem[345] = 18'h00518;   // ST T0
        mem[346] = 18'h00008;   // LD R.W6
        mem[347] = 18'h00646;   // MUL C.32768
        mem[348] = 18'h00080;   // LD P16
        mem[349] = 18'h00519;   // ST T1
        mem[350] = 18'h00016;   // LD R.FU
        mem[351] = 18'h00219;   // SUB R.T1
        mem[352] = 18'h00340;   // MAX C.0
        mem[353] = 18'h00418;   // MIN R.T0
        mem[354] = 18'h0050A;   // ST TU
        mem[355] = 18'h00045;   // LD C.30720
        mem[356] = 18'h00209;   // SUB R.H6
        mem[357] = 18'h00518;   // ST T0
        mem[358] = 18'h00009;   // LD R.H6
        mem[359] = 18'h00646;   // MUL C.32768
        mem[360] = 18'h00080;   // LD P16
        mem[361] = 18'h00519;   // ST T1
        mem[362] = 18'h00017;   // LD R.FV
        mem[363] = 18'h00219;   // SUB R.T1
        mem[364] = 18'h00340;   // MAX C.0
        mem[365] = 18'h00418;   // MIN R.T0
        mem[366] = 18'h0050B;   // ST TV
        mem[367] = 18'h0000D;   // LD R.CU
        mem[368] = 18'h00510;   // ST UA
        mem[369] = 18'h0000A;   // LD R.TU
        mem[370] = 18'h0020D;   // SUB R.CU
        mem[371] = 18'h00513;   // ST DU
        mem[372] = 18'h0000E;   // LD R.CV
        mem[373] = 18'h00511;   // ST VA
        mem[374] = 18'h0000B;   // LD R.TV
        mem[375] = 18'h0020E;   // SUB R.CV
        mem[376] = 18'h00514;   // ST DV
        mem[377] = 18'h0000F;   // LD R.CS
        mem[378] = 18'h00512;   // ST SA
        mem[379] = 18'h0000C;   // LD R.TS
        mem[380] = 18'h0020F;   // SUB R.CS
        mem[381] = 18'h00515;   // ST DS
        mem[382] = 18'h00B00;   // END
        mem[383] = 18'h00040;   // LD C.0
        mem[384] = 18'h0050A;   // ST TU
        mem[385] = 18'h0050B;   // ST TV
        mem[386] = 18'h00049;   // LD C.65536
        mem[387] = 18'h0050C;   // ST TS
        mem[388] = 18'h0096F;   // BRNN fly
    end
    localparam [7:0] PA_GEO = 8'd0, PA_FRAME = 8'd32, PA_FLAGS = 8'd50, PA_CALC = 8'd52, PA_HOME = 8'd127;
    // @@UROM_END

    // ---------------- 状态 ----------------
    reg [2:0]  st;
    reg        cs;
    reg [1:0]  k;
    reg [5:0]  img_q;
    reg [1:0]  bank_q;
    reg [7:0]  fcnt;
    reg        going_home, home_req;
    reg        on;                            // 内部状态；bundle 位在 FLAGS 指令时提交
    // 缓动
    reg [18:0] f;
    reg signed [14:0] d;
    reg signed [10:0] ddd;
    reg [6:0]  fly_n;
    reg [16:0] e;
    // 输出寄存器（跨到 pix）
    reg        bq_on;
    reg [15:0] bq_u; reg [14:0] bq_v; reg [16:0] bq_s;
    assign bundle    = {29'd0, bq_s, bq_v, bq_u, 3'b000, bq_on};
    assign tour_busy = on || (st == T_GEO);

    // ---------------- 引擎 ----------------
    reg [7:0]  pc;
    reg [1:0]  ph;                            // 0 = 读指令，1 = 锁存指令 / 读操作数，2 = 执行
    reg        run, run_job;                  // 运行中 / 运行的是主状态机的作业（非每帧程序）
    reg [11:0] ir;
    reg signed [17:0] acc;
    reg        fpend;                         // 有一帧待提交
    reg        jreq;                          // 主状态机作业请求
    reg [7:0]  jaddr;
    // mem 声明和初值在上面 @@UROM 区（asm_seq.py 生成）
    reg [17:0] mq;
    // 拍 0：读微程序；拍 1：mq = 指令，按它读操作数（R = 工作变量 / C = 常数 {0001, a}，G = 几何 {001, bank, idx}）；
    //   拍 2 和空闲时不读（mq 保持操作数，DIV 等待期间 x 不变）
    wire [11:0] uw   = mq[11:0];
    wire [8:0]  ra   = !ph[0] ? {1'b1, pc} :
                       (uw[7:5] == 3'd1) ? {3'b001, bank_q, uw[3:0]} : {3'b000, uw[6], uw[4:0]};
    wire        m_re = run && !ph[1];
    // 几何写入（plan_valid 后 8 拍顺序写；与引擎 ST 冲突时让一拍）
    reg [3:0]  gval;
    reg [1:0]  gbank;
    reg [3:0]  gw;                            // 0 = 空闲，1..8 = 写一项（几何索引见 asm_seq.py GEO，6~9 空号）
    wire       e_we  = run && ph[1] && (ir[11:8] == O_ST);
    wire       g_we  = (gw != 4'd0) && !e_we;
    reg  [11:0] g_val;
    always @(*) begin
        case (gw)
            4'd1:  g_val = g_c0;
            4'd2:  g_val = g_r0t;
            4'd3:  g_val = g_cw;
            4'd4:  g_val = g_ch;
            4'd5:  g_val = g_n;
            4'd6:  g_val = {2'd0, g_d};
            4'd7:  g_val = {2'd0, g_ox};      // → 第 10 项
            default: g_val = {3'd0, g_oyt};   // → 第 11 项
        endcase
    end
    wire [3:0] g_idx = (gw > 4'd6) ? gw + 4'd3 : gw - 4'd1;
    always @(posedge clk) begin
        if (e_we)      mem[{4'b0000, ir[4:0]}] <= acc;
        else if (g_we) mem[{3'b001, gbank, g_idx}] <= {6'd0, g_val};
        if (m_re) mq <= mem[ra];
    end

    // 操作数
    reg  signed [17:0] ma, mb;
    reg  signed [35:0] prod;
    always @(posedge clk) prod <= ma * mb;
    reg  [17:0] x;
    always @(*) begin
        case (ir[7:5])
            3'd0, 3'd1, 3'd2: x = mq;
            3'd3: x = (ir[1:0] == 2'd0) ? {15'd0, ft_data[2:0]} : (ir[1:0] == 2'd1) ? {6'd0, ft_data[26:15]} :
                      (ir[1:0] == 2'd2) ? {6'd0, ft_data[14:3]} : {1'b0, e};
            3'd4: x = prod[33:16];
            default: x = prod[17:0];
        endcase
    end
    // （L11 试过 ADD 与 SUB 共用一个加减器：x 取反 + 进位输入反而 +11 LUT，按 M5 不做）
    wire signed [18:0] dif = {acc[17], acc} - {x[17], x};
    wire               lt  = dif[18];                        // acc < x
    // 除法器：被除数 = prod[29:0]（商 < 2^17），除数 16 位；17 拍恢复除法
    reg  [16:0] rem, dq;
    reg  [15:0] dv;
    reg  [4:0]  dcnt;
    reg         dbusy, dgo;
    wire [16:0] dt  = {rem[15:0], dq[16]};
    wire [17:0] ds  = {1'b0, dt} - {2'b00, dv};                // 比较和减法合并：不借位 = 够减
    wire        dge = !ds[17];

    // 主状态机的作业请求
    task job; input [7:0] a; begin jreq <= 1'b1; jaddr <= a; end endtask
    task go_home; begin
        going_home <= 1'b1; home_req <= 1'b0;
        job(PA_HOME);
        st <= T_CALC;
    end endtask

    wire abort    = on && ((cur_img != img_q) || (disp_bank != bank_q) || block || !enable);
    wire job_busy = jreq || (run && run_job);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= T_IDLE; cs <= 1'b0; k <= 2'd0; img_q <= 6'd0; bank_q <= 2'd0; fcnt <= 8'd0;
            going_home <= 1'b0; home_req <= 1'b0;
            on <= 1'b0;
            f <= 19'd0; d <= 15'sd0; ddd <= 11'sd0; fly_n <= 7'd0; e <= 17'd0;
            bq_on <= 1'b0;
            bq_u <= 16'd0; bq_v <= 15'd0; bq_s <= 17'd65536;
            ft_addr <= 8'd0;
            pc <= 8'd0; ph <= 2'd0; run <= 1'b0; run_job <= 1'b0; ir <= 12'd0; acc <= 18'sd0;
            fpend <= 1'b0; jreq <= 1'b0; jaddr <= 8'd0;
            ma <= 18'sd0; mb <= 18'sd0;
            rem <= 17'd0; dq <= 17'd0; dv <= 16'd0; dcnt <= 5'd0; dbusy <= 1'b0; dgo <= 1'b0;
            gval <= 4'd0; gbank <= 2'd0; gw <= 4'd0;
        end else begin
            // ================= 几何表写入 =================
            if (fb_load_start) begin
                gbank <= fb_load_bank;
                gval[fb_load_bank] <= 1'b0;
                gw <= 4'd0;
            end else if (plan_valid) begin
                gw <= 4'd1;
            end else if (g_we) begin
                if (gw == 4'd8) begin gval[gbank] <= 1'b1; gw <= 4'd0; end
                else gw <= gw + 4'd1;
            end

            // ================= 除法器 =================
            if (dbusy) begin
                rem  <= dge ? ds[16:0] : dt;
                dq   <= {dq[15:0], dge};
                dcnt <= dcnt + 5'd1;
                if (dcnt == 5'd16) dbusy <= 1'b0;
            end

            // ================= 引擎 =================
            if (frame_tick && (st != T_IDLE || bq_on || on)) fpend <= 1'b1;
            if (!run) begin
                if (fpend) begin                                    // 每帧程序优先（视角只在 START / FLY 里变）
                    fpend <= 1'b0;
                    pc <= (st == T_FLY || st == T_START) ? PA_FRAME : PA_FLAGS;
                    run <= 1'b1; run_job <= 1'b0; ph <= 2'd0;
                end else if (jreq) begin
                    jreq <= 1'b0;
                    pc <= jaddr;
                    run <= 1'b1; run_job <= 1'b1; ph <= 2'd0;
                end
            end else if (ph == 2'd0) begin
                ph <= 2'd1;
            end else if (ph == 2'd1) begin
                ir <= uw;
                ph <= 2'd2;
            end else begin
                ph <= 2'd0;
                pc <= pc + 8'd1;
                case (ir[11:8])
                    O_LD:  acc <= x;
                    O_ADD: acc <= acc + x;
                    O_SUB: acc <= dif[17:0];
                    O_MAX: if (lt)  acc <= x;
                    O_MIN: if (!lt) acc <= x;
                    O_MUL: begin ma <= acc; mb <= x; end
                    O_DIV: begin
                        if (!dgo) begin                             // 启动：被除数 = 上一条 MUL 的乘积
                            rem <= {4'd0, prod[29:17]}; dq <= prod[16:0]; dv <= x[15:0];
                            dcnt <= 5'd0; dbusy <= 1'b1; dgo <= 1'b1;
                            ph <= 2'd2; pc <= pc;
                        end else if (dbusy) begin
                            ph <= 2'd2; pc <= pc;
                        end else begin
                            acc <= {1'b0, dq}; dgo <= 1'b0;
                        end
                    end
                    O_BRN:  if (acc[17])  pc <= ir[7:0];
                    O_BRNN: if (!acc[17]) pc <= ir[7:0];
                    O_OUT: case (ir[2:0])
                        3'd0: bq_u   <= acc[15:0];
                        3'd1: bq_v   <= acc[14:0];
                        3'd2: bq_s   <= acc[16:0];
                        default: bq_on <= on;
                    endcase
                    O_END: run <= 1'b0;
                    default: ;                                      // O_ST：写口在上面
                endcase
            end

            // ================= 主状态机 =================
            if (chord && on && !going_home) home_req <= 1'b1;
            if (abort && st != T_IDLE) begin                        // 立即硬切回原显示
                on <= 1'b0;
                home_req <= 1'b0;
                st <= T_IDLE;
            end else case (st)
            T_IDLE: begin
                ft_addr <= {cur_img, 2'd0};                         // 预读当前图的第 0 个焦点：没有条目就不进入
                if (chord && enable && !block && !start_block && gval[disp_bank] && !run && !jreq &&
                    ft_data[2:0] != 3'd0) begin
                    img_q <= cur_img; bank_q <= disp_bank;
                    e <= 17'd0;
                    job(PA_GEO);
                    st <= T_GEO;
                end
            end
            T_GEO: if (!job_busy) begin                             // 几何预处理完，视角 = 整图
                on <= 1'b1;
                fcnt <= 8'd0; k <= 2'd0; going_home <= 1'b0; home_req <= 1'b0;
                st <= T_START;
            end
            T_START: begin
                if (frame_tick) begin
                    fcnt <= fcnt + 8'd1;
                    if (fcnt == START_FRAMES - 1) begin
                        if (home_req || chord) begin going_home <= 1'b1; st <= T_END; end
                        else begin ft_addr <= {img_q, 2'd0}; cs <= 1'b0; st <= T_FT; end
                    end
                end
            end
            T_FT: begin                                             // 焦点表同步读：cs = 1 时有效
                cs <= 1'b1;
                if (home_req) go_home;
                else if (cs) begin
                    if (ft_data[2:0] == 3'd0)                       // 没有更多焦点 → 回整图
                        go_home;
                    else begin
                        job(PA_CALC);
                        st <= T_CALC;
                    end
                end
            end
            T_CALC: if (!job_busy) begin                            // CALC / HOME 程序里已置好起点和增量
                f <= 19'd0; d <= 15'sd190; ddd <= 11'sd372; fly_n <= 7'd0; e <= 17'd0;
                st <= T_FLY;
            end
            T_FLY: begin
                if (!run && !fpend && fly_n == 7'd64) begin
                    if (going_home) st <= T_END;
                    else if (home_req) go_home;
                    else begin fcnt <= 8'd0; st <= T_DWELL; end
                end else if (home_req && !going_home && !run && !fpend) begin   // 导览中再按：从当前视角飞回整图
                    go_home;
                end else if (frame_tick && fly_n != 7'd64) begin
                    f   <= f + {{4{d[14]}}, d};
                    d   <= d + {{4{ddd[10]}}, ddd};
                    ddd <= ddd - 11'sd12;
                    e   <= (f + {{4{d[14]}}, d}) >> 2;
                    fly_n <= fly_n + 7'd1;
                end
            end
            T_DWELL: begin
                if (home_req) go_home;
                else if (tour_next || fcnt == DWELL_FRAMES) begin
                    if (k == 2'd3) go_home;
                    else begin
                        k <= k + 2'd1;
                        ft_addr <= {img_q, k + 2'd1};
                        cs <= 1'b0;
                        st <= T_FT;
                    end
                end else if (frame_tick)
                    fcnt <= fcnt + 8'd1;
            end
            default: begin                                          // T_END
                on <= 1'b0;
                st <= T_IDLE;
            end
            endcase
        end
    end
endmodule
