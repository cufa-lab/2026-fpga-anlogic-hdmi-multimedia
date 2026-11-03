`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// script_glyph_bram32k —— 扩1 字形 RAM：8192 × 64，16 个 EG_PHY_BRAM32K（每块 4096×8；8 块按字节并排成一个 bank，2 个 bank）
//   地址 = {槽 slot[7:0], 行对 w[3:0], 半 h}（bank = 槽号最高位），256 个槽，一个槽 32 字 = 一个 32×32、每点 2 位灰度的字形：
//   字 {w, h} = 第 2w、2w+1 行的第 16h..16h+15 列，8 个字节（块 0..7）= 8 个 2×2 小块，
//   块 j 字节 = {p(2w,16h+2j), p(2w,16h+2j+1), p(2w+1,16h+2j), p(2w+1,16h+2j+1)}，每点 2 位（高位在前）。
//   这样 2× 显示（原生 32 点）取 1 点、1× 显示（16 点）取 1 字节求 2×2 平均，都只要一个 8 选 1。
//   ASCII 16×32 只用 h = 0 的字。
// 端口 A（sys）：sys_park = 0 时只写块 sys_blk（字库字节直写），= 1 时同时写块 6、7；
//   偶数块收 sys_data[15:8]，奇数块收 sys_data[7:0]（写单字节时两半给同一字节）。
//   读（sys_re，1 拍延迟）只看第 6、7 块：script_loader 解析时把字的字库序号 fidx 暂存在本槽第 0 字的这两块里。
// 端口 B（pix）：只读 64 位。
// -----------------------------------------------------------------------------
module script_glyph_bram32k (
    input  wire        sys_clk,
    input  wire        sys_we,
    input  wire        sys_park,
    input  wire [2:0]  sys_blk,
    input  wire        sys_re,
    input  wire [12:0] sys_addr,
    input  wire [15:0] sys_data,
    output wire [15:0] sys_q,
    input  wire        pix_clk,
    input  wire        pix_re,
    input  wire [12:0] pix_addr,
    output wire [63:0] pix_q
);
    wire [15:0] doa [0:15];
    wire [15:0] dob [0:15];
    wire [63:0] q0, q1;
    reg         sys_bank_q, pix_bank_q;
    always @(posedge sys_clk) if (sys_re) sys_bank_q <= sys_addr[12];
    always @(posedge pix_clk) if (pix_re) pix_bank_q <= pix_addr[12];
    genvar b;
    generate
        for (b = 0; b < 16; b = b + 1) begin : g_blk
            localparam integer BK = b / 8, BY = b % 8;
            wire sel_a = (sys_addr[12] == BK);
            wire we_b  = sys_we && sel_a && (sys_park ? (BY >= 6) : (sys_blk == BY));
            EG_PHY_BRAM32K #(.MODE("DP16K"), .DATA_WIDTH_A("8"), .DATA_WIDTH_B("8"),
                             .REGMODE_A("NOREG"), .REGMODE_B("NOREG")) u_blk (
                .doa(doa[b]), .dia({8'd0, (BY % 2) ? sys_data[7:0] : sys_data[15:8]}),
                .addra(sys_addr[11:1]), .bytea(sys_addr[0]), .bytewea(1'b0),
                .csa(we_b || (sys_re && sel_a)), .wea(we_b), .clka(sys_clk), .rsta(1'b0), .ocea(sys_re),
                .dob(dob[b]), .dib(16'd0), .addrb(pix_addr[11:1]), .byteb(pix_addr[0]), .byteweb(1'b0),
                .csb(pix_re && (pix_addr[12] == BK)), .web(1'b0), .clkb(pix_clk), .rstb(1'b0), .oceb(pix_re)
            );
            if (BK == 0) begin : g_q0
                assign q0[63 - 8 * BY -: 8] = dob[b][7:0];
            end else begin : g_q1
                assign q1[63 - 8 * BY -: 8] = dob[b][7:0];
            end
        end
    endgenerate
    assign pix_q = pix_bank_q ? q1 : q0;
    assign sys_q = sys_bank_q ? {doa[14][7:0], doa[15][7:0]} : {doa[6][7:0], doa[7][7:0]};
endmodule

// -----------------------------------------------------------------------------
// script_dedup_ram —— 扩1 去重 hash 表：fidx[11:0] → 槽号（4096×8，sys 单时钟，推断为 4 块 BRAM9K）
//   上电内容任意都不会出错（script_loader 命中后还要核对槽内暂存的 fidx）；仿真里清 0 免得 X 传播。
// -----------------------------------------------------------------------------
module script_dedup_ram (
    input  wire        clk,
    input  wire        we,
    input  wire        re,
    input  wire [11:0] addr,
    input  wire [7:0]  data,
    output reg  [7:0]  q
);
    (* ram_style = "bram" *) reg [7:0] mem [0:4095];
    // synthesis translate_off
    integer k;
    initial for (k = 0; k < 4096; k = k + 1) mem[k] = 8'd0;
    // synthesis translate_on
    always @(posedge clk)
        if (we)
            mem[addr] <= data;
    always @(posedge clk)
        if (re)
            q <= mem[addr];
endmodule
