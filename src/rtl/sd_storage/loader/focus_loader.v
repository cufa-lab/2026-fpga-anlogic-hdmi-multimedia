`timescale 1ns/1ps

module focus_loader (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        enable,
    input  wire        aux_ok,               // 清单里有 FOCUS 文件
    // pcm_track_locator B 口（locator 项 0x31 = FOCUS）
    output reg         loc_req,
    output wire [5:0]  loc_idx,
    input  wire        loc_valid,
    input  wire [31:0] loc_lba,
    input  wire [31:0] loc_size,
    // 讲解 C 口
    output reg         sec_req,
    output wire [31:0] sec_lba,
    input  wire        sec_busy,
    input  wire        sec_valid,
    input  wire [7:0]  sec_byte,
    input  wire        sec_done,
    input  wire [7:0]  sec_error,
    output reg         done,
    // 焦点表读口（同步读，下一拍有效）
    input  wire [7:0]  rd_addr,
    output reg  [26:0] rd_data
);
    assign loc_idx = 6'h31;

    localparam [2:0] S_CLR = 3'd0, S_WAIT = 3'd1, S_LOC = 3'd2, S_RQ = 3'd3, S_WT = 3'd4, S_DONE = 3'd5;

    reg [2:0]  state;
    reg [7:0]  clr_a;
    reg [31:0] f_lba;
    reg        sec2;                         // 正在读第 2 个扇区
    assign sec_lba = f_lba + {31'd0, sec2};  // 只读 2 个扇区：不另设地址寄存器和三选一
    reg [9:0]  bcnt;                         // 已收字节（文件固定 1024 字节，正好 2 个扇区）
    reg [1:0]  retry;
    reg        fail;                         // 重读用完：清表后 done
    reg [23:0] sh;                           // 本项已收的低 3 字节

    // 表
    (* ram_style = "bram" *) reg [26:0] tab [0:255];
    reg        we;
    reg [7:0]  wa;
    reg [26:0] wd;
    always @(posedge clk) if (we) tab[wa] <= wd;
    // P3 低功耗：表只在加载期间（done 之前，再多一拍盖住最后一次写）被写，之后是静态的；
    // 读口改为"加载期间每拍读 / 之后只在地址变化时读"，输出与每拍都读逐拍相同，平时 BRAM 读口停止翻转
    reg        done_q;
    reg [7:0]  ra_q;
    wire       rd_en = !done_q || (rd_addr != ra_q);
    always @(posedge clk) begin
        done_q <= done;                                     // 只做读使能，复位后 done=0 一拍即同步，不接复位
        ra_q   <= rd_addr;
        if (rd_en) rd_data <= tab[rd_addr];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_CLR; clr_a <= 8'd0; done <= 1'b0;
            loc_req <= 1'b0; sec_req <= 1'b0; sec2 <= 1'b0;
            f_lba <= 32'd0; bcnt <= 10'd0; retry <= 2'd0; fail <= 1'b0;
            we <= 1'b0; wa <= 8'd0; wd <= 27'd0;
        end else begin
            we      <= 1'b0;
            loc_req <= 1'b0;
            sec_req <= 1'b0;

            // ---------------- 直拷：每 4 字节（小端）写一项 ----------------
            if (sec_valid && state == S_WT) begin
                bcnt <= bcnt + 10'd1;
                if (bcnt[1:0] == 2'd3) begin
                    we <= 1'b1; wa <= bcnt[9:2]; wd <= {sec_byte[2:0], sh};
                end
            end

            // ---------------- 主状态机 ----------------
            case (state)
            S_CLR: begin
                we <= 1'b1; wa <= clr_a; wd <= 27'd0;
                clr_a <= clr_a + 8'd1;
                if (clr_a == 8'd255) state <= fail ? S_DONE : S_WAIT;
            end
            S_WAIT: if (enable) begin
                if (aux_ok) begin loc_req <= 1'b1; state <= S_LOC; end
                else state <= S_DONE;
            end
            S_LOC: if (loc_valid) begin
                if (loc_size != 32'd1024) state <= S_DONE;
                else begin
                    f_lba   <= loc_lba;
                    sec2    <= 1'b0;
                    bcnt    <= 10'd0;
                    state   <= S_RQ;
                end
            end
            S_RQ: if (!sec_busy) begin
                sec_req <= 1'b1;
                state   <= S_WT;
            end
            S_WT: if (sec_done) begin
                if (sec_error != 8'h00) begin
                    if (retry == 2'd3) begin fail <= 1'b1; state <= S_CLR; end
                    else begin
                        retry   <= retry + 2'd1;
                        sec2    <= 1'b0;
                        bcnt    <= 10'd0;
                        state   <= S_RQ;
                    end
                end else if (bcnt[9]) begin                         // 第 1 个扇区收完（bcnt = 512）→ 读第 2 个；第 2 个收完 bcnt 回绕到 0
                    sec2    <= 1'b1;
                    state   <= S_RQ;
                end else
                    state <= S_DONE;
            end
            default: done <= 1'b1;                                  // S_DONE
            endcase
        end
    end

    // 低 3 字节移位暂存（纯数据，不接复位）
    always @(posedge clk)
        if (sec_valid && state == S_WT) sh <= {sec_byte, sh[23:8]};
endmodule
