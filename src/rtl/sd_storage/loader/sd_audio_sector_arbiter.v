`timescale 1ns/1ps

module sd_audio_sector_arbiter #(
    parameter [7:0] ERR_FLUSH = 8'h10
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        flush,
    input  wire        b_enable,
    // A：FAT32 客户端
    input  wire        a_req,
    input  wire [31:0] a_lba,
    output wire        a_busy,
    output wire        a_valid,
    output wire        a_done,
    // B：音频
    input  wire        b_req,
    input  wire [31:0] b_lba,
    output wire        b_busy,
    output wire        b_valid,
    output wire        b_done,
    output wire        b_flushed,       // 与 b_done 同拍：本次结束是 flush 补发的（错误码 ERR_FLUSH）
    // C：讲解（扩1）
    input  wire        c_req,
    input  wire [31:0] c_lba,
    output wire        c_busy,
    output wire        c_valid,
    output wire        c_done,
    output wire        c_flushed,
    // S：sd_sector_adapter
    output reg         s_req,
    output wire [31:0] s_lba,
    input  wire        s_busy,
    input  wire        s_valid,
    input  wire        s_done
);
    localparam [1:0] OWN_NONE = 2'd0;
    localparam [1:0] OWN_A    = 2'd1;
    localparam [1:0] OWN_B    = 2'd2;
    localparam [1:0] OWN_C    = 2'd3;

    reg [1:0] owner;
    reg       pend_a, pend_b, pend_c;
    reg [1:0] last;                   // 上一个扇区给的是谁（OWN_A/B/C）
    reg       flush_done;             // 单拍：B 口被 flush 收尾
    reg       flush_done_c;           // 单拍：C 口被 flush 收尾

    wire want_a  = pend_a;
    wire want_b  = pend_b && b_enable;
    wire want_c  = pend_c && b_enable;
    wire can_go  = (owner == OWN_NONE) && !s_req && !s_busy && !flush;
    // 轮转：从 last 的下一位开始找
    wire [1:0] pick = (last == OWN_A) ? (want_b ? OWN_B : want_c ? OWN_C : OWN_A) :
                      (last == OWN_B) ? (want_c ? OWN_C : want_a ? OWN_A : OWN_B) :
                                        (want_a ? OWN_A : want_b ? OWN_B : OWN_C);

    assign s_lba     = (owner == OWN_B) ? b_lba : (owner == OWN_C) ? c_lba : a_lba;
    assign a_busy    = pend_a || (owner == OWN_A);
    assign b_busy    = pend_b || (owner == OWN_B);
    assign c_busy    = pend_c || (owner == OWN_C);
    assign a_valid   = s_valid && (owner == OWN_A);
    assign a_done    = s_done  && (owner == OWN_A);
    assign b_valid   = s_valid && (owner == OWN_B);
    assign b_done    = (s_done && (owner == OWN_B)) || flush_done;
    assign b_flushed = flush_done;
    assign c_valid   = s_valid && (owner == OWN_C);
    assign c_done    = (s_done && (owner == OWN_C)) || flush_done_c;
    assign c_flushed = flush_done_c;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            owner      <= OWN_NONE;
            pend_a     <= 1'b0;
            pend_b     <= 1'b0;
            pend_c     <= 1'b0;
            last       <= OWN_C;
            s_req      <= 1'b0;
            flush_done <= 1'b0;
            flush_done_c <= 1'b0;
        end else begin
            s_req      <= 1'b0;
            flush_done <= 1'b0;
            flush_done_c <= 1'b0;

            if (a_req) pend_a <= 1'b1;
            if (b_req) pend_b <= 1'b1;
            if (c_req) pend_c <= 1'b1;

            if (flush) begin
                flush_done   <= (owner == OWN_B);       // 只有在途的音频扇区以错误结束
                flush_done_c <= (owner == OWN_C);
                if (owner == OWN_B) pend_b <= 1'b0;   // 未放行的音频请求保留，SD 恢复后照常读
                if (owner == OWN_C) pend_c <= 1'b0;
                owner      <= OWN_NONE;
                pend_a     <= 1'b0;                   // A 口客户端随软复位一起复位
            end else if (s_done && owner != OWN_NONE) begin
                if (owner == OWN_A) pend_a <= 1'b0;
                if (owner == OWN_B) pend_b <= 1'b0;
                if (owner == OWN_C) pend_c <= 1'b0;
                last   <= owner;
                owner  <= OWN_NONE;
            end else if (can_go && (want_a || want_b || want_c)) begin
                owner <= pick;
                s_req <= 1'b1;
            end
        end
    end
endmodule
