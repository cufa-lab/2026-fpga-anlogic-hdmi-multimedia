`timescale 1ns/1ps

module pcm_track_locator (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        req,              // A：单拍
    input  wire [5:0]  idx,
    input  wire        req_b,            // B：单拍
    input  wire [5:0]  idx_b,
    input  wire        cat_free,
    output reg         cat_own,
    output reg  [8:0]  cat_raddr,
    input  wire [31:0] cat_rdata,
    output reg         info_valid,       // A：单拍；结果保持到下一次查询
    output reg         info_valid_b,     // B：单拍
    output reg  [31:0] start_lba,
    output reg  [31:0] total_sectors,
    output reg  [31:0] size_bytes
);
    localparam [2:0] L_IDLE = 3'd0;
    localparam [2:0] L_WAIT = 3'd1;      // 等读口
    localparam [2:0] L_CL   = 3'd2;      // 读首扇区 LBA（2 拍）
    localparam [2:0] L_SZ   = 3'd3;      // 读大小（2 拍）

    reg [2:0]  state;
    reg [5:0]  idx_q;
    reg        own_b;                    // 本次查询属于 B
    reg        pend_a, pend_b;
    reg [5:0]  pidx_a, pidx_b;
    reg        phase;                    // BRAM 地址寄存 + 同步读：每个字等 2 拍

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= L_IDLE;
            idx_q         <= 6'd0;
            own_b         <= 1'b0;
            pend_a        <= 1'b0;
            pend_b        <= 1'b0;
            pidx_a        <= 6'd0;
            pidx_b        <= 6'd0;
            phase         <= 1'b0;
            cat_own       <= 1'b0;
            cat_raddr     <= 9'd0;
            start_lba     <= 32'd0;
            total_sectors <= 32'd0;
            size_bytes    <= 32'd0;
            info_valid    <= 1'b0;
            info_valid_b  <= 1'b0;
        end else begin
            info_valid   <= 1'b0;
            info_valid_b <= 1'b0;
            if (req)   begin pend_a <= 1'b1; pidx_a <= idx;   end
            if (req_b) begin pend_b <= 1'b1; pidx_b <= idx_b; end
            case (state)
            L_IDLE: if (pend_a || pend_b) begin
                own_b <= !pend_a;
                idx_q <= pend_a ? pidx_a : pidx_b;
                if (pend_a) pend_a <= req; else pend_b <= req_b;   // 同拍的新请求不丢
                state <= L_WAIT;
            end
            L_WAIT: if (cat_free) begin
                cat_own   <= 1'b1;
                cat_raddr <= {2'b10, idx_q, 1'b0};
                phase     <= 1'b0;
                state     <= L_CL;
            end
            L_CL: begin
                phase <= ~phase;
                if (phase) begin
                    start_lba <= cat_rdata;
                    cat_raddr <= {cat_raddr[8:1], 1'b1};
                    state     <= L_SZ;
                end
            end
            L_SZ: begin
                phase <= ~phase;
                if (phase) begin
                    total_sectors <= {9'd0, cat_rdata[31:9]};
                    size_bytes    <= cat_rdata;
                    cat_own       <= 1'b0;
                    info_valid    <= !own_b;
                    info_valid_b  <= own_b;
                    state         <= L_IDLE;
                end
            end
            default: state <= L_IDLE;
            endcase
        end
    end

endmodule
