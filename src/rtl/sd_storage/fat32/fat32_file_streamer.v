`timescale 1ns/1ps

module fat32_file_streamer #(
    parameter integer MAX_VISITED_CLUSTERS = 8192,
    parameter integer SECTOR_RETRIES       = 2,
    parameter integer SKIP_EN              = 0
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        file_start,
    input  wire [31:0] first_cluster,
    input  wire [31:0] file_size,
    input  wire [7:0]  sectors_per_cluster,
    input  wire [31:0] fat_start_lba,
    input  wire [31:0] cluster_base_lba,
    input  wire [31:0] max_cluster,
    output wire        file_busy,
    output reg         file_done,
    output reg         file_valid,
    output reg  [7:0]  file_byte,
    output reg  [31:0] file_byte_offset,
    output reg  [7:0]  fs_error,
    output reg         sector_req,
    output reg  [31:0] sector_lba,
    input  wire        sector_busy,
    input  wire        sector_valid,
    input  wire [7:0]  sector_byte,
    input  wire [8:0]  sector_byte_index,
    input  wire        sector_done,
    input  wire [7:0]  sector_error,
    input  wire [31:0] skip_until          // SKIP_EN=1：偏移 < skip_until 的字节下游不要
);
    localparam [7:0] ERR_NONE              = 8'h00;
    localparam [7:0] ERR_SECTOR            = 8'h10;
    localparam [7:0] ERR_FAT_LOOP          = 8'h23;
    localparam [7:0] ERR_CLUSTER_RANGE     = 8'h24;
    localparam [7:0] ERR_EARLY_EOF         = 8'h25;
    localparam [7:0] ERR_VISITED_EXHAUSTED = 8'h26;
    localparam [7:0] ERR_BAD_CLUSTER       = 8'h27;

    localparam [2:0] ST_IDLE      = 3'd0;
    localparam [2:0] ST_C2L       = 3'd1;     // 等簇→扇区换算
    localparam [2:0] ST_DATA_REQ  = 3'd2;
    localparam [2:0] ST_DATA_WAIT = 3'd3;
    localparam [2:0] ST_FAT_REQ   = 3'd4;
    localparam [2:0] ST_FAT_WAIT  = 3'd5;
    localparam [2:0] ST_FAT_PARSE = 3'd6;
    localparam [2:0] ST_FINISH    = 3'd7;

    localparam integer CHAIN_COUNT_WIDTH = $clog2(MAX_VISITED_CLUSTERS + 1);

    reg [2:0]  state;
    reg [27:0] current_cluster;
    reg [31:0] byte_count;            // 已输出字节数（= 下一个字节的偏移）；file_size 在读文件期间必须保持不变
    reg [7:0]  sector_in_cluster;
    reg [31:0] fat_word;              // FAT 表项（边收边拼，小端）
    reg [CHAIN_COUNT_WIDTH-1:0] chain_count;
    reg        final_chain_check;
    reg [9:0]  emitted;               // 本扇区已输出字节数（重读时跳过）
    reg [2:0]  sector_retry_count;
    reg        c2l_start;
    reg        skip_pend;             // SKIP_EN：刚跳过一个扇区，在 DATA_WAIT 当作"读完"处理

    wire        c2l_done;
    wire [31:0] c2l_lba;
    fat32_cluster_lba u_c2l (
        .clk(clk), .rst_n(rst_n), .start(c2l_start), .cluster(current_cluster),
        .sectors_per_cluster(sectors_per_cluster), .cluster_base_lba(cluster_base_lba),
        .busy(), .done(c2l_done), .lba(c2l_lba)
    );

    wire [27:0] fat_next  = fat_word[27:0];
    wire        fat_eof   = (fat_next >= 28'hFFFFFF8);
    wire        fat_bad   = (fat_next == 28'hFFFFFF7);
    wire        fat_range = (fat_next < 28'd2) || ({4'd0, fat_next} > max_cluster);
    // 整扇区 [byte_count, byte_count+512) 都在 skip_until 之前（byte_count 低 9 位此时为 0，只比高位）
    wire        skip_now  = (SKIP_EN != 0) && (emitted == 10'd0) && (byte_count[31:9] < skip_until[31:9]);
    wire        emit      = (state == ST_DATA_WAIT) && sector_valid && (byte_count != file_size) &&
                            ({1'b0, sector_byte_index} >= emitted);

    assign file_busy = (state != ST_IDLE);

    task finish_with_error;
        input [7:0] code;
        begin fs_error <= code; state <= ST_FINISH; end
    endtask

    task go_fat;
        begin
            sector_lba <= fat_start_lba + {11'd0, current_cluster[27:7]};
            state      <= ST_FAT_REQ;
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            file_done <= 1'b0;
            file_valid <= 1'b0;
            file_byte <= 8'd0;
            file_byte_offset <= 32'd0;
            fs_error <= ERR_NONE;
            sector_req <= 1'b0;
            sector_lba <= 32'd0;
            current_cluster <= 28'd0;
            byte_count <= 32'd0;
            sector_in_cluster <= 8'd0;
            fat_word <= 32'd0;
            chain_count <= 0;
            final_chain_check <= 1'b0;
            emitted <= 10'd0;
            sector_retry_count <= 3'd0;
            c2l_start <= 1'b0;
            skip_pend <= 1'b0;
        end else begin
            file_done  <= 1'b0;
            file_valid <= 1'b0;
            sector_req <= 1'b0;
            c2l_start  <= 1'b0;

            if (emit) begin
                file_valid       <= 1'b1;
                file_byte        <= sector_byte;
                file_byte_offset <= byte_count;
                byte_count       <= byte_count + 32'd1;
                emitted          <= {1'b0, sector_byte_index} + 10'd1;
            end

            // FAT 表项 4 字节按偏移 (c&0x7F)*4 现场拼
            if (state == ST_FAT_WAIT && sector_valid &&
                sector_byte_index[8:2] == current_cluster[6:0])
                fat_word <= {sector_byte, fat_word[31:8]};

            case (state)
                ST_IDLE: if (file_start) begin
                    fs_error <= ERR_NONE;
                    file_byte_offset <= 32'd0;
                    byte_count <= 32'd0;
                    sector_in_cluster <= 8'd0;
                    chain_count <= 1;
                    final_chain_check <= 1'b0;
                    emitted <= 10'd0;
                    sector_retry_count <= 3'd0;
                    current_cluster <= first_cluster[27:0];
                    if (file_size == 0) begin
                        state <= ST_FINISH;
                    end else if (first_cluster < 2 || first_cluster > max_cluster ||
                                 sectors_per_cluster == 0) begin
                        finish_with_error(ERR_CLUSTER_RANGE);
                    end else begin
                        c2l_start <= 1'b1;
                        state <= ST_C2L;
                    end
                end
                ST_C2L: if (c2l_done) begin
                    sector_lba <= c2l_lba;
                    state <= ST_DATA_REQ;
                end
                ST_DATA_REQ: if (skip_now) begin
                    byte_count <= byte_count + 32'd512;   // 不读卡：当作这个扇区已读完
                    skip_pend  <= 1'b1;
                    state      <= ST_DATA_WAIT;
                end else if (!sector_busy) begin
                    sector_req <= 1'b1;
                    state <= ST_DATA_WAIT;
                end
                ST_DATA_WAIT: if (sector_done || skip_pend) begin
                    skip_pend <= 1'b0;
                    if (sector_error != 0 && !skip_pend) begin
                        if (sector_retry_count < SECTOR_RETRIES) begin
                            sector_retry_count <= sector_retry_count + 3'd1;
                            state <= ST_DATA_REQ;          // emitted 保留：重读时跳过已输出部分
                        end else
                            finish_with_error(ERR_SECTOR);
                    end else begin
                        sector_retry_count <= 3'd0;
                        emitted <= 10'd0;
                        if (byte_count == file_size) begin          // 适配层保证 valid 与 done 不同拍
                            final_chain_check <= 1'b1;
                            go_fat;
                        end else if (sector_in_cluster + 1'b1 < sectors_per_cluster) begin
                            sector_in_cluster <= sector_in_cluster + 1'b1;
                            sector_lba <= sector_lba + 1'b1;
                            state <= ST_DATA_REQ;
                        end else begin
                            sector_in_cluster <= 8'd0;
                            go_fat;
                        end
                    end
                end
                ST_FAT_REQ: if (!sector_busy) begin
                    fat_word <= 32'd0;
                    sector_req <= 1'b1;
                    state <= ST_FAT_WAIT;
                end
                ST_FAT_WAIT: if (sector_done) begin
                    if (sector_error != 0) begin
                        if (sector_retry_count < SECTOR_RETRIES) begin
                            sector_retry_count <= sector_retry_count + 3'd1;
                            state <= ST_FAT_REQ;
                        end else
                            finish_with_error(ERR_SECTOR);
                    end else begin
                        sector_retry_count <= 3'd0;
                        state <= ST_FAT_PARSE;
                    end
                end
                ST_FAT_PARSE: begin
                    if (final_chain_check) begin
                        final_chain_check <= 1'b0;
                        if (fat_bad)        finish_with_error(ERR_BAD_CLUSTER);
                        else if (fat_eof)   state <= ST_FINISH;
                        else if (fat_range) finish_with_error(ERR_CLUSTER_RANGE);
                        else                finish_with_error(ERR_FAT_LOOP);
                    end else if (fat_bad)   finish_with_error(ERR_BAD_CLUSTER);
                    else if (fat_eof)       finish_with_error(ERR_EARLY_EOF);
                    else if (fat_range)     finish_with_error(ERR_CLUSTER_RANGE);
                    else if (chain_count >= MAX_VISITED_CLUSTERS)
                                            finish_with_error(ERR_VISITED_EXHAUSTED);
                    else if (fat_next == current_cluster)
                                            finish_with_error(ERR_FAT_LOOP);
                    else begin
                        chain_count <= chain_count + 1'b1;
                        current_cluster <= fat_next;
                        c2l_start <= 1'b1;
                        state <= ST_C2L;
                    end
                end
                ST_FINISH: begin
                    file_done <= 1'b1;
                    state <= ST_IDLE;
                end
                default: state <= ST_IDLE;
            endcase
        end
    end
endmodule
