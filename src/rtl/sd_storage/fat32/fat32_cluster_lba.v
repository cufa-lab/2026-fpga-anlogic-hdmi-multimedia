`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// fat32_cluster_lba —— 簇号 → 扇区号：lba = cluster_base_lba + cluster << spc_shift
//   cluster_base_lba = data_start_lba - 2·spc 由扫描器在挂载时算好（L9，第二批优化：
//   原来每个实例在 start 时各算一次 cluster - 2，现在三处都省掉这个减法器）。
//   每簇扇区数一定是 2 的幂（BPB 校验过），乘法换成移位；移位按每拍 1 位顺序做（最多 7 拍），
//   只要一个 32 位寄存器 + 一个加法器，比 3 级桶形移位器省一半 LUT。换簇是低频操作，几拍延迟无所谓。
//   start 单拍 → busy → done 单拍；lba = cluster_base_lba + acc 组合输出，计算结束后 acc 不变，
//   lba 保持到下一次 start（不另设结果寄存器）。
// -----------------------------------------------------------------------------
module fat32_cluster_lba (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    input  wire [27:0] cluster,
    input  wire [7:0]  sectors_per_cluster,
    input  wire [31:0] cluster_base_lba,
    output wire        busy,
    output reg         done,
    output wire [31:0] lba
);
    reg        run;
    reg [7:0]  spc_left;      // 还要乘的倍数（右移到 1 为止）
    reg [31:0] acc;

    assign busy = run;
    assign lba  = cluster_base_lba + acc;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            run      <= 1'b0;
            done     <= 1'b0;
            spc_left <= 8'd0;
            acc      <= 32'd0;
        end else begin
            done <= 1'b0;
            if (start) begin
                run      <= 1'b1;
                acc      <= {4'd0, cluster};
                spc_left <= sectors_per_cluster;
            end else if (run) begin
                if (spc_left[7:1] == 7'd0) begin
                    done <= 1'b1;
                    run  <= 1'b0;
                end else begin
                    acc      <= {acc[30:0], 1'b0};
                    spc_left <= {1'b0, spc_left[7:1]};
                end
            end
        end
    end
endmodule
