`timescale 1ns/1ps

//BCH 改为逐样本计算。
//   原实现在 224 位 packet_body 寄存器上挂 4 份并行 BCH(64,56)。现在每收一个样本，
//   就用同一棵 BCH 树算出它所在子包的 ECC 存进 ecc0..ecc3；packet_body 不再整包寄存，
//   直接由 4 组样本寄存器 + 每槽的 C/P 位拼出（纯连线）。
//   packet_valid 期间端口上的 header/body/ECC/frame_base 与原版逐位相同
//   packet_valid=0 时 body/ECC 的值不再保持（下游只在 valid 期间读）。
module hdmi_audio_sample_packetizer (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               sample_valid,
    output wire               sample_ready,
    input  wire signed [15:0] sample_left,
    input  wire signed [15:0] sample_right,
    output reg                packet_valid,
    input  wire               packet_ready,
    output wire [23:0]        packet_header,
    output wire [7:0]         packet_header_ecc,
    output wire [223:0]       packet_body,
    output wire [31:0]        packet_body_ecc,
    output reg  [7:0]         packet_frame_base
);
    reg [1:0] collect_count;
    reg [7:0] next_audio_frame_base;
    reg first_block;                      // HB2.B0：本包首帧是 IEC 状态块第 0 帧
    reg signed [15:0] left0, left1, left2, left3;
    reg signed [15:0] right0, right1, right2, right3;
    reg [2:0] ctl0, ctl1, ctl2, ctl3;     // {右 P, C, 左 P}
    reg [7:0] ecc0, ecc1, ecc2, ecc3;

    assign sample_ready = !packet_valid;

    function [55:0] make_subpacket;
        input signed [15:0] left_sample;
        input signed [15:0] right_sample;
        input [2:0] ctl;
        begin
            // 16-bit LPCM is left-justified in the 24-bit IEC 60958 sample
            // word. Control nibble order is P/C/U/V for right then left.
            make_subpacket = {ctl[2], ctl[1], 2'b00, ctl[0], ctl[1], 2'b00,
                              right_sample, 8'h00, left_sample, 8'h00};
        end
    endfunction

    function [7:0] bch56;
        input [55:0] data;
        integer i;
        reg [7:0] r;
        begin
            // 与 hdmi_bch8 相同：发送顺序 data[0] 先，反射多项式 0x83
            r = 8'h00;
            for (i = 0; i < 56; i = i + 1)
                r = (r >> 1) ^ ((r[0] ^ data[i]) ? 8'h83 : 8'h00);
            bch56 = r;
        end
    endfunction

    // 当前样本所在槽的 C/P 位与 ECC（四个槽共用这一棵树）
    // C[25]=1 declares 48 kHz and C[33]=1 declares 16-bit; all other bits in
    // this minimal consumer status block are zero.
    wire [7:0] slot_frame = next_audio_frame_base + {6'd0, collect_count};
    wire       slot_cs    = (slot_frame == 8'd25) || (slot_frame == 8'd33);
    wire [2:0] slot_ctl   = {^{sample_right, slot_cs}, slot_cs, ^{sample_left, slot_cs}};
    wire [7:0] slot_ecc   = bch56(make_subpacket(sample_left, sample_right, slot_ctl));

    // Low byte is HB0. Layout=0 and all four subpackets are present.
    assign packet_header = {first_block ? 8'h10 : 8'h00, 8'h0f, 8'h02};
    hdmi_bch8 #(.DATA_BYTES(3)) u_header_bch (
        .data(packet_header), .ecc(packet_header_ecc)
    );
    assign packet_body = {make_subpacket(left3, right3, ctl3),
                          make_subpacket(left2, right2, ctl2),
                          make_subpacket(left1, right1, ctl1),
                          make_subpacket(left0, right0, ctl0)};
    assign packet_body_ecc = {ecc3, ecc2, ecc1, ecc0};

    // 控制/状态寄存器复位；样本/C/P/ECC 是纯数据，只在 valid 期间被读，不接复位
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            collect_count <= 2'd0;
            packet_valid <= 1'b0;
            packet_frame_base <= 8'd0;
            next_audio_frame_base <= 8'd0;
            first_block <= 1'b0;
        end else if (packet_valid) begin
            if (packet_ready)
                packet_valid <= 1'b0;
        end else if (sample_valid) begin
            collect_count <= collect_count + 1'b1;
            if (collect_count == 2'd3) begin
                first_block <= next_audio_frame_base == 8'd0;
                packet_frame_base <= next_audio_frame_base;
                packet_valid <= 1'b1;
                if (next_audio_frame_base == 8'd188)
                    next_audio_frame_base <= 8'd0;
                else
                    next_audio_frame_base <= next_audio_frame_base + 8'd4;
            end
        end
    end

    always @(posedge clk) begin
        if (!packet_valid && sample_valid) begin
            case (collect_count)
                2'd0: begin left0 <= sample_left; right0 <= sample_right; ctl0 <= slot_ctl; ecc0 <= slot_ecc; end
                2'd1: begin left1 <= sample_left; right1 <= sample_right; ctl1 <= slot_ctl; ecc1 <= slot_ecc; end
                2'd2: begin left2 <= sample_left; right2 <= sample_right; ctl2 <= slot_ctl; ecc2 <= slot_ecc; end
                default: begin left3 <= sample_left; right3 <= sample_right; ctl3 <= slot_ctl; ecc3 <= slot_ecc; end
            endcase
        end
    end
endmodule
