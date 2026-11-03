`timescale 1ns/1ps

// 认领数据岛时只锁存包来源号 active_source（0=ACR 1=InfoFrame 2=音频），
//   不再把 288 位整包拷进 active_*。包调度器在认领时已锁定来源，被锁定的包生成器在
//   packet_ready（载荷最后一拍）之前保持包内容不变，所以 mapper 直接读来源的包内容，
//   发出的符号与原版逐位相同。原来"包内容与拷贝不一致"的自检随之删除（顶层本就悬空）。
module hdmi_data_island_scheduler #(
    parameter integer MIN_CONTROL_CHARS = 12
) (
    input  wire         clk,
    input  wire         rst_n,
    input  wire         video_active,
    input  wire         video_preamble,
    input  wire         video_guard,
    input  wire         island_slot,
    input  wire         hsync,
    input  wire         vsync,
    input  wire         packet_valid,
    output wire         packet_ready,
    output wire         packet_claim,
    input  wire [1:0]   packet_source,
    output reg  [2:0]   mode,
    output reg  [1:0]   channel0_control,
    output reg  [1:0]   channel1_control,
    output reg  [1:0]   channel2_control,
    output reg  [4:0]   packet_pixel_index,
    output reg  [1:0]   active_source,
    output wire         sequence_active,
    output reg          sequence_error
);
    localparam [2:0] MODE_CONTROL     = 3'd0;
    localparam [2:0] MODE_VIDEO       = 3'd1;
    localparam [2:0] MODE_VIDEO_GUARD = 3'd2;
    localparam [2:0] MODE_DATA_ISLAND = 3'd3;
    localparam [2:0] MODE_DATA_GUARD  = 3'd4;

    localparam [2:0] STATE_IDLE     = 3'd0;
    localparam [2:0] STATE_PREAMBLE = 3'd1;
    localparam [2:0] STATE_LEADING  = 3'd2;
    localparam [2:0] STATE_PAYLOAD  = 3'd3;
    localparam [2:0] STATE_TRAILING = 3'd4;

    reg [2:0] state;
    reg [5:0] state_count;
    reg [5:0] control_count;

    assign sequence_active = state != STATE_IDLE;
    assign packet_ready = state == STATE_PAYLOAD && state_count == 6'd31;
    assign packet_claim = state == STATE_IDLE && island_slot && packet_valid &&
                          control_count >= MIN_CONTROL_CHARS;

    always @* begin
        channel0_control = {vsync, hsync};
        channel1_control = {1'b0, video_preamble};
        channel2_control = 2'b00;
        packet_pixel_index = state_count[4:0];

        case (state)
            STATE_PREAMBLE: begin
                mode = MODE_CONTROL;
                channel1_control = 2'b01;
                channel2_control = 2'b01;
            end
            STATE_LEADING, STATE_TRAILING: begin
                mode = MODE_DATA_GUARD;
            end
            STATE_PAYLOAD: begin
                mode = MODE_DATA_ISLAND;
            end
            default: begin
                if (video_guard)
                    mode = MODE_VIDEO_GUARD;
                else if (video_active)
                    mode = MODE_VIDEO;
                else
                    mode = MODE_CONTROL;
            end
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= STATE_IDLE;
            state_count <= 6'd0;
            control_count <= 6'd0;
            active_source <= 2'd0;
            sequence_error <= 1'b0;
        end else begin
            case (state)
                STATE_IDLE: begin
                    state_count <= 6'd0;
                    if (video_active || video_guard) begin
                        control_count <= 6'd0;
                    end else if (control_count != 6'h3f) begin
                        control_count <= control_count + 1'b1;
                    end

                    if (island_slot) begin
                        if (packet_valid &&
                            control_count >= MIN_CONTROL_CHARS) begin
                            active_source <= packet_source;
                            state <= STATE_PREAMBLE;
                            state_count <= 6'd0;
                        end else if (packet_valid) begin
                            sequence_error <= 1'b1;
                        end
                    end
                end

                STATE_PREAMBLE: begin
                    if (state_count == 6'd7) begin
                        state <= STATE_LEADING;
                        state_count <= 6'd0;
                    end else begin
                        state_count <= state_count + 1'b1;
                    end
                end

                STATE_LEADING: begin
                    if (state_count == 6'd1) begin
                        state <= STATE_PAYLOAD;
                        state_count <= 6'd0;
                    end else begin
                        state_count <= state_count + 1'b1;
                    end
                end

                STATE_PAYLOAD: begin
                    if (state_count == 6'd31) begin
                        state <= STATE_TRAILING;
                        state_count <= 6'd0;
                    end else begin
                        state_count <= state_count + 1'b1;
                    end
                end

                STATE_TRAILING: begin
                    if (state_count == 6'd1) begin
                        state <= STATE_IDLE;
                        state_count <= 6'd0;
                        control_count <= 6'd0;
                    end else begin
                        state_count <= state_count + 1'b1;
                    end
                end

                default: begin
                    state <= STATE_IDLE;
                    state_count <= 6'd0;
                    control_count <= 6'd0;
                    sequence_error <= 1'b1;
                end
            endcase

            if (state != STATE_IDLE && state != STATE_TRAILING) begin
                if (!packet_valid || packet_source != active_source)
                    sequence_error <= 1'b1;
            end

            if (state != STATE_IDLE && (video_active || video_guard))
                sequence_error <= 1'b1;
        end
    end
endmodule
