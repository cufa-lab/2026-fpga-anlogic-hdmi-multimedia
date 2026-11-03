`timescale 1ns/1ps
// -----------------------------------------------------------------------------
//   [M1] 4 种 {wav_active, force_arpeggio} 组合 x 随机 valid/ready/数据：输出 = 被选音源，
//        被选者 ready = out_ready，未选者 ready = 0（未选音源不会被"偷偷消费"）
//   [M2] 纯组合：选择切换同拍生效
//   [T1] audio_sample_tick：25MHz 下间隔只有 520/521 两种，100 万拍内 tick 数 = 1e6*48000/25e6 (±1)
// -----------------------------------------------------------------------------
module tb_audio_source_mux;
    reg clk = 0, rst_n = 0;
    always #20 clk = ~clk;

    reg wav_valid, wav_active, arp_valid, force_arp, out_ready;
    reg signed [15:0] wl, wr, al, ar;
    wire wav_ready, arp_ready, out_valid;
    wire signed [15:0] ol, orr;

    audio_source_mux dut (
        .clk(clk), .rst_n(rst_n),
        .wav_valid(wav_valid), .wav_ready(wav_ready), .wav_left(wl), .wav_right(wr), .wav_active(wav_active),
        .arp_valid(arp_valid), .arp_ready(arp_ready), .arp_left(al), .arp_right(ar),
        .force_arpeggio(force_arp),
        .out_valid(out_valid), .out_ready(out_ready), .out_left(ol), .out_right(orr)
    );

    wire tick;
    audio_sample_tick u_tick (.clk(clk), .rst_n(rst_n), .tick(tick));

    integer errors = 0, i, n_wav = 0, n_arp = 0;
    reg use_wav;
    initial begin
        for (i = 0; i < 4000; i = i + 1) begin
            {wav_active, force_arp, wav_valid, arp_valid, out_ready} = $random;
            wl = $random; wr = $random; al = $random; ar = $random;
            #1;
            use_wav = wav_active && !force_arp;
            if (use_wav) n_wav = n_wav + 1; else n_arp = n_arp + 1;
            if (out_valid !== (use_wav ? wav_valid : arp_valid) ||
                ol !== (use_wav ? wl : al) || orr !== (use_wav ? wr : ar) ||
                wav_ready !== (use_wav && out_ready) || arp_ready !== (!use_wav && out_ready)) begin
                errors = errors + 1;
                if (errors < 10) $display("FAIL [M1] act=%0d force=%0d wv=%0d av=%0d rdy=%0d -> ov=%0d wr=%0d ar=%0d",
                                          wav_active, force_arp, wav_valid, arp_valid, out_ready, out_valid, wav_ready, arp_ready);
            end
        end
        $display("INFO [M1] 4000 random vectors (wav selected %0d, arpeggio %0d)", n_wav, n_arp);
        // [M2] 同拍切换
        wav_valid = 1; arp_valid = 0; out_ready = 1; wav_active = 1; force_arp = 0; wl = 16'sd1234; al = -16'sd5;
        #1 if (!out_valid || ol !== 16'sd1234 || !wav_ready) begin errors = errors + 1; $display("FAIL [M2] wav"); end
        force_arp = 1;
        #1 if (out_valid || ol !== -16'sd5 || wav_ready || !arp_ready) begin errors = errors + 1; $display("FAIL [M2] force"); end
        force_arp = 0; wav_active = 0;
        #1 if (out_valid || wav_ready) begin errors = errors + 1; $display("FAIL [M2] inactive"); end
    end

    // [T1]
    integer cyc = 0, last = -1, n_tick = 0, gap_min = 1 << 30, gap_max = 0;
    always @(posedge clk) if (rst_n) begin
        cyc = cyc + 1;
        if (tick) begin
            n_tick = n_tick + 1;
            if (last >= 0) begin
                if (cyc - last < gap_min) gap_min = cyc - last;
                if (cyc - last > gap_max) gap_max = cyc - last;
            end
            last = cyc;
        end
    end
    initial begin
        #100 rst_n = 1;
        wait (cyc == 1000000);
        $display("INFO [T1] %0d ticks in 1e6 cycles (exp 1920), interval %0d..%0d", n_tick, gap_min, gap_max);
        if (n_tick < 1919 || n_tick > 1921) begin errors = errors + 1; $display("FAIL [T1] tick count"); end
        if (gap_min < 520 || gap_max > 521) begin errors = errors + 1; $display("FAIL [T1] tick interval"); end
        if (errors == 0) $display("PASS tb_audio_source_mux");
        else             $display("FAIL tb_audio_source_mux: %0d errors", errors);
        $finish;
    end
endmodule
