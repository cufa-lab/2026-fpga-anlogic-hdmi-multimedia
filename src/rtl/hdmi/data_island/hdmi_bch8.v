`timescale 1ns/1ps

module hdmi_bch8 #(
    parameter integer DATA_BYTES = 3
) (
    input  wire [DATA_BYTES*8-1:0] data,
    output reg  [7:0]              ecc
);
    integer bit_index;
    reg [7:0] remainder;

    // HDMI 1.4 5.2.3.4: BCH(32,24)/BCH(64,56), G(x)=1+x^6+x^7+x^8.
    // The ECC is computed over the bits in transmission order (data[0]
    // first, i.e. LSB-first within each byte), so it is a right-shifting
    // LFSR with the reflected polynomial 0x83.
    // Fix vs course code: the original used MSB-first/left shift/0xC1, which
    // disagreed with the official Anlogic transmitter on 570/570 headers
    // (golden ModelSim check, see doc/).
    always @* begin
        remainder = 8'h00;
        for (bit_index = 0; bit_index < DATA_BYTES*8;
             bit_index = bit_index + 1)
            remainder = (remainder >> 1) ^
                        ((remainder[0] ^ data[bit_index]) ? 8'h83 : 8'h00);
        ecc = remainder;
    end
endmodule
