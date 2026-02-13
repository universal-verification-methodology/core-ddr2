`timescale 1ns/1ps

/**
 * Generic SECDED (Single Error Correction, Double Error Detection) ECC
 * encoder/decoder.
 *
 * NOTE: The current implementation is tuned for DATA_WIDTH=64 and
 * PARITY_BITS=7, matching a classic SECDED(72,64) code. Other parameter
 * combinations are not validated in this design.
 */
module ecc_secded #(
    parameter integer DATA_WIDTH  = 64,
    parameter integer PARITY_BITS = 7   // Number of Hamming parity bits
) (
    // Encoder inputs/outputs
    input  wire [DATA_WIDTH-1:0]  data_in,
    output reg  [PARITY_BITS:0]   ecc_out,

    // Decoder inputs
    input  wire [DATA_WIDTH-1:0]  data_check,
    input  wire [PARITY_BITS:0]   ecc_check,

    // Decoder outputs
    output reg  [DATA_WIDTH-1:0]  data_corrected,
    output reg                    single_err,
    output reg                    double_err,
    output reg  [PARITY_BITS:0]   syndrome
);

    // Internal signals for parity calculations (fixed for 64-bit data).
    wire [6:0] parity_bits;
    wire       overall_parity;

    // Hamming code parity bit positions (1-indexed) for 64 data bits:
    // Bit 1: covers positions with bit 0 set (1,3,5,7,...,63)
    // Bit 2: covers positions with bit 1 set (2,3,6,7,...,62,63)
    // Bit 4: covers positions with bit 2 set (4,5,6,7,12,13,14,15,...)
    // Bit 8: covers positions with bit 3 set (8-15,24-31,40-47,56-63)
    // Bit 16: covers positions with bit 4 set (16-31,48-63)
    // Bit 32: covers positions with bit 5 set (32-63)
    // Bit 64: covers positions with bit 6 set (64+) – not used for 64-bit data.

    // Calculate Hamming parity bits (covering all data bits).
    // Bit 0: covers positions where bit 0 of position is set (1,3,5,7,...,63)
    assign parity_bits[0] = ^(data_in & 64'hAAAAAAAAAAAAAAAA);

    // Bit 1: covers positions where bit 1 of position is set (2,3,6,7,...,62,63)
    assign parity_bits[1] = ^(data_in & 64'hCCCCCCCCCCCCCCCC);

    // Bit 2: covers positions where bit 2 of position is set (4,5,6,7,12,13,14,15,...)
    assign parity_bits[2] = ^(data_in & 64'hF0F0F0F0F0F0F0F0);

    // Bit 3: covers positions where bit 3 of position is set (8-15, 24-31, 40-47, 56-63)
    assign parity_bits[3] = ^(data_in & 64'hFF00FF00FF00FF00);

    // Bit 4: covers positions where bit 4 of position is set (16-31, 48-63)
    assign parity_bits[4] = ^(data_in & 64'hFFFF0000FFFF0000);

    // Bit 5: covers positions where bit 5 of position is set (32-63)
    assign parity_bits[5] = ^(data_in & 64'hFFFFFFFF00000000);

    // Bit 6: covers all data bits (for extended parity)
    assign parity_bits[6] = ^data_in;

    // Overall parity: XOR of all 64 data bits + 7 ECC bits.
    assign overall_parity = ^data_in ^ ^parity_bits;

    // Encoder: generate ECC code.
    always @* begin
        ecc_out[6:0] = parity_bits;
        ecc_out[7]   = overall_parity;
    end

    // ------------------------------------------------------------------------
    // Decoder: check and correct errors (logic mirrors the original ecc_64_8).
    // ------------------------------------------------------------------------
    wire [6:0] syndrome_bits;
    wire       syndrome_parity;
    wire [6:0] error_position;
    wire       single_bit_error;
    wire       parity_error;

    // Calculate syndrome (XOR of received data parity with received ECC).
    wire [6:0] computed_parity;
    assign computed_parity[0] = ^(data_check & 64'hAAAAAAAAAAAAAAAA);
    assign computed_parity[1] = ^(data_check & 64'hCCCCCCCCCCCCCCCC);
    assign computed_parity[2] = ^(data_check & 64'hF0F0F0F0F0F0F0F0);
    assign computed_parity[3] = ^(data_check & 64'hFF00FF00FF00FF00);
    assign computed_parity[4] = ^(data_check & 64'hFFFF0000FFFF0000);
    assign computed_parity[5] = ^(data_check & 64'hFFFFFFFF00000000);
    assign computed_parity[6] = ^data_check;

    assign syndrome_bits = computed_parity ^ ecc_check[6:0];

    // Overall parity check (XOR of all data + ECC parity bits).
    assign syndrome_parity = (^data_check ^ ^ecc_check[6:0]) ^ ecc_check[7];

    // Error position (1-indexed, 0 means no error).
    assign error_position   = syndrome_bits;
    assign single_bit_error = (syndrome_bits != 7'b0) && (syndrome_bits <= 7'd64);
    assign parity_error     = syndrome_parity;

    // Decode error type.
    always @* begin
        syndrome = {syndrome_parity, syndrome_bits};

        if (syndrome_bits == 7'b0 && !syndrome_parity) begin
            // No error.
            single_err     = 1'b0;
            double_err     = 1'b0;
            data_corrected = data_check;
        end else if (single_bit_error && parity_error) begin
            // Single-bit error: correct it.
            single_err = 1'b1;
            double_err = 1'b0;
            // Flip the bit at error_position (1-indexed, so subtract 1 for 0-indexed).
            if (error_position > 7'd0 && error_position <= 7'd64) begin
                data_corrected = data_check ^ (64'd1 << (error_position - 7'd1));
            end else begin
                data_corrected = data_check;
            end
        end else begin
            // Double-bit error or ECC bit error: uncorrectable.
            single_err     = 1'b0;
            double_err     = 1'b1;
            data_corrected = data_check; // Return original data (uncorrectable).
        end
    end

endmodule

