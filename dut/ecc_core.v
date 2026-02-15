`timescale 1ns/1ps

/**
 * Generic ECC wrapper for DDR2 server controller.
 *
 * This module provides a stable encoder/decoder interface that can be backed
 * either by the existing SECDED(72,64) implementation (`ecc_secded`) or by a
 * future chipkill/x4-capable ECC engine. The goal is to allow the top-level
 * controller to swap ECC schemes without rewiring all call sites.
 *
 * NOTE: In this revision only SECDED mode is functionally implemented. The
 *       CHIPKILL mode is stubbed out to behave like a pass-through (no error
 *       detection/correction) and is intended purely as a placeholder for
 *       future work.
 */
module ecc_core #(
    parameter integer DATA_WIDTH   = 64,
    parameter integer SECDED_PARITY_BITS = 7,
    // ECC_MODE:
    //   0 = SECDED(72,64) using ecc_secded (current behavior)
    //   1 = Chipkill/x4-capable ECC (stubbed in this revision)
    parameter integer ECC_MODE     = 0
) (
    // Encoder inputs/outputs
    input  wire [DATA_WIDTH-1:0]  data_in,
    output wire [SECDED_PARITY_BITS:0] ecc_out,

    // Decoder inputs
    input  wire [DATA_WIDTH-1:0]  data_check,
    input  wire [SECDED_PARITY_BITS:0] ecc_check,

    // Decoder outputs
    output wire [DATA_WIDTH-1:0]  data_corrected,
    output wire                   single_err,
    output wire                   double_err,
    output wire [SECDED_PARITY_BITS:0] syndrome
);

    // ------------------------------------------------------------------------
    // SECDED(72,64) implementation (current, production path).
    // ------------------------------------------------------------------------
    generate
        if (ECC_MODE == 0) begin : gen_secded_core
            ecc_secded #(
                .DATA_WIDTH (DATA_WIDTH),
                .PARITY_BITS(SECDED_PARITY_BITS)
            ) u_secded (
                .data_in       (data_in),
                .ecc_out       (ecc_out),
                .data_check    (data_check),
                .ecc_check     (ecc_check),
                .data_corrected(data_corrected),
                .single_err    (single_err),
                .double_err    (double_err),
                .syndrome      (syndrome)
            );
        end else begin : gen_chipkill_stub
            // ----------------------------------------------------------------
            // Placeholder for a future chipkill/x4-capable ECC engine.
            //
            // For now this path behaves like a transparent pass-through:
            //   - No ECC bits are generated (ecc_out = 0).
            //   - No correction is applied (data_corrected = data_check).
            //   - No errors are reported (single_err = double_err = 0).
            //   - Syndrome is zero.
            //
            // This allows top-level code to be wired up under an
            // ECC_MODE!=0 configuration without affecting functional behavior
            // until a real chipkill implementation is introduced.
            // ----------------------------------------------------------------
            assign ecc_out        = { (SECDED_PARITY_BITS+1){1'b0} };
            assign data_corrected = data_check;
            assign single_err     = 1'b0;
            assign double_err     = 1'b0;
            assign syndrome       = { (SECDED_PARITY_BITS+1){1'b0} };
        end
    endgenerate

endmodule

