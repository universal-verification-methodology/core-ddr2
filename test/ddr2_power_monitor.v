`timescale 1ns/1ps

/**
 * DDR2 power-mode monitor.
 *
 * Checks basic protocol safety around CKE-driven low-power modes:
 *   - While CKE is low and at least one rank is selected (CS# low), only NOP
 *     commands are allowed on the DDR2 command bus.
 *
 * This is a coarse checker intended to complement the protocol engine's
 * internal self-refresh / power-down FSMs and the existing timing monitors.
 */
module ddr2_power_monitor (
    input  wire clk,
    input  wire reset,
    input  wire cke_pad,
    // Any-rank CS# (active low when any rank is selected).
    input  wire csbar_any,
    input  wire rasbar_pad,
    input  wire casbar_pad,
    input  wire webar_pad
);

    // Simple command decode helpers.
    wire cmd_is_nop = (csbar_any == 1'b1) ||
                      (csbar_any == 1'b0 &&
                       rasbar_pad == 1'b1 &&
                       casbar_pad == 1'b1 &&
                       webar_pad  == 1'b1);

    always @(posedge clk) begin
        if (!reset) begin
            // When CKE is low and at least one rank is selected, the command
            // bus must effectively be NOP. Any attempt to drive ACT/READ/WRITE/
            // PRECHARGE/REFRESH while CKE=0 is treated as a protocol error.
            if (cke_pad == 1'b0 && csbar_any == 1'b0 && !cmd_is_nop) begin
                $display("[%0t] ERROR: Non-NOP DDR2 command observed while CKE=0 (csbar=%b rasbar=%b casbar=%b webar=%b)",
                         $time, csbar_any, rasbar_pad, casbar_pad, webar_pad);
                $fatal;
            end
        end
    end

endmodule

