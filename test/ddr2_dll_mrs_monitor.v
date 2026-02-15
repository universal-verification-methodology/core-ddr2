`timescale 1ns/1ps

/**
 * Runtime DLL MRS monitor.
 *
 * Observes the DDR2 command/address bus and the controller's DLL_BUSY /
 * DLL_MODE status to ensure that each runtime DLL mode change sequence
 * issues exactly one Mode Register Set (MRS) command with the expected
 * A[12:0] contents:
 *
 *   - When dll_mode_i==0 at request time: A should equal MRS_DLL_ON_VAL.
 *   - When dll_mode_i==1 at request time: A should equal MRS_DLL_OFF_VAL.
 *
 * Any mismatch or missing MRS while DLL_BUSY is asserted causes $fatal.
 */
module ddr2_dll_mrs_monitor #(
    parameter [12:0] MRS_DLL_ON_VAL  = 13'h0013,
    parameter [12:0] MRS_DLL_OFF_VAL = 13'h0013
) (
    input  wire        clk,
    input  wire        reset,
    input  wire        dll_busy_i,
    input  wire        dll_mode_i,
    input  wire        cke_pad,
    input  wire        csbar_pad_any,
    input  wire        rasbar_pad,
    input  wire        casbar_pad,
    input  wire        webar_pad,
    input  wire [1:0]  ba_pad,
    input  wire [12:0] a_pad
);

    // Track rising/falling edges of DLL busy.
    reg dll_busy_d;
    always @(posedge clk) begin
        if (reset)
            dll_busy_d <= 1'b0;
        else
            dll_busy_d <= dll_busy_i;
    end

    wire dll_busy_rise = dll_busy_i & ~dll_busy_d;
    wire dll_busy_fall = ~dll_busy_i & dll_busy_d;

    // Latch the requested DLL mode at the start of a sequence so that
    // subsequent MRS commands can be checked against the correct pattern.
    reg        dll_mode_latched;
    reg        in_window;
    reg        mrs_seen;

    // Simple decoder for LM/MRS commands.
    wire mrs_cmd =
        cke_pad &&
        (csbar_pad_any == 1'b0) &&
        (rasbar_pad == 1'b0) &&
        (casbar_pad == 1'b0) &&
        (webar_pad == 1'b0) &&
        (ba_pad == 2'b00);

    always @(posedge clk) begin
        if (reset) begin
            dll_mode_latched <= 1'b0;
            in_window        <= 1'b0;
            mrs_seen         <= 1'b0;
        end else begin
            // Start of a DLL reconfiguration window.
            if (dll_busy_rise) begin
                dll_mode_latched <= dll_mode_i;
                in_window        <= 1'b1;
                mrs_seen         <= 1'b0;
                $display("[%0t] DLL_MRS_MON: DLL_BUSY asserted (dll_mode=%0b).",
                         $time, dll_mode_i);
            end

            // Within an active DLL_BUSY window, look for exactly one MRS.
            if (in_window && !mrs_seen && mrs_cmd) begin
                mrs_seen <= 1'b1;
                if (!dll_mode_latched) begin
                    // Expect DLL-on encoding.
                    if (a_pad !== MRS_DLL_ON_VAL) begin
                        $display("[%0t] ERROR: DLL_MRS_MON expected MRS_DLL_ON_VAL=0x%03h but saw A=0x%03h.",
                                 $time, MRS_DLL_ON_VAL, a_pad);
                        $fatal;
                    end else begin
                        $display("[%0t] DLL_MRS_MON: Observed DLL-ON MRS A=0x%03h as expected.",
                                 $time, a_pad);
                    end
                end else begin
                    // Expect DLL-off encoding.
                    if (a_pad !== MRS_DLL_OFF_VAL) begin
                        $display("[%0t] ERROR: DLL_MRS_MON expected MRS_DLL_OFF_VAL=0x%03h but saw A=0x%03h.",
                                 $time, MRS_DLL_OFF_VAL, a_pad);
                        $fatal;
                    end else begin
                        $display("[%0t] DLL_MRS_MON: Observed DLL-OFF MRS A=0x%03h as expected.",
                                 $time, a_pad);
                    end
                end
            end

            // End of a DLL reconfiguration window.
            if (dll_busy_fall) begin
                if (!in_window) begin
                    $display("[%0t] ERROR: DLL_MRS_MON saw DLL_BUSY falling without a matching rise.",
                             $time);
                    $fatal;
                end
                if (!mrs_seen) begin
                    $display("[%0t] ERROR: DLL_MRS_MON: DLL_BUSY deasserted without any MRS observed.",
                             $time);
                    $fatal;
                end
                in_window <= 1'b0;
            end
        end
    end

endmodule

