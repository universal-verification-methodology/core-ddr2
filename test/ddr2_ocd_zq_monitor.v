`timescale 1ns/1ps

/**
 * DDR2 OCD / ZQ calibration monitor.
 *
 * This monitor observes DDR2 load-mode (LM) and logical ZQ calibration
 * commands on the CA bus and checks:
 *   - That any configured EMRS1(OCD enter) command occurs at most once.
 *   - That EMRS1(OCD exit/default) does not occur before OCD enter.
 *   - That an optional logical ZQ calibration command does not occur before
 *     OCD exit/default (when EMRS1_OCD_EXIT_VAL is non-zero).
 *
 * Timing windows (tMRD, tOCD, tZQinit) are represented coarsely in controller
 * clock cycles via parameters so the same monitor can be reused across
 * multiple speed-grades and device configurations.
 */
module ddr2_ocd_zq_monitor #(
    parameter [12:0] EMRS1_OCD_ENTER_VAL = 13'h000,
    parameter [12:0] EMRS1_OCD_EXIT_VAL  = 13'h000,
    parameter [12:0] ZQCAL_CMD_A_VAL     = 13'h000,
    parameter [1:0]  ZQCAL_CMD_BA_VAL    = 2'b00,
    parameter integer TOCD_MIN_CYCLES    = 0,
    parameter integer TZQINIT_MIN_CYCLES = 0
) (
    input  wire       clk,
    input  wire       reset,
    input  wire       cke_pad,
    input  wire       csbar_pad,
    input  wire       rasbar_pad,
    input  wire       casbar_pad,
    input  wire       webar_pad,
    input  wire [1:0] ba_pad,
    input  wire [12:0] a_pad
);

    // Load mode (MRS / EMRSx) decode.
    wire lm_cmd   = (csbar_pad == 1'b0 &&
                     rasbar_pad == 1'b0 &&
                     casbar_pad == 1'b0 &&
                     webar_pad  == 1'b0);
    wire is_emrs1 = lm_cmd && (ba_pad == 2'b01);

    // OCD enter / exit patterns are device-dependent and supplied via params.
    wire ocd_enter = is_emrs1 && (a_pad == EMRS1_OCD_ENTER_VAL);
    wire ocd_exit  = is_emrs1 && (a_pad == EMRS1_OCD_EXIT_VAL);

    // Logical ZQ calibration command: CS# low, WE# low, RAS#/CAS# high and
    // BA/A matching the configured ZQCAL patterns.
    wire zq_cmd = (csbar_pad == 1'b0 &&
                   rasbar_pad == 1'b1 &&
                   casbar_pad == 1'b1 &&
                   webar_pad  == 1'b0 &&
                   ba_pad     == ZQCAL_CMD_BA_VAL &&
                   a_pad      == ZQCAL_CMD_A_VAL);

    reg seen_ocd_enter;
    reg seen_ocd_exit;
    reg seen_zq;
    integer since_ocd_enter;
    integer since_zq;

    always @(posedge clk) begin
        if (reset) begin
            seen_ocd_enter  <= 1'b0;
            seen_ocd_exit   <= 1'b0;
            seen_zq         <= 1'b0;
            since_ocd_enter <= 0;
            since_zq        <= 0;
        end else begin
            // Simple cycle counters since specific calibration points.
            if (seen_ocd_enter && !ocd_exit)
                since_ocd_enter <= since_ocd_enter + 1;

            if (seen_zq)
                since_zq <= since_zq + 1;

            // OCD enter: must occur at most once.
            if (ocd_enter) begin
                if (seen_ocd_enter) begin
                    $display("[%0t] ERROR: OCD enter EMRS1 observed more than once.", $time);
                    $fatal;
                end
                seen_ocd_enter  <= 1'b1;
                since_ocd_enter <= 0;
            end

            // OCD exit/default: must follow enter, and optionally satisfy a
            // coarse tOCD-style delay if configured.
            if (ocd_exit) begin
                if (!seen_ocd_enter) begin
                    $display("[%0t] ERROR: OCD exit EMRS1 observed before OCD enter.", $time);
                    $fatal;
                end
                if (TOCD_MIN_CYCLES != 0 && since_ocd_enter < TOCD_MIN_CYCLES) begin
                    $display("[%0t] ERROR: tOCD violated: %0d < %0d cycles.",
                             $time, since_ocd_enter, TOCD_MIN_CYCLES);
                    $fatal;
                end
                seen_ocd_exit <= 1'b1;
            end

            // Logical ZQ calibration command: must not occur before OCD exit
            // (if an exit encoding is configured).
            if (zq_cmd) begin
                if (!seen_ocd_exit && (EMRS1_OCD_EXIT_VAL != 13'h000)) begin
                    $display("[%0t] ERROR: ZQ calibration command observed before OCD exit/default.", $time);
                    $fatal;
                end
                seen_zq  <= 1'b1;
                since_zq <= 0;
            end
        end
    end

endmodule

