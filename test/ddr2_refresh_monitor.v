`timescale 1ns/1ps

`include "ddr2_timing_config.vh"

/**
 * DDR2 AUTO REFRESH interval monitor.
 *
 * Observes the DDR2 command pins and identifies AUTO REFRESH commands
 * (CS# low, RAS# low, CAS# low, WE# high). Tracks the cycle interval
 * between successive refreshes and enforces a configurable minimum and
 * maximum interval in controller clock cycles.
 *
 * Only tracks refresh intervals after the controller is ready (ready_i
 * asserted). This prevents initialization refreshes from being flagged
 * as errors, since JEDEC requires two refreshes during initialization
 * that are spaced much closer together than normal operational refreshes.
 *
 * Parameters are intentionally loose defaults; tighten them to match
 * your specific frequency / tREFI budget.
 */
module ddr2_refresh_monitor #(
    parameter integer TREFI_MIN_CLK = 50000,  // minimum allowed interval
    parameter integer TREFI_MAX_CLK = 200000  // maximum allowed interval
) (
    input wire clk,
    input wire reset,
    input wire ready_i,      // Controller ready signal - only track refreshes after this is asserted
    input wire cke_pad,
    input wire csbar_pad,
    input wire rasbar_pad,
    input wire casbar_pad,
    input wire webar_pad
);

    // Resolve effective timing values from the central config if present;
    // otherwise fall back to the module parameters.
    localparam integer TREFI_MIN_CLK_EFF =
`ifdef DDR2_TIMING_TREFI_MIN_CLK
        `DDR2_TIMING_TREFI_MIN_CLK;
`else
        TREFI_MIN_CLK;
`endif
    localparam integer TREFI_MAX_CLK_EFF =
`ifdef DDR2_TIMING_TREFI_MAX_CLK
        `DDR2_TIMING_TREFI_MAX_CLK;
`else
        TREFI_MAX_CLK;
`endif

    // Cycle counter since last detected refresh.
    integer cycles_since_refresh;
    reg     have_seen_refresh;

    wire is_refresh_cmd;
    assign is_refresh_cmd =
        cke_pad &&                 // only when device is active
        !csbar_pad &&              // chip selected
        !rasbar_pad &&             // RAS# low
        !casbar_pad &&             // CAS# low
         webar_pad;                // WE# high  => AUTO REFRESH

    always @(posedge clk) begin
        if (reset) begin
            cycles_since_refresh <= 0;
            have_seen_refresh    <= 1'b0;
        end else begin
            // Reset tracking state when controller becomes not ready (e.g., during re-initialization)
            if (!ready_i) begin
                cycles_since_refresh <= 0;
                have_seen_refresh    <= 1'b0;
            end else begin
                // Only track refresh intervals after controller is ready.
                // Free-running counter between refreshes (saturates at a large value).
                if (cycles_since_refresh < TREFI_MAX_CLK_EFF * 4)
                    cycles_since_refresh <= cycles_since_refresh + 1;

                if (is_refresh_cmd) begin
                    // Only check intervals for refreshes that occur after initialization
                    // (we're already in the ready_i==1 branch, so this refresh is valid)
                    if (have_seen_refresh) begin
                        // Interval = cycles since last refresh including this cycle.
                        // We read cycles_since_refresh before the increment above takes
                        // effect (non-blocking), so add 1 to get the actual elapsed cycles.
                        $display("[%0t] REF_MON: AUTO REFRESH interval = %0d cycles.",
                                 $time, cycles_since_refresh + 1);
                        // Optional lower-bound check: only enforced if TREFI_MIN_CLK_EFF
                        // is non-zero. This lets us disable the "too frequent"
                        // constraint in simulation modes that intentionally pull
                        // refreshes closer together.
                        if (TREFI_MIN_CLK_EFF > 0) begin
                            if ((cycles_since_refresh + 1) < TREFI_MIN_CLK_EFF) begin
                                $display("[%0t] ERROR: REF_MON: Refresh interval too short (%0d < %0d cycles).",
                                         $time, cycles_since_refresh + 1, TREFI_MIN_CLK_EFF);
                                $fatal;
                            end
                        end
                        // Always enforce an upper bound so we still flag a stuck
                        // or starved refresh scheduler.
                        if ((cycles_since_refresh + 1) > TREFI_MAX_CLK_EFF) begin
                            $display("[%0t] ERROR: REF_MON: Refresh interval too long (%0d > %0d cycles).",
                                     $time, cycles_since_refresh + 1, TREFI_MAX_CLK_EFF);
                            $fatal;
                        end
                    end else begin
                        $display("[%0t] REF_MON: First AUTO REFRESH observed.", $time);
                    end

                    have_seen_refresh    <= 1'b1;
                    cycles_since_refresh <= 0;
                end
            end
        end
    end

endmodule

