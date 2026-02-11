`timescale 1ns/1ps

/**
 * DDR2 front-end FIFO flow-control monitor.
 *
 * Checks that the controller's public `FILLCOUNT` / `NOTFULL` contract is
 * respected. In particular, when `FILLCOUNT` reaches or exceeds the documented
 * threshold (33 entries), `NOTFULL` must be deasserted so that the host
 * interface cannot enqueue additional commands.
 *
 * This is a pure checker module: it has no outputs other than `$display` /
 * `$fatal` messages on protocol violations.
 */
module ddr2_fifo_monitor (
    input wire       clk,
    input wire       reset,
    input wire [6:0] fillcount,
    input wire       notfull
);

    // Sticky flag so we only print the "entering full" message once per
    // full-condition excursion, but still treat every violation as fatal.
    reg in_full_region;

    always @(posedge clk) begin
        if (reset) begin
            in_full_region <= 1'b0;
        end else begin
            // Track when FILLCOUNT enters / leaves the full region.
            if (fillcount >= 7'd33) begin
                if (!in_full_region) begin
                    in_full_region <= 1'b1;
                    $display("[%0t] FIFO_MON: FILLCOUNT entered full region (fillcount=%0d).",
                             $time, fillcount);
                end
                // Core invariant: NOTFULL must be low when FILLCOUNT >= 33.
                if (notfull) begin
                    $display("[%0t] ERROR: FIFO_MON: NOTFULL should be 0 when FILLCOUNT=%0d (>=33).",
                             $time, fillcount);
                    $fatal;
                end
            end else begin
                if (in_full_region) begin
                    $display("[%0t] FIFO_MON: FILLCOUNT left full region (fillcount=%0d).",
                             $time, fillcount);
                end
                in_full_region <= 1'b0;
            end
        end
    end

endmodule

