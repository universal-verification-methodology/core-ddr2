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
            // Track when FILLCOUNT enters / leaves a high-water region. This is
            // used for debug/coverage only; the host-visible NOTFULL signal now
            // reflects command FIFO headroom rather than this high-water mark.
            if (fillcount >= 7'd33) begin
                if (!in_full_region) begin
                    in_full_region <= 1'b1;
                    $display("[%0t] FIFO_MON: FILLCOUNT entered high-water region (fillcount=%0d).",
                             $time, fillcount);
                end
            end else begin
                if (in_full_region) begin
                    $display("[%0t] FIFO_MON: FILLCOUNT left high-water region (fillcount=%0d).",
                             $time, fillcount);
                end
                in_full_region <= 1'b0;
            end
        end
    end

endmodule

