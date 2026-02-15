`timescale 1ns/1ps

/**
 * DDR2 ODT behavior monitor.
 *
 * Checks that:
 *   - ODT is asserted at least once within a small window after each WRITE
 *     command, indicating dynamic ODT during writes.
 *   - ODT is deasserted during READ commands.
 *
 * This is a coarse functional check rather than a cycle-accurate JEDEC
 * tAON/tAOF validator, but it ensures that the controller actually toggles
 * ODT in a directionally correct way.
 */
module ddr2_odt_monitor (
    input  wire clk,
    input  wire reset,
    input  wire cke_pad,
    input  wire csbar_pad_any,
    input  wire rasbar_pad,
    input  wire casbar_pad,
    input  wire webar_pad,
    input  wire odt_pad
);

    // Simple command decodes when any rank is selected.
    wire cmd_active = cke_pad && (csbar_pad_any == 1'b0);
    wire is_read  = cmd_active && rasbar_pad && !casbar_pad && webar_pad;
    wire is_write = cmd_active && rasbar_pad && !casbar_pad && !webar_pad;

    // Track a short window after each WRITE during which we expect to see
    // ODT asserted at least once.
    reg       write_window_active;
    reg [5:0] write_window_cnt;
    reg       odt_seen_in_window;

    always @(posedge clk) begin
        if (reset) begin
            write_window_active <= 1'b0;
            write_window_cnt    <= 6'd0;
            odt_seen_in_window  <= 1'b0;
        end else begin
            // Start a new window on each WRITE command.
            if (is_write) begin
                write_window_active <= 1'b1;
                write_window_cnt    <= 6'd31;  // ~32 cycles window
                odt_seen_in_window  <= odt_pad;
            end else if (write_window_active) begin
                // Track whether ODT ever asserts while the window is open.
                if (odt_pad)
                    odt_seen_in_window <= 1'b1;

                if (write_window_cnt != 6'd0)
                    write_window_cnt <= write_window_cnt - 6'd1;
                else begin
                    // End of window: ODT must have been seen at least once.
                    if (!odt_seen_in_window) begin
                        $display("[%0t] ERROR: ODT_MON: ODT was never asserted within window after WRITE command.",
                                 $time);
                        $fatal;
                    end
                    write_window_active <= 1'b0;
                end
            end

            // For READ commands, we require ODT to be deasserted on the same
            // cycle as the READ opcode.
            if (is_read && odt_pad) begin
                $display("[%0t] ERROR: ODT_MON: ODT should be deasserted during READ command.",
                         $time);
                $fatal;
            end
        end
    end

endmodule

