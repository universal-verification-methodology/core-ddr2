`timescale 1ns/1ps

/**
 * DDR2 DQS activity monitor.
 *
 * For each WRITE command, opens a small observation window and counts
 * transitions on DQS[0]. Flags an error if:
 * - No edges are seen (controller failed to toggle DQS during a WRITE).
 * - Excessive edges are seen (likely glitchy or stuck-toggling behavior).
 */
/* verilator lint_off UNUSEDSIGNAL */
module ddr2_dqs_monitor #(
    parameter integer MIN_DQS_EDGES = 2,
    parameter integer MAX_DQS_EDGES = 32,
    parameter integer WINDOW_CYCLES = 32
) (
    input wire       clk,
    input wire       reset,
    input wire       cke_pad,
    input wire       csbar_pad,
    input wire       rasbar_pad,
    input wire       casbar_pad,
    input wire       webar_pad,
    input wire [1:0] dqs_pad
);

    // Decode WRITE command.
    wire is_write_cmd;
    assign is_write_cmd =
        cke_pad && !csbar_pad &&
        rasbar_pad && !casbar_pad && !webar_pad;

    reg [5:0]  win_cnt;
    reg [7:0]  edge_cnt;
    reg        in_window;
    reg        last_dqs;
    reg        last_dqs_valid;

    always @(posedge clk) begin
        if (reset) begin
            win_cnt        <= 0;
            edge_cnt       <= 0;
            in_window      <= 1'b0;
            last_dqs       <= 1'b0;
            last_dqs_valid <= 1'b0;
        end else begin
            // Start a new observation window on each WRITE command.
            if (is_write_cmd) begin
                in_window      <= 1'b1;
                win_cnt        <= 0;
                edge_cnt       <= 0;
                last_dqs       <= dqs_pad[0];
                last_dqs_valid <= 1'b1;
                $display("[%0t] DQS_MON: WRITE detected, starting DQS observation window.", $time);
            end else if (in_window) begin
                win_cnt <= win_cnt + 1;
                if (last_dqs_valid && dqs_pad[0] ^ last_dqs)
                    edge_cnt <= edge_cnt + 1;
                last_dqs       <= dqs_pad[0];
                last_dqs_valid <= 1'b1;

                if (win_cnt >= WINDOW_CYCLES[5:0]) begin
                    // End of window: evaluate edge count.
                    $display("[%0t] DQS_MON: window complete, edge_cnt=%0d.", $time, edge_cnt);
                    if (edge_cnt < MIN_DQS_EDGES[7:0]) begin
                        $display("[%0t] ERROR: DQS_MON: too few DQS edges after WRITE (edges=%0d, min=%0d).",
                                 $time, edge_cnt, MIN_DQS_EDGES);
                        $fatal;
                    end
                    if (edge_cnt > MAX_DQS_EDGES[7:0]) begin
                        $display("[%0t] ERROR: DQS_MON: too many DQS edges after WRITE (edges=%0d, max=%0d).",
                                 $time, edge_cnt, MAX_DQS_EDGES);
                        $fatal;
                    end
                    in_window <= 1'b0;
                end
            end
        end
    end

endmodule
/* verilator lint_on UNUSEDSIGNAL */
