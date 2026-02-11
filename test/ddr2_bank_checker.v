`timescale 1ns/1ps

/**
 * DDR2 bank/row checker.
 *
 * Tracks per-bank "open row" state and flags illegal row conflicts:
 * - If a bank already has an open row (ACT seen, no PRE yet), a new ACT to
 *   the *same* bank with a *different* row address must be preceded by
 *   PRECHARGE and must satisfy tRP_min between PRE and ACT.
 *
 * This is a coarse safety net derived from common DDR2 controller references.
 */
module ddr2_bank_checker #(
    parameter integer TRP_MIN = 4
) (
    input wire       clk,
    input wire       reset,
    input wire       cke_pad,
    input wire       csbar_pad,
    input wire       rasbar_pad,
    input wire       casbar_pad,
    input wire       webar_pad,
    input wire [1:0] ba_pad,
    input wire [12:0] a_pad
);

    // Decode ACT/PRE commands.
    wire is_act, is_pre;
    assign is_act = cke_pad && !csbar_pad && !rasbar_pad && casbar_pad && webar_pad;
    assign is_pre = cke_pad && !csbar_pad && !rasbar_pad && casbar_pad && !webar_pad;

    integer i;
    reg       bank_open [0:3];
    reg [12:0] open_row [0:3];
    reg [7:0]  since_pre [0:3];

    always @(posedge clk) begin
        if (reset) begin
            for (i = 0; i < 4; i = i + 1) begin
                bank_open[i] <= 1'b0;
                open_row[i]  <= 13'b0;
                since_pre[i] <= 0;
            end
        end else begin
            // Default increment.
            for (i = 0; i < 4; i = i + 1) begin
                if (since_pre[i] != 8'hFF)
                    since_pre[i] <= since_pre[i] + 1;
            end

            if (is_pre) begin
                bank_open[ba_pad] <= 1'b0;
                since_pre[ba_pad] <= 0;
            end

            if (is_act) begin
                if (bank_open[ba_pad] && open_row[ba_pad] != a_pad) begin
                    // Row conflict: same bank, different row, old row still open.
                    if (since_pre[ba_pad] < TRP_MIN[7:0]) begin
                        $display("[%0t] ERROR: BANK: row conflict on bank %0d (open_row=%0d new_row=%0d, tRP violated).",
                                 $time, ba_pad, open_row[ba_pad], a_pad);
                        $fatal;
                    end
                end
                bank_open[ba_pad] <= 1'b1;
                open_row[ba_pad]  <= a_pad;
            end
        end
    end

endmodule

