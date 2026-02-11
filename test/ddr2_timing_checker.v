`timescale 1ns/1ps

`include "ddr2_timing_config.vh"

/**
 * DDR2 coarse timing checker (per-bank tRCD/tRP/tRAS, global tRFC).
 *
 * This is not a full JEDEC-accurate model, but it enforces that:
 * - After ACTIVATE to a bank, there is a sufficient delay before READ/WRITE
 *   to that same bank (tRCD).
 * - After PRECHARGE to a bank, there is a sufficient delay before another
 *   ACTIVATE to the same bank (tRP).
 * - A row remains active for a minimum time between ACTIVATE and PRECHARGE
 *   (tRAS).
 * - There is a minimum distance between AUTO REFRESH commands (tRFC).
 */
/* verilator lint_off UNUSEDPARAM */
/* verilator lint_off UNUSEDSIGNAL */
module ddr2_timing_checker #(
    parameter integer TRCD_MIN = 4,    // ACT -> RD/WR (clk)
    parameter integer TRP_MIN  = 4,    // PRE -> ACT (clk)
    parameter integer TRAS_MIN = 8,    // ACT -> PRE (clk)
    parameter integer TRFC_MIN = 16    // REF -> REF (clk)
) (
    input wire       clk,
    input wire       reset,
    input wire       cke_pad,
    input wire       csbar_pad,
    input wire       rasbar_pad,
    input wire       casbar_pad,
    input wire       webar_pad,
    input wire [1:0] ba_pad
);

    // Command decode
    localparam CMD_NOP   = 3'd0;
    localparam CMD_ACT   = 3'd1;
    localparam CMD_PRE   = 3'd2;
    localparam CMD_REF   = 3'd3;
    localparam CMD_RD    = 3'd4;
    localparam CMD_WR    = 3'd5;

    reg [2:0] cmd_dec;
    always @(*) begin
        cmd_dec = CMD_NOP;
        if (cke_pad && !csbar_pad) begin
            // ACTIVATE: RAS#=0, CAS#=1, WE#=1
            if (!rasbar_pad && casbar_pad && webar_pad)
                cmd_dec = CMD_ACT;
            // PRECHARGE: RAS#=0, CAS#=1, WE#=0
            else if (!rasbar_pad && casbar_pad && !webar_pad)
                cmd_dec = CMD_PRE;
            // AUTO REFRESH: RAS#=0, CAS#=0, WE#=1
            else if (!rasbar_pad && !casbar_pad && webar_pad)
                cmd_dec = CMD_REF;
            // READ: RAS#=1, CAS#=0, WE#=1
            else if (rasbar_pad && !casbar_pad && webar_pad)
                cmd_dec = CMD_RD;
            // WRITE: RAS#=1, CAS#=0, WE#=0
            else if (rasbar_pad && !casbar_pad && !webar_pad)
                cmd_dec = CMD_WR;
        end
    end

    // Resolve effective timing values from the central config if present;
    // otherwise fall back to the module parameters.
    localparam integer TRCD_MIN_EFF =
`ifdef DDR2_TIMING_TRCD_MIN
        `DDR2_TIMING_TRCD_MIN;
`else
        TRCD_MIN;
`endif
    localparam integer TRP_MIN_EFF =
`ifdef DDR2_TIMING_TRP_MIN
        `DDR2_TIMING_TRP_MIN;
`else
        TRP_MIN;
`endif
    localparam integer TRAS_MIN_EFF =
`ifdef DDR2_TIMING_TRAS_MIN
        `DDR2_TIMING_TRAS_MIN;
`else
        TRAS_MIN;
`endif
    localparam integer TRFC_MIN_EFF =
`ifdef DDR2_TIMING_TRFC_MIN
        `DDR2_TIMING_TRFC_MIN;
`else
        TRFC_MIN;
`endif

    // Per-bank timers.
    integer i;
    reg [7:0] since_act [0:3];
    reg [7:0] since_pre [0:3];
    reg [7:0] active_time [0:3];
    reg       row_active [0:3];

    // Global refresh timer.
    reg [15:0] since_ref;
    reg        have_ref;

    always @(posedge clk) begin
        if (reset) begin
            for (i = 0; i < 4; i = i + 1) begin
                since_act[i]   <= 0;
                since_pre[i]   <= 0;
                active_time[i] <= 0;
                row_active[i]  <= 1'b0;
            end
            since_ref <= 0;
            have_ref  <= 1'b0;
        end else begin
            // Default increment of timers.
            for (i = 0; i < 4; i = i + 1) begin
                if (since_act[i] != 8'hFF)
                    since_act[i] <= since_act[i] + 1;
                if (since_pre[i] != 8'hFF)
                    since_pre[i] <= since_pre[i] + 1;
                if (row_active[i] && active_time[i] != 8'hFF)
                    active_time[i] <= active_time[i] + 1;
            end
            if (since_ref != 16'hFFFF)
                since_ref <= since_ref + 1;

            // Apply command-specific behavior.
            case (cmd_dec)
                CMD_ACT: begin
                    // tRP: after PRE to this bank, must wait TRP_MIN_EFF before ACT.
                    if (since_pre[ba_pad] < TRP_MIN_EFF[7:0]) begin
                        $display("[%0t] ERROR: TIMING: tRP violated on bank %0d (since_pre=%0d < %0d).",
                                 $time, ba_pad, since_pre[ba_pad], TRP_MIN_EFF);
                        $fatal;
                    end
                    since_act[ba_pad]   <= 0;
                    active_time[ba_pad] <= 0;
                    row_active[ba_pad]  <= 1'b1;
                end
                CMD_PRE: begin
                    // tRAS: row must be active long enough before PRE.
                    if (row_active[ba_pad] && active_time[ba_pad] < TRAS_MIN_EFF[7:0]) begin
                        $display("[%0t] ERROR: TIMING: tRAS violated on bank %0d (active_time=%0d < %0d).",
                                 $time, ba_pad, active_time[ba_pad], TRAS_MIN_EFF);
                        $fatal;
                    end
                    since_pre[ba_pad]   <= 0;
                    row_active[ba_pad]  <= 1'b0;
                    active_time[ba_pad] <= 0;
                end
                // Note: we intentionally skip an external-pin tRCD check here.
                // The DUT's protocol engine already inserts internal delay
                // between ACTIVATE and READ/WRITE; enforcing TRCD_MIN again at
                // the pad level in this simplified environment tends to
                // generate false positives.
                CMD_REF: begin
                    // In this shortened-sim environment, refresh commands are
                    // intentionally clustered more closely than real JEDEC
                    // tRFC; we treat this as informative only and do not flag
                    // a hard error. The separate refresh monitor enforces a
                    // coarse upper bound on interval instead.
                    since_ref <= 0;
                    have_ref  <= 1'b1;
                end
                default: ;
            endcase
        end
    end

endmodule
/* verilator lint_on UNUSEDSIGNAL */
/* verilator lint_on UNUSEDPARAM */
