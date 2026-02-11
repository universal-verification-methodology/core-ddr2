`timescale 1ns/1ps

`include "ddr2_timing_config.vh"

/**
 * DDR2 timing checker (per-bank tRCD/tRP/tRAS, global tRFC, plus basic
 * tRRD/tFAW coverage).
 *
 * The intent is to approximate JEDEC timing at the pad level using the
 * controller clock as a reference:
 * - After ACTIVATE to a bank, there must be a sufficient delay before
 *   READ/WRITE to that same bank (tRCD).
 * - After PRECHARGE to a bank, there must be a sufficient delay before
 *   another ACTIVATE to the same bank (tRP).
 * - A row remains active for a minimum time between ACTIVATE and PRECHARGE
 *   (tRAS).
 * - There is a minimum distance between AUTO REFRESH commands (tRFC).
 *
 * Exact numeric thresholds are provided by `ddr2_timing_config.vh` so that
 * different speed grades / simulation profiles can be selected centrally.
 */
/* verilator lint_off UNUSEDPARAM */
/* verilator lint_off UNUSEDSIGNAL */
module ddr2_timing_checker #(
    parameter integer TRCD_MIN = 4,    // ACT -> RD/WR (clk)
    parameter integer TRP_MIN  = 4,    // PRE -> ACT (clk)
    parameter integer TRAS_MIN = 8,    // ACT -> PRE (clk)
    parameter integer TRFC_MIN = 16,   // REF -> REF (clk)
    parameter integer TRRD_MIN = 2,    // ACT -> ACT (different bank) (clk)
    parameter integer TFAW_MIN = 16    // four-ACT window (clk)
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
    reg [2:0] cmd_dec_prev;
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

    // One-cycle pulse when an ACT command is *issued* (edge-detected) as
    // opposed to held across multiple cycles. This avoids double-counting a
    // single ACTIVATE that happens to be driven for more than one clk.
    wire act_pulse = (cmd_dec == CMD_ACT) && (cmd_dec_prev != CMD_ACT);
    // Similarly, edge-detect REF commands to avoid processing the same REF
    // multiple times if it's held across multiple cycles.
    wire ref_pulse = (cmd_dec == CMD_REF) && (cmd_dec_prev != CMD_REF);

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
    localparam integer TRRD_MIN_EFF =
`ifdef DDR2_TIMING_TRRD_MIN
        `DDR2_TIMING_TRRD_MIN;
`else
        TRRD_MIN;
`endif
    localparam integer TFAW_MIN_EFF =
`ifdef DDR2_TIMING_TFAW_MIN
        `DDR2_TIMING_TFAW_MIN;
`else
        TFAW_MIN;
`endif

    // Per-bank timers.
    integer i;
    reg [7:0] since_act [0:3];
    reg [7:0] since_pre [0:3];
    reg [7:0] active_time [0:3];
    reg       row_active [0:3];

    // Global refresh timer: distance between successive AUTO REFRESH commands.
    reg [15:0] since_ref;
    reg        have_ref;

    // Global ACT timing state for tRRD/tFAW.
    // - since_last_act_any: cycles since the most recent ACT (for tRRD).
    // - act_cycle: free-running cycle counter used for tFAW windowing.
    // - last_act_cycles[0..3]: timestamps of the four most recent ACTs used to
    //   approximate JEDEC's "four-activate window" (tFAW). When a new ACT
    //   occurs, we compare the current cycle against the oldest stored ACT; if
    //   the delta is smaller than TFAW_MIN_EFF, we flag a violation.
    reg [15:0] since_last_act_any;
    reg        have_act_any;
    // Use a wider, non-saturating cycle counter for the ACT timestamps so that
    // long-running simulations (millions of cycles) do not cause the
    // tFAW window to collapse spuriously once the counter saturates. A 32-bit
    // width is ample for these testbenches while keeping the logic simple.
    reg [31:0] act_cycle;
    reg [31:0] last_act_cycles [0:3];
    integer    fa_idx;

    always @(posedge clk) begin
        if (reset) begin
            for (i = 0; i < 4; i = i + 1) begin
                since_act[i]   <= 0;
                since_pre[i]   <= 0;
                active_time[i] <= 0;
                row_active[i]  <= 1'b0;
            end
            since_ref          <= 0;
            have_ref           <= 1'b0;
            since_last_act_any <= 0;
            have_act_any       <= 1'b0;
            act_cycle          <= 0;
            for (i = 0; i < 4; i = i + 1) begin
                last_act_cycles[i] <= 0;
            end
            cmd_dec_prev <= CMD_NOP;
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

            // Global ACT timing increments.
            if (since_last_act_any != 16'hFFFF)
                since_last_act_any <= since_last_act_any + 1;
            // Allow the ACT timestamp counter to run freely; wrap-around is
            // acceptable because the tFAW comparison only cares about small
            // deltas (on the order of tens of cycles) between the current ACT
            // and the oldest entry in the four‑ACT window.
            act_cycle <= act_cycle + 1;

            // Apply command-specific behavior.
            case (cmd_dec)
                CMD_ACT: begin
                    if (act_pulse) begin
                        // tRRD: distance between successive ACTs (any banks).
                        if (have_act_any && since_last_act_any < TRRD_MIN_EFF[15:0]) begin
                            $display("[%0t] ERROR: TIMING: tRRD violated (since_last_act_any=%0d < %0d).",
                                     $time, since_last_act_any, TRRD_MIN_EFF);
                            $fatal;
                        end
                        since_last_act_any <= 0;
                        have_act_any       <= 1'b1;

                        // tFAW: four-activate window. Once we have seen at
                        // least four ACTs, ensure that the time between the
                        // oldest of the last four and the current one is
                        // >= TFAW_MIN_EFF.
                        if (last_act_cycles[0] != 16'd0) begin
                            if ((act_cycle - last_act_cycles[0]) < TFAW_MIN_EFF[15:0]) begin
                                $display("[%0t] ERROR: TIMING: tFAW violated (ACT window=%0d < %0d cycles).",
                                         $time, act_cycle - last_act_cycles[0], TFAW_MIN_EFF);
                                $fatal;
                            end
                        end
                        // Shift window and insert current ACT timestamp.
                        for (fa_idx = 0; fa_idx < 3; fa_idx = fa_idx + 1) begin
                            last_act_cycles[fa_idx] <= last_act_cycles[fa_idx+1];
                        end
                        last_act_cycles[3] <= act_cycle;

                        // tRP: after PRE to this bank, must wait TRP_MIN_EFF
                        // before ACT.
                        if (since_pre[ba_pad] < TRP_MIN_EFF[7:0]) begin
                            $display("[%0t] ERROR: TIMING: tRP violated on bank %0d (since_pre=%0d < %0d).",
                                     $time, ba_pad, since_pre[ba_pad], TRP_MIN_EFF);
                            $fatal;
                        end
                        since_act[ba_pad]   <= 0;
                        active_time[ba_pad] <= 0;
                        row_active[ba_pad]  <= 1'b1;
                    end
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
                CMD_RD,
                CMD_WR: begin
                    // tRCD: enforce ACTIVATE-to-READ/WRITE minimum on this bank.
                    if (row_active[ba_pad] && since_act[ba_pad] < TRCD_MIN_EFF[7:0]) begin
                        $display("[%0t] ERROR: TIMING: tRCD violated on bank %0d (since_act=%0d < %0d).",
                                 $time, ba_pad, since_act[ba_pad], TRCD_MIN_EFF);
                        $fatal;
                    end
                end
                CMD_REF: begin
                    // tRFC: enforce a minimum distance between successive AUTO
                    // REFRESH commands. The separate refresh monitor checks
                    // that the overall refresh interval (tREFI) stays within
                    // bounds; here we focus specifically on REF-to-REF.
                    // Only process REF commands on the rising edge to avoid
                    // double-counting if the command is held for multiple cycles.
                    if (ref_pulse) begin
                        if (have_ref && since_ref < TRFC_MIN_EFF[15:0]) begin
                            $display("[%0t] ERROR: TIMING: tRFC violated (REF→REF interval=%0d < %0d).",
                                     $time, since_ref, TRFC_MIN_EFF);
                            $fatal;
                        end
                        since_ref <= 0;
                        have_ref  <= 1'b1;
                    end
                end
                default: ;
            endcase

            // Track previous decoded command for edge detection.
            cmd_dec_prev <= cmd_dec;
        end
    end

endmodule
/* verilator lint_on UNUSEDSIGNAL */
/* verilator lint_on UNUSEDPARAM */
