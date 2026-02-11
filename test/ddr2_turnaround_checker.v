`timescale 1ns/1ps
`include "ddr2_timing_config.vh"

/**
 * DDR2 turnaround / write-or-read-to-precharge timing checker.
 *
 * This coarse, pad-level monitor enforces:
 * - tWTR: minimum delay from WRITE to subsequent READ (any bank)
 * - tRTW: minimum delay from READ to subsequent WRITE (any bank)
 * - tWR : minimum delay from WRITE to PRECHARGE on the same bank
 * - tRTP: minimum delay from READ  to PRECHARGE on the same bank
 *
 * It is intended as a simulation aid, not a full JEDEC compliance model.
 */
module ddr2_turnaround_checker (
    input  wire       clk,
    input  wire       reset,
    input  wire       cke_pad,
    // AND-reduction of per-rank CS#; 0 when any rank is selected.
    input  wire       csbar_pad_any,
    input  wire       rasbar_pad,
    input  wire       casbar_pad,
    input  wire       webar_pad,
    input  wire [1:0] ba_pad
);

    // -------------------------------------------------------------------------
    // Timing parameters in controller clock cycles.
    // Values default to conservative JEDEC-like numbers but can be overridden
    // via `ddr2_timing_config.vh`.
    // -------------------------------------------------------------------------
`ifdef DDR2_TIMING_TWTR_MIN
    localparam integer TWTR_MIN = `DDR2_TIMING_TWTR_MIN;
`else
    localparam integer TWTR_MIN = 2;
`endif

`ifdef DDR2_TIMING_TRTW_MIN
    localparam integer TRTW_MIN = `DDR2_TIMING_TRTW_MIN;
`else
    localparam integer TRTW_MIN = 4;
`endif

`ifdef DDR2_TIMING_TWR_MIN
    localparam integer TWR_MIN  = `DDR2_TIMING_TWR_MIN;
`else
    localparam integer TWR_MIN  = 4;
`endif

`ifdef DDR2_TIMING_TRTP_MIN
    localparam integer TRTP_MIN = `DDR2_TIMING_TRTP_MIN;
`else
    localparam integer TRTP_MIN = 2;
`endif

    // -------------------------------------------------------------------------
    // Command decode at the DDR2 bus.
    // -------------------------------------------------------------------------
    wire cmd_active = cke_pad && !csbar_pad_any;

    // READ: RAS#=1, CAS#=0, WE#=1
    wire cmd_read  = cmd_active &&  rasbar_pad && !casbar_pad &&  webar_pad;
    // WRITE: RAS#=1, CAS#=0, WE#=0
    wire cmd_write = cmd_active &&  rasbar_pad && !casbar_pad && !webar_pad;
    // PRECHARGE: RAS#=0, CAS#=1, WE#=0
    wire cmd_pre   = cmd_active && !rasbar_pad &&  casbar_pad && !webar_pad;

    // -------------------------------------------------------------------------
    // Track last READ / WRITE times (global and per-bank) and enforce:
    // - tWTR / tRTW on any-bank sequences.
    // - tWR / tRTP on PRECHARGE to the same bank.
    // -------------------------------------------------------------------------
    integer cycle;
    integer last_wr_any;
    integer last_rd_any;

    integer last_wr_bank [0:3];
    integer last_rd_bank [0:3];

    integer b;

    initial begin
        cycle       = 0;
        last_wr_any = -1;
        last_rd_any = -1;
        for (b = 0; b < 4; b = b + 1) begin
            last_wr_bank[b] = -1;
            last_rd_bank[b] = -1;
        end
    end

    always @(posedge clk) begin
        if (reset) begin
            cycle       <= 0;
            last_wr_any <= -1;
            last_rd_any <= -1;
            for (b = 0; b < 4; b = b + 1) begin
                last_wr_bank[b] <= -1;
                last_rd_bank[b] <= -1;
            end
        end else begin
            cycle <= cycle + 1;

            // Track WRITE issue times and enforce READ-to-WRITE (tRTW).
            if (cmd_write) begin
                last_wr_any <= cycle;
                last_wr_bank[ba_pad] <= cycle;
                if (last_rd_any >= 0 && (cycle - last_rd_any) < TRTW_MIN) begin
                    $display("[%0t] ERROR: tRTW violated: READ→WRITE interval %0d < %0d cycles",
                             $time, cycle - last_rd_any, TRTW_MIN);
                    $fatal;
                end
            end

            // Track READ issue times and enforce WRITE-to-READ (tWTR).
            if (cmd_read) begin
                last_rd_any <= cycle;
                last_rd_bank[ba_pad] <= cycle;
                if (last_wr_any >= 0 && (cycle - last_wr_any) < TWTR_MIN) begin
                    $display("[%0t] ERROR: tWTR violated: WRITE→READ interval %0d < %0d cycles",
                             $time, cycle - last_wr_any, TWTR_MIN);
                    $fatal;
                end
            end

            // PRECHARGE: enforce tWR and tRTP on this bank.
            if (cmd_pre) begin
                if (last_wr_bank[ba_pad] >= 0 &&
                    (cycle - last_wr_bank[ba_pad]) < TWR_MIN) begin
                    $display("[%0t] ERROR: tWR violated on bank %0d: WRITE→PRE %0d < %0d cycles",
                             $time, ba_pad, cycle - last_wr_bank[ba_pad], TWR_MIN);
                    $fatal;
                end
                if (last_rd_bank[ba_pad] >= 0 &&
                    (cycle - last_rd_bank[ba_pad]) < TRTP_MIN) begin
                    $display("[%0t] ERROR: tRTP violated on bank %0d: READ→PRE %0d < %0d cycles",
                             $time, ba_pad, cycle - last_rd_bank[ba_pad], TRTP_MIN);
                    $fatal;
                end
            end
        end
    end

endmodule

