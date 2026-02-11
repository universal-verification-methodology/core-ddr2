`timescale 1ns/1ps

/**
 * DDR2 protocol engine: command decode, ACTIVATE/READ/WRITE/REFRESH sequencing,
 * timing counters, ring buffer and return FIFO control.
 */
/* verilator lint_off UNUSEDPARAM */
/* verilator lint_off UNUSEDSIGNAL */
module ddr2_protocol_engine #(
    parameter integer AL              = 4,
    parameter integer CL              = 4,
    parameter integer CWL             = 3,
    // Logical address geometry parameters. By default these match the original
    // fixed configuration (25-bit logical address: 13 row, 10 column, 2 bank).
    // The mapping from ADDR to {row, bank, column} assumes:
    //   ADDR_WIDTH == ROW_ADDR_WIDTH + COL_ADDR_WIDTH + BANK_ADDR_WIDTH
    //   - Lowest 3 bits are the intra-burst column offset (COL low bits).
    //   - Next BANK_ADDR_WIDTH bits encode the bank.
    //   - Remaining COL high bits sit just below the row.
    //   - Upper ROW_ADDR_WIDTH bits encode the row.
    parameter integer ROW_ADDR_WIDTH  = 13,
    parameter integer COL_ADDR_WIDTH  = 10,
    parameter integer BANK_ADDR_WIDTH = 2,
    parameter integer ADDR_WIDTH      = 25
) (
    input  wire         clk,
    input  wire         reset,
    input  wire         ready_init_i,
    // Optional rank select from the higher-level controller. In the current
    // single-rank configuration this is tied low; it is captured alongside
    // addr/cmd/sz for debug/observability and also driven out on rank_sel_o
    // so that top-level logic can derive per-rank CS# vectors.
    input  wire         rank_sel_i,
    input  wire [ADDR_WIDTH-1:0]  addr_cmdFIFO_i,
    input  wire [2:0]   cmd_cmdFIFO_i,
    input  wire [1:0]   sz_cmdFIFO_i,
    input  wire [15:0]  din_dataFIFO_i,
    input  wire         emptyBar_cmdFIFO_i,
    input  wire         fullBar_returnFIFO_i,
    input  wire [6:0]   fillcount_returnFIFO_i,
    input  wire [15:0]  ringBuff_dout_i,
    output reg          get_cmdFIFO_o,
    output reg          get_dataFIFO_o,
    output reg          put_returnFIFO_o,
    output reg  [ADDR_WIDTH-1:0]  addr_returnFIFO_o,
    output reg          listen_ringBuff_o,
    output reg  [2:0]   readPtr_ringBuff_o,
    output reg          csbar_o,
    output reg          rasbar_o,
    output reg          casbar_o,
    output reg          webar_o,
    output reg  [1:0]   ba_o,
    output reg  [12:0]  a_o,
    output reg  [15:0]  dq_SSTL_o,
    output reg  [1:0]   dm_SSTL_o,
    output reg  [1:0]   dqs_SSTL_o,
    output reg          ts_i_o,
    output reg          ri_i_o,
    // Observable rank select associated with the current command/burst.
    output reg          rank_sel_o
);

    localparam NOP  = 3'b000;
    localparam SCR  = 3'b001;
    localparam SCW  = 3'b010;
    localparam BLR  = 3'b011;
    localparam BLW  = 3'b100;

    localparam PRE_ALL_A = 13'h400;
    localparam CNT_GETCMD   = 6'd2;
    localparam CNT_ACTIVATE = 6'd1;
    // tRCD wait: minimum cycles between ACTIVATE and READ/WRITE.
    // Use timing config if available, otherwise default to 4 cycles.
    // Since ACTIVATE already takes 1 cycle, we need to wait 3 more cycles.
    localparam integer TRCD_WAIT =
`ifdef DDR2_TIMING_TRCD_MIN
        (`DDR2_TIMING_TRCD_MIN > 1) ? (`DDR2_TIMING_TRCD_MIN - 1) : 0;
`else
        3;  // Default: 4 cycles total - 1 cycle for ACTIVATE = 3 wait cycles
`endif
    // SCREAD delay between issuing a READ and starting to capture return data.
    // This should be tuned to line up the controller's capture point with the
    // simple DDR2 memory model's READ_LAT so that the first word pushed into
    // the return FIFO is already valid.
    localparam CNT_SCREAD   = 6'd26;
    localparam CNT_SCWRITE  = 6'd12;   // 2*(AL+CL-1)-2
    localparam CNT_SCWRDATA_SCW = 6'd7;
    localparam CNT_SCWREND  = 6'd17;
    localparam CNT_SCRNLSN  = 6'd5;
    localparam CNT_SCRDNTH  = 6'd3;
    localparam CNT_SCREND   = 6'd3;
    localparam CNT_RPRE     = 6'd1;
    localparam CNT_RNOP     = 6'd7;
    localparam CNT_RREF     = 6'd1;
    localparam CNT_RIDL     = 6'd55;
    localparam REF_CNT_INIT = 12'd3900;
    localparam REF_THRESH   = 12'd100;

    // Minimum spacing (in controller clocks) enforced between successive ACT
    // commands issued on the DDR2 bus by this protocol engine. Choosing a
    // conservative value here (>= 6) guarantees that the JEDEC-style four-
    // activate window tFAW of 16 cycles is respected, since the time between
    // the first and fourth ACT will be at least 3 * ACT_GAP_MIN cycles. This
    // mirrors the behavior checked by ddr2_timing_checker while keeping the
    // implementation simple and self-contained.
    localparam integer ACT_GAP_MIN = 6;

    localparam STATE_BITS = 5;
    localparam IDLE       = 5'd0,
               GETCMD     = 5'd1,
               ACTIVATE   = 5'd2,
               WAIT_TRCD  = 5'd23,  // Wait for tRCD after ACTIVATE
               ISSUE_READ = 5'd3,
               SCREAD     = 5'd4,
               SCRLSN     = 5'd5,
               SCRNLSN    = 5'd6,
               SCRDNTH    = 5'd7,
               SCREND     = 5'd8,
               ISSUE_WRITE= 5'd9,
               SCWRITE    = 5'd10,
               SCWRDATA   = 5'd11,
               SCWREND    = 5'd12,
               BLR_ACT    = 5'd13,
               BLR_ISSUE  = 5'd14,
               BLR_READ   = 5'd15,
               BLW_ACT    = 5'd16,
               BLW_ISSUE  = 5'd17,
               BLW_WRITE  = 5'd18,
               RPRE       = 5'd19,
               RNOP       = 5'd20,
               RREF       = 5'd21,
               RIDL       = 5'd22;

    reg [STATE_BITS-1:0] state;
    reg [5:0] cnt;
    reg [11:0] refCnt;
    reg [ADDR_WIDTH-1:0] addr_reg;
    reg [2:0]  cmd_reg;
    reg [1:0]  sz_reg;
    // Latched copy of the rank select associated with the current command.
    reg        rank_sel_reg;
    reg [ROW_ADDR_WIDTH-1:0]  row_reg;
    reg [BANK_ADDR_WIDTH-1:0] bank_reg;
    reg [COL_ADDR_WIDTH-1:0]  col_reg;
    reg [2:0]  burst_cnt;   // 0..7 for read pointer / write words
    reg [1:0]  blk_cnt;     // block burst index for BLR/BLW
    reg        from_blw;    // 1 if SCWRDATA entered from BLW_WRITE
    reg [STATE_BITS-1:0] next_state_after_trcd;  // State to transition to after WAIT_TRCD
    // Global ACT timing helper: tracks cycles since the most recent ACT so
    // that a minimum spacing (ACT_GAP_MIN) can be enforced between ACT
    // commands. This provides a simple, architecture-local way to satisfy both
    // tRRD/tFAW-style constraints without duplicating the full checker.
    reg [5:0]  act_gap_cnt;
    reg        have_act_any;

    // Address slicing helpers. With the default geometry
    // (ROW_ADDR_WIDTH=13, COL_ADDR_WIDTH=10, BANK_ADDR_WIDTH=2, ADDR_WIDTH=25)
    // this reduces exactly to the original mapping:
    //   row  = ADDR[24:12]
    //   bank = ADDR[4:3]
    //   col  = {ADDR[11:5], ADDR[2:0]}
    localparam integer COL_LOW_BITS   = 3;
    localparam integer COL_HIGH_BITS  = COL_ADDR_WIDTH - COL_LOW_BITS;
    localparam integer ROW_MSB        = ADDR_WIDTH - 1;
    localparam integer ROW_LSB        = ADDR_WIDTH - ROW_ADDR_WIDTH;
    localparam integer BANK_LSB       = COL_LOW_BITS;
    localparam integer BANK_MSB       = BANK_LSB + BANK_ADDR_WIDTH - 1;
    localparam integer COL_HIGH_MSB   = BANK_LSB + BANK_ADDR_WIDTH + COL_HIGH_BITS - 1;
    localparam integer COL_HIGH_LSB   = BANK_LSB + BANK_ADDR_WIDTH;

    wire [ROW_ADDR_WIDTH-1:0]  row_addr  = addr_cmdFIFO_i[ROW_MSB:ROW_LSB];
    wire [BANK_ADDR_WIDTH-1:0] bank_addr = addr_cmdFIFO_i[BANK_MSB:BANK_LSB];
    wire [COL_ADDR_WIDTH-1:0]  col_addr  = {
        addr_cmdFIFO_i[COL_HIGH_MSB:COL_HIGH_LSB],
        addr_cmdFIFO_i[COL_LOW_BITS-1:0]
    };

    // Internal helpers for mapping column to the 13-bit DDR2 A bus.
    // We use 9 column bits on A[9:1]; A10 is the auto-precharge flag and
    // the remaining upper address bits are zero for this configuration.
    wire [8:0] col_a_bus = col_reg[8:0];
    // Block-read/write column offset helper: work purely in 9 bits so that
    // the ADD operands match in width and avoid implicit expansion.
    wire [8:0]  col_with_blk = col_reg[8:0] + {4'b0, blk_cnt, 3'b0};

    wire need_refresh = ready_init_i && (refCnt < REF_THRESH);
    wire [5:0] cnt_scwrdata_sz = (sz_reg == 2'd0) ? 6'd7 :
                                 (sz_reg == 2'd1) ? 6'd15 :
                                 (sz_reg == 2'd2) ? 6'd23 : 6'd31;
    wire [6:0] return_thresh = (sz_reg == 2'd0) ? 7'd56 :
                               (sz_reg == 2'd1) ? 7'd48 :
                               (sz_reg == 2'd2) ? 7'd40 : 7'd32;

    always @(posedge clk) begin
        if (reset) begin
            state   <= IDLE;
            cnt     <= 6'd0;
            refCnt  <= REF_CNT_INIT;
            get_cmdFIFO_o  <= 1'b0;
            get_dataFIFO_o <= 1'b0;
            put_returnFIFO_o <= 1'b0;
            listen_ringBuff_o <= 1'b0;
            readPtr_ringBuff_o <= 3'd0;
            csbar_o <= 1'b1;
            rasbar_o <= 1'b1;
            casbar_o <= 1'b1;
            webar_o  <= 1'b1;
            ba_o     <= 2'b00;
            a_o      <= 13'b0;
            dq_SSTL_o <= 16'b0;
            dm_SSTL_o <= 2'b00;
            dqs_SSTL_o <= 2'b00;
            ts_i_o   <= 1'b0;
            ri_i_o   <= 1'b0;
            addr_reg <= {ADDR_WIDTH{1'b0}};
            cmd_reg  <= NOP;
            sz_reg   <= 2'b00;
            rank_sel_reg <= 1'b0;
            row_reg  <= {ROW_ADDR_WIDTH{1'b0}};
            bank_reg <= {BANK_ADDR_WIDTH{1'b0}};
            col_reg  <= {COL_ADDR_WIDTH{1'b0}};
            burst_cnt <= 3'd0;
            blk_cnt  <= 2'd0;
            from_blw <= 1'b0;
            next_state_after_trcd <= IDLE;
            addr_returnFIFO_o <= {ADDR_WIDTH{1'b0}};
            rank_sel_o <= 1'b0;
            act_gap_cnt <= 6'd0;
            have_act_any <= 1'b0;
        end else begin
            if (state == IDLE && !need_refresh && ready_init_i)
                refCnt <= refCnt - 12'd1;
            if (state == RIDL && cnt == 6'd0)
                refCnt <= REF_CNT_INIT;

            get_cmdFIFO_o  <= 1'b0;
            get_dataFIFO_o <= 1'b0;
            put_returnFIFO_o <= 1'b0;
            listen_ringBuff_o <= 1'b0;
            ts_i_o <= 1'b0;
            ri_i_o <= 1'b0;
            // By default, propagate the latched rank select so that any cycle
            // that drives a CS# can be associated with the correct logical rank.
            rank_sel_o <= rank_sel_reg;

            // Global ACT gap counter behaviour: once at least one ACT has been
            // issued, count up each cycle until saturated. Specific ACT issue
            // points below will reset this counter when a new ACT is driven.
            if (have_act_any && act_gap_cnt != 6'h3F)
                act_gap_cnt <= act_gap_cnt + 6'd1;

            if (cnt > 6'd0)
                cnt <= cnt - 6'd1;

            case (state)
                IDLE: begin
                    csbar_o <= 1'b1;
                    rasbar_o <= 1'b1;
                    casbar_o <= 1'b1;
                    webar_o  <= 1'b1;
                    if (need_refresh) begin
                        state <= RPRE;
                        cnt   <= CNT_RPRE;
                    end else if (emptyBar_cmdFIFO_i && ready_init_i) begin
                        $display("[%0t] PROT: IDLE sees cmdFIFO non-empty (need_refresh=%b)",
                                 $time, need_refresh);
                        get_cmdFIFO_o <= 1'b1;
                        state <= GETCMD;
                        cnt   <= CNT_GETCMD;
                    end
                end

                GETCMD: begin
                    $display("[%0t] PROT: GETCMD state cnt=%0d", $time, cnt);
                    csbar_o <= 1'b1;
                    rasbar_o <= 1'b1;
                    casbar_o <= 1'b1;
                    webar_o  <= 1'b1;
                    if (cnt == 6'd0) begin
                        // Enforce a minimum inter-ACT gap before issuing a new
                        // ACTIVATE to open a row for this command. If the gap
                        // is not yet satisfied, simply hold in GETCMD until
                        // enough cycles have elapsed; the command and its
                        // associated address remain stable on the FIFO outputs
                        // while we wait.
                        if (!have_act_any || act_gap_cnt >= ACT_GAP_MIN[5:0]) begin
                            addr_reg     <= addr_cmdFIFO_i;
                            cmd_reg      <= cmd_cmdFIFO_i;
                            sz_reg       <= sz_cmdFIFO_i;
                            rank_sel_reg <= rank_sel_i;
                            $display("[%0t] PROT: GETCMD addr=%0d cmd=%0b sz=%0b rank_sel=%0b",
                                     $time, addr_cmdFIFO_i, cmd_cmdFIFO_i, sz_cmdFIFO_i, rank_sel_i);
                            row_reg  <= row_addr;
                            bank_reg <= bank_addr;
                            col_reg  <= col_addr;
                            state    <= ACTIVATE;
                            cnt      <= CNT_ACTIVATE;
                            act_gap_cnt <= 6'd0;
                            have_act_any <= 1'b1;
                        end
                    end
                end

                ACTIVATE: begin
                    csbar_o <= 1'b0;
                    rasbar_o <= 1'b0;
                    casbar_o <= 1'b1;
                    webar_o  <= 1'b1;
                    ba_o     <= bank_reg;
                    a_o      <= row_reg;
                    if (cnt == 6'd0) begin
                        // Wait for tRCD before issuing READ/WRITE commands
                        if (cmd_reg == SCR) begin
                            state <= WAIT_TRCD;
                            next_state_after_trcd <= ISSUE_READ;
                            cnt   <= TRCD_WAIT[5:0];
                        end else if (cmd_reg == SCW) begin
                            state <= WAIT_TRCD;
                            next_state_after_trcd <= ISSUE_WRITE;
                            cnt   <= TRCD_WAIT[5:0];
                        end else if (cmd_reg == BLR) begin
                            if (fillcount_returnFIFO_i < return_thresh) begin
                                // For block reads, an additional ACT is used
                                // to open the row for the multi-burst access.
                                // Gate this ACT behind the global ACT gap so
                                // that repeated BLR sequences cannot violate
                                // tFAW.
                                if (!have_act_any || act_gap_cnt >= ACT_GAP_MIN[5:0]) begin
                                    state   <= BLR_ACT;
                                    blk_cnt <= 2'd0;
                                    cnt     <= CNT_ACTIVATE;
                                    act_gap_cnt <= 6'd0;
                                    have_act_any <= 1'b1;
                                end
                            end else
                                state <= IDLE;
                        end else if (cmd_reg == BLW) begin
                            // Similarly gate the BLW ACT that opens the row
                            // for a block write so that successive BLW bursts
                            // cannot compress ACTs inside the four-activate
                            // window.
                            if (!have_act_any || act_gap_cnt >= ACT_GAP_MIN[5:0]) begin
                                state   <= BLW_ACT;
                                blk_cnt <= 2'd0;
                                cnt     <= CNT_ACTIVATE;
                                act_gap_cnt <= 6'd0;
                                have_act_any <= 1'b1;
                            end
                        end else
                            state <= IDLE;
                    end
                end

                WAIT_TRCD: begin
                    // Wait state to satisfy tRCD timing between ACTIVATE and READ/WRITE
                    csbar_o <= 1'b1;
                    rasbar_o <= 1'b1;
                    casbar_o <= 1'b1;
                    webar_o  <= 1'b1;
                    if (cnt == 6'd0) begin
                        state <= next_state_after_trcd;
                    end
                end

                ISSUE_READ: begin
                    csbar_o <= 1'b0;
                    rasbar_o <= 1'b1;
                    casbar_o <= 1'b0;
                    webar_o  <= 1'b1;
                    ba_o     <= bank_reg;
                    a_o      <= {3'b0, col_a_bus, 1'b1};  // A10=1 auto precharge
                    $display("[%0t] PROT: ISSUE_READ bank=%0d row=%0d col=%0d cs=%b ras=%b cas=%b we=%b",
                             $time, bank_reg, row_reg, col_reg, csbar_o, rasbar_o, casbar_o, webar_o);
                    state    <= SCREAD;
                    cnt      <= CNT_SCREAD;
                end

                SCREAD: begin
                    csbar_o <= 1'b1;
                    rasbar_o <= 1'b1;
                    casbar_o <= 1'b1;
                    webar_o  <= 1'b1;
                    if (cnt == 6'd0) begin
                        // Start capturing the 8-beat burst from the ring
                        // buffer exactly when the memory model has begun
                        // driving valid read data.
                        listen_ringBuff_o   <= 1'b1;
                        state               <= SCREND;
                        cnt                 <= 6'd8;   // push 8 words
                        put_returnFIFO_o    <= 1'b1;
                        addr_returnFIFO_o   <= addr_reg;
                        readPtr_ringBuff_o  <= 3'd0;
                    end
                end

                SCREND: begin
                    put_returnFIFO_o    <= (cnt != 6'd0);
                    addr_returnFIFO_o   <= addr_reg + {{(ADDR_WIDTH-3){1'b0}}, 3'(8 - cnt)};
                    readPtr_ringBuff_o  <= 3'(8 - cnt);
                    if (cnt == 6'd0) begin
                        if (cmd_reg == BLR && blk_cnt < sz_reg) begin
                            blk_cnt <= blk_cnt + 2'd1;
                            state <= BLR_ISSUE;
                        end else
                            state <= IDLE;
                    end
                end

                ISSUE_WRITE: begin
                    csbar_o <= 1'b0;
                    rasbar_o <= 1'b1;
                    casbar_o <= 1'b0;
                    webar_o  <= 1'b0;
                    ba_o     <= bank_reg;
                    a_o      <= {3'b0, col_a_bus, 1'b1};  // A10=1 auto precharge
                    state    <= SCWRITE;
                    cnt      <= CNT_SCWRITE;
                end

                SCWRITE: begin
                    csbar_o <= 1'b1;
                    rasbar_o <= 1'b1;
                    casbar_o <= 1'b1;
                    webar_o  <= 1'b1;
                    from_blw <= 1'b0;
                    if (cnt == 6'd0) begin
                        state    <= SCWRDATA;
                        cnt      <= CNT_SCWRDATA_SCW;
                        burst_cnt <= 3'd0;
                        ts_i_o   <= 1'b1;
                        get_dataFIFO_o <= 1'b1;
                        dq_SSTL_o <= din_dataFIFO_i;
                        dm_SSTL_o <= 2'b00;
                    end
                end

                SCWRDATA: begin
                    ts_i_o <= 1'b1;
                    csbar_o <= 1'b1;
                    rasbar_o <= 1'b1;
                    casbar_o <= 1'b1;
                    webar_o  <= 1'b1;
                    if (burst_cnt < 3'd7) begin
                        get_dataFIFO_o <= 1'b1;
                        dq_SSTL_o <= din_dataFIFO_i;
                        dm_SSTL_o <= 2'b00;
                        dqs_SSTL_o <= ~dqs_SSTL_o;
                        burst_cnt <= burst_cnt + 3'd1;
                    end
                    if (cnt == 6'd0) begin
                        state <= SCWREND;
                        cnt   <= CNT_SCWREND;
                        ts_i_o <= 1'b0;
                    end
                end

                SCWREND: begin
                    if (cnt == 6'd0) begin
                        if (from_blw && blk_cnt < sz_reg) begin
                            blk_cnt <= blk_cnt + 2'd1;
                            state <= BLW_ISSUE;
                        end else
                            state <= IDLE;
                    end
                end

                BLR_ACT: begin
                    csbar_o <= 1'b0;
                    rasbar_o <= 1'b0;
                    casbar_o <= 1'b1;
                    webar_o  <= 1'b1;
                    ba_o     <= bank_reg;
                    a_o      <= row_reg;
                    if (cnt == 6'd0) begin
                        // Wait for tRCD before issuing READ command
                        state <= WAIT_TRCD;
                        next_state_after_trcd <= BLR_ISSUE;
                        cnt   <= TRCD_WAIT[5:0];
                    end
                end

                BLR_ISSUE: begin
                    csbar_o <= 1'b0;
                    rasbar_o <= 1'b1;
                    casbar_o <= 1'b0;
                    webar_o  <= 1'b1;
                    ba_o     <= bank_reg;
                    a_o      <= (blk_cnt == sz_reg)
                                   ? {3'b0, col_with_blk, 1'b1}
                                   : {3'b0, col_with_blk, 1'b0};
                    state    <= BLR_READ;
                    cnt      <= CNT_SCREAD;
                end

                BLR_READ: begin
                    csbar_o <= 1'b1;
                    rasbar_o <= 1'b1;
                    casbar_o <= 1'b1;
                    webar_o  <= 1'b1;
                    if (cnt == 6'd0) begin
                        listen_ringBuff_o <= 1'b1;
                        state <= SCRNLSN;
                        cnt   <= CNT_SCRNLSN;
                        readPtr_ringBuff_o <= 3'd0;
                    end
                end

                BLW_ACT: begin
                    csbar_o <= 1'b0;
                    rasbar_o <= 1'b0;
                    casbar_o <= 1'b1;
                    webar_o  <= 1'b1;
                    ba_o     <= bank_reg;
                    a_o      <= row_reg;
                    if (cnt == 6'd0) begin
                        // Wait for tRCD before issuing WRITE command
                        state <= WAIT_TRCD;
                        next_state_after_trcd <= BLW_ISSUE;
                        cnt   <= TRCD_WAIT[5:0];
                    end
                end

                BLW_ISSUE: begin
                    csbar_o <= 1'b0;
                    rasbar_o <= 1'b1;
                    casbar_o <= 1'b0;
                    webar_o  <= 1'b0;
                    ba_o     <= bank_reg;
                    a_o      <= (blk_cnt == sz_reg)
                                   ? {3'b0, col_with_blk, 1'b1}
                                   : {3'b0, col_with_blk, 1'b0};
                    state    <= BLW_WRITE;
                    cnt      <= CNT_SCWRITE;
                end

                BLW_WRITE: begin
                    csbar_o <= 1'b1;
                    rasbar_o <= 1'b1;
                    casbar_o <= 1'b1;
                    webar_o  <= 1'b1;
                    from_blw <= 1'b1;
                    if (cnt == 6'd0) begin
                        state    <= SCWRDATA;
                        cnt      <= cnt_scwrdata_sz;
                        burst_cnt <= 3'd0;
                        ts_i_o   <= 1'b1;
                        get_dataFIFO_o <= 1'b1;
                        dq_SSTL_o <= din_dataFIFO_i;
                        dm_SSTL_o <= 2'b00;
                    end
                end

                RPRE: begin
                    csbar_o <= 1'b0;
                    rasbar_o <= 1'b0;
                    casbar_o <= 1'b1;
                    webar_o  <= 1'b0;
                    a_o      <= PRE_ALL_A;
                    if (cnt == 6'd0) begin
                        state <= RNOP;
                        cnt   <= CNT_RNOP;
                    end
                end

                RNOP: begin
                    csbar_o <= 1'b1;
                    rasbar_o <= 1'b1;
                    casbar_o <= 1'b1;
                    webar_o  <= 1'b1;
                    if (cnt == 6'd0) begin
                        state <= RREF;
                        cnt   <= CNT_RREF;
                    end
                end

                RREF: begin
                    csbar_o <= 1'b0;
                    rasbar_o <= 1'b0;
                    casbar_o <= 1'b0;
                    webar_o  <= 1'b1;
                    if (cnt == 6'd0) begin
                        state <= RIDL;
                        cnt   <= CNT_RIDL;
                    end
                end

                RIDL: begin
                    csbar_o <= 1'b1;
                    rasbar_o <= 1'b1;
                    casbar_o <= 1'b1;
                    webar_o  <= 1'b1;
                    if (cnt == 6'd0)
                        state <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
/* verilator lint_on UNUSEDSIGNAL */
/* verilator lint_on UNUSEDPARAM */
