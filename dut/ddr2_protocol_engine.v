`timescale 1ns/1ps

/**
 * DDR2 protocol engine: command decode, ACTIVATE/READ/WRITE/REFRESH sequencing,
 * timing counters, ring buffer and return FIFO control.
 */
/* verilator lint_off UNUSEDPARAM */
/* verilator lint_off UNUSEDSIGNAL */
module ddr2_protocol_engine #(
    parameter AL = 4,
    parameter CL = 4,
    parameter CWL = 3
) (
    input  wire         clk,
    input  wire         reset,
    input  wire         ready_init_i,
    input  wire [24:0]  addr_cmdFIFO_i,
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
    output reg  [24:0]  addr_returnFIFO_o,
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
    output reg          ri_i_o
);

    localparam NOP  = 3'b000;
    localparam SCR  = 3'b001;
    localparam SCW  = 3'b010;
    localparam BLR  = 3'b011;
    localparam BLW  = 3'b100;

    localparam PRE_ALL_A = 13'h400;
    localparam CNT_GETCMD   = 6'd2;
    localparam CNT_ACTIVATE = 6'd1;
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

    localparam STATE_BITS = 5;
    localparam IDLE       = 5'd0,
               GETCMD     = 5'd1,
               ACTIVATE   = 5'd2,
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
    reg [24:0] addr_reg;
    reg [2:0]  cmd_reg;
    reg [1:0]  sz_reg;
    reg [12:0] row_reg;
    reg [1:0]  bank_reg;
    reg [9:0]  col_reg;
    reg [2:0]  burst_cnt;   // 0..7 for read pointer / write words
    reg [1:0]  blk_cnt;     // block burst index for BLR/BLW
    reg        from_blw;    // 1 if SCWRDATA entered from BLW_WRITE
    wire [12:0] row_addr = addr_cmdFIFO_i[24:12];
    wire [1:0]  bank_addr = addr_cmdFIFO_i[4:3];
    wire [9:0]  col_addr = {addr_cmdFIFO_i[11:5], addr_cmdFIFO_i[2:0]};

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
            addr_reg <= 25'b0;
            cmd_reg  <= NOP;
            sz_reg   <= 2'b00;
            row_reg  <= 13'b0;
            bank_reg <= 2'b00;
            col_reg  <= 10'b0;
            burst_cnt <= 3'd0;
            blk_cnt  <= 2'd0;
            from_blw <= 1'b0;
            addr_returnFIFO_o <= 25'b0;
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
                        addr_reg <= addr_cmdFIFO_i;
                        cmd_reg  <= cmd_cmdFIFO_i;
                        sz_reg   <= sz_cmdFIFO_i;
                        $display("[%0t] PROT: GETCMD addr=%0d cmd=%0b sz=%0b",
                                 $time, addr_cmdFIFO_i, cmd_cmdFIFO_i, sz_cmdFIFO_i);
                        row_reg  <= row_addr;
                        bank_reg <= bank_addr;
                        col_reg  <= col_addr;
                        state    <= ACTIVATE;
                        cnt      <= CNT_ACTIVATE;
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
                        if (cmd_reg == SCR) begin
                            state <= ISSUE_READ;
                        end else if (cmd_reg == SCW) begin
                            state <= ISSUE_WRITE;
                        end else if (cmd_reg == BLR) begin
                            if (fillcount_returnFIFO_i < return_thresh) begin
                                state <= BLR_ACT;
                                blk_cnt <= 2'd0;
                                cnt   <= CNT_ACTIVATE;
                            end else
                                state <= IDLE;
                        end else if (cmd_reg == BLW) begin
                            state <= BLW_ACT;
                            blk_cnt <= 2'd0;
                            cnt   <= CNT_ACTIVATE;
                        end else
                            state <= IDLE;
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
                    addr_returnFIFO_o   <= addr_reg + {22'b0, 3'(8 - cnt)};
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
                        state <= BLR_ISSUE;
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
                        state <= BLW_ISSUE;
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
