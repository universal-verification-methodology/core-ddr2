`timescale 1ns/1ps

/**
 * Very simple behavioral DDR2-like memory model.
 * - Watches DDR2 command/address pins.
 * - On ACTIVATE, latches row per bank.
 * - On WRITE, captures an 8-word burst into an internal memory array.
 * - On READ, after a fixed latency, drives an 8-word burst back on DQ/DQS.
 *
 * This is NOT timing-accurate JEDEC DDR2; it is only good enough to
 * exercise the controller at the command/burst level in simulation.
 */
/* verilator lint_off UNUSEDSIGNAL */
module ddr2_simple_mem #(
    parameter MEM_DEPTH = 1024,        // number of 16-bit words per rank in model
    parameter READ_LAT  = 10,          // read latency in CLK cycles
    // Number of rank-selection bits; model supports 2**RANK_BITS independent
    // rank spaces behind a shared command/address bus and a vector of CS#
    // inputs. With RANK_BITS=0 the model behaves like the original
    // single-rank version.
    parameter RANK_BITS = 0
) (
    input  wire         clk,           // controller/system clock
    input  wire         cke_pad,
    // Per-rank chip-select inputs (active low).
    input  wire [(1<<RANK_BITS)-1:0] csbar_pad_vec,
    input  wire         rasbar_pad,
    input  wire         casbar_pad,
    input  wire         webar_pad,
    input  wire [1:0]   ba_pad,
    input  wire [12:0]  a_pad,
    inout  wire [15:0]  dq_pad,
    inout  wire [1:0]   dqs_pad,
    inout  wire [1:0]   dqsbar_pad,
    // Debug / observation interface for closed-loop validation.
    output reg          dbg_wr_valid,
    output reg  [31:0]  dbg_wr_addr,
    output reg  [15:0]  dbg_wr_data,
    output reg          dbg_rd_valid,
    output reg  [31:0]  dbg_rd_addr,
    output reg  [15:0]  dbg_rd_data
);

    localparam integer NUM_RANKS = (1<<RANK_BITS);

    // Simple word-addressable memory. Address space is reduced to MEM_DEPTH
    // using the low bits of the logical address. One independent array per
    // rank.
    reg [15:0] mem [0:NUM_RANKS-1][0:MEM_DEPTH-1];

    // Track active row per bank per rank.
    reg [12:0] active_row [0:NUM_RANKS-1][0:3];

    // Internal decoded command type.
    localparam CMD_NOP    = 3'd0;
    localparam CMD_ACT    = 3'd1;
    localparam CMD_READ   = 3'd2;
    localparam CMD_WRITE  = 3'd3;

    // Simple helpers for address flattening.
    function [31:0] make_word_addr;
        input [12:0] row;
        input [1:0]  bank;
        input [9:0]  col;
        begin
            // Use a compact mapping in the low bits and zero-extend in the
            // high bits so that Verilator sees a width-consistent 32-bit
            // value without implicit expansion.
            // Logical address: {row[4:0], bank[1:0], col[4:0]} => up to 2^12 words.
            make_word_addr = {20'b0, row[4:0], bank[1:0], col[4:0]};
        end
    endfunction

    // Pending WRITE burst state.
    reg        wr_pending;
    reg [RANK_BITS-1:0] wr_rank;
    reg [12:0] wr_row;
    reg [1:0]  wr_bank;
    reg [9:0]  wr_col;
    reg [2:0]  wr_beat;     // 0..7 within burst
    reg [15:0] wr_last_dq;  // last non-Z DQ value seen during burst

    // Pending READ burst state.
    reg        rd_pending;
    reg [RANK_BITS-1:0] rd_rank;
    reg [12:0] rd_row;
    reg [1:0]  rd_bank;
    reg [9:0]  rd_col;
    reg [2:0]  rd_beat;   // 0..7 within burst
    reg [7:0]  rd_lat_cnt;
    reg        rd_drive;  // when 1, drive dq_pad/dqs_pad
    reg [15:0] rd_data_reg;
    reg [1:0]  dqs_drive;

    // ------------------------------------------------------------------------
    // Initialization
    // ------------------------------------------------------------------------
    // In a real DDR2 device, internal state powers up to unknown values until
    // initialization completes. For this very simple behavioral model we
    // explicitly initialize internal state so that WRITE and READ bursts can
    // start cleanly on the first command without X-propagation issues in
    // simulation (e.g. conditions like `if (!wr_pending && cmd_dec == CMD_WRITE)`).
    integer init_i;
    integer init_r;
    initial begin
        // Clear storage array and per-bank active rows for all ranks.
        for (init_r = 0; init_r < NUM_RANKS; init_r = init_r + 1) begin
            for (init_i = 0; init_i < MEM_DEPTH; init_i = init_i + 1) begin
                mem[init_r][init_i] = 16'b0;
            end
            for (init_i = 0; init_i < 4; init_i = init_i + 1) begin
                active_row[init_r][init_i] = 13'b0;
            end
        end

        // Clear WRITE burst tracking.
        wr_pending  = 1'b0;
        wr_rank     = {RANK_BITS{1'b0}};
        wr_row      = 13'b0;
        wr_bank     = 2'b0;
        wr_col      = 10'b0;
        wr_beat     = 3'b0;
        wr_last_dq  = 16'b0;

        // Clear READ burst tracking.
        rd_pending  = 1'b0;
        rd_rank     = {RANK_BITS{1'b0}};
        rd_row      = 13'b0;
        rd_bank     = 2'b0;
        rd_col      = 10'b0;
        rd_beat     = 3'b0;
        rd_lat_cnt  = 8'b0;
        rd_drive    = 1'b0;
        rd_data_reg = 16'b0;
        dqs_drive   = 2'b00;
    end

    // Decode command on each clk edge when CKE is high.
    reg [2:0] cmd_dec;

    // Decode active rank from CS# vector. In well-formed traffic there should
    // be at most one rank selected (CS# low) at a time on this channel. If
    // multiple CS# lines are low simultaneously the lowest-index rank wins.
    function [RANK_BITS-1:0] active_rank;
        input [(1<<RANK_BITS)-1:0] cs_vec;
        integer i;
        begin
            active_rank = {RANK_BITS{1'b0}};
            for (i = 0; i < NUM_RANKS; i = i + 1) begin
                if (!cs_vec[i[RANK_BITS-1:0]])
                    active_rank = i[RANK_BITS-1:0];
            end
        end
    endfunction
    always @(*) begin
        cmd_dec = CMD_NOP;
        if (cke_pad && (csbar_pad_vec != {NUM_RANKS{1'b1}})) begin
            // ACTIVATE: RAS#=0, CAS#=1, WE#=1
            if (!rasbar_pad && casbar_pad && webar_pad)
                cmd_dec = CMD_ACT;
            // READ: RAS#=1, CAS#=0, WE#=1
            else if (rasbar_pad && !casbar_pad && webar_pad)
                cmd_dec = CMD_READ;
            // WRITE: RAS#=1, CAS#=0, WE#=0
            else if (rasbar_pad && !casbar_pad && !webar_pad)
                cmd_dec = CMD_WRITE;
            else
                cmd_dec = CMD_NOP;
        end
    end

    // Main behavioral logic.
    always @(posedge clk) begin
        integer idx;

        // Track ACTIVATE to remember open row for the active rank.
        if (cmd_dec == CMD_ACT) begin
            active_row[active_rank(csbar_pad_vec)][ba_pad] <= a_pad;
        end

        // Debug: observe decoded READ commands.
        if (cmd_dec == CMD_READ) begin
            $display("[%0t] MEM: READ decode rank=%0d bank=%0d row=%0d col=%0d",
                     $time, active_rank(csbar_pad_vec), ba_pad,
                     active_row[active_rank(csbar_pad_vec)][ba_pad],
                     a_pad[9:0]);
        end

        // Default debug flags low unless set later this cycle.
        dbg_wr_valid <= 1'b0;
        dbg_rd_valid <= 1'b0;

        // Start a WRITE burst on WRITE command.
        if (!wr_pending && cmd_dec == CMD_WRITE) begin
            wr_rank    <= active_rank(csbar_pad_vec);
            $display("[%0t] MEM: WRITE cmd received rank=%0d bank=%0d row=%0d col=%0d",
                     $time, wr_rank, ba_pad,
                     active_row[wr_rank][ba_pad], a_pad[9:0]);
            wr_pending <= 1'b1;
            wr_row     <= active_row[wr_rank][ba_pad];
            wr_bank    <= ba_pad;
            wr_col     <= a_pad[9:0];  // use low 10 bits as column
            wr_beat    <= 3'd0;
            wr_last_dq <= 16'b0;
        end

        // Start a READ burst on READ command.
        if (!rd_pending && cmd_dec == CMD_READ) begin
            rd_rank    <= active_rank(csbar_pad_vec);
            $display("[%0t] MEM: READ cmd received rank=%0d bank=%0d row=%0d col=%0d",
                     $time, rd_rank, ba_pad,
                     active_row[rd_rank][ba_pad], a_pad[9:0]);
            rd_pending <= 1'b1;
            rd_row     <= active_row[rd_rank][ba_pad];
            rd_bank    <= ba_pad;
            rd_col     <= a_pad[9:0];
            rd_beat    <= 3'd0;
            rd_lat_cnt <= READ_LAT[7:0];
            rd_drive   <= 1'b0;
            dqs_drive  <= 2'b00;
        end else if (rd_pending) begin
            if (!rd_drive) begin
                // Count down latency before driving.
                if (rd_lat_cnt != 8'd0) begin
                    rd_lat_cnt <= rd_lat_cnt - 8'd1;
                end else begin
                    rd_drive <= 1'b1;
                end
            end else begin
                // Drive one word per clk for 8 beats.
                idx = make_word_addr(rd_row, rd_bank, rd_col + {7'b0, rd_beat});
                rd_data_reg <= mem[rd_rank][idx % MEM_DEPTH];
                $display("[%0t] MEM: READ beat=%0d rank=%0d idx=%0d data=0x%04h",
                         $time, rd_beat, rd_rank, idx % MEM_DEPTH,
                         mem[rd_rank][idx % MEM_DEPTH]);
                // Debug: expose read activity.
                dbg_rd_valid <= 1'b1;
                dbg_rd_addr  <= idx;
                dbg_rd_data  <= mem[rd_rank][idx % MEM_DEPTH];
                dqs_drive   <= ~dqs_drive;
                if (rd_beat == 3'd7) begin
                    rd_pending <= 1'b0;
                    rd_drive   <= 1'b0;
                end
                rd_beat <= rd_beat + 3'd1;
            end
        end

        if (!rd_drive) begin
            dqs_drive <= 2'b00;
        end
    end

    // Capture WRITE data using the system clock. For this simplified model we
    // treat each 8-beat WRITE burst as writing the same 16-bit word to all 8
    // locations. We track the last non-Z DQ value during the burst and, when
    // the 8 beats have completed, we update all 8 words in memory in one shot.
    // Using clk here (instead of both edges of DQS) avoids multiple drivers on
    // the same signals and keeps the behavior deterministic for Verilator.
    always @(posedge clk) begin
        integer idx_base;
        integer beat_i;
        if (wr_pending) begin
            // Track last non-Z DQ value during the burst.
            if (dq_pad !== 16'bzzzz_zzzz_zzzz_zzzz)
                wr_last_dq <= dq_pad;

            // Advance beat counter and commit when the 8-beat burst completes.
            if (wr_beat == 3'd7) begin
                idx_base = make_word_addr(wr_row, wr_bank, wr_col);
                for (beat_i = 0; beat_i < 8; beat_i = beat_i + 1) begin
                    mem[wr_rank][(idx_base + beat_i) % MEM_DEPTH] <= wr_last_dq;
                end
                dbg_wr_valid <= 1'b1;
                dbg_wr_addr  <= idx_base;
                dbg_wr_data  <= wr_last_dq;
                $display("[%0t] MEM: WRITE burst committed base_idx=%0d data=0x%04h",
                         $time, idx_base % MEM_DEPTH, wr_last_dq);
                wr_pending <= 1'b0;
                wr_beat    <= 3'd0;
            end else begin
                wr_beat <= wr_beat + 3'd1;
            end
        end
    end

    // Tri-state drivers for DQ and DQS.
    assign dq_pad     = rd_drive ? rd_data_reg : 16'bz;
    assign dqs_pad    = rd_drive ? dqs_drive   : 2'bzz;
    assign dqsbar_pad = rd_drive ? ~dqs_drive  : 2'bzz;

endmodule
/* verilator lint_on UNUSEDSIGNAL */
