`timescale 1ns/1ps

/**
 * DDR2 JEDEC initialization engine.
 * Performs power-up wait, CKE assertion, precharge all, MRS/EMRS programming,
 * DLL reset, optional DLL-on / DLL-off configuration, optional OCD-related
 * EMRS1 programming (via configurable EMRS1 values), auto refresh, and asserts
 * ready when done.
 *
 * The exact mode-register and extended-mode-register encodings are fully
 * parameterized so that this engine can be targeted at different commercial
 * DDR2 parts by overriding bitfields and timing constants rather than changing
 * the FSM structure. In particular:
 *   - MRS DLL reset / DLL-on / DLL-off encodings can be tailored per device.
 *   - EMRS1_INIT_VAL / EMRS1_FINAL_VAL can encode OCD / ODT / drive-strength
 *     sequences as required by the chosen vendor datasheet.
 */
/* verilator lint_off UNUSEDPARAM */
module ddr2_init_engine #(
    parameter BL = 3'b011,     // Burst length 8
    parameter BT = 1'b0,       // Sequential burst type
    parameter CL = 3'b100,     // CAS latency 4
    parameter AL = 3'b100,     // Additive latency 4
    // Mode register encodings:
    //   - MRS_DLL_RST_VAL : MRS value used when issuing the initial DLL reset.
    //   - MRS_DLL_ON_VAL  : MRS value used for the final "DLL on" configuration.
    //   - MRS_DLL_OFF_VAL : Optional alternate value for a "DLL off" profile.
    //     The engine will choose between DLL-on and DLL-off using DLL_INIT_MODE.
    parameter [12:0] MRS_DLL_RST_VAL = 13'h0413,
    parameter [12:0] MRS_DLL_ON_VAL  = 13'h0013,
    parameter [12:0] MRS_DLL_OFF_VAL = 13'h0013,
    // When set to 1, the engine uses MRS_DLL_OFF_VAL for the final MRS write
    // instead of MRS_DLL_ON_VAL. This allows simple bring-up in DLL-off mode
    // for parts that support that feature. The default (0) preserves the
    // original DLL-on behavior.
    parameter        DLL_INIT_MODE   = 1'b0,
    // Extended mode register 1 encodings used during initialization. These are
    // exposed as parameters so that board- or device-specific ODT / drive
    // strength / OCD settings can be adjusted without touching the FSM:
    //   - EMRS1_INIT_VAL: initial EMRS1 write (e.g. DLL enable, ODT off).
    //   - EMRS1_FINAL_VAL: final EMRS1 write (e.g. ODT on, OCD default).
    // For OCD/ZQ-related sequences, system integration can choose values that
    // implement the vendor-prescribed EMRS1 patterns while reusing this FSM.
    parameter [12:0] EMRS1_INIT_VAL  = 13'h600,
    parameter [12:0] EMRS1_FINAL_VAL = 13'h640,
    // Optional explicit OCD calibration wait and ZQ calibration wait. When
    // OCD_CALIB_EN is non-zero, an additional NOP window of OCD_WAIT_CYCLES
    // is inserted after the final EMRS1 write before the last PREALL. When
    // ZQ_CALIB_EN is non-zero, an additional NOP window of ZQ_WAIT_CYCLES is
    // inserted after the final PREALL before READY. These waits are generic
    // and can be tuned per device to cover tOCD and tZQinit without modeling
    // analog behavior.
    parameter        OCD_CALIB_EN    = 1'b0,
    parameter [16:0] OCD_WAIT_CYCLES = 17'd0,
    parameter        ZQ_CALIB_EN     = 1'b0,
    parameter [16:0] ZQ_WAIT_CYCLES  = 17'd0,
    // Timing parameters (all in controller clocks) exposed for speed-grade
    // configurability. Defaults preserve the current JEDEC-like behavior for a
    // 500 MHz controller clock:
    //   - TXSR_CYCLES    : tXSR wait after CKE assertion.
    //   - TRP_CYCLES     : tRP between PREALL and ACT.
    //   - TMRD_CYCLES    : tMRD between load-mode commands.
    //   - TRFC_CYCLES    : tRFC between AUTO REFRESH commands.
    //   - TMRD_DLL_CYCLES: extended DLL-lock wait window after final MRS.
    //   - FINAL_CYCLES   : small guard window before READY.
    parameter [16:0] TXSR_CYCLES     = 17'h00C7,   // 199
    parameter [16:0] TRP_CYCLES      = 17'h0007,   // 7
    parameter [16:0] TMRD_CYCLES     = 17'h0003,   // 3
    parameter [16:0] TRFC_CYCLES     = 17'h0064,   // 100
    parameter [16:0] TMRD_DLL_CYCLES = 17'h0195,   // 405
    parameter [16:0] FINAL_CYCLES    = 17'h0004
) (
    input  wire        clk,
    input  wire        reset,
    input  wire        init,
    output reg         ready,
    output reg         csbar,
    output reg         rasbar,
    output reg         casbar,
    output reg         webar,
    output reg  [1:0]  ba,
    output reg  [12:0] a,
    output reg  [1:0]  dm,
    output reg         odt,
    output reg         ts_con,
    output reg         cke
);
/* verilator lint_on UNUSEDPARAM */

    // Counter value constants (CLK cycles at 500 MHz)
`ifdef SIM_SHORT_INIT
    localparam [16:0] CNT_PWRUP   = 17'd100;      // short for sim
`else
    localparam [16:0] CNT_PWRUP   = 17'h186A0;   // 100,000 - 200 us
`endif
    localparam [16:0] CNT_TXSR    = TXSR_CYCLES;
    localparam [16:0] CNT_PRE     = 17'h0001;
    localparam [16:0] CNT_TRP     = TRP_CYCLES;
    localparam [16:0] CNT_TMRD    = TMRD_CYCLES;
    localparam [16:0] CNT_TRFC    = TRFC_CYCLES;      // JEDEC: 50 DDR2 cycles = 100 controller cycles = 200 ns
    localparam [16:0] CNT_TMRD_DLL = TMRD_DLL_CYCLES;
    localparam [16:0] CNT_FINAL    = FINAL_CYCLES;
    localparam [16:0] CNT_OCD_WAIT = OCD_WAIT_CYCLES;
    localparam [16:0] CNT_ZQ_WAIT  = ZQ_WAIT_CYCLES;

    // MRS: BL=8, BT=0, CL=4, DLL reset (init) / DLL-on or DLL-off (final)
    localparam [12:0] MRS_DLL_RST = MRS_DLL_RST_VAL;  // DLL reset encoding
    localparam [12:0] MRS_DLL_ON  = MRS_DLL_ON_VAL;
    localparam [12:0] MRS_DLL_OFF = MRS_DLL_OFF_VAL;
    // Choose the final MRS value based on DLL_INIT_MODE so that systems can
    // bring the memory up in either DLL-on (default) or DLL-off configuration.
    localparam [12:0] MRS_FINAL   = (DLL_INIT_MODE != 1'b0) ? MRS_DLL_OFF
                                                            : MRS_DLL_ON;
    // EMRS1: AL=4, DLL enable, ODT/drive strength per parameters above.
    // EMRS1_INIT_VAL and EMRS1_FINAL_VAL are module parameters so that
    // different ODT/drive configurations (and future OCD sequences) can be
    // selected at instantiation time.
    localparam [12:0] EMRS1_INIT  = EMRS1_INIT_VAL;
    localparam [12:0] EMRS1_FINAL = EMRS1_FINAL_VAL;  // typically ODT on
    localparam [12:0] EMRS2       = 13'b0;
    localparam [12:0] EMRS3       = 13'b0;
    localparam [12:0] PRE_ALL    = 13'h400;  // A10=1 precharge all

    localparam STATE_BITS = 5;
    localparam S_IDLE      = 5'd0,
               S_PWRUP     = 5'd1,   // 200 us NOP, CKE=0
               S_CKE       = 5'd2,   // CKE=1, wait tXSR
               S_PRE       = 5'd3,   // PRECHARGE ALL
               S_TRP       = 5'd4,   // wait tRP
               S_EMRS2     = 5'd5,
               S_EMRS2_W   = 5'd6,
               S_EMRS3     = 5'd7,
               S_EMRS3_W   = 5'd8,
               S_EMRS1     = 5'd9,
               S_EMRS1_W   = 5'd10,
               S_MRS       = 5'd11,  // MRS with DLL reset
               S_MRS_W     = 5'd12,
               S_REF1      = 5'd13,  // first AUTO REFRESH
               S_REF1_W    = 5'd14,
               S_REF2      = 5'd15,
               S_REF2_W    = 5'd16,
               S_MRS2      = 5'd17,  // MRS no DLL reset
               S_MRS2_W    = 5'd18,
               S_EMRS1_2   = 5'd19,  // EMRS1 with ODT / final OCD state
               S_EMRS1_2W  = 5'd20,
               S_FINAL_PRE = 5'd21,
               S_FINAL_W   = 5'd22,
               S_READY     = 5'd23,
               S_OCD_WAIT  = 5'd24,
               S_ZQ_WAIT   = 5'd25;

    reg [STATE_BITS-1:0] state, state_next;
    reg [16:0] cntr;
    wire       cntr_done = (cntr == 17'd0);

    always @(posedge clk) begin
        if (reset) begin
            state <= S_IDLE;
            ready <= 1'b0;
            cke   <= 1'b0;
            csbar <= 1'b1;
            rasbar <= 1'b1;
            casbar <= 1'b1;
            webar  <= 1'b1;
            ba     <= 2'b00;
            a      <= 13'b0;
            dm     <= 2'b00;
            odt    <= 1'b0;
            ts_con <= 1'b1;  // tristate / idle during init
            cntr   <= 17'd0;
        end else begin
            state <= state_next;
            if (!cntr_done)
                cntr <= cntr - 17'd1;

            case (state)
                S_IDLE: begin
                    ready <= 1'b0;
                    cke   <= 1'b0;
                    csbar <= 1'b1;
                    rasbar <= 1'b1;
                    casbar <= 1'b1;
                    webar  <= 1'b1;
                    if (init) begin
                        cntr <= CNT_PWRUP;
                    end
                end
                S_PWRUP: begin
                    cke <= 1'b0;
                    csbar <= 1'b1;
                    rasbar <= 1'b1;
                    casbar <= 1'b1;
                    webar  <= 1'b1;
                    if (cntr_done)
                        cntr <= CNT_TXSR;
                end
                S_CKE: begin
                    cke <= 1'b1;
                    csbar <= 1'b1;
                    rasbar <= 1'b1;
                    casbar <= 1'b1;
                    webar  <= 1'b1;
                    if (cntr_done)
                        cntr <= CNT_PRE;
                end
                S_PRE: begin
                    cke <= 1'b1;
                    csbar <= 1'b0;
                    rasbar <= 1'b0;
                    casbar <= 1'b1;
                    webar  <= 1'b0;
                    a      <= PRE_ALL;
                    if (cntr_done)
                        cntr <= CNT_TRP;
                end
                S_TRP: begin
                    cke <= 1'b1;
                    csbar <= 1'b1;
                    rasbar <= 1'b1;
                    casbar <= 1'b1;
                    webar  <= 1'b1;
                    if (cntr_done)
                        cntr <= CNT_TMRD;
                end
                S_EMRS2: begin
                    cke <= 1'b1;
                    csbar <= 1'b0;
                    rasbar <= 1'b0;
                    casbar <= 1'b0;
                    webar  <= 1'b0;
                    a      <= EMRS2;
                    ba     <= 2'b10;  // EMRS2
                    if (cntr_done)
                        cntr <= CNT_TMRD;
                end
                S_EMRS2_W: begin
                    cke <= 1'b1;
                    csbar <= 1'b1;
                    rasbar <= 1'b1;
                    casbar <= 1'b1;
                    webar  <= 1'b1;
                    if (cntr_done)
                        cntr <= CNT_TMRD;
                end
                S_EMRS3: begin
                    cke <= 1'b1;
                    csbar <= 1'b0;
                    rasbar <= 1'b0;
                    casbar <= 1'b0;
                    webar  <= 1'b0;
                    a      <= EMRS3;
                    ba     <= 2'b11;  // EMRS3
                    if (cntr_done)
                        cntr <= CNT_TMRD;
                end
                S_EMRS3_W: begin
                    cke <= 1'b1;
                    csbar <= 1'b1;
                    rasbar <= 1'b1;
                    casbar <= 1'b1;
                    webar  <= 1'b1;
                    if (cntr_done)
                        cntr <= CNT_TMRD;
                end
                S_EMRS1: begin
                    cke <= 1'b1;
                    csbar <= 1'b0;
                    rasbar <= 1'b0;
                    casbar <= 1'b0;
                    webar  <= 1'b0;
                    a      <= EMRS1_INIT;
                    ba     <= 2'b01;  // EMRS1
                    if (cntr_done)
                        cntr <= CNT_TMRD;
                end
                S_EMRS1_W: begin
                    cke <= 1'b1;
                    csbar <= 1'b1;
                    rasbar <= 1'b1;
                    casbar <= 1'b1;
                    webar  <= 1'b1;
                    if (cntr_done)
                        cntr <= CNT_TMRD;
                end
                S_MRS: begin
                    cke <= 1'b1;
                    csbar <= 1'b0;
                    rasbar <= 1'b0;
                    casbar <= 1'b0;
                    webar  <= 1'b0;
                    a      <= MRS_DLL_RST;
                    ba     <= 2'b00;
                    if (cntr_done)
                        cntr <= CNT_TMRD;
                end
                S_MRS_W: begin
                    cke <= 1'b1;
                    csbar <= 1'b1;
                    rasbar <= 1'b1;
                    casbar <= 1'b1;
                    webar  <= 1'b1;
                    if (cntr_done)
                        cntr <= CNT_TRFC;
                end
                S_REF1: begin
                    cke <= 1'b1;
                    csbar <= 1'b0;
                    rasbar <= 1'b0;
                    casbar <= 1'b0;
                    webar  <= 1'b1;
                    if (cntr_done)
                        cntr <= CNT_TRFC;
                end
                S_REF1_W: begin
                    cke <= 1'b1;
                    csbar <= 1'b1;
                    rasbar <= 1'b1;
                    casbar <= 1'b1;
                    webar  <= 1'b1;
                    if (cntr_done)
                        cntr <= CNT_TRFC;
                end
                S_REF2: begin
                    cke <= 1'b1;
                    csbar <= 1'b0;
                    rasbar <= 1'b0;
                    casbar <= 1'b0;
                    webar  <= 1'b1;
                    if (cntr_done)
                        cntr <= CNT_TRFC;
                end
                S_REF2_W: begin
                    cke <= 1'b1;
                    csbar <= 1'b1;
                    rasbar <= 1'b1;
                    casbar <= 1'b1;
                    webar  <= 1'b1;
                    if (cntr_done)
                        cntr <= CNT_TMRD_DLL;
                end
                S_MRS2: begin
                    cke <= 1'b1;
                    csbar <= 1'b0;
                    rasbar <= 1'b0;
                    casbar <= 1'b0;
                    webar  <= 1'b0;
                    a      <= MRS_FINAL;
                    ba     <= 2'b00;
                    if (cntr_done)
                        cntr <= CNT_TMRD;
                end
                S_MRS2_W: begin
                    cke <= 1'b1;
                    csbar <= 1'b1;
                    rasbar <= 1'b1;
                    casbar <= 1'b1;
                    webar  <= 1'b1;
                    if (cntr_done)
                        cntr <= CNT_TMRD;
                end
                S_EMRS1_2: begin
                    cke <= 1'b1;
                    csbar <= 1'b0;
                    rasbar <= 1'b0;
                    casbar <= 1'b0;
                    webar  <= 1'b0;
                    a      <= EMRS1_FINAL;
                    ba     <= 2'b01;
                    if (cntr_done)
                        cntr <= CNT_TMRD;
                end
                S_EMRS1_2W: begin
                    cke <= 1'b1;
                    csbar <= 1'b1;
                    rasbar <= 1'b1;
                    casbar <= 1'b1;
                    webar  <= 1'b1;
                    if (cntr_done) begin
                        if (OCD_CALIB_EN && CNT_OCD_WAIT != 17'd0)
                            cntr <= CNT_OCD_WAIT;
                        else
                            cntr <= CNT_FINAL;
                    end
                end
                S_FINAL_PRE: begin
                    cke <= 1'b1;
                    csbar <= 1'b0;
                    rasbar <= 1'b0;
                    casbar <= 1'b1;
                    webar  <= 1'b0;
                    a      <= PRE_ALL;
                    odt    <= 1'b1;
                    if (cntr_done) begin
                        if (ZQ_CALIB_EN && CNT_ZQ_WAIT != 17'd0)
                            cntr <= CNT_ZQ_WAIT;
                        else
                            cntr <= CNT_FINAL;
                    end
                end
                S_FINAL_W: begin
                    cke <= 1'b1;
                    csbar <= 1'b1;
                    rasbar <= 1'b1;
                    casbar <= 1'b1;
                    webar  <= 1'b1;
                    odt    <= 1'b1;
                    ts_con <= 1'b0;  // release to protocol engine
                    if (cntr_done)
                        ;
                end
                S_OCD_WAIT: begin
                    // Generic NOP wait window to cover OCD calibration
                    // stabilization time after the final EMRS1 write. No
                    // additional commands are issued; the exact EMRS1 bitfields
                    // are controlled by EMRS1_* parameters.
                    cke <= 1'b1;
                    csbar <= 1'b1;
                    rasbar <= 1'b1;
                    casbar <= 1'b1;
                    webar  <= 1'b1;
                    if (cntr_done)
                        cntr <= CNT_FINAL;
                end
                S_ZQ_WAIT: begin
                    // Generic NOP wait window to cover ZQ calibration /
                    // impedance calibration time after the final PREALL.
                    cke <= 1'b1;
                    csbar <= 1'b1;
                    rasbar <= 1'b1;
                    casbar <= 1'b1;
                    webar  <= 1'b1;
                    if (cntr_done)
                        cntr <= CNT_FINAL;
                end
                S_READY: begin
                    ready <= 1'b1;
                    cke   <= 1'b1;
                    csbar <= 1'b1;
                    rasbar <= 1'b1;
                    casbar <= 1'b1;
                    webar  <= 1'b1;
                    odt    <= 1'b1;
                    ts_con <= 1'b0;
                end
                default: begin
                    csbar <= 1'b1;
                    rasbar <= 1'b1;
                    casbar <= 1'b1;
                    webar  <= 1'b1;
                end
            endcase
        end
    end

    always @(*) begin
        state_next = state;
        case (state)
            S_IDLE:      state_next = init ? S_PWRUP : S_IDLE;
            S_PWRUP:     state_next = cntr_done ? S_CKE : S_PWRUP;
            S_CKE:       state_next = cntr_done ? S_PRE : S_CKE;
            S_PRE:       state_next = S_TRP;
            S_TRP:       state_next = cntr_done ? S_EMRS2 : S_TRP;
            S_EMRS2:     state_next = S_EMRS2_W;
            S_EMRS2_W:   state_next = cntr_done ? S_EMRS3 : S_EMRS2_W;
            S_EMRS3:     state_next = S_EMRS3_W;
            S_EMRS3_W:   state_next = cntr_done ? S_EMRS1 : S_EMRS3_W;
            S_EMRS1:     state_next = S_EMRS1_W;
            S_EMRS1_W:   state_next = cntr_done ? S_MRS : S_EMRS1_W;
            S_MRS:       state_next = S_MRS_W;
            S_MRS_W:     state_next = cntr_done ? S_REF1 : S_MRS_W;
            S_REF1:      state_next = S_REF1_W;
            S_REF1_W:    state_next = cntr_done ? S_REF2 : S_REF1_W;
            S_REF2:      state_next = S_REF2_W;
            S_REF2_W:    state_next = cntr_done ? S_MRS2 : S_REF2_W;
            S_MRS2:      state_next = S_MRS2_W;
            S_MRS2_W:    state_next = cntr_done ? S_EMRS1_2 : S_MRS2_W;
            S_EMRS1_2:   state_next = S_EMRS1_2W;
            S_EMRS1_2W:  state_next = cntr_done
                                       ? ((OCD_CALIB_EN && CNT_OCD_WAIT != 17'd0)
                                              ? S_OCD_WAIT
                                              : S_FINAL_PRE)
                                       : S_EMRS1_2W;
            S_OCD_WAIT:  state_next = cntr_done ? S_FINAL_PRE : S_OCD_WAIT;
            S_FINAL_PRE: state_next = (ZQ_CALIB_EN && CNT_ZQ_WAIT != 17'd0)
                                       ? S_ZQ_WAIT
                                       : S_FINAL_W;
            S_ZQ_WAIT:   state_next = cntr_done ? S_FINAL_W : S_ZQ_WAIT;
            S_FINAL_W:   state_next = cntr_done ? S_READY : S_FINAL_W;
            S_READY:     state_next = S_READY;
            default:     state_next = S_IDLE;
        endcase
    end

endmodule
/* verilator lint_on UNUSEDPARAM */
