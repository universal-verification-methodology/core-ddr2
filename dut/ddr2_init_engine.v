`timescale 1ns/1ps

/**
 * DDR2 JEDEC initialization engine.
 * Performs power-up wait, CKE assertion, precharge all, MRS/EMRS programming,
 * DLL reset, auto refresh, and asserts ready when done.
 */
/* verilator lint_off UNUSEDPARAM */
module ddr2_init_engine #(
    parameter BL = 3'b011,   // Burst length 8
    parameter BT = 1'b0,      // Sequential burst type
    parameter CL = 3'b100,    // CAS latency 4
    parameter AL = 3'b100     // Additive latency 4
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
    localparam [16:0] CNT_TXSR    = 17'h00C7;    // 199
    localparam [16:0] CNT_PRE     = 17'h0001;
    localparam [16:0] CNT_TRP     = 17'h0007;   // 7
    localparam [16:0] CNT_TMRD    = 17'h0003;   // 3
    localparam [16:0] CNT_TRFC    = 17'h018F;   // 399
    localparam [16:0] CNT_TMRD_DLL= 17'h0195;   // 405
    localparam [16:0] CNT_FINAL  = 17'h0004;

    // MRS: BL=8, BT=0, CL=4, DLL reset (init) / no reset (final)
    localparam [12:0] MRS_DLL_RST = 13'h0413;  // DLL reset
    localparam [12:0] MRS_FINAL   = 13'h0013;   // no DLL reset
    // EMRS1: AL=4, DLL enable, ODT off (init) / ODT on (final)
    localparam [12:0] EMRS1_INIT  = 13'h600;
    localparam [12:0] EMRS1_FINAL = 13'h640;  // ODT on
    localparam [12:0] EMRS2       = 13'b0;
    localparam [12:0] EMRS3       = 13'b0;
    localparam [12:0] PRE_ALL    = 13'h400;  // A10=1 precharge all

    localparam STATE_BITS = 5;
    localparam S_IDLE     = 5'd0,
               S_PWRUP    = 5'd1,   // 200 us NOP, CKE=0
               S_CKE      = 5'd2,   // CKE=1, wait tXSR
               S_PRE      = 5'd3,   // PRECHARGE ALL
               S_TRP      = 5'd4,   // wait tRP
               S_EMRS2    = 5'd5,
               S_EMRS2_W  = 5'd6,
               S_EMRS3    = 5'd7,
               S_EMRS3_W  = 5'd8,
               S_EMRS1    = 5'd9,
               S_EMRS1_W  = 5'd10,
               S_MRS      = 5'd11,  // MRS with DLL reset
               S_MRS_W    = 5'd12,
               S_REF1     = 5'd13,  // first AUTO REFRESH
               S_REF1_W   = 5'd14,
               S_REF2     = 5'd15,
               S_REF2_W   = 5'd16,
               S_MRS2     = 5'd17,  // MRS no DLL reset
               S_MRS2_W   = 5'd18,
               S_EMRS1_2  = 5'd19,  // EMRS1 with ODT
               S_EMRS1_2W = 5'd20,
               S_FINAL_PRE= 5'd21,
               S_FINAL_W  = 5'd22,
               S_READY    = 5'd23;

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
                    if (cntr_done)
                        cntr <= CNT_FINAL;
                end
                S_FINAL_PRE: begin
                    cke <= 1'b1;
                    csbar <= 1'b0;
                    rasbar <= 1'b0;
                    casbar <= 1'b1;
                    webar  <= 1'b0;
                    a      <= PRE_ALL;
                    odt    <= 1'b1;
                    if (cntr_done)
                        cntr <= CNT_FINAL;
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
            S_EMRS1_2W:  state_next = cntr_done ? S_FINAL_PRE : S_EMRS1_2W;
            S_FINAL_PRE: state_next = S_FINAL_W;
            S_FINAL_W:   state_next = cntr_done ? S_READY : S_FINAL_W;
            S_READY:     state_next = S_READY;
            default:     state_next = S_IDLE;
        endcase
    end

endmodule
/* verilator lint_on UNUSEDPARAM */
