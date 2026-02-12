`timescale 1ns/1ps

/**
 * DDR2 PHY: muxes init vs protocol command/address; drives DQ/DQS with direction control.
 * No actual SSTL cells; direct drive for simulation/synthesis with external pads.
 */
module ddr2_phy (
    input  wire         clk,
    input  wire         ready,     // 1 = protocol engine drives cmd/addr; 0 = init engine
    // Init engine (when ready=0)
    input  wire         init_csbar,
    input  wire         init_rasbar,
    input  wire         init_casbar,
    input  wire         init_webar,
    input  wire [1:0]   init_ba,
    input  wire [12:0]  init_a,
    input  wire [1:0]   init_dm,
    input  wire         init_odt,
    input  wire         init_cke,
    // Protocol engine (when ready=1)
    input  wire         prot_csbar,
    input  wire         prot_rasbar,
    input  wire         prot_casbar,
    input  wire         prot_webar,
    input  wire [1:0]   prot_ba,
    input  wire [12:0]  prot_a,
    input  wire [1:0]   prot_dm,
    input  wire         prot_odt,
    input  wire [15:0]  prot_dq_o,
    input  wire [1:0]   prot_dqs_o,
    input  wire         ts_i,     // 1 = drive DQ/DQS (write)
    input  wire         ri_i,     // 1 = read phase (high-Z drive)
    // Optional power-management hook: when pm_use_cke_override is high, CKE is
    // driven from pm_cke_value instead of the default (init_cke before READY,
    // constant 1 after READY). This is used by the protocol engine to enter
    // and exit self-refresh by pulling CKE low while holding the bus at NOP.
    input  wire         pm_use_cke_override,
    input  wire         pm_cke_value,
    // To pads
    output wire         ck_pad,
    output wire         ckbar_pad,
    output wire         cke_pad,
    output wire         csbar_pad,
    output wire         rasbar_pad,
    output wire         casbar_pad,
    output wire         webar_pad,
    output wire [1:0]   ba_pad,
    output wire [12:0]  a_pad,
    output wire [1:0]   dm_pad,
    output wire         odt_pad,
    inout  wire [15:0]  dq_pad,
    inout  wire [1:0]   dqs_pad,
    inout  wire [1:0]   dqsbar_pad,
    // Back to controller (read data / DQS input)
    output wire [15:0]  dq_i,     // captured DQ when reading
    output wire [1:0]   dqs_i     // DQS input for ring buffer
);

    reg ck_reg;
    always @(posedge clk) ck_reg <= ~ck_reg;
    assign ck_pad   = ck_reg;
    assign ckbar_pad = ~ck_reg;

    assign cke_pad  = ready
                      ? (pm_use_cke_override ? pm_cke_value : 1'b1)
                      : init_cke;
    assign csbar_pad= ready ? prot_csbar : init_csbar;
    assign rasbar_pad= ready ? prot_rasbar : init_rasbar;
    assign casbar_pad= ready ? prot_casbar : init_casbar;
    assign webar_pad = ready ? prot_webar : init_webar;
    assign ba_pad   = ready ? prot_ba : init_ba;
    assign a_pad    = ready ? prot_a : init_a;
    assign dm_pad   = ready ? prot_dm : init_dm;
    assign odt_pad  = ready ? prot_odt : init_odt;

    // DQ: drive when ts_i (write), else high-Z; for inout we use tri-state in simulation
    assign dq_pad   = (ts_i && !ri_i) ? prot_dq_o : 16'bz;
    assign dq_i     = dq_pad;   // capture when not driving

    assign dqs_pad  = (ts_i && !ri_i) ? prot_dqs_o : 2'bzz;
    assign dqsbar_pad = (ts_i && !ri_i) ? ~prot_dqs_o : 2'bzz;
    assign dqs_i    = dqs_pad;

endmodule
