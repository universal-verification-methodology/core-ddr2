`timescale 1ns/1ps

/**
 * DDR2 "server" controller top-level.
 *
 * This module is intended as a staging point for a more PC/server-style
 * DDR2 controller (multi-rank / larger logical address space, potentially
 * wider data paths) while reusing the existing `ddr2_controller` as the
 * core per-device engine.
 *
 * For now, `ddr2_server_controller` is a thin wrapper around a single
 * `ddr2_controller` instance and exposes the same host and DDR2 interfaces.
 * The intent is that future revisions can:
 *
 *   - Add address decoding (rank / bank / row / column) above the current
 *     25‑bit logical word address.
 *   - Replicate `ddr2_controller` instances to form wider data buses or
 *     multiple ranks/channels.
 *   - Introduce a more PC/server-like front-end (e.g. 64‑bit interface or
 *     AXI‑style bus) while preserving the proven DDR2 timing/PHY logic.
 *
 * Keeping this wrapper separate allows the existing `ddr2_controller`
 * tests and usage to remain unchanged, while experiments with "GB‑class"
 * configurations are done against this new top cell.
 */
/* verilator lint_off UNUSEDSIGNAL */
module ddr2_server_controller #(
    // Host-visible address width. This may be wider than the core
    // `ADDR_WIDTH` once rank/channel bits are introduced. For now it defaults
    // to the same 25-bit space as the underlying controller.
    parameter integer HOST_ADDR_WIDTH = 25,
    // Logical core address width, passed down into `ddr2_controller`.
    parameter integer ADDR_WIDTH      = 25,
    // Number of rank-selection bits in the host address. In future revisions
    // these high-order bits will be decoded into rank CS# lines; at present
    // they are only defined for documentation and future expansion.
    parameter integer RANK_BITS      = 0
) (
    input  wire         CLK,
    input  wire         RESET,
    input  wire         INITDDR,
    // Optional host-driven power-management controls mirrored from the core
    // controller interface so that the server-style wrapper can participate in
    // the same self-refresh / precharge power-down scenarios exercised by the
    // core-top testbench.
    input  wire         SELFREF_REQ,
    input  wire         SELFREF_EXIT,
    input  wire         PWRDOWN_REQ,
    input  wire         PWRDOWN_EXIT,
    input  wire [2:0]   CMD,
    input  wire [1:0]   SZ,
    // Host-visible logical word address. In future we will treat the upper
    // RANK_BITS as a rank index and the lower ADDR_WIDTH bits as the core
    // address passed down to `ddr2_controller`.
    input  wire [HOST_ADDR_WIDTH-1:0]  ADDR,
    input  wire         cmd_put,       // assert one cycle to enqueue command
    // 64-bit host data path formed by four 16-bit DDR2 slices operated in
    // lockstep. Each slice is handled by an underlying `ddr2_controller`
    // instance.
    input  wire [63:0]  DIN,
    input  wire         put_dataFIFO,
    input  wire         FETCHING,
    // Propagate runtime DLL controls down to the core slice; currently only
    // slice 0 exposes DLL status externally, but all slices see the same
    // DLL_REQ / DLL_MODE controls.
    input  wire         DLL_REQ,
    input  wire         DLL_MODE,
    output wire [63:0]  DOUT,
    // Host-visible return address. For now this is just a zero-extended copy
    // of the core controller's RADDR; in multi-rank/channel configurations
    // the upper bits will carry rank/channel indices.
    output wire [HOST_ADDR_WIDTH-1:0]  RADDR,
    output wire [6:0]   FILLCOUNT,
    output wire         READY,
    output wire         VALIDOUT,
    output wire         NOTFULL,
    output wire         SELFREF_ACTIVE,
    output wire         PWRDOWN_ACTIVE,
    output wire         DLL_BUSY,
    // Two-rank chip-select vector on the DDR2 bus (active low).
    output wire [1:0]   C0_CSBAR_PAD,
    output wire         C0_RASBAR_PAD,
    output wire         C0_CASBAR_PAD,
    output wire         C0_WEBAR_PAD,
    output wire [1:0]   C0_BA_PAD,
    output wire [12:0]  C0_A_PAD,
    output wire [1:0]   C0_DM_PAD,
    output wire         C0_ODT_PAD,
    output wire         C0_CK_PAD,
    output wire         C0_CKBAR_PAD,
    output wire         C0_CKE_PAD,
    inout  wire [15:0]  C0_DQ_PAD,
    inout  wire [1:0]   C0_DQS_PAD,
    inout  wire [1:0]   C0_DQSBAR_PAD
`ifdef SIM_DIRECT_READ
    ,
    input  wire         sim_mem_rd_valid,
    input  wire [15:0]  sim_mem_rd_data
`endif
);

    // ------------------------------------------------------------------------
    // Current implementation: single logical region with a direct mapping from
    // host address to core address and a 64-bit host data path implemented as
    // four 16-bit DDR2 slices in lockstep. With HOST_ADDR_WIDTH == ADDR_WIDTH
    // and RANK_BITS == 0 the address behavior is identical to a bare
    // `ddr2_controller`, but the data path has been widened at the host side.
    // ------------------------------------------------------------------------

    // Host-to-core address mapping. For future multi-rank support, the upper
    // RANK_BITS of the host address will encode a logical rank index while the
    // lower ADDR_WIDTH bits carry the core address. Today RANK_BITS defaults
    // to zero so this mapping is effectively a pass-through.
    wire [HOST_ADDR_WIDTH-1:0] host_addr = ADDR;

    // Core address is taken from the least-significant ADDR_WIDTH bits.
    wire [ADDR_WIDTH-1:0] core_addr = host_addr[ADDR_WIDTH-1:0];

    // Placeholder for rank index extraction; not yet used to drive any CS#
    // signals since we only model a single physical device at this stage.
    // This is kept for documentation and future expansion so that the
    // host-visible interface already looks "PC-like" with potential rank bits
    // in the address map.
    // verilator lint_off UNUSED
    wire [ (RANK_BITS>0) ? (RANK_BITS-1) : 0 : 0] rank_index_unused;
    generate
        if (RANK_BITS > 0) begin : gen_rank_index
            assign rank_index_unused = host_addr[HOST_ADDR_WIDTH-1 -: RANK_BITS];
        end else begin : gen_rank_index
            assign rank_index_unused = 1'b0;
        end
    endgenerate
    // verilator lint_on UNUSED

    // Core return addresses and data for four 16-bit slices.
    wire [ADDR_WIDTH-1:0] core_raddr0;
    wire [ADDR_WIDTH-1:0] core_raddr1;
    wire [ADDR_WIDTH-1:0] core_raddr2;
    wire [ADDR_WIDTH-1:0] core_raddr3;
    wire [15:0]           core_dout0;
    wire [15:0]           core_dout1;
    wire [15:0]           core_dout2;
    wire [15:0]           core_dout3;

    // Control/status signals are expected to be identical across slices since
    // they are driven in lockstep. We take them from slice 0.
    wire [6:0]  fillcount0;
    wire        ready0;
    wire        validout0;
    wire        notfull0;
    wire        selfref_active0;
    wire        pdown_active0;

    // Slice 0: fully connected to external DDR2 pads and used as the reference
    // for READY/NOTFULL/VALIDOUT/FILLCOUNT. In SIM_DIRECT_READ mode, this
    // slice also drives the host-visible read data for lane 0.
    ddr2_controller #(
        .ADDR_WIDTH(ADDR_WIDTH)
    ) u_core0 (
        .CLK           (CLK),
        .RESET         (RESET),
        .INITDDR       (INITDDR),
        .SELFREF_REQ   (SELFREF_REQ),
        .SELFREF_EXIT  (SELFREF_EXIT),
        .PWRDOWN_REQ   (PWRDOWN_REQ),
        .PWRDOWN_EXIT  (PWRDOWN_EXIT),
        .CMD           (CMD),
        .SZ            (SZ),
        .ADDR          (core_addr),
        // Use the most-significant host address bit as a simple rank select
        // hint for the core controller/protocol engine. In the current design
        // this is used purely for observability and does not yet drive any
        // multi-CS behavior at the pad level.
        .RANK_SEL      (host_addr[HOST_ADDR_WIDTH-1]),
        .cmd_put       (cmd_put),
        .DIN           (DIN[15:0]),
        .put_dataFIFO  (put_dataFIFO),
        .FETCHING      (FETCHING),
        .DLL_REQ       (DLL_REQ),
        .DLL_MODE      (DLL_MODE),
        .DOUT          (core_dout0),
        .RADDR         (core_raddr0),
        .FILLCOUNT     (fillcount0),
        .READY         (ready0),
        .VALIDOUT      (validout0),
        .NOTFULL       (notfull0),
        .SELFREF_ACTIVE(selfref_active0),
        .PWRDOWN_ACTIVE(pdown_active0),
        .DLL_BUSY      (DLL_BUSY),
        .C0_CSBAR_PAD  (C0_CSBAR_PAD),
        .C0_RASBAR_PAD (C0_RASBAR_PAD),
        .C0_CASBAR_PAD (C0_CASBAR_PAD),
        .C0_WEBAR_PAD  (C0_WEBAR_PAD),
        .C0_BA_PAD     (C0_BA_PAD),
        .C0_A_PAD      (C0_A_PAD),
        .C0_DM_PAD     (C0_DM_PAD),
        .C0_ODT_PAD    (C0_ODT_PAD),
        .C0_CK_PAD     (C0_CK_PAD),
        .C0_CKBAR_PAD  (C0_CKBAR_PAD),
        .C0_CKE_PAD    (C0_CKE_PAD),
        .C0_DQ_PAD     (C0_DQ_PAD),
        .C0_DQS_PAD    (C0_DQS_PAD),
        .C0_DQSBAR_PAD (C0_DQSBAR_PAD)
`ifdef SIM_DIRECT_READ
        ,
        .sim_mem_rd_valid (sim_mem_rd_valid),
        .sim_mem_rd_data  (sim_mem_rd_data)
`endif
    );

    // Slices 1–3: internal-only DDR2 instances. For now their pad-level
    // interfaces are not exposed at the top level; they are intended to model
    // additional x16 devices from a host/data-path perspective when using the
    // simplified SIM_DIRECT_READ mode.
    wire [1:0]  csbar_pad_dummy1, csbar_pad_dummy2, csbar_pad_dummy3;
    wire        rasbar_pad_dummy1, rasbar_pad_dummy2, rasbar_pad_dummy3;
    wire        casbar_pad_dummy1, casbar_pad_dummy2, casbar_pad_dummy3;
    wire        webar_pad_dummy1, webar_pad_dummy2, webar_pad_dummy3;
    wire [1:0]  ba_pad_dummy1, ba_pad_dummy2, ba_pad_dummy3;
    wire [12:0] a_pad_dummy1, a_pad_dummy2, a_pad_dummy3;
    wire [1:0]  dm_pad_dummy1, dm_pad_dummy2, dm_pad_dummy3;
    wire        odt_pad_dummy1, odt_pad_dummy2, odt_pad_dummy3;
    wire        ck_pad_dummy1, ck_pad_dummy2, ck_pad_dummy3;
    wire        ckbar_pad_dummy1, ckbar_pad_dummy2, ckbar_pad_dummy3;
    wire        cke_pad_dummy1, cke_pad_dummy2, cke_pad_dummy3;
    wire [15:0] dq_pad_dummy1, dq_pad_dummy2, dq_pad_dummy3;
    wire [1:0]  dqs_pad_dummy1, dqs_pad_dummy2, dqs_pad_dummy3;
    wire [1:0]  dqsn_pad_dummy1, dqsn_pad_dummy2, dqsn_pad_dummy3;

    ddr2_controller #(
        .ADDR_WIDTH(ADDR_WIDTH)
    ) u_core1 (
        .CLK           (CLK),
        .RESET         (RESET),
        .INITDDR       (INITDDR),
        .SELFREF_REQ   (SELFREF_REQ),
        .SELFREF_EXIT  (SELFREF_EXIT),
        .PWRDOWN_REQ   (PWRDOWN_REQ),
        .PWRDOWN_EXIT  (PWRDOWN_EXIT),
        .CMD           (CMD),
        .SZ            (SZ),
        .ADDR          (core_addr),
        .RANK_SEL      (host_addr[HOST_ADDR_WIDTH-1]),
        .cmd_put       (cmd_put),
        .DIN           (DIN[31:16]),
        .put_dataFIFO  (put_dataFIFO),
        .FETCHING      (FETCHING),
        .DLL_REQ       (DLL_REQ),
        .DLL_MODE      (DLL_MODE),
        .DOUT          (core_dout1),
        .RADDR         (core_raddr1),
        .FILLCOUNT     (/* unused */),
        .READY         (/* unused */),
        .VALIDOUT      (/* unused */),
        .NOTFULL       (/* unused */),
        .DLL_BUSY      (/* unused */),
        .C0_CSBAR_PAD  (csbar_pad_dummy1),
        .C0_RASBAR_PAD (rasbar_pad_dummy1),
        .C0_CASBAR_PAD (casbar_pad_dummy1),
        .C0_WEBAR_PAD  (webar_pad_dummy1),
        .C0_BA_PAD     (ba_pad_dummy1),
        .C0_A_PAD      (a_pad_dummy1),
        .C0_DM_PAD     (dm_pad_dummy1),
        .C0_ODT_PAD    (odt_pad_dummy1),
        .C0_CK_PAD     (ck_pad_dummy1),
        .C0_CKBAR_PAD  (ckbar_pad_dummy1),
        .C0_CKE_PAD    (cke_pad_dummy1),
        .C0_DQ_PAD     (dq_pad_dummy1),
        .C0_DQS_PAD    (dqs_pad_dummy1),
        .C0_DQSBAR_PAD (dqsn_pad_dummy1)
`ifdef SIM_DIRECT_READ
        ,
        .sim_mem_rd_valid (sim_mem_rd_valid),
        .sim_mem_rd_data  (sim_mem_rd_data)
`endif
    );

    ddr2_controller #(
        .ADDR_WIDTH(ADDR_WIDTH)
    ) u_core2 (
        .CLK           (CLK),
        .RESET         (RESET),
        .INITDDR       (INITDDR),
        .SELFREF_REQ   (SELFREF_REQ),
        .SELFREF_EXIT  (SELFREF_EXIT),
        .PWRDOWN_REQ   (PWRDOWN_REQ),
        .PWRDOWN_EXIT  (PWRDOWN_EXIT),
        .CMD           (CMD),
        .SZ            (SZ),
        .ADDR          (core_addr),
        .RANK_SEL      (host_addr[HOST_ADDR_WIDTH-1]),
        .cmd_put       (cmd_put),
        .DIN           (DIN[47:32]),
        .put_dataFIFO  (put_dataFIFO),
        .FETCHING      (FETCHING),
        .DLL_REQ       (DLL_REQ),
        .DLL_MODE      (DLL_MODE),
        .DOUT          (core_dout2),
        .RADDR         (core_raddr2),
        .FILLCOUNT     (/* unused */),
        .READY         (/* unused */),
        .VALIDOUT      (/* unused */),
        .NOTFULL       (/* unused */),
        .DLL_BUSY      (/* unused */),
        .C0_CSBAR_PAD  (csbar_pad_dummy2),
        .C0_RASBAR_PAD (rasbar_pad_dummy2),
        .C0_CASBAR_PAD (casbar_pad_dummy2),
        .C0_WEBAR_PAD  (webar_pad_dummy2),
        .C0_BA_PAD     (ba_pad_dummy2),
        .C0_A_PAD      (a_pad_dummy2),
        .C0_DM_PAD     (dm_pad_dummy2),
        .C0_ODT_PAD    (odt_pad_dummy2),
        .C0_CK_PAD     (ck_pad_dummy2),
        .C0_CKBAR_PAD  (ckbar_pad_dummy2),
        .C0_CKE_PAD    (cke_pad_dummy2),
        .C0_DQ_PAD     (dq_pad_dummy2),
        .C0_DQS_PAD    (dqs_pad_dummy2),
        .C0_DQSBAR_PAD (dqsn_pad_dummy2)
`ifdef SIM_DIRECT_READ
        ,
        .sim_mem_rd_valid (sim_mem_rd_valid),
        .sim_mem_rd_data  (sim_mem_rd_data)
`endif
    );

    ddr2_controller #(
        .ADDR_WIDTH(ADDR_WIDTH)
    ) u_core3 (
        .CLK           (CLK),
        .RESET         (RESET),
        .INITDDR       (INITDDR),
        .SELFREF_REQ   (SELFREF_REQ),
        .SELFREF_EXIT  (SELFREF_EXIT),
        .PWRDOWN_REQ   (PWRDOWN_REQ),
        .PWRDOWN_EXIT  (PWRDOWN_EXIT),
        .CMD           (CMD),
        .SZ            (SZ),
        .ADDR          (core_addr),
        .RANK_SEL      (host_addr[HOST_ADDR_WIDTH-1]),
        .cmd_put       (cmd_put),
        .DIN           (DIN[63:48]),
        .put_dataFIFO  (put_dataFIFO),
        .FETCHING      (FETCHING),
        .DLL_REQ       (DLL_REQ),
        .DLL_MODE      (DLL_MODE),
        .DOUT          (core_dout3),
        .RADDR         (core_raddr3),
        .FILLCOUNT     (/* unused */),
        .READY         (/* unused */),
        .VALIDOUT      (/* unused */),
        .NOTFULL       (/* unused */),
        .DLL_BUSY      (/* unused */),
        .C0_CSBAR_PAD  (csbar_pad_dummy3),
        .C0_RASBAR_PAD (rasbar_pad_dummy3),
        .C0_CASBAR_PAD (casbar_pad_dummy3),
        .C0_WEBAR_PAD  (webar_pad_dummy3),
        .C0_BA_PAD     (ba_pad_dummy3),
        .C0_A_PAD      (a_pad_dummy3),
        .C0_DM_PAD     (dm_pad_dummy3),
        .C0_ODT_PAD    (odt_pad_dummy3),
        .C0_CK_PAD     (ck_pad_dummy3),
        .C0_CKBAR_PAD  (ckbar_pad_dummy3),
        .C0_CKE_PAD    (cke_pad_dummy3),
        .C0_DQ_PAD     (dq_pad_dummy3),
        .C0_DQS_PAD    (dqs_pad_dummy3),
        .C0_DQSBAR_PAD (dqsn_pad_dummy3)
`ifdef SIM_DIRECT_READ
        ,
        .sim_mem_rd_valid (sim_mem_rd_valid),
        .sim_mem_rd_data  (sim_mem_rd_data)
`endif
    );

    // Zero-extend slice 0's core return address into the host-visible space
    // for now. Once rank/channel bits are actively used, this mapping will be
    // extended accordingly.
    assign RADDR = { {(HOST_ADDR_WIDTH-ADDR_WIDTH){1'b0}}, core_raddr0 };

    // Aggregate 4×16-bit slice data into the 64-bit host bus.
    assign DOUT      = {core_dout3, core_dout2, core_dout1, core_dout0};
    assign FILLCOUNT      = fillcount0;
    assign READY          = ready0;
    assign VALIDOUT       = validout0;
    assign NOTFULL        = notfull0;
    assign SELFREF_ACTIVE = selfref_active0;
    assign PWRDOWN_ACTIVE = pdown_active0;

endmodule
/* verilator lint_on UNUSEDSIGNAL */

