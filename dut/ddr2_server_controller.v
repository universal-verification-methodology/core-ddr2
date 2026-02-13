`timescale 1ns/1ps

/**
 * DDR2 "server" controller top-level with ECC, scrubbing, and RAS features.
 *
 * This module extends the basic DDR2 controller with server-grade features:
 *   - ECC (Error Correction Code): SECDED encoding/decoding for 64-bit data
 *   - Memory Scrubbing: Background read-verify-correct cycles
 *   - RAS (Reliability, Availability, Serviceability): Error tracking and reporting
 *
 * The controller maintains backward compatibility with the basic interface while
 * adding ECC protection transparently to the host.
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
    // ECC/Scrubbing/RAS control inputs
    input  wire         ECC_ENABLE,          // Enable ECC encoding/decoding
    input  wire         SCRUB_ENABLE,         // Enable memory scrubbing
    input  wire [7:0]   RAS_REG_ADDR,          // RAS register read address
    output wire [31:0]  RAS_REG_DATA,         // RAS register data output
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
    // ECC/Scrubbing/RAS status outputs
    output wire         ECC_SINGLE_ERR,       // Single-bit error detected and corrected
    output wire         ECC_DOUBLE_ERR,       // Double-bit error detected (uncorrectable)
    output wire         SCRUB_ACTIVE,         // Scrubbing is active
    output wire         RAS_IRQ_CORR,         // Interrupt: correctable error threshold
    output wire         RAS_IRQ_UNCORR,       // Interrupt: uncorrectable error
    output wire         RAS_RANK_DEGRADED,     // Rank degradation status
    output wire         RAS_FATAL_ERROR,       // Fatal error status
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
    // ECC, Scrubbing, and RAS Integration
    // ------------------------------------------------------------------------
    
    // ECC encoder/decoder instances
    wire [7:0]  ecc_code_write;
    wire [7:0]  ecc_code_read;
    // Raw outputs from the ECC decoder before gating by validity/enable.
    wire [63:0] data_corrected_raw;
    wire        ecc_single_err_raw;
    wire        ecc_double_err_raw;
    wire [7:0]  ecc_syndrome_int;
    // Gated/filtered ECC status used by the rest of the design.
    wire        ecc_single_err_int;
    wire        ecc_double_err_int;
    // Final corrected data word after applying ECC where appropriate.
    wire [63:0] data_corrected;
    
    // ECC storage (simple register array for ECC bits per address)
    // In a real implementation, this would be stored alongside data in memory.
    // Here we track a small shadow RAM of ECC codes plus a "valid" bit so that
    // locations which have never been written do not spuriously flag errors.
    reg [7:0] ecc_storage [0:1023];   // Store ECC for up to 1024 addresses
    reg       ecc_valid   [0:1023];   // ECC entry is valid for this address
    reg [ADDR_WIDTH-1:0] last_write_addr;
    reg [ADDR_WIDTH-1:0] last_read_addr;
    integer               ecc_init_idx;
    
    // ECC encoder (for writes)
    ecc_secded #(
        .DATA_WIDTH (64),
        .PARITY_BITS(7)
    ) u_ecc_encoder (
        .data_in       (DIN),
        .ecc_out       (ecc_code_write),
        .data_check    ({64{1'b0}}),
        .ecc_check     ({8{1'b0}}),
        .data_corrected(),
        .single_err    (),
        .double_err    (),
        .syndrome      ()
    );
    
    // ECC decoder (for reads)
    wire [63:0] read_data_64 = {core_dout3, core_dout2, core_dout1, core_dout0};
    wire [7:0]  read_ecc_code;
    
    ecc_secded #(
        .DATA_WIDTH (64),
        .PARITY_BITS(7)
    ) u_ecc_decoder (
        .data_in       ({64{1'b0}}),
        .ecc_out       (),
        .data_check    (read_data_64),
        .ecc_check     (read_ecc_code),
        .data_corrected(data_corrected_raw),
        .single_err    (ecc_single_err_raw),
        .double_err    (ecc_double_err_raw),
        .syndrome      (ecc_syndrome_int)
    );
    
    // ECC storage management
    always @(posedge CLK) begin
        if (RESET) begin
            last_write_addr <= {ADDR_WIDTH{1'b0}};
            last_read_addr  <= {ADDR_WIDTH{1'b0}};
            // Clear shadow ECC RAM and validity flags so that no spurious
            // X-propagation or false error reporting occurs on first use.
            for (ecc_init_idx = 0; ecc_init_idx < 1024; ecc_init_idx = ecc_init_idx + 1) begin
                ecc_storage[ecc_init_idx] <= 8'd0;
                ecc_valid[ecc_init_idx]   <= 1'b0;
            end
        end else begin
            // Store ECC on writes (when command is issued)
            if (arb_cmd_put && (arb_cmd == 3'b010 || arb_cmd == 3'b100) && ECC_ENABLE) begin
                if (arb_addr < 1024) begin
                    ecc_storage[arb_addr[9:0]] <= ecc_code_write;
                    ecc_valid[arb_addr[9:0]]   <= 1'b1;
                end
                last_write_addr <= arb_addr;
            end
            
            // Retrieve ECC on reads
            if (validout0 && ECC_ENABLE) begin
                last_read_addr <= core_raddr0;
            end
        end
    end
    
    // Only treat an ECC entry as meaningful if it is within the tracked range
    // and has been explicitly written at least once.
    wire addr_has_ecc = (core_raddr0 < 1024) && ecc_valid[core_raddr0[9:0]];
    
    assign read_ecc_code = addr_has_ecc ? ecc_storage[core_raddr0[9:0]] : 8'd0;
    
    // Gate raw ECC status by address validity and global enable so that
    // never-written locations (or ECC disabled) do not produce spurious
    // correctable/uncorrectable error indications.
    assign ecc_single_err_int = (ECC_ENABLE && addr_has_ecc) ? ecc_single_err_raw  : 1'b0;
    assign ecc_double_err_int = (ECC_ENABLE && addr_has_ecc) ? ecc_double_err_raw  : 1'b0;
    
    // Select corrected data only when we have a valid ECC entry and the
    // decoder has flagged a correctable single-bit error; otherwise fall back
    // to the raw read data from the four slices.
    assign data_corrected =
        (ECC_ENABLE && addr_has_ecc && ecc_single_err_raw) ? data_corrected_raw
                                                           : read_data_64;
    
    // Scrubbing engine
    wire        scrub_cmd_req;
    wire [2:0]  scrub_cmd;
    wire [1:0]  scrub_sz;
    wire [ADDR_WIDTH-1:0] scrub_addr;
    wire        scrub_cmd_put;
    wire        scrub_data_put;
    wire [63:0] scrub_data_in;
    wire [31:0] scrub_progress_int;
    wire        scrub_active_int;
    wire        scrub_complete_int;
    // In fast/SIM_DIRECT_READ mode the simplified read path does not model the
    // full bus-level turnaround/timing behavior exercised by the scrubbing
    // engine. To avoid spurious JEDEC timing violations in that configuration,
    // completely disable the hardware scrubber while still allowing the host
    // to toggle SCRUB_ENABLE for register coverage.
    wire        scrub_enable_int;

`ifdef SIM_DIRECT_READ
    assign scrub_enable_int = 1'b0;
`else
    assign scrub_enable_int = SCRUB_ENABLE && ECC_ENABLE;
`endif
    
    ddr2_scrubber #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .SCRUB_BURST_SIZE(8)
    ) u_scrubber (
        .clk(CLK),
        .reset(RESET),
        .enable(scrub_enable_int),
        .ready(ready0),
        .notfull(notfull0),
        .ecc_single_err(ecc_single_err_int),
        .ecc_double_err(ecc_double_err_int),
        .cmd_req(scrub_cmd_req),
        .cmd(scrub_cmd),
        .sz(scrub_sz),
        .addr(scrub_addr),
        .cmd_put(scrub_cmd_put),
        .data_put(scrub_data_put),
        .data_in(scrub_data_in),
        .fetching(FETCHING),
        .validout(validout0),
        .data_out(read_data_64),
        .raddr(core_raddr0),
        .scrub_progress(scrub_progress_int),
        .scrub_active(scrub_active_int),
        .scrub_complete(scrub_complete_int)
    );
    
    // Command arbitration: host commands have priority over scrubbing
    wire [2:0]  arb_cmd;
    wire [1:0]  arb_sz;
    wire [ADDR_WIDTH-1:0] arb_addr;
    wire        arb_cmd_put;
    wire        arb_data_put;
    wire [63:0] arb_data_in;
    
    assign arb_cmd     = cmd_put ? CMD : (scrub_cmd_req ? scrub_cmd : 3'b000);
    assign arb_sz      = cmd_put ? SZ : (scrub_cmd_req ? scrub_sz : 2'b00);
    assign arb_addr    = cmd_put ? core_addr : (scrub_cmd_req ? scrub_addr : {ADDR_WIDTH{1'b0}});
    assign arb_cmd_put = cmd_put || (scrub_cmd_put && !cmd_put);
    assign arb_data_put = put_dataFIFO || (scrub_data_put && !put_dataFIFO);
    assign arb_data_in = put_dataFIFO ? DIN : (scrub_data_put ? scrub_data_in : 64'd0);
    
    // RAS registers
    wire [3:0] err_rank = 4'd0;  // Single rank for now
    
    ddr2_ras_registers #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .NUM_RANKS(1)
    ) u_ras_regs (
        .clk(CLK),
        .reset(RESET),
        .ecc_single_err(ecc_single_err_int && ECC_ENABLE),
        .ecc_double_err(ecc_double_err_int && ECC_ENABLE),
        .ecc_syndrome(ecc_syndrome_int),
        .err_addr(last_read_addr),
        .err_rank(err_rank),
        .scrub_enable(SCRUB_ENABLE),
        .scrub_active(scrub_active_int),
        .scrub_progress(scrub_progress_int),
        .reg_addr(RAS_REG_ADDR),
        .reg_data_out(RAS_REG_DATA),
        .irq_ecc_corr(RAS_IRQ_CORR),
        .irq_ecc_uncorr(RAS_IRQ_UNCORR),
        .rank_degraded(RAS_RANK_DEGRADED),
        .fatal_error(RAS_FATAL_ERROR)
    );
    
    // Output assignments
    assign ECC_SINGLE_ERR = ecc_single_err_int;
    assign ECC_DOUBLE_ERR = ecc_double_err_int;
    assign SCRUB_ACTIVE = scrub_active_int;
    
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
        .CMD           (arb_cmd),
        .SZ            (arb_sz),
        .ADDR          (arb_addr),
        // Use the most-significant host address bit as a simple rank select
        // hint for the core controller/protocol engine. In the current design
        // this is used purely for observability and does not yet drive any
        // multi-CS behavior at the pad level.
        .RANK_SEL      (host_addr[HOST_ADDR_WIDTH-1]),
        .cmd_put       (arb_cmd_put),
        .DIN           (arb_data_in[15:0]),
        .put_dataFIFO  (arb_data_put),
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
        .CMD           (arb_cmd),
        .SZ            (arb_sz),
        .ADDR          (arb_addr),
        .RANK_SEL      (host_addr[HOST_ADDR_WIDTH-1]),
        .cmd_put       (arb_cmd_put),
        .DIN           (arb_data_in[31:16]),
        .put_dataFIFO  (arb_data_put),
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
        .CMD           (arb_cmd),
        .SZ            (arb_sz),
        .ADDR          (arb_addr),
        .RANK_SEL      (host_addr[HOST_ADDR_WIDTH-1]),
        .cmd_put       (arb_cmd_put),
        .DIN           (arb_data_in[47:32]),
        .put_dataFIFO  (arb_data_put),
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
        .CMD           (arb_cmd),
        .SZ            (arb_sz),
        .ADDR          (arb_addr),
        .RANK_SEL      (host_addr[HOST_ADDR_WIDTH-1]),
        .cmd_put       (arb_cmd_put),
        .DIN           (arb_data_in[63:48]),
        .put_dataFIFO  (arb_data_put),
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
    // `data_corrected` already incorporates ECC correction when applicable.
    assign DOUT          = data_corrected;
    assign FILLCOUNT      = fillcount0;
    assign READY          = ready0;
    assign VALIDOUT       = validout0;
    assign NOTFULL        = notfull0;
    assign SELFREF_ACTIVE = selfref_active0;
    assign PWRDOWN_ACTIVE = pdown_active0;

endmodule
/* verilator lint_on UNUSEDSIGNAL */

