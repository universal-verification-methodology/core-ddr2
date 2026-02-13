`timescale 1ns/1ps

/**
 * RAS (Reliability, Availability, Serviceability) registers for DDR2 controller.
 *
 * Tracks error statistics, provides status registers, and implements threshold-based
 * error reporting for server-grade memory controller.
 */
module ddr2_ras_registers #(
    parameter integer ADDR_WIDTH = 25,
    parameter integer NUM_RANKS = 1
) (
    input  wire         clk,
    input  wire         reset,
    
    // Error inputs
    input  wire         ecc_single_err,      // Single-bit error detected and corrected
    input  wire         ecc_double_err,      // Double-bit error detected (uncorrectable)
    input  wire [7:0]   ecc_syndrome,        // ECC syndrome for diagnostics
    input  wire [ADDR_WIDTH-1:0] err_addr,  // Address where error occurred
    input  wire [3:0]   err_rank,            // Rank where error occurred
    
    // Control inputs
    input  wire         scrub_enable,       // Enable scrubbing
    input  wire         scrub_active,       // Scrubbing is active
    input  wire [31:0]  scrub_progress,      // Scrubbing progress (address counter)
    
    // Register read interface (simple CSR-style)
    input  wire [7:0]   reg_addr,           // Register address
    output reg  [31:0]  reg_data_out,       // Register data output
    
    // Status outputs
    output wire         irq_ecc_corr,        // Interrupt: correctable error threshold exceeded
    output wire         irq_ecc_uncorr,      // Interrupt: uncorrectable error detected
    output wire         rank_degraded,       // Rank degradation status (any rank)
    output wire         fatal_error          // Fatal error status
);

    // Error counters (per rank)
    reg [31:0] correctable_err_cnt [0:NUM_RANKS-1];
    reg [31:0] uncorrectable_err_cnt [0:NUM_RANKS-1];
    
    // Last error context
    reg [ADDR_WIDTH-1:0] last_err_addr;
    reg [3:0]            last_err_rank;
    reg [7:0]            last_err_syndrome;
    reg                  last_err_type;  // 0=single, 1=double
    
    // Configuration registers
    reg [31:0] corr_err_threshold;      // Correctable error threshold
    reg [31:0] uncorr_err_threshold;    // Uncorrectable error threshold
    reg [31:0] scrub_interval;          // Scrubbing interval (cycles)
    
    // Status registers
    reg [31:0] scrub_count;             // Number of scrubbing cycles completed
    reg [31:0] total_corr_errors;       // Total correctable errors across all ranks
    reg [31:0] total_uncorr_errors;      // Total uncorrectable errors across all ranks
    
    // Interrupt and status flags
    reg irq_corr_reg;
    reg irq_uncorr_reg;
    reg rank_degraded_reg;
    reg fatal_error_reg;
    
    integer i;
    
    // Initialize registers
    initial begin
        corr_err_threshold = 32'd1000;    // Default: 1000 correctable errors
        uncorr_err_threshold = 32'd1;     // Default: 1 uncorrectable error
        scrub_interval = 32'd1000000;      // Default: 1M cycles
        for (i = 0; i < NUM_RANKS; i = i + 1) begin
            correctable_err_cnt[i] = 32'd0;
            uncorrectable_err_cnt[i] = 32'd0;
        end
        total_corr_errors = 32'd0;
        total_uncorr_errors = 32'd0;
        scrub_count = 32'd0;
        last_err_addr = {ADDR_WIDTH{1'b0}};
        last_err_rank = 4'd0;
        last_err_syndrome = 8'd0;
        last_err_type = 1'b0;
        irq_corr_reg = 1'b0;
        irq_uncorr_reg = 1'b0;
        rank_degraded_reg = 1'b0;
        fatal_error_reg = 1'b0;
    end
    
    // Error counter updates
    always @(posedge clk) begin
        if (reset) begin
            for (i = 0; i < NUM_RANKS; i = i + 1) begin
                correctable_err_cnt[i] <= 32'd0;
                uncorrectable_err_cnt[i] <= 32'd0;
            end
            total_corr_errors <= 32'd0;
            total_uncorr_errors <= 32'd0;
            last_err_addr <= {ADDR_WIDTH{1'b0}};
            last_err_rank <= 4'd0;
            last_err_syndrome <= 8'd0;
            last_err_type <= 1'b0;
            irq_corr_reg <= 1'b0;
            irq_uncorr_reg <= 1'b0;
            rank_degraded_reg <= 1'b0;
            fatal_error_reg <= 1'b0;
        end else begin
            // Update error counters
            if (ecc_single_err) begin
                if (err_rank < NUM_RANKS) begin
                    correctable_err_cnt[err_rank] <= correctable_err_cnt[err_rank] + 32'd1;
                end
                total_corr_errors <= total_corr_errors + 32'd1;
                last_err_addr <= err_addr;
                last_err_rank <= err_rank;
                last_err_syndrome <= ecc_syndrome;
                last_err_type <= 1'b0;
                
                // Check threshold
                if (err_rank < NUM_RANKS && 
                    correctable_err_cnt[err_rank] >= corr_err_threshold) begin
                    irq_corr_reg <= 1'b1;
                    rank_degraded_reg <= 1'b1;
                end
            end
            
            if (ecc_double_err) begin
                if (err_rank < NUM_RANKS) begin
                    uncorrectable_err_cnt[err_rank] <= uncorrectable_err_cnt[err_rank] + 32'd1;
                end
                total_uncorr_errors <= total_uncorr_errors + 32'd1;
                last_err_addr <= err_addr;
                last_err_rank <= err_rank;
                last_err_syndrome <= ecc_syndrome;
                last_err_type <= 1'b1;
                
                // Uncorrectable error always triggers interrupt and fatal flag
                irq_uncorr_reg <= 1'b1;
                fatal_error_reg <= 1'b1;
                
                // Check threshold
                if (err_rank < NUM_RANKS && 
                    uncorrectable_err_cnt[err_rank] >= uncorr_err_threshold) begin
                    rank_degraded_reg <= 1'b1;
                end
            end
            
            // Update scrubbing counter
            if (scrub_active && scrub_enable) begin
                scrub_count <= scrub_count + 32'd1;
            end
        end
    end
    
    // Register read interface (simple address decode)
    always @(*) begin
        case (reg_addr)
            8'h00: reg_data_out = total_corr_errors;
            8'h04: reg_data_out = total_uncorr_errors;
            8'h08: reg_data_out = {last_err_type, 3'd0, last_err_rank, last_err_syndrome};
            8'h0C: reg_data_out = last_err_addr;
            8'h10: reg_data_out = corr_err_threshold;
            8'h14: reg_data_out = uncorr_err_threshold;
            8'h18: reg_data_out = scrub_count;
            8'h1C: reg_data_out = scrub_progress;
            8'h20: reg_data_out = {fatal_error_reg, rank_degraded_reg, irq_uncorr_reg, irq_corr_reg, 28'd0};
            default: begin
                if (reg_addr >= 8'h40 && reg_addr < 8'h40 + (NUM_RANKS * 4)) begin
                    // Per-rank correctable error counters (starting at 0x40)
                    i = (reg_addr - 8'h40) >> 2;
                    if (i < NUM_RANKS) begin
                        reg_data_out = correctable_err_cnt[i];
                    end else begin
                        reg_data_out = 32'd0;
                    end
                end else if (reg_addr >= 8'h80 && reg_addr < 8'h80 + (NUM_RANKS * 4)) begin
                    // Per-rank uncorrectable error counters (starting at 0x80)
                    i = (reg_addr - 8'h80) >> 2;
                    if (i < NUM_RANKS) begin
                        reg_data_out = uncorrectable_err_cnt[i];
                    end else begin
                        reg_data_out = 32'd0;
                    end
                end else begin
                    reg_data_out = 32'd0;
                end
            end
        endcase
    end
    
    // Status outputs
    assign irq_ecc_corr = irq_corr_reg;
    assign irq_ecc_uncorr = irq_uncorr_reg;
    assign rank_degraded = rank_degraded_reg;
    assign fatal_error = fatal_error_reg;

endmodule
