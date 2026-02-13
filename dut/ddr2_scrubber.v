`timescale 1ns/1ps

/**
 * DDR2 memory scrubbing engine.
 *
 * Performs background read-verify-correct cycles across the entire memory address space.
 * When a single-bit error is detected via ECC, automatically writes back the corrected data.
 *
 * Operates at low priority behind host commands to minimize performance impact.
 */
module ddr2_scrubber #(
    parameter integer ADDR_WIDTH = 25,
    parameter integer SCRUB_BURST_SIZE = 8  // Number of words per scrub operation
) (
    input  wire         clk,
    input  wire         reset,
    
    // Control
    input  wire         enable,             // Enable scrubbing
    input  wire         ready,               // Controller is ready
    input  wire         notfull,             // Command FIFO has space
    
    // ECC status (from RAS registers or ECC decoder)
    input  wire         ecc_single_err,     // Single-bit error detected
    input  wire         ecc_double_err,      // Double-bit error detected
    
    // Command interface (drives controller command bus)
    output reg          cmd_req,             // Request to issue command
    output reg  [2:0]   cmd,                 // Command (BLR for read, BLW for write)
    output reg  [1:0]   sz,                  // Size (for BLR/BLW)
    output reg  [ADDR_WIDTH-1:0] addr,       // Address
    output reg          cmd_put,             // Assert to enqueue command
    
    // Data interface
    output reg          data_put,            // Assert to enqueue write data
    output reg  [63:0]  data_in,             // Write data (corrected data on write-back)
    input  wire         fetching,            // Host is fetching read data
    input  wire         validout,            // Read data valid
    input  wire [63:0]  data_out,            // Read data from controller
    input  wire [ADDR_WIDTH-1:0] raddr,     // Return address
    
    // Status
    output reg  [31:0]  scrub_progress,      // Current scrubbing address
    output reg          scrub_active,        // Scrubbing is active
    output reg          scrub_complete       // Scrubbing cycle complete
);

    // State machine states
    localparam IDLE        = 3'd0;
    localparam WAIT_IDLE   = 3'd1;
    localparam ISSUE_READ  = 3'd2;
    localparam WAIT_DATA   = 3'd3;
    localparam CHECK_ECC   = 3'd4;
    localparam ISSUE_WRITE = 3'd5;
    localparam WAIT_WRITE  = 3'd6;
    
    reg [2:0] state;
    reg [2:0] next_state;
    
    // Address counter for scrubbing
    reg [ADDR_WIDTH-1:0] scrub_addr;
    reg [ADDR_WIDTH-1:0] scrub_start_addr;
    reg [ADDR_WIDTH-1:0] scrub_end_addr;
    
    // Burst counter
    reg [3:0] burst_count;
    reg [3:0] words_remaining;
    
    // Data buffer for read data
    reg [63:0] read_data_buffer [0:SCRUB_BURST_SIZE-1];
    reg [ADDR_WIDTH-1:0] read_addr_buffer [0:SCRUB_BURST_SIZE-1];
    reg [3:0] buffer_idx;
    
    // Write-back flag
    reg writeback_needed;
    reg [63:0] corrected_data;
    
    // Idle counter (wait for controller to be idle before scrubbing)
    reg [15:0] idle_counter;
    localparam IDLE_THRESHOLD = 16'd100;  // Wait 100 cycles of idle before scrubbing
    
    integer i;
    
    initial begin
        state = IDLE;
        scrub_addr = {ADDR_WIDTH{1'b0}};
        scrub_start_addr = {ADDR_WIDTH{1'b0}};
        scrub_end_addr = {ADDR_WIDTH{1'b1}};
        burst_count = 4'd0;
        words_remaining = 4'd0;
        buffer_idx = 4'd0;
        writeback_needed = 1'b0;
        idle_counter = 16'd0;
        scrub_progress = 32'd0;
        scrub_active = 1'b0;
        scrub_complete = 1'b0;
    end
    
    // State machine
    always @(posedge clk) begin
        if (reset) begin
            state <= IDLE;
            scrub_addr <= {ADDR_WIDTH{1'b0}};
            burst_count <= 4'd0;
            words_remaining <= 4'd0;
            buffer_idx <= 4'd0;
            writeback_needed <= 1'b0;
            idle_counter <= 16'd0;
            scrub_progress <= 32'd0;
            scrub_active <= 1'b0;
            scrub_complete <= 1'b0;
            cmd_req <= 1'b0;
            cmd_put <= 1'b0;
            data_put <= 1'b0;
        end else begin
            case (state)
                IDLE: begin
                    scrub_active <= 1'b0;
                    scrub_complete <= 1'b0;
                    cmd_req <= 1'b0;
                    cmd_put <= 1'b0;
                    data_put <= 1'b0;
                    buffer_idx <= 4'd0;
                    writeback_needed <= 1'b0;
                    
                    if (enable && ready) begin
                        // Reset scrubbing address range
                        scrub_addr <= scrub_start_addr;
                        scrub_progress <= 32'd0;
                        state <= WAIT_IDLE;
                        idle_counter <= 16'd0;
                    end
                end
                
                WAIT_IDLE: begin
                    // Wait for controller to be idle (notfull indicates no pending commands)
                    if (notfull && ready) begin
                        idle_counter <= idle_counter + 16'd1;
                        if (idle_counter >= IDLE_THRESHOLD) begin
                            state <= ISSUE_READ;
                            scrub_active <= 1'b1;
                            idle_counter <= 16'd0;
                        end
                    end else begin
                        idle_counter <= 16'd0;
                    end
                end
                
                ISSUE_READ: begin
                    if (notfull && ready) begin
                        // Issue block read command
                        cmd <= 3'b011;  // BLR
                        sz <= 2'b00;     // 8 words
                        addr <= scrub_addr;
                        cmd_put <= 1'b1;
                        words_remaining <= SCRUB_BURST_SIZE;
                        buffer_idx <= 4'd0;
                        state <= WAIT_DATA;
                    end
                end
                
                WAIT_DATA: begin
                    cmd_put <= 1'b0;
                    
                    if (validout && fetching) begin
                        // Capture read data
                        read_data_buffer[buffer_idx] <= data_out;
                        read_addr_buffer[buffer_idx] <= raddr;
                        
                        // Check for ECC errors on this word
                        if (ecc_single_err) begin
                            // Single-bit error detected - need to write back corrected data
                            writeback_needed <= 1'b1;
                            scrub_addr <= raddr;
                            corrected_data <= data_out;  // Parent module provides corrected data
                        end
                        
                        buffer_idx <= buffer_idx + 4'd1;
                        words_remaining <= words_remaining - 4'd1;
                        
                        if (words_remaining == 4'd1) begin
                            // All data received, check ECC
                            state <= CHECK_ECC;
                            buffer_idx <= 4'd0;
                        end
                    end
                end
                
                CHECK_ECC: begin
                    // Check if any word had a correctable error
                    // (ECC checking happens in parent module during read, we check flags)
                    // Note: ecc_single_err/ecc_double_err are sampled during read phase
                    if (writeback_needed) begin
                        // Write back was already scheduled, proceed to write
                        state <= ISSUE_WRITE;
                    end else begin
                        // No errors or uncorrectable error, continue to next address
                        state <= WAIT_WRITE;
                    end
                end
                
                ISSUE_WRITE: begin
                    if (notfull && ready && writeback_needed) begin
                        // Issue block write command with corrected data
                        cmd <= 3'b100;  // BLW
                        sz <= 2'b00;     // 8 words (but we only write one corrected word)
                        addr <= scrub_addr;
                        cmd_put <= 1'b1;
                        data_in <= corrected_data;
                        data_put <= 1'b1;
                        state <= WAIT_WRITE;
                    end else if (!writeback_needed) begin
                        state <= WAIT_WRITE;
                    end
                end
                
                WAIT_WRITE: begin
                    cmd_put <= 1'b0;
                    data_put <= 1'b0;
                    
                    // Wait a few cycles for write to complete
                    if (notfull) begin
                        // Advance to next address
                        if (scrub_addr >= scrub_end_addr) begin
                            // Scrubbing complete
                            scrub_complete <= 1'b1;
                            scrub_active <= 1'b0;
                            state <= IDLE;
                        end else begin
                            scrub_addr <= scrub_addr + SCRUB_BURST_SIZE;
                            scrub_progress <= scrub_progress + SCRUB_BURST_SIZE;
                            state <= WAIT_IDLE;
                            idle_counter <= 16'd0;
                        end
                    end
                end
                
                default: begin
                    state <= IDLE;
                end
            endcase
        end
    end

endmodule
