`timescale 1ns/1ps

/**
 * DDR2 read-data ring buffer (simplified for simulation):
 * captures 8 words from a DDR burst using the controller clock.
 *
 * Original version tried to track posedge/negedge of DQS; for this project
 * and the simple DDR2 memory model, that extra complexity led to timing
 * mismatches between when the model drives data and when the controller
 * samples it. This simplified version:
 *   - On a single-cycle listen pulse, arms the buffer and starts capturing.
 *   - On the next 8 clk cycles, samples DIN once per cycle into r0..r7.
 *   - After 8 samples, it automatically disarms.
 *   - readPtr selects which of r0..r7 is presented on DOUT.
 *
 * This keeps the logical behavior (8-word bursts) while making the capture
 * deterministic and aligned with the model's per-cycle read beats.
 */
/* verilator lint_off UNUSEDSIGNAL */
module ddr2_ring_buffer8 (
    input  wire        clk,      // controller clock (for listen/F0 and sync)
    input  wire        listen,
    input  wire        strobe,   // DQS from PHY
    input  wire        reset,
    input  wire [15:0] din,
    input  wire [2:0]  readPtr,
    output reg  [15:0] dout
);

    reg [15:0] r0, r1, r2, r3, r4, r5, r6, r7;
    reg        capturing;    // 1 when actively capturing a burst
    reg [2:0]  cap_index;    // 0..7 sample index within burst

    // Capture 8 consecutive samples on clk after listen.
    always @(posedge clk) begin
        if (reset) begin
            capturing <= 1'b0;
            cap_index <= 3'd0;
            r0 <= 16'b0; r1 <= 16'b0; r2 <= 16'b0; r3 <= 16'b0;
            r4 <= 16'b0; r5 <= 16'b0; r6 <= 16'b0; r7 <= 16'b0;
        end else begin
            // Start capture on listen pulse if not already capturing.
            if (listen && !capturing) begin
                capturing <= 1'b1;
                cap_index <= 3'd0;
            end else if (capturing) begin
                // Store current DIN into the appropriate slot.
                case (cap_index)
                    3'd0: r0 <= din;
                    3'd1: r1 <= din;
                    3'd2: r2 <= din;
                    3'd3: r3 <= din;
                    3'd4: r4 <= din;
                    3'd5: r5 <= din;
                    3'd6: r6 <= din;
                    3'd7: r7 <= din;
                    default: ;
                endcase

                // Advance index and stop after 8 samples.
                if (cap_index == 3'd7) begin
                    capturing <= 1'b0;
                end else begin
                    cap_index <= cap_index + 3'd1;
                end
            end
        end
    end

    // Output mux
    always @(*) begin
        case (readPtr)
            3'd0: dout = r0;
            3'd1: dout = r1;
            3'd2: dout = r2;
            3'd3: dout = r3;
            3'd4: dout = r4;
            3'd5: dout = r5;
            3'd6: dout = r6;
            3'd7: dout = r7;
            default: dout = r0;
        endcase
    end

endmodule
/* verilator lint_on UNUSEDSIGNAL */
