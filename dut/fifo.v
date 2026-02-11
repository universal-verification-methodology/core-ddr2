`timescale 1ns/1ps

/**
 * Synchronous single-clock FIFO.
 * Used for command, data, and return FIFOs in ddr2_controller.
 * Parameters: WIDTH = data width, DEPTH_LOG2 = log2(depth).
 */
module fifo #(
    parameter WIDTH     = 16,
    parameter DEPTH_LOG2 = 6
) (
    input  wire                  clk,
    input  wire                  reset,
    input  wire [WIDTH-1:0]      data_in,
    input  wire                  put,
    input  wire                  get,
    output reg  [WIDTH-1:0]      data_out,
    output wire [DEPTH_LOG2:0]   fillcount,
    output wire                  full,
    output wire                  empty,
    output wire                  full_bar,
    output wire                  empty_bar
);

    localparam DEPTH = 1 << DEPTH_LOG2;

    reg [WIDTH-1:0] mem [0:DEPTH-1];
    reg [DEPTH_LOG2-1:0] wr_ptr;
    reg [DEPTH_LOG2-1:0] rd_ptr;
    reg [DEPTH_LOG2:0]    count;

    assign fillcount = count;
    assign full      = (count == DEPTH);
    assign empty     = (count == 0);
    assign full_bar  = ~full;
    assign empty_bar = ~empty;

    always @(posedge clk) begin
        if (reset) begin
            wr_ptr <= 0;
            rd_ptr <= 0;
            count  <= 0;
        end else begin
            if (put && full_bar) begin
                mem[wr_ptr] <= data_in;
                wr_ptr      <= wr_ptr + 1;
            end
            if (get && empty_bar) begin
                rd_ptr <= rd_ptr + 1;
            end
            case ({put && full_bar, get && empty_bar})
                2'b01:   count <= count - 1;
                2'b10:   count <= count + 1;
                default: ;
            endcase
        end
    end

    // Registered output: data_out reflects mem[rd_ptr] on the cycle get is asserted
    always @(posedge clk) begin
        if (reset)
            data_out <= 0;
        else if (get && empty_bar)
            data_out <= mem[rd_ptr];
    end

endmodule
