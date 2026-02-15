`timescale 1ns/1ps

/**
 * DDR2 command/address CRC + retry front-end (stub).
 *
 * This module is intended to sit between the host and the existing
 * `ddr2_controller` / `ddr2_server_controller` command interface. It will
 * eventually:
 *   - Compute a CRC (or strong parity) over {CMD,SZ,ADDR,OP} for each host
 *     command.
 *   - Buffer CRC metadata alongside the command FIFO.
 *   - Receive CRC status feedback from a downstream agent and, on error,
 *     re-issue the affected command subject to retry limits.
 *
 * In this revision, the implementation is a pure pass-through wrapper so that
 * the interface can be explored and wired up under an experimental define
 * without changing functional behavior.
 */
module ddr2_cmd_crc_frontend #(
    parameter integer ADDR_WIDTH = 25
) (
    input  wire         clk,
    input  wire         reset,

    // Host-side command interface.
    input  wire [2:0]   host_cmd,
    input  wire [1:0]   host_sz,
    input  wire [ADDR_WIDTH-1:0] host_addr,
    input  wire         host_cmd_put,

    // Downstream controller command interface (to ddr2_controller/server).
    output wire [2:0]   ctrl_cmd,
    output wire [1:0]   ctrl_sz,
    output wire [ADDR_WIDTH-1:0] ctrl_addr,
    output wire         ctrl_cmd_put

    // Future extensions (not yet implemented):
    //   - CRC metadata outputs (per-command CRC bits).
    //   - CRC feedback inputs (crc_ok/crc_error) from memory model or DIMM.
    //   - Retry counters and RAS integration.
);

    // For now this is a simple pass-through so that the module can be
    // instantiated without affecting behavior. CRC and retry logic will be
    // added in future revisions.
    assign ctrl_cmd     = host_cmd;
    assign ctrl_sz      = host_sz;
    assign ctrl_addr    = host_addr;
    assign ctrl_cmd_put = host_cmd_put;

endmodule

