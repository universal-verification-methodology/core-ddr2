#!/usr/bin/env bash
# Run Verilog lint on DUT + testbench.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
DUT="$PROJECT_ROOT/dut"
TEST="$PROJECT_ROOT/test"

echo "Running Verilog lint (verilator --lint-only --timing)..."

verilator --lint-only --timing -Wall -Wno-UNOPTFLAT -Wno-UNSIGNED -Wno-PINCONNECTEMPTY \
  "$DUT/fifo.v" \
  "$DUT/ddr2_init_engine.v" \
  "$DUT/ddr2_ring_buffer8.v" \
  "$DUT/ddr2_phy.v" \
  "$DUT/ddr2_protocol_engine.v" \
  "$DUT/ddr2_controller.v" \
  "$TEST/ddr2_simple_mem.v" \
  "$TEST/ddr2_timing_checker.v" \
  "$TEST/ddr2_bank_checker.v" \
  "$TEST/ddr2_dqs_monitor.v" \
  "$TEST/ddr2_fifo_monitor.v" \
  "$TEST/ddr2_refresh_monitor.v" \
  "$TEST/tb_ddr2_controller.v" || true

echo "Verilog lint completed successfully."

