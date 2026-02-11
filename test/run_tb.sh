#!/usr/bin/env bash
# Run Icarus Verilog simulation for ddr2_controller testbench.
# Result is written to test/result.txt (and stdout).
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
DUT="$PROJECT_ROOT/dut"
TEST="$PROJECT_ROOT/test"
BUILD="${BUILD:-$PROJECT_ROOT/build}"
RESULT="${RESULT:-$TEST/result.txt}"
mkdir -p "$BUILD"
cd "$BUILD"
echo "Compiling DUT and testbench..."

# By default we build a "fast" configuration that uses:
#   -DSIM_SHORT_INIT : shortened init time so tests complete quickly
#   -DSIM_DIRECT_READ: bypasses the full DDR2 return path and drives
#                     host-visible read data from a simplified model
#                     inside the controller. This is useful for quick
#                     bring-up and functional checks.
#
# For a more production-like, full bus-level verification (including
# the closed-loop DQ path and timing-sensitive monitors), unset
# SIM_DIRECT_READ by exporting FULL_BUS=1 before running this script:
#   FULL_BUS=1 ./test/run_tb.sh
#
SIM_DEFINES="-g2012 -DSIM_SHORT_INIT"
if [ "${FULL_BUS:-0}" -eq 0 ]; then
  SIM_DEFINES="$SIM_DEFINES -DSIM_DIRECT_READ"
fi

# Enable negative tests (expected to fail via $fatal) by exporting
# NEGATIVE_TESTS=1 before running this script. By default, only the
# positive regression suite runs.
if [ "${NEGATIVE_TESTS:-0}" -ne 0 ]; then
  SIM_DEFINES="$SIM_DEFINES -DNEGATIVE_TESTS"
fi

iverilog -o tb_ddr2_controller.vvp \
    $SIM_DEFINES -I"$TEST" \
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
    "$TEST/tb_ddr2_controller.v"
echo "Running simulation..."
vvp tb_ddr2_controller.vvp 2>&1 | tee "$RESULT"
echo "Done. VCD: $BUILD/tb_ddr2_controller.iverilog.vcd"
echo "Result: $RESULT"
