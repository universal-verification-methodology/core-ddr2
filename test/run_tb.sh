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
LOG_DIR="$TEST/logs"
mkdir -p "$BUILD"
mkdir -p "$LOG_DIR"
cd "$BUILD"
echo "Compiling DUT and testbench..."

# By default we build configurations with:
#   -DSIM_SHORT_INIT : shortened init time so tests complete quickly
#   -DSTRICT_JEDEC   : enforce strict JEDEC timing bounds (refresh intervals, etc.)
#   -DSIM_DIRECT_READ: (in "fast" mode) bypasses the full DDR2 return path and drives
#                     host-visible read data from a simplified model
#                     inside the controller. This is useful for quick
#                     bring-up and functional checks.
#
# Default behavior: sweeps both "core" and "server" tops, and both "fast" and "full" modes.
# This provides comprehensive coverage including:
#   - Fast mode: quick functional checks with simplified read path
#   - Full mode: production-like, full bus-level verification (including
#                the closed-loop DQ path and timing-sensitive monitors)
#
# To run a single configuration, set environment variables:
#   TOP=core FULL_BUS=1 ./test/run_tb.sh    # Run only core top in full bus mode
#   TOP=server FULL_BUS=0 ./test/run_tb.sh  # Run only server top in fast mode
#
# The behavioral DDR2 memory model (`ddr2_simple_mem`) can also be
# re-parameterized at build time. By default this script will sweep a
# small set of configurations; you can restrict it to a single point
# by explicitly setting the environment variables below:
#   MEM_DEPTH  : number of 16-bit words per rank
#   READ_LAT   : read latency in CLK cycles
#   RANK_BITS  : log2(number of ranks)
#
# If an env var is set, only that value is used (no sweep on that axis).
# If it is left unset, a small default list is swept.

BASE_SIM_DEFINES="-g2012 -DSIM_SHORT_INIT -DSTRICT_JEDEC"

# Enable negative tests (expected to fail via $fatal) by exporting
# NEGATIVE_TESTS=1 before running this script. By default, only the
# positive regression suite runs.
if [ "${NEGATIVE_TESTS:-0}" -ne 0 ]; then
  BASE_SIM_DEFINES="$BASE_SIM_DEFINES -DNEGATIVE_TESTS"
fi

# Decide which tops and modes to run.
# Default (no TOP/FULL_BUS provided): sweep both tops and both modes.
if [ -z "${TOP:-}" ] && [ -z "${FULL_BUS:-}" ]; then
  TOP_LIST=("core" "server")
  MODE_LIST=("fast" "full")
else
  TOP_LIST=("${TOP:-core}")
  if [ "${FULL_BUS:-0}" -eq 0 ]; then
    MODE_LIST=("fast")
  else
    MODE_LIST=("full")
  fi
fi

# Define sweep lists (or singletons if env vars are provided).
if [ -n "${MEM_DEPTH:-}" ]; then
  MEM_DEPTH_LIST=("${MEM_DEPTH}")
else
  MEM_DEPTH_LIST=(1024 4096)
fi

# READ latency:
# - If READ_LAT is explicitly provided, use that value for all modes.
# - Otherwise, sweep a broader set (24 and 32) in fast/SIM_DIRECT_READ mode,
#   but keep full-bus MODE=full runs aligned with the controller's internal
#   return-path timing at READ_LAT=24. Using 32 cycles in full-bus mode causes
#   the memory model to start driving data after the controller asserts
#   VALIDOUT, which trips the "VALIDOUT asserted before any MEM_RD_VALID"
#   safety check in the testbenches.
if [ -n "${READ_LAT:-}" ]; then
  READ_LAT_LIST_FAST=("${READ_LAT}")
  READ_LAT_LIST_FULL=("${READ_LAT}")
else
  READ_LAT_LIST_FAST=(24 32)
  READ_LAT_LIST_FULL=(24)
fi

if [ -n "${RANK_BITS:-}" ]; then
  RANK_BITS_LIST=("${RANK_BITS}")
else
  # By default, exercise multi-rank configurations across a small sweep of
  # logical rank counts: 2, 4 and 8 ranks (RANK_BITS = 1, 2, 3). Each point
  # overrides both the testbench memory model (via RANK_BITS_OVERRIDE) and the
  # DUT parameters (RANK_BITS on ddr2_controller / ddr2_protocol_engine) so
  # that the CS# vector widths and rank indices stay consistent end-to-end.
  RANK_BITS_LIST=(1 2 3)
fi

echo "Sweeping memory model configurations..."
: > "$RESULT"

for md in "${MEM_DEPTH_LIST[@]}"; do
  for rb in "${RANK_BITS_LIST[@]}"; do
    for mode in "${MODE_LIST[@]}"; do
      # Select per-mode READ_LAT sweep.
      if [ "$mode" = "full" ]; then
        READ_LAT_LIST=("${READ_LAT_LIST_FULL[@]}")
      else
        READ_LAT_LIST=("${READ_LAT_LIST_FAST[@]}")
      fi

      for rl in "${READ_LAT_LIST[@]}"; do
        # Per-mode defines:
        #   fast => add -DSIM_DIRECT_READ
        #   full => no SIM_DIRECT_READ (full bus-level closed-loop)
        SIM_MODE_DEFINES="$BASE_SIM_DEFINES"
        if [ "$mode" = "fast" ]; then
          SIM_MODE_DEFINES="$SIM_MODE_DEFINES -DSIM_DIRECT_READ"
        fi

        SIM_DEFINES="$SIM_MODE_DEFINES -DMEM_DEPTH_OVERRIDE=${md} -DREAD_LAT_OVERRIDE=${rl} -DRANK_BITS_OVERRIDE=${rb} -Pddr2_controller.RANK_BITS=${rb} -Pddr2_protocol_engine.RANK_BITS=${rb}"

        for top in "${TOP_LIST[@]}"; do
          LOG_TAG="TOP=${top}_MODE=${mode}_MEM_DEPTH=${md}_READ_LAT=${rl}_RANK_BITS=${rb}"
          # Make the log file name shell- and filesystem-friendly by removing '=' and ','.
          LOG_FILE_BASENAME=$(echo "${LOG_TAG}" | tr ' =' '__')
          LOG_FILE="$LOG_DIR/${LOG_FILE_BASENAME}.log"

          echo "----------------------------------------------------------------" | tee -a "$RESULT"
          echo "Config: TOP=${top}, MODE=${mode}, MEM_DEPTH=${md}, READ_LAT=${rl}, RANK_BITS=${rb}" | tee -a "$RESULT"
          echo "----------------------------------------------------------------" | tee -a "$RESULT"

          if [ "$top" = "server" ]; then
            echo "Building tb_ddr2_server_controller (TOP=server, MODE=${mode})..."
            iverilog -o tb_ddr2_server_controller.vvp \
                $SIM_DEFINES -I"$TEST" \
                "$DUT/fifo.v" \
                "$DUT/ddr2_init_engine.v" \
                "$DUT/ddr2_ring_buffer8.v" \
                "$DUT/ddr2_phy.v" \
                "$DUT/ddr2_protocol_engine.v" \
                "$DUT/ddr2_controller.v" \
                "$DUT/ecc_secded.v" \
                "$DUT/ddr2_scrubber.v" \
                "$DUT/ddr2_ras_registers.v" \
                "$DUT/ddr2_server_controller.v" \
                "$TEST/ddr2_simple_mem.v" \
                "$TEST/ddr2_timing_checker.v" \
                "$TEST/ddr2_turnaround_checker.v" \
                "$TEST/ddr2_bank_checker.v" \
                "$TEST/ddr2_dqs_monitor.v" \
                "$TEST/ddr2_ocd_zq_monitor.v" \
                "$TEST/ddr2_power_monitor.v" \
                "$TEST/ddr2_fifo_monitor.v" \
                "$TEST/ddr2_refresh_monitor.v" \
                "$TEST/tb_ddr2_server_controller.v"
            echo "Running simulation for server top (MODE=${mode}, READ_LAT=${rl})..."
            echo "Log: $LOG_FILE"
            vvp tb_ddr2_server_controller.vvp 2>&1 | tee "$LOG_FILE" | tee -a "$RESULT"
            echo "Done. VCD: $BUILD/tb_ddr2_server_controller.iverilog.vcd"
          else
            echo "Building tb_ddr2_controller (TOP=core, MODE=${mode})..."
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
                "$TEST/ddr2_turnaround_checker.v" \
                "$TEST/ddr2_bank_checker.v" \
                "$TEST/ddr2_dqs_monitor.v" \
                "$TEST/ddr2_ocd_zq_monitor.v" \
                "$TEST/ddr2_power_monitor.v" \
                "$TEST/ddr2_fifo_monitor.v" \
                "$TEST/ddr2_refresh_monitor.v" \
                "$TEST/tb_ddr2_controller.v"
            echo "Running simulation for core top (MODE=${mode}, READ_LAT=${rl})..."
            echo "Log: $LOG_FILE"
            vvp tb_ddr2_controller.vvp 2>&1 | tee "$LOG_FILE" | tee -a "$RESULT"
            echo "Done. VCD: $BUILD/tb_ddr2_controller.iverilog.vcd"
          fi
        done
      done
    done
  done
done

echo "Result: $RESULT"

