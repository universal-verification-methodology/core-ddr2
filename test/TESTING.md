## DDR2 Controller Verification – Testbench, Monitors & Timing Config

This document describes how the DDR2 controller testbench is structured, how to run it, and what each verification monitor checks. All tests are written in **pure Verilog** and run with **Icarus Verilog**.

---

## How to Run the Tests

- **Command** (from repo root):

```bash
./test/run_tb.sh
```

- **Outputs** (default configuration):
  - VCD waveform: `build/tb_ddr2_controller.iverilog.vcd` (or `tb_ddr2_server_controller.iverilog.vcd` for server top)
  - Text log (including all monitor messages): `test/result.txt`
  - Optional CSV trace (only when `CSV_TRACE` is defined): `build/ddr2_trace.csv`

- **Default behavior**: The testbench now **automatically sweeps** multiple memory model configurations:
  - `MEM_DEPTH`: 1024, 4096 words
  - `READ_LAT`: 24, 32 cycles
  - `RANK_BITS`: 1 (single rank)
  - All combinations are tested sequentially, with results appended to `test/result.txt`

The script compiles:

- DUT RTL from `dut/`:
  - `fifo.v`
  - `ddr2_init_engine.v`
  - `ddr2_ring_buffer8.v`
  - `ddr2_phy.v`
  - `ddr2_protocol_engine.v`
  - `ddr2_controller.v`
- Testbench and simple memory model from `test/`:
  - `ddr2_simple_mem.v`
  - `tb_ddr2_controller.v`
- Verification monitors from `test/`:
  - `ddr2_mrs_monitor.v`
  - `ddr2_timing_checker.v`
  - `ddr2_refresh_monitor.v`
  - `ddr2_dqs_monitor.v`
  - `ddr2_bank_checker.v`
  - `ddr2_fifo_monitor.v`
  - Shared timing config header: `ddr2_timing_config.vh`

On any serious mismatch or timing violation, the simulation calls **`$fatal`**, which terminates the run with a clear error message in `test/result.txt`.

---

## Simulation Modes and Defines

The `run_tb.sh` script supports several compile-time modes via Verilog `define`s, selected by environment variables:

- **Fast functional mode (default)**:
  - Enabled flags: `SIM_SHORT_INIT`, `SIM_DIRECT_READ`
  - Behavior:
    - Shortened DDR2 init time so regressions complete quickly.
    - Read data is driven by a simplified internal model (`SIM_DIRECT_READ` path), so the host-visible interface is deterministic and checked cycle-by-cycle via the scoreboard (`sb_check`).
  - How to run:
    ```bash
    ./test/run_tb.sh
    ```
  - Recommended for: day-to-day regressions and CI.

- **Full bus-level mode (closed-loop DDR2 bus)**:
  - Enabled flags: `SIM_SHORT_INIT`, **no** `SIM_DIRECT_READ` (set `FULL_BUS=1`).
  - Behavior:
    - Data returns through the full DDR2 bus and `ddr2_simple_mem` model; the testbench enforces that `VALIDOUT` never asserts before any `MEM_RD_VALID`, and the pin-level monitors (`ddr2_timing_checker`, `ddr2_refresh_monitor`, `ddr2_bank_checker`, `ddr2_dqs_monitor`, `ddr2_fifo_monitor`) validate protocol/timing.
  - How to run:
    ```bash
    FULL_BUS=1 ./test/run_tb.sh
    ```
  - Recommended for: deeper protocol/timing sweeps when you care about the full DQ/DQS path.

- **Negative tests (expected failures)**:
  - Enabled flags: `NEGATIVE_TESTS=1` (in addition to one of the modes above).
  - Behavior:
    - After all positive tests pass and `All tests completed successfully.` is printed, the testbench deliberately:
      - Enqueues a command with an **illegal `CMD` encoding** (`3'b111`), which is caught by a host-side checker inside `tb_ddr2_controller.v`.
      - (Optionally) can perform a scalar read from an address that was never written, which the scoreboard flags as “read from unknown addr”.
    - These are designed to **end the run with `$fatal` on purpose** and are **not** meant to be used in pass/fail CI without wrapping.
  - How to run:
    ```bash
    NEGATIVE_TESTS=1 ./test/run_tb.sh
    ```
  - How to interpret:
    - A run that terminates with a message like:
      - `ERROR: Host issued illegal CMD encoding: CMD=111 ...`
      - followed by the usual `$fatal` location line
    - is considered **PASS** for the negative-test suite.

- **CSV trace control**:
  - The per-cycle CSV trace in `ddr2_trace.csv` is guarded by the `CSV_TRACE` macro.
  - When `CSV_TRACE` is defined:
    - `tb_ddr2_controller.v` opens `ddr2_trace.csv`, writes a header, and emits one line per clock cycle with key host and memory signals.
    - The file is explicitly closed before `$finish;`.
  - When `CSV_TRACE` is not defined:
    - No CSV is produced; only `result.txt` and the VCD are generated.
  - Example (enable CSV in a one-off debug run):
    ```bash
    CSV_TRACE=1 ./test/run_tb.sh   # after adding -DCSV_TRACE wiring in the build if desired
    ```

- **Top selection (core vs server)**:
  - By default, `TOP=core` builds and runs `tb_ddr2_controller.v` (16-bit host data bus).
  - To test the server-style wrapper (64-bit host data bus):
    ```bash
    TOP=server ./test/run_tb.sh
    ```
  - The server top uses the same test scenarios but exercises the wider data path and multi-rank-capable memory model.

- **Memory model parameterization**:
  - The behavioral DDR2 memory model (`ddr2_simple_mem`) can be re-parameterized at build time.
  - **By default**, the testbench sweeps multiple configurations automatically:
    - `MEM_DEPTH`: 1024, 4096 words per rank
    - `READ_LAT`: 24, 32 cycles
    - `RANK_BITS`: 1 (single rank)
  - **To run a single configuration** instead of sweeping, set environment variables:
    ```bash
    MEM_DEPTH=8192 READ_LAT=40 RANK_BITS=2 ./test/run_tb.sh
    ```
  - When an env var is set, only that value is used (no sweep on that axis).
  - When left unset, the default sweep list is used.
  - These parameters are passed as Verilog `define` overrides (`MEM_DEPTH_OVERRIDE`, `READ_LAT_OVERRIDE`, `RANK_BITS_OVERRIDE`) and consumed by the testbench when instantiating the memory model.

---

## Timing Configuration (`ddr2_timing_config.vh`)

The coarse timing used by the monitors is centralized in `test/ddr2_timing_config.vh`. This header is included by both `ddr2_timing_checker.v` and `ddr2_refresh_monitor.v`, so you can change the timing profile for a specific speed grade or corner by editing a single file.

The key macros (all in controller clock cycles) are:

- `DDR2_TIMING_TRCD_MIN`: ACT → RD/WR minimum delay.
- `DDR2_TIMING_TRP_MIN`: PRE → ACT minimum delay to the same bank.
- `DDR2_TIMING_TRAS_MIN`: ACT → PRE minimum active time per row.
- `DDR2_TIMING_TRFC_MIN`: minimum delay between AUTO REFRESH commands (coarse).
- `DDR2_TIMING_TREFI_MIN_CLK`: minimum allowed refresh interval (0 disables the “too frequent” check).
- `DDR2_TIMING_TREFI_MAX_CLK`: maximum allowed refresh interval (detects starved refresh).

In the default **simulation** profile:

- `TRCD_MIN`, `TRP_MIN`, and `TRAS_MIN` are set to small, JEDEC‑like values to catch obvious timing violations without making the run overly long.
- `TREFI_MIN_CLK` is set to `0` (the controller purposely compresses refresh intervals under `SIM_SHORT_INIT`).
- `TREFI_MAX_CLK` is a generous upper bound (100k cycles) so a stuck or starved refresh scheduler still triggers a failure.

To adapt the monitors to a different target:

- Edit `ddr2_timing_config.vh` and adjust the macros to the desired number of cycles for your controller clock period and DDR2 speed grade.
- Re-run `./test/run_tb.sh`; you should see the same functional behavior, with timing violations now checked against your updated limits.

If a macro is not defined, each monitor falls back to its internal parameter default, so the config header can be partially overridden if needed.

---

## Testbench Overview (`tb_ddr2_controller.v`)

The top-level testbench instantiates:

- `ddr2_controller` as `u_dut`
- A simple behavioral DDR2 memory model `ddr2_simple_mem` as `u_mem`
- A set of **monitors** (see below) that watch DDR2 pins and host-facing signals

Key host-side signals driven by the testbench:

- `CMD[2:0]`: NOP, SCR, SCW, BLR, BLW
- `SZ[1:0]`: block size (1–4 bursts = 8/16/24/32 words)
- `ADDR[24:0]`: logical word address
- `DIN[15:0]`: write data
- `FETCHING`: pops from the return FIFO to consume read data

### Scoreboard and Data Pattern

The testbench includes a simple **scoreboard** and deterministic **data pattern**:

- `pattern_for_addr(addr)`: pure function mapping an address to a 16‑bit data pattern.
- `sb_write(addr, data)`: records expected data for a given address.
- `sb_check(addr, data)`: on each read, checks the observed data against the recorded value.

Whenever `VALIDOUT` is high, the testbench:

- Logs `RADDR` and `DOUT`.
- Invokes `sb_check(RADDR, DOUT)`.
- Calls `$fatal` on any mismatch (wrong data) or unknown address.

### Helper Tasks

The main helper tasks in the current testbench are:

- `do_scalar_rw(addr)`: scalar write then scalar read to a single address.
  - Uses `pattern_for_addr(addr)` for write data.
  - Records the expected value with `sb_write`.
  - Issues SCR and relies on the `VALIDOUT` monitor + `sb_check` for verification.
- `do_block_rw(base_addr, sz)`: block write + block read for a given `SZ`.
  - Writes `nwords` words (8/16/24/32) using `pattern_for_addr(base_addr + beat)`.
  - Calls `sb_write` for each address in the block.
  - Issues BLR and counts the expected number of beats while `sb_check` validates each returned word.

All these tasks include **data and address checks via the scoreboard**; mismatches trigger `$fatal`.

---

## Test Scenarios Covered

The main `initial` block in `tb_ddr2_controller.v` runs the following scenarios in order:

- **Initialization sequence**
  - Assert `RESET`, then pulse `INITDDR`
  - Wait for `READY` while logging cycles
  - Check that `NOTFULL` is asserted after init

- **Test 1 – Scalar operations**
  - Run `do_scalar_rw` at representative addresses:
    - `0`, `1`, `128`, `512`
  - Verifies:
    - SCW/SCR command path.
    - Address decode.
    - Data path, by checking `DOUT` against the pattern recorded for `RADDR`.
    - RADDR tagging (RADDR matches the address written).

- **Test 2 – Block operations**
  - Run `do_block_rw` for all `SZ` values:
    - `SZ=00` → 8 words.
    - `SZ=01` → 16 words.
    - `SZ=10` → 24 words.
    - `SZ=11` → 32 words.
  - Verifies:
    - BLW/BLR command handling.
    - Burst addressing and RADDR sequencing across all supported block sizes.
    - Data integrity for every beat in the block via the scoreboard.

- **Test 3 – FIFO stress / refresh-interleaving**
  - Loop of mixed scalar and block operations to keep the controller busy:
    - `do_scalar_rw` and `do_block_rw` repeated across many addresses (e.g., around `600`/`700`).
  - Verifies:
    - Refresh insertion while traffic is active (AUTO REFRESH messages appear in `result.txt`).
    - Data remains correct across refreshes and long traffic sequences.
    - Overall stability of protocol FSMs and FIFO flow control under load.

> Note: Additional scenarios like “invalid commands” and “reset during traffic” are described in the spec but are not yet implemented in the current shipped testbench.

At the end of all tests, the testbench prints:

- `All tests completed successfully.` (or similar)

If any assertion or monitor detects an error, simulation ends earlier with a descriptive message.

---

## Monitors and What They Check

### `ddr2_mrs_monitor.v`

- **Purpose**: Check DDR2 **mode register programming** and ordering.
- **Inputs**: `CLK`, `RESET`, `C0_CSBAR_PAD`, `C0_RASBAR_PAD`, `C0_CASBAR_PAD`, `C0_WEBAR_PAD`, `C0_BA_PAD`, `C0_A_PAD`.
- **Checks**:
  - Recognizes **load-mode (LM)** commands (MRS / EMRSx).
  - Enforces expected **sequence**:
    - EMRS2 → EMRS3 → EMRS1 (init) → MRS (DLL reset) → MRS (final, no reset) → EMRS1 (ODT on).
  - Enforces minimum **tMRD** between LM commands (NOP window).
  - Optionally checks A[12:0] contents for EMRS2/EMRS3/MRS DLL reset bits.
- **Failure mode**:
  - `$fatal` with messages such as:
    - `ERROR: EMRS2 not first in sequence`
    - `ERROR: MRS (DLL reset phase) has A[8]=0, expected 1`
    - `ERROR: tMRD violated between LM commands`

### `ddr2_timing_checker.v`

- **Purpose**: Enforce basic **JEDEC timing** constraints at a coarse level using the shared configuration from `ddr2_timing_config.vh` (or its internal defaults if the header is not present on the include path).
- **Inputs**: `CLK`, `RESET`, `C0_CSBAR_PAD`, `C0_RASBAR_PAD`, `C0_CASBAR_PAD`, `C0_WEBAR_PAD`, `C0_BA_PAD`.
- **Checks** (per-bank where applicable):
  - **tRCD**: ACTIVATE → READ/WRITE delay (note: intentionally skipped at pad level to avoid false positives; the protocol engine handles this internally).
  - **tRP**: PRECHARGE → ACTIVATE delay to same bank.
  - **tRAS**: Minimum ACTIVE time between ACTIVATE and PRECHARGE.
  - **tRFC**: Minimum delay between AUTO REFRESH commands (global).
  - **tRRD**: Minimum delay between successive ACTIVATE commands (any banks) - **new in recent updates**.
    - Uses edge detection to avoid false positives when the ACTIVATE state is held for multiple cycles.
  - **tFAW**: Four-activate window - ensures that within any window of 4 ACTIVATE commands, the time span is at least `TFAW_MIN` cycles - **new in recent updates**.
    - Maintains a sliding window of the last 4 ACT timestamps and checks the oldest vs. current.
- **Failure mode**:
  - `$fatal` with messages such as:
    - `ERROR: tRP violated on bank X`
    - `ERROR: tRAS violated on bank X`
    - `ERROR: tRRD violated (since_last_act_any=X < Y)`
    - `ERROR: tFAW violated (ACT window=X < Y cycles)`

### `ddr2_refresh_monitor.v`

- **Purpose**: Track **refresh interval** over long simulations using the bounds defined in `ddr2_timing_config.vh` (or the module’s default parameters if the macros are not defined).
- **Inputs**: `CLK`, `RESET`, `C0_CSBAR_PAD`, `C0_RASBAR_PAD`, `C0_CASBAR_PAD`, `C0_WEBAR_PAD`.
- **Checks**:
  - Logs each AUTO REFRESH and the cycle interval since the previous one.
  - Ensures interval stays between effective `TREFI_MIN_CLK` and `TREFI_MAX_CLK`, where the “effective” values are taken from `DDR2_TIMING_TREFI_MIN_CLK` / `DDR2_TIMING_TREFI_MAX_CLK` if those macros are defined, otherwise from the module parameters.
- **Failure mode**:
  - `$fatal` with messages such as:
    - `ERROR: Refresh interval too short`
    - `ERROR: Refresh interval too long`

### `ddr2_dqs_monitor.v`

- **Purpose**: Sanity-check **DQS behavior** around write bursts.
- **Inputs**: `CLK`, `RESET`, DDR2 command pads, `C0_DQS_PAD`.
- **Checks**:
  - On each WRITE command, opens a time window and counts **edges on DQS[0]**.
  - Ensures number of DQS edges is within a reasonable range:
    - Not zero (DQS must toggle during writes).
    - Not excessively large (should be limited to burst duration).
- **Failure mode**:
  - `$fatal` with messages such as:
    - `ERROR: too few DQS edges after WRITE`
    - `ERROR: too many DQS edges after WRITE`

### `ddr2_bank_checker.v`

- **Purpose**: Validate **bank/row state transitions** and row conflicts.
- **Inputs**: `CLK`, `RESET`, DDR2 command pads, address bus (`C0_A_PAD`) and bank address (`C0_BA_PAD`).
- **Checks**:
  - Tracks **active row per bank**.
  - When an ACTIVATE targets a bank with an already open row and a **different row address**, ensures:
    - A PRECHARGE occurred first.
    - The PRECHARGE→ACTIVATE delay satisfies **tRP**.
- **Failure mode**:
  - `$fatal` with messages such as:
    - `ERROR: Row conflict on bank X: tRP not satisfied`

### `ddr2_fifo_monitor.v`

- **Purpose**: Check **front-end FIFO flow control** (`FILLCOUNT` and `NOTFULL`).
- **Inputs**: `CLK`, `RESET`, `FILLCOUNT`, `NOTFULL`.
- **Checks**:
  - When `FILLCOUNT >= 33`, `NOTFULL` must deassert (per spec).
- **Failure mode**:
  - `$fatal` with messages such as:
    - `ERROR: NOTFULL should be 0 when FILLCOUNT>=33`

---

## Data Integrity Checks in the Testbench

The testbench enforces **data integrity** centrally through the scoreboard and `VALIDOUT` monitor:

- Every write (scalar or block) records the expected `{address, data}` pair via `sb_write`.
- Every cycle where `VALIDOUT == 1`:
  - The testbench logs `RADDR` and `DOUT`.
  - `sb_check(RADDR, DOUT)` compares the observed data with the recorded expectation.
  - Any mismatch triggers `$fatal` with an error indicating the address and expected vs. observed values.

Because the checking is centralized in the monitor, helper tasks only need to:

- Generate deterministic data with `pattern_for_addr`.
- Call `sb_write` when enqueuing write data.
- Drive the proper commands (SCW/SCR/BLW/BLR) and let data flow; all comparisons are automatic.

---

## Interpreting `test/result.txt`

When you run `./test/run_tb.sh`, the simulation log in `test/result.txt` will include:

- Informational messages:
  - Initialization progress (INITDDR, READY timing).
  - Issued commands (SCW/SCR/BLW/BLR, addresses, sizes).
  - Observed BLR beats (RADDR/DOUT).
  - Detected AUTO REFRESH commands and intervals.
  - DQS monitor summaries per WRITE.
  - Monitor logs for MRS/EMRS commands.
- Success line at the end (if no failures):
  - `All tests completed successfully.` (or similar).

On failure, you will see:

- A specific **`ERROR:`** message from one of the monitors or tasks.
- Followed by an immediate **simulation stop** due to `$fatal`.

Use the timestamp (`[%0t]`), bank number, address, and cycle counts in the message to correlate with the VCD (`tb_ddr2_controller.iverilog.vcd`) and debug the DUT.

---

## Recent Updates

### Memory Model Parameterization and Sweep

- **Automatic configuration sweep**: The testbench now automatically tests multiple memory configurations by default:
  - Sweeps `MEM_DEPTH` (1024, 4096), `READ_LAT` (24, 32), and `RANK_BITS` (1).
  - Each configuration runs sequentially with clear headers in `result.txt`.
  - To disable sweeping and test a single point, set the env vars: `MEM_DEPTH`, `READ_LAT`, `RANK_BITS`.

- **Server top support**: Added `tb_ddr2_server_controller.v` for testing the server-style wrapper:
  - 64-bit host data bus (vs. 16-bit for core).
  - Same test scenarios, adapted for wider data paths.
  - Run with `TOP=server ./test/run_tb.sh`.

### Enhanced Timing Checks

- **tRRD and tFAW coverage**: Extended `ddr2_timing_checker.v` to enforce:
  - **tRRD**: Minimum delay between successive ACTIVATE commands (any banks).
    - Default: 2 cycles (configurable via `DDR2_TIMING_TRRD_MIN`).
    - Uses edge detection to avoid false positives when ACTIVATE is held for multiple cycles.
  - **tFAW**: Four-activate window constraint.
    - Default: 16 cycles (configurable via `DDR2_TIMING_TFAW_MIN`).
    - Maintains a sliding window of the last 4 ACT timestamps.
    - Flags violations when the window span is too short.

- **Edge detection fix**: The timing checker now properly detects ACT command **edges** rather than counting every cycle where ACTIVATE is held active, preventing false tRRD violations during multi-cycle ACTIVATE states.

### Implementation Notes

- The memory model parameters are passed via Verilog `define` overrides (`MEM_DEPTH_OVERRIDE`, `READ_LAT_OVERRIDE`, `RANK_BITS_OVERRIDE`).
- Both `tb_ddr2_controller.v` and `tb_ddr2_server_controller.v` support these overrides.
- The sweep functionality is implemented in `run_tb.sh` using nested loops over configuration arrays.

---

## Extending the Testbench

To add new checks:

- **New functional scenarios**:
  - Add new tasks in `tb_ddr2_controller.v` (or `tb_ddr2_server_controller.v`) and call them from the main `initial` block.
  - Reuse existing patterns for scalar/block R/W and checks.
- **New monitors**:
  - Place new monitor modules under `test/`.
  - Instantiate them in both `tb_ddr2_controller.v` and `tb_ddr2_server_controller.v` if needed.
  - Add the new `.v` files to the `iverilog` command in `run_tb.sh`.
- **New timing constraints**:
  - Add parameters to `ddr2_timing_checker.v` and corresponding macros to `ddr2_timing_config.vh`.
  - Use edge detection for command-based checks to avoid false positives.

All new checks should fail loudly via `$fatal` so that CI and automation can treat any regression as a non-zero exit from `run_tb.sh`.

