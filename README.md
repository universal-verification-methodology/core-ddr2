## DDR2 Controller Core (`ddr2_controller`)

[![License: CC BY 4.0](https://img.shields.io/badge/License-CC%20BY%204.0-blue.svg)](https://creativecommons.org/licenses/by/4.0/)
[![Status: Experimental](https://img.shields.io/badge/status-experimental-orange.svg)](https://github.com/your-org/core-ddr2)
[![Tests: Icarus Verilog](https://img.shields.io/badge/tests-Icarus%20Verilog-brightgreen.svg)](https://github.com/your-org/core-ddr2/tree/main/test)
[![UVM: Verilator](https://img.shields.io/badge/UVM-Verilator-blue.svg)](https://github.com/your-org/core-ddr2/tree/main/test-uvm)

This repository contains a JEDEC-style DDR2 SDRAM controller targeting the memory device (512 Mb, x16, 4 banks). The controller exposes a simple FIFO-like front-end interface and maps host commands into DDR2 transactions, including full power-up initialization, periodic refresh, scalar and block reads/writes, and DQS-based data capture.

The verification and simulation flow is described in `test/TESTING.md`; UVM-based verification is documented in `test-uvm/README_UVM.md`. This `README` provides a practical overview and entry point.

---

## Features

- **Target device**: Micron `mt47h32m16_37e` (x16 DDR2 SDRAM).
- **Top module**: `ddr2_controller`.
- **Initialization**: JEDEC-like DDR2 power-up, mode register programming (MRS/EMRSx), DLL enable/reset.
- **Operations**:
  - Scalar read/write (`SCR`/`SCW`).
  - Block read/write (`BLR`/`BLW`) for 8/16/24/32-word bursts.
- **Timing management**: Internal state machines and counters derived from JEDEC timing parameters (tRCD, tRP, tRAS, tRFC, etc.).
- **Data path**:
  - Command, write-data and return FIFOs for decoupling host and DDR2 timing.
  - 8‑deep read-data ring buffer using DQS edges for capture and alignment.
- **PHY interface**: SSTL18 DDR2 PHY wrapper (`ddr2_phy`) driving CK/CK#, CKE, CS#, RAS#, CAS#, WE#, BA, A, DQ, DQS/DQS#, DM and ODT pins.

For block‑level and state‑machine details, see the project documentation in `docs/` and the JEDEC JESD79-2 reference where applicable.

---

## Repository Layout

- `dut/` – RTL implementation of the controller and supporting blocks:
  - `ddr2_controller.v`: top-level controller.
  - `ddr2_protocol_engine.v`: main transaction/state machine.
  - `ddr2_init_engine.v`: power‑up and mode‑register initialization engine.
  - `ddr2_ring_buffer8.v`: read‑data ring buffer for DDR capture.
  - `ddr2_phy.v`: SSTL18 DDR2 PHY / pad interface.
  - `fifo.v`: synchronous single‑clock FIFOs for command/data/return paths.
- `test/` – Verilog testbench, monitors and scripts:
  - `tb_ddr2_controller.v`: top‑level testbench and scoreboard.
  - `ddr2_simple_mem.v`: simple behavioral DDR2 memory model.
  - `ddr2_mrs_monitor.v`, `ddr2_timing_checker.v`, `ddr2_refresh_monitor.v`, `ddr2_dqs_monitor.v`, `ddr2_bank_checker.v`, `ddr2_fifo_monitor.v`: protocol and timing monitors.
  - `ddr2_timing_config.vh`: shared timing configuration header.
  - `TESTING.md`: detailed testing and monitor documentation.
  - `run_tb.sh`: convenience script to build and run simulations with Icarus Verilog.
- `test-uvm/` – UVM-based SystemVerilog testbench (Verilator + UVM-2017): `tb_ddr2_controller_uvm.sv`, `ddr2_tb_pkg.sv`, `ddr2_host_if.sv`, `run_tb.sh`; see `test-uvm/README_UVM.md` for build and test instructions.
- `tools/` – tool dependencies (e.g. `uvm-2017` for UVM verification).
- `build-uvm/` – build output for the UVM testbench.
- `docs/` – project documentation and reference material (e.g. JEDEC JESD79-2).

---

## Building and Running the Testbench

All verification is currently implemented in Verilog and intended to run with **Icarus Verilog**.

1. **Install dependencies**
   - Install Icarus Verilog (`iverilog`) and a compatible `vvp` runtime on your system.

2. **Run the main regression**

   ```bash
   ./test/run_tb.sh
   ```

   This runs the default “fast functional” configuration (shortened initialization, direct readback path) and produces:

   - VCD waveform: `build/tb_ddr2_controller.iverilog.vcd`
   - Text log: `test/result.txt`

3. **Run with full bus‑level DDR2 path**

   ```bash
   FULL_BUS=1 ./test/run_tb.sh
   ```

   In this mode, all reads/writes traverse the full DDR2 bus plus `ddr2_simple_mem` model, and pin‑level monitors enforce protocol and timing constraints.

4. **Optional CSV trace for debugging**

   ```bash
   CSV_TRACE=1 ./test/run_tb.sh
   ```

   When enabled in the build, the testbench emits `build/ddr2_trace.csv` with a per‑cycle trace of key host and memory‑side signals.

For the full list of supported modes and monitors, see `test/TESTING.md`.

5. **UVM testbench (Verilator + UVM-2017)**

   A SystemVerilog UVM testbench lives in `test-uvm/` and uses the IEEE UVM-2017 library under `tools/uvm-2017`. It provides structured tests (e.g. init, scalar/block R/W, address edges, stress) and protocol checkers. Build and run instructions are in `test-uvm/README_UVM.md`.

---

## Host Interface Summary

At the top level, `ddr2_controller` exposes a simple, synchronous interface to a host system:

- **Inputs**
  - `CLK` (500 MHz controller clock), `RESET`, `INITDDR`.
  - `CMD[2:0]`: encoded command (`NOP`, `SCR`, `SCW`, `BLR`, `BLW`).
  - `SZ[1:0]`: block size (1–4 bursts of 8 words).
  - `ADDR[24:0]`: logical word address.
  - `DIN[15:0]`: write data.
  - `FETCHING`: pops read results from the return FIFO.

- **Outputs**
  - `READY`: high when initialization has completed.
  - `NOTFULL`: flow‑control indication that input FIFOs can accept more commands/data.
  - `FILLCOUNT[6:0]`: current occupancy of the write‑data FIFO.
  - `DOUT[15:0]`: read data.
  - `RADDR[24:0]`: address tag associated with `DOUT`.
  - `VALIDOUT`: high when `DOUT`/`RADDR` are valid.

Address mapping, timing and protocol details are documented in the project `docs/` and in the JEDEC JESD79-2 specification.

---

## Compliance Disclaimer

This controller and its accompanying monitors are **inspired by and aligned with** the JEDEC DDR2 specification, but:

- They are **not guaranteed to be fully compliant** with all aspects of any specific DDR2 JEDEC standard or speed grade.
- All timing parameters and behavioral assumptions are tuned for the reference Micron `mt47h32m16_37e` model and a specific simulation environment.
- Real‑world behavior may differ depending on process, voltage, temperature, synthesis/implementation details, and specific memory devices.

**Use this core at your own risk.** Before deploying in silicon or production systems, you must perform your own timing closure, formal/functional verification, and lab validation against the actual target DDR2 devices and operating conditions.

If you identify issues, missing checks, or opportunities to improve JEDEC alignment, **please open an issue or pull request** so that we can track and incorporate improvements.

---

## Contributing

Contributions are welcome, especially in the following areas:

- Improved timing parameterization for additional DDR2 speed grades or devices.
- Additional monitors and negative tests in `test/`.
- Bug fixes or clarifications in `docs/`, `test/TESTING.md`, and `test-uvm/README_UVM.md`.
- Porting the testbench to additional simulators or adding CI scripts.

If you plan a larger change, consider opening an issue first to discuss scope and direction.

---

## License

Unless otherwise noted in individual files, the contents of this repository are licensed under the **Creative Commons Attribution 4.0 International (CC BY 4.0)** license.

By contributing to this project, you agree that your contributions will be licensed under CC BY 4.0 as well.

For the full legal text of the license, please refer to the official Creative Commons site for **CC BY 4.0**.
