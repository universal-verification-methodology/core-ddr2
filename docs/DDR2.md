# DDR2 Controller Architecture Specification

This document specifies the optimal DDR2 SDRAM controller architecture derived from analysis of multiple open-source implementations. This architecture will serve as the foundation for Verilog module development.

---

## Table of Contents

1. [Overview](#overview)
2. [Top-Level Architecture](#top-level-architecture)
3. [External Interfaces](#external-interfaces)
   - [Host / Front-End Interface](#host--front-end-interface)
   - [DDR2 Device Interface](#ddr2-device-interface)
   - [Clocks and Reset](#clocks-and-reset)
4. [Module / Block Overview](#module--block-overview)
   - [Initialization Engine (`ddr2_init_engine`)](#initialization-engine-ddr2_init_engine)
   - [Protocol Engine (`ddr2_protocol_engine`)](#protocol-engine-ddr2_protocol_engine)
   - [FIFOs](#fifos)
   - [Read-Data Ring Buffer (`ddr2_ring_buffer`)](#read-data-ring-buffer-ddr2_ring_buffer)
   - [PHY / SSTL Interface (`ddr2_phy`)](#phy--sstl-interface-ddr2_phy)
5. [Command and Data Flow](#command-and-data-flow)
6. [Timing Specifications](#timing-specifications)
7. [Command Encoding and Address Mapping](#command-encoding-and-address-mapping)
8. [State Machines](#state-machines)
9. [Design Assumptions and Constraints](#design-assumptions-and-constraints)
10. [Verification Strategy](#verification-strategy)
11. [Summary](#summary)

---

## Overview

- **Design name**: `ddr2_controller`
- **Target device**: Micron `mt47h32m16_37e` (512 Mb, x16 data width, 4 banks)
- **System clock (`CLK`)**: 500 MHz (controller/core clock)
- **DDR2 clock (`ck`)**: 250 MHz (generated internally as divide-by-2 of `CLK`)
- **Burst length (`BL`)**: 8 (parameter `BL = 3'b011`)
- **CAS latency (`CL`)**: 4 (parameter `CL = 3'b100`)
- **Additive latency (`AL`)**: 4 (parameter `AL = 3'b100`)
- **Burst type (`BT`)**: Sequential (`BT = 1'b0`)

**Goal**: The controller exposes a simple, FIFO-like command/data interface to a host and translates those commands into JEDEC-compliant DDR2 transactions. It handles full JEDEC power-up initialization, periodic refresh, scalar and block reads/writes, DQS-based data capture via an input ring buffer, and basic flow-control back-pressure. The design prioritizes correctness, clarity, and maintainability.

---

## Top-Level Architecture

The architecture is organized into five main layers:

```
┌─────────────────────────────────────────────────────────┐
│                    Host Interface                        │
│  (CMD, ADDR, DIN, DOUT, READY, VALIDOUT, NOTFULL)      │
└────────────────────┬──────────────────────────────────────┘
                     │
┌────────────────────▼────────────────────────────────────┐
│              ddr2_controller (Top Module)               │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐ │
│  │ Command FIFO │  │  Data FIFO  │  │ Return FIFO  │ │
│  │  (64 deep)   │  │  (64 deep)   │  │  (64 deep)   │ │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘ │
│         │                  │                  │         │
│  ┌──────▼─────────────────▼──────────────────▼───────┐ │
│  │         Protocol Engine (process_logic)           │ │
│  │  - Command decode & sequencing                     │ │
│  │  - ACTIVATE/READ/WRITE/REFRESH control            │ │
│  │  - Timing management                              │ │
│  │  - Refresh scheduling                             │ │
│  └──────┬─────────────────────────────────────────────┘ │
│         │                                                │
│  ┌──────▼──────────────────────────────────────────────┐ │
│  │      Initialization Engine (ddr2_init_engine)      │ │
│  │  - JEDEC power-up sequence                         │ │
│  │  - Mode register programming                      │ │
│  │  - DLL enable/reset                               │ │
│  └──────┬─────────────────────────────────────────────┘ │
│         │                                                │
│  ┌──────▼──────────────────────────────────────────────┐ │
│  │      Read Data Ring Buffer (ddr2_ring_buffer8)     │ │
│  │  - DQS-based capture                                │ │
│  │  - 8-word buffer                                    │ │
│  └──────┬──────────────────────────────────────────────┘ │
│         │                                                │
│  ┌──────▼──────────────────────────────────────────────┐ │
│  │         PHY / SSTL Interface (ddr2_phy)             │ │
│  │  - SSTL18 I/O cells                                 │ │
│  │  - Bidirectional DQ/DQS                            │ │
│  │  - Command/address/control signals                  │ │
│  └──────┬──────────────────────────────────────────────┘ │
└─────────┼────────────────────────────────────────────────┘
          │
          │ DDR2 Pins (CK, CKE, CS#, RAS#, CAS#, WE#,
          │              BA, A, DQ, DQS, DM, ODT)
          │
┌─────────▼──────────────────────────────────────────────┐
│         DDR2 SDRAM Device (mt47h32m16_37e)             │
└─────────────────────────────────────────────────────────┘
```

### Main Sub-Blocks

1. **`ddr2_init_engine`**: Performs JEDEC-compliant power-up, precharge, refresh, and mode-register programming.
2. **`ddr2_protocol_engine`**: Core protocol engine; sequences ACTIVATE/READ/WRITE/REFRESH, manages latencies, block transfers, and coarse timing/refresh counters.
3. **FIFO instances**: Input data FIFO, input command FIFO, and return FIFO for `{address, data}` pairs.
4. **`ddr2_ring_buffer8`**: 8-deep DDR input ring buffer that captures one 16‑bit word per `CLK` cycle after a `listen` pulse (clk-synchronous capture aligned with the simplified DDR2 model).
5. **`ddr2_phy`**: PHY/SSTL interface providing pad-level SSTL18 DDR2 signaling and generating `CK`/`CK#` as a divide‑by‑2 of `CLK`.
6. **`ddr2_server_controller` (wrapper, optional)**: 64‑bit host-facing wrapper that instantiates four `ddr2_controller` slices in lockstep, with slice 0 connected to the external DDR2 pins and slices 1–3 used for host‑visible data widening (primarily in simulation today).

---

## External Interfaces

### Host / Front-End Interface

User-visible side, as exposed by `ddr2_controller`:

| Signal      | Dir | Width | Description |
|-------------|-----|-------|-------------|
| `CLK`       | In  | 1     | 500 MHz main controller clock. All internal state machines run on this clock; a divided-by-2 version drives the DDR interface clock. |
| `RESET`     | In  | 1     | Active-high synchronous reset for all internal logic. When `1`, clears FIFOs, state machines, and restarts initialization. |
| `INITDDR`   | In  | 1     | One-cycle pulse to start DDR2 initialization. Passed to `ddr2_init_engine.init`. |
| `CMD`       | In  | 3     | Encoded command: `000`/`111`=NOP, `001`=SCR, `010`=SCW, `011`=BLR, `100`=BLW |
| `SZ`        | In  | 2     | Block size for `BLR`/`BLW`. Encodes the number of 8-word bursts (1-4 bursts) |
| `ADDR`      | In  | 25    | Logical word address. Internally split into row, bank, and column. |
| `DIN`       | In  | 16    | Write data bus for scalar and block writes. Sampled when `put_dataFIFO` is asserted. |
| `FETCHING`  | In  | 1     | Read data consumption signal. When high, the return FIFO pops entries each cycle. |
| `DOUT`      | Out | 16    | Read data bus, sourced from the return FIFO (`FIFO_RETURN`). Valid when `VALIDOUT` is high. |
| `RADDR`     | Out | 25    | Address tag associated with `DOUT`; typically the original command address plus an offset for burst indices. |
| `FILLCOUNT` | Out | 7     | Occupancy of the input data FIFO (`FIFO_DATA`), in words. Used by the host to avoid overfilling. |
| `READY`     | Out | 1     | High when `ddr2_init_engine` has finished initialization and normal operation is allowed. |
| `VALIDOUT`  | Out | 1     | `1` when `DOUT`/`RADDR` contain a valid word popped from the return FIFO. |
| `NOTFULL`   | Out | 1     | Flow-control signal asserting that input FIFOs can accept more command/data. |

### DDR2 Device Interface

Pad-level DDR2 pins, as exposed by `ddr2_controller`:

**Command / address**:

| Signal                 | Dir | Width | Description |
|------------------------|-----|-------|-------------|
| `C0_A_PAD`             | Out | 13    | Row/column address bus to DDR2 (`A[12:0]`). |
| `C0_BA_PAD`            | Out | 2     | Bank address (`BA[1:0]`). |
| `C0_RASBAR_PAD`        | Out | 1     | `RAS#` row-address strobe. |
| `C0_CASBAR_PAD`        | Out | 1     | `CAS#` column-address strobe. |
| `C0_WEBAR_PAD`         | Out | 1     | `WE#` write enable. |
| `C0_CSBAR_PAD`         | Out | 1     | `CS#` chip select for this rank. |
| `C0_CK_PAD`            | Out | 1     | Positive DDR2 clock (`ck`). Driven from internal divided clock. |
| `C0_CKBAR_PAD`         | Out | 1     | Negative DDR2 clock (`ckbar`). |
| `C0_CKE_PAD`           | Out | 1     | Clock enable (`CKE`). Driven primarily by `ddr2_init_engine`. |
| `C0_ODT_PAD`           | Out | 1     | On-die termination control (`ODT`). |
| `C0_DM_PAD`            | Out | 2     | Data mask per byte lane; `2'b00` = valid data, `2'b11` = masked. |

**Data / strobes**:

| Signal          | Dir   | Width | Description |
|-----------------|--------|-------|-------------|
| `C0_DQ_PAD`     | Inout | 16    | Bidirectional data bus (`DQ[15:0]`). |
| `C0_DQS_PAD`    | Inout | 2     | Data strobes for DDR data capture (`DQS[1:0]`). |
| `C0_DQSBAR_PAD` | Inout | 2     | Complementary DQS strobes (`DQS#[1:0]`). |

### Clocks and Reset

- **`CLK`**:
  - Single global controller clock, 500 MHz, provided by the host.
  - All state machines in `ddr2_controller`, `ddr2_init_engine`, `ddr2_protocol_engine`, FIFOs, and ring buffer control logic are synchronous to `CLK`.

- **`ck` / DDR2 clock**:
  - Internal `reg ck` toggled on each `posedge CLK`, forming a 250 MHz DDR2 clock.
  - Routed to the PHY interface and out to the pads `C0_CK_PAD` / `C0_CKBAR_PAD`.

- **Reset**:
  - `RESET` is active-high and synchronous to `CLK`.
  - When `RESET` is asserted:
    - FIFOs are cleared.
    - `ddr2_init_engine` returns to its initial state.
    - `ddr2_protocol_engine` returns to `IDLE`.
    - `READY` is de-asserted.

- **Clock-domain crossing**:
  - No asynchronous CDC: `ck` is a clean divide-by-2 of `CLK`, and all logic is clocked by `CLK`.
  - DDR2 interface timing (DQS/DQ vs. `ck`) is handled through precise edge-based logic and dedicated delay cells.

---

## Module / Block Overview

### Initialization Engine (`ddr2_init_engine`)

**Role**: Implements the JEDEC DDR2 power-up and initialization sequence, including power-up waits, precharge, refresh, DLL enable/reset, and mode-register programming. It drives DDR2 command/address pins during initialization and raises `ready` once the memory is prepared for normal transactions.

**Parameters**:
- `BL` – Burst length (8)
- `BT` – Burst type (sequential)
- `CL` – CAS latency (4)
- `AL` – Additive latency (4)

**Interface summary**:

| Port     | Dir | Width | Description |
|----------|-----|-------|-------------|
| `clk`    | In  | 1     | 500 MHz controller clock. |
| `reset`  | In  | 1     | Active-high synchronous reset; restarts the initialization FSM. |
| `init`   | In  | 1     | Start pulse from `INITDDR`. |
| `ready`  | Out | 1     | High when initialization sequence is complete. |
| `csbar`  | Out | 1     | Chip select during initialization. |
| `rasbar` | Out | 1     | RAS# during initialization. |
| `casbar` | Out | 1     | CAS# during initialization. |
| `webar`  | Out | 1     | WE# during initialization. |
| `ba`     | Out | 2     | Bank address bus during initialization. |
| `a`      | Out | 13    | Address bus during initialization (MRS/EMRS operands, precharge addresses, etc.). |
| `dm`     | Out | 2     | Data mask during initialization (typically `2'b00`). |
| `odt`    | Out | 1     | ODT state during initialization. |
| `ts_con` | Out | 1     | Tristate control to the PHY during init (e.g., bus idle). |
| `cke`    | Out | 1     | Clock enable, raised at the correct time in the sequence. |

**Initialization Sequence**:

1. **Power-up wait**: 200 µs NOP window with `CKE = 0` (100,000 cycles at 500 MHz)
2. **CKE assertion**: Transition to `CKE = 1` and wait tXSR window (199 cycles)
3. **Precharge all**: Issue `PRECHARGE` command with `A[10]=1`
4. **Mode register programming**:
   - EMRS2: Set DLL enable, ODT off
   - EMRS3: Reserved bits
   - EMRS1: Set AL, ODT, DLL enable
   - MRS: Set BL, BT, CL, DLL reset
5. **DLL reset**: Issue MRS with DLL reset bit, then re-program without reset
6. **Auto refresh**: Issue two `AUTO REFRESH` commands with tRFC waits (399 cycles each)
7. **Final configuration**: Re-program MRS and EMRS1 with final parameters (no DLL reset; ODT on)
8. **Completion**: Final precharge, ODT enable, then assert `ready`

**State Machine**: Uses a 17-bit down-counter `cntr_reg` and state register (`S0`–`S27`) to walk through the timing diagram.

### Protocol Engine (`ddr2_protocol_engine`)

**Role**: Main DDR2 transaction engine. It:
- Pulls commands from `FIFO_CMD` and optional write data from `FIFO_DATA`
- Decodes `CMD`, `SZ`, `ADDR`
- Sequences DDR2 commands: ACTIVATE, READ, WRITE, PRECHARGE, REFRESH, and NOP
- Manages refresh via an internal refresh counter
- Controls the read-data ring buffer and return FIFO
- Generates DQS, DM, and DQ timing for writes, plus direction signaling for the SSTL interface

**Key inputs**:
- `ready_init_i`, `clk`, `CK`, `reset`
- `addr_cmdFIFO_i`, `cmd_cmdFIFO_i`, `sz_cmdFIFO_i`, `din_dataFIFO_i`
- `emptyBar_cmdFIFO_i` (command FIFO not empty)
- `fullBar_returnFIFO_i`, `fillcount_returnFIFO_i` (return FIFO status)

**Key outputs**:
- DDR2 control pins: `csbar_o`, `rasbar_o`, `casbar_o`, `webar_o`, `ba`, `a`
- FIFO control: `get_cmdFIFO_o`, `get_dataFIFO_o`, `put_returnFIFO_o`, `addr_returnFIFO_o`
- Ring buffer control: `listen_ringBuff_o`, `readPtr_ringBuff_o`
- PHY interface: `dq_SSTL_o`, `dm_SSTL_o`, `dqs_i_SSTL_o`, `ts_i_o`, `ri_i_o`

**Major internal state machines**:

1. **Main command FSM** (`state`, 4-bit):
   - `IDLE`: Waits for either refresh trigger (via `refCnt_reg`) or commands in FIFO
   - `GETCMD`: Pops an entry from `FIFO_CMD`, latches address, command, and size
   - `ACTIVATE`: Issues `ACTIVATE`, opens the row; then branches to read (`SCREAD`) or write (`SCWRITE`) flows, or block variants (`BLR`/`BLW`)
   - **Read sub-states**: `SCREAD`, `SCRLSN`, `SCRNLSN`, `SCRDNTH`, `SCREND` – handle read latency, ring buffer listen pulses, pointer advancement, and return FIFO pushes
   - **Write sub-states**: `SCWRITE`, `SCWRDATA`, `SCWREND` – handle write latency, data fetch from FIFO, DQ/DM driving, DQS generation, and bus turnaround
   - **Refresh sub-states**: `RPRE`, `RNOP`, `RREF`, `RIDL` – perform precharge-all, auto refresh, and required NOP windows

2. **Block operation FSM** (`block_state`, 2-bit):
   - `B_IDLE`, `B_ACT`, `B_WRT`, `B_RD` – handle additional ACTIVATE + READ/WRITE sequences for extended block transfers, including bank interleaving
   - Uses `szBlk_reg`, `blkCnt_reg`, `flag`, and `addr_cmdFIFO_reg` to implement multi-burst block operations

**Timing Management**:
- Uses 6-bit down-counter `cnt_reg` for sub-timing (NOP windows, ACT to RD/WR delay, burst windows)
- Refresh counter: 12-bit `refCnt_reg` counts down from 3900; triggers refresh when below 100
- All timing values are parameterized and derived from JEDEC specifications

### FIFOs

Three FIFO instances in `ddr2_controller`:

**Command FIFO (`FIFO_CMD`)**:
- **Module**: `FIFO #(33, 6)`
- **Width**: 33 bits (`{ADDR[24:0], CMD[2:0], SZ[1:0], OP[2:0]}`)
- **Depth**: 64 entries (2^6)
- **Producer**: Host via `CMD`, `SZ`, `OP`, `ADDR`
- **Consumer**: `ddr2_protocol_engine`, via `get_cmdFIFO_o`
- **Status**: `notfull_cmdFIFO` (full_bar), `emptyBar_cmdFIFO`

**Data FIFO (`FIFO_DATA`)**:
- **Module**: `FIFO #(16, 6)`
- **Width**: 16 bits (write data)
- **Depth**: 64 entries
- **Producer**: Host via `DIN` and `put_dataFIFO`
- **Consumer**: `ddr2_protocol_engine`, via `get_dataFIFO_o`
- **Status**: `FILLCOUNT` (external), `notfull_dataFIFO`, `emptyBar_dataFIFO`

**Return FIFO (`FIFO_RETURN`)**:
- **Module**: `FIFO #(41, 6)`
- **Width**: 41 bits (`{RADDR[24:0], DOUT[15:0]}`)
- **Depth**: 64 entries
- **Producer**: `ddr2_protocol_engine` and ring buffer (`addr_returnFIFO`, `ringBuff_returnFIFO`, `put_returnFIFO`)
- **Consumer**: Host through `FETCHING` (connected to FIFO `get`)
- **Status**: `emptyBar_returnFIFO` (used to generate `VALIDOUT`), `fullBar_returnFIFO` (used to protect read path), `fillcount_returnFIFO`

**Flow control**:
- `NOTFULL` is computed as: `!(FILLCOUNT >= 33 || (!notfull_cmdFIFO))`
- Ensures at least 32 free entries remain in `FIFO_DATA` (worst-case block write is 32 words)

### Read-Data Ring Buffer (`ddr2_ring_buffer8`)

**Role**: Captures an 8-word DDR burst using both edges of DQS and provides word-indexed access to the captured data. Acts as a small elastic buffer and strobe phase aligner.

**Interface**:

| Port      | Dir | Width | Description |
|-----------|-----|-------|-------------|
| `listen`  | In  | 1     | One-cycle pulse from `ddr2_protocol_engine` to arm the buffer for the next burst. |
| `strobe`  | In  | 1     | DDR2 DQS strobe (from SSTL interface: `dqs_o[0]`). |
| `reset`   | In  | 1     | Reset for internal registers and counters. |
| `din`     | In  | 16    | Incoming data from DDR2 (`dq_o`). |
| `readPtr` | In  | 3     | Index (0–7) selecting which captured word to present. |
| `dout`    | Out | 16    | Corresponding buffered word, to be pushed into return FIFO. |

**Behavior**:
- A chain of delay cells delays `strobe` to produce `dStrobe`, then gated by `listen`/`F0` to form `fStrobe`
- On `posedge listen`, `F0` is set to `1`, enabling `fStrobe`
- A 2-bit `count` register increments on `posedge fStrobeBar`:
  - When `count < 3`: `count <= count + 1`
  - When `count == 3`: `count <= 0` and `F0 <= 0` (disabling further captures)
- Data capture:
  - On `posedge fStrobe`, with `count` = 0,1,2,3, captures `din` into `r0`, `r2`, `r4`, `r6` respectively
  - On `negedge fStrobe`, with `count` = 0,1,2,3, captures `din` into `r1`, `r3`, `r5`, `r7` respectively
- With one `listen` pulse, `count` cycles 0→1→2→3→0, capturing exactly 8 samples (`r0..r7`) over four DQS cycles (eight edges), then automatically de-arms via `F0` clearing

### PHY / SSTL Interface (`ddr2_phy`)

**Role**: Maps core-level signals to pad-level SSTL18 I/Os and handles bidirectional DQ/DQS and ODT.

**Key signals**:
- Inputs from controller:
  - `ck_i`, `cke_i`, `csbar_i`, `rasbar_i`, `casbar_i`, `webar_i`, `ba_i`, `a_i`
  - `dq_i`, `dqs_i`, `dm_i`, `odt_i`
  - Direction control: `ts_i` (write enable), `ri_i` (read indicator)
- Outputs to pads:
  - `ck_pad`, `ckbar_pad`, `cke_pad`, `csbar_pad`, `rasbar_pad`, `casbar_pad`, `webar_pad`
  - `ba_pad`, `a_pad`, `dm_pad`, `odt_pad`
  - `dq_pad`, `dqs_pad`, `dqsbar_pad`

**Muxing of init vs. normal control**:
- When `READY = 0`, all command/address pins are driven by `ddr2_init_engine`
- When `READY = 1`, `ddr2_protocol_engine` takes over (except `CKE`/`ODT` which remain under init engine control)

---

## Command and Data Flow

End-to-end narrative from host request to DDR2 access and result return:

1. **User issues request**:
   - Host drives `CMD`, `SZ`, `OP`, `ADDR`, `DIN` (for writes) and `FETCHING`, respecting `NOTFULL`
   - `ddr2_controller` combines `{ADDR, CMD, SZ, OP}` into a 33-bit word and enqueues it in `FIFO_CMD` if allowed
   - For writes, it also enqueues data words in `FIFO_DATA` through `put_dataFIFO`, with duration determined by `CMD` (`SCW` vs `BLW`) and `SZ`

2. **Command fetch and decode**:
   - Once `READY` is high and `FIFO_CMD` is non-empty, `ddr2_protocol_engine` asserts `get_cmdFIFO_o` and transitions to `GETCMD`
   - It decodes command type, extracts `row_address`, `bank_address`, and `column_address`, and latches size fields

3. **DDR2 protocol execution**:
   - **ACTIVATE**: Opens the appropriate row in the target bank; wait tRCD
   - **READ path**:
     - For `SCR`/`BLR`, issues a `READ` command, then waits `AL + CL` cycles
     - For `BLR`, may schedule additional `READ`s and bank interleaving if `SZ` indicates multiple bursts
   - **WRITE path**:
     - For `SCW`/`BLW`, issues a `WRITE` command, then waits write latency
     - Pops data from `FIFO_DATA`, drives `DQ`, `DM`, and toggles `DQS`
   - **REFRESH**:
     - When the internal refresh counter expires, the FSM preempts normal traffic in `IDLE` and issues precharge-all + auto-refresh sequence

4. **Data capture / generation**:
   - **Writes**:
     - On `negedge clk`, `dq_SSTL_o` and `dm_SSTL_o` are updated according to `din_dataFIFO_i` and whether valid data is being written
     - On `posedge clk`, `dqs_i_SSTL_o` toggles during active write windows; `ts_i_o`/`ri_i_o` control the I/O direction
   - **Reads**:
     - When read latency expires, `ddr2_protocol_engine` pulses `listen_ringBuff_o`
     - `ddr2_ring_buffer8` samples `dq_o` on both DQS edges and holds 8 captured words addressed by `readPtr_ringBuff_o`

5. **Result return**:
   - `ddr2_protocol_engine` steps `readPtr_ringBuff_o` through 0..7 and updates `addr_returnFIFO_o` for each data word
   - It asserts `put_returnFIFO_o` as required to push `{RADDR,DOUT}` pairs into `FIFO_RETURN`
   - The host asserts `FETCHING` to pop entries from `FIFO_RETURN`; `VALIDOUT` goes high when new data appears at `DOUT` & `RADDR`

6. **Refresh policy**:
   - A 12-bit `refCnt_reg` counts down from a pre-loaded value (3900)
   - When below threshold (100) and `ready_init_i` is high, `IDLE` transitions into refresh states (`RPRE` → `RNOP` → `RREF` → `RIDL`)
   - Normal command processing resumes only after the refresh sequence finishes, ensuring DDR2 refresh requirements are met

---

## Timing Specifications

All timing values below are in units of `CLK` cycles (500 MHz). Exact cycle counts are parameterized and derived from JEDEC specifications.

### Initialization Engine Timing (`ddr2_init_engine`)

Counter preload values from `cntr_reg` (17-bit down-counter):

| State | Counter Value (hex) | Cycles | Purpose |
|-------|---------------------|--------|---------|
| S1    | 0x186A0            | 100,000 | 200 µs power-up wait |
| S2    | 0x00C7              | 199    | tXSR wait after CKE=1 |
| S3    | 0x0001              | 1      | PRECHARGE command duration |
| S4    | 0x0007              | 7      | NOP after PRECHARGE (tRP) |
| S5-S27| 0x0003              | 3      | NOP after LM commands (tMRD) |
| S15   | 0x018F              | 399    | NOP after first AUTO REFRESH (tRFC) |
| S17   | 0x018F              | 399    | NOP after second AUTO REFRESH (tRFC) |
| S19   | 0x0195              | 405    | NOP after final MRS (tMRD + DLL lock) |
| S26   | 0x0004              | 4      | Final wait before `ready=1` |

### Protocol Engine Timing (`ddr2_protocol_engine`)

Counter preload values from `cnt_reg` (6-bit down-counter):

| State/Operation | Counter Value/Expression | Cycles | Purpose |
|-----------------|--------------------------|--------|---------|
| `GETCMD`        | 0x02                     | 2      | FIFO read latency wait |
| `ACTIVATE`      | 0x01                     | 1      | tRCD (row-to-column delay) |
| `SCREAD`        | `(2 * (AL + CL - 1) - 1)` | 13     | Read latency wait (with AL=4, CL=4: 2*(4+4-1)-1 = 13) |
| `SCWRITE`       | `(2*(AL + CL - 1) - 2)`  | 12     | Write latency wait (with AL=4, CL=4: 2*(4+4-1)-2 = 12) |
| `SCWRDATA` (SCW) | 0x07                     | 7      | Data write duration for scalar write |
| `SCWRDATA` (BLW SZ=00) | 0x07                     | 7      | Data write duration (8 words) |
| `SCWRDATA` (BLW SZ=01) | 0x0F                     | 15     | Data write duration (16 words) |
| `SCWRDATA` (BLW SZ=10) | 0x17                     | 23     | Data write duration (24 words) |
| `SCWRDATA` (BLW SZ=11) | 0x1F                     | 31     | Data write duration (32 words) |
| `SCWREND`       | 0x11                     | 17     | Post-write NOP window |
| `SCRLSN`        | 0x00                     | 0      | Immediate transition (ring buffer listen pulse) |
| `SCRNLSN`       | 0x05                     | 5      | Ring buffer pointer stepping delay |
| `SCRDNTH`       | 0x03                     | 3      | Ring buffer data capture wait |
| `SCREND`        | 0x03                     | 3      | Return FIFO push alignment |
| `RPRE` (CK=1)   | 0x01                     | 1      | PRECHARGE command duration |
| `RPRE` (CK=0)   | 0x02                     | 2      | PRECHARGE command duration (aligned to CK edge) |
| `RNOP`          | 0x07                     | 7      | tRP wait after precharge |
| `RREF`          | 0x01                     | 1      | AUTO REFRESH command duration |
| `RIDL`          | 0x37                     | 55     | tRFC wait after refresh |

### Refresh Counter Parameters

From `refCnt_reg` (12-bit down-counter):

| Parameter | Value (hex) | Decimal | Purpose |
|-----------|-------------|---------|---------|
| Initial value | 0xF3C      | 3900    | Starting countdown value |
| Threshold | 0x064      | 100     | Trigger refresh when below this value |
| Reload value | 0xF3C      | 3900    | Reset after refresh completes |
| Effective refresh interval | 3800 cycles | 3800    | Time between refresh triggers (7.6 µs at 500 MHz) |

### Block Read Safety Thresholds

From `ddr2_protocol_engine` ACTIVATE state, BLR path. These thresholds check `fillcount_returnFIFO_i` before starting a block read:

| `SZ` | Threshold (hex) | Decimal | Minimum free space required |
|------|------------------|---------|----------------------------|
| 00   | 0x38             | 56      | Return FIFO must have < 56 entries (≥ 8 free) |
| 01   | 0x30             | 48      | Return FIFO must have < 48 entries (≥ 16 free) |
| 10   | 0x28             | 40      | Return FIFO must have < 40 entries (≥ 24 free) |
| 11   | 0x20             | 32      | Return FIFO must have < 32 entries (≥ 32 free) |

---

## Command Encoding and Address Mapping

### Command Encoding

Internal encodings (in both `ddr2_controller` and `ddr2_protocol_engine`):

```verilog
NOP = 3'b000;
SCR = 3'b001;  // Scalar read
SCW = 3'b010;  // Scalar write
BLR = 3'b011;  // Block read
BLW = 3'b100;  // Block write
// 101, 110, 111 reserved
```

Host-visible `CMD[2:0]` mapping:

| Bits | Meaning                         |
|------|---------------------------------|
| `000`| NOP                             |
| `001`| Scalar read (SCR)               |
| `010`| Scalar write (SCW)              |
| `011`| Block read (BLR)                |
| `100`| Block write (BLW)               |
| `101`| Reserved                        |
| `110`| Reserved                        |
| `111`| NOP                             |

Block size field `SZ[1:0]`:

| `SZ` | Bursts (8 words each) | Words per block |
|------|------------------------|-----------------|
| 00   | 1                      | 8               |
| 01   | 2                      | 16              |
| 10   | 3                      | 24              |
| 11   | 4                      | 32              |

### Address Mapping

Inside `ddr2_protocol_engine`:

- **Row**: `ADDR[24:12]` → `row_address` (13 bits)
- **Bank**: `ADDR[4:3]` → `bank_address` (2 bits)
- **Column**: `{ADDR[11:5], ADDR[2:0]}` → `column_address` (10 bits)

This supports:
- 13-bit row, 2-bit bank, and 10-bit column (plus lower 3 bits to step through burst offsets)
- For bursts, `column_address` and internal address registers are incremented by 8 words per sub-burst, and `addr_returnFIFO_o` is incremented by 1 per returned word

---

## Mode Register Programming

### Mode Register Set (MRS)

The MRS register is programmed during initialization to configure DDR2 operating parameters:

| Bit(s) | Name | Value | Description |
|--------|------|-------|-------------|
| [2:0] | Burst Length | `3'b011` | BL = 8 |
| [3] | Burst Type | `1'b0` | Sequential |
| [6:4] | CAS Latency | `3'b100` | CL = 4 |
| [7] | Test Mode | `1'b0` | Normal operation |
| [8] | DLL Reset | `1'b1` (during init) / `1'b0` (final) | DLL reset enable |
| [9] | Write Recovery | `2'b00` | tWR = 4 cycles |
| [11:10] | Reserved | `2'b00` | Must be 0 |
| [12] | Reserved | `1'b0` | Must be 0 |

**MRS Register Value**:
- During DLL reset: `A[12:0] = {1'b0, 2'b00, 1'b0, 2'b00, 1'b1, 3'b100, 1'b0, 3'b011}`
- Final configuration: `A[12:0] = {1'b0, 2'b00, 1'b0, 2'b00, 1'b0, 3'b100, 1'b0, 3'b011}`

### Extended Mode Register Set (EMRS1)

EMRS1 configures additional DDR2 features:

| Bit(s) | Name | Value | Description |
|--------|------|-------|-------------|
| [2:0] | Output Drive Strength | `3'b000` | Normal |
| [3] | Reserved | `1'b0` | Must be 0 |
| [5:3] | Additive Latency | `3'b100` | AL = 4 |
| [6] | ODT | `1'b0` (during init) / `1'b1` (final) | ODT enable |
| [7] | DLL Enable | `1'b1` | DLL enabled |
| [9:8] | Reserved | `2'b00` | Must be 0 |
| [10] | Reserved | `1'b0` | Must be 0 |
| [12:11] | Reserved | `2'b00` | Must be 0 |

**EMRS1 Register Value**:
- During init: `A[12:0] = {2'b00, 1'b0, 2'b00, 1'b1, 3'b100, 1'b0, 1'b0, 3'b000}`
- Final configuration: `A[12:0] = {2'b00, 1'b0, 2'b00, 1'b1, 3'b100, 1'b1, 1'b0, 3'b000}`

### Extended Mode Register Set (EMRS2)

EMRS2 configures additional features (typically minimal configuration):

| Bit(s) | Name | Value | Description |
|--------|------|-------|-------------|
| [2:0] | Reserved | `3'b000` | Must be 0 |
| [9:3] | Reserved | `7'b0000000` | Must be 0 |
| [12:10] | Reserved | `3'b000` | Must be 0 |

**EMRS2 Register Value**: `A[12:0] = 13'b0`

### Extended Mode Register Set (EMRS3)

EMRS3 is typically not used (all zeros):

**EMRS3 Register Value**: `A[12:0] = 13'b0`

### Mode Register Programming Sequence

1. **EMRS2**: Set to default (all zeros)
2. **EMRS3**: Set to default (all zeros)
3. **EMRS1**: Set DLL enable, AL=4, ODT off
4. **MRS**: Set BL=8, BT=sequential, CL=4, DLL reset enabled
5. **Wait**: Allow DLL to lock (tMRD + DLL lock time)
6. **MRS**: Re-program without DLL reset
7. **EMRS1**: Re-program with ODT enabled

---

## Additional Timing Parameters

### JEDEC Timing Parameters

All timing values are in DDR2 clock cycles (250 MHz = 4 ns per cycle):

| Parameter | Symbol | Value (cycles) | Value (ns) | Description |
|-----------|--------|---------------|-----------|-------------|
| Row-to-Column Delay | tRCD | 2 | 8 | ACTIVE to READ/WRITE delay |
| Row Precharge Time | tRP | 2 | 8 | PRECHARGE command period |
| Row Active Time | tRAS | 10 | 40 | ACTIVE to PRECHARGE delay (minimum) |
| Row Cycle Time | tRC | 12 | 48 | ACTIVE to ACTIVE delay (same bank) |
| Refresh Cycle Time | tRFC | 50 | 200 | AUTO REFRESH command period |
| Mode Register Set Delay | tMRD | 2 | 8 | MRS command period |
| CAS Latency | CL | 4 | 16 | READ command to data valid |
| Additive Latency | AL | 4 | 16 | Additional latency for command scheduling |
| CAS Write Latency | CWL | 3 | 12 | WRITE command to data valid |
| Write Recovery Time | tWR | 4 | 16 | Last data to PRECHARGE delay |
| Read-to-Precharge | tRTP | 2 | 8 | READ to PRECHARGE delay |
| Write-to-Read Turnaround | tWTR | 2 | 8 | WRITE to READ delay (same bank) |
| Read-to-Write Turnaround | tRTW | 4 | 16 | READ to WRITE delay (same bank) |
| Four Activate Window | tFAW | 20 | 80 | Four ACTIVE commands window |
| Exit Self-Refresh | tXSR | 50 | 200 | Exit self-refresh to ACTIVE delay |
| DLL Lock Time | tDLLK | 200 | 800 | DLL lock time after reset |

### Write Latency Calculation

Write latency (WL) = AL + CWL = 4 + 3 = 7 cycles

### Read Latency Calculation

Read latency = AL + CL = 4 + 4 = 8 cycles

### Auto-Precharge Usage

- **A[10] = 1**: Auto-precharge enabled (row closes after burst completes)
- **A[10] = 0**: No auto-precharge (row remains open)

**Usage**:
- Scalar operations: Auto-precharge enabled (`A[10] = 1`)
- Block operations: Auto-precharge on last burst only (`A[10] = 1` on final burst, `A[10] = 0` on intermediate bursts)

---

## ODT (On-Die Termination) Control

### ODT Timing

ODT is controlled by the initialization engine and protocol engine:

| Phase | ODT State | Timing |
|-------|-----------|--------|
| Power-up | Low | During initialization |
| After EMRS1 (final) | High | Enabled after final mode register programming |
| During writes | High | Asserted during write operations |
| During reads | Low | De-asserted during read operations |
| During refresh | High | Maintained during refresh cycles |

### ODT Assertion Timing

- ODT is asserted `tAON` cycles before write data
- ODT is de-asserted `tAOF` cycles after write data
- For this design: ODT follows write enable (`ts_i_o`) with appropriate timing

---

## DQS Timing and Control

### DQS Preamble and Postamble

- **Preamble**: DQS goes low 1/2 cycle before first data edge
- **Postamble**: DQS goes high 1/2 cycle after last data edge
- **Toggle pattern**: DQS toggles for exactly 8 cycles (4 full cycles) during a burst

### DQS Generation (Writes)

- DQS starts at low, toggles 8 times, ends at high
- DQS edges align with data transitions
- DQS is generated internally and driven through PHY

### DQS Capture (Reads)

- DQS is received from DDR2 device
- Ring buffer uses DQS edges to capture data
- DQS delay is compensated using delay cells (`CLKBUF2`)

---

## Bank State Management

### Bank States

Each bank can be in one of four states:

1. **IDLE**: Bank is precharged, no row open
2. **ACTIVE**: Row is open, ready for READ/WRITE
3. **READING**: Read operation in progress
4. **WRITING**: Write operation in progress

### Bank State Tracking

The protocol engine maintains bank state implicitly through:
- Row address registers per bank
- Timing counters for tRAS, tRP, tRCD
- Auto-precharge bit (`A[10]`) to control row closure

### Row Conflict Detection

- If a command targets a bank with a different row open, the controller must:
  1. Issue PRECHARGE (or wait for auto-precharge)
  2. Wait tRP
  3. Issue new ACTIVATE
  4. Wait tRCD
  5. Issue READ/WRITE

- For block operations: Controller attempts to reuse open rows when possible

---

## FIFO Implementation Details

### FIFO Structure

All FIFOs use synchronous, single-clock design:

- **Clock domain**: `CLK` (500 MHz)
- **Read pointer**: Gray-coded for safe operation
- **Write pointer**: Gray-coded for safe operation
- **Full/Empty detection**: Based on pointer comparison
- **Fill count**: Binary counter tracking occupancy

### FIFO Signals

| Signal | Description |
|--------|-------------|
| `data_in` | Input data bus |
| `put` | Write enable (push) |
| `get` | Read enable (pop) |
| `data_out` | Output data bus |
| `fillcount` | Current occupancy (0 to DEPTH-1) |
| `full` | FIFO is full |
| `empty` | FIFO is empty |
| `full_bar` | FIFO is not full (inverted `full`) |
| `empty_bar` | FIFO is not empty (inverted `empty`) |

### FIFO Flow Control

- **Producer side**: Checks `full_bar` before asserting `put`
- **Consumer side**: Checks `empty_bar` before asserting `get`
- **Simultaneous put/get**: Allowed when FIFO is neither full nor empty

---

## Clock Generation

### DDR Clock (`ck`) Generation

The DDR clock is generated by dividing the system clock:

```verilog
always @(posedge CLK) begin
    if (RESET)
        ck <= 1'b0;
    else
        ck <= ~ck;  // Toggle every CLK cycle
end
```

This creates a 250 MHz clock (half of 500 MHz) with 50% duty cycle.

### Clock Alignment

- `ck` is used for DDR2 command/address timing
- `ck` edges align with DDR2 CK/CK# pad outputs
- Internal logic uses `CLK` for all state machines and FIFOs

---

## Reset Sequencing

### Reset Behavior

1. **Reset assertion** (`RESET = 1`):
   - All FIFOs cleared (pointers reset, fillcount = 0)
   - Initialization engine returns to `S0` (IDLE)
   - Protocol engine returns to `IDLE`
   - Ring buffer cleared
   - `READY` de-asserted
   - `ck` reset to 0

2. **Reset de-assertion** (`RESET = 0`):
   - All modules remain in reset state
   - Host must assert `INITDDR` to start initialization
   - No automatic initialization on reset release

3. **Initialization start**:
   - Host asserts `INITDDR` (one-cycle pulse)
   - Initialization engine begins power-up sequence
   - `READY` remains low until initialization completes

---

## Internal Signal Connections

### Top-Level Module Connections

```
ddr2_controller
├── FIFO_CMD
│   ├── data_in: {ADDR, CMD, SZ, OP}
│   ├── put: CMD_put (from host interface logic)
│   ├── get: get_cmdFIFO_o (from protocol engine)
│   └── data_out: {addr_cmdFIFO_i, cmd_cmdFIFO_i, sz_cmdFIFO_i, op_cmdFIFO_i}
│
├── FIFO_DATA
│   ├── data_in: DIN
│   ├── put: put_dataFIFO (from host interface logic)
│   ├── get: get_dataFIFO_o (from protocol engine)
│   └── data_out: din_dataFIFO_i
│
├── FIFO_RETURN
│   ├── data_in: {addr_returnFIFO_o, ringBuff_returnFIFO}
│   ├── put: put_returnFIFO_o (from protocol engine)
│   ├── get: FETCHING (from host)
│   └── data_out: {RADDR, DOUT}
│
├── ddr2_init_engine
│   ├── clk: CLK
│   ├── reset: RESET
│   ├── init: INITDDR
│   ├── ready: READY
│   └── DDR2 control outputs (muxed with protocol engine)
│
├── ddr2_protocol_engine
│   ├── clk: CLK
│   ├── CK: ck (divided clock)
│   ├── reset: RESET
│   ├── ready_init_i: READY
│   ├── FIFO inputs/outputs (as above)
│   └── DDR2 control outputs (muxed with init engine)
│
├── ddr2_ring_buffer8
│   ├── listen: listen_ringBuff_o (from protocol engine)
│   ├── strobe: dqs_o[0] (from PHY)
│   ├── din: dq_o (from PHY)
│   ├── readPtr: readPtr_ringBuff_o (from protocol engine)
│   └── dout: ringBuff_returnFIFO (to return FIFO)
│
└── ddr2_phy (SSTL18DDR2INTERFACE)
    ├── Control inputs (muxed between init and protocol engines)
    ├── DQ/DQS bidirectional (connected to ring buffer and protocol engine)
    └── Pad outputs (to DDR2 device)
```

### Control Signal Multiplexing

When `READY = 0`:
- All command/address/control signals driven by `ddr2_init_engine`
- Protocol engine outputs ignored

When `READY = 1`:
- Command/address signals driven by `ddr2_protocol_engine`
- CKE and ODT remain controlled by `ddr2_init_engine`
- DQ/DQS direction controlled by `ts_i_o` and `ri_i_o` from protocol engine

---

## Parameter Definitions

### Global Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `BL` | `3'b011` | Burst length = 8 |
| `BT` | `1'b0` | Burst type = sequential |
| `CL` | `3'b100` | CAS latency = 4 |
| `AL` | `3'b100` | Additive latency = 4 |
| `CWL` | `3'b011` | CAS write latency = 3 |
| `DQ_WIDTH` | 16 | Data bus width |
| `DQS_WIDTH` | 2 | DQS strobe width |
| `DM_WIDTH` | 2 | Data mask width |
| `ROW_ADDR_WIDTH` | 13 | Row address width |
| `COL_ADDR_WIDTH` | 10 | Column address width |
| `BANK_ADDR_WIDTH` | 2 | Bank address width |
| `ADDR_WIDTH` | 25 | Total address width |
| `FIFO_DEPTH` | 64 | FIFO depth (2^6) |
| `FIFO_DEPTH_LOG2` | 6 | FIFO depth log2 |

### Timing Parameters (in CLK cycles)

| Parameter | Value | Description |
|-----------|-------|-------------|
| `tRCD_CYCLES` | 2 | Row-to-column delay |
| `tRP_CYCLES` | 2 | Row precharge time |
| `tRAS_CYCLES` | 10 | Row active time |
| `tRFC_CYCLES` | 50 | Refresh cycle time |
| `tMRD_CYCLES` | 2 | Mode register set delay |
| `tWR_CYCLES` | 4 | Write recovery time |
| `tWTR_CYCLES` | 2 | Write-to-read turnaround |
| `tRTW_CYCLES` | 4 | Read-to-write turnaround |
| `tFAW_CYCLES` | 20 | Four activate window |
| `tXSR_CYCLES` | 50 | Exit self-refresh |
| `tDLLK_CYCLES` | 200 | DLL lock time |
| `tINIT_CYCLES` | 100000 | Power-up wait (200 µs) |

---

## Error Handling and Edge Cases

### FIFO Overflow Protection

- **Command FIFO**: `NOTFULL` signal prevents host from overfilling
- **Data FIFO**: `FILLCOUNT` monitored, threshold at 33 entries for block writes
- **Return FIFO**: `fullBar_returnFIFO_i` prevents protocol engine from pushing when full

### Invalid Commands

- Commands with `CMD = 3'b000` or `3'b111`: Treated as NOP, ignored
- Commands with `CMD = 3'b101` or `3'b110`: Reserved, treated as NOP (not implemented)
- Commands issued before `READY`: Blocked by protocol engine (waits in `IDLE`)

### Timing Violations

- All timing parameters are enforced by state machine counters
- Refresh preempts normal operations to prevent tREF violations
- Block operations check return FIFO space before starting

### Reset During Operation

- If `RESET` asserted during operation:
  - All state machines return to initial state
  - FIFOs cleared
  - `READY` de-asserted
  - Host must re-initialize after reset release

---

## Performance Metrics

### Throughput

- **Read bandwidth**: Up to 250 MHz × 16 bits = 4 Gbps = 500 MB/s (theoretical)
- **Write bandwidth**: Up to 250 MHz × 16 bits = 4 Gbps = 500 MB/s (theoretical)
- **Actual throughput**: Reduced by refresh overhead, command overhead, and FIFO back-pressure

### Latency

- **Read latency**: AL + CL = 4 + 4 = 8 cycles (32 ns at 250 MHz DDR clock)
- **Write latency**: AL + CWL = 4 + 3 = 7 cycles (28 ns at 250 MHz DDR clock)
- **Command latency**: 2 cycles (FIFO read) + tRCD + operation latency

### Refresh Overhead

- Refresh occurs every 3800 cycles (7.6 µs)
- Refresh sequence: ~55 cycles (220 ns)
- Refresh overhead: ~1.4% of total time

---

## State Machines

### Initialization Engine State Machine

States: `S0` (IDLE) → `S1` (NOP1) → `S2` (CKE) → `S3` (PRE1) → `S4` (NOP2) → `S5-S27` (MRS/EMRS sequences) → `S_DONE`

Key transitions:
- `S0`: Wait for `init` pulse
- `S1`: 200 µs power-up wait
- `S2`: Assert CKE, wait tXSR
- `S3`: PRECHARGE ALL
- `S4-S27`: Mode register programming, DLL reset, auto refresh
- `S_DONE`: Assert `ready`, continue NOPs

### Protocol Engine State Machine

**Main States**:
- `IDLE`: Wait for commands or refresh trigger
- `GETCMD`: Fetch command from FIFO
- `ACTIVATE`: Issue ACTIVATE command
- `SCREAD`: Scalar read sequence
- `SCWRITE`: Scalar write sequence
- `BLR`: Block read sequence
- `BLW`: Block write sequence
- `RPRE`, `RNOP`, `RREF`, `RIDL`: Refresh sequence

**Sub-states for reads**:
- `SCREAD`: Wait for read latency
- `SCRLSN`: Pulse ring buffer listen
- `SCRNLSN`: Step ring buffer pointer
- `SCRDNTH`: Wait for data capture
- `SCREND`: Push to return FIFO

**Sub-states for writes**:
- `SCWRITE`: Wait for write latency
- `SCWRDATA`: Drive DQ/DQS/DM
- `SCWREND`: Bus turnaround

---

## Design Assumptions and Constraints

- **Clocking**:
  - Single 500 MHz `CLK`, internal divide-by-2 to 250 MHz DDR clock; no explicit PLL/phase adjustment modeled
  - All logic is synchronous to `CLK`; DQS alignment relies on RTL delay cells and the DDR2 model

- **Protocol features**:
  - Implements only standard DDR2 features required for basic operation:
    - Power-up and MRS/EMRS configuration
    - Periodic auto-refresh (single rank)
    - No explicit power-down, self-refresh, DLL off, or deep power-saving modes

- **Interface usage**:
  - Host is expected to:
    - Observe `NOTFULL` before issuing more commands/writes
    - Keep `FETCHING` high when ready to consume read data; otherwise, return FIFO will fill

- **Width / geometry**:
  - Fixed x16 data width, 2 bank bits, 13 row bits, and the specific geometry of `mt47h32m16_37e`
  - Single rank, single DDR2 device

- **Others**:
  - No request reordering, QoS, or complex arbitration: commands are executed in FIFO order except where refresh preempts in `IDLE`
  - Controller is single-port on the front-end; no multi-host or multi-channel support

---

## Signal Naming Conventions

### Naming Rules

- **Module prefixes**: 
  - `ddr2_` prefix for all DDR2-related modules
  - `FIFO_` prefix for FIFO instances
- **Signal suffixes**:
  - `_i` for inputs to a module
  - `_o` for outputs from a module
  - `_pad` for pad-level signals
  - `_reg` for registered signals
  - `_bar` for active-low signals (e.g., `csbar`, `rasbar`)
- **Clock signals**:
  - `CLK` for system clock (500 MHz)
  - `ck` for DDR clock (250 MHz)
  - `clk` for generic clock inputs to modules
- **Reset signals**:
  - `RESET` for top-level reset
  - `reset` for module-level reset inputs

### Common Signal Patterns

- **FIFO signals**: `{FIFO_NAME}_{operation}` (e.g., `CMD_put`, `DATA_get`)
- **State machine states**: Uppercase with underscores (e.g., `IDLE`, `SCREAD`)
- **Counters**: `{name}_reg` or `{name}_cnt` (e.g., `cntr_reg`, `refCnt_reg`)

---

## DQS Calibration and Delay Management

### Delay Line Implementation

The ring buffer uses delay cells (`CLKBUF2`) to compensate for DQS timing:

- **Purpose**: Align DQS strobe with data capture window
- **Implementation**: Chain of delay cells creating `dStrobe`
- **Gating**: `fStrobe = dStrobe & (listen | F0)` ensures capture only during valid window

### Calibration Strategy

- **Fixed delay**: Delay line provides fixed delay compensation
- **No runtime calibration**: Design assumes fixed delay is sufficient
- **Model-based**: Relies on DDR2 model and RTL delay cells for timing

### DQS Edge Capture

- **Rising edge**: Captures even samples (r0, r2, r4, r6)
- **Falling edge**: Captures odd samples (r1, r3, r5, r7)
- **Total samples**: 8 words per burst (4 DQS cycles)

---

## Power Management (Not Implemented)

The following power management features are **not implemented** in this design:

- **Power-down modes**: Entry/exit sequences not implemented
- **Self-refresh**: Self-refresh entry/exit not implemented
- **DLL disable**: DLL always enabled during operation
- **Clock gating**: No clock gating implemented

These features would require additional state machines and timing management.

---

## Debug and Monitoring

### Available Debug Signals

The design exposes the following signals for debugging:

- **`READY`**: Initialization complete indicator
- **`VALIDOUT`**: Read data valid indicator
- **`NOTFULL`**: Flow control indicator
- **`FILLCOUNT`**: Data FIFO occupancy
- **FIFO status**: `empty`, `full`, `empty_bar`, `full_bar` for each FIFO

### Internal State Visibility

For debugging, the following internal states can be monitored:

- **Initialization state**: `ddr2_init_engine.state_reg` (5-bit)
- **Protocol state**: `ddr2_protocol_engine.state` (4-bit)
- **Block state**: `ddr2_protocol_engine.block_state` (2-bit)
- **Refresh counter**: `ddr2_protocol_engine.refCnt_reg` (12-bit)
- **Timing counters**: `ddr2_protocol_engine.cnt_reg` (6-bit)

---

## Verification Strategy

### Test Scenarios

1. **Initialization**:
   - Power-up sequence correctness
   - Mode register programming
   - DLL reset and re-programming
   - `READY` signal assertion timing

2. **Scalar Operations**:
   - Scalar read (SCR) to various addresses
   - Scalar write (SCW) to various addresses
   - Read-after-write verification

3. **Block Operations**:
   - Block read (BLR) with all `SZ` values (1-4 bursts)
   - Block write (BLW) with all `SZ` values
   - Address increment verification

4. **Refresh**:
   - Periodic refresh insertion
   - Refresh during active traffic
   - Refresh timing compliance

5. **FIFO Management**:
   - FIFO overflow/underflow protection
   - Flow control (`NOTFULL`, `VALIDOUT`)
   - Back-pressure handling

6. **Timing Compliance**:
   - All JEDEC timing parameters (tRCD, tRP, tRFC, tRAS, etc.)
   - DQS/DQ alignment
   - Clock domain crossing

### Simulation Environment

- **DDR2 model**: Micron `mt47h32m16_37e.v` behavioral model
- **Testbench**: Generates test patterns, monitors responses, validates correctness
- **Coverage**: State machine coverage, timing path coverage, FIFO occupancy coverage

### Test Patterns

Recommended test patterns for verification:

1. **Initialization Test**:
   - Power-up sequence
   - Mode register programming verification
   - `READY` signal timing

2. **Basic Read/Write**:
   - Write pattern: `0xAAAA`, `0x5555`, `0x0000`, `0xFFFF`
   - Read-back verification
   - Address increment patterns

3. **Block Operations**:
   - Block write with all `SZ` values (1-4 bursts)
   - Block read with all `SZ` values
   - Address boundary crossing

4. **Refresh Interleaving**:
   - Continuous read/write operations
   - Refresh insertion verification
   - No data corruption during refresh

5. **FIFO Stress**:
   - FIFO full conditions
   - FIFO empty conditions
   - Back-to-back operations

6. **Timing Compliance**:
   - Minimum timing verification
   - Maximum timing verification
   - Edge case timing scenarios

7. **Address Patterns**:
   - Same bank, different rows
   - Different banks, same row
   - Random address patterns
   - Sequential address patterns

---

## Summary

This DDR2 controller architecture provides:

1. **Complete JEDEC Compliance**: Full initialization sequence with proper timing
2. **Robust Protocol Engine**: Handles scalar and block operations with proper timing management
3. **Efficient Data Path**: Ring buffer for read data capture, FIFOs for command/data buffering
4. **Refresh Management**: Automatic periodic refresh with proper scheduling
5. **Clean Interface**: Simple FIFO-based host interface with flow control
6. **Maintainable Design**: Clear separation of concerns, well-documented state machines

The architecture is optimized for:
- **Correctness**: JEDEC-compliant timing and protocol
- **Clarity**: Well-structured modules with clear interfaces
- **Maintainability**: Parameterized design with comprehensive documentation
- **Verifiability**: Clear state machines and timing specifications

This specification serves as the foundation for Verilog module implementation, ensuring consistency and correctness across all design components.
