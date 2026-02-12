//------------------------------------------------------------------------------
// DDR2 controller timing configuration (simulation profile / strict profile)
//
// This header centralizes coarse timing parameters used by the verification
// monitors so they can be adjusted per-speed-grade in a single place.
//
// Values here are expressed in controller clock cycles.
//
// To change the profile, edit these `define values or provide an alternate
// version of this file on the include path.
//------------------------------------------------------------------------------
// Strict vs. relaxed profiles
//------------------------------------------------------------------------------
// When `STRICT_JEDEC is defined, the monitors enforce JEDEC-like timing
// numbers more tightly (especially for refresh interval). The default profile
// remains simulation-friendly so short-init / shortened-refresh configurations
// continue to work unless STRICT_JEDEC is explicitly enabled on the
// compile/simulation command-line (e.g. +define+STRICT_JEDEC).
//------------------------------------------------------------------------------

// Coarse JEDEC-like bank timing (used by ddr2_timing_checker).
// These values are already expressed in controller clock cycles and are used
// in both relaxed and strict profiles.
`define DDR2_TIMING_TRCD_MIN      4    // ACT -> RD/WR
`define DDR2_TIMING_TRP_MIN       4    // PRE -> ACT
`define DDR2_TIMING_TRAS_MIN      20   // ACT -> PRE (JEDEC: 10 DDR2 cycles = 20 controller cycles = 40 ns)
`define DDR2_TIMING_TRFC_MIN      100  // REF -> REF (JEDEC: 50 DDR2 cycles = 100 controller cycles = 200 ns)

// Turnaround and write/read-to-precharge timings (used by ddr2_turnaround_checker).
// Expressed in controller clock cycles. JEDEC base values (DDR2 cycles) are:
//   - tWTR = 2 DDR2 cycles  = 4 controller cycles
//   - tRTW = 4 DDR2 cycles  = 8 controller cycles
//   - tWR  = 4 DDR2 cycles  = 8 controller cycles
//   - tRTP = 2 DDR2 cycles  = 4 controller cycles
`define DDR2_TIMING_TWTR_MIN      4    // WRITE -> READ (any bank), JEDEC: 2 DDR2 cycles = 4 controller cycles
`define DDR2_TIMING_TRTW_MIN      8    // READ  -> WRITE (any bank), JEDEC: 4 DDR2 cycles = 8 controller cycles
`define DDR2_TIMING_TWR_MIN       8    // WRITE -> PRE   (same bank), JEDEC: 4 DDR2 cycles = 8 controller cycles
`define DDR2_TIMING_TRTP_MIN      4    // READ  -> PRE   (same bank), JEDEC: 2 DDR2 cycles = 4 controller cycles

// Refresh interval bounds (used by ddr2_refresh_monitor).
//
// In the relaxed simulation profile (default) the controller intentionally
// compresses tREFI, so we:
//   - disable the "too frequent" check (MIN = 0)
//   - keep a generous upper bound so that a stuck refresh scheduler is still
//     detected (MAX = 100k cycles).
//
// When STRICT_JEDEC is defined, we instead enforce a tight window around the
// JEDEC tREFI worst-case requirement (~7.8us @ 500 MHz ≈ 3900 CLK cycles).
// MAX is set to 3900 so that any refresh spacing longer than one nominal
// tREFI interval is treated as a specification violation. The controller's
// internal REF_CNT_INIT/REF_THRESH policy is tuned so that even under heavy
// traffic the measured interval stays within this bound.
`ifdef STRICT_JEDEC
`define DDR2_TIMING_TREFI_MIN_CLK 0
`define DDR2_TIMING_TREFI_MAX_CLK 3900
`else
`define DDR2_TIMING_TREFI_MIN_CLK 0
`define DDR2_TIMING_TREFI_MAX_CLK 100000
`endif

