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
`define DDR2_TIMING_TRAS_MIN      8    // ACT -> PRE
`define DDR2_TIMING_TRFC_MIN      16   // REF -> REF (coarse window between REF commands)

// Turnaround and write/read-to-precharge timings (used by ddr2_turnaround_checker).
// Expressed in controller clock cycles.
`define DDR2_TIMING_TWTR_MIN      2    // WRITE -> READ (any bank)
`define DDR2_TIMING_TRTW_MIN      4    // READ  -> WRITE (any bank)
`define DDR2_TIMING_TWR_MIN       4    // WRITE -> PRE   (same bank)
`define DDR2_TIMING_TRTP_MIN      2    // READ  -> PRE   (same bank)

// Refresh interval bounds (used by ddr2_refresh_monitor).
//
// In the relaxed simulation profile (default) the controller intentionally
// compresses tREFI, so we:
//   - disable the "too frequent" check (MIN = 0)
//   - keep a generous upper bound so that a stuck refresh scheduler is still
//     detected (MAX = 100k cycles).
//
// When STRICT_JEDEC is defined, we instead enforce a much tighter window
// around the controller's nominal refresh policy (~3.8k–3.9k CLK cycles),
// corresponding to a tREFI of roughly 7.6–7.8us at 500 MHz.
`ifdef STRICT_JEDEC
`define DDR2_TIMING_TREFI_MIN_CLK 3600
`define DDR2_TIMING_TREFI_MAX_CLK 4200
`else
`define DDR2_TIMING_TREFI_MIN_CLK 0
`define DDR2_TIMING_TREFI_MAX_CLK 100000
`endif

