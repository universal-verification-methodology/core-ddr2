//------------------------------------------------------------------------------
// DDR2 controller timing configuration (simulation profile)
//
// This header centralizes coarse timing parameters used by the verification
// monitors so they can be adjusted per-speed-grade in a single place.
//
// Values here are expressed in controller clock cycles.
//
// To change the profile, edit these `define values or provide an alternate
// version of this file on the include path.
//------------------------------------------------------------------------------

// Coarse JEDEC-like bank timing (used by ddr2_timing_checker).
`define DDR2_TIMING_TRCD_MIN      4    // ACT -> RD/WR
`define DDR2_TIMING_TRP_MIN       4    // PRE -> ACT
`define DDR2_TIMING_TRAS_MIN      8    // ACT -> PRE
`define DDR2_TIMING_TRFC_MIN      16   // REF -> REF (coarse)

// Refresh interval bounds (used by ddr2_refresh_monitor).
//
// In the current SIM_SHORT_INIT configuration the controller intentionally
// compresses tREFI, so we:
//   - disable the "too frequent" check (MIN = 0)
//   - keep a generous upper bound so that a stuck refresh scheduler is still
//     detected (MAX = 100k cycles).
`define DDR2_TIMING_TREFI_MIN_CLK 0
`define DDR2_TIMING_TREFI_MAX_CLK 100000

