// ═══════════════════════════════════════════════════════════════
//  CSI-Radar-S3 — csi.h
//  CSI receiver + per-beacon feature pipeline + spatial estimator.
//
//  Design:
//    • Beacons discovered by ESP-NOW recv callback (MAC prefix match).
//    • CSI RX ISR routes each frame to the correct BeaconState by MAC,
//      copies amplitudes into that slot, sets its dirty flag.
//    • Main loop calls csi_process_frames() which drains dirty slots,
//      updates each beacon's feature vector, filter chain, and metrics.
//    • csi_update_spatial() fuses the per-beacon link_metric_ema values
//      into a 2-D or 1-D position estimate depending on RadarMode.
// ═══════════════════════════════════════════════════════════════
#pragma once

#include "config.h"

// Set up Wi-Fi + ESP-NOW + CSI RX in the exact config that matches beacons.
// Must be called after WiFi is off / clean.
void csi_engine_begin();
void csi_engine_end();

// Return count of *unique* beacons seen since the last reset,
// filtered by expected MAC prefix. Called during discovery.
int  csi_get_beacon_count();

// Called from main state machine to reset discovery.
void csi_reset_discovery();

// Set which of the tracked beacons are the "active" trio/pair/single used
// for the fused estimator. Uses first N slots by default.
void csi_choose_mode(RadarMode m);

// Assign physical positions to beacons based on selected geometry.
// For 3 beacons: equilateral triangle centered at (0,0) with side `side_cm`.
// For 2 beacons: horizontal line, ±side_cm/2.
// For 1 beacon: at (0, side_cm/2), T-Display at origin.
void csi_assign_default_geometry(float side_cm);

// ── v0.9: beacon control ─────────────────────────────────────
// The extended beacon firmware has always accepted these; nothing ever
// sent them.  target_id 0 addresses every beacon.
void csi_beacon_command(uint8_t target_id, uint8_t op,
                        uint16_t arg_u16 = 0, uint32_t arg_u32 = 0);
void csi_beacon_ping_all();                 // request PONGs (fw_marker, rate)
void csi_beacon_set_rate_all(uint16_t hz);  // set TX rate on every beacon
void csi_beacon_apply_run_config();         // ping + set rate + set sleep
// Periodically verify beacons are still on the requested rate and
// re-command any that are not.  Call from the main loop.
void csi_beacon_enforce_rate();
// PING-only upkeep, safe to run DURING calibration: carries no config so
// it cannot change a beacon's rate mid-capture.
void csi_beacon_keepalive();
// One-shot: arm beacon light-sleep AFTER cal is accepted.  Never during.
void csi_beacon_enable_sleep_after_cal();
// Non-blocking driver for beacon configuration. Call every loop.
void csi_beacon_service();
bool csi_beacon_try_consume_pong(const uint8_t *data, int len);

// Reset filter chain / calibration state for all beacons (retain baseline
// if `hard` is false; wipe everything if true).
void csi_reset_filters(bool hard);

// Drain dirty CSI slots and advance the per-beacon feature pipelines.
// Returns number of frames processed this call.
int  csi_process_frames();

// Feed a frame into the CALIBRATION accumulators (baseline or walk).
// Call between csi_process_frames() invocations.
void csi_baseline_accumulate();
void csi_baseline_finalize();     // computes threshold from P95, marks baseline valid
void csi_walk_accumulate();
void csi_walk_finalize();

// Progress helpers (0..1) for UI progress bars.
float csi_baseline_progress();
// Seconds the baseline will take at the beacons' CURRENT transmit rate.
// The sample count is fixed (statistical resolution is the invariant),
// so the duration is what moves when the rate changes.
int   csi_baseline_expected_seconds();    // min progress across active beacons
float csi_walk_progress();

// Spatial estimator — updates g_app.est_x/est_y/est_confidence.
void  csi_update_spatial();

// Query a bit of info for UI.
uint32_t csi_frames_seen();
