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
float csi_baseline_progress();    // min progress across active beacons
float csi_walk_progress();

// Spatial estimator — updates g_app.est_x/est_y/est_confidence.
void  csi_update_spatial();

// Query a bit of info for UI.
uint32_t csi_frames_seen();
