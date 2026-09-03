// ═══════════════════════════════════════════════════════════════
//  stereo.h — dual-RX phase-based AoA math.
//
//  This module does one job: given two receivers looking at the
//  same broadcast frame from the same beacon, take the phase
//  disparity across subcarriers, fit a line, subtract the
//  empty-room baseline, remove common LO drift via median across
//  the beacon set, and turn what's left into an angle-of-arrival
//  estimate per beacon.
//
//  All heavy lifting is on the PRIMARY.  SECONDARY only extracts
//  its own line-fit (slope, intercept) per frame and sends it to
//  PRIMARY via peer_send_csi_summary.
// ═══════════════════════════════════════════════════════════════
#pragma once

#include "config.h"

// Fit a straight line y = slope*x + intercept over the selected subcarriers.
// Uses phase values already unwrapped across the selected set.  Returns the
// number of points actually used (in case some were nan / invalid).
int stereo_fit_line(const float *phase64,
                    float &slope, float &intercept);

// Snapshot current line-fit into this beacon's baseline (called at end of
// baseline capture on the PRIMARY; SECONDARY snapshots its own baseline
// locally in parallel).
void stereo_snapshot_baseline(BeaconState &b);

// Compute and buffer a stereo entry for this beacon at the current frame.
// Called from csi_process_frames whenever we successfully computed a
// per-frame line fit for this beacon.  Records (counter, slope, intercept).
// Called on both PRIMARY and SECONDARY.
void stereo_record_local(BeaconState &b, uint32_t counter, uint32_t stamp_ms);

// SECONDARY: for each new local record we just captured, send a summary
// to primary.  Bundles the last un-sent slope/intercept.  Cheap — no-op
// if role != SECONDARY.
void stereo_flush_summaries_to_peer();

// PRIMARY: called from peer_handle_csi_summary.  Attempts to pair the
// incoming peer summary with a matching local record within
// STEREO_PAIR_WINDOW_MS and, if paired, records residuals for the current
// LO-drift solve.
void stereo_ingest_peer_summary(const PeerCsiSummary &pkt);

// PRIMARY: stash a per-beacon baseline received from SECONDARY.  If the
// local baseline is already valid for this beacon, fold immediately into
// the disparity baseline.  Called from peer_handle_baseline.
void stereo_ingest_peer_baseline(const PeerBaselinePacket &pkt);

// Called at the end of csi_baseline_finalize on both units.  On SECONDARY,
// transmits our per-beacon baseline to primary.  On PRIMARY, folds any
// already-stashed peer baseline into the disparity baseline.
void stereo_on_baseline_finalized();

// PRIMARY: called from the main loop after peer summaries have been
// ingested for this cycle.  Solves the common LO drift (median across
// beacons of Δintercept), removes it, and updates per-beacon aoa_rad /
// aoa_conf.
void stereo_update_aoa();

// Convert phase residuals to angle-of-arrival in radians.  Positive =
// right of array normal.  Assumes baseline STEREO_BASELINE_CM.
float stereo_intercept_to_aoa(float residual_intercept);
