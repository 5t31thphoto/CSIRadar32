#pragma once
#include "config.h"
#include "rfcore.h"   // -I rust/rfcore from the build flags

enum TacticalCaptureKind : uint8_t { TCAP_NONE, TCAP_STAND, TCAP_WALK, TCAP_CIRCUIT, TCAP_CHECK, TCAP_BASELINE, TCAP_DEPLOY, TCAP_RETURN };
struct TacticalStatus {
  bool active, ready, baseline_ready, contaminated;
  float baseline_quality, motion, occupancy;
  uint8_t kind, track_count;
  float perimeter_s, perimeter_ds, bearing, bearing_conf;
  MantisRfTrack tracks[MANTIS_RF_MAX_TRACKS];
  uint8_t field[MANTIS_RF_FIELD*MANTIS_RF_FIELD];
};
void tactical_begin();
void tactical_reset_model();
void tactical_begin_capture(TacticalCaptureKind kind, LandmarkId a, LandmarkId b);
bool tactical_end_capture();
void tactical_observe(const FrameObservation &obs);
void tactical_runtime_observe(const FrameObservation &obs);
void tactical_begin_deploy(uint8_t beacon_index);
void tactical_begin_return();
void tactical_begin_check(float x, float y);
void tactical_begin_baseline();
void tactical_accept_baseline();
bool tactical_baseline_ready();
float tactical_baseline_quality();
bool tactical_baseline_contaminated();
bool tactical_ready();
// ── ADOPT AN EXISTING FULL-MODE CALIBRATION ───────────────────
// A completed Full Mode walk already contains exactly what the RF chart
// wants: labelled response samples at known positions, plus an
// empty-room reference.  Re-walking the room to collect the same
// information again would be busywork.
//
// Returns the number of nodes seeded, or 0 when there is no usable
// kernel (in which case the caller should run a deployment instead).
int tactical_adopt_full_cal();
bool tactical_can_adopt();

void tactical_finalize_model();
void tactical_update();
const TacticalStatus &tactical_status();
void tactical_render_radar();
void tactical_render_field();
// Dynamic field-check selector. Returns a normalized location and a short operator cue.
bool tactical_next_check(float *x, float *y, const char **cue, LandmarkId *lm);
void tactical_complete_check();
void tactical_cancel_check();
