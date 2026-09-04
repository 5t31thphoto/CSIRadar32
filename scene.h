// ═══════════════════════════════════════════════════════════════
//  scene.h — MantisSec spatial reconstruction engine (v0.3 core)
//
//  Mental model:
//    - Beacons = radio illuminators at approx-known positions.
//    - RXs     = radio-domain "cameras" (broad, low-res, per-source
//                signature per instant).
//    - People  = occluders that perturb each beacon→RX link.
//    - Empty-room baseline = the "background photograph" of the
//                            static scene (walls, furniture).
//    - Walk cal = samples the room's response Green's function by
//                 physically placing a known occluder (the user, with
//                 PROBE) at scripted landmarks.
//    - Runtime = sparse inverse solve: given the current perturbation
//                pattern, find the occupancy field over the room whose
//                combined kernel-response best explains it.
//
//  Interfaces:
//    - Cal capture: scene_begin_landmark_capture / scene_end_landmark_capture
//                   scene_begin_transit_capture  / scene_end_transit_capture
//                   scene_begin_rotate_capture   / scene_end_rotate_capture
//                   scene_begin_empty_room       / scene_end_empty_room
//    - Feeding data (both cal AND runtime): scene_observe(FrameObs)
//    - Cal finalization: scene_finalize_cal() → produces model artifact
//    - Runtime: scene_update() runs the reconstruction (rate-limited)
//    - Readout: scene_get_field(), scene_get_tracks()
//
//  All positions are NORMALIZED (geometry-frame, roughly [-1, +1]).
// ═══════════════════════════════════════════════════════════════
#pragma once
#include "config.h"

// ── Kernel sample (one measured position → response tuple) ────
struct KernelSample {
    float pos[2];                     // normalized geometry coords
    struct PerBeacon {
        // Central tendency
        float mean_amp;               // averaged amp perturbation
        float mean_phase;             // circular-mean phase perturbation
        float mean_aoa_dev;           // circular-mean AoA (stereo)

        // Dispersion (all reach inference via IDW to per-cell sigmas)
        float std_amp;                // noise estimate at this landmark, this beacon
        float std_phase;              // circular std ≈ sqrt(-2·ln R_phase)
        float std_aoa;                // circular std of AoA

        // Aspect (populated at ROTATE landmarks only; = std_amp elsewhere)
        // Variance of amp across a full 360° rotation of the person at
        // this landmark.  Wide aspect_var means this landmark's response
        // depends strongly on body orientation → cells near it should
        // trust amplitude less.  IDW-interpolated at inference into a
        // per-cell "aspect uncertainty" that widens the amp sigma.
        float aspect_var;

        // Rotation Fourier basis (populated at ROTATE landmarks only).
        // h_amp(landmark, θ, b) ≈ a0 + a1·cos(θ) + b1·sin(θ)
        //                            + a2·cos(2θ) + b2·sin(2θ)
        // Enables v0.6 to marginalize over latent orientation at inference.
        // v0.5 uses aspect_var (which is essentially the sum of squared
        // non-DC coefficients) but stores the coefficients for v0.6.
        float fourier[5];             // [a0, a1, b1, a2, b2]

        // Local Jacobian from adjacent transit slices (populated at
        // landmarks that had an incoming or outgoing transit; zero'd
        // elsewhere).  Turns kernel_predict from IDW-of-means into a
        // first-order Taylor expansion at IDW-of-anchors, which is a
        // genuine step toward continuous local modeling.
        float grad_amp[2];            // ∂mean_amp/∂x, ∂mean_amp/∂y
        uint8_t grad_valid;           // 0 or 1

        // Provenance / weighting
        uint16_t sample_count;        // enters IDW weights via sqrt(count)
        uint8_t  saw_aoa;             // count of frames with valid AoA
    } b[MAX_BEACONS];
    uint8_t  n_beacons;
    uint8_t  landmark_id;             // LM_* if this is a stand-point, 0xFF for transit
    uint8_t  transit_from;            // for transit samples
    uint8_t  transit_to;
    float    transit_frac;            // 0..1 arc-length position along transit
};

// ── Occupancy field (2D grid over normalized geometry) ─────────
struct OccupancyField {
    float cell[FIELD_DIM][FIELD_DIM]; // occupancy weight ∈ [0, 1]
    float cell_max;                   // for normalized rendering
    uint32_t last_update_ms;
    bool valid;
};

// ── Target track (temporally linked detection) ─────────────────
struct TargetTrack {
    bool     active;
    uint8_t  id;                      // stable ID (for trail color)
    float    pos[2];                  // normalized coords
    float    vel[2];
    float    cov_xx, cov_yy, cov_xy;  // EKF covariance (real, not heuristic)
    float    confidence;              // 0..1  (aliasing reduces this)
    uint16_t age_frames;
    uint16_t missed_frames;
    // Set when this track's current position is close to an alias-pair
    // landmark from the cal report.  Renderer draws ambiguity indicator;
    // scene widens the heuristic covariance floor while set.
    uint8_t  ambiguity_flag;
    // Trail history (ring buffer)
    struct TrailPoint {
        float    pos[2];
        float    conf;
        uint32_t t_ms;
    } trail[TRACK_TRAIL_LEN];
    uint8_t  trail_head;
    uint16_t trail_count;
};

// ── Alias pair (physically far, RF-space close) ───────────────
// Stored explicitly so runtime can flag ambiguity when a pick lands
// near either endpoint.
#define CAL_MAX_ALIAS_PAIRS 16
struct AliasPair {
    uint8_t lm_a, lm_b;               // landmark IDs
    float   rf_distance;              // normalized RF-space distance
    float   phys_distance;            // physical distance (normalized units)
};

// ── Cal quality report (shown at end of cal) ───────────────────
struct CalReport {
    bool     valid;
    CalMode  mode;
    uint16_t landmarks_captured;      // out of LM_COUNT
    uint16_t transit_samples;
    uint16_t rotate_samples;
    uint16_t total_kernel_samples;
    float    cross_val_error;         // leave-one-out landmark prediction error
    float    loop_closure_error;      // overall (kept for backwards compat)
    float    per_beacon_loop_closure[MAX_BEACONS];  // NEW: per-beacon drift
    float    geometry_correction_mag; // how much we adjusted beacon positions
    float    per_beacon_snr[MAX_BEACONS];
    float    per_beacon_orient_reliability[MAX_BEACONS];  // from rotation experiment (diagnostic)
    uint8_t  worst_landmark;          // LM_* with highest cross-val error
    float    worst_landmark_error;
    // Fisher-lite observability: mean of the smaller eigenvalue of the
    // local response Jacobian across all kernel landmark samples.
    float    mean_observability;
    // Alias pairs — stored, not just counted.  Runtime uses these to
    // flag track ambiguity + reduce confidence in ambiguous regions.
    uint8_t   alias_pair_count;
    AliasPair alias_pairs[CAL_MAX_ALIAS_PAIRS];
    // Geometry validation: PROBE at "beacon N landmark" should observe
    // beacon N as its strongest.  Non-zero = beacon likely mispositioned
    // or misidentified.  Bit b set means beacon b failed the check.
    uint8_t   geometry_validation_fail_mask;
};

// ═══════════════════════════════════════════════════════════════
//  LIFECYCLE
// ═══════════════════════════════════════════════════════════════
void scene_begin();                           // module init
void scene_reset();                           // wipe kernel + field + tracks

// Compute normalized landmark positions from current beacon geometry.
// Must be called AFTER beacons have been discovered and had their
// pos_x/pos_y assigned (from geometry guide).
void scene_derive_landmarks_from_geometry();

// Look up a landmark's normalized position (0..1 unit-ish frame).
void scene_landmark_pos(LandmarkId id, float *out_x, float *out_y);

// ═══════════════════════════════════════════════════════════════
//  CALIBRATION CAPTURE
// ═══════════════════════════════════════════════════════════════
void scene_cal_begin(CalMode mode);           // reset kernel, start cal
void scene_cal_abort();

// Empty-room baseline is captured separately by csi.cpp (existing v0.2
// mechanism).  scene_cal_ack_empty_room() marks that phase complete.
void scene_cal_ack_empty_room();

// Landmark (STAND) capture — averaged fingerprint at a known point.
void scene_begin_landmark_capture(LandmarkId lm);
void scene_end_landmark_capture();

// Transit (WALK) capture — continuous stream, resampled at end into
// N samples arc-length parametrized along the landmark_a → landmark_b line.
void scene_begin_transit_capture(LandmarkId from, LandmarkId to);
void scene_end_transit_capture();

// Rotate capture — user stands still at a landmark and rotates 360°.
// We record how each beacon's response modulates purely with orientation
// (no position change).  Feeds orientation-invariance weights.
void scene_begin_rotate_capture(LandmarkId at);
void scene_end_rotate_capture();

// Absorb any pending PROBE observations from the peer link (ANCHOR only,
// stereo mode).  Called in the main loop while cal is active.
void scene_cal_absorb_peer_stream();

// Called by the wizard on step entry so PROBE-side outgoing packets
// can carry real step/landmark provenance (v0.3 first-cut sent
// placeholders).  Both PROBE and ANCHOR track this locally.
void scene_cal_note_step(uint8_t step_idx, uint8_t landmark_id);

// Called on PROBE (stereo) at every frame to transmit its own view to
// ANCHOR.  Cheap; no-op if not PROBE.
void scene_cal_transmit_probe_frame(const FrameObservation &obs);

// Run the model finalization pass (compute kernel stats, normalize
// features, cross-validate, geometry refinement).  Called once after
// the walk sequence completes.  Populates `report` with quality metrics.
void scene_finalize_cal(CalReport &report);

// v0.4: tripwire (1-beacon) shortcut finalize.  No walk cal is done —
// beacon 1 pose is fixed by geometry alone, and the "kernel" is
// stand-in / empty.  Marks the scene calibrated so runtime can begin.
// Detection in tripwire mode falls back to bulk-amplitude thresholding
// against the empty-room baseline (like v0.1 tripwire), which is what
// the mode has always meant.
void scene_cal_tripwire_finalize();

// ═══════════════════════════════════════════════════════════════
//  RUNTIME
// ═══════════════════════════════════════════════════════════════
// Feed one frame's observation vector into the scene.  Called from
// the main loop (both during cal capture AND during runtime).
void scene_observe(const FrameObservation &obs);

// Run reconstruction + tracker step.  Rate-limits internally to
// SCENE_UPDATE_MS.  Only meaningful post-cal.
void scene_update();

// Readouts for the render layer.
const OccupancyField *scene_get_field();
const TargetTrack    *scene_get_track(int idx);   // 0..TRACK_MAX-1
int                   scene_active_track_count();

// Diagnostic accessors
int                   scene_kernel_sample_count();
const KernelSample   *scene_kernel_sample(int idx);
float                 scene_novelty_score();      // last-frame residual energy

// Convenience: is cal complete?
bool scene_cal_complete();
