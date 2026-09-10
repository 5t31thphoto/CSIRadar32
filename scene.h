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

        // Aspect (populated at ROTATE landmarks only; ZERO elsewhere).
        // The comment here used to claim "= std_amp elsewhere", which is
        // wrong: write_kernel_sample_from_range() memsets the sample and
        // only scene_end_rotate_capture() ever writes this field, so at
        // every STAND and TRANSIT sample it stays 0.  That matters --
        // meas_model() adds mean(aspect_var) into the amplitude variance,
        // and the Mahalanobis ambiguity margin uses the same sum, so both
        // correctly reduce to std_amp^2 away from rotation landmarks.
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
        // Absolute room-frame heading the rotation STARTED from, i.e.
        // the bearing from this landmark to B1.  Without it the Fourier
        // phase is measured from an arbitrary pose and cannot be
        // compared between landmarks -- which is why these coefficients
        // sat unused.  alpha_room = start_heading + turn_angle.
        float rot_start_heading;

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
    bool     is_new;      // first frame: no previous pos to difference
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
    // v0.8: set by scene_apply_self_suppression() when this track is
    // (probably) the operator carrying the undocked probe.  The track
    // is NOT hidden — the renderer dims it and labels it "YOU".
    uint8_t  is_self;
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
    // v0.85: was `rf_distance`, an RMS amplitude difference normalized by
    // each beacon's observed amp RANGE — a scale with no statistical
    // meaning, compared against a hand-picked 0.15.  Now the Mahalanobis
    // distance between the two landmarks' predicted observations under
    // the SAME per-channel sigma that meas_model() uses at inference
    // (std_amp^2 + aspect_var for amplitude, std_phase^2 for phase).
    // Units are sigma of the sensor's own noise, so it is comparable
    // across rooms, beacon counts and hardware, and it predicts the
    // confusions the solver will actually make rather than a proxy.
    //
    // For two Gaussian hypotheses with equal priors the probability of
    // confusing them is Phi(-sigma_distance / 2).
    float   sigma_distance;
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
    uint8_t   alias_pair_count;       // how many are STORED below (capped)
    AliasPair alias_pairs[CAL_MAX_ALIAS_PAIRS];   // the worst offenders
    // v0.85 ambiguity metrics.  A raw count is not comparable across
    // beacon counts: the number of far-pairs tested grows quadratically
    // with landmark count (39 at N=3, 98 at N=6), so requiring "zero"
    // gets strictly harder as the array gets BETTER.  These three are
    // properties of the deployment rather than of the ceremony.
    uint16_t  alias_pairs_found;      // UNCAPPED count below D_DETECT
    uint16_t  far_pairs_tested;       // denominator, for the rate
    // Minimum Mahalanobis separation over all physically-far pairs.
    // This is the peak sidelobe of the array's ambiguity function.
    float     ambiguity_margin_sigma;
    // Largest physical separation (ring-radius units) at which two
    // places were still confusable.  When healthy this is just the
    // resolution — only nearby points blur together.  When large, it
    // IS the alias distance.  One number, no threshold.
    float     resolution_sep;
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
// Honest cal feedback for the UI: is a capture window open, and how many
// frames has it actually buffered?
bool scene_capture_open();
int  scene_capture_frame_count();
int  scene_capture_kind();  // 0=none 1=stand 2=walk 3=rotate

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

// ═══════════════════════════════════════════════════════════════
//  v0.85 — EXTERIOR CONTACTS (the perimeter wire)
// ═══════════════════════════════════════════════════════════════
// Detections outside the walked region, where there is no learned
// h(p) and therefore no honest position.  These carry a BEARING and a
// sector width, never coordinates — a different kind of claim, not a
// weaker version of a track.  Renderers must not draw them as dots.
//
// Nothing that trips the sensor is ever discarded: residual the
// interior solve cannot explain arrives here rather than being
// dropped, which is why the coverage gate on birth_search does not
// cost detections.
int  scene_exterior_count();                       // active contacts
const ExteriorContact *scene_get_exterior(int idx); // nullptr if none

// True once the ROTATE at LM_OPPOSITE_RX has produced an aspect
// envelope.  Until then the perimeter detector runs at its
// uncalibrated default sensitivity (which is less sensitive, not more).
bool scene_exterior_calibrated();
float scene_exterior_amp_floor();      // aspect envelope low point, 0..1
float scene_exterior_bearing_spread(); // radians

// ═══════════════════════════════════════════════════════════════
//  v0.8 — PROBE UNDOCK MODE
// ═══════════════════════════════════════════════════════════════
// These run on the PROBE unit (self-localization) and the ANCHOR unit
// (self-suppression) respectively.
//
// Why the probe can self-locate at all: during the walk cal every unit
// buffers ITS OWN frames into the capture buffer and folds them into
// its OWN local s_kernel[] (see the comment above scene_observe).  On
// the probe, that kernel is therefore already a probe-perspective
// kernel: "what beacon b looks like from position p", sampled at every
// landmark the user walked to.  What the probe never did was run
// scene_finalize_cal() — the anchor does that.  So the probe's kernel
// exists but its per-beacon weights/normalization were never computed.
// scene_finalize_probe_kernel() fills exactly that gap.
//
// Call once on the PROBE at the end of cal (ST_CAL_FINALIZE).  Cheap.
// Returns true if the local kernel is usable for self-localization.
bool scene_finalize_probe_kernel();

// True once scene_finalize_probe_kernel() has succeeded.
bool scene_probe_kernel_ready();

// Estimate where THIS unit (the probe) currently is, by running the
// inverse sensor model against the local probe-perspective kernel and
// solving for a single position rather than a set of target tracks.
//
//   out_pos[2]  — normalized geometry-frame position
//   out_cov[3]  — flattened 2x2 covariance (cov_xx, cov_yy, cov_xy)
//   out_conf    — 0..1, from local observability + fit quality
//
// Returns false (and leaves the outputs untouched) if the kernel isn't
// ready or the current frame has too few valid beacons.
bool scene_estimate_probe_position(const FrameObservation &obs,
                                   float out_pos[2],
                                   float out_cov[3],
                                   float *out_conf);

// ANCHOR side.  Post-processing pass over the current track list: any
// active track within SELF_MAHALANOBIS_THRESH of the reported probe
// position is tagged TargetTrack::is_self.  Call after scene_update().
void scene_apply_self_suppression(const float probe_pos[2],
                                  const float probe_cov[3]);

// Clear every is_self tag (used when the probe redocks or goes stale).
void scene_clear_self_suppression();
