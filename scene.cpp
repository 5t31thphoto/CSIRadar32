// ═══════════════════════════════════════════════════════════════
//  scene.cpp — reconstruction engine
//
//  Cal phase: builds a KernelSample[] database from user-scripted
//    walk (landmark stands, transit walks, rotate-in-place).
//  Runtime phase: matching-pursuit sparse solve of the occupancy
//    field from current observation, temporal-linking into tracks.
// ═══════════════════════════════════════════════════════════════
#include "scene.h"
#include "peer.h"
#include <math.h>
#include <string.h>

// ── Module-static storage ─────────────────────────────────────
static KernelSample s_kernel[KERNEL_MAX_SAMPLES];
static int          s_kernel_count = 0;
static bool         s_cal_complete = false;

static OccupancyField s_field = {};
static TargetTrack    s_tracks[TRACK_MAX] = {};
static uint8_t        s_next_track_id = 1;
static float          s_novelty = 0.0f;
static uint32_t       s_last_scene_update_ms = 0;

// Landmark positions in normalized geometry frame.  Populated by
// scene_derive_landmarks_from_geometry() from BeaconState.pos_x/y.
static float          s_lm_pos[LM_COUNT][2] = {};

// Cal-time state
static CalMode        s_cal_mode = CAL_MODE_UNKNOWN;
static bool           s_cal_active = false;
static bool           s_empty_room_ready = false;

// Currently-active capture (one at a time — script serializes them)
enum CaptureKind : uint8_t { CAP_NONE = 0, CAP_LANDMARK, CAP_TRANSIT, CAP_ROTATE };
static CaptureKind    s_cap_kind = CAP_NONE;
static LandmarkId     s_cap_landmark_a = LM_RX;
static LandmarkId     s_cap_landmark_b = LM_RX;
static uint32_t       s_cap_start_ms = 0;

// Accumulators for the currently-active capture.  Kept generic so
// the same buffer feeds STAND / WALK / ROTATE with different post-hoc
// resampling strategies.
//
// v0.3 design point (fixing a v0.3 first-cut bug): ANCHOR and PROBE
// frames are stored in SEPARATE buffers.  They measure fundamentally
// different quantities:
//   - ANCHOR observations = "what the runtime receiver sees when the
//     user (occluder) is at position P" — these ARE the kernel K(P).
//   - PROBE observations  = "what each beacon looks like from position
//     P" — these are a mobile field-strength meter used for
//     (a) arc-length parameterization of WALK transits (their RF-space
//         trajectory monotonically tracks the user's physical progress),
//     (b) geometry validation (PROBE at "beacon 1" should read beacon 1
//         strongest — cross-checks that the user actually stood where
//         they said).
// Collapsing them into one buffer was mixing categories.  Fixed.
#define CAP_MAX_FRAMES 400
struct CapFrame {
    uint32_t t_ms;
    float amp[MAX_BEACONS];
    float phase[MAX_BEACONS];
    float aoa[MAX_BEACONS];
    uint8_t aoa_valid[MAX_BEACONS];
    uint8_t beacon_valid[MAX_BEACONS];
};
static CapFrame  s_cap_anchor[CAP_MAX_FRAMES];  // this-unit observations
static int       s_cap_anchor_len = 0;
static CapFrame  s_cap_probe[CAP_MAX_FRAMES];   // peer PROBE-stream observations (ANCHOR-side only)
static int       s_cap_probe_len = 0;

// Feature normalization ranges (per beacon), computed at finalize.
static float s_amp_range_lo[MAX_BEACONS], s_amp_range_hi[MAX_BEACONS];
static float s_beacon_snr[MAX_BEACONS];

// Rotation-derived per-beacon reliability (populated in
// scene_end_rotate_capture from the amp variance during a fixed-position
// 360° rotation).  Higher rotation-variance = beacon's response is
// dominated by orientation more than position → less useful as a
// positional indicator → lower reliability.  Used as an additional
// weight in pursuit_score at INFERENCE time (not just once at finalize).
// Range: (0, 1] where 1 = orientation-invariant, 0 = pure aspect noise.
static float s_beacon_orient_reliability[MAX_BEACONS] = {1,1,1,1};

// Per-beacon runtime weights = SNR × orient_reliability × loop-closure penalty.
// Precomputed at finalize so the inner scoring loop just multiplies.
static float s_beacon_weight[MAX_BEACONS] = {1,1,1,1};

// Wizard step / landmark provenance for PROBE→ANCHOR packet tagging.
// Populated by wizard via scene_cal_note_step().
static uint8_t s_cal_cur_step = 0;
static uint8_t s_cal_cur_landmark = 0xFF;

// Adaptive background — per-beacon amp perturbation offset that slowly
// tracks the residual when three gates all hold (quiet scene, no tracks,
// small |r|).  Subtracted from raw observation before pursuit.
static float s_bg_amp[MAX_BEACONS] = {};

// Cached alias-pair table copied from the CalReport at finalize.  Runtime
// checks track positions against this table to flag ambiguity.
static uint8_t   s_alias_count = 0;
static AliasPair s_alias_pairs[CAL_MAX_ALIAS_PAIRS] = {};

// Empty-room reference (already handled by csi.cpp baseline; we just
// note we've observed the ack).  Perturbations arriving via
// FrameObservation are ALREADY background-subtracted upstream.

// ═══════════════════════════════════════════════════════════════
//  UTILITY
// ═══════════════════════════════════════════════════════════════
static inline float sq(float x) { return x * x; }
static inline float wrap_pi(float x) {
    while (x >  (float)M_PI) x -= 2.0f * (float)M_PI;
    while (x < -(float)M_PI) x += 2.0f * (float)M_PI;
    return x;
}
static inline float dist2(float ax, float ay, float bx, float by) {
    return sq(ax - bx) + sq(ay - by);
}

// Grid cell → normalized position (cell center)
static inline void cell_to_pos(int gx, int gy, float *x, float *y) {
    *x = -SCENE_EXTENT + (2.0f * SCENE_EXTENT) * ((float)gx + 0.5f) / (float)FIELD_DIM;
    *y = -SCENE_EXTENT + (2.0f * SCENE_EXTENT) * ((float)gy + 0.5f) / (float)FIELD_DIM;
}
static inline float clampf(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}
// Huber loss: quadratic below k, linear beyond.  Robust to outliers.
static inline float huber(float z) {
    float az = fabsf(z);
    return az <= HUBER_K ? 0.5f * z * z
                         : HUBER_K * (az - 0.5f * HUBER_K);
}

// ── Small dense linear algebra for the GN solver ────────────────
// Symmetric positive-definite N×N solve via Cholesky.  N is small
// (≤ MAX_STATE_DIM = 12), matrices live on the stack.  Returns true
// if A is strictly PD (all diagonals stayed positive during
// factorization); false → caller should fall back to gradient step.
static bool cholesky_solve(float *A, float *b, float *x, int N) {
    // A is N×N row-major, modified in place to hold L (lower triangular).
    for (int i = 0; i < N; i++) {
        float diag = A[i*N + i];
        for (int k = 0; k < i; k++) diag -= A[i*N + k] * A[i*N + k];
        if (diag <= 1e-9f) return false;
        A[i*N + i] = sqrtf(diag);
        for (int j = i + 1; j < N; j++) {
            float sum = A[j*N + i];
            for (int k = 0; k < i; k++) sum -= A[j*N + k] * A[i*N + k];
            A[j*N + i] = sum / A[i*N + i];
        }
    }
    // Forward substitution: L y = b
    float y[MAX_STATE_DIM];
    for (int i = 0; i < N; i++) {
        float sum = b[i];
        for (int k = 0; k < i; k++) sum -= A[i*N + k] * y[k];
        y[i] = sum / A[i*N + i];
    }
    // Back substitution: L^T x = y
    for (int i = N - 1; i >= 0; i--) {
        float sum = y[i];
        for (int k = i + 1; k < N; k++) sum -= A[k*N + i] * x[k];
        x[i] = sum / A[i*N + i];
    }
    return true;
}

// ═══════════════════════════════════════════════════════════════
//  LIFECYCLE
// ═══════════════════════════════════════════════════════════════
void scene_begin() {
    scene_reset();
}

void scene_reset() {
    s_kernel_count = 0;
    s_cal_complete = false;
    s_cal_active = false;
    s_empty_room_ready = false;
    s_cap_kind = CAP_NONE;
    s_cap_anchor_len = 0;
    s_cap_probe_len = 0;
    memset(&s_field, 0, sizeof(s_field));
    memset(s_tracks, 0, sizeof(s_tracks));
    memset(s_lm_pos, 0, sizeof(s_lm_pos));
    for (int b = 0; b < MAX_BEACONS; b++) {
        s_amp_range_lo[b] = 0;
        s_amp_range_hi[b] = 1;
        s_beacon_snr[b] = 1;
        s_beacon_orient_reliability[b] = 1;
        s_beacon_weight[b] = 1;
        s_bg_amp[b] = 0;
    }
    s_alias_count = 0;
    memset(s_alias_pairs, 0, sizeof(s_alias_pairs));
    s_novelty = 0;
    s_next_track_id = 1;
    s_last_scene_update_ms = 0;
    s_cal_cur_step = 0;
    s_cal_cur_landmark = 0xFF;
}

// ═══════════════════════════════════════════════════════════════
//  LANDMARK POSITIONS
// ═══════════════════════════════════════════════════════════════
// Normalize beacon triangle so its centroid is at origin and its
// average vertex distance from centroid is 1.  Then derive landmarks
// from those normalized positions.  This keeps the kernel and the
// field in a scale-invariant frame regardless of physical geometry.
void scene_derive_landmarks_from_geometry() {
    // Collect active beacon nominal positions
    float bx[MAX_BEACONS], by[MAX_BEACONS];
    int n = 0;
    for (int i = 0; i < MAX_BEACONS && n < 3; i++) {
        if (!g_app.beacon[i].active) continue;
        bx[n] = g_app.beacon[i].pos_x;
        by[n] = g_app.beacon[i].pos_y;
        n++;
    }
    if (n < 1) {
        // No beacons — leave landmarks at origin
        for (int i = 0; i < LM_COUNT; i++) { s_lm_pos[i][0] = 0; s_lm_pos[i][1] = 0; }
        return;
    }
    // Centroid
    float cx = 0, cy = 0;
    for (int i = 0; i < n; i++) { cx += bx[i]; cy += by[i]; }
    cx /= n; cy /= n;
    // Mean radius from centroid
    float r_sum = 0;
    for (int i = 0; i < n; i++) r_sum += sqrtf(sq(bx[i] - cx) + sq(by[i] - cy));
    float r = (r_sum / n);
    if (r < 1e-4f) r = 1.0f;

    // RX at origin in the geometry frame (physical origin)
    float rx_x = (0.0f - cx) / r;
    float rx_y = (0.0f - cy) / r;

    // Normalize beacon positions
    float nbx[3], nby[3];
    for (int i = 0; i < n; i++) {
        nbx[i] = (bx[i] - cx) / r;
        nby[i] = (by[i] - cy) / r;
    }
    // Fill in any missing beacon slots with the last available position
    for (int i = n; i < 3; i++) { nbx[i] = nbx[n-1]; nby[i] = nby[n-1]; }

    s_lm_pos[LM_RX][0]        = rx_x;         s_lm_pos[LM_RX][1]        = rx_y;
    s_lm_pos[LM_BEACON_1][0]  = nbx[0];       s_lm_pos[LM_BEACON_1][1]  = nby[0];
    s_lm_pos[LM_BEACON_2][0]  = nbx[1];       s_lm_pos[LM_BEACON_2][1]  = nby[1];
    s_lm_pos[LM_BEACON_3][0]  = nbx[2];       s_lm_pos[LM_BEACON_3][1]  = nby[2];
    s_lm_pos[LM_CENTROID][0]  = 0;            s_lm_pos[LM_CENTROID][1]  = 0;
    s_lm_pos[LM_MID_12][0]    = 0.5f*(nbx[0]+nbx[1]); s_lm_pos[LM_MID_12][1] = 0.5f*(nby[0]+nby[1]);
    s_lm_pos[LM_MID_23][0]    = 0.5f*(nbx[1]+nbx[2]); s_lm_pos[LM_MID_23][1] = 0.5f*(nby[1]+nby[2]);
    s_lm_pos[LM_MID_13][0]    = 0.5f*(nbx[0]+nbx[2]); s_lm_pos[LM_MID_13][1] = 0.5f*(nby[0]+nby[2]);
    // Opposite RX = reflect centroid through RX (or just past RX away from beacons)
    s_lm_pos[LM_OPPOSITE_RX][0] = 2.0f * rx_x;
    s_lm_pos[LM_OPPOSITE_RX][1] = 2.0f * rx_y;
}

void scene_landmark_pos(LandmarkId id, float *out_x, float *out_y) {
    if (id < LM_COUNT) { *out_x = s_lm_pos[id][0]; *out_y = s_lm_pos[id][1]; }
    else { *out_x = 0; *out_y = 0; }
}

// ═══════════════════════════════════════════════════════════════
//  CAL LIFECYCLE
// ═══════════════════════════════════════════════════════════════
void scene_cal_begin(CalMode mode) {
    scene_reset();
    s_cal_mode = mode;
    s_cal_active = true;
    s_empty_room_ready = false;
    Serial.printf("[scene] cal begin mode=%d\n", (int)mode);
}

void scene_cal_abort() {
    s_cal_active = false;
    s_cap_kind = CAP_NONE;
    s_cap_anchor_len = 0;
    s_cap_probe_len = 0;
    Serial.println("[scene] cal aborted");
}

void scene_cal_ack_empty_room() {
    s_empty_room_ready = true;
    Serial.println("[scene] empty-room baseline acknowledged");
}

// Called by wizard when it enters/exits a step so PROBE-side outgoing
// packets can carry real step/landmark provenance (v0.3 first-cut sent
// 0 / 0xFF placeholders).  Both PROBE and ANCHOR track this locally.
void scene_cal_note_step(uint8_t step_idx, uint8_t landmark_id) {
    s_cal_cur_step = step_idx;
    s_cal_cur_landmark = landmark_id;
}

bool scene_cal_complete() { return s_cal_complete; }

// ═══════════════════════════════════════════════════════════════
//  CAPTURE HANDLERS
// ═══════════════════════════════════════════════════════════════
static void cap_reset() {
    s_cap_kind = CAP_NONE;
    s_cap_anchor_len = 0;
    s_cap_probe_len = 0;
}

void scene_begin_landmark_capture(LandmarkId lm) {
    cap_reset();
    s_cap_kind = CAP_LANDMARK;
    s_cap_landmark_a = lm;
    s_cap_landmark_b = lm;
    s_cap_start_ms = millis();
    // Announce to PROBE (from ANCHOR, if stereo) so PROBE tags its stream
    if (g_app.peer.cal_role == CAL_ROLE_ANCHOR)
        peer_send_command(PEER_OP_CAL_BEGIN, (uint8_t)lm);
    Serial.printf("[scene] STAND lm=%u\n", (unsigned)lm);
}

void scene_begin_transit_capture(LandmarkId from, LandmarkId to) {
    cap_reset();
    s_cap_kind = CAP_TRANSIT;
    s_cap_landmark_a = from;
    s_cap_landmark_b = to;
    s_cap_start_ms = millis();
    if (g_app.peer.cal_role == CAL_ROLE_ANCHOR)
        peer_send_command(PEER_OP_CAL_BEGIN, 0xFF, ((uint16_t)from << 8) | (uint16_t)to);
    Serial.printf("[scene] WALK %u→%u\n", (unsigned)from, (unsigned)to);
}

void scene_begin_rotate_capture(LandmarkId at) {
    cap_reset();
    s_cap_kind = CAP_ROTATE;
    s_cap_landmark_a = at;
    s_cap_landmark_b = at;
    s_cap_start_ms = millis();
    if (g_app.peer.cal_role == CAL_ROLE_ANCHOR)
        peer_send_command(PEER_OP_CAL_BEGIN, (uint8_t)at, 0xFFFF);
    Serial.printf("[scene] ROTATE at lm=%u\n", (unsigned)at);
}

// Fold accumulated frames into a KernelSample entry.  Common code
// path for STAND / TRANSIT-slice / ROTATE captures.  Operates on a
// specified buffer + index range — buffer is always ANCHOR (that's
// what runtime will see); PROBE data is used elsewhere for
// parameterization / validation, not for the kernel itself.
static void write_kernel_sample_from_range(const CapFrame *buf,
                                           int start, int end,
                                           float pos_x, float pos_y,
                                           uint8_t lm_id,
                                           uint8_t transit_from,
                                           uint8_t transit_to,
                                           float transit_frac) {
    if (s_kernel_count >= KERNEL_MAX_SAMPLES) return;
    if (end <= start) return;

    KernelSample &s = s_kernel[s_kernel_count];
    memset(&s, 0, sizeof(s));
    s.pos[0] = pos_x;
    s.pos[1] = pos_y;
    s.landmark_id  = lm_id;
    s.transit_from = transit_from;
    s.transit_to   = transit_to;
    s.transit_frac = transit_frac;
    s.n_beacons = MAX_BEACONS;

    // Per-beacon: mean amp + std amp, circular-mean phase + AoA.
    // Phase and AoA are angular quantities — arithmetic mean is wrong
    // near the ±π boundary, so we average unit vectors and take atan2.
    for (int b = 0; b < MAX_BEACONS; b++) {
        double sum_amp = 0, sum_amp2 = 0;
        double sum_phase_i = 0, sum_phase_q = 0;
        double sum_aoa_i = 0, sum_aoa_q = 0;
        int n_ok = 0, n_aoa = 0;
        for (int i = start; i < end; i++) {
            if (!buf[i].beacon_valid[b]) continue;
            float a = buf[i].amp[b];
            sum_amp  += a;
            sum_amp2 += (double)a * a;
            float ph = buf[i].phase[b];
            sum_phase_i += cos(ph);
            sum_phase_q += sin(ph);
            if (buf[i].aoa_valid[b]) {
                float ang = buf[i].aoa[b];
                sum_aoa_i += cos(ang);
                sum_aoa_q += sin(ang);
                n_aoa++;
            }
            n_ok++;
        }
        if (n_ok == 0) continue;
        float mean_a = (float)(sum_amp / n_ok);
        float var_a  = (float)(sum_amp2 / n_ok) - sq(mean_a);
        if (var_a < 0) var_a = 0;
        s.b[b].mean_amp = mean_a;
        s.b[b].std_amp  = sqrtf(var_a);
        s.b[b].mean_phase = atan2f((float)sum_phase_q, (float)sum_phase_i);
        // Circular std for angular quantities: for a von Mises-like
        // distribution, R = |mean of unit vectors| ∈ [0, 1] gives
        // std_ang ≈ sqrt(-2·ln R).  Matches a Gaussian sigma at small
        // dispersions (R→1) and diverges as the distribution goes
        // uniform (R→0).  Preferable to (1−R) if we ever plug these
        // into a likelihood.
        float R_phase = sqrtf((float)(sum_phase_i*sum_phase_i + sum_phase_q*sum_phase_q))
                        / (float)n_ok;
        s.b[b].std_phase = sqrtf(fmaxf(0.0f, -2.0f * logf(fmaxf(1e-4f, R_phase))));
        s.b[b].sample_count = (uint16_t)n_ok;
        s.b[b].saw_aoa = (uint8_t)(n_aoa < 255 ? n_aoa : 255);
        if (n_aoa > 0) {
            s.b[b].mean_aoa_dev = atan2f((float)sum_aoa_q, (float)sum_aoa_i);
            float R_aoa = sqrtf((float)(sum_aoa_i*sum_aoa_i + sum_aoa_q*sum_aoa_q))
                          / (float)n_aoa;
            s.b[b].std_aoa = sqrtf(fmaxf(0.0f, -2.0f * logf(fmaxf(1e-4f, R_aoa))));
        }
    }
    s_kernel_count++;
}

void scene_end_landmark_capture() {
    if (s_cap_kind != CAP_LANDMARK) return;
    float px, py;
    scene_landmark_pos(s_cap_landmark_a, &px, &py);
    write_kernel_sample_from_range(s_cap_anchor, 0, s_cap_anchor_len, px, py,
                                    (uint8_t)s_cap_landmark_a, 0xFF, 0xFF, 0);
    Serial.printf("[scene] STAND done lm=%u anchor_frames=%d probe_frames=%d total_samples=%d\n",
                  (unsigned)s_cap_landmark_a, s_cap_anchor_len, s_cap_probe_len,
                  s_kernel_count);
    if (g_app.peer.cal_role == CAL_ROLE_ANCHOR) peer_send_command(PEER_OP_CAL_END);
    cap_reset();
}

// ── Transit resampling — arc-length in RF-space, not time ─────
// The whole point of PROBE streaming during cal is that PROBE's
// moving observations DO change monotonically as the user walks
// from landmark A to landmark B (each beacon's amplitude at PROBE
// varies smoothly with PROBE's own position).  We use PROBE's
// fingerprint trajectory as the arc-length parameter, so walking
// speed variability drops out entirely.
//
// If PROBE data isn't available (solo mode; or stereo with peer
// drop), we fall back to time-based slicing — honestly worse but
// still functional.
static void compute_arclength_boundaries(int N_slices,
                                         int anchor_len,
                                         const CapFrame *probe, int probe_len,
                                         int *out_boundaries /*len N_slices+1*/) {
    // Baseline: uniform-time slicing over the ANCHOR buffer.
    for (int k = 0; k <= N_slices; k++) {
        out_boundaries[k] = (k * anchor_len) / N_slices;
    }

    // If PROBE stream is present and reasonably synced, upgrade to
    // arc-length-in-fingerprint-space.
    if (probe_len < 8) return;

    // Build cumulative arc length of PROBE's own observation trajectory.
    // We treat PROBE's per-beacon amplitude vector as the coordinate;
    // per-frame delta = Euclidean distance in beacon-amp space.
    // Normalize by dividing each beacon channel by its own peak so
    // no single loud channel dominates arc length.
    float peak[MAX_BEACONS] = {0};
    for (int i = 0; i < probe_len; i++)
        for (int b = 0; b < MAX_BEACONS; b++)
            if (probe[i].beacon_valid[b] && fabsf(probe[i].amp[b]) > peak[b])
                peak[b] = fabsf(probe[i].amp[b]);
    for (int b = 0; b < MAX_BEACONS; b++)
        if (peak[b] < 1e-6f) peak[b] = 1.0f;

    // Cumulative arc length, sized to PROBE frames
    static float cum[CAP_MAX_FRAMES + 1];
    cum[0] = 0;
    for (int i = 1; i < probe_len; i++) {
        float d2 = 0;
        for (int b = 0; b < MAX_BEACONS; b++) {
            if (!probe[i].beacon_valid[b] || !probe[i-1].beacon_valid[b]) continue;
            float dv = (probe[i].amp[b] - probe[i-1].amp[b]) / peak[b];
            d2 += dv * dv;
        }
        cum[i] = cum[i-1] + sqrtf(d2);
    }
    float total = cum[probe_len - 1];
    if (total < 1e-3f) return;   // no motion in PROBE fingerprint; keep uniform

    // For each slice boundary, find the PROBE frame index at that
    // fraction of total arc length, then translate to an ANCHOR frame
    // index using time (probe.t_ms vs anchor.t_ms) — they were captured
    // concurrently on the same wall clock during cal.
    for (int k = 0; k <= N_slices; k++) {
        float target = ((float)k / (float)N_slices) * total;
        // Binary-ish search in cum[] for the smallest i with cum[i] >= target
        int lo = 0, hi = probe_len - 1;
        while (lo < hi) {
            int mid = (lo + hi) / 2;
            if (cum[mid] < target) lo = mid + 1;
            else hi = mid;
        }
        // Map PROBE t_ms → nearest ANCHOR frame index
        uint32_t t = probe[lo].t_ms;
        int anchor_idx = 0;
        int best_dt = 0x7FFFFFFF;
        for (int j = 0; j < anchor_len; j++) {
            int dt = (int)(s_cap_anchor[j].t_ms) - (int)t;
            if (dt < 0) dt = -dt;
            if (dt < best_dt) { best_dt = dt; anchor_idx = j; }
        }
        out_boundaries[k] = anchor_idx;
    }
    // Enforce monotone non-decreasing (numerical guard)
    for (int k = 1; k <= N_slices; k++)
        if (out_boundaries[k] < out_boundaries[k-1])
            out_boundaries[k] = out_boundaries[k-1];
    Serial.printf("[scene] transit arc-length OK: probe_arc=%.2f probe_frames=%d\n",
                  (double)total, probe_len);
}

void scene_end_transit_capture() {
    if (s_cap_kind != CAP_TRANSIT) return;
    if (s_cap_anchor_len < 4) { cap_reset(); return; }
    const int N_SLICES = 8;
    float ax, ay, bx, by;
    scene_landmark_pos(s_cap_landmark_a, &ax, &ay);
    scene_landmark_pos(s_cap_landmark_b, &bx, &by);

    int boundaries[N_SLICES + 1];
    compute_arclength_boundaries(N_SLICES, s_cap_anchor_len,
                                  s_cap_probe, s_cap_probe_len, boundaries);

    for (int k = 0; k < N_SLICES; k++) {
        int start = boundaries[k];
        int end   = boundaries[k + 1];
        if (end <= start + 1) end = start + 2;
        if (end > s_cap_anchor_len) end = s_cap_anchor_len;
        if (start >= s_cap_anchor_len) break;
        float frac = ((float)k + 0.5f) / (float)N_SLICES;
        float px = ax + (bx - ax) * frac;
        float py = ay + (by - ay) * frac;
        write_kernel_sample_from_range(s_cap_anchor, start, end, px, py,
                                        0xFF,
                                        (uint8_t)s_cap_landmark_a,
                                        (uint8_t)s_cap_landmark_b,
                                        frac);
    }
    Serial.printf("[scene] WALK done %u→%u anchor=%d probe=%d slices=%d total=%d\n",
                  (unsigned)s_cap_landmark_a, (unsigned)s_cap_landmark_b,
                  s_cap_anchor_len, s_cap_probe_len, N_SLICES, s_kernel_count);
    if (g_app.peer.cal_role == CAL_ROLE_ANCHOR) peer_send_command(PEER_OP_CAL_END);
    cap_reset();
}

// Extract per-frame orientation angle θ during a rotation capture,
// using the PROBE-during-rotation stream.  PROBE (on the person's
// chest) sees beacons from a rotating viewpoint; its per-beacon
// amplitude vector traces a smooth periodic curve in feature space.
// We recover θ by:
//   1. Computing the total cumulative arc-length of PROBE's amp
//      trajectory (same idea as the transit arc-length code)
//   2. Assuming that arc-length is monotonic in θ over one full
//      rotation, and that the user completed approximately one
//      full turn during the capture window
//   3. Assigning θ_i = 2π · (cum[i] / total) to each PROBE frame
//   4. Mapping each ANCHOR frame's θ from its nearest-in-time
//      PROBE frame (both were captured concurrently)
//
// If PROBE data isn't available (solo mode / peer drop), we fall
// back to time-linear θ, which assumes the user rotated at constant
// rate — honestly worse but still yields a defensible Fourier fit.
//
// Populates theta_out[i] for i in [0, s_cap_anchor_len).
static void extract_rotation_theta(float *theta_out) {
    // Time-linear fallback first
    if (s_cap_anchor_len <= 1) {
        for (int i = 0; i < s_cap_anchor_len; i++) theta_out[i] = 0;
        return;
    }
    uint32_t t0 = s_cap_anchor[0].t_ms;
    uint32_t tN = s_cap_anchor[s_cap_anchor_len - 1].t_ms;
    float dt_total = (tN > t0) ? (float)(tN - t0) : 1.0f;
    for (int i = 0; i < s_cap_anchor_len; i++) {
        theta_out[i] = 2.0f * (float)M_PI *
                       ((float)(s_cap_anchor[i].t_ms - t0) / dt_total);
    }
    // Upgrade to PROBE-arc-length θ if we have enough PROBE data
    if (s_cap_probe_len < 8) {
        Serial.println("[scene] ROTATE θ: time-linear (no PROBE stream)");
        return;
    }
    // Per-beacon peak for normalization (so no channel dominates arc length)
    float peak[MAX_BEACONS] = {0};
    for (int i = 0; i < s_cap_probe_len; i++)
        for (int b = 0; b < MAX_BEACONS; b++)
            if (s_cap_probe[i].beacon_valid[b] && fabsf(s_cap_probe[i].amp[b]) > peak[b])
                peak[b] = fabsf(s_cap_probe[i].amp[b]);
    for (int b = 0; b < MAX_BEACONS; b++)
        if (peak[b] < 1e-6f) peak[b] = 1.0f;
    // Cumulative arc-length of PROBE's amp trajectory
    static float cum[CAP_MAX_FRAMES + 1];
    cum[0] = 0;
    for (int i = 1; i < s_cap_probe_len; i++) {
        float d2 = 0;
        for (int b = 0; b < MAX_BEACONS; b++) {
            if (!s_cap_probe[i].beacon_valid[b] || !s_cap_probe[i-1].beacon_valid[b]) continue;
            float dv = (s_cap_probe[i].amp[b] - s_cap_probe[i-1].amp[b]) / peak[b];
            d2 += dv * dv;
        }
        cum[i] = cum[i-1] + sqrtf(d2);
    }
    float total = cum[s_cap_probe_len - 1];
    if (total < 1e-3f) {
        Serial.println("[scene] ROTATE θ: time-linear fallback (no PROBE motion)");
        return;
    }
    // For each ANCHOR frame, find nearest PROBE frame in time,
    // then θ_anchor = 2π · (cum[nearest_probe] / total)
    for (int i = 0; i < s_cap_anchor_len; i++) {
        uint32_t t = s_cap_anchor[i].t_ms;
        int best_j = 0;
        int best_dt = 0x7FFFFFFF;
        for (int j = 0; j < s_cap_probe_len; j++) {
            int dt = (int)s_cap_probe[j].t_ms - (int)t;
            if (dt < 0) dt = -dt;
            if (dt < best_dt) { best_dt = dt; best_j = j; }
        }
        theta_out[i] = 2.0f * (float)M_PI * (cum[best_j] / total);
    }
    Serial.printf("[scene] ROTATE θ: PROBE-arc-length OK (probe_arc=%.2f)\n",
                  (double)total);
}

// Fit a 5-coefficient Fourier basis to (θ_i, amp_i) samples:
//   amp(θ) ≈ a0 + a1·cos(θ) + b1·sin(θ) + a2·cos(2θ) + b2·sin(2θ)
// via ordinary least squares — but since the basis functions are
// (approximately) orthogonal over a full rotation, we can use the
// direct discrete-Fourier estimator instead of solving a full linear
// system.  Also returns aspect_var = mean squared deviation of the
// fit from a0 (i.e. the total non-DC energy = "how much does this
// beacon's response depend on aspect at all").
static void fit_rotation_fourier(const float *theta, const float *amp,
                                 const uint8_t *valid, int n,
                                 float coeff_out[5], float *aspect_var_out,
                                 float *fit_std_out) {
    if (n < 5) {
        for (int k = 0; k < 5; k++) coeff_out[k] = 0;
        *aspect_var_out = 0;
        *fit_std_out = 0;
        return;
    }
    double s0 = 0, s_c1 = 0, s_s1 = 0, s_c2 = 0, s_s2 = 0;
    double y_c1 = 0, y_s1 = 0, y_c2 = 0, y_s2 = 0;
    double sum_y = 0;
    int nv = 0;
    for (int i = 0; i < n; i++) {
        if (!valid[i]) continue;
        float th = theta[i];
        float c1 = cosf(th), s1 = sinf(th);
        float c2 = cosf(2*th), s2 = sinf(2*th);
        s0   += 1;
        s_c1 += c1;   s_s1 += s1;
        s_c2 += c2;   s_s2 += s2;
        sum_y += amp[i];
        y_c1 += amp[i] * c1;
        y_s1 += amp[i] * s1;
        y_c2 += amp[i] * c2;
        y_s2 += amp[i] * s2;
        nv++;
    }
    if (nv < 5) {
        for (int k = 0; k < 5; k++) coeff_out[k] = 0;
        *aspect_var_out = 0;
        *fit_std_out = 0;
        return;
    }
    // For densely uniform θ over [0, 2π), cross-terms vanish and the
    // coefficients reduce to sum_y/N and 2·sum(y·basis)/N.  Our θ is
    // approximately uniform (PROBE arc-length gives us this), so use
    // that estimator — cheap, no matrix inverse, and stable.
    float a0 = (float)(sum_y / nv);
    float a1 = (float)(2.0 * y_c1 / nv);
    float b1 = (float)(2.0 * y_s1 / nv);
    float a2 = (float)(2.0 * y_c2 / nv);
    float b2 = (float)(2.0 * y_s2 / nv);
    coeff_out[0] = a0;
    coeff_out[1] = a1; coeff_out[2] = b1;
    coeff_out[3] = a2; coeff_out[4] = b2;
    // aspect_var = Parseval: half the sum of squared non-DC coeffs
    // (approximation of ∫(amp(θ) − a0)² dθ / 2π)
    *aspect_var_out = 0.5f * (a1*a1 + b1*b1 + a2*a2 + b2*b2);
    // Residual std: how well the 5-term fit actually explains the data.
    // Wide residual = aspect is more complex than 2nd harmonic captures
    // (fold back into aspect uncertainty).
    double sse = 0;
    for (int i = 0; i < n; i++) {
        if (!valid[i]) continue;
        float th = theta[i];
        float pred = a0 + a1*cosf(th) + b1*sinf(th)
                        + a2*cosf(2*th) + b2*sinf(2*th);
        float e = amp[i] - pred;
        sse += (double)e * e;
    }
    *fit_std_out = sqrtf((float)(sse / nv));
}

void scene_end_rotate_capture() {
    if (s_cap_kind != CAP_ROTATE) return;
    float px, py;
    scene_landmark_pos(s_cap_landmark_a, &px, &py);
    if (s_cap_anchor_len < 6) {
        // Too few frames to fit anything meaningful — fall back to
        // v0.4 behavior (write one representative sample, no aspect model).
        Serial.printf("[scene] ROTATE too few frames (%d) — no Fourier fit\n",
                      s_cap_anchor_len);
        write_kernel_sample_from_range(s_cap_anchor, 0, s_cap_anchor_len,
                                        px, py, (uint8_t)s_cap_landmark_a,
                                        0xFF, 0xFF, 0);
        if (g_app.peer.cal_role == CAL_ROLE_ANCHOR) peer_send_command(PEER_OP_CAL_END);
        cap_reset();
        return;
    }

    // Extract θ per ANCHOR frame from PROBE-during-rotation stream
    static float theta[CAP_MAX_FRAMES];
    extract_rotation_theta(theta);

    // Write the aspect-averaged kernel sample first (that becomes the
    // sample at this landmark)
    write_kernel_sample_from_range(s_cap_anchor, 0, s_cap_anchor_len,
                                    px, py, (uint8_t)s_cap_landmark_a,
                                    0xFF, 0xFF, 0);
    KernelSample &ks = s_kernel[s_kernel_count - 1];   // just-written

    // For each beacon, fit the Fourier basis over θ; populate aspect_var
    // and fourier[] into the kernel sample.  Also keep the legacy
    // diagnostic scalar (s_beacon_orient_reliability[b]) for the cal-
    // results screen — it's a global roll-up over all rotation landmarks.
    static float amp_series[CAP_MAX_FRAMES];
    static uint8_t valid_series[CAP_MAX_FRAMES];
    for (int b = 0; b < MAX_BEACONS; b++) {
        int nv = 0;
        for (int i = 0; i < s_cap_anchor_len; i++) {
            valid_series[i] = s_cap_anchor[i].beacon_valid[b] ? 1 : 0;
            amp_series[i]   = s_cap_anchor[i].amp[b];
            if (valid_series[i]) nv++;
        }
        if (nv < 5) continue;
        float coeff[5];
        float aspect_var, fit_std;
        fit_rotation_fourier(theta, amp_series, valid_series,
                             s_cap_anchor_len, coeff, &aspect_var, &fit_std);
        // Store into the just-written kernel sample
        for (int k = 0; k < 5; k++) ks.b[b].fourier[k] = coeff[k];
        // aspect_var is Parseval energy of non-DC harmonics + a term
        // for residual variance NOT captured by the 5-term fit
        ks.b[b].aspect_var = aspect_var + fit_std * fit_std;

        // Diagnostic scalar: how relatively-jittery is this beacon
        // across rotation?  Kept for the cal-results screen.
        float rel = sqrtf(ks.b[b].aspect_var) / (fabsf(coeff[0]) + 0.05f);
        float reliability = 1.0f / (1.0f + 4.0f * rel);
        if (reliability < 0.1f) reliability = 0.1f;
        // Multiple ROTATE landmarks will call this — take the min
        // (worst case) rather than overwriting
        if (reliability < s_beacon_orient_reliability[b] ||
            s_beacon_orient_reliability[b] == 1.0f) {
            s_beacon_orient_reliability[b] = reliability;
        }
    }
    Serial.printf("[scene] ROTATE lm=%u done anchor=%d probe=%d "
                  "aspect_var=[%.3f %.3f %.3f]\n",
                  (unsigned)s_cap_landmark_a, s_cap_anchor_len, s_cap_probe_len,
                  (double)ks.b[0].aspect_var,
                  (double)ks.b[1].aspect_var,
                  (double)ks.b[2].aspect_var);
    if (g_app.peer.cal_role == CAL_ROLE_ANCHOR) peer_send_command(PEER_OP_CAL_END);
    cap_reset();
}

// ═══════════════════════════════════════════════════════════════
//  PEER STREAM (PROBE → ANCHOR during cal)
// ═══════════════════════════════════════════════════════════════
// ANCHOR side: called when a PROBE observation packet arrives.  These
// frames go into the PROBE-only buffer (s_cap_probe), NOT into the
// kernel buffer.  PROBE frames are used at transit finalization for
// arc-length parameterization, not to seed the kernel directly.
static void anchor_absorb_probe_frame(const PeerCalObservation &pkt) {
    if (!s_cal_active) return;
    if (s_cap_kind == CAP_NONE) return;
    if (s_cap_probe_len >= CAP_MAX_FRAMES) return;
    CapFrame &f = s_cap_probe[s_cap_probe_len];
    f.t_ms = pkt.rx_stamp_ms;
    memset(f.beacon_valid, 0, sizeof(f.beacon_valid));
    memset(f.aoa_valid,    0, sizeof(f.aoa_valid));
    for (int i = 0; i < pkt.n_beacons && i < MAX_BEACONS; i++) {
        uint8_t id = pkt.b[i].beacon_id;
        for (int b = 0; b < MAX_BEACONS; b++) {
            if (!g_app.beacon[b].active) continue;
            if (g_app.beacon[b].id != id) continue;
            f.amp[b]   = pkt.b[i].amp_perturbation;
            f.phase[b] = pkt.b[i].phase_perturbation;
            f.beacon_valid[b] = 1;
            if (pkt.b[i].have_aoa) {
                f.aoa[b] = pkt.b[i].aoa_rad;
                f.aoa_valid[b] = 1;
            }
            break;
        }
    }
    s_cap_probe_len++;
}

// PROBE side: called from scene_observe when cal is active.  Sends
// this frame to ANCHOR as a PeerCalObservation packet.  Now carries
// real wizard step / landmark provenance (fixed from v0.3 first-cut
// placeholders).
void scene_cal_transmit_probe_frame(const FrameObservation &obs) {
    if (g_app.peer.cal_role != CAL_ROLE_PROBE) return;
    if (!g_app.peer.peer_present) return;
    PeerCalObservation pkt = {};
    pkt.magic = PEER_CAL_MAGIC;
    pkt.rx_stamp_ms = obs.frame_ms;
    pkt.cur_step_idx = s_cal_cur_step;
    pkt.cur_landmark = s_cal_cur_landmark;
    pkt.n_beacons = 0;
    for (int i = 0; i < MAX_BEACONS && pkt.n_beacons < MAX_BEACONS; i++) {
        if (!obs.beacon[i].fresh) continue;
        auto &pb = pkt.b[pkt.n_beacons];
        pb.beacon_id = obs.beacon[i].beacon_id;
        pb.amp_perturbation   = obs.beacon[i].amp_perturbation;
        pb.phase_perturbation = obs.beacon[i].phase_perturbation;
        pb.have_aoa           = obs.beacon[i].have_aoa ? 1 : 0;
        pb.aoa_rad            = obs.beacon[i].aoa_rad;
        pb.aoa_conf           = obs.beacon[i].aoa_conf;
        pkt.n_beacons++;
    }
    if (pkt.n_beacons == 0) return;
    extern void peer_send_cal_observation(const PeerCalObservation &);
    peer_send_cal_observation(pkt);
}

void scene_cal_absorb_peer_stream() {
    // Peer packets are demuxed by peer.cpp and delivered via
    // scene_ingest_peer_cal.  Currently no per-tick post-processing.
}

void scene_ingest_peer_cal(const PeerCalObservation &pkt) {
    anchor_absorb_probe_frame(pkt);
}

// ═══════════════════════════════════════════════════════════════
//  scene_observe — sole ingest point for per-frame observations
// ═══════════════════════════════════════════════════════════════
// During cal capture windows, this-unit's frames are stored in the
// ANCHOR buffer (s_cap_anchor) — regardless of whether this unit is
// PROBE or ANCHOR.  Both buffers get filled in stereo mode: ANCHOR
// unit fills s_cap_anchor from its own CSI + also fills s_cap_probe
// from PROBE's peer-streamed packets.  PROBE unit only fills its own
// s_cap_anchor buffer.
//
// Solo mode: the single unit's frames go into s_cap_anchor and are
// used directly as the kernel (with the known degradation that the
// receiver moved with the target — honestly reduced accuracy).
void scene_observe(const FrameObservation &obs) {
    // 1) During cal capture, buffer this-unit's frame into anchor buf.
    if (s_cal_active && s_cap_kind != CAP_NONE && s_cap_anchor_len < CAP_MAX_FRAMES) {
        CapFrame &f = s_cap_anchor[s_cap_anchor_len];
        f.t_ms = obs.frame_ms;
        memset(f.beacon_valid, 0, sizeof(f.beacon_valid));
        memset(f.aoa_valid,    0, sizeof(f.aoa_valid));
        for (int b = 0; b < MAX_BEACONS; b++) {
            if (!obs.beacon[b].fresh) continue;
            f.amp[b]   = obs.beacon[b].amp_perturbation;
            f.phase[b] = obs.beacon[b].phase_perturbation;
            f.beacon_valid[b] = 1;
            if (obs.beacon[b].have_aoa) {
                f.aoa[b] = obs.beacon[b].aoa_rad;
                f.aoa_valid[b] = 1;
            }
        }
        s_cap_anchor_len++;
    }

    // 2) If we're PROBE in stereo cal, transmit this frame to ANCHOR.
    if (s_cal_active && g_app.peer.cal_role == CAL_ROLE_PROBE) {
        scene_cal_transmit_probe_frame(obs);
    }

    // 3) The reconstruction (runtime) reads the latest observation from
    //    the module-global g_last_obs (defined further down in this file).
    extern FrameObservation g_last_obs;
    g_last_obs = obs;
}

// ═══════════════════════════════════════════════════════════════
//  MODEL FINALIZATION
// ═══════════════════════════════════════════════════════════════
// After the walk sequence ends, compute per-beacon normalization ranges,
// per-beacon SNR (variance-across-landmarks / variance-within-landmark),
// cross-validation error (leave-one-landmark-out), and loop closure.
void scene_finalize_cal(CalReport &report) {
    memset(&report, 0, sizeof(report));
    report.mode = s_cal_mode;
    report.total_kernel_samples = (uint16_t)s_kernel_count;

    if (s_kernel_count < 3) {
        report.valid = false;
        s_cal_active = false;
        return;
    }

    // Count landmarks vs transits
    for (int i = 0; i < s_kernel_count; i++) {
        if (s_kernel[i].landmark_id != 0xFF) report.landmarks_captured++;
        else report.transit_samples++;
    }

    // Per-beacon: dynamic range across all kernel samples
    for (int b = 0; b < MAX_BEACONS; b++) {
        float lo = 1e30f, hi = -1e30f;
        for (int i = 0; i < s_kernel_count; i++) {
            if (s_kernel[i].b[b].sample_count == 0) continue;
            float v = s_kernel[i].b[b].mean_amp;
            if (v < lo) lo = v;
            if (v > hi) hi = v;
        }
        if (lo > hi) { lo = 0; hi = 1; }
        s_amp_range_lo[b] = lo;
        s_amp_range_hi[b] = (hi - lo) > 1e-6f ? hi : lo + 1.0f;

        // Per-beacon SNR = variance-across-landmarks / mean-within-landmark-noise
        double sum_across = 0, sum_across2 = 0;
        double sum_within_noise = 0;
        int n_across = 0, n_within = 0;
        for (int i = 0; i < s_kernel_count; i++) {
            if (s_kernel[i].b[b].sample_count == 0) continue;
            sum_across  += s_kernel[i].b[b].mean_amp;
            sum_across2 += sq(s_kernel[i].b[b].mean_amp);
            n_across++;
            sum_within_noise += s_kernel[i].b[b].std_amp;
            n_within++;
        }
        float across_var = 0, within_noise = 0;
        if (n_across > 1) {
            float m = sum_across / n_across;
            across_var = (float)(sum_across2 / n_across) - sq(m);
            if (across_var < 0) across_var = 0;
        }
        if (n_within > 0) within_noise = sum_within_noise / n_within;
        if (within_noise < 1e-6f) within_noise = 1e-6f;
        // SNR here = spatial-discriminability / within-landmark-noise.
        s_beacon_snr[b] = sqrtf(across_var) / within_noise;
        if (!isfinite(s_beacon_snr[b]) || s_beacon_snr[b] < 0.01f) s_beacon_snr[b] = 0.01f;
        // s_beacon_weight[b] is the composite runtime weight — SNR ×
        // orient reliability, further modulated by per-beacon loop
        // closure penalty (computed below).  Precomputing lets the
        // inner scoring loop just multiply.  We finish this below.
        s_beacon_weight[b] = s_beacon_snr[b] * s_beacon_orient_reliability[b];
        // What the cal-results screen shows.
        report.per_beacon_snr[b] = s_beacon_weight[b];
        report.per_beacon_orient_reliability[b] = s_beacon_orient_reliability[b];
    }

    // ── Per-landmark local Jacobians from adjacent transit slices ──
    // For each landmark L, find transit slices whose transit_from==L or
    // transit_to==L.  Each such slice sits at a known physical offset
    // from L; the amp delta over that offset is a finite-difference
    // estimate of ∂h/∂p.  Combine multiple neighboring slices via a
    // least-squares fit of (dx, dy) → damp.
    for (int i = 0; i < s_kernel_count; i++) {
        if (s_kernel[i].landmark_id == 0xFF) continue;
        uint8_t lm = s_kernel[i].landmark_id;
        for (int b = 0; b < MAX_BEACONS; b++) {
            if (s_kernel[i].b[b].sample_count == 0) continue;
            // Build a 2×2 normal-equation system for [gx, gy] over all
            // adjacent transit slices:
            //   sum_k (dxk² gx + dxk·dyk gy) = sum_k dxk · dampk
            //   sum_k (dxk·dyk gx + dyk² gy) = sum_k dyk · dampk
            double Axx = 0, Axy = 0, Ayy = 0, Rx = 0, Ry = 0;
            int n_adj = 0;
            for (int j = 0; j < s_kernel_count; j++) {
                if (s_kernel[j].landmark_id != 0xFF) continue;   // transit only
                if (s_kernel[j].transit_from != lm &&
                    s_kernel[j].transit_to   != lm) continue;
                if (s_kernel[j].b[b].sample_count == 0) continue;
                float dx = s_kernel[j].pos[0] - s_kernel[i].pos[0];
                float dy = s_kernel[j].pos[1] - s_kernel[i].pos[1];
                float damp = s_kernel[j].b[b].mean_amp - s_kernel[i].b[b].mean_amp;
                Axx += (double)dx * dx;
                Axy += (double)dx * dy;
                Ayy += (double)dy * dy;
                Rx  += (double)dx * damp;
                Ry  += (double)dy * damp;
                n_adj++;
            }
            if (n_adj < 2) continue;
            double det = Axx * Ayy - Axy * Axy;
            if (fabs(det) < 1e-8) continue;
            float gx = (float)((Rx * Ayy - Ry * Axy) / det);
            float gy = (float)((Axx * Ry - Axy * Rx) / det);
            s_kernel[i].b[b].grad_amp[0] = gx;
            s_kernel[i].b[b].grad_amp[1] = gy;
            s_kernel[i].b[b].grad_valid  = 1;
        }
    }

    // ── Per-beacon loop closure ──
    // Compare first LM_RX capture vs last LM_RX capture per beacon.
    // A beacon that drifted becomes down-weighted at runtime.
    int first_rx = -1, last_rx = -1;
    for (int i = 0; i < s_kernel_count; i++) {
        if (s_kernel[i].landmark_id != LM_RX) continue;
        if (first_rx < 0) first_rx = i;
        last_rx = i;
    }
    if (first_rx >= 0 && last_rx > first_rx) {
        double d_all = 0; int n_all = 0;
        for (int b = 0; b < MAX_BEACONS; b++) {
            if (s_kernel[first_rx].b[b].sample_count == 0 ||
                s_kernel[last_rx].b[b].sample_count == 0) {
                report.per_beacon_loop_closure[b] = 0;
                continue;
            }
            float range = s_amp_range_hi[b] - s_amp_range_lo[b];
            if (range < 1e-6f) { report.per_beacon_loop_closure[b] = 0; continue; }
            float drift = fabsf(s_kernel[first_rx].b[b].mean_amp
                              - s_kernel[last_rx].b[b].mean_amp) / range;
            report.per_beacon_loop_closure[b] = drift;
            // Apply drift penalty to composite beacon weight.
            s_beacon_weight[b] *= 1.0f / (1.0f + 2.0f * drift);
            // Also update the per-beacon SNR shown on the results screen
            report.per_beacon_snr[b] = s_beacon_weight[b];
            d_all += (double)drift * drift;
            n_all++;
        }
        report.loop_closure_error = n_all > 0 ? sqrtf((float)(d_all / n_all)) : 0;
    }

    // Leave-one-landmark-out response-space cross-validation.
    //
    // For each landmark i, predict its physical position as the response-
    // distance-weighted mean of all OTHER landmark positions:
    //
    //   p̂_i = Σ_{j≠i} w_ij · p_j  /  Σ_{j≠i} w_ij
    //   w_ij = 1 / (‖y_i − y_j‖_W + ε)
    //
    // where ‖·‖_W is SNR-weighted Euclidean distance in normalized
    // amplitude space.  This uses ALL held-out landmarks weighted by
    // response similarity (not just the single nearest neighbor), so it
    // tracks the real reconstruction better than the v0.4 NN proxy.
    //
    // Cost is O(N²) with a tight inner loop — trivial at N≈24 landmark
    // samples (~600 pairs).  Adopted from ChatGPT's v0.4.
    float sum_err = 0;
    int n_err = 0;
    float worst = 0;
    uint8_t worst_lm = 0;
    for (int i = 0; i < s_kernel_count; i++) {
        if (s_kernel[i].landmark_id == 0xFF) continue;  // only landmarks
        float bx = 0, by = 0, ws = 0;
        for (int j = 0; j < s_kernel_count; j++) {
            if (j == i) continue;
            float d = 0;
            int n_shared = 0;
            for (int b = 0; b < MAX_BEACONS; b++) {
                if (s_kernel[i].b[b].sample_count == 0) continue;
                if (s_kernel[j].b[b].sample_count == 0) continue;
                float range = s_amp_range_hi[b] - s_amp_range_lo[b];
                if (range < 1e-6f) continue;
                float di = (s_kernel[i].b[b].mean_amp - s_kernel[j].b[b].mean_amp) / range;
                d += s_beacon_snr[b] * sq(di);
                n_shared++;
            }
            if (n_shared == 0) continue;
            float w = 1.0f / (sqrtf(d) + 1e-3f);
            bx += w * s_kernel[j].pos[0];
            by += w * s_kernel[j].pos[1];
            ws += w;
        }
        if (ws < 1e-9f) continue;
        float pred_x = bx / ws, pred_y = by / ws;
        float e = sqrtf(sq(s_kernel[i].pos[0] - pred_x) + sq(s_kernel[i].pos[1] - pred_y));
        sum_err += e;
        n_err++;
        if (e > worst) { worst = e; worst_lm = s_kernel[i].landmark_id; }
    }
    report.cross_val_error = n_err > 0 ? sum_err / n_err : 0;
    report.worst_landmark = worst_lm;
    report.worst_landmark_error = worst;

    // (loop closure now computed above as per-beacon + aggregate)
    report.geometry_correction_mag = 0;   // (deferred — hook exists)

    // ── Fisher-lite observability ────────────────────────────────
    // At each landmark, estimate the local response Jacobian J from
    // the difference between this landmark's fingerprint and its
    // nearest neighbor in physical space.  Then compute the smaller
    // eigenvalue of J^T J as a scalar observability proxy.  Average
    // across landmarks → single "how well-conditioned is this room"
    // number.  Higher = spatial changes are measurable; lower =
    // the RF field is flat here.
    {
        double sum_obs = 0;
        int n_obs = 0;
        for (int i = 0; i < s_kernel_count; i++) {
            if (s_kernel[i].landmark_id == 0xFF) continue;
            // Find nearest neighbor in PHYSICAL space (excluding self)
            float best_d2 = 1e30f;
            int   best_j = -1;
            for (int j = 0; j < s_kernel_count; j++) {
                if (j == i) continue;
                float d2 = dist2(s_kernel[i].pos[0], s_kernel[i].pos[1],
                                 s_kernel[j].pos[0], s_kernel[j].pos[1]);
                if (d2 < best_d2) { best_d2 = d2; best_j = j; }
            }
            if (best_j < 0 || best_d2 < 1e-6f) continue;
            float dx = s_kernel[best_j].pos[0] - s_kernel[i].pos[0];
            float dy = s_kernel[best_j].pos[1] - s_kernel[i].pos[1];
            // 2×2 Fisher approx: J is n_beacons × 2; F = J^T J
            float fxx = 0, fyy = 0, fxy = 0;
            for (int b = 0; b < MAX_BEACONS; b++) {
                if (s_kernel[i].b[b].sample_count == 0) continue;
                if (s_kernel[best_j].b[b].sample_count == 0) continue;
                float range = s_amp_range_hi[b] - s_amp_range_lo[b];
                if (range < 1e-6f) continue;
                float dy_b = (s_kernel[best_j].b[b].mean_amp
                            - s_kernel[i].b[b].mean_amp) / range;
                // grad ≈ dy_b * (dx, dy) / |dx,dy|²
                float inv_norm2 = 1.0f / (dx*dx + dy*dy);
                float gx = dy_b * dx * inv_norm2;
                float gy = dy_b * dy * inv_norm2;
                float w = s_beacon_snr[b] * s_beacon_orient_reliability[b];
                fxx += w * gx * gx;
                fyy += w * gy * gy;
                fxy += w * gx * gy;
            }
            // Smaller eigenvalue of 2×2 symmetric: λ_min = trace/2 − √(disc)
            float trace = fxx + fyy;
            float disc  = 0.25f * (fxx - fyy) * (fxx - fyy) + fxy * fxy;
            float lmin  = 0.5f * trace - sqrtf(disc);
            if (lmin < 0) lmin = 0;
            sum_obs += lmin;
            n_obs++;
        }
        report.mean_observability = n_obs > 0 ? (float)(sum_obs / n_obs) : 0;
    }

    // ── RF-alias detection ───────────────────────────────────────
    // Landmark pairs that are FAR apart physically (>= 1 unit) but
    // CLOSE in observation space (< 0.15 normalized).  Stored as
    // AliasPair entries so runtime can flag tracks landing near them.
    {
        int alias_n = 0;
        const float PHYS_FAR_SQ = 1.0f;
        const float RF_CLOSE = 0.15f;
        for (int i = 0; i < s_kernel_count && alias_n < CAL_MAX_ALIAS_PAIRS; i++) {
            if (s_kernel[i].landmark_id == 0xFF) continue;
            for (int j = i + 1; j < s_kernel_count && alias_n < CAL_MAX_ALIAS_PAIRS; j++) {
                if (s_kernel[j].landmark_id == 0xFF) continue;
                float pd2 = dist2(s_kernel[i].pos[0], s_kernel[i].pos[1],
                                  s_kernel[j].pos[0], s_kernel[j].pos[1]);
                if (pd2 < PHYS_FAR_SQ) continue;
                float rf_d = 0;
                int   rf_n = 0;
                for (int b = 0; b < MAX_BEACONS; b++) {
                    if (s_kernel[i].b[b].sample_count == 0) continue;
                    if (s_kernel[j].b[b].sample_count == 0) continue;
                    float range = s_amp_range_hi[b] - s_amp_range_lo[b];
                    if (range < 1e-6f) continue;
                    float di = (s_kernel[i].b[b].mean_amp
                              - s_kernel[j].b[b].mean_amp) / range;
                    rf_d += sq(di);
                    rf_n++;
                }
                if (rf_n == 0) continue;
                float rf_dist = sqrtf(rf_d / rf_n);
                if (rf_dist < RF_CLOSE) {
                    report.alias_pairs[alias_n].lm_a = s_kernel[i].landmark_id;
                    report.alias_pairs[alias_n].lm_b = s_kernel[j].landmark_id;
                    report.alias_pairs[alias_n].rf_distance = rf_dist;
                    report.alias_pairs[alias_n].phys_distance = sqrtf(pd2);
                    alias_n++;
                }
            }
        }
        report.alias_pair_count = (uint8_t)alias_n;
    }
    // Cache in module-static so runtime doesn't have to walk the report
    s_alias_count = report.alias_pair_count;
    memcpy(s_alias_pairs, report.alias_pairs,
           s_alias_count * sizeof(AliasPair));

    // ── Geometry validation from PROBE cal stream ─────────────────
    // When the user stood at "beacon N" landmark, PROBE (on their chest)
    // should have seen beacon N's amp perturbation dominate.  If not,
    // beacon N is probably mispositioned or misidentified.  We check
    // this by looking at kernel samples at LM_BEACON_1/2/3 and comparing
    // their per-beacon mean_amp magnitudes.
    //
    // NOTE: we use the ANCHOR-derived kernel entries here since the raw
    // PROBE frames are discarded post-transit; a full check would require
    // stashing the PROBE cal stream per landmark.  For v0.5 this uses
    // ANCHOR data at the landmark, which correlates strongly with what
    // PROBE would see at the same landmark (they're the same room, same
    // beacon geometry).  The check flags gross mislabels only.
    for (int b = 0; b < MAX_BEACONS; b++) {
        // Find the kernel sample at LM_BEACON_(b+1)
        LandmarkId beacon_lm = (LandmarkId)(LM_BEACON_1 + b);
        int lm_idx = -1;
        for (int i = 0; i < s_kernel_count; i++) {
            if (s_kernel[i].landmark_id == beacon_lm) { lm_idx = i; break; }
        }
        if (lm_idx < 0) continue;
        // Find |mean_amp| of beacon b at this landmark
        float mine = fabsf(s_kernel[lm_idx].b[b].mean_amp);
        if (mine < 1e-6f) continue;
        // Find max |mean_amp| across all beacons at this landmark
        float peak = 0;
        for (int b2 = 0; b2 < MAX_BEACONS; b2++) {
            float v = fabsf(s_kernel[lm_idx].b[b2].mean_amp);
            if (v > peak) peak = v;
        }
        if (peak < 1e-6f) continue;
        // Beacon b should be at or near the peak (person standing at its
        // landmark position; that beacon is closest = perturbation is
        // strongest).  If beacon b's mean_amp is well below peak, flag it.
        if (mine / peak < GEOM_VALIDATION_MIN_FRAC) {
            report.geometry_validation_fail_mask |= (1u << b);
            Serial.printf("[scene] GEOM WARN: at LM_BEACON_%d, beacon %d "
                          "amp=%.3f is only %.0f%% of peak %.3f\n",
                          b+1, b, (double)mine,
                          100.0 * mine / peak, (double)peak);
        }
    }

    report.valid = true;
    s_cal_complete = true;
    s_cal_active = false;

    Serial.printf("[scene] cal finalize: N=%d xval=%.3f loop=%.3f "
                  "obs=%.3f aliases=%u geom_fail=0x%02X\n",
                  s_kernel_count, report.cross_val_error, report.loop_closure_error,
                  (double)report.mean_observability,
                  (unsigned)report.alias_pair_count,
                  (unsigned)report.geometry_validation_fail_mask);
}

// ═══════════════════════════════════════════════════════════════
//  v0.6 — PROBABILISTIC INVERSE SENSOR MODEL
// ═══════════════════════════════════════════════════════════════
// The scene contains K unknown persons at positions p_1..p_K, each
// with a per-target strength α_k that absorbs body-size / posture
// variation.  The observation model is:
//
//   y = Σ_k α_k · h(p_k) + b + ε,   ε ~ N(0, Σ(p_1..p_K))
//
// where:
//   y ∈ ℝ^D_OBS is the stacked per-beacon observation vector
//       [amp_0..B-1, phase_0..B-1, aoa_0..B-1]
//   h(p) is the mean response learned during cal (built from kernel
//       samples via 6-NN IDW, first-order Taylor with grad_amp when
//       available)
//   Σ(p) is the diagonal per-channel covariance learned during cal
//       (per-landmark std_amp/std_phase/std_aoa IDW-interpolated;
//       aspect_var from rotation experiment adds to amp variance —
//       cells near ROTATE landmarks with wide aspect swings have
//       genuinely wider Σ_amp because a person there can produce
//       a wider range of readings depending on body aspect, and
//       we don't want to falsely reject observations that are still
//       "body-like" just because they're not near the aspect mean)
//   b is the adaptive background (per-beacon amp offset).
//
// Inference maximizes log-likelihood jointly over all K targets:
//
//   L(K, {p_k}, {α_k}) = -0.5 (y' - ŷ)ᵀ Σ⁻¹ (y' - ŷ) - 0.5 log|Σ|
//
// where y' = y - b - phase/aoa-baseline-of-empty-room and
// ŷ = Σ_k α_k · h(p_k).  Gauss-Newton on the joint state solves this.
// Number of targets K is chosen by comparing log-likelihood gain
// against MIN_LOG_LIK_GAIN_TO_ADD; targets whose α → 0 are killed.
//
// The 24×24 occupancy field is a RENDERING artifact — each tracked
// target rasterizes its Gaussian bump into the field for display.
// The actual state is the track list (p_k, α_k, cov_k).

// ── Measurement model: h(p) and Σ(p) ────────────────────────────
// Both computed from the same 6-NN sample-count-weighted IDW pass
// so we only walk the kernel once.

struct MeasModel {
    float mean[D_OBS];      // h(p)
    float var[D_OBS];       // diag Σ(p)
    uint8_t valid[D_OBS];   // per-channel validity (AoA channels may be invalid)
};

// Channel layout helpers
static inline int CH_AMP(int b)   { return b; }
static inline int CH_PHASE(int b) { return MAX_BEACONS + b; }
static inline int CH_AOA(int b)   { return 2 * MAX_BEACONS + b; }

static inline float idw_w_v6(float dist_sq, uint16_t sample_count) {
    float w_dist  = 1.0f / (sqrtf(dist_sq) + 1e-3f);
    float w_count = sqrtf((float)sample_count / (float)IDW_COUNT_FLOOR);
    if (w_count > 1.0f) w_count = 1.0f;
    return w_dist * w_count;
}

static void meas_model(float px, float py, MeasModel &m) {
    memset(&m, 0, sizeof(m));
    if (s_kernel_count == 0) return;
    // 6-NN
    const int K = 6;
    float best_d[K];
    int   best_i[K];
    for (int k = 0; k < K; k++) { best_d[k] = 1e30f; best_i[k] = -1; }
    for (int i = 0; i < s_kernel_count; i++) {
        float d = dist2(px, py, s_kernel[i].pos[0], s_kernel[i].pos[1]);
        for (int k = 0; k < K; k++) {
            if (d < best_d[k]) {
                for (int j = K - 1; j > k; j--) { best_d[j] = best_d[j-1]; best_i[j] = best_i[j-1]; }
                best_d[k] = d; best_i[k] = i;
                break;
            }
        }
    }
    // Per-beacon reduction, three channels each
    for (int b = 0; b < MAX_BEACONS; b++) {
        float acc_amp = 0, acc_amp_var = 0, acc_asp_var = 0;
        float acc_ph_i = 0, acc_ph_q = 0, acc_ph_var = 0;
        float acc_ao_i = 0, acc_ao_q = 0, acc_ao_var = 0;
        float w_amp = 0, w_aoa = 0;
        for (int k = 0; k < K; k++) {
            if (best_i[k] < 0) continue;
            const KernelSample &ks = s_kernel[best_i[k]];
            if (ks.b[b].sample_count == 0) continue;
            float w = idw_w_v6(best_d[k], ks.b[b].sample_count);
            // Amp with first-order Taylor correction from transit-fit grad
            float amp_at_p = ks.b[b].mean_amp;
            if (ks.b[b].grad_valid) {
                float dx = px - ks.pos[0];
                float dy = py - ks.pos[1];
                amp_at_p += ks.b[b].grad_amp[0] * dx + ks.b[b].grad_amp[1] * dy;
            }
            acc_amp     += w * amp_at_p;
            acc_amp_var += w * sq(ks.b[b].std_amp);
            acc_asp_var += w * ks.b[b].aspect_var;
            acc_ph_i    += w * cosf(ks.b[b].mean_phase);
            acc_ph_q    += w * sinf(ks.b[b].mean_phase);
            acc_ph_var  += w * sq(ks.b[b].std_phase);
            w_amp += w;
            if (ks.b[b].saw_aoa > 0) {
                acc_ao_i   += w * cosf(ks.b[b].mean_aoa_dev);
                acc_ao_q   += w * sinf(ks.b[b].mean_aoa_dev);
                acc_ao_var += w * sq(ks.b[b].std_aoa);
                w_aoa += w;
            }
        }
        if (w_amp > 1e-9f) {
            m.mean[CH_AMP(b)]   = acc_amp / w_amp;
            m.mean[CH_PHASE(b)] = atan2f(acc_ph_q, acc_ph_i);
            // Amp variance = base sample noise + aspect variance from
            // rotation cal.  aspect_var enters HERE, in the covariance,
            // as widening for "a body at this cell can produce a wider
            // range of readings across body aspect".  Cells far from
            // ROTATE landmarks get aspect_var ≈ 0 → tight sigma; cells
            // near ROTATE landmarks that showed wide aspect swings get
            // proportionally wider sigma → the likelihood correctly
            // accepts more of the body-envelope as "person here".
            float sig_amp_sq = acc_amp_var / w_amp + acc_asp_var / w_amp;
            m.var[CH_AMP(b)]   = fmaxf(SIGMA_AMP_FLOOR * SIGMA_AMP_FLOOR,
                                       sig_amp_sq);
            m.var[CH_PHASE(b)] = fmaxf(SIGMA_PHASE_FLOOR * SIGMA_PHASE_FLOOR,
                                       acc_ph_var / w_amp);
            m.valid[CH_AMP(b)]   = 1;
            m.valid[CH_PHASE(b)] = 1;
        }
        if (w_aoa > 1e-9f) {
            m.mean[CH_AOA(b)] = atan2f(acc_ao_q, acc_ao_i);
            m.var[CH_AOA(b)]  = fmaxf(SIGMA_AOA_FLOOR * SIGMA_AOA_FLOOR,
                                      acc_ao_var / w_aoa);
            m.valid[CH_AOA(b)] = 1;
        }
    }
}

// ── Observation vector construction from a FrameObservation ─────
struct ObsVec {
    float y[D_OBS];
    uint8_t valid[D_OBS];    // per-channel validity for THIS frame
    float aoa_conf[MAX_BEACONS];
};
static void build_obs_vec(const FrameObservation &obs, ObsVec &o) {
    memset(&o, 0, sizeof(o));
    for (int b = 0; b < MAX_BEACONS; b++) {
        if (!obs.beacon[b].fresh) continue;
        // Adaptive background subtracted from amp — background is only
        // updated when the scene is quiet, so this reflects the
        // human-induced perturbation, not the empty-room residual drift.
        o.y[CH_AMP(b)]   = obs.beacon[b].amp_perturbation - s_bg_amp[b];
        o.y[CH_PHASE(b)] = obs.beacon[b].phase_perturbation;
        o.valid[CH_AMP(b)]   = 1;
        o.valid[CH_PHASE(b)] = 1;
        if (obs.beacon[b].have_aoa && obs.beacon[b].aoa_conf >= AOA_MIN_CONF) {
            o.y[CH_AOA(b)] = obs.beacon[b].aoa_rad;
            o.valid[CH_AOA(b)] = 1;
        }
        o.aoa_conf[b] = obs.beacon[b].aoa_conf;
    }
}

// ── Multi-target forward model: ŷ = Σ_k α_k · h(p_k) ────────────
//
// Amplitude channels are additive scalars.  Phase and AoA channels
// are circular; a linear superposition of them is not meaningful in
// the phase domain (two waves with different phases don't add
// arithmetically — they add as complex exponentials).  For those,
// we compute a weighted circular-mean of the constituent means with
// weights α_k, then that becomes the prediction.  This is a first-
// order model; refinement to complex-baseband superposition is
// deferred.
struct TargetState {
    float pos[2];
    float alpha;
    // Cache: last-computed measurement model & its jacobian for this state
    MeasModel m;
    float dh_dx[D_OBS];      // ∂h/∂x at p (numerical)
    float dh_dy[D_OBS];      // ∂h/∂y at p (numerical)
    uint8_t dh_valid[D_OBS];
};
static void refresh_target_cache(TargetState &t) {
    meas_model(t.pos[0], t.pos[1], t.m);
    // Numerical Jacobian via central differences.  Cost = 4 extra
    // meas_model calls per target per GN iter, all cheap.
    float step = ((2.0f * SCENE_EXTENT) / (float)FIELD_DIM) * JACOBIAN_STEP_FRAC;
    MeasModel mxp, mxm, myp, mym;
    meas_model(t.pos[0] + step, t.pos[1], mxp);
    meas_model(t.pos[0] - step, t.pos[1], mxm);
    meas_model(t.pos[0], t.pos[1] + step, myp);
    meas_model(t.pos[0], t.pos[1] - step, mym);
    float inv2s = 1.0f / (2.0f * step);
    for (int d = 0; d < D_OBS; d++) {
        bool ok = mxp.valid[d] && mxm.valid[d] && myp.valid[d] && mym.valid[d];
        t.dh_valid[d] = ok ? 1 : 0;
        if (!ok) { t.dh_dx[d] = 0; t.dh_dy[d] = 0; continue; }
        // For circular channels (phase, aoa), wrap the difference
        int b = d % MAX_BEACONS;
        bool circular = (d >= CH_PHASE(0));   // phase and aoa channels
        (void)b;
        if (circular) {
            t.dh_dx[d] = wrap_pi(mxp.mean[d] - mxm.mean[d]) * inv2s;
            t.dh_dy[d] = wrap_pi(myp.mean[d] - mym.mean[d]) * inv2s;
        } else {
            t.dh_dx[d] = (mxp.mean[d] - mxm.mean[d]) * inv2s;
            t.dh_dy[d] = (myp.mean[d] - mym.mean[d]) * inv2s;
        }
    }
}

// Predict combined ŷ from K targets.  For amp: sum α_k · h_k(d).
// For circular channels: α-weighted circular mean of h_k(d).
// Combined variance: sum of per-target variances (targets add
// independent uncertainty).  Weighted by α_k² for amp (variance
// scales with mean²), unweighted for angular.
static void predict_multi(const TargetState *targets, int K,
                          float *y_pred, float *var_out, uint8_t *valid_out) {
    for (int d = 0; d < D_OBS; d++) {
        y_pred[d] = 0; var_out[d] = 0; valid_out[d] = 0;
    }
    if (K == 0) {
        for (int d = 0; d < D_OBS; d++) { valid_out[d] = 1; var_out[d] = SIGMA_AMP_FLOOR * SIGMA_AMP_FLOOR; }
        return;
    }
    // Amp: additive
    for (int b = 0; b < MAX_BEACONS; b++) {
        int d = CH_AMP(b);
        float sum = 0, var = 0;
        int nv = 0;
        for (int k = 0; k < K; k++) {
            if (!targets[k].m.valid[d]) continue;
            sum += targets[k].alpha * targets[k].m.mean[d];
            var += targets[k].alpha * targets[k].alpha * targets[k].m.var[d];
            nv++;
        }
        if (nv > 0) { y_pred[d] = sum; var_out[d] = var; valid_out[d] = 1; }
    }
    // Phase & AoA: α-weighted circular mean, variance averaged
    for (int ch_type = 0; ch_type < 2; ch_type++) {
        int base = (ch_type == 0) ? CH_PHASE(0) : CH_AOA(0);
        for (int b = 0; b < MAX_BEACONS; b++) {
            int d = base + b;
            float acc_i = 0, acc_q = 0, var = 0, w = 0;
            int nv = 0;
            for (int k = 0; k < K; k++) {
                if (!targets[k].m.valid[d]) continue;
                float wk = targets[k].alpha;
                acc_i += wk * cosf(targets[k].m.mean[d]);
                acc_q += wk * sinf(targets[k].m.mean[d]);
                var   += targets[k].m.var[d];
                w += wk;
                nv++;
            }
            if (nv > 0 && w > 1e-9f) {
                y_pred[d]   = atan2f(acc_q, acc_i);
                var_out[d]  = var / nv;
                valid_out[d] = 1;
            }
        }
    }
}

// Compute per-channel residual, wrapping circular channels
static inline float channel_residual(int d, float y_obs, float y_pred) {
    return (d >= CH_PHASE(0)) ? wrap_pi(y_obs - y_pred) : (y_obs - y_pred);
}

// Log-likelihood of observation given K targets.  Not normalized; the
// -0.5 log|Σ| term is included so BIC-style comparison of different
// K values is meaningful (widening Σ hurts likelihood too).
static float log_likelihood_multi(const ObsVec &obs,
                                  const TargetState *targets, int K,
                                  float *out_rms_residual /* nullable */) {
    float y_pred[D_OBS], var_pred[D_OBS];
    uint8_t val_pred[D_OBS];
    predict_multi(targets, K, y_pred, var_pred, val_pred);
    float ll = 0;
    float sse = 0; int ncontrib = 0;
    for (int d = 0; d < D_OBS; d++) {
        if (!obs.valid[d] || !val_pred[d]) continue;
        float r = channel_residual(d, obs.y[d], y_pred[d]);
        float v = fmaxf(1e-6f, var_pred[d]);
        // Per-beacon weight (SNR × orient × loop-closure) modulates
        // how much this channel counts.
        int b = d % MAX_BEACONS;
        float w = s_beacon_weight[b];
        // Huber-attenuated squared residual over variance
        float z = r / sqrtf(v);
        ll += -w * huber(z);
        ll += -0.5f * w * logf(v);
        sse += r * r;
        ncontrib++;
    }
    if (out_rms_residual) *out_rms_residual = ncontrib > 0 ? sqrtf(sse / ncontrib) : 0;
    return ll;
}

// ── Gauss-Newton joint MAP over all targets ─────────────────────
// State vector layout: [x_0, y_0, α_0, x_1, y_1, α_1, ...] of length
// 3K.  Returns final log-likelihood; targets[] mutated in place.
// Trust-region: step halving until cost improves or step < GN_MIN_STEP.
static float gn_solve_joint(const ObsVec &obs, TargetState *targets, int K) {
    if (K == 0) return log_likelihood_multi(obs, targets, 0, nullptr);
    // Warm the model caches
    for (int k = 0; k < K; k++) refresh_target_cache(targets[k]);
    float ll_curr = log_likelihood_multi(obs, targets, K, nullptr);

    for (int iter = 0; iter < GN_MAX_ITERS; iter++) {
        int N = DIMS_PER_TARGET * K;
        // Build J^T W J and J^T W r where W = diag(1/var_d · beacon_weight_b)
        static float H[MAX_STATE_DIM * MAX_STATE_DIM];
        static float g[MAX_STATE_DIM];
        for (int i = 0; i < N * N; i++) H[i] = 0;
        for (int i = 0; i < N; i++) g[i] = 0;
        // Combined prediction & variance (target caches already valid)
        float y_pred[D_OBS], var_pred[D_OBS];
        uint8_t val_pred[D_OBS];
        predict_multi(targets, K, y_pred, var_pred, val_pred);
        // Per-channel residual & weight
        for (int d = 0; d < D_OBS; d++) {
            if (!obs.valid[d] || !val_pred[d]) continue;
            float r = channel_residual(d, obs.y[d], y_pred[d]);
            float v = fmaxf(1e-6f, var_pred[d]);
            int b = d % MAX_BEACONS;
            float w = s_beacon_weight[b] / v;
            // Build per-target Jacobian ROW for this channel.  The
            // full Jacobian J[d, :] has three entries per target k:
            //   d(ŷ)/dx_k, d(ŷ)/dy_k, d(ŷ)/dα_k
            // For amp (additive): d(ŷ_d)/dp_k = α_k · dh_k/dp; d(ŷ_d)/dα_k = h_k(d)
            // For circular channels (α-weighted circular mean), we
            // approximate the Jacobian by treating each target's
            // contribution to the mean angle as linear in its own
            // params with the same shape as amp — this is a first-
            // order approximation that's exact when only one target
            // has a valid prediction for this channel (common case).
            float row[MAX_STATE_DIM];
            bool circular = (d >= CH_PHASE(0));
            (void)circular;
            for (int k = 0; k < K; k++) {
                if (!targets[k].m.valid[d]) { row[3*k+0] = row[3*k+1] = row[3*k+2] = 0; continue; }
                if (!targets[k].dh_valid[d]) { row[3*k+0] = row[3*k+1] = row[3*k+2] = 0; continue; }
                float d_dx = targets[k].alpha * targets[k].dh_dx[d];
                float d_dy = targets[k].alpha * targets[k].dh_dy[d];
                float d_da = targets[k].m.mean[d];
                row[3*k+0] = d_dx;
                row[3*k+1] = d_dy;
                row[3*k+2] = d_da;
            }
            // Accumulate H += w · row · rowᵀ; g += w · row · r
            for (int i = 0; i < N; i++) {
                g[i] += w * row[i] * r;
                for (int j = i; j < N; j++) {
                    H[i*N + j] += w * row[i] * row[j];
                    if (i != j) H[j*N + i] = H[i*N + j];   // symmetry
                }
            }
        }
        // Levenberg-Marquardt-ish damping on diagonal
        for (int i = 0; i < N; i++) H[i*N + i] += 1e-3f;
        // Solve H · δ = g
        static float delta[MAX_STATE_DIM];
        if (!cholesky_solve(H, g, delta, N)) break;
        // Clamp each parameter's step to its own bound
        for (int k = 0; k < K; k++) {
            float m = fmaxf(fabsf(delta[3*k+0]), fabsf(delta[3*k+1]));
            if (m > GN_MAX_STEP_POS) {
                float s = GN_MAX_STEP_POS / m;
                delta[3*k+0] *= s;
                delta[3*k+1] *= s;
            }
            if (fabsf(delta[3*k+2]) > GN_MAX_STEP_ALPHA) {
                delta[3*k+2] = (delta[3*k+2] > 0 ? 1 : -1) * GN_MAX_STEP_ALPHA;
            }
        }
        // Trust-region step: halve until accepted or too small
        float step_scale = 1.0f;
        TargetState trial[SCENE_MAX_TARGETS];
        for (int hs = 0; hs < 6; hs++) {
            for (int k = 0; k < K; k++) {
                trial[k] = targets[k];
                trial[k].pos[0]  = clampf(targets[k].pos[0]  + step_scale * delta[3*k+0],
                                          -SCENE_EXTENT, SCENE_EXTENT);
                trial[k].pos[1]  = clampf(targets[k].pos[1]  + step_scale * delta[3*k+1],
                                          -SCENE_EXTENT, SCENE_EXTENT);
                trial[k].alpha   = clampf(targets[k].alpha   + step_scale * delta[3*k+2],
                                          ALPHA_MIN, ALPHA_MAX);
                refresh_target_cache(trial[k]);
            }
            float ll_trial = log_likelihood_multi(obs, trial, K, nullptr);
            if (ll_trial > ll_curr + 1e-4f) {
                // Accept
                for (int k = 0; k < K; k++) targets[k] = trial[k];
                ll_curr = ll_trial;
                break;
            }
            step_scale *= 0.5f;
            if (step_scale < GN_MIN_STEP) break;
        }
        if (step_scale < GN_MIN_STEP) break;
    }
    return ll_curr;
}

// ── Posterior covariance from Fisher info at MAP ────────────────
// After GN converges, the position covariance is the (x,y) block of
// the inverse of J^T W J for that target's 2 positional params only.
// alpha_k is treated as a nuisance parameter (marginalized approximately
// by just extracting the 2×2 spatial block).
static void posterior_cov(const ObsVec &obs, const TargetState *targets, int K,
                          int k, float *cxx, float *cyy, float *cxy) {
    // Build 2×2 Fisher for target k's position params only.
    float fxx = 0, fyy = 0, fxy = 0;
    for (int d = 0; d < D_OBS; d++) {
        if (!obs.valid[d] || !targets[k].m.valid[d] || !targets[k].dh_valid[d]) continue;
        float v = fmaxf(1e-6f, targets[k].m.var[d]);
        int b = d % MAX_BEACONS;
        float w = s_beacon_weight[b] / v;
        float jx = targets[k].alpha * targets[k].dh_dx[d];
        float jy = targets[k].alpha * targets[k].dh_dy[d];
        fxx += w * jx * jx;
        fyy += w * jy * jy;
        fxy += w * jx * jy;
    }
    float det = fxx * fyy - fxy * fxy;
    if (det < 1e-9f) {
        *cxx = *cyy = POST_COV_FLOOR;
        *cxy = 0;
        return;
    }
    *cxx = fmaxf(POST_COV_FLOOR,  fyy / det);
    *cyy = fmaxf(POST_COV_FLOOR,  fxx / det);
    *cxy = -fxy / det;
    // Alias-region inflation — if this target is near a cal-report
    // alias landmark, inflate the covariance floor to reflect that
    // the sensor genuinely cannot pin down which of two aliased
    // locations produced this observation.
    for (int a = 0; a < s_alias_count; a++) {
        float lm_a_x, lm_a_y, lm_b_x, lm_b_y;
        scene_landmark_pos((LandmarkId)s_alias_pairs[a].lm_a, &lm_a_x, &lm_a_y);
        scene_landmark_pos((LandmarkId)s_alias_pairs[a].lm_b, &lm_b_x, &lm_b_y);
        float d_a = dist2(targets[k].pos[0], targets[k].pos[1], lm_a_x, lm_a_y);
        float d_b = dist2(targets[k].pos[0], targets[k].pos[1], lm_b_x, lm_b_y);
        if (d_a < ALIAS_PROXIMITY * ALIAS_PROXIMITY ||
            d_b < ALIAS_PROXIMITY * ALIAS_PROXIMITY) {
            *cxx = fmaxf(*cxx, POST_COV_FLOOR * POST_COV_ALIAS_INFLATE);
            *cyy = fmaxf(*cyy, POST_COV_FLOOR * POST_COV_ALIAS_INFLATE);
            break;
        }
    }
}

// ── Track ↔ target-state conversion ─────────────────────────────
static void track_to_target(const TargetTrack &tr, TargetState &t) {
    t.pos[0] = tr.pos[0];
    t.pos[1] = tr.pos[1];
    t.alpha  = ALPHA_INIT;
}

// ── Birth search: coarse residual scan for a K+1'th target ──────
// Given fitted targets, subtract their predicted amp contribution
// from the observation and search the field for the position whose
// h(p) best explains the remaining amp residual.
static bool birth_search(const ObsVec &obs, const TargetState *targets, int K,
                          float *out_px, float *out_py, float *out_score) {
    float y_pred[D_OBS], var_pred[D_OBS];
    uint8_t val_pred[D_OBS];
    predict_multi(targets, K, y_pred, var_pred, val_pred);
    // Amp-only residual
    float amp_resid[MAX_BEACONS];
    uint8_t amp_valid[MAX_BEACONS];
    for (int b = 0; b < MAX_BEACONS; b++) {
        int d = CH_AMP(b);
        amp_valid[b] = obs.valid[d];
        amp_resid[b] = amp_valid[b] ? (obs.y[d] - (val_pred[d] ? y_pred[d] : 0)) : 0;
    }
    float best_score = 0;
    float best_px = 0, best_py = 0;
    const float cell_step = (2.0f * SCENE_EXTENT) / (float)FIELD_DIM;
    const float excl_sq = sq(PEAK_EXCLUSION_CELLS * cell_step);
    for (int gy = 0; gy < FIELD_DIM; gy++) {
        for (int gx = 0; gx < FIELD_DIM; gx++) {
            float px, py;
            cell_to_pos(gx, gy, &px, &py);
            // Exclude cells near already-fitted targets
            bool exc = false;
            for (int k = 0; k < K; k++) {
                if (dist2(px, py, targets[k].pos[0], targets[k].pos[1]) < excl_sq) { exc = true; break; }
            }
            if (exc) continue;
            MeasModel m; meas_model(px, py, m);
            float num = 0, den = 0;
            for (int b = 0; b < MAX_BEACONS; b++) {
                if (!amp_valid[b] || !m.valid[CH_AMP(b)]) continue;
                float pn = m.mean[CH_AMP(b)];
                float sig = sqrtf(fmaxf(1e-6f, m.var[CH_AMP(b)]));
                float w = s_beacon_weight[b] / (sig * sig);
                num += w * amp_resid[b] * pn;
                den += w * pn * pn;
            }
            if (den < 1e-9f) continue;
            float alpha_hat = num / den;
            if (alpha_hat < ALPHA_MIN) continue;
            // Score proxy: how much variance-normalized amp energy this pick explains
            float score = num * alpha_hat;
            if (score > best_score) {
                best_score = score;
                best_px = px; best_py = py;
            }
        }
    }
    *out_px = best_px;
    *out_py = best_py;
    *out_score = best_score;
    return best_score > BIRTH_SEARCH_MIN_SCORE;
}

// ── Rasterize track posteriors into the display field ───────────
// Each track contributes a normalized Gaussian bump; the field is
// the sum.  Purely for rendering — the actual state is the tracks.
static void rasterize_field() {
    memset(&s_field, 0, sizeof(s_field));
    for (int t = 0; t < TRACK_MAX; t++) {
        if (!s_tracks[t].active) continue;
        float tx = s_tracks[t].pos[0];
        float ty = s_tracks[t].pos[1];
        float cxx = fmaxf(POST_COV_FLOOR, s_tracks[t].cov_xx);
        float cyy = fmaxf(POST_COV_FLOOR, s_tracks[t].cov_yy);
        float cxy = s_tracks[t].cov_xy;
        float det = cxx * cyy - cxy * cxy;
        if (det < 1e-9f) det = 1e-9f;
        float ixx =  cyy / det;
        float iyy =  cxx / det;
        float ixy = -cxy / det;
        float amp = s_tracks[t].confidence * FIELD_RASTER_GAIN;
        for (int gy = 0; gy < FIELD_DIM; gy++) {
            for (int gx = 0; gx < FIELD_DIM; gx++) {
                float px, py;
                cell_to_pos(gx, gy, &px, &py);
                float dx = px - tx, dy = py - ty;
                float m = dx*dx*ixx + 2*dx*dy*ixy + dy*dy*iyy;
                if (m > 8.0f) continue;   // beyond 2.8σ, skip
                float v = amp * expf(-0.5f * m);
                if (v > s_field.cell[gx][gy]) s_field.cell[gx][gy] = v;
                if (v > s_field.cell_max) s_field.cell_max = v;
            }
        }
    }
    s_field.valid = true;
    s_field.last_update_ms = millis();
}

// ── The main scene update ───────────────────────────────────────
FrameObservation g_last_obs = {};

void scene_update() {
    if (!s_cal_complete) return;
    uint32_t now = millis();
    uint32_t dt_ms = s_last_scene_update_ms == 0 ? SCENE_UPDATE_MS
                                                 : (now - s_last_scene_update_ms);
    if (s_last_scene_update_ms != 0 && dt_ms < SCENE_UPDATE_MS) return;
    s_last_scene_update_ms = now;
    float dt_s = fmaxf(1e-3f, (float)dt_ms / 1000.0f);

    // ── 1) Predict active tracks forward (motion + covariance growth) ──
    int active_count = 0;
    for (int t = 0; t < TRACK_MAX; t++) {
        if (!s_tracks[t].active) continue;
        s_tracks[t].pos[0] += s_tracks[t].vel[0] * dt_s;
        s_tracks[t].pos[1] += s_tracks[t].vel[1] * dt_s;
        s_tracks[t].pos[0] = clampf(s_tracks[t].pos[0], -SCENE_EXTENT, SCENE_EXTENT);
        s_tracks[t].pos[1] = clampf(s_tracks[t].pos[1], -SCENE_EXTENT, SCENE_EXTENT);
        float q = TRACK_PROCESS_NOISE * dt_s * dt_s;
        s_tracks[t].cov_xx += q;
        s_tracks[t].cov_yy += q;
        active_count++;
    }

    // ── 2) Build observation vector ──
    ObsVec obs;
    build_obs_vec(g_last_obs, obs);

    // ── 3) Compose current target set from active tracks ──
    TargetState targets[SCENE_MAX_TARGETS];
    int trk_idx[SCENE_MAX_TARGETS];
    int K = 0;
    for (int t = 0; t < TRACK_MAX && K < SCENE_MAX_TARGETS; t++) {
        if (!s_tracks[t].active) continue;
        track_to_target(s_tracks[t], targets[K]);
        trk_idx[K] = t;
        K++;
    }

    // ── 4) Joint MAP with current K ──
    float ll_cur = gn_solve_joint(obs, targets, K);

    // ── 5) Try birth: does adding a target improve joint likelihood
    //       by at least MIN_LOG_LIK_GAIN_TO_ADD? ──
    while (K < SCENE_MAX_TARGETS) {
        float bx, by, bscore;
        if (!birth_search(obs, targets, K, &bx, &by, &bscore)) break;
        TargetState trial[SCENE_MAX_TARGETS];
        for (int k = 0; k < K; k++) trial[k] = targets[k];
        trial[K].pos[0] = bx;
        trial[K].pos[1] = by;
        trial[K].alpha  = ALPHA_INIT;
        int K_new = K + 1;
        float ll_new = gn_solve_joint(obs, trial, K_new);
        if (ll_new - ll_cur > MIN_LOG_LIK_GAIN_TO_ADD) {
            for (int k = 0; k < K_new; k++) targets[k] = trial[k];
            trk_idx[K] = -1;   // will be assigned to a new track below
            K = K_new;
            ll_cur = ll_new;
        } else {
            break;
        }
    }

    // ── 6) Death: remove targets whose alpha dropped below floor ──
    // Prune in place; if K changes, re-solve on the pruned set.
    bool pruned = false;
    for (int k = K - 1; k >= 0; k--) {
        if (targets[k].alpha < MIN_ALPHA_TO_KEEP) {
            for (int j = k; j < K - 1; j++) {
                targets[j] = targets[j+1];
                trk_idx[j] = trk_idx[j+1];
            }
            K--;
            pruned = true;
        }
    }
    if (pruned && K > 0) ll_cur = gn_solve_joint(obs, targets, K);

    // ── 7) Post-MAP: compute posterior covariance per target,
    //       reconcile with track list ──
    bool track_updated[TRACK_MAX] = {};
    for (int k = 0; k < K; k++) {
        int tidx = trk_idx[k];
        if (tidx < 0 || !s_tracks[tidx].active) {
            // Newly-born target: allocate a track slot
            tidx = -1;
            for (int t = 0; t < TRACK_MAX; t++) {
                if (!s_tracks[t].active) { tidx = t; break; }
            }
            if (tidx < 0) continue;   // all slots full
            TargetTrack &nt = s_tracks[tidx];
            nt = {};
            nt.active = true;
            nt.id = s_next_track_id++;
            if (s_next_track_id == 0) s_next_track_id = 1;
        }
        TargetTrack &tr = s_tracks[tidx];
        // Velocity update: raw position delta / dt, EMA'd
        float new_vx = (targets[k].pos[0] - tr.pos[0]) / dt_s;
        float new_vy = (targets[k].pos[1] - tr.pos[1]) / dt_s;
        tr.vel[0] = 0.7f * tr.vel[0] + 0.3f * new_vx;
        tr.vel[1] = 0.7f * tr.vel[1] + 0.3f * new_vy;
        tr.pos[0] = targets[k].pos[0];
        tr.pos[1] = targets[k].pos[1];
        // Posterior covariance from Fisher info at MAP
        posterior_cov(obs, targets, K, k, &tr.cov_xx, &tr.cov_yy, &tr.cov_xy);
        // Confidence tracks alpha (normalized)
        tr.confidence = clampf(targets[k].alpha / 1.5f, 0, 1);
        tr.age_frames++;
        tr.missed_frames = 0;
        // Alias flag
        tr.ambiguity_flag = 0;
        for (int a = 0; a < s_alias_count; a++) {
            float ax, ay, bx, by;
            scene_landmark_pos((LandmarkId)s_alias_pairs[a].lm_a, &ax, &ay);
            scene_landmark_pos((LandmarkId)s_alias_pairs[a].lm_b, &bx, &by);
            if (dist2(tr.pos[0], tr.pos[1], ax, ay) < sq(ALIAS_PROXIMITY) ||
                dist2(tr.pos[0], tr.pos[1], bx, by) < sq(ALIAS_PROXIMITY)) {
                tr.ambiguity_flag = 1;
                tr.confidence *= ALIAS_CONF_MULT;
                break;
            }
        }
        // Trail
        tr.trail[tr.trail_head].pos[0] = tr.pos[0];
        tr.trail[tr.trail_head].pos[1] = tr.pos[1];
        tr.trail[tr.trail_head].conf   = tr.confidence;
        tr.trail[tr.trail_head].t_ms   = now;
        tr.trail_head = (tr.trail_head + 1) % TRACK_TRAIL_LEN;
        if (tr.trail_count < TRACK_TRAIL_LEN) tr.trail_count++;
        track_updated[tidx] = true;
    }

    // ── 8) Decay tracks that weren't updated (target death didn't
    //       trigger their slot cleanup because we prune from the
    //       target list, not the track list) ──
    for (int t = 0; t < TRACK_MAX; t++) {
        if (!s_tracks[t].active || track_updated[t]) continue;
        s_tracks[t].missed_frames++;
        s_tracks[t].confidence *= 0.85f;
        if (s_tracks[t].missed_frames > TRACK_MISS_DECAY
            || s_tracks[t].confidence < 0.02f) {
            s_tracks[t].active = false;
        }
    }

    // ── 9) Novelty = residual RMS from best-fit model ──
    float rms = 0;
    log_likelihood_multi(obs, targets, K, &rms);
    s_novelty = rms;

    // ── 10) Rasterize track posteriors into field for display ──
    rasterize_field();

    // ── 11) Adaptive background — three-way gate (unchanged from v0.5) ──
    bool scene_quiet = (K == 0) && (s_novelty < BACKGROUND_QUIET);
    if (scene_quiet) {
        for (int b = 0; b < MAX_BEACONS; b++) {
            if (!g_last_obs.beacon[b].fresh) continue;
            float raw = g_last_obs.beacon[b].amp_perturbation;
            if (fabsf(raw - s_bg_amp[b]) < BACKGROUND_GATE) {
                s_bg_amp[b] = (1.0f - BACKGROUND_ALPHA) * s_bg_amp[b]
                            + BACKGROUND_ALPHA * raw;
            }
        }
    }
}

// ═══════════════════════════════════════════════════════════════
//  READOUTS
// ═══════════════════════════════════════════════════════════════
const OccupancyField *scene_get_field() { return &s_field; }

const TargetTrack *scene_get_track(int idx) {
    if (idx < 0 || idx >= TRACK_MAX) return nullptr;
    return &s_tracks[idx];
}

int scene_active_track_count() {
    int n = 0;
    for (int t = 0; t < TRACK_MAX; t++) if (s_tracks[t].active) n++;
    return n;
}

int scene_kernel_sample_count() { return s_kernel_count; }
const KernelSample *scene_kernel_sample(int idx) {
    if (idx < 0 || idx >= s_kernel_count) return nullptr;
    return &s_kernel[idx];
}
float scene_novelty_score() { return s_novelty; }


