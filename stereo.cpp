// ═══════════════════════════════════════════════════════════════
//  stereo.cpp
// ═══════════════════════════════════════════════════════════════
#include "stereo.h"
#include "peer.h"
#include <math.h>
#include <string.h>

// ── helpers ────────────────────────────────────────────────────
static inline float wrap_pi(float x) {
    while (x >  (float)M_PI) x -= 2.0f * (float)M_PI;
    while (x < -(float)M_PI) x += 2.0f * (float)M_PI;
    return x;
}

// Unwrap a phase sequence in place, over the selected subcarrier set only.
// Simple 1-D unwrap: each successive sample is adjusted so its diff from
// the prior is within (-π, π].
static void unwrap_selected(float *seq_over_sel) {
    for (int i = 1; i < CSI_SEL_COUNT; i++) {
        float d = seq_over_sel[i] - seq_over_sel[i-1];
        while (d >   (float)M_PI) { seq_over_sel[i] -= 2.0f * (float)M_PI; d = seq_over_sel[i] - seq_over_sel[i-1]; }
        while (d <= -(float)M_PI) { seq_over_sel[i] += 2.0f * (float)M_PI; d = seq_over_sel[i] - seq_over_sel[i-1]; }
    }
}

// Extract phase at the selected subcarriers, unwrapped.
static int gather_selected_phase(const float *phase64, float *out_sel, int *out_sc_x) {
    int n = 0;
    for (int i = 0; i < CSI_SEL_COUNT; i++) {
        int sc = CSI_SEL_SC[i];
        if (sc < 0 || sc >= CSI_NUM_SUBCARRIERS) continue;
        float v = phase64[sc];
        if (!isfinite(v)) continue;
        out_sel[n]  = v;
        out_sc_x[n] = sc;
        n++;
    }
    if (n >= 2) unwrap_selected(out_sel);
    return n;
}

// ── public: line fit ───────────────────────────────────────────
int stereo_fit_line(const float *phase64, float &slope, float &intercept) {
    float ys[CSI_SEL_COUNT];
    int   xs[CSI_SEL_COUNT];
    int n = gather_selected_phase(phase64, ys, xs);
    if (n < 3) { slope = 0; intercept = 0; return n; }

    // Ordinary least squares on (x, y) with x = subcarrier index.
    double sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (int i = 0; i < n; i++) {
        double x = (double)xs[i];
        double y = (double)ys[i];
        sx += x; sy += y; sxx += x*x; sxy += x*y;
    }
    double denom = (double)n * sxx - sx * sx;
    if (fabs(denom) < 1e-9) { slope = 0; intercept = (float)(sy / n); return n; }
    double m = ((double)n * sxy - sx * sy) / denom;
    double c = (sy - m * sx) / (double)n;
    slope = (float)m;
    intercept = (float)c;
    return n;
}

// ── baseline snapshot ──────────────────────────────────────────
void stereo_snapshot_baseline(BeaconState &b) {
    // Fit line against the calibrated mean phase per subcarrier.
    // (b.phase_baseline was filled by csi_baseline_finalize.)
    float slope, intercept;
    int n = stereo_fit_line(b.phase_baseline, slope, intercept);
    if (n >= 3) {
        b.slope_baseline     = slope;
        b.intercept_baseline = intercept;
        b.phase_baseline_valid = true;
    } else {
        b.slope_baseline = 0;
        b.intercept_baseline = 0;
        b.phase_baseline_valid = false;
    }
}

// ── peer baseline stash (PRIMARY only) ─────────────────────────
// Indexed by beacon_id (1..8).  Slot 0 unused.
struct StashedPeerBase {
    bool  present;
    float slope;
    float intercept;
};
static StashedPeerBase s_peer_base_stash[16] = {};

// Fold PRIMARY's local slope/intercept baseline with a stashed peer
// baseline for the same beacon → replace with disparity baseline in
// b.slope_baseline / b.intercept_baseline.  Only run this ONCE per beacon
// per calibration cycle.  We detect that by re-checking: if the beacon's
// baseline slot is still "local only" we fold; otherwise skip.
static void fold_disparity_baseline_locked(BeaconState &b) {
    if (!b.phase_baseline_valid) return;
    if (g_app.peer.role != ROLE_PRIMARY) return;
    if (b.id >= 16) return;
    if (!s_peer_base_stash[b.id].present) return;
    // Local baseline currently holds LOCAL slope/intercept.  Replace with
    // disparity baseline = local - peer.
    b.slope_baseline     = b.slope_baseline     - s_peer_base_stash[b.id].slope;
    b.intercept_baseline = b.intercept_baseline - s_peer_base_stash[b.id].intercept;
    // Wrap intercept to [-π, π] for numerical hygiene.
    b.intercept_baseline = wrap_pi(b.intercept_baseline);
    // Mark stash consumed so we don't fold twice on re-cal without a fresh peer packet.
    s_peer_base_stash[b.id].present = false;
    Serial.printf("[stereo] disparity baseline set B%u slope=%.4f int=%.3f\n",
                  (unsigned)b.id, b.slope_baseline, b.intercept_baseline);
}

void stereo_ingest_peer_baseline(const PeerBaselinePacket &pkt) {
    if (g_app.peer.role != ROLE_PRIMARY) return;
    if (pkt.beacon_id >= 16) return;
    s_peer_base_stash[pkt.beacon_id].present   = pkt.valid ? true : false;
    s_peer_base_stash[pkt.beacon_id].slope     = pkt.slope_baseline;
    s_peer_base_stash[pkt.beacon_id].intercept = pkt.intercept_baseline;
    // If our local baseline for this beacon is already valid, fold now.
    for (int i = 0; i < MAX_BEACONS; i++) {
        BeaconState &b = g_app.beacon[i];
        if (b.active && b.id == pkt.beacon_id) {
            fold_disparity_baseline_locked(b);
            break;
        }
    }
}

// Called at end of csi_baseline_finalize on both units.
void stereo_on_baseline_finalized() {
    if (g_app.peer.role == ROLE_SECONDARY) {
        // Push our per-beacon baselines to primary — send each 3× spaced
        // out over the next second in case any drop; ESP-NOW has no ACK
        // at this level.
        for (int i = 0; i < MAX_BEACONS; i++) {
            BeaconState &b = g_app.beacon[i];
            if (!b.active) continue;
            PeerBaselinePacket p = {};
            p.magic              = PEER_BASE_MAGIC;
            p.beacon_id          = b.id;
            p.valid              = b.phase_baseline_valid ? 1 : 0;
            p.slope_baseline     = b.slope_baseline;
            p.intercept_baseline = b.intercept_baseline;
            p.cal_frames         = b.cal_count;
            for (int rep = 0; rep < 3; rep++) {
                peer_send_baseline(p);
                delay(30);
            }
        }
    } else if (g_app.peer.role == ROLE_PRIMARY) {
        // Fold any already-received peer baselines into disparity baselines.
        for (int i = 0; i < MAX_BEACONS; i++) {
            BeaconState &b = g_app.beacon[i];
            if (b.active) fold_disparity_baseline_locked(b);
        }
    }
}

// ── ring buffer of local (own-RX) records ──────────────────────
void stereo_record_local(BeaconState &b, uint32_t counter, uint32_t stamp_ms) {
    auto &e = b.stereo_ring[b.stereo_ring_head];
    e.counter   = counter;
    e.stamp_ms  = stamp_ms;
    e.slope     = b.slope_cur;
    e.intercept = b.intercept_cur;
    e.mean_amp  = b.feat_mean_amp;
    e.paired    = false;
    b.stereo_ring_head = (b.stereo_ring_head + 1) % STEREO_MAG_PACK_MAX;
}

// ── secondary → primary: send summaries for unpaired local records ─
void stereo_flush_summaries_to_peer() {
    if (g_app.peer.role != ROLE_SECONDARY) return;
    if (!g_app.peer.peer_present) return;
    uint32_t now = millis();
    for (int b = 0; b < MAX_BEACONS; b++) {
        BeaconState &bs = g_app.beacon[b];
        if (!bs.active) continue;
        // Send only entries newer than STEREO_PAIR_WINDOW_MS/2 (very
        // recent) that we haven't sent yet.  We reuse `paired` as the
        // "sent" flag on secondary — cleaner than a second field.
        for (int i = 0; i < STEREO_MAG_PACK_MAX; i++) {
            auto &e = bs.stereo_ring[i];
            if (e.counter == 0) continue;
            if (e.paired) continue;
            if ((now - e.stamp_ms) > STEREO_PAIR_WINDOW_MS) { e.paired = true; continue; }
            PeerCsiSummary s = {};
            s.magic        = PEER_FRAME_MAGIC;
            s.beacon_id    = bs.id;
            s.counter      = e.counter;
            s.rx_stamp_ms  = e.stamp_ms;
            s.slope        = e.slope;
            s.intercept    = e.intercept;
            s.mean_amp     = e.mean_amp;
            s.frame_snr_q8 = 0;
            s.baseline_valid = bs.phase_baseline_valid ? 1 : 0;
            peer_send_csi_summary(s);
            e.paired = true;   // mark as sent
        }
    }
}

// ── primary: pairing state ─────────────────────────────────────
// For each beacon we track the most recent successful pair's
// (Δslope, Δintercept) residual with respect to baseline.  Updated as
// peer summaries arrive; consumed by stereo_update_aoa().
struct PairResidual {
    float d_slope;
    float d_intercept;
    float amp_peer;
    uint32_t stamp_ms;
    bool valid;
};
static PairResidual s_pair[MAX_BEACONS] = {};

// Called when a peer summary arrives on primary.
void stereo_ingest_peer_summary(const PeerCsiSummary &pkt) {
    if (g_app.peer.role != ROLE_PRIMARY) return;

    // Locate matching beacon slot
    int slot = -1;
    for (int i = 0; i < MAX_BEACONS; i++) {
        if (g_app.beacon[i].active && g_app.beacon[i].id == pkt.beacon_id) {
            slot = i; break;
        }
    }
    if (slot < 0) { g_app.peer.peer_frames_dropped++; return; }

    BeaconState &b = g_app.beacon[slot];

    // Search local ring for a matching counter within window
    uint32_t now = millis();
    int best = -1;
    for (int i = 0; i < STEREO_MAG_PACK_MAX; i++) {
        auto &e = b.stereo_ring[i];
        if (e.counter != pkt.counter) continue;
        if ((now - e.stamp_ms) > STEREO_PAIR_WINDOW_MS) continue;
        best = i; break;
    }
    if (best < 0) {
        g_app.peer.peer_frames_dropped++;
        return;
    }
    auto &loc = b.stereo_ring[best];

    // Phase disparity local minus peer.  Do NOT subtract two intercepts
    // that were fit to different subcarrier axes — both fits are over the
    // same CSI_SEL_SC set with x = subcarrier index, so this is legit.
    float slope_disp = loc.slope - pkt.slope;
    float intercept_disp = wrap_pi(loc.intercept - pkt.intercept);

    // Subtract baseline stored on primary for THIS beacon.  Note the
    // baseline_slope/intercept on primary represents the SNAPSHOT of the
    // local-minus-peer disparity captured during calibration IF we stored
    // it that way.  For a first pass we store local-only baseline on
    // primary; the peer's baseline is echoed once at calibration end.
    // Concretely: b.slope_baseline / b.intercept_baseline here should
    // already be baseline OF THE DISPARITY.  See csi_baseline_finalize.
    float d_slope     = slope_disp     - b.slope_baseline;
    float d_intercept = wrap_pi(intercept_disp - b.intercept_baseline);

    s_pair[slot].d_slope     = d_slope;
    s_pair[slot].d_intercept = d_intercept;
    s_pair[slot].amp_peer    = pkt.mean_amp;
    s_pair[slot].stamp_ms    = now;
    s_pair[slot].valid       = true;

    loc.paired = true;
    g_app.peer.peer_pairs_ok++;
    g_app.peer.last_pair_ms = now;
}

// ── primary: solve LO drift, extract AoA ───────────────────────
// Common LO drift is estimated as the median of Δintercept across the
// beacons with recent valid pairs.  Median is more robust than mean when
// one beacon has a scatterer that spoofs a phase shift; that beacon
// becomes an outlier the median rejects.
static float median3(float a, float b, float c) {
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { float t = b; b = c; c = t; if (a > b) { t = a; a = b; b = t; } }
    return b;
}

void stereo_update_aoa() {
    if (g_app.peer.role != ROLE_PRIMARY) return;
    uint32_t now = millis();

    // Collect fresh residuals
    float vals[MAX_BEACONS];
    int   idxs[MAX_BEACONS];
    int n = 0;
    for (int i = 0; i < MAX_BEACONS; i++) {
        if (!s_pair[i].valid) continue;
        if ((now - s_pair[i].stamp_ms) > STEREO_PAIR_WINDOW_MS) continue;
        if (!g_app.beacon[i].phase_baseline_valid) continue;
        vals[n] = s_pair[i].d_intercept;
        idxs[n] = i;
        n++;
    }
    if (n < STEREO_MIN_BEACONS) return;

    // Median of Δintercept → common LO drift
    float lo_drift = 0;
    if (n == 1) lo_drift = vals[0];
    else if (n == 2) lo_drift = (vals[0] + vals[1]) * 0.5f;
    else if (n == 3) lo_drift = median3(vals[0], vals[1], vals[2]);
    else {
        // Simple sort for n==4
        for (int a = 0; a < n - 1; a++)
            for (int b = a + 1; b < n; b++)
                if (vals[a] > vals[b]) { float t = vals[a]; vals[a] = vals[b]; vals[b] = t; }
        lo_drift = (vals[n/2 - 1] + vals[n/2]) * 0.5f;
    }
    g_app.peer.lo_drift_rad = lo_drift;
    g_app.peer.lo_drift_ema = 0.85f * g_app.peer.lo_drift_ema + 0.15f * lo_drift;

    // Per-beacon residual after removing common LO drift → AoA
    for (int j = 0; j < n; j++) {
        int i = idxs[j];
        // NOTE: `vals[]` was reordered above during median — use s_pair.
        float res_i = wrap_pi(s_pair[i].d_intercept - lo_drift);
        float res_s = s_pair[i].d_slope;   // slope isn't LO-drift affected
        float aoa   = stereo_intercept_to_aoa(res_i);

        // Confidence: strong when signal amplitude is high AND residual
        // magnitude is well within unambiguous range AND phase agreement
        // (|res_i| much less than π).
        float amp_norm = fminf(1.0f, s_pair[i].amp_peer * 0.02f);
        float within  = fmaxf(0.0f, 1.0f - fabsf(res_i) / (float)M_PI);
        float conf    = amp_norm * within;

        BeaconState &b = g_app.beacon[i];
        // EMA smooth the AoA — reduces jitter without hiding real motion
        if (b.aoa_conf > 0.01f) b.aoa_rad = 0.7f * b.aoa_rad + 0.3f * aoa;
        else                    b.aoa_rad = aoa;
        b.aoa_conf         = 0.7f * b.aoa_conf + 0.3f * conf;
        b.residual_intercept = res_i;
        b.residual_slope     = res_s;
        b.last_aoa_ms        = now;
    }

    // Invalidate consumed pairs so the same reading doesn't re-fire.
    for (int i = 0; i < MAX_BEACONS; i++) s_pair[i].valid = false;
}

// ── angle math ─────────────────────────────────────────────────
// From two coplanar antennas at spacing d, phase difference at center
// frequency f_c is φ = (2π d / λ) * sin(θ).  Invert for θ.
float stereo_intercept_to_aoa(float residual_intercept) {
    // Baseline in meters
    const float d = STEREO_BASELINE_CM * 0.01f;
    const float k = 2.0f * (float)M_PI * d / CSI_WAVELENGTH_M;
    if (k <= 0.0f) return 0.0f;
    float s = residual_intercept / k;
    if (s >  1.0f) s =  1.0f;
    if (s < -1.0f) s = -1.0f;
    return asinf(s);   // radians
}
