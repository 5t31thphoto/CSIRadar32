// ═══════════════════════════════════════════════════════════════
//  CSI-Radar-S3 — csi.cpp
// ═══════════════════════════════════════════════════════════════
#include "csi.h"
#include "peer.h"
#include "stereo.h"
#include "scene.h"
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <math.h>
#include <string.h>

// Ensure the arduino-esp32 version macros are available so the ESP-NOW
// recv callback signature guard below picks the right prototype.
#ifndef ESP_ARDUINO_VERSION_MAJOR
  #if __has_include(<esp_arduino_version.h>)
    #include <esp_arduino_version.h>
  #endif
#endif
#ifndef ESP_ARDUINO_VERSION_MAJOR
  // Fallback: assume modern core if we can't detect
  #define ESP_ARDUINO_VERSION_MAJOR 3
#endif

// ── Local helpers ───────────────────────────────────────────────
static inline bool mac_prefix_match(const uint8_t *m) {
    for (int i = 0; i < 5; i++)
        if (m[i] != BEACON_MAC_PREFIX[i]) return false;
    return true;
}

// Look up or allocate a beacon slot for this MAC. Returns index or -1.
// Safe to call from ISR because MAX_BEACONS is tiny and we only ever add,
// never remove, during a session.
static int IRAM_ATTR slot_for_mac(const uint8_t *mac) {
    for (int i = 0; i < MAX_BEACONS; i++) {
        if (!g_app.beacon[i].active) continue;
        bool eq = true;
        for (int j = 0; j < 6; j++)
            if (g_app.beacon[i].mac[j] != mac[j]) { eq = false; break; }
        if (eq) return i;
    }
    // Allocate a new slot
    for (int i = 0; i < MAX_BEACONS; i++) {
        if (!g_app.beacon[i].active) {
            for (int j = 0; j < 6; j++) g_app.beacon[i].mac[j] = mac[j];
            g_app.beacon[i].id     = mac[5];
            g_app.beacon[i].active = true;
            g_app.beacon[i].frames = 0;
            g_app.beacon[i].dirty  = false;
            g_app.beacon[i].baseline_valid  = false;
            g_app.beacon[i].walk_calibrated = false;
            g_app.beacon[i].cal_count = 0;
            g_app.beacon[i].hampel_idx = g_app.beacon[i].hampel_count = 0;
            g_app.beacon[i].mv_idx     = g_app.beacon[i].mv_count     = 0;
            g_app.beacon[i].lp_x_prev  = g_app.beacon[i].lp_y_prev    = 0;
            g_app.beacon[i].filtered   = 0;
            g_app.beacon[i].moving_variance = 0;
            g_app.beacon[i].threshold  = 0;
            g_app.beacon[i].walk_peak  = 0;
            g_app.beacon[i].link_metric_raw = 0;
            g_app.beacon[i].link_metric_ema = 0;
            g_app.beacon[i].status = LS_IDLE;
            g_app.beacon[i].last_cal_frame = 0;
            g_app.beacon[i].last_proc_frame = 0;
            g_app.beacon[i].last_counter = 0;
            g_app.beacon[i].stereo_ring_head = 0;
            g_app.beacon[i].phase_baseline_valid = false;
            g_app.beacon[i].slope_baseline = 0;
            g_app.beacon[i].intercept_baseline = 0;
            g_app.beacon[i].aoa_rad = 0;
            g_app.beacon[i].aoa_conf = 0;
            for (int sc = 0; sc < CSI_NUM_SUBCARRIERS; sc++) {
                g_app.beacon[i].cal_phase_i[sc] = 0;
                g_app.beacon[i].cal_phase_q[sc] = 0;
                g_app.beacon[i].phase_baseline[sc] = 0;
            }
            for (int r = 0; r < STEREO_MAG_PACK_MAX; r++) {
                g_app.beacon[i].stereo_ring[r].counter = 0;
                g_app.beacon[i].stereo_ring[r].paired = true;
            }
            g_app.beacon_count++;
            return i;
        }
    }
    return -1;
}

// ── CSI RX callback (IRAM) ─────────────────────────────────────
// Extract amplitudes, route to per-beacon slot by MAC, set dirty flag.
static void IRAM_ATTR csi_rx_cb(void *ctx, wifi_csi_info_t *info) {
    (void)ctx;
    if (!info || !info->buf || info->len < 2) return;
    const uint8_t *src = (const uint8_t*)&info->mac;
    if (!mac_prefix_match(src)) return;   // ignore everything not from our beacons

    int slot = slot_for_mac(src);
    if (slot < 0) return;

    int pairs = info->len / 2;
    if (pairs > CSI_NUM_SUBCARRIERS) pairs = CSI_NUM_SUBCARRIERS;
    const int8_t *buf = (const int8_t*)info->buf;

    BeaconState &b = g_app.beacon[slot];

    // Compute amplitude AND phase per subcarrier.  atan2f is fine on the S3
    // FPU; ~50 cycles × 64 subcarriers ≈ 13 µs at 240 MHz.  Phase is what
    // the stereo AoA path needs.
    for (int i = 0; i < pairs; i++) {
        float q  = (float)buf[i * 2];
        float ii = (float)buf[i * 2 + 1];
        b.amplitude[i] = sqrtf(q * q + ii * ii);
        b.phase[i]     = atan2f(q, ii);
    }
    // Zero any remaining subcarriers so downstream math stays defined
    for (int i = pairs; i < CSI_NUM_SUBCARRIERS; i++) {
        b.amplitude[i] = 0.0f;
        b.phase[i]     = 0.0f;
    }

    // v0.4: inter-arrival EMA for RX-side beacon rate inference.
    // First frame just seeds last_frame_ms; from the second onward we
    // fold (now - last) into the EMA.  Alpha=0.15 tracks ~7 frames.
    uint32_t now_ms = millis();
    if (b.last_frame_ms != 0) {
        float delta = (float)(now_ms - b.last_frame_ms);
        if (b.inter_arrival_ms_ema == 0) {
            b.inter_arrival_ms_ema = delta;
        } else {
            b.inter_arrival_ms_ema += 0.15f * (delta - b.inter_arrival_ms_ema);
        }
    }
    b.frames++;
    b.last_frame_ms = now_ms;
    b.dirty = true;
    g_app.total_csi_frames++;
}

// ── ESP-NOW recv (main-task ctx) ───────────────────────────────
// Three jobs:
//   1) Demux peer packets first — a peer HELLO/SUMMARY/COMMAND is
//      identified by a magic word in the first 4 bytes.
//   2) For real beacon frames (MAC prefix match), extract the beacon
//      counter from the payload for stereo pairing.
//   3) Force slot allocation so discovery is snappy even before the
//      first CSI callback fires for this beacon.
static void handle_espnow_common(const uint8_t *src, const uint8_t *data, int len) {
    if (!src || !data || len <= 0) return;

    // Peer packets first (magic-word demux) — they come from another
    // T-Display, NOT from a beacon, so they won't match BEACON_MAC_PREFIX.
    if (peer_try_consume(src, data, len)) return;

    if (!mac_prefix_match(src)) return;
    int slot = slot_for_mac(src);
    if (slot < 0) return;

    // Beacon payload: uint32_t counter (see beacon firmware).
    if (len >= (int)sizeof(uint32_t)) {
        uint32_t counter;
        memcpy(&counter, data, sizeof(counter));
        g_app.beacon[slot].last_counter = counter;
    }
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
static void espnow_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (!info) return;
    handle_espnow_common(info->src_addr, data, len);
}
#else
static void espnow_recv_cb(const uint8_t *src, const uint8_t *data, int len) {
    handle_espnow_common(src, data, len);
}
#endif

// ── Filter blocks ──────────────────────────────────────────────
static float compute_turbulence(const volatile float *amp) {
    float sum = 0, sq = 0;
    int n = 0;
    for (int i = 0; i < CSI_SEL_COUNT; i++) {
        int sc = CSI_SEL_SC[i];
        if (sc >= CSI_NUM_SUBCARRIERS) continue;
        float v = amp[sc];
        sum += v; sq += v * v; n++;
    }
    if (n < 2) return 0;
    float mean = sum / n;
    float var  = (sq / n) - mean * mean;
    if (var < 0) var = 0;
    float sd = sqrtf(var);
    return (mean > 0.1f) ? (sd / mean) : sd;
}

static float hampel(BeaconState &b, float value) {
    b.hampel_buf[b.hampel_idx] = value;
    b.hampel_idx = (b.hampel_idx + 1) % HAMPEL_WIN;
    if (b.hampel_count < HAMPEL_WIN) b.hampel_count++;

    float sorted[HAMPEL_WIN];
    memcpy(sorted, b.hampel_buf, b.hampel_count * sizeof(float));
    // insertion sort — 7 items, cheap
    for (int i = 1; i < b.hampel_count; i++) {
        float k = sorted[i]; int j = i - 1;
        while (j >= 0 && sorted[j] > k) { sorted[j+1] = sorted[j]; j--; }
        sorted[j+1] = k;
    }
    float median = sorted[b.hampel_count / 2];

    float diffs[HAMPEL_WIN];
    for (int i = 0; i < b.hampel_count; i++)
        diffs[i] = fabsf(b.hampel_buf[i] - median);
    for (int i = 1; i < b.hampel_count; i++) {
        float k = diffs[i]; int j = i - 1;
        while (j >= 0 && diffs[j] > k) { diffs[j+1] = diffs[j]; j--; }
        diffs[j+1] = k;
    }
    float mad = diffs[b.hampel_count / 2] * MAD_SCALE;
    if (mad > 0 && fabsf(value - median) > HAMPEL_THRESH * mad) return median;
    return value;
}

static float lowpass(BeaconState &b, float x) {
    // 1st order Butterworth, cutoff LP_CUTOFF_HZ @ SAMPLE_RATE_HZ.
    // Precompute once — cheap to recompute per call, still.
    static float b0 = 0, a1 = 0;
    static bool inited = false;
    if (!inited) {
        float wc = tanf((float)M_PI * LP_CUTOFF_HZ / SAMPLE_RATE_HZ);
        float k  = 1.0f + wc;
        b0 = wc / k;
        a1 = (wc - 1.0f) / k;
        inited = true;
    }
    float y = b0 * x + b0 * b.lp_x_prev - a1 * b.lp_y_prev;
    b.lp_x_prev = x;
    b.lp_y_prev = y;
    return y;
}

static float moving_variance(BeaconState &b, float x) {
    b.mv_buf[b.mv_idx] = x;
    b.mv_idx = (b.mv_idx + 1) % MOVVAR_WIN;
    if (b.mv_count < MOVVAR_WIN) b.mv_count++;
    float s = 0, sq = 0;
    for (int i = 0; i < b.mv_count; i++) {
        s  += b.mv_buf[i];
        sq += b.mv_buf[i] * b.mv_buf[i];
    }
    float m = s / b.mv_count;
    float v = (sq / b.mv_count) - m * m;
    return v > 0 ? v : 0;
}

static float percentile(float *arr, int n, float pct) {
    // sort a copy; small N (~500) is fine on S3
    for (int i = 1; i < n; i++) {
        float k = arr[i]; int j = i - 1;
        while (j >= 0 && arr[j] > k) { arr[j+1] = arr[j]; j--; }
        arr[j+1] = k;
    }
    int idx = (int)(pct / 100.0f * (n - 1));
    if (idx >= n) idx = n - 1;
    if (idx < 0)  idx = 0;
    return arr[idx];
}

// ── Public API ─────────────────────────────────────────────────
void csi_engine_begin() {
    // Bring Wi-Fi up in the same config beacons use
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(100);

    WiFi.mode(WIFI_STA);
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT40);
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_set_channel(CSI_CHANNEL, WIFI_SECOND_CHAN_BELOW);
    delay(50);

    if (esp_now_init() != ESP_OK) {
        // If ESP-NOW init fails we can still get CSI, just no fast discovery.
        Serial.println("[CSI] esp_now_init failed");
    } else {
        esp_now_register_recv_cb(espnow_recv_cb);
    }

    wifi_csi_config_t cfg = {};
    cfg.lltf_en        = true;
    cfg.htltf_en       = true;
    cfg.stbc_htltf2_en = true;
    cfg.ltf_merge_en   = true;
    cfg.channel_filter_en = true;
    cfg.manu_scale     = false;
    cfg.shift          = false;
    esp_wifi_set_csi_config(&cfg);
    esp_wifi_set_csi_rx_cb(csi_rx_cb, NULL);
    esp_wifi_set_csi(true);
    esp_wifi_set_promiscuous(true);

    Serial.println("[CSI] engine started on ch 11 HT40");
}

void csi_engine_end() {
    esp_wifi_set_csi(false);
    esp_wifi_set_csi_rx_cb(NULL, NULL);
    esp_wifi_set_promiscuous(false);
    esp_now_deinit();
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
}

int csi_get_beacon_count() { return g_app.beacon_count; }

void csi_reset_discovery() {
    for (int i = 0; i < MAX_BEACONS; i++) {
        g_app.beacon[i].active = false;
        g_app.beacon[i].last_frame_ms = 0;
        g_app.beacon[i].inter_arrival_ms_ema = 0;
    }
    g_app.beacon_count = 0;
    g_app.total_csi_frames = 0;
}

void csi_choose_mode(RadarMode m) {
    g_app.mode = m;
}

void csi_assign_default_geometry(float side_cm) {
    // Coordinate frame: T-Display sits at (0, 0). Beacons live in +Y half-plane.
    // Units: centimetres. UI scales as needed.
    switch (g_app.mode) {
        case RM_TRIANGLE_3: {
            // Equilateral triangle around origin, apex at top of screen.
            // B0 top, B1 bottom-left, B2 bottom-right.
            float r = side_cm / sqrtf(3.0f);          // circumradius
            g_app.beacon[0].pos_x =  0.0f;             g_app.beacon[0].pos_y =  r;
            g_app.beacon[1].pos_x = -side_cm * 0.5f;   g_app.beacon[1].pos_y = -r * 0.5f;
            g_app.beacon[2].pos_x =  side_cm * 0.5f;   g_app.beacon[2].pos_y = -r * 0.5f;
            break;
        }
        case RM_LINE_2: {
            g_app.beacon[0].pos_x = -side_cm * 0.5f;   g_app.beacon[0].pos_y = 0;
            g_app.beacon[1].pos_x =  side_cm * 0.5f;   g_app.beacon[1].pos_y = 0;
            break;
        }
        case RM_TRIPWIRE_1: {
            g_app.beacon[0].pos_x = 0;                 g_app.beacon[0].pos_y = side_cm * 0.5f;
            break;
        }
        default: break;
    }
}

void csi_reset_filters(bool hard) {
    for (int i = 0; i < MAX_BEACONS; i++) {
        BeaconState &b = g_app.beacon[i];
        if (!b.active) continue;
        b.hampel_idx = b.hampel_count = 0;
        b.mv_idx = b.mv_count = 0;
        b.lp_x_prev = b.lp_y_prev = 0;
        b.filtered = 0;
        b.moving_variance = 0;
        b.link_metric_raw = 0;
        b.link_metric_ema = 0;
        b.cal_count = 0;
        b.last_cal_frame = b.frames;    // "no new frames yet" for accumulator
        b.status = LS_IDLE;
        // Zero phase accumulators so a fresh baseline pass has a clean sum.
        for (int sc = 0; sc < CSI_NUM_SUBCARRIERS; sc++) {
            b.cal_phase_i[sc] = 0;
            b.cal_phase_q[sc] = 0;
        }
        if (hard) {
            b.baseline_valid = false;
            b.walk_calibrated = false;
            b.threshold = 0;
            b.walk_peak = 0;
            b.phase_baseline_valid = false;
            b.aoa_conf = 0;
            b.aoa_rad  = 0;
            // Clear stereo ring
            for (int r = 0; r < STEREO_MAG_PACK_MAX; r++) {
                b.stereo_ring[r].counter = 0;
                b.stereo_ring[r].paired  = true;
            }
            b.stereo_ring_head = 0;
        }
    }
}

int csi_process_frames() {
    int processed = 0;
    for (int i = 0; i < MAX_BEACONS; i++) {
        BeaconState &b = g_app.beacon[i];
        if (!b.active || !b.dirty) continue;
        b.dirty = false;

        // 1. Feature extraction — kept independently, NOT reduced yet.
        float turb = compute_turbulence(b.amplitude);

        float mean_amp = 0, energy = 0, delta_base = 0, temporal = 0;
        int n = 0;
        for (int s = 0; s < CSI_SEL_COUNT; s++) {
            int sc = CSI_SEL_SC[s];
            if (sc >= CSI_NUM_SUBCARRIERS) continue;
            float a = b.amplitude[sc];
            mean_amp += a;
            energy   += a * a;
            if (b.baseline_valid) {
                float d = a - b.baseline[sc];
                delta_base += d * d;
            }
            float td = a - b.prev_amplitude[sc];
            temporal += td * td;
            b.prev_amplitude[sc] = a;
            n++;
        }
        if (n > 0) {
            mean_amp /= n;
            delta_base = sqrtf(delta_base / n);
            temporal   = sqrtf(temporal / n);
        }

        b.feat_turbulence     = turb;
        b.feat_mean_amp       = mean_amp;
        b.feat_energy         = energy;
        b.feat_delta_baseline = delta_base;
        b.feat_temporal_delta = temporal;

        // 2. Filter pipeline on the CV-turbulence scalar (still per-beacon).
        float filt = hampel(b, turb);
        filt       = lowpass(b, filt);
        b.filtered = filt;

        // 3. Moving variance → detector input.
        b.moving_variance = moving_variance(b, filt);

        // 4. Link metric fusion (only for the current-mode fused view).
        //    Combines moving_variance (motion) with baseline deviation
        //    (presence) and temporal delta (transient activity).
        float mv_norm = 0;
        if (b.threshold > 1e-9f) mv_norm = b.moving_variance / (b.threshold * 4.0f);
        if (mv_norm > 1.0f) mv_norm = 1.0f;

        float base_norm = 0;
        if (b.baseline_valid && b.feat_mean_amp > 0.1f)
            base_norm = b.feat_delta_baseline / (b.feat_mean_amp * 0.6f);
        if (base_norm > 1.0f) base_norm = 1.0f;

        float temp_norm = b.feat_temporal_delta / 30.0f;   // heuristic scale
        if (temp_norm > 1.0f) temp_norm = 1.0f;

        // Fused link metric — motion weighs most, presence & transient help.
        b.link_metric_raw = 0.6f * mv_norm + 0.25f * base_norm + 0.15f * temp_norm;
        if (b.link_metric_raw > 1.0f) b.link_metric_raw = 1.0f;

        // EMA smoothing
        b.link_metric_ema = (1.0f - LINK_METRIC_EMA_ALPHA) * b.link_metric_ema
                           + LINK_METRIC_EMA_ALPHA * b.link_metric_raw;

        // 5. Status from thresholds
        if (b.baseline_valid && b.threshold > 0) {
            float thr = b.threshold * g_app.sensitivity;
            if (b.moving_variance > thr * PRESENCE_MULT)      b.status = LS_PRESENCE;
            else if (b.moving_variance > thr * MOTION_MULT)   b.status = LS_MOTION;
            else                                              b.status = LS_IDLE;
        } else {
            b.status = LS_IDLE;
        }

        // 6. Stereo line-fit on this frame's phase (both PRIMARY and SECONDARY
        //    compute this; SECONDARY forwards it to PRIMARY via peer link).
        //    Snapshot phase into a local buffer first so unwrap can mutate.
        float phase_snap[CSI_NUM_SUBCARRIERS];
        for (int s = 0; s < CSI_NUM_SUBCARRIERS; s++) phase_snap[s] = b.phase[s];
        float m, c;
        int nfit = stereo_fit_line(phase_snap, m, c);
        if (nfit >= 3) {
            b.slope_cur     = m;
            b.intercept_cur = c;
            stereo_record_local(b, b.last_counter, b.last_frame_ms);
        }

        processed++;
    }
    // Secondary: after processing this cycle, flush any new local records
    // to the primary as compact summaries.  No-op on primary/solo.
    stereo_flush_summaries_to_peer();
    return processed;
}

// ── Calibration ────────────────────────────────────────────────
// Baseline pass: accumulate moving_variance (for threshold) AND per-subcarrier
// mean amplitude (for baseline vector).

void csi_baseline_accumulate() {
    for (int i = 0; i < MAX_BEACONS; i++) {
        BeaconState &b = g_app.beacon[i];
        if (!b.active) continue;
        if (b.cal_count >= BASELINE_FRAMES) continue;
        // Only advance if this beacon actually received a new frame since
        // the last accumulator tick — prevents double-counting a stale
        // moving_variance value.
        if (b.frames == b.last_cal_frame) continue;
        b.last_cal_frame = b.frames;

        // Store MV sample
        b.cal_values[b.cal_count] = b.moving_variance;

        // Running mean of amplitudes → baseline[]
        // Use incremental mean so we don't need a second buffer.
        float k = 1.0f / (float)(b.cal_count + 1);
        for (int sc = 0; sc < CSI_NUM_SUBCARRIERS; sc++) {
            b.baseline[sc] = b.baseline[sc] * (1.0f - k) + b.amplitude[sc] * k;
            // Accumulate cos/sin(phase) so we can average phases correctly
            // (wraparound-safe) via mean-of-unit-vectors in finalize.
            float ph = b.phase[sc];
            b.cal_phase_i[sc] += cosf(ph);
            b.cal_phase_q[sc] += sinf(ph);
        }

        b.cal_count++;
    }
}

void csi_baseline_finalize() {
    for (int i = 0; i < MAX_BEACONS; i++) {
        BeaconState &b = g_app.beacon[i];
        if (!b.active || b.cal_count < 30) continue;
        // Threshold = P95 of the MV samples, with a small safety headroom
        b.threshold = percentile(b.cal_values, b.cal_count, 95.0f) * 1.1f;
        if (b.threshold < 1e-6f) b.threshold = 1e-6f;

        // Baseline std as a coarse per-sc variability (last 64 samples if any).
        // For now, seed with a small floor so delta_base normalizes sanely.
        for (int sc = 0; sc < CSI_NUM_SUBCARRIERS; sc++) {
            if (b.baseline[sc] < 0.1f) b.baseline[sc] = 0.1f;
            b.baseline_std[sc] = 1.0f;
        }
        b.baseline_valid = true;

        // Compute per-subcarrier mean phase via mean-of-unit-vectors.
        // Σ(cos φ) and Σ(sin φ) were accumulated in csi_baseline_accumulate;
        // atan2 of the two sums gives a wraparound-safe mean.
        for (int sc = 0; sc < CSI_NUM_SUBCARRIERS; sc++) {
            float ci = b.cal_phase_i[sc];
            float qi = b.cal_phase_q[sc];
            if ((ci*ci + qi*qi) > 1e-6f) {
                b.phase_baseline[sc] = atan2f(qi, ci);
            } else {
                b.phase_baseline[sc] = 0.0f;
            }
        }
        // Snapshot line-fit of local phase baseline.  On SECONDARY that's
        // the value we'll send to PRIMARY.  On PRIMARY, this fills local
        // baseline; stereo_on_baseline_finalized will fold the peer's
        // baseline in to produce the actual disparity baseline.
        stereo_snapshot_baseline(b);
    }
    // Cross-unit exchange of phase baselines
    stereo_on_baseline_finalized();
}

void csi_walk_accumulate() {
    for (int i = 0; i < MAX_BEACONS; i++) {
        BeaconState &b = g_app.beacon[i];
        if (!b.active) continue;
        // Only update on new frames — cheap and keeps semantics clean.
        if (b.frames == b.last_cal_frame) continue;
        b.last_cal_frame = b.frames;
        if (b.moving_variance > b.walk_peak) b.walk_peak = b.moving_variance;
    }
}

void csi_walk_finalize() {
    // v0.3: The scene module owns per-beacon weighting via its kernel +
    // walk-derived SNR calculation.  csi_walk_finalize's only job here
    // is to mark beacons that responded during the walk as "walk_calibrated"
    // for legacy status displays.  The heavy lifting happens in
    // scene_finalize_cal() invoked from ST_CAL_FINALIZE.
    for (int i = 0; i < MAX_BEACONS; i++) {
        BeaconState &b = g_app.beacon[i];
        if (!b.active) continue;
        if (b.walk_peak > b.threshold * 3.0f) b.walk_calibrated = true;
    }
}

float csi_baseline_progress() {
    // Return progress of the SLOWEST beacon (min) so UI matches worst case.
    float minp = 1.0f;
    bool any = false;
    for (int i = 0; i < MAX_BEACONS; i++) {
        BeaconState &b = g_app.beacon[i];
        if (!b.active) continue;
        any = true;
        float p = (float)b.cal_count / (float)BASELINE_FRAMES;
        if (p > 1.0f) p = 1.0f;
        if (p < minp) minp = p;
    }
    return any ? minp : 0.0f;
}

float csi_walk_progress() {
    // Rough: fraction of active beacons that reached walk_calibrated.
    int total = 0, ok = 0;
    for (int i = 0; i < MAX_BEACONS; i++) {
        if (!g_app.beacon[i].active) continue;
        total++;
        if (g_app.beacon[i].walk_calibrated) ok++;
    }
    return total ? (float)ok / (float)total : 0.0f;
}

// ── Spatial estimator ──────────────────────────────────────────
// Level-1 approach from the design note: each link's midpoint between beacon
// and T-Display is where a perturber is most likely to be. Weighted centroid
// of midpoints, weighted by link_metric_ema, gives a rough position.
// Confidence = mean link metric (0 if no motion).
void csi_update_spatial() {
    // v0.3: Build a FrameObservation from current beacon state and feed
    // the scene module.  The scene owns all spatial estimation now (via
    // the walk-cal-derived kernel + matching-pursuit reconstruction);
    // this function is just the plumbing that pushes per-frame data.
    FrameObservation obs = {};
    obs.frame_ms = millis();
    obs.n_beacons = 0;
    uint32_t now = obs.frame_ms;
    for (int i = 0; i < MAX_BEACONS; i++) {
        BeaconState &b = g_app.beacon[i];
        if (!b.active) continue;
        obs.beacon[i].beacon_id = b.id;
        // "Fresh" = received a frame in the last ~200 ms
        obs.beacon[i].fresh = ((now - b.last_frame_ms) < 200);
        obs.beacon[i].amp_perturbation = b.link_metric_ema;
        // Phase perturbation: current fit intercept minus baseline
        // (wrap-π so it stays in [-π, π])
        float dphase = b.intercept_cur - b.intercept_baseline;
        while (dphase >  (float)M_PI) dphase -= 2.0f * (float)M_PI;
        while (dphase < -(float)M_PI) dphase += 2.0f * (float)M_PI;
        obs.beacon[i].phase_perturbation = dphase;
        // AoA (stereo only, and only if fresh within 500ms)
        if (g_app.peer.role == ROLE_PRIMARY
            && g_app.peer.peer_present
            && b.aoa_conf > 0.05f
            && (now - b.last_aoa_ms) < 500) {
            obs.beacon[i].have_aoa = true;
            obs.beacon[i].aoa_rad  = b.aoa_rad;
            obs.beacon[i].aoa_conf = b.aoa_conf;
        } else {
            obs.beacon[i].have_aoa = false;
        }
        obs.n_beacons++;
    }
    scene_observe(obs);

    // Legacy tripwire latch (unchanged semantics; the scene module
    // will supersede this eventually but the tripwire view still uses it).
    bool any_presence = false, any_motion = false;
    for (int i = 0; i < MAX_BEACONS; i++) {
        if (!g_app.beacon[i].active) continue;
        if (g_app.beacon[i].status == LS_PRESENCE) any_presence = true;
        else if (g_app.beacon[i].status == LS_MOTION) any_motion = true;
    }
    if (any_presence || any_motion) {
        g_app.alert_latched = true;
        g_app.last_alert_ms = millis();
    }
}

uint32_t csi_frames_seen() { return g_app.total_csi_frames; }
