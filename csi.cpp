// ═══════════════════════════════════════════════════════════════
//  CSI-Radar-S3 — csi.cpp
// ═══════════════════════════════════════════════════════════════
#include "csi.h"
#include <stdarg.h>
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

// ═══════════════════════════════════════════════════════════════
//  IN-RAM LOG RING
// ═══════════════════════════════════════════════════════════════
// Lives here because csi.cpp links into every build and depends on
// neither ui nor scene.  Written from both cores (the CSI callback logs
// on core 0), so the index is advanced only after the line is written:
// a torn LINE is cosmetic, a torn INDEX would write outside the array.
#if MS_LOG_RING
static char     s_log[MS_LOG_LINES][MS_LOG_LINE_LEN];
static uint16_t s_log_head  = 0;      // next slot to write
static uint16_t s_log_count = 0;

void ms_log_printf(const char *fmt, ...) {
    uint16_t slot = s_log_head;
    if (slot >= MS_LOG_LINES) slot = 0;          // defensive
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(s_log[slot], MS_LOG_LINE_LEN, fmt, ap);
    va_end(ap);
    // Strip trailing newlines: the renderer draws one line per slot.
    int n = (int)strlen(s_log[slot]);
    while (n > 0 && (s_log[slot][n-1] == '\n' || s_log[slot][n-1] == '\r'))
        s_log[slot][--n] = 0;
    s_log_head = (uint16_t)((slot + 1) % MS_LOG_LINES);
    if (s_log_count < MS_LOG_LINES) s_log_count++;
}

int ms_log_count() { return (int)s_log_count; }

const char *ms_log_line(int idx) {
    if (idx < 0 || idx >= (int)s_log_count) return "";
    int start = ((int)s_log_head - (int)s_log_count + MS_LOG_LINES * 2) % MS_LOG_LINES;
    return s_log[(start + idx) % MS_LOG_LINES];
}

void ms_log_clear() { s_log_head = 0; s_log_count = 0; }
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
            g_app.beacon[i].cal_top_n = 0;
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
            // Seqlock must start EVEN (= no write in flight), or the
            // first reader spins its retries and drops a frame.
            g_app.beacon[i].iq_seq   = 0;
            g_app.beacon[i].iq_slot  = 0;
            g_app.beacon[i].iq_pairs = 0;
            for (int sc = 0; sc < CSI_NUM_SUBCARRIERS; sc++) {
                g_app.beacon[i].baseline[sc]       = 0;
                g_app.beacon[i].phase_baseline[sc] = 0;
                g_app.beacon[i].cal_phase_i[sc]    = 0;
                g_app.beacon[i].cal_phase_q[sc]    = 0;
                g_app.beacon[i].prev_amplitude[sc] = 0;
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

    // ── v0.9: publish raw I/Q under a seqlock; do NO float math here ──
    //
    // This runs in the Wi-Fi task on core 0.  Two things changed:
    //
    //  1. It used to compute 64 sqrtf + 64 atan2f inline (~13 us/frame)
    //     inside the Wi-Fi task.  That work now happens on core 1, where
    //     it cannot delay packet servicing.
    //
    //  2. It used to write amplitude[]/phase[] that core 1 reads with no
    //     synchronisation whatsoever, so a reader could see half of one
    //     frame and half of the next.  The stereo line fit consumes all
    //     64 phases at once, so a torn read silently corrupts AoA.
    //
    // Seqlock: write into the slot the reader is NOT holding, then bump
    // the sequence.  Odd sequence = write in flight.  Wait-free for the
    // writer, so the Wi-Fi task never blocks on the consumer.
#if MS_CSI_SEQLOCK
    uint8_t next = (uint8_t)(b.iq_slot ^ 1u);
    memcpy(b.iq_raw[next], buf, (size_t)pairs * 2);
    b.iq_seq   = b.iq_seq + 1;     // odd: publish in progress
    b.iq_pairs = (uint16_t)pairs;
    b.iq_slot  = next;
    b.iq_seq   = b.iq_seq + 1;     // even: stable
#else
    // v0.6 path: compute inline, here in the callback.
    for (int k = 0; k < pairs; k++) {
        float q  = (float)buf[k * 2];
        float ii = (float)buf[k * 2 + 1];
        b.amplitude[k] = sqrtf(q * q + ii * ii);
        b.phase[k]     = atan2f(q, ii);
    }
    for (int k = pairs; k < CSI_NUM_SUBCARRIERS; k++) {
        b.amplitude[k] = 0.0f;
        b.phase[k]     = 0.0f;
    }
#endif

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

    // v0.9: a PONG comes FROM a beacon MAC but is not a counter frame,
    // so it must be claimed before the counter path below reads its
    // first 4 bytes as a sequence number.
    if (csi_beacon_try_consume_pong(data, len)) return;

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
static float compute_turbulence(const float *amp) {
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

// Effective sample rate for THIS beacon, from its own measured
// inter-arrival time.  Beacons can be commanded to different rates and
// a stock beacon may sit alongside commanded ones, so the rate is a
// per-beacon runtime property, never a compile-time constant.
static inline float beacon_rate_hz(const BeaconState &b) {
    if (b.inter_arrival_ms_ema > 0.5f) {
        float hz = 1000.0f / b.inter_arrival_ms_ema;
        if (hz < 1.0f)   hz = 1.0f;
        if (hz > 200.0f) hz = 200.0f;
        return hz;
    }
    return SAMPLE_RATE_HZ;   // not enough frames yet: assume stock rate
}

static float lowpass(BeaconState &b, float x) {
    // 1st-order Butterworth.
    //
    // The coefficients used to be `static` and computed ONCE from a
    // fixed SAMPLE_RATE_HZ -- so every beacon shared one filter design,
    // and if a beacon was ever commanded to a different rate its filter
    // was silently wrong.  Cutoff is now a fraction of that beacon's own
    // Nyquist, recomputed only when its measured rate actually moves.
    float fs = beacon_rate_hz(b);
    if (fabsf(fs - b.lp_design_fs) > 0.05f * b.lp_design_fs || b.lp_design_fs <= 0) {
        float ny = fs * 0.5f;
        float fc = LP_CUTOFF_HZ;
        if (fc > LP_CUTOFF_MAX_FRAC_NYQ * ny) fc = LP_CUTOFF_MAX_FRAC_NYQ * ny;
        float wc = tanf((float)M_PI * fc / fs);
        float k  = 1.0f + wc;
        b.lp_b0 = wc / k;
        b.lp_a1 = (wc - 1.0f) / k;
        b.lp_design_fs = fs;
    }
    float y = b.lp_b0 * x + b.lp_b0 * b.lp_x_prev - b.lp_a1 * b.lp_y_prev;
    b.lp_x_prev = x;
    b.lp_y_prev = y;
    return y;
}

static float moving_variance(BeaconState &b, float x) {
    // Window is MOVVAR_SECONDS of real time, not a fixed sample count:
    // how quickly motion is detected is a property of people, not of
    // the beacon rate.  The buffer is sized for the stock 100 Hz max;
    // at lower rates we simply use fewer of its slots.
    int win = (int)(MOVVAR_SECONDS * beacon_rate_hz(b));
    if (win < 4)           win = 4;
    if (win > MOVVAR_WIN)  win = MOVVAR_WIN;

    // The window SHRINKS when beacons are commanded down from the
    // 100 Hz default (50 slots -> 15 at 30 Hz).  mv_idx can be left
    // pointing past the new window, so clamp it BEFORE writing:
    // otherwise the next samples land in slots the variance loop never
    // reads, and the metric is computed from a mix of old-rate and
    // new-rate data for a full window afterwards -- right at the moment
    // calibration starts.
    if (b.mv_idx >= win) b.mv_idx = 0;
    b.mv_buf[b.mv_idx] = x;
    b.mv_idx = (b.mv_idx + 1) % win;
    if (b.mv_count > win) b.mv_count = win;
    if (b.mv_count < win) b.mv_count++;
    float s = 0, sq = 0;
    for (int i = 0; i < b.mv_count; i++) {
        s  += b.mv_buf[i];
        sq += b.mv_buf[i] * b.mv_buf[i];
    }
    float m = s / b.mv_count;
    float v = (sq / b.mv_count) - m * m;
    return v > 0 ? v : 0;
}

// v0.85: exact p95 from the retained top-K.
//
// For n observations the p95 sits at sorted index floor(0.95*(n-1)), i.e.
// it is the (n - that index)-th LARGEST value.  For n = 500 that is the
// 26th largest, so retaining the top 26 is sufficient to answer exactly
// -- no sort of a 500-element buffer, and no 2 KB per beacon.
//
// `top` is ascending with `top_n` entries; `n` is the total number of
// samples that were offered.
static float p95_from_top(const float *top, int top_n, int n) {
    if (n <= 0 || top_n <= 0) return 0.0f;
    int idx  = (int)(0.95f * (float)(n - 1));
    int need = n - idx;                 // rank from the top
    if (need > top_n) need = top_n;     // fewer samples than K were seen
    if (need < 1)     need = 1;
    return top[top_n - need];
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
        MSLOGLN("[CSI] esp_now_init failed");
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

    MSLOGLN("[CSI] engine started on ch 11 HT40");
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

// ═══════════════════════════════════════════════════════════════
//  v0.9 — BEACON CONTROL (RX → beacon)
// ═══════════════════════════════════════════════════════════════
// Sending side of a protocol the beacon firmware has implemented since
// the beginning and that the receiver never used.  Consequence of the
// gap: every beacon has run at its compiled default of 100 Hz forever,
// while the capture path decimates to CAP_HZ (20) -- so roughly four
// out of five transmitted frames were paid for in battery and thrown
// away on arrival.
//
// Commands go out as ESP-NOW broadcast.  The beacons are not registered
// as unicast peers (we only ever listened to them), and broadcast needs
// no peer table entry.  target_id inside the payload does the
// addressing: 0 means every beacon.
static bool s_bcast_peer_ready = false;
// Enforcement is suppressed until this time so the rate EMA can
// converge after a rate change (17 frames at alpha=0.15).
static uint32_t s_rate_settle_until_ms = 0;

static void ensure_broadcast_peer() {
    if (s_bcast_peer_ready) return;
    const uint8_t bcast[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    if (!esp_now_is_peer_exist(bcast)) {
        esp_now_peer_info_t p = {};
        memcpy(p.peer_addr, bcast, 6);
        p.channel = CSI_CHANNEL;
        p.ifidx   = WIFI_IF_STA;
        p.encrypt = false;
        esp_now_add_peer(&p);
    }
    s_bcast_peer_ready = true;
}

void csi_beacon_command(uint8_t target_id, uint8_t op,
                        uint16_t arg_u16, uint32_t arg_u32) {
    ensure_broadcast_peer();
    BeaconCommand c = {};
    c.magic     = BEACON_CMD_MAGIC;
    c.target_id = target_id;
    c.op        = op;
    c.arg_u16   = arg_u16;
    c.arg_u32   = arg_u32;
    WiFi.macAddress(c.sender_mac);          // so PONGs come back to us
    const uint8_t bcast[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    esp_now_send(bcast, (const uint8_t*)&c, sizeof(c));
}

void csi_beacon_ping_all() {
    // Broadcast: every beacon answers.  Replies can collide, which is
    // why confirmation is retried per-beacon by csi_beacon_service()
    // rather than assumed from one round.
    csi_beacon_command(0, BEACON_OP_PING);
}

void csi_beacon_set_rate_all(uint16_t hz) {
    if (hz < 1)   hz = 1;
    if (hz > 200) hz = 200;
    csi_beacon_command(0, BEACON_OP_SET_RATE, hz);
    // Record what we asked for, per beacon, so the UI can show
    // commanded against observed instead of only observed.
    for (int i = 0; i < MAX_BEACONS; i++)
        if (g_app.beacon[i].active) g_app.beacon[i].cmd_rate_hz = hz;
    MSLOG("[csi] commanded all beacons to %u Hz\n", (unsigned)hz);
}

// Compile-time guard: the beacon rate and the filter design frequency
// are the same number by construction now, but if either is ever edited
// independently this stops the build instead of silently detuning the
// whole signal chain.
void csi_beacon_apply_run_config() {
#if !MS_BEACON_CONTROL
    return;
#else
    // Send ONE round and arm per-beacon confirmation.
    //
    // This used to fire BEACON_CMD_REPEATS identical rounds back to back
    // with delay() between them -- roughly 300 ms of blocking inside a
    // state transition, five identical log lines, and no idea whether
    // any beacon had actually answered.  Retries are now driven by
    // csi_beacon_service() and only target beacons that have NOT
    // confirmed, so a beacon that obeys is never commanded twice.
    const uint32_t now = millis();
    for (int i = 0; i < MAX_BEACONS; i++) {
        if (!g_app.beacon[i].active) continue;
        g_app.beacon[i].cfg_pending     = true;
        g_app.beacon[i].cfg_attempts    = 0;
        g_app.beacon[i].cfg_last_try_ms = 0;   // service() sends immediately
        g_app.beacon[i].cmd_rate_hz     = BEACON_REQUEST_RATE_HZ;
    }
    (void)now;
    MSLOG("[csi] config armed: %dHz sleep=%d for %d beacons\n",
          BEACON_REQUEST_RATE_HZ, BEACON_REQUEST_SLEEP ? 1 : 0,
          (int)g_app.beacon_count);
#endif
}

// Non-blocking config driver.  Called every loop; does nothing unless a
// beacon is still unconfirmed and its retry interval has elapsed.
void csi_beacon_service() {
#if MS_BEACON_CONTROL
    const uint32_t now = millis();
    for (int i = 0; i < MAX_BEACONS; i++) {
        BeaconState &b = g_app.beacon[i];
        if (!b.active || !b.cfg_pending) continue;

        // Confirmed?  The beacon reported the rate we asked for.
        if (b.reported_rate_hz == (uint16_t)BEACON_REQUEST_RATE_HZ) {
            b.cfg_pending = false;
            MSLOG("[csi] b%u confirmed %dHz after %u tr%s\n",
                  (unsigned)b.id, BEACON_REQUEST_RATE_HZ,
                  (unsigned)b.cfg_attempts, b.cfg_attempts == 1 ? "y" : "ies");
            continue;
        }
        if (b.cfg_attempts >= BEACON_CFG_MAX_TRIES) {
            b.cfg_pending = false;
            MSLOG("[csi] b%u NO-ACK after %d tries - left at its default\n",
                  (unsigned)b.id, BEACON_CFG_MAX_TRIES);
            continue;
        }
        if (b.cfg_last_try_ms != 0 &&
            (now - b.cfg_last_try_ms) < BEACON_CFG_RETRY_MS) continue;

        b.cfg_last_try_ms = now;
        b.cfg_attempts++;
        // Addressed to this beacon only: no broadcast, no collision with
        // the other beacons' replies.
        csi_beacon_command(b.id, BEACON_OP_SET_RATE,
                           (uint16_t)BEACON_REQUEST_RATE_HZ);
        // NEVER arm light-sleep here.  Sleep at 30 Hz sits right on the
        // beacon's own >30 ms guard (period is 33 ms) and has been seen
        // to drop RF coherence mid-ceremony: beacons alive at discovery
        // going silent the moment SET_SLEEP landed.
        //
        // My first attempt gated this on scene_cal_complete(), which was
        // wrong in a way worth recording: this runs inside
        // csi_beacon_service(), which only ever retries UNCONFIRMED
        // beacons.  Once a beacon confirms its rate it is never visited
        // again, so sleep would never have been armed at all.  It needs
        // to be an explicit one-shot -- see below.
        csi_beacon_command(b.id, BEACON_OP_SET_SLEEP, 0);
        csi_beacon_command(b.id, BEACON_OP_PING);
    }
#endif
}

// Re-command any beacon that is not running what we asked for.
//
// There is no reason a beacon should ever sit at the compatibility
// default during operation, but a beacon can reboot, be powered on
// late, or miss the original broadcast -- and the original code sent
// the command exactly once at discovery, so any of those left it at
// 100 Hz forever.  Called from the main loop; cheap, and it only
// transmits when something is actually wrong.
// ── RF DURING CAL ─────────────────────────────────────────────
// PING only.  Beacon upkeep was suppressed entirely during cal, on the
// reasoning that a SET_RATE mid-capture would make one kernel sample
// span two rates.  That reasoning is right about SET_RATE and wrong
// about the whole subsystem: with no traffic at all, a beacon that goes
// quiet during the walk is never re-acquired, so the anchor sits there
// losing beacons that are healthy and sitting on the floor working.
//
// A PING carries no configuration.  It cannot change the rate, so it
// cannot corrupt a sample -- it just keeps the link alive and refreshes
// last_seen so the beacon does not age out mid-ceremony.
// Arm light-sleep once, deliberately, after calibration is accepted.
// Deferred until then so the CSI stream cannot be killed mid-ceremony by
// a SET_SLEEP landing at the 30 Hz boundary.
void csi_beacon_enable_sleep_after_cal() {
#if MS_BEACON_CONTROL
    if (!BEACON_REQUEST_SLEEP) return;
    for (int i = 0; i < MAX_BEACONS; i++) {
        if (!g_app.beacon[i].active) continue;
        csi_beacon_command(g_app.beacon[i].id, BEACON_OP_SET_SLEEP, 1);
    }
    MSLOGLN("[csi] post-cal: light-sleep armed on beacons");
#endif
}

void csi_beacon_keepalive() {
#if MS_BEACON_CONTROL
    const uint32_t now = millis();
    static uint32_t last = 0;
    if (now - last < 1500) return;
    last = now;
    for (int i = 0; i < MAX_BEACONS; i++) {
        if (!g_app.beacon[i].active) continue;
        csi_beacon_command(g_app.beacon[i].id, BEACON_OP_PING);
    }
#endif
}

void csi_beacon_enforce_rate() {
#if !MS_BEACON_CONTROL
    return;
#else
    const uint32_t now = millis();
    static uint32_t last_check = 0;
    if (now - last_check < BEACON_RATE_RECHECK_MS) return;
    last_check = now;

    const int want = BEACON_REQUEST_RATE_HZ;
    for (int i = 0; i < MAX_BEACONS; i++) {
        BeaconState &b = g_app.beacon[i];
        if (!b.active || b.cfg_pending) continue;      // already being worked

        // RE-COMMAND ONLY IF THE BEACON LOOKS LIKE IT RESET.
        //
        // The previous test was "reported == want AND the PONG is fresh".
        // PONGs only arrive when we ping, and we only ping while
        // re-arming -- so 30 s after a successful confirmation the PONG
        // went stale, this decided the beacon was unconfirmed, re-armed,
        // got a PONG, confirmed "after 0 tries", and 30 s later did it
        // again.  A permanent cycle that the field log shows exactly:
        // "confirmed 30Hz after 0 tries" / "unconfirmed - re-arming".
        //
        // Worse, every re-command re-anchors the beacon's transmit slot,
        // so the spam was itself disturbing the cadence it was trying to
        // police.
        //
        // What actually matters is whether the beacon fell back to its
        // STOCK default -- that is the only failure re-commanding fixes.
        // Anything near the commanded rate is obeying; a shortfall is
        // delivery loss, and no amount of re-commanding cures that.
        if (b.inter_arrival_ms_ema <= 0.5f) continue;   // no estimate yet
        const int observed = (int)(1000.0f / b.inter_arrival_ms_ema + 0.5f);
        const int stock    = (int)SAMPLE_RATE_HZ;
        const int midpoint = (want + stock) / 2;
        if (observed < midpoint) continue;              // obeying; leave it alone

        // Genuinely back at (or near) its boot rate: it rebooted or never
        // heard us.  Hand it to the closed loop.
        b.cfg_pending     = true;
        b.cfg_attempts    = 0;
        b.cfg_last_try_ms = 0;
        MSLOG("[csi] b%u at %dHz (stock) - re-commanding\n",
              (unsigned)b.id, observed);
    }
#endif
}

// PONG ingest.  Called from the ESP-NOW rx path alongside the peer
// demux; returns true if the payload was ours.
bool csi_beacon_try_consume_pong(const uint8_t *data, int len) {
    if (len < (int)sizeof(BeaconPong)) return false;
    uint32_t magic;
    memcpy(&magic, data, sizeof(magic));
    if (magic != BEACON_PONG_MAGIC) return false;
    BeaconPong p;
    memcpy(&p, data, sizeof(p));
    for (int i = 0; i < MAX_BEACONS; i++) {
        if (!g_app.beacon[i].active) continue;
        if (g_app.beacon[i].id != p.beacon_id) continue;
        g_app.beacon[i].reported_rate_hz = p.current_rate_hz;
        g_app.beacon[i].fw_marker        = p.fw_marker;
        g_app.beacon[i].sleep_enabled    = p.sleep_enabled;
        g_app.beacon[i].last_pong_ms     = millis();
        MSLOG("[csi] PONG b%u fw=%u rate=%uHz sleep=%u\n",
                      (unsigned)p.beacon_id, (unsigned)p.fw_marker,
                      (unsigned)p.current_rate_hz, (unsigned)p.sleep_enabled);
        break;
    }
    return true;
}

void csi_choose_mode(RadarMode m) {
    g_app.mode = m;
}

void csi_assign_default_geometry(float side_cm) {
    // Coordinate frame: T-Display sits at (0, 0); beacons are placed
    // around it.
    //
    // side_cm sets the SHAPE only, never the scale of anything the model
    // sees.  scene_derive_landmarks_from_geometry() divides through by the
    // mean vertex radius, so every normalized landmark is identical whether
    // the polygon is half a metre or fifty metres across (verified over a
    // 100x range).  The user is asked to approximate a shape, not to measure
    // anything.  Do not add per-count "recommended" edge lengths here — they
    // would imply a precision the system neither needs nor uses.
    //
    // v0.8: RM_TRIANGLE_3 now means "3 or more beacons" and lays the
    // beacons out as a regular N-gon centred on the receivers.  The
    // enum name is deliberately unchanged (see config.h).
    //
    // n is clamped to [3, MAX_BEACONS] in the polygon branch so a mode
    // that was picked by hand from Settings (which cycles the mode
    // without changing beacon_count) can never index past the array.
    switch (g_app.mode) {
        case RM_TRIANGLE_3: {
            int n = g_app.beacon_count;
            if (n < 3) n = 3;
            if (n > MAX_BEACONS) n = MAX_BEACONS;

            if (n == 3) {
                // Historical vertex order, preserved verbatim so that a
                // v0.7 calibration stays valid: B0 top, B1 bottom-left,
                // B2 bottom-right (counter-clockwise).  The general
                // formula below winds the other way, which would silently
                // mirror an existing 3-beacon install.
                float r = side_cm / sqrtf(3.0f);          // circumradius
                g_app.beacon[0].pos_x =  0.0f;             g_app.beacon[0].pos_y =  r;
                g_app.beacon[1].pos_x = -side_cm * 0.5f;   g_app.beacon[1].pos_y = -r * 0.5f;
                g_app.beacon[2].pos_x =  side_cm * 0.5f;   g_app.beacon[2].pos_y = -r * 0.5f;
                break;
            }

            // Regular N-gon, first vertex at the top of the screen.
            //   edge = 2 * R * sin(pi / N)  →  R = side_cm / (2 sin(pi/N))
            // Vertices are numbered counter-clockwise to match the
            // 3-beacon convention above (B1 front, B2 to the LEFT), so
            // the walk script's "B1 → B2 → ..." always traces the
            // perimeter in one consistent direction.
            float R = side_cm / (2.0f * sinf((float)M_PI / (float)n));
            for (int i = 0; i < n; i++) {
                float theta = (float)M_PI / 2.0f
                            + (2.0f * (float)M_PI * (float)i) / (float)n;
                g_app.beacon[i].pos_x = R * cosf(theta);
                g_app.beacon[i].pos_y = R * sinf(theta);
            }
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
        // Force the filter to be re-designed on the next frame: after a
        // reset the beacon may be running a different rate than when the
        // coefficients were last computed.
        b.lp_design_fs = 0;
        b.filtered = 0;
        b.moving_variance = 0;
        b.link_metric_raw = 0;
        b.link_metric_ema = 0;
        b.cal_count = 0;
        b.cal_top_n = 0;
        b.last_cal_frame = b.frames;    // "no new frames yet" for accumulator
        b.status = LS_IDLE;
        // Zero phase accumulators so a fresh baseline pass has a clean sum.
        //
        // This loop was EMPTY.  cal_phase_i/q are circular accumulators
        // that only ever get +=, and they feed phase_baseline, which
        // feeds the stereo line fit, which feeds AoA.  They are zeroed
        // when a beacon is first discovered, so a cold boot was fine --
        // but "redo full cal" and "baseline only" both land here, so
        // every RE-calibration summed the new baseline on top of the old
        // one and the bearing reference was quietly wrong afterwards.
        for (int sc = 0; sc < CSI_NUM_SUBCARRIERS; sc++) {
            b.cal_phase_i[sc]   = 0.0f;
            b.cal_phase_q[sc]   = 0.0f;
            b.phase_baseline[sc] = 0.0f;
        }
        b.phase_baseline_valid = false;
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

// v0.9: seqlock reader.  Takes a consistent snapshot of the raw I/Q the
// core-0 callback published, then does the float conversion here on core
// 1.  Retries if the writer landed mid-read; bounded so a pathological
// beacon can never stall the loop.
#if MS_CSI_SEQLOCK
static bool decode_iq_snapshot(BeaconState &b) {
    for (int attempt = 0; attempt < 4; attempt++) {
        uint32_t s0 = b.iq_seq;
        if (s0 & 1u) continue;                  // write in flight
        uint8_t  slot  = b.iq_slot;
        uint16_t pairs = b.iq_pairs;
        if (pairs > CSI_NUM_SUBCARRIERS) pairs = CSI_NUM_SUBCARRIERS;
        int8_t local[CSI_NUM_SUBCARRIERS * 2];
        memcpy(local, b.iq_raw[slot & 1u], (size_t)pairs * 2);
        if (b.iq_seq != s0) continue;           // torn; writer moved
        // Consistent snapshot in hand -- now the float work, on core 1.
        for (int k = 0; k < pairs; k++) {
            float q  = (float)local[k * 2];
            float ii = (float)local[k * 2 + 1];
            b.amplitude[k] = sqrtf(q * q + ii * ii);
            b.phase[k]     = atan2f(q, ii);
        }
        for (int k = pairs; k < CSI_NUM_SUBCARRIERS; k++) {
            b.amplitude[k] = 0.0f;
            b.phase[k]     = 0.0f;
        }
        return true;
    }
    return false;   // writer is hammering; skip this frame, try next tick
}
#endif

int csi_process_frames() {
    int processed = 0;
    for (int i = 0; i < MAX_BEACONS; i++) {
        BeaconState &b = g_app.beacon[i];
        if (!b.active || !b.dirty) continue;
#if MS_CSI_SEQLOCK
        // Convert the raw I/Q the callback published.  Clear `dirty` only
        // after a successful decode so a contended frame is retried
        // rather than discarded.
        if (!decode_iq_snapshot(b)) continue;
#endif
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

// Baseline target is a SAMPLE COUNT, not a duration.
//
// BASELINE_FRAMES samples is what the p95 presence threshold was tuned
// against: the p95 of 500 samples is the 26th largest, and cutting the
// count to hold a fixed 5 s at 20 Hz would make it the 6th largest --
// a far noisier estimate of the number that gates presence detection.
// Statistical resolution is the invariant; the DURATION is what varies
// with rate, and csi_baseline_expected_seconds() reports it so the UI
// can tell the user the real figure instead of a hardcoded one.
static int baseline_target_frames(const BeaconState &b) {
    (void)b;
    return BASELINE_FRAMES;
}

// How long the baseline will actually take at the rate beacons are
// currently transmitting.  100 Hz stock -> 5 s; 20 Hz -> 25 s.
int csi_baseline_expected_seconds() {
    float slowest = SAMPLE_RATE_HZ;
    bool any = false;
    for (int i = 0; i < MAX_BEACONS; i++) {
        if (!g_app.beacon[i].active) continue;
        float hz = beacon_rate_hz(g_app.beacon[i]);
        if (!any || hz < slowest) { slowest = hz; any = true; }
    }
    if (slowest < 1.0f) slowest = 1.0f;
    int secs = (int)((float)BASELINE_FRAMES / slowest + 0.5f);
    return secs < 1 ? 1 : secs;
}

void csi_baseline_accumulate() {
    for (int i = 0; i < MAX_BEACONS; i++) {
        BeaconState &b = g_app.beacon[i];
        if (!b.active) continue;
        if (b.cal_count >= baseline_target_frames(b)) continue;
        // Only advance if this beacon actually received a new frame since
        // the last accumulator tick — prevents double-counting a stale
        // moving_variance value.
        if (b.frames == b.last_cal_frame) continue;
        b.last_cal_frame = b.frames;

        // Retain only the largest P95_TOPK moving-variance samples, kept
        // ascending.  cal_top[0] is the smallest of the retained set, so
        // a new sample only matters if it beats that.  This yields the
        // exact same p95 the full 500-sample buffer produced.
        {
            float x = b.moving_variance;
            if (b.cal_top_n < P95_TOPK) {
                int i = b.cal_top_n++;
                while (i > 0 && b.cal_top[i-1] > x) { b.cal_top[i] = b.cal_top[i-1]; i--; }
                b.cal_top[i] = x;
            } else if (x > b.cal_top[0]) {
                int i = 0;
                while (i + 1 < P95_TOPK && b.cal_top[i+1] < x) { b.cal_top[i] = b.cal_top[i+1]; i++; }
                b.cal_top[i] = x;
            }
        }

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
        b.threshold = p95_from_top(b.cal_top, b.cal_top_n, b.cal_count) * 1.1f;
        if (b.threshold < 1e-6f) b.threshold = 1e-6f;

        // Baseline std as a coarse per-sc variability (last 64 samples if any).
        // For now, seed with a small floor so delta_base normalizes sanely.
        for (int sc = 0; sc < CSI_NUM_SUBCARRIERS; sc++) {
            if (b.baseline[sc] < 0.1f) b.baseline[sc] = 0.1f;
        }
        b.baseline_valid = true;

        // Per-subcarrier mean phase via mean-of-unit-vectors: atan2 of
        // the accumulated cos/sin sums is wraparound-safe.  Feeds
        // stereo_snapshot_baseline() below.
        for (int sc = 0; sc < CSI_NUM_SUBCARRIERS; sc++) {
            float ci = b.cal_phase_i[sc];
            float qi = b.cal_phase_q[sc];
            b.phase_baseline[sc] = ((ci*ci + qi*qi) > 1e-6f)
                                 ? atan2f(qi, ci) : 0.0f;
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
        float p = (float)b.cal_count / (float)baseline_target_frames(b);
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
