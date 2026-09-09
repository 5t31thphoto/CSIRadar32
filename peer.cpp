// ═══════════════════════════════════════════════════════════════
//  peer.cpp
// ═══════════════════════════════════════════════════════════════
#include "peer.h"
#include "stereo.h"
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <string.h>

static const uint8_t BROADCAST_MAC[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// Own MAC snapshot (STA interface)
static uint8_t s_own_mac[6] = {0};

// Peer registered in esp_now peer table?
static bool s_peer_added = false;
// Broadcast registered?
static bool s_broadcast_added = false;

static uint32_t s_discovery_start_ms = 0;

// ── helpers ────────────────────────────────────────────────────
static void add_broadcast_peer_once() {
    if (s_broadcast_added) return;
    esp_now_peer_info_t p = {};
    memcpy(p.peer_addr, BROADCAST_MAC, 6);
    p.channel = CSI_CHANNEL;
    p.ifidx   = WIFI_IF_STA;
    p.encrypt = false;
    if (esp_now_add_peer(&p) == ESP_OK) s_broadcast_added = true;
}

static void add_unicast_peer(const uint8_t mac[6]) {
    if (s_peer_added) {
        esp_now_del_peer(g_app.peer.peer_mac);
        s_peer_added = false;
    }
    esp_now_peer_info_t p = {};
    memcpy(p.peer_addr, mac, 6);
    p.channel = CSI_CHANNEL;
    p.ifidx   = WIFI_IF_STA;
    p.encrypt = false;
    if (esp_now_add_peer(&p) == ESP_OK) {
        memcpy(g_app.peer.peer_mac, mac, 6);
        s_peer_added = true;
    }
}

static int cmp_mac(const uint8_t a[6], const uint8_t b[6]) {
    return memcmp(a, b, 6);
}

// ── lifecycle ──────────────────────────────────────────────────
void peer_begin() {
    // Snapshot our own MAC.
    WiFi.macAddress(s_own_mac);
    memcpy(g_app.peer.own_mac, s_own_mac, 6);

    g_app.peer.role                = ROLE_UNKNOWN;
    g_app.peer.peer_present        = false;
    g_app.peer.last_peer_seen_ms   = 0;
    g_app.peer.last_hello_tx_ms    = 0;
    g_app.peer.peer_frames_rx      = 0;
    g_app.peer.peer_frames_dropped = 0;
    g_app.peer.peer_pairs_ok       = 0;
    g_app.peer.lo_drift_rad        = 0;
    g_app.peer.lo_drift_ema        = 0;
    g_app.peer.last_pair_ms        = 0;
    g_app.peer.primary_state_hint  = 0;

    add_broadcast_peer_once();
}

void peer_start_discovery() {
    s_discovery_start_ms = millis();
    g_app.peer.peer_present      = false;
    g_app.peer.role              = ROLE_UNKNOWN;
    g_app.peer.last_peer_seen_ms = 0;
    Serial.println("[peer] discovery: broadcasting HELLO");
}

bool peer_discovery_done() {
    return (millis() - s_discovery_start_ms) >= PEER_DISCOVERY_MS;
}

void peer_resolve_role() {
    if (!g_app.peer.peer_present) {
        g_app.peer.role = ROLE_SOLO;
        Serial.println("[peer] no peer discovered → SOLO");
        return;
    }
    // Lower MAC = PRIMARY (deterministic tiebreak)
    if (cmp_mac(s_own_mac, g_app.peer.peer_mac) < 0) {
        g_app.peer.role = ROLE_PRIMARY;
        add_unicast_peer(g_app.peer.peer_mac);
        Serial.println("[peer] role = PRIMARY");
    } else {
        g_app.peer.role = ROLE_SECONDARY;
        add_unicast_peer(g_app.peer.peer_mac);
        Serial.println("[peer] role = SECONDARY");
    }
}

void peer_tick() {
    uint32_t now = millis();

    // Heartbeat HELLO — during discovery every ~200ms so partners find each
    // other fast; after discovery every PEER_HEARTBEAT_MS as a keep-alive.
    bool discovering = (g_app.state == ST_PEER_DISCOVERY);
    uint32_t period = discovering ? 200 : PEER_HEARTBEAT_MS;
    if ((now - g_app.peer.last_hello_tx_ms) >= period) {
        peer_send_hello();
        g_app.peer.last_hello_tx_ms = now;
    }

    // Peer presence timeout
    if (g_app.peer.peer_present
        && (now - g_app.peer.last_peer_seen_ms) > PEER_TIMEOUT_MS) {
        Serial.println("[peer] TIMEOUT — peer lost");
        g_app.peer.peer_present = false;
    }
}

// ── senders ────────────────────────────────────────────────────
void peer_send_hello() {
    PeerHelloPacket p = {};
    p.magic = PEER_HELLO_MAGIC;
    memcpy(p.own_mac, s_own_mac, 6);
    p.uptime_ms = millis();
    memcpy(p.fw_version, FW_VERSION, sizeof(p.fw_version));
    p.role_wanted = (uint8_t)g_app.peer.role;
    add_broadcast_peer_once();
    esp_now_send(BROADCAST_MAC, (const uint8_t*)&p, sizeof(p));
}

void peer_send_csi_summary(const PeerCsiSummary &pkt) {
    if (!s_peer_added) return;   // no unicast peer yet
    esp_now_send(g_app.peer.peer_mac, (const uint8_t*)&pkt, sizeof(pkt));
}

void peer_send_command(uint8_t op, uint8_t a8, uint16_t a16, uint32_t a32) {
    if (!s_peer_added) return;
    PeerCommand c = {};
    c.magic   = PEER_CMD_MAGIC;
    c.op      = op;
    c.arg_u8  = a8;
    c.arg_u16 = a16;
    c.arg_u32 = a32;
    esp_now_send(g_app.peer.peer_mac, (const uint8_t*)&c, sizeof(c));
}

void peer_send_baseline(const PeerBaselinePacket &pkt) {
    if (!s_peer_added) return;
    esp_now_send(g_app.peer.peer_mac, (const uint8_t*)&pkt, sizeof(pkt));
}

void peer_send_cal_observation(const PeerCalObservation &pkt) {
    if (!s_peer_added) return;
    esp_now_send(g_app.peer.peer_mac, (const uint8_t*)&pkt, sizeof(pkt));
}

// Forward to scene module.  Declared here (not #included) to avoid
// pulling scene.h into peer.cpp — keeps the dep graph one-way.
extern void scene_ingest_peer_cal(const PeerCalObservation &pkt);

void peer_handle_cal_observation(const PeerCalObservation &pkt) {
    g_app.peer.last_peer_seen_ms = millis();
    scene_ingest_peer_cal(pkt);
}

// ── receivers (called from csi.cpp ESP-NOW cb) ─────────────────
void peer_handle_hello(const uint8_t *src_mac, const PeerHelloPacket &pkt) {
    // Ignore hellos from ourselves (bounce-back on some coex configs).
    if (memcmp(src_mac, s_own_mac, 6) == 0) return;
    if (!g_app.peer.peer_present) {
        memcpy(g_app.peer.peer_mac, src_mac, 6);
        g_app.peer.peer_present = true;
        Serial.printf("[peer] discovered %02X:%02X:%02X:%02X:%02X:%02X\n",
                      src_mac[0], src_mac[1], src_mac[2],
                      src_mac[3], src_mac[4], src_mac[5]);
    }
    g_app.peer.last_peer_seen_ms = millis();
}

void peer_handle_csi_summary(const PeerCsiSummary &pkt) {
    g_app.peer.peer_frames_rx++;
    g_app.peer.last_peer_seen_ms = millis();
    // Forward to stereo module to attempt pairing.
    stereo_ingest_peer_summary(pkt);
}

void peer_handle_command(const PeerCommand &cmd) {
    g_app.peer.last_peer_seen_ms = millis();
    switch (cmd.op) {
        case PEER_OP_ENTER_STREAMING:
            // Legacy — v0.3 uses full state hints instead.
            g_app.peer.primary_state_hint = ST_DASHBOARD;
            break;
        case PEER_OP_RECALIBRATE:
            // v0.3 re-cal: start from CAL_INTRO so the ceremony runs
            // in full (empty-room + walk).
            g_app.peer.primary_state_hint = ST_CAL_INTRO;
            break;
        case PEER_OP_SLEEP:
            g_app.peer.primary_state_hint = ST_SLEEP_ARM;
            break;
        case PEER_OP_STATE_HINT:
            g_app.peer.primary_state_hint = cmd.arg_u8;
            break;
        case PEER_OP_CAL_STEP_HINT: {
            // PROBE→ANCHOR: PROBE has advanced its wizard to step
            // cmd.arg_u16.  Sync our local wizard to match.  We forward
            // to wizard via a forward-declared function so peer.cpp
            // doesn't need to include wizard.h.
            extern void wizard_jump_to_step(int idx);
            wizard_jump_to_step((int)cmd.arg_u16);
            break;
        }
        case PEER_OP_CAL_BEGIN:
        case PEER_OP_CAL_END:
            // Informational — scene.cpp on ANCHOR emits these to mark
            // capture windows for PROBE's benefit.  PROBE currently
            // captures unconditionally so we ignore; hook exists for
            // future refinement.
            break;
        default: break;
    }
}

void peer_handle_baseline(const PeerBaselinePacket &pkt) {
    g_app.peer.last_peer_seen_ms = millis();
    // Forward to stereo module to stash and (if primary local baseline is
    // already valid) fold into the disparity baseline for this beacon.
    stereo_ingest_peer_baseline(pkt);
}

bool peer_try_consume(const uint8_t *src_mac, const uint8_t *data, int len) {
    if (len < (int)sizeof(uint32_t)) return false;
    uint32_t magic;
    memcpy(&magic, data, sizeof(magic));
    switch (magic) {
        case PEER_HELLO_MAGIC:
            if (len >= (int)sizeof(PeerHelloPacket)) {
                PeerHelloPacket p;
                memcpy(&p, data, sizeof(p));
                peer_handle_hello(src_mac, p);
            }
            return true;
        case PEER_FRAME_MAGIC:
            if (len >= (int)sizeof(PeerCsiSummary)) {
                PeerCsiSummary p;
                memcpy(&p, data, sizeof(p));
                peer_handle_csi_summary(p);
            }
            return true;
        case PEER_CMD_MAGIC:
            if (len >= (int)sizeof(PeerCommand)) {
                PeerCommand c;
                memcpy(&c, data, sizeof(c));
                peer_handle_command(c);
            }
            return true;
        case PEER_BASE_MAGIC:
            if (len >= (int)sizeof(PeerBaselinePacket)) {
                PeerBaselinePacket p;
                memcpy(&p, data, sizeof(p));
                peer_handle_baseline(p);
            }
            return true;
        case PEER_CAL_MAGIC:
            if (len >= (int)sizeof(PeerCalObservation)) {
                PeerCalObservation p;
                memcpy(&p, data, sizeof(p));
                peer_handle_cal_observation(p);
            }
            return true;
        default:
            return false;
    }
}
