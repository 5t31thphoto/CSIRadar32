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

// ── v0.8: probe undock state ──────────────────────────────────
// ANCHOR side: latest position fix reported by the undocked probe.
static struct {
    float    pos[2];
    float    cov[3];
    float    conf;
    uint32_t ts_ms;
    bool     valid;
} s_probe_pos = {};
// ANCHOR side: probe announced it is mobile (PEER_OP_UNDOCK_PROBE).
static bool s_probe_undocked_remote = false;
// PROBE side: latest track list streamed down from the anchor.
static PeerTrackStatePacket s_track_state = {};
static uint32_t s_track_state_ts_ms = 0;
static bool     s_track_state_valid = false;

// A fix older than this is treated as no fix at all.  Chosen as 5x the
// 10 Hz send period: long enough to ride out a few dropped ESP-NOW
// frames, short enough that a probe which has gone out of range stops
// suppressing tracks on the anchor within half a second.
#define PROBE_POS_STALE_MS  500

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
    MSLOGLN("[peer] discovery: broadcasting HELLO");
}

bool peer_discovery_done() {
    return (millis() - s_discovery_start_ms) >= PEER_DISCOVERY_MS;
}

void peer_resolve_role() {
    if (!g_app.peer.peer_present) {
        g_app.peer.role = ROLE_SOLO;
        MSLOGLN("[peer] no peer discovered → SOLO");
        return;
    }
    // Lower MAC = PRIMARY (deterministic tiebreak)
    if (cmp_mac(s_own_mac, g_app.peer.peer_mac) < 0) {
        g_app.peer.role = ROLE_PRIMARY;
        add_unicast_peer(g_app.peer.peer_mac);
        MSLOGLN("[peer] role = PRIMARY");
    } else {
        g_app.peer.role = ROLE_SECONDARY;
        add_unicast_peer(g_app.peer.peer_mac);
        MSLOGLN("[peer] role = SECONDARY");
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
        MSLOGLN("[peer] TIMEOUT — peer lost");
        g_app.peer.peer_present = false;
    }
}

// ── senders ────────────────────────────────────────────────────
// ═══════════════════════════════════════════════════════════════
//  v0.9 — WIRED TRANSPORT
// ═══════════════════════════════════════════════════════════════
// Everything below feeds the SAME peer_try_consume() demux the radio
// path uses, so no packet handler knows or cares which transport a
// frame arrived on.  One demux, two transports.
static HardwareSerial s_wire(PEER_WIRE_UART_NUM);
static bool     s_wire_open      = false;
static uint32_t s_wire_last_rx   = 0;
static uint32_t s_wire_last_ping = 0;
static uint32_t s_wire_frames_rx = 0;
static uint32_t s_wire_crc_err   = 0;

// CRC-16/CCITT.  A UART on jumper wire will occasionally deliver a
// corrupted byte, and these payloads are raw structs -- a bad frame
// accepted as a PeerCsiSummary would inject a garbage phase reading
// straight into the AoA solution.  Cheaper to check than to debug.
static uint16_t crc16_ccitt(const uint8_t *d, size_t n) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < n; i++) {
        crc ^= (uint16_t)d[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                                 : (uint16_t)(crc << 1);
    }
    return crc;
}

void peer_wire_begin() {
#if !MS_PEER_WIRE
    return;     // flag off: ESP-NOW only, as v0.6
#else
    s_wire.begin(PEER_WIRE_BAUD, SERIAL_8N1,
                 PEER_WIRE_RX_PIN, PEER_WIRE_TX_PIN);
    s_wire.setTimeout(0);
    s_wire_open    = true;
    s_wire_last_rx = 0;
    MSLOG("[peer] wire UART%d up @%d baud (tx=%d rx=%d)\n",
                  PEER_WIRE_UART_NUM, PEER_WIRE_BAUD,
                  PEER_WIRE_TX_PIN, PEER_WIRE_RX_PIN);
#endif
}

bool peer_wire_link_up() {
    return s_wire_open && s_wire_last_rx != 0 &&
           (millis() - s_wire_last_rx) < PEER_WIRE_STALE_MS;
}

PeerTransport peer_active_transport() {
    // The undocked probe has physically unplugged; never claim the wire.
    if (g_app.probe_undocked)  return s_peer_added ? PEER_TX_ESPNOW : PEER_TX_NONE;
    if (peer_wire_link_up())   return PEER_TX_WIRE;
    if (s_peer_added)          return PEER_TX_ESPNOW;
    return PEER_TX_NONE;
}

static void wire_write_frame(const void *payload, size_t len) {
    if (!s_wire_open || len > PEER_WIRE_MAX_PAYLOAD) return;
    uint8_t hdr[4] = { PEER_WIRE_SYNC0, PEER_WIRE_SYNC1,
                       (uint8_t)(len & 0xFF), (uint8_t)(len >> 8) };
    uint16_t crc = crc16_ccitt((const uint8_t*)payload, len);
    uint8_t tail[2] = { (uint8_t)(crc & 0xFF), (uint8_t)(crc >> 8) };
    s_wire.write(hdr, 4);
    s_wire.write((const uint8_t*)payload, len);
    s_wire.write(tail, 2);
}

// Single delivery point for every unicast peer packet.  Prefers the
// wire when it is alive, falls back to the radio otherwise, so pulling
// the cable mid-session degrades instead of breaking.
static void peer_deliver(const void *pkt, size_t len) {
    switch (peer_active_transport()) {
        case PEER_TX_WIRE:   wire_write_frame(pkt, len); break;
        case PEER_TX_ESPNOW: esp_now_send(g_app.peer.peer_mac,
                                          (const uint8_t*)pkt, len); break;
        default: break;
    }
}

// Byte-stream reassembly.  Called every loop; cheap when idle.
void peer_wire_poll() {
#if !MS_PEER_WIRE
    return;
#endif
    if (!s_wire_open) return;
    static uint8_t  buf[PEER_WIRE_MAX_PAYLOAD + 6];
    static int      have = 0;

    while (s_wire.available() > 0) {
        int c = s_wire.read();
        if (c < 0) break;
        if (have < (int)sizeof(buf)) buf[have++] = (uint8_t)c;

        // Resynchronise on the sync word rather than trusting alignment.
        if (have == 1 && buf[0] != PEER_WIRE_SYNC0) { have = 0; continue; }
        if (have == 2 && buf[1] != PEER_WIRE_SYNC1) {
            // buf[1] might itself start a frame; keep it.
            buf[0] = buf[1]; have = 1;
            if (buf[0] != PEER_WIRE_SYNC0) have = 0;
            continue;
        }
        if (have < 4) continue;

        uint16_t len = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);
        if (len == 0 || len > PEER_WIRE_MAX_PAYLOAD) { have = 0; continue; }
        if (have < 4 + len + 2) continue;

        uint16_t want = (uint16_t)buf[4 + len] | ((uint16_t)buf[5 + len] << 8);
        if (want == crc16_ccitt(buf + 4, len)) {
            s_wire_last_rx = millis();
            s_wire_frames_rx++;
            g_app.peer.last_peer_seen_ms = s_wire_last_rx;
            // Same demux as the radio.  src_mac is only consulted for
            // HELLO, and discovery stays on the air, so nullptr is safe.
            peer_try_consume(nullptr, buf + 4, (int)len);
        } else {
            s_wire_crc_err++;
        }
        have = 0;
    }

    // Keepalive so an idle wire still reads as connected.  Cheap: a
    // 12-byte command at ~7 Hz.
    uint32_t now = millis();
    // Only keepalive when a peer actually exists.  Without this a SOLO
    // unit transmits binary frames out GPIO43 at ~7 Hz forever -- and
    // GPIO43 is U0TXD, so anyone who plugs a serial adapter into that
    // JST to debug gets a stream of garbage instead of a console.
    // s_wire_last_rx covers the case where the wire is up before
    // ESP-NOW discovery has resolved a peer.
    bool peer_expected = g_app.peer.peer_present || (s_wire_last_rx != 0);
    if (s_wire_open && peer_expected && !g_app.probe_undocked &&
        (now - s_wire_last_ping) >= PEER_WIRE_PING_MS) {
        s_wire_last_ping = now;
        PeerCommand c = {};
        c.magic = PEER_CMD_MAGIC;
        c.op    = PEER_OP_WIRE_PING;
        c.arg_u32 = now;   // tx timestamp, for future clock-offset work
        wire_write_frame(&c, sizeof(c));
    }
}

void peer_wire_stats(uint32_t *frames, uint32_t *crc_errors) {
    if (frames)     *frames     = s_wire_frames_rx;
    if (crc_errors) *crc_errors = s_wire_crc_err;
}

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
    peer_deliver(&pkt, sizeof(pkt));
}

void peer_send_command(uint8_t op, uint8_t a8, uint16_t a16, uint32_t a32) {
    PeerCommand c = {};
    c.magic   = PEER_CMD_MAGIC;
    c.op      = op;
    c.arg_u8  = a8;
    c.arg_u16 = a16;
    c.arg_u32 = a32;
    peer_deliver(&c, sizeof(c));
}

void peer_send_baseline(const PeerBaselinePacket &pkt) {
    peer_deliver(&pkt, sizeof(pkt));
}

void peer_send_cal_observation(const PeerCalObservation &pkt) {
    peer_deliver(&pkt, sizeof(pkt));
}

void peer_send_probe_position(const PeerProbePositionPacket &pkt) {
    peer_deliver(&pkt, sizeof(pkt));
}

// v0.9: anchor -> probe tripwire mirror.  Only sent in TW_REMOTE, and
// only by the anchor, whose link is the one with fixed geometry.
void peer_send_tripwire(uint8_t status, uint8_t beacon_id, uint16_t pct) {
    PeerTripwirePacket p = {};
    p.magic      = PEER_TRIPWIRE_MAGIC;
    p.status     = status;
    p.beacon_id  = beacon_id;
    p.metric_pct = pct;
    p.stamp_ms   = millis();
    peer_deliver(&p, sizeof(p));
}

bool peer_tripwire_fresh() {
    return g_app.tw_remote_ms != 0 &&
           (millis() - g_app.tw_remote_ms) < PEER_TRIPWIRE_STALE_MS;
}

void peer_send_track_state(const PeerTrackStatePacket &pkt) {
    peer_deliver(&pkt, sizeof(pkt));
}

bool peer_probe_is_undocked() { return s_probe_undocked_remote; }

bool peer_has_recent_probe_position() {
    return s_probe_pos.valid
        && (millis() - s_probe_pos.ts_ms) < PROBE_POS_STALE_MS;
}

bool peer_get_probe_position(float out_pos[2], float out_cov[3], float *out_conf) {
    if (!peer_has_recent_probe_position()) return false;
    out_pos[0] = s_probe_pos.pos[0];
    out_pos[1] = s_probe_pos.pos[1];
    out_cov[0] = s_probe_pos.cov[0];
    out_cov[1] = s_probe_pos.cov[1];
    out_cov[2] = s_probe_pos.cov[2];
    if (out_conf) *out_conf = s_probe_pos.conf;
    return true;
}

const PeerTrackStatePacket *peer_last_track_state() {
    return s_track_state_valid ? &s_track_state : nullptr;
}

uint32_t peer_track_state_age_ms() {
    if (!s_track_state_valid) return 0xFFFFFFFFUL;
    return millis() - s_track_state_ts_ms;
}

void peer_reset_probe_undock() {
    s_probe_undocked_remote = false;
    s_probe_pos.valid       = false;
    s_track_state_valid     = false;
}

// ── v0.8 receive handlers ─────────────────────────────────────
static void peer_handle_probe_position(const PeerProbePositionPacket &pkt) {
    g_app.peer.last_peer_seen_ms = millis();
    // A probe that starts streaming fixes is undocked whether or not we
    // saw its UNDOCK command — that single unicast is unacknowledged and
    // can simply be lost, and losing it must not wedge the anchor into
    // never applying self-suppression.
    s_probe_undocked_remote = true;
    s_probe_pos.pos[0] = pkt.pos_x;
    s_probe_pos.pos[1] = pkt.pos_y;
    s_probe_pos.cov[0] = pkt.cov_xx;
    s_probe_pos.cov[1] = pkt.cov_yy;
    s_probe_pos.cov[2] = pkt.cov_xy;
    s_probe_pos.conf   = pkt.confidence;
    s_probe_pos.ts_ms  = millis();
    s_probe_pos.valid  = true;
}

static void peer_handle_track_state(const PeerTrackStatePacket &pkt) {
    g_app.peer.last_peer_seen_ms = millis();
    memcpy(&s_track_state, &pkt, sizeof(s_track_state));
    s_track_state_ts_ms = millis();
    s_track_state_valid = true;
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
        MSLOG("[peer] discovered %02X:%02X:%02X:%02X:%02X:%02X\n",
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
        case PEER_OP_CAL_ROLE_ANCHOR:
            // The other unit was pressed, so it carries the probe and
            // this one stays put as the anchor.
            g_app.peer.cal_role      = CAL_ROLE_ANCHOR;
            g_app.peer.role_override = RO_FORCE_ANCHOR;
            g_app.cal_mode           = CAL_MODE_STEREO;
            MSLOGLN("[peer] peer took the probe - this unit is the ANCHOR");
            break;
        case PEER_OP_WIRE_PING:
            // Liveness only.  peer_wire_poll() already stamped the link
            // when the frame arrived; nothing further to do.
            break;
        case PEER_OP_UNDOCK_PROBE:
            // PROBE→ANCHOR.  Start streaming track state back to it and
            // begin honouring its position fixes.
            s_probe_undocked_remote = true;
            MSLOGLN("[peer] probe undocked");
            break;
        case PEER_OP_REDOCK_PROBE:
            s_probe_undocked_remote = false;
            s_probe_pos.valid       = false;
            MSLOGLN("[peer] probe redocked");
            break;
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
        case PEER_PROBE_POS_MAGIC:
            if (len >= (int)sizeof(PeerProbePositionPacket)) {
                PeerProbePositionPacket p;
                memcpy(&p, data, sizeof(p));
                peer_handle_probe_position(p);
            }
            return true;
        case PEER_TRACK_STATE_MAGIC:
            if (len >= (int)sizeof(PeerTrackStatePacket)) {
                PeerTrackStatePacket p;
                memcpy(&p, data, sizeof(p));
                peer_handle_track_state(p);
            }
            return true;
        case PEER_TRIPWIRE_MAGIC:
            if (len >= (int)sizeof(PeerTripwirePacket)) {
                PeerTripwirePacket p;
                memcpy(&p, data, sizeof(p));
                g_app.tw_remote_ms     = millis();
                g_app.tw_remote_status = p.status;
                g_app.tw_remote_beacon = p.beacon_id;
                g_app.tw_remote_pct    = p.metric_pct;
            }
            return true;
        default:
            return false;
    }
}
