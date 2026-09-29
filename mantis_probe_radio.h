#pragma once
// ═══════════════════════════════════════════════════════════════
//  MANTIS PROBE RADIO
//  One radio implementation for every hand-held receiver
// ═══════════════════════════════════════════════════════════════
//
//  The Core2 and the Cardputer are the same receiver in different
//  cases, so they get one radio layer rather than two.  Without this
//  both devices boot, render a full UI, and detect nothing -- every
//  structure on screen populated by nobody.
//
//  Three jobs:
//
//    1. HEAR THE BEACONS.  ESP-NOW receive feeds mantis_rx_packet(),
//       which is where perspectives, dense reports and the surveyed
//       geometry all arrive.
//
//    2. MEASURE THE CHANNEL.  The CSI callback gives this device its
//       OWN view of each beacon -- the links a fixed anchor cannot see
//       from where it sits, and the ones that move as the operator
//       walks.
//
//    3. DRIVE THE ANCHOR.  PeerCmd out, so the operator advances the
//       calibration script from their hand instead of walking back to
//       the shelf.
//
//  ── THE CALLBACK DOES NOTHING BUT COPY ───────────────────────
//
//  Same discipline as the beacon and the T-Display: the Wi-Fi task
//  memcpys and returns.  Parsing inside it adds latency to the radio
//  path, and on a device that is also driving a 320x240 redraw that
//  latency lands on frames that were about to be measured.
//
//  ── CHANNEL IS FIXED AND SO IS RATE ──────────────────────────
//
//  A probe that roams channels stops hearing the mesh, and rate
//  adaptation would make RSSI incomparable between devices -- which
//  would break the geometry survey the beacons are publishing.
// ═══════════════════════════════════════════════════════════════
#include "mantis_air.h"
#include "mantis_receiver.h"
#include "mantis_control.h"
// MBC_SC_MAX sizes the CSI buffer below.  Missing this compiled fine on
// a host build because the whole body lives behind #if defined(ARDUINO)
// -- the host only ever saw the empty else-branch, so the header
// "passed" every standalone check while being broken on every device.
#include "mantis_beacon_csi.h"

#if defined(ARDUINO)
  #include <WiFi.h>
  #include <esp_now.h>
  #include <esp_wifi.h>
  #include <esp_timer.h>

static const uint8_t MANTIS_BCAST[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// Raw inbound packet, handed from the Wi-Fi task to loop().
//
// Seqlock rather than a mutex: the callback must never block, and a
// torn read is detectable and cheap to discard.  A dropped packet costs
// one frame in thirty; a blocked callback costs the radio.
typedef struct {
    volatile uint32_t seq;      // odd = write in progress
    uint8_t  data[192];
    uint16_t len;
    uint8_t  from_id;
    bool     pending;
} MantisRawPkt;

#define MANTIS_PROBE_QUEUE 8
static MantisRawPkt s_q[MANTIS_PROBE_QUEUE];
static volatile uint8_t s_q_head = 0;

// This device's own CSI to each beacon.
typedef struct {
    volatile uint32_t seq;
    int8_t   buf[MBC_SC_MAX * 2];
    int      len;
    int8_t   rssi, noise;
    bool     fresh;
} MantisProbeCsi;
static MantisProbeCsi s_csi[MANTIS_SLOTS];

static uint8_t mantis_probe_id_from_mac(const uint8_t *mac) {
    return mantis_beacon_id_from_mac(mac);      // 1A:00:uid:uid:00:id
}

// The anchor's latest status.  Same seqlock discipline as the beacon
// queue: the Wi-Fi task copies and leaves.
typedef struct {
    volatile uint32_t seq;
    uint8_t  data[sizeof(MantisAnchorStatus)];
    bool     pending;
} MantisAnchorSlot;
static MantisAnchorSlot s_anchor_rx;

static void mantis_probe_now_cb(const esp_now_recv_info_t *info,
                                const uint8_t *data, int len) {
    if (!info || len <= 0 || len > (int)sizeof(s_q[0].data)) return;
    const uint8_t from = mantis_probe_id_from_mac(info->src_addr);
    if (from == 0) {
        // Not a beacon.  The ANCHOR's status broadcast is the one other
        // thing a probe needs to hear -- the old radio dropped every
        // non-beacon frame, so no probe ever heard its anchor at all.
        if (len == (int)sizeof(MantisAnchorStatus)) {
            uint32_t magic; memcpy(&magic, data, 4);
            if (magic == MANTIS_ANCHOR_STATUS_MAGIC) {
                s_anchor_rx.seq++;
                memcpy((void *)s_anchor_rx.data, data, sizeof(MantisAnchorStatus));
                s_anchor_rx.pending = true;
                s_anchor_rx.seq++;
            }
        }
        return;
    }

    const uint8_t h = s_q_head;
    MantisRawPkt &p = s_q[h];
    p.seq++;
    memcpy((void *)p.data, data, len);
    p.len     = (uint16_t)len;
    p.from_id = from;
    p.pending = true;
    p.seq++;
    s_q_head = (uint8_t)((h + 1) % MANTIS_PROBE_QUEUE);
}

static void mantis_probe_csi_cb(void *, wifi_csi_info_t *info) {
    if (!info || !info->buf || info->len < 128) return;
    const uint8_t from = mantis_probe_id_from_mac(info->mac);
    if (from == 0) return;
    MantisProbeCsi &c = s_csi[from];
    c.seq++;
    const int n = info->len > (int)sizeof(c.buf) ? (int)sizeof(c.buf) : info->len;
    memcpy((void *)c.buf, info->buf, n);
    c.len   = n;
    c.rssi  = info->rx_ctrl.rssi;
    c.noise = info->rx_ctrl.noise_floor;
    c.fresh = true;
    c.seq++;
}

static inline void mantis_probe_radio_begin() {
    WiFi.mode(WIFI_STA);
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(MANTIS_CHANNEL, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);
    esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_MCS0_LGI);

    if (esp_now_init() != ESP_OK) return;
    esp_now_register_recv_cb(mantis_probe_now_cb);

    esp_now_peer_info_t p{};
    memcpy(p.peer_addr, MANTIS_BCAST, 6);
    p.channel = MANTIS_CHANNEL;
    p.ifidx   = WIFI_IF_STA;
    p.encrypt = false;
    esp_now_add_peer(&p);

    // This device measures the channel too.  A hand-held receiver is a
    // MOVING observer, so its links sweep geometries no fixed anchor
    // can reach -- that is the synthetic aperture the cal walk forms.
    wifi_csi_config_t c{};
    c.lltf_en = true; c.htltf_en = true; c.stbc_htltf2_en = false;
    c.ltf_merge_en = true; c.channel_filter_en = false; c.manu_scale = false;
    esp_wifi_set_csi_config(&c);
    esp_wifi_set_csi_rx_cb(mantis_probe_csi_cb, nullptr);
    esp_wifi_set_csi(true);
}

// Drain the queue into the receiver.  Called every loop.
static inline int mantis_probe_pump(MantisReceiver *rx, uint32_t now_ms) {
    int n = 0;
    for (int i = 0; i < MANTIS_PROBE_QUEUE; i++) {
        MantisRawPkt &p = s_q[i];
        if (!p.pending) continue;
        const uint32_t s0 = p.seq;
        if (s0 & 1u) continue;                 // mid-write; next loop
        static uint8_t local[192];
        const uint16_t len = p.len;
        const uint8_t  from = p.from_id;
        memcpy(local, (const void *)p.data, len);
        if (p.seq != s0) continue;             // torn: discard rather than trust
        p.pending = false;
        if (mantis_rx_packet(rx, from, local, len, now_ms)) n++;
    }
    return n;
}

static inline bool mantis_probe_tx_cmd(MantisProbeOp op, uint8_t a8,
                                       uint16_t a16, uint32_t a32) {
    MantisPeerCmd c{};
    const uint16_t n = mantis_probe_cmd(&c, op, a8, a16, a32);
    return esp_now_send(MANTIS_BCAST, (const uint8_t *)&c, n) == ESP_OK;
}

// Press one of the anchor's buttons.  Broadcast: a docked pair shares
// state over its wire, so whichever unit hears it is the instrument.
static inline void mantis_probe_press(MantisProbeLink *L, MantisRemoteKey k,
                                      uint32_t now_ms) {
    const uint16_t seq = mantis_probe_key_begin(L, k, now_ms);
    mantis_probe_tx_cmd(MPC_REMOTE_KEY, (uint8_t)k, seq, 0);
}

// Everything the link needs, every loop: take in the anchor's status,
// repeat an unconfirmed key press, announce ourselves.
static inline void mantis_probe_service(MantisProbeLink *L, uint8_t kind,
                                        uint32_t now_ms) {
    if (s_anchor_rx.pending) {
        const uint32_t s0 = s_anchor_rx.seq;
        if (!(s0 & 1u)) {
            MantisAnchorStatus st;
            memcpy(&st, (const void *)s_anchor_rx.data, sizeof(st));
            if (s_anchor_rx.seq == s0) {
                s_anchor_rx.pending = false;
                mantis_probe_on_status(L, &st, now_ms);
            }
        }
    }
    if (mantis_probe_key_due(L, now_ms)) {
        L->key_tries++;
        L->key_sent_ms = now_ms;
        // SAME sequence number: the anchor applies each seq once, so a
        // retry of a press that did land is harmless.
        mantis_probe_tx_cmd(MPC_REMOTE_KEY, L->key_code, L->key_seq, 0);
    }
    if (now_ms - L->last_hello_ms >= MANTIS_PROBE_HELLO_MS) {
        L->last_hello_ms = now_ms;
        mantis_probe_tx_cmd(MPC_PROBE_HELLO, kind, 0, 0);
    }
}

// Is this device hearing its own CSI from a given beacon?
static inline bool mantis_probe_csi_fresh(uint8_t beacon_id) {
    return (beacon_id < MANTIS_SLOTS) && s_csi[beacon_id].fresh;
}

#else   // host build: radio is hardware-bound
static inline void mantis_probe_radio_begin() {}
static inline int  mantis_probe_pump(MantisReceiver *, uint32_t) { return 0; }
static inline void mantis_probe_press(MantisProbeLink *L, MantisRemoteKey k, uint32_t now_ms) {
    mantis_probe_key_begin(L, k, now_ms);
}
static inline void mantis_probe_service(MantisProbeLink *, uint8_t, uint32_t) {}
static inline bool mantis_probe_csi_fresh(uint8_t) { return false; }
#endif
