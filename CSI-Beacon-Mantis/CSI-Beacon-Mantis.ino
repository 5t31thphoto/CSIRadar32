// ═══════════════════════════════════════════════════════════════
//  MANTIS SMART MESH BEACON  —  v1.0
// ═══════════════════════════════════════════════════════════════
//
//  This replaces the free-running broadcaster.  A beacon is now a node
//  in a self-organising mesh that keeps its own time, hears every other
//  beacon, reduces what it hears to a sufficient statistic, and reports
//  that statistic in the payload of a transmission it was making anyway.
//
//  ── WHAT IT RUNS ─────────────────────────────────────────────
//
//    mantis_sched       absolute-deadline slot scheduler
//    mantis_membership  slot = id-1, timekeeper = lowest live id,
//                       duplicate-id detection with yield
//    mantis_program     SOLO / CHORD / CHORUS / DENSE / SLEEP
//    mantis_beacon_csi  CSI -> amplitude + fit-intercept phase
//    mantis_report      idempotent perspective records
//    mantis_dense       subcarrier profile, delay spread, micro-Doppler
//
//  ── BACKWARD COMPATIBLE BY CONSTRUCTION ──────────────────────
//
//  Every payload leads with the uint32 counter the existing receiver
//  reads.  A receiver running old firmware sees exactly what it saw
//  before and keeps working; a new receiver reads past it.  So the mesh
//  can be deployed one beacon at a time.
//
//  ── THE MESH DOES NOT NEED A RECEIVER ────────────────────────
//
//  Nothing here waits for an anchor.  Beacons sync to each other, elect
//  their own timekeeper and report whether or not anyone is listening.
//  That is what makes "Cardputer + 3 beacons, no anchor" a real
//  deployment rather than a degraded one.
// ═══════════════════════════════════════════════════════════════
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_timer.h>
// esp_read_mac() and ESP_MAC_WIFI_STA are in esp_mac.h, NOT esp_wifi.h.
// Without this the build fails with "ESP_MAC_WIFI_STA was not declared;
// did you mean ESP_IF_WIFI_STA" -- a different enum entirely, which
// would have compiled to the wrong thing had it been accepted.
#include <esp_mac.h>
#include <esp_sleep.h>

#include "mantis_air.h"
#include "mantis_sched.h"
#include "mantis_membership.h"
#include "mantis_program.h"
#include "mantis_report.h"
#include "mantis_beacon_csi.h"
#include "mantis_dense.h"
#include "mantis_identity.h"
#include "mantis_geometry.h"
#include <Preferences.h>

// ── Identity ──────────────────────────────────────────────────
// ONE FIRMWARE FOR EVERY BEACON.  No per-device build, no six bins.
//
// The id is a SCHEDULING RESOURCE claimed from the mesh and then kept:
// listen, take a free one (preferring a MAC-derived guess so six blank
// beacons do not all reach for the same one), persist it, and use it for
// life.  Collisions resolve by MAC comparison in exactly one exchange.
//
// The PHYSICAL BOX is identified separately and permanently by a
// MAC-derived uid, which needs no protocol and no storage.  Everything
// the receiver learns about a location is keyed to that, so a reshuffled
// id cannot silently attach one box's geometry to another.
static MantisIdentity g_id;
static Preferences    g_nvs;
static uint8_t        beacon_id = 0;    // 0 until claimed
static uint16_t       g_uid     = 0;

static const uint8_t BROADCAST[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// ── State ─────────────────────────────────────────────────────
static MantisMembership  g_mesh;
static MantisBeaconLink  g_link[MANTIS_SLOTS];      // one per peer id

// Micro-Doppler, accumulated per link over a sliding window.
//
// This is the channel that separates a person from a fan: a body spreads
// its phase rate across bins because different parts move at different
// speeds, while machinery concentrates in one.  It has to be built up
// over time on the BEACON -- the receiver only ever sees the histogram,
// never the per-frame rates it came from.
// ── MESH SELF-SURVEY ──────────────────────────────────────────
// Every beacon hears every other beacon, so every beacon holds a
// complete pairwise RSSI matrix -- and therefore a distance matrix the
// receiver has no way to obtain.  Solving it here and broadcasting the
// answer is what replaces the assumed ring with a measured layout.
//
// The TIMEKEEPER publishes.  Not an election: it is already the lowest
// live id and every node computes that independently, so making it the
// publisher costs nothing and guarantees exactly one publisher.
static int8_t   g_peer_rssi[MANTIS_SLOTS][MANTIS_SLOTS];   // [hearer][heard]
static uint8_t  g_rssi_n[MANTIS_SLOTS][MANTIS_SLOTS];
static float    g_rssi_f[MANTIS_SLOTS][MANTIS_SLOTS];   // our own row, unrounded
static float    g_geo_x[MANTIS_SLOTS], g_geo_y[MANTIS_SLOTS];
static uint8_t  g_geo_stress = 255;
static uint16_t g_geo_cm     = 0;
static bool     g_geo_ready  = false;
static uint32_t g_geo_last_ms = 0;
static uint32_t g_row_ms[MANTIS_SLOTS];   // when each RSSI row was last refreshed

static uint8_t  g_dop[MANTIS_SLOTS][MANTIS_DENSE_DOP_BINS];
static float    g_prev_phi[MANTIS_SLOTS];
static bool     g_prev_ok[MANTIS_SLOTS];
static uint32_t g_dop_since_ms = 0;

// Window length.  Long enough that a walking gait contributes several
// limb swings, short enough that the histogram still describes NOW
// rather than the last minute.
#define MANTIS_DOP_WINDOW_MS 3000

// How often the timekeeper re-solves the layout.  Beacons do not move,
// so this is about catching the case where one WAS moved -- often
// enough to notice within a few seconds, rare enough to cost nothing.
#define MANTIS_SURVEY_INTERVAL_MS 5000
static uint32_t          g_counter = 0;             // the legacy field
static bool              g_learning = true;         // baseline accumulation
static uint32_t          g_boot_ms = 0;

// Raw CSI handed from the Wi-Fi callback to the main loop.
//
// Same seqlock discipline as the receiver: the callback does a memcpy
// and nothing else.  Decoding in the Wi-Fi task would add latency to the
// radio path, and on a beacon that latency lands directly on transmit
// timing, which is the one thing the whole frame structure exists to
// protect.
struct RawCsi {
    volatile uint32_t seq;      // odd = write in progress
    int8_t   buf[MBC_SC_MAX * 2];
    int      len;
    uint8_t  from_id;
    int8_t   rssi;
    int8_t   noise;
    uint32_t t_us;
};
static RawCsi g_raw[MANTIS_SLOTS];
// The last COMPLETE capture per link, kept after process_csi() consumes
// the raw slot.  The dense payload used to read g_raw[i] directly --
// whose len process_csi() had just zeroed -- so every subcarrier profile
// ever transmitted was built from an empty buffer.
static int8_t  g_last_csi[MANTIS_SLOTS][MBC_SC_MAX * 2];
static int     g_last_len[MANTIS_SLOTS];
static int8_t  g_last_noise[MANTIS_SLOTS];

static uint8_t id_from_mac(const uint8_t *mac) {
    return mantis_beacon_id_from_mac(mac);     // 1A:00:uid:uid:00:id
}

// ── CSI callback: copy only ───────────────────────────────────
static void csi_cb(void *, wifi_csi_info_t *info) {
    if (!info || !info->buf || info->len < 128) return;
    const uint8_t from = id_from_mac(info->mac);
    if (from == 0 || from == beacon_id) return;

    RawCsi &r = g_raw[from];
    r.seq++;                       // odd: writing
    const int n = info->len > (int)sizeof(r.buf) ? (int)sizeof(r.buf) : info->len;
    memcpy((void *)r.buf, info->buf, n);
    r.len     = n;
    r.from_id = from;
    r.rssi    = info->rx_ctrl.rssi;
    // The genuine per-link RSSI, from the PHY.  now_recv() only ever
    // sees the ESP-NOW callback, which does not carry it.
    if (beacon_id && beacon_id < MANTIS_SLOTS) {
        // Averaged in FLOAT.  The integer form (pr*7 + rssi)/8 truncates
        // toward zero, which for negative dBm is toward a STRONGER signal:
        // it settles up to 6 dB high, reads every link as shorter than it
        // is, and warps the survey.
        float &pf = g_rssi_f[beacon_id][from];
        uint8_t &pn = g_rssi_n[beacon_id][from];
        pf = (pn == 0) ? (float)info->rx_ctrl.rssi
                       : pf + ((float)info->rx_ctrl.rssi - pf) * 0.125f;
        g_peer_rssi[beacon_id][from] = (int8_t)lrintf(pf);
        if (pn < 255) pn++;
    }
    r.noise   = info->rx_ctrl.noise_floor;
    r.t_us    = (uint32_t)esp_timer_get_time();
    r.seq++;                       // even: done
}

// ── ESP-NOW receive: the air frame, and the mesh bookkeeping ──
static void now_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (!info || len < (int)(sizeof(uint32_t) + sizeof(MantisAirFrame))) return;
    const uint8_t from = id_from_mac(info->src_addr);
    if (from == 0) return;

    // Payload layout: legacy counter, then the air frame.
    MantisAirFrame f;
    memcpy(&f, data + sizeof(uint32_t), sizeof(f));
    if (f.magic != MANTIS_AIR_MAGIC) return;
    if (f.uid == g_uid) return;                 // our own frame, echoed

    // Identity bookkeeping runs on EVERY frame, in every state -- an id
    // can be lost after it is committed if a lower-uid box claims it.
    mantis_id_saw(&g_id, from, f.uid, millis());

    const int64_t now = esp_timer_get_time();
    mantis_mesh_on_frame(&g_mesh, &f, now, -50, mantis_frame_is_tk(&f));

    // ── Other beacons' RSSI rows ───────────────────────────────
    // A REPORT carries the reporter's own row of the pairwise RSSI
    // matrix.  Collecting them is what gives the timekeeper the WHOLE
    // matrix to survey from, rather than only the row it measures itself.
    const int body = len - (int)(sizeof(uint32_t) + sizeof(MantisAirFrame));
    if (body == (int)sizeof(MantisPerspective)) {
        MantisPerspective p;
        memcpy(&p, data + sizeof(uint32_t) + sizeof(MantisAirFrame), sizeof(p));
        if (mantis_report_valid(&p, sizeof(p), MANTIS_MAX_BEACON_ID)) {
            const uint8_t h = p.reporter_id;
            for (uint8_t k = 0; k < p.n_links; k++) {
                const uint8_t j = p.links[k].peer_id;
                const int8_t  r = p.link_rssi[k];
                if (j == 0 || j >= MANTIS_SLOTS || r == 0) continue;
                g_peer_rssi[h][j] = r;
                if (g_rssi_n[h][j] < 255) g_rssi_n[h][j]++;
            }
            g_row_ms[h] = millis();
        }
    }
}

// ── Radio ─────────────────────────────────────────────────────
static void radio_begin() {
    WiFi.mode(WIFI_STA);
    // Fixed MAC so peers can identify each other by id alone.
    uint8_t mac[6];
    mantis_beacon_mac(g_uid, beacon_id, mac);   // unique per box, see mantis_air.h
    esp_wifi_set_mac(WIFI_IF_STA, mac);

    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(MANTIS_CHANNEL, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);

    // FIXED rate and power.  Rate adaptation would make RSSI
    // incomparable between beacons and would break the free-space
    // prediction the static-scene map depends on.
    esp_wifi_set_max_tx_power(MANTIS_TX_POWER_QDBM);
    esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_MCS0_LGI);

    if (esp_now_init() != ESP_OK) { ESP.restart(); }
    esp_now_register_recv_cb(now_recv);

    esp_now_peer_info_t p{};
    memcpy(p.peer_addr, BROADCAST, 6);
    p.channel = MANTIS_CHANNEL;
    p.ifidx   = WIFI_IF_STA;
    p.encrypt = false;
    esp_now_add_peer(&p);

    // A beacon is a RECEIVER too.  This one line is what turns N links
    // into N-squared: a beacon idle in someone else's slot measures that
    // transmission instead of ignoring it, at no airtime cost whatever.
    wifi_csi_config_t c{};
    c.lltf_en = true; c.htltf_en = true; c.stbc_htltf2_en = false;
    c.ltf_merge_en = true; c.channel_filter_en = false;
    c.manu_scale = false;
    esp_wifi_set_csi_config(&c);
    esp_wifi_set_csi_rx_cb(csi_cb, nullptr);
    esp_wifi_set_csi(true);
}

// ── Consume whatever the callback captured ────────────────────
static void process_csi() {
    for (uint8_t i = 1; i < MANTIS_SLOTS; i++) {
        RawCsi &r = g_raw[i];
        const uint32_t s0 = r.seq;
        if (s0 & 1u) continue;                 // mid-write, try next loop
        if (r.len == 0) continue;

        static int8_t local[MBC_SC_MAX * 2];
        const int n = r.len;
        memcpy(local, (const void *)r.buf, n);
        if (r.seq != s0) continue;             // torn: discard rather than trust

        float amp, phi; uint8_t q;
        if (!mantis_beacon_reduce(local, n, &amp, &phi, &q)) continue;

        // ── MICRO-DOPPLER ────────────────────────────────────
        // Rate between consecutive observations of THIS link.  The slot
        // schedule is what makes this meaningful: dt is known exactly,
        // so the rate is a measurement rather than an estimate carrying
        // the jitter of a free-running transmitter.
        if (g_prev_ok[i]) {
            float d = phi - g_prev_phi[i];
            while (d >  (float)M_PI) d -= 2.0f * (float)M_PI;
            while (d < -(float)M_PI) d += 2.0f * (float)M_PI;
            const float dt = (float)MANTIS_FRAME_US / 1000000.0f;
            mantis_dense_dop_accum(g_dop[i], d / dt);
        }
        g_prev_phi[i] = phi;
        g_prev_ok[i]  = true;

        mantis_beacon_link_update(&g_link[i], amp, phi, q, g_learning,
                                  nullptr, i);
        memcpy(g_last_csi[i], local, n);
        g_last_len[i]   = n;
        g_last_noise[i] = r.noise;
        r.len = 0;
    }
}

// ── SOLVE THE MESH'S OWN LAYOUT ───────────────────────────────
//
// Only the timekeeper does this, and only occasionally: the geometry of
// a deployment does not change between frames, and a solve every few
// seconds is far more often than furniture moves.
//
// The result REPLACES the assumed ring.  Until it runs, every position
// downstream is relative to a fiction -- a perfect hexagon that the
// beacons were never actually placed in.
static uint8_t g_geo_ids[MANTIS_SLOTS];     // solved index -> beacon id
static uint8_t g_geo_n = 0;

static void survey_geometry(uint32_t now_ms) {
    // Only the timekeeper publishes, and never a stale solve: a node that
    // becomes timekeeper again after a handover re-surveys before it
    // echoes anything.
    if (!g_mesh.is_timekeeper) { g_geo_ready = false; return; }
    if (now_ms - g_geo_last_ms < MANTIS_SURVEY_INTERVAL_MS) return;
    g_geo_last_ms = now_ms;

    // Our own row is live by construction; other rows are live while the
    // reports that carry them keep arriving.
    g_row_ms[beacon_id] = now_ms;

    // Solve over LIVE ids only, compactly indexed.  Solving over all six
    // left dead ids sitting on the seed ring, and they were then
    // published as if measured.
    uint8_t ids[MANTIS_SLOTS]; uint8_t n = 0;
    for (uint8_t id = 1; id <= MANTIS_MAX_BEACON_ID; id++) {
        const bool me = (id == beacon_id);
        const bool heard = g_mesh.member[id].present
                        || (g_rssi_n[beacon_id][id] > 0
                            && (now_ms - g_row_ms[id]) < 3 * MANTIS_SURVEY_INTERVAL_MS);
        if (me || heard) ids[n++] = id;
    }
    g_geo_n = n;
    if (n < 3) { g_geo_ready = false; return; }

    // Unmeasured pairs are marked negative and the stress solver skips
    // them -- which is why stress majorisation was chosen over classical
    // MDS.  A pair is averaged over both directions when both are known.
    static float dist[MANTIS_SLOTS * MANTIS_SLOTS];
    for (uint8_t i = 0; i < n; i++)
        for (uint8_t j = 0; j < n; j++) {
            if (i == j) { dist[i * n + j] = 0.0f; continue; }
            const uint8_t a = ids[i], b = ids[j];
            int sum = 0; int cnt = 0;
            if (g_rssi_n[a][b]) { sum += g_peer_rssi[a][b]; cnt++; }
            if (g_rssi_n[b][a]) { sum += g_peer_rssi[b][a]; cnt++; }
            dist[i * n + j] = cnt ? mantis_rssi_to_m((int8_t)(sum / cnt)) : -1.0f;
        }

    float x[MANTIS_SLOTS], y[MANTIS_SLOTS];
    const float stress = mantis_geom_solve(dist, n, x, y, 120);
    mantis_geom_canonical(x, y, n);
    g_geo_cm = mantis_geom_normalise(x, y, n);
    for (uint8_t i = 0; i < MANTIS_SLOTS; i++) { g_geo_x[i] = 0; g_geo_y[i] = 0; g_geo_ids[i] = 0; }
    for (uint8_t i = 0; i < n; i++) { g_geo_x[i] = x[i]; g_geo_y[i] = y[i]; g_geo_ids[i] = ids[i]; }
    const float sq = stress * 255.0f;
    g_geo_stress = (uint8_t)(sq > 255.0f ? 255.0f : (sq < 0 ? 0 : sq));
    // Publish even a poor solve -- the CONFIDENCE travels with it, and the
    // receiver decides what it trusts.
    g_geo_ready = true;
}

// Fill the geometry packet.  Indexed by BEACON ID (node[id-1]); ids the
// survey did not solve are left without MANTIS_GN_VALID.
static void build_geometry(MantisGeomPacket *g, uint32_t seq) {
    *g = MantisGeomPacket{};
    g->counter     = g_counter;
    g->reporter_id = beacon_id;
    g->seq         = seq;
    g->n_nodes     = MANTIS_MAX_BEACON_ID;
    g->stress_q8   = g_geo_stress;
    g->scale_known = 0;            // RSSI ranges: shape yes, metres no
    g->cm_per_unit = g_geo_cm;
    for (uint8_t i = 0; i < g_geo_n; i++) {
        const uint8_t id = g_geo_ids[i];
        if (id == 0 || id > MANTIS_MAX_BEACON_ID) continue;
        MantisGeomNode &nd = g->node[id - 1];
        nd.x_q10 = (int16_t)(g_geo_x[i] * 1024.0f);
        nd.y_q10 = (int16_t)(g_geo_y[i] * 1024.0f);
        nd.noise_floor = g_last_noise[id];
        uint8_t heard = 0;
        for (uint8_t j = 1; j <= MANTIS_MAX_BEACON_ID; j++)
            if (j != id && g_rssi_n[id][j]) heard |= (uint8_t)(1u << (j - 1));
        nd.heard_mask = heard;
        nd.uptime_s = (uint16_t)((millis() / 1000) > 65535 ? 65535 : (millis() / 1000));
        nd.flags = MANTIS_GN_VALID;
        if (__builtin_popcount(heard) < 2) nd.flags |= MANTIS_GN_ISOLATED;
    }
    mantis_geom_seal(g);
}

// The lowest noise floor any link reported recently.  The report used to
// read g_raw[1] -- beacon 1's link -- which beacon 1 itself never has.
static int8_t quietest_noise() {
    int8_t best = 0;
    for (uint8_t i = 1; i < MANTIS_SLOTS; i++) {
        if (i == beacon_id || g_last_len[i] == 0) continue;
        if (best == 0 || g_last_noise[i] < best) best = g_last_noise[i];
    }
    return best;
}

// ── Build and send this slot's payload ────────────────────────
static void transmit(MantisPayloadKind kind) {
    uint8_t pkt[256];
    size_t  off = 0;

    // 1. THE LEGACY COUNTER, ALWAYS FIRST.
    memcpy(pkt, &g_counter, sizeof(g_counter));
    off += sizeof(g_counter);

    // 2. The air frame: identity and timing for the mesh.
    MantisAirFrame f{};
    mantis_mesh_fill_frame(&g_mesh, &f, esp_timer_get_time());
    f.uid = g_uid;
    memcpy(pkt + off, &f, sizeof(f));
    off += sizeof(f);

    // 3. Whatever the program asked for this frame.
    if (kind == MPL_REPORT) {
        MantisPerspective p{};
        p.reporter_id = beacon_id;
        p.seq         = f.seq;
        p.noise_floor = quietest_noise();
        p.rssi_self   = -50;
        p.flags = (g_mesh.sched.synced ? MANTIS_RF_SYNCED : 0)
                | (g_learning ? 0 : MANTIS_RF_BASELINE_OK);
        uint8_t k = 0;
        for (uint8_t i = 1; i < MANTIS_SLOTS && k < MANTIS_MAX_LINKS; i++) {
            if (i == beacon_id || !g_link[i].ready) continue;
            // Re-emit the CURRENT state of each link.  A report is state,
            // never a delta, so a lost one costs nothing and a duplicate
            // is harmless.
            mantis_beacon_link_update(&g_link[i], g_link[i].amp_ema,
                                      g_link[i].phi_ema,
                                      g_link[i].last_quality,
                                      false, &p.links[k], i);
            p.link_rssi[k] = g_rssi_n[beacon_id][i] ? g_peer_rssi[beacon_id][i] : 0;
            k++;
        }
        p.n_links = k;
        // THIS BEACON'S OWN VERDICT, one bit per peer.  Decided against
        // each link's own quiet behaviour, which only this beacon knows.
        p.blocked_mask = 0;
        for (uint8_t i = 1; i < MANTIS_SLOTS && i <= MANTIS_MAX_LINKS; i++)
            if (i != beacon_id && g_link[i].ready && g_link[i].blocked)
                p.blocked_mask |= (uint8_t)(1u << (i - 1));
        mantis_report_seal(&p);
        if (off + sizeof(p) <= sizeof(pkt)) { memcpy(pkt + off, &p, sizeof(p)); off += sizeof(p); }
    } else if (kind == MPL_DENSE) {
        MantisDensePacket d{};
        d.counter     = g_counter;
        d.reporter_id = beacon_id;
        d.seq         = f.seq;
        d.noise_floor = quietest_noise();
        // Two links per dense packet, rotating, so a full deep sweep
        // completes over several macroframes without ever making one
        // packet large.
        uint8_t k = 0;
        for (uint8_t step = 1; step < MANTIS_SLOTS && k < MANTIS_DENSE_LINKS; step++) {
            const uint8_t i = (uint8_t)(((f.seq / MANTIS_MACRO_FRAMES + step)
                                         % (MANTIS_SLOTS - 1)) + 1);
            if (i == beacon_id || !g_link[i].ready) continue;
            MantisDenseLink &L = d.link[k];
            L.peer_id = i;
            L.quality = g_link[i].last_quality;
            mantis_dense_profile(g_last_csi[i], g_last_len[i],
                                 L.sc_profile, &L.delay_spread_q12);
            float da = 0.0f;
            if (g_link[i].base_n > 0 && g_link[i].amp_base > 1e-3f)
                da = (g_link[i].amp_ema - g_link[i].amp_base) / g_link[i].amp_base;
            L.amp_mean_q8 = mantis_q8(da);
            L.phase_q12   = mantis_q12(g_link[i].phi_ema - g_link[i].phi_base);
            L.flags       = mantis_dense_classify(L.delay_spread_q12, da);
            memcpy(L.dop_hist, g_dop[i], sizeof(L.dop_hist));
            // A narrow spectrum is machinery, not a person.  Flagged on
            // the BEACON because only it has the per-frame rates.
            if (mantis_dense_is_periodic(g_dop[i])) L.flags |= MANTIS_DF_PERIODIC;
            k++;
        }
        d.n_links = k;
        mantis_dense_seal(&d);
        if (off + sizeof(d) <= sizeof(pkt)) { memcpy(pkt + off, &d, sizeof(d)); off += sizeof(d); }
    } else if (kind == MPL_ECHO) {
        // THE SURVEYED LAYOUT.  This branch did not exist: the slot was
        // scheduled, the timekeeper solved the geometry every 5 s, and
        // then transmitted a bare frame -- so no receiver anywhere ever
        // learned where the beacons actually are.
        MantisGeomPacket g;
        build_geometry(&g, f.seq);
        if (off + sizeof(g) <= sizeof(pkt)) { memcpy(pkt + off, &g, sizeof(g)); off += sizeof(g); }
    }

    esp_now_send(BROADCAST, pkt, off);
    g_counter++;
    g_mesh.frames_tx++;
}

// ── Arduino ───────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    g_boot_ms = millis();

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    g_uid = mantis_uid_from_mac(mac);

    // A previously claimed id, if there is one.  This is what keeps the
    // room's survey and baselines valid across a power cycle.
    g_nvs.begin("mantis", false);
    const uint8_t stored = (uint8_t)g_nvs.getUChar("bid", 0);
    mantis_id_begin(&g_id, g_uid, stored, millis());

    radio_begin();
    Serial.printf("[mantis] uid %u, stored id %u, %s\n",
                  g_uid, stored, mantis_id_state_name(g_id.state));
}

void loop() {
    const int64_t now = esp_timer_get_time();
    const uint32_t now_ms = millis();

    // ── IDENTITY FIRST ────────────────────────────────────────
    // Nothing else can run until we know who we are: the slot, the
    // reports and the whole schedule are derived from the id.
    if (mantis_id_tick(&g_id, now_ms)) {
        beacon_id = g_id.id;
        if (mantis_id_may_tx(&g_id)) {
            mantis_mesh_init(&g_mesh, beacon_id, g_uid);
            uint8_t mac[6];
            mantis_beacon_mac(g_uid, beacon_id, mac);
            esp_wifi_set_mac(WIFI_IF_STA, mac);
            Serial.printf("[mantis] id %u claimed (uid %u), slot %u\n",
                          beacon_id, g_uid, g_mesh.sched.my_slot);
        }
    }
    if (g_id.dirty && g_id.state == MID_COMMITTED) {
        g_id.dirty = false;
        g_nvs.putUChar("bid", g_id.id);   // keep it for life
        Serial.printf("[mantis] id %u persisted\n", g_id.id);
    }
    // Listening, claiming or locked out: stay off the air entirely.
    // Transmitting before the id is settled pollutes the very picture
    // being used to settle it.
    if (!mantis_id_may_tx(&g_id)) { process_csi(); return; }

    mantis_mesh_tick(&g_mesh, now);
    process_csi();
    survey_geometry(now_ms);

    // Age the Doppler window.  Halving rather than clearing keeps a
    // little history across the boundary, so a target that was moving a
    // moment ago does not read as brand new on the next window.
    if (now_ms - g_dop_since_ms > MANTIS_DOP_WINDOW_MS) {
        g_dop_since_ms = now_ms;
        for (uint8_t i = 0; i < MANTIS_SLOTS; i++)
            for (uint8_t b = 0; b < MANTIS_DENSE_DOP_BINS; b++)
                g_dop[i][b] = (uint8_t)(g_dop[i][b] >> 1);
    }

    // Baseline learning ends once the mesh has been stable for a while.
    // Deliberately time-based rather than command-based: a beacon must
    // reach a useful state on its own, because in a standalone
    // deployment there may be nobody to tell it to.
    if (g_learning && (millis() - g_boot_ms) > 20000 && g_mesh.sched.synced)
        g_learning = false;

    // ── COLD START ────────────────────────────────────────────
    // Unsynced and nobody is keeping time: transmit anyway, at our own
    // slot against a provisional epoch.  Otherwise a room full of
    // beacons that all boot together would sit silent forever, each
    // waiting for a sync that only another transmission can provide.
    if (!g_mesh.sched.synced && !g_mesh.id_conflict) {
        static int64_t s_listen_until = 0;
        if (s_listen_until == 0) s_listen_until = now + 2 * MANTIS_FRAME_US;
        if (now > s_listen_until) {
            g_mesh.sched.epoch_us     = now;
            g_mesh.sched.seq          = 0;
            g_mesh.sched.synced       = true;
            g_mesh.sched.last_sync_us = now;
            g_mesh.is_timekeeper      = true;
            g_mesh.tk_id              = beacon_id;
        }
    }

    const MantisDuty duty = mantis_duty(&g_mesh.sched, now, beacon_id,
                                        MANTIS_SLOTS - 2, g_learning);

    // Transmit exactly once per slot, and only inside the window that
    // cannot bleed into the next beacon's.
    static uint32_t last_tx_seq  = 0xFFFFFFFF;
    static uint8_t  last_tx_slot = 0xFF;
    if (duty.transmit && mantis_mesh_may_tx(&g_mesh, now)
        && (duty.seq != last_tx_seq || duty.slot != last_tx_slot)) {
        last_tx_seq  = duty.seq;
        last_tx_slot = duty.slot;
        // Only the timekeeper holds a solved layout.  Anyone else handed
        // the geometry slot sends its perspective instead, so the slot
        // is never wasted on a packet that would have been empty.
        MantisPayloadKind k = duty.payload;
        if (k == MPL_ECHO && !(g_mesh.is_timekeeper && g_geo_ready)) k = duty.fallback;
        transmit(k);
    }

    // ── SLEEP ─────────────────────────────────────────────────
    // Only in slots with no speaker, and never during baseline
    // learning -- that data is unrepeatable and power is the cheapest
    // thing to spend while acquiring it.
    if (duty.may_sleep) {
        int32_t off = 0; uint32_t sq = 0;
        mantis_sched_slot(&g_mesh.sched, now, &sq, &off);
        const int32_t left = (int32_t)MANTIS_SLOT_US - off - 400;
        if (left > 1500) {
            esp_sleep_enable_timer_wakeup((uint64_t)left);
            esp_light_sleep_start();
        }
    }

    if ((g_counter % 600) == 0 && g_counter) {
        Serial.printf("[mantis] seq=%lu tk=%d conflict=%d peers=%u rx=%lu\n",
                      (unsigned long)g_mesh.sched.seq,
                      (int)g_mesh.is_timekeeper, (int)g_mesh.id_conflict,
                      mantis_mesh_live(&g_mesh, now),
                      (unsigned long)g_mesh.frames_rx);
    }
}
