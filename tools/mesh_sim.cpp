// ═══════════════════════════════════════════════════════════════
//  MESH SIMULATION — the shared mesh headers, run together on a host
// ═══════════════════════════════════════════════════════════════
//
//  Pre-flight proves the firmware COMPILES.  This proves the radios
//  AGREE: N beacons boot within half a second of each other with blank
//  NVS, claim ids, elect a timekeeper, sync their slots, measure each
//  other, survey their own layout and report -- and a receiver built
//  from the same mantis_receiver.h the probes and the anchor run turns
//  that into a baseline, a surveyed map and, once someone walks in, a
//  contact in the right place.
//
//  The beacon loop below mirrors CSI-Beacon-Mantis.ino step for step
//  (identity -> membership -> duty -> transmit).  CSI itself is
//  synthesised: link amplitude drops when a body stands near the chord.
//
//  Exit code 0 = every invariant held.  Run by tools/preflight.sh.
// ═══════════════════════════════════════════════════════════════
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdint.h>

static int64_t g_now_us = 0;
int64_t mantis_host_now_us(void) { return g_now_us; }

#include "../mantis_air.h"
#include "../mantis_sched.h"
#include "../mantis_membership.h"
#include "../mantis_identity.h"
#include "../mantis_program.h"
#include "../mantis_report.h"
#include "../mantis_geometry.h"
#include "../mantis_receiver.h"
#include "../mantis_align.h"

static int failures = 0;
#define CHECK(cond, ...) do { if (!(cond)) { failures++; printf("  FAIL: " __VA_ARGS__); printf("\n"); } \
                              else { printf("  ok:   " __VA_ARGS__); printf("\n"); } } while (0)

static uint32_t rng = 12345;
static float frand() { rng = rng * 1664525u + 1013904223u; return (float)(rng >> 8) / 16777216.0f; }

struct SimBeacon {
    float wx, wy;                  // world position, metres
    uint16_t uid;
    int64_t boot_us;
    bool booted;
    MantisIdentity id;
    MantisMembership mesh;
    uint8_t beacon_id;
    int8_t  rssi[MANTIS_SLOTS][MANTIS_SLOTS];
    uint8_t rssi_n[MANTIS_SLOTS][MANTIS_SLOTS];
    float   rssi_f[MANTIS_SLOTS][MANTIS_SLOTS];
    uint32_t counter;
    uint32_t last_seq; uint8_t last_slot;
    int64_t listen_until;
    float geo_x[MANTIS_SLOTS], geo_y[MANTIS_SLOTS]; uint8_t geo_ids[MANTIS_SLOTS], geo_n;
    uint8_t geo_stress; bool geo_ready; uint32_t geo_last_ms;
};

static int NB = 6;
static SimBeacon B[6];
static int g_conflict = 0;
static MantisReceiver RX;
static bool person = false;
static float pwx = 0.5f, pwy = 0.3f;
static uint32_t rx_frames = 0;

static float dist_pt_seg(float px, float py, float ax, float ay, float bx, float by) {
    const float dx = bx - ax, dy = by - ay;
    float t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy + 1e-9f);
    t = t < 0 ? 0 : (t > 1 ? 1 : t);
    const float cx = ax + t * dx - px, cy = ay + t * dy - py;
    return sqrtf(cx * cx + cy * cy);
}

static SimBeacon *by_id(uint8_t id) {
    for (int k = 0; k < NB; k++)
        if (B[k].booted && B[k].beacon_id == id && mantis_id_may_tx(&B[k].id)) return &B[k];
    return nullptr;
}

// One frame on the air: every other radio hears it (5% loss).
static void air(SimBeacon &tx, const uint8_t *pkt, int len) {
    MantisAirFrame f; memcpy(&f, pkt + 4, sizeof(f));
    for (int k = 0; k < NB; k++) {
        SimBeacon &r = B[k];
        if (&r == &tx || !r.booted) continue;
        if (frand() < 0.05f) continue;
        const float d = hypotf(r.wx - tx.wx, r.wy - tx.wy);
        const int8_t rs = (int8_t)(-40.0f - 26.0f * log10f(d > 0.2f ? d : 0.2f) + (frand() - 0.5f) * 3.0f);
        // CSI path: record the RSSI row for this hearer.
        if (r.beacon_id && tx.beacon_id) {
            float &pf = r.rssi_f[r.beacon_id][tx.beacon_id];
            uint8_t &pn = r.rssi_n[r.beacon_id][tx.beacon_id];
            pf = pn ? pf + (rs - pf) * 0.125f : (float)rs;
            r.rssi[r.beacon_id][tx.beacon_id] = (int8_t)lrintf(pf); if (pn < 255) pn++;
        }
        // ESP-NOW path.
        mantis_id_saw(&r.id, f.beacon_id, f.uid, (uint32_t)(g_now_us / 1000));
        mantis_mesh_on_frame(&r.mesh, &f, g_now_us, -50, mantis_frame_is_tk(&f));
        const int body = len - 4 - (int)sizeof(f);
        if (body == (int)sizeof(MantisPerspective)) {
            MantisPerspective p; memcpy(&p, pkt + 4 + sizeof(f), sizeof(p));
            if (mantis_report_valid(&p, sizeof(p), MANTIS_MAX_BEACON_ID))
                for (uint8_t i = 0; i < p.n_links; i++)
                    if (p.link_rssi[i]) { r.rssi[p.reporter_id][p.links[i].peer_id] = p.link_rssi[i];
                                          r.rssi_n[p.reporter_id][p.links[i].peer_id]++; }
        }
    }
    if (frand() >= 0.05f) {
        mantis_rx_packet(&RX, f.beacon_id, pkt, len, (uint32_t)(g_now_us / 1000));
        rx_frames++;
    }
}

static void survey(SimBeacon &b, uint32_t now_ms) {
    if (!b.mesh.is_timekeeper) { b.geo_ready = false; return; }
    if (now_ms - b.geo_last_ms < 5000) return;
    b.geo_last_ms = now_ms;
    uint8_t ids[MANTIS_SLOTS], n = 0;
    for (uint8_t id = 1; id <= MANTIS_MAX_BEACON_ID; id++)
        if (id == b.beacon_id || b.mesh.member[id].present) ids[n++] = id;
    b.geo_n = n;
    if (n < 3) { b.geo_ready = false; return; }
    static float dist[64];
    for (uint8_t i = 0; i < n; i++) for (uint8_t j = 0; j < n; j++) {
        if (i == j) { dist[i * n + j] = 0; continue; }
        int sum = 0, c = 0; const uint8_t a = ids[i], bb = ids[j];
        if (b.rssi_n[a][bb]) { sum += b.rssi[a][bb]; c++; }
        if (b.rssi_n[bb][a]) { sum += b.rssi[bb][a]; c++; }
        dist[i * n + j] = c ? mantis_rssi_to_m((int8_t)(sum / c)) : -1.0f;
    }
    float x[MANTIS_SLOTS], y[MANTIS_SLOTS];
    const float st = mantis_geom_solve(dist, n, x, y, 120);
    mantis_geom_canonical(x, y, n);
    mantis_geom_normalise(x, y, n);
    for (uint8_t i = 0; i < n; i++) { b.geo_x[i] = x[i]; b.geo_y[i] = y[i]; b.geo_ids[i] = ids[i]; }
    b.geo_stress = (uint8_t)(st * 255 > 255 ? 255 : st * 255);
    b.geo_ready = true;
}

static void transmit(SimBeacon &b, MantisPayloadKind kind) {
    uint8_t pkt[256]; int off = 0;
    memcpy(pkt, &b.counter, 4); off = 4;
    MantisAirFrame f{}; mantis_mesh_fill_frame(&b.mesh, &f, g_now_us); f.uid = b.uid;
    memcpy(pkt + off, &f, sizeof(f)); off += sizeof(f);
    if (kind == MPL_REPORT) {
        MantisPerspective p{};
        p.reporter_id = b.beacon_id; p.seq = f.seq;
        uint8_t k = 0;
        for (uint8_t j = 1; j <= MANTIS_MAX_BEACON_ID && k < MANTIS_MAX_LINKS; j++) {
            if (j == b.beacon_id) continue;
            SimBeacon *o = by_id(j); if (!o) continue;
            float amp = (frand() - 0.5f) * 0.02f;
            if (person && dist_pt_seg(pwx, pwy, b.wx, b.wy, o->wx, o->wy) < 0.35f) amp -= 0.35f;
            p.links[k].peer_id = j; p.links[k].quality = 200;
            p.links[k].amp_q8 = mantis_q8(amp); p.links[k].phase_q12 = 0;
            p.link_rssi[k] = b.rssi_n[b.beacon_id][j] ? b.rssi[b.beacon_id][j] : 0;
            k++;
        }
        p.n_links = k;
        mantis_report_seal(&p);
        memcpy(pkt + off, &p, sizeof(p)); off += sizeof(p);
    } else if (kind == MPL_ECHO && b.mesh.is_timekeeper && b.geo_ready) {
        MantisGeomPacket g{};
        g.counter = b.counter; g.reporter_id = b.beacon_id; g.seq = f.seq;
        g.n_nodes = MANTIS_MAX_BEACON_ID; g.stress_q8 = b.geo_stress;
        for (uint8_t i = 0; i < b.geo_n; i++) {
            MantisGeomNode &nd = g.node[b.geo_ids[i] - 1];
            nd.x_q10 = (int16_t)(b.geo_x[i] * 1024); nd.y_q10 = (int16_t)(b.geo_y[i] * 1024);
            nd.flags = MANTIS_GN_VALID;
        }
        mantis_geom_seal(&g);
        memcpy(pkt + off, &g, sizeof(g)); off += sizeof(g);
    }
    air(b, pkt, off);
    b.counter++;
}

static void beacon_step(SimBeacon &b) {
    const uint32_t now_ms = (uint32_t)(g_now_us / 1000);
    if (!b.booted) {
        if (g_now_us < b.boot_us) return;
        b.booted = true;
        mantis_id_begin(&b.id, b.uid, (g_conflict && (&b == &B[0] || &b == &B[1])) ? 1 : 0, now_ms);
    }
    if (mantis_id_tick(&b.id, now_ms)) {
        b.beacon_id = b.id.id;
        if (mantis_id_may_tx(&b.id)) { mantis_mesh_init(&b.mesh, b.beacon_id, b.uid); b.listen_until = 0; }
    }
    if (!mantis_id_may_tx(&b.id)) return;
    mantis_mesh_tick(&b.mesh, g_now_us);
    survey(b, now_ms);
    if (!b.mesh.sched.synced && !b.mesh.id_conflict) {
        if (b.listen_until == 0) b.listen_until = g_now_us + 2 * MANTIS_FRAME_US;
        if (g_now_us > b.listen_until) {
            b.mesh.sched.epoch_us = g_now_us; b.mesh.sched.seq = 0; b.mesh.sched.synced = true;
            b.mesh.sched.last_sync_us = g_now_us; b.mesh.is_timekeeper = true; b.mesh.tk_id = b.beacon_id;
        }
    }
    const MantisDuty d = mantis_duty(&b.mesh.sched, g_now_us, b.beacon_id, MANTIS_SLOTS - 2, true);
    if (d.transmit && mantis_mesh_may_tx(&b.mesh, g_now_us) && (d.seq != b.last_seq || d.slot != b.last_slot)) {
        b.last_seq = d.seq; b.last_slot = d.slot;
        MantisPayloadKind k = d.payload;
        if (k == MPL_ECHO && !(b.mesh.is_timekeeper && b.geo_ready)) k = d.fallback;
        transmit(b, k);
    }
}

int main(int argc, char **argv) {
    if (argc > 1) NB = atoi(argv[1]);
    if (argc > 2) g_conflict = atoi(argv[2]);
    if (NB < 1 || NB > 6) NB = 6;
    printf("mesh simulation: %d beacons, %s, boots within 500 ms\n", NB,
           g_conflict ? "TWO BOXES WITH THE SAME STORED ID" : "blank NVS");
    for (int k = 0; k < NB; k++) {
        const float a = 2.0f * (float)M_PI * k / NB + 0.3f;
        B[k].wx = 2.2f * cosf(a) + (frand() - 0.5f) * 0.4f;
        B[k].wy = 1.6f * sinf(a) + (frand() - 0.5f) * 0.4f;
        B[k].uid = (uint16_t)(1000 + (int)(frand() * 60000));
        B[k].boot_us = (int64_t)(frand() * 500000);
        B[k].last_seq = 0xFFFFFFFF; B[k].last_slot = 0xFF;
    }
    // With three beacons there are only three chords.  Put the person ON
    // one, where the physics says they can be seen; anywhere else in a
    // triangle they cross nothing and the honest answer is "no contact".
    if (NB == 3) { pwx = 0.5f * (B[0].wx + B[1].wx); pwy = 0.5f * (B[0].wy + B[1].wy); }
    mantis_rx_begin(&RX, 1.2f, 0, 1, false);

    const int64_t STEP = 250;
    for (g_now_us = 0; g_now_us < 90LL * 1000000; g_now_us += STEP) {
        for (int k = 0; k < NB; k++) beacon_step(B[k]);
        if (g_now_us % 100000 == 0) mantis_rx_solve(&RX, (uint32_t)(g_now_us / 1000), 0.1f);
        if (!person && g_now_us > 70LL * 1000000) person = true;
    }

    printf("-- identity --\n");
    uint8_t seen = 0; int committed = 0;
    for (int k = 0; k < NB; k++) {
        printf("  box uid %5u -> id %u (%s)\n", B[k].uid, B[k].beacon_id, mantis_id_state_name(B[k].id.state));
        if (B[k].id.state == MID_COMMITTED) { committed++; seen |= (uint8_t)(1u << (B[k].beacon_id - 1)); }
    }
    CHECK(committed == NB, "all %d beacons committed an id", NB);
    CHECK(seen == (uint8_t)((1u << NB) - 1), "ids are unique and contiguous 1..%d (mask 0x%02X)", NB, seen);

    printf("-- timekeeping --\n");
    int tks = 0; int64_t epoch = 0; bool same = true;
    for (int k = 0; k < NB; k++) {
        if (B[k].mesh.is_timekeeper) tks++;
        uint32_t sq; int32_t off; mantis_sched_slot(&B[k].mesh.sched, g_now_us, &sq, &off);
        if (k == 0) epoch = B[k].mesh.sched.epoch_us;
        else if (llabs(B[k].mesh.sched.epoch_us - epoch) > 2000) same = false;
    }
    CHECK(tks == 1, "exactly one timekeeper (%d)", tks);
    CHECK(same, "every beacon shares one slot grid (epoch within 2 ms)");
    int conflicts = 0; for (int k = 0; k < NB; k++) conflicts += B[k].mesh.id_conflict;
    CHECK(conflicts == 0, "no beacon left stuck in id_conflict (%d)", conflicts);

    printf("-- receiver --\n");
    CHECK(RX.deploy.beacons == NB, "receiver sees %d live beacons (%u)", NB, RX.deploy.beacons);
    CHECK(RX.reports_in > (unsigned)(80 * NB), "reports accepted (%lu), rejected %lu", (unsigned long)RX.reports_in, (unsigned long)RX.rejected);
    if (NB < 3) {
        CHECK(RX.caps.link_presence && !RX.caps.localize, "fewer than 3 beacons: presence yes, localisation honestly off");
        printf(failures ? "MESH SIM: %d FAILURE(S)\n" : "MESH SIM: all invariants hold\n", failures);
        return failures ? 1 : 0;
    }
    CHECK(RX.geom_in > 0, "surveyed geometry adopted (%lu packets, stress %u)", (unsigned long)RX.geom_in, RX.geom_stress);
    CHECK(mantis_mesh_geometry_confidence(&RX.mesh) > 0.99f, "every live beacon has a measured position");
    CHECK(RX.baseline_latched && !RX.baseline_partial, "baseline latched on coverage, not on the backstop");

    // Survey fidelity: fit the published layout to the true one.
    float px[8], py[8], qx[8], qy[8]; int n = 0;
    for (int k = 0; k < NB; k++) {
        const uint8_t id = B[k].beacon_id;
        px[n] = RX.mesh.bx[id]; py[n] = RX.mesh.by[id]; qx[n] = B[k].wx; qy[n] = B[k].wy; n++;
    }
    MantisAlign A; mantis_align_fit(px, py, qx, qy, n, false, &A);
    printf("  survey vs truth: rms %.2f m (scale %.2f m/unit, mirror %d)\n", A.rms, A.scale, A.mirror);
    CHECK(A.valid && A.rms < 0.45f, "surveyed layout matches the real one (rms %.2f m)", A.rms);

    printf("-- detection --\n");
    float ppx, ppy; mantis_align_invert(&A, pwx, pwy, &ppx, &ppy);
    int believed = 0; float best = 9;
    for (int i = 0; i < MANTIS_FUSE_MAX; i++) {
        const MantisFused &t = RX.fusion.t[i];
        if (!t.active) continue;
        const float d = hypotf(t.x - ppx, t.y - ppy);
        if (d < best) best = d;
        if (mantis_fuse_believed(&t)) believed++;
    }
    int tg = 0; float tbest = 9;
    for (int i = 0; i < RX.targets.n; i++) if (RX.targets.t[i].active) {
        tg++; const float d = hypotf(RX.targets.t[i].x - ppx, RX.targets.t[i].y - ppy); if (d < tbest) tbest = d; }
    printf("  person (survey frame) %.2f,%.2f  targets %d nearest %.2f  fused nearest %.2f\n",
           ppx, ppy, tg, tbest, best);
    CHECK(RX.mesh.n_chords >= 3, "tomography has chords (%d)", RX.mesh.n_chords);
    // Three beacons make three chords: a body near one chord is on a LINE,
    // not a point, so the honest tolerance is wider.
    const float tol = NB >= 4 ? 0.45f : 0.75f;
    CHECK(tg > 0 && tbest < tol, "the person is found near where they stand (%.2f units, tol %.2f)", tbest, tol);

    printf("-- alignment round trip --\n");
    float rx2, ry2, bx2, by2; mantis_align_apply(&A, 0.2f, -0.4f, &rx2, &ry2);
    mantis_align_invert(&A, rx2, ry2, &bx2, &by2);
    CHECK(fabsf(bx2 - 0.2f) < 1e-3f && fabsf(by2 + 0.4f) < 1e-3f, "apply/invert are inverses");

    printf(failures ? "MESH SIM: %d FAILURE(S)\n" : "MESH SIM: all invariants hold\n", failures);
    return failures ? 1 : 0;
}
