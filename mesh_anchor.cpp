// ═══════════════════════════════════════════════════════════════
//  mesh_anchor.cpp — see mesh_anchor.h
// ═══════════════════════════════════════════════════════════════
#include "mesh_anchor.h"
#include "scene.h"
#include "wizard.h"
#include "csi.h"
#include "ui.h"
#include "input.h"
#include "tactical.h"
#include "lgfx_tdisplay_s3.h"
#include "mantis_receiver.h"
#include "mantis_align.h"
#include <esp_now.h>
#include <esp_heap_caps.h>
#include <string.h>
#include <math.h>

extern LovyanGFX &gfx_sprite();

// ── The receiver ──────────────────────────────────────────────
// ~12 KB.  PSRAM first: internal DRAM is reserved for the 108 KB display
// sprite, and the CI headroom gate is measuring exactly that.
static MantisReceiver *R = nullptr;

// ── Wi-Fi task -> loop hand-off ───────────────────────────────
// Seqlock ring, the same discipline as the probes and the beacons: the
// callback copies and returns, parsing happens here.
#define MA_QUEUE 12
struct MaPkt {
    volatile uint32_t seq;
    uint8_t  data[192];
    uint16_t len;
    uint8_t  from;
    volatile bool pending;
};
static MaPkt s_q[MA_QUEUE];
static volatile uint8_t s_head = 0;

// ── Alignment mesh -> radar ───────────────────────────────────
static MantisAlign s_align = {};
static bool        s_aligned = false;
static uint32_t    s_align_ms = 0;

// ── Hand-held probe ───────────────────────────────────────────
static uint32_t s_probe_ms   = 0;
static uint8_t  s_probe_kind = 0;
static uint16_t s_key_seq    = 0;
static uint32_t s_key_ms     = 0;
static uint32_t s_status_ms  = 0;

static const uint8_t BCAST[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// ONE unit answers the probe.  With a T-Display stereo pair both would
// otherwise broadcast status (the probe's screen flickering between two
// anchors) and BOTH would apply every remote press -- a double advance
// through the wizard.  The PRIMARY (lower MAC) serves; the pair's own
// state hints carry the result to the other unit.
static bool serves_probe() {
    return !g_app.peer.peer_present || g_app.peer.role != ROLE_SECONDARY;
}


void mesh_anchor_begin() {
    if (R) return;
    R = (MantisReceiver *)heap_caps_calloc(1, sizeof(MantisReceiver),
                                           MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!R) R = (MantisReceiver *)calloc(1, sizeof(MantisReceiver));
    if (!R) { MSLOGLN("[mesh] receiver alloc FAILED - mesh disabled"); return; }
    // This device is the fixed anchor: one T-Display, not docked unless
    // the stereo peer is present (updated as the session develops).
    mantis_rx_begin(R, 1.2f, /*tdisplays*/1, /*cardputers*/0, /*docked*/false);
    MSLOG("[mesh] receiver up (%u B)\n", (unsigned)sizeof(MantisReceiver));
}

void mesh_anchor_enqueue(uint8_t id, const uint8_t *data, int len) {
    if (!data || len <= 0 || len > (int)sizeof(s_q[0].data)) return;
    const uint8_t h = s_head;
    MaPkt &p = s_q[h];
    p.seq++;
    memcpy(p.data, data, (size_t)len);
    p.len = (uint16_t)len;
    p.from = id;
    p.pending = true;
    p.seq++;
    s_head = (uint8_t)((h + 1) % MA_QUEUE);
}

static void pump(uint32_t now) {
    for (int i = 0; i < MA_QUEUE; i++) {
        MaPkt &p = s_q[i];
        if (!p.pending) continue;
        const uint32_t s0 = p.seq;
        if (s0 & 1u) continue;
        static uint8_t local[192];
        const uint16_t len = p.len; const uint8_t from = p.from;
        memcpy(local, p.data, len);
        if (p.seq != s0) continue;
        p.pending = false;
        mantis_rx_packet(R, from, local, len, now);
    }
}

// Beacons both frames know: mesh id with a SURVEYED position, and the
// same id active on this anchor, where the radar draws it.
static void realign(uint32_t now) {
    if (now - s_align_ms < 1000) return;
    s_align_ms = now;
    float px[MAX_BEACONS], py[MAX_BEACONS], qx[MAX_BEACONS], qy[MAX_BEACONS];
    int n = 0;
    for (int i = 0; i < MAX_BEACONS; i++) {
        const BeaconState &b = g_app.beacon[i];
        if (!b.active || b.id == 0 || b.id >= MANTIS_SLOTS) continue;
        if (!R->mesh.live[b.id] || !R->mesh.pos_measured[b.id]) continue;
        float sx, sy;
        scene_landmark_pos((LandmarkId)(LM_BEACON_1 + i), &sx, &sy);
        px[n] = R->mesh.bx[b.id]; py[n] = R->mesh.by[b.id];
        qx[n] = sx; qy[n] = sy; n++;
    }
    MantisAlign a;
    if (!mantis_align_fit(px, py, qx, qy, n, s_align.mirror, &a)) { s_aligned = false; return; }
    // Beyond this the two frames disagree about where the beacons are:
    // drawing mesh output on the radar would put people in wrong places.
    s_aligned = (a.rms < 0.30f);
    s_align = a;
}

static void feed_alerts() {
    if (g_app.state != ST_DASHBOARD) return;
    if (mesh_anchor_believed() > 0) {
        g_app.alert_latched = true;
        g_app.last_alert_ms = millis();
    }
}

static void send_status(uint32_t now);

void mesh_anchor_tick() {
    if (!R) return;
    const uint32_t now = millis();
    pump(now);
    R->deploy.cardputers = mesh_anchor_probe_present() ? 1 : 0;
    R->deploy.tdisplays  = g_app.peer.peer_present ? 2 : 1;
    R->deploy.docked     = g_app.peer.peer_present && !g_app.probe_undocked;
    R->caps = mantis_caps(&R->deploy);
    if (mantis_rx_solve(R, now, 0.1f)) feed_alerts();
    realign(now);
    if (mesh_anchor_probe_present() && serves_probe() && now - s_status_ms >= 250) {
        s_status_ms = now;
        send_status(now);
    }
}

bool mesh_anchor_present()   { return R && mantis_mesh_live_count(&R->mesh) > 0; }
uint8_t mesh_anchor_live()   { return R ? mantis_mesh_live_count(&R->mesh) : 0; }
bool mesh_anchor_aligned()   { return R && s_aligned; }
float mesh_anchor_align_rms(){ return s_align.rms; }

const char *mesh_anchor_state() {
    if (!R) return "mesh off";
    return mantis_rx_state(R, millis());
}

int mesh_anchor_contacts(MeshContact *out, int max) {
    if (!R || !s_aligned) return 0;
    int n = 0;
    for (int i = 0; i < MANTIS_FUSE_MAX && n < max; i++) {
        const MantisFused &t = R->fusion.t[i];
        if (!t.active) continue;
        MeshContact &c = out[n++];
        mantis_align_apply(&s_align, t.x, t.y, &c.x, &c.y);
        c.conf = t.confidence; c.cls = (uint8_t)t.cls;
        c.believed = mantis_fuse_believed(&t);
    }
    return n;
}

int mesh_anchor_believed() {
    if (!R) return 0;
    int n = 0;
    for (int i = 0; i < MANTIS_FUSE_MAX; i++)
        if (mantis_fuse_believed(&R->fusion.t[i])) n++;
    return n;
}

bool mesh_anchor_corroborates(float sx, float sy) {
    MeshContact c[MANTIS_FUSE_MAX];
    const int n = mesh_anchor_contacts(c, MANTIS_FUSE_MAX);
    // Mesh tomography resolves about a Fresnel half-width; allow that plus
    // the alignment residual.
    const float r = 0.30f + s_align.rms;
    for (int i = 0; i < n; i++)
        if (c[i].believed && hypotf(c[i].x - sx, c[i].y - sy) < r) return true;
    return false;
}

// ── Drawing ───────────────────────────────────────────────────
void mesh_anchor_draw_overlay(int cx, int cy, float ppu, int y0, int y1) {
    if (!R || !s_aligned) return;
    auto &g = gfx_sprite();
    const MantisMesh &m = R->mesh;
    float hi = 0;
    for (int i = 0; i < MANTIS_TOMO_CELLS; i++) if (m.field[i] > hi) hi = m.field[i];
    if (hi > MANTIS_MESH_MIN_LLR * 0.5f) {
        const float step = 2.0f * m.extent / (float)MANTIS_TOMO_DIM;
        const int rad = (int)(step * s_align.scale * ppu * 0.45f) + 1;
        for (int gy = 0; gy < MANTIS_TOMO_DIM; gy++)
            for (int gx = 0; gx < MANTIS_TOMO_DIM; gx++) {
                const float v = m.field[gy * MANTIS_TOMO_DIM + gx] / hi;
                if (v < 0.45f) continue;
                float sx, sy;
                mantis_align_apply(&s_align, -m.extent + (gx + 0.5f) * step,
                                   m.extent - (gy + 0.5f) * step, &sx, &sy);
                const int px = cx + (int)(sx * ppu), py = cy - (int)(sy * ppu);
                if (py < y0 || py > y1 || px < 0 || px >= SCREEN_W) continue;
                g.fillCircle(px, py, rad > 4 ? 4 : rad,
                             v > 0.8f ? COL_MS_VIOLET_BRIGHT : COL_MS_VIOLET);
            }
    }
    MeshContact c[MANTIS_FUSE_MAX];
    const int n = mesh_anchor_contacts(c, MANTIS_FUSE_MAX);
    for (int i = 0; i < n; i++) {
        const int px = cx + (int)(c[i].x * ppu), py = cy - (int)(c[i].y * ppu);
        if (py < y0 || py > y1) continue;
        const uint16_t col = c[i].believed
            ? (c[i].cls == MFC_STATIC ? COL_MS_TEAL_BRIGHT : COL_MS_LIME)
            : COL_MS_VIOLET_BRIGHT;
        // A DIAMOND: the mesh's mark, distinct from the anchor's round
        // track rings, so the operator can always tell which instrument
        // is speaking.
        g.drawLine(px, py - 6, px + 6, py, col); g.drawLine(px + 6, py, px, py + 6, col);
        g.drawLine(px, py + 6, px - 6, py, col); g.drawLine(px - 6, py, px, py - 6, col);
        if (c[i].believed) g.fillCircle(px, py, 2, col);
        if (c[i].cls == MFC_STATIC) g.drawCircle(px, py, 9, col);
    }
}

void mesh_anchor_draw_view() {
    auto &g = gfx_sprite();
    int y = CONTENT_Y + 4;
    auto line = [&](uint16_t col, const char *txt) {
        g.setTextColor(col, COL_MS_BG);
        g.setCursor(4, y); g.print(txt); y += 16;
    };
    char b[48];
    g.setFont(&fonts::Font2);
    if (!R) { line(COL_MS_ALERT, "mesh receiver not allocated"); return; }
    const uint8_t live = mantis_mesh_live_count(&R->mesh);
    snprintf(b, sizeof(b), "%s", mesh_anchor_state());
    line(live ? COL_MS_LIME : COL_MS_WARN, b);
    snprintf(b, sizeof(b), "beacons %u  chords %d", live, R->mesh.n_chords);
    line(COL_MS_INK, b);
    const float gc = mantis_mesh_geometry_confidence(&R->mesh);
    snprintf(b, sizeof(b), "layout %s  stress %u", gc > 0.99f ? "SURVEYED" : "assumed", R->geom_stress);
    line(gc > 0.99f ? COL_MS_LIME : COL_MS_WARN, b);
    if (s_aligned) snprintf(b, sizeof(b), "on radar: yes (%d bcn, rms %.2f%s)",
                            s_align.n, (double)s_align.rms, s_align.mirror ? ", mirrored" : "");
    else           snprintf(b, sizeof(b), "on radar: no (need 2 surveyed)");
    line(s_aligned ? COL_MS_LIME : COL_MS_MID, b);
    snprintf(b, sizeof(b), "baseline %s", R->baseline_latched
             ? (R->baseline_partial ? "PARTIAL" : "ready") : "learning");
    line(R->baseline_latched && !R->baseline_partial ? COL_MS_INK : COL_MS_WARN, b);
    snprintf(b, sizeof(b), "reports %lu  geom %lu  bad %lu", (unsigned long)R->reports_in,
             (unsigned long)R->geom_in, (unsigned long)R->rejected);
    line(COL_MS_MID, b);

    // Per-beacon rows.
    y += 4;
    for (uint8_t id = 1; id <= MANTIS_MAX_BEACON_ID; id++) {
        if (!R->mesh.live[id]) continue;
        uint16_t uid = 0;
        for (uint8_t k = 1; k < MANTIS_SLOTS; k++)
            if (R->uid_bind[k].present && R->uid_bind[k].beacon_id == id) uid = R->uid_bind[k].uid;
        const bool sus = R->integrity.geometry_suspect && R->integrity.worst_id == id;
        snprintf(b, sizeof(b), "B%u %04X %s%s", id, uid,
                 R->mesh.pos_measured[id] ? "surveyed" : "assumed", sus ? " MOVED" : "");
        line(sus ? COL_MS_ALERT : COL_MS_INK, b);
    }

    // Contacts.
    y += 4;
    int shown = 0;
    for (int i = 0; i < MANTIS_FUSE_MAX; i++) {
        const MantisFused &t = R->fusion.t[i];
        if (!t.active) continue;
        snprintf(b, sizeof(b), "%s  %d%%  %u wit", mantis_fuse_class_name(t.cls),
                 (int)(t.confidence * 100), t.witnesses);
        line(mantis_fuse_believed(&t) ? COL_MS_LIME : COL_MS_MID, b); shown++;
    }
    if (!shown) line(COL_MS_MID, "no mesh contacts");

    if (mesh_anchor_probe_present()) {
        snprintf(b, sizeof(b), "probe: %s", s_probe_kind == 2 ? "Cardputer ADV" : "Core2");
        line(COL_MS_TEAL_BRIGHT, b);
    }
    if (R->caps.blocker && R->caps.blocker[0]) {
        snprintf(b, sizeof(b), "next: %s", R->caps.blocker);
        line(COL_MS_WARN, b);
    }
}

// ── Hand-held probes ──────────────────────────────────────────
void mesh_anchor_note_probe(uint8_t kind) {
    s_probe_ms = millis();
    if (kind) s_probe_kind = kind;
}
bool mesh_anchor_probe_present() {
    return s_probe_ms != 0 && (millis() - s_probe_ms) < 4000;
}
uint8_t mesh_anchor_probe_kind() { return s_probe_kind; }

void mesh_anchor_remote_key(uint8_t key, uint16_t seq) {
    mesh_anchor_note_probe(0);
    if (!serves_probe()) return;
    if (seq == s_key_seq) return;          // a retry of a press we applied
    s_key_seq = seq;
    s_key_ms  = millis();
    switch (key) {
        case 1: input_inject(BTN_LEFT,  false); break;
        case 2: input_inject(BTN_RIGHT, false); break;
        case 3: input_inject(BTN_LEFT,  true);  break;
        case 4: input_inject(BTN_RIGHT, true);  break;
        default: break;
    }
    // Answer immediately rather than at the next 250 ms tick: the probe
    // is waiting to show the press landed.
    s_status_ms = 0;
}

// What the anchor is asking the operator to do right now.
static void fill_hint(PeerAnchorStatus &s) {
    const char *h = "";
    s.progress = 0; s.step = 0; s.steps = 0;
    if (wizard_active()) {
        const WizardStep *st = wizard_current_step();
        s.step  = (uint8_t)(wizard_current_index() + 1);
        s.steps = (uint8_t)wizard_total_steps();
        s.progress = (uint8_t)(wizard_overall_progress() * 100.0f);
        if (st) h = (wizard_current_phase() == WP_READY && st->ready_prompt)
                        ? st->ready_prompt : (st->instruction ? st->instruction : st->title);
    } else {
        switch (g_app.state) {
            case ST_DISCOVERY:        h = "Finding beacons - power them on"; break;
            case ST_GEOMETRY_GUIDE:   h = "Place beacons around the room"; break;
            case ST_CAL_INTRO:        h = "Calibration: walk the room with the probe"; break;
            case ST_CAL_ANCHOR_PLACE: h = "Set the anchor at its final spot"; break;
            case ST_CAL_EMPTY_ROOM:
                h = "Leave the room - learning it empty";
                s.progress = (uint8_t)(csi_baseline_progress() * 100.0f); break;
            case ST_CAL_FINALIZE:     h = "Training the model..."; break;
            case ST_CAL_RESULTS:      h = "Review calibration, then accept"; break;
            case ST_DASHBOARD:        h = scene_cal_complete() ? "Watching"
                                        : "Not calibrated: mesh only"; break;
            case ST_TACTICAL_INTRO:   h = "Tactical deploy: carry beacons out"; break;
            case ST_TACTICAL_DEPLOY:  h = "Place this beacon, then confirm"; break;
            case ST_TACTICAL_RETURN:  h = "Walk back to the anchor"; break;
            case ST_TACTICAL_CIRCUIT: h = "Walk one lap of the perimeter"; break;
            case ST_TACTICAL_BASELINE:h = "Stand clear - learning the room"; break;
            case ST_TACTICAL_CRITIQUE:h = "Building the RF chart"; break;
            case ST_TACTICAL_CHECK:   h = "Stand where the anchor asks"; break;
            case ST_SETTINGS:         h = "Anchor settings"; break;
            default: break;
        }
    }
    strncpy(s.hint, h, sizeof(s.hint) - 1);
}

static void put_track(PeerAnchorStatus &s, int k, float x, float y,
                      float conf, uint8_t flags) {
    if (k >= ANCHOR_STATUS_TRACKS) return;
    if (s_aligned && (s.flags & AS_FRAME_MESH))
        mantis_align_invert(&s_align, x, y, &x, &y);
    auto q10 = [](float v) -> int16_t {
        v *= 1024.0f; if (v > 32000.f) v = 32000.f; if (v < -32000.f) v = -32000.f;
        return (int16_t)v; };
    s.track[k].x_q10 = q10(x); s.track[k].y_q10 = q10(y);
    const float c = conf < 0 ? 0 : (conf > 1 ? 1 : conf);
    s.track[k].conf  = (uint8_t)(c * 255.0f);
    s.track[k].flags = (uint8_t)(flags | AT_ACTIVE);
}

static void send_status(uint32_t now) {
    PeerAnchorStatus s = {};
    s.magic      = PEER_ANCHOR_STATUS_MAGIC;
    s.uptime_ms  = now;
    s.app_state  = (uint8_t)g_app.state;
    s.dash_view  = (uint8_t)g_app.dash_view;
    s.remote_ack = s_key_seq;
    s.beacons    = (uint8_t)g_app.beacon_count;
    s.mesh_beacons  = mesh_anchor_live();
    s.mesh_contacts = (uint8_t)mesh_anchor_believed();
    if (scene_cal_complete())               s.flags |= AS_CAL_COMPLETE;
    if (tactical_ready())                   s.flags |= AS_TACTICAL;
    if (g_app.alert_latched)                s.flags |= AS_ALERT;
    if (R && R->baseline_latched)           s.flags |= AS_MESH_BASELINE;
    if (g_app.peer.peer_present)            s.flags |= AS_STEREO;
    if (now - s_key_ms < 5000 && s_key_ms)  s.flags |= AS_REMOTE_DRIVES;

    const char *t = "", *l = "", *r = "";
    ui_last_chrome(&t, &l, &r);
    strncpy(s.title, t, sizeof(s.title) - 1);
    strncpy(s.left,  l, sizeof(s.left)  - 1);
    strncpy(s.right, r, sizeof(s.right) - 1);
    fill_hint(s);

    // WHICH FRAME the positions travel in.
    //   calibrated + aligned   the MESH frame: the probe overlays the
    //                          anchor's tracks on its own mesh map
    //   calibrated, unaligned  the anchor's frame (anchor at the origin)
    //   tactical chart         the anchor's frame
    //   not calibrated         the MESH frame, mesh contacts only -- the
    //                          anchor still has something true to say
    const bool scene_src = scene_cal_complete();
    const bool tac_src   = !scene_src && tactical_ready();
    if ((scene_src && s_aligned) || (!scene_src && !tac_src)) s.flags |= AS_FRAME_MESH;
    int16_t axq = 0x7FFF, ayq = 0x7FFF;          // 0x7FFF = unknown
    if (!(s.flags & AS_FRAME_MESH)) { axq = 0; ayq = 0; }
    else if (s_aligned) {
        float ax, ay; mantis_align_invert(&s_align, 0, 0, &ax, &ay);
        axq = (int16_t)(ax * 1024.0f); ayq = (int16_t)(ay * 1024.0f);
    }
    s.anchor_x_q10 = axq; s.anchor_y_q10 = ayq;

    int k = 0;
    if (scene_src) {
        for (int i = 0; i < TRACK_MAX && k < ANCHOR_STATUS_TRACKS; i++) {
            const TargetTrack *tr = scene_get_track(i);
            if (!tr || !tr->active) continue;
            uint8_t f = 0;
            if (tr->is_self) f |= AT_SELF;
            if (mesh_anchor_corroborates(tr->pos[0], tr->pos[1])) f |= AT_CORROBORATED;
            put_track(s, k++, tr->pos[0], tr->pos[1], tr->confidence, f);
        }
    } else if (tac_src) {
        const TacticalStatus &ts = tactical_status();
        for (int i = 0; i < MANTIS_RF_MAX_TRACKS && k < ANCHOR_STATUS_TRACKS; i++) {
            if (!ts.tracks[i].active) continue;
            put_track(s, k++, ts.tracks[i].x, ts.tracks[i].y, ts.tracks[i].confidence, 0);
        }
    }
    // Mesh contacts are already IN the mesh frame; they ride along
    // whenever that is the frame, flagged as the mesh's own mark.
    if (R && (s.flags & AS_FRAME_MESH)) {
        for (int i = 0; i < MANTIS_FUSE_MAX && k < ANCHOR_STATUS_TRACKS; i++) {
            const MantisFused &f = R->fusion.t[i];
            if (!mantis_fuse_believed(&f)) continue;
            s.track[k].x_q10 = (int16_t)(f.x * 1024.0f);
            s.track[k].y_q10 = (int16_t)(f.y * 1024.0f);
            s.track[k].conf  = (uint8_t)(f.confidence * 255.0f);
            s.track[k].flags = AT_ACTIVE | AT_MESH_ONLY;
            k++;
        }
    }
    s.n_tracks = (uint8_t)k;
    esp_now_send(BCAST, (const uint8_t *)&s, sizeof(s));
}
