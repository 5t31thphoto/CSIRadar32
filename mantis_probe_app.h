#pragma once
// ═══════════════════════════════════════════════════════════════
//  MANTIS PROBE APP — shared by the Core2 and the Cardputer ADV
// ═══════════════════════════════════════════════════════════════
//
//  Two devices, one probe.  Everything that decides WHAT the operator is
//  told lives here, so the two can differ only in how it looks on their
//  glass and which keys drive it:
//
//    ALARMS     edge-triggered from the fused mesh view and the anchor's
//               own status.  The previous firmwares had a complete alarm
//               table, per-alarm enables, MP3 files, tone fallbacks and a
//               vibration motor -- and nothing ever FIRED one except the
//               preview in the settings menu.
//
//    THE MAP    one renderer, templated on the canvas, drawing the
//               probe's own mesh contacts together with the anchor's
//               tracks in the same (surveyed) frame.
//
//    THE REMOTE the anchor's screen, mirrored: what it is doing, what it
//               wants, what its two buttons do right now.
// ═══════════════════════════════════════════════════════════════
#include "mantis_receiver.h"
#include "mantis_control.h"
#include "mantis_alarms.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

// ── Alarm policy ──────────────────────────────────────────────
typedef struct {
    uint8_t  believed;          // fused mesh contacts believed last time
    uint8_t  anchor_tracks;     // confident anchor tracks last time
    bool     anchor_alert;
    bool     anchor_live;
    bool     moving;
    bool     suspect;
    uint8_t  live_beacons;
    bool     primed;            // first evaluation only records state
} MantisAlarmWatch;

static inline uint8_t mantis_anchor_confident_tracks(const MantisProbeLink *L) {
    uint8_t n = 0;
    for (int i = 0; i < L->st.n_tracks && i < MANTIS_AS_TRACKS; i++) {
        const uint8_t f = L->st.track[i].flags;
        if ((f & MAT_ACTIVE) && !(f & MAT_SELF) && !(f & MAT_MESH_ONLY)
            && L->st.track[i].conf > 128) n++;
    }
    return n;
}

// Returns the ONE most important alarm that has just become true, or
// CPA_NONE.  Call at the solve rate.  Rising edges only: a contact that
// stays in the room is one alarm, not ten a second.
static inline CardputerAlarm mantis_alarm_eval(MantisAlarmWatch *w,
                                               const MantisReceiver *rx,
                                               const MantisProbeLink *L,
                                               uint32_t now_ms) {
    uint8_t believed = 0;
    for (int i = 0; i < MANTIS_FUSE_MAX; i++)
        if (mantis_fuse_believed(&rx->fusion.t[i])) believed++;
    const bool live   = mantis_probe_anchor_live(L, now_ms);
    const bool alert  = live && (L->st.flags & MAS_ALERT);
    const uint8_t atr = live ? mantis_anchor_confident_tracks(L) : 0;
    const bool moving = rx->fusion.anything_moving;
    const bool sus    = rx->integrity.geometry_suspect;
    const uint8_t nb  = rx->deploy.beacons;

    CardputerAlarm out = CPA_NONE;
    if (w->primed) {
        // Priority order: the most actionable first.
        if (alert && !w->anchor_alert)                          out = CPA_PERIMETER;
        else if (believed > w->believed || atr > w->anchor_tracks) out = CPA_NEW_PRESENCE;
        else if ((sus && !w->suspect) || (nb < w->live_beacons)
                 || (w->anchor_live && !live))                  out = CPA_MESH_FAULT;
        else if (moving && !w->moving)                          out = CPA_MOTION;
    }
    w->believed = believed; w->anchor_tracks = atr; w->anchor_alert = alert;
    w->anchor_live = live; w->moving = moving; w->suspect = sus;
    w->live_beacons = nb; w->primed = true;
    return out;
}

// ── Rendering ─────────────────────────────────────────────────
typedef struct {
    uint16_t bg, ink, mid, dim, lime, teal, violet, alert, warn, mesh;
} MantisProbePalette;

// Everything on one map, in the mesh's surveyed frame:
//   beacons   solid = surveyed, hollow = assumed, red = moved
//   glow      the tomography field
//   diamonds  this probe's fused contacts (filled = believed, ring = static)
//   rings     the ANCHOR's tracks, when it sends them in this frame
//             (double ring = corroborated by the mesh at the anchor)
//   square    the anchor itself
//   cross     this probe's own position
template <class G>
static inline void mantis_probe_draw_map(G &g, const MantisReceiver &rx,
                                         const MantisProbeLink &L, uint32_t now_ms,
                                         int cx, int cy, int r,
                                         const MantisProbePalette &P) {
    g.drawCircle(cx, cy, r, P.dim);
    const MantisMesh &m = rx.mesh;
    const float ex = m.extent > 0 ? m.extent : 1.2f;
    const float sc = (float)r / ex;
    auto X = [&](float x) { return cx + (int)(x * sc); };
    auto Y = [&](float y) { return cy - (int)(y * sc); };

    float hi = 0;
    for (int i = 0; i < MANTIS_TOMO_CELLS; i++) if (m.field[i] > hi) hi = m.field[i];
    if (hi > MANTIS_MESH_MIN_LLR * 0.5f) {
        const float step = (2.0f * ex) / (float)MANTIS_TOMO_DIM;
        for (int gy = 0; gy < MANTIS_TOMO_DIM; gy++)
            for (int gx = 0; gx < MANTIS_TOMO_DIM; gx++) {
                const float v = m.field[gy * MANTIS_TOMO_DIM + gx] / hi;
                if (v < 0.42f) continue;
                const int px = X(-ex + (gx + 0.5f) * step), py = Y(ex - (gy + 0.5f) * step);
                if ((px - cx) * (px - cx) + (py - cy) * (py - cy) > r * r) continue;
                g.fillCircle(px, py, v > 0.78f ? 3 : 2, P.mesh);
            }
    }

    for (uint8_t i = 1; i < MANTIS_SLOTS; i++) {
        if (!m.live[i] || !m.have_pos[i]) continue;
        const int px = X(m.bx[i]), py = Y(m.by[i]);
        const bool sus = rx.integrity.geometry_suspect && rx.integrity.worst_id == i;
        const uint16_t c = sus ? P.alert : (m.pos_measured[i] ? P.lime : P.mid);
        if (m.pos_measured[i]) g.fillCircle(px, py, 3, c); else g.drawCircle(px, py, 3, c);
        g.setTextColor(P.ink, P.bg);
        g.setCursor(px - 2, py - 11);
        g.printf("%u", (unsigned)i);
    }

    for (int i = 0; i < MANTIS_FUSE_MAX; i++) {
        const MantisFused &t = rx.fusion.t[i];
        if (!t.active) continue;
        const int px = X(t.x), py = Y(t.y);
        const bool b = mantis_fuse_believed(&t);
        const uint16_t c = !b ? P.violet : (t.cls == MFC_STATIC ? P.teal : P.lime);
        g.drawLine(px, py - 5, px + 5, py, c); g.drawLine(px + 5, py, px, py + 5, c);
        g.drawLine(px, py + 5, px - 5, py, c); g.drawLine(px - 5, py, px, py - 5, c);
        if (b) g.fillCircle(px, py, 2, c);
        if (t.cls == MFC_STATIC) g.drawCircle(px, py, 8, c);
    }

    if (mantis_probe_anchor_live(&L, now_ms) && (L.st.flags & MAS_FRAME_MESH)) {
        const bool have_own_mesh = mantis_mesh_live_count(&m) > 0;
        for (int i = 0; i < L.st.n_tracks && i < MANTIS_AS_TRACKS; i++) {
            const uint8_t f = L.st.track[i].flags;
            if (!(f & MAT_ACTIVE)) continue;
            if ((f & MAT_MESH_ONLY) && have_own_mesh) continue;   // we draw our own
            const int px = X(L.st.track[i].x_q10 / 1024.0f);
            const int py = Y(L.st.track[i].y_q10 / 1024.0f);
            const uint16_t c = (f & MAT_SELF) ? P.mid : P.teal;
            g.drawCircle(px, py, 5, c);
            g.fillCircle(px, py, 2, c);
            if (f & MAT_CORROBORATED) g.drawCircle(px, py, 8, P.lime);
            if (f & MAT_SELF) { g.setTextColor(P.mid, P.bg); g.setCursor(px + 7, py - 4); g.printf("YOU"); }
        }
        if (L.st.anchor_x_q10 != 0x7FFF) {
            const int px = X(L.st.anchor_x_q10 / 1024.0f), py = Y(L.st.anchor_y_q10 / 1024.0f);
            g.fillRect(px - 3, py - 3, 7, 7, P.teal);
        }
    }

    if (rx.probe.known) {
        const int px = X(rx.probe.x), py = Y(rx.probe.y);
        g.drawLine(px - 5, py, px + 5, py, P.violet);
        g.drawLine(px, py - 5, px, py + 5, P.violet);
    }
}

// Word-wrap `text` into lines of at most `cols` characters; returns count.
static inline int mantis_wrap(const char *text, int cols, char out[][48], int max_lines) {
    int n = 0; const char *p = text ? text : "";
    if (cols > 47) cols = 47;
    while (*p && n < max_lines) {
        while (*p == ' ') p++;
        int len = (int)strlen(p);
        int take = len <= cols ? len : cols;
        if (len > cols) {
            int cut = take;
            while (cut > 0 && p[cut] != ' ') cut--;
            if (cut > 0) take = cut;
        }
        memcpy(out[n], p, take); out[n][take] = 0;
        n++; p += take;
    }
    return n;
}
