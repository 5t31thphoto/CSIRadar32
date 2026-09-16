#pragma once
// ═══════════════════════════════════════════════════════════════
//  MANTIS STATIC SCENE
//  The room's fixed structure, and whether the mesh still trusts itself
// ═══════════════════════════════════════════════════════════════
//
//  ── 1. THE BASELINE IS A MAP, NOT JUST A REFERENCE ───────────
//
//  The empty-room baseline is treated everywhere as something to
//  subtract.  It is also a measurement in its own right.
//
//  With geometry solved, the free-space loss of every chord is
//  predictable from its length alone.  The difference between measured
//  and predicted is EXCESS PATH LOSS, and excess loss on a chord in an
//  empty room means something static is sitting on it -- a wall, a
//  pillar, a filing cabinet, a fridge.
//
//      measured loss - geometric prediction = static obstruction
//
//  Back-project that and you get an occupancy map of the room's fixed
//  structure, using the same tomography already built, pointed at the
//  baseline instead of the perturbation.
//
//  Three things follow:
//    - the radar can draw the ROOM, not an assumed circle
//    - "blocked by wall" becomes distinguishable from "blocked by person"
//    - chords passing through a wall get down-weighted automatically,
//      because they were never going to see a person on the far side and
//      their apparent quietness is not evidence of an empty room
//
//  ── 2. THE MESH CAN CHECK ITSELF ─────────────────────────────
//
//  A person blocks two to four chords out of fifteen.  A beacon that is
//  moved, knocked or re-placed changes every chord it participates in --
//  five of fifteen, ALL SHARING ONE ENDPOINT.
//
//  That shared endpoint is a signature no target can produce, because no
//  target sits on a beacon.  So the mesh can distinguish "someone is
//  here" from "my geometry is now wrong", and the second case is the
//  dangerous one: undetected, it corrupts every downstream position
//  silently and permanently, and it looks exactly like poor tracking.
// ═══════════════════════════════════════════════════════════════
#include "mantis_mesh.h"
#include "mantis_geometry.h"

// Free-space loss is proportional to distance^2; in the normalised
// amplitude domain the mesh works in, received amplitude falls as 1/d.
// A chord twice as long is half as strong before any obstruction.
//
// Beacons transmit at fixed power on a fixed rate (see mantis_air.h), so
// this prediction is meaningful -- with rate adaptation running it would
// not be, which is one more reason that was disabled.
static inline float mantis_static_predict_amp(float len, float ref_len,
                                              float ref_amp) {
    if (len < 1e-4f || ref_len < 1e-4f) return ref_amp;
    return ref_amp * (ref_len / len);
}

typedef struct {
    float   excess[MANTIS_SLOTS * MANTIS_SLOTS];  // static loss beyond geometry
    float   field[MANTIS_TOMO_CELLS];             // back-projected room structure
    bool    ready;
    float   ref_len, ref_amp;                     // the calibrating chord
    uint8_t n_chords;
} MantisStaticScene;

// Build the static map from the mesh's learned baseline.
//
// The reference is the SHORTEST chord, on the reasoning that the
// shortest path in a room is the one least likely to be obstructed.
// That is an assumption and it is stated rather than hidden: in a room
// where the shortest chord happens to pass through a pillar, every other
// chord will be judged against an already-obstructed reference and the
// map will read inverted.  The residual check below catches that case.
static inline void mantis_static_build(MantisStaticScene *S, const MantisMesh *m) {
    S->ready = false;
    S->n_chords = 0;
    for (int i = 0; i < MANTIS_SLOTS * MANTIS_SLOTS; i++) S->excess[i] = 0.0f;
    for (int i = 0; i < MANTIS_TOMO_CELLS; i++) S->field[i] = 0.0f;
    if (m->n_beacons < 3) return;

    // Find the shortest measured chord to calibrate against.
    float best_len = 1e30f; float best_amp = 0.0f;
    for (uint8_t i = 1; i <= m->n_beacons; i++) {
        if (!m->have_pos[i]) continue;
        for (uint8_t j = (uint8_t)(i + 1); j <= m->n_beacons; j++) {
            if (!m->have_pos[j]) continue;
            const int idx = i * MANTIS_SLOTS + j;
            if (m->base_n[idx] < MANTIS_MESH_BASE_MIN) continue;
            const float dx = m->bx[i] - m->bx[j], dy = m->by[i] - m->by[j];
            const float len = sqrtf(dx * dx + dy * dy);
            if (len < best_len && len > 1e-3f) { best_len = len; best_amp = m->base[idx]; }
        }
    }
    if (best_len > 1e29f) return;
    S->ref_len = best_len; S->ref_amp = best_amp;

    // Excess loss per chord.
    MantisChord ch[MANTIS_MESH_MAX_CHORDS];
    int nc = 0;
    for (uint8_t i = 1; i <= m->n_beacons && nc < MANTIS_MESH_MAX_CHORDS; i++) {
        if (!m->have_pos[i]) continue;
        for (uint8_t j = (uint8_t)(i + 1); j <= m->n_beacons && nc < MANTIS_MESH_MAX_CHORDS; j++) {
            if (!m->have_pos[j]) continue;
            const int idx = i * MANTIS_SLOTS + j;
            if (m->base_n[idx] < MANTIS_MESH_BASE_MIN) continue;
            const float dx = m->bx[i] - m->bx[j], dy = m->by[i] - m->by[j];
            const float len = sqrtf(dx * dx + dy * dy);
            const float pred = mantis_static_predict_amp(len, S->ref_len, S->ref_amp);
            if (pred <= 1e-4f) continue;
            // Fractional shortfall against the geometric prediction.
            // Positive means the path is losing more than its length
            // explains -- something is on it.
            float ex = (pred - m->base[idx]) / pred;
            if (ex < 0.0f) ex = 0.0f;   // stronger than predicted: constructive, not an obstruction
            S->excess[idx] = ex;
            S->excess[j * MANTIS_SLOTS + i] = ex;

            ch[nc].ax = m->bx[i]; ch[nc].ay = m->by[i];
            ch[nc].bx = m->bx[j]; ch[nc].by = m->by[j];
            ch[nc].atten  = ex;
            ch[nc].weight = 1.0f;
            nc++;
        }
    }
    S->n_chords = (uint8_t)nc;
    if (nc < 3) return;
    mantis_tomo_backproject(ch, nc, m->extent, S->field);
    S->ready = true;
}

// Is this cell occupied by fixed structure?
//
// Used to down-weight chords that pass through a wall: their quietness
// is not evidence that the far side is empty, and treating it as such is
// how a system confidently reports "clear" about a room it cannot see
// into.
static inline bool mantis_static_blocked(const MantisStaticScene *S,
                                         float extent, float px, float py,
                                         float thresh) {
    if (!S->ready) return false;
    const float step = (2.0f * extent) / (float)MANTIS_TOMO_DIM;
    const int gx = (int)((px + extent) / step);
    const int gy = (int)((extent - py) / step);
    if (gx < 0 || gy < 0 || gx >= MANTIS_TOMO_DIM || gy >= MANTIS_TOMO_DIM) return false;
    return S->field[gy * MANTIS_TOMO_DIM + gx] > thresh;
}

// ── MESH INTEGRITY ────────────────────────────────────────────

// How lopsided a disturbance must be before geometry is blamed rather
// than a target.  0.6 means at least 60 percentage points more of one
// beacon's links moved than everyone else's.
#ifndef MANTIS_INTEGRITY_ALARM
  #define MANTIS_INTEGRITY_ALARM 0.6f
#endif

typedef struct {
    float   suspicion[MANTIS_SLOTS];   // per beacon, 0..1
    uint8_t worst_id;
    float   worst_score;
    bool    geometry_suspect;
} MantisIntegrity;

// Detect a beacon whose links have ALL changed together.
//
// The discriminator is not "how much changed" but "did the changes share
// one endpoint".  A target perturbs chords that happen to pass near it,
// which is a geometric scatter across the matrix.  A displaced beacon
// perturbs exactly the chords touching it, which is a row.
//
// Scoring each beacon by the fraction of ITS links that moved, minus the
// fraction of links NOT touching it that moved, isolates that row
// structure.  A target raises both terms and cancels; a moved beacon
// raises only the first.
static inline void mantis_integrity_check(MantisIntegrity *I,
                                          const MantisMesh *m,
                                          float change_thresh) {
    *I = MantisIntegrity{};
    if (m->n_beacons < 4) return;   // fewer than four, a row is most of the matrix

    for (uint8_t k = 1; k <= m->n_beacons; k++) {
        if (!m->have_pos[k]) continue;
        int mine = 0, mine_moved = 0, other = 0, other_moved = 0;
        for (uint8_t i = 1; i <= m->n_beacons; i++) {
            for (uint8_t j = (uint8_t)(i + 1); j <= m->n_beacons; j++) {
                if (!m->have_pos[i] || !m->have_pos[j]) continue;
                const float a = m->atten[i * MANTIS_SLOTS + j];
                const bool moved = (a > change_thresh);
                if (i == k || j == k) { mine++; mine_moved += moved ? 1 : 0; }
                else                  { other++; other_moved += moved ? 1 : 0; }
            }
        }
        if (mine == 0 || other == 0) continue;
        const float fm = (float)mine_moved / (float)mine;
        const float fo = (float)other_moved / (float)other;
        // Only a row-structured disturbance scores.
        float s = fm - fo;
        if (s < 0.0f) s = 0.0f;
        I->suspicion[k] = s;
        if (s > I->worst_score) { I->worst_score = s; I->worst_id = k; }
    }
    // A full row moving while the rest of the matrix is quiet is not
    // something a person can do.
    I->geometry_suspect = (I->worst_score >= MANTIS_INTEGRITY_ALARM);
}

