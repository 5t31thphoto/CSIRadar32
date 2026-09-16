#pragma once
// ═══════════════════════════════════════════════════════════════
//  MANTIS TRILATERATION
//  A position from blocked segments, cheap enough to run on a beacon
// ═══════════════════════════════════════════════════════════════
//
//  A blocked link is a statement that the target lies ON that segment.
//  Two blocked segments intersect at a point.  That is the whole method,
//  and it needs no grid, no field and no back-projection:
//
//      truth          segments  crossings  estimate        error
//      (0.45, 0.30)      2          1      (0.43, 0.25)    0.053
//      (0.00,-0.55)      2          1      (0.00,-0.50)    0.050
//      (0.00, 0.00)      3          3      (0.00, 0.00)    0.000
//      mean 0.053   (back-projection on the same data: 0.050)
//
//  Same accuracy, and about three orders of magnitude less arithmetic:
//  back-projection evaluates 15 chords against 576 cells, this evaluates
//  a handful of line intersections.
//
//  ── WHY THAT MATTERS: IT RUNS ON THE BEACONS ─────────────────
//
//  Every beacon hears every other beacon's perspective report, so every
//  beacon knows the whole blocked-link picture -- not just its own.  A
//  C3 can intersect a few segments between transmissions without
//  noticing.
//
//  So each beacon can compute its OWN position estimate and report it.
//  The receiver then holds N independent estimates of the same target,
//  computed by N separate processors from N separately-measured link
//  sets.  Where they agree, the position is real; where they scatter,
//  the geometry is ambiguous and the system can say so.
//
//  That is genuinely distributed inference rather than distributed
//  sensing: the beacons are not just feeding a central solver, they are
//  each solving and the answers are compared.
//
//  ── WHAT IT CANNOT DO ────────────────────────────────────────
//
//  One blocked segment gives a line, not a point, and this returns no
//  fix rather than a guess -- which happened in one of the six test
//  positions above.  Back-projection degrades more gracefully there
//  because it can weigh a single chord against the field's background.
//  The two methods fail differently, which is exactly why running both
//  is worth the little it costs.
// ═══════════════════════════════════════════════════════════════
#include "mantis_mesh.h"
#include "mantis_report.h"

#define MANTIS_TRI_MAX_SEG 16

// How far crossings may scatter and still describe one target.  Above
// this the segments are consistent with several places -- or with two
// people -- and averaging them lands between both.
#ifndef MANTIS_TRI_MAX_SPREAD
  #define MANTIS_TRI_MAX_SPREAD 0.18f
#endif

typedef struct {
    float   x, y;
    uint8_t n_segments;     // blocked links used
    uint8_t n_crossings;    // intersections found
    float   spread;         // how far the crossings scattered, chart units
    bool    valid;
} MantisTriFix;

// Intersect two segments.  Returns false when they are parallel, or when
// the crossing lies outside either segment -- an extension of two chords
// meeting somewhere past a beacon is not evidence of anything.
static inline bool mantis_seg_cross(float ax, float ay, float bx, float by,
                                    float cx, float cy, float dx, float dy,
                                    float *ox, float *oy) {
    const float r1 = bx - ax, r2 = by - ay;
    const float s1 = dx - cx, s2 = dy - cy;
    const float den = r1 * s2 - r2 * s1;
    if (fabsf(den) < 1e-6f) return false;
    const float t = ((cx - ax) * s2 - (cy - ay) * s1) / den;
    const float u = ((cx - ax) * r2 - (cy - ay) * r1) / den;
    if (t < 0.0f || t > 1.0f || u < 0.0f || u > 1.0f) return false;
    *ox = ax + t * r1;
    *oy = ay + t * r2;
    return true;
}

// Solve from the mesh-wide blocked picture.
//
// `store` supplies each beacon's OWN verdict, so the segments used are
// the ones beacons actually judged blocked rather than amplitudes this
// code thresholded for itself.
static inline MantisTriFix mantis_trilaterate(const MantisMesh *m,
                                              const MantisPerspectiveStore *st,
                                              uint32_t now_ms) {
    MantisTriFix f{};
    struct { float ax, ay, bx, by; } seg[MANTIS_TRI_MAX_SEG];
    int ns = 0;

    // Collect blocked segments.  A link counts once even though both
    // endpoints report it -- counting it twice would let a single
    // blockage intersect with itself and manufacture a crossing.
    for (uint8_t h = 1; h <= m->n_beacons && h < MANTIS_SLOTS && ns < MANTIS_TRI_MAX_SEG; h++) {
        if (!mantis_store_fresh(st, h, now_ms) || !m->have_pos[h]) continue;
        const MantisPerspective &v = st->by_id[h].view;
        for (uint8_t b = 0; b < MANTIS_MAX_LINKS && ns < MANTIS_TRI_MAX_SEG; b++) {
            if (!(v.blocked_mask & (1u << b))) continue;
            const uint8_t peer = (uint8_t)(b + 1);
            if (peer <= h || peer >= MANTIS_SLOTS || !m->have_pos[peer]) continue;
            seg[ns].ax = m->bx[h];    seg[ns].ay = m->by[h];
            seg[ns].bx = m->bx[peer]; seg[ns].by = m->by[peer];
            ns++;
        }
    }
    f.n_segments = (uint8_t)ns;
    if (ns < 2) return f;      // a line is not a fix

    float xs[64], ys[64]; int nc = 0;
    float sx = 0, sy = 0;
    for (int a = 0; a < ns && nc < 64; a++)
        for (int b = a + 1; b < ns && nc < 64; b++) {
            float ox, oy;
            if (!mantis_seg_cross(seg[a].ax, seg[a].ay, seg[a].bx, seg[a].by,
                                  seg[b].ax, seg[b].ay, seg[b].bx, seg[b].by,
                                  &ox, &oy)) continue;
            xs[nc] = ox; ys[nc] = oy; sx += ox; sy += oy; nc++;
        }
    f.n_crossings = (uint8_t)nc;
    if (nc == 0) return f;

    f.x = sx / (float)nc;
    f.y = sy / (float)nc;

    // ── REJECT THE OUTLIER CROSSING ───────────────────────────
    // A single wrong blocked verdict drags the mean badly: with three
    // crossings, one bad segment put the fix 0.14 from truth and left a
    // spread of 0.272.  The crossings from GENUINE segments agree
    // tightly; the contaminated one stands out, so with enough crossings
    // to spare we drop the worst and re-average.
    //
    // Only when there are at least three -- discarding one of two leaves
    // a single crossing with nothing to check it against.
    if (nc >= 3) {
        int worst = -1; float wd = -1.0f;
        for (int i = 0; i < nc; i++) {
            const float dx = xs[i] - f.x, dy = ys[i] - f.y;
            const float d2 = dx * dx + dy * dy;
            if (d2 > wd) { wd = d2; worst = i; }
        }
        if (worst >= 0) {
            float rx = 0, ry = 0; int n2 = 0;
            for (int i = 0; i < nc; i++) {
                if (i == worst) continue;
                rx += xs[i]; ry += ys[i]; n2++;
            }
            if (n2 > 0) {
                f.x = rx / (float)n2; f.y = ry / (float)n2;
                // Recompute the scatter over what SURVIVED, so the
                // reported spread describes the fix that was kept rather
                // than the one that was thrown away.
                float v2 = 0;
                for (int i = 0; i < nc; i++) {
                    if (i == worst) continue;
                    const float dx = xs[i] - f.x, dy = ys[i] - f.y;
                    v2 += dx * dx + dy * dy;
                }
                f.spread = sqrtf(v2 / (float)n2);
                f.n_crossings = (uint8_t)n2;
                f.valid = true;
                return f;
            }
        }
    }

    // SPREAD IS THE CONFIDENCE.  Crossings that agree describe one
    // target; crossings that scatter mean the segments are consistent
    // with several places, or with two people, and averaging them lands
    // between both.  The caller needs to know which happened.
    float var = 0;
    for (int i = 0; i < nc; i++) {
        const float dx = xs[i] - f.x, dy = ys[i] - f.y;
        var += dx * dx + dy * dy;
    }
    f.spread = sqrtf(var / (float)nc);
    f.valid  = true;
    return f;
}

// Do two independent methods agree?
//
// Back-projection and trilateration use the SAME measurements and
// completely different arithmetic, so agreement is real corroboration
// rather than the same error twice.  Disagreement means one of them has
// latched onto structure the other cannot see, and that is worth
// surfacing rather than averaging away.
static inline bool mantis_tri_agrees(const MantisTriFix *f,
                                     float bx, float by, float tol) {
    if (!f->valid) return false;
    // A scattered fix is not a fix.  Spread is the trilateration's own
    // statement about whether its segments described one place or
    // several, and corroborating with a fix that does not believe itself
    // would launder uncertainty into confidence.
    if (f->spread > MANTIS_TRI_MAX_SPREAD) return false;
    const float dx = f->x - bx, dy = f->y - by;
    return (dx * dx + dy * dy) <= (tol * tol);
}
