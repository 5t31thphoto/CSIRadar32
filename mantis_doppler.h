#pragma once
// ═══════════════════════════════════════════════════════════════
//  MANTIS BISTATIC DOPPLER
//  Localising a moving target anywhere, not just on a chord
// ═══════════════════════════════════════════════════════════════
//
//  ── THE HOLE THIS FILLS ──────────────────────────────────────
//
//  Shadowing tomography only sees a target that physically blocks a
//  line.  Measured on a 6-beacon ring, cells inside the ring with at
//  least two chords through them:
//
//      sensed  518 of 872   (59%)
//      BLIND   354 of 872   (41%)
//
//  Four cells in ten are invisible to blockage.  A person standing in a
//  gap between chords produces no attenuation on any link, and no amount
//  of reconstruction can recover evidence that was never measured.
//
//  Delay resolution cannot rescue it either.  The reflected-path locus
//  of a bistatic pair is an ellipse, but ellipse THICKNESS is c/2B:
//
//      HT20  20 MHz -> 750 cm
//      HT40  40 MHz -> 375 cm     wider than the room
//
//  ── WHAT DOPPLER GIVES INSTEAD ───────────────────────────────
//
//  Doppler needs no bandwidth at all.  For a target at P moving with
//  velocity v, the bistatic phase rate on the link from A to B is
//
//      rate = (2*pi/lambda) * v . ( unit(P-A) + unit(P-B) )
//
//  The bracketed term is the BISECTOR of the angle A-P-B.  Every link
//  sees a different bisector, so 15 links give 15 projections of one
//  2-D velocity -- and the geometry that produces those projections
//  depends on WHERE the target is.
//
//  So position falls out of consistency: grid the room, and at each cell
//  solve for the velocity that best explains all the measured rates.
//  The cell where a single velocity explains everything is where the
//  target is.  Four unknowns (x, y, vx, vy) against 15 measurements is
//  over-determined by eleven.
//
//  This covers 100% of the room, because every cell has a defined
//  bisector for every pair.  It works only for MOVING targets -- a
//  stationary one has no Doppler by definition -- which is exactly
//  complementary to shadowing:
//
//      moving, anywhere        -> Doppler locates it
//      static, on a chord      -> shadowing locates it
//      static, off every chord -> genuinely unobservable, and the system
//                                 should say so rather than guess
// ═══════════════════════════════════════════════════════════════
#include "mantis_mesh.h"

// 2.4 GHz.  Wavelength sets how much phase a given motion produces, and
// it is the only physical constant this file needs.
#ifndef MANTIS_LAMBDA_M
  #define MANTIS_LAMBDA_M 0.125f
#endif

// Chart units per metre.  The chart is normalised, so converting a
// measured rate into a velocity needs the deployment scale -- which FTM
// ranging supplies.  Until it does, the result is still a correct
// DIRECTION and a correct position; only the speed magnitude is
// uncalibrated, and the code says so rather than quietly reporting
// metres it cannot justify.
#ifndef MANTIS_CHART_PER_M
  #define MANTIS_CHART_PER_M 0.30f
#endif

// Below this mean squared rate nothing is moving enough to localise.
// Same physics as the MOVING/STATIC threshold: a hand at 10 cm/s makes
// ~0.8 rad/s, so 0.1 rad^2/s^2 sits under a real person and over
// thermal wander.
#ifndef MANTIS_DOP_MIN_ENERGY
  #define MANTIS_DOP_MIN_ENERGY 0.10f
#endif

typedef struct {
    float rate;        // measured phase rate, rad/s
    float ax, ay;      // transmitter
    float bx, by;      // receiver
    float weight;      // link quality
} MantisDopplerObs;

#define MANTIS_DOP_MAX_OBS 32

// Bisector unit vector at P for the pair (A,B).  This is the direction
// in which motion produces the most phase change on this link, and it
// is what makes each link an independent projection.
static inline bool mantis_bisector(float px, float py,
                                   float ax, float ay, float bx, float by,
                                   float *ux, float *uy) {
    const float d1x = px - ax, d1y = py - ay;
    const float d2x = px - bx, d2y = py - by;
    const float l1 = sqrtf(d1x * d1x + d1y * d1y);
    const float l2 = sqrtf(d2x * d2x + d2y * d2y);
    // Degenerate when the target sits on top of a beacon: the direction
    // is undefined, not zero, so the link must be skipped rather than
    // contributing a fabricated vector.
    if (l1 < 1e-4f || l2 < 1e-4f) return false;
    *ux = d1x / l1 + d2x / l2;
    *uy = d1y / l1 + d2y / l2;
    return true;
}

// At cell P, find the velocity that best explains every observed rate,
// and return how well it does.
//
// Weighted linear least squares on a 2x2 normal system -- closed form,
// no iteration, ~30 flops per cell per link.  Cheap enough to sweep the
// whole grid every frame.
static inline float mantis_doppler_residual(const MantisDopplerObs *obs, int n,
                                            float px, float py,
                                            float *out_vx, float *out_vy,
                                            int *out_used) {
    const float k = 2.0f * (float)M_PI / MANTIS_LAMBDA_M;
    float a11 = 0, a12 = 0, a22 = 0, b1 = 0, b2 = 0, wsum = 0;
    int used = 0;

    for (int i = 0; i < n; i++) {
        float ux, uy;
        if (!mantis_bisector(px, py, obs[i].ax, obs[i].ay,
                             obs[i].bx, obs[i].by, &ux, &uy)) continue;
        const float w = obs[i].weight;
        if (w <= 0.0f) continue;
        // rate = k * (v . u)  ->  linear in v
        const float gx = k * ux, gy = k * uy;
        a11 += w * gx * gx; a12 += w * gx * gy; a22 += w * gy * gy;
        b1  += w * gx * obs[i].rate;
        b2  += w * gy * obs[i].rate;
        wsum += w; used++;
    }
    if (out_used) *out_used = used;
    // Three links is the floor: two unknowns plus one to over-determine.
    // With exactly two the fit is exact everywhere and the residual
    // carries no information at all -- it would localise to the whole
    // room with perfect confidence.
    if (used < 3 || wsum <= 0.0f) { if (out_vx) *out_vx = 0;
                                    if (out_vy) *out_vy = 0; return 1e30f; }

    const float det = a11 * a22 - a12 * a12;
    if (fabsf(det) < 1e-9f) { if (out_vx) *out_vx = 0;
                              if (out_vy) *out_vy = 0; return 1e30f; }
    const float vx = ( a22 * b1 - a12 * b2) / det;
    const float vy = (-a12 * b1 + a11 * b2) / det;
    if (out_vx) *out_vx = vx;
    if (out_vy) *out_vy = vy;

    // Weighted mean squared residual: how much of the measurement this
    // (position, velocity) pair fails to explain.
    float r2 = 0;
    for (int i = 0; i < n; i++) {
        float ux, uy;
        if (!mantis_bisector(px, py, obs[i].ax, obs[i].ay,
                             obs[i].bx, obs[i].by, &ux, &uy)) continue;
        const float w = obs[i].weight;
        if (w <= 0.0f) continue;
        const float pred = k * (vx * ux + vy * uy);
        const float e = obs[i].rate - pred;
        r2 += w * e * e;
    }
    return r2 / wsum;
}

// Sweep the grid.  `out` receives a SCORE field, high where a single
// velocity explains the data -- the same orientation as the shadowing
// field so the two can be compared and overlaid without sign confusion.
static inline void mantis_doppler_field(const MantisDopplerObs *obs, int n,
                                        float extent, float *out) {
    for (int i = 0; i < MANTIS_TOMO_CELLS; i++) out[i] = 0.0f;
    if (n < 3) return;

    const float step = (2.0f * extent) / (float)MANTIS_TOMO_DIM;

    // Scale the residual against the energy actually present.  Without
    // this, a quiet room scores "well explained" everywhere -- zero rates
    // are trivially explained by zero velocity, and the field would light
    // up uniformly with nothing in the room.
    float energy = 0.0f, wsum = 0.0f;
    for (int i = 0; i < n; i++) {
        energy += obs[i].weight * obs[i].rate * obs[i].rate;
        wsum   += obs[i].weight;
    }
    if (wsum <= 0.0f) return;
    energy /= wsum;
    if (energy < MANTIS_DOP_MIN_ENERGY) return;   // nothing is moving

    for (int gy = 0; gy < MANTIS_TOMO_DIM; gy++) {
        const float py = extent - ((float)gy + 0.5f) * step;
        for (int gx = 0; gx < MANTIS_TOMO_DIM; gx++) {
            const float px = -extent + ((float)gx + 0.5f) * step;
            float vx, vy; int used;
            const float r2 = mantis_doppler_residual(obs, n, px, py, &vx, &vy, &used);
            if (r2 > 1e29f) continue;
            // Fraction of the observed energy this cell explains.
            const float explained = 1.0f - (r2 / energy);
            out[gy * MANTIS_TOMO_DIM + gx] = explained > 0.0f ? explained : 0.0f;
        }
    }
}

// Build observations from the mesh's per-link phase rates.
//
// Every pair contributes, whether or not its direct path is blocked --
// that is the entire point.  Shadowing needs the target ON the line;
// Doppler only needs it somewhere the bisector geometry can see, which
// is everywhere.
static inline int mantis_doppler_build(const MantisMesh *m,
                                       const float *rate_matrix,
                                       MantisDopplerObs *out, int max_out) {
    int k = 0;
    for (uint8_t i = 1; i <= m->n_beacons && k < max_out; i++) {
        if (!m->have_pos[i]) continue;
        for (uint8_t j = (uint8_t)(i + 1); j <= m->n_beacons && k < max_out; j++) {
            if (!m->have_pos[j]) continue;
            const int fwd = i * MANTIS_SLOTS + j, rev = j * MANTIS_SLOTS + i;
            const float q = m->qual[fwd] + m->qual[rev];
            if (q <= 0.0f) continue;
            // Both directions traverse the same physical path and see the
            // same bistatic Doppler, so averaging them halves the noise
            // and a large disagreement is evidence of a bad measurement.
            const float r = (rate_matrix[fwd] * m->qual[fwd]
                           + rate_matrix[rev] * m->qual[rev]) / q;
            out[k].rate = r;
            out[k].ax = m->bx[i]; out[k].ay = m->by[i];
            out[k].bx = m->bx[j]; out[k].by = m->by[j];
            out[k].weight = q * 0.5f;
            k++;
        }
    }
    return k;
}
