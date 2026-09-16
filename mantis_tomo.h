#pragma once
// ═══════════════════════════════════════════════════════════════
//  MANTIS CHORD TOMOGRAPHY
//  Geometry, not fingerprinting
// ═══════════════════════════════════════════════════════════════
//
//  A blockage measurement on a link is a LINE INTEGRAL through the room.
//  That is a Radon-transform sample, and whether you can reconstruct
//  from a set of them depends entirely on how those lines cover the
//  (angle, offset) plane.
//
//  Measured on a 6-beacon ring:
//
//      spokes only (beacon -> receiver)
//          6 lines, 3 angles, 1 OFFSET  -- every line through the origin
//      chords (beacon -> beacon)
//          15 lines, 6 angles, 5 offsets
//
//  The spoke set is a fan from a single vertex.  It can say "something
//  is on this bearing" and can NEVER say where along it; the offset
//  information simply is not present in the measurement.  Everything the
//  old system did to get position out of it -- the learned kernel, the
//  birth search, the response manifold -- was compensating for that
//  missing diversity with prior knowledge.
//
//  The chords supply the missing axis directly.  A target is where the
//  attenuated chords cross, which is arithmetic rather than inference.
//
//  WHAT THIS BUYS THAT THE MANIFOLD CANNOT
//
//    - It needs NO calibration.  The instant the beacons are placed and
//      ranged, it localises.  No walk, no kernel, no baseline beyond a
//      few quiet frames.
//    - It cannot be fooled by an un-walked region.  Geometry does not
//      have coverage holes; a chord either passes through a cell or it
//      does not.
//    - It degrades predictably.  Lose a beacon and you lose exactly the
//      chords through it, and you can SEE which cells got weaker.
//
//  WHAT IT CANNOT DO
//
//    21 lines is a very sparse sinogram.  This produces a coarse
//    occupancy field with pronounced streak artefacts along the chords,
//    not a picture.  It is a strong PRIOR for the learned model to
//    refine, and a sanity check on it -- not a replacement.  Used alone
//    it would over-claim, which is the failure mode this whole project
//    keeps having to correct.
// ═══════════════════════════════════════════════════════════════
#include "mantis_air.h"
#include <math.h>

#ifndef MANTIS_TOMO_DIM
  #define MANTIS_TOMO_DIM 24          // matches the existing field grid
#endif
#define MANTIS_TOMO_CELLS (MANTIS_TOMO_DIM * MANTIS_TOMO_DIM)

// One measured line integral.
typedef struct {
    float ax, ay;      // endpoint A, chart coordinates
    float bx, by;      // endpoint B
    float atten;       // perturbation along this path, 0 = clear
    float weight;      // confidence: link quality x geometry trust
} MantisChord;

// Half-width of a chord's influence, in chart units.  A real path is not
// a mathematical line: Fresnel-zone width at 2.4 GHz over a few metres
// is tens of centimetres, so a chord genuinely blurs. Modelling it as a
// hairline would produce single-cell streaks that look precise and are
// not.
#ifndef MANTIS_CHORD_HALFWIDTH
  #define MANTIS_CHORD_HALFWIDTH 0.13f
#endif

// Attenuation a human body causes on a blocked chord, and the noise on
// that measurement.  These are the only physical priors the tomography
// needs -- everything else is geometry.  Both are measurable from the
// calibration walk rather than guessed, and the ratio A/s is what sets
// how decisively a single chord can vote.
#ifndef MANTIS_TOMO_TARGET_ATTEN
  #define MANTIS_TOMO_TARGET_ATTEN 0.35f
#endif
#ifndef MANTIS_TOMO_NOISE
  #define MANTIS_TOMO_NOISE 0.06f
#endif

// Perpendicular distance from a point to a SEGMENT (not an infinite
// line): a target beyond a beacon is not on that chord, and treating it
// as though it were would smear energy outside the ring.
static inline float mantis_pt_seg_d2(float px, float py,
                                     float ax, float ay, float bx, float by) {
    const float dx = bx - ax, dy = by - ay;
    const float L  = dx * dx + dy * dy;
    float t = (L <= 1e-9f) ? 0.0f : ((px - ax) * dx + (py - ay) * dy) / L;
    t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
    const float ex = px - (ax + t * dx), ey = py - (ay + t * dy);
    return ex * ex + ey * ey;
}

// Back-project a set of chords onto an occupancy field.
//
// Plain back-projection, deliberately: filtered BP needs a dense,
// regularly-sampled sinogram and ours is 21 irregular lines.  The filter
// would amplify noise far more than it would sharpen anything.
//
// `extent` is the half-width of the field in chart units.
// `out` is MANTIS_TOMO_CELLS, row-major, y descending.
// Reconstruct an occupancy field from chord attenuations.
//
// This took three attempts and the first two are worth recording,
// because each failed for a reason that looked like a bug and was
// actually the wrong estimator.
//
//   1. MEAN back-projection.  Normalising by coverage made a cell
//      crossed by one blocked chord score identically to a cell crossed
//      by four.  The field filled with ties; the peak sat 0.4 units from
//      truth with a margin of exactly 1.00 everywhere.
//
//   2. MINIMUM attenuation.  Correct in spirit -- one clear line of
//      sight really does prove a cell empty -- but the minimum over N
//      noisy samples is biased low, and the CENTRE has the most chords
//      crossing it.  So the centre always scored lowest and the peak
//      fled to the sparsely-covered edge.  Mean error got worse, 1.03.
//
//   3. LOG-ODDS SUM.  Each chord contributes evidence, so more chords
//      mean more evidence rather than more chances to draw a low sample.
//      The empty-cell veto emerges naturally: a chord measuring no
//      attenuation contributes a strongly NEGATIVE term, which is
//      exactly "this line of sight is clear, so nothing is on it".
//
// For a chord of attenuation a, with A the attenuation a target would
// cause and s the measurement noise, the Gaussian log-likelihood ratio
// between "occupied" and "empty" is
//
//      llr = (a * A - A * A / 2) / (s * s)
//
// positive when a is near A, negative when a is near zero.  Summing is
// then just independent evidence combining, and the bias is gone.
static inline void mantis_tomo_backproject(const MantisChord *ch, int n,
                                           float extent, float *out) {
    for (int i = 0; i < MANTIS_TOMO_CELLS; i++) out[i] = 0.0f;
    if (n <= 0 || extent <= 0.0f) return;

    const float step = (2.0f * extent) / (float)MANTIS_TOMO_DIM;
    const float hw2  = MANTIS_CHORD_HALFWIDTH * MANTIS_CHORD_HALFWIDTH;
    const float A    = MANTIS_TOMO_TARGET_ATTEN;
    const float s2   = MANTIS_TOMO_NOISE * MANTIS_TOMO_NOISE;

    for (int gy = 0; gy < MANTIS_TOMO_DIM; gy++) {
        const float py = extent - ((float)gy + 0.5f) * step;
        for (int gx = 0; gx < MANTIS_TOMO_DIM; gx++) {
            const float px  = -extent + ((float)gx + 0.5f) * step;
            const int   idx = gy * MANTIS_TOMO_DIM + gx;

            float llr = 0.0f; int hits = 0;
            for (int c = 0; c < n; c++) {
                const MantisChord &k = ch[c];
                if (k.weight <= 0.0f) continue;
                const float d2 = mantis_pt_seg_d2(px, py, k.ax, k.ay, k.bx, k.by);
                if (d2 > hw2) continue;
                // A chord clipping the cell edge is weaker evidence than
                // one through its centre, in BOTH directions.
                const float prox = 1.0f - d2 / hw2;
                hits++;
                llr += k.weight * prox * (k.atten * A - 0.5f * A * A) / s2;
            }
            // Fewer than two independent looks is not a reconstruction.
            out[idx] = (hits >= 2) ? llr : 0.0f;
        }
    }
}

// How many chords look through each cell.  A caller that wants to know
// whether a low score means "clear" or "never measured" needs this; they
// are not the same claim.
static inline int mantis_tomo_coverage(const MantisChord *ch, int n,
                                       float extent, float px, float py) {
    const float hw2 = MANTIS_CHORD_HALFWIDTH * MANTIS_CHORD_HALFWIDTH;
    (void)extent;
    int hits = 0;
    for (int c = 0; c < n; c++) {
        if (ch[c].weight <= 0.0f) continue;
        if (mantis_pt_seg_d2(px, py, ch[c].ax, ch[c].ay, ch[c].bx, ch[c].by) <= hw2)
            hits++;
    }
    return hits;
}

// Where is the field strongest, and is that peak trustworthy?
//
// `margin` is peak over runner-up among cells that are not adjacent to
// the peak.  With a sparse sinogram, streaks along a single chord are
// the dominant artefact, and they produce a ridge rather than a point --
// so a low margin means "a chord is blocked somewhere" and not "a target
// is here".  The caller must respect that distinction.
static inline bool mantis_tomo_peak(const float *field, float extent,
                                    float *out_x, float *out_y,
                                    float *out_val, float *out_margin) {
    int best = -1; float bv = -1e30f;
    for (int i = 0; i < MANTIS_TOMO_CELLS; i++)
        if (field[i] > bv) { bv = field[i]; best = i; }
    if (best < 0 || bv <= 0.0f) return false;

    const int bx = best % MANTIS_TOMO_DIM, by = best / MANTIS_TOMO_DIM;
    float second = 0.0f;
    for (int i = 0; i < MANTIS_TOMO_CELLS; i++) {
        const int x = i % MANTIS_TOMO_DIM, y = i / MANTIS_TOMO_DIM;
        const int dx = x - bx, dy = y - by;
        if (dx * dx + dy * dy <= 9) continue;      // skip the peak's own lobe
        if (field[i] > second) second = field[i];
    }
    const float step = (2.0f * extent) / (float)MANTIS_TOMO_DIM;
    if (out_x)      *out_x = -extent + ((float)bx + 0.5f) * step;
    if (out_y)      *out_y =  extent - ((float)by + 0.5f) * step;
    if (out_val)    *out_val = bv;
    if (out_margin) *out_margin = (second > 1e-6f) ? (bv / second) : 99.0f;
    return true;
}

// Build the chord set from the mesh's reports.
//
// Only links whose BOTH endpoints have measured positions are used.
// A chord with a guessed endpoint is not a measurement, and mixing one
// in would corrupt the whole reconstruction while looking like extra
// evidence.
static inline int mantis_tomo_build(const float *bx, const float *by,
                                    const uint8_t *have_pos, uint8_t n_beacons,
                                    const float *atten_matrix,   // n x n, row=hearer
                                    const float *qual_matrix,
                                    MantisChord *out, int max_out) {
    int m = 0;
    for (uint8_t i = 0; i < n_beacons && m < max_out; i++) {
        if (!have_pos[i]) continue;
        for (uint8_t j = (uint8_t)(i + 1); j < n_beacons && m < max_out; j++) {
            if (!have_pos[j]) continue;
            // Both directions measured the same physical path; average
            // them.  Reciprocity is a real property here, so a large
            // disagreement is evidence of a bad measurement, not of
            // asymmetric geometry.
            const float a1 = atten_matrix[i * n_beacons + j];
            const float a2 = atten_matrix[j * n_beacons + i];
            const float q1 = qual_matrix[i * n_beacons + j];
            const float q2 = qual_matrix[j * n_beacons + i];
            const float qs = q1 + q2;
            if (qs <= 0.0f) continue;
            const float a  = (a1 * q1 + a2 * q2) / qs;
            // Reciprocity check -- but ONLY when both directions were
            // actually measured.
            //
            // With one direction unmeasured its attenuation reads as
            // zero, so |a1 - a2| equals a1 and a perfectly good one-way
            // measurement gets penalised in proportion to how much
            // signal it found.  The stronger the evidence, the harder it
            // was punished, which is exactly backwards.
            const bool both = (q1 > 0.0f) && (q2 > 0.0f);
            const float trust = both
                ? (1.0f / (1.0f + 4.0f * fabsf(a1 - a2)))
                : 0.75f;   // one-way: usable, but no cross-check to earn full trust
            out[m].ax = bx[i]; out[m].ay = by[i];
            out[m].bx = bx[j]; out[m].by = by[j];
            out[m].atten  = a;
            out[m].weight = (qs * 0.5f) * trust;
            m++;
        }
    }
    return m;
}
