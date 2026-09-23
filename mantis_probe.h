#pragma once
// ═══════════════════════════════════════════════════════════════
//  MANTIS PROBE — the operator as a known, cooperative target
// ═══════════════════════════════════════════════════════════════
//
//  The probe is in the user's hand.  So at every instant during
//  calibration and during undocked operation, the system knows where a
//  human body is.  That is ground truth, continuously, for free, and it
//  is the single largest asset in the whole deployment -- it was sitting
//  unused because we were treating the operator as a nuisance to be
//  waited out rather than as an instrument.
//
//  Four things fall out of it.
//
//  ── 1. SELF-SUPERVISION ──────────────────────────────────────
//  Every calibration frame is a LABELLED sample.  A 90-second walk at
//  30 Hz across 15 chords is 40,500 labelled link observations.  A
//  conventional system has to ASSUME the room was empty for the baseline
//  and occupied for the walk; we know it, frame by frame.
//
//  Concretely, this replaces a guessed constant with a measurement.
//  MANTIS_TOMO_TARGET_ATTEN was 0.35 because that is a plausible number
//  for a human body.  With the probe we measure the actual attenuation
//  THIS operator causes on THESE links in THIS room -- and that constant
//  sets how decisively a single chord may vote.
//
//  ── 2. LIVE VALIDATION, INDEFINITELY ─────────────────────────
//  Undocked, the operator is a target at a known position.  So the
//  inference can be scored against truth on every frame, with no test
//  set and no operator effort: does the mesh put a target where the
//  probe actually is?  When it stops doing so, the model is wrong RIGHT
//  NOW and can say so instead of degrading silently.
//
//  ── 3. SELF-CANCELLATION ─────────────────────────────────────
//  "Is someone ELSE here?" is a different question from "is anyone
//  here?", and the operator is always the loudest target in their own
//  data.  Knowing where you are lets you predict your own contribution
//  and subtract it.  What remains is everyone else.
//
//  ── 4. SYNTHETIC APERTURE ────────────────────────────────────
//  The probe is a radio.  As the operator walks, probe-to-beacon links
//  sweep geometries a fixed array cannot reach:
//
//      fixed beacons only   59% of the room sensed
//      + a 24-step walk     96%
//
//  The walk is not merely sampling responses.  It is forming an aperture.
// ═══════════════════════════════════════════════════════════════
#include "mantis_mesh.h"
#include <stdint.h>
#include <math.h>

typedef struct {
    float   x, y;          // where the probe is, in chart units
    bool    known;         // do we actually believe that?
    float   sigma;         // position uncertainty, chart units

    // Measured body attenuation, learned from the probe itself.
    float   body_atten;    // replaces the guessed MANTIS_TOMO_TARGET_ATTEN
    float   body_width;    // effective blocking half-width, measured
    uint32_t learn_n;

    // Live validation, from the mesh's own estimate of the operator.
    float   val_err_ema;   // running mean position error, chart units
    uint32_t val_n, val_hits, val_misses;
} MantisProbe;

// How close the mesh's estimate must land to count as finding the
// operator.  Set to the shadowing channel's demonstrated accuracy plus
// a cell: closer would fail on quantisation alone.
#ifndef MANTIS_PROBE_HIT_RADIUS
  #define MANTIS_PROBE_HIT_RADIUS 0.22f
#endif

// Predicted attenuation on one chord from a body at (px,py).
//
// Deliberately the SAME profile the reconstruction assumes, so that
// cancellation and detection cannot disagree about what a body looks
// like.  Two different body models would leave a residual shaped like
// the difference between them, and that residual would be reported as
// another person.
static inline float mantis_probe_chord_atten(const MantisProbe *p,
                                             float ax, float ay,
                                             float bx, float by) {
    if (!p->known) return 0.0f;
    const float w  = (p->learn_n > 0) ? p->body_width : MANTIS_CHORD_HALFWIDTH;
    const float a  = (p->learn_n > 0) ? p->body_atten : MANTIS_TOMO_TARGET_ATTEN;
    const float d2 = mantis_pt_seg_d2(p->x, p->y, ax, ay, bx, by);
    if (d2 >= w * w) return 0.0f;
    const float d = sqrtf(d2);
    return a * (1.0f - d / w);
}

// LEARN the body profile from the probe.
//
// Called while the operator is walking and the room is otherwise empty,
// which is exactly the calibration condition.  For each chord the
// operator is currently blocking, the measured attenuation IS the body's
// attenuation at that geometry.
static inline void mantis_probe_learn(MantisProbe *p, const MantisMesh *m) {
    if (!p->known || m->n_beacons < 2) return;
    const float w = MANTIS_CHORD_HALFWIDTH;

    for (uint8_t i = 1; i <= m->n_beacons; i++) {
        if (!m->have_pos[i]) continue;
        for (uint8_t j = 1; j <= m->n_beacons; j++) {
            if (i == j || !m->have_pos[j]) continue;
            const float d2 = mantis_pt_seg_d2(p->x, p->y, m->bx[i], m->by[i],
                                              m->bx[j], m->by[j]);
            // Only chords the operator is squarely on.  A glancing block
            // measures the edge of the body, where the profile is
            // steepest and the position error hurts most.
            if (d2 > (0.35f * w) * (0.35f * w)) continue;
            const float meas = m->atten[i * MANTIS_SLOTS + j];
            if (meas <= 0.0f) continue;

            // INVERT THE PROFILE.  body_atten is the PEAK attenuation --
            // what a chord through the centre of the body sees -- but a
            // measurement taken at offset d saw only a*(1 - d/w).
            //
            // Averaging the raw measurements estimates the MEAN over the
            // sampled region instead, which is systematically low: with
            // a true peak of 0.38 it learned 0.301.  Cancellation then
            // under-subtracted by 20%, leaving operator residue that
            // competed with the actual intruder and dropped its score
            // below the detection floor.  The operator was cancelled
            // just enough to look like a second person.
            const float d = sqrtf(d2);
            const float profile = 1.0f - d / w;
            if (profile < 0.5f) continue;          // too little leverage to invert
            const float peak = meas / profile;

            const uint32_t n = p->learn_n;
            p->body_atten = (n == 0) ? peak
                                     : (p->body_atten + (peak - p->body_atten) / (float)(n + 1));
            if (p->learn_n < 100000) p->learn_n++;
        }
    }
    // Width is not learned separately yet; the Fresnel half-width is a
    // physical quantity rather than a per-operator one, and pretending
    // to measure it from a handful of glancing chords would be worse
    // than using the value physics gives.
    p->body_width = w;
}

// CANCEL the operator from the attenuation matrix.
//
// `out_atten` receives the residual: what the mesh sees MINUS what the
// operator explains.  The mesh's own matrix is left untouched, because
// the operator is a real target and a view that hides them is the wrong
// default for anything except intruder search.
//
// Residuals are clamped at zero.  Over-subtraction produces negative
// attenuation, which is not evidence of anything and would otherwise
// read as "unusually clear" -- a phantom hole in the reconstruction
// exactly where the operator stands.
static inline void mantis_probe_cancel(const MantisProbe *p, const MantisMesh *m,
                                       float *out_atten) {
    for (int i = 0; i < MANTIS_SLOTS * MANTIS_SLOTS; i++) out_atten[i] = m->atten[i];
    if (!p->known) return;

    for (uint8_t i = 1; i <= m->n_beacons; i++) {
        if (!m->have_pos[i]) continue;
        for (uint8_t j = 1; j <= m->n_beacons; j++) {
            if (i == j || !m->have_pos[j]) continue;
            const int idx = i * MANTIS_SLOTS + j;
            const float pred = mantis_probe_chord_atten(p, m->bx[i], m->by[i],
                                                        m->bx[j], m->by[j]);
            const float r = out_atten[idx] - pred;
            out_atten[idx] = (r > 0.0f) ? r : 0.0f;
        }
    }
}

// Score the mesh's estimate against where the probe actually is.
//
// This is a free, permanent regression test running on live data.  A
// rising error means the model has drifted, the room has changed, or a
// beacon has moved -- and it is detectable long before anyone notices
// the tracking is wrong.
static inline void mantis_probe_validate(MantisProbe *p, bool mesh_found,
                                         float mx, float my) {
    if (!p->known) return;
    p->val_n++;
    if (!mesh_found) { p->val_misses++; return; }
    const float e = sqrtf((mx - p->x) * (mx - p->x) + (my - p->y) * (my - p->y));
    if (e <= MANTIS_PROBE_HIT_RADIUS) p->val_hits++; else p->val_misses++;
    p->val_err_ema = (p->val_n == 1) ? e : (p->val_err_ema + 0.05f * (e - p->val_err_ema));
}

// Fraction of frames where the mesh found the operator where they were.
// Returns -1 when there is not yet enough evidence to say, which is a
// different answer from "badly" and must not be rendered as 0%.
static inline float mantis_probe_confidence(const MantisProbe *p) {
    if (p->val_n < 30) return -1.0f;
    return (float)p->val_hits / (float)p->val_n;
}

// Chords the walk has contributed, for the coverage readout.
//
// A probe at (px,py) forms a chord to every beacon it can hear.  These
// are transient -- they exist only while the operator stands there --
// but during a cal walk they accumulate into the aperture that takes
// coverage from 59% to 96%.
static inline int mantis_probe_chords(const MantisProbe *p, const MantisMesh *m,
                                      MantisChord *out, int max_out) {
    if (!p->known) return 0;
    int k = 0;
    for (uint8_t i = 1; i <= m->n_beacons && k < max_out; i++) {
        if (!m->have_pos[i]) continue;
        out[k].ax = p->x;      out[k].ay = p->y;
        out[k].bx = m->bx[i];  out[k].by = m->by[i];
        out[k].atten = 0.0f;   // filled by the caller from measurement
        out[k].weight = 1.0f;
        k++;
    }
    return k;
}
